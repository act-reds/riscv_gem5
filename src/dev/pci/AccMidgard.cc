/* @file
 * Non-Volatile Dual In-line Memory Module Virtualization Implementation
 */

#include "dev/pci/AccMidgard.hh"

#include <algorithm>
#include <semaphore.h>
#include <assert.h>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "debug/DMACopyEngine.hh"
#include "debug/AccMidgardDebug.hh"
#include "debug/Drain.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AccMidgard.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace copy_engine_reg;

static void data_init(std::vector<std::vector<int>> &A, std::vector<std::vector<int>> &B, uint64_t size)
{
    srand(888);
    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < size; j++)
        {
            A[i][j] = rand() % 256;
        }
    }
    for (size_t i = 0; i < size; i++)
    {
        for (size_t j = 0; j < size; j++)
        {
            B[i][j] = rand() % 256;
        }
    }
}

AccMidgard::AccMidgard(const AccMidgardParams &p)
    : PciDevice(p),
      DMAStats(this, p.ChanCnt),
      sramPort(p.name + ".sram_port", this, true),
      dramPort(p.name + ".dram_port", this, false)
{
    // All Reg regs are initialized to 0 by default
    regs.chanCount = p.ChanCnt;
    regs.xferCap = findMsbSet(p.XferCap);
    regs.attnStatus = 0;

    warn("CXL initialization!!\n");
    if (regs.chanCount > 64)
        fatal("AccMidgard interface doesn't support more than 64 DMA engines\n");

    for (int x = 0; x < regs.chanCount; x++) {
        DMAChannel *ch = new DMAChannel(this, x);
        chan.push_back(ch);
    }


    int READ_latency_Matrix = 0;
    u_int64_t time_stamp = 0;
    int row_index_ = 0;
    RTL_Wrapper = new AccController(p.BufferBlock_Num, p.BufferBlock_Size,
                                    READ_latency_Matrix,
                                    &chan);

    warn("call child functions from here");
}


AccMidgard::DMAChannel::DMAChannel(AccMidgard *_ce, int cid)
    : ce(_ce), channelId(cid), busy(false), underReset(false),
      refreshNext(false), latBeforeBegin(ce->params().latBeforeBegin),
      latAfterCompletion(ce->params().latAfterCompletion),
      completionDataReg(0), nextState(Idle),
      fetchCompleteEvent([this]{ fetchDescComplete(); }, name()),
      addrCompleteEvent([this]{ fetchAddrComplete(); }, name()),
      AccelCompleteEvent([this]{callAccelComplete();}, name()),
      readCompleteEvent([this]{ readCopyBytesComplete(); }, name()),
      writeCompleteEvent([this]{ writeCopyBytesComplete(); }, name()),
      statusCompleteEvent([this]{ writeStatusComplete(); }, name()),
      resetIntrEvent([this]{ resetIntr(); }, name())
    //   cacheFlushEvent([this]{ cacheFlushComplete(); }, name()),
    //   cacheInvalidateEvent([this]{ cacheInvalidateComplete(); }, name())
{
        cr.status.dma_transfer_status(3);
        cr.descChainAddr = 0;
        cr.completionAddr = 0;

        buffer_offset = new uint16_t(0);
        curDmaDesc = new DmaDesc;
        memset(curDmaDesc, 0, sizeof(DmaDesc));
        copyBuffer = new uint8_t[ce->params().XferCap];
}

AccMidgard::~AccMidgard()
{
    for (int x = 0; x < chan.size(); x++) {
        delete chan[x];
    }
}

AccMidgard::DMAChannel::~DMAChannel()
{
    delete curDmaDesc;
    delete [] copyBuffer;
}

Port &
AccMidgard::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "dma") {
        if (idx >= static_cast<int>(chan.size())) {
            panic("AccMidgard::getPort: unknown index %d\n", idx);
        }
        return DmaDevice::getPort(if_name,idx);
    } else if (if_name == "sram_port") {
        return sramPort;
    } else if (if_name == "dram_port") {
        return dramPort;
    } else {
        return PciDevice::getPort(if_name, idx);
    }
}

void
AccMidgard::DMAChannel::recvCommand()
{
    if (cr.command.start_dma()) {
        DPRINTF(DMACopyEngine, "receive command:start_dma\n");
        assert(!busy);
        cr.status.dma_transfer_status(0);
        nextState = DescriptorFetch;
        fetchAddress = cr.descChainAddr;
        if (ce->drainState() == DrainState::Running)
            fetchDescriptor(cr.descChainAddr);
    } else if (cr.command.append_dma()) {
        DPRINTF(DMACopyEngine, "receive command:append_dma\n");
        if (!busy) {
            nextState = AddressFetch;
            if (ce->drainState() == DrainState::Running)
                fetchNextAddr(lastDescriptorAddr);
        } else
        {
            DPRINTF(DMACopyEngine, "system busy, schedule to next state\n");
            refreshNext = true;
        }
    } else if (cr.command.reset_dma()) {
        DPRINTF(DMACopyEngine, "receive command:reset_dma\n");
        if (busy){
            DPRINTF(DMACopyEngine, "Warning: Reset Command Set during busy!!!\n");
            underReset = true;
        }
        else {
            cr.status.dma_transfer_status(3);
            nextState = Idle;
        }
    } else if (cr.command.resume_dma() || cr.command.abort_dma() ||
            cr.command.suspend_dma())
        panic("Resume, Abort, and Suspend are not supported\n");
    cr.command(0);
}

Tick
AccMidgard::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;
    warn("AccMidgard Read function called!!\n");
    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    int size = pkt->getSize();
    if (size != sizeof(uint64_t) && size != sizeof(uint32_t) &&
        size != sizeof(uint16_t) && size != sizeof(uint8_t)) {
        panic("Unknown size for MMIO access: %d\n", pkt->getSize());
    }

    DPRINTF(DMACopyEngine, "Read device register %#X size: %d\n", daddr, size);

    ///
    /// Handle read of register here
    ///

    if (daddr < 0x80) {
        switch (daddr) {
          case GEN_CHANCOUNT:
            assert(size == sizeof(regs.chanCount));
            pkt->setLE<uint8_t>(regs.chanCount);
            break;
          case GEN_XFERCAP:
            assert(size == sizeof(regs.xferCap));
            pkt->setLE<uint8_t>(regs.xferCap);
            break;
          case GEN_INTRCTRL:
            assert(size == sizeof(uint8_t));
            pkt->setLE<uint8_t>(regs.intrctrl());
            regs.intrctrl.master_int_enable(0);
            break;
          case GEN_ATTNSTATUS:
            assert(size == sizeof(regs.attnStatus));
            pkt->setLE<uint32_t>(regs.attnStatus);
            regs.attnStatus = 0;
            break;
        //   case RESEVER:
        //     break;
          default:
            panic("Read request to unknown register number: %#x\n", daddr);
        }
        pkt->makeAtomicResponse();
        return pioDelay;
    }


    // Find which channel we're accessing
    int chanid = 0;
    daddr -= 0x80;
    while (daddr >= 0x80) {
        chanid++;
        daddr -= 0x80;
    }

    if (chanid >= regs.chanCount)
        panic("Access to channel %d (device only configured for %d channels)",
                chanid, regs.chanCount);

    ///
    /// Channel registers are handled here
    ///
    chan[chanid]->channelRead(pkt, daddr, size);

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
AccMidgard::DMAChannel::channelRead(Packet *pkt, Addr daddr, int size)
{
    switch (daddr) {
      case CHAN_CONTROL:
        assert(size == sizeof(uint16_t));
        pkt->setLE<uint16_t>(cr.ctrl());
        cr.ctrl.in_use(1);
        break;
      case CHAN_STATUS:
        assert(size == sizeof(uint64_t));
        pkt->setLE<uint64_t>(cr.status() | (busy ? 0 : 1));
        break;
      case CHAN_CHAINADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            pkt->setLE<uint64_t>(cr.descChainAddr);
        else
            pkt->setLE<uint32_t>(bits(cr.descChainAddr,0,31));
        break;
      case CHAN_CHAINADDR_HIGH:
        assert(size == sizeof(uint32_t));
        pkt->setLE<uint32_t>(bits(cr.descChainAddr,32,63));
        break;
      case CHAN_COMMAND:
        assert(size == sizeof(uint8_t));
        pkt->setLE<uint32_t>(cr.command());
        break;
      case CHAN_CMPLNADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            pkt->setLE<uint64_t>(cr.completionAddr);
        else
            pkt->setLE<uint32_t>(bits(cr.completionAddr,0,31));
        break;
      case CHAN_CMPLNADDR_HIGH:
        assert(size == sizeof(uint32_t));
        pkt->setLE<uint32_t>(bits(cr.completionAddr,32,63));
        break;
      case CHAN_ERROR:
        assert(size == sizeof(uint32_t));
        pkt->setLE<uint32_t>(cr.error());
        break;
      default:
        panic("Read request to unknown channel register number: (%d)%#x\n",
                channelId, daddr);
    }
}


Tick
AccMidgard::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    warn("AccMidgard::write start!!!\n");

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    int size = pkt->getSize();

    ///
    /// Handle write of register here
    ///

    if (size == sizeof(uint64_t)) {
        [[maybe_unused]] uint64_t val = pkt->getLE<uint64_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n",
                daddr, val);
    } else if (size == sizeof(uint32_t)) {
        [[maybe_unused]] uint32_t val = pkt->getLE<uint32_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n",
                daddr, val);
    } else if (size == sizeof(uint16_t)) {
        [[maybe_unused]] uint16_t val = pkt->getLE<uint16_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n",
                daddr, val);
    } else if (size == sizeof(uint8_t)) {
        [[maybe_unused]] uint8_t val = pkt->getLE<uint8_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n",
                daddr, val);
    } else {
        panic("Unknown size for MMIO access: %d\n", size);
    }

    if (daddr < 0x80) {
        switch (daddr) {
          case GEN_CHANCOUNT:
          case GEN_XFERCAP:
          case GEN_ATTNSTATUS:
            DPRINTF(DMACopyEngine, "Warning, ignorning write to register %x\n",
                    daddr);
            break;
          case GEN_INTRCTRL:
            DPRINTF(DMACopyEngine, "regs.intrctrl.master_int_enable\n");
            regs.intrctrl.master_int_enable(bits(pkt->getLE<uint8_t>(), 0, 0));
            break;
          default:
            panic("Read request to unknown register number: %#x\n", daddr);
        }
        pkt->makeAtomicResponse();
        return pioDelay;
    }

    // Find which channel we're accessing
    int chanid = 0;
    daddr -= 0x80;
    while (daddr >= 0x80) {
        chanid++;
        daddr -= 0x80;
    }

    if (chanid >= regs.chanCount)
        panic("Access to channel %d (device only configured for %d channels)",
                chanid, regs.chanCount);

    ///
    /// Channel registers are handled here
    ///
    chan[chanid]->channelWrite(pkt, daddr, size);

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
AccMidgard::DMAChannel::channelWrite(Packet *pkt, Addr daddr, int size)
{
    switch (daddr) {
      case CHAN_CONTROL:
        assert(size == sizeof(uint16_t));
        int old_int_disable;
        old_int_disable = cr.ctrl.interrupt_disable();
        cr.ctrl(pkt->getLE<uint16_t>());
        if (cr.ctrl.interrupt_disable())
            cr.ctrl.interrupt_disable(0);
        else
            cr.ctrl.interrupt_disable(old_int_disable);
        break;
      case CHAN_STATUS:
        assert(size == sizeof(uint64_t));
        DPRINTF(DMACopyEngine, "Warning, ignorning write to register %x\n",
                    daddr);
        break;
      case CHAN_CHAINADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            cr.descChainAddr = pkt->getLE<uint64_t>();
        else
            cr.descChainAddr =  (uint64_t)pkt->getLE<uint32_t>() |
                (cr.descChainAddr & ~mask(32));
        DPRINTF(DMACopyEngine, "Chain Address %x\n", cr.descChainAddr);
        break;
      case CHAN_CHAINADDR_HIGH:
        assert(size == sizeof(uint32_t));
        cr.descChainAddr =  ((uint64_t)pkt->getLE<uint32_t>() << 32) |
            (cr.descChainAddr & mask(32));
        DPRINTF(DMACopyEngine, "Chain Address %x\n", cr.descChainAddr);
        break;
      case CHAN_COMMAND:
        assert(size == sizeof(uint8_t));
        cr.command(pkt->getLE<uint8_t>());
        recvCommand();
        break;
      case CHAN_CMPLNADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            cr.completionAddr = pkt->getLE<uint64_t>();
        else
            cr.completionAddr =  pkt->getLE<uint32_t>() |
                (cr.completionAddr & ~mask(32));
        break;
      case CHAN_CMPLNADDR_HIGH:
        assert(size == sizeof(uint32_t));
        cr.completionAddr =  ((uint64_t)pkt->getLE<uint32_t>() <<32) |
            (cr.completionAddr & mask(32));
        break;
      case CHAN_ERROR:
        assert(size == sizeof(uint32_t));
        cr.error(~pkt->getLE<uint32_t>() & cr.error());
        break;
      default:
        panic("Read request to unknown channel register number: (%d)%#x\n",
                channelId, daddr);
    }
}

AccMidgard::
DMAStats::DMAStats(statistics::Group *parent,
                                 const uint8_t &channel_count)
    : statistics::Group(parent, "AccMidgard"),
      ADD_STAT(bytesCopied, statistics::units::Byte::get(),
               "Number of bytes copied by each engine"),
      ADD_STAT(copiesProcessed, statistics::units::Count::get(),
               "Number of copies processed by each engine")
{
    bytesCopied
        .init(channel_count)
        .flags(statistics::total)
        ;
    copiesProcessed
        .init(channel_count)
        .flags(statistics::total)
        ;
}

// void
// AccMidgard::DMAChannel::cacheFlush()
// {
//     PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(curDmaDesc->src), curDmaDesc->len,
//                                                     (uint8_t*)curDmaDesc, 0, MemCmd::FlushReqDMA,
//                                                     0, &cacheFlushEvent,0);
//     DPRINTF(DMACopyEngine, "%s:%s\n", __func__, Ptrpkt->print());

//     // Addr addr = ce->pciToDma(curDmaDesc->src);
//     // uint32_t length = curDmaDesc->len;
//     // uint32_t cacheLineSize = ce->dmaPort.sys->cacheLineSize();
//     // // Split the length into 64B, and invalidate the cache line one by one
//     // for (Addr addr_= addr ; addr_ < addr + length; addr_+=cacheLineSize)
//     // {
//     //     uint32_t size = std::min<uint32_t>(cacheLineSize, addr + length - addr_);
//     //     PacketPtr Ptrpkt_atomic = ce->RTL_Wrapper->createPacket(addr_, size,
//     //                                                 nullptr, 0, MemCmd::FlushReq, 0, &cacheFlushEvent,0);
//     //     flushQueue.emplace(Ptrpkt_atomic);
//     // }
//     flushQueue.emplace(Ptrpkt);
//     cacheFlushComplete();
// }

// void
// AccMidgard::DMAChannel::cacheFlushComplete()
// {
//     if (!flushQueue.empty())
//     {
//         PacketPtr Ptrpkt = flushQueue.front();
//         DPRINTF(DMACopyEngine, "Flush Cache with Ptr:%s\n", Ptrpkt->print());
//         ce->dramPort.sendPacket(Ptrpkt, true);
//         ce->dramPort.tick();
//         flushQueue.pop();
//         nextState = CacheFlush;
//     }
//     else
//     {
//         DPRINTF(DMACopyEngine, "cacheFlushComplete\n");
//         nextState = DMARead;
//         if (inDrain()) return;
//         readCopyBytes();
//     }
// }

// void
// AccMidgard::DMAChannel::cacheInvalidate()
// {
//     PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(curDmaDesc->dest), curDmaDesc->len,
//                                                     (uint8_t*)curDmaDesc, 0, MemCmd::InvalidateReqDMA,
//                                                     0, &cacheInvalidateEvent,0);
//     DPRINTF(DMACopyEngine, "%s:%s\n", __func__, Ptrpkt->print());
//     // Addr addr = ce->pciToDma(curDmaDesc->dest);
//     // uint32_t length = curDmaDesc->len;
//     // uint32_t cacheLineSize = ce->dmaPort.sys->cacheLineSize();
//     // // Split the length into 64B, and invalidate the cache line one by one
//     // for (Addr addr_= addr ; addr_ < addr + length; addr_+=cacheLineSize)
//     // {
//     //     uint32_t size = std::min<uint32_t>(cacheLineSize, addr + length - addr_);
//     //     PacketPtr Ptrpkt_atomic = ce->RTL_Wrapper->createPacket(addr_, size,
//     //                                                 nullptr, 0, MemCmd::InvalidateReqDMA, 0, &cacheInvalidateEvent,0);
//     //     invaQueue.emplace(Ptrpkt_atomic);
//     // }
//     invaQueue.emplace(Ptrpkt);
//     cacheInvalidateComplete();
// }

// void
// AccMidgard::DMAChannel::cacheInvalidateComplete()
// {
//     if (!invaQueue.empty())
//     {
//         PacketPtr Ptrpkt = invaQueue.front();
//         DPRINTF(DMACopyEngine, "Invalidate Cache with Ptr:%s\n", Ptrpkt->print());
//         ce->dramPort.sendPacket(Ptrpkt, true);
//         ce->dramPort.tick();
//         invaQueue.pop();
//         nextState = CacheInvalidate;
//     }
//     else
//     {
//         DPRINTF(DMACopyEngine, "cacheInvalidateComplete\n");
//         nextState = DMAWrite;
//         if (inDrain()) return;
//         writeCopyBytes();
//     }
// }

void AccMidgard::DMAChannel::resetIntr()
{
    ce->intrClear();
}

void
AccMidgard::DMAChannel::fetchDescriptor(Addr address)
{
    DPRINTF(DMACopyEngine, "Reading descriptor from at memory location %#x(%#x)\n",
           address, ce->pciToDma(address));
    assert(address);
    busy = true;

    DPRINTF(DMACopyEngine, "dmaAction: %#x, %d bytes, from addr %#x\n",
            ce->pciToDma(address), sizeof(DmaDesc), curDmaDesc);

    // ce->dmaPort.dmaAction(MemCmd::ReadReqLLC, ce->pciToDma(address),
    //                  sizeof(DmaDesc), &fetchCompleteEvent,
    //                  (uint8_t*)curDmaDesc, latBeforeBegin);
    PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(address), sizeof(DmaDesc),
                                                    (uint8_t*)curDmaDesc, 0, MemCmd::ReadReq,
                                                    latBeforeBegin, &fetchCompleteEvent,0);
    if (ce->sys->isTimingMode())
    {
        ce->dramPort.sendPacket(Ptrpkt, true);
        ce->dramPort.tick();
    }
    else
    {
        ce->dramPort.sendPacket(Ptrpkt, false);
    }
    lastDescriptorAddr = address;
}

void
AccMidgard::DMAChannel::fetchDescComplete()
{
    DPRINTF(DMACopyEngine, "Read of descriptor complete\n");

    if ((curDmaDesc->command & DESC_CTRL_NULL)) {
        DPRINTF(DMACopyEngine, "Got NULL descriptor, skipping\n");
        assert(!(curDmaDesc->command & DESC_CTRL_CP_STS));
        if (curDmaDesc->command & DESC_CTRL_CP_STS) {
            panic("Shouldn't be able to get here\n");
            nextState = CompletionWrite;
            if (inDrain()) return;
            writeCompletionStatus();
        }
        else if(refreshNext){
            continueProcessing();
            return;
        }
        else{
            busy = false;
            nextState = Idle;
            inDrain();
        }
        return;
    }

    if (curDmaDesc->command & ~DESC_CTRL_CP_STS)
        panic("Descriptor has flag other that completion status set\n");

    if(curDmaDesc->forward && curDmaDesc->src==0x0000)// Just Write
    {
        DPRINTF(DMACopyEngine, "Got source with no forward, start writing process\n");
        nextState = DMAWrite;
        // Next part is about how to handle the DMA write
        /// @todo: more functions about the DMA write
    }
    else
    {
        // nextState = CacheFlush;
        nextState = DMARead;
        if (inDrain()) return;
        // cacheFlush();
        readCopyBytes();
    }
}

void
AccMidgard::DMAChannel::readCopyBytes()
{
    DPRINTF(DMACopyEngine, "Reading %d bytes from memory location %#x(%#x) to buffer\n",
           curDmaDesc->len, curDmaDesc->src,
           ce->pciToDma(curDmaDesc->src));

    // ce->dmaPort.dmaAction(MemCmd::ReadReqLLC, ce->pciToDma(curDmaDesc->src),
    //                  curDmaDesc->len, &readCompleteEvent, copyBuffer, 0);
    PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(curDmaDesc->src), curDmaDesc->len,
                                                    copyBuffer, Request::UNCACHEABLE, MemCmd::ReadReqLLC, 0, &readCompleteEvent,0);
    if (ce->sys->isTimingMode())
    {
        ce->dramPort.sendPacket(Ptrpkt, true);
        ce->dramPort.tick();
    }
    else
    {
        ce->dramPort.sendPacket(Ptrpkt, false);
    }
}

void
AccMidgard::DMAChannel::readCopyBytesComplete()
{
    DPRINTF(DMACopyEngine, "Read of bytes to copy complete\n");

    nextState = DMAWrite;
    if (inDrain()) return;
    if(curDmaDesc->forward)
    {
        DPRINTF(DMACopyEngine, "Got readforward, forward the packet!\n");
        // cacheInvalidate();
        writeCopyBytes();
    }
    else
    {
        curDmaDesc->readdone = 1;
        nextState = Idle;
        callAccel();
        DPRINTF(DMACopyEngine, "wait for the Acc to use it\n");
    }
}

void
AccMidgard::DMAChannel::writeCopyBytes()
{
    DPRINTF(DMACopyEngine, "Writing %d bytes from buffer to memory location %#x(%#x)\n",
           curDmaDesc->len, curDmaDesc->dest,
           ce->pciToDma(curDmaDesc->dest));

    // ce->dmaPort.dmaAction(MemCmd::WriteReqLLC, ce->pciToDma(curDmaDesc->dest),
    //                  curDmaDesc->len, &writeCompleteEvent, copyBuffer, 0);
    PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(curDmaDesc->dest), curDmaDesc->len,
                                                    copyBuffer, Request::UNCACHEABLE, MemCmd::WriteReq, 0, &writeCompleteEvent,0);

    // Packet *snoop_pkt = new Packet(Ptrpkt, true, false);
    // snoop_pkt->headerDelay = snoop_pkt->payloadDelay = 0;
    // snoop_pkt->setExpressSnoop();
    // snoop_pkt->setCacheResponding();

    // ce->dramPort.sendPacket(snoop_pkt, false);
    // ce->dramPort.tick();

    // Split the packet into 64B, and invalidate the cache line one by one
    // for (int i = 0; i < curDmaDesc->len; i+=64)
    // {
    //     PacketPtr Ptrpkt_atomic = ce->RTL_Wrapper->createPacket(curDmaDesc->dest, curDmaDesc->len,
    //                                                 copyBuffer, 0, MemCmd::CleanSharedReq, 0, nullptr,0);
    //     ce->dramPort.sendPacket(Ptrpkt_atomic, true);
    //     ce->dramPort.tick();
    // }

    if (ce->sys->isTimingMode())
    {
        ce->dramPort.sendPacket(Ptrpkt, true);
        ce->dramPort.tick();
    }
    else
    {
        ce->dramPort.sendPacket(Ptrpkt, false);
    }

    // ce->DMAStats.bytesCopied[channelId] += curDmaDesc->len;
    // ce->DMAStats.copiesProcessed[channelId]++;
}

void
AccMidgard::DMAChannel::writeCopyBytesComplete()
{
    DPRINTF(DMACopyEngine, "Write of bytes to copy complete user1: %#x\n",
            curDmaDesc->user1);

    cr.status.compl_desc_addr(lastDescriptorAddr >> 6);
    completionDataReg = cr.status() | 1;

    if (curDmaDesc->command & DESC_CTRL_CP_STS) {
        nextState = CompletionWrite;
        if (inDrain()) return;
        writeCompletionStatus();
        return;
    }

    continueProcessing();
}

void
AccMidgard::DMAChannel::continueProcessing()
{
    busy = false;
     DPRINTF(DMACopyEngine, "DMAChannel continueProcessing\n");
    if (underReset) {
        underReset = false;
        refreshNext = false;
        busy = false;
        nextState = Idle;
        return;
    }

    if (curDmaDesc->next) {
        nextState = DescriptorFetch;
        fetchAddress = curDmaDesc->next;
        if (inDrain()) return;
        fetchDescriptor(curDmaDesc->next);
    } else if (refreshNext) {
        nextState = AddressFetch;
        refreshNext = false;
        if (inDrain()) return;
        fetchNextAddr(lastDescriptorAddr);
    } else if (call_acce) {
        nextState = Idle;
        if (inDrain()) return;
        fetchNextAddr(lastDescriptorAddr);
    } else {
        inDrain();
        nextState = Idle;
    }
}

void
AccMidgard::DMAChannel::writeCompletionStatus()
{
    DPRINTF(DMACopyEngine, "Writing completion status %#x to address %#x(%#x)\n",
            completionDataReg, cr.completionAddr,
            ce->pciToDma(cr.completionAddr));

    // ce->dmaPort.dmaAction(MemCmd::WriteReqLLC,
    //                  ce->pciToDma(cr.completionAddr),
    //                  sizeof(completionDataReg), &statusCompleteEvent,
    //                  (uint8_t*)&completionDataReg, latAfterCompletion);
    PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(cr.completionAddr), sizeof(completionDataReg),
                                                    (uint8_t*)&completionDataReg, 0, MemCmd::WriteReq,
                                                    latAfterCompletion, &statusCompleteEvent,0);
    if (ce->sys->isTimingMode())
    {
        ce->dramPort.sendPacket(Ptrpkt, true);
        ce->dramPort.tick();
    }
    else
    {
        ce->dramPort.sendPacket(Ptrpkt, false);
    }
}

void
AccMidgard::DMAChannel::writeStatusComplete()
{
    DPRINTF(DMACopyEngine, "Writing completion status complete\n");
    continueProcessing();
}

void
AccMidgard::DMAChannel::fetchNextAddr(Addr address)
{
    DPRINTF(DMACopyEngine, "Fetching next address...\n");
    busy = true;
    // ce->dmaPort.dmaAction(MemCmd::ReadReqLLC,
    //                  ce->pciToDma(address + offsetof(DmaDesc, next)),
    //                  sizeof(Addr), &addrCompleteEvent,
    //                  (uint8_t*)curDmaDesc + offsetof(DmaDesc, next), 0);
    PacketPtr Ptrpkt = ce->RTL_Wrapper->createPacket(ce->pciToDma(address + offsetof(DmaDesc, next)), sizeof(Addr),
                                                    (uint8_t*)curDmaDesc + offsetof(DmaDesc, next), 0, MemCmd::ReadReq,
                                                    0, &addrCompleteEvent,0);
    if (ce->sys->isTimingMode())
    {
        ce->dramPort.sendPacket(Ptrpkt, true);
        ce->dramPort.tick();
    }
    else
    {
        ce->dramPort.sendPacket(Ptrpkt, false);
    }
}

void
AccMidgard::DMAChannel::fetchAddrComplete()
{
    DPRINTF(DMACopyEngine, "Fetching next address complete: %#x\n",
            curDmaDesc->next);
    if (!curDmaDesc->next) {
        DPRINTF(DMACopyEngine, "Got NULL descriptor, nothing more to do\n");
        busy = false;
        nextState = Idle;
        inDrain();
        DPRINTF(DMACopyEngine, "Accelerator Operation Finished, Trigger interruption!\n");
        ce->intrPost();
        ce->schedule(resetIntrEvent, curTick() + ce->clockPeriod());
        return;
    }
    nextState = DescriptorFetch;
    fetchAddress = curDmaDesc->next;
    if (inDrain()) return;
    fetchDescriptor(curDmaDesc->next);
}

void
AccMidgard::DMAChannel::callAccel()
{
    bool compute_start = ce->RTL_Wrapper->receiveData();
    if(compute_start)
        ce->schedule(AccelCompleteEvent, curTick() + ce->clockPeriod()*32); // Add latency here
    else
        ce->schedule(AccelCompleteEvent, curTick());
}

void
AccMidgard::DMAChannel::callAccelComplete()
{
    static int print_once = 1;
    if(readDone() == 0)
    {
        print_once = 1;
        continueProcessing();
    }
    else
    {
        if(print_once)
        {
            DPRINTF(DMACopyEngine,"Waiting for another channel ready\n");
            print_once = 0;
        }
        ce->schedule(AccelCompleteEvent, curTick() + ce->clockPeriod());
    }
}

bool
AccMidgard::DMAChannel::inDrain()
{
    if (drainState() == DrainState::Draining) {
        DPRINTF(Drain, "AccMidgard done draining, processing drain event\n");
        signalDrainDone();
    }

    return ce->drainState() != DrainState::Running;
}

DrainState
AccMidgard::DMAChannel::drain()
{
    if (nextState == Idle || ce->drainState() != DrainState::Running) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "DMAChannel not drained\n");
        return DrainState::Draining;
    }
}

void
AccMidgard::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);
    regs.serialize(cp);
    for (int x =0; x < chan.size(); x++)
        chan[x]->serializeSection(cp, csprintf("channel%d", x));
}

void
AccMidgard::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);
    regs.unserialize(cp);
    for (int x = 0; x < chan.size(); x++)
        chan[x]->unserializeSection(cp, csprintf("channel%d", x));
}

void
AccMidgard::DMAChannel::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(channelId);
    SERIALIZE_SCALAR(busy);
    SERIALIZE_SCALAR(underReset);
    SERIALIZE_SCALAR(refreshNext);
    SERIALIZE_SCALAR(lastDescriptorAddr);
    SERIALIZE_SCALAR(completionDataReg);
    SERIALIZE_SCALAR(fetchAddress);
    int nextState = this->nextState;
    SERIALIZE_SCALAR(nextState);
    arrayParamOut(cp, "curDmaDesc", (uint8_t*)curDmaDesc, sizeof(DmaDesc));
    SERIALIZE_ARRAY(copyBuffer, ce->params().XferCap);
    cr.serialize(cp);

}

void
AccMidgard::DMAChannel::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(channelId);
    UNSERIALIZE_SCALAR(busy);
    UNSERIALIZE_SCALAR(underReset);
    UNSERIALIZE_SCALAR(refreshNext);
    UNSERIALIZE_SCALAR(lastDescriptorAddr);
    UNSERIALIZE_SCALAR(completionDataReg);
    UNSERIALIZE_SCALAR(fetchAddress);
    int nextState;
    UNSERIALIZE_SCALAR(nextState);
    this->nextState = (ChannelState)nextState;
    arrayParamIn(cp, "curDmaDesc", (uint8_t*)curDmaDesc, sizeof(DmaDesc));
    UNSERIALIZE_ARRAY(copyBuffer, ce->params().XferCap);
    cr.unserialize(cp);

}

void
AccMidgard::DMAChannel::restartStateMachine()
{
    switch(nextState) {
      case AddressFetch:
        fetchNextAddr(lastDescriptorAddr);
        break;
      case DescriptorFetch:
        fetchDescriptor(fetchAddress);
        break;
    //   case CacheFlush:
    //     cacheFlush();
    //     break;
    //   case CacheInvalidate:
    //     cacheInvalidate();
    //     break;
      case DMARead:
        readCopyBytes();
        break;
      case DMAWrite:
        writeCopyBytes();
        break;
      case CompletionWrite:
        writeCompletionStatus();
        break;
      case Idle:
        break;
      default:
        panic("Unknown state for DMAChannel\n");
    }
}

bool
AccMidgard::DMAChannel::readDone()
{
    return (bool)curDmaDesc->readdone;
}

uint8_t *
AccMidgard::DMAChannel::getCopyBuffer()
{
    return copyBuffer;
}

uint16_t *
AccMidgard::DMAChannel::getCopyBuffer_offset()
{
    return buffer_offset;
}

copy_engine_reg::DmaDesc *
AccMidgard::DMAChannel::getCurDmaDesc()
{
    return curDmaDesc;
}

void
AccMidgard::DMAChannel::drainResume()
{
    DPRINTF(DMACopyEngine, "Restarting state machine at state %d\n", nextState);
    restartStateMachine();
}

AccMidgard::AccController::AccController(int buffer_num_, int buffer_size_,
                                        int READ_latency,
                                        std::vector<DMAChannel*> *chan_)
{

    // parameters assignment
    READ_latency = READ_latency;
    buffer_num = buffer_num_;
    buffer_size = buffer_size_;
    time_stamp = time_stamp;

    // DMA channel used in AccController
    chan = chan_;
    chan_number = chan_->size();
    chan_ready.resize(chan_number,false);

    //Vector initialization
    local_buffer.resize(buffer_num, std::vector<uint8_t>(buffer_size));

}

AccMidgard::AccController::~AccController()
{
    // Clear the contents of the vectors
    local_buffer.clear();

    // Shrink the vectors to fit their capacities after clearing
    local_buffer.shrink_to_fit();
}

void
AccMidgard::AccController::data_read(std::vector<std::vector<u_int8_t>> &local_buffer, uint64_t block_index, Addr addr, uint32_t size)
{
    /// Read Addr, size from the descriptor
}

PacketPtr
AccMidgard::AccController::createPacket(Addr addr, int size,
    uint8_t *data, Request::Flags flags, MemCmd::Command cmd,
    Tick delay, Event *event, uint32_t block_index_)
{
    RequestPtr req = std::make_shared<Request>(
        addr, size, flags, 0);
    req->taskId(context_switch_task_id::Unknown);

    req->setStreamId(0);
    req->setSubstreamId(0);
    req->taskId(context_switch_task_id::DMA);

    PacketPtr pkt = new Packet(req, cmd);
    pkt->dataStatic(data);

    auto state = new AcceleratorState;
    state->event = event;
    state->block_index = block_index_;
    state->delay = delay;

    pkt->senderState = state;
    return pkt;
}

bool
AccMidgard::AccController::sendData()
{

}

bool
AccMidgard::AccController::receiveData()
{

}


bool
AccMidgard::handleResponseMem(PacketPtr pkt, bool sram)
{
    // if (pkt->hasData()){
    //     assert(pkt->isRead());
    //     // Get the data ptr and sent it
    //     const uint8_t* dataPtr = pkt->getConstPtr<uint8_t>();
    //     auto *state = dynamic_cast<AcceleratorState*>(pkt->senderState);
    //     schedule(state->event, curTick());
    // }
    // else {
    //     // call DRAM or try to access data from PCIe bus
    //     /**
    //      * Explain what next to do with the DRAM port
    //      * 1. Judge whether there is a DRAM port
    //      * 2. If no try to access the PCIe interface
    //      */
    //     if (pkt->isWrite())
    //     {
    //         const uint8_t* dataPtr = pkt->getConstPtr<uint8_t>();
    //         auto *state = dynamic_cast<AcceleratorState*>(pkt->senderState);
    //         schedule(state->event, curTick());
    //     }
    // }
    auto *state = dynamic_cast<AcceleratorState*>(pkt->senderState);
    schedule(state->event, curTick());
    return true;
}

// DRAM PORT
void
AccMidgard::MemDevicePort::sendPacket(PacketPtr pkt, bool timing)
{
    if (timing) {
        //DPRINTF(DMACopyEngine, "Add Mem Req pending %#x size: %d timing s: %d\n",
        //    pkt->getAddr(), pkt->getSize(), pending_req.size());
        // we add as a pending request, we deal later
        pending_req.push(pkt);
    }
    else {
        //DPRINTF(DMACopyEngine, "Send Mem Req to DRAM %#x size: %d functional\n",
        //    pkt->getAddr(), pkt->getSize());
        // send Atomic
        sendAtomic(pkt);
        // Update all the pointers
        recentData32 = *pkt->getConstPtr<uint32_t>();
    }
}

// void
// AccMidgard::MemDevicePort::recvRangeChange()
// {
//     controller->sendRangeChange();
// }

bool
AccMidgard::MemDevicePort::recvTimingResp(PacketPtr pkt)
{
    //DPRINTF(DMACopyEngine, "Got response SRAM: %d\n", sram);
    return controller->handleResponseMem(pkt,sram);
}

void
AccMidgard::MemDevicePort::recvReqRetry()
{
    // we check we have pending packets
    assert(blockedRetry);
    // Grab the blocked packet.
    PacketPtr pkt = pending_req.front();
    bool sent = sendTimingReq(pkt);
    // if not sent put it in the queue
    if (sent) {
        pending_req.pop();
        blockedRetry = false;
    }
    // else
    // we do nothing
    // we failed sending the packet we wait
}

// In this function we send the packets
void
AccMidgard::MemDevicePort::tick()
{
    // we check we have pending packets
    if (!blockedRetry and !pending_req.empty()) {
        PacketPtr pkt = pending_req.front();
        bool sent = sendTimingReq(pkt);
        // if not sent, put it in the queue
        if (sent) {
            pending_req.pop();
        } else {
            blockedRetry = true;
        }
    }
}

} // namespace gem5
