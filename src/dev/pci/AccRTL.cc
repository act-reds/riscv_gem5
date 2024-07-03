/* @file
 * Non-Volatile Dual In-line Memory Module Virtualization Implementation
 */

#include "dev/pci/AccRTL.hh"

#include <algorithm>
#include <semaphore.h>
#include <assert.h>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "debug/DMACopyEngine.hh"
#include "debug/AccRTLDebug.hh"
#include "debug/Drain.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/AccRTL.hh"
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

AccRTL::AccRTL(const AccRTLParams &p)
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
        fatal("AccRTL interface doesn't support more than 64 DMA engines\n");

    for (int x = 0; x < regs.chanCount; x++) {
        DMAChannel *ch = new DMAChannel(this, x);
        chan.push_back(ch);
    }

    int Matrix_lenth_ = 512;
    int buffer_length_ = 64;
    int READ_latency_Matrix = 0;
    int row_index_ = 0;
    u_int64_t time_stamp = 0;
    RTL_Wrapper = new AccController(this, Matrix_lenth_,buffer_length_,
                                    READ_latency_Matrix,&chan,
                                    row_index_,time_stamp);

    warn("call child functions from here");
}


AccRTL::DMAChannel::DMAChannel(AccRTL *_ce, int cid)
    : ce(_ce), channelId(cid), burstSize(ce->params().burstSize), busy(false), underReset(false),
      refreshNext(false), latBeforeBegin(ce->params().latBeforeBegin),
      latAfterCompletion(ce->params().latAfterCompletion),
      completionDataReg(0), nextState(Idle),
      fetchCompleteEvent([this]{ fetchDescComplete(); }, name()),
      addrCompleteEvent([this]{ fetchAddrComplete(); }, name()),
      AccelCompleteEvent([this]{callAccelComplete();}, name()),
      readCompleteEvent([this]{ readCopyBytesComplete(); }, name()),
      writeCompleteEvent([this]{ writeCopyBytesComplete(); }, name()),
      statusCompleteEvent([this]{ writeStatusComplete(); }, name())
{
        cr.status.dma_transfer_status(3);
        cr.descChainAddr = 0;
        cr.completionAddr = 0;

        buffer_offset = new uint16_t(0);
        curDmaDesc = new DmaDesc;
        memset(curDmaDesc, 0, sizeof(DmaDesc));
        copyBuffer = new uint8_t[ce->params().XferCap];
}

AccRTL::~AccRTL()
{
    for (int x = 0; x < chan.size(); x++) {
        delete chan[x];
    }
}

AccRTL::DMAChannel::~DMAChannel()
{
    delete curDmaDesc;
    delete [] copyBuffer;
}

Port &
AccRTL::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "dma") {
        // pass it along to our super class
        return PciDevice::getPort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(chan.size())) {
            panic("AccRTL::getPort: unknown index %d\n", idx);
        }

        return DmaDevice::getPort(if_name,idx);
    }
}

void
AccRTL::DMAChannel::recvCommand()
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
AccRTL::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;
    warn("AccRTL Read function called!!\n");
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
AccRTL::DMAChannel::channelRead(Packet *pkt, Addr daddr, int size)
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
AccRTL::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    warn("AccRTL::write start!!!\n");

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
AccRTL::DMAChannel::channelWrite(Packet *pkt, Addr daddr, int size)
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

AccRTL::
DMAStats::DMAStats(statistics::Group *parent,
                                 const uint8_t &channel_count)
    : statistics::Group(parent, "AccRTL"),
      ADD_STAT(bytesCopied_read, statistics::units::Byte::get(),
               "Number of bytes read by each engine"),
      ADD_STAT(copiesProcessed_read, statistics::units::Count::get(),
               "Number of copies processed by each engine"),
      ADD_STAT(bytesCopied_write, statistics::units::Byte::get(),
               "Number of bytes write by each engine"),
      ADD_STAT(copiesProcessed_write, statistics::units::Count::get(),
               "Number of copies processed by each engine")
{
    bytesCopied_read
        .init(channel_count)
        .flags(statistics::total)
        ;
    copiesProcessed_read
        .init(channel_count)
        .flags(statistics::total)
        ;
    bytesCopied_write
        .init(channel_count)
        .flags(statistics::total)
        ;
    copiesProcessed_write
        .init(channel_count)
        .flags(statistics::total)
        ;
}

void
AccRTL::DMAChannel::fetchDescriptor(Addr address)
{
    DPRINTF(DMACopyEngine, "Reading descriptor from at memory location %#x(%#x)\n",
           address, ce->pciToDma(address));
    assert(address);
    busy = true;

    DPRINTF(DMACopyEngine, "dmaAction: %#x, %d bytes, from addr %#x\n",
            ce->pciToDma(address), sizeof(DmaDesc), curDmaDesc);

    ce->dmaPort.dmaAction(MemCmd::ReadReq, ce->pciToDma(address),
                     sizeof(DmaDesc), &fetchCompleteEvent,
                     (uint8_t*)curDmaDesc, latBeforeBegin);
    lastDescriptorAddr = address;
}

void
AccRTL::DMAChannel::fetchDescComplete()
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
        nextState = DMARead;
        if (inDrain()) return;
        readCopyBytes();
    }
}

void
AccRTL::DMAChannel::readCopyBytes()
{
    DPRINTF(DMACopyEngine, "Reading %d bytes from memory location %#x(%#x) to buffer\n",
           curDmaDesc->len, curDmaDesc->src,
           ce->pciToDma(curDmaDesc->src));

    // ce->dmaPort.dmaAction_burst(MemCmd::ReadReq, ce->pciToDma(curDmaDesc->src),
    //                  curDmaDesc->len, &readCompleteEvent, copyBuffer, 0, burstSize,Request::UNCACHEABLE);
    ce->dmaPort.dmaAction(MemCmd::ReadReq, ce->pciToDma(curDmaDesc->src),
                     curDmaDesc->len, &readCompleteEvent, copyBuffer, 0);

    ce->DMAStats.bytesCopied_read[channelId] += curDmaDesc->len;
    ce->DMAStats.copiesProcessed_read[channelId]++;
}

void
AccRTL::DMAChannel::readCopyBytesComplete()
{
    DPRINTF(DMACopyEngine, "Read of bytes to copy complete\n");

    nextState = DMAWrite;
    if (inDrain()) return;
    if(curDmaDesc->forward)
    {
        DPRINTF(DMACopyEngine, "Got readforward, forward the packet!\n");
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
AccRTL::DMAChannel::writeCopyBytes()
{
    DPRINTF(DMACopyEngine, "Writing %d bytes from buffer to memory location %#x(%#x)\n",
           curDmaDesc->len, curDmaDesc->dest,
           ce->pciToDma(curDmaDesc->dest));

    // ce->dmaPort.dmaAction_burst(MemCmd::WriteReq, ce->pciToDma(curDmaDesc->dest),
    //                  curDmaDesc->len, &writeCompleteEvent, copyBuffer, 0, burstSize, Request::UNCACHEABLE);
    ce->dmaPort.dmaAction(MemCmd::WriteReq, ce->pciToDma(curDmaDesc->dest),
                     curDmaDesc->len, &writeCompleteEvent, copyBuffer, 0);

    ce->DMAStats.bytesCopied_write[channelId] += curDmaDesc->len;
    ce->DMAStats.copiesProcessed_write[channelId]++;
}

void
AccRTL::DMAChannel::writeCopyBytesComplete()
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
AccRTL::DMAChannel::continueProcessing()
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
AccRTL::DMAChannel::writeCompletionStatus()
{
    DPRINTF(DMACopyEngine, "Writing completion status %#x to address %#x(%#x)\n",
            completionDataReg, cr.completionAddr,
            ce->pciToDma(cr.completionAddr));

    ce->dmaPort.dmaAction(MemCmd::WriteReq,
                     ce->pciToDma(cr.completionAddr),
                     sizeof(completionDataReg), &statusCompleteEvent,
                     (uint8_t*)&completionDataReg, latAfterCompletion);
}

void
AccRTL::DMAChannel::writeStatusComplete()
{
    DPRINTF(DMACopyEngine, "Writing completion status complete\n");
    continueProcessing();
}

void
AccRTL::DMAChannel::fetchNextAddr(Addr address)
{
    DPRINTF(DMACopyEngine, "Fetching next address...\n");
    busy = true;
    ce->dmaPort.dmaAction(MemCmd::ReadReq,
                     ce->pciToDma(address + offsetof(DmaDesc, next)),
                     sizeof(Addr), &addrCompleteEvent,
                     (uint8_t*)curDmaDesc + offsetof(DmaDesc, next), 0);
}

void
AccRTL::DMAChannel::fetchAddrComplete()
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
        return;
    }
    nextState = DescriptorFetch;
    fetchAddress = curDmaDesc->next;
    if (inDrain()) return;
    fetchDescriptor(curDmaDesc->next);
}

void
AccRTL::DMAChannel::callAccel()
{
    bool compute_start = ce->RTL_Wrapper->receiveData();
    if(compute_start)
        ce->schedule(AccelCompleteEvent, curTick() + ce->clockPeriod()*ce->AccLatency); // Add latency here
    else
        ce->schedule(AccelCompleteEvent, curTick());
}

void
AccRTL::DMAChannel::callAccelComplete()
{
    static int print_once = 1;
    if(readDone() == 0)
    {
    //     DPRINTF(DMACopyEngine,"both channel ready!\n");
    //     continueProcessing();
    //     // if(ce->RTL_Wrapper->set_index == 1)
    //     // {
    //     //     (*ce->RTL_Wrapper->chan)[0]->continueProcessing();
    //     //     (*ce->RTL_Wrapper->chan)[1]->continueProcessing();
    //     // }
    //     // else if (ce->RTL_Wrapper->set_index == 2)
    //     // {
    //     //     (*ce->RTL_Wrapper->chan)[2]->continueProcessing();
    //     //     (*ce->RTL_Wrapper->chan)[3]->continueProcessing();
    //     // }
    // }
    // else if (ce->RTL_Wrapper->channel1_ready == true)
    // {
    //     DPRINTF(DMACopyEngine,"No need to wait, continue!\n");
    //     ce->RTL_Wrapper->channel1_ready = false;
    //     continueProcessing();
    // }
    // else if (ce->RTL_Wrapper->channel2_ready == true)
    // {
    //     DPRINTF(DMACopyEngine,"No need to wait, continue!\n");
    //     ce->RTL_Wrapper->channel2_ready = false;
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
AccRTL::DMAChannel::inDrain()
{
    if (drainState() == DrainState::Draining) {
        DPRINTF(Drain, "AccRTL done draining, processing drain event\n");
        signalDrainDone();
    }

    return ce->drainState() != DrainState::Running;
}

DrainState
AccRTL::DMAChannel::drain()
{
    if (nextState == Idle || ce->drainState() != DrainState::Running) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "DMAChannel not drained\n");
        return DrainState::Draining;
    }
}

void
AccRTL::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);
    regs.serialize(cp);
    for (int x =0; x < chan.size(); x++)
        chan[x]->serializeSection(cp, csprintf("channel%d", x));
}

void
AccRTL::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);
    regs.unserialize(cp);
    for (int x = 0; x < chan.size(); x++)
        chan[x]->unserializeSection(cp, csprintf("channel%d", x));
}

void
AccRTL::DMAChannel::serialize(CheckpointOut &cp) const
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
AccRTL::DMAChannel::unserialize(CheckpointIn &cp)
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
AccRTL::DMAChannel::restartStateMachine()
{
    switch(nextState) {
      case AddressFetch:
        fetchNextAddr(lastDescriptorAddr);
        break;
      case DescriptorFetch:
        fetchDescriptor(fetchAddress);
        break;
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
AccRTL::DMAChannel::readDone()
{
    return (bool)curDmaDesc->readdone;
}

uint8_t *
AccRTL::DMAChannel::getCopyBuffer()
{
    return copyBuffer;
}

uint16_t *
AccRTL::DMAChannel::getCopyBuffer_offset()
{
    return buffer_offset;
}

copy_engine_reg::DmaDesc *
AccRTL::DMAChannel::getCurDmaDesc()
{
    return curDmaDesc;
}

void
AccRTL::DMAChannel::drainResume()
{
    DPRINTF(DMACopyEngine, "Restarting state machine at state %d\n", nextState);
    restartStateMachine();
}

AccRTL::AccController::AccController(AccRTL *_ce, int Matrix_lenth_, int buffer_length_,
                                    int READ_latency_Matrix,
                                    std::vector<DMAChannel*> *chan_,
                                    int row_index_, u_int64_t time_stamp):
        AccWrapper(_ce),
        LoadCompleteEvent_1([this]{ loaddata_complete(1); }, "Readchan1"),
        LoadCompleteEvent_2([this]{ loaddata_complete(2); }, "Readchan2"),
        StoreCompleteEvent([this]{ storedata_complete(); }, "write")
{
    Matrix_lenth = Matrix_lenth_;
    buffer_length = buffer_length_;
    READ_latency_Matrix = READ_latency_Matrix;
    row_index_block = row_index_;
    time_stamp = time_stamp;

    ARRAY_SIZE = ARRAY_SIZE_;


    //DPRINTF(DMACopyEngine,"AccController initialization!\n");
    // DMA channel used in AccController
    chan = chan_;
    chan_number = chan_->size();
    chan_ready.resize(chan_number,false);
    //DPRINTF(DMACopyEngine,"channel number:%d\n",chan_number);
    //Vector initialization
    inputFlow.resize(ARRAY_SIZE+buffer_length-1);
    local_buffer.resize(ARRAY_SIZE, std::vector<long int>(ARRAY_SIZE));
    //DPRINTF(DMACopyEngine,"inputFlow size:%d\n",(ARRAY_SIZE+Matrix_lenth-1));
    //DPRINTF(DMACopyEngine,"local_buffer size:%d\n",ARRAY_SIZE);
    // Matrix initialization
    A.resize(Matrix_lenth, std::vector<int>(Matrix_lenth));
    B.resize(Matrix_lenth, std::vector<int>(Matrix_lenth));
    C.resize(Matrix_lenth, std::vector<long int>(Matrix_lenth));
    // SubMatrix initialization
    A_channel1.resize(ARRAY_SIZE, std::vector<int>(buffer_length));
    A_channel2.resize(ARRAY_SIZE, std::vector<int>(buffer_length));
    // B_channel1.resize(buffer_length, std::vector<int>(ARRAY_SIZE));
    // B_channel2.resize(buffer_length, std::vector<int>(ARRAY_SIZE));
    B_channel1.resize(ARRAY_SIZE, std::vector<int>(buffer_length));
    B_channel2.resize(ARRAY_SIZE, std::vector<int>(buffer_length));
    C_channel.resize(ARRAY_SIZE, std::vector<long int>(ARRAY_SIZE));

    //DPRINTF(DMACopyEngine,"create shared memory space for RTL exe\n");
    // For shared data memory
    int shm_fd = shm_open(shmName, O_CREAT | O_RDWR, 0666);
    int result = ftruncate(shm_fd, sizeof(SharedData));
    if (result == -1) {
        perror("ftruncate failed");
        // Handle the error as appropriate for your application
    }
    sharedData = (SharedData*)mmap(0, sizeof(SharedData), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
    semaphore = sem_open(semName, O_CREAT, 0666, 1);

    data_init(A,B,Matrix_lenth);

    pid_t pid = fork();
    if (pid == -1) {
        std::cerr << "Fork failed." << std::endl;
    }

    if (pid == 0) {  // Child process
    	std::cout<<"Child process"<<std::endl;
        std::cout<<"debug: "<<1<<std::endl;
        std::cout<<"Matrix_lenth: "<<Matrix_lenth<<std::endl;
        std::cout<<"trace: "<<1<<std::endl;
        //DPRINTF(DMACopyEngine,"child RTL process: debug:%d; trace:%d\n",1,1);
        const char* args[] = {"/home/quliu/Desktop/RTL_generator/systolic_array_matrix_multiplier_NN/obj_dir/Vsystolic_array", "1", "1", nullptr};
        execvp(args[0], const_cast<char* const*>(args));
        std::cerr << "Execvp failed." << std::endl;
    }
    else
    {
        std::cout<<"Parent process"<<std::endl;
        time_stamp = 1000;
        function_caller(&AccRTL::AccController::reset);
    }
}

AccRTL::AccController::~AccController()
{
    // Clear the contents of the vectors
    local_buffer.clear();
    inputFlow.clear();
    A.clear();
    B.clear();
    C.clear();
    A_channel1.clear();
    B_channel1.clear();
    A_channel2.clear();
    B_channel2.clear();
    C_channel.clear();

    // Shrink the vectors to fit their capacities after clearing
    local_buffer.shrink_to_fit();
    inputFlow.shrink_to_fit();
    A.shrink_to_fit();
    B.shrink_to_fit();
    C.shrink_to_fit();
    A_channel1.shrink_to_fit();
    B_channel1.shrink_to_fit();
    A_channel2.shrink_to_fit();
    B_channel2.shrink_to_fit();
    C_channel.shrink_to_fit();
    //DPRINTF(DMACopyEngine,"execution finished, clear everything\n");
}

void
AccRTL::AccController::function_caller(void (AccController::*control_function)())
{
    while(true)
    {
        sem_wait(semaphore);
        time_stamp = sharedData->time_stamp;
        if(sharedData->permission == 0)
        {
            (this->*control_function)();
            sharedData->permission = 1;
            break;
        }
        sem_post(semaphore);
    }
    sem_post(semaphore);
}


void
AccRTL::AccController::convertToSystolicInput(const std::vector<std::vector<int>>& A,
    const std::vector<std::vector<int>>& B, std::vector<TimeStepInput> &inputFlow)
{
    for (int time = 0; time < ARRAY_SIZE + buffer_length - 1; ++time) {
    inputFlow[time].matrixA_row.resize(ARRAY_SIZE, 0);
    inputFlow[time].matrixB_col.resize(ARRAY_SIZE, 0);

    // Determine the row of A to feed in
    for (int i = 0; i < ARRAY_SIZE; ++i) {
        if (((time - i) >= 0) && ((time - i) < (buffer_length)) ) {
            inputFlow[time].matrixA_row[i] = A[i][time - i];
        }
    }

    // Determine the column of B to feed in
    for (int j = 0; j < ARRAY_SIZE; ++j) {
        if (((time - j) >= 0) && ((time - j) < (buffer_length)) ) {
            inputFlow[time].matrixB_col[j] = B[j][time - j];
        }
    }
    }
    //DPRINTF(DMACopyEngine,"convert to SystolicInput!\n");
}

void
AccRTL::AccController::read_data()
{
    static int row_value = 0;
    //DPRINTF(DMACopyEngine,"read_data, row_value:%d\n",row_value);
    for(int i = 0; i<ARRAY_SIZE; i++)
    {
        sharedData->inp_west[i] = inputFlow[row_value].matrixB_col[i];
        sharedData->inp_north[i] = inputFlow[row_value].matrixA_row[i];
        sharedData->instruction = READ_DATA;
    }

    row_value = (row_value >= ((buffer_length+ARRAY_SIZE) - 2))? 0 : row_value+1;
}

void
AccRTL::AccController::read_finish()
{
    //DPRINTF(DMACopyEngine,"read_data, finished!!\n");
    sharedData->instruction = READ_FINISH;
}

void
AccRTL::AccController::start_function()
{
    sharedData->instruction = START_COMPUTE;
}


void
AccRTL::AccController::read_result()
{
    //DPRINTF(DMACopyEngine,"read result!!\n");
    sharedData->instruction = READ_RES;
    sharedData->row_index = row_index_inner;
}

void
AccRTL::AccController::check_res()
{
    // printf("result_index[%d]:",row_index_);
    for(int i = 0; i<ARRAY_SIZE; i++)
    {
        // printf(" ,%ld",sharedData->result[i]);
        local_buffer[row_index_inner][i]=sharedData->result[i];
    }
    // printf("\n");
    sharedData->instruction = CHECK_RES;
}

void
AccRTL::AccController::done()
{
    sharedData->instruction = DONE;
}

void
AccRTL::AccController::reset()
{
    sharedData->time_last = 10 + READ_latency_Matrix;
    sharedData->instruction = RESET;
}

void
AccRTL::AccController::pause()
{
    sharedData->time_last = 10;
    sharedData->instruction = PAUSE;
}

int
AccRTL::AccController::data_read_row(std::vector<std::vector<int>> &sub_matrix_A,
    std::vector<std::vector<int>> &matrix_A, uint64_t row_index, uint64_t block_index)
{
    //DPRINTF(DMACopyEngine,"read row data from external SRAM,row_index:%d\n",
    //        row_index);
    for (size_t i = 0; i < ARRAY_SIZE; i++)
    {
        for (size_t j = 0; j < buffer_length; j++)
        {
            sub_matrix_A[i][j] = matrix_A[i + row_index][j+block_index];
        }
    }
    return buffer_length/16;
}

int
AccRTL::AccController::data_read_column(std::vector<std::vector<int>> &sub_matrix_B,
    const std::vector<std::vector<int>> &matrix_B, uint64_t col_index, uint64_t block_index)
{
    //DPRINTF(DMACopyEngine,"read row data from external SRAM,col_index:%d\n",
    //        col_index);
    for (size_t i = 0; i < buffer_length; i++)
    {
        for (size_t j = 0; j < ARRAY_SIZE; j++)
        {
            sub_matrix_B[i][j] = matrix_B[i+block_index][j + col_index];
        }
    }
    return buffer_length/16;
}

void
AccRTL::AccController::Matrix_subtore(const std::vector<std::vector<long int>> &sub_Matrix,
    std::vector<std::vector<long int>> &Matrix, uint64_t row_index, uint64_t col_index)
{
    //DPRINTF(DMACopyEngine,"Matrix substore to SRAM:row_index:%d;col_index:%d\n",
    //        row_index, col_index);
    for (size_t i = 0; i < ARRAY_SIZE; i++)
    {
        for (size_t j = 0; j < ARRAY_SIZE; j++)
        {
            Matrix[row_index+i][col_index+j] = sub_Matrix[j][i];
        }
    }
}

void
AccRTL::AccController::Matrix_store(const std::vector<std::vector<int>> &A,
    const std::vector<std::vector<int>> &B, const std::vector<std::vector<long int>> &C)
{
    //DPRINTF(DMACopyEngine,"Store data back to external storage\n");
    std::ofstream file("matrices_MN_class.txt");
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing." << std::endl;
        return;
    }

    file << "Matrix A:" << std::endl;
    for (const auto &row : A)
    {
        for (int val : row)
        {
            file << val << " ";
        }
        file << std::endl;
    }

    file << "\nMatrix B:" << std::endl;
    for (const auto &row : B)
    {
        for (int val : row)
        {
            file << val << " ";
        }
        file << std::endl;
    }

    file << "\nMatrix C:" << std::endl;
    for (const auto &row : C)
    {
        for (int val : row)
        {
            file << val << " ";
        }
        file << std::endl;
    }

    file.close();
}

void
AccRTL::AccController::Matrix_calculation()
{
    printf("Matrix_calculation!\n");
    for (size_t row_index = 0; row_index < Matrix_lenth; row_index += ARRAY_SIZE) // Row
    {
        for (size_t col_index = 0; col_index < Matrix_lenth; col_index+=ARRAY_SIZE) // Column
        {
            function_caller(&AccRTL::AccController::reset);
            for (size_t block_index = 0; block_index < Matrix_lenth;)
            {
                if(channel1_read & (block_index < Matrix_lenth))
                {
                    channel1_ready = false;
                    READ_latency_Matrix += data_read_row(A_channel1, A, row_index, block_index);
                    READ_latency_Matrix += data_read_column(B_channel1, B, col_index, block_index);
                    channel1_ready = true;
                    channel1_read = false;
                    block_index += buffer_length;
                }

                if(channel2_read & (block_index < Matrix_lenth))
                {
                    channel2_ready = false;
                    READ_latency_Matrix += data_read_row(A_channel2, A, row_index, block_index);
                    READ_latency_Matrix += data_read_column(B_channel2, B, col_index, block_index);
                    channel2_ready = true;
                    channel2_read = false;
                    block_index += buffer_length;
                }

                function_caller(&AccRTL::AccController::pause);
                if (channel1_ready == true)
                {
                    // Convert matrices to systolic input flow
                    convertToSystolicInput(A_channel1, B_channel1, inputFlow);
                    for (size_t i = 0; i < (ARRAY_SIZE+buffer_length-1); i++)
                    {
                        function_caller(&AccRTL::AccController::read_data);
                    }
                    READ_latency_Matrix = 0; // Not sure
                    channel1_read = true;
                }
                if (channel2_ready == true)
                {
                    // Convert matrices to systolic input flow
                    convertToSystolicInput(A_channel2, B_channel2, inputFlow);
                    for (size_t i = 0; i < (ARRAY_SIZE+buffer_length-1); i++)
                    {
                        function_caller(&AccRTL::AccController::read_data);
                    }
                    READ_latency_Matrix = 0; // Not sure
                    channel2_read = true;
                }
            }

            function_caller(&AccRTL::AccController::read_finish);
            function_caller(&AccRTL::AccController::start_function);// Maybe change to pause!!
            // Finish the row and column multiplication
            for (size_t i = 0; i < ARRAY_SIZE; i++)
            {
                function_caller(&AccRTL::AccController::read_result);
                function_caller(&AccRTL::AccController::check_res);
                row_index_inner++;
            }
            Matrix_subtore(local_buffer,C,row_index,col_index);
            row_index_inner = 0;
            READ_latency_Matrix = 0;
            for (auto& row : local_buffer)
                std::fill(row.begin(), row.end(), 0);
        }
    }
    printf("try to store data!\n");
    function_caller(&AccRTL::AccController::done);
    Matrix_store(A,B,C);
    printf("time_stamp from parent process:%ld\n",time_stamp);

    sem_unlink(semName);
}

void
AccRTL::AccController::load_data(std::vector<std::vector<int>> &A_channel, std::vector<std::vector<int>> &B_channel)
{
    READ_latency_Matrix += data_read_row(A_channel, A, row_index_block, block_index);
    READ_latency_Matrix += data_read_column(B_channel, B, col_index_block, block_index);
    uint64_t address;
    // (*chan)[0]->readCopyBytes();
}


void
AccRTL::AccController::loaddata_complete(int channel_index)
{
    // If both channel are ready, start the computing
    if(channel_index == 0)
    {
        printf("channel 1 reading finish!!\n");
    }
    else
    {
        printf("channel 2 reading finish!!\n");
    }
    if(true)
        computing();
}

void
AccRTL::AccController::store_data()
{
    int addr = row_index_block; // Store the data by its address

    // store the result of the whole row into the result
}

void
AccRTL::AccController::storedata_complete()
{
    // If both channel are ready, start the computing
    printf("store data finish!!\n");
}

void DataArray_Conversion(uint8_t *copyBuffer, std::vector<std::vector<int>> &channel_data, int array_size)
{
    printf("channel data:\n");
    int index = 0;
    for (size_t i = 0; i < channel_data.size(); i++)
    {
        for (size_t j = 0; j < channel_data[0].size(); j++)
        {
            // channel_data[i][j] = ((int*)copyBuffer)[index++];
                                // +(copyBuffer[(i*channel_data.size() + j)*4 + 1] << 8)
                                // +(copyBuffer[(i*channel_data.size() + j)*4 + 2] << 16)
                                // +(copyBuffer[(i*channel_data.size() + j)*4 + 3] << 24);
            int inBlockCol = (index % array_size) + ((int)(index/(array_size*array_size)))*array_size;
            int inBlockRow = (index / array_size)%array_size;
            channel_data[inBlockRow][inBlockCol] = ((int*)copyBuffer)[index++];
            printf("%d,",channel_data[i][j]);
        }
        printf("\n");
    }
}

void
AccRTL::AccController::computing()
{
    if(set_index == 1)// The first of the A,B Matrix
    {
        DataArray_Conversion((*chan)[0]->getCopyBuffer(), A_channel1, ARRAY_SIZE);
        DataArray_Conversion((*chan)[1]->getCopyBuffer(), B_channel1, ARRAY_SIZE);
        convertToSystolicInput(A_channel1, B_channel1, inputFlow);
        channel1_ready = true;
    }
    else if (set_index == 2)// The second set of the A,B Matrix
    {
        DataArray_Conversion((*chan)[2]->getCopyBuffer(), A_channel2, ARRAY_SIZE);
        DataArray_Conversion((*chan)[3]->getCopyBuffer(), B_channel2, ARRAY_SIZE);
        convertToSystolicInput(A_channel2, B_channel2, inputFlow);
        channel2_ready = true;
    }
    else
    {
        printf("Error!!should have ready input set!\n");
    }

    for (size_t i = 0; i < (ARRAY_SIZE+buffer_length-1); i++)
    {
        function_caller(&AccRTL::AccController::read_data);
    }
    set_index = 0;

    if(block_index == AccWrapper->chan[2]->getCurDmaDesc()->user2)
    {
        function_caller(&AccRTL::AccController::read_finish);
        function_caller(&AccRTL::AccController::start_function);// Maybe change to pause!!
        row_index_inner = 0;
        for (size_t i = 0; i < ARRAY_SIZE; i++)
        {
            function_caller(&AccRTL::AccController::read_result);
            function_caller(&AccRTL::AccController::check_res);
            row_index_inner++;
        }
        row_index_block += ARRAY_SIZE;
        col_index_block += ARRAY_SIZE;
        // Matrix_subtore(local_buffer,C,row_index_block,col_index_block);
        for (size_t i = 0; i < local_buffer.size(); i++)
        {
            printf("local_buffer[%d][j]:\n",i);
            for (size_t j = 0; j < local_buffer[0].size(); j++)
            {
                ((int *)(*chan)[2]->getCopyBuffer())[i*local_buffer[0].size() + j + *((*chan)[2]->getCopyBuffer_offset())] = local_buffer[j][i];
                printf("%ld,",local_buffer[i][j]);
            }
            printf("\n");
        }
        *((*chan)[2]->getCopyBuffer_offset()) += ARRAY_SIZE*ARRAY_SIZE;
        printf("reading data from localbuffer!\n");
        printf("restart the state Machine for channel two!\n");
        if (*((*chan)[2]->getCopyBuffer_offset()) >= ARRAY_SIZE*ARRAY_SIZE*4)
        {
            (*chan)[2]->restartStateMachine();
            *((*chan)[2]->getCopyBuffer_offset()) = 0;
        }
        block_index = 0;
        function_caller(&AccRTL::AccController::reset);
    }
    READ_latency_Matrix = 0; // Not sure
    AccWrapper->AccLatency = 256;
}

bool
AccRTL::AccController::receiveData()
{
    uint8_t ready_bits = 0x00;
    for (size_t i = 0; i < chan_number; i++)
    {
        chan_ready[i] = (*chan)[i]->readDone();
        ready_bits = chan_ready[i] + (ready_bits << 1);
    }

    switch (ready_bits>>2)
    {
    case 0b00:
        assert(ready_bits != 0);
        break;
    case 0b01:
    case 0b10:
        printf("only one channel is ready!\n");
        break;
    case 0b11:
        block_index += buffer_length;
        set_index = 1;
        printf("both channels are ready!\n");
        computing();
        (*chan)[0]->getCurDmaDesc()->readdone = 0;
        (*chan)[1]->getCurDmaDesc()->readdone = 0;
        break;
    default:
        break;
    }

    switch ((ready_bits & 0x03))
    {
    case 0b00:
        assert(ready_bits != 0);
        break;
    case 0b01:
    case 0b10:
        printf("only one channel is ready!\n");
        break;
    case 0b11:
        block_index += buffer_length;
        set_index = 2;
        printf("both channels are ready!\n");
        computing();
        (*chan)[0]->getCurDmaDesc()->readdone = 0;
        (*chan)[1]->getCurDmaDesc()->readdone = 0;
        break;
    default:
        break;
    }

    return (bool) set_index;
}

void
AccRTL::AccController::state_processing()
{
    switch (Accstate)
    {
    case ReadData:
        read_data();
        break;
    case WriteData:
        // write_data();
        break;
    case Computing:
        computing();
        break;
    case Idle:
        break;
    default:
        break;
    }

    // The following logic function for state modification
    // transit from one state to another state

}

void
AccRTL::AccController::state_controller()
{
    // for (size_t row_index = 0; row_index < Matrix_lenth; row_index += ARRAY_SIZE) // Row
    // {
    //     for (size_t col_index = 0; col_index < Matrix_lenth; col_index+=ARRAY_SIZE) // Column
    //     {
    //         function_caller(&AccRTL::AccController::reset);
    //         for (size_t block_index = 0; block_index < Matrix_lenth;)

    if (Accstate == ReadData)
    {
        if(channel1_read & (block_index < Matrix_lenth))
        {
            channel1_ready = false;
            load_data(A_channel1, B_channel1);
            channel1_ready = true;
            channel1_read = false;
            block_index += buffer_length;
        }

        if(channel2_read & (block_index < Matrix_lenth))
        {
            channel2_ready = false;
            load_data(A_channel2, B_channel2);
            channel2_ready = true;
            channel2_read = false;
            block_index += buffer_length;
        }

        if (block_index>=Matrix_lenth)
        {
            Accstate = Computing;
        }

    }
}


bool
AccRTL::handleResponseMem(PacketPtr pkt, bool sram)
{
    if (pkt->hasData()){
        if (pkt->isRead()){
            //DPRINTF(DMACopyEngine,
            //       "Handling response for data read Timing\n");
            // Get the data ptr and sent it
            const uint8_t* dataPtr = pkt->getConstPtr<uint8_t>();
        } else {
            // this is somehow odd, report!
            //DPRINTF(DMACopyEngine, "Got response for addr %#x no read\n",
            //        pkt->getAddr());
        }
    }
    else {
         //DPRINTF(DMACopyEngine, "Got response for addr %#x no data\n",
         //pkt->getAddr());
         // call DRAM or try to access data from PCIe bus
         /**
          * Explain what next to do with the DRAM port
          * 1. Judge whether there is a DRAM port
          * 2. If no try to access the PCIe interface
         */
    }

    return true;
}

// DRAM PORT
void
AccRTL::MemDevicePort::sendPacket(PacketPtr pkt, bool timing)
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
// AccRTL::MemDevicePort::recvRangeChange()
// {
//     controller->sendRangeChange();
// }

bool
AccRTL::MemDevicePort::recvTimingResp(PacketPtr pkt)
{
    //DPRINTF(DMACopyEngine, "Got response SRAM: %d\n", sram);
    return controller->handleResponseMem(pkt,sram);
}

void
AccRTL::MemDevicePort::recvReqRetry()
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
AccRTL::MemDevicePort::tick()
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
