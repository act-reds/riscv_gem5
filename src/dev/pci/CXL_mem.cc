/* @file
 * Non-Volatile Dual In-line Memory Module Virtualization Implementation
 */

#include "dev/pci/CXL_mem.hh"

#include <algorithm>

#include "base/compiler.hh"
#include "base/trace.hh"
#include "debug/DMACopyEngine.hh"
#include "debug/Drain.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/CXLMem.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace copy_engine_reg;

CXLMem::CXLMem(const Params &p)
    : PciDevice(p),
      CXLMemStats(this, p.ChanCnt)
{
    // All Reg regs are initialized to 0 by default
    regs.chanCount = p.ChanCnt;
    regs.xferCap = findMsbSet(p.XferCap);
    regs.attnStatus = 0;

    warn("CXL initialization!!\n");
    if (regs.chanCount > 64)
        fatal("CXLMem interface doesn't support more than 64 DMA engines\n");

    for (int x = 0; x < regs.chanCount; x++) {
        CXLMemChannel *ch = new CXLMemChannel(this, x);
        chan.push_back(ch);
    }
}


CXLMem::CXLMemChannel::CXLMemChannel(CXLMem *_ce, int cid)
    : ce(_ce), channelId(cid), busy(false), underReset(false),
      refreshNext(false), latBeforeBegin(ce->params().latBeforeBegin),
      latAfterCompletion(ce->params().latAfterCompletion),
      completionDataReg(0), nextState(Idle),
      fetchCompleteEvent([this]{ fetchDescComplete(); }, name()),
      addrCompleteEvent([this]{ fetchAddrComplete(); }, name()),
      readCompleteEvent([this]{ readCopyBytesComplete(); }, name()),
      writeCompleteEvent([this]{ writeCopyBytesComplete(); }, name()),
      statusCompleteEvent([this]{ writeStatusComplete(); }, name())

{
        cr.status.dma_transfer_status(3);
        cr.descChainAddr = 0;
        cr.completionAddr = 0;


        curDmaDesc = new DmaDesc;
        memset(curDmaDesc, 0, sizeof(DmaDesc));
        copyBuffer = new uint8_t[ce->params().XferCap];
}

CXLMem::~CXLMem()
{
    for (int x = 0; x < chan.size(); x++) {
        delete chan[x];
    }
}

CXLMem::CXLMemChannel::~CXLMemChannel()
{
    delete curDmaDesc;
    delete [] copyBuffer;
}

Port &
CXLMem::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "dma") {
        // pass it along to our super class
        return PciDevice::getPort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(chan.size())) {
            panic("CXLMem::getPort: unknown index %d\n", idx);
        }

        return DmaDevice::getPort(if_name,idx);
    }
}

void
CXLMem::CXLMemChannel::recvCommand()
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
CXLMem::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;
    warn("CXLMem Read function called!!\n");
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
CXLMem::CXLMemChannel::channelRead(Packet *pkt, Addr daddr, int size)
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
CXLMem::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    warn("CXLMem::write start!!!\n");

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
CXLMem::CXLMemChannel::channelWrite(Packet *pkt, Addr daddr, int size)
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

CXLMem::
CXLMemStats::CXLMemStats(statistics::Group *parent,
                                 const uint8_t &channel_count)
    : statistics::Group(parent, "CXLMem"),
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

void
CXLMem::CXLMemChannel::fetchDescriptor(Addr address)
{
    DPRINTF(DMACopyEngine, "Reading descriptor from at memory location %#x(%#x)\n",
           address, ce->pciToDma(address));
    assert(address);
    busy = true;

    DPRINTF(DMACopyEngine, "dmaAction: %#x, %d bytes, to addr %#x\n",
            ce->pciToDma(address), sizeof(DmaDesc), curDmaDesc);

    ce->dmaPort.dmaAction(MemCmd::ReadReq, ce->pciToDma(address),
                     sizeof(DmaDesc), &fetchCompleteEvent,
                     (uint8_t*)curDmaDesc, latBeforeBegin);
    lastDescriptorAddr = address;
}

void
CXLMem::CXLMemChannel::fetchDescComplete()
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
        } else if(refreshNext)
        {
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

    nextState = DMARead;
    if (inDrain()) return;
    readCopyBytes();
}

void
CXLMem::CXLMemChannel::readCopyBytes()
{
    DPRINTF(DMACopyEngine, "Reading %d bytes from memory location %#x(%#x) to buffer\n",
           curDmaDesc->len, curDmaDesc->src,
           ce->pciToDma(curDmaDesc->src));
    ce->dmaPort.dmaAction(MemCmd::ReadReq, ce->pciToDma(curDmaDesc->src),
                     curDmaDesc->len, &readCompleteEvent, copyBuffer, 0);
}

void
CXLMem::CXLMemChannel::readCopyBytesComplete()
{
    DPRINTF(DMACopyEngine, "Read of bytes to copy complete\n");

    nextState = DMAWrite;
    if (inDrain()) return;
    writeCopyBytes();
}

void
CXLMem::CXLMemChannel::writeCopyBytes()
{
    DPRINTF(DMACopyEngine, "Writing %d bytes from buffer to memory location %#x(%#x)\n",
           curDmaDesc->len, curDmaDesc->dest,
           ce->pciToDma(curDmaDesc->dest));

    ce->dmaPort.dmaAction(MemCmd::WriteReq, ce->pciToDma(curDmaDesc->dest),
                     curDmaDesc->len, &writeCompleteEvent, copyBuffer, 0);

    ce->CXLMemStats.bytesCopied[channelId] += curDmaDesc->len;
    ce->CXLMemStats.copiesProcessed[channelId]++;
}

void
CXLMem::CXLMemChannel::writeCopyBytesComplete()
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
CXLMem::CXLMemChannel::continueProcessing()
{
    busy = false;
     DPRINTF(DMACopyEngine, "CXLMemChannel continueProcessing\n");
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
    } else {
        inDrain();
        nextState = Idle;
    }
}

void
CXLMem::CXLMemChannel::writeCompletionStatus()
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
CXLMem::CXLMemChannel::writeStatusComplete()
{
    DPRINTF(DMACopyEngine, "Writing completion status complete\n");
    continueProcessing();
}

void
CXLMem::CXLMemChannel::fetchNextAddr(Addr address)
{
    DPRINTF(DMACopyEngine, "Fetching next address...\n");
    busy = true;
    ce->dmaPort.dmaAction(MemCmd::ReadReq,
                     ce->pciToDma(address + offsetof(DmaDesc, next)),
                     sizeof(Addr), &addrCompleteEvent,
                     (uint8_t*)curDmaDesc + offsetof(DmaDesc, next), 0);
}

void
CXLMem::CXLMemChannel::fetchAddrComplete()
{
    DPRINTF(DMACopyEngine, "Fetching next address complete: %#x\n",
            curDmaDesc->next);
    if (!curDmaDesc->next) {
        DPRINTF(DMACopyEngine, "Got NULL descriptor, nothing more to do\n");
        busy = false;
        nextState = Idle;
        inDrain();
        return;
    }
    nextState = DescriptorFetch;
    fetchAddress = curDmaDesc->next;
    if (inDrain()) return;
    fetchDescriptor(curDmaDesc->next);
}

bool
CXLMem::CXLMemChannel::inDrain()
{
    if (drainState() == DrainState::Draining) {
        DPRINTF(Drain, "CXLMem done draining, processing drain event\n");
        signalDrainDone();
    }

    return ce->drainState() != DrainState::Running;
}

DrainState
CXLMem::CXLMemChannel::drain()
{
    if (nextState == Idle || ce->drainState() != DrainState::Running) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "CXLMemChannel not drained\n");
        return DrainState::Draining;
    }
}

void
CXLMem::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);
    regs.serialize(cp);
    for (int x =0; x < chan.size(); x++)
        chan[x]->serializeSection(cp, csprintf("channel%d", x));
}

void
CXLMem::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);
    regs.unserialize(cp);
    for (int x = 0; x < chan.size(); x++)
        chan[x]->unserializeSection(cp, csprintf("channel%d", x));
}

void
CXLMem::CXLMemChannel::serialize(CheckpointOut &cp) const
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
CXLMem::CXLMemChannel::unserialize(CheckpointIn &cp)
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
CXLMem::CXLMemChannel::restartStateMachine()
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
        panic("Unknown state for CXLMemChannel\n");
    }
}

void
CXLMem::CXLMemChannel::drainResume()
{
    DPRINTF(DMACopyEngine, "Restarting state machine at state %d\n", nextState);
    restartStateMachine();
}

} // namespace gem5
