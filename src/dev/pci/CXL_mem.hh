/* @file
 * Non-Volatile Dual In-line Memory Module Virtualization Implementation
 */

#ifndef __DEV_PCI_CXL_MEM_HH__
#define __DEV_PCI_CXL_MEM_HH__

#include <vector>

#include "base/statistics.hh"
#include "dev/pci/copy_engine_defs.hh"
#include "dev/pci/device.hh"
#include "params/CXLMem.hh"
#include "sim/drain.hh"
#include "sim/eventq.hh"

namespace gem5
{

class CXLMem : public PciDevice
{
    class CXLMemChannel : public Drainable, public Serializable
    {
      private:
        CXLMem *ce;
        copy_engine_reg::ChanRegs  cr;
        int channelId;
        copy_engine_reg::DmaDesc *curDmaDesc;
        uint8_t *copyBuffer;

        bool busy;
        bool underReset;
        bool refreshNext;
        Addr lastDescriptorAddr;
        Addr fetchAddress;

        Tick latBeforeBegin;
        Tick latAfterCompletion;

        uint64_t completionDataReg;

        enum ChannelState
        {
            Idle,
            AddressFetch,
            DescriptorFetch,
            DMARead,
            DMAWrite,
            CompletionWrite
        };

        ChannelState nextState;

      public:
        CXLMemChannel(CXLMem *_ce, int cid);
        virtual ~CXLMemChannel();

        std::string
        name()
        {
            assert(ce);
            return ce->name() + csprintf("-chan%d", channelId);
        }

        virtual Tick read(PacketPtr pkt)
                        { panic("CXLMemChannel has no I/O access\n");}
        virtual Tick write(PacketPtr pkt)
                        { panic("CXLMemChannel has no I/O access\n"); }

        void channelRead(PacketPtr pkt, Addr daddr, int size);
        void channelWrite(PacketPtr pkt, Addr daddr, int size);

        DrainState drain() override;
        void drainResume() override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

      private:
        void fetchDescriptor(Addr address);
        void fetchDescComplete();
        EventFunctionWrapper fetchCompleteEvent;

        void fetchNextAddr(Addr address);
        void fetchAddrComplete();
        EventFunctionWrapper addrCompleteEvent;

        void readCopyBytes();
        void readCopyBytesComplete();
        EventFunctionWrapper readCompleteEvent;

        void writeCopyBytes();
        void writeCopyBytesComplete();
        EventFunctionWrapper writeCompleteEvent;

        void writeCompletionStatus();
        void writeStatusComplete();
        EventFunctionWrapper statusCompleteEvent;


        void continueProcessing();
        void recvCommand();
        bool inDrain();
        void restartStateMachine();
    };

  private:

    struct CXLMemStats : public statistics::Group
    {
        CXLMemStats(statistics::Group *parent,
            const uint8_t& channel_count);

        statistics::Vector bytesCopied;
        statistics::Vector copiesProcessed;
    } CXLMemStats;

    // device registers
    copy_engine_reg::Regs regs;

    // Array of channels each one with regs/dma port/etc
    std::vector<CXLMemChannel*> chan;

  public:
    PARAMS(CXLMem);
    CXLMem(const Params &params);
    ~CXLMem();

    Port &getPort(const std::string &if_name,
            PortID idx = InvalidPortID) override;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif //__DEV_PCI_COPY_ENGINE_HH__
