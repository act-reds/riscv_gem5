/* @file
 * Non-Volatile Dual In-line Memory Module Virtualization Implementation
 */

#ifndef __DEV_PCI_ACCMIDGARD_HH__
#define __DEV_PCI_ACCMIDGARD_HH__

#include <algorithm>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sstream>
#include <sys/mman.h>
#include <fcntl.h>
#include <semaphore.h>
#include <vector>
#include <cassert>
#include <string>
#include <queue>

#include "base/statistics.hh"
#include "cpu/base.hh"
#include "cpu/translation.hh"
#include "dev/pci/copy_engine_defs.hh"
#include "dev/pci/device.hh"
#include "params/AccMidgard.hh"
#include "sim/drain.hh"
#include "sim/eventq.hh"

namespace gem5
{

typedef enum{
    RESET,
    READ_DATA,
    READ_FINISH,
    START_COMPUTE,
    CHECK_RES,
    READ_RES,
    PAUSE,
    DONE
} control_bit;

struct AccMidgardParams;

class AccMidgard : public PciDevice
{
public:
  class DMAChannel : public Drainable, public Serializable
  {
    private:
      AccMidgard *ce;
      copy_engine_reg::ChanRegs  cr;
      int channelId;
      copy_engine_reg::DmaDesc *curDmaDesc;
      uint8_t *copyBuffer;

      bool busy;
      bool underReset;
      bool refreshNext;
      bool call_acce = 1;
      Addr lastDescriptorAddr;
      Addr fetchAddress;

      Tick latBeforeBegin;
      Tick latAfterCompletion;

      uint64_t completionDataReg;

      uint16_t *buffer_offset;

      // std::queue<PacketPtr> flushQueue;
      // std::queue<PacketPtr> invaQueue;

      enum ChannelState
      {
          Idle,
          AddressFetch,
          DescriptorFetch,
          // CacheFlush,
          // CacheInvalidate,
          DMARead,
          DMAWrite,
          CompletionWrite
      };

      ChannelState nextState;

    public:
      DMAChannel(AccMidgard *_ce, int cid);
      virtual ~DMAChannel();

      std::string
      name()
      {
          assert(ce);
          return ce->name() + csprintf("-chan%d", channelId);
      }

      virtual Tick read(PacketPtr pkt)
                      { panic("DMAChannel has no I/O access\n");}
      virtual Tick write(PacketPtr pkt)
                      { panic("DMAChannel has no I/O access\n"); }

      void channelRead(PacketPtr pkt, Addr daddr, int size);
      void channelWrite(PacketPtr pkt, Addr daddr, int size);

      bool readDone();
      uint8_t * getCopyBuffer();
      uint16_t * getCopyBuffer_offset();
      copy_engine_reg::DmaDesc *getCurDmaDesc();

      DrainState drain() override;
      void drainResume() override;

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;

      void continueProcessing();
      void restartStateMachine();

    private:
      void fetchDescriptor(Addr address);
      void fetchDescComplete();
      EventFunctionWrapper fetchCompleteEvent;

      void fetchNextAddr(Addr address);
      void fetchAddrComplete();
      EventFunctionWrapper addrCompleteEvent;

      void callAccel();
      void callAccelComplete();
      EventFunctionWrapper AccelCompleteEvent;

      void readCopyBytes();
      void readCopyBytesComplete();
      EventFunctionWrapper readCompleteEvent;

      void writeCopyBytes();
      void writeCopyBytesComplete();
      EventFunctionWrapper writeCompleteEvent;

      void writeCompletionStatus();
      void writeStatusComplete();
      EventFunctionWrapper statusCompleteEvent;

      void resetIntr();
      EventFunctionWrapper resetIntrEvent;
      // void cacheFlush();
      // void cacheFlushComplete();
      // EventFunctionWrapper cacheFlushEvent;

      // void cacheInvalidate();
      // void cacheInvalidateComplete();
      // EventFunctionWrapper cacheInvalidateEvent;

      void recvCommand();
      bool inDrain();
  };

  class AcceleratorState : public Packet::SenderState
  {
    public:
      Tick delay = 0;
      uint32_t block_index;
      Event *event = nullptr;
  };

  class AccController
  {
    private:
      int READ_latency = 0;
      u_int64_t time_stamp = 0;


      // SharedData* sharedData;
      // sem_t* semaphore;

      /// @brief The matrix is split into multiple different blocks
      /// Used for coarse granularity
      int buffer_num = 0;
      int buffer_size = 0;

    public:
      int set_index;
      /// @brief two channels for tow sets of input data.
      bool channel1_read = true;
      bool channel2_read = true;
      bool channel1_ready = false;
      bool channel2_ready = false;
      enum ControllerState
      {
        Idle,
        ReadData,
        WriteData,
        Computing
      };

      ControllerState Accstate;

      std::vector<DMAChannel*> *chan;
      std::vector<bool> chan_ready;
      int chan_number;

      /// @brief Functions used to send/store data
      std::vector<std::vector<u_int8_t>> local_buffer;

      /// @brief Functions used to read data through PCIe interface
      /// @param local_buffer:local buffer array
      /// @param block_index: target block index
      /// @param addr: the physical addr of the memory space
      /// @param size: the size moved from memory addr to the buffer
      /// @return
      void data_read(std::vector<std::vector<u_int8_t>> &local_buffer, uint64_t block_index, Addr addr, uint32_t size);

      PacketPtr createPacket(Addr addr, int size,
                            uint8_t *data, Request::Flags flags, MemCmd::Command cmd, Tick delay,
                            Event *event, uint32_t block_index_);

      bool receiveData();

      bool sendData();

      AccController(int buffer_num_, int buffer_size_, int READ_latency,
                    std::vector<DMAChannel*> *chan_);
      ~AccController();
  };

  /**
   * Port on the memory-side that receives responses.
   * Mostly just forwards requests to the controller
   */
  class MemDevicePort : public RequestPort
  {
    private:
      /// The object that owns this object (RTL)
      AccMidgard *controller;

      /// If we tried to send a packet and it was blocked, store it here
      //PacketPtr blockedPacket;

    public:
      /**
       * Constructor. Just calls the superclass constructor.
       */
      MemDevicePort(const std::string& name, AccMidgard *controller,
                    bool sram_):
          RequestPort(name, controller),
          controller(controller),
          sram(sram_),
          blockedRetry(false)
      {}

      uint32_t cacheline_size_;

      uint32_t recentData32;

      std::queue<PacketPtr> pending_req;

      bool sram;
      // if we are blocked due to a req retry
      bool blockedRetry;

      /**
       * Send a packet across this port. This is called by the controller and
       * all of the flow control is hanled in this function.
       *
       * @param packet to send.
       */
      void sendPacket(PacketPtr pkt, bool timing);

      /**
       * We check if we have any pending request and we try to send it
       *
       */
      void tick();

    protected:
      /**
       * Receive a timing response from the slave port.
       */
      bool recvTimingResp(PacketPtr pkt) override;

      /**
       * Called by the slave port if sendTimingReq was called on this
       * master port (causing recvTimingReq to be called on the slave
       * port) and was unsuccesful.
       */
      void recvReqRetry() override;

      /**
       * Called to receive an address range change from the peer slave
       * port. The default implementation ignores the change and does
       * nothing. Override this function in a derived class if the controller
       * needs to be aware of the address ranges, e.g. in an
       * interconnect component like a bus.
       */
      // void recvRangeChange() override;
  };


  private:

    struct DMAStats : public statistics::Group
    {
        DMAStats(statistics::Group *parent,
            const uint8_t& channel_count);

        statistics::Vector bytesCopied;
        statistics::Vector copiesProcessed;
    } DMAStats;

    // device registers
    copy_engine_reg::Regs regs;

    // Array of channels each one with regs/dma port/etc
    std::vector<DMAChannel*> chan;

    // RTL Wrapper
    AccController *RTL_Wrapper;

  public:

    MemDevicePort sramPort;
    MemDevicePort dramPort;

    PARAMS(AccMidgard);
    AccMidgard(const AccMidgardParams &params);
    ~AccMidgard();

    Port &getPort(const std::string &if_name,
            PortID idx = InvalidPortID) override;


    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    bool handleResponseMem(PacketPtr pkt, bool sram);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace gem5

#endif //__DEV_PCI_COPY_ENGINE_HH__
