/* @file
 * Non-Volatile Dual In-line Memory Module Virtualization Implementation
 */

#ifndef __DEV_PCI_ACCRTL_HH__
#define __DEV_PCI_ACCRTL_HH__

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
#include "params/AccRTL.hh"
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

struct AccRTLParams;

class AccRTL : public PciDevice
{
  class DMAChannel : public Drainable, public Serializable
  {
    private:
      AccRTL *ce;
      copy_engine_reg::ChanRegs  cr;
      int channelId;
      copy_engine_reg::DmaDesc *curDmaDesc;
      uint8_t *copyBuffer;

      const int burstSize;

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
      DMAChannel(AccRTL *_ce, int cid);
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

      void recvCommand();
      bool inDrain();
  };

  class AccController
  {
  private:
      const static int ARRAY_SIZE_ = 16;
      int Matrix_lenth = 256;
      int buffer_length = 32;
      int READ_latency_Matrix = 0;
      int row_index_inner = 0;
      u_int64_t time_stamp = 0;

      // Shared Memory space with child process
      const char* shmName = "/mySharedMemory";
      const char* semName = "/mySemaphore";

      // Shared data structure between parent process and child process
      struct SharedData{
          bool permission;
          control_bit instruction;
          bool ready;
          int row_index;  // 0: continue, 1: new data, 2: terminate
          int inp_west[ARRAY_SIZE_];
          int inp_north[ARRAY_SIZE_];
          u_int64_t time_stamp;
          u_int64_t time_last;
          long result[ARRAY_SIZE_];
      };

      SharedData* sharedData;
      sem_t* semaphore;

      // Struct to represent the input for a single time step
      struct TimeStepInput{
          std::vector<int> matrixA_row;
          std::vector<int> matrixB_col;
      };


      /// @brief The matrix is split into multiple different blocks
      int block_index = 0;
      int row_index_block = 0;
      int col_index_block = 0;

  public:
      int set_index = 0;
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

      AccRTL *AccWrapper;

      std::vector<DMAChannel*> *chan;
      std::vector<bool> chan_ready;
      int chan_number;

      int ARRAY_SIZE;
      /// @brief Functions used to send/store data
      std::vector<TimeStepInput> inputFlow;
      std::vector<std::vector<long int>> local_buffer;

      /// @brief he following data is for testing
      std::vector<std::vector<int>> A;
      std::vector<std::vector<int>> B;
      std::vector<std::vector<long int>> C;

      std::vector<std::vector<int>> A_channel1;
      std::vector<std::vector<int>> A_channel2;
      std::vector<std::vector<int>> B_channel1;
      std::vector<std::vector<int>> B_channel2;
      std::vector<std::vector<long int>> C_channel;

      /// @brief Convert the SystolicArray into suitable format
      /// @param A
      /// @param B
      /// @param inputFlow
      void convertToSystolicInput(const std::vector<std::vector<int>>& A, const std::vector<std::vector<int>>& B, std::vector<TimeStepInput> &inputFlow);

      /// @brief helper function used to call function of RTL executable
      /// @param control_function
      void function_caller(void (AccController::*control_function)());
      void read_data();
      void read_finish();
      void start_function();
      void read_result();
      void check_res();
      void done();
      void reset();
      void pause();

      /// @brief Functions used to read data through PCIe interface
      /// @param sub_matrix_A
      /// @param matrix_A
      /// @param row_index
      /// @param block_index
      /// @return
      int data_read_row(std::vector<std::vector<int>> &sub_matrix_A, std::vector<std::vector<int>> &matrix_A, uint64_t row_index, uint64_t block_index);
      int data_read_column(std::vector<std::vector<int>> &sub_matrix_B, const std::vector<std::vector<int>> &matrix_B, uint64_t col_index, uint64_t block_index);
      void Matrix_subtore(const std::vector<std::vector<long int>> &sub_Matrix, std::vector<std::vector<long int>> &Matrix, uint64_t row_index, uint64_t col_index);
      void Matrix_store(const std::vector<std::vector<int>> &A, const std::vector<std::vector<int>> &B, const std::vector<std::vector<long int>> &C);

      void Matrix_calculation();

      bool receiveData();
      void sendData();

      // Lots of function to be deleted
      void state_processing();
      void state_controller();
      void start_computation_kernel();

      void load_data(std::vector<std::vector<int>> &A_channel,
                    std::vector<std::vector<int>> &B_channel);
      void loaddata_complete(int channel_index);
      EventFunctionWrapper LoadCompleteEvent_1;
      EventFunctionWrapper LoadCompleteEvent_2;

      void store_data();
      void storedata_complete();
      EventFunctionWrapper StoreCompleteEvent;

      void computing();

      AccController(AccRTL *_ce, int Matrix_lenth_, int buffer_length_, int READ_latency_Matrix,
                    std::vector<DMAChannel*> *chan_,
                    int row_index_, u_int64_t time_stamp);
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
      AccRTL *controller;

      /// If we tried to send a packet and it was blocked, store it here
      //PacketPtr blockedPacket;

    public:
      /**
       * Constructor. Just calls the superclass constructor.
       */
      MemDevicePort(const std::string& name, AccRTL *controller,
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

        statistics::Vector bytesCopied_read;
        statistics::Vector copiesProcessed_read;
        statistics::Vector bytesCopied_write;
        statistics::Vector copiesProcessed_write;
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

    int AccLatency;

    PARAMS(AccRTL);
    AccRTL(const AccRTLParams &params);
    ~AccRTL();

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
