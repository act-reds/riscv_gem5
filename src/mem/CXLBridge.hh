
/**
 * @file
 * Declaration of a memory-mapped bridge that connects a master
 * and a slave through a request and response queue.
 */

#ifndef __MEM_CXLBRIDGE_HH__
#define __MEM_CXLBRIDGE_HH__


#include <deque>

#include "base/types.hh"
#include "dev/pci/host.hh"
#include "dev/pci/pcireg.h"
#include "dev/pci/types.hh"
#include "mem/PciBridge.hh"
#include "mem/CXL_PCIe.hh"
#include "params/CXLBridge.hh"

namespace gem5
{

enum dvsec_type {
    CXL2_TYPE3_DEVICE,
    CXL2_ROOT_PORT,
    CXL2_UPSTREAM_PORT,
    CXL2_DOWNSTREAM_PORT
};

class CXLBridge ;

// class cxl_config_class: public config_class
// {
//   public:
//   // 4KB extended configuration spcace.
//   // But only for DVSEC.
//   // Config and PCIe capability are used previous defination
//   uint8_t storage_dvsec[PCIE_CONFIG_SIZE] = {0};
//   uint16_t dvsec_header;
//   // find the header of previous DVSEC
//   uint16_t pcie_find_capability_list(uint32_t cap_id, uint16_t *prev_p);
//   // Update the pointer to next dvsec sector
//   void dvsec_header_update(uint16_t prev, uint16_t next);

//   //Replace original write/read with new functions
//   //Which support extended DVSEC reading/writing
//   Tick readConfig(PacketPtr pkt);
//   Tick writeConfig(PacketPtr pkt);


//   cxl_config_class(int pci_bus , int pci_dev, int pci_func
//   , uint8_t root_port_number):
//   config_class(pci_bus , pci_dev, pci_func, root_port_number)
//   {}
// } ;


/**
 * CXL Bridge class based on PciBridge
 */
class CXLBridge: public PciBridge
{
  protected:
    void pcie_set_long(uint8_t *config, uint32_t val);
    void pcie_set_word(uint8_t *config, uint16_t val);
    void pcie_add_capability(config_class * storage_ptr,uint16_t cap_id, uint8_t cap_ver,
                         uint16_t offset, uint16_t size);
  public:
    int rp_offset_dvsec;
    int up_offset_dvsec;
    int dp_offset_dvsec;
    typedef CXLBridgeParams Params;
    void CXL_init_dvsec(config_class * storage_ptr, int dvsec_offset,
                        enum dvsec_type type);

    void CXL_creat_dvsec(config_class * storage_ptr, enum dvsec_type cxl_dev_type, uint16_t length,
                         enum dvsec_id type, uint8_t rev, uint8_t *body);

    CXLBridge(const Params &p);
    ~CXLBridge() ;
};
}//gem5
#endif //__MEM_PCIBRIDGE_HH__
