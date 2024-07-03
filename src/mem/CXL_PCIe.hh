/**
 * @file
 * Declaration of a memory-mapped bridge that connects a master
 * and a slave through a request and response queue.
 */

#ifndef __CXL_PCIE_HH__
#define __CXL_PCIE_HH__

#define CXL_VENDOR_ID 0x1e98
#define PCIE_DVSEC_HEADER1_OFFSET 0x4 /* Offset from start of extend cap */
#define PCIE_DVSEC_ID_OFFSET 0x8
#define PCIE_CXL_DEVICE_DVSEC_LENGTH 0x38
#define PCIE_CXL1_DEVICE_DVSEC_REVID 0
#define PCIE_CXL2_DEVICE_DVSEC_REVID 1
#define EXTENSIONS_PORT_DVSEC_LENGTH 0x28
#define EXTENSIONS_PORT_DVSEC_REVID 0
#define GPF_PORT_DVSEC_LENGTH 0x10
#define GPF_PORT_DVSEC_REVID  0
#define GPF_DEVICE_DVSEC_LENGTH 0x10
#define GPF_DEVICE_DVSEC_REVID 0
#define PCIE_FLEXBUS_PORT_DVSEC_LENGTH_2_0 0x14
#define PCIE_FLEXBUS_PORT_DVSEC_REVID_2_0  1
#define REG_LOC_DVSEC_LENGTH 0x24
#define REG_LOC_DVSEC_REVID  0

namespace gem5
{
enum dvsec_id {
    PCIE_CXL_DEVICE_DVSEC      = 0,
    NON_CXL_FUNCTION_MAP_DVSEC = 2,
    EXTENSIONS_PORT_DVSEC      = 3,
    GPF_PORT_DVSEC             = 4,
    GPF_DEVICE_DVSEC           = 5,
    PCIE_FLEXBUS_PORT_DVSEC    = 7,
    REG_LOC_DVSEC              = 8,
    MLD_DVSEC                  = 9,
    CXL20_MAX_DVSEC
};

typedef struct DVSECHeader {
    uint32_t cap_hdr = 0;
    uint32_t dv_hdr1 = 0;
    uint16_t dv_hdr2 = 0;
} DVSECHeader;

/* CXL 2.0 - 8.1.3 (ID 0001) */
//PCIe DVSEC for CXL Device
typedef struct CXLDVSECDevice {
    DVSECHeader hdr = {0,0,0};
    uint16_t cap = 0;
    uint16_t ctrl = 0;
    uint16_t status = 0;
    uint16_t ctrl2 = 0;
    uint16_t status2 = 0;
    uint16_t lock = 0;
    uint16_t cap2 = 0;
    uint32_t range1_size_hi = 0;
    uint32_t range1_size_lo = 0;
    uint32_t range1_base_hi = 0;
    uint32_t range1_base_lo = 0;
    uint32_t range2_size_hi = 0;
    uint32_t range2_size_lo = 0;
    uint32_t range2_base_hi = 0;
    uint32_t range2_base_lo = 0;
} CXLDVSECDevice; //0x38 size

/* CXL 2.0 - 8.1.5 (ID 0003) */
//The PCIe configuration space of a CXL 2.0 Root Port,
//CXL Downstream Switch Port and CXL Upstream Switch Port
typedef struct CXLDVSECPortExtensions {
    DVSECHeader hdr = {0,0,0};
    uint16_t status = 0;
    uint16_t control = 0;
    uint8_t alt_bus_base = 0;
    uint8_t alt_bus_limit = 0;
    uint16_t alt_memory_base = 0;
    uint16_t alt_memory_limit = 0;
    uint16_t alt_prefetch_base = 0;
    uint16_t alt_prefetch_limit = 0;
    uint32_t alt_prefetch_base_high = 0;
    uint32_t alt_prefetch_limit_high = 0;
    uint32_t rcrb_base = 0;
    uint32_t rcrb_base_high = 0;
} CXLDVSECPortExtensions;//0x28 size

#define PORT_CONTROL_OFFSET          0xc
#define PORT_CONTROL_UNMASK_SBR      1
#define PORT_CONTROL_ALT_MEMID_EN    4

/* CXL 2.0 - 8.1.6 GPF DVSEC (ID 0004) */
//The PCIe configuration space of CXL Downstream Switch Ports
//and CXL 2.0 capable Root Ports
typedef struct CXLDVSECPortGPF {
    DVSECHeader hdr = {0,0,0};
    uint16_t rsvd = 0;
    uint16_t phase1_ctrl = 0;
    uint16_t phase2_ctrl = 0;
} CXLDVSECPortGPF;//0x10 size

/* CXL 2.0 - 8.1.7 GPF DVSEC for CXL Device */
//Device 0, Function 0 of a CXL.mem capable devices must
//implement this DVSEC capability
typedef struct CXLDVSECDeviceGPF {
    DVSECHeader hdr = {0,0,0};
    uint16_t phase2_duration = 0;
    uint32_t phase2_power = 0;
} CXLDVSECDeviceGPF;//0x10 size

/* CXL 2.0 - 8.1.8/8.2.1.3 Flex Bus DVSEC (ID 0007) */
// The DVSEC associated with a CXL 2.0 device shall be accessible via Device 0, Function
// 0 of the device. Upstream Switch Ports, Downstream Switch Ports and CXL 2.0 Root
// Ports shall implement this DVSEC in the Configuration Space associated with the Port.
typedef struct CXLDVSECPortFlexBus {
    DVSECHeader hdr = {0,0,0};
    uint16_t cap = 0;
    uint16_t ctrl = 0;
    uint16_t status = 0;
    uint32_t rcvd_mod_ts_data_phase1 = 0;
} CXLDVSECPortFlexBus;//0x14 size

/* CXL 2.0 - 8.1.9 Register Locator DVSEC (ID 0008) */
// The PCIe configuration space of a CXL 2.0 Root Port, CXL Downstream Switch Port, CXL
// Upstream Switch Port and CXL 2.0 Device must implement this DVSEC capability.
typedef struct CXLDVSECRegisterLocator {
    DVSECHeader hdr = {0,0,0};
    uint16_t rsvd = 0;
    uint32_t reg0_base_lo = 0;
    uint32_t reg0_base_hi = 0;
    uint32_t reg1_base_lo = 0;
    uint32_t reg1_base_hi = 0;
    uint32_t reg2_base_lo = 0;
    uint32_t reg2_base_hi = 0;
} CXLDVSECRegisterLocator;//0x24 size

/* BAR Equivalence Indicator */
#define BEI_BAR_10H 0
#define BEI_BAR_14H 1
#define BEI_BAR_18H 2
#define BEI_BAR_1cH 3
#define BEI_BAR_20H 4
#define BEI_BAR_24H 5

/* Register Block Identifier */
#define RBI_EMPTY          0
#define RBI_COMPONENT_REG  (1 << 8)
#define RBI_BAR_VIRT_ACL   (2 << 8)
#define RBI_CXL_DEVICE_REG (3 << 8)
} //end gem5 space
#endif //__MEM_PCIBRIDGE_HH__
