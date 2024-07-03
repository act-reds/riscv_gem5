/**
 * @file
 * Implementation of a memory-mapped bridge that connects a master
 * and a slave through a request and response queue.
 */
#include "base/inifile.hh"
#include "base/intmath.hh"

// #include "base/misc.hh" # changed to "base/logging.hh"
#include "mem/PciBridge.hh"
#include "mem/CXLBridge.hh"
#include "mem/CXL_PCIe.hh"

#include "base/logging.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "debug/PciBridge.hh"
#include "debug/CXLBridge.hh"
#include "debug/ReadWriteConfig.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/CXLBridge.hh"
#include "params/Bridge.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"

using namespace std;

namespace gem5
{
    CXLBridge::CXLBridge(const Params &p):
    PciBridge(p),rp_offset_dvsec(p.offset_dvsec_rp),
    up_offset_dvsec(p.offset_dvsec_up),dp_offset_dvsec(p.offset_dvsec_dp)
    {
        DPRINTF(CXLBridge,"CXLBridge initialization!\n");
        // dynamic cast it to CXL class, and then init the dvsec
        // If it is switch, it looks like uptream + 3 downstream
        // otherwise cxl, upstream + 3 root port
        if(is_switch)
        {
            CXL_init_dvsec(storage_ptr1, rp_offset_dvsec, CXL2_ROOT_PORT);
            CXL_init_dvsec(storage_ptr2, rp_offset_dvsec, CXL2_ROOT_PORT);
            CXL_init_dvsec(storage_ptr3, rp_offset_dvsec, CXL2_ROOT_PORT);
        }
        else
        {
            CXL_init_dvsec(storage_ptr1, dp_offset_dvsec, CXL2_DOWNSTREAM_PORT);
            CXL_init_dvsec(storage_ptr2, dp_offset_dvsec, CXL2_DOWNSTREAM_PORT);
            CXL_init_dvsec(storage_ptr3, dp_offset_dvsec, CXL2_DOWNSTREAM_PORT);
        }
        CXL_init_dvsec(storage_ptr4, up_offset_dvsec, CXL2_UPSTREAM_PORT);
    }

    void CXLBridge::pcie_set_long(uint8_t *config, uint32_t val)
    {
        __builtin_memcpy(config, &val, sizeof(val));
    }

    void CXLBridge::pcie_set_word(uint8_t *config, uint16_t val)
    {
        __builtin_memcpy(config, &val, sizeof(val));
    }

    void CXLBridge::pcie_add_capability(config_class * storage_ptr,uint16_t cap_id, uint8_t cap_ver,
                            uint16_t offset, uint16_t size)
    {
        DPRINTF(CXLBridge, "Add DVSEC registers here: %#x\n",offset);
        assert(offset >= PCI_CONFIG_SPACE_SIZE);
        assert(offset < (uint16_t)(offset + size));
        assert((uint16_t)(offset + size) <= PCIE_CONFIG_SPACE_SIZE);
        assert(size >= 8);

        if (offset != PCI_CONFIG_SPACE_SIZE) {
            uint16_t prev;

            /*
            * 0xffffffff is not a valid cap id (it's a 16 bit field). use
            * internally to find the last capability in the linked list.
            */
            storage_ptr->pcie_find_capability_list(0xffffffff, &prev);
            assert(prev >= PCI_CONFIG_SPACE_SIZE);

            // modify the current next pointer
            storage_ptr->dvsec_header_update(prev, offset);
        }

        // Add the PCIe extended capbility header
        pcie_set_long(storage_ptr->storage_dvsec + storage_ptr->dvsec_header, PCI_EXT_CAP(cap_id, cap_ver, 0));
    }

    void CXLBridge::CXL_creat_dvsec(config_class * storage_ptr, enum dvsec_type cxl_dev_type,
                                    uint16_t length, enum dvsec_id type, uint8_t rev,
                                    uint8_t *body)
    {
        std::string DVSEC_ = "";
        switch (type)
        {
        case PCIE_CXL_DEVICE_DVSEC:
            DVSEC_ = "CXL2_TYPE3_DEVICE";
            break;
        case NON_CXL_FUNCTION_MAP_DVSEC:
            DVSEC_ = "NON_CXL_FUNCTION_MAP_DVSEC";
            break;
        case EXTENSIONS_PORT_DVSEC:
            DVSEC_ = "EXTENSIONS_PORT_DVSEC";
            break;
        case GPF_PORT_DVSEC:
            DVSEC_ = "GPF_PORT_DVSEC";
            break;
        case GPF_DEVICE_DVSEC:
            DVSEC_ = "GPF_DEVICE_DVSEC";
            break;
        case PCIE_FLEXBUS_PORT_DVSEC:
            DVSEC_ = "PCIE_FLEXBUS_PORT_DVSEC";
            break;
        case REG_LOC_DVSEC:
            DVSEC_ = "REG_LOC_DVSEC";
            break;
        case MLD_DVSEC:
            DVSEC_ = "MLD_DVSEC";
            break;
        case CXL20_MAX_DVSEC:
            DVSEC_ = "CXL20_MAX_DVSEC";
            break;
        }
        DPRINTF(CXLBridge,"set DVSEC to %s field\n",DVSEC_);
        /* Create the DVSEC in the MCFG space */
        int header_offset = storage_ptr->dvsec_header;

        // the PCI Express Extended Capability Header
        pcie_add_capability(storage_ptr, PCI_EXT_CAP_ID_DVSEC, 1, header_offset, length);

        // the Designated Vendor-Specific Header 1
        pcie_set_long(storage_ptr->storage_dvsec + header_offset + PCIE_DVSEC_HEADER1_OFFSET,
                    (length << 20) | (rev << 16) | CXL_VENDOR_ID);

        // the Designated Vendor-Specific Header 2
        pcie_set_word(storage_ptr->storage_dvsec + header_offset + PCIE_DVSEC_ID_OFFSET, type);

        // Cppy the rest data into the DVSEC sector
        memcpy(storage_ptr->storage_dvsec + header_offset + sizeof(DVSECHeader),
           body + sizeof(DVSECHeader),
           length - sizeof(DVSECHeader));

        // After adding new DVSEC sector, shift the ptr to the tail
        storage_ptr->dvsec_header += length;
    }


    void CXLBridge::CXL_init_dvsec(config_class * storage_ptr, int dvsec_offset,
                                  enum dvsec_type type)
    {
        // initialize the offset of DVSEC for rp,up,dp
        storage_ptr->dvsec_header = dvsec_offset;

        // Instantiate some DVSEC regs
        CXLDVSECPortExtensions CXLDVSECPortExtensions_instance_root = { 0 };
        CXLDVSECPortGPF CXLDVSECPortGPF_instance_root = {
                    .rsvd        = 0,
                    .phase1_ctrl = 1, /* 1μs timeout */
                    .phase2_ctrl = 1, /* 1μs timeout */
                };
        CXLDVSECPortFlexBus CXLDVSECPortFlexBus_instance_root = {
                    .cap                     = 0x26, /* IO, Mem, non-MLD */
                    .ctrl                    = 0x2,
                    .status                  = 0x26, /* same */
                    .rcvd_mod_ts_data_phase1 = 0xef,
                };
        CXLDVSECRegisterLocator CXLDVSECRegisterLocator_instance_root = {
                    .rsvd         = 0,
                    .reg0_base_lo = RBI_COMPONENT_REG | CXL_COMPONENT_REG_BAR_IDX,
                    .reg0_base_hi = 0,
                };
        CXLDVSECPortExtensions CXLDVSECPortExtensions_instance_up = {
                    .status = 0x1, /* Port Power Management Init Complete */
                };
        CXLDVSECPortFlexBus CXLDVSECPortFlexBus_instance_up = {
                    .cap                     = 0x27, /* Cache, IO, Mem, non-MLD */
                    .ctrl                    = 0x27, /* Cache, IO, Mem */
                    .status                  = 0x26, /* same */
                    .rcvd_mod_ts_data_phase1 = 0xef, /* WTF? */
                };
        CXLDVSECRegisterLocator CXLDVSECRegisterLocator_instance_up = {
                    .rsvd         = 0,
                    .reg0_base_lo = RBI_COMPONENT_REG | CXL_COMPONENT_REG_BAR_IDX,
                    .reg0_base_hi = 0,
                };
        CXLDVSECPortExtensions CXLDVSECPortExtensions_instance_dw = { 0 };
        CXLDVSECPortFlexBus CXLDVSECPortFlexBus_instance_dw = {
                    .cap                     = 0x27, /* Cache, IO, Mem, non-MLD */
                    .ctrl                    = 0x02, /* IO always enabled */
                    .status                  = 0x26, /* same */
                    .rcvd_mod_ts_data_phase1 = 0xef, /* WTF? */
                };
        CXLDVSECPortGPF CXLDVSECPortGPF_instance_dw = {
                    .rsvd        = 0,
                    .phase1_ctrl = 1, /* 1μs timeout */
                    .phase2_ctrl = 1, /* 1μs timeout */
                };
        CXLDVSECRegisterLocator CXLDVSECRegisterLocator_instance_dw = {
                    .rsvd         = 0,
                    .reg0_base_lo = RBI_COMPONENT_REG | CXL_COMPONENT_REG_BAR_IDX,
                    .reg0_base_hi = 0,
                };
        uint8_t *dvsec;
        switch (type) {// Here we only discuss the Bridge part
            case CXL2_ROOT_PORT:
                DPRINTF(CXLBridge,"Init DVSEC for CXL2_ROOT_PORT\n");
                dvsec = (uint8_t *)&CXLDVSECPortExtensions_instance_root;
                CXL_creat_dvsec(storage_ptr, CXL2_ROOT_PORT,
                               EXTENSIONS_PORT_DVSEC_LENGTH,
                               EXTENSIONS_PORT_DVSEC,
                               EXTENSIONS_PORT_DVSEC_REVID, dvsec);

                dvsec = (uint8_t *)&CXLDVSECPortGPF_instance_root;
                CXL_creat_dvsec(storage_ptr, CXL2_ROOT_PORT,
                               GPF_PORT_DVSEC_LENGTH, GPF_PORT_DVSEC,
                               GPF_PORT_DVSEC_REVID, dvsec);

                dvsec = (uint8_t *)&CXLDVSECPortFlexBus_instance_root;
                CXL_creat_dvsec(storage_ptr, CXL2_ROOT_PORT,
                               PCIE_FLEXBUS_PORT_DVSEC_LENGTH_2_0,
                               PCIE_FLEXBUS_PORT_DVSEC,
                               PCIE_FLEXBUS_PORT_DVSEC_REVID_2_0, dvsec);

                dvsec = (uint8_t *)&CXLDVSECRegisterLocator_instance_root;
                CXL_creat_dvsec(storage_ptr, CXL2_ROOT_PORT,
                               REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC,
                               REG_LOC_DVSEC_REVID, dvsec);
                break;
            case CXL2_UPSTREAM_PORT:
                DPRINTF(CXLBridge,"Init DVSEC for CXL2_UPSTREAM_PORT\n");
                dvsec = (uint8_t *)&CXLDVSECPortExtensions_instance_up;
                CXL_creat_dvsec(storage_ptr, CXL2_UPSTREAM_PORT,
                               EXTENSIONS_PORT_DVSEC_LENGTH,
                               EXTENSIONS_PORT_DVSEC,
                               EXTENSIONS_PORT_DVSEC_REVID, dvsec);

                dvsec = (uint8_t *)&CXLDVSECPortFlexBus_instance_up;
                CXL_creat_dvsec(storage_ptr, CXL2_UPSTREAM_PORT,
                               PCIE_FLEXBUS_PORT_DVSEC_LENGTH_2_0,
                               PCIE_FLEXBUS_PORT_DVSEC,
                               PCIE_FLEXBUS_PORT_DVSEC_REVID_2_0, dvsec);

                dvsec = (uint8_t *)&CXLDVSECRegisterLocator_instance_up;
                CXL_creat_dvsec(storage_ptr, CXL2_UPSTREAM_PORT,
                               REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC,
                               REG_LOC_DVSEC_REVID, dvsec);
                break;
            case CXL2_DOWNSTREAM_PORT:
                DPRINTF(CXLBridge,"Init DVSEC for CXL2_DOWNSTREAM_PORT\n");
                dvsec = (uint8_t *)&CXLDVSECPortExtensions_instance_dw;
                CXL_creat_dvsec(storage_ptr, CXL2_DOWNSTREAM_PORT,
                               EXTENSIONS_PORT_DVSEC_LENGTH,
                               EXTENSIONS_PORT_DVSEC,
                               EXTENSIONS_PORT_DVSEC_REVID, dvsec);

                dvsec = (uint8_t *)&CXLDVSECPortFlexBus_instance_dw;
                CXL_creat_dvsec(storage_ptr, CXL2_DOWNSTREAM_PORT,
                               PCIE_FLEXBUS_PORT_DVSEC_LENGTH_2_0,
                               PCIE_FLEXBUS_PORT_DVSEC,
                               PCIE_FLEXBUS_PORT_DVSEC_REVID_2_0, dvsec);

                dvsec = (uint8_t *)&CXLDVSECPortGPF_instance_dw;
                CXL_creat_dvsec(storage_ptr, CXL2_DOWNSTREAM_PORT,
                               GPF_PORT_DVSEC_LENGTH, GPF_PORT_DVSEC,
                               GPF_PORT_DVSEC_REVID, dvsec);

                dvsec = (uint8_t *)&CXLDVSECRegisterLocator_instance_dw;
                CXL_creat_dvsec(storage_ptr, CXL2_DOWNSTREAM_PORT,
                               REG_LOC_DVSEC_LENGTH, REG_LOC_DVSEC,
                               REG_LOC_DVSEC_REVID, dvsec);
                break;
            default:
                break;
        }
    }


    CXLBridge::~CXLBridge()
    {
        delete storage_ptr1;
        delete storage_ptr2;
        delete storage_ptr3;
        delete storage_ptr4;
    }

} // gem5
