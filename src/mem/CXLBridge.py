# from MemObject import MemObject # MemObject is deprecated
from m5.objects.ClockedObject import ClockedObject
from m5.objects.PciBridge import PciBridge
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

# Root Complex configuration. Assume it has 3 root ports

PCI_ERR_SIZEOF = 0x48
PCI_ACS_SIZEOF = 0x8

GEN_PCIE_ROOT_PORT_AER_OFFSET = 0x100
CXL_UPSTREAM_PORT_AER_OFFSET = 0x100
CXL_DOWNSTREAM_PORT_AER_OFFSET = 0x100

GEN_PCIE_ROOT_PORT_ACS_OFFSET = GEN_PCIE_ROOT_PORT_AER_OFFSET + PCI_ERR_SIZEOF
CXL_ROOT_PORT_DVSEC_OFFSET = GEN_PCIE_ROOT_PORT_ACS_OFFSET + PCI_ACS_SIZEOF
CXL_UPSTREAM_PORT_DVSEC_OFFSET = CXL_UPSTREAM_PORT_AER_OFFSET + PCI_ERR_SIZEOF
CXL_DOWNSTREAM_PORT_DVSEC_OFFSET = (
    CXL_DOWNSTREAM_PORT_AER_OFFSET + PCI_ERR_SIZEOF
)


class CXLBridge(PciBridge):
    type = "CXLBridge"
    # abstract = True
    cxx_class = "gem5::CXLBridge"
    cxx_header = "mem/CXLBridge.hh"
    offset_dvsec_rp = Param.Int(
        CXL_ROOT_PORT_DVSEC_OFFSET, "the offset of DVSEC in root port"
    )
    offset_dvsec_up = Param.Int(
        CXL_UPSTREAM_PORT_DVSEC_OFFSET, "the offset of DVSEC in upstream port"
    )
    offset_dvsec_dp = Param.Int(
        CXL_DOWNSTREAM_PORT_DVSEC_OFFSET,
        "the offset of DVSEC in downstream port",
    )


class CXL_Root_Complex(CXLBridge):
    # type = 'Root_Complex'
    # #abstract = True
    # cxx_class = "gem5::Root_Complex"
    # cxx_header = 'mem/PciBridge.hh'
    PrimaryBusNumber = 0x00
    SecondaryBusNumber = 0x00
    SubordinateBusNumber = 0x00
    pci_bus = 0
    pci_dev1 = 4  # 4
    pci_dev2 = 5  # 5
    pci_dev3 = 6  # 6
    pci_func1 = 0
    pci_func2 = 0
    pci_func3 = 0

    VendorId = 0x8086
    DeviceId_upstream = 0x7075
    DeviceId1 = 0x9C90  # intel pci express root port 1
    DeviceId2 = 0x9C92  # intel pci express root port 2
    DeviceId3 = 0x9C94  # intel pci express root port 3
    ClassCode = 0x06
    SubClassCode = 0x04
    ProgIF = 0x00
    HeaderType = 0x01
    Revision = 0x00
    BIST = 0x00
    Command = 0x0407  # Enable bus mastering, intx gen,memory address space decoder, io address space decoder
    Status = 0x0010  # Indicate that capability ptr has been implemented
    SecondaryStatus = 0x0000
    BridgeControl = 0x0000
    InterruptPin = 0x01
    InterruptLine = 0x20
    CapabilityPointer = 0xD0
    PXCAPBaseOffset = 0xD0
    PXCAPNextCapability = 0x00  # implement only PCIe capability
    PXCAPCapId = 0x10  # PCIe capability
    PXCAPCapabilities = 0x0041  # 4 indicates PCIe root port
    PXCAPDevCapabilities = 0x00000241  # Max payload 256B , L0 acceptable latency: 64-128ns. L1 acceptable latency 1-2 us
    PXCAPDevCtrl = (
        0x1020  # Max payload size : 256B. Max read request size : 256B
    )
    PXCAPLinkCap = 0x01009011  # Max link width : 6'b1 , Max link speed : 2.5 Gbps , ASPM:00 , L0s exit latency: 64-128 ns , L1 exit latency 1-2 us. Port number : 1.
    PXCAPDevStatus = 0x0000
    PXCAPLinkCtrl = 0x0008  # Root ports:128 B Read Completion Boundary(RCB) support , ASPM disabled=> Bits 1 and 0 are just 0 .
    PXCAPLinkStatus = (
        0x0011  # Negotiated link width : 1 , Negotiated link speed : 2.5 Gbps
    )

    # Need to implement (slot??) registers and root registers for a root port. Assume that drivers are not needed for Root Port detection and Configuration (Assume that kernel bus drivers do this).
    # Also need to assign bridge params to the PciBridge. So , master-port, slave-port and latencies need to be assigned. Assume bridges can raise interrupts (not required ??).
    PXCAPRootStatus = 0x00000000  # PME related stuff. Ignore for now
    PXCAPRootControl = 0x0000  # PME related stuff again. Ignore for now.


class CXLSwitch(CXLBridge):
    # type = 'PciESwitch'
    # #abstract = True
    # cxx_class = "gem5::PciESwitch"
    # cxx_header = 'mem/PciBridge.hh'
    is_switch = 1
    pci_dev1 = 0
    pci_func1 = 0
    pci_dev2 = 1
    pci_func2 = 0
    pci_dev3 = 2
    pci_func3 = 0
    DeviceId1 = 0x8233
    DeviceId2 = 0x8233
    DeviceId3 = 0x8233

    VendorId = 0x104C  # Vendor is texas instruments
    Status = 0x0010
    Command = 0x0407
    ClassCode = 0x06
    SubClassCode = 0x04
    ProgIF = 0

    Revision = 0
    BIST = 0
    HeaderType = 0x01
    LatencyTimer = 0
    CacheLineSize = 0
    Bar0 = 0x00000000
    Bar1 = 0x00000000
    BAR0Size = "0B"
    BAR1Size = "0B"
    SecondaryLatencyTimer = 0

    CapabilityPointer = 0xD0
    InterruptPin = 0x01
    InterruptLine = 0x20
    BridgeControl = 0x0000
    SecondaryStatus = 0x0000
    ###PCIe capability fields (8 datawords for PCI Bridge)
    PXCAPBaseOffset = 0xD0
    PXCAPNextCapability = 0x00
    PXCAPCapId = 0x10

    PXCAPDevCtrl = 0x1020
    PXCAPDevStatus = 0x0000
    PXCAPLinkCap = 0x01009011
    PXCAPLinkCtrl = 0x0008
    PXCAPLinkStatus = 0x0011
    PXCAPDevCap2 = 0x00000000
    PXCAPDevCtrl2 = 0x00000000
    PXCAPRootStatus = 0x00000000
    PXCAPRootControl = 0x0000
