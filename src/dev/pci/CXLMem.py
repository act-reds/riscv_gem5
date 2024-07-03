# Non-Volatile Dual In-line Memory Module Virtualization Implementation

from m5.objects.PciDevice import (
    PciDevice,
    PciMemBar,
)
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class CXLMem(PciDevice):
    type = "CXLMem"
    cxx_header = "dev/pci/CXL_mem.hh"
    cxx_class = "gem5::CXLMem"
    VendorID = 0x8086
    DeviceID = 0x1A38
    Revision = 0xA2  # CM2 stepping (newest listed)
    SubsystemID = 0
    SubsystemVendorID = 0
    Status = 0x0000
    SubClassCode = 0x08
    ClassCode = 0x80
    ProgIF = 0x00
    MaximumLatency = 0x00
    MinimumGrant = 0xFF
    InterruptLine = 0x20
    InterruptPin = 0x01

    BAR0 = PciMemBar(size="128KiB")

    ChanCnt = Param.UInt8(1, "Number of DMA channels that exist on device")
    XferCap = Param.MemorySize(
        "4KiB", "Number of bits of transfer size that are supported"
    )

    latBeforeBegin = Param.Latency(
        "20ns", "Latency after a DMA command is seen before it's proccessed"
    )
    latAfterCompletion = Param.Latency(
        "20ns",
        "Latency after a DMA command is complete before "
        "it's reported as such",
    )
