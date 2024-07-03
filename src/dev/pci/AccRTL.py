# Non-Volatile Dual In-line Memory Module Virtualization Implementation

from m5.objects.PciDevice import (
    PciDevice,
    PciMemBar,
)
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class AccRTL(PciDevice):
    type = "AccRTL"
    cxx_header = "dev/pci/AccRTL.hh"
    cxx_class = "gem5::AccRTL"
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

    dram_port = RequestPort("Regular Speed to DRAM, sends requests")
    sram_port = RequestPort("Regular Speed to SRAM, sends requests")

    burstSize = Param.UInt32(128, "Number of bytes in a burst")

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

    SystolicArray_Length = Param.UInt32(64, "Number of bytes in a burst")
    SystolicArray_Width = Param.UInt32(16, "Number of bytes in a burst")
