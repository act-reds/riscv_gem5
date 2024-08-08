# Copyright (c) 2021 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
This example runs a simple linux boot. It uses the 'riscv-disk-img' resource.
It is built with the sources in `src/riscv-fs` in [gem5 resources](
https://github.com/gem5/gem5-resources).

Characteristics
---------------

* Runs exclusively on the RISC-V ISA with the classic caches
* Assumes that the kernel is compiled into the bootloader
* Automatically generates the DTB file
* Will boot but requires a user to login using `m5term` (username: `root`,
  password: `root`)
"""

import argparse

from m5.objects import *
from m5.objects import (
    AccRTL,
    AddrRange,
    BadAddr,
    Bridge,
    CowDiskImage,
    CXL_Root_Complex,
    CXLMem,
    Frequency,
    GenericRiscvPciHost,
    HiFive,
    IdeController,
    IGbE_e1000,
    IGbE_pcie,
    IOXBar,
    PCIELink,
    PMAChecker,
    Port,
    RawDiskImage,
    RiscvBootloaderKernelWorkload,
    RiscvMmioVirtIO,
    RiscvRTC,
    VirtIOBlock,
    VirtIORng,
)

from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    BinaryResource,
    DiskImageResource,
    obtain_resource,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires


def add_pcie(board):
    switch_up_lanes = 4
    lanes = 4  # 1
    cacheline_size = 64
    replay_buffer_size = 64

    # Declare PCIe links to connect the root complex
    board.pcie_switch = PCIELink(
        lanes=switch_up_lanes,
        speed="32Gbps",
        mps=cacheline_size,
        max_queue_size=replay_buffer_size,
        debug_flag=False,
    )
    board.pcie_0 = PCIELink(
        lanes=lanes,
        speed="32Gbps",
        mps=cacheline_size,
        max_queue_size=replay_buffer_size,
        debug_flag=False,
    )
    board.pcie_1 = PCIELink(
        lanes=lanes,
        speed="32Gbps",
        mps=cacheline_size,
        max_queue_size=replay_buffer_size,
        debug_flag=False,
    )

    board.Root_Complex = CXL_Root_Complex(
        is_transmit=False,
        req_size=64,
        resp_size=64,
        delay="150ns",
    )

    board.Root_Complex.offset_dvsec_rp = 0x100
    board.Root_Complex.offset_dvsec_up = 0x100
    board.Root_Complex.offset_dvsec_dp = 0x100
    # board.Root_Complex.slave = board.get_cache_hierarchy().get_mem_side_port()
    board.Root_Complex.slave = board.iobus.mem_side_ports
    # board.Root_Complex.slave = board.bridge.mem_side_port
    board.Root_Complex.slave_dma1 = board.pcie_switch.upstreamMaster
    board.Root_Complex.slave_dma2 = board.pcie_0.upstreamMaster
    board.Root_Complex.slave_dma3 = board.pcie_1.upstreamMaster
    board.Root_Complex.master_dma = board.iobus.cpu_side_ports
    board.Root_Complex.master1 = board.pcie_switch.upstreamSlave
    board.Root_Complex.master2 = board.pcie_0.upstreamSlave
    board.Root_Complex.master3 = board.pcie_1.upstreamSlave
    board.Root_Complex.host = board.platform.pci_host

    board.ethernet2 = IGbE_e1000(
        pci_bus=6,
        pci_dev=0,
        pci_func=0,
        InterruptLine=0x1E,
        InterruptPin=4,
        root_port_number=1,
        is_invisible=0,
    )
    board.ethernet2.pio = board.pcie_0.downstreamMaster
    board.ethernet2.dma = board.pcie_0.downstreamSlave
    board.ethernet2.host = board.platform.pci_host

    # board.ide = IdeController(
    #     disks=[],
    #     pci_bus=6,
    #     pci_dev=0,
    #     pci_func=0,
    #     InterruptLine=0x1E,
    #     InterruptPin=4,
    #     root_port_number=1,
    #     is_invisible=0,
    # )
    # board.ide.pio = board.pcie_0.downstreamMaster
    # board.ide.dma = board.pcie_0.downstreamSlave
    # board.ide.host = board.platform.pci_host
    # board.ide.disks.append(board.disk)

    # board.ide = IdeController(
    #     pci_bus=7,
    #     pci_dev=0,
    #     pci_func=0,
    #     InterruptLine=0x2,
    #     InterruptPin=1,
    #     root_port_number=2,
    #     is_invisible=0,
    # )
    # board.ide.pio = board.pcie_1.downstreamMaster
    # board.ide.dma = board.pcie_1.downstreamSlave
    # board.ide.host = board.platform.pci_host
    # board.ide.sid = 1

    board.CXL_acc = AccRTL(
        pci_bus=7,
        pci_dev=0,
        pci_func=0,
        InterruptLine=0x2,
        InterruptPin=1,
        root_port_number=2,
        is_invisible=0,
    )
    board.CXL_acc.pio = board.pcie_1.downstreamMaster
    board.CXL_acc.dma = board.pcie_1.downstreamSlave
    board.CXL_acc.host = board.platform.pci_host

    ## Test with PCI only connection
    board.CXL_acc_pci = AccRTL(
        pci_bus=0,
        pci_dev=1,
        pci_func=0,
        InterruptLine=0x2,
        InterruptPin=1,
        root_port_number=2,
        is_invisible=0,
    )
    board.CXL_acc_pci.pio = board.iobus.mem_side_ports
    board.CXL_acc_pci.dma = board.iobus.cpu_side_ports
    board.CXL_acc_pci.host = board.platform.pci_host

    board.ethernet7 = IGbE_e1000(
        pci_bus=5,
        pci_dev=0,
        pci_func=0,
        InterruptLine=0x1E,
        InterruptPin=4,
        root_port_number=1,
        is_invisible=0,
    )
    board.ethernet7.pio = board.pcie_switch.downstreamMaster
    board.ethernet7.dma = board.pcie_switch.downstreamSlave
    board.ethernet7.host = board.platform.pci_host


def make_riskv_system(args):
    cache_hierarchy = NoCache()

    # Setup the system memory.
    memory = SingleChannelDDR3_1600()

    # Setup a single core Processor.
    processor = SimpleProcessor(
        cpu_type=CPUTypes.ATOMIC, isa=ISA.RISCV, num_cores=4
    )

    # Setup the board.
    board = RiscvBoard(
        clk_freq="1GHz",
        processor=processor,
        memory=memory,
        cache_hierarchy=cache_hierarchy,
    )

    board.readfile = args.readfile_script

    if args.enable_pcie:
        add_pcie(board)

    if args.enable_gdb:
        board.workload.wait_for_remote_gdb = True

    # Set the Full System workload.
    board.set_kernel_disk_workload(
        kernel=BinaryResource(local_path="../output/images/bbl"),
        disk_image=DiskImageResource(
            local_path="../output/images/rootfs.ext2"
        ),
    )

    simulator = Simulator(board=board, checkpoint_path=args.checkpoint_restore)

    print("Beginning simulation!")

    simulator.run()


def main():
    parser = argparse.ArgumentParser(
        description="Passing arguments to customize Risc-V system launch"
    )

    parser.add_argument(
        "--enable-gdb",
        action="store_true",
        help="enable GDB mode - system waits for a remote connection to port :7000",
    )

    parser.add_argument(
        "--enable-pcie",
        action="store_true",
        help="add the PCIe hierarchy to the system",
    )

    parser.add_argument(
        "--readfile-script",
        type=str,
        help="path to the file that can be read from the system",
    )

    parser.add_argument(
        "--checkpoint-restore", type=str, help="path to the checkpoint folder"
    )

    args = parser.parse_args()

    requires(isa_required=ISA.RISCV)

    make_riskv_system(args)


main()
