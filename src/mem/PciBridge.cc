/*
 * Copyright (c) 2011-2013, 2015 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Ali Saidi
 *          Steve Reinhardt
 *          Andreas Hansson
 */

/**
 * @file
 * Implementation of a memory-mapped bridge that connects a master
 * and a slave through a request and response queue.
 */
#include "base/inifile.hh"
#include "base/intmath.hh"

// #include "base/misc.hh" # changed to "base/logging.hh"
#include "mem/PciBridge.hh"

#include "base/logging.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "debug/PciBridge.hh"
#include "debug/CXLBridge.hh"
#include "debug/ReadWriteConfig.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/PciBridge.hh"
#include "sim/byteswap.hh"
#include "sim/core.hh"

using namespace std;

namespace gem5
{
    PciBridge::PciBridgeSlavePort::PciBridgeSlavePort(const std::string &_name,
                                                      PciBridge &_bridge,
                                                      bool upstream,
                                                      uint8_t _slaveID,
                                                      uint8_t _rcid,
                                                      Cycles _delay,
                                                      int _resp_limit)
        : ResponsePort(_name, &_bridge), bridge(_bridge),
          delay(_delay),
          outstandingResponses(0),
          retryReq(false), respQueueLimit(_resp_limit),
          sendEvent([this]
                    { trySendTiming(); },
                    _name)
    {
        is_upstream = upstream;
        // this is an upstream slave port used to accept requests from CPU.
        slaveID = _slaveID;
        rcid = _rcid;
        // slave ID : 0 for upstream root port ,
        // 1 for downstream root port 1 ,
        // 2 for downstream root port 2 ,
        // 3 for downstream root port 3 .
    }

    PciBridge::
    PciBridgeMasterPort::
    PciBridgeMasterPort(const std::string &_name,
                        PciBridge &_bridge, uint8_t _rcid,
                        Cycles _delay, int _req_limit)
        : RequestPort(_name, &_bridge), bridge(_bridge), totalCount(0),
          delay(_delay), reqQueueLimit(_req_limit),
          sendEvent([this]
                    { trySendTiming(); },
                    _name)
    {
        rcid = _rcid;
    }

    PciBridge::~PciBridge()
    {
        delete storage_ptr1;
        delete storage_ptr2;
        delete storage_ptr3;
        delete storage_ptr4;
    }

    void PciBridge::initialize_ports(config_class *storage_ptr
                                    , const Params &p)
    {
        storage_ptr->valid_io_limit = false;
        storage_ptr->valid_io_base = false;
        storage_ptr->valid_memory_base = false;
        storage_ptr->valid_memory_limit = false;
        storage_ptr->valid_prefetchable_memory_base = false;
        storage_ptr->valid_prefetchable_memory_limit = false;
        storage_ptr->is_switch = p.is_switch;
        storage_ptr->storage1.VendorId = htole(p.VendorId);
        storage_ptr->storage1.Command = htole(p.Command);
        storage_ptr->storage1.Status = htole(p.Status);
        storage_ptr->storage1.ClassCode = htole(p.ClassCode);
        storage_ptr->storage1.SubClassCode = htole(p.SubClassCode);
        storage_ptr->storage1.ProgIF = htole(p.ProgIF);
        storage_ptr->storage1.Revision = htole(p.Revision);
        storage_ptr->storage1.BIST = htole(p.BIST);
        storage_ptr->storage1.HeaderType = htole(p.HeaderType);
        storage_ptr->storage1.LatencyTimer = htole(p.LatencyTimer);
        storage_ptr->storage1.CacheLineSize = htole(p.CacheLineSize);
        storage_ptr->storage1.Bar0 = htole(p.Bar0);
        storage_ptr->storage1.Bar1 = htole(p.Bar1);
        storage_ptr->storage1.SecondaryLatencyTimer
        = htole(p.SecondaryLatencyTimer);
        storage_ptr->storage1.MemoryLimit = htole(p.MemoryLimit);
        storage_ptr->storage1.MemoryBase = htole(p.MemoryBase);
        storage_ptr->storage1.PrefetchableMemoryLimit
        = htole(p.PrefetchableMemoryLimit);
        storage_ptr->storage1.PrefetchableMemoryBase
        = htole(p.PrefetchableMemoryBase);
        storage_ptr->storage1.PrefetchableMemoryLimitUpper
        = htole(p.PrefetchableMemoryLimitUpper);
        storage_ptr->storage1.PrefetchableMemoryBaseUpper
        = htole(p.PrefetchableMemoryBaseUpper);
        storage_ptr->storage1.IOLimit = htole(p.IOLimit);
        storage_ptr->storage1.IOBase = htole(p.IOBase);
        storage_ptr->storage1.IOLimitUpper = htole(p.IOLimitUpper);
        storage_ptr->storage1.IOBaseUpper = htole(p.IOBaseUpper);
        storage_ptr->storage1.CapabilityPointer = htole(p.CapabilityPointer);
        storage_ptr->storage1.ExpansionROMBaseAddress
        = htole(p.ExpansionROMBaseAddress);
        storage_ptr->storage1.InterruptPin
        = htole(p.InterruptPin);
        storage_ptr->storage1.InterruptLine
        = htole(p.InterruptLine);
        storage_ptr->storage1.BridgeControl
        = htole(p.BridgeControl);
        storage_ptr->storage1.SecondaryStatus
        = htole(p.SecondaryStatus);
        storage_ptr->storage1.PrimaryBusNumber
        = htole(p.PrimaryBusNumber);
        storage_ptr->storage1.SecondaryBusNumber
        = htole(p.SecondaryBusNumber);
        storage_ptr->storage1.SubordinateBusNumber
        = htole(p.SubordinateBusNumber);
        for (int i = 0; i < 3; i++)
            storage_ptr->storage1.reserved[i] = 0;
            // set the reserved values according
            // to PCI bridge header specifications to 0

        // Now need to configure the PCIe capability field ,
        // which is implemented for every PCIe device, including a root port
        storage_ptr->storage2.PXCAPNextCapability
        = htole(p.PXCAPNextCapability);
        storage_ptr->storage2.PXCAPCapId
        = htole(p.PXCAPCapId);
        // storage_ptr->storage2.PXCAPCapabilities
        // = htole(p.PXCAPCapabilities) ;
        storage_ptr->storage2.PXCAPDevCapabilities
        = htole(p.PXCAPDevCapabilities);
        storage_ptr->storage2.PXCAPDevCtrl
        = htole(p.PXCAPDevCtrl);
        storage_ptr->storage2.PXCAPDevStatus
        = htole(p.PXCAPDevStatus);
        storage_ptr->storage2.PXCAPLinkCap
        = htole(p.PXCAPLinkCap);
        storage_ptr->storage2.PXCAPLinkCtrl
        = htole(p.PXCAPLinkCtrl);
        storage_ptr->storage2.PXCAPLinkStatus
        = htole(p.PXCAPLinkStatus);
        storage_ptr->storage2.PXCAPRootStatus
        = htole(p.PXCAPRootStatus);
        storage_ptr->storage2.PXCAPRootControl
        = htole(p.PXCAPRootControl);

        storage_ptr->storage2.PXCAPSlotCapabilities = 0;
        storage_ptr->storage2.PXCAPSlotControl = 0;
        storage_ptr->storage2.PXCAPSlotStatus = 0;

        storage_ptr->configDelay = p.config_latency;
        // Latency for configuration space accesses.
        // Copied over from PCI device configuration access time.
        storage_ptr->barsize[0] = p.BAR0Size;
        storage_ptr->barsize[1] = p.BAR1Size;
        // indicates whether BAR for the bridges are implemented or not.
        storage_ptr->barflags[0] = 0;
        storage_ptr->barflags[1] = 0;
        storage_ptr->PXCAPBaseOffset = p.PXCAPBaseOffset;
        storage_ptr->is_valid = 0;
    }

    PciBridge::PciBridge(const Params &p)
        : ClockedObject(p),
          slavePort(p.name + ".slave", *this, true, 0, p.rc_id,
                    ticksToCycles(p.delay), p.resp_size),
          slavePort_DMA1(p.name + ".slave_dma1", *this, false, 1, p.rc_id
          , ticksToCycles(p.delay), p.resp_size),
          slavePort_DMA2(p.name + ".slave_dma2", *this, false, 2, p.rc_id
          , ticksToCycles(p.delay), p.resp_size),
          slavePort_DMA3(p.name + ".slave_dma3", *this, false, 3, p.rc_id
          , ticksToCycles(p.delay), p.resp_size),
          masterPort_DMA(p.name + ".master_dma", *this, p.rc_id
          , ticksToCycles(p.delay), p.req_size),
          masterPort1(p.name + ".master1", *this, p.rc_id,
                      ticksToCycles(p.delay), p.req_size),
          masterPort2(p.name + ".master2", *this, p.rc_id
          , ticksToCycles(p.delay), p.req_size),
          masterPort3(p.name + ".master3", *this, p.rc_id
          , ticksToCycles(p.delay), p.req_size)
    {
        cout << "PCI Bridge name" << p.name << "\n";

        is_transmit = p.is_transmit;
        int pci_bus = (p.is_switch) ? p.pci_bus + 1 : p.pci_bus;
        storage_ptr1 = new config_class(pci_bus, p.pci_dev1, p.pci_func1, 0);
        // allocate a new config class to provide PCI functionality
        // to bridge corresponding to first root port
        storage_ptr2 = new config_class(pci_bus, p.pci_dev2, p.pci_func2, 0);
        // config structure for second root port
        storage_ptr3 = new config_class(pci_bus, p.pci_dev3, p.pci_func3, 0);
        // config structure for third root port
        storage_ptr4 = new config_class(p.pci_bus, p.pci_upstream_dev
        , p.pci_upstream_func, 0);
        storage_ptr1->storage1.DeviceId = htole(p.DeviceId1);
        // Assign values depending on the PCI-PCI
        // Bridge header configured in python
        storage_ptr2->storage1.DeviceId = htole(p.DeviceId2);
        storage_ptr3->storage1.DeviceId = htole(p.DeviceId3);
        storage_ptr4->storage1.DeviceId = htole(p.DeviceId_upstream);

        initialize_ports(storage_ptr1, p);
        initialize_ports(storage_ptr2, p);
        initialize_ports(storage_ptr3, p);
        initialize_ports(storage_ptr4, p);

        uint16_t capabilities = (is_switch) ?
        p.PXCAPCapabilities_downstream : p.PXCAPCapabilities;
        storage_ptr4->storage2.PXCAPCapabilities
        = htole(p.PXCAPCapabilities_upstream);
        storage_ptr1->storage2.PXCAPCapabilities
        = htole(capabilities);
        storage_ptr2->storage2.PXCAPCapabilities
        = htole(capabilities);
        storage_ptr3->storage2.PXCAPCapabilities
        = htole(capabilities);

        storage_ptr1->pci_bus = pci_bus;
        // should be the same as the Primary Bus Number
        // of the Bridge (Upstream bus).
        storage_ptr1->pci_dev = p.pci_dev1;
        storage_ptr1->pci_func = p.pci_func1;
        // should be 0 , because it is configured
        // to be a single function device
        storage_ptr2->pci_bus = pci_bus;
        storage_ptr2->pci_dev = p.pci_dev2;
        storage_ptr2->pci_func = p.pci_func2;
        storage_ptr3->pci_bus = pci_bus;
        storage_ptr3->pci_dev = p.pci_dev3;
        storage_ptr3->pci_func = p.pci_func3;
        storage_ptr4->pci_bus = p.pci_bus;
        storage_ptr4->pci_dev = p.pci_upstream_dev;
        storage_ptr4->pci_func = p.pci_upstream_func;

        storage_ptr1->id = 1;
        storage_ptr2->id = 2;
        storage_ptr3->id = 3;
        storage_ptr4->id = 0;
        storage_ptr1->bridge = this;
        // Assign the Port being used as
        // the port that is being configured.
        storage_ptr2->bridge = this;
        storage_ptr3->bridge = this;
        storage_ptr4->bridge = this;
        p.host->registerBridge(storage_ptr1, storage_ptr1->BridgeAddr);
        // register the pci-pci bridge with
        // the pci host for config acceses
        p.host->registerBridge(storage_ptr2, storage_ptr2->BridgeAddr);
        // register the pci-pci bridge with
        // the pci host for config acceses
        p.host->registerBridge(storage_ptr3, storage_ptr3->BridgeAddr);
        // register the pci-pci bridge with
        // the pci host for config acceses
        if (p.is_switch)
            p.host->registerBridge(storage_ptr4, storage_ptr4->BridgeAddr);
        rc_id = p.rc_id;
        is_switch = p.is_switch;
    }

    // Accessor functions to get address register or base
    // and limit registers values .
    Addr config_class::getBar0()
    {
        uint32_t bar0 = letoh(storage1.Bar0);
        // Assume it is only a 32-bit BAR and never a 64 bit one
        bar0 = bar0 & ~(barsize[0] - 1);
        return (uint64_t)bar0;
    }

    Addr config_class::getBar1()
    {
        uint32_t bar1 = letoh(storage1.Bar1);
        bar1 = bar1 & ~(barsize[1] - 1);
        return (uint64_t)bar1;
    }

    Addr config_class::getIOBase()
    {
        uint8_t stored_base = letoh(storage1.IOBase);
        uint16_t stored_base_upper = letoh(storage1.IOBaseUpper);
        // if (id == 2 )
        // printf("Stored Base : %02x , Stored Base upper :
        // %04x\n" , stored_base , stored_base_upper) ;
        uint32_t base_val = 0;
        base_val = (uint32_t)stored_base_upper;
        base_val = base_val << 16;
        stored_base = stored_base >> 4;
        base_val += stored_base * IO_BASE_SHIFT;
        // value of 32 bit IO Base
        return (Addr)base_val;
    }

    Addr config_class::getIOLimit()
    {
        uint8_t stored_limit = letoh(storage1.IOLimit);
        uint16_t stored_limit_upper = letoh(storage1.IOLimitUpper);
        uint32_t limit_val = 0;
        limit_val = (uint32_t)stored_limit_upper;
        limit_val = limit_val << 16;
        stored_limit = stored_limit >> 4;
        limit_val += stored_limit * IO_BASE_SHIFT;
        // value of 32 bit IO Limit .
        // Still need to make the lower bits all 1s to satisfy alignment.
        limit_val |= 0x00000FFF;
        return (Addr)limit_val;
    }
    Addr config_class::getMemoryBase()
    {
        uint16_t stored_base = letoh(storage1.MemoryBase);
        stored_base = stored_base >> 4;
        uint32_t base_val = (uint32_t)stored_base;
        base_val = base_val << 20;
        return (Addr)base_val;
    }

    Addr config_class::getMemoryLimit()
    {
        uint16_t stored_limit = letoh(storage1.MemoryLimit);
        stored_limit = stored_limit >> 4;
        uint32_t limit_val = (uint32_t)stored_limit;
        limit_val = limit_val << 20;
        limit_val |= 0x000FFFFF;
        // make the botton 20 bits of memory limit all 1's
        return (Addr)limit_val;
    }

    Addr config_class::getPrefetchableMemoryBase()
    {
        uint16_t stored_base =
        letoh(storage1.PrefetchableMemoryBase);
        uint32_t stored_base_upper =
        letoh(storage1.PrefetchableMemoryBaseUpper);
        uint64_t base_val = 0;
        base_val = (uint64_t)stored_base_upper;
        base_val = base_val << 32;
        stored_base = stored_base >> 4;
        base_val += stored_base * PREFETCH_BASE_SHIFT;
        return base_val;
    }

    Addr config_class::getPrefetchableMemoryLimit()
    {

        uint16_t stored_limit =
        letoh(storage1.PrefetchableMemoryLimit);
        uint16_t stored_limit_upper =
        letoh(storage1.PrefetchableMemoryLimitUpper);
        uint64_t limit_val = 0;
        limit_val = (uint64_t)stored_limit_upper;
        limit_val = limit_val << 32;
        stored_limit = stored_limit >> 4;
        limit_val += stored_limit * PREFETCH_BASE_SHIFT;
        limit_val |= ((uint64_t)0x000FFFFF);
        return limit_val;
    }

    PciBridge::PciBridgeMasterPort *
    PciBridge::getMasterPort(Addr address)
    {

        if ((address >= storage_ptr1->getMemoryBase())
        && (address <= storage_ptr1->getMemoryLimit())
        && (storage_ptr1->getMemoryBase() != 0)
        && (storage_ptr1->getMemoryLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort1;
        if ((address >= storage_ptr1->getPrefetchableMemoryBase())
        && (address <= storage_ptr1->getPrefetchableMemoryLimit())
        && (storage_ptr1->getPrefetchableMemoryBase() != 0)
        && (storage_ptr1->getPrefetchableMemoryLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort1;
        if ((address >= storage_ptr1->getIOBase())
        && (address <= storage_ptr1->getIOLimit())
        && (storage_ptr1->getIOBase() != 0)
        && (storage_ptr1->getIOLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort1;

        if ((address >= storage_ptr2->getMemoryBase())
        && (address <= storage_ptr2->getMemoryLimit())
        && (storage_ptr2->getMemoryBase() != 0)
        && (storage_ptr2->getMemoryLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort2;
        if ((address >= storage_ptr2->getPrefetchableMemoryBase())
        && (address <= storage_ptr2->getPrefetchableMemoryLimit())
        && (storage_ptr2->getPrefetchableMemoryBase() != 0)
        && (storage_ptr2->getPrefetchableMemoryLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort2;
        if ((address >= storage_ptr2->getIOBase())
        && (address <= storage_ptr2->getIOLimit())
        && (storage_ptr2->getIOBase() != 0)
        && (storage_ptr2->getIOLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort2;

        if ((address >= storage_ptr3->getMemoryBase())
        && (address <= storage_ptr3->getMemoryLimit())
        && (storage_ptr3->getMemoryBase() != 0)
        && (storage_ptr3->getMemoryLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort3;
        if ((address >= storage_ptr3->getPrefetchableMemoryBase())
        && (address <= storage_ptr3->getPrefetchableMemoryLimit())
        && (storage_ptr3->getPrefetchableMemoryBase() != 0)
        && (storage_ptr3->getPrefetchableMemoryLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort3;
        if ((address >= storage_ptr3->getIOBase())
        && (address <= storage_ptr3->getIOLimit())
        && (storage_ptr3->getIOBase() != 0)
        && (storage_ptr3->getIOLimit() != 0))
            return (PciBridge::PciBridgeMasterPort *)&masterPort3;

        // printf("Forwarding packet to master port dma \n") ;
        return (PciBridge::PciBridgeMasterPort *)&masterPort_DMA;
        // If no downstream ports are willing to handle the request ,
        // just give it to the DMA Port
    }

    PciBridge::PciBridgeSlavePort *

    PciBridge::getSlavePort(int bus_num)
    {

        if ((bus_num >= storage_ptr1->storage1.SecondaryBusNumber)
        && (bus_num <= storage_ptr1->storage1.SubordinateBusNumber)
        && (storage_ptr1->is_valid == 1))
            return (PciBridgeSlavePort *)&slavePort_DMA1;
        else if ((bus_num >= storage_ptr2->storage1.SecondaryBusNumber)
        && (bus_num <= storage_ptr2->storage1.SubordinateBusNumber)
        && (storage_ptr2->is_valid == 1))
        {
            // printf("Slaveport 2 has primary bus number %02x ,
            // Subordinate Bus numberr %02x, given : %d\n",
            // (uint8_t)storage_ptr2->storage1.PrimaryBusNumber,
            // (uint8_t)storage_ptr2->storage1.SubordinateBusNumber, bus_num) ;
            return (PciBridgeSlavePort *)&slavePort_DMA2;
        }
        else if ((bus_num >= storage_ptr3->storage1.SecondaryBusNumber)
        && (bus_num <= storage_ptr3->storage1.SubordinateBusNumber)
        && (storage_ptr3->is_valid == 1))
            return (PciBridgeSlavePort *)&slavePort_DMA3;

        // printf("Returning upstream slavePort for bus num %d\n" , bus_num) ;
        return (PciBridgeSlavePort *)&slavePort;
    }

    // Accepts an offset and decides if this location can be written to.
    // Use the PCI bridge specification to decide what registers are RO or R/W.

    bool config_class::isWritable(int offset)
    {
        if (offset == PCI_VENDOR_ID || offset == PCI_DEVICE_ID
        || offset == PCI_REVISION_ID || offset == PCI_CLASS_CODE
        || offset == PCI_SUB_CLASS_CODE || offset == PCI_BASE_CLASS_CODE
        || offset == PCI_CACHE_LINE_SIZE || offset == PCI_LATENCY_TIMER
        || offset == PCI_HEADER_TYPE || offset == PCI1_SEC_LAT_TIMER
        || offset == PCI1_RESERVED || offset == PCI1_INTR_PIN)
            return false;

        return true;
    }

    uint16_t config_class::pcie_find_capability_list(uint32_t cap_id, uint16_t *prev_p)
    {
        uint16_t prev = 0;
        uint16_t next;
        uint32_t header = *((uint32_t*)(storage_dvsec + PCI_CONFIG_SPACE_SIZE));

        if (!header) {
            /* no extended capability */
            next = 0;
            if (prev_p)
                *prev_p = prev;
            DPRINTF(CXLBridge, "no extended capability\n");
            return next;
        }
        for (next = PCI_CONFIG_SPACE_SIZE; next;
            prev = next, next = PCI_EXT_CAP_NEXT(header)) {
            DPRINTF(CXLBridge, "next:%#x\n",next);
            assert(next >= PCI_CONFIG_SPACE_SIZE);
            assert(next <= PCIE_CONFIG_SPACE_SIZE - 8);

            header = *((uint32_t*)(storage_dvsec + next));
            if (PCI_EXT_CAP_ID(header) == cap_id) {
                break;
            }
        }

        if (prev_p) {
            *prev_p = prev;
        }
        DPRINTF(CXLBridge, "the previous pointer is %#x\n", *prev_p);
        return next;
    }


    void config_class::dvsec_header_update(uint16_t prev, uint16_t next)
    {
        uint8_t *dvsec_ptr = &storage_dvsec[prev];

        uint32_t header = *((uint32_t*)dvsec_ptr);
        header = (header & ~PCI_EXT_CAP_NEXT_MASK) |
            ((next << PCI_EXT_CAP_NEXT_SHIFT) & PCI_EXT_CAP_NEXT_MASK);

        // Assign the value back to the header
        __builtin_memcpy(dvsec_ptr, &header, sizeof(header));
    }

    Tick config_class::readConfig(PacketPtr pkt)
    {
        int offset = pkt->getAddr() & PCIE_CONFIG_SIZE;
        if (offset < PCIE_HEADER_SIZE)
        {
            // we now know that it is an access to
            // standard PCI-PCI bridge header
            switch (pkt->getSize())
            {
            case sizeof(uint8_t):
                pkt->setLE<uint8_t>(*((uint8_t *)&storage1.data[offset]));
                // printf("Bridge offset %d data %02x\n"
                // , offset, *((uint8_t *)&storage1.data[offset]));
                DPRINTF(ReadWriteConfig,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint8_t>());
                break;
            case sizeof(uint16_t):
                pkt->setLE<uint16_t>(*((uint16_t *)&storage1.data[offset]));
                // printf("Bridge offset %d data %04x\n"
                // , offset, *((uint16_t *)&storage1.data[offset]));
                DPRINTF(ReadWriteConfig,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint16_t>());
                break;
            case sizeof(uint32_t):
                pkt->setLE<uint32_t>(*((uint32_t *)&storage1.data[offset]));
                // printf("Bridge offset %d data %08x\n"
                // , offset, *((uint32_t *)&storage1.data[offset]));
                DPRINTF(ReadWriteConfig,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint32_t>());
                break;
            default:
                panic("invalid read access size\n");
            }
        }
        else if (offset >= PXCAPBaseOffset
        && offset < PXCAPBaseOffset + PXCAPSIZE)
        {
            switch (pkt->getSize())
            {
            case sizeof(uint8_t):
                pkt->setLE<uint8_t>(*((uint8_t *)
                &storage2.data[offset - PXCAPBaseOffset]));
                DPRINTF(ReadWriteConfig,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint8_t>());
                break;
            case sizeof(uint16_t):
                pkt->setLE<uint16_t>(*((uint16_t *)
                &storage2.data[offset - PXCAPBaseOffset]));
                DPRINTF(ReadWriteConfig,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint16_t>());
                break;
            case sizeof(uint32_t):
                pkt->setLE<uint32_t>(*((uint32_t *)
                &storage2.data[offset - PXCAPBaseOffset]));
                DPRINTF(ReadWriteConfig,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint32_t>());
                break;
            default:
                panic("invalid read access size\n");
            }
        }
        else if(offset >= PCI_CONFIG_SPACE_SIZE
        && offset < PCIE_CONFIG_SPACE_SIZE)
        {
            DPRINTF(CXLBridge,"ReadConfig in DVSEC\n");
            switch (pkt->getSize())
            {
            case sizeof(uint8_t):
                pkt->setLE<uint8_t>(*((uint8_t *)
                &storage_dvsec[offset]));
                DPRINTF(CXLBridge,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint8_t>());
                break;
            case sizeof(uint16_t):
                pkt->setLE<uint16_t>(*((uint16_t *)
                &storage_dvsec[offset]));
                DPRINTF(CXLBridge,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint16_t>());
                break;
            case sizeof(uint32_t):
                pkt->setLE<uint32_t>(*((uint32_t *)
                &storage_dvsec[offset]));
                DPRINTF(CXLBridge,"ReadConfig:%#x\n",
                (uint32_t)pkt->getLE<uint32_t>());
                break;
            default:
                panic("invalid read access size\n");
            }
        }
        else
        {
            DPRINTF(ReadWriteConfig,"ReadConfig:0x0000\n");
            switch (pkt->getSize())
            {
            case sizeof(uint8_t):
                pkt->setLE<uint8_t>(0);
                break;
            case sizeof(uint16_t):
                pkt->setLE<uint16_t>(0);
                break;
            case sizeof(uint32_t):
                pkt->setLE<uint32_t>(0);
                break;
            default:
                panic("invalid read access size\n");
            }
        }
        pkt->makeAtomicResponse();

        return configDelay;
    }

    Tick config_class::writeConfig(PacketPtr pkt)
    {
        int offset = pkt->getAddr() & PCIE_CONFIG_SIZE;

        // if ((offset == PCI1_SUB_BUS_NUM) &&(id==3))
        // printf("Subordinate bus num written with size %d\n"
        // , (int)pkt->getSize()) ;
        // if ((offset == PCI1_SEC_BUS_NUM) &&(id==3))
        // printf("Secondary bus num written with size %d\n"
        // , (int)pkt->getSize()) ;
        // if ((offset == PCI1_PRI_BUS_NUM) &&(id==3))
        // printf("Primary bus num written with size %d\n"
        // , (int)pkt->getSize()) ;

        int flag = 0;
        // set flag if the IO Base/Limit
        // or Mem Base/Limit or BARs are modified.
        std::string field = "";
        if (offset < PCIE_HEADER_SIZE)
        {
            if (!isWritable(offset)) // if we can't write to this location
            {
                pkt->makeAtomicResponse();
                return configDelay;
            }
            else
            {
                switch (pkt->getSize())
                {
                case sizeof(uint8_t):
                    field = "";
                    if (offset == PCI1_PRI_BUS_NUM)
                    { // Must be writing to one of the 1 B RW registers :
                    // Primary Bus Num,Secondary Bus Num
                    // ,Subordinate Bus Num, IO Limit, IO Base,Int line,BIST
                        storage1.PrimaryBusNumber = pkt->getLE<uint8_t>();
                        field = "PrimaryBusNumber";
                    }
                    else if (offset == PCI1_SEC_BUS_NUM)
                    {
                        storage1.SecondaryBusNumber = pkt->getLE<uint8_t>();
                        is_valid = 1;
                        field = "SecondaryBusNumber";
                    }
                    else if (offset == PCI1_SUB_BUS_NUM)
                    {
                        storage1.SubordinateBusNumber = pkt->getLE<uint8_t>();
                        field = "SubordinateBusNumber";
                    }
                    else if (offset == PCI1_IO_BASE)
                    // For the IO Base register,
                    // only the top 4 bits are writable by software.
                    // The hex value represents the most significant
                    // Hex Digit in 16 Bit IO address.
                    // Aligned in 4 KB granularity
                    // since bottom 12 address bits are always 0.
                    {
                        uint8_t temp = letoh(storage1.IOBase);
                        uint8_t val = letoh(pkt->getLE<uint8_t>());
                        temp = (val & ~IO_BASE_MASK) | (temp & IO_BASE_MASK);
                        // printf("Val is %02x , Temp is %02x\n"
                        // , val , temp) ;
                        storage1.IOBase = htole(temp);
                        flag = 1; // Ranges change
                        valid_io_base = true;
                        field = "IO Base register";
                    }

                    else if (offset == PCI1_IO_LIMIT)
                    {
                        uint8_t temp = letoh(storage1.IOLimit);
                        // Only top 4 bits of IO Limit
                        // register are writable by S.W.
                        // This hex value represents
                        // the most significanr 4 bits
                        // of 16 bit IO address.
                        // Bottom 12 bits are all assumed to be 1 .
                        // E.G. IO_Limit register [ 7:4] = 0x4 indicates
                        // that the IO Limit is 0x4FFF .
                        // This makes sense since IO Bases are aligned
                        // on  4 KB boundaries.
                        uint8_t val = letoh(pkt->getLE<uint8_t>());
                        temp = (val & ~IO_LIMIT_MASK) | (temp & IO_LIMIT_MASK);
                        storage1.IOLimit = htole(temp);
                        flag = 1; // Ranges change
                        valid_io_limit = true;
                        field = "IO Limit register";
                    }
                    else if (offset == PCI1_INTR_LINE)
                    {
                        storage1.InterruptLine = pkt->getLE<uint8_t>();
                        field = "InterruptLine";
                    }
                    else if (offset == PCI_BIST)
                    {
                        storage1.BIST = pkt->getLE<uint8_t>();
                        field = "BIST";
                    }
                    DPRINTF(PciBridge
                    , "Config write to %s field of bridge. Data: %02x\n",
                    field, pkt->getLE<uint8_t>());
                    DPRINTF(ReadWriteConfig,"WriteConfig:%#x\n",
                    (uint32_t)pkt->getLE<uint8_t>());
                    break;
                case sizeof(uint16_t):
                    // Writing to 2B RW config regs. Command , Status ,
                    // Secondary Status, Mem base, Mem limit,
                    // Prefetch mem base, prefetch mem limit, IO limit
                    // upper , IO Base Upper , Bridge Control
                    field = "";
                    if (offset == PCI_COMMAND)
                    {
                        storage1.Command =
                        htole(letoh(pkt->getLE<uint8_t>()) | 0x0400);
                        // disable intx interrupts at all times
                        field = "Command register";
                    }
                    else if (offset == PCI_STATUS)
                    {
                        storage1.Status = pkt->getLE<uint8_t>();
                        field = "Status register";
                    }
                    else if (offset == PCI1_SECONDARY_STATUS)
                    {
                        storage1.SecondaryStatus = pkt->getLE<uint8_t>();
                        field = "Secondary Status register";
                    }
                    else if (offset == PCI1_MEM_BASE)
                    {
                        uint16_t temp = letoh(storage1.MemoryBase);
                        // Upper 12 bits of this register are
                        // the upper 12 bits of 32 bit
                        // memory base address.
                        // The lower 20 bits are assumed to be
                        // 0 . Thus memory base addresses
                        // are aligned at 1 MB boundary.
                        uint16_t val = letoh(pkt->getLE<uint16_t>());
                        temp = (val & ~MMIO_BASE_MASK)
                        | (temp & MMIO_BASE_MASK);
                        storage1.MemoryBase = htole(temp);
                        flag = 1;
                        valid_memory_base = true;
                        field = "Memory base";
                    }
                    else if (offset == PCI1_MEM_LIMIT)
                    {
                        uint16_t temp = letoh(storage1.MemoryLimit);
                        uint16_t val = letoh(pkt->getLE<uint16_t>());
                        temp = (val & ~MMIO_LIMIT_MASK)
                        | (temp & MMIO_LIMIT_MASK);
                        //  Upper 12 bits of this register are writable
                        // and are upper 12 bits of 32 bit memory limit.
                        // The lower 20 bits are 1.
                        storage1.MemoryLimit = htole(temp);
                        flag = 1;
                        valid_memory_limit = true;
                        field = "Memory limit";
                    }

                    else if (offset == PCI1_PRF_MEM_BASE)
                    {
                        uint16_t temp = letoh(storage1.PrefetchableMemoryBase);
                        // Upper 12 bits of this register are writable
                        // and are the upper 12 bits of 32 bit
                        // Prefetch Mem Base address.
                        // The lower 20 bits are assumed to be 0 ,
                        // so the pretchable mem base is aligned at 1 MB.
                        uint16_t val = letoh(pkt->getLE<uint16_t>());
                        temp = (val & ~PREFETCH_MEM_BASE_MASK)
                        | (temp & PREFETCH_MEM_BASE_MASK);
                        storage1.PrefetchableMemoryBase = htole(temp);
                        flag = 1;
                        valid_prefetchable_memory_base = true;
                        field = "Prefetch Memory Base";
                    }
                    else if (offset == PCI1_PRF_MEM_LIMIT)
                    {
                        uint16_t temp =
                        letoh(storage1.PrefetchableMemoryLimit);
                        // Upper 12 bits are writable and
                        // are the upper 12 bits of 32 bit Prefetch
                        // Mem Limit. the Lower 20 bits
                        // are all 1 , so each limit ends with FFFFF.
                        uint16_t val = letoh(pkt->getLE<uint16_t>());
                        temp = (val & ~PREFETCH_MEM_LIMIT_MASK)
                        | (temp & PREFETCH_MEM_BASE_MASK);
                        storage1.PrefetchableMemoryLimit = htole(temp);
                        flag = 1;
                        valid_prefetchable_memory_limit = true;
                        field = "Prefetch Memory Limit";
                    }

                    else if (offset == PCI1_IO_BASE_UPPER)
                    // Upper 16 bits of 32 bit IO Base.
                    //Valid if bit 0 of IO Base register is set.
                    {
                        // printf("trying to store %04x in IO Base upper\n"
                        // , pkt->getLE<uint16_t>()) ;
                        // storage1.IOBaseUpper = pkt->getLE<uint16_t>() ;
                        //comment this out
                        flag = 1;
                        valid_io_base = true;
                        field = "IO Base Upper";
                    }
                    else if (offset == PCI1_IO_LIMIT_UPPER)
                    // Upper 16 bits of 32 bit IO Limit.
                    // Valid if bit 0 of IO Limit register is set.
                    {
                        //  storage1.IOLimitUpper = pkt->getLE<uint16_t>() ;
                        //comment this out
                        //printf("trying to store %04x in IO limit upper\n"
                        // , pkt->getLE<uint16_t>()) ;
                        flag = 1;
                        valid_io_limit = true;
                        field = "IO Limit Upper";
                    }
                    else if (offset == PCI1_BRIDGE_CTRL)
                    {
                        storage1.BridgeControl = pkt->getLE<uint16_t>();
                        field = "Bridge Control";
                    }
                    else if (offset == PCI1_IO_BASE)
                    {
                        //   printf("Assigning io base and limit\n") ;
                        uint16_t temp = letoh(pkt->getLE<uint16_t>());
                        flag = 1;
                        valid_io_base = true;
                        valid_io_limit = true;
                        uint8_t t = (uint8_t)(temp & 0x00FF);
                        uint8_t stored_val = letoh(storage1.IOBase);
                        stored_val = (t & ~IO_BASE_MASK)
                        | (stored_val & IO_BASE_MASK);
                        storage1.IOBase = htole(stored_val);
                        DPRINTF(PciBridge,"storage1.IOBase: %#x\n",storage1.IOBase);
                        temp = temp >> 8;
                        stored_val = letoh(storage1.IOLimit);
                        t = (uint8_t)temp;
                        stored_val = (t & ~IO_LIMIT_MASK)
                        | (stored_val & IO_LIMIT_MASK);
                        storage1.IOLimit = htole(stored_val);
                        DPRINTF(PciBridge,"storage1.IOLimit: %#x\n",storage1.IOLimit);
                        field = "IO Base";
                    }
                    DPRINTF(PciBridge, "Config write to %s field of bridge."
                    "Data: %04x\n", field, pkt->getLE<uint16_t>());
                    DPRINTF(ReadWriteConfig,"WriteConfig:%#x\n",
                    (uint32_t)pkt->getLE<uint16_t>());
                    break;

                case sizeof(uint32_t):
                    // Can be expansion ROM Base Address ,
                    // BAR0 , BAR1 or PrefetchableBase/Limit Upper
                    field = "";
                    if (offset == PCI1_ROM_BASE_ADDR)
                    {
                        // storage1.ExpansionROMBaseAddress
                        // = pkt->getLE<uint32_t>() ;
                        storage1.ExpansionROMBaseAddress =
                        htole((uint32_t)0);
                        field = "ROM Base Address";
                    }
                    else if (offset == PCI1_PRF_BASE_UPPER)
                    // Upper 32 bits of 64 bit Prefetchable Memory base
                    {
                        valid_prefetchable_memory_base = true;
                        storage1.PrefetchableMemoryBaseUpper =
                        pkt->getLE<uint32_t>();
                        flag = 1;
                        field = "Prefetch Memory Base";
                    }
                    else if (offset == PCI1_PRF_LIMIT_UPPER)
                    // Upper 32 bits of 64 bit Prefetchable Memory Limit
                    {
                        valid_prefetchable_memory_limit = true;
                        storage1.PrefetchableMemoryLimitUpper =
                        pkt->getLE<uint32_t>();
                        flag = 1;
                        field = "Prefetch Limit Upper";
                    }
                    else if (offset == PCI1_BASE_ADDR0)
                    {
                        int val = letoh(pkt->getLE<uint32_t>());
                        uint32_t Bar0 = letoh(storage1.Bar0);
                        // BAR's are stored in Little Endian order
                        // ( may be different from host order depending on ISA)
                        if (val == 0xFFFFFFFF)
                        // writing into BAR initially to determine
                        // memory size requested.
                        {
                            if (Bar0 & 1) // LSB is set , indicating IO BAR
                            {
                                Bar0 = (~(barsize[0]) - 1)
                                | (Bar0 & IO_MASK);
                                // LSB always has to remain set ,
                                // otherwise it becomes a memory BAR.
                                // Minimum size is 4 B.
                            }
                            else
                            {
                                Bar0 = ~(barsize[0] - 1)
                                | (Bar0 & MEM_MASK);
                                // minimum Bar Size is 128 B for a memory BAr
                                // , according to PCI Express spec.
                                // Can't set it lower than that.
                                // Want to preserve size
                                // and prefetchable bit values
                                // in the memory BAR
                            }
                        }
                        else
                        {
                            barflags[0] = 1;
                            flag = 1;
                            // Base address field for MEM BAR is [31:7] .
                            if (Bar0 & 1) // IO BAr
                            {
                                Bar0 = (val & ~(barsize[0] - 1))
                                | (Bar0 & IO_MASK);
                            }
                            else
                            {

                                Bar0 = (val & ~(barsize[0] - 1))
                                | (Bar0 & MEM_MASK);
                                // Again, the BAR Size requested
                                // has to be >= 128 B
                                // for a memory BAR ,
                                // and the base address has
                                // to be aligned to the Bar Size.
                            }
                        }
                        storage1.Bar0 = htole(Bar0);
                        DPRINTF(PciBridge,"setting BAR0 from devices: %#x!!\n",
                        storage1.Bar0);
                        // convert from host to little endian
                        field = "BAR0";
                    }
                    else if (offset == PCI1_BASE_ADDR1)
                    {

                        int val = letoh(pkt->getLE<uint32_t>());
                        uint32_t Bar1 = letoh(storage1.Bar1);

                        if (val == 0xFFFFFFFF)
                        // writing into BAR initially
                        // to determine memory size requested.
                        {
                            if (Bar1 & 1) // LSB is set , indicating IO BAR
                            {
                                Bar1 = ~(barsize[1] - 1)
                                | (Bar1 & IO_MASK);
                                // LSB always has to remain set ,
                                // otherwise it becomes a memory BAR.
                                // Minimum size is 4 B.
                            }
                            else
                            {
                                Bar1 = ~(barsize[1] - 1)
                                | (Bar1 & MEM_MASK);
                                // minimum Bar Size is 128 B for a memory BAr ,
                                //according to PCI Express spec.
                                //Can't set it lower than that.
                            }
                        }
                        else
                        {
                            flag = 1;
                            barflags[1] = 1;
                            // Base address field is [31:7] .
                            if (Bar1 & 1) // IO BAr
                            {
                                Bar1 = (val & ~(barsize[1] - 1))
                                | (Bar1 & IO_MASK);
                            }
                            else
                            {

                                Bar1 = (val & ~(barsize[1] - 1))
                                | (Bar1 & MEM_MASK);
                                // Again, the BAR Size requested
                                // has to be >= 128 B
                                // for a memory BAR ,
                                // and the base address has to be aligned
                                // to the Bar Size.
                            }
                        }
                        storage1.Bar1 = htole(Bar1);
                        DPRINTF(PciBridge,"setting BAR1 from devices: %#x!!\n",
                        storage1.Bar1);
                        field = "BAR1";
                    }

                    else if (offset == PCI1_MEM_BASE)
                    {
                        valid_memory_base = true;
                        valid_memory_limit = true;
                        uint32_t val = pkt->getLE<uint32_t>();
                        uint16_t base_val = (uint16_t)(val & 0x0000FFFF);
                        uint16_t cur_base_val = letoh(storage1.MemoryBase);
                        uint16_t limit_val = (uint16_t)(val >> 16);
                        uint16_t cur_limit_val = letoh(storage1.MemoryLimit);
                        cur_base_val = (base_val & ~MMIO_BASE_MASK)
                        | (cur_base_val & MMIO_LIMIT_MASK);
                        cur_limit_val = (limit_val & ~MMIO_LIMIT_MASK)
                        | (cur_limit_val & MMIO_LIMIT_MASK);
                        storage1.MemoryBase = htole(cur_base_val);
                        storage1.MemoryLimit = htole(cur_limit_val);
                        flag = 1;
                        field = "Mem Base";
                    }
                    else if (offset == PCI1_IO_BASE_UPPER)
                    {
                        valid_io_base = true;
                        valid_io_limit = true;
                        printf("trying to store %08x in IO Base upper"
                        ", IO Limit upper \n"
                        , pkt->getLE<uint32_t>());
                        /*      uint32_t val = pkt->getLE<uint32_t>() ;
                        // Comment out from here
                            printf("Val for IOBase upper is %08x\n" , val) ;
                            storage1.IOBaseUpper =
                            (uint16_t)(val & 0x0000FFFF) ;
                            storage1.IOLimitUpper =
                            (uint16_t)(val >> 16) ; */
                        // To here
                        flag = 1;
                        field = "IO Base Upper";
                    }
                    else if (offset == PCI1_PRF_MEM_BASE)
                    {
                        valid_prefetchable_memory_base = true;
                        valid_prefetchable_memory_limit = true;
                        uint32_t val = pkt->getLE<uint32_t>();
                        uint16_t base_val =
                        (uint16_t)(val & 0x0000FFFF);
                        uint16_t cur_base_val =
                        letoh(storage1.PrefetchableMemoryBase);
                        uint16_t limit_val = (uint16_t)(val >> 16);
                        uint16_t cur_limit_val =
                        letoh(storage1.PrefetchableMemoryLimit);
                        cur_base_val = (base_val & ~PREFETCH_MEM_BASE_MASK)
                        | (cur_base_val & PREFETCH_MEM_BASE_MASK);
                        cur_limit_val = (limit_val & ~PREFETCH_MEM_LIMIT_MASK)
                        | (cur_limit_val & PREFETCH_MEM_BASE_MASK);
                        storage1.PrefetchableMemoryBase =
                        htole(cur_base_val);
                        storage1.PrefetchableMemoryLimit =
                        htole(cur_limit_val);
                        flag = 1;
                        field = "Prefetch Memory Base";
                    }
                    else if (offset == PCI_COMMAND)
                    {
                        uint32_t val = pkt->getLE<uint32_t>();
                        storage1.Command = (uint16_t)(val & 0x0000FFFF);
                        storage1.Status = (uint16_t)(val >> 16);
                        field = "Command";
                    }
                    else if (offset == PCI1_PRI_BUS_NUM)
                    {
                        *(uint32_t *)&storage1.data[PCI1_PRI_BUS_NUM] =
                        pkt->getLE<uint32_t>();
                        is_valid = 1;
                        //  if (id == 3)printf ("Pri bus num
                        // , Sec bus num , sub bus num = %d,%d,%d\n" ,
                        // (int)storage1.PrimaryBusNumber ,
                        // (int)storage1.SecondaryBusNumber
                        // , (int)storage1.SubordinateBusNumber) ;
                        field = "Primary Bus Number";
                    }
                    DPRINTF(PciBridge,
                    "Config write to %s field of bridge. Data: %08x\n",
                    field, pkt->getLE<uint32_t>());
                    DPRINTF(ReadWriteConfig,"WriteConfig:%#x\n",
                    (uint32_t)pkt->getLE<uint32_t>());
                    break;

                default:
                    panic("invalid write access size\n");
                } // end of switch

            } // end of else

        } // end of if offset < PCIE_HEADER_SIZE

        else if (offset >= PXCAPBaseOffset && offset
        < PXCAPBaseOffset + PXCAPSIZE)
        // writing to PCIe capability structure.
        // Ignore writes to slot registers, not implemented.
        {
            offset -= PXCAPBaseOffset;
            // get the offset from the base of the PCIe cap register set
            if ((offset < SLOT_REG_BASE) && (offset > SLOT_REG_LIMIT))
            // ignoring writes to slot registers
            {
                switch (pkt->getSize())
                {
                case sizeof(uint8_t):
                    *(uint8_t *)&storage2.data[offset] =
                    pkt->getLE<uint8_t>();
                    break;
                case sizeof(uint16_t):
                    *(uint16_t *)&storage2.data[offset] =
                    pkt->getLE<uint16_t>();
                    break;
                case sizeof(uint32_t):
                    *(uint32_t *)&storage2.data[offset] =
                    pkt->getLE<uint32_t>();
                    break;
                default:
                    panic("invalid write access size\n");
                }
            }
            DPRINTF(PciBridge,
            "Writing to bridge PCIe capability structure\n");
        }
        else if (offset >= PCI_DEVICE_SPECIFIC
                && offset < PCI_CONFIG_SIZE)
        {
            warn_once("Device specific PCI config space "
                    "not implemented for %s!\n", name());
            switch (pkt->getSize()) {
                case sizeof(uint8_t):
                case sizeof(uint16_t):
                case sizeof(uint32_t):
                    break;
                default:
                    panic("invalid access size(?) for PCI configspace!\n");
            }
        } else if (offset >= PCI_CONFIG_SPACE_SIZE &&
        offset < PCIE_CONFIG_SIZE) {
            DPRINTF(CXLBridge,"WriteConfig in PCIE capability\n");
            // get the offset from the base of the PCIe cap register set
            switch (pkt->getSize())
            {
            case sizeof(uint8_t):
                *(uint8_t *)&storage_dvsec[offset] =
                pkt->getLE<uint8_t>();
                break;
            case sizeof(uint16_t):
                *(uint16_t *)&storage_dvsec[offset] =
                pkt->getLE<uint16_t>();
                break;
            case sizeof(uint32_t):
                *(uint32_t *)&storage_dvsec[offset] =
                pkt->getLE<uint32_t>();
                break;
            default:
                panic("invalid write access size\n");
            }
        }
        else
        {
            DPRINTF(CXLBridge,"write in empty place\n");
            DPRINTF(CXLBridge,"offset is:%04x"
             ", while PXCAPBaseOffset is:%04x and PXCAPSIZE is:%04x\n",
                    offset,PXCAPBaseOffset,PXCAPSIZE);
        }

        pkt->makeAtomicResponse();
        if (flag)
        {
            if (id == 0 || is_switch == 0)
                bridge->slavePort.public_sendRangeChange();
                // send a range change when any of
                // the bridge limit/BAR registers are changed
            if (id == 1)
                bridge->slavePort_DMA1.public_sendRangeChange();
            if (id == 2)
                bridge->slavePort_DMA2.public_sendRangeChange();
            if (id == 3)
                bridge->slavePort_DMA3.public_sendRangeChange();
        }
        return configDelay;
    }

    Port &
    PciBridge::getPort(const std::string &if_name, PortID idx)
    {
        if (if_name == "master1")
            return masterPort1;
        else if (if_name == "master2")
            return masterPort2;
        else if (if_name == "master3")
            return masterPort3;
        else if (if_name == "master_dma")
            return masterPort_DMA;
        else if (if_name == "slave")
            return slavePort;
        else if (if_name == "slave_dma1")
            return slavePort_DMA1;
        else if (if_name == "slave_dma2")
            return slavePort_DMA2;
        else if (if_name == "slave_dma3")
            return slavePort_DMA3;
        else
            // pass it along to our super class
            return ClockedObject::getPort(if_name, idx);
        // fatal("%s does not have any masterPort named %s\n"
        // , name(), if_name);
    }

    /*
    MasterPort&
    PciBridge::getMasterPort(const std::string &if_name, PortID idx)
    {
        if (if_name == "master1")
            return masterPort1;
        else if (if_name == "master2")
            return masterPort2 ;
        else if (if_name == "master3")
            return masterPort3 ;
        else if (if_name == "master_dma")
            return masterPort_DMA ;
        else
          // pass it along to our super class
          //return ClockedObject::getPort(if_name, idx);
          fatal("%s does not have any masterPort named %s\n"
          , name(), if_name);
    }

    SlavePort&
    PciBridge::getSlavePort(const std::string &if_name, PortID idx)
    {
        if (if_name == "slave")
            return slavePort;
        else if (if_name == "slave_dma1")
            return slavePort_DMA1 ;
        else if (if_name == "slave_dma2")
            return slavePort_DMA2 ;
        else if (if_name == "slave_dma3")
            return slavePort_DMA3 ;

        else
          // pass it along to our super class
          //return ClockedObject::getPort(if_name, idx);
          fatal("%s does not have any slavePort named %s\n"
          , name(), if_name);
    }
    */

    void
    PciBridge::init()
    {
        // make sure both sides are connected and have the same block size
        if (!slavePort.isConnected() || !slavePort_DMA1.isConnected()
        || !slavePort_DMA2.isConnected() || !slavePort_DMA3.isConnected()
        || !masterPort_DMA.isConnected() || !masterPort1.isConnected()
        || !masterPort2.isConnected() || !masterPort3.isConnected())
            fatal("Both ports of a bridge must be connected.\n");

        // notify the master side  of our address ranges
        slavePort.sendRangeChange();
    }

    bool
    PciBridge::PciBridgeSlavePort::respQueueFull() const
    {
        return outstandingResponses == respQueueLimit;
    }

    bool
    PciBridge::PciBridgeMasterPort::reqQueueFull() const
    {
        return transmitList.size() == reqQueueLimit;
    }

    bool
    PciBridge::PciBridgeMasterPort::recvTimingResp(PacketPtr pkt)
    {
        // all checks are done when the request is accepted on the slave
        // side, so we are guaranteed to have space for the response
        DPRINTF(PciBridge, "recvTimingResp: %s addr 0x%x\n",
                pkt->cmdString(), pkt->getAddr());

        DPRINTF(PciBridge, "Request queue size: %d\n", transmitList.size());
        // if ((pkt->req_bus[rcid] == 0) && (bridge.is_switch == 1))
        // printf("Master Port received response to switch port\n");
        PciBridgeSlavePort *slavePort =
        bridge.getSlavePort(pkt->req_bus[rcid]);
        // if (bridge.is_switch == 1)
        // printf("Found slave %d\n" , slavePort->slaveID) ;
        if ((slavePort->slaveID == 1) && (bridge.is_switch == 0)
        && (pkt->isResponse()) && (pkt->hasData())
        && (bridge.is_transmit == 1))
            totalCount += pkt->getSize();

        // technically the packet only reaches us after the header delay,
        // and typically we also need to deserialise any payload (unless
        // the two sides of the bridge are synchronous)
        Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
        pkt->headerDelay = pkt->payloadDelay = 0;
        // if (bridge.is_switch==0)printf("sched resp : cur:%lu, when :%lu
        // , r_d %lu\n",curTick(),
        // bridge.clockEdge(delay) + receive_delay, receive_delay) ;

        slavePort->schedTimingResp(pkt, bridge.clockEdge(delay) +
                                            receive_delay);
        // if ((slavePort->slaveID == 0) && (bridge.is_switch ==1))
        // printf("Scheduling response to upstream slave port\n") ;
        return true;
    }

    bool
    PciBridge::PciBridgeSlavePort::recvTimingReq(PacketPtr pkt)
    {
        Addr pkt_Addr = pkt->getAddr();
        // if ((slaveID==0) &&(bridge.is_switch==0))
        // printf("Slave received timing req to Addr: %08x\n"
        // , (unsigned int)pkt_Addr) ;
        PciBridgeMasterPort *masterPort =
        bridge.getMasterPort(pkt_Addr);
        if (pkt->req_bus[rcid] == -1)
        // slave port assigns the requester id
        {
            if (slaveID == 0)
            {

                pkt->req_bus[rcid] =
                bridge.storage_ptr4->pci_bus;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
            }
            else if (slaveID == 1)
            {
                // printf("Root Port 1 received DMA access\n") ;
                pkt->req_bus[rcid] =
                bridge.storage_ptr1->storage1.SecondaryBusNumber;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
                //  printf("Root Port 1 received DMA access ,
                //  appending %d to address %16lx \n"
                // , (int)pkt->req_bus[rcid] , pkt_Addr) ;
            }
            else if (slaveID == 2)
            {

                pkt->req_bus[rcid] =
                bridge.storage_ptr2->storage1.SecondaryBusNumber;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
                //   printf("Root Port 2 received DMA access
                // , appending %d\n" ,
                //(int)pkt->req_bus[rcid]) ;
            }

            else if (slaveID == 3)
            {
                // printf("Root port 3 received DMA\n") ;

                pkt->req_bus[rcid] =
                bridge.storage_ptr3->storage1.SecondaryBusNumber;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
                //  printf("Root Port 3 received DMA access
                // , appending %d\n" ,
                // (int)pkt->req_bus[rcid]) ;
            }
        }

        DPRINTF(PciBridge, "recvTimingReq: %s addr 0x%x\n",
                pkt->cmdString(), pkt->getAddr());

        panic_if(pkt->cacheResponding()
        , "Should not see packets where cache "
         "is responding");

        // we should not get a new request after committing to retry the
        // current one, but unfortunately the CPU violates this rule, so
        // simply ignore it for now
        if (retryReq)
            return false;

        DPRINTF(PciBridge
        , "Response queue size: %d pkt->resp: %d\n",
        transmitList.size(), outstandingResponses);

        // if the request queue is full then there is no hope
        if (masterPort->reqQueueFull())
        {
            DPRINTF(PciBridge, "Request queue full\n");
            retryReq = true;
        }
        else
        {
            // look at the response queue if we expect to see a response
            bool expects_response = pkt->needsResponse();
            if (expects_response)
            {
                if (respQueueFull())
                {
                    DPRINTF(PciBridge, "Response queue full\n");
                    retryReq = true;
                }
                else
                {
                    // ok to send the request with space for the response
                    DPRINTF(PciBridge, "Reserving space for response\n");
                    assert(outstandingResponses != respQueueLimit);
                    ++outstandingResponses;
                    // if ((slaveID==2) && bridge.is_switch==0)
                    // printf("Outstanding responses inc : %u\n"
                    // , outstandingResponses) ;

                    // no need to set retryReq to false as this is already the
                    // case
                }
            }

            if (!retryReq)
            {
                // technically the packet only reaches us after the header
                // delay, and typically we also need to deserialise any
                // payload (unless the two sides of the bridge are
                // synchronous)
                Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
                pkt->headerDelay = pkt->payloadDelay = 0;

                // if (bridge.is_switch==0)
                // printf("sched req : cur:%lu, when :%lu, r_d %lu\n",
                // curTick(), bridge.clockEdge(delay) + receive_delay
                // , receive_delay) ;
                masterPort->schedTimingReq(pkt, bridge.clockEdge(delay) +
                                                    receive_delay);
            }
        }

        // remember that we are now stalling a packet and that we have to
        // tell the sending master to retry once space becomes available,
        // we make no distinction whether the stalling is due to the
        // request queue or response queue being full
        return !retryReq;
    }

    void
    PciBridge::PciBridgeSlavePort::retryStalledReq()
    {
        if (retryReq)
        {
            DPRINTF(PciBridge
            , "Request waiting for retry, now retrying\n");
            retryReq = false;
            sendRetryReq();
        }
    }

    void
    PciBridge::PciBridgeMasterPort::schedTimingReq(PacketPtr pkt, Tick when)
    {
        // If we're about to put this packet at the head of the queue, we
        // need to schedule an event to do the transmit.  Otherwise there
        // should already be an event scheduled for sending the head
        // packet.
        if (transmitList.empty())
        {
            bridge.schedule(sendEvent, when);
        }

        assert(transmitList.size() != reqQueueLimit);

        transmitList.emplace_back(pkt, when);
    }

    void
    PciBridge::PciBridgeSlavePort::schedTimingResp(PacketPtr pkt, Tick when)
    {
        // If we're about to put this packet
        // at the head of the queue, we
        // need to schedule an event to do the transmit.
        // Otherwise there should already be an event
        // scheduled for sending the head
        // packet.
        if (transmitList.empty())
        {
            bridge.schedule(sendEvent, when);
        }

        transmitList.emplace_back(pkt, when);
    }

    void
    PciBridge::PciBridgeMasterPort::trySendTiming()
    {
        assert(!transmitList.empty());

        DeferredPacket req = transmitList.front();

        assert(req.tick <= curTick());

        PacketPtr pkt = req.pkt;

        DPRINTF(PciBridge
        , "trySend request addr 0x%x, queue size %d\n",
                pkt->getAddr(), transmitList.size());

        if (sendTimingReq(pkt))
        {
            // send successful
            transmitList.pop_front();
            DPRINTF(PciBridge, "trySend request successful\n");

            // If there are more packets to send
            // , schedule event to try again.
            if (!transmitList.empty())
            {
                DeferredPacket next_req = transmitList.front();
                DPRINTF(PciBridge, "Scheduling next send\n");
                bridge.schedule(sendEvent, std::max(next_req.tick,
                                                    bridge.clockEdge()));
            }

            // if we have stalled a request due to a full request queue,
            // then send a retry at this point, also note that if the
            // request we stalled was waiting for the response queue
            // rather than the request queue we might stall it again
            bridge.slavePort.retryStalledReq();
            bridge.slavePort_DMA1.retryStalledReq();
            bridge.slavePort_DMA2.retryStalledReq();
            bridge.slavePort_DMA3.retryStalledReq();
            // call all slave ports retry stalled request just to be safe
            // and avoid a request being infinitely stalled .
        }

        // if the send failed, then we try again once we receive a retry,
        // and therefore there is no need to take any action
    }


    void
    PciBridge::PciBridgeSlavePort::trySendTiming()
    {

        assert(!transmitList.empty());

        DeferredPacket resp = transmitList.front();

        assert(resp.tick <= curTick());

        PacketPtr pkt = resp.pkt;
        //  if ((slaveID==0) && (bridge.is_switch == 0))
        // printf("Upstream switch port received response\n") ;

        DPRINTF(PciBridge
        , "trySend response addr 0x%x, outstanding %d\n",
                pkt->getAddr(), outstandingResponses);

        if (sendTimingResp(pkt))
        {
            // send successful
            transmitList.pop_front();
            DPRINTF(PciBridge, "trySend response successful\n");

            assert(outstandingResponses != 0);
            --outstandingResponses;
            // if (slaveID==2 && bridge.is_switch ==0)
            // printf("Outstanding responses dec %u\n", outstandingResponses) ;

            // If there are more packets to send, schedule event to try again.
            if (!transmitList.empty())
            {
                DeferredPacket next_resp = transmitList.front();
                DPRINTF(PciBridge, "Scheduling next send\n");
                bridge.schedule(sendEvent
                , std::max(next_resp.tick,bridge.clockEdge()));
            }

            // if there is space in the request queue and we were stalling
            // a request, it will definitely be possible to accept it now
            // since there is guaranteed space in the response queue
            if (retryReq)
            {
                DPRINTF(PciBridge
                , "Request waiting for retry, now retrying\n");
                retryReq = false;
                sendRetryReq();
            }
        }

        // if the send failed,
        // then we try again once we receive a retry,
        // and therefore there is no need to take any action
    }

    void
    PciBridge::PciBridgeMasterPort::recvReqRetry()
    {
        trySendTiming();
    }

    void
    PciBridge::PciBridgeSlavePort::recvRespRetry()
    {
        trySendTiming();
    }

    Tick
    PciBridge::PciBridgeSlavePort::recvAtomic(PacketPtr pkt)
    {

        // printf("REcv atomic called\n") ;
        if (pkt->req_bus[rcid] == -1)
        // slave port assigns the requester id
        {
            if (slaveID == 0)
            {

                pkt->req_bus[rcid] = bridge.storage_ptr4->pci_bus;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
            }
            else if (slaveID == 1)
            {
                //  printf("Root Port 1 received atomic DMA access\n") ;
                pkt->req_bus[rcid] =
                bridge.storage_ptr1->storage1.SecondaryBusNumber;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
                //  printf("Root Port 1 received atomic DMA access ,
                // appending %d\n" , (int)pkt->req_bus[rcid]) ;
            }
            else if (slaveID == 2)
            {

                pkt->req_bus[rcid] =
                bridge.storage_ptr2->storage1.SecondaryBusNumber;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
                //   printf("Root Port 2 received atomic DMA access
                //, appending %d\n" , (int)pkt->req_bus[rcid]) ;
            }

            else if (slaveID == 3)
            {
                //  printf("Root port 3 received DMA\n") ;

                pkt->req_bus[rcid] =
                bridge.storage_ptr3->storage1.SecondaryBusNumber;
                pkt->req_dev[rcid] = 0;
                pkt->req_func[rcid] = 0;
                //  printf("Root Port 3 received atomic DMA access
                // , appending %d\n" , (int)pkt->req_bus[rcid]) ;
            }
        }

        PciBridgeMasterPort *masterPort = bridge.getMasterPort(pkt->getAddr());
        panic_if(pkt->cacheResponding(), "Should not see packets where cache "
                                         "is responding");

        return 0 * delay * bridge.clockPeriod() + masterPort->sendAtomic(pkt);
    }

    void
    PciBridge::PciBridgeSlavePort::recvFunctional(PacketPtr pkt)
    {
        // printf("REceive functional called\n") ;
        PciBridgeMasterPort *masterPort = bridge.getMasterPort(pkt->getAddr());
        pkt->pushLabel(name());

        // check the response queue
        for (auto i = transmitList.begin(); i != transmitList.end(); ++i)
        {
            if (pkt->trySatisfyFunctional((*i).pkt))
            {
                pkt->makeResponse();
                return;
            }
        }

        // also check the master port's request queue
        if (masterPort->trySatisfyFunctional(pkt))
        {
            return;
        }

        pkt->popLabel();

        // fall through if pkt still not satisfied
        masterPort->sendFunctional(pkt);
    }

    bool
    PciBridge::PciBridgeMasterPort::trySatisfyFunctional(PacketPtr pkt)
    {
        bool found = false;
        auto i = transmitList.begin();

        while (i != transmitList.end() && !found)
        {
            if (pkt->trySatisfyFunctional((*i).pkt))
            {
                pkt->makeResponse();
                found = true;
            }
            ++i;
        }

        return found;
    }

    void
    PciBridge::
    PciBridgeSlavePort::
    fill_ranges(AddrRangeList &ranges, config_class *storage_ptr) const
    {
        uint32_t barsize[2];
        barsize[0] = storage_ptr->barsize[0];
        barsize[1] = storage_ptr->barsize[1];
        Addr Bar0 = storage_ptr->getBar0();
        Addr Bar1 = storage_ptr->getBar1();
        Addr MemoryBase = storage_ptr->getMemoryBase();
        Addr MemoryLimit = storage_ptr->getMemoryLimit();
        Addr PrefetchableMemoryBase =
        storage_ptr->getPrefetchableMemoryBase();
        Addr PrefetchableMemoryLimit =
        storage_ptr->getPrefetchableMemoryLimit();
        Addr IOLimit = storage_ptr->getIOLimit();
        Addr IOBase = storage_ptr->getIOBase();
        // uint8_t flag1, flag2, flag3 ;
        // flag1 = 0 ;
        // flag2 = 0 ;
        // flag3 = 0 ;
        DPRINTF(PciBridge,"Bar0 size is %d;\n",barsize[0]);
        DPRINTF(PciBridge,"Bar0 addr is %#lx;\n", Bar0);
        if (barsize[0] != 0 && Bar0 != 0)
        {
            DPRINTF(PciBridge,"Bar0 assigned\n");
            ranges.push_back(RangeSize(Bar0, barsize[0]));
        }

        if (barsize[1] != 0 && Bar1 != 0)
        {
            DPRINTF(PciBridge,"Bar1 assigned \n");
            ranges.push_back(RangeSize(Bar1, barsize[1]));
        }
        if (storage_ptr->valid_memory_base && storage_ptr->valid_memory_limit
        && (MemoryBase < MemoryLimit) && (MemoryBase != 0))
        {
            // flag1 = 1 ;
            //  printf("Memory base is %08x , memory limit is %08x\n" ,
            // (unsigned int)MemoryBase, (unsigned int)MemoryLimit) ;
            DPRINTF(PciBridge,"MemoryBase assigned \n");
            ranges.push_back(RangeIn(MemoryBase, MemoryLimit));
        }
        if (storage_ptr->valid_prefetchable_memory_base
        && storage_ptr->valid_prefetchable_memory_limit
        && (PrefetchableMemoryBase < PrefetchableMemoryLimit)
        && (PrefetchableMemoryBase != 0))
        {
            // flag2 = 1 ;
            //  printf("Prefetch Memory Base is %08x ,
            // Prefetch MEmory Limit is %08x\n" ,
            // (unsigned int) PrefetchableMemoryBase ,
            // (unsigned int) PrefetchableMemoryLimit) ;
            DPRINTF(PciBridge,"PrefetchableMemoryBase assigned \n");
            ranges.push_back(RangeIn(PrefetchableMemoryBase
            , PrefetchableMemoryLimit));
        }
        DPRINTF(PciBridge,"storage_ptr->valid_io_base: %#x\n",storage_ptr->valid_io_base);
        DPRINTF(PciBridge,"storage_ptr->valid_io_limit: %#x\n",storage_ptr->valid_io_limit);
        DPRINTF(PciBridge,"IOBase: %#x\n",IOBase);
        DPRINTF(PciBridge,"IOLimit: %#x\n",IOLimit);

        if (storage_ptr->valid_io_base
        && storage_ptr->valid_io_limit
        && (IOBase < IOLimit)
        && (IOBase != 0))
        {
            // flag3 = 1 ;
            //  printf("pushing back addr io ranges %08x - %08x\n" ,
            // (unsigned int)IOBase ,(unsigned int) IOLimit) ;
            DPRINTF(PciBridge,"IOBase assigned \n");
            ranges.push_back(RangeIn(IOBase, IOLimit));

            // printf("IO Base is %08x , IO limit is %08x\n" ,
            // (unsigned int) IOBase , (unsigned int)IOLimit) ;
        }
    }

    AddrRangeList
    PciBridge::PciBridgeSlavePort::getAddrRanges() const
    {
        DPRINTF(PciBridge,"start returning;\n");
        AddrRangeList ranges;
        if ((slaveID == 0) && (bridge.is_switch == 0))
        {
            fill_ranges(ranges, bridge.storage_ptr1);
            fill_ranges(ranges, bridge.storage_ptr2);
            fill_ranges(ranges, bridge.storage_ptr3);
            DPRINTF(PciBridge,"start filling devices ranges;\n");
            DPRINTF(PciBridge,"bridge.storage_ptr1->valid_io_base %d;\n",
                    bridge.storage_ptr1->valid_io_base);
            DPRINTF(PciBridge,"bridge.storage_ptr1->valid_io_limit %d;\n",
                    bridge.storage_ptr1->valid_io_limit);
            // printf("Root port ranges\n") ;
            // for (auto it = ranges.begin() ; it != ranges.end() ; it++)
            //  printf("Range is %16lx to %16lx\n"
            // , (*it).start() , (*it).end()) ;
        }
        else if ((slaveID == 0) && (bridge.is_switch == 1))
        {
            // printf("Upstream switch port !!!\n") ;
            fill_ranges(ranges, bridge.storage_ptr4);
            DPRINTF(PciBridge,"start filling switch ranges;\n");
            // for (auto it = ranges.begin() ;  it!=ranges.end(); it++)
            //  printf("Upstream switch port Range start %16lx ,
            // Upstream switch port Range end %16lx\n"
            // , (*it).start() , (*it).end()) ;
        }
        else if (slaveID == 1)
        {
            DPRINTF(PciBridge,"start filling devices:slaveID=1 ranges;\n");
            fill_ranges(ranges, bridge.storage_ptr1);
        }

        else if (slaveID == 2)
        {
            DPRINTF(PciBridge,"start filling devices:slaveID=2 ranges;\n");
            fill_ranges(ranges, bridge.storage_ptr2);
        }
        else if (slaveID == 3)
        {
            DPRINTF(PciBridge,"start filling devices:slaveID=3 ranges;\n");
            fill_ranges(ranges, bridge.storage_ptr3);
        }

        ranges.sort();
        for (const auto& r: ranges) {
            DPRINTF(PciBridge, "Returning address range %s for\n",
                    r.to_string());
        }
        // sort this list
        // This is a downstream slave port used to
        // accept dma requests from the device.
        // Need to configure the range as the ~ of downstream ranges

        if (is_upstream)
        {
            DPRINTF(PciBridge,"upstream: Finish returning\n");
            return ranges;
        }
        else
        {

            AddrRangeList downstream_ranges;

            Addr last_val = 0;
            for (auto it = ranges.begin(); it != ranges.end(); it++)
            {
                if (last_val < (*it).start())
                    downstream_ranges.push_back(RangeIn(last_val
                    , ((*it).start()) - 1));
                last_val = (*it).end() + 1;
            }

            if (last_val == 0)
                return downstream_ranges;
            downstream_ranges.push_back(RangeIn(last_val, ADDR_MAX));
            DPRINTF(PciBridge,"downstream: Finish returning\n");
            return downstream_ranges;
        }

        return ranges;
    }

    void
    PciBridge::serialize(CheckpointOut &cp) const
    {
        SERIALIZE_ARRAY(storage_ptr4->storage1.data
        , sizeof(storage_ptr4->storage1.data)
        / sizeof(storage_ptr4->storage1.data[0]));
        SERIALIZE_ARRAY(storage_ptr4->storage2.data
        , sizeof(storage_ptr4->storage2.data)
        / sizeof(storage_ptr4->storage2.data[0]));
        SERIALIZE_ARRAY(storage_ptr1->storage1.data
        , sizeof(storage_ptr1->storage1.data)
        / sizeof(storage_ptr1->storage1.data[0]));
        SERIALIZE_ARRAY(storage_ptr1->storage2.data
        , sizeof(storage_ptr1->storage2.data)
        / sizeof(storage_ptr1->storage2.data[0]));
        SERIALIZE_ARRAY(storage_ptr2->storage1.data
        , sizeof(storage_ptr2->storage1.data)
        / sizeof(storage_ptr2->storage1.data[0]));
        SERIALIZE_ARRAY(storage_ptr2->storage2.data
        , sizeof(storage_ptr2->storage2.data)
        / sizeof(storage_ptr2->storage2.data[0]));
        SERIALIZE_ARRAY(storage_ptr3->storage1.data
        , sizeof(storage_ptr3->storage1.data)
        / sizeof(storage_ptr3->storage1.data[0]));
        SERIALIZE_ARRAY(storage_ptr3->storage2.data
        , sizeof(storage_ptr3->storage2.data)
        / sizeof(storage_ptr3->storage2.data[0]));
        SERIALIZE_SCALAR(storage_ptr4->is_valid);
        SERIALIZE_SCALAR(storage_ptr4->valid_io_base);
        SERIALIZE_SCALAR(storage_ptr4->valid_io_limit);
        SERIALIZE_SCALAR(storage_ptr4->valid_memory_base);
        SERIALIZE_SCALAR(storage_ptr4->valid_memory_limit);
        SERIALIZE_SCALAR(storage_ptr4->valid_prefetchable_memory_base);
        SERIALIZE_SCALAR(storage_ptr4->valid_prefetchable_memory_limit);
        SERIALIZE_SCALAR(storage_ptr3->is_valid);
        SERIALIZE_SCALAR(storage_ptr3->valid_io_base);
        SERIALIZE_SCALAR(storage_ptr3->valid_io_limit);
        SERIALIZE_SCALAR(storage_ptr3->valid_memory_base);
        SERIALIZE_SCALAR(storage_ptr3->valid_memory_limit);
        SERIALIZE_SCALAR(storage_ptr3->valid_prefetchable_memory_base);
        SERIALIZE_SCALAR(storage_ptr3->valid_prefetchable_memory_limit);
        SERIALIZE_SCALAR(storage_ptr2->is_valid);
        SERIALIZE_SCALAR(storage_ptr2->valid_io_base);
        SERIALIZE_SCALAR(storage_ptr2->valid_io_limit);
        SERIALIZE_SCALAR(storage_ptr2->valid_memory_base);
        SERIALIZE_SCALAR(storage_ptr2->valid_memory_limit);
        SERIALIZE_SCALAR(storage_ptr2->valid_prefetchable_memory_base);
        SERIALIZE_SCALAR(storage_ptr2->valid_prefetchable_memory_limit);
        SERIALIZE_SCALAR(storage_ptr1->is_valid);
        SERIALIZE_SCALAR(storage_ptr1->valid_io_base);
        SERIALIZE_SCALAR(storage_ptr1->valid_io_limit);
        SERIALIZE_SCALAR(storage_ptr1->valid_memory_base);
        SERIALIZE_SCALAR(storage_ptr1->valid_memory_limit);
        SERIALIZE_SCALAR(storage_ptr1->valid_prefetchable_memory_base);
        SERIALIZE_SCALAR(storage_ptr1->valid_prefetchable_memory_limit);
    }

    void
    PciBridge::unserialize(CheckpointIn &cp)
    {
        printf("unserializing R.C\n");

        UNSERIALIZE_ARRAY(storage_ptr4->storage1.data
        , sizeof(storage_ptr4->storage1.data)
        / sizeof(storage_ptr4->storage1.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr4->storage2.data
        , sizeof(storage_ptr4->storage2.data)
        / sizeof(storage_ptr4->storage2.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr1->storage1.data
        , sizeof(storage_ptr1->storage1.data)
        / sizeof(storage_ptr1->storage1.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr1->storage2.data
        , sizeof(storage_ptr1->storage2.data)
        / sizeof(storage_ptr1->storage2.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr2->storage1.data
        , sizeof(storage_ptr2->storage1.data)
        / sizeof(storage_ptr2->storage1.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr2->storage2.data
        , sizeof(storage_ptr2->storage2.data)
        / sizeof(storage_ptr2->storage2.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr3->storage1.data
        , sizeof(storage_ptr3->storage1.data)
        / sizeof(storage_ptr3->storage1.data[0]));
        UNSERIALIZE_ARRAY(storage_ptr3->storage2.data
        , sizeof(storage_ptr3->storage2.data)
        / sizeof(storage_ptr3->storage2.data[0]));

        UNSERIALIZE_SCALAR(storage_ptr4->is_valid);
        UNSERIALIZE_SCALAR(storage_ptr4->valid_io_base);
        UNSERIALIZE_SCALAR(storage_ptr4->valid_io_limit);
        UNSERIALIZE_SCALAR(storage_ptr4->valid_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr4->valid_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr4->valid_prefetchable_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr4->valid_prefetchable_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr3->is_valid);
        UNSERIALIZE_SCALAR(storage_ptr3->valid_io_base);
        UNSERIALIZE_SCALAR(storage_ptr3->valid_io_limit);
        UNSERIALIZE_SCALAR(storage_ptr3->valid_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr3->valid_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr3->valid_prefetchable_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr3->valid_prefetchable_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr2->is_valid);
        UNSERIALIZE_SCALAR(storage_ptr2->valid_io_base);
        UNSERIALIZE_SCALAR(storage_ptr2->valid_io_limit);
        UNSERIALIZE_SCALAR(storage_ptr2->valid_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr2->valid_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr2->valid_prefetchable_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr2->valid_prefetchable_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr1->is_valid);
        UNSERIALIZE_SCALAR(storage_ptr1->valid_io_base);
        UNSERIALIZE_SCALAR(storage_ptr1->valid_io_limit);
        UNSERIALIZE_SCALAR(storage_ptr1->valid_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr1->valid_memory_limit);
        UNSERIALIZE_SCALAR(storage_ptr1->valid_prefetchable_memory_base);
        UNSERIALIZE_SCALAR(storage_ptr1->valid_prefetchable_memory_limit);

        slavePort.public_sendRangeChange();
        slavePort_DMA1.public_sendRangeChange();
        slavePort_DMA2.public_sendRangeChange();
        slavePort_DMA3.public_sendRangeChange();
    }
} // gem5
