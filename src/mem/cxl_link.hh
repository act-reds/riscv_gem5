/*
 * Copyright (c) Edward Hanson
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
 * Authors: Edward Hanson (edward.t.hanson@duke.edu)
 *          Code adopted from Krishna Parasuram Srinivasan
 */

/**
 * @file
 * Declarations of classes in a CXL Link
 */


#ifndef __MEM_CXL_LINK_HH__
#define __MEM_CXL_LINK_HH__

#include <deque>
#include <queue>
#include <string>

#include "base/types.hh"
#include "mem/port.hh"

// #include "base/cp_annotate.hh"
// #include "mem/mem_object.hh"
#include "params/CXLLink.hh"
#include "sim/eventq.hh"

/**
 * TLP request and response header size is 12B
 * with 32 bit PCI Express addresses
 */
#define PCIE_REQUEST_HEADER_SIZE 12

#define PCIE_RESPONSE_HEADER_SIZE 12

/**
 * Data Link Layer overhead added to a TLP :
 * Sequence number + LCRC - 6B
 */
#define DLL_OVERHEAD 6

/**
 * Physical layer overhead is 2B, due to framing characterss
 */
#define PHYSICAL_OVERHEAD 2

/**
 * 8b/10b encoding used for Gen 2 PCI Express
 */
#define ENCODING_FACTOR 1.25

/**
 * Size of DLLP is 6B
 */
#define DLLP_SIZE 6

/* --- CXL-specific overheads/constants --- */
/**
 *
 */


/**
 * This header file contains class definitions used in a PCI Express Link.
 * There are 4 main classes declared in this file
 *
 * 1. DataLinkPacket -
 * Used to represent a data link layer packet in Gem5.
 * Data Link Layer packets are generated and consumed at
 * the ends of the PCI Express Link.
 *
 * 2. CXLLinkPacket  -
 * This class is used to represent a packet transmitted
 * across the link. The transmitted packet can be either a
 * Gem5 packet or a DataLinkPacket(DLLP).
 *
 * 3. ReplayBuffer   -
 * This class represents the replay buffer used to implement
 * reliable transmission across a PCI Express Link.
 * Each link interface has a replay buffer. -- REUSED FROM PCIE FILES
 *
 * 4. CXLLink        -
 * This class represents the actual CXL Link in Gem5
 * It consists of 2 link interfaces, with each link
 * interface used to transmit packets from devices attached
 * on that end of the link.
 *
 * It is important to note that in this
 * PCI Express Link implementation
 * , Gem5 packets are considered to represent
 * the Transaction Layer Packets (TLPs)
 * present in the PCI Express protocol.
 * The size of the Transaction Layer packet
 * is based on the size of the Gem5 packet
 * , as well as overheads present in the
 * PCI Express protocol, as defined above.
 * Both TLPs and DLLPs are encapsulated in a
 *  PCIELinkPacket before being sent
 * out onto the unidirectional link attached to a
 * particular link interface.
 *
 * TODO: augment these comments with CXL structures
 */


/**
 * Class used to represent an Ack DLLP.
 * The Ack DLLP indicates that a packet
 * has been successfully received by
 * the device on one end of the link.
 * The Ack DLLP contains a sequence number
 * that indicates the sequence
 * number of the successfully received packet
 */
namespace gem5
{
class DataLinkPacket
{
  public:
    /**
     * This variable stores the sequence
     * number associated with Ack DLLP
     */
    uint64_t seqNum ;

    /**
     * crc contained in Ack DLLP. Not used.
     */
    uint16_t crc ;

    /**
     * Function to return the size of the DLLP,
     * which is a constant value
     * since a DLLP does not contain a payload.
     *
     * @return The size in Bytes of the DLLP
     */
    int
    getSize()
    {
       return DLLP_SIZE ;
    }

    /**
     * Constructor to create a new data link packet (Ack)
     *
     * @param _seq_num
     * The sequence number of the packet acknowledged
      by this Ack
     */

    DataLinkPacket(int seq_num) : seqNum(seq_num)
    {}
} ;


/**
 * Class used to represent a CXL packet transmitted
 * across a link.
 * This packet can either be a Gem5 packet (TLP) or a DLLP.
 * If the packet is a
 * Gem5 packet, the pkt pointer is valid.
 * If the packet is a DLLP, the dllp
 * pointer is valid
 */

class CXLLinkPacket : public PCIELinkPacket
{
  public:

    /**
     * Pointer to a Gem5 packet if the PCIELinkPacket
     * encapsulates a Gem5 pkt
     */
    //PacketPtr pkt ;

    /**
     * Pointer to a DataLinkPacket, if this PCIELinkPacket
     * encapsulates an Ack
     */
    //DataLinkPacket * dllp ;

    /**
     * Does the PCIELinkPAcket encapsulate a TLP ?
     */
    //bool isTLP ;

    /**
     * Does the PCIELinkPacket encapsulate a DLLP ?
     */
    //bool isDLLP ;

    /**
     * Sequence number field of TLP if TLP is encapsulated
     */
    //uint64_t seqNum ;

    /**
     * Constructor for the CXL Link Packet.
     * Assigns the pkt and dllp pointers as NULL
     * Code that creates a new CXLLinkPAcket has to
     * assign variables based on
     * whether a TLP or DLLP is being created
     */
    CXLLinkPacket() ;

    /**
     * Returns the size of the CXLLinkPacket,
     * depending on whether a TLP or
     * DLLP is encapsulated within. Overheads such as
     * header size and sequence
     * numbers and framing characters are taken into account here.
     *
     * @ret The effective size of the
     * CXLLinkPacket (not the size of the class)
     */
    //int getSize() ;

} ;


/**
 * This class represents a CXL Link in Gem5. The
 * CXL link can be visualized
 * as containing 2 interfaces. Each interface is
 * attached through a master/slave
 * port pair to the device on that end of the link.
 * Each interface is attached
 * to a unidirectional link(based on Ethernet Link),
 * that can be used to transmit
 * packets to the other interface. An interface's transmit
 * function is called to
 * send a packet to the other interface,while
 * the interface's receive function
 * is called to receive a packet from the
 * unidirectional link attached to the
 * other interface. Each link interface also
 * contains a replay buffer, to hold
 * unAcked transmitted Gem5 packets
 */
class CXLLink : public ClockedObject
{
  public :
    // Forward Declarations
    class LinkMasterPort ;
    class LinkSlavePort ;

    /**
     * The Link Class used to represent a link
     * interface + unidirectional link
     * that the interface transmits packets on.
     *
     * The unidirectional link is based on
     * the ethernet link in Gem5. It
     * represents a unidirectional CXL link,
     * and can be configured with the
     * number  of lanes and the bandwidth
     * and delay(assumed to be 0). The
     * unidirectional link accepts packets
     * from the link interface via the
     * transmit() function. The unidirectional
     * link schedules an event for
     * a future time based on the size of
     * the CXLLinkPacket and the bandwidth
     * and number of lanes of the link.
     * Once the event occurs, the packet is
     * transmitted to the other interface
     * belonging to the CXLLink.
     *
     * Each link interface accepts Gem5 packets
     * from its master and slave
     * ports, denoted as masterPortSrc and slavePortSrc
     * respectively. These
     * attached master and slave ports are to
     * be connected to a CXL device's
     * slave and master ports respectively.
     * A CXL device in this context refers
     * to either a switch/root port or an endpoint
     * (e.g. NVMe, NIC, etc.)
     *
     * In addition to receiving Gem5 packets from a
     * device, each link interface
     * also receives CXLLinkPackets from the other
     * interface of the CXLLink,
     * via the other interface's assigned unidirectional link. Each
     * CXLLinkPacket can either encapsulate a
     * Gem5 packet or a DataLinkPacket.
     * If the received CXLLinkPacket encapsulates
     * a Gem5 packet(TLP), then the
     * Gem5 packet is sent to either the attached
     * master or slave port based on
     * whether the packet is a request or a response.
     *
     * DataLinkPackets originate from and are consumed by
     * link interfaces. Each
     * DataLinkPacket is an acknowledgement message
     * for a particular TLP(Gem5
     * packet) that is successfully received by a link interface.
     * Successfully
     * received in this context indicates that devices
     * attached to the interface
     * accept the Gem5 packet.
     *
     * Each interface implements a simplified version of the
     * Ack/NAK protocol in
     * the data link layer of real CXL devices.
     * Since unidirectional links are
     * assumed to be error free, only Ack DLLPs are used.
     * When an interface
     * transmits a TLP (Gem5 packet), a pointer to the
     * transmited CXLLinkPacket
     * is stored in its replay buffer.
     * If the other interface receives the TLP
     * successfully, it sends back an Ack to acknowledge
     * this TLP. The sending
     * interface, on receipt of this Ack
     * removes the replay buffer entry
     * corresponding to the CXLLinkPAcket
     *  representing the acknowledged TLP.
     *
     * Each interface also maintains a replay timer.
     * If after the timeout
     * interval, an Ack is not received for a transmitted TLP,
     * all stored
     * CXLLinkPackets are retransmitted from the replay buffer.
     * Conversely,
     * the timer is reset on receipt of an ACk DLLP.
     *
     * An Ack DLLP need not be sent for every received TLP.
     * An ack timer is
     * maintained to send an Ack back for all
     * successfully received TLPs on
     * timer expiration.
     *
     * For more info on Ack/NAK protocol,
     * refer to PCI Express System
     * Architecture book.
     */

    class Link
    {
      public :
        const std::string __name ;

       /**
        * Ptr to parent simobject,
        * used to schedule events in Gem5
        */
        CXLLink * parent ;

        /**
         * Number of ticks a byte
         * takes to be transmitted on the link
         */
        const double ticksPerByte;

        /**
         * Propagation delay across link.
         * Set to 0.
         */
        const Tick linkDelay;

        /**
         * Not used. Set to 0.
         */
        const Tick delayVar;

        /**
         * Pointer to the master port attached
         * to the link interface. Is
         * connected to the slave/PIO port of a
         * em5 device or root/switch port
         */
        LinkMasterPort * masterPortSrc ;

        /**
         * Pointer to the slave port attached to
         *  the link interface. Is
         * connected to the master/DMA port of a
         * Gem5 device or root/switch port
         */
        LinkSlavePort * slavePortSrc ;

        /**
         * Pointer to the master port attached to
         * the other interface of the
         * CXLLink.
         */
        LinkMasterPort * masterPortDest ;

        /**
         * Pointer to the slave port attached to
         * the other interface of the
         * CXLLink.
         */
        LinkSlavePort * slavePortDest ;

        /**
         * Number of lanes making up each unidirectional link.
         * The bandwidth
         * scales proportionately with number of lanes present.
         * The number of
         * lanes making up a link can be 1,2,4,8,12,16 or 32.
         */
        int lanes ;

        /**
         * Packet currently being transmitted
         * on this unidirectional link
         */
        CXLLinkPacket * packet ;

        /**
         * Indicates whether a packet is currently
         * being transmitted on the link
         * @return
         * true if a packet is transmitted
         * on the unidirectional link.
         * false if link is free
         */
        bool
        busy()
        {
            return packet != NULL ;
        }

        /**
         * Queue of inflight packets on the unidirectional link
         */
        std::deque<std::pair<Tick , CXLLinkPacket*>> txQueue ;

        /**
         * Denotes the function called
         * when a CXLLinkPacket is finished
         * being transmitted across
         * the unidirectional link.
         */
        EventFunctionWrapper doneEvent ;

        /**
         * Denotes the function called when a
         * CXLLinkPacket is ready to be sent
         * to the receiving interface
         */
        EventFunctionWrapper txQueueEvent ;

        /**
         * Denotes function called when a
         * timeout occurs and retransmission of
         * packets on the unidirectional
         * link needs to take place.
         */
        EventFunctionWrapper timeoutEvent ;

        /**
         * Denotes function called when an
         * Ack needs to be sent back to the
         * other interface to indicate the
         * successful reception of TLPs (Gem5
         * packets)
         */
        EventFunctionWrapper ackEvent ;

        /**
         * Replay buffer present in the link
         * interface to hold transmitted but
         * unAcked CXLLinkPackets that encapsulate a TLP.
         * Remember that only
         * TLPs(Gem5 packets) need an Ack, not DLLPs.
         */
        ReplayBuffer buffer ;

        /**
         * Link destructor. Needs to be implemented
         */
        ~Link(){}

        /**
         * Function used to transmit CXLLinkPackets on the unidirectional link.
         * The DoneEvent is scheduled after a duration corresponding to the
         * time taken to transmit a CXLLinkPacket on the link, which depends
         * on ticksPerByte, number of lanes present in the unidirectional link,
         * etc. If a packet is currently being transmitted, return false.
         *
         * @param pkt Pointer to CXLLinkPAcket to transmit on unidirect. link.
         * @return successful transmission or not
         */
        bool transmit(CXLLinkPacket * pkt) ;

        /**
         * Function called by an interface's attached master and slave ports to
         * transmit a Gem5 packet on the unidirectional link. Each Gem5 packet
         * is encapsulated within a CXLLinkPacket before being transmitted out
         * onto the link. The CXLLinkPackets encapsulating a Gem5 packet are
         * assigned a sequence number by the sending interface and are stored
         * in the sending interface's replay buffer after transmission.
         * @param pkt Gem5 packet to be transmitted.
         * @param master Is the master port attached to interface sending this
         * packet ?
         * @return Packet successfully transmitted or needs to be resent ?.
         */
        bool linkTransmitInterface(PacketPtr pkt, bool master) ;

        /**
         * Function called by a link to send a packet to a receiving interface.
         * Upon reception of a CXLLinkPAcket encapsulating a TLP, a sequence
         * number check is performed to avoid out of order packets. The Gem5
         * packet is extracted from the CXLLinkPacket, is sent to the attached
         * slave or master port. If the device attached to this interface
         * accepts the Gem5 packet (TLP), an Ack is scheduled to be sent back
         * to the sending interface with the sequence number of the received
         * CXLLinkPacket.
         *
         * Upon reception of a CXLLinkPacket encapsulating a Ack DLLP, entries
         * in the replay buffer are freed based on the sequence number of the
         * Ack. All CXLLinkPAckets with a seq. number < Ack seq. number are
         * freed from the replay buffer as reliable in order transmission of
         * these packets is assumed to have taken place
         *
         * @param packet CXLLinkPAcket received from the unidirectional link.
         */
        void linkReceiveInterface(CXLLinkPacket * packet )  ;

        /**
         * Sequence number to be assigned to TLP before transmission across the
         * link.
         */
        uint64_t sendSeqNum ;

        /**
         * Sequence number of the next TLP to be received from the other
         * interface making up the CXLLink.
         */
        uint64_t recvSeqNum ;

        /**
         * Sequence number of the last packet an Ack was sent for. Used to
         * avoid sending duplicate Acks.
         */
        uint64_t lastAcked ;

        /**
         * Size of the replay buffer present in the link interface
         */
        int  maxQueueSize ;

        /**
         * Time after which the retry timer expires and the contents of the
         * replay buffer are replayed.
         *
         * Consider the following calculations.
         * A=(Maximum payload size * Time to transmit a byte on the link)/Lanes
         * B = AckFactor - 1.4 for x1, 1.4 for x2. 1.4 for x4
         * , 2.5 for x8 and
         *     with 128B Maximum Payload Size
         * C = Internal Delay
         * D = Rx_L0_Adjustment
         *

         * The retryTime is calculated as (A*B + C)*3 + D.
         * In the implemented CXLLink, both the internal delay and
         * Rx_L0_Adjustment times are taken to be 0.
         *
         * Maximum PAyload Size = Gem5 Cache Line Size
         * Time to transmit a byte on the link = ticksPerByte
         * Lanes = Number of lanes configured in
         *  each unidirectional link.
         */
        uint32_t retryTime ;

        /**
         * Was there a Gem5 response packet that could
         * not be sent earlier due
         * to the link being occupied ? Need to resend
         * it once the link frees
         * up ?
         */
        bool retryResp ;

        /**
         * Was there a Gem5 request packet that could
         * not be sent earlier and
         * needs to be resent ?
         */
        bool retryReq ;

        /**
         * TLP which is currently being transmitted on
         * the unidirectional link.
         * Could also represent the next TLP to be
         * transmitted on the
         * unidirectional link if a DLLP or retransmission
         * was already being
         * transmitted on the link when the TLP Gem5 packet
         * was received from
         * the device attached to the interface.
         */
        CXLLinkPacket * replayPacket ;

        /**
         * Next DLLP to be sent out onto the link
         * once the current CXLLinkPAck.
         * is finished being transmitted
         * on the unidirectional link
         */
        CXLLinkPacket * replayDLLP ;

        /**
         * Indicates whether the contents of
         * the replay buffer in the interface
         * need to be replayed.
         */
        bool retransmit ;

        /**
         * Indicates the replay buffer entry
         * to be retransmitted next
         */
        int retransmitIdx ;

        /**
         * Double Pointer to the other Link
         * making up the CXLLink
         */
        Link ** otherLink ;

        /**
         * Function called as soon as a CXLLinkPAcket
         * is finished being
         * transmitted on the unidirectional link.
         * his function calls the
         * linkReceiveInterface() function
         * of the receiving interface.
         */
        void processTxQueue() ;

        /**
         * A CXLLinkPacket is done being transmitted
         * on the unidirectional link
         * This function does the following steps.
         * 1. Set the packet variable to false to
         * indicate that the unidirection
         *    link is free.
         *
         * 2. If the packet which just tranmsitted
         * encapsulates a TLP, then
         *    check if retryReq or retryResp is true.
         * If one of them is set,
         *    then make the master or slave ports of
         * the interface ask for a
         *    packet retransmission from the device
         * attached to this interface.
         *
         * 3. Now choose the next packet to be
         * transmitted on the link, based on
         *    the following priority. P
         * ending ACK DLLP (if any) > Next TLP to be
         *    retransmitted (if any) > Next TLP
         * to be transmitted for the first
         *    time(if any).
         */
        void txDone() ;

        /**
         * Once a CXLLinkPAcket is finished being
         * transmitted on the unidirect.
         * link, this function is invoked.
         * It calls the receive function of
         * the other Link making up the CXLLink.
         *
         * @param pkt The packet that has
         * finished transmission across the
         * unidirectional link and needs to
         * be sent to the opposing link
         * interface.
         */
        void txComplete(CXLLinkPacket * pkt) ;

        /**
         * Function called on expiry of replay timer.
         * Initiates a retransmission
         * of the first CXLLinkPAcket in the replay buffer.
         *
         */
        void timeoutFunc() ;

        /**
         * Function called on expiry of Ack Timer.
         * This function checks if there
         * are any unAcked TLPs that have been received.
         * If so, it sends an
         * Ack to the opposite interface,
         * and updates the value of last_acked.
         */
        void sendAck() ;


        /**
         * Returns the name of this Link.
         * Used for statistics and printing out
         * config and debugging info.
         */
        const std::string name() const { return __name ; }

        /**
         * Link constructor called
         * during creation of a Link instance.
         *
         * @param _name:
         * Name of this Link
         *
         * @param p:
         * Pointer to CXLLink which this Link is a part of
         *
         * @param rate
         * The number of ticks to transmit a byte. The value of
         * ticksperByte.
         *
         * @param delay
         * The propagation delay of a packet transmitted across
         * a unidrectional link
         *
         * @param delay_var
         * The variance in delay.
         *
         * @param master_port_src
         * The LinkMasterPort attached to this Link's
           interface.
         *
         * @param slave_port_src
         * The LinkSlavePort attached to this Link's
           interface.
         *
         * @param master_port_dest
         * The LinkMasterPort attached to the opposite
         * interface.
         *
         * @param slave_port_dest
         * The LinkSlavePort attached to the opposite
         * interface.
         *
         * @param num_lanes
         * The number of lanes in the unidirectional link.
         *
         * @param mps :
         * The maximum payload size of a TLP tranmsiited across
           the unidirectional link. Used to calculate retryTime.
         *
         * @param opposite_link
         * A pointer to the other link making up the
         *        CXLLink
         */

        Link (const std::string & _name, CXLLink * p,
              double rate, Tick delay,
              Tick delay_var,
              LinkMasterPort * master_port_src ,
              LinkSlavePort * slave_port_src,
              LinkMasterPort * master_port_dest,
              LinkSlavePort * slave_port_dest,
              int num_lanes,int max_queue_size,
              int mps, Link ** opposite_link );
    } ;

    /**
     * Class that represents
     * the Master Port attached to a Link. This Master
     * Port is connected to the
     * Slave Port of a device attached to one end of
     * the CXLLink.
     */
    class LinkMasterPort : public Requestport
    {
      public:
        /**
         * Called by the peer Slave Port
         * belonging to a connected device to
         * send a Gem5 packet to the Link Master Port.
         *
         * @param pkt The Gem5 packet to be received.
         */
        bool recvTimingResp(PacketPtr pkt) ;

        void recvFromLink(CXLLinkPacket * pkt){} ;

        void recvReqRetry(){}

        /**
         * Pointer to parent CXLLink
         */
        CXLLink * parent ;

        /**
         * Double Pointer to the Link
         * this LinkMasterPort is attached to.
         */
        Link ** transmitLink ;

        /**
         * Constructor
         * @param _name The name of this LinkMasterPort
         * @param parent_ptr The parent CXLLink.
         * @param transmit_link_dptr
         * The Link this LinkMasterPort is attached to
         */
        LinkMasterPort(const std::string & _name, CXLLink * parent_ptr,
                       Link ** transmit_link_dptr) ;
    } ;

    /**
     * Class that represents the Slave Port attached to a Link.
     * This Slave
     * Port is connected to the Master Port of a device
     * attached to one end of
     * the CXLLink.
     */
    class LinkSlavePort : public ResponsePort
    {
      public:

        void recvRespRetry(){}

        /**
         * Since PCI Express consists of a serial link
         * , and not a bus, there is
         * no need for address mapping of devices connected to the link.
         *
         * @retval Returns an empty AddrRangeList
         */
        AddrRangeList getAddrRanges() const
        { AddrRangeList temp ; return temp;}

        /**
         * Function called by the peer
         * Master Port belonging to a connected PCI
         * device or switch/root port.
         * This functions accepts a timing request
         * packet
         *
         * @param pkt Timing Req packet which is received.
         */
        bool recvTimingReq(PacketPtr pkt) ;


        /**
         * Function to receive a functional
         * request packet from peer MAster Port
         * Just passes packet along without modelling timing or delay
         */
        void recvFunctional(PacketPtr pkt) ;

        /** Function to receive an atomic request packet
         * from peer Slave Port.
         *  Just passes packet along without
         * modelling timing or delay
         */
        Tick recvAtomic(PacketPtr pkt) ;

        /**
         * Pointer to parent CXLLink
         */
        CXLLink * parent ;

        /**
         * Double Ptr to Link this LinkSlavePort is attached to.
         */
        Link ** transmitLink ;

        /**
         * Constructor.
         * @param _name
         * Name of the LinkSlavePort instance
         * @param _parent
         * pointer to parent CXLLink.
         * @param transmit_link_dptr
         * Double Ptr to the Link this LinkSlavePort
         * is attached to.
         */
        LinkSlavePort(const std::string & _name, CXLLink * parent_ptr,
                      Link ** transmit_link_dptr) ;
    } ;

    /**
     * Upstream and Downstream Slave ports
     * used to connect the CXLLink to the
     * Master Ports of connected devices.
     * Upstream refers to direction of CPU
     * and downstream away from the CPU.
     * One slave port is assigned to each
     * of the 2 Links that make up the CXLlink.
     */
    LinkSlavePort upstreamSlave, downstreamSlave ;

    /**
     * Upstream and Downstream Master ports
     * used to connect the CXLLink to the
     * Slave Ports of connected devices.
     * Upstream refers to direction of CPU
     * and downstream away from the CPU.
     * One master port is assigned to each of
     * the 2 Links that make up the CXLLink.
     */
    LinkMasterPort upstreamMaster , downstreamMaster ;

    /**
     * Representing the 2 Links that make up a CXLLink.
     * These are created
     * on the heap.
     */
    Link * links[2] ;

    /**
     * Check connectivity of all 4 ports
     * belonging to the CXLLink
     */
    void init() ;

    /**
     * Returns a reference to the Port based on name
     */
    virtual Port& getPort(const std::string &if_name, PortID idx) ;

    /**
     * Returns a reference to the LinkMasterPort based on name
     */
    Requestport& getMasterPort(const std::string &if_name, PortID idx) ;

    /**
     * Returns a reference to a LinkSlavePort based on name
     */
    ResponsePort&  getSlavePort(const std::string &if_name, PortID idx) ;

    typedef CXLLinkParams Params ;

    /**
     * Constructor for CXLLink class.
     * Dynamically creates the 2 Links that make up a CXLLink.
     * @param p Used to initialize the CXLLink based on python configuration
     */
     CXLLink (const Params &p) ;
} ;
}//gem5
#endif //__MEM_CXL_LINK_HH__
