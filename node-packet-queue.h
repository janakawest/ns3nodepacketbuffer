/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Hiroaki Nishi Laboratory, Keio University, Japan
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as 
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * Author: Janaka Wijekoon <janaka@west.sd.ekio.ac.jp> 
 */
#ifndef NODE_PACKETQUEUE_H
#define NODE_PACKETQUEUE_H

#include "net-device.h"

#include <vector>  

#include "ns3/ptr.h"
#include "ns3/packet.h" 
#include "ns3/address-utils.h"
#include "ns3/log.h"

namespace ns3 {
class NodeQueueEntry
{
public:
  NodeQueueEntry (Ptr<NetDevice> dev = 0, 
									Ptr<const Packet> pa = 0, 
									uint16_t proto = 0, 
									Address const frm = Address(), 
									Address const to = Address(), 
									NetDevice::PacketType pktType = NetDevice::PacketType(0), 
									bool promscs = 0)
        						:m_device (dev),
        						m_packet (pa),
										m_protocol (proto),
										m_from (frm),
										m_to (to),
										m_packetType (pktType),
										m_promiscuous(promscs)
  {
  }

  /**
  * \brief get and set the packet's forwarding netdevice
  */
	Ptr <NetDevice> GetNetDevice(void) 
	{
		return m_device;
	}
	void SetNetDevice(Ptr <NetDevice> dev)
	{
		m_device = dev;
	}

  /**
  * \brief get and set the packet added to the packet buffer
  */
  Ptr<const Packet> GetPacket (void) const
  {
        return m_packet;
  }
  void SetPacket (Ptr<const Packet> p)
  {
        m_packet = p;
  }

  /**
  * \brief get and set transport layer protocol that the packet is attached on 
  */
	uint16_t GetProtocol(void) const
	{
		return m_protocol;
	} 
	void SetProtocol(uint16_t protocol)
	{
		m_protocol = protocol;
	}

  /**
  * \brief get and set the source address of the packet
  */
  Address GetFrom (void) const
  {
        return m_from;
  }
  void SetFrom (Address from)
  {
        m_from = from;
  }
	
  /**
  * \brief get and set the destination address of the packet
  */
	Address GetTo (void) const
	{
		return m_to;
	}
	void SetTo (Address to)
	{
		m_to = to;
	}

  /**
  * \brief get and set the type of the packet
  */	
	NetDevice::PacketType GetPacketType(void) const
	{
		return m_packetType;
	}
	void SetPacketType(NetDevice::PacketType packetType)
	{
		m_packetType = packetType;
	}

  /**
  * \brief get and set whether the packet is came through a promiscuous  port or not
  */
	bool GetPromiscuous (void) const
	{
		return m_promiscuous;
	}
	void SetPromiscuous (bool promiscuous)
	{
		m_promiscuous = promiscuous;
	}

  /**
  * \brief get and set the size of the queue
  */	
	uint32_t GetQueueSize(void) const
	{
		return queueSize;
	}
	void SetQueueSize(uint32_t size)
	{
		queueSize = size;
	}


private:

	Ptr<NetDevice> m_device; //!< Currosponding NetDevice
  Ptr<const Packet> m_packet; //!< Data packet
	uint16_t m_protocol;//!< Protocol that packet has to be routing with
  Address m_from;//!< From Address 
	Address m_to;//!< to Address
	NetDevice::PacketType m_packetType;//!< Packet Type
	bool m_promiscuous;//!< the interface mode
	uint32_t queueSize;//!< the size of the Queue
};

class NodeQueue
{
public:
	static TypeId GetTypeId (void);
	NodeQueue()
	{
		//Constructor
	}
	~NodeQueue()
	{
		//Destructor
	}
	
  /**
  * \brief push a packet in to the packet queue
  * \param enEntry the queue entry which contains the packet and its meta-data
  */
	bool EnQueue ( NodeQueueEntry & enEntry );

  /**
  * \brief get a packet out of the packet queue
  * as the packet queue is implemented as a FIFO queue, the dqueue method retrieves the top object of the queue 
  * \param dnEntry the queue entry which contains the packet and its meta-data
  */
	bool DeQueue ( NodeQueueEntry & deEntry );

  /**
  * \brief get the size of the queue
  */
	uint32_t GetSize();

  /**
  * \brief \returns if the queue is empty
  */
	bool IsEmpty();

private:
	std::vector<NodeQueueEntry> m_queue; //!< the packet queue vector
};
} // END OF ns3
#endif /* NODRQUEUE_H */
