/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 Georgia Tech Research Corporation, INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: George F. Riley<riley@ece.gatech.edu>
 *          Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
 
#include <math.h>

#include "node.h"
#include "node-list.h"
#include "net-device.h"
#include "application.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/object-vector.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/global-value.h"
#include "ns3/boolean.h"
#include "ns3/core-module.h"
#include "ns3/simulator.h"

#include "ns3/double.h"
#include "node-packet-queue.h"

NS_LOG_COMPONENT_DEFINE ("Node");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (Node);

/**
 * \brief A global switch to enable all checksums for all protocols.
 */
static GlobalValue g_checksumEnabled  = GlobalValue ("ChecksumEnabled",
                                                     "A global switch to enable all checksums for all protocols",
                                                     BooleanValue (false),
                                                     MakeBooleanChecker ());

TypeId 
Node::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Node")
    .SetParent<Object> ()
    .AddConstructor<Node> ()
    .AddAttribute ("DeviceList", "The list of devices associated to this Node.",
                   ObjectVectorValue (),
                   MakeObjectVectorAccessor (&Node::m_devices),
                   MakeObjectVectorChecker<NetDevice> ())
    .AddAttribute ("ApplicationList", "The list of applications associated to this Node.",
                   ObjectVectorValue (),
                   MakeObjectVectorAccessor (&Node::m_applications),
                   MakeObjectVectorChecker<Application> ())
    .AddAttribute ("Id", "The id (unique integer) of this Node.",
                   TypeId::ATTR_GET, // allow only getting it.
                   UintegerValue (0),
                   MakeUintegerAccessor (&Node::m_id),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SystemId", "The systemId of this node: a unique integer used for parallel simulations.",
                   TypeId::ATTR_GET|TypeId::ATTR_SET,
                   UintegerValue (0),
                   MakeUintegerAccessor (&Node::m_sid),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}

Node::Node()
  : m_id (0),
    m_sid (0),
		m_initiator(0),
		m_totPacketCount (0),
 		m_Mue(0.0),
		m_Lambda(0.0),
		m_serviceRate(0.0)
{
  NS_LOG_FUNCTION (this);
  Construct ();
}

Node::Node(uint32_t sid)
  : m_id (0),
    m_sid (sid),
		m_initiator(0),
		m_totPacketCount (0),
		m_Mue(0.0),
		m_Lambda(0.0),
		m_serviceRate(0.0)
{ 
  NS_LOG_FUNCTION (this << sid);
  Construct ();
}

void
Node::Construct (void)
{
  NS_LOG_FUNCTION (this);
	
	m_rng = CreateObject<UniformRandomVariable> ();
	
  m_id = NodeList::Add (this);
}

Node::~Node ()
{
  NS_LOG_FUNCTION (this);
}

uint32_t
Node::GetId (void) const
{
  NS_LOG_FUNCTION (this);
  return m_id;
}

uint32_t
Node::GetSystemId (void) const
{
  NS_LOG_FUNCTION (this);
  return m_sid;
}

uint32_t
Node::AddDevice (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);
  uint32_t index = m_devices.size ();
  m_devices.push_back (device);
  device->SetNode (this);
  device->SetIfIndex (index);
  device->SetReceiveCallback (MakeCallback (&Node::NonPromiscReceiveFromDevice, this));
  Simulator::ScheduleWithContext (GetId (), Seconds (0.0), 
                                  &NetDevice::Initialize, device);
  NotifyDeviceAdded (device);
  
  // creating per-node packet counters
  DeviceStats stat;
  stat.RxCount = 0;
  stat.avgPacketSize = 0;
  stat.TxCount = 0;
  
  m_deviceStats.push_back (std::make_pair (device, stat));
  
//  m_devicesPacketCount.insert (std::pair <Ptr<NetDevice>,uint32_t> (device,0));
//	m_devicesAvgPktSize.insert (std::pair <Ptr<NetDevice>,uint32_t> (device,0));
//	m_devicesCumPktCount.insert (std::pair <Ptr<NetDevice>,uint64_t> (device,0));
//	m_devicesMue.insert (std::pair<Ptr<NetDevice>,double> (device, 0.0));
	
  return index;
}
Ptr<NetDevice>
Node::GetDevice (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  NS_ASSERT_MSG (index < m_devices.size (), "Device index " << index <<
                 " is out of range (only have " << m_devices.size () << " devices).");
  return m_devices[index];
}
uint32_t 
Node::GetNDevices (void) const
{
  NS_LOG_FUNCTION (this);
  return m_devices.size ();
}

uint32_t 
Node::AddApplication (Ptr<Application> application)
{
  NS_LOG_FUNCTION (this << application);
  uint32_t index = m_applications.size ();
  m_applications.push_back (application);
  application->SetNode (this);
  Simulator::ScheduleWithContext (GetId (), Seconds (0.0), 
                                  &Application::Initialize, application);
  return index;
}
Ptr<Application> 
Node::GetApplication (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  NS_ASSERT_MSG (index < m_applications.size (), "Application index " << index <<
                 " is out of range (only have " << m_applications.size () << " applications).");
  return m_applications[index];
}
uint32_t 
Node::GetNApplications (void) const
{
  NS_LOG_FUNCTION (this);
  return m_applications.size ();
}

void 
Node::DoDispose ()
{
  NS_LOG_FUNCTION (this);
    // used for testing purposes  
//    uint8_t count = 1;
//    for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
//    {
//      std::cout << int (m_id) << " " << int (count) << " " << (iter->second.avgPacketSize * iter->second.RxCount * 8)/Simulator::Now ().GetSeconds () << std::endl; // calculate bps
//      count++;
//    }	
//    std::cout<<std::endl;
  
  m_deviceAdditionListeners.clear ();
  m_handlers.clear ();
  for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
       i != m_devices.end (); i++)
    {
      Ptr<NetDevice> device = *i;
      device->Dispose ();
      *i = 0;
    }
  m_devices.clear ();
  for (std::vector<Ptr<Application> >::iterator i = m_applications.begin ();
       i != m_applications.end (); i++)
    {
      Ptr<Application> application = *i;
      application->Dispose ();
      *i = 0;
    }
  
  m_applications.clear ();
  m_deviceStats.clear (); // clear device stat map
  
  Object::DoDispose ();
}

// \name for queueing the packets at the nodes
// \{  

  uint16_t 
  Node::GetBufferSize(void)
  {
  	NS_LOG_FUNCTION (this);
	
	  return m_nodePacketBuffer.GetSize();  
  }
  
  uint64_t 
  Node::GetNofPacketsOfDevice (Ptr<NetDevice> device)
  {
	  NS_LOG_FUNCTION (this << device);
	  
	  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
	  {
	    if (iter->first == device)
	    {
        if ((iter->second.RxCount - iter->second.TxCount) == 0)
	        return 1;
	      else
	        return (iter->second.RxCount - iter->second.TxCount);
	    }
	  }
	  return 0;
  }
  
	uint64_t 
	Node::GetAveragePacketSizeOfDevice (Ptr<NetDevice> device)
	{
		NS_LOG_FUNCTION (this << device);
		
	  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
	  {
	    if (iter->first == device)
	      return (iter->second.avgPacketSize);
	  }
	  return 0;
	}
	
	double 
	Node::GetAveragePacketSizeOfRouter (void)
	{
		NS_LOG_FUNCTION (this);
		double averagePacketSize = 0.0;
		uint8_t deviceCount = 0;
		
	  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
    {
      averagePacketSize = averagePacketSize + iter->second.avgPacketSize;
      deviceCount++;
    }		
		
  	return (averagePacketSize/deviceCount);
	}
	
	double 
	Node::GetRouterMue (void)
	{
		NS_LOG_FUNCTION (this);  
  	return m_Mue;
	}
	
	double
	Node::GetRouterServiceRate (void)
	{
		NS_LOG_FUNCTION (this);	
	  return m_serviceRate;
	}
	
	double 
	Node::GetRouterLambda (void)
	{
	  NS_LOG_FUNCTION (this);
	  
	  return (m_totPacketCount / Simulator::Now ().GetSeconds ());	
	}
// \}


void 
Node::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);
  for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
       i != m_devices.end (); i++)
    {
      Ptr<NetDevice> device = *i;
      device->Initialize ();
    }
  for (std::vector<Ptr<Application> >::iterator i = m_applications.begin ();
       i != m_applications.end (); i++)
    {
      Ptr<Application> application = *i;
      application->Initialize ();
    }

  Object::DoInitialize ();
}

void
Node::RegisterProtocolHandler (ProtocolHandler handler, 
                               uint16_t protocolType,
                               Ptr<NetDevice> device,
                               bool promiscuous)
{
  NS_LOG_FUNCTION (this << &handler << protocolType << device << promiscuous);
  struct Node::ProtocolHandlerEntry entry;
  entry.handler = handler;
  entry.protocol = protocolType;
  entry.device = device;
  entry.promiscuous = promiscuous;

  // On demand enable promiscuous mode in netdevices
  if (promiscuous)
    {
      if (device == 0)
        {
          for (std::vector<Ptr<NetDevice> >::iterator i = m_devices.begin ();
               i != m_devices.end (); i++)
            {
              Ptr<NetDevice> dev = *i;
              dev->SetPromiscReceiveCallback (MakeCallback (&Node::PromiscReceiveFromDevice, this));
            }
        }
      else
        {
          device->SetPromiscReceiveCallback (MakeCallback (&Node::PromiscReceiveFromDevice, this));
        }
    }

  m_handlers.push_back (entry);
}

void
Node::UnregisterProtocolHandler (ProtocolHandler handler)
{
  NS_LOG_FUNCTION (this << &handler);
  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->handler.IsEqual (handler))
        {
          m_handlers.erase (i);
          break;
        }
    }
}

bool
Node::ChecksumEnabled (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  BooleanValue val;
  g_checksumEnabled.GetValue (val);
  return val.Get ();
}

bool
Node::PromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                const Address &from, const Address &to, NetDevice::PacketType packetType)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from << &to << packetType);

 	// Note:  This has been modified to buffer packet on Callback Fire
 	//  Author: Janaka Wijekoon <janaka@west.sd.ekio.ac.jp>
 	//  If we use the original Node::ReceiveFromDevice function, uncomment the following line.
 	//  Then the packets will not be buffered. As soon as a packet arrives to the NetDevice,
 	//  the packet will be moved to the Protocol handler for the forwarding process.
  //  return ReceiveFromDevice (device, packet, protocol, from, to, packetType, true);
	
	NodeQueueEntry nodeEnqueueEntry(device,
																	packet,
																	protocol,
																	from,
																	to,
																	packetType,
																	true);
	m_totPacketCount++;
	
  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
  {
    if (iter->first == device)
    {
      if (iter->second.RxCount == 0)
      {
        iter->second.RxCount = iter->second.RxCount + 1;
        iter->second.avgPacketSize = packet->GetSize();
      }
      else
      {
        iter->second.RxCount = iter->second.RxCount + 1;
        iter->second.avgPacketSize = ((iter->second.avgPacketSize * (iter->second.RxCount - 1)) + packet->GetSize()) / (iter->second.RxCount) ;        
      }
    }
  }	

	NS_LOG_DEBUG ("Move the packet " << packet << " to the buffer.");											
	m_nodePacketBuffer.EnQueue(nodeEnqueueEntry);

	if (!m_initiator)
  {
    NS_LOG_DEBUG ("Initiate the packet transmitter.");
    ScheduleTransmit(device);
    m_outEvent = Simulator::Schedule (Seconds (50), 
                                      &Node::PrintStats, 
                                      this);    
    m_initiator = 1;
  }
	
	//Note: As every packet is been buffered, the return is always true
  return true;
}

bool
Node::NonPromiscReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                                   const Address &from)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from);

 	// Note:  This has been modified to buffer packet on Callback Fire
 	//  Author: Janaka Wijekoon <janaka@west.sd.ekio.ac.jp>
 	//  If we use the original Node::ReceiveFromDevice function, uncomment the following line.
 	//  Then the packets will not be buffered. As soon as a packet arrives to the NetDevice,
 	//  the packet will be moved to the Protocol handler for the forwarding process.  
  // return ReceiveFromDevice (device, packet, protocol, from, device->GetAddress (), NetDevice::PacketType (0), false);
	
	NodeQueueEntry nodeEnqueueEntry(device,
																	packet,
																	protocol,
																	from,
																	device->GetAddress (),
																	NetDevice::PacketType (0),
																	false);

	m_totPacketCount++;
	
  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
  {
    if (iter->first == device)
    {
      if (iter->second.RxCount == 0)
      {
        iter->second.RxCount = iter->second.RxCount + 1;
        iter->second.avgPacketSize = packet->GetSize();
      }
      else
      {
        iter->second.RxCount = iter->second.RxCount + 1;
        iter->second.avgPacketSize = ((iter->second.avgPacketSize * (iter->second.RxCount - 1)) + packet->GetSize()) / (iter->second.RxCount); 
      }
    }
  }

	NS_LOG_DEBUG ("Move the packet " << packet << " to the buffer.");
  m_nodePacketBuffer.EnQueue(nodeEnqueueEntry);

	if(!m_initiator)
	{
    NS_LOG_DEBUG ("Initiate the packet transmitter.");
    	
		ScheduleTransmit(device);
    m_outEvent = Simulator::Schedule (Seconds (50), 
                                      &Node::PrintStats, 
                                      this);   		
		m_initiator = 1;
	}
	
	//Note: As every packet is been buffered, the return is always true
	return true;
}

void
Node::ScheduleTransmit(Ptr<NetDevice> device)
{
	NS_LOG_FUNCTION (this);
	
	m_serviceRate = 90000000.0;//90Mbps
	double randValue = 0.0, tempTime = 0.0;
	Time t_reSchedule = Time ();
	
  if (!m_nodePacketBuffer.IsEmpty ())
	{
	  randValue = m_rng->GetValue (0.0, 1.0);
	  m_Mue = m_serviceRate / (GetAveragePacketSizeOfRouter () * 8); // convert it to process a bit

	  tempTime = ((-1/m_Mue) *(log (randValue))) * 1000000 ; // microseconds
	  t_reSchedule = MicroSeconds (tempTime);	
	  
  	ReceiveFromBuffer ();
	  m_nextTransmission = Simulator::Schedule (t_reSchedule, &Node::ScheduleTransmit, this, device);
	}
	else
	{
	  m_nextTransmission = Simulator::Schedule (MilliSeconds (m_rng->GetValue (2.0, 10.0)), 
	                                            &Node::ScheduleTransmit, this, device);
	}
}

void
Node::ReceiveFromBuffer(void)
{
  if (m_nodePacketBuffer.IsEmpty())
  {
	  NS_LOG_DEBUG ("No packet to transmit, reschedule.");
	  return;//break;
  }
  else
  {
    NodeQueueEntry deQueueEntry;

    m_nodePacketBuffer.DeQueue(deQueueEntry);

    NS_LOG_DEBUG (  deQueueEntry.GetNetDevice() 
							      << deQueueEntry.GetPacket() 
							      << deQueueEntry.GetProtocol() 
							      << deQueueEntry.GetFrom() 
							      << deQueueEntry.GetTo() 
							      << deQueueEntry.GetPacketType() 
							      << deQueueEntry.GetPromiscuous());
							
    NS_ASSERT_MSG (Simulator::GetContext () == GetId (), 
								      "Received packet with erroneous context ; " 
								      << "make sure the channels in use are correctly updating events context " 
								      << "when transferring events from one node to another.");
    NS_LOG_DEBUG ("Node " << GetId () 
								      << " ReceiveFromDevice:  dev "
								      << deQueueEntry.GetNetDevice()->GetIfIndex () 
								      << " (type=" 
								      << deQueueEntry.GetNetDevice()->GetInstanceTypeId ().GetName () 
								      << ") Packet UID " 
								      << deQueueEntry.GetPacket()->GetUid ());
								
    for (ProtocolHandlerList::iterator i = m_handlers.begin (); i != m_handlers.end (); i++)
    {
      if (i->device == 0 || (i->device != 0 && i->device == deQueueEntry.GetNetDevice()))
      {
	      if (i->protocol == 0 || i->protocol == deQueueEntry.GetProtocol())
	      {
		      if (deQueueEntry.GetPromiscuous() == i->promiscuous)
		      {
			      i->handler( deQueueEntry.GetNetDevice(),
									      deQueueEntry.GetPacket(),
									      deQueueEntry.GetProtocol(),
									      deQueueEntry.GetFrom(),
									      deQueueEntry.GetTo(),
									      deQueueEntry.GetPacketType());
		      }
	      }
      }
    }
    for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++)
    {
      if (iter->first == deQueueEntry.GetNetDevice())
      {
        iter->second.TxCount = iter->second.TxCount + 1;
        break;          
      }
    }
  }
}

void
Node::PrintStats ()
{
  NS_LOG_FUNCTION (this);

    //used for testing purposes  
  uint64_t totBitCount = 0;
  NS_UNUSED (totBitCount); // suppress "set but not used" compiler warning in optimized builds  
  uint8_t count = 1;
  for (deviceStatI iter = m_deviceStats.begin (); iter != m_deviceStats.end (); iter++, count++)
  {
//    totBitCount = totBitCount + (iter->second.avgPacketSize * iter->second.RxCount * 8);
    if (count != 1) // omit the LOCALHOST interface
    {
      std::cout << int (m_id) 
                << " " << int (count) 
                << " " 
                << ((iter->second.avgPacketSize * 
                     iter->second.RxCount * 8) / 
                     Simulator::Now ().GetSeconds ()) / 1000.0 
                << std::endl; // calculate kbps
    }
  }	
  
//  std::cout << int (m_id) << " " 
//            << int (totBitCount / Simulator::Now ().GetSeconds ()) 
//            << std::endl; // calculate in bps


//	std::cout << int (m_id)
//						<< "  " << GetRouterLambda () / GetRouterMue ()
//						<< std::endl;

  m_outEvent = Simulator::Schedule (Seconds (50), &Node::PrintStats, this);  
}

bool
Node::ReceiveFromDevice (Ptr<NetDevice> device, Ptr<const Packet> packet, uint16_t protocol,
                         const Address &from, const Address &to, NetDevice::PacketType packetType, bool promiscuous)
{
  NS_LOG_FUNCTION (this << device << packet << protocol << &from << &to << packetType << promiscuous);
  NS_ASSERT_MSG (Simulator::GetContext () == GetId (), "Received packet with erroneous context ; " <<
                 "make sure the channels in use are correctly updating events context " <<
                 "when transfering events from one node to another.");
  NS_LOG_DEBUG ("Node " << GetId () << " ReceiveFromDevice:  dev "
                        << device->GetIfIndex () << " (type=" << device->GetInstanceTypeId ().GetName ()
                        << ") Packet UID " << packet->GetUid ());
  bool found = false;

  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->device == 0 ||
          (i->device != 0 && i->device == device))
        {
          if (i->protocol == 0 || 
              i->protocol == protocol)
            {
              if (promiscuous == i->promiscuous)
                {
                  i->handler (device, packet, protocol, from, to, packetType);
                  found = true;
                }
            }
        }
    }
  return found;
}
void 
Node::RegisterDeviceAdditionListener (DeviceAdditionListener listener)
{
  NS_LOG_FUNCTION (this << &listener);
  m_deviceAdditionListeners.push_back (listener);
  // and, then, notify the new listener about all existing devices.
  for (std::vector<Ptr<NetDevice> >::const_iterator i = m_devices.begin ();
       i != m_devices.end (); ++i)
    {
      listener (*i);
    }
}
void 
Node::UnregisterDeviceAdditionListener (DeviceAdditionListener listener)
{
  NS_LOG_FUNCTION (this << &listener);
  for (DeviceAdditionListenerList::iterator i = m_deviceAdditionListeners.begin ();
       i != m_deviceAdditionListeners.end (); i++)
    {
      if ((*i).IsEqual (listener))
        {
          m_deviceAdditionListeners.erase (i);
          break;
         }
    }
}
 
void 
Node::NotifyDeviceAdded (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);
  for (DeviceAdditionListenerList::iterator i = m_deviceAdditionListeners.begin ();
       i != m_deviceAdditionListeners.end (); i++)
    {
      (*i) (device);
    }  
}
 

} // namespace ns3
