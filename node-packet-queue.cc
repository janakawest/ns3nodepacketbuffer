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

#include <vector>

#include "ns3/node-packet-queue.h"
#include "ns3/log.h"


NS_LOG_COMPONENT_DEFINE ("NodePacketQueue");

namespace ns3 {

uint32_t
NodeQueue::GetSize ()
{
	return m_queue.size ();		
}

bool
NodeQueue::IsEmpty()
{
	return (m_queue.size() == 0)?true:false;
}

bool 
NodeQueue::EnQueue (NodeQueueEntry & enEntry)
{
	NS_LOG_FUNCTION (this << enEntry.GetPacket()->GetUid());
	/*for the time being the Maximum length for the queue is not defined.*/

	m_queue.push_back (enEntry);
	return true;
}

bool 
NodeQueue::DeQueue (NodeQueueEntry & deEntry)
{
	NS_LOG_FUNCTION (this);
	
	std::vector<NodeQueueEntry>::iterator i = m_queue.begin (); 
	deEntry = *i;
	m_queue.erase (i);
	
	return true;	
}

}//End of ns3
