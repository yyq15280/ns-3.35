/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Natale Patriciello <natale.patriciello@gmail.com>
 *               2016 Stefano Avallone <stavallo@unina.it>
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
 */

#include "traffic-control-layer.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/log.h"
#include "ns3/object-map.h"
#include "ns3/packet.h"
#include "ns3/socket.h"
#include "ns3/queue-disc.h"
#include <tuple>
#include "ns3/tcp-header.h"
#include "ns3/ipv4-header.h"
#include "ns3/simulator.h"
#include "ns3/ipv4-packet-filter.h"
#include "ns3/ipv4-queue-disc-item.h"
#include "ns3/wifi-module.h"
#include "ns3/point-to-point-net-device.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("TrafficControlLayer");

NS_OBJECT_ENSURE_REGISTERED (TrafficControlLayer);

/******************
 * FlowState
 *****************/
TypeId FlowState::GetTypeId (void)
{
	static TypeId tid = TypeId ("ns3::FlowState")
		.SetParent<Object> ()
		;
	return tid;
}

FlowState::FlowState(){
	m_windowSize = 10;
	m_isSlowStart = true;
  m_reduceSeq = SequenceNumber32(0);
}


/***********************
 * TrafficControlLaye
 **********************/
TypeId
TrafficControlLayer::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::TrafficControlLayer")
    .SetParent<Object> ()
    .SetGroupName ("TrafficControl")
    .AddConstructor<TrafficControlLayer> ()
    .AddAttribute ("RootQueueDiscList", "The list of root queue discs associated to this Traffic Control layer.",
                   ObjectMapValue (),
                   MakeObjectMapAccessor (&TrafficControlLayer::GetNDevices,
                                          &TrafficControlLayer::GetRootQueueDiscOnDeviceByIndex),
                   MakeObjectMapChecker<QueueDisc> ())
    .AddAttribute ("Ccmode",
                   "True to modify window field in TCP header",
                   UintegerValue (0),
                   MakeUintegerAccessor (&TrafficControlLayer::m_ccmode),
                   MakeUintegerChecker <uint32_t> ())
                       .AddAttribute ("C", "Cubic Scaling factor",
                   DoubleValue (10000),
                   MakeDoubleAccessor (&TrafficControlLayer::m_c),
                   MakeDoubleChecker <double> (0.0))
    .AddAttribute ("FlowNumber", "Basic flow add burst flow",
                   UintegerValue (21),
                   MakeUintegerAccessor (&TrafficControlLayer::m_flowNumber),
                   MakeUintegerChecker <uint16_t> ())
    .AddAttribute ("g", "the weight given to new samples against the past in the estimation of alpha",
                   DoubleValue (0.0625),
                   MakeDoubleAccessor (&TrafficControlLayer::m_g),
                   MakeDoubleChecker <double> (0.0))
    .AddAttribute ("IsProbMark",
                   "True to use probability mark",
                   BooleanValue (false),
                   MakeBooleanAccessor (&TrafficControlLayer::m_isProbMark),
                   MakeBooleanChecker ())
    .AddAttribute ("Pmax", "the maximum marking probability",
                   DoubleValue (1.0),
                   MakeDoubleAccessor (&TrafficControlLayer::m_Pmax),
                   MakeDoubleChecker <double> (0.0))
    .AddAttribute ("Kmax", "the maximum marking threshold",
                   UintegerValue (105),
                   MakeUintegerAccessor (&TrafficControlLayer::m_Kmax),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("Kmin", "the minimum mark threshold",
                   UintegerValue (20),
                   MakeUintegerAccessor (&TrafficControlLayer::m_Kmin),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("MSS", "maximum segment size",
                   UintegerValue (1448),
                   MakeUintegerAccessor (&TrafficControlLayer::m_MSS),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("Eta", "maximum segment size",
                   DoubleValue (0.995),
                   MakeDoubleAccessor (&TrafficControlLayer::m_eta),
                   MakeDoubleChecker <double> (0.0))
    .AddAttribute ("Tau", "reduce the queue from qLen to Kmin after Tau time, ms",
                   UintegerValue (100),
                   MakeUintegerAccessor (&TrafficControlLayer::m_tau),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("DeviceRate", "the device rate, Mbps",
                   UintegerValue (650),
                   MakeUintegerAccessor (&TrafficControlLayer::m_deviceRate),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("IsWired",
                   "True, wired network; false, wireless network",
                   BooleanValue (false),
                   MakeBooleanAccessor (&TrafficControlLayer::m_isWired),
                   MakeBooleanChecker ())
  ;
  return tid;
}

TypeId
TrafficControlLayer::GetInstanceTypeId (void) const
{
  return GetTypeId ();
}

TrafficControlLayer::TrafficControlLayer ()
  : Object ()
{
  NS_LOG_FUNCTION (this);
}

TrafficControlLayer::~TrafficControlLayer ()
{
  NS_LOG_FUNCTION (this);
}

void
TrafficControlLayer::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_node = 0;
  m_handlers.clear ();
  m_netDevices.clear ();
  Object::DoDispose ();
}

void
TrafficControlLayer::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);

  ScanDevices ();

  // initialize the root queue discs
  for (auto& ndi : m_netDevices)
    {
      if (ndi.second.m_rootQueueDisc)
        {
          ndi.second.m_rootQueueDisc->Initialize ();
        }
    }

  for(uint32_t i = 0; i < m_flowNumber; i++)
  {
    m_windowSize.push_back(10);  // MSS
    m_isSlowStart.push_back(true);
    m_Wmax.push_back(0);
    m_Wmin.push_back(0);
    m_epochStart.push_back(Time::Min ());
    m_updateSeq.push_back(SequenceNumber32(9*1448+1));
    m_reduceSeq.push_back(SequenceNumber32(0));
    m_totalPackets.push_back(0);
    m_markedPackets.push_back(0);
    m_alpha.push_back(0.0);
    m_isCongestion.push_back(false);
    m_canCalcRtt.push_back(true);
    m_calcRttSeqNum.push_back(SequenceNumber32(0));
    m_calcRttStartTime.push_back(Time::Min ());
    m_rtt.push_back(-1.0);
    m_beta.push_back(1.0);
    m_rate.push_back(-1.0);
  }

  m_targetRate = m_deviceRate;
  m_totalDqRate = 0.0;
  m_lastSendBytes = 0;
  m_lastSendTime = 2.0;

  Object::DoInitialize ();
}

void
TrafficControlLayer::RegisterProtocolHandler (Node::ProtocolHandler handler,
                                              uint16_t protocolType, Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << protocolType << device);

  struct ProtocolHandlerEntry entry;
  entry.handler = handler;
  entry.protocol = protocolType;
  entry.device = device;
  entry.promiscuous = false;

  m_handlers.push_back (entry);

  NS_LOG_DEBUG ("Handler for NetDevice: " << device << " registered for protocol " <<
                protocolType << ".");
}

void
TrafficControlLayer::ScanDevices (void)
{
  NS_LOG_FUNCTION (this);

  NS_ASSERT_MSG (m_node, "Cannot run ScanDevices without an aggregated node");

  NS_LOG_DEBUG ("Scanning devices on node " << m_node->GetId ());
  for (uint32_t i = 0; i < m_node->GetNDevices (); i++)
    {
      NS_LOG_DEBUG ("Scanning devices on node " << m_node->GetId ());
      Ptr<NetDevice> dev = m_node->GetDevice (i);
      NS_LOG_DEBUG ("Checking device " << i << " with pointer " << dev << " of type " << dev->GetInstanceTypeId ().GetName ());

      // note: there may be no NetDeviceQueueInterface aggregated to the device
      Ptr<NetDeviceQueueInterface> ndqi = dev->GetObject<NetDeviceQueueInterface> ();
      NS_LOG_DEBUG ("Pointer to NetDeviceQueueInterface: " << ndqi);

      std::map<Ptr<NetDevice>, NetDeviceInfo>::iterator ndi = m_netDevices.find (dev);

      if (ndi != m_netDevices.end ())
        {
          NS_LOG_DEBUG ("Device entry found; installing NetDeviceQueueInterface pointer " << ndqi << " to internal map");
          ndi->second.m_ndqi = ndqi;
        }
      else if (ndqi)
      // if no entry for the device is found, it means that no queue disc has been
      // installed. Nonetheless, create an entry for the device and store a pointer
      // to the NetDeviceQueueInterface object if the latter is not null, because
      // the Traffic Control layer checks whether the device queue is stopped even
      // when there is no queue disc.
        {
          NS_LOG_DEBUG ("No device entry found; create entry for device and store pointer to NetDeviceQueueInterface: " << ndqi);
          m_netDevices[dev] = {nullptr, ndqi, QueueDiscVector ()};
          ndi = m_netDevices.find (dev);
        }

      // if a queue disc is installed, set the wake callbacks on netdevice queues
      if (ndi != m_netDevices.end () && ndi->second.m_rootQueueDisc)
        {
          NS_LOG_DEBUG ("Setting the wake callbacks on NetDevice queues");
          ndi->second.m_queueDiscsToWake.clear ();

          if (ndqi)
            {
              for (uint16_t i = 0; i < ndqi->GetNTxQueues (); i++)
                {
                  Ptr<QueueDisc> qd;

                  if (ndi->second.m_rootQueueDisc->GetWakeMode () == QueueDisc::WAKE_ROOT)
                    {
                      qd = ndi->second.m_rootQueueDisc;
                    }
                  else if (ndi->second.m_rootQueueDisc->GetWakeMode () == QueueDisc::WAKE_CHILD)
                    {
                      NS_ABORT_MSG_IF (ndi->second.m_rootQueueDisc->GetNQueueDiscClasses () != ndqi->GetNTxQueues (),
                                      "The number of child queue discs does not match the number of netdevice queues");

                      qd = ndi->second.m_rootQueueDisc->GetQueueDiscClass (i)->GetQueueDisc ();
                    }
                  else
                    {
                      NS_ABORT_MSG ("Invalid wake mode");
                    }

                  ndqi->GetTxQueue (i)->SetWakeCallback (MakeCallback (&QueueDisc::Run, qd));
                  ndi->second.m_queueDiscsToWake.push_back (qd);
                }
            }
          else
            {
              ndi->second.m_queueDiscsToWake.push_back (ndi->second.m_rootQueueDisc);
            }

          // set the NetDeviceQueueInterface object and the SendCallback on the queue discs
          // into which packets are enqueued and dequeued by calling Run
          for (auto& q : ndi->second.m_queueDiscsToWake)
            {
              q->SetNetDeviceQueueInterface (ndqi);
              q->SetSendCallback ([dev] (Ptr<QueueDiscItem> item)
                                  { dev->Send (item->GetPacket (), item->GetAddress (), item->GetProtocol ()); });
            }
        }
    }
}

void
TrafficControlLayer::SetRootQueueDiscOnDevice (Ptr<NetDevice> device, Ptr<QueueDisc> qDisc)
{
  NS_LOG_FUNCTION (this << device << qDisc);

  std::map<Ptr<NetDevice>, NetDeviceInfo>::iterator ndi = m_netDevices.find (device);

  if (ndi == m_netDevices.end ())
    {
      // No entry found for this device. Create one.
      m_netDevices[device] = {qDisc, nullptr, QueueDiscVector ()};
    }
  else
    {
      NS_ABORT_MSG_IF (ndi->second.m_rootQueueDisc,
                       "Cannot install a root queue disc on a device already having one. "
                       "Delete the existing queue disc first.");

      ndi->second.m_rootQueueDisc = qDisc;
    }
}

Ptr<QueueDisc>
TrafficControlLayer::GetRootQueueDiscOnDevice (Ptr<NetDevice> device) const
{
  NS_LOG_FUNCTION (this << device);

  std::map<Ptr<NetDevice>, NetDeviceInfo>::const_iterator ndi = m_netDevices.find (device);

  if (ndi == m_netDevices.end ())
    {
      return 0;
    }
  return ndi->second.m_rootQueueDisc;
}

Ptr<QueueDisc>
TrafficControlLayer::GetRootQueueDiscOnDeviceByIndex (uint32_t index) const
{
  NS_LOG_FUNCTION (this << index);
  return GetRootQueueDiscOnDevice (m_node->GetDevice (index));
}

void
TrafficControlLayer::DeleteRootQueueDiscOnDevice (Ptr<NetDevice> device)
{
  NS_LOG_FUNCTION (this << device);

  std::map<Ptr<NetDevice>, NetDeviceInfo>::iterator ndi = m_netDevices.find (device);

  NS_ASSERT_MSG (ndi != m_netDevices.end () && ndi->second.m_rootQueueDisc != 0,
                 "No root queue disc installed on device " << device);

  // remove the root queue disc
  ndi->second.m_rootQueueDisc = 0;
  for (auto& q : ndi->second.m_queueDiscsToWake)
    {
      q->SetNetDeviceQueueInterface (nullptr);
      q->SetSendCallback (nullptr);
    }
  ndi->second.m_queueDiscsToWake.clear ();

  Ptr<NetDeviceQueueInterface> ndqi = ndi->second.m_ndqi;
  if (ndqi)
    {
      // remove configured callbacks, if any
      for (uint16_t i = 0; i < ndqi->GetNTxQueues (); i++)
        {
          ndqi->GetTxQueue (i)->SetWakeCallback (MakeNullCallback <void> ());
        }
    }
  else
    {
      // remove the empty entry
      m_netDevices.erase (ndi);
    }
}

void
TrafficControlLayer::SetNode (Ptr<Node> node)
{
  NS_LOG_FUNCTION (this << node);
  m_node = node;
}

void
TrafficControlLayer::NotifyNewAggregate ()
{
  NS_LOG_FUNCTION (this);
  if (m_node == 0)
    {
      Ptr<Node> node = this->GetObject<Node> ();
      //verify that it's a valid node and that
      //the node was not set before
      if (node != 0)
        {
          this->SetNode (node);
        }
    }
  Object::NotifyNewAggregate ();
}

uint32_t
TrafficControlLayer::GetNDevices (void) const
{
  return m_node->GetNDevices ();
}


void
TrafficControlLayer::Receive (Ptr<NetDevice> device, Ptr<const Packet> p,
                              uint16_t protocol, const Address &from, const Address &to,
                              NetDevice::PacketType packetType)
{
  NS_LOG_FUNCTION (this << device << p << protocol << from << to << packetType);

  bool found = false;

  for (ProtocolHandlerList::iterator i = m_handlers.begin ();
       i != m_handlers.end (); i++)
    {
      if (i->device == 0
          || (i->device != 0 && i->device == device))
        {
          if (i->protocol == 0
              || i->protocol == protocol)
            {
              NS_LOG_DEBUG ("Found handler for packet " << p << ", protocol " <<
                            protocol << " and NetDevice " << device <<
                            ". Send packet up");
              i->handler (device, p, protocol, from, to, packetType);
              found = true;
            }
        }
    }

  NS_ABORT_MSG_IF (!found, "Handler for protocol " << p << " and device " << device <<
                           " not found. It isn't forwarded up; it dies here.");
}

void
TrafficControlLayer::ModifyRwnd (Ptr<QueueDiscItem> item, Ptr<QueueDisc> qDisc, Ptr<NetDevice> device)
{
  TcpHeader tcpHdr;
  item->GetPacket ()->PeekHeader (tcpHdr);

  // Three-way Handshake, SYN->(SYN,ACK)->ACK, then all data packets have ACK flag
  if (tcpHdr.GetFlags () & TcpHeader::ACK && !(tcpHdr.GetFlags () & TcpHeader::SYN)) {
    // empty packet, namely the real ACK
    if (item->GetPacket()->GetSize() == 4*tcpHdr.GetLength()) {
      uint16_t flowID = tcpHdr.GetSourcePort () - 50000;
      if (flowID < m_flowNumber)
      {       
        item->GetPacket ()->RemoveHeader (tcpHdr);
        uint16_t rwndsize = CalcRwnd(tcpHdr, flowID);
        tcpHdr.SetWindowSize(rwndsize);
        item->GetPacket ()->AddHeader (tcpHdr);

        if (m_canCalcRtt[flowID])
        {
          m_calcRttSeqNum[flowID] = tcpHdr.GetAckNumber () + (m_windowSize[flowID] - 1) * 1448;
          m_calcRttStartTime[flowID] = Simulator::Now();
          m_canCalcRtt[flowID] = false;
        }
      }       
    }
    else {
      uint16_t flowID = tcpHdr.GetDestinationPort () - 50000;
      m_totalPackets[flowID]+=1;
      
      if (!m_canCalcRtt[flowID])
      {
        if (tcpHdr.GetSequenceNumber() >= m_calcRttSeqNum[flowID])
        {
          double rtt_tmp = Simulator::Now().GetSeconds() - m_calcRttStartTime[flowID].GetSeconds();
          if (rtt_tmp > 0.008) 
          {
            rtt_tmp += 0.001;  // add 1ms because of wifi propagation delay
            if (m_rtt[flowID] < 0) {
              m_rtt[flowID] = rtt_tmp;
            }
            else
            {
              // if (rtt_tmp < m_rtt[flowID])
              // {
              //   m_rtt[flowID] = rtt_tmp;
              // }
              m_rtt[flowID] = 0.875 * m_rtt[flowID] + 0.125 * rtt_tmp;
            }
            UpdateBeta();
          }
          m_canCalcRtt[flowID] = true;
        }
      }
      
      Ipv4Header ipv4Hdr = DynamicCast<Ipv4QueueDiscItem>(item)->GetHeader();
      
      uint32_t qLen;
      if (m_isWired) {
        Ptr<Queue<Packet>> queue = DynamicCast<PointToPointNetDevice>(device)->GetQueue();
        qLen = qDisc->GetCurrentSize().GetValue() + queue->GetCurrentSize().GetValue();
      }
      else {
        Ptr<ns3::WifiMacQueue> queue = DynamicCast<RegularWifiMac>(DynamicCast<WifiNetDevice>(device)->GetMac())->GetTxopQueue(AC_BE);
        qLen = qDisc->GetCurrentSize().GetValue() + queue->GetCurrentSize().GetValue();
      }

      bool isMark = false;
      if (m_isProbMark) 
      {
        m_uv = CreateObject<UniformRandomVariable> ();
        double u = m_uv->GetValue ();
        double vProb = 0.0;
        if (qLen >= m_Kmin) {
          if (qLen > m_Kmax) {
            vProb = 1.0;
          }
          else {
            vProb = m_Pmax / (m_Kmax - m_Kmin) * (qLen - m_Kmin);
          }
        }
        if (u < vProb) {
            isMark = true;
          }
      }
      else
      {
        if (qLen >= m_Kmin) {
          isMark = true;
        }
      }  

      if ((ipv4Hdr.GetTos() & 0x03) == 0x03 || isMark)
      // if ((ipv4Hdr.GetTos() & 0x03) == 0x03)
      {
        m_markedPackets[flowID]+=1;
        m_isCongestion[flowID] = true;
        if ((ipv4Hdr.GetTos() & 0x03) == 0x03)
        {
          ipv4Hdr.SetTos(ipv4Hdr.GetTos() & 0xfc);
          DynamicCast<Ipv4QueueDiscItem>(item)->SetHeader(ipv4Hdr);
        }          
      }

      if (tcpHdr.GetSequenceNumber() >= m_updateSeq[flowID])
      {
        UpdateAlpha(tcpHdr, flowID);
      }
    }
  }
}

void 
TrafficControlLayer::UpdateBeta(void)
{
  double rate_sum = 0.0;
  uint32_t valid_flow_num = 0;
  for(uint32_t i = 0; i < m_flowNumber; i++) {
    if (m_rtt[i] > 0) {
      m_rate[i] = m_windowSize[i] / m_rtt[i];
      rate_sum += m_rate[i];
      valid_flow_num += 1;
    }
  }

  for(uint32_t i = 0; i < m_flowNumber; i++) {
    if (m_rtt[i] > 0) {
      if (valid_flow_num == 1) {
        m_beta[i] = 1.0;
      }
      else {
        m_beta[i] = (1 - m_rate[i] / rate_sum) * (valid_flow_num / (valid_flow_num - 1));
      }       
    }
  }

}

void
TrafficControlLayer::UpdateAlpha (TcpHeader& tcpHeader, uint16_t flowID)
{
  m_updateSeq[flowID] = tcpHeader.GetSequenceNumber () + m_windowSize[flowID] * m_MSS;
  double F = (double)m_markedPackets[flowID] / m_totalPackets[flowID];
  m_alpha[flowID] = (1 - m_g) * m_alpha[flowID] +m_g * F; 
  m_markedPackets[flowID] = 0;
  m_totalPackets[flowID] = 0;
}

uint16_t
TrafficControlLayer::CalcRwnd (TcpHeader& tcpHeader, uint16_t flowID)
{
  if (m_isCongestion[flowID])
  {
    m_epochStart[flowID] = Simulator::Now ();
    if (tcpHeader.GetAckNumber () > m_reduceSeq[flowID]) 
    {
      m_Wmax[flowID] = m_windowSize[flowID];
      m_reduceSeq[flowID] = tcpHeader.GetAckNumber () + (m_windowSize[flowID] - 1) * m_MSS;
      // m_windowSize[flowID] = m_windowSize[flowID] * (1 - m_alpha[flowID] / 2); 
      m_windowSize[flowID] = m_windowSize[flowID] * (1 - std::pow(m_alpha[flowID], m_beta[flowID]) / 2); 
      m_Wmin[flowID] = m_windowSize[flowID];
      m_isSlowStart[flowID] = false;
      m_isCongestion[flowID] = false;
    }
  }
  else if (m_isSlowStart[flowID])
  {
    m_windowSize[flowID]+=1;
  }
  else
  {
    double K = std::pow ((m_Wmax[flowID] - m_Wmin[flowID]) / m_c, 1 / 3.);
    Time t = Simulator::Now () - m_epochStart[flowID];
    double tagetWindowSize = m_c * std::pow ((t.GetSeconds () - K), 3) + m_Wmax[flowID];
    m_windowSize[flowID] = (tagetWindowSize - m_windowSize[flowID]) * m_beta[flowID] + m_windowSize[flowID];
  }

  m_windowSize[flowID] = std::max (m_windowSize[flowID], 2U);
  uint16_t rwndsize;
  if (m_windowSize[flowID] * m_MSS % 2048 == 0)
  {
    rwndsize = m_windowSize[flowID] * m_MSS / 2048;
  } 
  else
  {
    rwndsize = m_windowSize[flowID] * m_MSS / 2048 + 1;
  }
  return rwndsize; 
}


// APCC
Ptr<FlowState> 
TrafficControlLayer::GetFlow(uint32_t sip, uint16_t sport, bool create){
	uint64_t key = ((uint64_t)sip << 32) | (uint64_t)sport;
	auto it = m_flowTable.find(key);
	if (it != m_flowTable.end())
		return it->second;
	if (create){
		// create new flow state
		Ptr<FlowState> f = CreateObject<FlowState>();
		// init the flow
		f->m_windowSize = 10;
		f->m_isSlowStart = true;
    f->m_reduceSeq = SequenceNumber32(0);
		// store in map
		m_flowTable[key] = f;
		return f;
	}
	return NULL;
}

void
TrafficControlLayer::UseAPCC (Ptr<QueueDiscItem> item, Ptr<QueueDisc> qDisc, Ptr<NetDevice> device)
{
  if (item->GetProtocol() != 2048) return ;  // 2048: IP protocol
  
  TcpHeader tcpHdr;
  item->GetPacket ()->PeekHeader (tcpHdr); 
  Ipv4Header ipv4Hdr = DynamicCast<Ipv4QueueDiscItem>(item)->GetHeader();

  // Three-way Handshake, SYN->(SYN,ACK)->ACK, then all data packets have ACK flag
  if (tcpHdr.GetFlags () & TcpHeader::SYN) return ;

  // empty packet, namely the real ACK
  if (item->GetPacket()->GetSize() == 4*tcpHdr.GetLength()) {
    Ptr<FlowState> flow = GetFlow(ipv4Hdr.GetSource().Get(), tcpHdr.GetSourcePort(), true);
    item->GetPacket ()->RemoveHeader (tcpHdr);
    uint16_t rwndsize = CalcRwndAPCC(flow, tcpHdr);
    tcpHdr.SetWindowSize(rwndsize);
    item->GetPacket ()->AddHeader (tcpHdr);
  }

  // data packets
  else {
    Ptr<FlowState> flow = GetFlow(ipv4Hdr.GetDestination().Get(), tcpHdr.GetDestinationPort(), true);
    uint32_t qLen;
    if (m_isWired) {
      Ptr<Queue<Packet>> queue = DynamicCast<PointToPointNetDevice>(device)->GetQueue();
      qLen = qDisc->GetCurrentSize().GetValue() + queue->GetCurrentSize().GetValue();
      calcTotalDqRate(DynamicCast<PointToPointNetDevice>(device));
    }
    else {
      Ptr<ns3::WifiMacQueue> queue = DynamicCast<RegularWifiMac>(DynamicCast<WifiNetDevice>(device)->GetMac())->GetTxopQueue(AC_BE);
      qLen = qDisc->GetCurrentSize().GetValue() + queue->GetCurrentSize().GetValue();
      calcTotalDqRate(queue);
    }
    
    if (!flow->m_isSlowStart) {
      calcTargetRate(qLen);
    }
    else {
      if (qLen > m_Kmin && m_totalDqRate > 600) {
        flow->m_isSlowStart = false;
      }
    }   
  }
}

void
TrafficControlLayer::calcTargetRate(uint32_t qLen) {
  if (qLen < m_Kmin) {
    m_targetRate = m_eta * m_deviceRate;
  }
  else {
    m_targetRate = m_eta * m_deviceRate - (qLen - m_Kmin) * 1.5 * 8 / m_tau;
    m_targetRate = std::max(10.0, m_targetRate);
  }
}

void
TrafficControlLayer::calcTotalDqRate(Ptr<PointToPointNetDevice> device) {
  double sendTime = Simulator::Now().GetSeconds();
  uint64_t sendBytes = device->m_sendBytes;
  if (sendBytes == m_lastSendBytes || sendTime - m_lastSendTime < 0.01) return ;
  m_totalDqRate = (sendBytes - m_lastSendBytes) * 8 / (sendTime - m_lastSendTime) / 1000000;
  m_lastSendBytes = sendBytes;
  m_lastSendTime = sendTime;
}

void
TrafficControlLayer::calcTotalDqRate(Ptr<ns3::WifiMacQueue> queue) {
  double sendTime = Simulator::Now().GetSeconds();
  uint64_t sendBytes = queue->m_sendBytes;
  if (sendBytes == m_lastSendBytes || sendTime - m_lastSendTime < 0.01) return ;
  m_totalDqRate = (sendBytes - m_lastSendBytes) * 8 / (sendTime - m_lastSendTime) / 1000000;
  m_lastSendBytes = sendBytes;
  m_lastSendTime = sendTime;
}

uint16_t 
TrafficControlLayer::CalcRwndAPCC (Ptr<FlowState> f, TcpHeader& tcpHeader){
  if (f->m_isSlowStart) {
    f->m_windowSize+=1;
  }
  else {
    if (tcpHeader.GetAckNumber () > f->m_reduceSeq) {
        f->m_reduceSeq = tcpHeader.GetAckNumber () + (f->m_windowSize - 1) * m_MSS;
        f->m_windowSize = f->m_windowSize * (m_targetRate / m_totalDqRate);
    }
  }

  f->m_windowSize = std::max (f->m_windowSize, 2U);
  uint16_t rwndsize;
  if (f->m_windowSize * m_MSS % 2048 == 0) {
    rwndsize = f->m_windowSize * m_MSS / 2048;
  } 
  else {
    rwndsize = f->m_windowSize * m_MSS / 2048 + 1;
  }
  return rwndsize; 
}

void
TrafficControlLayer::Send (Ptr<NetDevice> device, Ptr<QueueDiscItem> item)
{
  NS_LOG_FUNCTION (this << device << item);

  NS_LOG_DEBUG ("Send packet to device " << device << " protocol number " <<
                item->GetProtocol ());

  Ptr<NetDeviceQueueInterface> devQueueIface;
  std::map<Ptr<NetDevice>, NetDeviceInfo>::iterator ndi = m_netDevices.find (device);

  if (ndi != m_netDevices.end ())
    {
      devQueueIface = ndi->second.m_ndqi;
    }

  // determine the transmission queue of the device where the packet will be enqueued
  std::size_t txq = 0;
  if (devQueueIface && devQueueIface->GetNTxQueues () > 1)
    {
      txq = devQueueIface->GetSelectQueueCallback () (item);
      // otherwise, Linux determines the queue index by using a hash function
      // and associates such index to the socket which the packet belongs to,
      // so that subsequent packets of the same socket will be mapped to the
      // same tx queue (__netdev_pick_tx function in net/core/dev.c). It is
      // pointless to implement this in ns-3 because currently the multi-queue
      // devices provide a select queue callback
    }

  NS_ASSERT (!devQueueIface || txq < devQueueIface->GetNTxQueues ());

  if (ndi == m_netDevices.end () || ndi->second.m_rootQueueDisc == 0)
    {
      // The device has no attached queue disc, thus add the header to the packet and
      // send it directly to the device if the selected queue is not stopped
      if (!devQueueIface || !devQueueIface->GetTxQueue (txq)->IsStopped ())
        {
          item->AddHeader ();
          // a single queue device makes no use of the priority tag
          if (!devQueueIface || devQueueIface->GetNTxQueues () == 1)
            {
              SocketPriorityTag priorityTag;
              item->GetPacket ()->RemovePacketTag (priorityTag);
            }
          device->Send (item->GetPacket (), item->GetAddress (), item->GetProtocol ());
        }
    }
  else
    {
      // Enqueue the packet in the queue disc associated with the netdevice queue
      // selected for the packet and try to dequeue packets from such queue disc
      item->SetTxQueueIndex (txq);

      Ptr<QueueDisc> qDisc = ndi->second.m_queueDiscsToWake[txq];
      NS_ASSERT (qDisc);
      if (m_ccmode == 2) {
        ModifyRwnd(item, qDisc, device);
      }
      else if (m_ccmode == 3) {
        UseAPCC(item, qDisc, device);
      }
      qDisc->Enqueue (item);
      qDisc->Run ();
    }
}

} // namespace ns3
