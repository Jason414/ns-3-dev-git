/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Universita' degli Studi di Napoli Federico II
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
 * Author: Stefano Avallone <stefano.avallone@.unina.it>
 */

#include "ns3/abort.h"
#include "ns3/queue-limits.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/queue-item.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NetDeviceQueueInterface");

NS_OBJECT_ENSURE_REGISTERED (ReceiverStatusManager);

TypeId
ReceiverStatusManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ReceiverStatusManager")
    .SetParent<Object> ()
    .SetGroupName("Network")
  ;
  return tid;
}

ReceiverStatusManager::~ReceiverStatusManager ()
{
  NS_LOG_FUNCTION (this);
}

void
ReceiverStatusManager::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_wakeCallback = nullptr;
  m_device = 0;
  Object::DoDispose ();
}

Ptr<NetDeviceQueueInterface>
ReceiverStatusManager::GetNetDeviceQueueInterface (void) const
{
  NS_LOG_FUNCTION (this);
  return m_ndqi;
}

void
ReceiverStatusManager::SetNetDeviceQueueInterface (Ptr<NetDeviceQueueInterface> ndqi)
{
  NS_LOG_FUNCTION (this << ndqi);

  NS_ABORT_MSG_IF (m_ndqi, "Cannot set the NetDeviceQueueInterface object twice");
  NS_ABORT_MSG_IF (!ndqi, "The NetDeviceQueueInterface object cannot be null");

  m_ndqi = ndqi;
}

const ReceiverStatusManager::WakeCallback&
ReceiverStatusManager::GetWakeCallback (void) const
{
  NS_LOG_FUNCTION (this);
  return m_wakeCallback;
}

void
ReceiverStatusManager::SetWakeCallback (WakeCallback cb)
{
  m_wakeCallback = cb;
}


NS_OBJECT_ENSURE_REGISTERED (NetDeviceQueueStatusManager);

TypeId
NetDeviceQueueStatusManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NetDeviceQueueStatusManager")
    .SetParent<ReceiverStatusManager> ()
    .SetGroupName("Network")
    .AddConstructor<NetDeviceQueueStatusManager> ()
    .AddAttribute ("WakeThresholdPackets",
                   "The minimum number of available packets required to wake up a queue disc."
                   " Used if the queue size is measured in packets",
                   QueueSizeValue (QueueSize ("1p")),
                   MakeQueueSizeAccessor (&NetDeviceQueueStatusManager::m_wakeThresholdPackets),
                   MakeQueueSizeChecker ())
    .AddAttribute ("WakeThresholdBytes",
                   "The minimum number of available bytes required to wake up a queue disc."
                   " Used if the queue size is measured in bytes",
                   QueueSizeValue (QueueSize ("0B")),
                   MakeQueueSizeAccessor (&NetDeviceQueueStatusManager::m_wakeThresholdBytes),
                   MakeQueueSizeChecker ())
  ;
  return tid;
}

NetDeviceQueueStatusManager::NetDeviceQueueStatusManager ()
  : m_stoppedByDevice (false),
    m_stoppedByQueueLimits (false),
    m_mtu (0),
    NS_LOG_TEMPLATE_DEFINE ("NetDeviceQueueInterface")
{
  NS_LOG_FUNCTION (this);
}

NetDeviceQueueStatusManager::~NetDeviceQueueStatusManager ()
{
  NS_LOG_FUNCTION (this);
}

void
NetDeviceQueueStatusManager::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);

  Ptr<NetDeviceQueueInterface> ndqi = GetNetDeviceQueueInterface ();

  NS_ASSERT_MSG (ndqi, "No NetDeviceQueueInterface has been set");

  Ptr<NetDevice> dev = ndqi->GetObject<NetDevice> ();

  if (dev)
    {
      m_mtu = dev->GetMtu ();
      // set the wake threshold (in bytes) to the MTU of the device, if it is still null
      if (!m_wakeThresholdBytes.GetValue ())
        {
          m_wakeThresholdBytes = QueueSize (QueueSizeUnit::BYTES, m_mtu);
        }
    }
  ReceiverStatusManager::DoInitialize ();
}

void
NetDeviceQueueStatusManager::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  m_queueLimits = 0;
  ReceiverStatusManager::DoDispose ();
}

bool
NetDeviceQueueStatusManager::IsStopped (void)
{
  NS_LOG_FUNCTION (this);
  return m_stoppedByDevice || m_stoppedByQueueLimits;
}

void
NetDeviceQueueStatusManager::Start (void)
{
  NS_LOG_FUNCTION (this);
  m_stoppedByDevice = false;
}

void
NetDeviceQueueStatusManager::Stop (void)
{
  NS_LOG_FUNCTION (this);
  m_stoppedByDevice = true;
}

void
NetDeviceQueueStatusManager::Wake (void)
{
  NS_LOG_FUNCTION (this);

  bool wasStoppedByDevice = m_stoppedByDevice;
  m_stoppedByDevice = false;

  // Request the queue disc to dequeue a packet
  if (wasStoppedByDevice && GetWakeCallback ())
    {
      Simulator::ScheduleNow (&ReceiverStatusManager::WakeCallback::operator(),
                              &GetWakeCallback ());
    }
}

void
NetDeviceQueueStatusManager::NotifyQueuedBytes (uint32_t bytes)
{
  NS_LOG_FUNCTION (this << bytes);
  if (!m_queueLimits)
    {
      return;
    }
  m_queueLimits->Queued (bytes);
  if (m_queueLimits->Available () >= 0)
    {
      return;
    }
  m_stoppedByQueueLimits = true;
}

void
NetDeviceQueueStatusManager::NotifyTransmittedBytes (uint32_t bytes)
{
  NS_LOG_FUNCTION (this << bytes);
  if ((!m_queueLimits) || (!bytes))
    {
      return;
    }
  m_queueLimits->Completed (bytes);
  if (m_queueLimits->Available () < 0)
    {
      return;
    }
  bool wasStoppedByQueueLimits = m_stoppedByQueueLimits;
  m_stoppedByQueueLimits = false;
  // Request the queue disc to dequeue a packet
  if (wasStoppedByQueueLimits && GetWakeCallback ())
    {
      Simulator::ScheduleNow (&ReceiverStatusManager::WakeCallback::operator(),
                              &GetWakeCallback ());
    }
}

void
NetDeviceQueueStatusManager::ResetQueueLimits ()
{
  NS_LOG_FUNCTION (this);
  if (!m_queueLimits)
    {
      return;
    }
  m_queueLimits->Reset ();
}

void
NetDeviceQueueStatusManager::SetQueueLimits (Ptr<QueueLimits> ql)
{
  NS_LOG_FUNCTION (this << ql);
  m_queueLimits = ql;
}

Ptr<QueueLimits>
NetDeviceQueueStatusManager::GetQueueLimits ()
{
  NS_LOG_FUNCTION (this);
  return m_queueLimits;
}


NS_OBJECT_ENSURE_REGISTERED (NetDeviceQueueInterface);

TypeId
NetDeviceQueueInterface::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NetDeviceQueueInterface")
    .SetParent<Object> ()
    .SetGroupName("Network")
    .AddConstructor<NetDeviceQueueInterface> ()
    .AddAttribute ("ReceiverStatusManagerType",
                   "Type of ReceiverStatusManager objects.",
                   TypeIdValue (NetDeviceQueueStatusManager::GetTypeId ()),
                   MakeTypeIdAccessor (&NetDeviceQueueInterface::m_receiverStatusManagerTypeId),
                   MakeTypeIdChecker ())
    .AddAttribute ("NTxQueues", "The number of device transmission queues",
                   TypeId::ATTR_GET | TypeId::ATTR_CONSTRUCT,
                   UintegerValue (1),
                   MakeUintegerAccessor (&NetDeviceQueueInterface::SetNTxQueues,
                                         &NetDeviceQueueInterface::GetNTxQueues),
                   MakeUintegerChecker<uint16_t> (1, 65535))
  ;
  return tid;
}

NetDeviceQueueInterface::NetDeviceQueueInterface ()
{
  NS_LOG_FUNCTION (this);

  // the default select queue callback returns 0
  m_selectQueueCallback = [] (Ptr<QueueItem> item) { return 0; };
}

NetDeviceQueueInterface::~NetDeviceQueueInterface ()
{
  NS_LOG_FUNCTION (this);
}

Ptr<ReceiverStatusManager>
NetDeviceQueueInterface::GetTxQueue (std::size_t i) const
{
  NS_ASSERT (i < m_txQueuesVector.size ());
  return m_txQueuesVector[i];
}

std::size_t
NetDeviceQueueInterface::GetNTxQueues (void) const
{
  return m_txQueuesVector.size ();
}

void
NetDeviceQueueInterface::DoInitialize (void)
{
  NS_LOG_FUNCTION (this);

  for (auto& q : m_txQueuesVector)
    {
      q->Initialize ();
    }
  Object::DoInitialize ();
}

void
NetDeviceQueueInterface::DoDispose (void)
{
  NS_LOG_FUNCTION (this);

  m_txQueuesVector.clear ();
  Object::DoDispose ();
}

void
NetDeviceQueueInterface::NotifyNewAggregate (void)
{
  NS_LOG_FUNCTION (this);

  // Notify the NetDeviceQueue objects that an object was aggregated
  for (auto& tx : m_txQueuesVector)
    {
      tx->NotifyAggregatedObject (this);
    }
  Object::NotifyNewAggregate ();
}

void
NetDeviceQueueInterface::SetNTxQueues (std::size_t numTxQueues)
{
  NS_LOG_FUNCTION (this << numTxQueues);
  NS_ASSERT (numTxQueues > 0);

  NS_ABORT_MSG_IF (!m_txQueuesVector.empty (), "Cannot call SetNTxQueues after creating device queues");

  ObjectFactory factory;
  factory.SetTypeId (m_receiverStatusManagerTypeId);

  // create the netdevice queues
  for (std::size_t i = 0; i < numTxQueues; i++)
    {
      m_txQueuesVector.push_back (factory.Create<ReceiverStatusManager> ());
    }
}

void
NetDeviceQueueInterface::SetSelectQueueCallback (SelectQueueCallback cb)
{
  m_selectQueueCallback = cb;
}

NetDeviceQueueInterface::SelectQueueCallback
NetDeviceQueueInterface::GetSelectQueueCallback (void) const
{
  return m_selectQueueCallback;
}

} // namespace ns3
