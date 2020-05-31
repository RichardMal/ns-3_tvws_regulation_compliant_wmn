/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
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
 * Authors: Kirill Andreev <andreev@iitp.ru>
 */

#include "etx-metric.h"
#include "ns3/wifi-phy.h"

namespace ns3 {
namespace dot11s {
NS_OBJECT_ENSURE_REGISTERED (EtxLinkMetricCalculator);
  
TypeId
EtxLinkMetricCalculator::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::EtxLinkMetricCalculator")
    .SetParent<Object> ()
    .SetGroupName ("Mesh")
    .AddConstructor<EtxLinkMetricCalculator> ()
    .AddAttribute ( "TestLength",
                    "Rate should be estimated using test length.",
                    UintegerValue (1024),
                    MakeUintegerAccessor (
                      &EtxLinkMetricCalculator::SetTestLength),
                    MakeUintegerChecker<uint16_t> (1)
                    )
    .AddAttribute ( "Dot11MetricTid",
                    "TID used to calculate metric (data rate)",
                    UintegerValue (0),
                    MakeUintegerAccessor (
                      &EtxLinkMetricCalculator::SetHeaderTid),
                    MakeUintegerChecker<uint8_t> (0)
                    )
  ;
  return tid;
}
EtxLinkMetricCalculator::EtxLinkMetricCalculator ()
{
}
void
EtxLinkMetricCalculator::SetHeaderTid (uint8_t tid)
{
  m_testHeader.SetDsFrom ();
  m_testHeader.SetDsTo ();
  m_testHeader.SetTypeData ();
  m_testHeader.SetQosTid (tid);
}
void
EtxLinkMetricCalculator::SetTestLength (uint16_t testLength)
{
  m_testFrame = Create<Packet> (testLength + 6 /*Mesh header*/ + 36 /*802.11 header*/);
}
uint32_t
EtxLinkMetricCalculator::CalculateMetric (Mac48Address peerAddress, Ptr<MeshWifiInterfaceMac> mac)
{
  /* Airtime link metric is defined in 11B.10 of 802.11s Draft D3.0 as:
   *
   * airtime = (O + Bt/r) /  (1 - frame error rate), where
   * o  -- the PHY dependent channel access which includes frame headers, training sequences,
   *       access protocol frames, etc.
   * bt -- the test packet length in bits (8192 by default),
   * r  -- the current bitrate of the packet,
   *
   * Final result is expressed in units of 0.01 Time Unit = 10.24 us (as required by 802.11s draft)
   */
  NS_ASSERT (!peerAddress.IsGroup ());
  //obtain current rate:
  WifiMode mode = mac->GetWifiRemoteStationManager ()->GetDataTxVector (peerAddress, &m_testHeader, m_testFrame).GetMode();
  //obtain frame error rate:
  double failAvg = mac->GetWifiRemoteStationManager ()->GetInfo (peerAddress).GetFrameErrorRate ();
  if (failAvg == 1)
    {
      // Retrun max metric value when frame error rate equals to 1
      return (uint32_t)0xffffffff;
    }
  NS_ASSERT (failAvg < 1.0);
  WifiTxVector txVector;
  txVector.SetMode (mode);
  txVector.SetPreambleType (WIFI_PREAMBLE_LONG);
  //calculate metric
  //Richard: uncomment this block(4 lines) to calculate original airtime metric
  //uint32_t metric = (uint32_t)((double)( /*Overhead + payload*/
  //                               mac->GetPifs () + mac->GetSlot () + mac->GetEifsNoDifs () + //DIFS + SIFS + AckTxTime = PIFS + SLOT + EifsNoDifs
  //                               mac->GetWifiPhy ()->CalculateTxDuration (m_testFrame->GetSize (), txVector, mac->GetWifiPhy ()->GetFrequency())
   //                              ).GetMicroSeconds () / (10.24 * (1.0 - failAvg)));

  //uint32_t metric = 2; //Richard: hardcode value to try code out


  //return metric; //Richard: original return

  /***************************************************
   * 1. For ETX, lets keep things simple by considering frame error rate to be eque to etx
   * 2. Instead of changing return turn of EtxLinkMetricCalculator::CalculateMetric () from uint32_t
   * let me try to cast  failAvg, which double to unint32_t. The "+0.5" in the cast is for sanity according to
   * http://www.cplusplus.com/forum/beginner/2717/
   ***************************************************/

  //Try:
  //return  static_cast <uint32_t>(failAvg + 0.5); //Richard. this I haven't tried
  //or
  return  (uint32_t)failAvg; //Richard. this compiles fine


}
} // namespace dot11s
} // namespace ns3
