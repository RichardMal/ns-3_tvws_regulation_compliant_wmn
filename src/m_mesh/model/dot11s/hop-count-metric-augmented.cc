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

#include "hop-count-metric-augmented.h"
#include "ns3/wifi-phy.h"
#include "ns3/core-module.h" //Richard

namespace ns3 {
namespace dot11s {
NS_OBJECT_ENSURE_REGISTERED (HopCountAugmentedLinkMetricCalculator);

/*
 * Richard: begin get node's "datarate"
 */

uint32_t priority = 255;
WifiMode case1 = WifiMode("OfdmRate6Mbps");
WifiMode case2 = WifiMode("OfdmRate12Mbps");

void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{

     // std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
     // std::cout << "PHYTX mode=" << mode<< std::endl;
	if (mode == case1 )
		priority = 1;
	if (mode == case2)
		priority = 2;

}

//end get node's "datarate"
  
TypeId
HopCountAugmentedLinkMetricCalculator::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::HopCountAugmentedLinkMetricCalculator")
    .SetParent<Object> ()
    .SetGroupName ("Mesh")
    .AddConstructor<HopCountAugmentedLinkMetricCalculator> ()
    .AddAttribute ( "TestLength",
                    "Rate should be estimated using test length.",
                    UintegerValue (1024),
                    MakeUintegerAccessor (
                      &HopCountAugmentedLinkMetricCalculator::SetTestLength),
                    MakeUintegerChecker<uint16_t> (1)
                    )
    .AddAttribute ( "Dot11MetricTid",
                    "TID used to calculate metric (data rate)",
                    UintegerValue (0),
                    MakeUintegerAccessor (
                      &HopCountAugmentedLinkMetricCalculator::SetHeaderTid),
                    MakeUintegerChecker<uint8_t> (0)
                    )
  ;
  return tid;
}
HopCountAugmentedLinkMetricCalculator::HopCountAugmentedLinkMetricCalculator ()
{
}
void
HopCountAugmentedLinkMetricCalculator::SetHeaderTid (uint8_t tid)
{
  m_testHeader.SetDsFrom ();
  m_testHeader.SetDsTo ();
  m_testHeader.SetTypeData ();
  m_testHeader.SetQosTid (tid);
}
void
HopCountAugmentedLinkMetricCalculator::SetTestLength (uint16_t testLength)
{
  m_testFrame = Create<Packet> (testLength + 6 /*Mesh header*/ + 36 /*802.11 header*/);
}
uint32_t
HopCountAugmentedLinkMetricCalculator::CalculateMetric (Mac48Address peerAddress, Ptr<MeshWifiInterfaceMac> mac)
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

  /*
   * Richard: fire each time packet is transmitted to get the PHY bit rate
   */
  Config::Connect("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));

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
 // uint32_t metric = (uint32_t)((double)( /*Overhead + payload*/
 //                                mac->GetPifs () + mac->GetSlot () + mac->GetEifsNoDifs () + //DIFS + SIFS + AckTxTime = PIFS + SLOT + EifsNoDifs
 //                                mac->GetWifiPhy ()->CalculateTxDuration (m_testFrame->GetSize (), txVector, mac->GetWifiPhy ()->GetFrequency())
  //                               ).GetMicroSeconds () / (10.24 * (1.0 - failAvg)));
  //return metric; //original return
  uint32_t metric = 1;//Richard: 1 implies links have equal quality, which is what hopcount does
  //lets take r to be the datata
  //uint32_t r = (uint32_t)((double)(mac->GetWifiPhy ()->CalculateTxDuration (m_testFrame->GetSize (),txVector, mac->GetWifiPhy ()->GetFrequency())).GetMicroSeconds ());

  uint32_t beta = 0.9; //Richard: scaling constant, tryout values: 0.1, 0.5, 0.9.
  //Changing this scaling value doesn't seem to affect metric valuation

  return metric+(priority*beta); //prioritise links with higher PHY bit rate
}
} // namespace dot11s
} // namespace ns3
