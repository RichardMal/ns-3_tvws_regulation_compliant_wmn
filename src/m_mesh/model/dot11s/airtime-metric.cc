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

#include "airtime-metric.h"
#include "ns3/wifi-phy.h"
#include "ns3/core-module.h" //Richard
#include <iostream>
#include <fstream> //Richard

namespace ns3 {
namespace dot11s {
NS_OBJECT_ENSURE_REGISTERED (AirtimeLinkMetricCalculator);

/**********************************************************
 * Richard: start of dynamically selectable metric crank
 *
 * ****************************************************
 */
uint32_t m_metric = 3; // 0= Airtime, 1=hopcount-augmented, 2=etx-augmented, 3=Airtime-augmented, 4=hopcount
std::string m_metricName;
//std::ifstream inFile; //Richard
uint32_t network_class;//Richard: give it some random initial value


/*******************************************************
 * Richard: start adding means to report noise.
 * This is used in Airtime-Augmented
 */

double rfNoise =1;

void PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{

      //std::cout << "PHYRXERROR snr=" << snr << std::endl;
	 if (snr<=0 )
		 rfNoise=1;
	 else
		 rfNoise = snr + 1; //add 1 to avoid division by zero

}
//Richard: end means to report noise

/**********************************************
 * Richard: begin get node's "datarate".
 * This is use in ETX augmented and Hop-count augmented
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
AirtimeLinkMetricCalculator::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::dot11s::AirtimeLinkMetricCalculator")
    .SetParent<Object> ()
    .SetGroupName ("Mesh")
    .AddConstructor<AirtimeLinkMetricCalculator> ()
    .AddAttribute ( "TestLength",
                    "Rate should be estimated using test length.",
                    UintegerValue (1024),
                    MakeUintegerAccessor (
                      &AirtimeLinkMetricCalculator::SetTestLength),
                    MakeUintegerChecker<uint16_t> (1)
                    )
    .AddAttribute ( "Dot11MetricTid",
                    "TID used to calculate metric (data rate)",
                    UintegerValue (0),
                    MakeUintegerAccessor (
                      &AirtimeLinkMetricCalculator::SetHeaderTid),
                    MakeUintegerChecker<uint8_t> (0)
                    )
  ;
  return tid;
}
AirtimeLinkMetricCalculator::AirtimeLinkMetricCalculator ()
{
}
void
AirtimeLinkMetricCalculator::SetHeaderTid (uint8_t tid)
{
  m_testHeader.SetDsFrom ();
  m_testHeader.SetDsTo ();
  m_testHeader.SetTypeData ();
  m_testHeader.SetQosTid (tid);
}
void
AirtimeLinkMetricCalculator::SetTestLength (uint16_t testLength)
{
  m_testFrame = Create<Packet> (testLength + 6 /*Mesh header*/ + 36 /*802.11 header*/);
}
uint32_t
AirtimeLinkMetricCalculator::CalculateMetric (Mac48Address peerAddress, Ptr<MeshWifiInterfaceMac> mac)
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

  //Richard: get the nose?
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeCallback (&PhyRxErrorTrace));
    //double rfnoise = 1;
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
  uint32_t metric_airtime = (uint32_t)((double)( /*Overhead + payload*/
                                 mac->GetPifs () + mac->GetSlot () + mac->GetEifsNoDifs () + //DIFS + SIFS + AckTxTime = PIFS + SLOT + EifsNoDifs
                                 mac->GetWifiPhy ()->CalculateTxDuration (m_testFrame->GetSize (), txVector, mac->GetWifiPhy ()->GetFrequency())
                                 ).GetMicroSeconds () / (10.24 * (1.0 - failAvg)));

  uint32_t metric = 0; //Richard: give it some arbitrary random default.

  //std::string filename = "/home/richard/eclipse_workspace/ns3_programming/scratch/ntwenu.txt";

  std::ifstream inFile; //Richard: inFile gets problematically when declared globally
  //inFile.open("/home/richard/workspace/ns3_programming/scratch/ntwenu.txt"); //home pc
 // inFile.open("/home/richard/eclipse_workspace/ns3_programming/scratch/ntwenu.txt"); //Richard: full file path required, lab PC
  inFile.open("../ns3_programming/scratch/ntwenu.txt"); //home or lab pc [Richard, 6 March 2019]
  //the 2 dots ".." imply getting out of current directory by 2 levels. Current working directory in this case is the scratch folder.

  //inFile.open("/scratch/ntwenu.txt"); //home pc
  //inFile.open(filename.c_str());

  //std::ofstream inFile2;
  //inFile2.open("/home/richard/workspace/ns3_programming/scratch/ntwenu.txt7");


  if(inFile.fail())
    {
  	  std::cout<<"Cannot open network status file"<<std::endl;
  	 exit(1);
    }

  inFile>>network_class; //Richard: see what network class is in the file

  //Richard: lets see if we read the file right..
  //std::cout<<"Network class: "<<network_class<<std::endl;


  switch (m_metric)
    {
    case 0:
      //list.Add (dsdv, 100);
      metric = metric_airtime;
      m_metricName = "Airtime"; //original HWMP Airtime metric
      break;
    case 1:{
      uint32_t scale = 0.9; //Richard: scale = 0.1 implies, alpha=0.9 and beta=0. Other scaling constant values to tryout: 0.1, 0.5, 0.9.
      metric = 1 + (priority*scale);
      m_metricName = "Hop-count-augmented"; }
      break;
    case 2:{
      //list.Add (aodv, 100);
      uint32_t scale = 0.1; //Richard: scale = 0.1 implies, alpha=0.9 and beta=0. Other scaling constant values to tryout: 0.1, 0.5, 0.9.
      metric = failAvg+(priority*scale);
      m_metricName = "ETX-augmented";}
      break;
    case 3:{
      uint32_t scale = 0.1; //Richard: scaling constant, tryout values: 0.1, 0.5, 0.9.
      metric = rfNoise/(metric_airtime*scale);
      m_metricName = "Airtime-augmented";}
      break;
    case 4:{

          metric = 1;
          m_metricName = "hopcount";}
          break;
    default:
      NS_FATAL_ERROR ("No such metric:" << m_metric);
    }

  return metric;

}
} // namespace dot11s
} // namespace ns3
