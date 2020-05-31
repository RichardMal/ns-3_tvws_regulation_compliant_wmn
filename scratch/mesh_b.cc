/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
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
 * Author: Kirill Andreev <andreev@iitp.ru>
 *
 *
 * By default this script creates m_xSize * m_ySize square grid topology with
 * IEEE802.11s stack installed at each node with peering management
 * and HWMP protocol.
 * The side of the square cell is defined by m_step parameter.
 * When topology is created, UDP ping is installed to opposite corners
 * by diagonals. packet size of the UDP ping and interval between two
 * successive packets is configurable.
 * 
 *  m_xSize * step
 *  |<--------->|
 *   step
 *  |<--->|
 *  * --- * --- * <---Ping sink  _
 *  | \   |   / |                ^
 *  |   \ | /   |                |
 *  * --- * --- * m_ySize * step |
 *  |   / | \   |                |
 *  | /   |   \ |                |
 *  * --- * --- *                _
 *  ^ Ping source
 *
 *  See also MeshTest::Configure to read more about configurable
 *  parameters.
 */
/*****************************************************************************
 * Richard TO DO
 *
 * 1. visualize [done]
 * 2. measure throughput? [done]
 * 3. trace route or draw out routing table of each node
 * this might be useful: https://groups.google.com/forum/#!searchin/ns-3-users/hwmp$20mesh$20routing$20table|sort:relevance/ns-3-users/Fj6rb4UJhAk/o
 * & this:
 *
 * https://groups.google.com/forum/#!msg/ns-3-users/DEO13IjUQQM/bG1a9YFTl3oJ
 *
 * &/or this:
 *
 * https://groups.google.com/forum/#!searchin/ns-3-users/print$20intermediate$20nodes$20hwmp%7Csort:relevance/ns-3-users/P7RTeHAkE2E/lP4zYLa5f48J
 *
 * 4. To add metric (proper augmented), this link might be useful:
 *  It is a discussion thread in ns3-groups under the subject, "Using packet tag data in a routing metric"
 *  https://groups.google.com/forum/#!searchin/ns-3-users/hwmp$20mesh$20routing$20table%7Csort:relevance/ns-3-users/nf_6Ao1J4go/J9iB5B-JCHAJ
 * 5. This link might be usefule when I get to factoring in interference and channel width?
 * https://groups.google.com/forum/#!searchin/ns-3-users/different$20wifi$20mode%7Csort:relevance/ns-3-users/T6dWROsG3-I/tmfrlCEVAwAJ
 */
/*
 * Richard random notes:
 * identifiers in manet-routing-compare.cc: adhocInterface, adhocNodes
 * are in mesh_b.cc equivalent to: interfaces, nodes respectively
 *
 * Things affecting throughput:
 * 1. packetinterval
 * 2.packet size
 * 3. phy standard
 * 4. phy data mode
 * 5. distance i.e m_step
 * 6. node count
 * 7. txpower
 * 8. TXGain
 * 9. RXGain
 */


#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/m_mesh-module.h"
#include "ns3/mobility-module.h"
#include "ns3/m_mesh-helper.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include "ns3/m_hwmp-protocol.h" //Richard



using namespace ns3;

/*
 * Richard: begin block to learn how to trace source and make call backs
 */


void PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{

      std::cout << "PHYRXERROR snr=" << snr << std::endl;

}



void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{

     // std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
      std::cout << "PHYTX mode=" << mode<< std::endl;

}

/*
 * Richard: start aim to get node's noise value
 * Good place to look: https://groups.google.com/forum/#!searchin/ns-3-users/SignalNoiseDbm$20signalNoise%7Csort:relevance/ns-3-users/7EF7tHzA5XE/qd9DD0E-BQAJ
 */

  static void PhyTrace  (Ptr< const Packet > packet, uint16_t channelFreqMhz,
		 uint16_t channelNumber, uint32_t rate, WifiPreamble preamble,
		 WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
  {

        WifiMacHeader hdr;
        Ptr<Packet> m_currentPacket;
        m_currentPacket = packet->Copy();
        m_currentPacket->RemoveHeader (hdr);
        Mac48Address source = hdr.GetAddr2();
	Mac48Address dest = hdr.GetAddr1();



	std::cout << "My PhyTrace List" << std::endl;
    std::cout<<"Packet received from " << source << " to "<< dest <<  " on Channel " << channelNumber << "signal strength" <<signalNoise.signal<<" with noise "<<signalNoise.noise<< std::endl;

	   //writing output to files

	     std::ofstream myfile;
	     myfile.open ("./mesh-point_reports/rss.txt",std::ios::app);
	     std::ostringstream oss;
	     //oss  << "distance between Source Node(" << i << ") and Node(" << j <<  ") est " << dist ; NS_LOG_UNCOND (oss.str());
	     myfile <<"Packet received from " << source << " to "<< dest <<  " on Channel " << channelNumber << " with Signal"<< signalNoise.signal<<" and noise " << signalNoise.noise<< std::endl;
	     NS_LOG_UNCOND (oss.str());
	     myfile.close();



}

//Richard: end my addition

/*
 * Richard: end blockt to learn callback mechanism
 */

NS_LOG_COMPONENT_DEFINE ("TestMeshScript");

class MeshTest
{
public:
  /// Init test
  MeshTest ();
  /// Configure test from command line arguments
  void Configure (int argc, char ** argv);
  /// Run test
  int Run ();

private:
  int       m_xSize;
  int       m_ySize;
  double    m_step;
  double    m_randomStart;
  double    m_totalTime;
  double    m_packetInterval;
  uint16_t  m_packetSize;
  uint32_t  m_nIfaces;
  bool      m_chan;
  bool      m_pcap;
  std::string m_stack;
  std::string m_root;
  std::string m_root2; //Richard
  std::string m_root3; //Richard
  std::string m_root4; //Richard
  std::string m_root5; //Richard
  std::string m_root6; //Richard
  std::string m_root7; //Richard
  /// List of network nodes
  NodeContainer nodes, nodes2;
  /// List of all mesh point devices
  NetDeviceContainer meshDevices, meshDevices2;
  //Addresses of interfaces:
  Ipv4InterfaceContainer interfaces, interfaces2;
  // MeshHelper. Report is not static methods
  MeshHelper mesh, mesh2;
  double m_txp; //Richard
  double m_rxGain; //Richard
  double m_txGain; //Richard
private:
  /// Create nodes and setup their mobility
  void CreateNodes ();
  /// Install internet m_stack on nodes
  void InstallInternetStack ();
  /// Install applications
  void InstallApplication ();
  /// Print mesh devices diagnostics
  void Report ();
  Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node); //Richard
  void ReceivePacket (Ptr<Socket> socket); //Richard
  uint32_t port = 9; //Richard
  uint32_t bytesTotal; //Richard
  uint32_t packetsReceived; //Richard
  uint64_t  m_maxPktsPerFile = 900000; //Richard, trying to explicitly set max  packet per trace file limit. Default is 100000

 }; //end MeshTest declaration



MeshTest::MeshTest () :
/*
 * Richard. it appears there is an issue with m_xSiZe and m_ySize: when set to equal even numbers like
 * 6x6 or 4x4, the simulation exits with an error
 */
  m_xSize (8),
  m_ySize (1), //total number of nodes = m_xSize * m_ySize
  //Richard: reducing m_step to less than 50, results in crazy interference and messes up multi-hoping as can be seen from visualizer
  m_step (100.0), //Richard: originally set to "100". I presume m_step is the distance between nodes, which means total grid horizontal distance = m_xSize *m_step
  m_randomStart (0.1),
  m_totalTime (4.0), //Richard: simulation time in seconds, I think?
  m_packetInterval (0.0001), //Richard: originally (0.1), now set to 0.0001 does seem to affect throughput; this means packet is sent every <interval> second over simulation period
  m_packetSize (1024), //Richard: found it at 1024
  m_nIfaces (2), //Richard: this might be a good start to setup multiradio nodes
  m_chan (true),
  m_pcap (false), //Richard: original setting
  //m_pcap (true), //Richard: try out true; simulation seem to take longer when "true"
  m_stack ("ns3::Dot11sStack"),
  m_root ("ff:ff:ff:ff:ff:ff"), //Richard: broadcast address?? implies no proactivity, only reactive mode is working
    //m_root ("00:00:00:00:00:19") //Richard: setting non broadcast mesh root activates proactive routing??
  /* Richard: for mac address keep in mind the decimal-hex conversion */
  /*
   * Richard: I realised that the lines below aren't necessary, I specify the mesh roots in dot11s-installer.cc
   */
  /*
  m_root2 ("00:00:00:00:00:02"),
  m_root3 ("00:00:00:00:00:03"),
  m_root4 ("00:00:00:00:00:04"),
  m_root5 ("00:00:00:00:00:05"),
  m_root6 ("00:00:00:00:00:06"),
  m_root7 ("00:00:00:00:00:07"),
  */

  /*
   * Richard: I don't know why but, when the txpower is not set explicitly, simulation run, not sure what the default value is.
   * However, when set explicitly, there's an increase in throughput proportionate with the txpower value
   */
  m_txp(20), //Richard
  m_rxGain(0), //Richard: increasing the rx/tx Gain obliterates the multi-hoping
  m_txGain(0) //Richard
{
}
void
MeshTest::Configure (int argc, char *argv[])
{
   CommandLine cmd;
  cmd.AddValue ("x-size", "Number of nodes in a row grid. [6]", m_xSize);
  cmd.AddValue ("y-size", "Number of rows in a grid. [6]", m_ySize);
  cmd.AddValue ("step",   "Size of edge in our grid, meters. [100 m]", m_step);
  /*
   * As soon as starting node means that it sends a beacon,
   * simultaneous start is not good.
   */
  cmd.AddValue ("start",  "Maximum random start delay, seconds. [0.1 s]", m_randomStart);
  cmd.AddValue ("time",  "Simulation time, seconds [100 s]", m_totalTime);
  cmd.AddValue ("packet-interval",  "Interval between packets in UDP ping, seconds [0.001 s]", m_packetInterval);
  cmd.AddValue ("packet-size",  "Size of packets in UDP ping", m_packetSize);
  cmd.AddValue ("interfaces", "Number of radio interfaces used by each mesh point. [1]", m_nIfaces);
  cmd.AddValue ("channels",   "Use different frequency channels for different interfaces. [0]", m_chan);
  cmd.AddValue ("pcap",   "Enable PCAP traces on interfaces. [0]", m_pcap);
  cmd.AddValue ("stack",  "Type of protocol stack. ns3::Dot11sStack by default", m_stack);
  cmd.AddValue ("root", "Mac address of root mesh point in HWMP", m_root);

  cmd.Parse (argc, argv);
  NS_LOG_DEBUG ("Grid:" << m_xSize << "*" << m_ySize);
  NS_LOG_DEBUG ("Simulation time: " << m_totalTime << " s");
}
void
MeshTest::CreateNodes ()
{ 
  /*
   * Create m_ySize*m_xSize stations to form a grid topology
   */
  nodes.Create (m_ySize*m_xSize); //Richard: use this when setting up even grid
  nodes2.Create(3); //creates n nodes
  // Configure YansWifiChannel
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();


  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  /*
    * Richard: set propagation delay. I expect this to reduce packet loss when simulatio runs longer
    */
  //wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel"); //Richard: this must be the default?

/*
 * Richard: for larger uniformrandom variable Max values, throughput becomes zero
 * when reduced, throughput increases but at some point delay begins to rise
 * Conclusion; setting the propagationdelay to randomPropagationModel does not fix the high packet loss issue for longer simulation:(
 */
  //wifiChannel.SetPropagationDelay("ns3::RandomPropagationDelayModel", "Variable", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0.00000001]"));

  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp)); //Richard
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp)); //Richard
  wifiPhy.Set ("RxGain", DoubleValue(m_rxGain) );  //Richard
  wifiPhy.Set ("TxGain", DoubleValue(m_txGain) ); //Richard

  /*
   * Richard: set propagation delay
   */

  /*
   * Richard:
   * RxNoiseFigure <= 7 don't appear to affect throughput any
   * RxNoiseFigure between 7-8 shows a drop in performance
   * above 8, performance drops to zero.
   * The line below when uncommented, sets the RxNoise for all nodes
   */
  //wifiPhy.Set ("RxNoiseFigure", DoubleValue (7) ); //Richard





   /*
   * Create mesh helper and set stack installer to it
   * Stack installer creates all needed protocols and install them to
   * mesh point device
   */
  mesh = MeshHelper::Default ();
  if (!Mac48Address (m_root.c_str ()).IsBroadcast ())
    {
      mesh.SetStackInstaller (m_stack, "Root", Mac48AddressValue (Mac48Address (m_root.c_str ())));
      //Richard: peep out dot11s-installer.cc, around line 100 for designated roots
    }
  else
    {
      //If root is not set, we do not use "Root" attribute, because it
      //is specified only for 11s
      mesh.SetStackInstaller (m_stack);
    }
  if (m_chan)
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::SPREAD_CHANNELS);
    }
  else
    {
      mesh.SetSpreadInterfaceChannels (MeshHelper::ZERO_CHANNEL);
    }
  mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart)));
  // Set number of interfaces - default is single-interface mesh point
  mesh.SetNumberOfInterfaces (m_nIfaces);

  /*
   * Richard: decided to explicitly set the standard in the next line to increase throughput.
   * Standards are defined in "wifi-phy.h. source: https://www.nsnam.org/doxygen/group__wifi.html#ga1299834f4e1c615af3ca738033b76a49
   * and the datamode i.e.
   * (i) mesh.setStandard; and
   * (ii) mesh.setremotestationmanager ....datamode
   * when these two lines are commented out, simulation still runs, which is with some default values unknown to me
   * Datamode options: "OfdmRate6Mbps", OfdmRate54Mbps"
   * complete list of datamode options can be found at file=../src/wifi/model/wifi-mode.cc, line=626
   * However, I haven't been lucky with most of the datamode options. To worth with some of them, other changes are needed elsewhere e.g. distance
   */

  /*********************************************************
   * Richard: This way of setting phymod works.
   * For my kind of distance between nodes, these are the only modes that seem to work:
   * OfdmRate6Mbps, OfdmRate9Mbps, OfdmRate12Mbps
   *
   * */
   //mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate6Mbps")); //Richard
     //mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate9Mbps")); //Richard
   //  mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate12Mbps")); //Richard

   /****************************************************************************************
    * Richard:    these lines set the specified attributes for all nodes.
    * But sadly I have not seen performance difference when any is changed.
    * However, I intend on trying to set other attributes using this approach.
    *
    * NB. there's no change in performance because of the order of logic,
    * when place after the statement meshDevices2 = mesh2.Install (wifiPhy, nodes2);
    * the lines do seem to take effect then (Aaha moment, 17 July 2017 at 23h30)
    *
    *
    *  The ALL is in the wilcard (*)
    * to set channelwidth for specific node, replace * with node ID e.g 1, 2, etc.
    *  Remember that counting starts from zero (0).
    *  The lines works when placed here but fails in other places tried so far.
    *  I think it fails because value is set already, Config::Set may work only when the value has not been set.
    *  to use it, maybe value attribute has to be unset then set again.
    *****************************************************************************************/
   //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (5));
  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/TxGain", DoubleValue (20));
   //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/ConstantRateWifiManager/DataMode", StringValue  ("OfdmRate6Mbps") );
   //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiPhy/WifiPhyStandard", WIFI_PHY_STANDARD_80211a);
  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::ConstantRateWifiManager/DataMode", StringValue  ("OfdmRate6Mbps") );



  /* I have not seen performance different by changing control mode value*/
  //mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate6Mbps"), "ControlMode", StringValue ("OfdmRate9Mbps") ); //Richard

  /*now try this:?*/
  //mesh.SetRemoteStationManager ("ns3::AarfWifiManager"); // Use AARF rate control

   /*
    * All these standards work, performance depends on distance (m_step)
    * as a matter of fact *80211g requires distance of + 100 in order to work.
    */
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211a); //Richard: this is default i.e. when not specified meshHelper falls on this stanadrd
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211b); //RICHARD
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211g);
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211_10MHZ); //RICHARD
 // mesh.SetStandard(WIFI_PHY_STANDARD_80211_5MHZ); //RICHARD
  //mesh.SetStandard (WIFI_PHY_STANDARD_UNSPECIFIED);
  //mesh.SetStandard(WIFI_PHY_STANDARD_holland); //RICHARD


/****************************************************************************
 * Richard: my new plan to implement different PHY bit rates is as follows:
 * 1) create MeshHelper mesh2, which is equivalent to mesh;
 * 2) set mesh and mesh2 to different PHY modes
 * 3) figure out how to put 1 & 2 together
 * 5) to achieve 3), I replicated all the four key objects so I have mesh2, nodes2, meshDevices2, interface2
 *   and just so I don't have to re-do a whole bunch of stuff, just before the key point of use, I set the
 *   duplicate to the value of it's original as in mesh2=mesh; meshDevices2=meshDevices2.
 *   This doesn't appear necessary for the other two objects, which are "nodes2" and "interfaces2"
 * NB. I checked now and it looks like, there is variation is performance. That tells me that that the settings are taking effect:)
 * So, looks like I'm in luck.
 * ***************************************************************************
 */


   mesh2 = mesh;

  mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate12Mbps"));
  mesh2.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate6Mbps"));



  // Install protocols and return container if MeshPointDevices
  meshDevices = mesh.Install (wifiPhy, nodes);




  meshDevices2 = meshDevices;
  meshDevices2 = mesh2.Install (wifiPhy, nodes2);

  /*
   * Richard: the line below sets the RxNoiseFigure for specific nodes:
   *
   *  my presumption is that it doesn't matter that I use two node containers
   * in terms of indexing, my presumption is that config sees them as starting from
   * zero to size of nodes+nodes?). But I'm yet to verify this.
   */

  /*
   * Richard: set RxNoiseFigure on the first node on all it's interfaces.
   *
   * comment out the line below when evaluating etx vs a_atx and hop-count vs a_hop-count.
   * when noise is set, etx and a_etc perform the same because protocol chooses the same path
   */




   Config::Set ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8)); //Richard: DEFAULT = 7;

  /*
   * Richard; test logic to get nodes noise
   */

 //Config::ConnectWithoutContext("/NodeList/[0]/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSnifferRx));
  std::cout<<"In MeshTest createNodes()"<<std::endl;
  // Setup mobility - static grid topology
  MobilityHelper mobility;

  //Richard: block below positions nodes in a X * Y grid fashion
  /*
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (m_step),
                                 "DeltaY", DoubleValue (m_step),
                                 "GridWidth", UintegerValue (m_xSize),
                                 "LayoutType", StringValue ("RowFirst"));

  */

  //Richard: manually setting nodes at custom positions
  /*
   * Wanting to make this kind of topology:
   *
   *       1+--2+-- 3+
   *       |        |
   * 0+---8+   9+   10+---7+
   *       |        |
   *       4+--5+--6+
   *
   * node ID 8, 9 & 10 belong to nodes2
   * nodes2 is positioned last so, naturally will take last indices after nodes in first container are place
   * hence the weired numbering:)
   */

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
   /*
    * Richard: syntax: vector (x, y, z)
    * the nodes are positioned in the same order the coordinates are added
    * and as such, in the current setting, when in netanim, node 3, 4, 6, 7
    * will appear as though they are in the same position because netanim is unable to show the 3D aspect
    * of node position. The "z" value, which is not visible in netanim is what differentiates the positions.
    *
    * On a second thought: I think that the Z axis would only make sense if the experiments coves a 3D aspect e.g.
    * deployment of nodes in a build where you have nodes spread around, above and below. Therefore, to make it easier
    * to visualize and keep control on the connectivity/range to facilitate multi-hoping, I'm going to deploy nodes
    * along the y and x axes only
    */

  /*
   * Richard: uncomment this block to position nodes in a 3-node bus topology to test things.
   * Also, check dot11s-installer.cc for mesh root addresses
   */
/*
   positionAlloc->Add (Vector (  0, 0, 0)); //position of first node in "nodes"
   positionAlloc->Add (Vector (  2*m_step, 0, 0));
   positionAlloc->Add (Vector (  m_step, 0, 0)); //the only node in "nodes2" container

*/
  /*
   * Richard: uncomment this block to position nodes for Airtime vs a_Airtime metric performance evaluation.
   * Also, check dot11s-installer.cc, about line 100-111 for mesh root addresses
   */


   positionAlloc->Add (Vector (  0, 350, 0)); //position of first node in "nodes"

   //top row
   positionAlloc->Add (Vector (  m_step, 350-m_step, 0));
   positionAlloc->Add (Vector (  m_step*2, 350-m_step, 0));
   positionAlloc->Add (Vector (  m_step*3, 350-m_step, 0));


   //bottom row
   positionAlloc->Add (Vector (  m_step, 350+m_step, 0));
   positionAlloc->Add (Vector (  m_step*2, 350+m_step, 0));
   positionAlloc->Add (Vector (  m_step*3, 350+m_step, 0));


   positionAlloc->Add (Vector (  m_step*4, 350, 0)); //position of 8th node in "nodes"

   //set positions for nodes in nodes2
   //place nodes in center row of my "kite" topology;)
   positionAlloc->Add (Vector (  m_step, 350, 0)); //position of 1st node in "nodes?
   positionAlloc->Add (Vector (  m_step*2, 350, 0));
   positionAlloc->Add (Vector (  m_step*3, 350, 0)); //position of 3rd node in "nodes2"


   /********************************************************************************************
    * Richard: uncomment this block to position nodes for etx and augmented etx performance evaluation
    * Also, check dot11s-installer.cc for mesh root addresses
    */

/*
    positionAlloc->Add (Vector (  0, 350, 0)); //node 0


     positionAlloc->Add (Vector (  m_step, 350-m_step, 0)); //node 1
     positionAlloc->Add (Vector (  m_step*2, 350-m_step, 0)); //node 2
     positionAlloc->Add (Vector (  m_step*3, 350-m_step, 0)); //node 3

     positionAlloc->Add (Vector (  m_step, 350, 0)); //node 4
     positionAlloc->Add (Vector (  m_step*3, 350, 0)); //node 5

     positionAlloc->Add (Vector (  m_step*4, 350, 0)); //node 6



     //nodes in "nodes2" container take the following 4 positions

     positionAlloc->Add (Vector (  m_step, 350+m_step, 0)); //node 7
     positionAlloc->Add (Vector (  m_step*2, 350+m_step, 0));//node 8
     positionAlloc->Add (Vector (  m_step*2, 350, 0)); //node 9
     positionAlloc->Add (Vector (  m_step*3, 350+m_step, 0));//node 10

*/
     /*
      * Richard: uncomment this block to position nodes for hop-count vs a_hop-count evaluation.
      * Also, check dot11s-installer.cc for mesh root addresses
      */

/*
     positionAlloc->Add (Vector (  0, 350, 0)); //position of first node in "nodes"

     //top row
     positionAlloc->Add (Vector (  m_step, 350-m_step, 0));
     positionAlloc->Add (Vector (  m_step*2, 350-m_step, 0));
     positionAlloc->Add (Vector (  m_step*3, 350-m_step, 0));


     //bottom row
     positionAlloc->Add (Vector (  m_step, 350+m_step, 0));
     positionAlloc->Add (Vector (  m_step*2, 350+m_step, 0));
     positionAlloc->Add (Vector (  m_step*3, 350+m_step, 0));


     positionAlloc->Add (Vector (  m_step*4, 350, 0)); //position of 8th node in "nodes"

     //set positions for nodes in nodes2
     //place nodes in center row of my "kite" topology;)
     positionAlloc->Add (Vector (  m_step, 350, 0)); //position of 1st node in "nodes?
     positionAlloc->Add (Vector (  m_step*2, 350, 0));
     positionAlloc->Add (Vector (  m_step*3, 350, 0)); //position of 3rd node in "nodes2"
*/


  mobility.SetPositionAllocator (positionAlloc);


  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
  mobility.Install (nodes2);


  if (m_pcap)
    wifiPhy.EnablePcapAll (std::string ("mp-"));

   /* Richard
    * Try to trace using ascii
    * The notes say, place this just before Simulator::Run()
    * Doesn't not seem to work:(
    */

  /*
  AsciiTraceHelper ascii;
  wifiPhy.EnableAsciiAll(ascii.CreateFileStream("Asciitest.tr"));
	*/
}
void
MeshTest::InstallInternetStack ()
{
  InternetStackHelper internetStack;
  internetStack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  interfaces = address.Assign (meshDevices);

  internetStack.Install(nodes2); //Richard
  interfaces2 = address.Assign (meshDevices2); //Richard
}
void
MeshTest::InstallApplication ()
{

  UdpEchoServerHelper echoServer (9);
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (0));
  serverApps.Start (Seconds (0.0));
  serverApps.Stop (Seconds (m_totalTime));
  /*
   * Richard: FlowID1: the following line specifies the destination node???
   */
  UdpEchoClientHelper echoClient (interfaces.GetAddress (0), 9);
  //Richard: comment these out, work with traffic in run() only and to see if throughput goes up
  echoClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)(m_totalTime*(1/m_packetInterval))));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (m_packetInterval)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));


  /*
   * Richard; install echo client on the last node in nodes container.
   * nodes2 has three nodes that are placed in between so they are able to forward traffic to the echoclient
   */

  /*
   * Richard: FlowID2: the following line specifies the source node???
   */

  ApplicationContainer clientApps = echoClient.Install (nodes.Get (m_xSize*m_ySize-1)); //Richard: original echoclient

  //Richard: try installing echoClient on third node in the other container? .. it worked!
  //ApplicationContainer clientApps = echoClient.Install (nodes2.Get (2));


  std::cout<<"In MeshTest::InstallApplication()..."<<std::endl; //Richard
  /*
   * Richard: when the next line is uncommented, flowmonitor returns stats for a single flow only
   * whereas, when commented out, flowmonitor has two flows i.e A->B and B->A
   */
  //Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(0), nodes.Get(0)); //Richard: show each transmission??
  //Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(0), nodes.Get(7));

  //Richard: try to configure a specific node with something e.g ip address 10.1.1.100

  /****************************************************************************************
   * Richard:    this line sets the channelWidth for all nodes. The ALL is in the wilcard (*)
   * to set channelwidth for specific node, replace * with node ID e.g 1, 2, etc.
   *  Remember that counting starts from zero (0)
   *****************************************************************************************/
  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (5));

  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSnifferRx));
  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&MonitorSnifferRx));

  //Config::ConnectWithoutContext("/NodeList/[0]/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSnifferRx));
 // Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSnifferRx));


  clientApps.Start (Seconds (0.0));
 clientApps.Stop (Seconds (m_totalTime));


}

//Richard. added PrintReceivedPacket()
static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;
  ns3::DataRateValue dv;
  DataRate dataRate;// = socket->GetNode()->GetAttribute("DataRate", dv);

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
     // DataRate dataRate;

      oss << " received one packet from " << addr.GetIpv4 () << " Data rate: ?";//<< socket->GetTxAvailable();//<<dataRate.GetBitRate();
    }
  else
    {
      oss << " received one packet!";
    }
  //Richard: Try printing intermediate node address, not working:(
  //oss<<"The Node Address is = "<<socket->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();

  return oss.str ();
}

//Richard: added the setupPacketReceive method.
Ptr<Socket>
MeshTest::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&MeshTest::ReceivePacket, this));

  return sink;
}

//Richard. added the ReceivePacket method
void
MeshTest::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;

  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;

      /* Richard:
       * Note packed ID and save it for later analysis.
       * I particularly would like to distinguish data packets from HWMP control packets when tracing path
       */
      std::cout<<" In MeshTest::ReceivePacket(): "<<" Packet ID: "<< packet->GetUid() <<std::endl;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}


int
MeshTest::Run ()
{
  //Richard: unsure if this is the best place for this but, lets see..
// Richard; decided to move this logic to MeshTest::createNodes
	/*
  std::string datarate ("20480bps"); //this is the rate at which the application generates data, which != PHY rate
  std::string phyMode ("DsssRate11Mbps");//in case we try to vary this

  Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("1024"));
  Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (datarate));
   */

  //Set Non-unicastMode rate to unicast mode
  //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode)); //when uncommented, throughput is zero

  /*
   * Richard: Peep this out: Ns3 implements rate adaptation algorithms like:
   * ns3::IdealWifiManager
   * ns3::ArfMacStations
   * ns3::AArfMacStations
   * ns3::IdealMacStations
   * ns3::CrMacStations
   * ns3::OnoeMacStations
   * ns3::AmrrMacStation
   */

  // Richard: end
   //: port(9);
  std::cout<<"In MeshTest::Run(), starting simulation..."<<std::endl; //Richard

  ns3::PacketMetadata::Enable (); //Richard: trying to see packet movement in netanim
  //ns3::AnimationInterface & EnableIpv4RouteTracking ("hwmp-route.xml", Seconds (0), Seconds (120), Seconds (0.25));

  CreateNodes ();
  InstallInternetStack ();
  InstallApplication ();

  /*************
   * Richard: run some app to send packets back and forth for throughput measurements
   * or, consider moving this logic to InstallApplication()?
   */
 // OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
 // onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  //onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

/*
  int nsinks = 5;
  for (int i = 1; i < nsinks; i++) //Richard.
  {
	  Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(i), nodes.Get(i));
	  AddressValue remoteAddress (InetSocketAddress (interfaces.GetAddress(i), port ));
	  onoff1.SetAttribute ("Remote", remoteAddress);

	  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
	  ApplicationContainer temp = onoff1.Install (nodes.Get (i + nsinks)); //Richard. original line
	 // ApplicationContainer temp = onoff1.Install (nodes.Get (24)); //Richard.
	  temp.Start (Seconds (var->GetValue (100.0,101.0)));
	  temp.Stop (Seconds (m_totalTime));
  }
*/


  /* when below line commented out, I get, " terminated with signal SIGSEGV" type of error */
  Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this); //Richard

  
  //Simulator::Stop (Seconds (m_totalTime)); //Richard: original position of line
  
  
  

  /**** Richard added code
     *  Start adding animation code to visualise
     * */
  //possible approach
  /*
   for (int n =0; n < nWifis; n++)
  {

    //AnimationInterface::SetConstantPosition (adhocNodes.Get (n), 94, 100);
    AnimationInterface::SetConstantPosition (nodes.Get (n), 94, 100);
  }
  */

  //alternatively hard-wire positions
  /*
  AnimationInterface::SetConstantPosition (nodes.Get (0), 256, 255);
  AnimationInterface::SetConstantPosition (nodes.Get (1), 770, 255);
  AnimationInterface::SetConstantPosition (nodes.Get (2), 515, 112);
  AnimationInterface::SetConstantPosition (nodes.Get (3), 515, 398);
  */

  //the (.) before the file path moves one step out of directory of execution
 // AnimationInterface  anim ("./visualisation_files/vis_test_augmented_airtime_scaling_beta_0.1_with_rxnoise.xml"); //trace file
  AnimationInterface  anim ("./visualisation_files/vis_test_m_HWMP.xml"); //trace file
  /*animation-interface.h defines #define 	MAX_PKTS_PER_TRACE_FILE   100000*/
  anim.SetMaxPktsPerTraceFile(m_maxPktsPerFile);
  //anim.EnablePacketMetadata(true); //needs to be done earlier in the program i.e. before packet sending starts.

  /* Richard:
   * Using AnimationInterface::EnableIpv4RouteTracking (std::string fileName, Time startTime, Time stopTime, TimepollInterval=Seconds(5))
   * will create a separate XML file that tracks routing tables at a node (or a container of nodes) at the polling interval.
   * Once the routing-table XML is generated, select the "Stats" tab in netanim and click the drop-down menu to select "Routing".
   * Click the file-browser button to load the XML file with routing-table information.
   * https://www.nsnam.org/wiki/NetAnim_3.105
   *
   * I noticed that once xml file is load in netanim, you can move the "Sim Time" slider on top to get the
   *  routing table at different points withing the interval
   */
   /* this is only going to work when doing layer 2 routing*/
  //anim.EnableIpv4RouteTracking ("hwmp-route-using-default-metric.xml", Seconds (0), Seconds (120), Seconds (0.25));

    /***
     * END animation code
     */
  
  /*
   * Richard; print routing table?
   *
   */

  /*Initial strand of thought:
   * It works but, it works at layer 3 whereas hwmp operates at later 2:(
   * */
  Ptr<Node> node = nodes.Get(0);
  Ptr<NetDevice> dev = node->GetDevice(0);
  Ptr<MeshPointDevice> meshDev = dev->GetObject<MeshPointDevice> ();
  Ptr<MeshL2RoutingProtocol> l2routingProtocol = meshDev->GetRoutingProtocol();
  Ptr<dot11s::HwmpProtocol> hwmpProt = l2routingProtocol->GetObject<dot11s::HwmpProtocol> ();



  /* second strand of though: */ //again looks like this is for layer3 routing:(
/*
  Ipv4GlobalRoutingHelper g;
   Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dynamic-global-routing-table.log?", std::ios::out);
   //g.PrintRoutingTableAllAt (Seconds (12), routingStream);
   g.PrintRoutingTableAllEvery (Seconds (2), routingStream);
*/

   //tryout my method in MeshHelper

   MeshHelper g;
     Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dynamic-global-routing-table.log??", std::ios::out);
     g.PrintRoutingTableAllAt (Seconds (5), routingStream);
     //g.PrintRoutingTableAllEvery (Seconds (2), routingStream);



  //Try to get throughput measurements
  Ptr<FlowMonitor> flowmon;
  FlowMonitorHelper flowmonHelper;
  flowmon = flowmonHelper.InstallAll ();
  
  Simulator::Stop (Seconds (m_totalTime)); //Richard: brought this line here

  /*
   * Richard: what we know is that this is the right place to put the config::connect statement
   * as seen in examples/wireless/wifi-ap.cc
   */

  /*
   * Richard:
   * if its a "TX" trace, it means node_i will fire whenever packet is received.
   * if its a "RX" trace, it means node_i will fire whenever a packet is sent.
   */
  Config::Connect("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));
  Config::Connect ("/NodeList/0/DeviceList/*/Phy/State/RxError", MakeCallback (&PhyRxErrorTrace));
  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&PhyTrace));

  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy", MakeCallback (&PhyTrace));


  //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&PhyTrace));
  //Config::ConnectWithoutContext("/NodeList/[0]/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSnifferRx));


  Simulator::Run ();
  //the "." before the file path moves one step out of directory of execution
  //flowmon->SerializeToXmlFile (("./folder_for_measurements/test_augmented_airtime_scaling_beta_0.1_with_rxnoise.xml"), false, false); //my version ~Richard
  flowmon->SerializeToXmlFile (("./folder_for_measurements/setup-tests/test_m_HWMP_airtime-augmented.xml"), false, false); //my version ~Richard

  /*
   * Try displaying stats, this block needs debugging.
   */
  /*
  flowmon->CheckForLostPackets(Seconds(m_totalTime));
   Ptr<Ipv4FlowClassifier> classifier =  DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
   std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
   std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i;
   for (i=stats.begin();i != stats.end(); i++)
   {
     Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
         std::cout << "Flow " << i->first << " ( " << t.sourceAddress
  <<" --> " << t.destinationAddress << " ) " << std::endl;
         std::cout << "delaySum     : " << i->second.delaySum <<
  std::endl;
         std::cout << "lastDelay    : " << i->second.lastDelay <<
  std::endl;
         std::cout << "jitterSum    : " << i->second.jitterSum <<
  std::endl;
         std::cout << "Tx Packets   : " << i->second.txPackets <<
  std::endl;
         std::cout << "Rx Packets   : " << i->second.rxPackets <<
  std::endl;
         std::cout << "Lost Packets : " << i->second.lostPackets <<
  std::endl;
         std::cout << "Throughput   : " << i->second.rxPackets *
  m_packetSize * 8 /(1024 * 1024 * m_totalTime ) << "Mbps"<< std::endl;
   }
*/
  //End try to display stats

  /* Richard: notice the absense of "simulator::schedule(.....)",
   * I moved this to thought that what's in installApplications() is adequate
   *
   *  */


  Simulator::Destroy ();



  std::cout<<".. ended running MeshTest::Run()"<<std::endl;
  return 0;
}
void
MeshTest::Report ()
{
  unsigned n (0);
  for (NetDeviceContainer::Iterator i = meshDevices.Begin (); i != meshDevices.End (); ++i, ++n)
    {
      std::ostringstream os;
      os << "./mesh-point_reports/mp-report-" << n << ".xml";
      std::cerr << "Printing mesh point device #" << n << " diagnostics to " << os.str () << "\n";
      std::ofstream of;
      of.open (os.str ().c_str ());
      if (!of.is_open ())
        {
          std::cerr << "Error: Can't open file " << os.str () << "\n";
          return;
        }
      mesh.Report (*i, of);
      of.close ();
    }
}
int
main (int argc, char *argv[])
{
//***************************************************************************************************
/*
	std::ifstream inFile; //Richard
  //inFile.open("/home/richard/eclipse_workspace/ns3_programming/scratch/ntwenu.txt"); //full file path required
  inFile.open("/home/richard/workspace/ns3_programming/scratch/ntwenu.txt");
  int network_class;//Richard: give it some random initial value
  if(inFile.fail())
  {
	  std::cout<<"Cannot open network status file"<<std::endl;
	  exit(1);
  }


  inFile>>network_class; //Richard: see what network class is in the file

  std::cout<<"In main(), starting simulation..."<<std::endl;
  std::cout<<"In main(), Network class: "<<network_class<<std::endl;
*/
	//int network_class;
	std::string filename = "/home/richard/workspace/ns3_programming/scratch/ntwenu.txt";
	std::ofstream inFile;
	inFile.open("/home/richard/workspace/ns3_programming/scratch/ntwenu.txt2");
	//inFile>>network_class;
//**************************************************************************************************

  MeshTest t; 
  t.Configure (argc, argv);

  return t.Run ();
}
