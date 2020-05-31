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
 ****************************************************************************************************
 * Came back to it again 18 Feb 2019
 * I took mesh_c, renamed it "mesh_D" and (i) modified it into a chain topology; (ii)
 * NB. reverting back to mesh_c.cc topology can be realised quite easily by
 * uncommenting lines with the following keywords: meshDevice2, mesh2, node2, interface2.
 * **********
 * LIST of methods added:
 * 1) ChannelAssignment()
 */


#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
//#include "ns3/mesh-module.h"
#include "ns3/m_mesh-module.h"
#include "ns3/mobility-module.h"
//#include "ns3/mesh-helper.h"
#include "ns3/m_mesh-helper.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

//#include "ns3/hwmp-protocol.h" //Richard
#include "ns3/m_hwmp-protocol.h" //Richard
#include "ns3/udp-echo-helper.h" //Richard
#include <list> //Richard C++ library
#include <iterator> //Richard C++ library
#include <thread>   //Richard std::this_thread::sleep_for
#include <chrono>   //Richard std::chrono::seconds
#include <exception>// Richard: fort try .. catch blocks
#include <vector> //Richard: for <vector> whatnot :)
#include <mutex>

using namespace ns3;


 /***************
  * Global variables to simplify things, [Richard, 5 March]
  */

  std::string out_file_for_alll_nodes = "./Richard_output_logs/node#_ALL.csv";
  const int channelListenDuration = 1; //how long to linger on a channel listening for beacon
  bool detected = false;

  std::mutex mtx;
  /*
   * End global variables
   */

  /*****************************************
   * This is the kind of structure I'm trying to develop:
   *
   * -------------------------------
   * | nodeId | deviceId | channel |
   * -------------------------------
   * |   0    |    1     |   -     |
   * |   1    |    1     |   -     |
   * |   2    |    1     |   -     |
   * |   -    |    -     |   -     |
   * |   n    |    1     |   -     |
   * -------------------------------
   *
   * For now all fields are simple ints. In future it may be better to define 'channel' and 'deviceId' as lists
   * so they are able to hold more than one value.
   * The idea is:
   * i) initialise table with -1 values throughput
   * ii) the node will lookup it's ID in the table
   * 		-if it's it's -1, node will scan the channel space
   * 		-if beacon detected, the nodeId and channel on which beacon was detected gets entered in the table.
   *
   */

  struct ScanStatus
  {

  int nodeId;
  int deviceId; //in future make this a list in the case of multi-radio nodes
  int channel; //in future make this a list for cases when a beacon is detected on multiple channels
  std::list <int> chan; //this is the list of channels on which beacon has been detected. -a list because beacon may be detected on multiple channels.
  };

  std::vector<ScanStatus> table; //let me try with this as global variable

void show ()
{
	/*
	 * Display what's in the scanstatus table
	 */

	std::list <int>::iterator it;

	 std::cout<<"nodeId, deviceId, channel"<< std::endl; //header row
	  for(std::vector<ScanStatus>::iterator i = table.begin(); i != table.end(); ++i)
	  {
	      std::cout<<i->nodeId<<", "<<i->deviceId<<", "<<i->channel;

	      for( it = i->chan.begin(); it != i->chan.end(); ++it) //iterate over the chan list
	      {
	              std::cout<<", "<<*it;
	      }
	      std::cout<<std::endl;
	  }//end for

}
  void setChannel (std::string channelSetter, UintegerValue chan)
  {

	  std::cout<<"setChannel () triggered "<<std::endl;
	  Config::Set(channelSetter, UintegerValue(chan));

  }

  void func1()
  {
	  std::cout<<"func1 triggered "<<std::endl;
	  Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
  }




  void func2()
   {
	  std::cout<<"func2 triggered "<<std::endl;
 	  Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(36));
   }

  void func3()
   {
 	  Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(44));
   }

  void func4()
   {
 	  Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(48));
   }

/*
 * Richard: begin block to learn how to trace source and make call backs.
 * NOTE: the function name e.g. PhyRxErrorTrace is not a keyword, it can be anything
 * but I think the convention is to much what the signature is referring to [18 Feb 2019]
 */

void PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{
      std::cout <<"PHYRXERROR snr is==" << snr << std::endl;
      std::cout<<"*************************************************** \n\n\n"<<std::endl;
      std::cout<<"Simulator time: "<<(Simulator::Now ()).GetSeconds ()<<std::endl;
      std::cout<<"\n\n\n *************************************************** "<<std::endl;
}



void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{

     // std::cout << "PHYTX mode=" << mode << " " << *packet << std::endl;
      std::cout << "PHYTX mode is =" << mode<< std::endl;

}

/*
 * Richard: start aim to get node's noise value
 * Good place to look: https://groups.google.com/forum/#!searchin/ns-3-users/SignalNoiseDbm$20signalNoise%7Csort:relevance/ns-3-users/7EF7tHzA5XE/qd9DD0E-BQAJ
 * see: https://groups.google.com/forum/#!topic/ns-3-users/PtxUB-imoGQ
 * *******************
 * After much struggle with PhyTrace below and no luck, I placed an output line in wifi-phy.cc ~/src/wifi/model/wifi-phy.cc
 * Inspired by https://groups.google.com/forum/#!topic/ns-3-users/tXSVAIaIQBA
 *
 */

  static void PhyTrace  (Ptr< const Packet > packet, uint16_t channelFreqMhz,	uint16_t channelNumber, uint32_t rate, WifiPreamble preamble,		 WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
  //OR try
  //static void PhyTrace (Ptr< const Packet > packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, WifiPreamble preamble,WifiTxVector txVector, struct mpduInfo aMpdu, struct signalNoiseDbm signalNoise)
  {
/*//commented out to troubleshoot
        WifiMacHeader hdr;
        Ptr<Packet> m_currentPacket;
        m_currentPacket = packet->Copy();
        m_currentPacket->RemoveHeader (hdr);
        Mac48Address source = hdr.GetAddr2();
	Mac48Address dest = hdr.GetAddr1();
*/


	std::cout << "My PhyTrace List" << std::endl;
    //std::cout<<"Packet received from " << source << " to "<< dest <<  " on Channel " << channelNumber << "signal strength" <<signalNoise.signal<<" with noise "<<signalNoise.noise<< std::endl;
	std::cout<<  " on Channel " << channelNumber << "signal strength" <<signalNoise.signal<<" with noise "<<signalNoise.noise<< std::endl;
	   //writing output to files
/*//commented out to troubleshoot
	     std::ofstream myfile;
	     myfile.open ("./mesh-point_reports/rss.txt",std::ios::app);
	     std::ostringstream oss;
	     */

	     //oss  << "distance between Source Node(" << i << ") and Node(" << j <<  ") est " << dist ; NS_LOG_UNCOND (oss.str());

	/* //commented out to troubleshoot
	     myfile <<"Packet received from " << source << " to "<< dest <<  " on Channel " << channelNumber << " with Signal"<< signalNoise.signal<<" and noise " << signalNoise.noise<< std::endl;
	     NS_LOG_UNCOND (oss.str());
	     myfile.close();
	     */

}

  /*
   * Attempt to create callback function to get channel number from this space.
   * I finally came ok with MonitorSniffRx and MonitorSniffTx. These functions are defined in ~src/wifi/model/wifi-phy.cc
   */

  //void MonitorSniffRx (Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
  void MonitorSniffRx (std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
  {
  	 //std::cout << "From wifi-phy.cc line# 2193: signal_power = " << signalNoise.signal << "  noise_power = "<< signalNoise.noise << "   packet_size = " << packet->GetSize() << std::endl;
  	 std::cout<<"**************** " <<std::endl;
	  std::cout << "From NotifyMonitorSniffRx() in mesh_D.cc line# ~193: channel# = " << channelNumber << "  channel FreqMhz = "<< channelFreqMhz << std::endl;
	  std::cout<<"**************** " <<std::endl;
	  // m_phyMonitorSniffRxTrace (packet, channelFreqMhz, channelNumber, txVector, aMpdu, signalNoise);

	 // node1Associated = true;

	/***************************************************************
	 * Check this out: https://groups.google.com/forum/#!searchin/ns-3-users/how$20to$20get$20node$20that$20fired$20trace$20call$20back%7Csort:date/ns-3-users/aI8_qu37KXo/7Ce__87VBwAJ
	 *Adil Alsuhaim raises the issue of there being four addresses: i) Receiver Address. ii)  Destination Address. iii) Transmitter Address; iv)   Source Address.
	 *
	 *Also: https://groups.google.com/forum/#!searchin/ns-3-users/how$20to$20get$20node$20that$20fired$20trace$20call$20back%7Csort:date/ns-3-users/aI8_qu37KXo/7Ce__87VBwAJ
	 */
	  WifiMacHeader hdr;
	    	    Ptr<Packet> m_currentPacket;
	    	    m_currentPacket = packet->Copy();
	    	    m_currentPacket->RemoveHeader (hdr);
	    	    Mac48Address source = hdr.GetAddr2(); /*Look into hdr.GetAddr1(), hdr.GetAddr2(), hdr.GetAddr3(),  hdr.GetAddr4() */
	    		Mac48Address dest = hdr.GetAddr1();


	  std::ofstream out (out_file_for_alll_nodes.c_str (), std::ios::app); //hardwired
	    	    out << context <<", "<<
	    	 	channelNumber <<", "<<
	    	  Simulator::Now ().GetSeconds () <<", "<<
	    	    " 0, " <<
	    	    " 1, " <<
	    	    source <<", "<<
	    		dest<<", "<<
	    		packet->GetUid()<<
	    	    std::endl;
	    	    out.close ();
/*
 * Attempt to set flag
 */
	    	    std::string nodeId=context.substr( 10,1 );
	    	    std::string  deviceId=context.substr( 23,1 );
	    	    if (nodeId=="4")
	    	    {
	    	    	//node1Associated=true;
	    	    		//std::cout<<"NodeId in *Rx() match found: "<<nodeId<<", detected = "<<*detected<<", getBeaconDetectionStatus() ="<<getBeaconDetectionStatus()<<std::endl;
	    	    }
	    	    else
	    	    {
	    	    	std::cout<<"NodeId in *Rx() match NOT found, detected = "<< std::endl;
	    	    }




  }

  void  MonitorSniffTx (std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu)
    {
    	 //std::cout << "From wifi-phy.cc line# 2193: signal_power = " << signalNoise.signal << "  noise_power = "<< signalNoise.noise << "   packet_size = " << packet->GetSize() << std::endl;
    	 std::cout<<"**************** " <<std::endl;
  	 std::cout << "From NotifyMonitorSniffTx() in mesh_D.cc line# ~207:"<< "Time:"<<Simulator::Now ().GetSeconds ()<<" Uid: "<<packet->GetUid()  <<"channel# = " << channelNumber << "  channel FreqMhz = "<< channelFreqMhz << std::endl;
  	  std::cout<<"**************** " <<std::endl;

    // m_phyMonitorSniffRxTrace (packet, channelFreqMhz, channelNumber, txVector, aMpdu, signalNoise);

 /************************************
  * I've created the file and written the column headers in MeshTest::Run ()
  * So, at this stage, I just want to append to the file.
  * NOTE: "std::ofstream out (out_file_for_alll_nodes.c_str ());" overwrites the file,
  * whereas 'std::ofstream out (out_file_for_alll_nodes.c_str (), std::ios::app);" appends to the file
  *
  * *****************************************
  * */

  /*****************************************
   * Attempting to get source and destination address of transmitted packet
   */

  	 /*OPTION 1:*/
  	WifiMacHeader hdr;
  	    Ptr<Packet> m_currentPacket;
  	    m_currentPacket = packet->Copy();
  	    m_currentPacket->RemoveHeader (hdr);
  	    Mac48Address source = hdr.GetAddr2();
  		Mac48Address dest = hdr.GetAddr1();
  	 /*OPTION 2: works fine, but I think option 1 is more readable*/
 /*
  	 	        packet->Copy()->RemoveHeader (hdr);
  		  	    Mac48Address source = hdr.GetAddr2();
  		  		Mac48Address dest = hdr.GetAddr1();
*/
  		std::cout<<"Source?: "<<source<<std::endl;
  		std::cout<<"Destination?: "<<dest<<std::endl;


  	std::ofstream out (out_file_for_alll_nodes.c_str (), std::ios::app); //hardwired
  	    out << context <<", "<<
  	 	channelNumber <<", "<<
  	  Simulator::Now ().GetSeconds () <<", "<<
  	    " 1, " <<
  	    " 0, " <<
  	     source <<", "<<
  		dest<<", "<<
  		packet->GetUid()<<
  	    std::endl;
  	    out.close ();

  	    if (channelNumber == 0)
  	    {
  	    	 /**
  	    	   * \todo
  	    	   * Correct channel switching is:
  	    	   *
  	    	   * 1. Interface down, e.g. to stop packets from layer 3
  	    	   * 2. Wait before all output queues will be empty
  	    	   * 3. Switch PHY channel
  	    	   * 4. Interface up
  	    	   *
  	    	   * In the absence of elegant solution, I use a dirty trick:
  	    	   * schedule channel change to occur 1 second from now. [Richard 8 March 2019]
  	    	   */

  	    	/*
  	    	 * NOTE on how substr(pos, len) works:
  	    	 * pos specifies the position, len indicates the number of character to consider from pos onwards.
  	    	 * In my case, the expected context is something like "/NodeList/1/DeviceList/1/Phy/MonitorSnifferTx"
  	    	 * So, to get the node id, I tell substr() to go to position 10 and get 1 character;
  	    	 *      to get the device id, I say get 1 character starting at position 23.
  	    	 */
  	    	//std::string nodeId="4"; std::string  deviceId="*"; UintegerValue chanNum = 40; //earlier line used to test the logic
  	    	//std::cout<<" Context node ID "<<context.substr( 10,1 )<<" device ID: "<<context.substr(23,1)<<std::endl; //works fine -hard wired string positions

  	    	std::string nodeId=context.substr( 10,1 );
  	    	std::string  deviceId=context.substr( 23,1 );
  	    	//UintegerValue chanNum = 40;

  	    	/*
  	    	 * Richard. Traditionally, the function would be called as channelAssignment (var1, var2);
  	    	 * But notice how it's down when scheduled: &channelAssignment, var1, var2 i.e. no bracket
  	    	 */
  	    	//Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()+1), &ChannelAssignment, nodeId, deviceId); //Richard:
  	    	//Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));

  	    }
    }

//Richard: end my addition

/*
 * Richard: end block to learn callback mechanism
 */



NS_LOG_COMPONENT_DEFINE ("TestMeshScript");
//LogComponentEnable ("YansWifiPhy", LOG_LEVEL_ALL);

/*
 * START atttempt to get channel setting
 */

Ptr<YansWifiPhy>
GetYansWifiPhyPtr (const NetDeviceContainer &nc)
{
  Ptr<WifiNetDevice> wnd = nc.Get (0)->GetObject<WifiNetDevice> ();
  Ptr<WifiPhy> wp = wnd->GetPhy ();
  return wp->GetObject<YansWifiPhy> ();
}

Ptr<YansWifiPhy> phySta;
/*
 * END attempt to get channel setting [18 Feb 2019]
 */

class MeshTest
{
public:
  /// Init test
  MeshTest ();
  /// Configure test from command line arguments
  void Configure (int argc, char ** argv);
  /// Run test
  int Run ();
  void BeaconDetection(std::string nodeId, std::string  deviceId, uint16_t  channelNum); //Richard: this is my addition [23 February 2019]
  void  MonitorSniffTx2 (std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu);
  void MonitorSniffRx2 (std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise);
  void setFlag( bool found); //beacon detected= true
  bool getFlag();
  int flagNode5 = 0;

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
  bool node5Associated = false;

  //std::ostringstream msg; //Richard, I only got successful in letting different nodes send different message after commenting this out and declaring it as a local variable in ReceivePacket() [28 Feb 2019]
  //std::string msg; //Richard
  /// List of network nodes
  NodeContainer nodes; // nodes2;
  /// List of all mesh point devices
  NetDeviceContainer meshDevices, meshDevices2;
  //Addresses of interfaces:
  Ipv4InterfaceContainer interfaces, interfaces2;
  // MeshHelper. Report is not static methods
  MeshHelper mesh;// mesh2;
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
  uint64_t m_maxPktsPerFile = 900000; //Richard, trying to explicitly set max  packet per trace file limit. Default is 100000



 }; //end MeshTest declaration



MeshTest::MeshTest () :
/*
 * Richard. it appears there is an issue with m_xSiZe and m_ySize: when set to equal even numbers like
 * 6x6 or 4x4, the simulation exits with an error
 */
  m_xSize (5),
  m_ySize (1), //total number of nodes = m_xSize * m_ySize
  //Richard: reducing m_step to less than 50, results in crazy interference and messes up multi-hoping as can be seen from visualizer
  m_step (100.0), //Richard: originally set to "100". I presume m_step is the distance between nodes, which means total grid horizontal distance = m_xSize *m_step
  m_randomStart (0.1),
  m_totalTime (10), //Richard: simulation time in seconds, I think?
  /*
   * ******************************************************************************
   *  This value of packetInterval (0.00396500) gives throughput that's roughly equal to
   *  manet-routing-compare.cc throughput when network consists of two nodes only.
   *  Roughly in that AODV and OLSR throughput is 2055.02 whereas, HWMP is 2055.76.
   *  With this amount of difference, we can safely assume that manet-routing-compare_b.cc and mesh_c.cc are set up identically
   *  ****************************************************************************
   */
  m_packetInterval (0.1), //Richard: originally (0.1), now set to 0.0001 does seem to affect throughput; this means packet is sent every <interval> second over simulation period
  m_packetSize (1024), //Richard: found it at 1024
  m_nIfaces (1), //Richard: this might be a good start to setup multiradio nodes
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
  m_txp(20), //Richard: values are in dBm rather than dB
  m_rxGain(0), //Richard: increasing the rx/tx Gain obliterates the multi-hoping
  m_txGain(0) //Richard

{
}
void
MeshTest::Configure (int argc, char *argv[])
{
	/*START attempt add payload
	 *
	 */
	//msg << "Hello World, here I am" << '\0'; //Richard
	//m_packetSize += msg.str().length(); //Richard
	/*
	 * End attempt to add payload
	 */

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



  // nodes2.Create(28); //creates n nodes




  // Configure YansWifiChannel
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default (); //Richard: changed this initialisation to match what's in "manet-routing-compare.cc"


  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default (); //Richard: changed this initialisation to match what's in "manet-routing-compare.cc"

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


 //wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel"); //Richard: added this to match what's in manet-routing-compare.cc, but no luck in making the setups identical:(

 //wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel"); //Richard: added this to match what's in manet-routing-compare.cc
//FrissPropagationLossModel yields zero throughput
 //Useful post: https://groups.google.com/forum/#!msg/ns-3-users/w8rOfbJ7gB8/eu8y63tzI6wJ;context-place=searchin/ns-3-users/propagationlossmodel$20mesh$20network%7Csort:date


  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("TxPowerStart",DoubleValue (m_txp)); //Richard
  wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txp)); //Richard
  //wifiPhy.Set ("TxPowerStart",DoubleValue (1000.0)); //Richard: try this?
  //wifiPhy.Set ("TxPowerEnd", DoubleValue (1000.0)); //Richard: try this?
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
  //mesh.SetMacType ("RandomStart", TimeValue (Seconds (m_randomStart))); //original line

  mesh.SetMacType ("RandomStart", TimeValue (Seconds (0.5)), "BeaconInterval", TimeValue (Seconds (0.5)), "BeaconGeneration", BooleanValue (true) ); //Richard: my version. default beaconInterval is 0.5

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
  mesh.SetStandard(WIFI_PHY_STANDARD_80211a); //Richard: this is default i.e. when not specified meshHelper falls on this stanadrd
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211b); //RICHARD
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211g);
  //mesh.SetStandard(WIFI_PHY_STANDARD_80211_10MHZ); //RICHARD
 //mesh.SetStandard(WIFI_PHY_STANDARD_80211_5MHZ); //RICHARD
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





   //mesh2 = mesh;






  /*
   * Richard: uncomment this to specify Datamode
   * Keep in mind, 802.22a standard is the one that uses ofdm
   */
  mesh.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate6Mbps"));




  //mesh2.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue  ("OfdmRate12Mbps"));



  // Install protocols and return container if MeshPointDevices
  meshDevices = mesh.Install (wifiPhy, nodes);



  meshDevices2 = meshDevices;



 // meshDevices2 = mesh2.Install (wifiPhy, nodes2);

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



  /*
   * Test post-install reconfiguration
   */

  //* ******************************************************************************************888888
   //*
   //*This line:Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(36));
   //*  works in setting the channel
   //*

/*
 * To set the channel for all nodes and all interfaces on the nodes:
 */
 //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));

   /*
    * Try setting channel manually for nodes 1-5:
    */

  Config::Set ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
  Config::Set ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
  Config::Set ("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
  Config::Set ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(48));
  Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(0));

  /*
   * Try to start off with passive scanning by disabbling beacongeneration
   */
  Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BeaconGeneration", BooleanValue (false));
  Config::Set ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BeaconGeneration", BooleanValue (false));

  //set rxnoise for node[3] only
  //Config::Set ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8)); //DEFAULT IS 7 or possibly less
  //set rxnoise for all nodes
  // Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (7)); //DEFAULT IS 7 or possibly less

 /*
 * Richard for the innner 3x3 at the center of the 7x7 grid
 */
  Config::Set ("/NodeList/8/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/9/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/10/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/11/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/12/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));


  Config::Set ("/NodeList/15/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/16/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/17/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/18/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/19/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));

  Config::Set ("/NodeList/22/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/23/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/24/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/25/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/26/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));

  Config::Set ("/NodeList/29/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/30/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/31/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/32/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/33/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));

  Config::Set ("/NodeList/36/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/37/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/38/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/39/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));
  Config::Set ("/NodeList/40/DeviceList/*/$ns3::WifiNetDevice/Phy/RxNoiseFigure", DoubleValue (8));

  /*
   * Richard; test logic to get nodes noise
   */

 //Config::ConnectWithoutContext("/NodeList/[0]/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSnifferRx));
  std::cout<<"In MeshTest createNodes()"<<std::endl;
  // Setup mobility - static grid topology
  MobilityHelper mobility;

  /**************************************************************
   * Richard: use this block to position nodes in a X * Y grid fashion
   * **********************************************************
   */

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (m_step),
                                 "DeltaY", DoubleValue (m_step),
                                 "GridWidth", UintegerValue (m_xSize),
                                 "LayoutType", StringValue ("RowFirst"));


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
   * Richard: when using manual positioning, remember to uncomment "mobility.SetPositionAllocator..." statement down below
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

/*
   positionAlloc->Add (Vector (  0, 350, 0)); //position of first node in "nodes"

   positionAlloc->Add (Vector (  m_step, 350, 0)); //TEMPORAL: position second node in "nodes"

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

/*
 *Richard: uncomment this if not using X*Y grid
 */
 // mobility.SetPositionAllocator (positionAlloc);


  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);


 // mobility.Install (nodes2);


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

 // internetStack.Install(nodes2); //Richard
  //interfaces2 = address.Assign (meshDevices2); //Richard
}
void
MeshTest::InstallApplication ()
{

  UdpEchoServerHelper echoServer (9);
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (0));
  serverApps.Start (Seconds (0.0));
  serverApps.Stop (Seconds (m_totalTime));
  /*
   * Richard: next line specifies destination in FlowID1
   */
  UdpEchoClientHelper echoClient (interfaces.GetAddress (0), 9);
  //Richard: comment these out, work with traffic in run() only and to see if throughput goes up
  echoClient.SetAttribute ("MaxPackets", UintegerValue ((uint32_t)(m_totalTime*(1/m_packetInterval))));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (m_packetInterval)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));


  /*
   * Richard; install echo client on the last node in nodes container.
   * nodes2 has three nodes that are placed in between so they are able to forward traffic to the echoclient.
   * Richard: FlowID1: the following line specifies the source node
   */

  ApplicationContainer clientApps = echoClient.Install (nodes.Get (m_xSize*m_ySize-1)); //Richard: original echoclient

  //ApplicationContainer clientApps = echoClient.Install (nodes2.Get (27)); //Richard: when using two node containers


echoClient.SetFill(clientApps.Get(0), "send this"); //this sends ".." to the server.


  //Richard: try installing echoClient on third node in the other container? .. it worked!
  //ApplicationContainer clientApps = echoClient.Install (nodes2.Get (2));

  std::cout<<"In MeshTest::InstallApplication()..."<<std::endl; //Richard


  /*****************
   * START ATTEMPT to fire SetupPacketReceive() from here [25 Feb 2019
   */

  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
   // onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
   // onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));


    int nsinks = 4;
    for (int i = 0; i < nsinks; i++) //Richard.
    {
     std::cout<<" Here"<<std::endl;
  	  Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(i), nodes.Get(i));
  	  AddressValue remoteAddress (InetSocketAddress (interfaces.GetAddress(i), port ));
  	  onoff1.SetAttribute ("Remote", remoteAddress); //this specifies the receiving or destination node. Richard [26 Feb 2018]

  	  Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
  	  /*
  	   * NOTE on nodes.Get(i+?) in next line. It's important to ensure that the "i+?" does not ever evaluate to greater than
  	   *  or equal to the number of nodes in the container.
  	   *  If that's violated, the simulation fails to take off and exists with a SIGSEGV error.
  	   */
  	  ApplicationContainer temp = onoff1.Install (nodes.Get (i+1)); //this specifies the sending or source node [26 Feb 2019]. Richard. (original line)
  	 // ApplicationContainer temp = onoff1.Install (nodes.Get (24)); //Richard.



  	  temp.Start (Seconds (0.0)); //Richard this works fine
  	// temp.Start (Seconds (var->GetValue (1,2))); //Richard: trying out this approach from manet-routing-compare.cc to throw in some randomness
  	  temp.Stop (Seconds (m_totalTime));
  	  std::cout<<" End here "<<std::endl;
   }

    /*****************
      * END ATTEMPT to fire SetupPacketReceive() from here [25 Feb 2019
      */


  /*
   * Richard: when the next line is uncommented, flowmonitor returns stats for a single flow only
   * whereas, when commented out, flowmonitor has two flows i.e A->B and B->A
   */
  //Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(0), nodes.Get(0)); //Richard: show each transmission??
  //Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(0), nodes.Get(7));

  //Richard: try to configure a specific node with something e.g ip address 10.1.1.100

  /****************************************************************************************
   * Richard:    this line sets the channelWidth for all nodes. The ALL is in the wildcard (*)
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
  std::ostringstream filename_out;
  std::ostringstream oss;
  ns3::DataRateValue dv;
  DataRate dataRate;// = socket->GetNode()->GetAttribute("DataRate", dv);

  oss <<"At time: "<< Simulator::Now ().GetSeconds () << " Node# " << socket->GetNode ()->GetId ();
  filename_out << "./Richard_output_logs/node#_" << socket->GetNode ()->GetId ()<< ".txt"; //name output file by node ID


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

  /************************************8
   * Attempting to send some useful info to file
   */
  std::ofstream out;
  out.open (filename_out.str ().c_str ());
  if (!out.is_open ())
    {
        std::cerr << "Error: Can't open file " << filename_out.str () << "\n";

    }
  else
  {
	  out<<"At time "<< Simulator::Now ().GetSeconds ()<< " Node# " << socket->GetNode ()->GetId () <<std::endl;
  }


  out.close();


  return oss.str ();
}



/*****************
 * NOTE:
 * I commented out SetupPacketReceive() and ReceivePacket () because current throughput does not appear
 * to be dependent on these two functions. By the way, I jacked these code fragments from manet-routing-compare.cc
 * [25 Feb 2019]
 */

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

  Ptr<Packet> packet; //original line

	/*
	 * Attempting to specify payload. [ Richard , 26 February 2019]
	 *
	 * */

//  Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);

  Address senderAddress;
  std::ostringstream msg; //This should be local variable. I struggled to customise it when it's a global variable [Richard, 28 February, 2019]


  while ((packet = socket->RecvFrom (senderAddress)))
    {
	  /*
	   * I'm adding addr here.
	   * My intention is to use it (possibly) to customise payload based on senderAddress to mimick node's respective channel lists
	   * [27 Feb 2019]
	   * It appears this block's code executes nonlinearly. I say that because I've struggled to customise data piece sent using if statements.
	   */
	  InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);


	   if (addr.GetIpv4()==Ipv4Address("10.1.1.5"))
	  {
		   //msg << "5 * * * * *" ;
		   msg << "5 * * * * *" << '\0';
		  //msg = "5 * * * * *" ; //Richard
		   packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);
		   //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
		   std::cout<<"Block: if 10.1.1.5"<<std::endl;

	  }
	   if (addr.GetIpv4()==Ipv4Address("10.1.1.4"))
	  {
		   msg << "4 * * * * *" ;
	  		//msg << "4 * * * * *" << '\0';
	  		//msg = "4 * * * * *" ;
	  		packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);
	  		//NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
	  		std::cout<<"Block: if 10.1.1.4"<<std::endl;
	  }

	   if (addr.GetIpv4()==Ipv4Address("10.1.1.3"))
	  {
		   msg << "3 * * * * *";
		  // msg << "4 * * * * *" << '\0';
		  //msg = "3 * * * * *";
		  packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);
		  //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
		  std::cout<<"Block: if 10.1.1.3"<<std::endl;
	  }
	  if (addr.GetIpv4()==Ipv4Address("10.1.1.2"))
	  {
		   //msg << "2 * * * * *";
		  msg << "2 * * * * *" << '\0';
		 // msg = "2 * * * * *" ;
		  packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);
		  //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
		  std::cout<<"Block: if 10.1.1.2"<<std::endl;
	  }
	   if (addr.GetIpv4()==Ipv4Address("10.1.1.1"))
	  {
		   msg << "1 * * * * *" ;
		  //msg << "1 * * * * *" << '\0';
		  //msg = "1 * * * * *" ;
		  packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);
		  //NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
		  std::cout<<"Block: if 10.1.1.1"<<std::endl;
	  }
	   /*
	  else
	  {
		  msg << "unknown data";
		  //msg << "unknown data" << '\0';
		 // msg = "unknown data" ;
		  packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize);
	  }
	   */
/*
 * Pack the data in the packet
 */
	  // packet = Create<Packet> ((uint8_t*) msg.str().c_str(), msg.str().length()+ m_packetSize); //Richard, aha moment! moving this line to hear did the trick [27 Feb 2019]



      bytesTotal += packet->GetSize ();
      packetsReceived += 1;

      /* Richard:
       * Note packed ID and save it for later analysis.
       * I particularly would like to distinguish data packets from HWMP control packets when tracing path
       */

      std::cout<<" In MeshTest::ReceivePacket(): "<<" Packet ID: "<< packet->GetUid() <<std::endl;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress)); //Original position of line [Richard, 27 Feb 2019]
      /*Attempting to print packet contents: */
      NS_LOG_UNCOND ("content of packet " << *packet); //this alone prints only the payload size
      /* or alternatively*/
     // packet->Print(std::cout); //this also works, prints out "Payload (size=1024)" //this alone prints only the payload size
/*
 * Attempting to read the actual data sent/received, not just the payload size
 */
      uint8_t *buffer = new uint8_t[packet->GetSize ()];
      packet->CopyData(buffer, packet->GetSize ());

      std::string s = std::string((char*)buffer);
      std::cout<<"Received: "<<s<< " from "<<addr.GetIpv4() <<std::endl;
     // std::cout<<"Received: "<<s<< " from " <<std::endl;



      //reset msg
      //msg<<'\0';
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


  //blank out the last output file and write the column headers
    std::ofstream out (out_file_for_alll_nodes.c_str ()); //a little more generic
  //std::ofstream out ("/home/richard/eclipse_workspace/ns3_programming/Richard_output_logs/node#_ALL.csv"); //hardwired

   if(out.fail())
   {
   	 std::cout<<"Failed to open log file: \""<<out_file_for_alll_nodes.c_str ()<<"\""<<std::endl;
   	 exit(1);
   }
   else
   {
	out<<"Description: "
			"nodeID is indicates the node and interface that fired the trace call back * "
			"channel is the node's operating channel * "
			"time indicates the time at which this occurred *"
			"I'm using MonitorSnifferTx and MonitorSnifferRx trace call functions * "
			"which means that a call back is made whenever a node transmits or receives a packet * "
			"I used 1 and 0 in the [transmitted] and [received] fields to indicate whether call back was triggered by a frame transmission or frame reception. "
			"source indicates the source of the packet. If source = 00:00:00:00:00:00 * it means this is an ACK frame "
			"According to the standard an ACK does not carry the sender address  only the destination. "
			"Furthermore it may be worthwhile to read up on the four kinds of addresses: "
			" i) Receiver Address ii)  Destination Address iii) Transmitter Address iv)  Source Address "
			"accessed via hdr.GetAddr1() * hdr.GetAddr2() * hdr.GetAddr3() *  hdr.GetAddr4() where hdr is a variable of WifiMacHeader type.  "
			",,,,,,,"<<std::endl;
    out <<"nodeID, " <<
    "channel, " <<
    "time, " <<
    "transmitted, " <<
    "received, " <<
    "source, " <<
	"destination, "<<
	"packet->GetUid()"<<
    std::endl;
    out.close ();
   }


  ns3::PacketMetadata::Enable (); //Richard: trying to see packet movement in netanim
  //ns3::AnimationInterface & EnableIpv4RouteTracking ("hwmp-route.xml", Seconds (0), Seconds (120), Seconds (0.25));

  CreateNodes ();
  InstallInternetStack ();
  InstallApplication ();

  //LogComponentEnable ("YansWifiPhy", LOG_LEVEL_ALL); //Richard: outputs info useful for debugging

  /*************
   * Richard: run some app to send packets back and forth for throughput measurements
   * or, consider moving this logic to InstallApplication()?
   */

/*
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));


  int nsinks = 5;
  for (int i = 1; i < nsinks; i++) //Richard.
  {
	  std::cout<<" Here";
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
  /***************************************************************************
   * NOTES (light bulbs coming on).
   * This is how Simulator::Schedule (... works:
   * Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this);
   * means that Report () function will run at m_totalTime.
   * In this case, the Report () function runs at the end of the simulation because m_totalTime is the simulation duration.
   * If for example, m_totalTime=10 and I say,  " Simulator::Schedule (Seconds (5), &MeshTest::Report, this);"
   * It means Report () will be triggered at 5 midway through the simulation
   *
   */
  Simulator::Schedule (Seconds (m_totalTime), &MeshTest::Report, this); //Richard
  //Simulator::Schedule (Seconds (1), &MeshTest::ChannelAssignment, this); //Richard
  Simulator::Schedule (Seconds (m_totalTime), &show ); //Richard

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
  AnimationInterface  anim ("./visualisation_files/mesh_D_m_hwmp.xml"); //trace file
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
  Ptr<Node> node = nodes.Get(4);
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
/*
   MeshHelper g;
     Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dynamic-global-routing-table.log??", std::ios::out);
     g.PrintRoutingTableAllAt (Seconds (5), routingStream);
    */
     //g.PrintRoutingTableAllEvery (Seconds (2), routingStream);

/*START try of setDown, setUp */
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
      uint32_t ipv4ifIndex = 1;
      Simulator::Schedule (Seconds (1),&Ipv4::SetDown,ipv4, ipv4ifIndex);
      Simulator::Schedule (Seconds (2),&Ipv4::SetUp,ipv4, ipv4ifIndex);


     //Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()), &func1);
     //Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()+1), &func2);
 /*END try of setDown, setUp */


 /*********************
  * \START attempt to alter beacon interval.
  * The default beacon interval is defined in m_mesh/model/mesh-wifi-interface-mac.cc
  * Whereas, I'm trying to find a way to change on a per node basis
  *
  */
      	  /*EXPLORATION 1*/
     Ptr<WifiNetDevice> wifidevice = node->GetObject<WifiNetDevice>();
      //Ptr<ApWifiMac> apmac = DynamicCast<ApWifiMac>(wifidevice->GetMac());
      //Ptr<MeshWifiInterfaceMac> apmac = DynamicCast<MeshWifiInterfaceMac>(wifidevice->GetMac());

        //Ptr<NetDevice> ntwenu = node->GetDevice(1);
     //Ptr<NetDevice> wifidevice2 = node->GetObject<NetDevice>();

       //wifidevice->SetAttribute("BeaconInterval", TimeValue (Seconds (0.5)));
//wifidevice->SetMacType("RandomStart", TimeValue (Seconds (0.5)), "BeaconInterval", TimeValue (Seconds (0.5)));

       //Simulator::Schedule(Seconds(5), &ApWifiMac::SetBeaconInterval, apmac, NanoSeconds(922337203685477)); //noluck to far
       //Simulator::Schedule(Seconds(1), &MeshWifiInterfaceMac::SetBeaconInterval, ntwenu, TimeValue (Seconds (505)));

      /*EXPLORATION 2*/ //This approach might be it! [21 March 2019]. I've observed a reduction in the number of transmissions as the beaconInterval increases or when beacongeneration is false
     //Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BeaconInterval", TimeValue (Seconds (9.5)));
     //Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/BeaconGeneration", BooleanValue (true));
     //Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/RandomStart", TimeValue (Seconds (9.5)));

     /*EXPLORATION 3*/

      //PointerValue ntwenu;
      //nodes.Get(4)->GetAttribute ("RandomStart",ntwenu);

      /*END attempt to alter beacon interval*/




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

  //uncomment these next two lines to get phy datamode displayed and snr

 //Config::Connect("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));
 Config::Connect ("/NodeList/0/DeviceList/*/Phy/State/RxError", MakeCallback (&PhyRxErrorTrace));

  //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&PhyTrace));

 /*
  * Richard: any node:
  */
 //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy", MakeCallback (&PhyTrace));

 /*
  * Richard: only the first node, which helps reduce run time
  */

 /*
  * So far no luck in getting desired output with PhyTrace call back.
  * As a matter of fact, I don't think PhyTrace is firing at all [18 Feb 2019]
  */

 Config::Connect ("/NodeList/4/DeviceList/1/$ns3::WifiNetDevice/Phy", MakeCallback (&PhyTrace));
 //Config::ConnectWithoutContext("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy", MakeCallback (&PhyTrace));

 //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx", MakeCallback (&PhyTrace));
 //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiPhy::MonitorSnifferTxTracedCallback", MakeCallback (&MonitorSniffRx)); //no luck in getting this to work
 //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiPhy::MonitorSnifferTxTracedCallback", MakeCallback (&MonitorSniffRx)); //no luck in getting this to work
 //Config::ConnectWithoutContext("/NodeList/0/DeviceList/0/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSniffRx)); //no luck in getting this to work
 //Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSniffRx)); //no luck in getting this to work


 //Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/WifiPhy/MonitorSnifferRxTracedCallback", MakeCallback (&MonitorSniffRx)); //no luck in getting this to work

 /*********************************
  * The line below works (finally): Config::ConnectWithoutContext ("/NodeList/[i]/DeviceList/[i]/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));
  * I knew the answer was somewhere in MonitorSniffRx, so I used it as search term -that's how I found the inspiration that finally worked
  * Source of inspiration: https://groups.google.com/forum/#!searchin/ns-3-users/MakeCallback$20($26MonitorSniffRx%7Csort:date/ns-3-users/aI8_qu37KXo/7Ce__87VBwAJ
  * [26 Feb 2019]
  ****************************
  */
 /**************
  * config::connect vs config::connectWithoutContext:
  * Firstly, connect is used when "context" is included in the function parameter list.
  * So to use "connectWithoutContext", "context" should be ommitted in the function parameter list.
  * Using context makes it possible to get the context i.e. to display node that fired the callback.
  * The context looks something like, /NodeList/1/DeviceList/1/Phy/MonitorSnifferTx.
  * Given the context, it's quite easy to manipulate the string and get the node # and node interface that fired the trace call
  */

 /*Richard. MonitorSnifferRx fireS whenever node receives packet, any packet */
// Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));


 /*Richard. MonitorSnifferTx fireS whenever node transmits packet, any packet to anywhere */
 //Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/Phy/MonitorSnifferTx", MakeCallback (&MonitorSniffTx));
// Config::Connect("/NodeList/*/DeviceList/*/Phy/MonitorSnifferTx", MakeCallback (&MonitorSniffTx));

 Config::Connect("/NodeList/*/DeviceList/*/Phy/MonitorSnifferTx", MakeCallback (&MeshTest::MonitorSniffTx2, this));//Richard: lets use the new version that's inside MeshTest class

 Config::Connect("/NodeList/*/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MeshTest::MonitorSniffRx2, this));

// Config::ConnectWithoutContext("/NodeList/[0]/DeviceList/*/$ns3::WifiPhy::MonitorSnifferRxTracedCallback", MakeCallback (&PhyTrace));


  Simulator::Run ();
  //the "." before the file path moves one step out of directory of execution
  flowmon->SerializeToXmlFile (("./folder_for_measurements/mesh_D_m_hwmp.xml"), false, false); //my version ~Richard

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

/*
 * Richard:
 * I've added ChannelAssignment() to:
 * i) change channel setting at run-time as scheduled in Run()
 * ii)
 */

void MeshTest::BeaconDetection(std::string nodeId, std::string  deviceId, uint16_t  channelNum)
{

	//mtx.lock(); //No impact on simulations whatsoever
	//Config::Connect("/NodeList/4/DeviceList/1/Phy/MonitorSnifferRx", MakeCallback (&MeshTest::MonitorSniffRx2, this)); //let me try putting line here

	std::cout<<"In MeshTest::ChannelAssignment: "<<std::endl;
	std::string channelSettingString = "/NodeList/"+nodeId+"/DeviceList/"+deviceId+"/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber";

	std::cout<<"Channel setting string: "<<channelSettingString<<std::endl;

	/*
	 * Do something crude, set or reset channel number, just for the fun of it [22 Feb 2019]
	 */

	// Config::Set ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
	// Config::Set ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
	// Config::Set ("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
	// Config::Set ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));
	// Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));

	    /*START try of setDown, setUp */
/*
	    Ptr<Node> node = nodes.Get(4);

	   Ptr<Ipv4> ipv4 = node->GetObject<Ipv4> ();
	        uint32_t ipv4ifIndex = 1;
 */
/*
 * Richard: I have not seen any benefit of this for my cause at this stage [11 March 2019]
 * because packets sent is same regardless of whether I interface is down or up.
 * Maybe it's because SetDown()/SetUp works at the IP layer whereas the what I'm really looking at are frames being transmitted at the lower layer
 */
	     // Simulator::Schedule (Seconds (Simulator::Now ().GetSeconds ()),&Ipv4::SetDown,ipv4, ipv4ifIndex);
	       //Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()), &func1);
	       //Simulator::Schedule (Seconds (Simulator::Now ().GetSeconds ()+10),&Ipv4::SetUp,ipv4, ipv4ifIndex);

	      // Simulator::Schedule (Seconds (Simulator::Now ().GetSeconds ()+15),&Ipv4::SetDown,ipv4, ipv4ifIndex);
	       //Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()+2), &func2);
	      // Simulator::Schedule (Seconds (Simulator::Now ().GetSeconds ()),&Ipv4::SetUp,ipv4, ipv4ifIndex);

	   /*END try of setDown, setUp */

	      //*****start channel list and iterate over it
	      /*
	       * List of supported channels in ns3 can be found in ~wifi-phy.cc  line# +/-92
	       */
	/*
	 * START Thread this block of code:
	 */

	//std::thread t1([&]{ /* The '&' is a fix to the compilation error: "in lambda function ..... this was not captured for this lambda function" */
	      std::list<int> listOfchannels ={36, 40, 44}; //36, 44, 48, 40, et al.
	      std::list <int> :: iterator it;
	      //uint64_t  channelNum = 0; //initialise to 0
	      double delay = 0; //using double for granularity



	      //if (detected ==true)
	     // {
	    //	  std::cout<<"Beacon detected on channel### : "<<channelNum<<std::endl;
	     // }
	    //  else
	      //{
	    //	  std::cout<<"Entering while loop ..."<<std::endl;
	     // while (detected==false and delay < 10)//important to cap sanning time otherwise loop runs eternally
	     // {

	    	 // BeaconDetected = &node1Associated; //BeaconDetected local variable pointing to the address of "detected" global variable
	          for(it = listOfchannels.begin(); it != listOfchannels.end(); ++it)
	          {

	              std::cout << "Scanning channel " << *it <<std::endl;
	              //set channel
	              //listen on this channel for a moment

	              channelNum = *it;
	              std::cout << "Scanning channel3 " <<channelNum<<std::endl;
	              //Config::Set (channelSettingString, UintegerValue(channelNum)); //set channel directly and instantly


	              Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()+delay), &setChannel, channelSettingString,  channelNum); //call setChannel() at scheduled time
	              //setChannel (channelSettingString,  channelNum); //call setChannel() directly


	              for (int count =0;count < channelListenDuration; count++)
	              {
	            	  //listen for beacon
	            	   std::cout<<"Listening on channel "<<channelNum<<std::endl;
	            	  //Config::Connect("/NodeList/4/DeviceList/1/Phy/MonitorSnifferRx", MakeCallback (&MeshTest::MonitorSniffRx2, this)); //lets see if this fires
	            	  // std::this_thread::sleep_for (std::chrono::seconds(channelListenDuration));//lets try listening for 1 second
	            	   detected = MeshTest::getFlag();
	              }
	            //  std::this_thread::sleep_for (std::chrono::seconds(channelListenDuration));//lets try listening for 1 second
	             /*
	              * This delay in between setting channels is critical.
	              * If too small, the following error occurs: assert failed. cond="!IsStateSwitching ()", file=../src/wifi/model/wifi-phy.cc, line=1393.
	              * "0.0003" is the smallest I've gotten to work. "1", "2", "3", "4", ... are good candidates.
	              * When beaconGeneration is disabled for two or more nodes, I've found myself needing to increase this a notch.
	              */
	              delay=delay+0.0005;

	              if (detected ==true)
	              {
	            	   std::cout<<"Beacon detected on channel... : "<<channelNum<<std::endl;
	              }
	              else
	              {
	            	  std::cout<<"Beacon not detected on channel: "<<channelNum<<", delay ="<<delay<<std::endl;
	              //std::cout<<"Node 1 status (associated?): " <<node1Associated<<std::endl;
	              std::cout<<"Node 5 status (associated?): " <<MeshTest::getFlag()<<std::endl;
	              }
	          } //****** end for 'it' channel list iteration


//mtx.unlock(); //no effect

	    //  } //end do-while
	   //  }//end if-else
	 //}); //END thread
	//t1.join();




}
/*****************************************/
void  MeshTest::MonitorSniffTx2 (std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu)
    {
    	 //std::cout << "From wifi-phy.cc line# 2193: signal_power = " << signalNoise.signal << "  noise_power = "<< signalNoise.noise << "   packet_size = " << packet->GetSize() << std::endl;
    	 std::cout<<"**************** " <<std::endl;
  	 std::cout << "From NotifyMonitorSniffTx2() in mesh_D.cc line# ~1793:"<< "Time:"<<Simulator::Now ().GetSeconds ()<<" Uid: "<<packet->GetUid()  <<"channel# = " << channelNumber << "  channel FreqMhz = "<< channelFreqMhz << std::endl;
  	  std::cout<<"**************** " <<std::endl;

    // m_phyMonitorSniffRxTrace (packet, channelFreqMhz, channelNumber, txVector, aMpdu, signalNoise);

 /************************************
  * I've created the file and written the column headers in MeshTest::Run ()
  * So, at this stage, I just want to append to the file.
  * NOTE: "std::ofstream out (out_file_for_alll_nodes.c_str ());" overwrites the file,
  * whereas 'std::ofstream out (out_file_for_alll_nodes.c_str (), std::ios::app);" appends to the file
  *
  * *****************************************
  * */

  /*****************************************
   * Attempting to get source and destination address of transmitted packet
   */

  	 /*OPTION 1:*/
  	WifiMacHeader hdr;
  	    Ptr<Packet> m_currentPacket;
  	    m_currentPacket = packet->Copy();
  	    m_currentPacket->RemoveHeader (hdr);
  	    Mac48Address source = hdr.GetAddr2();
  		Mac48Address dest = hdr.GetAddr1();
  	 /*OPTION 2: works fine, but I think option 1 is more readable*/
 /*
  	 	        packet->Copy()->RemoveHeader (hdr);
  		  	    Mac48Address source = hdr.GetAddr2();
  		  		Mac48Address dest = hdr.GetAddr1();
*/
  		std::cout<<"Source?: "<<source<<std::endl;
  		std::cout<<"Destination?: "<<dest<<std::endl;


  	std::ofstream out (out_file_for_alll_nodes.c_str (), std::ios::app); //hardwired
  	    out << context <<", "<<
  	 	channelNumber <<", "<<
  	  Simulator::Now ().GetSeconds () <<", "<<
  	    " 1, " <<
  	    " 0, " <<
  	     source <<", "<<
  		dest<<", "<<
  		packet->GetUid()<<
  	    std::endl;
  	    out.close ();

  	  std::string m_nodeId=context.substr( 10,1 );
  	  std::string  m_deviceId=context.substr( 23,1 );

  	int index = std::stoi(m_nodeId);//convert string to int

  	    if (table[index].nodeId == -1 || table[index].channel == 0)
  	    {
  	    	 /**
  	    	   * \todo
  	    	   * Correct channel switching is:
  	    	   *
  	    	   * 1. Interface down, e.g. to stop packets from layer 3
  	    	   * 2. Wait before all output queues will be empty
  	    	   * 3. Switch PHY channel
  	    	   * 4. Interface up
  	    	   *
  	    	   * In the absence of elegant solution, I use a dirty trick:
  	    	   * schedule channel change to occur 1 second from now. [Richard 8 March 2019]
  	    	   */

  	    	/*
  	    	 * NOTE on how substr(pos, len) works:
  	    	 * pos specifies the position, len indicates the number of character to consider from pos onwards.
  	    	 * In my case, the expected context is something like "/NodeList/1/DeviceList/1/Phy/MonitorSnifferTx"
  	    	 * So, to get the node id, I tell substr() to go to position 10 and get 1 character;
  	    	 *      to get the device id, I say get 1 character starting at position 23.
  	    	 */
  	    	//std::string nodeId="4"; std::string  deviceId="*"; UintegerValue chanNum = 40; //earlier line used to test the logic
  	    	//std::cout<<" Context node ID "<<context.substr( 10,1 )<<" device ID: "<<context.substr(23,1)<<std::endl; //works fine -hard wired string positions

  	    	//std::string m_nodeId=context.substr( 10,1 );
  	    	//std::string  m_deviceId=context.substr( 23,1 );
  	    	//UintegerValue chanNum = 40;

  	    	/*
  	    	 * Richard. Traditionally, the function would be called as channelAssignment (var1, var2);
  	    	 * But notice how it's down when scheduled: &channelAssignment, var1, var2 i.e. no bracket
  	    	 */

  	    	/*Schedule call ChannelAssignment (_) to run at specific time i.e. a little later from now*/
  	    	//Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()+1), &MeshTest::ChannelAssignment, nodeId, deviceId); //Richard: if function being called is outside the class -no "this"
  	    	//Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()), &MeshTest::ChannelAssignment, m_nodeId, m_deviceId, this); //Richard: add "this" for function that is class member
  	    	//Simulator::Schedule (Seconds ( Simulator::Now ().GetSeconds ()), &ChannelAssignment, m_nodeId, m_deviceId); //Richard: add "this" for function that is class member

  	    	/* or just call the function and handle the scheduling from within the function*/

  	    	MeshTest::BeaconDetection( m_nodeId, m_deviceId, channelNumber);

  	    	//Config::Set ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/ChannelNumber", UintegerValue(40));

  	    	flagNode5 = 1;//attempt to ensure node#5 invokes channelAssignment () only once.
  	    	//std::cout<<" FladNode5 = "<<flagNode5<<std::endl;

  	    }

    }

void MeshTest::MonitorSniffRx2 (std::string context, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, WifiTxVector txVector, MpduInfo aMpdu, SignalNoiseDbm signalNoise)
  {
  	 //std::cout << "From wifi-phy.cc line# 2193: signal_power = " << signalNoise.signal << "  noise_power = "<< signalNoise.noise << "   packet_size = " << packet->GetSize() << std::endl;
  	 std::cout<<"**************** " <<std::endl;
	  std::cout << "From NotifyMonitorSniffRx2() in mesh_D.cc line# ~2003: channel# = " << channelNumber << "  channel FreqMhz = "<< channelFreqMhz << std::endl;
	  std::cout<<"**************** " <<std::endl;
	  // m_phyMonitorSniffRxTrace (packet, channelFreqMhz, channelNumber, txVector, aMpdu, signalNoise);

	 // node1Associated = true;

	/***************************************************************
	 * Check this out: https://groups.google.com/forum/#!searchin/ns-3-users/how$20to$20get$20node$20that$20fired$20trace$20call$20back%7Csort:date/ns-3-users/aI8_qu37KXo/7Ce__87VBwAJ
	 *Adil Alsuhaim raises the issue of there being four addresses: i) Receiver Address. ii)  Destination Address. iii) Transmitter Address; iv)   Source Address.
	 *
	 *Also: https://groups.google.com/forum/#!searchin/ns-3-users/how$20to$20get$20node$20that$20fired$20trace$20call$20back%7Csort:date/ns-3-users/aI8_qu37KXo/7Ce__87VBwAJ
	 */
	  WifiMacHeader hdr;
	    	    Ptr<Packet> m_currentPacket;
	    	    m_currentPacket = packet->Copy();
	    	    m_currentPacket->RemoveHeader (hdr);
	    	    Mac48Address source = hdr.GetAddr2(); /*Look into hdr.GetAddr1(), hdr.GetAddr2(), hdr.GetAddr3(),  hdr.GetAddr4() */
	    		Mac48Address dest = hdr.GetAddr1();


	  std::ofstream out (out_file_for_alll_nodes.c_str (), std::ios::app); //hardwired
	    	    out << context <<", "<<
	    	 	channelNumber <<", "<<
	    	  Simulator::Now ().GetSeconds () <<", "<<
	    	    " 0, " <<
	    	    " 1, " <<
	    	    source <<", "<<
	    		dest<<", "<<
	    		packet->GetUid()<<
	    	    std::endl;
	    	    out.close ();
/*
 * Attempt to set flag
 */
	    	    std::string nodeId=context.substr( 10,1 );
	    	    std::string  deviceId=context.substr( 23,1 );

	    	 	int index = std::stoi(nodeId);//convert string to int

	    	 	std::list<int>::iterator findIter = std::find(table[index].chan.begin(), table[index].chan.end(), channelNumber);//if the node has already received on this channel

	    	  	//if (table[index].nodeId == -1 || table[index].channel == 0)
	    	  	 if (table[index].nodeId == -1 || *findIter != channelNumber)//Has node received beacon on this channel before?
	    	    {
	    	    	/*update table */
	    	  		table[index].nodeId = index;
	    	  		table[index].deviceId = std::stoi(deviceId); //get the interface on which packet came on
	    	  		table[index].channel = channelNumber;
	    	  		table[index].chan.push_back(channelNumber);

	    	    	MeshTest::setFlag(true);
	    	    	std::cout<<"NodeId in **Rx()2? match found: "<<nodeId<<", MeshTest::getFlag() ="<<MeshTest::getFlag()<<", at time: "<<Simulator::Now ().GetSeconds () <<std::endl;
	    	    }
	    	    else
	    	    {
	    	    	//node5Associated=false;
	    	    	MeshTest::setFlag(false);
	    	    	std::cout<<"NodeId in **Rx()2? match NOT found, MeshTest::getFlag()"<<MeshTest::getFlag()<<", at time: "<<Simulator::Now ().GetSeconds () <<std::endl;
	    	    }




  }

void MeshTest::setFlag(bool found)
{
  node5Associated = found;
}


bool MeshTest::getFlag()
{
  return node5Associated;
}
//***************************




int
main (int argc, char *argv[])
{
  std::cout<<"In main(), starting simulation..."<<std::endl;

 // std::vector<ScanStatus> table;
   int nodeCount = 5;

   /*
    * auto populate table. Use -1 to indicate initial value
    */
   for (int i=0;i<nodeCount; i++)
   {
   table.push_back(ScanStatus()); //push back new element with default constructor and then modify it
   table[i].nodeId = -1;
   table[i].deviceId = -1; //0 (zero) is for loopback
   table[i].channel = -1;

   }

  MeshTest t; 
  t.Configure (argc, argv);


  return t.Run ();
}
