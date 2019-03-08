/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011, 2012 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Authors: Marco Miozzo <marco.miozzo@cttc.es>
 *          Nicola Baldo <nbaldo@cttc.es>
 *          Dizhi Zhou <dizhi.zhou@gmail.com>
 *          Biljana Bojovic <bbojovic@cttc.es>
 *
 * Adapted from LTE scheduler tests  
 */

#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LenaSchedulerEval");

#if 0
LenaTestRrFfMacSchedulerSuite::LenaTestRrFfMacSchedulerSuite ()
  : TestSuite ("lte-rr-ff-mac-scheduler", SYSTEM)
{
  NS_LOG_INFO ("creating LenaRrFfMacSchedulerTestCase");

  bool errorModel = true;


  // DOWNLINK- DISTANCE 0 -> MCS 28 -> Itbs 26 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 24 PRB at Itbs 26 -> 2196 -> 2196000 bytes/sec
  // 3 users -> 8 PRB at Itbs 26 -> 749 -> 749000 bytes/sec
  // 6 users -> 4 PRB at Itbs 26 -> 373 -> 373000 bytes/sec
  // 9 user -> 2 PRB at Itbs 26 -> 185 -> 185000 bytes/sec
  // 12 users -> 2 PRB at Itbs 26 -> 185 -> 185000 bytes/sec
  // 15 users -> 2 PRB at Itbs 26 * 0.8 -> 148 -> 148000 bytes/sec
  // UPLINK- DISTANCE 0 -> MCS 28 -> Itbs 26 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 25 PRB at Itbs 26 -> 2292 -> 2292000 bytes/sec
  // 3 users -> 8 PRB at Itbs 26 -> 749 -> 749000 bytes/sec
  // 6 users -> 4 PRB at Itbs 26 -> 373 -> 373000 bytes/sec
  // after the patch enforcing min 3 PRBs per UE:
  // 9 users -> 3 PRB at Itbs 26 -> 277 bytes * 8/9 UE/TTI ->  246220 bytes/sec
  // 12 users -> 3 PRB at Itbs 26 -> 277 bytes * 8/12 UE/TTI -> 184670 bytes/sec
  // 15 users -> 3 PRB at Itbs 26 -> 277 bytes * 8/15 UE/TTI -> 147730 bytes/sec
  AddTestCase (new LenaRrFfMacSchedulerTestCase (1,0,2196000,2292000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (3,0,749000,749000,errorModel), TestCase::QUICK);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (6,0,373000,373000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (9,0,185000,246220,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (12,0,185000,184670,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (15,0,148000,147730,errorModel), TestCase::EXTENSIVE);

  // DOWNLINK - DISTANCE 4800 -> MCS 22 -> Itbs 20 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 24 PRB at Itbs 20 -> 1383 -> 1383000 bytes/sec
  // 3 users -> 8 PRB at Itbs 20 -> 469 -> 469000 bytes/sec
  // 6 users -> 4 PRB at Itbs 20 -> 233 -> 233000 bytes/sec
  // 9 user -> 2 PRB at Itbs 20 -> 113 -> 113000 bytes/sec
  // 12 users -> 2 PRB at Itbs 20 -> 113 -> 113000 bytes/sec
  // 15 users -> 2 PRB at Itbs 20 * 0.8 -> 90.4 -> 90400 bytes/sec
  // UPLINK - DISTANCE 4800 -> MCS 14 -> Itbs 13 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 25 PRB at Itbs 13 -> 807 -> 807000 bytes/sec
  // 3 users -> 8 PRB at Itbs 13 -> 253 -> 253000 bytes/sec
  // 6 users -> 4 PRB at Itbs 13 -> 125 -> 125000 bytes/sec
  // after the patch enforcing min 3 PRBs per UE:
  // 9 users -> 3 PRB at Itbs 13 -> 93  bytes * 8/9 UE/TTI  -> 82667 bytes/sec
  // 12 users -> 3 PRB at Itbs 13 -> 93  bytes * 8/12 UE/TTI  -> 62000 bytes/sec
  // 15 users -> 3 PRB at Itbs 13 -> 93  bytes * 8/15 UE/TTI  -> 49600 bytes/sec
  AddTestCase (new LenaRrFfMacSchedulerTestCase (1,4800,1383000,807000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (3,4800,469000,253000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (6,4800,233000,125000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (9,4800,113000,82667,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (12,4800,113000,62000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (15,4800,90400,49600,errorModel), TestCase::EXTENSIVE);

  // DOWNLINK - DISTANCE 6000 -> MCS 20 -> Itbs 18 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 24 PRB at Itbs 15 -> 1191 -> 1191000 bytes/sec
  // 3 users -> 8 PRB at Itbs 15 -> 389 -> 389000 bytes/sec
  // 6 users -> 4 PRB at Itbs 15 -> 193 -> 193000 bytes/sec
  // 9 user -> 2 PRB at Itbs 15 -> 97 -> 97000 bytes/sec
  // 12 users -> 2 PRB at Itbs 15 -> 97 -> 97000 bytes/sec
  // 15 users -> 2 PRB at Itbs 15 * 0.8 -> 77.6 -> 77600 bytes/sec
  // UPLINK - DISTANCE 6000 -> MCS 12 -> Itbs 11 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 25 PRB at Itbs 11 -> 621 -> 621000 bytes/sec
  // 3 users -> 8 PRB at Itbs 11 -> 201 -> 201000 bytes/sec
  // 6 users -> 4 PRB at Itbs 11 -> 97 -> 97000 bytes/sec
  // 9 users -> 3 PRB at Itbs 11 -> 73 bytes * 8/9 UE/TTI -> 64889 bytes/sec
  // 12 users -> 3 PRB at Itbs 11 -> 73 bytes * 8/12 UE/TTI -> 48667 bytes/sec
  // 15 users -> 3 PRB at Itbs 11 -> 73 bytes * 8/15 UE/TTI -> 38993 bytes/sec
  AddTestCase (new LenaRrFfMacSchedulerTestCase (1,6000,1191000,621000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (3,6000,389000,201000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (6,6000,193000,97000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (9,6000,97000,64889,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (12,6000,97000,48667,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (15,6000,77600,38993,errorModel), TestCase::EXTENSIVE);

  // DOWNLINK - DISTANCE 20000 -> MCS 8 -> Itbs 8 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 24 PRB at Itbs 8 -> 421 -> 421000 bytes/sec
  // 3 users -> 8 PRB at Itbs 8 -> 137 -> 137000 bytes/sec
  // 6 users -> 4 PRB at Itbs 8 -> 67 -> 67000 bytes/sec
  // 9 user -> 2 PRB at Itbs 8 -> 32 -> 32000 bytes/sec
  // 12 users -> 2 PRB at Itbs 8 -> 32 -> 32000 bytes/sec
  // 15 users -> 2 PRB at Itbs 8 * 0.8 -> 25.6 -> 25600 bytes/sec
  // UPLINK - DISTANCE 20000 -> MCS 2 -> Itbs 2 (from table 7.1.7.2.1-1 of 36.213)
  // 1 user -> 25 PRB at Itbs 2 -> 137 -> 137000 bytes/sec
  // 3 users -> 8 PRB at Itbs 2 -> 41 -> 41000 bytes/sec
  // 6 users -> 4 PRB at Itbs 2 -> 22 -> 22000 bytes/sec
  // after the patch enforcing min 3 PRBs per UE:
  // 9 users -> 3 PRB at Itbs 2 -> 18 bytes * 8/9 UE/TTI -> 16000 bytes/sec
  // 12 users -> 3 PRB at Itbs 2 -> 18 bytes * 8/12 UE/TTI -> 12000 bytes/sec
  // 15 users -> 3 PRB at Itbs 2 -> 18 bytes * 8/15 UE/TTI -> 9600 bytes/sec
  AddTestCase (new LenaRrFfMacSchedulerTestCase (1,20000,421000,137000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (3,20000,137000,41000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (6,20000,67000,22000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (9,20000,32000,16000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (12,20000,32000,12000,errorModel), TestCase::EXTENSIVE);
  AddTestCase (new LenaRrFfMacSchedulerTestCase (15,20000,25600,9600,errorModel), TestCase::EXTENSIVE);

  // DOWNLINK - DISTANCE 100000 -> CQI == 0 -> out of range -> 0 bytes/sec
  // UPLINK - DISTANCE 100000 -> CQI == 0 -> out of range -> 0 bytes/sec
  AddTestCase (new LenaRrFfMacSchedulerTestCase (1,100000,0,0,errorModel), TestCase::QUICK);
}

static LenaTestRrFfMacSchedulerSuite lenaTestRrFfMacSchedulerSuite;

std::string 
LenaRrFfMacSchedulerTestCase::BuildNameString (uint16_t nUser, double dist)
{
  std::ostringstream oss;
  oss << nUser << " UEs, distance " << dist << " m";
  return oss.str ();
}

LenaRrFfMacSchedulerTestCase::LenaRrFfMacSchedulerTestCase (uint16_t nUser, double dist, double thrRefDl, double thrRefUl, bool errorModelEnabled)
  : TestCase (BuildNameString (nUser, dist)),              
    m_nUser (nUser),
    m_dist (dist),
    m_thrRefDl (thrRefDl),
    m_thrRefUl (thrRefUl),
    m_errorModelEnabled (errorModelEnabled)
{
}

LenaRrFfMacSchedulerTestCase::~LenaRrFfMacSchedulerTestCase ()
{
}

void
LenaRrFfMacSchedulerTestCase::DoRun (void)

/// DlSchedulingCallbackInfo structure
struct DlSchedulingCallbackInfo
{
  uint32_t frameNo; ///< frame number
  uint32_t subframeNo; ///< subframe number
  uint16_t rnti; ///< RNTI
  uint8_t  mcsTb1; ///< MCS TB1
  uint16_t sizeTb1; ///< size TB1
  uint8_t  mcsTb2; ///< MCS TB2
  uint16_t sizeTb2; ///< size TB2
  uint8_t  componentCarrierId; ///< component carrier ID
};

#endif

// Parse context strings of the form "/NodeList/3/DeviceList/1/Mac/Assoc"
// to extract the NodeId
uint32_t
ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);  // skip "/NodeList/"
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0,pos).c_str ());
}

void
NotifyDlScheduling (std::string context, struct DlSchedulingCallbackInfo info)
{
  std::cout << Simulator::Now ().GetSeconds () << " " << ContextToNodeId (context) << " " << info.rnti << " " << info.sizeTb1 << " " << info.sizeTb2 << std::endl;
}


  // DOWNLINK- DISTANCE 0 -> MCS 28 -> Itbs 26 (from table 7.1.7.2.1-1 of 36.213)
  // 15 users -> 2 PRB at Itbs 26 * 0.8 -> 148 -> 148000 bytes/sec
  // 15 users -> 3 PRB at Itbs 26 -> 277 bytes * 8/15 UE/TTI -> 147730 bytes/sec

int main (int argc, char *argv[])
{
  uint16_t m_nUser = 15; ///< number of UE nodes
  double m_dist = 0; ///< the distance between nodes (m)
  double m_interval = 1; ///< ms
  uint32_t m_packetSize = 200; ///< bytes
  double m_thrRefDl = 148000; ///< the DL throughput reference
  double m_thrRefUl = 147730; ///< the UL throughput reference
  bool m_errorModelEnabled = true; ///< indicates whether the error model is enabled

  CommandLine cmd;
  cmd.Parse (argc, argv);

  if (!m_errorModelEnabled)
    {
      Config::SetDefault ("ns3::LteSpectrumPhy::CtrlErrorModelEnabled", BooleanValue (false));
      Config::SetDefault ("ns3::LteSpectrumPhy::DataErrorModelEnabled", BooleanValue (false));
    }
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (false));

  // This is needed as the RR scheduler does not allocate resources properly for retransmission
  Config::SetDefault ("ns3::LteRlcAm::TxOpportunityForRetxAlwaysBigEnough", BooleanValue (true));

  //Disable Uplink Power Control
  Config::SetDefault ("ns3::LteUePhy::EnableUplinkPowerControl", BooleanValue (false));

  /**
   * Initialize Simulation Scenario: 1 eNB and m_nUser UEs
   */

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  
  lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::FriisSpectrumPropagationLossModel"));
 Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);

  //LogComponentEnable ("TdTbfqFfMacScheduler", LOG_DEBUG);

  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.001)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // interface 0 is localhost, 1 is the p2p device
  Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);

  // Create Nodes: eNodeB and UE
  NodeContainer enbNodes;
  NodeContainer ueNodes;
  enbNodes.Create (1);
  ueNodes.Create (m_nUser);

  // Install Mobility Model
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (enbNodes);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (ueNodes);

  // Create Devices and install them in the Nodes (eNB and UE)
  NetDeviceContainer enbDevs;
  NetDeviceContainer ueDevs;
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");
  enbDevs = lteHelper->InstallEnbDevice (enbNodes);
  ueDevs = lteHelper->InstallUeDevice (ueNodes);

  Ptr<LteEnbNetDevice> lteEnbDev = enbDevs.Get (0)->GetObject<LteEnbNetDevice> ();
  Ptr<LteEnbPhy> enbPhy = lteEnbDev->GetPhy ();
  enbPhy->SetAttribute ("TxPower", DoubleValue (30.0));
  enbPhy->SetAttribute ("NoiseFigure", DoubleValue (5.0));

  // Set UEs' position and power
  for (int i = 0; i < m_nUser; i++)
    {
      Ptr<ConstantPositionMobilityModel> mm = ueNodes.Get (i)->GetObject<ConstantPositionMobilityModel> ();
      mm->SetPosition (Vector (m_dist, 0.0, 0.0));
      Ptr<LteUeNetDevice> lteUeDev = ueDevs.Get (i)->GetObject<LteUeNetDevice> ();
      Ptr<LteUePhy> uePhy = lteUeDev->GetPhy ();
      uePhy->SetAttribute ("TxPower", DoubleValue (23.0));
      uePhy->SetAttribute ("NoiseFigure", DoubleValue (9.0));
    }

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  // Assign IP address to UEs
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<Node> ueNode = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

  // Attach a UE to a eNB
  lteHelper->Attach (ueDevs, enbDevs.Get (0));

  // Activate an EPS bearer on all UEs
  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      Ptr<NetDevice> ueDevice = ueDevs.Get (u);
      GbrQosInformation qos;
      qos.gbrDl = (m_packetSize + 32) * (1000 / m_interval) * 8;  // bit/s, considering IP, UDP, RLC, PDCP header size
      qos.gbrUl = 0;
      qos.mbrDl = qos.gbrDl;
      qos.mbrUl = 0;
      
      enum EpsBearer::Qci q = EpsBearer::GBR_CONV_VOICE;
      EpsBearer bearer (q, qos);
      lteHelper->ActivateDedicatedEpsBearer (ueDevice, bearer, EpcTft::Default ());
    }

  // Install downlink and uplink applications
  uint16_t dlPort = 1234;
  uint16_t ulPort = 2000;
  PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
  ApplicationContainer clientApps;
  ApplicationContainer serverApps;

  for (uint32_t u = 0; u < ueNodes.GetN (); ++u)
    {
      ++ulPort;
      PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
      serverApps.Add (ulPacketSinkHelper.Install (remoteHost));  // receive packets from UEs
      serverApps.Add (dlPacketSinkHelper.Install (ueNodes.Get (u))); // receive packets from remotehost

      UdpClientHelper dlClient (ueIpIface.GetAddress (u), dlPort); // uplink packets generator
      dlClient.SetAttribute ("Interval", TimeValue (MilliSeconds (m_interval)));
      dlClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
      dlClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

      UdpClientHelper ulClient (remoteHostAddr, ulPort);           // downlink packets generator
      ulClient.SetAttribute ("Interval", TimeValue (MilliSeconds (m_interval)));
      ulClient.SetAttribute ("MaxPackets", UintegerValue (1000000));
      ulClient.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

      clientApps.Add (dlClient.Install (remoteHost));
      clientApps.Add (ulClient.Install (ueNodes.Get (u)));
    }

  serverApps.Start (Seconds (0.001));
  clientApps.Start (Seconds (0.001));

  double statsStartTime = 0.300; // need to allow for RRC connection establishment + SRS
  double statsDuration = 0.4;
  Simulator::Stop (Seconds (statsStartTime + statsDuration - 0.0001));

  lteHelper->EnableRlcTraces ();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("StartTime", TimeValue (Seconds (statsStartTime)));
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (statsDuration)));

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::LteEnbNetDevice/ComponentCarrierMap/*/LteEnbMac/DlScheduling",
                   MakeCallback (&NotifyDlScheduling));

  Simulator::Run ();

  /**
   * Check that the assignation is done in a RR fashion
   */
  NS_LOG_INFO ("DL - Test with " << m_nUser << " user(s) at distance " << m_dist);
  std::vector <uint64_t> dlDataRxed;
  if (m_errorModelEnabled)
    {
      m_thrRefDl *= 0.95; // for couting the Vienna AMC behavior: BLER between 0% and 10%
    }
  for (int i = 0; i < m_nUser; i++)
    {
      // get the imsi
      uint64_t imsi = ueDevs.Get (i)->GetObject<LteUeNetDevice> ()->GetImsi ();
      uint8_t lcId = 3;
      dlDataRxed.push_back (rlcStats->GetDlRxData (imsi, lcId));
      NS_LOG_INFO ("\tUser " << i << " imsi " << imsi << " lcid " << (uint16_t) lcId << " bytes rxed " << (double)dlDataRxed.at (i) << "  thr " << (double)dlDataRxed.at (i) / statsDuration << " ref " << m_thrRefDl);
//      NS_TEST_ASSERT_MSG_EQ_TOL ((double)dlDataRxed.at (i) / statsDuration, m_thrRefDl, m_thrRefDl * tolerance, " Unfair Throughput!");
    }

  NS_LOG_INFO ("UL - Test with " << m_nUser << " user(s) at distance " << m_dist);
  std::vector <uint64_t> ulDataRxed;
  if (m_errorModelEnabled)
    {
      m_thrRefUl *= 0.95; // for couting the Vienna AMC behavior: BLER between 0% and 10%
    }
  for (int i = 0; i < m_nUser; i++)
    {
      // get the imsi
      uint64_t imsi = ueDevs.Get (i)->GetObject<LteUeNetDevice> ()->GetImsi ();
      // get the lcId
      uint8_t lcId = 3;
      ulDataRxed.push_back (rlcStats->GetUlRxData (imsi, lcId));
      NS_LOG_INFO ("\tUser " << i << " imsi " << imsi << " lcid " << (uint16_t) lcId << " bytes rxed " << (double)ulDataRxed.at (i) << "  thr " << (double)ulDataRxed.at (i) / statsDuration << " ref " << m_thrRefUl << " txed " << rlcStats->GetUlTxData (imsi, lcId) / statsDuration);
//    NS_TEST_ASSERT_MSG_EQ_TOL ((double)ulDataRxed.at (i) / statsDuration, m_thrRefUl, m_thrRefUl * tolerance, " Unfair Throughput!");
    }

  Simulator::Destroy ();
}
