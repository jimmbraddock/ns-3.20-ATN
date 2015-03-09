#include "ns3/olsr-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/atn-helper.h"
#include "ns3/v4ping-helper.h"
#include <iostream>
#include <cmath>
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"

using namespace ns3;

class AtnSimulate
{
public:
  AtnSimulate ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);

private:
  ///\name parameters
  //\{
  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;
  //\}

  ///\name network
  //\{
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  //\}

private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
};

int main (int argc, char **argv)
{
  AtnSimulate test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
AtnSimulate::AtnSimulate () :
  size (3),
  step (100),
  totalTime (10),
  pcap (true),
  printRoutes (true)
{
}

bool
AtnSimulate::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
  LogComponentEnable("V4Ping", LOG_LEVEL_ALL);
  LogComponentEnable("Atn", LOG_LEVEL_ALL);
  LogComponentEnable("OlsrRoutingProtocol", LOG_LEVEL_DEBUG);
  LogComponentEnable("YansWifiPhy", LOG_INFO);
  //LogComponentEnable("YansWifiChannel", LOG_INFO);

  //SeedManager::SetSeed (12345);
  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);

  cmd.Parse (argc, argv);
  return true;
}

void
AtnSimulate::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
AtnSimulate::Report (std::ostream &)
{
}

void
AtnSimulate::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), nodes.Get (i));
    }

  // На 120 метрах возможна связь. Пока это предел
//  Ptr<MobilityModel> m = CreateObject<ConstantPositionMobilityModel> ();
//  Ptr<MobilityModel> m2 = CreateObject<ConstantPositionMobilityModel> ();
//  m2->SetPosition (Vector (0, 600, 0));
//  nodes.Get (0)->AggregateObject (m);

//  nodes.Get (1)->AggregateObject (m2);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::GaussMarkovMobilityModel",
                            "Bounds", BoxValue (Box (0, 1500, 0, 1500, 0, 0)),
                            "TimeStep", TimeValue (Seconds (0.5)),
                            "Alpha", DoubleValue (0.85),
                            "MeanVelocity", StringValue ("ns3::UniformRandomVariable[Min=250|Max=600]"),
                            "MeanDirection", StringValue ("ns3::UniformRandomVariable[Min=0|Max=6.283185307]"),
                            "MeanPitch", StringValue ("ns3::UniformRandomVariable[Min=0.05|Max=0.05]"),
                            "NormalVelocity", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.0|Bound=0.0]"),
                            "NormalDirection", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.2|Bound=0.4]"),
                            "NormalPitch", StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=0.02|Bound=0.04]"));
  mobility.SetPositionAllocator("ns3::RandomBoxPositionAllocator",
                                "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=200]"),
                                "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=200]"),
                                "Z", StringValue ("ns3::UniformRandomVariable[Min=0|Max=0]"));
  mobility.Install (nodes);
}

void
AtnSimulate::CreateDevices ()
{
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;//= YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "ReferenceDistance", DoubleValue(30.0), "Exponent", DoubleValue(5.0));
  //wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel", "MaxRange", DoubleValue(300.0));
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.EnablePcapAll ("pcap");
  WifiHelper wifi = WifiHelper();

  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  std::string phyMode ("DsssRate1Mbps");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
//  wifiPhy.Set ("TxPowerStart",DoubleValue (7.5));
//  wifiPhy.Set ("TxPowerEnd", DoubleValue (7.5));

  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("atn-simulator"));
    }
}

void
AtnSimulate::InstallInternetStack ()
{
  OlsrHelper olsr;
  InternetStackHelper stack;
  stack.SetRoutingHelper (olsr); // has effect on the next Install ()
  stack.Install (nodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

//  Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("olsr.routes", std::ios::out);
//  olsr.PrintRoutingTableAllAt (Seconds (8), routingStream);
}

void
AtnSimulate::InstallApplications ()
{
  AtnHelper atn;
  ApplicationContainer app = atn.Install (nodes);
  app.Start (Seconds (0));
  app.Stop (Seconds (totalTime) - Seconds (0.001));


  Ptr<Node> appSource = NodeList::GetNode (0);
  Ptr<Node> appSink = NodeList::GetNode (2);
  // Let's fetch the IP address of the last node, which is on Ipv4Interface 1
  Ipv4Address remoteAddr = appSink->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();

  OnOffHelper onoff ("ns3::UdpSocketFactory",
                     Address (InetSocketAddress (remoteAddr, 4567)));
  onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
  ApplicationContainer app2 = onoff.Install (appSource);
  app2.Start (Seconds (3));
  app2.Stop (Seconds (7));

  // Create a packet sink to receive these packets
  PacketSinkHelper sink ("ns3::UdpSocketFactory",
                         InetSocketAddress (Ipv4Address::GetAny (), 4567));
  ApplicationContainer app3 = sink.Install (appSink);
  app3.Start (Seconds (3));
  app3.Stop (Seconds (totalTime) - Seconds (0.001));
}

