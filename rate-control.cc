/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
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
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

/**
 * This example program is designed to illustrate the behavior of
 * rate-adaptive WiFi rate controls such as Minstrel.  Power-adaptive
 * rate controls can be illustrated also, but separate examples exist for
 * highlighting the power adaptation.
 *
 * This simulation consist of 2 nodes, one AP and one STA.
 * The AP generates UDP traffic with a CBR of 54 Mbps to the STA.
 * The AP can use any power and rate control mechanism and the STA uses
 * only Minstrel rate control.
 * The STA can be configured to move away from (or towards to) the AP.
 * By default, the AP is at coordinate (0,0,0) and the STA starts at
 * coordinate (5,0,0) (meters) and moves away on the x axis by 1 meter every
 * second.
 *
 * The output consists of:
 * - A plot of average throughput vs. distance.
 * - (if logging is enabled) the changes of rate to standard output.
 *
 * Example usage:
 * ./ns3 run "wifi-rate-adaptation-distance --standard=802.11a --staManager=ns3::MinstrelWifiManager
 * --apManager=ns3::MinstrelWifiManager --outputFileName=minstrel"
 *
 * Another example (moving towards the AP):
 * ./ns3 run "wifi-rate-adaptation-distance --standard=802.11a --staManager=ns3::MinstrelWifiManager
 * --apManager=ns3::MinstrelWifiManager --outputFileName=minstrel --stepsSize=1 --STA1_x=-200"
 *
 * Example for HT rates with SGI and channel width of 40MHz:
 * ./ns3 run "wifi-rate-adaptation-distance --staManager=ns3::MinstrelHtWifiManager
 * --apManager=ns3::MinstrelHtWifiManager --outputFileName=minstrelHt --shortGuardInterval=true
 * --channelWidth=40"
 *
 * To enable the log of rate changes:
 * export NS_LOG=RateAdaptationDistance=level_info
 */

#include "ns3/boolean.h"
#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/gnuplot.h"
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/propagation-loss-model.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RateAdaptationDistance");

/** Node statistics */
class NodeStatistics
{
  public:
    /**
     * Constructor
     * \param aps AP devices
     * \param stas STA devices
     */
    NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas);

    /**
     * RX callback
     * \param path path
     * \param packet received packet
     * \param from sender
     */
    void RxCallback(std::string path, Ptr<const Packet> packet, const Address& from);
    /**
     * Set node position
     * \param node the node
     * \param position the position
     */
    void SetPosition(Ptr<Node> node, Vector position);
    /**
     * Advance node position
     * \param node the node
     * \param stepsSize the size of a step
     * \param stepsTime the time interval between steps
     */
    void AdvancePosition(Ptr<Node> node, double stepsSize, int stepsTime);
    /**
     * Get node position
     * \param node the node
     * \return the position
     */
    Vector GetPosition(Ptr<Node> node);
    /**
     * \return the gnuplot 2d dataset
     */
    Gnuplot2dDataset GetDatafile();

  private:
    uint32_t m_bytesTotal;     //!< total bytes
    Gnuplot2dDataset m_output; //!< gnuplot 2d dataset
};

NodeStatistics::NodeStatistics(NetDeviceContainer aps, NetDeviceContainer stas)
{
    m_bytesTotal = 0;
}

void
NodeStatistics::RxCallback(std::string path, Ptr<const Packet> packet, const Address& from)
{
    m_bytesTotal += packet->GetSize();
}

void
NodeStatistics::SetPosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    mobility->SetPosition(position);
}

Vector
NodeStatistics::GetPosition(Ptr<Node> node)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    return mobility->GetPosition();
}

void
NodeStatistics::AdvancePosition(Ptr<Node> node, double stepsSize, int stepsTime)
{
    Vector pos = GetPosition(node);
    double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
    m_bytesTotal = 0;
    m_output.Add(pos.x, mbs);
    pos.x += stepsSize;
    SetPosition(node, pos);
    Simulator::Schedule(Seconds(1*stepsTime),
                        &NodeStatistics::AdvancePosition,
                        this,
                        node,
                        stepsSize,
                        stepsTime);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile()
{
    return m_output;
}

/**
 * Callback for 'Rate' trace source
 *
 * \param oldRate old MCS rate (bits/sec)
 * \param newRate new MCS rate (bits/sec)
 */
void
RateCallback(uint64_t oldRate, uint64_t newRate)
{
    NS_LOG_INFO("Rate " << newRate / 1000000.0 << " Mbps");
    // std::cout<<"Rate " << newRate / 1000000.0 << " Mbps"<<std::endl;
}
struct DataForThpt
{
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;
  uint32_t totalRxPackets; //Total number of received packets in all flows
  uint64_t totalRxBytes; // Total bytes received in all flows
  double totalDelaySum; // Total delay sum in all flows
  // average delay (ms)
  double
  averageDelay ()
  {
    return totalRxPackets ? totalDelaySum / totalRxPackets / 100000000 : 0;
  }
} data; //data is a structure variable which will store all these global variables.

double duration = 1.0; // Duration of simulation (s)
double statInterval = 0.5; // Time interval of calling function Throughput

double InitDistance = 2;
static void
Throughput ()
{
  data.monitor->CheckForLostPackets ();
  const FlowMonitor::FlowStatsContainer stats = data.monitor->GetFlowStats ();

  uint64_t totalRxBytes = 0;
  uint32_t totalRxPackets = 0;
  double totalDelaySum = 0;

  // Iterating through every flow
  for (FlowMonitor::FlowStatsContainerCI iter = stats.begin (); iter != stats.end (); iter++)
    {
      totalRxBytes += iter->second.rxBytes;
      totalDelaySum += iter->second.delaySum.GetDouble ();
      totalRxPackets += iter->second.rxPackets;
    }
  uint64_t rxBytesDiff = totalRxBytes - data.totalRxBytes;
  uint32_t rxPacketsDiff = totalRxPackets - data.totalRxPackets;
  double delayDiff = totalDelaySum - data.totalDelaySum;

  data.totalRxBytes = totalRxBytes;
  data.totalRxPackets = totalRxPackets;
  data.totalDelaySum = totalDelaySum;

  double delay = 0.0; // ms
  if (rxPacketsDiff != 0 && delayDiff != 0)
    {
      delay = delayDiff / rxPacketsDiff / 1000000;
    }
  double tpt = 8.0 * rxBytesDiff / statInterval / (1024 * 1024); // Mbps
  InitDistance += 0.25;
  std::cout <<"Throughput: " << tpt << "Mbps" << "\tDistance between Ap and Sta:"<<InitDistance<<std::endl;
  Simulator::Schedule (Seconds (statInterval), &Throughput);
}

int
main(int argc, char* argv[])
{
    uint32_t rtsThreshold = 65535;
    std::string staManager = "ns3::rl-rateWifiManager";
    std::string apManager = "ns3::rl-rateWifiManager";
    std::string standard = "802.11n-5GHz";
    std::string outputFileName = "minstrelHT";
    uint32_t BeMaxAmpduSize = 65535;//Disable the A-MPDU
    bool shortGuardInterval = false;
    uint32_t chWidth = 20;
    int ap1_x = 0;
    int ap1_y = 0;
    int sta1_x = 5;
    int sta1_y = 0;
    int steps = 500;
    double stepsSize = 0.5;
    int stepsTime = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("staManager", "Rate adaptation manager of the STA", staManager);
    cmd.AddValue("apManager", "Rate adaptation manager of the AP", apManager);
    cmd.AddValue("standard", "Wifi standard (a/b/g/n/ac only)", standard);
    cmd.AddValue("shortGuardInterval",
                 "Enable Short Guard Interval in all stations",
                 shortGuardInterval);
    cmd.AddValue("channelWidth", "Channel width of all the stations", chWidth);
    cmd.AddValue("rtsThreshold", "RTS threshold", rtsThreshold);
    cmd.AddValue("BeMaxAmpduSize", "BE Mac A-MPDU size", BeMaxAmpduSize);
    cmd.AddValue("outputFileName", "Output filename", outputFileName);
    cmd.AddValue("steps", "How many different distances to try", steps);
    cmd.AddValue("stepsTime", "Time on each step", stepsTime);
    cmd.AddValue("stepsSize", "Distance between steps", stepsSize);
    cmd.AddValue("AP1_x", "Position of AP1 in x coordinate", ap1_x);
    cmd.AddValue("AP1_y", "Position of AP1 in y coordinate", ap1_y);
    cmd.AddValue("STA1_x", "Position of STA1 in x coordinate", sta1_x);
    cmd.AddValue("STA1_y", "Position of STA1 in y coordinate", sta1_y);
    cmd.Parse(argc, argv);

    int simuTime = steps * stepsTime;

    if (standard != "802.11a" && standard != "802.11b" && standard != "802.11g" &&
        standard == "802.11n-2.4GHz" && standard != "802.11n-5GHz" && standard != "802.11ac")
    {
        NS_FATAL_ERROR("Standard " << standard << " is not supported by this program");
    }

    // Define the APs
    // Use Sta-Sta Instead
    NodeContainer wifiApNodes;
    wifiApNodes.Create(2);

    // Define the STAs
    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(1);

    std::string errorModelType = "ns3::NistErrorRateModel"; // Error Model
    YansWifiPhyHelper wifiPhy;
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();

    // Set Propagation Loss Model
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::MatrixPropagationLossModel","DefaultLoss",DoubleValue(30.0));

    // Set Transmission Power
    wifiPhy.Set ("TxPowerStart", DoubleValue (20.0));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (20.0));

    // Create a loss model and set default loss
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.SetErrorRateModel (errorModelType);

    // Channel configuration via ChannelSettings attribute can be performed here
    std::string frequencyBand;
    if (standard == "802.11b" || standard == "802.11g" || standard == "802.11n-2.4GHz")
    {
        frequencyBand = "BAND_2_4GHZ";
    }
    else
    {
        frequencyBand = "BAND_5GHZ";
    }
    wifiPhy.Set("ChannelSettings",
                StringValue("{0, " + std::to_string(chWidth) + ", " + frequencyBand + ", 0}"));

    // By default, the CCA sensitivity is -82 dBm, meaning if the RSS is
    // below this value, the receiver will reject the Wi-Fi frame.
    // However, we want to allow the rate adaptation to work down to low
    // SNR values.  To allow this, we need to do three things:  1) disable
    // the noise figure (set it to 0 dB) so that the noise level in 20 MHz
    // is around -101 dBm, 2) lower the CCA sensitivity to a value that
    // disables it (e.g. -110 dBm), and 3) disable the Wi-Fi preamble
    // detection model.
    wifiPhy.Set("CcaSensitivity", DoubleValue(-110));
    wifiPhy.Set("RxNoiseFigure", DoubleValue(0));
    wifiPhy.DisablePreambleDetectionModel();

    NetDeviceContainer wifiApDevices;
    NetDeviceContainer wifiStaDevices;
    NetDeviceContainer wifiDevices;

    WifiHelper wifi;
    if (standard == "802.11a" || standard == "802.11b" || standard == "802.11g")
    {
        if (standard == "802.11a")
        {
            wifi.SetStandard(WIFI_STANDARD_80211a);
        }
        else if (standard == "802.11b")
        {
            wifi.SetStandard(WIFI_STANDARD_80211b);
        }
        else if (standard == "802.11g")
        {
            wifi.SetStandard(WIFI_STANDARD_80211g);
        }
        WifiMacHelper wifiMac;

        // Configure the STA node
        wifi.SetRemoteStationManager(staManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
        // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        //                            "DataMode", StringValue("HtMcs1"), // Use MCS 1 for data
        //                            "ControlMode", StringValue("HtMcs1"),
        //                            "RtsCtsThreshold", UintegerValue(rtsThreshold));

        Ssid ssid = Ssid("AP");
        wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        wifiStaDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes));

        // Configure the AP node
        wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
        // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        //                            "DataMode", StringValue("HtMcs1"), // Use MCS 1 for data
        //                            "ControlMode", StringValue("HtMcs1"),
        //                            "RtsCtsThreshold", UintegerValue(rtsThreshold));

        ssid = Ssid("AP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes));
    }
    else if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz" || standard == "802.11ac")
    {
        if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz")
        {
            wifi.SetStandard(WIFI_STANDARD_80211n);
        }
        else if (standard == "802.11ac")
        {
            wifi.SetStandard(WIFI_STANDARD_80211ac);
        }

        WifiMacHelper wifiMac;

        // Configure the STA node
        wifi.SetRemoteStationManager(staManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
        // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        //                            "DataMode", StringValue("HtMcs7"), // Use MCS 1 for data
        //                            "ControlMode", StringValue("HtMcs1"),
        //                            "RtsCtsThreshold", UintegerValue(rtsThreshold));

        Ssid ssid = Ssid("AP");
        wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));
        wifiStaDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiStaNodes));

        // Configure the AP node
        wifi.SetRemoteStationManager(apManager, "RtsCtsThreshold", UintegerValue(rtsThreshold));
        // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
        //                     "DataMode", StringValue("HtMcs7"), // Use MCS 1 for data
        //                     "ControlMode", StringValue("HtMcs1"),
        //                     "RtsCtsThreshold", UintegerValue(rtsThreshold));

        ssid = Ssid("AP");
        wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        wifiApDevices.Add(wifi.Install(wifiPhy, wifiMac, wifiApNodes));

        Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize",
                    UintegerValue(BeMaxAmpduSize));
    }

    wifiDevices.Add(wifiStaDevices);
    wifiDevices.Add(wifiApDevices);

    // Set guard interval
    Config::Set(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HtConfiguration/ShortGuardIntervalSupported",
        BooleanValue(shortGuardInterval));

    // Configure the mobility.
    // Initial position of AP and STA
    // positionAlloc->Add(Vector(ap1_x, ap1_y, 0.0));
    // positionAlloc->Add(Vector(sta1_x, sta1_y, 0.0));
    // mobility.SetPositionAllocator(positionAlloc);
    // positionAlloc->Add(Vector(0, 0, 0.0));
    // mobility.SetPositionAllocator(positionAlloc); 
    // Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    // positionAlloc->Add(Vector(0, 0, 0.0));
    // mobility.SetPositionAllocator(positionAlloc); 
    // Ptr<ListPositionAllocator> positionAllocSta = CreateObject<ListPositionAllocator> ();
    // positionAllocSta->Add(Vector(5, 0, 0.0));
    // mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // mobility.SetPositionAllocator(positionAllocSta); 
    MobilityHelper mobilitySta;
    mobilitySta.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (5.0),
                                 "DeltaY", DoubleValue (10.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));
    // mobilitySta.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
    //                             "Bounds", StringValue("0|24|0|1"),
    //                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=3.0]"));
    mobilitySta.Install (wifiStaNodes);

    MobilityHelper mobilityAp;
    mobilityAp.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (2.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (60.0),
                                 "DeltaY", DoubleValue (0.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));
    mobilityAp.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install (wifiApNodes);


    // Statistics counter
    NodeStatistics atpCounter = NodeStatistics(wifiApDevices, wifiStaDevices);

    // Move the STA by stepsSize meters every stepsTime seconds
    Simulator::Schedule(Seconds(0.5 + stepsTime),
                        &NodeStatistics::AdvancePosition,
                        &atpCounter,
                        wifiApNodes.Get(0),
                        stepsSize,
                        stepsTime);

    // Configure the IP stack
    InternetStackHelper stack;
    stack.Install(wifiApNodes);
    stack.Install(wifiStaNodes);
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = address.Assign(wifiStaDevices);
    Ipv4InterfaceContainer j = address.Assign(wifiApDevices);
    Ipv4Address sinkAddress = i.GetAddress(0);
    // Ipv4Address sinkAddress2 = i.GetAddress(3);
    // Ipv4Address sinkAddress3 = i.GetAddress(1);
    // std::cout<<"sinkAddress: "<<sinkAddress<<" sink2Address: "<<sinkAddress2<<"\n";
    uint16_t port = 9;
    uint16_t port2 = 10;
    uint16_t port3 = 12;
    uint16_t port4 = 14;

    // Configure the CBR generator
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    ApplicationContainer apps_sink = sink.Install(wifiStaNodes.Get(0));
    // PacketSinkHelper sink2("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port2));
    // ApplicationContainer apps_sink2 = sink.Install(wifiStaNodes.Get(3));
    // PacketSinkHelper sink3("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port3));
    // ApplicationContainer apps_sink3 = sink3.Install(wifiStaNodes.Get(1));

    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port));
    onoff.SetConstantRate(DataRate("100Mb/s"), 1420);
    onoff.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    onoff.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    ApplicationContainer apps_source = onoff.Install(wifiApNodes.Get(0));

    OnOffHelper onoff2("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port2));
    onoff2.SetConstantRate(DataRate("400Mb/s"), 1420);
    onoff2.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    onoff2.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    ApplicationContainer apps_source2 = onoff2.Install(wifiApNodes.Get(1));

    // OnOffHelper onoff3("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port3));
    // onoff3.SetConstantRate(DataRate("400Mb/s"), 1420);
    // onoff3.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    // onoff3.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    // ApplicationContainer apps_source3 = onoff3.Install(wifiApNodes.Get(2));

    // OnOffHelper onoff4("ns3::UdpSocketFactory", InetSocketAddress(sinkAddress, port4));
    // onoff4.SetConstantRate(DataRate("400Mb/s"), 1420);
    // onoff4.SetAttribute("StartTime", TimeValue(Seconds(0.5)));
    // onoff4.SetAttribute("StopTime", TimeValue(Seconds(simuTime)));
    // ApplicationContainer apps_source4 = onoff4.Install(wifiApNodes.Get(3));
    apps_sink.Start(Seconds(0.5));
    apps_sink.Stop(Seconds(simuTime));


    //------------------------------------------------------------
    //-- Setup stats and data collection
    //--------------------------------------------

    // // Register packet receptions to calculate throughput
    // Config::Connect("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
    //                 MakeCallback(&NodeStatistics::RxCallback, &atpCounter));

    // Callbacks to print every change of rate
    Config::ConnectWithoutContextFailSafe(
        "/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + apManager + "/Rate",
        MakeCallback(RateCallback));

    data.monitor = data.flowmon.InstallAll ();
    data.totalDelaySum = 0;
    data.totalRxBytes = 0;
    data.totalRxPackets = 0;
    Simulator::Schedule (Seconds (0), &Throughput);
    Simulator::Stop(Seconds(simuTime));
    Simulator::Run();
    Ptr<PacketSink> sink1 = DynamicCast<PacketSink> (apps_sink.Get (0));
    std::cout << "Average Delay: " << data.averageDelay () << "ms" << std::endl;
    std::cout << "Average Throughput: " << sink1->GetTotalRx () * 8.0 / simuTime / (1024 * 1024)
        << "Mbps" << std::endl;
    Simulator::Destroy();

    return 0;
}