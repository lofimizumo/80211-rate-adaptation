/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2004,2005 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Modify: Xun Deng <dorence@hust.edu.cn> 
 *         Hao Yin <haoyin@uw.edu>env
 */

#include "rl-env.h"

#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-tx-vector.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-utils.h"
#include "ns3/node.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac-header.h"
#include <ctime>
#include "ns3/config.h"
#include <string>


#define Min(a,b) ((a < b) ? a : b)
#define Max(a,b) ((a > b) ? a : b) 

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("RLRateEnv");

NS_OBJECT_ENSURE_REGISTERED (RLRateEnv);

TypeId
RLRateEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::rl-rateWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RLRateEnv> ()
    .AddAttribute ("DataMode", "The transmission mode to use for every data packet transmission",
                   StringValue ("OfdmRate6Mbps"),
                   MakeWifiModeAccessor (&RLRateEnv::m_dataMode),
                   MakeWifiModeChecker ())
    .AddAttribute ("ControlMode", "The transmission mode to use for every RTS packet transmission.",
                   StringValue ("OfdmRate6Mbps"),
                   MakeWifiModeAccessor (&RLRateEnv::m_ctlMode),
                   MakeWifiModeChecker ())
  ;
  return tid;
}

RLRateEnv::RLRateEnv (uint16_t id)
{
  m_ns3ai_mod = new Ns3AIRL<AiConstantRateEnv, AiConstantRateAct> (id);
  m_ns3ai_mod->SetCond (2, 0);
  NS_LOG_FUNCTION (this);
  // Simulator::Schedule (m_timeStep, &RLRateEnv::MetreRead, this);
}

RLRateEnv::~RLRateEnv ()
{
  delete m_ns3ai_mod;
  NS_LOG_FUNCTION (this);
}

WifiRemoteStation *
RLRateEnv::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);
  WifiRemoteStation *station = new WifiRemoteStation ();
  return station;
}

void
RLRateEnv::DoInitialize()
{
    BuildSnrThresholds();
}

double 
RLRateEnv::CalculateFER(const std::unordered_set<int>& A, const std::unordered_set<int>& B) {
    // Convert unordered_sets to vectors
    std::vector<int> vA(A.begin(), A.end());
    std::vector<int> vB(B.begin(), B.end());

    // Sort the vectors
    std::sort(vA.begin(), vA.end());
    std::sort(vB.begin(), vB.end());

    // Find intersection
    std::vector<int> vC;
    std::set_intersection(vA.begin(), vA.end(), vB.begin(), vB.end(), std::back_inserter(vC));

    // Convert vector to unordered_set
    std::unordered_set<int> C(vC.begin(), vC.end());

    return 1-float(Max(A.size(), B.size()) - C.size())/float(Max(A.size(), B.size()));
}

void 
RLRateEnv::TraceTxOk (Ptr<const Packet> packet)
{
  // std::cout<<"Packet Sent: "<<m_txTotal_ap<<std::endl;
  // MacAddress 02 for AP, 01 for STA
  // Packet goes from 02 to 01
  if (m_wifiMacAddress == Mac48Address("00:00:00:00:00:02"))
  {
    packetsSentPerStep.insert(packet->GetUid());
    m_txPerStep++;
    m_txTotal_ap++;
    m_txPerMetreRead_sta1++;
    if (m_txPerStep == 20)
    {
      // std::cout<<"Sent: "<<m_txPerStep<<" Sent(Phy):"<<m_txPhyPerStep<<" Received: "<<m_rxPerStep<<" Lost: "<<m_txPerStep-m_rxPerStep<<" FER: "<<(double)(m_txPerStep-m_rxPerStep)/m_txPerStep<<std::endl;

      // We reset the counter every 600 packets
      Time cur_time = Simulator::Now();
      m_throughput_sta1 = (m_rxPerMetreRead_sta1 *1420 * 8.0) / (1024*1024)/(cur_time.GetSeconds() - m_startTime.GetSeconds());
      // m_fer = 1-((double)m_rxPerStep / (double)m_txPerStep);
      // std::cout << "Throughput(NS3): \t" << m_throughput_sta1 << " Mbps "<<" Total Packets:" << m_rxPerMetreRead_sta1 <<" Sent:"<<m_txPerStep<<" Received"<<m_rxPerStep<<"snr: "<<m_snr<< std::endl;
      m_rxPerMetreRead_sta1 = 0;
      m_txPerMetreRead_sta1 = 0;
      m_startTime = Simulator::Now ();
      m_txPerStep = 0;
      m_rxPerStep = 0;
      m_txPhyPerStep = 0;
      // std::cout<<"size of packetsSentPerStep: "<<packetsSentPerStep.size()<<"size of packetsReceivedPerStep: "<<packetsReceivedPerStep.size()<<std::endl;
      // m_fer = CalculateFER(packetsSentPerStep, packetsReceivedPerStep);
      packetsSentPerStep.clear();
      packetsReceivedPerStep.clear();
      m_readyToUpdate = true;
    }
  } 
}

void
RLRateEnv::TrackRxOk (Ptr<const Packet> packet)
{
  // std::cout<<"Rx Ok from"<<GetMac()->GetAddress()<<std::endl;
  if (m_wifiMacAddress == Mac48Address("00:00:00:00:00:02"))
  {
    packetsReceivedPerStep.insert(packet->GetUid());
    m_rxTotal_ap++;
    // std::cout<<"m_rxTotal_ap: "<<m_rxTotal_ap<<"m_txTotal_ap: "<<m_txTotal_ap<<std::endl;
    m_rxPerMetreRead_sta1++;
    m_rxPerStep++;
  } 
}

void
RLRateEnv::TrackPhyTxOk (Ptr<const Packet> packet, double txPowerDbm)
{
  // std::cout<<"Phy Tx Ok from"<<GetMac()->GetAddress()<<std::endl;
  if (m_wifiMacAddress == Mac48Address("00:00:00:00:00:02"))
  {
    m_txPhyPerStep++;
  } 
}
// void
// RLRateEnv::TrackTimeout (uint8_t reason, Ptr<const WifiPsdu> psdu, const WifiTxVector &txVector)
// {
//   std::cout<<"Timeout called"<<std::endl;
// }
void
RLRateEnv::TrackCw (uint32_t cw, uint8_t slot)
{
  if (cw != 15)
  {
    m_cw = cw;
    // std::cout<<"Cw: "<<cw<<"slot:"<<slot<<std::endl;
  }
}

bool isMacTxTraced = false;
bool isMacRxTraced = false;
bool isPhyTxTraced = false;
bool isCwTraced = false;
bool isMpduTraced = false;
// bool isMpduTraced = false;
void
RLRateEnv::DoReportRxOk (WifiRemoteStation *station,
                                       double rxSnr, WifiMode txMode)
{
  //start tracing when the AP receives the first RxOk
  m_snr = rxSnr;
  if (GetMac()->GetAddress() == Mac48Address("00:00:00:00:00:02"))
  {
    // if(!isMpduTraced)
    // {
    //   Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/PsduResponseTimeout", MakeCallback (&RLRateEnv::TrackTimeout, this));
    //   std::cout<<"create mpdutx trace at mac"<<GetMac()->GetAddress()<<std::endl;
    //   isMpduTraced = true;
    // }
    if (!isCwTraced)
    {
      std::string txop = GetMac()->GetQosSupported()
            ? "BE_Txop"
            : "Txop";
      Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::WifiMac/" + txop + "/CwTrace", MakeCallback (&RLRateEnv::TrackCw, this));
      std::cout<<"create cw trace at mac"<<GetMac()->GetAddress()<<std::endl;
      isCwTraced = true;
    }
    
    if (!isMacTxTraced)
    {
      // auto phy = GetPhy();
      // phy->TraceConnectWithoutContext ("PhyTxBegin", MakeCallback (&RLRateEnv::TrackPhyTxOk, this));
      Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacTx", MakeCallback (&RLRateEnv::TraceTxOk, this));
      std::cout<<"create phytx trace at mac"<<GetMac()->GetAddress()<<std::endl;
      isMacTxTraced = true;
    }
    if (!isMacRxTraced)
    {
      // auto mac = GetMac();
      // mac->TraceConnectWithoutContext ("MacRx", MakeCallback (&RLRateEnv::TrackRxOk, this));
      Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback (&RLRateEnv::TrackRxOk, this));
      std::cout<<"create macrx trace at mac"<<GetMac()->GetAddress()<<std::endl;
      isMacRxTraced = true;
    }
    if (!isPhyTxTraced)
    {
      // auto phy = GetPhy();
      // phy->TraceConnectWithoutContext ("PhyTxBegin", MakeCallback (&RLRateEnv::TrackPhyTxOk, this));
      Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin", MakeCallback (&RLRateEnv::TrackPhyTxOk, this));
      std::cout<<"create phytx trace at mac"<<GetMac()->GetAddress()<<std::endl;
      isPhyTxTraced = true;
    }
  }
  
  NS_LOG_FUNCTION (this << station << rxSnr << txMode);
}

void
RLRateEnv::DoReportRtsFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}

void
RLRateEnv::DoReportDataFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  m_txFail_ap++;
  // std::cout<<"packet failed: "<<m_txFail_ap<<"packet sent: "<<m_txTotal_ap<<std::endl;
}

void
RLRateEnv::DoReportRtsOk (WifiRemoteStation *st,
                                        double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << st << ctsSnr << ctsMode << rtsSnr);
}

void
RLRateEnv::DoReportDataOk (WifiRemoteStation *st, double ackSnr, WifiMode ackMode,
                                         double dataSnr, uint16_t dataChannelWidth, uint8_t dataNss)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode << dataSnr << dataChannelWidth << +dataNss);
}

void
RLRateEnv::DoReportFinalRtsFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}

void
RLRateEnv::DoReportFinalDataFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}

void
RLRateEnv::BuildSnrThresholds()
{
    m_thresholds.clear();
    WifiMode mode;
    WifiTxVector txVector;
    uint8_t nss = 1;
    for (const auto& mode : GetPhy()->GetModeList())
    {
        txVector.SetChannelWidth(20);
        txVector.SetNss(nss);
        txVector.SetMode(mode);
        NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName());
        AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
    }
    // Add all MCSes
    if (GetHtSupported())
    {
        for (const auto& mode : GetPhy()->GetMcsList())
        {
            for (uint16_t j = 20; j <= GetPhy()->GetChannelWidth(); j *= 2)
            {
                txVector.SetChannelWidth(j);
                if (mode.GetModulationClass() == WIFI_MOD_CLASS_HT)
                {
                    uint16_t guardInterval = GetShortGuardIntervalSupported() ? 400 : 800;
                    txVector.SetGuardInterval(guardInterval);
                    // derive NSS from the MCS index
                    nss = (mode.GetMcsValue() / 8) + 1;
                    NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName() << " channel width " << j
                                                  << " nss " << +nss << " GI " << guardInterval);
                    txVector.SetNss(nss);
                    txVector.SetMode(mode);
                    AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
                }
                else // VHT or HE
                {
                    uint16_t guardInterval;
                    if (mode.GetModulationClass() == WIFI_MOD_CLASS_VHT)
                    {
                        guardInterval = GetShortGuardIntervalSupported() ? 400 : 800;
                    }
                    else
                    {
                        guardInterval = GetGuardInterval();
                    }
                    txVector.SetGuardInterval(guardInterval);
                    for (uint8_t k = 1; k <= GetPhy()->GetMaxSupportedTxSpatialStreams(); k++)
                    {
                        if (mode.IsAllowed(j, k))
                        {
                            NS_LOG_DEBUG("Adding mode = " << mode.GetUniqueName()
                                                          << " channel width " << j << " nss " << +k
                                                          << " GI " << guardInterval);
                            txVector.SetNss(k);
                            txVector.SetMode(mode);
                            AddSnrThreshold(txVector, GetPhy()->CalculateSnr(txVector, m_ber));
                        }
                        else
                        {
                            NS_LOG_DEBUG("Mode = " << mode.GetUniqueName() << " disallowed");
                        }
                    }
                }
            }
        }
    }
}

void
RLRateEnv::AddSnrThreshold(WifiTxVector txVector, double snr)
{
    m_thresholds.emplace_back(snr, txVector);
}

double
RLRateEnv::GetSnrThreshold (WifiTxVector txVector) const
{
    auto it = std::find_if(m_thresholds.begin(),
                           m_thresholds.end(),
                           [&txVector](const std::pair<double, WifiTxVector>& p) -> bool {
                               return ((txVector.GetMode() == p.second.GetMode()) &&
                                       (txVector.GetNss() == p.second.GetNss()) &&
                                       (txVector.GetChannelWidth() == p.second.GetChannelWidth()));
                           });
    // if (it == m_thresholds.end())
    // {
    //     // This means capabilities have changed in runtime, hence rebuild SNR thresholds
    //     BuildSnrThresholds();
    //     it = std::find_if(m_thresholds.begin(),
    //                       m_thresholds.end(),
    //                       [&txVector](const std::pair<double, WifiTxVector>& p) -> bool {
    //                           return ((txVector.GetMode() == p.second.GetMode()) &&
    //                                   (txVector.GetNss() == p.second.GetNss()) &&
    //                                   (txVector.GetChannelWidth() == p.second.GetChannelWidth()));
    //                       });
    //     NS_ASSERT_MSG(it != m_thresholds.end(), "SNR threshold not found");
    // }
    return it->first;
}

WifiMode
RLRateEnv::DoGetDataMode (WifiRemoteStation *st, uint32_t size)
{
  std::cout<<"DoGetDataMode"<<"packet sent: "<<m_txTotal_ap<<"failed: "<<m_txFail_ap<<std::endl;
  return m_dataMode;
}

bool isMacSetup = false;
WifiTxVector
RLRateEnv::DoGetDataTxVector (WifiRemoteStation *st, uint16_t allowedWidth)
{
  // std::cout<<"DoGetDataTxVector"<<"packet sent: "<<m_txTotal_ap<<"failed: "<<m_txFail_ap<<std::endl;
  NS_LOG_FUNCTION (this << st);
  m_wifiMacAddress = GetMac()->GetAddress ();
  
  // // set input
  if (m_readyToUpdate)
  {
      if (m_wifiMacAddress == Mac48Address("00:00:00:00:00:01"))
      {
        std::cout<<"setting env for mac:"<<m_wifiMacAddress<<std::endl;
      }
    // std::cout<<"setting env for mac:"<<m_wifiMacAddress<<std::endl;
    double maxThreshold = 0.0;
    WifiMode maxMode = GetDefaultModeForSta (st);
    WifiTxVector txVector;
    txVector.SetChannelWidth (20);
    for (uint32_t i = 0; i < GetNMcsSupported(st); i++)
      {
        WifiMode mode = GetMcsSupported (st, i);
        txVector.SetMode (mode);
        double threshold = GetSnrThreshold (txVector);
        if (threshold > maxThreshold
            && threshold < m_snr)
          {
            maxThreshold = threshold;
            maxMode = mode;
          }
      }
    // uncomment to specify arbitrary MCS
    // std::cout<<"next_mcs: "<<(int)next_mcs<<std::endl;
    int8_t max_MCS = maxMode.GetMcsValue ();

    auto env = m_ns3ai_mod->EnvSetterCond ();
    // std::cout<<"env setting finished"<<std::endl;
    if (m_dataMode.GetModulationClass () == WIFI_MOD_CLASS_HT)
      env->mcs = m_dataMode.GetMcsValue ();
    // else
      // env->mcs = 0xffu;
    env->cw = m_cw;
    env->max_mcs = max_MCS;
    env->throughput = m_throughput_sta1;
    env->snr = m_snr;
    m_ns3ai_mod->SetCompleted ();
    

    // std::cout<<"Waiting for next MCS from RL-Agol"<<m_wifiMacAddress<<std::endl;
    // get output
    auto act = m_ns3ai_mod->ActionGetterCond ();
    // m_nss = act->nss;
    m_next_mcs = act->next_mcs;
    // std::cout<<"next_mcs: "<<(int)m_next_mcs<<std::endl;
    m_ns3ai_mod->GetCompleted ();
    m_readyToUpdate = false;

    
    // std::cout<<"max_MCS: "<<(int)max_MCS<<"\tnext_mcs: "<<(int)Min(max_MCS, m_next_mcs)<<std::endl;
    // m_dataMode = GetMcsSupported (st, Min(max_MCS, m_next_mcs));
    if (m_snr<5.5)
    {
      m_next_mcs = 0;
      // std::cout<<"triggered"<<std::endl;
    }
    m_dataMode = GetMcsSupported (st, m_next_mcs);
  }
  return WifiTxVector (
      m_dataMode,
      GetDefaultTxPowerLevel (),
      GetPreambleForTransmission (
          m_dataMode.GetModulationClass (),
          GetShortPreambleEnabled ()),
      ConvertGuardIntervalToNanoSeconds (
          m_dataMode,
          GetShortGuardIntervalSupported (st),
          NanoSeconds (GetGuardInterval (st))),
      GetNumberOfAntennas (),
      m_nss,
      0,
      GetPhy()->GetTxBandwidth(
          m_dataMode,
          std::min(allowedWidth, GetChannelWidth(st))),
      GetAggregation (st));
}

WifiTxVector
RLRateEnv::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  return WifiTxVector (
      m_ctlMode,
      GetDefaultTxPowerLevel (),
      GetPreambleForTransmission (
          m_ctlMode.GetModulationClass (),
          GetShortPreambleEnabled ()),
      800,
      1,
      1,
      0,
      GetPhy()->GetTxBandwidth(
          m_dataMode,
          GetChannelWidth(st)),
      GetAggregation (st));
}

void RLRateEnv::MetreRead(void)
{
  // else if (macAddress == Mac48Address("00:00:00:00:00:02"))
  // {
  //   Time cur_time = Simulator::Now();
  //   m_throughput_sta5 = (m_bytesTotal_sta5 *1456 * 8.0) / (1024*1024)/(cur_time.GetSeconds() - m_startTime.GetSeconds());
  //   std::cout << "Throughput(NS3): \t" << m_throughput_sta5 << " Mbps" << "during: " << cur_time.GetSeconds() - m_startTime.GetSeconds() << "s" <<"Total Packets:" << m_bytesTotal_sta5 <<"mac address: "<<macAddress<< std::endl;
  //   m_bytesTotal_sta5 = 0;
  //   if (m_txTotal != 0)
  //   {
  //     // std::cout << "PER: \t" << (float)m_rxTotal / (float)m_txTotal << std::endl;
  //   }
  // }
  // else if (macAddress == Mac48Address("00:00:00:00:00:03"))
  // {
  //   Time cur_time = Simulator::Now();
  //   m_throughput_sta5 = (m_bytesTotal_sta5 *1456 * 8.0) / (1024*1024)/(cur_time.GetSeconds() - m_startTime.GetSeconds());
  //   std::cout << "Throughput(NS3): \t" << m_throughput_sta5 << " Mbps" << "during: " << cur_time.GetSeconds() - m_startTime.GetSeconds() << "s" <<"Total Packets:" << m_bytesTotal_sta5 <<"mac address: "<<macAddress<< std::endl;
  //   m_bytesTotal_sta5 = 0;
  //   if (m_txTotal != 0)
  //   {
  //     // std::cout << "PER: \t" << (float)m_rxTotal / (float)m_txTotal << std::endl;
  //   }
  // }
  // else if (macAddress == Mac48Address("00:00:00:00:00:04"))
  // {
  //   Time cur_time = Simulator::Now();
  //   m_throughput_sta8 = (m_bytesTotal_sta8 *1456 * 8.0) / (1024*1024)/(cur_time.GetSeconds() - m_startTime.GetSeconds());
  //   std::cout << "Throughput(NS3): \t" << m_throughput_sta8 << " Mbps" << "during: " << cur_time.GetSeconds() - m_startTime.GetSeconds() << "s" <<"Total Packets:" << m_bytesTotal_sta5 <<"mac address: "<<macAddress<< std::endl;
  //   m_bytesTotal_sta8 = 0;
  //   if (m_txTotal != 0)
  //   {
  //     // std::cout << "PER: \t" << (float)m_rxTotal / (float)m_txTotal << std::endl;
  //   }
  // }
  // Simulator::Schedule(m_timeStep, &RLRateEnv::MetreRead, this);
}


} //namespace ns3