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
 *         Hao Yin <haoyin@uw.edu>
 */


#ifndef AI_CONSTANT_RATE_WIFI_MANAGER_H
#define AI_CONSTANT_RATE_WIFI_MANAGER_H

#include "ns3/wifi-remote-station-manager.h"
#include "ns3/ns3-ai-module.h"
#include <unordered_set>
#include <vector>
#include <algorithm>

namespace ns3 {

struct AiConstantRateEnv
{
  uint8_t mcs;
  uint8_t max_mcs;
  uint16_t cw;
  double throughput;
  double snr;
} Packed;

struct AiConstantRateAct
{
  uint8_t nss;
  uint8_t next_mcs;
} Packed;

/**
 * \ingroup wifi
 * \brief use constant rates for data and RTS transmissions
 *
 * This class uses always the same transmission rate for every
 * packet sent.
 */
class RLRateEnv : public WifiRemoteStationManager
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  RLRateEnv (uint16_t id = 2335);
  virtual ~RLRateEnv ();


private:
  WifiRemoteStation* DoCreateStation (void) const override;
  void DoReportRxOk (WifiRemoteStation *station,
                     double rxSnr, WifiMode txMode) override;
  void DoReportRtsFailed (WifiRemoteStation *station) override;
  void DoReportDataFailed (WifiRemoteStation *station) override;
  void DoReportRtsOk (WifiRemoteStation *station,
                      double ctsSnr, WifiMode ctsMode, double rtsSnr) override;
  void DoReportDataOk (WifiRemoteStation *station, double ackSnr, WifiMode ackMode,
                       double dataSnr, uint16_t dataChannelWidth, uint8_t dataNss) override;
  void DoReportFinalRtsFailed (WifiRemoteStation *station) override;
  void DoReportFinalDataFailed (WifiRemoteStation *station) override;
  WifiTxVector DoGetDataTxVector (WifiRemoteStation *station, uint16_t allowedWidth) override;
  WifiTxVector DoGetRtsTxVector (WifiRemoteStation *station) override;
  WifiMode DoGetDataMode (WifiRemoteStation *station, uint32_t size);
  void CalculateThroughput (void);
  void EnqueueTimestamp (Ptr<const Packet> packet);
  void TrackRxOk (Ptr<const Packet> packet);
  void TraceTxOk (Ptr<const Packet> p);
  void TrackCw (uint32_t cw, uint8_t slot);
  // void TrackTimeout (uint8_t reason, Ptr<const WifiPsdu> psdu, const WifiTxVector &txVector);
  void TrackPhyTxOk (Ptr<const Packet> packet, double txPowerDbm);
  void TrackMonitorSniffRx(Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, bool isShortPreamble, double signalDbm, double noiseDbm);
  void MetreRead(void);
  void AddSnrThreshold(WifiTxVector txVector, double snr);
  void BuildSnrThresholds();
  void DoInitialize();
  double GetSnrThreshold (WifiTxVector txVector) const;
  double CalculateFER (const std::unordered_set<int>& A, const std::unordered_set<int>& B);

  WifiMode m_dataMode; //!< Wifi mode for unicast Data frames
  WifiMode m_ctlMode;  //!< Wifi mode for RTS frames
  double m_snr=0;
  double m_throughput_sta1=0;
  double m_throughput_sta8=0;
  double m_packetTotal=0;
  double m_fer=0;
  uint32_t m_packetRxPerStep=0;
  uint32_t m_txPerStep=0;
  uint32_t m_txPhyPerStep=0;
  uint32_t m_rxPerStep=0;
  uint32_t m_rxPerMetreRead_sta1 = 0;
  uint32_t m_txPerMetreRead_sta1 = 0;
  uint32_t m_bytesTotal_sta8 = 0;
  uint32_t m_rxTotal = 0;
  uint32_t m_txTotal = 0;
  uint32_t m_rxTotal_ap = 0;
  uint32_t m_txTotal_ap = 0;
  uint32_t m_txFail_ap = 0;
  bool m_readyToUpdate = false;
  uint8_t m_nss = 1;
  uint16_t m_cw = 15;
  uint8_t m_next_mcs = 0;
  std::unordered_set<int> packetsSentPerStep;
  std::unordered_set<int> packetsReceivedPerStep;
  Time m_startTime;
  Time m_timeStep{MilliSeconds(12)};
  double m_ber=1e-6;            //!< The maximum Bit Error Rate acceptable at any transmission mode
  typedef std::vector<std::pair<double, WifiTxVector>> Thresholds;
  Thresholds m_thresholds; 
  ns3::Mac48Address m_wifiMacAddress;
  WifiRemoteStation *m_station;

  Ns3AIRL<AiConstantRateEnv, AiConstantRateAct> * m_ns3ai_mod;
  uint16_t m_ns3ai_id;
};

} //namespace ns3

#endif /* AI_CONSTANT_RATE_WIFI_MANAGER_H */
