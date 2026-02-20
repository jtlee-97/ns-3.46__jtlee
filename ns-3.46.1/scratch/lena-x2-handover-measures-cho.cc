/*
 * Copyright (c) 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Manuel Requena <manuel.requena@cttc.es>
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-common.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/channel-condition-model.h"

#include <fstream>
#include <cmath>
#include <limits>
#include <map>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LenaX2HandoverMeasures");

std::ofstream cho_summary_log;
std::ofstream cho_detail_log;
std::ofstream position_distance_log;
std::ofstream x2_delay_log;
std::ofstream sinr_sample_log;

NodeContainer g_ueNodes;
NodeContainer g_enbNodes;
std::map<uint16_t, Ptr<Node>> g_cellIdToEnbNode;

struct SinrSnapshot
{
    double rsrp{0.0};
    double sinrLin{0.0};
    double sinrDb{-std::numeric_limits<double>::infinity()};
    uint64_t timestampMs{0};
};

std::map<uint16_t, SinrSnapshot> g_latestSinrByCell;

bool g_terminalChoTrace = true;
bool g_terminalChoDetailedTrace = false;
bool g_terminalSinrTrace = false;
bool g_includePreAttachSinr = false;
bool g_idealRadioPropagationDelay = true;
double g_radioPropagationSpeedup = 1000.0;

Time
ComputeLightSpeedDelay(double distanceMeters)
{
    static constexpr double c = 299792458.0;
    return Seconds(distanceMeters / c);
}

struct X2DynamicLink
{
    Ptr<Node> srcNode;
    Ptr<Node> dstNode;
    Ptr<PointToPointChannel> channel;
};

void
UpdateDynamicX2Delays(const std::vector<X2DynamicLink>& x2Links,
                      double updatePeriodSec,
                      double simTimeSec)
{
    const uint64_t nowMs = Simulator::Now().GetMilliSeconds();

    for (const auto& link : x2Links)
    {
        Ptr<MobilityModel> srcMobility = link.srcNode->GetObject<MobilityModel>();
        Ptr<MobilityModel> dstMobility = link.dstNode->GetObject<MobilityModel>();
        NS_ASSERT_MSG(srcMobility, "Mobility model not found on X2 source node");
        NS_ASSERT_MSG(dstMobility, "Mobility model not found on X2 destination node");
        NS_ASSERT_MSG(link.channel, "X2 point-to-point channel is null");

        const double distanceMeters = srcMobility->GetDistanceFrom(dstMobility);
        const Time x2Delay = ComputeLightSpeedDelay(distanceMeters);
        link.channel->SetAttribute("Delay", TimeValue(x2Delay));

        x2_delay_log << nowMs
                 << "ms [X2-DELAY-UPDATE] srcNode=" << link.srcNode->GetId()
                 << " dstNode=" << link.dstNode->GetId()
                 << " distanceM=" << distanceMeters
                 << " oneWayDelayMs=" << x2Delay.GetMilliSeconds() << std::endl;
    }

    if (Simulator::Now().GetSeconds() + updatePeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(updatePeriodSec),
                            &UpdateDynamicX2Delays,
                            x2Links,
                            updatePeriodSec,
                            simTimeSec);
    }
}

void
LogPositionsAndDistances(NodeContainer ueNodes,
                         NodeContainer enbNodes,
                         double logPeriodSec,
                         double simTimeSec)
{
    const uint64_t nowMs = Simulator::Now().GetMilliSeconds();

    Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    NS_ASSERT_MSG(ueMobility, "Mobility model not found on UE node");
    Ptr<GeocentricConstantPositionMobilityModel> ueGeoMobility =
        ueNodes.Get(0)->GetObject<GeocentricConstantPositionMobilityModel>();

    const Vector ueTopocentric = ueMobility->GetPosition();
    Vector ueGeographic(0.0, 0.0, 0.0);
    if (ueGeoMobility)
    {
        ueGeographic = ueGeoMobility->GetGeographicPosition();
    }

    position_distance_log << nowMs << "ms UE node=" << ueNodes.Get(0)->GetId()
                          << " latDeg=" << ueGeographic.x
                          << " lonDeg=" << ueGeographic.y
                          << " altM=" << ueGeographic.z
                          << " x=" << ueTopocentric.x
                          << " y=" << ueTopocentric.y
                          << " z=" << ueTopocentric.z
                          << " distToUeM=0" << std::endl;

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> enbMobility = enbNodes.Get(i)->GetObject<MobilityModel>();
        NS_ASSERT_MSG(enbMobility, "Mobility model not found on eNB node");
        Ptr<GeocentricConstantPositionMobilityModel> enbGeoMobility =
            enbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();

        const Vector enbTopocentric = enbMobility->GetPosition();
        Vector enbGeographic(0.0, 0.0, 0.0);
        if (enbGeoMobility)
        {
            enbGeographic = enbGeoMobility->GetGeographicPosition();
        }

        const double distToUeM = enbMobility->GetDistanceFrom(ueMobility);

        position_distance_log << nowMs << "ms gNB node=" << enbNodes.Get(i)->GetId()
                              << " latDeg=" << enbGeographic.x
                              << " lonDeg=" << enbGeographic.y
                              << " altM=" << enbGeographic.z
                              << " x=" << enbTopocentric.x
                              << " y=" << enbTopocentric.y
                              << " z=" << enbTopocentric.z
                              << " distToUeM=" << distToUeM << std::endl;
    }

    if (Simulator::Now().GetSeconds() + logPeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(logPeriodSec),
                            &LogPositionsAndDistances,
                            ueNodes,
                            enbNodes,
                            logPeriodSec,
                            simTimeSec);
    }
}

void
UpdateLeoEnbPositions(NodeContainer enbNodes,
                      const std::vector<double>& initialLongitudesDeg,
                      double orbitLatitudeDeg,
                      double leoAltitudeM,
                      double leoSpeedMps,
                      double updatePeriodSec,
                      double simTimeSec)
{
    const double earthRadiusM = 6371000.0;
    const double orbitRadiusM = earthRadiusM + leoAltitudeM;
    const double deltaLongitudeDeg =
        (leoSpeedMps * Simulator::Now().GetSeconds() / orbitRadiusM) * 180.0 / M_PI;

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> enbMobility =
            enbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        NS_ASSERT_MSG(enbMobility, "Geocentric mobility model not found on eNB node");
        enbMobility->SetGeographicPosition(
            Vector(orbitLatitudeDeg, initialLongitudesDeg.at(i) + deltaLongitudeDeg, leoAltitudeM));
    }

    if (Simulator::Now().GetSeconds() + updatePeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(updatePeriodSec),
                            &UpdateLeoEnbPositions,
                            enbNodes,
                            initialLongitudesDeg,
                            orbitLatitudeDeg,
                            leoAltitudeM,
                            leoSpeedMps,
                            updatePeriodSec,
                            simTimeSec);
    }
}

std::string
ExtractBetween(const std::string& text, const std::string& beginToken, const std::string& endToken)
{
    size_t beginPos = text.find(beginToken);
    if (beginPos == std::string::npos)
    {
        return "-";
    }
    beginPos += beginToken.size();
    size_t endPos = text.find(endToken, beginPos);
    if (endPos == std::string::npos)
    {
        return "-";
    }
    return text.substr(beginPos, endPos - beginPos);
}

std::string
CompactContextId(const std::string& context)
{
    std::string nodeId = ExtractBetween(context, "/NodeList/", "/");
    std::string deviceId = ExtractBetween(context, "/DeviceList/", "/");
    return "N" + nodeId + "D" + deviceId;
}

std::string
CompactContextType(const std::string& context)
{
    size_t pos = context.find("/DeviceList/");
    if (pos == std::string::npos)
    {
        return "Unknown";
    }
    pos = context.find('/', pos + std::string("/DeviceList/").size());
    if (pos == std::string::npos || pos + 1 >= context.size())
    {
        return "Unknown";
    }

    std::string tail = context.substr(pos + 1);
    const std::string token = "$ns3::";
    size_t tokenPos = 0;
    while ((tokenPos = tail.find(token, tokenPos)) != std::string::npos)
    {
        tail.erase(tokenPos, token.size());
    }
    return tail;
}

double
GetAppliedRadioSpeedMps()
{
    static constexpr double c = 299792458.0;
    if (!g_idealRadioPropagationDelay)
    {
        return c;
    }
    return c * std::max(1.0, g_radioPropagationSpeedup);
}

uint16_t
GetCellIdFromContext(const std::string& context)
{
    const std::string nodeIdStr = ExtractBetween(context, "/NodeList/", "/");
    const std::string devIdStr = ExtractBetween(context, "/DeviceList/", "/");
    if (nodeIdStr == "-" || devIdStr == "-")
    {
        return 0;
    }

    Ptr<Node> node = NodeList::GetNode(static_cast<uint32_t>(std::stoul(nodeIdStr)));
    if (!node)
    {
        return 0;
    }

    const uint32_t devId = static_cast<uint32_t>(std::stoul(devIdStr));
    if (devId >= node->GetNDevices())
    {
        return 0;
    }

    Ptr<LteEnbNetDevice> enbDev = DynamicCast<LteEnbNetDevice>(node->GetDevice(devId));
    if (!enbDev)
    {
        return 0;
    }
    return enbDev->GetCellId();
}

double
ComputeUeToCellDistanceMeters(uint16_t cellId)
{
    if (g_ueNodes.GetN() == 0)
    {
        return -1.0;
    }
    auto it = g_cellIdToEnbNode.find(cellId);
    if (it == g_cellIdToEnbNode.end() || !it->second)
    {
        return -1.0;
    }

    Ptr<MobilityModel> ueMobility = g_ueNodes.Get(0)->GetObject<MobilityModel>();
    Ptr<MobilityModel> enbMobility = it->second->GetObject<MobilityModel>();
    if (!ueMobility || !enbMobility)
    {
        return -1.0;
    }
    return ueMobility->GetDistanceFrom(enbMobility);
}

double
ComputeCellToCellDistanceMeters(uint16_t srcCellId, uint16_t dstCellId)
{
    auto srcIt = g_cellIdToEnbNode.find(srcCellId);
    auto dstIt = g_cellIdToEnbNode.find(dstCellId);
    if (srcIt == g_cellIdToEnbNode.end() || dstIt == g_cellIdToEnbNode.end())
    {
        return -1.0;
    }

    Ptr<MobilityModel> srcMobility = srcIt->second->GetObject<MobilityModel>();
    Ptr<MobilityModel> dstMobility = dstIt->second->GetObject<MobilityModel>();
    if (!srcMobility || !dstMobility)
    {
        return -1.0;
    }
    return srcMobility->GetDistanceFrom(dstMobility);
}

std::string
FormatSinrForCell(uint16_t cellId)
{
    auto it = g_latestSinrByCell.find(cellId);
    if (it == g_latestSinrByCell.end())
    {
        return "NA";
    }
    return std::to_string(it->second.sinrDb);
}

void
PrintChoTerminalTrace(uint64_t timestampMs,
                      const std::string& eventName,
                      uint16_t targetCellId,
                      const std::string& sender,
                      const std::string& receiver,
                      double distanceMeters,
                      double txTimeAppliedMs,
                      double txTimePhysicalMs,
                      uint8_t servingRsrp,
                      uint8_t targetRsrp,
                      const std::string& servingSinrDb,
                      const std::string& targetSinrDb)
{
    std::cout << timestampMs << "ms [CHO-MSG]"
              << " name=" << eventName
              << " tgtCell=" << targetCellId
              << " sender=" << sender
              << " receiver=" << receiver
              << " distanceM=" << distanceMeters
              << " txAppliedMs=" << txTimeAppliedMs
              << " txPhysicalMs=" << txTimePhysicalMs
              << " srvRsrp=" << static_cast<uint16_t>(servingRsrp)
              << " tgtRsrp=" << static_cast<uint16_t>(targetRsrp)
              << " srvSinrDb=" << servingSinrDb
              << " tgtSinrDb=" << targetSinrDb
              << std::endl;
}

std::string
SummarizeChoActor(const std::string& eventName)
{
    if (eventName == "A3ConditionSatisfied")
    {
        return "actor=Source-gNB action=PROCESS io=internal";
    }
    if (eventName == "ChoPrepareStart")
    {
        return "actor=Source-gNB action=PROCESS io=internal";
    }
    if (eventName == "HoRequestSentToCandidates")
    {
        return "actor=Source-gNB action=TX io=out";
    }
    if (eventName == "HoRequestAckReceived")
    {
        return "actor=Source-gNB action=RX io=in";
    }
    if (eventName == "RrcReconfigurationWithChoSent")
    {
        return "actor=Source-gNB action=TX io=out";
    }
    if (eventName == "ChoStoredInUe")
    {
        return "actor=UE action=PROCESS io=internal";
    }
    if (eventName == "ChoExecutionStart")
    {
        return "actor=UE action=PROCESS io=internal";
    }
    if (eventName == "TriggerHandoverCalled")
    {
        return "actor=Source-gNB action=PROCESS io=internal";
    }
    if (eventName == "ChoCancelledConditionLeave")
    {
        return "actor=Source-gNB action=PROCESS io=internal";
    }
    if (eventName == "ChoPendingEntryRemoved")
    {
        return "actor=Source-gNB action=PROCESS io=internal";
    }
    if (eventName == "AlgorithmInitialized")
    {
        return "actor=Source-gNB action=PROCESS io=internal";
    }
    return "actor=Unknown action=PROCESS io=internal";
}

std::string
SummarizeChoMessage(const std::string& eventName)
{
    if (eventName == "HoRequestSentToCandidates" || eventName == "HoRequestAckReceived")
    {
        return "message=HoPreparation";
    }
    if (eventName == "RrcReconfigurationWithChoSent" || eventName == "ChoStoredInUe")
    {
        return "message=RrcReconfiguration(CHO)";
    }
    if (eventName == "TriggerHandoverCalled")
    {
        return "message=TriggerHandover";
    }
    if (eventName == "A3ConditionSatisfied")
    {
        return "message=MeasurementReport/A3";
    }
    return "message=" + eventName;
}

void
NotifyConnectionEstablishedUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [UE-CONN] [" << CompactContextId(context)
                    << "] IMSI=" << imsi << " Cell=" << cellid << " RNTI=" << rnti << std::endl;
}

void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellid,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [UE-HO-START] ["
                    << CompactContextId(context) << "] IMSI=" << imsi << " srcCell=" << cellid
                    << " tgtCell=" << targetCellId << " RNTI=" << rnti << std::endl;
}

void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [UE-HO-END] [" << CompactContextId(context)
                    << "] IMSI=" << imsi << " newCell=" << cellid << " RNTI=" << rnti << std::endl;
}

void
NotifyConnectionEstablishedEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [ENB-CONN] [" << CompactContextId(context)
                    << "] Cell=" << cellid << " IMSI=" << imsi << " RNTI=" << rnti << std::endl;
}

void
NotifyHandoverStartEnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [ENB-HO-START] ["
                    << CompactContextId(context) << "] srcCell=" << cellid << " tgtCell="
                    << targetCellId << " IMSI=" << imsi << " RNTI=" << rnti << std::endl;
}

void
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [ENB-HO-END] [" << CompactContextId(context)
                    << "] newCell=" << cellid << " IMSI=" << imsi << " RNTI=" << rnti << std::endl;
}

void
NotifyChoEvent(std::string context,
               uint64_t timestampMs,
               uint16_t rnti,
               std::string eventName,
               uint16_t targetCellId,
               uint8_t servingRsrp,
               uint8_t targetRsrp,
               uint16_t candidateCount)
{
    const std::string actorInfo = SummarizeChoActor(eventName);
    const std::string messageInfo = SummarizeChoMessage(eventName);
    cho_summary_log << timestampMs << "ms [CHO-EVENT] [" << CompactContextId(context) << "] " << eventName
                    << " " << actorInfo << " " << messageInfo
                    << " UE-RNTI=" << rnti << " tgtCell=" << targetCellId
                    << " srvRsrp=" << static_cast<uint16_t>(servingRsrp)
                    << " tgtRsrp=" << static_cast<uint16_t>(targetRsrp)
                    << " cand=" << candidateCount << std::endl;

    if (g_terminalChoTrace)
    {
        const uint16_t sourceCellId = GetCellIdFromContext(context);
        double distanceMeters = -1.0;
        double txAppliedMs = -1.0;
        double txPhysicalMs = -1.0;
        std::string sender = "Unknown";
        std::string receiver = "Unknown";

        static constexpr double c = 299792458.0;

        if (eventName == "HoRequestSentToCandidates" || eventName == "HoRequestAckReceived")
        {
            sender = (eventName == "HoRequestSentToCandidates")
                         ? ("gNB(cell=" + std::to_string(sourceCellId) + ")")
                         : ("gNB(cell=" + std::to_string(targetCellId) + ")");
            receiver = (eventName == "HoRequestSentToCandidates")
                           ? ("gNB(cell=" + std::to_string(targetCellId) + ")")
                           : ("gNB(cell=" + std::to_string(sourceCellId) + ")");
            distanceMeters = ComputeCellToCellDistanceMeters(sourceCellId, targetCellId);
            if (distanceMeters >= 0.0)
            {
                txPhysicalMs = 1000.0 * distanceMeters / c;
                txAppliedMs = txPhysicalMs;
            }
        }
        else if (eventName == "A3ConditionSatisfied")
        {
            sender = "UE";
            receiver = "gNB(cell=" + std::to_string(sourceCellId) + ")";
            distanceMeters = ComputeUeToCellDistanceMeters(sourceCellId);
            if (distanceMeters >= 0.0)
            {
                txPhysicalMs = 1000.0 * distanceMeters / c;
                txAppliedMs = 1000.0 * distanceMeters / GetAppliedRadioSpeedMps();
            }
        }
        else if (eventName == "RrcReconfigurationWithChoSent" || eventName == "ChoStoredInUe")
        {
            sender = "gNB(cell=" + std::to_string(sourceCellId) + ")";
            receiver = "UE";
            distanceMeters = ComputeUeToCellDistanceMeters(sourceCellId);
            if (distanceMeters >= 0.0)
            {
                txPhysicalMs = 1000.0 * distanceMeters / c;
                txAppliedMs = 1000.0 * distanceMeters / GetAppliedRadioSpeedMps();
            }
        }
        else
        {
            sender = (eventName == "ChoStoredInUe" || eventName == "ChoExecutionStart")
                         ? "UE"
                         : ("gNB(cell=" + std::to_string(sourceCellId) + ")");
            receiver = "UE";
            distanceMeters = ComputeUeToCellDistanceMeters(sourceCellId);
            if (distanceMeters >= 0.0)
            {
                txPhysicalMs = 1000.0 * distanceMeters / c;
                txAppliedMs = 1000.0 * distanceMeters / GetAppliedRadioSpeedMps();
            }
        }

        const std::string servingSinrDb = FormatSinrForCell(sourceCellId);
        const std::string targetSinrDb = FormatSinrForCell(targetCellId);
        PrintChoTerminalTrace(timestampMs,
                             eventName,
                             targetCellId,
                             sender,
                             receiver,
                             distanceMeters,
                             txAppliedMs,
                             txPhysicalMs,
                             servingRsrp,
                             targetRsrp,
                             servingSinrDb,
                             targetSinrDb);
    }
}

void
NotifyChoDetailedEvent(std::string context,
                       uint64_t timestampMs,
                       uint16_t rnti,
                       std::string stepName,
                       std::string detail)
{
    cho_detail_log << timestampMs << "ms [CHO-DETAIL] [" << CompactContextId(context) << "] ["
                   << CompactContextType(context) << "] " << stepName << " UE-RNTI=" << rnti
                   << " | " << detail << std::endl;

    if (g_terminalChoDetailedTrace)
    {
        std::cout << timestampMs << "ms [CHO-DETAIL-TTY] [" << CompactContextId(context) << "] ["
                  << CompactContextType(context) << "] " << stepName << " UE-RNTI=" << rnti
                  << " | " << detail << std::endl;
    }

    const bool isRaStep =
        stepName.rfind("Step5_RA_", 0) == 0 || stepName == "Step5_RandomAccessStart" ||
        stepName == "Step5_RandomAccessComplete";

    if (isRaStep)
    {
        cho_summary_log << timestampMs << "ms [CHO-RA] [" << CompactContextId(context)
                       << "] " << stepName << " UE-RNTI=" << rnti << " | " << detail
                       << std::endl;
    }
}

void
NotifyUeSinrSample(std::string context,
                   uint16_t cellId,
                   uint16_t rnti,
                   double rsrp,
                   double sinr,
                   uint8_t componentCarrierId)
{
    const double sinrDb = (sinr > 0.0) ? (10.0 * std::log10(sinr)) : -std::numeric_limits<double>::infinity();

    if (rnti == 0 && !g_includePreAttachSinr)
    {
        return;
    }

    g_latestSinrByCell[cellId] =
        {rsrp, sinr, sinrDb, static_cast<uint64_t>(Simulator::Now().GetMilliSeconds())};

    sinr_sample_log << Simulator::Now().GetMilliSeconds() << "ms [SINR] ["
                    << CompactContextId(context) << "] cell=" << cellId << " rnti=" << rnti
                    << " rsrp=" << rsrp << " sinrLin=" << sinr << " sinrDb=" << sinrDb
                    << " ccId=" << static_cast<uint16_t>(componentCarrierId) << std::endl;

    if (g_terminalSinrTrace)
    {
        std::cout << Simulator::Now().GetMilliSeconds() << "ms [SINR] [" << CompactContextId(context)
                  << "] cell=" << cellId << " rnti=" << rnti << " rsrp=" << rsrp
                  << " sinrLin=" << sinr << " sinrDb=" << sinrDb << std::endl;
    }
}

void
NotifyInitialCellSelectionEndError(std::string context, uint64_t imsi, uint16_t cellId)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [UE-INIT-CELL-ERR] ["
                    << CompactContextId(context) << "] IMSI=" << imsi << " Cell=" << cellId
                    << std::endl;
}

void
NotifyRandomAccessErrorUe(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [UE-RA-ERR] ["
                    << CompactContextId(context) << "] IMSI=" << imsi << " Cell=" << cellId
                    << " RNTI=" << rnti << std::endl;
}

void
NotifyConnectionTimeoutUe(std::string context,
                          uint64_t imsi,
                          uint16_t cellId,
                          uint16_t rnti,
                          uint8_t connEstFailCount)
{
    cho_summary_log << Simulator::Now().GetMilliSeconds() << "ms [UE-CONN-TIMEOUT] ["
                    << CompactContextId(context) << "] IMSI=" << imsi << " Cell=" << cellId
                    << " RNTI=" << rnti
                    << " connEstFailCount=" << static_cast<uint16_t>(connEstFailCount)
                    << std::endl;
}

int
main(int argc, char* argv[])
{
    cho_summary_log.open("cho-summary.log");
    cho_detail_log.open("cho-detail.log");
    position_distance_log.open("position-distance.log");
    x2_delay_log.open("x2-delay.log");
    sinr_sample_log.open("sinr-sample.log");

    uint16_t numberOfUes = 1;
    uint16_t numberOfEnbs = 7;
    int32_t initialAttachEnbIndex = 0;
    uint16_t numBearersPerUe = 0;
    double simTime = 95.0;
    double enbTxPowerDbm = 46.0;
    bool useSatEirpTxPower = true;
    double satEirpDensityDbwPerMhz = 34.0;
    double satAntennaGainDb = 30.0;
    double satApertureRadiusM = 2.0;
    double satAntennaMinGainDb = -20.0;
    bool useCircularApertureAntenna = false;
    double ueAntennaGainDb = 30.0;
    double ueTxPowerDbm = 23.0;
    double carrierFrequencyHz = 2.0e9;
    uint16_t lteBandwidthRb = 100;
    bool shadowingEnabled = false;
    bool enableLteBuiltinTraces = false;
    bool terminalChoTrace = true;
    bool terminalChoDetailedTrace = false;
    bool terminalSinrTrace = false;
    bool includePreAttachSinr = false;
    bool idealRadioPropagationDelay = true;
    double radioPropagationSpeedup = 1000.0;
    double leoAltitudeM = 600000.0;
    double leoSpeedMps = 7560.0;
    double orbitLatitudeDeg = 0.0;
    double ueLatitudeDeg = 0.0;
    double ueLongitudeDeg = 0.0;
    double enbLongitudeStepDeg = 3.0;
    double positionUpdatePeriodMs = 100.0;
    double positionLogPeriodMs = 100.0;

    double choHysteresisDb = 0.0;
    uint16_t choTttMs = 256;
    uint16_t measurementReportDelayMs = 1;
    uint16_t choDecisionDelayMs = 1;
    uint16_t hoRequestPropagationDelayMs = 1;
    uint16_t hoPreparationDelayMs = 8;
    uint16_t hoPreparationPerTargetOffsetMs = 1;
    uint16_t choCommandDeliveryDelayMs = 2;
    uint16_t randomAccessDurationMs = 6;
    uint16_t randomAccessStepDelayMs = 1;
    uint16_t pathSwitchDelayMs = 2;
    bool useIdealRrc = true;
    uint16_t sibRaResponseWindowSize = 10;
    bool useNtnRachTiming = true;
    uint16_t ntnRaResponseWindowSize = 80;
    uint16_t ueT300Ms = 2000;
    uint16_t enbConnectionRequestTimeoutMs = 500;
    uint16_t enbConnectionSetupTimeoutMs = 1000;
    uint16_t enbHandoverJoiningTimeoutMs = 1500;

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Total duration of the simulation (in seconds)", simTime);
    cmd.AddValue(
        "initialAttachEnbIndex",
        "Initial attach eNB index [0..numberOfEnbs-1], -1 means nearest eNB by distance",
        initialAttachEnbIndex);
    cmd.AddValue("enbTxPowerDbm",
                 "TX power [dBm] used by eNBs when useSatEirpTxPower=false (default = 46.0)",
                 enbTxPowerDbm);
    cmd.AddValue("useSatEirpTxPower",
                 "If true, compute eNB Tx power from satEirpDensityDbwPerMhz and LTE bandwidth",
                 useSatEirpTxPower);
    cmd.AddValue("satEirpDensityDbwPerMhz",
                 "Satellite EIRP density [dBW/MHz] (default = 34.0)",
                 satEirpDensityDbwPerMhz);
    cmd.AddValue("satAntennaGainDb",
                 "Satellite/eNB antenna gain [dB] (default = 30.0)",
                 satAntennaGainDb);
    cmd.AddValue("satApertureRadiusM",
                 "Satellite circular-aperture radius [m] for Bessel antenna model (default = 2.0)",
                 satApertureRadiusM);
    cmd.AddValue("satAntennaMinGainDb",
                 "Satellite circular-aperture minimum gain [dB] (default = -20.0)",
                 satAntennaMinGainDb);
    cmd.AddValue("useCircularApertureAntenna",
                 "Use Bessel-based CircularApertureAntennaModel for satellite gNB (requires beam steering model; otherwise RSRP can collapse)",
                 useCircularApertureAntenna);
    cmd.AddValue("ueAntennaGainDb", "UE antenna gain [dB] (default = 30.0)", ueAntennaGainDb);
    cmd.AddValue("useIdealRrc",
                 "Use idealized RRC signaling (false enables real RRC messages)",
                 useIdealRrc);
    cmd.AddValue("sibRaResponseWindowSize",
                 "LTE-standard SIB RA response window size in subframes (allowed: 2,3,4,5,6,7,8,10)",
                 sibRaResponseWindowSize);
    cmd.AddValue("useNtnRachTiming",
                 "Use NTN-specific RA response timing in UE/eNB MAC local timers",
                 useNtnRachTiming);
    cmd.AddValue("ntnRaResponseWindowSize",
                 "NTN effective RA response window size in subframes/TTIs for local timers",
                 ntnRaResponseWindowSize);
    cmd.AddValue("ueT300Ms",
                 "UE RRC T300 timeout in ms (standard LTE range: 100..2000)",
                 ueT300Ms);
    cmd.AddValue("enbConnectionRequestTimeoutMs",
                 "eNB timeout waiting for RRC Connection Request after RA (ms)",
                 enbConnectionRequestTimeoutMs);
    cmd.AddValue("enbConnectionSetupTimeoutMs",
                 "eNB timeout waiting for RRC Connection Setup Complete (ms)",
                 enbConnectionSetupTimeoutMs);
    cmd.AddValue("enbHandoverJoiningTimeoutMs",
                 "eNB timeout waiting for HO joining complete (ms)",
                 enbHandoverJoiningTimeoutMs);
    cmd.AddValue("ueTxPowerDbm", "UE Tx power [dBm] (default = 23.0)", ueTxPowerDbm);
    cmd.AddValue("carrierFrequencyHz",
                 "Carrier frequency [Hz] for NTN pathloss model (default = 2e9)",
                 carrierFrequencyHz);
    cmd.AddValue("lteBandwidthRb", "LTE bandwidth in RBs (default = 100, 20MHz)", lteBandwidthRb);
    cmd.AddValue("shadowingEnabled",
                 "Enable/disable NTN shadowing in pathloss model (default = false)",
                 shadowingEnabled);
    cmd.AddValue("enableLteBuiltinTraces",
                 "Enable LteHelper built-in PHY/MAC/RLC/PDCP traces (default = false)",
                 enableLteBuiltinTraces);
    cmd.AddValue("terminalChoTrace",
                 "Print CHO message trace to terminal with sender/receiver/distance/tx-time",
                 terminalChoTrace);
    cmd.AddValue("terminalChoDetailedTrace",
                 "Mirror ChoDetailedEvent lines to terminal",
                 terminalChoDetailedTrace);
    cmd.AddValue("terminalSinrTrace", "Print SINR samples to terminal", terminalSinrTrace);
    cmd.AddValue("includePreAttachSinr",
                 "Include pre-attach SINR samples (RNTI=0) in sinr-sample.log",
                 includePreAttachSinr);
    cmd.AddValue("idealRadioPropagationDelay",
                 "If true, speed up DL/UL propagation to relax LTE random-access timing in NTN",
                 idealRadioPropagationDelay);
    cmd.AddValue("radioPropagationSpeedup",
                 "Propagation speed multiplier when idealRadioPropagationDelay=true (default = 1000)",
                 radioPropagationSpeedup);
    cmd.AddValue("leoAltitudeM", "LEO satellite altitude in meters (default = 600000)", leoAltitudeM);
    cmd.AddValue("leoSpeedMps", "LEO satellite speed in m/s (default = 7560)", leoSpeedMps);
    cmd.AddValue("orbitLatitudeDeg", "Satellite orbit latitude in degrees (default = 0.0)", orbitLatitudeDeg);
    cmd.AddValue("ueLatitudeDeg", "UE latitude in degrees (default = 0.0)", ueLatitudeDeg);
    cmd.AddValue("ueLongitudeDeg", "UE longitude in degrees (default = 0.0)", ueLongitudeDeg);
    cmd.AddValue("enbLongitudeStepDeg",
                 "Initial inter-satellite longitude spacing in degrees (default = 3.0)",
                 enbLongitudeStepDeg);
    cmd.AddValue("positionUpdatePeriodMs",
                 "Satellite position update period in ms (default = 100)",
                 positionUpdatePeriodMs);
    cmd.AddValue("positionLogPeriodMs",
                 "Position/distance log period in ms (default = 100)",
                 positionLogPeriodMs);
    cmd.AddValue("choHysteresisDb", "CHO hysteresis in dB (default = 0.0)", choHysteresisDb);
    cmd.AddValue("choTttMs", "CHO time-to-trigger in ms (default = 256)", choTttMs);
    cmd.AddValue("measurementReportDelayMs", "MR send->receive delay in ms", measurementReportDelayMs);
    cmd.AddValue("choDecisionDelayMs", "CHO decision processing delay in ms", choDecisionDelayMs);
    cmd.AddValue("hoRequestPropagationDelayMs", "HO request/ACK one-way propagation delay in ms", hoRequestPropagationDelayMs);
    cmd.AddValue("hoPreparationDelayMs", "Admission-control base delay in ms", hoPreparationDelayMs);
    cmd.AddValue("hoPreparationPerTargetOffsetMs",
                 "Additional admission-control delay per candidate index in ms",
                 hoPreparationPerTargetOffsetMs);
    cmd.AddValue("choCommandDeliveryDelayMs", "Source->UE CHO command delay in ms", choCommandDeliveryDelayMs);
    cmd.AddValue("randomAccessDurationMs", "UE random-access duration in ms", randomAccessDurationMs);
    cmd.AddValue("randomAccessStepDelayMs", "UE random-access per-step timeline delay in ms", randomAccessStepDelayMs);
    cmd.AddValue("pathSwitchDelayMs", "Modeled path-switch delay in ms", pathSwitchDelayMs);
    cmd.Parse(argc, argv);

    if (lteBandwidthRb < 6 || lteBandwidthRb > 100)
    {
        NS_FATAL_ERROR("lteBandwidthRb must be within [6, 100]");
    }

    if (sibRaResponseWindowSize != 2 && sibRaResponseWindowSize != 3 &&
        sibRaResponseWindowSize != 4 && sibRaResponseWindowSize != 5 &&
        sibRaResponseWindowSize != 6 && sibRaResponseWindowSize != 7 &&
        sibRaResponseWindowSize != 8 && sibRaResponseWindowSize != 10)
    {
        NS_FATAL_ERROR("sibRaResponseWindowSize must be one of {2,3,4,5,6,7,8,10}");
    }

    if (ntnRaResponseWindowSize < 2 || ntnRaResponseWindowSize > 10240)
    {
        NS_FATAL_ERROR("ntnRaResponseWindowSize must be within [2, 10240]");
    }

    if (ueT300Ms < 100 || ueT300Ms > 2000)
    {
        NS_FATAL_ERROR("ueT300Ms must be within [100, 2000]");
    }

    if (enbConnectionRequestTimeoutMs < 1 || enbConnectionRequestTimeoutMs > ueT300Ms)
    {
        NS_FATAL_ERROR("enbConnectionRequestTimeoutMs must be within [1, ueT300Ms]");
    }

    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(useIdealRrc));
    Config::SetDefault("ns3::LteUeRrc::T300", TimeValue(MilliSeconds(ueT300Ms)));
    Config::SetDefault("ns3::LteEnbRrc::ConnectionRequestTimeoutDuration",
                       TimeValue(MilliSeconds(enbConnectionRequestTimeoutMs)));
    Config::SetDefault("ns3::LteEnbRrc::ConnectionSetupTimeoutDuration",
                       TimeValue(MilliSeconds(enbConnectionSetupTimeoutMs)));
    Config::SetDefault("ns3::LteEnbRrc::HandoverJoiningTimeoutDuration",
                       TimeValue(MilliSeconds(enbHandoverJoiningTimeoutMs)));
    Config::SetDefault("ns3::LteEnbMac::RaResponseWindowSize",
                       UintegerValue(sibRaResponseWindowSize));
    Config::SetDefault("ns3::LteEnbMac::UseNtnRachTiming", BooleanValue(useNtnRachTiming));
    Config::SetDefault("ns3::LteEnbMac::NtnRaResponseWindowSize",
                       UintegerValue(ntnRaResponseWindowSize));
    Config::SetDefault("ns3::LteUeMac::UseNtnRachTiming", BooleanValue(useNtnRachTiming));
    Config::SetDefault("ns3::LteUeMac::NtnRaResponseWindowSize",
                       UintegerValue(ntnRaResponseWindowSize));

    if (useSatEirpTxPower)
    {
        const double bandwidthHz = static_cast<double>(lteBandwidthRb) * 180000.0;
        enbTxPowerDbm =
            (satEirpDensityDbwPerMhz + 10.0 * std::log10(bandwidthHz / 1e6) - satAntennaGainDb) +
            30.0;
    }

    g_terminalChoTrace = terminalChoTrace;
    g_terminalChoDetailedTrace = terminalChoDetailedTrace;
    g_terminalSinrTrace = terminalSinrTrace;
    g_includePreAttachSinr = includePreAttachSinr;
    g_idealRadioPropagationDelay = idealRadioPropagationDelay;
    g_radioPropagationSpeedup = radioPropagationSpeedup;

    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(lteBandwidthRb));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(lteBandwidthRb));

    if (useCircularApertureAntenna)
    {
        lteHelper->SetEnbAntennaModelType("ns3::CircularApertureAntennaModel");
        lteHelper->SetEnbAntennaModelAttribute("OperatingFrequency", DoubleValue(carrierFrequencyHz));
        lteHelper->SetEnbAntennaModelAttribute("AntennaCircularApertureRadius",
                                               DoubleValue(satApertureRadiusM));
        lteHelper->SetEnbAntennaModelAttribute("AntennaMaxGainDb", DoubleValue(satAntennaGainDb));
        lteHelper->SetEnbAntennaModelAttribute("AntennaMinGainDb",
                                               DoubleValue(satAntennaMinGainDb));
    }
    else
    {
        lteHelper->SetEnbAntennaModelType("ns3::IsotropicAntennaModel");
        lteHelper->SetEnbAntennaModelAttribute("Gain", DoubleValue(satAntennaGainDb));
    }
    lteHelper->SetUeAntennaModelType("ns3::IsotropicAntennaModel");
    lteHelper->SetUeAntennaModelAttribute("Gain", DoubleValue(ueAntennaGainDb));

    lteHelper->SetHandoverAlgorithmType("ns3::ConditionalHandoverAlgorithm");
    lteHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(choHysteresisDb));
    lteHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(choTttMs)));
    lteHelper->SetHandoverAlgorithmAttribute("MeasurementReportDelay",
                                             TimeValue(MilliSeconds(measurementReportDelayMs)));
    lteHelper->SetHandoverAlgorithmAttribute("ChoDecisionDelay",
                                             TimeValue(MilliSeconds(choDecisionDelayMs)));
    lteHelper->SetHandoverAlgorithmAttribute("HoRequestPropagationDelay",
                                             TimeValue(MilliSeconds(hoRequestPropagationDelayMs)));
    lteHelper->SetHandoverAlgorithmAttribute("HoPreparationDelay",
                                             TimeValue(MilliSeconds(hoPreparationDelayMs)));
    lteHelper->SetHandoverAlgorithmAttribute("HoPreparationPerTargetOffset",
                                             TimeValue(MilliSeconds(hoPreparationPerTargetOffsetMs)));
    lteHelper->SetHandoverAlgorithmAttribute("ChoCommandDeliveryDelay",
                                             TimeValue(MilliSeconds(choCommandDeliveryDelayMs)));
    lteHelper->SetHandoverAlgorithmAttribute("RandomAccessDuration",
                                             TimeValue(MilliSeconds(randomAccessDurationMs)));
    lteHelper->SetHandoverAlgorithmAttribute("RandomAccessStepDelay",
                                             TimeValue(MilliSeconds(randomAccessStepDelayMs)));
    lteHelper->SetHandoverAlgorithmAttribute("PathSwitchDelay",
                                             TimeValue(MilliSeconds(pathSwitchDelayMs)));

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);
    Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress(1);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numberOfEnbs);
    ueNodes.Create(numberOfUes);
    g_enbNodes = enbNodes;
    g_ueNodes = ueNodes;

    MobilityHelper enbMobility;
    enbMobility.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    enbMobility.Install(enbNodes);

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);

    const Vector referencePoint(ueLatitudeDeg, ueLongitudeDeg, 0.0);
    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> enbGeoMobility =
            enbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        NS_ASSERT_MSG(enbGeoMobility, "Geocentric mobility model not found on eNB node");
        enbGeoMobility->SetCoordinateTranslationReferencePoint(referencePoint);
    }
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> ueGeoMobility =
            ueNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        NS_ASSERT_MSG(ueGeoMobility, "Geocentric mobility model not found on UE node");
        ueGeoMobility->SetCoordinateTranslationReferencePoint(referencePoint);
        ueGeoMobility->SetGeographicPosition(Vector(ueLatitudeDeg, ueLongitudeDeg, 0.0));
    }

    std::vector<double> enbInitialLongitudesDeg(numberOfEnbs, ueLongitudeDeg);
    const double center = (static_cast<double>(numberOfEnbs) - 1.0) / 2.0;
    for (uint16_t i = 0; i < numberOfEnbs; ++i)
    {
        enbInitialLongitudesDeg[i] = ueLongitudeDeg + (static_cast<double>(i) - center) * enbLongitudeStepDeg;
    }

    UpdateLeoEnbPositions(enbNodes,
                          enbInitialLongitudesDeg,
                          orbitLatitudeDeg,
                          leoAltitudeM,
                          leoSpeedMps,
                          positionUpdatePeriodMs / 1000.0,
                          simTime);

    LogPositionsAndDistances(ueNodes, enbNodes, positionLogPeriodMs / 1000.0, simTime);

    // Configure 3GPP NTN Rural propagation model for LEO satellite scenario
    lteHelper->SetPathlossModelType(TypeId::LookupByName("ns3::ThreeGppNTNRuralPropagationLossModel"));
    lteHelper->SetPathlossModelAttribute("Frequency", DoubleValue(carrierFrequencyHz));
    lteHelper->SetPathlossModelAttribute("ShadowingEnabled", BooleanValue(shadowingEnabled));
    
    Ptr<ThreeGppNTNRuralChannelConditionModel> channelConditionModel =
        CreateObject<ThreeGppNTNRuralChannelConditionModel>();
    lteHelper->SetPathlossModelAttribute("ChannelConditionModel", PointerValue(channelConditionModel));

    Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(enbTxPowerDbm));
    Config::SetDefault("ns3::LteUePhy::TxPower", DoubleValue(ueTxPowerDbm));

    cho_summary_log << "0ms [NTN-LINK-BUDGET]"
                    << " scenario=NTN-Rural"
                    << " freqReuse=1"
                    << " freqHz=" << carrierFrequencyHz
                    << " bandwidthRb=" << lteBandwidthRb
                    << " enbTxPowerDbm=" << enbTxPowerDbm
                    << " ueTxPowerDbm=" << ueTxPowerDbm
                    << " satAntennaGainDb=" << satAntennaGainDb
                    << " satApertureRadiusM=" << satApertureRadiusM
                    << " satAntennaModel="
                    << (useCircularApertureAntenna ? "CircularAperture(Bessel,no-beam-steering)"
                                                   : "Isotropic(equivalent-gain)")
                    << " ueAntennaGainDb=" << ueAntennaGainDb
                    << " shadowingEnabled=" << (shadowingEnabled ? "true" : "false")
                    << " idealRadioPropagationDelay="
                    << (idealRadioPropagationDelay ? "true" : "false")
                    << " radioPropagationSpeedup=" << radioPropagationSpeedup
                    << " useIdealRrc=" << (useIdealRrc ? "true" : "false")
                    << " sibRaResponseWindowSize=" << sibRaResponseWindowSize
                    << " useNtnRachTiming=" << (useNtnRachTiming ? "true" : "false")
                    << " ntnRaResponseWindowSize=" << ntnRaResponseWindowSize
                    << " ueT300Ms=" << ueT300Ms
                    << " enbConnectionRequestTimeoutMs=" << enbConnectionRequestTimeoutMs
                    << " enbConnectionSetupTimeoutMs=" << enbConnectionSetupTimeoutMs
                    << " enbHandoverJoiningTimeoutMs=" << enbHandoverJoiningTimeoutMs
                    << std::endl;

    if (useCircularApertureAntenna)
    {
        std::cout << "[ANTENNA-NOTE] CircularAperture(Bessel) enabled without explicit beam steering; "
                  << "RSRP may drop near 0 due to off-boresight attenuation." << std::endl;
    }

    std::cout << "[TOPOLOGY] enbCount=" << numberOfEnbs << " ueCount=" << numberOfUes
              << " movingEnb=true leoAltitudeM=" << leoAltitudeM << " leoSpeedMps=" << leoSpeedMps
              << " ueLatDeg=" << ueLatitudeDeg << " ueLonDeg=" << ueLongitudeDeg
              << " enbLonStepDeg=" << enbLongitudeStepDeg << std::endl;
    std::cout << "[RADIO-DELAY] mode=" << (idealRadioPropagationDelay ? "ideal" : "real")
              << " appliedSpeedMps=" << GetAppliedRadioSpeedMps()
              << " physicalSpeedMps=299792458" << std::endl;

    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    g_cellIdToEnbNode.clear();
    for (uint32_t i = 0; i < enbLteDevs.GetN(); ++i)
    {
        Ptr<LteEnbNetDevice> enbDev = DynamicCast<LteEnbNetDevice>(enbLteDevs.Get(i));
        if (enbDev)
        {
            g_cellIdToEnbNode[enbDev->GetCellId()] = enbNodes.Get(i);
        }
    }

    Ptr<SpectrumChannel> dlChannel = lteHelper->GetDownlinkSpectrumChannel();
    Ptr<SpectrumChannel> ulChannel = lteHelper->GetUplinkSpectrumChannel();
    NS_ASSERT_MSG(dlChannel, "Downlink spectrum channel is not initialized");
    NS_ASSERT_MSG(ulChannel, "Uplink spectrum channel is not initialized");
    Ptr<ConstantSpeedPropagationDelayModel> dlDelayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<ConstantSpeedPropagationDelayModel> ulDelayModel =
        CreateObject<ConstantSpeedPropagationDelayModel>();
    if (idealRadioPropagationDelay)
    {
        static constexpr double c = 299792458.0;
        const double speedup = std::max(1.0, radioPropagationSpeedup);
        dlDelayModel->SetSpeed(c * speedup);
        ulDelayModel->SetSpeed(c * speedup);
    }
    dlChannel->SetPropagationDelayModel(dlDelayModel);
    ulChannel->SetPropagationDelayModel(ulDelayModel);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));

    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/InitialCellSelectionEndError",
                    MakeCallback(&NotifyInitialCellSelectionEndError));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/RandomAccessError",
                    MakeCallback(&NotifyRandomAccessErrorUe));
    Config::Connect("/NodeList/*/DeviceList/*/LteUeRrc/ConnectionTimeout",
                    MakeCallback(&NotifyConnectionTimeoutUe));
    Config::Connect(
        "/NodeList/*/DeviceList/*/$ns3::LteEnbNetDevice/LteHandoverAlgorithm/$ns3::ConditionalHandoverAlgorithm/ChoEvent",
        MakeCallback(&NotifyChoEvent));
    Config::Connect(
        "/NodeList/*/DeviceList/*/$ns3::LteEnbNetDevice/LteHandoverAlgorithm/$ns3::ConditionalHandoverAlgorithm/ChoDetailedEvent",
        MakeCallback(&NotifyChoDetailedEvent));
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/LteUePhy/ReportCurrentCellRsrpSinr",
                    MakeCallback(&NotifyUeSinrSample));

    uint32_t attachEnbIndex = 0;
    if (initialAttachEnbIndex >= 0)
    {
        const uint32_t requestedIndex = static_cast<uint32_t>(initialAttachEnbIndex);
        if (requestedIndex >= enbNodes.GetN())
        {
            NS_FATAL_ERROR("initialAttachEnbIndex is out of range");
        }
        attachEnbIndex = requestedIndex;
    }
    else
    {
        Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
        NS_ASSERT_MSG(ueMobility, "Mobility model not found on UE node for initial attach");

        double minDistance = std::numeric_limits<double>::max();
        for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
        {
            Ptr<MobilityModel> enbMobility = enbNodes.Get(i)->GetObject<MobilityModel>();
            NS_ASSERT_MSG(enbMobility, "Mobility model not found on eNB node for initial attach");
            const double distance = ueMobility->GetDistanceFrom(enbMobility);
            if (distance < minDistance)
            {
                minDistance = distance;
                attachEnbIndex = i;
            }
        }
    }

    cho_summary_log << "0ms [INIT-ATTACH]"
                    << " selectedEnbIndex=" << attachEnbIndex
                    << std::endl;

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        lteHelper->Attach(ueLteDevs.Get(i), enbLteDevs.Get(attachEnbIndex));
    }

    uint16_t dlPort = 10000;
    uint16_t ulPort = 20000;

    Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable>();
    startTimeSeconds->SetAttribute("Min", DoubleValue(0));
    startTimeSeconds->SetAttribute("Max", DoubleValue(0.010));

    for (uint32_t u = 0; u < numberOfUes; ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

        for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {
            ++dlPort;
            ++ulPort;

            ApplicationContainer clientApps;
            ApplicationContainer serverApps;

            UdpClientHelper dlClientHelper(ueIpIfaces.GetAddress(u), dlPort);
            clientApps.Add(dlClientHelper.Install(remoteHost));
            PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            serverApps.Add(dlPacketSinkHelper.Install(ue));

            UdpClientHelper ulClientHelper(remoteHostAddr, ulPort);
            clientApps.Add(ulClientHelper.Install(ue));
            PacketSinkHelper ulPacketSinkHelper("ns3::UdpSocketFactory",
                                                InetSocketAddress(Ipv4Address::GetAny(), ulPort));
            serverApps.Add(ulPacketSinkHelper.Install(remoteHost));

            Ptr<EpcTft> tft = Create<EpcTft>();
            EpcTft::PacketFilter dlpf;
            dlpf.localPortStart = dlPort;
            dlpf.localPortEnd = dlPort;
            tft->Add(dlpf);
            EpcTft::PacketFilter ulpf;
            ulpf.remotePortStart = ulPort;
            ulpf.remotePortEnd = ulPort;
            tft->Add(ulpf);
            EpsBearer bearer(EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
            lteHelper->ActivateDedicatedEpsBearer(ueLteDevs.Get(u), bearer, tft);

            Time startTime = Seconds(startTimeSeconds->GetValue());
            serverApps.Start(startTime);
            clientApps.Start(startTime);
        }
    }

    std::vector<X2DynamicLink> x2Links;
    x2Links.reserve(static_cast<size_t>(numberOfEnbs) * static_cast<size_t>(numberOfEnbs - 1) / 2);

    for (uint32_t i = 0; i < enbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> enbMobilityA = enbNodes.Get(i)->GetObject<MobilityModel>();
        NS_ASSERT_MSG(enbMobilityA, "Mobility model not found on source gNB node");

        for (uint32_t j = i + 1; j < enbNodes.GetN(); ++j)
        {
            Ptr<MobilityModel> enbMobilityB = enbNodes.Get(j)->GetObject<MobilityModel>();
            NS_ASSERT_MSG(enbMobilityB, "Mobility model not found on target gNB node");

            const double distanceMeters = enbMobilityA->GetDistanceFrom(enbMobilityB);
            const Time x2Delay = ComputeLightSpeedDelay(distanceMeters);

            const uint32_t srcNDevicesBefore = enbNodes.Get(i)->GetNDevices();
            const uint32_t dstNDevicesBefore = enbNodes.Get(j)->GetNDevices();

            epcHelper->SetAttribute("X2LinkDelay", TimeValue(x2Delay));
            lteHelper->AddX2Interface(enbNodes.Get(i), enbNodes.Get(j));

            Ptr<PointToPointNetDevice> srcX2Dev =
                DynamicCast<PointToPointNetDevice>(enbNodes.Get(i)->GetDevice(srcNDevicesBefore));
            Ptr<PointToPointNetDevice> dstX2Dev =
                DynamicCast<PointToPointNetDevice>(enbNodes.Get(j)->GetDevice(dstNDevicesBefore));
            NS_ASSERT_MSG(srcX2Dev, "Failed to get source X2 point-to-point device");
            NS_ASSERT_MSG(dstX2Dev, "Failed to get destination X2 point-to-point device");

            Ptr<PointToPointChannel> x2Channel =
                DynamicCast<PointToPointChannel>(srcX2Dev->GetChannel());
            NS_ASSERT_MSG(x2Channel, "Failed to get X2 point-to-point channel");
            NS_ASSERT_MSG(srcX2Dev->GetChannel() == dstX2Dev->GetChannel(),
                          "X2 endpoints are not attached to the same channel");

            x2Links.push_back({enbNodes.Get(i), enbNodes.Get(j), x2Channel});

            x2_delay_log << Simulator::Now().GetMilliSeconds()
                         << "ms [X2-DELAY-CONFIG] srcNode=" << enbNodes.Get(i)->GetId()
                         << " dstNode=" << enbNodes.Get(j)->GetId()
                         << " distanceM=" << distanceMeters
                         << " oneWayDelayMs=" << x2Delay.GetMilliSeconds() << std::endl;
        }
    }

    UpdateDynamicX2Delays(x2Links, positionUpdatePeriodMs / 1000.0, simTime);

    if (enableLteBuiltinTraces)
    {
        lteHelper->EnablePhyTraces();
        lteHelper->EnableMacTraces();
        lteHelper->EnableRlcTraces();
        lteHelper->EnablePdcpTraces();
    }

    Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats();
    if (rlcStats)
    {
        rlcStats->SetAttribute("EpochDuration", TimeValue(Seconds(1)));
    }
    Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats();
    if (pdcpStats)
    {
        pdcpStats->SetAttribute("EpochDuration", TimeValue(Seconds(1)));
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    cho_summary_log.close();
    cho_detail_log.close();
    position_distance_log.close();
    x2_delay_log.close();
    sinr_sample_log.close();

    return 0;
}
