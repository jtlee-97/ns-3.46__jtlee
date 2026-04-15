#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/netanim-module.h"
#include "ns3/node-list.h"
#include "ns3/nr-conditional-handover-algorithm.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-net-device.h"

#include <cmath>
#include <fstream>
#include <limits>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrX2HandoverMeasuresCho");

std::ofstream choSummaryLog;
std::ofstream choDetailLog;
std::ofstream positionDistanceLog;
std::ofstream x2DelayLog;
std::ofstream sinrLog;

bool gEnableMacPhyCtrlTrace = true;
int64_t gMacPhyTraceStartMs = -1;
int64_t gMacPhyTraceEndMs = -1;
bool gMacPhyIncludeRachPreamble = true;
bool gMacPhyIncludeRar = true;
bool gMacPhyIncludeUlDci = false;
bool gMacPhyIncludeDlDci = true;
bool gMacPhyIncludeDlHarq = false;
bool gMacPhyIncludeSr = false;
bool gEnableSinrTrace = true;
int64_t gSinrTraceStartMs = -1;
int64_t gSinrTraceEndMs = -1;

NodeContainer gUeNodes;
NodeContainer gGnbNodes;
std::map<uint16_t, Ptr<Node>> gCellIdToGnbNode;
AnimationInterface* gNetAnim = nullptr;
bool gNetAnimCtrlCounterEnabled = false;
uint32_t gNetAnimCtrlTxCounterId = std::numeric_limits<uint32_t>::max();
uint32_t gNetAnimCtrlRxCounterId = std::numeric_limits<uint32_t>::max();
std::map<uint32_t, uint32_t> gNodeCtrlTxCount;
std::map<uint32_t, uint32_t> gNodeCtrlRxCount;
bool gNetAnimMsgTypeCounterEnabled = false;
std::map<std::string, uint32_t> gNetAnimMsgTypeTxCounterId;
std::map<std::string, uint32_t> gNetAnimMsgTypeRxCounterId;
std::map<uint32_t, std::map<std::string, uint32_t>> gNodeMsgTypeTxCount;
std::map<uint32_t, std::map<std::string, uint32_t>> gNodeMsgTypeRxCount;

std::vector<std::string>
GetEnabledControlMessageTypes()
{
    std::vector<std::string> types;
    if (gMacPhyIncludeRachPreamble)
    {
        types.push_back("RACH_PREAMBLE");
    }
    if (gMacPhyIncludeRar)
    {
        types.push_back("RAR");
    }
    if (gMacPhyIncludeUlDci)
    {
        types.push_back("UL_DCI");
    }
    if (gMacPhyIncludeDlDci)
    {
        types.push_back("DL_DCI");
    }
    if (gMacPhyIncludeDlHarq)
    {
        types.push_back("DL_HARQ");
    }
    if (gMacPhyIncludeSr)
    {
        types.push_back("SR");
    }
    return types;
}

bool
TryReadAttrDouble(const std::string& line, const std::string& attr, double& value)
{
    const std::string token = attr + "=\"";
    const size_t start = line.find(token);
    if (start == std::string::npos)
    {
        return false;
    }
    const size_t valueStart = start + token.size();
    const size_t valueEnd = line.find('"', valueStart);
    if (valueEnd == std::string::npos)
    {
        return false;
    }

    std::istringstream iss(line.substr(valueStart, valueEnd - valueStart));
    iss >> value;
    return !iss.fail();
}

std::string
ReplaceAttrDouble(const std::string& line, const std::string& attr, double value)
{
    const std::string token = attr + "=\"";
    const size_t start = line.find(token);
    if (start == std::string::npos)
    {
        return line;
    }
    const size_t valueStart = start + token.size();
    const size_t valueEnd = line.find('"', valueStart);
    if (valueEnd == std::string::npos)
    {
        return line;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3) << value;

    std::string out = line;
    out.replace(valueStart, valueEnd - valueStart, oss.str());
    return out;
}

void
GenerateCompactNetAnimXml(const std::string& sourceFile,
                          const std::string& compactFile,
                          double compactHalfSpan)
{
    std::ifstream in(sourceFile);
    if (!in.is_open())
    {
        return;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line))
    {
        lines.push_back(line);
    }
    in.close();

    struct CoordRef
    {
        size_t lineIndex;
        bool nodeLoc;
        double x;
        double y;
    };
    std::vector<CoordRef> coords;
    coords.reserve(lines.size() / 4);

    double minX = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double minY = std::numeric_limits<double>::max();
    double maxY = std::numeric_limits<double>::lowest();

    for (size_t i = 0; i < lines.size(); ++i)
    {
        double x = 0.0;
        double y = 0.0;
        bool matched = false;
        bool nodeLoc = false;

        if (lines[i].find("<node ") != std::string::npos)
        {
            matched = TryReadAttrDouble(lines[i], "locX", x) && TryReadAttrDouble(lines[i], "locY", y);
            nodeLoc = true;
        }
        else if (lines[i].find("<nu p=\"p\"") != std::string::npos)
        {
            matched = TryReadAttrDouble(lines[i], "x", x) && TryReadAttrDouble(lines[i], "y", y);
            nodeLoc = false;
        }

        if (!matched)
        {
            continue;
        }

        coords.push_back({i, nodeLoc, x, y});
        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
    }

    if (coords.empty())
    {
        return;
    }

    const double centerX = 0.5 * (minX + maxX);
    const double centerY = 0.5 * (minY + maxY);
    const double spanX = std::max(1.0, maxX - minX);
    const double spanY = std::max(1.0, maxY - minY);
    const double scale = (2.0 * compactHalfSpan) / std::max(spanX, spanY);

    for (const auto& ref : coords)
    {
        const double nx = (ref.x - centerX) * scale;
        const double ny = (ref.y - centerY) * scale;
        if (ref.nodeLoc)
        {
            lines[ref.lineIndex] = ReplaceAttrDouble(lines[ref.lineIndex], "locX", nx);
            lines[ref.lineIndex] = ReplaceAttrDouble(lines[ref.lineIndex], "locY", ny);
        }
        else
        {
            lines[ref.lineIndex] = ReplaceAttrDouble(lines[ref.lineIndex], "x", nx);
            lines[ref.lineIndex] = ReplaceAttrDouble(lines[ref.lineIndex], "y", ny);
        }
    }

    std::ofstream out(compactFile);
    if (!out.is_open())
    {
        return;
    }
    for (const auto& l : lines)
    {
        out << l << '\n';
    }
}

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

        x2DelayLog << nowMs << "ms [X2-DELAY-UPDATE] srcNode=" << link.srcNode->GetId()
                   << " dstNode=" << link.dstNode->GetId() << " distanceM=" << distanceMeters
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
ApplyNetAnimVisualStyle(AnimationInterface& anim,
                        const NodeContainer& gnbNodes,
                        const NodeContainer& ueNodes,
                        Ptr<Node> pgw,
                        Ptr<Node> remoteHost,
                        double gnbNodeSize,
                        double ueNodeSize,
                        double coreNodeSize)
{
    const std::vector<std::tuple<uint8_t, uint8_t, uint8_t>> gnbPalette = {
        {231, 76, 60},
        {52, 152, 219},
        {155, 89, 182},
        {241, 196, 15},
        {230, 126, 34},
        {26, 188, 156},
        {149, 165, 166},
    };

    std::map<uint32_t, uint16_t> nodeIdToCellId;
    for (const auto& entry : gCellIdToGnbNode)
    {
        if (entry.second)
        {
            nodeIdToCellId[entry.second->GetId()] = entry.first;
        }
    }

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<Node> gnb = gnbNodes.Get(i);
        const auto color = gnbPalette.at(i % gnbPalette.size());
        const uint16_t cellId = nodeIdToCellId.count(gnb->GetId()) ? nodeIdToCellId.at(gnb->GetId()) : 0;
        anim.UpdateNodeDescription(gnb,
                                   "g" + std::to_string(i) + "/c" +
                                       std::to_string(cellId));
        anim.UpdateNodeColor(gnb, std::get<0>(color), std::get<1>(color), std::get<2>(color));
        anim.UpdateNodeSize(gnb, gnbNodeSize, gnbNodeSize);
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        anim.UpdateNodeDescription(ue, "UE" + std::to_string(i));
        anim.UpdateNodeColor(ue, 40, 167, 69);
        anim.UpdateNodeSize(ue, ueNodeSize, ueNodeSize);
    }

    if (pgw)
    {
        anim.UpdateNodeDescription(pgw, "PGW");
        anim.UpdateNodeColor(pgw, 108, 117, 125);
        anim.UpdateNodeSize(pgw, coreNodeSize, coreNodeSize);
    }

    if (remoteHost)
    {
        anim.UpdateNodeDescription(remoteHost, "RemoteHost");
        anim.UpdateNodeColor(remoteHost, 23, 162, 184);
        anim.UpdateNodeSize(remoteHost, coreNodeSize, coreNodeSize);
    }

    uint32_t stationaryIndex = 0;
    for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
    {
        Ptr<Node> node = *it;
        if (!node || node->GetObject<MobilityModel>())
        {
            continue;
        }

        const double x = -280000.0 + 80000.0 * static_cast<double>(stationaryIndex);
        const double y = -220000.0;
        AnimationInterface::SetConstantPosition(node, x, y, 0.0);

        if (node != pgw && node != remoteHost)
        {
            anim.UpdateNodeDescription(node, "Core-" + std::to_string(node->GetId()));
            anim.UpdateNodeColor(node, 108, 117, 125);
            anim.UpdateNodeSize(node, coreNodeSize, coreNodeSize);
        }

        ++stationaryIndex;
    }
}

void
LogPositionsAndDistances(NodeContainer ueNodes,
                         NodeContainer gnbNodes,
                         double logPeriodSec,
                         double simTimeSec)
{
    const uint64_t nowMs = Simulator::Now().GetMilliSeconds();

    Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    NS_ASSERT_MSG(ueMobility, "Mobility model not found on UE node");

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> gnbMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
        NS_ASSERT_MSG(gnbMobility, "Mobility model not found on gNB node");

        const double distToUeM = gnbMobility->GetDistanceFrom(ueMobility);

        positionDistanceLog << nowMs << "ms [DIST] gnbNode=" << gnbNodes.Get(i)->GetId()
                            << " ueNode=" << ueNodes.Get(0)->GetId() << " distToUeM=" << distToUeM
                            << std::endl;
    }

    if (Simulator::Now().GetSeconds() + logPeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(logPeriodSec),
                            &LogPositionsAndDistances,
                            ueNodes,
                            gnbNodes,
                            logPeriodSec,
                            simTimeSec);
    }
}

void
UpdateLeoGnbPositions(NodeContainer gnbNodes,
                      const std::vector<double>& initialLongitudesDeg,
                      const std::vector<double>& initialLatitudesDeg,
                      double leoAltitudeM,
                      double leoSpeedMps,
                      double updatePeriodSec,
                      double simTimeSec)
{
    const double earthRadiusM = 6371000.0;
    const double orbitRadiusM = earthRadiusM + leoAltitudeM;
    const double deltaLongitudeDeg =
        (leoSpeedMps * Simulator::Now().GetSeconds() / orbitRadiusM) * 180.0 / M_PI;

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> gnbMobility =
            gnbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        NS_ASSERT_MSG(gnbMobility, "Geocentric mobility model not found on gNB node");
        gnbMobility->SetGeographicPosition(Vector(initialLatitudesDeg.at(i),
                                                  initialLongitudesDeg.at(i) + deltaLongitudeDeg,
                                                  leoAltitudeM));
    }

    if (Simulator::Now().GetSeconds() + updatePeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(updatePeriodSec),
                            &UpdateLeoGnbPositions,
                            gnbNodes,
                            initialLongitudesDeg,
                            initialLatitudesDeg,
                            leoAltitudeM,
                            leoSpeedMps,
                            updatePeriodSec,
                            simTimeSec);
    }
}

void
UpdateNetAnimDepthCue(const NodeContainer& gnbNodes,
                      const NodeContainer& ueNodes,
                      AnimationInterface* anim,
                      double minSize,
                      double maxSize,
                      double updatePeriodSec,
                      double simTimeSec)
{
    if (!anim || ueNodes.GetN() == 0 || gnbNodes.GetN() == 0)
    {
        return;
    }

    Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    if (!ueMobility)
    {
        return;
    }

    std::vector<double> distances;
    distances.reserve(gnbNodes.GetN());
    double minDist = std::numeric_limits<double>::max();
    double maxDist = 0.0;

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> gnbMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
        if (!gnbMobility)
        {
            distances.push_back(0.0);
            continue;
        }

        const double dist = gnbMobility->GetDistanceFrom(ueMobility);
        distances.push_back(dist);
        minDist = std::min(minDist, dist);
        maxDist = std::max(maxDist, dist);
    }

    const double span = std::max(1.0, maxDist - minDist);
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        const double norm = (distances.at(i) - minDist) / span;
        const double size = maxSize - norm * (maxSize - minSize);
        anim->UpdateNodeSize(gnbNodes.Get(i), size, size);
    }

    if (Simulator::Now().GetSeconds() + updatePeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(updatePeriodSec),
                            &UpdateNetAnimDepthCue,
                            gnbNodes,
                            ueNodes,
                            anim,
                            minSize,
                            maxSize,
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

uint32_t
ParseNodeIdFromContext(const std::string& context)
{
    const std::string nodeIdText = ExtractBetween(context, "/NodeList/", "/");
    if (nodeIdText == "-")
    {
        return std::numeric_limits<uint32_t>::max();
    }

    std::istringstream iss(nodeIdText);
    uint32_t nodeId = std::numeric_limits<uint32_t>::max();
    iss >> nodeId;
    return iss.fail() ? std::numeric_limits<uint32_t>::max() : nodeId;
}

std::string
ControlMessageTypeToString(Ptr<const NrControlMessage> msg)
{
    if (!msg)
    {
        return "NULL";
    }

    switch (msg->GetMessageType())
    {
    case NrControlMessage::UL_DCI:
        return "UL_DCI";
    case NrControlMessage::DL_DCI:
        return "DL_DCI";
    case NrControlMessage::DL_CQI:
        return "DL_CQI";
    case NrControlMessage::MIB:
        return "MIB";
    case NrControlMessage::SIB1:
        return "SIB1";
    case NrControlMessage::RACH_PREAMBLE:
        return "RACH_PREAMBLE";
    case NrControlMessage::RAR:
        return "RAR";
    case NrControlMessage::BSR:
        return "BSR";
    case NrControlMessage::DL_HARQ:
        return "DL_HARQ";
    case NrControlMessage::SR:
        return "SR";
    case NrControlMessage::SRS:
        return "SRS";
    }
    return "UNKNOWN";
}

bool
IsChoRelevantControlMessage(Ptr<const NrControlMessage> msg)
{
    if (!msg)
    {
        return false;
    }

    switch (msg->GetMessageType())
    {
    case NrControlMessage::RACH_PREAMBLE:
        return gMacPhyIncludeRachPreamble;
    case NrControlMessage::RAR:
        return gMacPhyIncludeRar;
    case NrControlMessage::UL_DCI:
        return gMacPhyIncludeUlDci;
    case NrControlMessage::DL_DCI:
        return gMacPhyIncludeDlDci;
    case NrControlMessage::DL_HARQ:
        return gMacPhyIncludeDlHarq;
    case NrControlMessage::SR:
        return gMacPhyIncludeSr;
    default:
        return false;
    }
}

void
EmitMacPhyCtrlMessage(const std::string& context,
                      const std::string& layer,
                      const std::string& direction,
                      const SfnSf& sfnSf,
                      uint16_t nodeId,
                      uint16_t rnti,
                      uint8_t bwpId,
                      Ptr<const NrControlMessage> ctrlMessage)
{
    if (!gEnableMacPhyCtrlTrace)
    {
        return;
    }

    const int64_t nowMs = static_cast<int64_t>(Simulator::Now().GetMilliSeconds());
    if (gMacPhyTraceStartMs >= 0 && nowMs < gMacPhyTraceStartMs)
    {
        return;
    }
    if (gMacPhyTraceEndMs >= 0 && nowMs > gMacPhyTraceEndMs)
    {
        return;
    }

    if (!IsChoRelevantControlMessage(ctrlMessage))
    {
        return;
    }

    choDetailLog << Simulator::Now().GetMilliSeconds()
                 << "ms [CHO-DETAIL] step=Step5_MacPhyCtrlMsg"
                 << " layer=" << layer
                 << " direction=" << direction
                 << " context=" << CompactContextId(context)
                 << " nodeId=" << nodeId
                 << " UE-RNTI=" << rnti
                 << " bwpId=" << static_cast<uint16_t>(bwpId)
                 << " sfnSf=" << sfnSf
                 << " msgType=" << ControlMessageTypeToString(ctrlMessage)
                 << std::endl;

    if (gNetAnim && gNetAnimCtrlCounterEnabled)
    {
        const std::string msgType = ControlMessageTypeToString(ctrlMessage);
        const uint32_t ctxNodeId = ParseNodeIdFromContext(context);
        if (ctxNodeId != std::numeric_limits<uint32_t>::max())
        {
            if (direction == "TX" && gNetAnimCtrlTxCounterId != std::numeric_limits<uint32_t>::max())
            {
                const uint32_t value = ++gNodeCtrlTxCount[ctxNodeId];
                gNetAnim->UpdateNodeCounter(gNetAnimCtrlTxCounterId, ctxNodeId, value);
            }
            else if (direction == "RX" &&
                     gNetAnimCtrlRxCounterId != std::numeric_limits<uint32_t>::max())
            {
                const uint32_t value = ++gNodeCtrlRxCount[ctxNodeId];
                gNetAnim->UpdateNodeCounter(gNetAnimCtrlRxCounterId, ctxNodeId, value);
            }

            if (gNetAnimMsgTypeCounterEnabled)
            {
                if (direction == "TX")
                {
                    auto it = gNetAnimMsgTypeTxCounterId.find(msgType);
                    if (it != gNetAnimMsgTypeTxCounterId.end())
                    {
                        const uint32_t value = ++gNodeMsgTypeTxCount[ctxNodeId][msgType];
                        gNetAnim->UpdateNodeCounter(it->second, ctxNodeId, value);
                    }
                }
                else if (direction == "RX")
                {
                    auto it = gNetAnimMsgTypeRxCounterId.find(msgType);
                    if (it != gNetAnimMsgTypeRxCounterId.end())
                    {
                        const uint32_t value = ++gNodeMsgTypeRxCount[ctxNodeId][msgType];
                        gNetAnim->UpdateNodeCounter(it->second, ctxNodeId, value);
                    }
                }
            }
        }
    }
}

double
LinearToDb(double linearValue)
{
    if (linearValue <= 0.0)
    {
        return -std::numeric_limits<double>::infinity();
    }
    return 10.0 * std::log10(linearValue);
}

void
EmitSinrLog(const std::string& context,
            const std::string& sinrType,
            uint16_t cellId,
            uint16_t rnti,
            double sinrLinear,
            uint16_t bwpId)
{
    if (!gEnableSinrTrace)
    {
        return;
    }

    const int64_t nowMs = static_cast<int64_t>(Simulator::Now().GetMilliSeconds());
    if (gSinrTraceStartMs >= 0 && nowMs < gSinrTraceStartMs)
    {
        return;
    }
    if (gSinrTraceEndMs >= 0 && nowMs > gSinrTraceEndMs)
    {
        return;
    }

    sinrLog << nowMs << "ms [SINR]"
            << " type=" << sinrType
            << " context=" << CompactContextId(context)
            << " cellId=" << cellId
            << " UE-RNTI=" << rnti
            << " bwpId=" << bwpId
            << " linear=" << sinrLinear
            << " dB=" << LinearToDb(sinrLinear) << std::endl;
}

void
NotifyUeDlDataSinr(std::string context,
                   uint16_t cellId,
                   uint16_t rnti,
                   double sinr,
                   uint16_t bwpId)
{
    EmitSinrLog(context, "DL_DATA", cellId, rnti, sinr, bwpId);
}

void
NotifyUeDlCtrlSinr(std::string context,
                   uint16_t cellId,
                   uint16_t rnti,
                   double sinr,
                   uint16_t bwpId)
{
    EmitSinrLog(context, "DL_CTRL", cellId, rnti, sinr, bwpId);
}

void
NotifyGnbPhyRxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "GNB-PHY", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyGnbPhyTxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "GNB-PHY", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyGnbMacRxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "GNB-MAC", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyGnbMacTxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "GNB-MAC", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyUePhyRxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "UE-PHY", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyUePhyTxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "UE-PHY", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyUeMacRxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "UE-MAC", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyUeMacTxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitMacPhyCtrlMessage(context, "UE-MAC", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

void
NotifyConnectionEstablishedUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [UE-CONN] [" << CompactContextId(context)
                  << "] IMSI=" << imsi << " Cell=" << cellid << " RNTI=" << rnti << std::endl;
}

void
NotifyConnectionEstablishedGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [GNB-CONN] [" << CompactContextId(context)
                  << "] IMSI=" << imsi << " Cell=" << cellid << " RNTI=" << rnti << std::endl;
}

void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellid,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [UE-HO-START] ["
                  << CompactContextId(context) << "] IMSI=" << imsi << " srcCell=" << cellid
                  << " tgtCell=" << targetCellId << " RNTI=" << rnti << std::endl;

    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [CHO-EVENT] ["
                  << CompactContextId(context)
                  << "] TriggerHandoverCalled actor=UE action=PROCESS io=internal"
                  << " message=MeasurementReport/A3 UE-RNTI=" << rnti << " tgtCell=" << targetCellId
                  << std::endl;

    choDetailLog << Simulator::Now().GetMilliSeconds()
                 << "ms [CHO-DETAIL] step=Step5_CfraStart UE-RNTI=" << rnti
                 << " actor=UE action=PROCESS stage=ContentionFreeRandomAccess"
                 << " trigger=RrcReconfiguration(CHO-Execute)"
                 << " sourceCellId=" << cellid << " targetCellId=" << targetCellId << std::endl;

    choDetailLog << Simulator::Now().GetMilliSeconds()
                 << "ms [CHO-DETAIL] step=Step5_CfraPreambleTxByUe UE-RNTI=" << rnti
                 << " actor=UE action=TX message=RACH-Preamble(CFRA)"
                 << " targetCellId=" << targetCellId << std::endl;
}

void
NotifyHandoverStartGnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [GNB-HO-START] ["
                  << CompactContextId(context) << "] IMSI=" << imsi << " srcCell=" << cellid
                  << " tgtCell=" << targetCellId << " RNTI=" << rnti << std::endl;
}

void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [UE-HO-END] [" << CompactContextId(context)
                  << "] IMSI=" << imsi << " newCell=" << cellid << " RNTI=" << rnti << std::endl;

    choDetailLog << Simulator::Now().GetMilliSeconds()
                 << "ms [CHO-DETAIL] step=Step5_CfraCompleted UE-RNTI=" << rnti
                 << " actor=UE action=PROCESS stage=ContentionFreeRandomAccess"
                 << " result=Success targetCellId=" << cellid << std::endl;
}

void
NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [GNB-HO-END] [" << CompactContextId(context)
                  << "] IMSI=" << imsi << " newCell=" << cellid << " RNTI=" << rnti << std::endl;
}

void
NotifyConnectionTimeoutUe(std::string context,
                          uint64_t imsi,
                          uint16_t cellId,
                          uint16_t rnti,
                          uint8_t connEstFailCount)
{
    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [UE-CONN-TIMEOUT] ["
                  << CompactContextId(context) << "] IMSI=" << imsi << " Cell=" << cellId
                  << " RNTI=" << rnti
                  << " connEstFailCount=" << static_cast<uint16_t>(connEstFailCount) << std::endl;
}

void
NotifyMeasurementReport(std::string context,
                        uint64_t imsi,
                        uint16_t cellId,
                        uint16_t rnti,
                        NrRrcSap::MeasurementReport report)
{
    uint16_t bestCell = 0;
    uint8_t bestRsrp = 0;

    const auto& results = report.measResults;
    const uint8_t servingRsrp = results.measResultPCell.rsrpResult;
    if (results.haveMeasResultNeighCells)
    {
        for (const auto& n : results.measResultListEutra)
        {
            if (n.haveRsrpResult && n.rsrpResult >= bestRsrp)
            {
                bestRsrp = n.rsrpResult;
                bestCell = n.physCellId;
            }
        }
    }

    int32_t rsrpDelta = static_cast<int32_t>(bestRsrp) - static_cast<int32_t>(servingRsrp);

    choSummaryLog << Simulator::Now().GetMilliSeconds() << "ms [CHO-EVENT] [" << CompactContextId(context)
                  << "] A3ConditionSatisfied actor=Source-gNB action=RX io=in"
                  << " message=MeasurementReport/A3"
                  << " IMSI=" << imsi << " srcCell=" << cellId << " UE-RNTI=" << rnti
                  << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
                  << " tgtCell=" << bestCell << " tgtRsrp=" << static_cast<uint16_t>(bestRsrp)
                  << " deltaRsrp=" << rsrpDelta
                  << std::endl;
}

void
NotifyNrChoEvent(uint64_t timestampMs,
                 uint16_t rnti,
                 std::string eventName,
                 uint16_t targetCellId,
                 uint8_t servingRsrp,
                 uint8_t targetRsrp,
                 uint16_t candidateCount)
{
    choSummaryLog << timestampMs << "ms [CHO-EVENT]"
                  << " event=" << eventName << " UE-RNTI=" << rnti
                  << " tgtCell=" << targetCellId
                  << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
                  << " targetRsrp=" << static_cast<uint16_t>(targetRsrp)
                  << " candidateCount=" << candidateCount << std::endl;
}

void
NotifyNrChoDetailedEvent(uint64_t timestampMs,
                         uint16_t rnti,
                         std::string stepName,
                         std::string detail)
{
    choDetailLog << timestampMs << "ms [CHO-DETAIL]"
                 << " step=" << stepName << " UE-RNTI=" << rnti << " " << detail
                 << std::endl;
}

int
main(int argc, char* argv[])
{
    choSummaryLog.open("cho-summary-nr.log");
    choDetailLog.open("cho-detail-nr.log");
    positionDistanceLog.open("position-distance-nr.log");
    x2DelayLog.open("x2-delay-nr.log");
    sinrLog.open("sinr-nr.log");

    uint16_t numberOfUes = 1;
    uint16_t numberOfGnbs = 7;
    int32_t initialAttachGnbIndex = 0;
    double simTime = 95.0;
    bool useIdealRrc = true;

    double gnbTxPowerDbm = 46.0;
    double ueTxPowerDbm = 23.0;
    double gnbAntennaGainDb = 30.0;
    double ueAntennaGainDb = 30.0;
    bool useSatAntennaPattern = true;
    double satApertureRadiusInWavelengths = 10.0;
    double satAntennaMinGainDb = -100.0;

    double carrierFrequencyHz = 2.0e9;
    double bandwidthHz = 20e6;
    bool shadowingEnabled = false;

    double leoAltitudeM = 600000.0;
    double leoSpeedMps = 7560.0;
    double orbitLatitudeDeg = 0.0;
    double gnbLatitudeStepDeg = 0.25;
    double ueLatitudeDeg = 0.0;
    double ueLongitudeDeg = 0.0;
    double gnbLongitudeStepDeg = 3.0;
    double positionUpdatePeriodMs = 100.0;
    double positionLogPeriodMs = 100.0;

    bool enableA3Handover = true;
    bool enableChoExecution = true;
    double hoHysteresisDb = 1.0;
    uint16_t hoTttMs = 0;
    uint16_t gnbHandoverJoiningTimeoutMs = 1500;
    bool scheduleManualHandover = false;
    double manualHandoverTimeSec = 2.0;
    bool enableX2Interface = true;
    bool enableMacPhyCtrlTrace = true;
    int64_t macPhyTraceStartMs = -1;
    int64_t macPhyTraceEndMs = -1;
    bool macPhyIncludeRachPreamble = true;
    bool macPhyIncludeRar = true;
    bool macPhyIncludeUlDci = false;
    bool macPhyIncludeDlDci = true;
    bool macPhyIncludeDlHarq = false;
    bool macPhyIncludeSr = false;
    bool enableSinrTrace = true;
    int64_t sinrTraceStartMs = -1;
    int64_t sinrTraceEndMs = -1;
    bool enableNetAnim = false;
    std::string netAnimFile = "ntn-topology-animation.xml";
    double netAnimStartMs = 0.0;
    double netAnimStopMs = -1.0;
    double netAnimMobilityPollMs = 100.0;
    bool netAnimAutoStyle = true;
    double netAnimGnbSize = 22.0;
    double netAnimUeSize = 30.0;
    double netAnimCoreSize = 14.0;
    bool netAnimEnablePacketMetadata = false;
    bool netAnimEnableCtrlCounters = true;
    bool netAnimEnableMsgTypeCounters = true;
    bool netAnimEnableDepthCue = true;
    double netAnimDepthCueMinSize = 16.0;
    double netAnimDepthCueMaxSize = 34.0;
    bool netAnimShowRoutePaths = false;
    bool netAnimShowX2LinkLabels = false;
    bool netAnimGenerateCompactXml = true;
    std::string netAnimCompactFile = "ntn-topology-animation-compact.xml";
    double netAnimCompactHalfSpan = 1200.0;

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Total duration of the simulation (in seconds)", simTime);
    cmd.AddValue("numberOfUes", "Number of UEs", numberOfUes);
    cmd.AddValue("numberOfGnbs", "Number of gNBs", numberOfGnbs);
    cmd.AddValue("initialAttachGnbIndex",
                 "Initial attach gNB index [0..numberOfGnbs-1], -1 means closest gNB",
                 initialAttachGnbIndex);
    cmd.AddValue("useIdealRrc", "Use idealized RRC signaling", useIdealRrc);
    cmd.AddValue("gnbTxPowerDbm", "gNB Tx power [dBm]", gnbTxPowerDbm);
    cmd.AddValue("ueTxPowerDbm", "UE Tx power [dBm]", ueTxPowerDbm);
    cmd.AddValue("gnbAntennaGainDb", "gNB antenna gain [dB]", gnbAntennaGainDb);
    cmd.AddValue("ueAntennaGainDb", "UE antenna gain [dB]", ueAntennaGainDb);
    cmd.AddValue("useSatAntennaPattern",
                 "Use 3GPP 38.811 circular-aperture satellite antenna pattern on gNB",
                 useSatAntennaPattern);
    cmd.AddValue("satApertureRadiusInWavelengths",
                 "Satellite antenna aperture radius [wavelengths], e.g., 10 for TR 38.811 Fig 6.4.1.1",
                 satApertureRadiusInWavelengths);
    cmd.AddValue("satAntennaMinGainDb",
                 "Minimum gain floor [dB] for circular-aperture antenna model",
                 satAntennaMinGainDb);
    cmd.AddValue("carrierFrequencyHz", "Carrier frequency [Hz]", carrierFrequencyHz);
    cmd.AddValue("bandwidthHz", "Carrier bandwidth [Hz]", bandwidthHz);
    cmd.AddValue("shadowingEnabled", "Enable channel shadowing", shadowingEnabled);
    cmd.AddValue("leoAltitudeM", "LEO satellite altitude in meters", leoAltitudeM);
    cmd.AddValue("leoSpeedMps", "LEO satellite speed in m/s", leoSpeedMps);
    cmd.AddValue("orbitLatitudeDeg", "Satellite orbit latitude in degrees", orbitLatitudeDeg);
    cmd.AddValue("gnbLatitudeStepDeg",
                 "Inter-satellite latitude spacing in degrees (visual separation)",
                 gnbLatitudeStepDeg);
    cmd.AddValue("ueLatitudeDeg", "UE latitude in degrees", ueLatitudeDeg);
    cmd.AddValue("ueLongitudeDeg", "UE longitude in degrees", ueLongitudeDeg);
    cmd.AddValue("gnbLongitudeStepDeg",
                 "Initial inter-satellite longitude spacing in degrees",
                 gnbLongitudeStepDeg);
    cmd.AddValue("positionUpdatePeriodMs", "Satellite position update period in ms", positionUpdatePeriodMs);
    cmd.AddValue("positionLogPeriodMs", "Position/distance log period in ms", positionLogPeriodMs);
    cmd.AddValue("enableA3Handover", "Enable NR A3-based automatic handover", enableA3Handover);
    cmd.AddValue("enableChoExecution",
                 "If true, execute TriggerHandover in NR conditional handover algorithm",
                 enableChoExecution);
    cmd.AddValue("hoHysteresisDb", "A3 HO hysteresis in dB", hoHysteresisDb);
    cmd.AddValue("hoTttMs", "A3 HO time-to-trigger in ms", hoTttMs);
    cmd.AddValue("gnbHandoverJoiningTimeoutMs", "gNB HO joining timeout in ms", gnbHandoverJoiningTimeoutMs);
    cmd.AddValue("scheduleManualHandover",
                 "Schedule one manual HO request as a fallback path",
                 scheduleManualHandover);
    cmd.AddValue("manualHandoverTimeSec", "Manual HO trigger time in seconds", manualHandoverTimeSec);
    cmd.AddValue("enableX2Interface",
                 "Enable X2 interface setup among gNBs",
                 enableX2Interface);
    cmd.AddValue("enableMacPhyCtrlTrace",
                 "Enable MAC/PHY control-message logging in cho-detail-nr.log",
                 enableMacPhyCtrlTrace);
    cmd.AddValue("macPhyTraceStartMs",
                 "MAC/PHY trace start time [ms], -1 means from beginning",
                 macPhyTraceStartMs);
    cmd.AddValue("macPhyTraceEndMs",
                 "MAC/PHY trace end time [ms], -1 means until simulation end",
                 macPhyTraceEndMs);
    cmd.AddValue("macPhyIncludeRachPreamble",
                 "Include RACH_PREAMBLE in MAC/PHY trace",
                 macPhyIncludeRachPreamble);
    cmd.AddValue("macPhyIncludeRar", "Include RAR in MAC/PHY trace", macPhyIncludeRar);
    cmd.AddValue("macPhyIncludeUlDci", "Include UL_DCI in MAC/PHY trace", macPhyIncludeUlDci);
    cmd.AddValue("macPhyIncludeDlDci", "Include DL_DCI in MAC/PHY trace", macPhyIncludeDlDci);
    cmd.AddValue("macPhyIncludeDlHarq",
                 "Include DL_HARQ in MAC/PHY trace",
                 macPhyIncludeDlHarq);
    cmd.AddValue("macPhyIncludeSr", "Include SR in MAC/PHY trace", macPhyIncludeSr);
    cmd.AddValue("enableSinrTrace",
                 "Enable SINR logging in sinr-nr.log",
                 enableSinrTrace);
    cmd.AddValue("sinrTraceStartMs",
                 "SINR trace start time [ms], -1 means from beginning",
                 sinrTraceStartMs);
    cmd.AddValue("sinrTraceEndMs",
                 "SINR trace end time [ms], -1 means until simulation end",
                 sinrTraceEndMs);
    cmd.AddValue("enableNetAnim", "Enable NetAnim XML output", enableNetAnim);
    cmd.AddValue("netAnimFile", "NetAnim output XML file", netAnimFile);
    cmd.AddValue("netAnimStartMs", "NetAnim capture start time [ms]", netAnimStartMs);
    cmd.AddValue("netAnimStopMs",
                 "NetAnim capture stop time [ms], -1 means simulation end",
                 netAnimStopMs);
    cmd.AddValue("netAnimMobilityPollMs",
                 "NetAnim mobility polling period [ms]",
                 netAnimMobilityPollMs);
    cmd.AddValue("netAnimAutoStyle",
                 "Apply default style (color/size/label) to improve readability",
                 netAnimAutoStyle);
    cmd.AddValue("netAnimGnbSize", "NetAnim gNB node size", netAnimGnbSize);
    cmd.AddValue("netAnimUeSize", "NetAnim UE node size", netAnimUeSize);
    cmd.AddValue("netAnimCoreSize", "NetAnim core-network node size", netAnimCoreSize);
    cmd.AddValue("netAnimEnablePacketMetadata",
                 "Enable packet metadata in NetAnim for richer message-flow inspection",
                 netAnimEnablePacketMetadata);
    cmd.AddValue("netAnimEnableCtrlCounters",
                 "Show per-node NR control-message TX/RX counters in NetAnim",
                 netAnimEnableCtrlCounters);
    cmd.AddValue("netAnimEnableMsgTypeCounters",
                 "Show per-message-type NR control-message counters in NetAnim",
                 netAnimEnableMsgTypeCounters);
    cmd.AddValue("netAnimEnableDepthCue",
                 "Pseudo-3D depth cue by dynamically resizing gNB nodes by UE distance",
                 netAnimEnableDepthCue);
    cmd.AddValue("netAnimDepthCueMinSize", "Minimum gNB size for depth cue", netAnimDepthCueMinSize);
    cmd.AddValue("netAnimDepthCueMaxSize", "Maximum gNB size for depth cue", netAnimDepthCueMaxSize);
    cmd.AddValue("netAnimShowRoutePaths",
                 "Show route-path text overlays in NetAnim (can clutter view)",
                 netAnimShowRoutePaths);
    cmd.AddValue("netAnimShowX2LinkLabels",
                 "Show X2 link distance/delay labels in NetAnim",
                 netAnimShowX2LinkLabels);
    cmd.AddValue("netAnimGenerateCompactXml",
                 "Generate compact/normalized NetAnim XML for better fit",
                 netAnimGenerateCompactXml);
    cmd.AddValue("netAnimCompactFile", "Output file for compact NetAnim XML", netAnimCompactFile);
    cmd.AddValue("netAnimCompactHalfSpan",
                 "Half-span for compact NetAnim coordinates",
                 netAnimCompactHalfSpan);
    cmd.Parse(argc, argv);

    gEnableMacPhyCtrlTrace = enableMacPhyCtrlTrace;
    gMacPhyTraceStartMs = macPhyTraceStartMs;
    gMacPhyTraceEndMs = macPhyTraceEndMs;
    gMacPhyIncludeRachPreamble = macPhyIncludeRachPreamble;
    gMacPhyIncludeRar = macPhyIncludeRar;
    gMacPhyIncludeUlDci = macPhyIncludeUlDci;
    gMacPhyIncludeDlDci = macPhyIncludeDlDci;
    gMacPhyIncludeDlHarq = macPhyIncludeDlHarq;
    gMacPhyIncludeSr = macPhyIncludeSr;
    gEnableSinrTrace = enableSinrTrace;
    gSinrTraceStartMs = sinrTraceStartMs;
    gSinrTraceEndMs = sinrTraceEndMs;

    if (numberOfGnbs < 2)
    {
        NS_FATAL_ERROR("numberOfGnbs must be >= 2 for handover");
    }

    Config::SetDefault("ns3::NrGnbRrc::HandoverJoiningTimeoutDuration",
                       TimeValue(MilliSeconds(gnbHandoverJoiningTimeoutMs)));

    NodeContainer ueNodes;
    NodeContainer gnbNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);
    gGnbNodes = gnbNodes;
    gUeNodes = ueNodes;

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    gnbMobility.Install(gnbNodes);

    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    ueMobility.Install(ueNodes);

    const Vector referencePoint(ueLatitudeDeg, ueLongitudeDeg, 0.0);
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> gnbGeoMobility =
            gnbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        NS_ASSERT_MSG(gnbGeoMobility, "Geocentric mobility model not found on gNB node");
        gnbGeoMobility->SetCoordinateTranslationReferencePoint(referencePoint);
    }
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> ueGeoMobility =
            ueNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        NS_ASSERT_MSG(ueGeoMobility, "Geocentric mobility model not found on UE node");
        ueGeoMobility->SetCoordinateTranslationReferencePoint(referencePoint);
        ueGeoMobility->SetGeographicPosition(Vector(ueLatitudeDeg, ueLongitudeDeg, 0.0));
    }

    std::vector<double> gnbInitialLongitudesDeg(numberOfGnbs, ueLongitudeDeg);
    std::vector<double> gnbInitialLatitudesDeg(numberOfGnbs, orbitLatitudeDeg);
    const double center = (static_cast<double>(numberOfGnbs) - 1.0) / 2.0;
    for (uint16_t i = 0; i < numberOfGnbs; ++i)
    {
        gnbInitialLongitudesDeg[i] = ueLongitudeDeg + (static_cast<double>(i) - center) * gnbLongitudeStepDeg;
        gnbInitialLatitudesDeg[i] = orbitLatitudeDeg + (static_cast<double>(i) - center) * gnbLatitudeStepDeg;
    }

    UpdateLeoGnbPositions(gnbNodes,
                          gnbInitialLongitudesDeg,
                          gnbInitialLatitudesDeg,
                          leoAltitudeM,
                          leoSpeedMps,
                          positionUpdatePeriodMs / 1000.0,
                          simTime);

    choSummaryLog << "0ms [TOPOLOGY] mode=ntn-geocentric"
                  << " orbitLatitudeDeg=" << orbitLatitudeDeg
                  << " ueLatitudeDeg=" << ueLatitudeDeg
                  << " ueLongitudeDeg=" << ueLongitudeDeg
                  << " leoAltitudeM=" << leoAltitudeM
                  << " leoSpeedMps=" << leoSpeedMps
                  << " gnbLatitudeStepDeg=" << gnbLatitudeStepDeg
                  << " gnbLongitudeStepDeg=" << gnbLongitudeStepDeg << std::endl;

    choSummaryLog << "0ms [MACPHY-TRACE-CONFIG]"
                  << " enabled=" << (gEnableMacPhyCtrlTrace ? "true" : "false")
                  << " startMs=" << gMacPhyTraceStartMs
                  << " endMs=" << gMacPhyTraceEndMs
                  << " includeRach=" << (gMacPhyIncludeRachPreamble ? "true" : "false")
                  << " includeRar=" << (gMacPhyIncludeRar ? "true" : "false")
                  << " includeUlDci=" << (gMacPhyIncludeUlDci ? "true" : "false")
                  << " includeDlDci=" << (gMacPhyIncludeDlDci ? "true" : "false")
                  << " includeDlHarq=" << (gMacPhyIncludeDlHarq ? "true" : "false")
                  << " includeSr=" << (gMacPhyIncludeSr ? "true" : "false")
                  << std::endl;

    choSummaryLog << "0ms [SINR-TRACE-CONFIG]"
                  << " enabled=" << (gEnableSinrTrace ? "true" : "false")
                  << " startMs=" << gSinrTraceStartMs
                  << " endMs=" << gSinrTraceEndMs << std::endl;

    choSummaryLog << "0ms [SAT-ANTENNA-CONFIG]"
                  << " useSatAntennaPattern=" << (useSatAntennaPattern ? "true" : "false")
                  << " apertureRadiusLambda=" << satApertureRadiusInWavelengths
                  << " maxGainDb=" << gnbAntennaGainDb
                  << " minGainDb=" << satAntennaMinGainDb << std::endl;

    choSummaryLog << "0ms [NETANIM-CONFIG]"
                  << " enabled=" << (enableNetAnim ? "true" : "false")
                  << " file=" << netAnimFile
                  << " startMs=" << netAnimStartMs
                  << " stopMs=" << netAnimStopMs
                  << " mobilityPollMs=" << netAnimMobilityPollMs
                  << " autoStyle=" << (netAnimAutoStyle ? "true" : "false")
                  << " gnbSize=" << netAnimGnbSize
                  << " ueSize=" << netAnimUeSize
                  << " coreSize=" << netAnimCoreSize
                  << " packetMetadata=" << (netAnimEnablePacketMetadata ? "true" : "false")
                  << " ctrlCounters=" << (netAnimEnableCtrlCounters ? "true" : "false")
                  << " msgTypeCounters=" << (netAnimEnableMsgTypeCounters ? "true" : "false")
                  << " depthCue=" << (netAnimEnableDepthCue ? "true" : "false")
                  << " depthCueMin=" << netAnimDepthCueMinSize
                  << " depthCueMax=" << netAnimDepthCueMaxSize
                  << " routePaths=" << (netAnimShowRoutePaths ? "true" : "false")
                  << " x2Labels=" << (netAnimShowX2LinkLabels ? "true" : "false")
                  << " compactXml=" << (netAnimGenerateCompactXml ? "true" : "false")
                  << " compactFile=" << netAnimCompactFile
                  << " compactHalfSpan=" << netAnimCompactHalfSpan << std::endl;

    LogPositionsAndDistances(ueNodes, gnbNodes, positionLogPeriodMs / 1000.0, simTime);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetBeamformingHelper(beamformingHelper);
    nrHelper->SetAttribute("UseIdealRrc", BooleanValue(useIdealRrc));
    if (enableA3Handover)
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrConditionalHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(hoHysteresisDb));
        nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(hoTttMs)));
        nrHelper->SetHandoverAlgorithmAttribute("EnableHandoverExecution",
                                                BooleanValue(enableChoExecution));
    }
    else
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrNoOpHandoverAlgorithm");
    }

    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled",
                       BooleanValue(shadowingEnabled));
    auto bandwidthAndBwp =
        nrHelper->CreateBandwidthParts({{carrierFrequencyHz, bandwidthHz, 1}}, "NTN-Rural");
    BandwidthPartInfoPtrVector allBwps = bandwidthAndBwp.second;

    Ptr<AntennaModel> gnbElement;
    if (useSatAntennaPattern)
    {
        constexpr double c = 299792458.0;
        const double apertureRadiusM = satApertureRadiusInWavelengths * (c / carrierFrequencyHz);
        Ptr<CircularApertureAntennaModel> satElement = CreateObject<CircularApertureAntennaModel>();
        satElement->SetAttribute("OperatingFrequency", DoubleValue(carrierFrequencyHz));
        satElement->SetAttribute("AntennaCircularApertureRadius", DoubleValue(apertureRadiusM));
        satElement->SetAttribute("AntennaMaxGainDb", DoubleValue(gnbAntennaGainDb));
        satElement->SetAttribute("AntennaMinGainDb", DoubleValue(satAntennaMinGainDb));
        gnbElement = satElement;
    }
    else
    {
        Ptr<IsotropicAntennaModel> isoElement = CreateObject<IsotropicAntennaModel>();
        isoElement->SetAttribute("Gain", DoubleValue(gnbAntennaGainDb));
        gnbElement = isoElement;
    }

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(gnbElement));

    Ptr<IsotropicAntennaModel> ueElement = CreateObject<IsotropicAntennaModel>();
    ueElement->SetAttribute("Gain", DoubleValue(ueAntennaGainDb));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueElement));

    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPowerDbm));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        NrHelper::GetGnbPhy(gnbDevs.Get(i), 0)->SetAttribute("Numerology", UintegerValue(0));
    }

    gCellIdToGnbNode.clear();
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        if (gnbDev)
        {
            gCellIdToGnbNode[gnbDev->GetCellId()] = gnbNodes.Get(i);
        }
    }

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);
    internet.Install(ueNodes);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(1500));
    p2ph.SetChannelAttribute("Delay", TimeValue(Seconds(0.010)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    (void) ueIpIfaces;

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ue = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ue->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedGnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionEstablished",
                    MakeCallback(&NotifyConnectionEstablishedUe));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartGnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverStart",
                    MakeCallback(&NotifyHandoverStartUe));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkUe));
    Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/ConnectionTimeout",
                    MakeCallback(&NotifyConnectionTimeoutUe));
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/RecvMeasurementReport",
                    MakeCallback(&NotifyMeasurementReport));

    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/GnbPhyRxedCtrlMsgsTrace",
                    MakeCallback(&NotifyGnbPhyRxedCtrlMessage));
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbPhy/GnbPhyTxedCtrlMsgsTrace",
                    MakeCallback(&NotifyGnbPhyTxedCtrlMessage));
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/GnbMacRxedCtrlMsgsTrace",
                    MakeCallback(&NotifyGnbMacRxedCtrlMessage));
    Config::Connect("/NodeList/*/DeviceList/*/BandwidthPartMap/*/NrGnbMac/GnbMacTxedCtrlMsgsTrace",
                    MakeCallback(&NotifyGnbMacTxedCtrlMessage));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyRxedCtrlMsgsTrace",
        MakeCallback(&NotifyUePhyRxedCtrlMessage));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/UePhyTxedCtrlMsgsTrace",
        MakeCallback(&NotifyUePhyTxedCtrlMessage));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacRxedCtrlMsgsTrace",
        MakeCallback(&NotifyUeMacRxedCtrlMessage));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUeMac/UeMacTxedCtrlMsgsTrace",
        MakeCallback(&NotifyUeMacTxedCtrlMessage));
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                    MakeCallback(&NotifyUeDlDataSinr));
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlCtrlSinr",
                    MakeCallback(&NotifyUeDlCtrlSinr));

    bool anyChoTraceConnected = false;
    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        if (!gnb)
        {
            continue;
        }
        Ptr<NrGnbRrc> rrc = gnb->GetRrc();
        if (!rrc)
        {
            continue;
        }
        Ptr<NrConditionalHandoverAlgorithm> choAlg = rrc->GetObject<NrConditionalHandoverAlgorithm>();
        if (!choAlg)
        {
            if (enableA3Handover)
            {
                choSummaryLog << "0ms [CHO-HOOK] cell=" << gnb->GetCellId()
                              << " status=not-found" << std::endl;
            }
            continue;
        }
        choAlg->TraceConnectWithoutContext("ChoEvent", MakeCallback(&NotifyNrChoEvent));
        choAlg->TraceConnectWithoutContext("ChoDetailedEvent", MakeCallback(&NotifyNrChoDetailedEvent));
        choSummaryLog << "0ms [CHO-HOOK] cell=" << gnb->GetCellId() << " status=connected"
                      << std::endl;
        anyChoTraceConnected = true;
    }
    if (enableA3Handover && !anyChoTraceConnected)
    {
        choSummaryLog << "0ms [CHO-HOOK] summary=no-conditional-handover-algorithm-connected"
                      << std::endl;
    }

    uint32_t attachGnbIndex = 0;
    if (initialAttachGnbIndex >= 0)
    {
        const uint32_t requestedIndex = static_cast<uint32_t>(initialAttachGnbIndex);
        if (requestedIndex >= gnbNodes.GetN())
        {
            NS_FATAL_ERROR("initialAttachGnbIndex is out of range");
        }
        attachGnbIndex = requestedIndex;
    }
    else
    {
        Ptr<MobilityModel> ueMobilityRef = ueNodes.Get(0)->GetObject<MobilityModel>();
        NS_ASSERT_MSG(ueMobilityRef, "Mobility model not found on UE node for initial attach");

        double minDistance = std::numeric_limits<double>::max();
        for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
        {
            Ptr<MobilityModel> gnbMobilityRef = gnbNodes.Get(i)->GetObject<MobilityModel>();
            NS_ASSERT_MSG(gnbMobilityRef, "Mobility model not found on gNB node for initial attach");
            const double distance = ueMobilityRef->GetDistanceFrom(gnbMobilityRef);
            if (distance < minDistance)
            {
                minDistance = distance;
                attachGnbIndex = i;
            }
        }
    }

    choSummaryLog << "0ms [INIT-ATTACH] selectedGnbIndex=" << attachGnbIndex << std::endl;

    for (uint16_t i = 0; i < numberOfUes; i++)
    {
        nrHelper->AttachToGnb(ueDevs.Get(i), gnbDevs.Get(attachGnbIndex));
    }

    std::vector<X2DynamicLink> x2Links;
    if (enableX2Interface)
    {
        x2Links.reserve(static_cast<size_t>(gnbNodes.GetN()) *
                        static_cast<size_t>(gnbNodes.GetN() - 1) /
                        2);

        for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
        {
            for (uint32_t j = i + 1; j < gnbNodes.GetN(); ++j)
            {
                Ptr<MobilityModel> srcMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
                Ptr<MobilityModel> dstMobility = gnbNodes.Get(j)->GetObject<MobilityModel>();
                NS_ASSERT_MSG(srcMobility, "Mobility model not found on X2 source gNB");
                NS_ASSERT_MSG(dstMobility, "Mobility model not found on X2 target gNB");

                const double distanceMeters = srcMobility->GetDistanceFrom(dstMobility);
                const Time x2Delay = ComputeLightSpeedDelay(distanceMeters);

                const uint32_t srcNDevicesBefore = gnbNodes.Get(i)->GetNDevices();
                const uint32_t dstNDevicesBefore = gnbNodes.Get(j)->GetNDevices();

                epcHelper->SetAttribute("X2LinkDelay", TimeValue(x2Delay));
                nrHelper->AddX2Interface(gnbNodes.Get(i), gnbNodes.Get(j));

                Ptr<PointToPointNetDevice> srcX2Dev =
                    DynamicCast<PointToPointNetDevice>(gnbNodes.Get(i)->GetDevice(srcNDevicesBefore));
                Ptr<PointToPointNetDevice> dstX2Dev =
                    DynamicCast<PointToPointNetDevice>(gnbNodes.Get(j)->GetDevice(dstNDevicesBefore));
                NS_ASSERT_MSG(srcX2Dev, "Failed to get source X2 point-to-point device");
                NS_ASSERT_MSG(dstX2Dev, "Failed to get destination X2 point-to-point device");

                Ptr<PointToPointChannel> x2Channel =
                    DynamicCast<PointToPointChannel>(srcX2Dev->GetChannel());
                NS_ASSERT_MSG(x2Channel, "Failed to get X2 point-to-point channel");
                NS_ASSERT_MSG(srcX2Dev->GetChannel() == dstX2Dev->GetChannel(),
                              "X2 endpoints are not attached to the same channel");

                x2Links.push_back({gnbNodes.Get(i), gnbNodes.Get(j), x2Channel});

                x2DelayLog << Simulator::Now().GetMilliSeconds()
                           << "ms [X2-DELAY-CONFIG] mode=pairwise-distance"
                           << " srcNode=" << gnbNodes.Get(i)->GetId()
                           << " dstNode=" << gnbNodes.Get(j)->GetId()
                           << " distanceM=" << distanceMeters
                           << " oneWayDelayMs=" << x2Delay.GetMilliSeconds()
                           << " oneWayDelayUs=" << x2Delay.GetNanoSeconds() / 1000.0
                           << " oneWayDelayNs=" << x2Delay.GetNanoSeconds() << std::endl;
            }
        }

        UpdateDynamicX2Delays(x2Links, positionUpdatePeriodMs / 1000.0, simTime);
    }
    else
    {
        x2DelayLog << Simulator::Now().GetMilliSeconds()
                   << "ms [X2-DELAY-CONFIG] mode=disabled-by-flag" << std::endl;
    }

    if (scheduleManualHandover && numberOfUes > 0 && numberOfGnbs > 1)
    {
        const uint32_t sourceIndex = attachGnbIndex;
        const uint32_t targetIndex = (attachGnbIndex + 1) % numberOfGnbs;
        nrHelper->HandoverRequest(Seconds(manualHandoverTimeSec),
                                  ueDevs.Get(0),
                                  gnbDevs.Get(sourceIndex),
                                  gnbDevs.Get(targetIndex));
        choSummaryLog << "0ms [MANUAL-HO] sourceIndex=" << sourceIndex
                      << " targetIndex=" << targetIndex
                      << " triggerSec=" << manualHandoverTimeSec << std::endl;
    }

    std::cout << "[NR-HO] algorithm="
              << (enableA3Handover ? "NrConditionalHandoverAlgorithm" : "NrNoOpHandoverAlgorithm")
              << " hysteresisDb=" << hoHysteresisDb << " tttMs=" << hoTttMs
              << " enableChoExecution=" << (enableChoExecution ? "true" : "false")
              << " enableX2Interface=" << (enableX2Interface ? "true" : "false")
              << " useIdealRrc=" << (useIdealRrc ? "true" : "false") << std::endl;

    std::unique_ptr<AnimationInterface> anim;
    if (enableNetAnim)
    {
        uint32_t stationaryIndex = 0;
        for (auto it = NodeList::Begin(); it != NodeList::End(); ++it)
        {
            Ptr<Node> node = *it;
            if (!node || node->GetObject<MobilityModel>())
            {
                continue;
            }
            const double x = -280000.0 + 80000.0 * static_cast<double>(stationaryIndex);
            const double y = -220000.0;
            AnimationInterface::SetConstantPosition(node, x, y, 0.0);
            ++stationaryIndex;
        }

        anim = std::make_unique<AnimationInterface>(netAnimFile);
        gNetAnim = anim.get();
        anim->SetStartTime(MilliSeconds(netAnimStartMs));
        const double effectiveStopMs = (netAnimStopMs < 0.0) ? simTime * 1000.0 : netAnimStopMs;
        anim->SetStopTime(MilliSeconds(effectiveStopMs));
        anim->SetMobilityPollInterval(MilliSeconds(netAnimMobilityPollMs));
        if (netAnimEnablePacketMetadata)
        {
            anim->EnablePacketMetadata(true);
        }

        if (netAnimEnableCtrlCounters)
        {
            gNetAnimCtrlCounterEnabled = true;
            gNodeCtrlTxCount.clear();
            gNodeCtrlRxCount.clear();
            gNetAnimCtrlTxCounterId =
                anim->AddNodeCounter("NR CtrlMsg TX", AnimationInterface::UINT32_COUNTER);
            gNetAnimCtrlRxCounterId =
                anim->AddNodeCounter("NR CtrlMsg RX", AnimationInterface::UINT32_COUNTER);
        }
        else
        {
            gNetAnimCtrlCounterEnabled = false;
            gNetAnimCtrlTxCounterId = std::numeric_limits<uint32_t>::max();
            gNetAnimCtrlRxCounterId = std::numeric_limits<uint32_t>::max();
        }

        if (netAnimEnableMsgTypeCounters)
        {
            gNetAnimMsgTypeCounterEnabled = true;
            gNetAnimMsgTypeTxCounterId.clear();
            gNetAnimMsgTypeRxCounterId.clear();
            gNodeMsgTypeTxCount.clear();
            gNodeMsgTypeRxCount.clear();

            for (const auto& type : GetEnabledControlMessageTypes())
            {
                gNetAnimMsgTypeTxCounterId[type] =
                    anim->AddNodeCounter("TX_" + type, AnimationInterface::UINT32_COUNTER);
                gNetAnimMsgTypeRxCounterId[type] =
                    anim->AddNodeCounter("RX_" + type, AnimationInterface::UINT32_COUNTER);
            }
        }
        else
        {
            gNetAnimMsgTypeCounterEnabled = false;
            gNetAnimMsgTypeTxCounterId.clear();
            gNetAnimMsgTypeRxCounterId.clear();
            gNodeMsgTypeTxCount.clear();
            gNodeMsgTypeRxCount.clear();
        }

        if (netAnimAutoStyle)
        {
            ApplyNetAnimVisualStyle(*anim,
                                    gnbNodes,
                                    ueNodes,
                                    pgw,
                                    remoteHost,
                                    netAnimGnbSize,
                                    netAnimUeSize,
                                    netAnimCoreSize);
        }

        if (netAnimShowX2LinkLabels)
        {
            for (const auto& link : x2Links)
            {
                Ptr<MobilityModel> srcMobility = link.srcNode->GetObject<MobilityModel>();
                Ptr<MobilityModel> dstMobility = link.dstNode->GetObject<MobilityModel>();
                if (!srcMobility || !dstMobility)
                {
                    continue;
                }
                const double distM = srcMobility->GetDistanceFrom(dstMobility);
                const double delayMs = ComputeLightSpeedDelay(distM).GetMilliSeconds();
                anim->UpdateLinkDescription(link.srcNode,
                                            link.dstNode,
                                            "X2 d=" + std::to_string(static_cast<uint64_t>(distM)) +
                                                "m / " +
                                                std::to_string(static_cast<uint64_t>(delayMs)) + "ms");
            }
        }

        if (netAnimShowRoutePaths && ueIpIfaces.GetN() > 0)
        {
            std::ostringstream ueIpText;
            ueIpText << ueIpIfaces.GetAddress(0);
            anim->AddSourceDestination(remoteHost->GetId(), ueIpText.str());
        }

        if (netAnimShowRoutePaths && internetIpIfaces.GetN() > 1)
        {
            std::ostringstream remoteIpText;
            remoteIpText << internetIpIfaces.GetAddress(1);
            anim->AddSourceDestination(ueNodes.Get(0)->GetId(), remoteIpText.str());
        }

        if (netAnimEnableDepthCue)
        {
            UpdateNetAnimDepthCue(gnbNodes,
                                  ueNodes,
                                  anim.get(),
                                  netAnimDepthCueMinSize,
                                  netAnimDepthCueMaxSize,
                                  positionUpdatePeriodMs / 1000.0,
                                  simTime);
        }
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    if (enableNetAnim && netAnimGenerateCompactXml)
    {
        GenerateCompactNetAnimXml(netAnimFile, netAnimCompactFile, netAnimCompactHalfSpan);
    }

    gNetAnim = nullptr;
    gNetAnimCtrlCounterEnabled = false;
    gNetAnimCtrlTxCounterId = std::numeric_limits<uint32_t>::max();
    gNetAnimCtrlRxCounterId = std::numeric_limits<uint32_t>::max();
    gNetAnimMsgTypeCounterEnabled = false;
    gNetAnimMsgTypeTxCounterId.clear();
    gNetAnimMsgTypeRxCounterId.clear();
    gNodeMsgTypeTxCount.clear();
    gNodeMsgTypeRxCount.clear();

    choSummaryLog.close();
    choDetailLog.close();
    positionDistanceLog.close();
    x2DelayLog.close();
    sinrLog.close();

    return 0;
}
