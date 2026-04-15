#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-conditional-handover-algorithm.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/propagation-module.h"
#include "ns3/spectrum-value.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrX2ConditionalHandoverTest");

std::ofstream gMetricLog;
std::ofstream gSinrBreakdownLog;
std::ofstream gCellBudgetLog;
std::ofstream gCtrlMsgLog;
std::ofstream gGeometryLog;
std::ofstream gHoLog;
std::ofstream gUeMeasurementLog;
std::map<uint16_t, Vector> gCellToPosition;
std::map<uint16_t, double> gCellToOffAxisDeg;
std::map<uint16_t, double> gCellToSatPatternGainDb;
std::map<uint32_t, double> gTxNodeToSatPatternGainDb;
Vector gUePosition;
bool gEnableMetricLog = true;
int64_t gMetricStartMs = -1;
int64_t gMetricEndMs = -1;
bool gSnapshotOnce = true;
bool gPrintedRsrp = false;
bool gPrintedSinrCtrl = false;
bool gPrintedSinrData = false;
Ptr<SpectrumValue> gDlDataSignalPsd;
Ptr<SpectrumValue> gDlDataSinrPsd;
Ptr<SpectrumValue> gDlCtrlSignalPsd;
Ptr<SpectrumValue> gDlCtrlSinrPsd;
double gUeNoiseFigureDb = 5.0;

struct X2DynamicLink
{
    Ptr<Node> srcNode;
    Ptr<Node> dstNode;
    Ptr<PointToPointChannel> channel;
};

static double ComputeElevationDeg(const Vector& satPos, const Vector& uePos);
static double ComputeOffAxisDeg(const Vector& satPos, const Vector& uePos);
static double ComputeTr38811CircularApertureGainDb(double offAxisDeg,
                                                   double frequencyHz,
                                                   double apertureRadiusM,
                                                   double maxGainDb,
                                                   double minGainDb);
static Time ComputeLightSpeedDelay(double distanceMeters);
static void UpdateLeoGnbPositions(NodeContainer gnbNodes,
                                  const std::vector<double>& initialLongitudesDeg,
                                  const std::vector<double>& initialLatitudesDeg,
                                  double leoAltitudeM,
                                  double leoSpeedMps,
                                  double updatePeriodSec,
                                  double simTimeSec);
static void UpdateLiveCellGeometry(NodeContainer gnbNodes,
                                   NetDeviceContainer gnbDevs,
                                   double carrierFrequencyHz,
                                   double gnbTxPowerDbm,
                                   double gnbAntennaGainDb,
                                   double ueAntennaGainDb,
                                   bool useSatAntennaPattern,
                                   double satApertureRadiusM,
                                   double satAntennaMinGainDb,
                                   double updatePeriodSec,
                                   double simTimeSec);
static void UpdateDynamicX2Delays(const std::vector<X2DynamicLink>& x2Links,
                                  double updatePeriodSec,
                                  double simTimeSec);

static inline double
LinearToDb(double value)
{
    if (value <= 0.0)
    {
        return -std::numeric_limits<double>::infinity();
    }
    return 10.0 * std::log10(value);
}

static inline double
WattsToDbm(double valueW)
{
    if (valueW <= 0.0)
    {
        return -std::numeric_limits<double>::infinity();
    }
    return 10.0 * std::log10(valueW) + 30.0;
}

static inline double
ComputeFsplDb(double distanceM, double frequencyHz)
{
    constexpr double c = 299792458.0;
    const double argument = (4.0 * M_PI * distanceM * frequencyHz) / c;
    return 20.0 * std::log10(argument);
}

static Time
ComputeLightSpeedDelay(double distanceMeters)
{
    static constexpr double c = 299792458.0;
    return Seconds(distanceMeters / c);
}

static inline double
DecodeRsrpRangeToDbm(uint8_t rsrpRange)
{
    return nr::EutranMeasurementMapping::RsrpRange2Dbm(rsrpRange);
}

static inline double
DecodeRsrqRangeToDb(uint8_t rsrqRange)
{
    return nr::EutranMeasurementMapping::RsrqRange2Db(rsrqRange);
}

static void
UpdateDynamicX2Delays(const std::vector<X2DynamicLink>& x2Links,
                      double updatePeriodSec,
                      double simTimeSec)
{
    for (const auto& link : x2Links)
    {
        Ptr<MobilityModel> srcMobility = link.srcNode->GetObject<MobilityModel>();
        Ptr<MobilityModel> dstMobility = link.dstNode->GetObject<MobilityModel>();
        NS_ASSERT_MSG(srcMobility, "Mobility model not found on X2 source node");
        NS_ASSERT_MSG(dstMobility, "Mobility model not found on X2 destination node");
        NS_ASSERT_MSG(link.channel, "X2 point-to-point channel is null");

        const double distanceMeters = srcMobility->GetDistanceFrom(dstMobility);
        link.channel->SetAttribute("Delay", TimeValue(ComputeLightSpeedDelay(distanceMeters)));
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

static inline double
ComputeTr38811SlantRangeM(double elevationDeg, double altitudeM, double earthRadiusM)
{
    const double elevationRad = elevationDeg * M_PI / 180.0;
    const double sinElevation = std::sin(elevationRad);
    const double radicand =
        earthRadiusM * earthRadiusM * sinElevation * sinElevation + altitudeM * altitudeM +
        2.0 * altitudeM * earthRadiusM;
    return std::sqrt(radicand) - earthRadiusM * sinElevation;
}

static void
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

static void
UpdateLiveCellGeometry(NodeContainer gnbNodes,
                       NetDeviceContainer gnbDevs,
                       double carrierFrequencyHz,
                       double gnbTxPowerDbm,
                       double gnbAntennaGainDb,
                       double ueAntennaGainDb,
                       bool useSatAntennaPattern,
                       double satApertureRadiusM,
                       double satAntennaMinGainDb,
                       double updatePeriodSec,
                       double simTimeSec)
{
    uint16_t closestCellId = 0;
    double closestDistanceM = std::numeric_limits<double>::max();
    double closestElevationDeg = 0.0;
    uint16_t strongestEstimatedCellId = 0;
    double strongestEstimatedRxPowerDbmNoShadow = -std::numeric_limits<double>::infinity();
    double strongestEstimatedDistanceM = 0.0;
    double strongestEstimatedElevationDeg = 0.0;

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        Ptr<MobilityModel> gnbMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
        if (!gnbDev || !gnbMobility)
        {
            continue;
        }

        const uint16_t cellId = gnbDev->GetCellId();
        const Vector livePosition = gnbMobility->GetPosition();
        const double distanceM = CalculateDistance(livePosition, gUePosition);
        const double elevationDeg = ComputeElevationDeg(livePosition, gUePosition);
        constexpr double earthRadiusM = 6371e3;
        const double slantRangeM =
            ComputeTr38811SlantRangeM(elevationDeg, livePosition.z, earthRadiusM);
        const double offAxisDeg = ComputeOffAxisDeg(livePosition, gUePosition);
        const double satPatternGainDb =
            useSatAntennaPattern
                ? ComputeTr38811CircularApertureGainDb(offAxisDeg,
                                                       carrierFrequencyHz,
                                                       satApertureRadiusM,
                                                       gnbAntennaGainDb,
                                                       satAntennaMinGainDb)
                : gnbAntennaGainDb;
        const double fsplDb = ComputeFsplDb(slantRangeM, carrierFrequencyHz);
        const double estimatedRxPowerDbmNoShadow =
            gnbTxPowerDbm + satPatternGainDb + ueAntennaGainDb - fsplDb;

        gCellToPosition[cellId] = livePosition;
        gCellToOffAxisDeg[cellId] = offAxisDeg;
        gCellToSatPatternGainDb[cellId] = satPatternGainDb;
        gTxNodeToSatPatternGainDb[gnbNodes.Get(i)->GetId()] = satPatternGainDb;

        if (distanceM < closestDistanceM)
        {
            closestDistanceM = distanceM;
            closestCellId = cellId;
            closestElevationDeg = elevationDeg;
        }

        if (estimatedRxPowerDbmNoShadow > strongestEstimatedRxPowerDbmNoShadow)
        {
            strongestEstimatedRxPowerDbmNoShadow = estimatedRxPowerDbmNoShadow;
            strongestEstimatedCellId = cellId;
            strongestEstimatedDistanceM = distanceM;
            strongestEstimatedElevationDeg = elevationDeg;
        }
    }

    if (closestCellId != 0 || strongestEstimatedCellId != 0)
    {
        const Vector closestPosition = gCellToPosition.at(closestCellId);
        gGeometryLog << Simulator::Now().GetMilliSeconds()
                     << "ms [GEOMETRY] closestCellId=" << closestCellId
                     << " x=" << closestPosition.x
                     << " y=" << closestPosition.y
                     << " z=" << closestPosition.z
                     << " distanceM=" << closestDistanceM
                     << " elevationDeg=" << closestElevationDeg
                     << " offAxisDeg=" << gCellToOffAxisDeg.at(closestCellId)
                     << " satPatternGainDb=" << gCellToSatPatternGainDb.at(closestCellId)
                     << " strongestEstimatedCellId=" << strongestEstimatedCellId
                     << " strongestEstimatedRxPowerDbmNoShadow="
                     << strongestEstimatedRxPowerDbmNoShadow
                     << " strongestEstimatedDistanceM=" << strongestEstimatedDistanceM
                     << " strongestEstimatedElevationDeg=" << strongestEstimatedElevationDeg
                     << std::endl;
    }

    if (Simulator::Now().GetSeconds() + updatePeriodSec <= simTimeSec)
    {
        Simulator::Schedule(Seconds(updatePeriodSec),
                            &UpdateLiveCellGeometry,
                            gnbNodes,
                            gnbDevs,
                            carrierFrequencyHz,
                            gnbTxPowerDbm,
                            gnbAntennaGainDb,
                            ueAntennaGainDb,
                            useSatAntennaPattern,
                            satApertureRadiusM,
                            satAntennaMinGainDb,
                            updatePeriodSec,
                            simTimeSec);
    }
}

static inline double
QuantizeElevationDeg(double elevationDeg)
{
    return (elevationDeg < 10.0) ? 10.0 : std::round(elevationDeg / 10.0) * 10.0;
}

static inline double
ComputeScintillationLossDb(double frequencyHz, double elevationQuantizedDeg)
{
    if (frequencyHz < 6e9)
    {
        return 6.22 / std::pow(frequencyHz / 1e9, 1.5);
    }

    if (elevationQuantizedDeg <= 10.0)
    {
        return 1.08;
    }
    if (elevationQuantizedDeg <= 20.0)
    {
        return 0.63;
    }
    if (elevationQuantizedDeg <= 30.0)
    {
        return 0.46;
    }
    if (elevationQuantizedDeg <= 40.0)
    {
        return 0.37;
    }
    if (elevationQuantizedDeg <= 50.0)
    {
        return 0.31;
    }
    if (elevationQuantizedDeg <= 60.0)
    {
        return 0.26;
    }
    if (elevationQuantizedDeg <= 70.0)
    {
        return 0.22;
    }
    if (elevationQuantizedDeg <= 80.0)
    {
        return 0.19;
    }
    return 0.17;
}

static double
ComputeActiveBandwidthHz(const Ptr<const SpectrumValue>& activityMask)
{
    if (!activityMask)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double bandwidthHz = 0.0;
    auto bandIt = activityMask->ConstBandsBegin();
    auto maskIt = activityMask->ConstValuesBegin();
    for (; maskIt != activityMask->ConstValuesEnd(); ++maskIt, ++bandIt)
    {
        if (*maskIt > 0.0)
        {
            bandwidthHz += bandIt->fh - bandIt->fl;
        }
    }
    return bandwidthHz;
}

static inline double
DbmToWatts(double valueDbm)
{
    return std::pow(10.0, (valueDbm - 30.0) / 10.0);
}

static std::vector<std::string>
SplitCsvLine(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }
    return fields;
}

static void
WriteServingSinrSummaryCsv(const std::string& propagationCsvFile,
                           const std::string& servingSinrCsvFile,
                           uint32_t servingTxNodeId,
                           double bandwidthHz,
                           double noiseFigureDb,
                           double ueAntennaGainDb)
{
    std::ifstream input(propagationCsvFile);
    std::ofstream output(servingSinrCsvFile, std::ios::trunc);
    if (!input.is_open() || !output.is_open())
    {
        return;
    }

    std::string line;
    std::getline(input, line); // header

    bool foundServing = false;
    double servingRxPowerDbm = -std::numeric_limits<double>::infinity();
    double signalW = 0.0;
    double interferenceW = 0.0;

    output << "txNodeId,role,actualTxPowerDbm,propagationLossDb,satBeamGainDb,ueAntennaGainDb,"
              "beamGainDeltaFromServingDb,rxPowerWithAntennaDbm,rxPowerWithAntennaW"
           << std::endl;

    double servingSatBeamGainDb = 0.0;
    auto servingGainIt = gTxNodeToSatPatternGainDb.find(servingTxNodeId);
    if (servingGainIt != gTxNodeToSatPatternGainDb.end())
    {
        servingSatBeamGainDb = servingGainIt->second;
    }

    while (std::getline(input, line))
    {
        if (line.empty())
        {
            continue;
        }

        const auto fields = SplitCsvLine(line);
        if (fields.size() < 31)
        {
            continue;
        }

        const uint32_t txNodeId = static_cast<uint32_t>(std::stoul(fields[1]));
        const double propagationLossDb = std::stod(fields[22]);
        const double actualTxPowerDbm = std::stod(fields[29]);
        const double satBeamGainDb = gTxNodeToSatPatternGainDb.contains(txNodeId)
                                         ? gTxNodeToSatPatternGainDb.at(txNodeId)
                                         : 0.0;
        const double beamGainDeltaFromServingDb = satBeamGainDb - servingSatBeamGainDb;
        const double rxPowerWithAntennaDbm =
            actualTxPowerDbm + satBeamGainDb + ueAntennaGainDb - propagationLossDb;
        const double rxPowerW = DbmToWatts(rxPowerWithAntennaDbm);
        const char* role = (txNodeId == servingTxNodeId) ? "serving" : "interfering";

        output << txNodeId << ',' << role << ',' << actualTxPowerDbm << ',' << propagationLossDb
               << ',' << satBeamGainDb << ',' << ueAntennaGainDb << ','
               << beamGainDeltaFromServingDb << ',' << rxPowerWithAntennaDbm << ',' << rxPowerW
               << std::endl;

        if (txNodeId == servingTxNodeId)
        {
            foundServing = true;
            servingRxPowerDbm = rxPowerWithAntennaDbm;
            signalW += rxPowerW;
        }
        else
        {
            interferenceW += rxPowerW;
        }
    }

    const double thermalNoiseWPerHz = std::pow(10.0, (-174.0 - 30.0) / 10.0);
    const double noiseFigureLinear = std::pow(10.0, noiseFigureDb / 10.0);
    const double noiseW = thermalNoiseWPerHz * noiseFigureLinear * bandwidthHz;
    const double sinrLinear =
        foundServing ? (signalW / std::max(1e-30, interferenceW + noiseW)) : 0.0;
    const double sinrDb =
        foundServing ? LinearToDb(sinrLinear) : -std::numeric_limits<double>::infinity();

    output << std::endl;
    output << "servingTxNodeId,servingRxPowerDbm,interferencePowerDbm,noisePowerDbm,sinrDb,"
              "bandwidthHz,noiseFigureDb,servingSatBeamGainDb,ueAntennaGainDb"
           << std::endl;
    output << servingTxNodeId << ',' << servingRxPowerDbm << ',' << WattsToDbm(interferenceW)
           << ',' << WattsToDbm(noiseW) << ',' << sinrDb << ',' << bandwidthHz << ','
           << noiseFigureDb << ',' << servingSatBeamGainDb << ',' << ueAntennaGainDb
           << std::endl;
}

struct SnapshotUeRecord
{
    uint32_t ueIndex;
    uint32_t nominalGnbIndex;
    Vector localPosition;
    Ptr<MobilityModel> mobility;
};

static std::vector<SnapshotUeRecord>
CreateSnapshotUes(NodeContainer& snapshotUeNodes,
                  uint32_t uesPerCell,
                  double cellRadiusMeters,
                  const std::vector<Vector>& gnbPositions,
                  const Vector& referencePoint)
{
    std::vector<SnapshotUeRecord> records;
    if (uesPerCell == 0)
    {
        return records;
    }

    const uint32_t totalUes = static_cast<uint32_t>(gnbPositions.size()) * uesPerCell;
    snapshotUeNodes.Create(totalUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    mobility.Install(snapshotUeNodes);

    Ptr<UniformRandomVariable> angleRv = CreateObject<UniformRandomVariable>();
    angleRv->SetAttribute("Min", DoubleValue(0.0));
    angleRv->SetAttribute("Max", DoubleValue(2.0 * M_PI));
    Ptr<UniformRandomVariable> radiusRv = CreateObject<UniformRandomVariable>();
    radiusRv->SetAttribute("Min", DoubleValue(0.0));
    radiusRv->SetAttribute("Max", DoubleValue(1.0));

    const double earthRadiusM = 6371000.0;
    const double latRad = referencePoint.x * M_PI / 180.0;
    const double cosLat = std::max(1e-6, std::cos(latRad));

    records.reserve(totalUes);
    uint32_t ueIndex = 0;
    for (uint32_t gnbIndex = 0; gnbIndex < gnbPositions.size(); ++gnbIndex)
    {
        const Vector& center = gnbPositions.at(gnbIndex);
        for (uint32_t k = 0; k < uesPerCell; ++k, ++ueIndex)
        {
            const double angle = angleRv->GetValue();
            const double radius = cellRadiusMeters * std::sqrt(radiusRv->GetValue());
            const double localX = center.x + radius * std::cos(angle);
            const double localY = center.y + radius * std::sin(angle);
            const Vector localPosition(localX, localY, 0.0);

            auto ueGeo =
                snapshotUeNodes.Get(ueIndex)->GetObject<GeocentricConstantPositionMobilityModel>();
            ueGeo->SetCoordinateTranslationReferencePoint(referencePoint);
            ueGeo->SetGeographicPosition(
                Vector(referencePoint.x + (localY / earthRadiusM) * 180.0 / M_PI,
                       referencePoint.y + (localX / (earthRadiusM * cosLat)) * 180.0 / M_PI,
                       0.0));

            records.push_back({ueIndex, gnbIndex, localPosition, ueGeo});
        }
    }

    return records;
}

static void
WriteAllUeSinrSnapshotCsv(const std::string& outputCsvFile,
                         const std::vector<SnapshotUeRecord>& ueRecords,
                         const std::vector<Vector>& gnbPositions,
                         NodeContainer& gnbNodes,
                         double carrierFrequencyHz,
                         double gnbTxPowerDbm,
                         double gnbAntennaGainDb,
                         double ueAntennaGainDb,
                         bool useSatAntennaPattern,
                         double satApertureRadiusM,
                         double satAntennaMinGainDb,
                         double bandwidthHz,
                         double noiseFigureDb,
                         bool shadowingEnabled)
{
    std::ofstream output(outputCsvFile, std::ios::trunc);
    if (!output.is_open())
    {
        return;
    }

    Ptr<ThreeGppNTNRuralPropagationLossModel> lossModel =
        CreateObject<ThreeGppNTNRuralPropagationLossModel>();
    lossModel->SetFrequency(carrierFrequencyHz);
    lossModel->SetAttribute("ShadowingEnabled", BooleanValue(shadowingEnabled));
    lossModel->SetChannelConditionModel(CreateObject<ThreeGppNTNRuralChannelConditionModel>());

    const double thermalNoiseWPerHz = std::pow(10.0, (-174.0 - 30.0) / 10.0);
    const double noiseFigureLinear = std::pow(10.0, noiseFigureDb / 10.0);
    const double noiseW = thermalNoiseWPerHz * noiseFigureLinear * bandwidthHz;

    output << "ueIndex,nominalGnbIndex,ueX,ueY,servingGnbIndex,servingDistanceM,servingElevationDeg,"
              "servingOffAxisDeg,servingPropagationLossDb,servingDlCouplingLossDb,"
              "servingDlCouplingLossWithScintDb,"
              "servingPropagationRxDbm,servingSatBeamGainDb,ueAntennaGainDb,servingElevationQuantizedDeg,"
              "servingLosCond,servingRxPowerDbm,interferencePowerDbm,noisePowerDbm,sinrDb,"
              "bestInterfererGnbIndex,bestInterfererDistanceM,bestInterfererElevationDeg,"
              "bestInterfererOffAxisDeg,bestInterfererSatBeamGainDb,bestInterfererRxPowerDbm,servingMinusBestInterfererDb,"
              "servingMinusSumInterferenceDb,nominalServingRxPowerDbm,nominalMinusServingDb,"
              "nominalForcedServingGnbIndex,nominalForcedDistanceM,nominalForcedElevationDeg,"
              "nominalForcedOffAxisDeg,nominalForcedPropagationLossDb,nominalForcedDlCouplingLossDb,"
              "nominalForcedDlCouplingLossWithScintDb,nominalForcedSatBeamGainDb,nominalForcedRxPowerDbm,"
              "nominalForcedInterferencePowerDbm,nominalForcedSinrDb"
           << std::endl;

    for (const auto& ue : ueRecords)
    {
        double bestRxDbm = -std::numeric_limits<double>::infinity();
        double bestPropagationRxDbm = -std::numeric_limits<double>::infinity();
        double bestPropagationLossDb = 0.0;
        double bestSatBeamGainDb = 0.0;
        double bestDistanceM = 0.0;
        double bestElevationDeg = 0.0;
        double bestOffAxisDeg = 0.0;
        uint32_t bestGnbIndex = 0;
        double nominalServingRxDbm = -std::numeric_limits<double>::infinity();
        std::vector<double> perCellRxPowerW;
        std::vector<double> perCellRxPowerDbm;
        std::vector<double> perCellPropagationLossDb;
        std::vector<double> perCellSatBeamGainDb;
        perCellRxPowerW.reserve(gnbPositions.size());
        perCellRxPowerDbm.reserve(gnbPositions.size());
        perCellPropagationLossDb.reserve(gnbPositions.size());
        perCellSatBeamGainDb.reserve(gnbPositions.size());

        for (uint32_t gnbIndex = 0; gnbIndex < gnbPositions.size(); ++gnbIndex)
        {
            Ptr<MobilityModel> gnbMobility = gnbNodes.Get(gnbIndex)->GetObject<MobilityModel>();
            const double propagationRxDbm =
                lossModel->CalcRxPower(gnbTxPowerDbm, gnbMobility, ue.mobility);
            const double propagationLossDb = gnbTxPowerDbm - propagationRxDbm;
            const double offAxisDeg = ComputeOffAxisDeg(gnbPositions.at(gnbIndex), ue.localPosition);
            const double satBeamGainDb = useSatAntennaPattern
                                             ? ComputeTr38811CircularApertureGainDb(
                                                   offAxisDeg,
                                                   carrierFrequencyHz,
                                                   satApertureRadiusM,
                                                   gnbAntennaGainDb,
                                                   satAntennaMinGainDb)
                                             : gnbAntennaGainDb;
            const double rxPowerDbm = propagationRxDbm + satBeamGainDb + ueAntennaGainDb;
            perCellRxPowerW.push_back(DbmToWatts(rxPowerDbm));
            perCellRxPowerDbm.push_back(rxPowerDbm);
            perCellPropagationLossDb.push_back(propagationLossDb);
            perCellSatBeamGainDb.push_back(satBeamGainDb);

            if (gnbIndex == ue.nominalGnbIndex)
            {
                nominalServingRxDbm = rxPowerDbm;
            }

            if (rxPowerDbm > bestRxDbm)
            {
                bestRxDbm = rxPowerDbm;
                bestPropagationRxDbm = propagationRxDbm;
                bestPropagationLossDb = propagationLossDb;
                bestSatBeamGainDb = satBeamGainDb;
                bestDistanceM = CalculateDistance(gnbPositions.at(gnbIndex), ue.localPosition);
                bestElevationDeg = ComputeElevationDeg(gnbPositions.at(gnbIndex), ue.localPosition);
                bestOffAxisDeg = offAxisDeg;
                bestGnbIndex = gnbIndex;
            }
        }

        double signalW = 0.0;
        double interferenceW = 0.0;
        double bestInterfererRxDbm = -std::numeric_limits<double>::infinity();
        uint32_t bestInterfererGnbIndex = 0;
        double bestInterfererDistanceM = 0.0;
        double bestInterfererElevationDeg = 0.0;
        double bestInterfererOffAxisDeg = 0.0;
        double bestInterfererSatBeamGainDb = 0.0;
        for (uint32_t gnbIndex = 0; gnbIndex < perCellRxPowerW.size(); ++gnbIndex)
        {
            if (gnbIndex == bestGnbIndex)
            {
                signalW += perCellRxPowerW.at(gnbIndex);
            }
            else
            {
                interferenceW += perCellRxPowerW.at(gnbIndex);
                if (perCellRxPowerDbm.at(gnbIndex) > bestInterfererRxDbm)
                {
                    bestInterfererRxDbm = perCellRxPowerDbm.at(gnbIndex);
                    bestInterfererGnbIndex = gnbIndex;
                    bestInterfererDistanceM =
                        CalculateDistance(gnbPositions.at(gnbIndex), ue.localPosition);
                    bestInterfererElevationDeg =
                        ComputeElevationDeg(gnbPositions.at(gnbIndex), ue.localPosition);
                    bestInterfererOffAxisDeg =
                        ComputeOffAxisDeg(gnbPositions.at(gnbIndex), ue.localPosition);
                    bestInterfererSatBeamGainDb = useSatAntennaPattern
                                                      ? ComputeTr38811CircularApertureGainDb(
                                                            bestInterfererOffAxisDeg,
                                                            carrierFrequencyHz,
                                                            satApertureRadiusM,
                                                            gnbAntennaGainDb,
                                                            satAntennaMinGainDb)
                                                      : gnbAntennaGainDb;
                }
            }
        }

        const double sinrLinear = signalW / std::max(1e-30, interferenceW + noiseW);
        const double servingElevationQuantizedDeg = QuantizeElevationDeg(bestElevationDeg);
        const double servingScintillationLossDb =
            ComputeScintillationLossDb(carrierFrequencyHz, servingElevationQuantizedDeg);
        const double servingDlCouplingLossWithScintDb =
            bestPropagationLossDb - bestSatBeamGainDb - ueAntennaGainDb;
        const double servingDlCouplingLossDb =
            servingDlCouplingLossWithScintDb - servingScintillationLossDb;
        const char* servingLosCond = (bestPropagationLossDb - servingScintillationLossDb > 160.0)
                                         ? "NLOS"
                                         : "LOS";
        const double servingMinusBestInterfererDb =
            bestRxDbm - bestInterfererRxDbm;
        const double servingMinusSumInterferenceDb =
            bestRxDbm - WattsToDbm(interferenceW);
        const double nominalMinusServingDb = nominalServingRxDbm - bestRxDbm;
        const uint32_t nominalForcedServingGnbIndex = ue.nominalGnbIndex;
        const double nominalForcedDistanceM =
            CalculateDistance(gnbPositions.at(nominalForcedServingGnbIndex), ue.localPosition);
        const double nominalForcedElevationDeg =
            ComputeElevationDeg(gnbPositions.at(nominalForcedServingGnbIndex), ue.localPosition);
        const double nominalForcedOffAxisDeg =
            ComputeOffAxisDeg(gnbPositions.at(nominalForcedServingGnbIndex), ue.localPosition);
        const double nominalForcedPropagationLossDb =
            perCellPropagationLossDb.at(nominalForcedServingGnbIndex);
        const double nominalForcedSatBeamGainDb =
            perCellSatBeamGainDb.at(nominalForcedServingGnbIndex);
        const double nominalForcedDlCouplingLossWithScintDb =
            nominalForcedPropagationLossDb - nominalForcedSatBeamGainDb - ueAntennaGainDb;
        const double nominalForcedDlCouplingLossDb =
            nominalForcedDlCouplingLossWithScintDb -
            ComputeScintillationLossDb(carrierFrequencyHz,
                                       QuantizeElevationDeg(nominalForcedElevationDeg));
        const double nominalForcedRxPowerDbm = perCellRxPowerDbm.at(nominalForcedServingGnbIndex);
        double nominalForcedSignalW = 0.0;
        double nominalForcedInterferenceW = 0.0;
        for (uint32_t gnbIndex = 0; gnbIndex < perCellRxPowerW.size(); ++gnbIndex)
        {
            if (gnbIndex == nominalForcedServingGnbIndex)
            {
                nominalForcedSignalW += perCellRxPowerW.at(gnbIndex);
            }
            else
            {
                nominalForcedInterferenceW += perCellRxPowerW.at(gnbIndex);
            }
        }
        const double nominalForcedSinrDb =
            LinearToDb(nominalForcedSignalW /
                       std::max(1e-30, nominalForcedInterferenceW + noiseW));
        output << ue.ueIndex << ',' << ue.nominalGnbIndex << ',' << ue.localPosition.x << ','
               << ue.localPosition.y << ',' << bestGnbIndex << ',' << bestDistanceM << ','
               << bestElevationDeg << ',' << bestOffAxisDeg << ',' << bestPropagationLossDb << ','
               << servingDlCouplingLossDb << ',' << servingDlCouplingLossWithScintDb << ','
               << bestPropagationRxDbm << ','
               << bestSatBeamGainDb << ',' << ueAntennaGainDb << ','
               << servingElevationQuantizedDeg << ',' << servingLosCond << ','
               << bestRxDbm << ',' << WattsToDbm(interferenceW) << ',' << WattsToDbm(noiseW)
               << ',' << LinearToDb(sinrLinear) << ',' << bestInterfererGnbIndex << ','
               << bestInterfererDistanceM << ',' << bestInterfererElevationDeg << ','
               << bestInterfererOffAxisDeg << ',' << bestInterfererSatBeamGainDb << ','
               << bestInterfererRxDbm << ',' << servingMinusBestInterfererDb << ','
               << servingMinusSumInterferenceDb << ',' << nominalServingRxDbm << ','
               << nominalMinusServingDb << ',' << nominalForcedServingGnbIndex << ','
               << nominalForcedDistanceM << ',' << nominalForcedElevationDeg << ','
               << nominalForcedOffAxisDeg << ',' << nominalForcedPropagationLossDb << ','
               << nominalForcedDlCouplingLossDb << ',' << nominalForcedDlCouplingLossWithScintDb
               << ',' << nominalForcedSatBeamGainDb << ',' << nominalForcedRxPowerDbm << ','
               << WattsToDbm(nominalForcedInterferenceW) << ',' << nominalForcedSinrDb
               << std::endl;
    }
}

static void
CaptureDlDataSignalPsd(const SpectrumValue& value)
{
    gDlDataSignalPsd = value.Copy();
}

static void
CaptureDlDataSinrPsd(const SpectrumValue& value)
{
    gDlDataSinrPsd = value.Copy();
}

static void
CaptureDlCtrlSignalPsd(const SpectrumValue& value)
{
    gDlCtrlSignalPsd = value.Copy();
}

static void
CaptureDlCtrlSinrPsd(const SpectrumValue& value)
{
    gDlCtrlSinrPsd = value.Copy();
}

static void
LogSinrBreakdown(const char* type,
                 uint16_t cellId,
                 uint16_t rnti,
                 uint16_t bwpId,
                 double sinrLinear,
                 const Ptr<const SpectrumValue>& signalPsd,
                 const Ptr<const SpectrumValue>& sinrPsd)
{
    if (!gSinrBreakdownLog.is_open() || !signalPsd)
    {
        return;
    }

    const double signalW = Integral(*signalPsd);
    const double activeBandwidthHz = ComputeActiveBandwidthHz(signalPsd);
    const double thermalNoiseWPerHz = std::pow(10.0, (-174.0 - 30.0) / 10.0);
    const double noiseFigureLinear = std::pow(10.0, gUeNoiseFigureDb / 10.0);
    const double noiseW = thermalNoiseWPerHz * noiseFigureLinear * activeBandwidthHz;
    const double interferenceW =
        (sinrLinear > 0.0) ? std::max(0.0, (signalW / sinrLinear) - noiseW)
                           : std::numeric_limits<double>::quiet_NaN();

    double avgSinrFromPsd = std::numeric_limits<double>::quiet_NaN();
    if (sinrPsd && sinrPsd->GetValuesN() > 0)
    {
        avgSinrFromPsd = Sum(*sinrPsd) / static_cast<double>(sinrPsd->GetValuesN());
    }

    gSinrBreakdownLog << Simulator::Now().GetMilliSeconds()
                      << "ms [SINR-BREAKDOWN] type=" << type
                      << " cellId=" << cellId
                      << " RNTI=" << rnti
                      << " bwpId=" << bwpId
                      << " signalW=" << signalW
                      << " signalDbm=" << WattsToDbm(signalW)
                      << " noiseW=" << noiseW
                      << " noiseDbm=" << WattsToDbm(noiseW)
                      << " activeBandwidthHz=" << activeBandwidthHz
                      << " interferenceW=" << interferenceW
                      << " interferenceDbm=" << WattsToDbm(interferenceW)
                      << " sinrLinear=" << sinrLinear
                      << " sinrDb=" << LinearToDb(sinrLinear)
                      << " avgSinrFromPsdLinear=" << avgSinrFromPsd
                      << " avgSinrFromPsdDb=" << LinearToDb(avgSinrFromPsd)
                      << std::endl;
}

static inline double
ComputeElevationDeg(const Vector& satPos, const Vector& uePos)
{
    const double dx = satPos.x - uePos.x;
    const double dy = satPos.y - uePos.y;
    const double dz = satPos.z - uePos.z;
    const double horizontal = std::sqrt(dx * dx + dy * dy);
    return std::atan2(dz, horizontal) * 180.0 / M_PI;
}

static inline double
ComputeOffAxisDeg(const Vector& satPos, const Vector& uePos)
{
    const double elevationDeg = ComputeElevationDeg(satPos, uePos);
    return std::max(0.0, 90.0 - elevationDeg);
}

static inline double
ComputeTr38811CircularApertureGainDb(double offAxisDeg,
                                     double operatingFrequencyHz,
                                     double apertureRadiusM,
                                     double maxGainDb,
                                     double minGainDb)
{
    constexpr double c = 299792458.0;
    // 3GPP TR 38.811 Sec. 6.4.1 normalized reflector pattern:
    // G_norm(theta) = 1, theta = 0
    // G_norm(theta) = 4 * |J1(k a sin(theta)) / (k a sin(theta))|^2, 0 < |theta| <= 90 deg
    // We convert the normalized gain to an absolute gain by adding maxGainDb.
    const double theta = std::abs(offAxisDeg) * M_PI / 180.0;
    if (theta == 0.0)
    {
        return maxGainDb;
    }
    if (theta >= M_PI_2)
    {
        return minGainDb;
    }

    const double k = (2.0 * M_PI * operatingFrequencyHz) / c;
    const double x = k * apertureRadiusM * std::sin(theta);
    if (std::abs(x) < 1e-12)
    {
        return maxGainDb;
    }

    const double ratio = std::cyl_bessel_j(1, x) / x;
    const double gainLinear = 4.0 * std::pow(std::abs(ratio), 2);
    if (gainLinear <= 0.0)
    {
        return minGainDb;
    }

    return std::max(minGainDb, maxGainDb + 10.0 * std::log10(gainLinear));
}

static double
ComputeCircularAperture3dBeamwidthDeg(double apertureRadiusInWavelengths)
{
    double lo = 0.0;
    double hi = 30.0;
    for (uint32_t i = 0; i < 200; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        const double theta = mid * M_PI / 180.0;
        const double x = 2.0 * M_PI * apertureRadiusInWavelengths * std::sin(theta);
        double relativeGainDb = 0.0;
        if (std::abs(x) >= 1e-12)
        {
            const double ratio = std::cyl_bessel_j(1, x) / x;
            relativeGainDb = 10.0 * std::log10(4.0 * ratio * ratio);
        }

        if (relativeGainDb > -3.0)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return lo + hi;
}

static inline bool
InMetricWindow()
{
    if (!gEnableMetricLog)
    {
        return false;
    }
    const int64_t nowMs = static_cast<int64_t>(Simulator::Now().GetMilliSeconds());
    if (gMetricStartMs >= 0 && nowMs < gMetricStartMs)
    {
        return false;
    }
    if (gMetricEndMs >= 0 && nowMs > gMetricEndMs)
    {
        return false;
    }
    return true;
}

void
LogUeMeasurementTrace(std::string context,
                      uint16_t rnti,
                      uint16_t cellId,
                      double rsrp,
                      double rsrq,
                      bool isServing,
                      uint8_t bwpId)
{
    if (!gUeMeasurementLog.is_open())
    {
        return;
    }

    gUeMeasurementLog << Simulator::Now().GetMilliSeconds()
                      << "ms [UE-MEAS] rnti=" << rnti
                      << " context=" << context
                      << " cellId=" << cellId
                      << " rsrpDbm=" << rsrp
                      << " rsrqDb=" << rsrq
                      << " isServing=" << (isServing ? "true" : "false")
                      << " bwpId=" << static_cast<uint16_t>(bwpId) << std::endl;
}

void
LogRsrpTrace(std::string context,
             uint16_t cellId,
             uint16_t imsi,
             uint16_t rnti,
             double rsrpDbm,
             uint8_t bwpId)
{
    if (!InMetricWindow())
    {
        return;
    }
    if (gSnapshotOnce && gPrintedRsrp)
    {
        return;
    }

    const auto it = gCellToPosition.find(cellId);
    if (it == gCellToPosition.end())
    {
        return;
    }

    const double distanceM = CalculateDistance(it->second, gUePosition);
    const double elevationDeg = ComputeElevationDeg(it->second, gUePosition);
    const auto itOffAxis = gCellToOffAxisDeg.find(cellId);
    const auto itPatternGain = gCellToSatPatternGainDb.find(cellId);
    const double offAxisDeg = (itOffAxis != gCellToOffAxisDeg.end())
                                  ? itOffAxis->second
                                  : std::numeric_limits<double>::quiet_NaN();
    const double satPatternGainDb = (itPatternGain != gCellToSatPatternGainDb.end())
                                        ? itPatternGain->second
                                        : std::numeric_limits<double>::quiet_NaN();

    gMetricLog << Simulator::Now().GetMilliSeconds() << "ms [CH-METRIC] type=RSRP"
               << " context=" << context
               << " cellId=" << cellId
               << " IMSI=" << imsi
               << " RNTI=" << rnti
               << " bwpId=" << static_cast<uint16_t>(bwpId)
               << " rsrpDbm=" << rsrpDbm
               << " distanceM=" << distanceM
               << " elevationDeg=" << elevationDeg
               << " offAxisDeg=" << offAxisDeg
               << " satPatternGainDb=" << satPatternGainDb
               << std::endl;

    if (gSnapshotOnce)
    {
        gPrintedRsrp = true;
    }
}

void
LogDlDataSinrTrace(std::string context,
                   uint16_t cellId,
                   uint16_t rnti,
                   double sinrLinear,
                   uint16_t bwpId)
{
    if (!InMetricWindow())
    {
        return;
    }
    if (gSnapshotOnce && gPrintedSinrData)
    {
        return;
    }

    const auto it = gCellToPosition.find(cellId);
    if (it == gCellToPosition.end())
    {
        return;
    }

    const double distanceM = CalculateDistance(it->second, gUePosition);
    const double elevationDeg = ComputeElevationDeg(it->second, gUePosition);
    const auto itOffAxis = gCellToOffAxisDeg.find(cellId);
    const auto itPatternGain = gCellToSatPatternGainDb.find(cellId);
    const double offAxisDeg = (itOffAxis != gCellToOffAxisDeg.end())
                                  ? itOffAxis->second
                                  : std::numeric_limits<double>::quiet_NaN();
    const double satPatternGainDb = (itPatternGain != gCellToSatPatternGainDb.end())
                                        ? itPatternGain->second
                                        : std::numeric_limits<double>::quiet_NaN();

    gMetricLog << Simulator::Now().GetMilliSeconds() << "ms [CH-METRIC] type=SINR_DL_DATA"
               << " context=" << context
               << " cellId=" << cellId
               << " RNTI=" << rnti
               << " bwpId=" << bwpId
               << " sinrLinear=" << sinrLinear
               << " sinrDb=" << LinearToDb(sinrLinear)
               << " distanceM=" << distanceM
               << " elevationDeg=" << elevationDeg
               << " offAxisDeg=" << offAxisDeg
               << " satPatternGainDb=" << satPatternGainDb
               << std::endl;

    LogSinrBreakdown("DL_DATA",
                     cellId,
                     rnti,
                     bwpId,
                     sinrLinear,
                     gDlDataSignalPsd,
                     gDlDataSinrPsd);

    if (gSnapshotOnce)
    {
        gPrintedSinrData = true;
    }
}

void
LogDlCtrlSinrTrace(std::string context,
                   uint16_t cellId,
                   uint16_t rnti,
                   double sinrLinear,
                   uint16_t bwpId)
{
    if (!InMetricWindow())
    {
        return;
    }
    if (gSnapshotOnce && gPrintedSinrCtrl)
    {
        return;
    }

    const auto it = gCellToPosition.find(cellId);
    if (it == gCellToPosition.end())
    {
        return;
    }

    const double distanceM = CalculateDistance(it->second, gUePosition);
    const double elevationDeg = ComputeElevationDeg(it->second, gUePosition);
    const auto itOffAxis = gCellToOffAxisDeg.find(cellId);
    const auto itPatternGain = gCellToSatPatternGainDb.find(cellId);
    const double offAxisDeg = (itOffAxis != gCellToOffAxisDeg.end())
                                  ? itOffAxis->second
                                  : std::numeric_limits<double>::quiet_NaN();
    const double satPatternGainDb = (itPatternGain != gCellToSatPatternGainDb.end())
                                        ? itPatternGain->second
                                        : std::numeric_limits<double>::quiet_NaN();

    gMetricLog << Simulator::Now().GetMilliSeconds() << "ms [CH-METRIC] type=SINR_DL_CTRL"
               << " context=" << context
               << " cellId=" << cellId
               << " RNTI=" << rnti
               << " bwpId=" << bwpId
               << " sinrLinear=" << sinrLinear
               << " sinrDb=" << LinearToDb(sinrLinear)
               << " distanceM=" << distanceM
               << " elevationDeg=" << elevationDeg
               << " offAxisDeg=" << offAxisDeg
               << " satPatternGainDb=" << satPatternGainDb
               << std::endl;

    LogSinrBreakdown("DL_CTRL",
                     cellId,
                     rnti,
                     bwpId,
                     sinrLinear,
                     gDlCtrlSignalPsd,
                     gDlCtrlSinrPsd);

    if (gSnapshotOnce)
    {
        gPrintedSinrCtrl = true;
    }
}

static std::string
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

static std::string
CompactContextId(const std::string& context)
{
    return "N" + ExtractBetween(context, "/NodeList/", "/") + "D" +
           ExtractBetween(context, "/DeviceList/", "/");
}

static std::string
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
    default:
        return "UNKNOWN";
    }
}

static void
EmitProcedureTrace(const std::string& line)
{
    std::cout << line << std::endl;
    if (gHoLog.is_open())
    {
        gHoLog << line << std::endl;
    }
}

static bool
IsHoRelevantCtrlMessage(Ptr<const NrControlMessage> ctrlMessage)
{
    if (!ctrlMessage)
    {
        return false;
    }

    switch (ctrlMessage->GetMessageType())
    {
    case NrControlMessage::RACH_PREAMBLE:
    case NrControlMessage::RAR:
        return true;
    default:
        return false;
    }
}

static void
EmitHoCtrlMessageIfRelevant(const char* layer,
                            const char* direction,
                            uint16_t nodeId,
                            uint16_t rnti,
                            const SfnSf& sfnSf,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    if (!IsHoRelevantCtrlMessage(ctrlMessage))
    {
        return;
    }

    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [HO-MSG]"
        << " layer=" << layer
        << " dir=" << direction
        << " nodeId=" << nodeId
        << " rnti=" << rnti
        << " sfnSf=" << sfnSf
        << " message=" << ControlMessageTypeToString(ctrlMessage);
    EmitProcedureTrace(oss.str());
}

static void
EmitCtrlMsgTrace(const std::string& context,
                 const char* layer,
                 const char* direction,
                 const SfnSf& sfnSf,
                 uint16_t nodeId,
                 uint16_t rnti,
                 uint8_t bwpId,
                 Ptr<const NrControlMessage> ctrlMessage)
{
    if (!gCtrlMsgLog.is_open())
    {
        return;
    }

    std::ostringstream oss;
    oss << Simulator::Now().GetSeconds() << "s [CTRL]"
        << " layer=" << layer
        << " dir=" << direction
        << " context=" << CompactContextId(context)
        << " nodeId=" << nodeId
        << " rnti=" << rnti
        << " bwpId=" << static_cast<uint32_t>(bwpId)
        << " sfnSf=" << sfnSf
        << " msgType=" << ControlMessageTypeToString(ctrlMessage);

    gCtrlMsgLog << oss.str() << std::endl;
    EmitHoCtrlMessageIfRelevant(layer, direction, nodeId, rnti, sfnSf, ctrlMessage);
}

static void
NotifyGnbPhyRxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "GNB-PHY", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyGnbPhyTxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "GNB-PHY", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyGnbMacRxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "GNB-MAC", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyGnbMacTxedCtrlMessage(std::string context,
                            const SfnSf sfnSf,
                            const uint16_t nodeId,
                            const uint16_t rnti,
                            const uint8_t bwpId,
                            Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "GNB-MAC", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyUePhyRxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "UE-PHY", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyUePhyTxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "UE-PHY", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyUeMacRxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "UE-MAC", "RX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyUeMacTxedCtrlMessage(std::string context,
                           const SfnSf sfnSf,
                           const uint16_t nodeId,
                           const uint16_t rnti,
                           const uint8_t bwpId,
                           Ptr<const NrControlMessage> ctrlMessage)
{
    EmitCtrlMsgTrace(context, "UE-MAC", "TX", sfnSf, nodeId, rnti, bwpId, ctrlMessage);
}

static void
NotifyConnectionEstablishedGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [RRC-GNB-CONNECTED] context=" << context
        << " imsi=" << imsi << " cellId=" << cellid << " rnti=" << rnti;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyConnectionEstablishedUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [RRC-UE-CONNECTED] context=" << context
        << " imsi=" << imsi << " cellId=" << cellid << " rnti=" << rnti;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyHandoverStartGnb(std::string context,
                       uint64_t imsi,
                       uint16_t cellid,
                       uint16_t rnti,
                       uint16_t targetCellId)
{
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [HO-START-GNB] context=" << context
        << " imsi=" << imsi << " sourceCellId=" << cellid << " rnti=" << rnti
        << " targetCellId=" << targetCellId;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyHandoverStartUe(std::string context,
                      uint64_t imsi,
                      uint16_t cellid,
                      uint16_t rnti,
                      uint16_t targetCellId)
{
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [HO-START-UE] context=" << context
        << " imsi=" << imsi << " sourceCellId=" << cellid << " rnti=" << rnti
        << " targetCellId=" << targetCellId;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [HO-ENDOK-GNB] context=" << context
        << " imsi=" << imsi << " targetCellId=" << cellid << " rnti=" << rnti;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyHandoverEndOkUe(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [HO-ENDOK-UE] context=" << context
        << " imsi=" << imsi << " targetCellId=" << cellid << " rnti=" << rnti;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyMeasurementReport(std::string context,
                        uint64_t imsi,
                        uint16_t cellId,
                        uint16_t rnti,
                        NrRrcSap::MeasurementReport report)
{
    const uint32_t measId = report.measResults.measId;
    const auto& measResults = report.measResults;
    std::ostringstream oss;
    oss << Simulator::Now().GetMilliSeconds() << "ms [MR] context=" << context
        << " imsi=" << imsi << " servingCellId=" << cellId << " rnti=" << rnti
        << " measId=" << measId
        << " servingRsrpRange="
        << static_cast<uint32_t>(measResults.measResultPCell.rsrpResult)
        << " servingRsrpDbm="
        << DecodeRsrpRangeToDbm(measResults.measResultPCell.rsrpResult)
        << " servingRsrqRange="
        << static_cast<uint32_t>(measResults.measResultPCell.rsrqResult)
        << " servingRsrqDb="
        << DecodeRsrqRangeToDb(measResults.measResultPCell.rsrqResult)
        << " neighborCount=" << measResults.measResultListEutra.size();

    for (const auto& neighbor : measResults.measResultListEutra)
    {
        oss << " neighCellId=" << neighbor.physCellId;
        if (neighbor.haveRsrpResult)
        {
            oss << " neighRsrpRange=" << static_cast<uint32_t>(neighbor.rsrpResult)
                << " neighRsrpDbm=" << DecodeRsrpRangeToDbm(neighbor.rsrpResult);
        }
        if (neighbor.haveRsrqResult)
        {
            oss << " neighRsrqRange=" << static_cast<uint32_t>(neighbor.rsrqResult)
                << " neighRsrqDb=" << DecodeRsrqRangeToDb(neighbor.rsrqResult);
        }
    }

    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyNrChoEvent(uint64_t timestampMs,
                 uint16_t rnti,
                 std::string eventName,
                 uint16_t targetCellId,
                 uint8_t servingRsrp,
                 uint8_t targetRsrp,
                 uint16_t candidateCount)
{
    std::ostringstream oss;
    oss << timestampMs << "ms [CHO-EVENT] rnti=" << rnti << " event=" << eventName
        << " targetCellId=" << targetCellId
        << " servingRsrp=" << static_cast<uint32_t>(servingRsrp)
        << " targetRsrp=" << static_cast<uint32_t>(targetRsrp)
        << " candidateCount=" << candidateCount;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

static void
NotifyNrChoDetailedEvent(uint64_t timestampMs,
                         uint16_t rnti,
                         std::string stepName,
                         std::string detail)
{
    std::ostringstream oss;
    oss << timestampMs << "ms [CHO-DETAIL] rnti=" << rnti << " step=" << stepName << ' ' << detail;
    EmitProcedureTrace(oss.str());
    gMetricLog << oss.str() << std::endl;
}

std::vector<Vector>
CreateHexPositions(double isdMeters, double altitudeMeters, uint32_t tiers)
{
    std::vector<Vector> positions;
    positions.reserve(1 + 3 * tiers * (tiers + 1));
    const double radius = isdMeters / std::sqrt(3.0);

    positions.emplace_back(0.0, 0.0, altitudeMeters);
    for (uint32_t tier = 1; tier <= tiers; ++tier)
    {
        for (uint32_t side = 0; side < 6; ++side)
        {
            for (uint32_t step = 0; step < tier; ++step)
            {
                const double angle = (side * 60.0 + 30.0) * M_PI / 180.0;
                const double dx = radius * std::sqrt(3.0) *
                                  (tier * std::cos(angle) -
                                   step * std::sin(angle + M_PI / 6.0));
                const double dy = radius * std::sqrt(3.0) *
                                  (tier * std::sin(angle) +
                                   step * std::cos(angle + M_PI / 6.0));
                positions.emplace_back(dx, dy, altitudeMeters);
            }
        }
    }

    return positions;
}

int
main(int argc, char* argv[])
{
    const std::string outputDir = "output";
    std::filesystem::create_directories(outputDir);
    const std::string metricLogFile = outputDir + "/conditional-handover-test-metrics.log";
    const std::string sinrBreakdownLogFile =
        outputDir + "/conditional-handover-test-sinr-breakdown.log";
    const std::string cellBudgetLogFile = outputDir + "/conditional-handover-test-cell-budget.log";
    const std::string ctrlMsgLogFile = outputDir + "/conditional-handover-test-ctrl-msg.log";
    const std::string geometryLogFile = outputDir + "/conditional-handover-test-geometry.log";
    const std::string hoLogFile = outputDir + "/conditional-handover-test-ho.log";
    const std::string ueMeasurementLogFile =
        outputDir + "/conditional-handover-test-ue-measurements.log";
    const std::string propagationDebugLogFile =
        outputDir + "/conditional-handover-test-propagation-debug.csv";
    const std::string servingSinrCsvFile =
        outputDir + "/conditional-handover-test-serving-sinr.csv";
    gMetricLog.open(metricLogFile);
    gSinrBreakdownLog.open(sinrBreakdownLogFile);
    gCellBudgetLog.open(cellBudgetLogFile);
    gCtrlMsgLog.open(ctrlMsgLogFile);
    gGeometryLog.open(geometryLogFile);
    gHoLog.open(hoLogFile);
    gUeMeasurementLog.open(ueMeasurementLogFile);
    std::ofstream(propagationDebugLogFile, std::ios::trunc).close();
    std::ofstream(servingSinrCsvFile, std::ios::trunc).close();

    double simTime = 10.0;
    bool useIdealRrc = false;

    double satAltitudeMeters = 600000.0;
    double interSiteDistanceMeters = 41750.0;
    uint32_t gnbTiers = 2;
    uint32_t snapshotUeAnchorTiers = 2;
    double ueLatitudeDeg = 0.0;
    double ueLongitudeDeg = 0.0;
    double ueOffsetXMeters = 0.0;
    double ueOffsetYMeters = 0.0;

    // LEO-600 S-band calibration defaults from 3GPP TR 38.811 Table 6.1.1.1-1.
    double bandwidthHz = 20e6;
    double gnbEirpDensityDbwPerMHz = 34.0;
    double ueTxPowerDbm = 23.0;
    double gnbAntennaGainDb = 30.0;
    double gnbTxPowerDbm =
        gnbEirpDensityDbwPerMHz + 10.0 * std::log10(bandwidthHz * 1e-6) + 30.0 -
        gnbAntennaGainDb;
    double ueAntennaGainDb = 0.0;
    double ueNoiseFigureDb = 7.0;
    bool useSatAntennaPattern = true;
    // 6.671286 lambda yields 3 dB beamwidth ~= 4.4127 deg with the ns-3 circular aperture model.
    double satApertureRadiusInWavelengths = 6.671286;
    double satAntennaMinGainDb = -100.0;
    bool alignGnbBoresightToNadir = true;

    double carrierFrequencyHz = 2.0e9;
    bool shadowingEnabled = true;
    uint32_t snapshotUesPerCell = 0;
    double snapshotCellRadiusMeters = 25000.0;
    double leoSpeedMps = 7560.0;
    double positionUpdatePeriodMs = 100.0;
    bool enableA3Handover = true;
    bool enableChoExecution = true;
    double hoHysteresisDb = 0.0;
    double hoA3OffsetDb = 0.0;
    double hoTttMs = 0.0;
    uint8_t l3RsrpFilterCoefficient = 4;
    uint8_t l3RsrqFilterCoefficient = 4;
    bool enableX2Interface = true;
    bool enableRawCtrlTrace = false;
    bool enableTraffic = false;

    bool enableMetricLog = false;
    bool enablePhyMetricTrace = false;
    bool enablePropagationDebugLog = false;
    int64_t metricStartMs = -1;
    int64_t metricEndMs = -1;
    bool snapshotOnce = false;
    double snapshotDurationMs = 300.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration [s]", simTime);
    cmd.AddValue("useIdealRrc", "Use idealized RRC", useIdealRrc);
    cmd.AddValue("satAltitudeMeters", "gNB(satellite) altitude [m]", satAltitudeMeters);
    cmd.AddValue("interSiteDistanceMeters",
                 "Hex-cell inter-site distance [m] (default 43.3 km)",
                 interSiteDistanceMeters);
    cmd.AddValue("gnbTiers", "Number of hex tiers used for gNB deployment (2 => 19 gNBs, 4 => 61 gNBs)", gnbTiers);
    cmd.AddValue("snapshotUeAnchorTiers",
                 "Number of inner hex tiers used as UE anchor beams (2 => inner 19 beams)",
                 snapshotUeAnchorTiers);
    cmd.AddValue("ueLatitudeDeg", "UE latitude [deg]", ueLatitudeDeg);
    cmd.AddValue("ueLongitudeDeg", "UE longitude [deg]", ueLongitudeDeg);
    cmd.AddValue("ueOffsetXMeters",
                 "UE local x offset from the center cell footprint [m]",
                 ueOffsetXMeters);
    cmd.AddValue("ueOffsetYMeters",
                 "UE local y offset from the center cell footprint [m]",
                 ueOffsetYMeters);
    cmd.AddValue("gnbTxPowerDbm", "gNB Tx power [dBm]", gnbTxPowerDbm);
    cmd.AddValue("ueTxPowerDbm", "UE Tx power [dBm]", ueTxPowerDbm);
    cmd.AddValue("gnbAntennaGainDb", "gNB antenna gain [dB]", gnbAntennaGainDb);
    cmd.AddValue("ueAntennaGainDb", "UE antenna gain [dB]", ueAntennaGainDb);
    cmd.AddValue("ueNoiseFigureDb", "UE receiver noise figure [dB]", ueNoiseFigureDb);
    cmd.AddValue("useSatAntennaPattern",
                 "Use 3GPP 38.811 circular-aperture satellite antenna pattern on gNB",
                 useSatAntennaPattern);
    cmd.AddValue("satApertureRadiusInWavelengths",
                 "Satellite antenna aperture radius [wavelengths], e.g., 10 for TR 38.811 Fig 6.4.1.1",
                 satApertureRadiusInWavelengths);
    cmd.AddValue("satAntennaMinGainDb",
                 "Minimum gain floor [dB] for circular-aperture antenna model",
                 satAntennaMinGainDb);
    cmd.AddValue("alignGnbBoresightToNadir",
                 "If true, rotate gNB antenna boresight toward nadir (-z) for EMC-style beams",
                 alignGnbBoresightToNadir);
    cmd.AddValue("carrierFrequencyHz", "Carrier frequency [Hz]", carrierFrequencyHz);
    cmd.AddValue("bandwidthHz", "Carrier bandwidth [Hz]", bandwidthHz);
    cmd.AddValue("shadowingEnabled", "Enable channel shadowing", shadowingEnabled);
    cmd.AddValue("snapshotUesPerCell", "Number of snapshot UEs generated per gNB", snapshotUesPerCell);
    cmd.AddValue("snapshotCellRadiusMeters",
                 "Radius of the per-gNB circular UE placement area [m]",
                 snapshotCellRadiusMeters);
    cmd.AddValue("leoSpeedMps", "LEO satellite speed [m/s]", leoSpeedMps);
    cmd.AddValue("positionUpdatePeriodMs",
                 "Satellite and X2-delay update period [ms]",
                 positionUpdatePeriodMs);
    cmd.AddValue("enableA3Handover", "Enable NR A3-based conditional handover", enableA3Handover);
    cmd.AddValue("enableChoExecution",
                 "Execute actual NR conditional handover after CHO execution condition is met",
                 enableChoExecution);
    cmd.AddValue("hoHysteresisDb", "A3/CHO hysteresis [dB]", hoHysteresisDb);
    cmd.AddValue("hoA3OffsetDb", "A3/CHO event offset [dB]", hoA3OffsetDb);
    cmd.AddValue("hoTttMs", "A3/CHO time-to-trigger [ms]", hoTttMs);
    cmd.AddValue("l3RsrpFilterCoefficient",
                 "NR L3 RSRP filter coefficient applied by NrGnbRrc measurement config",
                 l3RsrpFilterCoefficient);
    cmd.AddValue("l3RsrqFilterCoefficient",
                 "NR L3 RSRQ filter coefficient applied by NrGnbRrc measurement config",
                 l3RsrqFilterCoefficient);
    cmd.AddValue("enableX2Interface", "Enable X2 interfaces among all gNB pairs", enableX2Interface);
    cmd.AddValue("enableRawCtrlTrace",
                 "Enable raw PHY/MAC control-message trace logging to ctrl-msg.log",
                 enableRawCtrlTrace);
    cmd.AddValue("enableTraffic",
                 "Enable user-plane UDP traffic; disable for lighter HO/control-only runs",
                 enableTraffic);
    cmd.AddValue("enableMetricLog", "Enable channel metric logging", enableMetricLog);
    cmd.AddValue("enablePhyMetricTrace",
                 "Enable PHY-side RSRP/SINR trace logging into metrics/sinr-breakdown logs",
                 enablePhyMetricTrace);
    cmd.AddValue("enablePropagationDebugLog",
                 "Enable ThreeGppPropagationLossModel propagation-debug CSV logging",
                 enablePropagationDebugLog);
    cmd.AddValue("metricStartMs", "Metric log start [ms], -1 means beginning", metricStartMs);
    cmd.AddValue("metricEndMs", "Metric log end [ms], -1 means end", metricEndMs);
    cmd.AddValue("snapshotOnce",
                 "If true, capture one static snapshot and exit quickly (no periodic traffic traces)",
                 snapshotOnce);
    cmd.AddValue("snapshotDurationMs",
                 "Simulation duration used in snapshotOnce mode [ms]",
                 snapshotDurationMs);
    cmd.Parse(argc, argv);

    gEnableMetricLog = enableMetricLog;
    gMetricStartMs = metricStartMs;
    gMetricEndMs = metricEndMs;
    gSnapshotOnce = snapshotOnce;
    gPrintedRsrp = false;
    gPrintedSinrCtrl = false;
    gPrintedSinrData = false;

    if (snapshotOnce)
    {
        simTime = std::max(0.001, snapshotDurationMs / 1000.0);
        std::cout << "[CHO-TEST] snapshotOnce=true, overriding simTime to " << simTime << "s"
                  << std::endl;
    }

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));
    Config::SetDefault("ns3::NrGnbRrc::RsrpFilterCoefficient",
                       UintegerValue(l3RsrpFilterCoefficient));
    Config::SetDefault("ns3::NrGnbRrc::RsrqFilterCoefficient",
                       UintegerValue(l3RsrqFilterCoefficient));

    const std::vector<Vector> gnbPositions =
        CreateHexPositions(interSiteDistanceMeters, satAltitudeMeters, gnbTiers);
    const std::vector<Vector> uePositions = {Vector(ueOffsetXMeters, ueOffsetYMeters, 0.0)};
    const std::vector<Vector> snapshotUeAnchorPositions =
        CreateHexPositions(interSiteDistanceMeters, satAltitudeMeters, snapshotUeAnchorTiers);
    const uint16_t numberOfGnbs = static_cast<uint16_t>(gnbPositions.size());
    const uint16_t numberOfUes = static_cast<uint16_t>(uePositions.size());

    NodeContainer ueNodes;
    NodeContainer gnbNodes;
    gnbNodes.Create(numberOfGnbs);
    ueNodes.Create(numberOfUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::GeocentricConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);

    const double earthRadiusM = 6371000.0;
    const double latRad = ueLatitudeDeg * M_PI / 180.0;
    const double cosLat = std::max(1e-6, std::cos(latRad));
    const Vector referencePoint(ueLatitudeDeg, ueLongitudeDeg, 0.0);

    gUePosition = uePositions.at(0);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        const Vector& ueLocalPos = uePositions.at(i);
        auto ueGeo = ueNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        ueGeo->SetCoordinateTranslationReferencePoint(referencePoint);
        ueGeo->SetGeographicPosition(
            Vector(ueLatitudeDeg + (ueLocalPos.y / earthRadiusM) * 180.0 / M_PI,
                   ueLongitudeDeg + (ueLocalPos.x / (earthRadiusM * cosLat)) * 180.0 / M_PI,
                   0.0));
    }

    std::vector<double> gnbInitialLongitudesDeg(numberOfGnbs, ueLongitudeDeg);
    std::vector<double> gnbInitialLatitudesDeg(numberOfGnbs, ueLatitudeDeg);
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        const double deltaLatDeg = (gnbPositions.at(i).y / earthRadiusM) * 180.0 / M_PI;
        const double deltaLonDeg = (gnbPositions.at(i).x / (earthRadiusM * cosLat)) * 180.0 / M_PI;

        auto gnbGeo = gnbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        gnbGeo->SetCoordinateTranslationReferencePoint(referencePoint);
        gnbInitialLatitudesDeg.at(i) = ueLatitudeDeg + deltaLatDeg;
        gnbInitialLongitudesDeg.at(i) = ueLongitudeDeg + deltaLonDeg;
    }

    UpdateLeoGnbPositions(gnbNodes,
                          gnbInitialLongitudesDeg,
                          gnbInitialLatitudesDeg,
                          satAltitudeMeters,
                          leoSpeedMps,
                          positionUpdatePeriodMs / 1000.0,
                          simTime);

    NodeContainer snapshotUeNodes;
    const auto snapshotUeRecords = CreateSnapshotUes(snapshotUeNodes,
                                                     snapshotUesPerCell,
                                                     snapshotCellRadiusMeters,
                                                     snapshotUeAnchorPositions,
                                                     referencePoint);

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
        nrHelper->SetHandoverAlgorithmAttribute("A3Offset", DoubleValue(hoA3OffsetDb));
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
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(positionUpdatePeriodMs)));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::DebugLogEnabled",
                       BooleanValue(enablePropagationDebugLog));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::DebugLogFileName",
                       StringValue(propagationDebugLogFile));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::DebugAssumedTxPowerDbm",
                       DoubleValue(gnbTxPowerDbm));
    auto bandwidthAndBwp =
        nrHelper->CreateBandwidthParts({{carrierFrequencyHz, bandwidthHz, 1}}, "NTN-Rural");
    BandwidthPartInfoPtrVector allBwps = bandwidthAndBwp.second;

    double satApertureRadiusM = 0.0;
    Ptr<AntennaModel> gnbElement;
    if (useSatAntennaPattern)
    {
        constexpr double c = 299792458.0;
        satApertureRadiusM = satApertureRadiusInWavelengths * (c / carrierFrequencyHz);
        const double sat3dBeamwidthDeg =
            ComputeCircularAperture3dBeamwidthDeg(satApertureRadiusInWavelengths);
        Ptr<CircularApertureAntennaModel> satElement = CreateObject<CircularApertureAntennaModel>();
        satElement->SetAttribute("OperatingFrequency", DoubleValue(carrierFrequencyHz));
        satElement->SetAttribute("AntennaCircularApertureRadius", DoubleValue(satApertureRadiusM));
        satElement->SetAttribute("AntennaMaxGainDb", DoubleValue(gnbAntennaGainDb));
        satElement->SetAttribute("AntennaMinGainDb", DoubleValue(satAntennaMinGainDb));
        gnbElement = satElement;

        std::cout << "[CHO-TEST] gNB antenna pattern=CircularAperture"
                  << " apertureRadiusLambda=" << satApertureRadiusInWavelengths
                  << " apertureRadiusM=" << satApertureRadiusM
                  << " estimated3dBeamwidthDeg=" << sat3dBeamwidthDeg
                  << " maxGainDb=" << gnbAntennaGainDb
                  << " minGainDb=" << satAntennaMinGainDb
                  << std::endl;
    }
    else
    {
        Ptr<IsotropicAntennaModel> isoElement = CreateObject<IsotropicAntennaModel>();
        isoElement->SetAttribute("Gain", DoubleValue(gnbAntennaGainDb));
        gnbElement = isoElement;
        std::cout << "[CHO-TEST] gNB antenna pattern=Isotropic"
                  << " gainDb=" << gnbAntennaGainDb
                  << std::endl;
    }

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(1));
    if (alignGnbBoresightToNadir)
    {
        // UniformPlanarArray defaults to +x boresight. A +90 deg downtilt rotates it to -z,
        // i.e., nadir for the EMC assumption with satellites above the UE.
        nrHelper->SetGnbAntennaAttribute("BearingAngle", DoubleValue(0.0));
        nrHelper->SetGnbAntennaAttribute("DowntiltAngle", DoubleValue(M_PI_2));
    }
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(gnbElement));

    std::cout << "[CHO-TEST] gNB array orientation="
              << (alignGnbBoresightToNadir ? "nadir" : "default(+x)")
              << std::endl;

    Ptr<IsotropicAntennaModel> ueElement = CreateObject<IsotropicAntennaModel>();
    ueElement->SetAttribute("Gain", DoubleValue(ueAntennaGainDb));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(ueElement));

    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPowerDbm));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPowerDbm));
    nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(ueNoiseFigureDb));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    Ptr<NrUePhy> uePhy = NrHelper::GetUePhy(ueDevs.Get(0), 0);
    NS_ASSERT_MSG(uePhy, "Failed to retrieve UE PHY");
    Ptr<NrSpectrumPhy> ueSpectrumPhy = uePhy->GetSpectrumPhy();
    NS_ASSERT_MSG(ueSpectrumPhy, "Failed to retrieve UE spectrum PHY");
    gUeNoiseFigureDb = uePhy->GetNoiseFigure();

    Ptr<NrChunkProcessor> dlDataPowerProcessor = Create<NrChunkProcessor>();
    dlDataPowerProcessor->AddCallback(MakeCallback(&CaptureDlDataSignalPsd));
    ueSpectrumPhy->AddDataPowerChunkProcessor(dlDataPowerProcessor);

    Ptr<NrChunkProcessor> dlDataSinrProcessor = Create<NrChunkProcessor>();
    dlDataSinrProcessor->AddCallback(MakeCallback(&CaptureDlDataSinrPsd));
    ueSpectrumPhy->AddDataSinrChunkProcessor(dlDataSinrProcessor);

    Ptr<NrChunkProcessor> dlCtrlPowerProcessor = Create<NrChunkProcessor>();
    dlCtrlPowerProcessor->AddCallback(MakeCallback(&CaptureDlCtrlSignalPsd));
    ueSpectrumPhy->AddRsPowerChunkProcessor(dlCtrlPowerProcessor);

    Ptr<NrChunkProcessor> dlCtrlSinrProcessor = Create<NrChunkProcessor>();
    dlCtrlSinrProcessor->AddCallback(MakeCallback(&CaptureDlCtrlSinrPsd));
    ueSpectrumPhy->AddDlCtrlSinrChunkProcessor(dlCtrlSinrProcessor);

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        NrHelper::GetGnbPhy(gnbDevs.Get(i), 0)->SetAttribute("Numerology", UintegerValue(0));
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
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(10)));
    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    Ipv4InterfaceContainer ueIpIfaces = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueDevs));
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(u)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    double minDistance = std::numeric_limits<double>::max();
    uint32_t closestGnbIndex = 0;

    gCellBudgetLog << "0ms [CELL-BUDGET] shadowingEnabled=" << shadowingEnabled
                   << " note=rxPowerEstimateExcludesRandomShadowing"
                   << std::endl;

    for (uint32_t i = 0; i < gnbDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnbDev = DynamicCast<NrGnbNetDevice>(gnbDevs.Get(i));
        NS_ASSERT_MSG(gnbDev, "Failed to cast NrGnbNetDevice");

        const uint16_t cellId = gnbDev->GetCellId();
        gCellToPosition[cellId] = gnbPositions.at(i);
        gTxNodeToSatPatternGainDb[gnbNodes.Get(i)->GetId()] = 0.0;

        Ptr<MobilityModel> gnbMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
        const double distanceCartesianM = gnbMobility->GetDistanceFrom(ueMobility);
        const double deltaDistFromServingM = distanceCartesianM - minDistance;
        const double elev = ComputeElevationDeg(gnbPositions.at(i), gUePosition);
        constexpr double earthRadiusM = 6371e3;
        const double altitudeM = gnbPositions.at(i).z;
        const double slantRangeM = ComputeTr38811SlantRangeM(elev, altitudeM, earthRadiusM);
        const double offAxisDeg = ComputeOffAxisDeg(gnbPositions.at(i), gUePosition);
        const double satPatternGainDb = useSatAntennaPattern
                            ? ComputeTr38811CircularApertureGainDb(offAxisDeg,
                                                                   carrierFrequencyHz,
                                                                   satApertureRadiusM,
                                                                   gnbAntennaGainDb,
                                                                   satAntennaMinGainDb)
                            : gnbAntennaGainDb;
        const double fsplDb = ComputeFsplDb(slantRangeM, carrierFrequencyHz);
        const double estimatedRxPowerDbmNoShadow =
            gnbTxPowerDbm + satPatternGainDb + ueAntennaGainDb - fsplDb;

        gCellToOffAxisDeg[cellId] = offAxisDeg;
        gCellToSatPatternGainDb[cellId] = satPatternGainDb;
        gTxNodeToSatPatternGainDb[gnbNodes.Get(i)->GetId()] = satPatternGainDb;

        if (distanceCartesianM < minDistance)
        {
            minDistance = distanceCartesianM;
            closestGnbIndex = i;
        }

        gMetricLog << "0ms [CHO-ENV] gnbIndex=" << i
                   << " cellId=" << cellId
                   << " x=" << gnbPositions.at(i).x
                   << " y=" << gnbPositions.at(i).y
                   << " z=" << gnbPositions.at(i).z
                   << " distanceCartesianM=" << distanceCartesianM
                   << " slantRangeM=" << slantRangeM
                   << " elevationDeg=" << elev
                   << " offAxisDeg=" << offAxisDeg
                   << " satPatternGainDb=" << satPatternGainDb
                   << std::endl;

        gCellBudgetLog << "0ms [CELL-BUDGET] role="
                       << ((i == closestGnbIndex) ? "serving" : "neighbor")
                       << " gnbIndex=" << i
                       << " cellId=" << cellId
                       << " distanceCartesianM=" << distanceCartesianM
                       << " slantRangeM=" << slantRangeM
                       << " deltaDistFromServingM=" << deltaDistFromServingM
                       << " elevationDeg=" << elev
                       << " offAxisDeg=" << offAxisDeg
                       << " satPatternGainDb=" << satPatternGainDb
                       << " fsplDb=" << fsplDb
                       << " estimatedRxPowerDbmNoShadow=" << estimatedRxPowerDbmNoShadow
                       << std::endl;
    }

    UpdateLiveCellGeometry(gnbNodes,
                           gnbDevs,
                           carrierFrequencyHz,
                           gnbTxPowerDbm,
                           gnbAntennaGainDb,
                           ueAntennaGainDb,
                           useSatAntennaPattern,
                           satApertureRadiusM,
                           satAntennaMinGainDb,
                           positionUpdatePeriodMs / 1000.0,
                           simTime);

    std::vector<X2DynamicLink> x2Links;
    if (enableX2Interface)
    {
        x2Links.reserve(static_cast<size_t>(gnbNodes.GetN()) *
                        static_cast<size_t>(gnbNodes.GetN() - 1) / 2);
        for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
        {
            for (uint32_t j = i + 1; j < gnbNodes.GetN(); ++j)
            {
                Ptr<MobilityModel> srcMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
                Ptr<MobilityModel> dstMobility = gnbNodes.Get(j)->GetObject<MobilityModel>();
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
                if (srcX2Dev && dstX2Dev)
                {
                    Ptr<PointToPointChannel> x2Channel =
                        DynamicCast<PointToPointChannel>(srcX2Dev->GetChannel());
                    if (x2Channel && srcX2Dev->GetChannel() == dstX2Dev->GetChannel())
                    {
                        x2Links.push_back({gnbNodes.Get(i), gnbNodes.Get(j), x2Channel});
                    }
                }
            }
        }
    }

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
            continue;
        }
        choAlg->TraceConnectWithoutContext("ChoEvent", MakeCallback(&NotifyNrChoEvent));
        choAlg->TraceConnectWithoutContext("ChoDetailedEvent",
                                           MakeCallback(&NotifyNrChoDetailedEvent));
        anyChoTraceConnected = true;
    }
    gMetricLog << "0ms [CHO-HOOK] connected=" << (anyChoTraceConnected ? "true" : "false")
               << std::endl;

    const uint32_t attachIndex = 0;
    nrHelper->AttachToGnb(ueDevs.Get(0), gnbDevs.Get(attachIndex));
    gMetricLog << "0ms [INIT-ATTACH] ueIndex=0 selectedGnbIndex=" << attachIndex
               << " selectedCellId=1 distanceM=" << minDistance << std::endl;

    std::cout << "[CHO-TEST] UE0 attached to center gNB index=" << attachIndex
              << " distanceM=" << minDistance
              << " elevationDeg=" << ComputeElevationDeg(gnbPositions.at(attachIndex), gUePosition)
              << std::endl;

    gMetricLog << "0ms [CHO-TOPOLOGY] ue0=(" << gUePosition.x << ',' << gUePosition.y << ','
               << gUePosition.z << ") satAltitudeM=" << satAltitudeMeters
               << " leoSpeedMps=" << leoSpeedMps
               << " isdM=" << interSiteDistanceMeters
               << " numGnbs=" << numberOfGnbs
               << " numUes=" << numberOfUes
               << " initialAttachGnbIndex=" << attachIndex
               << " initialAttachCellId=1"
               << " initialAttachDistanceM=" << minDistance
               << std::endl;

    if (enablePhyMetricTrace)
    {
        Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportRsrp",
                        MakeCallback(&LogRsrpTrace));
        Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                        MakeCallback(&LogDlDataSinrTrace));
        Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlCtrlSinr",
                        MakeCallback(&LogDlCtrlSinrTrace));
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
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/RecvMeasurementReport",
                    MakeCallback(&NotifyMeasurementReport));
    Config::Connect(
        "/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportUeMeasurements",
        MakeCallback(&LogUeMeasurementTrace));
    if (enableRawCtrlTrace)
    {
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
    }

    ApplicationContainer serverApps;
    ApplicationContainer clientApps;
    const double serverStartSec = snapshotOnce ? 0.001 : 0.05;
    const double clientStartSec = snapshotOnce ? 0.005 : 0.1;
    const double appStopSec = std::max(clientStartSec + 0.001, simTime - 0.0001);
    if (enableTraffic)
    {
        for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
        {
            const uint16_t dlPort = static_cast<uint16_t>(1234 + u);
            UdpServerHelper dlPacketSinkHelper(dlPort);
            serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

            UdpClientHelper dlClient(ueIpIfaces.GetAddress(u), dlPort);
            dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(snapshotOnce ? 1 : 10)));
            dlClient.SetAttribute("PacketSize", UintegerValue(300));
            dlClient.SetAttribute("MaxPackets", UintegerValue(snapshotOnce ? 10 : 1000000));
            clientApps.Add(dlClient.Install(remoteHost));
        }

        serverApps.Start(Seconds(serverStartSec));
        serverApps.Stop(Seconds(appStopSec));
        clientApps.Start(Seconds(clientStartSec));
        clientApps.Stop(Seconds(appStopSec));
    }

    if (!x2Links.empty())
    {
        Simulator::Schedule(Seconds(positionUpdatePeriodMs / 1000.0),
                            &UpdateDynamicX2Delays,
                            x2Links,
                            positionUpdatePeriodMs / 1000.0,
                            simTime);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    WriteServingSinrSummaryCsv(propagationDebugLogFile,
                               servingSinrCsvFile,
                               gnbNodes.Get(closestGnbIndex)->GetId(),
                               bandwidthHz,
                               gUeNoiseFigureDb,
                               ueAntennaGainDb);
    WriteAllUeSinrSnapshotCsv(outputDir + "/conditional-handover-test-all-ue-sinr.csv",
                              snapshotUeRecords,
                              gnbPositions,
                              gnbNodes,
                              carrierFrequencyHz,
                              gnbTxPowerDbm,
                              gnbAntennaGainDb,
                              ueAntennaGainDb,
                              useSatAntennaPattern,
                              satApertureRadiusM,
                              satAntennaMinGainDb,
                              bandwidthHz,
                              gUeNoiseFigureDb,
                              shadowingEnabled);

    gMetricLog.close();
    gSinrBreakdownLog.close();
    gCellBudgetLog.close();
    gCtrlMsgLog.close();
    gGeometryLog.close();
    gHoLog.close();
    gUeMeasurementLog.close();

    std::cout << "[CHO-TEST] done. metricsLog=" << metricLogFile
              << " sinrBreakdownLog=" << sinrBreakdownLogFile
              << " cellBudgetLog=" << cellBudgetLogFile
              << " ctrlMsgLog=" << ctrlMsgLogFile
              << " geometryLog=" << geometryLogFile
              << " hoLog=" << hoLogFile
              << " ueMeasurementLog=" << ueMeasurementLogFile
              << " propagationDebugLog=" << propagationDebugLogFile
              << " servingSinrCsv=" << servingSinrCsvFile
              << " allUeSinrCsv=" << outputDir + "/conditional-handover-test-all-ue-sinr.csv"
              << std::endl;
    return 0;
}
