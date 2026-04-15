#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
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

NS_LOG_COMPONENT_DEFINE("NrX2HandoverMeasuresChannelTest");

std::ofstream gMetricLog;
std::ofstream gSinrBreakdownLog;
std::ofstream gCellBudgetLog;
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

static double ComputeElevationDeg(const Vector& satPos, const Vector& uePos);
static double ComputeOffAxisDeg(const Vector& satPos, const Vector& uePos);
static double ComputeTr38811CircularApertureGainDb(double offAxisDeg,
                                                   double frequencyHz,
                                                   double apertureRadiusM,
                                                   double maxGainDb,
                                                   double minGainDb);

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

    std::cout << Simulator::Now().GetMilliSeconds() << "ms [CH-METRIC] type=RSRP"
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

    std::cout << Simulator::Now().GetMilliSeconds() << "ms [CH-METRIC] type=SINR_DL_DATA"
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

    std::cout << Simulator::Now().GetMilliSeconds() << "ms [CH-METRIC] type=SINR_DL_CTRL"
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
    const std::string metricLogFile = outputDir + "/channel-test-metrics.log";
    const std::string sinrBreakdownLogFile = outputDir + "/channel-test-sinr-breakdown.log";
    const std::string cellBudgetLogFile = outputDir + "/channel-test-cell-budget.log";
    const std::string propagationDebugLogFile = outputDir + "/channel-test-propagation-debug.csv";
    const std::string servingSinrCsvFile = outputDir + "/channel-test-serving-sinr.csv";
    gMetricLog.open(metricLogFile);
    gSinrBreakdownLog.open(sinrBreakdownLogFile);
    gCellBudgetLog.open(cellBudgetLogFile);
    std::ofstream(propagationDebugLogFile, std::ios::trunc).close();
    std::ofstream(servingSinrCsvFile, std::ios::trunc).close();

    uint16_t numberOfUes = 1;
    double simTime = 3.0;
    bool useIdealRrc = false;

    double satAltitudeMeters = 600000.0;
    double interSiteDistanceMeters = 41750.0;
    uint32_t gnbTiers = 4;
    uint32_t snapshotUeAnchorTiers = 2;
    double ueLatitudeDeg = 0.0;
    double ueLongitudeDeg = 0.0;

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
    uint32_t snapshotUesPerCell = 1000;
    double snapshotCellRadiusMeters = 25000.0;

    bool enableMetricLog = true;
    int64_t metricStartMs = -1;
    int64_t metricEndMs = -1;
    bool snapshotOnce = true;
    double snapshotDurationMs = 300.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTime", "Simulation duration [s]", simTime);
    cmd.AddValue("useIdealRrc", "Use idealized RRC", useIdealRrc);
    cmd.AddValue("satAltitudeMeters", "gNB(satellite) altitude [m]", satAltitudeMeters);
    cmd.AddValue("interSiteDistanceMeters",
                 "Hex-cell inter-site distance [m] (default 43.3 km)",
                 interSiteDistanceMeters);
    cmd.AddValue("gnbTiers", "Number of hex tiers used for gNB deployment (4 => 61 gNBs)", gnbTiers);
    cmd.AddValue("snapshotUeAnchorTiers",
                 "Number of inner hex tiers used as UE anchor beams (2 => inner 19 beams)",
                 snapshotUeAnchorTiers);
    cmd.AddValue("ueLatitudeDeg", "UE latitude [deg]", ueLatitudeDeg);
    cmd.AddValue("ueLongitudeDeg", "UE longitude [deg]", ueLongitudeDeg);
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
    cmd.AddValue("enableMetricLog", "Enable channel metric logging", enableMetricLog);
    cmd.AddValue("metricStartMs", "Metric log start [ms], -1 means beginning", metricStartMs);
    cmd.AddValue("metricEndMs", "Metric log end [ms], -1 means end", metricEndMs);
    cmd.AddValue("snapshotOnce",
                 "If true, capture one static snapshot and exit quickly (no periodic traffic traces)",
                 snapshotOnce);
    cmd.AddValue("snapshotDurationMs",
                 "Simulation duration used in snapshotOnce mode [ms]",
                 snapshotDurationMs);
    cmd.Parse(argc, argv);

    if (useSatAntennaPattern && !useIdealRrc)
    {
        std::cout << "[CH-TEST] useSatAntennaPattern=true requires stable idealized RRC in this setup; "
                  << "forcing useIdealRrc=true" << std::endl;
        useIdealRrc = true;
    }

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
        std::cout << "[CH-TEST] snapshotOnce=true, overriding simTime to " << simTime << "s"
                  << std::endl;
    }

    Config::SetDefault("ns3::UdpClient::Interval", TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::UdpClient::MaxPackets", UintegerValue(1000000));

    const std::vector<Vector> gnbPositions =
        CreateHexPositions(interSiteDistanceMeters, satAltitudeMeters, gnbTiers);
    const std::vector<Vector> snapshotUeAnchorPositions =
        CreateHexPositions(interSiteDistanceMeters, satAltitudeMeters, snapshotUeAnchorTiers);
    const uint16_t numberOfGnbs = static_cast<uint16_t>(gnbPositions.size());

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

    gUePosition = Vector(0.0, 0.0, 0.0);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        auto ueGeo = ueNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        ueGeo->SetCoordinateTranslationReferencePoint(referencePoint);
        ueGeo->SetGeographicPosition(Vector(ueLatitudeDeg, ueLongitudeDeg, 0.0));
    }

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        const double deltaLatDeg = (gnbPositions.at(i).y / earthRadiusM) * 180.0 / M_PI;
        const double deltaLonDeg = (gnbPositions.at(i).x / (earthRadiusM * cosLat)) * 180.0 / M_PI;

        auto gnbGeo = gnbNodes.Get(i)->GetObject<GeocentricConstantPositionMobilityModel>();
        gnbGeo->SetCoordinateTranslationReferencePoint(referencePoint);
        gnbGeo->SetGeographicPosition(
            Vector(ueLatitudeDeg + deltaLatDeg, ueLongitudeDeg + deltaLonDeg, satAltitudeMeters));
    }

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
    nrHelper->SetHandoverAlgorithmType("ns3::NrNoOpHandoverAlgorithm");

    Config::SetDefault("ns3::ThreeGppPropagationLossModel::ShadowingEnabled",
                       BooleanValue(shadowingEnabled));
    Config::SetDefault("ns3::ThreeGppPropagationLossModel::DebugLogEnabled", BooleanValue(true));
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

        std::cout << "[CH-TEST] gNB antenna pattern=CircularAperture"
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
        std::cout << "[CH-TEST] gNB antenna pattern=Isotropic"
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

    std::cout << "[CH-TEST] gNB array orientation="
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

    double minDistance = std::numeric_limits<double>::max();
    uint32_t closestGnbIndex = 0;
    Ptr<MobilityModel> ueMobility = ueNodes.Get(0)->GetObject<MobilityModel>();
    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        Ptr<MobilityModel> gnbMobility = gnbNodes.Get(i)->GetObject<MobilityModel>();
        const double dist = gnbMobility->GetDistanceFrom(ueMobility);
        if (dist < minDistance)
        {
            minDistance = dist;
            closestGnbIndex = i;
        }
    }

    nrHelper->AttachToGnb(ueDevs.Get(0), gnbDevs.Get(closestGnbIndex));

    std::cout << "[CH-TEST] UE attached to closest gNB index=" << closestGnbIndex
              << " distanceM=" << minDistance
              << " elevationDeg=" << ComputeElevationDeg(gnbPositions.at(closestGnbIndex), gUePosition)
              << std::endl;

    gMetricLog << "0ms [CH-TOPOLOGY] ue=(0,0,0) satAltitudeM=" << satAltitudeMeters
               << " isdM=" << interSiteDistanceMeters
               << " numGnbs=" << numberOfGnbs
               << " selectedClosestGnbIndex=" << closestGnbIndex
               << " selectedDistanceM=" << minDistance
               << std::endl;

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

        std::cout << "0ms [CH-ENV] gnbIndex=" << i
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

        gMetricLog << "0ms [CH-ENV] gnbIndex=" << i
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

    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/ReportRsrp",
                    MakeCallback(&LogRsrpTrace));
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlDataSinr",
                    MakeCallback(&LogDlDataSinrTrace));
    Config::Connect("/NodeList/*/DeviceList/*/ComponentCarrierMapUe/*/NrUePhy/DlCtrlSinr",
                    MakeCallback(&LogDlCtrlSinrTrace));

    uint16_t dlPort = 1234;
    UdpServerHelper dlPacketSinkHelper(dlPort);
    ApplicationContainer serverApps = dlPacketSinkHelper.Install(ueNodes.Get(0));
    const double serverStartSec = snapshotOnce ? 0.001 : 0.05;
    const double clientStartSec = snapshotOnce ? 0.005 : 0.1;
    const double appStopSec = std::max(clientStartSec + 0.001, simTime - 0.0001);
    serverApps.Start(Seconds(serverStartSec));
    serverApps.Stop(Seconds(appStopSec));

    UdpClientHelper dlClient(ueIpIfaces.GetAddress(0), dlPort);
    dlClient.SetAttribute("Interval", TimeValue(MilliSeconds(snapshotOnce ? 1 : 10)));
    dlClient.SetAttribute("PacketSize", UintegerValue(300));
    dlClient.SetAttribute("MaxPackets", UintegerValue(snapshotOnce ? 10 : 1000000));
    ApplicationContainer clientApps = dlClient.Install(remoteHost);
    clientApps.Start(Seconds(clientStartSec));
    clientApps.Stop(Seconds(appStopSec));

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    WriteServingSinrSummaryCsv(propagationDebugLogFile,
                               servingSinrCsvFile,
                               closestGnbIndex,
                               bandwidthHz,
                               gUeNoiseFigureDb,
                               ueAntennaGainDb);
    WriteAllUeSinrSnapshotCsv(outputDir + "/channel-test-all-ue-sinr.csv",
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

    std::cout << "[CH-TEST] done. metricsLog=" << metricLogFile
              << " sinrBreakdownLog=" << sinrBreakdownLogFile
              << " cellBudgetLog=" << cellBudgetLogFile
              << " propagationDebugLog=" << propagationDebugLogFile
              << " servingSinrCsv=" << servingSinrCsvFile
              << " allUeSinrCsv=" << outputDir + "/channel-test-all-ue-sinr.csv" << std::endl;
    return 0;
}
