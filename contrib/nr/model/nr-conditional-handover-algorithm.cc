/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "nr-conditional-handover-algorithm.h"

#include "nr-common.h"
#include "nr-gnb-rrc.h"

#include "nr-gnb-net-device.h"
#include "nr-ue-net-device.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/node-list.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"

#include <algorithm>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("NrConditionalHandoverAlgorithm");

NS_OBJECT_ENSURE_REGISTERED(NrConditionalHandoverAlgorithm);

NrConditionalHandoverAlgorithm::NrConditionalHandoverAlgorithm()
    : m_handoverManagementSapUser(nullptr),
      m_hysteresisDb(3.0),
      m_a3OffsetDb(0.0),
      m_timeToTrigger(MilliSeconds(256)),
      m_choDecisionDelay(MilliSeconds(1)),
      m_hoPreparationDelay(MilliSeconds(8)),
      m_hoPreparationPerTargetOffset(MilliSeconds(1)),
      m_randomAccessDuration(MilliSeconds(6)),
      m_randomAccessStepDelay(MilliSeconds(1)),
      m_pathSwitchDelay(MilliSeconds(2)),
      m_enableHandoverExecution(false)
{
    NS_LOG_FUNCTION(this);
    m_handoverManagementSapProvider =
        new MemberNrHandoverManagementSapProvider<NrConditionalHandoverAlgorithm>(this);
}

NrConditionalHandoverAlgorithm::~NrConditionalHandoverAlgorithm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
NrConditionalHandoverAlgorithm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NrConditionalHandoverAlgorithm")
            .SetParent<NrHandoverAlgorithm>()
            .SetGroupName("Nr")
            .AddConstructor<NrConditionalHandoverAlgorithm>()
            .AddAttribute(
                "Hysteresis",
                "Handover margin (hysteresis) in dB "
                "(rounded to the nearest multiple of 0.5 dB)",
                DoubleValue(3.0),
                MakeDoubleAccessor(&NrConditionalHandoverAlgorithm::m_hysteresisDb),
                MakeDoubleChecker<uint8_t>(0.0, 15.0))
            .AddAttribute("A3Offset",
                          "A3 event offset in dB.",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&NrConditionalHandoverAlgorithm::m_a3OffsetDb),
                          MakeDoubleChecker<double>(-30.0, 30.0))
            .AddAttribute("TimeToTrigger",
                          "Time during which neighbour cell's RSRP must continuously higher than "
                          "serving cell's RSRP in order to prepare CHO",
                          TimeValue(MilliSeconds(256)),
                          MakeTimeAccessor(&NrConditionalHandoverAlgorithm::m_timeToTrigger),
                          MakeTimeChecker())
            .AddAttribute("ChoDecisionDelay",
                          "Serving gNB CHO candidate screening and decision processing delay.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&NrConditionalHandoverAlgorithm::m_choDecisionDelay),
                          MakeTimeChecker())
            .AddAttribute("HoPreparationDelay",
                          "Delay for target-side admission control and preparation ACK.",
                          TimeValue(MilliSeconds(8)),
                          MakeTimeAccessor(&NrConditionalHandoverAlgorithm::m_hoPreparationDelay),
                          MakeTimeChecker())
            .AddAttribute("HoPreparationPerTargetOffset",
                          "Additional admission-control delay per candidate target index.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(
                              &NrConditionalHandoverAlgorithm::m_hoPreparationPerTargetOffset),
                          MakeTimeChecker())
            .AddAttribute("RandomAccessDuration",
                          "Duration of random access before TriggerHandover.",
                          TimeValue(MilliSeconds(6)),
                          MakeTimeAccessor(&NrConditionalHandoverAlgorithm::m_randomAccessDuration),
                          MakeTimeChecker())
            .AddAttribute("RandomAccessStepDelay",
                          "Per-step delay in modeled random access timeline.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&NrConditionalHandoverAlgorithm::m_randomAccessStepDelay),
                          MakeTimeChecker())
            .AddAttribute("PathSwitchDelay",
                          "Modeled delay to complete path switch after handover completion.",
                          TimeValue(MilliSeconds(2)),
                          MakeTimeAccessor(&NrConditionalHandoverAlgorithm::m_pathSwitchDelay),
                          MakeTimeChecker())
            .AddAttribute("EnableHandoverExecution",
                          "If true, trigger actual handover to target cell. If false, run CHO "
                          "pipeline and emit traces only.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&NrConditionalHandoverAlgorithm::m_enableHandoverExecution),
                          MakeBooleanChecker())
            .AddTraceSource(
                "ChoEvent",
                "Detailed CHO event trace: timestampMs, rnti, eventName, targetCellId, "
                "servingRsrp, targetRsrp, candidateCount",
                MakeTraceSourceAccessor(&NrConditionalHandoverAlgorithm::m_choEventTrace),
                "ns3::TracedCallback::Uint64Uint16StringUint16Uint8Uint8Uint16")
            .AddTraceSource("ChoDetailedEvent",
                            "Step-by-step CHO details: timestampMs, rnti, stepName, detail",
                            MakeTraceSourceAccessor(&NrConditionalHandoverAlgorithm::m_choDetailedTrace),
                            "ns3::TracedCallback::Uint64Uint16StringString");
    return tid;
}

void
NrConditionalHandoverAlgorithm::SetNrHandoverManagementSapUser(NrHandoverManagementSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_handoverManagementSapUser = s;
}

NrHandoverManagementSapProvider*
NrConditionalHandoverAlgorithm::GetNrHandoverManagementSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_handoverManagementSapProvider;
}

void
NrConditionalHandoverAlgorithm::DoInitialize()
{
    NS_LOG_FUNCTION(this);

    uint8_t hysteresisIeValue = nr::EutranMeasurementMapping::ActualHysteresis2IeValue(m_hysteresisDb);
    int8_t a3OffsetIeValue = nr::EutranMeasurementMapping::ActualA3Offset2IeValue(m_a3OffsetDb);

    NrRrcSap::ReportConfigEutra reportConfig;
    reportConfig.eventId = NrRrcSap::ReportConfigEutra::EVENT_A3;
    reportConfig.a3Offset = a3OffsetIeValue;
    reportConfig.hysteresis = hysteresisIeValue;
    reportConfig.timeToTrigger = m_timeToTrigger.GetMilliSeconds();
    reportConfig.reportOnLeave = false;
    reportConfig.triggerQuantity = NrRrcSap::ReportConfigEutra::RSRP;
    reportConfig.maxReportCells = 3;
    reportConfig.reportInterval = NrRrcSap::ReportConfigEutra::MS240;
    m_measIds = m_handoverManagementSapUser->AddUeMeasReportConfigForHandover(reportConfig);

    EmitChoEventTrace(0,
                      "AlgorithmInitialized",
                      0,
                      0,
                      0,
                      static_cast<uint16_t>(m_measIds.size()));

    {
        std::ostringstream oss;
        oss << "Config: hysteresisDb=" << m_hysteresisDb
            << " a3OffsetDb=" << m_a3OffsetDb
            << " tttMs=" << m_timeToTrigger.GetMilliSeconds()
            << " decisionDelayMs=" << m_choDecisionDelay.GetMilliSeconds()
            << " hoPreparationDelayMs=" << m_hoPreparationDelay.GetMilliSeconds()
            << " hoPreparationPerTargetOffsetMs="
            << m_hoPreparationPerTargetOffset.GetMilliSeconds()
            << " randomAccessDurationMs=" << m_randomAccessDuration.GetMilliSeconds()
            << " randomAccessStepDelayMs=" << m_randomAccessStepDelay.GetMilliSeconds()
            << " pathSwitchDelayMs=" << m_pathSwitchDelay.GetMilliSeconds()
            << " enableHandoverExecution=" << (m_enableHandoverExecution ? "true" : "false");
        EmitChoDetailedTrace(0, "Step0_ChoConfiguration", oss.str());
    }

    NrHandoverAlgorithm::DoInitialize();
}

void
NrConditionalHandoverAlgorithm::DoDispose()
{
    NS_LOG_FUNCTION(this);

    for (auto& [rnti, state] : m_pendingCho)
    {
        if (state.m_decisionEvent.IsPending())
        {
            state.m_decisionEvent.Cancel();
        }
        if (state.m_ackEvent.IsPending())
        {
            state.m_ackEvent.Cancel();
        }
        if (state.m_choCommandDeliveryEvent.IsPending())
        {
            state.m_choCommandDeliveryEvent.Cancel();
        }
        if (state.m_randomAccessCompleteEvent.IsPending())
        {
            state.m_randomAccessCompleteEvent.Cancel();
        }
    }
    m_pendingCho.clear();

    delete m_handoverManagementSapProvider;
    m_handoverManagementSapProvider = nullptr;

    NrHandoverAlgorithm::DoDispose();
}

void
NrConditionalHandoverAlgorithm::DoReportUeMeas(uint16_t rnti, NrRrcSap::MeasResults measResults)
{
    NS_LOG_FUNCTION(this << rnti << static_cast<uint16_t>(measResults.measId));

    if (std::find(begin(m_measIds), end(m_measIds), measResults.measId) == std::end(m_measIds))
    {
        return;
    }

    if (!measResults.haveMeasResultNeighCells || measResults.measResultListEutra.empty())
    {
        return;
    }

    const Time mrRecvTime = Simulator::Now();
    const Time serviceDelay = ComputeServiceLinkDelay(rnti);
    const Time mrSentTime = mrRecvTime - serviceDelay;
    const uint8_t servingRsrp = measResults.measResultPCell.rsrpResult;

    auto pendingIt = m_pendingCho.find(rnti);

    if (pendingIt != m_pendingCho.end() && pendingIt->second.m_choCommandDelivered)
    {
        for (auto& candidate : pendingIt->second.m_candidates)
        {
            for (const auto& meas : measResults.measResultListEutra)
            {
                if (meas.physCellId == candidate.m_cellId && meas.haveRsrpResult)
                {
                    candidate.m_rsrp = meas.rsrpResult;
                    break;
                }
            }
        }

        pendingIt->second.m_servingRsrp = servingRsrp;

        const uint16_t bestTargetCellId = ComputeBestTargetCellId(pendingIt->second.m_candidates);
        const uint8_t bestTargetRsrp = ComputeBestTargetRsrp(pendingIt->second.m_candidates);
        const double executionThreshold = servingRsrp + m_hysteresisDb + m_a3OffsetDb;

        {
            std::ostringstream oss;
            oss << "actor=UE action=PROCESS stage=ExecutionMonitoring"
                << " bestTargetCellId=" << bestTargetCellId
                << " bestTargetRsrp=" << static_cast<uint16_t>(bestTargetRsrp)
                << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
                << " a3OffsetDb=" << m_a3OffsetDb
                << " hysteresisDb=" << m_hysteresisDb
                << " threshold=" << executionThreshold
                << " condition=(bestTarget > serving+offset+hyst)="
                << (bestTargetRsrp > executionThreshold ? "true" : "false");
            EmitChoDetailedTrace(rnti, "Step3_ExecutionMonitoring", oss.str());
        }

        if (bestTargetRsrp > executionThreshold)
        {
            EmitChoEventTrace(rnti,
                              "ChoExecutionConditionMet",
                              bestTargetCellId,
                              servingRsrp,
                              bestTargetRsrp,
                              pendingIt->second.m_selectedCandidateCount);
            ExecuteCho(rnti);
        }
        return;
    }

    {
        std::ostringstream txOss;
        txOss << "actor=UE action=TX message=MeasurementReport to=Source-gNB"
              << " measId=" << static_cast<uint16_t>(measResults.measId)
              << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
              << " rawNeighborResultCount=" << measResults.measResultListEutra.size();
        EmitChoDetailedTraceAt(mrSentTime.GetMilliSeconds(), rnti, "Step1_MR_TxByUe", txOss.str());

        std::ostringstream rxOss;
        rxOss << "actor=Source-gNB action=RX message=MeasurementReport from=UE"
              << " measId=" << static_cast<uint16_t>(measResults.measId)
              << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
              << " rawNeighborResultCount=" << measResults.measResultListEutra.size();
        EmitChoDetailedTraceAt(mrRecvTime.GetMilliSeconds(), rnti, "Step1_MR_RxBySource", rxOss.str());

        std::ostringstream procOss;
        procOss << "actor=Source-gNB action=PROCESS message=MeasurementReport"
                << " measId=" << static_cast<uint16_t>(measResults.measId)
                << " hysteresisDb=" << m_hysteresisDb;
        EmitChoDetailedTrace(rnti, "Step1_MR_ProcessedBySource", procOss.str());
    }

    std::vector<CandidateInfo> candidates;
    for (auto it = measResults.measResultListEutra.begin(); it != measResults.measResultListEutra.end(); ++it)
    {
        if (!it->haveRsrpResult || !IsValidNeighbour(it->physCellId))
        {
            continue;
        }

        const double candidateThreshold =
            measResults.measResultPCell.rsrpResult + m_hysteresisDb + m_a3OffsetDb;
        if (it->rsrpResult > candidateThreshold)
        {
            candidates.push_back({it->physCellId, it->rsrpResult});
            EmitChoEventTrace(rnti,
                              "A3ConditionSatisfied",
                              it->physCellId,
                              servingRsrp,
                              it->rsrpResult,
                              0);

            std::ostringstream oss;
            oss << "Candidate accepted: cellId=" << it->physCellId
                << " targetRsrp=" << static_cast<uint16_t>(it->rsrpResult)
                << " threshold=" << candidateThreshold
                << " condition=(target > serving+offset+hyst) true";
            EmitChoDetailedTrace(rnti, "Step1_CandidateEvaluation", oss.str());
        }
        else
        {
            std::ostringstream oss;
            oss << "Candidate rejected: cellId=" << it->physCellId
                << " targetRsrp=" << static_cast<uint16_t>(it->rsrpResult)
                << " threshold=" << candidateThreshold
                << " condition=(target > serving+offset+hyst) false";
            EmitChoDetailedTrace(rnti, "Step1_CandidateEvaluation", oss.str());
        }
    }

    if (candidates.empty())
    {
        if (pendingIt != m_pendingCho.end())
        {
            EmitChoEventTrace(rnti, "ChoCancelledConditionLeave", 0, servingRsrp, 0, 0);
            CancelCho(rnti);
        }
        return;
    }

    if (pendingIt != m_pendingCho.end())
    {
        CancelCho(rnti);
    }

    PendingChoState state;
    state.m_candidates = candidates;
    state.m_servingRsrp = servingRsrp;
    state.m_measReportSentTime = mrSentTime;
    state.m_measReportRecvTime = mrRecvTime;
    state.m_decisionStartTime = Simulator::Now();
    state.m_selectedTargetCellId = ComputeBestTargetCellId(candidates);
    state.m_selectedTargetRsrp = ComputeBestTargetRsrp(candidates);
    state.m_selectedCandidateCount = static_cast<uint16_t>(candidates.size());
    state.m_choCommandDelivered = false;
    state.m_lastComputedX2Delay = Time(0);
    state.m_lastComputedServiceDelay = serviceDelay;
    state.m_x2DelayByTarget.clear();

    EmitChoEventTrace(rnti,
                      "ChoPrepareStart",
                      state.m_selectedTargetCellId,
                      servingRsrp,
                      state.m_selectedTargetRsrp,
                      state.m_selectedCandidateCount);

    state.m_decisionEvent = Simulator::Schedule(
        m_choDecisionDelay,
        &NrConditionalHandoverAlgorithm::OnChoDecisionCompleted,
        this,
        rnti);
    m_pendingCho.emplace(rnti, std::move(state));
}

void
NrConditionalHandoverAlgorithm::OnChoDecisionCompleted(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end() || it->second.m_candidates.empty())
    {
        return;
    }

    const Time now = Simulator::Now();
    const int64_t decisionMs = (now - it->second.m_decisionStartTime).GetMilliSeconds();

    {
        std::ostringstream oss;
        oss << "actor=Source-gNB action=PROCESS stage=ChoDecision"
            << " decisionDoneAtMs=" << now.GetMilliSeconds()
            << " decisionDurationMs=" << decisionMs
            << " candidates=" << BuildCandidateListString(it->second.m_candidates)
            << " selectedTargetCellId=" << it->second.m_selectedTargetCellId;
        EmitChoDetailedTrace(rnti, "Step2_ChoDecisionCompleted", oss.str());
    }

    Ptr<NrGnbRrc> gnbRrc = GetObject<NrGnbRrc>();
    if (gnbRrc == nullptr)
    {
        NS_LOG_WARN("CHO algorithm is not aggregated to NrGnbRrc; canceling pending CHO");
        CancelCho(rnti);
        return;
    }

    it->second.m_hoRequestSentTime = now;
    it->second.m_x2DelayByTarget.clear();

    uint16_t acceptedCount = 0;
    Time maxX2Delay = Time(0);
    for (const auto& candidate : it->second.m_candidates)
    {
        const bool accepted = gnbRrc->PrepareConditionalHandover(rnti, candidate.m_cellId);
        if (!accepted)
        {
            NS_LOG_WARN("Conditional handover preparation rejected for rnti="
                        << rnti << " targetCellId=" << candidate.m_cellId);
            continue;
        }

        ++acceptedCount;
        Time x2Delay = ComputeX2PropagationDelay(rnti, candidate.m_cellId);
        if (x2Delay.IsZero())
        {
            x2Delay = NanoSeconds(1);
        }
        it->second.m_x2DelayByTarget[candidate.m_cellId] = x2Delay;
        if (x2Delay > maxX2Delay)
        {
            maxX2Delay = x2Delay;
        }

        std::ostringstream oss;
        oss << "actor=Source-gNB action=TX message=HoPreparationRequest"
            << " via=RealX2 targetCellId=" << candidate.m_cellId
            << " mode=ConditionalDeferredExecution"
            << " x2PropagationDelayUs=" << x2Delay.GetMicroSeconds();
        EmitChoDetailedTraceAt(now.GetMilliSeconds(), rnti, "Step2_HoReq_TxBySource", oss.str());

        std::ostringstream rxOss;
        rxOss << "actor=Target-gNB(cellId=" << candidate.m_cellId
              << ") action=RX message=HoPreparationRequest from=Source-gNB"
              << " via=RealX2";
        EmitChoDetailedTraceAt((now + x2Delay).GetMilliSeconds(),
                               rnti,
                               "Step2_HoReq_RxByTarget",
                               rxOss.str());
    }

    if (acceptedCount == 0)
    {
        CancelCho(rnti);
        return;
    }

    it->second.m_selectedCandidateCount = acceptedCount;
    it->second.m_lastComputedX2Delay = maxX2Delay.IsZero() ? NanoSeconds(1) : maxX2Delay;

    EmitChoEventTrace(rnti,
                      "HoRequestSentToCandidates",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

    it->second.m_ackEvent =
        Simulator::Schedule(it->second.m_lastComputedX2Delay,
                            &NrConditionalHandoverAlgorithm::PollConditionalPreparation,
                            this,
                            rnti);
}

void
NrConditionalHandoverAlgorithm::PollConditionalPreparation(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end())
    {
        return;
    }

    Ptr<NrGnbRrc> gnbRrc = GetObject<NrGnbRrc>();
    if (gnbRrc == nullptr)
    {
        CancelCho(rnti);
        return;
    }

    const uint16_t preparedTargetCellId =
        ComputeBestPreparedTargetCellId(gnbRrc, rnti, it->second.m_candidates);
    if (preparedTargetCellId != 0)
    {
        it->second.m_selectedTargetCellId = preparedTargetCellId;
        it->second.m_selectedTargetRsrp =
            ComputeTargetRsrpByCellId(it->second.m_candidates, preparedTargetCellId);
        OnHandoverPreparationCompleted(rnti);
        return;
    }

    Time retryDelay = it->second.m_lastComputedX2Delay;
    if (retryDelay.IsZero())
    {
        retryDelay = NanoSeconds(1);
    }
    it->second.m_ackEvent =
        Simulator::Schedule(retryDelay,
                            &NrConditionalHandoverAlgorithm::PollConditionalPreparation,
                            this,
                            rnti);
}

void
NrConditionalHandoverAlgorithm::OnHandoverPreparationCompleted(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end() || it->second.m_candidates.empty())
    {
        return;
    }

    EmitChoEventTrace(rnti,
                      "HoRequestAckReceived",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

    {
        const Time now = Simulator::Now();
        const auto delayIt = it->second.m_x2DelayByTarget.find(it->second.m_selectedTargetCellId);
        const Time x2Delay = (delayIt != it->second.m_x2DelayByTarget.end())
                     ? delayIt->second
                     : (it->second.m_lastComputedX2Delay.IsZero()
                        ? NanoSeconds(1)
                        : it->second.m_lastComputedX2Delay);
        const Time targetAckTx = now - x2Delay;
        const Time targetProcStart = it->second.m_hoRequestSentTime + x2Delay;

        std::ostringstream txAckOss;
        txAckOss << "actor=Target-gNB(cellId=" << it->second.m_selectedTargetCellId
                 << ") action=TX message=HoPreparationAck to=Source-gNB via=RealX2";
        EmitChoDetailedTraceAt(targetAckTx.GetMilliSeconds(),
                               rnti,
                               "Step2_HoAck_TxByTarget",
                               txAckOss.str());

        std::ostringstream procOss;
        procOss << "actor=Target-gNB(cellId=" << it->second.m_selectedTargetCellId
                << ") action=PROCESS stage=AdmissionControl"
                << " startAtMs=" << targetProcStart.GetMilliSeconds()
                << " doneAtMs=" << targetAckTx.GetMilliSeconds()
                << " mode=RealX2";
        EmitChoDetailedTraceAt(targetAckTx.GetMilliSeconds(),
                               rnti,
                               "Step2_AdmissionProcessedByTarget",
                               procOss.str());

        std::ostringstream oss;
        oss << "actor=Source-gNB action=RX message=HoPreparationAck(targetCellId="
            << it->second.m_selectedTargetCellId << ")"
            << " allAckReceivedAtMs=" << Simulator::Now().GetMilliSeconds()
            << "ms mode=RealX2ConditionalPreparation"
            << " x2PropagationDelayUs=" << x2Delay.GetMicroSeconds();
        EmitChoDetailedTrace(rnti, "Step2_AdmissionControlAndAck", oss.str());
    }

    EmitChoEventTrace(rnti,
                      "RrcReconfigurationWithChoPrepared",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

        const Time now = Simulator::Now();
        const Time serviceDelay = ComputeServiceLinkDelay(rnti);
        it->second.m_lastComputedServiceDelay = serviceDelay;

        EmitChoDetailedTrace(
        rnti,
        "Step3_ChoPreparedAtSource",
        "actor=Source-gNB action=STORE message=RrcReconfiguration(CHO) mode=DeferredUntilCondition");

    {
        std::ostringstream txOss;
        txOss << "actor=Source-gNB action=TX message=RrcReconfiguration(CHO) to=UE"
              << " mode=DeferredUntilCondition serviceDelayUs=" << serviceDelay.GetMicroSeconds();
        EmitChoDetailedTraceAt(
            now.GetMilliSeconds(),
            rnti,
            "Step3_RrcReconfigCho_TxBySource",
            txOss.str());

        std::ostringstream rxOss;
        rxOss << "actor=UE action=RX message=RrcReconfiguration(CHO) from=Source-gNB"
              << " mode=DeferredUntilCondition";
        EmitChoDetailedTraceAt((now + serviceDelay).GetMilliSeconds(),
                               rnti,
                               "Step3_RrcReconfigCho_RxByUe",
                               rxOss.str());
    }

    it->second.m_choCommandDelivered = false;
    it->second.m_choCommandDeliveryEvent =
        Simulator::Schedule(serviceDelay,
                            &NrConditionalHandoverAlgorithm::OnChoCommandDeliveredToUe,
                            this,
                            rnti);
}

void
NrConditionalHandoverAlgorithm::OnChoCommandDeliveredToUe(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end() || it->second.m_candidates.empty())
    {
        return;
    }

    EmitChoEventTrace(rnti,
                      "ChoStoredInUe",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

    {
        std::ostringstream oss;
        oss << "actor=UE action=STORE choContextAtMs=" << Simulator::Now().GetMilliSeconds()
            << " candidates=" << BuildCandidateListString(it->second.m_candidates)
            << " executionMonitoring=STARTED";
        EmitChoDetailedTrace(rnti, "Step3_UeReceivedChoCommand", oss.str());
    }

    it->second.m_choCommandDelivered = true;

    const double executionThreshold = it->second.m_servingRsrp + m_hysteresisDb + m_a3OffsetDb;
    if (it->second.m_selectedTargetRsrp > executionThreshold)
    {
        EmitChoEventTrace(rnti,
                          "ChoExecutionConditionMet",
                          it->second.m_selectedTargetCellId,
                          it->second.m_servingRsrp,
                          it->second.m_selectedTargetRsrp,
                          it->second.m_selectedCandidateCount);
        ExecuteCho(rnti);
    }
}

void
NrConditionalHandoverAlgorithm::ExecuteCho(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end() || it->second.m_candidates.empty())
    {
        return;
    }

    Ptr<NrGnbRrc> gnbRrc = GetObject<NrGnbRrc>();
    if (gnbRrc == nullptr)
    {
        CancelCho(rnti);
        return;
    }

    const uint16_t targetCellId =
        ComputeBestPreparedTargetCellId(gnbRrc, rnti, it->second.m_candidates);
    if (targetCellId == 0)
    {
        CancelCho(rnti);
        return;
    }
    const uint8_t targetRsrp = ComputeTargetRsrpByCellId(it->second.m_candidates, targetCellId);

    it->second.m_selectedTargetCellId = targetCellId;
    it->second.m_selectedTargetRsrp = targetRsrp;

    EmitChoEventTrace(rnti,
                      "ChoExecutionStart",
                      targetCellId,
                      it->second.m_servingRsrp,
                      targetRsrp,
                      it->second.m_selectedCandidateCount);

    {
        std::ostringstream oss;
        oss << "actor=UE action=PROCESS stage=ChoExecutionCondition"
            << " conditionMetAtMs=" << Simulator::Now().GetMilliSeconds()
            << " selectedTargetCellId=" << targetCellId
            << " selectedTargetRsrp=" << static_cast<uint16_t>(targetRsrp)
            << " servingRsrp=" << static_cast<uint16_t>(it->second.m_servingRsrp)
            << " candidates=" << BuildCandidateListString(it->second.m_candidates);
        EmitChoDetailedTrace(rnti, "Step4_UeSelectionDecision", oss.str());
    }

    EmitChoDetailedTrace(
        rnti,
        "Step5_ExecutePreparedCho",
        "actor=Source-gNB action=EXECUTE preparedChoCommand targetCellId=" +
            std::to_string(targetCellId));

    {
        const Time tx = Simulator::Now();
        const Time serviceDelay = it->second.m_lastComputedServiceDelay.IsZero()
                                      ? ComputeServiceLinkDelay(rnti)
                                      : it->second.m_lastComputedServiceDelay;
        EmitChoDetailedTraceAt(tx.GetMilliSeconds(),
                               rnti,
                               "Step5_RrcReconfigExec_TxBySource",
                               "actor=Source-gNB action=TX message=RrcReconfiguration(CHO-Execute)");
        EmitChoDetailedTraceAt((tx + serviceDelay).GetMilliSeconds(),
                               rnti,
                               "Step5_RrcReconfigExec_RxByUe",
                               "actor=UE action=RX message=RrcReconfiguration(CHO-Execute)");
    }

    if (m_enableHandoverExecution)
    {
        const bool ok = gnbRrc->ExecuteConditionalHandover(rnti, targetCellId);
        EmitChoEventTrace(rnti,
                          ok ? "TriggerHandoverCalled" : "TriggerHandoverSkipped",
                          it->second.m_selectedTargetCellId,
                          it->second.m_servingRsrp,
                          it->second.m_selectedTargetRsrp,
                          it->second.m_selectedCandidateCount);
    }
    else
    {
        EmitChoEventTrace(rnti,
                          "TriggerHandoverSkipped",
                          it->second.m_selectedTargetCellId,
                          it->second.m_servingRsrp,
                          it->second.m_selectedTargetRsrp,
                          it->second.m_selectedCandidateCount);
    }

    m_pendingCho.erase(it);
}

Time
NrConditionalHandoverAlgorithm::ComputeLightSpeedDelay(double distanceMeters)
{
    static constexpr double kLightSpeed = 299792458.0;
    return Seconds(distanceMeters / kLightSpeed);
}

Ptr<Node>
NrConditionalHandoverAlgorithm::FindGnbNodeByCellId(uint16_t cellId) const
{
    auto listEnd = NodeList::End();
    for (auto nodeIt = NodeList::Begin(); nodeIt != listEnd; ++nodeIt)
    {
        Ptr<Node> node = *nodeIt;
        for (uint32_t devIndex = 0; devIndex < node->GetNDevices(); ++devIndex)
        {
            Ptr<NrGnbNetDevice> gnbDev = node->GetDevice(devIndex)->GetObject<NrGnbNetDevice>();
            if (gnbDev && gnbDev->GetCellId() == cellId)
            {
                return node;
            }
        }
    }
    return nullptr;
}

Ptr<Node>
NrConditionalHandoverAlgorithm::FindUeNodeByRnti(uint16_t rnti) const
{
    Ptr<NrGnbRrc> gnbRrc = GetObject<NrGnbRrc>();
    if (gnbRrc == nullptr || !gnbRrc->HasUeManager(rnti))
    {
        return nullptr;
    }

    const uint64_t imsi = gnbRrc->GetUeManager(rnti)->GetImsi();
    auto listEnd = NodeList::End();
    for (auto nodeIt = NodeList::Begin(); nodeIt != listEnd; ++nodeIt)
    {
        Ptr<Node> node = *nodeIt;
        for (uint32_t devIndex = 0; devIndex < node->GetNDevices(); ++devIndex)
        {
            Ptr<NrUeNetDevice> ueDev = node->GetDevice(devIndex)->GetObject<NrUeNetDevice>();
            if (ueDev && ueDev->GetImsi() == imsi)
            {
                return node;
            }
        }
    }
    return nullptr;
}

Time
NrConditionalHandoverAlgorithm::ComputeServiceLinkDelay(uint16_t rnti) const
{
    Ptr<NrGnbRrc> gnbRrc = GetObject<NrGnbRrc>();
    if (gnbRrc == nullptr || !gnbRrc->HasUeManager(rnti))
    {
        return Time(0);
    }

    Ptr<NrUeManager> ueManager = gnbRrc->GetUeManager(rnti);
    const uint16_t servingCellId = gnbRrc->ComponentCarrierToCellId(ueManager->GetComponentCarrierId());
    Ptr<Node> gnbNode = FindGnbNodeByCellId(servingCellId);
    Ptr<Node> ueNode = FindUeNodeByRnti(rnti);
    if (gnbNode == nullptr || ueNode == nullptr)
    {
        return Time(0);
    }

    Ptr<MobilityModel> gnbMobility = gnbNode->GetObject<MobilityModel>();
    Ptr<MobilityModel> ueMobility = ueNode->GetObject<MobilityModel>();
    if (gnbMobility == nullptr || ueMobility == nullptr)
    {
        return Time(0);
    }

    return ComputeLightSpeedDelay(gnbMobility->GetDistanceFrom(ueMobility));
}

Time
NrConditionalHandoverAlgorithm::ComputeX2PropagationDelay(uint16_t rnti, uint16_t targetCellId) const
{
    Ptr<NrGnbRrc> gnbRrc = GetObject<NrGnbRrc>();
    if (gnbRrc == nullptr || !gnbRrc->HasUeManager(rnti))
    {
        return Time(0);
    }

    Ptr<NrUeManager> ueManager = gnbRrc->GetUeManager(rnti);
    const uint16_t sourceCellId = gnbRrc->ComponentCarrierToCellId(ueManager->GetComponentCarrierId());
    Ptr<Node> sourceNode = FindGnbNodeByCellId(sourceCellId);
    Ptr<Node> targetNode = FindGnbNodeByCellId(targetCellId);
    if (sourceNode == nullptr || targetNode == nullptr)
    {
        return Time(0);
    }

    Ptr<MobilityModel> sourceMobility = sourceNode->GetObject<MobilityModel>();
    Ptr<MobilityModel> targetMobility = targetNode->GetObject<MobilityModel>();
    if (sourceMobility == nullptr || targetMobility == nullptr)
    {
        return Time(0);
    }

    return ComputeLightSpeedDelay(sourceMobility->GetDistanceFrom(targetMobility));
}

void
NrConditionalHandoverAlgorithm::CompleteChoAfterRandomAccess(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end() || it->second.m_selectedTargetCellId == 0)
    {
        return;
    }

    EmitChoDetailedTrace(rnti,
                         "Step5_RandomAccessComplete",
                         "actor=Target-gNB action=PROCESS message=RrcReconfigurationComplete"
                         " result=RandomAccessCompleted atMs=" +
                             std::to_string(Simulator::Now().GetMilliSeconds()));

    EmitChoEventTrace(rnti,
                      "TriggerHandoverCalled",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

    {
        const uint16_t canceledTargets = it->second.m_selectedCandidateCount > 0
                                             ? static_cast<uint16_t>(it->second.m_selectedCandidateCount - 1)
                                             : 0;
        std::ostringstream oss;
        oss << "actor=Source-gNB action=PROCESS stage=PathSwitchAndChoCleanup"
            << " startMs=" << Simulator::Now().GetMilliSeconds()
            << " doneMs=" << (Simulator::Now() + m_pathSwitchDelay).GetMilliSeconds()
            << " canceledTargetCount=" << canceledTargets;
        EmitChoDetailedTrace(rnti, "Step6_7_PathSwitchAndCancelOthers", oss.str());
    }

    if (m_enableHandoverExecution)
    {
        m_handoverManagementSapUser->TriggerHandover(rnti, it->second.m_selectedTargetCellId);
    }
    else
    {
        EmitChoEventTrace(rnti,
                          "TriggerHandoverSkipped",
                          it->second.m_selectedTargetCellId,
                          it->second.m_servingRsrp,
                          it->second.m_selectedTargetRsrp,
                          it->second.m_selectedCandidateCount);
    }

    m_pendingCho.erase(it);
}

void
NrConditionalHandoverAlgorithm::CancelCho(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end())
    {
        return;
    }

    if (it->second.m_decisionEvent.IsPending())
    {
        it->second.m_decisionEvent.Cancel();
    }
    if (it->second.m_ackEvent.IsPending())
    {
        it->second.m_ackEvent.Cancel();
    }
    if (it->second.m_choCommandDeliveryEvent.IsPending())
    {
        it->second.m_choCommandDeliveryEvent.Cancel();
    }
    if (it->second.m_randomAccessCompleteEvent.IsPending())
    {
        it->second.m_randomAccessCompleteEvent.Cancel();
    }

    EmitChoEventTrace(rnti, "ChoPendingEntryRemoved", 0, 0, 0, 0);
    m_pendingCho.erase(it);
}

void
NrConditionalHandoverAlgorithm::EmitChoEventTrace(uint16_t rnti,
                                                  const std::string& eventName,
                                                  uint16_t targetCellId,
                                                  uint8_t servingRsrp,
                                                  uint8_t targetRsrp,
                                                  uint16_t candidateCount)
{
    m_choEventTrace(Simulator::Now().GetMilliSeconds(),
                    rnti,
                    eventName,
                    targetCellId,
                    servingRsrp,
                    targetRsrp,
                    candidateCount);
}

void
NrConditionalHandoverAlgorithm::EmitChoDetailedTrace(uint16_t rnti,
                                                     const std::string& stepName,
                                                     const std::string& detail)
{
    EmitChoDetailedTraceAt(Simulator::Now().GetMilliSeconds(), rnti, stepName, detail);
}

void
NrConditionalHandoverAlgorithm::EmitChoDetailedTraceAt(uint64_t timestampMs,
                                                       uint16_t rnti,
                                                       const std::string& stepName,
                                                       const std::string& detail)
{
    m_choDetailedTrace(timestampMs, rnti, stepName, detail);
}

std::string
NrConditionalHandoverAlgorithm::BuildCandidateListString(
    const std::vector<CandidateInfo>& candidates) const
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < candidates.size(); ++i)
    {
        oss << "{cellId=" << candidates[i].m_cellId << ",rsrp="
            << static_cast<uint16_t>(candidates[i].m_rsrp) << "}";
        if (i + 1 < candidates.size())
        {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

std::string
NrConditionalHandoverAlgorithm::BuildTargetReceiveTimesString(const PendingChoState& state) const
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < state.m_candidates.size(); ++i)
    {
        const auto delayIt = state.m_x2DelayByTarget.find(state.m_candidates[i].m_cellId);
        const Time x2Delay = (delayIt != state.m_x2DelayByTarget.end())
                                 ? delayIt->second
                                 : (state.m_lastComputedX2Delay.IsZero() ? NanoSeconds(1)
                                                                          : state.m_lastComputedX2Delay);
        const Time rx = state.m_hoRequestSentTime + x2Delay;
        oss << "{cellId=" << state.m_candidates[i].m_cellId << ",rxMs=" << rx.GetMilliSeconds()
            << "}";
        if (i + 1 < state.m_candidates.size())
        {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

std::string
NrConditionalHandoverAlgorithm::BuildAdmissionAckTimesString(const PendingChoState& state) const
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < state.m_candidates.size(); ++i)
    {
        const auto delayIt = state.m_x2DelayByTarget.find(state.m_candidates[i].m_cellId);
        const Time x2Delay = (delayIt != state.m_x2DelayByTarget.end())
                                 ? delayIt->second
                                 : (state.m_lastComputedX2Delay.IsZero() ? NanoSeconds(1)
                                                                          : state.m_lastComputedX2Delay);
        const Time hoReqRx = state.m_hoRequestSentTime + x2Delay;
        const Time offset =
            NanoSeconds(m_hoPreparationPerTargetOffset.GetNanoSeconds() * static_cast<int64_t>(i));
        const Time admissionDone = hoReqRx + m_hoPreparationDelay + offset;
        const Time ackTx = admissionDone;
        const Time ackRx = ackTx + x2Delay;

        oss << "{cellId=" << state.m_candidates[i].m_cellId
            << ",hoReqRxMs=" << hoReqRx.GetMilliSeconds()
            << ",admissionDoneMs=" << admissionDone.GetMilliSeconds()
            << ",ackTxMs=" << ackTx.GetMilliSeconds() << ",ackRxMs=" << ackRx.GetMilliSeconds()
            << "}";
        if (i + 1 < state.m_candidates.size())
        {
            oss << ",";
        }
    }
    oss << "]";
    return oss.str();
}

Time
NrConditionalHandoverAlgorithm::ComputeAllAckReceivedTime(const PendingChoState& state) const
{
    if (state.m_candidates.empty())
    {
        return Simulator::Now();
    }

    Time maxAckRx = state.m_hoRequestSentTime;
    for (size_t i = 0; i < state.m_candidates.size(); ++i)
    {
        const auto delayIt = state.m_x2DelayByTarget.find(state.m_candidates[i].m_cellId);
        const Time x2Delay = (delayIt != state.m_x2DelayByTarget.end())
                                 ? delayIt->second
                                 : (state.m_lastComputedX2Delay.IsZero() ? NanoSeconds(1)
                                                                          : state.m_lastComputedX2Delay);
        const Time hoReqRx = state.m_hoRequestSentTime + x2Delay;
        const Time offset =
            NanoSeconds(m_hoPreparationPerTargetOffset.GetNanoSeconds() * static_cast<int64_t>(i));
        const Time admissionDone = hoReqRx + m_hoPreparationDelay + offset;
        const Time ackRx = admissionDone + x2Delay;
        if (ackRx > maxAckRx)
        {
            maxAckRx = ackRx;
        }
    }
    return maxAckRx;
}

uint16_t
NrConditionalHandoverAlgorithm::ComputeBestTargetCellId(const std::vector<CandidateInfo>& candidates) const
{
    const auto best = std::max_element(
        candidates.begin(),
        candidates.end(),
        [](const CandidateInfo& lhs, const CandidateInfo& rhs) { return lhs.m_rsrp < rhs.m_rsrp; });
    return (best == candidates.end()) ? 0 : best->m_cellId;
}

uint8_t
NrConditionalHandoverAlgorithm::ComputeBestTargetRsrp(const std::vector<CandidateInfo>& candidates) const
{
    const auto best = std::max_element(
        candidates.begin(),
        candidates.end(),
        [](const CandidateInfo& lhs, const CandidateInfo& rhs) { return lhs.m_rsrp < rhs.m_rsrp; });
    return (best == candidates.end()) ? 0 : best->m_rsrp;
}

uint16_t
NrConditionalHandoverAlgorithm::ComputeBestPreparedTargetCellId(
    Ptr<NrGnbRrc> gnbRrc,
    uint16_t rnti,
    const std::vector<CandidateInfo>& candidates) const
{
    if (gnbRrc == nullptr)
    {
        return 0;
    }

    uint16_t bestCellId = 0;
    uint8_t bestRsrp = 0;
    bool found = false;
    for (const auto& candidate : candidates)
    {
        if (!gnbRrc->IsConditionalHandoverPrepared(rnti, candidate.m_cellId))
        {
            continue;
        }

        if (!found || candidate.m_rsrp > bestRsrp)
        {
            bestCellId = candidate.m_cellId;
            bestRsrp = candidate.m_rsrp;
            found = true;
        }
    }

    return bestCellId;
}

uint8_t
NrConditionalHandoverAlgorithm::ComputeTargetRsrpByCellId(const std::vector<CandidateInfo>& candidates,
                                                           uint16_t targetCellId) const
{
    for (const auto& candidate : candidates)
    {
        if (candidate.m_cellId == targetCellId)
        {
            return candidate.m_rsrp;
        }
    }

    return 0;
}

bool
NrConditionalHandoverAlgorithm::IsValidNeighbour(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    return true;
}

} // namespace ns3
