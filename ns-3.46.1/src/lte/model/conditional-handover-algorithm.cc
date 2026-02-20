/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "conditional-handover-algorithm.h"

#include "lte-common.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"

#include <algorithm>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("ConditionalHandoverAlgorithm");

NS_OBJECT_ENSURE_REGISTERED(ConditionalHandoverAlgorithm);

ConditionalHandoverAlgorithm::ConditionalHandoverAlgorithm()
    : m_handoverManagementSapUser(nullptr),
      m_hysteresisDb(3.0),
      m_timeToTrigger(MilliSeconds(256)),
      m_measurementReportDelay(MilliSeconds(1)),
      m_choDecisionDelay(MilliSeconds(1)),
      m_hoRequestPropagationDelay(MilliSeconds(1)),
      m_hoPreparationDelay(MilliSeconds(8)),
      m_hoPreparationPerTargetOffset(MilliSeconds(1)),
      m_choCommandDeliveryDelay(MilliSeconds(2)),
      m_randomAccessDuration(MilliSeconds(6)),
      m_randomAccessStepDelay(MilliSeconds(1)),
      m_pathSwitchDelay(MilliSeconds(2))
{
    NS_LOG_FUNCTION(this);
    m_handoverManagementSapProvider =
        new MemberLteHandoverManagementSapProvider<ConditionalHandoverAlgorithm>(this);
}

ConditionalHandoverAlgorithm::~ConditionalHandoverAlgorithm()
{
    NS_LOG_FUNCTION(this);
}

TypeId
ConditionalHandoverAlgorithm::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::ConditionalHandoverAlgorithm")
            .SetParent<LteHandoverAlgorithm>()
            .SetGroupName("Lte")
            .AddConstructor<ConditionalHandoverAlgorithm>()
            .AddAttribute(
                "Hysteresis",
                "Handover margin (hysteresis) in dB "
                "(rounded to the nearest multiple of 0.5 dB)",
                DoubleValue(3.0),
                MakeDoubleAccessor(&ConditionalHandoverAlgorithm::m_hysteresisDb),
                MakeDoubleChecker<uint8_t>(0.0, 15.0))
            .AddAttribute("TimeToTrigger",
                          "Time during which neighbour cell's RSRP must continuously higher than "
                          "serving cell's RSRP in order to prepare CHO",
                          TimeValue(MilliSeconds(256)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_timeToTrigger),
                          MakeTimeChecker())
            .AddAttribute("MeasurementReportDelay",
                          "UE measurement report transport delay before serving gNB receives it.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_measurementReportDelay),
                          MakeTimeChecker())
            .AddAttribute("ChoDecisionDelay",
                          "Serving gNB CHO candidate screening and decision processing delay.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_choDecisionDelay),
                          MakeTimeChecker())
            .AddAttribute("HoRequestPropagationDelay",
                          "One-way propagation delay for HO Preparation and ACK messages.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_hoRequestPropagationDelay),
                          MakeTimeChecker())
            .AddAttribute("HoPreparationDelay",
                          "Delay for target-side admission control and preparation ACK.",
                          TimeValue(MilliSeconds(8)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_hoPreparationDelay),
                          MakeTimeChecker())
            .AddAttribute("HoPreparationPerTargetOffset",
                          "Additional admission-control delay per candidate target index.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_hoPreparationPerTargetOffset),
                          MakeTimeChecker())
            .AddAttribute("ChoCommandDeliveryDelay",
                          "Delay from source sending CHO command to UE receiving it.",
                          TimeValue(MilliSeconds(2)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_choCommandDeliveryDelay),
                          MakeTimeChecker())
            .AddAttribute("RandomAccessDuration",
                          "Duration of random access before TriggerHandover.",
                          TimeValue(MilliSeconds(6)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_randomAccessDuration),
                          MakeTimeChecker())
            .AddAttribute("RandomAccessStepDelay",
                          "Per-step delay in modeled random access timeline.",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_randomAccessStepDelay),
                          MakeTimeChecker())
            .AddAttribute("PathSwitchDelay",
                          "Modeled delay to complete path switch after handover completion.",
                          TimeValue(MilliSeconds(2)),
                          MakeTimeAccessor(&ConditionalHandoverAlgorithm::m_pathSwitchDelay),
                          MakeTimeChecker())
            .AddTraceSource(
                "ChoEvent",
                "Detailed CHO event trace: timestampMs, rnti, eventName, targetCellId, "
                "servingRsrp, targetRsrp, candidateCount",
                MakeTraceSourceAccessor(&ConditionalHandoverAlgorithm::m_choEventTrace),
                "ns3::TracedCallback::Uint64Uint16StringUint16Uint8Uint8Uint16")
            .AddTraceSource("ChoDetailedEvent",
                            "Step-by-step CHO details: timestampMs, rnti, stepName, detail",
                            MakeTraceSourceAccessor(&ConditionalHandoverAlgorithm::m_choDetailedTrace),
                            "ns3::TracedCallback::Uint64Uint16StringString");
    return tid;
}

void
ConditionalHandoverAlgorithm::SetLteHandoverManagementSapUser(LteHandoverManagementSapUser* s)
{
    NS_LOG_FUNCTION(this << s);
    m_handoverManagementSapUser = s;
}

LteHandoverManagementSapProvider*
ConditionalHandoverAlgorithm::GetLteHandoverManagementSapProvider()
{
    NS_LOG_FUNCTION(this);
    return m_handoverManagementSapProvider;
}

void
ConditionalHandoverAlgorithm::DoInitialize()
{
    NS_LOG_FUNCTION(this);

    uint8_t hysteresisIeValue = EutranMeasurementMapping::ActualHysteresis2IeValue(m_hysteresisDb);

    LteRrcSap::ReportConfigEutra reportConfig;
    reportConfig.eventId = LteRrcSap::ReportConfigEutra::EVENT_A3;
    reportConfig.a3Offset = 0;
    reportConfig.hysteresis = hysteresisIeValue;
    reportConfig.timeToTrigger = m_timeToTrigger.GetMilliSeconds();
    reportConfig.reportOnLeave = false;
    reportConfig.triggerQuantity = LteRrcSap::ReportConfigEutra::RSRP;
    reportConfig.reportInterval = LteRrcSap::ReportConfigEutra::MS1024;
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
            << " tttMs=" << m_timeToTrigger.GetMilliSeconds()
            << " measReportDelayMs=" << m_measurementReportDelay.GetMilliSeconds()
            << " decisionDelayMs=" << m_choDecisionDelay.GetMilliSeconds()
            << " hoReqPropagationDelayMs=" << m_hoRequestPropagationDelay.GetMilliSeconds()
            << " hoPreparationDelayMs=" << m_hoPreparationDelay.GetMilliSeconds()
            << " hoPreparationPerTargetOffsetMs="
            << m_hoPreparationPerTargetOffset.GetMilliSeconds()
            << " choCommandDeliveryDelayMs=" << m_choCommandDeliveryDelay.GetMilliSeconds()
            << " randomAccessDurationMs=" << m_randomAccessDuration.GetMilliSeconds()
            << " randomAccessStepDelayMs=" << m_randomAccessStepDelay.GetMilliSeconds()
            << " pathSwitchDelayMs=" << m_pathSwitchDelay.GetMilliSeconds();
        EmitChoDetailedTrace(0, "Step0_ChoConfiguration", oss.str());
    }

    LteHandoverAlgorithm::DoInitialize();
}

void
ConditionalHandoverAlgorithm::DoDispose()
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
}

void
ConditionalHandoverAlgorithm::DoReportUeMeas(uint16_t rnti, LteRrcSap::MeasResults measResults)
{
    NS_LOG_FUNCTION(this << rnti << (uint16_t)measResults.measId);

    if (std::find(begin(m_measIds), end(m_measIds), measResults.measId) == std::end(m_measIds))
    {
        return;
    }

    if (!measResults.haveMeasResultNeighCells || measResults.measResultListEutra.empty())
    {
        return;
    }

    const Time mrRecvTime = Simulator::Now();
    const Time mrSentTime = mrRecvTime - m_measurementReportDelay;
    const uint8_t servingRsrp = measResults.measResultPCell.rsrpResult;

    auto pendingIt = m_pendingCho.find(rnti);

    // CHO execution monitoring: once CHO command is delivered, skip Step1 MR flow
    // and only evaluate execution condition with incoming measurements.
    if (pendingIt != m_pendingCho.end() && pendingIt->second.m_choCommandDelivered)
    {
        // Update candidate RSRP values with new measurements
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

        // Update serving RSRP
        pendingIt->second.m_servingRsrp = servingRsrp;

        // Check execution condition: best target RSRP > serving RSRP + hysteresis
        const uint16_t bestTargetCellId = ComputeBestTargetCellId(pendingIt->second.m_candidates);
        const uint8_t bestTargetRsrp = ComputeBestTargetRsrp(pendingIt->second.m_candidates);

        {
            std::ostringstream oss;
            oss << "actor=UE action=PROCESS stage=ExecutionMonitoring"
                << " bestTargetCellId=" << bestTargetCellId
                << " bestTargetRsrp=" << static_cast<uint16_t>(bestTargetRsrp)
                << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
                << " hysteresisDb=" << m_hysteresisDb
                << " condition=(bestTarget > serving+hyst)="
                << (bestTargetRsrp > servingRsrp + m_hysteresisDb ? "true" : "false");
            EmitChoDetailedTrace(rnti, "Step3_ExecutionMonitoring", oss.str());
        }

        if (bestTargetRsrp > servingRsrp + m_hysteresisDb)
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
        EmitChoDetailedTraceAt(
            mrSentTime.GetMilliSeconds(), rnti, "Step1_MR_TxByUe", txOss.str());

        std::ostringstream rxOss;
        rxOss << "actor=Source-gNB action=RX message=MeasurementReport from=UE"
              << " measId=" << static_cast<uint16_t>(measResults.measId)
              << " servingRsrp=" << static_cast<uint16_t>(servingRsrp)
              << " rawNeighborResultCount=" << measResults.measResultListEutra.size();
        EmitChoDetailedTraceAt(
            mrRecvTime.GetMilliSeconds(), rnti, "Step1_MR_RxBySource", rxOss.str());

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

        if (it->rsrpResult > measResults.measResultPCell.rsrpResult + m_hysteresisDb)
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
                << " condition=(target > serving+hyst) true";
            EmitChoDetailedTrace(rnti, "Step1_CandidateEvaluation", oss.str());
        }
        else
        {
            std::ostringstream oss;
            oss << "Candidate rejected: cellId=" << it->physCellId
                << " targetRsrp=" << static_cast<uint16_t>(it->rsrpResult)
                << " condition=(target > serving+hyst) false";
            EmitChoDetailedTrace(rnti, "Step1_CandidateEvaluation", oss.str());
        }
    }

    // CHO preparation: process measurement report for CHO candidate selection
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

    EmitChoEventTrace(rnti,
                      "ChoPrepareStart",
                      state.m_selectedTargetCellId,
                      servingRsrp,
                      state.m_selectedTargetRsrp,
                      state.m_selectedCandidateCount);

    state.m_decisionEvent =
        Simulator::Schedule(m_choDecisionDelay, &ConditionalHandoverAlgorithm::OnChoDecisionCompleted, this, rnti);
    m_pendingCho.emplace(rnti, std::move(state));
}

void
ConditionalHandoverAlgorithm::OnChoDecisionCompleted(uint16_t rnti)
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

    it->second.m_hoRequestSentTime = now;
    EmitChoEventTrace(rnti,
                      "HoRequestSentToCandidates",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

    {
        std::ostringstream txOss;
        txOss << "actor=Source-gNB action=TX message=HoPreparationRequest"
              << " toCandidates=" << BuildCandidateListString(it->second.m_candidates)
              << " targetReceiveTimes=" << BuildTargetReceiveTimesString(it->second);
        EmitChoDetailedTraceAt(
            now.GetMilliSeconds(), rnti, "Step2_HoReq_TxBySource", txOss.str());

        for (size_t i = 0; i < it->second.m_candidates.size(); ++i)
        {
            const Time hoReqRx = now + m_hoRequestPropagationDelay;
            const Time offset =
                NanoSeconds(m_hoPreparationPerTargetOffset.GetNanoSeconds() * static_cast<int64_t>(i));
            const Time prepDone = hoReqRx + m_hoPreparationDelay + offset;
            const Time ackTx = prepDone;

            std::ostringstream rxOss;
            rxOss << "actor=Target-gNB(cellId=" << it->second.m_candidates[i].m_cellId
                  << ") action=RX message=HoPreparationRequest from=Source-gNB";
            EmitChoDetailedTraceAt(
                hoReqRx.GetMilliSeconds(), rnti, "Step2_HoReq_RxByTarget", rxOss.str());

            std::ostringstream procOss;
            procOss << "actor=Target-gNB(cellId=" << it->second.m_candidates[i].m_cellId
                    << ") action=PROCESS stage=AdmissionControl"
                    << " startAtMs=" << hoReqRx.GetMilliSeconds()
                    << " doneAtMs=" << prepDone.GetMilliSeconds();
            EmitChoDetailedTraceAt(
                prepDone.GetMilliSeconds(), rnti, "Step2_AdmissionProcessedByTarget", procOss.str());

            std::ostringstream ackTxOss;
            ackTxOss << "actor=Target-gNB(cellId=" << it->second.m_candidates[i].m_cellId
                     << ") action=TX message=HoPreparationAck to=Source-gNB";
            EmitChoDetailedTraceAt(
                ackTx.GetMilliSeconds(), rnti, "Step2_HoAck_TxByTarget", ackTxOss.str());
        }
    }

    it->second.m_allAckReceivedTime = ComputeAllAckReceivedTime(it->second);
    const Time wait = it->second.m_allAckReceivedTime - now;
    it->second.m_ackEvent =
        Simulator::Schedule(wait, &ConditionalHandoverAlgorithm::OnHandoverPreparationCompleted, this, rnti);
}

void
ConditionalHandoverAlgorithm::OnHandoverPreparationCompleted(uint16_t rnti)
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
        std::ostringstream oss;
        oss << "actor=Source-gNB action=RX message=HoPreparationAck(allCandidates)"
            << " allAckReceivedAtMs=" << Simulator::Now().GetMilliSeconds()
            << "ms perTargetAckTimeline=" << BuildAdmissionAckTimesString(it->second);
        EmitChoDetailedTrace(rnti, "Step2_AdmissionControlAndAck", oss.str());
    }

    EmitChoEventTrace(rnti,
                      "RrcReconfigurationWithChoSent",
                      it->second.m_selectedTargetCellId,
                      it->second.m_servingRsrp,
                      it->second.m_selectedTargetRsrp,
                      it->second.m_selectedCandidateCount);

    {
        const Time tx = Simulator::Now();
        const Time rx = tx + m_choCommandDeliveryDelay;
        std::ostringstream txOss;
        txOss << "actor=Source-gNB action=TX message=RrcReconfiguration(CHO) to=UE"
              << " payloadCandidates=" << BuildCandidateListString(it->second.m_candidates)
              << " selectedTargetCellId=" << it->second.m_selectedTargetCellId;
        EmitChoDetailedTraceAt(
            tx.GetMilliSeconds(), rnti, "Step3_RrcReconfigCho_TxBySource", txOss.str());

        std::ostringstream rxOss;
        rxOss << "actor=UE action=RX message=RrcReconfiguration(CHO) from=Source-gNB"
              << " selectedTargetCellId=" << it->second.m_selectedTargetCellId;
        EmitChoDetailedTraceAt(rx.GetMilliSeconds(), rnti, "Step3_RrcReconfigCho_RxByUe", rxOss.str());

        std::ostringstream procOss;
        procOss << "actor=UE action=PROCESS message=RrcReconfiguration(CHO)";
        EmitChoDetailedTraceAt(
            rx.GetMilliSeconds(), rnti, "Step3_RrcReconfigCho_ProcessedByUe", procOss.str());
    }

    it->second.m_choCommandDeliveryEvent = Simulator::Schedule(m_choCommandDeliveryDelay,
                                                               &ConditionalHandoverAlgorithm::OnChoCommandDeliveredToUe,
                                                               this,
                                                               rnti);
}

void
ConditionalHandoverAlgorithm::OnChoCommandDeliveredToUe(uint16_t rnti)
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

    // Mark that CHO command has been delivered, execution monitoring starts now
    it->second.m_choCommandDelivered = true;
}

void
ConditionalHandoverAlgorithm::ExecuteCho(uint16_t rnti)
{
    auto it = m_pendingCho.find(rnti);
    if (it == m_pendingCho.end() || it->second.m_candidates.empty())
    {
        return;
    }

    const uint16_t targetCellId = ComputeBestTargetCellId(it->second.m_candidates);
    const uint8_t targetRsrp = ComputeBestTargetRsrp(it->second.m_candidates);

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
        "Step5_DetachServingCell",
        "actor=UE action=DETACH from=ServingCell atMs=" +
            std::to_string(Simulator::Now().GetMilliSeconds()));

    const Time raStart = Simulator::Now();
    const Time preambleTx = raStart;
    const Time preambleRx = preambleTx + m_randomAccessStepDelay;
    const Time rarTx = preambleRx + m_randomAccessStepDelay;
    const Time rarRx = rarTx + m_randomAccessStepDelay;
    const Time completeTx = rarRx + m_randomAccessStepDelay;
    const Time completeRx = completeTx + m_randomAccessStepDelay;
    const Time raCompleteTime = std::max(raStart + m_randomAccessDuration, completeRx);

    {
        EmitChoDetailedTraceAt(
            preambleTx.GetMilliSeconds(),
            rnti,
            "Step5_RA_Msg1Preamble_TxByUe",
            "actor=UE action=TX message=CFRA-Preamble(dedicated) to=Target-gNB(cellId=" +
                std::to_string(targetCellId) + ")");
        EmitChoDetailedTraceAt(
            preambleRx.GetMilliSeconds(),
            rnti,
            "Step5_RA_Msg1Preamble_RxByTarget",
            "actor=Target-gNB(cellId=" + std::to_string(targetCellId) +
                ") action=RX message=CFRA-Preamble(dedicated)");

        EmitChoDetailedTraceAt(
            rarTx.GetMilliSeconds(),
            rnti,
            "Step5_RA_Msg2RAR_TxByTarget",
            "actor=Target-gNB(cellId=" + std::to_string(targetCellId) +
                ") action=TX message=RA-Msg2(RAR) to=UE");
        EmitChoDetailedTraceAt(
            rarRx.GetMilliSeconds(),
            rnti,
            "Step5_RA_Msg2RAR_RxByUe",
            "actor=UE action=RX message=RA-Msg2(RAR) from=Target-gNB(cellId=" +
                std::to_string(targetCellId) + ")");

        EmitChoDetailedTraceAt(
            completeTx.GetMilliSeconds(),
            rnti,
            "Step5_RA_RrcReconfigComplete_TxByUe",
            "actor=UE action=TX message=RrcReconfigurationComplete to=Target-gNB(cellId=" +
                std::to_string(targetCellId) + ")");
        EmitChoDetailedTraceAt(
            completeRx.GetMilliSeconds(),
            rnti,
            "Step5_RA_RrcReconfigComplete_RxByTarget",
            "actor=Target-gNB(cellId=" + std::to_string(targetCellId) +
                ") action=RX message=RrcReconfigurationComplete");

        std::ostringstream summaryOss;
        summaryOss << "actor=UE action=PROCESS stage=RandomAccessTimeline"
                   << " startMs=" << raStart.GetMilliSeconds()
                   << " completeMs=" << raCompleteTime.GetMilliSeconds();
        EmitChoDetailedTraceAt(
            raStart.GetMilliSeconds(), rnti, "Step5_RandomAccessStart", summaryOss.str());
    }

    it->second.m_randomAccessCompleteEvent = Simulator::Schedule(
        raCompleteTime - Simulator::Now(),
        &ConditionalHandoverAlgorithm::CompleteChoAfterRandomAccess,
        this,
        rnti);
}

void
ConditionalHandoverAlgorithm::CompleteChoAfterRandomAccess(uint16_t rnti)
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

    m_handoverManagementSapUser->TriggerHandover(rnti, it->second.m_selectedTargetCellId);
    m_pendingCho.erase(it);
}

void
ConditionalHandoverAlgorithm::CancelCho(uint16_t rnti)
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
ConditionalHandoverAlgorithm::EmitChoEventTrace(uint16_t rnti,
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
ConditionalHandoverAlgorithm::EmitChoDetailedTrace(uint16_t rnti,
                                                   const std::string& stepName,
                                                   const std::string& detail)
{
    EmitChoDetailedTraceAt(Simulator::Now().GetMilliSeconds(), rnti, stepName, detail);
}

void
ConditionalHandoverAlgorithm::EmitChoDetailedTraceAt(uint64_t timestampMs,
                                                     uint16_t rnti,
                                                     const std::string& stepName,
                                                     const std::string& detail)
{
    m_choDetailedTrace(timestampMs, rnti, stepName, detail);
}

std::string
ConditionalHandoverAlgorithm::BuildCandidateListString(const std::vector<CandidateInfo>& candidates) const
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
ConditionalHandoverAlgorithm::BuildTargetReceiveTimesString(const PendingChoState& state) const
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < state.m_candidates.size(); ++i)
    {
        const Time rx = state.m_hoRequestSentTime + m_hoRequestPropagationDelay;
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
ConditionalHandoverAlgorithm::BuildAdmissionAckTimesString(const PendingChoState& state) const
{
    std::ostringstream oss;
    oss << "[";
    for (size_t i = 0; i < state.m_candidates.size(); ++i)
    {
        const Time hoReqRx = state.m_hoRequestSentTime + m_hoRequestPropagationDelay;
        const Time offset =
            NanoSeconds(m_hoPreparationPerTargetOffset.GetNanoSeconds() * static_cast<int64_t>(i));
        const Time admissionDone = hoReqRx + m_hoPreparationDelay + offset;
        const Time ackTx = admissionDone;
        const Time ackRx = ackTx + m_hoRequestPropagationDelay;

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
ConditionalHandoverAlgorithm::ComputeAllAckReceivedTime(const PendingChoState& state) const
{
    if (state.m_candidates.empty())
    {
        return Simulator::Now();
    }

    Time maxAckRx = state.m_hoRequestSentTime;
    for (size_t i = 0; i < state.m_candidates.size(); ++i)
    {
        const Time hoReqRx = state.m_hoRequestSentTime + m_hoRequestPropagationDelay;
        const Time offset =
            NanoSeconds(m_hoPreparationPerTargetOffset.GetNanoSeconds() * static_cast<int64_t>(i));
        const Time admissionDone = hoReqRx + m_hoPreparationDelay + offset;
        const Time ackRx = admissionDone + m_hoRequestPropagationDelay;
        if (ackRx > maxAckRx)
        {
            maxAckRx = ackRx;
        }
    }
    return maxAckRx;
}

uint16_t
ConditionalHandoverAlgorithm::ComputeBestTargetCellId(const std::vector<CandidateInfo>& candidates) const
{
    const auto best = std::max_element(
        candidates.begin(),
        candidates.end(),
        [](const CandidateInfo& lhs, const CandidateInfo& rhs) { return lhs.m_rsrp < rhs.m_rsrp; });
    return (best == candidates.end()) ? 0 : best->m_cellId;
}

uint8_t
ConditionalHandoverAlgorithm::ComputeBestTargetRsrp(const std::vector<CandidateInfo>& candidates) const
{
    const auto best = std::max_element(
        candidates.begin(),
        candidates.end(),
        [](const CandidateInfo& lhs, const CandidateInfo& rhs) { return lhs.m_rsrp < rhs.m_rsrp; });
    return (best == candidates.end()) ? 0 : best->m_rsrp;
}

bool
ConditionalHandoverAlgorithm::IsValidNeighbour(uint16_t cellId)
{
    NS_LOG_FUNCTION(this << cellId);
    return true;
}

} // namespace ns3
