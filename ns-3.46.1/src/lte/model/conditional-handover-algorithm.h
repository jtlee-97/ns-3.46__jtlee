/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef CONDITIONAL_HANDOVER_ALGORITHM_H
#define CONDITIONAL_HANDOVER_ALGORITHM_H

#include "lte-handover-algorithm.h"
#include "lte-handover-management-sap.h"
#include "lte-rrc-sap.h"

#include "ns3/nstime.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>

namespace ns3
{

/**
 * @brief Conditional handover algorithm based on Event A3 reports.
 *
 * The algorithm prepares CHO candidate cells on A3 measurement reports and
 * executes handover after a configurable delay. If no candidate remains valid,
 * pending CHO execution is canceled.
 */
class ConditionalHandoverAlgorithm : public LteHandoverAlgorithm
{
  public:
    ConditionalHandoverAlgorithm();

    ~ConditionalHandoverAlgorithm() override;

    static TypeId GetTypeId();

    void SetLteHandoverManagementSapUser(LteHandoverManagementSapUser* s) override;
    LteHandoverManagementSapProvider* GetLteHandoverManagementSapProvider() override;

    friend class MemberLteHandoverManagementSapProvider<ConditionalHandoverAlgorithm>;

  protected:
    void DoInitialize() override;
    void DoDispose() override;
    void DoReportUeMeas(uint16_t rnti, LteRrcSap::MeasResults measResults) override;

  private:
    struct CandidateInfo
    {
        uint16_t m_cellId;
        uint8_t m_rsrp;
    };

    struct PendingChoState
    {
        std::vector<CandidateInfo> m_candidates;
      uint8_t m_servingRsrp;
      Time m_measReportSentTime;
      Time m_measReportRecvTime;
      Time m_decisionStartTime;
      Time m_hoRequestSentTime;
      Time m_allAckReceivedTime;
      uint16_t m_selectedTargetCellId;
      uint8_t m_selectedTargetRsrp;
      uint16_t m_selectedCandidateCount;
      bool m_choCommandDelivered;
      EventId m_decisionEvent;
      EventId m_ackEvent;
      EventId m_choCommandDeliveryEvent;
      EventId m_randomAccessCompleteEvent;
    };

    bool IsValidNeighbour(uint16_t cellId);
    std::string BuildCandidateListString(const std::vector<CandidateInfo>& candidates) const;
    std::string BuildTargetReceiveTimesString(const PendingChoState& state) const;
    std::string BuildAdmissionAckTimesString(const PendingChoState& state) const;
    Time ComputeAllAckReceivedTime(const PendingChoState& state) const;
    uint16_t ComputeBestTargetCellId(const std::vector<CandidateInfo>& candidates) const;
    uint8_t ComputeBestTargetRsrp(const std::vector<CandidateInfo>& candidates) const;

    void OnChoDecisionCompleted(uint16_t rnti);
    void OnHandoverPreparationCompleted(uint16_t rnti);
    void OnChoCommandDeliveredToUe(uint16_t rnti);
    void ExecuteCho(uint16_t rnti);
    void CompleteChoAfterRandomAccess(uint16_t rnti);
    void CancelCho(uint16_t rnti);
    void EmitChoEventTrace(uint16_t rnti,
                 const std::string& eventName,
                 uint16_t targetCellId,
                 uint8_t servingRsrp,
                 uint8_t targetRsrp,
                 uint16_t candidateCount);
    void EmitChoDetailedTrace(uint16_t rnti, const std::string& stepName, const std::string& detail);
    void EmitChoDetailedTraceAt(uint64_t timestampMs,
                  uint16_t rnti,
                  const std::string& stepName,
                  const std::string& detail);

    LteHandoverManagementSapUser* m_handoverManagementSapUser;
    LteHandoverManagementSapProvider* m_handoverManagementSapProvider;
    std::vector<uint8_t> m_measIds;
    std::map<uint16_t, PendingChoState> m_pendingCho;
    double m_hysteresisDb;
    Time m_timeToTrigger;
    Time m_measurementReportDelay;
    Time m_choDecisionDelay;
    Time m_hoRequestPropagationDelay;
    Time m_hoPreparationDelay;
    Time m_hoPreparationPerTargetOffset;
    Time m_choCommandDeliveryDelay;
    Time m_randomAccessDuration;
    Time m_randomAccessStepDelay;
    Time m_pathSwitchDelay;

    /**
     * CHO detailed event trace source.
     *
     * Callback signature:
     *   (timestampMs, rnti, eventName, targetCellId, servingRsrp, targetRsrp, candidateCount)
     */
    TracedCallback<uint64_t, uint16_t, std::string, uint16_t, uint8_t, uint8_t, uint16_t>
      m_choEventTrace;

    /**
     * CHO step-by-step detailed trace source.
     *
     * Callback signature:
     *   (timestampMs, rnti, stepName, detail)
     */
    TracedCallback<uint64_t, uint16_t, std::string, std::string> m_choDetailedTrace;
};

} // namespace ns3

#endif /* CONDITIONAL_HANDOVER_ALGORITHM_H */
