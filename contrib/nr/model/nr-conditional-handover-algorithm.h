/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef NR_CONDITIONAL_HANDOVER_ALGORITHM_H
#define NR_CONDITIONAL_HANDOVER_ALGORITHM_H

#include "nr-handover-algorithm.h"
#include "nr-handover-management-sap.h"
#include "nr-rrc-sap.h"

#include "ns3/nstime.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

class Node;
class NrGnbRrc;

class NrConditionalHandoverAlgorithm : public NrHandoverAlgorithm
{
  public:
    NrConditionalHandoverAlgorithm();

    ~NrConditionalHandoverAlgorithm() override;

    static TypeId GetTypeId();

    void SetNrHandoverManagementSapUser(NrHandoverManagementSapUser* s) override;
    NrHandoverManagementSapProvider* GetNrHandoverManagementSapProvider() override;

    friend class MemberNrHandoverManagementSapProvider<NrConditionalHandoverAlgorithm>;

  protected:
    void DoInitialize() override;
    void DoDispose() override;
    void DoReportUeMeas(uint16_t rnti, NrRrcSap::MeasResults measResults) override;

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
        Time m_lastComputedX2Delay;
        Time m_lastComputedServiceDelay;
        std::map<uint16_t, Time> m_x2DelayByTarget;
    };

      static Time ComputeLightSpeedDelay(double distanceMeters);
      Ptr<Node> FindGnbNodeByCellId(uint16_t cellId) const;
      Ptr<Node> FindUeNodeByRnti(uint16_t rnti) const;
      Time ComputeServiceLinkDelay(uint16_t rnti) const;
      Time ComputeX2PropagationDelay(uint16_t rnti, uint16_t targetCellId) const;

    bool IsValidNeighbour(uint16_t cellId);
    std::string BuildCandidateListString(const std::vector<CandidateInfo>& candidates) const;
    std::string BuildTargetReceiveTimesString(const PendingChoState& state) const;
    std::string BuildAdmissionAckTimesString(const PendingChoState& state) const;
    Time ComputeAllAckReceivedTime(const PendingChoState& state) const;
    uint16_t ComputeBestTargetCellId(const std::vector<CandidateInfo>& candidates) const;
    uint8_t ComputeBestTargetRsrp(const std::vector<CandidateInfo>& candidates) const;
    uint16_t ComputeBestPreparedTargetCellId(Ptr<NrGnbRrc> gnbRrc,
                         uint16_t rnti,
                         const std::vector<CandidateInfo>& candidates) const;
    uint8_t ComputeTargetRsrpByCellId(const std::vector<CandidateInfo>& candidates,
                      uint16_t targetCellId) const;

    void OnChoDecisionCompleted(uint16_t rnti);
    void PollConditionalPreparation(uint16_t rnti);
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

    NrHandoverManagementSapUser* m_handoverManagementSapUser;
    NrHandoverManagementSapProvider* m_handoverManagementSapProvider;
    std::vector<uint8_t> m_measIds;
    std::map<uint16_t, PendingChoState> m_pendingCho;
    double m_hysteresisDb;
    double m_a3OffsetDb;
    Time m_timeToTrigger;
    Time m_choDecisionDelay;
    Time m_hoPreparationDelay;
    Time m_hoPreparationPerTargetOffset;
    Time m_randomAccessDuration;
    Time m_randomAccessStepDelay;
    Time m_pathSwitchDelay;
    bool m_enableHandoverExecution;

    TracedCallback<uint64_t, uint16_t, std::string, uint16_t, uint8_t, uint8_t, uint16_t>
        m_choEventTrace;

    TracedCallback<uint64_t, uint16_t, std::string, std::string> m_choDetailedTrace;
};

} // namespace ns3

#endif /* NR_CONDITIONAL_HANDOVER_ALGORITHM_H */
