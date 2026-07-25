#pragma once

#include <string>
#include <vector>

#include "hacdcpf/resilience/certified_restoration.hpp"
#include "hacdcpf/resilience/resilience_assessment.hpp"

namespace hacdcpf::analysis {

struct DistributionResilienceDAECertificationOptions {
  double switching_time_s{0.1};
  int fidelity_level{3};
  bool certify_initial_transition{true};
  bool certify_final_state{true};
  bool require_supported_transitions{true};
  /// Replay aggregate MIP served/demand ratios as domain-qualified bus load
  /// scaling in the DAE initial state and transition event.
  bool replay_mip_load_service{true};
  int max_transitions{64};

  bool command_path_available{true};
  bool authorization_valid{true};
  bool acknowledgement_available{true};
};

struct DistributionResilienceDAETransition {
  int from_step{-1};
  int to_step{0};
  HybridPowerSystem initial_system;
  CertifiedRestorationAction action;
  std::vector<std::string> unsupported_transitions;
};

struct DistributionResilienceDAETransitionResult {
  int from_step{-1};
  int to_step{0};
  CertifiedRestorationAction action;
  ExecutabilityReport executability;
  MultiFidelityCertificate certificate;
  std::vector<std::string> unsupported_transitions;
};

/// Structured return from the DAE oracle to a restoration master. These are
/// candidate feasibility/model-resolution cuts; this bridge does not claim a
/// cut was enforced unless applied_to_restoration_mip is true.
struct DistributionResilienceDAEFeedback {
  int to_step{0};
  std::string action_id;
  std::string feedback_type;
  std::string reason;
  std::vector<std::string> event_labels;
  std::string recommended_master_action;
  bool applied_to_restoration_mip{false};
};

struct DistributionResilienceDAECertificateResult {
  bool attempted{false};
  bool all_transitions_safe{false};
  bool proof_valid{false};
  std::string status;
  std::string model_scope{
      "strict restoration MIP topology and aggregate bus-service replay + scenario-specific full-DAE threshold certificates"};
  std::vector<std::string> limitations;
  std::vector<DistributionResilienceDAETransitionResult> transitions;
  std::vector<DistributionResilienceDAEFeedback> feedback;
};

struct CertifiedDistributionResilienceResult {
  DistributionResilienceResult restoration;
  DistributionResilienceDAECertificateResult dynamic_certification;
};

/// Convert a strict-MIP time sequence into DAE transition actions. The returned
/// initial_system for each action is the canonical topology at from_step.
std::vector<DistributionResilienceDAETransition>
build_distribution_resilience_dae_transitions(
    const HybridPowerSystem& system,
    const DistributionResilienceResult& restoration,
    const DistributionResilienceDAECertificationOptions& options = {});

/// Run scenario-specific L3 (or caller-selected) DAE threshold certificates on
/// every material topology/MESS transition represented by the restoration plan.
DistributionResilienceDAECertificateResult
certify_distribution_resilience_dynamics(
    const HybridPowerSystem& system,
    const DistributionResilienceResult& restoration,
    const dynamics::DynamicSolverOptions& dynamic_options,
    const MultiFidelityCertificateOptions& certificate_options = {},
    const DistributionResilienceDAECertificationOptions& options = {});

/// End-to-end strict restoration MIP followed by DAE secondary certification.
CertifiedDistributionResilienceResult
run_certified_distribution_resilience_mip(
    const HybridPowerSystem& system,
    const DistributionResilienceOptions& restoration_options,
    const dynamics::DynamicSolverOptions& dynamic_options,
    const MultiFidelityCertificateOptions& certificate_options = {},
    const DistributionResilienceDAECertificationOptions& options = {});

}  // namespace hacdcpf::analysis
