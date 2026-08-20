#pragma once

#include <string>
#include <vector>

#include "hacdcpf/dynamics/DynamicResults.hpp"
#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/reliability/protection_frt.hpp"

namespace hacdcpf::analysis {

struct OnlineDERFRTMonitor {
  std::string stable_id;
  int ac_bus_id{0};
  dynamics::IEEE1547Settings settings;
  DERMomentaryCessationSettings momentary_cessation;
  double loss_of_generation_shed_mw{0.0};
};

struct OnlineProtectionDAEInput {
  int fault_ac_bus_id{0};
  int fault_phase{0};
  double fault_time_s{0.0};
  double fault_r_pu{0.0};
  double fault_x_pu{0.0};
  int protected_ac_branch_index{0};
  ProtectionRelayModel relay;
  BreakerClearingChain breaker;
  InstrumentTransformerSettings instrument_transformers;
  std::vector<OnlineDERFRTMonitor> der_monitors;
  dynamics::DynamicSolverOptions dynamic_options;
};

struct OnlineProtectionDAEResult {
  bool success{false};
  std::string message;
  ProtectionRelayEvaluation relay;
  double protection_clear_time_s{0.0};
  dynamics::DynamicResults discovery_run;
  dynamics::DynamicResults closed_loop_run;
  std::vector<ProtectionMeasurementPoint> measured_relay_trajectory;
  std::vector<std::string> der_stable_ids;
  std::vector<DERFRTTrajectoryResult> der_results;
  std::vector<std::vector<DERTrajectoryPoint>> discovery_der_trajectories;
  std::vector<std::vector<DERTrajectoryPoint>> closed_loop_der_trajectories;
  bool online_network_dae_coupled{false};
  bool relay_trajectory_from_dae{false};
  bool protection_action_applied{false};
  bool dead_island_load_shedding_applied{false};
  std::vector<int> deenergized_ac_bus_ids;
  bool der_frt_state_machine_consumed{false};
  bool converter_dynamic_feedback_consumed{false};
  std::string model_scope{
      "mass-matrix-dae+trajectory-derived-relay+branch-trip+der-frt"};
};

OnlineProtectionDAEResult run_online_protection_dae(
    const HybridPowerSystem& system,
    const OnlineProtectionDAEInput& input);

/// One initiating event whose primary and backup protection trajectories are
/// derived from the network DAE before the protection/cyber event tree is
/// aggregated. Information-state and consequence fields share the same
/// semantics as ProtectionCyberReliabilityScenario.
struct OnlineProtectionCyberReliabilityScenario {
  std::string scenario_id;
  double initiating_frequency_per_year{0.0};
  OnlineProtectionDAEInput primary;
  OnlineProtectionDAEInput backup;
  double primary_relay_success_probability{1.0};
  double primary_breaker_success_probability{1.0};
  double backup_relay_success_probability{1.0};
  double backup_breaker_success_probability{1.0};
  double coordination_margin_s{0.0};
  double uncleared_terminal_time_s{0.0};
  std::vector<ProtectionCyberComponent> information_components;
  std::vector<ProtectionCyberFunction> information_functions;
  std::vector<ProtectionCyberEnvironment> information_environments;
  ProtectionCyberFunctionBindings function_bindings;
  ProtectionCyberConsequenceModel consequence;
};

struct OnlineProtectionCyberScenarioDiagnostic {
  std::string scenario_id;
  OnlineProtectionDAEResult primary;
  OnlineProtectionDAEResult backup;
  double primary_event_tree_clear_time_s{0.0};
  double backup_event_tree_clear_time_s{0.0};
};

struct OnlineProtectionCyberReliabilityResult {
  ProtectionCyberReliabilityComparison comparison;
  std::vector<OnlineProtectionCyberScenarioDiagnostic> diagnostics;
  bool dae_trajectories_consumed_by_event_tree{false};
  std::string model_scope{
      "mass-matrix-dae+protection-cyber-event-tree+three-window-consequence"};
};

OnlineProtectionCyberReliabilityResult
compare_online_protection_cyber_reliability(
    const HybridPowerSystem& system,
    const std::vector<OnlineProtectionCyberReliabilityScenario>& scenarios,
    double curtailment_threshold_mw = 0.01);

}  // namespace hacdcpf::analysis
