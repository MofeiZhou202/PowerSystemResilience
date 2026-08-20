#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "hacdcpf/reliability/protection_frt.hpp"

namespace hacdcpf::analysis {

struct DetectionLatencyClass {
  double probability{0.0};
  double latency_s{0.0};
};

/// One row of P(reported class | true class). Reported class 0 is reserved for
/// "no alarm"; positive columns map to fault_class_ids[column - 1].
struct IntelligentDetectionModel {
  std::vector<std::string> fault_class_ids;
  std::vector<std::vector<double>> confusion_matrix;
  std::vector<std::vector<DetectionLatencyClass>> latency_by_true_class;
  double timely_detection_limit_s{0.0};
  double no_fault_decision_windows_per_year{0.0};
};

struct IntelligentDetectionClass {
  std::string true_fault_class;
  std::string reported_fault_class;
  double probability{0.0};
  double latency_s{0.0};
  bool missed{false};
  bool correct{false};
  bool timely{false};
};

struct IntelligentDetectionResult {
  std::vector<IntelligentDetectionClass> classes;
  std::vector<double> detection_probability;
  std::vector<double> correct_classification_probability;
  std::vector<double> expected_detection_latency_s;
  std::vector<double> timely_detection_probability;
  double false_alarm_frequency_per_year{0.0};
  bool probabilities_normalized{false};
};

IntelligentDetectionResult evaluate_intelligent_detection(
    const IntelligentDetectionModel& model);

struct IsolationCandidate {
  std::string id;
  std::vector<size_t> covered_fault_classes;
  double isolated_load_mw{0.0};
  double operation_time_s{0.0};
  double action_success_probability{1.0};
  std::vector<size_t> required_information_functions;
};

struct IntelligentIsolationInput {
  std::vector<double> posterior_fault_probability;
  std::vector<IsolationCandidate> candidates;
  double missed_isolation_cost{1.0};
  double isolated_load_cost{1.0};
  double operation_time_cost{0.0};
  std::vector<bool> information_function_available;
};

struct IsolationCandidateEvaluation {
  std::string id;
  bool executable{false};
  double covered_probability{0.0};
  double expected_cost{0.0};
};

struct IntelligentIsolationResult {
  int selected_candidate{-1};
  std::string selected_candidate_id;
  double selected_expected_cost{0.0};
  std::vector<IsolationCandidateEvaluation> candidates;
};

IntelligentIsolationResult select_minimum_risk_isolation(
    const IntelligentIsolationInput& input);

struct RiskConstrainedRestorationAction {
  std::string id;
  double restored_load_mw{0.0};
  double switching_cost{0.0};
  double unsafe_probability{0.0};
  double execution_success_probability{1.0};
  double remote_time_s{0.0};
  double manual_time_s{0.0};
  std::vector<size_t> required_information_functions;
};

struct RiskConstrainedRestorationInput {
  std::vector<RiskConstrainedRestorationAction> actions;
  std::vector<bool> information_function_available;
  double maximum_unsafe_probability{0.0};
  double unserved_energy_cost_per_mwh{1.0};
  double decision_horizon_hr{1.0};
};

struct RiskConstrainedRestorationResult {
  int selected_action{-1};
  std::string selected_action_id;
  bool used_manual_fallback{false};
  double selected_duration_s{0.0};
  double expected_restored_load_mw{0.0};
  double objective{0.0};
  std::vector<std::string> rejected_actions;
};

RiskConstrainedRestorationResult select_risk_constrained_restoration(
    const RiskConstrainedRestorationInput& input);

/// Finite-state, finite-action, finite-observation POMDP solved by an exact
/// belief-tree recursion for the supplied initial belief. The exponential
/// algorithm intentionally rejects models beyond the documented bounds.
struct FinitePOMDPModel {
  std::vector<std::string> state_ids;
  std::vector<std::string> action_ids;
  std::vector<std::string> observation_ids;
  std::vector<double> initial_belief;
  /// [action][state][next_state]
  std::vector<std::vector<std::vector<double>>> transition_probability;
  /// [action][next_state][observation]
  std::vector<std::vector<std::vector<double>>> observation_probability;
  /// [state][action]
  std::vector<std::vector<double>> stage_cost;
  std::vector<double> terminal_cost;
  int horizon_steps{1};
};

struct FinitePOMDPResult {
  int first_action{-1};
  std::string first_action_id;
  double expected_total_cost{0.0};
  int belief_nodes_evaluated{0};
  bool exact_for_initial_belief{false};
};

FinitePOMDPResult solve_finite_pomdp(const FinitePOMDPModel& model);

struct ChronologicalCyberComponent {
  ProtectionCyberComponent service;
  double failure_rate_per_year{0.0};
  double mean_repair_time_hr{0.0};
  double backup_recharge_power_w{0.0};
  bool initially_available{true};
};

struct ChronologicalCommonCause {
  std::string group;
  double occurrence_rate_per_year{0.0};
  double mean_duration_hr{0.0};
};

struct ChronologicalPhysicalFault {
  ProtectionCyberReliabilityScenario scenario;
  std::vector<std::string> deenergized_bus_ids;
  double information_power_outage_duration_hr{0.0};
  /// Optional deterministic event times for analytic/regression cases. When
  /// non-empty, the Poisson occurrence process is disabled for this fault.
  std::vector<double> scheduled_occurrence_hours;
};

struct ChronologicalCyberPhysicalOptions {
  int simulated_years{1};
  uint64_t random_seed{1};
  double hours_per_year{8760.0};
  int maximum_events{1000000};
  double curtailment_threshold_mw{0.01};
};

struct ChronologicalCyberPhysicalResult {
  double eens_mwh_yr{0.0};
  double lole_hr_yr{0.0};
  double lolf_occ_yr{0.0};
  int physical_faults_sampled{0};
  int cyber_transitions_sampled{0};
  int common_cause_events_sampled{0};
  bool subhour_event_timing{false};
  bool cyber_power_coupling_modelled{false};
  bool common_cause_modelled{false};
  bool probability_weighted_protection_modelled{false};
  bool packet_delivery_sampled{false};
  bool battery_energy_trajectory_modelled{false};
  bool outage_interval_union_modelled{false};
  double minimum_battery_energy_wh{0.0};
  int merged_interruption_intervals{0};
  std::string model_scope{
      "chronological-cyber-physical-event-queue+protection-event-tree"};
};

ChronologicalCyberPhysicalResult run_chronological_cyber_physical_reliability(
    const std::vector<ChronologicalCyberComponent>& cyber_components,
    const std::vector<ProtectionCyberFunction>& functions,
    const std::vector<ChronologicalCommonCause>& common_causes,
    const std::vector<ChronologicalPhysicalFault>& physical_faults,
    const ChronologicalCyberPhysicalOptions& options = {});

}  // namespace hacdcpf::analysis
