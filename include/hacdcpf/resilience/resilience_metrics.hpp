#pragma once

#include <optional>
#include <string>
#include <vector>

namespace hacdcpf::analysis {

/// Stable status values returned by the Chapter 3 metric evaluator.
enum class ResilienceMetricStatus {
  Computed,
  Approximate,
  Unavailable,
  Invalid,
  NotApplicable,
  NotRequested,
};

const char* to_string(ResilienceMetricStatus status);

/// One normalized point from a distribution-resilience run.
/// `hour` is in hours; power fields are MW and ratios are dimensionless.
struct ResilienceMetricStep {
  double hour{0.0};
  double demand_mw{0.0};
  double served_mw{0.0};
  double shed_mw{0.0};
  double restoration_ratio{1.0};
  int active_faults{0};
  int repaired_faults{0};
  // Interval beginning at hour; absent for point-sampled reference curves.
  std::optional<double> duration_hr;
  std::optional<double> weighted_shed_mw;
  std::vector<double> shed_by_priority; // Critical, High, Medium, Low; MW
  std::optional<int> island_count;
  std::optional<int> switch_actions;
};

struct ResilienceMetricDefinition {
  std::string id;
  std::string name_zh;
  std::string name_en;
  std::string symbol;
  std::string phase;
  std::string topic;
  std::string formula_ref;
  std::string unit;
  std::string direction;
  std::string calculation_scope;
  std::string availability;
  std::vector<std::string> required_inputs;
  std::vector<std::string> source_notes;
  std::vector<std::string> limitations;
};

struct ResilienceMetricRun {
  std::vector<ResilienceMetricStep> steps;
  /// Required for a strict LEDSR result. It is not inferred from solver stages.
  std::optional<double> disaster_end_hr;
  /// Optional explicit start of restoration. If absent, the evaluator uses the
  /// first step after disaster_end_hr whose restoration ratio increases.
  std::optional<double> recovery_start_hr;
  /// Rich-load importance data is not assumed merely because a run has tiers.
  bool has_traceable_importance_mapping{false};
  bool usable{true}; // False for infeasible or incomplete solver artifacts.
  std::vector<std::string> limitations;
  std::optional<double> mess_energy_delivered_mwh;
  std::optional<double> mess_travel_distance_km;
};

struct ResilienceMetricEvaluationOptions {
  std::vector<std::string> selected_metric_ids;
  double target_ratio{0.9};
  /// Permit the documented system-level APDA proxy. False by default.
  bool allow_apda_system_gap_approximation{false};
  /// Permit the documented priority/rich-load RES proxy. False by default.
  bool allow_res_approximation{false};
};

struct ResilienceMetricResult {
  std::string id;
  ResilienceMetricStatus status{ResilienceMetricStatus::NotRequested};
  std::optional<double> value;
  std::string unit;
  std::string formula_ref;
  std::string direction;
  std::string scope;
  std::vector<std::string> source_fields;
  std::vector<std::string> source_notes;
  std::vector<std::string> assumptions;
  std::vector<std::string> limitations;
  std::vector<std::string> missing_dependencies;
  std::string reason_code;
  bool approximate{false};
  bool censored{false};
};

inline constexpr const char* resilience_metric_definition_version = "book_ch3_2026.2";
/// Chapter 3 catalog (18 entries) plus explicitly named operational metrics.
const std::vector<ResilienceMetricDefinition>& resilience_metric_catalog();

/// Evaluates only selected metrics. An empty selection evaluates all catalog
/// entries, which is useful for the result contract and focused tests.
std::vector<ResilienceMetricResult> evaluate_resilience_metrics(
    const ResilienceMetricRun& run,
    const ResilienceMetricEvaluationOptions& options = {});

}  // namespace hacdcpf::analysis
