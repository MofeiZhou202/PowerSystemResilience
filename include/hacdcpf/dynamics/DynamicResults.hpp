#pragma once

#include <complex>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct DynamicModelProfile {
  std::string standard;
  std::string profile;
  std::string model_name;
  std::string parameter_set;
  std::string source_id;
  std::string notes;
};

struct DynamicDeviceOutput {
  std::string name;
  std::string type;
  int component_index{0};
  int bus{0};
  std::string canvas_type;
  int canvas_index{-1};
  std::string component_domain;
  std::string source_type;
  std::string model_standard;
  std::string model_name;
  std::string parameter_set;
  std::vector<DynamicModelProfile> model_profiles;
  std::map<std::string, double> values;
};

struct DynamicAppliedEventRecord {
  double time_s{0.0};
  std::string type;
  std::string label;
  int component_index{0};
  int target_id{0};
  int bus{0};
  int phase{-1};
  double value{0.0};
  double duration_s{0.0};
  std::string component_type;
  std::string target_type;
  std::map<std::string, double> params;
};

struct DynamicResidualDiagnostic {
  std::string device_name;
  std::string device_type;
  int component_index{0};
  int state_index{-1};
  double residual{0.0};
};

struct DynamicInitializationSummary {
  bool power_flow_requested{true};
  bool power_flow_converged{false};
  bool fallback_voltage_setpoints{false};
  int iterations{0};
  double residual{0.0};
  bool dynamic_trim_converged{false};
  int dynamic_trim_iterations{0};
  double dynamic_initial_dxdt_inf_norm{0.0};
  double dynamic_fast_dxdt_inf_norm{0.0};
  double min_ac_voltage_pu{0.0};
  double max_ac_voltage_pu{0.0};
  double min_dc_voltage_pu{0.0};
  double max_dc_voltage_pu{0.0};
  std::vector<DynamicResidualDiagnostic> dynamic_residual_diagnostics;
  std::vector<std::string> warnings;
};

struct DynamicSnapshot {
  double time_s{0.0};
  Eigen::VectorXd state;
  Eigen::VectorXcd vac_abc;
  Eigen::VectorXd vdc;
  double max_ac_voltage_pu{0.0};
  double min_ac_voltage_pu{0.0};
  double max_dc_voltage_pu{0.0};
  double min_dc_voltage_pu{0.0};
  double frequency_hz{0.0};
  std::vector<DynamicDeviceOutput> device_outputs;
};

struct DynamicResults {
  bool success{false};
  std::string message;
  int steps{0};
  int failed_step{-1};
  int newton_iterations{0};
  int jacobian_evaluations{0};
  int jacobian_residual_evaluations{0};
  int jacobian_fd_columns{0};
  int jacobian_colored_groups{0};
  int linear_factorizations{0};
  int rejected_steps{0};
  double max_local_error_norm{0.0};
  double min_accepted_step_s{0.0};
  double max_accepted_step_s{0.0};

  std::vector<DynamicSnapshot> snapshots;
  std::vector<std::string> warnings;
  std::vector<std::string> applied_events;
  std::vector<DynamicAppliedEventRecord> applied_event_records;
  DynamicInitializationSummary initialization;

  [[nodiscard]] const DynamicSnapshot* final_snapshot() const noexcept {
    return snapshots.empty() ? nullptr : &snapshots.back();
  }
};

struct DynamicResultExportOptions {
  bool include_ac_voltage_magnitudes{true};
  bool include_dc_voltages{true};
  bool include_device_outputs{true};
  char delimiter{','};
};

std::string to_csv(const DynamicResults& results,
                   const DynamicResultExportOptions& options = {});

}  // namespace hacdcpf::dynamics
