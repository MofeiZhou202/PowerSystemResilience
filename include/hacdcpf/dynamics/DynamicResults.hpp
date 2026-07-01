#pragma once

#include <complex>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct DynamicDeviceOutput {
  std::string name;
  std::string type;
  int component_index{0};
  int bus{0};
  std::string canvas_type;
  int canvas_index{-1};
  std::string component_domain;
  std::string source_type;
  std::map<std::string, double> values;
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
  int rejected_steps{0};

  std::vector<DynamicSnapshot> snapshots;
  std::vector<std::string> warnings;
  std::vector<std::string> applied_events;
  DynamicInitializationSummary initialization;

  [[nodiscard]] const DynamicSnapshot* final_snapshot() const noexcept {
    return snapshots.empty() ? nullptr : &snapshots.back();
  }
};

}  // namespace hacdcpf::dynamics
