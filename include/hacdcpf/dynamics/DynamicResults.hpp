#pragma once

#include <complex>
#include <string>
#include <vector>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

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
};

struct DynamicResults {
  bool success{false};
  std::string message;
  int steps{0};
  int failed_step{-1};

  std::vector<DynamicSnapshot> snapshots;
  std::vector<std::string> warnings;
  std::vector<std::string> applied_events;

  [[nodiscard]] const DynamicSnapshot* final_snapshot() const noexcept {
    return snapshots.empty() ? nullptr : &snapshots.back();
  }
};

}  // namespace hacdcpf::dynamics
