#pragma once

#include <complex>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>

#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/dynamics/DynamicState.hpp"
#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/NetworkState.hpp"
#include "hacdcpf/dynamics/devices/DynamicDevice.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::dynamics {

struct DynamicACBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  int from_pos{-1};
  int to_pos{-1};
  bool in_service{true};
  Eigen::Matrix3cd y_ff{Eigen::Matrix3cd::Zero()};
  Eigen::Matrix3cd y_ft{Eigen::Matrix3cd::Zero()};
  Eigen::Matrix3cd y_tf{Eigen::Matrix3cd::Zero()};
  Eigen::Matrix3cd y_tt{Eigen::Matrix3cd::Zero()};
};

struct DynamicDCBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  int from_pos{-1};
  int to_pos{-1};
  bool in_service{true};
  double conductance_pu{0.0};
};

struct DynamicFaultShunt {
  int bus{0};
  int bus_pos{-1};
  bool is_ac{true};
  bool active{true};
  double g_pu{0.0};
  double b_pu{0.0};
  double clear_time_s{0.0};
};

struct DynamicACBusLoad {
  int bus{0};
  int bus_pos{-1};
  double p_mw{0.0};
  double q_mvar{0.0};
  double nominal_voltage_pu{1.0};
  double scale{1.0};
};

struct DynamicDCBusLoad {
  int bus{0};
  int bus_pos{-1};
  double p_mw{0.0};
  double scale{1.0};
};

class DynamicNetwork {
 public:
  using Complex = std::complex<double>;

  double base_mva{100.0};
  double frequency_hz{50.0};

  std::vector<int> ac_bus_ids;
  std::vector<int> dc_bus_ids;
  std::vector<DynamicACBranch> ac_branches;
  std::vector<DynamicDCBranch> dc_branches;
  std::vector<DynamicFaultShunt> fault_shunts;
  std::vector<DynamicACBusLoad> ac_bus_loads;
  std::vector<DynamicDCBusLoad> dc_bus_loads;

  std::unordered_map<int, int> ac_bus_pos_by_id;
  std::unordered_map<int, int> dc_bus_pos_by_id;

  Eigen::SparseMatrix<Complex> Yac_base;
  Eigen::SparseMatrix<double> Gdc_base;

  [[nodiscard]] int acPhaseNodeCount() const noexcept {
    return static_cast<int>(ac_bus_ids.size()) * 3;
  }

  [[nodiscard]] int dcBusCount() const noexcept {
    return static_cast<int>(dc_bus_ids.size());
  }

  [[nodiscard]] int acPhaseNode(int bus_pos, int phase) const noexcept {
    return 3 * bus_pos + phase;
  }

  [[nodiscard]] int acBusPosition(int bus_id) const;
  [[nodiscard]] int dcBusPosition(int bus_id) const;

  void rebuildBaseMatrices(double singular_regularization_pu);
  void assembleEffectiveMatrices(
      const DynamicStamp& stamp,
      double singular_regularization_pu,
      Eigen::SparseMatrix<Complex>& Yac_eff,
      Eigen::VectorXcd& Iac_eff,
      Eigen::SparseMatrix<double>& Gdc_eff,
      Eigen::VectorXd& Idc_eff) const;
};

struct DynamicSystem {
  HybridPowerSystem canonical_system;
  PowerFlowResult initial_power_flow;
  DynamicInitializationSummary initialization;
  DynamicNetwork network;
  std::vector<std::unique_ptr<DynamicDevice>> devices;
  std::vector<DynamicEvent> events;
  DynamicState x;
  NetworkState y;
  DynamicSolverOptions options;
  std::vector<std::string> warnings;

  void assignStateIndices();
  void initializeStatesFromPowerFlow();
  [[nodiscard]] bool solveNetwork(double t, std::string& error);
  [[nodiscard]] bool evaluateDerivatives(double t,
                                         const Eigen::VectorXd& state,
                                         Eigen::VectorXd& dxdt,
                                         std::string& error);
  [[nodiscard]] double derivativeInfinityNorm(double t, std::string& error);
  [[nodiscard]] int stateCount() const noexcept { return x.size(); }
};

}  // namespace hacdcpf::dynamics
