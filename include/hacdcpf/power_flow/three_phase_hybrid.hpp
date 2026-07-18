#pragma once

#include <array>
#include <complex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf::powerflow {

enum class ThreePhaseHybridPFControlMode {
  EqualPhasePQ = 0,
  GridFollowingPQ = 1,
  GridFormingVoltage = 2,
  EqualPhaseVdcQ = 3,
  GridFollowingVdcQ = 4,
  GridFormingVdc = 5,
};

struct ThreePhaseHybridPFConverter {
  std::vector<int> phase_nodes;
  int dc_terminal{-1};
  double efficiency{0.98};
  ThreePhaseHybridPFControlMode control_mode{
      ThreePhaseHybridPFControlMode::GridFollowingPQ};
  // Three-phase totals on the network per-unit base. For Vdc-Q modes,
  // p_set_pu is the Newton initial value and q_set_pu remains specified.
  double p_set_pu{0.0};
  double q_set_pu{0.0};
  double v_dc_set_pu{1.0};
  std::complex<double> internal_voltage_positive{1.0, 0.0};
  double virtual_r_pu{0.01};
  double virtual_x_pu{0.10};
};

struct ThreePhaseHybridPFCase {
  std::string name;
  double base_mva{1.0};
  Eigen::SparseMatrix<std::complex<double>> y_ac;
  Eigen::VectorXcd i_ac_fixed;
  Eigen::VectorXd p_load_pu;
  Eigen::VectorXd q_load_pu;
  Eigen::VectorXcd voltage_start;
  std::vector<int> ac_phase_index;
  std::vector<int> reference_nodes;
  Eigen::VectorXcd reference_voltage;

  Eigen::SparseMatrix<double> g_dc;
  Eigen::VectorXd p_dc_load_pu;
  Eigen::VectorXd v_dc_start;
  std::vector<ThreePhaseHybridPFConverter> converters;
};

struct ThreePhaseHybridPFOptions {
  int max_iterations{80};
  int max_line_search_steps{18};
  double tolerance{1e-9};
  double minimum_dc_voltage_pu{0.05};
  double minimum_positive_sequence_voltage_pu{1e-6};
  double armijo{1e-4};
  bool verbose{false};
};

struct ThreePhaseHybridPFConverterResult {
  std::vector<std::complex<double>> phase_power_pu;
  std::vector<std::complex<double>> phase_current_pu;
  double p_dc_pu{0.0};
  double solved_control{0.0};
  std::complex<double> internal_voltage_positive{0.0, 0.0};
  double current_vuf{0.0};
};

struct ThreePhaseHybridPFResult {
  bool converged{false};
  std::string status;
  int iterations{0};
  int variables{0};
  int equations{0};
  int jacobian_nonzeros{0};
  double residual{0.0};
  double runtime_ms{0.0};
  double max_vuf{0.0};
  double ac_active_balance_residual{0.0};
  double ac_reactive_balance_residual{0.0};
  double dc_balance_residual{0.0};
  double converter_coupling_residual{0.0};
  Eigen::VectorXcd voltage;
  Eigen::VectorXd dc_voltage;
  std::vector<ThreePhaseHybridPFConverterResult> converters;
};

ThreePhaseHybridPFResult solve_three_phase_hybrid_pf(
    const ThreePhaseHybridPFCase& problem,
    const ThreePhaseHybridPFOptions& options = {});

}  // namespace hacdcpf::powerflow
