#pragma once

#include <complex>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/graph/sparse_kron_reduction.hpp"
#include "hacdcpf/power_flow/three_phase_hybrid.hpp"

namespace hacdcpf::opf::phase_hybrid {

struct PhaseGenerator {
  int phase_node{-1};
  double p_min_pu{0.0};
  double p_max_pu{0.0};
  double q_min_pu{0.0};
  double q_max_pu{0.0};
  double cost_c2{0.0};
  double cost_c1{0.0};
};

enum class PhaseVSCControlMode {
  EqualPhasePower = 0,
  GridFollowingPLL = 1,
  PositiveSequenceCurrent = GridFollowingPLL,
  GridFormingDroop = 2,
};

struct PhaseVSC {
  std::vector<int> phase_nodes;
  int dc_terminal{-1};
  double efficiency{0.98};
  double s_max_pu{0.0};
  double phase_current_max_pu{0.0};
  bool fixed_unity_power_factor{false};
  PhaseVSCControlMode control_mode{PhaseVSCControlMode::EqualPhasePower};
  double nominal_frequency_hz{50.0};
  double pll_kp{0.01};
  double pll_ki{1.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.10};
  double p_droop_pu{0.01};
  double q_droop_pu{0.05};
  double voltage_reference_pu{1.0};
  double voltage_integral_gain{10.0};
};

struct PhaseVSCDynamicEquilibrium {
  PhaseVSCControlMode control_mode{PhaseVSCControlMode::EqualPhasePower};
  double pll_angle_rad{0.0};
  double pll_integrator{0.0};
  std::complex<double> internal_voltage_positive{0.0, 0.0};
  double active_power_reference_pu{0.0};
  double reactive_power_reference_pu{0.0};
  double voltage_reference_pu{1.0};
  double filtered_active_power_pu{0.0};
  double filtered_reactive_power_pu{0.0};
  double voltage_integrator{0.0};
  double max_differential_residual{0.0};
};

struct ThreePhaseHybridOPFCase {
  std::string name;
  double base_mva{1.0};
  graph::SparseComplexMatrix y_ac;
  Eigen::VectorXcd i_ac_fixed;
  Eigen::VectorXd p_load_pu;
  Eigen::VectorXd q_load_pu;
  Eigen::VectorXd v_min_pu;
  Eigen::VectorXd v_max_pu;
  Eigen::VectorXcd voltage_start;
  std::vector<int> ac_phase_index;

  std::vector<int> reference_nodes;
  Eigen::VectorXcd reference_voltage;
  std::vector<std::vector<int>> three_phase_bus_nodes;
  double vuf_max{0.03};

  std::vector<PhaseGenerator> generators;
  std::vector<PhaseVSC> converters;
  Eigen::SparseMatrix<double> g_dc;
  Eigen::VectorXd p_dc_load_pu;
  Eigen::VectorXd v_dc_start;
  Eigen::VectorXd v_dc_min_pu;
  Eigen::VectorXd v_dc_max_pu;
  std::vector<int> dc_reference_terminals;
  Eigen::VectorXd dc_reference_voltage_pu;
};

enum class ModelVariant {
  Full,
  GraphReduced,
};

enum class SolverBackend {
  Ipopt,
  NativeIPM,
};

struct ThreePhaseHybridOPFOptions {
  ModelVariant variant{ModelVariant::Full};
  SolverBackend backend{SolverBackend::Ipopt};
  graph::SparseKronOptions reduction_options{};
  int max_iterations{300};
  double tolerance{1e-7};
  /// Cooperative wall-clock budget for Phase I. A sparse factorization is an
  /// indivisible unit and may finish after this deadline; the factorization
  /// cap below is the hard finite-work bound.
  double phase_one_time_limit_ms{5000.0};
  int phase_one_max_iterations{12};
  int phase_one_max_factorizations{14};
  int phase_one_max_backtracks{12};
  /// Legacy unlimited-budget fallback. It is considered only when
  /// phase_one_time_limit_ms <= 0; finite-budget Phase I never invokes Ipopt.
  bool warm_start_with_ipopt{false};
  bool verify_derivatives{false};
  bool verbose{false};
  bool use_constraint_oracle{false};
  double oracle_initial_margin{1e-3};
  double oracle_activation_margin{1e-5};
  int oracle_probe_iterations{0};
  int oracle_max_rounds{8};
  std::vector<int> oracle_seed_rows;
  // Internal/advanced override used by the exact constraint-oracle rounds.
  // Empty means that every nonlinear inequality is enforced.
  std::vector<int> enforced_inequality_rows;
  Eigen::VectorXd primal_start;
  Eigen::VectorXd equality_dual_start;
  Eigen::VectorXd nonlinear_inequality_dual_start;
  Eigen::VectorXd nonlinear_slack_start;
};

struct ConstraintOracleRoundDiagnostic {
  int round{0};
  int enforced_rows{0};
  int added_rows{0};
  int iterations{0};
  bool restricted_converged{false};
  double runtime_ms{0.0};
  double max_full_inequality{0.0};
};

struct ThreePhaseHybridOPFResult {
  bool converged{false};
  std::string status;
  std::string solver;
  int iterations{0};
  int variables{0};
  int equalities{0};
  int inequalities{0};
  int enforced_inequalities{0};
  int constraint_oracle_rounds{0};
  int constraint_oracle_added_rows{0};
  int equality_jacobian_nonzeros{0};
  int inequality_jacobian_nonzeros{0};
  double max_omitted_inequality{
      -std::numeric_limits<double>::infinity()};
  int eliminated_phase_nodes{0};
  double objective{0.0};
  double runtime_ms{0.0};
  double initial_primal_residual{0.0};
  double initial_dual_residual{0.0};
  /// Phase I certificate in the assembled OPF's original per-unit
  /// coordinates. Phase II preserves the point only after MIPSolvers repeats
  /// this audit, including variable bounds, at its primal tolerance.
  double phase_one_constraint_violation{
      std::numeric_limits<double>::infinity()};
  double phase_one_initial_violation{
      std::numeric_limits<double>::infinity()};
  double phase_one_dual_fit_residual{
      std::numeric_limits<double>::infinity()};
  bool phase_one_primal_feasible{false};
  bool phase_one_dual_initialized{false};
  bool phase_one_budget_exhausted{false};
  int phase_one_iterations{0};
  int phase_one_factorizations{0};
  int phase_one_backtracks{0};
  double phase_one_runtime_ms{0.0};
  std::string phase_one_termination{"not-run"};
  std::string phase_one_linear_solver{"unselected"};
  bool phase_two_start_requested{false};
  bool phase_two_start_accepted{false};
  /// Numeric KKT backend selected by Phase II. "unselected" is valid when
  /// the accepted Phase I point satisfies termination before factorization.
  std::string phase_two_linear_solver_backend{"unselected"};
  int initial_worst_equality{-1};
  int initial_worst_inequality{-1};
  double primal_residual{0.0};
  double dual_residual{0.0};
  double complementarity{0.0};
  double max_voltage_violation{0.0};
  double max_vuf{0.0};
  double max_converter_current_vuf{0.0};
  double max_converter_current_loading{0.0};
  double max_converter_violation{0.0};
  double max_dynamic_equilibrium_residual{0.0};
  double max_equality_jacobian_error{0.0};
  double max_inequality_jacobian_error{0.0};
  double max_lagrangian_hessian_error{0.0};
  Eigen::VectorXd primal;
  Eigen::VectorXd equality_dual;
  Eigen::VectorXd inequality_dual;
  Eigen::VectorXd inequality_slack;
  Eigen::VectorXcd full_voltage;
  Eigen::VectorXd dc_voltage;
  std::vector<double> generator_active_power_pu;
  std::vector<double> generator_reactive_power_pu;
  std::vector<std::vector<std::complex<double>>> converter_phase_power_pu;
  std::vector<double> converter_dc_power_pu;
  std::vector<int> enforced_inequality_rows;
  std::vector<ConstraintOracleRoundDiagnostic> constraint_oracle_trace;
  std::vector<PhaseVSCDynamicEquilibrium> converter_dynamic_equilibria;
  graph::SparseKronResult reduction;
};

ThreePhaseHybridOPFResult solve_three_phase_hybrid_opf(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFOptions& options = {});

powerflow::ThreePhaseHybridPFCase make_three_phase_hybrid_pf_case(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFResult& operating_point);

std::vector<ThreePhaseHybridOPFResult> solve_three_phase_hybrid_opf_sequence(
    const std::vector<ThreePhaseHybridOPFCase>& problems,
    const ThreePhaseHybridOPFOptions& options = {});

std::vector<ThreePhaseHybridOPFResult> solve_three_phase_hybrid_opf_branches(
    const ThreePhaseHybridOPFCase& base_problem,
    const ThreePhaseHybridOPFResult& certified_base,
    const std::vector<ThreePhaseHybridOPFCase>& perturbed_problems,
    const ThreePhaseHybridOPFOptions& options = {});

}  // namespace hacdcpf::opf::phase_hybrid
