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
  // Adapter DTO in phase-node coordinates. The rich VSCConverter remains the
  // authored source of truth; GFM reference and impedance fields are populated
  // through model::resolve_gfm_norton_parameters.
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
  double voltage_reference_angle_rad{0.0};
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

// Per-phase AC line thermal limit |sum_k coefficients[k]*v[nodes[k]]|^2 <=
// i_max_pu^2, where the coefficients are the series-admittance entries of the
// segment. Nodes are full AC phase-node indices; a segment incident to an
// eliminated passive node is recovered through the Kron map T automatically.
struct ACLineCurrentLimit {
  std::vector<int> nodes;
  std::vector<std::complex<double>> coefficients;
  double i_max_pu{0.0};
};

// DC branch current limit (conductance_pu*(u_from - u_to))^2 <= i_max_pu^2.
struct DCLineCurrentLimit {
  int from_node{-1};
  int to_node{-1};
  double conductance_pu{0.0};
  double i_max_pu{0.0};
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

  std::vector<ACLineCurrentLimit> ac_line_limits;
  std::vector<DCLineCurrentLimit> dc_line_limits;
};

enum class ModelVariant {
  Full,
  GraphReduced,
};

enum class SolverBackend {
  Ipopt,
  NativeIPM,
};

/// Initial values used for an independently perturbed parametric OPF.
/// PrimalOnly supplies only the saved OPF variables. PrimalDual additionally
/// supplies equality, inequality, and variable-bound multipliers. Native IPM
/// also reuses nonlinear slacks; Ipopt reconstructs its internal slacks.
enum class ParametricWarmStartMode { PrimalOnly, PrimalDual };

struct ThreePhaseHybridOPFOptions {
  ModelVariant variant{ModelVariant::Full};
  SolverBackend backend{SolverBackend::Ipopt};
  graph::SparseKronOptions reduction_options{};
  int max_iterations{300};
  /// Native filter-IPM work budgets before and during feasibility restoration.
  /// The final primal-dual retry retains max_iterations. A nonpositive primary
  /// cap disables early restoration; a nonpositive restoration cap is clamped
  /// to one iteration.
  int native_primary_max_iterations_before_restoration{20};
  int native_restoration_max_iterations{100};
  double tolerance{1e-7};
  /// Cooperative wall-clock budget for Phase I. A sparse factorization is an
  /// indivisible unit and may finish after this deadline; the factorization
  /// cap below is the hard finite-work bound.
  double phase_one_time_limit_ms{5000.0};
  int phase_one_max_iterations{12};
  int phase_one_max_factorizations{14};
  int phase_one_max_backtracks{12};
  double phase_one_barrier_mu{0.1};
  double phase_one_admission_mu_factor{1.0};
  double phase_one_primal_mu_factor{0.1};
  double phase_one_centrality_tolerance{0.5};
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
  Eigen::VectorXd variable_lower_bound_dual_start;
  Eigen::VectorXd variable_upper_bound_dual_start;
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
  bool phase_one_in_handoff_corridor{false};
  bool phase_one_dual_initialized{false};
  double phase_one_handoff_primal_tolerance{0.0};
  double phase_one_perturbed_primal_residual{
      std::numeric_limits<double>::infinity()};
  double phase_one_centrality{
      std::numeric_limits<double>::infinity()};
  double phase_one_barrier_mu{0.0};
  bool phase_one_budget_exhausted{false};
  int phase_one_iterations{0};
  int phase_one_factorizations{0};
  int phase_one_backtracks{0};
  double phase_one_runtime_ms{0.0};
  std::string phase_one_termination{"not-run"};
  std::string phase_one_linear_solver{"unselected"};
  bool phase_two_start_requested{false};
  bool phase_two_start_accepted{false};
  std::string phase_two_start_rejection_reason;
  /// Numeric KKT backend selected by Phase II. "unselected" is valid when
  /// the accepted Phase I point satisfies termination before factorization.
  std::string phase_two_linear_solver_backend{"unselected"};
  /// Native Phase-II solve diagnostics. Constraint-generation results report
  /// the final restricted solve rather than a sum over enrichment rounds.
  int phase_two_initial_attempt_iterations{0};
  int phase_two_initial_attempt_factorizations{0};
  int phase_two_total_factorizations{0};
  int phase_two_restoration_factorizations{0};
  int phase_two_retry_factorizations{0};
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
  Eigen::VectorXd variable_lower_bound_dual;
  Eigen::VectorXd variable_upper_bound_dual;
  bool primal_dual_warm_start_used{false};
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
    const ThreePhaseHybridOPFResult& base_result,
    const std::vector<ThreePhaseHybridOPFCase>& perturbed_problems,
    const ThreePhaseHybridOPFOptions& options = {},
    ParametricWarmStartMode warm_start_mode =
        ParametricWarmStartMode::PrimalDual);

}  // namespace hacdcpf::opf::phase_hybrid
