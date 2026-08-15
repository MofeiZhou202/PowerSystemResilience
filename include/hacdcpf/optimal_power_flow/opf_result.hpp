#pragma once

/// optimal_power_flow/opf_result.hpp
/// =====================================
/// Result types for AC OPF, DC OPF, and parity IPM.
/// Consolidates: ac_opf.hpp (ACOPFResult) + dc_opf.hpp (DCOPFResult).

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "hacdcpf/model/converter_model_scope.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"  // DeviceTerminalFlow

namespace hacdcpf::opf {

// ═══════════════════════════════════════════════════════════════════════// OPF Feasibility Audit
//
// Independent post-solve feasibility check.  Populated by
// verify_opf_result() when called after solve_ac_opf().
// ═══════════════════════════════════════════════════════════════════
struct OpfAudit {
  bool   audited{false};                         ///< true after verify_opf_result() runs
  double max_power_balance_violation_mw{0.0};   ///< max |P_gen - P_load - P_loss| per bus
  double max_voltage_limit_violation_pu{0.0};   ///< max voltage outside [Vmin, Vmax]
  double max_branch_limit_violation_pu{0.0};    ///< max branch loading above rate_a_mva
  double max_gen_limit_violation_mw{0.0};       ///< max generator output outside [Pmin,Pmax]
  double objective_recomputed{0.0};             ///< cost recomputed from pg_mw / qg_mvar
  double objective_reported{0.0};               ///< cost as reported by the solver
  double objective_discrepancy_pct{0.0};        ///< |recomputed - reported| / |reported| * 100
  std::vector<std::string> violations;          ///< human-readable violation descriptions

  /// True if no violations were found.
  [[nodiscard]] bool feasible() const noexcept { return violations.empty(); }
};

// ═══════════════════════════════════════════════════════════════════// OPF solver path selector
// ═══════════════════════════════════════════════════════════════════════
enum class OPFSolverPath {
  Unknown,
  NativeAC,
  ParityIPM,
};

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Profiling
// ═══════════════════════════════════════════════════════════════════════
struct ACOPFProfiling {
  std::string linear_solver_backend;
  int analyze_calls{0};
  int factorization_calls{0};
  int linear_solve_calls{0};
  int total_iterations{0};
  int accepted_steps{0};
  int rejected_steps{0};
  int backend_escalations{0};
  int scaling_rebuilds{0};
  bool warm_start_used{false};
  bool prepared_session_used{false};
  bool formulation_reused{false};
  bool mapping_reused{false};
  bool symbolic_reused{false};
  bool continuation_state_reused{false};
  bool numeric_refactor_attempted{false};
  bool numeric_refactor_accepted{false};
  double numeric_refactor_relative_drift{
      std::numeric_limits<double>::infinity()};
  double numeric_refactor_backward_error{
      std::numeric_limits<double>::infinity()};
  std::string numeric_refactor_status{"not-requested"};
  std::string prepared_session_invalidation_reason{"not-prepared"};
  double initial_primal_residual{0.0};
  double initial_dual_residual{0.0};
  bool dc_phase_one_requested{false};
  bool dc_phase_one_accepted{false};
  int dc_phase_one_iterations{0};
  double dc_phase_one_runtime_ms{0.0};
  double dc_phase_one_time_limit_ms{0.0};
  bool dc_phase_one_budget_exhausted{false};
  double dc_phase_one_budget_overshoot_ms{0.0};
  int dc_phase_one_symbolic_analyze_calls{0};
  /// The AC baseline and DC candidate share this immutable formulation.
  int parity_formulation_builds{0};
  double dc_phase_one_residual{
      std::numeric_limits<double>::infinity()};
  double dc_phase_one_candidate_primal{
      std::numeric_limits<double>::infinity()};
  double dc_phase_one_candidate_dual{
      std::numeric_limits<double>::infinity()};
  double dc_phase_one_baseline_primal{
      std::numeric_limits<double>::infinity()};
  double dc_phase_one_baseline_dual{
      std::numeric_limits<double>::infinity()};
  std::string dc_phase_one_status{"not-requested"};
  double phase_one_initial_violation{
      std::numeric_limits<double>::infinity()};
  double phase_one_constraint_violation{
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
  bool phase_one_structural_step_attempted{false};
  bool phase_one_structural_step_accepted{false};
  int phase_one_structural_factorizations{0};
  double phase_one_structural_violation{
      std::numeric_limits<double>::infinity()};
  std::string phase_one_structure{"ac-state-basic"};
  double phase_one_runtime_ms{0.0};
  std::string phase_one_termination{"not-run"};
  std::string phase_one_linear_solver{"unselected"};
  bool dispatch_dual_predictor_attempted{false};
  bool dispatch_dual_predictor_accepted{false};
  double dispatch_dual_predictor_runtime_ms{0.0};
  double dispatch_dual_predictor_baseline_raw{
      std::numeric_limits<double>::infinity()};
  double dispatch_dual_predictor_candidate_raw{
      std::numeric_limits<double>::infinity()};
  double dispatch_dual_predictor_baseline_normalized{
      std::numeric_limits<double>::infinity()};
  double dispatch_dual_predictor_candidate_normalized{
      std::numeric_limits<double>::infinity()};
  std::string dispatch_dual_predictor_status{"not-attempted"};
  bool phase_two_start_accepted{false};
  std::string phase_two_start_rejection_reason;
  double max_ac_p_balance_residual_pu{0.0};
  double max_ac_q_balance_residual_pu{0.0};
  double max_dc_balance_residual_pu{0.0};
  double max_converter_balance_residual_pu{0.0};
  double max_other_equality_residual_pu{0.0};
  double max_nonlinear_inequality_violation_pu{0.0};
  double final_barrier_mu{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Result
// ═══════════════════════════════════════════════════════════════════════
struct ACOPFResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;
  std::vector<double> pg_mw;
  std::vector<double> qg_mvar;
  std::vector<double> external_grid_p_mw;
  std::vector<double> external_grid_q_mvar;

  std::vector<double> dpd_mw;
  std::vector<double> dqd_mvar;

  std::vector<double> pac_mw;
  std::vector<double> qac_mvar;

  /// LCC quasi-steady operating points evaluated at the optimized AC/DC
  /// voltages. Entries are keyed by LCCConverter.index and use authored bus
  /// identifiers, matching PowerFlowResult::lcc_transfers.
  std::vector<LCCTransfer> lcc_transfers;

  std::vector<double> pren_mw;
  std::vector<double> qren_mvar;
  std::vector<double> pstor_mw;
  std::vector<double> qstor_mvar;
  std::vector<double> pdcdc_mw;
  std::vector<double> pflex_mw;
  std::vector<double> er_port_p_mw;
  std::vector<double> er_port_q_mvar;

  std::vector<double> lmp_p;
  std::vector<double> lmp_q;

  /// True only when the returned nodal prices came from certified equality
  /// multipliers for the formulation that produced this primal solution.
  bool lmp_valid{false};
  std::string lmp_validity_reason;

  /// Opaque native-IPM continuation state.  Callers normally leave these
  /// untouched and pass this result through ACOPFOptions::warm_start.  The
  /// solver validates all dimensions before reuse and otherwise falls back to
  /// the public physical-variable mapping above.
  std::vector<double> ipm_primal_state;
  std::vector<double> ipm_equality_dual_state;
  std::vector<double> ipm_inequality_dual_state;
  std::vector<double> ipm_slack_state;
  /// Exact native-IPM layout identity. It is process-local metadata used to
  /// prevent dimension-compatible but semantically misaligned dual reuse.
  std::uint64_t ipm_layout_signature{0};

  /// Optional Davidenko homotopy tangent dw/dt at the returned point
  /// (primal, slack, equality-dual, inequality-dual), populated when
  /// ACOPFOptions::compute_homotopy_tangent is set; ACOPFOptions::homotopy_t
  /// supplies the cost scale t of the solved problem.  Objective-continuation
  /// drivers extrapolate the next path point as w + Δt·w′ with positivity
  /// repair on the barrier pairs.
  std::vector<double> ipm_tangent_primal;
  std::vector<double> ipm_tangent_slack;
  std::vector<double> ipm_tangent_equality_dual;
  std::vector<double> ipm_tangent_inequality_dual;

  struct ComponentRef {
    int original_index{0};
    int source_type{0};
  };
  std::vector<ComponentRef> gen_map;
  std::vector<ComponentRef> ren_map;
  std::vector<ComponentRef> stor_map;
  std::vector<ComponentRef> dcdc_map;
  std::vector<ComponentRef> flex_map;
  std::vector<ComponentRef> er_port_map;

  bool converged{false};
  int iterations{0};
  int outer_iterations{0};
  double objective{0.0};
  double max_constraint_violation{0.0};
  double max_stationarity{0.0};
  std::string status;
  OPFSolverPath solver_path{OPFSolverPath::Unknown};
  ACOPFProfiling profiling;

  /// Probable causes of infeasibility / non-convergence.
  /// Populated when converged==false by the solver and/or verify_opf_result().
  std::vector<std::string> infeasibility_hints;

  /// Independent post-solve feasibility audit.
  /// Populated by verify_opf_result(sys, result).
  OpfAudit audit;

  /// Declares which parts of the unified converter model this OPF honored.
  ConverterModelScope converter_model_scope{};

  /// Honest capability boundary for this particular solve path.  Entries are
  /// intended for API/GUI display and must describe omitted physics,
  /// approximations, or unavailable certificates rather than numerical hints.
  std::vector<std::string> model_limitations;

  // Switch / circuit-breaker terminal flows at the OPF dispatch point.
  std::vector<DeviceTerminalFlow> ac_switch_flows;
  std::vector<DeviceTerminalFlow> ac_circuit_breaker_flows;
};

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Jacobian Diagnostics
// ═══════════════════════════════════════════════════════════════════════
struct ACOPFJacobianDiagnostics {
  bool ok{false};
  double max_abs_error{0.0};
  double max_rel_error{0.0};
  int worst_row{-1};
  int worst_col{-1};
  std::string status;
};

// ═══════════════════════════════════════════════════════════════════════
// DC OPF Result
// ═══════════════════════════════════════════════════════════════════════
struct DCOPFResult {
  std::vector<double> va;
  std::vector<double> pg_mw;
  std::vector<double> external_grid_p_mw;
  std::vector<double> pf_mw;

  bool converged{false};
  int iterations{0};
  double objective{0.0};
  std::string status;
  std::string solver_name;
  double runtime_sec{0.0};

  /// Structure-aware NativeLCQP initialization diagnostics. The projection is
  /// built in original DCOPF coordinates before Ruiz scaling.
  bool structural_warm_start_requested{false};
  bool structural_warm_start_built{false};
  bool structural_warm_start_used{false};
  int structural_warm_start_components{0};
  int structural_warm_start_factorizations{0};
  double structural_warm_start_residual{
      std::numeric_limits<double>::infinity()};
  double solver_initial_primal_residual{
      std::numeric_limits<double>::infinity()};
  std::string structural_warm_start_status{"not-requested"};
  bool phase_one_warm_start_only{false};
  bool phase_one_budget_exhausted{false};
  double phase_one_budget_overshoot_ms{0.0};
  int native_qp_symbolic_analyze_calls{0};
  double phase_one_iterate_residual{
      std::numeric_limits<double>::infinity()};

  std::vector<double> lmp;
  std::vector<double> branch_mu_lower;
  std::vector<double> branch_mu_upper;
  /// True only when branch_mu_lower/upper are reliable congestion duals.
  /// Current native supporting-LP extraction does not recover bounded Pf
  /// variable duals robustly, so this remains false for branch-limit studies.
  bool branch_mu_valid{false};
  std::string branch_mu_validity_reason;

  /// True when lmp[] was recovered from equality-row duals for the reported
  /// primal solution.  This does not imply branch congestion dual validity.
  bool lmp_valid{false};
  std::string lmp_validity_reason;

  std::vector<double> load_shedding_mw;
  double total_load_shedding_mw{0.0};

  /// Ordered list of solver backends attempted (most-recent last).
  /// Each entry is of the form "<backend>:<status>" where status is one of
  /// "ok" or a short failure reason. Recorded by solve_dc_opf so callers can
  /// audit the actual fallback chain rather than guessing from solver_name.
  std::vector<std::string> solver_chain;

  /// Objective model actually used to compute `objective`.
  /// One of: "QP" (true quadratic costs, 0.5 x'Qx + c'x + c0),
  ///         "LP" (linearised piecewise / linear costs).
  /// Disambiguates QP-vs-LP semantics across fallback paths.
  std::string objective_model;

  /// Effective number of linear intervals used for a quadratic-cost LP
  /// approximation. Zero means the selected backend used the true QP cost.
  int pwl_segments_effective{0};

  /// Declares converter-model fidelity.  DC-OPF models no converter physics,
  /// so all converter flags here stay false.
  ConverterModelScope converter_model_scope{};
  std::vector<std::string> model_limitations;
};

}  // namespace hacdcpf::opf
