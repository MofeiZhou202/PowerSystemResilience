#pragma once

/// optimal_power_flow/opf_result.hpp
/// =====================================
/// Result types for AC OPF, DC OPF, and parity IPM.
/// Consolidates: ac_opf.hpp (ACOPFResult) + dc_opf.hpp (DCOPFResult).

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
  double initial_primal_residual{0.0};
  double initial_dual_residual{0.0};
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

  /// Opaque native-IPM continuation state.  Callers normally leave these
  /// untouched and pass this result through ACOPFOptions::warm_start.  The
  /// solver validates all dimensions before reuse and otherwise falls back to
  /// the public physical-variable mapping above.
  std::vector<double> ipm_primal_state;
  std::vector<double> ipm_equality_dual_state;
  std::vector<double> ipm_inequality_dual_state;
  std::vector<double> ipm_slack_state;

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

  std::vector<double> lmp;
  std::vector<double> branch_mu_lower;
  std::vector<double> branch_mu_upper;
  /// True only when branch_mu_lower/upper are reliable congestion duals.
  /// Current native supporting-LP extraction does not recover bounded Pf
  /// variable duals robustly, so this remains false for branch-limit studies.
  bool branch_mu_valid{false};

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

  /// Declares converter-model fidelity.  DC-OPF models no converter physics,
  /// so all converter flags here stay false.
  ConverterModelScope converter_model_scope{};
};

}  // namespace hacdcpf::opf
