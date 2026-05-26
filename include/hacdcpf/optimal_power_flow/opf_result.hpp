#pragma once

/// optimal_power_flow/opf_result.hpp
/// =====================================
/// Result types for AC OPF, DC OPF, and parity IPM.
/// Consolidates: ac_opf.hpp (ACOPFResult) + dc_opf.hpp (DCOPFResult).

#include <string>
#include <vector>

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

  std::vector<double> lmp_p;
  std::vector<double> lmp_q;

  struct ComponentRef {
    int original_index{0};
    int source_type{0};
  };
  std::vector<ComponentRef> ren_map;
  std::vector<ComponentRef> stor_map;
  std::vector<ComponentRef> dcdc_map;
  std::vector<ComponentRef> flex_map;

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
};

}  // namespace hacdcpf::opf
