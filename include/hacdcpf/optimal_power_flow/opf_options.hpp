#pragma once

/// optimal_power_flow/opf_options.hpp
/// =====================================
/// Options for AC OPF, DC OPF, and combined OPF solvers.
/// Consolidates: ac_opf.hpp (ACOPFOptions) + dc_opf.hpp (DCOPFOptions).

#include <string>

namespace hacdcpf::opf {

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Solver Backend
// ═══════════════════════════════════════════════════════════════════════
/// Selects which nonlinear-OPF engine `solve_ac_opf` uses.
///
/// - `Auto`             : parity full-space IPM first, then Ipopt as a fallback
///                        if the parity IPM does not converge (and Ipopt is
///                        compiled in).  Pure-AC cases may still use the fast
///                        economic-dispatch path only when explicitly selected.
/// - `ParityIPM`        : the self-developed full-space primal-dual IPM.
/// - `Ipopt`            : the embedded Ipopt (filter line-search) NLP solver,
///                        applied to the same parity formulation.
/// - `EconomicDispatch` : merit-order economic dispatch + a single AC power
///                        flow.  Fast but suboptimal; pure-AC systems only.
enum class ACOPFSolverBackend {
  Auto,
  ParityIPM,
  Ipopt,
  EconomicDispatch
};

// ═══════════════════════════════════════════════════════════════════════
// AC OPF Options
// ═══════════════════════════════════════════════════════════════════════
struct ACOPFOptions {
  /// Preferred nonlinear-OPF backend.  When left at `Auto`, the legacy
  /// `use_parity_ipm` / `enable_primal_dual` flags continue to control path
  /// selection (with an Ipopt fallback for hybrid AC/DC and large cases).
  ACOPFSolverBackend ac_solver_backend{ACOPFSolverBackend::Auto};

  int max_inner_iterations{80};
  int max_outer_iterations{8};
  int max_line_search_steps{20};

  double feasibility_tol{1e-6};
  double stationarity_tol{1e-6};

  double barrier_mu0{1e-2};
  double barrier_mu_reduction{0.2};
  double barrier_mu_min{1e-8};

  double merit_penalty{10.0};
  double regularization{1e-6};
  double step_backoff{0.5};
  double interior_fraction{0.995};

  int ac_eval_threads{1};
  bool enable_primal_dual{false};
  bool use_parity_ipm{false};
  bool allow_fallback{true};
  bool verbose{false};

  // Constraint-family toggles for the hybrid parity-IPM formulation
  // (multi-converter model §3.1).  Default true keeps the always-enforce
  // behavior; a GUI/caller can disable a family to relax the problem.
  bool enforce_branch_limits{true};
  bool enforce_converter_capacity{true};
  bool enforce_converter_current_limits{true};
  bool enforce_converter_modulation_limits{true};
};

// ═══════════════════════════════════════════════════════════════════════
// DC OPF Solver Backend
// ═══════════════════════════════════════════════════════════════════════
enum class DCOPFSolverBackend {
  Auto,
  Native,
  NativeQP,
  HiGHS,
  Gurobi
};

// ═══════════════════════════════════════════════════════════════════════
// DC OPF Options
// ═══════════════════════════════════════════════════════════════════════
struct DCOPFOptions {
  double feasibility_tol{1e-6};
  double optimality_tol{1e-6};
  int max_iterations{10000};
  bool verbose{false};

  DCOPFSolverBackend solver{DCOPFSolverBackend::Auto};

  int pwl_segments{4};

  bool include_branch_limits{true};
  double branch_limit_margin{1.0};

  bool load_shedding{true};
  double voll{0.0};

  // When true (default), the solver runs a supporting simplex LP after the
  // primal solve to recover constraint dual variables (LMPs and congestion
  // prices).  Set to false when only the primal dispatch is needed and the
  // extra LP solve overhead should be avoided.
  bool compute_lmp{true};
};

}  // namespace hacdcpf::opf
