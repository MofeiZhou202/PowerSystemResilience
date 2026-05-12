#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::opf {

// Which solver path and variable coverage was used for a completed solve.
enum class OPFSolverPath {
  Unknown,        // No solve yet or unknown
  NativeAC,       // Reduced-space AC OPF (generators only)
  ParityIPM,      // Full-space parity IPM (generators + DER + DC)
};

struct ACOPFOptions {
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
};

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

struct ACOPFResult {
  std::vector<double> vm;
  std::vector<double> va;
  std::vector<double> vdc;  // DC bus voltage magnitudes (p.u.)
  std::vector<double> pg_mw;
  std::vector<double> qg_mvar;

  // Per-bus load shedding from parity formulation (MW / Mvar)
  std::vector<double> dpd_mw;    // active load shed per bus
  std::vector<double> dqd_mvar;  // reactive load shed per bus

  // Converter operating points (bus injection convention, per in-service converter)
  std::vector<double> pac_mw;    // AC-side active power injection (positive = into AC bus)
  std::vector<double> qac_mvar;  // AC-side reactive power injection (positive = into AC bus)

  // Enhanced component dispatch (bus injection convention, per variable-eligible component).
  // These arrays are *packed*: entry [k] corresponds to the k-th component
  // that participated as a decision variable, NOT the k-th entry in the
  // original system table.  Use the companion ren_map / stor_map /
  // dcdc_map / flex_map vectors to recover original component identity.
  std::vector<double> pren_mw;   // curtailable renewable dispatch (MW, positive = injection)
  std::vector<double> qren_mvar; // curtailable renewable Q (MVAr, positive = injection)
  std::vector<double> pstor_mw;  // storage dispatch (MW, positive = discharge/injection)
  std::vector<double> qstor_mvar;// storage Q dispatch (MVAr, positive = injection)
  std::vector<double> pdcdc_mw;  // DCDC transfer (MW, positive = bus_in→bus_out)
  std::vector<double> pflex_mw;  // flexible load actual demand (MW, positive = consumption)

  // Locational Marginal Prices (from parity IPM dual variables)
  std::vector<double> lmp_p;    // active power LMP per AC bus ($/MWh)
  std::vector<double> lmp_q;    // reactive power LMP per AC bus ($/MVArh)

  // --- Component identity maps (parallel to the dispatch arrays above) ---
  // ren_map[k]:  original index into renewable_gens (if source_type==0)
  //              or pv_systems (if source_type==1).
  // stor_map/dcdc_map/flex_map: analogous for their respective collections.
  struct ComponentRef {
    int  original_index{0};   // position in the original system collection
    int  source_type{0};      // 0 = primary collection, 1 = secondary (PV)
  };
  std::vector<ComponentRef> ren_map;   // parallel to pren_mw / qren_mvar
  std::vector<ComponentRef> stor_map;  // parallel to pstor_mw / qstor_mvar
  std::vector<ComponentRef> dcdc_map;  // parallel to pdcdc_mw
  std::vector<ComponentRef> flex_map;  // parallel to pflex_mw

  bool converged{false};
  int iterations{0};
  int outer_iterations{0};
  double objective{0.0};
  double max_constraint_violation{0.0};
  double max_stationarity{0.0};
  std::string status;
  OPFSolverPath solver_path{OPFSolverPath::Unknown};
  ACOPFProfiling profiling;
};

struct ACOPFJacobianDiagnostics {
  bool ok{false};
  double max_abs_error{0.0};
  double max_rel_error{0.0};
  int worst_row{-1};
  int worst_col{-1};
  std::string status;
};

// AC OPF core with optional hybrid AC/DC primal-dual path.
// If primal-dual fails and `allow_fallback` is true, the solver falls back to
// the AC-only dispatch + PF path.
ACOPFResult solve_ac_opf(const HybridPowerSystem& sys, const ACOPFOptions& opt = {});

// Finite-difference check for OPF equality Jacobian assembly.
// Uses a central-difference probe around an interior initial point.
ACOPFJacobianDiagnostics check_ac_opf_jacobian_fd(const HybridPowerSystem& sys,
                                                  int max_columns = 64,
                                                  double fd_eps = 1e-6,
                                                  int ac_eval_threads = 1);

}  // namespace hacdcpf::opf
