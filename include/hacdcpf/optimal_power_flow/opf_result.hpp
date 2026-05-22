#pragma once

/// optimal_power_flow/opf_result.hpp
/// =====================================
/// Result types for AC OPF, DC OPF, and parity IPM.
/// Consolidates: ac_opf.hpp (ACOPFResult) + dc_opf.hpp (DCOPFResult).

#include <string>
#include <vector>

namespace hacdcpf::opf {

// ═══════════════════════════════════════════════════════════════════════
// OPF solver path selector
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

  std::vector<double> load_shedding_mw;
  double total_load_shedding_mw{0.0};
};

}  // namespace hacdcpf::opf
