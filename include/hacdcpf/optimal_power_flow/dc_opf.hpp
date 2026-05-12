#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/system.hpp"

namespace hacdcpf::opf {

// --------------------------------------------------------------------------
// DC OPF (Linearized AC OPF Approximation)
//
// The DC OPF uses the following simplifications:
//   - Voltage magnitudes fixed at 1.0 p.u.
//   - Reactive power flows ignored
//   - Active power flow: P_ij ≈ (θ_i - θ_j) / x_ij
//   - Losses ignored in power balance
//
// This results in a linear programming (LP) formulation:
//   min  Σ (c_2*Pg² + c_1*Pg + c_0)  [quadratic costs linearized via PWL]
//   s.t. Power balance at each bus
//        Generator limits: Pmin ≤ Pg ≤ Pmax
//        Branch flow limits: |P_ij| ≤ Pmax_ij
// --------------------------------------------------------------------------

enum class DCOPFSolverBackend {
  Auto,      // Use best available: NativeQP > Gurobi > HiGHS > Native
  Native,    // Built-in dual simplex solver (linearized costs only)
  NativeQP,  // Built-in LCQP solver (supports true quadratic costs)
  HiGHS,     // HiGHS LP/QP solver
  Gurobi     // Gurobi LP/QP solver (native C API)
};

struct DCOPFOptions {
  double feasibility_tol{1e-6};
  double optimality_tol{1e-6};
  int max_iterations{10000};
  bool verbose{false};
  
  // Solver backend selection
  DCOPFSolverBackend solver{DCOPFSolverBackend::Auto};
  
  // Cost linearization: number of PWL segments for quadratic cost curves
  int pwl_segments{4};
  
  // If true, include branch flow limits (thermal limits)
  bool include_branch_limits{true};
  
  // Branch limit margin (fraction, e.g., 0.9 means use 90% of line rating)
  double branch_limit_margin{1.0};

  // Load shedding: if true, add per-bus slack variables with VOLL penalty
  // so that infeasible systems shed load instead of returning failure.
  bool load_shedding{true};

  // Value of Lost Load ($/MWh). If 0.0, auto-compute from max gen cost.
  double voll{0.0};
};

struct DCOPFResult {
  // Bus voltage angles (radians)
  std::vector<double> va;
  
  // Generator active power dispatch (MW)
  std::vector<double> pg_mw;
  
  // Branch active power flows (MW), in from-bus direction
  std::vector<double> pf_mw;
  
  // Solution status
  bool converged{false};
  int iterations{0};
  double objective{0.0};
  std::string status;
  std::string solver_name;
  double runtime_sec{0.0};
  
  // Marginal prices (dual variables) at buses ($/MWh)
  std::vector<double> lmp;
  
  // Shadow prices on branch flow limits ($/MWh)
  std::vector<double> branch_mu_lower;  // Lower limit shadow price
  std::vector<double> branch_mu_upper;  // Upper limit shadow price

  // Per-bus load shedding (MW). Non-zero only when load_shedding is enabled
  // and the system has insufficient generation capacity.
  std::vector<double> load_shedding_mw;
  double total_load_shedding_mw{0.0};
};

// --------------------------------------------------------------------------
// Main solver entry points
// --------------------------------------------------------------------------

/// Solve DC OPF for the given system.
/// Returns optimal generator dispatch and bus angles.
DCOPFResult solve_dc_opf(const HybridPowerSystem& sys,
                         const DCOPFOptions& opt = {});

/// Check if the DC OPF solution is feasible for the given system.
/// Returns (feasible, max_violation, violation_description).
std::tuple<bool, double, std::string>
check_dc_opf_feasibility(const HybridPowerSystem& sys,
                         const DCOPFResult& result,
                         double tol = 1e-6);

}  // namespace hacdcpf::opf
