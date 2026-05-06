#pragma once

#include <string>

#include "hacdcpf/engine/solver/solver_adapter.hpp"

namespace hacdcpf::engine {

/// Options for primal-dual hybrid gradient (PDHG/PDLP-style) LP solve.
/// Targets very large sparse LPs where factorization-based methods do not scale.
///
/// Algorithmic components (Google OR-Tools PDLP paper, Applegate et al. 2023):
///   1. Ruiz row+column equilibration with exact unscaling
///   2. Pock-Chambolle diagonal preconditioning with primal-weight balancing
///   3. Adaptive step-size gain based on progress monitoring
///   4. Adaptive restarts triggered by normalized duality gap improvement
///   5. Running-average convergence criteria on ergodic iterates
///   6. Raw CSC/CSR SpMV kernels — zero Eigen overhead per iteration
struct PDLPOptions {
  int max_iter{100000};
  int check_interval{40};
  double tol_primal{1e-6};
  double tol_dual{1e-6};
  double tol_gap{1e-6};
  double step_scale{0.99};

  // Ruiz equilibration rounds (row + column). 0 = disable.
  int ruiz_rounds{10};

  // Adaptive restart: restart when metric ratio drops below this threshold.
  double restart_threshold{0.36};

  // Periodic fallback restart interval (0 = disable).
  int restart_interval{500};

  // Primal weight adaptation: rebalance when primal/dual ratio exceeds this.
  double primal_weight_update_threshold{5.0};

  // --- IP-PMM step-size regularization (Pougkakiotis & Gondzio 2021) ---
  // Adds ρ and δ to step-size denominators:
  //   τ[j] = scale * pw / max(col_asum[j] + ρ, ε)
  //   σ[i] = scale / (pw * max(row_asum[i] + δ, ε))
  // This prevents step-size blowup on near-zero columns/rows, improving
  // convergence stability on ill-conditioned LPs.
  // The regularization decays adaptively as the solver converges.
  bool use_pmm_regularization{false};

  // Initial primal step-size regularization ρ₀.
  double pmm_rho_init{0.01};

  // Initial dual step-size regularization δ₀.
  double pmm_delta_init{0.01};

  // Minimum allowed regularization floor.
  double pmm_reg_min{1e-12};

  // Decay threshold: use faster decay when residual improves by this factor.
  double pmm_update_threshold{0.90};

  bool verbose{false};
};

/// Presets for specific use-cases.
namespace pdlp_presets {

/// Fast root-LP solve: medium tolerance, aggressive restart.
inline PDLPOptions root_lp() {
  PDLPOptions o;
  o.max_iter = 80000;
  o.check_interval = 40;
  o.tol_primal = 1e-6;
  o.tol_dual = 1e-6;
  o.tol_gap = 1e-6;
  o.ruiz_rounds = 10;
  o.restart_threshold = 0.3;
  o.restart_interval = 400;
  o.use_pmm_regularization = true;
  o.pmm_rho_init = 1.0;
  o.pmm_delta_init = 1.0;
  return o;
}

/// MILP node LP: loose tolerance, minimal overhead, very fast.
inline PDLPOptions milp_node() {
  PDLPOptions o;
  o.max_iter = 5000;
  o.check_interval = 25;
  o.tol_primal = 1e-4;
  o.tol_dual = 1e-4;
  o.tol_gap = 1e-4;
  o.ruiz_rounds = 5;
  o.restart_threshold = 0.4;
  o.restart_interval = 200;
  o.use_pmm_regularization = true;
  o.pmm_rho_init = 0.5;
  o.pmm_delta_init = 0.5;
  return o;
}

/// High-precision solve for final polishing.
inline PDLPOptions high_precision() {
  PDLPOptions o;
  o.max_iter = 200000;
  o.check_interval = 80;
  o.tol_primal = 1e-8;
  o.tol_dual = 1e-8;
  o.tol_gap = 1e-8;
  o.ruiz_rounds = 15;
  o.restart_threshold = 0.25;
  o.restart_interval = 600;
  o.use_pmm_regularization = true;
  o.pmm_rho_init = 2.0;
  o.pmm_delta_init = 2.0;
  o.pmm_reg_min = 1e-14;
  return o;
}

}  // namespace pdlp_presets

/// Native LP adapter based on first-order primal-dual iterations.
class NativePDLPAdapter final : public SolverAdapter {
 public:
  explicit NativePDLPAdapter(PDLPOptions opt = {});

  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_lp(const LPModel& prob) const override;

 private:
  PDLPOptions opt_;
};

}  // namespace hacdcpf::engine
