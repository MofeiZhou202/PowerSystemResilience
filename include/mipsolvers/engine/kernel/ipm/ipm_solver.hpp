#pragma once

#include <mutex>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// Globalization strategy selector.
/// Merit (default) preserves the historical Mehrotra + normalized-residual
/// merit function backtracking line search. Filter enables the Wächter–Biegler
/// primal–dual filter line search with inertia-corrected Newton steps. Filter
/// must be combined with `use_inertia_correction=true` to retain its
/// nonconvex-robustness guarantee — the default configuration does so.
enum class Globalization { Merit, Filter };

/// Options for the Interior-Point Method driver. Filter-specific fields are
/// ignored when `globalization == Merit`.
struct IPMOptions {
  int max_iter{400};
  double tol_primal{1e-6};
  double tol_dual{1e-6};
  double tol_complementarity{1e-6};
  /// "Acceptable" convergence level (Ipopt-style): if the best iterate
  /// satisfies primal_feas ≤ tol_accept AND dual_feas ≤ tol_accept, it
  /// is accepted as a near-optimal solution even when the strict tolerances
  /// are not met.  Disabled when set to 0.
  double tol_accept{1e-2};
  double alpha_max{0.95};  // fraction-to-boundary τ
  int reduced_kkt_max_eq{8};
  int qn_sparse_block_size{8};
  int qn_max_blocks{6};
  bool verbose{false};

  // --- Filter-driver options (PR2+) -----------------------------------
  Globalization globalization{Globalization::Filter};
  bool use_inertia_correction{true};

  // Additional PR3 feature flags (all default-on; flip false to disable
  // individually).
  bool use_second_order_correction{true};
  bool use_restoration_phase{true};
  bool scale_problem{true};
  double scaling_g_max{100.0};
  double restoration_zeta{1e-4};

  // Barrier schedule (IPOPT Implementation Paper defaults).
  double mu_init{0.1};
  double mu_min{1e-11};
  double kappa_mu{0.2};
  double theta_mu{1.5};
  double kappa_epsilon{10.0};

  // Filter line search constants (Wächter–Biegler 2006, Table 1 defaults).
  double filter_gamma_theta{1e-5};
  double filter_gamma_phi{1e-5};
  double filter_s_theta{1.1};
  double filter_s_phi{2.3};
  double filter_delta{1.0};
  double filter_eta_phi{1e-4};
  double filter_theta_min_scale{1e-4};  // θ_min = scale · max(1, θ₀)
  double filter_alpha_min{1e-10};
};

/// Detailed IPM result (extends SolveResult with multiplier information).
struct IPMDetail {
  Eigen::VectorXd lambda_eq;  // equality multipliers
  Eigen::VectorXd mu_ineq;    // inequality multipliers
  Eigen::VectorXd z_slack;    // inequality slacks
  double complementarity{0.0};
};

/// Native Mehrotra predictor-corrector IPM for NLP problems.
/// Operates on NLPModel via callbacks: f, grad, hess, g/jac_g, h/jac_h.
/// Variable bounds are read from NLPModel::vars.
class NativeIPMAdapter final : public SolverAdapter {
 public:
  explicit NativeIPMAdapter(IPMOptions opt = {});
  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  SolveResult solve_nlp(const NLPModel& prob) const override;

  /// Solve and return extended result including multipliers.
  std::pair<SolveResult, IPMDetail> solve_nlp_detail(const NLPModel& prob) const;

 private:
    struct StartupCurvatureCache {
      bool valid{false};
      Eigen::VectorXd x;
      Eigen::VectorXd grad;
      Eigen::SparseMatrix<double> hess;
    };

  IPMOptions opt_;
    mutable std::mutex startup_cache_mutex_;
    mutable StartupCurvatureCache startup_cache_;
};

}  // namespace mipsolvers::engine
