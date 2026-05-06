#pragma once

#include <memory>
#include <vector>

#include <Eigen/Sparse>

#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"
#include "hacdcpf/engine/solver/solver_adapter.hpp"

namespace hacdcpf::engine {

/// Result of an inertia-aware KKT factorization attempt.
/// For a nonconvex primal-dual IPM the saddle-point matrix
///   K(δ_W, δ_C) = [ H + δ_W I   Jgᵀ      ]
///                 [ Jg          -δ_C I   ]
/// must have inertia (n_free, m_eq, 0) to guarantee that the Newton direction
/// is a descent direction on the reduced space. Wrong signature means either
/// H has negative curvature on the tangent space (fix by boosting δ_W) or Jg
/// is rank-deficient (fix by boosting δ_C).
struct InertiaStatus {
  bool correct{false};            ///< true iff the factored system has inertia (n, m, 0)
  int n_pos{0};                   ///< eigenvalues detected positive (primal block)
  int n_neg{0};                   ///< eigenvalues detected negative (Schur block)
  int n_zero{0};                  ///< eigenvalues detected zero (rank deficiency)
  double delta_w_used{0.0};       ///< primal regularization applied on success
  double delta_c_used{0.0};       ///< dual regularization applied on success
  int factorization_attempts{0};  ///< count of (δ_W, δ_C) retries, diagnostic
};

/// Wächter-Biegler regularization schedule parameters for inertia correction.
/// Defaults match IPOPT's documented constants (see Ipopt Implementation Paper, 2006).
struct InertiaSettings {
  double delta_w_min{1e-20};
  double delta_w_0{1e-4};          ///< First positive δ_W when unregularized factor fails.
  double delta_w_max{1e40};        ///< Cap — exceeding requests restoration.
  double kappa_w_plus_first{100.0};///< Multiplier used the first time δ_W goes positive.
  double kappa_w_plus{8.0};        ///< Subsequent growth factor while inertia still wrong.
  double kappa_w_minus{1.0 / 3.0}; ///< δ_W_last ← κ_W⁻ · δ_W after a successful solve.
  double delta_c_stripe{1e-8};     ///< δ_C = delta_c_stripe · μ^{1/4} when Jg rank-deficient.
  double mu{1.0};                  ///< Current barrier parameter (only used for δ_C).
};

/// Cached KKT matrix and its LU factorization.
/// The KKT system has the saddle-point structure:
///   [ W + δI   Jg' ] [ dx      ]   [ rhs_x  ]
///   [ Jg      -δI  ] [ dlambda ] = [ rhs_eq ]
struct SparseKKTCache {
  Eigen::SparseMatrix<double> kkt;       ///< Regularised KKT matrix (after factorisation build)
  Eigen::SparseMatrix<double> kkt_orig;  ///< Copy used for iterative refinement residual
  std::unique_ptr<SparseLinearSolver> solver;
  bool factored{false};
  bool pattern_analyzed{false};
  int n{0};    ///< Primal dimension
  int meq{0};  ///< Number of equality constraints
  int dim{0};  ///< Total KKT dimension
  int nnz{0};  ///< Cached nonzero count for pattern reuse
};

/// Build and factorise the KKT matrix from W (primal Hessian/augmented),
/// jg (equality constraint Jacobian), and a Tikhonov regularisation reg.
/// Returns true on success; on failure cache.factored is set to false.
bool factor_kkt_sparse(SparseKKTCache& cache,
                       const Eigen::SparseMatrix<double>& w,
                       const Eigen::SparseMatrix<double>& jg,
                       double reg);

/// Back-solve the KKT system using a cached factorisation.
/// Splits the solution into primal (dx) and dual (dlambda) parts.
/// Applies up to 2 steps of iterative refinement.
/// Returns false if not factored or if NaN appears.
bool solve_kkt_sparse(SparseKKTCache& cache,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& dx,
                      Eigen::VectorXd& dlambda);

/// Solve the KKT system through the reduced Schur complement when the equality
/// block is small enough to treat densely.
/// Returns false if the reduced path is not applicable or fails numerically.
bool solve_kkt_reduced_sparse(const Eigen::SparseMatrix<double>& w,
                              const Eigen::SparseMatrix<double>& jg,
                              const Eigen::VectorXd& rhs,
                              Eigen::VectorXd& dx,
                              Eigen::VectorXd& dlambda,
                              int max_eq_dim,
                              double min_reg,
                              double max_reg);

/// Assemble the inequality residual vector h and its Jacobian jh from:
///   - nonlinear inequalities provided by prob.h / prob.jac_h
///   - variable lower bounds  (lb_cols: h_i = lb_i - x_i <= 0)
///   - variable upper bounds  (ub_cols: h_i = x_i - ub_i <= 0)
void split_inequalities(const NLPModel& prob,
                        const Eigen::VectorXd& x,
                        const std::vector<int>& lb_cols,
                        const std::vector<int>& ub_cols,
                        Eigen::VectorXd& h,
                        Eigen::SparseMatrix<double>& jh);

/// Factor and solve the regularized KKT system with Wächter-Biegler inertia
/// correction. Uses a Schur-complement strategy that relies only on Cholesky
/// (SPD) factorizations: `H + δ_W I` is factored by SimplicialLDLT — failure
/// signals negative curvature and triggers a δ_W bump. The Schur complement
/// `S = Jg (H+δ_W I)⁻¹ Jgᵀ + δ_C I` is then factored; failure signals a
/// rank-deficient equality Jacobian and triggers a δ_C bump.
///
/// Both SPD factorizations succeeding is equivalent to the augmented KKT
/// matrix having inertia (n_free, m_eq, 0) — the sufficient condition for
/// the Newton direction to be a descent direction on the reduced space.
///
/// `delta_w_last` is an in/out parameter used to warm-start the regularization
/// from the previous successful value, following the IPOPT schedule.
/// Returns false (and populates `status` with the closest attempt) if the cap
/// `settings.delta_w_max` is exceeded — callers should then trigger
/// feasibility restoration.
bool factor_and_solve_kkt_inertia_corrected(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::VectorXd& rhs,
    const InertiaSettings& settings,
    double& delta_w_last,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dlambda,
    InertiaStatus& status);

/// Assemble `H + diag(Σ)` for the primal-dual bound treatment used by the new
/// driver. `sigma_diag` is expected to be length `n` and typically holds
///   Σ_j = z_L_j / max(x_j - lb_j, ε) + z_U_j / max(ub_j - x_j, ε)
/// for j with finite bounds, 0 otherwise. Caller owns the scaling policy.
Eigen::SparseMatrix<double> assemble_primal_hessian_with_bounds(
    const Eigen::SparseMatrix<double>& hess,
    const Eigen::VectorXd& sigma_diag);

}  // namespace hacdcpf::engine
