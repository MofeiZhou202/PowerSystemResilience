#pragma once

#include <memory>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>

#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// Result of an inertia-aware KKT factorization attempt.
/// For a nonconvex primal-dual IPM the saddle-point matrix
///   K(δ_W, δ_C) = [ H + δ_W I   Jgᵀ      ]
///                 [ Jg          -δ_C I   ]
/// should have inertia (n_free, m_eq, 0) for the usual reduced-space descent
/// property. When the equality nullity is modest, the sparse implementation
/// certifies this condition directly on null(Jg). It falls back to the stronger
/// full-space condition H + δ_W I > 0 only when a reliable null-space basis
/// cannot be constructed economically.
struct InertiaStatus {
  bool correct{false};            ///< true iff the factored system has inertia (n, m, 0)
  int n_pos{0};                   ///< eigenvalues detected positive (primal block)
  int n_neg{0};                   ///< eigenvalues detected negative (Schur block)
  int n_zero{0};                  ///< eigenvalues detected zero (rank deficiency)
  double delta_w_used{0.0};       ///< primal regularization applied on success
  double delta_c_used{0.0};       ///< dual regularization applied on success
  int factorization_attempts{0};  ///< count of (δ_W, δ_C) retries, diagnostic
  bool direct_factor_inertia{false}; ///< inertia read from the LDLT factor
  bool reduced_space_certificate{false}; ///< inertia certified on null(Jg)
  int tangent_dimension{0};       ///< n - rank(Jg) used by the certificate
  double min_reduced_curvature{0.0}; ///< min eig of (Z'WZ, Z'Z)
  double reduced_curvature_margin{0.0}; ///< positive numerical margin required
  double nullspace_residual{0.0}; ///< infinity norm of Jg*Z
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
  int max_tangent_dimension{32};   ///< Dense null-space certificate size cap.
  double reduced_curvature_tolerance{1e-10}; ///< Absolute projected-eigenvalue margin floor.
  double nullspace_residual_tolerance{1e-8}; ///< Relative acceptance tolerance for Jg*Z.
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
  std::vector<int> pattern_outer;  ///< Exact compressed-column outer pattern
  std::vector<int> pattern_inner;  ///< Exact compressed-column row indices
  int symbolic_analyses{0};        ///< Number of ordering/symbolic analyses
  int numeric_factorizations{0};   ///< Number of numeric factorizations
  int linear_solves{0};            ///< Number of back-solves, including refinement

  // ── Assembly scatter map (analyze-once for the [W; Jg] → KKT structure) ──
  // Positions of each source entry inside kkt's CSC value array.  Rebuilt
  // only when the (w, jg) sparsity structure changes; per-iteration assembly
  // then refills values in O(nnz) with no triplet allocation — the same
  // "analyze once, factorize many" contract as the IPM-LP kernel.
  int asm_w_nnz{-1};               ///< Fingerprint: nnz(W) (-1 = map not built)
  int asm_jg_nnz{-1};              ///< Fingerprint: nnz(Jg)
  std::vector<int> asm_w_outer;    ///< Fingerprint: W column pointers
  std::vector<int> asm_w_inner;    ///< Fingerprint: W row indices
  std::vector<int> asm_jg_outer;   ///< Fingerprint: Jg column pointers
  std::vector<int> asm_jg_inner;   ///< Fingerprint: Jg row indices
  std::vector<int> asm_w_pos;      ///< nnz(W): value index of W(i,j) in kkt
  std::vector<int> asm_jg_top;     ///< nnz(Jg): value index of Jg(r,c) at (n+r, c)
  std::vector<int> asm_jg_bot;     ///< nnz(Jg): value index of Jg(r,c) at (c, n+r)
  std::vector<int> asm_diag_w;     ///< n: value index of (i,i)
  std::vector<int> asm_diag_c;     ///< meq: value index of (n+i, n+i)
};

/// Persistent factorization state for the filter-IPM augmented KKT system.
///
/// The primal LDLT certifies W + delta_W I is positive definite. The sparse
/// augmented factor then solves
///
///   [ W + delta_W I   Jg'       ] [dx] = [rx]
///   [ Jg              -delta_C I] [dl]   [rg].
///
/// If delta_C is zero and this matrix is nonsingular, Jg has full row rank and
/// the inertia is (n, meq, 0). A positive delta_C makes the same inertia
/// unconditional once the primal block is positive definite. The augmented
/// factor is retained for all predictor/corrector/SOC right-hand sides, while
/// both primal and augmented symbolic analyses are reused across iterations
/// whenever their exact compressed sparsity patterns are unchanged.
struct SparseInertiaKKTCache {
  SparseKKTCache augmented;
  SparseKKTCache tangent_basis;
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower> primal_ldlt;
  std::vector<int> primal_pattern_outer;
  std::vector<int> primal_pattern_inner;
  int primal_rows{0};
  bool primal_pattern_analyzed{false};
  bool factored{false};
  int primal_symbolic_analyses{0};
  int primal_numeric_factorizations{0};
  std::vector<int> equality_pattern_outer;
  std::vector<int> equality_pattern_inner;
  std::vector<int> basic_columns;
  std::vector<int> free_columns;
  std::vector<int> preferred_free_columns;
  int equality_rows{0};
  bool tangent_partition_valid{false};
  int tangent_partition_strategy{0}; ///< 1=model hint, 2=natural, 3=sparse QR
  int tangent_partition_analyses{0};
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

/// Factor cache.kkt as currently assembled (re-analyzing the pattern only
/// when the exact compressed structure changed).  Used by cached-assembly
/// paths that keep cache.kkt resident across calls (e.g. the augmented
/// Newton assembler in ipm_solver.cpp).
bool factor_current_kkt(SparseKKTCache& cache, int n, int meq);

/// Factor a sparse augmented KKT matrix with inertia-preserving
/// regularization. This operation performs no right-hand-side solve; call
/// solve_kkt_inertia_corrected_sparse() repeatedly for predictor, corrector,
/// and second-order-correction right-hand sides.
bool factor_kkt_inertia_corrected_sparse(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const InertiaSettings& settings,
    double& delta_w_last,
    SparseInertiaKKTCache& cache,
    InertiaStatus& status);

/// Solve against the most recently factored augmented KKT matrix. The numeric
/// and symbolic factors are not recomputed.
bool solve_kkt_inertia_corrected_sparse(
    SparseInertiaKKTCache& cache,
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

/// Compatibility wrapper that factors the sparse augmented KKT matrix and
/// solves one right-hand side. New IPM code should keep a
/// `SparseInertiaKKTCache` and call factor_kkt_inertia_corrected_sparse() once
/// per Newton matrix, followed by any number of
/// solve_kkt_inertia_corrected_sparse() back-solves.
///
/// For modest equality nullity, the implementation constructs `Jg Z = 0` from
/// a cached sparse state/control partition and certifies
/// `Z' (H + δ_W I) Z > 0`. When that dense certificate is too large or cannot
/// be built, a MUMPS build reads the negative-pivot count from the augmented
/// LDLT factor directly. Builds without an inertia-capable factor use the
/// stronger sparse-LDLT full-space certificate as a final fallback. No path
/// forms the generally dense Schur complement `Jg (H+δ_W I)⁻¹ Jgᵀ`.
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

}  // namespace mipsolvers::engine
