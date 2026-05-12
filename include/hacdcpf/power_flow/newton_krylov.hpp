#pragma once

#include <functional>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/power_flow/jacobian_builder.hpp"

namespace hacdcpf::powerflow {

// ─── GMRES statistics ──────────────────────────────────────────────────────

/// Statistics returned from a GMRES solve.
struct GmresStats {
  int iterations{0};          ///< Total Arnoldi steps taken across all restarts.
  double final_relres{1.0};   ///< Final ‖r‖/‖b‖ (relative residual).
  bool converged{false};      ///< True if final_relres < tol.
};

// ─── GMRES(m) solver ──────────────────────────────────────────────────────

/// Solve A·x = b by restarted GMRES(restart) with left preconditioning.
///
/// The solve is warm-started from the initial value of x on entry.
///
/// @param matvec      Callable: y = A · v  (must be thread-safe if called in parallel)
/// @param prec_solve  Callable: z = P⁻¹ · v  (use identity λ if no preconditioner)
/// @param b           Right-hand side vector
/// @param x           In/out: initial guess → solution
/// @param restart     Krylov subspace size m (GMRES(m))
/// @param max_outer   Maximum number of outer restart cycles
/// @param tol         Convergence criterion: ‖r‖/‖b‖ < tol
/// @return            Iteration statistics
GmresStats gmres_solve(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& matvec,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& prec_solve,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    int restart,
    int max_outer,
    double tol);

// ─── Schur-complement block preconditioner ────────────────────────────────

/// Schur-complement AC/DC block preconditioner for the Newton Jacobian.
///
/// Exploits the block-triangular structure of the hybrid AC/DC Jacobian:
///
///   J = [ A   B ]    A: AC block  ( (np+nq) × (np+nq) )
///       [ 0   D ]    D: DC block  ( ndc_eq  × ndc_eq   )
///                    B: coupling  ( (np+nq) × ndc_eq   )
///
/// The exact Schur complement is S = D (because the lower-left block is 0),
/// so P is the block upper-triangular factorisation of J:
///
///   P = [ A   B ] = [ A  0 ] [ I  A⁻¹B ]
///       [ 0   D ]   [ 0  D ] [ 0  I    ]
///
/// Applying P⁻¹ r:
///   1.  z_DC = D⁻¹ r_DC
///   2.  z_AC = A⁻¹ (r_AC − B z_DC)
///
/// Both A and D are factored with Eigen::SparseLU.
class SchurBlockPreconditioner {
 public:
  /// Construct from the (optionally scaled) Jacobian and context layout.
  /// @return  true on success; false if any factorisation failed.
  bool build(const Eigen::SparseMatrix<double>& jacobian, const JacobianContext& ctx);

  /// Apply z = P⁻¹ r.  Falls back to identity if !is_valid().
  Eigen::VectorXd apply(const Eigen::VectorXd& r) const;

  bool is_valid() const noexcept { return valid_; }

 private:
  int n_ac_{0};  ///< np + nq
  int n_dc_{0};  ///< ndc_eq

  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_A_;  ///< Factor of AC block A
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu_D_;  ///< Factor of DC block D
  Eigen::SparseMatrix<double> B_;                      ///< AC-DC coupling block

  bool valid_{false};
};

// ─── Newton-Krylov linear step ────────────────────────────────────────────

/// Result of a Newton-Krylov step.
struct NKLinearResult {
  Eigen::VectorXd step;    ///< Newton step Δx such that J·Δx ≈ rhs
  GmresStats stats;        ///< GMRES convergence statistics
  bool success{false};     ///< True when GMRES converged (or residual is small enough)
  bool used_schur{false};  ///< True when Schur preconditioner was applied
};

/// Compute a Newton-Krylov step by solving J·Δx = rhs via GMRES.
///
/// @param jacobian            Sparse Jacobian J (may be scaled)
/// @param rhs                 Right-hand side (usually −F or scaled −F̂)
/// @param ctx                 Jacobian block-layout context
/// @param use_schur           Attempt Schur-complement preconditioning
/// @param gmres_restart       Krylov restart size m
/// @param gmres_max_outer     Maximum restart cycles
/// @param gmres_tol           Relative residual tolerance
NKLinearResult newton_krylov_step(const Eigen::SparseMatrix<double>& jacobian,
                                   const Eigen::VectorXd& rhs,
                                   const JacobianContext& ctx,
                                   bool use_schur,
                                   int gmres_restart,
                                   int gmres_max_outer,
                                   double gmres_tol);

}  // namespace hacdcpf::powerflow
