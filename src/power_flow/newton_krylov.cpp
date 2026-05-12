/// @file newton_krylov.cpp
/// @brief Phase 5: Newton-Krylov (GMRES) inner solver with Schur-complement
///        AC/DC block preconditioner.
///
/// Implements restarted GMRES(m) with Modified Gram-Schmidt (MGS) Arnoldi
/// and Givens rotation least-squares update, plus a Schur-complement
/// preconditioner that factors the AC and DC blocks of the Jacobian
/// separately to reduce per-iteration cost when ndc_eq << np+nq.

#include "hacdcpf/power_flow/newton_krylov.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::powerflow {

// ─── GMRES(m) ─────────────────────────────────────────────────────────────

GmresStats gmres_solve(
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& matvec,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& prec_solve,
    const Eigen::VectorXd& b,
    Eigen::VectorXd& x,
    int restart,
    int max_outer,
    double tol) {
  GmresStats stats;
  const int n = static_cast<int>(b.size());
  if (n == 0) {
    stats.converged = true;
    stats.final_relres = 0.0;
    return stats;
  }

  // Cap restart to problem size.
  restart = std::min(restart, n);
  restart = std::max(restart, 1);
  max_outer = std::max(max_outer, 1);

  const double norm_b = b.norm();
  if (norm_b < 1e-300) {
    x.setZero();
    stats.converged = true;
    stats.final_relres = 0.0;
    return stats;
  }

  // Dense Krylov basis: column j is v_{j+1}.
  Eigen::MatrixXd V(n, restart + 1);
  // Upper Hessenberg matrix H ∈ ℝ^{(m+1)×m}.
  Eigen::MatrixXd H(restart + 1, restart);
  // Givens rotation coefficients.
  Eigen::VectorXd cs(restart);
  Eigen::VectorXd sn(restart);
  // Transformed right-hand side e₁·β.
  Eigen::VectorXd g(restart + 1);

  for (int outer = 0; outer < max_outer; ++outer) {
    // r = P⁻¹·(b − A·x)
    Eigen::VectorXd r = prec_solve(b - matvec(x));

    const double beta = r.norm();
    if (beta / norm_b < tol) {
      stats.converged = true;
      stats.final_relres = beta / norm_b;
      return stats;
    }

    V.col(0) = r / beta;
    g.setZero();
    g[0] = beta;
    H.setZero();

    int m_eff = 0;  // effective Arnoldi steps completed this outer cycle
    for (int j = 0; j < restart; ++j) {
      ++m_eff;
      // w = P⁻¹ · A · v_j
      Eigen::VectorXd w = prec_solve(matvec(V.col(j)));
      ++stats.iterations;

      // Modified Gram-Schmidt orthogonalisation.
      for (int i = 0; i <= j; ++i) {
        H(i, j) = w.dot(V.col(i));
        w -= H(i, j) * V.col(i);
      }
      H(j + 1, j) = w.norm();

      if (H(j + 1, j) > 1e-300) {
        V.col(j + 1) = w / H(j + 1, j);
      }
      // (If H(j+1,j) ≈ 0, the solution is already exact in the Krylov space.)

      // Apply previously computed Givens rotations to new column of H.
      for (int i = 0; i < j; ++i) {
        const double tmp = cs[i] * H(i, j) + sn[i] * H(i + 1, j);
        H(i + 1, j) = -sn[i] * H(i, j) + cs[i] * H(i + 1, j);
        H(i, j) = tmp;
      }

      // Compute and apply new Givens rotation G_j to eliminate H(j+1, j).
      const double denom = std::hypot(H(j, j), H(j + 1, j));
      if (denom < 1e-300) {
        break;
      }
      cs[j] = H(j, j) / denom;
      sn[j] = H(j + 1, j) / denom;
      H(j, j) = denom;
      H(j + 1, j) = 0.0;

      // Update g: g_{j+1} = -sn_j · g_j,  g_j = cs_j · g_j.
      g[j + 1] = -sn[j] * g[j];
      g[j] = cs[j] * g[j];

      const double relres = std::abs(g[j + 1]) / norm_b;
      if (relres < tol) {
        break;
      }
    }

    // Solve upper-triangular system H[0:m_eff, 0:m_eff] · y = g[0:m_eff].
    Eigen::VectorXd y =
        H.topLeftCorner(m_eff, m_eff).triangularView<Eigen::Upper>().solve(g.head(m_eff));

    // Update solution: x += V[:,0:m_eff] · y.
    x += V.leftCols(m_eff) * y;

    // Recompute true relative residual (after update, not just estimate).
    const double true_relres = (b - matvec(x)).norm() / norm_b;
    stats.final_relres = true_relres;
    if (true_relres < tol) {
      stats.converged = true;
      return stats;
    }
  }

  // Final residual after all restarts.
  stats.final_relres = (b - matvec(x)).norm() / norm_b;
  stats.converged = (stats.final_relres < tol);
  return stats;
}

// ─── SchurBlockPreconditioner ─────────────────────────────────────────────

bool SchurBlockPreconditioner::build(const Eigen::SparseMatrix<double>& jacobian,
                                     const JacobianContext& ctx) {
  valid_ = false;
  n_ac_ = ctx.np + ctx.nq;
  n_dc_ = ctx.ndc_eq;

  // Trivial (no variables) — treat as identity.
  if (n_ac_ == 0 && n_dc_ == 0) {
    valid_ = true;
    return true;
  }

  // Factor AC block A = J[0:n_ac_, 0:n_ac_].
  if (n_ac_ > 0) {
    Eigen::SparseMatrix<double> A = jacobian.topLeftCorner(n_ac_, n_ac_);
    lu_A_.analyzePattern(A);
    lu_A_.factorize(A);
    if (lu_A_.info() != Eigen::Success) {
      return false;
    }
  }

  // Factor DC block D = J[n_ac_:, n_ac_:].
  if (n_dc_ > 0) {
    Eigen::SparseMatrix<double> D = jacobian.bottomRightCorner(n_dc_, n_dc_);
    lu_D_.analyzePattern(D);
    lu_D_.factorize(D);
    if (lu_D_.info() != Eigen::Success) {
      return false;
    }
  }

  // Extract coupling block B = J[0:n_ac_, n_ac_:].
  if (n_ac_ > 0 && n_dc_ > 0) {
    B_ = jacobian.block(0, n_ac_, n_ac_, n_dc_);
  }

  valid_ = true;
  return true;
}

Eigen::VectorXd SchurBlockPreconditioner::apply(const Eigen::VectorXd& r) const {
  if (!valid_) {
    return r;  // Identity fallback.
  }

  Eigen::VectorXd z(r.size());

  if (n_dc_ > 0 && n_ac_ > 0) {
    // Step 1: z_DC = D⁻¹ · r_DC
    const Eigen::VectorXd z_dc = lu_D_.solve(r.tail(n_dc_));
    // Step 2: z_AC = A⁻¹ · (r_AC − B · z_DC)
    const Eigen::VectorXd z_ac = lu_A_.solve(r.head(n_ac_) - B_ * z_dc);
    z.head(n_ac_) = z_ac;
    z.tail(n_dc_) = z_dc;
  } else if (n_ac_ > 0) {
    z = lu_A_.solve(r);
  } else {
    // n_dc_ > 0 only.
    z = lu_D_.solve(r);
  }

  return z;
}

// ─── newton_krylov_step ───────────────────────────────────────────────────

NKLinearResult newton_krylov_step(const Eigen::SparseMatrix<double>& jacobian,
                                   const Eigen::VectorXd& rhs,
                                   const JacobianContext& ctx,
                                   bool use_schur,
                                   int gmres_restart,
                                   int gmres_max_outer,
                                   double gmres_tol) {
  NKLinearResult result;
  result.step = Eigen::VectorXd::Zero(rhs.size());

  // Matrix-vector product: y = J · v.
  auto matvec = [&](const Eigen::VectorXd& v) -> Eigen::VectorXd {
    return jacobian * v;
  };

  // Build preconditioner.
  SchurBlockPreconditioner schur_prec;
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> prec_solve;

  if (use_schur && schur_prec.build(jacobian, ctx)) {
    result.used_schur = true;
    prec_solve = [&](const Eigen::VectorXd& v) -> Eigen::VectorXd {
      return schur_prec.apply(v);
    };
  } else {
    // Identity preconditioner — plain GMRES without preconditioning.
    prec_solve = [](const Eigen::VectorXd& v) -> Eigen::VectorXd { return v; };
  }

  // Solve J · Δx = rhs.
  result.stats = gmres_solve(matvec, prec_solve, rhs, result.step,
                              gmres_restart, gmres_max_outer, gmres_tol);

  // Accept if converged or residual is within a loose tolerance.
  result.success =
      result.stats.converged ||
      (result.stats.final_relres < gmres_tol * 100.0 &&
       result.step.allFinite() &&
       result.step.cwiseAbs().maxCoeff() < 1e6);

  return result;
}

}  // namespace hacdcpf::powerflow
