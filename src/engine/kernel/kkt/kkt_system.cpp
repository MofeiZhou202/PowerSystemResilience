#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>

namespace mipsolvers::engine {

bool factor_kkt_sparse(SparseKKTCache& cache,
                       const Eigen::SparseMatrix<double>& w,
                       const Eigen::SparseMatrix<double>& jg,
                       double reg) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  cache.n = n;
  cache.meq = meq;
  const int dim = n + meq;

  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(w.nonZeros() + 2 * jg.nonZeros() + dim));

  for (int col = 0; col < w.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, col); it; ++it) {
      tri.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < n; ++i) {
    tri.emplace_back(i, i, reg);
  }

  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      tri.emplace_back(it.col(), n + it.row(), it.value());
      tri.emplace_back(n + it.row(), it.col(), it.value());
    }
  }

  for (int i = 0; i < meq; ++i) {
    tri.emplace_back(n + i, n + i, -reg);
  }

  cache.kkt.resize(dim, dim);
  cache.kkt.setFromTriplets(tri.begin(), tri.end());
  cache.kkt.makeCompressed();
  cache.kkt_orig = cache.kkt;

  if (!cache.solver) {
    cache.solver = make_default_sparse_solver();
    cache.pattern_analyzed = false;
  }
  const bool pattern_changed = !cache.pattern_analyzed || cache.dim != dim ||
                               cache.nnz != cache.kkt.nonZeros();
  if (pattern_changed) {
    cache.solver->analyze_pattern(cache.kkt);
    cache.pattern_analyzed = true;
    cache.dim = dim;
    cache.nnz = cache.kkt.nonZeros();
  }

  if (!cache.solver->factorize(cache.kkt)) {
    cache.factored = false;
    return false;
  }
  cache.factored = true;
  return true;
}

bool solve_kkt_sparse(SparseKKTCache& cache,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& dx,
                      Eigen::VectorXd& dlambda) {
  if (!cache.factored) {
    return false;
  }
  Eigen::VectorXd sol;
  if (!cache.solver || !cache.solver->solve(rhs, sol) || !sol.allFinite()) {
    return false;
  }
  for (int ref = 0; ref < 2; ++ref) {
    Eigen::VectorXd residual = rhs - cache.kkt_orig * sol;
    if (residual.cwiseAbs().maxCoeff() < 1e-14 * std::max(1.0, rhs.cwiseAbs().maxCoeff())) {
      break;
    }
    Eigen::VectorXd correction;
    if (!cache.solver->solve(residual, correction) || !correction.allFinite()) {
      break;
    }
    sol += correction;
  }
  dx = sol.head(cache.n);
  dlambda = sol.tail(cache.meq);
  return true;
}

bool solve_kkt_reduced_sparse(const Eigen::SparseMatrix<double>& w,
                              const Eigen::SparseMatrix<double>& jg,
                              const Eigen::VectorXd& rhs,
                              Eigen::VectorXd& dx,
                              Eigen::VectorXd& dlambda,
                              int max_eq_dim,
                              double min_reg,
                              double max_reg) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  if (meq <= 0 || meq > max_eq_dim) {
    return false;
  }

  const Eigen::SparseMatrix<double> jgt = jg.transpose();
  for (double reg = min_reg; reg <= max_reg; reg *= 10.0) {
    Eigen::SparseMatrix<double> w_reg = w;
    w_reg.reserve(w.nonZeros() + n);
    for (int i = 0; i < n; ++i) {
      w_reg.coeffRef(i, i) += reg;
    }
    w_reg.makeCompressed();

    auto solver = make_default_sparse_solver();
    solver->analyze_pattern(w_reg);
    if (!solver->factorize(w_reg)) {
      continue;
    }

    const Eigen::VectorXd rhs_x = rhs.head(n);
    const Eigen::VectorXd rhs_eq = rhs.tail(meq);
    Eigen::VectorXd z_rhs;
    if (!solver->solve(rhs_x, z_rhs) || !z_rhs.allFinite()) {
      continue;
    }

    Eigen::MatrixXd z_cols = Eigen::MatrixXd::Zero(n, meq);
    bool column_failure = false;
    for (int i = 0; i < meq; ++i) {
      Eigen::VectorXd col_rhs = Eigen::VectorXd::Zero(n);
      for (Eigen::SparseMatrix<double>::InnerIterator it(jgt, i); it; ++it) {
        col_rhs[it.row()] = it.value();
      }
      Eigen::VectorXd z_col;
      if (!solver->solve(col_rhs, z_col) || !z_col.allFinite()) {
        column_failure = true;
        break;
      }
      z_cols.col(i) = z_col;
    }
    if (column_failure) {
      continue;
    }

    Eigen::MatrixXd schur = Eigen::MatrixXd(jg * z_cols);
    schur.diagonal().array() += reg;
    Eigen::LDLT<Eigen::MatrixXd> ldlt(schur);
    if (ldlt.info() != Eigen::Success) {
      continue;
    }

    dlambda = ldlt.solve(Eigen::VectorXd(jg * z_rhs) - rhs_eq);
    if (!dlambda.allFinite()) {
      continue;
    }
    dx = z_rhs - z_cols * dlambda;
    if (dx.allFinite()) {
      return true;
    }
  }
  return false;
}

namespace {

Eigen::SparseMatrix<double> add_scaled_identity(
    const Eigen::SparseMatrix<double>& a, double delta) {
  const int n = static_cast<int>(a.rows());
  Eigen::SparseMatrix<double> out = a;
  out.makeCompressed();
  if (delta == 0.0) return out;
  for (int i = 0; i < n; ++i) {
    out.coeffRef(i, i) += delta;
  }
  out.makeCompressed();
  return out;
}

// Materialize H_delta^{-1} * Jg^T one column at a time into a dense matrix.
// Returns false if any column solve fails numerically.
bool apply_inverse_to_jgt(
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower>& ldlt,
    const Eigen::SparseMatrix<double>& jg,
    Eigen::MatrixXd& z_cols) {
  const int n = static_cast<int>(jg.cols());
  const int meq = static_cast<int>(jg.rows());
  z_cols.setZero(n, meq);
  Eigen::SparseMatrix<double> jgt = jg.transpose();
  for (int i = 0; i < meq; ++i) {
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n);
    for (Eigen::SparseMatrix<double>::InnerIterator it(jgt, i); it; ++it) {
      rhs[it.row()] = it.value();
    }
    const Eigen::VectorXd col = ldlt.solve(rhs);
    if (!col.allFinite()) return false;
    z_cols.col(i) = col;
  }
  return true;
}

}  // namespace

bool factor_and_solve_kkt_inertia_corrected(
    const Eigen::SparseMatrix<double>& w,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::VectorXd& rhs,
    const InertiaSettings& settings,
    double& delta_w_last,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dlambda,
    InertiaStatus& status) {
  const int n = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  status = InertiaStatus{};
  if (rhs.size() != n + meq) return false;

  const Eigen::VectorXd rhs_x = rhs.head(n);
  const Eigen::VectorXd rhs_eq = (meq > 0) ? Eigen::VectorXd(rhs.tail(meq)) : Eigen::VectorXd();

  // Wächter-Biegler δ_W schedule: start from last successful δ_W; on failure,
  // if δ_W was zero → jump to δ_w_0; else multiply by κ_W⁺_first (first
  // repair) or κ_W⁺ (subsequent). δ_C stays at 0 unless we later detect Jg
  // rank deficiency.
  double delta_w = std::max(0.0, delta_w_last);
  double delta_c = 0.0;
  bool delta_w_was_zero = (delta_w == 0.0);

  for (; status.factorization_attempts < 60; ++status.factorization_attempts) {
    // Primal SPD test: factor H + δ_W I.
    Eigen::SparseMatrix<double> h_delta = add_scaled_identity(w, delta_w);
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>, Eigen::Lower> primal_ldlt;
    primal_ldlt.compute(h_delta);
    const bool primal_ok = (primal_ldlt.info() == Eigen::Success);

    // Additional guard: SimplicialLDLT does not pivot, so a "success" status
    // on a matrix with a tiny negative pivot is possible for numerically
    // borderline inputs. Verify by scanning D.
    bool primal_has_negative = false;
    if (primal_ok) {
      const Eigen::VectorXd d = primal_ldlt.vectorD();
      for (int i = 0; i < d.size(); ++i) {
        if (!(d[i] > 0.0)) { primal_has_negative = true; break; }
      }
    }

    if (!primal_ok || primal_has_negative) {
      if (delta_w_was_zero) {
        delta_w = settings.delta_w_0;
        delta_w_was_zero = false;
      } else {
        const double mult = (delta_w < settings.delta_w_0 * 1.1)
                                ? settings.kappa_w_plus_first
                                : settings.kappa_w_plus;
        delta_w = std::max(delta_w * mult, settings.delta_w_0);
      }
      if (delta_w > settings.delta_w_max) {
        status.delta_w_used = delta_w;
        status.delta_c_used = delta_c;
        return false;
      }
      continue;
    }

    status.n_pos = n;

    if (meq == 0) {
      // Pure unconstrained primal block.
      dx = primal_ldlt.solve(rhs_x);
      dlambda.resize(0);
      if (!dx.allFinite()) {
        // Treat as indefinite and regularize more.
        if (delta_w_was_zero) {
          delta_w = settings.delta_w_0;
          delta_w_was_zero = false;
        } else {
          delta_w = std::max(delta_w * settings.kappa_w_plus,
                             settings.delta_w_0);
        }
        if (delta_w > settings.delta_w_max) return false;
        continue;
      }
      status.correct = true;
      status.delta_w_used = delta_w;
      status.delta_c_used = 0.0;
      delta_w_last = std::max(settings.delta_w_min, delta_w * settings.kappa_w_minus);
      return true;
    }

    // Schur complement S = Jg H_δ⁻¹ Jgᵀ + δ_C I.
    Eigen::MatrixXd z_cols;
    if (!apply_inverse_to_jgt(primal_ldlt, jg, z_cols)) {
      delta_w = std::max(delta_w * settings.kappa_w_plus, settings.delta_w_0);
      if (delta_w > settings.delta_w_max) return false;
      continue;
    }

    Eigen::MatrixXd schur_dense = jg * z_cols;
    // Symmetrize numerically.
    schur_dense = 0.5 * (schur_dense + schur_dense.transpose());
    if (delta_c > 0.0) {
      schur_dense.diagonal().array() += delta_c;
    }

    Eigen::LDLT<Eigen::MatrixXd> schur_ldlt(schur_dense);
    bool schur_ok = (schur_ldlt.info() == Eigen::Success);
    bool schur_all_positive = false;
    if (schur_ok) {
      const Eigen::VectorXd d = schur_ldlt.vectorD();
      schur_all_positive = true;
      for (int i = 0; i < d.size(); ++i) {
        // S should be positive definite (its eigenvalues equal the magnitudes
        // of the negative eigenvalues of the block-diagonal congruent KKT);
        // any non-positive pivot means Jg is rank-deficient at tolerance.
        if (!(d[i] > 0.0)) { schur_all_positive = false; break; }
      }
    }

    if (!schur_ok || !schur_all_positive) {
      // Dual regularization — Jg is rank-deficient. δ_C = stripe · μ^{1/4}.
      delta_c = settings.delta_c_stripe *
                std::pow(std::max(settings.mu, 1e-20), 0.25);
      // Retry with the same δ_W but now positive δ_C.
      Eigen::MatrixXd schur_reg = schur_dense;
      schur_reg.diagonal().array() += delta_c;
      schur_ldlt.compute(schur_reg);
      if (schur_ldlt.info() != Eigen::Success) {
        // Cannot fix — bump δ_W and restart from scratch.
        if (delta_w_was_zero) {
          delta_w = settings.delta_w_0;
          delta_w_was_zero = false;
        } else {
          delta_w = std::max(delta_w * settings.kappa_w_plus,
                             settings.delta_w_0);
        }
        if (delta_w > settings.delta_w_max) return false;
        continue;
      }
      status.n_zero = meq;  // diagnostic
    }
    status.n_neg = meq;

    // Block solve: dλ = S⁻¹ (Jg H_δ⁻¹ rhs_x − rhs_eq)
    //              dx  = H_δ⁻¹ (rhs_x − Jgᵀ dλ)
    const Eigen::VectorXd v = primal_ldlt.solve(rhs_x);
    if (!v.allFinite()) {
      delta_w = std::max(delta_w * settings.kappa_w_plus, settings.delta_w_0);
      if (delta_w > settings.delta_w_max) return false;
      continue;
    }
    const Eigen::VectorXd rhs_schur = Eigen::VectorXd(jg * v) - rhs_eq;
    dlambda = schur_ldlt.solve(rhs_schur);
    if (!dlambda.allFinite()) {
      delta_w = std::max(delta_w * settings.kappa_w_plus, settings.delta_w_0);
      if (delta_w > settings.delta_w_max) return false;
      continue;
    }
    const Eigen::VectorXd corrected_rhs_x =
        rhs_x - Eigen::VectorXd(jg.transpose() * dlambda);
    dx = primal_ldlt.solve(corrected_rhs_x);
    if (!dx.allFinite()) {
      delta_w = std::max(delta_w * settings.kappa_w_plus, settings.delta_w_0);
      if (delta_w > settings.delta_w_max) return false;
      continue;
    }

    status.correct = true;
    status.delta_w_used = delta_w;
    status.delta_c_used = delta_c;
    delta_w_last = std::max(settings.delta_w_min, delta_w * settings.kappa_w_minus);
    return true;
  }
  return false;
}

Eigen::SparseMatrix<double> assemble_primal_hessian_with_bounds(
    const Eigen::SparseMatrix<double>& hess,
    const Eigen::VectorXd& sigma_diag) {
  const int n = static_cast<int>(hess.rows());
  if (sigma_diag.size() != n) return hess;
  Eigen::SparseMatrix<double> out = hess;
  out.makeCompressed();
  for (int i = 0; i < n; ++i) {
    const double sigma_i = sigma_diag[i];
    if (sigma_i != 0.0) {
      out.coeffRef(i, i) += sigma_i;
    }
  }
  out.makeCompressed();
  return out;
}

void split_inequalities(const NLPModel& prob,
                        const Eigen::VectorXd& x,
                        const std::vector<int>& lb_cols,
                        const std::vector<int>& ub_cols,
                        Eigen::VectorXd& h,
                        Eigen::SparseMatrix<double>& jh) {
  const int n = static_cast<int>(prob.vars.size());

  Eigen::VectorXd h_nonlin;
  Eigen::SparseMatrix<double> jh_nonlin;
  if (prob.h) {
    prob.h(x, h_nonlin);
  } else {
    h_nonlin = Eigen::VectorXd::Zero(0);
  }

  const int m_nonlinear = static_cast<int>(h_nonlin.size());

  if (prob.jac_h && m_nonlinear > 0) {
    prob.jac_h(x, jh_nonlin);
  } else {
    jh_nonlin.resize(m_nonlinear, n);
    jh_nonlin.setZero();
  }

  const int m = m_nonlinear + static_cast<int>(lb_cols.size()) + static_cast<int>(ub_cols.size());
  h = Eigen::VectorXd::Zero(m);

  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(jh_nonlin.nonZeros() + lb_cols.size() + ub_cols.size()));

  if (m_nonlinear > 0) {
    h.head(m_nonlinear) = h_nonlin;
    for (int col = 0; col < jh_nonlin.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jh_nonlin, col); it; ++it) {
        tri.emplace_back(it.row(), it.col(), it.value());
      }
    }
  }

  int row = m_nonlinear;
  for (int c : lb_cols) {
    h[row] = prob.vars[c].lb - x[c];
    tri.emplace_back(row, c, -1.0);
    ++row;
  }
  for (int c : ub_cols) {
    h[row] = x[c] - prob.vars[c].ub;
    tri.emplace_back(row, c, 1.0);
    ++row;
  }

  jh.resize(m, n);
  jh.setFromTriplets(tri.begin(), tri.end());
  jh.makeCompressed();
}

}  // namespace mipsolvers::engine
