#include "hacdcpf/optimal_power_flow/native_ipm_solver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/detail/logging.hpp"

namespace hacdcpf::opf::parity {

namespace {

double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

// Fraction-to-boundary rule: max alpha s.t. v + alpha*dv > 0
double ftb(const Eigen::VectorXd& v, const Eigen::VectorXd& dv, double tau) {
  double alpha = 1.0;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (dv[i] < 0.0) {
      const double candidate = -tau * v[i] / dv[i];
      if (candidate < alpha) {
        alpha = candidate;
      }
    }
  }
  return alpha;
}

// Cache for the sparse KKT system, including the SparseLU factorization.
struct SparseKKTCache {
  Eigen::SparseMatrix<double> kkt;
  Eigen::SparseMatrix<double> kkt_orig;
  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu;
  bool factored{false};
  int nn{0};
  int meq{0};
};

bool factor_kkt_sparse(SparseKKTCache& cache,
                       const Eigen::SparseMatrix<double>& w,
                       const Eigen::SparseMatrix<double>& jg,
                       double reg) {
  const int nn = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  cache.nn = nn;
  cache.meq = meq;
  const int dim = nn + meq;

  // Assemble KKT as sparse matrix
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<size_t>(w.nonZeros() + 2 * jg.nonZeros() + dim));

  // (1,1) block: W + δI
  for (int col = 0; col < w.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, col); it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < nn; ++i) {
    trips.emplace_back(i, i, reg);
  }

  // (1,2) block: Jg'  and (2,1) block: Jg
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      trips.emplace_back(it.col(), nn + it.row(), it.value());
      trips.emplace_back(nn + it.row(), it.col(), it.value());
    }
  }

  // (2,2) block: -δI
  for (int i = 0; i < meq; ++i) {
    trips.emplace_back(nn + i, nn + i, -reg);
  }

  cache.kkt.resize(dim, dim);
  cache.kkt.setFromTriplets(trips.begin(), trips.end());
  cache.kkt.makeCompressed();
  cache.kkt_orig = cache.kkt;

  // Always use full compute() — symbolic reuse via analyzePattern/factorize
  // is not reliably profitable with progressive regularization.
  cache.lu.compute(cache.kkt);
  if (cache.lu.info() != Eigen::Success) {
    cache.factored = false;
    return false;
  }
  cache.factored = true;
  return true;
}

bool kkt_solve_sparse(SparseKKTCache& cache,
                      const Eigen::VectorXd& rhs,
                      Eigen::VectorXd& dx,
                      Eigen::VectorXd& dlambda) {
  if (!cache.factored) {
    return false;
  }
  Eigen::VectorXd sol = cache.lu.solve(rhs);
  if (!sol.allFinite()) {
    return false;
  }
  // Iterative refinement (2 steps)
  for (int ref = 0; ref < 2; ++ref) {
    Eigen::VectorXd residual = rhs - cache.kkt_orig * sol;
    if (residual.cwiseAbs().maxCoeff() < 1e-14 * rhs.cwiseAbs().maxCoeff()) {
      break;
    }
    Eigen::VectorXd correction = cache.lu.solve(residual);
    if (!correction.allFinite()) {
      break;
    }
    sol += correction;
  }
  dx = sol.head(cache.nn);
  dlambda = sol.tail(cache.meq);
  return true;
}

// Dense KKT cache (for small systems where dense is competitive)
struct DenseKKTCache {
  Eigen::MatrixXd buf;
  Eigen::MatrixXd buf_orig;  // copy for iterative refinement
  Eigen::PartialPivLU<Eigen::MatrixXd> lu;
  bool factored{false};
  int nn{0};
  int meq{0};
};

// Fill KKT = [W+δI, Jg'; Jg, -δI] into dense buffer and factor with pivoted LU
bool factor_kkt_dense(DenseKKTCache& cache,
                      const Eigen::SparseMatrix<double>& w,
                      const Eigen::SparseMatrix<double>& jg,
                      double reg) {
  const int nn = static_cast<int>(w.rows());
  const int meq = static_cast<int>(jg.rows());
  cache.nn = nn;
  cache.meq = meq;
  const int dim = nn + meq;

  cache.buf.setZero(dim, dim);

  // (1,1) block: W + δI
  for (int col = 0; col < w.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(w, col); it; ++it) {
      cache.buf(it.row(), it.col()) += it.value();
    }
  }
  for (int i = 0; i < nn; ++i) {
    cache.buf(i, i) += reg;
  }

  // (1,2) and (2,1) blocks: Jg' and Jg
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      cache.buf(it.col(), nn + it.row()) = it.value();  // Jg' (top-right)
      cache.buf(nn + it.row(), it.col()) = it.value();  // Jg  (bottom-left)
    }
  }

  // (2,2) block: -δI
  for (int i = 0; i < meq; ++i) {
    cache.buf(nn + i, nn + i) = -reg;
  }

  cache.buf_orig = cache.buf;  // save for iterative refinement
  cache.lu.compute(cache.buf);
  cache.factored = true;
  return true;
}

bool kkt_solve_dense(DenseKKTCache& cache,
                     const Eigen::VectorXd& rhs,
                     Eigen::VectorXd& dx,
                     Eigen::VectorXd& dlambda) {
  if (!cache.factored) {
    return false;
  }
  Eigen::VectorXd sol = cache.lu.solve(rhs);
  if (!sol.allFinite()) {
    return false;
  }
  // Iterative refinement: 2 steps to combat ill-conditioning from extreme σ ratios
  for (int ref = 0; ref < 2; ++ref) {
    Eigen::VectorXd residual = rhs - cache.buf_orig * sol;
    if (residual.cwiseAbs().maxCoeff() < 1e-14 * rhs.cwiseAbs().maxCoeff()) {
      break;
    }
    Eigen::VectorXd correction = cache.lu.solve(residual);
    if (!correction.allFinite()) {
      break;
    }
    sol += correction;
  }
  dx = sol.head(cache.nn);
  dlambda = sol.tail(cache.meq);
  return true;
}

void assemble_inequalities(const Problem& prob,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& xmin,
                           const Eigen::VectorXd& xmax,
                           const std::vector<int>& lb_cols,
                           const std::vector<int>& ub_cols,
                           Eigen::VectorXd& h,
                           Eigen::SparseMatrix<double>& jh) {
  Eigen::VectorXd h_nonlin;
  Eigen::SparseMatrix<double> jh_nonlin;
  nonlinear_inequality_constraints(prob, x, h_nonlin);
  nonlinear_inequality_jacobian(prob, x, jh_nonlin);

  const int nn = prob.vidx.n_total;
  const int m_nonlin = prob.cidx.n_ineq_nonlin;
  const int m = m_nonlin + static_cast<int>(lb_cols.size()) + static_cast<int>(ub_cols.size());

  h = Eigen::VectorXd::Zero(m);
  std::vector<Eigen::Triplet<double>> t;
  t.reserve(static_cast<size_t>(jh_nonlin.nonZeros() + lb_cols.size() + ub_cols.size()));

  if (m_nonlin > 0) {
    h.head(m_nonlin) = h_nonlin;
    for (int col = 0; col < jh_nonlin.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jh_nonlin, col); it; ++it) {
        t.emplace_back(it.row(), it.col(), it.value());
      }
    }
  }

  int row = m_nonlin;
  for (int c : lb_cols) {
    h[row] = xmin[c] - x[c];
    t.emplace_back(row, c, -1.0);
    ++row;
  }
  for (int c : ub_cols) {
    h[row] = x[c] - xmax[c];
    t.emplace_back(row, c, 1.0);
    ++row;
  }

  jh.resize(m, nn);
  jh.setFromTriplets(t.begin(), t.end());
  jh.makeCompressed();
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// Mehrotra Predictor-Corrector IPM — faithful port of Julia InteriorPointMethod.jl
// ═══════════════════════════════════════════════════════════════════════════════

IPMResult solve_primal_dual_ipm(const Problem& prob, const IPMOptions& opt) {
  const int n = prob.vidx.n_total;
  const int meq = prob.cidx.n_eq_total;
  if (n <= 0 || meq < 0) {
    throw std::runtime_error("solve_primal_dual_ipm: invalid problem size.");
  }

  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  Eigen::VectorXd x;
  build_variable_bounds(prob, xmin, xmax);
  build_initial_point(prob, xmin, xmax, x);

  // Identify finite-bound variable indices
  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  lb_cols.reserve(static_cast<size_t>(n));
  ub_cols.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    if (std::isfinite(xmin[i])) {
      lb_cols.push_back(i);
    }
    if (std::isfinite(xmax[i])) {
      ub_cols.push_back(i);
    }
  }

  // Evaluate initial inequality constraints
  Eigen::VectorXd rh;
  Eigen::SparseMatrix<double> dh;
  assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
  const int niq = static_cast<int>(rh.size());
  if (niq <= 0) {
    throw std::runtime_error("solve_primal_dual_ipm: no inequality constraints assembled.");
  }

  // Slack initialization: use a larger minimum floor (1.0) to ensure
  // well-conditioned condensed KKT from the start.
  // This creates initial artificial infeasibility (h + z ≠ 0) but gives
  // max(σ)/min(σ) ≈ 100 instead of 10^6, enabling accurate Newton steps.
  Eigen::VectorXd z(niq);
  for (int i = 0; i < niq; ++i) {
    z[i] = (rh[i] >= 0.0) ? std::max(rh[i], 1.0) : std::max(-rh[i], 1e-2);
  }

  // Multiplier initialization: μ_i ≈ 1/z_i to maintain μ·z ≈ 1
  Eigen::VectorXd mu(niq);
  for (int i = 0; i < niq; ++i) {
    mu[i] = std::max(1.0 / z[i], 1e-2);
  }

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(meq);

  const double tau = opt.alpha_max;  // fraction-to-boundary factor

  // Evaluate at starting point
  EvalWorkspace eq_ws;
  Eigen::VectorXd grad;
  Eigen::VectorXd hdiag;
  Eigen::VectorXd rg;
  Eigen::SparseMatrix<double> jg;
  objective_gradient_hessian_diag(prob, x, grad, hdiag);
  equality_constraints(prob, x, eq_ws, rg);
  equality_jacobian(prob, x, eq_ws, jg);

  // Lagrangian gradient: Lx = ∇f + Jg'·λ + Jh'·μ
  Eigen::VectorXd Lx = grad + jg.transpose() * lambda + dh.transpose() * mu;

  // Lagrangian Hessian
  const int m_nonlin = prob.cidx.n_ineq_nonlin;
  auto get_nu_ptr = [&](const Eigen::VectorXd& mu_vec) -> const Eigen::VectorXd* {
    static thread_local Eigen::VectorXd nu_buf;
    if (m_nonlin > 0) {
      nu_buf = mu_vec.head(m_nonlin);
      return &nu_buf;
    }
    return nullptr;
  };

  Eigen::SparseMatrix<double> Lxx;
  lagrangian_hessian(prob, x, lambda, get_nu_ptr(mu), Lxx, 1e-12);

  // Convergence measures (normalized, matching Julia)
  double obj = objective(prob, x);
  const double obj0 = obj;  // reference for cost condition (NOT updated each iter)

  auto compute_convergence = [&](const Eigen::VectorXd& x_v, const Eigen::VectorXd& z_v,
                                 const Eigen::VectorXd& lambda_v, const Eigen::VectorXd& mu_v,
                                 const Eigen::VectorXd& rg_v, const Eigen::VectorXd& rh_v,
                                 const Eigen::VectorXd& Lx_v, double obj_v) {
    const double maxh = (niq > 0) ? std::max(rh_v.maxCoeff(), 0.0) : 0.0;
    const double scale_x = std::max(inf_norm(x_v), inf_norm(z_v));
    const double feascond = std::max(inf_norm(rg_v), maxh) / (1.0 + scale_x);
    const double gradcond = inf_norm(Lx_v) / (1.0 + std::max(inf_norm(mu_v), inf_norm(lambda_v)));
    // Average complementarity per constraint — normalizes by niq to be problem-size-independent.
    const double compcond = (niq > 0) ? z_v.dot(mu_v) / static_cast<double>(niq) / (1.0 + inf_norm(x_v)) : 0.0;
    const double costcond = std::abs(obj_v - obj0) / (1.0 + std::abs(obj0));
    return std::make_tuple(feascond, gradcond, compcond, costcond);
  };

  auto [feascond, gradcond, compcond, costcond] =
      compute_convergence(x, z, lambda, mu, rg, rh, Lx, obj);

  const double feas_tol = opt.tol_primal;
  // Complementarity tolerance: for large problems the average z·μ/niq can stall
  // at ~1e-3 due to barrier ill-conditioning, while feas and grad are fully converged.
  // Use a relaxed tolerance for complementarity (minimum 2e-3).
  const double comp_tol = std::max(feas_tol * 10.0, 2e-3);
  bool converged = (feascond < feas_tol && gradcond < feas_tol && compcond < comp_tol);

  IPMResult out;
  out.status = "maximum iterations reached";

  // Best-iterate tracking: store the iterate with the smallest max(feas, grad, comp).
  // For oscillating problems on constraint boundaries, the best iterate may satisfy
  // convergence even if the final iterate does not.
  double best_metric = std::max({feascond, gradcond, compcond});
  Eigen::VectorXd best_x = x;
  Eigen::VectorXd best_lambda = lambda;
  Eigen::VectorXd best_mu = mu;
  Eigen::VectorXd best_z = z;
  double best_feas = feascond;
  double best_grad = gradcond;
  double best_comp = compcond;
  int best_iter = 0;

  // Use dense KKT for small systems (fast pivoted LU), sparse for larger ones
  constexpr int kDenseThreshold = 1500;  // dim = n + meq
  const bool use_dense = (n + meq) <= kDenseThreshold;
  DenseKKTCache dense_cache;
  SparseKKTCache sparse_cache;

  // Unified factor/solve lambdas
  auto factor_kkt = [&](const Eigen::SparseMatrix<double>& w,
                        const Eigen::SparseMatrix<double>& jg_mat,
                        double reg) -> bool {
    if (use_dense) {
      return factor_kkt_dense(dense_cache, w, jg_mat, reg);
    }
    return factor_kkt_sparse(sparse_cache, w, jg_mat, reg);
  };
  auto solve_kkt = [&](const Eigen::VectorXd& rhs,
                       Eigen::VectorXd& dx_out,
                       Eigen::VectorXd& dlambda_out) -> bool {
    if (use_dense) {
      return kkt_solve_dense(dense_cache, rhs, dx_out, dlambda_out);
    }
    return kkt_solve_sparse(sparse_cache, rhs, dx_out, dlambda_out);
  };

  for (int iter = 0; iter < opt.max_iter && !converged; ++iter) {
    out.iterations = iter;
    out.primal_inf = feascond;
    out.dual_inf = gradcond;
    out.complementarity = compcond;

    if (opt.verbose) {
      [[maybe_unused]] const double maxh_dbg = (niq > 0) ? std::max(rh.maxCoeff(), 0.0) : 0.0;
      HACDCPF_LOG_DEBUG("[parity-ipm] iter={} feas={} grad={} comp={} cost={} |rg|={} maxh={} |Lx|={}",
                iter, feascond, gradcond, compcond, costcond,
                inf_norm(rg), maxh_dbg, inf_norm(Lx));
    }

    // Safeguard z, μ — use 1e-12 floor to prevent extreme σ = μ/z ratios
    const Eigen::VectorXd z_safe = z.cwiseMax(1e-12);
    const Eigen::VectorXd mu_safe = mu.cwiseMax(1e-12);
    const Eigen::VectorXd sigma_s = mu_safe.cwiseQuotient(z_safe);  // Σ = diag(μ/z)

    // Build condensed reduced Hessian: W = Lxx + Jh' · diag(μ/z) · Jh
    Eigen::SparseMatrix<double> dh_scaled = dh;
    for (int col = 0; col < dh_scaled.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(dh_scaled, col); it; ++it) {
        it.valueRef() *= sigma_s[it.row()];
      }
    }
    Eigen::SparseMatrix<double> W = Lxx + dh.transpose() * dh_scaled;
    W.makeCompressed();

    // Equality Jacobian in the format needed for KKT: jg is meq × n
    // dg_for_kkt = Jg transposed for KKT fill: we pass jg directly (meq × n)
    // In Julia: dg is n × neq, we pass it as-is. Our jg is meq × n.
    // The KKT uses Jg (rows=constraints, cols=vars), matching our jg.

    // ────────────────────────────────────────────────────────────────
    // STEP 1: Affine (predictor) step with γ = 0
    // ────────────────────────────────────────────────────────────────
    // N_aff = Lx + Jh' · (μ .* rh ./ z)    [condensed RHS, γ=0]
    const Eigen::VectorXd N_aff = Lx + dh.transpose() * (mu_safe.cwiseProduct(rh).cwiseQuotient(z_safe));
    const Eigen::VectorXd rhs_eq = -rg;

    Eigen::VectorXd dx_aff;
    Eigen::VectorXd dlambda_aff;

    // Progressive regularization
    bool factor_ok = false;
    const double W_norm = std::max(1.0, W.coeffs().cwiseAbs().maxCoeff());
    constexpr std::array<double, 4> kRegLevels{0.0, 1.0, 1e-6, 1e-4};
    constexpr double kDeltaMin = 1e-8;
    for (double level : kRegLevels) {
      const double reg = (level == 0.0) ? 0.0 :
                         (level == 1.0) ? kDeltaMin * W_norm : level * W_norm;
      const double reg_eff = std::max(reg, 1e-12);
      if (!factor_kkt(W, jg, reg_eff)) {
        continue;
      }
      Eigen::VectorXd rhs_aff(n + meq);
      rhs_aff.head(n) = -N_aff;
      rhs_aff.tail(meq) = rhs_eq;
      if (!solve_kkt(rhs_aff, dx_aff, dlambda_aff)) {
        continue;
      }
      factor_ok = true;
      break;
    }
    if (!factor_ok) {
      out.status = "KKT factorization failed";
      break;
    }

    // Recover affine Δz, Δμ
    const Eigen::VectorXd jh_dx_aff = dh * dx_aff;
    Eigen::VectorXd dz_aff = -(rh + z_safe) - jh_dx_aff;
    Eigen::VectorXd dmu_aff = -(mu_safe + sigma_s.cwiseProduct(dz_aff));

    // Affine step lengths (probe with τ = 1.0)
    double alpha_p_aff = std::min(ftb(z_safe, dz_aff, 1.0), 1.0);
    double alpha_d_aff = std::min(ftb(mu_safe, dmu_aff, 1.0), 1.0);

    // Adaptive centering parameter σ = (μ_aff / μ_cur)³
    const double mu_cur = z_safe.dot(mu_safe) / static_cast<double>(niq);
    const double mu_aff_val =
        (z_safe + alpha_p_aff * dz_aff).dot(mu_safe + alpha_d_aff * dmu_aff)
        / static_cast<double>(niq);
    const double ratio = std::clamp(mu_aff_val / (mu_cur + 1e-16), 0.0, 1.0);
    const double sigma = ratio * ratio * ratio;
    double gamma = std::clamp(sigma * mu_cur, 1e-14, 1e4);

    // ────────────────────────────────────────────────────────────────
    // STEP 2: Corrector step with Mehrotra second-order centering
    // ────────────────────────────────────────────────────────────────
    // cross = clamp(Δμ_aff ⊙ Δz_aff, ±0.1γ)
    Eigen::VectorXd cross = dmu_aff.cwiseProduct(dz_aff);
    const double cross_limit = 0.1 * gamma;
    for (int i = 0; i < niq; ++i) {
      cross[i] = std::clamp(cross[i], -cross_limit, cross_limit);
    }

    // N_corr = Lx + Jh' · ((μ.*rh + γ·e - cross) ./ z)
    Eigen::VectorXd ineq_rhs(niq);
    for (int i = 0; i < niq; ++i) {
      ineq_rhs[i] = (mu_safe[i] * rh[i] + gamma - cross[i]) / z_safe[i];
    }
    const Eigen::VectorXd N_corr = Lx + dh.transpose() * ineq_rhs;

    Eigen::VectorXd rhs_corr(n + meq);
    rhs_corr.head(n) = -N_corr;
    rhs_corr.tail(meq) = rhs_eq;

    // Reuse LU factorization — just a back-solve
    Eigen::VectorXd dx;
    Eigen::VectorXd dlambda;
    if (!solve_kkt(rhs_corr, dx, dlambda)) {
      out.status = "corrector back-solve failed";
      break;
    }

    // Recover corrector Δz, Δμ
    const Eigen::VectorXd jh_dx = dh * dx;
    Eigen::VectorXd dz = -(rh + z_safe) - jh_dx;
    // Δμ = -(μ + σ_s·Δz + (cross - γ·e) / z)
    Eigen::VectorXd dmu(niq);
    for (int i = 0; i < niq; ++i) {
      dmu[i] = -(mu_safe[i] + sigma_s[i] * dz[i] + (cross[i] - gamma) / z_safe[i]);
    }

    if (!dx.allFinite() || !dlambda.allFinite()) {
      out.status = "NaN in corrector step";
      break;
    }

    // ────────────────────────────────────────────────────────────────
    // Gondzio extra correctors (up to 2)
    // ────────────────────────────────────────────────────────────────
    if (gamma > 1e-12) {
      constexpr double kBetaGC = 0.1;
      for (int gc = 0; gc < 2; ++gc) {
        const double alpha_p_t = std::min(ftb(z_safe, dz, tau), 1.0);
        const double alpha_d_t = std::min(ftb(mu_safe, dmu, tau), 1.0);
        const Eigen::VectorXd z_t = z_safe + alpha_p_t * dz;
        const Eigen::VectorXd mu_t = mu_safe + alpha_d_t * dmu;

        // Find under-corrected complementarity pairs
        Eigen::VectorXd eps_gc =
            (Eigen::VectorXd::Constant(niq, kBetaGC * gamma) - z_t.cwiseProduct(mu_t)).cwiseMax(0.0);
        if (eps_gc.maxCoeff() < 1e-12 * gamma) {
          break;
        }

        // Solve extra correction (equality residual = 0)
        Eigen::VectorXd rhs_gc(n + meq);
        rhs_gc.head(n) = -(dh.transpose() * eps_gc.cwiseQuotient(z_safe));
        rhs_gc.tail(meq).setZero();

        Eigen::VectorXd dx_gc;
        Eigen::VectorXd dlambda_gc;
        if (!solve_kkt(rhs_gc, dx_gc, dlambda_gc)) {
          break;
        }
        if (!dx_gc.allFinite()) {
          break;
        }

        const Eigen::VectorXd dz_gc = -(dh * dx_gc);
        const Eigen::VectorXd dmu_gc = eps_gc.cwiseQuotient(z_safe) - sigma_s.cwiseProduct(dz_gc);

        dx += dx_gc;
        dlambda += dlambda_gc;
        dz += dz_gc;
        dmu += dmu_gc;
      }
    }

    // ────────────────────────────────────────────────────────────────
    // Fraction-to-boundary step lengths (separate primal/dual)
    // ────────────────────────────────────────────────────────────────
    double alpha_p = std::max(std::min(ftb(z_safe, dz, tau), 1.0), 1e-10);
    double alpha_d = std::max(std::min(ftb(mu_safe, dmu, tau), 1.0), 1e-10);

    if (opt.verbose) {
      HACDCPF_LOG_DEBUG("  α_p={} α_d={} |dx|={} min(z)={} max(σ)={} min(σ)={}",
                alpha_p, alpha_d, inf_norm(dx),
                z_safe.minCoeff(), sigma_s.maxCoeff(), sigma_s.minCoeff());
    }

    // ────────────────────────────────────────────────────────────────
    // Accept step (no line search — direct acceptance per Julia)
    // ────────────────────────────────────────────────────────────────
    x += alpha_p * dx;
    z += alpha_p * dz;
    mu += alpha_d * dmu;
    lambda += alpha_d * dlambda;

    // Safeguard positivity
    z = z.cwiseMax(1e-12);
    mu = mu.cwiseMax(1e-12);

    if (!x.allFinite()) {
      out.status = "NaN in x after step";
      break;
    }

    // Re-evaluate functions and derivatives
    obj = objective(prob, x);
    equality_constraints(prob, x, eq_ws, rg);
    equality_jacobian(prob, x, eq_ws, jg);
    assemble_inequalities(prob, x, xmin, xmax, lb_cols, ub_cols, rh, dh);
    objective_gradient_hessian_diag(prob, x, grad, hdiag);
    Lx = grad + jg.transpose() * lambda + dh.transpose() * mu;
    lagrangian_hessian(prob, x, lambda, get_nu_ptr(mu), Lxx, 1e-12);

    std::tie(feascond, gradcond, compcond, costcond) =
        compute_convergence(x, z, lambda, mu, rg, rh, Lx, obj);

    // Track best iterate
    const double cur_metric = std::max({feascond, gradcond, compcond});
    if (cur_metric < best_metric) {
      best_metric = cur_metric;
      best_x = x;
      best_lambda = lambda;
      best_mu = mu;
      best_z = z;
      best_feas = feascond;
      best_grad = gradcond;
      best_comp = compcond;
      best_iter = iter + 1;
    }

    if (feascond < feas_tol && gradcond < feas_tol && compcond < comp_tol) {
      converged = true;
      out.converged = true;
      out.status = "converged";
      out.iterations = iter + 1;
    }
  }

  // If not converged, attempt restoration from best iterate.
  // Accept with 10× tolerance if best iterate is substantially better.
  if (!converged && opt.verbose) {
    HACDCPF_LOG_DEBUG("[parity-ipm] best iterate at iter={} feas={} grad={} comp={} metric={}",
              best_iter, best_feas, best_grad, best_comp, best_metric);
  }
  if (!converged && best_feas < feas_tol && best_grad < feas_tol && best_comp < comp_tol) {
    x = best_x;
    lambda = best_lambda;
    mu = best_mu;
    z = best_z;
    feascond = best_feas;
    gradcond = best_grad;
    compcond = best_comp;
    converged = true;
    out.converged = true;
    out.status = "converged (best iterate restoration)";
    out.iterations = best_iter;
  }

  out.x = x;
  out.lambda_eq = lambda;
  out.mu = mu;
  out.z = z;
  out.primal_inf = feascond;
  out.dual_inf = gradcond;
  out.complementarity = compcond;
  return out;
}

}  // namespace hacdcpf::opf::parity
