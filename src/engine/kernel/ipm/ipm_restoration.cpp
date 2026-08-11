#include "mipsolvers/engine/kernel/ipm/ipm_restoration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mipsolvers::engine {

namespace {

Eigen::VectorXd make_dr(const Eigen::VectorXd& x_R) {
  Eigen::VectorXd d(x_R.size());
  for (int i = 0; i < x_R.size(); ++i) {
    const double a = std::max(1.0, std::abs(x_R[i]));
    d[i] = std::min(1.0, 1.0 / a);
  }
  return d;
}

Eigen::VectorXd restoration_variable_scale(
    const NLPModel& prob, const Eigen::VectorXd& x_R) {
  Eigen::VectorXd scale(x_R.size());
  for (int col = 0; col < x_R.size(); ++col) {
    if (col >= static_cast<int>(prob.vars.size())) {
      scale[col] = std::max(1.0, std::abs(x_R[col]));
      continue;
    }
    const auto& var = prob.vars[static_cast<std::size_t>(col)];
    const bool finite_lb = variable_has_finite_lower_bound(var.lb);
    const bool finite_ub = variable_has_finite_upper_bound(var.ub);
    const double width = finite_lb && finite_ub
        ? std::max(0.0, var.ub - var.lb)
        : 0.0;
    scale[col] = std::max({1.0, std::abs(x_R[col]), width,
                           finite_lb ? std::abs(var.lb) : 0.0,
                           finite_ub ? std::abs(var.ub) : 0.0});
  }
  return scale;
}

}  // namespace

RestorationBuild build_restoration_nlp(const NLPModel& prob,
                                       const Eigen::VectorXd& x_R,
                                       double zeta) {
  RestorationBuild build;
  const int n_x = static_cast<int>(x_R.size());

  // Determine equality count by probing jac_g at x_R once, if available.
  int m_eq = 0;
  Eigen::VectorXd g0;
  if (prob.g) {
    prob.g(x_R, g0);
    m_eq = static_cast<int>(g0.size());
  }
  Eigen::VectorXd equality_row_reach = Eigen::VectorXd::Zero(m_eq);
  if (m_eq > 0 && prob.jac_g) {
    Eigen::SparseMatrix<double> jacobian;
    prob.jac_g(x_R, jacobian);
    if (jacobian.rows() == m_eq && jacobian.cols() == n_x) {
      const Eigen::VectorXd variable_scale =
          restoration_variable_scale(prob, x_R);
      for (int col = 0; col < jacobian.outerSize(); ++col) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(jacobian, col); it;
             ++it) {
          const double scaled = it.value() * variable_scale[col];
          equality_row_reach[it.row()] += scaled * scaled;
        }
      }
      equality_row_reach = equality_row_reach.array().sqrt();
    }
  }

  const int n_p = m_eq;
  const int n_n = m_eq;
  const int N = n_x + n_p + n_n;

  build.n_x = n_x;
  build.n_p = n_p;
  build.n_n = n_n;
  build.x_R = x_R;

  NLPModel& out = build.model;
  out.sense = Sense::Minimize;

  // Variable bounds: propagate x bounds, p,n >= 0.
  out.vars.resize(static_cast<size_t>(N));
  for (int j = 0; j < n_x; ++j) {
    if (j < static_cast<int>(prob.vars.size())) {
      out.vars[j] = prob.vars[j];
    } else {
      VariableMeta v;
      v.lb = -kVariableNoBound;
      v.ub = kVariableNoBound;
      out.vars[j] = v;
    }
  }
  for (int j = n_x; j < N; ++j) {
    VariableMeta v;
    v.lb = 0.0;
    v.ub = kVariableNoBound;
    out.vars[j] = v;
  }

  // Initial point: x = x_R, p = max(0, g(x_R)+), n = max(0, -g(x_R)+).
  Eigen::VectorXd w0 = Eigen::VectorXd::Zero(N);
  w0.head(n_x) = x_R;
  if (m_eq > 0 && prob.g) {
    const double root_epsilon =
        std::sqrt(std::numeric_limits<double>::epsilon());
    const double representability_floor =
        std::sqrt(std::numeric_limits<double>::min());
    for (int i = 0; i < m_eq; ++i) {
      const double gi = g0[i];
      const double resolution = std::max(
          representability_floor,
          root_epsilon * std::max(std::abs(gi), equality_row_reach[i]));
      // Adding the same row-local resolution to both sides preserves
      // p-n=-g while obtaining a strictly positive, scale-covariant start.
      const double p_i = resolution + std::max(-gi, 0.0);
      const double n_i = resolution + std::max(gi, 0.0);
      w0[n_x + i] = p_i;
      w0[n_x + n_p + i] = n_i;
    }
  }
  out.x0 = w0;

  Eigen::VectorXd dr = make_dr(x_R);
  const double zeta_val = zeta > 0.0 && std::isfinite(zeta)
      ? zeta
      : std::sqrt(std::numeric_limits<double>::epsilon());
  const Eigen::VectorXd x_ref = x_R;

  // Objective: Σ (p_i + n_i) + (ζ/2) ‖D_R (x − x_R)‖²
  out.f = [zeta_val, dr, x_ref, n_x, n_p, n_n](
              const Eigen::VectorXd& w) -> double {
    double s_pen = 0.0;
    for (int i = 0; i < n_p + n_n; ++i) {
      s_pen += w[n_x + i];
    }
    double s_prox = 0.0;
    for (int j = 0; j < n_x; ++j) {
      const double t = dr[j] * (w[j] - x_ref[j]);
      s_prox += t * t;
    }
    return s_pen + 0.5 * zeta_val * s_prox;
  };

  out.grad = [zeta_val, dr, x_ref, n_x, n_p, n_n](
                 const Eigen::VectorXd& w, Eigen::VectorXd& g) {
    g = Eigen::VectorXd::Zero(n_x + n_p + n_n);
    for (int j = 0; j < n_x; ++j) {
      g[j] = zeta_val * dr[j] * dr[j] * (w[j] - x_ref[j]);
    }
    for (int i = 0; i < n_p + n_n; ++i) {
      g[n_x + i] = 1.0;
    }
  };

  // Objective Hessian: diag(ζ·D_R²) on the x block, zero elsewhere.
  out.hess = [zeta_val, dr, n_x, n_p, n_n](
                 const Eigen::VectorXd& /*w*/,
                 Eigen::SparseMatrix<double>& H) {
    const int N = n_x + n_p + n_n;
    H.resize(N, N);
    H.reserve(Eigen::VectorXi::Constant(N, 1));
    for (int j = 0; j < n_x; ++j) {
      const double d = dr[j];
      H.insert(j, j) = zeta_val * d * d;
    }
    H.makeCompressed();
  };

  // Equality constraints: g(x) + p − n = 0   (size m_eq)
  if (m_eq > 0) {
    out.g = [g_orig = prob.g, n_x, n_p, m_eq](
                const Eigen::VectorXd& w, Eigen::VectorXd& v) {
      Eigen::VectorXd x = w.head(n_x);
      Eigen::VectorXd g_val;
      g_orig(x, g_val);
      v.resize(m_eq);
      for (int i = 0; i < m_eq; ++i) {
        v[i] = g_val[i] + w[n_x + i] - w[n_x + n_p + i];
      }
    };

    out.jac_g = [jg_orig = prob.jac_g, n_x, n_p, n_n, m_eq](
                    const Eigen::VectorXd& w,
                    Eigen::SparseMatrix<double>& J) {
      Eigen::VectorXd x = w.head(n_x);
      Eigen::SparseMatrix<double> Jx;
      jg_orig(x, Jx);
      const int N = n_x + n_p + n_n;
      J.resize(m_eq, N);
      // Pre-reserve: Jx nnz + 2 * m_eq for ±I blocks.
      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(static_cast<size_t>(Jx.nonZeros() + 2 * m_eq));
      for (int k = 0; k < Jx.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Jx, k); it; ++it) {
          triplets.emplace_back(it.row(), it.col(), it.value());
        }
      }
      for (int i = 0; i < m_eq; ++i) {
        triplets.emplace_back(i, n_x + i, 1.0);
        triplets.emplace_back(i, n_x + n_p + i, -1.0);
      }
      J.setFromTriplets(triplets.begin(), triplets.end());
      J.makeCompressed();
    };
  }

  // Inequality constraints: keep h_nonlin(x) ≤ 0.
  if (prob.h) {
    out.h = [h_orig = prob.h, n_x](const Eigen::VectorXd& w,
                                    Eigen::VectorXd& v) {
      Eigen::VectorXd x = w.head(n_x);
      h_orig(x, v);
    };
  }
  if (prob.jac_h) {
    out.jac_h = [jh_orig = prob.jac_h, n_x, n_p, n_n](
                    const Eigen::VectorXd& w,
                    Eigen::SparseMatrix<double>& J) {
      Eigen::VectorXd x = w.head(n_x);
      Eigen::SparseMatrix<double> Jx;
      jh_orig(x, Jx);
      const int N = n_x + n_p + n_n;
      J.resize(Jx.rows(), N);
      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(static_cast<size_t>(Jx.nonZeros()));
      for (int k = 0; k < Jx.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Jx, k); it; ++it) {
          triplets.emplace_back(it.row(), it.col(), it.value());
        }
      }
      J.setFromTriplets(triplets.begin(), triplets.end());
      J.makeCompressed();
    };
  }

  // Lagrangian Hessian: call prob.lagrangian_hess on the x-block (unscaled)
  // and embed it into the leading n_x × n_x block. Add the proximal
  // contribution (ζ·D_R²) on the diagonal. Slack blocks contribute no
  // curvature because g is linear in (p, n) and the l1 penalty is linear.
  if (prob.lagrangian_hess || prob.hess) {
    auto lag_orig = prob.lagrangian_hess;
    auto hess_orig = prob.hess;
    out.lagrangian_hess = [lag_orig, hess_orig, zeta_val, dr, n_x, n_p, n_n,
                           m_eq](const Eigen::VectorXd& w,
                                 const Eigen::VectorXd& lambda,
                                 const Eigen::VectorXd* mu_nonlin,
                                 Eigen::SparseMatrix<double>& H) {
      Eigen::VectorXd x = w.head(n_x);
      Eigen::SparseMatrix<double> Hx;
      if (lag_orig) {
        // Restoration λ is for the augmented constraints g+p-n=0, but the
        // Hessian of p and n is zero, so only the x-block contribution of
        // λᵀ g(x) matters. Pass the m_eq-sized λ directly.
        Eigen::VectorXd lam = lambda.size() == m_eq
                                  ? lambda
                                  : Eigen::VectorXd::Zero(m_eq);
        lag_orig(x, lam, mu_nonlin, Hx);
      } else if (hess_orig) {
        hess_orig(x, Hx);
      }

      const int N = n_x + n_p + n_n;
      H.resize(N, N);
      std::vector<Eigen::Triplet<double>> triplets;
      triplets.reserve(static_cast<size_t>(Hx.nonZeros() + n_x));
      for (int k = 0; k < Hx.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(Hx, k); it; ++it) {
          triplets.emplace_back(it.row(), it.col(), it.value());
        }
      }
      // Proximal diagonal ζ·D_R² on x block.
      for (int j = 0; j < n_x; ++j) {
        const double d = dr[j];
        triplets.emplace_back(j, j, zeta_val * d * d);
      }
      H.setFromTriplets(triplets.begin(), triplets.end());
      H.makeCompressed();
    };
  }

  return build;
}

Eigen::VectorXd extract_x_from_restoration(const RestorationBuild& build,
                                           const Eigen::VectorXd& w) {
  if (w.size() < build.n_x) return build.x_R;
  return w.head(build.n_x);
}

RestorationWarmStart recover_restoration_warm_start(
    const NLPModel& original, const RestorationBuild& build,
    const Eigen::VectorXd& restoration_x,
    const Eigen::VectorXd& restoration_equality_dual,
    const Eigen::VectorXd& restoration_inequality_dual,
    const Eigen::VectorXd& restoration_slack,
    int restoration_nonlinear_inequalities,
    const std::vector<int>& restoration_lower_bound_columns,
    const std::vector<int>& restoration_upper_bound_columns) {
  RestorationWarmStart warm;
  const int n = static_cast<int>(original.vars.size());
  if (restoration_x.size() < build.n_x || build.n_x != n ||
      restoration_nonlinear_inequalities < 0 ||
      restoration_inequality_dual.size() != restoration_slack.size() ||
      restoration_inequality_dual.size() !=
          restoration_nonlinear_inequalities +
              static_cast<int>(restoration_lower_bound_columns.size()) +
              static_cast<int>(restoration_upper_bound_columns.size()) ||
      !restoration_equality_dual.allFinite() ||
      !restoration_inequality_dual.allFinite() ||
      !restoration_slack.allFinite()) {
    return warm;
  }

  warm.x = restoration_x.head(n);
  if (!warm.x.allFinite()) return RestorationWarmStart{};

  int original_nonlinear_inequalities = 0;
  if (original.h) {
    Eigen::VectorXd h;
    original.h(warm.x, h);
    if (!h.allFinite()) return RestorationWarmStart{};
    original_nonlinear_inequalities = static_cast<int>(h.size());
  }
  if (original_nonlinear_inequalities !=
      restoration_nonlinear_inequalities) {
    return RestorationWarmStart{};
  }

  std::vector<int> original_lower_columns;
  std::vector<int> original_upper_columns;
  for (int col = 0; col < n; ++col) {
    const auto& var = original.vars[static_cast<std::size_t>(col)];
    if (variable_has_finite_lower_bound(var.lb)) {
      original_lower_columns.push_back(col);
    }
    if (variable_has_finite_upper_bound(var.ub)) {
      original_upper_columns.push_back(col);
    }
  }

  std::vector<int> restoration_lower_position(
      static_cast<std::size_t>(n), -1);
  std::vector<int> restoration_upper_position(
      static_cast<std::size_t>(n), -1);
  for (int k = 0;
       k < static_cast<int>(restoration_lower_bound_columns.size()); ++k) {
    const int col = restoration_lower_bound_columns[static_cast<std::size_t>(k)];
    if (col >= 0 && col < n) {
      restoration_lower_position[static_cast<std::size_t>(col)] = k;
    }
  }
  for (int k = 0;
       k < static_cast<int>(restoration_upper_bound_columns.size()); ++k) {
    const int col = restoration_upper_bound_columns[static_cast<std::size_t>(k)];
    if (col >= 0 && col < n) {
      restoration_upper_position[static_cast<std::size_t>(col)] = k;
    }
  }

  const int original_rows = original_nonlinear_inequalities +
      static_cast<int>(original_lower_columns.size()) +
      static_cast<int>(original_upper_columns.size());
  warm.inequality_dual.resize(original_rows);
  warm.slack.resize(original_rows);
  if (original_nonlinear_inequalities > 0) {
    warm.inequality_dual.head(original_nonlinear_inequalities) =
        restoration_inequality_dual.head(original_nonlinear_inequalities);
    warm.slack.head(original_nonlinear_inequalities) =
        restoration_slack.head(original_nonlinear_inequalities);
  }

  int destination = original_nonlinear_inequalities;
  const int restoration_lower_offset = restoration_nonlinear_inequalities;
  for (int col : original_lower_columns) {
    const int position =
        restoration_lower_position[static_cast<std::size_t>(col)];
    if (position < 0) return RestorationWarmStart{};
    const int source = restoration_lower_offset + position;
    warm.inequality_dual[destination] = restoration_inequality_dual[source];
    warm.slack[destination] = restoration_slack[source];
    ++destination;
  }

  const int restoration_upper_offset = restoration_lower_offset +
      static_cast<int>(restoration_lower_bound_columns.size());
  for (int col : original_upper_columns) {
    const int position =
        restoration_upper_position[static_cast<std::size_t>(col)];
    if (position < 0) return RestorationWarmStart{};
    const int source = restoration_upper_offset + position;
    warm.inequality_dual[destination] = restoration_inequality_dual[source];
    warm.slack[destination] = restoration_slack[source];
    ++destination;
  }

  if ((warm.inequality_dual.array() <= 0.0).any() ||
      (warm.slack.array() <= 0.0).any()) {
    return RestorationWarmStart{};
  }
  warm.equality_dual = restoration_equality_dual;
  warm.valid = true;
  return warm;
}

}  // namespace mipsolvers::engine
