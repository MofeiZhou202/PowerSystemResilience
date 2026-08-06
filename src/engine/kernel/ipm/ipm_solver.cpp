#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_filter.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_restoration.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_scaling.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

constexpr double kBoundInfinity = 1e19;
// Numerical interior floor. This must remain well below
// tol_complementarity / ||mu||_inf; otherwise active constraints acquire an
// artificial complementarity floor s_i*mu_i above the requested KKT tolerance.
constexpr double kMinPositive = 1e-12;
constexpr double kMinReg = 1e-9;
constexpr double kMaxReg = 1e-2;
constexpr int kMaxBacktracks = 20;

bool is_effectively_finite(double value) {
  return std::isfinite(value) && std::abs(value) < kBoundInfinity;
}

double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

struct ProductDistribution {
  int count{0};
  double minimum{0.0};
  double median{0.0};
  double maximum{0.0};
};

ProductDistribution summarize_products(const Eigen::VectorXd& lhs,
                                       const Eigen::VectorXd& rhs,
                                       int begin, int count) {
  ProductDistribution out;
  if (count <= 0) return out;
  std::vector<double> products;
  products.reserve(static_cast<std::size_t>(count));
  for (int i = begin; i < begin + count; ++i) {
    products.push_back(lhs[i] * rhs[i]);
  }
  std::sort(products.begin(), products.end());
  out.count = count;
  out.minimum = products.front();
  out.maximum = products.back();
  const std::size_t middle = products.size() / 2;
  out.median = products.size() % 2 == 0
      ? 0.5 * (products[middle - 1] + products[middle])
      : products[middle];
  return out;
}

double objective_value(const NLPModel& prob, const Eigen::VectorXd& x) {
  return prob.f ? prob.f(x) : 0.0;
}

bool build_objective_gradient(const NLPModel& prob,
                              const Eigen::VectorXd& x,
                              Eigen::VectorXd& grad,
                              std::string& status) {
  if (!prob.grad) {
    status = "NLP model missing gradient callback";
    return false;
  }
  prob.grad(x, grad);
  if (grad.size() != x.size() || !grad.allFinite()) {
    status = "NLP gradient callback returned invalid values";
    return false;
  }
  if (prob.sense == Sense::Maximize) {
    grad = -grad;
  }
  return true;
}

Eigen::VectorXd nonlinear_inequality_multipliers(const Eigen::VectorXd& mu,
                                                 int n_nonlinear_ineq) {
  if (n_nonlinear_ineq <= 0 || mu.size() == 0) {
    return Eigen::VectorXd::Zero(0);
  }
  const int count = std::min(n_nonlinear_ineq, static_cast<int>(mu.size()));
  return mu.head(count);
}

Eigen::SparseMatrix<double> symmetrize_hessian(Eigen::SparseMatrix<double> hess) {
  hess.makeCompressed();
  Eigen::SparseMatrix<double> ht = hess.transpose();
  Eigen::SparseMatrix<double> hs = (hess + ht) * 0.5;
  hs.makeCompressed();
  return hs;
}

Eigen::SparseMatrix<double> diagonal_sparse(const Eigen::VectorXd& diag) {
  const int n = static_cast<int>(diag.size());
  Eigen::SparseMatrix<double> out(n, n);
  out.reserve(Eigen::VectorXi::Constant(n, 1));
  for (int i = 0; i < n; ++i) {
    out.insert(i, i) = diag[i];
  }
  out.makeCompressed();
  return out;
}

struct DiagonalQNState {
  bool active{false};
  Eigen::VectorXd diag;
  Eigen::VectorXd prev_x;
  Eigen::VectorXd prev_objective_grad;
  Eigen::SparseMatrix<double> prev_jg;
  Eigen::SparseMatrix<double> prev_jh;
  struct SparseBlockUpdate {
    std::vector<int> index;
    Eigen::MatrixXd values;
  };
  std::vector<SparseBlockUpdate> blocks;
  int sparse_block_size{8};
  int max_blocks{6};
};

struct NLPState {
  double obj_orig{0.0};
  double obj_eff{0.0};
  Eigen::VectorXd grad;
  Eigen::SparseMatrix<double> hess;
  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> jg;
  Eigen::VectorXd h;
  Eigen::SparseMatrix<double> jh;
  int n_nonlinear_ineq{0};
};

bool initialize_equality_duals_least_squares(
    const NLPState& state,
    const Eigen::VectorXd& mu,
    double multiplier_norm_limit,
    Eigen::VectorXd& lambda) {
  const int meq = static_cast<int>(state.jg.rows());
  if (meq == 0 || state.jg.cols() != state.grad.size() ||
      lambda.size() != meq || mu.size() != state.jh.rows() ||
      !std::isfinite(multiplier_norm_limit) || multiplier_norm_limit <= 0.0) {
    return false;
  }

  const Eigen::VectorXd stationarity_without_equalities =
      state.grad + state.jh.transpose() * mu;
  const Eigen::SparseMatrix<double> metric =
      diagonal_sparse(Eigen::VectorXd::Ones(state.grad.size()));
  SparseInertiaKKTCache cache;
  InertiaSettings settings;
  // This projection needs the factor solve, not a dense tangent-space
  // certificate. In MUMPS builds this goes directly to symmetric LDLT and
  // avoids the catastrophic cost of a large SparseQR cold start.
  settings.max_tangent_dimension = -1;
  settings.mu = 1.0;
  InertiaStatus inertia;
  double delta_w_last = 0.0;
  if (!factor_kkt_inertia_corrected_sparse(
          metric, state.jg, settings, delta_w_last, cache, inertia)) {
    return false;
  }

  Eigen::VectorXd rhs(state.grad.size() + meq);
  rhs.head(state.grad.size()) = -stationarity_without_equalities;
  rhs.tail(meq).setZero();
  Eigen::VectorXd stationarity_remainder;
  Eigen::VectorXd candidate;
  if (!solve_kkt_inertia_corrected_sparse(
          cache, rhs, stationarity_remainder, candidate) ||
      candidate.size() != meq ||
      !candidate.allFinite() || inf_norm(candidate) > multiplier_norm_limit) {
    return false;
  }

  const Eigen::SparseMatrix<double> jg_transpose = state.jg.transpose();
  const double current_residual = inf_norm(
      stationarity_without_equalities + jg_transpose * lambda);
  const double candidate_residual = inf_norm(
      stationarity_without_equalities + jg_transpose * candidate);
  const double required_improvement =
      32.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, current_residual);
  if (candidate_residual + required_improvement >= current_residual) {
    return false;
  }

  lambda = candidate;
  return true;
}

void initialize_quasi_newton_state(int n, DiagonalQNState& state) {
  state.active = true;
  state.diag = Eigen::VectorXd::Ones(n);
  state.prev_x.resize(0);
  state.prev_objective_grad.resize(0);
  state.prev_jg.resize(0, n);
  state.prev_jh.resize(0, n);
  state.blocks.clear();
}

Eigen::VectorXd apply_sparse_qn_blocks(const DiagonalQNState& state,
                                       const Eigen::VectorXd& v) {
  Eigen::VectorXd out = Eigen::VectorXd::Zero(v.size());
  for (const auto& block : state.blocks) {
    Eigen::VectorXd local = Eigen::VectorXd::Zero(static_cast<int>(block.index.size()));
    for (int k = 0; k < static_cast<int>(block.index.size()); ++k) {
      local[k] = v[block.index[static_cast<size_t>(k)]];
    }
    const Eigen::VectorXd corr = block.values * local;
    for (int k = 0; k < static_cast<int>(block.index.size()); ++k) {
      out[block.index[static_cast<size_t>(k)]] += corr[k];
    }
  }
  return out;
}

std::vector<int> select_top_abs_indices(const Eigen::VectorXd& v, int max_count) {
  std::vector<int> index(static_cast<size_t>(v.size()));
  std::iota(index.begin(), index.end(), 0);
  const int count = std::min(max_count, static_cast<int>(index.size()));
  std::partial_sort(index.begin(), index.begin() + count, index.end(), [&](int a, int b) {
    return std::abs(v[a]) > std::abs(v[b]);
  });
  index.resize(static_cast<size_t>(count));
  std::sort(index.begin(), index.end());
  return index;
}

Eigen::SparseMatrix<double> sparse_qn_hessian_matrix(const DiagonalQNState& state) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(state.diag.size()) +
                   static_cast<size_t>(state.blocks.size() * state.sparse_block_size *
                                       state.sparse_block_size));
  for (int i = 0; i < state.diag.size(); ++i) {
    triplets.emplace_back(i, i, std::max(1e-6, state.diag[i]));
  }
  for (const auto& block : state.blocks) {
    for (int r = 0; r < static_cast<int>(block.index.size()); ++r) {
      for (int c = 0; c < static_cast<int>(block.index.size()); ++c) {
        const double value = block.values(r, c);
        if (std::abs(value) > 1e-12) {
          triplets.emplace_back(block.index[static_cast<size_t>(r)],
                                block.index[static_cast<size_t>(c)], value);
        }
      }
    }
  }
  Eigen::SparseMatrix<double> out(state.diag.size(), state.diag.size());
  out.setFromTriplets(triplets.begin(), triplets.end());
  out.makeCompressed();
  return out;
}

void update_quasi_newton_state(const Eigen::VectorXd& x,
                               const Eigen::VectorXd& objective_grad,
                               const Eigen::SparseMatrix<double>& jg,
                               const Eigen::SparseMatrix<double>& jh,
                               const Eigen::VectorXd& lambda,
                               const Eigen::VectorXd& mu,
                               DiagonalQNState& state) {
  if (!state.active) {
    return;
  }
  if (state.diag.size() != x.size()) {
    state.diag = Eigen::VectorXd::Ones(x.size());
  }
  const bool multiplier_dimensions_match =
      lambda.size() == jg.rows() && mu.size() == jh.rows();
  const bool previous_dimensions_match =
      state.prev_x.size() == x.size() &&
      state.prev_objective_grad.size() == x.size() &&
      state.prev_jg.cols() == x.size() &&
      state.prev_jh.cols() == x.size() &&
      state.prev_jg.rows() == lambda.size() &&
      state.prev_jh.rows() == mu.size();
  if (multiplier_dimensions_match && previous_dimensions_match) {
    const Eigen::VectorXd s = x - state.prev_x;
    // NLP secant vector with the new multipliers held fixed:
    // y_k = grad_x L(x_{k+1}, lambda_{k+1}, mu_{k+1})
    //     - grad_x L(x_k,     lambda_{k+1}, mu_{k+1}).
    // This includes nonlinear constraint curvature without requiring a
    // Lagrangian-Hessian callback and avoids contaminating y_k with the
    // multiplier step itself.
    const Eigen::VectorXd current_lagrangian_grad =
        objective_grad + jg.transpose() * lambda + jh.transpose() * mu;
    const Eigen::VectorXd previous_lagrangian_grad =
        state.prev_objective_grad + state.prev_jg.transpose() * lambda +
        state.prev_jh.transpose() * mu;
    const Eigen::VectorXd y =
        current_lagrangian_grad - previous_lagrangian_grad;
    const double sy = s.dot(y);
    const double ss = s.squaredNorm();
    const double fallback_curv = (sy > 1e-12 && ss > 1e-12)
        ? std::clamp(std::abs(sy / ss), 1e-4, 1e4)
        : 1.0;
    for (int i = 0; i < x.size(); ++i) {
      const double si = s[i];
      const double yi = y[i];
      if (std::abs(si) > 1e-8 && yi * si > 1e-12) {
        const double curv = std::clamp(yi / si, 1e-6, 1e6);
        state.diag[i] = 0.2 * state.diag[i] + 0.8 * curv;
      } else {
        state.diag[i] = 0.9 * state.diag[i] + 0.1 * fallback_curv;
      }
    }

    const Eigen::VectorXd residual =
        y - state.diag.cwiseProduct(s) - apply_sparse_qn_blocks(state, s);
    const std::vector<int> support =
        select_top_abs_indices(residual, state.sparse_block_size);
    if (!support.empty()) {
      Eigen::VectorXd local_residual =
          Eigen::VectorXd::Zero(static_cast<int>(support.size()));
      Eigen::VectorXd local_step =
          Eigen::VectorXd::Zero(static_cast<int>(support.size()));
      for (int k = 0; k < static_cast<int>(support.size()); ++k) {
        const int index = support[static_cast<size_t>(k)];
        local_residual[k] = residual[index];
        local_step[k] = s[index];
      }
      const double denom = local_residual.dot(local_step);
      const double skip_threshold = 1e-8 *
          local_residual.norm() * local_step.norm();
      // Limited-memory SR1 skip condition. Negative denominators are retained
      // so genuine nonconvex Lagrangian curvature reaches inertia correction.
      if (std::abs(denom) > std::max(1e-14, skip_threshold)) {
        state.blocks.push_back(
            {support, (local_residual * local_residual.transpose()) / denom});
        if (static_cast<int>(state.blocks.size()) > state.max_blocks) {
          state.blocks.erase(state.blocks.begin());
        }
      }
    }
  }
  state.prev_x = x;
  state.prev_objective_grad = objective_grad;
  state.prev_jg = jg;
  state.prev_jh = jh;
}

bool build_lagrangian_hessian(const NLPModel& prob,
                              const Eigen::VectorXd& x,
                              const Eigen::VectorXd& lambda_eq,
                              const Eigen::VectorXd* mu_nonlinear,
                              Eigen::SparseMatrix<double>& hess,
                              std::string& status,
                              const DiagonalQNState* qn_state = nullptr) {
  const int n = static_cast<int>(x.size());
  if (prob.lagrangian_hess) {
    prob.lagrangian_hess(x, lambda_eq, mu_nonlinear, hess);
    if (hess.rows() != n || hess.cols() != n) {
      status = "NLP Lagrangian Hessian callback returned invalid shape";
      return false;
    }
    hess = symmetrize_hessian(std::move(hess));
  } else if (prob.hess) {
    prob.hess(x, hess);
    if (hess.rows() != n || hess.cols() != n) {
      status = "NLP Hessian callback returned invalid shape";
      return false;
    }
    hess = symmetrize_hessian(std::move(hess));
    if (prob.sense == Sense::Maximize) {
      hess *= -1.0;
    }
  } else {
    if (qn_state && qn_state->active && qn_state->diag.size() == n) {
      hess = sparse_qn_hessian_matrix(*qn_state);
    } else {
      hess = diagonal_sparse(Eigen::VectorXd::Ones(n));
    }
  }
  return true;
}

Eigen::VectorXd complementarity_target(const Eigen::VectorXd& s,
                                       const Eigen::VectorXd& mu,
                                       const Eigen::VectorXd& ds_aff,
                                       const Eigen::VectorXd& dmu_aff,
                                       double alpha_aff_pri,
                                       double alpha_aff_dual) {
  if (s.size() == 0) {
    return Eigen::VectorXd::Zero(0);
  }

  const Eigen::VectorXd cur = s.cwiseProduct(mu).cwiseMax(
      Eigen::VectorXd::Constant(s.size(), kMinPositive));
  const Eigen::VectorXd aff =
      (s + alpha_aff_pri * ds_aff)
          .cwiseProduct(mu + alpha_aff_dual * dmu_aff)
          .cwiseMax(Eigen::VectorXd::Zero(s.size()));

  Eigen::VectorXd target(s.size());
  for (int i = 0; i < s.size(); ++i) {
    const double ratio = std::max(0.0, aff[i] / cur[i]);
    const double sigma_i = std::clamp(std::pow(ratio, 3.0), 0.0, 1.0);
    target[i] = sigma_i * cur[i];
  }
  return target;
}

bool build_equalities(const NLPModel& prob,
                      const Eigen::VectorXd& x,
                      Eigen::VectorXd& g,
                      Eigen::SparseMatrix<double>& jg,
                      std::string& status) {
  const int n = static_cast<int>(x.size());
  if (prob.g) {
    if (!prob.jac_g) {
      status = "NLP model missing jac_g callback";
      return false;
    }
    prob.g(x, g);
    if (!g.allFinite()) {
      status = "NLP equality callback returned non-finite values";
      return false;
    }
    prob.jac_g(x, jg);
    if (jg.rows() != g.size() || jg.cols() != n) {
      status = "NLP jac_g callback returned invalid shape";
      return false;
    }
    jg.makeCompressed();
  } else {
    g = Eigen::VectorXd::Zero(0);
    jg.resize(0, n);
    jg.setZero();
  }
  return true;
}

bool build_inequalities(const NLPModel& prob,
                        const Eigen::VectorXd& x,
                        const std::vector<int>& lb_cols,
                        const std::vector<int>& ub_cols,
                        Eigen::VectorXd& h,
                        Eigen::SparseMatrix<double>& jh,
                        int& n_nonlinear_ineq,
                        std::string& status) {
  const int n = static_cast<int>(x.size());
  n_nonlinear_ineq = 0;
  Eigen::VectorXd h_nonlin = Eigen::VectorXd::Zero(0);
  Eigen::SparseMatrix<double> jh_nonlin(0, n);
  if (prob.h) {
    prob.h(x, h_nonlin);
    if (!h_nonlin.allFinite()) {
      status = "NLP inequality callback returned non-finite values";
      return false;
    }
    n_nonlinear_ineq = static_cast<int>(h_nonlin.size());
    if (n_nonlinear_ineq > 0 && !prob.jac_h) {
      status = "NLP model missing jac_h callback";
      return false;
    }
    if (n_nonlinear_ineq > 0) {
      prob.jac_h(x, jh_nonlin);
      if (jh_nonlin.rows() != n_nonlinear_ineq || jh_nonlin.cols() != n) {
        status = "NLP jac_h callback returned invalid shape";
        return false;
      }
      jh_nonlin.makeCompressed();
    }
  }

  const int m = n_nonlinear_ineq + static_cast<int>(lb_cols.size()) + static_cast<int>(ub_cols.size());
  h = Eigen::VectorXd::Zero(m);
  if (n_nonlinear_ineq > 0) {
    h.head(n_nonlinear_ineq) = h_nonlin;
  }

  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(jh_nonlin.nonZeros() + lb_cols.size() + ub_cols.size()));
  for (int col = 0; col < jh_nonlin.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jh_nonlin, col); it; ++it) {
      tri.emplace_back(it.row(), it.col(), it.value());
    }
  }

  int row = n_nonlinear_ineq;
  for (int c : lb_cols) {
    h[row] = prob.vars[static_cast<size_t>(c)].lb - x[c];
    tri.emplace_back(row, c, -1.0);
    ++row;
  }
  for (int c : ub_cols) {
    h[row] = x[c] - prob.vars[static_cast<size_t>(c)].ub;
    tri.emplace_back(row, c, 1.0);
    ++row;
  }

  jh.resize(m, n);
  jh.setFromTriplets(tri.begin(), tri.end());
  if (!h.allFinite()) {
    status = "Split inequality construction returned non-finite values";
    return false;
  }
  if (jh.rows() != h.size() || jh.cols() != x.size()) {
    status = "Split inequality Jacobian has invalid shape";
    return false;
  }
  jh.makeCompressed();
  return true;
}

double max_positive_step(const Eigen::VectorXd& v,
                         const Eigen::VectorXd& dv,
                         double alpha_max) {
  double alpha = 1.0;
  for (int i = 0; i < v.size(); ++i) {
    if (dv[i] < 0.0) {
      alpha = std::min(alpha, -alpha_max * v[i] / dv[i]);
    }
  }
  return std::clamp(alpha, 0.0, 1.0);
}

Eigen::SparseMatrix<double> scale_rows(const Eigen::SparseMatrix<double>& a,
                                       const Eigen::VectorXd& d) {
  Eigen::SparseMatrix<double> out = a;
  for (int col = 0; col < out.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(out, col); it; ++it) {
      it.valueRef() *= d[it.row()];
    }
  }
  out.makeCompressed();
  return out;
}

bool factor_kkt_with_regularization(const Eigen::SparseMatrix<double>& w,
                                    const Eigen::SparseMatrix<double>& jg,
                                    SparseKKTCache& cache) {
  for (double reg = kMinReg; reg <= kMaxReg; reg *= 10.0) {
    if (factor_kkt_sparse(cache, w, jg, reg)) {
      return true;
    }
  }
  cache.factored = false;
  return false;
}

// ── Augmented (uncondensed) Newton system ────────────────────────────────────
// Assembles the primal-dual Newton matrix in the uncondensed form
//   [ H + δ_W I   Jgᵀ   Jhᵀ  ]
//   [ Jg         -δ_C I   0   ]
//   [ Jh           0   -SM⁻¹  ]
// instead of the condensed [H + Jhᵀ(M/S)Jh, Jgᵀ; Jg, 0].  The augmented form
// trades a larger sparse factorization for: (i) no Jhᵀ(M/S)Jh product —
// O(nnz) assembly instead of a sparse triple product whose fill can be
// quadratic in the row counts; (ii) condition number ~κ of the Jacobian part
// instead of ~κ² (the same normal-equations penalty as in dense LA); and
// (iii) an inertia identity: by block elimination of the -SM⁻¹ block,
// inertia(augmented) = inertia(condensed) + (0, m_ineq, 0), so the usual
// (n, m_eq, 0) descent condition on the condensed form is equivalent to
// (n, m_eq + m_ineq, 0) here.  The sparsity pattern is invariant per problem;
// values are refilled per iteration through a scatter map (the same
// analyze-once/factorize-many contract as the KKT assembler).
struct AugmentedNewtonCache {
  SparseKKTCache kkt;  // solver + factor state (generic sparse LU)
  int n = 0, meq = 0, miq = 0;
  int h_nnz = -1, jg_nnz = -1, jh_nnz = -1;
  std::vector<int> h_outer, h_inner, jg_outer, jg_inner, jh_outer, jh_inner;
  std::vector<int> h_pos;   // nnz(H): value index of H(i,j)
  std::vector<int> jg_top;  // nnz(Jg): value index of Jg(r,c) at (n+r, c)
  std::vector<int> jg_bot;  // nnz(Jg): value index of Jg(r,c) at (c, n+r)
  std::vector<int> jh_top;  // nnz(Jh): value index of Jh(r,c) at (n+meq+r, c)
  std::vector<int> jh_bot;  // nnz(Jh): value index of Jh(r,c) at (c, n+meq+r)
  std::vector<int> diag_w;  // n: value index of (i,i)                 [+δ_W]
  std::vector<int> diag_c;  // meq: value index of (n+i,n+i)          [-δ_C]
  std::vector<int> diag_s;  // miq: value index of (n+meq+i,n+meq+i)  [-s_i/μ_i]
};

struct NewtonStructureProfile {
  std::string selected{"unselected"};
  int condensed_dimension{0};
  int augmented_dimension{0};
  int condensed_nonzeros{0};
  int augmented_nonzeros{0};
  double condensed_flops{0.0};
  double augmented_flops{0.0};
  double condensed_lnz{0.0};
  double augmented_lnz{0.0};
  bool analyzed{false};
};

Eigen::SparseMatrix<double> assemble_lower_newton_pattern(
    const Eigen::SparseMatrix<double>& h,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::SparseMatrix<double>* jh) {
  const int n = static_cast<int>(h.rows());
  const int meq = static_cast<int>(jg.rows());
  const int miq = jh == nullptr ? 0 : static_cast<int>(jh->rows());
  const int dim = n + meq + miq;
  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(h.nonZeros() + jg.nonZeros() +
                                  (jh == nullptr ? 0 : jh->nonZeros()) + dim));
  for (int col = 0; col < h.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(h, col); it; ++it) {
      if (it.row() >= it.col()) tri.emplace_back(it.row(), it.col(), 1.0);
    }
  }
  for (int i = 0; i < n; ++i) tri.emplace_back(i, i, 1.0);
  for (int col = 0; col < jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      tri.emplace_back(n + it.row(), it.col(), 1.0);
    }
  }
  for (int i = 0; i < meq; ++i) tri.emplace_back(n + i, n + i, 1.0);
  if (jh != nullptr) {
    for (int col = 0; col < jh->outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(*jh, col); it; ++it) {
        tri.emplace_back(n + meq + it.row(), it.col(), 1.0);
      }
    }
    for (int i = 0; i < miq; ++i)
      tri.emplace_back(n + meq + i, n + meq + i, 1.0);
  }
  Eigen::SparseMatrix<double> pattern(dim, dim);
  pattern.setFromTriplets(tri.begin(), tri.end());
  pattern.makeCompressed();
  return pattern;
}

bool select_augmented_by_symbolic_cost(
    const Eigen::SparseMatrix<double>& condensed_h,
    const Eigen::SparseMatrix<double>& h,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::SparseMatrix<double>& jh,
    NewtonStructureProfile& profile) {
  Eigen::SparseMatrix<double> condensed =
      assemble_lower_newton_pattern(condensed_h, jg, nullptr);
  Eigen::SparseMatrix<double> augmented =
      assemble_lower_newton_pattern(h, jg, &jh);
  profile.condensed_dimension = static_cast<int>(condensed.rows());
  profile.augmented_dimension = static_cast<int>(augmented.rows());
  profile.condensed_nonzeros = static_cast<int>(condensed.nonZeros());
  profile.augmented_nonzeros = static_cast<int>(augmented.nonZeros());

  CholmodLDLT condensed_symbolic;
  CholmodLDLT augmented_symbolic;
  condensed_symbolic.set_simplicial(true);
  augmented_symbolic.set_simplicial(true);
  const bool condensed_ok = condensed_symbolic.analyze(
      condensed.rows(), condensed.outerIndexPtr(), condensed.innerIndexPtr(),
      condensed.valuePtr(), condensed.nonZeros());
  const bool augmented_ok = augmented_symbolic.analyze(
      augmented.rows(), augmented.outerIndexPtr(), augmented.innerIndexPtr(),
      augmented.valuePtr(), augmented.nonZeros());
  if (!condensed_ok || !augmented_ok) return false;
  profile.condensed_flops = condensed_symbolic.symbolic_flops();
  profile.augmented_flops = augmented_symbolic.symbolic_flops();
  profile.condensed_lnz = condensed_symbolic.symbolic_nonzeros();
  profile.augmented_lnz = augmented_symbolic.symbolic_nonzeros();
  profile.analyzed = profile.condensed_flops > 0.0 &&
                     profile.augmented_flops > 0.0 &&
                     profile.condensed_lnz > 0.0 && profile.augmented_lnz > 0.0;
  return profile.analyzed &&
         profile.augmented_flops < profile.condensed_flops &&
         profile.augmented_lnz < profile.condensed_lnz;
}

// Triplet-assemble the augmented pattern once (values zeroed except source
// entries; the diagonal slots exist explicitly so the scatter refill can
// address them).
Eigen::SparseMatrix<double> assemble_augmented_newton_pattern(
    const Eigen::SparseMatrix<double>& h,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::SparseMatrix<double>& jh) {
  const int n = static_cast<int>(h.rows());
  const int meq = static_cast<int>(jg.rows());
  const int miq = static_cast<int>(jh.rows());
  const int dim = n + meq + miq;
  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(h.nonZeros() + 2 * jg.nonZeros() +
                                  2 * jh.nonZeros() + dim));
  for (int col = 0; col < h.outerSize(); ++col)
    for (Eigen::SparseMatrix<double>::InnerIterator it(h, col); it; ++it)
      tri.emplace_back(it.row(), it.col(), it.value());
  for (int i = 0; i < n; ++i) tri.emplace_back(i, i, 0.0);
  for (int col = 0; col < jg.outerSize(); ++col)
    for (Eigen::SparseMatrix<double>::InnerIterator it(jg, col); it; ++it) {
      tri.emplace_back(n + it.row(), it.col(), it.value());
      tri.emplace_back(it.col(), n + it.row(), it.value());
    }
  for (int i = 0; i < meq; ++i) tri.emplace_back(n + i, n + i, 0.0);
  for (int col = 0; col < jh.outerSize(); ++col)
    for (Eigen::SparseMatrix<double>::InnerIterator it(jh, col); it; ++it) {
      tri.emplace_back(n + meq + it.row(), it.col(), it.value());
      tri.emplace_back(it.col(), n + meq + it.row(), it.value());
    }
  for (int i = 0; i < miq; ++i)
    tri.emplace_back(n + meq + i, n + meq + i, 0.0);
  Eigen::SparseMatrix<double> kkt(dim, dim);
  kkt.setFromTriplets(tri.begin(), tri.end());
  kkt.makeCompressed();
  return kkt;
}

void build_augmented_newton_scatter(AugmentedNewtonCache& c,
                                    const Eigen::SparseMatrix<double>& h,
                                    const Eigen::SparseMatrix<double>& jg,
                                    const Eigen::SparseMatrix<double>& jh) {
  const int n = c.n, meq = c.meq, miq = c.miq;
  const int* ko = c.kkt.kkt.outerIndexPtr();
  const int* ki = c.kkt.kkt.innerIndexPtr();
  auto find = [&](int col, int row) {
    const int* b = ki + ko[col];
    const int* e = ki + ko[col + 1];
    const int* p = std::lower_bound(b, e, row);
    return static_cast<int>(p - ki);
  };
  c.h_pos.resize(static_cast<size_t>(h.nonZeros()));
  for (int j = 0; j < h.outerSize(); ++j)
    for (int p = h.outerIndexPtr()[j]; p < h.outerIndexPtr()[j + 1]; ++p)
      c.h_pos[static_cast<size_t>(p)] = find(j, h.innerIndexPtr()[p]);
  c.jg_top.resize(static_cast<size_t>(jg.nonZeros()));
  c.jg_bot.resize(static_cast<size_t>(jg.nonZeros()));
  for (int j = 0; j < jg.outerSize(); ++j)
    for (int p = jg.outerIndexPtr()[j]; p < jg.outerIndexPtr()[j + 1]; ++p) {
      c.jg_top[static_cast<size_t>(p)] = find(j, n + jg.innerIndexPtr()[p]);
      c.jg_bot[static_cast<size_t>(p)] = find(n + jg.innerIndexPtr()[p], j);
    }
  c.jh_top.resize(static_cast<size_t>(jh.nonZeros()));
  c.jh_bot.resize(static_cast<size_t>(jh.nonZeros()));
  for (int j = 0; j < jh.outerSize(); ++j)
    for (int p = jh.outerIndexPtr()[j]; p < jh.outerIndexPtr()[j + 1]; ++p) {
      c.jh_top[static_cast<size_t>(p)] = find(j, n + meq + jh.innerIndexPtr()[p]);
      c.jh_bot[static_cast<size_t>(p)] = find(n + meq + jh.innerIndexPtr()[p], j);
    }
  c.diag_w.resize(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) c.diag_w[static_cast<size_t>(i)] = find(i, i);
  c.diag_c.resize(static_cast<size_t>(meq));
  for (int i = 0; i < meq; ++i) c.diag_c[static_cast<size_t>(i)] = find(n + i, n + i);
  c.diag_s.resize(static_cast<size_t>(miq));
  for (int i = 0; i < miq; ++i)
    c.diag_s[static_cast<size_t>(i)] = find(n + meq + i, n + meq + i);

  c.h_nnz = static_cast<int>(h.nonZeros());
  c.jg_nnz = static_cast<int>(jg.nonZeros());
  c.jh_nnz = static_cast<int>(jh.nonZeros());
  c.h_outer.assign(h.outerIndexPtr(), h.outerIndexPtr() + h.outerSize() + 1);
  c.h_inner.assign(h.innerIndexPtr(), h.innerIndexPtr() + h.nonZeros());
  c.jg_outer.assign(jg.outerIndexPtr(), jg.outerIndexPtr() + jg.outerSize() + 1);
  c.jg_inner.assign(jg.innerIndexPtr(), jg.innerIndexPtr() + jg.nonZeros());
  c.jh_outer.assign(jh.outerIndexPtr(), jh.outerIndexPtr() + jh.outerSize() + 1);
  c.jh_inner.assign(jh.innerIndexPtr(), jh.innerIndexPtr() + jh.nonZeros());
}

bool augmented_newton_structure_matches(const AugmentedNewtonCache& c,
                                        const Eigen::SparseMatrix<double>& h,
                                        const Eigen::SparseMatrix<double>& jg,
                                        const Eigen::SparseMatrix<double>& jh) {
  const int h_nnz = static_cast<int>(h.nonZeros());
  const int jg_nnz = static_cast<int>(jg.nonZeros());
  const int jh_nnz = static_cast<int>(jh.nonZeros());
  return c.h_nnz == h_nnz && c.jg_nnz == jg_nnz && c.jh_nnz == jh_nnz &&
         c.h_outer.size() == static_cast<size_t>(h.outerSize() + 1) &&
         c.jg_outer.size() == static_cast<size_t>(jg.outerSize() + 1) &&
         c.jh_outer.size() == static_cast<size_t>(jh.outerSize() + 1) &&
         std::memcmp(c.h_outer.data(), h.outerIndexPtr(),
                     (h.outerSize() + 1) * sizeof(int)) == 0 &&
         std::memcmp(c.jg_outer.data(), jg.outerIndexPtr(),
                     (jg.outerSize() + 1) * sizeof(int)) == 0 &&
         std::memcmp(c.jh_outer.data(), jh.outerIndexPtr(),
                     (jh.outerSize() + 1) * sizeof(int)) == 0 &&
         std::memcmp(c.h_inner.data(), h.innerIndexPtr(), h_nnz * sizeof(int)) == 0 &&
         std::memcmp(c.jg_inner.data(), jg.innerIndexPtr(), jg_nnz * sizeof(int)) == 0 &&
         std::memcmp(c.jh_inner.data(), jh.innerIndexPtr(), jh_nnz * sizeof(int)) == 0;
}

// Assemble [H + δ_W I, Jgᵀ, Jhᵀ; Jg, -δ_C I, 0; Jh, 0, -SM⁻¹] into the cache.
bool assemble_augmented_newton(AugmentedNewtonCache& c,
                               const Eigen::SparseMatrix<double>& h,
                               const Eigen::SparseMatrix<double>& jg,
                               const Eigen::SparseMatrix<double>& jh,
                               const Eigen::VectorXd& s,
                               const Eigen::VectorXd& mu_ineq,
                               double delta_w, double delta_c) {
  const int n = c.n, meq = c.meq, miq = c.miq;
  const bool structure_changed =
      !augmented_newton_structure_matches(c, h, jg, jh);
  if (structure_changed) {
    c.kkt.kkt = assemble_augmented_newton_pattern(h, jg, jh);
    build_augmented_newton_scatter(c, h, jg, jh);
  }
  double* v = c.kkt.kkt.valuePtr();
  std::memset(v, 0, static_cast<size_t>(c.kkt.kkt.nonZeros()) * sizeof(double));
  const double* hv = h.valuePtr();
  for (int k = 0; k < c.h_nnz; ++k)
    v[c.h_pos[static_cast<size_t>(k)]] += hv[k];
  const double* gv = jg.valuePtr();
  for (int k = 0; k < c.jg_nnz; ++k) {
    v[c.jg_top[static_cast<size_t>(k)]] += gv[k];
    v[c.jg_bot[static_cast<size_t>(k)]] += gv[k];
  }
  const double* jhv = jh.valuePtr();
  for (int k = 0; k < c.jh_nnz; ++k) {
    v[c.jh_top[static_cast<size_t>(k)]] += jhv[k];
    v[c.jh_bot[static_cast<size_t>(k)]] += jhv[k];
  }
  for (int i = 0; i < n; ++i) v[c.diag_w[static_cast<size_t>(i)]] += delta_w;
  for (int i = 0; i < meq; ++i) v[c.diag_c[static_cast<size_t>(i)]] -= delta_c;
  for (int i = 0; i < miq; ++i) {
    // -S·M⁻¹ diagonal: -s_i / μ_i (μ_i floored away from zero).
    const double mui = std::max(mu_ineq[i], kMinPositive);
    v[c.diag_s[static_cast<size_t>(i)]] -= s[i] / mui;
  }
  return !structure_changed;
}

// Solve the assembled system with the cached factor, splitting the solution
// into (dx, dλ, dμ) and applying up to 2 iterative-refinement steps.
bool solve_augmented_newton(AugmentedNewtonCache& c,
                            const Eigen::VectorXd& rhs,
                            Eigen::VectorXd& dx,
                            Eigen::VectorXd& dlambda,
                            Eigen::VectorXd& dmu) {
  if (!c.kkt.factored) return false;
  Eigen::VectorXd& sol = c.kkt.solve_solution;
  if (!c.kkt.solver || !c.kkt.solver->solve(rhs, sol) || !sol.allFinite())
    return false;
  ++c.kkt.linear_solves;
  Eigen::VectorXd& residual = c.kkt.solve_residual;
  Eigen::VectorXd& correction = c.kkt.solve_correction;
  const double rhs_scale = std::max(1.0, rhs.cwiseAbs().maxCoeff());
  constexpr double kNewtonForcingEta = 0.1;
  for (int ref = 0; ref < 2; ++ref) {
    residual = rhs - c.kkt.kkt * sol;
    const double old_error = residual.cwiseAbs().maxCoeff();
    if (old_error <= kNewtonForcingEta * rhs_scale)
      break;
    if (!c.kkt.solver->solve(residual, correction) ||
        !correction.allFinite()) {
      break;
    }
    ++c.kkt.linear_solves;
    sol += correction;
    const double new_error =
        (rhs - c.kkt.kkt * sol).cwiseAbs().maxCoeff();
    if (!std::isfinite(new_error) || new_error >= old_error) {
      sol -= correction;
      break;
    }
  }
  dx = sol.head(c.n);
  dlambda = sol.segment(c.n, c.meq);
  dmu = sol.tail(c.miq);
  return true;
}

void ensure_augmented_inertia_backend(AugmentedNewtonCache& c) {
#ifdef HACDCPF_HAVE_MUMPS
  if (dynamic_cast<MumpsSolver*>(c.kkt.solver.get()) == nullptr) {
    c.kkt.solver = std::make_unique<MumpsSolver>();
    c.kkt.pattern_analyzed = false;
    c.kkt.pattern_outer.clear();
    c.kkt.pattern_inner.clear();
  }
#elif defined(HACDCPF_HAVE_MKL_PARDISO)
  if (dynamic_cast<MKLPardisoLDLTSolver*>(c.kkt.solver.get()) == nullptr) {
    c.kkt.solver = std::make_unique<MKLPardisoLDLTSolver>();
    c.kkt.pattern_analyzed = false;
    c.kkt.pattern_outer.clear();
    c.kkt.pattern_inner.clear();
  }
#else
  (void)c;
#endif
}

bool augmented_factor_has_correct_inertia(const AugmentedNewtonCache& c) {
  if (!c.kkt.solver) return false;
  const int negative = c.kkt.solver->negative_eigenvalues();
  const int deficiency = c.kkt.solver->estimated_deficiency();
  return negative == c.meq + c.miq &&
         (deficiency < 0 || deficiency == 0);
}

// Factor + solve the augmented Newton step with Wächter–Biegler δ_W
// escalation.  The LDLT inertia must be (n, meq + miq, 0), which is exactly
// equivalent to positive curvature of the condensed Hessian on null(Jg).
bool factor_solve_augmented_newton(
    AugmentedNewtonCache& c,
    const Eigen::SparseMatrix<double>& h,
    const Eigen::SparseMatrix<double>& jg,
    const Eigen::SparseMatrix<double>& jh,
    const Eigen::VectorXd& s,
    const Eigen::VectorXd& mu_ineq,
    double& delta_w_last,
    const InertiaSettings& settings,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dlambda,
    Eigen::VectorXd& dmu) {
  ensure_augmented_inertia_backend(c);
  double delta_w = std::max(0.0, delta_w_last);
  bool delta_w_was_zero = (delta_w == 0.0);
  const double delta_c = settings.delta_c_stripe *
      std::pow(std::max(settings.mu, 1e-20), 0.25);
  for (int attempt = 0; attempt < 60; ++attempt) {
    // A Wächter-Biegler dual stripe makes the inertia certificate robust to
    // rank-deficient equality rows. It is applied from the first attempt so a
    // threshold-pivoted backend does not first factor a knowingly singular
    // zero dual block.
    const bool dual_pattern_unchanged =
        assemble_augmented_newton(c, h, jg, jh, s, mu_ineq, delta_w, delta_c);
    if (factor_current_kkt(c.kkt, c.n, c.meq + c.miq,
                           dual_pattern_unchanged) &&
        augmented_factor_has_correct_inertia(c) &&
        solve_augmented_newton(c, rhs, dx, dlambda, dmu)) {
      delta_w_last = std::max(settings.delta_w_min,
                              delta_w * settings.kappa_w_minus);
      return true;
    }
    // Wächter–Biegler δ_W schedule.
    if (delta_w_was_zero) {
      delta_w = settings.delta_w_0;
      delta_w_was_zero = false;
    } else if (attempt == 0) {
      delta_w *= settings.kappa_w_plus_first;
    } else {
      delta_w *= settings.kappa_w_plus;
    }
    if (delta_w > settings.delta_w_max) return false;
  }
  return false;
}


bool evaluate_nlp_state(const NLPModel& prob,
                        const Eigen::VectorXd& x,
                        const std::vector<int>& lb_cols,
                        const std::vector<int>& ub_cols,
                        NLPState& state,
                        std::string& status) {
  state.obj_orig = objective_value(prob, x);
  state.obj_eff = (prob.sense == Sense::Maximize) ? -state.obj_orig : state.obj_orig;
  if (!std::isfinite(state.obj_orig) || !std::isfinite(state.obj_eff)) {
    status = "NLP objective became non-finite";
    return false;
  }
  if (!build_objective_gradient(prob, x, state.grad, status)) return false;
  if (!build_equalities(prob, x, state.g, state.jg, status)) return false;
  if (!build_inequalities(prob, x, lb_cols, ub_cols, state.h, state.jh,
                          state.n_nonlinear_ineq, status)) return false;
  return true;
}

bool evaluate_nlp_values(const NLPModel& prob,
                         const Eigen::VectorXd& x,
                         const std::vector<int>& lb_cols,
                         const std::vector<int>& ub_cols,
                         NLPState& state,
                         std::string& status) {
  state.obj_orig = objective_value(prob, x);
  state.obj_eff =
      (prob.sense == Sense::Maximize) ? -state.obj_orig : state.obj_orig;
  if (!std::isfinite(state.obj_orig) || !std::isfinite(state.obj_eff)) {
    status = "NLP objective became non-finite";
    return false;
  }

  if (prob.g) {
    prob.g(x, state.g);
    if (!state.g.allFinite()) {
      status = "NLP equality callback returned non-finite values";
      return false;
    }
  } else {
    state.g = Eigen::VectorXd::Zero(0);
  }

  Eigen::VectorXd h_nonlinear = Eigen::VectorXd::Zero(0);
  if (prob.h) {
    prob.h(x, h_nonlinear);
    if (!h_nonlinear.allFinite()) {
      status = "NLP inequality callback returned non-finite values";
      return false;
    }
  }
  state.n_nonlinear_ineq = static_cast<int>(h_nonlinear.size());
  state.h.resize(state.n_nonlinear_ineq +
                 static_cast<int>(lb_cols.size() + ub_cols.size()));
  if (state.n_nonlinear_ineq > 0) {
    state.h.head(state.n_nonlinear_ineq) = h_nonlinear;
  }
  int row = state.n_nonlinear_ineq;
  for (int col : lb_cols) {
    state.h[row++] = prob.vars[static_cast<size_t>(col)].lb - x[col];
  }
  for (int col : ub_cols) {
    state.h[row++] = x[col] - prob.vars[static_cast<size_t>(col)].ub;
  }
  if (!state.h.allFinite()) {
    status = "Split inequality construction returned non-finite values";
    return false;
  }
  return true;
}

void interiorize_initial_point(const std::vector<VariableMeta>& vars, Eigen::VectorXd& x);
void initialize_barrier_state(const Eigen::VectorXd& h,
                              Eigen::VectorXd& s,
                              Eigen::VectorXd& mu);

void recenter_barrier_state(const NLPModel& prob,
                            const Eigen::VectorXd& h,
                            Eigen::VectorXd& x,
                            Eigen::VectorXd& s,
                            Eigen::VectorXd& mu,
                            DiagonalQNState* qn_state) {
  interiorize_initial_point(prob.vars, x);
  initialize_barrier_state(h, s, mu);
  if (qn_state && qn_state->active && qn_state->diag.size() == x.size()) {
    qn_state->diag = qn_state->diag.cwiseMax(Eigen::VectorXd::Constant(x.size(), 1e-3));
  }
}

void interiorize_initial_point(const std::vector<VariableMeta>& vars, Eigen::VectorXd& x) {
  const int n = static_cast<int>(vars.size());
  if (x.size() != n) {
    x = Eigen::VectorXd::Zero(n);
  }

  for (int i = 0; i < n; ++i) {
    const double lb = vars[static_cast<size_t>(i)].lb;
    const double ub = vars[static_cast<size_t>(i)].ub;
    if (!std::isfinite(x[i])) x[i] = 0.0;

    const bool has_lb = is_effectively_finite(lb);
    const bool has_ub = is_effectively_finite(ub);
    if (has_lb && has_ub) {
      const double width = ub - lb;
      const double scale = std::max({1.0, std::abs(lb), std::abs(ub)});
      const double margin = std::max(
          2.0 * kMinPositive,
          16.0 * std::numeric_limits<double>::epsilon() * scale);
      if (width <= 2.0 * margin) {
        x[i] = 0.5 * (lb + ub);
        continue;
      }
      x[i] = std::min(ub - margin, std::max(lb + margin, x[i]));
    } else if (has_lb) {
      const double margin = std::max(
          2.0 * kMinPositive,
          16.0 * std::numeric_limits<double>::epsilon() *
              std::max(1.0, std::abs(lb)));
      x[i] = std::max(x[i], lb + margin);
    } else if (has_ub) {
      const double margin = std::max(
          2.0 * kMinPositive,
          16.0 * std::numeric_limits<double>::epsilon() *
              std::max(1.0, std::abs(ub)));
      x[i] = std::min(x[i], ub - margin);
    }
  }
}

struct ResidualSummary {
  double primal_feas{0.0};
  double dual_feas{0.0};
  double complementarity{0.0};
  double merit{0.0};
};

struct IterateSnapshot {
  bool valid{false};
  Eigen::VectorXd x;
  Eigen::VectorXd s;
  Eigen::VectorXd lambda;
  Eigen::VectorXd mu;
  double objective{0.0};
  ResidualSummary residuals;
  int iteration{0};
};

ResidualSummary summarize_residuals(const Eigen::VectorXd& r_dual,
                                    const Eigen::VectorXd& r_eq,
                                    const Eigen::VectorXd& r_ineq,
                                    const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& s,
                                    const Eigen::VectorXd& lambda,
                                    const Eigen::VectorXd& mu);

struct TrialPoint {
  bool valid{false};
  Eigen::VectorXd x;
  Eigen::VectorXd s;
  Eigen::VectorXd lambda;
  Eigen::VectorXd mu;
  NLPState state;
  ResidualSummary residuals;
};

bool evaluate_trial_point(const NLPModel& prob,
                          const std::vector<int>& lb_cols,
                          const std::vector<int>& ub_cols,
                          const Eigen::VectorXd& x_trial,
                          const Eigen::VectorXd& s_trial,
                          const Eigen::VectorXd& lambda_trial,
                          const Eigen::VectorXd& mu_trial,
                          std::string& status,
                          TrialPoint& trial) {
  if ((s_trial.size() > 0 && (s_trial.array() <= kMinPositive).any()) ||
      (mu_trial.size() > 0 && (mu_trial.array() <= kMinPositive).any())) {
    return false;
  }

  NLPState trial_state;
  if (!evaluate_nlp_state(prob, x_trial, lb_cols, ub_cols, trial_state, status)) {
    return false;
  }

  const Eigen::VectorXd r_dual_trial =
      trial_state.grad + trial_state.jg.transpose() * lambda_trial +
      trial_state.jh.transpose() * mu_trial;
  const Eigen::VectorXd r_eq_trial = trial_state.g;
  const Eigen::VectorXd r_ineq_trial = trial_state.h + s_trial;

  trial.valid = true;
  trial.x = x_trial;
  trial.s = s_trial;
  trial.lambda = lambda_trial;
  trial.mu = mu_trial;
  trial.state = std::move(trial_state);
  trial.residuals = summarize_residuals(
      r_dual_trial, r_eq_trial, r_ineq_trial,
      trial.x, trial.s, trial.lambda, trial.mu);
  return true;
}

bool evaluate_filter_trial_values(const NLPModel& prob,
                                  const std::vector<int>& lb_cols,
                                  const std::vector<int>& ub_cols,
                                  const Eigen::VectorXd& x_trial,
                                  const Eigen::VectorXd& s_trial,
                                  const Eigen::VectorXd& lambda_trial,
                                  const Eigen::VectorXd& mu_trial,
                                  std::string& status,
                                  TrialPoint& trial) {
  if ((s_trial.size() > 0 && (s_trial.array() <= kMinPositive).any()) ||
      (mu_trial.size() > 0 && (mu_trial.array() <= kMinPositive).any())) {
    return false;
  }
  NLPState trial_state;
  if (!evaluate_nlp_values(prob, x_trial, lb_cols, ub_cols, trial_state,
                           status)) {
    return false;
  }
  if (trial_state.g.size() != lambda_trial.size() ||
      trial_state.h.size() != s_trial.size() ||
      trial_state.h.size() != mu_trial.size()) {
    status = "NLP constraint callback changed dimension during line search";
    return false;
  }
  trial.valid = true;
  trial.x = x_trial;
  trial.s = s_trial;
  trial.lambda = lambda_trial;
  trial.mu = mu_trial;
  trial.state = std::move(trial_state);
  return true;
}

bool sufficient_primal_dual_progress(const ResidualSummary& current,
                                     const ResidualSummary& trial,
                                     double alpha_primal,
                                     double alpha_dual) {
  const double alpha = std::max(1e-8, std::min(alpha_primal, alpha_dual));
  const double merit_target =
      current.merit * (1.0 - 1e-4 * alpha) + 1e-10;
  const double primal_target =
      current.primal_feas * (1.0 - 5e-3 * alpha) + 1e-12;
  const double dual_target =
      current.dual_feas * (1.0 - 5e-3 * alpha) + 1e-12;
  const double comp_target =
      current.complementarity * (1.0 - 5e-3 * alpha) + 1e-12;

  const bool merit_ok = trial.merit <= merit_target;
  const bool primal_ok = trial.primal_feas <= primal_target;
  const bool dual_ok = trial.dual_feas <= dual_target;
  const bool comp_ok = trial.complementarity <= comp_target;
  const int improved = static_cast<int>(primal_ok) +
                       static_cast<int>(dual_ok) +
                       static_cast<int>(comp_ok);

  return merit_ok || improved >= 2 ||
         (primal_ok && trial.dual_feas <= current.dual_feas + 1e-10) ||
         (dual_ok && trial.primal_feas <= current.primal_feas + 1e-10);
}

void initialize_barrier_state(const Eigen::VectorXd& h,
                              Eigen::VectorXd& s,
                              Eigen::VectorXd& mu) {
  if (h.size() == 0) {
    s = Eigen::VectorXd::Zero(0);
    mu = Eigen::VectorXd::Zero(0);
    return;
  }

  s.resize(h.size());
  mu.resize(h.size());
  for (int i = 0; i < h.size(); ++i) {
    s[i] = (h[i] >= 0.0) ? std::max(h[i], 1.0) : std::max(-h[i], 1e-2);
    mu[i] = std::max(1.0 / s[i], 1e-2);
  }
}

ResidualSummary summarize_residuals(const Eigen::VectorXd& r_dual,
                                    const Eigen::VectorXd& r_eq,
                                    const Eigen::VectorXd& r_ineq,
                                    const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& s,
                                    const Eigen::VectorXd& lambda,
                                    const Eigen::VectorXd& mu) {
  ResidualSummary out;
  const double scale_x = std::max(inf_norm(x), inf_norm(s));
  out.primal_feas = std::max(inf_norm(r_eq), inf_norm(r_ineq));
  out.dual_feas = inf_norm(r_dual);
  out.complementarity = (s.size() == 0)
      ? 0.0
      : inf_norm(s.cwiseProduct(mu));
  const double scaled_primal = out.primal_feas / (1.0 + scale_x);
  const double scaled_dual = out.dual_feas /
      (1.0 + std::max(inf_norm(lambda), inf_norm(mu)));
  const double scaled_complementarity = out.complementarity /
      (1.0 + inf_norm(mu));
  // Scaling is useful for globalization and best-iterate ranking, but KKT
  // termination and reported residuals above remain absolute.
  out.merit = std::max(
      {scaled_primal, scaled_dual, scaled_complementarity});
  return out;
}

// ------------------------- Filter-driver helpers --------------------------
// These support the Wächter–Biegler primal–dual filter line-search path. The
// Merit path is untouched.

double compute_theta(const Eigen::VectorXd& g,
                     const Eigen::VectorXd& h,
                     const Eigen::VectorXd& s) {
  double th = 0.0;
  if (g.size() > 0) th += g.lpNorm<1>();
  if (h.size() > 0 && s.size() == h.size()) {
    th += (h + s).lpNorm<1>();
  }
  return th;
}

double compute_barrier_phi(double f_value,
                           double mu_bar,
                           const Eigen::VectorXd& s) {
  double phi = f_value;
  for (int i = 0; i < s.size(); ++i) {
    phi -= mu_bar * std::log(std::max(s[i], kMinPositive));
  }
  return phi;
}

double barrier_descent_slope(const Eigen::VectorXd& grad,
                             const Eigen::VectorXd& dx,
                             const Eigen::VectorXd& s,
                             const Eigen::VectorXd& ds,
                             double mu_bar) {
  // ∇φ_μ · [dx; ds] = ∇f · dx − μ_bar Σ ds_i / s_i.
  double slope = grad.dot(dx);
  for (int i = 0; i < s.size(); ++i) {
    slope -= mu_bar * ds[i] / std::max(s[i], kMinPositive);
  }
  return slope;
}

struct FilterSolveOutcome {
  bool converged{false};
  int iterations{0};
  std::string status{"Max iterations reached"};
  Eigen::VectorXd x;
  Eigen::VectorXd s;
  Eigen::VectorXd lambda;
  Eigen::VectorXd mu_ineq;
  int n_nonlinear_ineq{0};
  ResidualSummary initial_residuals{};
  ResidualSummary final_residuals{};
  double objective{0.0};
  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  NewtonStructureProfile newton_profile;
  int symbolic_analyses{0};
  int numeric_factorizations{0};
  int linear_solves{0};
  int primary_factorizations{0};
  int inertia_retry_factorizations{0};
  int inertia_certificate_factorizations{0};
  int active_set_polish_factorizations{0};
};

struct ActiveSetPolishOutcome {
  bool converged{false};
  Eigen::VectorXd x;
  Eigen::VectorXd s;
  Eigen::VectorXd lambda;
  Eigen::VectorXd mu;
  NLPState state;
  ResidualSummary residuals;
  int iterations{0};
  int symbolic_analyses{0};
  int numeric_factorizations{0};
  int linear_solves{0};
};

Eigen::SparseMatrix<double> stack_active_jacobian(
    const NLPState& state, const std::vector<int>& active) {
  const int meq = static_cast<int>(state.jg.rows());
  const int n = static_cast<int>(state.jg.cols());
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(state.jg.nonZeros()) +
                   static_cast<std::size_t>(state.jh.nonZeros()));
  for (int col = 0; col < state.jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.jg, col); it;
         ++it) {
      triplets.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int k = 0; k < static_cast<int>(active.size()); ++k) {
    const int source_row = active[static_cast<std::size_t>(k)];
    for (int col = 0; col < state.jh.outerSize(); ++col) {
      const double value = state.jh.coeff(source_row, col);
      if (value != 0.0) {
        triplets.emplace_back(meq + k, col, value);
      }
    }
  }
  Eigen::SparseMatrix<double> jacobian(
      meq + static_cast<int>(active.size()), n);
  jacobian.setFromTriplets(triplets.begin(), triplets.end());
  jacobian.makeCompressed();
  return jacobian;
}

bool select_independent_active_rows(
    const NLPState& state, const Eigen::VectorXd& s,
    const Eigen::VectorXd& mu, double path_scale,
    std::vector<int>& active) {
  const int n = static_cast<int>(state.jg.cols());
  std::vector<Eigen::VectorXd> row_basis;
  row_basis.reserve(static_cast<std::size_t>(n));
  const auto append_if_independent = [&](Eigen::VectorXd row) {
    const double row_norm = row.norm();
    if (!(row_norm > 0.0) || !std::isfinite(row_norm)) return false;
    row /= row_norm;
    // Two-pass modified Gram-Schmidt is sufficient here because the selected
    // rows only define a crossover working set; the sparse LDLT inertia and
    // linear-residual checks remain the authoritative numerical certificates.
    for (int pass = 0; pass < 2; ++pass) {
      for (const Eigen::VectorXd& basis_row : row_basis) {
        row.noalias() -= basis_row.dot(row) * basis_row;
      }
    }
    const double residual_norm = row.norm();
    const double rank_margin = std::sqrt(
        std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(1, n)));
    if (!(residual_norm > rank_margin)) return false;
    row_basis.push_back(row / residual_norm);
    return true;
  };

  for (int row = 0; row < state.jg.rows(); ++row) {
    Eigen::VectorXd dense = Eigen::VectorXd::Zero(n);
    for (int col = 0; col < state.jg.outerSize(); ++col) {
      dense[col] = state.jg.coeff(row, col);
    }
    if (!append_if_independent(std::move(dense))) {
      return false;
    }
  }

  std::stable_sort(active.begin(), active.end(), [&](int lhs, int rhs) {
    const auto confidence = [&](int row) {
      return std::min(mu[row] / path_scale,
                      path_scale / std::max(s[row], kMinPositive));
    };
    return confidence(lhs) > confidence(rhs);
  });
  std::vector<int> independent;
  independent.reserve(active.size());
  for (int source_row : active) {
    if (static_cast<int>(row_basis.size()) >= n) break;
    Eigen::VectorXd dense = Eigen::VectorXd::Zero(n);
    for (int col = 0; col < state.jh.outerSize(); ++col) {
      dense[col] = state.jh.coeff(source_row, col);
    }
    if (append_if_independent(std::move(dense))) {
      independent.push_back(source_row);
    }
  }
  active = std::move(independent);
  return !active.empty();
}

// Crossover from the central path to the limiting active-set KKT system.
// Under strict complementarity, active slacks are O(mu_bar) with multipliers
// bounded away from zero, while inactive multipliers are O(mu_bar) with
// slacks bounded away from zero.  A sqrt(mu)-scale separator therefore
// identifies only constraints for which both sides of that asymptotic split
// agree.  This routine is deliberately a certificate path: it never publishes
// a point unless the full, unperturbed NLP KKT residuals pass the user gates.
bool try_active_set_kkt_polish(
    const NLPModel& prob, const IPMOptions& opt,
    const std::vector<int>& lb_cols, const std::vector<int>& ub_cols,
    const Eigen::VectorXd& x_start, const Eigen::VectorXd& s_start,
    const Eigen::VectorXd& lambda_start, const Eigen::VectorXd& mu_start,
    const ResidualSummary& start_residuals, ActiveSetPolishOutcome& out) {
  if (!prob.lagrangian_hess || x_start.size() == 0 ||
      s_start.size() != mu_start.size() ||
      start_residuals.primal_feas >
          std::max(1e-7, 10.0 * opt.tol_primal) ||
      start_residuals.dual_feas > opt.tol_dual ||
      start_residuals.complementarity <= opt.tol_complementarity) {
    return false;
  }

  const double path_scale = std::sqrt(std::max(
      {opt.tol_complementarity, start_residuals.complementarity,
       64.0 * std::numeric_limits<double>::epsilon()}));
  const double slack_threshold = 10.0 * path_scale;
  const double multiplier_threshold = 0.1 * path_scale;
  std::vector<int> active;
  active.reserve(static_cast<std::size_t>(s_start.size()));
  for (int i = 0; i < s_start.size(); ++i) {
    if (s_start[i] <= slack_threshold &&
        mu_start[i] >= multiplier_threshold) {
      active.push_back(i);
    }
  }
  if (opt.verbose) {
    std::cerr << "[NativeIPM] active-set polish: active=" << active.size()
              << "/" << s_start.size() << ", separator=" << path_scale
              << ", start=(" << start_residuals.primal_feas << ", "
              << start_residuals.dual_feas << ", "
              << start_residuals.complementarity << ")\n";
  }
  if (active.empty()) return false;

  NLPState selection_state;
  std::string selection_status;
  if (!evaluate_nlp_state(prob, x_start, lb_cols, ub_cols,
                          selection_state, selection_status)) {
    return false;
  }
  const std::vector<int> active_candidates = active;

  Eigen::VectorXd x = x_start;
  Eigen::VectorXd lambda = lambda_start;
  Eigen::VectorXd mu = Eigen::VectorXd::Zero(mu_start.size());

  SparseInertiaKKTCache polish_cache;
  polish_cache.preferred_free_columns = prob.equality_free_columns;
  polish_cache.augmented.refinement_tolerance = 1e-11;
  const auto record_polish_work = [&]() {
    out.symbolic_analyses = polish_cache.augmented.symbolic_analyses;
    out.numeric_factorizations =
        polish_cache.augmented.numeric_factorizations;
    out.linear_solves = polish_cache.augmented.linear_solves;
  };
  double delta_w_last = 0.0;
  const Eigen::VectorXd dual_contribution =
      selection_state.jg.transpose() * lambda_start +
      selection_state.jh.transpose() * mu_start;
  const Eigen::SparseMatrix<double> identity =
      diagonal_sparse(Eigen::VectorXd::Ones(x.size()));
  InertiaSettings projection_settings;
  projection_settings.mu = std::max(opt.tol_complementarity,
                                     start_residuals.complementarity);
  const int selection_meq = static_cast<int>(selection_state.g.size());
  std::vector<unsigned char> excluded(
      static_cast<std::size_t>(mu_start.size()), 0);
  bool projection_accepted = false;
  for (std::size_t projection_attempt = 0;
       projection_attempt <= active_candidates.size(); ++projection_attempt) {
    active.clear();
    for (int row : active_candidates) {
      if (excluded[static_cast<std::size_t>(row)] == 0) {
        active.push_back(row);
      }
    }
    const std::size_t eligible_count = active.size();
    if (!select_independent_active_rows(selection_state, s_start, mu_start,
                                        path_scale, active)) {
      break;
    }
    if (opt.verbose && active.size() != eligible_count) {
      std::cerr << "[NativeIPM] active-set rank filter: retained="
                << active.size() << "/" << eligible_count << '\n';
    }
    const Eigen::SparseMatrix<double> selection_jacobian =
        stack_active_jacobian(selection_state, active);
    InertiaStatus projection_inertia;
    delta_w_last = 0.0;
    if (!factor_kkt_inertia_corrected_sparse(
            identity, selection_jacobian, projection_settings, delta_w_last,
            polish_cache, projection_inertia)) {
      break;
    }
    Eigen::VectorXd projection_rhs(x.size() + selection_jacobian.rows());
    projection_rhs.head(x.size()) = dual_contribution;
    projection_rhs.tail(selection_jacobian.rows()).setZero();
    Eigen::VectorXd projection_remainder;
    Eigen::VectorXd projected_duals;
    if (!solve_kkt_inertia_corrected_sparse(
            polish_cache, projection_rhs, projection_remainder,
            projected_duals)) {
      break;
    }

    int most_negative_position = -1;
    double most_negative_value = -path_scale;
    for (int k = 0; k < static_cast<int>(active.size()); ++k) {
      const double value = projected_duals[selection_meq + k];
      if (value < most_negative_value) {
        most_negative_value = value;
        most_negative_position = k;
      }
    }
    if (most_negative_position >= 0) {
      const int rejected_row =
          active[static_cast<std::size_t>(most_negative_position)];
      excluded[static_cast<std::size_t>(rejected_row)] = 1;
      if (opt.verbose) {
        std::cerr << "[NativeIPM] active-set dual-sign filter: remove row="
                  << rejected_row << ", multiplier=" << most_negative_value
                  << '\n';
      }
      continue;
    }

    lambda = projected_duals.head(selection_meq);
    mu.setZero();
    for (int k = 0; k < static_cast<int>(active.size()); ++k) {
      mu[active[static_cast<std::size_t>(k)]] =
          std::max(0.0, projected_duals[selection_meq + k]);
    }
    projection_accepted = true;
    break;
  }
  if (!projection_accepted) {
    if (opt.verbose) {
      std::cerr << "[NativeIPM] active-set polish rejected: no full-rank "
                   "dual-feasible working set\n";
    }
    record_polish_work();
    return false;
  }
  delta_w_last = 0.0;
  constexpr int kMaxPolishIterations = 8;
  for (int iteration = 0; iteration < kMaxPolishIterations; ++iteration) {
    NLPState state;
    std::string status;
    if (!evaluate_nlp_state(prob, x, lb_cols, ub_cols, state, status)) {
      record_polish_work();
      return false;
    }
    Eigen::VectorXd s(state.h.size());
    for (int i = 0; i < s.size(); ++i) {
      s[i] = std::max(-state.h[i], kMinPositive);
    }
    const Eigen::VectorXd r_dual =
        state.grad + state.jg.transpose() * lambda +
        state.jh.transpose() * mu;
    const ResidualSummary residuals = summarize_residuals(
        r_dual, state.g, state.h + s, x, s, lambda, mu);
    if (opt.verbose) {
      std::cerr << "[NativeIPM] active-set polish iter=" << iteration
                << ": primal=" << residuals.primal_feas
                << ", dual=" << residuals.dual_feas
                << ", complementarity=" << residuals.complementarity
                << '\n';
    }
    if (residuals.primal_feas <= opt.tol_primal &&
        residuals.dual_feas <= opt.tol_dual &&
        residuals.complementarity <= opt.tol_complementarity) {
      out.converged = true;
      out.x = x;
      out.s = std::move(s);
      out.lambda = lambda;
      out.mu = mu;
      out.state = std::move(state);
      out.residuals = residuals;
      out.iterations = iteration;
      record_polish_work();
      return true;
    }

    const int meq = static_cast<int>(state.g.size());
    const Eigen::SparseMatrix<double> active_jacobian =
        stack_active_jacobian(state, active);
    Eigen::VectorXd active_constraints(meq + static_cast<int>(active.size()));
    if (meq > 0) active_constraints.head(meq) = state.g;
    for (int k = 0; k < static_cast<int>(active.size()); ++k) {
      active_constraints[meq + k] =
          state.h[active[static_cast<std::size_t>(k)]];
    }

    const Eigen::VectorXd mu_nonlinear =
        nonlinear_inequality_multipliers(mu, state.n_nonlinear_ineq);
    Eigen::SparseMatrix<double> hessian;
    if (!build_lagrangian_hessian(prob, x, lambda, &mu_nonlinear,
                                  hessian, status)) {
      record_polish_work();
      return false;
    }
    InertiaSettings settings;
    settings.mu = std::max(opt.tol_complementarity,
                           start_residuals.complementarity);
    InertiaStatus inertia;
    if (!factor_kkt_inertia_corrected_sparse(
            hessian, active_jacobian, settings, delta_w_last,
            polish_cache, inertia)) {
      if (opt.verbose) {
        std::cerr << "[NativeIPM] active-set polish rejected: KKT rank/inertia "
                     "certificate failed (rows="
                  << active_jacobian.rows() << ", cols="
                  << active_jacobian.cols() << ")\n";
      }
      record_polish_work();
      return false;
    }
    Eigen::VectorXd rhs(x.size() + active_constraints.size());
    rhs << -r_dual, -active_constraints;
    Eigen::VectorXd dx;
    Eigen::VectorXd dy;
    if (!solve_kkt_inertia_corrected_sparse(polish_cache, rhs, dx, dy) ||
        dy.size() != active_constraints.size()) {
      if (opt.verbose) {
        std::cerr << "[NativeIPM] active-set polish rejected: certified KKT "
                     "solve residual failed\n";
      }
      record_polish_work();
      return false;
    }

    Eigen::VectorXd dlambda = dy.head(meq);
    Eigen::VectorXd dmu = Eigen::VectorXd::Zero(mu.size());
    for (int k = 0; k < static_cast<int>(active.size()); ++k) {
      dmu[active[static_cast<std::size_t>(k)]] = dy[meq + k];
    }
    double alpha_dual = 1.0;
    for (int row : active) {
      if (dmu[row] < 0.0) {
        alpha_dual = std::min(alpha_dual, -0.995 * mu[row] / dmu[row]);
      }
    }

    const double current_gate = std::max(
        {residuals.primal_feas / std::max(opt.tol_primal, 1e-16),
         residuals.dual_feas / std::max(opt.tol_dual, 1e-16),
         residuals.complementarity /
             std::max(opt.tol_complementarity, 1e-16)});
    bool accepted = false;
    double alpha = std::min(1.0, alpha_dual);
    for (int backtrack = 0; backtrack < 16; ++backtrack, alpha *= 0.5) {
      const Eigen::VectorXd trial_x = x + alpha * dx;
      const Eigen::VectorXd trial_lambda = lambda + alpha * dlambda;
      const Eigen::VectorXd trial_mu = mu + alpha * dmu;
      if ((trial_mu.array() < 0.0).any()) continue;
      NLPState trial_state;
      if (!evaluate_nlp_state(prob, trial_x, lb_cols, ub_cols,
                              trial_state, status)) {
        continue;
      }
      Eigen::VectorXd trial_s(trial_state.h.size());
      for (int i = 0; i < trial_s.size(); ++i) {
        trial_s[i] = std::max(-trial_state.h[i], kMinPositive);
      }
      const Eigen::VectorXd trial_r_dual =
          trial_state.grad + trial_state.jg.transpose() * trial_lambda +
          trial_state.jh.transpose() * trial_mu;
      const ResidualSummary trial_residuals = summarize_residuals(
          trial_r_dual, trial_state.g, trial_state.h + trial_s,
          trial_x, trial_s, trial_lambda, trial_mu);
      const double trial_gate = std::max(
          {trial_residuals.primal_feas / std::max(opt.tol_primal, 1e-16),
           trial_residuals.dual_feas / std::max(opt.tol_dual, 1e-16),
           trial_residuals.complementarity /
               std::max(opt.tol_complementarity, 1e-16)});
      if (trial_gate < current_gate * (1.0 - 1e-4 * alpha)) {
        x = trial_x;
        lambda = trial_lambda;
        mu = trial_mu;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      if (opt.verbose) {
        std::cerr << "[NativeIPM] active-set polish rejected: full KKT gate "
                     "did not decrease\n";
      }
      record_polish_work();
      return false;
    }
  }
  record_polish_work();
  return false;
}

FilterSolveOutcome solve_nlp_filter_impl(const NLPModel& prob,
                                         const IPMOptions& opt) {
  FilterSolveOutcome result;
  const int n = static_cast<int>(prob.vars.size());
  Eigen::VectorXd x = (prob.x0.size() == n) ? prob.x0 : Eigen::VectorXd::Zero(n);
  interiorize_initial_point(prob.vars, x);

  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  lb_cols.reserve(static_cast<size_t>(n));
  ub_cols.reserve(static_cast<size_t>(n));
  for (int j = 0; j < n; ++j) {
    if (is_effectively_finite(prob.vars[static_cast<size_t>(j)].lb)) lb_cols.push_back(j);
    if (is_effectively_finite(prob.vars[static_cast<size_t>(j)].ub)) ub_cols.push_back(j);
  }
  result.lb_cols = lb_cols;
  result.ub_cols = ub_cols;

  NLPState state;
  std::string eval_status;
  if (!evaluate_nlp_state(prob, x, lb_cols, ub_cols, state, eval_status)) {
    result.status = eval_status;
    result.x = x;
    return result;
  }

  // Slacks are initialized strictly positive so the barrier log is defined.
  Eigen::VectorXd s(state.h.size());
  for (int i = 0; i < s.size(); ++i) {
    s[i] = std::max(-state.h[i], 1e-2);
  }

  const double effective_mu_min = std::max(
      opt.mu_min, 0.1 * std::max(opt.tol_complementarity, 0.0));
  double mu_bar = std::max(effective_mu_min, opt.mu_init);

  // Primal–dual multiplier initialization.
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(state.g.size());
  const bool has_equality_dual_start =
      opt.equality_dual_start.size() == lambda.size() &&
      opt.equality_dual_start.allFinite();
  const bool has_inequality_dual_start =
      opt.inequality_dual_start.size() == state.h.size() &&
      opt.inequality_dual_start.allFinite() &&
      (opt.inequality_dual_start.array() > 0.0).all();
  if (opt.slack_start.size() == s.size() && opt.slack_start.allFinite() &&
      (opt.slack_start.array() > 0.0).all()) {
    s = opt.slack_start.cwiseMax(
        Eigen::VectorXd::Constant(s.size(), 2.0 * kMinPositive));
  }
  Eigen::VectorXd mu_ineq(state.h.size());
  int representability_floor_clamps = 0;
  int legacy_absolute_floor_hits = 0;
  for (int i = 0; i < mu_ineq.size(); ++i) {
    const double central_multiplier =
        mu_bar / std::max(s[i], kMinPositive);
    legacy_absolute_floor_hits += static_cast<int>(central_multiplier < 1e-4);
    representability_floor_clamps +=
        static_cast<int>(central_multiplier < 2.0 * kMinPositive);
    // Preserve S*mu = mu_bar for model nonlinearities whenever representable.
    // Generated box rows retain the historical bound-dual stabilization;
    // unlike physical nonlinear slacks, their scale is the variable
    // coordinate itself and the floor prevents a cold bound dual from being
    // numerically absent.
    mu_ineq[i] = i < state.n_nonlinear_ineq
        ? std::max(central_multiplier, 2.0 * kMinPositive)
        : std::clamp(central_multiplier, 1e-4, 1e4);
  }
  if (has_inequality_dual_start) {
    mu_ineq = opt.inequality_dual_start.cwiseMax(
        Eigen::VectorXd::Constant(mu_ineq.size(), 2.0 * kMinPositive));
  }
  if (has_equality_dual_start) {
    lambda = opt.equality_dual_start;
  } else if (opt.least_square_init_duals &&
             (prob.hess || prob.lagrangian_hess)) {
    initialize_equality_duals_least_squares(
        state, mu_ineq, opt.constr_mult_init_max, lambda);
  }

  if (opt.verbose && !has_inequality_dual_start && s.size() > 0) {
    const int nonlinear_count = state.n_nonlinear_ineq;
    const int lower_count = static_cast<int>(lb_cols.size());
    const int upper_count = static_cast<int>(ub_cols.size());
    const ProductDistribution nonlinear =
        summarize_products(s, mu_ineq, 0, nonlinear_count);
    const ProductDistribution lower =
        summarize_products(s, mu_ineq, nonlinear_count, lower_count);
    const ProductDistribution upper = summarize_products(
        s, mu_ineq, nonlinear_count + lower_count, upper_count);
    const auto print_distribution = [](const char* name,
                                       const ProductDistribution& values) {
      std::cerr << ' ' << name << "=(n=" << values.count
                << ",min=" << values.minimum
                << ",median=" << values.median
                << ",max=" << values.maximum << ')';
    };
    std::cerr << "[NativeIPM] initial complementarity: mu_bar=" << mu_bar
              << ", representability_floor_clamps="
              << representability_floor_clamps
              << ", legacy_1e-4_floor_hits=" << legacy_absolute_floor_hits;
    print_distribution("nonlinear", nonlinear);
    print_distribution("lower", lower);
    print_distribution("upper", upper);
    std::cerr << '\n' << std::flush;
  }

  Filter filter;
  const double theta0 = compute_theta(state.g, state.h, s);
  filter.reset_with_theta_upper_bound(1e4 * std::max(1.0, theta0));

  // When no analytical Hessian is provided, use the evolving sparse
  // quasi-Newton approximation instead of freezing a finite-difference
  // Hessian at the initial point.
  DiagonalQNState filter_qn_state;
  if (!prob.hess && !prob.lagrangian_hess) {
    initialize_quasi_newton_state(n, filter_qn_state);
    filter_qn_state.sparse_block_size = std::max(2, opt.qn_sparse_block_size);
    filter_qn_state.max_blocks = std::max(1, opt.qn_max_blocks);
  }

  double delta_w_last = 0.0;
  SparseInertiaKKTCache kkt_cache;
  kkt_cache.preferred_free_columns = prob.equality_free_columns;
  kkt_cache.augmented.refinement_tolerance = 0.1;
  SparseKKTCache regularized_kkt_cache;
  regularized_kkt_cache.refinement_tolerance = 0.1;
  // Augmented (uncondensed) Newton path cache — persists across iterations
  // so the sparsity pattern/scatter map is built once per problem.
  AugmentedNewtonCache augmented_cache;
  NewtonStructureProfile newton_profile;
  bool newton_formulation_selected = false;
  int retired_symbolic_analyses = 0;
  int retired_numeric_factorizations = 0;
  int retired_linear_solves = 0;
  int inertia_retry_factorizations = 0;
  int active_set_polish_factorizations = 0;
  int total_iters = 0;
  int outer_iters = 0;
  const int max_outer = 60;
  const int max_total = opt.max_iter;

  Eigen::VectorXd best_x = x;
  Eigen::VectorXd best_s = s;
  Eigen::VectorXd best_lambda = lambda;
  Eigen::VectorXd best_mu_ineq = mu_ineq;
  double best_kkt_merit = std::numeric_limits<double>::infinity();
  bool have_best = false;
  ResidualSummary best_residuals{};
  std::string terminal_status =
      "Filter: max iterations reached without convergence";

  auto snapshot_outcome = [&](bool converged, int iters,
                              const std::string& status,
                              const ResidualSummary& rs) {
    result.converged = converged;
    result.iterations = iters;
    result.status = status;
    result.x = x;
    result.s = s;
    result.lambda = lambda;
    result.mu_ineq = mu_ineq;
    result.n_nonlinear_ineq = state.n_nonlinear_ineq;
    result.final_residuals = rs;
    result.objective = state.obj_orig;
    result.newton_profile = newton_profile;
    if (newton_profile.selected == "augmented") {
      result.symbolic_analyses =
          retired_symbolic_analyses + augmented_cache.kkt.symbolic_analyses;
      result.numeric_factorizations =
          retired_numeric_factorizations +
          augmented_cache.kkt.numeric_factorizations;
      result.linear_solves =
          retired_linear_solves + augmented_cache.kkt.linear_solves;
    } else if (opt.use_inertia_correction) {
      result.symbolic_analyses =
          retired_symbolic_analyses + kkt_cache.augmented.symbolic_analyses;
      result.numeric_factorizations =
          retired_numeric_factorizations +
          kkt_cache.augmented.numeric_factorizations;
      result.linear_solves =
          retired_linear_solves + kkt_cache.augmented.linear_solves;
    } else {
      result.symbolic_analyses =
          retired_symbolic_analyses + regularized_kkt_cache.symbolic_analyses;
      result.numeric_factorizations =
          retired_numeric_factorizations +
          regularized_kkt_cache.numeric_factorizations;
      result.linear_solves =
          retired_linear_solves + regularized_kkt_cache.linear_solves;
    }
    result.inertia_retry_factorizations = inertia_retry_factorizations;
    result.active_set_polish_factorizations =
        active_set_polish_factorizations;
    result.primary_factorizations = std::max(
        0, result.numeric_factorizations -
               result.inertia_retry_factorizations -
               result.active_set_polish_factorizations);
    result.inertia_certificate_factorizations =
        kkt_cache.primal_numeric_factorizations;
  };

  for (; outer_iters < max_outer && total_iters < max_total; ++outer_iters) {
    double inner_tol = opt.kappa_epsilon * mu_bar;
    const bool at_minimum_barrier =
        mu_bar <= effective_mu_min *
                      (1.0 + 32.0 * std::numeric_limits<double>::epsilon());
    if (at_minimum_barrier) {
      // At the final barrier, E_mu uses the perturbed residual S*mu-mu_bar,
      // while user convergence uses the unperturbed complementarity S*mu.
      // The ordinary kappa_epsilon*mu threshold can therefore declare the
      // inner problem solved just above the requested KKT tolerance and then
      // leave no smaller barrier to advance to. Tighten the final inner solve
      // enough to permit one last Newton refinement.
      double requested_accuracy = std::min(opt.tol_primal, opt.tol_dual);
      if (s.size() > 0) {
        requested_accuracy =
            std::min(requested_accuracy, opt.tol_complementarity);
      }
      if (requested_accuracy > 0.0 && std::isfinite(requested_accuracy)) {
        inner_tol = std::min(inner_tol,
                             0.1 * std::max(1e-16, requested_accuracy));
      }
    }
    bool inner_converged_at_mu = false;

    for (; total_iters < max_total; ++total_iters) {
      const auto derivative_eval_start = std::chrono::steady_clock::now();
      if (!evaluate_nlp_state(prob, x, lb_cols, ub_cols, state, eval_status)) {
        snapshot_outcome(false, total_iters, eval_status, best_residuals);
        return result;
      }
      const double state_eval_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - derivative_eval_start).count();

      // Build Lagrangian Hessian (full user callback if available).
      const auto hessian_eval_start = std::chrono::steady_clock::now();
      Eigen::SparseMatrix<double> hess;
      const Eigen::VectorXd mu_nonlinear =
          nonlinear_inequality_multipliers(mu_ineq, state.n_nonlinear_ineq);
      if (filter_qn_state.active) {
        update_quasi_newton_state(x, state.grad, state.jg, state.jh,
                                  lambda, mu_ineq, filter_qn_state);
      }
      if (!build_lagrangian_hessian(prob, x, lambda, &mu_nonlinear, hess,
                                    eval_status,
                                    filter_qn_state.active ? &filter_qn_state : nullptr)) {
        snapshot_outcome(false, total_iters, eval_status, best_residuals);
        return result;
      }
      const double hessian_eval_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - hessian_eval_start).count();
      state.hess = hess;
      if (opt.verbose) {
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " derivatives: H.nnz=" << state.hess.nonZeros()
                  << ", Jg.nnz=" << state.jg.nonZeros()
                  << ", Jh.nnz=" << state.jh.nonZeros()
                  << ", state_seconds=" << state_eval_seconds
                  << ", hessian_seconds=" << hessian_eval_seconds << '\n'
                  << std::flush;
      }

      // KKT residuals in absolute scale.
      const Eigen::VectorXd r_d = state.grad +
                                  state.jg.transpose() * lambda +
                                  state.jh.transpose() * mu_ineq;
      const Eigen::VectorXd r_eq = state.g;
      const Eigen::VectorXd r_ineq = state.h + s;
      const Eigen::VectorXd r_comp =
          (s.size() == 0) ? Eigen::VectorXd()
                          : Eigen::VectorXd(s.cwiseProduct(mu_ineq).array() - mu_bar);

      ResidualSummary rs =
          summarize_residuals(r_d, r_eq, r_ineq, x, s, lambda, mu_ineq);
      if (total_iters == 0) result.initial_residuals = rs;
      if (opt.verbose) {
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " residuals: primal=" << rs.primal_feas
                  << ", dual=" << rs.dual_feas
                  << ", complementarity=" << rs.complementarity
                  << ", merit=" << rs.merit
                  << ", mu_bar=" << mu_bar << '\n' << std::flush;
      }

      if (rs.merit < best_kkt_merit) {
        best_kkt_merit = rs.merit;
        best_x = x;
        best_s = s;
        best_lambda = lambda;
        best_mu_ineq = mu_ineq;
        best_residuals = rs;
        have_best = true;
      }

      // Outer convergence: problem solved to user tolerance.
      if (rs.primal_feas <= opt.tol_primal &&
          rs.dual_feas <= opt.tol_dual &&
          (s.size() == 0 || rs.complementarity <= opt.tol_complementarity)) {
        snapshot_outcome(true, total_iters + 1, "Converged", rs);
        return result;
      }

      // Inner convergence: barrier subproblem satisfied at current μ.
      const double E_mu = std::max({inf_norm(r_d), inf_norm(r_eq),
                                    inf_norm(r_ineq),
                                    (s.size() == 0) ? 0.0 : inf_norm(r_comp)});
      if (E_mu <= inner_tol) {
        inner_converged_at_mu = true;
        break;  // proceed to μ update
      }

      // Build the primal-dual reduced KKT matrix W = H + Σ + Jh^T (M/S) Jh
      // (condensed path) — or solve the augmented uncondensed system.
      const auto w_build_start = std::chrono::steady_clock::now();
      Eigen::VectorXd dx;
      Eigen::VectorXd dlambda;
      Eigen::VectorXd ds = Eigen::VectorXd::Zero(s.size());
      Eigen::VectorXd dmu_ineq = Eigen::VectorXd::Zero(mu_ineq.size());
      Eigen::VectorXd rhs;  // Newton rhs (layout depends on the active path)
      InertiaSettings isettings;
      isettings.mu = mu_bar;
      double iteration_delta_w_used = 0.0;

      Eigen::SparseMatrix<double> condensed_w;
      bool condensed_w_ready = false;
      auto build_condensed_w = [&]() {
        if (condensed_w_ready) return;
        condensed_w = state.hess;
        if (state.jh.rows() > 0) {
          const Eigen::VectorXd d = mu_ineq.cwiseQuotient(
              s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)));
          Eigen::SparseMatrix<double> scaled_jh = scale_rows(state.jh, d);
          condensed_w += state.jh.transpose() * scaled_jh;
        }
        condensed_w.makeCompressed();
        condensed_w_ready = true;
      };

      if (!newton_formulation_selected) {
        const bool augmented_forced =
            opt.use_augmented_newton ||
            opt.newton_formulation == NewtonFormulation::Augmented;
        const bool condensed_forced =
            opt.newton_formulation == NewtonFormulation::Condensed;
        bool choose_augmented = augmented_forced && state.jh.rows() > 0;
        if (!augmented_forced && !condensed_forced && state.jh.rows() > 0) {
          build_condensed_w();
#if defined(HACDCPF_HAVE_MUMPS) || defined(HACDCPF_HAVE_MKL_PARDISO)
          choose_augmented = select_augmented_by_symbolic_cost(
              condensed_w, state.hess, state.jg, state.jh, newton_profile);
#else
          // Automatic augmented routing requires a direct inertia-capable
          // factor. Explicit Augmented still fails with a numerical status.
          choose_augmented = false;
#endif
        }
        newton_profile.selected = choose_augmented ? "augmented" : "condensed";
        newton_formulation_selected = true;
        if (opt.verbose) {
          std::cerr << "[NativeIPM] Newton formulation="
                    << newton_profile.selected
                    << " condensed=(dim=" << newton_profile.condensed_dimension
                    << ",nnz=" << newton_profile.condensed_nonzeros
                    << ",flops=" << newton_profile.condensed_flops
                    << ",lnz=" << newton_profile.condensed_lnz
                    << ") augmented=(dim="
                    << newton_profile.augmented_dimension
                    << ",nnz=" << newton_profile.augmented_nonzeros
                    << ",flops=" << newton_profile.augmented_flops
                    << ",lnz=" << newton_profile.augmented_lnz << ")\n";
        }
      }
      bool use_augmented_newton =
          newton_profile.selected == "augmented" && state.jh.rows() > 0;
      if (use_augmented_newton) {
        // ── Augmented (uncondensed) Newton path ──
        // [H + δ_W I, Jgᵀ, Jhᵀ; Jg, 0, 0; Jh, 0, -SM⁻¹] (dx, dλ, dμ) =
        //   [-r_d; -r_eq; -r_ineq - M⁻¹(μ̄e - Sμ)]
        augmented_cache.n = static_cast<int>(state.hess.rows());
        augmented_cache.meq = static_cast<int>(state.jg.rows());
        augmented_cache.miq = static_cast<int>(state.jh.rows());
        const int miq = augmented_cache.miq;
        Eigen::VectorXd rhs_ineq_aug(miq);
        for (int i = 0; i < miq; ++i) {
          const double mui = std::max(mu_ineq[i], kMinPositive);
          rhs_ineq_aug[i] = -r_ineq[i] - (mu_bar - s[i] * mu_ineq[i]) / mui;
        }
        rhs.resize(augmented_cache.n + augmented_cache.meq + miq);
        rhs << -r_d, -r_eq, rhs_ineq_aug;
        if (!factor_solve_augmented_newton(
                augmented_cache, state.hess, state.jg, state.jh, s, mu_ineq,
                delta_w_last, isettings, rhs, dx, dlambda, dmu_ineq)) {
          snapshot_outcome(
              false, total_iters + 1,
              "Filter: augmented KKT regularization cap exceeded", rs);
          return result;
        }
        ds = -r_ineq - state.jh * dx;
        if (opt.verbose) {
          std::cerr << "[NativeIPM] iter=" << total_iters
                    << " augmented Newton: dim="
                    << augmented_cache.n + augmented_cache.meq + miq
                    << " nnz=" << augmented_cache.kkt.nnz
                    << ", seconds="
                    << std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - w_build_start)
                           .count()
                    << '\n' << std::flush;
        }
      } else {
      build_condensed_w();
      Eigen::SparseMatrix<double>& w = condensed_w;
      if (opt.verbose) {
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " W assembled: nnz=" << w.nonZeros()
                  << ", seconds="
                  << std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - w_build_start)
                         .count()
                  << '\n' << std::flush;
      }

      // Eliminate ds and dmu from
      //   Jh dx + ds = -r_ineq,
      //   M ds + S dmu = mu_bar*e - S*mu.
      // This gives
      //   dmu = S^-1(mu_bar*e - S*mu + M*r_ineq + M*Jh*dx),
      // hence
      //   rhs_x = -r_d - Jh^T S^-1(M*r_ineq + mu_bar*e - S*mu)
      //                  rhs_eq = −g
      Eigen::VectorXd rhs_x = -r_d;
      if (state.jh.rows() > 0) {
        Eigen::VectorXd v(s.size());
        for (int i = 0; i < s.size(); ++i) {
          const double si = std::max(s[i], kMinPositive);
          v[i] = (mu_ineq[i] * r_ineq[i] + mu_bar -
                  s[i] * mu_ineq[i]) / si;
        }
        rhs_x -= state.jh.transpose() * v;
      }
      rhs.resize(rhs_x.size() + r_eq.size());
      rhs << rhs_x, -r_eq;

      InertiaStatus istatus;
      const auto factor_start = std::chrono::steady_clock::now();
      if (opt.verbose) {
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " augmented KKT factor: start (dim="
                  << w.rows() + state.jg.rows() << ")\n" << std::flush;
      }
      const bool kkt_ok = opt.use_inertia_correction
          ? (factor_kkt_inertia_corrected_sparse(
                 w, state.jg, isettings, delta_w_last, kkt_cache, istatus) &&
             solve_kkt_inertia_corrected_sparse(
                 kkt_cache, rhs, dx, dlambda))
          : (factor_kkt_with_regularization(
                 w, state.jg, regularized_kkt_cache) &&
             solve_kkt_sparse(regularized_kkt_cache, rhs, dx, dlambda));
      if (opt.use_inertia_correction) {
        inertia_retry_factorizations +=
            std::max(0, istatus.factorization_attempts - 1);
      }
      iteration_delta_w_used = istatus.delta_w_used;
      if (!kkt_ok) {
        snapshot_outcome(false, total_iters + 1,
                         opt.use_inertia_correction
                             ? "Filter: KKT inertia correction cap exceeded"
                             : "Filter: regularized KKT factorization failed",
                         rs);
        return result;
      }
      const bool condensation_precision_exhausted =
          opt.newton_formulation == NewtonFormulation::Auto &&
          opt.use_inertia_correction && state.jh.rows() > 0 &&
          istatus.reduced_space_certificate &&
          istatus.min_reduced_curvature > 0.0 &&
          istatus.reduced_curvature_margin >
              istatus.min_reduced_curvature;
      if (condensation_precision_exhausted) {
        augmented_cache.n = static_cast<int>(state.hess.rows());
        augmented_cache.meq = static_cast<int>(state.jg.rows());
        augmented_cache.miq = static_cast<int>(state.jh.rows());
        Eigen::VectorXd rhs_aug(
            augmented_cache.n + augmented_cache.meq + augmented_cache.miq);
        Eigen::VectorXd rhs_ineq_aug(augmented_cache.miq);
        for (int i = 0; i < augmented_cache.miq; ++i) {
          const double mui = std::max(mu_ineq[i], kMinPositive);
          rhs_ineq_aug[i] =
              -r_ineq[i] - (mu_bar - s[i] * mu_ineq[i]) / mui;
        }
        rhs_aug << -r_d, -r_eq, rhs_ineq_aug;
        Eigen::VectorXd augmented_dx;
        Eigen::VectorXd augmented_dlambda;
        Eigen::VectorXd augmented_dmu;
        // Condensed regularization compensates for the formed Schur product
        // and is not transferable to the uncondensed matrix.
        delta_w_last = 0.0;
        if (factor_solve_augmented_newton(
                augmented_cache, state.hess, state.jg, state.jh, s, mu_ineq,
                delta_w_last, isettings, rhs_aug, augmented_dx,
                augmented_dlambda, augmented_dmu)) {
          retired_symbolic_analyses +=
              kkt_cache.augmented.symbolic_analyses;
          retired_numeric_factorizations +=
              kkt_cache.augmented.numeric_factorizations;
          retired_linear_solves += kkt_cache.augmented.linear_solves;
          dx = std::move(augmented_dx);
          dlambda = std::move(augmented_dlambda);
          dmu_ineq = std::move(augmented_dmu);
          ds = -r_ineq - state.jh * dx;
          rhs = std::move(rhs_aug);
          use_augmented_newton = true;
          newton_profile.selected = "augmented";
          if (opt.verbose) {
            std::cerr
                << "[NativeIPM] condensed precision certificate exhausted; "
                   "escalating to augmented Newton at iter="
                << total_iters << " (lambda_min="
                << istatus.min_reduced_curvature << ", roundoff_margin="
                << istatus.reduced_curvature_margin << ")\n";
          }
        }
      }
      if (opt.verbose) {
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " augmented KKT factor: done (nnz="
                  << (opt.use_inertia_correction
                          ? kkt_cache.augmented.nnz
                          : regularized_kkt_cache.nnz)
                  << ", inertia=" << opt.use_inertia_correction
                  << ", delta_w=" << istatus.delta_w_used
                  << ", delta_c=" << istatus.delta_c_used
                  << ", tangent_cert="
                  << istatus.reduced_space_certificate
                  << ", tangent_dim=" << istatus.tangent_dimension
                  << ", reduced_lambda_min="
                  << istatus.min_reduced_curvature
                  << ", reduced_margin="
                  << istatus.reduced_curvature_margin
                  << ", null_residual=" << istatus.nullspace_residual
                  << ", seconds="
                  << std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - factor_start)
                         .count()
                  << ")\n" << std::flush;
      }

      // Recover step directions for the eliminated variables:
      //   ds       = −r_ineq − Jh dx
      //   dmu_ineq = (μ_bar − s·μ − μ·ds) / s
      if (!use_augmented_newton && s.size() > 0) {
        ds = -r_ineq - state.jh * dx;
        for (int i = 0; i < s.size(); ++i) {
          const double si = std::max(s[i], kMinPositive);
          dmu_ineq[i] = (mu_bar - s[i] * mu_ineq[i] - mu_ineq[i] * ds[i]) / si;
        }
      }
      }  // end condensed (explicit Jhᵀ(M/S)Jh) Newton path

      // Fraction-to-boundary step caps.
      const double tau = std::max(0.5, std::min(opt.alpha_max,
                                                1.0 - 0.01 * mu_bar));
      double alpha_max_primal =
          (s.size() == 0) ? 1.0 : max_positive_step(s, ds, tau);
      double alpha_max_dual =
          (mu_ineq.size() == 0) ? 1.0
                                : max_positive_step(mu_ineq, dmu_ineq, tau);
      if (s.size() > 0 && mu_ineq.size() > 0) {
        constexpr int kMaxExtraCorrectors = 2;
        constexpr double kExtraCorrBeta = 0.1;
        const Eigen::VectorXd target_compl =
            s.cwiseProduct(mu_ineq);
        for (int gc = 0; gc < kMaxExtraCorrectors; ++gc) {
          const double alpha_probe_primal = std::min(1.0, alpha_max_primal);
          const double alpha_probe_dual = std::min(1.0, alpha_max_dual);
          const Eigen::VectorXd s_probe = s + alpha_probe_primal * ds;
          const Eigen::VectorXd mu_probe = mu_ineq + alpha_probe_dual * dmu_ineq;
          const Eigen::VectorXd eps_gc =
              (kExtraCorrBeta * target_compl - s_probe.cwiseProduct(mu_probe))
                  .cwiseMax(0.0);
          if (eps_gc.size() == 0 || eps_gc.maxCoeff() <= 1e-12) {
            break;
          }

          Eigen::VectorXd dx_gc;
          Eigen::VectorXd dlambda_gc;
          Eigen::VectorXd dmu_gc;
          bool gc_ok = false;
          if (use_augmented_newton) {
            // Augmented correction rhs: [0; 0; -M⁻¹ε] — derived from the
            // same correction equations (see Newton-step comment).
            Eigen::VectorXd rhs_gc = Eigen::VectorXd::Zero(
                augmented_cache.n + augmented_cache.meq + augmented_cache.miq);
            for (int i = 0; i < augmented_cache.miq; ++i) {
              const double mui = std::max(mu_ineq[i], kMinPositive);
              rhs_gc[augmented_cache.n + augmented_cache.meq + i] =
                  -eps_gc[i] / mui;
            }
            gc_ok = solve_augmented_newton(augmented_cache, rhs_gc, dx_gc,
                                           dlambda_gc, dmu_gc);
          } else {
            const int n_x = static_cast<int>(state.hess.rows());
            Eigen::VectorXd rhs_gc(n_x + r_eq.size());
            rhs_gc.head(n_x) =
                -(state.jh.transpose() *
                  eps_gc.cwiseQuotient(
                      s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive))));
            rhs_gc.tail(r_eq.size()).setZero();
            gc_ok = opt.use_inertia_correction
                ? solve_kkt_inertia_corrected_sparse(
                      kkt_cache, rhs_gc, dx_gc, dlambda_gc)
                : solve_kkt_sparse(
                      regularized_kkt_cache, rhs_gc, dx_gc, dlambda_gc);
            if (gc_ok) {
              const Eigen::VectorXd ds_gc0 = -state.jh * dx_gc;
              dmu_gc =
                  eps_gc.cwiseQuotient(
                      s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive))) -
                  mu_ineq.cwiseQuotient(
                      s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)))
                      .cwiseProduct(ds_gc0);
            }
          }
          if (!gc_ok) {
            break;
          }

          dx += dx_gc;
          dlambda += dlambda_gc;
          ds += -state.jh * dx_gc;
          dmu_ineq += dmu_gc;
        }
      }

      const double theta_k = compute_theta(state.g, state.h, s);
      const double theta_min =
          opt.filter_theta_min_scale * std::max(1.0, theta_k);
      double alpha = std::min(1.0, alpha_max_primal);

      // Filter line search.
      const double phi_k = compute_barrier_phi(state.obj_orig, mu_bar, s);
      const double slope_k =
          barrier_descent_slope(state.grad, dx, s, ds, mu_bar);

      bool accepted = false;
      bool accepted_was_f_type = false;
      TrialPoint accepted_trial;
      double theta_trial = 0.0;
      double phi_trial = 0.0;
      bool soc_attempted = false;
      const double alpha0 = alpha;  // remember alpha_max for SOC gate
      const double alpha_dual = std::min(1.0, alpha_max_dual);
      if (opt.verbose) {
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " direction: |dx|inf=" << inf_norm(dx)
                  << ", |ds|inf=" << inf_norm(ds)
                  << ", |dmu|inf=" << inf_norm(dmu_ineq)
                  << ", alpha_pri_max=" << alpha_max_primal
                  << ", alpha_dual_max=" << alpha_max_dual
                  << ", theta=" << theta_k
                  << ", phi=" << phi_k
                  << ", slope=" << slope_k << '\n' << std::flush;
      }

      while (alpha > opt.filter_alpha_min) {
        const double trial_alpha_dual = std::min(alpha_dual, alpha);
        const Eigen::VectorXd x_trial = x + alpha * dx;
        Eigen::VectorXd s_trial =
          (s.size() == 0) ? Eigen::VectorXd() : Eigen::VectorXd(s + alpha * ds);
        const Eigen::VectorXd lambda_trial =
            lambda + trial_alpha_dual * dlambda;
        const Eigen::VectorXd mu_trial =
            mu_ineq + trial_alpha_dual * dmu_ineq;

        // Safety: slack positivity (fraction-to-boundary guarantees it, but
        // numerical rounding can still dip).
        bool slack_valid = true;
        for (int i = 0; i < s_trial.size(); ++i) {
          if (!(s_trial[i] > 0.0)) { slack_valid = false; break; }
        }
        if (!slack_valid) {
          alpha *= 0.5;
          continue;
        }

        TrialPoint trial;
        if (!evaluate_filter_trial_values(prob, lb_cols, ub_cols,
                                          x_trial, s_trial, lambda_trial,
                                          mu_trial, eval_status, trial)) {
          alpha *= 0.5;
          continue;
        }

        theta_trial = compute_theta(trial.state.g, trial.state.h, s_trial);
        phi_trial = compute_barrier_phi(trial.state.obj_orig, mu_bar, s_trial);

        // A filter only sees primal feasibility and barrier objective. At an
        // already stationary primal point, a valid Newton step may update only
        // lambda/mu, leaving both filter coordinates unchanged. Accept that
        // special case only when the primal displacement is at roundoff scale
        // and the independently recomputed KKT residuals prove componentwise
        // non-worsening plus strict dual/complementarity progress.
        const double relative_primal_step = std::max(
            alpha * inf_norm(dx) / (1.0 + inf_norm(x)),
            alpha * inf_norm(ds) / (1.0 + inf_norm(s)));
        if (relative_primal_step <= 1e-12) {
          const Eigen::VectorXd r_dual_trial =
              state.grad + state.jg.transpose() * lambda_trial +
              state.jh.transpose() * mu_trial;
          const Eigen::VectorXd r_eq_trial = trial.state.g;
          const Eigen::VectorXd r_ineq_trial = trial.state.h + s_trial;
          trial.residuals = summarize_residuals(
              r_dual_trial, r_eq_trial, r_ineq_trial, x_trial, s_trial,
              lambda_trial, mu_trial);
          const double primal_guard =
              rs.primal_feas + 1e-10 * (1.0 + rs.primal_feas);
          const double dual_guard =
              rs.dual_feas + 1e-10 * (1.0 + rs.dual_feas);
          const bool componentwise_safe =
              trial.residuals.primal_feas <= primal_guard &&
              trial.residuals.dual_feas <= dual_guard;
          const bool complementarity_progress = s.size() > 0 &&
              trial.residuals.complementarity <=
                  rs.complementarity *
                      (1.0 - 1e-4 * trial_alpha_dual) + 1e-14;
          const bool dual_progress =
              trial.residuals.dual_feas <=
                  rs.dual_feas *
                      (1.0 - 1e-4 * trial_alpha_dual) + 1e-14;
          const bool merit_progress =
              trial.residuals.merit + 1e-14 < rs.merit;
          if (componentwise_safe && merit_progress &&
              (complementarity_progress || dual_progress)) {
            accepted = true;
            accepted_was_f_type = true;  // no new primal filter entry
            accepted_trial = std::move(trial);
            if (opt.verbose) {
              std::cerr << "[NativeIPM] iter=" << total_iters
                        << " centrality step accepted: alpha=" << alpha
                        << ", alpha_dual=" << trial_alpha_dual
                        << ", kkt_merit=" << rs.merit << " -> "
                        << accepted_trial.residuals.merit << '\n'
                        << std::flush;
            }
            break;
          }
        }

        bool filter_ok = filter.is_acceptable(theta_trial, phi_trial,
                                              opt.filter_gamma_theta,
                                              opt.filter_gamma_phi);

        const bool switching =
            (slope_k < 0.0) &&
            (alpha * std::pow(-slope_k, opt.filter_s_phi) >
             opt.filter_delta * std::pow(theta_k, opt.filter_s_theta)) &&
            (theta_k <= theta_min);

        bool accept = false;
        if (filter_ok) {
          if (switching) {
            accept = phi_trial <=
                     phi_k + opt.filter_eta_phi * alpha * slope_k + 1e-12;
            accepted_was_f_type = accept;
          } else {
            accept = (theta_trial <=
                      (1.0 - opt.filter_gamma_theta) * theta_k) ||
                     (phi_trial <= phi_k - opt.filter_gamma_phi * theta_k);
            accepted_was_f_type = false;
          }
        }

        if (accept) {
          accepted = true;
          accepted_trial = std::move(trial);
          if (opt.verbose) {
            std::cerr << "[NativeIPM] iter=" << total_iters
                      << " line search accepted: alpha=" << alpha
                      << ", alpha_dual=" << trial_alpha_dual
                      << ", theta_trial=" << theta_trial
                      << ", phi_trial=" << phi_trial
                      << '\n' << std::flush;
          }
          break;
        }

        if (opt.verbose && alpha == alpha0) {
          std::cerr << "[NativeIPM] iter=" << total_iters
                    << " full step rejected: filter_ok=" << filter_ok
                    << ", switching=" << switching
                    << ", theta_trial=" << theta_trial
                    << ", phi_trial=" << phi_trial
                    << '\n' << std::flush;
        }

        // Second-order correction: on the very first rejection, if θ is the
        // blocker (θ_trial > θ_k), resolve the KKT with RHS
        //   (rhs_x, −(α · g + g(x + α dx)))
        // and try the composite step x + α·dx + dx_soc once. Gated by
        // opt.use_second_order_correction and done at most once per iteration.
        if (opt.use_second_order_correction && !soc_attempted &&
            alpha == alpha0 && theta_trial > theta_k && r_eq.size() > 0) {
          soc_attempted = true;

          Eigen::VectorXd c_soc = alpha * r_eq + trial.state.g;
          Eigen::VectorXd rhs_soc = rhs;
          Eigen::VectorXd dx_soc;
          Eigen::VectorXd dlambda_soc;
          Eigen::VectorXd dmu_soc;
          bool soc_ok;
          if (use_augmented_newton) {
            // Augmented layout: the equality block is the middle segment.
            rhs_soc.segment(augmented_cache.n, r_eq.size()) = -c_soc;
            soc_ok = solve_augmented_newton(augmented_cache, rhs_soc, dx_soc,
                                            dlambda_soc, dmu_soc);
          } else {
            // Overwrite the equality block of the RHS with -c_soc; the primal
            // block (rhs_x) stays the same.
            rhs_soc.tail(r_eq.size()) = -c_soc;
            soc_ok = opt.use_inertia_correction
                ? solve_kkt_inertia_corrected_sparse(
                      kkt_cache, rhs_soc, dx_soc, dlambda_soc)
                : solve_kkt_sparse(
                      regularized_kkt_cache, rhs_soc, dx_soc, dlambda_soc);
          }
          if (soc_ok) {
            // Composite primal step; update slacks via linearized h.
            const Eigen::VectorXd x_soc_trial = x + alpha * dx + dx_soc;
            Eigen::VectorXd s_soc_trial;
            Eigen::VectorXd ds_soc_delta = Eigen::VectorXd::Zero(s.size());
            if (s.size() > 0) {
              ds_soc_delta = -state.jh * dx_soc;
              s_soc_trial = s + alpha * ds + ds_soc_delta;
            } else {
              s_soc_trial = Eigen::VectorXd::Zero(0);
            }

            bool soc_slack_ok = true;
            for (int i = 0; i < s_soc_trial.size(); ++i) {
              if (!(s_soc_trial[i] > 0.0)) { soc_slack_ok = false; break; }
            }

            if (soc_slack_ok) {
              Eigen::VectorXd dmu_soc_extra = Eigen::VectorXd::Zero(mu_ineq.size());
              if (mu_ineq.size() > 0 && s.size() > 0) {
                if (use_augmented_newton) {
                  dmu_soc_extra = dmu_soc;  // solved directly on this path
                } else {
                  dmu_soc_extra = -mu_ineq.cwiseProduct(ds_soc_delta).cwiseQuotient(
                      s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)));
                }
              }
              const Eigen::VectorXd lambda_soc_trial =
                  lambda + trial_alpha_dual * dlambda + dlambda_soc;
              const Eigen::VectorXd mu_soc_trial =
                  mu_ineq + trial_alpha_dual * dmu_ineq + dmu_soc_extra;
              TrialPoint soc_trial;
              if (evaluate_filter_trial_values(prob, lb_cols, ub_cols,
                                               x_soc_trial, s_soc_trial,
                                               lambda_soc_trial, mu_soc_trial,
                                               eval_status, soc_trial)) {
                const double theta_soc = compute_theta(soc_trial.state.g,
                                                       soc_trial.state.h,
                                                       s_soc_trial);
                const double phi_soc = compute_barrier_phi(
                    soc_trial.state.obj_orig, mu_bar, s_soc_trial);

                bool soc_filter_ok = filter.is_acceptable(
                    theta_soc, phi_soc, opt.filter_gamma_theta,
                    opt.filter_gamma_phi);
                bool soc_accept = false;
                if (soc_filter_ok) {
                  if (switching) {
                    soc_accept = phi_soc <= phi_k +
                                 opt.filter_eta_phi * alpha * slope_k + 1e-12;
                    accepted_was_f_type = soc_accept;
                  } else {
                    soc_accept = (theta_soc <=
                                  (1.0 - opt.filter_gamma_theta) * theta_k) ||
                                 (phi_soc <=
                                  phi_k - opt.filter_gamma_phi * theta_k);
                    accepted_was_f_type = false;
                  }
                }

                if (soc_accept) {
                  theta_trial = theta_soc;
                  phi_trial = phi_soc;
                  accepted_trial = std::move(soc_trial);
                  accepted = true;
                  break;
                }
              }
            }
          }
        }

        alpha *= 0.5;
      }

      if (!accepted) {
        if (opt.verbose) {
          std::cerr << "[NativeIPM] iter=" << total_iters
                    << " line search collapsed: alpha=" << alpha
                    << ", alpha0=" << alpha0
                    << ", theta_trial=" << theta_trial
                    << ", phi_trial=" << phi_trial
                    << ", theta_current=" << theta_k
                    << ", phi_current=" << phi_k
                    << ", slope=" << slope_k
                    << ", soc_attempted=" << soc_attempted
                    << '\n' << std::flush;
        }
        if (have_best) {
          x = best_x;
          s = best_s;
          lambda = best_lambda;
          mu_ineq = best_mu_ineq;
        }
        ActiveSetPolishOutcome polish;
        bool polish_converged = false;
        if (have_best) {
          polish_converged = try_active_set_kkt_polish(
              prob, opt, lb_cols, ub_cols, x, s, lambda, mu_ineq,
              best_residuals, polish);
          retired_symbolic_analyses += polish.symbolic_analyses;
          retired_numeric_factorizations += polish.numeric_factorizations;
          active_set_polish_factorizations += polish.numeric_factorizations;
          retired_linear_solves += polish.linear_solves;
        }
        if (polish_converged) {
          x = std::move(polish.x);
          s = std::move(polish.s);
          lambda = std::move(polish.lambda);
          mu_ineq = std::move(polish.mu);
          state = std::move(polish.state);
          snapshot_outcome(
              true, total_iters + polish.iterations + 1,
              "Converged after active-set KKT polish", polish.residuals);
          return result;
        }
        snapshot_outcome(false, total_iters + 1,
                         "Filter: accepted-step collapse", best_residuals);
        return result;
      }

      // Apply one coherent primal-dual trial accepted by the same gate.
      x = accepted_trial.x;
      s = accepted_trial.s;
      lambda = accepted_trial.lambda;
      mu_ineq = accepted_trial.mu;
      // Keep mu_ineq strictly positive.
      for (int i = 0; i < mu_ineq.size(); ++i) {
        mu_ineq[i] = std::max(mu_ineq[i], kMinPositive);
      }
      // Keep slacks strictly positive.
      for (int i = 0; i < s.size(); ++i) {
        s[i] = std::max(s[i], kMinPositive);
      }
      state = std::move(accepted_trial.state);

      // Retain a proven regularization across a boundary-limited or locally
      // untrustworthy step. A normal/tangential step classifier is still
      // needed before this policy can be made less conservative.
      if (!use_augmented_newton && opt.use_inertia_correction &&
          iteration_delta_w_used > 0.0) {
        const double meaningful_step =
            1.0 / std::sqrt(static_cast<double>(
                std::max(1, static_cast<int>(x.size()))));
        const double predicted_phi_reduction =
            std::max(0.0, -alpha * slope_k);
        const double actual_phi_reduction = phi_k - phi_trial;
        const double reduction_roundoff =
            64.0 * std::numeric_limits<double>::epsilon() *
            std::max(1.0, std::abs(phi_k));
        const bool objective_model_predictive =
            predicted_phi_reduction <= reduction_roundoff ||
            actual_phi_reduction >= 0.1 * predicted_phi_reduction;
        const bool feasibility_model_predictive =
            theta_trial <= theta_k *
                (1.0 - opt.filter_gamma_theta * std::max(alpha, 1e-3));
        const bool decay_is_earned = alpha >= meaningful_step &&
            (objective_model_predictive || feasibility_model_predictive);
        if (!decay_is_earned) {
          delta_w_last = std::max(delta_w_last, iteration_delta_w_used);
          if (opt.verbose) {
            std::cerr << "[NativeIPM] regularization retained: delta_w="
                      << delta_w_last << ", alpha=" << alpha
                      << ", dimension_step_floor=" << meaningful_step
                      << ", actual_phi_reduction=" << actual_phi_reduction
                      << ", predicted_phi_reduction="
                      << predicted_phi_reduction << '\n';
          }
        }
      }

      // Filter augmentation: insert (θ_k, φ_k) for θ-type (non-f-type) steps.
      if (!accepted_was_f_type) {
        filter.add_entry(theta_k, phi_k,
                         opt.filter_gamma_theta, opt.filter_gamma_phi);
      }
    }

    if (!inner_converged_at_mu) break;  // iteration budget exhausted

    // Barrier update (IPOPT monotone rule).
    const double mu_new = std::max(effective_mu_min,
                                   std::min(opt.kappa_mu * mu_bar,
                                            std::pow(mu_bar, opt.theta_mu)));
    if (mu_new >= mu_bar) {  // cannot decrease further — μ clamped at μ_min
      // We already failed the outer KKT check above (otherwise we would have
      // returned), even after the tightened final-barrier inner solve.
      terminal_status =
          "Filter: minimum barrier reached without KKT convergence";
      break;
    }
    mu_bar = mu_new;
  }

  // Exhausted outer iterations. Fall back to the best iterate seen so the
  // caller gets the closest feasible primal it has seen.
  if (have_best) {
    x = best_x;
    s = best_s;
    lambda = best_lambda;
    mu_ineq = best_mu_ineq;
  }

  snapshot_outcome(false, total_iters, terminal_status, best_residuals);
  return result;
}

bool try_ipopt_fallback(const NLPModel& prob,
                        const std::string& native_status,
                        SolveResult& out,
                        IPMDetail& detail) {
  IpoptAdapter ipopt;
  SolveResult fallback = ipopt.solve_nlp(prob);
  if (fallback.x.size() != static_cast<Eigen::Index>(prob.vars.size()) ||
      !fallback.x.allFinite()) {
    return false;
  }

  Eigen::VectorXd equalities = Eigen::VectorXd::Zero(0);
  Eigen::VectorXd nonlinear_inequalities = Eigen::VectorXd::Zero(0);
  if (prob.g) prob.g(fallback.x, equalities);
  if (prob.h) prob.h(fallback.x, nonlinear_inequalities);
  const int meq = static_cast<int>(equalities.size());
  const int mnlin = static_cast<int>(nonlinear_inequalities.size());

  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  for (int col = 0; col < fallback.x.size(); ++col) {
    const auto& var = prob.vars[static_cast<std::size_t>(col)];
    if (is_effectively_finite(var.lb)) lb_cols.push_back(col);
    if (is_effectively_finite(var.ub)) ub_cols.push_back(col);
  }

  const int mineq = mnlin + static_cast<int>(lb_cols.size()) +
                    static_cast<int>(ub_cols.size());
  if (fallback.constraint_duals.size() != mnlin + meq ||
      fallback.box_dual_lb.size() != fallback.x.size() ||
      fallback.box_dual_ub.size() != fallback.x.size()) {
    return false;
  }

  detail.lambda_eq = fallback.constraint_duals.tail(meq);
  detail.mu_ineq = Eigen::VectorXd::Constant(mineq, 2.0 * kMinPositive);
  detail.z_slack = Eigen::VectorXd::Constant(mineq, 2.0 * kMinPositive);
  if (mnlin > 0) {
    detail.mu_ineq.head(mnlin) =
        fallback.constraint_duals.head(mnlin).array().max(2.0 * kMinPositive);
    detail.z_slack.head(mnlin) =
        (-nonlinear_inequalities.array()).max(2.0 * kMinPositive);
  }
  int row = mnlin;
  for (int col : lb_cols) {
    detail.mu_ineq[row] =
        std::max(fallback.box_dual_lb[col], 2.0 * kMinPositive);
    detail.z_slack[row] = std::max(
        fallback.x[col] - prob.vars[static_cast<std::size_t>(col)].lb,
        2.0 * kMinPositive);
    ++row;
  }
  for (int col : ub_cols) {
    detail.mu_ineq[row] =
        std::max(fallback.box_dual_ub[col], 2.0 * kMinPositive);
    detail.z_slack[row] = std::max(
        prob.vars[static_cast<std::size_t>(col)].ub - fallback.x[col],
        2.0 * kMinPositive);
    ++row;
  }

  detail.complementarity = detail.mu_ineq.size() > 0
      ? detail.mu_ineq.cwiseProduct(detail.z_slack).cwiseAbs().maxCoeff()
      : 0.0;

  const bool ipopt_converged = fallback.stats.success;
  const std::string ipopt_status = fallback.stats.status;
  fallback.stats.solver_name = "NativeIPM[IpoptFallback]";
  fallback.stats.status = "Ipopt fallback (" + ipopt_status +
                          ") after native failure: " + native_status;
  out = std::move(fallback);
  return ipopt_converged;
}

struct FixedNLPMap {
  int original_dimension{0};
  std::vector<int> reduced_to_original;
  std::vector<int> original_to_reduced;
  Eigen::VectorXd fixed_values;

  Eigen::VectorXd expand(const Eigen::VectorXd& reduced) const {
    Eigen::VectorXd full = fixed_values;
    for (int r = 0; r < static_cast<int>(reduced_to_original.size()); ++r) {
      full[reduced_to_original[static_cast<std::size_t>(r)]] = reduced[r];
    }
    return full;
  }

  Eigen::VectorXd gather(const Eigen::VectorXd& full) const {
    Eigen::VectorXd reduced(reduced_to_original.size());
    for (int r = 0; r < reduced.size(); ++r) {
      reduced[r] = full[reduced_to_original[static_cast<std::size_t>(r)]];
    }
    return reduced;
  }
};

Eigen::SparseMatrix<double> gather_sparse_columns(
    const Eigen::SparseMatrix<double>& full,
    const FixedNLPMap& map) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(full.nonZeros()));
  for (int col = 0; col < full.outerSize(); ++col) {
    const int reduced_col = map.original_to_reduced[static_cast<std::size_t>(col)];
    if (reduced_col < 0) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(full, col); it; ++it) {
      triplets.emplace_back(it.row(), reduced_col, it.value());
    }
  }
  Eigen::SparseMatrix<double> reduced(
      full.rows(), static_cast<int>(map.reduced_to_original.size()));
  reduced.setFromTriplets(triplets.begin(), triplets.end());
  reduced.makeCompressed();
  return reduced;
}

Eigen::SparseMatrix<double> gather_sparse_principal_block(
    const Eigen::SparseMatrix<double>& full,
    const FixedNLPMap& map) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(full.nonZeros()));
  for (int col = 0; col < full.outerSize(); ++col) {
    const int reduced_col = map.original_to_reduced[static_cast<std::size_t>(col)];
    if (reduced_col < 0) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(full, col); it; ++it) {
      const int reduced_row =
          map.original_to_reduced[static_cast<std::size_t>(it.row())];
      if (reduced_row >= 0) {
        triplets.emplace_back(reduced_row, reduced_col, it.value());
      }
    }
  }
  const int nr = static_cast<int>(map.reduced_to_original.size());
  Eigen::SparseMatrix<double> reduced(nr, nr);
  reduced.setFromTriplets(triplets.begin(), triplets.end());
  reduced.makeCompressed();
  return reduced;
}

bool build_fixed_nlp_reduction(const NLPModel& original,
                               NLPModel& reduced,
                               std::shared_ptr<FixedNLPMap>& map) {
  const int n = static_cast<int>(original.vars.size());
  auto candidate = std::make_shared<FixedNLPMap>();
  candidate->original_dimension = n;
  candidate->original_to_reduced.assign(static_cast<std::size_t>(n), -1);
  candidate->fixed_values = Eigen::VectorXd::Zero(n);

  for (int col = 0; col < n; ++col) {
    const VariableMeta& var = original.vars[static_cast<std::size_t>(col)];
    const bool fixed = is_effectively_finite(var.lb) &&
                       is_effectively_finite(var.ub) && var.lb == var.ub;
    if (fixed) {
      candidate->fixed_values[col] = var.lb;
    } else {
      candidate->original_to_reduced[static_cast<std::size_t>(col)] =
          static_cast<int>(candidate->reduced_to_original.size());
      candidate->reduced_to_original.push_back(col);
      candidate->fixed_values[col] =
          original.x0.size() == n && std::isfinite(original.x0[col])
              ? original.x0[col]
              : 0.0;
    }
  }
  if (candidate->reduced_to_original.size() == static_cast<std::size_t>(n) ||
      candidate->reduced_to_original.empty()) {
    return false;
  }

  reduced = NLPModel{};
  reduced.sense = original.sense;
  reduced.solver_options = original.solver_options;
  reduced.vars.reserve(candidate->reduced_to_original.size());
  for (int original_col : candidate->reduced_to_original) {
    reduced.vars.push_back(original.vars[static_cast<std::size_t>(original_col)]);
  }
  Eigen::VectorXd full_x0 = original.x0.size() == n
      ? original.x0 : candidate->fixed_values;
  for (int col = 0; col < n; ++col) {
    if (candidate->original_to_reduced[static_cast<std::size_t>(col)] < 0) {
      full_x0[col] = candidate->fixed_values[col];
    }
  }
  reduced.x0 = candidate->gather(full_x0);

  reduced.f = [&original, candidate](const Eigen::VectorXd& x) {
    return original.f(candidate->expand(x));
  };
  reduced.grad = [&original, candidate](const Eigen::VectorXd& x,
                                        Eigen::VectorXd& grad) {
    Eigen::VectorXd full_grad;
    original.grad(candidate->expand(x), full_grad);
    grad = candidate->gather(full_grad);
  };
  if (original.hess) {
    reduced.hess = [&original, candidate](const Eigen::VectorXd& x,
                                          Eigen::SparseMatrix<double>& hess) {
      Eigen::SparseMatrix<double> full;
      original.hess(candidate->expand(x), full);
      hess = gather_sparse_principal_block(full, *candidate);
    };
  }
  if (original.lagrangian_hess) {
    reduced.lagrangian_hess =
        [&original, candidate](const Eigen::VectorXd& x,
                               const Eigen::VectorXd& lambda,
                               const Eigen::VectorXd* nu,
                               Eigen::SparseMatrix<double>& hess) {
          Eigen::SparseMatrix<double> full;
          original.lagrangian_hess(candidate->expand(x), lambda, nu, full);
          hess = gather_sparse_principal_block(full, *candidate);
        };
  }
  if (original.g) {
    reduced.g = [&original, candidate](const Eigen::VectorXd& x,
                                       Eigen::VectorXd& g) {
      original.g(candidate->expand(x), g);
    };
  }
  if (original.jac_g) {
    reduced.jac_g = [&original, candidate](const Eigen::VectorXd& x,
                                           Eigen::SparseMatrix<double>& jac) {
      Eigen::SparseMatrix<double> full;
      original.jac_g(candidate->expand(x), full);
      jac = gather_sparse_columns(full, *candidate);
    };
  }
  if (original.h) {
    reduced.h = [&original, candidate](const Eigen::VectorXd& x,
                                       Eigen::VectorXd& h) {
      original.h(candidate->expand(x), h);
    };
  }
  if (original.jac_h) {
    reduced.jac_h = [&original, candidate](const Eigen::VectorXd& x,
                                           Eigen::SparseMatrix<double>& jac) {
      Eigen::SparseMatrix<double> full;
      original.jac_h(candidate->expand(x), full);
      jac = gather_sparse_columns(full, *candidate);
    };
  }

  std::vector<int> reduced_free_columns;
  reduced_free_columns.reserve(original.equality_free_columns.size());
  for (int original_col : original.equality_free_columns) {
    if (original_col < 0 || original_col >= n) continue;
    const int reduced_col =
        candidate->original_to_reduced[static_cast<std::size_t>(original_col)];
    if (reduced_col >= 0) reduced_free_columns.push_back(reduced_col);
  }
  reduced.equality_free_columns = std::move(reduced_free_columns);
  map = std::move(candidate);
  return true;
}

std::vector<int> finite_lower_bound_columns(const NLPModel& prob) {
  std::vector<int> columns;
  for (int col = 0; col < static_cast<int>(prob.vars.size()); ++col) {
    if (is_effectively_finite(prob.vars[static_cast<std::size_t>(col)].lb)) {
      columns.push_back(col);
    }
  }
  return columns;
}

std::vector<int> finite_upper_bound_columns(const NLPModel& prob) {
  std::vector<int> columns;
  for (int col = 0; col < static_cast<int>(prob.vars.size()); ++col) {
    if (is_effectively_finite(prob.vars[static_cast<std::size_t>(col)].ub)) {
      columns.push_back(col);
    }
  }
  return columns;
}

double raw_primal_violation(const NLPModel& prob, const Eigen::VectorXd& x) {
  double violation = 0.0;
  if (prob.g) {
    Eigen::VectorXd equality;
    prob.g(x, equality);
    if (!equality.allFinite()) return std::numeric_limits<double>::infinity();
    violation = std::max(violation, inf_norm(equality));
  }
  if (prob.h) {
    Eigen::VectorXd inequality;
    prob.h(x, inequality);
    if (!inequality.allFinite()) return std::numeric_limits<double>::infinity();
    if (inequality.size() > 0) {
      violation = std::max(violation, std::max(0.0, inequality.maxCoeff()));
    }
  }
  for (int col = 0; col < static_cast<int>(prob.vars.size()); ++col) {
    const auto& variable = prob.vars[static_cast<std::size_t>(col)];
    if (is_effectively_finite(variable.lb)) {
      violation = std::max(violation, variable.lb - x[col]);
    }
    if (is_effectively_finite(variable.ub)) {
      violation = std::max(violation, x[col] - variable.ub);
    }
  }
  return std::max(0.0, violation);
}

bool recover_partitioned_equality_duals(
    const NLPModel& prob, const NLPState& state,
    const Eigen::VectorXd& inequality_dual, Eigen::VectorXd& equality_dual,
    double& state_stationarity, double& control_stationarity) {
  const int n = static_cast<int>(state.grad.size());
  const int meq = static_cast<int>(state.jg.rows());
  if (meq <= 0 || meq > n || state.jg.cols() != n ||
      inequality_dual.size() != state.jh.rows() ||
      static_cast<int>(prob.equality_free_columns.size()) != n - meq) {
    return false;
  }

  std::vector<unsigned char> is_control(static_cast<std::size_t>(n), 0);
  for (int col : prob.equality_free_columns) {
    if (col < 0 || col >= n || is_control[static_cast<std::size_t>(col)] != 0) {
      return false;
    }
    is_control[static_cast<std::size_t>(col)] = 1;
  }
  std::vector<int> state_columns;
  state_columns.reserve(static_cast<std::size_t>(meq));
  std::vector<int> variable_to_state(static_cast<std::size_t>(n), -1);
  for (int col = 0; col < n; ++col) {
    if (is_control[static_cast<std::size_t>(col)] == 0) {
      variable_to_state[static_cast<std::size_t>(col)] =
          static_cast<int>(state_columns.size());
      state_columns.push_back(col);
    }
  }
  if (static_cast<int>(state_columns.size()) != meq) return false;

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(state.jg.nonZeros()));
  for (int col = 0; col < state.jg.outerSize(); ++col) {
    const int state_row = variable_to_state[static_cast<std::size_t>(col)];
    if (state_row < 0) continue;
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.jg, col); it;
         ++it) {
      triplets.emplace_back(state_row, it.row(), it.value());
    }
  }
  Eigen::SparseMatrix<double> state_jacobian_transpose(meq, meq);
  state_jacobian_transpose.setFromTriplets(triplets.begin(), triplets.end());
  state_jacobian_transpose.makeCompressed();

  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu;
  lu.analyzePattern(state_jacobian_transpose);
  lu.factorize(state_jacobian_transpose);
  if (lu.info() != Eigen::Success) return false;

  const Eigen::VectorXd stationarity_without_equalities =
      state.grad + state.jh.transpose() * inequality_dual;
  Eigen::VectorXd rhs(meq);
  for (int row = 0; row < meq; ++row) {
    rhs[row] = -stationarity_without_equalities[
        state_columns[static_cast<std::size_t>(row)]];
  }
  Eigen::VectorXd candidate = lu.solve(rhs);
  if (lu.info() != Eigen::Success || !candidate.allFinite()) return false;

  const Eigen::VectorXd stationarity =
      stationarity_without_equalities + state.jg.transpose() * candidate;
  state_stationarity = 0.0;
  control_stationarity = 0.0;
  for (int col = 0; col < n; ++col) {
    if (is_control[static_cast<std::size_t>(col)] != 0) {
      control_stationarity =
          std::max(control_stationarity, std::abs(stationarity[col]));
    } else {
      state_stationarity =
          std::max(state_stationarity, std::abs(stationarity[col]));
    }
  }
  const double solve_scale = std::max(1.0, inf_norm(rhs));
  if (!std::isfinite(state_stationarity) ||
      state_stationarity > 1e-7 * solve_scale) {
    return false;
  }
  equality_dual = std::move(candidate);
  return true;
}

bool restore_fixed_nlp_certificate(
    const NLPModel& original, const NLPModel& reduced,
    const FixedNLPMap& map, SolveResult& out, IPMDetail& detail,
    const IPMOptions& opt) {
  const int n = map.original_dimension;
  if (out.x.size() != static_cast<int>(map.reduced_to_original.size())) {
    return false;
  }
  out.x = map.expand(out.x);
  if (!out.stats.success) return true;

  Eigen::VectorXd nonlinear = Eigen::VectorXd::Zero(0);
  if (original.h) original.h(out.x, nonlinear);
  const int n_nonlinear = static_cast<int>(nonlinear.size());
  const std::vector<int> reduced_lb = finite_lower_bound_columns(reduced);
  const std::vector<int> reduced_ub = finite_upper_bound_columns(reduced);
  if (detail.mu_ineq.size() != n_nonlinear +
          static_cast<int>(reduced_lb.size() + reduced_ub.size()) ||
      detail.z_slack.size() != detail.mu_ineq.size()) {
    return false;
  }

  const std::vector<int> original_lb = finite_lower_bound_columns(original);
  const std::vector<int> original_ub = finite_upper_bound_columns(original);
  Eigen::VectorXd original_mu = Eigen::VectorXd::Zero(
      n_nonlinear + static_cast<int>(original_lb.size() + original_ub.size()));
  Eigen::VectorXd original_slack = Eigen::VectorXd::Zero(original_mu.size());
  if (n_nonlinear > 0) {
    original_mu.head(n_nonlinear) = detail.mu_ineq.head(n_nonlinear);
    original_slack.head(n_nonlinear) = detail.z_slack.head(n_nonlinear);
  }

  Eigen::VectorXd bound_lb = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd bound_ub = Eigen::VectorXd::Zero(n);
  int reduced_row = n_nonlinear;
  for (int reduced_col : reduced_lb) {
    const int original_col =
        map.reduced_to_original[static_cast<std::size_t>(reduced_col)];
    bound_lb[original_col] = detail.mu_ineq[reduced_row++];
  }
  for (int reduced_col : reduced_ub) {
    const int original_col =
        map.reduced_to_original[static_cast<std::size_t>(reduced_col)];
    bound_ub[original_col] = detail.mu_ineq[reduced_row++];
  }

  Eigen::VectorXd grad;
  original.grad(out.x, grad);
  if (original.sense == Sense::Maximize) grad = -grad;
  if (original.g && detail.lambda_eq.size() > 0) {
    Eigen::SparseMatrix<double> jac_g;
    original.jac_g(out.x, jac_g);
    grad.noalias() += jac_g.transpose() * detail.lambda_eq;
  }
  if (n_nonlinear > 0) {
    Eigen::SparseMatrix<double> jac_h;
    original.jac_h(out.x, jac_h);
    grad.noalias() +=
        jac_h.transpose() * original_mu.head(n_nonlinear);
  }
  for (int col = 0; col < n; ++col) {
    if (map.original_to_reduced[static_cast<std::size_t>(col)] < 0) {
      bound_lb[col] = std::max(grad[col], 0.0);
      bound_ub[col] = std::max(-grad[col], 0.0);
    }
  }

  int row = n_nonlinear;
  for (int col : original_lb) {
    original_mu[row] = bound_lb[col];
    original_slack[row] = out.x[col] -
        original.vars[static_cast<std::size_t>(col)].lb;
    ++row;
  }
  for (int col : original_ub) {
    original_mu[row] = bound_ub[col];
    original_slack[row] =
        original.vars[static_cast<std::size_t>(col)].ub - out.x[col];
    ++row;
  }

  NLPState original_state;
  std::string status;
  if (!evaluate_nlp_state(original, out.x, original_lb, original_ub,
                          original_state, status)) {
    return false;
  }
  const Eigen::VectorXd r_dual =
      original_state.grad + original_state.jg.transpose() * detail.lambda_eq +
      original_state.jh.transpose() * original_mu;
  const ResidualSummary residuals = summarize_residuals(
      r_dual, original_state.g, original_state.h + original_slack,
      out.x, original_slack, detail.lambda_eq, original_mu);

  out.stats.objective = original.f(out.x);
  out.stats.primal_feas = residuals.primal_feas;
  out.stats.dual_feas = residuals.dual_feas;
  out.stats.complementarity = residuals.complementarity;
  out.stats.residual_inf = std::max(residuals.primal_feas, residuals.dual_feas);
  out.stats.success = residuals.primal_feas <= opt.tol_primal &&
                      residuals.dual_feas <= opt.tol_dual &&
                      residuals.complementarity <= opt.tol_complementarity;
  if (!out.stats.success) {
    out.stats.status =
        "Fixed-variable postsolve failed original-space KKT certification";
  }
  out.box_dual_lb = std::move(bound_lb);
  out.box_dual_ub = std::move(bound_ub);
  detail.mu_ineq = std::move(original_mu);
  detail.z_slack = std::move(original_slack);
  detail.complementarity = residuals.complementarity;
  out.constraint_duals.resize(detail.mu_ineq.size() + detail.lambda_eq.size());
  if (detail.mu_ineq.size() > 0) {
    out.constraint_duals.head(detail.mu_ineq.size()) = detail.mu_ineq;
  }
  if (detail.lambda_eq.size() > 0) {
    out.constraint_duals.tail(detail.lambda_eq.size()) = detail.lambda_eq;
  }
  return true;
}

void transform_filter_options_to_scaled_coordinates(
    const ScalingFactors& scaling, IPMOptions& options) {
  // r_d^s = s_f*r_d and (s*mu)^s = s_f*(s*mu). A single primal
  // tolerance must protect every scaled constraint row.
  double min_constraint_scale = 1.0;  // generated bound rows are unscaled
  if (scaling.s_g.size() > 0) {
    min_constraint_scale =
        std::min(min_constraint_scale, scaling.s_g.minCoeff());
  }
  if (scaling.s_h.size() > 0) {
    min_constraint_scale =
        std::min(min_constraint_scale, scaling.s_h.minCoeff());
  }
  min_constraint_scale = std::clamp(min_constraint_scale, 1e-16, 1.0);
  const double objective_scale = std::clamp(scaling.s_f, 1e-16, 1.0);
  options.tol_primal *= min_constraint_scale;
  options.tol_dual *= objective_scale;
  options.tol_complementarity *= objective_scale;
  if (options.tol_accept > 0.0) {
    options.tol_accept *= std::min(min_constraint_scale, objective_scale);
  }
  options.mu_init *= objective_scale;
  options.mu_min *= objective_scale;

  // Original -> scaled dual/slack warm starts. Nonlinear inequalities lead;
  // generated lower/upper bound rows have no row scale.
  if (options.equality_dual_start.size() == scaling.s_g.size()) {
    for (int i = 0; i < options.equality_dual_start.size(); ++i) {
      options.equality_dual_start[i] *=
          objective_scale / scaling.s_g[i];
    }
  }
  if (options.inequality_dual_start.size() > 0) {
    const int nonlinear_count = std::min(
        static_cast<int>(scaling.s_h.size()),
        static_cast<int>(options.inequality_dual_start.size()));
    for (int i = 0; i < nonlinear_count; ++i) {
      options.inequality_dual_start[i] *=
          objective_scale / scaling.s_h[i];
    }
    for (int i = nonlinear_count; i < options.inequality_dual_start.size();
         ++i) {
      options.inequality_dual_start[i] *= objective_scale;
    }
  }
  if (options.slack_start.size() > 0) {
    const int nonlinear_count = std::min(
        static_cast<int>(scaling.s_h.size()),
        static_cast<int>(options.slack_start.size()));
    for (int i = 0; i < nonlinear_count; ++i) {
      options.slack_start[i] *= scaling.s_h[i];
    }
  }
}

bool audit_filter_outcome_in_original_coordinates(
    const NLPModel& original, const ScalingFactors& scaling,
    const FilterSolveOutcome& scaled, ResidualSummary& residuals) {
  if (scaled.x.size() != static_cast<int>(original.vars.size())) return false;
  Eigen::VectorXd lambda = scaled.lambda;
  Eigen::VectorXd mu = scaled.mu_ineq;
  Eigen::VectorXd slack = scaled.s;
  if (!(scaling.s_f > 0.0) || !std::isfinite(scaling.s_f)) return false;
  if (lambda.size() != scaling.s_g.size()) return false;
  for (int i = 0; i < lambda.size(); ++i) {
    lambda[i] *= scaling.s_g[i] / scaling.s_f;
  }
  const int nonlinear_count = scaled.n_nonlinear_ineq;
  if (nonlinear_count > mu.size() || nonlinear_count > slack.size() ||
      nonlinear_count > scaling.s_h.size()) {
    return false;
  }
  for (int i = 0; i < nonlinear_count; ++i) {
    mu[i] *= scaling.s_h[i] / scaling.s_f;
    slack[i] /= scaling.s_h[i];
  }
  for (int i = nonlinear_count; i < mu.size(); ++i) {
    mu[i] /= scaling.s_f;
  }

  NLPState state;
  std::string status;
  if (!evaluate_nlp_state(original, scaled.x, scaled.lb_cols, scaled.ub_cols,
                          state, status)) {
    return false;
  }
  if (state.h.size() != slack.size() || state.h.size() != mu.size() ||
      state.g.size() != lambda.size()) {
    return false;
  }
  const Eigen::VectorXd r_dual =
      state.grad + state.jg.transpose() * lambda + state.jh.transpose() * mu;
  residuals = summarize_residuals(r_dual, state.g, state.h + slack, scaled.x,
                                  slack, lambda, mu);
  return true;
}

}  // namespace

NativeIPMAdapter::NativeIPMAdapter(IPMOptions opt) : opt_(std::move(opt)) {}

std::string NativeIPMAdapter::name() const { return "NativeIPM"; }

bool NativeIPMAdapter::supports(ProblemClass cls) const {
  return cls == ProblemClass::NLP;
}

SolveResult NativeIPMAdapter::solve_nlp(const NLPModel& prob) const {
  auto [res, detail] = solve_nlp_detail(prob);
  return res;
}

// AUDIT-NAV: NLP-IPM 总入口；默认滤子路径依次拥有缩放、KKT 惯性修正、试点
// 接受、二阶修正/恢复和障碍更新，只有完整 KKT 条件可发布成功。
std::pair<SolveResult, IPMDetail> NativeIPMAdapter::solve_nlp_detail(const NLPModel& prob) const {
  const auto t0 = std::chrono::steady_clock::now();
  SolveResult out;
  out.stats.solver_name = name();
  IPMDetail detail;
  detail.original_dimension = static_cast<int>(prob.vars.size());
  detail.reduced_dimension = detail.original_dimension;

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid NLP model" : vr.errors.front();
    return {out, detail};
  }

  if (!prob.f || !prob.grad) {
    out.stats.status = "NLP model missing objective callbacks";
    return {out, detail};
  }

  NLPModel reduced_prob;
  std::shared_ptr<FixedNLPMap> fixed_map;
  if (build_fixed_nlp_reduction(prob, reduced_prob, fixed_map)) {
    NativeIPMAdapter reduced_solver(opt_);
    auto reduced_result = reduced_solver.solve_nlp_detail(reduced_prob);
    out = std::move(reduced_result.first);
    detail = std::move(reduced_result.second);
    detail.original_dimension = static_cast<int>(prob.vars.size());
    detail.reduced_dimension =
        static_cast<int>(fixed_map->reduced_to_original.size());
    detail.fixed_variables_eliminated =
        detail.original_dimension - detail.reduced_dimension;
    if (!restore_fixed_nlp_certificate(
            prob, reduced_prob, *fixed_map, out, detail, opt_)) {
      out.stats.success = false;
      out.stats.status = "Fixed-variable postsolve mapping failed";
    }
    out.stats.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
    return {out, detail};
  }

  // Filter-based driver (Wächter–Biegler). Opt-in via IPMOptions::globalization.
  if (opt_.globalization == Globalization::Filter) {
    const int n_f = static_cast<int>(prob.vars.size());

    // --- Scaling: compute factors at the interiorized x0 and wrap the model.
    ScalingFactors sf;
    const NLPModel* active_prob = &prob;
    NLPModel scaled_prob;
    IPMOptions active_opt = opt_;
    const bool do_scale = opt_.scale_problem;
    if (do_scale) {
      Eigen::VectorXd x0_interior = (prob.x0.size() == n_f)
                                        ? prob.x0
                                        : Eigen::VectorXd::Zero(n_f);
      interiorize_initial_point(prob.vars, x0_interior);
      sf = compute_scaling_factors(prob, x0_interior, opt_.scaling_g_max);
      scaled_prob = build_scaled_nlp_model(prob, sf);
      active_prob = &scaled_prob;

      transform_filter_options_to_scaled_coordinates(sf, active_opt);
    }

    FilterSolveOutcome fo = solve_nlp_filter_impl(*active_prob, active_opt);
    int solve_chain_numeric_factorizations = fo.numeric_factorizations;
    int solve_chain_symbolic_analyses = fo.symbolic_analyses;
    int solve_chain_linear_solves = fo.linear_solves;
    int solve_chain_primary_factorizations = fo.primary_factorizations;
    int solve_chain_inertia_retries = fo.inertia_retry_factorizations;
    int solve_chain_inertia_certificates =
        fo.inertia_certificate_factorizations;
    int solve_chain_polish_factorizations =
        fo.active_set_polish_factorizations;
    int selected_base_numeric_factorizations = fo.numeric_factorizations;
    int selected_base_symbolic_analyses = fo.symbolic_analyses;
    int selected_base_linear_solves = fo.linear_solves;
    int selected_base_primary_factorizations = fo.primary_factorizations;
    int selected_base_inertia_retries = fo.inertia_retry_factorizations;
    int selected_base_inertia_certificates =
        fo.inertia_certificate_factorizations;
    int selected_base_polish_factorizations =
        fo.active_set_polish_factorizations;
    int restoration_factorizations = 0;
    int retry_factorizations = 0;
    bool restoration_warm_start_used = false;
    double restoration_normal_residual_before = 0.0;
    double restoration_normal_residual_after = 0.0;
    double restoration_state_stationarity = 0.0;
    double restoration_control_stationarity = 0.0;

    // --- Feasibility restoration on line-search / inertia failure.
    if (!fo.converged && opt_.use_restoration_phase) {
      const bool triggerable =
        fo.status.rfind("Filter: line-search step too small", 0) == 0 ||
        fo.status.rfind("Filter: accepted-step collapse", 0) == 0 ||
          fo.status.rfind("Filter: KKT inertia correction cap exceeded", 0) ==
          0 ||
        fo.status.rfind("Filter: max iterations reached without convergence", 0) ==
          0;
      if (triggerable && prob.g) {
        Eigen::VectorXd x_R = (fo.x.size() == n_f)
                                  ? fo.x
                                  : ((prob.x0.size() == n_f)
                                         ? prob.x0
                                         : Eigen::VectorXd::Zero(n_f));
        restoration_normal_residual_before = raw_primal_violation(prob, x_R);
        const bool restoration_is_applicable =
            std::isfinite(restoration_normal_residual_before) &&
            restoration_normal_residual_before > opt_.tol_primal;
        if (!restoration_is_applicable) {
          if (opt_.verbose) {
            std::cerr << "[NativeIPM] restoration skipped: original normal "
                         "residual="
                      << restoration_normal_residual_before
                      << " already satisfies primal tolerance="
                      << opt_.tol_primal << '\n';
          }
        } else {
        RestorationBuild rb =
            build_restoration_nlp(prob, x_R, opt_.restoration_zeta);
        IPMOptions rst_opt = opt_;
        rst_opt.use_restoration_phase = false;
        rst_opt.use_second_order_correction = false;
        rst_opt.scale_problem = false;
        rst_opt.globalization = Globalization::Filter;
        rst_opt.max_iter = std::min(opt_.max_iter, 200);

        FilterSolveOutcome rf = solve_nlp_filter_impl(rb.model, rst_opt);
        restoration_factorizations += rf.numeric_factorizations;
        solve_chain_numeric_factorizations += rf.numeric_factorizations;
        solve_chain_symbolic_analyses += rf.symbolic_analyses;
        solve_chain_linear_solves += rf.linear_solves;
        solve_chain_primary_factorizations += rf.primary_factorizations;
        solve_chain_inertia_retries += rf.inertia_retry_factorizations;
        solve_chain_inertia_certificates +=
            rf.inertia_certificate_factorizations;
        solve_chain_polish_factorizations +=
            rf.active_set_polish_factorizations;

        if (rf.x.size() >= rb.n_x) {
          Eigen::VectorXd x_new = extract_x_from_restoration(rb, rf.x);
          restoration_normal_residual_after =
              raw_primal_violation(prob, x_new);
          const double normal_roundoff =
              64.0 * std::numeric_limits<double>::epsilon() *
              std::max(1.0, restoration_normal_residual_before);
          const bool restoration_reduces_normal_residual =
              std::isfinite(restoration_normal_residual_after) &&
              restoration_normal_residual_after + normal_roundoff <
                  restoration_normal_residual_before;
          if (!restoration_reduces_normal_residual) {
            if (opt_.verbose) {
              std::cerr << "[NativeIPM] restoration retry skipped: normal "
                           "residual did not decrease (before="
                        << restoration_normal_residual_before << ", after="
                        << restoration_normal_residual_after << ")\n";
            }
          } else {
          NLPModel prob_retry = prob;
          prob_retry.x0 = x_new;

          RestorationWarmStart warm = recover_restoration_warm_start(
              prob, rb, rf.x, rf.lambda, rf.mu_ineq, rf.s,
              rf.n_nonlinear_ineq, rf.lb_cols, rf.ub_cols);
          if (warm.valid) {
            const std::vector<int> original_lb =
                finite_lower_bound_columns(prob);
            const std::vector<int> original_ub =
                finite_upper_bound_columns(prob);
            NLPState retry_state;
            std::string retry_state_status;
            if (evaluate_nlp_state(prob, warm.x, original_lb, original_ub,
                                   retry_state, retry_state_status) &&
                retry_state.g.size() == warm.equality_dual.size() &&
                retry_state.h.size() == warm.inequality_dual.size()) {
              Eigen::VectorXd partitioned_lambda = warm.equality_dual;
              if (recover_partitioned_equality_duals(
                      prob, retry_state, warm.inequality_dual,
                      partitioned_lambda, restoration_state_stationarity,
                      restoration_control_stationarity)) {
                warm.equality_dual = std::move(partitioned_lambda);
              } else {
                const Eigen::VectorXd stationarity =
                    retry_state.grad +
                    retry_state.jg.transpose() * warm.equality_dual +
                    retry_state.jh.transpose() * warm.inequality_dual;
                restoration_state_stationarity = inf_norm(stationarity);
                restoration_control_stationarity = inf_norm(stationarity);
              }
              restoration_warm_start_used = true;
            } else {
              warm.valid = false;
            }
          }

          ScalingFactors sf2;
          NLPModel scaled_retry;
          const NLPModel* retry_prob = &prob_retry;
          if (opt_.scale_problem) {
            Eigen::VectorXd x0i = x_new;
            interiorize_initial_point(prob_retry.vars, x0i);
            sf2 = compute_scaling_factors(prob_retry, x0i, opt_.scaling_g_max);
            scaled_retry = build_scaled_nlp_model(prob_retry, sf2);
            retry_prob = &scaled_retry;
          }

          IPMOptions retry_opt = opt_;
          retry_opt.use_restoration_phase = false;  // one-shot restoration
          if (restoration_warm_start_used) {
            retry_opt.equality_dual_start = warm.equality_dual;
            retry_opt.inequality_dual_start = warm.inequality_dual;
            retry_opt.slack_start = warm.slack;
            const double recovered_barrier =
                warm.slack.dot(warm.inequality_dual) /
                std::max(1, static_cast<int>(warm.slack.size()));
            if (std::isfinite(recovered_barrier) && recovered_barrier > 0.0) {
              retry_opt.mu_init = std::clamp(
                  recovered_barrier, retry_opt.mu_min, retry_opt.mu_init);
            }
          }
          if (opt_.scale_problem) {
            transform_filter_options_to_scaled_coordinates(sf2, retry_opt);
          }

          if (opt_.verbose) {
            std::cerr << "[NativeIPM] restoration structure: normal_before="
                      << restoration_normal_residual_before
                      << ", normal_after="
                      << restoration_normal_residual_after
                      << ", normal_step_accepted="
                      << restoration_reduces_normal_residual
                      << ", warm_start=" << restoration_warm_start_used
                      << ", state_stationarity="
                      << restoration_state_stationarity
                      << ", control_reduced_gradient="
                      << restoration_control_stationarity << '\n';
          }

          FilterSolveOutcome fo_retry =
              solve_nlp_filter_impl(*retry_prob, retry_opt);
          retry_factorizations += fo_retry.numeric_factorizations;
          solve_chain_numeric_factorizations +=
              fo_retry.numeric_factorizations;
          solve_chain_symbolic_analyses += fo_retry.symbolic_analyses;
          solve_chain_linear_solves += fo_retry.linear_solves;
          solve_chain_primary_factorizations +=
              fo_retry.primary_factorizations;
          solve_chain_inertia_retries +=
              fo_retry.inertia_retry_factorizations;
          solve_chain_inertia_certificates +=
              fo_retry.inertia_certificate_factorizations;
          solve_chain_polish_factorizations +=
              fo_retry.active_set_polish_factorizations;
          ResidualSummary original_before;
          ResidualSummary original_retry;
          bool before_audited = false;
          bool retry_audited = false;
          if (opt_.scale_problem) {
            before_audited = audit_filter_outcome_in_original_coordinates(
                prob, sf, fo, original_before);
            retry_audited = audit_filter_outcome_in_original_coordinates(
                prob, sf2, fo_retry, original_retry);
          } else {
            original_before = fo.final_residuals;
            original_retry = fo_retry.final_residuals;
            before_audited = true;
            retry_audited = true;
          }
          const double before_stationarity = std::max(
              original_before.dual_feas, original_before.complementarity);
          const double retry_stationarity = std::max(
              original_retry.dual_feas, original_retry.complementarity);
          const bool retry_improves_original_kkt = before_audited &&
              retry_audited &&
              original_retry.primal_feas < original_before.primal_feas &&
              retry_stationarity < before_stationarity;
          if (opt_.verbose && before_audited && retry_audited) {
            std::cerr << "[NativeIPM] restoration retry audit: before=(p="
                      << original_before.primal_feas << ",d="
                      << original_before.dual_feas << ",c="
                      << original_before.complementarity << ") retry=(p="
                      << original_retry.primal_feas << ",d="
                      << original_retry.dual_feas << ",c="
                      << original_retry.complementarity << ") accepted="
                      << retry_improves_original_kkt << '\n';
          }
          // Restoration is a primal recovery mechanism. Compare in the
          // original model coordinates and require simultaneous progress in
          // primal feasibility and the dual/complementarity envelope.
          if (retry_improves_original_kkt) {
            fo = fo_retry;
            sf = sf2;
            selected_base_numeric_factorizations =
                fo_retry.numeric_factorizations;
            selected_base_symbolic_analyses = fo_retry.symbolic_analyses;
            selected_base_linear_solves = fo_retry.linear_solves;
            selected_base_primary_factorizations =
                fo_retry.primary_factorizations;
            selected_base_inertia_retries =
                fo_retry.inertia_retry_factorizations;
            selected_base_inertia_certificates =
                fo_retry.inertia_certificate_factorizations;
            selected_base_polish_factorizations =
                fo_retry.active_set_polish_factorizations;
          }
          }
        }
        }
      }
    }

    // --- Unscale multipliers back to the original problem's coordinates.
    bool needs_original_scale_refinement = false;
    if (opt_.scale_problem && sf.s_f > 0.0) {
      if (fo.lambda.size() == sf.s_g.size()) {
        for (int i = 0; i < fo.lambda.size(); ++i) {
          fo.lambda[i] = fo.lambda[i] * sf.s_g[i] / sf.s_f;
        }
      }
      const int n_nl = fo.n_nonlinear_ineq;
      if (fo.mu_ineq.size() >= n_nl && fo.s.size() >= n_nl &&
          sf.s_h.size() >= n_nl) {
        for (int i = 0; i < n_nl; ++i) {
          fo.mu_ineq[i] = fo.mu_ineq[i] * sf.s_h[i] / sf.s_f;
          fo.s[i] = fo.s[i] / sf.s_h[i];
        }
      }
      // Box-slack multipliers (for lb_cols, ub_cols) have s_h = 1 (no scaling
      // was applied to the corresponding generated rows); the only remaining
      // factor is 1/s_f.
      for (int i = n_nl; i < fo.mu_ineq.size(); ++i) {
        fo.mu_ineq[i] = fo.mu_ineq[i] / sf.s_f;
      }
      // Recompute the objective in the original coordinates.
      if (fo.x.size() == n_f && prob.f) {
        fo.objective = prob.f(fo.x);
      }

      // Certify and report the returned iterate in the original model scale.
      // Internal scaled residuals are useful for globalization, but exposing
      // them beside unscaled multipliers/slacks would be inconsistent.
      NLPState original_state;
      std::string original_status;
      if (fo.x.size() == n_f &&
          evaluate_nlp_state(prob, fo.x, fo.lb_cols, fo.ub_cols,
                             original_state, original_status)) {
        const Eigen::VectorXd r_dual =
            original_state.grad + original_state.jg.transpose() * fo.lambda +
            original_state.jh.transpose() * fo.mu_ineq;
        const Eigen::VectorXd r_ineq = original_state.h + fo.s;
        fo.final_residuals = summarize_residuals(
            r_dual, original_state.g, r_ineq, fo.x, fo.s, fo.lambda,
            fo.mu_ineq);
        if (fo.converged &&
            (fo.final_residuals.primal_feas > opt_.tol_primal ||
             fo.final_residuals.dual_feas > opt_.tol_dual ||
             fo.final_residuals.complementarity >
                 opt_.tol_complementarity)) {
          fo.converged = false;
          fo.status = "Scaled solve failed unscaled KKT certification";
          needs_original_scale_refinement = true;
        }
      } else if (fo.converged) {
        fo.converged = false;
        fo.status = "Scaled solve could not evaluate unscaled KKT: " +
                    original_status;
      }
    }

    // A scaled-space solution can be close to, but not yet inside, the strict
    // original-scale KKT gate. A line-search collapse at an already feasible
    // primal point is the same structural state: restoration has no normal
    // residual to repair, while the dual/tangential trajectory still has work
    // left. Continue with the same unscaled primal-dual central state and a
    // fresh filter, rather than cold-starting or invoking restoration.
    const int remaining_iterations = opt_.max_iter - fo.iterations;
    const bool tangential_filter_stall = !fo.converged &&
        fo.final_residuals.primal_feas <= opt_.tol_primal &&
        (fo.status.rfind("Filter: accepted-step collapse", 0) == 0 ||
         fo.status.rfind("Filter: line-search step too small", 0) == 0);
    const bool continue_original_trajectory =
        needs_original_scale_refinement || tangential_filter_stall;
    if (continue_original_trajectory && remaining_iterations > 0 &&
        fo.x.size() == n_f && fo.lambda.allFinite() &&
        fo.mu_ineq.allFinite() && fo.s.allFinite() &&
        (fo.mu_ineq.array() > kMinPositive).all() &&
        (fo.s.array() > kMinPositive).all()) {
      const int prior_iterations = fo.iterations;
      const int prior_symbolic_analyses = fo.symbolic_analyses;
      const int prior_numeric_factorizations = fo.numeric_factorizations;
      const int prior_linear_solves = fo.linear_solves;
      const int prior_primary_factorizations = fo.primary_factorizations;
      const int prior_inertia_retries = fo.inertia_retry_factorizations;
      const int prior_inertia_certificates =
          fo.inertia_certificate_factorizations;
      const int prior_polish_factorizations =
          fo.active_set_polish_factorizations;
      const ResidualSummary original_initial_residuals = fo.initial_residuals;

      NLPModel refine_prob = prob;
      refine_prob.x0 = fo.x;
      IPMOptions refine_opt = opt_;
      refine_opt.max_iter = remaining_iterations;
      refine_opt.scale_problem = false;
      refine_opt.use_restoration_phase = false;
      refine_opt.equality_dual_start = fo.lambda;
      refine_opt.inequality_dual_start = fo.mu_ineq;
      refine_opt.slack_start = fo.s;
      if (fo.mu_ineq.size() > 0) {
        refine_opt.mu_init = std::clamp(
            fo.mu_ineq.dot(fo.s) / fo.mu_ineq.size(),
            refine_opt.mu_min, refine_opt.mu_init);
      }

      FilterSolveOutcome refined =
          solve_nlp_filter_impl(refine_prob, refine_opt);
      const bool adopt_refinement = refined.converged ||
          refined.final_residuals.merit < fo.final_residuals.merit;
      if (adopt_refinement) {
        refined.iterations += prior_iterations;
        refined.symbolic_analyses += prior_symbolic_analyses;
        refined.numeric_factorizations += prior_numeric_factorizations;
        refined.linear_solves += prior_linear_solves;
        refined.primary_factorizations += prior_primary_factorizations;
        refined.inertia_retry_factorizations += prior_inertia_retries;
        refined.inertia_certificate_factorizations +=
            prior_inertia_certificates;
        refined.active_set_polish_factorizations +=
            prior_polish_factorizations;
        refined.initial_residuals = original_initial_residuals;
        if (refined.converged) {
          refined.status = tangential_filter_stall
              ? "Converged after tangential filter continuation"
              : "Converged after original-scale KKT refinement";
        } else {
          refined.status = "Original-scale KKT refinement: " + refined.status;
        }
        fo = std::move(refined);
      } else {
        fo.symbolic_analyses += refined.symbolic_analyses;
        fo.numeric_factorizations += refined.numeric_factorizations;
        fo.linear_solves += refined.linear_solves;
        fo.primary_factorizations += refined.primary_factorizations;
        fo.inertia_retry_factorizations +=
            refined.inertia_retry_factorizations;
        fo.inertia_certificate_factorizations +=
            refined.inertia_certificate_factorizations;
        fo.active_set_polish_factorizations +=
            refined.active_set_polish_factorizations;
        fo.iterations += refined.iterations;
      }
    }

    detail.newton_formulation = fo.newton_profile.selected;
    detail.condensed_dimension = fo.newton_profile.condensed_dimension;
    detail.augmented_dimension = fo.newton_profile.augmented_dimension;
    detail.condensed_nonzeros = fo.newton_profile.condensed_nonzeros;
    detail.augmented_nonzeros = fo.newton_profile.augmented_nonzeros;
    detail.condensed_symbolic_flops = fo.newton_profile.condensed_flops;
    detail.augmented_symbolic_flops = fo.newton_profile.augmented_flops;
    detail.condensed_symbolic_nonzeros = fo.newton_profile.condensed_lnz;
    detail.augmented_symbolic_nonzeros = fo.newton_profile.augmented_lnz;
    detail.symbolic_analyses = solve_chain_symbolic_analyses +
        std::max(0, fo.symbolic_analyses - selected_base_symbolic_analyses);
    detail.numeric_factorizations = solve_chain_numeric_factorizations +
        std::max(0, fo.numeric_factorizations -
                        selected_base_numeric_factorizations);
    detail.linear_solves = solve_chain_linear_solves +
        std::max(0, fo.linear_solves - selected_base_linear_solves);
    detail.primary_factorizations = solve_chain_primary_factorizations +
        std::max(0, fo.primary_factorizations -
                        selected_base_primary_factorizations);
    detail.inertia_retry_factorizations = solve_chain_inertia_retries +
        std::max(0, fo.inertia_retry_factorizations -
                        selected_base_inertia_retries);
    detail.inertia_certificate_factorizations =
        solve_chain_inertia_certificates +
        std::max(0, fo.inertia_certificate_factorizations -
                        selected_base_inertia_certificates);
    detail.restoration_factorizations = restoration_factorizations;
    detail.retry_factorizations = retry_factorizations;
    detail.restoration_warm_start_used = restoration_warm_start_used;
    detail.restoration_normal_residual_before =
        restoration_normal_residual_before;
    detail.restoration_normal_residual_after =
        restoration_normal_residual_after;
    detail.restoration_state_stationarity = restoration_state_stationarity;
    detail.restoration_control_stationarity =
        restoration_control_stationarity;
    detail.active_set_polish_factorizations =
        solve_chain_polish_factorizations +
        std::max(0, fo.active_set_polish_factorizations -
                        selected_base_polish_factorizations);
    if (opt_.verbose) {
      std::cerr << "[NativeIPM] factorization breakdown: total_kkt="
                << detail.numeric_factorizations << ", primary="
                << detail.primary_factorizations << ", inertia_retries="
                << detail.inertia_retry_factorizations
                << ", inertia_certificates="
                << detail.inertia_certificate_factorizations
                << ", restoration=" << detail.restoration_factorizations
                << ", retry=" << detail.retry_factorizations
                << ", active_set_polish="
                << detail.active_set_polish_factorizations << '\n';
    }

    out.x = fo.x.size() == n_f ? fo.x : Eigen::VectorXd::Zero(n_f);
    out.stats.success = fo.converged;
    out.stats.status = fo.status;
    out.stats.iterations = fo.iterations;
    out.stats.objective = fo.objective;
    out.stats.primal_feas = fo.final_residuals.primal_feas;
    out.stats.dual_feas = fo.final_residuals.dual_feas;
    out.stats.complementarity = fo.final_residuals.complementarity;
    out.stats.residual_inf =
        std::max(fo.final_residuals.primal_feas, fo.final_residuals.dual_feas);

    if (fo.converged) {
      out.constraint_duals.resize(fo.mu_ineq.size() + fo.lambda.size());
      if (fo.mu_ineq.size() > 0) {
        out.constraint_duals.head(fo.mu_ineq.size()) = fo.mu_ineq;
      }
      if (fo.lambda.size() > 0) {
        out.constraint_duals.tail(fo.lambda.size()) = fo.lambda;
      }
      out.box_dual_lb = Eigen::VectorXd::Zero(n_f);
      out.box_dual_ub = Eigen::VectorXd::Zero(n_f);
      for (int k = 0; k < static_cast<int>(fo.lb_cols.size()); ++k) {
        const int idx = fo.n_nonlinear_ineq + k;
        if (idx < fo.mu_ineq.size()) {
          out.box_dual_lb[fo.lb_cols[static_cast<size_t>(k)]] = fo.mu_ineq[idx];
        }
      }
      for (int k = 0; k < static_cast<int>(fo.ub_cols.size()); ++k) {
        const int idx = fo.n_nonlinear_ineq +
                        static_cast<int>(fo.lb_cols.size()) + k;
        if (idx < fo.mu_ineq.size()) {
          out.box_dual_ub[fo.ub_cols[static_cast<size_t>(k)]] = fo.mu_ineq[idx];
        }
      }
      detail.lambda_eq = fo.lambda;
      detail.mu_ineq = fo.mu_ineq;
      detail.z_slack = fo.s;
      detail.complementarity = fo.final_residuals.complementarity;
    } else {
      // Filter failed even after restoration; try the Merit-backed native
      // driver before the optional Ipopt last-resort. Merit handles
      // quasi-Newton-only NLPs (no Hessian) and badly-conditioned small
      // problems more gracefully than the Filter prototype.
      // Merit is a recovery engine for models that have only quasi-Newton
      // curvature. Re-running a full exact-Hessian OPF with Merit discards the
      // structural Filter trajectory, repeats hundreds of derivative/KKT
      // evaluations, and has no stronger certificate. Keep exact-curvature
      // models on the Filter result (or the explicitly allowed external path).
      std::pair<SolveResult, IPMDetail> merit_pair;
      const bool merit_is_structurally_applicable =
          !prob.hess && !prob.lagrangian_hess;
      if (merit_is_structurally_applicable) {
        IPMOptions merit_opt = opt_;
        merit_opt.globalization = Globalization::Merit;
        merit_opt.use_restoration_phase = false;
        merit_opt.use_second_order_correction = false;
        merit_opt.scale_problem = false;
        NativeIPMAdapter merit_solver(merit_opt);
        merit_pair = merit_solver.solve_nlp_detail(prob);
        if (merit_pair.first.stats.success) {
          if (merit_pair.first.stats.solver_name == name()) {
            merit_pair.first.stats.solver_name = "NativeIPM[MeritFallback]";
            merit_pair.first.stats.status =
                "Native Merit fallback after Filter failure: " + fo.status +
                "; " + merit_pair.first.stats.status;
          }
          merit_pair.first.stats.runtime_sec =
              std::chrono::duration<double>(
                  std::chrono::steady_clock::now() - t0).count();
          return merit_pair;
        }
      } else if (opt_.verbose) {
        std::cerr << "[NativeIPM] exact Lagrangian Hessian available; "
                     "skipping non-structural Merit retry after Filter "
                  << fo.status << '\n';
      }

      // A terminated Ipopt run is not accepted as a solution, but its full
      // primal-dual iterate is still a valid warm state.  Re-evaluate that
      // state with the native residual definitions and let the Filter driver
      // certify it (or continue Newton iterations) under the requested KKT
      // tolerances.
      const auto& candidate = merit_pair.first;
      const auto& candidate_detail = merit_pair.second;
      std::string fallback_context = fo.status + " (initial p=" +
          std::to_string(fo.initial_residuals.primal_feas) + ", d=" +
          std::to_string(fo.initial_residuals.dual_feas) + ", c=" +
          std::to_string(fo.initial_residuals.complementarity) + ")";
      const bool complete_candidate =
          opt_.allow_external_fallback &&
          candidate.stats.solver_name.find("IpoptFallback") !=
              std::string::npos &&
          candidate.x.size() == n_f && candidate.x.allFinite() &&
          candidate_detail.lambda_eq.size() == fo.lambda.size() &&
          candidate_detail.mu_ineq.size() == fo.mu_ineq.size() &&
          candidate_detail.z_slack.size() == fo.s.size() &&
          candidate_detail.lambda_eq.allFinite() &&
          candidate_detail.mu_ineq.allFinite() &&
          candidate_detail.z_slack.allFinite() &&
          (candidate_detail.mu_ineq.array() > kMinPositive).all() &&
          (candidate_detail.z_slack.array() > kMinPositive).all();
      if (complete_candidate) {
        NLPModel refine_prob = prob;
        refine_prob.x0 = candidate.x;
        IPMOptions refine_opt = opt_;
        refine_opt.scale_problem = false;
        refine_opt.use_restoration_phase = false;
        refine_opt.equality_dual_start = candidate_detail.lambda_eq;
        refine_opt.inequality_dual_start = candidate_detail.mu_ineq;
        refine_opt.slack_start = candidate_detail.z_slack;
        if (candidate_detail.mu_ineq.size() > 0) {
          refine_opt.mu_init = std::clamp(
              candidate_detail.mu_ineq.dot(candidate_detail.z_slack) /
                  candidate_detail.mu_ineq.size(),
              refine_opt.mu_min, 0.1);
        }

        FilterSolveOutcome refined =
            solve_nlp_filter_impl(refine_prob, refine_opt);
        if (refined.converged) {
          out.x = refined.x;
          out.stats.success = true;
          out.stats.status = "Converged after Ipopt primal-dual warm start";
          out.stats.solver_name = "NativeIPM[IpoptWarmStart]";
          out.stats.iterations = refined.iterations;
          out.stats.objective = refined.objective;
          out.stats.primal_feas = refined.final_residuals.primal_feas;
          out.stats.dual_feas = refined.final_residuals.dual_feas;
          out.stats.complementarity =
              refined.final_residuals.complementarity;
          out.stats.residual_inf = std::max(
              out.stats.primal_feas, out.stats.dual_feas);
          out.constraint_duals.resize(
              refined.mu_ineq.size() + refined.lambda.size());
          out.constraint_duals << refined.mu_ineq, refined.lambda;
          out.box_dual_lb = Eigen::VectorXd::Zero(n_f);
          out.box_dual_ub = Eigen::VectorXd::Zero(n_f);
          for (int k = 0; k < static_cast<int>(refined.lb_cols.size()); ++k) {
            out.box_dual_lb[refined.lb_cols[static_cast<std::size_t>(k)]] =
                refined.mu_ineq[refined.n_nonlinear_ineq + k];
          }
          for (int k = 0; k < static_cast<int>(refined.ub_cols.size()); ++k) {
            out.box_dual_ub[refined.ub_cols[static_cast<std::size_t>(k)]] =
                refined.mu_ineq[
                    refined.n_nonlinear_ineq +
                    static_cast<int>(refined.lb_cols.size()) + k];
          }
          detail.lambda_eq = refined.lambda;
          detail.mu_ineq = refined.mu_ineq;
          detail.z_slack = refined.s;
          detail.complementarity =
              refined.final_residuals.complementarity;
          out.stats.runtime_sec = std::chrono::duration<double>(
              std::chrono::steady_clock::now() - t0).count();
          return {out, detail};
        }
        fallback_context += "; primal-dual refinement failed: " +
                            refined.status;
      } else {
        fallback_context += "; incomplete Ipopt warm state (lambda=" +
                            std::to_string(candidate_detail.lambda_eq.size()) +
                            "/" + std::to_string(fo.lambda.size()) +
                            ", mu=" +
                            std::to_string(candidate_detail.mu_ineq.size()) +
                            "/" + std::to_string(fo.mu_ineq.size()) +
                            ", slack=" +
                            std::to_string(candidate_detail.z_slack.size()) +
                            "/" + std::to_string(fo.s.size()) + ")";
      }
      if (opt_.allow_external_fallback &&
          try_ipopt_fallback(prob, fallback_context, out, detail)) {
        out.stats.runtime_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return {out, detail};
      }
    }

    out.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return {out, detail};
  }

  const int n = static_cast<int>(prob.vars.size());
  Eigen::VectorXd x = (prob.x0.size() == n) ? prob.x0 : Eigen::VectorXd::Zero(n);
  interiorize_initial_point(prob.vars, x);

  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  lb_cols.reserve(static_cast<size_t>(n));
  ub_cols.reserve(static_cast<size_t>(n));
  for (int j = 0; j < n; ++j) {
    if (is_effectively_finite(prob.vars[static_cast<size_t>(j)].lb)) lb_cols.push_back(j);
    if (is_effectively_finite(prob.vars[static_cast<size_t>(j)].ub)) ub_cols.push_back(j);
  }

  Eigen::VectorXd grad;
  Eigen::SparseMatrix<double> hess;
  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> jg;
  Eigen::VectorXd h;
  Eigen::SparseMatrix<double> jh;
  std::string status;
  DiagonalQNState qn_state;
  if (!prob.hess && !prob.lagrangian_hess) {
    initialize_quasi_newton_state(n, qn_state);
    qn_state.sparse_block_size = std::max(2, opt_.qn_sparse_block_size);
    qn_state.max_blocks = std::max(1, opt_.qn_max_blocks);
  }
  NLPState current_state;
  bool current_state_valid = false;

  if (!evaluate_nlp_state(prob, x, lb_cols, ub_cols, current_state, status)) {
    out.stats.status = status;
    if (opt_.allow_external_fallback &&
        try_ipopt_fallback(prob, out.stats.status, out, detail)) {
      out.stats.runtime_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }
    return {out, detail};
  }
  current_state_valid = true;
  grad = current_state.grad;
  g = current_state.g;
  jg = current_state.jg;
  h = current_state.h;
  jh = current_state.jh;
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(g.size());
  Eigen::VectorXd s;
  Eigen::VectorXd mu;
  initialize_barrier_state(h, s, mu);
  const bool has_equality_dual_start =
      opt_.equality_dual_start.size() == lambda.size() &&
      opt_.equality_dual_start.allFinite();
  const bool has_inequality_dual_start =
      opt_.inequality_dual_start.size() == mu.size() &&
      opt_.inequality_dual_start.allFinite() &&
      (opt_.inequality_dual_start.array() > 0.0).all();
  if (opt_.slack_start.size() == s.size() && opt_.slack_start.allFinite() &&
      (opt_.slack_start.array() > 0.0).all()) {
    s = opt_.slack_start.cwiseMax(
        Eigen::VectorXd::Constant(s.size(), 2.0 * kMinPositive));
    if (!has_inequality_dual_start) {
      for (int i = 0; i < mu.size(); ++i) {
        mu[i] = std::max(1.0 / s[i], 1e-2);
      }
    }
  }
  if (has_inequality_dual_start) {
    mu = opt_.inequality_dual_start.cwiseMax(
        Eigen::VectorXd::Constant(mu.size(), 2.0 * kMinPositive));
  }
  if (has_equality_dual_start) {
    lambda = opt_.equality_dual_start;
  } else if (opt_.least_square_init_duals && !qn_state.active) {
    initialize_equality_duals_least_squares(
        current_state, mu, opt_.constr_mult_init_max, lambda);
  }

  const Eigen::VectorXd mu_nonlinear0 =
      nonlinear_inequality_multipliers(mu, current_state.n_nonlinear_ineq);
  if (!build_lagrangian_hessian(prob, x, lambda, &mu_nonlinear0, hess, status,
                                qn_state.active ? &qn_state : nullptr)) {
    out.stats.status = status;
    if (opt_.allow_external_fallback &&
        try_ipopt_fallback(prob, out.stats.status, out, detail)) {
      out.stats.runtime_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    }
    return {out, detail};
  }
  current_state.hess = hess;

  bool converged = false;
  std::string native_status = "Max iterations reached";
  int iter_done = 0;
  int recovery_attempts = 0;
  int stall_iters = 0;
  double best_merit = std::numeric_limits<double>::infinity();
  IterateSnapshot best_iterate;
  const double comp_tol = opt_.tol_complementarity;

  for (int iter = 0; iter < opt_.max_iter; ++iter) {
    iter_done = iter + 1;

    if (!current_state_valid &&
        !evaluate_nlp_state(prob, x, lb_cols, ub_cols, current_state, status)) {
      native_status = status;
      break;
    }
    grad = current_state.grad;
    g = current_state.g;
    jg = current_state.jg;
    h = current_state.h;
    jh = current_state.jh;
    if (qn_state.active) {
      update_quasi_newton_state(x, grad, jg, jh, lambda, mu, qn_state);
    }
    const Eigen::VectorXd mu_nonlinear =
        nonlinear_inequality_multipliers(mu, current_state.n_nonlinear_ineq);
    if (!build_lagrangian_hessian(prob, x, lambda, &mu_nonlinear, hess, status,
                                  qn_state.active ? &qn_state : nullptr)) {
      native_status = status;
      break;
    }
    current_state.hess = hess;

    const Eigen::VectorXd r_dual = grad + jg.transpose() * lambda + jh.transpose() * mu;
    const Eigen::VectorXd r_eq = g;
    const Eigen::VectorXd r_ineq = h + s;
    const ResidualSummary cur = summarize_residuals(r_dual, r_eq, r_ineq, x, s, lambda, mu);

    out.stats.objective = current_state.obj_orig;
    out.stats.primal_feas = cur.primal_feas;
    out.stats.dual_feas = cur.dual_feas;
    out.stats.complementarity = cur.complementarity;
    out.stats.residual_inf = std::max(cur.primal_feas, cur.dual_feas);
    out.stats.iterations = iter;

    if (cur.merit + 1e-12 < best_merit) {
      best_merit = cur.merit;
      stall_iters = 0;
      best_iterate.valid = true;
      best_iterate.x = x;
      best_iterate.s = s;
      best_iterate.lambda = lambda;
      best_iterate.mu = mu;
      best_iterate.objective = current_state.obj_orig;
      best_iterate.residuals = cur;
      best_iterate.iteration = iter + 1;
    } else if (iter > 0) {
      ++stall_iters;
    }

    if (cur.primal_feas <= opt_.tol_primal &&
        cur.dual_feas <= opt_.tol_dual &&
        cur.complementarity <= comp_tol) {
      converged = true;
      native_status = "Converged";
      break;
    }

    if (stall_iters >= 6) {
      recenter_barrier_state(prob, h, x, s, mu, qn_state.active ? &qn_state : nullptr);
      stall_iters = 0;
      current_state_valid = false;
      continue;
    }

    Eigen::SparseMatrix<double> w = hess;
    if (jh.rows() > 0) {
      const Eigen::VectorXd d = mu.cwiseQuotient(
          s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)));
      Eigen::SparseMatrix<double> scaled_jh = scale_rows(jh, d);
      w += jh.transpose() * scaled_jh;
    }
    w.makeCompressed();

    Eigen::VectorXd rhs_aff_x = -r_dual;
    if (jh.rows() > 0) {
      const Eigen::VectorXd cent_aff = s.cwiseProduct(mu);
      rhs_aff_x -= jh.transpose() * ((-cent_aff + mu.cwiseProduct(r_ineq)).cwiseQuotient(s));
    }
    Eigen::VectorXd rhs_aff(rhs_aff_x.size() + r_eq.size());
    rhs_aff << rhs_aff_x, -r_eq;

    Eigen::VectorXd dx_aff;
    Eigen::VectorXd dlambda_aff;
    SparseKKTCache kkt_cache;
    if (!factor_kkt_with_regularization(w, jg, kkt_cache) ||
        !solve_kkt_sparse(kkt_cache, rhs_aff, dx_aff, dlambda_aff)) {
      if (recovery_attempts < 2) {
        ++recovery_attempts;
        recenter_barrier_state(prob, h, x, s, mu, qn_state.active ? &qn_state : nullptr);
        current_state_valid = false;
        continue;
      }
      native_status = "NativeIPM KKT factorization failed";
      break;
    }

    const Eigen::VectorXd ds_aff = -r_ineq - jh * dx_aff;
    Eigen::VectorXd dmu_aff;
    if (s.size() == 0) {
      dmu_aff = Eigen::VectorXd::Zero(0);
    } else {
      dmu_aff = (-s.cwiseProduct(mu) - mu.cwiseProduct(ds_aff)).cwiseQuotient(s);
    }

    const double alpha_aff_pri = max_positive_step(s, ds_aff, opt_.alpha_max);
    const double alpha_aff_dual = max_positive_step(mu, dmu_aff, opt_.alpha_max);
    const Eigen::VectorXd target_compl =
        complementarity_target(s, mu, ds_aff, dmu_aff, alpha_aff_pri, alpha_aff_dual);

    Eigen::VectorXd rhs_x = -r_dual;
    if (jh.rows() > 0) {
      const Eigen::VectorXd cent_corr =
          s.cwiseProduct(mu) + ds_aff.cwiseProduct(dmu_aff) -
          target_compl;
      rhs_x -= jh.transpose() * ((-cent_corr + mu.cwiseProduct(r_ineq)).cwiseQuotient(s));
    }
    Eigen::VectorXd rhs(rhs_x.size() + r_eq.size());
    rhs << rhs_x, -r_eq;

    Eigen::VectorXd dx;
    Eigen::VectorXd dlambda;
    if (!solve_kkt_sparse(kkt_cache, rhs, dx, dlambda)) {
      if (recovery_attempts < 2) {
        ++recovery_attempts;
        recenter_barrier_state(prob, h, x, s, mu, qn_state.active ? &qn_state : nullptr);
        current_state_valid = false;
        continue;
      }
      native_status = "NativeIPM corrected KKT solve failed";
      break;
    }

    Eigen::VectorXd ds = -r_ineq - jh * dx;
    Eigen::VectorXd dmu;
    if (s.size() == 0) {
      dmu = Eigen::VectorXd::Zero(0);
    } else {
      dmu = (-(s.cwiseProduct(mu) + ds_aff.cwiseProduct(dmu_aff) -
               target_compl) -
             mu.cwiseProduct(ds)).cwiseQuotient(s);
    }

    if (s.size() > 0 && mu.size() > 0) {
      constexpr int kMaxExtraCorrectors = 2;
      constexpr double kExtraCorrBeta = 0.1;
      for (int gc = 0; gc < kMaxExtraCorrectors; ++gc) {
        const double alpha_probe_primal =
            std::min(1.0, max_positive_step(s, ds, opt_.alpha_max));
        const double alpha_probe_dual =
            std::min(1.0, max_positive_step(mu, dmu, opt_.alpha_max));
        const Eigen::VectorXd s_probe = s + alpha_probe_primal * ds;
        const Eigen::VectorXd mu_probe = mu + alpha_probe_dual * dmu;
        const Eigen::VectorXd eps_gc =
            (kExtraCorrBeta * target_compl - s_probe.cwiseProduct(mu_probe))
                .cwiseMax(0.0);
        if (eps_gc.size() == 0 || eps_gc.maxCoeff() <= 1e-12) {
          break;
        }

        Eigen::VectorXd rhs_gc(rhs_x.size() + r_eq.size());
        rhs_gc.head(rhs_x.size()) =
            -(jh.transpose() *
              eps_gc.cwiseQuotient(
                  s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive))));
        rhs_gc.tail(r_eq.size()).setZero();

        Eigen::VectorXd dx_gc;
        Eigen::VectorXd dlambda_gc;
        const bool gc_ok =
            solve_kkt_sparse(kkt_cache, rhs_gc, dx_gc, dlambda_gc);
        if (!gc_ok) {
          break;
        }

        const Eigen::VectorXd ds_gc = -jh * dx_gc;
        const Eigen::VectorXd dmu_gc =
            eps_gc.cwiseQuotient(
                s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive))) -
            mu.cwiseQuotient(
                s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)))
                .cwiseProduct(ds_gc);

        dx += dx_gc;
        dlambda += dlambda_gc;
        ds += ds_gc;
        dmu += dmu_gc;
      }
    }

    double alpha_primal = std::min(1.0, max_positive_step(s, ds, opt_.alpha_max));
    double alpha_dual = std::min(1.0, max_positive_step(mu, dmu, opt_.alpha_max));
    if (!(alpha_primal > 0.0) || !std::isfinite(alpha_primal) ||
        !(alpha_dual > 0.0) || !std::isfinite(alpha_dual)) {
      if (recovery_attempts < 2) {
        ++recovery_attempts;
        recenter_barrier_state(prob, h, x, s, mu, qn_state.active ? &qn_state : nullptr);
        current_state_valid = false;
        continue;
      }
      native_status = "NativeIPM step length collapsed";
      break;
    }

    bool accepted = false;
    TrialPoint accepted_trial;
    for (int ls = 0; ls < kMaxBacktracks; ++ls) {
      const Eigen::VectorXd x_trial = x + alpha_primal * dx;
      const Eigen::VectorXd s_trial = s + alpha_primal * ds;
      const Eigen::VectorXd lambda_trial = lambda + alpha_dual * dlambda;
      const Eigen::VectorXd mu_trial = mu + alpha_dual * dmu;

      TrialPoint trial;
      if (!evaluate_trial_point(prob, lb_cols, ub_cols,
                                x_trial, s_trial, lambda_trial, mu_trial,
                                status, trial)) {
        alpha_primal *= 0.5;
        alpha_dual *= 0.5;
        continue;
      }

      const bool sufficient_progress =
          sufficient_primal_dual_progress(cur, trial.residuals,
                                          alpha_primal, alpha_dual);
      if (sufficient_progress) {
        accepted_trial = std::move(trial);
        accepted = true;
        break;
      }

      alpha_primal *= 0.5;
      alpha_dual *= 0.5;
    }

    if (!accepted) {
      if (recovery_attempts < 2) {
        ++recovery_attempts;
        recenter_barrier_state(prob, h, x, s, mu, qn_state.active ? &qn_state : nullptr);
        current_state_valid = false;
        continue;
      }
      native_status = "NativeIPM line search failed";
      break;
    }
    x = accepted_trial.x;
    s = accepted_trial.s;
    lambda = accepted_trial.lambda;
    mu = accepted_trial.mu;
    current_state = std::move(accepted_trial.state);
    current_state_valid = true;
    recovery_attempts = 0;
  }

  if (!converged && best_iterate.valid &&
      best_iterate.residuals.primal_feas <= opt_.tol_primal &&
      best_iterate.residuals.dual_feas <= opt_.tol_dual &&
      best_iterate.residuals.complementarity <= comp_tol) {
    x = best_iterate.x;
    s = best_iterate.s;
    lambda = best_iterate.lambda;
    mu = best_iterate.mu;
    converged = true;
    native_status = "Converged (best iterate restoration)";
    iter_done = best_iterate.iteration;
  }

  // Acceptable convergence is still a KKT condition: all three components
  // must meet the relaxed tolerance.
  if (!converged && best_iterate.valid && opt_.tol_accept > 0.0 &&
      best_iterate.residuals.primal_feas <= opt_.tol_accept &&
      best_iterate.residuals.dual_feas <= opt_.tol_accept &&
      best_iterate.residuals.complementarity <= opt_.tol_accept) {
    x = best_iterate.x;
    s = best_iterate.s;
    lambda = best_iterate.lambda;
    mu = best_iterate.mu;
    converged = true;
    native_status = "Converged (acceptable tolerance)";
    iter_done = best_iterate.iteration;
  }

  // A restored best iterate may not match current_state. Re-evaluate it before
  // reporting residuals and multipliers.
  if (converged &&
      !evaluate_nlp_state(prob, x, lb_cols, ub_cols, current_state, status)) {
    converged = false;
    native_status = status;
  }

  if (converged) {
    grad = current_state.grad;
    g = current_state.g;
    jg = current_state.jg;
    h = current_state.h;
    jh = current_state.jh;
    const int n_nonlinear_ineq = current_state.n_nonlinear_ineq;

    const Eigen::VectorXd r_dual = grad + jg.transpose() * lambda + jh.transpose() * mu;
    const Eigen::VectorXd r_eq = g;
    const Eigen::VectorXd r_ineq = h + s;
    const ResidualSummary fin = summarize_residuals(r_dual, r_eq, r_ineq, x, s, lambda, mu);

    out.x = x;
    out.stats.success = true;
    out.stats.status = native_status;
    out.stats.iterations = iter_done;
    out.stats.objective = objective_value(prob, x);
    out.stats.primal_feas = fin.primal_feas;
    out.stats.dual_feas = fin.dual_feas;
    out.stats.complementarity = fin.complementarity;
    out.stats.residual_inf = std::max(fin.primal_feas, fin.dual_feas);

    out.constraint_duals.resize(mu.size() + lambda.size());
    if (mu.size() > 0) {
      out.constraint_duals.head(mu.size()) = mu;
    }
    if (lambda.size() > 0) {
      out.constraint_duals.tail(lambda.size()) = lambda;
    }

    out.box_dual_lb = Eigen::VectorXd::Zero(n);
    out.box_dual_ub = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < static_cast<int>(lb_cols.size()); ++k) {
      out.box_dual_lb[lb_cols[static_cast<size_t>(k)]] = mu[n_nonlinear_ineq + k];
    }
    for (int k = 0; k < static_cast<int>(ub_cols.size()); ++k) {
      out.box_dual_ub[ub_cols[static_cast<size_t>(k)]] =
          mu[n_nonlinear_ineq + static_cast<int>(lb_cols.size()) + k];
    }

    detail.lambda_eq = lambda;
    detail.mu_ineq = mu;
    detail.z_slack = s;
    detail.complementarity = fin.complementarity;
  } else {
    if (best_iterate.valid) {
      x = best_iterate.x;
      s = best_iterate.s;
      lambda = best_iterate.lambda;
      mu = best_iterate.mu;
      out.stats.objective = best_iterate.objective;
      out.stats.primal_feas = best_iterate.residuals.primal_feas;
      out.stats.dual_feas = best_iterate.residuals.dual_feas;
      out.stats.complementarity = best_iterate.residuals.complementarity;
      out.stats.residual_inf = std::max(best_iterate.residuals.primal_feas,
                                        best_iterate.residuals.dual_feas);
      native_status += " (best iterate restored)";
    }
    out.x = x;
    out.stats.success = false;
    out.stats.status = native_status;
    out.stats.iterations = iter_done;
    if (!best_iterate.valid) {
      out.stats.objective = current_state_valid ? current_state.obj_orig : objective_value(prob, x);
    }
    if (opt_.allow_external_fallback &&
        try_ipopt_fallback(prob, native_status, out, detail)) {
      out.stats.runtime_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      return {out, detail};
    }
  }

  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return {out, detail};
}

}  // namespace mipsolvers::engine
