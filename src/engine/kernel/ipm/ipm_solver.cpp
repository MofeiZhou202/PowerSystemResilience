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
};

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
  for (int i = 0; i < mu_ineq.size(); ++i) {
    mu_ineq[i] = std::clamp(mu_bar / std::max(s[i], kMinPositive), 1e-4, 1e4);
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
      result.symbolic_analyses = augmented_cache.kkt.symbolic_analyses;
      result.numeric_factorizations =
          augmented_cache.kkt.numeric_factorizations;
      result.linear_solves = augmented_cache.kkt.linear_solves;
    } else if (opt.use_inertia_correction) {
      result.symbolic_analyses = kkt_cache.augmented.symbolic_analyses;
      result.numeric_factorizations =
          kkt_cache.augmented.numeric_factorizations;
      result.linear_solves = kkt_cache.augmented.linear_solves;
    } else {
      result.symbolic_analyses = regularized_kkt_cache.symbolic_analyses;
      result.numeric_factorizations =
          regularized_kkt_cache.numeric_factorizations;
      result.linear_solves = regularized_kkt_cache.linear_solves;
    }
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
      const bool use_augmented_newton =
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
      if (!kkt_ok) {
        snapshot_outcome(false, total_iters + 1,
                         opt.use_inertia_correction
                             ? "Filter: KKT inertia correction cap exceeded"
                             : "Filter: regularized KKT factorization failed",
                         rs);
        return result;
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
      if (s.size() > 0) {
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
      const double alpha_max_primal =
          (s.size() == 0) ? 1.0 : max_positive_step(s, ds, tau);
      const double alpha_max_dual =
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
      double alpha = std::min(1.0, alpha_max_primal);

      // Filter line search.
      const double theta_k = compute_theta(state.g, state.h, s);
      const double phi_k = compute_barrier_phi(state.obj_orig, mu_bar, s);
      const double slope_k =
          barrier_descent_slope(state.grad, dx, s, ds, mu_bar);
      const double theta_min =
          opt.filter_theta_min_scale * std::max(1.0, theta_k);

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
        const Eigen::VectorXd x_trial = x + alpha * dx;
        Eigen::VectorXd s_trial =
          (s.size() == 0) ? Eigen::VectorXd() : Eigen::VectorXd(s + alpha * ds);
        const Eigen::VectorXd lambda_trial = lambda + alpha_dual * dlambda;
        const Eigen::VectorXd mu_trial = mu_ineq + alpha_dual * dmu_ineq;

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
                  rs.complementarity * (1.0 - 1e-4 * alpha_dual) + 1e-14;
          const bool dual_progress =
              trial.residuals.dual_feas <=
                  rs.dual_feas * (1.0 - 1e-4 * alpha_dual) + 1e-14;
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
                        << ", alpha_dual=" << alpha_dual
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
                      << ", alpha_dual=" << alpha_dual
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
                  lambda + alpha_dual * dlambda + dlambda_soc;
              const Eigen::VectorXd mu_soc_trial =
                  mu_ineq + alpha_dual * dmu_ineq + dmu_soc_extra;
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

  const ValidationReport vr = validate(prob);
  if (!vr.valid) {
    out.stats.status = vr.errors.empty() ? "Invalid NLP model" : vr.errors.front();
    return {out, detail};
  }

  if (!prob.f || !prob.grad) {
    out.stats.status = "NLP model missing objective callbacks";
    return {out, detail};
  }

  // Filter-based driver (Wächter–Biegler). Opt-in via IPMOptions::globalization.
  if (opt_.globalization == Globalization::Filter) {
    const int n_f = static_cast<int>(prob.vars.size());

    // --- Scaling: compute factors at the interiorized x0 and wrap the model.
    ScalingFactors sf;
    const NLPModel* active_prob = &prob;
    NLPModel scaled_prob;
    const bool do_scale = opt_.scale_problem;
    if (do_scale) {
      Eigen::VectorXd x0_interior = (prob.x0.size() == n_f)
                                        ? prob.x0
                                        : Eigen::VectorXd::Zero(n_f);
      interiorize_initial_point(prob.vars, x0_interior);
      sf = compute_scaling_factors(prob, x0_interior, opt_.scaling_g_max);
      scaled_prob = build_scaled_nlp_model(prob, sf);
      active_prob = &scaled_prob;
    }

    FilterSolveOutcome fo = solve_nlp_filter_impl(*active_prob, opt_);

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
        RestorationBuild rb =
            build_restoration_nlp(prob, x_R, opt_.restoration_zeta);
        IPMOptions rst_opt = opt_;
        rst_opt.use_restoration_phase = false;
        rst_opt.use_second_order_correction = false;
        rst_opt.scale_problem = false;
        rst_opt.globalization = Globalization::Filter;
        rst_opt.max_iter = std::min(opt_.max_iter, 200);

        FilterSolveOutcome rf = solve_nlp_filter_impl(rb.model, rst_opt);

        if (rf.x.size() >= rb.n_x) {
          Eigen::VectorXd x_new = extract_x_from_restoration(rb, rf.x);
          NLPModel prob_retry = prob;
          prob_retry.x0 = x_new;

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

          FilterSolveOutcome fo_retry =
              solve_nlp_filter_impl(*retry_prob, retry_opt);
          // Only adopt the retry if it did at least as well as the original.
          if (fo_retry.converged ||
              fo_retry.final_residuals.merit < fo.final_residuals.merit) {
            fo = fo_retry;
            sf = sf2;
          }
        }
      }
    }

    // --- Unscale multipliers back to the original problem's coordinates.
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
        }
      } else if (fo.converged) {
        fo.converged = false;
        fo.status = "Scaled solve could not evaluate unscaled KKT: " +
                    original_status;
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
    detail.symbolic_analyses = fo.symbolic_analyses;
    detail.numeric_factorizations = fo.numeric_factorizations;
    detail.linear_solves = fo.linear_solves;

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
      IPMOptions merit_opt = opt_;
      merit_opt.globalization = Globalization::Merit;
      merit_opt.use_restoration_phase = false;
      merit_opt.use_second_order_correction = false;
      merit_opt.scale_problem = false;
      NativeIPMAdapter merit_solver(merit_opt);
      auto merit_pair = merit_solver.solve_nlp_detail(prob);
      if (merit_pair.first.stats.success) {
        if (merit_pair.first.stats.solver_name == name()) {
          merit_pair.first.stats.solver_name = "NativeIPM[MeritFallback]";
          merit_pair.first.stats.status =
              "Native Merit fallback after Filter failure: " + fo.status +
              "; " + merit_pair.first.stats.status;
        }
        merit_pair.first.stats.runtime_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return merit_pair;
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
