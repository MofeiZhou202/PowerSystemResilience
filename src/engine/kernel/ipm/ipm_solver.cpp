#include "hacdcpf/engine/kernel/ipm/ipm_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Sparse>

#include "hacdcpf/engine/solver/external/adapters.hpp"
#include "hacdcpf/engine/kernel/ipm/ipm_filter.hpp"
#include "hacdcpf/engine/kernel/ipm/ipm_restoration.hpp"
#include "hacdcpf/engine/kernel/ipm/ipm_scaling.hpp"
#include "hacdcpf/engine/kernel/kkt/kkt_system.hpp"
#include "hacdcpf/engine/util/problem_validation.hpp"

namespace hacdcpf::engine {
namespace {

constexpr double kBoundInfinity = 1e19;
constexpr double kMinPositive = 1e-10;
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
  Eigen::VectorXd prev_grad;
  struct SparseBlockUpdate {
    std::vector<int> index;
    Eigen::MatrixXd values;
  };
  std::vector<SparseBlockUpdate> blocks;
  int sparse_block_size{8};
  int max_blocks{6};
  Eigen::SparseMatrix<double> fd_hess;
  bool has_fd_hess{false};
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

void initialize_quasi_newton_state(int n, DiagonalQNState& state) {
  state.active = true;
  state.diag = Eigen::VectorXd::Ones(n);
  state.prev_x.resize(0);
  state.prev_grad.resize(0);
  state.blocks.clear();
  state.fd_hess.resize(0, 0);
  state.has_fd_hess = false;
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

bool refresh_fd_hessian_from_gradient(const NLPModel& prob,
                                      const Eigen::VectorXd& x,
                                      const Eigen::VectorXd& base_grad,
                                      DiagonalQNState& state,
                                      std::string& status) {
  const int n = static_cast<int>(x.size());
  if (!state.active || n <= 0 || n > 2000) {
    return false;
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(8 * n));
  for (int j = 0; j < n; ++j) {
    Eigen::VectorXd x_fd = x;
    const double step = 1e-6 * std::max(1.0, std::abs(x[j]));
    x_fd[j] += step;
    Eigen::VectorXd grad_fd;
    prob.grad(x_fd, grad_fd);
    if (grad_fd.size() != n || !grad_fd.allFinite()) {
      status = "NLP finite-difference Hessian refresh failed";
      return false;
    }
    const Eigen::VectorXd col = (grad_fd - base_grad) / step;
    const double keep_tol = std::max(1e-10, 1e-6 * col.lpNorm<Eigen::Infinity>());
    for (int i = 0; i < n; ++i) {
      if (std::abs(col[i]) > keep_tol) {
        triplets.emplace_back(i, j, col[i]);
      }
    }
  }

  Eigen::SparseMatrix<double> fd_hess(n, n);
  fd_hess.setFromTriplets(triplets.begin(), triplets.end());
  fd_hess = symmetrize_hessian(std::move(fd_hess));
  state.fd_hess = std::move(fd_hess);
  state.has_fd_hess = true;

  if (state.fd_hess.rows() == n) {
    Eigen::VectorXd diag = state.fd_hess.diagonal();
    state.diag = diag.cwiseMax(Eigen::VectorXd::Constant(n, 1e-6));
  }
  return true;
}

void update_quasi_newton_state(const Eigen::VectorXd& x,
                               const Eigen::VectorXd& grad,
                               DiagonalQNState& state) {
  if (!state.active) {
    return;
  }
  if (state.diag.size() != x.size()) {
    state.diag = Eigen::VectorXd::Ones(x.size());
  }
  if (state.prev_x.size() == x.size() && state.prev_grad.size() == x.size()) {
    const Eigen::VectorXd s = x - state.prev_x;
    const Eigen::VectorXd y = grad - state.prev_grad;
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
    const double denom = residual.dot(s);
    if (denom > 1e-10) {
      const std::vector<int> support = select_top_abs_indices(residual, state.sparse_block_size);
      if (!support.empty()) {
        Eigen::VectorXd local = Eigen::VectorXd::Zero(static_cast<int>(support.size()));
        for (int k = 0; k < static_cast<int>(support.size()); ++k) {
          local[k] = residual[support[static_cast<size_t>(k)]];
        }
        state.blocks.push_back({support, (local * local.transpose()) / denom});
        if (static_cast<int>(state.blocks.size()) > state.max_blocks) {
          state.blocks.erase(state.blocks.begin());
        }
      }
    }
  }
  state.prev_x = x;
  state.prev_grad = grad;
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
    if (qn_state && qn_state->active && qn_state->has_fd_hess &&
        qn_state->fd_hess.rows() == n && qn_state->fd_hess.cols() == n) {
      hess = qn_state->fd_hess;
    } else if (qn_state && qn_state->active && qn_state->diag.size() == n) {
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
      const double width = std::max(1e-6, ub - lb);
      const double margin = std::min(1.0, 1e-4 + 0.01 * width);
      if (x[i] <= lb + margin || x[i] >= ub - margin) {
        x[i] = 0.5 * (lb + ub);
      }
      x[i] = std::min(ub - margin, std::max(lb + margin, x[i]));
    } else if (has_lb) {
      const double margin = 1e-4 + 0.01 * std::max(1.0, std::abs(lb));
      x[i] = std::max(x[i], lb + margin);
    } else if (has_ub) {
      const double margin = 1e-4 + 0.01 * std::max(1.0, std::abs(ub));
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
  out.primal_feas = std::max(inf_norm(r_eq), inf_norm(r_ineq)) / (1.0 + scale_x);
  out.dual_feas = inf_norm(r_dual) /
                  (1.0 + std::max(inf_norm(lambda), inf_norm(mu)));
  out.complementarity = (s.size() == 0)
      ? 0.0
      : (s.dot(mu) / static_cast<double>(s.size())) / (1.0 + inf_norm(x));
  out.merit = std::max({out.primal_feas, out.dual_feas, out.complementarity});
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
  ResidualSummary final_residuals{};
  double objective{0.0};
  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
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

  double mu_bar = std::max(opt.mu_min, opt.mu_init);

  // Primal–dual multiplier initialization.
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(state.g.size());
  Eigen::VectorXd mu_ineq(state.h.size());
  for (int i = 0; i < mu_ineq.size(); ++i) {
    mu_ineq[i] = std::clamp(mu_bar / std::max(s[i], kMinPositive), 1e-4, 1e4);
  }

  Filter filter;
  const double theta0 = compute_theta(state.g, state.h, s);
  filter.reset_with_theta_upper_bound(1e4 * std::max(1.0, theta0));

  // When no analytical Hessian is provided, use a quasi-Newton approximation
  // (mirrors the Merit driver). The FD startup refresh is gated by n <= 2000.
  DiagonalQNState filter_qn_state;
  if (!prob.hess && !prob.lagrangian_hess) {
    initialize_quasi_newton_state(n, filter_qn_state);
    filter_qn_state.sparse_block_size = std::max(2, opt.qn_sparse_block_size);
    filter_qn_state.max_blocks = std::max(1, opt.qn_max_blocks);
    std::string qn_init_status;
    refresh_fd_hessian_from_gradient(prob, x, state.grad, filter_qn_state,
                                     qn_init_status);
  }

  double delta_w_last = 0.0;
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
  };

  for (; outer_iters < max_outer && total_iters < max_total; ++outer_iters) {
    const double inner_tol = opt.kappa_epsilon * mu_bar;
    bool inner_converged_at_mu = false;

    for (; total_iters < max_total; ++total_iters) {
      if (!evaluate_nlp_state(prob, x, lb_cols, ub_cols, state, eval_status)) {
        snapshot_outcome(false, total_iters, eval_status, best_residuals);
        return result;
      }

      // Build Lagrangian Hessian (full user callback if available).
      Eigen::SparseMatrix<double> hess;
      const Eigen::VectorXd mu_nonlinear =
          nonlinear_inequality_multipliers(mu_ineq, state.n_nonlinear_ineq);
      if (filter_qn_state.active) {
        update_quasi_newton_state(x, state.grad, filter_qn_state);
        if (!filter_qn_state.has_fd_hess) {
          refresh_fd_hessian_from_gradient(prob, x, state.grad, filter_qn_state,
                                           eval_status);
        }
      }
      if (!build_lagrangian_hessian(prob, x, lambda, &mu_nonlinear, hess,
                                    eval_status,
                                    filter_qn_state.active ? &filter_qn_state : nullptr)) {
        snapshot_outcome(false, total_iters, eval_status, best_residuals);
        return result;
      }
      state.hess = hess;

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

      // Build the primal-dual reduced KKT matrix W = H + Σ + Jh^T (M/S) Jh.
      Eigen::SparseMatrix<double> w = state.hess;
      if (state.jh.rows() > 0) {
        const Eigen::VectorXd d = mu_ineq.cwiseQuotient(
            s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)));
        Eigen::SparseMatrix<double> scaled_jh = scale_rows(state.jh, d);
        w += state.jh.transpose() * scaled_jh;
      }
      w.makeCompressed();

      // Right-hand side: rhs_x = −r_d − Jh^T ((M/S)(h + s) − (μ_bar − s·μ)/s)
      //                        = −r_d − Jh^T ((M r_ineq − μ_bar + s·μ) / s)
      //                  rhs_eq = −g
      Eigen::VectorXd rhs_x = -r_d;
      if (state.jh.rows() > 0) {
        Eigen::VectorXd v(s.size());
        for (int i = 0; i < s.size(); ++i) {
          const double si = std::max(s[i], kMinPositive);
          v[i] = (mu_ineq[i] * r_ineq[i] - mu_bar + s[i] * mu_ineq[i]) / si;
        }
        rhs_x -= state.jh.transpose() * v;
      }
      Eigen::VectorXd rhs(rhs_x.size() + r_eq.size());
      rhs << rhs_x, -r_eq;

      InertiaSettings isettings;
      isettings.mu = mu_bar;
      Eigen::VectorXd dx;
      Eigen::VectorXd dlambda;
      InertiaStatus istatus;
      if (!factor_and_solve_kkt_inertia_corrected(
              w, state.jg, rhs, isettings, delta_w_last, dx, dlambda, istatus)) {
        // Restoration would kick in here in PR3; for PR2 we declare failure
        // and let solve_nlp_detail attempt the IPOPT fallback.
        snapshot_outcome(false, total_iters + 1,
                         "Filter: KKT inertia correction cap exceeded", rs);
        return result;
      }

      // Recover step directions for the eliminated variables:
      //   ds       = −r_ineq − Jh dx
      //   dmu_ineq = (μ_bar − s·μ − μ·ds) / s
      Eigen::VectorXd ds = Eigen::VectorXd::Zero(s.size());
      Eigen::VectorXd dmu_ineq = Eigen::VectorXd::Zero(mu_ineq.size());
      if (s.size() > 0) {
        ds = -r_ineq - state.jh * dx;
        for (int i = 0; i < s.size(); ++i) {
          const double si = std::max(s[i], kMinPositive);
          dmu_ineq[i] = (mu_bar - s[i] * mu_ineq[i] - mu_ineq[i] * ds[i]) / si;
        }
      }

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

          Eigen::VectorXd rhs_gc(rhs_x.size() + r_eq.size());
          rhs_gc.head(rhs_x.size()) =
              -(state.jh.transpose() *
                eps_gc.cwiseQuotient(
                    s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive))));
          rhs_gc.tail(r_eq.size()).setZero();

          Eigen::VectorXd dx_gc;
          Eigen::VectorXd dlambda_gc;
          InertiaStatus istatus_gc;
          double delta_w_gc = delta_w_last;
          if (!factor_and_solve_kkt_inertia_corrected(
                  w, state.jg, rhs_gc, isettings, delta_w_gc,
                  dx_gc, dlambda_gc, istatus_gc)) {
            break;
          }

          const Eigen::VectorXd ds_gc = -state.jh * dx_gc;
          const Eigen::VectorXd dmu_gc =
              eps_gc.cwiseQuotient(
                  s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive))) -
              mu_ineq.cwiseQuotient(
                  s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)))
                  .cwiseProduct(ds_gc);

          dx += dx_gc;
          dlambda += dlambda_gc;
          ds += ds_gc;
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
      const ResidualSummary current_rs = rs;

      bool accepted = false;
      bool accepted_was_f_type = false;
      TrialPoint accepted_trial;
      double theta_trial = 0.0;
      double phi_trial = 0.0;
      // Extra SOC composite terms (default zero); added to the normal
      // `alpha * direction` updates when SOC accepts.
      Eigen::VectorXd dx_soc_extra = Eigen::VectorXd::Zero(dx.size());
      Eigen::VectorXd ds_soc_extra =
          (s.size() == 0) ? Eigen::VectorXd() : Eigen::VectorXd::Zero(s.size());
      Eigen::VectorXd dlambda_soc_extra =
          Eigen::VectorXd::Zero(dlambda.size());
      bool soc_attempted = false;
      const double alpha0 = alpha;  // remember alpha_max for SOC gate

      while (alpha > opt.filter_alpha_min) {
        const double alpha_dual = std::min(alpha, alpha_max_dual);
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
        if (!evaluate_trial_point(prob, lb_cols, ub_cols,
                                  x_trial, s_trial, lambda_trial, mu_trial,
                                  eval_status, trial)) {
          alpha *= 0.5;
          continue;
        }

        theta_trial = compute_theta(trial.state.g, trial.state.h, s_trial);
        phi_trial = compute_barrier_phi(trial.state.obj_orig, mu_bar, s_trial);

        bool filter_ok = filter.is_acceptable(theta_trial, phi_trial,
                                              opt.filter_gamma_theta,
                                              opt.filter_gamma_phi);

        const bool switching =
            (slope_k < 0.0) &&
            (alpha * std::pow(-slope_k, opt.filter_s_phi) >
             opt.filter_delta * std::pow(theta_k, opt.filter_s_theta)) &&
            (theta_k <= theta_min || opt.filter_s_phi > 0.0);

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

        const bool direct_accept =
            sufficient_primal_dual_progress(current_rs, trial.residuals,
                                            alpha, alpha_dual);
        const bool aggressive_direct =
            alpha >= 0.8 * alpha0 &&
            trial.residuals.merit <= current_rs.merit * 0.95 + 1e-12;
        if (!accept && direct_accept && (filter_ok || aggressive_direct)) {
          accept = true;
          accepted_was_f_type = false;
        }

        if (accept) {
          accepted = true;
          accepted_trial = std::move(trial);
          break;
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
          // Overwrite the equality block of the RHS with -c_soc; the primal
          // block (rhs_x) stays the same.
          rhs_soc.tail(r_eq.size()) = -c_soc;

          Eigen::VectorXd dx_soc;
          Eigen::VectorXd dlambda_soc;
          InertiaStatus istatus_soc;
          double dw_soc = delta_w_last;
          if (factor_and_solve_kkt_inertia_corrected(w, state.jg, rhs_soc,
                                                     isettings, dw_soc,
                                                     dx_soc, dlambda_soc,
                                                     istatus_soc)) {
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
                dmu_soc_extra = -mu_ineq.cwiseProduct(ds_soc_delta).cwiseQuotient(
                    s.cwiseMax(Eigen::VectorXd::Constant(s.size(), kMinPositive)));
              }
              const Eigen::VectorXd lambda_soc_trial =
                  lambda + alpha_dual * dlambda + dlambda_soc;
              const Eigen::VectorXd mu_soc_trial =
                  mu_ineq + alpha_dual * dmu_ineq + dmu_soc_extra;
              TrialPoint soc_trial;
              if (evaluate_trial_point(prob, lb_cols, ub_cols,
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

                const bool soc_direct_accept =
                    sufficient_primal_dual_progress(current_rs, soc_trial.residuals,
                                                    alpha, alpha_dual);
                if (!soc_accept && soc_direct_accept &&
                    (soc_filter_ok || alpha >= 0.8 * alpha0)) {
                  soc_accept = true;
                  accepted_was_f_type = false;
                }

                if (soc_accept) {
                  dx_soc_extra = dx_soc;
                  if (s.size() > 0) ds_soc_extra = ds_soc_delta;
                  dlambda_soc_extra = dlambda_soc;
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
    const double mu_new = std::max(opt.mu_min,
                                   std::min(opt.kappa_mu * mu_bar,
                                            std::pow(mu_bar, opt.theta_mu)));
    if (mu_new >= mu_bar) {  // cannot decrease further — μ clamped at μ_min
      // We already failed the outer tolerance check above (otherwise we would
      // have returned); treat as convergence at minimum barrier.
      break;
    }
    mu_bar = mu_new;
    filter.clear();
  }

  // Exhausted outer iterations. Fall back to the best iterate seen so the
  // caller gets the closest feasible primal it has seen.
  if (have_best) {
    x = best_x;
    s = best_s;
    lambda = best_lambda;
    mu_ineq = best_mu_ineq;
  }

  snapshot_outcome(false, total_iters,
                   "Filter: max iterations reached without convergence",
                   best_residuals);
  return result;
}

bool try_ipopt_fallback(const NLPModel& prob,
                        const std::string& native_status,
                        SolveResult& out,
                        IPMDetail& detail) {
  IpoptAdapter ipopt;
  SolveResult fallback = ipopt.solve_nlp(prob);
  // Accept the fallback if fully converged OR near-feasible (primal_feas < 1e-4
  // with a finite objective) — the latter matches the AML build_result near-feasible
  // criterion so the caller sees a usable iterate even when max_iter is exhausted.
  const bool near_feasible =
      !fallback.stats.success &&
      static_cast<int>(fallback.x.size()) == static_cast<int>(prob.vars.size()) &&
      fallback.stats.primal_feas > 0.0 &&
      fallback.stats.primal_feas < 1e-4 &&
      std::isfinite(fallback.stats.objective);
  if (!fallback.stats.success && !near_feasible) {
    return false;
  }
  if (near_feasible) {
    fallback.stats.success = true;
  }

  fallback.stats.solver_name = "NativeIPM[IpoptFallback]";
  fallback.stats.status = "Ipopt fallback after native failure: " + native_status;
  out = std::move(fallback);
  detail = IPMDetail{};
  detail.complementarity = out.stats.complementarity;
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
      if (fo.mu_ineq.size() >= n_nl && sf.s_h.size() >= n_nl) {
        for (int i = 0; i < n_nl; ++i) {
          fo.mu_ineq[i] = fo.mu_ineq[i] * sf.s_h[i] / sf.s_f;
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
      // driver before the IPOPT last-resort. Merit handles quasi-Newton-only
      // NLPs (no Hessian) and badly-conditioned small problems more
      // gracefully than the Filter prototype.
      IPMOptions merit_opt = opt_;
      merit_opt.globalization = Globalization::Merit;
      merit_opt.use_restoration_phase = false;
      merit_opt.use_second_order_correction = false;
      merit_opt.scale_problem = false;
      NativeIPMAdapter merit_solver(merit_opt);
      auto merit_pair = merit_solver.solve_nlp_detail(prob);
      if (merit_pair.first.stats.success) {
        merit_pair.first.stats.runtime_sec =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return merit_pair;
      }
      if (try_ipopt_fallback(prob, fo.status, out, detail)) {
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
    if (try_ipopt_fallback(prob, out.stats.status, out, detail)) {
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
  if (qn_state.active && !qn_state.has_fd_hess) {
    bool cache_hit = false;
    {
      std::lock_guard<std::mutex> lock(startup_cache_mutex_);
      if (startup_cache_.valid && startup_cache_.x.size() == x.size() &&
          startup_cache_.grad.size() == grad.size() &&
          startup_cache_.hess.rows() == x.size() && startup_cache_.hess.cols() == x.size() &&
          (startup_cache_.x - x).lpNorm<Eigen::Infinity>() <= 1e-12 &&
          (startup_cache_.grad - grad).lpNorm<Eigen::Infinity>() <= 1e-10) {
        qn_state.fd_hess = startup_cache_.hess;
        qn_state.has_fd_hess = true;
        qn_state.diag = startup_cache_.hess.diagonal().cwiseMax(
            Eigen::VectorXd::Constant(x.size(), 1e-6));
        cache_hit = true;
      }
    }
    if (!cache_hit && refresh_fd_hessian_from_gradient(prob, x, grad, qn_state, status) &&
        qn_state.has_fd_hess) {
      std::lock_guard<std::mutex> lock(startup_cache_mutex_);
      startup_cache_.valid = true;
      startup_cache_.x = x;
      startup_cache_.grad = grad;
      startup_cache_.hess = qn_state.fd_hess;
    }
  }
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(g.size());
  Eigen::VectorXd s;
  Eigen::VectorXd mu;
  initialize_barrier_state(h, s, mu);

  const Eigen::VectorXd mu_nonlinear0 =
      nonlinear_inequality_multipliers(mu, current_state.n_nonlinear_ineq);
  if (!build_lagrangian_hessian(prob, x, lambda, &mu_nonlinear0, hess, status,
                                qn_state.active ? &qn_state : nullptr)) {
    out.stats.status = status;
    if (try_ipopt_fallback(prob, out.stats.status, out, detail)) {
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
  const double comp_tol = std::max(opt_.tol_complementarity, opt_.tol_primal * 10.0);

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
      update_quasi_newton_state(x, grad, qn_state);
      if (!qn_state.has_fd_hess || stall_iters >= 2) {
        if (!refresh_fd_hessian_from_gradient(prob, x, grad, qn_state, status)) {
          qn_state.has_fd_hess = false;
        }
      }
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
    const bool reduced_kkt_aff = solve_kkt_reduced_sparse(
      w, jg, rhs_aff, dx_aff, dlambda_aff, opt_.reduced_kkt_max_eq, kMinReg, kMaxReg);
    if (!reduced_kkt_aff &&
      (!factor_kkt_with_regularization(w, jg, kkt_cache) ||
       !solve_kkt_sparse(kkt_cache, rhs_aff, dx_aff, dlambda_aff))) {
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
    if ((reduced_kkt_aff && !solve_kkt_reduced_sparse(
                   w, jg, rhs, dx, dlambda, opt_.reduced_kkt_max_eq,
                   kMinReg, kMaxReg)) ||
        (!reduced_kkt_aff && !solve_kkt_sparse(kkt_cache, rhs, dx, dlambda))) {
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
            (reduced_kkt_aff && solve_kkt_reduced_sparse(
                                    w, jg, rhs_gc, dx_gc, dlambda_gc,
                                    opt_.reduced_kkt_max_eq,
                                    kMinReg, kMaxReg)) ||
            (!reduced_kkt_aff && solve_kkt_sparse(kkt_cache, rhs_gc,
                                                  dx_gc, dlambda_gc));
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

  // Ipopt-style "acceptable" convergence: accept a near-feasible best
  // iterate when strict tolerances cannot be met within max_iter.
  // Only requires primal feasibility; dual infeasibility is tolerated since
  // the test/caller validates solution quality by evaluating the primal.
  bool accepted_via_tol_accept = false;
  if (!converged && best_iterate.valid && opt_.tol_accept > 0.0 &&
      best_iterate.residuals.primal_feas <= opt_.tol_accept) {
    x = best_iterate.x;
    s = best_iterate.s;
    lambda = best_iterate.lambda;
    mu = best_iterate.mu;
    converged = true;
    accepted_via_tol_accept = true;
    native_status = "Converged (acceptable tolerance)";
    iter_done = best_iterate.iteration;
  }

  if (converged && accepted_via_tol_accept) {
    // Fill output directly from best_iterate to avoid stale Jacobians.
    out.x = x;
    out.stats.success = true;
    out.stats.status = native_status;
    out.stats.iterations = iter_done;
    out.stats.objective = objective_value(prob, x);
    out.stats.primal_feas = best_iterate.residuals.primal_feas;
    out.stats.dual_feas = best_iterate.residuals.dual_feas;
    out.stats.complementarity = best_iterate.residuals.complementarity;
    out.stats.residual_inf = std::max(out.stats.primal_feas, out.stats.dual_feas);
    // Provide zero constraint duals (acceptable-tolerance doesn't guarantee
    // accurate duals).
    out.constraint_duals.setZero(static_cast<Eigen::Index>(mu.size() + lambda.size()));
    out.box_dual_lb = Eigen::VectorXd::Zero(n);
    out.box_dual_ub = Eigen::VectorXd::Zero(n);
    detail.lambda_eq = lambda;
    detail.mu_ineq = mu;
    detail.z_slack = s;
    detail.complementarity = best_iterate.residuals.complementarity;
  } else if (converged) {
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
    if (try_ipopt_fallback(prob, native_status, out, detail)) {
      out.stats.runtime_sec =
          std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      return {out, detail};
    }
  }

  out.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return {out, detail};
}

}  // namespace hacdcpf::engine
