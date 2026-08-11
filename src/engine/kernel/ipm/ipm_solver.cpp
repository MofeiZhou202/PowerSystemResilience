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
#include <Eigen/SparseQR>

#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_filter.hpp"
#include "mipsolvers/engine/kernel/ipm/lcqp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_restoration.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_scaling.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/cholmod_ldlt.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

double minimum_safe_positive() {
  // sqrt(min_normal) keeps both the value and its reciprocal representable.
  return std::sqrt(std::numeric_limits<double>::min());
}

const double kMinPositive = minimum_safe_positive();

double strict_minimum_safe_positive() {
  return std::nextafter(kMinPositive,
                        std::numeric_limits<double>::infinity());
}

bool is_effectively_finite(double value) {
  return variable_has_finite_lower_bound(value) &&
         variable_has_finite_upper_bound(value);
}

double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

double sparse_abs_max(const Eigen::SparseMatrix<double>& matrix) {
  double scale = 0.0;
  for (int k = 0; k < matrix.nonZeros(); ++k) {
    scale = std::max(scale, std::abs(matrix.valuePtr()[k]));
  }
  return scale;
}

double comparison_roundoff(double scale) {
  return std::sqrt(std::numeric_limits<double>::epsilon()) *
      std::max(1.0, std::abs(scale));
}

double arithmetic_roundoff(double scale) {
  return std::numeric_limits<double>::epsilon() *
      std::max(1.0, std::abs(scale));
}

// Progress-detection roundoff for two residuals of the same quantity. Unlike
// comparison_roundoff it has no unit floor, so a strict improvement between two
// small same-scale residuals (e.g. two ~1e-10 dual feasibilities) is not masked
// by an absolute sqrt(eps) margin.
double relative_roundoff(double a, double b) {
  return std::sqrt(std::numeric_limits<double>::epsilon()) *
      std::max(std::abs(a), std::abs(b));
}

double effective_fraction_to_boundary(double caller_policy) {
  const double root_epsilon =
      std::sqrt(std::numeric_limits<double>::epsilon());
  if (caller_policy > 0.0 && caller_policy < 1.0 &&
      std::isfinite(caller_policy)) {
    return caller_policy;
  }
  return 1.0 - root_epsilon;
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
  int sparse_block_size{0};
  int max_blocks{0};
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
    Eigen::VectorXd& lambda) {
  const int meq = static_cast<int>(state.jg.rows());
  if (meq == 0 || state.jg.cols() != state.grad.size() ||
      lambda.size() != meq || mu.size() != state.jh.rows()) {
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
      candidate.size() != meq || !candidate.allFinite()) {
    return false;
  }

  const Eigen::SparseMatrix<double> jg_transpose = state.jg.transpose();
  const double current_residual = inf_norm(
      stationarity_without_equalities + jg_transpose * lambda);
  const double candidate_residual = inf_norm(
      stationarity_without_equalities + jg_transpose * candidate);
  const double required_improvement = comparison_roundoff(current_residual);
  if (candidate_residual + required_improvement >= current_residual) {
    return false;
  }

  lambda = candidate;
  return true;
}

bool initialize_rank_deficient_equality_duals(
    const NLPState& state, const Eigen::VectorXd& mu,
    Eigen::VectorXd& lambda) {
  const int meq = static_cast<int>(state.jg.rows());
  if (meq == 0 || mu.size() != state.jh.rows() || lambda.size() != meq) {
    return false;
  }
  Eigen::SparseMatrix<double> transpose = state.jg.transpose();
  Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
  qr.compute(transpose);
  if (qr.info() != Eigen::Success || qr.rank() >= meq) return false;

  const Eigen::VectorXd rhs =
      -(state.grad + state.jh.transpose() * mu);
  const Eigen::VectorXd candidate = qr.solve(rhs);
  if (qr.info() != Eigen::Success || candidate.size() != meq ||
      !candidate.allFinite()) {
    return false;
  }
  const double current_residual = inf_norm(-rhs + transpose * lambda);
  const double candidate_residual = inf_norm(-rhs + transpose * candidate);
  if (candidate_residual + arithmetic_roundoff(candidate_residual) >=
      current_residual) {
    return false;
  }
  lambda = candidate;
  return true;
}

Eigen::VectorXd initial_variable_scale(const std::vector<VariableMeta>& vars,
                                       const Eigen::VectorXd& x) {
  Eigen::VectorXd scale(x.size());
  for (int col = 0; col < x.size(); ++col) {
    const auto& var = vars[static_cast<std::size_t>(col)];
    const bool finite_lb = is_effectively_finite(var.lb);
    const bool finite_ub = is_effectively_finite(var.ub);
    const double width = finite_lb && finite_ub
        ? std::max(0.0, var.ub - var.lb)
        : 0.0;
    scale[col] = std::max({1.0, std::abs(x[col]), width,
                           finite_lb ? std::abs(var.lb) : 0.0,
                           finite_ub ? std::abs(var.ub) : 0.0});
  }
  return scale;
}

Eigen::VectorXd initialize_slacks_from_linearization_resolution(
    const NLPState& state, const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& x) {
  Eigen::VectorXd row_reach_squared = Eigen::VectorXd::Zero(state.h.size());
  const Eigen::VectorXd variable_scale = initial_variable_scale(vars, x);
  for (int col = 0; col < state.jh.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.jh, col); it; ++it) {
      const double scaled = it.value() * variable_scale[col];
      row_reach_squared[it.row()] += scaled * scaled;
    }
  }
  const double root_epsilon =
      std::sqrt(std::numeric_limits<double>::epsilon());
  const double representability_floor = strict_minimum_safe_positive();
  Eigen::VectorXd slack(state.h.size());
  for (int row = 0; row < state.h.size(); ++row) {
    const double row_reach = std::sqrt(row_reach_squared[row]);
    const double resolution = std::max(
        representability_floor,
        root_epsilon * std::max(std::abs(state.h[row]), row_reach));
    slack[row] = std::max(-state.h[row], resolution);
  }
  return slack;
}

Eigen::VectorXd stationarity_invisible_multiplier_floor(
    const NLPState& state) {
  const int inequalities = static_cast<int>(state.jh.rows());
  // The floor is a representation device, not a stationarity budget or a
  // complementarity target. sqrt(min_normal) is the smallest positive value
  // whose reciprocal remains representable; all physical multiplier content
  // is supplied row-wise by the nonnegative stationarity selector.
  Eigen::VectorXd row_scale = Eigen::VectorXd::Zero(inequalities);
  for (int col = 0; col < state.jh.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.jh, col); it;
         ++it) {
      row_scale[it.row()] =
          std::max(row_scale[it.row()], std::abs(it.value()));
    }
  }
  Eigen::VectorXd floor(inequalities);
  const double normalized_floor = strict_minimum_safe_positive();
  for (int row = 0; row < inequalities; ++row) {
    const double scale = row_scale[row] > 0.0 && std::isfinite(row_scale[row])
        ? row_scale[row]
        : 1.0;
    floor[row] = normalized_floor / scale;
    if (!(floor[row] > 0.0) || !std::isfinite(floor[row])) {
      floor[row] = std::numeric_limits<double>::min();
    }
  }
  return floor;
}

bool initialize_primal_feasible_floor_projection(
    const NLPState& state, const Eigen::VectorXd& slack,
    Eigen::VectorXd& lambda, Eigen::VectorXd& mu, double& barrier) {
  if (slack.size() != state.jh.rows() ||
      (slack.size() > 0 && (!(slack.array() > 0.0).all() ||
                            !slack.allFinite()))) {
    return false;
  }
  mu = stationarity_invisible_multiplier_floor(state);
  lambda = Eigen::VectorXd::Zero(state.jg.rows());
  if (state.jg.rows() > 0) {
    // Failure leaves the zero equality multiplier in place; the positive
    // inequality floor remains a valid, scale-covariant interior state.
    initialize_equality_duals_least_squares(state, mu, lambda);
  }
  barrier = slack.size() > 0
      ? slack.dot(mu) / static_cast<double>(slack.size())
      : 0.0;
  return mu.allFinite() && (mu.size() == 0 || (mu.array() > 0.0).all()) &&
         lambda.allFinite() && std::isfinite(barrier) && barrier >= 0.0;
}

bool fit_primal_feasible_duals_active_projection(
    const NLPState& state, const Eigen::VectorXd& slack,
    double primal_tolerance, double dual_tolerance,
    double complementarity_tolerance,
    int selector_iteration_budget, Eigen::VectorXd& lambda,
    Eigen::VectorXd& mu, double& barrier) {
  const int n = static_cast<int>(state.grad.size());
  const int meq = static_cast<int>(state.jg.rows());
  const int inequalities = static_cast<int>(state.jh.rows());
  const bool trace = std::getenv("HACDCPF_OPF_TRACE") != nullptr;
  if (n == 0 || inequalities == 0 || slack.size() != inequalities ||
      !(primal_tolerance > 0.0) || !(dual_tolerance > 0.0) ||
      !(complementarity_tolerance > 0.0) ||
      selector_iteration_budget <= 0) {
    return false;
  }

  const Eigen::VectorXd multiplier_floor =
      stationarity_invisible_multiplier_floor(state);
  const Eigen::VectorXd base_stationarity =
      state.grad + state.jh.transpose() * multiplier_floor;
  Eigen::VectorXd floor_lambda = Eigen::VectorXd::Zero(meq);
  initialize_equality_duals_least_squares(
      state, multiplier_floor, floor_lambda);
  const double floor_dual = inf_norm(
      base_stationarity + state.jg.transpose() * floor_lambda);
  const double floor_complementarity =
      inf_norm(slack.cwiseProduct(multiplier_floor));
  if (floor_dual <= dual_tolerance + comparison_roundoff(floor_dual) &&
      floor_complementarity <=
          complementarity_tolerance +
              comparison_roundoff(floor_complementarity)) {
    // The floor projection already meets the caller's dual gate. Solving an
    // auxiliary multiplier QP cannot improve the certificate and is
    // especially wasteful for a many-row near-active Phase-I handoff.
    return false;
  }

  std::vector<int> active;
  active.reserve(static_cast<std::size_t>(inequalities));
  for (int row = 0; row < inequalities; ++row) {
    active.push_back(row);
  }
  if (trace) {
    std::cerr << "[NativeIPM] active dual projection: candidates="
              << active.size() << "/" << inequalities
              << ", noeq_dual=" << inf_norm(base_stationarity)
              << ", base_dual=" << floor_dual
              << ", lambda_inf=" << inf_norm(floor_lambda) << '\n';
  }
  if (active.empty()) return false;

  Eigen::VectorXd equality_scale =
      Eigen::VectorXd::Constant(meq, minimum_safe_positive());
  for (int col = 0; col < state.jg.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.jg, col); it;
         ++it) {
      equality_scale[it.row()] =
          std::max(equality_scale[it.row()], std::abs(it.value()));
    }
  }
  Eigen::VectorXd inequality_scale =
      Eigen::VectorXd::Constant(inequalities, minimum_safe_positive());
  for (int col = 0; col < state.jh.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.jh, col); it;
         ++it) {
      inequality_scale[it.row()] =
          std::max(inequality_scale[it.row()], std::abs(it.value()));
    }
  }
  const Eigen::SparseMatrix<double> identity =
      diagonal_sparse(Eigen::VectorXd::Ones(n));
  const double regularization =
      std::sqrt(std::numeric_limits<double>::epsilon());
  for (std::size_t projection = 0;; ++projection) {
    std::vector<int> active_position(
        static_cast<std::size_t>(inequalities), -1);
    for (int position = 0; position < static_cast<int>(active.size());
         ++position) {
      active_position[static_cast<std::size_t>(active[position])] = position;
    }

    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(static_cast<std::size_t>(state.jg.nonZeros() +
                                           state.jh.nonZeros()));
    for (int col = 0; col < state.jg.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(state.jg, col); it;
           ++it) {
        trips.emplace_back(it.row(), it.col(),
                           it.value() / equality_scale[it.row()]);
      }
    }
    for (int col = 0; col < state.jh.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(state.jh, col); it;
           ++it) {
        const int position =
            active_position[static_cast<std::size_t>(it.row())];
        if (position >= 0) {
          trips.emplace_back(meq + position, it.col(),
                             it.value() / inequality_scale[it.row()]);
        }
      }
    }
    Eigen::SparseMatrix<double> active_jacobian(
        meq + static_cast<int>(active.size()), n);
    active_jacobian.setFromTriplets(trips.begin(), trips.end());
    active_jacobian.makeCompressed();

    SparseKKTCache cache;
    if (!factor_kkt_sparse(cache, identity, active_jacobian,
                           regularization)) {
      if (trace) {
        std::cerr << "[NativeIPM] active dual projection rejected: factor, "
                  << "rows=" << active_jacobian.rows() << '\n';
      }
      return false;
    }
    Eigen::VectorXd rhs(n + active_jacobian.rows());
    rhs.head(n) = -base_stationarity;
    rhs.tail(active_jacobian.rows()).setZero();
    Eigen::VectorXd stationarity_remainder;
    Eigen::VectorXd projected_duals;
    if (!solve_kkt_sparse(cache, rhs, stationarity_remainder,
                          projected_duals) ||
        !projected_duals.allFinite()) {
      if (trace) {
        std::cerr << "[NativeIPM] active dual projection rejected: solve, "
                  << "rows=" << active_jacobian.rows() << '\n';
      }
      return false;
    }

    int negative_count = 0;
    for (int position = 0; position < static_cast<int>(active.size());
         ++position) {
      if (projected_duals[meq + position] < 0.0) ++negative_count;
    }
    if (trace && negative_count > 0) {
      std::cerr << "[NativeIPM] active dual projection: iteration="
                << projection << ", unconstrained_negative="
                << negative_count << ", retained_for_nonnegative_qp="
                << active.size() << '\n';
    }

    const double residual_scale = dual_tolerance;
    Eigen::VectorXd mu_projection = multiplier_floor;
    Eigen::VectorXd projected_inequality_energy(active.size());
    for (int position = 0; position < static_cast<int>(active.size());
         ++position) {
      projected_inequality_energy[position] =
          std::max(0.0, projected_duals[meq + position]);
      mu_projection[active[position]] +=
          std::max(0.0, projected_duals[meq + position]) /
          inequality_scale[active[position]];
    }
    Eigen::VectorXd active_projection_lambda = Eigen::VectorXd::Zero(meq);
    for (int row = 0; row < meq; ++row) {
      active_projection_lambda[row] +=
          projected_duals[row] / equality_scale[row];
    }
    const double selector_baseline = std::max(
        floor_dual / residual_scale,
        floor_complementarity / complementarity_tolerance);
    const double selector_comparison_error =
        comparison_roundoff(selector_baseline);
    const double active_projection_stationarity = inf_norm(
        state.grad + state.jg.transpose() * active_projection_lambda +
        state.jh.transpose() * mu_projection);
    const double active_projection_energy =
        inf_norm(slack.cwiseProduct(mu_projection));
    const double active_projection_kkt = std::max(
        active_projection_stationarity / residual_scale,
        active_projection_energy / complementarity_tolerance);
    if (trace) {
      std::cerr << "[NativeIPM] active dual projection audit: stationarity="
                << active_projection_stationarity << ", energy="
                << active_projection_energy << ", kkt="
                << active_projection_kkt << ", baseline="
                << selector_baseline << ", comparison_error="
                << selector_comparison_error << '\n';
    }
    if (active_projection_lambda.allFinite() && mu_projection.allFinite() &&
        (mu_projection.array() > 0.0).all() &&
        active_projection_kkt + selector_comparison_error <
            selector_baseline &&
        active_projection_stationarity <=
            dual_tolerance + comparison_roundoff(active_projection_stationarity) &&
        active_projection_energy <=
            complementarity_tolerance +
                comparison_roundoff(active_projection_energy)) {
      lambda = std::move(active_projection_lambda);
      mu = std::move(mu_projection);
      barrier = slack.dot(mu) / static_cast<double>(slack.size());
      if (trace) {
        std::cerr << "[NativeIPM] active dual projection accepted: active="
                  << active.size() << ", dual=" << floor_dual << " -> "
                  << active_projection_stationarity << ", kkt="
                  << active_projection_kkt << '\n';
      }
      return std::isfinite(barrier) && barrier > 0.0;
    }
    Eigen::VectorXd projected_lambda = floor_lambda;
    initialize_equality_duals_least_squares(
        state, mu_projection, projected_lambda);
    const Eigen::VectorXd floor_stationarity =
        state.grad + state.jg.transpose() * floor_lambda +
        state.jh.transpose() * multiplier_floor;
    const Eigen::VectorXd projected_stationarity =
        state.grad + state.jg.transpose() * projected_lambda +
        state.jh.transpose() * mu_projection;
    const Eigen::VectorXd stationarity_slope =
        projected_stationarity - floor_stationarity;

    // The regularized projection supplies a scale-covariant nonnegative
    // direction, but its full step minimizes a weighted two-norm rather than
    // the infinity-norm certificate used below. Minimize that certificate
    // exactly on the projection ray before constructing the full epigraph QP.
    struct RayModelValue {
      double value{-std::numeric_limits<double>::infinity()};
      double minimum_active_slope{std::numeric_limits<double>::infinity()};
      double maximum_active_slope{-std::numeric_limits<double>::infinity()};
    };
    const auto evaluate_ray_model = [&](double alpha) {
      RayModelValue model;
      const auto visit_line = [&](double offset, double slope) {
        const double value = offset + alpha * slope;
        if (value > model.value) {
          model.value = value;
          model.minimum_active_slope = slope;
          model.maximum_active_slope = slope;
        } else if (value == model.value) {
          model.minimum_active_slope =
              std::min(model.minimum_active_slope, slope);
          model.maximum_active_slope =
              std::max(model.maximum_active_slope, slope);
        }
      };
      for (int col = 0; col < n; ++col) {
        const double offset = floor_stationarity[col] / residual_scale;
        const double slope = stationarity_slope[col] / residual_scale;
        visit_line(offset, slope);
        visit_line(-offset, -slope);
      }
      for (int row = 0; row < inequalities; ++row) {
        visit_line(
            slack[row] * multiplier_floor[row] /
                complementarity_tolerance,
            slack[row] * (mu_projection[row] - multiplier_floor[row]) /
                complementarity_tolerance);
      }
      return model;
    };

    double lower = 0.0;
    double upper = 1.0;
    while (true) {
      const double midpoint = std::midpoint(lower, upper);
      if (midpoint == lower || midpoint == upper) break;
      const RayModelValue model = evaluate_ray_model(midpoint);
      if (model.maximum_active_slope < 0.0) {
        lower = midpoint;
      } else if (model.minimum_active_slope > 0.0) {
        upper = midpoint;
      } else {
        lower = midpoint;
        upper = midpoint;
        break;
      }
    }
    const RayModelValue lower_model = evaluate_ray_model(lower);
    const RayModelValue upper_model = evaluate_ray_model(upper);
    const double selected_alpha =
        upper_model.value <= lower_model.value ? upper : lower;
    const double selected_model =
        std::min(lower_model.value, upper_model.value);
    const double baseline_model = evaluate_ray_model(0.0).value;
    const double model_comparison_error =
        comparison_roundoff(baseline_model);
    const bool ray_model_improves = selected_alpha > 0.0 &&
        std::isfinite(selected_model) &&
        selected_model + model_comparison_error < baseline_model;
    if (ray_model_improves) {
      Eigen::VectorXd mu_candidate = multiplier_floor +
          selected_alpha * (mu_projection - multiplier_floor);
      Eigen::VectorXd lambda_candidate = floor_lambda +
          selected_alpha * (projected_lambda - floor_lambda);
      initialize_equality_duals_least_squares(
          state, mu_candidate, lambda_candidate);
      const double after = inf_norm(
          state.grad + state.jg.transpose() * lambda_candidate +
          state.jh.transpose() * mu_candidate);
      const double selected_complementarity =
          inf_norm(slack.cwiseProduct(mu_candidate));
      const double candidate_kkt_norm = std::max(
          after / residual_scale,
          selected_complementarity / complementarity_tolerance);
      if (lambda_candidate.allFinite() && mu_candidate.allFinite() &&
          (mu_candidate.array() > 0.0).all() &&
          candidate_kkt_norm + selector_comparison_error <
              selector_baseline &&
          after <= dual_tolerance + comparison_roundoff(after) &&
          selected_complementarity <=
              complementarity_tolerance +
                  comparison_roundoff(selected_complementarity)) {
        lambda = std::move(lambda_candidate);
        mu = std::move(mu_candidate);
        barrier = slack.dot(mu) / static_cast<double>(slack.size());
        if (trace) {
          std::cerr << "[NativeIPM] active dual ray accepted: active="
                    << active.size() << ", alpha=" << selected_alpha
                    << ", dual=" << floor_dual << " -> " << after
                    << ", kkt=" << candidate_kkt_norm << '\n';
        }
        return std::isfinite(barrier) && barrier > 0.0;
      }
    }
    if (trace) {
      std::cerr << "[NativeIPM] active dual ray deferred to epigraph: alpha="
                << selected_alpha << ", baseline=" << baseline_model
                << ", selected=" << selected_model << '\n';
    }

    const int selector_dual_dimension =
        meq + static_cast<int>(active.size());
    const int selector_epigraph = selector_dual_dimension;
    const int selector_dimension = selector_dual_dimension + 1;
    const int selector_rows =
        2 * n + static_cast<int>(active.size());
    LPModel selector;
    selector.sense = Sense::Minimize;
    selector.c = Eigen::VectorXd::Zero(selector_dimension);
    selector.c[selector_epigraph] = 1.0;
    selector.b = Eigen::VectorXd::Zero(selector_rows);
    std::vector<Eigen::Triplet<double>> selector_trips;
    selector_trips.reserve(static_cast<std::size_t>(
        2 * active_jacobian.nonZeros() + 2 * n + 2 * active.size()));
    for (int col = 0; col < active_jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(active_jacobian, col);
           it; ++it) {
        selector_trips.emplace_back(
            it.col(), it.row(), it.value() / residual_scale);
        selector_trips.emplace_back(
            n + it.col(), it.row(), -it.value() / residual_scale);
      }
    }
    for (int col = 0; col < n; ++col) {
      selector_trips.emplace_back(col, selector_epigraph, -1.0);
      selector_trips.emplace_back(n + col, selector_epigraph, -1.0);
      selector.b[col] = -floor_stationarity[col] / residual_scale;
      selector.b[n + col] = floor_stationarity[col] / residual_scale;
    }
    for (int position = 0; position < static_cast<int>(active.size());
         ++position) {
      const int row = 2 * n + position;
      selector_trips.emplace_back(
          row, meq + position,
          slack[active[position]] /
              inequality_scale[active[position]] /
              complementarity_tolerance);
      selector_trips.emplace_back(row, selector_epigraph, -1.0);
      selector.b[row] =
          -slack[active[position]] *
          multiplier_floor[active[position]] /
          complementarity_tolerance;
    }
    selector.A.resize(selector_rows, selector_dimension);
    selector.A.setFromTriplets(selector_trips.begin(), selector_trips.end());
    selector.A.makeCompressed();
    selector.Aeq.resize(0, selector_dimension);
    selector.beq.resize(0);
    selector.vars.reserve(static_cast<std::size_t>(selector_dimension));
    for (int row = 0; row < meq; ++row) {
      selector.vars.push_back(
          {VarType::Continuous, -kVariableNoBound, kVariableNoBound});
    }
    for (int position = 0; position < static_cast<int>(active.size());
         ++position) {
      selector.vars.push_back(
          {VarType::Continuous, 0.0, kVariableNoBound});
    }
    selector.vars.push_back(
        {VarType::Continuous, 0.0, kVariableNoBound});

    const auto try_selector_candidate =
        [&](const SolveResult& candidate, bool require_absolute_gate,
            const char* source) {
      if (candidate.x.size() != selector_dimension ||
          !candidate.x.allFinite()) {
        return false;
      }
      Eigen::VectorXd selector_mu = multiplier_floor;
      for (int position = 0; position < static_cast<int>(active.size());
           ++position) {
        selector_mu[active[position]] +=
            std::max(0.0, candidate.x[meq + position]) /
            inequality_scale[active[position]];
      }
      Eigen::VectorXd selector_lambda = floor_lambda;
      for (int row = 0; row < meq; ++row) {
        selector_lambda[row] += candidate.x[row] / equality_scale[row];
      }
      initialize_equality_duals_least_squares(
          state, selector_mu, selector_lambda);
      const double selector_stationarity = inf_norm(
          state.grad + state.jg.transpose() * selector_lambda +
          state.jh.transpose() * selector_mu);
      const double selector_energy =
          inf_norm(slack.cwiseProduct(selector_mu));
      const double selector_kkt = std::max(
          selector_stationarity / residual_scale,
          selector_energy / complementarity_tolerance);
      const bool absolute_gate =
          selector_stationarity <=
              dual_tolerance + comparison_roundoff(selector_stationarity) &&
          selector_energy <=
              complementarity_tolerance +
                  comparison_roundoff(selector_energy);
      if (!(selector_lambda.allFinite() && selector_mu.allFinite() &&
          (selector_mu.array() > 0.0).all() &&
          selector_kkt + selector_comparison_error < selector_baseline &&
          (!require_absolute_gate || absolute_gate))) {
        if (trace) {
          std::cerr << "[NativeIPM] active dual selector candidate rejected: "
                    << "source=" << source << ", status=\""
                    << candidate.stats.status << "\", iterations="
                    << candidate.stats.iterations
                    << ", stationarity=" << selector_stationarity
                    << ", energy=" << selector_energy
                    << ", kkt=" << selector_kkt
                    << ", baseline=" << selector_baseline
                    << ", absolute_gate=" << absolute_gate << '\n';
        }
        return false;
      }
      // A relative-only LCQP point is an initialization candidate, not an
      // optimality certificate. The main NLP convergence gate remains
      // authoritative after either candidate is installed.
      lambda = std::move(selector_lambda);
      mu = std::move(selector_mu);
      barrier = slack.dot(mu) / static_cast<double>(slack.size());
      if (trace) {
        std::cerr << "[NativeIPM] active dual selector candidate selected: "
                  << "source=" << source << ", status=\""
                  << candidate.stats.status << "\", iterations="
                  << candidate.stats.iterations << ", active="
                  << active.size() << ", dual=" << floor_dual << " -> "
                  << selector_stationarity << ", kkt=" << selector_kkt
                  << ", absolute_gate=" << absolute_gate << '\n';
      }
      return std::isfinite(barrier) && barrier > 0.0;
    };

    const SolveResult exact_selector = HighsAdapter{}.solve_lp(selector);
    if (try_selector_candidate(exact_selector, true, "highs_lp")) {
      return true;
    }

    QPModel fallback_selector;
    fallback_selector.sense = selector.sense;
    fallback_selector.Q.resize(selector_dimension, selector_dimension);
    fallback_selector.c = selector.c;
    fallback_selector.A = selector.A;
    fallback_selector.b = selector.b;
    fallback_selector.Aeq = selector.Aeq;
    fallback_selector.beq = selector.beq;
    fallback_selector.vars = selector.vars;
    Eigen::VectorXd selector_initial =
        Eigen::VectorXd::Zero(selector_dimension);
    selector_initial[selector_epigraph] = std::nextafter(
        selector_baseline, std::numeric_limits<double>::infinity());
    LCQPOptions fallback_options;
    fallback_options.max_iter = selector_iteration_budget;
    fallback_options.tol_primal = comparison_roundoff(1.0);
    fallback_options.tol_dual = comparison_roundoff(1.0);
    fallback_options.tol_gap = comparison_roundoff(1.0);
    fallback_options.centering_exponent = 0.0;
    fallback_options.initial_point = selector_initial;
    fallback_options.return_feasible_descent_candidate = false;
    fallback_options.candidate_objective_upper_bound =
        selector_baseline - selector_comparison_error;
    fallback_options.verbose = trace;
    const SolveResult fallback_result =
        NativeLCQPAdapter(fallback_options).solve_qp(fallback_selector);
    if (try_selector_candidate(fallback_result, false, "native_lcqp")) {
      return true;
    }
    if (trace) {
      std::cerr << "[NativeIPM] active dual selector fallback: highs=\""
                << exact_selector.stats.status << "\", lcqp=\""
                << fallback_result.stats.status << "\"\n";
    }

    return false;
  }
  return false;
}

bool project_stationarity_through_equalities(
    const NLPState& state, const Eigen::VectorXd& value,
    SparseInertiaKKTCache& cache, bool& cache_ready,
    Eigen::VectorXd& nullspace_component, Eigen::VectorXd& multiplier) {
  const int meq = static_cast<int>(state.jg.rows());
  if (meq == 0) {
    nullspace_component = value;
    multiplier.resize(0);
    return true;
  }
  if (!cache_ready) {
    const Eigen::SparseMatrix<double> metric =
        diagonal_sparse(Eigen::VectorXd::Ones(state.grad.size()));
    InertiaSettings settings;
    settings.max_tangent_dimension = -1;
    settings.mu = 1.0;
    InertiaStatus inertia;
    double delta_w_last = 0.0;
    if (!factor_kkt_inertia_corrected_sparse(
            metric, state.jg, settings, delta_w_last, cache, inertia)) {
      return false;
    }
    cache_ready = true;
  }

  Eigen::VectorXd rhs(value.size() + meq);
  rhs.head(value.size()) = -value;
  rhs.tail(meq).setZero();
  Eigen::VectorXd negative_component;
  if (!solve_kkt_inertia_corrected_sparse(
          cache, rhs, negative_component, multiplier) ||
      !negative_component.allFinite() || !multiplier.allFinite()) {
    return false;
  }
  nullspace_component = -negative_component;
  return true;
}

bool initialize_cold_primal_dual_state(
    const NLPState& state, const Eigen::VectorXd& slack,
    double requested_barrier, Eigen::VectorXd& lambda,
    Eigen::VectorXd& mu, double& barrier) {
  if (slack.size() != state.h.size() ||
      (slack.size() > 0 && (!(slack.array() > 0.0).all() ||
                            !slack.allFinite()))) {
    return false;
  }
  if (slack.size() == 0) {
    mu.resize(0);
    barrier = 0.0;
    return true;
  }

  const Eigen::VectorXd multiplier_floor =
      stationarity_invisible_multiplier_floor(state);
  const Eigen::VectorXd inverse_slack = slack.cwiseInverse();
  const Eigen::VectorXd base_stationarity =
      state.grad + state.jh.transpose() * multiplier_floor;
  const Eigen::VectorXd central_direction =
      state.jh.transpose() * inverse_slack;

  SparseInertiaKKTCache projection_cache;
  bool projection_ready = false;
  Eigen::VectorXd projected_base;
  Eigen::VectorXd projected_center;
  Eigen::VectorXd lambda_base;
  Eigen::VectorXd lambda_center;
  if (!project_stationarity_through_equalities(
          state, base_stationarity, projection_cache, projection_ready,
          projected_base, lambda_base) ||
      !project_stationarity_through_equalities(
          state, central_direction, projection_cache, projection_ready,
          projected_center, lambda_center)) {
    return false;
  }

  double central_barrier = requested_barrier;
  if (!(central_barrier > 0.0) || !std::isfinite(central_barrier)) {
    const double center_norm_squared = projected_center.squaredNorm();
    const double projection_error =
        std::sqrt(std::numeric_limits<double>::epsilon()) *
        std::max(1.0, central_direction.norm());
    const bool center_is_resolved =
        center_norm_squared > projection_error * projection_error;
    if (center_is_resolved) {
      central_barrier = std::max(
          0.0, -projected_center.dot(projected_base) / center_norm_squared);
    } else {
      central_barrier = 0.0;
    }
    if (!(central_barrier > 0.0) || !std::isfinite(central_barrier)) {
      // A positive central ray may be unable to reduce stationarity because
      // P*q and P*c point in the same direction. Stationarity then does not
      // identify a positive minimizer. Equilibrate the two resolved projected
      // blocks by norm; if P*c itself is unresolved, fall back to the mean
      // complementarity of the stationarity-invisible multiplier floor.
      central_barrier = center_is_resolved && projected_base.norm() > projection_error
          ? projected_base.norm() / std::sqrt(center_norm_squared)
          : slack.dot(multiplier_floor) / static_cast<double>(slack.size());
    }
  }

  mu = multiplier_floor + central_barrier * inverse_slack;
  lambda = lambda_base + central_barrier * lambda_center;
  if (!mu.allFinite() || !(mu.array() > 0.0).all() || !lambda.allFinite()) {
    return false;
  }
  barrier = slack.dot(mu) / static_cast<double>(slack.size());
  return std::isfinite(barrier) && barrier > 0.0;
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
  const double diagonal_scale = state.diag.size() == 0
      ? 1.0
      : std::max(1.0, state.diag.cwiseAbs().maxCoeff());
  const double positive_resolution = comparison_roundoff(diagonal_scale);
  for (int i = 0; i < state.diag.size(); ++i) {
    triplets.emplace_back(i, i,
                          std::max(positive_resolution, state.diag[i]));
  }
  for (const auto& block : state.blocks) {
    for (int r = 0; r < static_cast<int>(block.index.size()); ++r) {
      for (int c = 0; c < static_cast<int>(block.index.size()); ++c) {
        const double value = block.values(r, c);
        if (value != 0.0) {
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
    const double root_epsilon =
        std::sqrt(std::numeric_limits<double>::epsilon());
    const double secant_error = root_epsilon * s.norm() * y.norm();
    const double coordinate_scale = std::max(
        {1.0, inf_norm(x), inf_norm(state.prev_x)});
    const double step_resolution = root_epsilon * coordinate_scale;
    for (int i = 0; i < x.size(); ++i) {
      const double si = s[i];
      const double yi = y[i];
      if (std::abs(si) > step_resolution && yi * si > secant_error) {
        state.diag[i] = yi / si;
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
      const double skip_threshold = root_epsilon *
          local_residual.norm() * local_step.norm();
      // Limited-memory SR1 skip condition. Negative denominators are retained
      // so genuine nonconvex Lagrangian curvature reaches inertia correction.
      if (std::abs(denom) > skip_threshold) {
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
                                       double alpha_aff_dual,
                                       double centering_exponent) {
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
    const double sigma_i = std::clamp(
        std::pow(ratio, centering_exponent), 0.0, 1.0);
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

std::string diagnostic_variable_name(const NLPModel& prob, int col) {
  if (col < 0 || col >= static_cast<int>(prob.vars.size())) return "var?";
  const std::string& name = prob.vars[static_cast<std::size_t>(col)].name;
  return name.empty() ? "x[" + std::to_string(col) + "]" : name;
}

std::string diagnostic_inequality_name(
    const NLPModel& prob, int row, int inequality_count,
    int nonlinear_count, const std::vector<int>& lower_columns,
    const std::vector<int>& upper_columns) {
  if (row < 0 || row >= inequality_count) return "row?";
  if (row < nonlinear_count) {
    if (row < static_cast<int>(prob.nonlinear_inequality_names.size()) &&
        !prob.nonlinear_inequality_names[
             static_cast<std::size_t>(row)].empty()) {
      return prob.nonlinear_inequality_names[static_cast<std::size_t>(row)];
    }
    return "nonlinear[" + std::to_string(row) + "]";
  }
  const int bound_row = row - nonlinear_count;
  if (bound_row < static_cast<int>(lower_columns.size())) {
    return "lower:" + diagnostic_variable_name(
        prob, lower_columns[static_cast<std::size_t>(bound_row)]);
  }
  const int upper_row =
      bound_row - static_cast<int>(lower_columns.size());
  if (upper_row >= 0 &&
      upper_row < static_cast<int>(upper_columns.size())) {
    return "upper:" + diagnostic_variable_name(
        prob, upper_columns[static_cast<std::size_t>(upper_row)]);
  }
  return "bound?";
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
  const double root_epsilon =
      std::sqrt(std::numeric_limits<double>::epsilon());
  const double scale = std::max({1.0, sparse_abs_max(w), sparse_abs_max(jg)});
  if (factor_kkt_sparse(cache, w, jg, 0.0)) return true;
  const double maximum = scale / root_epsilon;
  for (double reg = root_epsilon * scale; reg <= maximum; reg *= 2.0) {
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

// Solve the assembled system through the common backward-error-audited KKT
// path, then split its combined dual block into equality and inequality parts.
bool solve_augmented_newton(AugmentedNewtonCache& c,
                            const Eigen::VectorXd& rhs,
                            Eigen::VectorXd& dx,
                            Eigen::VectorXd& dlambda,
                            Eigen::VectorXd& dmu) {
  Eigen::VectorXd combined_dual;
  if (!solve_kkt_sparse(c.kkt, rhs, dx, combined_dual) ||
      combined_dual.size() != c.meq + c.miq) {
    return false;
  }
  dlambda = combined_dual.head(c.meq);
  dmu = combined_dual.tail(c.miq);
  return dx.size() == c.n;
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

// The unregularized exact-feasible condensed factor is a valid Newton
// certificate only when its inertia is (n, meq, 0). When a symmetric-
// indefinite backend reports a different, *available* inertia (negative
// curvature on null(Jg) or a rank deficiency) the resulting direction is not a
// descent step and drives the filter into a grossly infeasible trial, so the
// solve must defer to inertia correction. An unavailable inertia (backend
// returns -1) preserves the legacy fast path unchanged.
bool exact_condensed_factor_inertia_ok(const SparseKKTCache& cache, int meq) {
  if (!cache.solver) return false;
  const int negative = cache.solver->negative_eigenvalues();
  const int deficiency = cache.solver->estimated_deficiency();
  if (negative >= 0 && negative != meq) return false;
  if (deficiency > 0) return false;
  return true;
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
    bool require_descent_inertia,
    const Eigen::VectorXd& rhs,
    Eigen::VectorXd& dx,
    Eigen::VectorXd& dlambda,
    Eigen::VectorXd& dmu) {
  ensure_augmented_inertia_backend(c);
  if (!require_descent_inertia) {
    const bool pattern_unchanged = assemble_augmented_newton(
        c, h, jg, jh, s, mu_ineq, 0.0, 0.0);
    if (factor_current_kkt(c.kkt, c.n, c.meq + c.miq,
                           pattern_unchanged) &&
        solve_augmented_newton(c, rhs, dx, dlambda, dmu)) {
      delta_w_last = 0.0;
      return true;
    }
  }
  const double root_epsilon =
      std::sqrt(std::numeric_limits<double>::epsilon());
  const double local_scale = std::max(
      {1.0, sparse_abs_max(h), sparse_abs_max(jg), sparse_abs_max(jh)});
  const double initial_delta =
      settings.delta_w_0 > 0.0 && std::isfinite(settings.delta_w_0)
          ? settings.delta_w_0
          : root_epsilon * local_scale;
  const double maximum_delta =
      settings.delta_w_max > 0.0 && std::isfinite(settings.delta_w_max)
          ? settings.delta_w_max
          : local_scale / root_epsilon;
  double delta_w = std::max(0.0, delta_w_last);
  bool delta_w_was_zero = (delta_w == 0.0);
  const double delta_c =
      settings.delta_c_stripe > 0.0 && std::isfinite(settings.delta_c_stripe)
          ? settings.delta_c_stripe
          : root_epsilon * local_scale;
  while (delta_w <= maximum_delta) {
    auto factor_and_solve = [&](double dual_stripe) {
      const bool pattern_unchanged = assemble_augmented_newton(
          c, h, jg, jh, s, mu_ineq, delta_w, dual_stripe);
      return factor_current_kkt(c.kkt, c.n, c.meq + c.miq,
                                pattern_unchanged) &&
          augmented_factor_has_correct_inertia(c) &&
          solve_augmented_newton(c, rhs, dx, dlambda, dmu);
    };
    // Preserve the exact equality Newton equation whenever the zero dual
    // block is nonsingular. The backward-error-scale stripe is rank repair,
    // not a default perturbation: use it only after the exact block fails.
    if (factor_and_solve(0.0) || factor_and_solve(delta_c)) {
      delta_w_last = std::max(settings.delta_w_min, delta_w);
      return true;
    }
    // Wächter–Biegler δ_W schedule.
    if (delta_w_was_zero) {
      delta_w = initial_delta;
      delta_w_was_zero = false;
    } else {
      delta_w *= 2.0;
    }
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
      const double lower_interior = std::nextafter(lb, ub);
      const double upper_interior = std::nextafter(ub, lb);
      if (!(lower_interior < upper_interior)) {
        x[i] = std::midpoint(lb, ub);
        continue;
      }
      x[i] = std::clamp(x[i], lower_interior, upper_interior);
    } else if (has_lb) {
      x[i] = std::max(
          x[i], std::nextafter(lb, std::numeric_limits<double>::infinity()));
    } else if (has_ub) {
      x[i] = std::min(
          x[i], std::nextafter(ub, -std::numeric_limits<double>::infinity()));
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

bool evaluate_primal_feasible_trial_point(
    const NLPModel& prob, const std::vector<int>& lb_cols,
    const std::vector<int>& ub_cols, const Eigen::VectorXd& x_trial,
    const Eigen::VectorXd& lambda, const Eigen::VectorXd& mu,
    const Eigen::VectorXd& dlambda, const Eigen::VectorXd& dmu,
    const Eigen::VectorXd& newton_slack,
    double primal_step, std::string& status, TrialPoint& trial) {
  NLPState trial_state;
  if (!evaluate_nlp_state(prob, x_trial, lb_cols, ub_cols, trial_state,
                          status)) {
    return false;
  }
  Eigen::VectorXd trial_slack =
      initialize_slacks_from_linearization_resolution(
          trial_state, prob.vars, x_trial);
  // The linearization-resolution slack pins s at -h(x), which floors s*z at
  // ~sqrt(eps)*z; a stiff active constraint then cannot reach a tight
  // complementarity gate. Let the strictly positive Newton slack shrink a row
  // below that pinned value while keeping the pinned value as the feasibility
  // upper bound; h+s then becomes a small primal residual the caller audits
  // against tol_primal.
  if (newton_slack.size() == trial_slack.size()) {
    for (int i = 0; i < trial_slack.size(); ++i) {
      if (newton_slack[i] > 0.0 && newton_slack[i] < trial_slack[i]) {
        trial_slack[i] = newton_slack[i];
      }
    }
  }
  if (trial_state.g.size() != lambda.size() ||
      trial_state.h.size() != mu.size() ||
      dlambda.size() != lambda.size() || dmu.size() != mu.size() ||
      trial_slack.size() != trial_state.h.size() ||
      !lambda.allFinite() || !mu.allFinite() ||
      !dlambda.allFinite() || !dmu.allFinite() ||
      !trial_slack.allFinite() ||
      (trial_slack.size() > 0 && !(trial_slack.array() > 0.0).all()) ||
      (mu.size() > 0 && !(mu.array() > 0.0).all())) {
    return false;
  }

  const Eigen::VectorXd lambda_trial = lambda + primal_step * dlambda;
  const Eigen::VectorXd multiplier_floor =
      stationarity_invisible_multiplier_floor(trial_state);
  // This is the exact Euclidean projection of the affine Newton endpoint onto
  // the row-wise strict dual cone.  Each row is projected independently; no
  // near-zero multiplier can impose a shared step length on unrelated rows.
  const Eigen::VectorXd mu_trial =
      (mu + primal_step * dmu).cwiseMax(multiplier_floor);
  if (!lambda_trial.allFinite() || !mu_trial.allFinite() ||
      (mu_trial.size() > 0 && !(mu_trial.array() > 0.0).all())) {
    return false;
  }
  const Eigen::VectorXd r_dual =
      trial_state.grad + trial_state.jg.transpose() * lambda_trial +
      trial_state.jh.transpose() * mu_trial;
  trial.valid = true;
  trial.x = x_trial;
  trial.s = std::move(trial_slack);
  trial.lambda = lambda_trial;
  trial.mu = mu_trial;
  trial.state = std::move(trial_state);
  trial.residuals = summarize_residuals(
      r_dual, trial.state.g, trial.state.h + trial.s, trial.x, trial.s,
      trial.lambda, trial.mu);
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
                                     const ResidualSummary& trial) {
  const bool merit_ok = trial.merit + comparison_roundoff(trial.merit) <
      current.merit;
  const bool primal_ok = trial.primal_feas +
      comparison_roundoff(trial.primal_feas) < current.primal_feas;
  const bool dual_ok = trial.dual_feas +
      comparison_roundoff(trial.dual_feas) < current.dual_feas;
  return merit_ok ||
         (primal_ok && trial.dual_feas <=
               current.dual_feas + comparison_roundoff(current.dual_feas)) ||
         (dual_ok && trial.primal_feas <=
              current.primal_feas + comparison_roundoff(current.primal_feas));
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
  std::string linear_solver_backend{"unselected"};
  int symbolic_analyses{0};
  int numeric_factorizations{0};
  int linear_solves{0};
  int primary_factorizations{0};
  int inertia_retry_factorizations{0};
  int inertia_certificate_factorizations{0};
  int active_set_polish_factorizations{0};
  int accepted_steps{0};
  int rejected_steps{0};
  int trial_value_evaluations{0};
  int trial_full_derivative_evaluations{0};
  int trial_rejections_before_derivatives{0};
};

FilterSolveOutcome solve_nlp_filter_impl(const NLPModel& prob,
                                         const IPMOptions& opt) {
  FilterSolveOutcome result;
  const int n = static_cast<int>(prob.vars.size());
  Eigen::VectorXd x = (prob.x0.size() == n) ? prob.x0 : Eigen::VectorXd::Zero(n);
  // A caller-audited Phase-I point is already tolerance-feasible in the
  // original coordinates. Preserve that exact point: generated bound rows and
  // positive slacks support an infeasible-start primal-dual state, while an
  // implicit projection here can destroy nonlinear equality feasibility.
  if (!opt.primal_feasible_start) interiorize_initial_point(prob.vars, x);

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

  Eigen::VectorXd s = initialize_slacks_from_linearization_resolution(
      state, prob.vars, x);

  const double effective_mu_min =
      opt.mu_min > 0.0 && std::isfinite(opt.mu_min)
          ? opt.mu_min
          : minimum_safe_positive();
  double mu_bar = 0.0;

  // Primal–dual multiplier initialization.
  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(state.g.size());
  const bool has_equality_dual_start =
      opt.equality_dual_start.size() == lambda.size() &&
      opt.equality_dual_start.allFinite();
  const bool has_inequality_dual_start =
      state.h.size() > 0 &&
      opt.inequality_dual_start.size() == state.h.size() &&
      opt.inequality_dual_start.allFinite() &&
      (opt.inequality_dual_start.array() > 0.0).all();
  const bool has_slack_start =
      opt.slack_start.size() == s.size() && opt.slack_start.allFinite() &&
      (opt.slack_start.array() > 0.0).all();
  if (has_slack_start) {
    s = opt.slack_start;
  }
  Eigen::VectorXd mu_ineq;
  bool cold_lambda_initialized = false;
  if (has_inequality_dual_start) {
    mu_ineq = opt.inequality_dual_start;
    mu_bar = s.size() > 0
        ? s.dot(mu_ineq) / static_cast<double>(s.size())
        : 0.0;
  } else if (s.size() > 0) {
    const bool active_fit_initialized = opt.primal_feasible_start &&
        fit_primal_feasible_duals_active_projection(
            state, s, opt.tol_primal, opt.tol_dual,
            opt.tol_complementarity, opt.max_iter, lambda, mu_ineq, mu_bar);
    const bool floor_projection_initialized =
        opt.primal_feasible_start && !active_fit_initialized &&
        initialize_primal_feasible_floor_projection(
            state, s, lambda, mu_ineq, mu_bar);
    cold_lambda_initialized =
        active_fit_initialized || floor_projection_initialized;
    if (!opt.primal_feasible_start) {
      cold_lambda_initialized = initialize_cold_primal_dual_state(
          state, s, opt.mu_init, lambda, mu_ineq, mu_bar);
    }
    if (!cold_lambda_initialized) {
      result.status = "Filter: scale-covariant dual initialization failed";
      result.x = x;
      return result;
    }
    if (opt.verbose && opt.primal_feasible_start) {
      std::cerr << "[NativeIPM] Phase-I dual initialization: mode="
                << (active_fit_initialized ? "active_projection"
                                           : "floor_projection")
                << ", mean_complementarity=" << mu_bar << '\n';
    }
  } else {
    mu_ineq.resize(0);
    mu_bar = std::max(0.0, opt.mu_init);
  }
  const bool has_complete_inequality_warm_start =
      has_inequality_dual_start && has_slack_start;
  const bool automatic_phase1_barrier = opt.primal_feasible_start &&
      !has_complete_inequality_warm_start &&
      !(opt.mu_init > 0.0 && std::isfinite(opt.mu_init));
  if (automatic_phase1_barrier && s.size() > 0) {
    // A Phase-I handoff is not a central-path point. Using mean(s_i*mu_i) as
    // its first scalar target would demand an increase on every product below
    // the mean and recreate a tau/s_i multiplier spike at small slacks. The
    // minimum current product is the largest scalar target satisfying the
    // row-wise invariant tau <= s_i*mu_i for every inequality.
    mu_bar = s.cwiseProduct(mu_ineq).minCoeff();
  }
  mu_bar = std::max(effective_mu_min, mu_bar);
  if (has_equality_dual_start) {
    lambda = opt.equality_dual_start;
  } else if (prob.hess || prob.lagrangian_hess) {
    if (opt.least_square_init_duals) {
      initialize_equality_duals_least_squares(state, mu_ineq, lambda);
    } else if (!opt.primal_feasible_start && s.size() == 0) {
      initialize_rank_deficient_equality_duals(state, mu_ineq, lambda);
    }
  }

  if (opt.primal_feasible_start && has_inequality_dual_start &&
      !has_complete_inequality_warm_start && s.size() > 0) {
    Eigen::VectorXd selected_lambda;
    Eigen::VectorXd selected_mu;
    double selected_barrier = 0.0;
    if (fit_primal_feasible_duals_active_projection(
            state, s, opt.tol_primal, opt.tol_dual,
            opt.tol_complementarity, opt.max_iter, selected_lambda,
            selected_mu, selected_barrier)) {
      const double warm_dual = inf_norm(
          state.grad + state.jg.transpose() * lambda +
          state.jh.transpose() * mu_ineq);
      const double warm_complementarity =
          inf_norm(s.cwiseProduct(mu_ineq));
      const double selected_dual = inf_norm(
          state.grad + state.jg.transpose() * selected_lambda +
          state.jh.transpose() * selected_mu);
      const double selected_complementarity =
          inf_norm(s.cwiseProduct(selected_mu));
      const double warm_kkt = std::max(
          warm_dual / opt.tol_dual,
          warm_complementarity / opt.tol_complementarity);
      const double selected_kkt = std::max(
          selected_dual / opt.tol_dual,
          selected_complementarity / opt.tol_complementarity);
      if (selected_kkt + comparison_roundoff(selected_kkt) < warm_kkt) {
        lambda = std::move(selected_lambda);
        mu_ineq = std::move(selected_mu);
        mu_bar = selected_barrier;
        if (automatic_phase1_barrier) {
          mu_bar = s.cwiseProduct(mu_ineq).minCoeff();
        }
        mu_bar = std::max(effective_mu_min, mu_bar);
        if (opt.verbose) {
          std::cerr << "[NativeIPM] Phase-I warm dual selector accepted: "
                    << "kkt=" << warm_kkt << " -> " << selected_kkt
                    << '\n';
        }
      }
    }
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
    std::cerr << "[NativeIPM] initial complementarity: mu_bar=" << mu_bar;
    print_distribution("nonlinear", nonlinear);
    print_distribution("lower", lower);
    print_distribution("upper", upper);
    std::cerr << '\n' << std::flush;
  }

  Filter filter;
  const double filter_roundoff = std::numeric_limits<double>::epsilon();
  const double filter_gamma_theta =
      opt.filter_gamma_theta > 0.0 && opt.filter_gamma_theta < 1.0
          ? opt.filter_gamma_theta
          : filter_roundoff;
  const double filter_gamma_phi =
      opt.filter_gamma_phi > 0.0 && opt.filter_gamma_phi < 1.0
          ? opt.filter_gamma_phi
          : filter_roundoff;
  const bool parameterized_switching_filter =
      opt.filter_s_theta > 0.0 && opt.filter_s_phi > 0.0 &&
      opt.filter_delta > 0.0 && opt.filter_eta_phi > 0.0 &&
      opt.filter_theta_min_scale > 0.0;
  // The public primal gate is ||[g; h+s]||_inf <= tol_primal, whereas the
  // filter uses theta = ||[g; h+s]||_1. For m residual rows, every point in
  // the public feasible set therefore satisfies theta <= m*tol_primal. Keep
  // the initial point admissible and advance the strict wall by one floating-
  // point value; no case scale or empirical violation multiplier enters it.
  const double theta_initial = compute_theta(state.g, state.h, s);
  const double phase1_primal_upper_bound = std::nextafter(
      std::max({inf_norm(state.g), inf_norm(state.h + s), opt.tol_primal}),
      std::numeric_limits<double>::infinity());
  const auto constraint_row_count = state.g.size() + state.h.size();
  const double componentwise_primal_contract = std::max(
      {inf_norm(state.g), inf_norm(state.h + s), opt.tol_primal});
  const double theta_contract =
      static_cast<double>(constraint_row_count) *
      componentwise_primal_contract;
  const double theta_upper_bound = std::nextafter(
      std::max(theta_initial, theta_contract),
      std::numeric_limits<double>::infinity());
  filter.reset_with_theta_upper_bound(theta_upper_bound);
  if (opt.verbose) {
    std::cerr << "[NativeIPM] filter contract: theta_initial="
              << theta_initial << ", rows=" << constraint_row_count
              << ", theta_contract=" << theta_contract
              << ", theta_upper_bound=" << theta_upper_bound << '\n';
  }

  // When no analytical Hessian is provided, use the evolving sparse
  // quasi-Newton approximation instead of freezing a finite-difference
  // Hessian at the initial point.
  DiagonalQNState filter_qn_state;
  if (!prob.hess && !prob.lagrangian_hess) {
    initialize_quasi_newton_state(n, filter_qn_state);
    filter_qn_state.sparse_block_size =
        opt.qn_sparse_block_size > 0 ? opt.qn_sparse_block_size : n;
    filter_qn_state.max_blocks =
        opt.qn_max_blocks > 0 ? opt.qn_max_blocks : 1;
  }

  double delta_w_last = 0.0;
  // Warm-started floor for the trust-region δ_W escalation (see the condensed
  // Newton factor below). Once an iteration needs a larger shift to bound the
  // Newton direction, subsequent iterations start from that shift instead of
  // re-escalating from the minimal-inertia value, then decay it back toward
  // zero so a transient ill-conditioning does not permanently over-regularize.
  double sticky_min_delta_w = 0.0;
  SparseInertiaKKTCache kkt_cache;
  kkt_cache.preferred_free_columns = prob.equality_free_columns;
  kkt_cache.augmented.refinement_tolerance =
      std::sqrt(std::numeric_limits<double>::epsilon());
  SparseKKTCache regularized_kkt_cache;
  regularized_kkt_cache.refinement_tolerance =
      std::sqrt(std::numeric_limits<double>::epsilon());
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
  int accepted_steps = 0;
  int rejected_steps = 0;
  int trial_value_evaluations = 0;
  int trial_full_derivative_evaluations = 0;
  int trial_rejections_before_derivatives = 0;
  int total_iters = 0;
  int outer_iters = 0;
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
      if (augmented_cache.kkt.solver) {
        result.linear_solver_backend =
            augmented_cache.kkt.solver->backend_name();
      }
      result.symbolic_analyses =
          retired_symbolic_analyses + augmented_cache.kkt.symbolic_analyses;
      result.numeric_factorizations =
          retired_numeric_factorizations +
          augmented_cache.kkt.numeric_factorizations;
      result.linear_solves =
          retired_linear_solves + augmented_cache.kkt.linear_solves;
    } else if (opt.use_inertia_correction) {
      if (kkt_cache.augmented.solver) {
        result.linear_solver_backend =
            kkt_cache.augmented.solver->backend_name();
      }
      result.symbolic_analyses =
          retired_symbolic_analyses + kkt_cache.augmented.symbolic_analyses;
      result.numeric_factorizations =
          retired_numeric_factorizations +
          kkt_cache.augmented.numeric_factorizations;
      result.linear_solves =
          retired_linear_solves + kkt_cache.augmented.linear_solves;
    } else {
      if (regularized_kkt_cache.solver) {
        result.linear_solver_backend =
            regularized_kkt_cache.solver->backend_name();
      }
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
    result.accepted_steps = accepted_steps;
    result.rejected_steps = rejected_steps;
    result.trial_value_evaluations = trial_value_evaluations;
    result.trial_full_derivative_evaluations =
        trial_full_derivative_evaluations;
    result.trial_rejections_before_derivatives =
        trial_rejections_before_derivatives;
  };

  for (; total_iters < max_total; ++outer_iters) {
    const double inner_neighborhood =
        opt.kappa_epsilon > 0.0 && std::isfinite(opt.kappa_epsilon)
            ? opt.kappa_epsilon
            : 1.0;
    double inner_tol = inner_neighborhood * mu_bar;
    const bool at_minimum_barrier =
        mu_bar <= std::nextafter(
            effective_mu_min, std::numeric_limits<double>::infinity());
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
        inner_tol = std::min(inner_tol, requested_accuracy);
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
                          : Eigen::VectorXd(
                                s.cwiseProduct(mu_ineq).array() - mu_bar);

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
      bool use_augmented_newton = false;
      bool use_exact_condensed_newton = false;
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
      use_augmented_newton =
          newton_profile.selected == "augmented" && state.jh.rows() > 0;
      auto solve_augmented_candidate = [&]() {
        augmented_cache.n = static_cast<int>(state.hess.rows());
        augmented_cache.meq = static_cast<int>(state.jg.rows());
        augmented_cache.miq = static_cast<int>(state.jh.rows());
        Eigen::VectorXd rhs_ineq_aug(augmented_cache.miq);
        for (int i = 0; i < augmented_cache.miq; ++i) {
          const double mui = std::max(mu_ineq[i], kMinPositive);
          rhs_ineq_aug[i] =
              -r_ineq[i] - (mu_bar - s[i] * mu_ineq[i]) / mui;
        }
        Eigen::VectorXd rhs_aug(
            augmented_cache.n + augmented_cache.meq + augmented_cache.miq);
        rhs_aug << -r_d, -r_eq, rhs_ineq_aug;
        // A Schur-complement regularization is not a certificate for the
        // uncondensed matrix. Start the equivalent formulation at the exact
        // KKT equations and let its own backward-error path decide.
        double augmented_delta_w = 0.0;
        if (!factor_solve_augmented_newton(
                augmented_cache, state.hess, state.jg, state.jh, s, mu_ineq,
                augmented_delta_w, isettings, !opt.primal_feasible_start,
                rhs_aug, dx, dlambda, dmu_ineq)) {
          return false;
        }
        delta_w_last = augmented_delta_w;
        rhs = std::move(rhs_aug);
        ds = -r_ineq - state.jh * dx;
        return true;
      };
      if (use_augmented_newton) {
        // ── Augmented (uncondensed) Newton path ──
        // [H + δ_W I, Jgᵀ, Jhᵀ; Jg, 0, 0; Jh, 0, -SM⁻¹] (dx, dλ, dμ) =
        //   [-r_d; -r_eq; -r_ineq - M⁻¹(μ̄e - Sμ)]
        if (!solve_augmented_candidate()) {
          if (opt.newton_formulation == NewtonFormulation::Auto) {
            use_augmented_newton = false;
            newton_profile.selected = "condensed";
            if (opt.verbose) {
              std::cerr << "[NativeIPM] augmented Newton certificate failed; "
                           "trying equivalent condensed formulation\n";
            }
          } else {
            snapshot_outcome(
                false, total_iters + 1,
                "Filter: augmented KKT regularization cap exceeded", rs);
            return result;
          }
        }
        if (use_augmented_newton && opt.verbose) {
          std::cerr << "[NativeIPM] iter=" << total_iters
                    << " augmented Newton: dim="
                    << augmented_cache.n + augmented_cache.meq +
                           augmented_cache.miq
                    << " nnz=" << augmented_cache.kkt.nnz
                    << ", seconds="
                    << std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - w_build_start)
                           .count()
                    << '\n' << std::flush;
        }
      }
      if (!use_augmented_newton) {
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
      // Trust-region safeguard shared by the exact-feasible and inertia-
      // corrected Newton paths. A near-singular reduced Hessian (structural in
      // OPF: PQ-bus voltages carry zero objective curvature) can certify the
      // target inertia yet yield an enormous, fraction-to-boundary-blocked
      // Newton direction that freezes progress ("accepted-step collapse").
      constexpr double kDirectionCapRatio = 1.0e3;
      constexpr int kDirectionEscalationCap = 3;
      const double direction_cap =
          kDirectionCapRatio * std::max(1.0, inf_norm(rhs));

      use_exact_condensed_newton = opt.primal_feasible_start &&
          factor_kkt_sparse(regularized_kkt_cache, w, state.jg, 0.0) &&
          exact_condensed_factor_inertia_ok(
              regularized_kkt_cache, static_cast<int>(state.jg.rows())) &&
          solve_kkt_sparse(regularized_kkt_cache, rhs, dx, dlambda);
      // The unregularized exact-feasible factor has no inertia guard; when it
      // returns a pathologically large direction the primal block is nearly
      // singular, so fall through to the inertia-corrected path below.
      if (use_exact_condensed_newton && inf_norm(dx) > direction_cap) {
        use_exact_condensed_newton = false;
      }
      const bool kkt_ok = use_exact_condensed_newton || (opt.use_inertia_correction
          ? (factor_kkt_inertia_corrected_sparse(
                 w, state.jg, isettings, delta_w_last, kkt_cache, istatus,
                 sticky_min_delta_w) &&
             solve_kkt_inertia_corrected_sparse(
                 kkt_cache, rhs, dx, dlambda))
          : (factor_kkt_with_regularization(
                 w, state.jg, regularized_kkt_cache) &&
             solve_kkt_sparse(regularized_kkt_cache, rhs, dx, dlambda)));
      // When the inertia-corrected factor certifies the target inertia on a
      // razor-thin reduced margin, its direction can still be enormous.
      // Escalate delta_w and re-solve so the step is bounded. Because the
      // reduced margin is min_curvature + delta_w (a small difference of
      // near-equal magnitudes), one or two doublings restore a healthy margin.
      // Well-conditioned directions never trigger this; an unnecessary
      // escalation only yields a more conservative descent step.
      double escalated_delta_w = 0.0;
      if (kkt_ok && opt.use_inertia_correction && !use_exact_condensed_newton) {
        double forced_delta_w = 0.0;
        for (int escalation = 0;
             escalation < kDirectionEscalationCap &&
             inf_norm(dx) > direction_cap;
             ++escalation) {
          forced_delta_w =
              std::max(istatus.delta_w_used, forced_delta_w) * 2.0;
          Eigen::VectorXd dx_prev = dx;
          Eigen::VectorXd dlambda_prev = dlambda;
          if (!(factor_kkt_inertia_corrected_sparse(
                    w, state.jg, isettings, delta_w_last, kkt_cache, istatus,
                    forced_delta_w) &&
                solve_kkt_inertia_corrected_sparse(
                    kkt_cache, rhs, dx, dlambda))) {
            dx = std::move(dx_prev);
            dlambda = std::move(dlambda_prev);
            break;
          }
          escalated_delta_w = forced_delta_w;
          ++inertia_retry_factorizations;
        }
      }
      // Warm-start memory: remember an escalated shift so the next iteration
      // starts already regularized (a single factor) instead of re-escalating
      // from the minimal-inertia value, then geometrically decay it so a
      // transient ill-conditioning does not permanently over-regularize.
      if (escalated_delta_w > 0.0) {
        sticky_min_delta_w = escalated_delta_w;
      } else {
        sticky_min_delta_w *= 0.5;
        if (sticky_min_delta_w <
            std::sqrt(std::numeric_limits<double>::epsilon())) {
          sticky_min_delta_w = 0.0;
        }
      }
      if (opt.use_inertia_correction) {
        inertia_retry_factorizations +=
            std::max(0, istatus.factorization_attempts - 1);
      }
      if (!kkt_ok) {
        if (opt.newton_formulation == NewtonFormulation::Auto &&
            state.jh.rows() > 0 && solve_augmented_candidate()) {
          use_augmented_newton = true;
          newton_profile.selected = "augmented";
          if (opt.verbose) {
            std::cerr << "[NativeIPM] condensed Newton certificate failed; "
                         "using equivalent augmented formulation\n";
          }
        } else {
          snapshot_outcome(false, total_iters + 1,
                           opt.use_inertia_correction
                               ? "Filter: KKT inertia correction cap exceeded"
                               : "Filter: regularized KKT factorization failed",
                           rs);
          return result;
        }
      }
      bool condensation_precision_exhausted = false;
      double condensed_full_residual = 0.0;
      double condensed_full_residual_limit = 0.0;
      if (!use_augmented_newton &&
          opt.newton_formulation == NewtonFormulation::Auto &&
          state.jh.rows() > 0) {
        const Eigen::VectorXd condensed_ds = -r_ineq - state.jh * dx;
        Eigen::VectorXd condensed_dmu(mu_ineq.size());
        for (int i = 0; i < s.size(); ++i) {
          const double si = std::max(s[i], kMinPositive);
          condensed_dmu[i] =
              (mu_bar - s[i] * mu_ineq[i] -
               mu_ineq[i] * condensed_ds[i]) / si;
        }
        const Eigen::VectorXd hessian_term = state.hess * dx;
        const Eigen::VectorXd equality_dual_term =
            state.jg.transpose() * dlambda;
        const Eigen::VectorXd inequality_dual_term =
            state.jh.transpose() * condensed_dmu;
        const Eigen::VectorXd regularization_term =
            istatus.delta_w_used * dx;
        const Eigen::VectorXd full_dual =
            r_d + hessian_term + equality_dual_term +
            inequality_dual_term + regularization_term;
        const Eigen::VectorXd full_equality =
            r_eq + state.jg * dx - istatus.delta_c_used * dlambda;
        const Eigen::VectorXd full_inequality =
            r_ineq + state.jh * dx + condensed_ds;
        const Eigen::VectorXd full_complementarity =
            r_comp + mu_ineq.cwiseProduct(condensed_ds) +
            s.cwiseProduct(condensed_dmu);
        condensed_full_residual = std::max(
            {inf_norm(full_dual), inf_norm(full_equality),
             inf_norm(full_inequality), inf_norm(full_complementarity)});
        const double full_equation_scale = std::max(
            {1.0, inf_norm(r_d), inf_norm(hessian_term),
             inf_norm(equality_dual_term), inf_norm(inequality_dual_term),
             inf_norm(regularization_term), inf_norm(r_eq),
             inf_norm(state.jg * dx),
             inf_norm(istatus.delta_c_used * dlambda), inf_norm(r_ineq),
             inf_norm(state.jh * dx), inf_norm(condensed_ds),
             inf_norm(r_comp),
             inf_norm(mu_ineq.cwiseProduct(condensed_ds)),
             inf_norm(s.cwiseProduct(condensed_dmu))});
        condensed_full_residual_limit =
            comparison_roundoff(full_equation_scale);
        condensation_precision_exhausted =
            !std::isfinite(condensed_full_residual) ||
            condensed_full_residual > condensed_full_residual_limit;
        if (!condensation_precision_exhausted) {
          ds = condensed_ds;
          dmu_ineq = condensed_dmu;
        }
      }
      if (condensation_precision_exhausted) {
        // The formed Schur solve failed its original block-equation backward
        // error. Its regularization is not transferable to the uncondensed
        // matrix, whose own exact-first certificate starts from zero.
        if (!solve_augmented_candidate()) {
          snapshot_outcome(false, total_iters + 1,
                           "Filter: both Newton formulations failed their "
                           "backward-error certificates", rs);
          return result;
        }
        use_augmented_newton = true;
        newton_profile.selected = "augmented";
        if (opt.verbose) {
          std::cerr
              << "[NativeIPM] condensed block residual exhausted precision; "
                 "using augmented Newton at iter="
              << total_iters << " (residual=" << condensed_full_residual
              << ", limit=" << condensed_full_residual_limit << ")\n";
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
          dmu_ineq[i] = (mu_bar - s[i] * mu_ineq[i] -
                         mu_ineq[i] * ds[i]) / si;
        }
      }
      }  // end condensed (explicit Jhᵀ(M/S)Jh) Newton path

      if (opt.verbose) {
        const Eigen::VectorXd linearized_dual =
            r_d + state.hess * dx + state.jg.transpose() * dlambda +
            state.jh.transpose() * dmu_ineq;
        const Eigen::VectorXd linearized_equality = r_eq + state.jg * dx;
        const Eigen::VectorXd linearized_inequality =
            r_ineq + state.jh * dx + ds;
        const Eigen::VectorXd linearized_complementarity = s.size() == 0
            ? Eigen::VectorXd()
            : Eigen::VectorXd(
                  r_comp + mu_ineq.cwiseProduct(ds) +
                  s.cwiseProduct(dmu_ineq));
        std::cerr << "[NativeIPM] iter=" << total_iters
                  << " Newton linearized residuals: dual="
                  << inf_norm(linearized_dual)
                  << ", equality=" << inf_norm(linearized_equality)
                  << ", inequality=" << inf_norm(linearized_inequality)
                  << ", complementarity="
                  << inf_norm(linearized_complementarity) << '\n'
                  << std::flush;
      }

      // Fraction-to-boundary step caps.
      const double tau = effective_fraction_to_boundary(opt.alpha_max);
      double alpha_max_primal =
          (s.size() == 0 || opt.primal_feasible_start)
              ? 1.0
              : max_positive_step(s, ds, tau);
      double alpha_max_dual =
          (mu_ineq.size() == 0) ? 1.0
                                : max_positive_step(mu_ineq, dmu_ineq, tau);

      const double theta_k = compute_theta(state.g, state.h, s);
      const double theta_min = parameterized_switching_filter
          ? opt.filter_theta_min_scale * std::max(1.0, theta_k)
          : 0.0;
      double alpha = std::min(1.0, alpha_max_primal);

      // Filter line search.
      const double phi_k = compute_barrier_phi(state.obj_orig, mu_bar, s);
      const double slope_k =
          barrier_descent_slope(state.grad, dx, s, ds, mu_bar);

      // Once the barrier direction ceases to be a descent direction while
      // primal feasibility is still outside the caller's gate, its remaining
      // role is purely normal restoration.  Hand that state to the dedicated
      // restoration model instead of accepting successively smaller filter
      // steps that cannot make objective progress.
      if (opt.use_restoration_phase && slope_k >= 0.0 &&
          rs.primal_feas > opt.tol_primal) {
        snapshot_outcome(false, total_iters + 1,
                         "Filter: accepted-step collapse", rs);
        return result;
      }

      bool accepted = false;
      bool accepted_was_f_type = false;
      TrialPoint accepted_trial;
      double theta_trial = 0.0;
      double phi_trial = 0.0;
      bool soc_considered = false;
      const double alpha0 = alpha;  // remember alpha_max for SOC gate
      const double alpha_dual = std::min(1.0, alpha_max_dual);
      // The line search advances the full primal-dual iterate.  Basing its
      // representability limit on (dx, ds) alone makes a pure multiplier
      // Newton step appear to have a zero direction and skips every trial.
      const double direction_scale = std::max(
          {inf_norm(dx), inf_norm(ds), inf_norm(dlambda),
           inf_norm(dmu_ineq)});
      const double iterate_scale = std::max(
          {1.0, inf_norm(x), inf_norm(s), inf_norm(lambda),
           inf_norm(mu_ineq)});
      const double resolvable_alpha =
          std::numeric_limits<double>::epsilon() * iterate_scale /
          std::max(direction_scale, minimum_safe_positive());
      const double alpha_lower_bound =
          opt.filter_alpha_min > 0.0 && std::isfinite(opt.filter_alpha_min)
              ? opt.filter_alpha_min
              : resolvable_alpha;
      bool first_filter_admissible_logged = false;
      if (opt.verbose) {
        Eigen::Index dx_index = -1;
        if (dx.size() > 0) dx.cwiseAbs().maxCoeff(&dx_index);
        int primal_blocker = -1;
        double primal_blocker_ratio = std::numeric_limits<double>::infinity();
        int dual_blocker = -1;
        double dual_blocker_ratio = std::numeric_limits<double>::infinity();
        for (int row = 0; row < ds.size(); ++row) {
          if (ds[row] < 0.0) {
            const double ratio = -s[row] / ds[row];
            if (ratio < primal_blocker_ratio) {
              primal_blocker_ratio = ratio;
              primal_blocker = row;
            }
          }
        }
        for (int row = 0; row < dmu_ineq.size(); ++row) {
          if (dmu_ineq[row] < 0.0) {
            const double ratio = -mu_ineq[row] / dmu_ineq[row];
            if (ratio < dual_blocker_ratio) {
              dual_blocker_ratio = ratio;
              dual_blocker = row;
            }
          }
        }
        std::cerr << "[NativeIPM] iter=" << total_iters
                   << " direction: |dx|inf=" << inf_norm(dx)
                   << ", dx_index=" << dx_index
                   << ", dx_name="
                   << diagnostic_variable_name(
                          prob, static_cast<int>(dx_index))
                   << ", x=" << (dx_index >= 0 ? x[dx_index] : 0.0)
                   << ", |ds|inf=" << inf_norm(ds)
                   << ", |dmu|inf=" << inf_norm(dmu_ineq)
                   << ", alpha_pri_max=" << alpha_max_primal
                   << ", alpha_dual_max=" << alpha_max_dual
                   << ", primal_blocker=" << primal_blocker
                   << ", primal_blocker_name="
                   << diagnostic_inequality_name(
                          prob, primal_blocker,
                          static_cast<int>(state.h.size()),
                          state.n_nonlinear_ineq, lb_cols, ub_cols)
                   << ", blocker_s="
                   << (primal_blocker >= 0 ? s[primal_blocker] : 0.0)
                   << ", blocker_ds="
                   << (primal_blocker >= 0 ? ds[primal_blocker] : 0.0)
                   << ", dual_blocker=" << dual_blocker
                   << ", dual_blocker_name="
                   << diagnostic_inequality_name(
                          prob, dual_blocker,
                          static_cast<int>(state.h.size()),
                          state.n_nonlinear_ineq, lb_cols, ub_cols)
                   << ", blocker_mu="
                   << (dual_blocker >= 0 ? mu_ineq[dual_blocker] : 0.0)
                   << ", blocker_dmu="
                   << (dual_blocker >= 0 ? dmu_ineq[dual_blocker] : 0.0)
                   << ", theta=" << theta_k
                  << ", phi=" << phi_k
                  << ", slope=" << slope_k << '\n' << std::flush;
      }

      while (alpha > alpha_lower_bound) {
        // A caller-audited feasible handoff has separate primal and dual
        // fraction-to-boundary intervals.  Coupling alpha_dual to alpha makes
        // one nearly active slack freeze every multiplier even when the dual
        // positivity interval permits the Newton update.  The independently
        // recomputed Phase-I KKT gate below is the acceptance certificate.
        const double trial_alpha_dual = opt.primal_feasible_start
            ? alpha_dual
            : std::min(alpha_dual, alpha);
        const Eigen::VectorXd x_trial = x + alpha * dx;
        Eigen::VectorXd s_trial =
          (s.size() == 0) ? Eigen::VectorXd() : Eigen::VectorXd(s + alpha * ds);
        // Primal and dual step lengths are independent, but both updates must
        // stay on their audited Newton rays.  In particular, a Phase-I
        // continuation must not replace the fraction-to-boundary dual step by
        // the full endpoint: doing so can create a negative multiplier before
        // the original-coordinate KKT gate is evaluated.
        const Eigen::VectorXd lambda_trial =
            lambda + trial_alpha_dual * dlambda;
        const Eigen::VectorXd mu_trial =
            mu_ineq + trial_alpha_dual * dmu_ineq;

        // Safety: slack positivity (fraction-to-boundary guarantees it, but
        // numerical rounding can still dip).
        bool slack_valid = true;
        for (int i = 0; !opt.primal_feasible_start && i < s_trial.size();
             ++i) {
          if (!(s_trial[i] > 0.0)) { slack_valid = false; break; }
        }
        if (!slack_valid) {
          ++rejected_steps;
          alpha *= 0.5;
          continue;
        }

        TrialPoint trial;
        ++trial_value_evaluations;
        const bool trial_evaluated = opt.primal_feasible_start
            ? evaluate_primal_feasible_trial_point(
                  prob, lb_cols, ub_cols, x_trial, lambda, mu_ineq,
                  dlambda, dmu_ineq, s_trial, alpha, eval_status, trial)
            : evaluate_filter_trial_values(
                  prob, lb_cols, ub_cols, x_trial, s_trial, lambda_trial,
                  mu_trial, eval_status, trial);
        if (!trial_evaluated) {
          ++rejected_steps;
          ++trial_rejections_before_derivatives;
          alpha *= 0.5;
          continue;
        }
        if (opt.primal_feasible_start) {
          ++trial_full_derivative_evaluations;
        }

        if (opt.primal_feasible_start) s_trial = trial.s;

        theta_trial = compute_theta(trial.state.g, trial.state.h, s_trial);
        phi_trial = compute_barrier_phi(
            trial.state.obj_orig, mu_bar, s_trial);

        // A filter only sees primal feasibility and barrier objective. At an
        // already stationary primal point, a valid Newton step may update only
        // lambda/mu, leaving both filter coordinates unchanged. Accept that
        // special case only when the primal displacement is at roundoff scale
        // and the independently recomputed KKT residuals prove componentwise
        // non-worsening plus strict dual/complementarity progress.
        const double relative_primal_direction = std::max(
            inf_norm(dx) / (1.0 + inf_norm(x)),
            inf_norm(ds) / (1.0 + inf_norm(s)));
        const bool centrality_candidate =
            relative_primal_direction <=
                std::sqrt(std::numeric_limits<double>::epsilon()) ||
            opt.primal_feasible_start;
        if (centrality_candidate && !opt.primal_feasible_start) {
          const Eigen::VectorXd r_dual_trial =
              state.grad + state.jg.transpose() * lambda_trial +
              state.jh.transpose() * mu_trial;
          const Eigen::VectorXd r_eq_trial = trial.state.g;
          const Eigen::VectorXd r_ineq_trial = trial.state.h + s_trial;
          trial.residuals = summarize_residuals(
              r_dual_trial, r_eq_trial, r_ineq_trial, x_trial, s_trial,
              lambda_trial, mu_trial);
          // A Phase-I handoff owns a tolerance-defined feasible
          // neighbourhood, not the often much smaller residual of its exact
          // endpoint.  Requiring every tangential/centrality step to preserve
          // that accidental residual can reduce a valid full step to repeated
          // powers-of-two backtracking even though all caller KKT gates remain
          // satisfied.  The neighbourhood below is derived solely from the
          // caller's contract; ordinary cold starts retain the roundoff guard.
          const double primal_guard =
              rs.primal_feas + arithmetic_roundoff(rs.primal_feas);
          const double dual_guard =
              rs.dual_feas + arithmetic_roundoff(rs.dual_feas);
          const bool componentwise_safe =
              trial.residuals.primal_feas <= primal_guard &&
              trial.residuals.dual_feas <= dual_guard;
          const bool complementarity_progress = s.size() > 0 &&
              trial.residuals.complementarity +
                      arithmetic_roundoff(trial.residuals.complementarity) <
                  rs.complementarity;
          const bool dual_progress =
              trial.residuals.dual_feas +
                      arithmetic_roundoff(trial.residuals.dual_feas) <
                  rs.dual_feas;
          const bool merit_progress =
              trial.residuals.merit +
                      arithmetic_roundoff(trial.residuals.merit) <
                  rs.merit;
          if (opt.verbose && alpha == alpha0) {
            std::cerr << "[NativeIPM] iter=" << total_iters
                      << " centrality gate: componentwise_safe="
                      << componentwise_safe << ", theta_safe="
                      << filter.satisfies_theta_upper_bound(theta_trial)
                      << ", merit_progress=" << merit_progress
                      << ", complementarity_progress="
                      << complementarity_progress << ", dual_progress="
                      << dual_progress << ", residuals=(primal="
                      << rs.primal_feas << " -> "
                      << trial.residuals.primal_feas << ",dual="
                      << rs.dual_feas << " -> "
                      << trial.residuals.dual_feas << ",complementarity="
                      << rs.complementarity << " -> "
                      << trial.residuals.complementarity << ",merit="
                      << rs.merit << " -> " << trial.residuals.merit
                      << ")\n" << std::flush;
          }
          if (componentwise_safe &&
              filter.satisfies_theta_upper_bound(theta_trial) &&
              merit_progress &&
              (complementarity_progress || dual_progress)) {
            if (!opt.primal_feasible_start) {
              ++trial_full_derivative_evaluations;
            }
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

        bool phase1_kkt_safe = true;
        if (opt.primal_feasible_start) {
          const bool primal_safe =
              trial.residuals.primal_feas < phase1_primal_upper_bound;
          const bool dual_safe =
              trial.residuals.dual_feas <=
              rs.dual_feas + comparison_roundoff(rs.dual_feas);
          const bool complementarity_safe =
              trial.residuals.complementarity <=
              rs.complementarity + comparison_roundoff(rs.complementarity);
          const bool dual_progress =
              trial.residuals.dual_feas +
                      relative_roundoff(trial.residuals.dual_feas,
                                        rs.dual_feas) <
                  rs.dual_feas;
          const bool complementarity_progress =
              trial.residuals.complementarity +
                      relative_roundoff(trial.residuals.complementarity,
                                        rs.complementarity) <
                  rs.complementarity;
          phase1_kkt_safe = primal_safe && dual_safe &&
              complementarity_safe &&
              (dual_progress || complementarity_progress);
        }

        if (opt.primal_feasible_start && phase1_kkt_safe) {
          accepted = true;
          accepted_was_f_type = true;
          accepted_trial = std::move(trial);
          if (opt.verbose) {
            std::cerr << "[NativeIPM] iter=" << total_iters
                      << " Phase-I KKT step accepted: alpha=" << alpha
                      << ", dual_update=rowwise_cone_projection"
                      << ", residuals=(primal="
                      << accepted_trial.residuals.primal_feas
                      << ",dual=" << accepted_trial.residuals.dual_feas
                      << ",complementarity="
                      << accepted_trial.residuals.complementarity << ")\n"
                      << std::flush;
          }
          break;
        }

        const bool filter_ok = filter.is_acceptable(
            theta_trial, phi_trial, filter_gamma_theta, filter_gamma_phi);

        const bool switching =
            parameterized_switching_filter &&
            (slope_k < 0.0) &&
            (alpha * std::pow(-slope_k, opt.filter_s_phi) >
             opt.filter_delta * std::pow(theta_k, opt.filter_s_theta)) &&
            (theta_k <= theta_min);

        bool accept = false;
        if (filter_ok) {
          if (switching) {
            accept = phi_trial <=
                     phi_k + opt.filter_eta_phi * alpha * slope_k +
                         comparison_roundoff(phi_k);
            accepted_was_f_type = accept;
          } else {
            accept = parameterized_switching_filter
                ? ((theta_trial <=
                    (1.0 - filter_gamma_theta) * theta_k) ||
                   (phi_trial <= phi_k - filter_gamma_phi * theta_k))
                : (theta_trial + arithmetic_roundoff(theta_trial) < theta_k ||
                   phi_trial + arithmetic_roundoff(phi_trial) < phi_k);
            accepted_was_f_type = false;
          }
        }
        if (opt.verbose && opt.primal_feasible_start && filter_ok &&
            !first_filter_admissible_logged) {
          first_filter_admissible_logged = true;
          std::cerr << "[NativeIPM] iter=" << total_iters
                    << " first filter-admissible trial: alpha=" << alpha
                    << ", theta=" << theta_trial << ", phi=" << phi_trial
                    << ", switching=" << switching
                    << ", accepted=" << accept << '\n';
        }

        if (accept) {
          if (!opt.primal_feasible_start) {
            ++trial_full_derivative_evaluations;
          }
          accepted = true;
          accepted_trial = std::move(trial);
          if (opt.verbose) {
            std::cerr << "[NativeIPM] iter=" << total_iters
                      << " line search accepted: alpha=" << alpha;
            if (opt.primal_feasible_start) {
              std::cerr << ", dual_update=rowwise_cone_projection";
            } else {
              std::cerr << ", alpha_dual=" << trial_alpha_dual;
            }
            std::cerr << ", theta_trial=" << theta_trial
                      << ", phi_trial=" << phi_trial
                      << '\n' << std::flush;
          }
          break;
        }

        if (!opt.primal_feasible_start) {
          ++trial_rejections_before_derivatives;
        }

        if (opt.verbose && alpha == alpha0) {
          std::cerr << "[NativeIPM] iter=" << total_iters
                    << " full step rejected: filter_ok=" << filter_ok
                    << ", switching=" << switching
                    << ", theta_trial=" << theta_trial
                    << ", phi_trial=" << phi_trial
                    << ", trial_residuals=(primal="
                    << trial.residuals.primal_feas
                    << ",dual=" << trial.residuals.dual_feas
                    << ",complementarity="
                    << trial.residuals.complementarity << ')'
                    << '\n' << std::flush;
        }

        // Second-order correction: when feasibility blocks the relevant trial,
        // resolve the KKT with RHS
        //   (rhs_x, −(α · g + g(x + α dx)))
        // and test its unique maximal-step composite candidate. Phase-I uses
        // the same model whenever its audited primal neighbourhood is violated.
        const bool phase1_needs_soc = opt.primal_feasible_start &&
            trial.residuals.primal_feas >= phase1_primal_upper_bound;
        const bool cold_start_needs_soc = !opt.primal_feasible_start &&
            alpha == alpha0 && theta_trial > theta_k;
        if (opt.use_second_order_correction &&
            (phase1_needs_soc || cold_start_needs_soc) &&
            r_eq.size() > 0) {
          soc_considered = true;

          const Eigen::VectorXd c_soc =
              trial.state.g - (1.0 - alpha) * r_eq;
          Eigen::VectorXd rhs_soc = Eigen::VectorXd::Zero(rhs.size());
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
            // A second-order correction solves only the nonlinear equality
            // remainder; it must not repeat the primal Newton forcing term.
            rhs_soc.tail(r_eq.size()) = -c_soc;
            if (use_exact_condensed_newton) {
              soc_ok = solve_kkt_sparse(
                  regularized_kkt_cache, rhs_soc, dx_soc, dlambda_soc);
            } else if (opt.use_inertia_correction) {
              soc_ok = solve_kkt_inertia_corrected_sparse(
                  kkt_cache, rhs_soc, dx_soc, dlambda_soc);
            } else {
              soc_ok = solve_kkt_sparse(
                  regularized_kkt_cache, rhs_soc, dx_soc, dlambda_soc);
            }
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
            for (int i = 0;
                 !opt.primal_feasible_start && i < s_soc_trial.size(); ++i) {
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
              ++trial_value_evaluations;
              const bool soc_trial_evaluated = opt.primal_feasible_start
                  ? evaluate_primal_feasible_trial_point(
                        prob, lb_cols, ub_cols, x_soc_trial,
                        lambda, mu_ineq, dlambda, dmu_ineq, s_soc_trial,
                        alpha, eval_status, soc_trial)
                  : evaluate_filter_trial_values(
                        prob, lb_cols, ub_cols, x_soc_trial, s_soc_trial,
                        lambda_soc_trial, mu_soc_trial, eval_status, soc_trial);
              if (soc_trial_evaluated) {
                if (opt.primal_feasible_start) {
                  ++trial_full_derivative_evaluations;
                }
                if (opt.primal_feasible_start) s_soc_trial = soc_trial.s;
                const double theta_soc = compute_theta(soc_trial.state.g,
                                                       soc_trial.state.h,
                                                       s_soc_trial);
                const double phi_soc = compute_barrier_phi(
                    soc_trial.state.obj_orig, mu_bar, s_soc_trial);

                bool soc_phase1_kkt_safe = true;
                if (opt.primal_feasible_start) {
                  const bool primal_safe =
                      soc_trial.residuals.primal_feas <
                      phase1_primal_upper_bound;
                  const bool dual_safe =
                      soc_trial.residuals.dual_feas <=
                      rs.dual_feas + comparison_roundoff(rs.dual_feas);
                  const bool complementarity_safe =
                      soc_trial.residuals.complementarity <=
                      rs.complementarity +
                          comparison_roundoff(rs.complementarity);
                  const bool dual_progress =
                      soc_trial.residuals.dual_feas +
                              relative_roundoff(
                                  soc_trial.residuals.dual_feas,
                                  rs.dual_feas) <
                          rs.dual_feas;
                  const bool complementarity_progress =
                      soc_trial.residuals.complementarity +
                              relative_roundoff(
                                  soc_trial.residuals.complementarity,
                                  rs.complementarity) <
                          rs.complementarity;
                  soc_phase1_kkt_safe = primal_safe && dual_safe &&
                      complementarity_safe &&
                      (dual_progress || complementarity_progress);
                  if (opt.verbose && alpha == alpha0) {
                    std::cerr << "[NativeIPM] iter=" << total_iters
                              << " Phase-I SOC candidate: alpha=" << alpha
                              << ", safe=" << soc_phase1_kkt_safe
                              << ", residuals=(primal="
                              << soc_trial.residuals.primal_feas
                              << ",dual="
                              << soc_trial.residuals.dual_feas
                              << ",complementarity="
                              << soc_trial.residuals.complementarity << ")\n"
                              << std::flush;
                  }
                }
                const bool soc_filter_ok = filter.is_acceptable(
                    theta_soc, phi_soc, filter_gamma_theta,
                    filter_gamma_phi);
                bool soc_accept =
                    opt.primal_feasible_start && soc_phase1_kkt_safe;
                if (soc_filter_ok) {
                  if (switching) {
                    soc_accept = phi_soc <= phi_k +
                                 opt.filter_eta_phi * alpha * slope_k +
                                     comparison_roundoff(phi_k);
                    accepted_was_f_type = soc_accept;
                  } else {
                    soc_accept = parameterized_switching_filter
                        ? ((theta_soc <=
                            (1.0 - filter_gamma_theta) * theta_k) ||
                           (phi_soc <=
                            phi_k - filter_gamma_phi * theta_k))
                        : (theta_soc + arithmetic_roundoff(theta_soc) <
                               theta_k ||
                           phi_soc + arithmetic_roundoff(phi_soc) < phi_k);
                    accepted_was_f_type = false;
                  }
                }

                if (soc_accept) {
                  if (!opt.primal_feasible_start) {
                    ++trial_full_derivative_evaluations;
                  }
                  theta_trial = theta_soc;
                  phi_trial = phi_soc;
                  accepted_trial = std::move(soc_trial);
                  accepted = true;
                  if (opt.verbose) {
                    std::cerr << "[NativeIPM] iter=" << total_iters
                              << " SOC accepted: alpha=" << alpha
                              << ", residuals=(primal="
                              << accepted_trial.residuals.primal_feas
                              << ",dual="
                              << accepted_trial.residuals.dual_feas
                              << ",complementarity="
                              << accepted_trial.residuals.complementarity
                              << ")\n" << std::flush;
                  }
                  break;
                }
                if (!opt.primal_feasible_start) {
                  ++trial_rejections_before_derivatives;
                }
              } else {
                ++trial_rejections_before_derivatives;
              }
            }
          }
        }

        ++rejected_steps;
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
                    << ", soc_considered=" << soc_considered
                    << '\n' << std::flush;
        }
        if (have_best && !opt.primal_feasible_start) {
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
      state = std::move(accepted_trial.state);
      ++accepted_steps;

      // Phase-I trials already carry complete derivatives. Ordinary filter
      // trials intentionally evaluate values only; on the final allowed step,
      // perform the derivative audit that the next loop iteration would have
      // performed. This prevents a true last-step solution from being labeled
      // as an iteration-limit failure without adding another Newton step.
      bool accepted_residuals_are_complete = opt.primal_feasible_start;
      ResidualSummary accepted_residuals = accepted_trial.residuals;
      if (!accepted_residuals_are_complete && total_iters + 1 >= max_total) {
        TrialPoint certified_trial;
        ++trial_value_evaluations;
        if (evaluate_trial_point(prob, lb_cols, ub_cols, x, s, lambda,
                                 mu_ineq, eval_status, certified_trial)) {
          state = std::move(certified_trial.state);
          accepted_residuals = certified_trial.residuals;
          accepted_residuals_are_complete = true;
          ++trial_full_derivative_evaluations;
        } else {
          ++trial_rejections_before_derivatives;
        }
      }
      // This is the same public KKT contract used at the top of the loop; it
      // neither consumes an extra iteration nor substitutes a scaled metric.
      if (accepted_residuals_are_complete &&
          accepted_residuals.primal_feas <= opt.tol_primal &&
          accepted_residuals.dual_feas <= opt.tol_dual &&
          (s.size() == 0 ||
           accepted_residuals.complementarity <=
               opt.tol_complementarity)) {
        snapshot_outcome(true, total_iters + 1, "Converged",
                         accepted_residuals);
        return result;
      }

      // Filter augmentation: insert (θ_k, φ_k) for θ-type (non-f-type) steps.
      if (!accepted_was_f_type) {
        filter.add_entry(theta_k, phi_k,
                         filter_gamma_theta, filter_gamma_phi);
      }
    }

    if (!inner_converged_at_mu) break;  // iteration budget exhausted

    const bool explicit_barrier_policy =
        opt.kappa_mu > 0.0 && opt.kappa_mu < 1.0 &&
        opt.theta_mu > 1.0 && std::isfinite(opt.kappa_mu) &&
        std::isfinite(opt.theta_mu);
    const double minimum_product = s.size() > 0
        ? s.cwiseProduct(mu_ineq).minCoeff()
        : effective_mu_min;
    double mu_new = explicit_barrier_policy
        ? std::max(effective_mu_min,
                   std::min(opt.kappa_mu * mu_bar,
                            std::pow(mu_bar, opt.theta_mu)))
        : std::max(effective_mu_min,
                   std::min(opt.tol_complementarity, minimum_product));
    if (!explicit_barrier_policy && mu_new >= mu_bar &&
        effective_mu_min < mu_bar) {
      // A target equal to the caller's complementarity gate leaves no room
      // for the observed central-neighborhood error. Do not jump from that
      // target to the representability floor: derive the smallest useful
      // decrease from the current product residual plus its comparison error.
      // If the same local centrality error persists at the next target, its
      // largest product is then strictly inside the caller's gate.
      const double centrality_residual = s.size() > 0
          ? inf_norm(Eigen::VectorXd(
                s.cwiseProduct(mu_ineq).array() - mu_bar))
          : 0.0;
      const double resolvable_decrease = centrality_residual +
          relative_roundoff(mu_bar, 0.0);
      mu_new = std::max(effective_mu_min, mu_bar - resolvable_decrease);
    }
    if (mu_new >= mu_bar) {  // cannot decrease further — μ clamped at μ_min
      // We already failed the outer KKT check above (otherwise we would have
      // returned), even after the tightened final-barrier inner solve.
      terminal_status =
          "Filter: minimum barrier reached without KKT convergence";
      break;
    }
    mu_bar = mu_new;
    // Filter entries compare the barrier objective phi_mu. Once mu changes,
    // old (theta, phi_mu) pairs belong to a different objective and no longer
    // define valid dominance tests for the new barrier subproblem.
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
  const double strict_positive = strict_minimum_safe_positive();
  detail.mu_ineq = Eigen::VectorXd::Constant(mineq, strict_positive);
  detail.z_slack = Eigen::VectorXd::Constant(mineq, strict_positive);
  if (mnlin > 0) {
    detail.mu_ineq.head(mnlin) =
        fallback.constraint_duals.head(mnlin).array().max(strict_positive);
    detail.z_slack.head(mnlin) =
        (-nonlinear_inequalities.array()).max(strict_positive);
  }
  int row = mnlin;
  for (int col : lb_cols) {
    detail.mu_ineq[row] =
        std::max(fallback.box_dual_lb[col], strict_positive);
    detail.z_slack[row] = std::max(
        fallback.x[col] - prob.vars[static_cast<std::size_t>(col)].lb,
        strict_positive);
    ++row;
  }
  for (int col : ub_cols) {
    detail.mu_ineq[row] =
        std::max(fallback.box_dual_ub[col], strict_positive);
    detail.z_slack[row] = std::max(
        prob.vars[static_cast<std::size_t>(col)].ub - fallback.x[col],
        strict_positive);
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
  reduced.nonlinear_inequality_names =
      original.nonlinear_inequality_names;
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
      state_stationarity > comparison_roundoff(solve_scale)) {
    return false;
  }
  equality_dual = std::move(candidate);
  return true;
}

bool restore_fixed_nlp_certificate(
    const NLPModel& original, const NLPModel& reduced,
    const FixedNLPMap& map, SolveResult& out, IPMDetail& detail,
    const IPMOptions& opt) {
  const double reduced_dual_feasibility = out.stats.dual_feas;
  const int n = map.original_dimension;
  if (out.x.size() != static_cast<int>(map.reduced_to_original.size())) {
    return false;
  }
  out.x = map.expand(out.x);

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
    if (std::getenv("HACDCPF_OPF_TRACE") != nullptr && r_dual.size() > 0) {
      Eigen::Index worst_column = -1;
      r_dual.cwiseAbs().maxCoeff(&worst_column);
      const Eigen::VectorXd equality_term =
          original_state.jg.transpose() * detail.lambda_eq;
      const Eigen::VectorXd inequality_term =
          original_state.jh.transpose() * original_mu;
      const bool fixed = map.original_to_reduced[
          static_cast<std::size_t>(worst_column)] < 0;
      std::cerr << "[NativeIPM] fixed-variable postsolve audit: reduced_dual="
                << reduced_dual_feasibility << ", original_dual="
                << residuals.dual_feas << ", worst_column=" << worst_column
                << ", fixed=" << fixed
                << ", gradient=" << original_state.grad[worst_column]
                << ", equality_term=" << equality_term[worst_column]
                << ", inequality_term=" << inequality_term[worst_column]
                << ", residual=" << r_dual[worst_column] << '\n';
    }
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
  min_constraint_scale = std::clamp(
      min_constraint_scale, minimum_safe_positive(), 1.0);
  const double objective_scale = std::clamp(
      scaling.s_f, minimum_safe_positive(), 1.0);
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

NativeIPMAdapter::NativeIPMAdapter(IPMOptions opt) : opt_(std::move(opt)) {
  if (std::getenv("HACDCPF_OPF_TRACE") != nullptr) opt_.verbose = true;
}

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
  detail.primal_feasible_start_requested = opt_.primal_feasible_start;
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

  const bool valid_primal_start =
      prob.x0.size() == static_cast<int>(prob.vars.size()) &&
      prob.x0.allFinite();
  const double start_primal_violation = valid_primal_start
      ? raw_primal_violation(prob, prob.x0)
      : std::numeric_limits<double>::infinity();
  const bool primal_feasible_start_accepted =
      opt_.primal_feasible_start &&
      std::isfinite(start_primal_violation) &&
      start_primal_violation <= opt_.tol_primal;
  detail.primal_feasible_start_accepted = primal_feasible_start_accepted;
  if (opt_.verbose && opt_.primal_feasible_start) {
    std::cerr << "[NativeIPM] primal-feasible start: requested=1, accepted="
              << primal_feasible_start_accepted
              << ", original_primal=" << start_primal_violation
              << ", tolerance=" << opt_.tol_primal << '\n';
  }

  NLPModel reduced_prob;
  std::shared_ptr<FixedNLPMap> fixed_map;
  if (build_fixed_nlp_reduction(prob, reduced_prob, fixed_map)) {
    NativeIPMAdapter reduced_solver(opt_);
    auto reduced_result = reduced_solver.solve_nlp_detail(reduced_prob);
    out = std::move(reduced_result.first);
    detail = std::move(reduced_result.second);
    detail.primal_feasible_start_requested = opt_.primal_feasible_start;
    detail.primal_feasible_start_accepted =
        primal_feasible_start_accepted &&
        detail.primal_feasible_start_accepted;
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
    active_opt.primal_feasible_start = primal_feasible_start_accepted;
    if (primal_feasible_start_accepted) {
      active_opt.least_square_init_duals = true;
    }
    const bool do_scale = opt_.scale_problem;
    const double scaling_target =
        opt_.scaling_g_max > 0.0 && std::isfinite(opt_.scaling_g_max)
            ? opt_.scaling_g_max
            : 1.0;
    if (do_scale) {
      Eigen::VectorXd x0_interior = (prob.x0.size() == n_f)
                                        ? prob.x0
                                        : Eigen::VectorXd::Zero(n_f);
      interiorize_initial_point(prob.vars, x0_interior);
      sf = compute_scaling_factors(prob, x0_interior, scaling_target);
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
    int solve_chain_accepted_steps = fo.accepted_steps;
    int solve_chain_rejected_steps = fo.rejected_steps;
    int solve_chain_trial_value_evaluations = fo.trial_value_evaluations;
    int solve_chain_trial_full_derivative_evaluations =
        fo.trial_full_derivative_evaluations;
    int solve_chain_trial_rejections_before_derivatives =
        fo.trial_rejections_before_derivatives;
    int selected_base_numeric_factorizations = fo.numeric_factorizations;
    int selected_base_symbolic_analyses = fo.symbolic_analyses;
    int selected_base_linear_solves = fo.linear_solves;
    int selected_base_primary_factorizations = fo.primary_factorizations;
    int selected_base_inertia_retries = fo.inertia_retry_factorizations;
    int selected_base_inertia_certificates =
        fo.inertia_certificate_factorizations;
    int selected_base_polish_factorizations =
        fo.active_set_polish_factorizations;
    int selected_base_accepted_steps = fo.accepted_steps;
    int selected_base_rejected_steps = fo.rejected_steps;
    int selected_base_trial_value_evaluations = fo.trial_value_evaluations;
    int selected_base_trial_full_derivative_evaluations =
        fo.trial_full_derivative_evaluations;
    int selected_base_trial_rejections_before_derivatives =
        fo.trial_rejections_before_derivatives;
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
        rst_opt.max_iter = std::max(0, opt_.max_iter - fo.iterations);

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
        solve_chain_accepted_steps += rf.accepted_steps;
        solve_chain_rejected_steps += rf.rejected_steps;
        solve_chain_trial_value_evaluations += rf.trial_value_evaluations;
        solve_chain_trial_full_derivative_evaluations +=
            rf.trial_full_derivative_evaluations;
        solve_chain_trial_rejections_before_derivatives +=
            rf.trial_rejections_before_derivatives;

        if (rf.x.size() >= rb.n_x) {
          Eigen::VectorXd x_new = extract_x_from_restoration(rb, rf.x);
          restoration_normal_residual_after =
              raw_primal_violation(prob, x_new);
          const double normal_roundoff =
              comparison_roundoff(restoration_normal_residual_before);
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
            sf2 = compute_scaling_factors(prob_retry, x0i, scaling_target);
            scaled_retry = build_scaled_nlp_model(prob_retry, sf2);
            retry_prob = &scaled_retry;
          }

          IPMOptions retry_opt = opt_;
          retry_opt.use_restoration_phase = false;  // one-shot restoration
          if (restoration_warm_start_used) {
            retry_opt.equality_dual_start = warm.equality_dual;
            retry_opt.inequality_dual_start = warm.inequality_dual;
            retry_opt.slack_start = warm.slack;
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
          solve_chain_accepted_steps += fo_retry.accepted_steps;
          solve_chain_rejected_steps += fo_retry.rejected_steps;
          solve_chain_trial_value_evaluations +=
              fo_retry.trial_value_evaluations;
          solve_chain_trial_full_derivative_evaluations +=
              fo_retry.trial_full_derivative_evaluations;
          solve_chain_trial_rejections_before_derivatives +=
              fo_retry.trial_rejections_before_derivatives;
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
            selected_base_accepted_steps = fo_retry.accepted_steps;
            selected_base_rejected_steps = fo_retry.rejected_steps;
            selected_base_trial_value_evaluations =
                fo_retry.trial_value_evaluations;
            selected_base_trial_full_derivative_evaluations =
                fo_retry.trial_full_derivative_evaluations;
            selected_base_trial_rejections_before_derivatives =
                fo_retry.trial_rejections_before_derivatives;
          }
          }
        }
        }
      }
    }

    // --- Unscale multipliers back to the original problem's coordinates.
    bool original_coordinates_audited = false;
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
        original_coordinates_audited = true;
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
    const bool original_primal_gate_passed = original_coordinates_audited &&
        fo.final_residuals.primal_feas <= opt_.tol_primal;
    const bool original_kkt_gate_passed = original_primal_gate_passed &&
        fo.final_residuals.dual_feas <= opt_.tol_dual &&
        fo.final_residuals.complementarity <= opt_.tol_complementarity;
    // Coordinate continuation is selected from the original-coordinate KKT
    // contract, not from a catalogue of scaled-driver failure strings. Once
    // the normal residual passes its caller gate, any unresolved dual or cone
    // product belongs to the tangential Phase-II trajectory. Likewise, an
    // independently audited Phase-I start keeps ownership of that trajectory
    // if the scaled solve perturbs primal feasibility before terminating.
    const bool tangential_filter_stall = !fo.converged &&
        original_primal_gate_passed && !original_kkt_gate_passed;
    const bool phase1_scaled_filter_stall = !fo.converged &&
        primal_feasible_start_accepted && original_coordinates_audited &&
        !original_kkt_gate_passed;
    const bool continue_original_trajectory =
        needs_original_scale_refinement || tangential_filter_stall ||
        phase1_scaled_filter_stall;
    if (opt_.verbose) {
      std::cerr << "[NativeIPM] original-scale continuation: requested="
                << continue_original_trajectory
                << ", scaled_certification_failed="
                << needs_original_scale_refinement
                << ", tangential_filter_stall=" << tangential_filter_stall
                << ", phase1_scaled_filter_stall="
                << phase1_scaled_filter_stall
                << ", remaining_iterations=" << remaining_iterations
                << ", residuals=(primal="
                << fo.final_residuals.primal_feas
                << ",dual=" << fo.final_residuals.dual_feas
                << ",complementarity="
                << fo.final_residuals.complementarity << ")"
                << ", warm_dimensions=(lambda=" << fo.lambda.size()
                << ",mu=" << fo.mu_ineq.size()
                << ",slack=" << fo.s.size() << ")" << '\n';
    }
    if (continue_original_trajectory && remaining_iterations > 0 &&
        fo.x.size() == n_f && fo.lambda.allFinite() &&
        fo.mu_ineq.allFinite() && fo.s.allFinite() &&
        (fo.mu_ineq.array() > 0.0).all() &&
        (fo.s.array() > 0.0).all()) {
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
      // Coordinate continuation is part of the same Phase-II trajectory.
      // Preserve the Newton formulation selected for the audited Phase-I
      // handoff; re-running Auto on the unscaled sparsity estimates can switch
      // back to the condensed system precisely when low-barrier M/S terms make
      // that algebraic elimination numerically fragile.
      refine_opt.newton_formulation = active_opt.newton_formulation;
      refine_opt.use_augmented_newton = active_opt.use_augmented_newton;
      refine_opt.equality_dual_start = fo.lambda;
      refine_opt.inequality_dual_start = fo.mu_ineq;
      refine_opt.slack_start = fo.s;
      const double recovered_barrier = fo.mu_ineq.size() > 0
          ? fo.mu_ineq.dot(fo.s) / fo.mu_ineq.size()
          : 0.0;
      if (opt_.verbose) {
        std::cerr << "[NativeIPM] original-scale continuation start: "
                  << "max_iter=" << refine_opt.max_iter
                  << ", recovered_mu=" << recovered_barrier << '\n';
      }

      FilterSolveOutcome refined =
          solve_nlp_filter_impl(refine_prob, refine_opt);
      const bool adopt_refinement = refined.converged ||
          refined.final_residuals.merit < fo.final_residuals.merit;
      if (opt_.verbose) {
        std::cerr << "[NativeIPM] original-scale continuation finish: "
                  << "adopted=" << adopt_refinement
                  << ", converged=" << refined.converged
                  << ", iterations=" << refined.iterations
                  << ", before_merit=" << fo.final_residuals.merit
                  << ", after_merit=" << refined.final_residuals.merit
                  << ", residuals=(primal="
                  << refined.final_residuals.primal_feas
                  << ",dual=" << refined.final_residuals.dual_feas
                  << ",complementarity="
                  << refined.final_residuals.complementarity << ")"
                  << ", status=\"" << refined.status << "\"\n";
      }
      if (adopt_refinement) {
        if (refined.linear_solver_backend == "unselected") {
          refined.linear_solver_backend = fo.linear_solver_backend;
        }
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
        refined.accepted_steps += fo.accepted_steps;
        refined.rejected_steps += fo.rejected_steps;
        refined.trial_value_evaluations += fo.trial_value_evaluations;
        refined.trial_full_derivative_evaluations +=
            fo.trial_full_derivative_evaluations;
        refined.trial_rejections_before_derivatives +=
            fo.trial_rejections_before_derivatives;
        refined.initial_residuals = original_initial_residuals;
        if (refined.converged) {
          refined.status = (tangential_filter_stall ||
                            phase1_scaled_filter_stall)
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
        fo.accepted_steps += refined.accepted_steps;
        fo.rejected_steps += refined.rejected_steps;
        fo.trial_value_evaluations += refined.trial_value_evaluations;
        fo.trial_full_derivative_evaluations +=
            refined.trial_full_derivative_evaluations;
        fo.trial_rejections_before_derivatives +=
            refined.trial_rejections_before_derivatives;
        fo.iterations += refined.iterations;
      }
    }

    detail.newton_formulation = fo.newton_profile.selected;
    detail.linear_solver_backend = fo.linear_solver_backend;
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
    detail.accepted_steps = solve_chain_accepted_steps +
        std::max(0, fo.accepted_steps - selected_base_accepted_steps);
    detail.rejected_steps = solve_chain_rejected_steps +
        std::max(0, fo.rejected_steps - selected_base_rejected_steps);
    detail.trial_value_evaluations = solve_chain_trial_value_evaluations +
        std::max(0, fo.trial_value_evaluations -
                        selected_base_trial_value_evaluations);
    detail.trial_full_derivative_evaluations =
        solve_chain_trial_full_derivative_evaluations +
        std::max(0, fo.trial_full_derivative_evaluations -
                        selected_base_trial_full_derivative_evaluations);
    detail.trial_rejections_before_derivatives =
        solve_chain_trial_rejections_before_derivatives +
        std::max(0, fo.trial_rejections_before_derivatives -
                        selected_base_trial_rejections_before_derivatives);
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

    // Preserve the best finite primal-dual state even when the filter does not
    // certify convergence. Fixed-variable postsolve and external audit need
    // these vectors to evaluate the original-coordinate KKT residuals; their
    // presence never upgrades success without those independent gates.
    const bool complete_filter_state =
        fo.lambda.allFinite() && fo.mu_ineq.allFinite() && fo.s.allFinite() &&
        fo.mu_ineq.size() == fo.s.size();
    if (complete_filter_state) {
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
          out.box_dual_lb[fo.lb_cols[static_cast<size_t>(k)]] =
              fo.mu_ineq[idx];
        }
      }
      for (int k = 0; k < static_cast<int>(fo.ub_cols.size()); ++k) {
        const int idx = fo.n_nonlinear_ineq +
                        static_cast<int>(fo.lb_cols.size()) + k;
        if (idx < fo.mu_ineq.size()) {
          out.box_dual_ub[fo.ub_cols[static_cast<size_t>(k)]] =
              fo.mu_ineq[idx];
        }
      }
      detail.lambda_eq = fo.lambda;
      detail.mu_ineq = fo.mu_ineq;
      detail.z_slack = fo.s;
      detail.complementarity = fo.final_residuals.complementarity;
    }

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
          refine_opt.mu_init = std::max(
              refine_opt.mu_min,
              candidate_detail.mu_ineq.dot(candidate_detail.z_slack) /
                  candidate_detail.mu_ineq.size());
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
    qn_state.sparse_block_size =
        opt_.qn_sparse_block_size > 0 ? opt_.qn_sparse_block_size : n;
    qn_state.max_blocks = opt_.qn_max_blocks > 0 ? opt_.qn_max_blocks : 1;
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
  Eigen::VectorXd s = initialize_slacks_from_linearization_resolution(
      current_state, prob.vars, x);
  Eigen::VectorXd mu;
  const bool has_equality_dual_start =
      opt_.equality_dual_start.size() == lambda.size() &&
      opt_.equality_dual_start.allFinite();
  const bool has_inequality_dual_start =
      opt_.inequality_dual_start.size() == current_state.h.size() &&
      opt_.inequality_dual_start.allFinite() &&
      (opt_.inequality_dual_start.array() > 0.0).all();
  if (opt_.slack_start.size() == s.size() && opt_.slack_start.allFinite() &&
      (opt_.slack_start.array() > 0.0).all()) {
    s = opt_.slack_start;
  }
  double initial_barrier = 0.0;
  if (!initialize_cold_primal_dual_state(
          current_state, s, opt_.mu_init, lambda, mu, initial_barrier)) {
    out.stats.status = "NativeIPM scale-covariant initialization failed";
    return {out, detail};
  }
  if (has_inequality_dual_start) {
    mu = opt_.inequality_dual_start;
  }
  if (has_equality_dual_start) {
    lambda = opt_.equality_dual_start;
  } else if (opt_.least_square_init_duals && !qn_state.active) {
    initialize_equality_duals_least_squares(current_state, mu, lambda);
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

    if (cur.merit + comparison_roundoff(cur.merit) < best_merit) {
      best_merit = cur.merit;
      best_iterate.valid = true;
      best_iterate.x = x;
      best_iterate.s = s;
      best_iterate.lambda = lambda;
      best_iterate.mu = mu;
      best_iterate.objective = current_state.obj_orig;
      best_iterate.residuals = cur;
      best_iterate.iteration = iter + 1;
    }

    if (cur.primal_feas <= opt_.tol_primal &&
        cur.dual_feas <= opt_.tol_dual &&
        cur.complementarity <= comp_tol) {
      converged = true;
      native_status = "Converged";
      break;
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

    const double fraction_to_boundary =
        effective_fraction_to_boundary(opt_.alpha_max);
    const double alpha_aff_pri =
        max_positive_step(s, ds_aff, fraction_to_boundary);
    const double alpha_aff_dual =
        max_positive_step(mu, dmu_aff, fraction_to_boundary);
    const double centering_exponent =
        opt_.theta_mu > 0.0 && std::isfinite(opt_.theta_mu)
            ? opt_.theta_mu
            : 1.0;
    const Eigen::VectorXd target_compl =
        complementarity_target(s, mu, ds_aff, dmu_aff, alpha_aff_pri,
                               alpha_aff_dual, centering_exponent);

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

    double alpha_primal = std::min(
        1.0, max_positive_step(s, ds, fraction_to_boundary));
    double alpha_dual = std::min(
        1.0, max_positive_step(mu, dmu, fraction_to_boundary));
    if (!(alpha_primal > 0.0) || !std::isfinite(alpha_primal) ||
        !(alpha_dual > 0.0) || !std::isfinite(alpha_dual)) {
      native_status = "NativeIPM step length collapsed";
      break;
    }

    bool accepted = false;
    TrialPoint accepted_trial;
    const double relative_direction = std::max(
        {inf_norm(dx) / (1.0 + inf_norm(x)),
         inf_norm(ds) / (1.0 + inf_norm(s)),
         inf_norm(dlambda) / (1.0 + inf_norm(lambda)),
         inf_norm(dmu) / (1.0 + inf_norm(mu))});
    const double minimum_resolvable_alpha =
        std::sqrt(std::numeric_limits<double>::epsilon()) /
        std::max(1.0, relative_direction);
    while (std::min(alpha_primal, alpha_dual) >
           minimum_resolvable_alpha) {
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
          sufficient_primal_dual_progress(cur, trial.residuals);
      if (sufficient_progress) {
        accepted_trial = std::move(trial);
        accepted = true;
        break;
      }

      alpha_primal *= 0.5;
      alpha_dual *= 0.5;
    }

    if (!accepted) {
      native_status = "NativeIPM line search failed";
      break;
    }
    x = accepted_trial.x;
    s = accepted_trial.s;
    lambda = accepted_trial.lambda;
    mu = accepted_trial.mu;
    current_state = std::move(accepted_trial.state);
    current_state_valid = true;
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
