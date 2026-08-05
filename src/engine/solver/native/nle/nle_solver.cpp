/// @file nle_solver.cpp
/// @brief General-purpose Newton solver for nonlinear equations F(x) = 0.
///
/// Solves systems of the form F: R^n → R^n using damped Newton iterations
/// with a dogleg trust-region globalization strategy.  The caller supplies
/// F and the Jacobian J via std::function callbacks, keeping the solver
/// completely domain-agnostic.
///
/// Algorithm:
///   1. Evaluate F(x) and J(x).
///   2. Solve J·dx = -F for the Newton step dx.
///   3. Accept/reject via dogleg trust-region update.
///   4. Update x and repeat until ‖F(x)‖ < tol or max_iters reached.

#include "mipsolvers/engine/solver/native/nle/nle_solver.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "mipsolvers/engine/kernel/globalization/globalization.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/linear_solver.hpp"

namespace mipsolvers::engine {

namespace {

constexpr double kDefaultTol     = 1e-8;
constexpr double kDefaultDelta0  = 1.0;   // initial trust-region radius
constexpr double kDeltaMax       = 1e4;
constexpr double kEta            = 0.1;   // sufficient-decrease threshold
constexpr int    kDefaultMaxIter = 200;

}  // namespace

// AUDIT-NAV: 领域无关 Dogleg 信赖域 NLE 主循环；与 native_adapters.cpp 中的
// 正则化回溯 Newton 以及电力潮流 NewtonSolver 是三条独立路径。
NLEResult NLESolver::solve(const NLEProblem& prob, const NLEOptions& opt) const {
  NLEResult result;
  result.x = prob.x0;

  const int n = static_cast<int>(result.x.size());
  if (n == 0) {
    result.status = NLEStatus::Success;
    return result;
  }

  const double tol     = opt.tol > 0.0 ? opt.tol : kDefaultTol;
  const int max_iters  = opt.max_iters > 0 ? opt.max_iters : kDefaultMaxIter;
  double delta         = kDefaultDelta0;

  auto linear = make_default_sparse_solver();

  for (int iter = 0; iter < max_iters; ++iter) {
    // 1. Evaluate residual and Jacobian.
    const Eigen::VectorXd F = prob.residual(result.x);
    const double norm_F = F.norm();

    result.residual_norm = norm_F;
    result.iterations    = iter;

    if (norm_F <= tol) {
      result.status = NLEStatus::Success;
      return result;
    }

    const Eigen::SparseMatrix<double> J = prob.jacobian(result.x);

    // 2. Solve J·dx = -F  (Newton step).
    linear->analyze_pattern(J);
    if (!linear->factorize(J)) {
      result.status = NLEStatus::SingularJacobian;
      return result;
    }

    Eigen::VectorXd dx_newton;
    if (!linear->solve(-F, dx_newton) || !dx_newton.allFinite()) {
      result.status = NLEStatus::LinearSolveFailed;
      return result;
    }

    // 3. Dogleg trust-region step.
    const Eigen::VectorXd gradient = J.transpose() * F;         // Jᵀ·F
    const Eigen::VectorXd Jg       = J * gradient;               // J·(Jᵀ·F)
    const Eigen::VectorXd dx       = dogleg_step(dx_newton, gradient, Jg, delta);

    // 4. Evaluate gain ratio ρ = (‖F‖ - ‖F(x+dx)‖) / predicted_reduction.
    const Eigen::VectorXd x_new = result.x + dx;
    const Eigen::VectorXd F_new = prob.residual(x_new);
    const double norm_F_new = F_new.norm();

    const double actual_reduction    = norm_F - norm_F_new;
    const Eigen::VectorXd F_lin      = F + J * dx;
    const double predicted_reduction = norm_F - F_lin.norm();

    const double rho = (std::abs(predicted_reduction) < 1e-14)
                           ? 0.0
                           : actual_reduction / predicted_reduction;

    // 5. Update trust-region radius.
    if (rho > 0.75) {
      delta = std::min(kDeltaMax, 2.0 * delta);
    } else if (rho < 0.25) {
      delta *= 0.25;
    }

    // 6. Accept step if sufficient decrease.
    if (rho > kEta) {
      result.x = x_new;
    }

    if (delta < 1e-14) {
      result.status = NLEStatus::TrustRegionTooSmall;
      return result;
    }
  }

  result.status = NLEStatus::MaxIterationsReached;
  return result;
}

}  // namespace mipsolvers::engine
