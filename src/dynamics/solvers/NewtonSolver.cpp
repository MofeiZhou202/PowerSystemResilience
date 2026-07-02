#include "hacdcpf/dynamics/solvers/NewtonSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

namespace hacdcpf::dynamics {

namespace {

bool numerical_jacobian(const NewtonSolver::ResidualFunction& residual,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& r0,
                        Eigen::MatrixXd& jac,
                        std::string& error) {
  const Eigen::Index n = x.size();
  jac = Eigen::MatrixXd::Zero(n, n);
  const double eps0 = std::sqrt(std::numeric_limits<double>::epsilon());
  for (Eigen::Index col = 0; col < n; ++col) {
    Eigen::VectorXd trial = x;
    const double h = eps0 * std::max(1.0, std::abs(x[col]));
    trial[col] += h;
    Eigen::VectorXd rp;
    if (!residual(trial, rp, error)) return false;
    jac.col(col) = (rp - r0) / h;
  }
  return true;
}

}  // namespace

NewtonSolverResult NewtonSolver::solve(ResidualFunction residual,
                                       Eigen::VectorXd& x) const {
  std::string error;
  Eigen::VectorXd r;
  if (!residual(x, r, error)) return {false, 0, 0.0, error};
  double norm = r.size() > 0 ? r.lpNorm<Eigen::Infinity>() : 0.0;
  if (norm <= options_.tolerance) return {true, 0, norm, {}};

  for (int iter = 0; iter < options_.max_iterations; ++iter) {
    Eigen::MatrixXd jac;
    if (!numerical_jacobian(residual, x, r, jac, error)) {
      return {false, iter, norm, error};
    }
    const Eigen::VectorXd delta = jac.colPivHouseholderQr().solve(-r);
    if (!delta.allFinite()) return {false, iter, norm, "Newton correction is non-finite"};

    double alpha = 1.0;
    bool accepted = false;
    const double min_alpha = std::max(1e-9, options_.min_damping);
    while (alpha >= min_alpha) {
      Eigen::VectorXd trial = x + alpha * delta;
      Eigen::VectorXd rt;
      if (!residual(trial, rt, error)) return {false, iter + 1, norm, error};
      const double nt = rt.size() > 0 ? rt.lpNorm<Eigen::Infinity>() : 0.0;
      if (nt <= (1.0 - 1e-4 * alpha) * std::max(norm, options_.tolerance)) {
        x = trial;
        r = rt;
        norm = nt;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) return {false, iter + 1, norm, "Newton line search failed"};
    if (norm <= options_.tolerance) return {true, iter + 1, norm, {}};
  }
  return {false, options_.max_iterations, norm, "Newton did not converge"};
}

}  // namespace hacdcpf::dynamics
