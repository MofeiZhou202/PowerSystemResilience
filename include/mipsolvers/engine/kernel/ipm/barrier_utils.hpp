#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// Infinity norm of a vector (returns 0 for empty vectors).
inline double inf_norm(const Eigen::VectorXd& v) {
  return (v.size() == 0) ? 0.0 : v.cwiseAbs().maxCoeff();
}

/// Fraction-to-boundary rule: returns the largest alpha in (0,1] such that
/// v + alpha*dv >= (1-tau)*v, i.e. the step stays strictly positive.
inline double ftb(const Eigen::VectorXd& v, const Eigen::VectorXd& dv, double tau) {
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

/// Clamp each component of x to the variable's [lb, ub] bounds.
inline void project_to_bounds(const std::vector<VariableMeta>& vars, Eigen::VectorXd& x) {
  for (int i = 0; i < static_cast<int>(x.size()); ++i) {
    x[i] = std::min(vars[i].ub, std::max(vars[i].lb, x[i]));
  }
}

}  // namespace mipsolvers::engine
