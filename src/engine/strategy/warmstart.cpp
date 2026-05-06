#include "hacdcpf/engine/strategy/warmstart.hpp"

#include <algorithm>
#include "hacdcpf/engine/solver/solver_adapter.hpp"

namespace hacdcpf::engine::strategy {

bool WarmStartManager::is_compatible(int num_vars, int num_constraints,
                                     const WarmStart& ws) const {
  if (ws.primal.has_value()) {
    if (static_cast<int>(ws.primal->size()) != num_vars) {
      return false;
    }
  }

  if (ws.constraint_duals.has_value()) {
    if (static_cast<int>(ws.constraint_duals->size()) != num_constraints) {
      return false;
    }
  }

  if (ws.bound_duals_lower.has_value()) {
    if (static_cast<int>(ws.bound_duals_lower->size()) != num_vars) {
      return false;
    }
  }

  if (ws.bound_duals_upper.has_value()) {
    if (static_cast<int>(ws.bound_duals_upper->size()) != num_vars) {
      return false;
    }
  }

  return ws.is_compatible(num_vars, num_constraints);
}

WarmStart WarmStartManager::extract_from_result(int num_vars, int num_constraints,
                                                const SolveResult& result) const {
  WarmStart ws;
  ws.num_vars = num_vars;
  ws.num_constraints = num_constraints;

  if (result.x.size() == num_vars) {
    ws.primal = result.x;
  }

  if (result.constraint_duals.size() == num_constraints) {
    ws.constraint_duals = result.constraint_duals;
  }

  if (result.box_dual_lb.size() == num_vars) {
    ws.bound_duals_lower = result.box_dual_lb;
  }

  if (result.box_dual_ub.size() == num_vars) {
    ws.bound_duals_upper = result.box_dual_ub;
  }

  return ws;
}

WarmStart WarmStartManager::merge(const std::vector<WarmStart>& candidates) const {
  WarmStart merged;

  for (const auto& ws : candidates) {
    if (!merged.primal.has_value() && ws.primal.has_value()) {
      merged.primal = ws.primal;
      if (ws.num_vars > 0) {
        merged.num_vars = ws.num_vars;
      }
    }

    if (!merged.constraint_duals.has_value() && ws.constraint_duals.has_value()) {
      merged.constraint_duals = ws.constraint_duals;
      if (ws.num_constraints > 0) {
        merged.num_constraints = ws.num_constraints;
      }
    }

    if (!merged.bound_duals_lower.has_value() && ws.bound_duals_lower.has_value()) {
      merged.bound_duals_lower = ws.bound_duals_lower;
    }

    if (!merged.bound_duals_upper.has_value() && ws.bound_duals_upper.has_value()) {
      merged.bound_duals_upper = ws.bound_duals_upper;
    }

    if (!merged.reuse_root_factorization && ws.reuse_root_factorization) {
      merged.reuse_root_factorization = true;
    }
  }

  return merged;
}

WarmStart WarmStartManager::scale(const WarmStart& ws,
                                  const Eigen::VectorXd& x_scale,
                                  const Eigen::VectorXd& c_scale) const {
  WarmStart scaled = ws;

  // Scale primal: x_new = x_old / x_scale
  if (scaled.primal.has_value()) {
    scaled.primal = scaled.primal.value().cwiseQuotient(x_scale);
  }

  // Scale duals: y_new = y_old * c_scale
  if (scaled.constraint_duals.has_value()) {
    scaled.constraint_duals = scaled.constraint_duals.value().cwiseProduct(c_scale);
  }

  // Bound duals scale with variable scaling
  if (scaled.bound_duals_lower.has_value()) {
    scaled.bound_duals_lower = scaled.bound_duals_lower.value().cwiseProduct(x_scale);
  }

  if (scaled.bound_duals_upper.has_value()) {
    scaled.bound_duals_upper = scaled.bound_duals_upper.value().cwiseProduct(x_scale);
  }

  return scaled;
}

bool WarmStartManager::validate(const WarmStart& ws) const {
  if (!ws.has_hint()) {
    return true;
  }

  // Basic checks: dimensions are consistent
  if (ws.primal.has_value() && ws.constraint_duals.has_value()) {
    if (ws.num_vars > 0 && ws.num_constraints > 0) {
      if (static_cast<int>(ws.primal->size()) != ws.num_vars ||
          static_cast<int>(ws.constraint_duals->size()) != ws.num_constraints) {
        return false;
      }
    }
  }

  // Check for NaN/Inf
  if (ws.primal.has_value()) {
    if (!ws.primal->allFinite()) {
      return false;
    }
  }

  if (ws.constraint_duals.has_value()) {
    if (!ws.constraint_duals->allFinite()) {
      return false;
    }
  }

  return true;
}

}  // namespace hacdcpf::engine::strategy
