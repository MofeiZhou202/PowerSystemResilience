#include "hacdcpf/engine/strategy/postsolve_manager.hpp"

#include <cmath>

namespace hacdcpf::engine::strategy {

SolveResult PostsolveManager::postsolve(const SolveResult& presolved_result,
                                        const PresolveMapping& mapping) const {
  SolveResult result = presolved_result;

  // Restore fixed variables
  restore_fixed_variables(result.x, mapping);

  // Undo variable shifts
  undo_variable_shifts(result.x, mapping);

  // Unscale
  if (!mapping.row_scales.empty() || !mapping.col_scales.empty()) {
    unscale(result.x, result.constraint_duals, mapping);
  }

  return result;
}

void PostsolveManager::restore_fixed_variables(Eigen::VectorXd& /* x */,
                                               const PresolveMapping& mapping) const {
  // Expand x vector to include fixed variables
  if (mapping.fixed_variables.empty()) {
    return;
  }

  // If we have a column mapping, we need to expand the solution
  // For now, this is a placeholder
}

void PostsolveManager::undo_variable_shifts(Eigen::VectorXd& x,
                                            const PresolveMapping& mapping) const {
  // Reverse variable shifts: x_orig = x_reduced + shift
  for (const auto& [var_idx, shift] : mapping.variable_shifts) {
    if (var_idx >= 0 && var_idx < static_cast<int>(x.size())) {
      x(var_idx) += shift;
    }
  }
}

void PostsolveManager::unscale(Eigen::VectorXd& x, Eigen::VectorXd& duals,
                               const PresolveMapping& mapping) const {
  // Unscale variables: x_orig = x_scaled * col_scale
  if (!mapping.col_scales.empty()) {
    for (int i = 0; i < std::min(static_cast<int>(x.size()),
                                  static_cast<int>(mapping.col_scales.size()));
         ++i) {
      if (mapping.col_scales[i] > 0) {
        x(i) *= mapping.col_scales[i];
      }
    }
  }

  // Unscale duals: y_orig = y_scaled / row_scale
  if (!mapping.row_scales.empty()) {
    for (int i = 0; i < std::min(static_cast<int>(duals.size()),
                                  static_cast<int>(mapping.row_scales.size()));
         ++i) {
      if (mapping.row_scales[i] > 0) {
        duals(i) /= mapping.row_scales[i];
      }
    }
  }
}

bool PostsolveManager::validate(const SolveResult& /* original_result */,
                                const SolveResult& presolved_result,
                                double /* feasibility_tol */) const {
  // Check that postsolve maintained solution quality
  if (presolved_result.stats.success) {
    // For now, just return true if presolved was successful
    // In Phase 3b, add actual feasibility checks
    return true;
  }

  return false;
}

}  // namespace hacdcpf::engine::strategy
