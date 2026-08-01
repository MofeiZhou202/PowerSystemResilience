#include "mipsolvers/engine/strategy/postsolve_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace mipsolvers::engine::strategy {

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

void PostsolveManager::restore_fixed_variables(Eigen::VectorXd& x,
                                               const PresolveMapping& mapping) const {
  if (mapping.fixed_variables.empty()) {
    return;
  }
  if (mapping.col_mapping.size() != static_cast<std::size_t>(x.size())) {
    throw std::invalid_argument(
        "Postsolve fixed-variable restoration requires one original-column "
        "index for every reduced column");
  }

  int original_size = 0;
  for (int original : mapping.col_mapping) {
    if (original < 0) {
      throw std::invalid_argument("Postsolve column mapping contains a negative index");
    }
    original_size = std::max(original_size, original + 1);
  }
  for (const auto& [original, value] : mapping.fixed_variables) {
    if (original < 0 || !std::isfinite(value)) {
      throw std::invalid_argument("Postsolve fixed-variable record is invalid");
    }
    original_size = std::max(original_size, original + 1);
  }

  Eigen::VectorXd restored = Eigen::VectorXd::Constant(
      original_size, std::numeric_limits<double>::quiet_NaN());
  for (int reduced = 0; reduced < x.size(); ++reduced) {
    const int original = mapping.col_mapping[static_cast<std::size_t>(reduced)];
    if (std::isfinite(restored[original])) {
      throw std::invalid_argument("Postsolve column mapping contains duplicates");
    }
    restored[original] = x[reduced];
  }
  for (const auto& [original, value] : mapping.fixed_variables) {
    if (std::isfinite(restored[original])) {
      throw std::invalid_argument(
          "Postsolve fixed variable overlaps an active reduced column");
    }
    restored[original] = value;
  }
  if (!restored.allFinite()) {
    throw std::invalid_argument(
        "Postsolve mapping does not reconstruct every original column");
  }
  x = std::move(restored);
}

void PostsolveManager::undo_variable_shifts(Eigen::VectorXd& x,
                                            const PresolveMapping& mapping) const {
  // Reverse variable shifts: x_orig = x_reduced + shift
  for (const auto& [var_idx, shift] : mapping.variable_shifts) {
    if (var_idx < 0 || var_idx >= static_cast<int>(x.size()) ||
        !std::isfinite(shift)) {
      throw std::invalid_argument("Postsolve variable shift is invalid");
    }
    x(var_idx) += shift;
  }
}

void PostsolveManager::unscale(Eigen::VectorXd& x, Eigen::VectorXd& duals,
                               const PresolveMapping& mapping) const {
  // Unscale variables: x_orig = x_scaled * col_scale
  if (!mapping.col_scales.empty()) {
    if (mapping.col_scales.size() != static_cast<std::size_t>(x.size())) {
      throw std::invalid_argument("Postsolve column scaling dimension mismatch");
    }
    for (int i = 0; i < x.size(); ++i) {
      const double scale = mapping.col_scales[static_cast<std::size_t>(i)];
      if (!(std::isfinite(scale) && scale > 0.0)) {
        throw std::invalid_argument("Postsolve column scale must be finite and positive");
      }
      x(i) *= scale;
    }
  }

  // Unscale duals: y_orig = y_scaled / row_scale
  if (!mapping.row_scales.empty()) {
    if (mapping.row_scales.size() != static_cast<std::size_t>(duals.size())) {
      throw std::invalid_argument("Postsolve row scaling dimension mismatch");
    }
    for (int i = 0; i < duals.size(); ++i) {
      const double scale = mapping.row_scales[static_cast<std::size_t>(i)];
      if (!(std::isfinite(scale) && scale > 0.0)) {
        throw std::invalid_argument("Postsolve row scale must be finite and positive");
      }
      duals(i) /= scale;
    }
  }
}

bool PostsolveManager::validate(const SolveResult& original_result,
                                const SolveResult& presolved_result,
                                double feasibility_tol) const {
  if (original_result.stats.success != presolved_result.stats.success) {
    return false;
  }
  if (!original_result.stats.success) return true;
  if (original_result.x.size() != presolved_result.x.size() ||
      !original_result.x.allFinite() || !presolved_result.x.allFinite()) {
    return false;
  }
  if (!std::isfinite(original_result.stats.objective) ||
      !std::isfinite(presolved_result.stats.objective)) {
    return false;
  }
  const double scale = std::max(
      {1.0, std::abs(original_result.stats.objective),
       std::abs(presolved_result.stats.objective)});
  return std::abs(original_result.stats.objective -
                  presolved_result.stats.objective) <=
         feasibility_tol * scale;
}

}  // namespace mipsolvers::engine::strategy
