#pragma once

#include <optional>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/api/options.hpp"

namespace mipsolvers::engine::strategy {

/**
 * @brief Metadata for translating between presolved and original problem space
 *
 * Tracks all transformations applied during presolve so that solutions
 * can be mapped back to the original problem space.
 */
struct PresolveMapping {
  // Column (variable) mapping: presolved_idx → original_idx
  std::vector<int> col_mapping;

  // Row (constraint) mapping: presolved_idx → original_idx
  std::vector<int> row_mapping;

  // Fixed variables: variable_idx → fixed_value
  std::map<int, double> fixed_variables;

  // Variable shifts: variable_idx → shift_amount (new_x = old_x - shift)
  std::map<int, double> variable_shifts;

  // Constraint scaling: constraint_idx → scale_factor
  std::vector<double> row_scales;

  // Variable scaling: variable_idx → scale_factor
  std::vector<double> col_scales;

  // Whether any transformations were applied
  bool was_modified() const {
    return !col_mapping.empty() || !row_mapping.empty() || !fixed_variables.empty();
  }
};

/**
 * @brief Result of presolve: presolved problem + mapping for postsolve
 */
struct PresolvedProblem {
  api::ProblemVariant presolved;
  PresolveMapping mapping;

  bool is_presolved() const { return mapping.was_modified(); }
};

/**
 * @brief Manages presolve transformations
 *
 * Applies multiple presolve techniques in sequence:
 * 1. Native presolve (empty row/column detection, fixed variable elimination)
 * 2. PaPILO presolve (if available)
 * 3. Scaling (Ruiz equilibration for numerical stability)
 *
 * Produces a smaller, better-conditioned problem that solves faster.
 */
class PresolveManager {
 public:
  /**
   * @brief Apply presolve transformations to a problem
   *
   * @param problem Input problem
   * @param options Solver options (enable/disable presolve, verbosity)
   * @return Presolved problem + mapping for postsolve
   */
  PresolvedProblem presolve(const api::ProblemVariant& problem,
                            const SolveOptions& options) const;

  /**
   * @brief Apply native presolve only (without PaPILO)
   */
  PresolvedProblem native_presolve(const api::ProblemVariant& problem) const;

  /**
   * @brief Apply scaling to improve numerical conditioning
   */
  PresolvedProblem scale(const api::ProblemVariant& problem,
                         const std::string& scaling_method = "ruiz") const;

  /**
   * @brief Check if presolve would likely help
   *
   * Returns true if problem is large or has poor numerical properties
   */
  bool should_presolve(const api::ProblemVariant& problem,
                       const SolveOptions& options) const;

  /**
   * @brief Get presolve statistics (rows removed, columns removed, etc)
   */
  struct Stats {
    int original_rows{0};
    int original_cols{0};
    int presolved_rows{0};
    int presolved_cols{0};
    int empty_rows_removed{0};
    int empty_cols_removed{0};
    int fixed_vars_eliminated{0};
    double scaling_time_sec{0};
  };

  Stats last_stats() const { return last_stats_; }

 private:
  mutable Stats last_stats_;
};

}  // namespace mipsolvers::engine::strategy
