#pragma once

#include <optional>
#include <Eigen/Core>

#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/strategy/presolve_manager.hpp"

namespace mipsolvers::engine::strategy {

/**
 * @brief Maps solutions from presolved space back to original problem space
 *
 * Reverses an explicit `PresolveMapping`. Invalid, incomplete, or
 * dimensionally inconsistent mappings raise `std::invalid_argument` instead
 * of returning a partially reconstructed solution.
 */
class PostsolveManager {
 public:
  /**
   * @brief Map solution from presolved to original problem space
   *
   * @param presolved_result Result from solving the presolved problem
   * @param mapping The presolve mapping (from PresolveManager)
   * @return Result in original problem space
   */
  SolveResult postsolve(const SolveResult& presolved_result,
                        const PresolveMapping& mapping) const;

  /**
   * @brief Restore fixed variable values in solution
   */
  void restore_fixed_variables(Eigen::VectorXd& x,
                               const PresolveMapping& mapping) const;

  /**
   * @brief Undo variable shifts
   */
  void undo_variable_shifts(Eigen::VectorXd& x,
                            const PresolveMapping& mapping) const;

  /**
   * @brief Unscale solution back to original scaling
   */
  void unscale(Eigen::VectorXd& x, Eigen::VectorXd& duals,
               const PresolveMapping& mapping) const;

  /**
   * @brief Validate result-level postsolve invariants
   *
   * This checks success state, vector dimensions/finiteness, and objective
   * agreement. Constraint feasibility requires the original model and is
   * therefore audited by the solver-specific postsolve path.
   */
  bool validate(const SolveResult& original_result,
                const SolveResult& presolved_result,
                double feasibility_tol = 1e-6) const;
};

}  // namespace mipsolvers::engine::strategy
