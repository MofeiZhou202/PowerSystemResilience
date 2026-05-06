#pragma once

#include <optional>
#include <Eigen/Core>

#include "hacdcpf/engine/api/result.hpp"
#include "hacdcpf/engine/strategy/presolve_manager.hpp"

namespace hacdcpf::engine::strategy {

/**
 * @brief Maps solutions from presolved space back to original problem space
 *
 * Reverses the transformations applied by PresolveManager to produce
 * a solution for the original problem.
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
   * @brief Validate that postsolve maintains feasibility
   */
  bool validate(const SolveResult& original_result,
                const SolveResult& presolved_result,
                double feasibility_tol = 1e-6) const;
};

}  // namespace hacdcpf::engine::strategy
