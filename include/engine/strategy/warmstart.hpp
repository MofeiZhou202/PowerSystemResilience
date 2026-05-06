#pragma once

#include <optional>
#include <map>
#include <Eigen/Core>
#include <Eigen/Sparse>

namespace hacdcpf::engine {
// Forward declaration from solver_adapter.hpp
struct SolveResult;
}

namespace hacdcpf::engine::strategy {

/**
 * @brief Warm-start hint container for solver continuation
 *
 * Carries primal/dual hints and basis information to warm-start a solver
 * from the solution of a previous problem. Supports:
 * - Primal point hints (x)
 * - Dual multiplier hints (constraint and bound duals)
 * - Objective function value hint
 */
struct WarmStart {
  // Primal solution hint
  std::optional<Eigen::VectorXd> primal;

  // Dual multiplier hints
  std::optional<Eigen::VectorXd> constraint_duals;
  std::optional<Eigen::VectorXd> bound_duals_lower;
  std::optional<Eigen::VectorXd> bound_duals_upper;

  // Problem dimensions (for compatibility checks)
  int num_vars{0};
  int num_constraints{0};

  // MILP-specific: whether to reuse root node factorization
  bool reuse_root_factorization{false};

  /**
   * @brief Check if this warm-start is non-empty
   */
  bool has_hint() const {
    return primal.has_value() || constraint_duals.has_value() ||
           bound_duals_lower.has_value() || bound_duals_upper.has_value();
  }

  /**
   * @brief Check if dimensions match a problem
   */
  bool is_compatible(int n_vars, int n_constraints) const {
    if (num_vars > 0 && num_vars != n_vars) return false;
    if (num_constraints > 0 && num_constraints != n_constraints) return false;
    return true;
  }
};

/**
 * @brief Manages warm-start hints and solution extraction
 *
 * Provides utilities for:
 * - Extracting warm-start from solve results
 * - Merging multiple warm-starts
 * - Validating compatibility
 */
class WarmStartManager {
 public:
  /**
   * @brief Check if a warm-start is compatible with a problem
   */
  bool is_compatible(int num_vars, int num_constraints, const WarmStart& ws) const;

  /**
   * @brief Extract warm-start from a previous solve result
   */
  WarmStart extract_from_result(int num_vars, int num_constraints,
                                const SolveResult& result) const;

  /**
   * @brief Merge multiple warm-starts (taking first available for each component)
   */
  WarmStart merge(const std::vector<WarmStart>& candidates) const;

  /**
   * @brief Scale warm-start for problem scaling changes
   */
  WarmStart scale(const WarmStart& ws, const Eigen::VectorXd& x_scale,
                  const Eigen::VectorXd& c_scale) const;

  /**
   * @brief Validate warm-start feasibility (basic checks)
   */
  bool validate(const WarmStart& ws) const;
};

}  // namespace hacdcpf::engine::strategy
