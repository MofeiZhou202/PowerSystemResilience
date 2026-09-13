#pragma once

#include <map>
#include <string>
#include <vector>

#include "mipsolvers/engine/solver/adapter_registry.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/solve_context.hpp"

namespace mipsolvers::engine::strategy {

/**
 * @brief Routes problems to solvers based on explicit policy and availability
 *
 * Uses a combination of:
 * - Problem class (LP, QP, NLP, MILP, MINLP, etc.)
 * - Solver preferences (user-specified, per-class policies)
 * - Availability (which solvers are linked)
 * - Fallback strategies (if primary solver fails)
 */
class StrategyDispatcher {
 public:
  void set_solver_preference(ProblemClass cls, const std::string& adapter_name);
  std::vector<std::string> list_solvers(const AdapterRegistry& registry,
                                        ProblemClass cls) const;

  /**
   * @brief Solve a problem with strategy-driven adapter selection and fallback
   */
  SolveResult solve(const AdapterRegistry& registry,
                    const api::ProblemVariant& problem,
                    const std::string& preferred_solver,
                    bool allow_fallback,
                    StrategyPolicy default_policy,
                    const std::map<ProblemClass, StrategyPolicy>& class_policy,
                    const SolveContext& context) const;

 private:
  std::vector<SolverAdapterPtr> candidate_adapters(const AdapterRegistry& registry,
                                                   ProblemClass cls,
                                                   const std::string& preferred_solver,
                                                   StrategyPolicy default_policy,
                                                   const std::map<ProblemClass, StrategyPolicy>& class_policy) const;

  std::map<ProblemClass, std::string> class_preferences_;
};

}  // namespace mipsolvers::engine::strategy
