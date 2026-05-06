#pragma once

#include <map>
#include <string>
#include <vector>

#include "hacdcpf/engine/solver/adapter_registry.hpp"
#include "hacdcpf/engine/api/options.hpp"
#include "hacdcpf/engine/api/problem.hpp"

namespace hacdcpf::engine::strategy {

/**
 * @brief Problem characteristics for heuristic-based solver selection
 */
struct ProblemCharacteristics {
  int num_vars{0};
  int num_constraints{0};
  int num_nonzeros{0};
  double density{0.0};  // nonzeros / (num_vars * num_constraints)
  bool is_sparse{false};
  bool is_large{false};
  bool is_dense{false};
  int condition_estimate{0};  // 0=unknown, 1=well-conditioned, 2=ill-conditioned
};

/**
 * @brief Routes problems to solvers based on problem class and characteristics
 *
 * Uses a combination of:
 * - Problem class (LP, QP, NLP, MILP, MINLP, etc.)
 * - Solver preferences (user-specified, per-class policies)
 * - Problem characteristics (size, sparsity, structure)
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
                    const std::map<ProblemClass, StrategyPolicy>& class_policy) const;

  /**
   * @brief Extract problem characteristics for heuristic-based selection
   */
  ProblemCharacteristics analyze(const api::ProblemVariant& problem) const;

  /**
   * @brief Estimate which solver will likely perform best based on problem characteristics
   */
  std::string estimate_best_solver(ProblemClass cls,
                                   const ProblemCharacteristics& characteristics,
                                   const AdapterRegistry& registry) const;

 private:
  std::vector<SolverAdapterPtr> candidate_adapters(const AdapterRegistry& registry,
                                                   ProblemClass cls,
                                                   const std::string& preferred_solver,
                                                   StrategyPolicy default_policy,
                                                   const std::map<ProblemClass, StrategyPolicy>& class_policy) const;

  std::map<ProblemClass, std::string> class_preferences_;
};

}  // namespace hacdcpf::engine::strategy
