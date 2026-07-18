#pragma once

#include <map>
#include <string>
#include <vector>

#include "mipsolvers/engine/solver/adapter_registry.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/api/result.hpp"
#include "mipsolvers/engine/strategy/dispatcher.hpp"

namespace mipsolvers::engine {

class SolverEngine {
 public:
  explicit SolverEngine(bool register_defaults = true);

  void register_adapter(const SolverAdapterPtr& adapter);
  std::size_t register_default_adapters();

  void set_solver_preference(ProblemClass cls, const std::string& adapter_name);
  std::vector<std::string> list_solvers(ProblemClass cls) const;

  api::Result solve(api::ProblemVariant problem,
                    const SolveOptions& options = {}) const;

  api::Result solve_le(const SparseLinSys& problem, const SolveOptions& options = {}) const;
  api::Result solve_nle(const NonlinearSystem& problem, const SolveOptions& options = {}) const;
  api::Result solve_lp(const LPModel& problem, const SolveOptions& options = {}) const;
  api::Result solve_qp(const QPModel& problem, const SolveOptions& options = {}) const;
  api::Result solve_nlp(const NLPModel& problem, const SolveOptions& options = {}) const;
  api::Result solve_milp(const MIPModel& problem, const SolveOptions& options = {}) const;
  api::Result solve_minlp(const MINLPModel& problem, const SolveOptions& options = {}) const;

 private:
  AdapterRegistry registry_;
  strategy::StrategyDispatcher dispatcher_;
};

namespace api {
using SolverEngine = ::mipsolvers::engine::SolverEngine;
}  // namespace api

}  // namespace mipsolvers::engine
