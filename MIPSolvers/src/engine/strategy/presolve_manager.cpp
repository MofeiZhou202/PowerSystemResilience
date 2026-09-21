#include "mipsolvers/engine/strategy/presolve_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace mipsolvers::engine::strategy {

bool PresolveManager::should_presolve(const api::ProblemVariant&,
                                      const SolveOptions&) const {
  return false;
}

PresolvedProblem PresolveManager::presolve(const api::ProblemVariant& problem,
                                           const SolveOptions&) const {
  return native_presolve(problem);
}

PresolvedProblem PresolveManager::native_presolve(const api::ProblemVariant& problem) const {
  PresolvedProblem result;
  result.presolved = problem;
  last_stats_ = Stats{};

  std::visit(
      [&](const auto& model) {
        using T = std::decay_t<decltype(model)>;
        int rows = 0;
        int cols = 0;
        if constexpr (std::is_same_v<T, SparseLinSys>) {
          rows = static_cast<int>(model.A.rows());
          cols = static_cast<int>(model.A.cols());
        } else if constexpr (std::is_same_v<T, NonlinearSystem>) {
          cols = static_cast<int>(model.x0.size());
        } else if constexpr (std::is_same_v<T, LPModel>) {
          rows = static_cast<int>(model.A.rows() + model.Aeq.rows());
          cols = static_cast<int>(model.c.size());
        } else if constexpr (std::is_same_v<T, QPModel>) {
          rows = static_cast<int>(model.A.rows() + model.Aeq.rows());
          cols = static_cast<int>(model.c.size());
        } else if constexpr (std::is_same_v<T, NLPModel>) {
          cols = static_cast<int>(model.vars.size());
        } else if constexpr (std::is_same_v<T, MIPModel>) {
          rows = static_cast<int>(model.linear_part.A.rows() +
                                  model.linear_part.Aeq.rows());
          cols = static_cast<int>(model.linear_part.c.size());
        } else if constexpr (std::is_same_v<T, MINLPModel>) {
          cols = static_cast<int>(model.nonlinear_part.vars.size());
        } else if constexpr (std::is_same_v<T, ConicModel>) {
          rows = static_cast<int>(model.G.rows() + model.A.rows());
          cols = static_cast<int>(model.c.size());
        }
        last_stats_.original_rows = rows;
        last_stats_.presolved_rows = rows;
        last_stats_.original_cols = cols;
        last_stats_.presolved_cols = cols;
      },
      problem);

  return result;
}

PresolvedProblem PresolveManager::scale(const api::ProblemVariant& problem,
                                        const std::string& scaling_method) const {
  if (scaling_method == "none" || scaling_method == "identity") {
    return native_presolve(problem);
  }
  throw std::logic_error(
      "PresolveManager has no generic scaling backend; use a solver kernel "
      "with explicit scaling support");
}

}  // namespace mipsolvers::engine::strategy
