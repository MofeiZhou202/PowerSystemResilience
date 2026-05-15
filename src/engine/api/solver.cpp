#include "mipsolvers/engine/api/solver.hpp"

#include <memory>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include "mipsolvers/engine/solver/external/adapters.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_solver.hpp"
#include "mipsolvers/engine/kernel/ipm/lcqp_solver.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/engine/solver/native/lp/pdlp_solver.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

api::Result to_api_result(const SolveResult& in) {
  api::Result out;
  out.x = in.x;
  out.constraint_duals = in.constraint_duals;
  out.box_dual_lb = in.box_dual_lb;
  out.box_dual_ub = in.box_dual_ub;

  out.stats.success = in.stats.success;
  out.stats.iterations = in.stats.iterations;
  out.stats.objective = in.stats.objective;
  out.stats.residual_inf = in.stats.residual_inf;
  out.stats.primal_feas = in.stats.primal_feas;
  out.stats.dual_feas = in.stats.dual_feas;
  out.stats.complementarity = in.stats.complementarity;
  out.stats.mip_gap = in.stats.mip_gap;
  out.stats.runtime_sec = in.stats.runtime_sec;
  out.stats.status = in.stats.status;
  out.stats.solver_name = in.stats.solver_name;
  out.stats.cglp_cuts_added = in.stats.cglp_cuts_added;
  out.stats.farkas_ray = in.stats.farkas_ray;
  out.stats.farkas_ray_eq = in.stats.farkas_ray_eq;
  out.stats.has_farkas_certificate = in.stats.has_farkas_certificate;
  return out;
}

// Ensure A and Aeq have the right column count even when no constraints are
// present. An Eigen SparseMatrix default-constructs to 0×0; adapters that
// iterate columns [0..n) will crash if cols() < n. Resize to 0×n in-place.
void normalize_lp_matrices(LPModel& lp) {
  const int n = static_cast<int>(lp.c.size());
  if (n <= 0) return;
  if (lp.A.rows() == 0 && lp.A.cols() != n) {
    lp.A.resize(0, n);
  }
  if (lp.Aeq.rows() == 0 && lp.Aeq.cols() != n) {
    lp.Aeq.resize(0, n);
  }
}

void normalize_qp_matrices(QPModel& qp) {
  const int n = static_cast<int>(qp.c.size());
  if (n <= 0) return;
  if (qp.A.rows() == 0 && qp.A.cols() != n) {
    qp.A.resize(0, n);
  }
  if (qp.Aeq.rows() == 0 && qp.Aeq.cols() != n) {
    qp.Aeq.resize(0, n);
  }
  if (qp.Q.rows() == 0 && qp.Q.cols() != n) {
    qp.Q.resize(n, n);
  }
}

api::ProblemVariant normalize_problem(api::ProblemVariant problem) {
  std::visit(
      [](auto& p) {
        using T = std::decay_t<decltype(p)>;
        if constexpr (std::is_same_v<T, LPModel>) {
          normalize_lp_matrices(p);
        } else if constexpr (std::is_same_v<T, QPModel>) {
          normalize_qp_matrices(p);
        } else if constexpr (std::is_same_v<T, MIPModel>) {
          normalize_lp_matrices(p.linear_part);
        }
      },
      problem);
  return problem;
}

std::string join_validation_errors(const ValidationReport& report) {
  if (report.errors.empty()) {
    return "unknown validation error";
  }

  std::ostringstream oss;
  for (std::size_t i = 0; i < report.errors.size(); ++i) {
    if (i > 0) {
      oss << "; ";
    }
    oss << report.errors[i];
  }
  return oss.str();
}

ValidationReport validate_problem(const api::ProblemVariant& problem) {
  return std::visit(
      [](const auto& p) {
        return validate(p);
      },
      problem);
}

void throw_if_invalid(const api::ProblemVariant& problem) {
  const ValidationReport report = validate_problem(problem);
  if (!report.valid) {
    throw std::invalid_argument(api::problem_class_name(api::problem_class(problem)) +
                                " validation failed: " + join_validation_errors(report));
  }
}

}  // namespace

SolverEngine::SolverEngine(bool register_defaults) {
  if (register_defaults) {
    register_default_adapters();
  }
}

void SolverEngine::register_adapter(const SolverAdapterPtr& adapter) {
  registry_.register_adapter(adapter);
}

std::size_t SolverEngine::register_default_adapters() {
  std::size_t inserted = 0;
  auto register_if_missing = [&](const SolverAdapterPtr& adapter) {
    if (!adapter) {
      return;
    }
    if (registry_.find_by_name(adapter->name())) {
      return;
    }
    registry_.register_adapter(adapter);
    ++inserted;
  };

  register_if_missing(std::make_shared<NativeLinearAdapter>());
  register_if_missing(std::make_shared<NativeNewtonAdapter>());
  register_if_missing(std::make_shared<NativeIPMLPAdapter>());
  register_if_missing(std::make_shared<NativePDLPAdapter>());
  register_if_missing(std::make_shared<NativeLCQPAdapter>());
  register_if_missing(std::make_shared<NativeIPMAdapter>());
  register_if_missing(std::make_shared<NativeNLPAdapter>());
  register_if_missing(std::make_shared<StrictHighsBranchAndCutAdapter>());
  register_if_missing(std::make_shared<NativeBranchAndCutAdapter>());

  auto gurobi = std::make_shared<GurobiAdapter>();
  if (gurobi->available()) {
    register_if_missing(gurobi);
  }

  auto highs = std::make_shared<HighsAdapter>();
  if (highs->available()) {
    register_if_missing(highs);
  }

  auto ipopt = std::make_shared<IpoptAdapter>();
  if (ipopt->available()) {
    register_if_missing(ipopt);
  }

  auto scip = std::make_shared<ScipAdapter>();
  if (scip->available()) {
    register_if_missing(scip);
  }

  return inserted;
}

void SolverEngine::set_solver_preference(ProblemClass cls, const std::string& adapter_name) {
  dispatcher_.set_solver_preference(cls, adapter_name);
}

std::vector<std::string> SolverEngine::list_solvers(ProblemClass cls) const {
  return dispatcher_.list_solvers(registry_, cls);
}

api::Result SolverEngine::solve(const api::ProblemVariant& problem,
                                const SolveOptions& options) const {
  const api::ProblemVariant normalized = normalize_problem(problem);
  throw_if_invalid(normalized);
  const SolveResult internal = dispatcher_.solve(
      registry_,
      normalized,
      options.preferred_solver,
      options.allow_fallback,
      options.strategy_policy,
      options.class_strategy_policy);
  return to_api_result(internal);
}

api::Result SolverEngine::solve_le(const SparseLinSys& problem,
                                   const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_nle(const NonlinearSystem& problem,
                                    const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_lp(const LPModel& problem,
                                   const SolveOptions& options) const {
  LPModel normalized = problem;
  normalize_lp_matrices(normalized);
  return solve(api::ProblemVariant{normalized}, options);
}

api::Result SolverEngine::solve_qp(const QPModel& problem,
                                   const SolveOptions& options) const {
  QPModel normalized = problem;
  normalize_qp_matrices(normalized);
  return solve(api::ProblemVariant{normalized}, options);
}

api::Result SolverEngine::solve_nlp(const NLPModel& problem,
                                    const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_milp(const MIPModel& problem,
                                     const SolveOptions& options) const {
  MIPModel normalized = problem;
  normalize_lp_matrices(normalized.linear_part);
  return solve(api::ProblemVariant{normalized}, options);
}

api::Result SolverEngine::solve_minlp(const MINLPModel& problem,
                                      const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

}  // namespace mipsolvers::engine
