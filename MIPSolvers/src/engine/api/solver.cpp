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
#include "mipsolvers/engine/solver/native/native_lp_selector.hpp"
#include "mipsolvers/engine/solver/native/lp/pdlp_solver.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

api::Result to_api_result(SolveResult in) {
  api::Result out;
  out.x = std::move(in.x);
  out.constraint_duals = std::move(in.constraint_duals);
  out.box_dual_lb = std::move(in.box_dual_lb);
  out.box_dual_ub = std::move(in.box_dual_ub);

  out.stats.success = in.stats.success;
  out.stats.strict_convergence = in.stats.strict_convergence;
  out.stats.acceptable_convergence = in.stats.acceptable_convergence;
  out.stats.iterations = in.stats.iterations;
  out.stats.objective = in.stats.objective;
  out.stats.residual_inf = in.stats.residual_inf;
  out.stats.primal_feas = in.stats.primal_feas;
  out.stats.dual_feas = in.stats.dual_feas;
  out.stats.complementarity = in.stats.complementarity;
  out.stats.barrier_parameter = in.stats.barrier_parameter;
  out.stats.unscaled_primal_feas = in.stats.unscaled_primal_feas;
  out.stats.unscaled_dual_feas = in.stats.unscaled_dual_feas;
  out.stats.unscaled_complementarity = in.stats.unscaled_complementarity;
  out.stats.initial_primal_feas = in.stats.initial_primal_feas;
  out.stats.warm_start_used = in.stats.warm_start_used;
  out.stats.relative_primal_residual = in.stats.relative_primal_residual;
  out.stats.relative_dual_residual = in.stats.relative_dual_residual;
  out.stats.relative_gap = in.stats.relative_gap;
  out.stats.dual_objective = in.stats.dual_objective;
  out.stats.mip_gap = in.stats.mip_gap;
  out.stats.runtime_sec = in.stats.runtime_sec;
  out.stats.thread_budget = in.stats.thread_budget;
  out.stats.portfolio_workers = in.stats.portfolio_workers;
  out.stats.worker_thread_limit = in.stats.worker_thread_limit;
  out.stats.memory_limit_bytes = in.stats.memory_limit_bytes;
  out.stats.memory_limit_enforced = in.stats.memory_limit_enforced;
  out.stats.portfolio_first_result_sec = in.stats.portfolio_first_result_sec;
  out.stats.portfolio_cancel_wait_sec = in.stats.portfolio_cancel_wait_sec;
  out.stats.hard_deadline_enforced = in.stats.hard_deadline_enforced;
  out.stats.deadline_overrun_sec = in.stats.deadline_overrun_sec;
  out.stats.persistent_backend_reused = in.stats.persistent_backend_reused;
  out.stats.incremental_update_count = in.stats.incremental_update_count;
  out.stats.status = in.stats.status;
  out.stats.solver_name = in.stats.solver_name;
  out.stats.cglp_cuts_added = in.stats.cglp_cuts_added;
  out.stats.farkas_ray = std::move(in.stats.farkas_ray);
  out.stats.farkas_ray_eq = std::move(in.stats.farkas_ray_eq);
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

void normalize_conic_matrices(ConicModel& cm) {
  const int n = static_cast<int>(cm.c.size());
  if (n <= 0) return;
  if (cm.G.rows() == 0 && cm.G.cols() != n) {
    cm.G.resize(0, n);
  }
  if (cm.A.rows() == 0 && cm.A.cols() != n) {
    cm.A.resize(0, n);
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
        } else if constexpr (std::is_same_v<T, ConicModel>) {
          normalize_conic_matrices(p);
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

// AUDIT-NAV: 公共求解入口从这里开始；默认适配器顺序、候选选择和最终结果映射
// 分别在 register_default_adapters、StrategyDispatcher::solve 和本文件 solve 中审核。
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
  register_if_missing(std::make_shared<NativeAutoLPAdapter>());
  register_if_missing(std::make_shared<NativeDualSimplexLPAdapter>());
  register_if_missing(std::make_shared<NativeIPMLPAdapter>());
  register_if_missing(std::make_shared<NativePDLPAdapter>());
  register_if_missing(std::make_shared<NativeLCQPAdapter>());
  register_if_missing(std::make_shared<NativeIPMAdapter>());
  register_if_missing(std::make_shared<NativeNLPAdapter>());
  register_if_missing(std::make_shared<NativeConicIPMAdapter>());
  register_if_missing(std::make_shared<StrictHighsBranchAndCutAdapter>());
  register_if_missing(std::make_shared<NativeBranchAndCutAdapter>());

  auto gurobi = std::make_shared<GurobiAdapter>();
  if (gurobi->available()) {
    register_if_missing(gurobi);
  }

  auto cplex = std::make_shared<CplexAdapter>();
  if (cplex->available()) {
    register_if_missing(cplex);
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

api::Result SolverEngine::solve(api::ProblemVariant problem,
                                const SolveOptions& options) const {
  // The transaction clock starts before normalization and validation. The
  // by-value public boundary is intentionally outside the measurable region;
  // callers can use an rvalue or LPModelSession to avoid that copy. See R1/R4
  // in general_solver_performance_program_2026-09-13.md.
  const SolveContext context(options);
  // problem arrives by value (moved in by rvalue callers); normalize_problem
  // takes it by value too, so the chain below is copy-free after the single
  // unavoidable copy at the public boundary for lvalue callers.
  const api::ProblemVariant normalized = normalize_problem(std::move(problem));
  throw_if_invalid(normalized);
  return solve_normalized(normalized, options, context);
}

api::Result SolverEngine::solve_normalized(
    const api::ProblemVariant& normalized, const SolveOptions& options,
    const SolveContext& context) const {
  SolveResult internal = dispatcher_.solve(
      registry_,
      normalized,
      options.preferred_solver,
      options.allow_fallback,
      options.strategy_policy,
      options.class_strategy_policy,
      context);
  internal.stats.thread_budget = context.thread_budget();
  internal.stats.memory_limit_bytes = context.memory_limit_bytes();
  // R1: an in-process backend can only cooperate. Hard deadlines require the
  // process supervisor, and no current adapter enforces a process-wide memory
  // cap (docs/archive/general_solver_performance_program_2026-09-13.md).
  internal.stats.memory_limit_enforced = false;
  internal.stats.hard_deadline_enforced = false;
  if (context.has_deadline()) {
    internal.stats.deadline_overrun_sec = std::max(
        0.0, context.elapsed_sec() - context.requested_time_limit_sec());
  }
  return to_api_result(std::move(internal));
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
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_lp(LPModel&& problem,
                                   const SolveOptions& options) const {
  return solve(api::ProblemVariant{std::move(problem)}, options);
}

api::Result SolverEngine::solve_qp(const QPModel& problem,
                                   const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_nlp(const NLPModel& problem,
                                    const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_milp(const MIPModel& problem,
                                     const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_milp(MIPModel&& problem,
                                     const SolveOptions& options) const {
  return solve(api::ProblemVariant{std::move(problem)}, options);
}

api::Result SolverEngine::solve_minlp(const MINLPModel& problem,
                                      const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

api::Result SolverEngine::solve_conic(const ConicModel& problem,
                                      const SolveOptions& options) const {
  return solve(api::ProblemVariant{problem}, options);
}

}  // namespace mipsolvers::engine
