#include "mipsolvers/engine/api/session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "Highs.h"
#include "mipsolvers/engine/api/problem.hpp"
#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/solve_context.hpp"
#include "mipsolvers/engine/util/problem_validation.hpp"

namespace mipsolvers::engine {
namespace {

void require_size(Eigen::Index actual, Eigen::Index expected,
                  const char* field) {
  if (actual != expected) {
    throw std::invalid_argument(std::string(field) + " has incompatible size");
  }
}

void throw_if_invalid_model(const LPModel& model) {
  const ValidationReport report = validate(model);
  if (!report.valid) {
    throw std::invalid_argument(
        report.errors.empty() ? "invalid persistent LP model" : report.errors.front());
  }
}

bool highs_status_has_solution(HighsModelStatus status) {
  return status == HighsModelStatus::kOptimal ||
         status == HighsModelStatus::kTimeLimit ||
         status == HighsModelStatus::kIterationLimit ||
         status == HighsModelStatus::kSolutionLimit;
}

const char* highs_status_label(HighsModelStatus status) {
  switch (status) {
    case HighsModelStatus::kOptimal: return "Optimal";
    case HighsModelStatus::kInfeasible: return "Infeasible";
    case HighsModelStatus::kUnbounded: return "Unbounded";
    case HighsModelStatus::kUnboundedOrInfeasible: return "UnboundedOrInfeasible";
    case HighsModelStatus::kTimeLimit: return "TimeLimit";
    case HighsModelStatus::kIterationLimit: return "IterationLimit";
    case HighsModelStatus::kSolutionLimit: return "SolutionLimit";
    case HighsModelStatus::kMemoryLimit: return "MemoryLimit";
    case HighsModelStatus::kInterrupt:
    case HighsModelStatus::kHighsInterrupt: return "Interrupted";
    default: return "Failed";
  }
}

}  // namespace

class LPModelSession::Impl {
 public:
  Impl(LPModel input, bool register_defaults)
      : engine(register_defaults), problem(std::move(input)) {
    LPModel& lp = std::get<LPModel>(problem);
    const int n = static_cast<int>(lp.c.size());
    if (n > 0 && lp.A.rows() == 0 && lp.A.cols() != n) {
      lp.A.resize(0, n);
    }
    if (n > 0 && lp.Aeq.rows() == 0 && lp.Aeq.cols() != n) {
      lp.Aeq.resize(0, n);
    }
    throw_if_invalid_model(lp);
    rebuild_highs();
  }

  void rebuild_highs() {
    const LPModel& lp = std::get<LPModel>(problem);
    const HighsInt n = static_cast<HighsInt>(lp.c.size());
    const HighsInt m_ineq = static_cast<HighsInt>(lp.A.rows());
    const HighsInt m_eq = static_cast<HighsInt>(lp.Aeq.rows());
    const HighsInt m = m_ineq + m_eq;
    std::vector<double> col_cost(static_cast<std::size_t>(n));
    std::vector<double> col_lower(static_cast<std::size_t>(n));
    std::vector<double> col_upper(static_cast<std::size_t>(n));
    for (HighsInt j = 0; j < n; ++j) {
      col_cost[static_cast<std::size_t>(j)] = lp.c[j];
      col_lower[static_cast<std::size_t>(j)] =
          lp.vars[static_cast<std::size_t>(j)].lb;
      col_upper[static_cast<std::size_t>(j)] =
          lp.vars[static_cast<std::size_t>(j)].ub;
    }
    std::vector<double> row_lower(static_cast<std::size_t>(m), -kHighsInf);
    std::vector<double> row_upper(static_cast<std::size_t>(m), kHighsInf);
    for (HighsInt i = 0; i < m_ineq; ++i) {
      row_lower[static_cast<std::size_t>(i)] =
          lp_row_lhs_or_neg_inf(lp, static_cast<int>(i));
      row_upper[static_cast<std::size_t>(i)] = lp.b[i];
    }
    for (HighsInt i = 0; i < m_eq; ++i) {
      row_lower[static_cast<std::size_t>(m_ineq + i)] = lp.beq[i];
      row_upper[static_cast<std::size_t>(m_ineq + i)] = lp.beq[i];
    }
    std::vector<HighsInt> start(static_cast<std::size_t>(n + 1), 0);
    std::vector<HighsInt> index;
    std::vector<double> value;
    index.reserve(static_cast<std::size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros()));
    value.reserve(index.capacity());
    for (HighsInt j = 0; j < n; ++j) {
      start[static_cast<std::size_t>(j)] = static_cast<HighsInt>(index.size());
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
        if (it.value() == 0.0) continue;
        index.push_back(static_cast<HighsInt>(it.row()));
        value.push_back(it.value());
      }
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
        if (it.value() == 0.0) continue;
        index.push_back(m_ineq + static_cast<HighsInt>(it.row()));
        value.push_back(it.value());
      }
    }
    start[static_cast<std::size_t>(n)] = static_cast<HighsInt>(index.size());

    auto candidate = std::make_unique<Highs>();
    if (candidate->setOptionValue("output_flag", false) == HighsStatus::kError ||
        candidate->setOptionValue("log_to_console", false) == HighsStatus::kError ||
        candidate->setOptionValue("solver", "simplex") == HighsStatus::kError ||
        candidate->passModel(
            n, m, static_cast<HighsInt>(index.size()),
            static_cast<HighsInt>(MatrixFormat::kColwise),
            static_cast<HighsInt>(lp.sense == Sense::Maximize
                                      ? ObjSense::kMaximize
                                      : ObjSense::kMinimize),
            0.0, col_cost.data(), col_lower.data(), col_upper.data(),
            row_lower.data(), row_upper.data(), start.data(), index.data(),
            value.data()) == HighsStatus::kError) {
      throw std::runtime_error("HiGHS rejected the persistent LP model");
    }
    highs = std::move(candidate);
    configured_threads = 0;
    backend_has_solution = false;
  }

  bool change_objective(const LPModel& lp) {
    if (lp.c.size() == 0) return true;
    return highs->changeColsCost(0, static_cast<HighsInt>(lp.c.size() - 1),
                                 lp.c.data()) != HighsStatus::kError;
  }

  bool change_variable_bounds(const LPModel& lp) {
    const HighsInt n = static_cast<HighsInt>(lp.c.size());
    if (n == 0) return true;
    std::vector<double> lower(static_cast<std::size_t>(n));
    std::vector<double> upper(static_cast<std::size_t>(n));
    for (HighsInt j = 0; j < n; ++j) {
      lower[static_cast<std::size_t>(j)] = lp.vars[static_cast<std::size_t>(j)].lb;
      upper[static_cast<std::size_t>(j)] = lp.vars[static_cast<std::size_t>(j)].ub;
    }
    return highs->changeColsBounds(0, n - 1, lower.data(), upper.data()) !=
           HighsStatus::kError;
  }

  bool change_rows(const LPModel& lp) {
    const HighsInt m_ineq = static_cast<HighsInt>(lp.A.rows());
    const HighsInt m_eq = static_cast<HighsInt>(lp.Aeq.rows());
    if (m_ineq > 0) {
      std::vector<double> lower(static_cast<std::size_t>(m_ineq));
      for (HighsInt i = 0; i < m_ineq; ++i)
        lower[static_cast<std::size_t>(i)] =
            lp_row_lhs_or_neg_inf(lp, static_cast<int>(i));
      if (highs->changeRowsBounds(0, m_ineq - 1, lower.data(), lp.b.data()) ==
          HighsStatus::kError)
        return false;
    }
    if (m_eq > 0 &&
        highs->changeRowsBounds(m_ineq, m_ineq + m_eq - 1, lp.beq.data(),
                                lp.beq.data()) == HighsStatus::kError)
      return false;
    return true;
  }

  api::Result solve_persistent(const SolveContext& context) const {
    const auto started = std::chrono::steady_clock::now();
    api::Result out;
    out.stats.solver_name = "HiGHS";
    out.stats.thread_budget = context.thread_budget();
    out.stats.memory_limit_bytes = context.memory_limit_bytes();
    out.stats.incremental_update_count = update_count;
    if (context.stop_requested()) {
      out.stats.status = context.deadline_expired() ? "Time limit" : "Cancelled";
      return out;
    }

    // A zero caller setting preserves HiGHS' automatic policy; the resolved
    // hardware capacity is a portfolio budget, not mandatory backend work.
    // See general_solver_performance_program_2026-09-13.md, R2.
    const int target_threads =
        context.has_explicit_thread_budget() ? context.thread_budget() : 0;
    bool retained_backend = backend_has_solution;
    if (configured_threads != target_threads) {
      // HiGHS owns a process-global scheduler. Rebuilding on a budget change
      // preserves the hard budget contract instead of reusing a stale pool.
      const_cast<Impl*>(this)->rebuild_highs();
      retained_backend = false;
    }
    if (!backend_has_solution) Highs::resetGlobalScheduler(/*blocking=*/true);
    if ((target_threads > 0 &&
         highs->setOptionValue("threads", target_threads) == HighsStatus::kError) ||
        highs->setOptionValue("random_seed",
                              static_cast<HighsInt>(context.random_seed())) ==
            HighsStatus::kError ||
        highs->setOptionValue("time_limit",
                              context.backend_time_limit_sec(kHighsInf)) ==
            HighsStatus::kError) {
      out.stats.status = "HiGHS rejected persistent solve resources";
      return out;
    }
    configured_threads = target_threads;
    const HighsStatus run_status = highs->run();
    const HighsModelStatus model_status = highs->getModelStatus();
    const HighsSolution& solution = highs->getSolution();
    const bool optimal = model_status == HighsModelStatus::kOptimal;
    out.stats.success =
        optimal || (highs_status_has_solution(model_status) && solution.value_valid);
    out.stats.strict_convergence = optimal;
    out.stats.acceptable_convergence = out.stats.success;
    out.stats.status = std::string("HiGHS ") + highs_status_label(model_status);
    if (run_status == HighsStatus::kError && !out.stats.success)
      out.stats.status = "HiGHS persistent solve failed";

    const LPModel& lp = std::get<LPModel>(problem);
    const HighsInt n = static_cast<HighsInt>(lp.c.size());
    const HighsInt m = static_cast<HighsInt>(lp.A.rows() + lp.Aeq.rows());
    const HighsInfo& info = highs->getInfo();
    out.stats.objective = info.objective_function_value;
    out.stats.iterations = static_cast<int>(info.simplex_iteration_count +
                                            info.ipm_iteration_count +
                                            info.pdlp_iteration_count);
    out.stats.primal_feas = info.max_primal_infeasibility;
    out.stats.dual_feas = info.max_dual_infeasibility;
    out.stats.residual_inf = std::max(out.stats.primal_feas, out.stats.dual_feas);
    if (solution.value_valid && static_cast<HighsInt>(solution.col_value.size()) >= n) {
      out.x.resize(n);
      for (HighsInt j = 0; j < n; ++j)
        out.x[j] = solution.col_value[static_cast<std::size_t>(j)];
    }
    if (solution.dual_valid &&
        static_cast<HighsInt>(solution.row_dual.size()) >= m &&
        static_cast<HighsInt>(solution.col_dual.size()) >= n) {
      // HiGHS C++ API, solution/basis conventions: row duals are in model row
      // order and reduced-cost signs identify active lower/upper box bounds.
      out.constraint_duals.resize(m);
      for (HighsInt i = 0; i < m; ++i)
        out.constraint_duals[i] = solution.row_dual[static_cast<std::size_t>(i)];
      out.box_dual_lb = Eigen::VectorXd::Zero(n);
      out.box_dual_ub = Eigen::VectorXd::Zero(n);
      for (HighsInt j = 0; j < n; ++j) {
        const double dual = solution.col_dual[static_cast<std::size_t>(j)];
        if (dual > 0.0) out.box_dual_lb[j] = dual;
        if (dual < 0.0) out.box_dual_ub[j] = -dual;
      }
    }
    backend_has_solution = out.stats.success;
    out.stats.persistent_backend_reused = retained_backend;
    ++solve_count;
    out.stats.runtime_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    if (context.has_deadline()) {
      out.stats.deadline_overrun_sec = std::max(
          0.0, context.elapsed_sec() - context.requested_time_limit_sec());
    }
    return out;
  }

  SolverEngine engine;
  api::ProblemVariant problem;
  mutable std::unique_ptr<Highs> highs;
  mutable int configured_threads{0};
  mutable bool backend_has_solution{false};
  mutable std::size_t solve_count{0};
  std::size_t update_count{0};
};

LPModelSession::LPModelSession(LPModel model, bool register_default_adapters)
    : impl_(std::make_unique<Impl>(std::move(model), register_default_adapters)) {}

LPModelSession::~LPModelSession() = default;
LPModelSession::LPModelSession(LPModelSession&&) noexcept = default;
LPModelSession& LPModelSession::operator=(LPModelSession&&) noexcept = default;

void LPModelSession::update_objective(Eigen::VectorXd objective) {
  LPModel& model = std::get<LPModel>(impl_->problem);
  require_size(objective.size(), model.c.size(), "objective");
  Eigen::VectorXd previous = std::move(model.c);
  model.c = std::move(objective);
  try {
    throw_if_invalid_model(model);
  } catch (...) {
    model.c = std::move(previous);
    throw;
  }
  if (!impl_->change_objective(model)) {
    model.c = std::move(previous);
    impl_->rebuild_highs();
    throw std::runtime_error("HiGHS rejected the incremental objective update");
  }
  ++impl_->update_count;
}

void LPModelSession::update_variable_bounds(Eigen::VectorXd lower,
                                            Eigen::VectorXd upper) {
  LPModel& model = std::get<LPModel>(impl_->problem);
  const Eigen::Index n = model.c.size();
  require_size(lower.size(), n, "variable lower bounds");
  require_size(upper.size(), n, "variable upper bounds");
  std::vector<VariableMeta> previous = model.vars;
  for (Eigen::Index j = 0; j < n; ++j) {
    model.vars[static_cast<std::size_t>(j)].lb = lower[j];
    model.vars[static_cast<std::size_t>(j)].ub = upper[j];
  }
  try {
    throw_if_invalid_model(model);
  } catch (...) {
    model.vars = std::move(previous);
    throw;
  }
  if (!impl_->change_variable_bounds(model)) {
    model.vars = std::move(previous);
    impl_->rebuild_highs();
    throw std::runtime_error("HiGHS rejected the incremental variable-bound update");
  }
  ++impl_->update_count;
}

void LPModelSession::update_inequality_rhs(Eigen::VectorXd lower,
                                           Eigen::VectorXd upper) {
  LPModel& model = std::get<LPModel>(impl_->problem);
  const Eigen::Index rows = model.A.rows();
  require_size(lower.size(), rows, "inequality lower RHS");
  require_size(upper.size(), rows, "inequality upper RHS");
  Eigen::VectorXd previous_lower = std::move(model.row_lhs);
  Eigen::VectorXd previous_upper = std::move(model.b);
  model.row_lhs = std::move(lower);
  model.b = std::move(upper);
  try {
    throw_if_invalid_model(model);
  } catch (...) {
    model.row_lhs = std::move(previous_lower);
    model.b = std::move(previous_upper);
    throw;
  }
  if (!impl_->change_rows(model)) {
    model.row_lhs = std::move(previous_lower);
    model.b = std::move(previous_upper);
    impl_->rebuild_highs();
    throw std::runtime_error("HiGHS rejected the incremental row-bound update");
  }
  ++impl_->update_count;
}

void LPModelSession::update_equality_rhs(Eigen::VectorXd rhs) {
  LPModel& model = std::get<LPModel>(impl_->problem);
  require_size(rhs.size(), model.Aeq.rows(), "equality RHS");
  Eigen::VectorXd previous = std::move(model.beq);
  model.beq = std::move(rhs);
  try {
    throw_if_invalid_model(model);
  } catch (...) {
    model.beq = std::move(previous);
    throw;
  }
  if (!impl_->change_rows(model)) {
    model.beq = std::move(previous);
    impl_->rebuild_highs();
    throw std::runtime_error("HiGHS rejected the incremental equality update");
  }
  ++impl_->update_count;
}

api::Result LPModelSession::solve(const SolveOptions& options) const {
  const SolveContext context(options);
  if (options.preferred_solver == "HiGHS" && !options.allow_fallback) {
    return impl_->solve_persistent(context);
  }
  return impl_->engine.solve_normalized(impl_->problem, options, context);
}

const LPModel& LPModelSession::model() const noexcept {
  return std::get<LPModel>(impl_->problem);
}

}  // namespace mipsolvers::engine
