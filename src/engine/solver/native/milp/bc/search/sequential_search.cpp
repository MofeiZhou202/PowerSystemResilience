/// @file sequential_search.cpp
/// @brief Implementation of sequential search driver (Phase 4.4b: Full Implementation)
///
/// Sequential search driver executes the standard branch-and-cut tree loop:
/// 1. Initialize with root node and search infrastructure
/// 2. Iteratively: select node → evaluate → branch
/// 3. Termination: time/node/gap limit or queue exhausted

#include "mipsolvers/engine/solver/native/milp/bc/search/sequential_search.hpp"

#include <chrono>
#include <queue>
#include <algorithm>
#include <cmath>

#include "mipsolvers/engine/solver/native/milp/bc/parallel/shared_state.hpp"
#include "mipsolvers/engine/solver/native/milp/bc/search/node_evaluator.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/bc/options.hpp"

namespace mipsolvers::engine::solver::native::milp::bc {

SequentialSearchDriver::SequentialSearchDriver(BCSolveContext& context)
    : ctx_(&context) {}

SequentialSearchStats SequentialSearchDriver::execute(
    const detail::Node& root_node) {
  SequentialSearchStats stats;
  stats.nodes_explored = 0;
  stats.nodes_pruned = 0;
  stats.solutions_found = 0;
  stats.search_time_sec = 0.0;
  stats.termination_reason = "Running";

  // PHASE 4.4b: Full sequential search loop
  auto search_start = std::chrono::steady_clock::now();

  // STEP 1: Initialize search infrastructure
  initialize_search(root_node);
  NodeEvaluator evaluator(*ctx_);
  const LPModel& base_lp = ctx_->base_lp();
  const BCOptions& opt = ctx_->options();

  // STEP 2-3: Main search loop: select → evaluate → branch
  while (!should_terminate()) {
    detail::Node current_node;
    if (!select_next_node(current_node)) {
      stats.termination_reason = "Queue exhausted (proven optimal or infeasible)";
      break;
    }

    stats.nodes_explored++;
    ctx_->increment_nodes();

    if (detail::incumbent_prunes_node(
            ctx_->has_incumbent(), ctx_->incumbent_objective(), current_node.bound)) {
      stats.nodes_pruned++;
      continue;
    }

    std::vector<int> frac;
    if (!detail::fractional_indices(base_lp.vars, current_node.x_relax, opt.int_tol, frac)) {
      const Eigen::VectorXd x_int = detail::project_integer_solution(
          base_lp.vars, current_node.x_relax, current_node.lb, current_node.ub);
      if (detail::satisfies_with_bounds(
              base_lp, x_int, current_node.lb, current_node.ub,
              detail::kNodeFeasibilityTol)) {
        const double obj = detail::objective_value(base_lp.c, x_int, base_lp.sense);
        if (ctx_->update_incumbent(x_int, obj)) {
          ++stats.solutions_found;
        }
      } else {
        stats.nodes_pruned++;
      }
      continue;
    }

    int branch_var = -1;
    {
      std::lock_guard<std::mutex> lk(ctx_->pseudocost_mutex());
      branch_var = detail::choose_branch_var(
          opt, frac, current_node.x_relax, ctx_->pseudocosts());
    }
    if (branch_var < 0) {
      stats.nodes_pruned++;
      continue;
    }

    const double xj = current_node.x_relax[branch_var];
    detail::Node child_down = current_node.branch_child();
    child_down.ub[branch_var] = std::min(child_down.ub[branch_var], std::floor(xj));
    detail::append_branch_reason(child_down, branch_var, false, child_down.ub[branch_var]);

    detail::Node child_up = current_node.branch_child();
    child_up.lb[branch_var] = std::max(child_up.lb[branch_var], std::ceil(xj));
    detail::append_branch_reason(child_up, branch_var, true, child_up.lb[branch_var]);

    const double incumbent_obj = ctx_->incumbent_objective();
    ChildEvalResult down_eval = evaluator.evaluate(
        child_down, false, branch_var, current_node.bound, incumbent_obj);
    ChildEvalResult up_eval = evaluator.evaluate(
        child_up, true, branch_var, current_node.bound, incumbent_obj);

    auto handle_child = [&](ChildEvalResult& eval) {
      if (!eval.feasible) {
        stats.nodes_pruned++;
        return;
      }

      for (const auto& sol : eval.new_solutions) {
        if (ctx_->update_incumbent(sol.x, sol.obj)) {
          ++stats.solutions_found;
        }
      }

      detail::Node child = std::move(eval.updated_child);
      std::vector<int> child_frac;
      if (!detail::fractional_indices(base_lp.vars, child.x_relax, opt.int_tol, child_frac)) {
        const Eigen::VectorXd x_int = detail::project_integer_solution(
            base_lp.vars, child.x_relax, child.lb, child.ub);
        if (detail::satisfies_with_bounds(
                base_lp, x_int, child.lb, child.ub, detail::kNodeFeasibilityTol)) {
          const double obj = detail::objective_value(base_lp.c, x_int, base_lp.sense);
          if (ctx_->update_incumbent(x_int, obj)) {
            ++stats.solutions_found;
          }
        } else {
          stats.nodes_pruned++;
        }
        return;
      }

      {
        std::lock_guard<std::mutex> lk(ctx_->pseudocost_mutex());
        child.estimate = detail::compute_node_estimate(
            base_lp.vars, child.x_relax, ctx_->pseudocosts(), opt.int_tol, child.bound);
      }

      const bool has_inc = ctx_->has_incumbent();
      queue_->push(std::move(child), has_inc);
    };

    handle_child(down_eval);
    handle_child(up_eval);
  }

  if (stats.termination_reason == "Running") {
    const BCOptions& final_opt = ctx_->options();
    if (ctx_->should_terminate()) {
      stats.termination_reason = "Termination requested";
    } else if (ctx_->nodes_explored() >= static_cast<int64_t>(final_opt.max_nodes)) {
      stats.termination_reason = "Node limit reached";
    } else if (final_opt.time_limit_sec > 0.0 && ctx_->remaining_time() <= 0.0) {
      stats.termination_reason = "Time limit reached";
    } else {
      const double gap = ctx_->gap();
      if (std::isfinite(gap) &&
          gap <= final_opt.gap_tol * std::max(1.0, std::abs(ctx_->incumbent_objective()))) {
        stats.termination_reason = "Gap target reached";
      } else if (queue_ && queue_->empty()) {
        stats.termination_reason = "Queue exhausted (proven optimal or infeasible)";
      } else {
        stats.termination_reason = "Search stopped";
      }
    }
  }

  auto search_end = std::chrono::steady_clock::now();
  stats.search_time_sec =
      std::chrono::duration<double>(search_end - search_start).count();

  return stats;
}

void SequentialSearchDriver::initialize_search(const detail::Node& root) {
  // Create the node queue using the context's configured selection strategy.
  const BCOptions& opt = ctx_->options();
  queue_ = std::make_unique<detail::NodeQueue>(opt.node_sel);

  // Seed with root node.
  const bool has_inc = ctx_->has_incumbent();
  queue_->push(root, has_inc);
}

bool SequentialSearchDriver::select_next_node(detail::Node& selected) {
  if (!queue_ || queue_->empty()) return false;
  return queue_->pop(selected, ctx_->has_incumbent());
}

void SequentialSearchDriver::branch(
    const detail::Node& current,
    int branch_var) {
  if (!queue_) return;

  // Determine the branching value from the LP solution at the current node.
  const double frac_val = current.x_relax.size() > branch_var
                              ? current.x_relax[branch_var]
                              : 0.5;
  const double floor_val = std::floor(frac_val);
  const double ceil_val  = std::ceil(frac_val);

  const bool has_inc = ctx_->has_incumbent();

  // Down-branch: x[branch_var] <= floor_val
  {
    detail::Node down = current.branch_child();
    down.ub[branch_var] = floor_val;
    detail::BranchDomainLiteral lit;
    lit.var_idx = branch_var;
    lit.is_lb   = false;
    lit.value   = floor_val;
    down.branch_reasons.push_back(lit);
    queue_->push(std::move(down), has_inc);
  }

  // Up-branch: x[branch_var] >= ceil_val
  {
    detail::Node up = current.branch_child();
    up.lb[branch_var] = ceil_val;
    detail::BranchDomainLiteral lit;
    lit.var_idx = branch_var;
    lit.is_lb   = true;
    lit.value   = ceil_val;
    up.branch_reasons.push_back(lit);
    queue_->push(std::move(up), has_inc);
  }
}

bool SequentialSearchDriver::should_terminate() const {
  if (!ctx_) return true;
  if (ctx_->should_terminate()) return true;

  const BCOptions& opt = ctx_->options();

  // Node count limit.
  if (ctx_->nodes_explored() >= static_cast<int64_t>(opt.max_nodes)) return true;

  // Time limit.
  if (opt.time_limit_sec > 0.0 && ctx_->remaining_time() <= 0.0) return true;

  // Gap tolerance: if gap is closed, stop.
  const double gap = ctx_->gap();
  if (std::isfinite(gap) && gap <= opt.gap_tol * std::max(1.0, std::abs(ctx_->incumbent_objective()))) {
    return true;
  }

  return false;
}

}  // namespace mipsolvers::engine::solver::native::milp::bc
