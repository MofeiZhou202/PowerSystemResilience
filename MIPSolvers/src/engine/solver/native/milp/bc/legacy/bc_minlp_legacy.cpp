/// @file bc_minlp_legacy.cpp
/// @brief Legacy MINLP branch-and-cut implementation.

#include "mipsolvers/engine/detail/bc_minlp_legacy.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_status.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/kernel/kkt/kkt_system.hpp"

namespace mipsolvers::engine::detail {
namespace {

Eigen::VectorXd sparse_abs_diagonal(const Eigen::SparseMatrix<double>& mat, int n) {
  Eigen::VectorXd diag = Eigen::VectorXd::Zero(n);
  for (int col = 0; col < mat.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(mat, col); it; ++it) {
      if (it.row() == it.col()) {
        diag[it.row()] = std::abs(it.value());
      }
    }
  }
  return diag;
}

double minlp_directional_branch_cost(const PseudoCost& pc, bool up_branch) {
  const double sum = up_branch ? pc.up_sum : pc.down_sum;
  const int cnt = up_branch ? pc.up_cnt : pc.down_cnt;
  if (cnt > 0) {
    return std::max(1e-6, sum / static_cast<double>(cnt));
  }
  const double other_sum = up_branch ? pc.down_sum : pc.up_sum;
  const int other_cnt = up_branch ? pc.down_cnt : pc.up_cnt;
  if (other_cnt > 0) {
    return std::max(1e-6, other_sum / static_cast<double>(other_cnt));
  }
  return 1.0;
}

void build_minlp_branch_scores(const NLPModel& base,
                               const Eigen::VectorXd& lb,
                               const Eigen::VectorXd& ub,
                               const SolveResult& relax,
                               double int_tol,
                               std::optional<Eigen::VectorXd>& down_scores,
                               std::optional<Eigen::VectorXd>& up_scores) {
  const int n = static_cast<int>(base.vars.size());
  if (relax.x.size() != n || !base.grad) {
    down_scores.reset();
    up_scores.reset();
    return;
  }
  down_scores = Eigen::VectorXd::Zero(n);
  up_scores = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd& down = *down_scores;
  Eigen::VectorXd& up = *up_scores;

  NLPModel node_nlp = base;
  apply_node_bounds(node_nlp.vars, lb, ub);

  Eigen::VectorXd grad = Eigen::VectorXd::Zero(n);
  node_nlp.grad(relax.x, grad);
  if (node_nlp.sense == Sense::Maximize) {
    grad = -grad;
  }

  Eigen::VectorXd hdiag = Eigen::VectorXd::Ones(n);
  if (node_nlp.hess) {
    Eigen::SparseMatrix<double> hess;
    node_nlp.hess(relax.x, hess);
    hdiag = sparse_abs_diagonal(hess, n).cwiseMax(Eigen::VectorXd::Constant(n, 1e-4));
  }

  Eigen::VectorXd structural = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd lambda;
  Eigen::VectorXd mu;

  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> jg;
  if (node_nlp.g && node_nlp.jac_g) {
    node_nlp.g(relax.x, g);
    node_nlp.jac_g(relax.x, jg);
  }

  std::vector<int> lb_cols;
  std::vector<int> ub_cols;
  lb_cols.reserve(static_cast<size_t>(n));
  ub_cols.reserve(static_cast<size_t>(n));
  for (int j = 0; j < n; ++j) {
    if (std::isfinite(node_nlp.vars[static_cast<size_t>(j)].lb)) {
      lb_cols.push_back(j);
    }
    if (std::isfinite(node_nlp.vars[static_cast<size_t>(j)].ub)) {
      ub_cols.push_back(j);
    }
  }

  Eigen::VectorXd h;
  Eigen::SparseMatrix<double> jh;
  split_inequalities(node_nlp, relax.x, lb_cols, ub_cols, h, jh);

  const int eq_dim = g.size();
  const int ineq_dim = h.size();
  if (relax.constraint_duals.size() == ineq_dim + eq_dim) {
    if (ineq_dim > 0) {
      mu = relax.constraint_duals.head(ineq_dim);
      structural += (jh.transpose() * mu).cwiseAbs();
    }
    if (eq_dim > 0) {
      lambda = relax.constraint_duals.tail(eq_dim);
      structural += (jg.transpose() * lambda).cwiseAbs();
    }
  }

  Eigen::VectorXd box_lb = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd box_ub = Eigen::VectorXd::Zero(n);
  if (relax.box_dual_lb.size() == n) {
    box_lb = relax.box_dual_lb.cwiseAbs();
  }
  if (relax.box_dual_ub.size() == n) {
    box_ub = relax.box_dual_ub.cwiseAbs();
  }

  for (int j = 0; j < n; ++j) {
    const VarType type = base.vars[static_cast<size_t>(j)].type;
    if (type != VarType::Integer && type != VarType::Binary) {
      continue;
    }

    const double xj = relax.x[j];
    const double floor_x = std::floor(xj);
    const double ceil_x = std::ceil(xj);
    const double dist_down = std::max(0.0, xj - floor_x);
    const double dist_up = std::max(0.0, ceil_x - xj);
    if (dist_down <= int_tol && dist_up <= int_tol) {
      continue;
    }

    const double local_curv = std::max(1e-4, hdiag[j]);
    const double coupling = structural[j];
    const double linear_down = std::max(0.0, -grad[j]) * dist_down;
    const double linear_up = std::max(0.0, grad[j]) * dist_up;
    down[j] = std::max(1e-6,
        linear_down + box_ub[j] * dist_down + 0.1 * coupling * dist_down +
        0.5 * local_curv * dist_down * dist_down);
    up[j] = std::max(1e-6,
        linear_up + box_lb[j] * dist_up + 0.1 * coupling * dist_up +
        0.5 * local_curv * dist_up * dist_up);
  }
}

double compute_minlp_node_estimate(const std::vector<VariableMeta>& vars,
                                   const Eigen::VectorXd& x,
                                   const std::optional<Eigen::VectorXd>& down_scores,
                                   const std::optional<Eigen::VectorXd>& up_scores,
                                   double int_tol,
                                   double node_bound) {
  double estimate = node_bound;
  const int n = static_cast<int>(vars.size());
  if (!down_scores || !up_scores) {
    return estimate;
  }
  if (x.size() != n || down_scores->size() != n || up_scores->size() != n) {
    return estimate;
  }
  const Eigen::VectorXd& down = *down_scores;
  const Eigen::VectorXd& up = *up_scores;
  for (int j = 0; j < n; ++j) {
    const VarType type = vars[static_cast<size_t>(j)].type;
    if (type != VarType::Integer && type != VarType::Binary) {
      continue;
    }
    const double xj = x[j];
    const double nearest = std::round(xj);
    if (std::abs(xj - nearest) <= int_tol) {
      continue;
    }
    estimate += std::min(down[j], up[j]);
  }
  return estimate;
}

int choose_minlp_branch_var(const BCOptions& opt,
                            const std::vector<int>& cand,
                            const Eigen::VectorXd& x,
                            const std::vector<PseudoCost>& pc,
                            const std::optional<Eigen::VectorXd>& down_scores,
                            const std::optional<Eigen::VectorXd>& up_scores) {
  int best = -1;
  double best_score = -1.0;
  for (int j : cand) {
    const double frac = x[j] - std::floor(x[j]);
    const double up_frac = 1.0 - frac;
    const double down_pc = minlp_directional_branch_cost(pc[static_cast<size_t>(j)], false);
    const double up_pc = minlp_directional_branch_cost(pc[static_cast<size_t>(j)], true);
    const double base_pc = std::sqrt(std::max(1e-9, down_pc * up_pc));
    const double local_down = (down_scores && down_scores->size() > j) ? (*down_scores)[j] : frac;
    const double local_up = (up_scores && up_scores->size() > j) ? (*up_scores)[j] : up_frac;
    const double local_signal = std::sqrt(std::max(1e-9, local_down * local_up));
    const bool has_history = pc[static_cast<size_t>(j)].down_cnt > 0 || pc[static_cast<size_t>(j)].up_cnt > 0;
    const double score = has_history
        ? base_pc * (1.0 + 0.5 * local_signal)
        : local_signal + std::min(frac, up_frac);
    if (score > best_score + 1e-12) {
      best_score = score;
      best = j;
    }
  }
  if (best >= 0) {
    return best;
  }
  return choose_branch_var(opt, cand, x, pc);
}

// ════════════════════════════════════════════════════════════════════════════
// MINLP B&C Entry Point
// ════════════════════════════════════════════════════════════════════════════

BCResult branch_and_cut_nlp_impl(const MINLPModel& prob, const BCOptions& opt) {
  auto solve_environment = current_bc_env_options();
  if (!solve_environment) solve_environment = capture_bc_env_options();
  ScopedBcEnvOptions environment_scope(solve_environment);
  BCResult out;
  out.effective_environment = solve_environment->active_settings();
  out.stats.solver_name = "NativeBranchAndCutMINLP";

  const auto t0 = std::chrono::steady_clock::now();

  NLPModel base = prob.nonlinear_part;
  const int n = static_cast<int>(base.vars.size());
  if (n <= 0) {
    out.stats.status = bc_status::kEmptyMinlpModel;
    return out;
  }

  for (int idx : prob.integer_idx) {
    if (idx >= 0 && idx < n) {
      base.vars[idx].type = VarType::Integer;
    }
  }
  for (int idx : prob.binary_idx) {
    if (idx >= 0 && idx < n) {
      base.vars[idx].type = VarType::Binary;
      base.vars[idx].lb = std::max(base.vars[idx].lb, 0.0);
      base.vars[idx].ub = std::min(base.vars[idx].ub, 1.0);
    }
  }

  Node root;
  root.lb = Eigen::VectorXd(n);
  root.ub = Eigen::VectorXd(n);
  for (int i = 0; i < n; ++i) {
    root.lb[i] = base.vars[i].lb;
    root.ub[i] = base.vars[i].ub;
  }
  root.x_seed = (base.x0.size() == n) ? clamp_to_bounds(base.x0, root.lb, root.ub)
                                      : Eigen::VectorXd::Zero(n);

  if (!bounds_consistent(root.lb, root.ub)) {
    out.stats.status = bc_status::kInfeasibleVariableBounds;
    return out;
  }

  SolveResult root_relax = solve_nlp_relaxation(base, root.lb, root.ub, &root.x_seed, opt);
  out.bc_stats.lp_solves += 1;
  if (!root_relax.stats.success || root_relax.x.size() != n) {
    out.stats.status = bc_status::kRootNlpRelaxationFailed;
    return out;
  }

  root.x_relax = clamp_to_bounds(root_relax.x, root.lb, root.ub);
  root.x_seed = root.x_relax;
  root.bound = root_relax.stats.objective;
  build_minlp_branch_scores(base, root.lb, root.ub, root_relax, opt.int_tol,
                            root.nlp_down_scores, root.nlp_up_scores);
  root.estimate = compute_minlp_node_estimate(
      base.vars, root.x_relax, root.nlp_down_scores, root.nlp_up_scores,
      opt.int_tol, root.bound);

  bool has_incumbent = false;
  double incumbent_obj = kInf;
  Eigen::VectorXd incumbent_x = Eigen::VectorXd::Zero(n);

  std::vector<PseudoCost> pc(static_cast<size_t>(n));
  std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey> pq;
  std::vector<Node> dfs;
  push_node(pq, dfs, root, opt.node_sel, has_incumbent);

  double global_lb = live_node_lower_bound(pq, dfs);
  if (!std::isfinite(global_lb)) {
    global_lb = root.bound;
  }

  while (true) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (elapsed > opt.time_limit_sec) {
      out.stats.status = bc_status::kTimeLimitReached;
      break;
    }
    if (out.bc_stats.nodes_explored >= opt.max_nodes) {
      out.stats.status = bc_status::kNodeLimitReached;
      break;
    }

    Node cur;
    if (!pop_node(pq, dfs, opt.node_sel, has_incumbent, cur)) {
      out.stats.status = bc_status::kSearchQueueExhausted;
      break;
    }

    ++out.bc_stats.nodes_explored;

    if (incumbent_prunes_node(has_incumbent, incumbent_obj, cur.bound)) {
      continue;
    }

    std::vector<int> frac;
    if (!fractional_indices(base.vars, cur.x_relax, opt.int_tol, frac)) {
      const double obj = cur.bound;
      if (!has_incumbent || obj < incumbent_obj) {
        has_incumbent = true;
        incumbent_obj = obj;
        incumbent_x = cur.x_relax;
      }
      continue;
    }

    const int j = choose_minlp_branch_var(
        opt, frac, cur.x_relax, pc, cur.nlp_down_scores, cur.nlp_up_scores);
    if (j < 0) {
      continue;
    }

    const double xj = cur.x_relax[j];
    Node left = cur;
    left.depth = cur.depth + 1;
    left.ub[j] = std::min(left.ub[j], std::floor(xj));
    left.x_seed = clamp_to_bounds(cur.x_relax, left.lb, left.ub);

    Node right = cur;
    right.depth = cur.depth + 1;
    right.lb[j] = std::max(right.lb[j], std::ceil(xj));
    right.x_seed = clamp_to_bounds(cur.x_relax, right.lb, right.ub);

    auto process = [&](Node& child, bool up) {
      if (!bounds_consistent(child.lb, child.ub)) {
        return;
      }
      SolveResult relax = solve_nlp_relaxation(base, child.lb, child.ub, &child.x_seed, opt);
      ++out.bc_stats.lp_solves;
      if (!relax.stats.success || relax.x.size() != n) {
        return;
      }

      child.x_relax = clamp_to_bounds(relax.x, child.lb, child.ub);
      child.x_seed = child.x_relax;
      child.bound = relax.stats.objective;
        build_minlp_branch_scores(base, child.lb, child.ub, relax, opt.int_tol,
                    child.nlp_down_scores, child.nlp_up_scores);
        child.estimate = compute_minlp_node_estimate(
          base.vars, child.x_relax, child.nlp_down_scores, child.nlp_up_scores,
          opt.int_tol, child.bound);

      const double gain = std::max(0.0, child.bound - cur.bound);
      if (up) {
        pc[j].up_sum += gain;
        pc[j].up_cnt += 1;
      } else {
        pc[j].down_sum += gain;
        pc[j].down_cnt += 1;
      }

      if (incumbent_prunes_node(has_incumbent, incumbent_obj, child.bound)) {
        return;
      }

      push_node(pq, dfs, child, opt.node_sel, has_incumbent);
    };

    process(left, false);
    process(right, true);

    const double live_lb = live_node_lower_bound(pq, dfs);
    if (std::isfinite(live_lb)) {
      global_lb = live_lb;
    } else if (has_incumbent) {
      global_lb = incumbent_obj;
    }
    out.bc_stats.best_bound = global_lb;

    if (has_incumbent && std::isfinite(global_lb)) {
      const double denom = std::max(1.0, std::abs(incumbent_obj));
      const double rel_gap = std::max(0.0, (incumbent_obj - global_lb) / denom);
      out.bc_stats.gap = rel_gap;
      if (!opt.require_tree_exhaustion_certificate &&
          rel_gap <= opt.gap_tol) {
        out.stats.status = bc_status::kOptimalityGapReached;
        break;
      }
    }
  }

  const double runtime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  out.bc_stats.runtime_sec = runtime;
  out.bc_stats.best_bound = global_lb;
  out.bc_stats.best_obj = has_incumbent ? incumbent_obj : kInf;

  out.stats.runtime_sec = runtime;
  out.stats.iterations = out.bc_stats.nodes_explored;
  out.stats.success = has_incumbent;
  out.stats.objective = has_incumbent ? incumbent_obj : 0.0;
  out.stats.mip_gap = std::isfinite(out.bc_stats.gap) ? out.bc_stats.gap : kInf;

  if (has_incumbent) {
    out.x = incumbent_x;
    if (out.stats.status == bc_status::kSearchQueueExhausted) {
      out.bc_stats.gap = 0.0;
      out.stats.mip_gap = 0.0;
      out.stats.status = bc_status::kOptimalTreeExhausted;
    } else if (out.stats.status.empty()) {
      out.stats.status = bc_status::kFeasibleIncumbentFound;
    }
  } else {
    out.x = Eigen::VectorXd::Zero(n);
    if (out.stats.status.empty()) {
      out.stats.status = bc_status::kNoFeasibleIntegerSolutionFound;
    }
  }

  out.bc_stats.status = out.stats.status;
  return out;
}


}  // namespace

BCResult branch_and_cut_nlp(const MINLPModel& prob, const BCOptions& opt) {
  return branch_and_cut_nlp_impl(prob, opt);
}

}  // namespace mipsolvers::engine::detail
