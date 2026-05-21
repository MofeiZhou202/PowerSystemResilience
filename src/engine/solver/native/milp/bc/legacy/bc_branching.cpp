/// @file bc_branching.cpp
/// @brief Branch variable selection strategies for the B&C solver.

#include "mipsolvers/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::detail {

namespace {

double directional_branch_cost(const PseudoCost& pseudocost,
                               double fractionality,
                               bool is_up,
                               double offset = 0.0) {
  constexpr double eps = 1e-6;
  const double base = is_up ? pseudocost.up_lcb() : pseudocost.down_lcb();
  const double history = is_up ? pseudocost.up_history_multiplier()
                               : pseudocost.down_history_multiplier();
  return std::max(fractionality * std::max(base + offset, eps) * history, eps);
}

double pseudocost_branch_score(int j,
                               const Eigen::VectorXd& x,
                               const std::vector<PseudoCost>& pc) {
  const double frac = x[j] - std::floor(x[j]);
  const double qd = directional_branch_cost(pc[j], frac, false);
  const double qu = directional_branch_cost(pc[j], 1.0 - frac, true);
  return qd * qu;
}

int branch_priority_value(int j, const std::vector<int>& priority) {
  return (j >= 0 && j < static_cast<int>(priority.size())) ? priority[j] : 0;
}

bool collect_dynamic_prior_scores(const std::vector<int>& cand,
                                  const Eigen::VectorXd& x,
                                  const BCBranchingPriorFn& dynamic_prior,
                                  const BCBranchContext& context,
                                  std::vector<double>& mapped_scores) {
  if (!dynamic_prior || cand.empty()) return false;

  BCBranchContext local_context = context;
  if (local_context.candidates == nullptr) {
    local_context.candidates = &cand;
  }
  if (local_context.lp_x == nullptr || local_context.lp_x_size == 0) {
    local_context.lp_x = x.data();
    local_context.lp_x_size = static_cast<std::size_t>(x.size());
  }
  if (local_context.candidates->size() != cand.size()) return false;
  const auto& score_columns = *local_context.candidates;

  std::vector<double> raw_scores;
  try {
    dynamic_prior(local_context, raw_scores);
  } catch (...) {
    return false;
  }
  if (raw_scores.empty()) return false;

  mapped_scores.assign(cand.size(), 0.0);
  if (raw_scores.size() == cand.size()) {
    bool any = false;
    for (std::size_t k = 0; k < cand.size(); ++k) {
      const double score = raw_scores[k];
      if (!std::isfinite(score)) continue;
      mapped_scores[k] = score;
      any = any || std::abs(score) > 0.0;
    }
    return any;
  }

  int max_col = -1;
  for (int j : score_columns) max_col = std::max(max_col, j);
  if (max_col < 0 || static_cast<int>(raw_scores.size()) <= max_col) {
    return false;
  }
  bool any = false;
  for (std::size_t k = 0; k < cand.size(); ++k) {
    const int col = score_columns[k];
    if (col < 0) continue;
    const double score = raw_scores[static_cast<std::size_t>(col)];
    if (!std::isfinite(score)) continue;
    mapped_scores[k] = score;
    any = any || std::abs(score) > 0.0;
  }
  return any;
}

double most_infeasible_score(int j, const Eigen::VectorXd& x) {
  const double frac = std::abs(x[j] - std::floor(x[j]));
  return 0.5 - std::abs(frac - 0.5);
}

double base_branch_score(BranchingStrategy strategy,
                         int ordinal,
                         int j,
                         const Eigen::VectorXd& x,
                         const std::vector<PseudoCost>& pc) {
  switch (strategy) {
    case BranchingStrategy::FirstFractional:
      return -static_cast<double>(ordinal) * 1e-9;
    case BranchingStrategy::MostInfeasible:
      return most_infeasible_score(j, x);
    case BranchingStrategy::Pseudocost:
    default:
      return std::log1p(std::max(0.0, pseudocost_branch_score(j, x, pc)));
  }
}

int choose_branch_var_with_dynamic_prior(const BCOptions& opt,
                                         const std::vector<int>& cand,
                                         const Eigen::VectorXd& x,
                                         const std::vector<PseudoCost>& pc,
                                         const std::vector<int>& priority,
                                         const BCBranchingPriorFn& dynamic_prior,
                                         const BCBranchContext& context) {
  std::vector<double> prior_scores;
  if (!collect_dynamic_prior_scores(cand, x, dynamic_prior, context, prior_scores)) {
    return priority.empty() ? choose_branch_var(opt, cand, x, pc)
                            : choose_branch_var(opt, cand, x, pc, priority);
  }

  int best = cand.front();
  int best_priority = priority.empty()
                          ? std::numeric_limits<int>::min()
                          : branch_priority_value(best, priority);
  double best_score = -std::numeric_limits<double>::infinity();
  for (std::size_t k = 0; k < cand.size(); ++k) {
    const int j = cand[k];
    const int prio = priority.empty() ? 0 : branch_priority_value(j, priority);
    if (!priority.empty() && prio < best_priority) continue;
    const double base = base_branch_score(opt.branching, static_cast<int>(k), j, x, pc);
    const double combined = base + prior_scores[k];
    if ((!priority.empty() && prio > best_priority) ||
        combined > best_score ||
        (std::abs(combined - best_score) <= 1e-12 && j < best)) {
      best = j;
      best_priority = prio;
      best_score = combined;
    }
  }
  return best;
}

}  // namespace

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const std::vector<PseudoCost>& pc,
                                const std::vector<int>* priority) {
  (void)priority;
  return pseudocost_branch_score(j, x, pc);
}

int choose_branch_var_most_infeasible(const std::vector<int>& cand,
                                      const Eigen::VectorXd& x) {
  int best = cand.front();
  double best_score = -1.0;
  for (int j : cand) {
    const double frac = std::abs(x[j] - std::floor(x[j]));
    const double s = 0.5 - std::abs(frac - 0.5);
    if (s > best_score) {
      best_score = s;
      best = j;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc) {
  int best = cand.front();
  double best_score = -1.0;
  for (int j : cand) {
    const double score = pseudocost_branch_score(j, x, pc);
    if (score > best_score) {
      best_score = score;
      best = j;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority) {
  if (priority.empty()) return choose_branch_var_pseudocost(cand, x, pc);
  int best = cand.front();
  double best_score = -1.0;
  int best_priority = branch_priority_value(best, priority);
  for (int j : cand) {
    const int prio = branch_priority_value(j, priority);
    const double score = pseudocost_branch_score(j, x, pc);
    if (prio > best_priority || (prio == best_priority && score > best_score)) {
      best_score = score;
      best = j;
      best_priority = prio;
    }
  }
  return best;
}

int choose_branch_var_pseudocost(const std::vector<int>& cand,
                                 const Eigen::VectorXd& x,
                                 const std::vector<PseudoCost>& pc,
                                 const std::vector<int>& priority,
                                 const BCBranchingPriorFn& dynamic_prior,
                                 const BCBranchContext& context) {
  if (cand.empty()) return -1;
  if (!dynamic_prior) return choose_branch_var_pseudocost(cand, x, pc, priority);
  BCOptions opt;
  opt.branching = BranchingStrategy::Pseudocost;
  return choose_branch_var_with_dynamic_prior(
      opt, cand, x, pc, priority, dynamic_prior, context);
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc) {
  if (cand.empty()) {
    return -1;
  }
  switch (opt.branching) {
    case BranchingStrategy::FirstFractional:
      return cand.front();
    case BranchingStrategy::MostInfeasible:
      return choose_branch_var_most_infeasible(cand, x);
    case BranchingStrategy::Pseudocost:
    default:
      return choose_branch_var_pseudocost(cand, x, pc);
  }
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority) {
  if (cand.empty()) {
    return -1;
  }
  switch (opt.branching) {
    case BranchingStrategy::FirstFractional:
      return cand.front();
    case BranchingStrategy::MostInfeasible:
      return choose_branch_var_most_infeasible(cand, x);
    case BranchingStrategy::Pseudocost:
    default:
      return choose_branch_var_pseudocost(cand, x, pc, priority);
  }
}

int choose_branch_var(const BCOptions& opt,
                      const std::vector<int>& cand,
                      const Eigen::VectorXd& x,
                      const std::vector<PseudoCost>& pc,
                      const std::vector<int>& priority,
                      const BCBranchingPriorFn& dynamic_prior,
                      const BCBranchContext& context) {
  if (cand.empty()) {
    return -1;
  }
  if (!dynamic_prior) {
    return choose_branch_var(opt, cand, x, pc, priority);
  }
  return choose_branch_var_with_dynamic_prior(
      opt, cand, x, pc, priority, dynamic_prior, context);
}

int choose_branch_var_reliability(
    const std::vector<int>& cand,
    const Eigen::VectorXd& x,
    std::vector<PseudoCost>& pc,
    const LPModel& base_lp,
    const StandardFormLP& base_sf,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const SimplexBasis* basis_hint,
    const SimplexOptions& simplex_opt,
    double parent_bound,
    int& lp_solves,
    int node_depth,
    int reliability_limit,
    int max_probes) {

  if (node_depth > 6) {
    return choose_branch_var_pseudocost(cand, x, pc);
  }

  // max_probes=0 means skip probing entirely (pure pseudocost).
  if (max_probes == 0) {
    return choose_branch_var_pseudocost(cand, x, pc);
  }

  // Scale probing budget with depth: aggressive at root, lighter deeper.
  const int effective_probes = (node_depth == 0) ? max_probes
                             : std::max(2, max_probes / 2);

  std::vector<int> unreliable;
  for (int j : cand) {
    // Variables with root probing data (both directions observed at least once)
    // need fewer additional observations — the root LP provides high-quality
    // pseudocost estimates.  Use an adaptive reliability threshold.
    const int limit = (pc[j].down_cnt >= 1 && pc[j].up_cnt >= 1)
                          ? std::max(2, reliability_limit / 2)
                          : reliability_limit;
    if (pc[j].down_cnt < limit || pc[j].up_cnt < limit) {
      unreliable.push_back(j);
    }
  }

  if (!unreliable.empty()) {
    std::sort(unreliable.begin(), unreliable.end(), [&x](int a, int b) {
      const double fa = std::abs(x[a] - std::floor(x[a]) - 0.5);
      const double fb = std::abs(x[b] - std::floor(x[b]) - 0.5);
      return fa < fb;
    });
    const int probes = std::min(effective_probes, static_cast<int>(unreliable.size()));
    StandardFormLP probe_sf = base_sf;

    for (int k = 0; k < probes; ++k) {
      const int j = unreliable[k];
      const double xj = x[j];
      const double floor_xj = std::floor(xj);

      // Probe down branch: ub[j] = floor(xj)
      {
        Eigen::VectorXd plb = node_lb, pub = node_ub;
        pub[j] = std::min(pub[j], floor_xj);
        if (bounds_consistent(plb, pub)) {
          update_standard_form_bounds(probe_sf, base_lp, plb, pub);
          auto res = solve_lp_from_sf(probe_sf, simplex_opt, basis_hint);
          ++lp_solves;
          if (res.result.stats.success) {
            const double gain = std::max(0.0, res.result.stats.objective - parent_bound);
            pc[j].add_down(gain);
          } else {
            const auto ft = classify_lp_result(res, static_cast<int>(base_lp.vars.size()));
            if (ft == LPFailureType::Infeasible) {
              pc[j].add_down_cutoff();
              if (res.result.stats.has_farkas_certificate) {
                pc[j].add_down_conflict();
              }
            }
            pc[j].add_down(1e6);
          }
        }
      }

      // Probe up branch: lb[j] = ceil(xj)
      {
        Eigen::VectorXd plb = node_lb, pub = node_ub;
        plb[j] = std::max(plb[j], floor_xj + 1.0);
        if (bounds_consistent(plb, pub)) {
          update_standard_form_bounds(probe_sf, base_lp, plb, pub);
          auto res = solve_lp_from_sf(probe_sf, simplex_opt, basis_hint);
          ++lp_solves;
          if (res.result.stats.success) {
            const double gain = std::max(0.0, res.result.stats.objective - parent_bound);
            pc[j].add_up(gain);
          } else {
            const auto ft = classify_lp_result(res, static_cast<int>(base_lp.vars.size()));
            if (ft == LPFailureType::Infeasible) {
              pc[j].add_up_cutoff();
              if (res.result.stats.has_farkas_certificate) {
                pc[j].add_up_conflict();
              }
            }
            pc[j].add_up(1e6);
          }
        }
      }
    }
  }

  return choose_branch_var_pseudocost(cand, x, pc);
}

double compute_node_estimate(const std::vector<VariableMeta>& vars,
                             const Eigen::VectorXd& x,
                             const std::vector<PseudoCost>& pc,
                             double int_tol,
                             double node_bound) {
  if (!std::isfinite(node_bound)) {
    return kInf;
  }

  std::vector<int> frac;
  if (!fractional_indices(vars, x, int_tol, frac)) {
    return node_bound;
  }

  const double offset = std::max(
      1e-6,
      int_tol * std::max(1.0, std::abs(node_bound)) /
          std::max(1, static_cast<int>(frac.size())));

  double estimate = node_bound;
  for (int j : frac) {
    const double frac_part = x[j] - std::floor(x[j]);
    const double down = directional_branch_cost(pc[j], frac_part, false, offset);
    const double up = directional_branch_cost(pc[j], 1.0 - frac_part, true, offset);
    estimate += std::min(down, up);
  }

  return std::max(node_bound, estimate);
}

}  // namespace mipsolvers::engine::detail
