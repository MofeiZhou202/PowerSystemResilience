/// @file bc_branching.cpp
/// @brief Branch variable selection strategies for the B&C solver.

#include "mipsolvers/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <cmath>
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
                               const std::vector<PseudoCost>& pc,
                               const std::vector<int>* priority = nullptr) {
  const double frac = x[j] - std::floor(x[j]);
  const double qd = directional_branch_cost(pc[j], frac, false);
  const double qu = directional_branch_cost(pc[j], 1.0 - frac, true);
  double score = qd * qu;

  if (priority) {
    const int pri = (j < static_cast<int>(priority->size())) ? (*priority)[j] : 0;
    if (pri > 0) score *= std::sqrt(static_cast<double>(pri + 1));
  }
  return score;
}

}  // namespace

double compute_branch_var_score(int j,
                                const Eigen::VectorXd& x,
                                const std::vector<PseudoCost>& pc,
                                const std::vector<int>* priority) {
  return pseudocost_branch_score(j, x, pc, priority);
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
  for (int j : cand) {
    const double score = pseudocost_branch_score(j, x, pc, &priority);
    if (score > best_score) {
      best_score = score;
      best = j;
    }
  }
  return best;
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
