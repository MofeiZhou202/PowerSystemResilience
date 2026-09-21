/// @file bc_utils_dual_proof_conflict.cpp
/// @brief Dual-proof conflict-clause and reduced-cost cutoff-clause resolution.
///
/// Extracted from bc_utils.cpp; declarations live in bc_utils.hpp.

#include "mipsolvers/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_numerics.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"

namespace mipsolvers::engine::detail {

DualProofResolutionStatus resolve_dual_proof_conflict_from_local_trail(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    double int_tol,
    double lp_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const Eigen::SparseVector<double>& coeff_sparse,
    double rhs,
    int max_literals,
    DualProofConflictResolution& out) {
  out = DualProofConflictResolution{};
  auto fail = [&](DualProofResolutionStatus status) {
    out.status = status;
    return status;
  };
  auto success = [&]() {
    out.status = DualProofResolutionStatus::Success;
    return DualProofResolutionStatus::Success;
  };
  const int n = static_cast<int>(vars.size());
  if (max_literals <= 0 || n <= 0 || coeff_sparse.size() < n ||
      coeff_sparse.nonZeros() <= 0 || !std::isfinite(rhs) ||
      root_lb.size() < n || root_ub.size() < n ||
      node_lb.size() < n || node_ub.size() < n) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);
  double root_min_activity = 0.0;
  double node_min_activity = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(coeff_sparse); it; ++it) {
    const int j = static_cast<int>(it.index());
    const double a = it.value();
    if (j < 0 || j >= n || !std::isfinite(a)) {
      return fail(DualProofResolutionStatus::InvalidInput);
    }
    coeff[static_cast<std::size_t>(j)] = a;
    if (a > 0.0) {
      if (!std::isfinite(root_lb[j]) || !std::isfinite(node_lb[j])) {
        return fail(DualProofResolutionStatus::InvalidInput);
      }
      root_min_activity += a * root_lb[j];
      node_min_activity += a * node_lb[j];
    } else if (a < 0.0) {
      if (!std::isfinite(root_ub[j]) || !std::isfinite(node_ub[j])) {
        return fail(DualProofResolutionStatus::InvalidInput);
      }
      root_min_activity += a * root_ub[j];
      node_min_activity += a * node_ub[j];
    }
  }
  if (!std::isfinite(root_min_activity) || !std::isfinite(node_min_activity)) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  const double tol = std::max({1e-9, int_tol, lp_tol});
  const double proof_tol = std::max(1e-7, tol * std::max(10.0, std::abs(rhs)));
  const double cutoff_rhs = rhs + proof_tol;
  if (node_min_activity < cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }
  if (root_min_activity >= cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  struct TrailCandidate {
    BranchDomainLiteral lit;
    std::vector<BranchDomainLiteral> reason;
    double delta{0.0};
    double base_bound{0.0};
    double priority{0.0};
    int trail_pos{-1};
    int prev_bound_pos{-1};
    bool is_branch{false};
  };

  const bool have_native_trail =
      local_domain_trail != nullptr && !local_domain_trail->empty();
  const int trail_end =
      have_native_trail
          ? static_cast<int>(local_domain_trail->size())
          : static_cast<int>(branch_reasons.size() + reason_bounds.size());

  auto proof_side_matches = [&](const BranchDomainLiteral& lit) {
    if (lit.var_idx < 0 || lit.var_idx >= n) return false;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    return (a > 0.0 && lit.is_lb) || (a < 0.0 && !lit.is_lb);
  };
  auto active_at_node = [&](const BranchDomainLiteral& lit) {
    if (lit.var_idx < 0 || lit.var_idx >= n) return false;
    return lit.is_lb ? (node_lb[lit.var_idx] >= lit.value - tol)
                     : (node_ub[lit.var_idx] <= lit.value + tol);
  };
  auto proof_side_stronger = [&](const BranchDomainLiteral& lhs,
                                 const BranchDomainLiteral& rhs_lit) {
    if (lhs.var_idx != rhs_lit.var_idx || lhs.is_lb != rhs_lit.is_lb) {
      return false;
    }
    return lhs.is_lb ? lhs.value > rhs_lit.value + tol
                     : lhs.value < rhs_lit.value - tol;
  };
  auto proof_side_same = [&](const BranchDomainLiteral& lhs,
                             const BranchDomainLiteral& rhs_lit) {
    return lhs.var_idx == rhs_lit.var_idx && lhs.is_lb == rhs_lit.is_lb &&
           std::abs(lhs.value - rhs_lit.value) <= tol;
  };
  auto proof_literal_delta = [&](const BranchDomainLiteral& lit,
                                 double* delta_out) -> bool {
    if (delta_out != nullptr) *delta_out = 0.0;
    if (lit.var_idx < 0 || lit.var_idx >= n ||
        !std::isfinite(lit.value)) {
      return false;
    }
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    if (a > 0.0 && lit.is_lb) {
      if (!std::isfinite(root_lb[lit.var_idx])) return false;
      const double delta = a * (lit.value - root_lb[lit.var_idx]);
      if (!std::isfinite(delta) || delta < -proof_tol) return false;
      if (delta_out != nullptr) *delta_out = std::max(0.0, delta);
      return true;
    }
    if (a < 0.0 && !lit.is_lb) {
      if (!std::isfinite(root_ub[lit.var_idx])) return false;
      const double delta = a * (lit.value - root_ub[lit.var_idx]);
      if (!std::isfinite(delta) || delta < -proof_tol) return false;
      if (delta_out != nullptr) *delta_out = std::max(0.0, delta);
      return true;
    }
    return false;
  };
  auto audited_activity_from_frontier =
      [&](const std::vector<BranchDomainLiteral>& frontier,
          double& audited_activity) -> bool {
    audited_activity = root_min_activity;
    std::vector<char> seen_lower(static_cast<std::size_t>(n), 0);
    std::vector<char> seen_upper(static_cast<std::size_t>(n), 0);
    for (const auto& lit : frontier) {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      char& seen = lit.is_lb ? seen_lower[static_cast<std::size_t>(lit.var_idx)]
                             : seen_upper[static_cast<std::size_t>(lit.var_idx)];
      if (seen != 0) return false;
      seen = 1;
      double delta = 0.0;
      if (!proof_literal_delta(lit, &delta)) return false;
      audited_activity += delta;
    }
    return std::isfinite(audited_activity);
  };
  auto frontier_implies_literals =
      [&](const std::vector<BranchDomainLiteral>& reason_frontier,
          const std::vector<BranchDomainLiteral>& needed_literals) -> bool {
    if (needed_literals.empty()) return false;
    Eigen::VectorXd closure_lb = root_lb;
    Eigen::VectorXd closure_ub = root_ub;
    auto apply_lit = [&](const BranchDomainLiteral& lit) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      if (lit.is_lb) {
        if (lit.value > closure_ub[lit.var_idx] + tol) return false;
        if (lit.value > closure_lb[lit.var_idx]) {
          closure_lb[lit.var_idx] = lit.value;
        }
      } else {
        if (lit.value < closure_lb[lit.var_idx] - tol) return false;
        if (lit.value < closure_ub[lit.var_idx]) {
          closure_ub[lit.var_idx] = lit.value;
        }
      }
      return true;
    };
    auto active_lit = [&](const BranchDomainLiteral& lit) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      return lit.is_lb ? closure_lb[lit.var_idx] >= lit.value - tol
                       : closure_ub[lit.var_idx] <= lit.value + tol;
    };
    for (const auto& lit : reason_frontier) {
      if (!apply_lit(lit)) return false;
    }
    if (have_native_trail) {
      bool changed = true;
      for (int pass = 0; pass < trail_end + 1 && changed; ++pass) {
        changed = false;
        for (const auto& entry : *local_domain_trail) {
          if (entry.pos < 0 || entry.pos >= trail_end ||
              entry.reason.empty()) {
            continue;
          }
          bool reason_active = true;
          for (const auto& lit : entry.reason) {
            if (!active_lit(lit)) {
              reason_active = false;
              break;
            }
          }
          if (!reason_active) continue;
          const double old_lb = closure_lb[entry.bound.var_idx];
          const double old_ub = closure_ub[entry.bound.var_idx];
          if (!apply_lit(entry.bound)) return false;
          if (closure_lb[entry.bound.var_idx] > old_lb + tol ||
              closure_ub[entry.bound.var_idx] < old_ub - tol) {
            changed = true;
          }
        }
      }
    }
    for (const auto& lit : needed_literals) {
      if (!active_lit(lit)) return false;
    }
    return true;
  };
  auto frontier_implies_proof_frontier =
      [&](const std::vector<BranchDomainLiteral>& reason_frontier,
          const std::vector<BranchDomainLiteral>& needed_proof_frontier,
          double& audited_activity) -> bool {
    if (audited_activity_from_frontier(reason_frontier, audited_activity)) {
      if (audited_activity >= cutoff_rhs - proof_tol) {
        return true;
      }
    }
    if (!frontier_implies_literals(reason_frontier, needed_proof_frontier)) {
      return false;
    }
    audited_activity = root_min_activity;
    for (const auto& lit : needed_proof_frontier) {
      double delta = 0.0;
      if (!proof_literal_delta(lit, &delta)) return false;
      audited_activity += delta;
    }
    return std::isfinite(audited_activity) &&
           audited_activity >= cutoff_rhs - proof_tol;
  };
  std::vector<TrailCandidate> candidates;
  candidates.reserve(static_cast<std::size_t>(coeff_sparse.nonZeros()));
  std::vector<int> candidate_by_col(static_cast<std::size_t>(n), -1);
  auto add_or_replace_candidate =
      [&](BranchDomainLiteral lit, std::vector<BranchDomainLiteral> reason,
          int trail_pos, int prev_bound_pos, bool is_branch) {
    const int j = lit.var_idx;
    if (j < 0 || j >= n || !std::isfinite(lit.value) ||
        trail_pos < 0 || trail_pos >= trail_end || !proof_side_matches(lit) ||
        !active_at_node(lit)) {
      return;
    }
    const double a = coeff[static_cast<std::size_t>(j)];
    const double base = lit.is_lb ? root_lb[j] : root_ub[j];
    if (!std::isfinite(base)) return;
    double delta = 0.0;
    if (lit.is_lb) {
      if (lit.value <= base + tol) return;
      delta = a * (lit.value - base);
    } else {
      if (lit.value >= base - tol) return;
      delta = a * (lit.value - base);
    }
    if (!std::isfinite(delta) || delta <= proof_tol * 1e-4) return;
    if (reason.empty()) reason.push_back(lit);
    canonicalize_branch_literals(reason);
    TrailCandidate cand{lit,
                        std::move(reason),
                        delta,
                        base,
                        std::abs(delta),
                        trail_pos,
                        prev_bound_pos,
                        is_branch};
    int& slot = candidate_by_col[static_cast<std::size_t>(j)];
    if (slot < 0) {
      slot = static_cast<int>(candidates.size());
      candidates.push_back(std::move(cand));
      return;
    }
    TrailCandidate& cur = candidates[static_cast<std::size_t>(slot)];
    if (proof_side_stronger(cand.lit, cur.lit) ||
        (proof_side_same(cand.lit, cur.lit) && cand.trail_pos > cur.trail_pos)) {
      cur = std::move(cand);
    }
  };

  if (have_native_trail) {
    for (const auto& entry : *local_domain_trail) {
      add_or_replace_candidate(entry.bound, entry.reason, entry.pos,
                               entry.prev_bound_pos, entry.is_branch);
    }
  } else {
    for (int pos = 0; pos < static_cast<int>(branch_reasons.size()); ++pos) {
      const auto& lit = branch_reasons[static_cast<std::size_t>(pos)];
      add_or_replace_candidate(lit, {lit}, pos, -1, true);
    }
    for (const auto& rb : reason_bounds) {
      add_or_replace_candidate(rb.bound, rb.reason, rb.trail_pos,
                               rb.prev_bound_pos, false);
    }
  }
  if (candidates.empty()) {
    return fail(DualProofResolutionStatus::MissingReason);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const TrailCandidate& a, const TrailCandidate& b) {
              if (a.priority != b.priority) return a.priority > b.priority;
              if (a.trail_pos != b.trail_pos) return a.trail_pos < b.trail_pos;
              if (a.delta != b.delta) return a.delta > b.delta;
              return a.reason.size() < b.reason.size();
            });

  double activity = root_min_activity;
  std::vector<TrailCandidate> selected;
  selected.reserve(candidates.size());
  for (const auto& cand : candidates) {
    selected.push_back(cand);
    activity += cand.delta;
    if (activity >= cutoff_rhs) break;
  }
  if (selected.empty() || activity < cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  for (int k = static_cast<int>(selected.size()) - 1; k >= 0; --k) {
    TrailCandidate& cand = selected[static_cast<std::size_t>(k)];
    const int j = cand.lit.var_idx;
    const double a = coeff[static_cast<std::size_t>(j)];
    const double without = activity - cand.delta;
    if (cand.lit.is_lb) {
      if (a <= 0.0) continue;
      double relaxed = (cutoff_rhs - without) / a + cand.base_bound;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        relaxed = std::ceil(relaxed - tol);
      }
      if (!std::isfinite(relaxed) || relaxed >= cand.lit.value - proof_tol) {
        continue;
      }
      if (relaxed <= cand.base_bound + tol) {
        activity = without;
        selected.erase(selected.begin() + k);
      } else {
        if (have_native_trail) {
          int p = cand.prev_bound_pos;
          while (p >= 0 && p < static_cast<int>(local_domain_trail->size()) &&
                 relaxed <= (*local_domain_trail)[static_cast<std::size_t>(p)]
                                    .bound.value +
                                tol) {
            cand.trail_pos = p;
            cand.prev_bound_pos =
                (*local_domain_trail)[static_cast<std::size_t>(p)].prev_bound_pos;
            p = cand.prev_bound_pos;
          }
        }
        activity += a * (relaxed - cand.lit.value);
        cand.lit.value = relaxed;
        cand.delta = a * (cand.lit.value - cand.base_bound);
        if (cand.is_branch) cand.reason = {cand.lit};
      }
    } else {
      if (a >= 0.0) continue;
      double relaxed = (cutoff_rhs - without) / a + cand.base_bound;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        relaxed = std::floor(relaxed + tol);
      }
      if (!std::isfinite(relaxed) || relaxed <= cand.lit.value + proof_tol) {
        continue;
      }
      if (relaxed >= cand.base_bound - tol) {
        activity = without;
        selected.erase(selected.begin() + k);
      } else {
        if (have_native_trail) {
          int p = cand.prev_bound_pos;
          while (p >= 0 && p < static_cast<int>(local_domain_trail->size()) &&
                 relaxed >= (*local_domain_trail)[static_cast<std::size_t>(p)]
                                    .bound.value -
                                tol) {
            cand.trail_pos = p;
            cand.prev_bound_pos =
                (*local_domain_trail)[static_cast<std::size_t>(p)].prev_bound_pos;
            p = cand.prev_bound_pos;
          }
        }
        activity += a * (relaxed - cand.lit.value);
        cand.lit.value = relaxed;
        cand.delta = a * (cand.lit.value - cand.base_bound);
        if (cand.is_branch) cand.reason = {cand.lit};
      }
    }
    if (activity <= cutoff_rhs + proof_tol) break;
  }
  if (selected.empty() || activity < cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  int integral_cols = 0;
  for (const auto& var : vars) {
    if (is_integer_type(var)) ++integral_cols;
  }
  if (10 * static_cast<int>(selected.size()) > 1000 + 3 * integral_cols ||
      static_cast<int>(selected.size()) > max_literals) {
    return fail(DualProofResolutionStatus::LiteralLimit);
  }

  std::vector<BranchDomainLiteral> proof_frontier;
  proof_frontier.reserve(selected.size());
  for (const auto& cand : selected) proof_frontier.push_back(cand.lit);
  canonicalize_branch_literals(proof_frontier);
  if (proof_frontier.empty() ||
      static_cast<int>(proof_frontier.size()) > max_literals) {
    return fail(proof_frontier.empty()
                    ? DualProofResolutionStatus::ActivityFailed
                    : DualProofResolutionStatus::LiteralLimit);
  }
  double proof_frontier_activity = 0.0;
  if (!audited_activity_from_frontier(proof_frontier,
                                      proof_frontier_activity) ||
      proof_frontier_activity < cutoff_rhs - proof_tol) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  struct FrontierEntry {
    BranchDomainLiteral lit;
    std::vector<BranchDomainLiteral> reason;
    int trail_pos{-1};
    bool is_branch{false};
  };

  auto compact_frontier = [&](std::vector<FrontierEntry>& frontier) {
    std::vector<FrontierEntry> compact;
    compact.reserve(frontier.size());
    for (auto& entry : frontier) {
      if (entry.lit.var_idx < 0 || entry.lit.var_idx >= n ||
          !std::isfinite(entry.lit.value)) {
        continue;
      }
      bool merged = false;
      for (auto& cur : compact) {
        if (cur.lit.var_idx != entry.lit.var_idx ||
            cur.lit.is_lb != entry.lit.is_lb) {
          continue;
        }
        const bool replace =
            proof_side_stronger(entry.lit, cur.lit) ||
            (proof_side_same(entry.lit, cur.lit) &&
             entry.trail_pos > cur.trail_pos);
        if (replace) cur = std::move(entry);
        merged = true;
        break;
      }
      if (!merged) compact.push_back(std::move(entry));
    }
    frontier.swap(compact);
  };

  auto find_prior_entry =
      [&](const BranchDomainLiteral& lit,
          int before_pos) -> const LocalDomainTrailEntry* {
    if (!have_native_trail || lit.var_idx < 0 || lit.var_idx >= n ||
        before_pos <= 0) {
      return nullptr;
    }
    const LocalDomainTrailEntry* best = nullptr;
    for (const auto& entry : *local_domain_trail) {
      if (entry.pos < 0 || entry.pos >= before_pos ||
          entry.bound.var_idx != lit.var_idx ||
          entry.bound.is_lb != lit.is_lb) {
        continue;
      }
      if (!branch_literal_stronger_or_equal(entry.bound, lit, tol)) continue;
      if (best == nullptr || entry.pos > best->pos) best = &entry;
    }
    return best;
  };

  auto fallback_literal_pos = [&](const BranchDomainLiteral& lit) -> int {
    int best = -1;
    if (have_native_trail) {
      for (const auto& entry : *local_domain_trail) {
        if (entry.bound.var_idx == lit.var_idx &&
            entry.bound.is_lb == lit.is_lb &&
            branch_literal_stronger_or_equal(entry.bound, lit, tol)) {
          best = std::max(best, entry.pos);
        }
      }
    } else {
      for (int i = 0; i < static_cast<int>(branch_reasons.size()); ++i) {
        if (branch_literal_stronger_or_equal(
                branch_reasons[static_cast<std::size_t>(i)], lit, tol)) {
          best = std::max(best, i);
        }
      }
      for (const auto& rb : reason_bounds) {
        if (branch_literal_stronger_or_equal(rb.bound, lit, tol)) {
          best = std::max(best, rb.trail_pos);
        }
      }
    }
    return best;
  };

  auto entry_for_literal = [&](const BranchDomainLiteral& lit,
                               int before_pos) -> FrontierEntry {
    const LocalDomainTrailEntry* prior = find_prior_entry(lit, before_pos);
    if (prior != nullptr) {
      return FrontierEntry{
          lit,
          prior->reason.empty()
              ? std::vector<BranchDomainLiteral>{lit}
              : prior->reason,
          prior->pos,
          prior->is_branch};
    }
    return FrontierEntry{lit, {lit}, fallback_literal_pos(lit), true};
  };

  auto flip_literal_for_conflict =
      [&](const BranchDomainLiteral& lit, BranchDomainLiteral& flipped) -> bool {
    if (lit.var_idx < 0 || lit.var_idx >= n || !std::isfinite(lit.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
    if (lit.is_lb) {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::ceil(lit.value - tol) - 1.0
                      : lit.value - std::max(1e-7, 10.0 * tol),
          false};
    } else {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::floor(lit.value + tol) + 1.0
                      : lit.value + std::max(1e-7, 10.0 * tol),
          true};
    }
    return true;
  };

  auto source_conflict_matches_flipped =
      [&](const BranchDomainLiteral& conflict_lit,
          const BranchDomainLiteral& flipped) {
    if (conflict_lit.var_idx != flipped.var_idx ||
        conflict_lit.is_lb != flipped.is_lb) {
      return false;
    }
    return conflict_lit.is_lb
               ? conflict_lit.value <= flipped.value + tol
               : conflict_lit.value >= flipped.value - tol;
  };

  auto explain_conflict_reason_frontier =
      [&](const LocalDomainTrailEntry& entry,
          std::vector<FrontierEntry>& replacement) -> bool {
    replacement.clear();
    if (!have_native_trail || !entry.has_source_conflict_clause ||
        entry.source_conflict_clause.empty() || entry.pos <= 0) {
      return false;
    }
    BranchDomainLiteral flipped_entry;
    if (!flip_literal_for_conflict(entry.bound, flipped_entry)) return false;
    bool found_flipped = false;
    for (const auto& conflict_lit : entry.source_conflict_clause) {
      if (!found_flipped &&
          source_conflict_matches_flipped(conflict_lit, flipped_entry)) {
        found_flipped = true;
        continue;
      }
      if (conflict_lit.var_idx < 0 || conflict_lit.var_idx >= n ||
          !std::isfinite(conflict_lit.value)) {
        return false;
      }
      const double global_bound =
          conflict_lit.is_lb ? root_lb[conflict_lit.var_idx]
                             : root_ub[conflict_lit.var_idx];
      if (std::isfinite(global_bound)) {
        const bool globally_active =
            conflict_lit.is_lb
                ? global_bound >= conflict_lit.value - tol
                : global_bound <= conflict_lit.value + tol;
        if (globally_active) continue;
      }
      const LocalDomainTrailEntry* prior =
          find_prior_entry(conflict_lit, entry.pos);
      if (prior == nullptr) return false;
      LocalDomainTrailEntry relaxed = *prior;
      while (relaxed.prev_bound_pos >= 0 &&
             relaxed.prev_bound_pos <
                 static_cast<int>(local_domain_trail->size())) {
        const auto& prev =
            (*local_domain_trail)[static_cast<std::size_t>(
                relaxed.prev_bound_pos)];
        const bool prev_still_active =
            conflict_lit.is_lb
                ? prev.bound.value >= conflict_lit.value - tol
                : prev.bound.value <= conflict_lit.value + tol;
        if (!prev_still_active) break;
        relaxed = prev;
      }
      replacement.push_back(FrontierEntry{
          conflict_lit,
          relaxed.reason.empty() && relaxed.is_branch
              ? std::vector<BranchDomainLiteral>{conflict_lit}
              : relaxed.reason,
          relaxed.pos,
          relaxed.is_branch});
    }
    return found_flipped;
  };

  auto frontier_literals = [&](const std::vector<FrontierEntry>& frontier) {
    std::vector<BranchDomainLiteral> lits;
    lits.reserve(frontier.size());
    for (const auto& entry : frontier) lits.push_back(entry.lit);
    canonicalize_branch_literals(lits);
    return lits;
  };

  auto resolve_reason_frontier = [&](std::vector<FrontierEntry> frontier,
                                     int stop_size,
                                     int min_resolve) {
    if (!have_native_trail) return frontier_literals(frontier);
    compact_frontier(frontier);
    int depth = local_branch_positions == nullptr
                    ? 0
                    : static_cast<int>(local_branch_positions->size());
    while (depth > 0) {
      const int branch_pos =
          (*local_branch_positions)[static_cast<std::size_t>(depth - 1)];
      if (branch_pos < 0 ||
          branch_pos >= static_cast<int>(local_domain_trail->size())) {
        break;
      }
      const auto& branch_entry =
          (*local_domain_trail)[static_cast<std::size_t>(branch_pos)];
      if (std::abs(branch_entry.bound.value - branch_entry.prev_bound_value) >
          tol) {
        break;
      }
      --depth;
    }
    const int start_pos =
        depth <= 0 || local_branch_positions == nullptr ||
                local_branch_positions->empty()
            ? 0
            : (*local_branch_positions)[static_cast<std::size_t>(depth - 1)] + 1;
    const int max_resolves = std::min<int>(
        4096, static_cast<int>(local_domain_trail->size()) +
                  4 * std::max(1, max_literals));
    std::vector<BranchDomainLiteral> best = frontier_literals(frontier);
	    std::unordered_set<int> skipped_positions;
	    struct ResolutionState {
	      std::vector<BranchDomainLiteral> literals;
	      std::vector<int> trail_positions;
	      std::vector<int> skipped_positions;
	    };
	    auto state_hash = [](const ResolutionState& state) {
	      std::size_t hash = conflict_clause_hash(state.literals);
	      for (int pos : state.trail_positions) {
	        hash ^= std::hash<int>{}(pos) + 0x9e3779b97f4a7c15ULL +
	                (hash << 6) + (hash >> 2);
	      }
	      for (int pos : state.skipped_positions) {
	        hash ^= std::hash<int>{}(pos) + 0x9e3779b97f4a7c15ULL +
	                (hash << 6) + (hash >> 2);
	      }
	      return hash;
	    };
	    auto states_equal = [](const ResolutionState& lhs,
	                           const ResolutionState& rhs) {
	      return branch_literal_lists_equal(lhs.literals, rhs.literals, 0.0) &&
	             lhs.trail_positions == rhs.trail_positions &&
	             lhs.skipped_positions == rhs.skipped_positions;
	    };
	    HashBucketExactSet<ResolutionState, decltype(state_hash),
	                       decltype(states_equal)>
	        seen(state_hash, states_equal);
	    auto current_state = [&]() {
	      ResolutionState state;
	      state.literals.reserve(frontier.size());
	      state.trail_positions.reserve(frontier.size());
	      for (const auto& entry : frontier) {
	        state.literals.push_back(entry.lit);
	        state.trail_positions.push_back(entry.trail_pos);
	      }
	      state.skipped_positions.assign(skipped_positions.begin(),
	                                     skipped_positions.end());
	      std::sort(state.skipped_positions.begin(),
	                state.skipped_positions.end());
	      return state;
	    };
	    int resolved = 0;
	    for (int iter = 0; iter < max_resolves; ++iter) {
	      std::vector<BranchDomainLiteral> sig = frontier_literals(frontier);
	      if (!seen.insert(current_state())) break;
	      if (!sig.empty() && static_cast<int>(sig.size()) <= max_literals &&
	          (best.empty() || sig.size() < best.size())) {
        best = sig;
      }

      int latest_idx = -1;
      int latest_pos = -1;
      int resolvable = 0;
      for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
        const int pos = frontier[static_cast<std::size_t>(i)].trail_pos;
	        if (pos < start_pos || pos < 0 ||
	            pos >= static_cast<int>(local_domain_trail->size())) {
	          continue;
	        }
	        if (skipped_positions.count(pos) != 0) continue;
	        const auto& entry =
	            (*local_domain_trail)[static_cast<std::size_t>(pos)];
	        if (entry.is_branch || entry.reason.empty()) continue;
        ++resolvable;
        if (pos > latest_pos) {
          latest_pos = pos;
          latest_idx = i;
        }
      }
      if (latest_idx < 0 ||
          (static_cast<int>(frontier.size()) <= stop_size &&
           resolved >= min_resolve && resolvable <= stop_size)) {
        break;
	      }
	      const auto& entry =
	          (*local_domain_trail)[static_cast<std::size_t>(latest_pos)];
	      auto covered_by_current_frontier =
	          [&](const BranchDomainLiteral& lit) {
	        for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
	          if (i == latest_idx) continue;
	          const auto& cur = frontier[static_cast<std::size_t>(i)].lit;
	          if (cur.var_idx != lit.var_idx || cur.is_lb != lit.is_lb) {
	            continue;
	          }
	          if (branch_literal_stronger_or_equal(cur, lit, tol)) {
	            return true;
	          }
	        }
	        return false;
	      };
	      std::vector<FrontierEntry> replacement;
	      bool replacement_valid =
	          explain_conflict_reason_frontier(entry, replacement);
	      if (replacement_valid) {
	        replacement.erase(
	            std::remove_if(
	                replacement.begin(), replacement.end(),
	                [&](const FrontierEntry& repl) {
	                  return covered_by_current_frontier(repl.lit);
	                }),
	            replacement.end());
	      } else {
	        replacement.clear();
	        replacement.reserve(entry.reason.size());
	        replacement_valid = true;
	        for (const auto& reason_lit : entry.reason) {
	          if (reason_lit.var_idx < 0 || reason_lit.var_idx >= n ||
	              !std::isfinite(reason_lit.value)) {
	            replacement_valid = false;
	            break;
	          }
	          if (covered_by_current_frontier(reason_lit)) continue;
	          replacement.push_back(entry_for_literal(reason_lit, latest_pos));
	        }
	      }
	      if (!replacement_valid) {
	        skipped_positions.insert(latest_pos);
	        continue;
	      }
	      frontier.erase(frontier.begin() + latest_idx);
	      if (!replacement.empty()) {
	        frontier.insert(frontier.end(),
	                        std::make_move_iterator(replacement.begin()),
	                        std::make_move_iterator(replacement.end()));
	      }
	      compact_frontier(frontier);
	      ++resolved;
      std::vector<BranchDomainLiteral> cur = frontier_literals(frontier);
      if (!cur.empty() && static_cast<int>(cur.size()) <= max_literals &&
          (best.empty() || cur.size() < best.size())) {
        best = cur;
      }
      if (static_cast<int>(cur.size()) <= stop_size &&
          resolved >= min_resolve) {
        break;
      }
    }
    return best;
  };

  std::vector<FrontierEntry> initial_frontier;
  initial_frontier.reserve(selected.size());
  for (const auto& cand : selected) {
    initial_frontier.push_back(
        FrontierEntry{cand.lit, cand.reason, cand.trail_pos, cand.is_branch});
  }
  std::vector<BranchDomainLiteral> resolved_frontier =
      resolve_reason_frontier(initial_frontier, /*stop_size=*/1,
                              /*min_resolve=*/1);
  canonicalize_branch_literals(resolved_frontier);
  if (resolved_frontier.empty() ||
      static_cast<int>(resolved_frontier.size()) > max_literals) {
    return fail(resolved_frontier.empty()
                    ? DualProofResolutionStatus::ScopeBlocked
                    : DualProofResolutionStatus::LiteralLimit);
  }
  double resolved_frontier_activity = 0.0;
  if (!frontier_implies_proof_frontier(resolved_frontier, proof_frontier,
                                       resolved_frontier_activity)) {
    return fail(have_native_trail ? DualProofResolutionStatus::ScopeBlocked
                                  : DualProofResolutionStatus::MissingReason);
  }

  auto flip_literal = [&](const BranchDomainLiteral& lit,
                          BranchDomainLiteral& flipped) -> bool {
    if (lit.var_idx < 0 || lit.var_idx >= n || !std::isfinite(lit.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
    if (lit.is_lb) {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::ceil(lit.value - tol) - 1.0
                      : lit.value - std::max(1e-7, 10.0 * tol),
          false};
    } else {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::floor(lit.value + tol) + 1.0
                      : lit.value + std::max(1e-7, 10.0 * tol),
          true};
    }
    return true;
  };

  BranchLiteralListSet seen_clauses;
  auto add_clause = [&](std::vector<BranchDomainLiteral> clause) {
    canonicalize_branch_literals(clause);
    if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
      return;
    }
    if (!seen_clauses.insert(clause)) return;
    out.conflict_clauses.push_back(std::move(clause));
  };

  if (out.conflict_clauses.empty()) {
    for (const auto& cand : selected) {
      if (cand.is_branch || cand.reason.empty()) continue;
      BranchDomainLiteral flipped;
      if (!flip_literal(cand.lit, flipped)) continue;
      std::vector<FrontierEntry> reason_frontier;
      reason_frontier.reserve(cand.reason.size());
      for (const auto& reason_lit : cand.reason) {
        reason_frontier.push_back(entry_for_literal(reason_lit, cand.trail_pos));
      }
      std::vector<BranchDomainLiteral> reconv =
          resolve_reason_frontier(std::move(reason_frontier), /*stop_size=*/0,
                                  /*min_resolve=*/0);
      if (!frontier_implies_literals(reconv, {cand.lit})) continue;
      reconv.push_back(flipped);
      add_clause(std::move(reconv));
    }
  }

  if (out.conflict_clauses.empty()) {
    add_clause(resolved_frontier);
    if (!branch_literal_lists_equal(proof_frontier, resolved_frontier)) {
      add_clause(proof_frontier);
    }
  }
  if (out.conflict_clauses.empty()) {
    return fail(DualProofResolutionStatus::ScopeBlocked);
  }

  out.valid = true;
  out.proof_frontier = std::move(proof_frontier);
  out.resolved_frontier = std::move(resolved_frontier);
  out.root_min_activity = root_min_activity;
  out.node_min_activity = node_min_activity;
  out.cutoff_rhs = cutoff_rhs;
  out.proof_margin = resolved_frontier_activity - cutoff_rhs;
  out.proof_activity = proof_frontier_activity;
  out.resolved_activity = resolved_frontier_activity;
  return success();
}

bool build_dual_proof_target_bound_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_relax,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const DomainReasonBound& target_bound,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin,
    PoolCut* violated_local_proof_cover,
    const DualProofRow* dual_proof,
    DualProofTargetExplanation* explanation,
    std::vector<PoolCut>* additional_local_proof_covers,
    const DualProofRow* priority_proof) {
  out_clause.clear();
  if (proof_margin != nullptr) *proof_margin = 0.0;
  if (violated_local_proof_cover != nullptr) {
    *violated_local_proof_cover = PoolCut{};
  }
  if (additional_local_proof_covers != nullptr) {
    additional_local_proof_covers->clear();
  }
  if (explanation != nullptr) *explanation = DualProofTargetExplanation{};
  const int n = static_cast<int>(vars.size());
  if (max_literals <= 0 || n <= 0 || dual_proof == nullptr ||
      !dual_proof->valid || dual_proof->coeff.size() < n ||
      !std::isfinite(dual_proof->rhs) ||
      root_lb.size() < n || root_ub.size() < n ||
      x_relax.size() < n) {
    return false;
  }
  const BranchDomainLiteral& target_lit = target_bound.bound;
  if (target_lit.var_idx < 0 || target_lit.var_idx >= n ||
      !std::isfinite(target_lit.value)) {
    return false;
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);
  double root_min_activity = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(dual_proof->coeff); it;
       ++it) {
    const int j = static_cast<int>(it.index());
    const double a = it.value();
    if (j < 0 || j >= n || !std::isfinite(a)) continue;
    coeff[static_cast<std::size_t>(j)] = a;
    if (a > 0.0) {
      if (!std::isfinite(root_lb[j])) return false;
      root_min_activity += a * root_lb[j];
    } else if (a < 0.0) {
      if (!std::isfinite(root_ub[j])) return false;
      root_min_activity += a * root_ub[j];
    }
  }

  DualProofTrailResolution trail_resolution;
  const Eigen::SparseVector<double>* priority_coeff_ptr =
      priority_proof != nullptr && priority_proof->valid &&
              priority_proof->coeff.size() >= n
          ? &priority_proof->coeff
          : nullptr;
  const DualProofResolutionStatus trail_status =
      resolve_dual_proof_target_bound_from_local_trail(
          vars, root_lb, root_ub, &x_relax, int_tol, branch_reasons,
          reason_bounds, local_domain_trail, local_branch_positions,
          target_bound, max_literals, dual_proof->coeff, dual_proof->rhs,
          trail_resolution, priority_coeff_ptr);
  if (dual_proof_resolution_success(trail_status) && trail_resolution.valid) {
    double best_violation = 0.0;
    if (violated_local_proof_cover != nullptr) {
      struct ProofTerm {
        BranchDomainLiteral lit;
        double coeff{0.0};
        double root_bound{0.0};
        double lp_activity{0.0};
      };
      auto make_proof_term = [&](const BranchDomainLiteral& lit,
                                 ProofTerm& term) -> bool {
        if (lit.var_idx < 0 || lit.var_idx >= n) return false;
        const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
        if (a > 0.0 && lit.is_lb) {
          if (!std::isfinite(root_lb[lit.var_idx])) return false;
          term = ProofTerm{lit, a, root_lb[lit.var_idx],
                           a * (x_relax[lit.var_idx] - root_lb[lit.var_idx])};
          return true;
        }
        if (a < 0.0 && !lit.is_lb) {
          if (!std::isfinite(root_ub[lit.var_idx])) return false;
          term = ProofTerm{lit, a, root_ub[lit.var_idx],
                           a * (x_relax[lit.var_idx] - root_ub[lit.var_idx])};
          return true;
        }
        return false;
      };
      std::vector<ProofTerm> terms;
      terms.reserve(trail_resolution.proof_frontier.size() + 1);
      std::vector<char> used_var(static_cast<std::size_t>(n), 0);
      auto add_term = [&](const BranchDomainLiteral& lit) {
        ProofTerm term;
        if (!make_proof_term(lit, term)) return false;
        if (used_var[static_cast<std::size_t>(lit.var_idx)] == 0) {
          used_var[static_cast<std::size_t>(lit.var_idx)] = 1;
          terms.push_back(term);
        }
        return true;
      };
      bool cover_valid =
          trail_resolution.cutoff_conflict ||
          add_term(trail_resolution.flipped_target);
      for (const auto& lit : trail_resolution.proof_frontier) {
        cover_valid = add_term(lit) && cover_valid;
      }
      if (cover_valid && !terms.empty()) {
        Eigen::SparseVector<double> cover_coeff(n);
        cover_coeff.reserve(static_cast<int>(terms.size()));
        double cover_rhs = dual_proof->rhs - root_min_activity;
        double norm2 = 0.0;
        for (const auto& term : terms) {
          cover_coeff.coeffRef(term.lit.var_idx) += term.coeff;
          cover_rhs += term.coeff * term.root_bound;
          norm2 += term.coeff * term.coeff;
        }
        // prune(scalar, epsilon) overload is unambiguous across Eigen versions;
        // prune(0.0) alone is mis-deduced as a functor in Eigen >= 3.4.
        cover_coeff.prune(0.0, 0.0);
        const double lhs = cover_coeff.dot(x_relax);
        const double scale = 1.0 + std::abs(lhs) + std::abs(cover_rhs);
        const double cut_tol = std::max(1e-7, 1e-10 * scale);
        if (lhs > cover_rhs + cut_tol) {
          best_violation = lhs - cover_rhs;
          *violated_local_proof_cover =
              PoolCut{std::move(cover_coeff), cover_rhs, 0, 0.0,
                      std::sqrt(norm2), 0};
        }
      }
    }

    if (proof_margin != nullptr) {
      *proof_margin = trail_resolution.proof_margin;
    }
    if (explanation != nullptr) {
      explanation->valid = true;
      explanation->cutoff_conflict = trail_resolution.cutoff_conflict;
      explanation->has_proved_target_bound =
          trail_resolution.has_proved_target_bound;
      explanation->proved_target_bound =
          trail_resolution.proved_target_bound;
      explanation->flipped_target = trail_resolution.flipped_target;
      explanation->initial_frontier = trail_resolution.proof_frontier;
      explanation->resolved_frontier = trail_resolution.resolved_frontier;
      explanation->clause = trail_resolution.clause;
      explanation->proof_margin = trail_resolution.proof_margin;
      explanation->proof_budget = trail_resolution.proof_budget;
      explanation->frontier_lp_activity =
          std::max(0.0, trail_resolution.resolved_activity -
                            (dual_proof->rhs -
                             trail_resolution.proof_budget));
      explanation->local_proof_cover_violation = best_violation;
      explanation->local_proof_cover_nnz =
          violated_local_proof_cover != nullptr &&
                  violated_local_proof_cover->coeff.size() == n
              ? violated_local_proof_cover->coeff.nonZeros()
              : 0;
      explanation->has_proof_activity_audit = true;
      explanation->proof_activity = trail_resolution.resolved_activity;
      explanation->proof_required_activity =
          trail_resolution.resolved_activity - trail_resolution.proof_margin;
    }
	    out_clause = trail_resolution.clause;
	    return true;
	  }

	  // Cross-frontier target-bound proofs are valid only when the
  // first-class local trail resolver produced the resolved reason-side
  // frontier.  Legacy unordered candidate fallbacks are deliberately absent:
  // they can publish a different frontier than the local trail actually
  // resolves to, which breaks HiGHS-style proof consumption.
  return false;

}

bool build_reduced_cost_cutoff_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& reduced_costs,
    const std::vector<int>& basis_indices,
    double node_bound,
    double incumbent_obj,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const BranchDomainLiteral& forbidden,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin,
    std::vector<std::vector<BranchDomainLiteral>>* reconvergence_clauses,
    PoolCut* violated_local_proof_cover,
    const DualProofRow* dual_proof) {
  out_clause.clear();
  if (reconvergence_clauses != nullptr) reconvergence_clauses->clear();
  if (violated_local_proof_cover != nullptr) *violated_local_proof_cover = PoolCut{};
  if (max_literals <= 0) return false;
  const int n = static_cast<int>(vars.size());
  if (n <= 0 || root_lb.size() < n || root_ub.size() < n ||
      node_lb.size() < n || node_ub.size() < n ||
      x_relax.size() < n || reduced_costs.size() < n) {
    return false;
  }
  if (forbidden.var_idx < 0 || forbidden.var_idx >= n) return false;
  const bool use_dual_proof =
      dual_proof != nullptr && dual_proof->valid &&
      dual_proof->coeff.size() >= n && std::isfinite(dual_proof->rhs);
  const double gap = incumbent_obj - node_bound;
  if (!use_dual_proof && (!std::isfinite(gap) || gap <= 1e-12)) return false;

  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int idx : basis_indices) {
    if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);

  if (use_dual_proof) {
    for (Eigen::SparseVector<double>::InnerIterator it(dual_proof->coeff); it; ++it) {
      const int j = static_cast<int>(it.index());
      if (j >= 0 && j < n && std::isfinite(it.value())) {
        coeff[static_cast<std::size_t>(j)] = it.value();
      }
    }
  } else {
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<std::size_t>(j)]) continue;
      const double rc_abs = std::abs(reduced_costs[j]);
      if (rc_abs <= 1e-12) continue;

      double a = 0.0;
      if (std::abs(x_relax[j] - node_lb[j]) <= int_tol) {
        a = rc_abs;
        if (!std::isfinite(root_lb[j]) || !std::isfinite(node_lb[j])) return false;
      } else if (std::abs(x_relax[j] - node_ub[j]) <= int_tol) {
        a = -rc_abs;
        if (!std::isfinite(root_ub[j]) || !std::isfinite(node_ub[j])) return false;
      } else {
        continue;
      }
      coeff[static_cast<std::size_t>(j)] = a;
    }
  }

  auto min_activity = [&](const Eigen::VectorXd& lb,
                          const Eigen::VectorXd& ub) -> double {
    double activity = 0.0;
    for (int j = 0; j < n; ++j) {
      const double a = coeff[static_cast<std::size_t>(j)];
      if (a > 0.0) {
        if (!std::isfinite(lb[j])) return -std::numeric_limits<double>::infinity();
        activity += a * lb[j];
      } else if (a < 0.0) {
        if (!std::isfinite(ub[j])) return -std::numeric_limits<double>::infinity();
        activity += a * ub[j];
      }
    }
    return activity;
  };

  auto max_activity = [&](const Eigen::VectorXd& lb,
                          const Eigen::VectorXd& ub) -> double {
    double activity = 0.0;
    for (int j = 0; j < n; ++j) {
      const double a = coeff[static_cast<std::size_t>(j)];
      if (a > 0.0) {
        if (!std::isfinite(ub[j])) return std::numeric_limits<double>::infinity();
        activity += a * ub[j];
      } else if (a < 0.0) {
        if (!std::isfinite(lb[j])) return std::numeric_limits<double>::infinity();
        activity += a * lb[j];
      }
    }
    return activity;
  };

  double root_min_activity = min_activity(root_lb, root_ub);
  double node_min_activity = min_activity(node_lb, node_ub);
  if (!std::isfinite(root_min_activity) || !std::isfinite(node_min_activity)) {
    return false;
  }

  double rhs = use_dual_proof ? dual_proof->rhs : (node_min_activity + gap);

  // HiGHS-style proof-row coefficient tightening: for a valid <= proof row,
  // coefficients of integer columns need not exceed max_activity - rhs in
  // magnitude.  Tightening before reason selection makes the activity budget
  // smaller and avoids selecting huge, numerically dominant proof terms.
  const double root_max_activity = max_activity(root_lb, root_ub);
  const double max_excess = root_max_activity - rhs;
  if (!use_dual_proof && std::isfinite(root_max_activity) && max_excess > 1e-7) {
    bool tightened = false;
    for (int j = 0; j < n; ++j) {
      if (!is_integer_type(vars[static_cast<std::size_t>(j)])) continue;
      double& a = coeff[static_cast<std::size_t>(j)];
      if (a > max_excess) {
        if (!std::isfinite(root_ub[j])) return false;
        const double delta = a - max_excess;
        rhs -= delta * root_ub[j];
        a = max_excess;
        tightened = true;
      } else if (a < -max_excess) {
        if (!std::isfinite(root_lb[j])) return false;
        const double delta = -a - max_excess;
        rhs += delta * root_lb[j];
        a = -max_excess;
        tightened = true;
      }
    }
    if (tightened) {
      root_min_activity = min_activity(root_lb, root_ub);
      node_min_activity = min_activity(node_lb, node_ub);
      if (!std::isfinite(root_min_activity) || !std::isfinite(node_min_activity)) {
        return false;
      }
    }
  }

  bool has_forbidden_coeff = false;
  const double forbidden_coeff = coeff[static_cast<std::size_t>(forbidden.var_idx)];
  if ((forbidden_coeff > 0.0 && forbidden.is_lb) ||
      (forbidden_coeff < 0.0 && !forbidden.is_lb)) {
    has_forbidden_coeff = true;
  }
  if (!has_forbidden_coeff) return false;

  auto literal_delta = [&](const BranchDomainLiteral& lit) -> double {
    if (lit.var_idx < 0 || lit.var_idx >= n) return 0.0;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    if (a > 0.0 && lit.is_lb) {
      if (!std::isfinite(root_lb[lit.var_idx])) return 0.0;
      return std::max(0.0, a * (lit.value - root_lb[lit.var_idx]));
    }
    if (a < 0.0 && !lit.is_lb) {
      if (!std::isfinite(root_ub[lit.var_idx])) return 0.0;
      return std::max(0.0, a * (lit.value - root_ub[lit.var_idx]));
    }
    return 0.0;
  };

  struct Candidate {
    std::vector<BranchDomainLiteral> proof_literals;
    std::vector<BranchDomainLiteral> reason;
    BranchDomainLiteral implied_bound;
    double delta{0.0};
    bool has_implied_bound{false};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(branch_reasons.size() + reason_bounds.size());
  for (const auto& lit : branch_reasons) {
    const double delta = literal_delta(lit);
    if (delta > 1e-10) candidates.push_back({{lit}, {lit}, lit, delta, false});
  }
  for (const auto& rb : reason_bounds) {
    if (rb.reason.empty()) continue;
    const double delta = literal_delta(rb.bound);
    if (delta > 1e-10) {
      candidates.push_back({{rb.bound}, rb.reason, rb.bound, delta, true});
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              if (a.delta != b.delta) return a.delta > b.delta;
              return a.proof_literals.size() < b.proof_literals.size();
            });

  std::vector<BranchDomainLiteral> proof_explained_clause;
  auto build_implied_bound_from_forbidden =
      [&](const BranchDomainLiteral& bad,
          BranchDomainLiteral& implied) -> bool {
    if (bad.var_idx < 0 || bad.var_idx >= n || !std::isfinite(bad.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(bad.var_idx)]);
    if (bad.is_lb) {
      implied = BranchDomainLiteral{
          bad.var_idx,
          integer_var ? std::ceil(bad.value - 1e-9) - 1.0
                      : bad.value - 1e-7,
          false};
    } else {
      implied = BranchDomainLiteral{
          bad.var_idx,
          integer_var ? std::floor(bad.value + 1e-9) + 1.0
                      : bad.value + 1e-7,
          true};
    }
    return true;
  };

  auto explain_bound_change_from_proof_row =
      [&](std::vector<BranchDomainLiteral>& explained_clause) -> bool {
    explained_clause.clear();
    BranchDomainLiteral implied;
    if (!build_implied_bound_from_forbidden(forbidden, implied)) return false;
    const int target = implied.var_idx;
    const double a0 = coeff[static_cast<std::size_t>(target)];
    if ((implied.is_lb && a0 >= -1e-12) ||
        (!implied.is_lb && a0 <= 1e-12)) {
      return false;
    }

    const bool integer_target =
        is_integer_type(vars[static_cast<std::size_t>(target)]);
    double relaxed_bound = implied.value;
    if (integer_target) {
      const double relax = std::max(0.0, 1.0 - 10.0 * int_tol);
      relaxed_bound += implied.is_lb ? -relax : relax;
    } else {
      relaxed_bound += implied.is_lb ? -1e-7 : 1e-7;
    }

    double target_global_contribution = 0.0;
    if (a0 > 0.0) {
      if (!std::isfinite(root_lb[target])) return false;
      target_global_contribution = a0 * root_lb[target];
    } else {
      if (!std::isfinite(root_ub[target])) return false;
      target_global_contribution = a0 * root_ub[target];
    }

    const double required_activity = rhs - a0 * relaxed_bound;
    double explained_activity = root_min_activity - target_global_contribution;
    std::vector<int> reason_indices;
    std::vector<BranchDomainLiteral> reason_literals;
    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const Candidate& cand = candidates[static_cast<std::size_t>(idx)];
      if (cand.delta <= 1e-10 || cand.proof_literals.empty()) continue;
      bool touches_target = false;
      for (const auto& lit : cand.proof_literals) {
        if (lit.var_idx == target) {
          touches_target = true;
          break;
        }
      }
      if (touches_target) continue;
      std::vector<BranchDomainLiteral> trial = reason_literals;
      trial.insert(trial.end(), cand.reason.begin(), cand.reason.end());
      canonicalize_branch_literals(trial);
      if (static_cast<int>(trial.size()) + 1 > max_literals) continue;
      reason_indices.push_back(idx);
      reason_literals.swap(trial);
      explained_activity += cand.delta;
      if (explained_activity >=
          required_activity - std::max(1e-7, 1e-10 * std::abs(required_activity))) {
        break;
      }
    }
    if (explained_activity <
        required_activity - std::max(1e-7, 1e-10 * std::abs(required_activity))) {
      return false;
    }

    bool changed_reason = true;
    while (changed_reason && !reason_indices.empty()) {
      changed_reason = false;
      for (std::size_t p = 0; p < reason_indices.size(); ++p) {
        const int idx = reason_indices[p];
        const double trial_activity =
            explained_activity - candidates[static_cast<std::size_t>(idx)].delta;
        if (trial_activity <
            required_activity - std::max(1e-7, 1e-10 * std::abs(required_activity))) {
          continue;
        }
        std::vector<int> trial_indices = reason_indices;
        trial_indices.erase(trial_indices.begin() + static_cast<std::ptrdiff_t>(p));
        std::vector<BranchDomainLiteral> trial_literals;
        for (int kept : trial_indices) {
          const auto& lits =
              candidates[static_cast<std::size_t>(kept)].reason;
          trial_literals.insert(trial_literals.end(), lits.begin(), lits.end());
        }
        canonicalize_branch_literals(trial_literals);
        if (static_cast<int>(trial_literals.size()) + 1 > max_literals) continue;
        reason_indices.swap(trial_indices);
        reason_literals.swap(trial_literals);
        explained_activity = trial_activity;
        changed_reason = true;
        break;
      }
    }

    explained_clause = std::move(reason_literals);
    explained_clause.push_back(forbidden);
    canonicalize_branch_literals(explained_clause);
    return !explained_clause.empty() &&
           static_cast<int>(explained_clause.size()) <= max_literals;
  };

  (void)explain_bound_change_from_proof_row(proof_explained_clause);

  std::vector<BranchDomainLiteral> clause;
  clause.reserve(static_cast<std::size_t>(std::min(max_literals,
                                                   static_cast<int>(candidates.size()) + 1)));
  clause.push_back(forbidden);
  canonicalize_branch_literals(clause);

  const double forbidden_delta = literal_delta(forbidden);
  std::vector<int> selected;
  selected.reserve(candidates.size());
  double selected_delta = 0.0;

  auto violated_by_delta = [&](double delta, double* margin_out = nullptr) -> bool {
    const double activity = root_min_activity + forbidden_delta + delta;
    const double scale = 1.0 + std::abs(activity) + std::abs(rhs);
    const double margin = activity - rhs;
    if (margin_out != nullptr) *margin_out = margin;
    return margin > std::max(1e-7, 1e-10 * scale);
  };

  double margin = 0.0;
  if (!violated_by_delta(selected_delta, &margin)) {
    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const auto& cand = candidates[static_cast<std::size_t>(idx)];
      std::vector<BranchDomainLiteral> trial = clause;
      trial.insert(trial.end(), cand.reason.begin(), cand.reason.end());
      canonicalize_branch_literals(trial);
      if (static_cast<int>(trial.size()) > max_literals) continue;
      selected.push_back(idx);
      selected_delta += cand.delta;
      clause.swap(trial);
      if (violated_by_delta(selected_delta, &margin)) break;
    }
  }

  if (!violated_by_delta(selected_delta, &margin)) {
    out_clause.clear();
    return false;
  }

  bool changed = true;
  while (changed && !selected.empty()) {
    changed = false;
    for (std::size_t p = 0; p < selected.size(); ++p) {
      const int idx = selected[p];
      const double trial_delta =
          selected_delta - candidates[static_cast<std::size_t>(idx)].delta;
      double trial_margin = 0.0;
      if (!violated_by_delta(trial_delta, &trial_margin)) continue;
      std::vector<int> trial_selected = selected;
      trial_selected.erase(trial_selected.begin() + static_cast<std::ptrdiff_t>(p));
      std::vector<BranchDomainLiteral> trial_clause{forbidden};
      for (int kept : trial_selected) {
        const auto& lits =
            candidates[static_cast<std::size_t>(kept)].reason;
        trial_clause.insert(trial_clause.end(), lits.begin(), lits.end());
      }
      canonicalize_branch_literals(trial_clause);
      if (static_cast<int>(trial_clause.size()) > max_literals) continue;
      selected.swap(trial_selected);
      selected_delta = trial_delta;
      clause.swap(trial_clause);
      margin = trial_margin;
      changed = true;
      break;
    }
  }

  canonicalize_branch_literals(clause);
  if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
    out_clause.clear();
    return false;
  }
  if (proof_margin != nullptr) *proof_margin = margin;
  out_clause = std::move(clause);
  std::vector<std::vector<BranchDomainLiteral>> extra_proof_conflicts;
  if (violated_local_proof_cover != nullptr) {
    struct ProofTerm {
      BranchDomainLiteral lit;
      double coeff{0.0};
      double root_bound{0.0};
    };
    auto add_proof_term = [&](const BranchDomainLiteral& lit,
                              std::vector<ProofTerm>& terms) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n) return false;
      const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
      if (a > 0.0 && lit.is_lb) {
        if (!std::isfinite(root_lb[lit.var_idx])) return false;
        terms.push_back({lit, a, root_lb[lit.var_idx]});
        return true;
      }
      if (a < 0.0 && !lit.is_lb) {
        if (!std::isfinite(root_ub[lit.var_idx])) return false;
        terms.push_back({lit, a, root_ub[lit.var_idx]});
        return true;
      }
      return false;
    };

    auto emit_if_violated = [&](const std::vector<ProofTerm>& terms) {
      if (violated_local_proof_cover->coeff.nonZeros() > 0 || terms.empty()) {
        return false;
      }
      Eigen::SparseVector<double> coeff(n);
      coeff.reserve(static_cast<int>(terms.size()));
      double cover_rhs = rhs - root_min_activity;
      double norm2 = 0.0;
      for (const auto& term : terms) {
        coeff.coeffRef(term.lit.var_idx) += term.coeff;
        cover_rhs += term.coeff * term.root_bound;
        norm2 += term.coeff * term.coeff;
      }
      coeff.prune(0.0, 0.0);
      const double lhs = coeff.dot(x_relax);
      const double scale = 1.0 + std::abs(lhs) + std::abs(cover_rhs);
      if (lhs > cover_rhs + std::max(1e-7, 1e-10 * scale)) {
        *violated_local_proof_cover =
            PoolCut{std::move(coeff), cover_rhs, 0, 0.0, std::sqrt(norm2), 0};
      }
      return true;
    };

    auto build_weighted_from_indices =
        [&](const std::vector<int>& indices, bool include_forbidden) {
      std::vector<ProofTerm> terms;
      terms.reserve(indices.size() + (include_forbidden ? 1 : 0));
      bool proof_cover_valid = true;
      if (include_forbidden) {
        proof_cover_valid = add_proof_term(forbidden, terms);
      }
      for (int idx : indices) {
        const Candidate& cand = candidates[static_cast<std::size_t>(idx)];
        for (const auto& lit : cand.proof_literals) {
          proof_cover_valid =
              add_proof_term(lit, terms) && proof_cover_valid;
        }
      }
      if (proof_cover_valid) emit_if_violated(terms);
    };

    const double proof_budget = rhs - root_min_activity;
    std::vector<int> active_cover;
    std::vector<BranchDomainLiteral> active_literals;
    double active_delta = 0.0;
    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const Candidate& cand = candidates[static_cast<std::size_t>(idx)];
      if (cand.proof_literals.empty()) continue;
      std::vector<BranchDomainLiteral> trial = active_literals;
      trial.insert(trial.end(), cand.proof_literals.begin(),
                   cand.proof_literals.end());
      canonicalize_branch_literals(trial);
      if (static_cast<int>(trial.size()) > max_literals) continue;
      active_cover.push_back(idx);
      active_literals.swap(trial);
      active_delta += cand.delta;
      if (active_delta > proof_budget + std::max(1e-7, 1e-10 * std::abs(proof_budget))) {
        break;
      }
    }
    if (active_delta > proof_budget + std::max(1e-7, 1e-10 * std::abs(proof_budget))) {
      bool changed_cover = true;
      while (changed_cover && !active_cover.empty()) {
        changed_cover = false;
        for (std::size_t p = 0; p < active_cover.size(); ++p) {
          const int idx = active_cover[p];
          const double trial_delta =
              active_delta - candidates[static_cast<std::size_t>(idx)].delta;
          if (trial_delta <=
              proof_budget + std::max(1e-7, 1e-10 * std::abs(proof_budget))) {
            continue;
          }
          std::vector<int> trial_cover = active_cover;
          trial_cover.erase(trial_cover.begin() + static_cast<std::ptrdiff_t>(p));
          std::vector<BranchDomainLiteral> trial_literals;
          for (int kept : trial_cover) {
            const auto& lits =
                candidates[static_cast<std::size_t>(kept)].proof_literals;
            trial_literals.insert(trial_literals.end(), lits.begin(), lits.end());
          }
          canonicalize_branch_literals(trial_literals);
          if (static_cast<int>(trial_literals.size()) > max_literals) continue;
          active_cover.swap(trial_cover);
          active_literals.swap(trial_literals);
          active_delta = trial_delta;
          changed_cover = true;
          break;
        }
      }
      build_weighted_from_indices(active_cover, false);
      std::vector<BranchDomainLiteral> cutoff_reason_clause;
      for (int kept : active_cover) {
        const auto& reason =
            candidates[static_cast<std::size_t>(kept)].reason;
        cutoff_reason_clause.insert(cutoff_reason_clause.end(),
                                    reason.begin(), reason.end());
      }
      canonicalize_branch_literals(cutoff_reason_clause);
      if (!cutoff_reason_clause.empty() &&
          static_cast<int>(cutoff_reason_clause.size()) <= max_literals) {
        extra_proof_conflicts.push_back(std::move(cutoff_reason_clause));
      }
    }
    build_weighted_from_indices(selected, true);
  }
  if (reconvergence_clauses != nullptr) {
    reconvergence_clauses->insert(reconvergence_clauses->end(),
                                  extra_proof_conflicts.begin(),
                                  extra_proof_conflicts.end());
    if (!proof_explained_clause.empty()) {
      reconvergence_clauses->push_back(proof_explained_clause);
    }
    auto flipped_literal = [&](const BranchDomainLiteral& lit,
                               BranchDomainLiteral& out) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      const bool integer_var =
          is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
      if (lit.is_lb) {
        out = BranchDomainLiteral{
            lit.var_idx,
            integer_var ? std::ceil(lit.value - 1e-9) - 1.0 : lit.value - 1e-7,
            false};
      } else {
        out = BranchDomainLiteral{
            lit.var_idx,
            integer_var ? std::floor(lit.value + 1e-9) + 1.0 : lit.value + 1e-7,
            true};
      }
      return true;
    };

    auto find_reason = [&](const BranchDomainLiteral& lit)
        -> const std::vector<BranchDomainLiteral>* {
      const std::vector<BranchDomainLiteral>* best = nullptr;
      for (const auto& rb : reason_bounds) {
        if (rb.bound.var_idx != lit.var_idx || rb.bound.is_lb != lit.is_lb) {
          continue;
        }
        const bool covers = lit.is_lb
            ? (rb.bound.value >= lit.value - 1e-9)
            : (rb.bound.value <= lit.value + 1e-9);
        if (!covers || rb.reason.empty()) continue;
        if (best == nullptr || rb.reason.size() < best->size()) best = &rb.reason;
      }
      return best;
    };

    auto recursive_reason_frontier = [&](std::vector<BranchDomainLiteral> frontier) {
      canonicalize_branch_literals(frontier);
      std::vector<BranchDomainLiteral> best = frontier;
      constexpr int kMaxResolve = 12;
      for (int iter = 0; iter < kMaxResolve; ++iter) {
        int pos = -1;
        const std::vector<BranchDomainLiteral>* repl = nullptr;
        for (int i = static_cast<int>(frontier.size()) - 1; i >= 0; --i) {
          const auto* reason = find_reason(frontier[static_cast<std::size_t>(i)]);
          if (reason == nullptr) continue;
          pos = i;
          repl = reason;
          break;
        }
        if (pos < 0 || repl == nullptr) break;
        std::vector<BranchDomainLiteral> trial;
        trial.reserve(frontier.size() + repl->size());
        for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
          if (i == pos) continue;
          trial.push_back(frontier[static_cast<std::size_t>(i)]);
        }
        trial.insert(trial.end(), repl->begin(), repl->end());
        canonicalize_branch_literals(trial);
        if (trial.empty() || static_cast<int>(trial.size()) > max_literals) break;
        frontier.swap(trial);
        if (frontier.size() < best.size()) best = frontier;
      }
      return best;
    };

    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const auto& cand = candidates[static_cast<std::size_t>(idx)];
      if (!cand.has_implied_bound || cand.reason.empty()) continue;
      BranchDomainLiteral flipped;
      if (!flipped_literal(cand.implied_bound, flipped)) continue;
      std::vector<BranchDomainLiteral> reason =
          recursive_reason_frontier(cand.reason);
      std::vector<BranchDomainLiteral> reconv = reason;
      reconv.push_back(flipped);
      canonicalize_branch_literals(reconv);
      if (reconv.empty() ||
          static_cast<int>(reconv.size()) > std::min(max_literals, 32)) {
        continue;
      }
      bool duplicate = false;
      for (const auto& existing : *reconvergence_clauses) {
        if (existing.size() != reconv.size()) continue;
        bool same = true;
        for (std::size_t i = 0; i < existing.size(); ++i) {
          if (existing[i].var_idx != reconv[i].var_idx ||
              existing[i].is_lb != reconv[i].is_lb ||
              std::abs(existing[i].value - reconv[i].value) > 1e-9) {
            same = false;
            break;
          }
        }
        if (same) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        reconvergence_clauses->push_back(std::move(reconv));
        if (reconvergence_clauses->size() >= 2) break;
      }
    }
  }
  return true;
}


}  // namespace mipsolvers::engine::detail
