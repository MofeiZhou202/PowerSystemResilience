/// @file bc_root_redcost_lurking.hpp
/// @brief Reduced-cost "lurking bound" store used at the B&C root node.
///
/// Records, per integer column, the objective cutoff at which a reduced-cost
/// argument would force a tighter bound, and later propagates those bounds
/// once the incumbent cutoff crosses the recorded threshold.  Extracted
/// verbatim from the monolithic branch_and_cut translation unit; it is a
/// header-only struct with all-inline methods.

#pragma once

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstddef>
#include <map>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/detail/bc_types.hpp"

namespace mipsolvers::engine::detail {

struct RootRedcostLurkingBounds {
  struct AddStats {
    int columns_seen{0};
    int records_added{0};
    int records_total{0};
    int side_rejected{0};
  };

  struct PropagateStats {
    int records_scanned{0};
    int records_active{0};
    int tightened{0};
    int pruned{0};
  };

  std::vector<std::multimap<double, double>> lower_bounds;
  std::vector<std::multimap<double, double>> upper_bounds;

  void ensure_size(int n) {
    if (n <= 0) return;
    if (static_cast<int>(lower_bounds.size()) < n) {
      lower_bounds.resize(static_cast<std::size_t>(n));
      upper_bounds.resize(static_cast<std::size_t>(n));
    }
  }

  int size() const {
    int total = 0;
    for (const auto& bucket : lower_bounds) {
      total += static_cast<int>(bucket.size());
    }
    for (const auto& bucket : upper_bounds) {
      total += static_cast<int>(bucket.size());
    }
    return total;
  }

  int prune_below_proof_lower_bound(double proof_lower_bound, double tol) {
    if (!std::isfinite(proof_lower_bound)) return 0;
    const double local_tol = std::max(1e-9, tol);
    int removed = 0;
    auto prune_bucket = [&](std::multimap<double, double>& bucket) {
      auto last = bucket.upper_bound(proof_lower_bound + local_tol);
      for (auto it = bucket.begin(); it != last;) {
        it = bucket.erase(it);
        ++removed;
      }
    };
    for (auto& bucket : lower_bounds) prune_bucket(bucket);
    for (auto& bucket : upper_bounds) prune_bucket(bucket);
    return removed;
  }

  bool add_candidate(int col,
                     bool is_lower,
                     double required_cutoff,
                     double bound,
                     double tol) {
    if (col < 0 || !std::isfinite(required_cutoff) || !std::isfinite(bound)) {
      return false;
    }
    ensure_size(col + 1);
    auto& bucket = is_lower ? lower_bounds[static_cast<std::size_t>(col)]
                            : upper_bounds[static_cast<std::size_t>(col)];
    const int strength = is_lower ? 1 : -1;
    auto pos = bucket.lower_bound(required_cutoff - tol);
    for (auto it = pos; it != bucket.end(); ++it) {
      const bool dominated =
          strength * it->second >= strength * bound - tol;
      if (dominated) return false;
    }
    auto inserted = bucket.emplace_hint(pos, required_cutoff, bound);
    for (auto it = bucket.begin(); it != inserted;) {
      if (strength * bound >= strength * it->second - tol) {
        it = bucket.erase(it);
      } else {
        ++it;
      }
    }
    return true;
  }

  AddStats add_from_root_lp(const std::vector<VariableMeta>& vars,
                            const std::vector<char>* implied_integer_cols,
                            const Eigen::VectorXd& x_relax,
                            const Eigen::VectorXd& reduced_costs,
                            const Eigen::VectorXd* sf_col_scale,
                            const std::vector<int>& basis_indices,
                            const Eigen::VectorXd& lb,
                            const Eigen::VectorXd& ub,
                            double lp_objective,
                            double int_tol) {
    AddStats stats;
    const int n = static_cast<int>(vars.size());
    if (n <= 0 || reduced_costs.size() < n ||
        lb.size() < n || ub.size() < n || !std::isfinite(lp_objective)) {
      stats.records_total = size();
      return stats;
    }
    ensure_size(n);
    (void)prune_below_proof_lower_bound(lp_objective, int_tol);
    std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
    for (int idx : basis_indices) {
      if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
    }
    auto is_integral = [&](int col) {
      return col >= 0 && col < n &&
             (is_integer_type(vars[static_cast<std::size_t>(col)]) ||
              (implied_integer_cols != nullptr &&
               col < static_cast<int>(implied_integer_cols->size()) &&
               (*implied_integer_cols)[static_cast<std::size_t>(col)] != 0));
    };
    auto col_dual = [&](int col) -> double {
      const double scale =
          sf_col_scale != nullptr && sf_col_scale->size() > col &&
                  std::isfinite((*sf_col_scale)[col]) &&
                  std::abs((*sf_col_scale)[col]) > 1e-18
              ? (*sf_col_scale)[col]
              : 1.0;
      return -reduced_costs[col] / scale;
    };

    int large_domain_cols = 0;
    for (int col = 0; col < n; ++col) {
      if (!is_integral(col) || is_basic[static_cast<std::size_t>(col)] ||
          !std::isfinite(lb[col]) || !std::isfinite(ub[col])) {
        continue;
      }
      const double rc_abs = std::abs(col_dual(col));
      if (ub[col] - lb[col] >= 512.0 && rc_abs > int_tol) {
        ++large_domain_cols;
      }
    }
    int max_steps_exp = 10;
    int exp_shift = 0;
    std::frexp(static_cast<double>(large_domain_cols) / 10.0, &exp_shift);
    if (exp_shift > 5) {
      exp_shift = std::min(exp_shift, max_steps_exp);
      max_steps_exp = max_steps_exp - exp_shift + 5;
    }
    const long long max_steps = 1LL << max_steps_exp;
    const double tol = std::max(1e-9, int_tol);
    for (int col = 0; col < n; ++col) {
      if (!is_integral(col) || !std::isfinite(lb[col]) ||
          !std::isfinite(ub[col])) {
        continue;
      }
      const double lpredcost = col_dual(col);
      if (!std::isfinite(lpredcost) || std::abs(lpredcost) <= tol ||
          ub[col] <= lb[col] + tol) {
        continue;
      }
      const double side_tol =
          std::max({1e-7, 10.0 * tol,
                    1e-10 * std::max(1.0, std::max(std::abs(lb[col]),
                                                    std::abs(ub[col])))});
      const bool at_lower = x_relax.size() > col &&
                            std::isfinite(x_relax[col]) &&
                            std::abs(x_relax[col] - lb[col]) <= side_tol;
      const bool at_upper = x_relax.size() > col &&
                            std::isfinite(x_relax[col]) &&
                            std::abs(x_relax[col] - ub[col]) <= side_tol;
      if ((lpredcost > tol && !at_lower) ||
          (lpredcost < -tol && !at_upper)) {
        ++stats.side_rejected;
        continue;
      }
      const double lo_d = std::ceil(lb[col] - tol);
      const double hi_d = std::floor(ub[col] + tol);
      if (!std::isfinite(lo_d) || !std::isfinite(hi_d) ||
          lo_d > hi_d || lo_d < static_cast<double>(LLONG_MIN / 4) ||
          hi_d > static_cast<double>(LLONG_MAX / 4)) {
        continue;
      }
      const long long lo = static_cast<long long>(lo_d);
      const long long hi = static_cast<long long>(hi_d);
      if (lo >= hi) continue;

      ++stats.columns_seen;

      if (lpredcost > tol) {
        const long long last = hi - 1;
        if (last >= lo) {
          long long step = 1;
          const long long range = last - lo;
          if (range > max_steps) {
            step = std::max<long long>(
                1, (range + max_steps - 1) >> max_steps_exp);
          }
          const double shift = static_cast<double>(step) - 10.0 * tol;
          for (long long lurk_ub = lo; lurk_ub <= last; lurk_ub += step) {
            const double required =
                lp_objective +
                (static_cast<double>(lurk_ub - lo) + shift) * lpredcost;
            if (required < lp_objective + tol) {
              if (last - lurk_ub < step) break;
              continue;
            }
            if (add_candidate(col, /*is_lower=*/false, required,
                              static_cast<double>(lurk_ub), tol)) {
              ++stats.records_added;
            }
            if (last - lurk_ub < step) break;
          }
        }
      }
      else if (lpredcost < -tol) {
        const long long last = lo + 1;
        if (last <= hi) {
          long long step = 1;
          const long long range = hi - last;
          if (range > max_steps) {
            step = std::max<long long>(
                1, (range + max_steps - 1) >> max_steps_exp);
          }
          const double shift = -static_cast<double>(step) + 10.0 * tol;
          for (long long lurk_lb = hi; lurk_lb >= last; lurk_lb -= step) {
            const double required =
                lp_objective +
                (static_cast<double>(lurk_lb - hi) + shift) * lpredcost;
            if (required < lp_objective + tol) {
              if (lurk_lb - last < step) break;
              continue;
            }
            if (add_candidate(col, /*is_lower=*/true, required,
                              static_cast<double>(lurk_lb), tol)) {
              ++stats.records_added;
            }
            if (lurk_lb - last < step) break;
          }
        }
      }
    }
    stats.records_total = size();
    return stats;
  }

  PropagateStats propagate(double cutoff,
                           double proof_lower_bound,
                           Eigen::VectorXd& lb,
                           Eigen::VectorXd& ub,
                           std::vector<DomainReasonBound>* reason_bounds_out,
                           int reason_depth,
                           int reason_trail_offset,
                           double tol) {
    PropagateStats stats;
    const int n = static_cast<int>(std::min(lb.size(), ub.size()));
    if (n <= 0 || !std::isfinite(cutoff)) return stats;
    ensure_size(n);
    const double local_tol = std::max(1e-9, tol);
    const double cutoff_tol =
        std::max(local_tol, 1e-10 * std::max(1.0, std::abs(cutoff)));
    auto append_reason = [&](int col, bool is_lower, double value,
                             double required_cutoff) {
      if (reason_bounds_out == nullptr) return;
      DomainReasonBound rb;
      rb.bound = BranchDomainLiteral{col, value, is_lower};
      rb.depth = reason_depth;
      rb.trail_pos =
          reason_trail_offset + static_cast<int>(reason_bounds_out->size());
      rb.source_conflict_literal =
          is_lower ? BranchDomainLiteral{
                         col, std::ceil(value - local_tol) - 1.0, false}
                   : BranchDomainLiteral{
                         col, std::floor(value + local_tol) + 1.0, true};
      rb.has_source_conflict_literal = true;
      rb.source_conflict_clause = {rb.source_conflict_literal};
      rb.has_source_conflict_clause = true;
      rb.has_proof_activity_audit =
          std::isfinite(required_cutoff) && std::isfinite(cutoff);
      rb.proof_activity = required_cutoff;
      rb.proof_required_activity = cutoff;
      rb.proof_activity_margin = required_cutoff - cutoff;
      reason_bounds_out->push_back(std::move(rb));
    };
    auto consume_bucket =
        [&](int col, bool is_lower,
            std::multimap<double, double>& bucket) {
      if (bucket.empty()) return true;
      if (std::isfinite(proof_lower_bound)) {
        bucket.erase(bucket.begin(),
                     bucket.upper_bound(proof_lower_bound + local_tol));
      }
      for (auto it = bucket.lower_bound(cutoff - cutoff_tol);
           it != bucket.end(); ++it) {
        ++stats.records_scanned;
        const double bound = it->second;
        const double required_cutoff = it->first;
        ++stats.records_active;
        if (is_lower) {
          if (bound > ub[col] + local_tol) {
            ++stats.pruned;
            return false;
          }
          if (bound > lb[col] + local_tol) {
            lb[col] = bound;
            ++stats.tightened;
            append_reason(col, true, bound, required_cutoff);
          }
        } else {
          if (bound < lb[col] - local_tol) {
            ++stats.pruned;
            return false;
          }
          if (bound < ub[col] - local_tol) {
            ub[col] = bound;
            ++stats.tightened;
            append_reason(col, false, bound, required_cutoff);
          }
        }
      }
      return lb[col] <= ub[col] + local_tol;
    };
    for (int col = 0; col < n; ++col) {
      if (!consume_bucket(col, true, lower_bounds[static_cast<std::size_t>(col)]) ||
          !consume_bucket(col, false, upper_bounds[static_cast<std::size_t>(col)])) {
        return stats;
      }
    }
    return stats;
  }
};

}  // namespace mipsolvers::engine::detail
