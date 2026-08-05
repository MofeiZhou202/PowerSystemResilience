/// @file bc_domain.hpp
/// @brief Stateful, incremental domain propagation analogous to HiGHS' HighsDomain.
///
/// Mirrors the algorithmic structure of HiGHS:
///   * Row activities (`act_min`, `act_max`) and infinity counters
///     (`act_inf_min`, `act_inf_max`) are computed ONCE in `init()`.
///   * `change_bound(col, lb, ub)` performs O(col-nnz) DELTA updates of the
///     row activities (HighsDomain::updateActivityLbChange/UbChange analog),
///     marks affected rows for re-propagation, and short-circuits to infeasible
///     when an updated min/max activity violates a row bound.
///   * `propagate()` walks ONLY the marked rows (HighsDomain::propagate analog),
///     using cached activity to derive residuals in O(row-nnz) per processed row
///     instead of O(row-nnz) twice.
///   * A trail of (col, old_lb, old_ub, kind) records every bound change so
///     `restore(savepoint)` can undo a probing window in reverse order, also
///     reverse-applying activity deltas. This lets a single `BCDomain` instance
///     be REUSED across all alpha steps of `linesearchRounding` (and the other
///     repair-LP callers), exactly as HiGHS' `localdom` is reused via
///     `fixCol → propagate → backtrack`.
///
/// Scope: this class implements the model-row propagation only. Cuts, conflict
/// pools, clique tables, and capacity thresholds — all of which HiGHS layers on
/// top — are NOT included. The native repair pipeline does not need them today
/// (the cuts are folded into the row matrix at base_lp construction time).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/detail/bc_numerics.hpp"

namespace mipsolvers::engine::detail {

class BCDomain {
 public:
  struct TrailEntry {
    int col;
    double old_lb;
    double old_ub;
    bool changed_lb;
    bool changed_ub;
  };

  struct Savepoint {
    std::size_t trail_size{0};
    std::vector<int> pending_rows;
    bool infeasible{false};
    bool propagation_complete{true};
  };

  static constexpr double kFeasTol = 1e-7;
  static constexpr double kBoundEps = 1e-9;

  BCDomain() = default;

  /// Initialise from an `LPModel` snapshot and a starting (lb,ub) box.
  /// Recomputes all row activities and marks all rows for an initial sweep.
  void init(const LPModel& lp,
            const Eigen::VectorXd& lb_in,
            const Eigen::VectorXd& ub_in) {
    lp_ = &lp;
    n_ = static_cast<int>(lp.vars.size());
    m_ineq_ = static_cast<int>(lp.A.rows());
    m_eq_ = static_cast<int>(lp.Aeq.rows());
    m_total_ = m_ineq_ + m_eq_;

    lb_ = lb_in;
    ub_ = ub_in;
    lb_.conservativeResize(n_);
    ub_.conservativeResize(n_);

    act_min_.assign(static_cast<size_t>(m_total_), StableActivitySum{});
    act_max_.assign(static_cast<size_t>(m_total_), StableActivitySum{});
    act_inf_min_.assign(static_cast<size_t>(m_total_), 0);
    act_inf_max_.assign(static_cast<size_t>(m_total_), 0);
    prop_flag_.assign(static_cast<size_t>(m_total_), 0);
    prop_inds_.clear();
    prop_inds_.reserve(static_cast<size_t>(m_total_));

    // row bounds: ineq rows  lhs<=Ax<=b ; eq rows  beq<=Aeqx<=beq
    row_lo_.assign(static_cast<size_t>(m_total_), -kInfBig);
    row_hi_.assign(static_cast<size_t>(m_total_),  kInfBig);
    if (lp_has_row_lhs(lp)) {
      for (int r = 0; r < m_ineq_; ++r) row_lo_[r] = lp.row_lhs[r];
    }
    for (int r = 0; r < m_ineq_; ++r) row_hi_[r] = lp.b[r];
    for (int r = 0; r < m_eq_; ++r) {
      row_lo_[m_ineq_ + r] = lp.beq[r];
      row_hi_[m_ineq_ + r] = lp.beq[r];
    }

    // Initial activity scan via row-major copy (one-time O(nnz)).
    A_row_ = lp.A;
    Aeq_row_ = lp.Aeq;
    for (int r = 0; r < m_ineq_; ++r) compute_row_activity_initial(r, A_row_, /*offset=*/0);
    for (int r = 0; r < m_eq_;   ++r) compute_row_activity_initial(r, Aeq_row_, /*offset=*/m_ineq_);

    // Mark every row for an initial pass and detect immediate infeasibility.
    infeasible_ = false;
    for (int rr = 0; rr < m_total_; ++rr) mark_propagate(rr);

    trail_.clear();
    rows_processed_ = 0;
  }

  bool infeasible() const { return infeasible_; }
  bool propagation_complete() const { return propagation_complete_; }
  const Eigen::VectorXd& lb() const { return lb_; }
  const Eigen::VectorXd& ub() const { return ub_; }
  int num_vars() const { return n_; }
  int num_rows() const { return m_total_; }
  std::size_t trail_size() const { return trail_.size(); }
  const TrailEntry& trail_entry(std::size_t pos) const {
    return trail_.at(pos);
  }
  std::uint64_t rows_processed() const { return rows_processed_; }

  /// Push a savepoint that `restore()` can later roll back to.
  Savepoint savepoint() const {
    return Savepoint{trail_.size(), prop_inds_, infeasible_,
                     propagation_complete_};
  }

  /// Roll back every bound change (and its activity-delta side effects)
  /// performed after `sp`, in LIFO order. Clears the infeasible flag.
  void restore(const Savepoint& sp) {
    // Rollback weakens bounds, so it cannot create a new row infeasibility.
    // Clear the failed probe state before applying reverse activity deltas;
    // otherwise the delta helpers deliberately short-circuit and leave cached
    // activities from the discarded probe behind.
    infeasible_ = false;
    while (trail_.size() > sp.trail_size) {
      const TrailEntry e = trail_.back();
      trail_.pop_back();
      // Reverse-update activities. We undo lb/ub one at a time using delta logic.
      if (e.changed_lb) {
        const double old_lb = lb_[e.col];
        const double new_lb = e.old_lb;
        apply_lb_delta(e.col, old_lb, new_lb, /*touch_marks=*/false);
        lb_[e.col] = new_lb;
      }
      if (e.changed_ub) {
        const double old_ub = ub_[e.col];
        const double new_ub = e.old_ub;
        apply_ub_delta(e.col, old_ub, new_ub, /*touch_marks=*/false);
        ub_[e.col] = new_ub;
      }
    }
    for (const int rr : prop_inds_) {
      if (rr >= 0 && rr < m_total_) {
        prop_flag_[static_cast<std::size_t>(rr)] = 0;
      }
    }
    prop_inds_ = sp.pending_rows;
    for (const int rr : prop_inds_) {
      if (rr >= 0 && rr < m_total_) {
        prop_flag_[static_cast<std::size_t>(rr)] = 1;
      }
    }
    infeasible_ = sp.infeasible;
    propagation_complete_ = sp.propagation_complete;
  }

  /// Fix `col` to a value (lb=ub=v). Returns `false` and sets `infeasible()`
  /// if the new bounds are inconsistent with the existing domain or if the
  /// activity-delta detects a row violation.
  bool fix_col(int col, double v) { return change_bound(col, v, v); }

  /// Tighten the bounds of `col` to [new_lb, new_ub] (intersected with current).
  /// Returns false if the resulting domain is empty or the activity-delta
  /// proves a row infeasibility.
  bool change_bound(int col, double new_lb, double new_ub) {
    if (infeasible_) return false;
    if (col < 0 || col >= n_) return true;
    const double cur_lb = lb_[col];
    const double cur_ub = ub_[col];
    if (new_lb < cur_lb) new_lb = cur_lb;
    if (new_ub > cur_ub) new_ub = cur_ub;
    if (is_integer_var(lp_->vars[col])) {
      new_lb = std::ceil(new_lb - kBoundEps);
      new_ub = std::floor(new_ub + kBoundEps);
    }
    if (new_lb > new_ub + kFeasTol) {
      infeasible_ = true;
      return false;
    }
    bool changed_lb = false;
    bool changed_ub = false;
    if (new_lb > cur_lb +
                     bound_improvement_tolerance(kBoundEps, cur_lb, new_lb)) {
      changed_lb = true;
    }
    if (new_ub < cur_ub -
                     bound_improvement_tolerance(kBoundEps, cur_ub, new_ub)) {
      changed_ub = true;
    }
    if (!changed_lb && !changed_ub) return true;

    trail_.push_back({col, cur_lb, cur_ub, changed_lb, changed_ub});

    if (changed_lb) {
      apply_lb_delta(col, cur_lb, new_lb, /*touch_marks=*/true);
      lb_[col] = new_lb;
    }
    if (!infeasible_ && changed_ub) {
      apply_ub_delta(col, cur_ub, new_ub, /*touch_marks=*/true);
      ub_[col] = new_ub;
    }
    return !infeasible_;
  }

  /// Process all marked rows (HighsDomain::propagate analog). Returns false
  /// when an inconsistency is found. Only rows whose activity moved close to
  /// or past a row bound after a `change_bound` are visited; further bound
  /// tightenings recurse via `change_bound` and may mark additional rows.
  bool propagate() {
    if (infeasible_) {
      propagation_complete_ = true;
      return false;
    }
    int safety = std::max(64, 8 * m_total_);
    while (!prop_inds_.empty() && safety-- > 0) {
      const int rr = prop_inds_.back();
      prop_inds_.pop_back();
      prop_flag_[static_cast<size_t>(rr)] = 0;
      ++rows_processed_;
      if (!propagate_row(rr)) {
        propagation_complete_ = true;
        return false;
      }
    }
    propagation_complete_ = prop_inds_.empty();
    return !infeasible_ && propagation_complete_;
  }

 private:
  static constexpr double kInfBig = std::numeric_limits<double>::infinity();

  static bool is_integer_var(const VariableMeta& var) {
    return var.type == VarType::Integer || var.type == VarType::Binary;
  }

  // --------------------------------------------------------------- helpers

  static void replace_activity_bound(StableActivitySum& activity, double a,
                                     double oldbound, double newbound,
                                     double sentinel, int& numinf) {
    if (oldbound == sentinel) {
      --numinf;
      if (newbound != sentinel) activity.add_product(a, newbound);
    } else if (newbound == sentinel) {
      ++numinf;
      activity.remove_product(a, oldbound);
    } else {
      activity.replace_product(a, oldbound, newbound);
    }
  }

  // Iterate column j's nonzeros in either A (offset=0) or Aeq (offset=m_ineq_).
  template <typename Fn>
  void for_each_nz_in_col(int col, Fn&& fn) const {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp_->A, col); it; ++it) {
      const int r = static_cast<int>(it.row());
      const double a = it.value();
      if (std::abs(a) > 1e-15) fn(r, a);            // ineq row index
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp_->Aeq, col); it; ++it) {
      const int r = static_cast<int>(it.row());
      const double a = it.value();
      if (std::abs(a) > 1e-15) fn(m_ineq_ + r, a);  // eq row index
    }
  }

  // Compute activity from scratch for one row; called only by init().
  void compute_row_activity_initial(int row_in_block,
                                    const Eigen::SparseMatrix<double, Eigen::RowMajor>& M,
                                    int row_offset) {
    const int rr = row_offset + row_in_block;
    StableActivitySum amin, amax;
    int infmin = 0, infmax = 0;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(M, row_in_block); it; ++it) {
      const int j = static_cast<int>(it.col());
      const double a = it.value();
      if (std::abs(a) <= 1e-15) continue;
      if (a > 0.0) {
        if (!std::isfinite(lb_[j])) ++infmin; else amin.add_product(a, lb_[j]);
        if (!std::isfinite(ub_[j])) ++infmax; else amax.add_product(a, ub_[j]);
      } else {
        if (!std::isfinite(ub_[j])) ++infmin; else amin.add_product(a, ub_[j]);
        if (!std::isfinite(lb_[j])) ++infmax; else amax.add_product(a, lb_[j]);
      }
    }
    act_min_[static_cast<size_t>(rr)] = amin;
    act_max_[static_cast<size_t>(rr)] = amax;
    act_inf_min_[static_cast<size_t>(rr)] = infmin;
    act_inf_max_[static_cast<size_t>(rr)] = infmax;
  }

  // Lower-bound change of column `col`: oldbound→newbound.
  // Updates row activities incrementally (HighsDomain::updateActivityLbChange).
  void apply_lb_delta(int col, double oldbound, double newbound,
                      bool touch_marks) {
    for_each_nz_in_col(col, [&](int rr, double a) {
      if (infeasible_) return;
      if (a > 0.0) {
        replace_activity_bound(act_min_[static_cast<size_t>(rr)], a, oldbound,
                               newbound, -kInfBig,
                               act_inf_min_[static_cast<size_t>(rr)]);
        if (act_inf_min_[static_cast<size_t>(rr)] == 0 &&
            act_min_[static_cast<size_t>(rr)].violates_upper(
                row_hi_[static_cast<size_t>(rr)], kFeasTol)) {
          infeasible_ = true; return;
        }
        if (touch_marks) mark_propagate(rr);
      } else {
        replace_activity_bound(act_max_[static_cast<size_t>(rr)], a, oldbound,
                               newbound, -kInfBig,
                               act_inf_max_[static_cast<size_t>(rr)]);
        if (act_inf_max_[static_cast<size_t>(rr)] == 0 &&
            act_max_[static_cast<size_t>(rr)].violates_lower(
                row_lo_[static_cast<size_t>(rr)], kFeasTol)) {
          infeasible_ = true; return;
        }
        if (touch_marks) mark_propagate(rr);
      }
    });
  }

  // Upper-bound change of column `col`: oldbound→newbound.
  void apply_ub_delta(int col, double oldbound, double newbound,
                      bool touch_marks) {
    for_each_nz_in_col(col, [&](int rr, double a) {
      if (infeasible_) return;
      if (a > 0.0) {
        replace_activity_bound(act_max_[static_cast<size_t>(rr)], a, oldbound,
                               newbound, kInfBig,
                               act_inf_max_[static_cast<size_t>(rr)]);
        if (act_inf_max_[static_cast<size_t>(rr)] == 0 &&
            act_max_[static_cast<size_t>(rr)].violates_lower(
                row_lo_[static_cast<size_t>(rr)], kFeasTol)) {
          infeasible_ = true; return;
        }
        if (touch_marks) mark_propagate(rr);
      } else {
        replace_activity_bound(act_min_[static_cast<size_t>(rr)], a, oldbound,
                               newbound, kInfBig,
                               act_inf_min_[static_cast<size_t>(rr)]);
        if (act_inf_min_[static_cast<size_t>(rr)] == 0 &&
            act_min_[static_cast<size_t>(rr)].violates_upper(
                row_hi_[static_cast<size_t>(rr)], kFeasTol)) {
          infeasible_ = true; return;
        }
        if (touch_marks) mark_propagate(rr);
      }
    });
  }

  // Mark a row for propagation iff its current activity could plausibly
  // deduce a tightening (HighsDomain::markPropagate analog, simplified — we
  // skip the capacity-threshold short-circuit but keep the `inf<=1` guard
  // that prevents propagating rows with too many free columns to be useful).
  void mark_propagate(int rr) {
    if (prop_flag_[static_cast<size_t>(rr)]) return;
    const double lo = row_lo_[static_cast<size_t>(rr)];
    const double hi = row_hi_[static_cast<size_t>(rr)];
    const int infmin = act_inf_min_[static_cast<size_t>(rr)];
    const int infmax = act_inf_max_[static_cast<size_t>(rr)];

    bool prop_upper = std::isfinite(hi);
    bool prop_lower = std::isfinite(lo);
    // The HiGHS guard skips rows with >1 missing-finite-bound columns, since
    // those cannot deduce any tightening on a single column.
    if (infmin > 1) prop_upper = false;
    if (infmax > 1) prop_lower = false;

    if (prop_upper || prop_lower) {
      prop_flag_[static_cast<size_t>(rr)] = 1;
      prop_inds_.push_back(rr);
      propagation_complete_ = false;
    }
  }

  // Process one row, deriving column tightenings from cached activity.
  // Mirrors HighsDomain::propagateRowUpper / propagateRowLower.
  bool propagate_row(int rr) {
    const double lo = row_lo_[static_cast<size_t>(rr)];
    const double hi = row_hi_[static_cast<size_t>(rr)];
    const StableActivitySum& amin = act_min_[static_cast<size_t>(rr)];
    const StableActivitySum& amax = act_max_[static_cast<size_t>(rr)];
    const int infmin = act_inf_min_[static_cast<size_t>(rr)];
    const int infmax = act_inf_max_[static_cast<size_t>(rr)];

    // Quick infeasibility re-check (cheap; HighsDomain does the same).
    if (infmin == 0 && std::isfinite(hi) &&
        amin.violates_upper(hi, kFeasTol)) {
      infeasible_ = true; return false;
    }
    if (infmax == 0 && std::isfinite(lo) &&
        amax.violates_lower(lo, kFeasTol)) {
      infeasible_ = true; return false;
    }

    auto sweep_row = [&](auto IteratorMaker, int row_in_block) {
      // Upper side: A_r x ≤ hi  ⇒ for each col j: hi - (amin - contrib_min(j)) ≥ a_j*x_j(direction).
      if (std::isfinite(hi) && infmin <= 1) {
        for (auto it = IteratorMaker(); it; ++it) {
          if (infeasible_) return;
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          double contrib;
          if (a > 0.0) {
            if (!std::isfinite(lb_[j])) {
              if (infmin > 1) continue;
              contrib = -kInfBig;  // residual must use full amin
            } else {
              contrib = a * lb_[j];
            }
          } else {
            if (!std::isfinite(ub_[j])) {
              if (infmin > 1) continue;
              contrib = -kInfBig;
            } else {
              contrib = a * ub_[j];
            }
          }
          double residual;
          if (infmin == 1) {
            if (contrib != -kInfBig) continue;
            residual = (hi - amin.value()) +
                       amin.comparison_tolerance(hi, kFeasTol);
          } else {
            const double bound = a > 0.0 ? lb_[j] : ub_[j];
            residual = amin.upper_residual(hi, a, bound, kFeasTol);
          }
          const double bound_val = residual / a;
          if (a > 0.0) {
            double new_ub = bound_val;
            if (is_integer_var(lp_->vars[j])) new_ub = std::floor(new_ub + kBoundEps);
            if (new_ub < ub_[j] - bound_improvement_tolerance(
                                         kBoundEps, ub_[j], new_ub)) {
              if (!change_bound(j, lb_[j], new_ub)) return;
            }
          } else {
            double new_lb = bound_val;
            if (is_integer_var(lp_->vars[j])) new_lb = std::ceil(new_lb - kBoundEps);
            if (new_lb > lb_[j] + bound_improvement_tolerance(
                                         kBoundEps, lb_[j], new_lb)) {
              if (!change_bound(j, new_lb, ub_[j])) return;
            }
          }
        }
      }
      // Lower side: lo ≤ A_r x  ⇒ symmetric using amax.
      if (std::isfinite(lo) && infmax <= 1) {
        for (auto it = IteratorMaker(); it; ++it) {
          if (infeasible_) return;
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          double contrib;
          if (a > 0.0) {
            if (!std::isfinite(ub_[j])) {
              if (infmax > 1) continue;
              contrib = kInfBig;
            } else {
              contrib = a * ub_[j];
            }
          } else {
            if (!std::isfinite(lb_[j])) {
              if (infmax > 1) continue;
              contrib = kInfBig;
            } else {
              contrib = a * lb_[j];
            }
          }
          double residual;
          if (infmax == 1) {
            if (contrib != kInfBig) continue;
            residual = (lo - amax.value()) -
                       amax.comparison_tolerance(lo, kFeasTol);
          } else {
            const double bound = a > 0.0 ? ub_[j] : lb_[j];
            residual = amax.lower_residual(lo, a, bound, kFeasTol);
          }
          const double bound_val = residual / a;
          if (a > 0.0) {
            double new_lb = bound_val;
            if (is_integer_var(lp_->vars[j])) new_lb = std::ceil(new_lb - kBoundEps);
            if (new_lb > lb_[j] + bound_improvement_tolerance(
                                         kBoundEps, lb_[j], new_lb)) {
              if (!change_bound(j, new_lb, ub_[j])) return;
            }
          } else {
            double new_ub = bound_val;
            if (is_integer_var(lp_->vars[j])) new_ub = std::floor(new_ub + kBoundEps);
            if (new_ub < ub_[j] - bound_improvement_tolerance(
                                         kBoundEps, ub_[j], new_ub)) {
              if (!change_bound(j, lb_[j], new_ub)) return;
            }
          }
        }
      }
      (void)row_in_block;
    };

    if (rr < m_ineq_) {
      const int rib = rr;
      sweep_row([&]() {
        return Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator(A_row_, rib);
      }, rib);
    } else {
      const int rib = rr - m_ineq_;
      sweep_row([&]() {
        return Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator(Aeq_row_, rib);
      }, rib);
    }
    return !infeasible_;
  }

  // ----------------------------------------------------------------- state

  const LPModel* lp_{nullptr};
  int n_{0};
  int m_ineq_{0};
  int m_eq_{0};
  int m_total_{0};

  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;

  // Row-major copies for fast row sweeps inside propagate().
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row_;

  // Per-row activity state.
  std::vector<StableActivitySum> act_min_;
  std::vector<StableActivitySum> act_max_;
  std::vector<int> act_inf_min_;
  std::vector<int> act_inf_max_;
  std::vector<double> row_lo_;
  std::vector<double> row_hi_;

  std::vector<char> prop_flag_;
  std::vector<int> prop_inds_;

  std::vector<TrailEntry> trail_;
  std::uint64_t rows_processed_{0};
  bool infeasible_{false};
  bool propagation_complete_{true};
};

}  // namespace mipsolvers::engine::detail
