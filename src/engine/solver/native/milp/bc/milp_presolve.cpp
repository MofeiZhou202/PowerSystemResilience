/// @file milp_presolve.cpp
/// @brief Comprehensive MILP presolve implementation.
///
/// Techniques implemented (roughly in order of application):
///   1. Remove fixed variables
///   2. Remove empty rows/columns
///   3. Singleton rows → bound tightening
///   4. Singleton columns → implied free substitution (if equality row exists)
///   5. Doubleton equations → variable substitution
///   6. Bound tightening (LP relaxation)
///   7. Forcing rows (all variables at bound → fix them)
///   8. Dominated columns (cost-based variable fixing)
///   9. Parallel row detection and merging
///  10. Coefficient strengthening (MIP)
///  11. Binary probing with bound intersection

#include "mipsolvers/engine/solver/native/milp/bc/milp_presolve.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mipsolvers::engine {

static constexpr double kInf = 1e20;

// ============================================================================
// Construction
// ============================================================================

MILPPresolve::MILPPresolve(PresolveOptions opts) : opts_(std::move(opts)) {}

bool MILPPresolve::deadline_expired() const {
  if (!(opts_.time_limit_sec > 0.0) || !std::isfinite(opts_.time_limit_sec)) {
    return false;
  }
  return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                       run_start_)
             .count() >= opts_.time_limit_sec;
}

// ============================================================================
// init_from_lp: build internal representation from LPModel
// ============================================================================

void MILPPresolve::init_from_lp(const LPModel& lp) {
  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int m = m_ineq + m_eq;

  n_orig_ = n;
  m_orig_ = m;
  m_ineq_orig_ = m_ineq;

  // Columns
  col_lb_.resize(n);
  col_ub_.resize(n);
  col_cost_.resize(n);
  col_type_.resize(n);
  col_deleted_.assign(n, false);
  col_lb_orig_.resize(n);
  col_ub_orig_.resize(n);
  cols_.resize(n);
  for (int j = 0; j < n; ++j) {
    col_lb_[j] = lp.vars[j].lb;
    col_ub_[j] = lp.vars[j].ub;
    col_cost_[j] = (j < lp.c.size()) ? lp.c[j] : 0.0;
    col_type_[j] = lp.vars[j].type;
    col_lb_orig_[j] = lp.vars[j].lb;
    col_ub_orig_[j] = lp.vars[j].ub;
  }

  // Rows: first m_ineq rows are row_lhs <= Ax <= b, last m_eq rows are
  // Aeq*x = beq. row_lhs is optional for legacy upper-only models.
  rows_.resize(m);
  row_lb_.resize(m);
  row_ub_.resize(m);
  row_deleted_.assign(m, false);
  row_activity_.resize(m);

  // Build row entries from inequality/ranged constraints.
  {
    Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
    const bool has_row_lhs = lp.row_lhs.size() == m_ineq;
    for (int r = 0; r < m_ineq; ++r) {
      row_lb_[r] = has_row_lhs ? lp.row_lhs[r] : -kInf;
      row_ub_[r] = lp.b[r];
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const double v = it.value();
        if (std::abs(v) > opts_.zero_tol) {
          const int c = static_cast<int>(it.col());
          rows_[r].push_back({c, v});
          cols_[c].push_back({r, v});
        }
      }
    }
  }

  // Build row entries from equality constraints (Aeq*x = beq → beq <= a^T x <= beq)
  {
    Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;
    for (int r = 0; r < m_eq; ++r) {
      const int ri = m_ineq + r;
      row_lb_[ri] = lp.beq[r];
      row_ub_[ri] = lp.beq[r];
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const double v = it.value();
        if (std::abs(v) > opts_.zero_tol) {
          const int c = static_cast<int>(it.col());
          rows_[ri].push_back({c, v});
          cols_[c].push_back({ri, v});
        }
      }
    }
  }

  stats_.orig_rows = m;
  stats_.orig_cols = n;
  int nnz = 0;
  for (int r = 0; r < m; ++r) nnz += static_cast<int>(rows_[r].size());
  stats_.orig_nnz = nnz;
  compute_all_activities();
}

// ============================================================================
// Compute/update row activity bounds
// ============================================================================

void MILPPresolve::compute_all_activities() {
  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    update_activity(r);
  }
}

void MILPPresolve::update_activity(int row) {
  ++stats_.activity_rows_recomputed;
  auto& ac = row_activity_[row];
  ac.min_act.reset();
  ac.max_act.reset();
  ac.n_inf_min = 0;
  ac.n_inf_max = 0;
  for (const auto& e : rows_[row]) {
    if (col_deleted_[e.col]) continue;
    if (e.val > 0) {
      if (col_lb_[e.col] <= -kInf + 1) ac.n_inf_min++;
      else ac.min_act.add_product(e.val, col_lb_[e.col]);
      if (col_ub_[e.col] >= kInf - 1) ac.n_inf_max++;
      else ac.max_act.add_product(e.val, col_ub_[e.col]);
    } else {
      if (col_ub_[e.col] >= kInf - 1) ac.n_inf_min++;
      else ac.min_act.add_product(e.val, col_ub_[e.col]);
      if (col_lb_[e.col] <= -kInf + 1) ac.n_inf_max++;
      else ac.max_act.add_product(e.val, col_lb_[e.col]);
    }
  }
}

bool MILPPresolve::set_col_lower_bound(int col, double value) {
  const double old = col_lb_[col];
  if (value == old) return false;
  const bool old_inf = old <= -kInf + 1;
  const bool new_inf = value <= -kInf + 1;
  for (const auto& ce : cols_[col]) {
    if (row_deleted_[ce.row]) continue;
    auto& ac = row_activity_[ce.row];
    auto* activity = ce.val > 0.0 ? &ac.min_act : &ac.max_act;
    int* n_inf = ce.val > 0.0 ? &ac.n_inf_min : &ac.n_inf_max;
    if (old_inf) {
      --(*n_inf);
    } else {
      activity->remove_product(ce.val, old);
    }
    if (new_inf) {
      ++(*n_inf);
    } else {
      activity->add_product(ce.val, value);
    }
    ++stats_.activity_delta_updates;
  }
  col_lb_[col] = value;
  return true;
}

bool MILPPresolve::set_col_upper_bound(int col, double value) {
  const double old = col_ub_[col];
  if (value == old) return false;
  const bool old_inf = old >= kInf - 1;
  const bool new_inf = value >= kInf - 1;
  for (const auto& ce : cols_[col]) {
    if (row_deleted_[ce.row]) continue;
    auto& ac = row_activity_[ce.row];
    auto* activity = ce.val > 0.0 ? &ac.max_act : &ac.min_act;
    int* n_inf = ce.val > 0.0 ? &ac.n_inf_max : &ac.n_inf_min;
    if (old_inf) {
      --(*n_inf);
    } else {
      activity->remove_product(ce.val, old);
    }
    if (new_inf) {
      ++(*n_inf);
    } else {
      activity->add_product(ce.val, value);
    }
    ++stats_.activity_delta_updates;
  }
  col_ub_[col] = value;
  return true;
}

void MILPPresolve::remove_col_from_activities(int col) {
  const double lb = col_lb_[col];
  const double ub = col_ub_[col];
  for (const auto& ce : cols_[col]) {
    if (row_deleted_[ce.row]) continue;
    auto& ac = row_activity_[ce.row];
    if (ce.val > 0.0) {
      if (lb <= -kInf + 1) --ac.n_inf_min;
      else ac.min_act.remove_product(ce.val, lb);
      if (ub >= kInf - 1) --ac.n_inf_max;
      else ac.max_act.remove_product(ce.val, ub);
    } else {
      if (ub >= kInf - 1) --ac.n_inf_min;
      else ac.min_act.remove_product(ce.val, ub);
      if (lb <= -kInf + 1) --ac.n_inf_max;
      else ac.max_act.remove_product(ce.val, lb);
    }
    ++stats_.activity_delta_updates;
  }
}

void MILPPresolve::mark_infeasible(std::string reason) {
  if (stats_.infeasible) return;
  stats_.infeasible = true;
  stats_.infeasibility_reason = std::move(reason);
}

bool MILPPresolve::detect_infeasibility(const char* phase) {
  for (int j = 0; j < n_orig_; ++j) {
    if (col_deleted_[j]) continue;
    if (col_lb_[j] > col_ub_[j] + opts_.bound_tol) {
      mark_infeasible(std::string("inconsistent column bounds after ") + phase);
      return true;
    }
  }

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    const auto& ac = row_activity_[r];
    const bool violates_upper =
        row_ub_[r] < kInf - 1 && ac.n_inf_min == 0 &&
        ac.min_act.violates_upper(row_ub_[r], opts_.bound_tol);
    const bool violates_lower =
        row_lb_[r] > -kInf + 1 && ac.n_inf_max == 0 &&
        ac.max_act.violates_lower(row_lb_[r], opts_.bound_tol);
    if (violates_upper || violates_lower) {
      mark_infeasible(std::string("infeasible row activity after ") + phase);
      return true;
    }
  }
  return false;
}

// ============================================================================
// Helper: get number of active (non-deleted) entries in a row
// ============================================================================

static int row_nnz(const std::vector<MILPPresolve::Entry>& entries,
                   const std::vector<bool>& col_deleted) {
  int cnt = 0;
  for (const auto& e : entries)
    if (!col_deleted[e.col]) ++cnt;
  return cnt;
}

// ============================================================================
// fix_variable: fix col to val, update all rows containing it
// ============================================================================

void MILPPresolve::fix_variable(int col, double val) {
  if (col_deleted_[col]) return;

  // Record for postsolve
  UndoRecord rec;
  rec.type = UndoType::FixedCol;
  rec.col = col;
  rec.value = val;
  undo_stack_.push_back(std::move(rec));

  remove_col_from_activities(col);
  col_lb_[col] = val;
  col_ub_[col] = val;

  // Update RHS of all rows containing this column
  for (const auto& ce : cols_[col]) {
    if (row_deleted_[ce.row]) continue;
    // Subtract a * val from both bounds of the row
    if (std::abs(val) > opts_.zero_tol) {
      const double delta = ce.val * val;
      if (row_lb_[ce.row] > -kInf + 1) row_lb_[ce.row] -= delta;
      if (row_ub_[ce.row] < kInf - 1) row_ub_[ce.row] -= delta;
    }
  }

  col_deleted_[col] = true;
  stats_.cols_removed++;
}

// ============================================================================
// delete_row
// ============================================================================

void MILPPresolve::delete_row(int row) {
  if (row_deleted_[row]) return;
  row_deleted_[row] = true;
  stats_.rows_removed++;
}

// ============================================================================
// remove_entry / add_entry / get_entry
// ============================================================================

void MILPPresolve::remove_entry(int row, int col) {
  // Remove from row list
  auto& r = rows_[row];
  r.erase(std::remove_if(r.begin(), r.end(),
    [col](const Entry& e) { return e.col == col; }), r.end());
  // Remove from column list
  auto& c = cols_[col];
  c.erase(std::remove_if(c.begin(), c.end(),
    [row](const CEntry& e) { return e.row == row; }), c.end());
  if (!row_deleted_[row]) update_activity(row);
}

void MILPPresolve::add_entry(int row, int col, double val) {
  rows_[row].push_back({col, val});
  cols_[col].push_back({row, val});
  if (!row_deleted_[row]) update_activity(row);
}

double MILPPresolve::get_entry(int row, int col) const {
  for (const auto& e : rows_[row])
    if (e.col == col) return e.val;
  return 0.0;
}

// ============================================================================
// 1. Remove fixed variables
// ============================================================================

int MILPPresolve::remove_fixed_variables() {
  int count = 0;
  for (int j = 0; j < n_orig_; ++j) {
    if (col_deleted_[j]) continue;
    if (is_fixed(j)) {
      fix_variable(j, col_lb_[j]);
      ++count;
    }
  }
  return count;
}

// ============================================================================
// 2. Remove empty rows and columns
// ============================================================================

int MILPPresolve::remove_empty_rows_and_cols() {
  int count = 0;

  // Empty rows
  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    if (row_nnz(rows_[r], col_deleted_) == 0) {
      // Check feasibility: 0 must be within [row_lb, row_ub]
      if (row_lb_[r] > opts_.bound_tol || row_ub_[r] < -opts_.bound_tol) {
        mark_infeasible("infeasible empty row");
        return count;
      }
      delete_row(r);
      ++count;
    }
  }

  // Empty columns (no constraints refer to this variable)
  for (int j = 0; j < n_orig_; ++j) {
    if (col_deleted_[j]) continue;
    bool has_row = false;
    for (const auto& ce : cols_[j]) {
      if (!row_deleted_[ce.row]) { has_row = true; break; }
    }
    if (!has_row) {
      // Free to set to whatever minimizes cost
      double val;
      if (std::abs(col_cost_[j]) < opts_.zero_tol) {
        val = std::max(col_lb_[j], std::min(col_ub_[j], 0.0));
      } else if (col_cost_[j] > 0) {
        val = col_lb_[j];  // minimize
      } else {
        val = col_ub_[j];
      }
      if (!std::isfinite(val) || val < -kInf + 1 || val > kInf - 1) {
        continue;  // unbounded — skip
      }
      fix_variable(j, val);
      ++count;
    }
  }

  return count;
}

// ============================================================================
// 3. Singleton rows → bound tightening
// ============================================================================

int MILPPresolve::process_singleton_rows() {
  int count = 0;
  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;

    // Count active entries
    int nnz = 0;
    int singleton_col = -1;
    double singleton_val = 0;
    for (const auto& e : rows_[r]) {
      if (col_deleted_[e.col]) continue;
      ++nnz;
      singleton_col = e.col;
      singleton_val = e.val;
      if (nnz > 1) break;
    }

    if (nnz != 1 || singleton_col < 0) continue;

    // row_lb <= a * x_j <= row_ub  →  derive bounds on x_j
    bool changed = false;
    if (singleton_val > opts_.zero_tol) {
      // x_j >= row_lb / a,  x_j <= row_ub / a
      if (row_lb_[r] > -kInf + 1) {
        double new_lb = row_lb_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
        if (new_lb > col_lb_[singleton_col] + opts_.bound_tol) {
          set_col_lower_bound(singleton_col, new_lb);
          changed = true;
        }
      }
      if (row_ub_[r] < kInf - 1) {
        double new_ub = row_ub_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_ub = std::floor(new_ub + opts_.bound_tol);
        if (new_ub < col_ub_[singleton_col] - opts_.bound_tol) {
          set_col_upper_bound(singleton_col, new_ub);
          changed = true;
        }
      }
    } else if (singleton_val < -opts_.zero_tol) {
      // Division by negative flips the inequality
      if (row_ub_[r] < kInf - 1) {
        double new_lb = row_ub_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
        if (new_lb > col_lb_[singleton_col] + opts_.bound_tol) {
          set_col_lower_bound(singleton_col, new_lb);
          changed = true;
        }
      }
      if (row_lb_[r] > -kInf + 1) {
        double new_ub = row_lb_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_ub = std::floor(new_ub + opts_.bound_tol);
        if (new_ub < col_ub_[singleton_col] - opts_.bound_tol) {
          set_col_upper_bound(singleton_col, new_ub);
          changed = true;
        }
      }
    }

    if (changed) {
      stats_.singletons_removed++;
      stats_.bounds_tightened++;
    }
    delete_row(r);
    ++count;
  }
  return count;
}

// ============================================================================
// 4. Singleton columns → implied free substitution
// ============================================================================

int MILPPresolve::process_singleton_columns() {
  int count = 0;
  for (int j = 0; j < n_orig_; ++j) {
    if (col_deleted_[j]) continue;

    // Count active rows for this column
    int nnz = 0;
    int singleton_row = -1;
    double singleton_val = 0;
    for (const auto& ce : cols_[j]) {
      if (row_deleted_[ce.row]) continue;
      ++nnz;
      singleton_row = ce.row;
      singleton_val = ce.val;
      if (nnz > 1) break;
    }

    if (nnz != 1 || singleton_row < 0) continue;

    // Column appears in exactly one row.
    // If the row is an equality, we can substitute x_j out.
    // If inequality, we can tighten and still remove.
    const bool is_eq = std::abs(row_lb_[singleton_row] - row_ub_[singleton_row]) < opts_.bound_tol;

    if (is_eq) {
      if (is_integer_var(j)) {
        const double scaled_rhs = row_ub_[singleton_row] / singleton_val;
        if (std::abs(scaled_rhs - std::round(scaled_rhs)) >
            opts_.bound_tol) {
          continue;
        }
        bool preserves_integrality = true;
        for (const auto& e : rows_[singleton_row]) {
          if (e.col == j || col_deleted_[e.col]) continue;
          const double ratio = e.val / singleton_val;
          if (!is_integer_var(e.col) ||
              std::abs(ratio - std::round(ratio)) > opts_.bound_tol) {
            preserves_integrality = false;
            break;
          }
        }
        if (!preserves_integrality) continue;
      }
      // Equality row: a_j * x_j + sum(a_k * x_k) = b
      // → x_j = (b - sum(a_k * x_k)) / a_j
      // Record for postsolve, then convert x_j bounds into a band constraint
      // on the remaining row before removing the column.
      UndoRecord rec;
      rec.type = UndoType::SubstitutedCol;
      rec.col = j;
      rec.pivot_row = singleton_row;
      rec.pivot_coeff = singleton_val;
      rec.rhs = row_ub_[singleton_row];
      for (const auto& e : rows_[singleton_row]) {
        if (e.col != j && !col_deleted_[e.col]) {
          rec.row_entries.emplace_back(e.col, e.val);
        }
      }
      undo_stack_.push_back(std::move(rec));

      // Preserve the eliminated variable bounds exactly:
      //   x_j = (rhs - sum_other) / a_j,  x_j in [lb, ub]
      // implies
      //   sum_other in [min(rhs - a_j*lb, rhs - a_j*ub),
      //                 max(rhs - a_j*lb, rhs - a_j*ub)].
      const double rhs = row_ub_[singleton_row];
      double implied_lb = -kInf;
      double implied_ub = kInf;
      if (col_lb_[j] > -kInf + 1 && col_ub_[j] < kInf - 1) {
        const double lhs_at_lb = rhs - singleton_val * col_lb_[j];
        const double lhs_at_ub = rhs - singleton_val * col_ub_[j];
        implied_lb = std::min(lhs_at_lb, lhs_at_ub);
        implied_ub = std::max(lhs_at_lb, lhs_at_ub);
      } else if (col_lb_[j] > -kInf + 1) {
        if (singleton_val > 0) implied_ub = rhs - singleton_val * col_lb_[j];
        else if (singleton_val < 0) implied_lb = rhs - singleton_val * col_lb_[j];
      } else if (col_ub_[j] < kInf - 1) {
        if (singleton_val > 0) implied_lb = rhs - singleton_val * col_ub_[j];
        else if (singleton_val < 0) implied_ub = rhs - singleton_val * col_ub_[j];
      }

      remove_entry(singleton_row, j);
      row_lb_[singleton_row] = implied_lb;
      row_ub_[singleton_row] = implied_ub;

      // Transfer the eliminated variable's objective cost onto the remaining
      // variables.  Substituting x_j = (rhs - sum_k a_k x_k) / a_j turns the
      // objective term c_j*x_j into (c_j*rhs/a_j) - sum_k (c_j*a_k/a_j) x_k, so
      // each remaining var k gains -(c_j/a_j)*a_k in cost.  Omitting this makes
      // the reduced model optimize the wrong objective whenever c_j != 0 (e.g.
      // a costed generation variable reduced to a singleton by prior row
      // deletions), which silently cuts off the true optimum.  The constant
      // c_j*rhs/a_j only shifts the objective value (recovered post-postsolve)
      // and does not affect the argmin, so it is not tracked here.
      if (std::abs(col_cost_[j]) > opts_.zero_tol) {
        const double cost_ratio = col_cost_[j] / singleton_val;
        for (const auto& e : rows_[singleton_row]) {
          if (col_deleted_[e.col]) continue;
          col_cost_[e.col] -= cost_ratio * e.val;
        }
      }

      col_deleted_[j] = true;
      stats_.cols_removed++;
      stats_.singletons_removed++;
      ++count;
    } else {
      // Inequality row with singleton column: tighten x_j bound from the row,
      // then if x_j has a cost coefficient and the constraint is redundant
      // given the new bound, remove the row.
      // This is already handled by singleton row processing after the column
      // is the only one left, so we skip here to avoid complexity.
    }
  }
  return count;
}

// ============================================================================
// 5. Doubleton equations → variable substitution
// ============================================================================

int MILPPresolve::process_doubleton_equations() {
  if (!opts_.do_doubleton) return 0;
  int count = 0;

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    if (std::abs(row_lb_[r] - row_ub_[r]) > opts_.bound_tol) continue;  // not equality

    // Count active entries
    int nnz = 0;
    int col1 = -1, col2 = -1;
    double val1 = 0, val2 = 0;
    for (const auto& e : rows_[r]) {
      if (col_deleted_[e.col]) continue;
      if (nnz == 0) { col1 = e.col; val1 = e.val; }
      else if (nnz == 1) { col2 = e.col; val2 = e.val; }
      ++nnz;
      if (nnz > 2) break;
    }

    if (nnz != 2 || col1 < 0 || col2 < 0) continue;

    // Choose which variable to eliminate: prefer continuous over integer,
    // prefer the one with more constraint appearances (removes more entries).
    int elim_col, keep_col;
    double elim_coeff, keep_coeff;

    bool c1_int = is_integer_var(col1);
    bool c2_int = is_integer_var(col2);

    if (c1_int && !c2_int) {
      // Eliminate continuous col2
      elim_col = col2; elim_coeff = val2;
      keep_col = col1; keep_coeff = val1;
    } else if (!c1_int && c2_int) {
      elim_col = col1; elim_coeff = val1;
      keep_col = col2; keep_coeff = val2;
    } else {
      // Both same type: eliminate the one with fewer row appearances
      int cnt1 = 0, cnt2 = 0;
      for (const auto& ce : cols_[col1]) if (!row_deleted_[ce.row]) cnt1++;
      for (const auto& ce : cols_[col2]) if (!row_deleted_[ce.row]) cnt2++;
      if (cnt1 <= cnt2) {
        elim_col = col1; elim_coeff = val1;
        keep_col = col2; keep_coeff = val2;
      } else {
        elim_col = col2; elim_coeff = val2;
        keep_col = col1; keep_coeff = val1;
      }
    }

    // Cannot eliminate integer variable if the substitution introduces fractions
    if (is_integer_var(elim_col) && is_integer_var(keep_col)) {
      // x_elim = (rhs - keep_coeff * x_keep) / elim_coeff
      // This preserves integrality only when both the slope and intercept are
      // integer. Checking the slope alone loses divisibility constraints.
      double ratio = keep_coeff / elim_coeff;
      double intercept = row_ub_[r] / elim_coeff;
      if (std::abs(ratio - std::round(ratio)) > opts_.bound_tol ||
          std::abs(intercept - std::round(intercept)) > opts_.bound_tol) {
        continue;
      }
    }

    const double rhs = row_ub_[r];
    // x_elim = (rhs - keep_coeff * x_keep) / elim_coeff
    const double scale = -keep_coeff / elim_coeff;
    const double offset = rhs / elim_coeff;

    // Record for postsolve
    UndoRecord rec;
    rec.type = UndoType::SubstitutedCol;
    rec.col = elim_col;
    rec.pivot_row = r;
    rec.pivot_coeff = elim_coeff;
    rec.rhs = rhs;
    rec.row_entries.emplace_back(keep_col, keep_coeff);
    undo_stack_.push_back(std::move(rec));

    // Update bounds of keep variable from elim variable's bounds
    // x_elim = offset + scale * x_keep
    // If scale > 0:  col_lb[elim] <= offset + scale*x_keep <= col_ub[elim]
    //                (col_lb[elim] - offset)/scale <= x_keep <= (col_ub[elim] - offset)/scale
    // If scale < 0:  flip
    if (std::abs(scale) > opts_.zero_tol) {
      double implied_lb, implied_ub;
      if (scale > 0) {
        implied_lb = (col_lb_[elim_col] > -kInf + 1)
            ? (col_lb_[elim_col] - offset) / scale : -kInf;
        implied_ub = (col_ub_[elim_col] < kInf - 1)
            ? (col_ub_[elim_col] - offset) / scale : kInf;
      } else {
        implied_lb = (col_ub_[elim_col] < kInf - 1)
            ? (col_ub_[elim_col] - offset) / scale : -kInf;
        implied_ub = (col_lb_[elim_col] > -kInf + 1)
            ? (col_lb_[elim_col] - offset) / scale : kInf;
      }
      if (is_integer_var(keep_col)) {
        if (implied_lb > -kInf + 1)
          implied_lb = std::ceil(implied_lb - opts_.bound_tol);
        if (implied_ub < kInf - 1)
          implied_ub = std::floor(implied_ub + opts_.bound_tol);
      }
      if (implied_lb > col_lb_[keep_col] + opts_.bound_tol)
        set_col_lower_bound(keep_col, implied_lb);
      if (implied_ub < col_ub_[keep_col] - opts_.bound_tol)
        set_col_upper_bound(keep_col, implied_ub);
    }

    // Substitute x_elim into all other rows that reference it.
    // Iterate over a copy because remove_entry mutates cols_[elim_col].
    const auto elim_col_rows = cols_[elim_col];
    for (const auto& ce : elim_col_rows) {
      if (row_deleted_[ce.row]) continue;
      if (ce.row == r) continue;

      const double a_elim = ce.val;  // coefficient of elim_col in this row

      // Remove elim_col from this row
      remove_entry(ce.row, elim_col);

      // Add (a_elim * scale) to the coefficient of keep_col in this row
      double existing = get_entry(ce.row, keep_col);
      double new_coeff = existing + a_elim * scale;

      if (std::abs(existing) > opts_.zero_tol) {
        // Update existing entry
        for (auto& e : rows_[ce.row]) {
          if (e.col == keep_col) { e.val = new_coeff; break; }
        }
        for (auto& c : cols_[keep_col]) {
          if (c.row == ce.row) { c.val = new_coeff; break; }
        }
      } else if (std::abs(new_coeff) > opts_.zero_tol) {
        // Add new entry
        add_entry(ce.row, keep_col, new_coeff);
      }

      // Adjust RHS: row_lb/ub -= a_elim * offset
      const double rhs_delta = a_elim * offset;
      if (row_lb_[ce.row] > -kInf + 1) row_lb_[ce.row] -= rhs_delta;
      if (row_ub_[ce.row] < kInf - 1) row_ub_[ce.row] -= rhs_delta;

      // Remove near-zero entries
      if (std::abs(new_coeff) <= opts_.zero_tol) {
        remove_entry(ce.row, keep_col);
      }
      update_activity(ce.row);
    }

    // Update objective: cost_keep += cost_elim * scale;  cost_offset += cost_elim * offset
    col_cost_[keep_col] += col_cost_[elim_col] * scale;

    col_deleted_[elim_col] = true;
    stats_.cols_removed++;
    delete_row(r);
    stats_.doubletons_removed++;
    ++count;
  }
  return count;
}

// ============================================================================
// 6. Bound tightening
// ============================================================================

int MILPPresolve::tighten_bounds() {
  int count = 0;

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    const auto& ac = row_activity_[r];

    for (const auto& e : rows_[r]) {
      if (col_deleted_[e.col]) continue;

      // Upper bound tightening from row_ub: a^T x <= row_ub
      if (row_ub_[r] < kInf - 1 && ac.n_inf_min == 0) {
        // min_activity_excl = min_act - contribution of e.col
        const double bound =
            e.val > 0 ? col_lb_[e.col] : col_ub_[e.col];
        const double residual = ac.min_act.upper_residual(
            row_ub_[r], e.val, bound, opts_.bound_tol);

        if (e.val > opts_.zero_tol) {
          double new_ub = residual / e.val;
          if (is_integer_var(e.col)) new_ub = std::floor(new_ub + opts_.bound_tol);
          if (new_ub < col_ub_[e.col] - detail::bound_improvement_tolerance(
                                                   opts_.bound_tol,
                                                   col_ub_[e.col], new_ub)) {
            set_col_upper_bound(e.col, new_ub);
            ++count;
          }
        } else if (e.val < -opts_.zero_tol) {
          double new_lb = residual / e.val;
          if (is_integer_var(e.col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
          if (new_lb > col_lb_[e.col] + detail::bound_improvement_tolerance(
                                                   opts_.bound_tol,
                                                   col_lb_[e.col], new_lb)) {
            set_col_lower_bound(e.col, new_lb);
            ++count;
          }
        }
      }

      // Lower bound tightening from row_lb: row_lb <= a^T x
      if (row_lb_[r] > -kInf + 1 && ac.n_inf_max == 0) {
        const double bound =
            e.val > 0 ? col_ub_[e.col] : col_lb_[e.col];
        const double residual = ac.max_act.lower_residual(
            row_lb_[r], e.val, bound, opts_.bound_tol);

        if (e.val > opts_.zero_tol) {
          double new_lb = residual / e.val;
          if (is_integer_var(e.col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
          if (new_lb > col_lb_[e.col] + detail::bound_improvement_tolerance(
                                                   opts_.bound_tol,
                                                   col_lb_[e.col], new_lb)) {
            set_col_lower_bound(e.col, new_lb);
            ++count;
          }
        } else if (e.val < -opts_.zero_tol) {
          double new_ub = residual / e.val;
          if (is_integer_var(e.col)) new_ub = std::floor(new_ub + opts_.bound_tol);
          if (new_ub < col_ub_[e.col] - detail::bound_improvement_tolerance(
                                                   opts_.bound_tol,
                                                   col_ub_[e.col], new_ub)) {
            set_col_upper_bound(e.col, new_ub);
            ++count;
          }
        }
      }

      // Special case: finite min activity with exactly 1 infinite contributor
      // → can tighten even with one inf
      if (row_ub_[r] < kInf - 1 && ac.n_inf_min == 1) {
        // Check if this variable is the infinite contributor
        bool this_is_inf = (e.val > 0 && col_lb_[e.col] <= -kInf + 1) ||
                           (e.val < 0 && col_ub_[e.col] >= kInf - 1);
        if (this_is_inf) {
          // min_activity_excl is finite, so we can derive a bound
          double contrib_finite = 0.0;
          bool ok = true;
          for (const auto& e2 : rows_[r]) {
            if (col_deleted_[e2.col] || e2.col == e.col) continue;
            if (e2.val > 0) {
              if (col_lb_[e2.col] <= -kInf + 1) { ok = false; break; }
              contrib_finite += e2.val * col_lb_[e2.col];
            } else {
              if (col_ub_[e2.col] >= kInf - 1) { ok = false; break; }
              contrib_finite += e2.val * col_ub_[e2.col];
            }
          }
          if (ok) {
            double residual = row_ub_[r] - contrib_finite;
            if (e.val > opts_.zero_tol) {
              double new_ub = residual / e.val;
              if (is_integer_var(e.col)) new_ub = std::floor(new_ub + opts_.bound_tol);
              if (new_ub < col_ub_[e.col] - opts_.bound_tol || col_ub_[e.col] >= kInf - 1) {
                set_col_upper_bound(e.col, new_ub);
                ++count;
              }
            } else if (e.val < -opts_.zero_tol) {
              double new_lb = residual / e.val;
              if (is_integer_var(e.col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
              if (new_lb > col_lb_[e.col] + opts_.bound_tol || col_lb_[e.col] <= -kInf + 1) {
                set_col_lower_bound(e.col, new_lb);
                ++count;
              }
            }
          }
        }
      }
    }
  }
  stats_.bounds_tightened += count;
  return count;
}

// ============================================================================
// 7. Forcing rows
// ============================================================================

int MILPPresolve::detect_forcing_rows() {
  int count = 0;

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    const auto& ac = row_activity_[r];

    // Redundant row: max_activity <= row_ub AND min_activity >= row_lb
    const bool max_below_ub =
        ac.n_inf_max == 0 && ac.max_act.within_upper(row_ub_[r], opts_.bound_tol);
    const bool min_above_lb =
        ac.n_inf_min == 0 && ac.min_act.within_lower(row_lb_[r], opts_.bound_tol);

    if (max_below_ub && (row_lb_[r] <= -kInf + 1 || min_above_lb)) {
      delete_row(r);
      ++count;
      continue;
    }

    // Forcing row at min: min_activity == row_ub → all vars at their contributing bound
    if (ac.n_inf_min == 0 &&
        ac.min_act.within_lower(row_ub_[r], opts_.bound_tol)) {
      for (const auto& e : rows_[r]) {
        if (col_deleted_[e.col]) continue;
        if (e.val > opts_.zero_tol) {
          // x_j at lower bound to achieve min activity
          fix_variable(e.col, col_lb_[e.col]);
        } else if (e.val < -opts_.zero_tol) {
          fix_variable(e.col, col_ub_[e.col]);
        }
      }
      delete_row(r);
      stats_.forcing_rows++;
      ++count;
      continue;
    }

    // Forcing row at max: max_activity == row_lb → all vars at opposite bound
    if (ac.n_inf_max == 0 && row_lb_[r] > -kInf + 1 &&
        ac.max_act.within_upper(row_lb_[r], opts_.bound_tol)) {
      for (const auto& e : rows_[r]) {
        if (col_deleted_[e.col]) continue;
        if (e.val > opts_.zero_tol) {
          fix_variable(e.col, col_ub_[e.col]);
        } else if (e.val < -opts_.zero_tol) {
          fix_variable(e.col, col_lb_[e.col]);
        }
      }
      delete_row(r);
      stats_.forcing_rows++;
      ++count;
      continue;
    }
  }
  return count;
}

// ============================================================================
// 8. Dominated columns
// ============================================================================

int MILPPresolve::detect_dominated_columns() {
  if (!opts_.do_dominated) return 0;
  int count = 0;

  for (int j = 0; j < n_orig_; ++j) {
    if (col_deleted_[j]) continue;
    if (!std::isfinite(col_lb_[j]) && !std::isfinite(col_ub_[j])) continue;

    // Check if column j can be fixed to one of its bounds based on cost
    // and constraint structure (weakly dominated).
    // A column is dominated at its lb if: for every row containing j,
    // decreasing x_j (toward lb) never violates the row.
    // This is true if c_j >= 0 (minimizing, pushing to lb helps cost)
    // and in every row, reducing x_j either loosens the constraint or is neutral.

    // Simple cases for binary/bounded variables with cost dominance:
    if (col_cost_[j] > opts_.zero_tol && col_lb_[j] > -kInf + 1) {
      // Cost pushes x_j to lower bound. Check if x_j only appears with
      // non-negative coefficients in <= constraints and non-positive in >= constraints.
      bool can_fix_lb = true;
      for (const auto& ce : cols_[j]) {
        if (row_deleted_[ce.row]) continue;
        // For row_lb <= a^T x <= row_ub:
        // If a_j > 0: reducing x_j reduces a^T x → may violate row_lb
        // If a_j < 0: reducing x_j increases a^T x → may violate row_ub
        if (ce.val > opts_.zero_tol && row_lb_[ce.row] > -kInf + 1) {
          can_fix_lb = false; break;
        }
        if (ce.val < -opts_.zero_tol && row_ub_[ce.row] < kInf - 1) {
          can_fix_lb = false; break;
        }
      }
      if (can_fix_lb) {
        fix_variable(j, col_lb_[j]);
        stats_.dominated_cols++;
        ++count;
        continue;
      }
    }

    if (col_cost_[j] < -opts_.zero_tol && col_ub_[j] < kInf - 1) {
      bool can_fix_ub = true;
      for (const auto& ce : cols_[j]) {
        if (row_deleted_[ce.row]) continue;
        if (ce.val > opts_.zero_tol && row_ub_[ce.row] < kInf - 1) {
          can_fix_ub = false; break;
        }
        if (ce.val < -opts_.zero_tol && row_lb_[ce.row] > -kInf + 1) {
          can_fix_ub = false; break;
        }
      }
      if (can_fix_ub) {
        fix_variable(j, col_ub_[j]);
        stats_.dominated_cols++;
        ++count;
        continue;
      }
    }
  }
  return count;
}

// ============================================================================
// 9. Parallel row detection
// ============================================================================

int MILPPresolve::detect_parallel_rows() {
  if (!opts_.do_parallel_rows) return 0;
  int count = 0;

  // Hash-based detection: hash each row's column pattern, compare candidates
  std::unordered_map<size_t, std::vector<int>> row_hash;

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    int nnz = row_nnz(rows_[r], col_deleted_);
    if (nnz < 1) continue;

    // Simple hash: combine column indices
    size_t h = 0;
    for (const auto& e : rows_[r]) {
      if (col_deleted_[e.col]) continue;
      h ^= std::hash<int>()(e.col) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    h ^= std::hash<int>()(nnz);
    row_hash[h].push_back(r);
  }

  for (auto& [h, rows] : row_hash) {
    if (rows.size() < 2) continue;

    for (size_t i = 0; i < rows.size(); ++i) {
      if (row_deleted_[rows[i]]) continue;
      for (size_t k = i + 1; k < rows.size(); ++k) {
        if (row_deleted_[rows[k]]) continue;

        const int r1 = rows[i], r2 = rows[k];

        // Check if rows are parallel: same column pattern, proportional coefficients
        // Collect active entries
        std::vector<std::pair<int, double>> e1, e2;
        for (const auto& e : rows_[r1])
          if (!col_deleted_[e.col]) e1.emplace_back(e.col, e.val);
        for (const auto& e : rows_[r2])
          if (!col_deleted_[e.col]) e2.emplace_back(e.col, e.val);

        if (e1.size() != e2.size()) continue;

        // Sort by column
        std::sort(e1.begin(), e1.end());
        std::sort(e2.begin(), e2.end());

        bool parallel = true;
        double ratio = 0.0;
        bool ratio_set = false;
        for (size_t idx = 0; idx < e1.size(); ++idx) {
          if (e1[idx].first != e2[idx].first) { parallel = false; break; }
          if (std::abs(e2[idx].second) < opts_.zero_tol) {
            if (std::abs(e1[idx].second) > opts_.zero_tol) { parallel = false; break; }
            continue;
          }
          double r_check = e1[idx].second / e2[idx].second;
          if (!ratio_set) { ratio = r_check; ratio_set = true; }
          else if (std::abs(r_check - ratio) > opts_.bound_tol * std::max(1.0, std::abs(ratio))) {
            parallel = false; break;
          }
        }

        if (!parallel || !ratio_set) continue;

        // Merge row2 into row1: keep the tighter constraint. Since
        // ratio = a(row1) / a(row2), row2's bounds must be multiplied by
        // ratio to match row1. When ratio < 0, multiplication flips the
        // inequality direction, so
        // what was the lower bound becomes the upper and vice versa.
        double lb2_scaled, ub2_scaled;
        if (ratio > 0) {
          lb2_scaled = (row_lb_[r2] > -kInf + 1) ? row_lb_[r2] * ratio : -kInf;
          ub2_scaled = (row_ub_[r2] < kInf - 1)  ? row_ub_[r2] * ratio : kInf;
        } else {
          // ratio < 0: scaling flips direction
          lb2_scaled = (row_ub_[r2] < kInf - 1)  ? row_ub_[r2] * ratio : -kInf;
          ub2_scaled = (row_lb_[r2] > -kInf + 1) ? row_lb_[r2] * ratio : kInf;
        }

        // Tighten row1's bounds
        if (lb2_scaled > row_lb_[r1] + opts_.bound_tol) row_lb_[r1] = lb2_scaled;
        if (ub2_scaled < row_ub_[r1] - opts_.bound_tol) row_ub_[r1] = ub2_scaled;

        delete_row(r2);
        stats_.parallel_rows++;
        ++count;
      }
    }
  }
  return count;
}

// ============================================================================
// 10. Binary probing
// ============================================================================

int MILPPresolve::run_probing() {
  int count = 0;
  std::vector<int> candidates;
  candidates.reserve(static_cast<std::size_t>(n_orig_));
  for (int j = 0; j < n_orig_; ++j) {
    if (!col_deleted_[j] && col_type_[j] == VarType::Binary &&
        !is_fixed(j)) {
      candidates.push_back(j);
    }
  }

  int max_probes =
      std::min(opts_.max_probing_candidates,
               static_cast<int>(candidates.size()));
  if (m_orig_ > 2000) max_probes = std::min(max_probes, 100);
  if (m_orig_ > 5000) max_probes = std::min(max_probes, 50);

  struct TrailEntry {
    int col;
    bool is_lb;
    double old_value;
  };
  struct TouchedColumn {
    int col;
    double base_lb;
    double base_ub;
  };
  struct WorldChange {
    int col;
    double base_lb;
    double base_ub;
    double lb;
    double ub;
  };
  struct ProbeWorld {
    bool infeasible{false};
    bool complete{true};
    std::vector<WorldChange> changes;
  };

  std::vector<TrailEntry> trail;
  std::vector<TouchedColumn> touched;
  std::vector<unsigned char> touched_flag(
      static_cast<std::size_t>(n_orig_), 0);
  std::vector<unsigned char> row_queued(
      static_cast<std::size_t>(m_orig_), 0);
  std::vector<int> pending_rows;
  std::vector<int> current_rows;

  auto record_implication = [&](int trigger_col, bool trigger_value_one,
                                int implied_col, bool implied_is_lb,
                                double implied_value) {
    if (trigger_col == implied_col || !std::isfinite(implied_value)) return;
    if (probing_implications_original_.size() >=
        opts_.max_probing_implications) {
      stats_.probing_truncated = true;
      return;
    }
    probing_implications_original_.push_back(
        {trigger_col, trigger_value_one, implied_col, implied_is_lb,
         implied_value});
  };

  auto normalize_implications = [&]() {
    auto& implications = probing_implications_original_;
    std::sort(implications.begin(), implications.end(),
              [](const ProbingImplication& lhs,
                 const ProbingImplication& rhs) {
                if (lhs.trigger_col != rhs.trigger_col)
                  return lhs.trigger_col < rhs.trigger_col;
                if (lhs.trigger_value_one != rhs.trigger_value_one)
                  return lhs.trigger_value_one < rhs.trigger_value_one;
                if (lhs.implied_col != rhs.implied_col)
                  return lhs.implied_col < rhs.implied_col;
                if (lhs.implied_is_lb != rhs.implied_is_lb)
                  return lhs.implied_is_lb < rhs.implied_is_lb;
                return lhs.implied_is_lb
                           ? lhs.implied_value > rhs.implied_value
                           : lhs.implied_value < rhs.implied_value;
              });
    std::vector<ProbingImplication> unique;
    unique.reserve(implications.size());
    for (const auto& implication : implications) {
      if (!unique.empty()) {
        const auto& previous = unique.back();
        if (previous.trigger_col == implication.trigger_col &&
            previous.trigger_value_one == implication.trigger_value_one &&
            previous.implied_col == implication.implied_col &&
            previous.implied_is_lb == implication.implied_is_lb) {
          continue;
        }
      }
      unique.push_back(implication);
    }
    implications.swap(unique);
  };

  auto run_world = [&](int trigger_col, double trigger_value) {
    ProbeWorld world;
    trail.clear();
    touched.clear();
    pending_rows.clear();
    current_rows.clear();

    auto touch_col = [&](int col) {
      auto& flag = touched_flag[static_cast<std::size_t>(col)];
      if (flag) return;
      flag = 1;
      touched.push_back({col, col_lb_[col], col_ub_[col]});
    };

    auto enqueue_col_rows = [&](int col) {
      for (const auto& ce : cols_[col]) {
        const int row = ce.row;
        if (row_deleted_[row]) continue;
        auto& queued = row_queued[static_cast<std::size_t>(row)];
        if (!queued) {
          queued = 1;
          pending_rows.push_back(row);
        }
      }
    };

    auto tighten_bound = [&](int col, bool is_lb,
                             double candidate) -> bool {
      if (col_deleted_[col] || !std::isfinite(candidate)) return true;
      if (is_integer_var(col)) {
        candidate =
            is_lb ? std::ceil(candidate - opts_.bound_tol)
                  : std::floor(candidate + opts_.bound_tol);
      }
      const double current = is_lb ? col_lb_[col] : col_ub_[col];
      const double improve_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, current, candidate);
      const bool improves =
          is_lb ? candidate > current + improve_tol
                : candidate < current - improve_tol;
      if (!improves) return true;

      const double opposite = is_lb ? col_ub_[col] : col_lb_[col];
      const double consistency_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, candidate, opposite);
      if ((is_lb && candidate > opposite + consistency_tol) ||
          (!is_lb && candidate < opposite - consistency_tol)) {
        world.infeasible = true;
        return false;
      }

      touch_col(col);
      trail.push_back({col, is_lb, current});
      ++stats_.probing_trail_pushes;
      if (is_lb)
        set_col_lower_bound(col, candidate);
      else
        set_col_upper_bound(col, candidate);
      enqueue_col_rows(col);
      return true;
    };

    tighten_bound(trigger_col, true, trigger_value);
    if (!world.infeasible)
      tighten_bound(trigger_col, false, trigger_value);

    const int max_waves = std::max(1, opts_.probing_depth);
    for (int wave = 0;
         world.complete && !world.infeasible && wave < max_waves &&
         !pending_rows.empty();
         ++wave) {
      if (deadline_expired()) {
        world.complete = false;
        stats_.timed_out = true;
        break;
      }
      current_rows.clear();
      current_rows.swap(pending_rows);
      for (const int row : current_rows) {
        if (stats_.probing_rows_processed >= opts_.max_probing_row_visits ||
            deadline_expired()) {
          world.complete = false;
          if (deadline_expired()) stats_.timed_out = true;
          stats_.probing_truncated = true;
          break;
        }
        row_queued[static_cast<std::size_t>(row)] = 0;
        if (row_deleted_[row]) continue;
        ++stats_.probing_rows_processed;
        const auto& activity = row_activity_[row];

        if (row_ub_[row] < kInf - 1 && activity.n_inf_min == 0 &&
            activity.min_act.violates_upper(row_ub_[row],
                                            opts_.bound_tol)) {
          world.infeasible = true;
          break;
        }
        if (row_lb_[row] > -kInf + 1 && activity.n_inf_max == 0 &&
            activity.max_act.violates_lower(row_lb_[row],
                                            opts_.bound_tol)) {
          world.infeasible = true;
          break;
        }

        for (const auto& entry : rows_[row]) {
          if (col_deleted_[entry.col]) continue;
          if (row_ub_[row] < kInf - 1 && activity.n_inf_min == 0) {
            const double activity_bound =
                entry.val > 0.0 ? col_lb_[entry.col] : col_ub_[entry.col];
            const double residual = activity.min_act.upper_residual(
                row_ub_[row], entry.val, activity_bound, opts_.bound_tol);
            if (!tighten_bound(entry.col, entry.val < 0.0,
                               residual / entry.val)) {
              break;
            }
          }
          if (row_lb_[row] > -kInf + 1 && activity.n_inf_max == 0) {
            const double activity_bound =
                entry.val > 0.0 ? col_ub_[entry.col] : col_lb_[entry.col];
            const double residual = activity.max_act.lower_residual(
                row_lb_[row], entry.val, activity_bound, opts_.bound_tol);
            if (!tighten_bound(entry.col, entry.val > 0.0,
                               residual / entry.val)) {
              break;
            }
          }
        }
        if (world.infeasible) break;
      }
    }

    if (world.complete && !world.infeasible) {
      world.changes.reserve(touched.size());
      for (const auto& column : touched) {
        world.changes.push_back(
            {column.col, column.base_lb, column.base_ub,
             col_lb_[column.col], col_ub_[column.col]});
      }
      std::sort(world.changes.begin(), world.changes.end(),
                [](const WorldChange& lhs, const WorldChange& rhs) {
                  return lhs.col < rhs.col;
                });
      stats_.probing_max_touched_cols =
          std::max(stats_.probing_max_touched_cols,
                   static_cast<std::uint64_t>(world.changes.size()));
    }

    for (auto it = trail.rbegin(); it != trail.rend(); ++it) {
      if (it->is_lb)
        set_col_lower_bound(it->col, it->old_value);
      else
        set_col_upper_bound(it->col, it->old_value);
    }
    for (const auto& column : touched)
      touched_flag[static_cast<std::size_t>(column.col)] = 0;
    for (const int row : pending_rows)
      row_queued[static_cast<std::size_t>(row)] = 0;
    for (const int row : current_rows)
      row_queued[static_cast<std::size_t>(row)] = 0;

    return world;
  };

  auto apply_world_unconditionally = [&](const ProbeWorld& world,
                                         int trigger_col) {
    for (const auto& change : world.changes) {
      if (change.col == trigger_col || col_deleted_[change.col]) continue;
      const double lb_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, col_lb_[change.col], change.lb);
      if (change.lb > col_lb_[change.col] + lb_tol) {
        set_col_lower_bound(change.col, change.lb);
        ++stats_.bounds_tightened;
        ++count;
      }
      const double ub_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, col_ub_[change.col], change.ub);
      if (change.ub < col_ub_[change.col] - ub_tol) {
        set_col_upper_bound(change.col, change.ub);
        ++stats_.bounds_tightened;
        ++count;
      }
    }
  };

  for (int probe = 0; probe < max_probes; ++probe) {
    if ((probe & 255) == 0 && deadline_expired()) {
      stats_.timed_out = true;
      return 0;
    }
    if (probing_implications_original_.size() >=
            opts_.max_probing_implications ||
        stats_.probing_rows_processed >= opts_.max_probing_row_visits ||
        deadline_expired()) {
      stats_.probing_truncated = true;
      if (deadline_expired()) stats_.timed_out = true;
      break;
    }
    const int trigger = candidates[static_cast<std::size_t>(probe)];
    if (col_deleted_[trigger] || is_fixed(trigger)) continue;

    const ProbeWorld zero = run_world(trigger, 0.0);
    if (!zero.complete) break;
    const ProbeWorld one = run_world(trigger, 1.0);
    if (!one.complete) break;

    if (zero.infeasible && one.infeasible) {
      mark_infeasible("both binary probing branches are infeasible");
      break;
    }
    if (zero.infeasible || one.infeasible) {
      const bool fixed_one = zero.infeasible;
      set_col_lower_bound(trigger, fixed_one ? 1.0 : 0.0);
      set_col_upper_bound(trigger, fixed_one ? 1.0 : 0.0);
      ++stats_.probing_fixings;
      ++count;
      apply_world_unconditionally(fixed_one ? one : zero, trigger);
      continue;
    }

    for (const auto& change : zero.changes) {
      if (change.col == trigger) continue;
      const double lb_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, change.base_lb, change.lb);
      if (change.lb > change.base_lb + lb_tol)
        record_implication(trigger, false, change.col, true, change.lb);
      const double ub_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, change.base_ub, change.ub);
      if (change.ub < change.base_ub - ub_tol)
        record_implication(trigger, false, change.col, false, change.ub);
    }
    for (const auto& change : one.changes) {
      if (change.col == trigger) continue;
      const double lb_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, change.base_lb, change.lb);
      if (change.lb > change.base_lb + lb_tol)
        record_implication(trigger, true, change.col, true, change.lb);
      const double ub_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, change.base_ub, change.ub);
      if (change.ub < change.base_ub - ub_tol)
        record_implication(trigger, true, change.col, false, change.ub);
    }

    std::size_t zero_pos = 0;
    std::size_t one_pos = 0;
    while (zero_pos < zero.changes.size() ||
           one_pos < one.changes.size()) {
      const WorldChange* zero_change = nullptr;
      const WorldChange* one_change = nullptr;
      int col = -1;
      if (one_pos >= one.changes.size() ||
          (zero_pos < zero.changes.size() &&
           zero.changes[zero_pos].col < one.changes[one_pos].col)) {
        zero_change = &zero.changes[zero_pos++];
        col = zero_change->col;
      } else if (zero_pos >= zero.changes.size() ||
                 one.changes[one_pos].col <
                     zero.changes[zero_pos].col) {
        one_change = &one.changes[one_pos++];
        col = one_change->col;
      } else {
        zero_change = &zero.changes[zero_pos++];
        one_change = &one.changes[one_pos++];
        col = zero_change->col;
      }
      if (col == trigger || col_deleted_[col]) continue;
      const double base_lb =
          zero_change ? zero_change->base_lb : one_change->base_lb;
      const double base_ub =
          zero_change ? zero_change->base_ub : one_change->base_ub;
      const double zero_lb = zero_change ? zero_change->lb : base_lb;
      const double one_lb = one_change ? one_change->lb : base_lb;
      const double zero_ub = zero_change ? zero_change->ub : base_ub;
      const double one_ub = one_change ? one_change->ub : base_ub;
      const double common_lb = std::min(zero_lb, one_lb);
      const double common_ub = std::max(zero_ub, one_ub);

      const double lb_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, col_lb_[col], common_lb);
      if (common_lb > col_lb_[col] + lb_tol) {
        set_col_lower_bound(col, common_lb);
        ++stats_.bounds_tightened;
        ++count;
      }
      const double ub_tol = detail::bound_improvement_tolerance(
          opts_.bound_tol, col_ub_[col], common_ub);
      if (common_ub < col_ub_[col] - ub_tol) {
        set_col_upper_bound(col, common_ub);
        ++stats_.bounds_tightened;
        ++count;
      }
    }
  }

  normalize_implications();
  stats_.probing_implications_learned =
      static_cast<std::uint64_t>(probing_implications_original_.size());
  return count;
}

// ============================================================================
// rebuild_model: construct reduced LPModel from internal state
// ============================================================================

void MILPPresolve::rebuild_model(LPModel& lp, std::vector<int>& binary_idx,
                                  std::vector<int>& integer_idx) {
  // Build column mapping (orig → reduced)
  orig_to_reduced_col_.assign(n_orig_, -1);
  reduced_to_orig_col_.clear();
  int n_reduced = 0;
  for (int j = 0; j < n_orig_; ++j) {
    if (!col_deleted_[j]) {
      orig_to_reduced_col_[j] = n_reduced;
      reduced_to_orig_col_.push_back(j);
      ++n_reduced;
    }
  }

  // Build row mapping
  orig_to_reduced_row_.assign(m_orig_, -1);
  std::vector<int> reduced_to_orig_row;
  int m_ineq_reduced = 0, m_eq_reduced = 0;

  // First pass: count reduced ineq and eq rows
  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    bool is_eq = std::abs(row_lb_[r] - row_ub_[r]) < opts_.bound_tol;
    if (is_eq) {
      m_eq_reduced++;
    } else {
      // Non-equality row: emit one ≤-inequality for each finite side.
      if (row_ub_[r] < kInf - 1) m_ineq_reduced++;
      if (row_lb_[r] > -kInf + 1) m_ineq_reduced++;
    }
  }

  // Second pass: build matrices
  // Separate back into A*x <= b and Aeq*x = beq format
  std::vector<Eigen::Triplet<double>> ineq_trips, eq_trips;
  Eigen::VectorXd new_b(m_ineq_reduced);
  Eigen::VectorXd new_beq(m_eq_reduced);
  int ineq_row = 0, eq_row = 0;

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    bool is_eq = std::abs(row_lb_[r] - row_ub_[r]) < opts_.bound_tol;

    if (is_eq) {
      for (const auto& e : rows_[r]) {
        if (col_deleted_[e.col]) continue;
        eq_trips.emplace_back(eq_row, orig_to_reduced_col_[e.col], e.val);
      }
      new_beq[eq_row] = row_ub_[r];
      orig_to_reduced_row_[r] = eq_row;
      ++eq_row;
    } else {
      // Inequality / range: emit ≤-inequality for each finite side.
      // row_lb_ <= a^T x <= row_ub_
      //   →  a^T x <= row_ub_          (if row_ub_ finite)
      //   → -a^T x <= -row_lb_         (if row_lb_ finite)
      bool emitted_primary = false;
      if (row_ub_[r] < kInf - 1) {
        for (const auto& e : rows_[r]) {
          if (col_deleted_[e.col]) continue;
          ineq_trips.emplace_back(ineq_row, orig_to_reduced_col_[e.col], e.val);
        }
        new_b[ineq_row] = row_ub_[r];
        orig_to_reduced_row_[r] = ineq_row;
        ++ineq_row;
        emitted_primary = true;
      }
      if (row_lb_[r] > -kInf + 1) {
        for (const auto& e : rows_[r]) {
          if (col_deleted_[e.col]) continue;
          ineq_trips.emplace_back(ineq_row, orig_to_reduced_col_[e.col], -e.val);
        }
        new_b[ineq_row] = -row_lb_[r];
        if (!emitted_primary) orig_to_reduced_row_[r] = ineq_row;
        ++ineq_row;
      }
    }
  }

  // Build reduced LPModel
  lp.c = Eigen::VectorXd::Zero(n_reduced);
  lp.vars.resize(n_reduced);
  for (int j = 0; j < n_reduced; ++j) {
    int orig = reduced_to_orig_col_[j];
    lp.c[j] = col_cost_[orig];
    lp.vars[j].type = col_type_[orig];
    lp.vars[j].lb = col_lb_[orig];
    lp.vars[j].ub = col_ub_[orig];
  }

  Eigen::SparseMatrix<double> A(m_ineq_reduced, n_reduced);
  A.setFromTriplets(ineq_trips.begin(), ineq_trips.end());
  A.makeCompressed();
  lp.A = std::move(A);
  lp.b = std::move(new_b);
  // Every finite lower side was emitted above as a negated <= row. Keeping
  // the source model's row_lhs would attach stale lower bounds to unrelated
  // reduced rows whenever the old and new row counts happen to match.
  lp.row_lhs.resize(0);

  Eigen::SparseMatrix<double> Aeq(m_eq_reduced, n_reduced);
  Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
  Aeq.makeCompressed();
  lp.Aeq = std::move(Aeq);
  lp.beq = std::move(new_beq);

  // Update binary/integer index lists
  binary_idx.clear();
  integer_idx.clear();
  for (int j = 0; j < n_reduced; ++j) {
    if (lp.vars[j].type == VarType::Binary) binary_idx.push_back(j);
    else if (lp.vars[j].type == VarType::Integer) integer_idx.push_back(j);
  }

  probing_implications_reduced_.clear();
  probing_implications_reduced_.reserve(
      probing_implications_original_.size());
  for (const auto& implication : probing_implications_original_) {
    if (implication.trigger_col < 0 || implication.trigger_col >= n_orig_ ||
        implication.implied_col < 0 || implication.implied_col >= n_orig_) {
      continue;
    }
    const int reduced_trigger =
        orig_to_reduced_col_[implication.trigger_col];
    const int reduced_implied =
        orig_to_reduced_col_[implication.implied_col];
    if (reduced_trigger < 0 || reduced_implied < 0 ||
        reduced_trigger == reduced_implied) {
      continue;
    }
    const double global_bound = implication.implied_is_lb
                                    ? col_lb_[implication.implied_col]
                                    : col_ub_[implication.implied_col];
    const double tolerance = detail::bound_improvement_tolerance(
        opts_.bound_tol, global_bound, implication.implied_value);
    const bool globally_redundant =
        implication.implied_is_lb
            ? implication.implied_value <= global_bound + tolerance
            : implication.implied_value >= global_bound - tolerance;
    if (globally_redundant) continue;
    probing_implications_reduced_.push_back(
        {reduced_trigger, implication.trigger_value_one, reduced_implied,
         implication.implied_is_lb, implication.implied_value});
  }
  stats_.probing_implications_learned =
      static_cast<std::uint64_t>(probing_implications_reduced_.size());

  // Stats
  stats_.final_cols = n_reduced;
  stats_.final_rows = m_ineq_reduced + m_eq_reduced;
  stats_.final_nnz =
      static_cast<int>(lp.A.nonZeros() + lp.Aeq.nonZeros());
}

// ============================================================================
// postsolve: recover original solution
// ============================================================================

Eigen::VectorXd MILPPresolve::postsolve(const Eigen::VectorXd& x_reduced) const {
  Eigen::VectorXd x(n_orig_);
  x.setZero();

  // Map reduced solution to original variables
  for (int j = 0; j < static_cast<int>(reduced_to_orig_col_.size()); ++j) {
    if (j < x_reduced.size()) {
      x[reduced_to_orig_col_[j]] = x_reduced[j];
    }
  }

  // Optional per-undo-record diagnostic: walk the undo stack in reverse and
  // after each record check the restored x against the pre-presolve bounds.
  // Gated by HACDCPF_PRESOLVE_POSTSOLVE_DEBUG. Logs the first record that
  // pushes any variable out of [col_lb_orig_, col_ub_orig_].
  const char* debug_env = bc_env_options().value("HACDCPF_PRESOLVE_POSTSOLVE_DEBUG");
  const bool debug_on = debug_env && debug_env[0] != '\0' && debug_env[0] != '0';
  const double debug_tol = 1e-6;
  auto check_var_bounds = [&](int col) -> double {
    if (col < 0 || col >= static_cast<int>(col_lb_orig_.size())) return 0.0;
    const double lb = col_lb_orig_[col];
    const double ub = col_ub_orig_[col];
    const double xv = x[col];
    double viol = 0.0;
    if (lb > -kInf + 1 && xv < lb - debug_tol) viol = std::max(viol, lb - xv);
    if (ub <  kInf - 1 && xv > ub + debug_tol) viol = std::max(viol, xv - ub);
    return viol;
  };
  auto type_name = [](UndoType t) {
    switch (t) {
      case UndoType::FixedCol: return "FixedCol";
      case UndoType::SubstitutedCol: return "SubstitutedCol";
      case UndoType::RedundantRow: return "RedundantRow";
    }
    return "?";
  };
  bool reported_initial = false;
  if (debug_on) {
    // Scan reduced-space mapped vars first.
    double worst = 0.0; int worst_col = -1;
    for (int j : reduced_to_orig_col_) {
      double v = check_var_bounds(j);
      if (v > worst) { worst = v; worst_col = j; }
    }
    if (worst_col >= 0) {
      std::fprintf(stderr,
          "[PRESOLVE/POSTSOLVE-DEBUG] initial reduced-mapped violation: "
          "var=%d viol=%.6g x=%.6g [lb=%.6g ub=%.6g]\n",
          worst_col, worst, x[worst_col],
          col_lb_orig_[worst_col], col_ub_orig_[worst_col]);
      reported_initial = true;
    }
  }

  // Apply undo stack in reverse order
  for (int i = static_cast<int>(undo_stack_.size()) - 1; i >= 0; --i) {
    const auto& rec = undo_stack_[i];
    switch (rec.type) {
      case UndoType::FixedCol:
        x[rec.col] = rec.value;
        break;
      case UndoType::SubstitutedCol: {
        // x_col = (rhs - sum(a_k * x_k)) / pivot_coeff
        double sum = 0.0;
        for (const auto& [col, coeff] : rec.row_entries) {
          sum += coeff * x[col];
        }
        x[rec.col] = (rec.rhs - sum) / rec.pivot_coeff;
        break;
      }
      case UndoType::RedundantRow:
        // Nothing to do
        break;
    }
    if (debug_on && rec.col >= 0 && rec.type != UndoType::RedundantRow) {
      double v = check_var_bounds(rec.col);
      if (v > debug_tol) {
        std::fprintf(stderr,
            "[PRESOLVE/POSTSOLVE-DEBUG] record #%d type=%s col=%d "
            "viol=%.6g x=%.6g [lb=%.6g ub=%.6g]",
            i, type_name(rec.type), rec.col, v, x[rec.col],
            col_lb_orig_[rec.col], col_ub_orig_[rec.col]);
        if (rec.type == UndoType::SubstitutedCol) {
          std::fprintf(stderr,
              " pivot_row=%d pivot_coeff=%.6g rhs=%.6g n_entries=%zu",
              rec.pivot_row, rec.pivot_coeff, rec.rhs,
              rec.row_entries.size());
          for (const auto& [cc, coef] : rec.row_entries) {
            std::fprintf(stderr, " | col=%d coef=%.6g x=%.6g [lb=%.6g ub=%.6g]",
                         cc, coef, x[cc],
                         (cc >= 0 && cc < static_cast<int>(col_lb_orig_.size())) ? col_lb_orig_[cc] : 0.0,
                         (cc >= 0 && cc < static_cast<int>(col_ub_orig_.size())) ? col_ub_orig_[cc] : 0.0);
          }
        }
        std::fprintf(stderr, "\n");
      }
    }
  }
  (void)reported_initial;

  return x;
}

Eigen::VectorXd MILPPresolve::forward_map(const Eigen::VectorXd& x_input) const {
  const int n_red = static_cast<int>(reduced_to_orig_col_.size());
  Eigen::VectorXd x_red(n_red);
  for (int j = 0; j < n_red; ++j) {
    int oj = reduced_to_orig_col_[j];
    x_red[j] = (oj >= 0 && oj < x_input.size()) ? x_input[oj] : 0.0;
  }
  return x_red;
}

// ============================================================================
// run: main presolve loop
// ============================================================================

PresolveStats MILPPresolve::run(LPModel& lp,
                                 std::vector<int>& binary_idx,
                                 std::vector<int>& integer_idx) {
  auto t0 = std::chrono::steady_clock::now();
  run_start_ = t0;
  stats_ = PresolveStats{};
  undo_stack_.clear();
  probing_implications_original_.clear();
  probing_implications_reduced_.clear();

  init_from_lp(lp);

  if (deadline_expired()) {
    stats_.timed_out = true;
    stats_.final_rows = stats_.orig_rows;
    stats_.final_cols = stats_.orig_cols;
    stats_.final_nnz = stats_.orig_nnz;
  }

  if (!stats_.timed_out && detect_infeasibility("initialization")) {
    stats_.final_rows = stats_.orig_rows;
    stats_.final_cols = stats_.orig_cols;
    stats_.final_nnz = stats_.orig_nnz;
  }

  for (int round = 0; !stats_.infeasible && round < opts_.max_rounds; ++round) {
    if (deadline_expired()) {
      stats_.timed_out = true;
      break;
    }
    int changes = 0;

    auto run_reduction = [&](auto&& reduction, const char* phase) {
      if (deadline_expired()) {
        stats_.timed_out = true;
        return false;
      }
      changes += reduction();
      if (deadline_expired()) {
        stats_.timed_out = true;
        return false;
      }
      return !stats_.infeasible && !detect_infeasibility(phase);
    };

    if (!run_reduction([&] { return remove_fixed_variables(); },
                       "fixed-variable removal")) break;
    if (!run_reduction([&] { return remove_empty_rows_and_cols(); },
                       "empty-row/column removal")) break;
    if (opts_.do_singleton_rows &&
        !run_reduction([&] { return process_singleton_rows(); },
                       "singleton-row processing")) break;
    if (opts_.do_singleton_columns &&
        !run_reduction([&] { return process_singleton_columns(); },
                       "singleton-column processing")) break;
    if (!run_reduction([&] { return tighten_bounds(); },
                       "bound tightening")) break;
    if (!run_reduction([&] { return remove_fixed_variables(); },
                       "post-tightening fixed-variable removal")) break;
    if (opts_.do_forcing_rows &&
        !run_reduction([&] { return detect_forcing_rows(); },
                       "forcing-row processing")) break;
    if (!run_reduction([&] { return remove_fixed_variables(); },
                       "post-forcing fixed-variable removal")) break;
    if (!run_reduction([&] { return remove_empty_rows_and_cols(); },
                       "post-forcing empty-row/column removal")) break;

    if (opts_.do_doubleton &&
        !run_reduction([&] { return process_doubleton_equations(); },
                       "doubleton processing")) break;

    if (opts_.do_dominated &&
        !run_reduction([&] { return detect_dominated_columns(); },
                       "dominated-column processing")) break;

    if (opts_.do_parallel_rows && round < 3 &&
        !run_reduction([&] { return detect_parallel_rows(); },
                       "parallel-row processing")) break;

    if (opts_.do_probing &&
        !run_reduction([&] { return run_probing(); },
                       "binary probing")) break;

    if (!run_reduction([&] { return remove_fixed_variables(); },
                       "final fixed-variable removal")) break;
    if (!run_reduction([&] { return remove_empty_rows_and_cols(); },
                       "final empty-row/column removal")) break;

    stats_.rounds = round + 1;
    if (changes == 0) break;
  }

  if (!stats_.infeasible && !stats_.timed_out) {
    rebuild_model(lp, binary_idx, integer_idx);
  } else {
    // No reduced model is published on an infeasibility or timeout
    // certificate: the caller must retain the original coordinate system.
    stats_.final_rows = stats_.orig_rows;
    stats_.final_cols = stats_.orig_cols;
    stats_.final_nnz = stats_.orig_nnz;
  }

  auto t1 = std::chrono::steady_clock::now();
  stats_.presolve_time_sec =
      std::chrono::duration<double>(t1 - t0).count();

  if (opts_.verbose) {
    fprintf(stderr,
      "[PRESOLVE] %d rounds in %.1fms: rows %d→%d cols %d→%d nnz %d→%d "
      "(fixed=%d singleton=%d doubleton=%d forcing=%d dominated=%d "
      "parallel=%d probing=%d infeasible=%d)\n",
      stats_.rounds,
      stats_.presolve_time_sec * 1000.0,
      stats_.orig_rows, stats_.final_rows,
      stats_.orig_cols, stats_.final_cols,
      stats_.orig_nnz, stats_.final_nnz,
      stats_.cols_removed,
      stats_.singletons_removed,
      stats_.doubletons_removed,
      stats_.forcing_rows,
      stats_.dominated_cols,
      stats_.parallel_rows,
      stats_.probing_fixings,
      stats_.infeasible ? 1 : 0);
  }

  return stats_;
}

// ============================================================================
// Convenience function
// ============================================================================

PresolveStats presolve_mip(LPModel& lp,
                           std::vector<int>& binary_idx,
                           std::vector<int>& integer_idx,
                           const PresolveOptions& opts) {
  MILPPresolve ps(opts);
  return ps.run(lp, binary_idx, integer_idx);
}

PresolveStats presolve_inplace(LPModel& lp, const PresolveOptions& opts) {
  // Run the full presolve pipeline on the internal representation,
  // then apply results back to the LPModel without removing variables.
  PresolveStats stats;
  auto t0 = std::chrono::steady_clock::now();

  // Create a working copy to run full presolve
  MILPPresolve worker(opts);
  // Build the reduction mapping using the supplied integrality indices.
  std::vector<int> bin_idx, int_idx;
  for (int j = 0; j < static_cast<int>(lp.vars.size()); ++j) {
    if (lp.vars[j].type == VarType::Binary) bin_idx.push_back(j);
    else if (lp.vars[j].type == VarType::Integer) int_idx.push_back(j);
  }

  // Save original model
  LPModel working = lp;
  stats = worker.run(working, bin_idx, int_idx);

  // Now apply the tighter bounds from the reduced model back to original variables.
  // The reduced model has renumbered variables. We need to map back.
  const auto& red_to_orig = worker.reduced_to_orig_col();
  for (int j = 0; j < static_cast<int>(red_to_orig.size()); ++j) {
    int orig = red_to_orig[j];
    lp.vars[orig].lb = working.vars[j].lb;
    lp.vars[orig].ub = working.vars[j].ub;
  }

  // For columns that were removed (fixed), apply their fixed values
  // by zeroing out their coefficient columns and adjusting RHS.
  const int n = static_cast<int>(lp.vars.size());
  const auto& orig_to_red = worker.orig_to_reduced_col();

  for (int j = 0; j < n; ++j) {
    if (orig_to_red[j] >= 0) continue;  // not removed

    // This variable was fixed/removed. Apply to original model.
    double val = lp.vars[j].lb;  // should already be tightened to a point
    // Check if postsolve recorded a fixed value
    // For simplicity, use the tighter bound
    if (std::abs(lp.vars[j].ub - lp.vars[j].lb) > 1e-9) {
      // Wasn't fixed — skip (might have been removed for other reasons)
      continue;
    }

    // Zero out column j in A and adjust b
    for (int r = 0; r < static_cast<int>(lp.A.rows()); ++r) {
      double a = lp.A.coeff(r, j);
      if (std::abs(a) > 1e-15) {
        lp.b[r] -= a * val;
      }
    }
    // Zero out column j in Aeq and adjust beq
    for (int r = 0; r < static_cast<int>(lp.Aeq.rows()); ++r) {
      double a = lp.Aeq.coeff(r, j);
      if (std::abs(a) > 1e-15) {
        lp.beq[r] -= a * val;
      }
    }

    // Zero the column
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      it.valueRef() = 0.0;
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      it.valueRef() = 0.0;
    }
    lp.c[j] = 0.0;
  }

  // Prune compressed storage of zeros
  lp.A.prune(1e-15);
  lp.Aeq.prune(1e-15);

  // ── Remove redundant rows from the original model ──
  // Use the row mapping from the presolve worker to identify which rows were
  // deleted. We rebuild A/b and Aeq/beq retaining only active rows.
  // This reduces LP solve size and benefits the native fallback path (when
  // PaPILO is off) by eliminating provably redundant constraints.
  {
    const int m_ineq_orig = static_cast<int>(lp.A.rows());
    const int m_eq_orig = static_cast<int>(lp.Aeq.rows());
    const auto& r2row = worker.orig_to_reduced_row();
    const int n_total_orig = m_ineq_orig + m_eq_orig;

    if (!r2row.empty() && static_cast<int>(r2row.size()) >= n_total_orig) {
      // Identify surviving ineq rows and eq rows
      std::vector<int> keep_ineq, keep_eq;
      keep_ineq.reserve(static_cast<size_t>(m_ineq_orig));
      keep_eq.reserve(static_cast<size_t>(m_eq_orig));
      for (int r = 0; r < m_ineq_orig; ++r) {
        if (r2row[static_cast<size_t>(r)] >= 0) keep_ineq.push_back(r);
      }
      for (int r = 0; r < m_eq_orig; ++r) {
        if (r2row[static_cast<size_t>(m_ineq_orig + r)] >= 0) keep_eq.push_back(r);
      }

      const int n_keep_ineq = static_cast<int>(keep_ineq.size());
      const int n_keep_eq = static_cast<int>(keep_eq.size());
      const int rows_dropped = (m_ineq_orig - n_keep_ineq) + (m_eq_orig - n_keep_eq);

      if (rows_dropped > 0) {
        // Rebuild A/b with surviving ineq rows
        const int n = static_cast<int>(lp.vars.size());
        Eigen::SparseMatrix<double> A_new(n_keep_ineq, n);
        Eigen::VectorXd b_new(n_keep_ineq);
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(static_cast<size_t>(lp.A.nonZeros()));
        for (int ri = 0; ri < n_keep_ineq; ++ri) {
          const int r = keep_ineq[ri];
          b_new[ri] = lp.b[r];
          for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, r); it; ++it) {
            trips.emplace_back(ri, static_cast<int>(it.row()), it.value());
          }
        }
        // Note: lp.A is ColMajor, inner iterator iterates non-zeros in column j.
        // We need row-indexed access, so use a row-major view.
        Eigen::SparseMatrix<double, Eigen::RowMajor> A_rm = lp.A;
        trips.clear();
        for (int ri = 0; ri < n_keep_ineq; ++ri) {
          const int r = keep_ineq[ri];
          b_new[ri] = lp.b[r];
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_rm, r); it; ++it) {
            trips.emplace_back(ri, static_cast<int>(it.col()), it.value());
          }
        }
        A_new.setFromTriplets(trips.begin(), trips.end());
        lp.A = std::move(A_new);
        lp.b = std::move(b_new);

        // Rebuild Aeq/beq with surviving eq rows
        if (m_eq_orig > 0) {
          Eigen::SparseMatrix<double> Aeq_new(n_keep_eq, n);
          Eigen::VectorXd beq_new(n_keep_eq);
          Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_rm = lp.Aeq;
          trips.clear();
          for (int ri = 0; ri < n_keep_eq; ++ri) {
            const int r = keep_eq[ri];
            beq_new[ri] = lp.beq[r];
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_rm, r); it; ++it) {
              trips.emplace_back(ri, static_cast<int>(it.col()), it.value());
            }
          }
          Aeq_new.setFromTriplets(trips.begin(), trips.end());
          lp.Aeq = std::move(Aeq_new);
          lp.beq = std::move(beq_new);
        }

        stats.rows_removed += rows_dropped;
      }
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  stats.presolve_time_sec = std::chrono::duration<double>(t1 - t0).count();

  if (opts.verbose) {
    fprintf(stderr,
      "[PRESOLVE-INPLACE] %d rounds in %.1fms: rows %d→%d cols %d→%d "
      "(bounds_tightened=%d fixed=%d)\n",
      stats.rounds,
      stats.presolve_time_sec * 1000.0,
      stats.orig_rows, stats.final_rows,
      stats.orig_cols, stats.final_cols,
      stats.bounds_tightened, stats.cols_removed);
  }

  return stats;
}

}  // namespace mipsolvers::engine
