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

#include "hacdcpf/engine/solver/native/milp/bc/milp_presolve.hpp"

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

namespace hacdcpf::engine {

static constexpr double kInf = 1e20;

// ============================================================================
// Construction
// ============================================================================

MILPPresolve::MILPPresolve(PresolveOptions opts) : opts_(std::move(opts)) {}

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

  // Rows: first m_ineq rows are Ax <= b, last m_eq rows are Aeq*x = beq
  rows_.resize(m);
  row_lb_.resize(m);
  row_ub_.resize(m);
  row_deleted_.assign(m, false);
  row_activity_.resize(m);

  // Build row entries from inequality constraints (Ax <= b → -inf <= a^T x <= b)
  {
    Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
    for (int r = 0; r < m_ineq; ++r) {
      row_lb_[r] = -kInf;
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
  auto& ac = row_activity_[row];
  ac.min_act = 0.0;
  ac.max_act = 0.0;
  ac.n_inf_min = 0;
  ac.n_inf_max = 0;
  for (const auto& e : rows_[row]) {
    if (col_deleted_[e.col]) continue;
    if (e.val > 0) {
      if (col_lb_[e.col] <= -kInf + 1) ac.n_inf_min++;
      else ac.min_act += e.val * col_lb_[e.col];
      if (col_ub_[e.col] >= kInf - 1) ac.n_inf_max++;
      else ac.max_act += e.val * col_ub_[e.col];
    } else {
      if (col_ub_[e.col] >= kInf - 1) ac.n_inf_min++;
      else ac.min_act += e.val * col_ub_[e.col];
      if (col_lb_[e.col] <= -kInf + 1) ac.n_inf_max++;
      else ac.max_act += e.val * col_lb_[e.col];
    }
  }
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
}

void MILPPresolve::add_entry(int row, int col, double val) {
  rows_[row].push_back({col, val});
  cols_[col].push_back({row, val});
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
        // Infeasible — we can't do anything, let LP detect it
        continue;
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
          col_lb_[singleton_col] = new_lb;
          changed = true;
        }
      }
      if (row_ub_[r] < kInf - 1) {
        double new_ub = row_ub_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_ub = std::floor(new_ub + opts_.bound_tol);
        if (new_ub < col_ub_[singleton_col] - opts_.bound_tol) {
          col_ub_[singleton_col] = new_ub;
          changed = true;
        }
      }
    } else if (singleton_val < -opts_.zero_tol) {
      // Division by negative flips the inequality
      if (row_ub_[r] < kInf - 1) {
        double new_lb = row_ub_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
        if (new_lb > col_lb_[singleton_col] + opts_.bound_tol) {
          col_lb_[singleton_col] = new_lb;
          changed = true;
        }
      }
      if (row_lb_[r] > -kInf + 1) {
        double new_ub = row_lb_[r] / singleton_val;
        if (is_integer_var(singleton_col)) new_ub = std::floor(new_ub + opts_.bound_tol);
        if (new_ub < col_ub_[singleton_col] - opts_.bound_tol) {
          col_ub_[singleton_col] = new_ub;
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
      // This preserves integrality only if keep_coeff / elim_coeff is integer
      double ratio = keep_coeff / elim_coeff;
      if (std::abs(ratio - std::round(ratio)) > opts_.bound_tol) continue;
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
        col_lb_[keep_col] = implied_lb;
      if (implied_ub < col_ub_[keep_col] - opts_.bound_tol)
        col_ub_[keep_col] = implied_ub;
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
  compute_all_activities();

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    const auto& ac = row_activity_[r];

    for (const auto& e : rows_[r]) {
      if (col_deleted_[e.col]) continue;

      // Upper bound tightening from row_ub: a^T x <= row_ub
      if (row_ub_[r] < kInf - 1 && ac.n_inf_min == 0) {
        // min_activity_excl = min_act - contribution of e.col
        double contrib = (e.val > 0) ? e.val * col_lb_[e.col] : e.val * col_ub_[e.col];
        double min_excl = ac.min_act - contrib;
        double residual = row_ub_[r] - min_excl;

        if (e.val > opts_.zero_tol) {
          double new_ub = residual / e.val;
          if (is_integer_var(e.col)) new_ub = std::floor(new_ub + opts_.bound_tol);
          if (new_ub < col_ub_[e.col] - opts_.bound_tol) {
            col_ub_[e.col] = new_ub;
            ++count;
          }
        } else if (e.val < -opts_.zero_tol) {
          double new_lb = residual / e.val;
          if (is_integer_var(e.col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
          if (new_lb > col_lb_[e.col] + opts_.bound_tol) {
            col_lb_[e.col] = new_lb;
            ++count;
          }
        }
      }

      // Lower bound tightening from row_lb: row_lb <= a^T x
      if (row_lb_[r] > -kInf + 1 && ac.n_inf_max == 0) {
        double contrib = (e.val > 0) ? e.val * col_ub_[e.col] : e.val * col_lb_[e.col];
        double max_excl = ac.max_act - contrib;
        double residual = row_lb_[r] - max_excl;

        if (e.val > opts_.zero_tol) {
          double new_lb = residual / e.val;
          if (is_integer_var(e.col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
          if (new_lb > col_lb_[e.col] + opts_.bound_tol) {
            col_lb_[e.col] = new_lb;
            ++count;
          }
        } else if (e.val < -opts_.zero_tol) {
          double new_ub = residual / e.val;
          if (is_integer_var(e.col)) new_ub = std::floor(new_ub + opts_.bound_tol);
          if (new_ub < col_ub_[e.col] - opts_.bound_tol) {
            col_ub_[e.col] = new_ub;
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
                col_ub_[e.col] = new_ub;
                ++count;
              }
            } else if (e.val < -opts_.zero_tol) {
              double new_lb = residual / e.val;
              if (is_integer_var(e.col)) new_lb = std::ceil(new_lb - opts_.bound_tol);
              if (new_lb > col_lb_[e.col] + opts_.bound_tol || col_lb_[e.col] <= -kInf + 1) {
                col_lb_[e.col] = new_lb;
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
  compute_all_activities();

  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    const auto& ac = row_activity_[r];

    // Redundant row: max_activity <= row_ub AND min_activity >= row_lb
    bool max_below_ub = (ac.n_inf_max == 0 && ac.max_act <= row_ub_[r] + opts_.bound_tol);
    bool min_above_lb = (ac.n_inf_min == 0 && ac.min_act >= row_lb_[r] - opts_.bound_tol);

    if (max_below_ub && (row_lb_[r] <= -kInf + 1 || min_above_lb)) {
      delete_row(r);
      ++count;
      continue;
    }

    // Forcing row at min: min_activity == row_ub → all vars at their contributing bound
    if (ac.n_inf_min == 0 && ac.min_act >= row_ub_[r] - opts_.bound_tol) {
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
        ac.max_act <= row_lb_[r] + opts_.bound_tol) {
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

        // Merge row2 into row1: keep the tighter constraint
        // Scale row2's bounds by 1/ratio to match row1's scale.
        // When ratio < 0, dividing flips the inequality direction, so
        // what was the lower bound becomes the upper and vice versa.
        double lb2_scaled, ub2_scaled;
        if (ratio > 0) {
          lb2_scaled = (row_lb_[r2] > -kInf + 1) ? row_lb_[r2] / ratio : -kInf;
          ub2_scaled = (row_ub_[r2] < kInf - 1)  ? row_ub_[r2] / ratio : kInf;
        } else {
          // ratio < 0: division flips direction
          lb2_scaled = (row_ub_[r2] < kInf - 1)  ? row_ub_[r2] / ratio : -kInf;
          ub2_scaled = (row_lb_[r2] > -kInf + 1) ? row_lb_[r2] / ratio : kInf;
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
// 10. Coefficient strengthening for MIP
// ============================================================================

int MILPPresolve::strengthen_coefficients() {
  if (!opts_.do_coefficient_strengthen) return 0;
  int count = 0;
  compute_all_activities();

  // For each inequality row a^T x <= b with integer variable x_j:
  // If x_j is binary with coefficient a_j > 0:
  //   New coefficient: a_j' = min(a_j, b - min_activity_excl_j)
  //   New RHS: b' = b - (a_j - a_j')
  for (int r = 0; r < m_orig_; ++r) {
    if (row_deleted_[r]) continue;
    if (row_ub_[r] >= kInf - 1) continue;  // No upper bound
    const auto& ac = row_activity_[r];
    if (ac.n_inf_min != 0) continue;  // Need finite min activity

    for (auto& e : rows_[r]) {
      if (col_deleted_[e.col]) continue;
      if (col_type_[e.col] != VarType::Binary) continue;
      if (e.val <= opts_.zero_tol) continue;  // Only positive coefficients

      double contrib = e.val * col_lb_[e.col];  // = 0 for binary
      double min_excl = ac.min_act - contrib;
      double max_coeff = row_ub_[r] - min_excl;

      if (max_coeff < e.val - opts_.bound_tol && max_coeff > opts_.zero_tol) {
        double reduction = e.val - max_coeff;
        e.val = max_coeff;
        // Update column entry too
        for (auto& ce : cols_[e.col]) {
          if (ce.row == r) { ce.val = max_coeff; break; }
        }
        // Adjust RHS: b' = b - reduction * ub (ub=1 for binary)
        row_ub_[r] -= reduction * col_ub_[e.col];
        stats_.coefficients_strengthened++;
        ++count;
      }
    }
  }
  return count;
}

// ============================================================================
// 11. Binary probing
// ============================================================================

int MILPPresolve::run_probing() {
  if (!opts_.do_probing) return 0;
  int count = 0;

  // Collect unfixed binary variables
  std::vector<int> candidates;
  for (int j = 0; j < n_orig_; ++j) {
    if (col_deleted_[j]) continue;
    if (col_type_[j] != VarType::Binary) continue;
    if (is_fixed(j)) continue;
    candidates.push_back(j);
  }

  // Scale probing effort with problem size to avoid O(n^2 * m) blowup
  int max_probes = std::min(opts_.max_probing_candidates,
                             static_cast<int>(candidates.size()));
  // For large problems, limit probing to avoid excessive time
  if (m_orig_ > 2000) max_probes = std::min(max_probes, 100);
  if (m_orig_ > 5000) max_probes = std::min(max_probes, 50);

  const auto t_probe_start = std::chrono::steady_clock::now();
  constexpr double kProbingTimeLimitSec = 1.0;  // Max 1 second for probing

  for (int pi = 0; pi < max_probes; ++pi) {
    // Time check every 10 probes
    if (pi > 0 && (pi % 10) == 0) {
      auto now = std::chrono::steady_clock::now();
      double elapsed = std::chrono::duration<double>(now - t_probe_start).count();
      if (elapsed > kProbingTimeLimitSec) break;
    }
    const int j = candidates[pi];
    if (col_deleted_[j] || is_fixed(j)) continue;

    // Save current bounds
    std::vector<double> save_lb = col_lb_;
    std::vector<double> save_ub = col_ub_;

    bool infeas_at_0 = false, infeas_at_1 = false;
    std::vector<double> lb_at_0, ub_at_0, lb_at_1, ub_at_1;

    // Collect rows that contain variable j (only these need recomputation)
    std::vector<int> affected_rows;
    for (const auto& ce : cols_[j]) {
      if (!row_deleted_[ce.row]) affected_rows.push_back(ce.row);
    }

    // Helper lambda: probe x_j = val, propagate bounds, return infeasible flag
    auto probe_val = [&](double val, bool& infeas, std::vector<double>& out_lb,
                         std::vector<double>& out_ub) {
      col_lb_ = save_lb; col_ub_ = save_ub;
      col_lb_[j] = val; col_ub_[j] = val;

      for (int d = 0; d < opts_.probing_depth; ++d) {
        // Only recompute activities for affected rows
        for (int r : affected_rows) update_activity(r);
        int changes = 0;

        for (int r : affected_rows) {
          const auto& ac = row_activity_[r];
          for (const auto& e : rows_[r]) {
            if (col_deleted_[e.col] || e.col == j) continue;

            if (row_ub_[r] < kInf - 1 && ac.n_inf_min == 0) {
              double contrib = (e.val > 0) ? e.val * col_lb_[e.col] : e.val * col_ub_[e.col];
              double residual = row_ub_[r] - (ac.min_act - contrib);
              if (e.val > opts_.zero_tol) {
                double nb = residual / e.val;
                if (is_integer_var(e.col)) nb = std::floor(nb + opts_.bound_tol);
                if (nb < col_ub_[e.col] - opts_.bound_tol) { col_ub_[e.col] = nb; ++changes; }
              } else if (e.val < -opts_.zero_tol) {
                double nb = residual / e.val;
                if (is_integer_var(e.col)) nb = std::ceil(nb - opts_.bound_tol);
                if (nb > col_lb_[e.col] + opts_.bound_tol) { col_lb_[e.col] = nb; ++changes; }
              }
            }
            if (row_lb_[r] > -kInf + 1 && ac.n_inf_max == 0) {
              double contrib = (e.val > 0) ? e.val * col_ub_[e.col] : e.val * col_lb_[e.col];
              double residual = row_lb_[r] - (ac.max_act - contrib);
              if (e.val > opts_.zero_tol) {
                double nb = residual / e.val;
                if (is_integer_var(e.col)) nb = std::ceil(nb - opts_.bound_tol);
                if (nb > col_lb_[e.col] + opts_.bound_tol) { col_lb_[e.col] = nb; ++changes; }
              } else if (e.val < -opts_.zero_tol) {
                double nb = residual / e.val;
                if (is_integer_var(e.col)) nb = std::floor(nb + opts_.bound_tol);
                if (nb < col_ub_[e.col] - opts_.bound_tol) { col_ub_[e.col] = nb; ++changes; }
              }
            }
          }
        }
        // Check feasibility on affected variables
        for (int r : affected_rows) {
          for (const auto& e : rows_[r]) {
            if (!col_deleted_[e.col] && col_lb_[e.col] > col_ub_[e.col] + opts_.bound_tol) {
              infeas = true; break;
            }
          }
          if (infeas) break;
        }
        if (infeas || changes == 0) break;
      }
      out_lb = col_lb_;
      out_ub = col_ub_;
    };

    probe_val(0.0, infeas_at_0, lb_at_0, ub_at_0);
    probe_val(1.0, infeas_at_1, lb_at_1, ub_at_1);

    // Restore bounds
    col_lb_ = save_lb;
    col_ub_ = save_ub;

    // Apply results
    if (infeas_at_0 && infeas_at_1) {
      // Both infeasible — problem is infeasible, abort
      return count;
    }
    if (infeas_at_0) {
      col_lb_[j] = 1.0; col_ub_[j] = 1.0;
      stats_.probing_fixings++;
      ++count;
    } else if (infeas_at_1) {
      col_lb_[j] = 0.0; col_ub_[j] = 0.0;
      stats_.probing_fixings++;
      ++count;
    } else {
      // Apply intersection of bounds: valid bounds = union of both probe worlds
      // lb = min(lb_at_0, lb_at_1), ub = max(ub_at_0, ub_at_1)
      for (int k = 0; k < n_orig_; ++k) {
        if (col_deleted_[k]) continue;
        double best_lb = std::min(lb_at_0[k], lb_at_1[k]);
        double best_ub = std::max(ub_at_0[k], ub_at_1[k]);
        if (best_lb > col_lb_[k] + opts_.bound_tol) {
          col_lb_[k] = best_lb;
          stats_.bounds_tightened++;
          ++count;
        }
        if (best_ub < col_ub_[k] - opts_.bound_tol) {
          col_ub_[k] = best_ub;
          stats_.bounds_tightened++;
          ++count;
        }
      }
    }
  }
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

  // Stats
  stats_.final_cols = n_reduced;
  stats_.final_rows = m_ineq_reduced + m_eq_reduced;
  int final_nnz = 0;
  for (int j = 0; j < n_reduced; ++j) {
    final_nnz += static_cast<int>(lp.A.outerIndexPtr()[j+1] - lp.A.outerIndexPtr()[j]);
    final_nnz += static_cast<int>(lp.Aeq.outerIndexPtr()[j+1] - lp.Aeq.outerIndexPtr()[j]);
  }
  stats_.final_nnz = final_nnz;
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
  const char* debug_env = std::getenv("HACDCPF_PRESOLVE_POSTSOLVE_DEBUG");
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
  stats_ = PresolveStats{};
  undo_stack_.clear();

  init_from_lp(lp);

  for (int round = 0; round < opts_.max_rounds; ++round) {
    int changes = 0;

    changes += remove_fixed_variables();
    changes += remove_empty_rows_and_cols();
    changes += process_singleton_rows();
    changes += process_singleton_columns();
    changes += tighten_bounds();
    changes += remove_fixed_variables();  // new fixings from tightening
    changes += detect_forcing_rows();
    changes += remove_fixed_variables();
    changes += remove_empty_rows_and_cols();

    if (opts_.do_doubleton)
      changes += process_doubleton_equations();

    if (opts_.do_dominated)
      changes += detect_dominated_columns();

    if (opts_.do_parallel_rows && round < 3)  // only first few rounds
      changes += detect_parallel_rows();

    if (opts_.do_coefficient_strengthen)
      changes += strengthen_coefficients();

    // Probing is expensive — only in later rounds after other reductions stabilize
    if (opts_.do_probing && round >= 1)
      changes += run_probing();

    changes += remove_fixed_variables();
    changes += remove_empty_rows_and_cols();

    stats_.rounds = round + 1;
    if (changes == 0) break;
  }

  rebuild_model(lp, binary_idx, integer_idx);

  auto t1 = std::chrono::steady_clock::now();
  stats_.presolve_time_sec =
      std::chrono::duration<double>(t1 - t0).count();

  if (opts_.verbose) {
    fprintf(stderr,
      "[PRESOLVE] %d rounds in %.1fms: rows %d→%d cols %d→%d nnz %d→%d "
      "(fixed=%d singleton=%d doubleton=%d forcing=%d dominated=%d "
      "parallel=%d strengthen=%d probing=%d)\n",
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
      stats_.coefficients_strengthened,
      stats_.probing_fixings);
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
  // Use a dummy run with binary/integer idx
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

}  // namespace hacdcpf::engine
