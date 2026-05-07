/// @file bc_implied_bounds.cpp
/// @brief Implied-integrality and variable-bound source helpers for legacy B&C.

#include "mipsolvers/engine/detail/bc_implied_bounds.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>
#include <unordered_set>
#include <utility>

#include <Eigen/Sparse>
#include <fmt/format.h>

#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"

namespace mipsolvers::engine::detail {

bool is_near_integer_scalar(double v, double tol = 1e-9) {
  return std::isfinite(v) && std::abs(v - std::round(v)) <= tol;
}

std::vector<char> detect_implied_integer_columns(const LPModel& lp) {
  const int n = static_cast<int>(lp.vars.size());
  std::vector<char> implied(static_cast<std::size_t>(n), 0);
  if (n == 0) {
    return implied;
  }

  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;
  const int m_ineq = static_cast<int>(A_row.rows());
  const int m_eq = static_cast<int>(Aeq_row.rows());

  auto row_scaled_integral = [&](const auto& row_matrix, int row, double scale,
                                 double rhs, bool has_rhs,
                                 double lhs = -kInf, bool has_lhs = false) {
    if (has_rhs && !is_near_integer_scalar(rhs * scale)) return false;
    if (has_lhs && !is_near_integer_scalar(lhs * scale)) return false;
    for (typename std::decay_t<decltype(row_matrix)>::InnerIterator it(row_matrix, row); it; ++it) {
      if (std::abs(it.value()) > 1e-12 &&
          !is_near_integer_scalar(it.value() * scale)) {
        return false;
      }
    }
    return true;
  };

  bool changed = true;
  int passes = 0;
  while (changed && passes++ < 8) {
    changed = false;

    std::vector<int> ineq_size(static_cast<std::size_t>(m_ineq), 0);
    std::vector<int> ineq_integral(static_cast<std::size_t>(m_ineq), 0);
    for (int r = 0; r < m_ineq; ++r) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        if (std::abs(it.value()) <= 1e-12) continue;
        ++ineq_size[static_cast<std::size_t>(r)];
        if (is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
            implied[static_cast<std::size_t>(j)]) {
          ++ineq_integral[static_cast<std::size_t>(r)];
        }
      }
    }
    std::vector<int> eq_size(static_cast<std::size_t>(m_eq), 0);
    std::vector<int> eq_integral(static_cast<std::size_t>(m_eq), 0);
    for (int r = 0; r < m_eq; ++r) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        if (std::abs(it.value()) <= 1e-12) continue;
        ++eq_size[static_cast<std::size_t>(r)];
        if (is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
            implied[static_cast<std::size_t>(j)]) {
          ++eq_integral[static_cast<std::size_t>(r)];
        }
      }
    }

    for (int col = 0; col < n; ++col) {
      const auto& var = lp.vars[static_cast<std::size_t>(col)];
      if (var.type != VarType::Continuous || implied[static_cast<std::size_t>(col)])
        continue;

      bool equation_witness = false;
      bool dual_detection_possible = true;
      bool touched = false;

      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, col); it; ++it) {
        const int r = static_cast<int>(it.row());
        const double a = it.value();
        if (std::abs(a) <= 1e-12) continue;
        touched = true;
        if (eq_size[static_cast<std::size_t>(r)] < 2 ||
            eq_integral[static_cast<std::size_t>(r)] <
                eq_size[static_cast<std::size_t>(r)] - 1) {
          dual_detection_possible = false;
          continue;
        }
        const double scale = 1.0 / a;
        if (row_scaled_integral(Aeq_row, r, scale, lp.beq[r], true)) {
          equation_witness = true;
          break;
        }
      }
      if (equation_witness) {
        implied[static_cast<std::size_t>(col)] = 1;
        changed = true;
        continue;
      }

      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
        const int r = static_cast<int>(it.row());
        if (std::abs(it.value()) <= 1e-12) continue;
        touched = true;
        if (ineq_size[static_cast<std::size_t>(r)] < 2 ||
            ineq_integral[static_cast<std::size_t>(r)] <
                ineq_size[static_cast<std::size_t>(r)] - 1) {
          dual_detection_possible = false;
          break;
        }
      }
      if (!touched || !dual_detection_possible) continue;
      if ((std::isfinite(var.lb) && !is_near_integer_scalar(var.lb)) ||
          (std::isfinite(var.ub) && !is_near_integer_scalar(var.ub))) {
        continue;
      }

      bool all_rows_integral = true;
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, col); it; ++it) {
        const double a = it.value();
        if (std::abs(a) <= 1e-12) continue;
        if (!row_scaled_integral(Aeq_row, static_cast<int>(it.row()),
                                 1.0 / a,
                                 lp.beq[static_cast<int>(it.row())], true)) {
          all_rows_integral = false;
          break;
        }
      }
      if (!all_rows_integral) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
        const double a = it.value();
        if (std::abs(a) <= 1e-12) continue;
        const int row = static_cast<int>(it.row());
        const double lhs = lp_row_lhs_or_neg_inf(lp, row);
        if (!row_scaled_integral(A_row, row, 1.0 / a, lp.b[row], true,
                                 lhs, std::isfinite(lhs))) {
          all_rows_integral = false;
          break;
        }
      }
      if (all_rows_integral) {
        implied[static_cast<std::size_t>(col)] = 1;
        changed = true;
      }
    }
  }
  return implied;
}

IntegralRowTighteningStats tighten_integral_row_sides(
    LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    double feastol) {
  IntegralRowTighteningStats stats;
  const int n = static_cast<int>(lp.vars.size());
  const int m = static_cast<int>(lp.A.rows());
  if (n <= 0 || m <= 0) return stats;

  auto is_integral_or_implied = [&](int j) {
    return j >= 0 && j < n &&
           (is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
            (j < static_cast<int>(implied_integer_cols.size()) &&
             implied_integer_cols[static_cast<std::size_t>(j)] != 0));
  };

  Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
  const bool has_lhs = lp_has_row_lhs(lp);
  const double tol = std::max(1e-9, feastol);

  for (int r = 0; r < m; ++r) {
    bool integral_row = true;
    bool has_nz = false;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, r);
         it; ++it) {
      const double a = it.value();
      if (std::abs(a) <= 1e-12) continue;
      has_nz = true;
      const int j = static_cast<int>(it.col());
      if (!is_integral_or_implied(j) || !is_near_integer_scalar(a, tol)) {
        integral_row = false;
        break;
      }
    }
    if (!has_nz || !integral_row) continue;
    ++stats.integral_rows;

    if (std::isfinite(lp.b[r])) {
      const double old_ub = lp.b[r];
      const double new_ub = std::floor(old_ub + tol);
      if (new_ub < old_ub - 1e-10) {
        lp.b[r] = new_ub;
        ++stats.upper_tightened;
        stats.max_upper_delta =
            std::max(stats.max_upper_delta, old_ub - new_ub);
      }
    }

    if (has_lhs) {
      const double old_lb = lp.row_lhs[r];
      if (std::isfinite(old_lb)) {
        const double new_lb = std::ceil(old_lb - tol);
        if (new_lb > old_lb + 1e-10) {
          lp.row_lhs[r] = new_lb;
          ++stats.lower_tightened;
          stats.max_lower_delta =
              std::max(stats.max_lower_delta, new_lb - old_lb);
        }
      }
    }
  }

  return stats;
}

void accumulate_variable_bound_source_stats(
    NativeVariableBoundSourceStats& dst,
    const NativeVariableBoundSourceStats& src) {
  dst.row_sides_scanned += src.row_sides_scanned;
  dst.single_binary_row_sides += src.single_binary_row_sides;
  dst.skipped_multi_binary += src.skipped_multi_binary;
  dst.candidates += src.candidates;
  dst.exported_implications += src.exported_implications;
  dst.cut_rows_scanned += src.cut_rows_scanned;
  dst.cut_mixed_rows += src.cut_mixed_rows;
  dst.cut_vub_candidates += src.cut_vub_candidates;
  dst.cut_vlb_candidates += src.cut_vlb_candidates;
  dst.cut_domain_tightenings += src.cut_domain_tightenings;
  dst.cut_exported_implications += src.cut_exported_implications;
  dst.presolve_vub_candidates += src.presolve_vub_candidates;
  dst.presolve_vlb_candidates += src.presolve_vlb_candidates;
  dst.presolve_varbounds_replayed += src.presolve_varbounds_replayed;
  dst.presolve_exported_implications += src.presolve_exported_implications;
  dst.implied_bound_passes += src.implied_bound_passes;
  dst.implied_bound_lower += src.implied_bound_lower;
  dst.implied_bound_upper += src.implied_bound_upper;
  dst.probe_vub_candidates += src.probe_vub_candidates;
  dst.probe_vlb_candidates += src.probe_vlb_candidates;
  dst.probe_varbounds_replayed += src.probe_varbounds_replayed;
  dst.probe_global_tightenings += src.probe_global_tightenings;
  dst.probe_exported_implications += src.probe_exported_implications;
}

std::vector<NativeVariableBoundSourceEntry> collect_variable_bound_sources(
    const VariableBoundTable& table) {
  std::vector<NativeVariableBoundSourceEntry> out;
  for (int col = 0; col < table.num_vars(); ++col) {
    for (const auto& entry : table.vubs(col)) {
      out.push_back(NativeVariableBoundSourceEntry{
          true, col, entry.trigger_col, entry.bound.coef,
          entry.bound.constant});
    }
    for (const auto& entry : table.vlbs(col)) {
      out.push_back(NativeVariableBoundSourceEntry{
          false, col, entry.trigger_col, entry.bound.coef,
          entry.bound.constant});
    }
  }
  return out;
}

std::string native_varbound_key(bool is_vub,
                                int col,
                                int trigger_col,
                                double coef,
                                double constant) {
  return fmt::format("{}:{}:{}:{:.17g}:{:.17g}", is_vub ? 1 : 0, col,
                     trigger_col, coef, constant);
}

void trace_highs_native_varbound_diff(
    const char* phase,
    const HiGHSPresolvedModelStats& highs_state,
    const LPModel* lp,
    const VariableBoundTable& table,
    int max_samples) {
  if (!bc_frontier_conformance_enabled() &&
      std::getenv("HACDCPF_HIGHS_PRESOLVE_STATS") == nullptr) {
    return;
  }
  if (!highs_state.side_state_available || highs_state.var_bounds.empty()) return;
  std::unordered_set<std::string> native_keys;
  native_keys.reserve(table.size() * 2 + 1);
  std::uint64_t native_vub = 0;
  std::uint64_t native_vlb = 0;
  for (int col = 0; col < table.num_vars(); ++col) {
    for (const auto& entry : table.vubs(col)) {
      native_keys.insert(native_varbound_key(true, col, entry.trigger_col,
                                             entry.bound.coef,
                                             entry.bound.constant));
      ++native_vub;
    }
    for (const auto& entry : table.vlbs(col)) {
      native_keys.insert(native_varbound_key(false, col, entry.trigger_col,
                                             entry.bound.coef,
                                             entry.bound.constant));
      ++native_vlb;
    }
  }

  std::uint64_t highs_vub_records = 0;
  std::uint64_t highs_vlb_records = 0;
  std::uint64_t missing = 0;
  std::uint64_t missing_vub = 0;
  std::uint64_t missing_vlb = 0;
  std::uint64_t missing_same_row = 0;
  std::uint64_t missing_no_same_row = 0;
  std::uint64_t missing_bad_cols = 0;
  fmt::memory_buffer sample;
  int emitted = 0;
  auto has_same_row = [&](int target_col, int trigger_col) {
    if (lp == nullptr || target_col < 0 || trigger_col < 0 ||
        target_col >= static_cast<int>(lp->vars.size()) ||
        trigger_col >= static_cast<int>(lp->vars.size())) {
      ++missing_bad_cols;
      return false;
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> arow = lp->A;
    arow.makeCompressed();
    for (int r = 0; r < arow.rows(); ++r) {
      bool has_target = false;
      bool has_trigger = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(arow, r);
           it; ++it) {
        const int col = static_cast<int>(it.col());
        if (col == target_col) has_target = true;
        if (col == trigger_col) has_trigger = true;
      }
      if (has_target && has_trigger) return true;
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> aeqrow = lp->Aeq;
    aeqrow.makeCompressed();
    for (int r = 0; r < aeqrow.rows(); ++r) {
      bool has_target = false;
      bool has_trigger = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(aeqrow, r);
           it; ++it) {
        const int col = static_cast<int>(it.col());
        if (col == target_col) has_target = true;
        if (col == trigger_col) has_trigger = true;
      }
      if (has_target && has_trigger) return true;
    }
    return false;
  };
  for (const auto& rec : highs_state.var_bounds) {
    if (rec.upper)
      ++highs_vub_records;
    else
      ++highs_vlb_records;
    const std::string key = native_varbound_key(
        rec.upper, rec.target_col, rec.trigger_col, rec.coef, rec.constant);
    if (native_keys.find(key) != native_keys.end()) continue;
    ++missing;
    if (rec.upper)
      ++missing_vub;
    else
      ++missing_vlb;
    if (has_same_row(rec.target_col, rec.trigger_col))
      ++missing_same_row;
    else
      ++missing_no_same_row;
    if (emitted < max_samples) {
      if (emitted++ > 0) fmt::format_to(std::back_inserter(sample), ";");
      fmt::format_to(std::back_inserter(sample), "{}{}{}={:.12g}*{}+{:.12g}",
                     rec.target_col, rec.upper ? "<=" : ">=", "hs",
                     rec.coef, rec.trigger_col, rec.constant);
    }
  }
  fmt::print(stderr,
             "[B&C-HIGHS-VB-DIFF] phase={} hs_vub={} hs_vlb={} "
             "native_vub={} native_vlb={} missing={} missing_vub={} "
             "missing_vlb={} same_row={} no_same_row={} bad_cols={} "
             "sample=[{}]\n",
             phase != nullptr ? phase : "unknown", highs_vub_records,
             highs_vlb_records, native_vub, native_vlb, missing,
             missing_vub, missing_vlb, missing_same_row, missing_no_same_row,
             missing_bad_cols, fmt::to_string(sample));
}

struct NativeImpliedColumnBounds {
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<int> lower_source;
  std::vector<int> upper_source;
  std::uint64_t passes{0};
  std::uint64_t lower_tightened{0};
  std::uint64_t upper_tightened{0};
};

NativeImpliedColumnBounds compute_implied_column_bounds_from_rows(
    const LPModel& lp,
    int max_passes = 16,
    int max_row_nnz = 512,
    double tol = 1e-9) {
  NativeImpliedColumnBounds out;
  const int n = static_cast<int>(lp.vars.size());
  out.lower.assign(static_cast<std::size_t>(n),
                   -std::numeric_limits<double>::infinity());
  out.upper.assign(static_cast<std::size_t>(n),
                   std::numeric_limits<double>::infinity());
  out.lower_source.assign(static_cast<std::size_t>(n), -1);
  out.upper_source.assign(static_cast<std::size_t>(n), -1);
  if (n <= 0) return out;

  struct Entry {
    int col{-1};
    double coef{0.0};
  };

  auto effective_lower = [&](int row, int col) {
    const double raw = lp.vars[static_cast<std::size_t>(col)].lb;
    if (out.lower_source[static_cast<std::size_t>(col)] == row) return raw;
    const double impl = out.lower[static_cast<std::size_t>(col)];
    return std::isfinite(impl) ? std::max(raw, impl) : raw;
  };
  auto effective_upper = [&](int row, int col) {
    const double raw = lp.vars[static_cast<std::size_t>(col)].ub;
    if (out.upper_source[static_cast<std::size_t>(col)] == row) return raw;
    const double impl = out.upper[static_cast<std::size_t>(col)];
    return std::isfinite(impl) ? std::min(raw, impl) : raw;
  };
  auto lower_activity_bound = [&](int row, int col, double coef) {
    return coef > 0.0 ? effective_lower(row, col)
                      : effective_upper(row, col);
  };
  auto upper_activity_bound = [&](int row, int col, double coef) {
    return coef < 0.0 ? effective_lower(row, col)
                      : effective_upper(row, col);
  };
  auto add_activity = [](double& activity, int& num_inf, double coef,
                         double bound) {
    if (std::isfinite(bound)) {
      activity += coef * bound;
    } else {
      ++num_inf;
    }
  };
  auto remove_activity = [](double& activity, int& num_inf, double coef,
                            double bound) {
    if (std::isfinite(bound)) {
      activity -= coef * bound;
    } else {
      --num_inf;
    }
  };
  auto try_tighten_lower = [&](int row, int col, double value) {
    if (!std::isfinite(value)) return false;
    const double raw_lb = lp.vars[static_cast<std::size_t>(col)].lb;
    const double raw_ub = lp.vars[static_cast<std::size_t>(col)].ub;
    if (value <= raw_lb + tol || value > raw_ub + tol) return false;
    const double cur = out.lower[static_cast<std::size_t>(col)];
    if (std::isfinite(cur) && value <= cur + tol) return false;
    out.lower[static_cast<std::size_t>(col)] = value;
    out.lower_source[static_cast<std::size_t>(col)] = row;
    ++out.lower_tightened;
    return true;
  };
  auto try_tighten_upper = [&](int row, int col, double value) {
    if (!std::isfinite(value)) return false;
    const double raw_lb = lp.vars[static_cast<std::size_t>(col)].lb;
    const double raw_ub = lp.vars[static_cast<std::size_t>(col)].ub;
    if (value >= raw_ub - tol || value < raw_lb - tol) return false;
    const double cur = out.upper[static_cast<std::size_t>(col)];
    if (std::isfinite(cur) && value >= cur - tol) return false;
    out.upper[static_cast<std::size_t>(col)] = value;
    out.upper_source[static_cast<std::size_t>(col)] = row;
    ++out.upper_tightened;
    return true;
  };

  auto process_row =
      [&](const Eigen::SparseMatrix<double, Eigen::RowMajor>& rows,
          int matrix_row, int source_row, double lhs, double rhs) {
        std::vector<Entry> entries;
        entries.reserve(32);
        double sum_lower = 0.0;
        double sum_upper = 0.0;
        int num_inf_lower = 0;
        int num_inf_upper = 0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(rows, matrix_row);
             it; ++it) {
          const int col = static_cast<int>(it.col());
          const double a = it.value();
          if (col < 0 || col >= n || std::abs(a) <= 1e-12) continue;
          if (static_cast<int>(entries.size()) >= max_row_nnz) return false;
          const auto& var = lp.vars[static_cast<std::size_t>(col)];
          if (std::abs(var.ub - var.lb) <= 1e-12) continue;
          entries.push_back(Entry{col, a});
          add_activity(sum_lower, num_inf_lower, a,
                       lower_activity_bound(source_row, col, a));
          add_activity(sum_upper, num_inf_upper, a,
                       upper_activity_bound(source_row, col, a));
        }
        if (entries.size() <= 1) return false;

        bool changed = false;
        for (const Entry& entry : entries) {
          if (std::abs(entry.coef) <= 1e-12) continue;
          if (std::isfinite(rhs)) {
            double residual = sum_lower;
            int residual_inf = num_inf_lower;
            remove_activity(residual, residual_inf, entry.coef,
                            lower_activity_bound(source_row, entry.col, entry.coef));
            if (residual_inf == 0 && std::isfinite(residual)) {
              const double bound = (rhs - residual) / entry.coef;
              if (entry.coef > 0.0) {
                changed =
                    try_tighten_upper(source_row, entry.col, bound) || changed;
              } else {
                changed =
                    try_tighten_lower(source_row, entry.col, bound) || changed;
              }
            }
          }
          if (std::isfinite(lhs)) {
            double residual = sum_upper;
            int residual_inf = num_inf_upper;
            remove_activity(residual, residual_inf, entry.coef,
                            upper_activity_bound(source_row, entry.col, entry.coef));
            if (residual_inf == 0 && std::isfinite(residual)) {
              const double bound = (lhs - residual) / entry.coef;
              if (entry.coef > 0.0) {
                changed =
                    try_tighten_lower(source_row, entry.col, bound) || changed;
              } else {
                changed =
                    try_tighten_upper(source_row, entry.col, bound) || changed;
              }
            }
          }
        }
        return changed;
      };

  Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeqrow = lp.Aeq;
  for (int pass = 0; pass < max_passes; ++pass) {
    bool changed = false;
    for (int r = 0; r < Arow.rows(); ++r) {
      changed =
          process_row(Arow, r, r, lp_row_lhs_or_neg_inf(lp, r), lp.b[r]) ||
          changed;
    }
    const int eq_offset = static_cast<int>(Arow.rows());
    for (int r = 0; r < Aeqrow.rows(); ++r) {
      changed = process_row(Aeqrow, r, eq_offset + r, lp.beq[r], lp.beq[r]) ||
                changed;
    }
    ++out.passes;
    if (!changed) break;
  }

  return out;
}

NativeVariableBoundSourceStats build_variable_bound_table_from_rows(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    VariableBoundTable& table,
    BinaryImplicationGraph* implication_graph,
    int max_row_nnz) {
  NativeVariableBoundSourceStats stats;
  const int n = static_cast<int>(lp.vars.size());
  table.reset(n);
  if (n <= 0) return stats;

  NativeImpliedColumnBounds implied_col_bounds =
      compute_implied_column_bounds_from_rows(lp, 16, max_row_nnz, 1e-9);
  stats.implied_bound_passes = implied_col_bounds.passes;
  stats.implied_bound_lower = implied_col_bounds.lower_tightened;
  stats.implied_bound_upper = implied_col_bounds.upper_tightened;

  auto is_implied_integer = [&](int j) {
    return j >= 0 && j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0;
  };
  auto is_integral_or_implied = [&](int j) {
    return j >= 0 && j < n &&
           (is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
            is_implied_integer(j));
  };
  auto binary_trigger_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > 1e-9 || var.ub < 1.0 - 1e-9) return false;
    return var.type == VarType::Binary ||
           (var.type == VarType::Integer && std::abs(var.lb) <= 1e-9 &&
            std::abs(var.ub - 1.0) <= 1e-9);
  };
  struct Entry {
    int col{-1};
    double coef{0.0};
  };

  auto effective_lower = [&](int row, int col) {
    const auto& var = lp.vars[static_cast<std::size_t>(col)];
    if (implied_col_bounds.lower_source[static_cast<std::size_t>(col)] == row)
      return var.lb;
    const double impl = implied_col_bounds.lower[static_cast<std::size_t>(col)];
    return std::isfinite(impl) ? std::max(var.lb, impl) : var.lb;
  };
  auto effective_upper = [&](int row, int col) {
    const auto& var = lp.vars[static_cast<std::size_t>(col)];
    if (implied_col_bounds.upper_source[static_cast<std::size_t>(col)] == row)
      return var.ub;
    const double impl = implied_col_bounds.upper[static_cast<std::size_t>(col)];
    return std::isfinite(impl) ? std::min(var.ub, impl) : var.ub;
  };
  auto bound_for_lower_activity = [&](int row, int col, double coef) {
    return coef > 0.0 ? effective_lower(row, col) : effective_upper(row, col);
  };
  auto bound_for_upper_activity = [&](int row, int col, double coef) {
    return coef < 0.0 ? effective_lower(row, col) : effective_upper(row, col);
  };
  auto add_activity = [](double& activity, int& num_inf, double coef,
                         double bound) {
    if (std::isfinite(bound)) {
      activity += coef * bound;
    } else {
      ++num_inf;
    }
  };
  auto remove_activity = [](double& activity, int& num_inf, double coef,
                            double bound) {
    if (std::isfinite(bound)) {
      activity -= coef * bound;
    } else {
      --num_inf;
    }
  };
  auto replace_activity = [&](double& activity, int& num_inf, double coef,
                              double old_bound, double new_bound) {
    remove_activity(activity, num_inf, coef, old_bound);
    add_activity(activity, num_inf, coef, new_bound);
  };

  auto process_ranged_row =
      [&](const Eigen::SparseMatrix<double, Eigen::RowMajor>& rows,
          int matrix_row, int source_row, double lhs, double rhs) {
    std::vector<Entry> entries;
    entries.reserve(32);
    int binary_col = -1;
    double binary_coef = 0.0;
    double sum_lower = 0.0;
    double sum_upper = 0.0;
    int num_inf_lower = 0;
    int num_inf_upper = 0;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(rows, matrix_row);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      const double a = it.value();
      if (col < 0 || col >= n || std::abs(a) <= 1e-12) continue;
      if (static_cast<int>(entries.size()) >= max_row_nnz) return;
      const auto& var = lp.vars[static_cast<std::size_t>(col)];
      if (std::abs(var.ub - var.lb) <= 1e-12) continue;
      entries.push_back(Entry{col, a});
      add_activity(sum_lower, num_inf_lower, a,
                   bound_for_lower_activity(source_row, col, a));
      add_activity(sum_upper, num_inf_upper, a,
                   bound_for_upper_activity(source_row, col, a));
      if (binary_trigger_like(col)) {
        if (binary_col != -1) {
          ++stats.skipped_multi_binary;
          return;
        }
        binary_col = col;
        binary_coef = a;
      }
    }
    if (entries.size() <= 1) return;
    if (binary_col < 0) return;
    const bool use_lhs = std::isfinite(lhs) && num_inf_upper <= 1;
    const bool use_rhs = std::isfinite(rhs) && num_inf_lower <= 1;
    if (!use_lhs && !use_rhs) return;
    stats.row_sides_scanned += (use_lhs ? 1 : 0) + (use_rhs ? 1 : 0);
    stats.single_binary_row_sides += (use_lhs ? 1 : 0) + (use_rhs ? 1 : 0);

    for (const Entry& target : entries) {
      if (target.col == binary_col || std::abs(target.coef) <= 1e-12) {
        continue;
      }
      double vlb_constant = -std::numeric_limits<double>::infinity();
      if (use_lhs) {
        double residual = sum_upper;
        int residual_inf = num_inf_upper;
        remove_activity(residual, residual_inf, target.coef,
                        bound_for_upper_activity(source_row, target.col,
                                                 target.coef));
        replace_activity(
            residual, residual_inf, binary_coef,
            bound_for_upper_activity(source_row, binary_col, binary_coef),
            0.0);
        if (residual_inf == 0 && std::isfinite(residual)) {
          vlb_constant = (lhs - residual) / std::abs(target.coef);
        }
      }
      double vub_constant = std::numeric_limits<double>::infinity();
      if (use_rhs) {
        double residual = sum_lower;
        int residual_inf = num_inf_lower;
        remove_activity(residual, residual_inf, target.coef,
                        bound_for_lower_activity(source_row, target.col,
                                                 target.coef));
        replace_activity(
            residual, residual_inf, binary_coef,
            bound_for_lower_activity(source_row, binary_col, binary_coef),
            0.0);
        if (residual_inf == 0 && std::isfinite(residual)) {
          vub_constant = (rhs - residual) / std::abs(target.coef);
        }
      }

      if (target.coef < 0.0) {
        vlb_constant *= -1.0;
        vub_constant *= -1.0;
        std::swap(vlb_constant, vub_constant);
      }
      const double vb_coef = -binary_coef / target.coef;
      if (!std::isfinite(vb_coef)) continue;
      const bool target_integral = is_integral_or_implied(target.col);
      if (std::isfinite(vlb_constant)) {
        ++stats.candidates;
        table.add_vlb(target.col, binary_col, vb_coef, vlb_constant,
                      lp.vars[static_cast<std::size_t>(target.col)].lb,
                      target_integral);
      }
      if (std::isfinite(vub_constant)) {
        ++stats.candidates;
        table.add_vub(target.col, binary_col, vb_coef, vub_constant,
                      lp.vars[static_cast<std::size_t>(target.col)].ub,
                      target_integral);
      }
    }
  };

  Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
  for (int r = 0; r < Arow.rows(); ++r) {
    process_ranged_row(Arow, r, r, lp_row_lhs_or_neg_inf(lp, r), lp.b[r]);
  }
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeqrow = lp.Aeq;
  const int eq_offset = static_cast<int>(Arow.rows());
  for (int r = 0; r < Aeqrow.rows(); ++r) {
    process_ranged_row(Aeqrow, r, eq_offset + r, lp.beq[r], lp.beq[r]);
  }
  if (implication_graph != nullptr) {
    stats.exported_implications =
        table.export_to_implication_graph(*implication_graph);
  }
  return stats;
}

NativeVariableBoundSourceStats augment_variable_bound_table_from_cut_rows(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    int first_row,
    int last_row,
    VariableBoundTable& table,
    BinaryImplicationGraph* implication_graph,
    int max_cut_nnz,
    double feastol) {
  NativeVariableBoundSourceStats stats;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0 || lp.A.rows() <= 0) return stats;
  first_row = std::max(0, first_row);
  last_row = std::min<int>(last_row, static_cast<int>(lp.A.rows()));
  if (first_row >= last_row) return stats;

  auto is_implied_integer = [&](int j) {
    return j >= 0 && j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0;
  };
  auto is_integral_or_implied = [&](int j) {
    return j >= 0 && j < n &&
           (is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
            is_implied_integer(j));
  };
  auto is_binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > feastol || var.ub < 1.0 - feastol) return false;
    if (var.type == VarType::Binary ||
        (var.type == VarType::Integer && std::abs(var.lb) <= feastol &&
         std::abs(var.ub - 1.0) <= feastol)) {
      return true;
    }
    return is_implied_integer(j) && std::abs(var.lb) <= feastol &&
           std::abs(var.ub - 1.0) <= feastol;
  };

  struct RowEntry {
    int col{-1};
    double val{0.0};
  };

  Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
  const std::uint64_t exported_before =
      implication_graph != nullptr
          ? static_cast<std::uint64_t>(implication_graph->size())
          : 0;

  for (int r = first_row; r < last_row; ++r) {
    if (!std::isfinite(lp.b[r])) continue;
    std::vector<RowEntry> entries;
    entries.reserve(32);
    int nbin = 0;
    double min_activity = 0.0;
    bool finite_activity = true;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, r);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      const double a = it.value();
      if (col < 0 || col >= n || std::abs(a) <= 1e-12) continue;
      if (static_cast<int>(entries.size()) >= max_cut_nnz) {
        finite_activity = false;
        break;
      }
      const auto& var = lp.vars[static_cast<std::size_t>(col)];
      if (std::abs(var.ub - var.lb) <= 1e-12) continue;
      const double bound = a > 0.0 ? var.lb : var.ub;
      if (!std::isfinite(bound)) {
        finite_activity = false;
        break;
      }
      min_activity += a * bound;
      entries.push_back(RowEntry{col, a});
      if (is_binary_like(col)) ++nbin;
    }
    if (!finite_activity || entries.size() <= 1) continue;
    ++stats.cut_rows_scanned;
    if (lp.b[r] - min_activity < 0.0) min_activity = lp.b[r];

    for (const RowEntry& entry : entries) {
      if (!is_integral_or_implied(entry.col)) continue;
      const auto& var = lp.vars[static_cast<std::size_t>(entry.col)];
      if (entry.val > 0.0) {
        const double bound_val =
            std::floor((lp.b[r] - min_activity) / entry.val + var.lb + feastol);
        if (std::isfinite(bound_val) && bound_val < var.ub - feastol) {
          ++stats.cut_domain_tightenings;
        }
      } else if (entry.val < 0.0) {
        const double bound_val =
            std::ceil((lp.b[r] - min_activity) / entry.val + var.ub - feastol);
        if (std::isfinite(bound_val) && bound_val > var.lb + feastol) {
          ++stats.cut_domain_tightenings;
        }
      }
    }

    if (nbin <= 1 || nbin >= static_cast<int>(entries.size())) continue;
    ++stats.cut_mixed_rows;

    for (const RowEntry& bin_entry : entries) {
      if (!is_binary_like(bin_entry.col)) continue;
      const int bin_col = bin_entry.col;
      const double implied_activity =
          lp.b[r] - min_activity - std::abs(bin_entry.val);
      if (!std::isfinite(implied_activity)) continue;

      for (const RowEntry& target : entries) {
        if (target.col == bin_col || is_binary_like(target.col)) continue;
        const auto& var = lp.vars[static_cast<std::size_t>(target.col)];
        const bool target_integral = is_integral_or_implied(target.col);
        if (target.val > 0.0) {
          double impl_ub =
              (implied_activity + target.val * var.lb) / target.val;
          if (target_integral) impl_ub = std::floor(impl_ub + feastol);
          if (!(std::isfinite(impl_ub) && impl_ub < var.ub - feastol)) continue;

          double coef = 0.0;
          double constant = 0.0;
          if (bin_entry.val < 0.0) {
            coef = var.ub - impl_ub;
            constant = impl_ub;
          } else {
            if (!std::isfinite(var.ub)) continue;
            coef = impl_ub - var.ub;
            constant = var.ub;
          }
          ++stats.cut_vub_candidates;
          table.add_vub(target.col, bin_col, coef, constant, var.ub,
                        target_integral);
        } else if (target.val < 0.0) {
          double impl_lb =
              (implied_activity + target.val * var.ub) / target.val;
          if (target_integral) impl_lb = std::ceil(impl_lb - feastol);
          if (!(std::isfinite(impl_lb) && impl_lb > var.lb + feastol)) continue;

          double coef = 0.0;
          double constant = 0.0;
          if (bin_entry.val < 0.0) {
            coef = var.lb - impl_lb;
            constant = impl_lb;
          } else {
            if (!std::isfinite(var.lb)) continue;
            coef = impl_lb - var.lb;
            constant = var.lb;
          }
          ++stats.cut_vlb_candidates;
          table.add_vlb(target.col, bin_col, coef, constant, var.lb,
                        target_integral);
        }
      }
    }
  }

  if (implication_graph != nullptr) {
    table.export_to_implication_graph(*implication_graph);
    const std::uint64_t exported_after =
        static_cast<std::uint64_t>(implication_graph->size());
    stats.cut_exported_implications =
        exported_after > exported_before ? exported_after - exported_before : 0;
  }
  return stats;
}

NativeVariableBoundSourceStats augment_variable_bound_table_from_sparse_cut_rows(
    const LPModel& domain_lp,
    const std::vector<Eigen::SparseVector<double>>& cut_rows,
    const std::vector<double>& cut_rhs,
    const std::vector<char>& implied_integer_cols,
    VariableBoundTable& table,
    BinaryImplicationGraph* implication_graph,
    int max_cut_nnz,
    double feastol) {
  NativeVariableBoundSourceStats stats;
  const int n = static_cast<int>(domain_lp.vars.size());
  const std::size_t nrows = std::min(cut_rows.size(), cut_rhs.size());
  if (n <= 0 || nrows == 0) return stats;

  auto is_implied_integer = [&](int j) {
    return j >= 0 && j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0;
  };
  auto is_integral_or_implied = [&](int j) {
    return j >= 0 && j < n &&
           (is_integer_type(domain_lp.vars[static_cast<std::size_t>(j)]) ||
            is_implied_integer(j));
  };
  auto is_binary_like = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = domain_lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > feastol || var.ub < 1.0 - feastol) return false;
    if (var.type == VarType::Binary ||
        (var.type == VarType::Integer && std::abs(var.lb) <= feastol &&
         std::abs(var.ub - 1.0) <= feastol)) {
      return true;
    }
    return is_implied_integer(j) && std::abs(var.lb) <= feastol &&
           std::abs(var.ub - 1.0) <= feastol;
  };

  struct RowEntry {
    int col{-1};
    double val{0.0};
  };

  const std::uint64_t exported_before =
      implication_graph != nullptr
          ? static_cast<std::uint64_t>(implication_graph->size())
          : 0;

  for (std::size_t row_id = 0; row_id < nrows; ++row_id) {
    const double rhs = cut_rhs[row_id];
    if (!std::isfinite(rhs)) continue;
    const auto& sparse_row = cut_rows[row_id];
    std::vector<RowEntry> entries;
    entries.reserve(32);
    int nbin = 0;
    double min_activity = 0.0;
    bool finite_activity = true;
    for (Eigen::SparseVector<double>::InnerIterator it(sparse_row); it; ++it) {
      const int col = static_cast<int>(it.index());
      const double a = it.value();
      if (col < 0 || col >= n || std::abs(a) <= 1e-12) continue;
      if (static_cast<int>(entries.size()) >= max_cut_nnz) {
        finite_activity = false;
        break;
      }
      const auto& var = domain_lp.vars[static_cast<std::size_t>(col)];
      if (std::abs(var.ub - var.lb) <= 1e-12) continue;
      const double bound = a > 0.0 ? var.lb : var.ub;
      if (!std::isfinite(bound)) {
        finite_activity = false;
        break;
      }
      min_activity += a * bound;
      entries.push_back(RowEntry{col, a});
      if (is_binary_like(col)) ++nbin;
    }
    if (!finite_activity || entries.size() <= 1) continue;
    ++stats.cut_rows_scanned;
    if (rhs - min_activity < 0.0) min_activity = rhs;

    for (const RowEntry& entry : entries) {
      if (!is_integral_or_implied(entry.col)) continue;
      const auto& var = domain_lp.vars[static_cast<std::size_t>(entry.col)];
      if (entry.val > 0.0) {
        const double bound_val =
            std::floor((rhs - min_activity) / entry.val + var.lb + feastol);
        if (std::isfinite(bound_val) && bound_val < var.ub - feastol) {
          ++stats.cut_domain_tightenings;
        }
      } else if (entry.val < 0.0) {
        const double bound_val =
            std::ceil((rhs - min_activity) / entry.val + var.ub - feastol);
        if (std::isfinite(bound_val) && bound_val > var.lb + feastol) {
          ++stats.cut_domain_tightenings;
        }
      }
    }

    if (nbin <= 1 || nbin >= static_cast<int>(entries.size())) continue;
    ++stats.cut_mixed_rows;

    for (const RowEntry& bin_entry : entries) {
      if (!is_binary_like(bin_entry.col)) continue;
      const int bin_col = bin_entry.col;
      const double implied_activity =
          rhs - min_activity - std::abs(bin_entry.val);
      if (!std::isfinite(implied_activity)) continue;
      for (const RowEntry& target : entries) {
        if (target.col == bin_col || is_binary_like(target.col)) continue;
        const auto& var = domain_lp.vars[static_cast<std::size_t>(target.col)];
        const bool target_integral = is_integral_or_implied(target.col);
        if (target.val > 0.0) {
          double impl_ub =
              (implied_activity + target.val * var.lb) / target.val;
          if (target_integral) impl_ub = std::floor(impl_ub + feastol);
          if (!(std::isfinite(impl_ub) && impl_ub < var.ub - feastol)) continue;

          double coef = 0.0;
          double constant = 0.0;
          if (bin_entry.val < 0.0) {
            coef = var.ub - impl_ub;
            constant = impl_ub;
          } else {
            if (!std::isfinite(var.ub)) continue;
            coef = impl_ub - var.ub;
            constant = var.ub;
          }
          ++stats.cut_vub_candidates;
          table.add_vub(target.col, bin_col, coef, constant, var.ub,
                        target_integral);
        } else if (target.val < 0.0) {
          double impl_lb =
              (implied_activity + target.val * var.ub) / target.val;
          if (target_integral) impl_lb = std::ceil(impl_lb - feastol);
          if (!(std::isfinite(impl_lb) && impl_lb > var.lb + feastol)) continue;

          double coef = 0.0;
          double constant = 0.0;
          if (bin_entry.val < 0.0) {
            coef = var.lb - impl_lb;
            constant = impl_lb;
          } else {
            if (!std::isfinite(var.lb)) continue;
            coef = impl_lb - var.lb;
            constant = var.lb;
          }
          ++stats.cut_vlb_candidates;
          table.add_vlb(target.col, bin_col, coef, constant, var.lb,
                        target_integral);
        }
      }
    }
  }

  if (implication_graph != nullptr) {
    table.export_to_implication_graph(*implication_graph);
    const std::uint64_t exported_after =
        static_cast<std::uint64_t>(implication_graph->size());
    stats.cut_exported_implications =
        exported_after > exported_before ? exported_after - exported_before : 0;
  }
  return stats;
}

RootImpliedIntegerArtifactStats build_implied_integer_row_artifacts(
    const LPModel& lp,
    const std::vector<char>& implied_integer_cols,
    CliqueTable& clique_table,
    BinaryImplicationGraph& implication_graph,
    int max_row_nnz,
    int max_binary_literals_per_row,
    std::uint64_t max_implications,
    const Eigen::VectorXd* lp_solution) {
  (void)lp_solution;
  RootImpliedIntegerArtifactStats stats;
  const int n = static_cast<int>(lp.vars.size());
  if (n <= 0 || lp.A.rows() == 0) return stats;

  auto is_implied_integer = [&](int j) {
    return j >= 0 && j < static_cast<int>(implied_integer_cols.size()) &&
           implied_integer_cols[static_cast<std::size_t>(j)] != 0;
  };
  auto is_integral_or_implied = [&](int j) {
    return j >= 0 && j < n &&
           (is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
            is_implied_integer(j));
  };
  auto is_binary_col = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.lb > 1e-9 || var.ub < 1.0 - 1e-9) return false;
    if (var.type == VarType::Binary) return true;
    return is_implied_integer(j) && std::abs(var.lb) <= 1e-9 &&
           std::abs(var.ub - 1.0) <= 1e-9;
  };
  auto rounded_bound = [&](int j, bool is_lb, double value) {
    if (!is_integral_or_implied(j)) return value;
    return is_lb ? std::ceil(value - 1e-9) : std::floor(value + 1e-9);
  };
  auto min_contribution = [&](double a, int j, double& contrib) {
    if (a > 0.0) {
      if (!std::isfinite(lp.vars[static_cast<std::size_t>(j)].lb)) return false;
      contrib = a * lp.vars[static_cast<std::size_t>(j)].lb;
    } else {
      if (!std::isfinite(lp.vars[static_cast<std::size_t>(j)].ub)) return false;
      contrib = a * lp.vars[static_cast<std::size_t>(j)].ub;
    }
    return true;
  };
  auto objective_literal = [&](int col, bool one) {
    return col >= 0 && col < n && col < lp.c.size() &&
           std::isfinite(lp.c[col]) && std::abs(lp.c[col]) > 1e-12 &&
           ((lp.c[col] < 0.0) == one);
  };

  struct Entry {
    int col{-1};
    double val{0.0};
    double min_contrib{0.0};
  };
  struct Literal {
    int col{-1};
    bool one{false};
    double delta{0.0};
  };

  std::vector<std::pair<CliqueTable::Literal, CliqueTable::Literal>>
      literal_clique_edges;
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;

  auto add_forbid_literal = [&](int trigger, bool trigger_one,
                                int lit_col, bool lit_one) {
    bool added = false;
    if (lit_one) {
      added = implication_graph.add_implication(
          trigger, trigger_one, lit_col, /*implied_is_lb=*/false, 0.0);
    } else {
      added = implication_graph.add_implication(
          trigger, trigger_one, lit_col, /*implied_is_lb=*/true, 1.0);
    }
    if (added) ++stats.implications_added;
    return added;
  };

  auto process_side = [&](const Eigen::SparseMatrix<double, Eigen::RowMajor>& row_matrix,
                          int row, double side_sign, double rhs) {
    if (!std::isfinite(rhs) ||
        stats.implications_added >= max_implications) {
      return;
    }
    std::vector<Entry> entries;
    entries.reserve(32);
    double min_activity = 0.0;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(row_matrix, row);
         it; ++it) {
      const double a = side_sign * it.value();
      if (std::abs(a) <= 1e-12) continue;
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= n) return;
      if (static_cast<int>(entries.size()) >= max_row_nnz) return;
      double c = 0.0;
      if (!min_contribution(a, j, c)) return;
      min_activity += c;
      entries.push_back(Entry{j, a, c});
    }
    if (entries.size() < 2) return;
    ++stats.row_sides_scanned;

    std::vector<Literal> literals;
    literals.reserve(std::min<int>(2 * entries.size(),
                                   max_binary_literals_per_row));
    for (const Entry& e : entries) {
      if (!is_binary_col(e.col)) continue;
      if (static_cast<int>(literals.size()) + 2 >
          max_binary_literals_per_row) {
        break;
      }
      literals.push_back(Literal{e.col, false, -e.min_contrib});
      literals.push_back(Literal{e.col, true, e.val - e.min_contrib});
    }
    if (!literals.empty()) ++stats.row_sides_with_binary_literals;

    const double scale = std::max(1.0, std::abs(rhs));
    const double conflict_tol = 1e-9 * scale + 1e-9;
    for (std::size_t a = 0; a < literals.size(); ++a) {
      for (std::size_t b = a + 1; b < literals.size(); ++b) {
        const Literal& la = literals[a];
        const Literal& lb = literals[b];
        if (la.col == lb.col) continue;
        ++stats.literal_pair_tests;
        const bool objective_pair =
            objective_literal(la.col, la.one) &&
            objective_literal(lb.col, lb.one);
        if (objective_pair) ++stats.objective_literal_pair_tests;
        if (min_activity + la.delta + lb.delta <= rhs + conflict_tol) {
          continue;
        }
        ++stats.literal_pair_conflicts;
        if (objective_pair) ++stats.objective_literal_pair_conflicts;
        bool any = false;
        any = add_forbid_literal(la.col, la.one, lb.col, lb.one) || any;
        any = add_forbid_literal(lb.col, lb.one, la.col, la.one) || any;
        literal_clique_edges.emplace_back(
            CliqueTable::Literal{la.col, la.one},
            CliqueTable::Literal{lb.col, lb.one});
        if (any) ++stats.conflicts_added;
        if (stats.implications_added >= max_implications) return;
      }
    }

    for (const Literal& lit : literals) {
      if (stats.implications_added >= max_implications) return;
      const double conditioned_min = min_activity + lit.delta;
      for (const Entry& target : entries) {
        if (target.col == lit.col) {
          continue;
        }
        ++stats.bound_candidates;
        const double residual =
            rhs - (conditioned_min - target.min_contrib);
        if (std::abs(target.val) <= 1e-12) continue;
        if (target.val > 0.0) {
          double ub = rounded_bound(target.col, /*is_lb=*/false,
                                    residual / target.val);
          if (ub < lp.vars[static_cast<std::size_t>(target.col)].ub - 1e-9) {
            ++stats.bound_improvements;
            if (implication_graph.add_implication(
                    lit.col, lit.one, target.col, /*implied_is_lb=*/false,
                    ub)) {
              ++stats.implications_added;
            }
            if (is_binary_col(target.col) && ub <= 1e-9) {
              literal_clique_edges.emplace_back(
                  CliqueTable::Literal{lit.col, lit.one},
                  CliqueTable::Literal{target.col, true});
            }
          }
        } else {
          double lb = rounded_bound(target.col, /*is_lb=*/true,
                                    residual / target.val);
          if (lb > lp.vars[static_cast<std::size_t>(target.col)].lb + 1e-9) {
            ++stats.bound_improvements;
            if (implication_graph.add_implication(
                    lit.col, lit.one, target.col, /*implied_is_lb=*/true,
                    lb)) {
              ++stats.implications_added;
            }
            if (is_binary_col(target.col) && lb >= 1.0 - 1e-9) {
              literal_clique_edges.emplace_back(
                  CliqueTable::Literal{lit.col, lit.one},
                  CliqueTable::Literal{target.col, false});
            }
          }
        }
        if (stats.implications_added >= max_implications) {
          return;
        }
      }
    }
  };

  for (int r = 0; r < A_row.rows(); ++r) {
    process_side(A_row, r, 1.0, lp.b[r]);
    const double lhs = lp_row_lhs_or_neg_inf(lp, r);
    if (std::isfinite(lhs)) process_side(A_row, r, -1.0, -lhs);
    if (stats.implications_added >= max_implications) break;
  }
  for (int r = 0; r < Aeq_row.rows() &&
                  stats.implications_added < max_implications; ++r) {
    process_side(Aeq_row, r, 1.0, lp.beq[r]);
    process_side(Aeq_row, r, -1.0, -lp.beq[r]);
  }

  stats.clique_edges_added =
      clique_table.add_literal_edges(lp, literal_clique_edges);
  return stats;
}

}  // namespace mipsolvers::engine::detail
