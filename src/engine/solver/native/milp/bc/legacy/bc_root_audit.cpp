/// @file bc_root_audit.cpp
/// @brief Root-cut and root-reduced-cost audit helpers for B&C diagnostics.

#include "mipsolvers/engine/detail/bc_root_audit.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_types.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

namespace mipsolvers::engine::detail {

void truncate_ineq_rows(LPModel& lp, int keep_rows) {
  const int current_rows = static_cast<int>(lp.A.rows());
  if (keep_rows < 0) {
    return;
  }
  keep_rows = std::min(keep_rows, current_rows);
  if (keep_rows < current_rows) {
    Eigen::SparseMatrix<double> kept = lp.A.topRows(keep_rows);
    lp.A = std::move(kept);
    lp.b.conservativeResize(keep_rows);
  }
  if (lp.row_lhs.size() > 0 && lp.row_lhs.size() != keep_rows) {
    if (lp.row_lhs.size() > keep_rows) {
      lp.row_lhs.conservativeResize(keep_rows);
    } else {
      Eigen::VectorXd lhs_new = Eigen::VectorXd::Constant(
          keep_rows, -std::numeric_limits<double>::infinity());
      lhs_new.head(lp.row_lhs.size()) = lp.row_lhs;
      lp.row_lhs = std::move(lhs_new);
    }
  }
}

RootCutRowAudit audit_root_cut_rows(const LPModel& lp,
                                    int first_new_row,
                                    const Eigen::VectorXd& lp_point,
                                    const Eigen::VectorXd* incumbent_point,
                                    double tol) {
  RootCutRowAudit out;
  const int m = static_cast<int>(lp.A.rows());
  const int n = static_cast<int>(lp.A.cols());
  if (first_new_row < 0 || first_new_row >= m || lp_point.size() != n) {
    return out;
  }

  out.rows = m - first_new_row;
  std::vector<double> lhs_lp(static_cast<std::size_t>(out.rows), 0.0);
  std::vector<double> lhs_inc(static_cast<std::size_t>(out.rows), 0.0);
  std::vector<double> norm_sq(static_cast<std::size_t>(out.rows), 0.0);
  std::vector<char> finite(static_cast<std::size_t>(out.rows), 1);
  const bool has_incumbent =
      incumbent_point != nullptr && incumbent_point->size() == n;

  for (int col = 0; col < lp.A.outerSize(); ++col) {
    const double xj = lp_point[col];
    const double zj = has_incumbent ? (*incumbent_point)[col] : 0.0;
    if (!std::isfinite(xj) || (has_incumbent && !std::isfinite(zj))) {
      continue;
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      const int row = static_cast<int>(it.row());
      if (row < first_new_row) continue;
      const int k = row - first_new_row;
      const double a = it.value();
      if (!std::isfinite(a)) {
        finite[static_cast<std::size_t>(k)] = 0;
        continue;
      }
      lhs_lp[static_cast<std::size_t>(k)] += a * xj;
      if (has_incumbent) {
        lhs_inc[static_cast<std::size_t>(k)] += a * zj;
      }
      norm_sq[static_cast<std::size_t>(k)] += a * a;
    }
  }

  for (int k = 0; k < out.rows; ++k) {
    const int row = first_new_row + k;
    const double rhs = lp.b[row];
    const double norm = std::sqrt(norm_sq[static_cast<std::size_t>(k)]);
    const double scale = std::max({1.0, norm, std::abs(rhs)});
    if (!finite[static_cast<std::size_t>(k)] || !std::isfinite(rhs) ||
        norm <= 1e-15) {
      ++out.invalid_rows;
      continue;
    }

    const double lp_viol = lhs_lp[static_cast<std::size_t>(k)] - rhs;
    out.max_lp_violation = std::max(out.max_lp_violation, lp_viol);
    if (lp_viol <= tol * scale) {
      ++out.nonviolated_at_lp;
    }

    if (has_incumbent) {
      const double inc_viol = lhs_inc[static_cast<std::size_t>(k)] - rhs;
      out.max_incumbent_violation =
          std::max(out.max_incumbent_violation, inc_viol);
      if (inc_viol > 100.0 * tol * scale) {
        ++out.incumbent_violations;
      }
    }
  }

  return out;
}

RootCutStandardFormAudit audit_root_cut_standard_form_rows(const LPModel& lp,
                                                           int first_new_row,
                                                           double tol) {
  RootCutStandardFormAudit out;
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int n = static_cast<int>(lp.A.cols());
  if (first_new_row < 0 || first_new_row >= m_ineq) return out;

  const StandardFormLP sf = build_standard_form_lp(lp);
  if (sf.n_original != n || static_cast<int>(sf.A.rows()) < m_ineq ||
      static_cast<int>(sf.row_sign.size()) < m_ineq ||
      sf.lb_shift.size() < n) {
    out.rows = m_ineq - first_new_row;
    out.mismatches = out.rows;
    return out;
  }

  const Eigen::SparseMatrix<double, Eigen::RowMajor> lp_a_row = lp.A;
  const Eigen::SparseMatrix<double, Eigen::RowMajor> sf_a_row = sf.A;
  for (int r = first_new_row; r < m_ineq; ++r) {
    ++out.rows;
    const int sign = sf.row_sign[static_cast<std::size_t>(r)];
    if (sign < 0) ++out.flipped_rows;

    double shifted_rhs = lp.b[r];
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(lp_a_row, r);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      if (col >= 0 && col < n) shifted_rhs -= it.value() * sf.lb_shift[col];
    }
    const double expected_rhs = static_cast<double>(sign) * shifted_rhs;
    const double rhs_err = std::abs(sf.b[r] - expected_rhs);
    out.max_rhs_error = std::max(out.max_rhs_error, rhs_err);
    bool mismatch = rhs_err > tol * std::max({1.0, std::abs(sf.b[r]),
                                              std::abs(expected_rhs)});

    std::unordered_map<int, double> expected;
    expected.reserve(static_cast<std::size_t>(lp_a_row.innerVector(r).nonZeros()));
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(lp_a_row, r);
         it; ++it) {
      expected[static_cast<int>(it.col())] = static_cast<double>(sign) * it.value();
    }
    std::unordered_set<int> seen;
    seen.reserve(expected.size());
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf_a_row, r);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      if (col >= n) continue;
      seen.insert(col);
      const auto pos = expected.find(col);
      const double ref = (pos == expected.end()) ? 0.0 : pos->second;
      const double err = std::abs(it.value() - ref);
      out.max_coeff_error = std::max(out.max_coeff_error, err);
      if (err > tol * std::max({1.0, std::abs(it.value()), std::abs(ref)})) {
        mismatch = true;
      }
    }
    for (const auto& [col, ref] : expected) {
      if (seen.find(col) != seen.end()) continue;
      const double err = std::abs(ref);
      out.max_coeff_error = std::max(out.max_coeff_error, err);
      if (err > tol * std::max(1.0, std::abs(ref))) mismatch = true;
    }
    if (mismatch) ++out.mismatches;
  }
  return out;
}

RootReducedCostAudit audit_root_reduced_costs(
    const std::vector<VariableMeta>& vars,
    const std::vector<char>* implied_integer_cols,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& reduced_costs,
    const std::vector<int>& basis_indices,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub,
    double node_bound,
    double incumbent_obj,
    double int_tol) {
  RootReducedCostAudit out;
  const int n = static_cast<int>(vars.size());
  if (x_relax.size() < n || reduced_costs.size() < n || lb.size() < n ||
      ub.size() < n) {
    return out;
  }
  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int idx : basis_indices) {
    if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
  }
  out.gap = std::isfinite(incumbent_obj) && std::isfinite(node_bound)
      ? incumbent_obj - node_bound
      : kInf;
  const double tol = std::max(1e-12, int_tol);
  for (int j = 0; j < n; ++j) {
    const double rc_abs = std::abs(reduced_costs[j]);
    if (!std::isfinite(rc_abs) || rc_abs <= tol) continue;
    ++out.rc_nonzero;
    out.rc_abs_sum += rc_abs;
    out.rc_abs_max = std::max(out.rc_abs_max, rc_abs);
    const bool declared_integer =
        is_integer_type(vars[static_cast<std::size_t>(j)]);
    const bool implied_integer =
        implied_integer_cols != nullptr &&
        j < static_cast<int>(implied_integer_cols->size()) &&
        (*implied_integer_cols)[static_cast<std::size_t>(j)];
    if (!declared_integer && !implied_integer) continue;
    ++out.rc_integer_nonzero;
    if (implied_integer && !declared_integer) ++out.rc_implied_integer_nonzero;
    const bool at_lower = std::isfinite(lb[j]) && std::abs(x_relax[j] - lb[j]) <= tol;
    const bool at_upper = std::isfinite(ub[j]) && std::abs(x_relax[j] - ub[j]) <= tol;
    if (at_lower) ++out.rc_at_lower;
    if (at_upper) ++out.rc_at_upper;
    if (is_basic[static_cast<std::size_t>(j)] || !std::isfinite(out.gap) ||
        out.gap <= 1e-12 || !std::isfinite(lb[j]) || !std::isfinite(ub[j]) ||
        ub[j] <= lb[j] + 1e-9) {
      continue;
    }
    const double max_increase = rc_abs * (ub[j] - lb[j]);
    if (max_increase <= out.gap + 1e-9) continue;
    if (at_lower) {
      const double new_ub = std::floor(out.gap / rc_abs + lb[j] + int_tol);
      if (new_ub < ub[j] - 1e-9) ++out.cutoff_fix_candidates;
    } else if (at_upper) {
      const double new_lb = std::ceil(ub[j] - out.gap / rc_abs - int_tol);
      if (new_lb > lb[j] + 1e-9) ++out.cutoff_fix_candidates;
    }
  }
  return out;
}

void note_root_subsolve_trace(std::vector<RootSubsolveTrace>& traces,
                              bool enabled,
                              const char* name,
                              double ms,
                              bool success,
                              bool incumbent_hit,
                              int lp_solves) {
  if (!enabled) return;
  auto it = std::find_if(traces.begin(), traces.end(),
                         [&](const RootSubsolveTrace& entry) {
                           return entry.name == name;
                         });
  if (it == traces.end()) {
    traces.push_back(RootSubsolveTrace{});
    it = std::prev(traces.end());
    it->name = name;
  }
  ++it->calls;
  if (success) ++it->successes;
  it->total_ms += ms;
  if (incumbent_hit) ++it->incumbent_hits;
  it->lp_solves += std::max(0, lp_solves);
}

void print_root_subsolve_traces(const std::vector<RootSubsolveTrace>& traces,
                                int root_frac_bin_count,
                                const std::string& root_incumbent_source) {
  std::fprintf(stderr, "[ROOT-SUBSOLVES] frac_bin=%d incumbent_source=%s\n",
               root_frac_bin_count, root_incumbent_source.c_str());
  for (const auto& entry : traces) {
    std::fprintf(stderr,
                 "[ROOT-SUBSOLVE] %s calls=%d ok=%d inc=%d lps=%d ms=%.0f\n",
                 entry.name.c_str(), entry.calls, entry.successes,
                 entry.incumbent_hits, entry.lp_solves, entry.total_ms);
  }
}

void print_root_phase_timing(std::chrono::steady_clock::time_point t0,
                             std::chrono::steady_clock::time_point t_root0,
                             std::chrono::steady_clock::time_point t_root1,
                             std::chrono::steady_clock::time_point t_cuts_done,
                             std::chrono::steady_clock::time_point t_pump_done,
                             std::chrono::steady_clock::time_point t_prog_done,
                             std::chrono::steady_clock::time_point t_dive_done,
                             std::chrono::steady_clock::time_point t_lns_done,
                             bool has_incumbent,
                             bool trace_root_subsolves,
                             const std::vector<RootSubsolveTrace>& traces,
                             int root_frac_bin_count,
                             const std::string& root_incumbent_source) {
  if (!trace_root_subsolves) return;
  const auto t_now = std::chrono::steady_clock::now();
  const double t_root_lp =
      std::chrono::duration<double, std::milli>(t_root1 - t_root0).count();
  const double t_cuts =
      std::chrono::duration<double, std::milli>(t_cuts_done - t_root1).count();
  const double t_pump =
      std::chrono::duration<double, std::milli>(t_pump_done - t_cuts_done)
          .count();
  const double t_prog =
      std::chrono::duration<double, std::milli>(t_prog_done - t_pump_done)
          .count();
  const double t_dive =
      std::chrono::duration<double, std::milli>(t_dive_done - t_prog_done)
          .count();
  const double t_lns =
      std::chrono::duration<double, std::milli>(t_lns_done - t_dive_done)
          .count();
  const double t_rest =
      std::chrono::duration<double, std::milli>(t_now - t_lns_done).count();
  const double t_total =
      std::chrono::duration<double, std::milli>(t_now - t0).count();
  std::fprintf(stderr,
               "[ROOT-PHASES] lp=%.0f cuts=%.0f pump=%.0f prog=%.0f "
               "dive=%.0f lns=%.0f rest=%.0f total=%.0fms inc=%d\n",
               t_root_lp, t_cuts, t_pump, t_prog, t_dive, t_lns, t_rest,
               t_total, has_incumbent ? 1 : 0);
  print_root_subsolve_traces(traces, root_frac_bin_count,
                             root_incumbent_source);
}

}  // namespace mipsolvers::engine::detail
