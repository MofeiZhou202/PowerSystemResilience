// dual_simplex_api.cpp — extracted public API helpers from dual_simplex.cpp

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace mipsolvers::engine {

namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
// VariableMeta uses +/-1e20 as its public no-bound sentinel.  Canonical LP
// state must contain mathematical infinities, never huge finite pseudo-bounds
// that contaminate bound-side selection and residual reconstruction.
constexpr double kModelBoundSentinel = 1e19;
std::atomic<std::uint64_t> next_structure_id{1};

bool has_finite_lower_bound(double value) {
  return std::isfinite(value) && value > -kModelBoundSentinel;
}

bool has_finite_upper_bound(double value) {
  return std::isfinite(value) && value < kModelBoundSentinel;
}
}  // namespace

StandardFormLP build_standard_form_lp(const LPModel& lp) {
  const int n = static_cast<int>(lp.vars.size());
  StandardFormLP sf;
  sf.structure_id = next_structure_id.fetch_add(1, std::memory_order_relaxed);
  sf.n_original = n;
  sf.original_types.reserve(static_cast<size_t>(n));
  sf.lb_shift = Eigen::VectorXd::Zero(n);

  // Row metadata: sense + original coefficient data (sparse).
  struct RowInfo {
    double rhs{0.0};
    double rhs_value{0.0};
    double slack_ub{kInf};
    char sense{'L'};
    int sign{1};  // +1 if not flipped, -1 if flipped
  };

  // Compute lb_shift.
  for (int i = 0; i < n; ++i) {
    sf.original_types.push_back(lp.vars[i].type);
    sf.lb_shift[i] =
        has_finite_lower_bound(lp.vars[i].lb) ? lp.vars[i].lb : 0.0;
  }

  // Count rows: inequality + equality only (upper bounds handled implicitly).
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int m = m_ineq + m_eq;

  std::vector<RowInfo> rows(static_cast<size_t>(m));

  // Compute shifted RHS for inequality rows using sparse matmul.  Ranged
  // rows lhs <= a*x <= rhs are kept as one equality with a bounded slack:
  //   a*(x-lb) + s = rhs - a*lb, 0 <= s <= rhs-lhs.
  // If the upper-side RHS is negative at the lb-shift point, orient the same
  // row from the lower side:
  //   -a*(x-lb) + s = -lhs + a*lb, 0 <= s <= rhs-lhs.
  if (m_ineq > 0) {
    const Eigen::VectorXd a_lb = lp.A * sf.lb_shift.head(lp.A.cols());
    for (int i = 0; i < m_ineq; ++i) {
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      const double rhs = lp.b[i];
      if (!std::isfinite(rhs) && std::isfinite(lhs)) {
        // One-sided G-type row A x >= lhs (b = +inf): surplus form
        // A x - s = lhs with s >= 0.  Keeping rhs = +inf would poison the
        // standard-form RHS (inf → NaN in Phase I).
        rows[i].rhs_value = lhs;
        rows[i].rhs = lhs - a_lb[i];
        rows[i].sense = 'G';
        rows[i].sign = 1;
        rows[i].slack_ub = kInf;
        continue;
      }
      rows[i].rhs_value = rhs;
      rows[i].rhs = rhs - a_lb[i];
      rows[i].sense = 'L';
      rows[i].sign = 1;
      if (std::isfinite(lhs) && std::isfinite(rhs)) {
        rows[i].slack_ub = std::max(0.0, rhs - lhs);
        if (rows[i].rhs < 0.0) {
          rows[i].rhs_value = lhs;
          rows[i].rhs = -lhs + a_lb[i];
          rows[i].sign = -1;
        }
      }
    }
  }

  // Compute shifted RHS for equality rows.
  if (m_eq > 0) {
    const Eigen::VectorXd aeq_lb = lp.Aeq * sf.lb_shift.head(lp.Aeq.cols());
    for (int i = 0; i < m_eq; ++i) {
      rows[m_ineq + i].rhs = lp.beq[i] - aeq_lb[i];
      rows[m_ineq + i].rhs_value = lp.beq[i];
      rows[m_ineq + i].sense = 'E';
      rows[m_ineq + i].sign = 1;
    }
  }

  // Flip rows with negative RHS to maintain non-negative RHS.
  for (int i = 0; i < m; ++i) {
    if (std::isfinite(rows[i].slack_ub)) continue;
    if (rows[i].rhs < 0.0) {
      rows[i].rhs *= -1.0;
      rows[i].sign = -1;
      if (rows[i].sense == 'L') {
        rows[i].sense = 'G';
      } else if (rows[i].sense == 'G') {
        rows[i].sense = 'L';
      }
    }
  }

  // Count slack/surplus/artificial variables.
  int n_slack = 0, n_surplus = 0, n_artificial = 0;
  sf.row_to_slack_col.assign(static_cast<size_t>(m), -1);
  sf.row_to_surplus_col.assign(static_cast<size_t>(m), -1);
  sf.row_to_artificial_col.assign(static_cast<size_t>(m), -1);

  for (int i = 0; i < m; ++i) {
    if (rows[i].sense == 'L') ++n_slack;
    else if (rows[i].sense == 'G') { ++n_surplus; ++n_artificial; }
    else ++n_artificial;
  }

  sf.n_slack = n_slack;
  sf.n_surplus = n_surplus;
  sf.n_artificial = n_artificial;

  const int slack_offset = n;
  const int surplus_offset = n + n_slack;
  const int artificial_offset = n + n_slack + n_surplus;

  int slack_cursor = 0, surplus_cursor = 0, artificial_cursor = 0;
  for (int i = 0; i < m; ++i) {
    if (rows[i].sense == 'L') {
      sf.row_to_slack_col[i] = slack_offset + slack_cursor++;
    } else if (rows[i].sense == 'G') {
      sf.row_to_surplus_col[i] = surplus_offset + surplus_cursor++;
      sf.row_to_artificial_col[i] = artificial_offset + artificial_cursor++;
    } else {
      sf.row_to_artificial_col[i] = artificial_offset + artificial_cursor++;
    }
  }

  const int cols = n + n_slack + n_surplus + n_artificial;

  // Build sparse A using triplets (no upper-bound rows).
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(lp.A.nonZeros() + lp.Aeq.nonZeros() + m));

  // Inequality rows from lp.A (sparse).
  for (int col = 0; col < lp.A.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      const int r = static_cast<int>(it.row());
      const double v = rows[r].sign * it.value();
      if (std::abs(v) > 1e-15) {
        triplets.emplace_back(r, col, v);
      }
    }
  }

  // Equality rows from lp.Aeq (sparse).
  for (int col = 0; col < lp.Aeq.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, col); it; ++it) {
      const int r = m_ineq + static_cast<int>(it.row());
      const double v = rows[r].sign * it.value();
      if (std::abs(v) > 1e-15) {
        triplets.emplace_back(r, col, v);
      }
    }
  }

  // Slack, surplus, artificial columns.
  for (int i = 0; i < m; ++i) {
    if (sf.row_to_slack_col[i] >= 0) {
      triplets.emplace_back(i, sf.row_to_slack_col[i], 1.0);
    }
    if (sf.row_to_surplus_col[i] >= 0) {
      triplets.emplace_back(i, sf.row_to_surplus_col[i], -1.0);
    }
    if (sf.row_to_artificial_col[i] >= 0) {
      triplets.emplace_back(i, sf.row_to_artificial_col[i], 1.0);
    }
  }

  sf.A.resize(m, cols);
  sf.A.setFromTriplets(triplets.begin(), triplets.end());
  sf.A.makeCompressed();

  // Build row-major copy for efficient row access (used by GMI cuts).
  sf.A_row = sf.A;

  // Store row signs (no upper-bound rows).
  sf.row_sign.resize(static_cast<size_t>(m));
  sf.row_rhs_value.resize(m);
  sf.ub_row_var.assign(static_cast<size_t>(m), -1);
  for (int i = 0; i < m; ++i) {
    sf.row_sign[i] = rows[i].sign;
    sf.row_rhs_value[i] = rows[i].rhs_value;
  }

  // RHS and objective.
  sf.b = Eigen::VectorXd::Zero(m);
  for (int i = 0; i < m; ++i) {
    sf.b[i] = rows[i].rhs;
  }

  const Eigen::VectorXd c_min = (lp.sense == Sense::Minimize) ? lp.c : (-lp.c);
  sf.objective_const = c_min.dot(sf.lb_shift);
  sf.c_max = Eigen::VectorXd::Zero(cols);
  sf.c_max.head(n) = -c_min;

  // Populate var_ub: upper bounds for each column in shifted space.
  // Original variables: ub - lb_shift (or +inf if unbounded).
  // Ranged-row slacks get the row width; other slack/surplus/artificial
  // columns are +inf.
  sf.var_ub = Eigen::VectorXd::Constant(cols, kInf);
  for (int i = 0; i < n; ++i) {
    if (has_finite_upper_bound(lp.vars[i].ub)) {
      sf.var_ub[i] = lp.vars[i].ub - sf.lb_shift[i];
    }
  }
  for (int i = 0; i < m; ++i) {
    const int slack_col = sf.row_to_slack_col[static_cast<std::size_t>(i)];
    if (slack_col >= 0 && std::isfinite(rows[i].slack_ub)) {
      sf.var_ub[slack_col] = rows[i].slack_ub;
    }
  }

  // Build O(1) reverse lookup: auxiliary column → row.
  sf.aux_col_to_row.assign(static_cast<std::size_t>(cols), -1);
  for (int i = 0; i < m; ++i) {
    if (sf.row_to_slack_col[i] >= 0)
      sf.aux_col_to_row[static_cast<std::size_t>(sf.row_to_slack_col[i])] = i;
    if (sf.row_to_surplus_col[i] >= 0)
      sf.aux_col_to_row[static_cast<std::size_t>(sf.row_to_surplus_col[i])] = i;
  }

  return sf;
}

bool append_leq_rows_to_standard_form(
    const StandardFormLP& base_sf,
    const std::vector<Eigen::SparseVector<double>>& rows,
    const std::vector<double>& rhs,
    StandardFormLP& out,
    double feasibility_tol) {
  (void)feasibility_tol;
  const int n_new_rows = static_cast<int>(rows.size());
  if (n_new_rows <= 0 || rhs.size() != rows.size()) return false;

  const int old_m = static_cast<int>(base_sf.A.rows());
  const int old_n = static_cast<int>(base_sf.A.cols());
  const int n_orig = base_sf.n_original;
  const int old_slack = base_sf.n_slack;
  const int old_surplus = base_sf.n_surplus;
  const int old_artificial = base_sf.n_artificial;
  if (n_orig < 0 || old_slack < 0 || old_surplus < 0 ||
      old_artificial < 0) {
    return false;
  }
  if (old_n != n_orig + old_slack + old_surplus + old_artificial) {
    return false;
  }
  if (base_sf.lb_shift.size() < n_orig ||
      base_sf.c_max.size() != old_n ||
      base_sf.var_ub.size() != old_n ||
      static_cast<int>(base_sf.row_to_slack_col.size()) != old_m ||
      static_cast<int>(base_sf.row_to_surplus_col.size()) != old_m ||
      static_cast<int>(base_sf.row_to_artificial_col.size()) != old_m) {
    return false;
  }
  const bool scaled = base_sf.row_scale.size() == old_m &&
                      base_sf.col_scale.size() == old_n;
  if ((base_sf.row_scale.size() != 0 && base_sf.row_scale.size() != old_m) ||
      (base_sf.col_scale.size() != 0 && base_sf.col_scale.size() != old_n)) {
    return false;
  }

  // Existing standard-form rows are [ineq | eq].  New <= rows must be inserted
  // before the trailing equality block to match build_standard_form_lp.
  int old_m_eq = 0;
  for (int i = old_m - 1; i >= 0; --i) {
    const bool is_eq =
        base_sf.row_to_slack_col[static_cast<std::size_t>(i)] < 0 &&
        base_sf.row_to_surplus_col[static_cast<std::size_t>(i)] < 0 &&
        base_sf.row_to_artificial_col[static_cast<std::size_t>(i)] >= 0;
    if (!is_eq) break;
    ++old_m_eq;
  }
  const int old_m_ineq = old_m - old_m_eq;
  const int new_m = old_m + n_new_rows;
  const int new_slack = old_slack + n_new_rows;
  const int new_n = old_n + n_new_rows;
  const int old_surplus_offset = n_orig + old_slack;
  const int old_artificial_offset = old_surplus_offset + old_surplus;
  const int new_slack_offset = n_orig + old_slack;

  std::vector<double> shifted_rhs(static_cast<std::size_t>(n_new_rows), 0.0);
  for (int k = 0; k < n_new_rows; ++k) {
    if (rows[static_cast<std::size_t>(k)].size() != n_orig &&
        rows[static_cast<std::size_t>(k)].size() != 0) {
      return false;
    }
    double a_lb = 0.0;
    for (Eigen::SparseVector<double>::InnerIterator it(
             rows[static_cast<std::size_t>(k)]);
         it; ++it) {
      const int j = static_cast<int>(it.index());
      if (j < 0 || j >= n_orig || !std::isfinite(it.value())) return false;
      a_lb += it.value() * base_sf.lb_shift[j];
    }
    const double b = rhs[static_cast<std::size_t>(k)] - a_lb;
    if (!std::isfinite(b)) {
      return false;
    }
    shifted_rhs[static_cast<std::size_t>(k)] = b;
  }

  auto map_old_row = [&](int r) -> int {
    return (r < old_m_ineq) ? r : r + n_new_rows;
  };
  auto map_old_col = [&](int c) -> int {
    if (c < old_surplus_offset) return c;  // original + old slacks
    if (c < old_artificial_offset + old_artificial) return c + n_new_rows;
    return -1;
  };

  out = StandardFormLP{};
  out.structure_id = next_structure_id.fetch_add(1, std::memory_order_relaxed);
  out.n_original = n_orig;
  out.n_slack = new_slack;
  out.n_surplus = old_surplus;
  out.n_artificial = old_artificial;
  out.original_types = base_sf.original_types;
  out.lb_shift = base_sf.lb_shift;
  out.objective_const = base_sf.objective_const;

  out.row_to_slack_col.assign(static_cast<std::size_t>(new_m), -1);
  out.row_to_surplus_col.assign(static_cast<std::size_t>(new_m), -1);
  out.row_to_artificial_col.assign(static_cast<std::size_t>(new_m), -1);
  out.row_sign.assign(static_cast<std::size_t>(new_m), 1);
  out.row_rhs_value = Eigen::VectorXd::Zero(new_m);
  out.ub_row_var.assign(static_cast<std::size_t>(new_m), -1);
  out.b = Eigen::VectorXd::Zero(new_m);
  out.c_max = Eigen::VectorXd::Zero(new_n);
  out.var_ub = Eigen::VectorXd::Constant(new_n, kInf);

  for (int r = 0; r < old_m; ++r) {
    const int nr = map_old_row(r);
    const int s = base_sf.row_to_slack_col[static_cast<std::size_t>(r)];
    const int u = base_sf.row_to_surplus_col[static_cast<std::size_t>(r)];
    const int a = base_sf.row_to_artificial_col[static_cast<std::size_t>(r)];
    out.row_to_slack_col[static_cast<std::size_t>(nr)] =
        s >= 0 ? map_old_col(s) : -1;
    out.row_to_surplus_col[static_cast<std::size_t>(nr)] =
        u >= 0 ? map_old_col(u) : -1;
    out.row_to_artificial_col[static_cast<std::size_t>(nr)] =
        a >= 0 ? map_old_col(a) : -1;
    if (r < static_cast<int>(base_sf.row_sign.size())) {
      out.row_sign[static_cast<std::size_t>(nr)] =
          base_sf.row_sign[static_cast<std::size_t>(r)];
    }
    if (base_sf.row_rhs_value.size() == old_m) {
      out.row_rhs_value[nr] = base_sf.row_rhs_value[r];
    }
    if (r < static_cast<int>(base_sf.ub_row_var.size())) {
      out.ub_row_var[static_cast<std::size_t>(nr)] =
          base_sf.ub_row_var[static_cast<std::size_t>(r)];
    }
    out.b[nr] = base_sf.b[r];
  }
  for (int k = 0; k < n_new_rows; ++k) {
    const int r = old_m_ineq + k;
    out.row_to_slack_col[static_cast<std::size_t>(r)] =
        new_slack_offset + k;
    out.row_sign[static_cast<std::size_t>(r)] = 1;
    out.row_rhs_value[r] = rhs[static_cast<std::size_t>(k)];
    out.b[r] = shifted_rhs[static_cast<std::size_t>(k)];
  }

  for (int c = 0; c < old_n; ++c) {
    const int nc = map_old_col(c);
    if (nc < 0 || nc >= new_n) return false;
    out.c_max[nc] = base_sf.c_max[c];
    out.var_ub[nc] = base_sf.var_ub[c];
  }

  if (scaled) {
    out.row_scale = Eigen::VectorXd::Ones(new_m);
    for (int r = 0; r < old_m; ++r) {
      out.row_scale[map_old_row(r)] = base_sf.row_scale[r];
    }
    out.col_scale = Eigen::VectorXd::Ones(new_n);
    for (int c = 0; c < old_n; ++c) {
      out.col_scale[map_old_col(c)] = base_sf.col_scale[c];
    }
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(
      base_sf.A.nonZeros() + n_new_rows +
      std::accumulate(rows.begin(), rows.end(), 0,
                      [](int acc, const Eigen::SparseVector<double>& v) {
                        return acc + static_cast<int>(v.nonZeros());
                      })));

  for (int c = 0; c < base_sf.A.outerSize(); ++c) {
    const int nc = map_old_col(c);
    if (nc < 0) return false;
    for (Eigen::SparseMatrix<double>::InnerIterator it(base_sf.A, c); it;
         ++it) {
      triplets.emplace_back(map_old_row(static_cast<int>(it.row())), nc,
                            it.value());
    }
  }

  for (int k = 0; k < n_new_rows; ++k) {
    const int r = old_m_ineq + k;
    for (Eigen::SparseVector<double>::InnerIterator it(
             rows[static_cast<std::size_t>(k)]);
         it; ++it) {
      const int j = static_cast<int>(it.index());
      const double cs = scaled ? base_sf.col_scale[j] : 1.0;
      const double v = it.value() * cs;
      if (std::abs(v) > 1e-15) triplets.emplace_back(r, j, v);
    }
    triplets.emplace_back(r, new_slack_offset + k, 1.0);
  }

  out.A.resize(new_m, new_n);
  out.A.setFromTriplets(triplets.begin(), triplets.end());
  out.A.makeCompressed();
  out.A_row = out.A;

  // Build O(1) reverse lookup for the updated standard form.
  out.aux_col_to_row.assign(static_cast<std::size_t>(new_n), -1);
  for (int i = 0; i < new_m; ++i) {
    if (out.row_to_slack_col[i] >= 0)
      out.aux_col_to_row[static_cast<std::size_t>(out.row_to_slack_col[i])] = i;
    if (out.row_to_surplus_col[i] >= 0)
      out.aux_col_to_row[static_cast<std::size_t>(out.row_to_surplus_col[i])] = i;
  }
  return true;
}

bool append_leq_rows_to_canonical_standard_form(
    const StandardFormLP& base_sf,
    const std::vector<Eigen::SparseVector<double>>& rows,
    const std::vector<double>& rhs,
    StandardFormLP& out,
    int ruiz_rounds,
    double feasibility_tol) {
  (void)feasibility_tol;
  const int n_new_rows = static_cast<int>(rows.size());
  if (n_new_rows <= 0 || rhs.size() != rows.size()) return false;

  const int old_m = static_cast<int>(base_sf.A.rows());
  const int old_n = static_cast<int>(base_sf.A.cols());
  const int n_orig = base_sf.n_original;
  if (n_orig < 0 ||
      old_n != n_orig + base_sf.n_slack + base_sf.n_surplus +
                   base_sf.n_artificial ||
      base_sf.lb_shift.size() < n_orig ||
      base_sf.c_max.size() != old_n ||
      base_sf.var_ub.size() != old_n ||
      static_cast<int>(base_sf.row_to_slack_col.size()) != old_m ||
      static_cast<int>(base_sf.row_to_surplus_col.size()) != old_m ||
      static_cast<int>(base_sf.row_to_artificial_col.size()) != old_m ||
      static_cast<int>(base_sf.row_sign.size()) != old_m ||
      base_sf.row_rhs_value.size() != old_m) {
    return false;
  }
  if ((base_sf.row_scale.size() != 0 && base_sf.row_scale.size() != old_m) ||
      (base_sf.col_scale.size() != 0 && base_sf.col_scale.size() != old_n)) {
    return false;
  }

  int old_m_eq = 0;
  for (int i = old_m - 1; i >= 0; --i) {
    const bool is_eq =
        base_sf.row_to_slack_col[static_cast<std::size_t>(i)] < 0 &&
        base_sf.row_to_surplus_col[static_cast<std::size_t>(i)] < 0 &&
        base_sf.row_to_artificial_col[static_cast<std::size_t>(i)] >= 0;
    if (!is_eq) break;
    ++old_m_eq;
  }
  const int old_m_ineq = old_m - old_m_eq;

  struct NewRowInfo {
    double rhs_shifted{0.0};
    double rhs_value{0.0};
    int sign{1};
    char sense{'L'};
  };
  std::vector<NewRowInfo> new_rows(static_cast<std::size_t>(n_new_rows));
  int add_slack = 0;
  int add_surplus = 0;
  int add_artificial = 0;
  for (int k = 0; k < n_new_rows; ++k) {
    const auto& row = rows[static_cast<std::size_t>(k)];
    if ((row.size() != n_orig && row.size() != 0) ||
        !std::isfinite(rhs[static_cast<std::size_t>(k)])) {
      return false;
    }
    double a_lb = 0.0;
    for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
      const int j = static_cast<int>(it.index());
      if (j < 0 || j >= n_orig || !std::isfinite(it.value())) return false;
      a_lb += it.value() * base_sf.lb_shift[j];
    }
    double shifted = rhs[static_cast<std::size_t>(k)] - a_lb;
    if (!std::isfinite(shifted)) return false;
    NewRowInfo info;
    info.rhs_value = rhs[static_cast<std::size_t>(k)];
    if (shifted < 0.0) {
      shifted = -shifted;
      info.sign = -1;
      info.sense = 'G';
      ++add_surplus;
      ++add_artificial;
    } else {
      info.sense = 'L';
      ++add_slack;
    }
    info.rhs_shifted = shifted;
    new_rows[static_cast<std::size_t>(k)] = info;
  }

  const int new_m = old_m + n_new_rows;
  const int new_slack = base_sf.n_slack + add_slack;
  const int new_surplus = base_sf.n_surplus + add_surplus;
  const int new_artificial = base_sf.n_artificial + add_artificial;
  const int new_n = n_orig + new_slack + new_surplus + new_artificial;
  const int old_surplus_offset = n_orig + base_sf.n_slack;
  const int old_artificial_offset = old_surplus_offset + base_sf.n_surplus;
  const int new_slack_insert = n_orig + base_sf.n_slack;
  const int new_surplus_insert = n_orig + new_slack + base_sf.n_surplus;
  int old_ineq_artificial = 0;
  for (int r = 0; r < old_m_ineq; ++r) {
    if (base_sf.row_to_artificial_col[static_cast<std::size_t>(r)] >= 0) {
      ++old_ineq_artificial;
    }
  }
  const int new_artificial_insert =
      n_orig + new_slack + new_surplus + old_ineq_artificial;

  auto map_old_row = [&](int r) -> int {
    return (r < old_m_ineq) ? r : r + n_new_rows;
  };
  auto map_old_col = [&](int c) -> int {
    if (c < n_orig + base_sf.n_slack) return c;
    if (c < old_artificial_offset) return c + add_slack;
    if (c < old_artificial_offset + base_sf.n_artificial) {
      const int old_art_index = c - old_artificial_offset;
      return c + add_slack + add_surplus +
             (old_art_index < old_ineq_artificial ? 0 : add_artificial);
    }
    return -1;
  };

  out = StandardFormLP{};
  out.structure_id = next_structure_id.fetch_add(1, std::memory_order_relaxed);
  out.n_original = n_orig;
  out.n_slack = new_slack;
  out.n_surplus = new_surplus;
  out.n_artificial = new_artificial;
  out.original_types = base_sf.original_types;
  out.lb_shift = base_sf.lb_shift;
  out.objective_const = base_sf.objective_const;
  out.row_to_slack_col.assign(static_cast<std::size_t>(new_m), -1);
  out.row_to_surplus_col.assign(static_cast<std::size_t>(new_m), -1);
  out.row_to_artificial_col.assign(static_cast<std::size_t>(new_m), -1);
  out.row_sign.assign(static_cast<std::size_t>(new_m), 1);
  out.row_rhs_value = Eigen::VectorXd::Zero(new_m);
  out.ub_row_var.assign(static_cast<std::size_t>(new_m), -1);
  out.b = Eigen::VectorXd::Zero(new_m);
  out.c_max = Eigen::VectorXd::Zero(new_n);
  out.var_ub = Eigen::VectorXd::Constant(new_n, kInf);

  for (int r = 0; r < old_m; ++r) {
    const int nr = map_old_row(r);
    const int s = base_sf.row_to_slack_col[static_cast<std::size_t>(r)];
    const int u = base_sf.row_to_surplus_col[static_cast<std::size_t>(r)];
    const int a = base_sf.row_to_artificial_col[static_cast<std::size_t>(r)];
    out.row_to_slack_col[static_cast<std::size_t>(nr)] =
        s >= 0 ? map_old_col(s) : -1;
    out.row_to_surplus_col[static_cast<std::size_t>(nr)] =
        u >= 0 ? map_old_col(u) : -1;
    out.row_to_artificial_col[static_cast<std::size_t>(nr)] =
        a >= 0 ? map_old_col(a) : -1;
    out.row_sign[static_cast<std::size_t>(nr)] =
        base_sf.row_sign[static_cast<std::size_t>(r)];
    out.row_rhs_value[nr] = base_sf.row_rhs_value[r];
    if (r < static_cast<int>(base_sf.ub_row_var.size())) {
      out.ub_row_var[static_cast<std::size_t>(nr)] =
          base_sf.ub_row_var[static_cast<std::size_t>(r)];
    }
    const double rs = base_sf.row_scale.size() == old_m ? base_sf.row_scale[r]
                                                        : 1.0;
    out.b[nr] = rs != 0.0 ? base_sf.b[r] / rs : base_sf.b[r];
  }

  int slack_cursor = 0;
  int surplus_cursor = 0;
  int artificial_cursor = 0;
  for (int k = 0; k < n_new_rows; ++k) {
    const int r = old_m_ineq + k;
    const NewRowInfo& info = new_rows[static_cast<std::size_t>(k)];
    out.row_sign[static_cast<std::size_t>(r)] = info.sign;
    out.row_rhs_value[r] = info.rhs_value;
    out.b[r] = info.rhs_shifted;
    if (info.sense == 'L') {
      out.row_to_slack_col[static_cast<std::size_t>(r)] =
          new_slack_insert + slack_cursor++;
    } else {
      out.row_to_surplus_col[static_cast<std::size_t>(r)] =
          new_surplus_insert + surplus_cursor++;
      out.row_to_artificial_col[static_cast<std::size_t>(r)] =
          new_artificial_insert + artificial_cursor++;
    }
  }

  for (int c = 0; c < old_n; ++c) {
    const int nc = map_old_col(c);
    if (nc < 0 || nc >= new_n) return false;
    const double cs = base_sf.col_scale.size() == old_n ? base_sf.col_scale[c]
                                                        : 1.0;
    out.c_max[nc] = cs != 0.0 ? base_sf.c_max[c] / cs : base_sf.c_max[c];
    out.var_ub[nc] =
        (std::isfinite(base_sf.var_ub[c]) && cs != 0.0)
            ? base_sf.var_ub[c] * cs
            : base_sf.var_ub[c];
  }

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(
      base_sf.A.nonZeros() + n_new_rows * 2 +
      std::accumulate(rows.begin(), rows.end(), 0,
                      [](int acc, const Eigen::SparseVector<double>& v) {
                        return acc + static_cast<int>(v.nonZeros());
                      })));

  for (int c = 0; c < base_sf.A.outerSize(); ++c) {
    const int nc = map_old_col(c);
    if (nc < 0) return false;
    const double old_cs =
        base_sf.col_scale.size() == old_n ? base_sf.col_scale[c] : 1.0;
    for (Eigen::SparseMatrix<double>::InnerIterator it(base_sf.A, c); it;
         ++it) {
      const int old_r = static_cast<int>(it.row());
      const double old_rs =
          base_sf.row_scale.size() == old_m ? base_sf.row_scale[old_r] : 1.0;
      const double denom = old_rs * old_cs;
      const double v = denom != 0.0 ? it.value() / denom : it.value();
      if (std::abs(v) > 1e-15) {
        triplets.emplace_back(map_old_row(old_r), nc, v);
      }
    }
  }

  for (int k = 0; k < n_new_rows; ++k) {
    const int r = old_m_ineq + k;
    const int sign = new_rows[static_cast<std::size_t>(k)].sign;
    for (Eigen::SparseVector<double>::InnerIterator it(
             rows[static_cast<std::size_t>(k)]);
         it; ++it) {
      const double v = sign * it.value();
      if (std::abs(v) > 1e-15) {
        triplets.emplace_back(r, static_cast<int>(it.index()), v);
      }
    }
    const int slack = out.row_to_slack_col[static_cast<std::size_t>(r)];
    const int surplus = out.row_to_surplus_col[static_cast<std::size_t>(r)];
    const int artificial = out.row_to_artificial_col[static_cast<std::size_t>(r)];
    if (slack >= 0) triplets.emplace_back(r, slack, 1.0);
    if (surplus >= 0) triplets.emplace_back(r, surplus, -1.0);
    if (artificial >= 0) triplets.emplace_back(r, artificial, 1.0);
  }

  out.A.resize(new_m, new_n);
  out.A.setFromTriplets(triplets.begin(), triplets.end());
  out.A.makeCompressed();
  out.A_row = out.A;
  if (ruiz_rounds > 0) {
    ruiz_scale_standard_form(out, ruiz_rounds);
  }
  return true;
}

// ---------------------------------------------------------------------------
// update_standard_form_cost — swap cost vector in place, preserving A, b,
// bounds, row/col scales, and any persisted basis factorization.
// ---------------------------------------------------------------------------
// Mirrors the transformation used in build_standard_form_lp:
//   c_min = (sense == Minimize) ? new_c : (-new_c)
//   c_max.head(n_orig) = -c_min          (internal convention: max c_max^T y)
//   objective_const   = c_min.dot(lb_shift)
// Slack/surplus/artificial entries of c_max stay zero. If Ruiz scaling was
// applied (col_scale non-empty), c_max is multiplied element-wise by
// col_scale on the original columns.
//
// Callers reusing a SimplexBasis hint across this change MUST clear its
// cached_reduced_costs because it depends on c_max.
// The SparseBasis LU factorization and cached B^{-1} depend only on A and
// basis indices; they remain valid.
void update_standard_form_cost(StandardFormLP& sf,
                               Sense sense,
                               const Eigen::VectorXd& new_c) {
  const int n_orig = sf.n_original;
  if (static_cast<int>(new_c.size()) != n_orig) {
    throw std::runtime_error(
        "update_standard_form_cost: new_c.size() != sf.n_original");
  }

  // Apply sense flip to convert to internal min-form cost on shifted vars.
  Eigen::VectorXd c_min(n_orig);
  if (sense == Sense::Minimize) {
    c_min = new_c;
  } else {
    c_min = -new_c;
  }

  // objective_const uses the UNSCALED lb_shift values. When Ruiz scaling is
  // active, lb_shift itself is not rescaled (it is applied in original
  // variable space), so this formula is scale-independent.
  sf.objective_const = c_min.dot(sf.lb_shift);

  // Write the new c_max on original columns; leave slack/surplus/artificial
  // entries (already zero, and we keep them zero).
  sf.c_max.head(n_orig) = -c_min;
  // Apply Ruiz column scaling on original columns (scaled LP uses D_c * c).
  if (sf.col_scale.size() >= n_orig) {
    sf.c_max.head(n_orig).array() *= sf.col_scale.head(n_orig).array();
  }
  // Explicitly zero slack/surplus/artificial cost entries (defensive).
  const int cols = static_cast<int>(sf.c_max.size());
  if (cols > n_orig) {
    sf.c_max.segment(n_orig, cols - n_orig).setZero();
  }
}

// ---------------------------------------------------------------------------
// Ruiz Equilibration Scaling
// ---------------------------------------------------------------------------
// Iteratively scales rows and columns of A so that the infinity norm of each
// row and column is approximately 1.  This balances the constraint matrix,
// improving the condition number of basis matrices and reducing numerical
// issues in LU factorisation.
//
// Scaling transforms:
//   A_scaled = D_r * A * D_c
//   b_scaled = D_r * b
//   c_scaled = D_c * c_max   (since max c^T x  ↔  max (D_c c)^T (D_c^{-1} x))
//   var_ub_scaled[j] = var_ub[j] / col_scale[j]
//
// The objective value is invariant: c_scaled^T y = (D_c c)^T (D_c^{-1} x) = c^T x.
// Solution recovery: x_std[j] = col_scale[j] * y[j].
//
void ruiz_scale_standard_form(StandardFormLP& sf, int rounds) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  if (m == 0 || n == 0) return;

  // Initialize cumulative scale factors.
  sf.row_scale = Eigen::VectorXd::Ones(m);
  sf.col_scale = Eigen::VectorXd::Ones(n);

  for (int round = 0; round < rounds; ++round) {
    // Compute row infinity norms of current A.
    Eigen::VectorXd row_max = Eigen::VectorXd::Zero(m);
    Eigen::VectorXd col_max = Eigen::VectorXd::Zero(n);

    for (int j = 0; j < sf.A.outerSize(); ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
        const double v = std::abs(it.value());
        if (v > row_max[it.row()]) row_max[it.row()] = v;
        if (v > col_max[j]) col_max[j] = v;
      }
    }

    // Compute per-round scale factors: D_r[i] = 1/sqrt(max_j |A[i,j]|).
    double max_deviation = 0.0;
    Eigen::VectorXd dr(m), dc(n);
    for (int i = 0; i < m; ++i) {
      if (row_max[i] > 1e-15) {
        dr[i] = 1.0 / std::sqrt(row_max[i]);
        max_deviation = std::max(max_deviation, std::abs(row_max[i] - 1.0));
      } else {
        dr[i] = 1.0;
      }
    }
    for (int j = 0; j < n; ++j) {
      if (col_max[j] > 1e-15) {
        dc[j] = 1.0 / std::sqrt(col_max[j]);
        max_deviation = std::max(max_deviation, std::abs(col_max[j] - 1.0));
      } else {
        dc[j] = 1.0;
      }
    }

    // Early termination: if already well-balanced, stop.
    if (round >= 2 && max_deviation < 1e-2) break;

    // Apply D_r * A * D_c element-wise (column-major).
    for (int j = 0; j < sf.A.outerSize(); ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(sf.A, j); it; ++it) {
        it.valueRef() *= dr[it.row()] * dc[j];
      }
    }
    // Apply to row-major copy.
    for (int i = 0; i < sf.A_row.outerSize(); ++i) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row, i); it; ++it) {
        it.valueRef() *= dr[i] * dc[it.col()];
      }
    }

    // Scale b, c_max, var_ub.
    sf.b.array() *= dr.array();
    sf.c_max.array() *= dc.array();
    for (int j = 0; j < n; ++j) {
      if (std::isfinite(sf.var_ub[j])) {
        sf.var_ub[j] /= dc[j];
      }
    }

    // Accumulate.
    sf.row_scale.array() *= dr.array();
    sf.col_scale.array() *= dc.array();
  }
}

// ---------------------------------------------------------------------------
// update_standard_form_bounds — fast path for node LP solves.
// ---------------------------------------------------------------------------
// Recompute only b, lb_shift, and objective_const for new variable bounds.
// Uses incremental column updates when few lower bounds changed (typical for
// B&C branching: only 1-2 lb changes per node).  Falls back to full sparse
// matmul when many bounds changed.
void update_standard_form_bounds(StandardFormLP& sf,
                                 const LPModel& lp,
                                 const Eigen::VectorXd& node_lb,
                                 const Eigen::VectorXd& node_ub) {
  const int n = sf.n_original;
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());

  // Compute new lb_shift and detect which components changed.
  // Collect changed indices for incremental update.
  constexpr int kIncrementalThreshold = 50;
  int n_changed = 0;
  int first_unscanned = n;
  // Use a small inline buffer for the common case (1-5 changes).
  int changed_buf[64];
  double delta_buf[64];
  bool use_incremental = true;

  for (int i = 0; i < n; ++i) {
    const double new_shift = std::isfinite(node_lb[i]) ? node_lb[i] : 0.0;
    const double d = new_shift - sf.lb_shift[i];
    if (std::abs(d) > 1e-15) {
      if (n_changed < 64) {
        changed_buf[n_changed] = i;
        delta_buf[n_changed] = d;
      }
      ++n_changed;
      if (n_changed > kIncrementalThreshold) {
        use_incremental = false;
      }
    }
    sf.lb_shift[i] = new_shift;
    if (!use_incremental) {
      first_unscanned = i + 1;
      break;
    }
  }
  // If we short-circuited, finish updating lb_shift.
  if (!use_incremental) {
    for (int i = first_unscanned; i < n; ++i) {
      sf.lb_shift[i] = std::isfinite(node_lb[i]) ? node_lb[i] : 0.0;
    }
  }

  if (n_changed == 0) {
    // Only upper bounds changed — skip expensive matmul entirely.
  } else if (use_incremental && n_changed <= 64) {
    // Incremental column-by-column update: O(sum of nnz per changed column).
    // For branching on one binary variable, this is typically O(5-10) instead
    // of the full O(nnz_A) ≈ O(12000).
    const bool scaled = sf.row_scale.size() > 0;
    for (int k = 0; k < n_changed; ++k) {
      const int j = changed_buf[k];
      const double d = delta_buf[k];
      // Update inequality rows.
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
        const double rs = scaled ? sf.row_scale[it.row()] : 1.0;
        sf.b[it.row()] -= rs * sf.row_sign[it.row()] * it.value() * d;
      }
      // Update equality rows.
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
        const int row = m_ineq + static_cast<int>(it.row());
        const double rs = scaled ? sf.row_scale[row] : 1.0;
        sf.b[row] -= rs * sf.row_sign[row] * it.value() * d;
      }
    }
  } else {
    // Many bounds changed — fall back to full sparse matmul.
    if (m_ineq > 0) {
      const Eigen::VectorXd a_lb = lp.A * sf.lb_shift.head(lp.A.cols());
      for (int i = 0; i < m_ineq; ++i) {
        const double rhs_value =
            sf.row_rhs_value.size() == sf.b.size() ? sf.row_rhs_value[i]
                                                    : lp.b[i];
        sf.b[i] = sf.row_sign[i] * (rhs_value - a_lb[i]);
      }
    }
    if (m_eq > 0) {
      const Eigen::VectorXd aeq_lb = lp.Aeq * sf.lb_shift.head(lp.Aeq.cols());
      for (int i = 0; i < m_eq; ++i) {
        const int row = m_ineq + i;
        const double rhs_value =
            sf.row_rhs_value.size() == sf.b.size() ? sf.row_rhs_value[row]
                                                    : lp.beq[i];
        sf.b[row] = sf.row_sign[row] * (rhs_value - aeq_lb[i]);
      }
    }
    // Apply Ruiz row scaling to recomputed b.
    if (sf.row_scale.size() > 0) {
      const int m_total = m_ineq + m_eq;
      for (int i = 0; i < m_total; ++i)
        sf.b[i] *= sf.row_scale[i];
    }
  }

  // Update var_ub for original variables: ub - lb_shift (then scale).
  const bool has_col_scale = sf.col_scale.size() > 0;
  for (int j = 0; j < n; ++j) {
    if (std::isfinite(node_ub[j])) {
      double ub = node_ub[j] - sf.lb_shift[j];
      sf.var_ub[j] = has_col_scale ? (ub / sf.col_scale[j]) : ub;
    } else {
      sf.var_ub[j] = kInf;
    }
  }

  // Update objective constant incrementally when few bounds changed.
  if (use_incremental && n_changed > 0 && n_changed <= 64) {
    const double sign = (lp.sense == Sense::Minimize) ? 1.0 : -1.0;
    for (int k = 0; k < n_changed; ++k) {
      sf.objective_const += sign * lp.c[changed_buf[k]] * delta_buf[k];
    }
  } else {
    const Eigen::VectorXd c_min = (lp.sense == Sense::Minimize) ? lp.c : (-lp.c);
    sf.objective_const = c_min.dot(sf.lb_shift);
  }
}

// Fast-path update when the exact bound changes are known (avoids scanning
// all n variables to detect which changed).  Typical for B&C branching:
// 1-2 changes per node, plus a few more from bound propagation.
void update_standard_form_bounds_incremental(
    StandardFormLP& sf,
    const LPModel& lp,
    const std::vector<BoundChangeInfo>& changes) {
  if (changes.empty()) return;
  const int n = sf.n_original;
  const int m_ineq = static_cast<int>(lp.A.rows());
  const double sign = (lp.sense == Sense::Minimize) ? 1.0 : -1.0;

  const bool scaled = sf.row_scale.size() > 0;

  for (const auto& bc : changes) {
    const int j = bc.var_idx;
    if (j < 0 || j >= n) continue;

    if (bc.is_lb) {
      // Lower bound changed: update lb_shift and b.
      const double old_shift = sf.lb_shift[j];
      const double new_shift = std::isfinite(old_shift + bc.delta) ? (old_shift + bc.delta) : old_shift;
      const double d = new_shift - old_shift;
      if (std::abs(d) < 1e-15) continue;
      sf.lb_shift[j] = new_shift;

      // Update b for inequality rows: b[i] -= row_scale[i] * row_sign[i] * A(i,j) * d
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
        const double rs = scaled ? sf.row_scale[it.row()] : 1.0;
        sf.b[it.row()] -= rs * sf.row_sign[it.row()] * it.value() * d;
      }
      // Update b for equality rows.
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
        const int row = m_ineq + static_cast<int>(it.row());
        const double rs = scaled ? sf.row_scale[row] : 1.0;
        sf.b[row] -= rs * sf.row_sign[row] * it.value() * d;
      }

      // Update var_ub[j] (in scaled space): delta in original space / col_scale.
      if (std::isfinite(sf.var_ub[j]) && sf.var_ub[j] != kInf) {
        const double cs = scaled ? sf.col_scale[j] : 1.0;
        sf.var_ub[j] -= d / cs;
      }

      // Update objective constant.
      sf.objective_const += sign * lp.c[j] * d;
    } else {
      // Upper bound changed: only var_ub needs updating.
      if (std::isfinite(sf.var_ub[j]) || std::isfinite(bc.delta)) {
        const double cs = scaled ? sf.col_scale[j] : 1.0;
        const double old_ub = sf.var_ub[j] * cs + sf.lb_shift[j];
        const double new_ub = old_ub + bc.delta;
        sf.var_ub[j] = std::isfinite(new_ub) ? ((new_ub - sf.lb_shift[j]) / cs) : kInf;
      }
    }
  }
}


}  // namespace mipsolvers::engine
