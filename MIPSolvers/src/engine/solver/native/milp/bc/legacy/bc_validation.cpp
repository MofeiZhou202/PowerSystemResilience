/// @file bc_validation.cpp
/// @brief Incumbent validation and diagnostic formatting for B&C.

#include "mipsolvers/engine/detail/bc_validation.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <Eigen/Sparse>
#include <fmt/format.h>

#include "mipsolvers/engine/detail/bc_utils.hpp"

namespace mipsolvers::engine::detail {

IncumbentValidationSummary validate_incumbent_solution(
    const LPModel& lp,
    const Eigen::VectorXd& x,
    double tol) {
  IncumbentValidationSummary summary;
  summary.expected_size = static_cast<int>(lp.vars.size());
  summary.actual_size = static_cast<int>(x.size());
  if (summary.actual_size != summary.expected_size) {
    summary.size_ok = false;
    return summary;
  }

  for (int j = 0; j < summary.expected_size; ++j) {
    const auto& var = lp.vars[j];
    const double below = var.lb - x[j];
    const double above = x[j] - var.ub;
    const double viol = std::max(0.0, std::max(below, above));
    if (viol > summary.max_bound_violation) {
      summary.max_bound_violation = viol;
      summary.bound_var = j;
    }
    if (is_integer_type(var)) {
      const double frac = std::abs(x[j] - std::round(x[j]));
      if (frac > summary.max_integrality_violation) {
        summary.max_integrality_violation = frac;
        summary.frac_var = j;
      }
    }
  }
  summary.bounds_ok = summary.max_bound_violation <= tol;
  summary.integrality_ok = summary.max_integrality_violation <= tol;

  if (lp.A.rows() > 0) {
    const Eigen::VectorXd ax = lp.A * x;
    for (int i = 0; i < ax.size(); ++i) {
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      const double upper_viol = std::max(0.0, ax[i] - lp.b[i]);
      const double lower_viol =
          std::isfinite(lhs) ? std::max(0.0, lhs - ax[i]) : 0.0;
      const double viol = std::max(upper_viol, lower_viol);
      if (viol > summary.max_ineq_violation) {
        summary.max_ineq_violation = viol;
        summary.ineq_row = i;
        summary.ineq_row_activity = ax[i];
        summary.ineq_row_lhs = lhs;
        summary.ineq_row_rhs = lp.b[i];
        summary.ineq_row_side = (upper_viol >= lower_viol) ? "upper" : "lower";
      }
    }
    summary.ineq_ok = summary.max_ineq_violation <= tol;
  }

  if (lp.Aeq.rows() > 0) {
    const Eigen::VectorXd aeqx = lp.Aeq * x;
    for (int i = 0; i < aeqx.size(); ++i) {
      const double viol = std::abs(aeqx[i] - lp.beq[i]);
      if (viol > summary.max_eq_violation) {
        summary.max_eq_violation = viol;
        summary.eq_row = i;
      }
    }
    summary.eq_ok = summary.max_eq_violation <= tol;
  }

  return summary;
}

std::string format_incumbent_validation_failure(
    const char* stage,
    const LPModel& lp,
    const IncumbentValidationSummary& summary) {
  if (!summary.size_ok) {
    return fmt::format("{}: size mismatch (expected {}, got {})",
                       stage, summary.expected_size, summary.actual_size);
  }
  std::string msg = fmt::format(
      "{}: bounds={:.3g}",
      stage,
      summary.max_bound_violation);
  if (summary.bound_var >= 0 && summary.bound_var < static_cast<int>(lp.vars.size()) &&
      !lp.vars[summary.bound_var].name.empty()) {
    msg += fmt::format(" at var {} [{}]", summary.bound_var,
                       lp.vars[summary.bound_var].name);
  } else if (summary.bound_var >= 0) {
    msg += fmt::format(" at var {}", summary.bound_var);
  }
  msg += fmt::format(
      "; ineq={:.3g} row {}",
      summary.max_ineq_violation,
      summary.ineq_row);
  if (summary.ineq_row >= 0) {
    msg += fmt::format(" side={} activity={:.12g} lhs={:.12g} rhs={:.12g}",
                       summary.ineq_row_side,
                       summary.ineq_row_activity,
                       summary.ineq_row_lhs,
                       summary.ineq_row_rhs);
  }
  msg += fmt::format(
      "; eq={:.3g} row {}; integrality={:.3g}",
      summary.max_eq_violation,
      summary.eq_row,
      summary.max_integrality_violation);
  if (summary.frac_var >= 0 && summary.frac_var < static_cast<int>(lp.vars.size()) &&
      !lp.vars[summary.frac_var].name.empty()) {
    msg += fmt::format(" at integer var {} [{}]", summary.frac_var,
                       lp.vars[summary.frac_var].name);
  } else if (summary.frac_var >= 0) {
    msg += fmt::format(" at integer var {}", summary.frac_var);
  }
  return msg;
}

std::string format_ineq_row_contributors(
    const LPModel& lp,
    const Eigen::VectorXd& x,
    int row,
    int max_terms) {
  if (row < 0 || row >= lp.A.rows() || x.size() < static_cast<int>(lp.vars.size())) {
    return "";
  }
  struct TermInfo {
    int col{-1};
    double a{0.0};
    double x{0.0};
    double ax{0.0};
  };
  std::vector<TermInfo> terms;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = lp.A;
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, row);
       it; ++it) {
    const int j = static_cast<int>(it.col());
    if (j < 0 || j >= x.size()) continue;
    terms.push_back(TermInfo{j, it.value(), x[j], it.value() * x[j]});
  }
  std::sort(terms.begin(), terms.end(), [](const TermInfo& a, const TermInfo& b) {
    return std::abs(a.ax) > std::abs(b.ax);
  });
  if (terms.empty()) return "";
  std::string out = " top=[";
  const int k = std::min(static_cast<int>(terms.size()), std::max(0, max_terms));
  for (int p = 0; p < k; ++p) {
    const TermInfo& t = terms[static_cast<std::size_t>(p)];
    if (p > 0) out += ", ";
    out += fmt::format("c{}:{} a={:.6g} x={:.6g} ax={:.6g}",
                       t.col,
                       (t.col >= 0 && t.col < static_cast<int>(lp.vars.size()) &&
                        !lp.vars[static_cast<std::size_t>(t.col)].name.empty())
                           ? lp.vars[static_cast<std::size_t>(t.col)].name
                           : "?",
                       t.a,
                       t.x,
                       t.ax);
  }
  out += "]";
  return out;
}

#ifdef HACDCPF_HAVE_PAPILO
std::string format_papilo_reduced_row_mapping_detail(
    const PaPILOPresolveResult& ps,
    const LPModel& reduced_lp,
    const Eigen::VectorXd& x_reduced,
    int original_row) {
  if (original_row < 0 || x_reduced.size() < static_cast<int>(reduced_lp.vars.size())) {
    return "";
  }
  int matches = 0;
  int first_ineq = -1;
  bool first_negated = false;
  for (int r = 0; r < static_cast<int>(ps.reduced_ineq_to_orig_row.size()); ++r) {
    if (ps.reduced_ineq_to_orig_row[static_cast<std::size_t>(r)] == original_row) {
      if (first_ineq < 0) {
        first_ineq = r;
        first_negated =
            r < static_cast<int>(ps.reduced_ineq_negated.size()) &&
            ps.reduced_ineq_negated[static_cast<std::size_t>(r)] != 0;
      }
      ++matches;
    }
  }
  int first_eq = -1;
  for (int r = 0; r < static_cast<int>(ps.reduced_eq_to_orig_row.size()); ++r) {
    if (ps.reduced_eq_to_orig_row[static_cast<std::size_t>(r)] == original_row) {
      if (first_eq < 0) first_eq = r;
      ++matches;
    }
  }
  if (matches == 0) {
    return " reduced_map=absent";
  }
  if (first_ineq >= 0 && first_ineq < reduced_lp.A.rows()) {
    double activity = 0.0;
    Eigen::SparseMatrix<double, Eigen::RowMajor> Arow = reduced_lp.A;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Arow, first_ineq);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j >= 0 && j < x_reduced.size()) activity += it.value() * x_reduced[j];
    }
    const double lhs = lp_row_lhs_or_neg_inf(reduced_lp, first_ineq);
    const double rhs = reduced_lp.b[first_ineq];
    const double viol = std::max(
        std::max(0.0, activity - rhs),
        std::isfinite(lhs) ? std::max(0.0, lhs - activity) : 0.0);
    return fmt::format(
        " reduced_map=ineq:{}{} matches={} lhs_size={} A_rows={} activity={:.12g} lhs={:.12g} rhs={:.12g} viol={:.3g}",
        first_ineq,
        first_negated ? "(neg)" : "",
        matches,
        static_cast<int>(reduced_lp.row_lhs.size()),
        static_cast<int>(reduced_lp.A.rows()),
        activity,
        lhs,
        rhs,
        viol);
  }
  if (first_eq >= 0 && first_eq < reduced_lp.Aeq.rows()) {
    double activity = 0.0;
    Eigen::SparseMatrix<double, Eigen::RowMajor> Aeqrow = reduced_lp.Aeq;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeqrow, first_eq);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j >= 0 && j < x_reduced.size()) activity += it.value() * x_reduced[j];
    }
    const double rhs = reduced_lp.beq[first_eq];
    return fmt::format(
        " reduced_map=eq:{} matches={} activity={:.12g} rhs={:.12g} viol={:.3g}",
        first_eq,
        matches,
        activity,
        rhs,
        std::abs(activity - rhs));
  }
  return fmt::format(" reduced_map=invalid matches={}", matches);
}

std::string format_papilo_compressed_validation_detail(
    const PaPILOReducedValidationSummary& summary) {
  if (!summary.available) {
    return " papilo_reduced=unavailable";
  }
  if (!summary.size_ok) {
    return fmt::format(" papilo_reduced=size_mismatch expected={} got={}",
                       summary.expected_active_size,
                       summary.actual_size);
  }
  std::string out = fmt::format(
      " papilo_reduced={} rows={} cols={} active_cols={} fixed_pop={} "
      "rowviol={:.3g} row={} orig={} side={} activity={:.12g} lhs={:.12g} rhs={:.12g} "
      "nonactive_terms={} nonactive_abs={:.6g} colviol={:.3g} col={} orig_col={} value={:.12g} lb={:.12g} ub={:.12g}",
      summary.ok() ? "ok" : "fail",
      summary.compressed_rows,
      summary.compressed_cols,
      summary.expected_active_size,
      summary.fixed_cols_populated,
      summary.max_row_violation,
      summary.worst_row,
      summary.worst_orig_row,
      summary.worst_row_side,
      summary.worst_row_activity,
      summary.worst_row_lhs,
      summary.worst_row_rhs,
      summary.worst_row_nonactive_terms,
      summary.worst_row_nonactive_abs_activity,
      summary.max_col_violation,
      summary.worst_col,
      summary.worst_orig_col,
      summary.worst_col_value,
      summary.worst_col_lb,
      summary.worst_col_ub);
  if (summary.worst_row_eq) out += " rowtype=eq";
  return out;
}
#endif


}  // namespace mipsolvers::engine::detail
