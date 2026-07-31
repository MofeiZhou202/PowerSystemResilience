/// @file papilo_presolve.cpp
/// @brief PaPILO-based presolve implementation for MILP problems.

#include "mipsolvers/engine/strategy/papilo_presolve.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

namespace mipsolvers::engine {

MatrixScalingStats compute_matrix_scaling_stats(const LPModel& lp) {
  MatrixScalingStats s;
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  s.rows = m_ineq + m_eq;
  s.cols = static_cast<int>(lp.vars.size());
  s.nnz = static_cast<long long>(lp.A.nonZeros()) +
          static_cast<long long>(lp.Aeq.nonZeros());

  std::vector<double> row_min(static_cast<std::size_t>(std::max(0, s.rows)),
                              std::numeric_limits<double>::infinity());
  std::vector<double> row_max(static_cast<std::size_t>(std::max(0, s.rows)),
                              0.0);
  std::vector<int> row_nnz(static_cast<std::size_t>(std::max(0, s.rows)), 0);
  std::vector<int> col_nnz(static_cast<std::size_t>(std::max(0, s.cols)), 0);

  auto scan = [&](const Eigen::SparseMatrix<double>& M, int row_offset) {
    for (int j = 0; j < M.outerSize(); ++j) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(M, j); it; ++it) {
        const double a = std::abs(it.value());
        if (a == 0.0) continue;
        const std::size_t i =
            static_cast<std::size_t>(it.row() + row_offset);
        row_min[i] = std::min(row_min[i], a);
        row_max[i] = std::max(row_max[i], a);
        ++row_nnz[i];
        if (j >= 0 && j < s.cols) ++col_nnz[static_cast<std::size_t>(j)];
        if (s.abs_min == 0.0 || a < s.abs_min) s.abs_min = a;
        s.abs_max = std::max(s.abs_max, a);
      }
    }
  };
  scan(lp.A, 0);
  scan(lp.Aeq, m_ineq);

  for (int i = 0; i < s.rows; ++i) {
    const std::size_t iz = static_cast<std::size_t>(i);
    s.max_row_nnz = std::max(s.max_row_nnz, row_nnz[iz]);
    if (row_nnz[iz] == 0) continue;
    const double range = row_max[iz] / row_min[iz];
    if (range > 1e8) ++s.rows_with_large_range;
    if (s.worst_row < 0 || range > s.worst_row_range) {
      s.worst_row = i;
      s.worst_row_range = range;
    }
  }
  for (int c : col_nnz) s.max_col_nnz = std::max(s.max_col_nnz, c);
  if (s.abs_min > 0.0) s.dynamic_range = s.abs_max / s.abs_min;

  for (int i = 0; i < lp.b.size(); ++i) {
    s.b_abs_max = std::max(s.b_abs_max, std::abs(lp.b[i]));
    const double lhs = i < lp.row_lhs.size() ? lp.row_lhs[i]
                                             : -std::numeric_limits<double>::infinity();
    if (std::isfinite(lhs)) s.b_abs_max = std::max(s.b_abs_max, std::abs(lhs));
  }
  for (int i = 0; i < lp.beq.size(); ++i) {
    s.b_abs_max = std::max(s.b_abs_max, std::abs(lp.beq[i]));
  }
  for (int j = 0; j < lp.c.size(); ++j) {
    s.c_abs_max = std::max(s.c_abs_max, std::abs(lp.c[j]));
  }
  return s;
}

namespace {

[[maybe_unused]] bool presolve_scaling_diag_enabled() {
  const char* e = std::getenv("MIPSOLVERS_PRESOLVE_SCALING_DIAG");
  return e != nullptr && e[0] != '\0' && e[0] != '0';
}

[[maybe_unused]] void print_matrix_scaling_stats(const char* stage,
                                                 const MatrixScalingStats& s) {
  fprintf(stderr,
          "[PRESOLVE-SCALING] stage=%s rows=%d cols=%d nnz=%lld "
          "|A|min=%.3e |A|max=%.3e dyn=%.3e max_row_nnz=%d max_col_nnz=%d "
          "rows_range_gt_1e8=%d worst_row=%d worst_range=%.3e "
          "|b|max=%.3e |c|max=%.3e\n",
          stage, s.rows, s.cols, s.nnz, s.abs_min, s.abs_max, s.dynamic_range,
          s.max_row_nnz, s.max_col_nnz, s.rows_with_large_range, s.worst_row,
          s.worst_row_range, s.b_abs_max, s.c_abs_max);
}

}  // namespace

}  // namespace mipsolvers::engine

#ifdef HACDCPF_HAVE_PAPILO

#include <papilo/core/Problem.hpp>
#include <papilo/core/ProblemBuilder.hpp>
#include <papilo/core/Presolve.hpp>
#include <papilo/core/postsolve/Postsolve.hpp>
#include <papilo/core/postsolve/PostsolveStorage.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <unordered_map>

namespace mipsolvers::engine {

static constexpr double kInf = 1e20;

/// Opaque storage for PaPILO postsolve data.
struct PaPILOPostsolveData {
  papilo::PostsolveStorage<double> storage;
  papilo::Num<double> num;
  papilo::Problem<double> reduced_problem;
  int orig_ncols{0};
  int orig_nrows{0};
};

namespace {

Eigen::VectorXd build_compressed_primal(const PaPILOPresolveResult& ps,
                                        const PaPILOPostsolveData& psd,
                                        const Eigen::VectorXd& x_reduced,
                                        int* fixed_cols_populated = nullptr) {
  const int n_orig = psd.orig_ncols;
  const int n_compressed =
      static_cast<int>(psd.storage.origcol_mapping.size());
  Eigen::VectorXd x_compressed = Eigen::VectorXd::Zero(n_compressed);

  std::vector<int> orig_to_compressed(n_orig, -1);
  for (int c = 0; c < n_compressed; ++c) {
    int oc = psd.storage.origcol_mapping[c];
    if (oc >= 0 && oc < n_orig)
      orig_to_compressed[static_cast<std::size_t>(oc)] = c;
  }

  for (int j = 0; j < static_cast<int>(ps.reduced_to_orig_col.size()); ++j) {
    int orig_j = ps.reduced_to_orig_col[static_cast<std::size_t>(j)];
    if (orig_j < 0 || orig_j >= n_orig || j >= static_cast<int>(x_reduced.size()))
      continue;
    int comp_j = orig_to_compressed[static_cast<std::size_t>(orig_j)];
    if (comp_j >= 0 && comp_j < n_compressed)
      x_compressed[comp_j] = x_reduced[j];
  }

  int populated = 0;
  if (psd.reduced_problem.getNCols() == n_compressed) {
    const auto& flags = psd.reduced_problem.getColFlags();
    const auto& lbs = psd.reduced_problem.getLowerBounds();
    const auto& ubs = psd.reduced_problem.getUpperBounds();
    for (int c = 0; c < n_compressed; ++c) {
      const bool fixed =
          flags[c].test(papilo::ColFlag::kFixed) ||
          (!flags[c].test(papilo::ColFlag::kLbInf) &&
           !flags[c].test(papilo::ColFlag::kUbInf) &&
           std::abs(lbs[c] - ubs[c]) <= 1e-9);
      if (!fixed) continue;

      double value = 0.0;
      if (!flags[c].test(papilo::ColFlag::kLbInf)) {
        value = lbs[c];
      } else if (!flags[c].test(papilo::ColFlag::kUbInf)) {
        value = ubs[c];
      }
      x_compressed[c] = value;
      ++populated;
    }
  }
  if (fixed_cols_populated) *fixed_cols_populated = populated;
  return x_compressed;
}

}  // namespace

PaPILOPresolveResult papilo_presolve_mip(const LPModel& lp, bool verbose,
                                          PaPILOProfile profile) {
  PaPILOPresolveResult result;
  auto t0 = std::chrono::steady_clock::now();

  const int ncols = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int nrows = m_ineq + m_eq;

  result.orig_cols = ncols;
  result.orig_rows = nrows;
  result.orig_nnz = static_cast<int>(lp.A.nonZeros() + lp.Aeq.nonZeros());

  const bool scaling_diag = verbose || presolve_scaling_diag_enabled();
  result.before_stats = compute_matrix_scaling_stats(lp);
  if (scaling_diag) {
    print_matrix_scaling_stats("before", result.before_stats);
  }

  if (ncols == 0 || nrows == 0) {
    result.reduced_lp = lp;
    result.success = true;
    return result;
  }

  // ── Build PaPILO problem ──
  papilo::ProblemBuilder<double> pb;
  pb.setNumCols(ncols);
  pb.setNumRows(nrows);

  // Objective — PaPILO always minimizes
  bool maximize = (lp.sense == Sense::Maximize);
  for (int j = 0; j < ncols; ++j) {
    pb.setObj(j, maximize ? -lp.c[j] : lp.c[j]);
  }

  // Variable bounds + integrality
  for (int j = 0; j < ncols; ++j) {
    const auto& v = lp.vars[j];
    if (v.lb <= -kInf) pb.setColLbInf(j, true);
    else                pb.setColLb(j, v.lb);

    if (v.ub >= kInf) pb.setColUbInf(j, true);
    else               pb.setColUb(j, v.ub);

    if (v.type == VarType::Binary || v.type == VarType::Integer)
      pb.setColIntegral(j, true);
  }

  // Inequality rows: row_lhs[i] <= A[i,:]*x <= b[i].  An empty row_lhs
  // preserves the historical A*x <= b convention.
  for (int i = 0; i < m_ineq; ++i) {
    const double lhs = lp_row_lhs_or_neg_inf(lp, i);
    if (std::isfinite(lhs)) pb.setRowLhs(i, lhs);
    else pb.setRowLhsInf(i, true);
    pb.setRowRhs(i, lp.b[i]);
  }
  // Add A entries column-wise (Eigen column-major)
  for (int j = 0; j < ncols; ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      const double value = it.value();
      if (value == 0.0) continue;
      pb.addEntry(static_cast<int>(it.row()), j, value);
    }
  }

  // Equality rows: beq[i] <= Aeq[i,:]*x <= beq[i]
  for (int i = 0; i < m_eq; ++i) {
    int r = m_ineq + i;
    pb.setRowLhs(r, lp.beq[i]);
    pb.setRowRhs(r, lp.beq[i]);
  }
  for (int j = 0; j < ncols; ++j) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      const double value = it.value();
      if (value == 0.0) continue;
      pb.addEntry(m_ineq + static_cast<int>(it.row()), j, value);
    }
  }

  auto problem = pb.build();

  // ── Run PaPILO presolve ──
  papilo::Presolve<double> presolve;
  presolve.addDefaultPresolvers();
  if (!verbose)
    presolve.setVerbosityLevel(papilo::VerbosityLevel::kQuiet);

  if (profile == PaPILOProfile::kAggressive) {
    auto& opts = presolve.getPresolveOptions();
    // Drive PaPILO past the default early-exit thresholds.  These values
    // mirror the "aggressive" profile in presolve_structure_diag: they
    // force further rounds whenever any reduction fires, permit larger
    // fill-in during substitution, and enable full linear-dependency
    // detection.  Leaves dualreds at its default (2).
    opts.abortfac = 1e-6;
    opts.abortfacmedium = 1e-6;
    opts.abortfacfast = 1e-6;
    opts.maxfillinpersubstitution = 30;
    opts.maxshiftperrow = 30;
    opts.detectlindep = 2;
    opts.max_consecutive_rounds_of_only_bound_changes = 2000;
  }

  auto ps_result = presolve.apply(problem);

  if (verbose) {
    const auto& st = presolve.getStatistics();
    fprintf(stderr,
      "[PAPILO:%s] rounds=%d  bnd_chg=%d side_chg=%d coef_chg=%d  "
      "tsx=%d conflicts=%d  deleted_rows=%d deleted_cols=%d  time=%.3fs\n",
      profile == PaPILOProfile::kAggressive ? "aggressive" : "default",
      st.nrounds, st.nboundchgs, st.nsidechgs, st.ncoefchgs,
      st.ntsxapplied, st.ntsxconflicts, st.ndeletedrows, st.ndeletedcols,
      st.presolvetime);
  }

  if (ps_result.status == papilo::PresolveStatus::kInfeasible) {
    result.infeasible = true;
    result.success = false;
    if (verbose)
      fprintf(stderr, "[PAPILO] Problem detected as infeasible during presolve\n");
    return result;
  }

  // Store postsolve data for later solution recovery
  auto psd = std::make_shared<PaPILOPostsolveData>();
  psd->storage = std::move(ps_result.postsolve);
  psd->orig_ncols = ncols;
  psd->orig_nrows = nrows;
  psd->reduced_problem = problem;
  result.postsolve_data = psd;

  // ── Extract reduced problem ──
  const int red_ncols = problem.getNCols();
  const int red_nrows = problem.getNRows();
  const auto& matrix = problem.getConstraintMatrix();
  const auto& obj = problem.getObjective();
  result.objective_offset = obj.offset;
  const auto& col_flags = problem.getColFlags();
  const auto& row_flags = problem.getRowFlags();
  const auto& lbs = problem.getLowerBounds();
  const auto& ubs = problem.getUpperBounds();
  const auto& lhs_vals = matrix.getLeftHandSides();
  const auto& rhs_vals = matrix.getRightHandSides();

  // Active (non-eliminated) columns
  std::vector<int> active_cols;
  std::vector<int> papilo_to_active(red_ncols, -1);
  for (int j = 0; j < red_ncols; ++j) {
    if (!col_flags[j].test(papilo::ColFlag::kFixed) &&
        !col_flags[j].test(papilo::ColFlag::kSubstituted)) {
      papilo_to_active[j] = static_cast<int>(active_cols.size());
      active_cols.push_back(j);
    }
  }

  // Active (non-redundant) rows
  std::vector<int> active_rows;
  std::vector<bool> row_is_eq;
  for (int i = 0; i < red_nrows; ++i) {
    if (row_flags[i].test(papilo::RowFlag::kRedundant)) continue;
    active_rows.push_back(i);
    row_is_eq.push_back(row_flags[i].test(papilo::RowFlag::kEquation));
  }

  const int n_active = static_cast<int>(active_cols.size());
  const int m_active = static_cast<int>(active_rows.size());
  result.reduced_compact_rows = m_active;

  // Separate into inequality and equality
  std::vector<int> ineq_rows_idx, eq_rows_idx;
  for (int k = 0; k < m_active; ++k)
    (row_is_eq[k] ? eq_rows_idx : ineq_rows_idx).push_back(k);

  // Build reduced LPModel
  LPModel& rlp = result.reduced_lp;
  rlp.sense = lp.sense;
  rlp.vars.resize(n_active);
  rlp.c.resize(n_active);

  for (int j = 0; j < n_active; ++j) {
    int pj = active_cols[j];
    rlp.c[j] = maximize ? -obj.coefficients[pj] : obj.coefficients[pj];

    auto& v = rlp.vars[j];
    v.lb = col_flags[pj].test(papilo::ColFlag::kLbInf) ? -kInf : lbs[pj];
    v.ub = col_flags[pj].test(papilo::ColFlag::kUbInf) ?  kInf : ubs[pj];

    if (col_flags[pj].test(papilo::ColFlag::kIntegral)) {
      if (v.lb >= -1e-9 && v.lb <= 1e-9 && v.ub >= 1.0 - 1e-9 && v.ub <= 1.0 + 1e-9) {
        v.type = VarType::Binary;
        v.lb = 0.0; v.ub = 1.0;
        result.binary_idx.push_back(j);
      } else {
        v.type = VarType::Integer;
        result.integer_idx.push_back(j);
      }
    } else {
      v.type = VarType::Continuous;
    }
  }

  // Inequality rows: keep finite two-sided rows compact as ranged rows.
  // Lower-only rows are still represented by negating them into a finite
  // upper side so every native inequality has a finite `b`.
  struct IneqSpec {
    int active_row_idx{-1};
    bool negate{false};
    double lhs{-std::numeric_limits<double>::infinity()};
    double rhs{kInf};
  };
  std::vector<IneqSpec> ineq_specs;
  for (int k : ineq_rows_idx) {
    int pi = active_rows[k];
    const bool has_rhs =
        !row_flags[pi].test(papilo::RowFlag::kRhsInf) &&
        rhs_vals[pi] < 0.5 * kInf;
    const bool has_lhs =
        !row_flags[pi].test(papilo::RowFlag::kLhsInf) &&
        lhs_vals[pi] > -0.5 * kInf;
    if (has_lhs && has_rhs) {
      ineq_specs.push_back(IneqSpec{k, false, lhs_vals[pi], rhs_vals[pi]});
      ++result.reduced_two_sided_rows_expanded;
    } else if (has_rhs) {
      ineq_specs.push_back(IneqSpec{
          k, false, -std::numeric_limits<double>::infinity(), rhs_vals[pi]});
    } else if (has_lhs) {
      ineq_specs.push_back(IneqSpec{
          k, true, -std::numeric_limits<double>::infinity(), -lhs_vals[pi]});
    }
  }

  const int total_ineq = static_cast<int>(ineq_specs.size());
  const int total_eq = static_cast<int>(eq_rows_idx.size());

  // Sparse inequality matrix
  std::vector<Eigen::Triplet<double>> trips;
  rlp.b.resize(total_ineq);
  rlp.row_lhs.resize(total_ineq);
  rlp.row_lhs.setConstant(-std::numeric_limits<double>::infinity());
  for (int r = 0; r < total_ineq; ++r) {
    const auto& spec = ineq_specs[static_cast<std::size_t>(r)];
    const int ak = spec.active_row_idx;
    const bool neg = spec.negate;
    int pi = active_rows[ak];
    auto rc = matrix.getRowCoefficients(pi);
    const double* vals = rc.getValues();
    const int* inds = rc.getIndices();
    int len = rc.getLength();
    rlp.b[r] = spec.rhs;
    rlp.row_lhs[r] = spec.lhs;
    if (std::isfinite(spec.lhs)) ++result.reduced_rows_with_finite_lhs;
    double sign = neg ? -1.0 : 1.0;
    for (int e = 0; e < len; ++e) {
      int ac = papilo_to_active[inds[e]];
      if (ac >= 0) trips.emplace_back(r, ac, sign * vals[e]);
    }
  }
  rlp.A.resize(total_ineq, n_active);
  rlp.A.setFromTriplets(trips.begin(), trips.end());

  // Sparse equality matrix
  trips.clear();
  rlp.beq.resize(total_eq);
  for (int r = 0; r < total_eq; ++r) {
    int pi = active_rows[eq_rows_idx[r]];
    rlp.beq[r] = rhs_vals[pi];
    auto rc = matrix.getRowCoefficients(pi);
    const double* vals = rc.getValues();
    const int* inds = rc.getIndices();
    int len = rc.getLength();
    for (int e = 0; e < len; ++e) {
      int ac = papilo_to_active[inds[e]];
      if (ac >= 0) trips.emplace_back(r, ac, vals[e]);
    }
  }
  rlp.Aeq.resize(total_eq, n_active);
  rlp.Aeq.setFromTriplets(trips.begin(), trips.end());

  // Column mappings: use origcol_mapping to convert compressed→original.
  // PaPILO compresses the problem during apply(), so active_cols[j] is a
  // compressed index, NOT the original column index.
  const auto& ocm = psd->storage.origcol_mapping;
  const auto& orm = psd->storage.origrow_mapping;
  result.reduced_to_orig_col.resize(n_active);
  result.orig_to_reduced_col.assign(ncols, -1);
  for (int j = 0; j < n_active; ++j) {
    int compressed_j = active_cols[j];
    int orig_j = (compressed_j >= 0 && compressed_j < static_cast<int>(ocm.size()))
                 ? ocm[compressed_j] : compressed_j;
    result.reduced_to_orig_col[j] = orig_j;
    if (orig_j >= 0 && orig_j < ncols)
      result.orig_to_reduced_col[orig_j] = j;
  }

  result.reduced_ineq_to_orig_row.assign(total_ineq, -1);
  result.reduced_ineq_negated.assign(total_ineq, 0);
  for (int r = 0; r < total_ineq; ++r) {
    const auto& spec = ineq_specs[static_cast<std::size_t>(r)];
    const int pi = active_rows[spec.active_row_idx];
    result.reduced_ineq_to_orig_row[static_cast<std::size_t>(r)] =
        (pi >= 0 && pi < static_cast<int>(orm.size())) ? orm[pi] : pi;
    result.reduced_ineq_negated[static_cast<std::size_t>(r)] =
        spec.negate ? 1 : 0;
  }
  result.reduced_eq_to_orig_row.assign(total_eq, -1);
  for (int r = 0; r < total_eq; ++r) {
    const int pi = active_rows[eq_rows_idx[static_cast<std::size_t>(r)]];
    result.reduced_eq_to_orig_row[static_cast<std::size_t>(r)] =
        (pi >= 0 && pi < static_cast<int>(orm.size())) ? orm[pi] : pi;
  }

  result.reduced_rows = total_ineq + total_eq;
  result.reduced_cols = n_active;
  result.reduced_nnz = static_cast<int>(rlp.A.nonZeros() + rlp.Aeq.nonZeros());
  result.success = true;

  // Populate aggregated bound/coeff change counts for auto-aggressive logic.
  {
    const auto& st = presolve.getStatistics();
    result.total_bnd_chg = static_cast<int>(st.nboundchgs);
    result.total_coef_chg = static_cast<int>(st.ncoefchgs);
  }

  auto t1 = std::chrono::steady_clock::now();
  result.presolve_time_sec = std::chrono::duration<double>(t1 - t0).count();

  if (result.reduced_cols > 0) {
    result.after_stats = compute_matrix_scaling_stats(result.reduced_lp);
    if (scaling_diag) {
      print_matrix_scaling_stats(
          profile == PaPILOProfile::kAggressive ? "after-aggressive" : "after",
          result.after_stats);
    }
  }

  if (verbose) {
    fprintf(stderr,
      "[PAPILO] Presolve %.1fms: rows %d→%d(native compact=%d +two-sided=%d) "
      "cols %d→%d nnz %d→%d\n",
      result.presolve_time_sec * 1000.0,
      result.orig_rows, result.reduced_rows, result.reduced_compact_rows,
      result.reduced_two_sided_rows_expanded,
      result.orig_cols, result.reduced_cols,
      result.orig_nnz, result.reduced_nnz);
  }

  return result;
}

Eigen::VectorXd papilo_postsolve(const PaPILOPresolveResult& ps,
                                  const Eigen::VectorXd& x_reduced) {
  if (!ps.postsolve_data) {
    // No PaPILO postsolve data — fallback to simple mapping
    const int n_orig = ps.orig_cols;
    Eigen::VectorXd x_orig = Eigen::VectorXd::Zero(n_orig);
    for (int j = 0; j < static_cast<int>(ps.reduced_to_orig_col.size()); ++j) {
      int oj = ps.reduced_to_orig_col[j];
      if (oj >= 0 && oj < n_orig && j < x_reduced.size())
        x_orig[oj] = x_reduced[j];
    }
    return x_orig;
  }

  const auto& psd = *ps.postsolve_data;

  // Build a PaPILO Solution in the reduced space.
  // We need to expand x_reduced (which has only active columns) into the
  // full reduced-problem column vector (including fixed/substituted entries).
  // PaPILO compresses columns: origcol_mapping maps compressed→original.
  // We need to place values at compressed indices.
  const int n_orig = psd.orig_ncols;
  const Eigen::VectorXd x_compressed = build_compressed_primal(ps, psd, x_reduced);

  papilo::Solution<double> reduced_sol(papilo::SolutionType::kPrimal);
  reduced_sol.primal.resize(static_cast<std::size_t>(x_compressed.size()), 0.0);
  for (int j = 0; j < x_compressed.size(); ++j)
    reduced_sol.primal[static_cast<std::size_t>(j)] = x_compressed[j];

  // Call PaPILO's postsolve
  papilo::Solution<double> original_sol(papilo::SolutionType::kPrimal);
  original_sol.primal.resize(n_orig, 0.0);

  papilo::Message msg{};
  msg.setVerbosityLevel(papilo::VerbosityLevel::kQuiet);
  papilo::Postsolve<double> postsolve(msg, psd.num);
  postsolve.undo(reduced_sol, original_sol, psd.storage);

  // Convert to Eigen
  Eigen::VectorXd x_orig(n_orig);
  for (int j = 0; j < n_orig; ++j)
    x_orig[j] = original_sol.primal[j];

  return x_orig;
}

PaPILOReducedValidationSummary papilo_validate_reduced_solution(
    const PaPILOPresolveResult& ps,
    const Eigen::VectorXd& x_reduced) {
  PaPILOReducedValidationSummary out;
  out.expected_active_size = static_cast<int>(ps.reduced_to_orig_col.size());
  out.actual_size = static_cast<int>(x_reduced.size());
  out.size_ok = out.actual_size == out.expected_active_size;
  if (!ps.postsolve_data) {
    return out;
  }

  const auto& psd = *ps.postsolve_data;
  const auto& problem = psd.reduced_problem;
  out.available = true;
  out.compressed_cols = problem.getNCols();
  out.compressed_rows = problem.getNRows();
  if (out.compressed_cols <= 0 && out.compressed_rows <= 0) {
    return out;
  }

  int fixed_populated = 0;
  const Eigen::VectorXd x_compressed =
      build_compressed_primal(ps, psd, x_reduced, &fixed_populated);
  out.fixed_cols_populated = fixed_populated;
  if (x_compressed.size() != out.compressed_cols) {
    out.size_ok = false;
    return out;
  }

  const auto& col_flags = problem.getColFlags();
  const auto& lbs = problem.getLowerBounds();
  const auto& ubs = problem.getUpperBounds();
  const auto& ocm = psd.storage.origcol_mapping;
  for (int j = 0; j < out.compressed_cols; ++j) {
    const bool has_lb = !col_flags[j].test(papilo::ColFlag::kLbInf);
    const bool has_ub = !col_flags[j].test(papilo::ColFlag::kUbInf);
    const double lb = has_lb ? lbs[j] : -std::numeric_limits<double>::infinity();
    const double ub = has_ub ? ubs[j] : std::numeric_limits<double>::infinity();
    const double viol = std::max(
        has_lb ? std::max(0.0, lb - x_compressed[j]) : 0.0,
        has_ub ? std::max(0.0, x_compressed[j] - ub) : 0.0);
    if (viol > out.max_col_violation) {
      out.max_col_violation = viol;
      out.worst_col = j;
      out.worst_orig_col =
          (j >= 0 && j < static_cast<int>(ocm.size())) ? ocm[j] : -1;
      out.worst_col_value = x_compressed[j];
      out.worst_col_lb = lb;
      out.worst_col_ub = ub;
    }
  }

  std::vector<char> active_compressed(static_cast<std::size_t>(out.compressed_cols), 0);
  {
    std::vector<int> orig_to_compressed(psd.orig_ncols, -1);
    for (int c = 0; c < static_cast<int>(ocm.size()); ++c) {
      int oc = ocm[c];
      if (oc >= 0 && oc < psd.orig_ncols)
        orig_to_compressed[static_cast<std::size_t>(oc)] = c;
    }
    for (int oj : ps.reduced_to_orig_col) {
      if (oj < 0 || oj >= psd.orig_ncols) continue;
      int cj = orig_to_compressed[static_cast<std::size_t>(oj)];
      if (cj >= 0 && cj < out.compressed_cols)
        active_compressed[static_cast<std::size_t>(cj)] = 1;
    }
  }

  const auto& matrix = problem.getConstraintMatrix();
  const auto& row_flags = problem.getRowFlags();
  const auto& lhs_vals = matrix.getLeftHandSides();
  const auto& rhs_vals = matrix.getRightHandSides();
  const auto& orm = psd.storage.origrow_mapping;
  for (int r = 0; r < out.compressed_rows; ++r) {
    if (row_flags[r].test(papilo::RowFlag::kRedundant)) continue;
    auto row = matrix.getRowCoefficients(r);
    const double* vals = row.getValues();
    const int* inds = row.getIndices();
    const int len = row.getLength();
    double activity = 0.0;
    int nonactive_terms = 0;
    double nonactive_abs_activity = 0.0;
    for (int e = 0; e < len; ++e) {
      const int j = inds[e];
      if (j < 0 || j >= x_compressed.size()) continue;
      const double ax = vals[e] * x_compressed[j];
      activity += ax;
      if (j >= 0 && j < out.compressed_cols &&
          !active_compressed[static_cast<std::size_t>(j)]) {
        ++nonactive_terms;
        nonactive_abs_activity += std::abs(ax);
      }
    }

    const bool has_lhs =
        !row_flags[r].test(papilo::RowFlag::kLhsInf) &&
        lhs_vals[r] > -0.5 * kInf;
    const bool has_rhs =
        !row_flags[r].test(papilo::RowFlag::kRhsInf) &&
        rhs_vals[r] < 0.5 * kInf;
    const double lhs = has_lhs ? lhs_vals[r] : -std::numeric_limits<double>::infinity();
    const double rhs = has_rhs ? rhs_vals[r] : std::numeric_limits<double>::infinity();
    const double lower_viol = has_lhs ? std::max(0.0, lhs - activity) : 0.0;
    const double upper_viol = has_rhs ? std::max(0.0, activity - rhs) : 0.0;
    const double viol = std::max(lower_viol, upper_viol);
    if (viol > out.max_row_violation) {
      out.max_row_violation = viol;
      out.worst_row = r;
      out.worst_orig_row =
          (r >= 0 && r < static_cast<int>(orm.size())) ? orm[r] : -1;
      out.worst_row_eq = row_flags[r].test(papilo::RowFlag::kEquation);
      out.worst_row_side = (upper_viol >= lower_viol) ? "upper" : "lower";
      out.worst_row_activity = activity;
      out.worst_row_lhs = lhs;
      out.worst_row_rhs = rhs;
      out.worst_row_nonactive_terms = nonactive_terms;
      out.worst_row_nonactive_abs_activity = nonactive_abs_activity;
    }
  }

  constexpr double tol = 1e-6;
  out.col_bounds_ok = out.max_col_violation <= tol;
  out.rows_ok = out.max_row_violation <= tol;
  return out;
}

Eigen::VectorXd papilo_forward_map(const PaPILOPresolveResult& ps,
                                    const Eigen::VectorXd& x_orig) {
  const int n_red = ps.reduced_cols;
  const int n_orig = static_cast<int>(x_orig.size());
  Eigen::VectorXd x_red(n_red);

  for (int j = 0; j < n_red; ++j) {
    int oj = ps.reduced_to_orig_col[j];
    x_red[j] = (oj >= 0 && oj < n_orig) ? x_orig[oj] : 0.0;
  }
  return x_red;
}

std::vector<PaPILOAffineExpression> papilo_export_affine_expressions(
    const PaPILOPresolveResult& ps,
    int max_terms,
    double tol) {
  std::vector<PaPILOAffineExpression> out(
      static_cast<std::size_t>(std::max(0, ps.orig_cols)));
  for (int j = 0; j < static_cast<int>(out.size()); ++j) {
    out[static_cast<std::size_t>(j)].original_col = j;
  }
  if (!ps.postsolve_data || ps.orig_cols <= 0) {
    return out;
  }

  const auto& psd = *ps.postsolve_data;
  const auto& storage = psd.storage;
  if (storage.nColsOriginal == 0) return out;

  std::unordered_map<int, int> orig_to_active;
  orig_to_active.reserve(ps.reduced_to_orig_col.size() * 2 + 1);
  for (int j = 0; j < static_cast<int>(ps.reduced_to_orig_col.size()); ++j) {
    const int orig = ps.reduced_to_orig_col[static_cast<std::size_t>(j)];
    if (orig >= 0 && orig < ps.orig_cols) orig_to_active.emplace(orig, j);
  }

  struct Expr {
    bool available{false};
    bool exact{false};
    bool too_dense{false};
    bool unsupported{false};
    double constant{0.0};
    std::unordered_map<int, double> terms;
  };

  std::vector<Expr> exprs(static_cast<std::size_t>(ps.orig_cols));
  for (const auto& [orig, active] : orig_to_active) {
    Expr& e = exprs[static_cast<std::size_t>(orig)];
    e.available = true;
    e.exact = true;
    e.terms[active] = 1.0;
  }

  auto mark_unsupported = [&](int col) {
    if (col < 0 || col >= ps.orig_cols) return;
    Expr& e = exprs[static_cast<std::size_t>(col)];
    e.available = true;
    e.exact = false;
    e.unsupported = true;
    e.terms.clear();
    e.constant = 0.0;
  };

  auto add_scaled_expr = [&](Expr& dst, const Expr& src, double scale) {
    dst.constant += scale * src.constant;
    for (const auto& [col, coef] : src.terms) {
      const double v = dst.terms[col] + scale * coef;
      if (std::abs(v) <= tol) {
        dst.terms.erase(col);
      } else {
        dst.terms[col] = v;
      }
    }
    if (static_cast<int>(dst.terms.size()) > max_terms) dst.too_dense = true;
  };

  auto build_from_row = [&](int col, double lhs, int first_term, int last_term,
                            const std::vector<int>& indices,
                            const std::vector<double>& values) {
    if (col < 0 || col >= ps.orig_cols) return;
    double col_coef = 0.0;
    Expr rebuilt;
    rebuilt.available = true;
    rebuilt.exact = true;
    rebuilt.constant = lhs;
    for (int p = first_term; p < last_term; ++p) {
      const int term_col = indices[static_cast<std::size_t>(p)];
      const double coef = values[static_cast<std::size_t>(p)];
      if (term_col == col) {
        col_coef = coef;
        continue;
      }
      if (term_col < 0 || term_col >= ps.orig_cols) {
        rebuilt.exact = false;
        rebuilt.unsupported = true;
        continue;
      }
      const Expr& term_expr = exprs[static_cast<std::size_t>(term_col)];
      if (!term_expr.available || !term_expr.exact) {
        rebuilt.exact = false;
        rebuilt.unsupported = true;
        continue;
      }
      add_scaled_expr(rebuilt, term_expr, -coef);
    }
    if (std::abs(col_coef) <= tol || !rebuilt.exact || rebuilt.too_dense) {
      rebuilt.exact = false;
      rebuilt.unsupported = rebuilt.unsupported || std::abs(col_coef) <= tol;
      rebuilt.terms.clear();
      rebuilt.constant = 0.0;
    } else {
      rebuilt.constant /= col_coef;
      for (auto& [_, coef] : rebuilt.terms) coef /= col_coef;
    }
    exprs[static_cast<std::size_t>(col)] = std::move(rebuilt);
  };

  const auto& types = storage.types;
  const auto& start = storage.start;
  const auto& indices = storage.indices;
  const auto& values = storage.values;
  for (int i = static_cast<int>(types.size()) - 1; i >= 0; --i) {
    const int first = start[static_cast<std::size_t>(i)];
    const int last = start[static_cast<std::size_t>(i + 1)];
    switch (types[static_cast<std::size_t>(i)]) {
      case ReductionType::kFixedCol: {
        if (first >= 0 && first < last) {
          const int col = indices[static_cast<std::size_t>(first)];
          if (col >= 0 && col < ps.orig_cols) {
            Expr e;
            e.available = true;
            e.exact = true;
            e.constant = values[static_cast<std::size_t>(first)];
            exprs[static_cast<std::size_t>(col)] = std::move(e);
          }
        }
        break;
      }
      case ReductionType::kFixedInfCol: {
        if (first >= 0 && first < last) {
          mark_unsupported(indices[static_cast<std::size_t>(first)]);
        }
        break;
      }
      case ReductionType::kSubstitutedCol: {
        if (first >= 0 && first < last) {
          const int col = indices[static_cast<std::size_t>(first)];
          const double lhs = values[static_cast<std::size_t>(first)];
          build_from_row(col, lhs, first + 1, last, indices, values);
        }
        break;
      }
      case ReductionType::kSubstitutedColWithDual: {
        if (first + 3 <= last) {
          const int row_length =
              static_cast<int>(values[static_cast<std::size_t>(first)]);
          const int row_terms_first = first + 3;
          const int row_terms_last = row_terms_first + row_length;
          const int col_slot = row_terms_last;
          if (row_terms_last <= last && col_slot < last) {
            const int col = indices[static_cast<std::size_t>(col_slot)];
            const double lhs = values[static_cast<std::size_t>(first + 1)];
            build_from_row(col, lhs, row_terms_first, row_terms_last, indices,
                           values);
          }
        }
        break;
      }
      case ReductionType::kParallelCol: {
        if (last - first >= 5) {
          mark_unsupported(indices[static_cast<std::size_t>(first)]);
          mark_unsupported(indices[static_cast<std::size_t>(first + 2)]);
        }
        break;
      }
      default:
        break;
    }
  }

  for (int j = 0; j < ps.orig_cols; ++j) {
    const Expr& e = exprs[static_cast<std::size_t>(j)];
    PaPILOAffineExpression& dst = out[static_cast<std::size_t>(j)];
    dst.available = e.available;
    dst.exact = e.exact && !e.too_dense && !e.unsupported;
    dst.too_dense = e.too_dense;
    dst.unsupported = e.unsupported;
    dst.constant = dst.exact ? e.constant : 0.0;
    if (!dst.exact) continue;
    dst.terms.reserve(e.terms.size());
    for (const auto& [col, coef] : e.terms) {
      if (std::abs(coef) > tol) dst.terms.push_back(PaPILOAffineTerm{col, coef});
    }
    std::sort(dst.terms.begin(), dst.terms.end(),
              [](const PaPILOAffineTerm& a, const PaPILOAffineTerm& b) {
                return a.col < b.col;
              });
  }
  return out;
}

}  // namespace mipsolvers::engine

#else  // !HACDCPF_HAVE_PAPILO

namespace mipsolvers::engine {

struct PaPILOPostsolveData {};

PaPILOPresolveResult papilo_presolve_mip(const LPModel& lp, bool /*verbose*/) {
  PaPILOPresolveResult result;
  result.reduced_lp = lp;
  result.orig_rows = static_cast<int>(lp.A.rows() + lp.Aeq.rows());
  result.orig_cols = static_cast<int>(lp.vars.size());
  result.orig_nnz = static_cast<int>(lp.A.nonZeros() + lp.Aeq.nonZeros());
  result.reduced_rows = result.orig_rows;
  result.reduced_compact_rows = result.orig_rows;
  result.reduced_cols = result.orig_cols;
  result.reduced_nnz = result.orig_nnz;
  if (lp_has_row_lhs(lp)) {
    for (int i = 0; i < lp.row_lhs.size(); ++i) {
      if (std::isfinite(lp.row_lhs[i])) ++result.reduced_rows_with_finite_lhs;
    }
  }
  result.success = false;
  return result;
}

Eigen::VectorXd papilo_postsolve(const PaPILOPresolveResult& /*ps*/,
                                  const Eigen::VectorXd& x_reduced) {
  return x_reduced;
}

PaPILOReducedValidationSummary papilo_validate_reduced_solution(
    const PaPILOPresolveResult& ps,
    const Eigen::VectorXd& x_reduced) {
  PaPILOReducedValidationSummary out;
  out.expected_active_size = static_cast<int>(ps.reduced_to_orig_col.size());
  out.actual_size = static_cast<int>(x_reduced.size());
  out.size_ok = out.actual_size == out.expected_active_size;
  return out;
}

Eigen::VectorXd papilo_forward_map(const PaPILOPresolveResult& /*ps*/,
                                    const Eigen::VectorXd& x_orig) {
  return x_orig;
}

std::vector<PaPILOAffineExpression> papilo_export_affine_expressions(
    const PaPILOPresolveResult& ps,
    int /*max_terms*/,
    double /*tol*/) {
  std::vector<PaPILOAffineExpression> out(
      static_cast<std::size_t>(std::max(0, ps.orig_cols)));
  for (int j = 0; j < static_cast<int>(out.size()); ++j) {
    out[static_cast<std::size_t>(j)].original_col = j;
  }
  return out;
}

}  // namespace mipsolvers::engine

#endif  // HACDCPF_HAVE_PAPILO
