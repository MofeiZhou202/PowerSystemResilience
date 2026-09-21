/// @file bc_cuts_common.hpp
/// @brief Shared cut-generation helpers (part of the bc_cuts.cpp split).
/// General GMI/MIR/scoring helpers used by both bc_cuts.cpp and the
/// transformed-tableau (xtab) translation unit. Co-located include-only header.

#pragma once

#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <fmt/format.h>

#include "../extern/pdqsort/pdqsort.h"
#include "util/HighsCDouble.h"
#include "mip/HighsGFkSolve.h"
#include "util/HighsRandom.h"
#include "util/HighsIntegers.h"

namespace mipsolvers::engine::detail {

inline double frac_part(double v) {
  const double f = v - std::floor(v);
  return (f < 1e-12 || f > 1.0 - 1e-12) ? 0.0 : f;
}

inline bool basis_tableau_cuts_admissible(const SimplexResult& simplex,
                                   const BCOptions& opt,
                                   const std::shared_ptr<BasisOps>& sbasis) {
  if (!sbasis || sbasis->kind() != BasisOpsKind::VendoredHighs) {
    return true;
  }
  if (!sbasis->bound_to_A(simplex.form.A)) {
    return false;
  }
  const double primal_tol = std::max(1e-7, opt.lp_tol * 20.0);
  const double dual_tol = std::max(1e-7, opt.lp_tol * 20.0);
  if (simplex.result.stats.residual_inf > primal_tol ||
      simplex.result.stats.primal_feas > primal_tol ||
      simplex.result.stats.dual_feas > dual_tol) {
    return false;
  }
  return true;
}

inline bool transformed_original_var_is_integer(const SimplexResult& simplex, int col) {
  if (col < 0 || col >= simplex.form.n_original) {
    return false;
  }
  const VarType t = simplex.form.original_types[static_cast<size_t>(col)];
  if (!(t == VarType::Integer || t == VarType::Binary)) {
    return false;
  }
  return is_integral(simplex.form.lb_shift[col], 1e-9);
}

inline double original_space_value(const SimplexResult& simplex,
                            int transformed_col,
                            double transformed_value) {
  if (transformed_col < 0) {
    return transformed_value;
  }
  if (transformed_col >= simplex.form.n_original) {
    const double col_scale =
        simplex.form.col_scale.size() > transformed_col
            ? simplex.form.col_scale[transformed_col]
            : 1.0;
    return col_scale * transformed_value;
  }
  const double col_scale =
      simplex.form.col_scale.size() > transformed_col
          ? simplex.form.col_scale[transformed_col]
          : 1.0;
  const double shift =
      simplex.form.lb_shift.size() > transformed_col
          ? simplex.form.lb_shift[transformed_col]
          : 0.0;
  return col_scale * transformed_value + shift;
}

inline int standard_form_aux_col_row(const StandardFormLP& sf, int col) {
  if (col < sf.n_original) {
    return -1;
  }
  // Fast O(1) path when the reverse map has been populated.
  if (!sf.aux_col_to_row.empty() && col < static_cast<int>(sf.aux_col_to_row.size())) {
    return sf.aux_col_to_row[static_cast<std::size_t>(col)];
  }
  // Fallback O(m) linear scan (should only trigger for legacy StandardFormLP
  // objects that were built without populating aux_col_to_row).
  const int m = static_cast<int>(sf.row_to_slack_col.size());
  for (int row = 0; row < m; ++row) {
    const std::size_t r = static_cast<std::size_t>(row);
    if (sf.row_to_slack_col[r] == col || sf.row_to_surplus_col[r] == col) {
      return row;
    }
  }
  return -1;
}

inline bool build_bounded_form_gmi_cut(const SimplexResult& simplex,
                                int row,
                                Eigen::VectorXd& cut_le,
                                double& rhs_le,
                                const std::shared_ptr<BasisOps>& sbasis,
                                const std::vector<char>* is_basic_hint = nullptr) {
  const int n_std = static_cast<int>(simplex.form.A.cols());
  const int n_orig = simplex.form.n_original;
  const int m = static_cast<int>(simplex.basis.indices.size());
  
  if (row < 0 || row >= m) {
    return false;
  }
  if (!sbasis && row >= static_cast<int>(simplex.basis_inverse.rows())) {
    return false;
  }

  const int basic_col = simplex.basis.indices[static_cast<size_t>(row)];
  if (!transformed_original_var_is_integer(simplex, basic_col)) {
    return false;
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // CRITICAL BUG FIX: Check fractionality in ORIGINAL space, not scaled space!
  // 
  // beta = x_basic[row] is in SCALED space.
  // For a scaled variable: z = (x_orig - lb_shift) / col_scale
  // So: x_orig = col_scale * z + lb_shift
  //
  // If col_scale != 1, a binary variable x_orig=1 becomes z=1/col_scale (e.g. 24.49)
  // which LOOKS fractional in scaled space but IS integer in original space!
  // GMI cuts should only be generated when the ORIGINAL value is fractional.
  // ═══════════════════════════════════════════════════════════════════════════
  const double beta_scaled = simplex.x_basic[row];
  
  // Convert to original space
  const double beta_orig =
      original_space_value(simplex, basic_col, beta_scaled);
  
  const double f0_orig = frac_part(beta_orig);
  if (f0_orig <= 1e-8 || f0_orig >= 1.0 - 1e-8) {
    // Original-space value is integral — no GMI cut needed
    return false;
  }
  
  // For GMI derivation, we still work in scaled space (tableau is scaled)
  // but use the original-space fractional part for the cut RHS
  const double f0 = f0_orig;

  Eigen::RowVectorXd tableau_row;
  if (sbasis) {
    if (!sparse_basis_tableau_row(sbasis, row, tableau_row)) {
      return false;
    }
  } else {
    const Eigen::VectorXd inverse_row =
        simplex.basis_inverse.row(row).transpose();
    tableau_row = simplex.form.A.transpose_multiply(inverse_row).transpose();
  }
  if (!tableau_row.allFinite()) {
    return false;
  }
  std::vector<char> is_basic;
  if (is_basic_hint && static_cast<int>(is_basic_hint->size()) == n_std) {
    // Reuse pre-built hint to avoid per-cut O(m) allocation + fill.
  } else {
    is_basic.assign(static_cast<size_t>(n_std), 0);
    for (int idx : simplex.basis.indices) {
      if (idx >= 0 && idx < n_std) is_basic[static_cast<size_t>(idx)] = 1;
    }
    is_basic_hint = &is_basic;
  }
  const std::vector<char>& is_basic_ref = *is_basic_hint;

  Eigen::VectorXd std_cut = Eigen::VectorXd::Zero(n_std);
  bool has_term = false;
  const double neg_scale = f0 / (1.0 - f0);
  const bool has_at_upper = !simplex.basis.at_upper.empty();
  const bool has_var_ub = simplex.form.var_ub.size() == n_std;
  double rhs_adjustment = 0.0;

  for (int j = 0; j < n_std; ++j) {
    if (is_basic_ref[static_cast<size_t>(j)]) {
      continue;
    }

    const bool j_at_upper = has_at_upper && simplex.basis.at_upper[static_cast<size_t>(j)];

    const double a_j = j_at_upper ? (-tableau_row[j]) : tableau_row[j];
    double coeff = 0.0;
    if (j < n_orig && transformed_original_var_is_integer(simplex, j)) {
      double fj = frac_part(a_j);
      if (fj <= f0) {
        coeff = fj;
      } else {
        coeff = neg_scale * (1.0 - fj);
      }
    } else {
      coeff = (a_j >= 0.0) ? a_j : (-neg_scale * a_j);
    }

    if (std::abs(coeff) <= 1e-12) {
      continue;
    }

    if (j_at_upper) {
      std_cut[j] = -coeff;
      if (has_var_ub && std::isfinite(simplex.form.var_ub[j])) {
        rhs_adjustment += coeff * simplex.form.var_ub[j];
      }
    } else {
      std_cut[j] = coeff;
    }
    has_term = true;
  }

  if (!has_term) {
    return false;
  }

  Eigen::VectorXd coeff_y = std_cut.head(n_orig);
  double rhs_ge = f0 - rhs_adjustment;

  for (int r = 0; r < static_cast<int>(simplex.form.row_to_slack_col.size()); ++r) {
    auto substitute_aux_col = [&](int aux_col) {
      if (aux_col < 0 || std::abs(std_cut[aux_col]) <= 1e-12) {
        return true;
      }
      const double aux_coeff = simplex.form.A_row.coeff(r, aux_col);
      if (!std::isfinite(aux_coeff) || std::abs(aux_coeff) <= 1e-12) {
        return false;
      }
      const double factor = std_cut[aux_col] / aux_coeff;
      for (StandardRowMatrix::InnerIterator it(simplex.form.A_row, r); it; ++it) {
        if (it.col() < n_orig) {
          coeff_y[it.col()] -= factor * it.value();
        }
      }
      rhs_ge -= factor * simplex.form.b[r];
      return true;
    };

    const int slack_col = simplex.form.row_to_slack_col[static_cast<size_t>(r)];
    if (!substitute_aux_col(slack_col)) return false;
    const int surplus_col = simplex.form.row_to_surplus_col[static_cast<size_t>(r)];
    if (!substitute_aux_col(surplus_col)) return false;

    const int artificial_col =
        simplex.form.row_to_artificial_col[static_cast<size_t>(r)];
    if (artificial_col >= 0 &&
        artificial_col < static_cast<int>(std_cut.size()) &&
        std::abs(std_cut[artificial_col]) > 1e-12) {
      return false;
    }
  }

  // ═══════════════════════════════════════════════════════════════════════════
  // Unscale GMI cut coefficients if StandardFormLP was Ruiz-scaled.
  // The tableau row and GMI derivation is in scaled space:
  //   cut_scaled' * y_scaled >= f0
  // where y_scaled[j] = (x_orig[j] - lb_shift[j]) / col_scale[j].
  // Transformed to original space:
  //   sum_j (cut_scaled[j] / col_scale[j]) * x_orig[j] >= f0 + sum_j cut_orig[j] * lb_shift[j]
  // ═══════════════════════════════════════════════════════════════════════════
  Eigen::VectorXd coeff_orig = coeff_y;
  const bool has_scaling = simplex.form.col_scale.size() >= n_orig;
  if (has_scaling) {
    for (int j = 0; j < n_orig; ++j) {
      if (simplex.form.col_scale[j] > 1e-12) {
        coeff_orig[j] = coeff_y[j] / simplex.form.col_scale[j];
      }
    }
  }

  const double rhs_x_ge = rhs_ge + coeff_orig.dot(simplex.form.lb_shift);
  cut_le = -coeff_orig;
  rhs_le = -rhs_x_ge;

  if (cut_le.norm() <= 1e-12 || !std::isfinite(rhs_le) || !cut_le.allFinite()) {
    return false;
  }
  return true;
}

inline int count_nonzeros(const Eigen::VectorXd& v, double tol = 1e-12) {
  int nnz = 0;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (std::abs(v[i]) > tol) {
      ++nnz;
    }
  }
  return nnz;
}

inline double abs_cosine_similarity(const Eigen::SparseVector<double>& a,
                             const Eigen::SparseVector<double>& b,
                             double norm_a,
                             double norm_b) {
  if (norm_a <= 1e-12 || norm_b <= 1e-12) {
    return 1.0;
  }
  const double cos = sparse_sparse_dot(a, b) / (norm_a * norm_b);
  return std::abs(cos);
}

inline Eigen::SparseVector<double> sparse_indicator_cut(
    int dimension,
    std::vector<int> indices,
    double value = 1.0) {
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  Eigen::SparseVector<double> cut(dimension);
  cut.reserve(static_cast<int>(indices.size()));
  for (int col : indices) {
    if (col >= 0 && col < dimension && std::abs(value) > 1e-15) {
      cut.insertBack(col) = value;
    }
  }
  return cut;
}

class SeparatorCandidateStorageTracker {
 public:
  explicit SeparatorCandidateStorageTracker(SeparatorStorageStats* stats)
      : stats_(stats) {}

  void record(const Eigen::SparseVector<double>& cut,
              std::size_t live_candidates) {
    if (stats_ == nullptr) return;
    const std::uint64_t entries =
        static_cast<std::uint64_t>(cut.nonZeros());
    live_entries_ += entries;
    ++stats_->sparse_candidates_created;
    stats_->sparse_candidate_entries_created += entries;
    stats_->peak_live_sparse_candidates = std::max(
        stats_->peak_live_sparse_candidates,
        static_cast<std::uint64_t>(live_candidates));
    stats_->peak_live_sparse_entries =
        std::max(stats_->peak_live_sparse_entries, live_entries_);
  }

  void record_ephemeral(const Eigen::SparseVector<double>& cut) {
    if (stats_ == nullptr) return;
    const std::uint64_t entries =
        static_cast<std::uint64_t>(cut.nonZeros());
    ++stats_->sparse_candidates_created;
    stats_->sparse_candidate_entries_created += entries;
    stats_->peak_live_sparse_candidates =
        std::max(stats_->peak_live_sparse_candidates, std::uint64_t{1});
    stats_->peak_live_sparse_entries =
        std::max(stats_->peak_live_sparse_entries, entries);
  }

 private:
  SeparatorStorageStats* stats_{nullptr};
  std::uint64_t live_entries_{0};
};

inline void record_dense_workspace(SeparatorStorageStats* stats,
                            std::size_t values) {
  if (stats == nullptr) return;
  ++stats->dense_workspace_materializations;
  stats->dense_workspace_values += static_cast<std::uint64_t>(values);
}

struct SeparatorRowWorkspace {
  explicit SeparatorRowWorkspace(const LPModel& lp)
      : inequalities(lp.A), equalities(lp.Aeq) {}

  int inequality_rows() const {
    return inequalities.rows() +
           static_cast<int>(appended_inequalities.size());
  }

  int equality_rows() const { return equalities.rows(); }

  template <typename Visitor>
  void visit_inequality_row(int row, Visitor&& visitor) const {
    if (row < 0 || row >= inequality_rows()) return;
    if (row < inequalities.rows()) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
               inequalities, row);
           it; ++it) {
        visitor(static_cast<int>(it.col()), it.value());
      }
      return;
    }
    const auto& sparse = appended_inequalities[
        static_cast<std::size_t>(row - inequalities.rows())];
    for (Eigen::SparseVector<double>::InnerIterator it(sparse); it; ++it) {
      visitor(static_cast<int>(it.index()), it.value());
    }
  }

  template <typename Visitor>
  void visit_equality_row(int row, Visitor&& visitor) const {
    if (row < 0 || row >= equalities.rows()) return;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             equalities, row);
         it; ++it) {
      visitor(static_cast<int>(it.col()), it.value());
    }
  }

  std::vector<std::pair<int, double>> inequality_row_terms(int row) const {
    std::vector<std::pair<int, double>> terms;
    visit_inequality_row(row, [&](int col, double value) {
      terms.emplace_back(col, value);
    });
    return terms;
  }

  void append_inequalities(
      const std::vector<Eigen::SparseVector<double>>& rows) {
    for (const auto& row : rows) {
      if (row.size() == inequalities.cols()) {
        appended_inequalities.push_back(row);
      }
    }
  }

  void sync_inequalities(const LPModel& lp) {
    const int known_rows = inequality_rows();
    const int new_rows = static_cast<int>(lp.A.rows()) - known_rows;
    if (new_rows <= 0 || lp.A.cols() != inequalities.cols()) return;
    std::vector<Eigen::SparseVector<double>> rows;
    rows.reserve(static_cast<std::size_t>(new_rows));
    for (int i = 0; i < new_rows; ++i) {
      rows.emplace_back(lp.A.cols());
    }
    for (int col = 0; col < lp.A.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
        const int row = static_cast<int>(it.row());
        if (row < known_rows || row >= known_rows + new_rows) continue;
        rows[static_cast<std::size_t>(row - known_rows)].insertBack(col) =
            it.value();
      }
    }
    append_inequalities(rows);
  }

  Eigen::SparseMatrix<double, Eigen::RowMajor> inequalities;
  Eigen::SparseMatrix<double, Eigen::RowMajor> equalities;
  std::vector<Eigen::SparseVector<double>> appended_inequalities;
};

inline double gmi_binary_activity_score(const SimplexResult& simplex,
                                 const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& cut) {
  const int n = std::min(static_cast<int>(cut.size()), simplex.form.n_original);
  double coeff_sum = 0.0;
  double weighted_fractionality = 0.0;
  int binary_nnz = 0;
  int high_frac_binary_nnz = 0;

  for (int j = 0; j < n; ++j) {
    if (simplex.form.original_types[static_cast<size_t>(j)] != VarType::Binary) {
      continue;
    }
    const double a = std::abs(cut[j]);
    if (a <= 1e-12) {
      continue;
    }
    ++binary_nnz;
    coeff_sum += a;

    const double frac = std::abs(x[j] - std::round(x[j]));
    const double centered = std::max(0.0, 1.0 - 2.0 * std::abs(x[j] - 0.5));
    weighted_fractionality += a * centered;
    if (frac >= 0.20) {
      ++high_frac_binary_nnz;
    }
  }

  if (binary_nnz == 0 || coeff_sum <= 1e-12) {
    return 0.0;
  }

  const double weighted = weighted_fractionality / coeff_sum;
  const double support = static_cast<double>(high_frac_binary_nnz) / static_cast<double>(binary_nnz);
  return 0.7 * weighted + 0.3 * support;
}

inline double gmi_binary_support_ratio(const SimplexResult& simplex,
                                const Eigen::VectorXd& cut) {
  const int n = std::min(static_cast<int>(cut.size()), simplex.form.n_original);
  int nnz_total = 0;
  int nnz_binary = 0;
  for (int j = 0; j < n; ++j) {
    if (std::abs(cut[j]) <= 1e-12) {
      continue;
    }
    ++nnz_total;
    if (simplex.form.original_types[static_cast<size_t>(j)] == VarType::Binary) {
      ++nnz_binary;
    }
  }
  if (nnz_total == 0) {
    return 0.0;
  }
  return static_cast<double>(nnz_binary) / static_cast<double>(nnz_total);
}

inline bool transformed_col_is_integer_like(
    const SimplexResult& simplex,
    int col,
    const std::vector<char>* implied_integer_cols = nullptr) {
  if (col < 0) {
    return false;
  }
  if (col >= simplex.form.n_original) {
    const int row = standard_form_aux_col_row(simplex.form, col);
    if (row < 0 || row >= simplex.form.A_row.rows()) {
      return false;
    }

    const bool have_row_scale =
        simplex.form.row_scale.size() == simplex.form.A_row.rows();
    const bool have_col_scale =
        simplex.form.col_scale.size() == simplex.form.A_row.cols();
    const double row_scale = have_row_scale ? simplex.form.row_scale[row] : 1.0;
    if (!std::isfinite(row_scale) || std::abs(row_scale) <= 1e-12) {
      return false;
    }

    double rhs = std::numeric_limits<double>::quiet_NaN();
    if (simplex.form.row_rhs_value.size() > row) {
      rhs = simplex.form.row_rhs_value[row];
    } else if (simplex.form.b.size() > row) {
      rhs = simplex.form.b[row] / row_scale;
    }
    if (!std::isfinite(rhs) || !is_integral(rhs, 1e-8)) {
      return false;
    }

    bool has_structural_coeff = false;
    for (StandardRowMatrix::InnerIterator it(
             simplex.form.A_row, row);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= simplex.form.n_original) {
        continue;
      }
      if (!transformed_col_is_integer_like(simplex, j,
                                           implied_integer_cols)) {
        return false;
      }
      const double col_scale = have_col_scale ? simplex.form.col_scale[j] : 1.0;
      if (!std::isfinite(col_scale) || std::abs(col_scale) <= 1e-12) {
        return false;
      }
      const double unscaled_coeff = it.value() / (row_scale * col_scale);
      if (!std::isfinite(unscaled_coeff) || !is_integral(unscaled_coeff, 1e-8)) {
        return false;
      }
      has_structural_coeff = true;
    }
    return has_structural_coeff;
  }
  const bool declared_integer =
      transformed_original_var_is_integer(simplex, col);
  const bool implied_integer =
      implied_integer_cols != nullptr &&
      col < static_cast<int>(implied_integer_cols->size()) &&
      (*implied_integer_cols)[static_cast<std::size_t>(col)] != 0 &&
      is_integral(simplex.form.lb_shift[col], 1e-9);
  return declared_integer || implied_integer;
}

}  // namespace mipsolvers::engine::detail
