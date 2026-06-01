/// @file bc_cuts.cpp
/// @brief Cut generation routines for the B&C solver.
///
/// Implements GMI (Gomory Mixed-Integer) cuts, single-row complemented MIR cuts,
/// cover cuts, flow cover cuts, implied bound cuts,
/// and the unified add_cuts dispatch. Includes scoring helpers for cut selection.

#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/detail/bc_clique_table.hpp"

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

namespace {

double frac_part(double v) {
  const double f = v - std::floor(v);
  return (f < 1e-12 || f > 1.0 - 1e-12) ? 0.0 : f;
}

bool basis_tableau_cuts_admissible(const SimplexResult& simplex,
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

bool transformed_original_var_is_integer(const SimplexResult& simplex, int col) {
  if (col < 0 || col >= simplex.form.n_original) {
    return false;
  }
  const VarType t = simplex.form.original_types[static_cast<size_t>(col)];
  if (!(t == VarType::Integer || t == VarType::Binary)) {
    return false;
  }
  return is_integral(simplex.form.lb_shift[col], 1e-9);
}

double original_space_value(const SimplexResult& simplex,
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

int standard_form_aux_col_row(const StandardFormLP& sf, int col) {
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

bool build_bounded_form_gmi_cut(const SimplexResult& simplex,
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
    tableau_row =
        simplex.basis_inverse.row(row) * simplex.form.A;
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
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(simplex.form.A_row, r); it; ++it) {
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

#if 0  // MIR cuts disabled: correct but hurt tree search performance.
/// Build a tableau-row MIR cut with strategic complementation.
///
/// Unlike Gomory which takes the tableau row as-is, MIR complements bounded
/// integer non-basics at lower bound when frac(a_j) >= 1-f0 to move them
/// into the "good" coefficient set. This can produce strictly tighter cuts.
/// Returns false if no complementation candidates exist (Gomory already covers).
bool build_bounded_form_mir_cut(const SimplexResult& simplex,
                                int row,
                                [[maybe_unused]] const Eigen::VectorXd& x_lp,
                                Eigen::VectorXd& cut_le,
                                double& rhs_le,
                                const std::shared_ptr<BasisOps>& sbasis,
                                const std::vector<char>* is_basic_hint = nullptr) {
  const int n_std = static_cast<int>(simplex.form.A.cols());
  const int n_orig = simplex.form.n_original;
  const int m = static_cast<int>(simplex.basis.indices.size());
  if (row < 0 || row >= m) return false;
  if (!sbasis && row >= static_cast<int>(simplex.basis_inverse.rows())) return false;

  const int basic_col = simplex.basis.indices[static_cast<size_t>(row)];
  if (!transformed_original_var_is_integer(simplex, basic_col)) return false;

  // beta already accounts for at-upper positions — no adjustment needed.
  const double beta = simplex.x_basic[row];
  const double f0_raw = frac_part(beta);
  if (f0_raw <= 1e-8 || f0_raw >= 1.0 - 1e-8) return false;

  // Compute tableau row via BTRAN (same as Gomory).
  Eigen::RowVectorXd tableau_row;
  if (sbasis) {
    if (!sparse_basis_tableau_row(sbasis, row, tableau_row)) return false;
  } else {
    tableau_row = simplex.basis_inverse.row(row) * simplex.form.A;
  }
  if (!tableau_row.allFinite()) return false;

  // Identify basic/nonbasic status — reuse pre-built hint when available.
  std::vector<char> is_basic_local;
  if (!is_basic_hint || static_cast<int>(is_basic_hint->size()) != n_std) {
    is_basic_local.assign(static_cast<size_t>(n_std), 0);
    for (int idx : simplex.basis.indices) {
      if (idx >= 0 && idx < n_std) is_basic_local[static_cast<size_t>(idx)] = 1;
    }
    is_basic_hint = &is_basic_local;
  }
  const std::vector<char>& is_basic = *is_basic_hint;

  const bool has_at_upper = !simplex.basis.at_upper.empty();
  const bool has_var_ub = simplex.form.var_ub.size() == n_std;

  // Strategic complementation: complement at-lower bounded integer non-basics
  // where frac(a_j) >= 1-f0 to move them into the "good" coefficient set.
  // Complementation: x_j = ub_j - x'_j changes coefficient a_j → -a_j and
  // adjusts RHS by -a_j * ub_j.
  std::vector<char> complemented(static_cast<size_t>(n_std), 0);
  double beta_comp = beta;
  int n_complemented = 0;

  for (int j = 0; j < n_std; ++j) {
    if (is_basic[static_cast<size_t>(j)]) continue;
    if (j >= n_orig || !transformed_original_var_is_integer(simplex, j)) continue;
    const bool j_at_upper = has_at_upper && simplex.basis.at_upper[static_cast<size_t>(j)];
    if (j_at_upper) continue;  // Already handled by at-upper substitution.
    if (!has_var_ub || !std::isfinite(simplex.form.var_ub[j])) continue;

    const double a_j = tableau_row[j];
    const double fj = frac_part(a_j);
    // Complement if f_j > f0 and complementing moves to "good" set (1-f_j ≤ f0).
    if (fj > f0_raw + 1e-10 && fj >= 1.0 - f0_raw - 1e-10) {
      complemented[static_cast<size_t>(j)] = 1;
      beta_comp -= a_j * simplex.form.var_ub[j];
      ++n_complemented;
    }
  }

  // Only generate MIR cut if we're actually complementing something.
  // Without complementation, the cut is identical to Gomory (already generated).
  if (n_complemented == 0) return false;

  const double f0 = frac_part(beta_comp);
  if (f0 <= 1e-8 || f0 >= 1.0 - 1e-8) return false;

  // Apply MIR rounding to the (complemented + at-upper-substituted) tableau row.
  Eigen::VectorXd std_cut = Eigen::VectorXd::Zero(n_std);
  bool has_term = false;
  double rhs_adjustment = 0.0;

  for (int j = 0; j < n_std; ++j) {
    if (is_basic[static_cast<size_t>(j)]) continue;  // NOLINT(readability)

    const bool j_at_upper = has_at_upper && simplex.basis.at_upper[static_cast<size_t>(j)];
    const bool j_comp = complemented[static_cast<size_t>(j)] != 0;
    // Effective coefficient: at-upper or complemented → sign flip.
    const double a_j = (j_at_upper || j_comp) ? (-tableau_row[j]) : tableau_row[j];

    double coeff = 0.0;
    if (j < n_orig && transformed_original_var_is_integer(simplex, j)) {
      // Integer variable: MIR rounding.
      const double fj = frac_part(a_j);
      if (fj <= f0 + 1e-10) {
        coeff = std::floor(a_j);
      } else {
        coeff = std::floor(a_j) + (fj - f0) / (1.0 - f0);
      }
    } else {
      // Continuous (or slack/surplus): MIR strengthening.
      if (a_j < -1e-12) {
        coeff = a_j / (1.0 - f0);
      } else {
        coeff = 0.0;
      }
    }

    if (std::abs(coeff) <= 1e-12) continue;

    // Un-substitute: at-upper and complemented variables both need
    // coeff * x'_j = coeff * (ub_j - x_j) → -coeff*x_j, rhs += coeff*ub_j.
    if (j_at_upper || j_comp) {
      std_cut[j] = -coeff;
      if (has_var_ub && std::isfinite(simplex.form.var_ub[j])) {
        rhs_adjustment += coeff * simplex.form.var_ub[j];
      }
    } else {
      std_cut[j] = coeff;
    }
    has_term = true;
  }

  if (!has_term) return false;

  // Convert from standard-form to original-variable space.
  Eigen::VectorXd coeff_y = std_cut.head(n_orig);
  double rhs_ge = std::floor(beta_comp) - rhs_adjustment;

  for (int r = 0; r < static_cast<int>(simplex.form.row_to_slack_col.size()); ++r) {
    const int slack_col = simplex.form.row_to_slack_col[static_cast<size_t>(r)];
    if (slack_col >= 0 && std::abs(std_cut[slack_col]) > 1e-12) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(simplex.form.A_row, r); it; ++it) {
        if (it.col() < n_orig) {
          coeff_y[it.col()] -= std_cut[slack_col] * it.value();
        }
      }
      rhs_ge -= std_cut[slack_col] * simplex.form.b[r];
    }
    const int surplus_col = simplex.form.row_to_surplus_col[static_cast<size_t>(r)];
    if (surplus_col >= 0 && std::abs(std_cut[surplus_col]) > 1e-12) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(simplex.form.A_row, r); it; ++it) {
        if (it.col() < n_orig) {
          coeff_y[it.col()] += std_cut[surplus_col] * it.value();
        }
      }
      rhs_ge += std_cut[surplus_col] * simplex.form.b[r];
    }
  }

  const double rhs_x_ge = rhs_ge + coeff_y.dot(simplex.form.lb_shift);
  cut_le = -coeff_y;
  rhs_le = -rhs_x_ge;

  if (cut_le.norm() <= 1e-12 || !std::isfinite(rhs_le) || !cut_le.allFinite()) return false;
  return true;
}
#endif  // MIR cuts disabled

int count_nonzeros(const Eigen::VectorXd& v, double tol = 1e-12) {
  int nnz = 0;
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    if (std::abs(v[i]) > tol) {
      ++nnz;
    }
  }
  return nnz;
}

double abs_cosine_similarity(const Eigen::VectorXd& a,
                             const Eigen::VectorXd& b,
                             double norm_a,
                             double norm_b) {
  if (norm_a <= 1e-12 || norm_b <= 1e-12) {
    return 1.0;
  }
  const double cos = a.dot(b) / (norm_a * norm_b);
  return std::abs(cos);
}

double gmi_binary_activity_score(const SimplexResult& simplex,
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

double gmi_binary_support_ratio(const SimplexResult& simplex,
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

bool transformed_col_is_integer_like(
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
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
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

double fast_floor_xtab(double x) {
  // Guard UB: static_cast<int64_t> is undefined for values outside the int64
  // range (~±9.22e18) and for inf/NaN.  For those rare inputs fall back to the
  // standard library floor which is always well-defined.
  constexpr double kSafeMax = 4503599627370496.0;  // 2^52: every integer here
                                                    // is exactly representable
  if (std::abs(x) >= kSafeMax || !std::isfinite(x)) {
    return std::floor(x);
  }
  const auto ix = static_cast<int64_t>(x);
  return static_cast<double>(ix - (x < static_cast<double>(ix)));
}

double pow2_scale_for_max_abs(double max_abs) {
  if (!(max_abs > 0.0) || !std::isfinite(max_abs)) {
    return 1.0;
  }
  int exp_shift = 0;
  std::frexp(max_abs, &exp_shift);
  exp_shift = std::min(10, -exp_shift);
  return std::ldexp(1.0, exp_shift);
}

double xtab_nearest_integer(double value) {
  return static_cast<double>(HighsIntegers::nearestInteger(value));
}

double xtab_integral_scale(const std::vector<double>& values,
                           double feastol,
                           double epsilon) {
  return HighsIntegers::integralScale(values, feastol, epsilon);
}

double xtab_fractionality_distance(double value) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(value - std::round(value));
}

enum class XTabBoundType : unsigned char {
  SimpleLb = 0,
  SimpleUb = 1,
  VariableLb = 2,
  VariableUb = 3,
};

struct XTabVarBoundExpr {
  bool valid{false};
  int trigger_col{-1};
  double constant{0.0};    // In transformed-space of target column.
  double coef{0.0};        // Coefficient on transformed trigger column.
  double dist{std::numeric_limits<double>::infinity()};
  double scaled_dist{std::numeric_limits<double>::infinity()};
  double min_value{std::numeric_limits<double>::infinity()};
  double max_value{-std::numeric_limits<double>::infinity()};
  double lift{0.0};
  bool integer_target{false};
};

struct XTabSparseVectorSum {
  std::vector<double> values;
  std::vector<int> nonzeros;

  explicit XTabSparseVectorSum(int dim = 0) { reset_dim(dim); }

  void reset_dim(int dim) {
    values.assign(static_cast<std::size_t>(std::max(0, dim)), 0.0);
    nonzeros.clear();
    nonzeros.reserve(static_cast<std::size_t>(std::max(0, dim)));
  }

  bool empty() const { return nonzeros.empty(); }

  void add(int index, double value) {
    if (index < 0 || index >= static_cast<int>(values.size())) return;
    double& current = values[static_cast<std::size_t>(index)];
    if (current != 0.0) {
      current += value;
    } else {
      current = value;
      nonzeros.push_back(index);
    }
    if (current == 0.0) {
      current = std::numeric_limits<double>::min();
    }
  }

  template <typename IsZero>
  void cleanup(IsZero&& is_zero) {
    int num_nz = static_cast<int>(nonzeros.size());
    for (int i = num_nz - 1; i >= 0; --i) {
      const int col = nonzeros[static_cast<std::size_t>(i)];
      const double value = values[static_cast<std::size_t>(col)];
      if (!is_zero(col, value)) continue;
      values[static_cast<std::size_t>(col)] = 0.0;
      --num_nz;
      std::swap(nonzeros[static_cast<std::size_t>(num_nz)],
                nonzeros[static_cast<std::size_t>(i)]);
    }
    nonzeros.resize(static_cast<std::size_t>(num_nz));
  }
};

struct XTabTransformContext {
  std::vector<XTabVarBoundExpr> best_vlb;
  std::vector<XTabVarBoundExpr> best_vub;
  std::uint64_t num_vlb{0};
  std::uint64_t num_vub{0};
  std::uint64_t num_integral_vlb{0};
  std::uint64_t num_integral_vub{0};
};

struct XTabRow {
  std::vector<int> inds;
  std::vector<double> vals;
  std::vector<double> upper;
  std::vector<double> solval;
  std::vector<XTabBoundType> bound_type;
  std::vector<XTabVarBoundExpr> varbound_expr;
  std::vector<unsigned char> complemented;
  std::vector<unsigned char> is_integral;
  double rhs{0.0};
  double initial_scale{1.0};
  bool integral_support{false};
  bool integral_coefficients{false};
};

struct XTabCmirTrace {
  int integer_terms{0};
  int continuous_terms{0};
  int initial_delta_count{0};
  int tested_delta_count{0};
  double continuous_contribution{0.0};
  double continuous_norm_sq{0.0};
  double max_abs_delta{0.0};
  double best_delta{-1.0};
  double best_efficacy{-std::numeric_limits<double>::infinity()};
  double final_f0{std::numeric_limits<double>::quiet_NaN()};
  double final_rhs{std::numeric_limits<double>::quiet_NaN()};
  bool accepted{false};
  bool lifted_accepted{false};
};

struct XTabLiftedCoverState {
  std::vector<int> cover;
  HighsCDouble cover_weight{0.0};
  HighsCDouble lambda{0.0};
};

struct XTabCutgenRandom {
  std::optional<HighsRandom> randgen;

  explicit XTabCutgenRandom(std::optional<std::uint64_t> seed) {
    if (seed.has_value()) {
      randgen.emplace(static_cast<HighsUInt>(*seed));
    }
  }

  int next_tiebreaker(int fallback) {
    if (!randgen.has_value()) return fallback;
    return static_cast<int>(randgen->integer());
  }
};

std::uint64_t xtab_highs_pair_hash(std::uint32_t a,
                                   std::uint32_t b,
                                   int k);

struct XTabSourceContext {
  int n_original{0};
  int n_rows{0};
  int dim{0};
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  Eigen::VectorXd solution;
  std::vector<unsigned char> integral;
  bool valid{false};

  bool is_logical_row_slack(int col) const {
    return col >= n_original && col < dim;
  }

  int row_from_logical_slack(int col) const {
    return col - n_original;
  }
};

double xtab_row_scale_or_one(const StandardFormLP& sf, int row) {
  if (row >= 0 && row < sf.row_scale.size()) {
    const double scale = sf.row_scale[row];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

double xtab_col_scale_or_one(const StandardFormLP& sf, int col) {
  if (col >= 0 && col < sf.col_scale.size()) {
    const double scale = sf.col_scale[col];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

[[maybe_unused]] double xtab_source_row_coeff(const StandardFormLP& sf, int row, int col) {
  if (row < 0 || row >= sf.A_row.rows() || col < 0 ||
      col >= sf.n_original) {
    return 0.0;
  }
  const double scaled = sf.A_row.coeff(row, col);
  if (std::abs(scaled) <= 1e-15) return 0.0;
  return scaled / (xtab_row_scale_or_one(sf, row) *
                   xtab_col_scale_or_one(sf, col));
}

bool xtab_source_row_has_highs_order(const StandardFormLP& sf, int row) {
  if (row < 0 || row >= static_cast<int>(sf.source_highs_row.size())) {
    return false;
  }
  const int highs_row = sf.source_highs_row[static_cast<std::size_t>(row)];
  if (highs_row < 0 ||
      highs_row + 1 > static_cast<int>(sf.source_row_start.size()) - 1) {
    return false;
  }
  const int start = sf.source_row_start[static_cast<std::size_t>(highs_row)];
  const int end = sf.source_row_start[static_cast<std::size_t>(highs_row + 1)];
  return start >= 0 && end >= start &&
         end <= static_cast<int>(sf.source_row_index.size()) &&
         end <= static_cast<int>(sf.source_row_value.size());
}

template <typename Func>
void xtab_visit_source_row_terms(const StandardFormLP& sf,
                                 int row,
                                 Func&& func) {
  if (xtab_source_row_has_highs_order(sf, row)) {
    const int highs_row = sf.source_highs_row[static_cast<std::size_t>(row)];
    const int start = sf.source_row_start[static_cast<std::size_t>(highs_row)];
    const int end = sf.source_row_start[static_cast<std::size_t>(highs_row + 1)];
    for (int p = start; p < end; ++p) {
      const int col = sf.source_row_index[static_cast<std::size_t>(p)];
      if (col < 0 || col >= sf.n_original) continue;
      const double value = sf.source_row_value[static_cast<std::size_t>(p)];
      if (std::abs(value) <= 1e-15) continue;
      func(col, value);
    }
    return;
  }

  const double row_scale_inv = 1.0 / xtab_row_scale_or_one(sf, row);
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(sf.A_row,
                                                                      row);
       it; ++it) {
    const int col = static_cast<int>(it.col());
    if (col < 0 || col >= sf.n_original) continue;
    const double scaled = it.value();
    if (std::abs(scaled) <= 1e-15) continue;
    const double value = scaled * row_scale_inv / xtab_col_scale_or_one(sf, col);
    if (std::abs(value) <= 1e-15) continue;
    func(col, value);
  }
}

double xtab_source_row_side(const StandardFormLP& sf, int row) {
  if (row < 0 || row >= sf.A_row.rows()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const int sign = row < static_cast<int>(sf.row_sign.size())
                       ? sf.row_sign[static_cast<std::size_t>(row)]
                       : 1;
  if (sf.row_rhs_value.size() == sf.A_row.rows()) {
    return static_cast<double>(sign) * sf.row_rhs_value[row];
  }
  return sf.b[row] / xtab_row_scale_or_one(sf, row);
}

int xtab_row_trace_limit() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_ROW_TRACE");
  if (env == nullptr) return 0;
  if (env[0] == '\0') return 20;
  if (std::string(env) == "all") return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return 20;
  if (value <= 0) return 0;
  return static_cast<int>(std::min<long>(value, 1000000));
}

int xtab_row_trace_terms() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_ROW_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 12;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 12;
  return static_cast<int>(std::min<long>(value, 200));
}

std::uint64_t xtab_row_trace_id_bound(const char* name,
                                      std::uint64_t default_value) {
  const char* env = std::getenv(name);
  if (env == nullptr || env[0] == '\0') return default_value;
  char* end = nullptr;
  const unsigned long long value = std::strtoull(env, &end, 10);
  if (end == env) return default_value;
  return static_cast<std::uint64_t>(value);
}

int xtab_transform_trace_col() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_TRANSFORM_TRACE_COL");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

int xtab_varbound_trace_col() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_VB_TRACE_COL");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

int xtab_modk_system_trace_limit() {
  const char* env = std::getenv("MIPSOLVERS_XMODK_SYSTEM_TRACE");
  if (env == nullptr || env[0] == '\0') return 0;
  if (std::string(env) == "all") return 1000000000;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return 40;
  return value > 0 ? static_cast<int>(std::min<long>(value, 1000000)) : 0;
}

int xtab_modk_system_trace_terms() {
  const char* env = std::getenv("MIPSOLVERS_XMODK_SYSTEM_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 16;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 16;
  return static_cast<int>(std::min<long>(value, 200));
}

int xtab_modk_transform_trace_row() {
  const char* env = std::getenv("MIPSOLVERS_XMODK_TRANSFORM_ROW");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

bool xtab_row_trace_family_enabled(const char* family) {
  const char* env = std::getenv("MIPSOLVERS_XTAB_ROW_TRACE_FAMILY");
  if (env == nullptr || env[0] == '\0' || std::string(env) == "all") {
    return true;
  }
  return family != nullptr && std::string(env) == family;
}

bool xtab_row_trace_meta_enabled() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_ROW_TRACE_META");
  if (env == nullptr) return false;
  return env[0] != '\0' && std::string(env) != "0";
}

bool xtab_row_trace_basis_enabled() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_ROW_TRACE_BASIS");
  if (env == nullptr) return false;
  return env[0] != '\0' && std::string(env) != "0";
}

std::uint64_t xtab_highs_pair_hash(std::uint32_t a,
                                   std::uint32_t b,
                                   int k) {
  static constexpr std::uint64_t c[] = {
      std::uint64_t{0xc8497d2a400d9551},
      std::uint64_t{0x80c8963be3e4c2f3},
      std::uint64_t{0x042d8680e260ae5b},
      std::uint64_t{0x8a183895eeac1536},
  };
  const int idx = 2 * k;
  return (static_cast<std::uint64_t>(a) + c[idx]) *
         (static_cast<std::uint64_t>(b) + c[idx + 1]);
}

std::uint64_t xtab_highs_hash_i64(std::int64_t value) {
  const std::uint64_t bits = static_cast<std::uint64_t>(value);
  const auto lo = static_cast<std::uint32_t>(bits & 0xffffffffu);
  const auto hi = static_cast<std::uint32_t>(bits >> 32);
  return xtab_highs_pair_hash(lo, hi, 1) ^
         (xtab_highs_pair_hash(lo, hi, 0) >> 32);
}

std::uint64_t xtab_modk_hash_mix(std::uint64_t h, std::uint64_t v) {
  v ^= v >> 33;
  v *= std::uint64_t{0xff51afd7ed558ccd};
  v ^= v >> 33;
  v *= std::uint64_t{0xc4ceb9fe1a85ec53};
  v ^= v >> 33;
  return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} + (h << 6) + (h >> 2));
}

std::uint64_t xtab_modk_system_hash(
    const std::vector<std::int64_t>& values,
    const std::vector<int>& indices,
    const std::vector<int>& starts) {
  std::uint64_t h = std::uint64_t{0x48494748534d4f44};
  h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(values.size()));
  h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(indices.size()));
  h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(starts.size()));
  for (int start : starts) {
    h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(start));
  }
  for (int index : indices) {
    h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(index));
  }
  for (std::int64_t value : values) {
    h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(value));
  }
  return h;
}

bool xtab_claim_row_trace(std::uint64_t& id, const char* family) {
  if (!xtab_row_trace_family_enabled(family)) return false;
  const int limit = xtab_row_trace_limit();
  if (limit == 0) return false;
  static std::atomic<std::uint64_t> next_id{1};
  id = next_id.fetch_add(1, std::memory_order_relaxed);
  const std::uint64_t min_id =
      xtab_row_trace_id_bound("MIPSOLVERS_XTAB_ROW_TRACE_MIN_ID", 1);
  const std::uint64_t max_id = xtab_row_trace_id_bound(
      "MIPSOLVERS_XTAB_ROW_TRACE_MAX_ID",
      std::numeric_limits<std::uint64_t>::max());
  if (id < min_id || id > max_id) return false;
  return limit < 0 || id <= static_cast<std::uint64_t>(limit);
}

const char* xtab_bound_type_name(XTabBoundType type) {
  switch (type) {
    case XTabBoundType::SimpleLb:
      return "simple_lb";
    case XTabBoundType::SimpleUb:
      return "simple_ub";
    case XTabBoundType::VariableLb:
      return "vlb";
    case XTabBoundType::VariableUb:
      return "vub";
  }
  return "unknown";
}

struct XTabSparseStats {
  int nnz{0};
  double l1{0.0};
  double max_abs{0.0};
  std::string sample;
};

XTabSparseStats xtab_sparse_stats(const Eigen::VectorXd& coeff,
                                  const std::vector<int>* active_cols,
                                  int max_terms) {
  XTabSparseStats stats;
  fmt::memory_buffer buffer;
  int emitted = 0;
  auto visit = [&](int col) {
    if (col < 0 || col >= coeff.size()) return;
    const double val = coeff[col];
    if (std::abs(val) <= 1e-12) return;
    ++stats.nnz;
    stats.l1 += std::abs(val);
    stats.max_abs = std::max(stats.max_abs, std::abs(val));
    if (emitted < max_terms) {
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{}:{:.6g}", col, val);
      ++emitted;
    }
  };
  if (active_cols != nullptr) {
    for (int col : *active_cols) visit(col);
  } else {
    for (int col = 0; col < coeff.size(); ++col) visit(col);
  }
  if (stats.nnz > emitted) {
    fmt::format_to(std::back_inserter(buffer), ",...");
  }
  stats.sample = fmt::to_string(buffer);
  return stats;
}

struct XTabRowStats {
  int nnz{0};
  int simple_lb{0};
  int simple_ub{0};
  int variable_lb{0};
  int variable_ub{0};
  int integral{0};
  int continuous{0};
  int logical{0};
  int complemented{0};
  double l1{0.0};
  double max_abs{0.0};
  double activity{0.0};
  double norm{0.0};
  std::string sample;
};

XTabRowStats xtab_row_stats(const XTabRow& row,
                            const XTabSourceContext& source_context,
                            int max_terms) {
  XTabRowStats stats;
  stats.nnz = static_cast<int>(row.inds.size());
  fmt::memory_buffer buffer;
  int emitted = 0;
  double norm_sq = 0.0;
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    const int col = row.inds[pos];
    const double val = row.vals[pos];
    stats.l1 += std::abs(val);
    stats.max_abs = std::max(stats.max_abs, std::abs(val));
    stats.activity += val * row.solval[pos];
    norm_sq += val * val;
    switch (row.bound_type[pos]) {
      case XTabBoundType::SimpleLb:
        ++stats.simple_lb;
        break;
      case XTabBoundType::SimpleUb:
        ++stats.simple_ub;
        break;
      case XTabBoundType::VariableLb:
        ++stats.variable_lb;
        break;
      case XTabBoundType::VariableUb:
        ++stats.variable_ub;
        break;
    }
    if (row.is_integral[pos] != 0) {
      ++stats.integral;
    } else {
      ++stats.continuous;
    }
    if (source_context.is_logical_row_slack(col)) ++stats.logical;
    if (row.complemented[pos] != 0) ++stats.complemented;
    if (emitted < max_terms) {
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{}:{:.17g}:{}:s{:.17g}",
                     col, val, xtab_bound_type_name(row.bound_type[pos]),
                     row.solval[pos]);
      ++emitted;
    }
  }
  if (stats.nnz > emitted) {
    fmt::format_to(std::back_inserter(buffer), ",...");
  }
  stats.norm = std::sqrt(norm_sq);
  stats.sample = fmt::to_string(buffer);
  return stats;
}

void xtab_trace_source_row(std::uint64_t id,
                           const char* family,
                           const char* stage,
                           const Eigen::VectorXd& coeff,
                           double rhs,
                           const std::vector<int>* active_cols) {
  const XTabSparseStats stats =
      xtab_sparse_stats(coeff, active_cols, xtab_row_trace_terms());
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={} rhs={:.17g} nnz={} "
             "l1={:.6g} max={:.6g} baseRowIndsVals=[{}]\n",
             id, family, stage, rhs, stats.nnz, stats.l1, stats.max_abs,
             stats.sample);
}

std::string xtab_source_row_signature(const SimplexResult& simplex,
                                      int row,
                                      int max_terms) {
  fmt::memory_buffer buffer;
  int emitted = 0;
  int nnz = 0;
  xtab_visit_source_row_terms(simplex.form, row, [&](int col, double val) {
    if (std::abs(val) <= 1e-12) return;
    ++nnz;
    if (emitted < max_terms) {
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{}:{:.12g}", col, val);
      ++emitted;
    }
  });
  if (nnz > emitted) {
    fmt::format_to(std::back_inserter(buffer), ",...");
  }
  return fmt::to_string(buffer);
}

void xtab_trace_source_meta(std::uint64_t id,
                            const char* family,
                            const char* stage,
                            const SimplexResult& simplex,
                            const XTabSourceContext& source_context,
                            const Eigen::VectorXd& coeff,
                            const std::vector<int>* active_cols) {
  if (!xtab_row_trace_meta_enabled() || !source_context.valid) return;
  const int max_terms = xtab_row_trace_terms();
  fmt::memory_buffer buffer;
  int emitted = 0;
  auto visit = [&](int col) {
    if (col < source_context.n_original || col >= source_context.dim ||
        col >= coeff.size()) {
      return;
    }
    const double val = coeff[col];
    if (std::abs(val) <= 1e-12) return;
    const int row = source_context.row_from_logical_slack(col);
    if (row < 0 || row >= source_context.n_rows) return;
    if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ";");
    fmt::format_to(std::back_inserter(buffer),
                   "{}:{:.12g}>row{}:lb{:.12g}:ub{:.12g}:act{:.12g}:sig[{}]",
                   col, val, row, source_context.lower[col],
                   source_context.upper[col], source_context.solution[col],
                   xtab_source_row_signature(simplex, row, max_terms));
    ++emitted;
  };
  if (active_cols != nullptr) {
    for (int col : *active_cols) {
      if (emitted >= max_terms) break;
      visit(col);
    }
  } else {
    for (int col = source_context.n_original;
         col < source_context.dim && emitted < max_terms; ++col) {
      visit(col);
    }
  }
  if (emitted == 0) return;
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={}_meta slacks=[{}]\n",
             id, family, stage, fmt::to_string(buffer));
}

void xtab_trace_row(std::uint64_t id,
                    const char* family,
                    const char* stage,
                    const XTabRow& row,
                    const XTabSourceContext& source_context,
                    bool integers_positive) {
  const XTabRowStats stats =
      xtab_row_stats(row, source_context, xtab_row_trace_terms());
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={} rhs={:.12g} nnz={} "
             "types=slb{}:sub{}:vlb{}:vub{} int={} cont={} log={} "
             "comp={} intPos={} activity={:.17g} viol={:.17g} "
             "norm={:.17g} l1={:.17g} max={:.17g} row=[{}]\n",
             id, family, stage, row.rhs, stats.nnz, stats.simple_lb,
             stats.simple_ub, stats.variable_lb, stats.variable_ub,
             stats.integral, stats.continuous, stats.logical,
             stats.complemented, integers_positive ? 1 : 0, stats.activity,
             stats.activity - row.rhs, stats.norm, stats.l1, stats.max_abs,
             stats.sample);
}

void xtab_trace_original_row(std::uint64_t id,
                             const char* family,
                             const char* stage,
                             const Eigen::VectorXd& cut,
                             double rhs,
                             const Eigen::VectorXd& x) {
  const XTabSparseStats stats =
      xtab_sparse_stats(cut, nullptr, xtab_row_trace_terms());
  const double activity =
      cut.size() == x.size() ? cut.dot(x)
                             : std::numeric_limits<double>::quiet_NaN();
  const double norm = std::max(1e-12, cut.norm());
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={} rhs={:.12g} nnz={} "
             "activity={:.12g} viol={:.12g} eff={:.6g} l1={:.6g} "
             "max={:.6g} row=[{}]\n",
             id, family, stage, rhs, stats.nnz, activity, activity - rhs,
             (activity - rhs) / norm, stats.l1, stats.max_abs, stats.sample);
}

bool xtab_cmir_delta_trace_enabled() {
  const char* env = std::getenv("MIPSOLVERS_XTAB_CMIR_DELTA_TRACE");
  return env != nullptr && env[0] != '\0' && std::string(env) != "0";
}

void xtab_trace_cmir_delta(std::uint64_t id,
                           const char* family,
                           const char* phase,
                           double delta,
                           double scale,
                           double down_rhs,
                           double f0,
                           double violation,
                           double norm_sq,
                           double efficacy,
                           double best_before) {
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage=cmir_delta phase={} "
             "delta={:.17g} scale={:.17g} downRhs={:.17g} f0={:.17g} "
             "viol={:.17g} normSq={:.17g} eff={:.17g} bestBefore={:.17g}\n",
             id, family == nullptr ? "" : family, phase, delta, scale,
             down_rhs, f0, violation, norm_sq, efficacy, best_before);
}

void xtab_trace_fail(std::uint64_t id,
                     const char* family,
                     const char* stage,
                     const char* reason) {
  fmt::print(stderr, "[B&C-XROW] id={} family={} stage={} fail={}\n", id,
             family, stage, reason);
}

enum class XTabRejectReason : unsigned char {
  None = 0,
  Transform,
  Preprocess,
  Cmir,
  EmptyCut,
  Untransform,
  NotViolated,
};

struct XTabSourceDiag {
  std::uint64_t basis_rows{0};
  std::uint64_t integer_basic_original{0};
  std::uint64_t integer_basic_aux{0};
  std::uint64_t fractional_basic_original{0};
  std::uint64_t fractional_basic_aux{0};
  std::uint64_t btran_rows{0};
  std::uint64_t row_ep_rows{0};
  std::uint64_t aggregate_ok{0};
  std::uint64_t transform_fail{0};
  std::uint64_t preprocess_fail{0};
  std::uint64_t cmir_fail{0};
  std::uint64_t empty_fail{0};
  std::uint64_t untransform_fail{0};
  std::uint64_t not_violated{0};
  std::uint64_t gate_calls{0};
  std::uint64_t gate_transform_ok{0};
  std::uint64_t gate_preprocess_ok{0};
  std::uint64_t gate_cmir_ok{0};
  std::uint64_t gate_untransform_ok{0};
  std::uint64_t gate_violation_ok{0};
  std::uint64_t generated{0};
  std::uint64_t filtered_density{0};
  std::uint64_t filtered_efficacy{0};
  std::uint64_t filtered_activity{0};
  std::uint64_t filtered_binary_support{0};
  std::uint64_t filtered_parallel{0};
  std::uint64_t selected{0};
  std::uint64_t vb_substitutions{0};
  std::uint64_t vb_trigger_terms{0};
};

void xtab_record_reject(XTabSourceDiag* diag, XTabRejectReason reason) {
  if (diag == nullptr) return;
  switch (reason) {
    case XTabRejectReason::Transform:
      ++diag->transform_fail;
      break;
    case XTabRejectReason::Preprocess:
      ++diag->preprocess_fail;
      break;
    case XTabRejectReason::Cmir:
      ++diag->cmir_fail;
      break;
    case XTabRejectReason::EmptyCut:
      ++diag->empty_fail;
      break;
    case XTabRejectReason::Untransform:
      ++diag->untransform_fail;
      break;
    case XTabRejectReason::NotViolated:
      ++diag->not_violated;
      break;
    case XTabRejectReason::None:
      break;
  }
}

void maybe_print_xtab_diag(const char* family, const XTabSourceDiag& diag) {
  if (std::getenv("MIPSOLVERS_XTAB_DIAG") == nullptr) return;
  fmt::print(stderr,
             "[B&C-XTAB-DIAG] family={} basis={} intOrig={} intAux={} "
             "fracOrig={} fracAux={} btran={} rowEp={} agg={} "
             "reject=T{} P{} C{} E{} U{} V{} gen={} "
             "gate={}>{}>{}>{}>{}>{} filt=d{} e{} a{} b{} p{} sel={} "
             "vb={}/{}\n",
             family, diag.basis_rows, diag.integer_basic_original,
             diag.integer_basic_aux, diag.fractional_basic_original,
             diag.fractional_basic_aux, diag.btran_rows, diag.row_ep_rows,
             diag.aggregate_ok, diag.transform_fail, diag.preprocess_fail,
             diag.cmir_fail, diag.empty_fail, diag.untransform_fail,
             diag.not_violated, diag.generated, diag.gate_calls,
             diag.gate_transform_ok, diag.gate_preprocess_ok,
             diag.gate_cmir_ok, diag.gate_untransform_ok,
             diag.gate_violation_ok, diag.filtered_density,
             diag.filtered_efficacy, diag.filtered_activity,
             diag.filtered_binary_support, diag.filtered_parallel,
             diag.selected, diag.vb_substitutions, diag.vb_trigger_terms);
}

struct XTabCandidateCut {
  Eigen::VectorXd coeff;
  double rhs{0.0};
  double score{0.0};
  double efficacy{0.0};
  double norm{0.0};
  int nnz{0};
  std::uint64_t source_trace_id{0};
};

PoolCut xtab_candidate_to_pool_cut(const XTabCandidateCut& cand) {
  Eigen::SparseVector<double> sparse = dense_to_sparse_cut(cand.coeff);
  const std::size_t hash = sparse_cut_hash(sparse);
  PoolCut cut{std::move(sparse), cand.rhs, 0, cand.efficacy, cand.norm, hash};
  cut.source_trace_id = cand.source_trace_id;
  return cut;
}

void xtab_flip_complementation(XTabRow& row, int k) {
  row.complemented[static_cast<std::size_t>(k)] =
      1 - row.complemented[static_cast<std::size_t>(k)];
  row.solval[static_cast<std::size_t>(k)] =
      row.upper[static_cast<std::size_t>(k)] -
      row.solval[static_cast<std::size_t>(k)];
  row.rhs -= row.upper[static_cast<std::size_t>(k)] *
             row.vals[static_cast<std::size_t>(k)];
  row.vals[static_cast<std::size_t>(k)] =
      -row.vals[static_cast<std::size_t>(k)];
}

void xtab_remove_complementation(XTabRow& row) {
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    if (row.complemented[static_cast<std::size_t>(k)] != 0) {
      xtab_flip_complementation(row, k);
    }
  }
}

void xtab_update_violation_and_norm(const XTabRow& row,
                                    int k,
                                    double coeff,
                                    double& violation,
                                    double& norm_sq,
                                    double feastol) {
  violation += coeff * row.solval[static_cast<std::size_t>(k)];
  if (coeff > 0.0 &&
      row.solval[static_cast<std::size_t>(k)] <= feastol) {
    return;
  }
  if (coeff < 0.0 &&
      row.solval[static_cast<std::size_t>(k)] >=
          row.upper[static_cast<std::size_t>(k)] - feastol) {
    return;
  }
  norm_sq += coeff * coeff;
}

bool xtab_is_integral_value(double value, double feastol) {
  return std::abs(value - std::round(value)) <= feastol;
}

bool xtab_determine_cover(const XTabRow& row,
                          double feastol,
                          XTabCutgenRandom* cutgen_random,
                          int fallback_tiebreaker,
                          XTabLiftedCoverState& state) {
  if (row.rhs <= 10.0 * feastol) return false;

  state.cover.clear();
  state.cover.reserve(row.inds.size());
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    if (row.is_integral[static_cast<std::size_t>(k)] == 0) continue;
    if (row.solval[static_cast<std::size_t>(k)] <= feastol) continue;
    state.cover.push_back(k);
  }

  const int max_cover_size = static_cast<int>(state.cover.size());
  int cover_size = 0;
  state.cover_weight = 0.0;
  const int random_tiebreaker =
      cutgen_random != nullptr
          ? cutgen_random->next_tiebreaker(fallback_tiebreaker)
          : fallback_tiebreaker;

  auto ub = [&](int k) { return row.upper[static_cast<std::size_t>(k)]; };
  auto sol = [&](int k) { return row.solval[static_cast<std::size_t>(k)]; };
  auto val = [&](int k) { return row.vals[static_cast<std::size_t>(k)]; };

  auto mid = std::partition(state.cover.begin(), state.cover.end(), [&](int k) {
    return sol(k) >= ub(k) - feastol;
  });
  cover_size = static_cast<int>(mid - state.cover.begin());
  for (int i = 0; i < cover_size; ++i) {
    const int k = state.cover[static_cast<std::size_t>(i)];
    state.cover_weight += val(k) * ub(k);
  }

  std::sort(mid, state.cover.end(), [&](int a, int b) {
    if (ub(a) < 1.5 && ub(b) > 1.5) return true;
    if (ub(a) > 1.5 && ub(b) < 1.5) return false;

    const double contrib_a = sol(a) * val(a);
    const double contrib_b = sol(b) * val(b);
    if (contrib_a > contrib_b + feastol) return true;
    if (contrib_a < contrib_b - feastol) return false;
    if (std::abs(val(a) - val(b)) <= feastol) {
      return xtab_highs_pair_hash(
                 static_cast<std::uint32_t>(row.inds[static_cast<std::size_t>(a)]),
                 static_cast<std::uint32_t>(random_tiebreaker), 0) >
             xtab_highs_pair_hash(
                 static_cast<std::uint32_t>(row.inds[static_cast<std::size_t>(b)]),
                 static_cast<std::uint32_t>(random_tiebreaker), 0);
    }
    return val(a) > val(b);
  });

  const double min_lambda =
      std::max(10.0 * feastol, feastol * std::abs(row.rhs));
  for (; cover_size != max_cover_size; ++cover_size) {
    const double lambda = double(state.cover_weight - row.rhs);
    if (lambda > min_lambda) break;

    const int k = state.cover[static_cast<std::size_t>(cover_size)];
    state.cover_weight += val(k) * ub(k);
  }
  if (cover_size == 0) return false;

  state.cover_weight.renormalize();
  state.lambda = state.cover_weight - row.rhs;
  if (double(state.lambda) <= min_lambda) return false;

  state.cover.resize(static_cast<std::size_t>(cover_size));
  return true;
}

void xtab_separate_lifted_knapsack_cover(XTabRow& row,
                                         const XTabLiftedCoverState& state,
                                         double feastol,
                                         double epsilon) {
  const int cover_size = static_cast<int>(state.cover.size());
  std::vector<int> cover = state.cover;
  std::vector<double> partial(static_cast<std::size_t>(cover_size), 0.0);
  std::vector<signed char> cover_flag(row.inds.size(), 0);

  std::sort(cover.begin(), cover.end(), [&](int a, int b) {
    return row.vals[static_cast<std::size_t>(a)] >
           row.vals[static_cast<std::size_t>(b)];
  });

  HighsCDouble abar_tmp = row.vals[static_cast<std::size_t>(cover.front())];
  HighsCDouble sigma = state.lambda;
  for (int i = 1; i != cover_size; ++i) {
    const HighsCDouble delta =
        abar_tmp - row.vals[static_cast<std::size_t>(cover[i])];
    const HighsCDouble kdelta = static_cast<double>(i) * delta;
    if (double(kdelta) < double(sigma)) {
      abar_tmp = row.vals[static_cast<std::size_t>(cover[i])];
      sigma -= kdelta;
    } else {
      abar_tmp -= sigma * (1.0 / static_cast<double>(i));
      sigma = 0.0;
      break;
    }
  }

  if (double(sigma) > 0.0) {
    abar_tmp = HighsCDouble(row.rhs) / static_cast<double>(cover_size);
  }
  const double abar = double(abar_tmp);

  HighsCDouble sum = 0.0;
  int cplus_size = 0;
  for (int i = 0; i != cover_size; ++i) {
    const int k = cover[static_cast<std::size_t>(i)];
    sum += std::min(abar, row.vals[static_cast<std::size_t>(k)]);
    partial[static_cast<std::size_t>(i)] = double(sum);

    if (row.vals[static_cast<std::size_t>(k)] > abar + feastol) {
      ++cplus_size;
      cover_flag[static_cast<std::size_t>(k)] = 1;
    } else {
      cover_flag[static_cast<std::size_t>(k)] = -1;
    }
  }

  bool half_integral = false;
  auto g = [&](double z) {
    const double hfrac = z / abar;
    double coef = 0.0;

    int h = static_cast<int>(std::floor(hfrac + 0.5));
    if (h != 0 && std::abs(hfrac - static_cast<double>(h)) *
                          std::max(1.0, abar) <= epsilon &&
        h <= cplus_size - 1) {
      half_integral = true;
      coef = 0.5;
    }

    h = std::max(h - 1, 0);
    for (; h < cover_size; ++h) {
      if (z <= partial[static_cast<std::size_t>(h)] + feastol) break;
    }

    return coef + static_cast<double>(h);
  };

  row.rhs = static_cast<double>(cover_size - 1);
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.vals[pos] == 0.0) continue;
    if (cover_flag[pos] == -1) {
      row.vals[pos] = 1.0;
    } else {
      row.vals[pos] = g(row.vals[pos]);
    }
  }

  if (half_integral) {
    row.rhs *= 2.0;
    for (double& v : row.vals) v *= 2.0;
  }

  row.integral_support = true;
  row.integral_coefficients = true;
}

bool xtab_separate_lifted_mixed_binary_cover(
    XTabRow& row,
    const XTabLiftedCoverState& state,
    double epsilon) {
  row.integral_support = false;
  row.integral_coefficients = false;

  const int cover_size = static_cast<int>(state.cover.size());
  if (cover_size == 0) return false;

  std::vector<int> cover = state.cover;
  std::vector<double> partial(static_cast<std::size_t>(cover_size), 0.0);
  std::vector<unsigned char> cover_flag(row.inds.size(), 0);
  for (int k : cover) cover_flag[static_cast<std::size_t>(k)] = 1;

  std::sort(cover.begin(), cover.end(), [&](int a, int b) {
    return row.vals[static_cast<std::size_t>(a)] >
           row.vals[static_cast<std::size_t>(b)];
  });

  double sum = 0.0;
  int p = cover_size;
  for (int i = 0; i != cover_size; ++i) {
    const int k = cover[static_cast<std::size_t>(i)];
    if (row.vals[static_cast<std::size_t>(k)] - double(state.lambda) <=
      epsilon) {
      p = i;
      break;
    }
    sum += row.vals[static_cast<std::size_t>(k)];
    partial[static_cast<std::size_t>(i)] = sum;
  }
  if (p == 0) return false;

  auto phi = [&](double a) {
    for (int i = 0; i < p; ++i) {
      if (a <= partial[static_cast<std::size_t>(i)] - double(state.lambda)) {
        return double(static_cast<double>(i) * state.lambda);
      }

      if (a <= partial[static_cast<std::size_t>(i)]) {
        return double(static_cast<double>(i + 1) * state.lambda +
                      (HighsCDouble(a) - partial[static_cast<std::size_t>(i)]));
      }
    }

    return double(static_cast<double>(p) * state.lambda +
                  (HighsCDouble(a) - partial[static_cast<std::size_t>(p - 1)]));
  };

  HighsCDouble rhs_acc = -state.lambda;
  row.integral_support = true;
  row.integral_coefficients = false;
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.is_integral[pos] == 0) {
      if (row.vals[pos] < 0.0) {
        row.integral_support = false;
      } else {
        row.vals[pos] = 0.0;
      }
      continue;
    }

    if (cover_flag[pos] != 0) {
      row.vals[pos] = std::min(row.vals[pos], double(state.lambda));
      rhs_acc += row.vals[pos];
    } else {
      row.vals[pos] = phi(row.vals[pos]);
    }
  }

  row.rhs = double(rhs_acc);
  return true;
}

bool xtab_separate_lifted_mixed_integer_cover(
    XTabRow& row,
    const XTabLiftedCoverState& state,
    double feastol,
    double epsilon) {
  (void)epsilon;
  row.integral_support = false;
  row.integral_coefficients = false;

  std::vector<int> cover = state.cover;
  const int cover_size = static_cast<int>(cover.size());
  std::vector<unsigned char> cover_flag(row.inds.size(), 0);
  for (int k : cover) cover_flag[static_cast<std::size_t>(k)] = 1;

  std::sort(cover.begin(), cover.end(), [&](int a, int b) {
    return row.vals[static_cast<std::size_t>(a)] >
           row.vals[static_cast<std::size_t>(b)];
  });

  std::vector<double> a(static_cast<std::size_t>(cover_size), 0.0);
  std::vector<double> u(static_cast<std::size_t>(cover_size + 1), 0.0);
  std::vector<double> m(static_cast<std::size_t>(cover_size + 1), 0.0);

  double usum = 0.0;
  double msum = 0.0;
  for (int c = 0; c != cover_size; ++c) {
    const int k = cover[static_cast<std::size_t>(c)];
    u[static_cast<std::size_t>(c)] = usum;
    m[static_cast<std::size_t>(c)] = msum;
    a[static_cast<std::size_t>(c)] = row.vals[static_cast<std::size_t>(k)];
    const double ub = row.upper[static_cast<std::size_t>(k)];
    usum += ub;
    msum += ub * a[static_cast<std::size_t>(c)];
  }
  u[static_cast<std::size_t>(cover_size)] = usum;
  m[static_cast<std::size_t>(cover_size)] = msum;

  int lpos = -1;
  int best_l_cplus_end = -1;
  double best_l_val = 0.0;
  bool best_l_at_upper = true;

  for (int i = 0; i != cover_size; ++i) {
    const int k = cover[static_cast<std::size_t>(i)];
    const double ub = row.upper[static_cast<std::size_t>(k)];

    const bool at_upper =
        row.solval[static_cast<std::size_t>(k)] >= ub - feastol;
    if (at_upper && !best_l_at_upper) continue;

    const double mju = ub * row.vals[static_cast<std::size_t>(k)];
    const double mu = mju - double(state.lambda);

    if (mu <= 10.0 * feastol) continue;
    if (std::abs(row.vals[static_cast<std::size_t>(k)]) <
        1000.0 * feastol) {
      continue;
    }

    const double mudival = mu / row.vals[static_cast<std::size_t>(k)];
    if (xtab_is_integral_value(mudival, feastol)) continue;
    const double eta = std::ceil(mudival);

    const double ul_minus_eta_plus_one = ub - eta + 1.0;
    const double cplus_threshold =
        ul_minus_eta_plus_one * row.vals[static_cast<std::size_t>(k)];

    const int cplus_end = static_cast<int>(
        std::upper_bound(cover.begin(), cover.end(), cplus_threshold,
                         [&](double threshold, int col) {
                           return threshold >
                                  row.vals[static_cast<std::size_t>(col)];
                         }) -
        cover.begin());

    double mcplus = m[static_cast<std::size_t>(cplus_end)];
    if (i < cplus_end) mcplus -= mju;

    const double jl_val =
        mcplus + eta * row.vals[static_cast<std::size_t>(k)];

    if (jl_val > best_l_val || (!at_upper && best_l_at_upper)) {
      lpos = i;
      best_l_cplus_end = cplus_end;
      best_l_val = jl_val;
      best_l_at_upper = at_upper;
    }
  }

  if (lpos == -1) return false;

  const int l = cover[static_cast<std::size_t>(lpos)];
  const double al = row.vals[static_cast<std::size_t>(l)];
  const double upper_l = row.upper[static_cast<std::size_t>(l)];
  const double mlu = upper_l * al;
  const double mu = mlu - double(state.lambda);

  a.resize(static_cast<std::size_t>(best_l_cplus_end));
  cover.resize(static_cast<std::size_t>(best_l_cplus_end));
  u.resize(static_cast<std::size_t>(best_l_cplus_end + 1));
  m.resize(static_cast<std::size_t>(best_l_cplus_end + 1));

  if (lpos < best_l_cplus_end) {
    a.erase(a.begin() + lpos);
    cover.erase(cover.begin() + lpos);
    u.erase(u.begin() + lpos + 1);
    m.erase(m.begin() + lpos + 1);
    for (int i = lpos + 1; i < best_l_cplus_end; ++i) {
      u[static_cast<std::size_t>(i)] -= upper_l;
      m[static_cast<std::size_t>(i)] -= mlu;
    }
  }

  const int cplus_size = static_cast<int>(a.size());
  const double mudival = mu / al;
  const double eta = std::ceil(mudival);
  double r = mu - std::floor(mudival) * al;
  if (r < 0.0) r = 0.0;

  const double ul_minus_eta_plus_one = upper_l - eta + 1.0;
  const double cplus_threshold = ul_minus_eta_plus_one * al;
  const int64_t kmin = static_cast<int64_t>(std::floor(eta - upper_l - 0.5));

  auto phi_l = [&](double value) {
    int64_t k = std::min(static_cast<int64_t>(value / al), int64_t{-1});

    for (; k >= kmin; --k) {
      if (value >= static_cast<double>(k) * al + r) {
        return value - static_cast<double>(k + 1) * r;
      }

      if (value >= static_cast<double>(k) * al) {
        return static_cast<double>(k) * (al - r);
      }
    }

    return static_cast<double>(kmin) * (al - r);
  };

  const int64_t kmax = static_cast<int64_t>(std::floor(upper_l - eta + 0.5));

  auto gamma_l = [&](double z) {
    for (int i = 0; i < cplus_size; ++i) {
      const int col = cover[static_cast<std::size_t>(i)];
      const int upper_i = static_cast<int>(
          row.upper[static_cast<std::size_t>(col)]);

      for (int h = 0; h <= upper_i; ++h) {
        const double mih = m[static_cast<std::size_t>(i)] +
                           static_cast<double>(h) *
                               a[static_cast<std::size_t>(i)];
        const double uih =
            u[static_cast<std::size_t>(i)] + static_cast<double>(h);
        const double mih_plus_delta_i =
            mih + a[static_cast<std::size_t>(i)] - cplus_threshold;
        if (z <= mih_plus_delta_i) {
          return uih * ul_minus_eta_plus_one * (al - r);
        }

        int64_t k =
            static_cast<int64_t>((z - mih_plus_delta_i) / al) - 1;
        for (; k <= kmax; ++k) {
          if (z <= mih_plus_delta_i + static_cast<double>(k) * al + r) {
            return (uih * ul_minus_eta_plus_one + static_cast<double>(k)) *
                   (al - r);
          }

          if (z <= mih_plus_delta_i + static_cast<double>(k + 1) * al) {
            return uih * ul_minus_eta_plus_one * (al - r) + z - mih -
                   a[static_cast<std::size_t>(i)] + cplus_threshold -
                   static_cast<double>(k + 1) * r;
          }
        }
      }
    }

    int64_t p = static_cast<int64_t>(
                    (z - m[static_cast<std::size_t>(cplus_size)]) / al) -
                1;
    for (;; ++p) {
      if (z <= m[static_cast<std::size_t>(cplus_size)] +
                   static_cast<double>(p) * al + r) {
        return (u[static_cast<std::size_t>(cplus_size)] *
                    ul_minus_eta_plus_one +
                static_cast<double>(p)) *
               (al - r);
      }

      if (z <= m[static_cast<std::size_t>(cplus_size)] +
                   static_cast<double>(p + 1) * al) {
        return u[static_cast<std::size_t>(cplus_size)] *
                   ul_minus_eta_plus_one * (al - r) +
               z - m[static_cast<std::size_t>(cplus_size)] -
               static_cast<double>(p + 1) * r;
      }
    }
  };

  row.rhs = (upper_l - eta) * r - double(state.lambda);
  row.integral_support = true;
  row.integral_coefficients = false;
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.vals[pos] == 0.0) continue;
    if (row.is_integral[pos] == 0) {
      if (row.vals[pos] < 0.0) {
        row.integral_support = false;
      } else {
        row.vals[pos] = 0.0;
      }
      continue;
    }

    if (cover_flag[pos] != 0) {
      row.vals[pos] = -phi_l(-row.vals[pos]);
      row.rhs += row.vals[pos] * row.upper[pos];
    } else {
      row.vals[pos] = gamma_l(row.vals[pos]);
    }
  }

  return true;
}

void xtab_erase_positions(XTabRow& row, std::vector<unsigned char>& erase) {
  int write = 0;
  const int len = static_cast<int>(row.inds.size());
  for (int read = 0; read < len; ++read) {
    if (erase[static_cast<std::size_t>(read)] != 0) {
      continue;
    }
    if (write != read) {
      row.inds[static_cast<std::size_t>(write)] =
          row.inds[static_cast<std::size_t>(read)];
      row.vals[static_cast<std::size_t>(write)] =
          row.vals[static_cast<std::size_t>(read)];
      row.upper[static_cast<std::size_t>(write)] =
          row.upper[static_cast<std::size_t>(read)];
      row.solval[static_cast<std::size_t>(write)] =
          row.solval[static_cast<std::size_t>(read)];
      row.bound_type[static_cast<std::size_t>(write)] =
          row.bound_type[static_cast<std::size_t>(read)];
      row.varbound_expr[static_cast<std::size_t>(write)] =
          row.varbound_expr[static_cast<std::size_t>(read)];
      row.complemented[static_cast<std::size_t>(write)] =
          row.complemented[static_cast<std::size_t>(read)];
      row.is_integral[static_cast<std::size_t>(write)] =
          row.is_integral[static_cast<std::size_t>(read)];
    }
    ++write;
  }
  row.inds.resize(static_cast<std::size_t>(write));
  row.vals.resize(static_cast<std::size_t>(write));
  row.upper.resize(static_cast<std::size_t>(write));
  row.solval.resize(static_cast<std::size_t>(write));
  row.bound_type.resize(static_cast<std::size_t>(write));
  row.varbound_expr.resize(static_cast<std::size_t>(write));
  row.complemented.resize(static_cast<std::size_t>(write));
  row.is_integral.resize(static_cast<std::size_t>(write));
}

bool xtab_logical_row_bounds(const SimplexResult& simplex,
                             int row,
                             double& lower,
                             double& upper) {
  const StandardFormLP& sf = simplex.form;
  const int m = static_cast<int>(sf.A_row.rows());
  const int n_std = static_cast<int>(sf.A.cols());
  if (row < 0 || row >= m || row >= sf.b.size()) {
    return false;
  }
  lower = -std::numeric_limits<double>::infinity();
  upper = std::numeric_limits<double>::infinity();
  const double side = xtab_source_row_side(sf, row);
  if (!std::isfinite(side)) {
    return false;
  }

  auto compute_activity_domain = [&]() {
    double min_activity = 0.0;
    double max_activity = 0.0;
    bool min_finite = true;
    bool max_finite = true;
    xtab_visit_source_row_terms(sf, row, [&](int j, double a) {
      if (std::abs(a) <= 1e-12) return;
      const double lb =
          j < sf.lb_shift.size() ? sf.lb_shift[j] : -std::numeric_limits<double>::infinity();
      const double ub =
          j < sf.var_ub.size()
              ? original_space_value(simplex, j, sf.var_ub[j])
              : std::numeric_limits<double>::infinity();
      if (a >= 0.0) {
        if (std::isfinite(lb) && min_finite) {
          min_activity += a * lb;
        } else {
          min_finite = false;
        }
        if (std::isfinite(ub) && max_finite) {
          max_activity += a * ub;
        } else {
          max_finite = false;
        }
      } else {
        if (std::isfinite(ub) && min_finite) {
          min_activity += a * ub;
        } else {
          min_finite = false;
        }
        if (std::isfinite(lb) && max_finite) {
          max_activity += a * lb;
        } else {
          max_finite = false;
        }
      }
    });
    return std::pair<double, double>{
        min_finite ? min_activity : -std::numeric_limits<double>::infinity(),
        max_finite ? max_activity : std::numeric_limits<double>::infinity()};
  };

  const int slack_col =
      row < static_cast<int>(sf.row_to_slack_col.size())
          ? sf.row_to_slack_col[static_cast<std::size_t>(row)]
          : -1;
  const int surplus_col =
      row < static_cast<int>(sf.row_to_surplus_col.size())
          ? sf.row_to_surplus_col[static_cast<std::size_t>(row)]
          : -1;

  if (slack_col >= 0) {
    upper = side;
    if (slack_col < n_std && slack_col < sf.var_ub.size() &&
        std::isfinite(sf.var_ub[slack_col])) {
      const double slack_scale = xtab_col_scale_or_one(sf, slack_col);
      const double width = sf.var_ub[slack_col] * slack_scale;
      if (!std::isfinite(width) || width < -1e-9) {
        return false;
      }
      lower = upper - std::max(0.0, width);
    } else {
      lower = compute_activity_domain().first;
    }
    return std::isfinite(upper);
  }

  if (surplus_col >= 0) {
    lower = side;
    upper = compute_activity_domain().second;
    return std::isfinite(lower);
  }

  lower = side;
  upper = side;
  return std::isfinite(lower);
}

bool xtab_logical_row_is_integer_like(
    const SimplexResult& simplex,
    int row,
    const std::vector<char>* implied_integer_cols) {
  const StandardFormLP& sf = simplex.form;
  if (row < 0 || row >= sf.A_row.rows()) {
    return false;
  }
  double lower = 0.0;
  double upper = 0.0;
  if (!xtab_logical_row_bounds(simplex, row, lower, upper)) {
    return false;
  }
  if (std::isfinite(lower) && !is_integral(lower, 1e-8)) {
    return false;
  }
  if (std::isfinite(upper) && !is_integral(upper, 1e-8)) {
    return false;
  }

  bool has_structural_coeff = false;
  bool row_is_integral = true;
  xtab_visit_source_row_terms(sf, row, [&](int j, double unscaled_coeff) {
    if (!transformed_col_is_integer_like(simplex, j, implied_integer_cols)) {
      row_is_integral = false;
      return;
    }
    if (!std::isfinite(unscaled_coeff) || !is_integral(unscaled_coeff, 1e-8)) {
      row_is_integral = false;
      return;
    }
    has_structural_coeff = true;
  });
  return row_is_integral && has_structural_coeff;
}

XTabSourceContext xtab_build_source_context(
    const SimplexResult& simplex,
    const std::vector<char>* implied_integer_cols) {
  XTabSourceContext ctx;
  const StandardFormLP& sf = simplex.form;
  const int n_orig = sf.n_original;
  const int m = static_cast<int>(sf.A_row.rows());
  if (n_orig <= 0 || m <= 0 || simplex.x_std.size() < n_orig ||
      sf.var_ub.size() < n_orig || sf.lb_shift.size() < n_orig ||
      sf.A_row.cols() < n_orig) {
    return ctx;
  }

  ctx.n_original = n_orig;
  ctx.n_rows = m;
  ctx.dim = n_orig + m;
  ctx.lower = Eigen::VectorXd::Constant(
      ctx.dim, -std::numeric_limits<double>::infinity());
  ctx.upper = Eigen::VectorXd::Constant(
      ctx.dim, std::numeric_limits<double>::infinity());
  ctx.solution = Eigen::VectorXd::Zero(ctx.dim);
  ctx.integral.assign(static_cast<std::size_t>(ctx.dim), 0);

  for (int j = 0; j < n_orig; ++j) {
    ctx.lower[j] = sf.lb_shift[j];
    ctx.upper[j] = original_space_value(simplex, j, sf.var_ub[j]);
    ctx.solution[j] = original_space_value(simplex, j, simplex.x_std[j]);
    if (!std::isfinite(ctx.solution[j])) {
      return XTabSourceContext{};
    }
    ctx.integral[static_cast<std::size_t>(j)] =
        transformed_col_is_integer_like(simplex, j, implied_integer_cols) ? 1
                                                                         : 0;
  }

  for (int row = 0; row < m; ++row) {
    const int col = n_orig + row;
    double lower = 0.0;
    double upper = 0.0;
    if (!xtab_logical_row_bounds(simplex, row, lower, upper)) {
      return XTabSourceContext{};
    }
    ctx.lower[col] = lower;
    ctx.upper[col] = upper;
    double activity = 0.0;
    xtab_visit_source_row_terms(sf, row, [&](int j, double value) {
      activity += value * ctx.solution[j];
    });
    if (!std::isfinite(activity)) {
      return XTabSourceContext{};
    }
    ctx.solution[col] = activity;
    ctx.integral[static_cast<std::size_t>(col)] =
        xtab_logical_row_is_integer_like(simplex, row, implied_integer_cols) ? 1
                                                                            : 0;
  }

  ctx.valid = true;
  return ctx;
}

double xtab_unscaled_source_value(const SimplexResult& simplex,
                                  const XTabSourceContext& source_context,
                                  int source_col) {
  (void)simplex;
  if (source_col < 0 || source_col >= source_context.dim) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return source_context.solution[source_col];
}

bool xtab_basic_source_info(const SimplexResult& simplex,
                            const XTabSourceContext& source_context,
                            int basic_col,
                            int& source_col,
                            double& btran_scale) {
  source_col = -1;
  btran_scale = 1.0;
  if (!source_context.valid || basic_col < 0) {
    return false;
  }
  if (basic_col < source_context.n_original) {
    source_col = basic_col;
    btran_scale = xtab_col_scale_or_one(simplex.form, basic_col);
    return true;
  }
  const int row = standard_form_aux_col_row(simplex.form, basic_col);
  if (row < 0 || row >= source_context.n_rows) {
    return false;
  }
  const double aux_coeff = simplex.form.A_row.coeff(row, basic_col);
  if (!std::isfinite(aux_coeff) || std::abs(aux_coeff) <= 1e-12) {
    return false;
  }
  source_col = source_context.n_original + row;
  // Native BTRAN is performed on the scaled standard-form basis.  HiGHS'
  // transformed-row sources are expressed in the unscaled row-activity model
  //     a_r x - row_activity_r = 0.
  // For a native slack/surplus basic column with scaled row coefficient q,
  // the derivative of the logical row activity with respect to the scaled
  // auxiliary variable is -q / row_scale(row).  The per-row multiplier is
  // mapped back to HiGHS row space later by multiplying each BTRAN entry by
  // that row's row_scale.
  btran_scale = -aux_coeff / xtab_row_scale_or_one(simplex.form, row);
  return true;
}

bool xtab_is_binary_trigger_col(const SimplexResult& simplex,
                                int col,
                                const std::vector<char>* implied_integer_cols) {
  if (col < 0 || col >= simplex.form.n_original ||
      col >= static_cast<int>(simplex.form.var_ub.size()) ||
      col >= static_cast<int>(simplex.form.lb_shift.size()) ||
      col >= static_cast<int>(simplex.form.original_types.size())) {
    return false;
  }
  if (!transformed_col_is_integer_like(simplex, col, implied_integer_cols)) {
    return false;
  }
  const VarType t = simplex.form.original_types[static_cast<std::size_t>(col)];
  const bool declared_binary_like =
      t == VarType::Binary || t == VarType::Integer;
  if (!declared_binary_like) {
    return false;
  }
  const double lb = simplex.form.lb_shift[col];
  const double ub_std = simplex.form.var_ub[col];
  if (!std::isfinite(lb) || !std::isfinite(ub_std)) {
    return false;
  }
  const double ub = original_space_value(simplex, col, ub_std);
  return lb >= -1e-9 && ub <= 1.0 + 1e-9;
}

bool xtab_can_use_varbound_target(const SimplexResult& simplex,
                                  int col,
                                  const std::vector<char>* implied_integer_cols,
                                  bool* integer_target_out = nullptr) {
  if (integer_target_out != nullptr) *integer_target_out = false;
  if (col < 0 || col >= simplex.form.n_original ||
      col >= static_cast<int>(simplex.form.var_ub.size()) ||
      col >= static_cast<int>(simplex.form.lb_shift.size())) {
    return false;
  }
  const double ub_std = simplex.form.var_ub[col];
  const double lb = simplex.form.lb_shift[col];
  if (!std::isfinite(lb) || !std::isfinite(ub_std)) {
    return false;
  }
  const double ub = original_space_value(simplex, col, ub_std);
  if (!std::isfinite(ub)) {
    return false;
  }

  const bool integer_like =
      transformed_col_is_integer_like(simplex, col, implied_integer_cols);
  if (!integer_like) {
    return true;
  }

  // Match HighsTransformedLp::transform(): non-binary integral columns may use
  // VUB/VLB rows when both simple bound slacks are positive.  The selected
  // variable-bound distance is checked per candidate below.
  if (col >= simplex.x_std.size()) {
    return false;
  }
  const double value = original_space_value(simplex, col, simplex.x_std[col]);
  if (!std::isfinite(value)) {
    return false;
  }
  const double simple_lb_dist = std::max(0.0, value - lb);
  const double simple_ub_dist = std::max(0.0, ub - value);
  if (ub - lb <= 1.5 + 1e-9 || simple_lb_dist <= 1e-8 ||
      simple_ub_dist <= 1e-8) {
    return false;
  }
  if (integer_target_out != nullptr) *integer_target_out = true;
  return true;
}

void xtab_update_best_varbound(XTabVarBoundExpr& best,
                               const XTabVarBoundExpr& cand,
                               bool is_lower_bound) {
  if (!cand.valid) return;
  if (!best.valid) {
    best = cand;
    return;
  }
  if (cand.scaled_dist < best.scaled_dist - 1e-12) {
    best = cand;
    return;
  }
  if (std::abs(cand.scaled_dist - best.scaled_dist) > 1e-12) {
    return;
  }

  // Match HighsImplications::getBestVub/getBestVlb: after the LP-distance
  // filter, prefer the globally stronger variable bound.  VUBs are stronger
  // when their minimum possible upper value is smaller; VLBs are stronger when
  // their maximum possible lower value is larger.  The lift tie-break is kept
  // only after this HiGHS-conformant ordering is exhausted.
  if (!is_lower_bound &&
      cand.min_value < best.min_value - 1e-12) {
    best = cand;
    return;
  }
  if (is_lower_bound &&
      cand.max_value > best.max_value + 1e-12) {
    best = cand;
    return;
  }
  if (((!is_lower_bound &&
        std::abs(cand.min_value - best.min_value) <= 1e-12) ||
       (is_lower_bound &&
        std::abs(cand.max_value - best.max_value) <= 1e-12)) &&
      cand.lift > best.lift + 1e-12) {
    best = cand;
  }
}

XTabTransformContext xtab_build_transform_context(
    const SimplexResult& simplex,
    const std::vector<char>* implied_integer_cols,
    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table) {
  XTabTransformContext ctx;
  const int n_orig = simplex.form.n_original;
  const bool have_graph =
      implication_graph != nullptr && !implication_graph->empty();
  const bool have_varbounds =
      variable_bound_table != nullptr && !variable_bound_table->empty();
  if (n_orig <= 0 || (!have_graph && !have_varbounds) ||
      simplex.x_std.size() < n_orig ||
      simplex.form.var_ub.size() < static_cast<Eigen::Index>(n_orig) ||
      simplex.form.lb_shift.size() < static_cast<Eigen::Index>(n_orig)) {
    return ctx;
  }
  ctx.best_vlb.assign(static_cast<std::size_t>(n_orig), XTabVarBoundExpr{});
  ctx.best_vub.assign(static_cast<std::size_t>(n_orig), XTabVarBoundExpr{});
  const int trace_vb_col = xtab_varbound_trace_col();

  auto add_varbound_expr = [&](int target,
                               int trigger,
                               double coef_orig,
                               double constant_orig,
                               bool is_lower_bound,
                               const char* source) {
    const bool trace = target == trace_vb_col;
    auto trace_reject = [&](const char* reason) {
      if (!trace) return;
      fmt::print(stderr,
                 "[B&C-XTAB-VB-CAND] target={} trigger={} sense={} "
                 "source={} reject={} coef={:.17g} constant={:.17g}\n",
                 target, trigger, is_lower_bound ? "vlb" : "vub",
                 source != nullptr ? source : "unknown", reason, coef_orig,
                 constant_orig);
    };
    bool integer_target = false;
    if (trigger < 0 || trigger >= n_orig || target < 0 ||
        target >= n_orig || target == trigger ||
        !std::isfinite(coef_orig) || !std::isfinite(constant_orig)) {
      trace_reject("invalid_input");
      return;
    }
    if (!xtab_is_binary_trigger_col(simplex, trigger, implied_integer_cols)) {
      trace_reject("trigger_not_binary");
      return;
    }
    if (!xtab_can_use_varbound_target(simplex, target,
                                      implied_integer_cols, &integer_target)) {
      trace_reject("target_unusable");
      return;
    }
    const double trigger_ub = simplex.form.var_ub[trigger];
    if (!std::isfinite(trigger_ub) || trigger_ub <= 1e-12) {
      trace_reject("trigger_bad_ub");
      return;
    }
    const double trigger_lb = simplex.form.lb_shift[trigger];
    const double trigger_ub_orig =
        original_space_value(simplex, trigger, trigger_ub);
    const double trigger_val =
        std::clamp(original_space_value(simplex, trigger,
                                        simplex.x_std[trigger]),
                   trigger_lb, trigger_ub_orig);
    if (!std::isfinite(trigger_lb) || !std::isfinite(trigger_ub_orig) ||
        !std::isfinite(trigger_val)) {
      trace_reject("trigger_bad_value");
      return;
    }

    const double target_ub_std = simplex.form.var_ub[target];
    const double target_lb = simplex.form.lb_shift[target];
    if (!std::isfinite(target_lb) || !std::isfinite(target_ub_std)) {
      trace_reject("target_bad_bounds");
      return;
    }
    const double target_ub =
        original_space_value(simplex, target, target_ub_std);
    const double target_val =
        original_space_value(simplex, target, simplex.x_std[target]);
    if (!std::isfinite(target_ub) || !std::isfinite(target_val)) {
      trace_reject("target_bad_value");
      return;
    }
    const double target_range = std::max(1.0, target_ub - target_lb);
    if (!std::isfinite(target_range) || target_range <= 0.0) {
      trace_reject("target_bad_range");
      return;
    }

    const double expr_at_lp = constant_orig + coef_orig * trigger_val;
    double dist = 0.0;
    double y_dist = 0.0;
    if (is_lower_bound) {
      dist = std::max(0.0, target_val - expr_at_lp);
      y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_val - trigger_lb
                                       : trigger_ub_orig - trigger_val);
    } else {
      dist = std::max(0.0, expr_at_lp - target_val);
      y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_ub_orig - trigger_val
                                       : trigger_val - trigger_lb);
    }
    y_dist = std::max(1e-9, y_dist);
    const double norm2 = 1.0 + coef_orig * coef_orig;
    if (dist * dist > y_dist * y_dist * norm2 + 1e-12) {
      trace_reject("bad_vbd_distance");
      return;
    }
    if (integer_target && dist > 1e-7) {
      trace_reject("integer_target_distance");
      return;
    }

    XTabVarBoundExpr cand;
    cand.valid = true;
    cand.trigger_col = trigger;
    cand.constant = constant_orig;
    cand.coef = coef_orig;
    cand.dist = dist;
    cand.scaled_dist = dist / target_range;
    cand.min_value =
        constant_orig + std::min(coef_orig * trigger_lb,
                                 coef_orig * trigger_ub_orig);
    cand.max_value =
        constant_orig + std::max(coef_orig * trigger_lb,
                                 coef_orig * trigger_ub_orig);
    cand.lift = std::abs(coef_orig) * (trigger_ub_orig - trigger_lb);
    cand.integer_target = integer_target;
    if (trace) {
      fmt::print(stderr,
                 "[B&C-XTAB-VB-CAND] target={} trigger={} sense={} "
                 "source={} accept coef={:.17g} constant={:.17g} "
                 "dist={:.17g} scaled={:.17g} min={:.17g} max={:.17g} "
                 "lift={:.17g} integerTarget={} targetVal={:.17g} "
                 "triggerVal={:.17g}\n",
                 target, trigger, is_lower_bound ? "vlb" : "vub",
                 source != nullptr ? source : "unknown", coef_orig,
                 constant_orig, cand.dist, cand.scaled_dist, cand.min_value,
                 cand.max_value, cand.lift, cand.integer_target ? 1 : 0,
                 target_val, trigger_val);
    }
    if (is_lower_bound) {
      XTabVarBoundExpr& best = ctx.best_vlb[static_cast<std::size_t>(target)];
      xtab_update_best_varbound(
          best, cand, true);
      if (trace) {
        fmt::print(stderr,
                   "[B&C-XTAB-VB-BEST] target={} sense=vlb trigger={} "
                   "coef={:.17g} constant={:.17g} dist={:.17g} "
                   "scaled={:.17g} min={:.17g} max={:.17g}\n",
                   target, best.trigger_col, best.coef, best.constant,
                   best.dist, best.scaled_dist, best.min_value,
                   best.max_value);
      }
    } else {
      XTabVarBoundExpr& best = ctx.best_vub[static_cast<std::size_t>(target)];
      xtab_update_best_varbound(
          best, cand, false);
      if (trace) {
        fmt::print(stderr,
                   "[B&C-XTAB-VB-BEST] target={} sense=vub trigger={} "
                   "coef={:.17g} constant={:.17g} dist={:.17g} "
                   "scaled={:.17g} min={:.17g} max={:.17g}\n",
                   target, best.trigger_col, best.coef, best.constant,
                   best.dist, best.scaled_dist, best.min_value,
                   best.max_value);
      }
    }
  };

  if (have_varbounds) {
    const int table_cols =
        std::min(n_orig, variable_bound_table->num_vars());
    for (int target = 0; target < table_cols; ++target) {
      for (const auto& entry : variable_bound_table->vlbs(target)) {
        add_varbound_expr(target, entry.trigger_col, entry.bound.coef,
                          entry.bound.constant, true, "table");
      }
      for (const auto& entry : variable_bound_table->vubs(target)) {
        add_varbound_expr(target, entry.trigger_col, entry.bound.coef,
                          entry.bound.constant, false, "table");
      }
    }
  }

  if (!have_graph) {
    for (const auto& vb : ctx.best_vlb) {
      if (vb.valid) {
        ++ctx.num_vlb;
        if (vb.integer_target) ++ctx.num_integral_vlb;
      }
    }
    for (const auto& vb : ctx.best_vub) {
      if (vb.valid) {
        ++ctx.num_vub;
        if (vb.integer_target) ++ctx.num_integral_vub;
      }
    }
    return ctx;
  }

  for (int trigger = 0; trigger < n_orig; ++trigger) {
        if (!xtab_is_binary_trigger_col(simplex, trigger, implied_integer_cols)) {
      continue;
    }
    const double trigger_ub = simplex.form.var_ub[trigger];
    if (!std::isfinite(trigger_ub) || trigger_ub <= 1e-12) {
      continue;
    }
    const double trigger_lb = simplex.form.lb_shift[trigger];
    const double trigger_ub_orig =
        original_space_value(simplex, trigger, trigger_ub);
    const double trigger_val =
        std::clamp(original_space_value(simplex, trigger,
                                        simplex.x_std[trigger]),
                   trigger_lb, trigger_ub_orig);
    if (!std::isfinite(trigger_lb) || !std::isfinite(trigger_ub_orig) ||
        !std::isfinite(trigger_val)) {
      continue;
    }

    for (bool trigger_one : {false, true}) {
      const auto range = implication_graph->implications(trigger, trigger_one);
      for (const auto* p = range.first; p != range.second; ++p) {
        if (p == nullptr || p->var_idx < 0 || p->var_idx >= n_orig ||
            p->var_idx == trigger || !std::isfinite(p->value) ||
            !xtab_can_use_varbound_target(simplex, p->var_idx,
                                          implied_integer_cols)) {
          continue;
        }

        const int target = p->var_idx;
        const double target_lb = simplex.form.lb_shift[target];
        const double target_ub_std = simplex.form.var_ub[target];
        if (!std::isfinite(target_lb) || !std::isfinite(target_ub_std)) {
          continue;
        }
        const double target_ub =
            original_space_value(simplex, target, target_ub_std);
        if (!std::isfinite(target_ub)) {
          continue;
        }
        const double target_range =
            std::max(1.0, target_ub - target_lb);
        const double target_val =
            original_space_value(simplex, target, simplex.x_std[target]);
        if (!std::isfinite(target_range) || target_range <= 0.0 ||
            !std::isfinite(target_val)) {
          continue;
        }

        // x_target_orig >=/<= constant_orig + coef_orig * y_trigger_orig.
        double constant_orig = 0.0;
        double coef_orig = 0.0;
        double improvement = 0.0;
        bool integer_target = false;
        if (!xtab_can_use_varbound_target(simplex, target,
                                          implied_integer_cols,
                                          &integer_target)) {
          continue;
        }

        if (p->is_lb) {
          if (p->value <= target_lb + 1e-9) continue;
          improvement = p->value - target_lb;
          if (trigger_one) {
            constant_orig = target_lb;
            coef_orig = improvement;
          } else {
            constant_orig = p->value;
            coef_orig = -improvement;
          }
        } else {
          if (p->value >= target_ub - 1e-9) continue;
          improvement = target_ub - p->value;
          if (trigger_one) {
            constant_orig = target_ub;
            coef_orig = -improvement;
          } else {
            constant_orig = p->value;
            coef_orig = improvement;
          }
        }
        if (!(improvement > 1e-12)) continue;

        const double expr_at_lp = constant_orig + coef_orig * trigger_val;
        double dist = 0.0;
        double y_dist = 0.0;
        if (p->is_lb) {
          dist = std::max(0.0, target_val - expr_at_lp);
          y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_val - trigger_lb
                                           : trigger_ub_orig - trigger_val);
        } else {
          dist = std::max(0.0, expr_at_lp - target_val);
          y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_ub_orig - trigger_val
                                           : trigger_val - trigger_lb);
        }
        y_dist = std::max(1e-9, y_dist);
        const double norm2 = 1.0 + coef_orig * coef_orig;
        if (dist * dist > y_dist * y_dist * norm2 + 1e-12) {
          continue;
        }
        if (integer_target && dist > 1e-7) {
          continue;
        }

        XTabVarBoundExpr cand;
        cand.valid = true;
        cand.trigger_col = trigger;
        cand.constant = constant_orig;
        cand.coef = coef_orig;
        cand.dist = dist;
        cand.scaled_dist = dist / target_range;
        cand.min_value =
            constant_orig + std::min(coef_orig * trigger_lb,
                                     coef_orig * trigger_ub_orig);
        cand.max_value =
            constant_orig + std::max(coef_orig * trigger_lb,
                                     coef_orig * trigger_ub_orig);
        cand.lift = std::abs(coef_orig) * (trigger_ub_orig - trigger_lb);
        cand.integer_target = integer_target;
        if (p->is_lb) {
          xtab_update_best_varbound(
              ctx.best_vlb[static_cast<std::size_t>(target)], cand, true);
        } else {
          xtab_update_best_varbound(
              ctx.best_vub[static_cast<std::size_t>(target)], cand, false);
        }
        if (target == trace_vb_col) {
          const XTabVarBoundExpr& best =
              p->is_lb ? ctx.best_vlb[static_cast<std::size_t>(target)]
                       : ctx.best_vub[static_cast<std::size_t>(target)];
          fmt::print(stderr,
                     "[B&C-XTAB-VB-BEST] target={} sense={} trigger={} "
                     "source=graph coef={:.17g} constant={:.17g} "
                     "dist={:.17g} scaled={:.17g} min={:.17g} max={:.17g}\n",
                     target, p->is_lb ? "vlb" : "vub", best.trigger_col,
                     best.coef, best.constant, best.dist, best.scaled_dist,
                     best.min_value, best.max_value);
        }
      }
    }
  }
  if (trace_vb_col >= 0 && trace_vb_col < n_orig) {
    const XTabVarBoundExpr& vlb =
        ctx.best_vlb[static_cast<std::size_t>(trace_vb_col)];
    const XTabVarBoundExpr& vub =
        ctx.best_vub[static_cast<std::size_t>(trace_vb_col)];
    fmt::print(stderr,
               "[B&C-XTAB-VB-FINAL] target={} vlbValid={} vlbTrigger={} "
               "vlbCoef={:.17g} vlbConst={:.17g} vlbDist={:.17g} "
               "vubValid={} vubTrigger={} vubCoef={:.17g} "
               "vubConst={:.17g} vubDist={:.17g}\n",
               trace_vb_col, vlb.valid ? 1 : 0, vlb.trigger_col, vlb.coef,
               vlb.constant, vlb.dist, vub.valid ? 1 : 0, vub.trigger_col,
               vub.coef, vub.constant, vub.dist);
  }
  for (const auto& vb : ctx.best_vlb) {
    if (vb.valid) {
      ++ctx.num_vlb;
      if (vb.integer_target) ++ctx.num_integral_vlb;
    }
  }
  for (const auto& vb : ctx.best_vub) {
    if (vb.valid) {
      ++ctx.num_vub;
      if (vb.integer_target) ++ctx.num_integral_vub;
    }
  }
  return ctx;
}

bool xtab_transform_base_row(
    const SimplexResult& simplex,
    const XTabSourceContext& source_context,
    const Eigen::VectorXd& coeff_std,
    double rhs_std,
    const std::vector<int>* active_cols,
    const std::vector<char>* implied_integer_cols,
    const XTabTransformContext* transform_context,
    std::uint64_t* vb_substitutions_out,
    std::uint64_t* vb_trigger_terms_out,
    bool* integers_positive_out,
    bool initial_integers_positive,
    double feastol,
    XTabRow& row) {
  (void)simplex;
  feastol = std::max(0.0, feastol);
  if (vb_substitutions_out != nullptr) *vb_substitutions_out = 0;
  if (vb_trigger_terms_out != nullptr) *vb_trigger_terms_out = 0;
  if (integers_positive_out != nullptr) *integers_positive_out = true;
  const int n_src = source_context.dim;
  const int n_orig = source_context.n_original;
  if (!source_context.valid || coeff_std.size() != n_src ||
      source_context.lower.size() != n_src ||
      source_context.upper.size() != n_src ||
      source_context.solution.size() != n_src ||
      static_cast<int>(source_context.integral.size()) != n_src) {
    return false;
  }
  (void)implied_integer_cols;
  row = XTabRow{};
  HighsCDouble rhs_acc = rhs_std;
  row.inds.reserve(static_cast<std::size_t>(std::min(n_src, 1024)));
  row.vals.reserve(row.inds.capacity());
  row.upper.reserve(row.inds.capacity());
  row.solval.reserve(row.inds.capacity());
  row.bound_type.reserve(row.inds.capacity());
  row.varbound_expr.reserve(row.inds.capacity());
  row.complemented.reserve(row.inds.capacity());
  row.is_integral.reserve(row.inds.capacity());

  auto choose_bound_type =
      [&](int col,
          double coeff,
          XTabVarBoundExpr& chosen_varbound) -> XTabBoundType {
    chosen_varbound = XTabVarBoundExpr{};
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    const double simple_lb_dist =
        std::isfinite(lb) ? std::max(0.0, sol - lb)
                          : std::numeric_limits<double>::infinity();
    const double simple_ub_dist =
        std::isfinite(ub) ? std::max(0.0, ub - sol)
                          : std::numeric_limits<double>::infinity();

    const XTabVarBoundExpr* vlb = nullptr;
    const XTabVarBoundExpr* vub = nullptr;
    if (transform_context != nullptr && col < n_orig) {
      const std::size_t pos = static_cast<std::size_t>(col);
      if (pos < transform_context->best_vlb.size() &&
          transform_context->best_vlb[pos].valid) {
        vlb = &transform_context->best_vlb[pos];
      }
      if (pos < transform_context->best_vub.size() &&
          transform_context->best_vub[pos].valid) {
        vub = &transform_context->best_vub[pos];
      }
    }

    const double lb_dist = vlb != nullptr ? vlb->dist : simple_lb_dist;
    const double ub_dist = vub != nullptr ? vub->dist : simple_ub_dist;
    const bool integer_like =
        source_context.integral[static_cast<std::size_t>(col)] != 0;
    const double tol = feastol;

    auto simple_type = [&]() {
      if (!std::isfinite(simple_lb_dist)) {
        return XTabBoundType::SimpleUb;
      }
      if (!std::isfinite(simple_ub_dist)) {
        return XTabBoundType::SimpleLb;
      }
      if (simple_lb_dist < simple_ub_dist - tol) {
        return XTabBoundType::SimpleLb;
      }
      if (simple_ub_dist < simple_lb_dist - tol) {
        return XTabBoundType::SimpleUb;
      }
      return coeff > 0.0 ? XTabBoundType::SimpleLb
                         : XTabBoundType::SimpleUb;
    };

    if (integer_like) {
      const double simple_bound_dist =
          std::min(simple_lb_dist, simple_ub_dist);
      const double bound_dist = std::min(lb_dist, ub_dist);
      const bool can_use_integer_vbd =
          col < n_orig && std::isfinite(ub) &&
          ub > 1.5 + 1e-9 && simple_lb_dist > tol &&
          simple_ub_dist > tol && (vlb != nullptr || vub != nullptr) &&
          bound_dist <= tol &&
          bound_dist <= simple_bound_dist + tol;
      if (!can_use_integer_vbd) {
        return simple_type();
      }
      if ((vlb == nullptr && vub != nullptr) ||
          (vub != nullptr && ub_dist < lb_dist - tol)) {
        chosen_varbound = *vub;
        return XTabBoundType::VariableUb;
      }
      if ((vub == nullptr && vlb != nullptr) ||
          (vlb != nullptr && lb_dist < ub_dist - tol)) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      if (coeff > 0.0 && vub != nullptr) {
        chosen_varbound = *vub;
        return XTabBoundType::VariableUb;
      }
      if (vlb != nullptr) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      return simple_type();
    }

    if (lb_dist < ub_dist - tol) {
      if (vlb != nullptr &&
          (coeff > 0.0 || simple_lb_dist > lb_dist + tol)) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      return XTabBoundType::SimpleLb;
    }
    if (ub_dist < lb_dist - tol) {
      if (vub != nullptr &&
          (coeff < 0.0 || simple_ub_dist > ub_dist + tol)) {
        chosen_varbound = *vub;
        return XTabBoundType::VariableUb;
      }
      return XTabBoundType::SimpleUb;
    }
    if (coeff > 0.0) {
      if (vlb != nullptr) {
        chosen_varbound = *vlb;
        return XTabBoundType::VariableLb;
      }
      return XTabBoundType::SimpleLb;
    }
    if (vub != nullptr) {
      chosen_varbound = *vub;
      return XTabBoundType::VariableUb;
    }
    return XTabBoundType::SimpleUb;
  };

  std::vector<int> inds;
  std::vector<double> vals;
  inds.reserve(active_cols != nullptr ? active_cols->size()
                                      : static_cast<std::size_t>(n_src));
  vals.reserve(inds.capacity());
  auto append_input_term = [&](int j) -> bool {
    if (j < 0 || j >= n_src) {
      return false;
    }
    const double a = coeff_std[j];
    if (!std::isfinite(a)) {
      return false;
    }
    if (std::abs(a) > 1e-12) {
      inds.push_back(j);
      vals.push_back(a);
    }
    return true;
  };

  if (active_cols != nullptr) {
    for (int j : *active_cols) {
      if (!append_input_term(j)) {
        return false;
      }
    }
  } else {
    for (int j = 0; j < n_src; ++j) {
      if (!append_input_term(j)) {
        return false;
      }
    }
  }
  int num_nz = static_cast<int>(inds.size());
  if (num_nz == 0) return false;

  std::vector<XTabBoundType> bound_types(static_cast<std::size_t>(n_src),
                                         XTabBoundType::SimpleLb);
  std::vector<XTabVarBoundExpr> bound_varbounds(static_cast<std::size_t>(n_src));
  XTabSparseVectorSum vector_sum(n_src);

  auto remove_term = [&](int pos) {
    --num_nz;
    inds[static_cast<std::size_t>(pos)] =
        inds[static_cast<std::size_t>(num_nz)];
    vals[static_cast<std::size_t>(pos)] =
        vals[static_cast<std::size_t>(num_nz)];
    inds[static_cast<std::size_t>(num_nz)] = 0;
    vals[static_cast<std::size_t>(num_nz)] = 0.0;
  };

  auto simple_lb_dist = [&](int col) {
    const double lb = source_context.lower[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(lb)) return std::numeric_limits<double>::infinity();
    const double dist = sol - lb;
    return dist <= feastol ? 0.0 : std::max(0.0, dist);
  };
  auto simple_ub_dist = [&](int col) {
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(ub)) return std::numeric_limits<double>::infinity();
    const double dist = ub - sol;
    return dist <= feastol ? 0.0 : std::max(0.0, dist);
  };

  for (int i = 0; i < num_nz;) {
    const int col = inds[static_cast<std::size_t>(i)];
    double& val = vals[static_cast<std::size_t>(i)];
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(sol) ||
        (!std::isfinite(lb) && !std::isfinite(ub))) {
      return false;
    }
    if (std::isfinite(lb) && std::isfinite(ub) && ub - lb < 1e-9) {
      rhs_acc -= std::min(lb, ub) * val;
      remove_term(i);
      continue;
    }

    XTabVarBoundExpr chosen_varbound;
    const bool integer_like =
        source_context.integral[static_cast<std::size_t>(col)] != 0;
    if (integer_like) {
      const double slb = simple_lb_dist(col);
      const double sub = simple_ub_dist(col);
      const double simple_bound_dist = std::min(slb, sub);
      const bool have_vlb =
          transform_context != nullptr && col < n_orig &&
          static_cast<std::size_t>(col) < transform_context->best_vlb.size() &&
          transform_context->best_vlb[static_cast<std::size_t>(col)].valid;
      const bool have_vub =
          transform_context != nullptr && col < n_orig &&
          static_cast<std::size_t>(col) < transform_context->best_vub.size() &&
          transform_context->best_vub[static_cast<std::size_t>(col)].valid;
      double bdist = simple_bound_dist;
      if (have_vlb) {
        bdist = std::min(
            bdist, transform_context->best_vlb[static_cast<std::size_t>(col)].dist);
      }
      if (have_vub) {
        bdist = std::min(
            bdist, transform_context->best_vub[static_cast<std::size_t>(col)].dist);
      }
      XTabBoundType type = XTabBoundType::SimpleLb;
      const bool use_simple =
          (std::isfinite(ub) && std::isfinite(lb) && ub - lb <= 1.5 + 1e-9) ||
          bdist != 0.0 || slb == 0.0 || sub == 0.0;
      if (use_simple) {
        if (slb < sub - feastol) {
          type = XTabBoundType::SimpleLb;
        } else if (sub < slb - feastol) {
          type = XTabBoundType::SimpleUb;
        } else if (val > 0.0) {
          type = XTabBoundType::SimpleLb;
        } else {
          type = XTabBoundType::SimpleUb;
        }
        bound_types[static_cast<std::size_t>(col)] = type;
        ++i;
        continue;
      }
    }

    const XTabBoundType bound_type = choose_bound_type(col, val, chosen_varbound);
    bound_types[static_cast<std::size_t>(col)] = bound_type;
    if (chosen_varbound.valid) {
      bound_varbounds[static_cast<std::size_t>(col)] = chosen_varbound;
    }

    switch (bound_type) {
      case XTabBoundType::SimpleLb:
        if (!std::isfinite(lb)) return false;
        if (val > 0.0) {
          rhs_acc -= lb * val;
          remove_term(i);
          continue;
        }
        break;
      case XTabBoundType::SimpleUb:
        if (!std::isfinite(ub)) return false;
        if (val < 0.0) {
          rhs_acc -= ub * val;
          remove_term(i);
          continue;
        }
        break;
      case XTabBoundType::VariableLb:
        if (!chosen_varbound.valid) return false;
        if (vb_substitutions_out != nullptr) ++(*vb_substitutions_out);
        rhs_acc -= chosen_varbound.constant * val;
        if (chosen_varbound.trigger_col >= 0 &&
            chosen_varbound.trigger_col < n_orig &&
            std::abs(val * chosen_varbound.coef) > 1e-12) {
          if (chosen_varbound.trigger_col == xtab_transform_trace_col()) {
            fmt::print(stderr,
                       "[B&C-TRANSFORM-TERM] target={} source={} type=vlb "
                       "val={:.17g} coef={:.17g} constant={:.17g} "
                       "contrib={:.17g} dist={:.17g}\n",
                       chosen_varbound.trigger_col, col, val,
                       chosen_varbound.coef, chosen_varbound.constant,
                       val * chosen_varbound.coef, chosen_varbound.dist);
          }
          vector_sum.add(chosen_varbound.trigger_col,
                         val * chosen_varbound.coef);
          if (vb_trigger_terms_out != nullptr) ++(*vb_trigger_terms_out);
        }
        if (val > 0.0) {
          remove_term(i);
          continue;
        }
        break;
      case XTabBoundType::VariableUb:
        if (!chosen_varbound.valid) return false;
        if (vb_substitutions_out != nullptr) ++(*vb_substitutions_out);
        rhs_acc -= chosen_varbound.constant * val;
        if (chosen_varbound.trigger_col >= 0 &&
            chosen_varbound.trigger_col < n_orig &&
            std::abs(val * chosen_varbound.coef) > 1e-12) {
          if (chosen_varbound.trigger_col == xtab_transform_trace_col()) {
            fmt::print(stderr,
                       "[B&C-TRANSFORM-TERM] target={} source={} type=vub "
                       "val={:.17g} coef={:.17g} constant={:.17g} "
                       "contrib={:.17g} dist={:.17g}\n",
                       chosen_varbound.trigger_col, col, val,
                       chosen_varbound.coef, chosen_varbound.constant,
                       val * chosen_varbound.coef, chosen_varbound.dist);
          }
          vector_sum.add(chosen_varbound.trigger_col,
                         val * chosen_varbound.coef);
          if (vb_trigger_terms_out != nullptr) ++(*vb_trigger_terms_out);
        }
        val = -val;
        if (val > 0.0) {
          remove_term(i);
          continue;
        }
        break;
    }
    ++i;
  }

  if (!vector_sum.empty()) {
    for (int i = 0; i < num_nz; ++i) {
      if (vals[static_cast<std::size_t>(i)] != 0.0) {
        vector_sum.add(inds[static_cast<std::size_t>(i)],
                       vals[static_cast<std::size_t>(i)]);
      }
    }
    vector_sum.cleanup([](int, double value) {
      return std::abs(value) <= 1e-9;
    });
    inds = vector_sum.nonzeros;
    num_nz = static_cast<int>(inds.size());
    vals.resize(static_cast<std::size_t>(num_nz));
    for (int j = 0; j < num_nz; ++j) {
      vals[static_cast<std::size_t>(j)] =
          vector_sum.values[static_cast<std::size_t>(inds[static_cast<std::size_t>(j)])];
    }
  } else {
    inds.resize(static_cast<std::size_t>(num_nz));
    vals.resize(static_cast<std::size_t>(num_nz));
  }

  bool integers_positive = initial_integers_positive;
  for (int j = 0; j < num_nz; ++j) {
    const int col = inds[static_cast<std::size_t>(j)];
    if (col < 0 || col >= n_src) return false;
    if (source_context.integral[static_cast<std::size_t>(col)] == 0) continue;
    if (bound_types[static_cast<std::size_t>(col)] ==
            XTabBoundType::VariableLb ||
        bound_types[static_cast<std::size_t>(col)] ==
            XTabBoundType::VariableUb) {
      continue;
    }
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double val = vals[static_cast<std::size_t>(j)];
    if (initial_integers_positive) {
      if ((std::isfinite(lb) && val > 0.0) || !std::isfinite(ub)) {
        bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleLb;
      } else {
        bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleUb;
      }
    } else if (simple_lb_dist(col) < simple_ub_dist(col)) {
      bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleLb;
    } else {
      bound_types[static_cast<std::size_t>(col)] = XTabBoundType::SimpleUb;
    }
  }

  for (int j = 0; j < num_nz; ++j) {
    const int col = inds[static_cast<std::size_t>(j)];
    double transformed_coeff = vals[static_cast<std::size_t>(j)];
    if (std::abs(transformed_coeff) <= 1e-12) continue;
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    const double sol = source_context.solution[col];
    if (!std::isfinite(sol)) return false;
    XTabBoundType bound_type = bound_types[static_cast<std::size_t>(col)];
    const XTabVarBoundExpr& varbound =
        bound_varbounds[static_cast<std::size_t>(col)];
    double upper_range = std::numeric_limits<double>::infinity();
    if (std::isfinite(lb) && std::isfinite(ub)) {
      upper_range = std::max(0.0, ub - lb);
    }
    auto clipped_dist = [feastol](double dist) {
      if (dist <= feastol) return 0.0;
      return std::max(0.0, dist);
    };
    double solval = std::numeric_limits<double>::infinity();
    switch (bound_type) {
      case XTabBoundType::SimpleLb:
        if (!std::isfinite(lb)) {
          return false;
        }
        rhs_acc -= lb * transformed_coeff;
        solval = clipped_dist(sol - lb);
        break;
      case XTabBoundType::SimpleUb:
        if (!std::isfinite(ub)) {
          return false;
        }
        rhs_acc -= ub * transformed_coeff;
        transformed_coeff = -transformed_coeff;
        solval = clipped_dist(ub - sol);
        break;
      case XTabBoundType::VariableLb:
        if (!varbound.valid) {
          return false;
        }
        solval = clipped_dist(varbound.dist);
        break;
      case XTabBoundType::VariableUb:
        if (!varbound.valid) {
          return false;
        }
        solval = clipped_dist(varbound.dist);
        break;
    }
    if (std::abs(transformed_coeff) <= 1e-12) {
      continue;
    }
    const bool integer_like =
        source_context.integral[static_cast<std::size_t>(col)] != 0;
    row.inds.push_back(col);
    row.vals.push_back(transformed_coeff);
    row.bound_type.push_back(bound_type);
    row.varbound_expr.push_back(varbound);
    row.solval.push_back(solval);
    row.upper.push_back(upper_range);
    row.complemented.push_back(0);
    row.is_integral.push_back(integer_like ? 1 : 0);
    if (integer_like && transformed_coeff <= 0.0) {
      integers_positive = false;
    }
  }
  if (integers_positive_out != nullptr) {
    *integers_positive_out = integers_positive;
  }
  row.rhs = double(rhs_acc);
  return !row.inds.empty() && std::isfinite(row.rhs);
}

bool xtab_preprocess_base_inequality(const SimplexResult& simplex,
                                     XTabRow& row,
                                     double feastol,
                                     bool& has_unbounded_ints,
                                     bool& has_general_ints,
                                     bool& has_continuous) {
  feastol = std::max(0.0, feastol);
  has_unbounded_ints = false;
  has_general_ints = false;
  has_continuous = false;
  if (row.inds.empty()) {
    return false;
  }

  double max_abs = 0.0;
  for (double v : row.vals) {
    max_abs = std::max(max_abs, std::abs(v));
  }
  row.initial_scale = pow2_scale_for_max_abs(max_abs);
  row.rhs *= row.initial_scale;
  for (double& v : row.vals) {
    v *= row.initial_scale;
  }

  double maxact = -feastol;
  std::vector<unsigned char> erase(row.inds.size(), 0);
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    row.is_integral[pos] =
        row.is_integral[pos] != 0 && std::abs(row.vals[pos]) > 10.0 * feastol;
    if (row.is_integral[pos] == 0) {
      if (std::isfinite(row.upper[pos]) &&
          row.upper[pos] < 2.0 * row.solval[pos]) {
        xtab_flip_complementation(row, k);
      }
      if (row.vals[pos] > 0.0 ||
          (std::isfinite(row.upper[pos]) &&
           std::abs(row.vals[pos]) * row.upper[pos] <= 10.0 * feastol)) {
        if (row.vals[pos] < 0.0) {
          if (!std::isfinite(row.upper[pos])) {
            return false;
          }
          row.rhs -= row.vals[pos] * row.upper[pos];
        }
        erase[pos] = 1;
        continue;
      }
      has_continuous = true;
      if (row.vals[pos] > 0.0) {
        if (!std::isfinite(row.upper[pos])) {
          maxact = std::numeric_limits<double>::infinity();
        } else {
          maxact += row.vals[pos] * row.upper[pos];
        }
      }
    } else {
      if (!std::isfinite(row.upper[pos])) {
        has_unbounded_ints = true;
        has_general_ints = true;
      } else if (std::abs(row.upper[pos] - 1.0) > 1e-9) {
        has_general_ints = true;
      }
      if (row.vals[pos] > 0.0) {
        if (!std::isfinite(row.upper[pos])) {
          maxact = std::numeric_limits<double>::infinity();
        } else {
          maxact += row.vals[pos] * row.upper[pos];
        }
      }
    }
  }

  int num_erase = static_cast<int>(
      std::count(erase.begin(), erase.end(), static_cast<unsigned char>(1)));
  const int max_len = 100 + static_cast<int>(
      0.15 * static_cast<double>(std::max(1, simplex.form.n_original)));
  const int live_len = static_cast<int>(row.inds.size()) - num_erase;
  if (live_len > max_len) {
    const int need_cancel = live_len - max_len;
    std::vector<int> cancellable;
    cancellable.reserve(static_cast<std::size_t>(need_cancel));
    for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
      if (erase[static_cast<std::size_t>(k)] != 0) {
        continue;
      }
      const double cancel_slack =
          row.vals[static_cast<std::size_t>(k)] > 0.0
              ? row.solval[static_cast<std::size_t>(k)]
              : row.upper[static_cast<std::size_t>(k)] -
                    row.solval[static_cast<std::size_t>(k)];
      if (cancel_slack <= feastol) {
        cancellable.push_back(k);
      }
    }
    if (static_cast<int>(cancellable.size()) < need_cancel) {
      return false;
    }
    std::partial_sort(
        cancellable.begin(), cancellable.begin() + need_cancel,
        cancellable.end(), [&](int a, int b) {
          return std::abs(row.vals[static_cast<std::size_t>(a)]) <
                 std::abs(row.vals[static_cast<std::size_t>(b)]);
        });
    for (int idx = 0; idx < need_cancel; ++idx) {
      const int k = cancellable[static_cast<std::size_t>(idx)];
      const std::size_t pos = static_cast<std::size_t>(k);
      if (row.vals[pos] < 0.0) {
        row.rhs -= row.vals[pos] * row.upper[pos];
      } else if (std::isfinite(row.upper[pos])) {
        maxact -= row.vals[pos] * row.upper[pos];
      }
      erase[pos] = 1;
    }
  }

  if (num_erase > 0 ||
      std::count(erase.begin(), erase.end(), static_cast<unsigned char>(1)) >
          num_erase) {
    xtab_erase_positions(row, erase);
  }
  return !row.inds.empty() && maxact > row.rhs;
}

bool xtab_cmir_cut_generation(XTabRow& row,
                              double feastol,
                              double min_efficacy,
                              XTabCmirTrace* trace = nullptr,
                              bool only_initial_cmir_scale = false,
                              std::uint64_t trace_id = 0,
                              const char* trace_family = nullptr) {
  feastol = std::max(0.0, feastol);
  constexpr double tiny = 1e-14;
  constexpr double f0_min = 0.005;
  constexpr double f0_max = 0.995;
  constexpr double max_cmir_scale = 1e6;
  if (row.inds.empty()) {
    return false;
  }
  if (trace != nullptr) {
    *trace = XTabCmirTrace{};
  }
  row.integral_support = false;
  row.integral_coefficients = false;

  std::vector<int> integer_inds;
  std::vector<double> deltas;
  integer_inds.reserve(row.inds.size());
  deltas.reserve(row.inds.size() + 2);
  double continuous_contribution = 0.0;
  double continuous_norm_sq = 0.0;
  double max_abs_delta = 0.0;

  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.is_integral[pos] != 0) {
      integer_inds.push_back(k);
      if (std::isfinite(row.upper[pos]) &&
          row.upper[pos] < 2.0 * row.solval[pos]) {
        xtab_flip_complementation(row, k);
      }
      if (only_initial_cmir_scale) {
        continue;
      }
      if (row.solval[pos] > feastol) {
        const double delta = std::abs(row.vals[pos]);
        if (delta > 1e-4 && delta != max_abs_delta) {
          max_abs_delta = std::max(max_abs_delta, delta);
          deltas.push_back(delta);
        }
      }
    } else {
      xtab_update_violation_and_norm(row, k, row.vals[pos],
                                     continuous_contribution,
                                     continuous_norm_sq, feastol);
    }
  }
  if (trace != nullptr) {
    trace->integer_terms = static_cast<int>(integer_inds.size());
    trace->continuous_terms =
        static_cast<int>(row.inds.size()) - trace->integer_terms;
    trace->continuous_contribution = continuous_contribution;
    trace->continuous_norm_sq = continuous_norm_sq;
    trace->max_abs_delta = max_abs_delta;
  }

  if (continuous_norm_sq == 0.0 && deltas.size() > 1) {
    const double int_scale = xtab_integral_scale(deltas, feastol, tiny);
    if (int_scale != 0.0 && int_scale <= 1e4) {
      const double scaled_rhs = row.rhs * int_scale;
      const double down_rhs = fast_floor_xtab(scaled_rhs);
      const double f0 = scaled_rhs - down_rhs;
      if (f0 >= f0_min && f0 <= f0_max) {
        deltas.push_back(1.0 / int_scale);
      }
    }
  }

  deltas.push_back(std::min(1.0, row.initial_scale));
  if (!only_initial_cmir_scale) {
    deltas.push_back(max_abs_delta + std::min(1.0, row.initial_scale));
  }
  std::sort(deltas.begin(), deltas.end());
  double cur_delta = deltas.empty() ? 0.0 : deltas.front();
  for (std::size_t i = 1; i < deltas.size(); ++i) {
    if (deltas[i] - cur_delta <= 10.0 * feastol) {
      deltas[i] = 0.0;
    } else {
      cur_delta = deltas[i];
    }
  }
  deltas.erase(std::remove(deltas.begin(), deltas.end(), 0.0), deltas.end());
  if (trace != nullptr) {
    trace->initial_delta_count = static_cast<int>(deltas.size());
  }
  const bool delta_trace = trace_id != 0 && xtab_cmir_delta_trace_enabled();
  if (delta_trace) {
    fmt::memory_buffer buffer;
    for (std::size_t i = 0; i < deltas.size(); ++i) {
      if (i > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{:.17g}", deltas[i]);
    }
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=cmir_deltas feastol={:.17g} "
               "initialScale={:.17g} deltas=[{}]\n",
               trace_id, trace_family == nullptr ? "" : trace_family,
               feastol, row.initial_scale, fmt::to_string(buffer));
  }

  double best_delta = -1.0;
  double best_efficacy = min_efficacy;
  auto evaluate_delta = [&](const char* phase, double delta) -> double {
    if (trace != nullptr) ++trace->tested_delta_count;
    if (!(delta > 0.0) || !std::isfinite(delta)) {
      return -std::numeric_limits<double>::infinity();
    }
    const double scale = 1.0 / delta;
    const double scaled_rhs = row.rhs * scale;
    const double down_rhs = fast_floor_xtab(scaled_rhs);
    const double f0 = scaled_rhs - down_rhs;
    if (f0 < f0_min || f0 > f0_max) {
      return -std::numeric_limits<double>::infinity();
    }
    const double inv_one_minus = 1.0 / (1.0 - f0);
    if (inv_one_minus > max_cmir_scale) {
      return -std::numeric_limits<double>::infinity();
    }
    const double cont_scale = scale * inv_one_minus;
    double norm_sq = cont_scale * cont_scale * continuous_norm_sq;
    double violation = cont_scale * continuous_contribution - down_rhs;
    for (int k : integer_inds) {
      const double scaled_a =
          row.vals[static_cast<std::size_t>(k)] * scale;
      const double down_a = fast_floor_xtab(scaled_a + tiny);
      const double fj = scaled_a - down_a;
      const double aj =
          down_a + std::max(0.0, (fj - f0) * inv_one_minus);
      xtab_update_violation_and_norm(row, k, aj, violation, norm_sq,
                                     feastol);
    }
    if (!(norm_sq > 0.0)) {
      return -std::numeric_limits<double>::infinity();
    }
    const double efficacy = violation / std::sqrt(norm_sq);
    if (delta_trace) {
      xtab_trace_cmir_delta(trace_id, trace_family, phase, delta, scale,
                            down_rhs, f0, violation, norm_sq, efficacy,
                            best_efficacy);
    }
    return efficacy;
  };

  for (double delta : deltas) {
    const double efficacy = evaluate_delta("candidate", delta);
    if (efficacy > best_efficacy) {
      best_delta = delta;
      best_efficacy = efficacy;
    }
  }
  if (best_delta < 0.0) {
    return false;
  }

  for (int k = 1; !only_initial_cmir_scale && k <= 3; ++k) {
    const double delta = best_delta * static_cast<double>(1 << k);
    const double efficacy = evaluate_delta("scale2", delta);
    const double efficacy_noise =
        1e-15 * std::max(1.0, std::abs(best_efficacy));
    const bool first_doubled_tie = k == 1 && efficacy == best_efficacy;
    if (efficacy > best_efficacy + efficacy_noise || first_doubled_tie) {
      best_delta = delta;
      best_efficacy = efficacy;
    }
  }

  for (int k : integer_inds) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (!std::isfinite(row.upper[pos]) || row.solval[pos] <= feastol) {
      continue;
    }
    xtab_flip_complementation(row, k);
    const double efficacy = evaluate_delta("flip", best_delta);
    if (efficacy > best_efficacy) {
      best_efficacy = efficacy;
    } else {
      xtab_flip_complementation(row, k);
    }
  }

  const HighsCDouble scale = 1.0 / HighsCDouble(best_delta);
  const HighsCDouble scaled_rhs = row.rhs * scale;
  const double down_rhs = std::floor(double(scaled_rhs));
  const HighsCDouble f0 = scaled_rhs - down_rhs;
  const HighsCDouble inv_one_minus = 1.0 / (1.0 - f0);
  row.rhs = down_rhs * best_delta;
  row.integral_support = true;
  row.integral_coefficients = false;
  if (trace != nullptr) {
    trace->best_delta = best_delta;
    trace->best_efficacy = best_efficacy;
    trace->final_f0 = double(f0);
    trace->final_rhs = row.rhs;
    trace->accepted = true;
  }
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (std::abs(row.vals[pos]) <= 0.0) {
      continue;
    }
    if (row.is_integral[pos] == 0) {
      if (row.vals[pos] > 0.0) {
        row.vals[pos] = 0.0;
      } else {
        row.vals[pos] = double(row.vals[pos] * inv_one_minus);
        row.integral_support = false;
      }
    } else {
      const HighsCDouble scaled_a = scale * row.vals[pos];
      const double down_a = std::floor(double(scaled_a + tiny));
      const HighsCDouble fj = scaled_a - down_a;
      HighsCDouble aj = down_a;
      if (fj > f0) {
        aj += (fj - f0) * inv_one_minus;
      }
      row.vals[pos] = double(aj * best_delta);
    }
  }
  return true;
}

bool xtab_try_generate_cut(XTabRow& row,
                           bool has_unbounded_ints,
                           bool has_general_ints,
                           bool has_continuous,
                           double feastol,
                           double min_efficacy,
                           int random_tiebreaker,
                           XTabCutgenRandom* cutgen_random,
                           XTabCmirTrace* trace = nullptr,
                           bool only_initial_cmir_scale = false,
                           std::uint64_t trace_id = 0,
                           const char* trace_family = nullptr) {
  if (trace != nullptr) *trace = XTabCmirTrace{};
  if (has_unbounded_ints) {
    return xtab_cmir_cut_generation(row, feastol, min_efficacy, trace,
                                    only_initial_cmir_scale, trace_id,
                                    trace_family);
  }

  const XTabRow saved_row = row;
  XTabLiftedCoverState cover_state;
  bool lifted_success = false;
  bool saved_integral_support = false;
  bool saved_integral_coefficients = false;
  double min_mir_efficacy = min_efficacy;
  XTabRow lifted_row;

  do {
    if (!xtab_determine_cover(row, feastol, cutgen_random, random_tiebreaker,
                              cover_state)) {
      break;
    }

    if (!has_continuous && !has_general_ints) {
      xtab_separate_lifted_knapsack_cover(row, cover_state, feastol, 1e-12);
      lifted_success = true;
    } else if (has_general_ints) {
      lifted_success =
          xtab_separate_lifted_mixed_integer_cover(row, cover_state, feastol,
                                                   1e-12);
    } else {
      lifted_success =
          xtab_separate_lifted_mixed_binary_cover(row, cover_state, 1e-12);
    }
  } while (false);

  if (lifted_success) {
    saved_integral_support = row.integral_support;
    saved_integral_coefficients = row.integral_coefficients;

    double violation = -row.rhs;
    double norm_sq = 0.0;
    for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
      xtab_update_violation_and_norm(row, k,
                                     row.vals[static_cast<std::size_t>(k)],
                                     violation, norm_sq, feastol);
    }
    const double efficacy =
        norm_sq > 0.0 ? violation / std::sqrt(norm_sq)
                      : -std::numeric_limits<double>::infinity();
    if (efficacy <= min_efficacy) {
      lifted_success = false;
    } else {
      min_mir_efficacy += efficacy;
      lifted_row = row;
    }
  }

  row = saved_row;
  XTabCmirTrace cmir_trace;
  if (xtab_cmir_cut_generation(row, feastol, min_mir_efficacy, &cmir_trace,
                               only_initial_cmir_scale, trace_id,
                               trace_family)) {
    if (trace != nullptr) *trace = cmir_trace;
    return true;
  }

  if (!lifted_success) {
    if (trace != nullptr) *trace = cmir_trace;
    return false;
  }

  row = std::move(lifted_row);
  row.integral_support = saved_integral_support;
  row.integral_coefficients = saved_integral_coefficients;
  if (trace != nullptr) {
    *trace = cmir_trace;
    trace->lifted_accepted = true;
  }
  return true;
}

bool xtab_untransform_cut(const SimplexResult& simplex,
                          const XTabSourceContext& source_context,
                          const XTabRow& transformed_cut,
                          Eigen::VectorXd& cut_le,
                          double& rhs_le) {
  const int n_orig = source_context.n_original;
  const int n_src = source_context.dim;
  if (!source_context.valid || n_orig <= 0 || n_src <= n_orig ||
      source_context.lower.size() != n_src ||
      source_context.upper.size() != n_src) {
    return false;
  }
  std::vector<HighsCDouble> std_coeff_acc(static_cast<std::size_t>(n_orig));
  std::vector<int> nonzero_cols;
  nonzero_cols.reserve(static_cast<std::size_t>(std::min(n_orig, n_src)));
  HighsCDouble rhs = transformed_cut.rhs;

  auto add_coeff = [&](int col, double value) {
    if (col < 0 || col >= n_orig || value == 0.0) return;
    HighsCDouble& acc = std_coeff_acc[static_cast<std::size_t>(col)];
    if (acc != 0.0) {
      acc += value;
    } else {
      acc = value;
      nonzero_cols.push_back(col);
    }
    if (acc == 0.0) {
      acc = (std::numeric_limits<double>::min)();
    }
  };

  auto add_logical_row = [&](int row, double scale) -> bool {
    if (row < 0 || row >= simplex.form.A_row.rows()) {
      return false;
    }
    // Use xtab_visit_source_row_terms so that HiGHS-presolved rows are read
    // from sf.source_row_index/value (original, unscaled) rather than from
    // sf.A_row (scaled/presolved).  For non-presolved rows the two paths are
    // equivalent; for presolved rows sf.A_row has different structure.
    xtab_visit_source_row_terms(
        simplex.form, row, [&](int c, double unscaled_coeff) {
          add_coeff(c, scale * unscaled_coeff);
        });
    return true;
  };

  for (int k = 0; k < static_cast<int>(transformed_cut.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    const int col = transformed_cut.inds[pos];
    if (col < 0 || col >= n_src) {
      return false;
    }
    const double val = transformed_cut.vals[pos];
    if (std::abs(val) <= 1e-12) {
      continue;
    }
    const bool is_logical_slack = source_context.is_logical_row_slack(col);
    switch (transformed_cut.bound_type[pos]) {
      case XTabBoundType::SimpleLb:
        if (is_logical_slack) {
          const int row = source_context.row_from_logical_slack(col);
          const double lower = source_context.lower[col];
          if (!std::isfinite(lower)) {
            return false;
          }
          rhs += val * lower;
          if (!add_logical_row(row, val)) {
            return false;
          }
        } else {
          const double lower = source_context.lower[col];
          if (!std::isfinite(lower)) {
            return false;
          }
          rhs += val * lower;
          add_coeff(col, val);
        }
        break;
      case XTabBoundType::SimpleUb: {
        const double ub = source_context.upper[col];
        if (!std::isfinite(ub)) {
          return false;
        }
        rhs -= val * ub;
        if (is_logical_slack) {
          const int row = source_context.row_from_logical_slack(col);
          if (!add_logical_row(row, -val)) {
            return false;
          }
        } else {
          add_coeff(col, -val);
        }
        break;
      }
      case XTabBoundType::VariableLb: {
        if (is_logical_slack) {
          return false;
        }
        const XTabVarBoundExpr& vb = transformed_cut.varbound_expr[pos];
        if (!vb.valid || vb.trigger_col < 0 || vb.trigger_col >= n_orig) {
          return false;
        }
        rhs += vb.constant * val;
        add_coeff(vb.trigger_col, -val * vb.coef);
        add_coeff(col, val);
        break;
      }
      case XTabBoundType::VariableUb: {
        if (is_logical_slack) {
          return false;
        }
        const XTabVarBoundExpr& vb = transformed_cut.varbound_expr[pos];
        if (!vb.valid || vb.trigger_col < 0 || vb.trigger_col >= n_orig) {
          return false;
        }
        rhs -= vb.constant * val;
        add_coeff(vb.trigger_col, val * vb.coef);
        add_coeff(col, -val);
        break;
      }
    }
  }

  Eigen::VectorXd coeff_orig = Eigen::VectorXd::Zero(n_orig);
  for (int col : nonzero_cols) {
    coeff_orig[col] = double(std_coeff_acc[static_cast<std::size_t>(col)]);
  }

  if (!std::isfinite(double(rhs)) || !coeff_orig.allFinite() ||
      coeff_orig.norm() <= 1e-12) {
    return false;
  }
  cut_le = std::move(coeff_orig);
  rhs_le = double(rhs);
  return true;
}

bool xtab_original_col_bounds(const SimplexResult& simplex,
                              int col,
                              double& lb,
                              double& ub) {
  if (col < 0 || col >= simplex.form.n_original ||
      col >= simplex.form.lb_shift.size() ||
      col >= simplex.form.var_ub.size()) {
    return false;
  }
  lb = simplex.form.lb_shift[col];
  ub = original_space_value(simplex, col, simplex.form.var_ub[col]);
  return std::isfinite(lb) && !std::isnan(ub);
}

bool xtab_postprocess_original_cut(const SimplexResult& simplex,
                                   const std::vector<char>* implied_integer_cols,
                                   Eigen::VectorXd& cut_le,
                                   double& rhs_le,
                                   double feastol,
                                   bool known_integral_cut = false) {
  feastol = std::max(0.0, feastol);
  constexpr double epsilon = 1e-12;
  const int n = simplex.form.n_original;
  if (cut_le.size() != n || n <= 0 || !std::isfinite(rhs_le) ||
      !cut_le.allFinite()) {
    return false;
  }
  if (rhs_le < 0.0 && rhs_le > -epsilon) {
    rhs_le = 0.0;
  }
  HighsCDouble rhs_acc = rhs_le;

  double max_abs = 0.0;
  for (int j = 0; j < n; ++j) {
    max_abs = std::max(max_abs, std::abs(cut_le[j]));
  }
  if (!(max_abs > 0.0)) {
    return false;
  }

  if (known_integral_cut) {
    for (int j = 0; j < n; ++j) {
      if (std::abs(cut_le[j]) <= epsilon) cut_le[j] = 0.0;
    }
    return cut_le.norm() > 1e-12 && std::isfinite(rhs_le);
  }

  const double min_coeff = 100.0 * feastol * std::max(max_abs, 1e-3);
  bool integral_support = true;
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (a == 0.0) {
      continue;
    }
    if (std::abs(a) <= min_coeff) {
      double lb = 0.0;
      double ub = 0.0;
      if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
          (a < 0.0 && !std::isfinite(ub))) {
        return false;
      }
      rhs_acc -= a * (a < 0.0 ? ub : lb);
      cut_le[j] = 0.0;
      continue;
    }
    if (integral_support &&
        !transformed_col_is_integer_like(simplex, j, implied_integer_cols)) {
      integral_support = false;
    }
  }

  max_abs = 0.0;
  double min_abs = std::numeric_limits<double>::infinity();
  std::vector<double> live_vals;
  live_vals.reserve(static_cast<std::size_t>(n));
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (std::abs(a) <= 0.0) {
      continue;
    }
    const double aa = std::abs(a);
    max_abs = std::max(max_abs, aa);
    min_abs = std::min(min_abs, aa);
    live_vals.push_back(a);
  }
  if (live_vals.empty()) {
    return false;
  }

  auto scale_cut = [&](double pivot) {
    if (!(pivot > epsilon) || !std::isfinite(pivot)) {
      return false;
    }
    int exp_shift = 0;
    std::frexp(pivot - epsilon, &exp_shift);
    exp_shift = std::min(10, -exp_shift);
    rhs_acc = ldexp(rhs_acc, exp_shift);
    const double scale = std::ldexp(1.0, exp_shift);
    for (int j = 0; j < n; ++j) {
      cut_le[j] *= scale;
    }
    return true;
  };

  if (integral_support) {
    const double int_scale = xtab_integral_scale(live_vals, feastol, epsilon);
    bool scale_smallest_to_one = true;
    if (int_scale != 0.0 &&
        int_scale * std::max(1.0, max_abs) <=
            static_cast<double>(uint64_t{1} << 52)) {
      rhs_acc.renormalize();
      rhs_acc *= int_scale;
      double scaled_max_abs = xtab_nearest_integer(max_abs * int_scale);
      for (int j = 0; j < n; ++j) {
        const double a = cut_le[j];
        if (a == 0.0) {
          continue;
        }
        const double scaled = int_scale * a;
        const double intval = xtab_nearest_integer(scaled);
        const double delta = scaled - intval;
        double lb = 0.0;
        double ub = 0.0;
        if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
            (delta < 0.0 && !std::isfinite(ub))) {
          return false;
        }
        cut_le[j] = intval;
        rhs_acc -= delta * (delta < 0.0 ? ub : lb);
      }
      rhs_acc = floor(rhs_acc + feastol);
      if (int_scale * scaled_max_abs * feastol < 0.5) {
        scale_smallest_to_one = false;
      }
    }
    if (scale_smallest_to_one) {
      min_abs = std::numeric_limits<double>::infinity();
      for (int j = 0; j < n; ++j) {
        if (cut_le[j] != 0.0) {
          min_abs = std::min(min_abs, std::abs(cut_le[j]));
        }
      }
      if (!scale_cut(min_abs)) {
        return false;
      }
    }
  } else {
    if (!scale_cut(max_abs)) {
      return false;
    }
  }

  for (int j = 0; j < n; ++j) {
    if (std::abs(cut_le[j]) <= epsilon) {
      cut_le[j] = 0.0;
    }
  }
  rhs_le = double(rhs_acc);
  return cut_le.norm() > 1e-12 && std::isfinite(rhs_le);
}

bool xtab_tighten_original_cut_coefficients(
    const SimplexResult& simplex,
    const std::vector<char>* implied_integer_cols,
    Eigen::VectorXd& cut_le,
    double& rhs_le,
    double feastol) {
  feastol = std::max(0.0, feastol);
  const int n = simplex.form.n_original;
  if (cut_le.size() != n || n <= 0 || !std::isfinite(rhs_le) ||
      !cut_le.allFinite()) {
    return false;
  }

  double max_activity = 0.0;
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (a == 0.0) {
      continue;
    }
    double lb = 0.0;
    double ub = 0.0;
    if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
        (a > 0.0 && !std::isfinite(ub)) ||
        (a < 0.0 && !std::isfinite(lb))) {
      // Cannot bound the maximum activity — skip tightening conservatively.
      return true;
    }
    max_activity += a * (a > 0.0 ? ub : lb);
  }

  const double max_abs_coeff = max_activity - rhs_le;
  if (!(max_abs_coeff > feastol) || !std::isfinite(max_abs_coeff)) {
    return true;
  }

  bool tightened = false;
  double new_rhs = rhs_le;
  for (int j = 0; j < n; ++j) {
    const double a = cut_le[j];
    if (a == 0.0 ||
        !transformed_col_is_integer_like(simplex, j, implied_integer_cols)) {
      continue;
    }
    double lb = 0.0;
    double ub = 0.0;
    if (!xtab_original_col_bounds(simplex, j, lb, ub) ||
        (a > 0.0 && !std::isfinite(ub))) {
      return true;
    }
    if (a > max_abs_coeff) {
      const double delta = a - max_abs_coeff;
      new_rhs -= delta * ub;
      cut_le[j] = max_abs_coeff;
      tightened = true;
    } else if (a < -max_abs_coeff) {
      const double delta = -a - max_abs_coeff;
      new_rhs += delta * lb;
      cut_le[j] = -max_abs_coeff;
      tightened = true;
    }
  }
  if (tightened) {
    rhs_le = new_rhs;
  }
  return std::isfinite(rhs_le) && cut_le.allFinite() && cut_le.norm() > 1e-12;
}

bool xtab_generate_cut_from_base_row(
    const SimplexResult& simplex,
    const XTabSourceContext& source_context,
    const Eigen::VectorXd& coeff_std,
    double rhs_std,
    const std::vector<int>* active_cols,
    const Eigen::VectorXd& x,
    const std::vector<char>* implied_integer_cols,
    const XTabTransformContext* transform_context,
    std::uint64_t* vb_substitutions_out,
    std::uint64_t* vb_trigger_terms_out,
    Eigen::VectorXd& cut_le,
    double& rhs_le,
	    double& efficacy_out,
    double cut_generation_feastol,
	    XTabRejectReason* reject_reason = nullptr,
	    XTabSourceDiag* gate_diag = nullptr,
    bool require_violation = true,
    const char* trace_family = "cut",
    const char* trace_basis_info = nullptr,
    XTabCutgenRandom* cutgen_random = nullptr,
    bool only_initial_cmir_scale = false,
    std::uint64_t* trace_id_out = nullptr) {
  if (reject_reason != nullptr) *reject_reason = XTabRejectReason::None;
  if (gate_diag != nullptr) ++gate_diag->gate_calls;
  std::uint64_t trace_id = 0;
  const bool trace_row = xtab_claim_row_trace(trace_id, trace_family);
  if (trace_id_out != nullptr) *trace_id_out = trace_id;
  if (trace_row) {
    if (trace_basis_info != nullptr && trace_basis_info[0] != '\0') {
      fmt::print(stderr, "[B&C-XROW] id={} family={} stage=basis {}\n",
                 trace_id, trace_family, trace_basis_info);
    }
    xtab_trace_source_row(trace_id, trace_family, "base", coeff_std, rhs_std,
                          active_cols);
    xtab_trace_source_meta(trace_id, trace_family, "base", simplex,
                           source_context, coeff_std, active_cols);
  }
  XTabRow row;
  std::uint64_t vb_substitutions = 0;
  std::uint64_t vb_trigger_terms = 0;
  bool integers_positive = true;
  if (!xtab_transform_base_row(simplex, source_context, coeff_std, rhs_std,
                               active_cols, implied_integer_cols,
                               transform_context,
                               &vb_substitutions, &vb_trigger_terms,
                               &integers_positive,
                               /*initial_integers_positive=*/true,
                               cut_generation_feastol, row)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
    if (trace_row) xtab_trace_fail(trace_id, trace_family, "transform", "fail");
    return false;
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "transform", row, source_context,
                   integers_positive);
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=transform_vbd "
               "vbSubs={} vbTerms={} initialIntPos=1\n",
               trace_id, trace_family, vb_substitutions, vb_trigger_terms);
  }
  if (gate_diag != nullptr) ++gate_diag->gate_transform_ok;
  if (vb_substitutions_out != nullptr) {
    *vb_substitutions_out += vb_substitutions;
  }
  if (vb_trigger_terms_out != nullptr) {
    *vb_trigger_terms_out += vb_trigger_terms;
  }
  bool has_unbounded_ints = false;
  bool has_general_ints = false;
  bool has_continuous = false;
  if (!xtab_preprocess_base_inequality(simplex, row, cut_generation_feastol,
                                       has_unbounded_ints, has_general_ints,
                                       has_continuous)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Preprocess;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "preprocess", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "preprocess", row, source_context,
                   integers_positive);
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=preprocess_flags "
               "unboundedInt={} generalInt={} continuous={}\n",
               trace_id, trace_family, has_unbounded_ints ? 1 : 0,
               has_general_ints ? 1 : 0, has_continuous ? 1 : 0);
  }
  if (gate_diag != nullptr) ++gate_diag->gate_preprocess_ok;
  if (!has_unbounded_ints && !integers_positive) {
    for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
      const std::size_t pos = static_cast<std::size_t>(k);
      if (row.is_integral[pos] == 0 || row.vals[pos] > 0.0) {
        continue;
      }
      xtab_flip_complementation(row, k);
    }
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "pre_cmir", row, source_context,
                   integers_positive);
  }
  const double cmir_min_efficacy =
      require_violation ? 10.0 * cut_generation_feastol
                        : -std::numeric_limits<double>::infinity();
  XTabCmirTrace cmir_trace;
  const int random_tiebreaker =
      static_cast<int>(trace_id == 0 ? 0 : (trace_id & 0x7fffffffULL));
  if (!xtab_try_generate_cut(row, has_unbounded_ints, has_general_ints,
                             has_continuous, cut_generation_feastol,
                             cmir_min_efficacy, random_tiebreaker,
                             cutgen_random,
                             trace_row ? &cmir_trace : nullptr,
                             only_initial_cmir_scale,
                             trace_row ? trace_id : 0,
                             trace_row ? trace_family : nullptr)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Cmir;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "cmir", "fail");
      fmt::print(stderr,
                 "[B&C-XROW] id={} family={} stage=cmir_stats "
                 "int={} cont={} deltas={} tested={} contContrib={:.12g} "
                 "contNorm2={:.12g} maxDelta={:.12g} bestDelta={:.12g} "
                 "bestEff={:.12g} minEff={:.12g}\n",
                 trace_id, trace_family, cmir_trace.integer_terms,
                 cmir_trace.continuous_terms, cmir_trace.initial_delta_count,
                 cmir_trace.tested_delta_count,
                 cmir_trace.continuous_contribution,
                 cmir_trace.continuous_norm_sq, cmir_trace.max_abs_delta,
	                 cmir_trace.best_delta, cmir_trace.best_efficacy,
	                 cmir_min_efficacy);
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family,
                   cmir_trace.lifted_accepted ? "lifted" : "cmir", row,
                   source_context, integers_positive);
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=cmir_stats "
               "int={} cont={} deltas={} tested={} contContrib={:.12g} "
               "contNorm2={:.12g} maxDelta={:.12g} bestDelta={:.12g} "
               "bestEff={:.12g} f0={:.12g} finalRhs={:.12g} minEff={:.12g} "
               "generator={}\n",
               trace_id, trace_family, cmir_trace.integer_terms,
               cmir_trace.continuous_terms, cmir_trace.initial_delta_count,
               cmir_trace.tested_delta_count,
               cmir_trace.continuous_contribution,
               cmir_trace.continuous_norm_sq, cmir_trace.max_abs_delta,
               cmir_trace.best_delta, cmir_trace.best_efficacy,
               cmir_trace.final_f0, cmir_trace.final_rhs, cmir_min_efficacy,
               cmir_trace.lifted_accepted ? "lifted" : "cmir");
  }
  if (gate_diag != nullptr) ++gate_diag->gate_cmir_ok;
  xtab_remove_complementation(row);
  if (trace_row) {
    xtab_trace_row(trace_id, trace_family, "remove_complementation", row,
                   source_context, integers_positive);
  }

  std::vector<unsigned char> erase(row.inds.size(), 0);
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    if (std::abs(row.vals[static_cast<std::size_t>(k)]) <= 1e-12) {
      erase[static_cast<std::size_t>(k)] = 1;
    }
  }
  xtab_erase_positions(row, erase);
  if (row.inds.empty()) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::EmptyCut;
    if (trace_row) xtab_trace_fail(trace_id, trace_family, "erase", "empty");
    return false;
  }
  if (!xtab_untransform_cut(simplex, source_context, row, cut_le, rhs_le)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "untransform", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_original_row(trace_id, trace_family, "untransform", cut_le,
                            rhs_le, x);
  }
  if (!xtab_postprocess_original_cut(simplex, implied_integer_cols, cut_le,
                                     rhs_le, cut_generation_feastol,
                                     row.integral_support &&
                                         row.integral_coefficients)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "postprocess", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_original_row(trace_id, trace_family, "postprocess", cut_le,
                            rhs_le, x);
  }
  if (!xtab_tighten_original_cut_coefficients(simplex, implied_integer_cols,
                                              cut_le, rhs_le,
                                              cut_generation_feastol)) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "tighten", "fail");
    }
    return false;
  }
  if (trace_row) {
    xtab_trace_original_row(trace_id, trace_family, "tighten", cut_le, rhs_le,
                            x);
  }
  if (gate_diag != nullptr) ++gate_diag->gate_untransform_ok;

  const double violation = cut_le.dot(x) - rhs_le;
  const double norm = std::max(1e-12, cut_le.norm());
  efficacy_out = violation / norm;
  const bool violated = violation > 10.0 * cut_generation_feastol;
  if (violated && gate_diag != nullptr) ++gate_diag->gate_violation_ok;
  if (!violated && reject_reason != nullptr) {
    *reject_reason = XTabRejectReason::NotViolated;
  }
  if (trace_row) {
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=final violated={} "
               "violation={:.12g} norm={:.12g} efficacy={:.12g} "
               "requireViolation={}\n",
               trace_id, trace_family, violated ? 1 : 0, violation, norm,
               efficacy_out, require_violation ? 1 : 0);
  }
  return require_violation ? violated : true;
}

bool aggregate_standard_rows(const SimplexResult& simplex,
                             const XTabSourceContext& source_context,
                             const std::vector<std::pair<int, double>>& rows,
                             Eigen::VectorXd& coeff,
                             double& rhs,
                             std::vector<int>* active_cols = nullptr);
double standard_row_max_abs(const SimplexResult& simplex, int row);

struct XTabLpAggregator {
  XTabLpAggregator(const SimplexResult& simplex,
                   const XTabSourceContext& source_context)
      : sf(simplex.form),
        ctx(source_context),
        sum(static_cast<std::size_t>(source_context.dim)),
        touched_flag(static_cast<std::size_t>(source_context.dim), 0) {}

  void add_value(int col, double value) {
    if (col < 0 || col >= ctx.dim || value == 0.0) return;
    HighsCDouble& acc = sum[static_cast<std::size_t>(col)];
    if (acc != 0.0) {
      acc += value;
    } else {
      acc = value;
      if (touched_flag[static_cast<std::size_t>(col)] == 0) {
        touched_flag[static_cast<std::size_t>(col)] = 1;
        touched.push_back(col);
      }
    }
    if (acc == 0.0) {
      acc = (std::numeric_limits<double>::min)();
    }
  }

  bool add_row(int row, double weight) {
    if (!ctx.valid || row < 0 || row >= sf.A_row.rows() ||
        row >= ctx.n_rows || !std::isfinite(weight)) {
      return false;
    }
    const double row_scale_inv = 1.0 / xtab_row_scale_or_one(sf, row);
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             sf.A_row, row);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      if (col < 0 || col >= ctx.n_original) {
        continue;
      }
      const double unscaled =
          it.value() * row_scale_inv / xtab_col_scale_or_one(sf, col);
      add_value(col, weight * unscaled);
    }
    const int slack_col = ctx.n_original + row;
    if (slack_col < 0 || slack_col >= ctx.dim) {
      return false;
    }
    add_value(slack_col, -weight);
    return true;
  }

  bool get_current_aggregation(Eigen::VectorXd& coeff,
                               double& rhs,
                               bool negate,
                               std::vector<int>* active_cols = nullptr) {
    if (coeff.size() != static_cast<Eigen::Index>(sum.size())) {
      coeff.resize(static_cast<int>(sum.size()));
    }
    coeff.setZero();
    if (active_cols != nullptr) active_cols->clear();
    const double sign = negate ? -1.0 : 1.0;
    rhs = 0.0;
    constexpr double droptol = 1e-12;
    int num_nz = static_cast<int>(touched.size());
    for (int i = num_nz - 1; i >= 0; --i) {
      const int col = touched[static_cast<std::size_t>(i)];
      const double val = double(sum[static_cast<std::size_t>(col)]);
      if (!std::isfinite(val)) {
        return false;
      }
      // HiGHS' HighsLpAggregator drops tiny structural coefficients but keeps
      // exact logical row-activity slack terms.
      const bool drop = col < ctx.n_original ? std::abs(val) <= droptol
                                             : val == 0.0;
      if (drop) {
        sum[static_cast<std::size_t>(col)] = 0.0;
        touched_flag[static_cast<std::size_t>(col)] = 0;
        --num_nz;
        std::swap(touched[static_cast<std::size_t>(num_nz)],
                  touched[static_cast<std::size_t>(i)]);
      }
    }
    touched.resize(static_cast<std::size_t>(num_nz));
    for (int col : touched) {
      const double val = sign * double(sum[static_cast<std::size_t>(col)]);
      coeff[col] = val;
      if (active_cols != nullptr) active_cols->push_back(col);
    }
    return std::isfinite(rhs);
  }

  void clear() {
    for (int col : touched) {
      sum[static_cast<std::size_t>(col)] = 0.0;
      touched_flag[static_cast<std::size_t>(col)] = 0;
    }
    touched.clear();
  }

  const StandardFormLP& sf;
  const XTabSourceContext& ctx;
  std::vector<HighsCDouble> sum;
  std::vector<int> touched;
  std::vector<unsigned char> touched_flag;
};

struct XTabAggregatedSourceRow {
  Eigen::VectorXd coeff;
  double rhs{0.0};
  std::vector<int> active_cols;
};

bool xtab_generate_path_mixing_cut(
    const SimplexResult& simplex,
    const XTabSourceContext& source_context,
    const std::vector<XTabAggregatedSourceRow>& aggregated_path,
    const Eigen::VectorXd& x,
    const std::vector<char>* implied_integer_cols,
    const XTabTransformContext* transform_context,
    std::uint64_t* vb_substitutions_out,
    std::uint64_t* vb_trigger_terms_out,
    Eigen::VectorXd& cut_le,
    double& rhs_le,
    double& efficacy_out,
    double cut_generation_feastol,
    XTabRejectReason* reject_reason = nullptr,
    const char* trace_family = "pathmix") {
  if (reject_reason != nullptr) *reject_reason = XTabRejectReason::None;
  if (vb_substitutions_out != nullptr) *vb_substitutions_out = 0;
  if (vb_trigger_terms_out != nullptr) *vb_trigger_terms_out = 0;
  std::uint64_t trace_id = 0;
  const bool trace_row = xtab_claim_row_trace(trace_id, trace_family);
  if (trace_row) {
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=pathmix_start rows={} "
               "nsrc={} norig={}\n",
               trace_id, trace_family, aggregated_path.size(),
               source_context.dim, simplex.form.n_original);
  }
  const double feastol = std::max(0.0, cut_generation_feastol);
  constexpr double epsilon = 1e-12;
  const int n_src = source_context.dim;
  const int n_orig = simplex.form.n_original;
  if (!source_context.valid || aggregated_path.size() < 2 || n_src <= 0 ||
      n_orig <= 0 || x.size() != n_orig) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "pathmix_validate", "fail");
    }
    return false;
  }

  struct PathTermInfo {
    int col{-1};
    double upper{0.0};
    double solval{0.0};
    XTabBoundType bound_type{XTabBoundType::SimpleLb};
    XTabVarBoundExpr varbound;
    unsigned char is_integral{0};
  };

  std::vector<XTabRow> transformed_rows;
  transformed_rows.reserve(aggregated_path.size());
  std::vector<int> pos(static_cast<std::size_t>(n_src), -1);
  std::vector<PathTermInfo> terms;
  terms.reserve(static_cast<std::size_t>(std::min(n_src, 4096)));
  std::vector<double> transformed_rhs;
  transformed_rhs.reserve(aggregated_path.size());
  double delta = 1.0;

  int source_row_index = 0;
  for (const auto& source_row : aggregated_path) {
    if (trace_row) {
      const std::string stage = fmt::format("path_base_{}", source_row_index);
      xtab_trace_source_row(trace_id, trace_family, stage.c_str(),
                            source_row.coeff, source_row.rhs,
                            &source_row.active_cols);
    }
    XTabRow row;
    std::uint64_t vb_substitutions = 0;
    std::uint64_t vb_trigger_terms = 0;
    bool integers_positive = true;
    if (!xtab_transform_base_row(simplex, source_context, source_row.coeff,
                                 source_row.rhs, &source_row.active_cols,
                                 implied_integer_cols, transform_context,
                                 &vb_substitutions, &vb_trigger_terms,
                                 &integers_positive,
                                 /*initial_integers_positive=*/false,
                                 cut_generation_feastol, row)) {
      if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
      if (trace_row) {
        const std::string stage =
            fmt::format("path_transform_{}", source_row_index);
        xtab_trace_fail(trace_id, trace_family, stage.c_str(), "fail");
      }
      break;
    }
    if (trace_row) {
      const std::string stage =
          fmt::format("path_transform_{}", source_row_index);
      xtab_trace_row(trace_id, trace_family, stage.c_str(), row,
                     source_context, integers_positive);
      fmt::print(stderr,
                 "[B&C-XROW] id={} family={} stage=path_transform_vbd_{} "
                 "vbSubs={} vbTerms={} initialIntPos=0 intPos={}\n",
                 trace_id, trace_family, source_row_index, vb_substitutions,
                 vb_trigger_terms, integers_positive ? 1 : 0);
    }
    if (vb_substitutions_out != nullptr) {
      *vb_substitutions_out += vb_substitutions;
    }
    if (vb_trigger_terms_out != nullptr) {
      *vb_trigger_terms_out += vb_trigger_terms;
    }
    const int k = static_cast<int>(transformed_rhs.size());
    if (k == 0) {
      if (row.rhs > epsilon) {
        if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Cmir;
        if (trace_row) {
          xtab_trace_fail(trace_id, trace_family, "path_rhs_order",
                          "first_rhs_positive");
        }
        break;
      }
      if (row.rhs >= -feastol) row.rhs = 0.0;
    } else if (row.rhs >= transformed_rhs.back() - feastol) {
      if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Cmir;
      if (trace_row) {
        xtab_trace_fail(trace_id, trace_family, "path_rhs_order",
                        "not_decreasing");
      }
      break;
    }
    delta = std::max(delta, std::abs(row.rhs));

    bool consistent = true;
    for (int t = 0; t < static_cast<int>(row.inds.size()); ++t) {
      const std::size_t tp = static_cast<std::size_t>(t);
      const int col = row.inds[tp];
      if (col < 0 || col >= n_src) {
        consistent = false;
        break;
      }
      int& p = pos[static_cast<std::size_t>(col)];
      if (p < 0) {
        p = static_cast<int>(terms.size());
        terms.push_back(PathTermInfo{
            col, row.upper[tp], row.solval[tp], row.bound_type[tp],
            row.varbound_expr[tp], row.is_integral[tp]});
      } else {
        const PathTermInfo& info = terms[static_cast<std::size_t>(p)];
        const double scale =
            std::max(1.0, std::max(std::abs(info.upper), std::abs(row.upper[tp])));
        auto same_varbound = [&]() {
          if (info.bound_type != XTabBoundType::VariableLb &&
              info.bound_type != XTabBoundType::VariableUb) {
            return true;
          }
          const XTabVarBoundExpr& a = info.varbound;
          const XTabVarBoundExpr& b = row.varbound_expr[tp];
          return a.valid == b.valid &&
                 a.trigger_col == b.trigger_col &&
                 std::abs(a.constant - b.constant) <=
                     100.0 * feastol * scale &&
                 std::abs(a.coef - b.coef) <=
                     100.0 * feastol * scale;
        };
        if (info.bound_type != row.bound_type[tp] ||
            info.is_integral != row.is_integral[tp] ||
            !same_varbound() ||
            std::abs(info.solval - row.solval[tp]) >
                100.0 * feastol * scale ||
            ((std::isfinite(info.upper) || std::isfinite(row.upper[tp])) &&
                (!std::isfinite(info.upper) || !std::isfinite(row.upper[tp]) ||
                 std::abs(info.upper - row.upper[tp]) >
                     100.0 * feastol * scale))) {
          consistent = false;
          break;
        }
      }
      if (row.is_integral[tp] != 0) {
        delta = std::max(delta, std::abs(row.vals[tp]));
      }
    }
    if (!consistent) {
      if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Transform;
      if (trace_row) {
        xtab_trace_fail(trace_id, trace_family, "path_consistency", "fail");
      }
      break;
    }

    transformed_rhs.push_back(row.rhs);
    transformed_rows.push_back(std::move(row));
    ++source_row_index;
  }

  if (transformed_rows.size() < 2) {
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "pathmix_transform",
                      "too_short");
    }
    return false;
  }

  delta = std::exp2(std::ceil(std::log2(delta + 1.0)));
  const int num_terms = static_cast<int>(terms.size());
  if (num_terms == 0) {
    if (reject_reason != nullptr) *reject_reason = XTabRejectReason::EmptyCut;
    if (trace_row) {
      xtab_trace_fail(trace_id, trace_family, "pathmix_terms", "empty");
    }
    return false;
  }
  if (trace_row) {
    fmt::memory_buffer rhs_sample;
    const int max_terms = xtab_row_trace_terms();
    const int emit = std::min<int>(max_terms, transformed_rhs.size());
    for (int i = 0; i < emit; ++i) {
      if (i > 0) fmt::format_to(std::back_inserter(rhs_sample), ",");
      fmt::format_to(std::back_inserter(rhs_sample), "{:.12g}",
                     transformed_rhs[static_cast<std::size_t>(i)]);
    }
    if (static_cast<int>(transformed_rhs.size()) > emit) {
      fmt::format_to(std::back_inserter(rhs_sample), ",...");
    }
    fmt::print(stderr,
               "[B&C-XROW] id={} family={} stage=pathmix_delta rows={} "
               "terms={} delta={:.12g} rhs=[{}]\n",
               trace_id, trace_family, transformed_rows.size(), num_terms,
               delta, fmt::to_string(rhs_sample));
  }

  std::vector<double> cut_vals(static_cast<std::size_t>(num_terms), 0.0);
  std::vector<double> max_frac(static_cast<std::size_t>(num_terms), 0.0);
  std::vector<double> down_sum(static_cast<std::size_t>(num_terms), 0.0);
  std::vector<double> f_sum(static_cast<std::size_t>(num_terms), 0.0);

  double cut_rhs = 0.0;
  double f_last = 0.0;
  const double inv_delta_scale = -1.0 / delta;

  for (int k = 0; k < static_cast<int>(transformed_rows.size()); ++k) {
    const XTabRow& row = transformed_rows[static_cast<std::size_t>(k)];
    const double f = transformed_rhs[static_cast<std::size_t>(k)] *
                     inv_delta_scale;
    const double f_diff = f - f_last;
    cut_rhs += f_diff;

    for (int t = 0; t < static_cast<int>(row.inds.size()); ++t) {
      const std::size_t tp = static_cast<std::size_t>(t);
      const int col = row.inds[tp];
      const int p = pos[static_cast<std::size_t>(col)];
      if (p < 0) continue;
      const double gj = row.vals[tp] * inv_delta_scale;
      const std::size_t pp = static_cast<std::size_t>(p);
      if (terms[pp].is_integral == 0) {
        cut_vals[pp] = std::max(cut_vals[pp], gj);
      } else {
        const double gj_down = std::floor(gj);
        const double hj = gj - gj_down;
        max_frac[pp] = std::max(max_frac[pp], hj);
        down_sum[pp] += f_diff * gj_down;
        f_sum[pp] += f_diff;
        cut_vals[pp] =
            down_sum[pp] +
            (f_sum[pp] < max_frac[pp] ? f_sum[pp] : max_frac[pp]);
      }
    }

    if (k > 0) {
      double violation = cut_rhs;
      for (int p = 0; p < num_terms; ++p) {
        violation -= terms[static_cast<std::size_t>(p)].solval *
                     cut_vals[static_cast<std::size_t>(p)];
      }
      violation *= delta;
      if (trace_row) {
        fmt::print(stderr,
                   "[B&C-XROW] id={} family={} stage=pathmix_probe k={} "
                   "pathViolation={:.12g} cutRhs={:.12g} delta={:.12g}\n",
                   trace_id, trace_family, k, violation, cut_rhs, delta);
      }
      if (violation > 10.0 * feastol) {
        XTabRow out;
        out.rhs = cut_rhs * (-delta);
        out.inds.reserve(static_cast<std::size_t>(num_terms));
        out.vals.reserve(static_cast<std::size_t>(num_terms));
        out.upper.reserve(static_cast<std::size_t>(num_terms));
        out.solval.reserve(static_cast<std::size_t>(num_terms));
        out.bound_type.reserve(static_cast<std::size_t>(num_terms));
        out.varbound_expr.reserve(static_cast<std::size_t>(num_terms));
        out.complemented.reserve(static_cast<std::size_t>(num_terms));
        out.is_integral.reserve(static_cast<std::size_t>(num_terms));
        for (int p = 0; p < num_terms; ++p) {
          const double val = cut_vals[static_cast<std::size_t>(p)] * (-delta);
          if (std::abs(val) <= epsilon) continue;
          const PathTermInfo& info = terms[static_cast<std::size_t>(p)];
          out.inds.push_back(info.col);
          out.vals.push_back(val);
          out.upper.push_back(info.upper);
          out.solval.push_back(info.solval);
          out.bound_type.push_back(info.bound_type);
          out.varbound_expr.push_back(info.varbound);
          out.complemented.push_back(0);
          out.is_integral.push_back(info.is_integral);
        }
        if (out.inds.empty()) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::EmptyCut;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "pathmix_row", "empty");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_row(trace_id, trace_family, "pathmix_row", out,
                         source_context, false);
        }
        if (!xtab_untransform_cut(simplex, source_context, out, cut_le,
                                  rhs_le)) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "untransform", "fail");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_original_row(trace_id, trace_family, "untransform",
                                  cut_le, rhs_le, x);
        }
        if (!xtab_postprocess_original_cut(simplex, implied_integer_cols,
                                           cut_le, rhs_le, feastol)) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "postprocess", "fail");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_original_row(trace_id, trace_family, "postprocess",
                                  cut_le, rhs_le, x);
        }
        if (!xtab_tighten_original_cut_coefficients(
                simplex, implied_integer_cols, cut_le, rhs_le, feastol)) {
          if (reject_reason != nullptr) *reject_reason = XTabRejectReason::Untransform;
          if (trace_row) {
            xtab_trace_fail(trace_id, trace_family, "tighten", "fail");
          }
          return false;
        }
        if (trace_row) {
          xtab_trace_original_row(trace_id, trace_family, "tighten", cut_le,
                                  rhs_le, x);
        }
        const double final_violation = cut_le.dot(x) - rhs_le;
        const double norm = std::max(1e-12, cut_le.norm());
        efficacy_out = final_violation / norm;
        const bool accepted = final_violation > 10.0 * feastol;
        if (trace_row) {
          fmt::print(stderr,
                     "[B&C-XROW] id={} family={} stage=final violated={} "
                     "violation={:.12g} norm={:.12g} efficacy={:.12g} "
                     "requireViolation=1\n",
                     trace_id, trace_family, accepted ? 1 : 0,
                     final_violation, norm, efficacy_out);
        }
        if (accepted) {
          return true;
        }
        if (reject_reason != nullptr) *reject_reason = XTabRejectReason::NotViolated;
        return false;
      }
    }
    f_last = f;
  }

  if (reject_reason != nullptr) *reject_reason = XTabRejectReason::NotViolated;
  if (trace_row) {
    xtab_trace_fail(trace_id, trace_family, "pathmix", "no_violation");
  }
  return false;
}

double standard_row_max_abs(const SimplexResult& simplex, int row) {
  double max_abs = 0.0;
  const double row_scale_inv = 1.0 / xtab_row_scale_or_one(simplex.form, row);
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
           simplex.form.A_row, row);
       it; ++it) {
    const int col = static_cast<int>(it.col());
    if (col >= 0 && col < simplex.form.n_original) {
      const double value = std::abs(it.value()) * row_scale_inv /
                           xtab_col_scale_or_one(simplex.form, col);
      max_abs = std::max(max_abs, value);
    }
  }
  return max_abs;
}

bool aggregate_standard_rows(const SimplexResult& simplex,
                             const XTabSourceContext& source_context,
                             const std::vector<std::pair<int, double>>& rows,
                             Eigen::VectorXd& coeff,
                             double& rhs,
                             std::vector<int>* active_cols) {
  XTabLpAggregator aggregator(simplex, source_context);
  for (const auto& [r, w] : rows) {
    if (!aggregator.add_row(r, w)) {
      return false;
    }
  }
  return aggregator.get_current_aggregation(coeff, rhs, false, active_cols);
}

template <int k, typename FoundModKCut>
bool xtab_separate_modk_system(const std::vector<std::int64_t>& values,
                               const std::vector<int>& indices,
                               const std::vector<int>& starts,
                               int num_cols,
                               FoundModKCut&& found_cut) {
  HighsGFkSolve solver;
  std::vector<HighsInt> highs_indices(indices.begin(), indices.end());
  std::vector<HighsInt> highs_starts(starts.begin(), starts.end());
  solver.fromCSC<static_cast<unsigned int>(k)>(values, highs_indices,
                                               highs_starts, num_cols + 1);
  solver.setRhs<static_cast<unsigned int>(k)>(num_cols, 1);
  bool found = false;
  solver.solve<static_cast<unsigned int>(k)>(
      [&](std::vector<HighsGFkSolve::SolutionEntry>& weights,
          int rhs_index) {
        found = true;
        found_cut(weights, rhs_index);
      });
  return found;
}

int add_transformed_modk_cuts_impl(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    const std::vector<char>* implied_integer_cols,
    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table,
    std::vector<PoolCut>* generated_cutpool_rows,
    double cut_generation_feastol,
    const std::function<int(PoolCut&&)>* cutpool_acceptor,
    std::optional<std::uint64_t> highs_cutgen_seed) {
  const bool direct_cutpool_mode = cutpool_acceptor != nullptr;
  (void)lp;
  (void)opt;
  cut_generation_feastol = std::max(0.0, cut_generation_feastol);
  if ((!direct_cutpool_mode && generated_cutpool_rows == nullptr) ||
      !simplex.exact_optimal || x.size() != simplex.form.n_original) {
    return 0;
  }

  XTabSourceDiag diag;
  const XTabTransformContext transform_context = xtab_build_transform_context(
      simplex, implied_integer_cols, implication_graph, variable_bound_table);
  const XTabSourceContext source_context =
      xtab_build_source_context(simplex, implied_integer_cols);
  XTabCutgenRandom cutgen_random(highs_cutgen_seed);
  if (!source_context.valid) {
    maybe_print_xtab_diag("modk", diag);
    return 0;
  }

  struct XModkLedger {
    int src_rows{0};
    int src_dim{0};
    int skipped_continuous{0};
    int active_le{0};
    int active_ge{0};
    int inactive{0};
    int transform_ok{0};
    int transform_fail{0};
    int integral_rows{0};
    int scaled_rows{0};
    int long_skip{0};
    int zero_skip{0};
    int system_nnz{0};
    int nonzero_rhs{0};
    int gf2_calls{0};
    int gf3_calls{0};
    int gf5_calls{0};
    int gf7_calls{0};
    int gf_solutions{0};
    int cutgen_calls{0};
    int cutgen_success{0};
    int pool_calls{0};
    int pool_accepted{0};
    std::uint64_t system_hash{0};
  } ledger;
  ledger.src_rows = source_context.n_rows;
  ledger.src_dim = source_context.dim;

  auto print_modk_ledger = [&](const char* stage, int accepted) {
    if (std::getenv("MIPSOLVERS_XPATH_LEDGER") == nullptr &&
        std::getenv("MIPSOLVERS_XTAB_DIAG") == nullptr) {
      return;
    }
    fmt::print(
        stderr,
        "[B&C-XMODK-LEDGER] stage={} mode={} srcRows={} srcDim={} "
        "skipCont={} active=le{}:ge{} inactive={} transform={}/{} "
        "rows=int{}:scaled{} longSkip={} zeroSkip={} systemNnz={} rhsNz={} "
        "gf=2:{}:3:{}:5:{}:7:{} sol={} cutgen={}/{} "
        "poolCalls={} poolAccepted={} acceptedRows={} sysHash={:016x}\n",
        stage, direct_cutpool_mode ? "cutpool" : "rows",
        ledger.src_rows, ledger.src_dim, ledger.skipped_continuous,
        ledger.active_le, ledger.active_ge, ledger.inactive,
        ledger.transform_ok, ledger.transform_fail, ledger.integral_rows,
        ledger.scaled_rows, ledger.long_skip, ledger.zero_skip,
        ledger.system_nnz, ledger.nonzero_rhs, ledger.gf2_calls,
        ledger.gf3_calls, ledger.gf5_calls, ledger.gf7_calls,
        ledger.gf_solutions, ledger.cutgen_success, ledger.cutgen_calls,
        ledger.pool_calls, ledger.pool_accepted, accepted,
        ledger.system_hash);
  };

  const int n_orig = simplex.form.n_original;
  const int m = source_context.n_rows;
  const double feastol = cut_generation_feastol;
  constexpr double epsilon = 1e-9;
  const int system_trace_limit = xtab_modk_system_trace_limit();
  const int system_trace_terms = xtab_modk_system_trace_terms();

  std::vector<unsigned char> integer_like(
      static_cast<std::size_t>(n_orig), 0);
  std::vector<double> bound_distance(static_cast<std::size_t>(n_orig), 0.0);
  for (int col = 0; col < n_orig; ++col) {
    integer_like[static_cast<std::size_t>(col)] =
        source_context.integral[static_cast<std::size_t>(col)] != 0 ? 1 : 0;
    if (integer_like[static_cast<std::size_t>(col)] != 0) continue;
    const double val = source_context.solution[col];
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    double lb_dist = std::isfinite(lb) ? std::max(0.0, val - lb)
                                       : std::numeric_limits<double>::infinity();
    double ub_dist = std::isfinite(ub) ? std::max(0.0, ub - val)
                                       : std::numeric_limits<double>::infinity();
    if (col < static_cast<int>(transform_context.best_vlb.size())) {
      const XTabVarBoundExpr& vlb =
          transform_context.best_vlb[static_cast<std::size_t>(col)];
      if (vlb.valid) lb_dist = std::min(lb_dist, vlb.dist);
    }
    if (col < static_cast<int>(transform_context.best_vub.size())) {
      const XTabVarBoundExpr& vub =
          transform_context.best_vub[static_cast<std::size_t>(col)];
      if (vub.valid) ub_dist = std::min(ub_dist, vub.dist);
    }
    const double dist = std::min(lb_dist, ub_dist);
    bound_distance[static_cast<std::size_t>(col)] =
        std::isfinite(dist) && dist > feastol ? dist : 0.0;
  }

  std::vector<unsigned char> skip_row(static_cast<std::size_t>(m), 0);
  for (int col = 0; col < n_orig; ++col) {
    if (integer_like[static_cast<std::size_t>(col)] != 0 ||
        bound_distance[static_cast<std::size_t>(col)] == 0.0) {
      continue;
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(simplex.form.A, col);
         it; ++it) {
      const int row = static_cast<int>(it.row());
      if (row < 0 || row >= m ||
          skip_row[static_cast<std::size_t>(row)] != 0) {
        continue;
      }
      skip_row[static_cast<std::size_t>(row)] = 1;
      ++ledger.skipped_continuous;
    }
  }

  const int max_int_row_len = static_cast<int>(1000 + 0.1 * n_orig);
  std::vector<std::pair<int, double>> integral_scales;
  std::vector<std::int64_t> int_system_value;
  std::vector<int> int_system_index;
  std::vector<int> int_system_start;
  int_system_start.push_back(0);

  for (int row = 0; row < m; ++row) {
    if (skip_row[static_cast<std::size_t>(row)] != 0) continue;

    const int scol = n_orig + row;
    const double lower = source_context.lower[scol];
    const double upper = source_context.upper[scol];
    const double act = source_context.solution[scol];
    bool leq_row = true;
    if (std::isfinite(upper) && upper - act <= feastol) {
      leq_row = true;
      ++ledger.active_le;
    } else if (std::isfinite(lower) && act - lower <= feastol) {
      leq_row = false;
      ++ledger.active_ge;
    } else {
      ++ledger.inactive;
      continue;
    }

    Eigen::VectorXd coeff = Eigen::VectorXd::Zero(source_context.dim);
    double rhs = leq_row ? upper : -lower;
    std::vector<int> active_cols;
    int source_row_len = 0;
    xtab_visit_source_row_terms(simplex.form, row, [&](int col, double val) {
      ++source_row_len;
      if (std::abs(val) <= 1e-12) return;
      coeff[col] = leq_row ? val : -val;
      active_cols.push_back(col);
    });

    XTabRow transformed;
    bool integers_positive = false;
    std::uint64_t vb_subs = 0;
    std::uint64_t vb_terms = 0;
    if (!xtab_transform_base_row(simplex, source_context, coeff, rhs,
                                 &active_cols, implied_integer_cols,
                                 &transform_context, &vb_subs, &vb_terms,
                                 &integers_positive,
                                 /*initial_integers_positive=*/false,
                                 feastol, transformed)) {
      ++ledger.transform_fail;
      continue;
    }
    ++ledger.transform_ok;
    diag.vb_substitutions += vb_subs;
    diag.vb_trigger_terms += vb_terms;
    if (row == xtab_modk_transform_trace_row()) {
      fmt::memory_buffer rawbuf;
      const int raw_emit = std::min<int>(active_cols.size(), 200);
      for (int k = 0; k < raw_emit; ++k) {
        const int col = active_cols[static_cast<std::size_t>(k)];
        if (k > 0) fmt::format_to(std::back_inserter(rawbuf), ",");
        fmt::format_to(std::back_inserter(rawbuf), "{}:{:.17g}", col,
                       col >= 0 && col < coeff.size() ? coeff[col] : 0.0);
      }
      if (static_cast<int>(active_cols.size()) > raw_emit) {
        fmt::format_to(std::back_inserter(rawbuf), ",...");
      }
      fmt::memory_buffer solbuf;
      const int emit = std::min<int>(transformed.inds.size(), 200);
      for (int k = 0; k < emit; ++k) {
        if (k > 0) fmt::format_to(std::back_inserter(solbuf), ",");
        fmt::format_to(std::back_inserter(solbuf), "{}:{:.17g}:{:.17g}:{}",
                       transformed.inds[static_cast<std::size_t>(k)],
                       transformed.vals[static_cast<std::size_t>(k)],
                       transformed.solval[static_cast<std::size_t>(k)],
                       transformed.is_integral[static_cast<std::size_t>(k)]);
      }
      if (static_cast<int>(transformed.inds.size()) > emit) {
        fmt::format_to(std::back_inserter(solbuf), ",...");
      }
      fmt::print(stderr,
                 "[B&C-XMODK-TRANSFORM] row={} side={} rawLen={} rhs={:.17g} "
                 "raw=[{}] transLen={} transRhs={:.17g} trans=[{}]\n",
                 row, leq_row ? "le" : "ge", source_row_len, rhs,
                 fmt::to_string(rawbuf),
                 static_cast<int>(transformed.inds.size()), transformed.rhs,
                 fmt::to_string(solbuf));
    }

    const int rowlen = static_cast<int>(transformed.inds.size());
    if (rowlen > max_int_row_len) {
      int int_row_len = 0;
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.solval[pos] <= feastol) continue;
        if (transformed.is_integral[pos] == 0) continue;
        ++int_row_len;
      }
      if (int_row_len > max_int_row_len ||
          (int_row_len == 0 && std::abs(transformed.rhs) <= epsilon)) {
        ++ledger.long_skip;
        continue;
      }
    }

    double intscale = 1.0;
    std::int64_t intrhs = 0;
    const int system_row_start = int_system_start.back();
    const bool row_integral =
        source_context.integral[static_cast<std::size_t>(scol)] != 0;
    if (!row_integral) {
      std::vector<double> scale_vals;
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.is_integral[pos] == 0) continue;
        if (transformed.solval[pos] > feastol) {
          scale_vals.push_back(transformed.vals[pos]);
        }
      }
      if (std::abs(transformed.rhs) > epsilon) {
        scale_vals.push_back(-transformed.rhs);
      }
      if (scale_vals.empty()) {
        ++ledger.zero_skip;
        continue;
      }
      intscale = xtab_integral_scale(scale_vals, feastol, epsilon);
      if (intscale == 0.0 || intscale > 1e6) {
        ++ledger.zero_skip;
        continue;
      }
      intrhs = static_cast<std::int64_t>(
          xtab_nearest_integer(intscale * transformed.rhs));
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.is_integral[pos] == 0) continue;
        if (transformed.solval[pos] > feastol) {
          int_system_index.push_back(transformed.inds[pos]);
          int_system_value.push_back(static_cast<std::int64_t>(
              xtab_nearest_integer(intscale * transformed.vals[pos])));
        }
      }
      ++ledger.scaled_rows;
    } else {
      intrhs = static_cast<std::int64_t>(
          xtab_nearest_integer(transformed.rhs));
      for (int i = 0; i < rowlen; ++i) {
        const std::size_t pos = static_cast<std::size_t>(i);
        if (transformed.solval[pos] > feastol) {
          int_system_index.push_back(transformed.inds[pos]);
          int_system_value.push_back(static_cast<std::int64_t>(
              xtab_nearest_integer(transformed.vals[pos])));
        }
      }
      ++ledger.integral_rows;
    }

    ledger.nonzero_rhs += intrhs != 0 ? 1 : 0;
    int_system_index.push_back(n_orig);
    int_system_value.push_back(intrhs);
    int_system_start.push_back(static_cast<int>(int_system_value.size()));
    if (system_trace_limit > 0 &&
        static_cast<int>(integral_scales.size()) < system_trace_limit) {
      fmt::memory_buffer terms;
      const int system_row_end = static_cast<int>(int_system_value.size());
      const int emit_end =
          std::min(system_row_end, system_row_start + system_trace_terms);
      for (int p = system_row_start; p < emit_end; ++p) {
        if (p > system_row_start) fmt::format_to(std::back_inserter(terms), ",");
        fmt::format_to(std::back_inserter(terms), "{}:{}",
                       int_system_index[static_cast<std::size_t>(p)],
                       int_system_value[static_cast<std::size_t>(p)]);
      }
      if (system_row_end > emit_end) {
        fmt::format_to(std::back_inserter(terms), ",...");
      }
      fmt::print(stderr,
                 "[B&C-XMODK-SYS] seq={} row={} side={} rowIntegral={} "
                 "intscale={:.17g} intrhs={} rawLen={} transLen={} start={} "
                 "end={} terms=[{}]\n",
                 integral_scales.size(), row, leq_row ? "le" : "ge",
                 row_integral ? 1 : 0, intscale, intrhs, source_row_len,
                 rowlen, system_row_start, system_row_end,
                 fmt::to_string(terms));
    }
    integral_scales.emplace_back(row, intscale);
  }

  ledger.system_nnz = static_cast<int>(int_system_value.size());
  ledger.system_hash =
      xtab_modk_system_hash(int_system_value, int_system_index,
                            int_system_start);
  if (integral_scales.empty() || ledger.nonzero_rhs == 0) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("empty_system", 0);
    return 0;
  }

  int accepted_rows = 0;
  std::set<std::vector<std::pair<int, int>>> used_weights;
  XTabLpAggregator aggregator(simplex, source_context);

  auto found_cut = [&](auto& weights, int rhs_index, int k) {
    (void)rhs_index;
    if (weights.empty()) return;
    std::sort(weights.begin(), weights.end());
    std::vector<std::pair<int, int>> key;
    key.reserve(weights.size());
    for (const auto& w : weights) {
      key.emplace_back(static_cast<int>(w.index), static_cast<int>(w.weight));
    }
    if (!used_weights.insert(key).second) return;
    ++ledger.gf_solutions;

    auto generate_from_weights = [&](bool negated_weights) {
      aggregator.clear();
      for (const auto& w : weights) {
        const int idx = static_cast<int>(w.index);
        if (idx < 0 || idx >= static_cast<int>(integral_scales.size())) {
          continue;
        }
        const double base_scale =
            integral_scales[static_cast<std::size_t>(idx)].second;
        const int row = integral_scales[static_cast<std::size_t>(idx)].first;
        double weight = 0.0;
        if (negated_weights) {
          weight = base_scale *
                   (static_cast<double>((w.weight * (k - 1)) % k) /
                    static_cast<double>(k));
        } else {
          weight = base_scale *
                   (static_cast<double>(w.weight) / static_cast<double>(k));
        }
        (void)aggregator.add_row(row, weight);
      }
      Eigen::VectorXd coeff;
      double rhs = 0.0;
      std::vector<int> active_cols;
      if (!aggregator.get_current_aggregation(coeff, rhs,
                                              !negated_weights,
                                              &active_cols)) {
        return;
      }
      Eigen::VectorXd cut;
      double cut_rhs = 0.0;
      double efficacy = 0.0;
      XTabRejectReason reject_reason = XTabRejectReason::None;
      ++ledger.cutgen_calls;
      const bool ok = xtab_generate_cut_from_base_row(
          simplex, source_context, coeff, rhs, &active_cols, x,
          implied_integer_cols, &transform_context, &diag.vb_substitutions,
          &diag.vb_trigger_terms, cut, cut_rhs, efficacy, feastol,
          &reject_reason, &diag, /*require_violation=*/true, "modk",
          nullptr, &cutgen_random,
          /*only_initial_cmir_scale=*/true);
      if (!ok) {
        xtab_record_reject(&diag, reject_reason);
        return;
      }
      const int nnz = count_nonzeros(cut);
      const double norm = std::max(1e-12, cut.norm());
      if (cutpool_acceptor != nullptr) {
        PoolCut row_cut = xtab_candidate_to_pool_cut(
            XTabCandidateCut{std::move(cut), cut_rhs, efficacy, efficacy,
                             norm, nnz});
        const int added = (*cutpool_acceptor)(std::move(row_cut));
        ++ledger.pool_calls;
        if (added > 0) {
          accepted_rows += added;
          ledger.pool_accepted += added;
          ++ledger.cutgen_success;
          ++diag.generated;
        }
      } else if (generated_cutpool_rows != nullptr) {
        generated_cutpool_rows->push_back(xtab_candidate_to_pool_cut(
            XTabCandidateCut{std::move(cut), cut_rhs, efficacy, efficacy,
                             norm, nnz}));
        ++accepted_rows;
        ++ledger.cutgen_success;
        ++diag.generated;
      }
    };

    generate_from_weights(/*negated_weights=*/true);
    generate_from_weights(/*negated_weights=*/false);
    aggregator.clear();
  };

  ledger.gf2_calls = 1;
  int accepted_before_gf = accepted_rows;
  (void)xtab_separate_modk_system<2>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 2);
      });
  if (accepted_rows != accepted_before_gf) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("done", accepted_rows);
    return accepted_rows;
  }

  used_weights.clear();
  ledger.gf3_calls = 1;
  accepted_before_gf = accepted_rows;
  (void)xtab_separate_modk_system<3>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 3);
      });
  if (accepted_rows != accepted_before_gf) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("done", accepted_rows);
    return accepted_rows;
  }

  used_weights.clear();
  ledger.gf5_calls = 1;
  accepted_before_gf = accepted_rows;
  (void)xtab_separate_modk_system<5>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 5);
      });
  if (accepted_rows != accepted_before_gf) {
    maybe_print_xtab_diag("modk", diag);
    print_modk_ledger("done", accepted_rows);
    return accepted_rows;
  }

  used_weights.clear();
  ledger.gf7_calls = 1;
  (void)xtab_separate_modk_system<7>(
      int_system_value, int_system_index, int_system_start, n_orig,
      [&](auto& weights, int rhs_index) {
        found_cut(weights, rhs_index, 7);
      });

  maybe_print_xtab_diag("modk", diag);
  print_modk_ledger("done", accepted_rows);
  return accepted_rows;
}

int add_mir_like_cuts(LPModel& lp,
                      const Eigen::VectorXd& x,
                      CutType type,
                      int max_cuts) {
  int added = 0;
  const int n = static_cast<int>(lp.vars.size());

  std::vector<Eigen::VectorXd> cut_rows;
  std::vector<double> cut_rhs;

  for (int r = 0; r < lp.A.rows() && added < max_cuts; ++r) {
    const double frac_rhs = frac_part(lp.b[r]);
    if (frac_rhs <= 1e-8) {
      continue;
    }

    Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
    bool has_int = false;
    for (int j = 0; j < n; ++j) {
      const double a = lp.A.coeff(r, j);
      if (std::abs(a) <= 1e-12 || !is_integer_type(lp.vars[j])) {
        continue;
      }
      has_int = true;
      const double fj = frac_part(a);
      if (fj <= 1e-12) {
        continue;
      }
      if (type == CutType::IntRounding) {
        cut[j] = -fj;
      } else {
        const double lb = lp.vars[j].lb;
        const double ub = lp.vars[j].ub;
        if (std::isfinite(lb) && std::isfinite(ub) && ub > lb + 1e-9) {
          cut[j] = -fj / (ub - lb);
        } else {
          cut[j] = -fj;
        }
      }
    }

    if (!has_int) {
      continue;
    }
    const double rhs = -frac_rhs;
    const double viol = cut.dot(x) - rhs;
    if (viol <= 1e-7) {
      continue;
    }

    cut_rows.push_back(std::move(cut));
    cut_rhs.push_back(rhs);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_rows_to_lp(lp, cut_rows, cut_rhs);
  }
  return added;
}

int add_cover_cuts(LPModel& lp,
                   const Eigen::VectorXd& x,
                   int max_cuts) {
  if (max_cuts <= 0) {
    return 0;
  }

  int added = 0;
  const int n = static_cast<int>(lp.vars.size());

  std::vector<Eigen::VectorXd> cut_rows;
  std::vector<double> cut_rhs;

  // Build row-major view once to avoid O(nnz) per column access in scans.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_cover = lp.A;

  for (int r = 0; r < lp.A.rows() && added < max_cuts; ++r) {
    if (lp.b[r] <= 0.0) {
      continue;
    }

    std::vector<int> cand;
    cand.reserve(static_cast<size_t>(n));
    std::vector<double> cand_coeff;  // parallel to cand, cached coefficients
    cand_coeff.reserve(static_cast<size_t>(n));
    bool valid_row = true;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_cover, r); it; ++it) {
      const double a = it.value();
      if (std::abs(a) <= 1e-12) {
        continue;
      }
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= n) { valid_row = false; break; }
      if (a < 0.0 || lp.vars[j].type != VarType::Binary) {
        valid_row = false;
        break;
      }
      cand.push_back(j);
      cand_coeff.push_back(a);
    }

    if (!valid_row || cand.size() < 2) {
      continue;
    }

    // Sort cand by coefficient descending using cached values (avoids repeated sparse lookups).
    std::vector<int> order(cand.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return cand_coeff[a] > cand_coeff[b];
    });
    std::vector<int> sorted_cand(cand.size());
    std::vector<double> sorted_coeff(cand.size());
    for (int i = 0; i < static_cast<int>(order.size()); ++i) {
      sorted_cand[i] = cand[order[i]];
      sorted_coeff[i] = cand_coeff[order[i]];
    }
    cand = sorted_cand;
    cand_coeff = sorted_coeff;

    // Build a coefficient lookup map for this row (for the knapsack lifting phase).
    std::unordered_map<int, double> row_coeffs;
    row_coeffs.reserve(cand.size());
    for (int i = 0; i < static_cast<int>(cand.size()); ++i) {
      row_coeffs[cand[i]] = cand_coeff[i];
    }

    std::vector<int> cover;
    cover.reserve(cand.size());
    double sum_w = 0.0;
    for (int i = 0; i < static_cast<int>(cand.size()); ++i) {
      cover.push_back(cand[i]);
      sum_w += cand_coeff[i];
      if (sum_w > lp.b[r] + 1e-9) {
        break;
      }
    }

    if (sum_w <= lp.b[r] + 1e-9 || cover.size() < 2) {
      continue;
    }

    const int cover_size = static_cast<int>(cover.size());
    Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
    for (int j : cover) cut[j] = 1.0;
    double rhs = static_cast<double>(cover_size) - 1.0;

    // Sequential up-lifting of non-cover variables for small rows.
    // For each non-cover variable k (decreasing weight), solve DP knapsack
    // to compute the tightest valid lifting coefficient.
    if (cand.size() <= 40 && cand.size() > cover.size()) {
      const double b_row = lp.b[r];
      std::vector<bool> in_cover(n, false);
      for (int j : cover) in_cover[j] = true;

      std::vector<int> non_cover;
      for (int k : cand) {
        if (!in_cover[k]) non_cover.push_back(k);
      }
      std::sort(non_cover.begin(), non_cover.end(), [&](int a, int b_) {
        return row_coeffs.at(a) > row_coeffs.at(b_);
      });

      struct KSItem { double weight; double coeff; };
      std::vector<KSItem> items;
      items.reserve(cand.size());
      for (int j : cover) items.push_back({row_coeffs.at(j), 1.0});

      constexpr int N_BINS = 512;  // Smaller table for speed
      std::vector<double> dp(N_BINS + 1);
      auto solve_ks = [&](const std::vector<KSItem>& its, double cap) -> double {
        if (cap < -1e-9 || its.empty()) return 0.0;
        double scale = (cap > 1e-12) ? static_cast<double>(N_BINS) / cap : 0.0;
        std::fill(dp.begin(), dp.end(), 0.0);
        for (const auto& item : its) {
          int w_disc = static_cast<int>(std::floor(item.weight * scale));
          if (w_disc <= 0) { for (int w = 0; w <= N_BINS; ++w) dp[w] += item.coeff; continue; }
          for (int w = N_BINS; w >= w_disc; --w) {
            double c = dp[w - w_disc] + item.coeff;
            if (c > dp[w]) dp[w] = c;
          }
        }
        return dp[N_BINS];
      };

      for (int k : non_cover) {
        double a_k = row_coeffs.at(k);
        double cap = b_row - a_k;
        double z_star = (cap < -1e-9) ? 0.0 : solve_ks(items, cap);
        double alpha_k = rhs - z_star;
        if (alpha_k > 1e-9) cut[k] = alpha_k;
        items.push_back({a_k, std::max(alpha_k, 0.0)});
      }
    }

    const double viol = cut.dot(x) - rhs;
    if (viol <= 1e-7) {
      continue;
    }

    cut_rows.push_back(std::move(cut));
    cut_rhs.push_back(rhs);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_rows_to_lp(lp, cut_rows, cut_rhs);
  }
  return added;
}

int add_basis_mir_cuts(LPModel& lp,
                       const Eigen::VectorXd& x,
                       [[maybe_unused]] const SimplexResult& simplex,
                       [[maybe_unused]] const BCOptions& opt,
                       int max_cuts,
                       [[maybe_unused]] const std::shared_ptr<BasisOps>& sbasis) {
  // True single-row complemented MIR cuts.
  // For each inequality row: sum_j a_j x_j <= b
  //   Separate integer vars I and continuous vars C.
  //   Complement integer vars near their upper bound: x'_j = ub_j - x_j
  //   Apply MIR rounding: let f0 = frac(b'), for integer j with f_j > f0:
  //     coeff_j = floor(a_j) + (frac(a_j) - f0)/(1-f0)
  //   For continuous vars with a_j > 0: coeff_j = a_j / (1-f0)
  //   For continuous vars with a_j < 0: coeff_j = 0 (dropped)
  if (max_cuts <= 0) {
    return 0;
  }

  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());

  struct MIRCandidate {
    Eigen::VectorXd coeff;
    double rhs;
    double violation;
    double norm;
  };
  std::vector<MIRCandidate> candidates;
  candidates.reserve(static_cast<size_t>(std::min(m_ineq + m_eq, max_cuts * 4)));

  // Build RowMajor views once to avoid O(n log nnz) coeff() lookups per row.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_mir_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_mir_row = lp.Aeq;

  auto try_mir_row = [&](auto&& visit_nonzeros, double b_val, int /*row_idx*/) {
    // Collect row coefficients via RowMajor InnerIterator (O(nnz), not O(n log nnz)).
    Eigen::VectorXd a = Eigen::VectorXd::Zero(n);
    bool has_integer = false;
    [[maybe_unused]] bool has_continuous = false;
    visit_nonzeros([&](int j, double val) {
      if (j >= 0 && j < n && std::abs(val) > 1e-12) {
        a[j] = val;
        if (is_integer_type(lp.vars[j])) {
          has_integer = true;
        } else {
          has_continuous = true;
        }
      }
    });
    if (!has_integer) return;  // pure continuous row - no MIR possible

    // Try complementing integer vars near upper bound
    Eigen::VectorXd a_comp = a;
    double b_comp = b_val;
    for (int j = 0; j < n; ++j) {
      if (std::abs(a[j]) <= 1e-12 || !is_integer_type(lp.vars[j])) continue;
      double ub = lp.vars[j].ub;
      if (!std::isfinite(ub)) continue;
      // Complement if x_j closer to ub than lb.
      // Guard against infinite lb: if lb is -inf, always treat as closer to lb
      // (no complementation), since the midpoint is undefined.
      double lb = lp.vars[j].lb;
      if (!std::isfinite(lb)) continue;
      double mid = (lb + ub) * 0.5;
      if (x[j] > mid) {
        // Replace x_j with (ub - x'_j): a_j*x_j = a_j*ub - a_j*x'_j
        a_comp[j] = -a[j];
        b_comp -= a[j] * ub;
      }
    }

    double f0 = frac_part(b_comp);
    if (f0 <= 1e-8 || f0 >= 1.0 - 1e-8) return;

    // Build MIR cut: sum mir_j * x'_j <= floor(b')
    Eigen::VectorXd mir = Eigen::VectorXd::Zero(n);
    double mir_rhs = std::floor(b_comp);
    bool has_term = false;

    for (int j = 0; j < n; ++j) {
      double aj = a_comp[j];
      if (std::abs(aj) <= 1e-12) continue;

      if (is_integer_type(lp.vars[j])) {
        double fj = frac_part(aj);
        if (fj <= f0 + 1e-10) {
          mir[j] = std::floor(aj);
        } else {
          mir[j] = std::floor(aj) + (fj - f0) / (1.0 - f0);
        }
      } else {
        // Continuous: a_j > 0 → 0 (absorbed into slack),
        //             a_j < 0 → a_j/(1-f0) (strengthened in MIR)
        if (aj < 0) {
          mir[j] = aj / (1.0 - f0);
        } else {
          mir[j] = 0.0;  // Absorbed into non-negative slack
        }
      }
      if (std::abs(mir[j]) > 1e-12) has_term = true;
    }
    if (!has_term) return;

    // Un-complement: convert back to original variables
    Eigen::VectorXd cut_orig = Eigen::VectorXd::Zero(n);
    double rhs_orig = mir_rhs;
    for (int j = 0; j < n; ++j) {
      if (std::abs(mir[j]) <= 1e-12) continue;
      bool complemented = (is_integer_type(lp.vars[j]) &&
                           std::isfinite(lp.vars[j].ub) &&
                           std::isfinite(lp.vars[j].lb) &&
                           x[j] > (lp.vars[j].lb + lp.vars[j].ub) * 0.5);
      if (complemented) {
        // mir_j * x'_j = mir_j * (ub - x_j) = -mir_j * x_j + mir_j * ub
        cut_orig[j] = -mir[j];
        rhs_orig -= mir[j] * lp.vars[j].ub;
      } else {
        cut_orig[j] = mir[j];
      }
    }

    // Check violation of original (<=) form
    double lhs = cut_orig.dot(x);
    double violation = lhs - rhs_orig;
    if (violation <= 1e-7) return;

    double norm = cut_orig.norm();
    if (norm < 1e-12) return;
    double efficacy = violation / norm;
    if (efficacy < 5e-4) return;

    // Density filter — P6.3: scale cap down for large problems
    int nnz = 0;
    for (int j = 0; j < n; ++j) {
      if (std::abs(cut_orig[j]) > 1e-12) ++nnz;
    }
    const double mir_density_cap = (n > 20000) ? 0.10 : (n > 10000) ? 0.30 : 0.60;
    if (n > 0 && static_cast<double>(nnz) / n > mir_density_cap) return;

    candidates.push_back({std::move(cut_orig), rhs_orig, violation, norm});
  };

  // Process inequality rows
  for (int r = 0; r < m_ineq; ++r) {
    try_mir_row([&](auto&& f) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_mir_row, r); it; ++it)
        f(static_cast<int>(it.col()), it.value());
    }, lp.b[r], r);
  }
  // Process equality rows as two inequalities (relaxed as <=)
  for (int r = 0; r < m_eq; ++r) {
    try_mir_row([&](auto&& f) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_mir_row, r); it; ++it)
        f(static_cast<int>(it.col()), it.value());
    }, lp.beq[r], m_ineq + r);
    // Also try negated form
    try_mir_row([&](auto&& f) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_mir_row, r); it; ++it)
        f(static_cast<int>(it.col()), -it.value());
    }, -lp.beq[r], m_ineq + m_eq + r);
  }

  if (candidates.empty()) return 0;

  // Sort by violation (efficacy) and select diverse cuts
  std::sort(candidates.begin(), candidates.end(),
            [](const MIRCandidate& a, const MIRCandidate& b) {
              return a.violation > b.violation;
            });

  int added = 0;
  std::vector<Eigen::VectorXd> cut_rows;
  std::vector<double> cut_rhs;
  std::vector<double> cut_norms;

  for (const auto& cand : candidates) {
    if (added >= max_cuts) break;
    // Parallelism filter against already selected
    // P2.2: Use stricter threshold (0.9) for better orthogonality.
    bool parallel = false;
    for (size_t si = 0; si < cut_rows.size(); ++si) {
      double cos_val = std::abs(cand.coeff.dot(cut_rows[si])) / (cand.norm * cut_norms[si]);
      if (cos_val > 0.90) { parallel = true; break; }
    }
    if (parallel) continue;

    cut_norms.push_back(cand.coeff.norm());
    cut_rows.push_back(cand.coeff);
    cut_rhs.push_back(cand.rhs);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_rows_to_lp(lp, cut_rows, cut_rhs);
  }
  return added;
}

/// Flow cover cuts for constraints of the form:
///   sum_j (a_j * x_j) + sum_k (d_k * y_k) <= b
/// where y_k are binary and x_j continuous with variable upper bounds x_j <= u_j * y_k.
/// These are very effective for SCUC problems where continuous generation
/// is linked to binary commitment decisions.
int add_flow_cover_cuts(LPModel& lp, const Eigen::VectorXd& x, int max_cuts) {
  if (max_cuts <= 0) return 0;

  const int n = static_cast<int>(lp.vars.size());
  const int m = static_cast<int>(lp.A.rows());

  struct FlowCoverCut {
    Eigen::VectorXd coeff;
    double rhs;
    double violation;
  };
  std::vector<FlowCoverCut> candidates;

  // Build RowMajor view once to avoid O(n log nnz) coeff() lookups per row.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_flow_row = lp.A;

  for (int r = 0; r < m; ++r) {
    // Identify rows with mixed binary+continuous structure.
    // Cache (col, val) pairs to avoid repeated O(log nnz) coeff() lookups below.
    struct ColCoeff { int col; double val; };
    std::vector<ColCoeff> bin_entries, cont_entries;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_flow_row, r);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      const double val = it.value();
      if (j < 0 || j >= n || std::abs(val) <= 1e-12) continue;
      if (lp.vars[j].type == VarType::Binary) {
        bin_entries.push_back({j, val});
      } else if (lp.vars[j].type == VarType::Continuous) {
        cont_entries.push_back({j, val});
      }
    }
    if (bin_entries.empty() || cont_entries.empty()) continue;

    // Identify N+ (positive coeff binary) and N- (negative coeff binary)
    std::vector<ColCoeff> Nplus_entries, Nminus_entries;
    for (const auto& e : bin_entries) {
      if (e.val > 1e-12) Nplus_entries.push_back(e);
      else if (e.val < -1e-12) Nminus_entries.push_back(e);
    }

    if (Nminus_entries.empty()) continue;  // Need negative binary coefficients for flow cover

    // Compute: lambda = b + sum_{j in N-} |a_j| (capacity when all N- = 1)
    double lambda = lp.b[r];
    for (const auto& e : Nminus_entries) {
      lambda += std::abs(e.val);
    }
    if (lambda <= 1e-9) continue;

    // Find a cover C subset of N- such that:
    //   sum_{j in C} |a_j| > lambda  (exceeds total capacity)
    // Greedy: sort by |a_j * (1 - x_j)| descending (most violated first)
    struct BinInfo {
      int col;
      double abs_coeff;
      double viol_score;
    };
    std::vector<BinInfo> nminus_info;
    nminus_info.reserve(Nminus_entries.size());
    for (const auto& e : Nminus_entries) {
      double ac = std::abs(e.val);
      nminus_info.push_back({e.col, ac, ac * (1.0 - x[e.col])});
    }
    std::sort(nminus_info.begin(), nminus_info.end(),
              [](const BinInfo& a, const BinInfo& b) { return a.viol_score > b.viol_score; });

    std::vector<int> cover;
    std::vector<double> cover_abs;  // parallel absolute coefficients
    double cover_sum = 0.0;
    for (const auto& bi : nminus_info) {
      cover.push_back(bi.col);
      cover_abs.push_back(bi.abs_coeff);
      cover_sum += bi.abs_coeff;
      if (cover_sum > lambda + 1e-9) break;
    }
    if (cover_sum <= lambda + 1e-9) continue;  // No cover found

    // Flow cover inequality:
    //   sum_{j in cont with a_j > 0} a_j * x_j
    //   + sum_{j in N+ with a_j > 0} min(a_j, lambda) * y_j
    //   - sum_{j in C} max(0, |a_j| - (cover_sum - lambda)) * y_j
    //   <= lambda - sum_{j in C} max(0, |a_j| - (cover_sum - lambda))
    double excess = cover_sum - lambda;

    Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
    double cut_rhs = lambda;

    // Continuous variables with positive coefficients (use cached values)
    for (const auto& e : cont_entries) {
      if (e.val > 1e-12) {
        cut[e.col] = e.val;
      }
    }

    // Positive binary variables: coefficient = min(a_j, lambda) (use cached values)
    for (const auto& e : Nplus_entries) {
      cut[e.col] = std::min(e.val, lambda);
    }

    // Cover variables: subtract lifting coefficient (use parallel cover_abs)
    for (int ci = 0; ci < static_cast<int>(cover.size()); ++ci) {
      double aj = cover_abs[static_cast<std::size_t>(ci)];
      double lift = std::max(0.0, aj - excess);
      cut[cover[static_cast<std::size_t>(ci)]] = -lift;
      cut_rhs -= lift;
    }

    // Check violation
    double viol = cut.dot(x) - cut_rhs;
    if (viol <= 1e-7) continue;

    candidates.push_back({std::move(cut), cut_rhs, viol});
  }

  if (candidates.empty()) return 0;

  std::sort(candidates.begin(), candidates.end(),
            [](const FlowCoverCut& a, const FlowCoverCut& b) { return a.violation > b.violation; });

  int added = 0;
  std::vector<Eigen::VectorXd> cut_rows;
  std::vector<double> cut_rhs_vec;
  for (const auto& c : candidates) {
    if (added >= max_cuts) break;
    cut_rows.push_back(c.coeff);
    cut_rhs_vec.push_back(c.rhs);
    ++added;
  }
  if (!cut_rows.empty()) {
    add_rows_to_lp(lp, cut_rows, cut_rhs_vec);
  }
  return added;
}

int add_projected_capacity_cover_cuts(LPModel& lp,
                                      const Eigen::VectorXd& x,
                                      int max_cuts) {
  if (max_cuts <= 0 || lp.Aeq.rows() == 0 || lp.A.rows() == 0) return 0;
  const int n = static_cast<int>(lp.vars.size());
  if (x.size() < n) return 0;

  struct Link {
    int bin{-1};
    double ub{0.0};
  };
  std::vector<Link> vub(static_cast<std::size_t>(n));

  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
  for (int r = 0; r < A_row.rows(); ++r) {
    int cont = -1;
    int bin = -1;
    double a_cont = 0.0;
    double a_bin = 0.0;
    int nnz = 0;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
      if (std::abs(it.value()) <= 1e-12) continue;
      ++nnz;
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= n) continue;
      if (!is_integer_type(lp.vars[j]) && it.value() > 0.0) {
        cont = j;
        a_cont = it.value();
      } else if (lp.vars[j].type == VarType::Binary && it.value() < 0.0) {
        bin = j;
        a_bin = it.value();
      }
    }
    if (nnz == 2 && cont >= 0 && bin >= 0 && a_cont > 1e-12 &&
        a_bin < -1e-12 && std::abs(lp.b[r]) <= 1e-8) {
      const double ub = -a_bin / a_cont;
      if (std::isfinite(ub) && ub > 1e-9) {
        vub[static_cast<std::size_t>(cont)] = Link{bin, ub};
      }
    }
  }

  struct Item {
    int bin;
    double cap;
    double value;
  };

  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;
  std::vector<Eigen::VectorXd> rows;
  std::vector<double> rhs;
  rows.reserve(static_cast<std::size_t>(max_cuts));
  rhs.reserve(static_cast<std::size_t>(max_cuts));

  for (int r = 0; r < Aeq_row.rows() && static_cast<int>(rows.size()) < max_cuts; ++r) {
    const double demand = lp.beq[r];
    if (!(demand > 1e-9) || !std::isfinite(demand)) continue;

    bool usable = true;
    std::vector<Item> items;
    double total_cap = 0.0;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
      const int j = static_cast<int>(it.col());
      const double a = it.value();
      if (j < 0 || j >= n || std::abs(a) <= 1e-12) continue;
      if (a <= 0.0 || is_integer_type(lp.vars[j])) {
        usable = false;
        break;
      }
      const Link link = vub[static_cast<std::size_t>(j)];
      if (link.bin < 0) {
        usable = false;
        break;
      }
      const double cap = a * link.ub;
      if (!(cap > 1e-9) || !std::isfinite(cap)) {
        usable = false;
        break;
      }
      items.push_back(Item{link.bin, cap, x[link.bin]});
      total_cap += cap;
    }
    if (!usable || items.size() < 2 || total_cap <= demand + 1e-8) continue;

    {
      std::vector<double> caps;
      caps.reserve(items.size());
      double activity = 0.0;
      for (const auto& item : items) {
        caps.push_back(item.cap);
        activity += item.value;
      }
      std::sort(caps.begin(), caps.end(), std::greater<double>());
      double cap_prefix = 0.0;
      int min_count = 0;
      while (min_count < static_cast<int>(caps.size()) &&
             cap_prefix + 1e-8 < demand) {
        cap_prefix += caps[static_cast<std::size_t>(min_count)];
        ++min_count;
      }
      if (cap_prefix + 1e-8 >= demand && min_count > 0 &&
          activity < static_cast<double>(min_count) - 1e-7 &&
          static_cast<int>(rows.size()) < max_cuts) {
        Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
        for (const auto& item : items) cut[item.bin] = -1.0;
        rows.push_back(std::move(cut));
        rhs.push_back(-static_cast<double>(min_count));
      }
    }
    if (static_cast<int>(rows.size()) >= max_cuts) break;

    const double cover_threshold = total_cap - demand;
    std::sort(items.begin(), items.end(), [](const Item& lhs, const Item& rhs) {
      const double rl = lhs.value / std::max(1e-9, lhs.cap);
      const double rr = rhs.value / std::max(1e-9, rhs.cap);
      if (std::abs(rl - rr) > 1e-12) return rl < rr;
      return lhs.value < rhs.value;
    });

    const int seed_limit = std::min<int>(static_cast<int>(items.size()), 8);
    for (int seed = 0; seed < seed_limit && static_cast<int>(rows.size()) < max_cuts; ++seed) {
      double cover_cap = 0.0;
      double cover_activity = 0.0;
      std::vector<int> cover_bins;
      auto take = [&](const Item& item) {
        if (std::find(cover_bins.begin(), cover_bins.end(), item.bin) != cover_bins.end()) return;
        cover_cap += item.cap;
        cover_activity += item.value;
        cover_bins.push_back(item.bin);
      };
      take(items[static_cast<std::size_t>(seed)]);
      for (int pos = 0; pos < static_cast<int>(items.size()) &&
                        cover_cap <= cover_threshold + 1e-8; ++pos) {
        take(items[static_cast<std::size_t>(pos)]);
      }
      if (cover_cap <= cover_threshold + 1e-8) continue;
      if (cover_activity >= 1.0 - 1e-7) continue;

      std::sort(cover_bins.begin(), cover_bins.end());
      cover_bins.erase(std::unique(cover_bins.begin(), cover_bins.end()), cover_bins.end());
      if (cover_bins.empty()) continue;

      Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
      for (int j : cover_bins) cut[j] = -1.0;
      rows.push_back(std::move(cut));
      rhs.push_back(-1.0);
    }
  }

  if (!rows.empty()) {
    add_rows_to_lp(lp, rows, rhs);
  }
  return static_cast<int>(rows.size());
}

/// Implied bound cuts from variable-bound constraints.
/// Detects constraints of the form: x_j <= M * y_k + c (or >= form)
/// where y_k is binary and x_j is continuous/integer.
/// Generates tighter bounds when LP solution is fractional.
int add_implied_bound_cuts(LPModel& lp, const Eigen::VectorXd& x, int max_cuts) {
  if (max_cuts <= 0) return 0;

  const int n = static_cast<int>(lp.vars.size());
  const int m = static_cast<int>(lp.A.rows());

  struct IBCut {
    Eigen::VectorXd coeff;
    double rhs;
    double violation;
  };
  std::vector<IBCut> candidates;

  // Build a row-major view once to avoid O(nnz) per column access in the inner loop.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;

  // Scan for variable bound rows: exactly one continuous and one binary variable
  for (int r = 0; r < m; ++r) {
    int cont_col = -1, bin_col = -1;
    double cont_coeff = 0.0, bin_coeff = 0.0;
    int nnz = 0;
    bool valid = true;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
      const double val = it.value();
      if (std::abs(val) <= 1e-12) continue;
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= n) { valid = false; break; }
      ++nnz;
      if (nnz > 2) { valid = false; break; }
      if (lp.vars[j].type == VarType::Binary) {
        if (bin_col >= 0) { valid = false; break; }
        bin_col = j; bin_coeff = val;
      } else {
        if (cont_col >= 0) { valid = false; break; }
        cont_col = j; cont_coeff = val;
      }
    }
    if (!valid || nnz != 2 || cont_col < 0 || bin_col < 0) continue;

    // Row: cont_coeff * x_cont + bin_coeff * y_bin <= b
    // Case 1: cont_coeff > 0, bin_coeff < 0 → x <= (b - bin_coeff*y) / cont_coeff
    //   When y=1: x <= (b - bin_coeff) / cont_coeff = tighter UB
    //   When y=0: x <= b / cont_coeff
    // Generate implied bound cut from other rows that use these variables

    // For each other row containing cont_col, try coefficient strengthening
    double y_val = x[bin_col];
    [[maybe_unused]] double x_val = x[cont_col];
    if (y_val < 1e-6 || y_val > 1.0 - 1e-6) continue;  // Binary already nearly integral

    // Upper bound on x when y=0 and y=1
    double ub_y0, ub_y1;
    if (cont_coeff > 1e-12) {
      ub_y0 = lp.b[r] / cont_coeff;
      ub_y1 = (lp.b[r] - bin_coeff) / cont_coeff;
    } else if (cont_coeff < -1e-12) {
      // Negative continuous coeff: it's a lower bound constraint
      // -|c| * x + d * y <= b → x >= (d*y - b)/|c|
      ub_y0 = std::numeric_limits<double>::infinity();
      ub_y1 = std::numeric_limits<double>::infinity();
      continue;
    } else {
      continue;
    }

    if (!std::isfinite(ub_y0) || !std::isfinite(ub_y1)) continue;
    if (ub_y1 >= ub_y0 - 1e-9) continue;  // No strengthening possible

    // Generate the well-known "VUB" cut: x <= ub_y1 * y + ub_y0 * (1-y)
    //   x <= ub_y0 + (ub_y1 - ub_y0) * y
    //   x - (ub_y1 - ub_y0) * y <= ub_y0
    Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
    cut[cont_col] = 1.0;
    cut[bin_col] = -(ub_y1 - ub_y0);  // Note: ub_y1 < ub_y0, so this is positive
    double cut_rhs = ub_y0;

    double viol = cut.dot(x) - cut_rhs;
    if (viol <= 1e-7) continue;

    candidates.push_back({std::move(cut), cut_rhs, viol});
  }

  if (candidates.empty()) return 0;

  std::sort(candidates.begin(), candidates.end(),
            [](const IBCut& a, const IBCut& b) { return a.violation > b.violation; });

  int added = 0;
  std::vector<Eigen::VectorXd> cut_rows;
  std::vector<double> cut_rhs_vec;
  for (const auto& c : candidates) {
    if (added >= max_cuts) break;
    cut_rows.push_back(c.coeff);
    cut_rhs_vec.push_back(c.rhs);
    ++added;
  }
  if (!cut_rows.empty()) {
    add_rows_to_lp(lp, cut_rows, cut_rhs_vec);
  }
  return added;
}

/// Basis-free MIR cuts that work without simplex basis (e.g., after IPM root).
/// Applies single-row complemented MIR directly to LP constraints.
int add_row_mir_cuts(LPModel& lp, const Eigen::VectorXd& x, int max_cuts) {
  if (max_cuts <= 0) return 0;

  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());

  struct MIRCand {
    Eigen::VectorXd coeff;
    double rhs;
    double violation;
  };
  std::vector<MIRCand> candidates;

  // Build RowMajor views once to avoid O(n log nnz) coeff() lookups per row.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_mir = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row_mir = lp.Aeq;

  auto try_row = [&](auto&& visit_nonzeros, double b_val) {
    Eigen::VectorXd a = Eigen::VectorXd::Zero(n);
    bool has_int = false;
    visit_nonzeros([&](int j, double val) {
      if (j >= 0 && j < n && std::abs(val) > 1e-12) {
        a[j] = val;
        if (is_integer_type(lp.vars[j])) has_int = true;
      }
    });
    if (!has_int) return;

    // Complement integer vars near upper bound
    Eigen::VectorXd ac = a;
    double bc = b_val;
    for (int j = 0; j < n; ++j) {
      if (std::abs(a[j]) <= 1e-12 || !is_integer_type(lp.vars[j])) continue;
      double ub = lp.vars[j].ub;
      if (!std::isfinite(ub)) continue;
      if (!std::isfinite(lp.vars[j].lb)) continue;
      if (x[j] > (lp.vars[j].lb + ub) * 0.5) {
        ac[j] = -a[j];
        bc -= a[j] * ub;
      }
    }

    double f0 = frac_part(bc);
    if (f0 <= 1e-8 || f0 >= 1.0 - 1e-8) return;

    Eigen::VectorXd mir = Eigen::VectorXd::Zero(n);
    double mir_rhs = std::floor(bc);
    bool has_term = false;
    for (int j = 0; j < n; ++j) {
      if (std::abs(ac[j]) <= 1e-12) continue;
      if (is_integer_type(lp.vars[j])) {
        double fj = frac_part(ac[j]);
        mir[j] = (fj <= f0 + 1e-10) ? std::floor(ac[j]) : (std::floor(ac[j]) + (fj - f0) / (1.0 - f0));
      } else {
        // Continuous: a < 0 → a/(1-f0) (strengthened), a > 0 → 0 (absorbed into slack)
        mir[j] = (ac[j] < 0) ? ac[j] / (1.0 - f0) : 0.0;
      }
      if (std::abs(mir[j]) > 1e-12) has_term = true;
    }
    if (!has_term) return;

    // Un-complement
    Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
    double rhs = mir_rhs;
    for (int j = 0; j < n; ++j) {
      if (std::abs(mir[j]) <= 1e-12) continue;
      bool comp = (is_integer_type(lp.vars[j]) && std::isfinite(lp.vars[j].ub) &&
                   std::isfinite(lp.vars[j].lb) &&
                   x[j] > (lp.vars[j].lb + lp.vars[j].ub) * 0.5);
      if (comp) { cut[j] = -mir[j]; rhs -= mir[j] * lp.vars[j].ub; }
      else { cut[j] = mir[j]; }
    }

    double viol = cut.dot(x) - rhs;
    if (viol <= 1e-7 || cut.norm() < 1e-12) return;
    candidates.push_back({std::move(cut), rhs, viol});
  };

  for (int r = 0; r < m_ineq; ++r) {
    try_row([&](auto&& f) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_mir, r); it; ++it)
        f(static_cast<int>(it.col()), it.value());
    }, lp.b[r]);
  }
  for (int r = 0; r < m_eq; ++r) {
    try_row([&](auto&& f) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row_mir, r); it; ++it)
        f(static_cast<int>(it.col()), it.value());
    }, lp.beq[r]);
    try_row([&](auto&& f) {
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row_mir, r); it; ++it)
        f(static_cast<int>(it.col()), -it.value());
    }, -lp.beq[r]);
  }

  if (candidates.empty()) return 0;
  std::sort(candidates.begin(), candidates.end(),
            [](const MIRCand& a, const MIRCand& b) { return a.violation > b.violation; });

  int added = 0;
  std::vector<Eigen::VectorXd> rows;
  std::vector<double> rhs_vec;
  for (const auto& c : candidates) {
    if (added >= max_cuts) break;
    rows.push_back(c.coeff);
    rhs_vec.push_back(c.rhs);
    ++added;
  }
  if (!rows.empty()) add_rows_to_lp(lp, rows, rhs_vec);
  return added;
}

}  // anonymous namespace

int add_transformed_modk_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    const std::vector<char>* implied_integer_cols,
    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table,
    std::vector<PoolCut>* generated_cutpool_rows,
    double cut_generation_feastol,
    const std::function<int(PoolCut&&)>* cutpool_acceptor,
    std::optional<std::uint64_t> highs_cutgen_seed) {
  return add_transformed_modk_cuts_impl(
      lp, x, simplex, opt, implied_integer_cols, implication_graph,
      variable_bound_table, generated_cutpool_rows, cut_generation_feastol,
      cutpool_acceptor, highs_cutgen_seed);
}

int add_basis_gomory_cuts(LPModel& lp,
                          const Eigen::VectorXd& x,
                          const SimplexResult& simplex,
                          const BCOptions& opt,
                          int max_cuts,
                          const std::shared_ptr<BasisOps>& sbasis) {
  if (max_cuts <= 0 || !simplex.exact_optimal) {
    return 0;
  }
  if (!basis_tableau_cuts_admissible(simplex, opt, sbasis)) {
    return 0;
  }

  const int m = static_cast<int>(simplex.basis.indices.size());
  if (!sbasis && simplex.basis_inverse.rows() != m) {
    return 0;
  }

  struct CandidateCut {
    Eigen::VectorXd coeff;
    double rhs{0.0};
    double score{0.0};
    double efficacy{0.0};
    double activity{0.0};
    double norm{0.0};
    int nnz{0};
  };

  const int n = static_cast<int>(x.size());
  const double density_cap = std::min(1.0, std::max(0.01, opt.gmi_max_density));
  const int max_nnz = std::max(2, static_cast<int>(std::ceil(density_cap * static_cast<double>(n))));
  const double min_efficacy = std::max(0.0, opt.gmi_min_efficacy);
  const double min_activity = std::min(1.0, std::max(0.0, opt.gmi_min_activity));
  const double min_binary_support = std::min(1.0, std::max(0.0, opt.gmi_min_binary_support));
  const double activity_weight = std::max(0.0, opt.gmi_activity_weight);

  std::vector<CandidateCut> candidates;
  candidates.reserve(static_cast<size_t>(max_cuts * 3));

  // Pre-filter: identify fractional integer basis rows and sort by
  // fractionality (most fractional first = closest to 0.5).  Only
  // evaluate the top candidates to limit expensive BTRAN computations.
  // For large m (e.g. 6884), this reduces BTRAN count from ~200 to ~60,
  // saving ~150ms in cut generation.
  struct FracRow { int row; double frac; };
  std::vector<FracRow> frac_rows;
  frac_rows.reserve(static_cast<size_t>(m));
  for (int row = 0; row < m; ++row) {
    const int basic_col = simplex.basis.indices[static_cast<size_t>(row)];
    if (!transformed_original_var_is_integer(simplex, basic_col)) continue;
    const double f = frac_part(
        original_space_value(simplex, basic_col, simplex.x_basic[row]));
    if (f <= 1e-8 || f >= 1.0 - 1e-8) continue;
    frac_rows.push_back({row, f});
  }
  // Sort by distance from 0.5 ascending (most fractional first).
  std::sort(frac_rows.begin(), frac_rows.end(),
            [](const FracRow& a, const FracRow& b) {
              return std::abs(a.frac - 0.5) < std::abs(b.frac - 0.5);
            });
  // Evaluate at most 3× budget candidates (enough for good diversity).
  const int eval_limit = std::min(static_cast<int>(frac_rows.size()),
                                  std::max(max_cuts * 3, 30));
  if (static_cast<int>(frac_rows.size()) > eval_limit)
    frac_rows.resize(static_cast<size_t>(eval_limit));

  // Build is_basic once here and pass it down to avoid one O(n_std) allocation
  // + fill per fractional row (can be hundreds per cut round).
  const int n_std_gmi = static_cast<int>(simplex.form.A.cols());
  std::vector<char> is_basic_cache(static_cast<size_t>(n_std_gmi), 0);
  for (int idx : simplex.basis.indices) {
    if (idx >= 0 && idx < n_std_gmi)
      is_basic_cache[static_cast<size_t>(idx)] = 1;
  }
  const std::vector<char>* is_basic_ptr = &is_basic_cache;

  for (const auto& fr : frac_rows) {
    Eigen::VectorXd cut;
    double rhs = 0.0;
    if (!build_bounded_form_gmi_cut(simplex, fr.row, cut, rhs, sbasis, is_basic_ptr)) {
      continue;
    }
    const double viol = cut.dot(x) - rhs;
    if (viol <= 1e-7) {
      continue;
    }

    const int nnz = count_nonzeros(cut);
    if (nnz > max_nnz) {
      continue;
    }

    const double norm = std::max(1e-12, cut.norm());
    const double efficacy = viol / norm;
    if (efficacy < min_efficacy) {
      continue;
    }

    const double activity = gmi_binary_activity_score(simplex, x, cut);
    if (activity < min_activity) {
      continue;
    }

    const double binary_support = gmi_binary_support_ratio(simplex, cut);
    if (binary_support < min_binary_support) {
      continue;
    }

    const double score = efficacy * (1.0 + activity_weight * activity);
    candidates.push_back(CandidateCut{cut, rhs, score, efficacy, activity, norm, nnz});

    // Tableau-row MIR cut with strategic complementation.
    // Currently disabled: while mathematically correct, MIR cuts are changing
    // the root LP landscape in ways that hurt tree search performance
    // (triggering cold simplex restarts). Net effect is negative.
    // TODO: Re-enable with tighter candidate filtering.
#if 0
    Eigen::VectorXd mir_cut;
    double mir_rhs = 0.0;
    if (build_bounded_form_mir_cut(simplex, fr.row, x, mir_cut, mir_rhs, sbasis)) {
      const double mir_viol = mir_cut.dot(x) - mir_rhs;
      if (mir_viol > 1e-7) {
        const int mir_nnz = count_nonzeros(mir_cut);
        if (mir_nnz <= max_nnz) {
          const double mir_norm = std::max(1e-12, mir_cut.norm());
          const double mir_eff = mir_viol / mir_norm;
          if (mir_eff > min_efficacy) {
            const double mir_act = gmi_binary_activity_score(simplex, x, mir_cut);
            const double mir_bs = gmi_binary_support_ratio(simplex, mir_cut);
            if (mir_act >= min_activity && mir_bs >= min_binary_support) {
              const double mir_score = mir_eff * (1.0 + activity_weight * mir_act);
              candidates.push_back(CandidateCut{mir_cut, mir_rhs, mir_score, mir_eff, mir_act, mir_norm, mir_nnz});
            }
          }
        }
      }
    }
#endif
  }

  std::sort(candidates.begin(), candidates.end(), [](const CandidateCut& a, const CandidateCut& b) {
    if (std::abs(a.score - b.score) > 1e-12) {
      return a.score > b.score;
    }
    return a.rhs < b.rhs;
  });

  int added = 0;
  std::vector<CandidateCut> selected;
  selected.reserve(static_cast<size_t>(max_cuts));

  for (const auto& cand : candidates) {
    if (added >= max_cuts) {
      break;
    }

    bool near_parallel = false;
    for (const auto& keep : selected) {
      if (abs_cosine_similarity(cand.coeff, keep.coeff, cand.norm, keep.norm) >= opt.gmi_max_parallelism) {
        near_parallel = true;
        break;
      }
    }
    if (near_parallel) {
      continue;
    }

    selected.push_back(cand);
    ++added;
  }

  if (!selected.empty()) {
    std::vector<Eigen::VectorXd> cut_rows;
    std::vector<double> cut_rhs;
    cut_rows.reserve(selected.size());
    cut_rhs.reserve(selected.size());
    for (const auto& s : selected) {
      cut_rows.push_back(s.coeff);
      cut_rhs.push_back(s.rhs);
    }
    add_rows_to_lp(lp, cut_rows, cut_rhs);
  }

  return added;
}

int add_transformed_tableau_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
    const std::shared_ptr<BasisOps>& sbasis,
    const std::vector<char>* implied_integer_cols,
	    const BinaryImplicationGraph* implication_graph,
	    const VariableBoundTable* variable_bound_table,
	    std::vector<PoolCut>* generated_cutpool_rows,
	    double cut_generation_feastol,
	    const std::function<int(PoolCut&&)>* cutpool_acceptor,
	    std::optional<std::uint64_t> highs_cutgen_seed) {
  cut_generation_feastol = std::max(0.0, cut_generation_feastol);
  const double tableau_feastol = cut_generation_feastol;
  const double tableau_fractionality_tol = 1000.0 * tableau_feastol;
  if (max_cuts <= 0 || !simplex.exact_optimal ||
      x.size() != simplex.form.n_original) {
    return 0;
  }
  if (!basis_tableau_cuts_admissible(simplex, opt, sbasis)) {
    return 0;
  }
  XTabSourceDiag diag;
  const int m = static_cast<int>(simplex.basis.indices.size());
  const int n_std = static_cast<int>(simplex.form.A.cols());
  const int n_orig = simplex.form.n_original;
  if (m <= 0 || n_std <= 0 || simplex.form.A_row.rows() != m ||
      simplex.form.b.size() != m || simplex.x_basic.size() != m ||
      simplex.x_std.size() != n_std || simplex.form.var_ub.size() != n_std) {
    return 0;
  }
  if (!sbasis && simplex.basis_inverse.rows() != m) {
    return 0;
  }
  const XTabTransformContext transform_context = xtab_build_transform_context(
      simplex, implied_integer_cols, implication_graph, variable_bound_table);
  const XTabSourceContext source_context =
      xtab_build_source_context(simplex, implied_integer_cols);
  XTabCutgenRandom cutgen_random(highs_cutgen_seed);
  if (!source_context.valid) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }
  if (std::getenv("MIPSOLVERS_XTAB_DIAG") != nullptr) {
    fmt::print(stderr,
               "[B&C-XTAB-CTX] family=tableau srcDim={} srcRows={} vlb={} "
               "vub={} intVlb={} intVub={}\n",
               source_context.dim, source_context.n_rows,
               transform_context.num_vlb, transform_context.num_vub,
               transform_context.num_integral_vlb,
               transform_context.num_integral_vub);
  }
  struct FractionalBasisRow {
    int row{-1};
    int basic_col{-1};
    int source_col{-1};
    double frac{0.0};
    double btran_scale{1.0};
    double score{0.0};
    std::vector<std::pair<int, double>> row_ep;
  };
  auto trace_fractional_rows =
      [&](const char* stage, const std::vector<FractionalBasisRow>& rows,
          const char* extra) {
        if (!xtab_row_trace_basis_enabled()) return;
        fmt::memory_buffer buffer;
        const int max_terms = xtab_row_trace_terms();
        int emitted = 0;
        for (const auto& fr : rows) {
          if (emitted >= max_terms) break;
          if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ";");
          fmt::format_to(std::back_inserter(buffer),
                         "{}:{}:{}:{:.17g}:{:.17g}:{}",
                         fr.row, fr.basic_col, fr.source_col, fr.frac,
                         fr.score, fr.row_ep.size());
          ++emitted;
        }
        if (static_cast<int>(rows.size()) > emitted) {
          fmt::format_to(std::back_inserter(buffer), ";...");
        }
        fmt::print(stderr,
                   "[B&C-XROW] id=0 family=tableau stage={} {} "
                   "count={} sample=[{}]\n",
                   stage, extra == nullptr ? "" : extra, rows.size(),
                   fmt::to_string(buffer));
      };
  auto trace_row_ep_filter =
      [&](const FractionalBasisRow& fr, int raw_count, int kept_count,
          double min_weight, double max_weight, double norm2,
          const char* reject) {
        if (!xtab_row_trace_basis_enabled()) return;
        fmt::print(stderr,
                   "[B&C-XROW] id=0 family=tableau stage=rowep "
                   "basisRow={} basicVar={} sourceCol={} raw={} kept={} "
                   "min={:.12g} max={:.12g} ratio={:.12g} norm2={:.12g} "
                   "frac={:.12g} score={:.12g} reject={}\n",
                   fr.row, fr.basic_col, fr.source_col, raw_count, kept_count,
                   min_weight, max_weight,
                   min_weight > 0.0 ? max_weight / min_weight
                                    : std::numeric_limits<double>::infinity(),
                   norm2, fr.frac, fr.score, reject);
      };
  std::vector<FractionalBasisRow> fractional_rows;
  fractional_rows.reserve(static_cast<std::size_t>(m));
  for (int row = 0; row < m; ++row) {
    ++diag.basis_rows;
    const int basic_col = simplex.basis.indices[static_cast<std::size_t>(row)];
    int source_col = -1;
    double btran_scale = 1.0;
    if (!xtab_basic_source_info(simplex, source_context, basic_col, source_col,
                                btran_scale) ||
        source_col < 0 || source_col >= source_context.dim ||
        source_context.integral[static_cast<std::size_t>(source_col)] == 0) {
      continue;
    }
    if (source_col < n_orig) {
      ++diag.integer_basic_original;
    } else {
      ++diag.integer_basic_aux;
    }
    const double value =
        xtab_unscaled_source_value(simplex, source_context, source_col);
    const double frac = xtab_fractionality_distance(value);
    if (frac < tableau_fractionality_tol) {
      continue;
    }
    if (source_col < n_orig) {
      ++diag.fractional_basic_original;
    } else {
      ++diag.fractional_basic_aux;
    }
    fractional_rows.push_back(
        FractionalBasisRow{row, basic_col, source_col, frac, btran_scale, 0.0, {}});
  }
  trace_fractional_rows("frac_scan", fractional_rows,
                        "key=basis_order");
  if (fractional_rows.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }

  const std::int64_t num_tries_before = 0;
  std::int64_t max_tries = 5000;
  std::int64_t integral_cols = 0;
  for (int col = 0; col < n_orig; ++col) {
    if (source_context.integral[static_cast<std::size_t>(col)] != 0) {
      ++integral_cols;
    }
  }
  max_tries = std::min<std::int64_t>(
      max_tries,
      200 + static_cast<std::int64_t>(
                0.1 * static_cast<double>(std::min<std::int64_t>(
                          m, std::max<std::int64_t>(1, integral_cols)))));
  if (max_tries <= 0) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }

  const bool truncated =
      static_cast<std::int64_t>(fractional_rows.size()) > max_tries;
  if (truncated) {
    std::sort(fractional_rows.begin(), fractional_rows.end(),
              [&](const FractionalBasisRow& a,
                  const FractionalBasisRow& b) {
                return std::make_pair(
                           a.frac,
                           xtab_highs_hash_i64(num_tries_before + a.row)) >
                       std::make_pair(
                           b.frac,
                           xtab_highs_hash_i64(num_tries_before + b.row));
              });
    fractional_rows.resize(static_cast<std::size_t>(max_tries));
  }
  {
    const std::string extra =
        fmt::format("maxTries={} triesBefore={} truncated={} key={}",
                    max_tries, num_tries_before, truncated ? 1 : 0,
                    truncated ? "fractionality_hash" : "basis_order");
    trace_fractional_rows("pre_rowep_order", fractional_rows, extra.c_str());
  }

  for (auto& fr : fractional_rows) {
    Eigen::VectorXd y;
    std::vector<std::pair<int, double>> sparse_y;
    const bool have_sparse_y =
        sbasis && sbasis->kind() == BasisOpsKind::VendoredHighs &&
        sparse_basis_inverse_row_sparse_entries(sbasis, fr.row, sparse_y);
    if (sbasis) {
      if (!have_sparse_y && !sparse_basis_inverse_row(sbasis, fr.row, y)) {
        continue;
      }
    } else {
      if (simplex.basis_inverse.rows() != m ||
          simplex.basis_inverse.cols() != m) {
        continue;
      }
      y = simplex.basis_inverse.row(fr.row).transpose();
    }
    if (!have_sparse_y && (y.size() != m || !y.allFinite())) {
      continue;
    }
    ++diag.btran_rows;

    double norm2 = 0.0;
    double min_weight = std::numeric_limits<double>::infinity();
    double max_weight = 0.0;
    int raw_count = 0;
    fr.row_ep.reserve(have_sparse_y ? sparse_y.size()
                                    : static_cast<std::size_t>(m));
    auto consume_row_ep_entry = [&](int r, double raw_weight) {
      if (r < 0 || r >= m || !std::isfinite(raw_weight)) return;
      const double w = raw_weight * fr.btran_scale *
                       xtab_row_scale_or_one(simplex.form, r);
      ++raw_count;
      const double row_max = standard_row_max_abs(simplex, r);
      const double scaled_weight = row_max * std::abs(w);
      if (scaled_weight <= tableau_feastol) {
        return;
      }
      min_weight = std::min(min_weight, scaled_weight);
      max_weight = std::max(max_weight, scaled_weight);
      norm2 += scaled_weight * scaled_weight;
      fr.row_ep.emplace_back(r, w);
    };
    if (have_sparse_y) {
      for (const auto& entry : sparse_y) {
        consume_row_ep_entry(entry.first, entry.second);
      }
    } else {
      for (int r = 0; r < m; ++r) {
        const double raw_weight = y[r];
        if (std::abs(raw_weight) <= 1e-12) {
          continue;
        }
        consume_row_ep_entry(r, raw_weight);
      }
    }
    const int kept_count = static_cast<int>(fr.row_ep.size());
    if (fr.row_ep.size() <= 1 || !(norm2 > 0.0) ||
        !(min_weight > 0.0) || max_weight / min_weight > 1e4) {
      const char* reject =
          fr.row_ep.size() <= 1
              ? "count"
              : (!(norm2 > 0.0) || !(min_weight > 0.0) ? "weight"
                                                        : "ratio");
      trace_row_ep_filter(fr, raw_count, kept_count, min_weight, max_weight,
                          norm2, reject);
      fr.row_ep.clear();
      continue;
    }
    ++diag.row_ep_rows;
    fr.score = fr.frac * (1.0 - fr.frac) / norm2;
    trace_row_ep_filter(fr, raw_count, kept_count, min_weight, max_weight,
                        norm2, "none");
  }
  fractional_rows.erase(
      std::remove_if(fractional_rows.begin(), fractional_rows.end(),
                     [&](const FractionalBasisRow& row) {
                       return row.row_ep.empty() ||
                              row.score <= tableau_feastol;
                     }),
      fractional_rows.end());
  if (fractional_rows.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }
  pdqsort_branchless(
      fractional_rows.begin(), fractional_rows.end(),
      [](const FractionalBasisRow& a, const FractionalBasisRow& b) {
        return a.score > b.score;
      });
  trace_fractional_rows("post_rowep_order", fractional_rows, "key=score");

  std::vector<XTabCandidateCut> candidates;
  candidates.reserve(static_cast<std::size_t>(max_cuts * 2));

  auto tableau_basis_trace_info = [&](const FractionalBasisRow& fr,
                                      int sign) -> std::string {
    if (!xtab_row_trace_basis_enabled()) return {};
    fmt::memory_buffer buffer;
    const char* basic_kind = "unknown";
    int basic_logical_row = -1;
    if (fr.basic_col >= 0 && fr.basic_col < n_orig) {
      basic_kind = "col";
    } else {
      basic_logical_row = standard_form_aux_col_row(simplex.form, fr.basic_col);
      basic_kind = basic_logical_row >= 0 ? "row" : "aux";
    }
    fmt::format_to(std::back_inserter(buffer),
                   "basisRow={} basicVar={} basicKind={} basicLogicalRow={} "
                   "sourceCol={} sign={} frac={:.12g} score={:.12g} "
                   "btranScale={:.12g} rowEp=[",
                   fr.row, fr.basic_col, basic_kind, basic_logical_row,
                   fr.source_col, sign, fr.frac, fr.score, fr.btran_scale);

    int emitted = 0;
    const int max_terms = xtab_row_trace_terms();
    for (const auto& row_weight : fr.row_ep) {
      if (emitted >= max_terms) break;
      const int row = row_weight.first;
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ";");
      double lower = 0.0;
      double upper = 0.0;
      const bool have_bounds =
          xtab_logical_row_bounds(simplex, row, lower, upper);
      fmt::format_to(std::back_inserter(buffer),
                     "row{}:w{:.12g}:lb{:.12g}:ub{:.12g}:sig[{}]",
                     row, row_weight.second,
                     have_bounds ? lower
                                 : -std::numeric_limits<double>::infinity(),
                     have_bounds ? upper
                                 : std::numeric_limits<double>::infinity(),
                     xtab_source_row_signature(simplex, row, max_terms));
      ++emitted;
    }
    if (static_cast<int>(fr.row_ep.size()) > emitted) {
      fmt::format_to(std::back_inserter(buffer), ";...");
    }
    fmt::format_to(std::back_inserter(buffer), "]");
    return fmt::to_string(buffer);
  };

  const double density_cap =
      std::min(1.0, std::max(0.01, opt.gmi_max_density));
  const int max_nnz =
      std::max(2, static_cast<int>(std::ceil(density_cap *
                                             static_cast<double>(n_orig))));
  const double min_efficacy = std::max(0.0, opt.gmi_min_efficacy);
  const double min_activity =
      std::min(1.0, std::max(0.0, opt.gmi_min_activity));
  const double min_binary_support =
      std::min(1.0, std::max(0.0, opt.gmi_min_binary_support));
  const double activity_weight = std::max(0.0, opt.gmi_activity_weight);

  double best_score = -1.0;
  int accepted_cutpool_rows = 0;
  for (const auto& fr : fractional_rows) {
    const int accepted_or_generated =
        cutpool_acceptor != nullptr
            ? accepted_cutpool_rows
            : static_cast<int>(candidates.size());
    if (cutpool_acceptor != nullptr) {
      if (accepted_cutpool_rows >= 1000) {
        break;
      }
    } else if (accepted_or_generated >= 4 * max_cuts) {
      break;
    }
    const double best_score_factor =
        (cutpool_acceptor != nullptr && accepted_cutpool_rows >= 50)
            ? 0.01
            : 0.0025;
    if (best_score > 0.0 && fr.score < best_score_factor * best_score) {
      break;
    }

    {
      const auto& row_ep_variant = fr.row_ep;
      Eigen::VectorXd base_coeff;
      double base_rhs = 0.0;
      std::vector<int> base_active_cols;
      if (!aggregate_standard_rows(simplex, source_context, row_ep_variant,
                                   base_coeff, base_rhs,
                                   &base_active_cols)) {
        continue;
      }
      ++diag.aggregate_ok;
      const int base_nnz = count_nonzeros(base_coeff);
      if (base_nnz > static_cast<int>(row_ep_variant.size()) &&
          10 * (base_nnz - static_cast<int>(row_ep_variant.size())) >
              10000 + n_orig) {
        continue;
      }

      double max_abs = 0.0;
      double min_abs = std::numeric_limits<double>::infinity();
      for (int j = 0; j < n_orig; ++j) {
        const double a = std::abs(base_coeff[j]);
        if (a <= 1e-12) {
          continue;
        }
        max_abs = std::max(max_abs, a);
        min_abs = std::min(min_abs, a);
      }
      if (!(min_abs > 0.0) || max_abs / min_abs > 1e6) {
        continue;
      }

      for (int sign : {1, -1}) {
        Eigen::VectorXd cut;
        double rhs = 0.0;
        double efficacy = 0.0;
        const Eigen::VectorXd signed_coeff =
            (sign == 1) ? base_coeff : (-base_coeff);
        const double signed_rhs = (sign == 1) ? base_rhs : (-base_rhs);
        const std::string basis_info = tableau_basis_trace_info(fr, sign);
        XTabRejectReason reject_reason = XTabRejectReason::None;
        if (!xtab_generate_cut_from_base_row(simplex, source_context,
                                             signed_coeff, signed_rhs,
                                             &base_active_cols, x,
                                             implied_integer_cols,
                                             &transform_context,
	                                             &diag.vb_substitutions,
	                                             &diag.vb_trigger_terms, cut, rhs,
	                                             efficacy, cut_generation_feastol,
	                                             &reject_reason, &diag,
		                                             /*require_violation=*/true,
                                             "tableau",
                                             basis_info.empty()
                                                 ? nullptr
                                                 : basis_info.c_str(),
                                             &cutgen_random)) {
          xtab_record_reject(&diag, reject_reason);
          continue;
        }
	        ++diag.generated;
	        const int nnz = count_nonzeros(cut);
	        const double norm = std::max(1e-12, cut.norm());
	        const double activity = gmi_binary_activity_score(simplex, x, cut);
	        const double score = efficacy * (1.0 + activity_weight * activity);
        if (cutpool_acceptor != nullptr) {
          PoolCut row = xtab_candidate_to_pool_cut(
              XTabCandidateCut{std::move(cut), rhs, score, efficacy, norm,
                               nnz});
          const int added = (*cutpool_acceptor)(std::move(row));
          if (added > 0) {
            accepted_cutpool_rows += added;
          }
          continue;
        }
	        if (generated_cutpool_rows != nullptr) {
	          candidates.push_back(
	              XTabCandidateCut{std::move(cut), rhs, score, efficacy, norm, nnz});
          continue;
        }
        if (nnz > max_nnz) {
          ++diag.filtered_density;
          continue;
        }
	        if (efficacy < min_efficacy) {
	          ++diag.filtered_efficacy;
	          continue;
	        }
	        if (activity < min_activity) {
	          ++diag.filtered_activity;
	          continue;
	        }
        const double binary_support = gmi_binary_support_ratio(simplex, cut);
        if (binary_support < min_binary_support) {
          ++diag.filtered_binary_support;
          continue;
        }
	        candidates.push_back(
	            XTabCandidateCut{std::move(cut), rhs, score, efficacy, norm, nnz});
      }
    }
    if (best_score < 0.0 &&
        (cutpool_acceptor != nullptr ? accepted_cutpool_rows > 0
                                     : !candidates.empty())) {
      best_score = fr.score;
    }
  }

  if (cutpool_acceptor != nullptr) {
    maybe_print_xtab_diag("tableau", diag);
    return accepted_cutpool_rows;
  }

  if (candidates.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }
	  if (generated_cutpool_rows != nullptr) {
	    generated_cutpool_rows->reserve(generated_cutpool_rows->size() +
	                                    candidates.size());
	    for (const auto& cand : candidates) {
	      generated_cutpool_rows->push_back(xtab_candidate_to_pool_cut(cand));
	    }
	    maybe_print_xtab_diag("tableau", diag);
	    return static_cast<int>(candidates.size());
	  }
  std::sort(candidates.begin(), candidates.end(),
            [](const XTabCandidateCut& a, const XTabCandidateCut& b) {
              if (std::abs(a.score - b.score) > 1e-12) {
                return a.score > b.score;
              }
              return a.rhs < b.rhs;
            });

  std::vector<XTabCandidateCut> selected;
  selected.reserve(static_cast<std::size_t>(max_cuts));
  for (const auto& cand : candidates) {
    if (static_cast<int>(selected.size()) >= max_cuts) {
      break;
    }
    bool near_parallel = false;
    for (const auto& keep : selected) {
      if (abs_cosine_similarity(cand.coeff, keep.coeff, cand.norm,
                                keep.norm) >= opt.gmi_max_parallelism) {
        near_parallel = true;
        break;
      }
    }
    if (near_parallel) {
      ++diag.filtered_parallel;
      continue;
    }
    selected.push_back(cand);
    ++diag.selected;
  }
  if (selected.empty()) {
    maybe_print_xtab_diag("tableau", diag);
    return 0;
  }

  std::vector<Eigen::VectorXd> rows;
  std::vector<double> rhs;
  rows.reserve(selected.size());
  rhs.reserve(selected.size());
  for (const auto& s : selected) {
    rows.push_back(s.coeff);
    rhs.push_back(s.rhs);
  }
  add_rows_to_lp(lp, rows, rhs);
  maybe_print_xtab_diag("tableau", diag);
  return static_cast<int>(selected.size());
}

int add_transformed_path_cuts(
    LPModel& lp,
    const Eigen::VectorXd& x,
    const SimplexResult& simplex,
    const BCOptions& opt,
    int max_cuts,
    const std::vector<char>* implied_integer_cols,
	    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table,
    std::vector<PoolCut>* generated_cutpool_rows,
    double cut_generation_feastol,
    const std::function<int(PoolCut&&)>* cutpool_acceptor,
    std::optional<std::uint64_t> highs_cutgen_seed,
    const std::function<int(int)>* highs_path_randint) {
  const bool direct_cutpool_mode = cutpool_acceptor != nullptr;
  cut_generation_feastol = std::max(0.0, cut_generation_feastol);
  if ((!direct_cutpool_mode && max_cuts <= 0) || !simplex.exact_optimal ||
      x.size() != simplex.form.n_original) {
    return 0;
  }
  XTabSourceDiag diag;
  int ledger_source_dim = 0;
  struct XPathLedger {
    int row_eq{0};
    int row_leq{0};
    int row_geq{0};
    int row_unusable{0};
    int integer_like_cols{0};
    int continuous_cols{0};
    int continuous_bd_cols{0};
    int rows_with_continuous{0};
    int substitution_rows{0};
    int in_arcs{0};
    int out_arcs{0};
    int usable_start_rows{0};
    std::uint64_t start_attempts{0};
    std::uint64_t path_iterations{0};
    std::uint64_t aggregation_calls{0};
    std::uint64_t substitution_events{0};
    std::uint64_t path_extensions{0};
    std::uint64_t try_negated_scale{0};
    std::uint64_t cutgen_calls{0};
    std::uint64_t cutgen_success{0};
    std::uint64_t cutpool_calls{0};
    std::uint64_t cutpool_accepted{0};
    std::uint64_t candidate_rows{0};
    std::uint64_t mix_attempts{0};
    std::uint64_t mix_success{0};
    std::uint64_t mix_accepted{0};
  } path_ledger;
  auto print_path_ledger = [&](const char* stage,
                               int accepted_cutpool_rows,
                               std::size_t candidates_size) {
    if (std::getenv("MIPSOLVERS_XPATH_LEDGER") == nullptr &&
        std::getenv("MIPSOLVERS_XTAB_DIAG") == nullptr) {
      return;
    }
    path_ledger.candidate_rows = static_cast<std::uint64_t>(candidates_size);
    fmt::print(
        stderr,
        "[B&C-XPATH-LEDGER] stage={} mode={} srcRows={} srcDim={} "
        "rowtype=eq{}:le{}:ge{}:bad{} cols=int{}:cont{}:bd{} "
        "rowsCont={} subst={} arcs=in{}:out{} usableStart={} "
        "starts={} iters={} aggs={} substEvents={} extensions={} "
        "tryNeg={} cutgen={}/{} gen={} mix={}/{}/{} "
        "poolCalls={} poolAccepted={} acceptedRows={} candidates={}\n",
        stage, direct_cutpool_mode ? "cutpool" : "bounded",
        simplex.form.A_row.rows(), ledger_source_dim,
        path_ledger.row_eq, path_ledger.row_leq, path_ledger.row_geq,
        path_ledger.row_unusable, path_ledger.integer_like_cols,
        path_ledger.continuous_cols, path_ledger.continuous_bd_cols,
        path_ledger.rows_with_continuous, path_ledger.substitution_rows,
        path_ledger.in_arcs, path_ledger.out_arcs,
        path_ledger.usable_start_rows, path_ledger.start_attempts,
        path_ledger.path_iterations, path_ledger.aggregation_calls,
        path_ledger.substitution_events, path_ledger.path_extensions,
        path_ledger.try_negated_scale, path_ledger.cutgen_success,
        path_ledger.cutgen_calls, diag.generated, path_ledger.mix_success,
        path_ledger.mix_attempts, path_ledger.mix_accepted,
        path_ledger.cutpool_calls, path_ledger.cutpool_accepted,
        accepted_cutpool_rows, path_ledger.candidate_rows);
  };
  const int n_orig = simplex.form.n_original;
  const int m = static_cast<int>(simplex.form.A_row.rows());
  if (m <= 0 || simplex.form.b.size() != m ||
      simplex.x_std.size() != simplex.form.A.cols()) {
    return 0;
  }
  const XTabTransformContext transform_context = xtab_build_transform_context(
      simplex, implied_integer_cols, implication_graph, variable_bound_table);
  const XTabSourceContext source_context =
      xtab_build_source_context(simplex, implied_integer_cols);
  ledger_source_dim = source_context.valid ? source_context.dim : 0;
  XTabCutgenRandom cutgen_random(highs_cutgen_seed);
  if (!source_context.valid) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("invalid_context", 0, 0);
    return 0;
  }
  if (std::getenv("MIPSOLVERS_XTAB_DIAG") != nullptr) {
    fmt::print(stderr,
               "[B&C-XTAB-CTX] family=path srcDim={} srcRows={} vlb={} "
               "vub={} intVlb={} intVub={}\n",
               source_context.dim, source_context.n_rows,
               transform_context.num_vlb, transform_context.num_vub,
               transform_context.num_integral_vlb,
               transform_context.num_integral_vub);
  }
  std::vector<unsigned char> integer_like_orig(
      static_cast<std::size_t>(n_orig), 0);
  for (int col = 0; col < n_orig; ++col) {
    if (!transformed_col_is_integer_like(simplex, col,
                                         implied_integer_cols)) {
      continue;
    }
    integer_like_orig[static_cast<std::size_t>(col)] = 1;
  }
  for (int col = 0; col < n_orig; ++col) {
    if (integer_like_orig[static_cast<std::size_t>(col)] != 0) {
      ++path_ledger.integer_like_cols;
    } else {
      ++path_ledger.continuous_cols;
    }
  }
  enum class PathRowType : signed char {
    Unusable = -2,
    Geq = -1,
    Eq = 0,
    Leq = 1,
  };
  std::vector<PathRowType> row_type(static_cast<std::size_t>(m),
                                    PathRowType::Unusable);
	  const double feastol = cut_generation_feastol;
  for (int r = 0; r < m; ++r) {
    const int scol = source_context.n_original + r;
    const double lower = source_context.lower[scol];
    const double upper = source_context.upper[scol];
    const double act = source_context.solution[scol];
    if (std::isfinite(lower) && std::isfinite(upper) &&
        std::abs(upper - lower) <= feastol) {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Eq;
      continue;
    }
    double lower_slack = std::numeric_limits<double>::infinity();
    double upper_slack = std::numeric_limits<double>::infinity();
    if (std::isfinite(lower)) {
      lower_slack = act - lower;
    }
    if (std::isfinite(upper)) {
      upper_slack = upper - act;
    }
    if (lower_slack > feastol && upper_slack > feastol) {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Unusable;
    } else if (lower_slack < upper_slack) {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Geq;
    } else {
      row_type[static_cast<std::size_t>(r)] = PathRowType::Leq;
    }
  }
  for (PathRowType type : row_type) {
    switch (type) {
      case PathRowType::Eq:
        ++path_ledger.row_eq;
        break;
      case PathRowType::Leq:
        ++path_ledger.row_leq;
        break;
      case PathRowType::Geq:
        ++path_ledger.row_geq;
        break;
      case PathRowType::Unusable:
        ++path_ledger.row_unusable;
        break;
    }
  }

  auto bound_distance = [&](int col) -> double {
    if (col < 0 || col >= n_orig) {
      return 0.0;
    }
    if (integer_like_orig[static_cast<std::size_t>(col)] != 0) {
      return 0.0;
    }
    if (col >= source_context.solution.size() ||
        col >= source_context.lower.size() ||
        col >= source_context.upper.size()) {
      return 0.0;
    }
    const double val = source_context.solution[col];
    const double lb = source_context.lower[col];
    const double ub = source_context.upper[col];
    if (!std::isfinite(val)) {
      return 0.0;
    }
    double lb_dist = std::isfinite(lb) ? std::max(0.0, val - lb)
                                       : std::numeric_limits<double>::infinity();
    double ub_dist = std::numeric_limits<double>::infinity();
    if (std::isfinite(ub)) {
      ub_dist = std::max(0.0, ub - val);
    }
    if (transform_context.best_vlb.size() > static_cast<std::size_t>(col)) {
      const XTabVarBoundExpr& vlb =
          transform_context.best_vlb[static_cast<std::size_t>(col)];
      if (vlb.valid && vlb.trigger_col >= 0 &&
          vlb.trigger_col < source_context.solution.size()) {
        const double trig = source_context.solution[vlb.trigger_col];
        const double expr = vlb.constant + vlb.coef * trig;
        if (std::isfinite(expr)) {
          lb_dist = std::min(lb_dist, std::max(0.0, val - expr));
        }
      }
    }
    if (transform_context.best_vub.size() > static_cast<std::size_t>(col)) {
      const XTabVarBoundExpr& vub =
          transform_context.best_vub[static_cast<std::size_t>(col)];
      if (vub.valid && vub.trigger_col >= 0 &&
          vub.trigger_col < source_context.solution.size()) {
        const double trig = source_context.solution[vub.trigger_col];
        const double expr = vub.constant + vub.coef * trig;
        if (std::isfinite(expr)) {
          ub_dist = std::min(ub_dist, std::max(0.0, expr - val));
        }
      }
    }
    const double dist = std::min(lb_dist, ub_dist);
    return std::isfinite(dist) && dist > feastol ? dist : 0.0;
  };
  std::vector<double> col_bound_distance(static_cast<std::size_t>(n_orig), 0.0);
  for (int col = 0; col < n_orig; ++col) {
    col_bound_distance[static_cast<std::size_t>(col)] = bound_distance(col);
    if (integer_like_orig[static_cast<std::size_t>(col)] == 0 &&
        col_bound_distance[static_cast<std::size_t>(col)] != 0.0) {
      ++path_ledger.continuous_bd_cols;
    }
  }

  struct PathArc {
    int row{-1};
    double coeff{0.0};
  };

  std::vector<int> num_continuous(static_cast<std::size_t>(m), 0);
  for (int col = 0; col < n_orig; ++col) {
    if (integer_like_orig[static_cast<std::size_t>(col)] != 0 ||
        col_bound_distance[static_cast<std::size_t>(col)] == 0.0) {
      continue;
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(simplex.form.A, col);
         it; ++it) {
      const int row = static_cast<int>(it.row());
      if (row >= 0 && row < m) {
        ++num_continuous[static_cast<std::size_t>(row)];
      }
    }
  }
  for (int r = 0; r < m; ++r) {
    if (num_continuous[static_cast<std::size_t>(r)] > 0) {
      ++path_ledger.rows_with_continuous;
    }
  }

  std::vector<PathArc> col_substitution(static_cast<std::size_t>(n_orig),
                                        PathArc{-1, 0.0});
  for (int r = 0; r < m; ++r) {
    if (row_type[static_cast<std::size_t>(r)] != PathRowType::Eq ||
        num_continuous[static_cast<std::size_t>(r)] != 1) {
      continue;
    }
    int subst_col = -1;
    double subst_val = 0.0;
	    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
	             simplex.form.A_row, r);
	         it; ++it) {
	      const int col = static_cast<int>(it.col());
      if (col < 0 || col >= n_orig ||
          integer_like_orig[static_cast<std::size_t>(col)] != 0 ||
          col_bound_distance[static_cast<std::size_t>(col)] == 0.0) {
        continue;
      }
	      subst_col = col;
	      subst_val = it.value() / (xtab_row_scale_or_one(simplex.form, r) *
	                                 xtab_col_scale_or_one(simplex.form, col));
	      break;
    }
    if (subst_col >= 0 &&
        col_substitution[static_cast<std::size_t>(subst_col)].row < 0) {
      col_substitution[static_cast<std::size_t>(subst_col)] =
          PathArc{r, subst_val};
      row_type[static_cast<std::size_t>(r)] = PathRowType::Unusable;
      ++path_ledger.substitution_rows;
    }
  }

  std::vector<double> col_source_drive(static_cast<std::size_t>(n_orig), 0.0);
  std::vector<std::vector<PathArc>> col_in_arcs(
      static_cast<std::size_t>(n_orig));
  std::vector<std::vector<PathArc>> col_out_arcs(
      static_cast<std::size_t>(n_orig));
  for (int col = 0; col < n_orig; ++col) {
    if (col_substitution[static_cast<std::size_t>(col)].row >= 0 ||
        integer_like_orig[static_cast<std::size_t>(col)] != 0) {
      continue;
    }
    const double bd = col_bound_distance[static_cast<std::size_t>(col)];
	    if (bd <= feastol) {
	      continue;
	    }
	    col_source_drive[static_cast<std::size_t>(col)] = bd;
  }

  for (int r = 0; r < m; ++r) {
    const PathRowType type = row_type[static_cast<std::size_t>(r)];
    if (type == PathRowType::Unusable) {
      continue;
    }
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(
             simplex.form.A_row, r);
         it; ++it) {
      const int col = static_cast<int>(it.col());
      if (col < 0 || col >= n_orig) {
        continue;
      }
      if (col_source_drive[static_cast<std::size_t>(col)] <= feastol) {
        continue;
      }
      const double a =
          it.value() / (xtab_row_scale_or_one(simplex.form, r) *
                        xtab_col_scale_or_one(simplex.form, col));
      if (std::abs(a) <= 1e-12) {
        continue;
      }
      auto& in_arcs = col_in_arcs[static_cast<std::size_t>(col)];
      auto& out_arcs = col_out_arcs[static_cast<std::size_t>(col)];
      switch (type) {
        case PathRowType::Leq:
          if (a < 0.0) {
            in_arcs.push_back(PathArc{r, a});
          } else {
            out_arcs.push_back(PathArc{r, a});
          }
          break;
        case PathRowType::Geq:
          if (a > 0.0) {
            in_arcs.push_back(PathArc{r, a});
          } else {
            out_arcs.push_back(PathArc{r, a});
          }
          break;
        case PathRowType::Eq:
          in_arcs.push_back(PathArc{r, a});
          out_arcs.push_back(PathArc{r, a});
          break;
        case PathRowType::Unusable:
          break;
      }
    }
  }
  for (const auto& arcs : col_in_arcs) {
    path_ledger.in_arcs += static_cast<int>(arcs.size());
  }
  for (const auto& arcs : col_out_arcs) {
    path_ledger.out_arcs += static_cast<int>(arcs.size());
  }

  int usable_start_rows = 0;
  for (int r = 0; r < m; ++r) {
    if (row_type[static_cast<std::size_t>(r)] != PathRowType::Unusable) {
      ++usable_start_rows;
    }
  }
  path_ledger.usable_start_rows = usable_start_rows;
  if (usable_start_rows == 0) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("no_start_rows", 0, 0);
    return 0;
  }
  if (std::getenv("MIPSOLVERS_XTAB_DIAG") != nullptr) {
    if (direct_cutpool_mode) {
      fmt::print(stderr,
                 "[B&C-XTAB-PATH-SCHED] mode=highs_row_order rows={} "
                 "contentCap=none\n",
                 usable_start_rows);
    } else {
      fmt::print(stderr,
                 "[B&C-XTAB-PATH-SCHED] mode=highs_row_order rows={} "
                 "maxCuts={}\n",
                 usable_start_rows, max_cuts);
    }
  }

  std::vector<XTabCandidateCut> candidates;
  const int reserve_hint =
      direct_cutpool_mode ? 8 : std::max(8, max_cuts * 2);
  candidates.reserve(static_cast<std::size_t>(reserve_hint));
  int accepted_cutpool_rows = 0;

  const double density_cap =
      std::min(1.0, std::max(0.01, opt.gmi_max_density));
  const int max_nnz =
      std::max(2, static_cast<int>(std::ceil(density_cap *
                                             static_cast<double>(n_orig))));
  const double min_efficacy = std::max(0.0, opt.gmi_min_efficacy);

  auto row_in_path = [](const std::vector<std::pair<int, double>>& path,
                        int row) {
    return std::any_of(path.begin(), path.end(),
                       [&](const auto& p) { return p.first == row; });
  };

  auto add_path_cuts_from_aggregation = [&](const Eigen::VectorXd& coeff,
                                            double rhs,
                                            const std::vector<int>& active_cols) {
    ++diag.aggregate_ok;
    bool accepted_any = false;
    for (int sign : {1, -1}) {
      ++path_ledger.cutgen_calls;
      Eigen::VectorXd cut;
      double cut_rhs = 0.0;
      double efficacy = 0.0;
      const Eigen::VectorXd signed_coeff = sign == 1 ? coeff : (-coeff);
          const double signed_rhs = sign == 1 ? rhs : (-rhs);
      XTabRejectReason reject_reason = XTabRejectReason::None;
        std::uint64_t source_trace_id = 0;
      if (!xtab_generate_cut_from_base_row(simplex, source_context,
                                           signed_coeff, signed_rhs,
                                           &active_cols, x,
                                           implied_integer_cols,
                                           &transform_context,
	                                           &diag.vb_substitutions,
	                                           &diag.vb_trigger_terms, cut, cut_rhs,
	                                           efficacy,
	                                           cut_generation_feastol,
		                                           &reject_reason, &diag,
		                                           /*require_violation=*/true,
                                           "path", nullptr, &cutgen_random,
                                           /*only_initial_cmir_scale=*/false,
                                           &source_trace_id)) {
        xtab_record_reject(&diag, reject_reason);
        continue;
      }
      const int nnz = count_nonzeros(cut);
      const double norm = std::max(1e-12, cut.norm());
      const double score = efficacy;
      if (cutpool_acceptor != nullptr) {
        PoolCut row = xtab_candidate_to_pool_cut(
            XTabCandidateCut{std::move(cut), cut_rhs, score, efficacy, norm,
                   nnz, source_trace_id});
        const int added = (*cutpool_acceptor)(std::move(row));
        ++path_ledger.cutpool_calls;
        if (added > 0) {
          accepted_cutpool_rows += added;
          path_ledger.cutpool_accepted += static_cast<std::uint64_t>(added);
          ++path_ledger.cutgen_success;
          ++diag.generated;
          accepted_any = true;
        }
        continue;
      }
      if (generated_cutpool_rows != nullptr) {
        candidates.push_back(XTabCandidateCut{std::move(cut), cut_rhs, score,
                                              efficacy, norm, nnz,
                                              source_trace_id});
        ++path_ledger.cutgen_success;
        ++diag.generated;
        accepted_any = true;
        continue;
      }
      if (nnz > max_nnz || efficacy < min_efficacy) {
        if (nnz > max_nnz) {
          ++diag.filtered_density;
        } else {
          ++diag.filtered_efficacy;
        }
        continue;
      }
      candidates.push_back(XTabCandidateCut{std::move(cut), cut_rhs, score,
                                            efficacy, norm, nnz,
                                            source_trace_id});
      ++path_ledger.cutgen_success;
      ++diag.generated;
      accepted_any = true;
    }
    return accepted_any;
  };

  auto try_path_mixing =
      [&](const std::vector<XTabAggregatedSourceRow>& path_aggs) {
    if (path_aggs.size() < 2) {
      return;
    }
    ++path_ledger.mix_attempts;
    Eigen::VectorXd cut;
    double cut_rhs = 0.0;
    double efficacy = 0.0;
    XTabRejectReason reject_reason = XTabRejectReason::None;
    if (!xtab_generate_path_mixing_cut(simplex, source_context, path_aggs, x,
                                       implied_integer_cols, &transform_context,
                                       &diag.vb_substitutions,
                                       &diag.vb_trigger_terms, cut, cut_rhs,
	                                       efficacy, cut_generation_feastol,
	                                       &reject_reason)) {
      xtab_record_reject(&diag, reject_reason);
      return;
    }
    const int nnz = count_nonzeros(cut);
    const double norm = std::max(1e-12, cut.norm());
    const double score = efficacy;
    if (cutpool_acceptor != nullptr) {
      PoolCut row = xtab_candidate_to_pool_cut(
          XTabCandidateCut{std::move(cut), cut_rhs, score, efficacy, norm,
                           nnz});
      const int added = (*cutpool_acceptor)(std::move(row));
      ++path_ledger.cutpool_calls;
      if (added > 0) {
        accepted_cutpool_rows += added;
        path_ledger.cutpool_accepted += static_cast<std::uint64_t>(added);
        path_ledger.mix_accepted += static_cast<std::uint64_t>(added);
        ++path_ledger.mix_success;
        ++diag.generated;
      }
      return;
    }
    if (generated_cutpool_rows != nullptr) {
      candidates.push_back(
          XTabCandidateCut{std::move(cut), cut_rhs, score, efficacy, norm, nnz});
      ++path_ledger.mix_success;
      ++diag.generated;
      return;
    }
    if (nnz > max_nnz || efficacy < min_efficacy) {
      if (nnz > max_nnz) {
        ++diag.filtered_density;
      } else {
        ++diag.filtered_efficacy;
      }
      return;
    }
    candidates.push_back(
        XTabCandidateCut{std::move(cut), cut_rhs, score, efficacy, norm, nnz});
    ++path_ledger.mix_success;
    ++diag.generated;
  };

  constexpr int kMaxPathLen = 6;
  const double max_weight = 1.0 / feastol;
  const double min_weight = feastol;
  auto weight_ok = [&](double w) {
    w = std::abs(w);
    return std::isfinite(w) && w >= min_weight && w <= max_weight;
  };
  HighsRandom fallback_path_randgen(0);

  auto skip_col = [&](int col,
                      const std::vector<std::vector<PathArc>>& same_arcs,
                      const std::vector<std::vector<PathArc>>& other_arcs,
                      const std::vector<std::pair<int, double>>& path,
                      int start_row,
                      bool& try_negated_scale) {
    const auto& same = same_arcs[static_cast<std::size_t>(col)];
    const auto& other = other_arcs[static_cast<std::size_t>(col)];
    if (path.size() == 1 && !try_negated_scale) {
      if (same.size() <= path.size()) {
        for (const PathArc& arc : same) {
          if (arc.row != start_row) {
            try_negated_scale = true;
            break;
          }
        }
      } else {
        try_negated_scale = true;
      }
    }
    if (other.empty()) {
      return true;
    }
    if (other.size() <= path.size()) {
      for (const PathArc& arc : other) {
        if (!row_in_path(path, arc.row)) return false;
      }
      return true;
    }
    return false;
  };

  auto find_row = [&](int col,
                      double val,
                      const std::vector<std::vector<PathArc>>& arcs_by_col,
                      const std::vector<std::pair<int, double>>& path,
                      int& row,
                      double& weight) {
    const auto& arcs = arcs_by_col[static_cast<std::size_t>(col)];
    if (arcs.empty()) {
      return false;
    }
    int start_pos = 0;
    if (cutpool_acceptor != nullptr) {
      if (highs_path_randint != nullptr) {
        start_pos = (*highs_path_randint)(static_cast<int>(arcs.size()));
      } else {
        start_pos = static_cast<int>(
            fallback_path_randgen.integer(static_cast<HighsInt>(arcs.size())));
      }
      if (start_pos < 0 || start_pos >= static_cast<int>(arcs.size())) {
        start_pos = 0;
      }
    }
    for (int step = 0; step < static_cast<int>(arcs.size()); ++step) {
      const PathArc& arc =
          arcs[static_cast<std::size_t>((start_pos + step) %
                                        static_cast<int>(arcs.size()))];
      if (row_in_path(path, arc.row) || std::abs(arc.coeff) <= 1e-12) {
        continue;
      }
      const double w = -val / arc.coeff;
      if (!weight_ok(w)) {
        continue;
      }
      row = arc.row;
      weight = w;
      return true;
    }
    return false;
  };

  XTabLpAggregator path_aggregator(simplex, source_context);
  for (int start_row = 0; start_row < m; ++start_row) {
      if (row_type[static_cast<std::size_t>(start_row)] ==
          PathRowType::Unusable) {
        continue;
      }
      std::vector<double> start_scales;
      const PathRowType type = row_type[static_cast<std::size_t>(start_row)];
      if (type == PathRowType::Leq) {
        start_scales = {1.0, -1.0};
      } else if (type == PathRowType::Geq) {
        start_scales = {-1.0, 1.0};
      } else {
        double row_dual = 0.0;
        if (simplex.result.constraint_duals.size() == m) {
          row_dual = simplex.result.constraint_duals[start_row];
        }
        start_scales = row_dual <= 1e-12 ? std::vector<double>{1.0, -1.0}
                                         : std::vector<double>{-1.0, 1.0};
      }
      for (double start_scale : start_scales) {
        ++path_ledger.start_attempts;
        path_aggregator.clear();
        std::vector<std::pair<int, double>> path;
        std::vector<XTabAggregatedSourceRow> path_aggs;
        bool try_negated_scale = false;
        if (!path_aggregator.add_row(start_row, start_scale)) {
          break;
        }
        path.emplace_back(start_row, start_scale);
	        while (static_cast<int>(path.size()) < kMaxPathLen) {
          ++path_ledger.path_iterations;
          Eigen::VectorXd coeff;
          double rhs = 0.0;
          std::vector<int> active_cols;
          if (!path_aggregator.get_current_aggregation(
                  coeff, rhs, false, &active_cols)) {
            break;
          }
          ++path_ledger.aggregation_calls;

          int best_out_col = -1;
          double best_out_val = 0.0;
          double best_out_dist = 0.0;
          int best_in_col = -1;
          double best_in_val = 0.0;
          double best_in_dist = 0.0;
          bool added_substitution_rows = false;
          bool abort_path = false;

          for (int col : active_cols) {
            if (col < 0 || col >= n_orig) {
              continue;
            }
            const double val = coeff[col];
            if (std::abs(val) <= 1e-10 ||
                col_bound_distance[static_cast<std::size_t>(col)] <= feastol ||
                integer_like_orig[static_cast<std::size_t>(col)] != 0) {
              continue;
            }
            const PathArc subst =
                col_substitution[static_cast<std::size_t>(col)];
	            if (subst.row >= 0 && std::abs(subst.coeff) > 1e-12) {
	              const double w = -val / subst.coeff;
	              if (std::isfinite(w)) {
	                if (!path_aggregator.add_row(subst.row, w)) {
	                  abort_path = true;
	                  break;
	                }
	                ++path_ledger.substitution_events;
	                added_substitution_rows = true;
	                continue;
	              }
            }
            if (added_substitution_rows) {
              continue;
            }

            if (val < 0.0) {
              if (skip_col(col, col_out_arcs, col_in_arcs, path, start_row,
                           try_negated_scale)) {
                continue;
              }
              const double dist =
                  col_bound_distance[static_cast<std::size_t>(col)];
              if (best_out_col < 0 || dist > best_out_dist) {
                best_out_col = col;
                best_out_val = val;
                best_out_dist = dist;
              }
            } else {
              if (skip_col(col, col_in_arcs, col_out_arcs, path, start_row,
                           try_negated_scale)) {
                continue;
              }
              const double dist =
                  col_bound_distance[static_cast<std::size_t>(col)];
              if (best_in_col < 0 || dist > best_in_dist) {
                best_in_col = col;
                best_in_val = val;
                best_in_dist = dist;
              }
            }
          }
          if (abort_path) {
            break;
          }
          if (added_substitution_rows) {
            continue;
          }

          const std::size_t before_cuts = candidates.size();
          const int before_accepted = accepted_cutpool_rows;
          const bool success =
              add_path_cuts_from_aggregation(coeff, rhs, active_cols);
          if (!path_aggs.empty() || best_out_col >= 0 || best_in_col >= 0) {
            XTabAggregatedSourceRow agg;
            agg.coeff = -coeff;
            agg.rhs = -rhs;
            agg.active_cols = active_cols;
            path_aggs.push_back(std::move(agg));
          }
          const bool cut_accepted_or_generated =
              cutpool_acceptor != nullptr
                  ? accepted_cutpool_rows > before_accepted
                  : (success && candidates.size() > before_cuts);
          if (cut_accepted_or_generated ||
              (best_out_col < 0 && best_in_col < 0)) {
            break;
          }

          int next_row = -1;
          double next_weight = 0.0;
          if (best_in_col < 0 ||
              (best_out_col >= 0 &&
               best_out_dist >= best_in_dist - feastol)) {
            if (!find_row(best_out_col, best_out_val, col_in_arcs, path,
                          next_row, next_weight)) {
              if (best_in_col < 0 ||
                  !find_row(best_in_col, best_in_val, col_out_arcs, path,
                            next_row, next_weight)) {
                break;
              }
            }
          } else {
            if (!find_row(best_in_col, best_in_val, col_out_arcs, path,
                          next_row, next_weight)) {
              break;
            }
          }
          if (!path_aggregator.add_row(next_row, next_weight)) {
            break;
          }
          ++path_ledger.path_extensions;
          path.emplace_back(next_row, next_weight);
        }
        try_path_mixing(path_aggs);
        if (try_negated_scale) ++path_ledger.try_negated_scale;
        if (!try_negated_scale) {
          break;
        }
      }
      path_aggregator.clear();
  }

  if (cutpool_acceptor != nullptr) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("done", accepted_cutpool_rows, candidates.size());
    return accepted_cutpool_rows;
  }

  if (candidates.empty()) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("empty_candidates", accepted_cutpool_rows,
                      candidates.size());
    return 0;
  }
	  if (generated_cutpool_rows != nullptr) {
	    generated_cutpool_rows->reserve(generated_cutpool_rows->size() +
	                                    candidates.size());
	    for (const auto& cand : candidates) {
	      generated_cutpool_rows->push_back(xtab_candidate_to_pool_cut(cand));
	    }
	    maybe_print_xtab_diag("path", diag);
	    print_path_ledger("generated_rows", accepted_cutpool_rows,
	                      candidates.size());
	    return static_cast<int>(candidates.size());
	  }
  std::sort(candidates.begin(), candidates.end(),
            [](const XTabCandidateCut& a, const XTabCandidateCut& b) {
              return a.score > b.score;
            });
  std::vector<XTabCandidateCut> selected;
  selected.reserve(static_cast<std::size_t>(max_cuts));
  for (const auto& cand : candidates) {
    if (static_cast<int>(selected.size()) >= max_cuts) {
      break;
    }
    bool near_parallel = false;
    for (const auto& keep : selected) {
      if (abs_cosine_similarity(cand.coeff, keep.coeff, cand.norm,
                                keep.norm) >= opt.gmi_max_parallelism) {
        near_parallel = true;
        break;
      }
    }
    if (!near_parallel) {
      selected.push_back(cand);
      ++diag.selected;
    } else {
      ++diag.filtered_parallel;
    }
  }
  if (selected.empty()) {
    maybe_print_xtab_diag("path", diag);
    print_path_ledger("empty_selected", accepted_cutpool_rows,
                      candidates.size());
    return 0;
  }
  std::vector<Eigen::VectorXd> rows;
  std::vector<double> rhs;
  rows.reserve(selected.size());
  rhs.reserve(selected.size());
  for (const auto& s : selected) {
    rows.push_back(s.coeff);
    rhs.push_back(s.rhs);
  }
  add_rows_to_lp(lp, rows, rhs);
  maybe_print_xtab_diag("path", diag);
  print_path_ledger("selected", accepted_cutpool_rows, candidates.size());
  return static_cast<int>(selected.size());
}

// ============================================================================
// Clique cuts from binary conflict graph
// ============================================================================
// For constraints sum_j a_j x_j <= b where all a_j > 0 and vars are binary,
// detect pairs (x_i, x_j) where a_i + a_j > b (conflict: both can't be 1).
// Extend conflicts greedily into maximal cliques. Cut: sum_{i in clique} x_i <= 1.

int add_clique_cuts(LPModel& lp,
                    const Eigen::VectorXd& x,
                    int max_cuts) {
  if (max_cuts <= 0) return 0;

  const int n = static_cast<int>(lp.vars.size());
  const int m = static_cast<int>(lp.A.rows());

  // Skip for very large problems: conflict graph construction is O(m * k^2)
  // and rebuilds each cut round. Raised from 8000→15000 to enable clique
  // cuts on SCUC-scale problems (m ≈ 9000-12000).
  if (m > 15000) return 0;

  // Build conflict adjacency: conflict[i] = set of j where some row forces
  // x_i + x_j <= 1 (i.e. a_i + a_j > b).
  // Limit to rows with at most 200 binary vars to avoid O(n^2) blowup.
  // Build conflict adjacency: conflict[i] = set of j where some row forces
  // x_i + x_j <= 1 (i.e. a_i + a_j > b).
  // Limit to rows with at most 200 binary vars to avoid O(n^2) blowup.
  std::vector<std::vector<int>> conflicts(n);

  // Build row-major view once to avoid O(nnz) per column access in the
  // conflict-graph construction loop.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row_clique = lp.A;

  for (int r = 0; r < m; ++r) {
    if (lp.b[r] <= 0.0) continue;

    std::vector<std::pair<int,double>> bin_vars;
    bool valid = true;
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row_clique, r); it; ++it) {
      const double a = it.value();
      if (std::abs(a) <= 1e-12) continue;
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= n) { valid = false; break; }
      if (a < 0.0 || lp.vars[j].type != VarType::Binary) { valid = false; break; }
      bin_vars.push_back({j, a});
    }
    if (!valid || bin_vars.size() < 2 || bin_vars.size() > 200) continue;

    const double b_r = lp.b[r];
    for (size_t i = 0; i < bin_vars.size(); ++i) {
      for (size_t k = i + 1; k < bin_vars.size(); ++k) {
        if (bin_vars[i].second + bin_vars[k].second > b_r + 1e-9) {
          conflicts[bin_vars[i].first].push_back(bin_vars[k].first);
          conflicts[bin_vars[k].first].push_back(bin_vars[i].first);
        }
      }
    }
  }

  // Sort and deduplicate conflict lists
  for (int j = 0; j < n; ++j) {
    auto& v = conflicts[j];
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  }

  // Find cliques violated by current LP solution, ordered by fractional sum
  struct CliqueCandidate {
    std::vector<int> clique;
    double violation;
  };
  std::vector<CliqueCandidate> candidates;

  // Seed cliques from high-fractional binary variables
  std::vector<int> seeds;
  for (int j = 0; j < n; ++j) {
    if (lp.vars[j].type != VarType::Binary) continue;
    if (conflicts[j].empty()) continue;
    if (x[j] > 0.1 && x[j] < 0.9) seeds.push_back(j);
  }
  // Sort seeds by fractional value descending (most violated first)
  std::sort(seeds.begin(), seeds.end(), [&](int a, int b) {
    return x[a] > x[b];
  });
  if (seeds.size() > 200) seeds.resize(200);

  std::vector<bool> used_as_seed(n, false);
  for (int seed : seeds) {
    if (used_as_seed[seed]) continue;
    used_as_seed[seed] = true;

    // Greedy clique extension: start with seed, add the neighbor with
    // highest x-value that is in conflict with ALL current clique members.
    std::vector<int> clique = {seed};
    std::vector<bool> in_clique(n, false);
    in_clique[seed] = true;

    // Candidate pool: neighbors of seed
    std::vector<int> pool = conflicts[seed];

    while (!pool.empty()) {
      // Pick candidate with highest LP value
      int best = -1;
      double best_val = -1.0;
      for (int c : pool) {
        if (!in_clique[c] && x[c] > best_val) {
          best_val = x[c];
          best = c;
        }
      }
      if (best < 0 || best_val < 1e-6) break;

      // Check: best must conflict with ALL clique members
      bool all_conflict = true;
      for (int cm : clique) {
        if (!std::binary_search(conflicts[best].begin(), conflicts[best].end(), cm)) {
          all_conflict = false;
          break;
        }
      }
      if (!all_conflict) {
        // O(1) removal: swap rejected candidate to the end and shrink.
        const auto it = std::find(pool.begin(), pool.end(), best);
        if (it != pool.end()) {
          *it = pool.back();
          pool.pop_back();
        }
        continue;
      }

      clique.push_back(best);
      in_clique[best] = true;

      // Intersect pool with neighbors of best
      std::vector<int> new_pool;
      for (int c : pool) {
        if (!in_clique[c] && std::binary_search(conflicts[best].begin(), conflicts[best].end(), c)) {
          new_pool.push_back(c);
        }
      }
      pool = std::move(new_pool);
    }

    if (clique.size() < 2) continue;

    // Check violation: sum(x_i, i in clique) > 1
    double lhs_val = 0.0;
    for (int c : clique) lhs_val += x[c];
    double viol = lhs_val - 1.0;
    if (viol > 1e-7) {
      candidates.push_back({std::move(clique), viol});
    }
  }

  // Sort by violation descending
  std::sort(candidates.begin(), candidates.end(), [](const CliqueCandidate& a, const CliqueCandidate& b) {
    return a.violation > b.violation;
  });

  int added = 0;
  std::vector<Eigen::VectorXd> cut_rows;
  std::vector<double> cut_rhs;

  for (const auto& cand : candidates) {
    if (added >= max_cuts) break;
    Eigen::VectorXd cut = Eigen::VectorXd::Zero(n);
    for (int c : cand.clique) cut[c] = 1.0;
    cut_rows.push_back(std::move(cut));
    cut_rhs.push_back(1.0);
    ++added;
  }

  if (!cut_rows.empty()) {
    add_rows_to_lp(lp, cut_rows, cut_rhs);
  }
  return added;
}

// ════════════════════════════════════════════════════════════════════════════
// Zero-half cuts — Caprara-Fischetti-Letchford style {0,1/2}-cuts
// ════════════════════════════════════════════════════════════════════════════
//
// For rows Ax ≤ b with integer variables, combine pairs of rows with
// multiplier 1/2. If the aggregated integer-variable coefficients are all
// integer (the "zero-half" property), floor the RHS to get a valid cut.
//
// We support mixed-integer rows by: (a) ensuring the continuous-variable
// coefficients sum to integers after aggregation, OR (b) bounding the
// continuous contribution via variable bounds and absorbing it into the RHS.
int add_zero_half_cuts(LPModel& lp, const Eigen::VectorXd& x, int max_cuts) {
  if (max_cuts <= 0) return 0;

  const int m = static_cast<int>(lp.A.rows());
  const int n = static_cast<int>(lp.vars.size());
  if (m < 2) return 0;

  // Build row-major view for efficient row traversal.
  const Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;

  // Step 1: For each row, build a "mod-2 signature" of integer variable
  // coefficients. Two rows can be combined for a zero-half cut if their
  // odd-integer-coefficient positions match (XOR to zero in GF(2)).
  // Also track whether continuous variables have integer coefficients.
  struct RowInfo {
    int row;
    double slack;              // b_r - a_r^T x
    int b_rounded;             // round(b_r) — needed for parity check
    std::vector<int> odd_cols; // sorted positions of odd-integer-coeff integer vars
    uint64_t fp;               // hash fingerprint of odd_cols
    bool cont_all_int;         // all continuous-var coefficients are near-integer
  };

  std::vector<RowInfo> rows;
  rows.reserve(static_cast<size_t>(m));

  for (int r = 0; r < m; ++r) {
    double activity = 0.0;
    std::vector<int> odd;
    bool has_int_var = false;
    bool cont_ok = true;

    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j >= n) continue;
      const double a = it.value();
      activity += a * x[j];

      if (lp.vars[j].type == VarType::Integer || lp.vars[j].type == VarType::Binary) {
        has_int_var = true;
        const int a_int = static_cast<int>(std::round(a));
        if (std::abs(a - a_int) < 1e-6 && (a_int & 1)) {
          odd.push_back(j);
        }
      } else {
        // Continuous variable: check if coefficient is near-integer
        if (std::abs(a - std::round(a)) > 1e-6) cont_ok = false;
      }
    }

    if (!has_int_var || odd.empty()) continue;

    // Guard against non-finite or very large RHS to prevent int overflow below.
    if (!std::isfinite(lp.b[r]) || std::abs(lp.b[r]) >= 2e9) continue;

    const double slack = lp.b[r] - activity;
    if (slack < -1e-6 || slack > 0.99) continue; // too violated or too loose

    std::sort(odd.begin(), odd.end());

    // FNV-1a hash fingerprint of odd column positions
    uint64_t h = 14695981039346656037ULL;
    for (int c : odd) {
      h ^= static_cast<uint64_t>(c);
      h *= 1099511628211ULL;
    }

    rows.push_back({r, slack,
                     static_cast<int>(std::round(lp.b[r])),
                     std::move(odd), h, cont_ok});
  }

  if (rows.size() < 2) return 0;

  // Step 2: Group rows by fingerprint for O(m) pair matching.
  std::unordered_map<uint64_t, std::vector<int>> groups;
  groups.reserve(rows.size());
  for (int i = 0; i < static_cast<int>(rows.size()); ++i) {
    groups[rows[i].fp].push_back(i);
  }

  // Step 3: For each group, try pairwise combinations.
  // The cut from combining rows r1, r2 (each with multiplier 1/2):
  //   (a_r1 + a_r2)/2 · x ≤ floor((b_r1 + b_r2)/2)
  // Violation = (b_r1+b_r2)/2 - floor(...)  - (slack_r1+slack_r2)/2
  //           = frac((b_r1+b_r2)/2) - (slack_r1+slack_r2)/2
  // For this to be positive, need (b_r1+b_r2) odd and small slacks.
  struct CutCand {
    int i1, i2;
    double violation;
  };
  std::vector<CutCand> cands;

  for (auto& [fp, idxs] : groups) {
    if (idxs.size() < 2) continue;

    // Sort by slack (tightest first) for best violations.
    std::sort(idxs.begin(), idxs.end(), [&](int a, int b) {
      return rows[a].slack < rows[b].slack;
    });

    // Limit pairs per group to avoid O(k^2) blowup in large groups.
    const int k = std::min(static_cast<int>(idxs.size()), 30);
    for (int i = 0; i < k; ++i) {
      for (int j = i + 1; j < k; ++j) {
        auto& r1 = rows[idxs[i]];
        auto& r2 = rows[idxs[j]];

        // Verify real match (not just hash collision).
        if (r1.odd_cols != r2.odd_cols) continue;

        // RHS parity: sum must be odd for fractional aggregated RHS.
        if (((r1.b_rounded + r2.b_rounded) & 1) == 0) continue;

        // Both rows must have integer continuous-var coefficients for safe rounding.
        if (!r1.cont_all_int || !r2.cont_all_int) continue;

        // Violation estimate.
        double viol = 0.5 - 0.5 * (r1.slack + r2.slack);
        if (viol < 1e-4) continue;

        cands.push_back({idxs[i], idxs[j], viol});
      }
    }
  }

  if (cands.empty()) return 0;

  // Sort by violation (best first).
  std::sort(cands.begin(), cands.end(),
            [](const CutCand& a, const CutCand& b) { return a.violation > b.violation; });

  // Step 4: Build and add the cuts.
  int added = 0;
  std::vector<Eigen::VectorXd> cut_rows_out;
  std::vector<double> cut_rhs_out;
  // Track norms for parallelism filtering.
  std::vector<double> cut_norms;

  for (const auto& cc : cands) {
    if (added >= max_cuts) break;

    auto& ri = rows[cc.i1];
    auto& rj = rows[cc.i2];

    // Build aggregated cut: (a_r1 + a_r2)/2
    Eigen::VectorXd coeff = Eigen::VectorXd::Zero(n);
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, ri.row); it; ++it) {
      if (it.col() < n) coeff[it.col()] += 0.5 * it.value();
    }
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, rj.row); it; ++it) {
      if (it.col() < n) coeff[it.col()] += 0.5 * it.value();
    }

    // Floor RHS.
    double rhs = std::floor(0.5 * (lp.b[ri.row] + lp.b[rj.row]) + 1e-9);

    // Round integer variable coefficients to nearest integer (should already
    // be integer by the zero-half property, this just cleans up numerics).
    for (int c = 0; c < n; ++c) {
      if (lp.vars[c].type == VarType::Integer || lp.vars[c].type == VarType::Binary) {
        coeff[c] = std::round(coeff[c]);
      }
    }

    // Verify violation on actual built cut.
    double lhs = coeff.dot(x);
    double actual_viol = lhs - rhs;
    if (actual_viol < 1e-4) continue;

    // Parallelism filter: reject if too similar to existing cuts.
    // P2.2: Use stricter threshold (0.9) for better orthogonality.
    double cnorm = coeff.norm();
    if (cnorm < 1e-12) continue;
    bool parallel = false;
    for (int ci = 0; ci < static_cast<int>(cut_norms.size()); ++ci) {
      double dot = coeff.dot(cut_rows_out[ci]);
      if (std::abs(dot) > 0.90 * cnorm * cut_norms[ci]) {
        parallel = true;
        break;
      }
    }
    if (parallel) continue;

    cut_norms.push_back(cnorm);
    cut_rows_out.push_back(std::move(coeff));
    cut_rhs_out.push_back(rhs);
    ++added;
  }

  if (!cut_rows_out.empty()) {
    add_rows_to_lp(lp, cut_rows_out, cut_rhs_out);
  }
  return added;
}

int add_cuts(LPModel& lp,
             const Eigen::VectorXd& x,
             const SimplexResult* simplex,
             const BCOptions& opt,
             int max_cuts,
             const std::shared_ptr<BasisOps>& sbasis,
             CutFamilyTracker* tracker,
             const CliqueTable* clique_table) {
  if (opt.cuts == CutType::None || max_cuts <= 0) {
    return 0;
  }

  int budget = max_cuts;
  int total = 0;

  // Build RowMajor view once so that compute_efficacy (called once per cut
  // family) does not re-copy the full LP matrix each time.
  Eigen::SparseMatrix<double, Eigen::RowMajor> efficacy_A_row = lp.A;

  // Helper to compute average efficacy of cuts added to LP rows [start_row, end_row).
  // Efficacy = violation / ||coeff|| where violation = coeff*x - rhs > 0.
  auto compute_efficacy = [&](int start_row, int end_row) -> double {
    if (end_row <= start_row) return 0.0;
    const int n = static_cast<int>(x.size());
    // Use a fresh RowMajor view when new rows were added since the last call.
    if (efficacy_A_row.rows() != lp.A.rows()) efficacy_A_row = lp.A;
    const auto& A_row = efficacy_A_row;
    double sum = 0.0;
    int count = 0;
    for (int i = start_row; i < end_row; ++i) {
      double lhs = 0.0;
      double norm_sq = 0.0;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, i); it; ++it) {
        if (it.col() < n) {
          lhs += it.value() * x[it.col()];
          norm_sq += it.value() * it.value();
        }
      }
      if (norm_sq > 1e-12) {
        double violation = lhs - lp.b[i];  // A*x <= b means violation = A*x - b > 0
        double efficacy = std::max(0.0, violation) / std::sqrt(norm_sq);
        sum += efficacy;
        ++count;
      }
    }
    return (count > 0) ? (sum / count) : 0.0;
  };

  // Helper to spend budget on a cut family and track efficacy.
  auto spend_tracked = [&](CutFamily family, const std::function<int(int)>& f, int family_budget) {
    if (budget <= 0) return;
    if (tracker && tracker->should_skip(family)) {
      tracker->get(family).skipped_rounds += 1;
      return;
    }
    
    const int start_rows = static_cast<int>(lp.A.rows());
    const int actual_budget = std::min(budget, family_budget);
    if (tracker) {
      tracker->get(family).attempts += 1;
      tracker->get(family).budget_total += actual_budget;
    }
    const int add = std::max(0, f(actual_budget));
    const int end_rows = static_cast<int>(lp.A.rows());

    if (tracker && add > 0) {
      double eff = compute_efficacy(start_rows, end_rows);
      tracker->get(family).add_round(eff);
      tracker->get(family).cuts_total += add;
      tracker->get(family).efficacy_sum += eff * add;
    } else if (tracker) {
      tracker->get(family).zero_rounds += 1;
    }

    total += add;
    budget -= add;
  };

  auto spend = [&](const std::function<int(int)>& f) {
    if (budget <= 0) {
      return;
    }
    const int add = std::max(0, f(budget));
    total += add;
    budget -= add;
  };

  if (opt.cuts == CutType::IntRounding) {
    spend([&](int b) { return add_mir_like_cuts(lp, x, CutType::IntRounding, b); });
    return total;
  }
  if (opt.cuts == CutType::MIR) {
    // True MIR: first try basis-based MIR, then row-based MIR
    if (simplex != nullptr) {
      spend_tracked(CutFamily::MIR, [&](int b) { return add_basis_mir_cuts(lp, x, *simplex, opt, b, sbasis); }, budget);
    }
    spend_tracked(CutFamily::MIR, [&](int b) { return add_row_mir_cuts(lp, x, b); }, budget);
    return total;
  }
  if (opt.cuts == CutType::Gomory) {
    if (simplex != nullptr) {
      spend_tracked(CutFamily::Gomory,
                    [&](int b) {
	                      return add_transformed_tableau_cuts(
	                          lp, x, *simplex, opt, b, sbasis, nullptr, nullptr,
	                          nullptr);
                    },
                    budget);
    }
    return total;
  }
  if (opt.cuts == CutType::Cover) {
    spend_tracked(CutFamily::Cover, [&](int b) { return add_cover_cuts(lp, x, b); }, budget);
    if (opt.enable_projected_capacity_cuts) {
      spend_tracked(CutFamily::Cover, [&](int b) { return add_projected_capacity_cover_cuts(lp, x, b); }, budget);
    }
    return total;
  }
  if (opt.cuts == CutType::FlowCover) {
    spend_tracked(CutFamily::FlowCover, [&](int b) { return add_flow_cover_cuts(lp, x, b); }, budget);
    return total;
  }
  if (opt.cuts == CutType::ImpliedBound) {
    spend_tracked(CutFamily::ImpliedBound, [&](int b) { return add_implied_bound_cuts(lp, x, b); }, budget);
    return total;
  }

  // CutType::All — generate all applicable cut families.
  // Gomory gets full budget first (strongest cuts). Supplementary cuts
  // share the remaining budget to add complementary strength.
  // P2.1: Skip families with low moving-average efficacy.
  // P6.3: Reduce supplementary cut budgets and disable expensive families
  //        on very large problems (n > 20000).
  const bool xlarge = (static_cast<int>(x.size()) > 20000);
  const bool large = (static_cast<int>(x.size()) > 10000);

  // Prefer violated cliques from the persistent clique table when available.
  // The table is built once at root; this amortises the O(m*k^2) conflict
  // graph construction across every cut round.
  auto add_clique_cuts_from_table = [&](int b) -> int {
    if (b <= 0 || clique_table == nullptr || clique_table->empty()) return 0;
    const int nx = static_cast<int>(x.size());
    auto cliques = clique_table->find_violated_cliques(x, b, 1e-7, xlarge ? 128 : 512);
    if (cliques.empty()) return 0;
    std::vector<Eigen::VectorXd> rows;
    std::vector<double> rhs;
    rows.reserve(cliques.size());
    rhs.reserve(cliques.size());
    for (const auto& cq : cliques) {
      Eigen::VectorXd cut = Eigen::VectorXd::Zero(nx);
      for (int c : cq.members) if (c < nx) cut[c] = 1.0;
      rows.push_back(std::move(cut));
      rhs.push_back(1.0);
    }
    add_rows_to_lp(lp, rows, rhs);
    return static_cast<int>(rows.size());
  };

  // 1. Gomory cuts (strongest, need simplex basis) — full budget
  if (simplex != nullptr) {
    spend_tracked(CutFamily::Gomory,
                  [&](int b) {
	                    return add_transformed_tableau_cuts(
	                        lp, x, *simplex, opt, b, sbasis, nullptr, nullptr,
	                        nullptr);
                  },
                  budget);
  }
  // 2. Basis-MIR cuts (complemented single-row MIR, stronger than row-MIR, needs simplex basis)
  if (simplex != nullptr && !xlarge) {
    spend_tracked(CutFamily::MIR, [&](int b) { return add_basis_mir_cuts(lp, x, *simplex, opt, b, sbasis); }, xlarge ? 0 : (large ? 2 : 4));
  }
  // 3. Row-based MIR cuts (works without basis, complementary to Gomory)
  spend_tracked(CutFamily::MIR, [&](int b) { return add_row_mir_cuts(lp, x, b); }, xlarge ? 2 : (large ? 3 : 5));
  // 3. Cover cuts (with sequential up-lifting)
  if (opt.enable_projected_capacity_cuts) {
    spend_tracked(CutFamily::Cover,
                  [&](int b) { return add_projected_capacity_cover_cuts(lp, x, b); },
                  budget);
  }
  spend_tracked(CutFamily::Cover, [&](int b) { return add_cover_cuts(lp, x, b); }, xlarge ? 2 : (large ? 3 : 5));

  // Clique cuts: always attempt when the persistent table is available
  // (near-zero cost when table is empty; fast lookup otherwise).
  if (clique_table != nullptr && !clique_table->empty()) {
    spend_tracked(CutFamily::Clique, add_clique_cuts_from_table,
                  xlarge ? 2 : (large ? 3 : 5));
  }
  // Implied-bound cuts: enable even on large problems with tight budget.
  // Per-row scan is O(m) with constant work per row; cheap at all sizes.
  spend_tracked(CutFamily::ImpliedBound,
                [&](int b) { return add_implied_bound_cuts(lp, x, b); },
                xlarge ? 1 : (large ? 2 : 3));

  // Remaining expensive supplementary families only on smaller problems.
  if (!large) {
    // Fallback clique cuts (on-the-fly conflict graph) when no table present.
    if (clique_table == nullptr) {
      spend_tracked(CutFamily::Clique, [&](int b) { return add_clique_cuts(lp, x, b); }, 5);
    }
    // Zero-half cuts (pairwise {0,1/2}-aggregation) — disable on very large
    if (!xlarge) {
      spend_tracked(CutFamily::ZeroHalf, [&](int b) { return add_zero_half_cuts(lp, x, b); }, 5);
    }
    // Flow-cover separation is intentionally not part of the generic All
    // bundle.  The legacy implementation only infers capacity structure from
    // signed row coefficients; on presolved/ranged rows this can produce rows
    // that cut off the true integer frontier.  Keep it available only when a
    // caller explicitly requests CutType::FlowCover for diagnostics.
  }
  return total;
}

}  // namespace mipsolvers::engine::detail
