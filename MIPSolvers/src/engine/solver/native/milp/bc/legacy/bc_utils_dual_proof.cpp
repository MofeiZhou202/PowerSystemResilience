/// @file bc_utils_dual_proof.cpp
/// @brief Dual-proof row construction and target-bound trail resolution.
///
/// Extracted from bc_utils.cpp; declarations live in bc_utils.hpp.

#include "mipsolvers/engine/detail/bc_utils.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <queue>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_numerics.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"

namespace mipsolvers::engine::detail {

namespace {

Eigen::VectorXd scaled_row_duals_from_simplex(const SimplexResult& simplex,
                                              const StandardFormLP* form_override = nullptr) {
  const StandardFormLP& sf =
      form_override != nullptr ? *form_override : simplex.form;
  const int m = static_cast<int>(sf.A.rows());
  Eigen::VectorXd y_scaled = Eigen::VectorXd::Zero(m);
  if (!simplex.result.stats.success ||
      static_cast<int>(simplex.basis.index_count()) != m ||
      static_cast<int>(sf.c_max.size()) != static_cast<int>(sf.A.cols())) {
    return y_scaled;
  }

  Eigen::VectorXd c_b(m);
  const auto& basis = simplex.basis.basis_indices();
  for (int i = 0; i < m; ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    c_b[i] = (col >= 0 && col < static_cast<int>(sf.c_max.size()))
                 ? sf.c_max[col]
                 : 0.0;
  }

  // Prefer the sparse basis object: after eta / Forrest-Tomlin updates it is
  // the first-class basis used to compute reduced costs.  A dense inverse may
  // be only the crash/base inverse and can give a stale, degenerate proof.
  if (simplex.basis.cached_sparse_basis) {
    // The cached factor stores a raw pointer to the standard-form matrix it was
    // built against, which may dangle if the SimplexResult's form was moved
    // after the solve.  Rebind to the live sf.A before solving so btran never
    // dereferences freed memory.  rebind_A invalidates the factor when the
    // stored basis is incompatible with sf.A, in which case btran returns an
    // empty vector and the caller falls back to solver row duals or rejects the
    // proof -- never a crash, never a wrong dual (validated by backward error).
    rebind_sparse_basis_matrix(simplex.basis.cached_sparse_basis, sf.A);
    y_scaled = sparse_basis_btran(simplex.basis.cached_sparse_basis, c_b);
  } else if (simplex.basis_inverse.rows() == m &&
             simplex.basis_inverse.cols() == m) {
    y_scaled.noalias() = simplex.basis_inverse.transpose() * c_b;
  }

  return y_scaled;
}

Eigen::VectorXd effective_min_row_duals_from_scaled(const StandardFormLP& sf,
                                                    const Eigen::VectorXd& y_scaled) {
  const int m = static_cast<int>(sf.A.rows());
  Eigen::VectorXd y_eff = Eigen::VectorXd::Zero(m);
  const bool have_row_scale = sf.row_scale.size() == m;
  for (int i = 0; i < m; ++i) {
    const double row_scale = have_row_scale ? sf.row_scale[i] : 1.0;
    const double row_sign =
        (i < static_cast<int>(sf.row_sign.size())) ? static_cast<double>(sf.row_sign[i]) : 1.0;
    // The simplex solves max -c_min^T x in signed/scaled standard form.
    // Mapping back to the effective minimization model gives
    // y_eff = - row_sign * row_scale * y_scaled.
    y_eff[i] = -row_sign * row_scale * y_scaled[i];
  }
  return y_eff;
}

Eigen::VectorXd scaled_row_duals_from_effective(const StandardFormLP& sf,
                                                const Eigen::VectorXd& y_eff) {
  const int m = static_cast<int>(sf.A.rows());
  Eigen::VectorXd y_scaled = Eigen::VectorXd::Zero(m);
  if (y_eff.size() != m) return y_scaled;
  const bool have_row_scale = sf.row_scale.size() == m;
  for (int i = 0; i < m; ++i) {
    const double row_scale = have_row_scale ? sf.row_scale[i] : 1.0;
    const double row_sign =
        (i < static_cast<int>(sf.row_sign.size())) ? static_cast<double>(sf.row_sign[i]) : 1.0;
    if (row_scale == 0.0 || !std::isfinite(row_scale)) return Eigen::VectorXd();
    y_scaled[i] = -row_sign * y_eff[i] / row_scale;
  }
  return y_scaled;
}

double max_scaled_stationarity_error(const StandardFormLP& sf,
                                     const SimplexResult& simplex,
                                     const Eigen::VectorXd& y_scaled) {
  const int m = static_cast<int>(sf.A.rows());
  const int sf_n = static_cast<int>(sf.c_max.size());
  if (y_scaled.size() != m || simplex.reduced_costs.size() < sf_n ||
      sf.A.cols() != sf_n || !y_scaled.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  const Eigen::VectorXd rc =
      sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_scaled);
  double err = 0.0;
  for (int j = 0; j < sf_n; ++j) {
    err = std::max(err, std::abs(simplex.reduced_costs[j] - rc[j]));
  }
  return err;
}

double sparse_activity(const Eigen::SparseVector<double>& coeff,
                       const Eigen::VectorXd& x) {
  double v = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(coeff); it; ++it) {
    const int j = static_cast<int>(it.index());
    if (j >= 0 && j < x.size()) v += it.value() * x[j];
  }
  return v;
}

bool compute_activity_range(const Eigen::SparseVector<double>& coeff,
                            const Eigen::VectorXd& lb,
                            const Eigen::VectorXd& ub,
                            double& min_activity,
                            double& max_activity) {
  min_activity = 0.0;
  max_activity = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(coeff); it; ++it) {
    const int j = static_cast<int>(it.index());
    if (j < 0 || j >= lb.size() || j >= ub.size()) return false;
    const double a = it.value();
    if (a >= 0.0) {
      if (!std::isfinite(lb[j]) || !std::isfinite(ub[j])) return false;
      min_activity += a * lb[j];
      max_activity += a * ub[j];
    } else {
      if (!std::isfinite(lb[j]) || !std::isfinite(ub[j])) return false;
      min_activity += a * ub[j];
      max_activity += a * lb[j];
    }
  }
  return true;
}

bool dual_proof_identity_ok(DualProofRow& proof,
                            double lp_objective,
                            double incumbent_obj,
                            double tol) {
  proof.expected_lp_gap = incumbent_obj - lp_objective;
  proof.actual_lp_gap = proof.rhs - proof.lp_activity;
  proof.gap_error = std::abs(proof.actual_lp_gap - proof.expected_lp_gap);
  const double scale =
      std::max({1.0, std::abs(incumbent_obj), std::abs(lp_objective),
                std::abs(proof.rhs), std::abs(proof.lp_activity)});
  const double accept =
      std::max({1e-6, 1000.0 * std::max(tol, 1e-12),
                1e-9 * scale});
  if (!std::isfinite(proof.expected_lp_gap) ||
      !std::isfinite(proof.actual_lp_gap) ||
      !std::isfinite(proof.gap_error) || proof.gap_error > accept) {
    proof.reject_reason =
        "dual_proof_identity_mismatch(expected=" +
        std::to_string(proof.expected_lp_gap) +
        ",actual=" + std::to_string(proof.actual_lp_gap) +
        ",err=" + std::to_string(proof.gap_error) +
        ",tol=" + std::to_string(accept) + ")";
    proof.valid = false;
    return false;
  }
  return true;
}

}  // namespace

bool build_full_dual_proof_row(const LPModel& lp,
                               const SimplexResult& simplex,
                               const Eigen::VectorXd& root_lb,
                               const Eigen::VectorXd& root_ub,
                               const Eigen::VectorXd& x_lp,
                               double lp_objective,
                               double incumbent_obj,
                               DualProofRow& out,
                               double tol,
                               const StandardFormLP* form_override) {
  out = DualProofRow{};
  const StandardFormLP& sf =
      form_override != nullptr ? *form_override : simplex.form;
  const int n = static_cast<int>(lp.vars.size());
  const int m_ineq = static_cast<int>(lp.A.rows());
  const int m_eq = static_cast<int>(lp.Aeq.rows());
  const int m = m_ineq + m_eq;
  if (!std::isfinite(incumbent_obj)) {
    out.reject_reason = "no_finite_incumbent";
    return false;
  }
  if (!simplex.result.stats.success) {
    out.reject_reason = "simplex_not_optimal";
    return false;
  }
  if (sf.A.rows() != m || sf.n_original != n) {
    out.reject_reason =
        "simplex_form_mismatch(sf_m=" + std::to_string(sf.A.rows()) +
        ",lp_m=" + std::to_string(m) +
        ",sf_n=" + std::to_string(sf.n_original) +
        ",lp_n=" + std::to_string(n) + ")";
    return false;
  }
  if (lp.c.size() != n || root_lb.size() != n || root_ub.size() != n ||
      x_lp.size() != n) {
    out.reject_reason = "dimension_mismatch";
    return false;
  }

  const Eigen::VectorXd basis_y_scaled =
      scaled_row_duals_from_simplex(simplex, &sf);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(m);
  Eigen::VectorXd y_scaled = basis_y_scaled;
  const bool have_basis_row_duals =
      basis_y_scaled.size() == m && basis_y_scaled.allFinite();
  const bool have_solver_row_duals =
      simplex.result.constraint_duals.size() == m &&
      simplex.result.constraint_duals.allFinite();
  if (have_solver_row_duals) {
    const Eigen::VectorXd solver_y_scaled =
        scaled_row_duals_from_effective(sf, simplex.result.constraint_duals);
    const double solver_staterr =
        max_scaled_stationarity_error(sf, simplex, solver_y_scaled);
    const double basis_staterr =
        have_basis_row_duals
            ? max_scaled_stationarity_error(sf, simplex, basis_y_scaled)
            : std::numeric_limits<double>::infinity();
    const double accept_tol = std::max(1e-6, 1000.0 * tol);
    const bool solver_matches =
        std::isfinite(solver_staterr) &&
        (solver_staterr <= accept_tol ||
         (std::isfinite(basis_staterr) &&
          solver_staterr <= std::max(10.0 * basis_staterr, accept_tol)));
    if (solver_matches) {
      out.used_solver_row_duals = true;
      y = simplex.result.constraint_duals;
      y_scaled = solver_y_scaled;
    } else if (have_basis_row_duals) {
      y = effective_min_row_duals_from_scaled(sf, basis_y_scaled);
      y_scaled = basis_y_scaled;
    } else {
      out.reject_reason = "row_dual_stationarity_invalid";
      return false;
    }
  } else {
    if (!have_basis_row_duals) {
      out.reject_reason = "row_dual_unavailable";
      return false;
    }
    y = effective_min_row_duals_from_scaled(sf, basis_y_scaled);
    y_scaled = basis_y_scaled;
  }

  const Eigen::VectorXd c_min = (lp.sense == Sense::Minimize) ? lp.c : (-lp.c);
  Eigen::VectorXd certificate_dense = c_min;
  for (int col = 0; col < lp.A.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      const int r = static_cast<int>(it.row());
      if (r >= 0 && r < m_ineq && col < n) {
        certificate_dense[col] -= it.value() * y[r];
      }
    }
  }
  for (int col = 0; col < lp.Aeq.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, col); it; ++it) {
      const int r = m_ineq + static_cast<int>(it.row());
      if (r >= m_ineq && r < m && col < n) {
        certificate_dense[col] -= it.value() * y[r];
      }
    }
  }

  out.max_stationarity_error = 0.0;
  out.max_scaled_stationarity_error = 0.0;
  out.max_cost_mapping_error = 0.0;
  const bool have_col_scale = sf.col_scale.size() >= n;
  const int sf_n = static_cast<int>(sf.c_max.size());
  if (y_scaled.size() == m && simplex.reduced_costs.size() >= sf_n) {
    Eigen::VectorXd scaled_recomputed =
        sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_scaled);
    for (int j = 0; j < sf_n; ++j) {
      out.max_scaled_stationarity_error =
          std::max(out.max_scaled_stationarity_error,
                   std::abs(simplex.reduced_costs[j] - scaled_recomputed[j]));
    }
  }
  if (simplex.reduced_costs.size() >= n) {
    for (int j = 0; j < n; ++j) {
      const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
      out.max_cost_mapping_error =
          std::max(out.max_cost_mapping_error,
                   std::abs(sf.c_max[j] + c_min[j] * col_scale));
      const double expected_rc = -certificate_dense[j] * col_scale;
      out.max_stationarity_error =
          std::max(out.max_stationarity_error,
                   std::abs(simplex.reduced_costs[j] - expected_rc));
    }
  }

  Eigen::VectorXd proof_y = Eigen::VectorXd::Zero(m);

  // HiGHS-style row side selection.  Positive row duals contribute through a
  // finite row lower side; negative row duals contribute through a finite row
  // upper side.  Equality rows have both sides equal and can use either sign.
  double rhs = incumbent_obj;
  for (int i = 0; i < m_ineq; ++i) {
    if (y[i] > tol) {
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (std::isfinite(lhs)) {
        proof_y[i] = y[i];
        rhs -= y[i] * lhs;
      } else {
        ++out.ignored_row_duals;
      }
    } else if (y[i] < -tol) {
      proof_y[i] = y[i];
      rhs -= y[i] * lp.b[i];
    }
  }
  for (int i = 0; i < m_eq; ++i) {
    const int r = m_ineq + i;
    if (std::abs(y[r]) > tol) {
      proof_y[r] = y[r];
      rhs -= y[r] * lp.beq[i];
    }
  }
  for (int i = 0; i < m; ++i) {
    if (std::abs(proof_y[i]) > tol) ++out.row_dual_nnz;
  }

  Eigen::VectorXd dense = c_min;

  for (int col = 0; col < lp.A.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      const int r = static_cast<int>(it.row());
      if (r >= 0 && r < m_ineq && col < n) {
        dense[col] -= it.value() * proof_y[r];
      }
    }
  }
  for (int col = 0; col < lp.Aeq.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, col); it; ++it) {
      const int r = m_ineq + static_cast<int>(it.row());
      if (r >= m_ineq && r < m && col < n) {
        dense[col] -= it.value() * proof_y[r];
      }
    }
  }
  // Mirror the current HiGHS proof cleanup: if a fixed or continuous column is
  // already at the domain side that makes its coefficient redundant, shift the
  // corresponding bound activity into the RHS and remove the coefficient.  We
  // keep non-fixed integer coefficients, and also non-fixed objective-effective
  // continuous coefficients.  The latter are the reason-side bound changes used
  // by objective propagation: removing a continuous objective column that is at
  // its current lower/upper bound erases the very proof mass needed to explain
  // an implied later bound change on that column.
  for (int j = 0; j < n; ++j) {
    const bool fixed =
        std::isfinite(root_lb[j]) && std::isfinite(root_ub[j]) &&
        std::abs(root_ub[j] - root_lb[j]) <= tol;
    const double a = dense[j];
    if (std::abs(a) <= tol) continue;
    const bool objective_effective_continuous =
        !fixed && lp.vars[j].type == VarType::Continuous &&
        std::abs(c_min[j]) > tol &&
        ((a > 0.0 && std::isfinite(root_lb[j]) &&
          std::isfinite(root_ub[j]) && root_ub[j] > root_lb[j] + tol) ||
         (a < 0.0 && std::isfinite(root_lb[j]) &&
          std::isfinite(root_ub[j]) && root_ub[j] > root_lb[j] + tol));
    if (objective_effective_continuous) continue;
    if (!fixed && lp.vars[j].type != VarType::Continuous) continue;
    if (a > 0.0 && std::isfinite(root_lb[j]) &&
        x_lp[j] - root_lb[j] <= std::max(1e-7, 10.0 * tol)) {
      rhs -= a * root_lb[j];
      dense[j] = 0.0;
      ++out.removed_bound_coefficients;
    } else if (a < 0.0 && std::isfinite(root_ub[j]) &&
               root_ub[j] - x_lp[j] <= std::max(1e-7, 10.0 * tol)) {
      rhs -= a * root_ub[j];
      dense[j] = 0.0;
      ++out.removed_bound_coefficients;
    }
  }

  out.coeff = dense_to_sparse_cut(dense, std::max(1e-12, tol * 1e-2));
  out.nnz = static_cast<int>(out.coeff.nonZeros());
  out.rhs = rhs;
  out.lp_activity = sparse_activity(out.coeff, x_lp);
  if (!dual_proof_identity_ok(out, lp_objective, incumbent_obj, tol)) {
    return false;
  }

  if (!compute_activity_range(out.coeff, root_lb, root_ub,
                              out.min_activity, out.max_activity)) {
    out.reject_reason = "infinite_activity";
    return false;
  }

  const double maxabscoef = out.max_activity - out.rhs;
  if (std::isfinite(maxabscoef) && maxabscoef > tol) {
    Eigen::VectorXd tightened = Eigen::VectorXd::Zero(n);
    for (Eigen::SparseVector<double>::InnerIterator it(out.coeff); it; ++it) {
      tightened[static_cast<int>(it.index())] = it.value();
    }
    double tightened_rhs = out.rhs;
    for (int j = 0; j < n; ++j) {
      if (lp.vars[j].type == VarType::Continuous) continue;
      const double a = tightened[j];
      if (a > maxabscoef) {
        const double delta = a - maxabscoef;
        tightened_rhs -= delta * root_ub[j];
        tightened[j] = maxabscoef;
        ++out.tightened_coefficients;
      } else if (a < -maxabscoef) {
        const double delta = -a - maxabscoef;
        tightened_rhs += delta * root_lb[j];
        tightened[j] = -maxabscoef;
        ++out.tightened_coefficients;
      }
    }
    if (out.tightened_coefficients > 0) {
      out.coeff = dense_to_sparse_cut(tightened, std::max(1e-12, tol * 1e-2));
      out.nnz = static_cast<int>(out.coeff.nonZeros());
      out.rhs = tightened_rhs;
      out.lp_activity = sparse_activity(out.coeff, x_lp);
      if (!dual_proof_identity_ok(out, lp_objective, incumbent_obj, tol)) {
        return false;
      }
      if (!compute_activity_range(out.coeff, root_lb, root_ub,
                                  out.min_activity, out.max_activity)) {
        out.reject_reason = "infinite_activity_after_tightening";
        return false;
      }
    }
  }

  out.valid = true;
  return true;
}

bool build_reduced_cost_mass_dual_proof_row(
    const LPModel& lp,
    const SimplexResult& simplex,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_lp,
    double lp_objective,
    double incumbent_obj,
    DualProofRow& out,
    double tol,
    const StandardFormLP* form_override,
    const std::vector<char>* implied_integer_cols,
    const std::vector<char>* force_keep_cols) {
  out = DualProofRow{};
  const StandardFormLP& sf =
      form_override != nullptr ? *form_override : simplex.form;
  const int n = static_cast<int>(lp.vars.size());
  if (!std::isfinite(incumbent_obj)) {
    out.reject_reason = "no_finite_incumbent";
    return false;
  }
  if (!simplex.result.stats.success) {
    out.reject_reason = "simplex_not_optimal";
    return false;
  }
  if (sf.n_original != n || lp.c.size() != n || root_lb.size() != n ||
      root_ub.size() != n || x_lp.size() != n ||
      simplex.reduced_costs.size() < n) {
    out.reject_reason = "dimension_mismatch";
    return false;
  }
  const auto& basis = simplex.basis.basis_indices();
  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
  }
  const bool have_col_scale = sf.col_scale.size() >= n;
  Eigen::VectorXd dense = Eigen::VectorXd::Zero(n);
  double rhs = incumbent_obj - lp_objective;
  int shifted = 0;
  int kept = 0;
  double max_abs_kept = 0.0;
  auto implied_integer = [&](int j) {
    return implied_integer_cols != nullptr &&
           j >= 0 && j < static_cast<int>(implied_integer_cols->size()) &&
           (*implied_integer_cols)[static_cast<std::size_t>(j)] != 0;
  };
  auto force_keep = [&](int j) {
    return force_keep_cols != nullptr &&
           j >= 0 && j < static_cast<int>(force_keep_cols->size()) &&
           (*force_keep_cols)[static_cast<std::size_t>(j)] != 0;
  };

  for (int j = 0; j < n; ++j) {
    const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
    if (!std::isfinite(col_scale) || std::abs(col_scale) <= 1e-30) {
      out.reject_reason = "bad_col_scale";
      return false;
    }
    // The simplex kernel stores standard-form maximization reduced costs.
    // In original minimization space the cutoff proof coefficient is the
    // opposite sign, mapped back through the column scaling.
    const double a = -simplex.reduced_costs[j] / col_scale;
    if (!std::isfinite(a) || std::abs(a) <= std::max(1e-12, tol)) {
      continue;
    }

    rhs += a * x_lp[j];

    const bool keep_target = force_keep(j);
    const bool integer_col =
        is_integer_type(lp.vars[static_cast<std::size_t>(j)]) ||
        implied_integer(j);
    bool keep = keep_target || (integer_col && !is_basic[static_cast<std::size_t>(j)]);
    if (keep) {
      if (!keep_target) {
        if (a > 0.0) {
          keep = std::isfinite(root_lb[j]) &&
                 x_lp[j] > root_lb[j] + std::max(1e-7, 10.0 * tol);
        } else {
          keep = std::isfinite(root_ub[j]) &&
                 x_lp[j] < root_ub[j] - std::max(1e-7, 10.0 * tol);
        }
      }
    }

    if (keep) {
      dense[j] = a;
      ++kept;
      max_abs_kept = std::max(max_abs_kept, std::abs(a));
    } else if (a > 0.0) {
      if (!std::isfinite(root_lb[j])) {
        out.reject_reason = "infinite_removed_lower_bound";
        return false;
      }
      rhs -= a * root_lb[j];
      ++shifted;
    } else {
      if (!std::isfinite(root_ub[j])) {
        out.reject_reason = "infinite_removed_upper_bound";
        return false;
      }
      rhs -= a * root_ub[j];
      ++shifted;
    }
  }

  if (kept <= 0 || max_abs_kept <= std::max(1e-12, tol)) {
    out.reject_reason = "no_local_reduced_cost_mass";
    return false;
  }

  out.coeff = dense_to_sparse_cut(dense, std::max(1e-12, tol * 1e-2));
  out.nnz = static_cast<int>(out.coeff.nonZeros());
  out.rhs = rhs;
  out.lp_activity = sparse_activity(out.coeff, x_lp);
  if (!dual_proof_identity_ok(out, lp_objective, incumbent_obj, tol)) {
    return false;
  }
  out.removed_bound_coefficients = shifted;
  out.row_dual_nnz = kept;
  out.used_solver_row_duals = false;

  if (!compute_activity_range(out.coeff, root_lb, root_ub,
                              out.min_activity, out.max_activity)) {
    out.reject_reason = "infinite_activity";
    return false;
  }

  out.valid = true;
  return true;
}

int apply_dual_proof_domain_fixing(
    const std::vector<VariableMeta>& vars,
    const DualProofRow& proof,
    double int_tol,
    double lp_tol,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    std::vector<BoundChangeInfo>* changes_out) {
  const int n = static_cast<int>(vars.size());
  if (!proof.valid || proof.coeff.size() < n || proof.coeff.nonZeros() <= 0 ||
      node_lb.size() < n || node_ub.size() < n ||
      !std::isfinite(proof.rhs)) {
    return 0;
  }

  const double tol = std::max({1e-9, int_tol, lp_tol});
  const double proof_tol =
      std::max(1e-7, 100.0 * tol * std::max(1.0, std::abs(proof.rhs)));

  double min_activity = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(proof.coeff); it; ++it) {
    const int j = static_cast<int>(it.index());
    if (j < 0 || j >= n) return 0;
    const double a = it.value();
    if (!std::isfinite(a)) return 0;
    if (a > 0.0) {
      if (!std::isfinite(node_lb[j])) return 0;
      min_activity += a * node_lb[j];
    } else if (a < 0.0) {
      if (!std::isfinite(node_ub[j])) return 0;
      min_activity += a * node_ub[j];
    }
  }
  if (!std::isfinite(min_activity)) return 0;
  if (min_activity > proof.rhs + proof_tol) return -1;

  const double safe_slack = std::max(0.0, proof.rhs - min_activity) + proof_tol;
  int tightened = 0;
  for (Eigen::SparseVector<double>::InnerIterator it(proof.coeff); it; ++it) {
    const int j = static_cast<int>(it.index());
    const double a = it.value();
    if (j < 0 || j >= n || std::abs(a) <= tol) continue;

    if (a > 0.0) {
      if (!std::isfinite(node_lb[j])) continue;
      double new_ub = node_lb[j] + safe_slack / a;
      if (!std::isfinite(new_ub)) continue;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        new_ub = std::floor(new_ub + int_tol);
      }
      new_ub = std::min(new_ub, node_ub[j]);
      if (new_ub < node_lb[j] - tol) return -1;
      if (new_ub < node_ub[j] - tol) {
        const double old = node_ub[j];
        node_ub[j] = new_ub;
        if (changes_out != nullptr) {
          changes_out->push_back(
              BoundChangeInfo{j, new_ub - old, false, old, new_ub});
        }
        ++tightened;
      }
    } else {
      if (!std::isfinite(node_ub[j])) continue;
      double new_lb = node_ub[j] + safe_slack / a;
      if (!std::isfinite(new_lb)) continue;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        new_lb = std::ceil(new_lb - int_tol);
      }
      new_lb = std::max(new_lb, node_lb[j]);
      if (new_lb > node_ub[j] + tol) return -1;
      if (new_lb > node_lb[j] + tol) {
        const double old = node_lb[j];
        node_lb[j] = new_lb;
        if (changes_out != nullptr) {
          changes_out->push_back(
              BoundChangeInfo{j, new_lb - old, true, old, new_lb});
        }
        ++tightened;
      }
    }
  }

  return tightened;
}

DualProofResolutionStatus resolve_dual_proof_target_bound_from_local_trail(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd* x_relax,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const DomainReasonBound& target_bound,
    int max_literals,
    const Eigen::SparseVector<double>& coeff_sparse,
    double rhs,
    DualProofTrailResolution& out,
    const Eigen::SparseVector<double>* priority_coeff_sparse) {
  out = DualProofTrailResolution{};
  auto fail = [&](DualProofResolutionStatus status) {
    out.status = status;
    return status;
  };
  auto success = [&]() {
    out.status = DualProofResolutionStatus::Success;
    return DualProofResolutionStatus::Success;
  };
  (void)x_relax;
  (void)priority_coeff_sparse;
  const int n = static_cast<int>(vars.size());
  if (max_literals <= 0 || n <= 0 || coeff_sparse.size() < n ||
      coeff_sparse.nonZeros() <= 0 || !std::isfinite(rhs) ||
      root_lb.size() < n || root_ub.size() < n) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  const BranchDomainLiteral& target_lit = target_bound.bound;
  if (target_lit.var_idx < 0 || target_lit.var_idx >= n ||
      !std::isfinite(target_lit.value)) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);
  for (Eigen::SparseVector<double>::InnerIterator it(coeff_sparse); it; ++it) {
    const int j = static_cast<int>(it.index());
    if (j >= 0 && j < n && std::isfinite(it.value())) {
      coeff[static_cast<std::size_t>(j)] = it.value();
    }
  }

  double root_min_activity = 0.0;
  for (int j = 0; j < n; ++j) {
    const double a = coeff[static_cast<std::size_t>(j)];
    if (a > 0.0) {
      if (!std::isfinite(root_lb[j])) {
        return fail(DualProofResolutionStatus::InvalidInput);
      }
      root_min_activity += a * root_lb[j];
    } else if (a < 0.0) {
      if (!std::isfinite(root_ub[j])) {
        return fail(DualProofResolutionStatus::InvalidInput);
      }
      root_min_activity += a * root_ub[j];
    }
  }

  const int target = target_lit.var_idx;
  const double a0 = coeff[static_cast<std::size_t>(target)];
  if ((target_lit.is_lb && a0 >= -1e-12) ||
      (!target_lit.is_lb && a0 <= 1e-12)) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  const bool integer_target =
      is_integer_type(vars[static_cast<std::size_t>(target)]);
  double relaxed_bound = target_lit.value;
  if (integer_target) {
    const double relax = std::max(0.0, 1.0 - 10.0 * int_tol);
    relaxed_bound += target_lit.is_lb ? -relax : relax;
  } else {
    relaxed_bound += target_lit.is_lb ? -1e-7 : 1e-7;
  }

  double target_global_contribution = 0.0;
  if (a0 > 0.0) {
    if (!std::isfinite(root_lb[target])) {
      return fail(DualProofResolutionStatus::InvalidInput);
    }
    target_global_contribution = a0 * root_lb[target];
  } else {
    if (!std::isfinite(root_ub[target])) {
      return fail(DualProofResolutionStatus::InvalidInput);
    }
    target_global_contribution = a0 * root_ub[target];
  }

  const double residual_root_activity =
      root_min_activity - target_global_contribution;
  const double required_activity = rhs - a0 * relaxed_bound;
  const double proof_tol =
      std::max(1e-7, 1e-10 * (1.0 + std::abs(rhs)));
  const double eps = std::max(1e-9, int_tol);
  const bool have_native_trail =
      local_domain_trail != nullptr && !local_domain_trail->empty();
  const int target_pos = target_bound.trail_pos >= 0
      ? target_bound.trail_pos
      : (have_native_trail
             ? static_cast<int>(local_domain_trail->size())
             : static_cast<int>(branch_reasons.size() + reason_bounds.size()));
  if (target_pos < 0) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  auto reason_touches_target =
      [&](const std::vector<BranchDomainLiteral>& lits) {
    for (const auto& lit : lits) {
      if (lit.var_idx == target) return true;
    }
    return false;
  };

  auto literal_priority = [&](const BranchDomainLiteral& lit) -> double {
    if (lit.var_idx < 0 || lit.var_idx >= n) return 0.0;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    if (std::abs(a) <= 1e-12) return 0.0;
    if (lit.is_lb) {
      if (!std::isfinite(root_lb[lit.var_idx])) return 0.0;
      return std::abs(a * (lit.value - root_lb[lit.var_idx]));
    }
    if (!std::isfinite(root_ub[lit.var_idx])) return 0.0;
    return std::abs(a * (lit.value - root_ub[lit.var_idx]));
  };
  auto flipped_literal = [&](const BranchDomainLiteral& lit,
                             BranchDomainLiteral& flipped) -> bool {
    if (lit.var_idx < 0 || lit.var_idx >= n || !std::isfinite(lit.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
    if (lit.is_lb) {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::ceil(lit.value - 1e-9) - 1.0 : lit.value - 1e-7,
          false};
    } else {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::floor(lit.value + 1e-9) + 1.0 : lit.value + 1e-7,
          true};
    }
    return true;
  };
  auto proof_literal_delta = [&](const BranchDomainLiteral& lit,
                                 double* delta_out) -> bool {
    if (delta_out != nullptr) *delta_out = 0.0;
    if (lit.var_idx < 0 || lit.var_idx >= n ||
        !std::isfinite(lit.value)) {
      return false;
    }
    if (lit.var_idx == target) return false;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    if (a > 0.0 && lit.is_lb) {
      if (!std::isfinite(root_lb[lit.var_idx])) return false;
      const double delta = a * (lit.value - root_lb[lit.var_idx]);
      if (!std::isfinite(delta) || delta < -proof_tol) return false;
      if (delta_out != nullptr) *delta_out = std::max(0.0, delta);
      return true;
    }
    if (a < 0.0 && !lit.is_lb) {
      if (!std::isfinite(root_ub[lit.var_idx])) return false;
      const double delta = a * (lit.value - root_ub[lit.var_idx]);
      if (!std::isfinite(delta) || delta < -proof_tol) return false;
      if (delta_out != nullptr) *delta_out = std::max(0.0, delta);
      return true;
    }
    return false;
  };
  auto audited_activity_from_frontier =
      [&](const std::vector<BranchDomainLiteral>& frontier,
          double& audited_activity) -> bool {
    audited_activity = residual_root_activity;
    std::vector<char> seen_lower(static_cast<std::size_t>(n), 0);
    std::vector<char> seen_upper(static_cast<std::size_t>(n), 0);
    for (const auto& lit : frontier) {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value) || lit.var_idx == target) {
        return false;
      }
      char& seen = lit.is_lb ? seen_lower[static_cast<std::size_t>(lit.var_idx)]
                             : seen_upper[static_cast<std::size_t>(lit.var_idx)];
      if (seen != 0) return false;
      seen = 1;
      double delta = 0.0;
      if (!proof_literal_delta(lit, &delta)) return false;
      audited_activity += delta;
    }
    return std::isfinite(audited_activity);
  };
  auto frontier_implies_proof_frontier =
      [&](const std::vector<BranchDomainLiteral>& reason_frontier,
          const std::vector<BranchDomainLiteral>& needed_proof_frontier,
          double& audited_activity) -> bool {
    if (audited_activity_from_frontier(reason_frontier, audited_activity)) {
      if (audited_activity >= required_activity - proof_tol) {
        return true;
      }
    }
    if (!have_native_trail) return false;
    Eigen::VectorXd closure_lb = root_lb;
    Eigen::VectorXd closure_ub = root_ub;
    auto apply_lit = [&](const BranchDomainLiteral& lit) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      if (lit.is_lb) {
        if (lit.value > closure_ub[lit.var_idx] + eps) return false;
        if (lit.value > closure_lb[lit.var_idx]) {
          closure_lb[lit.var_idx] = lit.value;
        }
      } else {
        if (lit.value < closure_lb[lit.var_idx] - eps) return false;
        if (lit.value < closure_ub[lit.var_idx]) {
          closure_ub[lit.var_idx] = lit.value;
        }
      }
      return true;
    };
    auto active_lit = [&](const BranchDomainLiteral& lit) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      return lit.is_lb ? closure_lb[lit.var_idx] >= lit.value - eps
                       : closure_ub[lit.var_idx] <= lit.value + eps;
    };
    for (const auto& lit : reason_frontier) {
      if (lit.var_idx == target || !apply_lit(lit)) return false;
    }
    bool changed = true;
    for (int pass = 0;
         pass < static_cast<int>(local_domain_trail->size()) + 1 && changed;
         ++pass) {
      changed = false;
      for (const auto& entry : *local_domain_trail) {
        if (entry.pos < 0 || entry.pos >= target_pos ||
            entry.bound.var_idx == target || entry.reason.empty()) {
          continue;
        }
        bool reason_active = true;
        for (const auto& lit : entry.reason) {
          if (!active_lit(lit)) {
            reason_active = false;
            break;
          }
        }
        if (!reason_active) continue;
        const double old_lb = closure_lb[entry.bound.var_idx];
        const double old_ub = closure_ub[entry.bound.var_idx];
        if (!apply_lit(entry.bound)) return false;
        if (closure_lb[entry.bound.var_idx] > old_lb + eps ||
            closure_ub[entry.bound.var_idx] < old_ub - eps) {
          changed = true;
        }
      }
    }
    audited_activity = residual_root_activity;
    for (const auto& lit : needed_proof_frontier) {
      if (!active_lit(lit)) return false;
      double delta = 0.0;
      if (!proof_literal_delta(lit, &delta)) return false;
      audited_activity += delta;
    }
    return std::isfinite(audited_activity) &&
           audited_activity >= required_activity - proof_tol;
  };

  if (residual_root_activity >= required_activity - proof_tol) {
    BranchDomainLiteral proved_target_lit = target_lit;
    const double raw_bound = (rhs - residual_root_activity) / a0;
    if (std::isfinite(raw_bound)) {
      double strengthened = raw_bound;
      if (integer_target) {
        strengthened = target_lit.is_lb
            ? std::ceil(strengthened - 10.0 * int_tol)
            : std::floor(strengthened + 10.0 * int_tol);
      } else {
        strengthened += target_lit.is_lb ? -1e-7 : 1e-7;
      }
      if (target_lit.is_lb) {
        proved_target_lit.value =
            std::max(proved_target_lit.value, strengthened);
      } else {
        proved_target_lit.value =
            std::min(proved_target_lit.value, strengthened);
      }
    }
    BranchDomainLiteral flipped;
    const bool target_was_strengthened =
        std::abs(proved_target_lit.value - target_lit.value) >
        std::max(1e-9, int_tol);
    if (target_bound.has_source_conflict_literal && !target_was_strengthened) {
      flipped = target_bound.source_conflict_literal;
    } else if (!flipped_literal(proved_target_lit, flipped)) {
      return fail(DualProofResolutionStatus::ScopeBlocked);
    }
    std::vector<BranchDomainLiteral> clause{flipped};
    canonicalize_branch_literals(clause);
    if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
      return fail(clause.empty() ? DualProofResolutionStatus::ScopeBlocked
                                 : DualProofResolutionStatus::LiteralLimit);
    }
    out.valid = true;
    out.cutoff_conflict = false;
    out.has_proved_target_bound = true;
    out.proved_target_bound = proved_target_lit;
    out.flipped_target = flipped;
    out.proof_frontier.clear();
    out.resolved_frontier.clear();
    out.clause = std::move(clause);
    out.reconvergence_clauses.clear();
    out.proof_margin = residual_root_activity - required_activity;
    out.proof_budget = rhs - root_min_activity;
    out.proof_activity = 0.0;
    out.resolved_activity = residual_root_activity;
    return success();
  }

  struct TrailCandidate {
    BranchDomainLiteral proof_lit;
    std::vector<BranchDomainLiteral> reason;
    double delta{0.0};
    double base_bound{0.0};
    double priority{0.0};
    int trail_pos{-1};
    int prev_bound_pos{-1};
    bool from_reason_bound{false};
  };

  // HiGHS' explainBoundChangeLeq/Geq uses getColLower/UpperPos() at the
  // target position: each variable contributes the currently active local
  // proof-side bound, measured from the global/root bound.  Using every
  // incremental same-side trail entry would understate the proof activity and
  // produces certificates that cannot cross a frontier reliably.
  std::vector<TrailCandidate> candidates;
  candidates.reserve(static_cast<std::size_t>(coeff_sparse.nonZeros()));
  std::vector<int> candidate_by_col(static_cast<std::size_t>(n), -1);

  auto proof_side_matches = [&](const BranchDomainLiteral& lit) {
    if (lit.var_idx < 0 || lit.var_idx >= n) return false;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    return (a > 0.0 && lit.is_lb) || (a < 0.0 && !lit.is_lb);
  };
  auto proof_side_stronger = [&](const BranchDomainLiteral& lhs,
                                 const BranchDomainLiteral& rhs) {
    if (lhs.var_idx != rhs.var_idx || lhs.is_lb != rhs.is_lb) return false;
    return lhs.is_lb ? lhs.value > rhs.value + eps
                     : lhs.value < rhs.value - eps;
  };
  auto proof_side_same = [&](const BranchDomainLiteral& lhs,
                             const BranchDomainLiteral& rhs) {
    return lhs.var_idx == rhs.var_idx && lhs.is_lb == rhs.is_lb &&
           std::abs(lhs.value - rhs.value) <= eps;
  };
  auto add_or_replace_candidate =
      [&](BranchDomainLiteral lit, std::vector<BranchDomainLiteral> reason,
          int trail_pos, int prev_bound_pos, bool from_reason_bound) {
    const int j = lit.var_idx;
    if (j < 0 || j >= n || j == target || !std::isfinite(lit.value) ||
        trail_pos < 0 || trail_pos >= target_pos ||
        !proof_side_matches(lit)) {
      return;
    }
    const double a = coeff[static_cast<std::size_t>(j)];
    const double base = lit.is_lb ? root_lb[j] : root_ub[j];
    if (!std::isfinite(base)) return;
    double delta = 0.0;
    if (lit.is_lb) {
      if (lit.value <= base + eps) return;
      delta = a * (lit.value - base);
    } else {
      if (lit.value >= base - eps) return;
      delta = a * (lit.value - base);
    }
    if (!std::isfinite(delta) || delta <= proof_tol * 1e-4) return;
    if (reason.empty() && !from_reason_bound) reason.push_back(lit);
    canonicalize_branch_literals(reason);
    TrailCandidate cand{lit,
                        std::move(reason),
                        delta,
                        base,
                        std::max(literal_priority(lit), std::abs(delta)),
                        trail_pos,
                        prev_bound_pos,
                        from_reason_bound};
    int& slot = candidate_by_col[static_cast<std::size_t>(j)];
    if (slot < 0) {
      slot = static_cast<int>(candidates.size());
      candidates.push_back(std::move(cand));
      return;
    }
    TrailCandidate& current = candidates[static_cast<std::size_t>(slot)];
    if (proof_side_stronger(cand.proof_lit, current.proof_lit) ||
        (proof_side_same(cand.proof_lit, current.proof_lit) &&
         cand.trail_pos > current.trail_pos)) {
      current = std::move(cand);
    }
  };

  if (have_native_trail) {
    for (const auto& entry : *local_domain_trail) {
      add_or_replace_candidate(entry.bound, entry.reason, entry.pos,
                               entry.prev_bound_pos, !entry.is_branch);
    }
  } else {
    for (int pos = 0; pos < static_cast<int>(branch_reasons.size()); ++pos) {
      const auto& lit = branch_reasons[static_cast<std::size_t>(pos)];
      add_or_replace_candidate(lit, {lit}, pos, -1, false);
    }
    for (const auto& rb : reason_bounds) {
      add_or_replace_candidate(rb.bound, rb.reason, rb.trail_pos,
                               rb.prev_bound_pos, true);
    }
  }
  if (candidates.empty()) {
    return fail(DualProofResolutionStatus::MissingReason);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const TrailCandidate& a, const TrailCandidate& b) {
              if (a.priority != b.priority) return a.priority > b.priority;
              if (a.trail_pos != b.trail_pos) return a.trail_pos < b.trail_pos;
              if (a.delta != b.delta) return a.delta > b.delta;
              return a.reason.size() < b.reason.size();
            });

  double activity = residual_root_activity;
  std::vector<TrailCandidate> selected;
  selected.reserve(static_cast<std::size_t>(std::min(max_literals, 32)));
  std::vector<char> used_lower(static_cast<std::size_t>(n), 0);
  std::vector<char> used_upper(static_cast<std::size_t>(n), 0);
  for (const auto& cand : candidates) {
    const int j = cand.proof_lit.var_idx;
    char& used = cand.proof_lit.is_lb
        ? used_lower[static_cast<std::size_t>(j)]
        : used_upper[static_cast<std::size_t>(j)];
    if (used != 0) continue;
    used = 1;
    selected.push_back(cand);
    activity += cand.delta;
    if (activity >= required_activity - proof_tol) break;
  }
  if (activity < required_activity - proof_tol) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  for (int k = static_cast<int>(selected.size()) - 1; k >= 0; --k) {
    auto& cand = selected[static_cast<std::size_t>(k)];
    const int j = cand.proof_lit.var_idx;
    const double a = coeff[static_cast<std::size_t>(j)];
    const double without = activity - cand.delta;
    if (cand.proof_lit.is_lb) {
      if (a <= 0.0) continue;
      double relaxed = (required_activity - without) / a + cand.base_bound;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        relaxed = std::ceil(relaxed - eps);
      }
      if (!std::isfinite(relaxed) ||
          relaxed >= cand.proof_lit.value - proof_tol) {
        continue;
      }
      if (relaxed <= cand.base_bound + eps) {
        activity = without;
        selected.erase(selected.begin() + k);
      } else {
        if (have_native_trail) {
          int p = cand.prev_bound_pos;
          while (p >= 0 && p < static_cast<int>(local_domain_trail->size()) &&
                 relaxed <= (*local_domain_trail)[static_cast<std::size_t>(p)]
                                    .bound.value +
                                eps) {
            cand.trail_pos = p;
            cand.prev_bound_pos =
                (*local_domain_trail)[static_cast<std::size_t>(p)]
                    .prev_bound_pos;
            p = cand.prev_bound_pos;
          }
        }
        activity += a * (relaxed - cand.proof_lit.value);
        cand.proof_lit.value = relaxed;
        cand.delta = a * (cand.proof_lit.value - cand.base_bound);
        if (!cand.from_reason_bound) cand.reason = {cand.proof_lit};
      }
    } else {
      if (a >= 0.0) continue;
      double relaxed = (required_activity - without) / a + cand.base_bound;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        relaxed = std::floor(relaxed + eps);
      }
      if (!std::isfinite(relaxed) ||
          relaxed <= cand.proof_lit.value + proof_tol) {
        continue;
      }
      if (relaxed >= cand.base_bound - eps) {
        activity = without;
        selected.erase(selected.begin() + k);
      } else {
        if (have_native_trail) {
          int p = cand.prev_bound_pos;
          while (p >= 0 && p < static_cast<int>(local_domain_trail->size()) &&
                 relaxed >= (*local_domain_trail)[static_cast<std::size_t>(p)]
                                    .bound.value -
                                eps) {
            cand.trail_pos = p;
            cand.prev_bound_pos =
                (*local_domain_trail)[static_cast<std::size_t>(p)]
                    .prev_bound_pos;
            p = cand.prev_bound_pos;
          }
        }
        activity += a * (relaxed - cand.proof_lit.value);
        cand.proof_lit.value = relaxed;
        cand.delta = a * (cand.proof_lit.value - cand.base_bound);
        if (!cand.from_reason_bound) cand.reason = {cand.proof_lit};
      }
    }
    if (activity <= required_activity + proof_tol) break;
  }
  if (selected.empty() || activity < required_activity - proof_tol) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  std::vector<BranchDomainLiteral> proof_frontier;
  proof_frontier.reserve(selected.size());
  for (const auto& cand : selected) proof_frontier.push_back(cand.proof_lit);
  canonicalize_branch_literals(proof_frontier);
  if (proof_frontier.empty()) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }
  double proof_frontier_activity = 0.0;
  if (!audited_activity_from_frontier(proof_frontier,
                                      proof_frontier_activity) ||
      proof_frontier_activity < required_activity - proof_tol) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  auto find_prior_entry =
      [&](const BranchDomainLiteral& lit,
          int before_pos) -> const LocalDomainTrailEntry* {
    if (!have_native_trail || lit.var_idx < 0 || lit.var_idx >= n ||
        before_pos <= 0) {
      return nullptr;
    }
    const LocalDomainTrailEntry* best = nullptr;
    for (const auto& entry : *local_domain_trail) {
      if (entry.pos < 0 || entry.pos >= before_pos ||
          entry.bound.var_idx != lit.var_idx ||
          entry.bound.is_lb != lit.is_lb) {
        continue;
      }
      if (!branch_literal_stronger_or_equal(entry.bound, lit, eps)) continue;
      if (best == nullptr || entry.pos > best->pos) best = &entry;
    }
    return best;
  };

  auto fallback_literal_pos = [&](const BranchDomainLiteral& lit,
                                  int before_pos) -> int {
    int best = -1;
    if (have_native_trail) {
      for (const auto& entry : *local_domain_trail) {
        if (entry.pos < 0 || entry.pos >= before_pos ||
            entry.bound.var_idx != lit.var_idx ||
            entry.bound.is_lb != lit.is_lb) {
          continue;
        }
        if (branch_literal_stronger_or_equal(entry.bound, lit, eps)) {
          best = std::max(best, entry.pos);
        }
      }
    } else {
      for (int i = 0; i < static_cast<int>(branch_reasons.size()); ++i) {
        if (i >= before_pos) continue;
        if (branch_literal_stronger_or_equal(
                branch_reasons[static_cast<std::size_t>(i)], lit, eps)) {
          best = std::max(best, i);
        }
      }
      for (const auto& rb : reason_bounds) {
        if (rb.trail_pos >= 0 && rb.trail_pos < before_pos &&
            branch_literal_stronger_or_equal(rb.bound, lit, eps)) {
          best = std::max(best, rb.trail_pos);
        }
      }
    }
    return best;
  };

  struct ReasonFrontierEntry {
    BranchDomainLiteral lit;
    std::vector<BranchDomainLiteral> reason;
    int trail_pos{-1};
    bool is_branch{false};
  };

  auto compact_reason_frontier =
      [&](std::vector<ReasonFrontierEntry>& frontier) {
    std::vector<ReasonFrontierEntry> compact;
    compact.reserve(frontier.size());
    for (auto& entry : frontier) {
      if (entry.lit.var_idx < 0 || entry.lit.var_idx >= n ||
          !std::isfinite(entry.lit.value)) {
        continue;
      }
      if (entry.trail_pos >= 0) {
        bool merged_by_pos = false;
        for (auto& cur : compact) {
          if (cur.trail_pos != entry.trail_pos) continue;
          if (cur.lit.var_idx == entry.lit.var_idx &&
              cur.lit.is_lb == entry.lit.is_lb) {
            if (entry.lit.is_lb) {
              cur.lit.value = std::max(cur.lit.value, entry.lit.value);
            } else {
              cur.lit.value = std::min(cur.lit.value, entry.lit.value);
            }
          }
          merged_by_pos = true;
          break;
        }
        if (!merged_by_pos) compact.push_back(std::move(entry));
        continue;
      }
      bool merged_terminal = false;
      for (auto& cur : compact) {
        if (cur.trail_pos >= 0 || cur.lit.var_idx != entry.lit.var_idx ||
            cur.lit.is_lb != entry.lit.is_lb) {
          continue;
        }
        const bool replace =
            proof_side_stronger(entry.lit, cur.lit) ||
            (proof_side_same(entry.lit, cur.lit) &&
             entry.trail_pos > cur.trail_pos);
        if (replace) cur = std::move(entry);
        merged_terminal = true;
        break;
      }
      if (!merged_terminal) compact.push_back(std::move(entry));
    }
    frontier.swap(compact);
    std::sort(frontier.begin(), frontier.end(),
              [](const ReasonFrontierEntry& a,
                 const ReasonFrontierEntry& b) {
                if (a.trail_pos != b.trail_pos) {
                  if (a.trail_pos < 0) return false;
                  if (b.trail_pos < 0) return true;
                  return a.trail_pos < b.trail_pos;
                }
                if (a.lit.var_idx != b.lit.var_idx) {
                  return a.lit.var_idx < b.lit.var_idx;
                }
                if (a.lit.is_lb != b.lit.is_lb) return a.lit.is_lb < b.lit.is_lb;
                return a.lit.value < b.lit.value;
              });
  };

  auto reason_frontier_literals =
      [&](const std::vector<ReasonFrontierEntry>& frontier) {
    std::vector<BranchDomainLiteral> lits;
    lits.reserve(frontier.size());
    for (const auto& entry : frontier) lits.push_back(entry.lit);
    canonicalize_branch_literals(lits);
    return lits;
  };

  auto reason_entry_for_literal =
      [&](const BranchDomainLiteral& lit, int before_pos) {
    const LocalDomainTrailEntry* prior = find_prior_entry(lit, before_pos);
    if (prior != nullptr) {
      return ReasonFrontierEntry{
          lit,
          prior->reason.empty() && prior->is_branch
              ? std::vector<BranchDomainLiteral>{lit}
              : prior->reason,
          prior->pos,
          prior->is_branch};
    }
    return ReasonFrontierEntry{lit, {lit},
                               fallback_literal_pos(lit, before_pos), true};
  };

  auto flip_literal_for_conflict =
      [&](const BranchDomainLiteral& lit, BranchDomainLiteral& flipped) -> bool {
    if (lit.var_idx < 0 || lit.var_idx >= n || !std::isfinite(lit.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
    if (lit.is_lb) {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::ceil(lit.value - eps) - 1.0
                      : lit.value - std::max(1e-7, 10.0 * eps),
          false};
    } else {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::floor(lit.value + eps) + 1.0
                      : lit.value + std::max(1e-7, 10.0 * eps),
          true};
    }
    return true;
  };

  auto source_conflict_matches_flipped =
      [&](const BranchDomainLiteral& conflict_lit,
          const BranchDomainLiteral& flipped) {
    if (conflict_lit.var_idx != flipped.var_idx ||
        conflict_lit.is_lb != flipped.is_lb) {
      return false;
    }
    return conflict_lit.is_lb
               ? conflict_lit.value <= flipped.value + eps
               : conflict_lit.value >= flipped.value - eps;
  };

  auto explain_conflict_reason_frontier =
      [&](const LocalDomainTrailEntry& entry,
          std::vector<ReasonFrontierEntry>& replacement) -> bool {
    replacement.clear();
    if (!have_native_trail || !entry.has_source_conflict_clause ||
        entry.source_conflict_clause.empty() || entry.pos <= 0) {
      return false;
    }
    BranchDomainLiteral flipped_entry;
    if (!flip_literal_for_conflict(entry.bound, flipped_entry)) {
      return false;
    }
    bool found_flipped = false;
    for (const auto& conflict_lit : entry.source_conflict_clause) {
      if (!found_flipped &&
          source_conflict_matches_flipped(conflict_lit, flipped_entry)) {
        found_flipped = true;
        continue;
      }
      if (conflict_lit.var_idx < 0 || conflict_lit.var_idx >= n ||
          !std::isfinite(conflict_lit.value)) {
        return false;
      }
      const double global_bound =
          conflict_lit.is_lb ? root_lb[conflict_lit.var_idx]
                             : root_ub[conflict_lit.var_idx];
      if (std::isfinite(global_bound)) {
        const bool globally_active =
            conflict_lit.is_lb
                ? global_bound >= conflict_lit.value - eps
                : global_bound <= conflict_lit.value + eps;
        if (globally_active) continue;
      }
      const LocalDomainTrailEntry* prior =
          find_prior_entry(conflict_lit, entry.pos);
      if (prior == nullptr) return false;
      LocalDomainTrailEntry relaxed = *prior;
      while (relaxed.prev_bound_pos >= 0 &&
             relaxed.prev_bound_pos <
                 static_cast<int>(local_domain_trail->size())) {
        const auto& prev =
            (*local_domain_trail)[static_cast<std::size_t>(
                relaxed.prev_bound_pos)];
        const bool prev_still_active =
            conflict_lit.is_lb
                ? prev.bound.value >= conflict_lit.value - eps
                : prev.bound.value <= conflict_lit.value + eps;
        if (!prev_still_active) break;
        relaxed = prev;
      }
      replacement.push_back(ReasonFrontierEntry{
          conflict_lit,
          relaxed.reason.empty() && relaxed.is_branch
              ? std::vector<BranchDomainLiteral>{conflict_lit}
              : relaxed.reason,
          relaxed.pos,
          relaxed.is_branch});
    }
    return found_flipped;
  };

  auto reason_entry_for_candidate =
      [&](const TrailCandidate& cand) -> ReasonFrontierEntry {
    if (have_native_trail && cand.trail_pos >= 0 &&
        cand.trail_pos < static_cast<int>(local_domain_trail->size())) {
      const auto& entry =
          (*local_domain_trail)[static_cast<std::size_t>(cand.trail_pos)];
      return ReasonFrontierEntry{
          cand.proof_lit,
          entry.reason.empty() && entry.is_branch
              ? std::vector<BranchDomainLiteral>{cand.proof_lit}
              : entry.reason,
          cand.trail_pos,
          entry.is_branch};
    }
    return ReasonFrontierEntry{
        cand.proof_lit,
        cand.reason.empty() ? std::vector<BranchDomainLiteral>{cand.proof_lit}
                            : cand.reason,
        cand.trail_pos,
        !cand.from_reason_bound};
  };

  auto resolve_local_reason_frontier =
      [&](std::vector<ReasonFrontierEntry> frontier,
          int stop_size,
          int min_resolve) {
    if (!have_native_trail) return reason_frontier_literals(frontier);
    compact_reason_frontier(frontier);
    int depth = local_branch_positions == nullptr
                    ? 0
                    : static_cast<int>(local_branch_positions->size());
    while (depth > 0) {
      const int branch_pos =
          (*local_branch_positions)[static_cast<std::size_t>(depth - 1)];
      if (branch_pos < 0 ||
          branch_pos >= static_cast<int>(local_domain_trail->size())) {
        break;
      }
      const auto& branch_entry =
          (*local_domain_trail)[static_cast<std::size_t>(branch_pos)];
      if (std::abs(branch_entry.bound.value -
                   branch_entry.prev_bound_value) > eps) {
        break;
      }
      --depth;
    }
    const int start_pos =
        depth <= 0 || local_branch_positions == nullptr ||
                local_branch_positions->empty()
            ? 0
            : (*local_branch_positions)[static_cast<std::size_t>(depth - 1)] + 1;
    const int max_resolves = std::min<int>(
        4096, static_cast<int>(local_domain_trail->size()) +
                  4 * std::max(1, max_literals));
	    std::unordered_set<int> skipped_positions;
	    struct ResolutionState {
	      std::vector<BranchDomainLiteral> literals;
	      std::vector<int> trail_positions;
	      std::vector<int> skipped_positions;
	    };
	    auto state_hash = [](const ResolutionState& state) {
	      std::size_t hash = conflict_clause_hash(state.literals);
	      for (int pos : state.trail_positions) {
	        hash ^= std::hash<int>{}(pos) + 0x9e3779b97f4a7c15ULL +
	                (hash << 6) + (hash >> 2);
	      }
	      for (int pos : state.skipped_positions) {
	        hash ^= std::hash<int>{}(pos) + 0x9e3779b97f4a7c15ULL +
	                (hash << 6) + (hash >> 2);
	      }
	      return hash;
	    };
	    auto states_equal = [](const ResolutionState& lhs,
	                           const ResolutionState& rhs) {
	      if (!branch_literal_lists_equal(lhs.literals, rhs.literals, 0.0) ||
	          lhs.trail_positions != rhs.trail_positions ||
	          lhs.skipped_positions != rhs.skipped_positions) {
	        return false;
	      }
	      return true;
	    };
	    HashBucketExactSet<ResolutionState, decltype(state_hash),
	                       decltype(states_equal)>
	        seen(state_hash, states_equal);
	    auto current_state = [&]() {
	      ResolutionState state;
	      state.literals.reserve(frontier.size());
	      state.trail_positions.reserve(frontier.size());
	      for (const auto& entry : frontier) {
	        state.literals.push_back(entry.lit);
	        state.trail_positions.push_back(entry.trail_pos);
	      }
	      state.skipped_positions.assign(skipped_positions.begin(),
	                                     skipped_positions.end());
	      std::sort(state.skipped_positions.begin(),
	                state.skipped_positions.end());
	      return state;
	    };
	    int resolved = 0;
	    for (int iter = 0; iter < max_resolves; ++iter) {
	      if (!seen.insert(current_state())) break;

	      int latest_idx = -1;
      int latest_pos = -1;
      int resolvable = 0;
      for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
        const int pos = frontier[static_cast<std::size_t>(i)].trail_pos;
	        if (pos < start_pos || pos < 0 ||
	            pos >= static_cast<int>(local_domain_trail->size())) {
	          continue;
	        }
	        if (skipped_positions.count(pos) != 0) continue;
	        const auto& entry =
	            (*local_domain_trail)[static_cast<std::size_t>(pos)];
	        if (entry.is_branch || entry.reason.empty()) continue;
        ++resolvable;
        if (pos > latest_pos) {
          latest_pos = pos;
          latest_idx = i;
        }
      }
	      if (latest_idx < 0 ||
	          (resolvable <= stop_size && resolved >= min_resolve)) {
	        break;
	      }
	      const auto& entry =
	          (*local_domain_trail)[static_cast<std::size_t>(latest_pos)];
	      auto covered_by_current_frontier =
	          [&](const BranchDomainLiteral& lit) {
	        for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
	          if (i == latest_idx) continue;
	          const auto& cur = frontier[static_cast<std::size_t>(i)].lit;
	          if (cur.var_idx != lit.var_idx || cur.is_lb != lit.is_lb) {
	            continue;
	          }
	          if (branch_literal_stronger_or_equal(cur, lit, eps)) {
	            return true;
	          }
	        }
	        return false;
	      };
	      std::vector<ReasonFrontierEntry> replacement;
	      bool replacement_valid =
	          explain_conflict_reason_frontier(entry, replacement);
	      if (replacement_valid) {
	        replacement.erase(
	            std::remove_if(
	                replacement.begin(), replacement.end(),
	                [&](const ReasonFrontierEntry& repl) {
	                  return covered_by_current_frontier(repl.lit);
	                }),
	            replacement.end());
	      } else {
	        replacement.clear();
	        replacement.reserve(entry.reason.size());
	        replacement_valid = true;
	        for (const auto& reason_lit : entry.reason) {
	          if (reason_lit.var_idx < 0 || reason_lit.var_idx >= n ||
	              !std::isfinite(reason_lit.value)) {
	            replacement_valid = false;
	            break;
	          }
	          if (covered_by_current_frontier(reason_lit)) continue;
	          replacement.push_back(
	              reason_entry_for_literal(reason_lit, latest_pos));
	        }
	      }
	      if (!replacement_valid) {
	        skipped_positions.insert(latest_pos);
	        continue;
	      }
	      frontier.erase(frontier.begin() + latest_idx);
	      if (!replacement.empty()) {
	        frontier.insert(frontier.end(),
	                        std::make_move_iterator(replacement.begin()),
	                        std::make_move_iterator(replacement.end()));
	      }
	      compact_reason_frontier(frontier);
	      ++resolved;
	    }
    return reason_frontier_literals(frontier);
  };

  std::vector<ReasonFrontierEntry> initial_frontier;
  initial_frontier.reserve(selected.size());
  for (const auto& cand : selected) {
    if (cand.proof_lit.var_idx == target) {
      return fail(DualProofResolutionStatus::ScopeBlocked);
    }
    initial_frontier.push_back(reason_entry_for_candidate(cand));
  }
  std::vector<BranchDomainLiteral> resolved_frontier =
      resolve_local_reason_frontier(std::move(initial_frontier),
                                    /*stop_size=*/0,
                                    /*min_resolve=*/0);
  canonicalize_branch_literals(resolved_frontier);
  if (static_cast<int>(resolved_frontier.size()) + 1 > max_literals) {
    return fail(DualProofResolutionStatus::LiteralLimit);
  }
  if (reason_touches_target(resolved_frontier)) {
    return fail(DualProofResolutionStatus::ScopeBlocked);
  }
  double resolved_frontier_activity = 0.0;
  if (!frontier_implies_proof_frontier(resolved_frontier, proof_frontier,
                                       resolved_frontier_activity)) {
    return fail(have_native_trail ? DualProofResolutionStatus::ScopeBlocked
                                  : DualProofResolutionStatus::MissingReason);
  }

  BranchDomainLiteral proved_target_lit = target_lit;
  const double raw_bound = (rhs - resolved_frontier_activity) / a0;
  if (std::isfinite(raw_bound)) {
    double strengthened = raw_bound;
    if (integer_target) {
      strengthened = target_lit.is_lb
          ? std::ceil(strengthened - 10.0 * int_tol)
          : std::floor(strengthened + 10.0 * int_tol);
    } else {
      strengthened += target_lit.is_lb ? -1e-7 : 1e-7;
    }
    if (target_lit.is_lb) {
      proved_target_lit.value = std::max(proved_target_lit.value, strengthened);
    } else {
      proved_target_lit.value = std::min(proved_target_lit.value, strengthened);
    }
  }

  BranchDomainLiteral flipped;
  const bool target_was_strengthened =
      std::abs(proved_target_lit.value - target_lit.value) >
      std::max(1e-9, int_tol);
  if (target_bound.has_source_conflict_literal && !target_was_strengthened) {
    flipped = target_bound.source_conflict_literal;
  } else if (!flipped_literal(proved_target_lit, flipped)) {
    return fail(DualProofResolutionStatus::ScopeBlocked);
  }

  const double selected_delta = activity - residual_root_activity;
  const double proof_budget = rhs - root_min_activity;
  const bool cutoff_conflict =
      selected_delta >
      proof_budget + std::max(1e-7, 1e-10 * (1.0 + std::abs(proof_budget)));

  std::vector<BranchDomainLiteral> clause = resolved_frontier;
  if (!cutoff_conflict) clause.push_back(flipped);
  canonicalize_branch_literals(clause);
  if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
    return fail(clause.empty() ? DualProofResolutionStatus::ScopeBlocked
                               : DualProofResolutionStatus::LiteralLimit);
  }

  std::vector<std::vector<BranchDomainLiteral>> reconvergence_clauses;
  if (!cutoff_conflict) {
    std::vector<BranchDomainLiteral> reconvergence = resolved_frontier;
    reconvergence.push_back(flipped);
    canonicalize_branch_literals(reconvergence);
    if (!reconvergence.empty() &&
        static_cast<int>(reconvergence.size()) <= max_literals) {
      reconvergence_clauses.push_back(std::move(reconvergence));
    }
  }
  for (const auto& cand : selected) {
    if (!cand.from_reason_bound) continue;
    BranchDomainLiteral flipped_selected;
    if (!flipped_literal(cand.proof_lit, flipped_selected)) continue;
    std::vector<ReasonFrontierEntry> reason_frontier;
    reason_frontier.reserve(cand.reason.size());
    const ReasonFrontierEntry cand_entry = reason_entry_for_candidate(cand);
    bool reason_valid = true;
    const std::vector<BranchDomainLiteral>& cand_reason =
        cand_entry.reason.empty() ? cand.reason : cand_entry.reason;
    for (const auto& reason_lit : cand_reason) {
      if (reason_lit.var_idx < 0 || reason_lit.var_idx >= n ||
          !std::isfinite(reason_lit.value)) {
        reason_valid = false;
        break;
      }
      reason_frontier.push_back(
          reason_entry_for_literal(reason_lit, cand.trail_pos));
    }
    if (!reason_valid) continue;
    std::vector<BranchDomainLiteral> reconv =
        reason_frontier.empty()
            ? std::vector<BranchDomainLiteral>()
            : resolve_local_reason_frontier(std::move(reason_frontier),
                                            /*stop_size=*/0,
                                            /*min_resolve=*/0);
    double reconv_activity = 0.0;
    if (!frontier_implies_proof_frontier(reconv, proof_frontier,
                                         reconv_activity)) {
      continue;
    }
    reconv.push_back(flipped_selected);
    canonicalize_branch_literals(reconv);
    if (reconv.empty() ||
        static_cast<int>(reconv.size()) > max_literals) {
      continue;
    }
    bool duplicate = false;
    const std::size_t h = conflict_clause_hash(reconv);
    for (const auto& existing : reconvergence_clauses) {
      if (conflict_clause_hash(existing) == h &&
          branch_literal_lists_equal(existing, reconv)) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) reconvergence_clauses.push_back(std::move(reconv));
  }

  out.valid = true;
  out.cutoff_conflict = cutoff_conflict;
  out.has_proved_target_bound = true;
  out.proved_target_bound = proved_target_lit;
  out.flipped_target = flipped;
  out.proof_frontier = std::move(proof_frontier);
  out.resolved_frontier = std::move(resolved_frontier);
  out.clause = std::move(clause);
  out.reconvergence_clauses = std::move(reconvergence_clauses);
  out.proof_margin = resolved_frontier_activity - required_activity;
  out.proof_budget = proof_budget;
  out.proof_activity = proof_frontier_activity;
  out.resolved_activity = resolved_frontier_activity;
  return success();
}


}  // namespace mipsolvers::engine::detail
