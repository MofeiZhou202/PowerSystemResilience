/// @file bc_relaxation_crash_basis.cpp
/// @brief Primal-activity crash basis recovery for the B&C LP relaxation.
///
/// Extracted from bc_relaxation.cpp; shares helpers via bc_relaxation_internal.hpp.

#include "mipsolvers/engine/detail/bc_relaxation_internal.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"

namespace mipsolvers::engine::detail {

CrashBasisRecoveryStats recover_primal_activity_basis_impl(const StandardFormLP& sf,
                                                           const Eigen::VectorXd& x_orig,
                                                           const SolveResult* ipm_res,
                                                           const SimplexBasis* seed_basis,
                                                           SimplexBasis& out_basis) {
  const int sf_m = static_cast<int>(sf.A.rows());
  const int sf_n = static_cast<int>(sf.A.cols());
  Eigen::VectorXd x_sf = primal_to_scaled_standard_form(sf, x_orig);

  CrashBasisRecoveryStats stats;
  out_basis.rows = sf_m;
  out_basis.cols = sf_n;
  out_basis.indices.resize(sf_m);
  out_basis.at_upper.assign(sf_n, 0);
  out_basis.cached_sparse_basis.reset();
  out_basis.cached_reduced_costs.reset();
  out_basis.persist_eta_count = 0;

  std::vector<double> partition_score(static_cast<size_t>(sf.n_original), 0.0);
  std::vector<char> col_interior(static_cast<size_t>(sf.n_original), 0);
  std::vector<double> col_max_abs(static_cast<size_t>(sf.n_original), 0.0);
  std::vector<int> col_nnz(static_cast<size_t>(sf.n_original), 0);
  std::vector<double> row_dual_score(static_cast<size_t>(sf_m), 1.0);
  const double crash_rel_tol = 1e-5;
  const bool have_col_scale = (sf.col_scale.size() == sf_n);
  const bool have_row_scale = (sf.row_scale.size() == sf_m);

  Eigen::VectorXd s_sf = Eigen::VectorXd::Zero(sf_n);
  bool have_rc = false;
  bool have_box = false;
  if (ipm_res != nullptr) {
    if (ipm_res->constraint_duals.size() > 0) {
      Eigen::VectorXd y_sf = Eigen::VectorXd::Zero(sf_m);
      for (int i = 0; i < sf_m && i < static_cast<int>(ipm_res->constraint_duals.size()); ++i) {
        // Match populate_dual_certificate's public simplex convention:
        // row_dual = -row_sign * row_scale * y_scaled.
        double y_orig = -sf.row_sign[i] * ipm_res->constraint_duals[i];
        y_sf[i] = have_row_scale ? (y_orig / sf.row_scale[i]) : y_orig;
        row_dual_score[static_cast<size_t>(i)] = std::abs(y_sf[i]) / (1.0 + std::abs(y_sf[i]));
      }
      s_sf = sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_sf);
      have_rc = true;
    }
    have_box = (ipm_res->box_dual_lb.size() >= sf.n_original &&
                ipm_res->box_dual_ub.size() >= sf.n_original);
  }

  for (int j = 0; j < sf.n_original; ++j) {
    for (StandardColumnMatrix::InnerIterator it(sf.A, j); it; ++it) {
      col_max_abs[static_cast<size_t>(j)] =
          std::max(col_max_abs[static_cast<size_t>(j)], std::abs(it.value()));
      ++col_nnz[static_cast<size_t>(j)];
    }
  }

  for (int j = 0; j < sf_n; ++j) {
    double val = x_sf[j];
    double ub_j = sf.var_ub[j];
    double range = std::isfinite(ub_j) ? ub_j : std::max(1.0, std::abs(val));
    double rel_from_lb = val / std::max(range, 1e-10);
    double rel_from_ub = std::isfinite(ub_j)
        ? (ub_j - val) / std::max(range, 1e-10) : 1.0;
    if (rel_from_ub < crash_rel_tol && std::isfinite(ub_j)) {
      out_basis.at_upper[static_cast<size_t>(j)] = 1;
    }
    if (j < sf.n_original) {
      const bool interior =
          rel_from_lb >= crash_rel_tol && rel_from_ub >= crash_rel_tol;
      col_interior[static_cast<size_t>(j)] = interior ? 1 : 0;
      if (have_rc) {
        // A column can be basic at a degenerate bound, but its reduced cost
        // still has to be zero. This is the direct dual-feasibility signal;
        // primal distance and bound multipliers are only fallbacks when row
        // duals are unavailable.
        const double dual_zero_tol =
            1e-7 * std::max(1.0, std::abs(sf.c_max[j]));
        const double dual_score = dual_zero_tol /
            (dual_zero_tol + std::abs(s_sf[j]));
        // Interior columns must survive every per-row edge truncation: forcing
        // one nonbasic would move it to a bound and destroy the IPM primal
        // point. Degenerate bound columns are rank-completion candidates only.
        partition_score[static_cast<size_t>(j)] =
            interior ? 2.0 + dual_score : dual_score;
      } else if (have_box) {
        const double col_sc = have_col_scale ? sf.col_scale[j] : 1.0;
        const double zl_j = ipm_res->box_dual_lb(j) * col_sc;
        const double zu_j = ipm_res->box_dual_ub(j) * col_sc;
        const double gap_lb = val;
        const double gap_ub = std::isfinite(ub_j) ? (ub_j - val) : 1.0;
        if (interior) {
          const double pi_lb = gap_lb /
              std::sqrt(gap_lb * gap_lb + zl_j * zl_j + 1e-20);
          const double pi_ub = gap_ub /
              std::sqrt(gap_ub * gap_ub + zu_j * zu_j + 1e-20);
          partition_score[static_cast<size_t>(j)] = pi_lb * pi_ub;
        } else {
          // Degenerate vertex variables can be both basic and exactly at a
          // bound. Distance-only scoring assigned all of them score zero and
          // made a useful SCUC basis impossible to identify. A small active
          // bound multiplier is the correct optimal-partition signal here.
          const double active_dual = rel_from_lb < crash_rel_tol
              ? std::abs(zl_j) : std::abs(zu_j);
          const double dual_zero_tol =
              1e-7 * std::max(1.0, std::abs(sf.c_max[j]));
          partition_score[static_cast<size_t>(j)] =
              dual_zero_tol / (dual_zero_tol + active_dual);
        }
      } else {
        partition_score[static_cast<size_t>(j)] = interior
            ? std::min(rel_from_lb, rel_from_ub) : 0.0;
      }
    }
  }

  const bool seeded =
      (seed_basis != nullptr && seed_basis->rows == sf_m && seed_basis->cols == sf_n &&
       static_cast<int>(seed_basis->index_count()) == sf_m);
  if (seeded) {
    out_basis.indices = seed_basis->basis_indices();
    if (seed_basis->at_upper.size() == static_cast<size_t>(sf_n)) {
      for (int j = sf.n_original; j < sf_n; ++j) {
        out_basis.at_upper[static_cast<size_t>(j)] =
            seed_basis->at_upper[static_cast<size_t>(j)];
      }
    }
  } else {
    for (int i = 0; i < sf_m; ++i) {
      if (sf.row_to_slack_col[i] >= 0) {
        out_basis.indices[static_cast<size_t>(i)] = sf.row_to_slack_col[i];
      } else if (sf.row_to_artificial_col[i] >= 0) {
        out_basis.indices[static_cast<size_t>(i)] = sf.row_to_artificial_col[i];
      } else {
        out_basis.indices[static_cast<size_t>(i)] = -1;
      }
    }
  }

  struct RowCandidate {
    double score;
    double partition;
    double row_dual;
    double activity;
    double coefficient_quality;
    double sparsity;
    int row;
    int col;
  };

  // Compact sparse adjacency. Keeping a bounded number of strong edges per
  // row prevents a high-degree network row from dominating recovery work.
  constexpr int kMaxCandidatesPerRow = 12;
  std::vector<RowCandidate> candidates;
  candidates.reserve(static_cast<size_t>(sf_m) * 4);
  std::vector<char> col_used(static_cast<size_t>(sf_n), 0);
  for (int i = 0; i < sf_m; ++i) {
    const int idx = out_basis.indices[static_cast<size_t>(i)];
    if (idx >= 0 && idx < sf_n) col_used[static_cast<size_t>(idx)] = 1;
  }

  const double min_partition_for_swap = (have_box || have_rc) ? 0.05 : 1e-6;
  const double min_row_dual_for_swap = (ipm_res != nullptr && ipm_res->constraint_duals.size() > 0)
      ? 1e-2 : 0.0;
  for (int row = 0; row < sf_m; ++row) {
    const int current_basis_col = out_basis.indices[static_cast<size_t>(row)];
    const bool row_is_seed_slack = (current_basis_col == sf.row_to_slack_col[row] ||
                                    current_basis_col == sf.row_to_artificial_col[row] ||
                                    current_basis_col < 0);
    if (seeded && !row_is_seed_slack) continue;

    const int slack_col = sf.row_to_slack_col[row];
    const int art_col = sf.row_to_artificial_col[row];
    const double row_score = row_dual_score[static_cast<size_t>(row)];
    const bool tight_row = (slack_col >= 0)
        ? x_sf[slack_col] <= 1e-6 * std::max(1.0, std::abs(sf.b[row]))
        : (art_col >= 0);
    if (!tight_row && row_score < min_row_dual_for_swap) continue;

    std::array<RowCandidate, kMaxCandidatesPerRow> best;
    for (auto& item : best) {
      item = {-std::numeric_limits<double>::infinity(), 0.0, 0.0,
              0.0, 0.0, 0.0, row, -1};
    }
    int row_candidates = 0;
    for (StandardRowMatrix::InnerIterator it(sf.A_row, row); it; ++it) {
      const int col = it.col();
      if (col >= sf.n_original || col_used[static_cast<size_t>(col)]) continue;
      const double part_score = partition_score[static_cast<size_t>(col)];
      if (part_score < min_partition_for_swap) continue;
      const double coeff = std::abs(it.value());
      const double normalized_coeff = coeff /
          std::max(col_max_abs[static_cast<size_t>(col)], 1e-30);
      const double activity = coeff * std::abs(x_sf[col]);
      const double sparsity = 1.0 /
          std::sqrt(static_cast<double>(std::max(1, col_nnz[static_cast<size_t>(col)])));
      const double score = part_score * (0.25 + row_score) *
                           (0.25 + normalized_coeff) * sparsity;
      RowCandidate cand{score, part_score, row_score, activity,
                        normalized_coeff, sparsity, row, col};
      ++row_candidates;
      for (int pos = 0; pos < kMaxCandidatesPerRow; ++pos) {
        if (cand.score > best[static_cast<size_t>(pos)].score) {
          for (int shift = kMaxCandidatesPerRow - 1; shift > pos; --shift) {
            best[static_cast<size_t>(shift)] = best[static_cast<size_t>(shift - 1)];
          }
          best[static_cast<size_t>(pos)] = cand;
          break;
        }
      }
    }
    if (row_candidates == 0) continue;
    ++stats.candidate_rows;
    for (const auto& cand : best) {
      if (cand.col >= 0) candidates.push_back(cand);
    }
  }
  stats.candidate_edges = static_cast<int>(candidates.size());

  const std::vector<int> base_basis = out_basis.indices;
  struct ColumnCandidate {
    int col;
    bool interior;
    double score;
    double partition;
    double row_dual;
    double activity;
    double coefficient_quality;
    double sparsity;
  };
  std::vector<ColumnCandidate> ordered_columns;
  ordered_columns.reserve(candidates.size());
  std::vector<int> candidate_slot(static_cast<size_t>(sf.n_original), -1);
  for (const RowCandidate& edge : candidates) {
    int& slot = candidate_slot[static_cast<size_t>(edge.col)];
    if (slot < 0) {
      slot = static_cast<int>(ordered_columns.size());
      ordered_columns.push_back(
          {edge.col, col_interior[static_cast<size_t>(edge.col)] != 0,
           edge.score, edge.partition, edge.row_dual, edge.activity,
           edge.coefficient_quality, edge.sparsity});
      continue;
    }
    ColumnCandidate& column = ordered_columns[static_cast<size_t>(slot)];
    column.coefficient_quality =
        std::max(column.coefficient_quality, edge.coefficient_quality);
    if (edge.score > column.score) {
      column.score = edge.score;
      column.partition = edge.partition;
      column.row_dual = edge.row_dual;
      column.activity = edge.activity;
    }
  }
  std::stable_sort(ordered_columns.begin(), ordered_columns.end(),
                   [](const ColumnCandidate& lhs, const ColumnCandidate& rhs) {
    if (lhs.interior != rhs.interior) return lhs.interior > rhs.interior;
    if (lhs.partition != rhs.partition) return lhs.partition > rhs.partition;
    if (lhs.coefficient_quality != rhs.coefficient_quality) {
      return lhs.coefficient_quality > rhs.coefficient_quality;
    }
    if (lhs.sparsity != rhs.sparsity) return lhs.sparsity > rhs.sparsity;
    if (lhs.score != rhs.score) return lhs.score > rhs.score;
    return lhs.col < rhs.col;
  });

  std::vector<char> replaceable(static_cast<size_t>(sf_m), 0);
  int remaining_replaceable = 0;
  for (int position = 0; position < sf_m; ++position) {
    const int basic_col = base_basis[static_cast<size_t>(position)];
    const bool logical =
        basic_col == sf.row_to_slack_col[static_cast<size_t>(position)] ||
        basic_col == sf.row_to_artificial_col[static_cast<size_t>(position)];
    replaceable[static_cast<size_t>(position)] = logical ? 1 : 0;
    remaining_replaceable += logical ? 1 : 0;
  }

  constexpr double kCrashAbsolutePivot = 1e-9;
  constexpr double kCrashRelativePivot = 0.1;
  constexpr int kMaxCrashUpdatesBeforeRefactor = 128;
  HFactorBackend crash_factor;
  bool construction_valid =
      remaining_replaceable > 0 &&
      crash_factor.factorize(sf.A, out_basis.indices.data(), sf_m);
  int updates_since_refactor = 0;
  double selected_partition_sum = 0.0;
  double selected_row_dual_sum = 0.0;
  double selected_activity_sum = 0.0;
  double accepted_relative_pivot_sum = 0.0;
  double min_accepted_relative_pivot =
      std::numeric_limits<double>::infinity();
  std::vector<int> rhs_index;
  std::vector<double> rhs_value;
  std::vector<int> direction_index;
  std::vector<double> direction_value;
  std::vector<int> direction_lookup;
  std::vector<int> row_ep_index;
  std::vector<double> row_ep_value;
  std::vector<int> row_ep_lookup;
  const std::vector<double> unit_value{1.0};
  for (const ColumnCandidate& candidate : ordered_columns) {
    if (!construction_valid || remaining_replaceable == 0) break;
    ++stats.attempted_columns;
    rhs_index.clear();
    rhs_value.clear();
    rhs_index.reserve(static_cast<size_t>(
        std::max(0, col_nnz[static_cast<size_t>(candidate.col)])));
    rhs_value.reserve(rhs_index.capacity());
    for (StandardColumnMatrix::InnerIterator it(sf.A, candidate.col);
         it; ++it) {
      if (it.value() == 0.0) continue;
      rhs_index.push_back(it.row());
      rhs_value.push_back(it.value());
    }
    if (!crash_factor.ftran_indexed(
            rhs_index, rhs_value, direction_index, direction_value,
            direction_lookup, true)) {
      ++stats.stable_pivot_rejections;
      continue;
    }

    double max_abs_direction = 0.0;
    double pivot_abs = 0.0;
    int pivot_position = -1;
    for (size_t k = 0; k < direction_index.size(); ++k) {
      const int position = direction_index[k];
      const double magnitude = std::abs(direction_value[k]);
      if (!std::isfinite(magnitude)) {
        max_abs_direction = std::numeric_limits<double>::infinity();
        break;
      }
      max_abs_direction = std::max(max_abs_direction, magnitude);
      if (position >= 0 && position < sf_m &&
          replaceable[static_cast<size_t>(position)] &&
          magnitude > pivot_abs) {
        pivot_abs = magnitude;
        pivot_position = position;
      }
    }
    const double relative_pivot =
        max_abs_direction > 0.0 ? pivot_abs / max_abs_direction : 0.0;
    if (pivot_position < 0 || !std::isfinite(relative_pivot) ||
        pivot_abs < kCrashAbsolutePivot ||
        relative_pivot < kCrashRelativePivot) {
      ++stats.stable_pivot_rejections;
      continue;
    }

    const std::vector<int> unit_index{pivot_position};
    if (!crash_factor.btran_indexed(
            unit_index, unit_value, row_ep_index, row_ep_value,
            row_ep_lookup, true) ||
        !crash_factor.update_captured(pivot_position, candidate.col)) {
      ++stats.stable_pivot_rejections;
      continue;
    }

    out_basis.indices[static_cast<size_t>(pivot_position)] = candidate.col;
    replaceable[static_cast<size_t>(pivot_position)] = 0;
    --remaining_replaceable;
    ++updates_since_refactor;
    ++stats.structural_matches;
    selected_partition_sum += candidate.partition;
    selected_row_dual_sum += candidate.row_dual;
    selected_activity_sum += candidate.activity;
    accepted_relative_pivot_sum += relative_pivot;
    min_accepted_relative_pivot =
        std::min(min_accepted_relative_pivot, relative_pivot);

    if (crash_factor.needs_refactorise() ||
        updates_since_refactor >= kMaxCrashUpdatesBeforeRefactor) {
      if (!crash_factor.factorize(
              sf.A, out_basis.indices.data(), sf_m)) {
        construction_valid = false;
        break;
      }
      ++stats.crash_refactors;
      updates_since_refactor = 0;
    }
  }

  // Every exchange above was admitted through the current factor. A final
  // full build is the publication check; there is deliberately no rank-repair
  // path that can silently replace an accepted or protected basis column.
  if (construction_valid && stats.structural_matches > 0) {
    stats.rank_valid = crash_factor.factorize(
        sf.A, out_basis.indices.data(), sf_m);
  }
  if (!stats.rank_valid) out_basis.indices = base_basis;

  stats.selected_swaps = 0;
  for (int row = 0; row < sf_m; ++row) {
    if (out_basis.indices[static_cast<size_t>(row)] !=
        base_basis[static_cast<size_t>(row)]) {
      ++stats.selected_swaps;
    }
    const int basic_col = out_basis.indices[static_cast<size_t>(row)];
    if (basic_col >= 0 && basic_col < sf_n) {
      out_basis.at_upper[static_cast<size_t>(basic_col)] = 0;
    }
  }
  if (stats.selected_swaps > 0) {
    const double denominator = std::max(1, stats.structural_matches);
    stats.mean_partition_score = selected_partition_sum / denominator;
    stats.mean_row_dual_score = selected_row_dual_sum / denominator;
    stats.mean_activity_score = selected_activity_sum / denominator;
    stats.min_accepted_relative_pivot = min_accepted_relative_pivot;
    stats.mean_accepted_relative_pivot =
        accepted_relative_pivot_sum / denominator;
  }
  const int min_required_swaps = std::min(seeded ? 3 : 8, stats.candidate_rows);
  stats.passes_screen =
      stats.rank_valid && stats.selected_swaps > 0 &&
      stats.selected_swaps >= min_required_swaps &&
      stats.mean_partition_score >= ((have_box || have_rc) ? 0.2 : 0.05) &&
      stats.mean_row_dual_score >= ((ipm_res != nullptr && ipm_res->constraint_duals.size() > 0) ? 0.02 : 0.0);
  if (lp_basis_trace_enabled()) {
    char detail[256];
    std::snprintf(detail, sizeof(detail),
                  "seeded=%d haveBox=%d haveRc=%d candRows=%d edges=%d "
                  "attempted=%d rejected=%d accepted=%d swaps=%d "
                  "refactors=%d rankValid=%d minReq=%d relPivot=%.3g:%.3g "
                  "part=%.6g rowdual=%.6g activity=%.6g screen=%d",
                  seeded ? 1 : 0, have_box ? 1 : 0, have_rc ? 1 : 0,
                  stats.candidate_rows, stats.candidate_edges,
                  stats.attempted_columns, stats.stable_pivot_rejections,
                  stats.structural_matches, stats.selected_swaps,
                  stats.crash_refactors, stats.rank_valid ? 1 : 0,
                  min_required_swaps, stats.min_accepted_relative_pivot,
                  stats.mean_accepted_relative_pivot,
                  stats.mean_partition_score,
                  stats.mean_row_dual_score, stats.mean_activity_score,
                  stats.passes_screen ? 1 : 0);
    trace_basis_source("primal_activity_recovered", sf, out_basis, detail);
  }
  return stats;
}

SimplexBasis build_primal_crash_basis(const StandardFormLP& sf,
                                      const Eigen::VectorXd& x_orig) {
  SimplexBasis crash_basis;
  (void)recover_primal_activity_basis_impl(sf, x_orig, nullptr, nullptr, crash_basis);
  trace_basis_source("primal_crash_basis", sf, crash_basis, "from=x0");
  return crash_basis;
}

}  // namespace mipsolvers::engine::detail
