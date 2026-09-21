/// @file bc_utils.cpp
/// @brief Utility function implementations for the branch-and-cut solver.

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

std::atomic<std::uint64_t> g_prop_calls{0};
std::atomic<std::uint64_t> g_prop_active_passes{0};
std::atomic<std::uint64_t> g_prop_changed_var_visits{0};
std::atomic<std::uint64_t> g_prop_ineq_rows_visited{0};
std::atomic<std::uint64_t> g_prop_eq_rows_visited{0};
std::atomic<std::uint64_t> g_prop_baseline_full_scan_rows{0};
std::atomic<std::uint64_t> g_prop_bound_tightenings{0};
std::atomic<std::uint64_t> g_prop_reason_clauses_materialized{0};
std::atomic<std::uint64_t> g_prop_wall_ns{0};

void accumulate_propagation_profile(const PropagationProfile& profile) {
  g_prop_calls.fetch_add(profile.calls, std::memory_order_relaxed);
  g_prop_active_passes.fetch_add(profile.active_passes, std::memory_order_relaxed);
  g_prop_changed_var_visits.fetch_add(profile.changed_var_visits, std::memory_order_relaxed);
  g_prop_ineq_rows_visited.fetch_add(profile.ineq_rows_visited, std::memory_order_relaxed);
  g_prop_eq_rows_visited.fetch_add(profile.eq_rows_visited, std::memory_order_relaxed);
  g_prop_baseline_full_scan_rows.fetch_add(profile.baseline_full_scan_rows, std::memory_order_relaxed);
  g_prop_bound_tightenings.fetch_add(profile.bound_tightenings, std::memory_order_relaxed);
  g_prop_reason_clauses_materialized.fetch_add(
      profile.reason_clauses_materialized, std::memory_order_relaxed);
  g_prop_wall_ns.fetch_add(profile.wall_ns, std::memory_order_relaxed);
}

}  // namespace

void reset_propagation_profile() {
  g_prop_calls.store(0, std::memory_order_relaxed);
  g_prop_active_passes.store(0, std::memory_order_relaxed);
  g_prop_changed_var_visits.store(0, std::memory_order_relaxed);
  g_prop_ineq_rows_visited.store(0, std::memory_order_relaxed);
  g_prop_eq_rows_visited.store(0, std::memory_order_relaxed);
  g_prop_baseline_full_scan_rows.store(0, std::memory_order_relaxed);
  g_prop_bound_tightenings.store(0, std::memory_order_relaxed);
  g_prop_reason_clauses_materialized.store(0, std::memory_order_relaxed);
  g_prop_wall_ns.store(0, std::memory_order_relaxed);
}

PropagationProfile get_propagation_profile() {
  PropagationProfile profile;
  profile.calls = g_prop_calls.load(std::memory_order_relaxed);
  profile.active_passes = g_prop_active_passes.load(std::memory_order_relaxed);
  profile.changed_var_visits = g_prop_changed_var_visits.load(std::memory_order_relaxed);
  profile.ineq_rows_visited = g_prop_ineq_rows_visited.load(std::memory_order_relaxed);
  profile.eq_rows_visited = g_prop_eq_rows_visited.load(std::memory_order_relaxed);
  profile.baseline_full_scan_rows = g_prop_baseline_full_scan_rows.load(std::memory_order_relaxed);
  profile.bound_tightenings = g_prop_bound_tightenings.load(std::memory_order_relaxed);
  profile.reason_clauses_materialized =
      g_prop_reason_clauses_materialized.load(std::memory_order_relaxed);
  profile.wall_ns = g_prop_wall_ns.load(std::memory_order_relaxed);
  return profile;
}

double objective_value(const Eigen::VectorXd& c, const Eigen::VectorXd& x, Sense sense) {
  const double v = c.dot(x);
  return (sense == Sense::Minimize) ? v : -v;
}

bool incumbent_prunes_node(bool has_incumbent,
                           double incumbent_obj,
                           double node_bound,
                           double prune_tol) {
  return has_incumbent && node_bound >= incumbent_obj - prune_tol;
}

Eigen::VectorXd project_integer_solution(const std::vector<VariableMeta>& vars,
                                         const Eigen::VectorXd& x,
                                         const Eigen::VectorXd& lb,
                                         const Eigen::VectorXd& ub) {
  Eigen::VectorXd projected = clamp_to_bounds(x, lb, ub);
  const int n = std::min(static_cast<int>(vars.size()), static_cast<int>(projected.size()));
  for (int i = 0; i < n; ++i) {
    if (!is_integer_type(vars[i])) continue;
    projected[i] = std::min(ub[i], std::max(lb[i], std::round(x[i])));
  }
  return projected;
}

Eigen::VectorXd clamp_to_bounds(const Eigen::VectorXd& x,
                                const Eigen::VectorXd& lb,
                                const Eigen::VectorXd& ub) {
  Eigen::VectorXd y = x;
  for (Eigen::Index i = 0; i < y.size(); ++i) {
    y[i] = std::min(ub[i], std::max(lb[i], y[i]));
  }
  return y;
}

bool fractional_indices(const std::vector<VariableMeta>& vars,
                        const Eigen::VectorXd& x,
                        double int_tol,
                        std::vector<int>& out) {
  out.clear();
  for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
    if (!is_integer_type(vars[i])) {
      continue;
    }
    if (!is_integral(x[i], int_tol)) {
      out.push_back(i);
    }
  }
  return !out.empty();
}

bool bounds_consistent(const Eigen::VectorXd& lb, const Eigen::VectorXd& ub) {
  for (Eigen::Index i = 0; i < lb.size(); ++i) {
    if (lb[i] > ub[i] +
                    bound_improvement_tolerance(1e-12, lb[i], ub[i])) {
      return false;
    }
  }
  return true;
}

void apply_node_bounds(std::vector<VariableMeta>& vars,
                       const Eigen::VectorXd& lb,
                       const Eigen::VectorXd& ub) {
  for (int i = 0; i < static_cast<int>(vars.size()); ++i) {
    vars[i].lb = std::max(vars[i].lb, lb[i]);
    vars[i].ub = std::min(vars[i].ub, ub[i]);
  }
}

void add_sparse_rows_to_lp(LPModel& lp,
                            const std::vector<Eigen::SparseVector<double>>& rows,
                            const std::vector<double>& rhs_vals,
                            SeparatorStorageStats* storage_stats) {
  const int n = static_cast<int>(lp.A.cols());
  const int raw_k = static_cast<int>(std::min(rows.size(), rhs_vals.size()));
  std::vector<int> valid_rows;
  valid_rows.reserve(static_cast<std::size_t>(raw_k));
  for (int i = 0; i < raw_k; ++i) {
    const auto& row = rows[static_cast<std::size_t>(i)];
    if (row.size() != n || !std::isfinite(rhs_vals[static_cast<std::size_t>(i)])) {
      continue;
    }
    bool finite = true;
    bool nonzero = false;
    for (Eigen::SparseVector<double>::InnerIterator it(row); it; ++it) {
      if (it.index() < 0 || it.index() >= n || !std::isfinite(it.value())) {
        finite = false;
        break;
      }
      if (std::abs(it.value()) > 1e-15) nonzero = true;
    }
    if (finite && nonzero) valid_rows.push_back(i);
  }

  const int k = static_cast<int>(valid_rows.size());
  if (k == 0) return;
  const int m = static_cast<int>(lp.A.rows());
  const bool had_row_lhs = lp_has_row_lhs(lp);
  const std::uint64_t prior_entries =
      static_cast<std::uint64_t>(lp.A.nonZeros());
  Eigen::VectorXi appended_per_col = Eigen::VectorXi::Zero(n);
  std::uint64_t appended_entries = 0;
  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    for (Eigen::SparseVector<double>::InnerIterator it(rows[static_cast<size_t>(src)]); it; ++it) {
      ++appended_per_col[static_cast<int>(it.index())];
      ++appended_entries;
    }
  }

  const double* old_value_ptr = lp.A.valuePtr();
  lp.A.conservativeResize(m + k, n);

  Eigen::VectorXi reserve_per_col = Eigen::VectorXi::Zero(n);
  bool needs_capacity = false;
  std::uint64_t spare_entries_before_append = 0;
  for (int col = 0; col < n; ++col) {
    const int appended = appended_per_col[col];
    const int used = lp.A.isCompressed()
                         ? lp.A.outerIndexPtr()[col + 1] -
                               lp.A.outerIndexPtr()[col]
                         : lp.A.innerNonZeroPtr()[col];
    const int capacity = lp.A.outerIndexPtr()[col + 1] -
                         lp.A.outerIndexPtr()[col];
    const int spare = capacity - used;
    int requested_spare = 0;
    if (spare < appended) {
      const int post_append_spare = std::max(4, (used + appended) / 2);
      requested_spare = appended + post_append_spare;
    }
    if (requested_spare > spare) {
      reserve_per_col[col] = requested_spare;
      needs_capacity = true;
    }
    spare_entries_before_append +=
        static_cast<std::uint64_t>(std::max(spare, requested_spare));
  }
  if (needs_capacity) lp.A.reserve(reserve_per_col);

  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    for (Eigen::SparseVector<double>::InnerIterator it(
             rows[static_cast<std::size_t>(src)]);
         it; ++it) {
      lp.A.insertBackUncompressed(m + i, static_cast<int>(it.index())) =
          it.value();
    }
  }

  lp.b.conservativeResize(m + k);
  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    lp.b[m + i] = rhs_vals[static_cast<size_t>(src)];
  }

  if (lp.row_lhs.size() > 0) {
    if (had_row_lhs) {
      lp.row_lhs.conservativeResize(m + k);
      lp.row_lhs.tail(k).setConstant(
          -std::numeric_limits<double>::infinity());
    } else {
      lp.row_lhs = Eigen::VectorXd::Constant(
          m + k, -std::numeric_limits<double>::infinity());
    }
  }

  if (storage_stats != nullptr) {
    ++storage_stats->matrix_append_calls;
    storage_stats->matrix_appended_rows += static_cast<std::uint64_t>(k);
    storage_stats->matrix_appended_entries += appended_entries;
    storage_stats->matrix_prior_entries_bypassing_triplet_rebuild +=
        prior_entries;
    if (old_value_ptr != lp.A.valuePtr()) {
      ++storage_stats->matrix_storage_reallocations;
    }
    storage_stats->matrix_peak_spare_entries = std::max(
        storage_stats->matrix_peak_spare_entries,
        spare_entries_before_append - appended_entries);
  }
}

bool satisfies_lp(const LPModel& lp, const Eigen::VectorXd& x, double tol) {
  // Use efficient matrix-vector multiplication instead of row-by-row traversal
  // (ColMajor sparse matrix row(i).dot(x) is O(NNZ) per row, not O(row_nnz))
  if (lp.A.rows() > 0) {
    Eigen::VectorXd Ax = lp.A * x;
    for (int i = 0; i < static_cast<int>(lp.A.rows()); ++i) {
      if (Ax[i] > lp.b[i] + tol) {
        return false;
      }
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (std::isfinite(lhs) && Ax[i] < lhs - tol) {
        return false;
      }
    }
  }
  if (lp.Aeq.rows() > 0) {
    Eigen::VectorXd Aeq_x = lp.Aeq * x;
    for (int i = 0; i < static_cast<int>(lp.Aeq.rows()); ++i) {
      if (std::abs(Aeq_x[i] - lp.beq[i]) > tol) {
        return false;
      }
    }
  }
  for (int i = 0; i < static_cast<int>(lp.vars.size()); ++i) {
    if (x[i] < lp.vars[i].lb - tol || x[i] > lp.vars[i].ub + tol) {
      return false;
    }
    if (is_integer_type(lp.vars[i]) && !is_integral(x[i], tol)) {
      return false;
    }
  }
  return true;
}

void append_branch_reason(Node& node,
                          int var_idx,
                          bool is_lb,
                          double value) {
  BranchDomainLiteral lit{var_idx, value, is_lb};
  const int n = node.lb.size() > 0 ? static_cast<int>(node.lb.size())
                                   : var_idx + 1;
  for (auto& literal : node.branch_reasons.write()) {
    if (literal.var_idx != var_idx || literal.is_lb != is_lb) continue;
    if (is_lb) literal.value = std::max(literal.value, value);
    else literal.value = std::min(literal.value, value);
    rebuild_local_domain_trail(node, n, nullptr, nullptr);
    return;
  }
  node.branch_reasons.push_back(lit);
  rebuild_local_domain_trail(node, n, nullptr, nullptr);
}

void rebuild_local_domain_trail(Node& node,
                                int n,
                                const Eigen::VectorXd* root_lb,
                                const Eigen::VectorXd* root_ub) {
  if (n <= 0) {
    n = node.lb.size() > 0 ? static_cast<int>(node.lb.size())
                           : static_cast<int>(node.ub.size());
  }
  node.local_domain_trail.clear();
  node.local_branch_positions.clear();
  if (n <= 0) return;
  std::vector<int> lower_pos(static_cast<std::size_t>(n), -1);
  std::vector<int> upper_pos(static_cast<std::size_t>(n), -1);
  auto previous_value = [&](const BranchDomainLiteral& lit, int prev_pos) {
    if (prev_pos >= 0 &&
        prev_pos < static_cast<int>(node.local_domain_trail.size())) {
      return node.local_domain_trail[static_cast<std::size_t>(prev_pos)]
          .bound.value;
    }
    if (lit.is_lb) {
      if (root_lb != nullptr && root_lb->size() > lit.var_idx &&
          std::isfinite((*root_lb)[lit.var_idx])) {
        return (*root_lb)[lit.var_idx];
      }
      return -kInf;
    }
    if (root_ub != nullptr && root_ub->size() > lit.var_idx &&
        std::isfinite((*root_ub)[lit.var_idx])) {
      return (*root_ub)[lit.var_idx];
    }
    return kInf;
  };
  auto append_entry = [&](BranchDomainLiteral lit,
                          std::vector<BranchDomainLiteral> reason,
                          int depth,
                          bool is_branch,
                          BranchDomainLiteral source_conflict_literal,
                          bool has_source_conflict_literal,
                          const std::vector<BranchDomainLiteral>*
                              source_conflict_clause = nullptr,
                          bool has_source_conflict_clause = false) {
    if (lit.var_idx < 0 || lit.var_idx >= n || !std::isfinite(lit.value)) {
      return;
    }
    int& side_pos = lit.is_lb ? lower_pos[static_cast<std::size_t>(lit.var_idx)]
                              : upper_pos[static_cast<std::size_t>(lit.var_idx)];
    const int prev_pos = side_pos;
    const int pos = static_cast<int>(node.local_domain_trail.size());
    LocalDomainTrailEntry entry;
    entry.bound = lit;
    entry.reason = std::move(reason);
    entry.pos = pos;
    entry.depth = depth;
    entry.prev_bound_pos = prev_pos;
    entry.prev_bound_value = previous_value(lit, prev_pos);
    entry.is_branch = is_branch;
    entry.source_conflict_literal = source_conflict_literal;
    entry.has_source_conflict_literal = has_source_conflict_literal;
    if (source_conflict_clause != nullptr && has_source_conflict_clause) {
      entry.source_conflict_clause = *source_conflict_clause;
      entry.has_source_conflict_clause = true;
    }
    node.local_domain_trail.push_back(std::move(entry));
    side_pos = pos;
    if (is_branch) node.local_branch_positions.push_back(pos);
  };
  auto trail_entry_active = [&](const LocalDomainTrailEntry& entry) {
    const int j = entry.bound.var_idx;
    if (j < 0 || j >= n || node.lb.size() < n || node.ub.size() < n) {
      return false;
    }
    return entry.bound.is_lb
        ? node.lb[j] >= entry.bound.value - 1e-9
        : node.ub[j] <= entry.bound.value + 1e-9;
  };
  auto append_reason_bound = [&](DomainReasonBound& rb) {
    if (rb.bound.var_idx < 0 || rb.bound.var_idx >= n ||
        !std::isfinite(rb.bound.value)) {
      return;
    }
    bool duplicate_active = false;
    for (const auto& entry : node.local_domain_trail) {
      if (entry.bound.var_idx != rb.bound.var_idx ||
          entry.bound.is_lb != rb.bound.is_lb) {
        continue;
      }
      if (branch_literal_stronger_or_equal(entry.bound, rb.bound, 1e-9) &&
          branch_literal_stronger_or_equal(rb.bound, entry.bound, 1e-9) &&
          trail_entry_active(entry)) {
        rb.trail_pos = entry.pos;
        rb.prev_bound_pos = entry.prev_bound_pos;
        rb.prev_bound_value = entry.prev_bound_value;
        duplicate_active = true;
        break;
      }
    }
    if (duplicate_active) return;
    append_entry(rb.bound, rb.reason, rb.depth, false,
                 rb.source_conflict_literal, rb.has_source_conflict_literal,
                 &rb.source_conflict_clause, rb.has_source_conflict_clause);
    if (!node.local_domain_trail.empty()) {
      const auto& entry = node.local_domain_trail.back();
      rb.trail_pos = entry.pos;
      rb.prev_bound_pos = entry.prev_bound_pos;
      rb.prev_bound_value = entry.prev_bound_value;
    }
  };

  const int branch_count = static_cast<int>(node.branch_reasons.size());
  auto& reason_bounds = node.domain_reason_bounds.write();
  std::vector<std::vector<int>> reason_by_depth(
      static_cast<std::size_t>(branch_count + 1));
  for (int r = 0; r < static_cast<int>(reason_bounds.size()); ++r) {
    const int depth = std::clamp(reason_bounds[static_cast<std::size_t>(r)].depth,
                                 0, branch_count);
    reason_by_depth[static_cast<std::size_t>(depth)].push_back(r);
  }

  for (int r : reason_by_depth[0]) {
    append_reason_bound(reason_bounds[static_cast<std::size_t>(r)]);
  }
  for (int i = 0; i < branch_count; ++i) {
    const auto& lit = node.branch_reasons[static_cast<std::size_t>(i)];
    append_entry(lit, {lit}, i + 1, true, BranchDomainLiteral{}, false);
    for (int r : reason_by_depth[static_cast<std::size_t>(i + 1)]) {
      append_reason_bound(reason_bounds[static_cast<std::size_t>(r)]);
    }
  }
  for (int j = 0; j < n; ++j) {
    if (node.lb.size() >= n && std::isfinite(node.lb[j])) {
      const double root = (root_lb != nullptr && root_lb->size() > j)
                              ? (*root_lb)[j]
                              : -kInf;
      bool covered = false;
      for (const auto& entry : node.local_domain_trail) {
        if (entry.bound.var_idx == j && entry.bound.is_lb &&
            branch_literal_stronger_or_equal(entry.bound,
                                             BranchDomainLiteral{j, node.lb[j], true},
                                             1e-9)) {
          covered = true;
          break;
        }
      }
      if (!covered && node.lb[j] > root + 1e-9) {
        append_entry(BranchDomainLiteral{j, node.lb[j], true},
                     {}, node.depth, false, BranchDomainLiteral{}, false);
      }
    }
    if (node.ub.size() >= n && std::isfinite(node.ub[j])) {
      const double root = (root_ub != nullptr && root_ub->size() > j)
                              ? (*root_ub)[j]
                              : kInf;
      bool covered = false;
      for (const auto& entry : node.local_domain_trail) {
        if (entry.bound.var_idx == j && !entry.bound.is_lb &&
            branch_literal_stronger_or_equal(entry.bound,
                                             BranchDomainLiteral{j, node.ub[j], false},
                                             1e-9)) {
          covered = true;
          break;
        }
      }
      if (!covered && node.ub[j] < root - 1e-9) {
        append_entry(BranchDomainLiteral{j, node.ub[j], false},
                     {}, node.depth, false, BranchDomainLiteral{}, false);
      }
    }
  }
}

void append_domain_reason_bound(Node& node,
                                DomainReasonBound rb,
                                int n,
                                const Eigen::VectorXd* root_lb,
                                const Eigen::VectorXd* root_ub) {
  if (n <= 0) {
    n = node.lb.size() > 0 ? static_cast<int>(node.lb.size())
                           : static_cast<int>(node.ub.size());
  }
  node.domain_reason_bounds.push_back(std::move(rb));
  DomainReasonBound& stored = node.domain_reason_bounds.mutable_back();
  if (n <= 0 || stored.bound.var_idx < 0 || stored.bound.var_idx >= n ||
      !std::isfinite(stored.bound.value)) {
    return;
  }

  if (node.local_domain_trail.empty() && !node.branch_reasons.empty()) {
    rebuild_local_domain_trail(node, n, root_lb, root_ub);
    return;
  }

  const int branch_count = static_cast<int>(node.branch_reasons.size());
  if (node.local_branch_positions.size() !=
      static_cast<std::size_t>(branch_count)) {
    rebuild_local_domain_trail(node, n, root_lb, root_ub);
    return;
  }
  for (int i = 0; i < branch_count; ++i) {
    const int pos = node.local_branch_positions[static_cast<std::size_t>(i)];
    if (pos < 0 || pos >= static_cast<int>(node.local_domain_trail.size()) ||
        !node.local_domain_trail[static_cast<std::size_t>(pos)].is_branch) {
      rebuild_local_domain_trail(node, n, root_lb, root_ub);
      return;
    }
  }

  auto trail_entry_active = [&](const LocalDomainTrailEntry& entry) {
    const int j = entry.bound.var_idx;
    if (j < 0 || j >= n || node.lb.size() < n || node.ub.size() < n) {
      return false;
    }
    return entry.bound.is_lb
        ? node.lb[j] >= entry.bound.value - 1e-9
        : node.ub[j] <= entry.bound.value + 1e-9;
  };

  for (auto& entry : node.local_domain_trail.write()) {
    if (entry.bound.var_idx != stored.bound.var_idx ||
        entry.bound.is_lb != stored.bound.is_lb) {
      continue;
    }
    if (branch_literal_stronger_or_equal(entry.bound, stored.bound, 1e-9) &&
        branch_literal_stronger_or_equal(stored.bound, entry.bound, 1e-9) &&
        trail_entry_active(entry)) {
      if (!entry.is_branch && entry.reason.empty() && !stored.reason.empty()) {
        entry.reason = stored.reason;
        entry.depth = stored.depth;
        entry.source_conflict_literal = stored.source_conflict_literal;
        entry.has_source_conflict_literal =
            stored.has_source_conflict_literal;
        entry.source_conflict_clause = stored.source_conflict_clause;
        entry.has_source_conflict_clause =
            stored.has_source_conflict_clause;
      }
      stored.trail_pos = entry.pos;
      stored.prev_bound_pos = entry.prev_bound_pos;
      stored.prev_bound_value = entry.prev_bound_value;
      return;
    }
  }

  int prev_pos = -1;
  for (int p = static_cast<int>(node.local_domain_trail.size()) - 1; p >= 0;
       --p) {
    const auto& entry = node.local_domain_trail[static_cast<std::size_t>(p)];
    if (entry.bound.var_idx == stored.bound.var_idx &&
        entry.bound.is_lb == stored.bound.is_lb) {
      prev_pos = p;
      break;
    }
  }
  auto previous_value = [&]() {
    if (prev_pos >= 0 &&
        prev_pos < static_cast<int>(node.local_domain_trail.size())) {
      return node.local_domain_trail[static_cast<std::size_t>(prev_pos)]
          .bound.value;
    }
    const int j = stored.bound.var_idx;
    if (stored.bound.is_lb) {
      if (root_lb != nullptr && root_lb->size() > j &&
          std::isfinite((*root_lb)[j])) {
        return (*root_lb)[j];
      }
      return -kInf;
    }
    if (root_ub != nullptr && root_ub->size() > j &&
        std::isfinite((*root_ub)[j])) {
      return (*root_ub)[j];
    }
    return kInf;
  };

  const int pos = static_cast<int>(node.local_domain_trail.size());
  LocalDomainTrailEntry entry;
  entry.bound = stored.bound;
  entry.reason = stored.reason;
  entry.pos = pos;
  entry.depth = stored.depth;
  entry.prev_bound_pos = prev_pos;
  entry.prev_bound_value = previous_value();
  entry.is_branch = false;
  entry.source_conflict_literal = stored.source_conflict_literal;
  entry.has_source_conflict_literal = stored.has_source_conflict_literal;
  entry.source_conflict_clause = stored.source_conflict_clause;
  entry.has_source_conflict_clause = stored.has_source_conflict_clause;
  node.local_domain_trail.push_back(std::move(entry));
  stored.trail_pos = pos;
  stored.prev_bound_pos = prev_pos;
  stored.prev_bound_value =
      node.local_domain_trail[static_cast<std::size_t>(pos)].prev_bound_value;
}

bool try_build_binary_conflict_cut(const LPModel& base_lp,
                                   const std::vector<BranchDomainLiteral>& reasons,
                                   PoolCut& out_cut,
                                   int max_literals) {
  constexpr double tol = 1e-9;

  const int n = static_cast<int>(base_lp.vars.size());
  if (reasons.empty()) {
    return false;
  }

  std::vector<BranchDomainLiteral> reason_literals = reasons;
  canonicalize_branch_literals(reason_literals);
  if (static_cast<int>(reason_literals.size()) > max_literals) {
    return false;
  }

  std::vector<std::pair<int, double>> cut_literals;
  cut_literals.reserve(static_cast<size_t>(std::max(1, std::min(n, max_literals))));
  int fixed_one_count = 0;

  for (const auto& reason : reason_literals) {
    const int j = reason.var_idx;
    if (j < 0 || j >= n || base_lp.vars[j].type != VarType::Binary) {
      return false;
    }

    const bool fixed_zero = !reason.is_lb && reason.value <= tol;
    const bool fixed_one = reason.is_lb && reason.value >= 1.0 - tol;
    if (!fixed_zero && !fixed_one) {
      return false;
    }

    cut_literals.emplace_back(j, fixed_zero ? -1.0 : 1.0);
    if (fixed_one) {
      ++fixed_one_count;
    }
    if (static_cast<int>(cut_literals.size()) > max_literals) {
      return false;
    }
  }

  if (cut_literals.empty()) {
    return false;
  }

  Eigen::SparseVector<double> coeff(n);
  coeff.reserve(static_cast<int>(cut_literals.size()));
  double norm2 = 0.0;
  for (const auto& [j, value] : cut_literals) {
    coeff.insertBack(j) = value;
    norm2 += value * value;
  }

  out_cut = PoolCut{std::move(coeff), static_cast<double>(fixed_one_count - 1),
                    0, 0.0, std::sqrt(norm2), 0};
  return true;
}

void push_node(std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
               std::vector<Node>& dfs,
               const Node& node,
               NodeSelection mode,
               bool has_incumbent) {
  if (mode == NodeSelection::DepthFirst) {
    dfs.push_back(node);
    return;
  }
  if (mode == NodeSelection::Hybrid && !has_incumbent) {
    dfs.push_back(node);
    return;
  }
  pq.push(QueueItem{node, node.bound});
}

bool pop_node(std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
              std::vector<Node>& dfs,
              NodeSelection mode,
              bool /*has_incumbent*/,
              Node& out) {
  if (mode == NodeSelection::DepthFirst) {
    if (dfs.empty()) {
      return false;
    }
    out = dfs.back();
    dfs.pop_back();
    return true;
  }

  // Hybrid mode: DFS before incumbent to find feasible solution fast.
  // After incumbent, still prefer DFS (warm-start friendly) but
  // fall back to best-bound (PQ) when DFS is empty.
  if (!dfs.empty()) {
    out = dfs.back();
    dfs.pop_back();
    return true;
  }

  if (!pq.empty()) {
    out = pq.top().node;
    pq.pop();
    return true;
  }

  return false;
}

double live_node_lower_bound(const std::priority_queue<QueueItem, std::vector<QueueItem>, QueueItemMinKey>& pq,
                             const std::vector<Node>& dfs) {
  double lb = kInf;
  if (!pq.empty()) {
    lb = pq.top().node.bound;
  }
  for (const auto& n : dfs) {
    lb = std::min(lb, n.bound);
  }
  return lb;
}

bool satisfies_with_bounds(const LPModel& lp,
                           const Eigen::VectorXd& x,
                           const Eigen::VectorXd& node_lb,
                           const Eigen::VectorXd& node_ub,
                           double tol) {
  const int n = static_cast<int>(lp.vars.size());
  if (x.size() != n || node_lb.size() != n || node_ub.size() != n ||
      lp.A.cols() != n || lp.Aeq.cols() != n ||
      lp.b.size() != lp.A.rows() || lp.beq.size() != lp.Aeq.rows() ||
      !std::isfinite(tol) || tol < 0.0) {
    return false;
  }
  // Check variable bounds (use tighter of node bounds and lp.vars bounds).
  for (int i = 0; i < n; ++i) {
    if (!std::isfinite(x[i]) || std::isnan(node_lb[i]) ||
        std::isnan(node_ub[i]) || std::isnan(lp.vars[i].lb) ||
        std::isnan(lp.vars[i].ub)) {
      return false;
    }
    const double lb = std::max(lp.vars[i].lb, node_lb[i]);
    const double ub = std::min(lp.vars[i].ub, node_ub[i]);
    if (x[i] < lb - tol || x[i] > ub + tol) {
      return false;
    }
    if (is_integer_type(lp.vars[i]) && !is_integral(x[i], tol)) {
      return false;
    }
  }
  // Check inequality constraints: A*x <= b.  Use a single matrix-vector
  // product rather than per-row .dot() — on ColMajor sparse matrices row()
  // materialises each row and makes an O(m·nnz/n) scan, which is catastrophic
  // for xlarge UC (60k rows).  One mat-vec is O(nnz).
  if (lp.A.rows() > 0) {
    const Eigen::VectorXd Ax = lp.A * x;
    for (int i = 0; i < Ax.size(); ++i) {
      if (Ax[i] > lp.b[i] + tol) return false;
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (std::isfinite(lhs) && Ax[i] < lhs - tol) return false;
    }
  }
  // Check equality constraints: Aeq*x = beq.
  if (lp.Aeq.rows() > 0) {
    const Eigen::VectorXd Aeqx = lp.Aeq * x;
    for (int i = 0; i < Aeqx.size(); ++i) {
      if (std::abs(Aeqx[i] - lp.beq[i]) > tol) return false;
    }
  }
  return true;
}

int reduced_cost_fixing(const std::vector<VariableMeta>& vars,
                        const Eigen::VectorXd& x_relax,
                        const Eigen::VectorXd& reduced_costs,
                        const std::vector<int>& basis_indices,
                        double node_bound,
                        double incumbent_obj,
                        double int_tol,
                        Eigen::VectorXd& node_lb,
                        Eigen::VectorXd& node_ub,
                        std::vector<BranchDomainLiteral>* forbidden_literals,
                        const Eigen::VectorXd* sf_col_scale,
                        std::vector<DomainReasonBound>* reason_bounds,
                        const std::vector<BranchDomainLiteral>* reason_frontier,
                        int reason_depth,
                        int reason_trail_offset) {
  const double gap = incumbent_obj - node_bound;
  if (gap <= 1e-12) return 0;

  const int n = static_cast<int>(vars.size());
  if (x_relax.size() < n || reduced_costs.size() < n) return 0;

  // Build a hash set of basic variable indices (size = #rows, not #vars).
  // This avoids allocating an O(n) vector when only O(m) entries are basic.
  std::unordered_set<int> basic_set;
  basic_set.reserve(basis_indices.size());
  for (int idx : basis_indices) {
    if (idx >= 0 && idx < n) basic_set.insert(idx);
  }

  int fixed = 0;
  for (int j = 0; j < n; ++j) {
    if (!is_integer_type(vars[j])) continue;
    if (basic_set.count(j)) continue;
    if (std::abs(node_ub[j] - node_lb[j]) < 1e-9) continue;  // already fixed

    // HiGHS' MIP propagation consumes minimization-space `col_dual`.
    // Native simplex stores scaled standard-form maximization reduced costs,
    // so the original-column dual is recovered by undoing both conventions.
    const double scale =
        sf_col_scale != nullptr && sf_col_scale->size() > j &&
                std::isfinite((*sf_col_scale)[j]) &&
                std::abs((*sf_col_scale)[j]) > 1e-18
            ? (*sf_col_scale)[j]
            : 1.0;
    const double lpredcost = -reduced_costs[j] / scale;
    const double tol = std::max(10.0 * int_tol,
                                1e-12 * std::max(1.0, std::abs(gap)));
    if (!std::isfinite(lpredcost) || std::abs(lpredcost) <= tol) continue;

    const double width = node_ub[j] - node_lb[j];
    if (!std::isfinite(width) || width <= int_tol) continue;
    // A reduced-cost implication is directional: a positive minimization
    // column dual is valid from the active lower side, while a negative one is
    // valid from the active upper side. Basis membership alone is insufficient
    // after standard-form remapping or a degenerate/stale warm basis. Requiring
    // primal-side agreement prevents applying the correct formula to the wrong
    // bound.
    const double side_tol = std::max(
        10.0 * int_tol,
        1e-9 * std::max({1.0, std::abs(node_lb[j]), std::abs(node_ub[j])}));
    const bool at_lower = std::abs(x_relax[j] - node_lb[j]) <= side_tol;
    const bool at_upper = std::abs(x_relax[j] - node_ub[j]) <= side_tol;
    const double max_increase = lpredcost * width;
    if (max_increase > gap && at_lower) {
      if (!std::isfinite(node_lb[j])) continue;
      const double old_ub = node_ub[j];
      double new_ub = std::floor(gap / lpredcost + node_lb[j] + int_tol);
      new_ub = std::min(old_ub, std::max(node_lb[j], new_ub));
      if (new_ub >= old_ub - 1e-9) continue;
      const BranchDomainLiteral forbidden{
          j, std::floor(new_ub + 1e-9) + 1.0, true};
      if (forbidden_literals != nullptr) forbidden_literals->push_back(forbidden);
      if (reason_bounds != nullptr) {
        DomainReasonBound rb;
        rb.bound = BranchDomainLiteral{j, new_ub, false};
        if (reason_frontier != nullptr) rb.reason = *reason_frontier;
        canonicalize_branch_literals(rb.reason);
        rb.trail_pos = reason_trail_offset +
                       static_cast<int>(reason_bounds->size());
        rb.depth = reason_depth;
        rb.source_conflict_literal = forbidden;
        rb.has_source_conflict_literal = true;
        rb.source_conflict_clause = rb.reason;
        rb.source_conflict_clause.push_back(forbidden);
        canonicalize_branch_literals(rb.source_conflict_clause);
        rb.has_source_conflict_clause = true;
        const double source_activity =
            node_bound + (forbidden.value - node_lb[j]) * lpredcost;
        rb.has_proof_activity_audit =
            std::isfinite(source_activity) && std::isfinite(incumbent_obj);
        rb.proof_activity = source_activity;
        rb.proof_required_activity = incumbent_obj;
        rb.proof_activity_margin = source_activity - incumbent_obj;
        reason_bounds->push_back(std::move(rb));
      }
      node_ub[j] = new_ub;
      ++fixed;
    } else if (max_increase < -gap && at_upper) {
      if (!std::isfinite(node_ub[j])) continue;
      const double old_lb = node_lb[j];
      double new_lb = std::ceil(gap / lpredcost + node_ub[j] - int_tol);
      new_lb = std::max(old_lb, std::min(node_ub[j], new_lb));
      if (new_lb <= old_lb + 1e-9) continue;
      const BranchDomainLiteral forbidden{
          j, std::ceil(new_lb - 1e-9) - 1.0, false};
      if (forbidden_literals != nullptr) forbidden_literals->push_back(forbidden);
      if (reason_bounds != nullptr) {
        DomainReasonBound rb;
        rb.bound = BranchDomainLiteral{j, new_lb, true};
        if (reason_frontier != nullptr) rb.reason = *reason_frontier;
        canonicalize_branch_literals(rb.reason);
        rb.trail_pos = reason_trail_offset +
                       static_cast<int>(reason_bounds->size());
        rb.depth = reason_depth;
        rb.source_conflict_literal = forbidden;
        rb.has_source_conflict_literal = true;
        rb.source_conflict_clause = rb.reason;
        rb.source_conflict_clause.push_back(forbidden);
        canonicalize_branch_literals(rb.source_conflict_clause);
        rb.has_source_conflict_clause = true;
        const double source_activity =
            node_bound + (forbidden.value - node_ub[j]) * lpredcost;
        rb.has_proof_activity_audit =
            std::isfinite(source_activity) && std::isfinite(incumbent_obj);
        rb.proof_activity = source_activity;
        rb.proof_required_activity = incumbent_obj;
        rb.proof_activity_margin = source_activity - incumbent_obj;
        reason_bounds->push_back(std::move(rb));
      }
      node_lb[j] = new_lb;
      ++fixed;
    }
  }
  return fixed;
}

int node_bound_propagation(const LPModel& lp,
                           Eigen::VectorXd& node_lb,
                           Eigen::VectorXd& node_ub,
                           int max_rounds) {
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;
  return node_bound_propagation(lp, A_row, Aeq_row, node_lb, node_ub,
                                max_rounds);
}

int node_bound_propagation(const LPModel& lp,
                           const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
                           const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
                           Eigen::VectorXd& node_lb,
                           Eigen::VectorXd& node_ub,
                           int max_rounds) {
  const int m_ineq = static_cast<int>(A_row.rows());
  const int m_eq = static_cast<int>(Aeq_row.rows());
  const int n_vars = static_cast<int>(node_lb.size());
  int total_tightened = 0;
  auto finite_domain_bound = [](double value) {
    return std::isfinite(value) && std::abs(value) < 1e19;
  };

  // Build column-to-row watchlists (O(nnz)) so subsequent rounds only revisit
  // rows affected by a bound change. This replaces the O(m * nnz_per_row)
  // full-scan per round with O(dirty_rows * nnz_per_row), which is critical
  // for SCUC/large UC instances with many tight propagation rounds.
  std::vector<std::vector<int>> col_to_ineq(static_cast<size_t>(n_vars));
  std::vector<std::vector<int>> col_to_eq(static_cast<size_t>(n_vars));
  for (int r = 0; r < m_ineq; ++r) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j < n_vars && std::abs(it.value()) > 1e-15) col_to_ineq[j].push_back(r);
    }
  }
  for (int r = 0; r < m_eq; ++r) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j < n_vars && std::abs(it.value()) > 1e-15) col_to_eq[j].push_back(r);
    }
  }

  // dirty bitsets: all rows active on the first round; only changed rows later.
  std::vector<char> dirty_ineq(static_cast<size_t>(m_ineq), 1);
  std::vector<char> dirty_eq(static_cast<size_t>(m_eq), 1);
  std::vector<char> next_dirty_ineq(static_cast<size_t>(m_ineq), 0);
  std::vector<char> next_dirty_eq(static_cast<size_t>(m_eq), 0);

  // Mark all rows containing column j dirty for the next round.
  auto mark_col_dirty = [&](int j) {
    if (j >= n_vars) return;
    for (int rr : col_to_ineq[static_cast<size_t>(j)]) next_dirty_ineq[static_cast<size_t>(rr)] = 1;
    for (int rr : col_to_eq[static_cast<size_t>(j)]) next_dirty_eq[static_cast<size_t>(rr)] = 1;
  };

  for (int round = 0; round < max_rounds; ++round) {
    int round_tightened = 0;
    std::fill(next_dirty_ineq.begin(), next_dirty_ineq.end(), 0);
    std::fill(next_dirty_eq.begin(), next_dirty_eq.end(), 0);

    for (int r = 0; r < m_ineq; ++r) {
      if (!dirty_ineq[static_cast<size_t>(r)]) continue;
      auto propagate_upper_side = [&](double row_sign, double rhs) {
        if (!std::isfinite(rhs)) return;
        StableActivitySum min_activity;
        bool has_inf = false;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          if (a > 0.0) {
            if (!finite_domain_bound(node_lb[j])) { has_inf = true; break; }
            min_activity.add_product(a, node_lb[j]);
          } else {
            if (!finite_domain_bound(node_ub[j])) { has_inf = true; break; }
            min_activity.add_product(a, node_ub[j]);
          }
        }
        if (has_inf) return;

        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_lb[j] : node_ub[j];
          const double residual = min_activity.upper_residual(
              rhs, a, activity_bound, 1e-9);
          if (a > 0.0) {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - bound_improvement_tolerance(
                                             1e-9, node_ub[j], new_ub)) {
              node_ub[j] = new_ub;
              ++round_tightened;
              mark_col_dirty(j);
            }
          } else {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + bound_improvement_tolerance(
                                             1e-9, node_lb[j], new_lb)) {
              node_lb[j] = new_lb;
              ++round_tightened;
              mark_col_dirty(j);
            }
          }
        }
      };
      propagate_upper_side(1.0, lp.b[r]);
      const double lhs = lp_row_lhs_or_neg_inf(lp, r);
      if (std::isfinite(lhs)) propagate_upper_side(-1.0, -lhs);
    }

    for (int r = 0; r < m_eq; ++r) {
      if (!dirty_eq[static_cast<size_t>(r)]) continue;
      StableActivitySum min_activity;
      StableActivitySum max_activity;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!finite_domain_bound(node_lb[j])) has_inf_min = true;
          else min_activity.add_product(a, node_lb[j]);
          if (!finite_domain_bound(node_ub[j])) has_inf_max = true;
          else max_activity.add_product(a, node_ub[j]);
        } else {
          if (!finite_domain_bound(node_ub[j])) has_inf_min = true;
          else min_activity.add_product(a, node_ub[j]);
          if (!finite_domain_bound(node_lb[j])) has_inf_max = true;
          else max_activity.add_product(a, node_lb[j]);
        }
      }

      if (!has_inf_min) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_lb[j] : node_ub[j];
          const double residual = min_activity.upper_residual(
              lp.beq[r], a, activity_bound, 1e-9);
          if (a > 0.0) {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - bound_improvement_tolerance(
                                             1e-9, node_ub[j], new_ub)) {
              node_ub[j] = new_ub;
              ++round_tightened;
              mark_col_dirty(j);
            }
          } else {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + bound_improvement_tolerance(
                                             1e-9, node_lb[j], new_lb)) {
              node_lb[j] = new_lb;
              ++round_tightened;
              mark_col_dirty(j);
            }
          }
        }
      }
      if (!has_inf_max) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_ub[j] : node_lb[j];
          const double residual = max_activity.lower_residual(
              lp.beq[r], a, activity_bound, 1e-9);
          if (a > 0.0) {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + bound_improvement_tolerance(
                                             1e-9, node_lb[j], new_lb)) {
              node_lb[j] = new_lb;
              ++round_tightened;
              mark_col_dirty(j);
            }
          } else {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - bound_improvement_tolerance(
                                             1e-9, node_ub[j], new_ub)) {
              node_ub[j] = new_ub;
              ++round_tightened;
              mark_col_dirty(j);
            }
          }
        }
      }
    }

    total_tightened += round_tightened;
    if (round_tightened == 0) break;
    std::swap(dirty_ineq, next_dirty_ineq);
    std::swap(dirty_eq, next_dirty_eq);
  }
  return total_tightened;
}

int node_bound_propagation_tracked(
    const LPModel& lp,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    int max_rounds,
    std::vector<BoundChangeInfo>& changes_out) {
  const int m_ineq = static_cast<int>(A_row.rows());
  const int m_eq = static_cast<int>(Aeq_row.rows());
  int total_tightened = 0;
  auto finite_domain_bound = [](double value) {
    return std::isfinite(value) && std::abs(value) < 1e19;
  };

  for (int round = 0; round < max_rounds; ++round) {
    int round_tightened = 0;

    for (int r = 0; r < m_ineq; ++r) {
      StableActivitySum min_activity;
      bool has_inf = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!finite_domain_bound(node_lb[j])) { has_inf = true; break; }
          min_activity.add_product(a, node_lb[j]);
        } else {
          if (!finite_domain_bound(node_ub[j])) { has_inf = true; break; }
          min_activity.add_product(a, node_ub[j]);
        }
      }
      if (has_inf) continue;

      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        const double activity_bound =
            (a > 0.0) ? node_lb[j] : node_ub[j];
        const double residual = min_activity.upper_residual(
            lp.b[r], a, activity_bound, 1e-9);
        if (a > 0.0) {
          double new_ub = residual / a;
          if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
          if (new_ub < node_ub[j] - bound_improvement_tolerance(
                                           1e-9, node_ub[j], new_ub)) {
            changes_out.push_back(
                {j, new_ub - node_ub[j], false, node_ub[j], new_ub});
            node_ub[j] = new_ub;
            ++round_tightened;
          }
        } else {
          double new_lb = residual / a;
          if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
          if (new_lb > node_lb[j] + bound_improvement_tolerance(
                                           1e-9, node_lb[j], new_lb)) {
            changes_out.push_back(
                {j, new_lb - node_lb[j], true, node_lb[j], new_lb});
            node_lb[j] = new_lb;
            ++round_tightened;
          }
        }
      }
    }

    for (int r = 0; r < m_eq; ++r) {
      StableActivitySum min_activity;
      StableActivitySum max_activity;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!finite_domain_bound(node_lb[j])) has_inf_min = true;
          else min_activity.add_product(a, node_lb[j]);
          if (!finite_domain_bound(node_ub[j])) has_inf_max = true;
          else max_activity.add_product(a, node_ub[j]);
        } else {
          if (!finite_domain_bound(node_ub[j])) has_inf_min = true;
          else min_activity.add_product(a, node_ub[j]);
          if (!finite_domain_bound(node_lb[j])) has_inf_max = true;
          else max_activity.add_product(a, node_lb[j]);
        }
      }

      if (!has_inf_min) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_lb[j] : node_ub[j];
          const double residual = min_activity.upper_residual(
              lp.beq[r], a, activity_bound, 1e-9);
          if (a > 0.0) {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - bound_improvement_tolerance(
                                             1e-9, node_ub[j], new_ub)) {
              changes_out.push_back(
                  {j, new_ub - node_ub[j], false, node_ub[j], new_ub});
              node_ub[j] = new_ub;
              ++round_tightened;
            }
          } else {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + bound_improvement_tolerance(
                                             1e-9, node_lb[j], new_lb)) {
              changes_out.push_back(
                  {j, new_lb - node_lb[j], true, node_lb[j], new_lb});
              node_lb[j] = new_lb;
              ++round_tightened;
            }
          }
        }
      }
      if (!has_inf_max) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_ub[j] : node_lb[j];
          const double residual = max_activity.lower_residual(
              lp.beq[r], a, activity_bound, 1e-9);
          if (a > 0.0) {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + bound_improvement_tolerance(
                                             1e-9, node_lb[j], new_lb)) {
              changes_out.push_back(
                  {j, new_lb - node_lb[j], true, node_lb[j], new_lb});
              node_lb[j] = new_lb;
              ++round_tightened;
            }
          } else {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - bound_improvement_tolerance(
                                             1e-9, node_ub[j], new_ub)) {
              changes_out.push_back(
                  {j, new_ub - node_ub[j], false, node_ub[j], new_ub});
              node_ub[j] = new_ub;
              ++round_tightened;
            }
          }
        }
      }
    }

    total_tightened += round_tightened;
    if (round_tightened == 0) break;
  }
  return total_tightened;
}

void BinaryImplicationGraph::reset(int n_vars) {
  n_vars_ = std::max(0, n_vars);
  num_implications_ = 0;
  adjacency_.assign(static_cast<std::size_t>(2 * n_vars_), {});
}

bool BinaryImplicationGraph::add_implication(int trigger_var,
                                             bool trigger_value_one,
                                             int implied_var,
                                             bool implied_is_lb,
                                             double implied_value) {
  if (trigger_var < 0 || implied_var < 0 || trigger_var >= n_vars_ || implied_var >= n_vars_) {
    return false;
  }
  if (adjacency_.empty()) {
    adjacency_.assign(static_cast<std::size_t>(2 * n_vars_), {});
  }
  auto& arcs = adjacency_[literal_index(trigger_var, trigger_value_one)];
  for (const auto& arc : arcs) {
    if (arc.var_idx == implied_var && arc.is_lb == implied_is_lb &&
        std::abs(arc.value - implied_value) <= 1e-9) {
      return false;
    }
  }
  arcs.push_back({implied_var, implied_value, implied_is_lb});
  ++num_implications_;
  return true;
}

int BinaryImplicationGraph::propagate(const std::vector<VariableMeta>& vars,
                                      Eigen::VectorXd& lb,
                                      Eigen::VectorXd& ub,
                                      std::vector<BoundChangeInfo>* changes_out,
                                      double tol) const {
  if (empty()) return 0;
  const int n = std::min({n_vars_, static_cast<int>(vars.size()),
                          static_cast<int>(lb.size()), static_cast<int>(ub.size())});
  int tightened = 0;
  std::deque<std::pair<int, bool>> queue;
  std::vector<char> seen_zero(static_cast<std::size_t>(n), 0);
  std::vector<char> seen_one(static_cast<std::size_t>(n), 0);

  auto binary_like_domain = [&](int var_idx) {
    if (var_idx < 0 || var_idx >= n) return false;
    const auto& var = vars[static_cast<std::size_t>(var_idx)];
    if (var.type == VarType::Binary) return true;
    return std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -tol && var.ub <= 1.0 + tol;
  };
  auto enqueue_literal = [&](int var_idx, bool value_one) {
    if (var_idx < 0 || var_idx >= n) return;
    if (!binary_like_domain(var_idx)) return;
    auto& seen = value_one ? seen_one[static_cast<std::size_t>(var_idx)]
                           : seen_zero[static_cast<std::size_t>(var_idx)];
    if (seen) return;
    seen = 1;
    queue.emplace_back(var_idx, value_one);
  };

  for (int j = 0; j < n; ++j) {
    if (!binary_like_domain(j)) continue;
    if (lb[j] >= 1.0 - tol && ub[j] <= 1.0 + tol) enqueue_literal(j, true);
    if (ub[j] <= tol && lb[j] >= -tol) enqueue_literal(j, false);
  }

  while (!queue.empty()) {
    const auto [trigger_var, trigger_value_one] = queue.front();
    queue.pop_front();
    const auto& arcs = adjacency_[literal_index(trigger_var, trigger_value_one)];
    for (const auto& arc : arcs) {
      const int j = arc.var_idx;
      if (j < 0 || j >= n) continue;
      if (arc.is_lb) {
        double new_lb = arc.value;
        if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
          new_lb = std::ceil(new_lb - tol);
        }
        if (new_lb > lb[j] + tol) {
          if (changes_out != nullptr) {
            changes_out->push_back(
                {j, new_lb - lb[j], true, lb[j], new_lb});
          }
          lb[j] = new_lb;
          ++tightened;
          if (binary_like_domain(j) && lb[j] >= 1.0 - tol && ub[j] <= 1.0 + tol) {
            enqueue_literal(j, true);
          }
        }
      } else {
        double new_ub = arc.value;
        if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
          new_ub = std::floor(new_ub + tol);
        }
        if (new_ub < ub[j] - tol) {
          if (changes_out != nullptr) {
            changes_out->push_back(
                {j, new_ub - ub[j], false, ub[j], new_ub});
          }
          ub[j] = new_ub;
          ++tightened;
          if (binary_like_domain(j) && ub[j] <= tol && lb[j] >= -tol) {
            enqueue_literal(j, false);
          }
        }
      }
    }
  }

  return tightened;
}

void VariableBoundTable::reset(int n_vars) {
  n_vars_ = std::max(0, n_vars);
  num_varbounds_ = 0;
  stats_ = SourceStats{};
  vubs_.assign(static_cast<std::size_t>(n_vars_), {});
  vlbs_.assign(static_cast<std::size_t>(n_vars_), {});
}

const std::vector<VariableBoundTable::Entry>& VariableBoundTable::vubs(
    int col) const {
  static const std::vector<Entry> kEmpty;
  if (col < 0 || col >= n_vars_) return kEmpty;
  return vubs_[static_cast<std::size_t>(col)];
}

const std::vector<VariableBoundTable::Entry>& VariableBoundTable::vlbs(
    int col) const {
  static const std::vector<Entry> kEmpty;
  if (col < 0 || col >= n_vars_) return kEmpty;
  return vlbs_[static_cast<std::size_t>(col)];
}

bool VariableBoundTable::strengthen_var_bound(VarBound& bound,
                                              int multiplier) {
  if (!std::isfinite(bound.coef) || !std::isfinite(bound.constant) ||
      std::abs(bound.coef) >= kInf || std::abs(bound.constant) >= kInf) {
    return false;
  }
  constexpr double f0min = 0.005;
  constexpr double f0max = 0.995;
  constexpr double tiny = 1e-12;
  const double old_coef = bound.coef;
  const double old_constant = bound.constant;
  const double downrhs = std::floor(multiplier * bound.constant);
  const double f0 = multiplier * bound.constant - downrhs;
  if (f0 < f0min || f0 > f0max) return false;
  const double downaj = std::floor(-multiplier * bound.coef + tiny);
  const double fj = -multiplier * bound.coef - downaj;
  bound.constant = multiplier * downrhs;
  bound.coef =
      -multiplier * (downaj + std::max(fj - f0, 0.0) / (1.0 - f0));
  return std::abs(bound.coef - old_coef) > 1e-12 ||
         std::abs(bound.constant - old_constant) > 1e-12;
}

bool VariableBoundTable::add_vub(int col,
                                 int trigger_col,
                                 double coef,
                                 double constant,
                                 double col_upper_bound,
                                 bool col_is_integral) {
  ++stats_.vub_attempts;
  if (!valid_indices(col, trigger_col) || !std::isfinite(coef) ||
      !std::isfinite(constant) || !std::isfinite(col_upper_bound)) {
    return false;
  }
  VarBound candidate{coef, constant};
  if (col_is_integral) {
    ++stats_.mir_attempts;
    if (strengthen_var_bound(candidate, 1)) ++stats_.mir_strengthened;
    if (std::abs(candidate.coef) <= 1e-12) return false;
  }

  const double min_bound = candidate.min_value();
  if (min_bound >= col_upper_bound - 1e-9) {
    ++stats_.redundant;
    return false;
  }

  auto& entries = vubs_[static_cast<std::size_t>(col)];
  for (Entry& entry : entries) {
    if (entry.trigger_col != trigger_col) continue;
    if (min_bound < entry.bound.min_value() - 1e-9) {
      entry.bound = candidate;
      ++stats_.vub_replaced;
      return true;
    }
    return false;
  }
  entries.push_back(Entry{trigger_col, candidate});
  ++num_varbounds_;
  ++stats_.vub_accepted;
  return true;
}

bool VariableBoundTable::add_vlb(int col,
                                 int trigger_col,
                                 double coef,
                                 double constant,
                                 double col_lower_bound,
                                 bool col_is_integral) {
  ++stats_.vlb_attempts;
  if (!valid_indices(col, trigger_col) || !std::isfinite(coef) ||
      !std::isfinite(constant) || !std::isfinite(col_lower_bound)) {
    return false;
  }
  VarBound candidate{coef, constant};
  if (col_is_integral) {
    ++stats_.mir_attempts;
    if (strengthen_var_bound(candidate, -1)) ++stats_.mir_strengthened;
    if (std::abs(candidate.coef) <= 1e-12) return false;
  }

  const double max_bound = candidate.max_value();
  if (max_bound <= col_lower_bound + 1e-9) {
    ++stats_.redundant;
    return false;
  }

  auto& entries = vlbs_[static_cast<std::size_t>(col)];
  for (Entry& entry : entries) {
    if (entry.trigger_col != trigger_col) continue;
    if (max_bound > entry.bound.max_value() + 1e-9) {
      entry.bound = candidate;
      ++stats_.vlb_replaced;
      return true;
    }
    return false;
  }
  entries.push_back(Entry{trigger_col, candidate});
  ++num_varbounds_;
  ++stats_.vlb_accepted;
  return true;
}

std::uint64_t VariableBoundTable::export_to_implication_graph(
    BinaryImplicationGraph& implication_graph,
    double tol) const {
  std::uint64_t added = 0;
  for (int col = 0; col < n_vars_; ++col) {
    for (const Entry& entry : vubs_[static_cast<std::size_t>(col)]) {
      if (implication_graph.add_implication(
              entry.trigger_col, false, col, false, entry.bound.constant)) {
        ++added;
      }
      if (implication_graph.add_implication(
              entry.trigger_col, true, col, false,
              entry.bound.constant + entry.bound.coef)) {
        ++added;
      }
    }
    for (const Entry& entry : vlbs_[static_cast<std::size_t>(col)]) {
      if (implication_graph.add_implication(
              entry.trigger_col, false, col, true, entry.bound.constant)) {
        ++added;
      }
      if (implication_graph.add_implication(
              entry.trigger_col, true, col, true,
              entry.bound.constant + entry.bound.coef)) {
        ++added;
      }
    }
  }
  (void)tol;
  return added;
}

void RowPropagationIndex::build(
    int n_vars,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row) {
  const int n = std::max(0, n_vars);
  ineq_rows_by_var_.assign(static_cast<std::size_t>(n), {});
  eq_rows_by_var_.assign(static_cast<std::size_t>(n), {});

  for (int r = 0; r < static_cast<int>(A_row.rows()); ++r) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j >= 0 && j < n) {
        ineq_rows_by_var_[static_cast<std::size_t>(j)].push_back(r);
      }
    }
  }
  for (int r = 0; r < static_cast<int>(Aeq_row.rows()); ++r) {
    for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j >= 0 && j < n) {
        eq_rows_by_var_[static_cast<std::size_t>(j)].push_back(r);
      }
    }
  }
}

bool add_binary_implications_from_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const std::vector<BranchDomainLiteral>& clause,
    BinaryImplicationGraph& implication_graph) {
  if (clause.size() != 2) return false;

  auto decode_binary_trigger = [&](const BranchDomainLiteral& lit,
                                   int& var_idx,
                                   bool& value_one) -> bool {
    var_idx = lit.var_idx;
    if (var_idx < 0 || var_idx >= static_cast<int>(vars.size())) return false;
    if (vars[static_cast<std::size_t>(var_idx)].type != VarType::Binary) return false;
    if (lit.is_lb && lit.value >= 1.0 - 1e-9) {
      value_one = true;
      return true;
    }
    if (!lit.is_lb && lit.value <= 1e-9) {
      value_one = false;
      return true;
    }
    return false;
  };

  auto negate_bound_literal = [&](const BranchDomainLiteral& lit,
                                  BranchDomainLiteral& negated) -> bool {
    const int var_idx = lit.var_idx;
    if (var_idx < 0 || var_idx >= static_cast<int>(vars.size()) ||
        !std::isfinite(lit.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(var_idx)]);
    if (lit.is_lb) {
      negated = BranchDomainLiteral{
          var_idx,
          integer_var ? std::ceil(lit.value - 1e-9) - 1.0
                      : lit.value - 1e-7,
          false};
    } else {
      negated = BranchDomainLiteral{
          var_idx,
          integer_var ? std::floor(lit.value + 1e-9) + 1.0
                      : lit.value + 1e-7,
          true};
    }
    return true;
  };

  bool added = false;
  int trigger_var = -1;
  bool trigger_one = false;
  BranchDomainLiteral implied;
  if (decode_binary_trigger(clause[0], trigger_var, trigger_one) &&
      negate_bound_literal(clause[1], implied)) {
    added |= implication_graph.add_implication(trigger_var, trigger_one,
                                               implied.var_idx,
                                               implied.is_lb,
                                               implied.value);
  }
  if (decode_binary_trigger(clause[1], trigger_var, trigger_one) &&
      negate_bound_literal(clause[0], implied)) {
    added |= implication_graph.add_implication(trigger_var, trigger_one,
                                               implied.var_idx,
                                               implied.is_lb,
                                               implied.value);
  }
  return added;
}

namespace {

struct ReasonArena {
  std::vector<std::vector<BranchDomainLiteral>> clauses;

  ReasonArena() { clauses.emplace_back(); }

  int singleton(const BranchDomainLiteral& lit) {
    clauses.push_back({lit});
    return static_cast<int>(clauses.size()) - 1;
  }

  int add_clause(std::vector<BranchDomainLiteral> literals) {
    canonicalize_branch_literals(literals);
    if (literals.empty()) return 0;
    clauses.push_back(std::move(literals));
    return static_cast<int>(clauses.size()) - 1;
  }

  int merge_ids(const std::vector<int>& ids) {
    std::vector<BranchDomainLiteral> merged;
    for (int id : ids) {
      if (id <= 0 || id >= static_cast<int>(clauses.size())) continue;
      const auto& clause = clauses[static_cast<std::size_t>(id)];
      merged.insert(merged.end(), clause.begin(), clause.end());
    }
    canonicalize_branch_literals(merged);
    if (merged.empty()) return 0;
    clauses.push_back(std::move(merged));
    return static_cast<int>(clauses.size()) - 1;
  }

  const std::vector<BranchDomainLiteral>& get(int id) const {
    static const std::vector<BranchDomainLiteral> kEmpty;
    if (id <= 0 || id >= static_cast<int>(clauses.size())) return kEmpty;
    return clauses[static_cast<std::size_t>(id)];
  }
};

void merge_reason_literals(const ReasonArena& arena,
                           int a,
                           int b,
                           std::vector<BranchDomainLiteral>& out) {
  out.clear();
  const auto& ra = arena.get(a);
  const auto& rb = arena.get(b);
  out.insert(out.end(), ra.begin(), ra.end());
  out.insert(out.end(), rb.begin(), rb.end());
  canonicalize_branch_literals(out);
}

template <typename ConflictPoolT>
bool clause_propagation_step(const ConflictPoolT* conflict_pool,
                             const std::vector<VariableMeta>& vars,
                             Eigen::VectorXd& node_lb,
                             Eigen::VectorXd& node_ub,
                             ReasonArena& arena,
                             std::vector<int>& lb_reason,
                             std::vector<int>& ub_reason,
                             std::vector<BoundChangeInfo>& changes_out,
                             std::deque<int>& changed_vars,
                             std::vector<char>& queued_vars,
                             int branch_reason_count,
                             int* tightened,
                             std::vector<DomainReasonBound>* reason_bounds_out,
                             std::vector<DomainPropagationEvent>* events_out,
                             DomainPropagationFailure* failure_out) {
  if (conflict_pool == nullptr) return true;
  const bool track_reasons = !lb_reason.empty();
  std::vector<BoundChangeInfo> clause_changes;
  int local_tightened = 0;
  std::vector<DomainReasonBound> clause_reason_bounds;
  const bool ok = track_reasons
      ? conflict_pool->propagate_with_reasons(
            vars, node_lb, node_ub, &local_tightened, &clause_changes,
            &clause_reason_bounds)
      : conflict_pool->propagate(vars, node_lb, node_ub,
                                 &local_tightened, &clause_changes);
  if (tightened != nullptr) *tightened += local_tightened;
  for (int k = 0; k < static_cast<int>(clause_reason_bounds.size()); ++k) {
    auto& rb = clause_reason_bounds[static_cast<std::size_t>(k)];
    rb.trail_pos =
        branch_reason_count + static_cast<int>(changes_out.size()) + k;
    rb.depth = branch_reason_count;
    if (rb.bound.var_idx >= 0 && rb.bound.var_idx < static_cast<int>(vars.size())) {
      canonicalize_branch_literals(rb.reason);
      std::vector<int> ids;
      for (const auto& lit : rb.reason) {
        if (lit.var_idx < 0 ||
            lit.var_idx >= static_cast<int>(vars.size())) {
          continue;
        }
        const int rid = lit.is_lb
            ? lb_reason[static_cast<std::size_t>(lit.var_idx)]
            : ub_reason[static_cast<std::size_t>(lit.var_idx)];
        ids.push_back(rid > 0 ? rid : arena.singleton(lit));
      }
      const int rid = arena.merge_ids(ids);
      rb.reason = arena.get(rid);
      if (rb.bound.is_lb) {
        lb_reason[static_cast<std::size_t>(rb.bound.var_idx)] = rid;
      } else {
        ub_reason[static_cast<std::size_t>(rb.bound.var_idx)] = rid;
      }
    }
  }
  changes_out.insert(changes_out.end(), clause_changes.begin(), clause_changes.end());
  if (events_out != nullptr) {
    events_out->reserve(events_out->size() + clause_changes.size());
    for (const auto& change : clause_changes) {
      DomainPropagationEvent event{
          DomainPropagationSource::ConflictPool,
          change.var_idx,
          -1,
          -1,
          false,
          change.is_lb,
          change.old_value,
          change.new_value,
          0.0,
          0.0,
          0.0,
          {}};
      for (const auto& reason_bound : clause_reason_bounds) {
        if (reason_bound.bound.var_idx == change.var_idx &&
            reason_bound.bound.is_lb == change.is_lb &&
            std::abs(reason_bound.bound.value - change.new_value) <= 1e-9) {
          event.source_clause = reason_bound.has_source_conflict_clause
              ? reason_bound.source_conflict_clause
              : reason_bound.reason;
          break;
        }
      }
      events_out->push_back(std::move(event));
    }
  }
  if (reason_bounds_out != nullptr) {
    reason_bounds_out->insert(reason_bounds_out->end(),
                              clause_reason_bounds.begin(),
                              clause_reason_bounds.end());
  }
  if (!ok) {
    if (failure_out != nullptr) {
      failure_out->available = true;
      failure_out->event.source = DomainPropagationSource::ConflictPool;
      for (auto it = clause_changes.rbegin(); it != clause_changes.rend(); ++it) {
        const int j = it->var_idx;
        if (j >= 0 && j < node_lb.size() &&
            node_lb[j] > node_ub[j] + 1e-9) {
          failure_out->event = DomainPropagationEvent{
              DomainPropagationSource::ConflictPool,
              j,
              -1,
              -1,
              false,
              it->is_lb,
              it->old_value,
              it->new_value,
              0.0,
              0.0,
              0.0,
              {}};
          break;
        }
      }
    }
    return false;
  }
  for (const auto& bc : clause_changes) {
    if (bc.var_idx >= 0 && bc.var_idx < static_cast<int>(queued_vars.size()) &&
        !queued_vars[static_cast<std::size_t>(bc.var_idx)]) {
      queued_vars[static_cast<std::size_t>(bc.var_idx)] = 1;
      changed_vars.push_back(bc.var_idx);
    }
  }
  return true;
}

template <typename ConflictPoolT>
bool propagate_node_domain_impl(
    const LPModel& lp,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
    const RowPropagationIndex& row_index,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    int max_rounds,
    const ConflictPoolT* conflict_pool,
    const CliqueTable* clique_table,
    const BinaryImplicationGraph* implication_graph,
    std::vector<BoundChangeInfo>& changes_out,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    std::vector<BranchDomainLiteral>* learned_conflict,
    int* total_tightened,
    std::vector<DomainReasonBound>* reason_bounds_out,
    const std::vector<DomainReasonBound>* existing_reason_bounds,
    std::vector<DomainPropagationEvent>* propagation_events,
    DomainPropagationFailure* propagation_failure,
    const std::function<bool()>* stop_requested,
    bool* interrupted) {
  if (interrupted != nullptr) *interrupted = false;
  if (propagation_failure != nullptr) {
    *propagation_failure = DomainPropagationFailure{};
  }
  PropagationProfile profile;
  profile.calls = 1;
  const auto _prop_t0 = std::chrono::steady_clock::now();
  const int n = static_cast<int>(lp.vars.size());
  const bool track_reasons =
      reason_bounds_out != nullptr || learned_conflict != nullptr;
  ReasonArena arena;
  std::vector<int> lb_reason(
      track_reasons ? static_cast<std::size_t>(n) : 0, 0);
  std::vector<int> ub_reason(
      track_reasons ? static_cast<std::size_t>(n) : 0, 0);
  std::deque<int> changed_vars;
  std::vector<char> queued_vars(static_cast<std::size_t>(n), 0);
  std::deque<int> pending_ineq_rows;
  std::deque<int> pending_eq_rows;
  std::vector<char> queued_ineq(static_cast<std::size_t>(A_row.rows()), 0);
  std::vector<char> queued_eq(static_cast<std::size_t>(Aeq_row.rows()), 0);

  auto enqueue_var = [&](int j) {
    if (j < 0 || j >= n) return;
    if (!queued_vars[static_cast<std::size_t>(j)]) {
      queued_vars[static_cast<std::size_t>(j)] = 1;
      changed_vars.push_back(j);
    }
  };

  for (const auto& lit : branch_reasons) {
    const int rid = track_reasons ? arena.singleton(lit) : 0;
    if (lit.var_idx >= 0 && lit.var_idx < n) {
      if (track_reasons) {
        if (lit.is_lb) lb_reason[static_cast<std::size_t>(lit.var_idx)] = rid;
        else ub_reason[static_cast<std::size_t>(lit.var_idx)] = rid;
      }
      enqueue_var(lit.var_idx);
    }
  }
  if (existing_reason_bounds != nullptr) {
    for (const auto& rb : *existing_reason_bounds) {
      const int j = rb.bound.var_idx;
      if (j < 0 || j >= n || !std::isfinite(rb.bound.value)) continue;
      const bool active = rb.bound.is_lb
          ? (node_lb[j] >= rb.bound.value - 1e-9)
          : (node_ub[j] <= rb.bound.value + 1e-9);
      if (!active) continue;
      if (track_reasons) {
        const int rid = !rb.reason.empty() ? arena.add_clause(rb.reason) : 0;
        if (rb.bound.is_lb) {
          const int old = lb_reason[static_cast<std::size_t>(j)];
          if (old <= 0 || rb.bound.value >= node_lb[j] - 1e-9) {
            lb_reason[static_cast<std::size_t>(j)] = rid;
          }
        } else {
          const int old = ub_reason[static_cast<std::size_t>(j)];
          if (old <= 0 || rb.bound.value <= node_ub[j] + 1e-9) {
            ub_reason[static_cast<std::size_t>(j)] = rid;
          }
        }
      }
      enqueue_var(j);
    }
  }
  const std::size_t input_change_count = changes_out.size();
  for (std::size_t ci = 0; ci < input_change_count; ++ci) {
    const int j = changes_out[ci].var_idx;
    if (j >= 0 && j < n) {
      if (track_reasons) {
        if (changes_out[ci].is_lb) {
          int& rid = lb_reason[static_cast<std::size_t>(j)];
          if (rid <= 0) {
            rid = arena.singleton(BranchDomainLiteral{j, node_lb[j], true});
          }
        } else {
          int& rid = ub_reason[static_cast<std::size_t>(j)];
          if (rid <= 0) {
            rid = arena.singleton(BranchDomainLiteral{j, node_ub[j], false});
          }
        }
      }
      enqueue_var(j);
    }
  }

  int local_tightened = 0;
  int conflict_var = -1;
  const int row_round_budget = std::max(0, max_rounds);
  int row_rounds_used = 0;
  const int max_passes = std::max(4, 2 * row_round_budget + 4);
  const std::uint64_t full_scan_rows =
      static_cast<std::uint64_t>(A_row.rows()) + static_cast<std::uint64_t>(Aeq_row.rows());
  std::uint64_t stop_poll_counter = 0;
  auto propagation_stop_requested = [&]() {
    // Amortized deadline polling; derivation and measured first-node overrun in
    // docs/native_milp_root_source_eligibility_2026-08-13.md.
    if (stop_requested == nullptr || ((stop_poll_counter++ & 63U) != 0U)) {
      return false;
    }
    if (!(*stop_requested)()) return false;
    if (interrupted != nullptr) *interrupted = true;
    return true;
  };

  auto finalize_profile = [&]() {
    profile.bound_tightenings = static_cast<std::uint64_t>(std::max(0, local_tightened));
    profile.reason_clauses_materialized =
        track_reasons && !arena.clauses.empty()
            ? static_cast<std::uint64_t>(arena.clauses.size() - 1)
            : 0;
    const auto _prop_t1 = std::chrono::steady_clock::now();
    profile.wall_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(_prop_t1 - _prop_t0).count());
    accumulate_propagation_profile(profile);
  };

  struct LinearProofAudit {
    BranchDomainLiteral source_conflict_literal;
    double proof_activity{0.0};
    double proof_required_activity{0.0};
  };

  auto apply_bound = [&](int j, bool is_lb, double new_val, int reason_id,
                         DomainPropagationSource source, int source_row,
                         int trigger_variable, bool trigger_value_one,
                         double source_coefficient, double source_rhs,
                         double source_activity_bound,
                         const LinearProofAudit* proof_audit = nullptr) -> bool {
    if (j < 0 || j >= n) return true;
    if (is_integer_type(lp.vars[static_cast<std::size_t>(j)])) {
      new_val = is_lb ? std::ceil(new_val - 1e-9) : std::floor(new_val + 1e-9);
    }
    if (is_lb) {
      if (new_val > node_lb[j] + bound_improvement_tolerance(
                                          1e-9, node_lb[j], new_val)) {
        const double old_value = node_lb[j];
        changes_out.push_back(
            {j, new_val - node_lb[j], true, node_lb[j], new_val});
        node_lb[j] = new_val;
        if (propagation_events != nullptr) {
          propagation_events->push_back(DomainPropagationEvent{
              source, j, source_row, trigger_variable, trigger_value_one,
              true, old_value, new_val, source_coefficient, source_rhs,
              source_activity_bound, {}});
        }
        const BranchDomainLiteral bound_lit{j, new_val, true};
        if (reason_bounds_out != nullptr) {
          auto reason = arena.get(reason_id);
          const int trail_pos =
              static_cast<int>(branch_reasons.size()) + local_tightened;
          DomainReasonBound rb;
          rb.bound = bound_lit;
          rb.reason = reason;
          rb.trail_pos = trail_pos;
          rb.depth = static_cast<int>(branch_reasons.size());
          if (proof_audit != nullptr &&
              proof_audit->source_conflict_literal.var_idx == j &&
              proof_audit->source_conflict_literal.is_lb != is_lb &&
              std::isfinite(proof_audit->source_conflict_literal.value) &&
              std::isfinite(proof_audit->proof_activity) &&
              std::isfinite(proof_audit->proof_required_activity)) {
            rb.source_conflict_literal =
                proof_audit->source_conflict_literal;
            rb.has_source_conflict_literal = true;
            rb.source_conflict_clause = rb.reason;
            rb.source_conflict_clause.push_back(rb.source_conflict_literal);
            canonicalize_branch_literals(rb.source_conflict_clause);
            rb.has_source_conflict_clause = true;
            rb.has_proof_activity_audit = true;
            rb.proof_activity = proof_audit->proof_activity;
            rb.proof_required_activity =
                proof_audit->proof_required_activity;
            rb.proof_activity_margin =
                rb.proof_activity - rb.proof_required_activity;
          }
          reason_bounds_out->push_back(std::move(rb));
        }
        if (track_reasons) {
          lb_reason[static_cast<std::size_t>(j)] = reason_id;
        }
        ++local_tightened;
        enqueue_var(j);
      }
    } else {
      if (new_val < node_ub[j] - bound_improvement_tolerance(
                                          1e-9, node_ub[j], new_val)) {
        const double old_value = node_ub[j];
        changes_out.push_back(
            {j, new_val - node_ub[j], false, node_ub[j], new_val});
        node_ub[j] = new_val;
        if (propagation_events != nullptr) {
          propagation_events->push_back(DomainPropagationEvent{
              source, j, source_row, trigger_variable, trigger_value_one,
              false, old_value, new_val, source_coefficient, source_rhs,
              source_activity_bound, {}});
        }
        const BranchDomainLiteral bound_lit{j, new_val, false};
        if (reason_bounds_out != nullptr) {
          auto reason = arena.get(reason_id);
          const int trail_pos =
              static_cast<int>(branch_reasons.size()) + local_tightened;
          DomainReasonBound rb;
          rb.bound = bound_lit;
          rb.reason = reason;
          rb.trail_pos = trail_pos;
          rb.depth = static_cast<int>(branch_reasons.size());
          if (proof_audit != nullptr &&
              proof_audit->source_conflict_literal.var_idx == j &&
              proof_audit->source_conflict_literal.is_lb != is_lb &&
              std::isfinite(proof_audit->source_conflict_literal.value) &&
              std::isfinite(proof_audit->proof_activity) &&
              std::isfinite(proof_audit->proof_required_activity)) {
            rb.source_conflict_literal =
                proof_audit->source_conflict_literal;
            rb.has_source_conflict_literal = true;
            rb.source_conflict_clause = rb.reason;
            rb.source_conflict_clause.push_back(rb.source_conflict_literal);
            canonicalize_branch_literals(rb.source_conflict_clause);
            rb.has_source_conflict_clause = true;
            rb.has_proof_activity_audit = true;
            rb.proof_activity = proof_audit->proof_activity;
            rb.proof_required_activity =
                proof_audit->proof_required_activity;
            rb.proof_activity_margin =
                rb.proof_activity - rb.proof_required_activity;
          }
          reason_bounds_out->push_back(std::move(rb));
        }
        if (track_reasons) {
          ub_reason[static_cast<std::size_t>(j)] = reason_id;
        }
        ++local_tightened;
        enqueue_var(j);
      }
    }
    if (node_lb[j] > node_ub[j] + bound_improvement_tolerance(
                                        1e-9, node_lb[j], node_ub[j])) {
      conflict_var = j;
      if (propagation_failure != nullptr) {
        propagation_failure->available = true;
        propagation_failure->event = DomainPropagationEvent{
            source, j, source_row, trigger_variable, trigger_value_one,
            is_lb, is_lb ? node_lb[j] : node_ub[j], new_val,
            source_coefficient, source_rhs, source_activity_bound, {}};
        if (propagation_events != nullptr && !propagation_events->empty() &&
            propagation_events->back().variable == j) {
          propagation_failure->event = propagation_events->back();
        }
      }
      return false;
    }
    return true;
  };

  auto binary_like_domain = [&](int j) {
    if (j < 0 || j >= n) return false;
    const auto& var = lp.vars[static_cast<std::size_t>(j)];
    if (var.type == VarType::Binary) return true;
    return std::isfinite(node_lb[j]) && std::isfinite(node_ub[j]) &&
           node_lb[j] >= -1e-9 && node_ub[j] <= 1.0 + 1e-9 &&
           std::isfinite(var.lb) && std::isfinite(var.ub) &&
           var.lb >= -1e-9 && var.ub <= 1.0 + 1e-9;
  };
  auto normalized_bound_value = [&](int j, bool is_lb, double value) {
    if (j >= 0 && j < n &&
        is_integer_type(lp.vars[static_cast<std::size_t>(j)])) {
      return is_lb ? std::ceil(value - 1e-9) : std::floor(value + 1e-9);
    }
    return value;
  };
  auto opposite_source_literal = [&](int j, bool target_is_lb,
                                     double target_value) {
    const bool integer_var =
        j >= 0 && j < n &&
        is_integer_type(lp.vars[static_cast<std::size_t>(j)]);
    if (target_is_lb) {
      return BranchDomainLiteral{
          j,
          integer_var ? std::ceil(target_value - 1e-9) - 1.0
                      : target_value - 1e-7,
          false};
    }
    return BranchDomainLiteral{
        j,
        integer_var ? std::floor(target_value + 1e-9) + 1.0
                    : target_value + 1e-7,
        true};
  };

  if (!clause_propagation_step(conflict_pool, lp.vars, node_lb, node_ub,
                               arena, lb_reason, ub_reason,
                               changes_out, changed_vars, queued_vars,
                               static_cast<int>(branch_reasons.size()),
                               &local_tightened, reason_bounds_out,
                               propagation_events, propagation_failure)) {
    if (learned_conflict != nullptr) *learned_conflict = branch_reasons;
    if (total_tightened != nullptr) *total_tightened += local_tightened;
    finalize_profile();
    return false;
  }

  for (int pass = 0; pass < max_passes; ++pass) {
    if (propagation_stop_requested()) break;
    if (changed_vars.empty() && pending_ineq_rows.empty() && pending_eq_rows.empty()) {
      break;
    }
    ++profile.active_passes;

    while (!changed_vars.empty()) {
      if (propagation_stop_requested()) break;
      const int j = changed_vars.front();
      changed_vars.pop_front();
      queued_vars[static_cast<std::size_t>(j)] = 0;
      ++profile.changed_var_visits;

      if (clique_table != nullptr && !clique_table->empty() &&
          binary_like_domain(j)) {
        for (int value_one_int = 0; value_one_int <= 1; ++value_one_int) {
          const bool value_one = (value_one_int == 1);
          const bool active = value_one ? (node_lb[j] >= 1.0 - 1e-9)
                                        : (node_ub[j] <= 1e-9);
          if (!active) continue;
          const int trigger_rid =
              track_reasons
                  ? (value_one ? lb_reason[static_cast<std::size_t>(j)]
                               : ub_reason[static_cast<std::size_t>(j)])
                  : 0;
          auto lit_rng = clique_table->literal_neighbours(j, value_one);
          for (const int* p = lit_rng.first; p != lit_rng.second; ++p) {
            const int forbidden = *p;
            const int implied_col = forbidden / 2;
            const bool forbidden_one = (forbidden % 2) != 0;
            const int rid =
                track_reasons ? arena.merge_ids({trigger_rid}) : 0;
            if (forbidden_one) {
              if (!apply_bound(implied_col, false, 0.0, rid,
                               DomainPropagationSource::Clique, -1, j,
                               value_one, 0.0, 0.0, 0.0)) break;
            } else {
              if (!apply_bound(implied_col, true, 1.0, rid,
                               DomainPropagationSource::Clique, -1, j,
                               value_one, 0.0, 0.0, 0.0)) break;
            }
          }
          if (conflict_var >= 0) break;

          if (value_one) {
            auto rng = clique_table->neighbours(j);
            for (const int* p = rng.first; p != rng.second; ++p) {
              const int rid =
                  track_reasons ? arena.merge_ids({trigger_rid}) : 0;
              if (!apply_bound(*p, false, 0.0, rid,
                               DomainPropagationSource::Clique, -1, j,
                               value_one, 0.0, 0.0, 0.0)) break;
            }
          }
          if (conflict_var >= 0) break;
        }
        if (conflict_var >= 0) break;
      }

      if (implication_graph != nullptr && !implication_graph->empty() &&
          binary_like_domain(j)) {
        for (int value_one_int = 0; value_one_int <= 1; ++value_one_int) {
          const bool value_one = (value_one_int == 1);
          const bool active = value_one ? (node_lb[j] >= 1.0 - 1e-9)
                                        : (node_ub[j] <= 1e-9);
          if (!active) continue;
          auto rng = implication_graph->implications(j, value_one);
          const int trigger_rid =
              track_reasons
                  ? (value_one ? lb_reason[static_cast<std::size_t>(j)]
                               : ub_reason[static_cast<std::size_t>(j)])
                  : 0;
          for (const auto* arc = rng.first; arc != rng.second; ++arc) {
            const int rid =
                track_reasons ? arena.merge_ids({trigger_rid}) : 0;
            if (!apply_bound(arc->var_idx, arc->is_lb, arc->value, rid,
                             DomainPropagationSource::Implication, -1, j,
                             value_one, 0.0, 0.0, 0.0)) break;
          }
          if (conflict_var >= 0) break;
        }
        if (conflict_var >= 0) break;
      }

      if (row_rounds_used < row_round_budget) {
        for (int r : row_index.ineq_rows_for(j)) {
          if (!queued_ineq[static_cast<std::size_t>(r)]) {
            queued_ineq[static_cast<std::size_t>(r)] = 1;
            pending_ineq_rows.push_back(r);
          }
        }
        for (int r : row_index.eq_rows_for(j)) {
          if (!queued_eq[static_cast<std::size_t>(r)]) {
            queued_eq[static_cast<std::size_t>(r)] = 1;
            pending_eq_rows.push_back(r);
          }
        }
      }
    }
    if ((interrupted != nullptr && *interrupted) || conflict_var >= 0) break;

    const bool process_row_round =
        row_rounds_used < row_round_budget &&
        (!pending_ineq_rows.empty() || !pending_eq_rows.empty());
    if (process_row_round) {
      ++row_rounds_used;
      profile.baseline_full_scan_rows += full_scan_rows;
    }

    while (!pending_ineq_rows.empty()) {
      if (propagation_stop_requested()) break;
      const int r = pending_ineq_rows.front();
      pending_ineq_rows.pop_front();
      queued_ineq[static_cast<std::size_t>(r)] = 0;
      ++profile.ineq_rows_visited;

      auto propagate_upper_side = [&](double row_sign, double rhs) {
        if (!std::isfinite(rhs) || conflict_var >= 0) return;
        StableActivitySum min_activity;
        bool has_inf = false;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          if (a > 0.0) {
            if (!std::isfinite(node_lb[j])) { has_inf = true; break; }
            min_activity.add_product(a, node_lb[j]);
          } else {
            if (!std::isfinite(node_ub[j])) { has_inf = true; break; }
            min_activity.add_product(a, node_ub[j]);
          }
        }
        if (has_inf) return;

        // Hoist ids buffer outside the inner loop to avoid per-entry heap
        // allocation (the loop body runs once per non-zero in the row).
        std::vector<int> ids;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_lb[j] : node_ub[j];
          const double residual = min_activity.upper_residual(
              rhs, a, activity_bound, 1e-9);
          int rid = 0;
          if (track_reasons) {
            ids.clear();
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(
                     A_row, r);
                 jt; ++jt) {
              const int k = static_cast<int>(jt.col());
              const double ak = row_sign * jt.value();
              if (k == j || std::abs(ak) <= 1e-15) continue;
              const int source_reason =
                  (ak > 0.0) ? lb_reason[static_cast<std::size_t>(k)]
                             : ub_reason[static_cast<std::size_t>(k)];
              if (source_reason > 0) ids.push_back(source_reason);
            }
            rid = arena.merge_ids(ids);
          }
          if (a > 0.0) {
            const double new_ub = normalized_bound_value(j, false, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, false, new_ub);
            const LinearProofAudit audit{
                source,
                min_activity.replacing_product(
                    a, activity_bound, source.value),
                rhs};
            if (!apply_bound(j, false, new_ub, rid,
                             DomainPropagationSource::InequalityRow, r, -1,
                             false, a, rhs, activity_bound, &audit)) break;
          } else {
            const double new_lb = normalized_bound_value(j, true, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, true, new_lb);
            const LinearProofAudit audit{
                source,
                min_activity.replacing_product(
                    a, activity_bound, source.value),
                rhs};
            if (!apply_bound(j, true, new_lb, rid,
                             DomainPropagationSource::InequalityRow, r, -1,
                             false, a, rhs, activity_bound, &audit)) break;
          }
        }
      };
      propagate_upper_side(1.0, lp.b[r]);
      const double lhs = lp_row_lhs_or_neg_inf(lp, r);
      if (std::isfinite(lhs)) propagate_upper_side(-1.0, -lhs);
      if (conflict_var >= 0) break;
    }
    if ((interrupted != nullptr && *interrupted) || conflict_var >= 0) break;

    while (!pending_eq_rows.empty()) {
      if (propagation_stop_requested()) break;
      const int r = pending_eq_rows.front();
      pending_eq_rows.pop_front();
      queued_eq[static_cast<std::size_t>(r)] = 0;
      ++profile.eq_rows_visited;

      StableActivitySum min_activity;
      StableActivitySum max_activity;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(node_lb[j])) has_inf_min = true;
          else min_activity.add_product(a, node_lb[j]);
          if (!std::isfinite(node_ub[j])) has_inf_max = true;
          else max_activity.add_product(a, node_ub[j]);
        } else {
          if (!std::isfinite(node_ub[j])) has_inf_min = true;
          else min_activity.add_product(a, node_ub[j]);
          if (!std::isfinite(node_lb[j])) has_inf_max = true;
          else max_activity.add_product(a, node_lb[j]);
        }
      }

      if (!has_inf_min) {
        std::vector<int> ids;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_lb[j] : node_ub[j];
          const double residual = min_activity.upper_residual(
              lp.beq[r], a, activity_bound, 1e-9);
          int rid = 0;
          if (track_reasons) {
            ids.clear();
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(
                     Aeq_row, r);
                 jt; ++jt) {
              const int k = static_cast<int>(jt.col());
              const double ak = jt.value();
              if (k == j || std::abs(ak) <= 1e-15) continue;
              const int source_reason =
                  (ak > 0.0) ? lb_reason[static_cast<std::size_t>(k)]
                             : ub_reason[static_cast<std::size_t>(k)];
              if (source_reason > 0) ids.push_back(source_reason);
            }
            rid = arena.merge_ids(ids);
          }
          if (a > 0.0) {
            const double new_ub = normalized_bound_value(j, false, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, false, new_ub);
            const LinearProofAudit audit{
                source,
                min_activity.replacing_product(
                    a, activity_bound, source.value),
                lp.beq[r]};
            if (!apply_bound(j, false, new_ub, rid,
                             DomainPropagationSource::EqualityRow, r, -1,
                             false, a, lp.beq[r], activity_bound, &audit)) break;
          } else {
            const double new_lb = normalized_bound_value(j, true, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, true, new_lb);
            const LinearProofAudit audit{
                source,
                min_activity.replacing_product(
                    a, activity_bound, source.value),
                lp.beq[r]};
            if (!apply_bound(j, true, new_lb, rid,
                             DomainPropagationSource::EqualityRow, r, -1,
                             false, a, lp.beq[r], activity_bound, &audit)) break;
          }
        }
      }
      if (conflict_var >= 0) break;

      if (!has_inf_max) {
        std::vector<int> ids;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double activity_bound =
              (a > 0.0) ? node_ub[j] : node_lb[j];
          const double residual = max_activity.lower_residual(
              lp.beq[r], a, activity_bound, 1e-9);
          int rid = 0;
          if (track_reasons) {
            ids.clear();
            for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(
                     Aeq_row, r);
                 jt; ++jt) {
              const int k = static_cast<int>(jt.col());
              const double ak = jt.value();
              if (k == j || std::abs(ak) <= 1e-15) continue;
              const int source_reason =
                  (ak > 0.0) ? ub_reason[static_cast<std::size_t>(k)]
                             : lb_reason[static_cast<std::size_t>(k)];
              if (source_reason > 0) ids.push_back(source_reason);
            }
            rid = arena.merge_ids(ids);
          }
          if (a > 0.0) {
            const double new_lb = normalized_bound_value(j, true, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, true, new_lb);
            const double source_activity = max_activity.replacing_product(
                a, activity_bound, source.value);
            const LinearProofAudit audit{source, -source_activity, -lp.beq[r]};
            if (!apply_bound(j, true, new_lb, rid,
                             DomainPropagationSource::EqualityRow, r, -1,
                             false, a, lp.beq[r], activity_bound, &audit)) break;
          } else {
            const double new_ub = normalized_bound_value(j, false, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, false, new_ub);
            const double source_activity = max_activity.replacing_product(
                a, activity_bound, source.value);
            const LinearProofAudit audit{source, -source_activity, -lp.beq[r]};
            if (!apply_bound(j, false, new_ub, rid,
                             DomainPropagationSource::EqualityRow, r, -1,
                             false, a, lp.beq[r], activity_bound, &audit)) break;
          }
        }
      }
      if (conflict_var >= 0) break;
    }
    if ((interrupted != nullptr && *interrupted) || conflict_var >= 0) break;

    if (!clause_propagation_step(conflict_pool, lp.vars, node_lb, node_ub,
                                 arena, lb_reason, ub_reason,
                                 changes_out, changed_vars, queued_vars,
                                 static_cast<int>(branch_reasons.size()),
                                 &local_tightened, reason_bounds_out,
                                 propagation_events, propagation_failure)) {
      if (learned_conflict != nullptr) *learned_conflict = branch_reasons;
      if (total_tightened != nullptr) *total_tightened += local_tightened;
      finalize_profile();
      return false;
    }
  }

  if (conflict_var >= 0) {
    if (learned_conflict != nullptr) {
      merge_reason_literals(arena,
                            lb_reason[static_cast<std::size_t>(conflict_var)],
                            ub_reason[static_cast<std::size_t>(conflict_var)],
                            *learned_conflict);
      if (learned_conflict->empty()) {
        *learned_conflict = branch_reasons;
      }
    }
    if (total_tightened != nullptr) *total_tightened += local_tightened;
    finalize_profile();
    return false;
  }

  if (total_tightened != nullptr) *total_tightened += local_tightened;
  finalize_profile();
  return bounds_consistent(node_lb, node_ub);
}

}  // namespace

const char* domain_propagation_source_name(
    DomainPropagationSource source) noexcept {
  switch (source) {
    case DomainPropagationSource::ConflictPool: return "conflict_pool";
    case DomainPropagationSource::Clique: return "clique";
    case DomainPropagationSource::Implication: return "implication";
    case DomainPropagationSource::InequalityRow: return "inequality_row";
    case DomainPropagationSource::EqualityRow: return "equality_row";
    case DomainPropagationSource::Unknown: return "unknown";
  }
  return "unknown";
}

bool propagate_node_domain(
    const LPModel& lp,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
    const RowPropagationIndex& row_index,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    int max_rounds,
    const ConflictPool* conflict_pool,
    const CliqueTable* clique_table,
    const BinaryImplicationGraph* implication_graph,
    std::vector<BoundChangeInfo>& changes_out,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    std::vector<BranchDomainLiteral>* learned_conflict,
    int* total_tightened,
    std::vector<DomainReasonBound>* reason_bounds_out,
    const std::vector<DomainReasonBound>* existing_reason_bounds,
    std::vector<DomainPropagationEvent>* propagation_events,
    DomainPropagationFailure* propagation_failure,
    const std::function<bool()>* stop_requested,
    bool* interrupted) {
  return propagate_node_domain_impl(lp, A_row, Aeq_row, row_index, node_lb, node_ub,
                                    max_rounds, conflict_pool, clique_table,
                                    implication_graph, changes_out,
                                    branch_reasons, learned_conflict,
                                    total_tightened, reason_bounds_out,
                                    existing_reason_bounds, propagation_events,
                                    propagation_failure, stop_requested,
                                    interrupted);
}

bool propagate_node_domain(
    const LPModel& lp,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& A_row,
    const Eigen::SparseMatrix<double, Eigen::RowMajor>& Aeq_row,
    const RowPropagationIndex& row_index,
    Eigen::VectorXd& node_lb,
    Eigen::VectorXd& node_ub,
    int max_rounds,
    const SharedConflictPool* conflict_pool,
    const CliqueTable* clique_table,
    const BinaryImplicationGraph* implication_graph,
    std::vector<BoundChangeInfo>& changes_out,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    std::vector<BranchDomainLiteral>* learned_conflict,
    int* total_tightened,
    std::vector<DomainReasonBound>* reason_bounds_out,
    const std::vector<DomainReasonBound>* existing_reason_bounds,
    std::vector<DomainPropagationEvent>* propagation_events,
    DomainPropagationFailure* propagation_failure,
    const std::function<bool()>* stop_requested,
    bool* interrupted) {
  return propagate_node_domain_impl(lp, A_row, Aeq_row, row_index, node_lb, node_ub,
                                    max_rounds, conflict_pool, clique_table,
                                    implication_graph, changes_out,
                                    branch_reasons, learned_conflict,
                                    total_tightened, reason_bounds_out,
                                    existing_reason_bounds, propagation_events,
                                    propagation_failure, stop_requested,
                                    interrupted);
}

}  // namespace mipsolvers::engine::detail
