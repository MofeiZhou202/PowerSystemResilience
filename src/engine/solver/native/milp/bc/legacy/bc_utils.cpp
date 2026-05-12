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
std::atomic<std::uint64_t> g_prop_wall_ns{0};

void accumulate_propagation_profile(const PropagationProfile& profile) {
  g_prop_calls.fetch_add(profile.calls, std::memory_order_relaxed);
  g_prop_active_passes.fetch_add(profile.active_passes, std::memory_order_relaxed);
  g_prop_changed_var_visits.fetch_add(profile.changed_var_visits, std::memory_order_relaxed);
  g_prop_ineq_rows_visited.fetch_add(profile.ineq_rows_visited, std::memory_order_relaxed);
  g_prop_eq_rows_visited.fetch_add(profile.eq_rows_visited, std::memory_order_relaxed);
  g_prop_baseline_full_scan_rows.fetch_add(profile.baseline_full_scan_rows, std::memory_order_relaxed);
  g_prop_bound_tightenings.fetch_add(profile.bound_tightenings, std::memory_order_relaxed);
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

int milp_presolve(LPModel& lp, int max_rounds) {
  const int n = static_cast<int>(lp.vars.size());
  int total_fixed = 0;

  // Build row-major copies for efficient row iteration.
  Eigen::SparseMatrix<double, Eigen::RowMajor> A_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> Aeq_row = lp.Aeq;

  for (int round = 0; round < max_rounds; ++round) {
    int round_fixed = 0;
    // Recompute after possible redundant row removal in prior round.
    const int m_ineq = static_cast<int>(lp.A.rows());
    const int m_eq = static_cast<int>(lp.Aeq.rows());

    // ── 1. Bound tightening on inequality rows: a^T x ≤ b ──
    for (int r = 0; r < m_ineq; ++r) {
      double min_activity = 0.0;
      bool has_inf = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(lp.vars[j].lb)) { has_inf = true; break; }
          min_activity += a * lp.vars[j].lb;
        } else {
          if (!std::isfinite(lp.vars[j].ub)) { has_inf = true; break; }
          min_activity += a * lp.vars[j].ub;
        }
      }
      if (has_inf) continue;

      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        const double contrib = (a > 0.0) ? a * lp.vars[j].lb : a * lp.vars[j].ub;
        const double residual = lp.b[r] - (min_activity - contrib);
        if (a > 0.0) {
          const double new_ub = residual / a;
          if (new_ub < lp.vars[j].ub - 1e-9) {
            lp.vars[j].ub = is_integer_type(lp.vars[j]) ? std::floor(new_ub + 1e-9) : new_ub;
          }
        } else {
          const double new_lb = residual / a;
          if (new_lb > lp.vars[j].lb + 1e-9) {
            lp.vars[j].lb = is_integer_type(lp.vars[j]) ? std::ceil(new_lb - 1e-9) : new_lb;
          }
        }
      }
    }

    // ── 2. Bound tightening on equality rows: a^T x = b_eq ──
    // Equalities give both upper and lower bounds (2x stronger).
    for (int r = 0; r < m_eq; ++r) {
      double min_activity = 0.0, max_activity = 0.0;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(lp.vars[j].lb)) has_inf_min = true;
          else min_activity += a * lp.vars[j].lb;
          if (!std::isfinite(lp.vars[j].ub)) has_inf_max = true;
          else max_activity += a * lp.vars[j].ub;
        } else {
          if (!std::isfinite(lp.vars[j].ub)) has_inf_min = true;
          else min_activity += a * lp.vars[j].ub;
          if (!std::isfinite(lp.vars[j].lb)) has_inf_max = true;
          else max_activity += a * lp.vars[j].lb;
        }
      }

      // From a^T x = beq, using min/max activity:
      // For each variable j with coefficient a_j:
      //   From below (min_activity side): tighten UB if a>0, LB if a<0
      //   From above (max_activity side): tighten LB if a>0, UB if a<0
      if (!has_inf_min) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * lp.vars[j].lb : a * lp.vars[j].ub;
          const double residual = lp.beq[r] - (min_activity - contrib);
          if (a > 0.0) {
            const double new_ub = residual / a;
            if (new_ub < lp.vars[j].ub - 1e-9) {
              lp.vars[j].ub = is_integer_type(lp.vars[j]) ? std::floor(new_ub + 1e-9) : new_ub;
            }
          } else {
            const double new_lb = residual / a;
            if (new_lb > lp.vars[j].lb + 1e-9) {
              lp.vars[j].lb = is_integer_type(lp.vars[j]) ? std::ceil(new_lb - 1e-9) : new_lb;
            }
          }
        }
      }
      if (!has_inf_max) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * lp.vars[j].ub : a * lp.vars[j].lb;
          const double residual = lp.beq[r] - (max_activity - contrib);
          if (a > 0.0) {
            const double new_lb = residual / a;
            if (new_lb > lp.vars[j].lb + 1e-9) {
              lp.vars[j].lb = is_integer_type(lp.vars[j]) ? std::ceil(new_lb - 1e-9) : new_lb;
            }
          } else {
            const double new_ub = residual / a;
            if (new_ub < lp.vars[j].ub - 1e-9) {
              lp.vars[j].ub = is_integer_type(lp.vars[j]) ? std::floor(new_ub + 1e-9) : new_ub;
            }
          }
        }
      }
    }

    // ── 2b. Singleton row handling ──
    // Inequality row with exactly 1 nonzero: a_j * x_j ≤ b → tighten bound.
    // Equality row with exactly 1 nonzero: a_j * x_j = b → fix x_j = b/a_j.
    {
      // Mark inequality singleton rows for removal after tightening.
      std::vector<bool> ineq_singleton(static_cast<size_t>(m_ineq), false);
      for (int r = 0; r < m_ineq; ++r) {
        int nnz = 0;
        int col_j = -1;
        double coeff = 0.0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          if (std::abs(it.value()) > 1e-15) {
            ++nnz;
            col_j = static_cast<int>(it.col());
            coeff = it.value();
            if (nnz > 1) break;
          }
        }
        if (nnz == 1 && col_j >= 0) {
          // a * x_j ≤ b → x_j ≤ b/a (if a>0) or x_j ≥ b/a (if a<0)
          const double bound = lp.b[r] / coeff;
          if (coeff > 0.0) {
            if (bound < lp.vars[col_j].ub - 1e-9) {
              lp.vars[col_j].ub = is_integer_type(lp.vars[col_j])
                  ? std::floor(bound + 1e-9) : bound;
            }
          } else {
            if (bound > lp.vars[col_j].lb + 1e-9) {
              lp.vars[col_j].lb = is_integer_type(lp.vars[col_j])
                  ? std::ceil(bound - 1e-9) : bound;
            }
          }
          ineq_singleton[static_cast<size_t>(r)] = true;
        }
      }

      // Remove singleton inequality rows.
      int singleton_ineq_count = 0;
      for (auto b : ineq_singleton) { if (b) ++singleton_ineq_count; }
      if (singleton_ineq_count > 0) {
        const int new_m = m_ineq - singleton_ineq_count;
        std::vector<Eigen::Triplet<double>> tri;
        tri.reserve(static_cast<size_t>(lp.A.nonZeros()));
        Eigen::VectorXd new_b(new_m);
        int out_row = 0;
        for (int r = 0; r < m_ineq; ++r) {
          if (ineq_singleton[static_cast<size_t>(r)]) continue;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
            tri.emplace_back(out_row, static_cast<int>(it.col()), it.value());
          }
          new_b[out_row] = lp.b[r];
          ++out_row;
        }
        Eigen::SparseMatrix<double> Anew(new_m, n);
        Anew.setFromTriplets(tri.begin(), tri.end());
        Anew.makeCompressed();
        lp.A = std::move(Anew);
        lp.b = std::move(new_b);
        A_row = lp.A;
      }

      // Equality singleton rows: a_j * x_j = b → fix x_j.
      std::vector<bool> eq_singleton(static_cast<size_t>(m_eq), false);
      for (int r = 0; r < m_eq; ++r) {
        int nnz = 0;
        int col_j = -1;
        double coeff = 0.0;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          if (std::abs(it.value()) > 1e-15) {
            ++nnz;
            col_j = static_cast<int>(it.col());
            coeff = it.value();
            if (nnz > 1) break;
          }
        }
        if (nnz == 1 && col_j >= 0) {
          const double val = lp.beq[r] / coeff;
          if (is_integer_type(lp.vars[col_j])) {
            const double rval = std::round(val);
            if (std::abs(rval - val) > 1e-6) continue;  // infeasible, skip
            lp.vars[col_j].lb = rval;
            lp.vars[col_j].ub = rval;
          } else {
            lp.vars[col_j].lb = val;
            lp.vars[col_j].ub = val;
          }
          eq_singleton[static_cast<size_t>(r)] = true;
          ++round_fixed;
        }
      }

      // Remove singleton equality rows.
      int singleton_eq_count = 0;
      for (auto b : eq_singleton) { if (b) ++singleton_eq_count; }
      if (singleton_eq_count > 0) {
        const int new_m = m_eq - singleton_eq_count;
        std::vector<Eigen::Triplet<double>> tri;
        tri.reserve(static_cast<size_t>(lp.Aeq.nonZeros()));
        Eigen::VectorXd new_beq(new_m);
        int out_row = 0;
        for (int r = 0; r < m_eq; ++r) {
          if (eq_singleton[static_cast<size_t>(r)]) continue;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
            tri.emplace_back(out_row, static_cast<int>(it.col()), it.value());
          }
          new_beq[out_row] = lp.beq[r];
          ++out_row;
        }
        Eigen::SparseMatrix<double> Aeq_new(new_m, n);
        Aeq_new.setFromTriplets(tri.begin(), tri.end());
        Aeq_new.makeCompressed();
        lp.Aeq = std::move(Aeq_new);
        lp.beq = std::move(new_beq);
        Aeq_row = lp.Aeq;
      }
    }

    // Recompute m_ineq after possible singleton row removal.
    const int m_ineq_updated = static_cast<int>(lp.A.rows());

    // ── 3. Redundant inequality removal: if max_activity ≤ b, row is redundant ──
    // Build a removal mask (we'll rebuild the matrix at the end).
    std::vector<bool> ineq_redundant(static_cast<size_t>(m_ineq_updated), false);
    int redundant_count = 0;
    for (int r = 0; r < m_ineq_updated; ++r) {
      double max_activity = 0.0;
      bool has_inf = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(lp.vars[j].ub)) { has_inf = true; break; }
          max_activity += a * lp.vars[j].ub;
        } else {
          if (!std::isfinite(lp.vars[j].lb)) { has_inf = true; break; }
          max_activity += a * lp.vars[j].lb;
        }
      }
      if (!has_inf && max_activity <= lp.b[r] + 1e-9) {
        ineq_redundant[static_cast<size_t>(r)] = true;
        ++redundant_count;
      }
    }

    // Remove redundant rows from A and b.
    if (redundant_count > 0) {
      const int new_m = m_ineq_updated - redundant_count;
      std::vector<Eigen::Triplet<double>> tri;
      tri.reserve(static_cast<size_t>(lp.A.nonZeros()));
      Eigen::VectorXd new_b(new_m);
      // Use row-major A_row for correct row iteration.
      int out_row = 0;
      for (int r = 0; r < m_ineq_updated; ++r) {
        if (ineq_redundant[static_cast<size_t>(r)]) continue;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          tri.emplace_back(out_row, static_cast<int>(it.col()), it.value());
        }
        new_b[out_row] = lp.b[r];
        ++out_row;
      }
      Eigen::SparseMatrix<double> Anew(new_m, n);
      Anew.setFromTriplets(tri.begin(), tri.end());
      Anew.makeCompressed();
      lp.A = std::move(Anew);
      lp.b = std::move(new_b);
      // Rebuild row-major copy.
      A_row = lp.A;
    }

    // ── 4. Fix variables whose bounds have collapsed ──
    for (int j = 0; j < n; ++j) {
      if (std::abs(lp.vars[j].ub - lp.vars[j].lb) < 1e-9 && std::isfinite(lp.vars[j].lb)) {
        const double fixed_val = lp.vars[j].lb;
        if (std::abs(lp.vars[j].ub - fixed_val) > 1e-15 || std::abs(lp.vars[j].lb - fixed_val) > 1e-15) {
          lp.vars[j].lb = fixed_val;
          lp.vars[j].ub = fixed_val;
          ++round_fixed;
        }
      }
    }

    // ── 5. Binary probing: fix x_j=0, propagate; fix x_j=1, propagate ──
    // If one direction is infeasible, the other must hold.
    {
      // Collect unfixed binary variables.
      std::vector<int> probe_candidates;
      for (int j = 0; j < n; ++j) {
        if (lp.vars[j].type != VarType::Binary) continue;
        if (std::abs(lp.vars[j].ub - lp.vars[j].lb) < 1e-9) continue; // already fixed
        probe_candidates.push_back(j);
      }
      // Budget: limit probing to avoid expensive rounds on large problems.
      const int max_probes = std::min(200, static_cast<int>(probe_candidates.size()));

      for (int pi = 0; pi < max_probes; ++pi) {
        const int j = probe_candidates[static_cast<size_t>(pi)];
        if (std::abs(lp.vars[j].ub - lp.vars[j].lb) < 1e-9) continue; // fixed by earlier probe

        // Save original bounds.
        std::vector<double> orig_lb(static_cast<size_t>(n)), orig_ub(static_cast<size_t>(n));
        for (int k = 0; k < n; ++k) {
          orig_lb[static_cast<size_t>(k)] = lp.vars[k].lb;
          orig_ub[static_cast<size_t>(k)] = lp.vars[k].ub;
        }

        bool infeas_at_0 = false, infeas_at_1 = false;
        std::vector<double> lb_at_0(orig_lb), ub_at_0(orig_ub);
        std::vector<double> lb_at_1(orig_lb), ub_at_1(orig_ub);

        // Probe x_j = 0: fix bounds, propagate.
        {
          lb_at_0[static_cast<size_t>(j)] = 0.0;
          ub_at_0[static_cast<size_t>(j)] = 0.0;
          for (int prop_round = 0; prop_round < 3; ++prop_round) {
            // Propagate through inequality rows containing j.
            for (int r = 0; r < static_cast<int>(lp.A.rows()); ++r) {
              double min_act = 0.0;
              bool has_inf = false;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                if (a > 0.0) {
                  if (!std::isfinite(lb_at_0[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * lb_at_0[static_cast<size_t>(k)];
                } else {
                  if (!std::isfinite(ub_at_0[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * ub_at_0[static_cast<size_t>(k)];
                }
              }
              if (has_inf) continue;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                const double contrib = (a > 0.0) ? a * lb_at_0[static_cast<size_t>(k)] : a * ub_at_0[static_cast<size_t>(k)];
                const double residual = lp.b[r] - (min_act - contrib);
                if (a > 0.0) {
                  const double nub = residual / a;
                  if (nub < ub_at_0[static_cast<size_t>(k)] - 1e-9) {
                    ub_at_0[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::floor(nub + 1e-9) : nub;
                  }
                } else {
                  const double nlb = residual / a;
                  if (nlb > lb_at_0[static_cast<size_t>(k)] + 1e-9) {
                    lb_at_0[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::ceil(nlb - 1e-9) : nlb;
                  }
                }
              }
            }
            // Also propagate through equalities.
            for (int r = 0; r < m_eq; ++r) {
              double min_act = 0.0;
              bool has_inf = false;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                if (a > 0.0) {
                  if (!std::isfinite(lb_at_0[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * lb_at_0[static_cast<size_t>(k)];
                } else {
                  if (!std::isfinite(ub_at_0[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * ub_at_0[static_cast<size_t>(k)];
                }
              }
              if (has_inf) continue;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                const double contrib = (a > 0.0) ? a * lb_at_0[static_cast<size_t>(k)] : a * ub_at_0[static_cast<size_t>(k)];
                const double residual = lp.beq[r] - (min_act - contrib);
                if (a > 0.0) {
                  const double nub = residual / a;
                  if (nub < ub_at_0[static_cast<size_t>(k)] - 1e-9)
                    ub_at_0[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::floor(nub + 1e-9) : nub;
                } else {
                  const double nlb = residual / a;
                  if (nlb > lb_at_0[static_cast<size_t>(k)] + 1e-9)
                    lb_at_0[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::ceil(nlb - 1e-9) : nlb;
                }
              }
            }
            // Check for infeasibility.
            for (int k = 0; k < n; ++k) {
              if (lb_at_0[static_cast<size_t>(k)] > ub_at_0[static_cast<size_t>(k)] + 1e-9) {
                infeas_at_0 = true;
                break;
              }
            }
            if (infeas_at_0) break;
          }
        }

        // Probe x_j = 1.
        {
          lb_at_1[static_cast<size_t>(j)] = 1.0;
          ub_at_1[static_cast<size_t>(j)] = 1.0;
          for (int prop_round = 0; prop_round < 3; ++prop_round) {
            for (int r = 0; r < static_cast<int>(lp.A.rows()); ++r) {
              double min_act = 0.0;
              bool has_inf = false;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                if (a > 0.0) {
                  if (!std::isfinite(lb_at_1[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * lb_at_1[static_cast<size_t>(k)];
                } else {
                  if (!std::isfinite(ub_at_1[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * ub_at_1[static_cast<size_t>(k)];
                }
              }
              if (has_inf) continue;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                const double contrib = (a > 0.0) ? a * lb_at_1[static_cast<size_t>(k)] : a * ub_at_1[static_cast<size_t>(k)];
                const double residual = lp.b[r] - (min_act - contrib);
                if (a > 0.0) {
                  const double nub = residual / a;
                  if (nub < ub_at_1[static_cast<size_t>(k)] - 1e-9)
                    ub_at_1[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::floor(nub + 1e-9) : nub;
                } else {
                  const double nlb = residual / a;
                  if (nlb > lb_at_1[static_cast<size_t>(k)] + 1e-9)
                    lb_at_1[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::ceil(nlb - 1e-9) : nlb;
                }
              }
            }
            for (int r = 0; r < m_eq; ++r) {
              double min_act = 0.0;
              bool has_inf = false;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                if (a > 0.0) {
                  if (!std::isfinite(lb_at_1[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * lb_at_1[static_cast<size_t>(k)];
                } else {
                  if (!std::isfinite(ub_at_1[static_cast<size_t>(k)])) { has_inf = true; break; }
                  min_act += a * ub_at_1[static_cast<size_t>(k)];
                }
              }
              if (has_inf) continue;
              for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
                const int k = static_cast<int>(it.col());
                const double a = it.value();
                if (std::abs(a) <= 1e-15) continue;
                const double contrib = (a > 0.0) ? a * lb_at_1[static_cast<size_t>(k)] : a * ub_at_1[static_cast<size_t>(k)];
                const double residual = lp.beq[r] - (min_act - contrib);
                if (a > 0.0) {
                  const double nub = residual / a;
                  if (nub < ub_at_1[static_cast<size_t>(k)] - 1e-9)
                    ub_at_1[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::floor(nub + 1e-9) : nub;
                } else {
                  const double nlb = residual / a;
                  if (nlb > lb_at_1[static_cast<size_t>(k)] + 1e-9)
                    lb_at_1[static_cast<size_t>(k)] = is_integer_type(lp.vars[k]) ? std::ceil(nlb - 1e-9) : nlb;
                }
              }
            }
            for (int k = 0; k < n; ++k) {
              if (lb_at_1[static_cast<size_t>(k)] > ub_at_1[static_cast<size_t>(k)] + 1e-9) {
                infeas_at_1 = true;
                break;
              }
            }
            if (infeas_at_1) break;
          }
        }

        // Apply probing results.
        if (infeas_at_0 && infeas_at_1) {
          // Both directions infeasible — problem is infeasible.
          // Just return; B&C will detect via LP failure.
          return total_fixed;
        }
        if (infeas_at_0) {
          // x_j must be 1
          lp.vars[j].lb = 1.0;
          lp.vars[j].ub = 1.0;
          ++round_fixed;
        } else if (infeas_at_1) {
          // x_j must be 0
          lp.vars[j].lb = 0.0;
          lp.vars[j].ub = 0.0;
          ++round_fixed;
        } else {
          // Apply intersection of implied bounds from both probes.
          // Since x_j is either 0 or 1, valid bounds are the union (convex hull):
          // lb = min(lb_at_0, lb_at_1), ub = max(ub_at_0, ub_at_1).
          for (int k = 0; k < n; ++k) {
            const double best_lb = std::min(lb_at_0[static_cast<size_t>(k)], lb_at_1[static_cast<size_t>(k)]);
            const double best_ub = std::max(ub_at_0[static_cast<size_t>(k)], ub_at_1[static_cast<size_t>(k)]);
            if (best_lb > lp.vars[k].lb + 1e-9) {
              lp.vars[k].lb = best_lb;
            }
            if (best_ub < lp.vars[k].ub - 1e-9) {
              lp.vars[k].ub = best_ub;
            }
          }
        }
      }
    }

    // ── 6. Fix variables whose bounds have collapsed ──
    for (int j = 0; j < n; ++j) {
      if (std::abs(lp.vars[j].ub - lp.vars[j].lb) < 1e-9 && std::isfinite(lp.vars[j].lb)) {
        const double fixed_val = lp.vars[j].lb;
        if (std::abs(lp.vars[j].ub - fixed_val) > 1e-15 || std::abs(lp.vars[j].lb - fixed_val) > 1e-15) {
          lp.vars[j].lb = fixed_val;
          lp.vars[j].ub = fixed_val;
          ++round_fixed;
        }
      }
    }

    total_fixed += round_fixed;
    if (round_fixed == 0) break;
  }

  // ── Final pass: Substitute fixed variables out of constraint matrices ──
  // For each variable with lb == ub, subtract its contribution from RHS
  // and zero out its coefficients. This reduces effective LP sparsity and
  // makes simplex solve faster without changing the problem.
  {
    std::vector<int> fixed_vars;
    std::vector<double> fixed_vals;
    for (int j = 0; j < n; ++j) {
      if (std::abs(lp.vars[j].ub - lp.vars[j].lb) < 1e-9 && std::isfinite(lp.vars[j].lb)) {
        fixed_vars.push_back(j);
        fixed_vals.push_back(lp.vars[j].lb);
      }
    }

    if (!fixed_vars.empty()) {
      // Build column-major view for efficient column access.
      // Subtract a_rj * x_j from b_r for each fixed variable j, then zero a_rj.
      // Inequality constraints: A * x <= b → b_r -= sum_j(a_rj * fix_j)
      for (size_t fi = 0; fi < fixed_vars.size(); ++fi) {
        const int j = fixed_vars[fi];
        const double fv = fixed_vals[fi];
        if (std::abs(fv) < 1e-15) continue;  // fixed at zero — no RHS change needed
        for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
          lp.b[it.row()] -= it.value() * fv;
        }
        for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
          lp.beq[it.row()] -= it.value() * fv;
        }
      }

      // Zero out columns of fixed variables.  Build new sparse matrices.
      const int m_ineq_final = static_cast<int>(lp.A.rows());
      const int m_eq_final = static_cast<int>(lp.Aeq.rows());

      std::vector<char> is_fixed(static_cast<size_t>(n), 0);
      for (int j : fixed_vars) is_fixed[static_cast<size_t>(j)] = 1;

      // Rebuild A with fixed-variable columns zeroed out.
      {
        std::vector<Eigen::Triplet<double>> tri;
        tri.reserve(static_cast<size_t>(lp.A.nonZeros()));
        for (int j = 0; j < n; ++j) {
          if (is_fixed[static_cast<size_t>(j)]) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
            tri.emplace_back(static_cast<int>(it.row()), j, it.value());
          }
        }
        Eigen::SparseMatrix<double> Anew(m_ineq_final, n);
        Anew.setFromTriplets(tri.begin(), tri.end());
        Anew.makeCompressed();
        lp.A = std::move(Anew);
      }

      // Rebuild Aeq with fixed-variable columns zeroed out.
      {
        std::vector<Eigen::Triplet<double>> tri;
        tri.reserve(static_cast<size_t>(lp.Aeq.nonZeros()));
        for (int j = 0; j < n; ++j) {
          if (is_fixed[static_cast<size_t>(j)]) continue;
          for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
            tri.emplace_back(static_cast<int>(it.row()), j, it.value());
          }
        }
        Eigen::SparseMatrix<double> Aeq_new(m_eq_final, n);
        Aeq_new.setFromTriplets(tri.begin(), tri.end());
        Aeq_new.makeCompressed();
        lp.Aeq = std::move(Aeq_new);
      }
    }
  }

  return total_fixed;
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
    if (lb[i] > ub[i] + 1e-12) {
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

void add_rows_to_lp(LPModel& lp,
                     const std::vector<Eigen::VectorXd>& rows,
                     const std::vector<double>& rhs_vals) {
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
    for (int j = 0; j < n; ++j) {
      const double a = row[j];
      if (!std::isfinite(a)) {
        finite = false;
        break;
      }
      if (std::abs(a) > 1e-15) nonzero = true;
    }
    if (finite && nonzero) {
      valid_rows.push_back(i);
    }
  }

  const int k = static_cast<int>(valid_rows.size());
  if (k == 0) {
    return;
  }
  const int m = static_cast<int>(lp.A.rows());
  const bool had_row_lhs = lp_has_row_lhs(lp);
  Eigen::SparseMatrix<double> Anew(m + k, n);

  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(lp.A.nonZeros() + static_cast<size_t>(k) * 20));

  for (int col = 0; col < lp.A.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      tri.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    for (int j = 0; j < n; ++j) {
      const double a = rows[static_cast<size_t>(src)][j];
      if (std::abs(a) > 1e-15) {
        tri.emplace_back(m + i, j, a);
      }
    }
  }

  Anew.setFromTriplets(tri.begin(), tri.end());
  Anew.makeCompressed();

  Eigen::VectorXd bnew(m + k);
  if (m > 0) {
    bnew.head(m) = lp.b;
  }
  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    bnew[m + i] = rhs_vals[static_cast<size_t>(src)];
  }

  lp.A = std::move(Anew);
  lp.b = std::move(bnew);
  if (lp.row_lhs.size() > 0) {
    Eigen::VectorXd lhs_new =
        Eigen::VectorXd::Constant(m + k, -std::numeric_limits<double>::infinity());
    if (had_row_lhs && m > 0) lhs_new.head(m) = lp.row_lhs;
    lp.row_lhs = std::move(lhs_new);
  }
}

void add_sparse_rows_to_lp(LPModel& lp,
                            const std::vector<Eigen::SparseVector<double>>& rows,
                            const std::vector<double>& rhs_vals) {
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
  Eigen::SparseMatrix<double> Anew(m + k, n);

  std::vector<Eigen::Triplet<double>> tri;
  tri.reserve(static_cast<size_t>(lp.A.nonZeros() + static_cast<size_t>(k) * 20));

  for (int col = 0; col < lp.A.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      tri.emplace_back(it.row(), it.col(), it.value());
    }
  }
  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    for (Eigen::SparseVector<double>::InnerIterator it(rows[static_cast<size_t>(src)]); it; ++it) {
      if (it.index() < n) {
        tri.emplace_back(m + i, static_cast<int>(it.index()), it.value());
      }
    }
  }

  Anew.setFromTriplets(tri.begin(), tri.end());
  Anew.makeCompressed();

  Eigen::VectorXd bnew(m + k);
  if (m > 0) bnew.head(m) = lp.b;
  for (int i = 0; i < k; ++i) {
    const int src = valid_rows[static_cast<std::size_t>(i)];
    bnew[m + i] = rhs_vals[static_cast<size_t>(src)];
  }

  lp.A = std::move(Anew);
  lp.b = std::move(bnew);
  if (lp.row_lhs.size() > 0) {
    Eigen::VectorXd lhs_new =
        Eigen::VectorXd::Constant(m + k, -std::numeric_limits<double>::infinity());
    if (had_row_lhs && m > 0) lhs_new.head(m) = lp.row_lhs;
    lp.row_lhs = std::move(lhs_new);
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
  for (auto& literal : node.branch_reasons) {
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
  std::vector<std::vector<int>> reason_by_depth(
      static_cast<std::size_t>(branch_count + 1));
  for (int r = 0; r < static_cast<int>(node.domain_reason_bounds.size()); ++r) {
    const int depth = std::clamp(node.domain_reason_bounds[static_cast<std::size_t>(r)].depth,
                                 0, branch_count);
    reason_by_depth[static_cast<std::size_t>(depth)].push_back(r);
  }

  for (int r : reason_by_depth[0]) {
    append_reason_bound(node.domain_reason_bounds[static_cast<std::size_t>(r)]);
  }
  for (int i = 0; i < branch_count; ++i) {
    const auto& lit = node.branch_reasons[static_cast<std::size_t>(i)];
    append_entry(lit, {lit}, i + 1, true, BranchDomainLiteral{}, false);
    for (int r : reason_by_depth[static_cast<std::size_t>(i + 1)]) {
      append_reason_bound(node.domain_reason_bounds[static_cast<std::size_t>(r)]);
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
  DomainReasonBound& stored = node.domain_reason_bounds.back();
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

  for (auto& entry : node.local_domain_trail) {
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
  // Check variable bounds (use tighter of node bounds and lp.vars bounds).
  for (int i = 0; i < n; ++i) {
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
  (void)x_relax;
  const double gap = incumbent_obj - node_bound;
  if (gap <= 1e-12) return 0;

  const int n = static_cast<int>(vars.size());
  if (reduced_costs.size() < n) return 0;

  // Build set of basic variable indices for O(1) lookup.
  std::vector<char> is_basic(static_cast<size_t>(n), 0);
  for (int idx : basis_indices) {
    if (idx >= 0 && idx < n) {
      is_basic[static_cast<size_t>(idx)] = 1;
    }
  }

  int fixed = 0;
  for (int j = 0; j < n; ++j) {
    if (!is_integer_type(vars[j])) continue;
    if (is_basic[static_cast<size_t>(j)]) continue;
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
    const double max_increase = lpredcost * width;
    if (max_increase > gap) {
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
    } else if (max_increase < -gap) {
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
          changes_out->push_back(BoundChangeInfo{j, new_ub - old, false});
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
          changes_out->push_back(BoundChangeInfo{j, new_lb - old, true});
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

  auto reason_frontier_hash =
      [&](const std::vector<ReasonFrontierEntry>& frontier) {
    std::size_t h = 0;
    for (const auto& entry : frontier) {
      std::size_t eh = branch_literal_hash(entry.lit);
      eh ^= std::hash<int>{}(entry.trail_pos) + 0x9e3779b97f4a7c15ULL +
            (eh << 6) + (eh >> 2);
      h ^= eh + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    }
    return h;
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
	    std::unordered_set<std::size_t> seen;
	    std::unordered_set<int> skipped_positions;
	    int resolved = 0;
	    for (int iter = 0; iter < max_resolves; ++iter) {
	      std::size_t h = reason_frontier_hash(frontier);
	      for (int pos : skipped_positions) {
	        h ^= std::hash<int>{}(pos) + 0x9e3779b97f4a7c15ULL +
	             (h << 6) + (h >> 2);
	      }
	      if (!seen.insert(h).second) break;

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
      if (conflict_clause_hash(existing) == h) {
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

DualProofResolutionStatus resolve_dual_proof_conflict_from_local_trail(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    double int_tol,
    double lp_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const Eigen::SparseVector<double>& coeff_sparse,
    double rhs,
    int max_literals,
    DualProofConflictResolution& out) {
  out = DualProofConflictResolution{};
  auto fail = [&](DualProofResolutionStatus status) {
    out.status = status;
    return status;
  };
  auto success = [&]() {
    out.status = DualProofResolutionStatus::Success;
    return DualProofResolutionStatus::Success;
  };
  const int n = static_cast<int>(vars.size());
  if (max_literals <= 0 || n <= 0 || coeff_sparse.size() < n ||
      coeff_sparse.nonZeros() <= 0 || !std::isfinite(rhs) ||
      root_lb.size() < n || root_ub.size() < n ||
      node_lb.size() < n || node_ub.size() < n) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);
  double root_min_activity = 0.0;
  double node_min_activity = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(coeff_sparse); it; ++it) {
    const int j = static_cast<int>(it.index());
    const double a = it.value();
    if (j < 0 || j >= n || !std::isfinite(a)) {
      return fail(DualProofResolutionStatus::InvalidInput);
    }
    coeff[static_cast<std::size_t>(j)] = a;
    if (a > 0.0) {
      if (!std::isfinite(root_lb[j]) || !std::isfinite(node_lb[j])) {
        return fail(DualProofResolutionStatus::InvalidInput);
      }
      root_min_activity += a * root_lb[j];
      node_min_activity += a * node_lb[j];
    } else if (a < 0.0) {
      if (!std::isfinite(root_ub[j]) || !std::isfinite(node_ub[j])) {
        return fail(DualProofResolutionStatus::InvalidInput);
      }
      root_min_activity += a * root_ub[j];
      node_min_activity += a * node_ub[j];
    }
  }
  if (!std::isfinite(root_min_activity) || !std::isfinite(node_min_activity)) {
    return fail(DualProofResolutionStatus::InvalidInput);
  }

  const double tol = std::max({1e-9, int_tol, lp_tol});
  const double proof_tol = std::max(1e-7, tol * std::max(10.0, std::abs(rhs)));
  const double cutoff_rhs = rhs + proof_tol;
  if (node_min_activity < cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }
  if (root_min_activity >= cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  struct TrailCandidate {
    BranchDomainLiteral lit;
    std::vector<BranchDomainLiteral> reason;
    double delta{0.0};
    double base_bound{0.0};
    double priority{0.0};
    int trail_pos{-1};
    int prev_bound_pos{-1};
    bool is_branch{false};
  };

  const bool have_native_trail =
      local_domain_trail != nullptr && !local_domain_trail->empty();
  const int trail_end =
      have_native_trail
          ? static_cast<int>(local_domain_trail->size())
          : static_cast<int>(branch_reasons.size() + reason_bounds.size());

  auto proof_side_matches = [&](const BranchDomainLiteral& lit) {
    if (lit.var_idx < 0 || lit.var_idx >= n) return false;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    return (a > 0.0 && lit.is_lb) || (a < 0.0 && !lit.is_lb);
  };
  auto active_at_node = [&](const BranchDomainLiteral& lit) {
    if (lit.var_idx < 0 || lit.var_idx >= n) return false;
    return lit.is_lb ? (node_lb[lit.var_idx] >= lit.value - tol)
                     : (node_ub[lit.var_idx] <= lit.value + tol);
  };
  auto proof_side_stronger = [&](const BranchDomainLiteral& lhs,
                                 const BranchDomainLiteral& rhs_lit) {
    if (lhs.var_idx != rhs_lit.var_idx || lhs.is_lb != rhs_lit.is_lb) {
      return false;
    }
    return lhs.is_lb ? lhs.value > rhs_lit.value + tol
                     : lhs.value < rhs_lit.value - tol;
  };
  auto proof_side_same = [&](const BranchDomainLiteral& lhs,
                             const BranchDomainLiteral& rhs_lit) {
    return lhs.var_idx == rhs_lit.var_idx && lhs.is_lb == rhs_lit.is_lb &&
           std::abs(lhs.value - rhs_lit.value) <= tol;
  };
  auto proof_literal_delta = [&](const BranchDomainLiteral& lit,
                                 double* delta_out) -> bool {
    if (delta_out != nullptr) *delta_out = 0.0;
    if (lit.var_idx < 0 || lit.var_idx >= n ||
        !std::isfinite(lit.value)) {
      return false;
    }
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
    audited_activity = root_min_activity;
    std::vector<char> seen_lower(static_cast<std::size_t>(n), 0);
    std::vector<char> seen_upper(static_cast<std::size_t>(n), 0);
    for (const auto& lit : frontier) {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
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
  auto frontier_implies_literals =
      [&](const std::vector<BranchDomainLiteral>& reason_frontier,
          const std::vector<BranchDomainLiteral>& needed_literals) -> bool {
    if (needed_literals.empty()) return false;
    Eigen::VectorXd closure_lb = root_lb;
    Eigen::VectorXd closure_ub = root_ub;
    auto apply_lit = [&](const BranchDomainLiteral& lit) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      if (lit.is_lb) {
        if (lit.value > closure_ub[lit.var_idx] + tol) return false;
        if (lit.value > closure_lb[lit.var_idx]) {
          closure_lb[lit.var_idx] = lit.value;
        }
      } else {
        if (lit.value < closure_lb[lit.var_idx] - tol) return false;
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
      return lit.is_lb ? closure_lb[lit.var_idx] >= lit.value - tol
                       : closure_ub[lit.var_idx] <= lit.value + tol;
    };
    for (const auto& lit : reason_frontier) {
      if (!apply_lit(lit)) return false;
    }
    if (have_native_trail) {
      bool changed = true;
      for (int pass = 0; pass < trail_end + 1 && changed; ++pass) {
        changed = false;
        for (const auto& entry : *local_domain_trail) {
          if (entry.pos < 0 || entry.pos >= trail_end ||
              entry.reason.empty()) {
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
          if (closure_lb[entry.bound.var_idx] > old_lb + tol ||
              closure_ub[entry.bound.var_idx] < old_ub - tol) {
            changed = true;
          }
        }
      }
    }
    for (const auto& lit : needed_literals) {
      if (!active_lit(lit)) return false;
    }
    return true;
  };
  auto frontier_implies_proof_frontier =
      [&](const std::vector<BranchDomainLiteral>& reason_frontier,
          const std::vector<BranchDomainLiteral>& needed_proof_frontier,
          double& audited_activity) -> bool {
    if (audited_activity_from_frontier(reason_frontier, audited_activity)) {
      if (audited_activity >= cutoff_rhs - proof_tol) {
        return true;
      }
    }
    if (!frontier_implies_literals(reason_frontier, needed_proof_frontier)) {
      return false;
    }
    audited_activity = root_min_activity;
    for (const auto& lit : needed_proof_frontier) {
      double delta = 0.0;
      if (!proof_literal_delta(lit, &delta)) return false;
      audited_activity += delta;
    }
    return std::isfinite(audited_activity) &&
           audited_activity >= cutoff_rhs - proof_tol;
  };
  std::vector<TrailCandidate> candidates;
  candidates.reserve(static_cast<std::size_t>(coeff_sparse.nonZeros()));
  std::vector<int> candidate_by_col(static_cast<std::size_t>(n), -1);
  auto add_or_replace_candidate =
      [&](BranchDomainLiteral lit, std::vector<BranchDomainLiteral> reason,
          int trail_pos, int prev_bound_pos, bool is_branch) {
    const int j = lit.var_idx;
    if (j < 0 || j >= n || !std::isfinite(lit.value) ||
        trail_pos < 0 || trail_pos >= trail_end || !proof_side_matches(lit) ||
        !active_at_node(lit)) {
      return;
    }
    const double a = coeff[static_cast<std::size_t>(j)];
    const double base = lit.is_lb ? root_lb[j] : root_ub[j];
    if (!std::isfinite(base)) return;
    double delta = 0.0;
    if (lit.is_lb) {
      if (lit.value <= base + tol) return;
      delta = a * (lit.value - base);
    } else {
      if (lit.value >= base - tol) return;
      delta = a * (lit.value - base);
    }
    if (!std::isfinite(delta) || delta <= proof_tol * 1e-4) return;
    if (reason.empty()) reason.push_back(lit);
    canonicalize_branch_literals(reason);
    TrailCandidate cand{lit,
                        std::move(reason),
                        delta,
                        base,
                        std::abs(delta),
                        trail_pos,
                        prev_bound_pos,
                        is_branch};
    int& slot = candidate_by_col[static_cast<std::size_t>(j)];
    if (slot < 0) {
      slot = static_cast<int>(candidates.size());
      candidates.push_back(std::move(cand));
      return;
    }
    TrailCandidate& cur = candidates[static_cast<std::size_t>(slot)];
    if (proof_side_stronger(cand.lit, cur.lit) ||
        (proof_side_same(cand.lit, cur.lit) && cand.trail_pos > cur.trail_pos)) {
      cur = std::move(cand);
    }
  };

  if (have_native_trail) {
    for (const auto& entry : *local_domain_trail) {
      add_or_replace_candidate(entry.bound, entry.reason, entry.pos,
                               entry.prev_bound_pos, entry.is_branch);
    }
  } else {
    for (int pos = 0; pos < static_cast<int>(branch_reasons.size()); ++pos) {
      const auto& lit = branch_reasons[static_cast<std::size_t>(pos)];
      add_or_replace_candidate(lit, {lit}, pos, -1, true);
    }
    for (const auto& rb : reason_bounds) {
      add_or_replace_candidate(rb.bound, rb.reason, rb.trail_pos,
                               rb.prev_bound_pos, false);
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

  double activity = root_min_activity;
  std::vector<TrailCandidate> selected;
  selected.reserve(candidates.size());
  for (const auto& cand : candidates) {
    selected.push_back(cand);
    activity += cand.delta;
    if (activity >= cutoff_rhs) break;
  }
  if (selected.empty() || activity < cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  for (int k = static_cast<int>(selected.size()) - 1; k >= 0; --k) {
    TrailCandidate& cand = selected[static_cast<std::size_t>(k)];
    const int j = cand.lit.var_idx;
    const double a = coeff[static_cast<std::size_t>(j)];
    const double without = activity - cand.delta;
    if (cand.lit.is_lb) {
      if (a <= 0.0) continue;
      double relaxed = (cutoff_rhs - without) / a + cand.base_bound;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        relaxed = std::ceil(relaxed - tol);
      }
      if (!std::isfinite(relaxed) || relaxed >= cand.lit.value - proof_tol) {
        continue;
      }
      if (relaxed <= cand.base_bound + tol) {
        activity = without;
        selected.erase(selected.begin() + k);
      } else {
        if (have_native_trail) {
          int p = cand.prev_bound_pos;
          while (p >= 0 && p < static_cast<int>(local_domain_trail->size()) &&
                 relaxed <= (*local_domain_trail)[static_cast<std::size_t>(p)]
                                    .bound.value +
                                tol) {
            cand.trail_pos = p;
            cand.prev_bound_pos =
                (*local_domain_trail)[static_cast<std::size_t>(p)].prev_bound_pos;
            p = cand.prev_bound_pos;
          }
        }
        activity += a * (relaxed - cand.lit.value);
        cand.lit.value = relaxed;
        cand.delta = a * (cand.lit.value - cand.base_bound);
        if (cand.is_branch) cand.reason = {cand.lit};
      }
    } else {
      if (a >= 0.0) continue;
      double relaxed = (cutoff_rhs - without) / a + cand.base_bound;
      if (is_integer_type(vars[static_cast<std::size_t>(j)])) {
        relaxed = std::floor(relaxed + tol);
      }
      if (!std::isfinite(relaxed) || relaxed <= cand.lit.value + proof_tol) {
        continue;
      }
      if (relaxed >= cand.base_bound - tol) {
        activity = without;
        selected.erase(selected.begin() + k);
      } else {
        if (have_native_trail) {
          int p = cand.prev_bound_pos;
          while (p >= 0 && p < static_cast<int>(local_domain_trail->size()) &&
                 relaxed >= (*local_domain_trail)[static_cast<std::size_t>(p)]
                                    .bound.value -
                                tol) {
            cand.trail_pos = p;
            cand.prev_bound_pos =
                (*local_domain_trail)[static_cast<std::size_t>(p)].prev_bound_pos;
            p = cand.prev_bound_pos;
          }
        }
        activity += a * (relaxed - cand.lit.value);
        cand.lit.value = relaxed;
        cand.delta = a * (cand.lit.value - cand.base_bound);
        if (cand.is_branch) cand.reason = {cand.lit};
      }
    }
    if (activity <= cutoff_rhs + proof_tol) break;
  }
  if (selected.empty() || activity < cutoff_rhs) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  int integral_cols = 0;
  for (const auto& var : vars) {
    if (is_integer_type(var)) ++integral_cols;
  }
  if (10 * static_cast<int>(selected.size()) > 1000 + 3 * integral_cols ||
      static_cast<int>(selected.size()) > max_literals) {
    return fail(DualProofResolutionStatus::LiteralLimit);
  }

  std::vector<BranchDomainLiteral> proof_frontier;
  proof_frontier.reserve(selected.size());
  for (const auto& cand : selected) proof_frontier.push_back(cand.lit);
  canonicalize_branch_literals(proof_frontier);
  if (proof_frontier.empty() ||
      static_cast<int>(proof_frontier.size()) > max_literals) {
    return fail(proof_frontier.empty()
                    ? DualProofResolutionStatus::ActivityFailed
                    : DualProofResolutionStatus::LiteralLimit);
  }
  double proof_frontier_activity = 0.0;
  if (!audited_activity_from_frontier(proof_frontier,
                                      proof_frontier_activity) ||
      proof_frontier_activity < cutoff_rhs - proof_tol) {
    return fail(DualProofResolutionStatus::ActivityFailed);
  }

  struct FrontierEntry {
    BranchDomainLiteral lit;
    std::vector<BranchDomainLiteral> reason;
    int trail_pos{-1};
    bool is_branch{false};
  };

  auto compact_frontier = [&](std::vector<FrontierEntry>& frontier) {
    std::vector<FrontierEntry> compact;
    compact.reserve(frontier.size());
    for (auto& entry : frontier) {
      if (entry.lit.var_idx < 0 || entry.lit.var_idx >= n ||
          !std::isfinite(entry.lit.value)) {
        continue;
      }
      bool merged = false;
      for (auto& cur : compact) {
        if (cur.lit.var_idx != entry.lit.var_idx ||
            cur.lit.is_lb != entry.lit.is_lb) {
          continue;
        }
        const bool replace =
            proof_side_stronger(entry.lit, cur.lit) ||
            (proof_side_same(entry.lit, cur.lit) &&
             entry.trail_pos > cur.trail_pos);
        if (replace) cur = std::move(entry);
        merged = true;
        break;
      }
      if (!merged) compact.push_back(std::move(entry));
    }
    frontier.swap(compact);
  };

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
      if (!branch_literal_stronger_or_equal(entry.bound, lit, tol)) continue;
      if (best == nullptr || entry.pos > best->pos) best = &entry;
    }
    return best;
  };

  auto fallback_literal_pos = [&](const BranchDomainLiteral& lit) -> int {
    int best = -1;
    if (have_native_trail) {
      for (const auto& entry : *local_domain_trail) {
        if (entry.bound.var_idx == lit.var_idx &&
            entry.bound.is_lb == lit.is_lb &&
            branch_literal_stronger_or_equal(entry.bound, lit, tol)) {
          best = std::max(best, entry.pos);
        }
      }
    } else {
      for (int i = 0; i < static_cast<int>(branch_reasons.size()); ++i) {
        if (branch_literal_stronger_or_equal(
                branch_reasons[static_cast<std::size_t>(i)], lit, tol)) {
          best = std::max(best, i);
        }
      }
      for (const auto& rb : reason_bounds) {
        if (branch_literal_stronger_or_equal(rb.bound, lit, tol)) {
          best = std::max(best, rb.trail_pos);
        }
      }
    }
    return best;
  };

  auto entry_for_literal = [&](const BranchDomainLiteral& lit,
                               int before_pos) -> FrontierEntry {
    const LocalDomainTrailEntry* prior = find_prior_entry(lit, before_pos);
    if (prior != nullptr) {
      return FrontierEntry{
          lit,
          prior->reason.empty()
              ? std::vector<BranchDomainLiteral>{lit}
              : prior->reason,
          prior->pos,
          prior->is_branch};
    }
    return FrontierEntry{lit, {lit}, fallback_literal_pos(lit), true};
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
          integer_var ? std::ceil(lit.value - tol) - 1.0
                      : lit.value - std::max(1e-7, 10.0 * tol),
          false};
    } else {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::floor(lit.value + tol) + 1.0
                      : lit.value + std::max(1e-7, 10.0 * tol),
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
               ? conflict_lit.value <= flipped.value + tol
               : conflict_lit.value >= flipped.value - tol;
  };

  auto explain_conflict_reason_frontier =
      [&](const LocalDomainTrailEntry& entry,
          std::vector<FrontierEntry>& replacement) -> bool {
    replacement.clear();
    if (!have_native_trail || !entry.has_source_conflict_clause ||
        entry.source_conflict_clause.empty() || entry.pos <= 0) {
      return false;
    }
    BranchDomainLiteral flipped_entry;
    if (!flip_literal_for_conflict(entry.bound, flipped_entry)) return false;
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
                ? global_bound >= conflict_lit.value - tol
                : global_bound <= conflict_lit.value + tol;
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
                ? prev.bound.value >= conflict_lit.value - tol
                : prev.bound.value <= conflict_lit.value + tol;
        if (!prev_still_active) break;
        relaxed = prev;
      }
      replacement.push_back(FrontierEntry{
          conflict_lit,
          relaxed.reason.empty() && relaxed.is_branch
              ? std::vector<BranchDomainLiteral>{conflict_lit}
              : relaxed.reason,
          relaxed.pos,
          relaxed.is_branch});
    }
    return found_flipped;
  };

  auto frontier_literals = [&](const std::vector<FrontierEntry>& frontier) {
    std::vector<BranchDomainLiteral> lits;
    lits.reserve(frontier.size());
    for (const auto& entry : frontier) lits.push_back(entry.lit);
    canonicalize_branch_literals(lits);
    return lits;
  };

  auto resolve_reason_frontier = [&](std::vector<FrontierEntry> frontier,
                                     int stop_size,
                                     int min_resolve) {
    if (!have_native_trail) return frontier_literals(frontier);
    compact_frontier(frontier);
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
      if (std::abs(branch_entry.bound.value - branch_entry.prev_bound_value) >
          tol) {
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
    std::vector<BranchDomainLiteral> best = frontier_literals(frontier);
	    std::unordered_set<std::size_t> seen;
	    std::unordered_set<int> skipped_positions;
	    int resolved = 0;
	    for (int iter = 0; iter < max_resolves; ++iter) {
	      std::vector<BranchDomainLiteral> sig = frontier_literals(frontier);
	      std::size_t h = conflict_clause_hash(sig);
	      for (int pos : skipped_positions) {
	        h ^= std::hash<int>{}(pos) + 0x9e3779b97f4a7c15ULL +
	             (h << 6) + (h >> 2);
	      }
	      if (!seen.insert(h).second) break;
	      if (!sig.empty() && static_cast<int>(sig.size()) <= max_literals &&
	          (best.empty() || sig.size() < best.size())) {
        best = sig;
      }

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
          (static_cast<int>(frontier.size()) <= stop_size &&
           resolved >= min_resolve && resolvable <= stop_size)) {
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
	          if (branch_literal_stronger_or_equal(cur, lit, tol)) {
	            return true;
	          }
	        }
	        return false;
	      };
	      std::vector<FrontierEntry> replacement;
	      bool replacement_valid =
	          explain_conflict_reason_frontier(entry, replacement);
	      if (replacement_valid) {
	        replacement.erase(
	            std::remove_if(
	                replacement.begin(), replacement.end(),
	                [&](const FrontierEntry& repl) {
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
	          replacement.push_back(entry_for_literal(reason_lit, latest_pos));
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
	      compact_frontier(frontier);
	      ++resolved;
      std::vector<BranchDomainLiteral> cur = frontier_literals(frontier);
      if (!cur.empty() && static_cast<int>(cur.size()) <= max_literals &&
          (best.empty() || cur.size() < best.size())) {
        best = cur;
      }
      if (static_cast<int>(cur.size()) <= stop_size &&
          resolved >= min_resolve) {
        break;
      }
    }
    return best;
  };

  std::vector<FrontierEntry> initial_frontier;
  initial_frontier.reserve(selected.size());
  for (const auto& cand : selected) {
    initial_frontier.push_back(
        FrontierEntry{cand.lit, cand.reason, cand.trail_pos, cand.is_branch});
  }
  std::vector<BranchDomainLiteral> resolved_frontier =
      resolve_reason_frontier(initial_frontier, /*stop_size=*/1,
                              /*min_resolve=*/1);
  canonicalize_branch_literals(resolved_frontier);
  if (resolved_frontier.empty() ||
      static_cast<int>(resolved_frontier.size()) > max_literals) {
    return fail(resolved_frontier.empty()
                    ? DualProofResolutionStatus::ScopeBlocked
                    : DualProofResolutionStatus::LiteralLimit);
  }
  double resolved_frontier_activity = 0.0;
  if (!frontier_implies_proof_frontier(resolved_frontier, proof_frontier,
                                       resolved_frontier_activity)) {
    return fail(have_native_trail ? DualProofResolutionStatus::ScopeBlocked
                                  : DualProofResolutionStatus::MissingReason);
  }

  auto flip_literal = [&](const BranchDomainLiteral& lit,
                          BranchDomainLiteral& flipped) -> bool {
    if (lit.var_idx < 0 || lit.var_idx >= n || !std::isfinite(lit.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
    if (lit.is_lb) {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::ceil(lit.value - tol) - 1.0
                      : lit.value - std::max(1e-7, 10.0 * tol),
          false};
    } else {
      flipped = BranchDomainLiteral{
          lit.var_idx,
          integer_var ? std::floor(lit.value + tol) + 1.0
                      : lit.value + std::max(1e-7, 10.0 * tol),
          true};
    }
    return true;
  };

  std::unordered_set<std::size_t> seen_clauses;
  auto add_clause = [&](std::vector<BranchDomainLiteral> clause) {
    canonicalize_branch_literals(clause);
    if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
      return;
    }
    const std::size_t h = conflict_clause_hash(clause);
    if (!seen_clauses.insert(h).second) return;
    out.conflict_clauses.push_back(std::move(clause));
  };

  if (out.conflict_clauses.empty()) {
    for (const auto& cand : selected) {
      if (cand.is_branch || cand.reason.empty()) continue;
      BranchDomainLiteral flipped;
      if (!flip_literal(cand.lit, flipped)) continue;
      std::vector<FrontierEntry> reason_frontier;
      reason_frontier.reserve(cand.reason.size());
      for (const auto& reason_lit : cand.reason) {
        reason_frontier.push_back(entry_for_literal(reason_lit, cand.trail_pos));
      }
      std::vector<BranchDomainLiteral> reconv =
          resolve_reason_frontier(std::move(reason_frontier), /*stop_size=*/0,
                                  /*min_resolve=*/0);
      if (!frontier_implies_literals(reconv, {cand.lit})) continue;
      reconv.push_back(flipped);
      add_clause(std::move(reconv));
    }
  }

  if (out.conflict_clauses.empty()) {
    add_clause(resolved_frontier);
    if (conflict_clause_hash(proof_frontier) !=
        conflict_clause_hash(resolved_frontier)) {
      add_clause(proof_frontier);
    }
  }
  if (out.conflict_clauses.empty()) {
    return fail(DualProofResolutionStatus::ScopeBlocked);
  }

  out.valid = true;
  out.proof_frontier = std::move(proof_frontier);
  out.resolved_frontier = std::move(resolved_frontier);
  out.root_min_activity = root_min_activity;
  out.node_min_activity = node_min_activity;
  out.cutoff_rhs = cutoff_rhs;
  out.proof_margin = resolved_frontier_activity - cutoff_rhs;
  out.proof_activity = proof_frontier_activity;
  out.resolved_activity = resolved_frontier_activity;
  return success();
}

bool build_dual_proof_target_bound_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& x_relax,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const std::vector<LocalDomainTrailEntry>* local_domain_trail,
    const std::vector<int>* local_branch_positions,
    const DomainReasonBound& target_bound,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin,
    PoolCut* violated_local_proof_cover,
    const DualProofRow* dual_proof,
    DualProofTargetExplanation* explanation,
    std::vector<PoolCut>* additional_local_proof_covers,
    const DualProofRow* priority_proof) {
  out_clause.clear();
  if (proof_margin != nullptr) *proof_margin = 0.0;
  if (violated_local_proof_cover != nullptr) {
    *violated_local_proof_cover = PoolCut{};
  }
  if (additional_local_proof_covers != nullptr) {
    additional_local_proof_covers->clear();
  }
  if (explanation != nullptr) *explanation = DualProofTargetExplanation{};
  const int n = static_cast<int>(vars.size());
  if (max_literals <= 0 || n <= 0 || dual_proof == nullptr ||
      !dual_proof->valid || dual_proof->coeff.size() < n ||
      !std::isfinite(dual_proof->rhs) ||
      root_lb.size() < n || root_ub.size() < n ||
      x_relax.size() < n) {
    return false;
  }
  const BranchDomainLiteral& target_lit = target_bound.bound;
  if (target_lit.var_idx < 0 || target_lit.var_idx >= n ||
      !std::isfinite(target_lit.value)) {
    return false;
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);
  double root_min_activity = 0.0;
  for (Eigen::SparseVector<double>::InnerIterator it(dual_proof->coeff); it;
       ++it) {
    const int j = static_cast<int>(it.index());
    const double a = it.value();
    if (j < 0 || j >= n || !std::isfinite(a)) continue;
    coeff[static_cast<std::size_t>(j)] = a;
    if (a > 0.0) {
      if (!std::isfinite(root_lb[j])) return false;
      root_min_activity += a * root_lb[j];
    } else if (a < 0.0) {
      if (!std::isfinite(root_ub[j])) return false;
      root_min_activity += a * root_ub[j];
    }
  }

  DualProofTrailResolution trail_resolution;
  const Eigen::SparseVector<double>* priority_coeff_ptr =
      priority_proof != nullptr && priority_proof->valid &&
              priority_proof->coeff.size() >= n
          ? &priority_proof->coeff
          : nullptr;
  const DualProofResolutionStatus trail_status =
      resolve_dual_proof_target_bound_from_local_trail(
          vars, root_lb, root_ub, &x_relax, int_tol, branch_reasons,
          reason_bounds, local_domain_trail, local_branch_positions,
          target_bound, max_literals, dual_proof->coeff, dual_proof->rhs,
          trail_resolution, priority_coeff_ptr);
  if (dual_proof_resolution_success(trail_status) && trail_resolution.valid) {
    double best_violation = 0.0;
    if (violated_local_proof_cover != nullptr) {
      struct ProofTerm {
        BranchDomainLiteral lit;
        double coeff{0.0};
        double root_bound{0.0};
        double lp_activity{0.0};
      };
      auto make_proof_term = [&](const BranchDomainLiteral& lit,
                                 ProofTerm& term) -> bool {
        if (lit.var_idx < 0 || lit.var_idx >= n) return false;
        const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
        if (a > 0.0 && lit.is_lb) {
          if (!std::isfinite(root_lb[lit.var_idx])) return false;
          term = ProofTerm{lit, a, root_lb[lit.var_idx],
                           a * (x_relax[lit.var_idx] - root_lb[lit.var_idx])};
          return true;
        }
        if (a < 0.0 && !lit.is_lb) {
          if (!std::isfinite(root_ub[lit.var_idx])) return false;
          term = ProofTerm{lit, a, root_ub[lit.var_idx],
                           a * (x_relax[lit.var_idx] - root_ub[lit.var_idx])};
          return true;
        }
        return false;
      };
      std::vector<ProofTerm> terms;
      terms.reserve(trail_resolution.proof_frontier.size() + 1);
      std::vector<char> used_var(static_cast<std::size_t>(n), 0);
      auto add_term = [&](const BranchDomainLiteral& lit) {
        ProofTerm term;
        if (!make_proof_term(lit, term)) return false;
        if (used_var[static_cast<std::size_t>(lit.var_idx)] == 0) {
          used_var[static_cast<std::size_t>(lit.var_idx)] = 1;
          terms.push_back(term);
        }
        return true;
      };
      bool cover_valid =
          trail_resolution.cutoff_conflict ||
          add_term(trail_resolution.flipped_target);
      for (const auto& lit : trail_resolution.proof_frontier) {
        cover_valid = add_term(lit) && cover_valid;
      }
      if (cover_valid && !terms.empty()) {
        Eigen::SparseVector<double> cover_coeff(n);
        cover_coeff.reserve(static_cast<int>(terms.size()));
        double cover_rhs = dual_proof->rhs - root_min_activity;
        double norm2 = 0.0;
        for (const auto& term : terms) {
          cover_coeff.coeffRef(term.lit.var_idx) += term.coeff;
          cover_rhs += term.coeff * term.root_bound;
          norm2 += term.coeff * term.coeff;
        }
        cover_coeff.prune(0.0);
        const double lhs = cover_coeff.dot(x_relax);
        const double scale = 1.0 + std::abs(lhs) + std::abs(cover_rhs);
        const double cut_tol = std::max(1e-7, 1e-10 * scale);
        if (lhs > cover_rhs + cut_tol) {
          best_violation = lhs - cover_rhs;
          *violated_local_proof_cover =
              PoolCut{std::move(cover_coeff), cover_rhs, 0, 0.0,
                      std::sqrt(norm2), 0};
        }
      }
    }

    if (proof_margin != nullptr) {
      *proof_margin = trail_resolution.proof_margin;
    }
    if (explanation != nullptr) {
      explanation->valid = true;
      explanation->cutoff_conflict = trail_resolution.cutoff_conflict;
      explanation->has_proved_target_bound =
          trail_resolution.has_proved_target_bound;
      explanation->proved_target_bound =
          trail_resolution.proved_target_bound;
      explanation->flipped_target = trail_resolution.flipped_target;
      explanation->initial_frontier = trail_resolution.proof_frontier;
      explanation->resolved_frontier = trail_resolution.resolved_frontier;
      explanation->clause = trail_resolution.clause;
      explanation->proof_margin = trail_resolution.proof_margin;
      explanation->proof_budget = trail_resolution.proof_budget;
      explanation->frontier_lp_activity =
          std::max(0.0, trail_resolution.resolved_activity -
                            (dual_proof->rhs -
                             trail_resolution.proof_budget));
      explanation->local_proof_cover_violation = best_violation;
      explanation->local_proof_cover_nnz =
          violated_local_proof_cover != nullptr &&
                  violated_local_proof_cover->coeff.size() == n
              ? violated_local_proof_cover->coeff.nonZeros()
              : 0;
      explanation->has_proof_activity_audit = true;
      explanation->proof_activity = trail_resolution.resolved_activity;
      explanation->proof_required_activity =
          trail_resolution.resolved_activity - trail_resolution.proof_margin;
    }
	    out_clause = trail_resolution.clause;
	    return true;
	  }

	  // Cross-frontier target-bound proofs are valid only when the
  // first-class local trail resolver produced the resolved reason-side
  // frontier.  Legacy unordered candidate fallbacks are deliberately absent:
  // they can publish a different frontier than the local trail actually
  // resolves to, which breaks HiGHS-style proof consumption.
  return false;

}

bool build_reduced_cost_cutoff_conflict_clause(
    const std::vector<VariableMeta>& vars,
    const Eigen::VectorXd& root_lb,
    const Eigen::VectorXd& root_ub,
    const Eigen::VectorXd& node_lb,
    const Eigen::VectorXd& node_ub,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& reduced_costs,
    const std::vector<int>& basis_indices,
    double node_bound,
    double incumbent_obj,
    double int_tol,
    const std::vector<BranchDomainLiteral>& branch_reasons,
    const std::vector<DomainReasonBound>& reason_bounds,
    const BranchDomainLiteral& forbidden,
    int max_literals,
    std::vector<BranchDomainLiteral>& out_clause,
    double* proof_margin,
    std::vector<std::vector<BranchDomainLiteral>>* reconvergence_clauses,
    PoolCut* violated_local_proof_cover,
    const DualProofRow* dual_proof) {
  out_clause.clear();
  if (reconvergence_clauses != nullptr) reconvergence_clauses->clear();
  if (violated_local_proof_cover != nullptr) *violated_local_proof_cover = PoolCut{};
  if (max_literals <= 0) return false;
  const int n = static_cast<int>(vars.size());
  if (n <= 0 || root_lb.size() < n || root_ub.size() < n ||
      node_lb.size() < n || node_ub.size() < n ||
      x_relax.size() < n || reduced_costs.size() < n) {
    return false;
  }
  if (forbidden.var_idx < 0 || forbidden.var_idx >= n) return false;
  const bool use_dual_proof =
      dual_proof != nullptr && dual_proof->valid &&
      dual_proof->coeff.size() >= n && std::isfinite(dual_proof->rhs);
  const double gap = incumbent_obj - node_bound;
  if (!use_dual_proof && (!std::isfinite(gap) || gap <= 1e-12)) return false;

  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int idx : basis_indices) {
    if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
  }

  std::vector<double> coeff(static_cast<std::size_t>(n), 0.0);

  if (use_dual_proof) {
    for (Eigen::SparseVector<double>::InnerIterator it(dual_proof->coeff); it; ++it) {
      const int j = static_cast<int>(it.index());
      if (j >= 0 && j < n && std::isfinite(it.value())) {
        coeff[static_cast<std::size_t>(j)] = it.value();
      }
    }
  } else {
    for (int j = 0; j < n; ++j) {
      if (is_basic[static_cast<std::size_t>(j)]) continue;
      const double rc_abs = std::abs(reduced_costs[j]);
      if (rc_abs <= 1e-12) continue;

      double a = 0.0;
      if (std::abs(x_relax[j] - node_lb[j]) <= int_tol) {
        a = rc_abs;
        if (!std::isfinite(root_lb[j]) || !std::isfinite(node_lb[j])) return false;
      } else if (std::abs(x_relax[j] - node_ub[j]) <= int_tol) {
        a = -rc_abs;
        if (!std::isfinite(root_ub[j]) || !std::isfinite(node_ub[j])) return false;
      } else {
        continue;
      }
      coeff[static_cast<std::size_t>(j)] = a;
    }
  }

  auto min_activity = [&](const Eigen::VectorXd& lb,
                          const Eigen::VectorXd& ub) -> double {
    double activity = 0.0;
    for (int j = 0; j < n; ++j) {
      const double a = coeff[static_cast<std::size_t>(j)];
      if (a > 0.0) {
        if (!std::isfinite(lb[j])) return -std::numeric_limits<double>::infinity();
        activity += a * lb[j];
      } else if (a < 0.0) {
        if (!std::isfinite(ub[j])) return -std::numeric_limits<double>::infinity();
        activity += a * ub[j];
      }
    }
    return activity;
  };

  auto max_activity = [&](const Eigen::VectorXd& lb,
                          const Eigen::VectorXd& ub) -> double {
    double activity = 0.0;
    for (int j = 0; j < n; ++j) {
      const double a = coeff[static_cast<std::size_t>(j)];
      if (a > 0.0) {
        if (!std::isfinite(ub[j])) return std::numeric_limits<double>::infinity();
        activity += a * ub[j];
      } else if (a < 0.0) {
        if (!std::isfinite(lb[j])) return std::numeric_limits<double>::infinity();
        activity += a * lb[j];
      }
    }
    return activity;
  };

  double root_min_activity = min_activity(root_lb, root_ub);
  double node_min_activity = min_activity(node_lb, node_ub);
  if (!std::isfinite(root_min_activity) || !std::isfinite(node_min_activity)) {
    return false;
  }

  double rhs = use_dual_proof ? dual_proof->rhs : (node_min_activity + gap);

  // HiGHS-style proof-row coefficient tightening: for a valid <= proof row,
  // coefficients of integer columns need not exceed max_activity - rhs in
  // magnitude.  Tightening before reason selection makes the activity budget
  // smaller and avoids selecting huge, numerically dominant proof terms.
  const double root_max_activity = max_activity(root_lb, root_ub);
  const double max_excess = root_max_activity - rhs;
  if (!use_dual_proof && std::isfinite(root_max_activity) && max_excess > 1e-7) {
    bool tightened = false;
    for (int j = 0; j < n; ++j) {
      if (!is_integer_type(vars[static_cast<std::size_t>(j)])) continue;
      double& a = coeff[static_cast<std::size_t>(j)];
      if (a > max_excess) {
        if (!std::isfinite(root_ub[j])) return false;
        const double delta = a - max_excess;
        rhs -= delta * root_ub[j];
        a = max_excess;
        tightened = true;
      } else if (a < -max_excess) {
        if (!std::isfinite(root_lb[j])) return false;
        const double delta = -a - max_excess;
        rhs += delta * root_lb[j];
        a = -max_excess;
        tightened = true;
      }
    }
    if (tightened) {
      root_min_activity = min_activity(root_lb, root_ub);
      node_min_activity = min_activity(node_lb, node_ub);
      if (!std::isfinite(root_min_activity) || !std::isfinite(node_min_activity)) {
        return false;
      }
    }
  }

  bool has_forbidden_coeff = false;
  const double forbidden_coeff = coeff[static_cast<std::size_t>(forbidden.var_idx)];
  if ((forbidden_coeff > 0.0 && forbidden.is_lb) ||
      (forbidden_coeff < 0.0 && !forbidden.is_lb)) {
    has_forbidden_coeff = true;
  }
  if (!has_forbidden_coeff) return false;

  auto literal_delta = [&](const BranchDomainLiteral& lit) -> double {
    if (lit.var_idx < 0 || lit.var_idx >= n) return 0.0;
    const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
    if (a > 0.0 && lit.is_lb) {
      if (!std::isfinite(root_lb[lit.var_idx])) return 0.0;
      return std::max(0.0, a * (lit.value - root_lb[lit.var_idx]));
    }
    if (a < 0.0 && !lit.is_lb) {
      if (!std::isfinite(root_ub[lit.var_idx])) return 0.0;
      return std::max(0.0, a * (lit.value - root_ub[lit.var_idx]));
    }
    return 0.0;
  };

  struct Candidate {
    std::vector<BranchDomainLiteral> proof_literals;
    std::vector<BranchDomainLiteral> reason;
    BranchDomainLiteral implied_bound;
    double delta{0.0};
    bool has_implied_bound{false};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(branch_reasons.size() + reason_bounds.size());
  for (const auto& lit : branch_reasons) {
    const double delta = literal_delta(lit);
    if (delta > 1e-10) candidates.push_back({{lit}, {lit}, lit, delta, false});
  }
  for (const auto& rb : reason_bounds) {
    if (rb.reason.empty()) continue;
    const double delta = literal_delta(rb.bound);
    if (delta > 1e-10) {
      candidates.push_back({{rb.bound}, rb.reason, rb.bound, delta, true});
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const Candidate& a, const Candidate& b) {
              if (a.delta != b.delta) return a.delta > b.delta;
              return a.proof_literals.size() < b.proof_literals.size();
            });

  std::vector<BranchDomainLiteral> proof_explained_clause;
  auto build_implied_bound_from_forbidden =
      [&](const BranchDomainLiteral& bad,
          BranchDomainLiteral& implied) -> bool {
    if (bad.var_idx < 0 || bad.var_idx >= n || !std::isfinite(bad.value)) {
      return false;
    }
    const bool integer_var =
        is_integer_type(vars[static_cast<std::size_t>(bad.var_idx)]);
    if (bad.is_lb) {
      implied = BranchDomainLiteral{
          bad.var_idx,
          integer_var ? std::ceil(bad.value - 1e-9) - 1.0
                      : bad.value - 1e-7,
          false};
    } else {
      implied = BranchDomainLiteral{
          bad.var_idx,
          integer_var ? std::floor(bad.value + 1e-9) + 1.0
                      : bad.value + 1e-7,
          true};
    }
    return true;
  };

  auto explain_bound_change_from_proof_row =
      [&](std::vector<BranchDomainLiteral>& explained_clause) -> bool {
    explained_clause.clear();
    BranchDomainLiteral implied;
    if (!build_implied_bound_from_forbidden(forbidden, implied)) return false;
    const int target = implied.var_idx;
    const double a0 = coeff[static_cast<std::size_t>(target)];
    if ((implied.is_lb && a0 >= -1e-12) ||
        (!implied.is_lb && a0 <= 1e-12)) {
      return false;
    }

    const bool integer_target =
        is_integer_type(vars[static_cast<std::size_t>(target)]);
    double relaxed_bound = implied.value;
    if (integer_target) {
      const double relax = std::max(0.0, 1.0 - 10.0 * int_tol);
      relaxed_bound += implied.is_lb ? -relax : relax;
    } else {
      relaxed_bound += implied.is_lb ? -1e-7 : 1e-7;
    }

    double target_global_contribution = 0.0;
    if (a0 > 0.0) {
      if (!std::isfinite(root_lb[target])) return false;
      target_global_contribution = a0 * root_lb[target];
    } else {
      if (!std::isfinite(root_ub[target])) return false;
      target_global_contribution = a0 * root_ub[target];
    }

    const double required_activity = rhs - a0 * relaxed_bound;
    double explained_activity = root_min_activity - target_global_contribution;
    std::vector<int> reason_indices;
    std::vector<BranchDomainLiteral> reason_literals;
    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const Candidate& cand = candidates[static_cast<std::size_t>(idx)];
      if (cand.delta <= 1e-10 || cand.proof_literals.empty()) continue;
      bool touches_target = false;
      for (const auto& lit : cand.proof_literals) {
        if (lit.var_idx == target) {
          touches_target = true;
          break;
        }
      }
      if (touches_target) continue;
      std::vector<BranchDomainLiteral> trial = reason_literals;
      trial.insert(trial.end(), cand.reason.begin(), cand.reason.end());
      canonicalize_branch_literals(trial);
      if (static_cast<int>(trial.size()) + 1 > max_literals) continue;
      reason_indices.push_back(idx);
      reason_literals.swap(trial);
      explained_activity += cand.delta;
      if (explained_activity >=
          required_activity - std::max(1e-7, 1e-10 * std::abs(required_activity))) {
        break;
      }
    }
    if (explained_activity <
        required_activity - std::max(1e-7, 1e-10 * std::abs(required_activity))) {
      return false;
    }

    bool changed_reason = true;
    while (changed_reason && !reason_indices.empty()) {
      changed_reason = false;
      for (std::size_t p = 0; p < reason_indices.size(); ++p) {
        const int idx = reason_indices[p];
        const double trial_activity =
            explained_activity - candidates[static_cast<std::size_t>(idx)].delta;
        if (trial_activity <
            required_activity - std::max(1e-7, 1e-10 * std::abs(required_activity))) {
          continue;
        }
        std::vector<int> trial_indices = reason_indices;
        trial_indices.erase(trial_indices.begin() + static_cast<std::ptrdiff_t>(p));
        std::vector<BranchDomainLiteral> trial_literals;
        for (int kept : trial_indices) {
          const auto& lits =
              candidates[static_cast<std::size_t>(kept)].reason;
          trial_literals.insert(trial_literals.end(), lits.begin(), lits.end());
        }
        canonicalize_branch_literals(trial_literals);
        if (static_cast<int>(trial_literals.size()) + 1 > max_literals) continue;
        reason_indices.swap(trial_indices);
        reason_literals.swap(trial_literals);
        explained_activity = trial_activity;
        changed_reason = true;
        break;
      }
    }

    explained_clause = std::move(reason_literals);
    explained_clause.push_back(forbidden);
    canonicalize_branch_literals(explained_clause);
    return !explained_clause.empty() &&
           static_cast<int>(explained_clause.size()) <= max_literals;
  };

  (void)explain_bound_change_from_proof_row(proof_explained_clause);

  std::vector<BranchDomainLiteral> clause;
  clause.reserve(static_cast<std::size_t>(std::min(max_literals,
                                                   static_cast<int>(candidates.size()) + 1)));
  clause.push_back(forbidden);
  canonicalize_branch_literals(clause);

  const double forbidden_delta = literal_delta(forbidden);
  std::vector<int> selected;
  selected.reserve(candidates.size());
  double selected_delta = 0.0;

  auto violated_by_delta = [&](double delta, double* margin_out = nullptr) -> bool {
    const double activity = root_min_activity + forbidden_delta + delta;
    const double scale = 1.0 + std::abs(activity) + std::abs(rhs);
    const double margin = activity - rhs;
    if (margin_out != nullptr) *margin_out = margin;
    return margin > std::max(1e-7, 1e-10 * scale);
  };

  double margin = 0.0;
  if (!violated_by_delta(selected_delta, &margin)) {
    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const auto& cand = candidates[static_cast<std::size_t>(idx)];
      std::vector<BranchDomainLiteral> trial = clause;
      trial.insert(trial.end(), cand.reason.begin(), cand.reason.end());
      canonicalize_branch_literals(trial);
      if (static_cast<int>(trial.size()) > max_literals) continue;
      selected.push_back(idx);
      selected_delta += cand.delta;
      clause.swap(trial);
      if (violated_by_delta(selected_delta, &margin)) break;
    }
  }

  if (!violated_by_delta(selected_delta, &margin)) {
    out_clause.clear();
    return false;
  }

  bool changed = true;
  while (changed && !selected.empty()) {
    changed = false;
    for (std::size_t p = 0; p < selected.size(); ++p) {
      const int idx = selected[p];
      const double trial_delta =
          selected_delta - candidates[static_cast<std::size_t>(idx)].delta;
      double trial_margin = 0.0;
      if (!violated_by_delta(trial_delta, &trial_margin)) continue;
      std::vector<int> trial_selected = selected;
      trial_selected.erase(trial_selected.begin() + static_cast<std::ptrdiff_t>(p));
      std::vector<BranchDomainLiteral> trial_clause{forbidden};
      for (int kept : trial_selected) {
        const auto& lits =
            candidates[static_cast<std::size_t>(kept)].reason;
        trial_clause.insert(trial_clause.end(), lits.begin(), lits.end());
      }
      canonicalize_branch_literals(trial_clause);
      if (static_cast<int>(trial_clause.size()) > max_literals) continue;
      selected.swap(trial_selected);
      selected_delta = trial_delta;
      clause.swap(trial_clause);
      margin = trial_margin;
      changed = true;
      break;
    }
  }

  canonicalize_branch_literals(clause);
  if (clause.empty() || static_cast<int>(clause.size()) > max_literals) {
    out_clause.clear();
    return false;
  }
  if (proof_margin != nullptr) *proof_margin = margin;
  out_clause = std::move(clause);
  std::vector<std::vector<BranchDomainLiteral>> extra_proof_conflicts;
  if (violated_local_proof_cover != nullptr) {
    struct ProofTerm {
      BranchDomainLiteral lit;
      double coeff{0.0};
      double root_bound{0.0};
    };
    auto add_proof_term = [&](const BranchDomainLiteral& lit,
                              std::vector<ProofTerm>& terms) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n) return false;
      const double a = coeff[static_cast<std::size_t>(lit.var_idx)];
      if (a > 0.0 && lit.is_lb) {
        if (!std::isfinite(root_lb[lit.var_idx])) return false;
        terms.push_back({lit, a, root_lb[lit.var_idx]});
        return true;
      }
      if (a < 0.0 && !lit.is_lb) {
        if (!std::isfinite(root_ub[lit.var_idx])) return false;
        terms.push_back({lit, a, root_ub[lit.var_idx]});
        return true;
      }
      return false;
    };

    auto emit_if_violated = [&](const std::vector<ProofTerm>& terms) {
      if (violated_local_proof_cover->coeff.nonZeros() > 0 || terms.empty()) {
        return false;
      }
      Eigen::SparseVector<double> coeff(n);
      coeff.reserve(static_cast<int>(terms.size()));
      double cover_rhs = rhs - root_min_activity;
      double norm2 = 0.0;
      for (const auto& term : terms) {
        coeff.coeffRef(term.lit.var_idx) += term.coeff;
        cover_rhs += term.coeff * term.root_bound;
        norm2 += term.coeff * term.coeff;
      }
      coeff.prune(0.0);
      const double lhs = coeff.dot(x_relax);
      const double scale = 1.0 + std::abs(lhs) + std::abs(cover_rhs);
      if (lhs > cover_rhs + std::max(1e-7, 1e-10 * scale)) {
        *violated_local_proof_cover =
            PoolCut{std::move(coeff), cover_rhs, 0, 0.0, std::sqrt(norm2), 0};
      }
      return true;
    };

    auto build_weighted_from_indices =
        [&](const std::vector<int>& indices, bool include_forbidden) {
      std::vector<ProofTerm> terms;
      terms.reserve(indices.size() + (include_forbidden ? 1 : 0));
      bool proof_cover_valid = true;
      if (include_forbidden) {
        proof_cover_valid = add_proof_term(forbidden, terms);
      }
      for (int idx : indices) {
        const Candidate& cand = candidates[static_cast<std::size_t>(idx)];
        for (const auto& lit : cand.proof_literals) {
          proof_cover_valid =
              add_proof_term(lit, terms) && proof_cover_valid;
        }
      }
      if (proof_cover_valid) emit_if_violated(terms);
    };

    const double proof_budget = rhs - root_min_activity;
    std::vector<int> active_cover;
    std::vector<BranchDomainLiteral> active_literals;
    double active_delta = 0.0;
    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const Candidate& cand = candidates[static_cast<std::size_t>(idx)];
      if (cand.proof_literals.empty()) continue;
      std::vector<BranchDomainLiteral> trial = active_literals;
      trial.insert(trial.end(), cand.proof_literals.begin(),
                   cand.proof_literals.end());
      canonicalize_branch_literals(trial);
      if (static_cast<int>(trial.size()) > max_literals) continue;
      active_cover.push_back(idx);
      active_literals.swap(trial);
      active_delta += cand.delta;
      if (active_delta > proof_budget + std::max(1e-7, 1e-10 * std::abs(proof_budget))) {
        break;
      }
    }
    if (active_delta > proof_budget + std::max(1e-7, 1e-10 * std::abs(proof_budget))) {
      bool changed_cover = true;
      while (changed_cover && !active_cover.empty()) {
        changed_cover = false;
        for (std::size_t p = 0; p < active_cover.size(); ++p) {
          const int idx = active_cover[p];
          const double trial_delta =
              active_delta - candidates[static_cast<std::size_t>(idx)].delta;
          if (trial_delta <=
              proof_budget + std::max(1e-7, 1e-10 * std::abs(proof_budget))) {
            continue;
          }
          std::vector<int> trial_cover = active_cover;
          trial_cover.erase(trial_cover.begin() + static_cast<std::ptrdiff_t>(p));
          std::vector<BranchDomainLiteral> trial_literals;
          for (int kept : trial_cover) {
            const auto& lits =
                candidates[static_cast<std::size_t>(kept)].proof_literals;
            trial_literals.insert(trial_literals.end(), lits.begin(), lits.end());
          }
          canonicalize_branch_literals(trial_literals);
          if (static_cast<int>(trial_literals.size()) > max_literals) continue;
          active_cover.swap(trial_cover);
          active_literals.swap(trial_literals);
          active_delta = trial_delta;
          changed_cover = true;
          break;
        }
      }
      build_weighted_from_indices(active_cover, false);
      std::vector<BranchDomainLiteral> cutoff_reason_clause;
      for (int kept : active_cover) {
        const auto& reason =
            candidates[static_cast<std::size_t>(kept)].reason;
        cutoff_reason_clause.insert(cutoff_reason_clause.end(),
                                    reason.begin(), reason.end());
      }
      canonicalize_branch_literals(cutoff_reason_clause);
      if (!cutoff_reason_clause.empty() &&
          static_cast<int>(cutoff_reason_clause.size()) <= max_literals) {
        extra_proof_conflicts.push_back(std::move(cutoff_reason_clause));
      }
    }
    build_weighted_from_indices(selected, true);
  }
  if (reconvergence_clauses != nullptr) {
    reconvergence_clauses->insert(reconvergence_clauses->end(),
                                  extra_proof_conflicts.begin(),
                                  extra_proof_conflicts.end());
    if (!proof_explained_clause.empty()) {
      reconvergence_clauses->push_back(proof_explained_clause);
    }
    auto flipped_literal = [&](const BranchDomainLiteral& lit,
                               BranchDomainLiteral& out) -> bool {
      if (lit.var_idx < 0 || lit.var_idx >= n ||
          !std::isfinite(lit.value)) {
        return false;
      }
      const bool integer_var =
          is_integer_type(vars[static_cast<std::size_t>(lit.var_idx)]);
      if (lit.is_lb) {
        out = BranchDomainLiteral{
            lit.var_idx,
            integer_var ? std::ceil(lit.value - 1e-9) - 1.0 : lit.value - 1e-7,
            false};
      } else {
        out = BranchDomainLiteral{
            lit.var_idx,
            integer_var ? std::floor(lit.value + 1e-9) + 1.0 : lit.value + 1e-7,
            true};
      }
      return true;
    };

    auto find_reason = [&](const BranchDomainLiteral& lit)
        -> const std::vector<BranchDomainLiteral>* {
      const std::vector<BranchDomainLiteral>* best = nullptr;
      for (const auto& rb : reason_bounds) {
        if (rb.bound.var_idx != lit.var_idx || rb.bound.is_lb != lit.is_lb) {
          continue;
        }
        const bool covers = lit.is_lb
            ? (rb.bound.value >= lit.value - 1e-9)
            : (rb.bound.value <= lit.value + 1e-9);
        if (!covers || rb.reason.empty()) continue;
        if (best == nullptr || rb.reason.size() < best->size()) best = &rb.reason;
      }
      return best;
    };

    auto recursive_reason_frontier = [&](std::vector<BranchDomainLiteral> frontier) {
      canonicalize_branch_literals(frontier);
      std::vector<BranchDomainLiteral> best = frontier;
      constexpr int kMaxResolve = 12;
      for (int iter = 0; iter < kMaxResolve; ++iter) {
        int pos = -1;
        const std::vector<BranchDomainLiteral>* repl = nullptr;
        for (int i = static_cast<int>(frontier.size()) - 1; i >= 0; --i) {
          const auto* reason = find_reason(frontier[static_cast<std::size_t>(i)]);
          if (reason == nullptr) continue;
          pos = i;
          repl = reason;
          break;
        }
        if (pos < 0 || repl == nullptr) break;
        std::vector<BranchDomainLiteral> trial;
        trial.reserve(frontier.size() + repl->size());
        for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
          if (i == pos) continue;
          trial.push_back(frontier[static_cast<std::size_t>(i)]);
        }
        trial.insert(trial.end(), repl->begin(), repl->end());
        canonicalize_branch_literals(trial);
        if (trial.empty() || static_cast<int>(trial.size()) > max_literals) break;
        frontier.swap(trial);
        if (frontier.size() < best.size()) best = frontier;
      }
      return best;
    };

    for (int idx = 0; idx < static_cast<int>(candidates.size()); ++idx) {
      const auto& cand = candidates[static_cast<std::size_t>(idx)];
      if (!cand.has_implied_bound || cand.reason.empty()) continue;
      BranchDomainLiteral flipped;
      if (!flipped_literal(cand.implied_bound, flipped)) continue;
      std::vector<BranchDomainLiteral> reason =
          recursive_reason_frontier(cand.reason);
      std::vector<BranchDomainLiteral> reconv = reason;
      reconv.push_back(flipped);
      canonicalize_branch_literals(reconv);
      if (reconv.empty() ||
          static_cast<int>(reconv.size()) > std::min(max_literals, 32)) {
        continue;
      }
      bool duplicate = false;
      for (const auto& existing : *reconvergence_clauses) {
        if (existing.size() != reconv.size()) continue;
        bool same = true;
        for (std::size_t i = 0; i < existing.size(); ++i) {
          if (existing[i].var_idx != reconv[i].var_idx ||
              existing[i].is_lb != reconv[i].is_lb ||
              std::abs(existing[i].value - reconv[i].value) > 1e-9) {
            same = false;
            break;
          }
        }
        if (same) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        reconvergence_clauses->push_back(std::move(reconv));
        if (reconvergence_clauses->size() >= 2) break;
      }
    }
  }
  return true;
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
        double min_activity = 0.0;
        bool has_inf = false;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          if (a > 0.0) {
            if (!std::isfinite(node_lb[j])) { has_inf = true; break; }
            min_activity += a * node_lb[j];
          } else {
            if (!std::isfinite(node_ub[j])) { has_inf = true; break; }
            min_activity += a * node_ub[j];
          }
        }
        if (has_inf) return;

        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * node_lb[j] : a * node_ub[j];
          const double residual = rhs - (min_activity - contrib);
          if (a > 0.0) {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - 1e-9) {
              node_ub[j] = new_ub;
              ++round_tightened;
              mark_col_dirty(j);
            }
          } else {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + 1e-9) {
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
      double min_activity = 0.0, max_activity = 0.0;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(node_lb[j])) has_inf_min = true;
          else min_activity += a * node_lb[j];
          if (!std::isfinite(node_ub[j])) has_inf_max = true;
          else max_activity += a * node_ub[j];
        } else {
          if (!std::isfinite(node_ub[j])) has_inf_min = true;
          else min_activity += a * node_ub[j];
          if (!std::isfinite(node_lb[j])) has_inf_max = true;
          else max_activity += a * node_lb[j];
        }
      }

      if (!has_inf_min) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * node_lb[j] : a * node_ub[j];
          const double residual = lp.beq[r] - (min_activity - contrib);
          if (a > 0.0) {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - 1e-9) {
              node_ub[j] = new_ub;
              ++round_tightened;
              mark_col_dirty(j);
            }
          } else {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + 1e-9) {
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
          const double contrib = (a > 0.0) ? a * node_ub[j] : a * node_lb[j];
          const double residual = lp.beq[r] - (max_activity - contrib);
          if (a > 0.0) {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + 1e-9) {
              node_lb[j] = new_lb;
              ++round_tightened;
              mark_col_dirty(j);
            }
          } else {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - 1e-9) {
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

  for (int round = 0; round < max_rounds; ++round) {
    int round_tightened = 0;

    for (int r = 0; r < m_ineq; ++r) {
      double min_activity = 0.0;
      bool has_inf = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(node_lb[j])) { has_inf = true; break; }
          min_activity += a * node_lb[j];
        } else {
          if (!std::isfinite(node_ub[j])) { has_inf = true; break; }
          min_activity += a * node_ub[j];
        }
      }
      if (has_inf) continue;

      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        const double contrib = (a > 0.0) ? a * node_lb[j] : a * node_ub[j];
        const double residual = lp.b[r] - (min_activity - contrib);
        if (a > 0.0) {
          double new_ub = residual / a;
          if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
          if (new_ub < node_ub[j] - 1e-9) {
            changes_out.push_back({j, new_ub - node_ub[j], false});
            node_ub[j] = new_ub;
            ++round_tightened;
          }
        } else {
          double new_lb = residual / a;
          if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
          if (new_lb > node_lb[j] + 1e-9) {
            changes_out.push_back({j, new_lb - node_lb[j], true});
            node_lb[j] = new_lb;
            ++round_tightened;
          }
        }
      }
    }

    for (int r = 0; r < m_eq; ++r) {
      double min_activity = 0.0, max_activity = 0.0;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(node_lb[j])) has_inf_min = true;
          else min_activity += a * node_lb[j];
          if (!std::isfinite(node_ub[j])) has_inf_max = true;
          else max_activity += a * node_ub[j];
        } else {
          if (!std::isfinite(node_ub[j])) has_inf_min = true;
          else min_activity += a * node_ub[j];
          if (!std::isfinite(node_lb[j])) has_inf_max = true;
          else max_activity += a * node_lb[j];
        }
      }

      if (!has_inf_min) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * node_lb[j] : a * node_ub[j];
          const double residual = lp.beq[r] - (min_activity - contrib);
          if (a > 0.0) {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - 1e-9) {
              changes_out.push_back({j, new_ub - node_ub[j], false});
              node_ub[j] = new_ub;
              ++round_tightened;
            }
          } else {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + 1e-9) {
              changes_out.push_back({j, new_lb - node_lb[j], true});
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
          const double contrib = (a > 0.0) ? a * node_ub[j] : a * node_lb[j];
          const double residual = lp.beq[r] - (max_activity - contrib);
          if (a > 0.0) {
            double new_lb = residual / a;
            if (is_integer_type(lp.vars[j])) new_lb = std::ceil(new_lb - 1e-9);
            if (new_lb > node_lb[j] + 1e-9) {
              changes_out.push_back({j, new_lb - node_lb[j], true});
              node_lb[j] = new_lb;
              ++round_tightened;
            }
          } else {
            double new_ub = residual / a;
            if (is_integer_type(lp.vars[j])) new_ub = std::floor(new_ub + 1e-9);
            if (new_ub < node_ub[j] - 1e-9) {
              changes_out.push_back({j, new_ub - node_ub[j], false});
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
            changes_out->push_back({j, new_lb - lb[j], true});
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
            changes_out->push_back({j, new_ub - ub[j], false});
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
                             std::vector<DomainReasonBound>* reason_bounds_out) {
  if (conflict_pool == nullptr) return true;
  std::vector<BoundChangeInfo> clause_changes;
  int local_tightened = 0;
  std::vector<DomainReasonBound> clause_reason_bounds;
  const bool ok = reason_bounds_out != nullptr
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
  if (reason_bounds_out != nullptr) {
    reason_bounds_out->insert(reason_bounds_out->end(),
                              clause_reason_bounds.begin(),
                              clause_reason_bounds.end());
  }
  if (!ok) {
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
    const std::vector<DomainReasonBound>* existing_reason_bounds) {
  PropagationProfile profile;
  profile.calls = 1;
  const auto _prop_t0 = std::chrono::steady_clock::now();
  const int n = static_cast<int>(lp.vars.size());
  ReasonArena arena;
  std::vector<int> lb_reason(static_cast<std::size_t>(n), 0);
  std::vector<int> ub_reason(static_cast<std::size_t>(n), 0);
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
    const int rid = arena.singleton(lit);
    if (lit.var_idx >= 0 && lit.var_idx < n) {
      if (lit.is_lb) lb_reason[static_cast<std::size_t>(lit.var_idx)] = rid;
      else ub_reason[static_cast<std::size_t>(lit.var_idx)] = rid;
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
      enqueue_var(j);
    }
  }
  const std::size_t input_change_count = changes_out.size();
  for (std::size_t ci = 0; ci < input_change_count; ++ci) {
    const int j = changes_out[ci].var_idx;
    if (j >= 0 && j < n) {
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
      enqueue_var(j);
    }
  }

  int local_tightened = 0;
  int conflict_var = -1;
  const int max_passes = std::max(1, 2 * max_rounds + 4);
  const std::uint64_t full_scan_rows =
      static_cast<std::uint64_t>(A_row.rows()) + static_cast<std::uint64_t>(Aeq_row.rows());

  auto finalize_profile = [&]() {
    profile.bound_tightenings = static_cast<std::uint64_t>(std::max(0, local_tightened));
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
                         const LinearProofAudit* proof_audit = nullptr) -> bool {
    if (j < 0 || j >= n) return true;
    if (is_integer_type(lp.vars[static_cast<std::size_t>(j)])) {
      new_val = is_lb ? std::ceil(new_val - 1e-9) : std::floor(new_val + 1e-9);
    }
    if (is_lb) {
      if (new_val > node_lb[j] + 1e-9) {
        changes_out.push_back({j, new_val - node_lb[j], true});
        node_lb[j] = new_val;
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
        lb_reason[static_cast<std::size_t>(j)] = reason_id;
        ++local_tightened;
        enqueue_var(j);
      }
    } else {
      if (new_val < node_ub[j] - 1e-9) {
        changes_out.push_back({j, new_val - node_ub[j], false});
        node_ub[j] = new_val;
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
        ub_reason[static_cast<std::size_t>(j)] = reason_id;
        ++local_tightened;
        enqueue_var(j);
      }
    }
    if (node_lb[j] > node_ub[j] + 1e-9) {
      conflict_var = j;
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
                               &local_tightened, reason_bounds_out)) {
    if (learned_conflict != nullptr) *learned_conflict = branch_reasons;
    if (total_tightened != nullptr) *total_tightened += local_tightened;
    finalize_profile();
    return false;
  }

  for (int pass = 0; pass < max_passes; ++pass) {
    if (changed_vars.empty() && pending_ineq_rows.empty() && pending_eq_rows.empty()) {
      break;
    }
    ++profile.active_passes;
    profile.baseline_full_scan_rows += full_scan_rows;

    while (!changed_vars.empty()) {
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
          const int trigger_rid = value_one ? lb_reason[static_cast<std::size_t>(j)]
                                            : ub_reason[static_cast<std::size_t>(j)];
          auto lit_rng = clique_table->literal_neighbours(j, value_one);
          for (const int* p = lit_rng.first; p != lit_rng.second; ++p) {
            const int forbidden = *p;
            const int implied_col = forbidden / 2;
            const bool forbidden_one = (forbidden % 2) != 0;
            std::vector<int> ids{trigger_rid};
            const int rid = arena.merge_ids(ids);
            if (forbidden_one) {
              if (!apply_bound(implied_col, false, 0.0, rid)) break;
            } else {
              if (!apply_bound(implied_col, true, 1.0, rid)) break;
            }
          }
          if (conflict_var >= 0) break;

          if (value_one) {
            auto rng = clique_table->neighbours(j);
            for (const int* p = rng.first; p != rng.second; ++p) {
              std::vector<int> ids{trigger_rid};
              const int rid = arena.merge_ids(ids);
              if (!apply_bound(*p, false, 0.0, rid)) break;
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
          const int trigger_rid = value_one ? lb_reason[static_cast<std::size_t>(j)]
                                            : ub_reason[static_cast<std::size_t>(j)];
          for (const auto* arc = rng.first; arc != rng.second; ++arc) {
            std::vector<int> ids{trigger_rid};
            const int rid = arena.merge_ids(ids);
            if (!apply_bound(arc->var_idx, arc->is_lb, arc->value, rid)) break;
          }
          if (conflict_var >= 0) break;
        }
        if (conflict_var >= 0) break;
      }

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
    if (conflict_var >= 0) break;

    while (!pending_ineq_rows.empty()) {
      const int r = pending_ineq_rows.front();
      pending_ineq_rows.pop_front();
      queued_ineq[static_cast<std::size_t>(r)] = 0;
      ++profile.ineq_rows_visited;

      auto propagate_upper_side = [&](double row_sign, double rhs) {
        if (!std::isfinite(rhs) || conflict_var >= 0) return;
        double min_activity = 0.0;
        bool has_inf = false;
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          if (a > 0.0) {
            if (!std::isfinite(node_lb[j])) { has_inf = true; break; }
            min_activity += a * node_lb[j];
          } else {
            if (!std::isfinite(node_ub[j])) { has_inf = true; break; }
            min_activity += a * node_ub[j];
          }
        }
        if (has_inf) return;

        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(A_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = row_sign * it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * node_lb[j] : a * node_ub[j];
          const double residual = rhs - (min_activity - contrib);
          std::vector<int> ids;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(A_row, r); jt; ++jt) {
            const int k = static_cast<int>(jt.col());
            const double ak = row_sign * jt.value();
            if (k == j || std::abs(ak) <= 1e-15) continue;
            const int rid = (ak > 0.0) ? lb_reason[static_cast<std::size_t>(k)]
                                       : ub_reason[static_cast<std::size_t>(k)];
            if (rid > 0) ids.push_back(rid);
          }
          const int rid = arena.merge_ids(ids);
          if (a > 0.0) {
            const double new_ub = normalized_bound_value(j, false, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, false, new_ub);
            const LinearProofAudit audit{
                source, min_activity - contrib + a * source.value, rhs};
            if (!apply_bound(j, false, new_ub, rid, &audit)) break;
          } else {
            const double new_lb = normalized_bound_value(j, true, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, true, new_lb);
            const LinearProofAudit audit{
                source, min_activity - contrib + a * source.value, rhs};
            if (!apply_bound(j, true, new_lb, rid, &audit)) break;
          }
        }
      };
      propagate_upper_side(1.0, lp.b[r]);
      const double lhs = lp_row_lhs_or_neg_inf(lp, r);
      if (std::isfinite(lhs)) propagate_upper_side(-1.0, -lhs);
      if (conflict_var >= 0) break;
    }
    if (conflict_var >= 0) break;

    while (!pending_eq_rows.empty()) {
      const int r = pending_eq_rows.front();
      pending_eq_rows.pop_front();
      queued_eq[static_cast<std::size_t>(r)] = 0;
      ++profile.eq_rows_visited;

      double min_activity = 0.0, max_activity = 0.0;
      bool has_inf_min = false, has_inf_max = false;
      for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
        const int j = static_cast<int>(it.col());
        const double a = it.value();
        if (std::abs(a) <= 1e-15) continue;
        if (a > 0.0) {
          if (!std::isfinite(node_lb[j])) has_inf_min = true;
          else min_activity += a * node_lb[j];
          if (!std::isfinite(node_ub[j])) has_inf_max = true;
          else max_activity += a * node_ub[j];
        } else {
          if (!std::isfinite(node_ub[j])) has_inf_min = true;
          else min_activity += a * node_ub[j];
          if (!std::isfinite(node_lb[j])) has_inf_max = true;
          else max_activity += a * node_lb[j];
        }
      }

      if (!has_inf_min) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * node_lb[j] : a * node_ub[j];
          const double residual = lp.beq[r] - (min_activity - contrib);
          std::vector<int> ids;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(Aeq_row, r); jt; ++jt) {
            const int k = static_cast<int>(jt.col());
            const double ak = jt.value();
            if (k == j || std::abs(ak) <= 1e-15) continue;
            const int rid = (ak > 0.0) ? lb_reason[static_cast<std::size_t>(k)]
                                       : ub_reason[static_cast<std::size_t>(k)];
            if (rid > 0) ids.push_back(rid);
          }
          const int rid = arena.merge_ids(ids);
          if (a > 0.0) {
            const double new_ub = normalized_bound_value(j, false, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, false, new_ub);
            const LinearProofAudit audit{
                source, min_activity - contrib + a * source.value, lp.beq[r]};
            if (!apply_bound(j, false, new_ub, rid, &audit)) break;
          } else {
            const double new_lb = normalized_bound_value(j, true, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, true, new_lb);
            const LinearProofAudit audit{
                source, min_activity - contrib + a * source.value, lp.beq[r]};
            if (!apply_bound(j, true, new_lb, rid, &audit)) break;
          }
        }
      }
      if (conflict_var >= 0) break;

      if (!has_inf_max) {
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(Aeq_row, r); it; ++it) {
          const int j = static_cast<int>(it.col());
          const double a = it.value();
          if (std::abs(a) <= 1e-15) continue;
          const double contrib = (a > 0.0) ? a * node_ub[j] : a * node_lb[j];
          const double residual = lp.beq[r] - (max_activity - contrib);
          std::vector<int> ids;
          for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator jt(Aeq_row, r); jt; ++jt) {
            const int k = static_cast<int>(jt.col());
            const double ak = jt.value();
            if (k == j || std::abs(ak) <= 1e-15) continue;
            const int rid = (ak > 0.0) ? ub_reason[static_cast<std::size_t>(k)]
                                       : lb_reason[static_cast<std::size_t>(k)];
            if (rid > 0) ids.push_back(rid);
          }
          const int rid = arena.merge_ids(ids);
          if (a > 0.0) {
            const double new_lb = normalized_bound_value(j, true, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, true, new_lb);
            const double source_activity =
                max_activity - contrib + a * source.value;
            const LinearProofAudit audit{source, -source_activity, -lp.beq[r]};
            if (!apply_bound(j, true, new_lb, rid, &audit)) break;
          } else {
            const double new_ub = normalized_bound_value(j, false, residual / a);
            const BranchDomainLiteral source =
                opposite_source_literal(j, false, new_ub);
            const double source_activity =
                max_activity - contrib + a * source.value;
            const LinearProofAudit audit{source, -source_activity, -lp.beq[r]};
            if (!apply_bound(j, false, new_ub, rid, &audit)) break;
          }
        }
      }
      if (conflict_var >= 0) break;
    }
    if (conflict_var >= 0) break;

    if (!clause_propagation_step(conflict_pool, lp.vars, node_lb, node_ub,
                                 arena, lb_reason, ub_reason,
                                 changes_out, changed_vars, queued_vars,
                                 static_cast<int>(branch_reasons.size()),
                                 &local_tightened, reason_bounds_out)) {
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
    const std::vector<DomainReasonBound>* existing_reason_bounds) {
  return propagate_node_domain_impl(lp, A_row, Aeq_row, row_index, node_lb, node_ub,
                                    max_rounds, conflict_pool, clique_table,
                                    implication_graph, changes_out,
                                    branch_reasons, learned_conflict,
                                    total_tightened, reason_bounds_out,
                                    existing_reason_bounds);
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
    const std::vector<DomainReasonBound>* existing_reason_bounds) {
  return propagate_node_domain_impl(lp, A_row, Aeq_row, row_index, node_lb, node_ub,
                                    max_rounds, conflict_pool, clique_table,
                                    implication_graph, changes_out,
                                    branch_reasons, learned_conflict,
                                    total_tightened, reason_bounds_out,
                                    existing_reason_bounds);
}

}  // namespace mipsolvers::engine::detail
