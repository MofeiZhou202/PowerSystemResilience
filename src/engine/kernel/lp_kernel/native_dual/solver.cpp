#include "certificate.hpp"
#include "primal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <iomanip>
#include <sstream>
#include <string>
#include <utility>

namespace mipsolvers::engine::native_dual {
namespace {

using detail::Audit;
using detail::BoundFlip;
using detail::Bounds;
using detail::Entering;
using detail::Leaving;
using detail::Move;
using detail::Phase;
using detail::PivotTransaction;
using detail::State;

Result empty_result(Status status, std::string message,
                    Statistics statistics = {}) {
  Result result;
  result.status = status;
  result.message = std::move(message);
  result.statistics = statistics;
  return result;
}

bool wall_time_hit(const SimplexOptions& options,
                   const std::chrono::steady_clock::time_point& start) {
  if (options.time_limit_hit != nullptr && *options.time_limit_hit) return true;
  if (options.time_limit_sec <= 0.0) return false;
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  if (elapsed < options.time_limit_sec) return false;
  if (options.time_limit_hit != nullptr) *options.time_limit_hit = true;
  return true;
}

void update_statistics(const Audit& audit, Statistics& statistics) {
  statistics.max_primal_infeasibility = audit.max_primal_infeasibility;
  statistics.max_dual_infeasibility = audit.max_dual_infeasibility;
}

void attach_farkas_certificate(const detail::Certificate& certificate,
                               Result& result) {
  result.has_farkas_certificate = certificate.valid;
  result.farkas_multiplier = certificate.multiplier;
  result.farkas_margin = certificate.margin;
  result.farkas_error_bound = certificate.error_bound;
}

enum class MinorKind {
  Pivoted,
  CycleBlocked,
  PossiblyOptimal,
  PossiblyPrimalInfeasible,
  NumericalTrouble,
};

struct MinorOutcome {
  MinorKind kind{MinorKind::NumericalTrouble};
  std::string message;
  Leaving leaving;
};

MinorOutcome numerical_trouble(std::string message) {
  MinorOutcome outcome;
  outcome.kind = MinorKind::NumericalTrouble;
  outcome.message = std::move(message);
  return outcome;
}

// ── Per-phase profiling (env MIPSOLVERS_DS_PROFILE) ──────────────────────────
// Accumulates wall time in each dual-simplex phase so a slow large-LP root can
// be attributed to CHUZR/BTRAN, PRICE (A^T*row_ep), the ratio test, FTRAN, DSE
// weight updates, or LU refactorization.  Zero cost when the env is unset.
struct DSProfile {
  bool enabled = false;
  double start_clock = 0.0;
  double leaving = 0.0, price = 0.0, entering = 0.0, ftran = 0.0, dse = 0.0,
         rebuild = 0.0, minor_total = 0.0, primal = 0.0, edge_init = 0.0,
         postcond = 0.0, rc_update = 0.0, valid = 0.0, lu_update = 0.0,
         cycle = 0.0;
  long pivots = 0, rebuilds = 0;
  int model_m = 0, model_n = 0;
  // Average structural support of the two hot simplex vectors.
  double sum_rowep_nnz = 0.0, sum_pivotrow_nnz = 0.0;
  long density_samples = 0;
  void reset() {
    enabled = std::getenv("MIPSOLVERS_DS_PROFILE") != nullptr;
    leaving = price = entering = ftran = dse = rebuild = minor_total = primal =
        edge_init = postcond = rc_update = valid = lu_update = cycle = 0.0;
    sum_rowep_nnz = sum_pivotrow_nnz = 0.0;
    density_samples = 0;
    pivots = rebuilds = 0;
    model_m = model_n = 0;
    start_clock = enabled ? std::chrono::duration<double>(
                                std::chrono::steady_clock::now()
                                    .time_since_epoch())
                                .count()
                          : 0.0;
  }
  void report(int m, int n) const {
    if (!enabled) return;
    const double wall =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count() -
        start_clock;
    const double phases = leaving + price + entering + ftran + dse;
    const double other = minor_total - phases;
    std::fprintf(stderr,
                 "[DS-PROFILE] m=%d n=%d wall=%.2fs pivots=%ld rebuilds=%ld "
                 "minor=%.2f rebuild=%.2f primalPhaseI=%.2f edgeInit=%.2f | "
                 "leaving(CHUZR+BTRAN)=%.2f price(A^T*rEP)=%.2f "
                 "entering(ratio/BFRT)=%.2f ftran=%.2f dse=%.2f "
                 "postcond(dualfeas)=%.2f rcUpdate=%.2f "
                 "validBlock(predFull+resid)=%.2f luUpdate=%.2f "
                 "cycleGuard=%.2f OTHER(update/copy)=%.2f\n",
                 m, n, wall, pivots, rebuilds, minor_total, rebuild, primal,
                 edge_init, leaving, price, entering, ftran, dse, postcond,
                 rc_update, valid, lu_update, cycle,
                 other - postcond - rc_update - valid - lu_update - cycle);
    if (density_samples > 0) {
      const double avg_rowep = sum_rowep_nnz / static_cast<double>(density_samples);
      const double avg_pivotrow =
          sum_pivotrow_nnz / static_cast<double>(density_samples);
      std::fprintf(stderr,
                   "[DS-DENSITY] samples=%ld rowEP_nnz=%.1f (%.2f%% of m=%d) "
                   "pivotRow_nnz=%.1f (%.2f%% of n=%d)\n",
                   density_samples, avg_rowep,
                   m > 0 ? 100.0 * avg_rowep / m : 0.0, m, avg_pivotrow,
                   n > 0 ? 100.0 * avg_pivotrow / n : 0.0, n);
    }
  }
};
thread_local DSProfile g_ds_profile;
inline double ds_clock() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct MinorScratch {
  std::vector<std::pair<int, double>> primal_changes;
  std::vector<detail::EdgeWeightChange> edge_weight_changes;
};
thread_local MinorScratch g_ds_scratch;

// The pivot's move/basis mutations are applied to `state` in place (instead of
// copy-modify-commit on O(n)/O(m) vectors).  Every numerical-trouble return
// between the mutation and the commit must restore the pre-pivot state exactly
// — the driver rebuilds from `state.basis`/`state.move` after a failure, so a
// partially applied exchange would corrupt the iterate.  Restoration is exact
// (the old values are integers/enums, not recomputed floats).
class PivotStateGuard {
 public:
  PivotStateGuard(State& state, const PivotTransaction& transaction,
                  int leaving_row, int leaving_col, Move old_leaving_move,
                  int entering_col, Move old_entering_move)
      : state_(state),
        transaction_(transaction),
        leaving_row_(leaving_row),
        leaving_col_(leaving_col),
        old_leaving_move_(old_leaving_move),
        entering_col_(entering_col),
        old_entering_move_(old_entering_move) {}
  ~PivotStateGuard() {
    if (dismissed_) return;
    state_.basis[static_cast<std::size_t>(leaving_row_)] = leaving_col_;
    state_.move[static_cast<std::size_t>(leaving_col_)] = old_leaving_move_;
    state_.move[static_cast<std::size_t>(entering_col_)] = old_entering_move_;
    for (const BoundFlip& flip : transaction_.flips) {
      if (flip.col >= 0 && flip.col < state_.n) {
        state_.move[static_cast<std::size_t>(flip.col)] = flip.old_move;
      }
    }
  }
  void dismiss() { dismissed_ = true; }
  PivotStateGuard(const PivotStateGuard&) = delete;
  PivotStateGuard& operator=(const PivotStateGuard&) = delete;

 private:
  State& state_;
  const PivotTransaction& transaction_;
  int leaving_row_;
  int leaving_col_;
  Move old_leaving_move_;
  int entering_col_;
  Move old_entering_move_;
  bool dismissed_{false};
};

MinorOutcome minor_iteration(State& state, Statistics& statistics) {
  ++state.pricing_epoch;
  Leaving leaving;
  std::string failure;
  const double _t_lv = g_ds_profile.enabled ? ds_clock() : 0.0;
  const bool _lv_ok = detail::choose_leaving(state, leaving, failure);
  if (g_ds_profile.enabled) g_ds_profile.leaving += ds_clock() - _t_lv;
  if (!_lv_ok) {
    return numerical_trouble(std::move(failure));
  }
  statistics.taboo_row_rejections += leaving.taboo_row_rejections;
  if (leaving.released_taboo_row && leaving.row >= 0) {
    detail::release_taboo_row(
        state, state.basis[static_cast<std::size_t>(leaving.row)], statistics);
  }
  if (leaving.row_ep_refined) ++statistics.iterative_refinements;
  if (leaving.row < 0) {
    MinorOutcome outcome;
    outcome.kind = MinorKind::PossiblyOptimal;
    return outcome;
  }

  const double _t_pr = g_ds_profile.enabled ? ds_clock() : 0.0;
  if (state.sf->A_row.rows() != state.m ||
      state.sf->A_row.cols() != state.n) {
    return numerical_trouble("PRICE row matrix is dimensionally inconsistent");
  }
  const detail::IndexedVector pivot_row =
      detail::multiply_AT_indexed(state.sf->A_row, leaving.row_ep);
  if (!pivot_row.finite()) {
    return numerical_trouble("packed PRICE produced non-finite values");
  }
  if (g_ds_profile.enabled) g_ds_profile.price += ds_clock() - _t_pr;
  if (g_ds_profile.enabled) {
    g_ds_profile.sum_rowep_nnz +=
        static_cast<double>(leaving.row_ep.index.size());
    g_ds_profile.sum_pivotrow_nnz +=
        static_cast<double>(pivot_row.index.size());
    ++g_ds_profile.density_samples;
  }
  PivotTransaction transaction;
  const double _t_en = g_ds_profile.enabled ? ds_clock() : 0.0;
  const bool _en_ok = detail::choose_entering_bfrt(
      state, leaving, pivot_row, transaction, failure);
  if (g_ds_profile.enabled) g_ds_profile.entering += ds_clock() - _t_en;
  if (!_en_ok) {
    return numerical_trouble(std::move(failure));
  }
  statistics.taboo_rejections += transaction.taboo_rejections;
  statistics.harris_second_pass_candidates +=
      transaction.harris_second_pass_candidates;
  statistics.unstable_pivot_rejections +=
      transaction.unstable_pivot_rejections;
  if (transaction.all_harris_candidates_taboo) {
    detail::add_taboo_row(
        state, state.basis[static_cast<std::size_t>(leaving.row)], statistics);
    MinorOutcome outcome;
    outcome.kind = MinorKind::CycleBlocked;
    return outcome;
  }
  const Entering& entering = transaction.entering;
  if (entering.col < 0) {
    if (transaction.stability_blocked) {
      ++statistics.stability_blocked_rows;
      detail::add_taboo_row(
          state, state.basis[static_cast<std::size_t>(leaving.row)], statistics);
      MinorOutcome outcome;
      outcome.kind = MinorKind::CycleBlocked;
      return outcome;
    }
    MinorOutcome outcome;
    outcome.kind = MinorKind::PossiblyPrimalInfeasible;
    outcome.leaving = std::move(leaving);
    return outcome;
  }

    detail::IndexedVector column;
    column.dimension = state.m;
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A,
                                                        entering.col);
         it; ++it) {
      column.index.push_back(it.row());
      column.value.push_back(it.value());
    }
    const double _t_ft = g_ds_profile.enabled ? ds_clock() : 0.0;
    const detail::IndexedSolveEvidence direction_solve =
        state.factor->indexed_ftran(column, true);
    if (g_ds_profile.enabled) g_ds_profile.ftran += ds_clock() - _t_ft;
    const detail::IndexedVector& direction = direction_solve.solution;
    if (!direction_solve.accepted) {
      return numerical_trouble("packed pivotal-column FTRAN failed");
    }
    const double column_pivot = direction.at(leaving.row);
    if (!std::isfinite(column_pivot) || column_pivot == 0.0) {
      return numerical_trouble("packed pivotal-column FTRAN has no pivot");
    }

    const bool has_flips = !transaction.flips.empty();
    const bool has_shifts = !transaction.cost_shifts.empty();
    auto is_flipped = [&](int col) {
      return std::any_of(transaction.flips.begin(), transaction.flips.end(),
                         [col](const BoundFlip& flip) {
                           return flip.col == col;
                         });
    };
    for (std::size_t k = 0; k < transaction.flips.size(); ++k) {
      const BoundFlip& flip = transaction.flips[k];
      if (flip.col < 0 || flip.col >= state.n ||
          state.basic[static_cast<std::size_t>(flip.col)] ||
          flip.col == entering.col ||
          state.move[static_cast<std::size_t>(flip.col)] != flip.old_move ||
          !(flip.range > 0.0) || !std::isfinite(flip.range) ||
          std::any_of(transaction.flips.begin(),
                      transaction.flips.begin() +
                          static_cast<std::ptrdiff_t>(k),
                      [&](const BoundFlip& prior) {
                        return prior.col == flip.col;
                      })) {
        return numerical_trouble(
            "BFRT transaction contains an invalid bound flip");
      }
    }
    for (std::size_t k = 0; k < transaction.cost_shifts.size(); ++k) {
      const detail::WorkingCostShift& shift = transaction.cost_shifts[k];
      if (shift.col < 0 || shift.col >= state.n ||
          state.basic[static_cast<std::size_t>(shift.col)] ||
          shift.col == entering.col || is_flipped(shift.col) ||
          !std::isfinite(shift.delta) ||
          std::any_of(transaction.cost_shifts.begin(),
                      transaction.cost_shifts.begin() +
                          static_cast<std::ptrdiff_t>(k),
                      [&](const detail::WorkingCostShift& prior) {
                        return prior.col == shift.col;
                      })) {
        return numerical_trouble(
            "BFRT transaction contains an invalid working-cost shift");
      }
    }
    auto cost_shift_at = [&](int col) {
      for (const detail::WorkingCostShift& shift : transaction.cost_shifts) {
        if (shift.col == col) return shift.delta;
      }
      return 0.0;
    };
    const int leaving_col =
        state.basis[static_cast<std::size_t>(leaving.row)];
    const double dual_step = leaving.side * entering.theta;
    std::vector<int> reduced_cost_support = pivot_row.index;
    reduced_cost_support.reserve(pivot_row.index.size() +
                                 transaction.cost_shifts.size());
    for (const detail::WorkingCostShift& shift : transaction.cost_shifts) {
      reduced_cost_support.push_back(shift.col);
    }
    std::sort(reduced_cost_support.begin(), reduced_cost_support.end());
    reduced_cost_support.erase(
        std::unique(reduced_cost_support.begin(), reduced_cost_support.end()),
        reduced_cost_support.end());
    const double _t_pc = g_ds_profile.enabled ? ds_clock() : 0.0;
    for (int j : reduced_cost_support) {
      if ((state.basic[static_cast<std::size_t>(j)] && j != leaving_col) ||
          j == entering.col) {
        continue;
      }
      int move =
          j == leaving_col
              ? (state.bounds.enterable[static_cast<std::size_t>(j)]
                     ? (leaving.side < 0 ? detail::sign(Move::Up)
                                         : detail::sign(Move::Down))
                     : detail::sign(Move::Fixed))
              : detail::sign(state.move[static_cast<std::size_t>(j)]);
      if (j != leaving_col && is_flipped(j)) move = -move;
      if (move == 0) continue;
      const double updated_reduced_cost =
          state.reduced_costs[j] + dual_step * pivot_row.at(j) +
          cost_shift_at(j);
      if (!std::isfinite(updated_reduced_cost)) {
        return numerical_trouble(
            "analytical BFRT postcondition produced a non-finite reduced "
            "cost at column " +
            std::to_string(j));
      }
      if (move * updated_reduced_cost > state.options->optimality_tol) {
        return numerical_trouble(
            "analytical BFRT postcondition violates dual feasibility at "
            "column " +
            std::to_string(j));
      }
    }
    if (g_ds_profile.enabled) g_ds_profile.postcond += ds_clock() - _t_pc;

    detail::IndexedVector bfrt_delta;
    bfrt_delta.dimension = state.m;
    if (has_flips) {
      const detail::IndexedSolveEvidence bfrt_solve =
          state.factor->indexed_ftran(transaction.bfrt_rhs);
      if (!bfrt_solve.accepted) {
        return numerical_trouble("packed BFRT RHS FTRAN failed");
      }
      bfrt_delta = bfrt_solve.solution;
    }

    const double leaving_bound =
        leaving.side < 0 ? state.bounds.lower[leaving_col]
                         : state.bounds.upper[leaving_col];
    const double bfrt_delta_at_row = bfrt_delta.at(leaving.row);
    double remaining_delta =
        state.x_basic[leaving.row] - bfrt_delta_at_row - leaving_bound;
    if (transaction.covered_violation == leaving.violation) {
      remaining_delta = 0.0;
    }
    // The projected coverage (row_ep dot) and the FTRAN image agree only to
    // rounding, so a flip set that near-exactly covers the violation leaves a
    // remaining step whose sign is noise. Treat wrong-signed noise inside the
    // rounding envelope as the degenerate zero step; a disagreement beyond it
    // is still a real failure.
    const double remaining_slack =
        1024.0 * std::numeric_limits<double>::epsilon() *
        std::max({1.0, std::abs(state.x_basic[leaving.row]),
                  std::abs(bfrt_delta_at_row), std::abs(leaving_bound)});
    if (!std::isfinite(remaining_delta) ||
        leaving.side * remaining_delta < -remaining_slack) {
      return numerical_trouble(
          "packed BFRT FTRAN disagrees with the leaving-row change");
    }
    if (leaving.side * remaining_delta < 0.0) remaining_delta = 0.0;

    std::vector<detail::EdgeWeightChange>& edge_weight_changes =
        g_ds_scratch.edge_weight_changes;
    bool restart_devex = false;
    const double _t_dse = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool _dse_ok = detail::compute_dse_weights(
        state, leaving, pivot_row, direction, column_pivot, edge_weight_changes,
        restart_devex, failure);
    if (g_ds_profile.enabled) g_ds_profile.dse += ds_clock() - _t_dse;
    if (!_dse_ok) {
    return numerical_trouble(std::move(failure));
    }

    // The pivot's move/basis exchange is applied to `state` in place (the old
    // copy-modify-commit built and discarded an O(n) move vector and an O(m)
    // basis vector on every pivot).  The guard restores the pre-pivot values
    // exactly on every failure return below; `state.basic` and the float
    // vectors are still only touched at the commit point.
    const Move old_leaving_move =
        state.move[static_cast<std::size_t>(leaving_col)];
    const Move entering_old_move =
        state.move[static_cast<std::size_t>(entering.col)];
    PivotStateGuard pivot_guard(state, transaction, leaving.row, leaving_col,
                                old_leaving_move, entering.col,
                                entering_old_move);
    for (const BoundFlip& flip : transaction.flips) {
      state.move[static_cast<std::size_t>(flip.col)] =
          flip.old_move == Move::Up ? Move::Down : Move::Up;
    }
    const double entering_bound =
        entering_old_move == Move::Up ? state.bounds.lower[entering.col]
                                      : state.bounds.upper[entering.col];
    const double primal_step = remaining_delta / column_pivot;
    std::vector<std::pair<int, double>>& primal_changes =
        g_ds_scratch.primal_changes;
    primal_changes.clear();
    primal_changes.reserve(bfrt_delta.index.size() + direction.index.size() +
                           1);
    for (std::size_t k = 0; k < bfrt_delta.index.size(); ++k) {
      primal_changes.emplace_back(bfrt_delta.index[k], -bfrt_delta.value[k]);
    }
    for (std::size_t k = 0; k < direction.index.size(); ++k) {
      primal_changes.emplace_back(direction.index[k],
                                  -direction.value[k] * primal_step);
    }
    std::sort(primal_changes.begin(), primal_changes.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.first < rhs.first;
              });
    std::size_t output = 0;
    for (std::size_t begin = 0; begin < primal_changes.size();) {
      const int row = primal_changes[begin].first;
      double delta = 0.0;
      std::size_t end = begin;
      while (end < primal_changes.size() &&
             primal_changes[end].first == row) {
        delta += primal_changes[end].second;
        ++end;
      }
      if (row < 0 || row >= state.m) {
        return numerical_trouble(
            "packed primal transaction contains an invalid row");
      }
      if (row != leaving.row && delta != 0.0) {
        const double value = state.x_basic[row] + delta;
        if (!std::isfinite(value)) {
          return numerical_trouble(
              "packed primal transaction is non-finite");
        }
        primal_changes[output++] = {row, value};
      }
      begin = end;
    }
    primal_changes.resize(output);
    const double leaving_value = entering_bound + primal_step;
    if (!std::isfinite(leaving_value)) {
      return numerical_trouble("packed primal transaction is non-finite");
    }
    primal_changes.emplace_back(leaving.row, leaving_value);

    state.move[static_cast<std::size_t>(leaving_col)] =
        state.bounds.enterable[static_cast<std::size_t>(leaving_col)]
            ? (leaving.side < 0 ? Move::Up : Move::Down)
            : Move::Fixed;
    state.move[static_cast<std::size_t>(entering.col)] = Move::Fixed;
    state.basis[static_cast<std::size_t>(leaving.row)] = entering.col;

    const double _t_lu = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool _lu_ok =
        state.factor->update_indexed(leaving.row, entering.col, failure);
    if (g_ds_profile.enabled) g_ds_profile.lu_update += ds_clock() - _t_lu;
    if (!_lu_ok) {
      return numerical_trouble(std::move(failure));
    }
    pivot_guard.dismiss();
    const double _t_cy = g_ds_profile.enabled ? ds_clock() : 0.0;
    detail::record_cycle_departure(state, leaving_col, entering.col);
    // O(changes) incremental cycle-signature maintenance: the pivot swaps one
    // basis slot, moves the entering column out of (and the leaving column
    // into) the nonbasic token set, and flips each BFRT column's side token.
    detail::cycle_signature_apply_basis_swap(state, leaving.row, leaving_col,
                                             entering.col);
    detail::cycle_signature_apply_move_toggle(
        state, entering.col, detail::sign(entering_old_move));
    detail::cycle_signature_apply_move_toggle(
        state, leaving_col,
        detail::sign(state.move[static_cast<std::size_t>(leaving_col)]));
    for (const BoundFlip& flip : transaction.flips) {
      const int old_sign = detail::sign(flip.old_move);
      detail::cycle_signature_apply_move_toggle(state, flip.col, old_sign);
      detail::cycle_signature_apply_move_toggle(state, flip.col, -old_sign);
    }
    if (g_ds_profile.enabled) g_ds_profile.cycle += ds_clock() - _t_cy;
    for (const detail::EdgeWeightChange& change : edge_weight_changes) {
      state.edge_weight[static_cast<std::size_t>(change.row)] = change.value;
    }
    state.basic[static_cast<std::size_t>(leaving_col)] = 0;
    state.basic[static_cast<std::size_t>(entering.col)] = 1;
    static thread_local std::vector<int> changed_primal_rows;
    changed_primal_rows.clear();
    changed_primal_rows.reserve(primal_changes.size());
    for (const auto& [row, value] : primal_changes) {
      state.x_basic[row] = value;
      changed_primal_rows.push_back(row);
    }
    detail::refresh_leaving_heap(state, &changed_primal_rows, leaving.row);
    const double _t_rc = g_ds_profile.enabled ? ds_clock() : 0.0;
    for (int j : reduced_cost_support) {
      if (j != leaving_col && j != entering.col &&
          state.basic[static_cast<std::size_t>(j)]) {
        state.reduced_costs[j] = 0.0;
      } else {
        state.reduced_costs[j] +=
            dual_step * pivot_row.at(j) + cost_shift_at(j);
      }
    }
    state.reduced_costs[entering.col] = 0.0;
    if (g_ds_profile.enabled) g_ds_profile.rc_update += ds_clock() - _t_rc;
    if (has_shifts) {
      state.costs_shifted = true;
      for (const detail::WorkingCostShift& shift : transaction.cost_shifts) {
        state.cost[shift.col] += shift.delta;
        state.cost_shift[shift.col] += shift.delta;
        ++statistics.cost_shifts;
        statistics.max_cost_shift =
            std::max(statistics.max_cost_shift, std::abs(shift.delta));
      }
    }
    const double _t_ca = g_ds_profile.enabled ? ds_clock() : 0.0;
    detail::record_cycle_arrival(state, statistics);
    if (g_ds_profile.enabled) g_ds_profile.cycle += ds_clock() - _t_ca;
    if (state.edge_weight_mode == detail::EdgeWeightMode::Devex) {
      ++state.devex_iterations;
      if (restart_devex) {
        ++statistics.devex_restarts;
        detail::initialize_devex_framework(state, statistics);
      }
    }
    statistics.bound_flips +=
        static_cast<int>(transaction.flips.size());
    if (std::abs(entering.theta) < 1e-9) ++statistics.degenerate_dual_steps;
    if (std::abs(primal_step) < 1e-9) ++statistics.degenerate_primal_steps;
    ++statistics.iterations;
    if (g_ds_profile.enabled) ++g_ds_profile.pivots;
    ++state.updates_since_rebuild;
    state.fresh_rebuild = false;
    state.reinvert_after_pivot = state.factor->needs_rebuild();
    MinorOutcome outcome;
    outcome.kind = MinorKind::Pivoted;
    return outcome;
}

// At a time / iteration limit the running objective is a dual bound only for
// the WORKING cost (stabilization perturbations from initialize_stabilized_cost
// plus any accumulated shifts), so publishing it as a bound on the original LP
// would be unsound.  Recover a rigorous original-cost bound instead: drop every
// stabilization delta, reconstruct the state from the current basis with
// checked solves, re-classify the nonbasic bound sides against the original
// reduced costs, and audit dual feasibility — the same certificate an Optimal
// result rests on.  On success state.objective is the exact original-cost dual
// objective of a dual-feasible point, which by weak duality bounds the LP
// optimum even though the solve is unfinished.  On failure the state may hold
// a partially restored iterate; that is acceptable because interrupted results
// only export statistics (see the non-Optimal early-return in the caller).
bool certify_interrupted_dual_bound(State& state) {
  if (state.phase != Phase::Two) return false;
  std::string failure;
  detail::restore_original_cost(state);
  if (!detail::reconstruct(state, failure)) return false;
  if (!detail::normalize_nonbasic_moves(state, failure)) return false;
  const Audit certification = detail::audit(state, false, true, false);
  return certification.ok && std::isfinite(state.objective);
}

Result run_phase(State& state, Statistics& statistics,
                 const std::chrono::steady_clock::time_point& start) {
  std::string failure;
  if (!detail::initialize_stabilized_cost(state, statistics, failure)) {
    return detail::make_result(
        state, Status::NumericalFailure,
        "stabilized-cost initialization failed: " + failure, statistics);
  }

  detail::RebuildReason rebuild_reason = detail::RebuildReason::Initial;
  std::string first_fresh_numerical_failure;
  for (;;) {
    const bool reinvert = rebuild_reason != detail::RebuildReason::Initial;
    const double _t_rb = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool _rb_ok = detail::major_rebuild(state, rebuild_reason, reinvert,
                                              statistics, failure);
    if (g_ds_profile.enabled) {
      g_ds_profile.rebuild += ds_clock() - _t_rb;
      ++g_ds_profile.rebuilds;
    }
    if (!_rb_ok) {
      return detail::make_result(state, Status::NumericalFailure,
                                 std::move(failure), statistics);
    }

    for (;;) {
      if (statistics.iterations >= state.options->max_iter) {
        std::ostringstream message;
        message << "dual simplex iteration limit"
                << " (iterations=" << statistics.iterations
                << ", major_rebuilds=" << statistics.major_rebuilds
                << ", reinversions=" << statistics.reinversions
                << ", cycles=" << statistics.cycles_detected
                << ", taboo_changes=" << statistics.taboo_changes
                << ", taboo_rejections=" << statistics.taboo_rejections
                << ", taboo_rows=" << statistics.taboo_rows
                << ", taboo_row_rejections="
                << statistics.taboo_row_rejections
                << ", taboo_row_releases=" << statistics.taboo_row_releases
                << ", devex_frameworks=" << statistics.devex_frameworks
                << ", devex_restarts=" << statistics.devex_restarts
                << ", bound_flips=" << statistics.bound_flips
                << ", unstable_pivot_rejections="
                << statistics.unstable_pivot_rejections
                << ", stability_blocked_rows="
                << statistics.stability_blocked_rows
                << ", pivot_identity_refinements="
                << statistics.pivot_identity_refinements
                << ", canonical_primal_corrections="
                << state.canonical_primal_corrections
                << ", canonical_pivot_reinversions="
                << statistics.canonical_pivot_reinversions << ')';
        const bool bound_certified = certify_interrupted_dual_bound(state);
        Result result = detail::make_result(state, Status::IterationLimit,
                                            message.str(), statistics);
        result.dual_bound_certified = bound_certified;
        return result;
      }
      if (wall_time_hit(*state.options, start)) {
        std::ostringstream message;
        message << "dual simplex time limit"
                << " (iterations=" << statistics.iterations
                << ", pricing_epochs=" << state.pricing_epoch
                << ", major_rebuilds=" << statistics.major_rebuilds
                << ", reinversions=" << statistics.reinversions
                << ", cycles=" << statistics.cycles_detected
                << ", taboo_rejections=" << statistics.taboo_rejections
                << ", taboo_rows=" << statistics.taboo_rows
                << ", taboo_row_releases=" << statistics.taboo_row_releases
                << ", unstable_pivot_rejections="
                << statistics.unstable_pivot_rejections
                << ", stability_blocked_rows="
                << statistics.stability_blocked_rows
                << ", pivot_identity_refinements="
                << statistics.pivot_identity_refinements << ')';
        const bool bound_certified = certify_interrupted_dual_bound(state);
        Result result = detail::make_result(state, Status::TimeLimit,
                                            message.str(), statistics);
        result.dual_bound_certified = bound_certified;
        return result;
      }
      if (state.options->incumbent_bound != nullptr &&
          statistics.iterations %
                  std::max(1, state.options->incumbent_check_interval) ==
              0) {
        const double incumbent = state.options->incumbent_bound->load(
            std::memory_order_relaxed);
        const double objective = state.sf->objective_const - state.objective;
        if (std::isfinite(incumbent) &&
            objective >= incumbent - state.options->optimality_tol) {
          return detail::make_result(state, Status::ObjectiveCutoff,
                                     "objective cutoff", statistics);
        }
      }

      const double _t_minor = g_ds_profile.enabled ? ds_clock() : 0.0;
      MinorOutcome outcome = minor_iteration(state, statistics);
      if (g_ds_profile.enabled) g_ds_profile.minor_total += ds_clock() - _t_minor;
      if (outcome.kind == MinorKind::CycleBlocked) continue;
      if (outcome.kind == MinorKind::Pivoted) {
        first_fresh_numerical_failure.clear();
        if (state.reinvert_after_pivot) {
          state.reinvert_after_pivot = false;
          rebuild_reason = detail::RebuildReason::NumericalTrouble;
          break;
        }
        const int update_limit =
            std::max(50, std::min(200, std::max(1, state.m / 4)));
        if (state.factor->needs_rebuild()) {
          rebuild_reason = detail::RebuildReason::SyntheticWork;
          break;
        }
        if (state.updates_since_rebuild >= update_limit) {
          rebuild_reason = detail::RebuildReason::UpdateLimit;
          break;
        }
        continue;
      }

      if (outcome.kind == MinorKind::NumericalTrouble) {
        if (!state.fresh_rebuild) {
          first_fresh_numerical_failure = outcome.message;
          rebuild_reason = detail::RebuildReason::NumericalTrouble;
          break;
        }
        if (first_fresh_numerical_failure.empty()) {
          first_fresh_numerical_failure = outcome.message;
          rebuild_reason = detail::RebuildReason::NumericalTrouble;
          break;
        }
        return detail::make_result(
            state, Status::NumericalFailure,
            "fresh rebuild confirmed numerical failure: " + outcome.message,
            statistics);
      }

      if (outcome.kind == MinorKind::PossiblyPrimalInfeasible) {
        if (!state.fresh_rebuild) {
          rebuild_reason = detail::RebuildReason::PossiblyPrimalInfeasible;
          break;
        }
        const detail::Certificate certificate =
            detail::primal_infeasibility_certificate(state, outcome.leaving);
        if (!certificate.valid) {
          return detail::make_result(
              state, Status::NumericalFailure,
              "fresh CHUZC has no entering column and no checked original-bound "
              "interval certificate",
              statistics);
        }
        Result result = detail::make_result(
            state, Status::PrimalInfeasible,
            "fresh rebuilt pivotal-row interval proves primal infeasibility",
            statistics);
        attach_farkas_certificate(certificate, result);
        return result;
      }

      if (!state.fresh_rebuild) {
        rebuild_reason = detail::RebuildReason::PossiblyOptimal;
        break;
      }

      if (!detail::working_cost_is_original(state)) {
        detail::restore_original_cost(state);
        ++statistics.cleanup_passes;
        if (!detail::reconstruct(state, failure)) {
          return detail::make_result(
              state, Status::NumericalFailure,
              "cleanup reconstruction failed: " + failure, statistics);
        }
        const Audit cleanup_audit = detail::audit(
            state, true, true, state.phase == Phase::Two);
        update_statistics(cleanup_audit, statistics);
        if (!cleanup_audit.ok) {
          const Audit primal_audit = detail::audit(
              state, true, false, state.phase == Phase::Two);
          if (!primal_audit.ok) {
            return detail::make_result(
                state, Status::NumericalFailure,
                "cleanup lost primal feasibility: " + primal_audit.failure,
                statistics);
          }
          Result cleanup = detail::run_primal_phase(state, statistics, start);
          if (cleanup.status == Status::Optimal) {
            cleanup.message = "optimal after unperturbed primal cleanup";
          } else {
            cleanup.message = "unperturbed primal cleanup: " + cleanup.message;
          }
          return cleanup;
        }
      }

      const Audit final = detail::audit(
          state, true, true, state.phase == Phase::Two);
      update_statistics(final, statistics);
      if (!final.ok) {
        return detail::make_result(
            state, Status::NumericalFailure,
            "fresh terminal invariant failed: " + final.failure, statistics);
      }
      return detail::make_result(state, Status::Optimal, "optimal",
                                 statistics);
    }
  }
}

Result solve_impl(const StandardFormLP& sf, const SimplexOptions& options,
                  const SimplexBasis* basis_hint, bool allow_phase_one) {
  Statistics statistics;
  State state;
  std::string failure;
  const auto start = std::chrono::steady_clock::now();
  g_ds_profile.reset();
  if (!detail::initialize(state, sf, options, Phase::Two, basis_hint,
                          statistics, failure)) {
    return empty_result(Status::InvalidBasis, std::move(failure), statistics);
  }
  g_ds_profile.model_m = state.m;
  g_ds_profile.model_n = state.n;

  auto run_primal_phase_one_from_current_basis = [&]() -> Result {
    state.phase = Phase::One;
    state.bounds = detail::make_primal_phase_one_bounds(sf);
    state.cost = detail::make_primal_phase_one_cost(sf);
    state.move.assign(static_cast<std::size_t>(state.n), Move::Up);
    if (!detail::reconstruct(state, failure)) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "primal Phase-I initialization failed: " + failure, statistics);
    }
    Result phase_one = detail::run_primal_phase(state, statistics, start);
    if (phase_one.status != Status::Optimal) {
      phase_one.message = "primal Phase I: " + phase_one.message;
      return phase_one;
    }
    const Eigen::VectorXd phase_one_x = detail::full_primal(state);
    double artificial_sum = 0.0;
    for (int col : sf.row_to_artificial_col) {
      if (col >= 0 && col < state.n) artificial_sum += phase_one_x[col];
    }
    if (artificial_sum > options.feasibility_tol) {
      const detail::Certificate certificate =
          detail::phase_one_farkas_certificate(state);
      if (!certificate.valid) {
        return detail::make_result(
            state, Status::NumericalFailure,
            "positive artificial Phase-I optimum has no checked separating "
            "row certificate",
            statistics);
      }
      Result result = detail::make_result(
          state, Status::PrimalInfeasible,
          "checked artificial-objective Phase-I optimum proves primal "
          "infeasibility",
          statistics);
      attach_farkas_certificate(certificate, result);
      return result;
    }

    state.phase = Phase::Two;
    state.bounds = detail::make_phase_two_bounds(sf);
    state.cost = sf.c_max;
    for (int col : sf.row_to_artificial_col) {
      if (col >= 0 && col < state.n &&
          !state.basic[static_cast<std::size_t>(col)]) {
        state.move[static_cast<std::size_t>(col)] = Move::Fixed;
      }
    }
    if (!detail::reconstruct(state, failure)) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "primal Phase-I transition failed: " + failure, statistics);
    }
    return detail::run_primal_phase(state, statistics, start);
  };

  // A cold logical basis is initialized by the standard artificial-objective
  // primal Phase I. This is the direct simplex strategy for a basis that has
  // no inherited dual-feasibility contract; it is not a retry or fallback.
  if (basis_hint == nullptr) {
    const int crash_replacements =
        detail::apply_certified_singleton_crash(sf, state.basis);
    if (crash_replacements > 0) {
      const double _t_ew = g_ds_profile.enabled ? ds_clock() : 0.0;
      const bool _ew_ok =
          state.factor->rebuild(state.basis, statistics.rank_repairs, failure) &&
          detail::reconstruct(state, failure) &&
          detail::initialize_exact_edge_weights(state, statistics, failure);
      if (g_ds_profile.enabled) g_ds_profile.edge_init += ds_clock() - _t_ew;
      if (!_ew_ok) {
        return detail::make_result(
            state, Status::NumericalFailure,
            "certified singleton crash reconstruction failed: " + failure,
            statistics);
      }
      ++statistics.reinversions;
    }

    // For this fixed cold basis, endpoint selection is an exact feasibility
    // classification, not a solve attempt: every boxed nonbasic can choose
    // the reduced-cost-compatible side, while a lower-only column with
    // positive reduced cost proves that no such side assignment exists.
    // Enter revised dual directly when the necessary-and-sufficient test
    // succeeds. A later dual failure is terminal and never falls through to
    // primal Phase I.
    std::string dual_start_failure;
    if (detail::normalize_nonbasic_moves(state, dual_start_failure)) {
      const Audit initial = detail::audit(state, false, true, false);
      update_statistics(initial, statistics);
      if (!initial.ok) {
        return detail::make_result(
            state, Status::NumericalFailure,
            "cold dual-start invariant failed: " + initial.failure,
            statistics);
      }
      return run_phase(state, statistics, start);
    }

    return run_primal_phase_one_from_current_basis();
  }

  if (detail::normalize_nonbasic_moves(state, failure)) {
    const Audit initial = detail::audit(state, false, true, false);
    update_statistics(initial, statistics);
    if (!initial.ok) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "initial Phase-II invariant failed: " + initial.failure,
          statistics);
    }
    return run_phase(state, statistics, start);
  }
  if (!allow_phase_one) {
    return detail::make_result(state, Status::DualInfeasibleStart,
                               std::move(failure), statistics);
  }

  if (options.allow_warm_primal_phase_one) {
    return run_primal_phase_one_from_current_basis();
  }

  return detail::make_result(
      state, Status::DualInfeasibleStart,
      "warm basis is neither directly dual-feasible nor authorized for a "
      "cold artificial restart: " + failure,
      statistics);
}

}  // namespace

const char* status_name(Status status) {
  switch (status) {
    case Status::Optimal:
      return "Optimal";
    case Status::PrimalInfeasible:
      return "Primal infeasible";
    case Status::DualInfeasibleStart:
      return "Dual-infeasible start";
    case Status::IterationLimit:
      return "Iteration limit";
    case Status::TimeLimit:
      return "Time limit";
    case Status::ObjectiveCutoff:
      return "Objective cutoff";
    case Status::NumericalFailure:
      return "Numerical failure";
    case Status::InvalidBasis:
      return "Invalid basis";
  }
  return "Unknown";
}

Result solve_phase2(const StandardFormLP& sf, const SimplexOptions& options,
                    const SimplexBasis& basis_hint) {
  Result result = solve_impl(sf, options, &basis_hint, false);
  g_ds_profile.report(g_ds_profile.model_m, g_ds_profile.model_n);
  return result;
}

Result solve(const StandardFormLP& sf, const SimplexOptions& options,
             const SimplexBasis* basis_hint) {
  Result result = solve_impl(sf, options, basis_hint, true);
  g_ds_profile.report(g_ds_profile.model_m, g_ds_profile.model_n);
  if (std::getenv("MIPSOLVERS_DS_VERBOSE") != nullptr) {
    const Statistics& s = result.statistics;
    std::fprintf(
        stderr,
        "DS %s: m=%d n=%d iters=%d degen_dual=%d degen_primal=%d "
        "bound_flips=%d cost_shifts=%d devex_frameworks=%d devex_restarts=%d "
        "cycles=%d taboo_rej=%d taboo_row_rej=%d stab_blocked=%d "
        "major_rebuilds=%d reinversions=%d message='%s'\n",
        status_name(result.status), static_cast<int>(result.basis.size()),
        static_cast<int>(result.reduced_costs.size()), s.iterations,
        s.degenerate_dual_steps, s.degenerate_primal_steps, s.bound_flips,
        s.cost_shifts, s.devex_frameworks, s.devex_restarts, s.cycles_detected,
        s.taboo_rejections, s.taboo_row_rejections, s.stability_blocked_rows,
        s.major_rebuilds, s.reinversions, result.message.c_str());
  }
  return result;
}

Result solve(const StandardFormLP& sf, const SimplexOptions& options,
             const SimplexBasis& basis_hint) {
  return solve(sf, options, &basis_hint);
}

}  // namespace mipsolvers::engine::native_dual
