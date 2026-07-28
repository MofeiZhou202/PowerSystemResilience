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

// Effective audit tier for the per-pivot hot loop.  SimplexOptions::paranoid,
// overridable by MIPSOLVERS_DS_PARANOID (0 = force tiered, 1 = force paranoid).
// Tiered mode skips the two long-double Tier-2 identity audits on the fast path
// (they still run when a cheap Tier-0 guard trips); every other safety check
// — checked FTRAN/BTRAN backward-error validation, ratio-test stability, BFRT
// analytical postcondition, and the final original-space audit — stays active.
bool resolve_paranoid(const State& state) {
  static const int env = [] {
    const char* e = std::getenv("MIPSOLVERS_DS_PARANOID");
    return e ? (e[0] == '0' ? 0 : 1) : -1;
  }();
  if (env >= 0) return env != 0;
  return state.options == nullptr ? true : state.options->paranoid;
}

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

MinorOutcome minor_iteration(State& state, Statistics& statistics) {
  ++state.pricing_epoch;
  const bool paranoid = resolve_paranoid(state);
  Leaving leaving;
  std::string failure;
  if (!detail::choose_leaving(state, leaving, failure)) {
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

  const Eigen::VectorXd pivot_row =
      detail::multiply_AT(state.sf->A, leaving.row_ep);
  PivotTransaction transaction;
  if (!detail::choose_entering_bfrt(state, leaving, pivot_row, transaction,
                                    failure)) {
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

    Eigen::VectorXd column = Eigen::VectorXd::Zero(state.m);
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A,
                                                        entering.col);
         it; ++it) {
      column[it.row()] = it.value();
    }
    detail::SolveEvidence direction_solve =
        state.factor->checked_ftran(column, true);
    Eigen::VectorXd direction = direction_solve.solution;
    if (!direction_solve.accepted) {
    return numerical_trouble(
        "pivotal-column FTRAN failed backward-error validation (" +
        state.factor->last_solve_diagnostics() + ")");
    }
    if (direction_solve.refined) ++statistics.iterative_refinements;
    // Tier 0 (every pivot): the checked FTRAN above already passed backward-error
    // validation and the ratio test required a stable |alpha_r|, so only guard a
    // zero / non-finite pivot cheaply here.  Escalate to the full long-double
    // pivot-identity audit (Tier 2) under paranoid mode or when the guard trips.
    const double tier0_pivot = direction[leaving.row];
    const bool need_pivot_evidence =
        paranoid || !std::isfinite(tier0_pivot) || tier0_pivot == 0.0;
    if (need_pivot_evidence) {
      detail::PivotEvidence pivot_evidence = state.factor->pivot_evidence(
          leaving.row, entering.col, direction, leaving.row_ep);
      if (!pivot_evidence.accepted && !direction_solve.refined) {
        direction_solve = state.factor->refine_ftran(column, direction);
        if (direction_solve.accepted) {
          direction = direction_solve.solution;
          ++statistics.iterative_refinements;
          ++statistics.pivot_identity_refinements;
          pivot_evidence = state.factor->pivot_evidence(
              leaving.row, entering.col, direction, leaving.row_ep);
        }
      }
      if (!pivot_evidence.accepted) {
        std::ostringstream message;
        message << std::setprecision(18)
                << "row-wise and column-wise pivot residual identity failed"
                << " (row_pivot=" << pivot_evidence.row_pivot
                << ", column_pivot=" << pivot_evidence.column_pivot
                << ", discrepancy=" << pivot_evidence.discrepancy
                << ", residual_envelope=" << pivot_evidence.residual_envelope
                << ", arithmetic_guard=" << pivot_evidence.arithmetic_guard
                << ", column_residual_inf="
                << pivot_evidence.column_residual_inf
                << ", row_residual_inf=" << pivot_evidence.row_residual_inf
                << ')';
        return numerical_trouble(message.str());
      }
    }
    const double column_pivot = direction[leaving.row];

    std::vector<char> flipped(static_cast<std::size_t>(state.n), 0);
    for (const BoundFlip& flip : transaction.flips) {
      if (flip.col < 0 || flip.col >= state.n ||
          state.basic[static_cast<std::size_t>(flip.col)] ||
          flip.col == entering.col ||
          state.move[static_cast<std::size_t>(flip.col)] != flip.old_move ||
          !(flip.range > 0.0) || !std::isfinite(flip.range)) {
      return numerical_trouble(
          "BFRT transaction contains an invalid bound flip");
      }
      flipped[static_cast<std::size_t>(flip.col)] = 1;
    }
    Eigen::VectorXd transaction_cost_shift = Eigen::VectorXd::Zero(state.n);
    for (const detail::WorkingCostShift& shift : transaction.cost_shifts) {
      if (shift.col < 0 || shift.col >= state.n ||
          state.basic[static_cast<std::size_t>(shift.col)] ||
          shift.col == entering.col ||
          flipped[static_cast<std::size_t>(shift.col)] ||
          !std::isfinite(shift.delta) ||
          transaction_cost_shift[shift.col] != 0.0) {
        return numerical_trouble(
            "BFRT transaction contains an invalid working-cost shift");
      }
      transaction_cost_shift[shift.col] = shift.delta;
    }
    for (int j = 0; j < state.n; ++j) {
      if (state.basic[static_cast<std::size_t>(j)] || j == entering.col)
        continue;
      int move = detail::sign(state.move[static_cast<std::size_t>(j)]);
      if (flipped[static_cast<std::size_t>(j)]) move = -move;
      if (move == 0) continue;
      const double updated_reduced_cost =
          state.reduced_costs[j] +
          leaving.side * entering.theta * pivot_row[j] +
          transaction_cost_shift[j];
      if (move * updated_reduced_cost > state.options->optimality_tol) {
      return numerical_trouble(
          "analytical BFRT postcondition violates dual feasibility at column " +
          std::to_string(j));
      }
    }

    Eigen::VectorXd flipped_basic = state.x_basic;
    Eigen::VectorXd bfrt_delta = Eigen::VectorXd::Zero(state.m);
    if (!transaction.flips.empty()) {
      bfrt_delta = state.factor->ftran(transaction.bfrt_rhs);
      if (bfrt_delta.size() != state.m || !bfrt_delta.allFinite()) {
      return numerical_trouble(
          "BFRT RHS FTRAN failed backward-error validation (" +
          state.factor->last_solve_diagnostics() + ")");
      }
      flipped_basic -= bfrt_delta;
    }

    const int leaving_col =
        state.basis[static_cast<std::size_t>(leaving.row)];
    const double leaving_bound =
        leaving.side < 0 ? state.bounds.lower[leaving_col]
                         : state.bounds.upper[leaving_col];
    double remaining_delta = flipped_basic[leaving.row] - leaving_bound;
    if (transaction.covered_violation == leaving.violation) {
      remaining_delta = 0.0;
    }
    const double predicted_delta =
        leaving.side *
        (leaving.violation - transaction.covered_violation);
    long double solve_discrepancy = 0.0L;
    long double solve_envelope = 0.0L;
    // Tier 2 (paranoid only): long-double row-solve consistency of the BFRT
    // flip FTRAN.  The cheap remaining-delta sign check below stays active in
    // both tiers, and the BFRT analytical dual-feasibility postcondition above
    // already validated the flip transaction.
    const bool solve_consistent =
        !paranoid || state.factor->row_solve_consistent(
                         leaving.row, transaction.bfrt_rhs, bfrt_delta,
                         leaving.row_ep, solve_discrepancy, solve_envelope);
    if (!solve_consistent || leaving.side * remaining_delta < 0.0) {
      long double rhs_projection = 0.0L;
      for (int row = 0; row < state.m; ++row) {
        rhs_projection +=
            static_cast<long double>(leaving.row_ep[row]) *
            static_cast<long double>(transaction.bfrt_rhs[row]);
      }
      const long double projected_coverage =
          static_cast<long double>(leaving.side) * rhs_projection;
      const long double ftran_coverage =
          static_cast<long double>(leaving.side) *
          (static_cast<long double>(state.x_basic[leaving.row]) -
           static_cast<long double>(flipped_basic[leaving.row]));
      std::ostringstream message;
      message << std::setprecision(18)
              << "BFRT FTRAN disagrees with the priced leaving-row change"
              << " (remaining=" << remaining_delta
              << ", predicted=" << predicted_delta
              << ", solve_discrepancy=" << solve_discrepancy
              << ", solve_envelope=" << solve_envelope
              << ", priced_coverage=" << transaction.covered_violation
              << ", projected_coverage=" << projected_coverage
              << ", ftran_coverage=" << ftran_coverage
              << ", flips=" << transaction.flips.size()
              << ", " << state.factor->last_solve_diagnostics() << ')';
    return numerical_trouble(message.str());
    }

    std::vector<double> updated_edge_weight;
    bool restart_devex = false;
    if (!detail::compute_dse_weights(state, leaving, pivot_row, direction,
                                     column_pivot, updated_edge_weight,
                                     restart_devex, failure)) {
    return numerical_trouble(std::move(failure));
    }

    std::vector<Move> candidate_move = state.move;
    for (const BoundFlip& flip : transaction.flips) {
      candidate_move[static_cast<std::size_t>(flip.col)] =
          flip.old_move == Move::Up ? Move::Down : Move::Up;
    }
    const Move entering_old_move =
        candidate_move[static_cast<std::size_t>(entering.col)];
    const double entering_bound =
        entering_old_move == Move::Up ? state.bounds.lower[entering.col]
                                      : state.bounds.upper[entering.col];
    const double primal_step = remaining_delta / column_pivot;
    Eigen::VectorXd predicted_post_pivot =
        flipped_basic - direction * primal_step;
    predicted_post_pivot[leaving.row] = entering_bound + primal_step;

    candidate_move[static_cast<std::size_t>(leaving_col)] =
        state.bounds.enterable[static_cast<std::size_t>(leaving_col)]
            ? (leaving.side < 0 ? Move::Up : Move::Down)
            : Move::Fixed;
    candidate_move[static_cast<std::size_t>(entering.col)] = Move::Fixed;
    std::vector<int> candidate_basis = state.basis;
    candidate_basis[static_cast<std::size_t>(leaving.row)] = entering.col;

    Eigen::VectorXd predicted_full = state.bounds.lower;
    for (int j = 0; j < state.n; ++j) {
      if (candidate_move[static_cast<std::size_t>(j)] == Move::Down)
        predicted_full[j] = state.bounds.upper[j];
    }
    for (int row = 0; row < state.m; ++row) {
      predicted_full[candidate_basis[static_cast<std::size_t>(row)]] =
          predicted_post_pivot[row];
    }
    Eigen::VectorXd candidate_reduced_costs =
        state.reduced_costs +
        leaving.side * entering.theta * pivot_row + transaction_cost_shift;
    for (int row = 0; row < state.m; ++row) {
      candidate_reduced_costs[
          candidate_basis[static_cast<std::size_t>(row)]] = 0.0;
    }
    for (int j = 0; j < state.n; ++j) {
      const bool candidate_basic =
          j == entering.col ||
          (state.basic[static_cast<std::size_t>(j)] && j != leaving_col);
      if (candidate_basic) continue;
      const int move = detail::sign(
          candidate_move[static_cast<std::size_t>(j)]);
      if (move != 0 &&
          move * candidate_reduced_costs[j] >
              state.options->optimality_tol) {
      return numerical_trouble(
          "incremental BFRT state violates dual feasibility at column " +
          std::to_string(j));
      }
    }
    const Eigen::VectorXd candidate_cost = state.cost + transaction_cost_shift;
    const Eigen::VectorXd candidate_cost_shift =
        state.cost_shift + transaction_cost_shift;
    const double candidate_objective = candidate_cost.dot(predicted_full);
    const double predicted_residual = detail::equation_residual_inf(
        state.sf->A, predicted_full, state.sf->b);
    const double equation_limit =
        state.options->feasibility_tol *
        std::max(1.0, state.sf->b.lpNorm<Eigen::Infinity>());
    if (!predicted_post_pivot.allFinite() || !predicted_full.allFinite() ||
        !candidate_reduced_costs.allFinite() || !candidate_cost.allFinite() ||
        !candidate_cost_shift.allFinite() ||
        !std::isfinite(candidate_objective) ||
        !std::isfinite(predicted_residual)) {
      return numerical_trouble(
          "BFRT incremental primal transaction is non-finite");
    }

    const bool refined_pivotal_solve =
        leaving.row_ep_refined || direction_solve.refined;
    const bool residual_requires_reinvert = predicted_residual > equation_limit;
    const bool reinvert_pivotal_basis =
        refined_pivotal_solve || residual_requires_reinvert;
    if (residual_requires_reinvert) {
      ++statistics.canonical_pivot_reinversions;
    }
    // The row/column identity and BFRT coverage above certify the algebraic
    // exchange. If its incrementally maintained primal state misses the fixed
    // canonical feasibility limit, atomically commit only the basis exchange;
    // the driver immediately INVERTs and reconstructs that new basis before
    // another minor iteration. The over-limit state is never used for pricing.
    if (!reinvert_pivotal_basis &&
        !state.factor->update(leaving.row, entering.col, direction,
                              leaving.row_ep, failure)) {
      return numerical_trouble(std::move(failure));
    }
    detail::record_cycle_departure(state, leaving_col, entering.col);
    state.move = std::move(candidate_move);
    state.basis = std::move(candidate_basis);
    state.edge_weight = std::move(updated_edge_weight);
    state.basic[static_cast<std::size_t>(leaving_col)] = 0;
    state.basic[static_cast<std::size_t>(entering.col)] = 1;
    state.x_basic = std::move(predicted_post_pivot);
    state.reduced_costs = std::move(candidate_reduced_costs);
    state.cost = candidate_cost;
    state.cost_shift = candidate_cost_shift;
    if (!transaction.cost_shifts.empty()) state.costs_shifted = true;
    for (const detail::WorkingCostShift& shift : transaction.cost_shifts) {
      ++statistics.cost_shifts;
      statistics.max_cost_shift =
          std::max(statistics.max_cost_shift, std::abs(shift.delta));
    }
    state.objective = candidate_objective;
    detail::record_cycle_arrival(state, statistics);
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
    ++state.updates_since_rebuild;
    state.fresh_rebuild = false;
    state.reinvert_after_pivot = reinvert_pivotal_basis;
    MinorOutcome outcome;
    outcome.kind = MinorKind::Pivoted;
    return outcome;
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
    if (!detail::major_rebuild(state, rebuild_reason, reinvert, statistics,
                               failure)) {
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
        return detail::make_result(state, Status::IterationLimit,
                                   message.str(), statistics);
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
        return detail::make_result(state, Status::TimeLimit,
                                   message.str(), statistics);
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

      MinorOutcome outcome = minor_iteration(state, statistics);
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
  if (!detail::initialize(state, sf, options, Phase::Two, basis_hint,
                          statistics, failure)) {
    return empty_result(Status::InvalidBasis, std::move(failure), statistics);
  }

  // A cold logical basis is initialized by the standard artificial-objective
  // primal Phase I. This is the direct simplex strategy for a basis that has
  // no inherited dual-feasibility contract; it is not a retry or fallback.
  if (basis_hint == nullptr) {
    const int crash_replacements =
        detail::apply_certified_singleton_crash(sf, state.basis);
    if (crash_replacements > 0) {
      if (!state.factor->rebuild(state.basis, statistics.rank_repairs,
                                 failure) ||
          !detail::reconstruct(state, failure) ||
          !detail::initialize_exact_edge_weights(state, statistics, failure)) {
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
  return solve_impl(sf, options, &basis_hint, false);
}

Result solve(const StandardFormLP& sf, const SimplexOptions& options,
             const SimplexBasis* basis_hint) {
  Result result = solve_impl(sf, options, basis_hint, true);
  if (std::getenv("MIPSOLVERS_DS_VERBOSE") != nullptr) {
    const Statistics& s = result.statistics;
    std::fprintf(
        stderr,
        "DS %s: m=%d n=%d iters=%d degen_dual=%d degen_primal=%d "
        "bound_flips=%d cost_shifts=%d devex_frameworks=%d devex_restarts=%d "
        "cycles=%d taboo_rej=%d taboo_row_rej=%d stab_blocked=%d "
        "major_rebuilds=%d reinversions=%d\n",
        status_name(result.status), static_cast<int>(result.basis.size()),
        static_cast<int>(result.reduced_costs.size()), s.iterations,
        s.degenerate_dual_steps, s.degenerate_primal_steps, s.bound_flips,
        s.cost_shifts, s.devex_frameworks, s.devex_restarts, s.cycles_detected,
        s.taboo_rejections, s.taboo_row_rejections, s.stability_blocked_rows,
        s.major_rebuilds, s.reinversions);
  }
  return result;
}

Result solve(const StandardFormLP& sf, const SimplexOptions& options,
             const SimplexBasis& basis_hint) {
  return solve(sf, options, &basis_hint);
}

}  // namespace mipsolvers::engine::native_dual
