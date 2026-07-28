#include "primal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace mipsolvers::engine::native_dual::detail {
namespace {

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

struct PrimalEntering {
  int col{-1};
  int move{0};
  double gain{0.0};
  Eigen::VectorXd direction;
};

struct PrimalLeaving {
  int row{-1};
  int side{0};
  double step{std::numeric_limits<double>::infinity()};
};

bool rebuild_primal_state(State& state, Statistics& statistics,
                          std::string& failure) {
  const Eigen::VectorXd previous_x = state.x_basic;
  const Eigen::VectorXd previous_reduced_costs = state.reduced_costs;
  const double previous_objective = state.objective;
  const int previous_updates = state.updates_since_rebuild;
  if (!state.factor->rebuild(state.basis, statistics.rank_repairs, failure)) {
    failure = "primal major INVERT failed: " + failure;
    return false;
  }
  ++statistics.reinversions;
  if (!reconstruct(state, failure)) {
    failure = "primal major reconstruction failed: " + failure;
    return false;
  }
  if (previous_x.size() == state.x_basic.size() && previous_x.allFinite()) {
    statistics.max_primal_drift =
        std::max(statistics.max_primal_drift,
                 (previous_x - state.x_basic).lpNorm<Eigen::Infinity>());
  }
  if (previous_reduced_costs.size() == state.reduced_costs.size() &&
      previous_reduced_costs.allFinite()) {
    statistics.max_dual_drift =
        std::max(statistics.max_dual_drift,
                 (previous_reduced_costs - state.reduced_costs)
                     .lpNorm<Eigen::Infinity>());
  }
  if (std::isfinite(previous_objective)) {
    statistics.max_objective_drift =
        std::max(statistics.max_objective_drift,
                 std::abs(previous_objective - state.objective));
  }
  statistics.max_updates_between_rebuilds =
      std::max(statistics.max_updates_between_rebuilds, previous_updates);
  ++statistics.major_rebuilds;
  ++statistics.rebuild_numerical_trouble;
  state.updates_since_rebuild = 0;
  state.fresh_rebuild = true;
  return true;
}

bool choose_entering(const State& state, PrimalEntering& entering) {
  for (int col = 0; col < state.n; ++col) {
    if (state.basic[static_cast<std::size_t>(col)]) continue;
    const int move = sign(state.move[static_cast<std::size_t>(col)]);
    if (move == 0) continue;
    const double gain = move * state.reduced_costs[col];
    if (gain <= state.options->optimality_tol) continue;
    if (gain > entering.gain ||
        (gain == entering.gain &&
         (entering.col < 0 || col < entering.col))) {
      entering.col = col;
      entering.move = move;
      entering.gain = gain;
    }
  }
  return entering.col >= 0;
}

bool choose_leaving_harris(const State& state, const PrimalEntering& entering,
                           PrimalLeaving& leaving) {
  const double feasibility = state.options->feasibility_tol;
  double relaxed_step = std::numeric_limits<double>::infinity();
  const double lower_q = state.bounds.lower[entering.col];
  const double upper_q = state.bounds.upper[entering.col];
  if (std::isfinite(upper_q)) {
    relaxed_step = std::max(0.0, upper_q - lower_q) + feasibility;
  }
  for (int row = 0; row < state.m; ++row) {
    const double change = -entering.move * entering.direction[row];
    const int basic_col = state.basis[static_cast<std::size_t>(row)];
    if (change > 0.0 && std::isfinite(state.bounds.upper[basic_col])) {
      const double distance =
          state.bounds.upper[basic_col] - state.x_basic[row];
      relaxed_step =
          std::min(relaxed_step, std::max(0.0, distance + feasibility) / change);
    } else if (change < 0.0) {
      const double distance = state.x_basic[row] - state.bounds.lower[basic_col];
      relaxed_step = std::min(
          relaxed_step, std::max(0.0, distance + feasibility) / -change);
    }
  }

  double best_pivot = -1.0;
  for (int row = 0; row < state.m; ++row) {
    const double change = -entering.move * entering.direction[row];
    const int basic_col = state.basis[static_cast<std::size_t>(row)];
    double exact_step = std::numeric_limits<double>::infinity();
    int side = 0;
    if (change > 0.0 && std::isfinite(state.bounds.upper[basic_col])) {
      exact_step = std::max(
          0.0, (state.bounds.upper[basic_col] - state.x_basic[row]) / change);
      side = 1;
    } else if (change < 0.0) {
      exact_step = std::max(
          0.0, (state.x_basic[row] - state.bounds.lower[basic_col]) / -change);
      side = -1;
    }
    if (exact_step > relaxed_step) continue;
    const double pivot = std::abs(entering.direction[row]);
    if (pivot > best_pivot ||
        (pivot == best_pivot && (leaving.row < 0 || row < leaving.row))) {
      best_pivot = pivot;
      leaving.row = row;
      leaving.side = side;
      leaving.step = exact_step;
    }
  }
  return std::isfinite(relaxed_step);
}

}  // namespace

Result run_primal_phase(
    State& state, Statistics& statistics,
    const std::chrono::steady_clock::time_point& solve_start) {
  while (statistics.iterations < state.options->max_iter) {
    if (wall_time_hit(*state.options, solve_start)) {
      return make_result(state, Status::TimeLimit, "primal simplex time limit",
                         statistics);
    }
    const Audit before = audit(state, true, false, false);
    update_statistics(before, statistics);
    if (!before.ok) {
      return make_result(state, Status::NumericalFailure,
                         "pre-primal-pivot invariant failed: " + before.failure,
                         statistics);
    }

    PrimalEntering entering;
    if (!choose_entering(state, entering)) {
      const Audit final = audit(state, true, true, false);
      update_statistics(final, statistics);
      if (!final.ok) {
        return make_result(state, Status::NumericalFailure,
                           "primal termination invariant failed: " +
                               final.failure,
                           statistics);
      }
      return make_result(state, Status::Optimal, "optimal", statistics);
    }

    Eigen::VectorXd column = Eigen::VectorXd::Zero(state.m);
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A,
                                                        entering.col);
         it; ++it) {
      column[it.row()] = it.value();
    }
    const SolveEvidence direction_solve =
        state.factor->checked_ftran(column, true);
    if (!direction_solve.accepted) {
      if (state.updates_since_rebuild > 0) {
        std::string failure;
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "fresh primal pivotal-column FTRAN failed (" +
                             state.factor->last_solve_diagnostics() + ')',
                         statistics);
    }
    if (direction_solve.refined) ++statistics.iterative_refinements;
    entering.direction = direction_solve.solution;

    PrimalLeaving leaving;
    if (!choose_leaving_harris(state, entering, leaving)) {
      return make_result(
          state, Status::NumericalFailure,
          "primal ratio test has no finite limiting bound; an unbounded "
          "certificate is required",
          statistics);
    }

    const double q_range =
        state.bounds.upper[entering.col] - state.bounds.lower[entering.col];
    if (std::isfinite(q_range) &&
        (leaving.row < 0 || q_range < leaving.step)) {
      const double primal_delta = entering.move * q_range;
      state.x_basic -= entering.direction * primal_delta;
      state.objective += state.reduced_costs[entering.col] * primal_delta;
      state.move[static_cast<std::size_t>(entering.col)] =
          entering.move > 0 ? Move::Down : Move::Up;
      ++statistics.bound_flips;
      ++statistics.iterations;
      continue;
    }
    if (leaving.row < 0 ||
        entering.direction[leaving.row] == 0.0) {
      return make_result(state, Status::NumericalFailure,
                         "Harris primal ratio test selected no algebraic pivot",
                         statistics);
    }

    const int leaving_col =
        state.basis[static_cast<std::size_t>(leaving.row)];
    Eigen::VectorXd unit = Eigen::VectorXd::Zero(state.m);
    unit[leaving.row] = 1.0;
    const SolveEvidence row_solve =
        state.factor->checked_btran(unit, true);
    std::string failure;
    if (!row_solve.accepted) {
      if (state.updates_since_rebuild > 0) {
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "fresh primal basis-update BTRAN failed (" +
                             state.factor->last_solve_diagnostics() + ')',
                         statistics);
    }
    if (row_solve.refined) ++statistics.iterative_refinements;
    const Eigen::VectorXd& factor_update_row = row_solve.solution;
    const double entering_bound =
        entering.move > 0 ? state.bounds.lower[entering.col]
                          : state.bounds.upper[entering.col];
    if (!std::isfinite(entering_bound)) {
      return make_result(state, Status::NumericalFailure,
                         "primal entering variable has no finite active bound",
                         statistics);
    }
    const double primal_delta = entering.move * leaving.step;
    Eigen::VectorXd candidate_x_basic =
        state.x_basic - entering.direction * primal_delta;
    candidate_x_basic[leaving.row] = entering_bound + primal_delta;

    const Eigen::VectorXd pivot_row =
        multiply_AT(state.sf->A, factor_update_row);
    if (pivot_row.size() != state.n || !pivot_row.allFinite()) {
      return make_result(state, Status::NumericalFailure,
                         "primal pivotal-row pricing failed", statistics);
    }
    const double pivot = entering.direction[leaving.row];
    const double dual_step = state.reduced_costs[entering.col] / pivot;
    Eigen::VectorXd candidate_reduced_costs =
        state.reduced_costs - dual_step * pivot_row;
    for (int row = 0; row < state.m; ++row) {
      if (row == leaving.row) continue;
      candidate_reduced_costs[
          state.basis[static_cast<std::size_t>(row)]] = 0.0;
    }
    candidate_reduced_costs[entering.col] = 0.0;
    if (!candidate_x_basic.allFinite() ||
        !candidate_reduced_costs.allFinite()) {
      return make_result(state, Status::NumericalFailure,
                         "primal revised-state update is non-finite",
                         statistics);
    }
    const double candidate_objective =
        state.objective + state.reduced_costs[entering.col] * primal_delta;
    if (!std::isfinite(candidate_objective)) {
      return make_result(state, Status::NumericalFailure,
                         "primal objective update is non-finite", statistics);
    }
    state.move[static_cast<std::size_t>(leaving_col)] =
        state.bounds.enterable[static_cast<std::size_t>(leaving_col)]
            ? (leaving.side < 0 ? Move::Up : Move::Down)
            : Move::Fixed;
    state.move[static_cast<std::size_t>(entering.col)] = Move::Fixed;
    const bool refined_pivotal_solve =
        direction_solve.refined || row_solve.refined;
    if (!refined_pivotal_solve &&
        !state.factor->update(leaving.row, entering.col, entering.direction,
                              factor_update_row, failure)) {
      return make_result(state, Status::NumericalFailure,
                         "primal basis update failed: " + failure,
                         statistics);
    }
    // DSE belongs to revised-dual pricing. This primal basis exchange makes
    // any previously cached row norms stale; warm dual entry rebuilds them.
    state.edge_weight.clear();
    state.basis[static_cast<std::size_t>(leaving.row)] = entering.col;
    state.basic[static_cast<std::size_t>(leaving_col)] = 0;
    state.basic[static_cast<std::size_t>(entering.col)] = 1;
    state.x_basic = std::move(candidate_x_basic);
    state.reduced_costs = std::move(candidate_reduced_costs);
    state.objective = candidate_objective;
    ++statistics.iterations;
    ++state.updates_since_rebuild;
    state.fresh_rebuild = false;
    if (refined_pivotal_solve &&
        !rebuild_primal_state(state, statistics, failure)) {
      return make_result(state, Status::NumericalFailure,
                         std::move(failure), statistics);
    }
  }
  return make_result(state, Status::IterationLimit,
                     "primal simplex iteration limit", statistics);
}

}  // namespace mipsolvers::engine::native_dual::detail
