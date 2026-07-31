#include "primal.hpp"
#include "pricing.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>

namespace mipsolvers::engine::native_dual::detail {
namespace {

constexpr int kPrimalRebuildInterval = 256;

double primal_clock() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct PrimalProfile {
  explicit PrimalProfile(Phase phase_) : phase(phase_) {}

  Phase phase;
  bool enabled{std::getenv("MIPSOLVERS_DS_PROFILE") != nullptr};
  double start{enabled ? primal_clock() : 0.0};
  double heap{0.0};
  double choose{0.0};
  double ftran{0.0};
  double ratio{0.0};
  double btran{0.0};
  double price{0.0};
  double update{0.0};
  long pivots{0};
  long ftran_calls{0};
  long btran_calls{0};
  long long ftran_rhs_nnz{0};
  long long ftran_result_nnz{0};
  long long btran_result_nnz{0};
  static constexpr int kAgeBuckets = 8;
  double ftran_age_time[kAgeBuckets]{};
  long ftran_age_calls[kAgeBuckets]{};
  long long ftran_age_result_nnz[kAgeBuckets]{};

  ~PrimalProfile() {
    if (!enabled) return;
    const double total = primal_clock() - start;
    const double known = heap + choose + ftran + ratio + btran + price + update;
    std::fprintf(stderr,
                 "[PRIMAL-PROFILE] phase=%s pivots=%ld total=%.3fs heap=%.3f "
                 "choose=%.3f ftran=%.3f ratio=%.3f btran=%.3f "
                 "price=%.3f update=%.3f other=%.3f ms/pivot=%.4f\n",
                 phase == Phase::One ? "I" : "II", pivots, total, heap,
                 choose, ftran, ratio, btran, price,
                 update, total - known,
                 pivots > 0 ? 1000.0 * total / pivots : 0.0);
    std::fprintf(
        stderr,
        "[PRIMAL-FTRAN] phase=%s calls=%ld rhs_nnz=%.1f result_nnz=%.1f "
        "update_btran_calls=%ld update_btran_result_nnz=%.1f\n",
        phase == Phase::One ? "I" : "II", ftran_calls,
        ftran_calls > 0
            ? static_cast<double>(ftran_rhs_nnz) / ftran_calls
            : 0.0,
        ftran_calls > 0
            ? static_cast<double>(ftran_result_nnz) / ftran_calls
            : 0.0,
        btran_calls,
        btran_calls > 0
            ? static_cast<double>(btran_result_nnz) / btran_calls
            : 0.0);
    std::fprintf(stderr, "[PRIMAL-FTRAN-AGE] phase=%s",
                 phase == Phase::One ? "I" : "II");
    for (int bucket = 0; bucket < kAgeBuckets; ++bucket) {
      const long calls = ftran_age_calls[bucket];
      if (calls == 0) continue;
      std::fprintf(stderr, " %d-%d:calls=%ld,time=%.3f,nnz=%.1f",
                   bucket * 32, bucket == kAgeBuckets - 1 ? 255
                                                          : bucket * 32 + 31,
                   calls, ftran_age_time[bucket],
                   static_cast<double>(ftran_age_result_nnz[bucket]) / calls);
    }
    std::fprintf(stderr, "\n");
  }
};

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
  IndexedVector direction;
};

struct PrimalLeaving {
  int row{-1};
  int side{0};
  double step{std::numeric_limits<double>::infinity()};
  // Direction entry at `row`, captured at selection so the commit path does
  // not need a coordinate lookup into the packed direction vector.
  double direction_value{0.0};
};

struct PrimalPricingHeap {
  struct Entry {
    double gain{0.0};
    int col{-1};
  };
  std::vector<Entry> entries;
  std::vector<int> position;
};

bool lower_entering_priority(const PrimalPricingHeap::Entry& lhs,
                             const PrimalPricingHeap::Entry& rhs) {
  if (lhs.gain != rhs.gain) return lhs.gain < rhs.gain;
  return lhs.col > rhs.col;
}

void swap_heap_entries(PrimalPricingHeap& heap, int lhs, int rhs) {
  if (lhs == rhs) return;
  std::swap(heap.entries[static_cast<std::size_t>(lhs)],
            heap.entries[static_cast<std::size_t>(rhs)]);
  heap.position[static_cast<std::size_t>(
      heap.entries[static_cast<std::size_t>(lhs)].col)] = lhs;
  heap.position[static_cast<std::size_t>(
      heap.entries[static_cast<std::size_t>(rhs)].col)] = rhs;
}

void sift_entering_up(PrimalPricingHeap& heap, int position) {
  while (position > 0) {
    const int parent = (position - 1) / 2;
    if (!lower_entering_priority(
            heap.entries[static_cast<std::size_t>(parent)],
            heap.entries[static_cast<std::size_t>(position)]))
      break;
    swap_heap_entries(heap, parent, position);
    position = parent;
  }
}

void sift_entering_down(PrimalPricingHeap& heap, int position) {
  const int count = static_cast<int>(heap.entries.size());
  for (;;) {
    const int left = 2 * position + 1;
    if (left >= count) return;
    const int right = left + 1;
    int best = left;
    if (right < count && lower_entering_priority(
                             heap.entries[static_cast<std::size_t>(left)],
                             heap.entries[static_cast<std::size_t>(right)])) {
      best = right;
    }
    if (!lower_entering_priority(
            heap.entries[static_cast<std::size_t>(position)],
            heap.entries[static_cast<std::size_t>(best)]))
      return;
    swap_heap_entries(heap, position, best);
    position = best;
  }
}

void remove_entering_column(PrimalPricingHeap& heap, int col) {
  const int position = heap.position[static_cast<std::size_t>(col)];
  if (position < 0) return;
  const int last = static_cast<int>(heap.entries.size()) - 1;
  swap_heap_entries(heap, position, last);
  heap.entries.pop_back();
  heap.position[static_cast<std::size_t>(col)] = -1;
  if (position >= static_cast<int>(heap.entries.size())) return;
  const int parent = (position - 1) / 2;
  if (position > 0 && lower_entering_priority(
                          heap.entries[static_cast<std::size_t>(parent)],
                          heap.entries[static_cast<std::size_t>(position)])) {
    sift_entering_up(heap, position);
  } else {
    sift_entering_down(heap, position);
  }
}

void refresh_entering_column(const State& state, PrimalPricingHeap& heap,
                             int col) {
  if (col < 0 || col >= state.n) return;
  remove_entering_column(heap, col);
  if (state.basic[static_cast<std::size_t>(col)]) return;
  const int move = sign(state.move[static_cast<std::size_t>(col)]);
  if (move == 0) return;
  const double gain = move * state.reduced_costs[col];
  if (gain <= state.options->optimality_tol) return;
  const int position = static_cast<int>(heap.entries.size());
  heap.entries.push_back({gain, col});
  heap.position[static_cast<std::size_t>(col)] = position;
  sift_entering_up(heap, position);
}

void initialize_entering_heap(const State& state, PrimalPricingHeap& heap) {
  heap.entries.clear();
  heap.position.assign(static_cast<std::size_t>(state.n), -1);
  heap.entries.reserve(static_cast<std::size_t>(state.n));
  for (int col = 0; col < state.n; ++col)
    refresh_entering_column(state, heap, col);
}

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

bool choose_entering(const State& state, PrimalPricingHeap& heap,
                     PrimalEntering& entering) {
  if (heap.entries.empty()) return false;
  const PrimalPricingHeap::Entry& top = heap.entries.front();
  entering.col = top.col;
  entering.move = sign(state.move[static_cast<std::size_t>(top.col)]);
  entering.gain = top.gain;
  return true;
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
  for (std::size_t k = 0; k < entering.direction.index.size(); ++k) {
    const int row = entering.direction.index[k];
    const double change = -entering.move * entering.direction.value[k];
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
  for (std::size_t k = 0; k < entering.direction.index.size(); ++k) {
    const int row = entering.direction.index[k];
    const double direction = entering.direction.value[k];
    const double change = -entering.move * direction;
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
    const double pivot = std::abs(direction);
    if (pivot > best_pivot ||
        (pivot == best_pivot && (leaving.row < 0 || row < leaving.row))) {
      best_pivot = pivot;
      leaving.row = row;
      leaving.side = side;
      leaving.step = exact_step;
      leaving.direction_value = direction;
    }
  }
  return std::isfinite(relaxed_step);
}

}  // namespace

Result run_primal_phase(
    State& state, Statistics& statistics,
    const std::chrono::steady_clock::time_point& solve_start) {
  PrimalProfile profile(state.phase);
  PrimalPricingHeap pricing_heap;
  const double heap_start = profile.enabled ? primal_clock() : 0.0;
  initialize_entering_heap(state, pricing_heap);
  if (profile.enabled) profile.heap += primal_clock() - heap_start;
  while (statistics.iterations < state.options->max_iter) {
    if (wall_time_hit(*state.options, solve_start)) {
      return make_result(state, Status::TimeLimit, "primal simplex time limit",
                         statistics);
    }
    PrimalEntering entering;
    const double choose_start = profile.enabled ? primal_clock() : 0.0;
    if (!choose_entering(state, pricing_heap, entering)) {
      if (profile.enabled) profile.choose += primal_clock() - choose_start;
      if (state.updates_since_rebuild > 0) {
        std::string failure;
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
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
    if (profile.enabled) profile.choose += primal_clock() - choose_start;

    IndexedVector column;
    column.dimension = state.m;
    for (Eigen::SparseMatrix<double>::InnerIterator it(state.sf->A,
                                                        entering.col);
         it; ++it) {
      column.index.push_back(it.row());
      column.value.push_back(it.value());
    }
    const double ftran_start = profile.enabled ? primal_clock() : 0.0;
    IndexedSolveEvidence direction_solve =
        state.factor->indexed_ftran(column, true);
    if (profile.enabled) {
      const double elapsed = primal_clock() - ftran_start;
      profile.ftran += elapsed;
      ++profile.ftran_calls;
      profile.ftran_rhs_nnz += static_cast<long long>(column.index.size());
      profile.ftran_result_nnz +=
          static_cast<long long>(direction_solve.solution.index.size());
      const int age_bucket = std::min(
          PrimalProfile::kAgeBuckets - 1, state.updates_since_rebuild / 32);
      profile.ftran_age_time[age_bucket] += elapsed;
      ++profile.ftran_age_calls[age_bucket];
      profile.ftran_age_result_nnz[age_bucket] +=
          static_cast<long long>(direction_solve.solution.index.size());
    }
    if (!direction_solve.accepted) {
      if (state.updates_since_rebuild > 0) {
        std::string failure;
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "fresh packed primal pivotal-column FTRAN failed",
                         statistics);
    }
    entering.direction = std::move(direction_solve.solution);

    PrimalLeaving leaving;
    const double ratio_start = profile.enabled ? primal_clock() : 0.0;
    if (!choose_leaving_harris(state, entering, leaving)) {
      if (state.updates_since_rebuild > 0) {
        std::string failure;
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
      return make_result(
          state, Status::NumericalFailure,
          "fresh primal ratio test has no finite limiting bound; an "
          "unbounded certificate is required",
          statistics);
    }
    if (profile.enabled) profile.ratio += primal_clock() - ratio_start;

    const double q_range =
        state.bounds.upper[entering.col] - state.bounds.lower[entering.col];
    if (std::isfinite(q_range) &&
        (leaving.row < 0 || q_range < leaving.step)) {
      const double primal_delta = entering.move * q_range;
      for (std::size_t k = 0; k < entering.direction.index.size(); ++k) {
        const int row = entering.direction.index[k];
        const double value =
            state.x_basic[row] - entering.direction.value[k] * primal_delta;
        if (!std::isfinite(value)) {
          return make_result(state, Status::NumericalFailure,
                             "packed primal bound flip is non-finite",
                             statistics);
        }
        state.x_basic[row] = value;
      }
      state.objective += state.reduced_costs[entering.col] * primal_delta;
      state.move[static_cast<std::size_t>(entering.col)] =
          entering.move > 0 ? Move::Down : Move::Up;
      refresh_entering_column(state, pricing_heap, entering.col);
      ++statistics.bound_flips;
      ++statistics.iterations;
      ++profile.pivots;
      continue;
    }
    if (leaving.row < 0 || leaving.direction_value == 0.0) {
      return make_result(state, Status::NumericalFailure,
                         "Harris primal ratio test selected no algebraic pivot",
                         statistics);
    }

    const int leaving_col =
        state.basis[static_cast<std::size_t>(leaving.row)];
    IndexedVector unit;
    unit.dimension = state.m;
    unit.index.push_back(leaving.row);
    unit.value.push_back(1.0);
    const double btran_start = profile.enabled ? primal_clock() : 0.0;
    const IndexedSolveEvidence row_solve =
        state.factor->indexed_btran(unit, true);
    if (profile.enabled) {
      profile.btran += primal_clock() - btran_start;
      ++profile.btran_calls;
      profile.btran_result_nnz +=
          static_cast<long long>(row_solve.solution.index.size());
    }
    std::string failure;
    if (!row_solve.accepted) {
      if (state.updates_since_rebuild > 0) {
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "fresh packed primal basis-update BTRAN failed",
                         statistics);
    }
    const IndexedVector& factor_update_row = row_solve.solution;
    const double entering_bound =
        entering.move > 0 ? state.bounds.lower[entering.col]
                          : state.bounds.upper[entering.col];
    if (!std::isfinite(entering_bound)) {
      return make_result(state, Status::NumericalFailure,
                         "primal entering variable has no finite active bound",
                         statistics);
    }
    const double primal_delta = entering.move * leaving.step;
    static thread_local std::vector<std::pair<int, double>> primal_changes;
    primal_changes.clear();
    primal_changes.reserve(entering.direction.index.size() + 1);
    for (std::size_t k = 0; k < entering.direction.index.size(); ++k) {
      const int row = entering.direction.index[k];
      if (row == leaving.row) continue;
      const double value =
          state.x_basic[row] - entering.direction.value[k] * primal_delta;
      if (!std::isfinite(value)) {
        return make_result(state, Status::NumericalFailure,
                           "packed primal state update is non-finite",
                           statistics);
      }
      primal_changes.emplace_back(row, value);
    }
    const double leaving_value = entering_bound + primal_delta;
    if (!std::isfinite(leaving_value)) {
      return make_result(state, Status::NumericalFailure,
                         "packed primal state update is non-finite",
                         statistics);
    }
    primal_changes.emplace_back(leaving.row, leaving_value);

    if (state.sf->A_row.rows() != state.m ||
        state.sf->A_row.cols() != state.n) {
      return make_result(state, Status::NumericalFailure,
                         "primal PRICE row matrix is inconsistent", statistics);
    }
    const double price_start = profile.enabled ? primal_clock() : 0.0;
    const IndexedVector pivot_row =
        multiply_AT_indexed(state.sf->A_row, factor_update_row);
    if (profile.enabled) profile.price += primal_clock() - price_start;
    if (!pivot_row.finite()) {
      return make_result(state, Status::NumericalFailure,
                         "packed primal pivotal-row pricing failed", statistics);
    }
    const double pivot = leaving.direction_value;
    const double dual_step = state.reduced_costs[entering.col] / pivot;
    if (!std::isfinite(dual_step)) {
      if (state.updates_since_rebuild > 0) {
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "packed primal dual step is non-finite", statistics);
    }
    bool reduced_cost_update_finite = true;
    for (std::size_t k = 0; k < pivot_row.index.size(); ++k) {
      const int col = pivot_row.index[k];
      const double value =
          state.reduced_costs[col] - dual_step * pivot_row.value[k];
      if (!std::isfinite(value)) {
        reduced_cost_update_finite = false;
        break;
      }
    }
    if (!reduced_cost_update_finite) {
      if (state.updates_since_rebuild > 0) {
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "fresh packed primal reduced-cost update is non-finite",
                         statistics);
    }
    const double candidate_objective =
        state.objective + state.reduced_costs[entering.col] * primal_delta;
    if (!std::isfinite(candidate_objective)) {
      if (state.updates_since_rebuild > 0) {
        if (!rebuild_primal_state(state, statistics, failure)) {
          return make_result(state, Status::NumericalFailure,
                             std::move(failure), statistics);
        }
        initialize_entering_heap(state, pricing_heap);
        continue;
      }
      return make_result(state, Status::NumericalFailure,
                         "fresh primal objective update is non-finite",
                         statistics);
    }
    const double update_start = profile.enabled ? primal_clock() : 0.0;
    if (!state.factor->update_indexed(leaving.row, entering.col, failure)) {
      return make_result(state, Status::NumericalFailure,
                         "primal basis update failed: " + failure,
                         statistics);
    }
    if (profile.enabled) profile.update += primal_clock() - update_start;
    state.move[static_cast<std::size_t>(leaving_col)] =
        state.bounds.enterable[static_cast<std::size_t>(leaving_col)]
            ? (leaving.side < 0 ? Move::Up : Move::Down)
            : Move::Fixed;
    state.move[static_cast<std::size_t>(entering.col)] = Move::Fixed;
    state.edge_weight.clear();
    state.basis[static_cast<std::size_t>(leaving.row)] = entering.col;
    state.basic[static_cast<std::size_t>(leaving_col)] = 0;
    state.basic[static_cast<std::size_t>(entering.col)] = 1;
    for (const auto& [row, value] : primal_changes) state.x_basic[row] = value;
    for (std::size_t k = 0; k < pivot_row.index.size(); ++k) {
      const int col = pivot_row.index[k];
      if (col != leaving_col && col != entering.col &&
          state.basic[static_cast<std::size_t>(col)]) {
        state.reduced_costs[col] = 0.0;
      } else {
        state.reduced_costs[col] -= dual_step * pivot_row.value[k];
      }
      refresh_entering_column(state, pricing_heap, col);
    }
    state.reduced_costs[entering.col] = 0.0;
    refresh_entering_column(state, pricing_heap, entering.col);
    refresh_entering_column(state, pricing_heap, leaving_col);
    state.objective = candidate_objective;
    ++statistics.iterations;
    ++profile.pivots;
    ++state.updates_since_rebuild;
    state.fresh_rebuild = false;
    if (state.factor->needs_rebuild() ||
        state.updates_since_rebuild >= kPrimalRebuildInterval) {
      if (!rebuild_primal_state(state, statistics, failure)) {
        return make_result(state, Status::NumericalFailure,
                           std::move(failure), statistics);
      }
      initialize_entering_heap(state, pricing_heap);
    }
  }
  return make_result(state, Status::IterationLimit,
                     "primal simplex iteration limit", statistics);
}

}  // namespace mipsolvers::engine::native_dual::detail
