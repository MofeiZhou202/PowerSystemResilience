#include "pricing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "mipsolvers/util/thread_pool.hpp"

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace mipsolvers::engine::native_dual::detail {
namespace {

struct Candidate {
  // Harris and BFRT need only this sufficient statistic. Margin is already
  // represented by breakpoint, and an accepted pivot is recovered exactly as
  // leaving_side * move_sign * alpha because both signs are +/-1.
  int col{-1};
  double alpha{0.0};
  long double breakpoint{0.0};
  double range{std::numeric_limits<double>::infinity()};
  bool taboo{false};
  int taboo_expiry{-1};
};

struct BfrtScanEvaluation {
  int col{-1};
  int direction{0};
  double signed_alpha{0.0};
  double range{0.0};
  double error{0.0};
  double margin{0.0};
  long double breakpoint{0.0};
  bool active{false};
  bool positive{false};
  bool stability_blocked{false};
  bool prefiltered{false};
  bool certified{false};
  bool stable{false};
  bool candidate{false};
  bool finite{true};
};

enum BfrtPrefilterFlag : std::uint8_t {
  kBfrtPositive = 1U << 0,
  kBfrtPrefiltered = 1U << 1,
  kBfrtCheapCertified = 1U << 2,
  kBfrtNeedsExact = 1U << 3,
};

struct BfrtPrefilterWorkspace {
  // Range is derived from immutable bounds and only positive lanes consume it.
  // Deferring that subtraction to the ordered merge avoids a write/read value
  // stream for every active lane and all bound traffic for nonpositive lanes.
  std::vector<double> signed_alpha;
  std::vector<std::uint8_t> flags;

  void resize(std::size_t count) {
    signed_alpha.resize(count);
    flags.resize(count);
  }
};

double price_tiny() {
  static const double value = [] {
    const char* environment = std::getenv("MIPSOLVERS_PRICE_TIGHT");
    if (environment != nullptr && std::string(environment) == "off") {
      return 0.0;
    }
    return 1e-14;
  }();
  return value;
}

bool lower_leaving_priority(const State::LeavingHeapEntry& lhs,
                            const State::LeavingHeapEntry& rhs) {
  if (lhs.merit != rhs.merit) return lhs.merit < rhs.merit;
  return lhs.row > rhs.row;
}

bool push_leaving_row(State& state, int row, std::string* failure) {
  if (row < 0 || row >= state.m) {
    if (failure != nullptr) {
      *failure = "incremental CHUZR received an invalid row";
    }
    return false;
  }
  const std::size_t index = static_cast<std::size_t>(row);
  const std::uint64_t version = ++state.leaving_row_version[index];
  int side = 0;
  const double violation = primal_infeasibility(state, row, side);
  if (side == 0) return true;
  const double weight = state.edge_weight[index];
  if (!(weight > 0.0) || !std::isfinite(weight)) {
    if (failure != nullptr) {
      *failure = "CHUZR encountered an invalid DSE weight";
    }
    return false;
  }
  state.leaving_heap.push_back({violation * violation / weight, row, version});
  std::push_heap(state.leaving_heap.begin(), state.leaving_heap.end(),
                 lower_leaving_priority);
  return true;
}

bool rebuild_leaving_heap(State& state, std::string& failure) {
  state.leaving_heap.clear();
  state.leaving_row_version.assign(static_cast<std::size_t>(state.m), 0);
  state.leaving_heap.reserve(static_cast<std::size_t>(state.m));
  for (int row = 0; row < state.m; ++row) {
    if (!push_leaving_row(state, row, &failure)) {
      state.leaving_heap.clear();
      state.leaving_heap_valid = false;
      return false;
    }
  }
  state.leaving_heap_valid = true;
  return true;
}

void remove_stale_leaving_entries(State& state) {
  while (!state.leaving_heap.empty()) {
    const State::LeavingHeapEntry& top = state.leaving_heap.front();
    if (top.row >= 0 && top.row < state.m &&
        top.version ==
            state.leaving_row_version[static_cast<std::size_t>(top.row)]) {
      return;
    }
    std::pop_heap(state.leaving_heap.begin(), state.leaving_heap.end(),
                  lower_leaving_priority);
    state.leaving_heap.pop_back();
  }
}

struct CertifiedLeavingCandidate {
  int row{-1};
  int side{0};
  int taboo_expiry{-1};
  double violation{0.0};
  long double merit_upper{0.0L};
  bool taboo{false};
};

bool choose_leaving_certified_exact(State& state, Leaving& leaving,
                                    std::string& failure) {
  struct ScopedCertificationTimer {
    double& total_sec;
    std::chrono::steady_clock::time_point start{
        std::chrono::steady_clock::now()};

    ~ScopedCertificationTimer() {
      total_sec +=
          std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
              .count();
    }
  } timer{state.certified_dse_time_sec};
  leaving = {};

  std::vector<long double> basis_column_norm(
      static_cast<std::size_t>(state.m), 0.0L);
  long double basis_transpose_inf_norm = 0.0L;
  for (int row = 0; row < state.m; ++row) {
    const int col = state.basis[static_cast<std::size_t>(row)];
    long double norm_one = 0.0L;
    long double norm_two_squared = 0.0L;
    for (StandardColumnMatrix::InnerIterator it(state.sf->A, col); it; ++it) {
      const long double magnitude = std::abs(
          static_cast<long double>(it.value()));
      norm_one += magnitude;
      norm_two_squared += magnitude * magnitude;
    }
    if (!(norm_two_squared > 0.0L) || !std::isfinite(norm_two_squared)) {
      failure = "certified CHUZR encountered a zero or non-finite basis column";
      return false;
    }
    basis_column_norm[static_cast<std::size_t>(row)] =
        std::sqrt(norm_two_squared);
    basis_transpose_inf_norm =
        std::max(basis_transpose_inf_norm, norm_one);
  }

  // The checked BTRAN contract accepts ||e_i-B^T y||_inf <=
  // 256 eps (||B^T||_inf ||y||_inf + 1). Account additionally for the
  // m-term dot-product rounding in that residual check. From
  // b_i^T y = 1-r_i and Cauchy-Schwarz, every accepted row satisfies the
  // lower bound below. It gives a proof-safe merit upper bound before BTRAN.
  const long double eps =
      static_cast<long double>(std::numeric_limits<double>::epsilon());
  const long double solve_constant = 256.0L * eps;
  const long double dot_product = static_cast<long double>(state.m) * eps;
  const long double dot_gamma =
      dot_product < 0.5L ? dot_product / (1.0L - dot_product) : 1.0L;
  const long double solve_slope = solve_constant + dot_gamma;
  const long double numerator = 1.0L - solve_constant;

  std::vector<CertifiedLeavingCandidate> candidates;
  candidates.reserve(static_cast<std::size_t>(state.m));
  int earliest_taboo_expiry = std::numeric_limits<int>::max();
  int taboo_candidate_count = 0;
  bool have_non_taboo = false;
  for (int row = 0; row < state.m; ++row) {
    int side = 0;
    const double violation = primal_infeasibility(state, row, side);
    if (side == 0) continue;
    const int leaving_col = state.basis[static_cast<std::size_t>(row)];
    int expiry = -1;
    const bool taboo = is_taboo_row(state, leaving_col, &expiry);
    have_non_taboo = have_non_taboo || !taboo;
    if (taboo) {
      ++taboo_candidate_count;
      earliest_taboo_expiry = std::min(earliest_taboo_expiry, expiry);
    }
    const long double denominator =
        basis_column_norm[static_cast<std::size_t>(row)] +
        solve_slope * basis_transpose_inf_norm;
    if (!(denominator > 0.0L) || !(numerator > 0.0L)) {
      failure = "certified CHUZR could not form a positive DSE lower bound";
      return false;
    }
    const long double weight_lower =
        (numerator / denominator) * (numerator / denominator);
    const long double violation_ld = static_cast<long double>(violation);
    const long double merit_upper =
        violation_ld * violation_ld / weight_lower;
    if (!(merit_upper >= 0.0L) || !std::isfinite(merit_upper)) {
      failure = "certified CHUZR produced a non-finite merit upper bound";
      return false;
    }
    candidates.push_back(
        {row, side, expiry, violation, merit_upper, taboo});
  }

  if (candidates.empty()) {
    return true;
  }

  candidates.erase(
      std::remove_if(candidates.begin(), candidates.end(),
                     [&](const CertifiedLeavingCandidate& candidate) {
                       if (have_non_taboo) return candidate.taboo;
                       return candidate.taboo_expiry != earliest_taboo_expiry;
                     }),
      candidates.end());
  state.certified_dse_candidates += static_cast<int>(candidates.size());
  std::sort(candidates.begin(), candidates.end(),
            [](const CertifiedLeavingCandidate& lhs,
               const CertifiedLeavingCandidate& rhs) {
              if (lhs.merit_upper != rhs.merit_upper) {
                return lhs.merit_upper > rhs.merit_upper;
              }
              return lhs.row < rhs.row;
            });

  int best_row = -1;
  int best_side = 0;
  double best_violation = 0.0;
  long double best_merit = -1.0L;
  std::vector<std::pair<int, double>> sparse_row;
  for (const CertifiedLeavingCandidate& candidate : candidates) {
    if (best_row >= 0 &&
        (candidate.merit_upper < best_merit ||
         (candidate.merit_upper == best_merit && candidate.row > best_row))) {
      break;
    }
    if (!state.factor->basis_inverse_row_sparse_entries(candidate.row,
                                                         sparse_row)) {
      failure = "certified CHUZR checked BTRAN failed";
      return false;
    }
    ++state.certified_dse_btrans;
    long double exact_weight = 0.0L;
    for (const auto& [index, value] : sparse_row) {
      (void)index;
      const long double value_ld = static_cast<long double>(value);
      exact_weight += value_ld * value_ld;
    }
    if (!(exact_weight > 0.0L) || !std::isfinite(exact_weight)) {
      failure = "certified CHUZR checked BTRAN produced an invalid DSE weight";
      return false;
    }
    state.edge_weight[static_cast<std::size_t>(candidate.row)] =
        static_cast<double>(exact_weight);
    const long double violation_ld =
        static_cast<long double>(candidate.violation);
    const long double exact_merit =
        violation_ld * violation_ld / exact_weight;
    if (exact_merit > best_merit ||
        (exact_merit == best_merit &&
         (best_row < 0 || candidate.row < best_row))) {
      if (best_row >= 0) ++state.certified_dse_rejections;
      best_row = candidate.row;
      best_side = candidate.side;
      best_violation = candidate.violation;
      best_merit = exact_merit;
    } else {
      ++state.certified_dse_rejections;
    }
  }
  if (best_row < 0) {
    failure = "certified CHUZR did not certify an infeasible basis row";
    return false;
  }

  leaving.row = best_row;
  leaving.side = best_side;
  leaving.violation = best_violation;
  leaving.delta = best_side * best_violation;
  leaving.released_taboo_row = !have_non_taboo;
  leaving.taboo_row_rejections = have_non_taboo ? taboo_candidate_count : 0;

  static thread_local IndexedVector unit;
  unit.clear(state.m);
  unit.index.push_back(best_row);
  unit.value.push_back(1.0);
  IndexedSolveEvidence row_solve = state.factor->indexed_btran(unit, true);
  ++state.certified_dse_btrans;
  leaving.row_ep = std::move(row_solve.solution);
  if (!row_solve.accepted || !leaving.row_ep.finite()) {
    failure = "certified CHUZR pivotal BTRAN failed";
    return false;
  }
  long double pivotal_weight = 0.0L;
  for (double value : leaving.row_ep.value) {
    const long double value_ld = static_cast<long double>(value);
    pivotal_weight += value_ld * value_ld;
  }
  if (!(pivotal_weight > 0.0L) || !std::isfinite(pivotal_weight)) {
    failure = "certified CHUZR pivotal BTRAN produced an invalid DSE weight";
    return false;
  }
  state.edge_weight[static_cast<std::size_t>(best_row)] =
      static_cast<double>(pivotal_weight);
  state.leaving_heap_valid = false;
  state.leaving_heap.clear();
  return true;
}

double dot_error_bound(const State& state, const IndexedVector& row_ep, int col,
                       const std::vector<double>* dense_row_ep = nullptr) {
  double absolute_dot = 0.0;
  int terms = 0;
  for (StandardColumnMatrix::InnerIterator it(state.sf->A, col); it; ++it) {
    const double multiplier =
        dense_row_ep != nullptr
            ? (*dense_row_ep)[static_cast<std::size_t>(it.row())]
            : row_ep.at(it.row());
    absolute_dot += std::abs(multiplier * it.value());
    ++terms;
  }
  const double eps = std::numeric_limits<double>::epsilon();
  const double product = terms * eps;
  const double gamma = product < 0.5 ? product / (1.0 - product) : 1.0;
  if (absolute_dot == 0.0) return 0.0;
  return gamma * absolute_dot + 256.0 * eps * absolute_dot;
}

void prefilter_active_phase_two(const State& state, const Leaving& leaving,
                                const IndexedVector& pivot_row,
                                const std::vector<int>& active_position,
                                double row_ep_max_abs,
                                double stable_pivot_tolerance,
                                BfrtPrefilterWorkspace& output) {
  const std::size_t count = active_position.size();
  output.resize(count);
  std::size_t position = 0;
#if defined(__aarch64__)
  const float64x2_t zero = vdupq_n_f64(0.0);
  const float64x2_t side = vdupq_n_f64(static_cast<double>(leaving.side));
  const float64x2_t stable = vdupq_n_f64(stable_pivot_tolerance);
  const uint64x2_t all_ones = vdupq_n_u64(~std::uint64_t{0});
  for (; position + 1 < count; position += 2) {
    const std::size_t pivot_position0 =
        static_cast<std::size_t>(active_position[position]);
    const std::size_t pivot_position1 =
        static_cast<std::size_t>(active_position[position + 1]);
    const int column0 = pivot_row.index[pivot_position0];
    const int column1 = pivot_row.index[pivot_position1];
    const std::size_t index0 = static_cast<std::size_t>(column0);
    const std::size_t index1 = static_cast<std::size_t>(column1);
    const int64x2_t direction_integer = {
        static_cast<std::int64_t>(sign(state.move[index0])),
        static_cast<std::int64_t>(sign(state.move[index1]))};
    const float64x2_t direction = vcvtq_f64_s64(direction_integer);
    const float64x2_t pivot = {pivot_row.value[pivot_position0],
                               pivot_row.value[pivot_position1]};
    const float64x2_t coefficient = {state.bfrt_error_coef[index0],
                                     state.bfrt_error_coef[index1]};
    const float64x2_t alpha = vmulq_f64(vmulq_f64(side, direction), pivot);
    const float64x2_t cheap_error = vmulq_n_f64(coefficient, row_ep_max_abs);

    const uint64x2_t positive = vcgtq_f64(alpha, zero);
    const uint64x2_t above_stable = vcgtq_f64(alpha, stable);
    const uint64x2_t prefiltered =
        vandq_u64(positive, veorq_u64(above_stable, all_ones));
    const uint64x2_t nonpositive = veorq_u64(vcgtq_f64(alpha, zero), all_ones);
    const uint64x2_t nonpositive_ambiguous =
        vandq_u64(nonpositive, vcgtq_f64(vaddq_f64(alpha, cheap_error), zero));
    const uint64x2_t positive_eligible = vandq_u64(positive, above_stable);
    const uint64x2_t cheap_certified =
        vandq_u64(positive_eligible, vcgtq_f64(alpha, cheap_error));
    const uint64x2_t needs_exact = vorrq_u64(
        nonpositive_ambiguous,
        vandq_u64(positive_eligible, veorq_u64(cheap_certified, all_ones)));

    vst1q_f64(output.signed_alpha.data() + position, alpha);
    uint64x2_t flags = vshlq_n_u64(vshrq_n_u64(positive, 63), 0);
    flags = vorrq_u64(flags, vshlq_n_u64(vshrq_n_u64(prefiltered, 63), 1));
    flags = vorrq_u64(flags, vshlq_n_u64(vshrq_n_u64(cheap_certified, 63), 2));
    flags = vorrq_u64(flags, vshlq_n_u64(vshrq_n_u64(needs_exact, 63), 3));
    const std::uint8_t flags0 =
        static_cast<std::uint8_t>(vgetq_lane_u64(flags, 0));
    const std::uint8_t flags1 =
        static_cast<std::uint8_t>(vgetq_lane_u64(flags, 1));
    const std::uint16_t packed_flags =
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(flags0) |
                                   (static_cast<std::uint16_t>(flags1) << 8));
    std::memcpy(output.flags.data() + position, &packed_flags,
                sizeof(packed_flags));
  }
#endif
  for (; position < count; ++position) {
    const std::size_t pivot_position =
        static_cast<std::size_t>(active_position[position]);
    const int column = pivot_row.index[pivot_position];
    const int direction = sign(state.move[static_cast<std::size_t>(column)]);
    const double alpha =
        leaving.side * direction * pivot_row.value[pivot_position];
    const double cheap_error =
        state.bfrt_error_coef[static_cast<std::size_t>(column)] *
        row_ep_max_abs;
    const bool positive = alpha > 0.0;
    const bool above_stable = alpha > stable_pivot_tolerance;
    const bool prefiltered = positive && !above_stable;
    const bool nonpositive_ambiguous = !positive && alpha + cheap_error > 0.0;
    const bool cheap_certified =
        positive && above_stable && alpha > cheap_error;
    const bool needs_exact =
        nonpositive_ambiguous || (positive && above_stable && !cheap_certified);
    output.signed_alpha[position] = alpha;
    output.flags[position] = static_cast<std::uint8_t>(
        (positive ? kBfrtPositive : 0) | (prefiltered ? kBfrtPrefiltered : 0) |
        (cheap_certified ? kBfrtCheapCertified : 0) |
        (needs_exact ? kBfrtNeedsExact : 0));
  }
}

void multiply_AT_indexed_impl(const StandardRowMatrix& A_row,
                              const IndexedVector& y, IndexedVector& result,
                              const std::vector<char>* basic,
                              const std::vector<Move>* move,
                              std::vector<int>* active_position) {
  if (active_position != nullptr) active_position->clear();
  result.clear(static_cast<int>(A_row.cols()));
  struct Accumulator {
    std::vector<double> value;
    std::vector<unsigned int> stamp;
    std::vector<int> touched;
    unsigned int epoch{0};
  };
  static thread_local Accumulator accumulator;
  const std::size_t columns = static_cast<std::size_t>(A_row.cols());
  if (accumulator.value.size() != columns) {
    accumulator.value.assign(columns, 0.0);
    accumulator.stamp.assign(columns, 0);
    accumulator.epoch = 0;
  }
  if (++accumulator.epoch == 0) {
    std::fill(accumulator.stamp.begin(), accumulator.stamp.end(), 0);
    ++accumulator.epoch;
  }
  accumulator.touched.clear();
  for (std::size_t k = 0; k < y.index.size(); ++k) {
    const int row = y.index[k];
    const double multiplier = y.value[k];
    if (row < 0 || row >= A_row.rows() || multiplier == 0.0) continue;
    for (StandardRowMatrix::InnerIterator it(A_row, row); it; ++it) {
      const int col = static_cast<int>(it.col());
      const std::size_t index = static_cast<std::size_t>(col);
      if (accumulator.stamp[index] != accumulator.epoch) {
        accumulator.stamp[index] = accumulator.epoch;
        accumulator.value[index] = 0.0;
        accumulator.touched.push_back(col);
      }
      accumulator.value[index] += multiplier * it.value();
    }
  }
  const double tiny = price_tiny();
  result.index.reserve(accumulator.touched.size());
  result.value.reserve(accumulator.touched.size());
  if (active_position != nullptr) {
    active_position->reserve(accumulator.touched.size());
  }
  for (const int col : accumulator.touched) {
    const double value = accumulator.value[static_cast<std::size_t>(col)];
    if (!(std::abs(value) <= tiny)) {
      const int packed_position = static_cast<int>(result.index.size());
      result.index.push_back(col);
      result.value.push_back(value);
      if (active_position != nullptr &&
          !(*basic)[static_cast<std::size_t>(col)] &&
          sign((*move)[static_cast<std::size_t>(col)]) != 0) {
        active_position->push_back(packed_position);
      }
    }
  }
}

}  // namespace

IndexedVector multiply_AT_indexed(const StandardRowMatrix& A_row,
                                  const IndexedVector& y) {
  IndexedVector result;
  multiply_AT_indexed(A_row, y, result);
  return result;
}

void multiply_AT_indexed(const StandardRowMatrix& A_row, const IndexedVector& y,
                         IndexedVector& result) {
  multiply_AT_indexed_impl(A_row, y, result, nullptr, nullptr, nullptr);
}

void multiply_AT_indexed_bfrt(const StandardRowMatrix& A_row,
                              const IndexedVector& y,
                              const std::vector<char>& basic,
                              const std::vector<Move>& move,
                              IndexedVector& result,
                              std::vector<int>& active_position) {
  multiply_AT_indexed_impl(A_row, y, result, &basic, &move, &active_position);
}

void multiply_AT_indexed_csc(const StandardColumnMatrix& A,
                             const IndexedVector& y, IndexedVector& result,
                             int thread_count) {
  const bool parallel_csc = thread_count > 1;

  result.clear(static_cast<int>(A.cols()));
  struct DensePriceScratch {
    std::vector<double> pivotal_row;
    std::vector<double> product;
  };
  static thread_local DensePriceScratch scratch;
  scratch.pivotal_row.assign(static_cast<std::size_t>(A.rows()), 0.0);
  for (std::size_t position = 0; position < y.index.size(); ++position) {
    const int row = y.index[position];
    if (row >= 0 && row < A.rows()) {
      scratch.pivotal_row[static_cast<std::size_t>(row)] = y.value[position];
    }
  }
  scratch.product.resize(static_cast<std::size_t>(A.cols()));
  const double* pivotal_row_values = scratch.pivotal_row.data();
  double* product_values = scratch.product.data();

  auto price_range = [&](std::size_t begin, std::size_t end) {
    auto compute = [&](const auto& matrix) {
      const auto* outer = matrix.outerIndexPtr();
      const auto* inner = matrix.innerIndexPtr();
      const double* values = matrix.valuePtr();
      for (std::size_t column = begin; column < end; ++column) {
        double sum = 0.0;
        const auto first = outer[column];
        const auto last = outer[column + 1];
        for (auto position = first; position < last; ++position) {
          sum += values[position] *
                 pivotal_row_values[static_cast<std::size_t>(inner[position])];
        }
        product_values[column] = sum;
      }
    };
    if (const auto* narrow = A.narrow_matrix()) {
      compute(*narrow);
    } else {
      compute(*A.wide_matrix());
    }
  };
  if (parallel_csc) {
    mipsolvers::util::ThreadPool::global().parallel_for(
        static_cast<std::size_t>(A.cols()), price_range, thread_count);
  } else {
    price_range(0, static_cast<std::size_t>(A.cols()));
  }

  const double tiny = price_tiny();
  result.index.reserve(static_cast<std::size_t>(A.cols()));
  result.value.reserve(static_cast<std::size_t>(A.cols()));
  for (int column = 0; column < A.cols(); ++column) {
    const double value = scratch.product[static_cast<std::size_t>(column)];
    if (!(std::abs(value) <= tiny)) {
      result.index.push_back(column);
      result.value.push_back(value);
    }
  }
}

void refresh_leaving_heap(State& state, const std::vector<int>* changed_rows) {
  if (!state.leaving_heap_valid) return;
  if (changed_rows == nullptr ||
      state.leaving_row_version.size() != static_cast<std::size_t>(state.m)) {
    state.leaving_heap_valid = false;
    state.leaving_heap.clear();
    return;
  }
  for (int row : *changed_rows) {
    if (!push_leaving_row(state, row, nullptr)) {
      state.leaving_heap_valid = false;
      state.leaving_heap.clear();
      return;
    }
  }
  if (state.leaving_heap.size() >
      4 * static_cast<std::size_t>(std::max(1, state.m))) {
    state.leaving_heap_valid = false;
    state.leaving_heap.clear();
  }
}

bool choose_leaving(State& state, Leaving& leaving, std::string& failure) {
  if (state.certified_exact_dse_pricing &&
      state.edge_weight_mode == EdgeWeightMode::SteepestEdge) {
    return choose_leaving_certified_exact(state, leaving, failure);
  }
  leaving = {};
  if (!state.leaving_heap_valid && !rebuild_leaving_heap(state, failure)) {
    return false;
  }
  double taboo_merit = -1.0;
  int taboo_expiry = std::numeric_limits<int>::max();
  Leaving taboo_fallback;
  static thread_local std::vector<State::LeavingHeapEntry> deferred;
  deferred.clear();
  while (true) {
    remove_stale_leaving_entries(state);
    if (state.leaving_heap.empty()) break;
    std::pop_heap(state.leaving_heap.begin(), state.leaving_heap.end(),
                  lower_leaving_priority);
    const State::LeavingHeapEntry candidate = state.leaving_heap.back();
    state.leaving_heap.pop_back();
    deferred.push_back(candidate);
    const int row = candidate.row;
    int side = 0;
    const double violation = primal_infeasibility(state, row, side);
    if (side == 0) continue;
    const double merit = candidate.merit;
    const int leaving_col = state.basis[static_cast<std::size_t>(row)];
    int expiry = -1;
    if (is_taboo_row(state, leaving_col, &expiry)) {
      ++leaving.taboo_row_rejections;
      if (expiry < taboo_expiry ||
          (expiry == taboo_expiry &&
           (merit > taboo_merit ||
            (merit == taboo_merit &&
             (taboo_fallback.row < 0 || row < taboo_fallback.row))))) {
        taboo_expiry = expiry;
        taboo_merit = merit;
        taboo_fallback.row = row;
        taboo_fallback.side = side;
        taboo_fallback.violation = violation;
        taboo_fallback.delta = side * violation;
      }
      continue;
    }
    leaving.row = row;
    leaving.side = side;
    leaving.violation = violation;
    leaving.delta = side * violation;
    break;
  }
  for (const State::LeavingHeapEntry& entry : deferred) {
    state.leaving_heap.push_back(entry);
    std::push_heap(state.leaving_heap.begin(), state.leaving_heap.end(),
                   lower_leaving_priority);
  }
  if (leaving.row < 0 && taboo_fallback.row >= 0) {
    const int rejections = leaving.taboo_row_rejections;
    leaving = taboo_fallback;
    leaving.released_taboo_row = true;
    leaving.taboo_row_rejections = rejections;
  }
  if (leaving.row < 0) return true;

  static thread_local IndexedVector unit;
  unit.clear(state.m);
  unit.index.push_back(leaving.row);
  unit.value.push_back(1.0);
  IndexedSolveEvidence row_solve = state.factor->indexed_btran(unit, true);
  leaving.row_ep = std::move(row_solve.solution);
  if (!row_solve.accepted) {
    failure = "CHUZR packed BTRAN failed";
    return false;
  }
  if (state.edge_weight_mode == EdgeWeightMode::Devex) return true;

  double exact_weight = 0.0;
  for (double value : leaving.row_ep.value) exact_weight += value * value;
  const double stored_weight =
      state.edge_weight[static_cast<std::size_t>(leaving.row)];
  const double error = std::abs(exact_weight - stored_weight);
  const double error_limit =
      2048.0 * std::numeric_limits<double>::epsilon() *
      std::max(1.0, exact_weight);
  if (error > error_limit) {
    // DSE weights rank eligible leaving rows; they are not a feasibility
    // certificate. The selected row has now been recomputed by exact BTRAN,
    // so replace its advisory cached value before PRICE. Pivot validity is
    // checked independently by row/column agreement and reconstruction.
    state.edge_weight[static_cast<std::size_t>(leaving.row)] = exact_weight;
    if (!push_leaving_row(state, leaving.row, nullptr)) {
      state.leaving_heap_valid = false;
      state.leaving_heap.clear();
    }
  }
  return true;
}

bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const IndexedVector& pivot_row,
                          PivotTransaction& transaction, std::string& failure) {
  std::vector<int> active_position;
  active_position.reserve(pivot_row.index.size());
  for (std::size_t position = 0; position < pivot_row.index.size();
       ++position) {
    const int col = pivot_row.index[position];
    if (!state.basic[static_cast<std::size_t>(col)] &&
        sign(state.move[static_cast<std::size_t>(col)]) != 0) {
      active_position.push_back(static_cast<int>(position));
    }
  }
  return choose_entering_bfrt(state, leaving, pivot_row, active_position,
                              transaction, failure);
}

bool choose_entering_bfrt(const State& state, const Leaving& leaving,
                          const IndexedVector& pivot_row,
                          const std::vector<int>& active_position,
                          PivotTransaction& transaction, std::string& failure) {
  transaction = {};
  transaction.bfrt_rhs.clear(state.m);
  if (pivot_row.dimension != state.n || !pivot_row.finite()) {
    failure = "PRICE produced an invalid pivotal row";
    return false;
  }
  // Every ratio-test loop is bounded by the packed PRICE support.
  const int scan_count = static_cast<int>(active_position.size());
  auto scan_position = [&](int s) -> std::size_t {
    return static_cast<std::size_t>(
        active_position[static_cast<std::size_t>(s)]);
  };
  auto scan_col = [&](int s) -> int {
    return pivot_row.index[scan_position(s)];
  };
  auto scan_pivot = [&](int s) -> double {
    return pivot_row.value[scan_position(s)];
  };
  static thread_local std::vector<Candidate> candidates;
  candidates.clear();
  candidates.reserve(static_cast<std::size_t>(scan_count));
  const double stable_pivot_tolerance =
      state.updates_since_rebuild < 10
          ? 1e-9
          : (state.updates_since_rebuild < 20 ? 3e-8 : 1e-6);
  auto add_capacity = [](double alpha, double range, double& capacity) {
    if (!std::isfinite(range)) {
      capacity = std::numeric_limits<double>::infinity();
    } else if (std::isfinite(capacity)) {
      capacity += alpha * range;
    }
  };
  static thread_local std::vector<double> dense_row_ep;
  double row_ep_max_abs = 0.0;
  for (std::size_t k = 0; k < leaving.row_ep.index.size(); ++k) {
    row_ep_max_abs =
        std::max(row_ep_max_abs, std::abs(leaving.row_ep.value[k]));
  }
  // The dense scatter only serves dot_error_bound lookups; build it on the
  // first exact bound evaluation so fully short-circuited scans skip the
  // O(m) clear. The scattered values are identical either way.
  bool dense_row_ep_built = false;
  auto dense_row_ep_for_dot = [&]() -> const std::vector<double>* {
    if (!dense_row_ep_built) {
      dense_row_ep.assign(static_cast<std::size_t>(state.m), 0.0);
      for (std::size_t k = 0; k < leaving.row_ep.index.size(); ++k) {
        dense_row_ep[static_cast<std::size_t>(leaving.row_ep.index[k])] =
            leaving.row_ep.value[k];
      }
      dense_row_ep_built = true;
    }
    return &dense_row_ep;
  };
  const bool have_error_coef =
      state.bfrt_error_coef.size() == static_cast<std::size_t>(state.n);
  const int bfrt_leaving_col =
      state.basis[static_cast<std::size_t>(leaving.row)];
  // An empty taboo table cannot mark any exchange; skip the keyed lookup.
  const bool taboo_possible = !state.taboo_changes.empty();
  const bool branch_free_phase_two =
      state.phase == Phase::Two && have_error_coef;
  if (branch_free_phase_two) {
    static thread_local BfrtPrefilterWorkspace prefilter;
    prefilter_active_phase_two(state, leaving, pivot_row, active_position,
                               row_ep_max_abs, stable_pivot_tolerance,
                               prefilter);
    for (int s = 0; s < scan_count; ++s) {
      const std::size_t position = static_cast<std::size_t>(s);
      const int col = scan_col(s);
      const int direction = sign(state.move[static_cast<std::size_t>(col)]);
      const double alpha = prefilter.signed_alpha[position];
      const std::uint8_t flags = prefilter.flags[position];
      const bool positive = (flags & kBfrtPositive) != 0;
      double range = 0.0;
      if (positive) {
        range = state.bounds.upper[col] - state.bounds.lower[col];
        ++transaction.positive_candidate_count;
        add_capacity(alpha, range, transaction.positive_capacity);
      }
      if ((flags & kBfrtPrefiltered) != 0) {
        transaction.stability_blocked = true;
        ++transaction.unstable_pivot_rejections;
        ++transaction.bfrt_stability_prefiltered;
        continue;
      }
      if (!positive) {
        if ((flags & kBfrtNeedsExact) != 0) {
          const double error = dot_error_bound(state, leaving.row_ep, col,
                                               dense_row_ep_for_dot());
          if (alpha + error > 0.0) {
            transaction.stability_blocked = true;
            ++transaction.unstable_pivot_rejections;
          }
        }
        continue;
      }

      double error = 0.0;
      if ((flags & kBfrtCheapCertified) != 0) {
        error = state.bfrt_error_coef[static_cast<std::size_t>(col)] *
                row_ep_max_abs;
      } else {
        error =
            dot_error_bound(state, leaving.row_ep, col, dense_row_ep_for_dot());
        if (!(alpha > error)) {
          if (alpha + error > 0.0) {
            transaction.stability_blocked = true;
            ++transaction.unstable_pivot_rejections;
          }
          continue;
        }
      }
      ++transaction.certified_candidate_count;
      add_capacity(alpha, range, transaction.certified_capacity);
      if (!(alpha > stable_pivot_tolerance)) {
        transaction.stability_blocked = true;
        ++transaction.unstable_pivot_rejections;
        continue;
      }
      ++transaction.stable_candidate_count;
      add_capacity(alpha, range, transaction.stable_capacity);
      add_capacity(error, range, transaction.stable_capacity_error);
      const double margin =
          std::max(0.0, -direction * state.reduced_costs[col]);
      const long double breakpoint =
          static_cast<long double>(margin) / static_cast<long double>(alpha);
      if (!std::isfinite(breakpoint)) {
        failure = "BFRT breakpoint is not finite";
        return false;
      }
      int taboo_expiry = -1;
      const bool taboo =
          taboo_possible &&
          is_taboo_change(state, bfrt_leaving_col, col, &taboo_expiry);
      candidates.push_back(
          {col, alpha, breakpoint, range, taboo, taboo_expiry});
    }
  }
  if (!branch_free_phase_two) {
    constexpr int kMinParallelBfrtScan = 8192;
    const bool parallel_scan =
        state.kernel_threads > 1 && scan_count >= kMinParallelBfrtScan;
    if (parallel_scan) (void)dense_row_ep_for_dot();

    static thread_local std::vector<BfrtScanEvaluation> evaluations;
    evaluations.resize(static_cast<std::size_t>(scan_count));
    BfrtScanEvaluation* evaluation_output = evaluations.data();
    const std::vector<double>* parallel_dense_row_ep =
        parallel_scan ? &dense_row_ep : nullptr;
    auto evaluate_range = [&](std::size_t begin, std::size_t end) {
      for (std::size_t position = begin; position < end; ++position) {
        BfrtScanEvaluation evaluation;
        const int j = scan_col(static_cast<int>(position));
        evaluation.col = j;
        if (state.basic[static_cast<std::size_t>(j)]) {
          evaluation_output[position] = evaluation;
          continue;
        }
        evaluation.direction = sign(state.move[static_cast<std::size_t>(j)]);
        if (evaluation.direction == 0) {
          evaluation_output[position] = evaluation;
          continue;
        }
        evaluation.active = true;
        const double pivot = scan_pivot(static_cast<int>(position));
        evaluation.signed_alpha =
            leaving.side * evaluation.direction * pivot;
        evaluation.range = state.bounds.upper[j] - state.bounds.lower[j];
        evaluation.positive = evaluation.signed_alpha > 0.0;

        // In Phase II, CHUZC can only pivot on alpha > Ta. Reject smaller
        // positive pivots before the more expensive column error traversal.
        if (state.phase == Phase::Two && evaluation.positive &&
            !(evaluation.signed_alpha > stable_pivot_tolerance)) {
          evaluation.stability_blocked = true;
          evaluation.prefiltered = true;
          evaluation_output[position] = evaluation;
          continue;
        }

        if (!evaluation.positive) {
          if (have_error_coef) {
            const double cheap_error =
                state.bfrt_error_coef[static_cast<std::size_t>(j)] *
                row_ep_max_abs;
            if (!(evaluation.signed_alpha + cheap_error > 0.0)) {
              evaluation_output[position] = evaluation;
              continue;
            }
          }
          const std::vector<double>* dense =
              parallel_scan ? parallel_dense_row_ep : dense_row_ep_for_dot();
          evaluation.error = dot_error_bound(state, leaving.row_ep, j, dense);
          evaluation.stability_blocked =
              evaluation.signed_alpha + evaluation.error > 0.0;
          evaluation_output[position] = evaluation;
          continue;
        }

        if (state.phase == Phase::Two && have_error_coef &&
            evaluation.signed_alpha >
                state.bfrt_error_coef[static_cast<std::size_t>(j)] *
                    row_ep_max_abs) {
          evaluation.error =
              state.bfrt_error_coef[static_cast<std::size_t>(j)] *
              row_ep_max_abs;
        } else {
          const std::vector<double>* dense =
              parallel_scan ? parallel_dense_row_ep : dense_row_ep_for_dot();
          evaluation.error = dot_error_bound(state, leaving.row_ep, j, dense);
          if (!(evaluation.signed_alpha > evaluation.error)) {
            evaluation.stability_blocked =
                evaluation.signed_alpha + evaluation.error > 0.0;
            evaluation_output[position] = evaluation;
            continue;
          }
        }
        evaluation.certified = true;
        if (!(evaluation.signed_alpha > stable_pivot_tolerance)) {
          evaluation.stability_blocked = true;
          evaluation_output[position] = evaluation;
          continue;
        }
        evaluation.stable = true;
        evaluation.margin =
            std::max(0.0, -evaluation.direction * state.reduced_costs[j]);
        evaluation.breakpoint =
            static_cast<long double>(evaluation.margin) /
            static_cast<long double>(evaluation.signed_alpha);
        evaluation.finite = std::isfinite(evaluation.breakpoint);
        evaluation.candidate = evaluation.finite;
        evaluation_output[position] = evaluation;
      }
    };
    if (parallel_scan) {
      mipsolvers::util::ThreadPool::global().parallel_for(
          static_cast<std::size_t>(scan_count), evaluate_range,
          state.kernel_threads);
    } else {
      evaluate_range(0, static_cast<std::size_t>(scan_count));
    }

    // Merge in the original PRICE order. All capacity sums, counters and
    // candidate insertion order are therefore bitwise independent of the
    // worker count even though each column's classification ran in parallel.
    for (const BfrtScanEvaluation& evaluation : evaluations) {
      if (!evaluation.active) continue;
      if (evaluation.positive) {
        ++transaction.positive_candidate_count;
        add_capacity(evaluation.signed_alpha, evaluation.range,
                     transaction.positive_capacity);
      }
      if (evaluation.stability_blocked) {
        transaction.stability_blocked = true;
        ++transaction.unstable_pivot_rejections;
      }
      if (evaluation.prefiltered) {
        ++transaction.bfrt_stability_prefiltered;
      }
      if (!evaluation.certified) continue;
      ++transaction.certified_candidate_count;
      add_capacity(evaluation.signed_alpha, evaluation.range,
                   transaction.certified_capacity);
      if (!evaluation.stable) continue;
      ++transaction.stable_candidate_count;
      add_capacity(evaluation.signed_alpha, evaluation.range,
                   transaction.stable_capacity);
      add_capacity(evaluation.error, evaluation.range,
                   transaction.stable_capacity_error);
      if (!evaluation.finite) {
        failure = "BFRT breakpoint is not finite";
        return false;
      }
      int taboo_expiry = -1;
      const bool taboo =
          taboo_possible && is_taboo_change(state, bfrt_leaving_col,
                                            evaluation.col, &taboo_expiry);
      candidates.push_back({evaluation.col, evaluation.signed_alpha,
                            evaluation.breakpoint, evaluation.range, taboo,
                            taboo_expiry});
    }
  }
  if (candidates.empty()) return true;

  transaction.bfrt_candidate_count = static_cast<int>(candidates.size());
  std::size_t selected_begin = 0;
  std::size_t selected_end = 0;
  // HiGHS' BFRT uses the same 1e-12 initial total-change budget. It is an
  // EXPAND allowance for the independently reconstructed leaving violation
  // and pivotal-row capacity, five orders below the default primal tolerance.
  constexpr long double kInitialTotalChange = 1e-12L;
  long double covered = kInitialTotalChange;
  std::size_t last_group_begin = 0;
  std::size_t last_group_end = 0;
  bool remaining_sorted = false;
  constexpr int kPartialGroupLimit = 8;
  const auto order_start = std::chrono::steady_clock::now();
  for (std::size_t begin = 0; begin < candidates.size();) {
    std::size_t end = begin;
    if (!remaining_sorted &&
        transaction.bfrt_group_count < kPartialGroupLimit) {
      long double minimum_breakpoint = candidates[begin].breakpoint;
      for (std::size_t i = begin + 1; i < candidates.size(); ++i) {
        minimum_breakpoint =
            std::min(minimum_breakpoint, candidates[i].breakpoint);
      }
      for (std::size_t i = begin; i < candidates.size(); ++i) {
        if (candidates[i].breakpoint == minimum_breakpoint) {
          std::swap(candidates[end], candidates[i]);
          ++end;
        }
      }
      // The former full sort used the column as its deterministic tie-break.
      // Preserve that exact summation and transaction order within a group.
      std::sort(candidates.begin() + static_cast<std::ptrdiff_t>(begin),
                candidates.begin() + static_cast<std::ptrdiff_t>(end),
                [](const Candidate& lhs, const Candidate& rhs) {
                  return lhs.col < rhs.col;
                });
    } else {
      if (!remaining_sorted) {
        const auto sort_start = std::chrono::steady_clock::now();
        std::sort(candidates.begin() + static_cast<std::ptrdiff_t>(begin),
                  candidates.end(),
                  [](const Candidate& lhs, const Candidate& rhs) {
                    if (lhs.breakpoint != rhs.breakpoint)
                      return lhs.breakpoint < rhs.breakpoint;
                    return lhs.col < rhs.col;
                  });
        transaction.bfrt_sort_time_sec +=
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          sort_start)
                .count();
        remaining_sorted = true;
      }
      end = begin + 1;
      while (end < candidates.size() &&
             candidates[end].breakpoint == candidates[begin].breakpoint) {
        ++end;
      }
    }
    ++transaction.bfrt_group_count;
    last_group_begin = begin;
    last_group_end = end;
    long double group_change = 0.0;
    bool group_has_unbounded_range = false;
    for (std::size_t i = begin; i < end; ++i) {
      if (std::isfinite(candidates[i].range)) {
        const long double change =
            static_cast<long double>(candidates[i].alpha) *
            static_cast<long double>(candidates[i].range);
        group_change += change;
      } else {
        group_has_unbounded_range = true;
      }
    }
    if (group_has_unbounded_range ||
        covered + group_change >=
            static_cast<long double>(leaving.violation)) {
      selected_begin = begin;
      selected_end = end;
      break;
    }
    covered += group_change;
    begin = end;
  }
  transaction.bfrt_order_time_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    order_start)
          .count();

  if (selected_end <= selected_begin && state.phase == Phase::DualOne &&
      last_group_end > last_group_begin) {
    // In exact arithmetic z=0 is feasible, so a Phase-I pivotal row cannot
    // have insufficient signed capacity. On a freshly reconstructed floating
    // iterate the apparent shortage is bounded by pi^T(b-Ax), where
    // pi=B^{-T}e_p is the already certified pivotal row. Use that computed
    // residual projection as a local proof, not a feasibility-tolerance
    // relaxation, before accepting the final exact-cover group.
    const Eigen::VectorXd equation_residual =
        state.sf->b - state.sf->A * full_primal(state);
    long double projected_residual = 0.0L;
    long double projected_residual_abs = 0.0L;
    for (std::size_t k = 0; k < leaving.row_ep.index.size(); ++k) {
      const int row = leaving.row_ep.index[k];
      const long double term =
          static_cast<long double>(leaving.row_ep.value[k]) *
          static_cast<long double>(equation_residual[row]);
      projected_residual += term;
      projected_residual_abs += std::abs(term);
    }
    long double btran_witness_error = 0.0L;
    long double btran_witness_error_abs = 0.0L;
    for (int basis_row = 0; basis_row < state.m; ++basis_row) {
      const int basis_col =
          state.basis[static_cast<std::size_t>(basis_row)];
      long double bt_pi = 0.0L;
      for (StandardColumnMatrix::InnerIterator it(state.sf->A,
                                                          basis_col);
           it; ++it) {
        bt_pi += static_cast<long double>(leaving.row_ep.at(it.row())) *
                 static_cast<long double>(it.value());
      }
      const long double btran_residual =
          (basis_row == leaving.row ? 1.0L : 0.0L) - bt_pi;
      const long double basis_displacement =
          static_cast<long double>(state.x_basic[basis_row]) -
          static_cast<long double>(state.dual_phase_one_anchor[basis_col]);
      const long double term = btran_residual * basis_displacement;
      btran_witness_error += term;
      btran_witness_error_abs += std::abs(term);
    }
    const long double projection_roundoff =
        (static_cast<long double>(leaving.row_ep.index.size() + state.m) +
         256.0L) *
        static_cast<long double>(std::numeric_limits<double>::epsilon()) *
        std::max({projected_residual_abs, btran_witness_error_abs,
                  static_cast<long double>(leaving.violation), 1.0L});
    const long double shortage =
        static_cast<long double>(leaving.violation) - covered;
    if (equation_residual.allFinite() && shortage > 0.0L &&
        shortage <= std::abs(projected_residual) +
                        std::abs(btran_witness_error) + projection_roundoff +
                        static_cast<long double>(
                            transaction.stable_capacity_error)) {
      selected_begin = last_group_begin;
      selected_end = last_group_end;
    }
  }

  // Finite bound ranges cannot span the leaving violation. Leave the
  // transaction empty so the caller can require a checked row certificate.
  if (selected_end <= selected_begin) return true;
  transaction.bfrt_selected_group_size =
      static_cast<int>(selected_end - selected_begin);

  // Harris pass 1 uses the same numerically certified pivot domain as pass 2.
  // A raw tableau coefficient whose sign is inside its dot-product error bound
  // is not an eligible pivot and cannot define a trustworthy ratio bound.
  // Earlier finite-range candidates have already been flipped, so only the
  // remaining certified suffix constrains the admissible dual step. Use the
  // current signed reduced cost directly: replacing it by max(0, -d*r) would
  // discard already-consumed tolerance when d*r>0.
  const long double dual_tolerance = state.options->optimality_tol;
  long double harris_step_upper =
      std::numeric_limits<long double>::infinity();
  for (std::size_t i = selected_begin; i < candidates.size(); ++i) {
    const Candidate& candidate = candidates[i];
    const int direction =
        sign(state.move[static_cast<std::size_t>(candidate.col)]);
    const long double alpha = static_cast<long double>(candidate.alpha);
    const long double signed_reduced_cost = static_cast<long double>(direction) *
                                            state.reduced_costs[candidate.col];
    harris_step_upper = std::min(
        harris_step_upper,
        (dual_tolerance - signed_reduced_cost) / alpha);
  }

  // Harris pass 2: among every candidate feasible at the relaxed first-pass
  // step, take the largest signed pivot. This separates stability from the
  // exact minimum-ratio ordering without relaxing the terminal dual audit.
  const Candidate* selected = nullptr;
  const Candidate* released = nullptr;
  for (std::size_t i = selected_begin; i < candidates.size(); ++i) {
    const Candidate& candidate = candidates[i];
    if (candidate.breakpoint > harris_step_upper) continue;
    ++transaction.harris_second_pass_candidates;
    if (candidate.taboo) {
      ++transaction.taboo_rejections;
      if (released == nullptr ||
          candidate.taboo_expiry < released->taboo_expiry ||
          (candidate.taboo_expiry == released->taboo_expiry &&
           (candidate.alpha > released->alpha ||
            (candidate.alpha == released->alpha &&
             candidate.col < released->col)))) {
        released = &candidate;
      }
      continue;
    }
    if (selected == nullptr || candidate.alpha > selected->alpha ||
        (candidate.alpha == selected->alpha &&
         candidate.col < selected->col)) {
      selected = &candidate;
    }
  }
  if (selected == nullptr && released != nullptr) {
    transaction.all_harris_candidates_taboo = true;
    return true;
  }
  if (selected == nullptr) {
    failure = "Harris BFRT second pass has no entering column";
    return false;
  }
  transaction.entering.col = selected->col;
  transaction.entering.pivot =
      leaving.side * sign(state.move[static_cast<std::size_t>(selected->col)]) *
      selected->alpha;
  transaction.entering.alpha = selected->alpha;
  transaction.entering.theta = static_cast<double>(selected->breakpoint);
  const long double theta = selected->breakpoint;

  long double step_lower = -std::numeric_limits<long double>::infinity();
  long double step_upper = harris_step_upper;
  int step_upper_col = -1;
  long double step_upper_alpha = 0.0L;
  long double step_upper_signed_reduced_cost = 0.0L;
  struct TransactionMarks {
    std::vector<unsigned int> flipped;
    std::vector<unsigned int> shifted;
    unsigned int epoch{0};
  };
  static thread_local TransactionMarks marks;
  if (marks.flipped.size() != static_cast<std::size_t>(state.n)) {
    marks.flipped.assign(static_cast<std::size_t>(state.n), 0);
    marks.shifted.assign(static_cast<std::size_t>(state.n), 0);
    marks.epoch = 0;
  }
  if (++marks.epoch == 0) {
    std::fill(marks.flipped.begin(), marks.flipped.end(), 0);
    std::fill(marks.shifted.begin(), marks.shifted.end(), 0);
    ++marks.epoch;
  }
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const Candidate& candidate = candidates[i];
    const bool flip = i < selected_begin && std::isfinite(candidate.range);
    if (flip) {
      transaction.flips.push_back(
          {candidate.col,
           state.move[static_cast<std::size_t>(candidate.col)],
           candidate.range});
      marks.flipped[static_cast<std::size_t>(candidate.col)] = marks.epoch;
    }
  }

  auto is_flipped = [&](int col) {
    return marks.flipped[static_cast<std::size_t>(col)] == marks.epoch;
  };

  auto is_shifted = [&](int col) {
    return marks.shifted[static_cast<std::size_t>(col)] == marks.epoch;
  };

  // A coefficient whose sign is inside its dot-product error bound cannot be
  // used as a pivot, but a large accepted dual step can still carry its
  // reduced cost across zero. Boxed columns join the BFRT bound transaction;
  // one-sided columns receive a deterministic working-cost shift that the
  // mandatory original-cost cleanup later removes.
  //
  // The flip/shift determination and the dual-step interval accumulation
  // both traverse the pivotal-row support. The interval terms of a column
  // depend only on that column's own flip/shift marks, which are final once
  // its flip/shift determination has run, so the two traversals are fused
  // into one pass without changing any computed value or its order.
  for (int s = 0; s < scan_count; ++s) {
    const int col = scan_col(s);
    if (state.basic[static_cast<std::size_t>(col)]) continue;
    const int direction = sign(state.move[static_cast<std::size_t>(col)]);
    if (direction == 0) continue;
    const long double alpha =
        static_cast<long double>(leaving.side) * direction *
        scan_pivot(s);
    const long double signed_reduced_cost =
        static_cast<long double>(direction) * state.reduced_costs[col];
    if (col != selected->col && !is_flipped(col)) {
      const long double updated_signed_reduced_cost =
          signed_reduced_cost + alpha * theta;
      if (updated_signed_reduced_cost > dual_tolerance) {
        const double range = state.bounds.upper[col] - state.bounds.lower[col];
        if (std::isfinite(range) && range > 0.0) {
          transaction.flips.push_back(
              {col, state.move[static_cast<std::size_t>(col)], range});
          marks.flipped[static_cast<std::size_t>(col)] = marks.epoch;
        } else {
          const double updated_reduced_cost =
              state.reduced_costs[col] +
              leaving.side * static_cast<double>(theta) *
                  scan_pivot(s);
          if (!std::isfinite(updated_reduced_cost)) {
            failure = "Harris BFRT requires a non-finite working-cost shift";
            return false;
          }
          transaction.cost_shifts.push_back({col, -updated_reduced_cost});
          marks.shifted[static_cast<std::size_t>(col)] = marks.epoch;
        }
      }
    }
    if (is_shifted(col)) continue;
    if (!(alpha > 0.0L)) continue;
    if (is_flipped(col)) {
      step_lower =
          std::max(step_lower,
                   (-dual_tolerance - signed_reduced_cost) / alpha);
    } else {
      const long double column_step_upper =
          (dual_tolerance - signed_reduced_cost) / alpha;
      if (column_step_upper < step_upper) {
        step_upper = column_step_upper;
        step_upper_col = col;
        step_upper_alpha = alpha;
        step_upper_signed_reduced_cost = signed_reduced_cost;
      }
    }
  }
  if (theta < step_lower || theta > step_upper) {
    std::ostringstream message;
    message << std::setprecision(18)
            << "exact-breakpoint BFRT violates its dual-feasibility interval"
            << " (theta=" << theta << ", lower=" << step_lower
            << ", upper=" << step_upper
            << ", entering_col=" << selected->col
            << ", entering_alpha=" << selected->alpha
            << ", limiting_col=" << step_upper_col
            << ", limiting_alpha=" << step_upper_alpha
            << ", limiting_signed_rc=" << step_upper_signed_reduced_cost;
    if (step_upper_col >= 0) {
      message << ", limiting_dot_error="
              << dot_error_bound(state, leaving.row_ep, step_upper_col)
              << ", limiting_lower=" << state.bounds.lower[step_upper_col]
              << ", limiting_upper=" << state.bounds.upper[step_upper_col];
    }
    message << ')';
    failure = message.str();
    return false;
  }
  transaction.dual_step_lower = static_cast<double>(step_lower);
  transaction.dual_step_upper = static_cast<double>(step_upper);

  static thread_local std::vector<std::pair<int, double>> rhs_terms;
  rhs_terms.clear();
  for (const BoundFlip& flip : transaction.flips) {
    const double delta = sign(flip.old_move) * flip.range;
    for (StandardColumnMatrix::InnerIterator it(state.sf->A, flip.col);
         it; ++it) {
      rhs_terms.emplace_back(it.row(), it.value() * delta);
    }
  }
  std::sort(rhs_terms.begin(), rhs_terms.end(),
            [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  for (std::size_t begin = 0; begin < rhs_terms.size();) {
    const int row = rhs_terms[begin].first;
    double value = 0.0;
    std::size_t end = begin;
    while (end < rhs_terms.size() && rhs_terms[end].first == row) {
      value += rhs_terms[end].second;
      ++end;
    }
    if (value != 0.0) {
      transaction.bfrt_rhs.index.push_back(row);
      transaction.bfrt_rhs.value.push_back(value);
    }
    begin = end;
  }
  long double rhs_projection = 0.0L;
  long double rhs_projection_abs = 0.0L;
  for (std::size_t k = 0; k < transaction.bfrt_rhs.index.size(); ++k) {
    const int row = transaction.bfrt_rhs.index[k];
    const long double term =
        static_cast<long double>(leaving.row_ep.at(row)) *
        static_cast<long double>(transaction.bfrt_rhs.value[k]);
    rhs_projection += term;
    rhs_projection_abs += std::abs(term);
  }
  const long double materialized_coverage =
      static_cast<long double>(leaving.side) * rhs_projection;
  // The projection and the stored violation are computed by different
  // summation orders, so a flip set that exactly covers the violation can
  // land a few ulps outside [0, violation] (observed on 118-bus SCUC:
  // coverage exceeding violation by ~5e-16 relative turned a legal
  // exact-cover pivot into a terminal failure). Accept within the
  // projection's own rounding envelope, then clamp so the downstream
  // entering step (violation - coverage) stays exactly >= 0; the
  // exact-cover clamp lands on the established covered==violation path.
  const long double violation_ld =
      static_cast<long double>(leaving.violation);
  const long double coverage_slack =
      (static_cast<long double>(transaction.bfrt_rhs.index.size()) + 256.0L) *
      static_cast<long double>(std::numeric_limits<double>::epsilon()) *
      std::max({rhs_projection_abs, std::abs(violation_ld), 1.0L});
  if (!std::isfinite(materialized_coverage) ||
      !(materialized_coverage >= -coverage_slack) ||
      !(materialized_coverage <= violation_ld + coverage_slack)) {
    std::ostringstream message;
    message << std::setprecision(18)
            << "materialized BFRT flips do not leave a valid entering step"
            << " (coverage=" << materialized_coverage
            << ", violation=" << leaving.violation
            << ", slack=" << coverage_slack << ')';
    failure = message.str();
    return false;
  }
  transaction.covered_violation = static_cast<double>(
      std::min(std::max(materialized_coverage, 0.0L), violation_ld));
  return true;
}

bool compute_dse_weights(const State& state, const Leaving& leaving,
                         const IndexedVector& pivot_row,
                         const IndexedVector& direction, double pivot,
                         EdgeWeightUpdate& update,
                         bool& restart_devex, std::string& failure) {
  restart_devex = false;
  update.nonpivotal_value.clear();
  update.pivotal_value = 0.0;
  if (direction.dimension != state.m || !direction.finite() || pivot == 0.0) {
    failure = "edge-weight update received an invalid pivotal column";
    return false;
  }
  if (state.edge_weight_mode == EdgeWeightMode::Devex) {
    if (pivot_row.dimension != state.n || !pivot_row.finite() ||
        state.devex_reference.size() != static_cast<std::size_t>(state.n)) {
      failure = "Devex update received an invalid reference row";
      return false;
    }
    long double exact_weight = 0.0L;
    for (std::size_t k = 0; k < pivot_row.index.size(); ++k) {
      const int col = pivot_row.index[k];
      if (!state.devex_reference[static_cast<std::size_t>(col)]) continue;
      const long double value = static_cast<long double>(pivot_row.value[k]);
      exact_weight += value * value;
    }
    exact_weight = std::max(1.0L, exact_weight);
    if (!std::isfinite(exact_weight)) {
      failure = "Devex pivotal reference weight is non-finite";
      return false;
    }
    const double computed = static_cast<double>(exact_weight);
    const double stored =
        state.edge_weight[static_cast<std::size_t>(leaving.row)];
    if (!(stored > 0.0) || !std::isfinite(stored)) {
      failure = "Devex stored pivotal weight is invalid";
      return false;
    }
    const double ratio = std::max(stored / computed, computed / stored);
    restart_devex = ratio > 9.0;

    const double new_pivotal =
        std::max(1.0, computed / (pivot * pivot));
    if (!std::isfinite(new_pivotal)) {
      failure = "Devex new pivotal weight is non-finite";
      return false;
    }
    const int update_count = static_cast<int>(direction.index.size());
    update.nonpivotal_value.reserve(static_cast<std::size_t>(update_count));
    for (int position = 0; position < update_count; ++position) {
      const int row = direction.index[static_cast<std::size_t>(position)];
      if (row < 0 || row >= state.m) {
        failure = "Devex direction pattern contains an invalid row";
        return false;
      }
      if (row == leaving.row) continue;
      const double candidate =
          new_pivotal * direction.value[static_cast<std::size_t>(position)] *
          direction.value[static_cast<std::size_t>(position)];
      if (!std::isfinite(candidate)) {
        failure = "Devex recurrence produced a non-finite weight";
        return false;
      }
      update.nonpivotal_value.push_back(
          std::max(state.edge_weight[static_cast<std::size_t>(row)],
                   candidate));
    }
    update.pivotal_value = new_pivotal;
    return true;
  }

  // HFactor already holds the FTRAN result in a dense HVector. The recurrence
  // only reads rho on the captured pivotal-column support, so extract exactly
  // those coordinates in direction order instead of exporting packed rho,
  // clearing O(m) dense scratch, and scattering rho back into it.
  static thread_local std::vector<double> rho_on_direction;
  if (!state.factor->indexed_ftran_at_captured_pattern(
          leaving.row_ep, rho_on_direction)) {
    failure = "DSE packed FTRAN failed";
    return false;
  }
  if (rho_on_direction.size() != direction.value.size()) {
    failure = "DSE auxiliary FTRAN pattern disagrees with pivotal column";
    return false;
  }
  const double old_pivotal_weight =
      state.edge_weight[static_cast<std::size_t>(leaving.row)];
  const double inv_pivot = 1.0 / pivot;
  const int update_count = static_cast<int>(direction.index.size());
  update.nonpivotal_value.reserve(static_cast<std::size_t>(update_count));
  for (int position = 0; position < update_count; ++position) {
    const int row = direction.index[static_cast<std::size_t>(position)];
    if (row < 0 || row >= state.m) {
      failure = "DSE direction pattern contains an invalid row";
      return false;
    }
    if (row == leaving.row) continue;
    const double ratio =
        direction.value[static_cast<std::size_t>(position)] * inv_pivot;
    const double rho_value =
        rho_on_direction[static_cast<std::size_t>(position)];
    const long double ratio_ld = static_cast<long double>(ratio);
    const long double value_ld =
        static_cast<long double>(
            state.edge_weight[static_cast<std::size_t>(row)]) -
        2.0L * ratio_ld * static_cast<long double>(rho_value) +
        ratio_ld * ratio_ld *
            static_cast<long double>(old_pivotal_weight);
    const double value = static_cast<double>(value_ld);
    const double roundoff =
        1024.0 * std::numeric_limits<double>::epsilon() *
        std::max({1.0,
                  std::abs(state.edge_weight[static_cast<std::size_t>(row)]),
                  std::abs(2.0 * ratio * rho_value),
                  std::abs(ratio * ratio * old_pivotal_weight)});
    if (!std::isfinite(value)) {
      failure = "Goldfarb-Reid update produced a non-finite DSE weight";
      return false;
    }
    update.nonpivotal_value.push_back(std::max(value, roundoff));
  }
  const double new_pivotal_weight =
      old_pivotal_weight * inv_pivot * inv_pivot;
  if (!(new_pivotal_weight > 0.0) ||
      !std::isfinite(new_pivotal_weight)) {
    failure = "pivotal DSE weight is invalid";
    return false;
  }
  update.pivotal_value = new_pivotal_weight;
  return true;
}

}  // namespace mipsolvers::engine::native_dual::detail
