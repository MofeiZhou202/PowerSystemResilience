#include "state.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <thread>
#include <utility>

namespace mipsolvers::engine::native_dual::detail {
namespace {

std::vector<char> artificial_mask(const StandardFormLP& sf) {
  std::vector<char> mask(static_cast<std::size_t>(sf.A.cols()), 0);
  for (int col : sf.row_to_artificial_col) {
    if (col >= 0 && col < sf.A.cols()) mask[static_cast<std::size_t>(col)] = 1;
  }
  return mask;
}

double deterministic_fraction(int column) {
  std::uint64_t value = static_cast<std::uint64_t>(column) +
                        0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  value ^= value >> 31;
  return static_cast<double>(value >> 11) * 0x1.0p-53;
}

std::uint64_t cycle_mix(std::uint64_t value) {
  value += 0x9e3779b97f4a7c15ULL;
  value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
  value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
  return value ^ (value >> 31);
}

int taboo_lifetime(int rows) noexcept {
  const int bounded_rows = std::max(1, rows);
  if (bounded_rows > (kMaxTabooLifetime - 1) / 2) {
    return kMaxTabooLifetime;
  }
  return 2 * bounded_rows + 1;
}

int lp_kernel_threads(const SimplexOptions& options) {
  int requested = options.lp_kernel_threads;
  if (const char* environment =
          std::getenv("MIPSOLVERS_LP_KERNEL_THREADS")) {
    char* end = nullptr;
    const long parsed = std::strtol(environment, &end, 10);
    if (end != environment && *end == '\0' && parsed >= 0 && parsed <= 256) {
      requested = static_cast<int>(parsed);
    }
  }
  if (requested == 1) return 1;
  const unsigned int hardware = std::thread::hardware_concurrency();
  const int available = hardware == 0 ? 1 : static_cast<int>(hardware);
  if (requested <= 0) return std::max(1, std::min(4, available));
  return std::max(1, std::min(requested, available));
}

template <typename Map, typename Queue>
void enforce_taboo_capacity(Map& taboo, Queue& fifo) {
  // Released entries leave ticketed FIFO records behind. Compact
  // periodically so repeated erase/reinsert churn cannot grow the queue while
  // the live hash table itself remains small.
  if (fifo.size() > kTabooCapacity) {
    Queue compacted;
    for (const auto& queued : fifo) {
      const auto current = taboo.find(queued.first);
      if (current != taboo.end() &&
          current->second.ticket == queued.second) {
        compacted.push_back(queued);
      }
    }
    fifo.swap(compacted);
  }

  while (taboo.size() > kTabooCapacity && !fifo.empty()) {
    const auto [old_key, ticket] = fifo.front();
    fifo.pop_front();
    const auto old = taboo.find(old_key);
    if (old != taboo.end() && old->second.ticket == ticket) {
      taboo.erase(old);
    }
  }
  while (taboo.size() > kTabooCapacity) taboo.erase(taboo.begin());
}

// The cycle signature is a Zobrist-style set hash of (basis assignment,
// nonbasic bound sides): both accumulators XOR independently mixed per-element
// tokens, so identical states always produce identical signatures and any
// single-element change is an O(1) XOR delta.  This makes the signature
// incrementally maintainable across a pivot (which touches ~3 elements)
// instead of recomputed over all m+n elements — the full recomputation was
// measured at ~11% of total solve time on the IEEE-118 root LP.
constexpr std::uint64_t kCycleBasisSaltA = 0xa4093822299f31d0ULL;
constexpr std::uint64_t kCycleBasisSaltB = 0x082efa98ec4e6c89ULL;
constexpr std::uint64_t kCycleMoveSaltA = 0x452821e638d01377ULL;
constexpr std::uint64_t kCycleMoveSaltB = 0xbe5466cf34e90c6cULL;

std::uint64_t cycle_basis_token(int row, int col) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32) ^
         static_cast<std::uint32_t>(col);
}

std::uint64_t cycle_move_token(int col, int move_sign) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(col)) << 2) ^
         static_cast<std::uint64_t>(move_sign + 1);
}

std::pair<std::uint64_t, std::uint64_t> compute_cycle_signature(
    const State& state) {
  std::uint64_t a = 0x243f6a8885a308d3ULL;
  std::uint64_t b = 0x13198a2e03707344ULL;
  for (int row = 0; row < static_cast<int>(state.basis.size()); ++row) {
    const std::uint64_t token = cycle_basis_token(
        row, state.basis[static_cast<std::size_t>(row)]);
    a ^= cycle_mix(token ^ kCycleBasisSaltA);
    b ^= cycle_mix(token ^ kCycleBasisSaltB);
  }
  for (int col = 0; col < static_cast<int>(state.move.size()); ++col) {
    if (!state.basic.empty() && state.basic[static_cast<std::size_t>(col)]) {
      continue;
    }
    const std::uint64_t token = cycle_move_token(
        col, sign(state.move[static_cast<std::size_t>(col)]));
    a ^= cycle_mix(token ^ kCycleMoveSaltA);
    b ^= cycle_mix(token ^ kCycleMoveSaltB);
  }
  return {a, b};
}

}  // namespace

void resync_cycle_signature(State& state) {
  const auto [a, b] = compute_cycle_signature(state);
  state.cycle_signature_live_a = a;
  state.cycle_signature_live_b = b;
}

void cycle_signature_apply_basis_swap(State& state, int row, int old_col,
                                      int new_col) {
  const std::uint64_t old_token = cycle_basis_token(row, old_col);
  const std::uint64_t new_token = cycle_basis_token(row, new_col);
  state.cycle_signature_live_a ^= cycle_mix(old_token ^ kCycleBasisSaltA) ^
                                  cycle_mix(new_token ^ kCycleBasisSaltA);
  state.cycle_signature_live_b ^= cycle_mix(old_token ^ kCycleBasisSaltB) ^
                                  cycle_mix(new_token ^ kCycleBasisSaltB);
}

void cycle_signature_apply_move_toggle(State& state, int col, int move_sign) {
  const std::uint64_t token = cycle_move_token(col, move_sign);
  state.cycle_signature_live_a ^= cycle_mix(token ^ kCycleMoveSaltA);
  state.cycle_signature_live_b ^= cycle_mix(token ^ kCycleMoveSaltB);
}

Eigen::VectorXd multiply_A(const StandardColumnMatrix& A,
                           const Eigen::VectorXd& x) {
  if (x.size() != A.cols()) return {};
  Eigen::VectorXd result = Eigen::VectorXd::Zero(A.rows());
  for (int col = 0; col < A.cols(); ++col) {
    const double value = x[col];
    if (value == 0.0) continue;
    for (StandardColumnMatrix::InnerIterator it(A, col); it; ++it) {
      result[it.row()] += it.value() * value;
    }
  }
  return result;
}

Eigen::VectorXd equation_residual_vector(
    const StandardColumnMatrix& A, const Eigen::VectorXd& x,
    const Eigen::VectorXd& rhs, bool exact) {
  if (x.size() != A.cols() || rhs.size() != A.rows() || !x.allFinite() ||
      !rhs.allFinite()) {
    return {};
  }
  if (!exact) {
    Eigen::VectorXd residual = rhs;
    for (int col = 0; col < A.cols(); ++col) {
      const double value = x[col];
      if (value == 0.0) continue;
      for (StandardColumnMatrix::InnerIterator it(A, col); it; ++it) {
        residual[it.row()] -= it.value() * value;
      }
    }
    return residual;
  }
  std::vector<long double> residual(static_cast<std::size_t>(A.rows()));
  for (int row = 0; row < A.rows(); ++row) {
    residual[static_cast<std::size_t>(row)] =
        static_cast<long double>(rhs[row]);
  }
  for (int col = 0; col < A.cols(); ++col) {
    const long double value = static_cast<long double>(x[col]);
    if (value == 0.0L) continue;
    for (StandardColumnMatrix::InnerIterator it(A, col); it; ++it) {
      residual[static_cast<std::size_t>(it.row())] -=
          static_cast<long double>(it.value()) * value;
    }
  }
  Eigen::VectorXd result(A.rows());
  for (int row = 0; row < A.rows(); ++row) {
    result[row] = static_cast<double>(residual[static_cast<std::size_t>(row)]);
  }
  return result;
}

Eigen::VectorXd multiply_AT(const StandardColumnMatrix& A,
                            const Eigen::VectorXd& y) {
  if (y.size() != A.rows()) return {};
  Eigen::VectorXd result = Eigen::VectorXd::Zero(A.cols());
  for (int col = 0; col < A.cols(); ++col) {
    double value = 0.0;
    for (StandardColumnMatrix::InnerIterator it(A, col); it; ++it) {
      value += it.value() * y[it.row()];
    }
    result[col] = value;
  }
  return result;
}

static void reconstruct_reduced_costs(State& state, const Eigen::VectorXd& y) {
  state.reduced_costs.resize(state.n);
  for (int col = 0; col < state.n; ++col) {
    double dual_product = 0.0;
    for (StandardColumnMatrix::InnerIterator it(state.sf->A, col); it; ++it) {
      dual_product += it.value() * y[it.row()];
    }
    state.reduced_costs[col] = state.cost[col] - dual_product;
  }
}

static double reconstruct_objective(const State& state) {
  double objective = state.cost.dot(state.bounds.lower);
  for (int col = 0; col < state.n; ++col) {
    if (!state.basic[static_cast<std::size_t>(col)] &&
        state.move[static_cast<std::size_t>(col)] == Move::Down) {
      objective += state.cost[col] *
                   (state.bounds.upper[col] - state.bounds.lower[col]);
    }
  }
  for (int row = 0; row < state.m; ++row) {
    const int col = state.basis[static_cast<std::size_t>(row)];
    objective +=
        state.cost[col] * (state.x_basic[row] - state.bounds.lower[col]);
  }
  return objective;
}

double equation_residual_inf(const StandardColumnMatrix& A,
                             const Eigen::VectorXd& x,
                             const Eigen::VectorXd& rhs) {
  const Eigen::VectorXd residual = equation_residual_vector(A, x, rhs, true);
  if (residual.size() != rhs.size() || !residual.allFinite()) {
    return std::numeric_limits<double>::infinity();
  }
  return residual.lpNorm<Eigen::Infinity>();
}

Bounds make_phase_two_bounds(const StandardFormLP& sf) {
  const int n = static_cast<int>(sf.A.cols());
  Bounds bounds;
  bounds.lower = Eigen::VectorXd::Zero(n);
  bounds.upper = Eigen::VectorXd::Zero(n);
  bounds.enterable.assign(static_cast<std::size_t>(n), 0);
  const std::vector<char> artificial = artificial_mask(sf);
  for (int j = 0; j < n; ++j) {
    if (artificial[static_cast<std::size_t>(j)]) continue;
    const double original_upper = sf.var_ub[j];
    if (!(original_upper > 0.0)) continue;
    bounds.upper[j] = original_upper;
    bounds.enterable[static_cast<std::size_t>(j)] = 1;
  }
  return bounds;
}

Eigen::VectorXd make_dual_phase_one_anchor(const StandardFormLP& sf) {
  Eigen::VectorXd anchor = Eigen::VectorXd::Zero(sf.A.cols());
  const std::vector<int> logical = logical_columns(sf);
  if (static_cast<int>(logical.size()) != sf.A.rows() ||
      sf.b.size() != sf.A.rows()) {
    return {};
  }
  for (int row = 0; row < sf.A.rows(); ++row) {
    const int col = logical[static_cast<std::size_t>(row)];
    if (col < 0 || col >= sf.A.cols()) return {};
    double pivot = 0.0;
    int nonzeros = 0;
    for (StandardColumnMatrix::InnerIterator it(sf.A, col); it; ++it) {
      if (it.value() == 0.0) continue;
      ++nonzeros;
      if (it.row() == row) pivot = it.value();
    }
    if (nonzeros != 1 || pivot == 0.0 || !std::isfinite(pivot)) return {};
    anchor[col] = sf.b[row] / pivot;
  }
  return anchor;
}

Bounds make_dual_phase_one_bounds(const StandardFormLP& sf,
                                  const Eigen::VectorXd& anchor) {
  const int n = static_cast<int>(sf.A.cols());
  Bounds bounds;
  bounds.lower = anchor;
  bounds.upper = anchor;
  bounds.enterable.assign(static_cast<std::size_t>(n), 0);
  if (anchor.size() != n) return bounds;

  const Bounds phase_two = make_phase_two_bounds(sf);
  for (int j = 0; j < n; ++j) {
    // In homogeneous coordinates z=x-x0, every Native column has a finite
    // lower bound. HiGHS' dual Phase-I map is therefore [0,1] for a
    // lower-only column and [0,0] for a boxed/fixed column. Translate those
    // bounds back to x coordinates before giving them to the existing kernel.
    if (phase_two.enterable[static_cast<std::size_t>(j)] &&
        !std::isfinite(phase_two.upper[j])) {
      bounds.upper[j] = anchor[j] + 1.0;
      bounds.enterable[static_cast<std::size_t>(j)] = 1;
    }
  }
  return bounds;
}

Bounds make_primal_phase_one_bounds(const StandardFormLP& sf) {
  Bounds bounds = make_phase_two_bounds(sf);
  for (int col : sf.row_to_artificial_col) {
    if (col < 0 || col >= sf.A.cols()) continue;
    bounds.lower[col] = 0.0;
    bounds.upper[col] = std::numeric_limits<double>::infinity();
    bounds.enterable[static_cast<std::size_t>(col)] = 1;
  }
  return bounds;
}

Eigen::VectorXd make_primal_phase_one_cost(const StandardFormLP& sf) {
  Eigen::VectorXd cost = Eigen::VectorXd::Zero(sf.A.cols());
  for (int col : sf.row_to_artificial_col) {
    if (col >= 0 && col < sf.A.cols()) cost[col] = -1.0;
  }
  return cost;
}

bool initialize_dual_phase_one(State& state, std::string& failure) {
  state.phase = Phase::DualOne;
  state.cost = state.sf->c_max;
  state.original_cost = state.sf->c_max;
  state.cost_perturbation.reset(state.n);
  state.cost_shift.reset(state.n);
  state.costs_perturbed = false;
  state.costs_shifted = false;
  state.perturbation_disabled = false;
  state.dual_phase_one_anchor = make_dual_phase_one_anchor(*state.sf);
  if (state.dual_phase_one_anchor.size() != state.n ||
      !state.dual_phase_one_anchor.allFinite()) {
    failure = "dual Phase-I anchor is invalid";
    return false;
  }
  const double anchor_residual = equation_residual_inf(
      state.sf->A, state.dual_phase_one_anchor, state.sf->b);
  const double residual_limit =
      state.options->feasibility_tol *
      std::max(1.0, state.sf->b.lpNorm<Eigen::Infinity>());
  if (!(anchor_residual <= residual_limit)) {
    failure = "dual Phase-I anchor does not satisfy Ax0=b";
    return false;
  }

  state.bounds =
      make_dual_phase_one_bounds(*state.sf, state.dual_phase_one_anchor);
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !state.bounds.enterable[static_cast<std::size_t>(j)]) {
      state.move[static_cast<std::size_t>(j)] = Move::Fixed;
      continue;
    }
    if (!(state.bounds.upper[j] > state.bounds.lower[j])) {
      failure = "dual Phase-I unit interval is not representable";
      return false;
    }
    state.move[static_cast<std::size_t>(j)] =
        state.reduced_costs[j] > 0.0 ? Move::Down : Move::Up;
  }
  if (!reconstruct(state, failure)) {
    failure = "dual Phase-I reconstruction failed: " + failure;
    return false;
  }
  const Audit initial = audit(state, false, true, false);
  if (!initial.ok) {
    failure = "dual Phase-I initial invariant failed: " + initial.failure;
    return false;
  }
  return true;
}

double dual_phase_one_objective(const State& state) {
  if (state.phase != Phase::DualOne ||
      state.dual_phase_one_anchor.size() != state.n) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  long double objective = 0.0L;
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)]) continue;
    const double value = state.move[static_cast<std::size_t>(j)] == Move::Down
                             ? state.bounds.upper[j]
                             : state.bounds.lower[j];
    objective += static_cast<long double>(value - state.dual_phase_one_anchor[j]) *
                 static_cast<long double>(state.reduced_costs[j]);
  }
  return static_cast<double>(objective);
}

DualInfeasibilitySummary original_dual_infeasibility_summary(
    const State& state) {
  DualInfeasibilitySummary summary;
  const Bounds original = make_phase_two_bounds(*state.sf);
  const double tolerance = state.options->optimality_tol;
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !original.enterable[static_cast<std::size_t>(j)] ||
        std::isfinite(original.upper[j])) {
      continue;
    }
    const double infeasibility = std::max(0.0, state.reduced_costs[j]);
    summary.sum += infeasibility;
    summary.max = std::max(summary.max, infeasibility);
    if (infeasibility > tolerance) ++summary.count;
  }
  return summary;
}

bool transition_dual_phase_one_to_two(State& state,
                                      DualInfeasibilitySummary& summary,
                                      std::string& failure) {
  if (state.phase != Phase::DualOne) {
    failure = "dual Phase-I transition requested from the wrong phase";
    return false;
  }
  summary = original_dual_infeasibility_summary(state);
  if (summary.count != 0) {
    failure = "dual Phase I ended with original-bound dual infeasibilities";
    return false;
  }

  state.phase = Phase::Two;
  state.bounds = make_phase_two_bounds(*state.sf);
  state.dual_phase_one_anchor.resize(0);
  if (!normalize_nonbasic_moves(state, failure)) {
    failure = "dual Phase-I transition reconstruction failed: " + failure;
    return false;
  }
  const Audit transitioned = audit(state, false, true, false);
  if (!transitioned.ok) {
    failure = "dual Phase-I transition invariant failed: " +
              transitioned.failure;
    return false;
  }
  return true;
}

bool build_logical_basis(const StandardFormLP& sf, std::vector<int>& basis,
                         std::string& failure) {
  basis = logical_columns(sf);
  if (static_cast<int>(basis.size()) != sf.A.rows()) {
    failure = "logical basis dimension mismatch";
    return false;
  }
  std::vector<char> seen(static_cast<std::size_t>(sf.A.cols()), 0);
  for (int col : basis) {
    if (col < 0 || col >= sf.A.cols()) {
      failure = "a row has no slack or artificial logical column";
      return false;
    }
    if (seen[static_cast<std::size_t>(col)]) {
      failure = "logical basis contains a duplicate column";
      return false;
    }
    seen[static_cast<std::size_t>(col)] = 1;
  }
  return true;
}

int apply_certified_singleton_crash(const StandardFormLP& sf,
                                    std::vector<int>& basis) {
  if (static_cast<int>(basis.size()) != sf.A.rows()) return 0;
  std::vector<int> candidate(static_cast<std::size_t>(sf.A.rows()), -1);
  for (int col = 0; col < sf.n_original && col < sf.A.cols(); ++col) {
    int row = -1;
    double coefficient = 0.0;
    int count = 0;
    for (StandardColumnMatrix::InnerIterator it(sf.A, col); it; ++it) {
      if (it.value() == 0.0) continue;
      row = it.row();
      coefficient = it.value();
      ++count;
      if (count > 1) break;
    }
    if (count != 1 || row < 0 || coefficient == 0.0) continue;
    const double value = sf.b[row] / coefficient;
    const double upper = sf.var_ub[col];
    if (value < 0.0 || (std::isfinite(upper) && value > upper)) continue;
    int& selected = candidate[static_cast<std::size_t>(row)];
    if (selected < 0 || col < selected) selected = col;
  }

  int replacements = 0;
  for (int row = 0; row < sf.A.rows(); ++row) {
    const int col = candidate[static_cast<std::size_t>(row)];
    if (col < 0) continue;
    basis[static_cast<std::size_t>(row)] = col;
    ++replacements;
  }
  return replacements;
}

bool rebuild_membership(State& state, std::string& failure) {
  state.basic.assign(static_cast<std::size_t>(state.n), 0);
  if (static_cast<int>(state.basis.size()) != state.m) {
    failure = "basis cardinality differs from row count";
    return false;
  }
  for (int col : state.basis) {
    if (col < 0 || col >= state.n) {
      failure = "basis column is out of range";
      return false;
    }
    if (state.basic[static_cast<std::size_t>(col)]) {
      failure = "basis column is duplicated";
      return false;
    }
    state.basic[static_cast<std::size_t>(col)] = 1;
  }
  if (static_cast<int>(state.move.size()) != state.n) {
    state.move.assign(static_cast<std::size_t>(state.n), Move::Up);
  }
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !state.bounds.enterable[static_cast<std::size_t>(j)]) {
      state.move[static_cast<std::size_t>(j)] = Move::Fixed;
    } else if (state.move[static_cast<std::size_t>(j)] == Move::Fixed) {
      state.move[static_cast<std::size_t>(j)] = Move::Up;
    }
  }
  return true;
}

bool correct_canonical_primal_residual(State& state, std::string& failure,
                                       bool exact_residual) {
  state.leaving_heap_valid = false;
  const double equation_limit =
      state.options->feasibility_tol *
      std::max(1.0, state.sf->b.lpNorm<Eigen::Infinity>());
  Eigen::VectorXd x = full_primal(state);
  Eigen::VectorXd residual =
      equation_residual_vector(state.sf->A, x, state.sf->b, exact_residual);
  if (residual.size() != state.m || !residual.allFinite()) {
    failure = "canonical primal residual is dimensionally invalid or non-finite";
    return false;
  }
  double residual_norm = residual.lpNorm<Eigen::Infinity>();
  if (residual_norm > equation_limit) {
    // Backward stability of Bx_B=rhs is necessary but can be weaker than the
    // simplex feasibility contract when ||B||*||x_B|| is large.  One classical
    // defect-correction step targets that contract directly without changing
    // its tolerance or rebuilding the factor.
    const SolveEvidence correction = state.factor->checked_ftran(residual);
    if (!correction.accepted || correction.solution.size() != state.m ||
        !correction.solution.allFinite()) {
      std::ostringstream message;
      message << std::setprecision(17)
              << "canonical primal residual correction FTRAN failed"
              << " (residual_inf=" << residual_norm
              << ", limit=" << equation_limit << "): "
              << state.factor->last_solve_diagnostics();
      failure = message.str();
      return false;
    }
    state.x_basic += correction.solution;
    ++state.canonical_primal_corrections;
    x = full_primal(state);
    residual = equation_residual_vector(state.sf->A, x, state.sf->b,
                                        exact_residual);
    if (residual.size() != state.m || !residual.allFinite()) {
      failure = "corrected canonical primal residual is invalid or non-finite";
      return false;
    }
    const double corrected_norm = residual.lpNorm<Eigen::Infinity>();
    if (corrected_norm > equation_limit) {
      std::ostringstream message;
      message << std::setprecision(17)
              << "one-step canonical primal residual correction did not satisfy "
                 "the fixed feasibility limit"
              << " (before=" << residual_norm << ", after=" << corrected_norm
              << ", limit=" << equation_limit << ')';
      failure = message.str();
      return false;
    }
  }
  return true;
}

bool reconstruct(State& state, std::string& failure, bool primal, bool dual,
                 bool exact_residual) {
  state.leaving_heap_valid = false;
  if (!rebuild_membership(state, failure)) return false;
  if (primal) {
    const bool anchored_dual_phase_one =
        state.phase == Phase::DualOne &&
        state.dual_phase_one_anchor.size() == state.n;
    Eigen::VectorXd rhs = anchored_dual_phase_one
                              ? Eigen::VectorXd::Zero(state.m)
                              : state.sf->b;
    for (int j = 0; j < state.n; ++j) {
      if (state.basic[static_cast<std::size_t>(j)]) continue;
      double value = state.bounds.lower[j];
      if (state.move[static_cast<std::size_t>(j)] == Move::Down) {
        value = state.bounds.upper[j];
        if (!std::isfinite(value)) {
          failure = "nonbasic variable is at an infinite upper bound";
          return false;
        }
      }
      if (anchored_dual_phase_one) {
        value -= state.dual_phase_one_anchor[j];
      }
      if (value == 0.0) continue;
      for (StandardColumnMatrix::InnerIterator it(state.sf->A, j); it;
           ++it) {
        rhs[it.row()] -= it.value() * value;
      }
    }
    const SolveEvidence primal_solve = state.factor->checked_ftran(rhs);
    state.x_basic = primal_solve.solution;
    if (!primal_solve.accepted || state.x_basic.size() != state.m ||
        !state.x_basic.allFinite()) {
      failure = "FTRAN failed backward-error validation: " +
                state.factor->last_solve_diagnostics();
      return false;
    }
    if (anchored_dual_phase_one) {
      for (int row = 0; row < state.m; ++row) {
        state.x_basic[row] += state.dual_phase_one_anchor[
            state.basis[static_cast<std::size_t>(row)]];
      }
    }
    const int rebuild_count = state.factor->rebuild_count();
    const int interval = state.options->intermediate_audit_interval;
    const bool periodic_audit =
        rebuild_count <= 1 ||
        (interval > 0 && rebuild_count % interval == 0);
    const bool already_audited =
        !exact_residual &&
        state.last_canonical_residual_audit_rebuild == rebuild_count;
    if ((exact_residual || periodic_audit) && !already_audited) {
      if (!correct_canonical_primal_residual(state, failure, exact_residual))
        return false;
      ++state.canonical_residual_audits;
      state.last_canonical_residual_audit_rebuild = rebuild_count;
    }
  }

  if (dual) {
    Eigen::VectorXd c_basic(state.m);
    for (int row = 0; row < state.m; ++row) {
      c_basic[row] =
          state.cost[state.basis[static_cast<std::size_t>(row)]];
    }
    const Eigen::VectorXd y = state.factor->btran(c_basic);
    if (y.size() != state.m || !y.allFinite()) {
      failure = "BTRAN failed backward-error validation";
      return false;
    }
    reconstruct_reduced_costs(state, y);
    if (!state.reduced_costs.allFinite()) {
      failure = "reduced-cost reconstruction is non-finite";
      return false;
    }
    for (int col : state.basis) state.reduced_costs[col] = 0.0;
  }

  state.objective = reconstruct_objective(state);
  if (!std::isfinite(state.objective)) {
    failure = "objective reconstruction is non-finite";
    return false;
  }
  return true;
}

bool normalize_nonbasic_moves(State& state, std::string& failure) {
  const double tolerance = state.options->optimality_tol;
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !state.bounds.enterable[static_cast<std::size_t>(j)]) {
      state.move[static_cast<std::size_t>(j)] = Move::Fixed;
      continue;
    }
    if (state.reduced_costs[j] > tolerance) {
      if (!std::isfinite(state.bounds.upper[j])) {
        failure = "lower-only nonbasic column " + std::to_string(j) +
                  " has positive reduced cost";
        return false;
      }
      state.move[static_cast<std::size_t>(j)] = Move::Down;
    } else {
      state.move[static_cast<std::size_t>(j)] = Move::Up;
    }
  }
  return reconstruct(state, failure);
}

DualShiftStartDecision decide_cost_shifted_dual_start(
    const State& state, const char* environment_policy) {
  if (environment_policy != nullptr) {
    const std::string policy(environment_policy);
    if (policy == "on") return {true, 0, 0, false};
    if (policy == "off") return {false, 0, 0, false};
  }

  const int count_threshold =
      std::max(1, state.options->dual_shift_start_max_count);
  const double configured_fraction =
      state.options->dual_shift_start_max_fraction;
  const double fraction =
      std::isfinite(configured_fraction) && configured_fraction > 0.0
          ? configured_fraction
          : 0.0;
  const long double scaled_threshold = std::ceil(
      static_cast<long double>(fraction) * std::max(0, state.n));
  constexpr int kIntMax = (std::numeric_limits<int>::max)();
  const int fraction_threshold =
      scaled_threshold >= static_cast<long double>(kIntMax)
          ? kIntMax
          : static_cast<int>(scaled_threshold);
  const int threshold = std::max(count_threshold, fraction_threshold);

  int required_shifts = 0;
  const double tolerance = state.options->optimality_tol;
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !state.bounds.enterable[static_cast<std::size_t>(j)] ||
        state.reduced_costs[j] <= tolerance ||
        std::isfinite(state.bounds.upper[j])) {
      continue;
    }
    ++required_shifts;
    if (required_shifts >= threshold) {
      return {false, required_shifts, threshold, true};
    }
  }
  return {true, required_shifts, threshold, false};
}

bool initialize_cost_shifted_dual_start(State& state, Statistics& statistics,
                                        std::string& failure) {
  state.original_cost = state.cost;
  state.cost_perturbation.reset(state.n);
  state.cost_shift.reset(state.n);
  state.costs_perturbed = false;
  state.costs_shifted = false;
  state.perturbation_disabled = false;

  const int previous_cost_shifts = statistics.cost_shifts;
  const double previous_max_cost_shift = statistics.max_cost_shift;
  int start_cost_shifts = 0;
  double start_max_cost_shift = 0.0;
  const double tolerance = state.options->optimality_tol;
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !state.bounds.enterable[static_cast<std::size_t>(j)]) {
      state.move[static_cast<std::size_t>(j)] = Move::Fixed;
      continue;
    }
    if (state.reduced_costs[j] <= tolerance) {
      state.move[static_cast<std::size_t>(j)] = Move::Up;
      continue;
    }
    if (std::isfinite(state.bounds.upper[j])) {
      state.move[static_cast<std::size_t>(j)] = Move::Down;
      continue;
    }

    const double shift = -state.reduced_costs[j];
    if (!std::isfinite(shift)) {
      failure = "dual-start cost shift is non-finite";
      return false;
    }
    state.cost[j] += shift;
    state.cost_shift.set(j, shift);
    state.reduced_costs[j] = 0.0;
    state.move[static_cast<std::size_t>(j)] = Move::Up;
    state.costs_shifted = true;
    ++start_cost_shifts;
    start_max_cost_shift = std::max(start_max_cost_shift, std::abs(shift));
  }

  if (!reconstruct(state, failure)) {
    failure = "cost-shifted dual-start reconstruction failed: " + failure;
    return false;
  }
  const Audit initial = audit(state, false, true, false);
  if (!initial.ok) {
    failure = "cost-shifted dual-start invariant failed: " + initial.failure;
    return false;
  }
  statistics.cost_shifts = previous_cost_shifts + start_cost_shifts;
  statistics.dual_start_cost_shifts += start_cost_shifts;
  statistics.max_cost_shift =
      std::max(previous_max_cost_shift, start_max_cost_shift);
  statistics.max_primal_infeasibility = initial.max_primal_infeasibility;
  statistics.max_dual_infeasibility = initial.max_dual_infeasibility;
  return true;
}

bool initialize_exact_edge_weights(State& state, Statistics& statistics,
                                   std::string& failure) {
  state.leaving_heap_valid = false;
  state.edge_weight_mode = EdgeWeightMode::SteepestEdge;
  state.devex_reference.clear();
  state.devex_iterations = 0;
  state.edge_weight.assign(static_cast<std::size_t>(state.m), 1.0);
  bool diagonal_basis = true;
  for (int row = 0; row < state.m && diagonal_basis; ++row) {
    const int col = state.basis[static_cast<std::size_t>(row)];
    double diagonal = 0.0;
    int nonzeros = 0;
    for (StandardColumnMatrix::InnerIterator it(state.sf->A, col); it;
         ++it) {
      if (it.value() == 0.0) continue;
      ++nonzeros;
      if (it.row() == row) diagonal = it.value();
    }
    if (nonzeros != 1 || diagonal == 0.0) {
      diagonal_basis = false;
      break;
    }
    state.edge_weight[static_cast<std::size_t>(row)] =
        1.0 / (diagonal * diagonal);
  }
  if (diagonal_basis) return true;

  const EdgeWeightEvidence batch = state.factor->compute_exact_edge_weights();
  statistics.dse_initialization_solves += state.m;
  statistics.dse_initialization_refinements += batch.refinements;
  statistics.iterative_refinements += batch.refinements;
  if (!batch.accepted) {
    std::ostringstream message;
    message << std::setprecision(17)
            << "batched DSE initialization failed on the current factor"
            << " (needs_rebuild=" << (batch.needs_rebuild ? 1 : 0)
            << ", max_residual=" << batch.max_residual
            << ", max_limit=" << batch.max_error_limit << ')';
    failure = message.str();
    return false;
  }
  state.edge_weight = batch.weights;
  return true;
}

void initialize_devex_framework(State& state, Statistics& statistics) {
  state.leaving_heap_valid = false;
  state.edge_weight_mode = EdgeWeightMode::Devex;
  state.edge_weight.assign(static_cast<std::size_t>(state.m), 1.0);
  state.devex_reference.assign(static_cast<std::size_t>(state.n), 0);
  for (int col : state.basis) {
    if (col >= 0 && col < state.n) {
      state.devex_reference[static_cast<std::size_t>(col)] = 1;
    }
  }
  state.devex_iterations = 0;
  ++statistics.devex_frameworks;
}

bool initialize_stabilized_cost(State& state, Statistics& statistics,
                                std::string& failure) {
  if (!state.costs_shifted) {
    state.original_cost = state.cost;
    state.cost_shift.reset(state.n);
  }
  state.cost_perturbation.reset(state.n);
  state.costs_perturbed = false;
  state.perturbation_disabled = false;
  if (state.phase != Phase::Two) return true;

  const Audit initial = audit(state, false, true, false);
  if (!initial.ok) {
    failure = "cannot initialize stabilized cost from an invalid dual state: " +
              initial.failure;
    return false;
  }
  if (initial.max_primal_infeasibility <= state.options->feasibility_tol) {
    return true;
  }

  double max_abs_structural_cost = 1.0;
  for (int j = 0; j < state.sf->n_original && j < state.n; ++j) {
    max_abs_structural_cost =
        std::max(max_abs_structural_cost, std::abs(state.original_cost[j]));
  }
  if (max_abs_structural_cost > 100.0) {
    max_abs_structural_cost =
        std::sqrt(std::sqrt(max_abs_structural_cost));
  }
  const double structural_base = 5e-7 * max_abs_structural_cost;
  const double logical_base = 1e-12;
  for (int j = 0; j < state.n; ++j) {
    if (!state.bounds.enterable[static_cast<std::size_t>(j)]) continue;
    const double fraction = deterministic_fraction(j);
    double perturbation = 0.0;
    if (j < state.sf->n_original) {
      const double magnitude = (1.0 + fraction) *
                               (std::abs(state.original_cost[j]) + 1.0) *
                               structural_base;
      if (std::isfinite(state.bounds.upper[j])) {
        perturbation = state.original_cost[j] >= 0.0 ? magnitude : -magnitude;
      } else {
        perturbation = -magnitude;
      }
    } else {
      perturbation = (0.5 - fraction) * logical_base;
    }
    state.cost_perturbation.set(j, perturbation);
    state.cost[j] += perturbation;
    statistics.max_cost_perturbation =
        std::max(statistics.max_cost_perturbation, std::abs(perturbation));
  }
  state.costs_perturbed = state.cost_perturbation.max_abs() > 0.0;
  return reconstruct(state, failure);
}

bool major_rebuild(State& state, RebuildReason reason, bool reinvert,
                   Statistics& statistics, std::string& failure) {
  const Eigen::VectorXd previous_x = state.x_basic;
  const Eigen::VectorXd previous_reduced_costs = state.reduced_costs;
  const double previous_objective = state.objective;
  const int previous_updates = state.updates_since_rebuild;

  if (reinvert) {
    if (!state.factor->rebuild(state.basis, statistics.rank_repairs, failure)) {
      failure = "major INVERT failed: " + failure;
      return false;
    }
    ++statistics.reinversions;
  }
  if (!reconstruct(state, failure)) {
    failure = "major reconstruction failed: " + failure;
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

  // Boxed nonbasics can restore their dual-feasible side without modifying
  // the working objective. This is a complete side classification for the
  // current basis, not a second solve attempt.
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        !state.bounds.enterable[static_cast<std::size_t>(j)]) {
      state.move[static_cast<std::size_t>(j)] = Move::Fixed;
      continue;
    }
    state.move[static_cast<std::size_t>(j)] =
        state.reduced_costs[j] > state.options->optimality_tol &&
                std::isfinite(state.bounds.upper[j])
            ? Move::Down
            : Move::Up;
  }
  if (!reconstruct(state, failure, true, false)) {
    failure = "bound-side reconstruction failed: " + failure;
    return false;
  }
  // Only one-sided nonbasics can remain. Correct their working costs exactly
  // to zero reduced cost, as HEkkDual does during rebuild.
  for (int j = 0; j < state.n; ++j) {
    if (state.basic[static_cast<std::size_t>(j)] ||
        sign(state.move[static_cast<std::size_t>(j)]) == 0) {
      continue;
    }
    const double infeasibility =
        sign(state.move[static_cast<std::size_t>(j)]) * state.reduced_costs[j];
    if (infeasibility <= state.options->optimality_tol) continue;
    if (std::isfinite(state.bounds.upper[j])) {
      failure = "boxed nonbasic side classification failed during rebuild";
      return false;
    }
    const double shift = -state.reduced_costs[j];
    state.cost[j] += shift;
    state.cost_shift.add(j, shift);
    state.reduced_costs[j] = 0.0;
    state.costs_shifted = true;
    ++statistics.cost_shifts;
    statistics.max_cost_shift =
        std::max(statistics.max_cost_shift, std::abs(shift));
  }
  if (state.costs_shifted && !reconstruct(state, failure, false, true)) {
    failure = "working-cost shift reconstruction failed: " + failure;
    return false;
  }

  const Audit rebuilt = audit(state, false, true, false);
  if (!rebuilt.ok) {
    failure = "major rebuild invariant failed: " + rebuilt.failure;
    return false;
  }
  statistics.max_primal_infeasibility = rebuilt.max_primal_infeasibility;
  statistics.max_dual_infeasibility = rebuilt.max_dual_infeasibility;
  statistics.max_updates_between_rebuilds =
      std::max(statistics.max_updates_between_rebuilds, previous_updates);
  ++statistics.major_rebuilds;
  switch (reason) {
    case RebuildReason::UpdateLimit:
      ++statistics.rebuild_update_limit;
      break;
    case RebuildReason::SyntheticWork:
      ++statistics.rebuild_synthetic_work;
      break;
    case RebuildReason::NumericalTrouble:
      ++statistics.rebuild_numerical_trouble;
      break;
    case RebuildReason::PossiblyOptimal:
      ++statistics.rebuild_possibly_optimal;
      break;
    case RebuildReason::PossiblyPrimalInfeasible:
      ++statistics.rebuild_possibly_infeasible;
      break;
    case RebuildReason::Initial:
    case RebuildReason::Cleanup:
      break;
  }
  state.updates_since_rebuild = 0;
  state.reinvert_after_pivot = false;
  state.fresh_rebuild = true;
  // The bound-side classification above may have changed nonbasic move sides
  // outside the minor iteration's incremental token updates — resync the live
  // cycle signature from the rebuilt state (O(m+n), amortized over the ~50-200
  // pivots between rebuilds).
  resync_cycle_signature(state);
  return true;
}

void restore_original_cost(State& state) {
  if (state.original_cost.size() == state.n) state.cost = state.original_cost;
  state.cost_perturbation.reset(state.n);
  state.cost_shift.reset(state.n);
  state.costs_perturbed = false;
  state.costs_shifted = false;
  state.perturbation_disabled = true;
  state.fresh_rebuild = false;
}

bool working_cost_is_original(const State& state) {
  return !state.costs_perturbed && !state.costs_shifted &&
         state.original_cost.size() == state.cost.size() &&
         (state.cost - state.original_cost).isZero(0.0);
}

void initialize_cycle_guard(State& state) {
  state.cycle_history.clear();
  state.taboo_changes.clear();
  state.taboo_rows.clear();
  state.cycle_history_fifo.clear();
  state.taboo_changes_fifo.clear();
  state.taboo_rows_fifo.clear();
  state.taboo_ticket = 0;
  const std::size_t expected_guard_entries = std::min<std::size_t>(
      kCycleHistoryCapacity,
      std::max<std::size_t>(64, 4 * static_cast<std::size_t>(std::max(1, state.m))));
  state.cycle_history.reserve(expected_guard_entries);
  state.taboo_changes.reserve(
      std::min(expected_guard_entries, kTabooCapacity));
  state.taboo_rows.reserve(std::min(expected_guard_entries, kTabooCapacity));
  state.pivot_sequence = 0;
  state.pricing_epoch = 0;
  resync_cycle_signature(state);
  const std::uint64_t a = state.cycle_signature_live_a;
  const std::uint64_t b = state.cycle_signature_live_b;
  state.cycle_signature_a = a;
  state.cycle_signature_b = b;
  State::CycleRecord record;
  record.signature_a = a;
  record.signature_b = b;
  const CycleKey key = std::make_pair(a, b);
  state.cycle_history.emplace(key, record);
  state.cycle_history_fifo.push_back(key);
}

bool is_taboo_change(const State& state, int leaving_col, int entering_col,
                     int* expires_after) {
  const auto key = std::make_tuple(state.cycle_signature_a,
                                   state.cycle_signature_b, leaving_col,
                                   entering_col);
  const auto found = state.taboo_changes.find(key);
  if (found == state.taboo_changes.end() ||
      found->second.expiry < state.pricing_epoch) {
    return false;
  }
  if (expires_after != nullptr) *expires_after = found->second.expiry;
  return true;
}

bool is_taboo_row(const State& state, int leaving_col, int* expires_after) {
  const auto key = std::make_tuple(state.cycle_signature_a,
                                   state.cycle_signature_b, leaving_col);
  const auto found = state.taboo_rows.find(key);
  if (found == state.taboo_rows.end() ||
      found->second.expiry < state.pricing_epoch) {
    return false;
  }
  if (expires_after != nullptr) *expires_after = found->second.expiry;
  return true;
}

void add_taboo_row(State& state, int leaving_col, Statistics& statistics) {
  const int lifetime = taboo_lifetime(state.m);
  const auto key = std::make_tuple(state.cycle_signature_a,
                                   state.cycle_signature_b, leaving_col);
  auto [position, inserted] = state.taboo_rows.try_emplace(key);
  position->second.expiry = state.pricing_epoch + lifetime;
  if (inserted) {
    position->second.ticket = ++state.taboo_ticket;
    state.taboo_rows_fifo.emplace_back(key, position->second.ticket);
  }
  enforce_taboo_capacity(state.taboo_rows, state.taboo_rows_fifo);
  if (inserted) ++statistics.taboo_rows;
}

void release_taboo_row(State& state, int leaving_col,
                       Statistics& statistics) {
  const auto key = std::make_tuple(state.cycle_signature_a,
                                   state.cycle_signature_b, leaving_col);
  const auto found = state.taboo_rows.find(key);
  if (found == state.taboo_rows.end()) return;
  // Aspiration: if CHUZR has exhausted every non-taboo row, advance the
  // pricing clock beyond the earliest available row tenure. Edge taboos use
  // the same clock, so no blocked state can livelock without a basis change.
  state.pricing_epoch =
      std::max(state.pricing_epoch, found->second.expiry + 1);
  state.taboo_rows.erase(found);
  ++statistics.taboo_row_releases;
}

void record_cycle_departure(State& state, int leaving_col, int entering_col) {
  const auto key =
      std::make_pair(state.cycle_signature_a, state.cycle_signature_b);
  auto [position, inserted] = state.cycle_history.try_emplace(key);
  State::CycleRecord& record = position->second;
  if (inserted) {
    record.signature_a = key.first;
    record.signature_b = key.second;
    state.cycle_history_fifo.push_back(key);
    while (state.cycle_history.size() > kCycleHistoryCapacity) {
      const CycleKey old_key = state.cycle_history_fifo.front();
      state.cycle_history_fifo.pop_front();
      state.cycle_history.erase(old_key);
    }
  }
  record.leaving_col = leaving_col;
  record.entering_col = entering_col;
  record.last_seen = state.pivot_sequence;
}

bool record_cycle_arrival(State& state, Statistics& statistics) {
  ++state.pivot_sequence;
  // The live signature was maintained incrementally at the pivot commit; the
  // env hook cross-checks it against a full recomputation (test/debug only).
  static const bool verify_signature =
      std::getenv("MIPSOLVERS_DS_VERIFY_SIG") != nullptr;
  if (verify_signature) {
    const auto [full_a, full_b] = compute_cycle_signature(state);
    if (full_a != state.cycle_signature_live_a ||
        full_b != state.cycle_signature_live_b) {
      std::fprintf(stderr,
                   "[DS-SIG] incremental cycle signature diverged "
                   "(pivot_sequence=%d updates_since_rebuild=%d "
                   "live=%016llx/%016llx full=%016llx/%016llx)\n",
                   state.pivot_sequence, state.updates_since_rebuild,
                   static_cast<unsigned long long>(
                       state.cycle_signature_live_a),
                   static_cast<unsigned long long>(
                       state.cycle_signature_live_b),
                   static_cast<unsigned long long>(full_a),
                   static_cast<unsigned long long>(full_b));
      std::abort();
    }
  }
  const std::uint64_t a = state.cycle_signature_live_a;
  const std::uint64_t b = state.cycle_signature_live_b;
  state.cycle_signature_a = a;
  state.cycle_signature_b = b;
  const auto key = std::make_pair(a, b);
  auto [position, inserted] = state.cycle_history.try_emplace(key);
  State::CycleRecord& record = position->second;
  if (inserted) {
    record.signature_a = a;
    record.signature_b = b;
    record.last_seen = state.pivot_sequence;
    state.cycle_history_fifo.push_back(key);
    while (state.cycle_history.size() > kCycleHistoryCapacity) {
      const CycleKey old_key = state.cycle_history_fifo.front();
      state.cycle_history_fifo.pop_front();
      state.cycle_history.erase(old_key);
    }
    return false;
  }

  ++statistics.cycles_detected;
  if (record.leaving_col >= 0 && record.entering_col >= 0) {
    const int lifetime = taboo_lifetime(state.m);
    const auto taboo_key =
        std::make_tuple(a, b, record.leaving_col, record.entering_col);
    const int expiry = state.pricing_epoch + lifetime;
    auto [taboo, taboo_inserted] =
        state.taboo_changes.try_emplace(taboo_key);
    taboo->second.expiry = expiry;
    if (taboo_inserted) {
      taboo->second.ticket = ++state.taboo_ticket;
      state.taboo_changes_fifo.emplace_back(taboo_key, taboo->second.ticket);
    }
    enforce_taboo_capacity(state.taboo_changes, state.taboo_changes_fifo);
    if (taboo_inserted) ++statistics.taboo_changes;
  }
  record.last_seen = state.pivot_sequence;
  return true;
}

bool initialize(State& state, const StandardFormLP& sf,
                const SimplexOptions& options, Phase phase,
                const SimplexBasis* hint, Statistics& statistics,
                std::string& failure) {
  state.sf = &sf;
  state.options = &options;
  state.phase = phase;
  state.m = static_cast<int>(sf.A.rows());
  state.n = static_cast<int>(sf.A.cols());
  state.kernel_threads = lp_kernel_threads(options);
  if (state.m <= 0 || state.n <= 0 || sf.b.size() != state.m ||
      sf.c_max.size() != state.n || sf.var_ub.size() != state.n) {
    failure = "standard-form dimensions are invalid";
    return false;
  }
  state.bounds = make_phase_two_bounds(sf);
  state.cost = sf.c_max;
  state.original_cost = sf.c_max;
  state.cost_perturbation.reset(state.n);
  state.cost_shift.reset(state.n);
  state.move.assign(static_cast<std::size_t>(state.n), Move::Up);
  // Dominating coefficient for the BFRT dot-product error bound. For column
  // j with k stored entries, dot_error_bound computes
  //   fl(gamma_k * dot + 256*eps*dot),  dot = fl(sum_i |row_ep_i * A_ij|),
  // whose value is bounded above in exact arithmetic by
  //   (gamma_k + 256*eps) * ||A_j||_1 * max|row_ep| * (1+eps)^(k+4).
  // The factor 2 dominates every accumulated rounding term for any k far
  // below 1/eps, so coefficient * max|row_ep| >= the computed bound always.
  state.bfrt_error_coef.assign(static_cast<std::size_t>(state.n), 0.0);
  for (int j = 0; j < state.n; ++j) {
    double norm1 = 0.0;
    int terms = 0;
    for (StandardColumnMatrix::InnerIterator it(sf.A, j); it; ++it) {
      norm1 += std::abs(it.value());
      ++terms;
    }
    const double eps = std::numeric_limits<double>::epsilon();
    const double product = terms * eps;
    const double gamma = product < 0.5 ? product / (1.0 - product) : 1.0;
    state.bfrt_error_coef[static_cast<std::size_t>(j)] =
        2.0 * ((gamma + 256.0 * eps) * norm1);
  }
  if (hint != nullptr) {
    if (hint->rows != state.m || hint->cols != state.n ||
        static_cast<int>(hint->index_count()) != state.m) {
      failure = "basis hint dimensions do not match standard form";
      return false;
    }
    state.basis = hint->basis_indices();
    if (hint->at_upper.size() == static_cast<std::size_t>(state.n)) {
      for (int j = 0; j < state.n; ++j) {
        if (hint->at_upper[static_cast<std::size_t>(j)])
          state.move[static_cast<std::size_t>(j)] = Move::Down;
      }
    }
  } else if (!build_logical_basis(sf, state.basis, failure)) {
    return false;
  }
  state.factor =
      std::make_shared<BasisFactor>(sf, logical_columns(sf));
  if (!state.factor->rebuild(state.basis, statistics.rank_repairs, failure))
    return false;
  ++statistics.reinversions;
  if (!reconstruct(state, failure)) return false;
  state.fresh_rebuild = true;
  state.updates_since_rebuild = 0;
  initialize_cycle_guard(state);
  if (hint != nullptr && hint->cached_dse_basis &&
      hint->cached_dse_weights &&
      *hint->cached_dse_basis == state.basis &&
      hint->cached_dse_weights->size() == static_cast<std::size_t>(state.m)) {
    state.edge_weight_mode = EdgeWeightMode::SteepestEdge;
    state.devex_reference.clear();
    state.devex_iterations = 0;
    state.edge_weight = *hint->cached_dse_weights;
    return true;
  }
  if (hint != nullptr) {
    initialize_devex_framework(state, statistics);
    return true;
  }
  return initialize_cold_edge_weights(state, statistics, failure);
}

bool initialize_cold_edge_weights(State& state, Statistics& statistics,
                                  std::string& failure) {
  if (state.options != nullptr && state.options->exact_dse_initialization) {
    return initialize_exact_edge_weights(state, statistics, failure);
  }
  initialize_devex_framework(state, statistics);
  return true;
}

Eigen::VectorXd full_primal(const State& state) {
  Eigen::VectorXd x = state.bounds.lower;
  for (int j = 0; j < state.n; ++j) {
    if (!state.basic.empty() && state.basic[static_cast<std::size_t>(j)])
      continue;
    if (state.move[static_cast<std::size_t>(j)] == Move::Down)
      x[j] = state.bounds.upper[j];
  }
  for (int row = 0; row < state.m; ++row) {
    x[state.basis[static_cast<std::size_t>(row)]] = state.x_basic[row];
  }
  return x;
}

double primal_infeasibility(const State& state, int row, int& side) {
  const int col = state.basis[static_cast<std::size_t>(row)];
  const double value = state.x_basic[row];
  if (value < state.bounds.lower[col] - state.options->feasibility_tol) {
    side = -1;
    return state.bounds.lower[col] - value;
  }
  if (value > state.bounds.upper[col] + state.options->feasibility_tol) {
    side = 1;
    return value - state.bounds.upper[col];
  }
  side = 0;
  return 0.0;
}

double dual_infeasibility(const State& state, int col) {
  if (state.basic[static_cast<std::size_t>(col)]) return 0.0;
  const int direction = sign(state.move[static_cast<std::size_t>(col)]);
  if (direction == 0) return 0.0;
  return std::max(0.0, direction * state.reduced_costs[col]);
}

Audit audit(const State& state, bool require_primal, bool require_dual,
            bool require_artificial_zero, bool exact_residual) {
  Audit result;
  if (state.x_basic.size() != state.m ||
      state.reduced_costs.size() != state.n || !state.x_basic.allFinite() ||
      !state.reduced_costs.allFinite()) {
    result.failure = "state vectors are dimensionally invalid or non-finite";
    return result;
  }
  for (int row = 0; row < state.m; ++row) {
    const int col = state.basis[static_cast<std::size_t>(row)];
    result.max_basic_reduced_cost =
        std::max(result.max_basic_reduced_cost,
                 std::abs(state.reduced_costs[col]));
    int side = 0;
    const double infeasibility = primal_infeasibility(state, row, side);
    if (infeasibility > result.max_primal_infeasibility) {
      result.max_primal_infeasibility = infeasibility;
      result.worst_primal_row = row;
    }
  }
  for (int j = 0; j < state.n; ++j) {
    const double infeasibility = dual_infeasibility(state, j);
    if (infeasibility > result.max_dual_infeasibility) {
      result.max_dual_infeasibility = infeasibility;
      result.worst_dual_col = j;
    }
  }
  const Eigen::VectorXd x = full_primal(state);
  const Eigen::VectorXd equation_residual = equation_residual_vector(
      state.sf->A, x, state.sf->b, exact_residual);
  result.equation_residual =
      equation_residual.size() == state.m && equation_residual.allFinite()
          ? equation_residual.lpNorm<Eigen::Infinity>()
          : std::numeric_limits<double>::infinity();
  const double equation_limit =
      state.options->feasibility_tol *
      std::max(1.0, state.sf->b.lpNorm<Eigen::Infinity>());
  if (result.equation_residual > equation_limit) {
    std::ostringstream message;
    message << std::setprecision(17)
            << "Ax=b residual exceeds the canonical feasibility limit"
            << " (residual_inf=" << result.equation_residual
            << ", limit=" << equation_limit << ')';
    if (state.factor != nullptr) {
      message << "; last factor evidence: "
              << state.factor->last_solve_diagnostics();
    }
    result.failure = message.str();
    return result;
  }
  if (result.max_basic_reduced_cost > state.options->optimality_tol) {
    result.failure = "a basic reduced cost exceeds the dual tolerance";
    return result;
  }
  if (require_primal &&
      result.max_primal_infeasibility > state.options->feasibility_tol) {
    result.failure = "a basic variable violates its active bounds";
    return result;
  }
  if (require_dual &&
      result.max_dual_infeasibility > state.options->optimality_tol) {
    result.failure = "a nonbasic reduced cost violates its move sign";
    return result;
  }
  if (require_artificial_zero) {
    for (int col : state.sf->row_to_artificial_col) {
      if (col >= 0 && col < state.n &&
          std::abs(x[col]) > state.options->feasibility_tol) {
        result.failure = "an artificial variable is nonzero";
        return result;
      }
    }
  }
  result.ok = true;
  return result;
}

SimplexBasis export_basis(const State& state) {
  SimplexBasis basis;
  basis.indices = state.basis;
  basis.rows = state.m;
  basis.cols = state.n;
  basis.at_upper.assign(static_cast<std::size_t>(state.n), 0);
  for (int j = 0; j < state.n; ++j) {
    basis.at_upper[static_cast<std::size_t>(j)] =
        state.move[static_cast<std::size_t>(j)] == Move::Down;
  }
  basis.cached_dse_basis =
      std::make_shared<const std::vector<int>>(state.basis);
  basis.cached_dse_weights =
      std::make_shared<const std::vector<double>>(state.edge_weight);
  basis.cached_sparse_basis = state.factor;
  return basis;
}

Result make_result(const State& state, Status status, std::string message,
                   Statistics statistics) {
  Result result;
  result.status = status;
  result.message = std::move(message);
  result.statistics = statistics;
  result.statistics.canonical_primal_corrections +=
      state.canonical_primal_corrections;
  result.statistics.iterative_refinements +=
      state.canonical_primal_corrections;
  result.basis = state.basis;
  result.at_upper.assign(static_cast<std::size_t>(state.n), 0);
  for (int j = 0; j < state.n; ++j) {
    result.at_upper[static_cast<std::size_t>(j)] =
        state.move[static_cast<std::size_t>(j)] == Move::Down;
  }
  result.edge_weights = state.edge_weight;
  result.x_basic = state.x_basic;
  result.reduced_costs = state.reduced_costs;
  result.max_objective = state.objective;
  result.basis_ops = state.factor;
  return result;
}

}  // namespace mipsolvers::engine::native_dual::detail
