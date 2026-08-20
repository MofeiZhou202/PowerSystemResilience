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
#include <vector>

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
  if (options.cancel_flag != nullptr &&
      options.cancel_flag->load(std::memory_order_relaxed)) {
    if (options.time_limit_hit != nullptr) *options.time_limit_hit = true;
    return true;
  }
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

void sync_phase_telemetry(const Statistics& source, Statistics& target) {
  target.dual_start_cost_shifts = source.dual_start_cost_shifts;
  target.dual_start_required_cost_shift_lower_bound =
      source.dual_start_required_cost_shift_lower_bound;
  target.dual_start_shift_threshold = source.dual_start_shift_threshold;
  target.dual_start_adaptive_rejections =
      source.dual_start_adaptive_rejections;
  target.dual_phase_one_iterations = source.dual_phase_one_iterations;
  target.dual_phase_two_iterations = source.dual_phase_two_iterations;
  target.primal_phase_one_iterations = source.primal_phase_one_iterations;
  target.primal_phase_two_iterations = source.primal_phase_two_iterations;
  target.primal_cleanup_iterations = source.primal_cleanup_iterations;
  target.phase_transitions = source.phase_transitions;
  target.cleanup_required = source.cleanup_required;
  target.cleanup_avoided = source.cleanup_avoided;
  target.transition_dual_infeasibility_count =
      source.transition_dual_infeasibility_count;
  target.dual_phase_one_terminal_reason =
      source.dual_phase_one_terminal_reason;
  target.dual_phase_one_terminal_leaving_row =
      source.dual_phase_one_terminal_leaving_row;
  target.dual_phase_one_terminal_leaving_side =
      source.dual_phase_one_terminal_leaving_side;
  target.dual_phase_one_positive_candidates =
      source.dual_phase_one_positive_candidates;
  target.dual_phase_one_certified_candidates =
      source.dual_phase_one_certified_candidates;
  target.dual_phase_one_stable_candidates =
      source.dual_phase_one_stable_candidates;
  target.dual_phase_one_time_sec = source.dual_phase_one_time_sec;
  target.dual_phase_two_time_sec = source.dual_phase_two_time_sec;
  target.primal_phase_one_time_sec = source.primal_phase_one_time_sec;
  target.primal_phase_two_time_sec = source.primal_phase_two_time_sec;
  target.cleanup_time_sec = source.cleanup_time_sec;
  target.primal_cleanup_time_sec = source.primal_cleanup_time_sec;
  target.dual_phase_one_initial_objective =
      source.dual_phase_one_initial_objective;
  target.dual_phase_one_final_objective =
      source.dual_phase_one_final_objective;
  target.transition_max_dual_infeasibility =
      source.transition_max_dual_infeasibility;
  target.dual_phase_one_terminal_violation =
      source.dual_phase_one_terminal_violation;
  target.dual_phase_one_positive_capacity =
      source.dual_phase_one_positive_capacity;
  target.dual_phase_one_certified_capacity =
      source.dual_phase_one_certified_capacity;
  target.dual_phase_one_stable_capacity =
      source.dual_phase_one_stable_capacity;
  target.dual_phase_one_stable_capacity_error =
      source.dual_phase_one_stable_capacity_error;
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
struct LinearFit {
  long samples{0};
  long double sum_x{0.0L};
  long double sum_y{0.0L};
  long double sum_xx{0.0L};
  long double sum_xy{0.0L};
  long double sum_yy{0.0L};

  void clear() { *this = {}; }
  void add(double x, double y) {
    if (!std::isfinite(x) || !std::isfinite(y)) return;
    const long double lx = x;
    const long double ly = y;
    ++samples;
    sum_x += lx;
    sum_y += ly;
    sum_xx += lx * lx;
    sum_xy += lx * ly;
    sum_yy += ly * ly;
  }
  double slope() const {
    const long double n = samples;
    const long double denominator = n * sum_xx - sum_x * sum_x;
    if (samples < 2 || denominator <= 0.0L) return 0.0;
    return static_cast<double>((n * sum_xy - sum_x * sum_y) / denominator);
  }
  double r_squared() const {
    const long double n = samples;
    const long double covariance = n * sum_xy - sum_x * sum_y;
    const long double variance_x = n * sum_xx - sum_x * sum_x;
    const long double variance_y = n * sum_yy - sum_y * sum_y;
    if (samples < 2 || variance_x <= 0.0L || variance_y <= 0.0L) return 0.0;
    const long double value = covariance * covariance /
                              (variance_x * variance_y);
    return static_cast<double>(std::min(1.0L, std::max(0.0L, value)));
  }
};

struct ReinvertAgeCell {
  long iterations{0};
  std::uint64_t solve_calls{0};
  long double solve_time_sec{0.0L};
  long double solve_synthetic_tick{0.0L};
};

struct ReinvertPhaseProfile {
  std::vector<ReinvertAgeCell> ages;
  double scheduled_reinvert_time_sec{0.0};
  double scheduled_reinvert_synthetic_tick{0.0};
  long long scheduled_interval_sum{0};
  long scheduled_reinverts{0};
  long safety_reinverts{0};

  void clear() { *this = {}; }

  void add_iteration(int age, double solve_time_sec,
                     double solve_synthetic_tick,
                     std::uint64_t solve_calls) {
    if (age < 0 || solve_calls == 0 || !std::isfinite(solve_time_sec) ||
        !std::isfinite(solve_synthetic_tick)) {
      return;
    }
    if (ages.size() <= static_cast<std::size_t>(age)) {
      ages.resize(static_cast<std::size_t>(age) + 1);
    }
    ReinvertAgeCell& cell = ages[static_cast<std::size_t>(age)];
    ++cell.iterations;
    cell.solve_calls += solve_calls;
    cell.solve_time_sec += solve_time_sec;
    cell.solve_synthetic_tick += solve_synthetic_tick;
  }

  void add_reinvert(bool scheduled, int interval, double time_sec,
                    double synthetic_tick) {
    if (!scheduled) {
      ++safety_reinverts;
      return;
    }
    ++scheduled_reinverts;
    scheduled_interval_sum += interval;
    scheduled_reinvert_time_sec += time_sec;
    scheduled_reinvert_synthetic_tick += synthetic_tick;
  }

  LinearFit time_fit() const {
    LinearFit fit;
    for (std::size_t age = 0; age < ages.size(); ++age) {
      const ReinvertAgeCell& cell = ages[age];
      if (cell.iterations == 0) continue;
      fit.add(static_cast<double>(age),
              static_cast<double>(cell.solve_time_sec / cell.iterations));
    }
    return fit;
  }

  LinearFit synthetic_tick_fit() const {
    LinearFit fit;
    for (std::size_t age = 0; age < ages.size(); ++age) {
      const ReinvertAgeCell& cell = ages[age];
      if (cell.iterations == 0) continue;
      fit.add(
          static_cast<double>(age),
          static_cast<double>(cell.solve_synthetic_tick / cell.iterations));
    }
    return fit;
  }

  long iterations() const {
    long total = 0;
    for (const ReinvertAgeCell& cell : ages) total += cell.iterations;
    return total;
  }

  std::uint64_t solve_calls() const {
    std::uint64_t total = 0;
    for (const ReinvertAgeCell& cell : ages) total += cell.solve_calls;
    return total;
  }
};

struct DSProfile {
  bool enabled = false;
  double start_clock = 0.0;
  double leaving = 0.0, price = 0.0, entering = 0.0, ftran = 0.0, dse = 0.0,
         rebuild = 0.0, minor_total = 0.0, primal = 0.0, edge_init = 0.0,
         postcond = 0.0, rc_update = 0.0, valid = 0.0, lu_update = 0.0,
         cycle = 0.0, bfrt_prefilter = 0.0, bfrt_candidate = 0.0,
         bfrt_sort = 0.0, bfrt_order = 0.0, bfrt_harris = 0.0,
         bfrt_terminal_scan = 0.0, bfrt_rhs = 0.0;
  long pivots = 0, rebuilds = 0;
  long bfrt_calls = 0, bfrt_candidates = 0, bfrt_groups = 0,
       bfrt_selected_group = 0, bfrt_flips = 0,
       bfrt_stability_prefiltered = 0, bfrt_exact_dots = 0,
       bfrt_exact_dots_wasted = 0;
  int bfrt_max_candidates = 0, bfrt_max_groups = 0,
      bfrt_max_selected_group = 0, bfrt_max_flips = 0;
  int model_m = 0, model_n = 0;
  // Wrapper-translation split (Step 1 measurement): total HFactor indexed-solve
  // wall time and the export/re-pack subset of it. Captured from the factor at
  // the end of solve_impl. solve_pure = solve_time - export_time.
  double solve_time = 0.0, export_time = 0.0;
  double export_btran_time = 0.0;
  detail::PriceDseArithmeticProfile arithmetic;
  std::uint64_t solve_count = 0;
  // Average structural support of the two hot simplex vectors.
  double sum_rowep_nnz = 0.0, sum_pivotrow_nnz = 0.0;
  long density_samples = 0;
  ReinvertPhaseProfile dual_one_reinvert;
  ReinvertPhaseProfile dual_two_reinvert;
  ReinvertPhaseProfile& reinvert_profile(Phase phase) {
    return phase == Phase::DualOne ? dual_one_reinvert : dual_two_reinvert;
  }
  void reset() {
    enabled = std::getenv("MIPSOLVERS_DS_PROFILE") != nullptr;
    leaving = price = entering = ftran = dse = rebuild = minor_total = primal =
        edge_init = postcond = rc_update = valid = lu_update = cycle =
            bfrt_prefilter = bfrt_candidate = bfrt_sort = bfrt_order =
                bfrt_harris = bfrt_terminal_scan = bfrt_rhs = 0.0;
    sum_rowep_nnz = sum_pivotrow_nnz = 0.0;
    density_samples = 0;
    dual_one_reinvert.clear();
    dual_two_reinvert.clear();
    pivots = rebuilds = 0;
    bfrt_calls = bfrt_candidates = bfrt_groups = bfrt_selected_group =
        bfrt_flips = bfrt_stability_prefiltered = bfrt_exact_dots =
            bfrt_exact_dots_wasted = 0;
    bfrt_max_candidates = bfrt_max_groups = bfrt_max_selected_group =
        bfrt_max_flips = 0;
    model_m = model_n = 0;
    solve_time = export_time = 0.0;
    export_btran_time = 0.0;
    arithmetic.clear();
    solve_count = 0;
    start_clock = enabled ? std::chrono::duration<double>(
                                std::chrono::steady_clock::now()
                                    .time_since_epoch())
                                .count()
                          : 0.0;
  }
  void report(int m, int n, const Statistics& statistics) const {
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
    {
      const double solve_pure = solve_time - export_time;
      const double solve_related = leaving + ftran + dse;
      const double export_ftran = export_time - export_btran_time;
      std::fprintf(
          stderr,
          "[DS-SOLVE-SPLIT] indexedSolves=%llu solveWall=%.3f "
          "pureSolve=%.3f export/convert=%.3f (%.1f%% of solveWall) | "
          "solveRelatedBuckets(leaving+ftran+dse)=%.3f export=%.1f%% of those "
          "| export=%.1f%% of minor=%.3f\n",
          static_cast<unsigned long long>(solve_count), solve_time, solve_pure,
          export_time, solve_time > 0 ? 100.0 * export_time / solve_time : 0.0,
          solve_related,
          solve_related > 0 ? 100.0 * export_time / solve_related : 0.0,
          minor_total > 0 ? 100.0 * export_time / minor_total : 0.0,
          minor_total);
      std::fprintf(
          stderr,
          "[DS-EXPORT-SPLIT] total=%.4f ftranConvert(reorder removes)=%.4f "
          "(%.1f%% of minor) btranPack(dedicated buffers remove)=%.4f "
          "(%.1f%% of minor)\n",
          export_time, export_ftran,
          minor_total > 0 ? 100.0 * export_ftran / minor_total : 0.0,
          export_btran_time,
          minor_total > 0 ? 100.0 * export_btran_time / minor_total : 0.0);
    }
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
    if (bfrt_calls > 0) {
      const double calls = static_cast<double>(bfrt_calls);
      std::fprintf(
          stderr,
          "[DS-BFRT] calls=%ld candidates=%.1f/%d order=%.6fs sort=%.6fs "
          "groups=%.1f/%d "
          "selectedGroup=%.2f/%d flips=%.2f/%d stabilityPrefiltered=%ld "
          "exactDots=%.1f/pivot(%ld) wastedExactDots=%.1f/pivot(%ld,%.0f%%)\n",
          bfrt_calls, bfrt_candidates / calls, bfrt_max_candidates,
          bfrt_order, bfrt_sort, bfrt_groups / calls, bfrt_max_groups,
          bfrt_selected_group / calls, bfrt_max_selected_group,
          bfrt_flips / calls, bfrt_max_flips,
          bfrt_stability_prefiltered, bfrt_exact_dots / calls, bfrt_exact_dots,
          bfrt_exact_dots_wasted / calls, bfrt_exact_dots_wasted,
          bfrt_exact_dots > 0
              ? 100.0 * static_cast<double>(bfrt_exact_dots_wasted) /
                    static_cast<double>(bfrt_exact_dots)
              : 0.0);
      std::fprintf(
          stderr,
          "[DS-BFRT-SPLIT] prefilter=%.6fs candidate=%.6fs order=%.6fs "
          "harris=%.6fs terminalScan=%.6fs rhs=%.6fs accounted=%.6fs "
          "(%.1f%% of entering)\n",
          bfrt_prefilter, bfrt_candidate, bfrt_order, bfrt_harris,
          bfrt_terminal_scan, bfrt_rhs,
          bfrt_prefilter + bfrt_candidate + bfrt_order + bfrt_harris +
              bfrt_terminal_scan + bfrt_rhs,
          entering > 0.0
              ? 100.0 * (bfrt_prefilter + bfrt_candidate + bfrt_order +
                         bfrt_harris + bfrt_terminal_scan + bfrt_rhs) /
                    entering
              : 0.0);
    }
    std::fprintf(
        stderr,
        "[DS-ARITH-SPLIT] priceAccumulate=%.6fs leavingDot=%.6fs "
        "pack=%.6fs audit=%.6fs priceAccounted=%.6fs (%.1f%% of price) | "
        "dseFtran=%.6fs recurrenceTransaction=%.6fs dseAccounted=%.6fs "
        "(%.1f%% of dse)\n",
        arithmetic.price_accumulate, arithmetic.price_leaving_dot,
        arithmetic.price_pack, arithmetic.price_audit,
        arithmetic.price_accumulate + arithmetic.price_leaving_dot +
            arithmetic.price_pack + arithmetic.price_audit,
        price > 0.0
            ? 100.0 * (arithmetic.price_accumulate +
                       arithmetic.price_leaving_dot + arithmetic.price_pack +
                       arithmetic.price_audit) /
                  price
            : 0.0,
        arithmetic.dse_ftran, arithmetic.dse_recurrence,
        arithmetic.dse_ftran + arithmetic.dse_recurrence,
        dse > 0.0
            ? 100.0 * (arithmetic.dse_ftran + arithmetic.dse_recurrence) / dse
            : 0.0);
    const auto report_reinvert = [](const char* phase,
                                    const ReinvertPhaseProfile& profile) {
      const LinearFit wall_fit = profile.time_fit();
      const LinearFit tick_fit = profile.synthetic_tick_fit();
      const double average_reinvert_time =
          profile.scheduled_reinverts > 0
              ? profile.scheduled_reinvert_time_sec /
                    profile.scheduled_reinverts
              : 0.0;
      const double average_reinvert_tick =
          profile.scheduled_reinverts > 0
              ? profile.scheduled_reinvert_synthetic_tick /
                    profile.scheduled_reinverts
              : 0.0;
      const double u_wall = wall_fit.slope();
      const double u_tick = tick_fit.slope();
      const double t_wall =
          u_wall > 0.0 && average_reinvert_time > 0.0
              ? std::sqrt(2.0 * average_reinvert_time / u_wall)
              : 0.0;
      const double t_tick =
          u_tick > 0.0 && average_reinvert_tick > 0.0
              ? std::sqrt(2.0 * average_reinvert_tick / u_tick)
              : 0.0;
      std::fprintf(
          stderr,
          "[DS-REINVERT-MODEL] phase=%s iterations=%ld ages=%ld "
          "solve_calls=%llu interval=%.1f scheduled=%ld safety=%ld "
          "R_sec=%.6g R_build_tick=%.6g u_sec=%.6g u_tick=%.6g "
          "T_wall=%.2f T_tick=%.2f r2=%.3f/%.3f\n",
          phase, profile.iterations(), wall_fit.samples,
          static_cast<unsigned long long>(profile.solve_calls()),
          profile.scheduled_reinverts > 0
              ? static_cast<double>(profile.scheduled_interval_sum) /
                    profile.scheduled_reinverts
              : 0.0,
          profile.scheduled_reinverts, profile.safety_reinverts,
          average_reinvert_time, average_reinvert_tick, u_wall, u_tick,
          t_wall, t_tick, wall_fit.r_squared(), tick_fit.r_squared());
    };
    if (dual_one_reinvert.iterations() > 0) {
      report_reinvert("I", dual_one_reinvert);
    }
    if (dual_two_reinvert.iterations() > 0) {
      report_reinvert("II", dual_two_reinvert);
    }
    std::fprintf(
        stderr,
        "[DS-PHASES] dualI=%.6fs/%d dualII=%.6fs/%d "
        "primalI=%.6fs/%d primalII=%.6fs/%d cleanup=%.6fs "
        "primalCleanup=%.6fs/%d transitions=%d phaseIObjective=%.17g->%.17g "
        "transitionDualInfeas=%d/%.3e startShifts=%d cleanupRequired=%d "
        "cleanupAvoided=%d\n",
        statistics.dual_phase_one_time_sec,
        statistics.dual_phase_one_iterations,
        statistics.dual_phase_two_time_sec,
        statistics.dual_phase_two_iterations,
        statistics.primal_phase_one_time_sec,
        statistics.primal_phase_one_iterations,
        statistics.primal_phase_two_time_sec,
        statistics.primal_phase_two_iterations, statistics.cleanup_time_sec,
        statistics.primal_cleanup_time_sec,
        statistics.primal_cleanup_iterations, statistics.phase_transitions,
        statistics.dual_phase_one_initial_objective,
        statistics.dual_phase_one_final_objective,
        statistics.transition_dual_infeasibility_count,
        statistics.transition_max_dual_infeasibility,
        statistics.dual_start_cost_shifts,
        statistics.cleanup_required, statistics.cleanup_avoided);
    const char* terminal =
        statistics.dual_phase_one_terminal_reason == 1
            ? "no-leaving"
            : (statistics.dual_phase_one_terminal_reason == 2
                   ? "no-entering"
                   : (statistics.dual_phase_one_terminal_reason == 3
                          ? "dual-feasible"
                          : "none"));
    std::fprintf(
        stderr,
        "[DS-PHASE-I-TERMINAL] kind=%s row=%d side=%d violation=%.17g "
        "candidates=%d/%d/%d capacity=%.17g/%.17g/%.17g "
        "stableCapacityError=%.3e\n",
        terminal, statistics.dual_phase_one_terminal_leaving_row,
        statistics.dual_phase_one_terminal_leaving_side,
        statistics.dual_phase_one_terminal_violation,
        statistics.dual_phase_one_positive_candidates,
        statistics.dual_phase_one_certified_candidates,
        statistics.dual_phase_one_stable_candidates,
        statistics.dual_phase_one_positive_capacity,
        statistics.dual_phase_one_certified_capacity,
        statistics.dual_phase_one_stable_capacity,
        statistics.dual_phase_one_stable_capacity_error);
  }
};
thread_local DSProfile g_ds_profile;
inline double ds_clock() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct MinorScratch {
  detail::IndexedVector entering_column;
  std::vector<std::pair<int, double>> primal_changes;
  std::vector<double> primal_delta;
  std::vector<unsigned int> primal_stamp;
  std::vector<int> primal_touched;
  unsigned int primal_epoch{0};
  detail::EdgeWeightUpdate edge_weight_update;
  std::vector<unsigned int> flipped_stamp;
  std::vector<unsigned int> shifted_stamp;
  std::vector<double> shift_delta;
  unsigned int transaction_epoch{0};
};
thread_local MinorScratch g_ds_scratch;

struct BfrtDelta {
  detail::IndexedVector packed;
  HFactorBackend::ResidentVectorView resident;
  bool factor_resident{false};

  void clear(int dimension) {
    packed.clear(dimension);
    resident = {};
    factor_resident = false;
  }
  int count() const {
    return factor_resident ? resident.count()
                           : static_cast<int>(packed.index.size());
  }
  int index(int position) const {
    return factor_resident
               ? resident.index(position)
               : packed.index[static_cast<std::size_t>(position)];
  }
  double value(int position) const {
    return factor_resident
               ? resident.value(position)
               : packed.value[static_cast<std::size_t>(position)];
  }
  double at(int row) const {
    return factor_resident ? resident.at(row) : packed.at(row);
  }
};
thread_local BfrtDelta g_bfrt_delta;

bool packed_bfrt_ftran_enabled() {
  static const bool enabled =
      std::getenv("MIPSOLVERS_DS_PACKED_BFRT_FTRAN") != nullptr;
  return enabled;
}

// S3 proposal step: build the primal transaction (basic-value changes) from the
// two FTRAN images into `scratch.primal_changes`. The scratch is a parameter so
// an S4 worker can propose on private storage; the arithmetic and order are the
// production ones. Returns false with a message on an invalid or non-finite row.
bool build_primal_transaction(MinorScratch& scratch, const State& state,
                               const BfrtDelta& bfrt_delta,
                              const detail::PivotalColumn& direction,
                              double primal_step, int leaving_row,
                              double entering_bound, std::string& failure) {
  std::vector<std::pair<int, double>>& primal_changes = scratch.primal_changes;
  primal_changes.clear();
  if (scratch.primal_delta.size() != static_cast<std::size_t>(state.m)) {
    scratch.primal_delta.assign(static_cast<std::size_t>(state.m), 0.0);
    scratch.primal_stamp.assign(static_cast<std::size_t>(state.m), 0);
    scratch.primal_epoch = 0;
  }
  if (++scratch.primal_epoch == 0) {
    std::fill(scratch.primal_stamp.begin(), scratch.primal_stamp.end(), 0);
    ++scratch.primal_epoch;
  }
  scratch.primal_touched.clear();
  auto add_primal_delta = [&](int row, double delta) {
    if (row < 0 || row >= state.m) return false;
    const std::size_t index = static_cast<std::size_t>(row);
    if (scratch.primal_stamp[index] != scratch.primal_epoch) {
      scratch.primal_stamp[index] = scratch.primal_epoch;
      scratch.primal_delta[index] = 0.0;
      scratch.primal_touched.push_back(row);
    }
    scratch.primal_delta[index] += delta;
    return true;
  };
  const int bfrt_count = bfrt_delta.count();
  for (int k = 0; k < bfrt_count; ++k) {
    if (!add_primal_delta(bfrt_delta.index(k), -bfrt_delta.value(k))) {
      failure = "BFRT primal transaction contains an invalid row";
      return false;
    }
  }
  const int direction_count = direction.count();
  for (int k = 0; k < direction_count; ++k) {
    if (!add_primal_delta(direction.index(k),
                          -direction.value(k) * primal_step)) {
      failure = "packed primal transaction contains an invalid row";
      return false;
    }
  }
  primal_changes.reserve(scratch.primal_touched.size() + 1);
  for (const int row : scratch.primal_touched) {
    const double delta = scratch.primal_delta[static_cast<std::size_t>(row)];
    if (row != leaving_row && delta != 0.0) {
      const double value = state.x_basic[row] + delta;
      if (!std::isfinite(value)) {
        failure = "packed primal transaction is non-finite";
        return false;
      }
      primal_changes.emplace_back(row, value);
    }
  }
  const double leaving_value = entering_bound + primal_step;
  if (!std::isfinite(leaving_value)) {
    failure = "packed primal transaction is non-finite";
    return false;
  }
  primal_changes.emplace_back(leaving_row, leaving_value);
  return true;
}

// S3 certification step: the analytical BFRT dual-feasibility postcondition.
// Reads solver state and the transaction's flip/shift stamps (via scratch); no
// mutation. Returns false with a message on a non-finite or dual-infeasible
// updated reduced cost.
bool certify_bfrt_dual_feasibility(const State& state,
                                   const MinorScratch& scratch,
                                   const detail::IndexedVector& pivot_row,
                                   int leaving_side, int entering_col,
                                   int leaving_col, double dual_step,
                                   std::string& failure) {
  const unsigned int transaction_epoch = scratch.transaction_epoch;
  auto is_flipped = [&](int col) {
    return scratch.flipped_stamp[static_cast<std::size_t>(col)] ==
           transaction_epoch;
  };
  auto cost_shift_at = [&](int col) {
    return scratch.shifted_stamp[static_cast<std::size_t>(col)] ==
                   transaction_epoch
               ? scratch.shift_delta[static_cast<std::size_t>(col)]
               : 0.0;
  };
  for (std::size_t k = 0; k < pivot_row.index.size(); ++k) {
    const int j = pivot_row.index[k];
    if ((state.basic[static_cast<std::size_t>(j)] && j != leaving_col) ||
        j == entering_col) {
      continue;
    }
    int move =
        j == leaving_col
            ? (state.bounds.enterable[static_cast<std::size_t>(j)]
                   ? (leaving_side < 0 ? detail::sign(Move::Up)
                                       : detail::sign(Move::Down))
                   : detail::sign(Move::Fixed))
            : detail::sign(state.move[static_cast<std::size_t>(j)]);
    if (j != leaving_col && is_flipped(j)) move = -move;
    if (move == 0) continue;
    const double updated_reduced_cost =
        state.reduced_costs[j] + dual_step * pivot_row.value[k] +
        cost_shift_at(j);
    if (!std::isfinite(updated_reduced_cost)) {
      failure =
          "analytical BFRT postcondition produced a non-finite reduced cost "
          "at column " +
          std::to_string(j);
      return false;
    }
    if (move * updated_reduced_cost > state.options->optimality_tol) {
      failure =
          "analytical BFRT postcondition violates dual feasibility at column " +
          std::to_string(j);
      return false;
    }
  }
  return true;
}

// S3 certification step: verify the BFRT flip-RHS FTRAN image agrees with the
// leaving-row change and derive the residual step. Reads state and bfrt_delta;
// returns the clamped remaining_delta via out-param, false with a message on a
// non-finite or beyond-envelope disagreement.
bool certify_bfrt_leaving_agreement(const State& state,
                                     const BfrtDelta& bfrt_delta,
                                    int leaving_row, int leaving_side,
                                    int leaving_col, double covered_violation,
                                    double violation, double& remaining_delta,
                                    std::string& failure) {
  const double leaving_bound = leaving_side < 0
                                   ? state.bounds.lower[leaving_col]
                                   : state.bounds.upper[leaving_col];
  const double bfrt_delta_at_row = bfrt_delta.at(leaving_row);
  remaining_delta =
      state.x_basic[leaving_row] - bfrt_delta_at_row - leaving_bound;
  if (covered_violation == violation) {
    remaining_delta = 0.0;
  }
  // The projected coverage (row_ep dot) and the FTRAN image agree only to
  // rounding, so a flip set that near-exactly covers the violation leaves a
  // remaining step whose sign is noise. Treat wrong-signed noise inside the
  // rounding envelope as the degenerate zero step; a disagreement beyond it is
  // still a real failure.
  const double remaining_slack =
      1024.0 * std::numeric_limits<double>::epsilon() *
      std::max({1.0, std::abs(state.x_basic[leaving_row]),
                std::abs(bfrt_delta_at_row), std::abs(leaving_bound)});
  if (!std::isfinite(remaining_delta) ||
      leaving_side * remaining_delta < -remaining_slack) {
    failure = "BFRT FTRAN disagrees with the leaving-row change";
    return false;
  }
  if (leaving_side * remaining_delta < 0.0) remaining_delta = 0.0;
  return true;
}

// One dual minor iteration as an S3 proposal / certification / commit pipeline.
//   Proposal    (read-only state + scratch): choose_leaving, multiply_AT_indexed_bfrt,
//               choose_entering_bfrt, the aq/BFRT FTRANs, compute_dse_weights,
//               build_primal_transaction.
//   Certification (no mutation, scalar order): finiteness of the priced row and
//               pivot, certify_bfrt_dual_feasibility, certify_bfrt_leaving_agreement.
//   Commit      (atomic): the factor update is the first and only fallible commit
//               step; basis/move/basic/reduced-cost/primal/weight state is published
//               only after it succeeds, so no rollback path is reachable.
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
    if (state.phase == Phase::DualOne) {
      statistics.dual_phase_one_terminal_reason = 1;
    }
    MinorOutcome outcome;
    outcome.kind = MinorKind::PossiblyOptimal;
    return outcome;
  }

  const double _t_pr = g_ds_profile.enabled ? ds_clock() : 0.0;
  if (state.sf->A_row.rows() != state.m || state.sf->A_row.cols() != state.n) {
    return numerical_trouble("PRICE row matrix is dimensionally inconsistent");
  }
  // The priced row is consumed entirely within this pivot, so its backing
  // storage is reused across iterations.
  static thread_local detail::IndexedVector pivot_row_storage;
  static thread_local std::vector<int> bfrt_active_position;
  if (!state.partition_row.empty()) {
    const int leaving_col_priced =
        state.basis[static_cast<std::size_t>(leaving.row)];
    if (leaving.has_resident_row_ep()) {
      detail::multiply_AT_partitioned_bfrt(
          state.partition_row, state.sf->A, leaving.resident_row_ep, state.move,
          leaving_col_priced, pivot_row_storage, bfrt_active_position,
          g_ds_profile.enabled ? &g_ds_profile.arithmetic : nullptr);
    } else {
      detail::multiply_AT_partitioned_bfrt(
          state.partition_row, state.sf->A, leaving.row_ep, state.move,
          leaving_col_priced, pivot_row_storage, bfrt_active_position,
          g_ds_profile.enabled ? &g_ds_profile.arithmetic : nullptr);
    }
  } else {
    if (leaving.has_resident_row_ep()) {
      detail::multiply_AT_indexed_bfrt(
          state.sf->A_row, leaving.resident_row_ep, state.basic, state.move,
          pivot_row_storage, bfrt_active_position,
          g_ds_profile.enabled ? &g_ds_profile.arithmetic : nullptr);
    } else {
      detail::multiply_AT_indexed_bfrt(
          state.sf->A_row, leaving.row_ep, state.basic, state.move,
          pivot_row_storage, bfrt_active_position,
          g_ds_profile.enabled ? &g_ds_profile.arithmetic : nullptr);
    }
  }
  const detail::IndexedVector& pivot_row = pivot_row_storage;
  const double audit_start = g_ds_profile.enabled ? ds_clock() : 0.0;
  const bool pivot_row_finite = pivot_row.finite();
  if (g_ds_profile.enabled) {
    g_ds_profile.arithmetic.price_audit += ds_clock() - audit_start;
  }
  if (!pivot_row_finite) {
    return numerical_trouble("packed PRICE produced non-finite values");
  }
  if (g_ds_profile.enabled) g_ds_profile.price += ds_clock() - _t_pr;
  if (g_ds_profile.enabled) {
    g_ds_profile.sum_rowep_nnz +=
        static_cast<double>(leaving.row_ep_count());
    g_ds_profile.sum_pivotrow_nnz +=
        static_cast<double>(pivot_row.index.size());
    ++g_ds_profile.density_samples;
  }
  PivotTransaction transaction;
  const double _t_en = g_ds_profile.enabled ? ds_clock() : 0.0;
  const bool _en_ok = detail::choose_entering_bfrt(
      state, leaving, pivot_row, bfrt_active_position, transaction, failure);
  if (g_ds_profile.enabled) g_ds_profile.entering += ds_clock() - _t_en;
  if (g_ds_profile.enabled && transaction.bfrt_candidate_count > 0) {
    ++g_ds_profile.bfrt_calls;
    g_ds_profile.bfrt_candidates += transaction.bfrt_candidate_count;
    g_ds_profile.bfrt_groups += transaction.bfrt_group_count;
    g_ds_profile.bfrt_selected_group +=
        transaction.bfrt_selected_group_size;
    g_ds_profile.bfrt_flips += static_cast<long>(transaction.flips.size());
    g_ds_profile.bfrt_sort += transaction.bfrt_sort_time_sec;
    g_ds_profile.bfrt_order += transaction.bfrt_order_time_sec;
    g_ds_profile.bfrt_prefilter += transaction.bfrt_prefilter_time_sec;
    g_ds_profile.bfrt_candidate += transaction.bfrt_candidate_time_sec;
    g_ds_profile.bfrt_harris += transaction.bfrt_harris_time_sec;
    g_ds_profile.bfrt_terminal_scan +=
        transaction.bfrt_terminal_scan_time_sec;
    g_ds_profile.bfrt_rhs += transaction.bfrt_rhs_time_sec;
    g_ds_profile.bfrt_stability_prefiltered +=
        transaction.bfrt_stability_prefiltered;
    g_ds_profile.bfrt_exact_dots += transaction.bfrt_exact_dot_calls;
    g_ds_profile.bfrt_exact_dots_wasted += transaction.bfrt_exact_dot_wasted;
    g_ds_profile.bfrt_max_candidates = std::max(
        g_ds_profile.bfrt_max_candidates, transaction.bfrt_candidate_count);
    g_ds_profile.bfrt_max_groups =
        std::max(g_ds_profile.bfrt_max_groups, transaction.bfrt_group_count);
    g_ds_profile.bfrt_max_selected_group =
        std::max(g_ds_profile.bfrt_max_selected_group,
                 transaction.bfrt_selected_group_size);
    g_ds_profile.bfrt_max_flips = std::max(
        g_ds_profile.bfrt_max_flips,
        static_cast<int>(transaction.flips.size()));
  }
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
    if (state.phase == Phase::DualOne) {
      statistics.dual_phase_one_terminal_reason = 2;
      statistics.dual_phase_one_terminal_leaving_row = leaving.row;
      statistics.dual_phase_one_terminal_leaving_side = leaving.side;
      statistics.dual_phase_one_terminal_violation = leaving.violation;
      statistics.dual_phase_one_positive_candidates =
          transaction.positive_candidate_count;
      statistics.dual_phase_one_certified_candidates =
          transaction.certified_candidate_count;
      statistics.dual_phase_one_stable_candidates =
          transaction.stable_candidate_count;
      statistics.dual_phase_one_positive_capacity =
          transaction.positive_capacity;
      statistics.dual_phase_one_certified_capacity =
          transaction.certified_capacity;
      statistics.dual_phase_one_stable_capacity =
          transaction.stable_capacity;
      statistics.dual_phase_one_stable_capacity_error =
          transaction.stable_capacity_error;
    }
    // Terminal Farkas construction outlives the minor-iteration hot path and
    // keeps its existing packed contract. Materialize only on this rare exit;
    // pivotal PRICE/BFRT/DSE remain factor-resident (derivation section 8.17).
    if (leaving.has_resident_row_ep()) {
      const int resident_count = leaving.resident_row_ep.count();
      leaving.row_ep.clear(state.m);
      leaving.row_ep.index.reserve(static_cast<std::size_t>(resident_count));
      leaving.row_ep.value.reserve(leaving.row_ep.index.capacity());
      for (int k = 0; k < resident_count; ++k) {
        const double value = leaving.resident_row_ep.value(k);
        if (value == 0.0) continue;
        leaving.row_ep.index.push_back(leaving.resident_row_ep.index(k));
        leaving.row_ep.value.push_back(value);
      }
    }
    outcome.kind = MinorKind::PossiblyPrimalInfeasible;
    outcome.leaving = std::move(leaving);
    return outcome;
  }

    detail::IndexedVector& column = g_ds_scratch.entering_column;
    column.clear(state.m);
    for (StandardColumnMatrix::InnerIterator it(state.sf->A,
                                                        entering.col);
         it; ++it) {
      column.index.push_back(it.row());
      column.value.push_back(it.value());
    }
    const double _t_ft = g_ds_profile.enabled ? ds_clock() : 0.0;
    detail::PivotalColumn direction;
    bool direction_accepted = false;
    if (std::getenv("MIPSOLVERS_DS_PACKED_COL_AQ") != nullptr) {
      detail::IndexedSolveEvidence direction_solve =
          state.factor->indexed_ftran(column, true);
      direction.packed = std::move(direction_solve.solution);
      direction.factor_resident = false;
      direction_accepted = direction_solve.accepted;
    } else {
      detail::ResidentSolveEvidence direction_solve =
          state.factor->resident_ftran(column);
      direction.resident = direction_solve.solution;
      direction.factor_resident = direction_solve.accepted;
      direction_accepted = direction_solve.accepted;
    }
    if (g_ds_profile.enabled) g_ds_profile.ftran += ds_clock() - _t_ft;
    if (!direction_accepted || !direction.finite()) {
      return numerical_trouble("pivotal-column FTRAN failed");
    }
    // col_aq (roadmap S2): read the pivotal element from the factor-resident
    // update_vec_aq backing (O(1)) instead of building a lookup over the packed
    // FTRAN image for one element. Bit-identical to direction.at(leaving.row);
    // the .at() fallback covers a stale capture.
    double column_pivot;
    if (!state.factor->captured_aq_value(leaving.row, column_pivot)) {
      column_pivot = direction.at(leaving.row);
    }
    if (!std::isfinite(column_pivot) || column_pivot == 0.0) {
      return numerical_trouble("packed pivotal-column FTRAN has no pivot");
    }

    const bool has_flips = !transaction.flips.empty();
    const bool has_shifts = !transaction.cost_shifts.empty();
    if (g_ds_scratch.flipped_stamp.size() !=
        static_cast<std::size_t>(state.n)) {
      g_ds_scratch.flipped_stamp.assign(static_cast<std::size_t>(state.n), 0);
      g_ds_scratch.shifted_stamp.assign(static_cast<std::size_t>(state.n), 0);
      g_ds_scratch.shift_delta.assign(static_cast<std::size_t>(state.n), 0.0);
      g_ds_scratch.transaction_epoch = 0;
    }
    if (++g_ds_scratch.transaction_epoch == 0) {
      std::fill(g_ds_scratch.flipped_stamp.begin(),
                g_ds_scratch.flipped_stamp.end(), 0);
      std::fill(g_ds_scratch.shifted_stamp.begin(),
                g_ds_scratch.shifted_stamp.end(), 0);
      ++g_ds_scratch.transaction_epoch;
    }
    const unsigned int transaction_epoch =
        g_ds_scratch.transaction_epoch;
    auto is_flipped = [&](int col) {
      return g_ds_scratch.flipped_stamp[static_cast<std::size_t>(col)] ==
             transaction_epoch;
    };
    for (std::size_t k = 0; k < transaction.flips.size(); ++k) {
      const BoundFlip& flip = transaction.flips[k];
      if (flip.col < 0 || flip.col >= state.n ||
          state.basic[static_cast<std::size_t>(flip.col)] ||
          flip.col == entering.col ||
          state.move[static_cast<std::size_t>(flip.col)] != flip.old_move ||
          !(flip.range > 0.0) || !std::isfinite(flip.range) ||
          is_flipped(flip.col)) {
        return numerical_trouble(
            "BFRT transaction contains an invalid bound flip");
      }
      g_ds_scratch.flipped_stamp[static_cast<std::size_t>(flip.col)] =
          transaction_epoch;
    }
    auto is_shifted = [&](int col) {
      return g_ds_scratch.shifted_stamp[static_cast<std::size_t>(col)] ==
             transaction_epoch;
    };
    for (std::size_t k = 0; k < transaction.cost_shifts.size(); ++k) {
      const detail::WorkingCostShift& shift = transaction.cost_shifts[k];
      if (shift.col < 0 || shift.col >= state.n ||
          state.basic[static_cast<std::size_t>(shift.col)] ||
          shift.col == entering.col || is_flipped(shift.col) ||
          !std::isfinite(shift.delta) || is_shifted(shift.col)) {
        return numerical_trouble(
            "BFRT transaction contains an invalid working-cost shift");
      }
      g_ds_scratch.shifted_stamp[static_cast<std::size_t>(shift.col)] =
          transaction_epoch;
      g_ds_scratch.shift_delta[static_cast<std::size_t>(shift.col)] =
          shift.delta;
    }
    auto cost_shift_at = [&](int col) {
      return is_shifted(col)
                 ? g_ds_scratch.shift_delta[static_cast<std::size_t>(col)]
                 : 0.0;
    };
    const int leaving_col =
        state.basis[static_cast<std::size_t>(leaving.row)];
    const double dual_step = leaving.side * entering.theta;
    // Cost shifts are created only while scanning pivot_row, whose stamped
    // accumulator already guarantees unique columns. Reuse its packed support
    // directly instead of copying and sorting it on every pivot.
    const double _t_pc = g_ds_profile.enabled ? ds_clock() : 0.0;
    if (!certify_bfrt_dual_feasibility(state, g_ds_scratch, pivot_row,
                                       leaving.side, entering.col, leaving_col,
                                       dual_step, failure)) {
      return numerical_trouble(std::move(failure));
    }
    if (g_ds_profile.enabled) g_ds_profile.postcond += ds_clock() - _t_pc;

    // BFRT's d=B^-1*r_flip remains in factor scratch. Its two consumers run
    // before the DSE auxiliary FTRAN reuses that scratch; the packed diagnostic
    // preserves the prior path (derivation document section 8.20).
    BfrtDelta& bfrt_delta = g_bfrt_delta;
    bfrt_delta.clear(state.m);
    if (has_flips) {
      if (packed_bfrt_ftran_enabled()) {
        if (!state.factor->indexed_ftran_into(transaction.bfrt_rhs,
                                               bfrt_delta.packed)) {
          return numerical_trouble("packed BFRT RHS FTRAN failed");
        }
      } else {
        detail::ResidentSolveEvidence solve =
            state.factor->resident_scratch_ftran(transaction.bfrt_rhs);
        bfrt_delta.resident = solve.solution;
        bfrt_delta.factor_resident = solve.accepted;
        if (!solve.accepted) {
          return numerical_trouble("resident BFRT RHS FTRAN failed");
        }
      }
    }

    double remaining_delta = 0.0;
    if (!certify_bfrt_leaving_agreement(
            state, bfrt_delta, leaving.row, leaving.side, leaving_col,
            transaction.covered_violation, leaving.violation, remaining_delta,
            failure)) {
      return numerical_trouble(std::move(failure));
    }

    // Maros (2003) section 9: build x_B-d-beta*step while d's resident view is
    // live. This proposal mutates only MinorScratch, so DSE can still fail
    // before the factor/basis/state commit without requiring rollback.
    const Move entering_old_move =
        state.move[static_cast<std::size_t>(entering.col)];
    const double entering_bound =
        entering_old_move == Move::Up ? state.bounds.lower[entering.col]
                                      : state.bounds.upper[entering.col];
    const double primal_step = remaining_delta / column_pivot;
    if (!build_primal_transaction(g_ds_scratch, state, bfrt_delta, direction,
                                  primal_step, leaving.row, entering_bound,
                                  failure)) {
      return numerical_trouble(std::move(failure));
    }
    const std::vector<std::pair<int, double>>& primal_changes =
        g_ds_scratch.primal_changes;

    detail::EdgeWeightUpdate& edge_weight_update =
        g_ds_scratch.edge_weight_update;
    bool restart_devex = false;
    const double _t_dse = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool _dse_ok = detail::compute_dse_weights(
        state, leaving, pivot_row, direction, column_pivot, edge_weight_update,
        restart_devex, failure,
        g_ds_profile.enabled ? &g_ds_profile.arithmetic : nullptr);
    if (g_ds_profile.enabled) g_ds_profile.dse += ds_clock() - _t_dse;
    if (!_dse_ok) {
      return numerical_trouble(std::move(failure));
    }

    // S3 commit: every certification and proposal above ran on unmutated state.
    // The factor update remains the first and only fallible commit step.
    const double _t_lu = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool _lu_ok =
        state.factor->update_indexed(leaving.row, entering.col, failure);
    if (g_ds_profile.enabled) g_ds_profile.lu_update += ds_clock() - _t_lu;
    if (!_lu_ok) {
      return numerical_trouble(std::move(failure));
    }
    // Factor committed. update_indexed reads none of the basis/move state, so
    // publishing the exchange here is bit-identical to the pre-update order.
    for (const BoundFlip& flip : transaction.flips) {
      state.move[static_cast<std::size_t>(flip.col)] =
          flip.old_move == Move::Up ? Move::Down : Move::Up;
    }
    state.move[static_cast<std::size_t>(leaving_col)] =
        state.bounds.enterable[static_cast<std::size_t>(leaving_col)]
            ? (leaving.side < 0 ? Move::Up : Move::Down)
            : Move::Fixed;
    state.move[static_cast<std::size_t>(entering.col)] = Move::Fixed;
    state.basis[static_cast<std::size_t>(leaving.row)] = entering.col;
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
    if (edge_weight_update.nonpivotal_row.size() !=
        edge_weight_update.nonpivotal_value.size()) {
      return numerical_trouble(
          "edge-weight transaction row/value sizes disagree");
    }
    for (std::size_t k = 0;
         k < edge_weight_update.nonpivotal_row.size(); ++k) {
      const int row = edge_weight_update.nonpivotal_row[k];
      if (row < 0 || row >= state.m || row == leaving.row) {
        return numerical_trouble(
            "edge-weight transaction contains an invalid row");
      }
      state.edge_weight[static_cast<std::size_t>(row)] =
          edge_weight_update.nonpivotal_value[k];
    }
    state.edge_weight[static_cast<std::size_t>(leaving.row)] =
        edge_weight_update.pivotal_value;
    state.basic[static_cast<std::size_t>(leaving_col)] = 0;
    state.basic[static_cast<std::size_t>(entering.col)] = 1;
    detail::apply_partition_row_swap(state, entering.col, leaving_col);
    if (detail::partition_row_verify_enabled() &&
        !state.partition_row.empty() &&
        !state.partition_row.verify(state.basic)) {
      return numerical_trouble(
          "partitioned row matrix drifted from the basis membership");
    }
    static thread_local std::vector<int> changed_primal_rows;
    changed_primal_rows.clear();
    changed_primal_rows.reserve(primal_changes.size());
    for (const auto& [row, value] : primal_changes) {
      state.x_basic[row] = value;
      changed_primal_rows.push_back(row);
    }
    // primal_changes always ends with leaving.row, so this stream already owns
    // every row whose basic value changed during the committed transaction.
    detail::refresh_leaving_heap(state, &changed_primal_rows);
    const double _t_rc = g_ds_profile.enabled ? ds_clock() : 0.0;
    for (std::size_t k = 0; k < pivot_row.index.size(); ++k) {
      const int j = pivot_row.index[k];
      if (j != leaving_col && j != entering.col &&
          state.basic[static_cast<std::size_t>(j)]) {
        state.reduced_costs[j] = 0.0;
      } else {
        state.reduced_costs[j] +=
            dual_step * pivot_row.value[k] + cost_shift_at(j);
      }
    }
    state.reduced_costs[entering.col] = 0.0;
    if (g_ds_profile.enabled) g_ds_profile.rc_update += ds_clock() - _t_rc;
    if (has_shifts) {
      state.costs_shifted = true;
      for (const detail::WorkingCostShift& shift : transaction.cost_shifts) {
        state.cost[shift.col] += shift.delta;
        state.cost_shift.add(shift.col, shift.delta);
        ++statistics.cost_shifts;
        statistics.max_cost_shift =
            std::max(statistics.max_cost_shift, std::abs(shift.delta));
      }
    }
    const double _t_ca = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool cycle_detected = detail::record_cycle_arrival(state, statistics);
    if (g_ds_profile.enabled) g_ds_profile.cycle += ds_clock() - _t_ca;
    // Bland's-rule anti-cycling trigger. record_cycle_arrival reports an EXACT
    // basis repeat (a genuine cycle), which -- unlike the working objective --
    // is not masked by the ~1e-6 objective jitter of a numerical limit cycle.
    // A hysteretic run length climbs on repeats and drains on fresh bases: once
    // it exceeds max(50, m/2) the taboo heuristic has demonstrably failed to
    // break the cycle, so engage smallest-index CHUZR/CHUZC (Bland 1977), which
    // produces a non-repeating basis sequence and hence finite termination.
    // Bland stays on until enough fresh bases drain the counter back to zero.
    {
      const int bland_threshold = std::max(50, state.m / 2);
      if (cycle_detected) {
        if (state.bland_stall_counter < 2 * bland_threshold) {
          ++state.bland_stall_counter;
        }
        if (state.bland_stall_counter >= bland_threshold) {
          state.bland_active = true;
        }
      } else if (state.bland_stall_counter > 0) {
        if (--state.bland_stall_counter == 0) {
          state.bland_active = false;
        }
      }
    }
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
    if (state.phase == Phase::DualOne) {
      ++statistics.dual_phase_one_iterations;
    } else {
      ++statistics.dual_phase_two_iterations;
    }
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
  if (!detail::reconstruct(state, failure, true, true, true)) return false;
  if (!detail::normalize_nonbasic_moves(state, failure)) return false;
  const Audit certification = detail::audit(state, false, true, false, true);
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
  detail::resync_partition_row(state);

  detail::RebuildReason rebuild_reason = detail::RebuildReason::Initial;
  std::string first_fresh_numerical_failure;
  auto finish_dual_phase_one = [&](bool require_primal) -> Result {
    if (!detail::working_cost_is_original(state)) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "dual Phase I changed the original objective", statistics);
    }
    const Audit phase_one_audit =
        detail::audit(state, require_primal, true, false);
    update_statistics(phase_one_audit, statistics);
    if (!phase_one_audit.ok) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "dual Phase-I terminal invariant failed: " +
              phase_one_audit.failure,
          statistics);
    }
    const double objective = detail::dual_phase_one_objective(state);
    if (!std::isfinite(objective)) {
      return detail::make_result(state, Status::NumericalFailure,
                                 "dual Phase-I objective is non-finite",
                                 statistics);
    }
    statistics.dual_phase_one_final_objective = objective;
    const detail::DualInfeasibilitySummary summary =
        detail::original_dual_infeasibility_summary(state);
    if (summary.count != 0) {
      return detail::make_result(
          state, Status::DualInfeasibleStart,
          "dual Phase I cannot produce an original-bound dual-feasible "
          "basis",
          statistics);
    }
    return detail::make_result(state, Status::Optimal,
                               "dual Phase I complete", statistics);
  };
  for (;;) {
    const bool reinvert = rebuild_reason != detail::RebuildReason::Initial;
    const int rebuild_interval = state.updates_since_rebuild;
    const double _t_rb = g_ds_profile.enabled ? ds_clock() : 0.0;
    const bool _rb_ok = detail::major_rebuild(state, rebuild_reason, reinvert,
                                              statistics, failure);
    if (g_ds_profile.enabled) {
      const double rebuild_elapsed = ds_clock() - _t_rb;
      g_ds_profile.rebuild += rebuild_elapsed;
      ++g_ds_profile.rebuilds;
      if (_rb_ok && reinvert) {
        g_ds_profile.reinvert_profile(state.phase).add_reinvert(
            rebuild_reason == detail::RebuildReason::UpdateLimit,
            rebuild_interval, rebuild_elapsed,
            state.factor->build_synthetic_tick());
      }
    }
    if (!_rb_ok) {
      return detail::make_result(state, Status::NumericalFailure,
                                 std::move(failure), statistics);
    }
    if (state.phase == Phase::DualOne &&
        detail::original_dual_infeasibility_summary(state).count == 0) {
      statistics.dual_phase_one_terminal_reason = 3;
      return finish_dual_phase_one(false);
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
      if (state.phase == Phase::Two &&
          state.options->incumbent_bound != nullptr &&
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

      const Phase iteration_phase = state.phase;
      const int iteration_age = state.updates_since_rebuild;
      const double solve_time_before =
          g_ds_profile.enabled
              ? state.factor->profiled_indexed_solve_time_sec()
              : 0.0;
      const double solve_tick_before =
          g_ds_profile.enabled
              ? state.factor->profiled_indexed_solve_synthetic_tick()
              : 0.0;
      const std::uint64_t solve_count_before =
          g_ds_profile.enabled
              ? state.factor->profiled_indexed_solve_count()
              : 0;
      const double _t_minor = g_ds_profile.enabled ? ds_clock() : 0.0;
      MinorOutcome outcome = minor_iteration(state, statistics);
      if (g_ds_profile.enabled) {
        g_ds_profile.minor_total += ds_clock() - _t_minor;
        g_ds_profile.reinvert_profile(iteration_phase).add_iteration(
            iteration_age,
            state.factor->profiled_indexed_solve_time_sec() -
                solve_time_before,
            state.factor->profiled_indexed_solve_synthetic_tick() -
                solve_tick_before,
            state.factor->profiled_indexed_solve_count() - solve_count_before);
      }
      // Anti-degeneracy re-perturbation. Bland's rule has saturated (the stall
      // counter is pinned at its 2*threshold cap) yet the cycle persists, which
      // means the active startup cost perturbation cancels across the cycling
      // basis (reduced costs ~1e-8). Replace it with a fresh, differently
      // seeded and escalated perturbation (Wolfe 1963 / Gill et al. EXPAND
      // 1989) and force a rebuild. Capped so a pathological instance still
      // terminates at the ordinary iteration limit rather than looping forever.
      constexpr int kMaxReperturbations = 12;
      if (state.bland_active &&
          state.bland_stall_counter >= 2 * std::max(50, state.m / 2) &&
          state.reperturbation_count < kMaxReperturbations &&
          state.phase == Phase::Two) {
        ++state.reperturbation_count;
        if (!detail::reperturb_stabilized_cost(
                state, state.reperturbation_count, statistics, failure)) {
          return detail::make_result(state, Status::NumericalFailure,
                                     std::move(failure), statistics);
        }
        state.bland_active = false;
        state.bland_stall_counter = 0;
        rebuild_reason = detail::RebuildReason::NumericalTrouble;
        break;
      }
      if (outcome.kind == MinorKind::CycleBlocked) {
        // An all-candidates-taboo minor iteration made no progress; count it
        // toward the same hysteretic Bland's-rule trigger as a detected cycle.
        const int bland_threshold = std::max(50, state.m / 2);
        if (state.bland_stall_counter < 2 * bland_threshold) {
          ++state.bland_stall_counter;
        }
        if (state.bland_stall_counter >= bland_threshold) {
          state.bland_active = true;
        }
        // Anti-cycling exhaustion guard. Once Bland's rule is engaged, every
        // re-perturbation has been spent, and the stall counter is pinned at
        // its 2*threshold cap, no escape mechanism remains: a CycleBlocked minor
        // iteration leaves the iterate unchanged (choose_leaving_bland re-picks
        // the same smallest-index row and the ratio test re-fails identically),
        // so the loop would otherwise spin on that iterate until the wall-clock
        // deadline — measured on MIPLIB3 blend2 as 9.8e6 no-progress minor
        // iterations against 21 pivots (12 s) inside a single node LP. The
        // CycleBlocked `continue` never reaches the top-of-loop max_iter test,
        // which is why the kMaxReperturbations cap ("terminate at the ordinary
        // iteration limit rather than looping forever") is otherwise defeated.
        // Terminate here with the same certified interrupted dual bound the
        // iteration-limit path returns; by weak duality it bounds the LP optimum
        // even though the basis is not optimal. Normal solves resolve their
        // cycle before exhaustion and never reach this state, so their pivot
        // paths and iteration counts are unchanged.
        if (state.phase == Phase::Two && state.bland_active &&
            state.reperturbation_count >= kMaxReperturbations &&
            state.bland_stall_counter >= 2 * bland_threshold) {
          std::ostringstream message;
          message << "dual simplex anti-cycling exhausted"
                  << " (iterations=" << statistics.iterations
                  << ", reperturbations=" << state.reperturbation_count
                  << ", cycles=" << statistics.cycles_detected
                  << ", taboo_rows=" << statistics.taboo_rows
                  << ", stability_blocked_rows="
                  << statistics.stability_blocked_rows << ')';
          const bool bound_certified = certify_interrupted_dual_bound(state);
          Result result = detail::make_result(
              state, Status::IterationLimit, message.str(), statistics);
          result.dual_bound_certified = bound_certified;
          return result;
        }
        continue;
      }
      if (outcome.kind == MinorKind::Pivoted) {
        first_fresh_numerical_failure.clear();
        if (state.phase == Phase::DualOne &&
            detail::original_dual_infeasibility_summary(state).count == 0) {
          // Phase I exists only to obtain an original-bound dual-feasible
          // basis. Rebuild before accepting the transition so the zero defect
          // is certified from freshly reconstructed reduced costs.
          rebuild_reason = detail::RebuildReason::PossiblyOptimal;
          break;
        }
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
        if (state.phase == Phase::DualOne) {
          return finish_dual_phase_one(false);
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

      if (state.phase == Phase::DualOne) {
        return finish_dual_phase_one(true);
      }

      if (!detail::working_cost_is_original(state)) {
        ++statistics.cleanup_required;
        const auto cleanup_start = std::chrono::steady_clock::now();
        detail::restore_original_cost(state);
        ++statistics.cleanup_passes;
        if (!detail::reconstruct(state, failure)) {
          statistics.cleanup_time_sec +=
              std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            cleanup_start)
                  .count();
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
            statistics.cleanup_time_sec +=
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - cleanup_start)
                    .count();
            return detail::make_result(
                state, Status::NumericalFailure,
                "cleanup lost primal feasibility: " + primal_audit.failure,
                statistics);
          }
          const int primal_cleanup_iteration_start = statistics.iterations;
          const auto primal_cleanup_start = std::chrono::steady_clock::now();
          Result cleanup =
              detail::run_primal_phase(state, statistics, start, true);
          statistics.primal_cleanup_iterations +=
              statistics.iterations - primal_cleanup_iteration_start;
          statistics.primal_cleanup_time_sec +=
              std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            primal_cleanup_start)
                  .count();
          if (cleanup.status == Status::Optimal) {
            cleanup.message = "optimal after unperturbed primal cleanup";
          } else {
            cleanup.message = "unperturbed primal cleanup: " + cleanup.message;
          }
          statistics.cleanup_time_sec +=
              std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                            cleanup_start)
                  .count();
          sync_phase_telemetry(statistics, cleanup.statistics);
          return cleanup;
        }
        statistics.cleanup_time_sec +=
            std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                          cleanup_start)
                .count();
      } else {
        ++statistics.cleanup_avoided;
      }

      const Audit final = detail::audit(
          state, true, true, state.phase == Phase::Two, true);
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
  state.factor->set_indexed_solve_profiling(g_ds_profile.enabled);
  // Capture the wrapper-translation split (Step 1 measurement) at scope exit,
  // while state.factor is still alive (report() runs after solve_impl returns).
  struct SolveSplitGuard {
    State& st;
    ~SolveSplitGuard() {
      if (!g_ds_profile.enabled || !st.factor) return;
      g_ds_profile.solve_time = st.factor->profiled_indexed_solve_time_sec();
      g_ds_profile.export_time = st.factor->profiled_indexed_export_time_sec();
      g_ds_profile.export_btran_time =
          st.factor->profiled_indexed_export_btran_time_sec();
      g_ds_profile.solve_count = st.factor->profiled_indexed_solve_count();
    }
  } solve_split_guard{state};

  auto run_dual_phase = [&]() -> Result {
    const Phase phase = state.phase;
    const double cleanup_before = statistics.cleanup_time_sec;
    const auto phase_start = std::chrono::steady_clock::now();
    Result result = run_phase(state, statistics, start);
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      phase_start)
            .count();
    if (phase == Phase::DualOne) {
      statistics.dual_phase_one_time_sec += elapsed;
    } else {
      statistics.dual_phase_two_time_sec +=
          std::max(0.0, elapsed -
                            (statistics.cleanup_time_sec - cleanup_before));
    }
    sync_phase_telemetry(statistics, result.statistics);
    return result;
  };

  auto run_primal_stage = [&](int& stage_iterations,
                              double& stage_time_sec) -> Result {
    const int iteration_start = statistics.iterations;
    const auto phase_start = std::chrono::steady_clock::now();
    Result result = detail::run_primal_phase(state, statistics, start);
    stage_iterations += statistics.iterations - iteration_start;
    stage_time_sec +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      phase_start)
            .count();
    sync_phase_telemetry(statistics, result.statistics);
    return result;
  };

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
    Result phase_one =
        run_primal_stage(statistics.primal_phase_one_iterations,
                         statistics.primal_phase_one_time_sec);
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
    return run_primal_stage(statistics.primal_phase_two_iterations,
                            statistics.primal_phase_two_time_sec);
  };

  auto try_cost_shifted_dual_start =
      [&](std::string& dual_start_failure) -> bool {
    const detail::DualShiftStartDecision decision =
        detail::decide_cost_shifted_dual_start(
            state, std::getenv("MIPSOLVERS_DUAL_SHIFT_START"));
    statistics.dual_start_required_cost_shift_lower_bound =
        decision.required_shift_lower_bound;
    statistics.dual_start_shift_threshold = decision.shift_threshold;
    statistics.dual_start_adaptive_rejections =
        decision.adaptive_rejection ? 1 : 0;
    if (!decision.try_shift_start) return false;
    if (!detail::initialize_cost_shifted_dual_start(
            state, statistics, dual_start_failure)) {
      if (std::getenv("MIPSOLVERS_DS_VERBOSE") != nullptr) {
        std::fprintf(stderr,
                     "[DUAL-SHIFT-START] initialization failed: %s\n",
                     dual_start_failure.c_str());
      }
      return false;
    }
    if (statistics.dual_start_required_cost_shift_lower_bound == 0) {
      statistics.dual_start_required_cost_shift_lower_bound =
          statistics.dual_start_cost_shifts;
    }
    return true;
  };

  // A cold basis that cannot select original-bound dual-feasible endpoints
  // enters a genuine dual Phase I. The logical anchor x0 satisfies Ax0=b, so
  // the auxiliary bounds act on z=x-x0 and the dual Phase-I objective is the
  // signed sum of unavoidable original dual infeasibilities.
  if (basis_hint == nullptr) {
    const int crash_replacements =
        detail::apply_certified_singleton_crash(sf, state.basis);
    if (crash_replacements > 0) {
      const double _t_ew = g_ds_profile.enabled ? ds_clock() : 0.0;
      const bool _ew_ok =
          state.factor->rebuild(state.basis, statistics.rank_repairs, failure) &&
          detail::reconstruct(state, failure) &&
          detail::initialize_uncached_edge_weights(state, statistics, failure);
      if (g_ds_profile.enabled) g_ds_profile.edge_init += ds_clock() - _t_ew;
      if (!_ew_ok) {
        return detail::make_result(
            state, Status::NumericalFailure,
            "certified singleton crash reconstruction failed: " + failure,
            statistics);
      }
      ++statistics.reinversions;
    }

    // For this fixed cold basis, original-bound endpoint selection is an exact
    // dual-feasibility classification. When it fails, zero the reduced costs
    // of infeasible one-sided nonbasics and enter Phase II directly. The
    // existing mandatory original-cost cleanup removes those shifts. Dual
    // Phase I remains the audited fallback if the shifted start cannot be
    // constructed.
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
      return run_dual_phase();
    }

    if (try_cost_shifted_dual_start(dual_start_failure)) {
      return run_dual_phase();
    }

    const std::vector<int> pre_phase_one_basis = state.basis;
    auto restore_pre_phase_one_basis = [&]() -> bool {
      state.basis = pre_phase_one_basis;
      if (!state.factor->rebuild(state.basis, statistics.rank_repairs,
                                 dual_start_failure)) {
        return false;
      }
      ++statistics.reinversions;
      state.updates_since_rebuild = 0;
      state.fresh_rebuild = true;
      return true;
    };

    if (!detail::initialize_dual_phase_one(state, dual_start_failure)) {
      if (std::getenv("MIPSOLVERS_DS_VERBOSE") != nullptr) {
        std::fprintf(stderr, "[DUAL-PHASE-I] initialization failed: %s\n",
                     dual_start_failure.c_str());
      }
      if (!restore_pre_phase_one_basis()) {
        return detail::make_result(
            state, Status::NumericalFailure,
            "cannot restore the pre-dual-Phase-I basis: " +
                dual_start_failure,
            statistics);
      }
      return run_primal_phase_one_from_current_basis();
    }
    statistics.dual_phase_one_initial_objective =
        detail::dual_phase_one_objective(state);
    Result phase_one = run_dual_phase();
    if (phase_one.status != Status::Optimal) {
      if (phase_one.status == Status::DualInfeasibleStart) {
        if (!restore_pre_phase_one_basis()) {
          return detail::make_result(
              state, Status::NumericalFailure,
              "cannot restore the pre-dual-Phase-I basis: " +
                  dual_start_failure,
              statistics);
        }
        return run_primal_phase_one_from_current_basis();
      }
      phase_one.message = "dual Phase I: " + phase_one.message;
      return phase_one;
    }

    detail::DualInfeasibilitySummary transition_summary;
    if (!detail::transition_dual_phase_one_to_two(
            state, transition_summary, dual_start_failure)) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "dual Phase-I transition failed: " + dual_start_failure,
          statistics);
    }
    ++statistics.phase_transitions;
    statistics.transition_dual_infeasibility_count =
        transition_summary.count;
    statistics.transition_max_dual_infeasibility = transition_summary.max;
    Result phase_two = run_dual_phase();
    if (phase_two.status != Status::NumericalFailure) return phase_two;
    if (std::getenv("MIPSOLVERS_DS_VERBOSE") != nullptr) {
      std::fprintf(stderr,
                   "[DUAL-PHASE-II] numerical failure after dual Phase I; "
                   "restoring the cold basis: %s\n",
                   phase_two.message.c_str());
    }
    if (!restore_pre_phase_one_basis()) {
      return detail::make_result(
          state, Status::NumericalFailure,
          "cannot restore the pre-dual-Phase-I basis after Phase-II failure: " +
              dual_start_failure,
          statistics);
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
    return run_dual_phase();
  }
  if (try_cost_shifted_dual_start(failure)) {
    return run_dual_phase();
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
    case Status::Unbounded:
      return "Unbounded";
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
  g_ds_profile.report(g_ds_profile.model_m, g_ds_profile.model_n,
                      result.statistics);
  return result;
}

// AUDIT-NAV: 原生对偶单纯形总入口；solve_impl 拥有 Phase I/II、定价、BFRT、
// 基更新和证书构造，本层负责最终 profiling 与原问题残差审计。
Result solve(const StandardFormLP& sf, const SimplexOptions& options,
             const SimplexBasis* basis_hint) {
  Result result = solve_impl(sf, options, basis_hint, true);
  g_ds_profile.report(g_ds_profile.model_m, g_ds_profile.model_n,
                      result.statistics);
  if (std::getenv("MIPSOLVERS_DS_VERBOSE") != nullptr) {
    const Statistics& s = result.statistics;
    std::fprintf(
        stderr,
        "DS %s: m=%d n=%d iters=%d degen_dual=%d degen_primal=%d "
        "bound_flips=%d cost_shifts=%d start_shifts=%d required_shifts_lb=%d "
        "shift_threshold=%d adaptive_phase1=%d devex_frameworks=%d "
        "devex_restarts=%d "
        "cycles=%d taboo_rej=%d taboo_row_rej=%d stab_blocked=%d "
        "major_rebuilds=%d reinversions=%d message='%s'\n",
        status_name(result.status), static_cast<int>(result.basis.size()),
        static_cast<int>(result.reduced_costs.size()), s.iterations,
        s.degenerate_dual_steps, s.degenerate_primal_steps, s.bound_flips,
        s.cost_shifts, s.dual_start_cost_shifts,
        s.dual_start_required_cost_shift_lower_bound,
        s.dual_start_shift_threshold,
        s.dual_start_adaptive_rejections, s.devex_frameworks,
        s.devex_restarts, s.cycles_detected, s.taboo_rejections,
        s.taboo_row_rejections, s.stability_blocked_rows, s.major_rebuilds,
        s.reinversions, result.message.c_str());
  }
  return result;
}

Result solve(const StandardFormLP& sf, const SimplexOptions& options,
             const SimplexBasis& basis_hint) {
  return solve(sf, options, &basis_hint);
}

}  // namespace mipsolvers::engine::native_dual
