/// @file bc_fallback.hpp
/// @brief LP failure classification, fallback recovery, and event logging for B&C.
///
/// Provides structured LP failure handling to prevent silent subtree pruning
/// on numerical failures. Implements a multi-level fallback hierarchy:
///   L1: Perturbation-assisted retry, certified again on the original LP
///   L2: Solver switch (warm-start → cold-start simplex)
///   L3: Re-scale and retry (rebuild StandardFormLP with tighter tolerances)
///   L4: Node deferral (re-queue with lower priority)
///   L5: Hard prune (only on confirmed INFEASIBLE)

#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/detail/bc_types.hpp"

namespace mipsolvers::engine::detail {

// ════════════════════════════════════════════════════════════════════════════
// Task 1.1 — LP Failure Classification
// ════════════════════════════════════════════════════════════════════════════

/// @brief Classification of LP solve outcomes.
enum class LPFailureType {
  Success,           ///< LP solved successfully
  Infeasible,        ///< Problem is truly infeasible (Farkas certificate available)
  ObjectiveCutoff,   ///< Dual bound already exceeds incumbent; safe bound prune
  NumericalFailure,  ///< Ill-conditioning, precision loss, NaN objective
  Timeout,           ///< Node LP exceeded iteration/time limit
  Unbounded,         ///< LP relaxation is unbounded
  WrongDimension,    ///< Solution vector has unexpected size
  Unknown,           ///< Unclassified failure
};

/// @brief Convert LPFailureType to string for logging.
inline const char* lp_failure_type_str(LPFailureType t) {
  switch (t) {
    case LPFailureType::Success:          return "success";
    case LPFailureType::Infeasible:       return "infeasible";
    case LPFailureType::ObjectiveCutoff:  return "objective_cutoff";
    case LPFailureType::NumericalFailure: return "numerical_failure";
    case LPFailureType::Timeout:          return "timeout";
    case LPFailureType::Unbounded:        return "unbounded";
    case LPFailureType::WrongDimension:   return "wrong_dimension";
    case LPFailureType::Unknown:          return "unknown";
  }
  return "unknown";
}

/// @brief Structured result from an LP solve with fallback metadata.
struct LPNodeResult {
  LPFailureType failure_type{LPFailureType::Unknown};
  SimplexResult simplex_result;
  bool fallback_triggered{false};
  int fallback_level{0};  ///< 0 = no fallback, 1–5 = level that resolved it
};

/// @brief Classify a SimplexResult into an LPFailureType.
/// @param res The simplex result to classify.
/// @param expected_n Expected number of variables in the solution.
/// @return The classified failure type.
inline LPFailureType classify_lp_result(const SimplexResult& res, int expected_n) {
  if (res.result.stats.success && res.result.x.size() == expected_n &&
      std::isfinite(res.result.stats.objective)) {
    return LPFailureType::Success;
  }
  if (res.result.x.size() != expected_n && res.result.x.size() > 0) {
    return LPFailureType::WrongDimension;
  }
  if (res.result.stats.status.find("Objective cutoff") != std::string::npos) {
    return LPFailureType::ObjectiveCutoff;
  }
  // Only certified infeasibility is safe to prune in the tree.
  if (res.result.stats.has_farkas_certificate) {
    return LPFailureType::Infeasible;
  }

  // Check status string for specific failure modes.
  const auto& status = res.result.stats.status;
  if (status.find("unbounded") != std::string::npos ||
      status.find("Unbounded") != std::string::npos) {
    return LPFailureType::Unbounded;
  }
  if (status.find("timeout") != std::string::npos ||
      status.find("Timeout") != std::string::npos ||
      status.find("iteration limit") != std::string::npos ||
      status.find("Iteration limit") != std::string::npos ||
      status.find("max_iter") != std::string::npos) {
    return LPFailureType::Timeout;
  }
  if (!std::isfinite(res.result.stats.objective)) {
    return LPFailureType::NumericalFailure;
  }
  if (!res.result.stats.success) {
    return LPFailureType::NumericalFailure;
  }
  return LPFailureType::Unknown;
}

/// Whether the LP kernel reports an actual wall-clock deadline.  Do not infer
/// this from LPFailureType::Timeout: that category intentionally also contains
/// per-solve iteration limits, which are recoverable by the fallback chain and
/// must not terminate the complete branch-and-bound search.
inline bool lp_wall_time_limit_reached(const SimplexResult& res) {
  return res.result.stats.status.starts_with("Time limit");
}

// ════════════════════════════════════════════════════════════════════════════
// Task 1.3 — Fallback Event Logger
// ════════════════════════════════════════════════════════════════════════════

/// @brief Single fallback event record.
struct FallbackEvent {
  int node_id{-1};
  int node_depth{0};
  LPFailureType failure_type{LPFailureType::Unknown};
  int fallback_level{0};        ///< 1–5
  std::string outcome;          ///< "success", "deferred", "pruned"
  double timestamp_sec{0.0};    ///< Wall-clock offset from solve start
};

/// @brief Summary statistics for fallback events.
struct FallbackSummary {
  int total_events{0};
  int nodes_recovered{0};       ///< Fallback succeeded (L1–L3)
  int nodes_deferred{0};        ///< L4: node re-queued
  int nodes_pruned{0};          ///< Total L5 hard prunes
  int nodes_pruned_infeasible{0};
  int nodes_pruned_unbounded{0};
  int nodes_pruned_unsafe{0};   ///< Defensive counter: should stay zero
  int l1_attempts{0}, l1_successes{0};
  int l2_attempts{0}, l2_successes{0};
  int l3_attempts{0}, l3_successes{0};
};

/// @brief Thread-safe logger for fallback events during B&C solve.
class FallbackLogger {
 public:
  explicit FallbackLogger(std::chrono::steady_clock::time_point t0)
      : t0_(t0) {}

  /// @brief Record a fallback event.
  void log(int node_id, int node_depth, LPFailureType failure_type,
           int fallback_level, const std::string& outcome) {
    const auto now = std::chrono::steady_clock::now();
    const double ts = std::chrono::duration<double>(now - t0_).count();
    FallbackEvent ev;
    ev.node_id = node_id;
    ev.node_depth = node_depth;
    ev.failure_type = failure_type;
    ev.fallback_level = fallback_level;
    ev.outcome = outcome;
    ev.timestamp_sec = ts;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      events_.push_back(std::move(ev));
    }
  }

  /// @brief Compute summary statistics.
  FallbackSummary summary() const {
    std::lock_guard<std::mutex> lk(mtx_);
    FallbackSummary s;
    s.total_events = static_cast<int>(events_.size());
    for (const auto& ev : events_) {
      if (ev.outcome == "success") {
        ++s.nodes_recovered;
      } else if (ev.outcome == "deferred") {
        ++s.nodes_deferred;
      } else if (ev.outcome == "pruned") {
        ++s.nodes_pruned;
        switch (ev.failure_type) {
          case LPFailureType::Infeasible:
            ++s.nodes_pruned_infeasible;
            break;
          case LPFailureType::Unbounded:
            ++s.nodes_pruned_unbounded;
            break;
          default:
            ++s.nodes_pruned_unsafe;
            break;
        }
      }
      switch (ev.fallback_level) {
        case 1: ++s.l1_attempts; if (ev.outcome == "success") ++s.l1_successes; break;
        case 2: ++s.l2_attempts; if (ev.outcome == "success") ++s.l2_successes; break;
        case 3: ++s.l3_attempts; if (ev.outcome == "success") ++s.l3_successes; break;
        default: break;
      }
    }
    return s;
  }

  /// @brief Get all events (for serialization).
  std::vector<FallbackEvent> events() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return events_;
  }

  /// @brief Check if any fallback events were recorded.
  bool has_events() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return !events_.empty();
  }

 private:
  std::chrono::steady_clock::time_point t0_;
  mutable std::mutex mtx_;
  std::vector<FallbackEvent> events_;
};

// ════════════════════════════════════════════════════════════════════════════
// Task 1.2 — Fallback Manager
// ════════════════════════════════════════════════════════════════════════════

/// @brief Configuration for fallback behavior.
struct FallbackConfig {
  int l1_max_retries{3};           ///< L1: max perturbation retries
  double l1_perturbation{1e-7};    ///< L1: perturbation magnitude for RHS/bounds
  int l3_max_retries{2};           ///< L3: max re-scale retries
  double l3_tol_multiplier{10.0};  ///< L3: tolerance relaxation per retry
};

/// @brief Manages LP fallback recovery for B&C nodes.
///
/// Implements the fallback hierarchy:
///   L1: Perturbation-assisted retry — use the perturbed basis only to
///       re-solve the original LP
///   L2: Solver switch — cold-start simplex (no warm-start)
///   L3: Re-scale & retry — rebuild SF with relaxed tolerances
///   L4: Node deferral — re-queue node (not discarded)
///   L5: Hard prune — only on confirmed INFEASIBLE
///
/// CRITICAL INVARIANT: NumericalFailure NEVER triggers L5 (hard prune).
class FallbackManager {
 public:
  FallbackManager(const FallbackConfig& config, FallbackLogger& logger,
                  const SimplexOptions& base_simplex_opt)
      : config_(config), logger_(logger), base_simplex_opt_(base_simplex_opt) {}

  /// @brief Handle an LP failure at a B&C node.
  /// @param sf The standard-form LP (may be modified for perturbation/re-scale).
  /// @param basis_hint Basis hint for warm-start (nullptr for cold-start).
  /// @param failure_type Classified failure type.
  /// @param node_id Node identifier for logging.
  /// @param node_depth Node depth for logging.
  /// @param expected_n Expected solution dimension.
  /// @param[out] deferred Set to true if node should be deferred (L4).
  /// @return The recovered SimplexResult, or the last failed result.
  SimplexResult handle_failure(StandardFormLP& sf,
                               const SimplexBasis* basis_hint,
                               LPFailureType failure_type,
                               int node_id, int node_depth,
                               int expected_n,
                               bool& deferred,
                               const SimplexOptions* invocation_opt = nullptr) {
    deferred = false;

    const SimplexOptions retry_base =
        invocation_opt != nullptr ? *invocation_opt : base_simplex_opt_;
    const auto retry_start = std::chrono::steady_clock::now();
    auto deadline_result = [&]() {
      if (retry_base.time_limit_hit != nullptr) {
        *retry_base.time_limit_hit = true;
      }
      SimplexResult result;
      result.result.stats.success = false;
      result.result.stats.status =
          "Time limit: LP fallback budget exhausted";
      return result;
    };
    auto prepare_retry = [&](SimplexOptions& opts) {
      if (retry_base.time_limit_hit != nullptr &&
          *retry_base.time_limit_hit) {
        return false;
      }
      if (!(retry_base.time_limit_sec > 0.0) ||
          !std::isfinite(retry_base.time_limit_sec)) {
        return true;
      }
      const double elapsed = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - retry_start).count();
      const double remaining = retry_base.time_limit_sec - elapsed;
      if (remaining <= 0.0) return false;
      opts.time_limit_sec = remaining;
      opts.time_limit_hit = retry_base.time_limit_hit;
      return true;
    };
    auto wall_limit_reached = [&](const SimplexResult& result) {
      if (lp_wall_time_limit_reached(result)) {
        if (retry_base.time_limit_hit != nullptr) {
          *retry_base.time_limit_hit = true;
        }
        return true;
      }
      return retry_base.time_limit_hit != nullptr &&
             *retry_base.time_limit_hit;
    };

    // L5: Only safe prune — confirmed infeasible.
    if (failure_type == LPFailureType::Infeasible) {
      logger_.log(node_id, node_depth, failure_type, 5, "pruned");
      return {};  // Caller prunes node
    }

    // L5: Unbounded — prune (LP relaxation unbounded means MILP is also).
    if (failure_type == LPFailureType::Unbounded) {
      logger_.log(node_id, node_depth, failure_type, 5, "pruned");
      return {};
    }

    // For NUMERICAL_FAILURE, TIMEOUT, WRONG_DIMENSION, UNKNOWN:
    // Try recovery levels L1–L3 before deferring at L4.
    // CRITICAL: Never prune on numerical failure.

    // L1: A perturbed LP can supply a useful basis, but its primal objective
    // and dual bound are not valid for the original node. Accept this level
    // only after that basis solves the unmodified LP successfully.
    for (int retry = 0; retry < config_.l1_max_retries; ++retry) {
      StandardFormLP perturbed_sf = sf;
      perturb_rhs(perturbed_sf, config_.l1_perturbation * (retry + 1));

      SimplexOptions opts = retry_base;
      if (!prepare_retry(opts)) return deadline_result();
      auto result = solve_lp_from_sf(perturbed_sf, opts, basis_hint);
      if (wall_limit_reached(result)) return result;
      auto ft = classify_lp_result(result, expected_n);
      if (ft == LPFailureType::Success) {
        SimplexOptions verify_opts = retry_base;
        verify_opts.allow_cold_start = true;
        if (!prepare_retry(verify_opts)) return deadline_result();
        auto verified = solve_lp_from_sf(sf, verify_opts, &result.basis);
        if (wall_limit_reached(verified)) return verified;
        if (classify_lp_result(verified, expected_n) ==
            LPFailureType::Success) {
          logger_.log(node_id, node_depth, failure_type, 1, "success");
          return verified;
        }
      }
    }

    // L2: Solver switch — cold-start simplex (no warm-start basis)
    {
      SimplexOptions cold_opts = retry_base;
      cold_opts.allow_cold_start = true;
      cold_opts.max_iter = retry_base.max_iter * 2;
      if (!prepare_retry(cold_opts)) return deadline_result();
      auto result = solve_lp_from_sf(sf, cold_opts, nullptr);
      if (wall_limit_reached(result)) return result;
      auto ft = classify_lp_result(result, expected_n);
      if (ft == LPFailureType::Success) {
        logger_.log(node_id, node_depth, failure_type, 2, "success");
        return result;
      }
    }

    // L3: Re-scale and retry — rebuild with relaxed tolerances
    for (int retry = 0; retry < config_.l3_max_retries; ++retry) {
      SimplexOptions relaxed_opts = retry_base;
      const double multiplier = config_.l3_tol_multiplier * (retry + 1);
      relaxed_opts.feasibility_tol = retry_base.feasibility_tol * multiplier;
      relaxed_opts.optimality_tol = retry_base.optimality_tol * multiplier;
      relaxed_opts.allow_cold_start = true;
      relaxed_opts.max_iter = retry_base.max_iter * 3;

      if (!prepare_retry(relaxed_opts)) return deadline_result();
      auto result = solve_lp_from_sf(sf, relaxed_opts, nullptr);
      if (wall_limit_reached(result)) return result;
      auto ft = classify_lp_result(result, expected_n);
      if (ft == LPFailureType::Success) {
        logger_.log(node_id, node_depth, failure_type, 3, "success");
        return result;
      }
    }

    // L4: Node deferral — all recovery levels exhausted
    logger_.log(node_id, node_depth, failure_type, 4, "deferred");
    deferred = true;
    return {};  // Caller re-queues node
  }

 private:
  /// @brief Add small perturbation to RHS vector.
  static void perturb_rhs(StandardFormLP& sf, double magnitude) {
    // Deterministic perturbation based on row index (reproducible).
    for (int i = 0; i < sf.b.size(); ++i) {
      const double scale = std::max(1.0, std::abs(sf.b[i]));
      // Alternate sign: even rows get +, odd get -.
      const double sign = (i % 2 == 0) ? 1.0 : -1.0;
      sf.b[i] += sign * magnitude * scale;
    }
  }

  FallbackConfig config_;
  FallbackLogger& logger_;
  SimplexOptions base_simplex_opt_;
};

}  // namespace mipsolvers::engine::detail
