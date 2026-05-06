/// @file detail/bc/branching/pseudocost.hpp
/// @brief Pseudocost tracking and branching statistics (Phase 4.5 reorganization).
///
/// Extracted from bc_types.hpp.
/// Defines pseudocost structures for dynamic variable selection heuristics.

#pragma once

#include <cmath>
#include <vector>

namespace mipsolvers::engine::detail {

/// @brief Pseudo-cost estimates for branching variable selection.
/// 
/// Tracks sum, sum-of-squares, and count for Welford-style variance computation.
/// Supports dual-branching (down/up) with inference and conflict scoring.
struct PseudoCost {
  double down_sum{0.0};
  double up_sum{0.0};
  double down_sq{0.0};  ///< Sum of squared observations (for variance)
  double up_sq{0.0};    ///< Sum of squared observations (for variance)
  double down_inference_sum{0.0};
  double up_inference_sum{0.0};
  double down_conflict_score{0.0};
  double up_conflict_score{0.0};
  int down_cnt{0};
  int up_cnt{0};
  int down_inference_cnt{0};
  int up_inference_cnt{0};
  int down_cutoff_cnt{0};
  int up_cutoff_cnt{0};

  /// @brief Average down-branch cost (default 1.0 if no observations).
  double down_avg() const { 
    return (down_cnt > 0) ? (down_sum / static_cast<double>(down_cnt)) : 1.0;
  }
  
  /// @brief Average up-branch cost (default 1.0 if no observations).
  double up_avg() const { 
    return (up_cnt > 0) ? (up_sum / static_cast<double>(up_cnt)) : 1.0;
  }

  /// @brief Variance of down-branch cost (0 if < 2 observations).
  double down_var() const {
    if (down_cnt < 2) return 0.0;
    double n = static_cast<double>(down_cnt);
    double mean = down_sum / n;
    return std::max(0.0, (down_sq - n * mean * mean) / (n - 1.0));
  }
  
  /// @brief Variance of up-branch cost (0 if < 2 observations).
  double up_var() const {
    if (up_cnt < 2) return 0.0;
    double n = static_cast<double>(up_cnt);
    double mean = up_sum / n;
    return std::max(0.0, (up_sq - n * mean * mean) / (n - 1.0));
  }

  /// @brief Lower confidence bound for down-branch (95% CI).
  /// Returns avg - 1.96 * sigma / sqrt(n), clamped to (0, avg].
  double down_lcb() const;
  
  /// @brief Lower confidence bound for up-branch (95% CI).
  double up_lcb() const;

  /// @brief Combined pseudocost score (for variable selection).
  /// Returns product of down/up costs with optional inference/conflict weights.
  double combined_score(double infer_weight = 0.1) const;
};

/// @brief Pseudocost update observation from a single branching decision.
struct PCUpdate {
  int branch_var{-1};
  bool is_up_branch{false};
  bool has_inference{false};
  double inference_count{0.0};
  bool has_gain{false};
  double gain{0.0};
  bool has_cutoff{false};
  int cutoff_count{0};
  double conflict_score{0.0};
};

}  // namespace mipsolvers::engine::detail
