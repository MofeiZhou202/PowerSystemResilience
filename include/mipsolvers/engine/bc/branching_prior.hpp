#pragma once
/// \file branching_prior.hpp
/// \brief User-supplied branching priors / branching "inspirations".
///
/// The B&C engine normally selects branching variables via pseudocost or
/// strong branching. This header defines an orthogonal extension point
/// where the caller can inject problem-specific hints — typically derived
/// from:
///
///   * Machine-learning policies (GNN / tree-search value models),
///   * Known structural dominance relations (e.g. UC minimum-up-time),
///   * Benders master-problem cut coefficients,
///   * Prior runs on similar instances (transfer learning).
///
/// The engine treats the prior as a *soft* hint: ties are broken by prior
/// score, but a zero-prior variable can still be selected if pseudocost
/// estimates strongly favor it. Variables absent from the prior receive
/// score 0.0.

#include <cstddef>
#include <functional>
#include <vector>

namespace mipsolvers::engine {

/// Per-variable branching-prior scores in [-inf, +inf]; larger ⇒ branch first.
struct BCBranchingPrior {
  /// Static scores, one per *original-space* variable. Empty ⇒ no static prior.
  std::vector<double> static_scores;

  /// Preferred rounding direction per variable: +1 up, -1 down, 0 engine-chosen.
  std::vector<int> preferred_direction;

  /// Relative weight of this prior against pseudocost (0 disables, 1 equal).
  double weight{0.5};

  /// True when no prior data is attached.
  bool empty() const {
    return static_scores.empty() && preferred_direction.empty();
  }
};

/// Lightweight snapshot passed to dynamic-prior callbacks.
struct BCBranchContext {
  int    node_id{-1};
  int    depth{0};
  double node_lb{-1e30};
  double best_obj{1e30};

  /// Candidate variables that are currently fractional.
  const std::vector<int>* candidates{nullptr};

  /// Current LP relaxation primal vector, in original-variable space.
  const double* lp_x{nullptr};
  std::size_t   lp_x_size{0};
};

/// Signature of a dynamic branching-prior callback.
///
/// Given the current node context and a list of candidate variables, the
/// callback may fill `out_scores` (same size as `candidates`) with its
/// per-candidate preference. Unset entries default to 0.0.
using BCBranchingPriorFn =
    std::function<void(const BCBranchContext& ctx, std::vector<double>& out_scores)>;

}  // namespace mipsolvers::engine
