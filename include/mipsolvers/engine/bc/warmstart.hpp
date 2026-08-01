#pragma once
/// \file warmstart.hpp
/// \brief Warm-start payload for the native branch-and-cut engine.
///
/// A `BCWarmStart` seeds primal incumbents for MILP or an initial point for
/// MINLP. Every supplied vector is dimension-checked and passes through the
/// solver's normal feasibility checks before it can become an incumbent.

#include <vector>

#include <Eigen/Core>

namespace mipsolvers::engine {

/// A primal solution in original-variable space.
struct BCPrimalHint {
  Eigen::VectorXd x;
};

/// Supported warm-start payload.
struct BCWarmStart {
  std::vector<BCPrimalHint> primal_hints;

  /// True when no payload is attached (fast short-circuit in the solver).
  bool empty() const { return primal_hints.empty(); }
};

}  // namespace mipsolvers::engine
