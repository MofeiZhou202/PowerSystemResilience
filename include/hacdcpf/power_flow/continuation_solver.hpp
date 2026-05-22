#pragma once

/// power_flow/continuation_solver.hpp
/// ====================================
/// Homotopy continuation solver for hybrid AC/DC power flow.
/// Replaces: power_flow/solvers/homotopy_continuation.hpp
///           (and top-level stub power_flow/homotopy_continuation.hpp).

#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::powerflow {

/// State tracking for the homotopy λ-parameter.
struct HomotopyState {
  double lambda{0.0};       ///< Current homotopy parameter ∈ [0, 1].
  double step{0.2};         ///< Current step size Δλ.
  int accepted_steps{0};    ///< Number of successful λ increments.
  int rejected_steps{0};    ///< Number of rejected step attempts.
  bool failed{false};       ///< True if minimum step size was violated.
};

/// Homotopy continuation solver.
///
/// Solves the hybrid power flow by gradually ramping "difficult" injections
/// from a flat-start (flat-voltage, zero-injection) base to the target values.
class HomotopyContinuationSolver {
 public:
  PowerFlowResult solve(const HybridPowerSystem& sys,
                        const PowerFlowOptions& opt) const;

  PowerFlowResult solve(const HybridPowerSystem& sys,
                        const PowerFlowOptions& opt,
                        HomotopyState& homotopy_out) const;
};

}  // namespace hacdcpf::powerflow
