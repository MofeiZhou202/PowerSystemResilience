#pragma once

#include <vector>

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/model/system.hpp"

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
///
/// The homotopy path is:
///   P_L(λ)      = λ · P_L^{target}
///   Q_L(λ)      = λ · Q_L^{target}
///   P_{vsc}(λ)  = λ · P_{vsc}^{target}
///   P_{dcdc}(λ) = λ · P_{dcdc}^{target}
///   ...
///
/// At λ = 0 the system is trivially solved (flat start).
/// At λ = 1 the full target system is solved.
///
/// Step adaptation:
///   Success → step ← min(2·step, step_max)
///   Failure → step ← step / 2;  fail if step < step_min
///
/// Typical use: invoked as a last-resort fallback when the direct Newton
/// solve fails.
class HomotopyContinuationSolver {
 public:
  /// Attempt to solve the hybrid power flow via homotopy continuation.
  ///
  /// @param sys      Target power system (full loading/setpoints).
  /// @param opt      Power flow options (tolerance, max_iter, etc.).
  /// @return         Power flow result at λ = 1, or a non-converged result
  ///                 if the homotopy path could not be completed.
  hacdcpf::PowerFlowResult solve(const hacdcpf::HybridPowerSystem& sys,
                                  const hacdcpf::PowerFlowOptions& opt) const;

  /// Same but reports HomotopyState for diagnostics.
  hacdcpf::PowerFlowResult solve(const hacdcpf::HybridPowerSystem& sys,
                                  const hacdcpf::PowerFlowOptions& opt,
                                  HomotopyState& homotopy_out) const;
};

}  // namespace hacdcpf::powerflow
