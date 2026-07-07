#pragma once

#include "hacdcpf/dynamics/DynamicResults.hpp"
#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::dynamics {

class DynamicSolver {
 public:
  DynamicResults solve(DynamicSystem& system) const;
};

// Compact small-signal (modal) screen about the system's current operating point
// (design doc §18), for attaching to a transient result. Wraps
// small_signal_analysis(); the input state is saved and restored, so it can be
// called on an initialized system before time-stepping without perturbing it.
DynamicModalSummary summarize_small_signal(DynamicSystem& system);

// When system.options.auto_select_stiff_solver is set and solver_type is an
// explicit method, replace it with the cheapest STABLE solver from a cost-ordered
// ladder (PartitionedHeun -> PartitionedRK4 -> TrapezoidalNewton), judged against
// the system's fastest small-signal eigenvalue at dt_s. This downshifts to a
// cheaper explicit method when the operating point is non-stiff and upshifts
// (ultimately to the A-stable implicit method) when it is stiff. Deliberate
// implicit choices are left untouched. Returns a human-readable note describing
// the switch, or an empty string when no change was made.
std::string maybe_auto_select_stiff_solver(DynamicSystem& system);

DynamicResults run_transient_simulation(
    const HybridPowerSystem& sys,
    const DynamicSolverOptions& options = {});

}  // namespace hacdcpf::dynamics
