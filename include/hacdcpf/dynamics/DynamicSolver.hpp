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

DynamicResults run_transient_simulation(
    const HybridPowerSystem& sys,
    const DynamicSolverOptions& options = {});

}  // namespace hacdcpf::dynamics
