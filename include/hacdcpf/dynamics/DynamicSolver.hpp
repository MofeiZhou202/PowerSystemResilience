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

DynamicResults run_transient_simulation(
    const HybridPowerSystem& sys,
    const DynamicSolverOptions& options = {});

}  // namespace hacdcpf::dynamics
