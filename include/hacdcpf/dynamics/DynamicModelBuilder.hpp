#pragma once

#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

namespace hacdcpf::dynamics {

class DynamicModelBuilder {
 public:
  DynamicSystem build(const HybridPowerSystem& sys,
                      const DynamicSolverOptions& options = {}) const;
};

}  // namespace hacdcpf::dynamics
