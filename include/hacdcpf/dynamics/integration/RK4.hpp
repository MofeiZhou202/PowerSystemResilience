#pragma once

#include "hacdcpf/dynamics/integration/TimeIntegrator.hpp"

namespace hacdcpf::dynamics {

class RK4 : public TimeIntegrator {
 public:
  [[nodiscard]] const char* name() const noexcept override { return "RK4"; }
  [[nodiscard]] IntegrationStepResult step(DynamicSystem& system,
                                           double t,
                                           double dt) const override;
};

}  // namespace hacdcpf::dynamics

