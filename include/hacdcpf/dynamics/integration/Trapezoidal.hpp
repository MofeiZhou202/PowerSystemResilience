#pragma once

#include "hacdcpf/dynamics/integration/TimeIntegrator.hpp"

namespace hacdcpf::dynamics {

class Trapezoidal : public TimeIntegrator {
 public:
  [[nodiscard]] const char* name() const noexcept override { return "Trapezoidal"; }
  [[nodiscard]] IntegrationStepResult step(DynamicSystem& system,
                                           double t,
                                           double dt) const override;
};

}  // namespace hacdcpf::dynamics

