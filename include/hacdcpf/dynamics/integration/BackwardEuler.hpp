#pragma once

#include "hacdcpf/dynamics/integration/TimeIntegrator.hpp"

namespace hacdcpf::dynamics {

class BackwardEuler : public TimeIntegrator {
 public:
  [[nodiscard]] const char* name() const noexcept override { return "BackwardEuler"; }
  [[nodiscard]] IntegrationStepResult step(DynamicSystem& system,
                                           double t,
                                           double dt) const override;
};

}  // namespace hacdcpf::dynamics

