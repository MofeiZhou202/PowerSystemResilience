#pragma once

#include <string>

namespace hacdcpf::dynamics {

class DynamicSystem;

struct IntegrationStepResult {
  bool success{false};
  int derivative_evaluations{0};
  int nonlinear_iterations{0};
  std::string message;
};

class TimeIntegrator {
 public:
  virtual ~TimeIntegrator() = default;

  [[nodiscard]] virtual const char* name() const noexcept = 0;

  [[nodiscard]] virtual IntegrationStepResult step(DynamicSystem& system,
                                                   double t,
                                                   double dt) const = 0;
};

}  // namespace hacdcpf::dynamics

