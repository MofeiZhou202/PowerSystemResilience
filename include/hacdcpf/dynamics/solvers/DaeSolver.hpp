#pragma once

#include <string>
#include <utility>

#include "hacdcpf/dynamics/integration/TimeIntegrator.hpp"
#include "hacdcpf/dynamics/solvers/AlgebraicNetworkSolver.hpp"

namespace hacdcpf::dynamics {

struct DaeStepResult {
  bool success{false};
  int derivative_evaluations{0};
  int nonlinear_iterations{0};
  std::string message;
};

class DaeSolver {
 public:
  DaeSolver(const TimeIntegrator& integrator,
            AlgebraicNetworkSolver algebraic_solver = AlgebraicNetworkSolver{})
      : integrator_(integrator),
        algebraic_solver_(std::move(algebraic_solver)) {}

  [[nodiscard]] DaeStepResult step(DynamicSystem& system,
                                   double t,
                                   double dt) const;

 private:
  const TimeIntegrator& integrator_;
  AlgebraicNetworkSolver algebraic_solver_;
};

}  // namespace hacdcpf::dynamics
