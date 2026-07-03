#pragma once

#include <string>
#include <utility>

#include "hacdcpf/dynamics/DynamicStamp.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/dynamics/solvers/SparseLinearSolver.hpp"

namespace hacdcpf::dynamics {

struct AlgebraicNetworkSolveResult {
  bool success{false};
  std::string message;
};

class AlgebraicNetworkSolver {
 public:
  AlgebraicNetworkSolver() = default;
  explicit AlgebraicNetworkSolver(SparseLinearSolver linear_solver)
      : linear_solver_(std::move(linear_solver)) {}

  [[nodiscard]] AlgebraicNetworkSolveResult solve(DynamicSystem& system,
                                                  double t) const;

  [[nodiscard]] AlgebraicNetworkSolveResult solve(DynamicSystem& system,
                                                  const DynamicStamp& stamp) const;

 private:
  SparseLinearSolver linear_solver_;
};

}  // namespace hacdcpf::dynamics
