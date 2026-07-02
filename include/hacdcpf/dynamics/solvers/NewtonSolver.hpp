#pragma once

#include <functional>
#include <string>

#include <Eigen/Core>

namespace hacdcpf::dynamics {

struct NewtonSolverOptions {
  int max_iterations{20};
  double tolerance{1e-8};
  double min_damping{1e-3};
};

struct NewtonSolverResult {
  bool converged{false};
  int iterations{0};
  double residual_norm{0.0};
  std::string message;
};

class NewtonSolver {
 public:
  using ResidualFunction = std::function<bool(const Eigen::VectorXd&,
                                              Eigen::VectorXd&,
                                              std::string&)>;

  explicit NewtonSolver(NewtonSolverOptions options = {})
      : options_(options) {}

  [[nodiscard]] NewtonSolverResult solve(ResidualFunction residual,
                                         Eigen::VectorXd& x) const;

 private:
  NewtonSolverOptions options_;
};

}  // namespace hacdcpf::dynamics

