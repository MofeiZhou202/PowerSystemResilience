#pragma once

#include <complex>
#include <string>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/dynamics/DynamicSolverOptions.hpp"

namespace hacdcpf::dynamics {

struct SparseLinearSolveResult {
  bool success{false};
  std::string message;
};

class SparseLinearSolver {
 public:
  SparseLinearSolver() = default;
  explicit SparseLinearSolver(DynamicLinearSolverType type) : type_(type) {}

  [[nodiscard]] DynamicLinearSolverType type() const noexcept { return type_; }

  [[nodiscard]] SparseLinearSolveResult solve(
      const Eigen::SparseMatrix<double>& a,
      const Eigen::VectorXd& b,
      Eigen::VectorXd& x) const;

  [[nodiscard]] SparseLinearSolveResult solve(
      const Eigen::SparseMatrix<std::complex<double>>& a,
      const Eigen::VectorXcd& b,
      Eigen::VectorXcd& x) const;

 private:
  DynamicLinearSolverType type_{DynamicLinearSolverType::EigenSparseLU};
};

}  // namespace hacdcpf::dynamics
