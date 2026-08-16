#pragma once

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/model/defaults.hpp"

namespace hacdcpf::dynamics::detail {

inline bool evaluate_derivatives(DynamicSystem& system,
                                 double t,
                                 const Eigen::VectorXd& state,
                                 Eigen::VectorXd& dxdt,
                                 std::string& error) {
  return system.evaluateDerivatives(t, state, dxdt, error);
}

inline bool numerical_state_jacobian(DynamicSystem& system,
                                     double t,
                                     const Eigen::VectorXd& state,
                                     const Eigen::VectorXd& f0,
                                     Eigen::MatrixXd& jac,
                                     std::string& error) {
  const Eigen::Index n = state.size();
  jac = Eigen::MatrixXd::Zero(n, n);
  if (n == 0) return true;
  constexpr double eps0 = NumericalConstants::kSqrtMachineEpsilon;
  for (Eigen::Index col = 0; col < n; ++col) {
    Eigen::VectorXd perturbed = state;
    const double h = eps0 * std::max(1.0, std::abs(state[col]));
    perturbed[col] += h;
    Eigen::VectorXd fp;
    if (!evaluate_derivatives(system, t, perturbed, fp, error)) return false;
    jac.col(col) = (fp - f0) / h;
  }
  return true;
}

inline IntegrationStepResult make_failure(std::string message) {
  IntegrationStepResult result;
  result.success = false;
  result.message = std::move(message);
  return result;
}

inline Eigen::SparseMatrix<double> dense_to_sparse(const Eigen::MatrixXd& dense) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(dense.size()));
  for (Eigen::Index j = 0; j < dense.cols(); ++j) {
    for (Eigen::Index i = 0; i < dense.rows(); ++i) {
      const double v = dense(i, j);
      if (v != 0.0) triplets.emplace_back(i, j, v);
    }
  }
  Eigen::SparseMatrix<double> sparse(dense.rows(), dense.cols());
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  return sparse;
}

inline bool sparse_solve(const Eigen::MatrixXd& dense,
                         const Eigen::VectorXd& rhs,
                         Eigen::VectorXd& x,
                         std::string& error) {
  Eigen::SparseMatrix<double> sparse = dense_to_sparse(dense);
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
  lu.compute(sparse);
  if (lu.info() != Eigen::Success) {
    error = "Sparse Newton correction factorization failed";
    return false;
  }
  x = lu.solve(rhs);
  if (lu.info() != Eigen::Success || !x.allFinite()) {
    error = "Sparse Newton correction solve failed";
    return false;
  }
  return true;
}

}  // namespace hacdcpf::dynamics::detail
