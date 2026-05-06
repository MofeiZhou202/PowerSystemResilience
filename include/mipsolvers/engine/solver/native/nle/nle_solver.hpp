#pragma once

/// @file nle_solver.hpp
/// @brief General-purpose Newton solver for nonlinear equations F(x) = 0.
///
/// Domain-agnostic: the caller provides residual and Jacobian callbacks.
/// Uses dogleg trust-region globalization (see globalization.hpp).

#include <functional>

#include <Eigen/Core>
#include <Eigen/Sparse>

namespace mipsolvers::engine {

/// Status codes returned by NLESolver.
enum class NLEStatus {
  Success,
  MaxIterationsReached,
  SingularJacobian,
  LinearSolveFailed,
  TrustRegionTooSmall,
};

/// Options for the general-purpose NLE solver.
struct NLEOptions {
  double tol       = 1e-8;   ///< Convergence tolerance: ‖F(x)‖ < tol
  int    max_iters = 200;    ///< Maximum Newton iterations
};

/// Problem specification: initial point + callbacks for F(x) and J(x).
struct NLEProblem {
  Eigen::VectorXd x0;  ///< Initial iterate

  /// Residual F(x) → R^n
  std::function<Eigen::VectorXd(const Eigen::VectorXd&)> residual;

  /// Jacobian J(x) → R^{n×n} (sparse, column-major)
  std::function<Eigen::SparseMatrix<double>(const Eigen::VectorXd&)> jacobian;
};

/// Result of an NLE solve.
struct NLEResult {
  Eigen::VectorXd x;            ///< Final iterate
  double          residual_norm{0.0};
  int             iterations{0};
  NLEStatus       status{NLEStatus::MaxIterationsReached};

  bool converged() const { return status == NLEStatus::Success; }
};

/// General-purpose Newton solver for nonlinear systems F(x) = 0.
///
/// This solver is domain-agnostic. For the power-flow-specific NR solver
/// (with AC/DC bus modelling, PV/PQ switching, etc.) see NewtonSolver.
class NLESolver {
 public:
  NLEResult solve(const NLEProblem& prob,
                  const NLEOptions& opt = {}) const;
};

}  // namespace mipsolvers::engine
