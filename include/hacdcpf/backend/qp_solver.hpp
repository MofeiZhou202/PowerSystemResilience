#pragma once

/// backend/qp_solver.hpp
/// ======================
/// Quadratic programming (QP) solver backend interface.
/// Used by the DC OPF (NativeQP path) and parity IPM for LP sub-problems.

#include <Eigen/Core>
#include <string>
#include <vector>

namespace hacdcpf::backend {

/// QP solve status.
enum class QPStatus {
  Optimal,
  Infeasible,
  Unbounded,
  MaxIterations,
  NumericalFailure,
  Unknown
};

/// Result of a QP solve.
struct QPResult {
  QPStatus status{QPStatus::Unknown};
  Eigen::VectorXd primal;   ///< Primal solution x*
  Eigen::VectorXd dual;     ///< Dual variables (constraint multipliers)
  double objective{0.0};
  int iterations{0};
  std::string solver_name;
};

/// Options for the QP solver.
struct QPOptions {
  double feasibility_tol{1e-8};
  double optimality_tol{1e-8};
  int max_iterations{10000};
  bool verbose{false};
};

/// Abstract interface for QP solvers.
class IQPSolver {
 public:
  virtual ~IQPSolver() = default;

  /// Solve: min 0.5 x'Hx + c'x  s.t. Ax <= b, lb <= x <= ub
  virtual QPResult solve(
      const Eigen::SparseMatrix<double>& H,
      const Eigen::VectorXd& c,
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const Eigen::VectorXd& lb,
      const Eigen::VectorXd& ub,
      const QPOptions& opts = {}) = 0;
};

}  // namespace hacdcpf::backend
