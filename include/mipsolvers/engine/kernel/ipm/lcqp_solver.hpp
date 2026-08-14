#pragma once

#include <string>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/solver_adapter.hpp"

namespace mipsolvers::engine {

/// Options for the native LCQP (Linear Constraints Quadratic Programming) solver.
/// Uses Interior Point Method for convex QP: min 0.5*x'Qx + c'x  s.t. linear constraints.
struct LCQPOptions {
  int max_iter{200};
  double tol_primal{1e-8};
  double tol_dual{1e-8};
  double tol_gap{1e-8};
  /// Cooperative wall-clock limit for one solve. A sparse factorization is
  /// indivisible and may finish after the deadline; 0 disables the limit.
  double time_limit_sec{0.0};
  bool verbose{false};
};

/// Native LCQP solver using Mehrotra predictor-corrector IPM.
/// Solves convex QP problems with Linear Constraints.
class NativeLCQPAdapter final : public SolverAdapter {
 public:
  explicit NativeLCQPAdapter(LCQPOptions opt = {});
  
  std::string name() const override;
  bool supports(ProblemClass cls) const override;
  
  /// Solve QP: min 0.5*x'Qx + c'x  s.t. A*x <= b, Aeq*x = beq, lb <= x <= ub
  SolveResult solve_qp(const QPModel& prob) const override;
  
  /// Can also solve LP as a special case (Q = 0)
  SolveResult solve_lp(const LPModel& prob) const override;

 private:
  LCQPOptions opt_;
  
  /// Internal IPM solve for standardized QP
  SolveResult solve_qp_ipm(
      const Eigen::SparseMatrix<double>& Q,
      const Eigen::VectorXd& c,
      const Eigen::SparseMatrix<double>& A,
      const Eigen::VectorXd& b,
      const Eigen::SparseMatrix<double>& Aeq,
      const Eigen::VectorXd& beq,
      const Eigen::VectorXd& lb,
      const Eigen::VectorXd& ub,
      const Eigen::VectorXd* x0) const;
};

}  // namespace mipsolvers::engine
