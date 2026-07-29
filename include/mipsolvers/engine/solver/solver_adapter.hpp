#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

struct SolveStats {
  bool success{false};
  int iterations{0};
  double objective{0.0};
  double residual_inf{0.0};
  double primal_feas{0.0};
  double dual_feas{0.0};
  double complementarity{0.0};
  double unscaled_primal_feas{0.0};
  double unscaled_dual_feas{0.0};
  double unscaled_complementarity{0.0};
  double mip_gap{0.0};
  double runtime_sec{0.0};
  std::string status;
  std::string solver_name;

  /// Cuts admitted by the CGLP disjunctive cut generator (0 unless native B&C
  /// with enable_cglp_cuts=true is used; propagated from BCStats::cglp_accepted).
  int cglp_cuts_added{0};

  // Infeasibility certificate (Farkas proof).
  // For an infeasible LP  min c'x s.t. Ax <= b, Aeq x = beq, lb <= x <= ub,
  // the Farkas ray y (dual) satisfies:
  //   y >= 0,  A'y + Aeq'y_eq = 0,  b'y + beq'y_eq < 0
  // proving no feasible x exists.
  Eigen::VectorXd farkas_ray;         ///< Dual ray (y for inequality rows)
  Eigen::VectorXd farkas_ray_eq;      ///< Dual ray (y_eq for equality rows)
  bool has_farkas_certificate{false};

  /// Rigorous LP dual bound (minimize convention) certified at an interrupted
  /// solve (time / iteration limit): the exact original-cost dual objective of
  /// an audited dual-feasible point.  NaN when no certified bound exists.  Set
  /// only by the native dual-simplex kernel; `objective` on an interrupted
  /// result remains the (uncertified) working-cost value.
  double certified_dual_bound{std::numeric_limits<double>::quiet_NaN()};
};

struct SolveResult {
  Eigen::VectorXd x;
  SolveStats stats;

  // Constraint dual values (shadow prices) for LP solves, including simplex
  // and IPM-crossover solves when an optimal basis certificate is available.
  // Layout: [inequality duals (m_ineq) | equality duals (m_eq)]
  // Empty if not available (e.g., MILP with integers).
  Eigen::VectorXd constraint_duals;

  // Bound multipliers from LP certificates (one per original variable).
  // box_dual_lb[j] ≈ z_l (multiplier for x_j >= lb_j).
  // box_dual_ub[j] ≈ z_u (multiplier for x_j <= ub_j).
  // Empty when not available (e.g., MILP or a solver path without a
  // certificate).
  Eigen::VectorXd box_dual_lb;
  Eigen::VectorXd box_dual_ub;
};

class SolverAdapter {
 public:
  virtual ~SolverAdapter() = default;

  virtual std::string name() const = 0;
  virtual bool supports(ProblemClass cls) const = 0;

  virtual SolveResult solve_le(const SparseLinSys& prob) const;
  virtual SolveResult solve_nle(const NonlinearSystem& prob) const;
  virtual SolveResult solve_lp(const LPModel& prob) const;
  virtual SolveResult solve_qp(const QPModel& prob) const;
  virtual SolveResult solve_nlp(const NLPModel& prob) const;
  virtual SolveResult solve_milp(const MIPModel& prob) const;
  virtual SolveResult solve_minlp(const MINLPModel& prob) const;
  virtual SolveResult solve_conic(const ConicModel& prob) const;

 protected:
  SolveResult unsupported_result(ProblemClass cls) const;
};

using SolverAdapterPtr = std::shared_ptr<SolverAdapter>;

}  // namespace mipsolvers::engine
