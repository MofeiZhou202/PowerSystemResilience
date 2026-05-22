#pragma once

/// backend/nlp_solver.hpp
/// =======================
/// Nonlinear programming (NLP) solver backend interface.
/// Used by the AC OPF and hybrid AC/DC OPF solvers.

#include <Eigen/Core>
#include <string>

namespace hacdcpf::backend {

/// NLP solve status.
enum class NLPStatus {
  Optimal,
  LocalOptimal,
  Infeasible,
  Unbounded,
  MaxIterations,
  NumericalFailure,
  Unknown
};

/// Result of an NLP solve.
struct NLPResult {
  NLPStatus status{NLPStatus::Unknown};
  Eigen::VectorXd x;       ///< Primal solution
  Eigen::VectorXd lambda;  ///< Equality constraint multipliers
  Eigen::VectorXd mu;      ///< Inequality constraint multipliers
  double objective{0.0};
  int iterations{0};
  double primal_infeasibility{0.0};
  double dual_infeasibility{0.0};
  double complementarity{0.0};
  std::string solver_name;
};

/// Options for NLP solvers.
struct NLPOptions {
  double feasibility_tol{1e-6};
  double optimality_tol{1e-6};
  int max_iterations{300};
  bool verbose{false};
};

/// Abstract interface for NLP solvers.
class INLPSolver {
 public:
  virtual ~INLPSolver() = default;
};

}  // namespace hacdcpf::backend
