#pragma once

// ═══════════════════════════════════════════════════════════════════════════
// Native conic interior-point solver (LP / SOCP / SDP).
//
// Mehrotra predictor-corrector IPM for cvxopt standard form
//   (P) min c'x  s.t.  G x + s = h,  A x = b,  s in K
//   (D) max -h'z - b'y  s.t.  G'z + A'y + c = 0,  z in K
// with K = R^l_+ x prod Q^{q_i} x prod S^{s_i} and Nesterov-Todd scalings,
// following the CVXOPT conelp algorithm.  The Newton (KKT) system is solved
// in the reduced symmetric form K = [G'H^{-1}G + delta I, A'; A, -delta I]
// via CHOLMOD (SPD case) with a MUMPS symmetric-indefinite fallback.
// ═══════════════════════════════════════════════════════════════════════════

#include <string>

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Solver tolerances and controls for ConicIPMSolver.
struct ConicIPMOptions {
  int max_iterations{100};  ///< Maximum number of Newton iterations.
  double abstol{1e-7};      ///< Absolute duality-gap tolerance.
  double reltol{1e-6};      ///< Relative duality-gap tolerance.
  double feastol{1e-7};     ///< Feasibility (primal/dual residual) tolerance.
  int refinement{1};        ///< Iterative-refinement steps on the 3x3 KKT residual
                            ///< (only applied when q/s blocks are present).
  int num_threads{0};       ///< OpenMP threads for parallel regions (0 = library default).
  bool verbose{false};      ///< Print the per-iteration progress table.
};

/// Result of a ConicIPMSolver run.  Objectives are reported in the model's
/// original sense; dual vectors y/z are those of the minimization form.
struct ConicIPMResult {
  std::string status;  ///< "optimal" | "primal infeasible" | "dual infeasible" | "unknown".
  Eigen::VectorXd x;   ///< Primal solution (best iterate).
  Eigen::VectorXd y;   ///< Equality multipliers (minimization form).
  Eigen::VectorXd s;   ///< Primal slack (best iterate).
  Eigen::VectorXd z;   ///< Conic multipliers (minimization form).
  double primal_objective{0.0};      ///< c'x in the model's sense.
  double dual_objective{0.0};        ///< -h'z - b'y in the model's sense.
  double gap{0.0};                   ///< Duality gap s'z.
  double relative_gap{0.0};          ///< gap / max(-pobj, dobj)-style relative measure.
  double primal_infeasibility{0.0};  ///< Scaled primal residual.
  double dual_infeasibility{0.0};    ///< Scaled dual residual.
  int iterations{0};                 ///< Newton iterations performed.
  double runtime_sec{0.0};           ///< Wall-clock time of solve().
};

/// Mehrotra predictor-corrector conic IPM (cvxopt conelp algorithm).
class ConicIPMSolver {
 public:
  explicit ConicIPMSolver(ConicIPMOptions options = {});

  /// Solve a cvxopt-standard-form conic model.  Never throws on numerical
  /// trouble; failures surface as result.status == "unknown".
  [[nodiscard]] ConicIPMResult solve(const ConicModel& model) const;

 private:
  ConicIPMOptions options_;
};

}  // namespace mipsolvers::engine
