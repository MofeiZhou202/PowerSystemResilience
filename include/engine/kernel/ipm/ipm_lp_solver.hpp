#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/engine/problem_types.hpp"
#include "hacdcpf/engine/solver/solver_adapter.hpp"

namespace hacdcpf::engine {

/// Options for the high-performance Interior Point Method LP solver.
///
/// This is a Mehrotra predictor-corrector IPM designed for general LP:
///   min c'x  s.t.  A x <= b,  Aeq x = beq,  lb <= x <= ub
///
/// Key features over LCQP solver:
///   - Full general inequality support (A x <= b via slack variables)
///   - Ruiz + Pock-Chambolle equilibration for better conditioning
///   - Mehrotra predictor-corrector with Gondzio correctors
///   - Multiple centrality corrector steps for faster convergence
///   - Crossover to basic feasible solution for MILP warm-starting
///   - Constraint duals returned for cut generation
///
/// Suitable as the LP relaxation solver in Branch & Cut.
struct IPMLPOptions {
  int max_iter{200};          ///< Maximum IPM iterations
  double tol_primal{1e-8};   ///< Primal feasibility tolerance
  double tol_dual{1e-8};     ///< Dual feasibility tolerance
  double tol_gap{1e-8};      ///< Complementarity gap tolerance
  int ruiz_rounds{10};       ///< Ruiz equilibration rounds (0 = disable)
  int max_correctors{3};     ///< Max Gondzio centering correctors per iteration
  bool presolve{true};       ///< Enable presolve (fixed vars, singleton rows, etc.)
  bool crossover{false};     ///< Purify solution to a vertex (for simplex warm-start)
  bool verbose{false};       ///< Print per-iteration log
};

/// Forward declaration for Apple Accelerate sparse Cholesky cache.
struct AccelSparseCache;

/// Native high-performance IPM for LP.
///
/// Converts general LP (inequality + equality + box) to standard form
/// internally and solves via Mehrotra predictor-corrector with Gondzio
/// centering corrections.  Returns constraint duals and Farkas certificates.
class NativeIPMLPAdapter final : public SolverAdapter {
 public:
  explicit NativeIPMLPAdapter(IPMLPOptions opt = {});
  ~NativeIPMLPAdapter();

  std::string name() const override;
  bool supports(ProblemClass cls) const override;

  SolveResult solve_lp(const LPModel& prob) const override;

  /// Solve LP with warm-start from a previous primal solution.
  /// When x0 is provided, the IPM uses it as initial primal point instead
  /// of the expensive least-squares initialization, typically reducing
  /// iteration count by 30-50%.
  SolveResult solve_lp(const LPModel& prob, const Eigen::VectorXd& x0) const;

  /// Solve a node LP: base_lp with overridden variable bounds.
  /// More efficient than copying the LPModel: directly uses node_lb/node_ub
  /// instead of modifying the vars array.
  SolveResult solve_node_lp(const LPModel& base_lp,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub) const;

  /// Solve a node LP with warm-start from parent node's solution.
  SolveResult solve_node_lp(const LPModel& base_lp,
                            const Eigen::VectorXd& node_lb,
                            const Eigen::VectorXd& node_ub,
                            const Eigen::VectorXd& x0) const;

  /// Prepare cached solver state for repeated node solves on the same LP.
  /// Caches CSR, scatter maps, symbolic factorization — only bounds change.
  void prepare_for_node_solves(const LPModel& base_lp);

  /// Fast node LP solve using cached infrastructure from prepare_for_node_solves.
  /// Only updates bounds and warm-start, skipping all structural setup.
  SolveResult solve_cached_node_lp(const Eigen::VectorXd& node_lb,
                                    const Eigen::VectorXd& node_ub,
                                    const Eigen::VectorXd& x0) const;

  /// Update the cached cost vector (for feasibility pump objective changes).
  /// new_c is in the original LP convention (sense_sign applied internally).
  /// Length must be n_orig. Slack costs remain 0.
  void update_cached_cost(const Eigen::VectorXd& new_c);

 private:
  IPMLPOptions opt_;
  mutable std::unique_ptr<AccelSparseCache> accel_cache_;

  /// Cached solver state for repeated node solves.
  struct CachedState;
  mutable std::unique_ptr<CachedState> cached_state_;
};

}  // namespace hacdcpf::engine
