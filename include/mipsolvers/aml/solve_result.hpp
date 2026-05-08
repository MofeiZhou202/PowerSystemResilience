#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "mipsolvers/aml/constraint.hpp"
#include "mipsolvers/aml/ids.hpp"
#include "mipsolvers/aml/key.hpp"
#include "mipsolvers/aml/variable.hpp"

namespace mipsolvers::aml {

// ════════════════════════════════════════════════════════════════════════════
// Status enumerations
// ════════════════════════════════════════════════════════════════════════════

/// Why the solver stopped.
enum class TerminationStatus {
  Optimal,
  Infeasible,
  Unbounded,
  InfeasibleOrUnbounded,
  TimeLimit,        ///< stopped at time limit; may have a feasible incumbent
  IterationLimit,
  NodeLimit,
  ObjectiveLimit,
  NumericalError,
  UserInterrupt,
  SolverError,
  Unknown,
};

/// What kind of primal solution is available.
enum class PrimalStatus {
  Optimal,    ///< proven globally optimal
  Feasible,   ///< feasible but not proven optimal
  Infeasible, ///< no primal exists
  NoSolution, ///< solver found nothing
  Unknown,
};

/// What kind of dual solution is available.
enum class DualStatus {
  Optimal,    ///< LP dual at optimum
  Feasible,   ///< LP dual feasible but not optimal
  Infeasible, ///< dual infeasible (primal unbounded)
  NoSolution, ///< not available: MILP, NLP, or unavailable
  Unknown,
};

// ════════════════════════════════════════════════════════════════════════════
// SolveResult
// ════════════════════════════════════════════════════════════════════════════
class SolveResult {
 public:
  TerminationStatus termination_status = TerminationStatus::Unknown;
  PrimalStatus      primal_status      = PrimalStatus::Unknown;
  DualStatus        dual_status        = DualStatus::Unknown;

  // ---- Status helpers ----------------------------------------------------
  [[nodiscard]] bool has_primal() const noexcept {
    return primal_status == PrimalStatus::Optimal ||
           primal_status == PrimalStatus::Feasible;
  }
  [[nodiscard]] bool is_optimal() const noexcept {
    return termination_status == TerminationStatus::Optimal &&
           primal_status      == PrimalStatus::Optimal;
  }
  [[nodiscard]] bool has_duals() const noexcept {
    return dual_status == DualStatus::Optimal ||
           dual_status == DualStatus::Feasible;
  }

  // ---- Objective metrics -------------------------------------------------
  double objective_value    = 0.0;
  double objective_bound    = 0.0;
  double optimality_gap     = 1.0;  ///< |obj - bound| / max(1, |obj|)

  // ---- Statistics --------------------------------------------------------
  double      solve_time_sec       = 0.0;
  int         simplex_iterations   = 0;
  int         branch_and_cut_nodes = 0;
  std::string solver_used;

  // ---- Primal solution storage -------------------------------------------
  // Populated by ModelImpl::extract_solution() after the native solve.
  std::unordered_map<VarId, double> primal_vals;

  // ---- Dual solution storage (LP only) -----------------------------------
  std::unordered_map<ConId, double> dual_vals;
  std::unordered_map<VarId, double> rc_vals;  ///< reduced costs

  // ---- Primal access helpers ---------------------------------------------
  [[nodiscard]] double var_value(VarId vid) const {
    auto it = primal_vals.find(vid);
    if (it == primal_vals.end()) return 0.0;
    return it->second;
  }
  [[nodiscard]] double var_value(const VarRef& v) const {
    return var_value(v.id());
  }
  [[nodiscard]] double var_value(const VarArray& a, const Key& k) const {
    return var_value(a(k).id());
  }
  [[nodiscard]] double var_value(const VarArray& a, const Atom& at) const {
    return var_value(a(at).id());
  }
  [[nodiscard]] double var_value(const VarArray& a,
                                  const Atom& a1, const Atom& a2) const {
    return var_value(a(Key::pair(a1, a2)).id());
  }

  // ---- Dual access helpers -----------------------------------------------
  [[nodiscard]] std::optional<double> dual(ConId cid) const {
    if (!has_duals()) return std::nullopt;
    auto it = dual_vals.find(cid);
    if (it == dual_vals.end()) return std::nullopt;
    return it->second;
  }
  [[nodiscard]] std::optional<double> dual(const ConstraintRef& c) const {
    return dual(c.id());
  }
  [[nodiscard]] std::optional<double> dual(const ConstraintArray& a,
                                            const Key& k) const {
    return dual(a(k).id());
  }
  [[nodiscard]] std::optional<double> dual(const ConstraintArray& a,
                                            const Atom& at) const {
    return dual(a(at).id());
  }
  [[nodiscard]] std::optional<double> reduced_cost(const VarRef& v) const {
    if (!has_duals()) return std::nullopt;
    auto it = rc_vals.find(v.id());
    if (it == rc_vals.end()) return std::nullopt;
    return it->second;
  }

  // ---- IIS data ----------------------------------------------------------
  struct IISData {
    std::vector<ConstraintRef> constraints;
    std::vector<VarRef>        bound_violations;
  };
  std::optional<IISData> iis;
};

}  // namespace mipsolvers::aml
