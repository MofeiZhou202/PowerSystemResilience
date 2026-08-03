/// @file bc_legacy_entry_helpers.hpp
/// @brief Warm-start install + callback-validation helpers for the legacy
/// solve_*_bc_legacy_core entry points (extracted from branch_and_cut.cpp).

#pragma once

#include <limits>
#include <stdexcept>

#include "mipsolvers/engine/branch_and_cut.hpp"

namespace mipsolvers::engine::detail {

inline MIPModel install_primal_warm_start(const MIPModel& prob,
                                   const BCWarmStart& ws) {
  MIPModel solve_prob = prob;
  if (ws.primal_hints.empty()) return solve_prob;

  const int n = static_cast<int>(prob.linear_part.c.size());
  bool selected = false;
  double selected_obj = prob.linear_part.sense == Sense::Minimize
                            ? std::numeric_limits<double>::infinity()
                            : -std::numeric_limits<double>::infinity();
  for (const BCPrimalHint& hint : ws.primal_hints) {
    if (hint.x.size() != n || !hint.x.allFinite()) {
      throw std::invalid_argument(
          "BCWarmStart primal hint must be finite and match the original "
          "MILP column count");
    }
    const double objective = prob.linear_part.c.dot(hint.x);
    const bool better = !selected ||
        (prob.linear_part.sense == Sense::Minimize
             ? objective < selected_obj
             : objective > selected_obj);
    if (better) {
      solve_prob.initial_solution = hint.x;
      selected_obj = objective;
      selected = true;
    }
  }
  return solve_prob;
}

inline MINLPModel install_primal_warm_start(const MINLPModel& prob,
                                     const BCWarmStart& ws) {
  MINLPModel solve_prob = prob;
  if (ws.primal_hints.empty()) return solve_prob;

  const int n = static_cast<int>(prob.nonlinear_part.vars.size());
  for (const BCPrimalHint& hint : ws.primal_hints) {
    if (hint.x.size() != n || !hint.x.allFinite()) {
      throw std::invalid_argument(
          "BCWarmStart primal hint must be finite and match the original "
          "MINLP variable count");
    }
  }
  solve_prob.nonlinear_part.x0 = ws.primal_hints.front().x;
  return solve_prob;
}

inline void validate_milp_callbacks(const BCCallbacks& cbs,
                             const BCOptions& opt) {
  if (cbs.branching_prior && opt.strict_highs_mip_contract) {
    throw std::invalid_argument(
        "BC branching_prior cannot control the strict HiGHS MIP tree; "
        "install static MIP branching priorities instead");
  }
  if (cbs.dynamic_node_cut && !opt.strict_highs_mip_contract) {
    throw std::invalid_argument(
        "BC dynamic_node_cut is supported only by the strict HiGHS MIP "
        "node lifecycle");
  }
}

inline void validate_minlp_callbacks(const BCCallbacks& cbs) {
  if (cbs.branching_prior || cbs.dynamic_node_cut) {
    throw std::invalid_argument(
        "MINLP currently supports hyperparam_tuner and post_solve callbacks "
        "only");
  }
}

}  // namespace mipsolvers::engine::detail
