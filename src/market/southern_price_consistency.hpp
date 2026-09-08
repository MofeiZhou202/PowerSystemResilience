#pragma once

#include "southern_solver_status.hpp"
#include "hacdcpf/engine/engine.hpp"
#include <nlohmann/json.hpp>
#include <limits>

namespace hacdcpf::market::detail {
inline nlohmann::json check_price_duals(const engine::SolveResult& first,
    const engine::SolveResult& repeat, Eigen::Index rows,
    double first_residual, double repeat_residual, double objective_delta) {
  // Exact vector identity is intentional: a tolerance would admit another
  // optimal dual. Original-unit feasibility/objective gates: execution contract,
  // Deterministic Pricing. No objective perturbation or price rounding.
  constexpr double audit_tolerance = 1e-6;
  const bool optimal = first.stats.success && repeat.stats.success &&
    southern_solver_status(first.stats.status,true,first.stats.mip_gap).optimality_proven &&
    southern_solver_status(repeat.stats.status,true,repeat.stats.mip_gap).optimality_proven;
  const bool finite = rows > 0 && first.constraint_duals.size() == rows &&
    repeat.constraint_duals.size() == rows && first.constraint_duals.allFinite() &&
    repeat.constraint_duals.allFinite();
  const double delta = finite ? (first.constraint_duals-repeat.constraint_duals).cwiseAbs().maxCoeff()
    : std::numeric_limits<double>::infinity();
  const bool audited = std::isfinite(first_residual) && first_residual <= audit_tolerance &&
    std::isfinite(repeat_residual) && repeat_residual <= audit_tolerance &&
    std::isfinite(objective_delta) && objective_delta <= audit_tolerance;
  const bool passed = optimal && finite && audited && delta == 0;
  return {{"policy","ordered-lp-dual-simplex-v1"},{"passed",passed},
    {"status",!optimal ? "optimality_not_proven" : !finite ? "missing_or_nonfinite_duals" :
      !audited ? "primal_or_objective_audit_failed" : delta != 0 ? "dual_mismatch" : "exact_match"},
    {"scope","same ordered LP, backend/version and platform; two fresh solves"},
    {"algorithm","dual_simplex"},{"threads",1},{"seed",0},
    {"dual_rows",rows},{"dual_tolerance",0},
    {"max_dual_difference",finite ? nlohmann::json(delta) : nlohmann::json(nullptr)},
    {"objective_difference",std::isfinite(objective_delta) ? nlohmann::json(objective_delta) : nlohmann::json(nullptr)},
    {"repeat_max_residual",std::isfinite(repeat_residual) ? nlohmann::json(repeat_residual) : nlohmann::json(nullptr)},
    {"repeat_solver_status",repeat.stats.status}};
}
} // namespace hacdcpf::market::detail
