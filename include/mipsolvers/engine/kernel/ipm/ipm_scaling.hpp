#pragma once

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine {

/// Gradient-based scaling factors (IPOPT-style "gradient-based" scaling).
///
/// Given ∇f, ∇g, ∇h evaluated at the interiorized initial point and a target
/// caller-selected `g_max`; the Native IPM's automatic policy uses the unit
/// normalization target. Each scale factor is chosen so that the
/// infinity-norm of the corresponding (scaled) gradient / row is bounded by
/// g_max:
///   s_f   = min(1, g_max / ‖∇f(x₀)‖_∞)
///   s_g,i = min(1, g_max / ‖row_i(∇g(x₀))‖_∞)
///   s_h,i = min(1, g_max / ‖row_i(∇h(x₀))‖_∞)
///
/// With these factors the scaled problem reads
///   min  s_f · f(x)   s.t.   S_g · g(x) = 0,   S_h · h(x) ≤ 0,   x_L ≤ x ≤ x_U
/// and the scaled Lagrangian Hessian is simply s_f · ∇²L(x, λ, μ_nonlin)
/// with λ = (S_g/s_f) λ_s, μ_nonlin = (S_h,nonlin/s_f) μ_s.
struct ScalingFactors {
  double s_f{1.0};
  Eigen::VectorXd s_g;   // size m_eq   (empty if no equalities)
  Eigen::VectorXd s_h;   // size m_ineq (empty if no nonlinear inequalities)
};

/// Compute scale factors at x0. Passing an x0 outside the variable bounds is
/// permitted; caller is expected to have interiorized already. Safe on NLPs
/// without a callback for f/g/h (the corresponding factor set stays empty).
ScalingFactors compute_scaling_factors(const NLPModel& prob,
                                       const Eigen::VectorXd& x0,
                                       double g_max);

/// Build a new NLPModel that wraps `prob` with the scale factors baked into
/// its callbacks. The returned model:
///   - owns captures to the original callbacks via std::function,
///   - exposes scaled callbacks (objective, gradient, constraints, Jacobians,
///     Hessian) that internally call the originals and scale their outputs,
///   - has identical `vars`, `x0`, `sense`, `symbolic_*` fields as `prob` so
///     bound handling and validation are unaffected.
///
/// The caller must keep `prob` alive for the lifetime of the returned model
/// because the lambdas capture `prob` by reference.
NLPModel build_scaled_nlp_model(const NLPModel& prob,
                                const ScalingFactors& factors);

}  // namespace mipsolvers::engine
