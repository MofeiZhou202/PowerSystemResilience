#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace hacdcpf::engine {

/// Dogleg trust-region step computation.
///
/// Given Newton step dx_N and gradient g = Jᵀ·F, compute the dogleg step
/// within trust-region radius Δ.
///
/// @param dx_newton  Full Newton step (J⁻¹·F).
/// @param gradient   Steepest descent direction: Jᵀ·F.
/// @param jacobian_grad  J·gradient (for Cauchy step length).
/// @param delta      Trust-region radius.
/// @return           Dogleg step.
inline Eigen::VectorXd dogleg_step(const Eigen::VectorXd& dx_newton,
                                    const Eigen::VectorXd& gradient,
                                    const Eigen::VectorXd& jacobian_grad,
                                    double delta) {
  const double newton_norm = dx_newton.norm();
  if (newton_norm <= delta) {
    return dx_newton;
  }

  // Cauchy step: minimizer of ||F + J·(-α·g)||² along steepest descent.
  const double grad_sq = gradient.squaredNorm();
  const double jg_sq = jacobian_grad.squaredNorm();
  const double alpha_cauchy = (jg_sq > 1e-30) ? (grad_sq / jg_sq) : 0.0;
  Eigen::VectorXd dx_cauchy = -alpha_cauchy * gradient;
  const double cauchy_norm = dx_cauchy.norm();

  if (cauchy_norm >= delta) {
    // Scale Cauchy step to trust-region boundary.
    return (delta / cauchy_norm) * dx_cauchy;
  }

  // Interpolate on dogleg path: dx_cauchy + τ*(dx_newton - dx_cauchy).
  Eigen::VectorXd diff = dx_newton - dx_cauchy;
  const double a = diff.squaredNorm();
  const double b = 2.0 * dx_cauchy.dot(diff);
  const double c = cauchy_norm * cauchy_norm - delta * delta;
  const double disc = b * b - 4.0 * a * c;
  const double tau = (disc > 0.0 && a > 1e-30) ? (-b + std::sqrt(disc)) / (2.0 * a) : 0.0;
  return dx_cauchy + std::clamp(tau, 0.0, 1.0) * diff;
}

/// Compute trust-region ratio ρ and update radius.
///
/// ρ = (||F(x)||² - ||F(x+d)||²) / (||F(x)||² - ||F(x)+J·d||²)
///
/// @param f_norm_sq   ||F(x)||²
/// @param f_new_norm_sq  ||F(x+d)||²
/// @param predicted_reduction  ||F(x)||² - ||F(x)+J·d||²
/// @param delta       Current trust-region radius (updated in-place).
/// @param step_norm   ||d||
/// @param delta_max   Maximum trust-region radius.
/// @return            True if step should be accepted (ρ > 0).
inline bool trust_region_update(double f_norm_sq,
                                 double f_new_norm_sq,
                                 double predicted_reduction,
                                 double& delta,
                                 double step_norm,
                                 double delta_max) {
  const double actual_reduction = f_norm_sq - f_new_norm_sq;
  const double rho = (std::abs(predicted_reduction) > 1e-30)
                         ? (actual_reduction / predicted_reduction)
                         : 0.0;

  if (rho > 0.75 && step_norm > 0.99 * delta) {
    delta = std::min(2.0 * delta, delta_max);
  } else if (rho < 0.25) {
    delta = std::max(0.25 * delta, 1e-10);
  }

  return rho > 0.0;
}

/// Pseudo-transient continuation: modify the Newton step by adding (1/δt)·I to Jacobian.
///
/// The modified system is: (J + (1/δt)·I)·dx = -F(x)
/// As δt → ∞, this recovers pure Newton.
///
/// @param resid_prev  ||F|| at previous iteration.
/// @param resid_curr  ||F|| at current iteration.
/// @param delta_t     Current pseudo-timestep (updated in-place).
/// @param growth      Growth factor for δt (default 2.0).
inline void ptc_update_timestep(double resid_prev,
                                 double resid_curr,
                                 double& delta_t,
                                 double growth) {
  if (resid_curr > 1e-30) {
    // SER (Switched Evolution/Relaxation) formula.
    delta_t *= (resid_prev / resid_curr);
  } else {
    delta_t *= growth;
  }
  // Cap to prevent numerical issues with very large δt.
  delta_t = std::min(delta_t, 1e12);
}

}  // namespace hacdcpf::engine
