#pragma once

#include <cmath>
#include <utility>

namespace hacdcpf::powerflow {

// ═══════════════════════════════════════════════════════════════════════
// Exact Fischer–Burmeister NCP function
// ═══════════════════════════════════════════════════════════════════════

/// Fischer-Burmeister NCP function: FB(a,b) = sqrt(a² + b²) - a - b.
/// FB(a,b) = 0  ⟺  a ≥ 0, b ≥ 0, ab = 0.
inline double fischer_burmeister(double a, double b) {
  // Facchinei & Pang (2003), vol. I, sec. 9.1: no hidden epsilon belongs in
  // the target NCP. In particular, phi_FB(0, 0) must be exactly zero.
  return std::hypot(a, b) - a - b;
}

/// Jacobian of Fischer-Burmeister w.r.t. (a, b).
/// Returns (dFB/da, dFB/db).
inline std::pair<double, double> fischer_burmeister_jacobian(double a, double b) {
  const double r = std::hypot(a, b);
  if (r == 0.0) {
    // Qi & Sun (1993): choose the symmetric element of the Clarke
    // generalized Jacobian of ||(a,b)|| at the origin.
    constexpr double inv_sqrt_two = 0.70710678118654752440;
    return {inv_sqrt_two - 1.0, inv_sqrt_two - 1.0};
  }
  return {a / r - 1.0, b / r - 1.0};
}

/// Median complementarity equation for PV/PQ switching at a generator bus.
///
/// All PV buses become PQ (Vm free). The Q-equation is replaced with:
///   F = median(Vm - Vm_set, Qg - Qmax, Qg - Qmin)
///
/// When Qmin < Qg < Qmax: forces Vm = Vm_set (PV behavior).
/// When Qg = Qmax: forces Vm ≤ Vm_set (PQ at upper limit).
/// When Qg = Qmin: forces Vm ≥ Vm_set (PQ at lower limit).
///
/// @param qg       Current reactive generation at bus (pu).
/// @param qmin     Minimum reactive limit (pu).
/// @param qmax     Maximum reactive limit (pu).
/// @param vm       Current voltage magnitude at bus (pu).
/// @param vm_set   Voltage setpoint (pu).
/// @return         NCP residual value.
inline double pv_pq_ncp(double qg, double qmin, double qmax, double vm, double vm_set) {
  const double voltage_error = vm - vm_set;
  const double upper_gap = qg - qmax;
  const double lower_gap = qg - qmin;
  return std::max(std::min(voltage_error, lower_gap), upper_gap);
}

/// Jacobian of PV/PQ NCP equation w.r.t. (Qg, Vm).
/// @return (dF/dQg, dF/dVm).
inline std::pair<double, double> pv_pq_ncp_jacobian(double qg,
                                                     double qmin,
                                                     double qmax,
                                                     double vm,
                                                     double vm_set) {
  const double voltage_error = vm - vm_set;
  const double upper_gap = qg - qmax;
  const double lower_gap = qg - qmin;
  const double inner = std::min(voltage_error, lower_gap);

  // A valid element of the Clarke generalized Jacobian is sufficient for the
  // semismooth Newton step. At a tie, average the adjacent one-sided slopes.
  constexpr double tie_tol = 1e-12;
  double inner_dq = 0.0;
  double inner_dv = 0.0;
  if (voltage_error < lower_gap - tie_tol) {
    inner_dv = 1.0;
  } else if (lower_gap < voltage_error - tie_tol) {
    inner_dq = 1.0;
  } else {
    inner_dq = 0.5;
    inner_dv = 0.5;
  }

  if (inner > upper_gap + tie_tol) return {inner_dq, inner_dv};
  if (upper_gap > inner + tie_tol) return {1.0, 0.0};
  return {0.5 * (inner_dq + 1.0), 0.5 * inner_dv};
}

// ═══════════════════════════════════════════════════════════════════════
// Phase 2: smooth NCP continuation primitives
// ═══════════════════════════════════════════════════════════════════════
//
// φ_μ(a, b) = √(a² + b² + 2μ²) − a − b,   μ > 0
//
// As μ → 0 this recovers the standard FB function.
// Using μ > 0 gives a smooth surrogate that avoids non-differentiability
// near a = b = 0, improving convergence when generator or converter limits
// are near-active.
// μ is annealed (ncp_mu0 → ncp_mu_min) as the residual decreases.

/// Smooth Fischer–Burmeister value with smoothing parameter μ.
///
/// @param a   First complementarity argument.
/// @param b   Second complementarity argument.
/// @param mu  Smoothing radius μ ≥ 0 (0 reduces to exact FB).
inline double smooth_fb(double a, double b, double mu) {
  if (mu <= 0.0) return fischer_burmeister(a, b);
  // Kanzow (1996): squaring the radius gives an O(mu) perturbation and
  // matches the CHKS min/max smoothing convention used below.
  return std::sqrt(a * a + b * b + 2.0 * mu * mu) - a - b;
}

/// Partial derivatives of smooth-FB w.r.t. (a, b).
///
/// ∂φ_μ/∂a = a / √(a²+b²+2μ²) − 1
/// ∂φ_μ/∂b = b / √(a²+b²+2μ²) − 1
///
/// @return (∂φ_μ/∂a, ∂φ_μ/∂b).
inline std::pair<double, double> smooth_fb_derivatives(double a, double b, double mu) {
  if (mu <= 0.0) return fischer_burmeister_jacobian(a, b);
  const double r = std::sqrt(a * a + b * b + 2.0 * mu * mu);
  const double safe_r = (r > 1e-30) ? r : 1e-30;
  return {a / safe_r - 1.0, b / safe_r - 1.0};
}

/// CHKS-smoothed median residual.
///
/// Chen, Harker, Kanzow and Smale smoothing replaces
///   max(a,b) by 0.5 * (a+b+sqrt((a-b)^2+4*mu^2))
/// and min analogously. Applying it to
///   max(min(V-Vset, Q-Qmin), Q-Qmax)
/// preserves a fixed equation layout and converges to the exact median NCP as
/// mu -> 0 without a discrete PV/PQ pattern change.
inline double pv_pq_smooth_ncp(double qg,
                                double qmin,
                                double qmax,
                                double vm,
                                double vm_set,
                                double mu) {
  if (mu <= 0.0) return pv_pq_ncp(qg, qmin, qmax, vm, vm_set);
  const double voltage_error = vm - vm_set;
  const double lower_gap = qg - qmin;
  const double upper_gap = qg - qmax;
  const double inner_delta = voltage_error - lower_gap;
  const double inner = 0.5 *
      (voltage_error + lower_gap -
       std::sqrt(inner_delta * inner_delta + 4.0 * mu * mu));
  const double outer_delta = inner - upper_gap;
  return 0.5 *
      (inner + upper_gap +
       std::sqrt(outer_delta * outer_delta + 4.0 * mu * mu));
}

/// Jacobian of smooth PV/PQ NCP w.r.t. (Qg, Vm).
/// @return (dF_μ/dQg, dF_μ/dVm).
inline std::pair<double, double> pv_pq_smooth_ncp_jacobian(double qg,
                                                             double qmin,
                                                             double qmax,
                                                             double vm,
                                                             double vm_set,
                                                             double mu) {
  if (mu <= 0.0) {
    return pv_pq_ncp_jacobian(qg, qmin, qmax, vm, vm_set);
  }
  const double voltage_error = vm - vm_set;
  const double lower_gap = qg - qmin;
  const double upper_gap = qg - qmax;
  const double inner_delta = voltage_error - lower_gap;
  const double inner_radius =
      std::sqrt(inner_delta * inner_delta + 4.0 * mu * mu);
  const double inner =
      0.5 * (voltage_error + lower_gap - inner_radius);
  const double inner_dq = 0.5 * (1.0 + inner_delta / inner_radius);
  const double inner_dv = 0.5 * (1.0 - inner_delta / inner_radius);

  const double outer_delta = inner - upper_gap;
  const double outer_radius =
      std::sqrt(outer_delta * outer_delta + 4.0 * mu * mu);
  const double inner_weight = 0.5 * (1.0 + outer_delta / outer_radius);
  const double upper_weight = 1.0 - inner_weight;
  return {inner_weight * inner_dq + upper_weight,
          inner_weight * inner_dv};
}

}  // namespace hacdcpf::powerflow
