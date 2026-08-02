#pragma once

#include <cmath>
#include <utility>

namespace hacdcpf::powerflow {

// ═══════════════════════════════════════════════════════════════════════
// Standard (near-zero-smoothed) Fischer–Burmeister NCP function
// ═══════════════════════════════════════════════════════════════════════

/// Fischer-Burmeister NCP function: FB(a,b) = sqrt(a² + b²) - a - b.
/// FB(a,b) = 0  ⟺  a ≥ 0, b ≥ 0, ab = 0.
inline double fischer_burmeister(double a, double b) {
  const double eps = 1e-14;
  return std::sqrt(a * a + b * b + eps) - a - b;
}

/// Jacobian of Fischer-Burmeister w.r.t. (a, b).
/// Returns (dFB/da, dFB/db).
inline std::pair<double, double> fischer_burmeister_jacobian(double a, double b) {
  const double eps = 1e-14;
  const double r = std::sqrt(a * a + b * b + eps);
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
// Phase 2: Smooth (μ-perturbed) Fischer–Burmeister NCP continuation
// ═══════════════════════════════════════════════════════════════════════
//
// φ_μ(a, b) = √(a² + b² + 2μ) − a − b,   μ > 0
//
// As μ → 0 this recovers the standard FB function.
// Using μ > 0 gives a smooth surrogate that avoids non-differentiability
// near a = b = 0, improving convergence when Q limits are near-active.
// μ is annealed (ncp_mu0 → ncp_mu_min) as the residual decreases.

/// Smooth Fischer–Burmeister value with smoothing parameter μ.
///
/// @param a   First complementarity argument.
/// @param b   Second complementarity argument.
/// @param mu  Smoothing parameter μ ≥ 0 (0 reduces to standard near-zero FB).
inline double smooth_fb(double a, double b, double mu) {
  return std::sqrt(a * a + b * b + 2.0 * mu) - a - b;
}

/// Partial derivatives of smooth-FB w.r.t. (a, b).
///
/// ∂φ_μ/∂a = a / √(a²+b²+2μ) − 1
/// ∂φ_μ/∂b = b / √(a²+b²+2μ) − 1
///
/// @return (∂φ_μ/∂a, ∂φ_μ/∂b).
inline std::pair<double, double> smooth_fb_derivatives(double a, double b, double mu) {
  const double r = std::sqrt(a * a + b * b + 2.0 * mu);
  const double safe_r = (r > 1e-30) ? r : 1e-30;
  return {a / safe_r - 1.0, b / safe_r - 1.0};
}

/// Semismooth median residual. The median formulation already has the required
/// physical zero set for every iterate, so continuation does not perturb it.
inline double pv_pq_smooth_ncp(double qg,
                                double qmin,
                                double qmax,
                                double vm,
                                double vm_set,
                                double mu) {
  (void)mu;
  return pv_pq_ncp(qg, qmin, qmax, vm, vm_set);
}

/// Jacobian of smooth PV/PQ NCP w.r.t. (Qg, Vm).
/// @return (dF_μ/dQg, dF_μ/dVm).
inline std::pair<double, double> pv_pq_smooth_ncp_jacobian(double qg,
                                                             double qmin,
                                                             double qmax,
                                                             double vm,
                                                             double vm_set,
                                                             double mu) {
  (void)mu;
  return pv_pq_ncp_jacobian(qg, qmin, qmax, vm, vm_set);
}

}  // namespace hacdcpf::powerflow
