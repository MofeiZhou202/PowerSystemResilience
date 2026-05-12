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

/// Complementarity equation for PV/PQ switching at a generator bus.
///
/// All PV buses become PQ (Vm free).  The Q-equation is replaced with:
///   F = FB(Qmax - Qg, Vm - Vm_set) + FB(Qg - Qmin, Vm_set - Vm)
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
  return fischer_burmeister(qmax - qg, vm - vm_set) +
         fischer_burmeister(qg - qmin, vm_set - vm);
}

/// Jacobian of PV/PQ NCP equation w.r.t. (Qg, Vm).
/// @return (dF/dQg, dF/dVm).
inline std::pair<double, double> pv_pq_ncp_jacobian(double qg,
                                                     double qmin,
                                                     double qmax,
                                                     double vm,
                                                     double vm_set) {
  const auto [da1, db1] = fischer_burmeister_jacobian(qmax - qg, vm - vm_set);
  const auto [da2, db2] = fischer_burmeister_jacobian(qg - qmin, vm_set - vm);
  // dF/dQg = da1 * d(Qmax-Qg)/dQg + da2 * d(Qg-Qmin)/dQg = -da1 + da2
  const double dF_dQg = -da1 + da2;
  // dF/dVm = db1 * d(Vm-Vm_set)/dVm + db2 * d(Vm_set-Vm)/dVm = db1 - db2
  const double dF_dVm = db1 - db2;
  return {dF_dQg, dF_dVm};
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

/// Smooth PV/PQ NCP residual:
///   F_μ = φ_μ(Qmax−Qg, Vm−Vm_set) + φ_μ(Qg−Qmin, Vm_set−Vm)
///
/// Reduces to pv_pq_ncp() when μ → 0.
inline double pv_pq_smooth_ncp(double qg,
                                double qmin,
                                double qmax,
                                double vm,
                                double vm_set,
                                double mu) {
  return smooth_fb(qmax - qg, vm - vm_set, mu) +
         smooth_fb(qg - qmin, vm_set - vm, mu);
}

/// Jacobian of smooth PV/PQ NCP w.r.t. (Qg, Vm).
/// @return (dF_μ/dQg, dF_μ/dVm).
inline std::pair<double, double> pv_pq_smooth_ncp_jacobian(double qg,
                                                             double qmin,
                                                             double qmax,
                                                             double vm,
                                                             double vm_set,
                                                             double mu) {
  const auto [da1, db1] = smooth_fb_derivatives(qmax - qg, vm - vm_set, mu);
  const auto [da2, db2] = smooth_fb_derivatives(qg - qmin, vm_set - vm, mu);
  // Chain rule: same sign analysis as standard NCP.
  const double dF_dQg = -da1 + da2;
  const double dF_dVm = db1 - db2;
  return {dF_dQg, dF_dVm};
}

}  // namespace hacdcpf::powerflow
