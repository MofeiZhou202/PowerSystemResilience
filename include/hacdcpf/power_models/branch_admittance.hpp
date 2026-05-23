#pragma once

/// power_models/branch_admittance.hpp
/// ====================================
/// Shared π-branch admittance computation for ACOPF and ACDCOPF builders.
///
/// Defines the canonical BranchAdmittance struct and branch_admittance()
/// function used by both acopf_builder.cpp and acdcopf_builder.cpp.
/// The formulation follows the convention where the complex tap ratio is
/// τ·e^{−jφ} (tap magnitude τ, phase-shift angle φ in radians):
///
///   Y_ff = (ys + jbc/2) / τ²
///   Y_tt =  ys + jbc/2
///   Y_ft = −ys / (τ · e^{−jφ})  =  −ys · e^{+jφ} / τ
///   Y_tf = −ys / (τ · e^{+jφ})  =  −ys · e^{−jφ} / τ
///
/// where ys = gs + j·bs = 1/(r + jx) is the series admittance.

#include <cmath>

namespace hacdcpf::power_models {

/// Admittance matrix elements for a π-branch with tap ratio and phase shift.
/// Fields Gff/Bff are the from-bus diagonal shunt; Gtt/Btt the to-bus shunt;
/// Gft/Bft the from-to off-diagonal; Gtf/Btf the to-from off-diagonal.
/// All values in per-unit on the system base.
struct BranchAdmittance {
  double Gff{0.0}, Bff{0.0};  // from-from (diagonal, from-bus shunt)
  double Gtt{0.0}, Btt{0.0};  // to-to   (diagonal, to-bus shunt)
  double Gft{0.0}, Bft{0.0};  // from-to (off-diagonal)
  double Gtf{0.0}, Btf{0.0};  // to-from (off-diagonal)
};

/// Compute π-branch admittance elements.
///
/// @param r        Series resistance (pu)
/// @param x        Series reactance  (pu)
/// @param bc       Total line charging susceptance (pu); split as bc/2 per end
/// @param tau      Off-nominal tap ratio magnitude (1.0 for lines)
/// @param phi_rad  Phase-shift angle in radians (0.0 for non-phase-shifting)
/// @return         Populated BranchAdmittance struct
inline BranchAdmittance branch_admittance(double r, double x, double bc,
                                          double tau, double phi_rad) {
  constexpr double kMinImpedanceSq = 1e-12;  // (1e-6)² — clamp threshold
  const double z2 = r * r + x * x;
  // Clamp denominator to avoid 1/0 for zero-impedance branches;
  // bus-merging in the projection layer should eliminate these before OPF.
  const double denom = (z2 < kMinImpedanceSq) ? 1e-6 : z2;
  const double gs = r / denom;
  const double bs = -x / denom;

  const double tau2    = tau * tau;
  const double cos_phi = std::cos(phi_rad);
  const double sin_phi = std::sin(phi_rad);

  BranchAdmittance Y;

  // Diagonal shunt elements
  Y.Gff = gs / tau2;
  Y.Bff = (bs + bc / 2.0) / tau2;
  Y.Gtt = gs;
  Y.Btt = bs + bc / 2.0;

  // Off-diagonal elements: Y_ft = −ys · e^{+jφ} / τ
  //   real(Y_ft) = −(gs·cos_φ − bs·sin_φ) / τ
  //   imag(Y_ft) = −(gs·sin_φ + bs·cos_φ) / τ
  Y.Gft = -(gs * cos_phi - bs * sin_phi) / tau;
  Y.Bft = -(gs * sin_phi + bs * cos_phi) / tau;

  // Off-diagonal elements: Y_tf = −ys · e^{−jφ} / τ
  //   real(Y_tf) = −(gs·cos_φ + bs·sin_φ) / τ
  //   imag(Y_tf) =  (gs·sin_φ − bs·cos_φ) / τ
  Y.Gtf = -(gs * cos_phi + bs * sin_phi) / tau;
  Y.Btf =  (gs * sin_phi - bs * cos_phi) / tau;

  return Y;
}

}  // namespace hacdcpf::power_models
