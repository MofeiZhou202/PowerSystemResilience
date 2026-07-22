#pragma once

/// power_flow/lcc_model.hpp
/// ==========================
/// LCC (line-commutated converter) quasi-steady station model for the unified
/// Newton power flow (dat card manual ch.4, BD/LD cards).
///
/// The station is evaluated from the CURRENT Newton iterate (valve-side AC
/// voltage magnitude and DC terminal voltage), exactly like the VSC injection
/// helpers in converter_model.hpp: the injections enter p_spec/q_spec/pdc_spec
/// and the DC-side derivative enters the always-on DC self-consistency
/// Jacobian.  Per station (E = valve-side line voltage, X_c per phase):
///   U_d0 = (3*sqrt(2)/pi) * n_bridges * E
///   rectifier: U_d = U_d0*cos(alpha) - (3/pi)*n_b*X_c*I_d - n_b*dU_v
///   inverter:  U_d = U_d0*cos(gamma) - (3/pi)*n_b*X_c*I_d + n_b*dU_v
///   Q = P * tan(phi),  cos(phi) ~= U_d / U_d0
///
/// Fixed-tap honesty note: no converter-transformer tap control is modelled.
/// With the tap fixed at its nominal ratio the valve-side voltage — and hence
/// U_d0 — floats with the AC solution, so the back-calculated firing /
/// extinction angles depart from the scheduled AlphaN/GamaN that a tap
/// changer would hold; the gap against a tapped reference solution (e.g.
/// DSP) is a modelling limitation, reported in LCCTransfer /
/// model_limitations.
///
/// Current-limit note: the DC current is clamped to [0, rated_current_a]
/// (LCC current cannot reverse; the rated current is also the link's current
/// order).  When the clamp binds at the solution, a CEA / constant-alpha
/// setpoint is NOT held — the station degenerates to constant-current
/// behaviour and gamma/alpha float (gamma stays above gamma_min, so
/// commutation margin is preserved).  This is reported via
/// LCCTransfer::id_at_limit and a [LCC-PHYS-03] diagnostic, never silently.

#include <utility>

#include <Eigen/Core>

#include "hacdcpf/assembly/solver_data.hpp"

namespace hacdcpf::powerflow {

/// Operating point of one LCC station evaluated at a given AC/DC voltage
/// state.  Units: kV, kA, MW, Mvar, degrees.  Sign conventions match the
/// solver injection convention:
///   p_ac_mw   AC-side active power injected INTO the AC grid
///             (rectifier negative = absorbs; inverter positive = delivers).
///   q_ac_mvar AC-side reactive power injected into the AC grid
///             (an LCC always consumes reactive power -> negative).
///   p_dc_mw   DC-side power injected INTO the DC network
///             (rectifier positive; inverter negative).
struct LCCOperatingPoint {
  double ud0_kv{0.0};      // ideal no-load DC voltage (3*sqrt(2)/pi)*n_b*E
  double ud_kv{0.0};       // DC terminal voltage used for the evaluation
  double id_ka{0.0};       // DC current (magnitude, positive in power direction)
  double alpha_deg{0.0};   // firing angle (rectifier: back-calculated)
  double gamma_deg{0.0};   // extinction angle (inverter: control value / back-calc)
  double p_ac_mw{0.0};
  double q_ac_mvar{0.0};
  double p_dc_mw{0.0};
  bool id_at_limit{false};        // current clamped at rated_current_a
  bool alpha_beyond_range{false}; // cos(alpha) argument outside [-1,1]:
                                  // even alpha = 0 cannot reach U_d (tap needed)
  bool valid{false};              // false when inputs/config were unusable
};

/// Evaluate a station's operating point at the given valve-side AC line
/// voltage @p e_kv (kV) and DC terminal voltage @p ud_kv (kV).
/// Dispatches on station_role + control_mode:
///   Rectifier: ConstantPower / ConstantCurrent / ConstantAlpha.
///   Inverter:  ConstantGamma / ConstantPower / ConstantCurrent.
/// Unsupported role/mode combinations return valid=false (callers route this
/// to diagnostics; the station then injects nothing).
LCCOperatingPoint lcc_operating_point(const LCCConverter& conv,
                                      double e_kv,
                                      double ud_kv);

/// DC bus voltage base (kV) used to convert the solved vdc pu of the
/// station's DC bus to kV: DCBus.base_kv, falling back to rated_dc_kv.
double lcc_dc_base_kv(const SolverData& data, const LCCConverter& conv);

/// AC-side injection (p_pu, q_pu) on data.base_mva at the current iterate.
/// (0,0) for out-of-service stations, unresolvable buses or invalid configs.
std::pair<double, double> lcc_ac_injection(const SolverData& data,
                                           const LCCConverter& conv,
                                           const Eigen::VectorXd& vm,
                                           const Eigen::VectorXd& vdc);

/// DC-side injection (pu on data.base_mva) at the current iterate.
double lcc_dc_injection(const SolverData& data,
                        const LCCConverter& conv,
                        const Eigen::VectorXd& vm,
                        const Eigen::VectorXd& vdc);

/// d(pdc_pu)/d(vdc_pu) of the station's DC injection at the current iterate
/// (the AC-voltage dependence of U_d0 is treated Picard-style and not
/// differentiated).  Non-zero for the characteristic control modes
/// (ConstantGamma / ConstantAlpha, voltage-source-behind-X_c behaviour) and
/// for constant-current behaviour (ConstantCurrent mode or an active current
/// clamp, where the injection degenerates to ±U_d*I); zero for
/// ConstantPower.
double lcc_dc_jacobian_vdc(const SolverData& data,
                           const LCCConverter& conv,
                           const Eigen::VectorXd& vm,
                           const Eigen::VectorXd& vdc);

/// Full per-station Jacobian of the AC/DC injections at the current iterate,
/// all in pu/pu on data.base_mva (AC rows: valve-side AC bus; DC row: DC bus).
/// The entries mirror the VSC coupling blocks:
///   dpac_dvm / dqac_dvm : P/Q-row(ac) -> Vm-col(ac)
///   dpac_dvdc / dqac_dvdc: P/Q-row(ac) -> Vdc-col(dc)
///   dpdc_dvm            : DC-row(dc) -> Vm-col(ac)   (through U_d0)
///   dpdc_dvdc           : DC-row(dc) -> Vdc-col(dc)  (same as
///                         lcc_dc_jacobian_vdc, provided for symmetry)
struct LCCJacobian {
  double dpac_dvm{0.0};
  double dqac_dvm{0.0};
  double dpac_dvdc{0.0};
  double dqac_dvdc{0.0};
  double dpdc_dvm{0.0};
  double dpdc_dvdc{0.0};
};

LCCJacobian lcc_ac_dc_jacobian(const SolverData& data,
                               const LCCConverter& conv,
                               const Eigen::VectorXd& vm,
                               const Eigen::VectorXd& vdc);

/// True when the station regulates its DC voltage through its external
/// characteristic (ConstantGamma inverter / ConstantAlpha rectifier): such a
/// station anchors its DC island's voltage like a droop VSC, so the DC island
/// needs no further voltage reference (pin or promotion).
bool lcc_forms_dc_voltage(const LCCConverter& conv);

/// True when station_role and control_mode form a supported combination.
bool lcc_control_supported(const LCCConverter& conv);

}  // namespace hacdcpf::powerflow
