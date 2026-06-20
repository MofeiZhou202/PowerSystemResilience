#pragma once

/// analysis/harmonics_power_flow.hpp
/// =================================
/// Harmonic power-flow (HPF) study for hybrid AC/DC distribution networks.
///
/// This module implements the frequency-domain "harmonic penetration" form of
/// the unified AC/DC harmonic power flow described in docs/harmonic_general.md,
/// docs/harmonic_initial.md, docs/harmonic_nic.md and docs/harmonic_multiple_nic.md
/// (Becker et al., "Harmonic Power-Flow Study of Hybrid AC/DC Grids with
/// Converter-Interfaced Distributed Energy Resources").
///
/// Pipeline (mirrors the documentation):
///   solve_power_flow(system)                         // Step 0: operating point
///        -> harmonic operating point extraction       // V1, I1, Vdc0, Idc0
///        -> per-order frequency-domain network build   // Ybus(h), Ydc(r)
///        -> resource current injection (sources + NIC) // I_hat(h)
///        -> linear solve  Y(h) V(h) = I(h)             // direct nodal method
///        -> post-processing (THD / IHD / branch flows)
///
/// First-version ("Level 1") modelling choices, all documented and configurable:
///   * Resources are modelled as Norton harmonic-current sources; their internal
///     output admittance is optionally stamped into the network matrix.
///   * The two-port NIC (Network-Interfacing Converter) bridges the AC and DC
///     sub-systems by injecting a consistent harmonic spectrum on both ports,
///     derived from the SAME fundamental through-power operating point
///     (I_ac,1 = conj(S_ac)/conj(V_ac,1), I_dc,0 = P_dc/V_dc,0).
///   * AC network branches scale as Z(h) = r + j*h*x with line charging j*h*b.
///   * DC ripple branches are purely resistive (the DCBranch model carries no
///     inductance); the ripple network is therefore frequency-flat by design.
///
/// The model is linear at each harmonic order, so the solve is a single complex
/// sparse factorisation per order — fast and numerically reproducible.

#include <complex>
#include <map>
#include <string>
#include <vector>

#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"

namespace hacdcpf::harmonics {

using Complex = std::complex<double>;

// ───────────────────────────────────────────────────────────────────────────
// Port behaviour (grid-forming vs grid-following), used to assign NIC port roles
// ───────────────────────────────────────────────────────────────────────────
enum class PortBehavior {
  GridForming,    ///< Controls voltage; the unknown is the injected current.
  GridFollowing,  ///< Controls current; the unknown is the terminal voltage.
};

// ───────────────────────────────────────────────────────────────────────────
// A single component of a current-source spectrum.
//   order      harmonic / ripple order (h for AC, r for DC)
//   mag_percent magnitude as a percentage of the fundamental (AC) or DC operating
//               current of the owning resource
//   phase_deg   phase angle in degrees, relative to the fundamental reference
// ───────────────────────────────────────────────────────────────────────────
struct HarmonicSpectrumLine {
  int    order{0};
  double mag_percent{0.0};
  double phase_deg{0.0};
};

using HarmonicSpectrum = std::vector<HarmonicSpectrumLine>;

// ───────────────────────────────────────────────────────────────────────────
// User-defined single-port harmonic current source (nonlinear load / CIDER).
// The injection convention is "positive = into the network" (bus-injection
// positive), matching the steady-state power-flow convention of this project.
// ───────────────────────────────────────────────────────────────────────────
struct HarmonicCurrentSource {
  int  bus{0};            ///< AC bus index (or DC bus index when is_dc==true)
  bool is_dc{false};      ///< true: source lives on the DC sub-system
  std::string name;

  /// Fundamental (or DC operating) current magnitude [pu on system base] used as
  /// the reference for the percentage spectrum.  When <= 0 the module derives it
  /// from the base power flow (nodal injection current), if available.
  double i_base_pu{0.0};

  /// Phase reference of the fundamental current [deg].  Used to rotate the
  /// per-order phase angles.  Ignored when the base current is derived from PF.
  double i_base_phase_deg{0.0};

  HarmonicSpectrum spectrum;  ///< orders > 1 (AC) or > 0 (DC)
};

// ───────────────────────────────────────────────────────────────────────────
// Two-port Network-Interfacing Converter harmonic model ("Level 1").
//
// Maps to an existing VSCConverter (by index) when vsc_index >= 0, in which case
// the AC/DC buses and the fundamental operating point are taken from the model /
// base power flow.  May also be specified standalone via bus_ac / bus_dc.
// ───────────────────────────────────────────────────────────────────────────
struct HarmonicNIC {
  int vsc_index{-1};      ///< index into HybridPowerSystem::vsc_converters, or -1
  int bus_ac{0};          ///< AC port bus index
  int bus_dc{0};          ///< DC port bus index
  std::string name;

  PortBehavior ac_port{PortBehavior::GridFollowing};
  PortBehavior dc_port{PortBehavior::GridForming};

  /// AC-side current spectrum, percentage of the fundamental AC port current.
  /// Empty -> the module fills in a default six-pulse spectrum.
  HarmonicSpectrum ac_spectrum;

  /// DC-side ripple-current spectrum, percentage of the DC operating current.
  /// Empty -> the module fills in a default characteristic ripple spectrum.
  HarmonicSpectrum dc_spectrum;

  /// Optional Norton output admittance [pu] stamped at the AC port (per order it
  /// is scaled as g + j*h*b).  Zero -> ideal current source.
  Complex y_out_ac{0.0, 0.0};

  /// Optional fundamental operating-point overrides.  When non-positive the
  /// module derives them from the base power flow / VSC transfer.
  double s_ac_p_mw{0.0};
  double s_ac_q_mvar{0.0};
  double p_dc_mw{0.0};
};

// ───────────────────────────────────────────────────────────────────────────
// Options
// ───────────────────────────────────────────────────────────────────────────
struct HPFOptions {
  /// AC harmonic orders to study (the fundamental, 1, is always handled
  /// separately as the operating-point reference and need not be listed).
  std::vector<int> ac_orders{5, 7, 11, 13, 17, 19, 23, 25};

  /// DC ripple orders to study (0 is the steady component from the base PF).
  std::vector<int> dc_orders{2, 6, 12, 18, 24};

  /// Run a base fundamental power flow internally to obtain the operating point.
  /// When false the operating point is read from the model's stored bus voltages
  /// (ACBus::vm_pu / va_deg, DCBus::vm_pu).
  bool run_base_power_flow{true};

  /// Model in-service loads as shunt impedances in the harmonic network
  /// (CIGRE/IEC parallel R // jX derived from the fundamental P, Q).
  bool include_load_impedance{true};

  /// Default sub-transient reactance [pu] for AC voltage sources (slack / PV /
  /// generators / external grids) when the component does not specify one.  This
  /// grounds the harmonic network so the matrix is non-singular.
  double default_source_xpp_pu{0.2};

  /// Internal (output) impedance [pu] of DC voltage-forming nodes (DC_V buses and
  /// grid-forming NIC DC ports).  Grounds the DC ripple network; smaller = stiffer.
  double dc_source_impedance_pu{0.01};

  /// Lower bound applied to |Z_ss| diagonal / shunt magnitudes to keep the
  /// per-order matrix well-conditioned.
  double min_shunt_pu{1e-9};

  /// Automatically attach a default six-pulse spectrum to every in-service VSC
  /// converter that is not already supplied as a HarmonicNIC.
  bool auto_nic_from_vscs{true};

  /// Emit per-branch harmonic current flows in the result.
  bool compute_branch_flows{true};

  PowerFlowOptions base_pf_options{};  ///< options forwarded to the base PF
};

// ───────────────────────────────────────────────────────────────────────────
// Results
// ───────────────────────────────────────────────────────────────────────────
struct HarmonicBusResult {
  int  bus{0};
  bool is_dc{false};

  /// Fundamental (AC) or steady (DC) voltage magnitude [pu], the THD reference.
  double v_fund_pu{1.0};

  /// Complex harmonic voltage phasor per order [pu].  For AC this includes the
  /// fundamental (order 1); for DC it includes the steady component (order 0).
  std::map<int, Complex> v_by_order;

  /// Voltage total harmonic distortion [%] =
  ///   sqrt( sum_{h != fundamental} |V_h|^2 ) / |V_fund| * 100.
  double thd_pct{0.0};
};

struct HarmonicBranchFlow {
  int    from_bus{0};
  int    to_bus{0};
  bool   is_dc{false};
  /// Current magnitude per order [pu] in the from->to direction.
  std::map<int, double> i_by_order;
  /// Current total harmonic distortion [%] referred to the fundamental current.
  double thd_i_pct{0.0};
};

struct HPFResult {
  bool        ok{false};
  std::string message;

  std::vector<int> ac_orders;  ///< orders actually solved (excludes fundamental)
  std::vector<int> dc_orders;  ///< ripple orders actually solved (excludes 0)

  std::vector<HarmonicBusResult>  ac_bus_results;
  std::vector<HarmonicBusResult>  dc_bus_results;
  std::vector<HarmonicBranchFlow> ac_branch_flows;
  std::vector<HarmonicBranchFlow> dc_branch_flows;

  /// Whether the per-order linear systems factorised successfully.
  bool base_pf_converged{false};
  std::map<int, bool> ac_order_solved;
  std::map<int, bool> dc_order_solved;

  // Aggregate distortion summary.
  double max_ac_thd_pct{0.0};
  int    max_ac_thd_bus{-1};
  double max_dc_thd_pct{0.0};
  int    max_dc_thd_bus{-1};

  std::string summary() const;
};

// ───────────────────────────────────────────────────────────────────────────
// Inputs grouped: the explicit harmonic resources to study.  Either of the
// vectors may be empty; when auto_nic_from_vscs is set the converters are filled
// in automatically.
// ───────────────────────────────────────────────────────────────────────────
struct HarmonicStudyInputs {
  std::vector<HarmonicCurrentSource> sources;  ///< user nonlinear loads / CIDERs
  std::vector<HarmonicNIC>           nics;     ///< explicit NIC overrides
};

// ───────────────────────────────────────────────────────────────────────────
// Public entry points
// ───────────────────────────────────────────────────────────────────────────

/// Solve the hybrid AC/DC harmonic power flow.
/// @param sys     the hybrid power system (raw, un-projected model)
/// @param inputs  harmonic sources / NIC overrides
/// @param opt     study options
HPFResult solve_harmonic_power_flow(const HybridPowerSystem& sys,
                                    const HarmonicStudyInputs& inputs,
                                    const HPFOptions& opt = {});

/// Convenience overload: no explicit sources, converters auto-modelled.
HPFResult solve_harmonic_power_flow(const HybridPowerSystem& sys,
                                    const HPFOptions& opt = {});

/// Default characteristic six-pulse AC current spectrum (percent of fundamental).
HarmonicSpectrum default_six_pulse_ac_spectrum();

/// Default characteristic DC-side ripple spectrum (percent of DC current).
HarmonicSpectrum default_dc_ripple_spectrum();

}  // namespace hacdcpf::harmonics
