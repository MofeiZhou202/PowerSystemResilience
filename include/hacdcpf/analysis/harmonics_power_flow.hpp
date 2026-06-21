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

#include <array>
#include <complex>
#include <map>
#include <string>
#include <utility>
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
// Frequency-dependent conductor resistance (skin effect) model.
//   None             R(h) = R₁
//   SqrtOrder        R(h) = R₁·√h                  (docs/harmonic_three_phase.md §2.3)
//   ProportionalSqrt R(h) = R₁·(1 + k·√h)          (k = skin_coefficient)
enum class SkinEffectModel {
  None,
  SqrtOrder,
  ProportionalSqrt,
};

// Frequency-dependent DC ripple network parameters.  By default the DC ripple
// network is purely resistive (DCBranch carries only r_pu).  These optional
// overrides add series inductance to DC branches and shunt capacitance (DC-link
// smoothing) to DC buses, so the DC ripple impedance becomes frequency-dependent:
//     Z_branch(r) = r_pu + j·r·x_pu ,   Y_bus_cap(r) = j·r·b_pu .
struct DCRippleModel {
  std::map<int, double> branch_x_pu;  ///< DCBranch::index -> fundamental-freq reactance
  std::map<int, double> bus_b_pu;     ///< DCBus::index   -> fundamental-freq susceptance
};

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

  /// Frequency-dependent conductor resistance (skin effect) applied to AC line /
  /// transformer series resistance.  Default: off (R independent of order).
  SkinEffectModel skin_effect{SkinEffectModel::None};
  double skin_coefficient{0.0};  ///< k for ProportionalSqrt

  /// Newton-Raphson outer loop (used by solve_harmonic_power_flow_newton).
  int    newton_max_iter{30};
  double newton_tol{1e-10};

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

// ═══════════════════════════════════════════════════════════════════════════
// Three-phase (phase-domain) harmonic power flow
// ═══════════════════════════════════════════════════════════════════════════
// Extends the single-phase / positive-sequence model above to an unbalanced
// abc-domain network, per docs/harmonic_three_phase.md.  The AC network is
// assembled on 3·N phase-nodes; line impedances become 3×3 phase matrices
// Z_abc(h) = A·diag(z0(h), z1(h), z1(h))·A⁻¹ (or explicit phase matrices), and a
// *balanced* harmonic current source automatically carries the correct phase
// sequence for its order:
//     h mod 3 == 1  -> positive sequence  (I_b = a²·I_a, I_c = a·I_a)
//     h mod 3 == 2  -> negative sequence  (I_b = a·I_a,  I_c = a²·I_a)
//     h mod 3 == 0  -> zero    sequence  (I_a = I_b = I_c)
// with a = e^{j2π/3}.

/// Returns the symmetrical-component sequence of a balanced harmonic of order h:
/// 1 = positive, 2 = negative, 0 = zero.
int harmonic_sequence_of_order(int h);

// Phase-aware harmonic current source (nonlinear load / CIDER / converter AC port).
struct ThreePhaseHarmonicSource {
  int  bus{0};
  std::string name;

  /// When true the spectrum is applied as a balanced set with the natural phase
  /// sequence for each order; only the A-phase reference (i_base_pu) is used.
  bool balanced{true};

  /// Reference current magnitude [pu] for the percentage spectrum.  In balanced
  /// mode this is the A-phase reference; the other phases follow the sequence.
  double i_base_pu{0.0};
  double i_base_phase_deg{0.0};

  /// Per-phase reference magnitudes used only when balanced == false.
  double i_base_pu_a{0.0};
  double i_base_pu_b{0.0};
  double i_base_pu_c{0.0};

  HarmonicSpectrum spectrum;  ///< orders and magnitudes (percent of reference)
};

// Three-phase NIC AC port modelled as a balanced harmonic current source whose
// magnitude follows the converter through-power operating point.  (The DC-side
// ripple coupling is handled by the single-phase DC path.)
struct ThreePhaseHarmonicNIC {
  int vsc_index{-1};
  int bus_ac{0};
  std::string name;

  /// Total three-phase AC operating power; per-phase split is S/3 unless the
  /// per-phase overrides are positive.
  double s_ac_p_mw{0.0};
  double s_ac_q_mvar{0.0};

  HarmonicSpectrum ac_spectrum;  ///< empty -> default six-pulse spectrum
};

struct ThreePhaseHarmonicInputs {
  std::vector<ThreePhaseHarmonicSource> sources;
  std::vector<ThreePhaseHarmonicNIC>    nics;
};

struct ThreePhaseHarmonicBusResult {
  int bus{0};
  PhaseMask phase_mask{PhaseMask::abc()};

  /// Fundamental phase voltage magnitudes [pu] (THD references).
  double v_fund_pu_a{1.0};
  double v_fund_pu_b{1.0};
  double v_fund_pu_c{1.0};

  /// Complex harmonic voltage phasor per order [pu], per phase (incl. order 1).
  std::map<int, Complex> v_by_order_a;
  std::map<int, Complex> v_by_order_b;
  std::map<int, Complex> v_by_order_c;

  /// Per-phase voltage THD [%].
  double thd_a_pct{0.0};
  double thd_b_pct{0.0};
  double thd_c_pct{0.0};
};

struct HPF3phResult {
  bool        ok{false};
  std::string message;

  std::vector<int> ac_orders;  ///< orders actually solved (excludes fundamental)
  std::vector<ThreePhaseHarmonicBusResult> bus_results;

  bool base_pf_converged{false};
  std::map<int, bool> ac_order_solved;

  double max_thd_pct{0.0};
  int    max_thd_bus{-1};

  // Populated only by the three-phase Newton solver (defaults for the linear path).
  bool newton_converged{true};
  std::map<int, int>    newton_iterations;
  std::map<int, double> newton_residual;
  int max_newton_iterations{0};

  std::string summary() const;
};

/// Solve the three-phase (abc-domain) harmonic power flow.
HPF3phResult solve_harmonic_power_flow_3ph(const ThreePhaseACSystem& sys,
                                           const ThreePhaseHarmonicInputs& inputs,
                                           const HPFOptions& opt = {});

/// Convenience overload: converters auto-modelled, no explicit sources.
HPF3phResult solve_harmonic_power_flow_3ph(const ThreePhaseACSystem& sys,
                                           const HPFOptions& opt = {});

// ═══════════════════════════════════════════════════════════════════════════
// Harmonic distortion limit compliance (IEEE 519-2014 / GB-T 14549-1993)
// ═══════════════════════════════════════════════════════════════════════════
// Post-processes a harmonic power-flow result against a standard's voltage
// distortion limits.  Limits depend on the bus nominal voltage level; GB-T also
// distinguishes odd vs even individual-harmonic limits.

enum class HarmonicStandard {
  IEEE519_2014,    ///< IEEE 519-2014 Table 1 (voltage distortion limits)
  GBT14549_1993,   ///< GB/T 14549-1993 public-grid voltage harmonic limits
};

struct HarmonicLimitCheck {
  int    bus{0};
  bool   is_dc{false};
  int    phase{-1};        ///< -1 single-phase/aggregate; 0/1/2 = a/b/c
  double base_kv{0.0};

  double thd_pct{0.0};
  double thd_limit_pct{0.0};
  bool   thd_ok{true};

  int    worst_ihd_order{0};   ///< order of the largest individual distortion
  double worst_ihd_pct{0.0};   ///< |V_h|/|V_1| * 100 at that order
  double ihd_limit_pct{0.0};   ///< limit applicable to that order
  bool   ihd_ok{true};

  bool   compliant{true};      ///< thd_ok && ihd_ok
};

struct HarmonicComplianceReport {
  HarmonicStandard standard{HarmonicStandard::IEEE519_2014};
  std::vector<HarmonicLimitCheck> checks;
  bool all_compliant{true};
  int  n_violations{0};
  int  worst_bus{-1};          ///< bus with the largest THD/limit ratio
  double worst_ratio{0.0};     ///< max over buses of thd_pct / thd_limit_pct
  std::string summary() const;
};

/// Look up the (THD limit, individual-harmonic limit) [%] for a bus at the given
/// nominal voltage and harmonic order under the chosen standard.
std::pair<double, double> harmonic_voltage_limits(HarmonicStandard standard,
                                                  double base_kv, int order);

/// Check a single-phase / positive-sequence result against a standard.  The bus
/// nominal voltages are taken from the system model (AC and DC buses).
HarmonicComplianceReport check_harmonic_limits(const HPFResult& result,
                                               const HybridPowerSystem& sys,
                                               HarmonicStandard standard);

/// Check a three-phase result against a standard (one entry per bus & phase).
HarmonicComplianceReport check_harmonic_limits(const HPF3phResult& result,
                                               const ThreePhaseACSystem& sys,
                                               HarmonicStandard standard);

// ═══════════════════════════════════════════════════════════════════════════
// Coupled three-phase AC + DC harmonic power flow (full NIC bridge)
// ═══════════════════════════════════════════════════════════════════════════
// A single Network-Interfacing Converter bridges the three-phase (abc-domain) AC
// sub-system and the DC ripple sub-system.  Both networks are assembled into ONE
// linear system spanning all AC phase-nodes × AC orders and all DC nodes × DC
// orders, with the NIC supplying the bidirectional cross-coupling at the
// characteristic six-pulse order pairs (an AC order h couples to a DC order r
// whenever |h − r| = 1, e.g. AC 5th & 7th ↔ DC 6th, AC 11th & 13th ↔ DC 12th).
//
// Level-1 cross-coupling (docs/harmonic_general.md §10.1, docs/harmonic_three_phase.md):
//   ΔI_ac,abc(h)  += k_ad · V_dc(r)          (DC ripple voltage drives AC current)
//   ΔI_dc(r)      += k_da · V_ac,a(h)         (AC harmonic voltage drives DC current)
// With k_ad = k_da = 0 the solve reduces to the decoupled spectrum-injection model.

struct ThreePhaseHybridNIC {
  int vsc_index{-1};
  int bus_ac{0};           ///< bus index in the ThreePhaseACSystem (AC port)
  int bus_dc{0};           ///< bus index in the DCSystem (DC port)
  std::string name;

  PortBehavior ac_port{PortBehavior::GridFollowing};
  PortBehavior dc_port{PortBehavior::GridForming};

  /// Fundamental operating point (total three-phase AC power and DC power).
  double s_ac_p_mw{0.0};
  double s_ac_q_mvar{0.0};
  double p_dc_mw{0.0};

  /// AC-side balanced current spectrum (% of the per-phase fundamental current).
  /// Empty -> default six-pulse spectrum.
  HarmonicSpectrum ac_spectrum;
  /// DC-side ripple-current spectrum (% of the DC operating current).
  /// Empty -> default ripple spectrum.
  HarmonicSpectrum dc_spectrum;

  /// Bidirectional Level-1 coupling gains [pu].  Zero -> spectrum-only coupling.
  Complex k_ad{0.0, 0.0};  ///< AC current per unit DC ripple voltage
  Complex k_da{0.0, 0.0};  ///< DC current per unit AC harmonic voltage

  /// Optional Norton output immittances stamped at the ports.
  Complex y_out_ac{0.0, 0.0};   ///< AC-side output admittance (g + j·h·b)
  Complex y_out_dc{0.0, 0.0};   ///< DC-side output admittance (real)
};

struct ThreePhaseHybridInputs {
  std::vector<ThreePhaseHarmonicSource> ac_sources;  ///< AC nonlinear loads/CIDERs
  std::vector<HarmonicCurrentSource>    dc_sources;  ///< DC ripple sources (is_dc)
  std::vector<ThreePhaseHybridNIC>      nics;        ///< bridging converters
};

struct HPFHybrid3phResult {
  bool        ok{false};
  std::string message;

  std::vector<int> ac_orders;
  std::vector<int> dc_orders;

  std::vector<ThreePhaseHarmonicBusResult> ac_bus_results;
  std::vector<HarmonicBusResult>           dc_bus_results;

  bool combined_solved{false};

  double max_ac_thd_pct{0.0};
  int    max_ac_thd_bus{-1};
  double max_dc_thd_pct{0.0};
  int    max_dc_thd_bus{-1};

  std::string summary() const;
};

/// Solve the coupled three-phase AC + DC harmonic power flow.  The AC and DC
/// networks are factorised together so a single converter genuinely drives both.
HPFHybrid3phResult solve_harmonic_power_flow_3ph_hybrid(
    const ThreePhaseACSystem& ac, const DCSystem& dc,
    const ThreePhaseHybridInputs& inputs, const HPFOptions& opt = {});

// ═══════════════════════════════════════════════════════════════════════════
// Newton-Raphson harmonic power flow (nonlinear resources)
// ═══════════════════════════════════════════════════════════════════════════
// The linear "penetration" solvers above treat resources as fixed Norton current
// sources, so each frequency is a single linear solve.  When a resource current
// depends nonlinearly on its own terminal voltage (saturating devices, control
// loops, constant-harmonic-power loads, ...), the mismatch must be driven to
// zero with a Newton iteration:
//     r(V̂) = Ŷ·V̂ − Î_res(V̂) = 0 ,   Ĵ = Ŷ − ∂Î_res/∂V̂ = Ĵ_GRD − Ĵ_RSC
//     V̂ ← V̂ − Ĵ⁻¹·r(V̂)
// (the paper's Ĵ = Ĵ_RSC − Ĵ_GRD form, up to overall sign).  With no nonlinear
// resource the Jacobian is constant and Newton converges in a single step,
// reproducing the linear solver exactly.

// A single-port nonlinear harmonic resource.  Its injected current at order h is
//     Î_h = i_src(h) − y_out(h)·V_h − g2(h)·V_h²
// where the holomorphic quadratic term g2 provides the voltage dependence that
// makes the problem nonlinear (a second-order Taylor model of a saturating
// device / converter characteristic).  Linear terms are folded into the network.
struct HarmonicNonlinearSource {
  int  bus{0};
  std::string name;
  std::map<int, Complex> i_src;   ///< constant source current per order [pu]
  std::map<int, Complex> y_out;   ///< linear Norton output admittance per order
  std::map<int, Complex> g2;      ///< quadratic voltage coefficient per order
};

struct HPFNewtonResult {
  bool        ok{false};
  std::string message;

  std::vector<int> ac_orders;
  std::vector<HarmonicBusResult> ac_bus_results;

  bool converged{true};
  std::map<int, int>    iterations;      ///< Newton iterations per order
  std::map<int, double> final_residual;  ///< ‖r‖∞ per order
  int max_iterations_used{0};

  double max_ac_thd_pct{0.0};
  int    max_ac_thd_bus{-1};

  std::string summary() const;
};

/// Solve the (single-phase / positive-sequence) harmonic power flow with a
/// Newton-Raphson outer loop over nonlinear resources.  `linear_inputs` supplies
/// the usual linear current sources / NICs; `nonlinear` supplies the
/// voltage-dependent resources that drive the iteration.
HPFNewtonResult solve_harmonic_power_flow_newton(
    const HybridPowerSystem& sys,
    const std::vector<HarmonicNonlinearSource>& nonlinear,
    const HarmonicStudyInputs& linear_inputs = {},
    const HPFOptions& opt = {});

// A three-phase (abc-domain) single-port nonlinear harmonic resource.  Each
// per-order entry holds the {a, b, c} phase values of the constant source
// current, linear Norton output admittance, and the holomorphic quadratic
// voltage coefficient:  Î_φ = i_src_φ − y_out_φ·V_φ − g2_φ·V_φ².
struct ThreePhaseNonlinearSource {
  int  bus{0};
  std::string name;
  std::map<int, std::array<Complex, 3>> i_src;
  std::map<int, std::array<Complex, 3>> y_out;
  std::map<int, std::array<Complex, 3>> g2;
};

/// Solve the three-phase (abc-domain) harmonic power flow with a Newton-Raphson
/// outer loop over per-phase nonlinear resources.  Reduces to the linear
/// three-phase solver when no quadratic coefficient is supplied.
HPF3phResult solve_harmonic_power_flow_3ph_newton(
    const ThreePhaseACSystem& sys,
    const std::vector<ThreePhaseNonlinearSource>& nonlinear,
    const ThreePhaseHarmonicInputs& linear_inputs = {},
    const HPFOptions& opt = {});

// A constant-harmonic-power load.  At order h it draws a constant complex power
// S_h, so its injected current is  Î_h = −conj(S_h)/conj(V_h).  The conjugation
// makes this NON-holomorphic in V_h, so it is solved with the real/imaginary
// (2N) Newton formulation in solve_harmonic_power_flow_newton_real().
struct ConstantPowerHarmonicLoad {
  int  bus{0};
  std::string name;
  std::map<int, Complex> s_set;  ///< per-order complex power drawn [pu]
};

/// Newton-Raphson harmonic power flow using the real/imaginary (2N) formulation,
/// which handles non-holomorphic resources such as constant-harmonic-power loads.
/// `linear_inputs` provide the harmonic excitation; reduces to the linear solve
/// when no constant-power load is supplied.
HPFNewtonResult solve_harmonic_power_flow_newton_real(
    const HybridPowerSystem& sys,
    const std::vector<ConstantPowerHarmonicLoad>& cp_loads,
    const HarmonicStudyInputs& linear_inputs = {},
    const HPFOptions& opt = {});

// A bilinear frequency-mixing term:  Î_{out} −= coeff · V_p · V_q .  A quadratic
// time-domain nonlinearity i = g·v² generates exactly such products (harmonic
// convolution), coupling different orders so they must be solved together.
struct HarmonicMixingTerm {
  int     out_order{0};   ///< h: order whose injected current is affected
  int     p_order{0};     ///< p
  int     q_order{0};     ///< q
  Complex coeff{0.0, 0.0};///< k
};

// A cross-order nonlinear resource: a linear source spectrum plus bilinear
// frequency-mixing terms that couple the harmonic orders.
struct CrossOrderNonlinearSource {
  int  bus{0};
  std::string name;
  std::map<int, Complex>          i_src;   ///< linear source current per order
  std::vector<HarmonicMixingTerm> mixing;  ///< bilinear cross-order coupling terms
};

/// Newton-Raphson harmonic power flow that solves ALL harmonic orders
/// simultaneously (the stacked state), so cross-order (frequency-mixing)
/// nonlinear resources are handled.  Reduces to independent per-order linear
/// solves when no mixing term is supplied.
HPFNewtonResult solve_harmonic_power_flow_newton_coupled(
    const HybridPowerSystem& sys,
    const std::vector<CrossOrderNonlinearSource>& resources,
    const HarmonicStudyInputs& linear_inputs = {},
    const HPFOptions& opt = {});

// ═══════════════════════════════════════════════════════════════════════════
// Frequency scan / resonance analysis
// ═══════════════════════════════════════════════════════════════════════════
// Sweeps the driving-point (Thévenin) impedance  Z_dp(f) = (Ŷ(f)⁻¹)_kk  at one
// or more buses over a continuous frequency range and flags resonances:
//   * a |Z| PEAK is a parallel resonance (harmonic currents excite high voltage),
//   * a |Z| DIP  is a series   resonance.
struct FrequencyScanOptions {
  double f_start{1.0};   ///< start frequency (multiple of the fundamental)
  double f_end{25.0};    ///< end frequency
  double f_step{0.1};    ///< step
  std::vector<int> buses;       ///< probed buses (empty -> all in-service)
  bool   detect_resonances{true};
  double resonance_min_pu{0.0}; ///< ignore peaks with |Z| below this
};

struct HarmonicResonance {
  int    bus{0};
  double freq_order{0.0};
  double z_mag{0.0};
  bool   parallel{true};  ///< true = parallel (peak), false = series (dip)
  int    sequence{-1};    ///< -1 single-phase; 0/1/2 = zero/positive/negative
};

struct FrequencyScanResult {
  bool        ok{false};
  std::string message;
  std::vector<double> freqs;
  std::map<int, std::vector<double>> z_mag;      ///< bus -> |Z_dp| per frequency
  std::map<int, std::vector<double>> z_ang_deg;  ///< bus -> angle(Z_dp) per freq
  std::vector<HarmonicResonance> resonances;
  std::string summary() const;
};

/// Single-phase / positive-sequence driving-point impedance frequency scan.
FrequencyScanResult frequency_scan(const HybridPowerSystem& sys,
                                   const FrequencyScanOptions& sopt,
                                   const HPFOptions& opt = {});

struct SequenceScanResult {
  bool        ok{false};
  std::string message;
  int         bus{0};
  std::vector<double> freqs;
  std::vector<double> z1_mag;  ///< positive-sequence |Z_dp| per frequency
  std::vector<double> z2_mag;  ///< negative-sequence
  std::vector<double> z0_mag;  ///< zero-sequence
  std::vector<HarmonicResonance> resonances;  ///< tagged by sequence
  std::string summary() const;
};

/// Per-sequence (positive / negative / zero) driving-point impedance scan at a
/// bus of a three-phase (abc-domain) network.
SequenceScanResult sequence_frequency_scan(const ThreePhaseACSystem& sys, int bus,
                                           const FrequencyScanOptions& sopt,
                                           const HPFOptions& opt = {});

// ═══════════════════════════════════════════════════════════════════════════
// Three-phase Newton variants: non-holomorphic and cross-order
// ═══════════════════════════════════════════════════════════════════════════

// A three-phase constant-harmonic-power load: per order, the {a,b,c} complex
// power drawn.  Î_φ = −conj(S_φ)/conj(V_φ) (non-holomorphic; real/imag 2N Newton).
struct ThreePhaseConstantPowerLoad {
  int  bus{0};
  std::string name;
  std::map<int, std::array<Complex, 3>> s_set;
};

/// Three-phase real/imaginary (2N) Newton for non-holomorphic constant-power
/// loads in the abc-domain.  Reduces to the linear three-phase solve otherwise.
HPF3phResult solve_harmonic_power_flow_3ph_newton_real(
    const ThreePhaseACSystem& sys,
    const std::vector<ThreePhaseConstantPowerLoad>& cp_loads,
    const ThreePhaseHarmonicInputs& linear_inputs = {},
    const HPFOptions& opt = {});

// A per-phase bilinear frequency-mixing term:  Î_{φ,out} −= coeff·V_{φ,p}·V_{φ,q}.
// `phase` selects a single phase (0/1/2) or all three when -1.
struct ThreePhaseMixingTerm {
  int     out_order{0};
  int     p_order{0};
  int     q_order{0};
  int     phase{-1};
  Complex coeff{0.0, 0.0};
};

struct ThreePhaseCrossOrderSource {
  int  bus{0};
  std::string name;
  std::map<int, std::array<Complex, 3>> i_src;
  std::vector<ThreePhaseMixingTerm>     mixing;
};

/// Three-phase stacked Newton that solves all orders together so cross-order
/// (frequency-mixing) nonlinear resources are handled in the abc-domain.
HPF3phResult solve_harmonic_power_flow_3ph_newton_coupled(
    const ThreePhaseACSystem& sys,
    const std::vector<ThreePhaseCrossOrderSource>& resources,
    const ThreePhaseHarmonicInputs& linear_inputs = {},
    const HPFOptions& opt = {});

// ═══════════════════════════════════════════════════════════════════════════
// Harmonic metrics: losses, K-factor, THD / TDD of current
// ═══════════════════════════════════════════════════════════════════════════
// Post-processes a single-phase / positive-sequence harmonic result into the
// standard reporting quantities (branch currents are recomputed from the bus
// voltage spectra so the same series resistance — including skin effect — is
// used for the I²R losses).
struct HarmonicMetricsOptions {
  /// Maximum demand load current I_L [pu] used for TDD (IEEE 519).  When <= 0,
  /// the branch fundamental current is used (then TDD == THD).
  double i_demand_pu{0.0};
};

struct BranchHarmonicMetrics {
  int    from_bus{0};
  int    to_bus{0};
  double i_fund_pu{0.0};         ///< |I_1|
  double i_rms_pu{0.0};          ///< sqrt(Σ_h |I_h|²)
  double thd_i_pct{0.0};         ///< sqrt(Σ_{h>1}|I_h|²)/|I_1|·100
  double tdd_pct{0.0};           ///< sqrt(Σ_{h>1}|I_h|²)/I_L·100
  double k_factor{1.0};          ///< Σ h²|I_h|² / Σ |I_h|²  (UL 1561)
  double p_loss_pu{0.0};         ///< Σ_h |I_h|²·R(h)
  double p_loss_harmonic_pu{0.0};///< Σ_{h>1} |I_h|²·R(h)
};

struct HarmonicMetricsResult {
  std::vector<BranchHarmonicMetrics> ac_branches;
  double total_loss_pu{0.0};
  double total_harmonic_loss_pu{0.0};
  double harmonic_loss_fraction{0.0};  ///< harmonic / total loss
  double max_k_factor{1.0};
  int    max_k_factor_branch{-1};
  double max_thd_i_pct{0.0};
  double max_tdd_pct{0.0};
  std::string summary() const;
};

/// Compute branch harmonic metrics (losses, K-factor, current THD / TDD) from a
/// harmonic power-flow result.
HarmonicMetricsResult harmonic_metrics(const HybridPowerSystem& sys,
                                       const HPFResult& result,
                                       const HarmonicMetricsOptions& mopt = {},
                                       const HPFOptions& opt = {});

// ═══════════════════════════════════════════════════════════════════════════
// Hybrid AC + DC Newton with bilinear NIC cross-domain coupling
// ═══════════════════════════════════════════════════════════════════════════
// Solves the AC and DC harmonic networks TOGETHER (one stacked Newton state)
// when the network-interfacing converter introduces a *nonlinear* coupling: the
// switching action mixes AC and DC frequency components multiplicatively, so an
// injected current is a product of two state voltages.  A bilinear term means
//     Î_out −= coeff · V_a · V_b ,
// where each of {out, a, b} names an AC or DC bus at a particular order.  Such
// products make the combined system nonlinear, requiring the Newton iteration
// (linear k_ad/k_da coupling is the special case handled by the linear hybrid
// solver).  Reduces to independent AC/DC linear solves when no bilinear term.
struct HybridBilinearTerm {
  bool out_is_dc{false};  int out_order{0};  int out_bus{0};
  bool a_is_dc{false};    int a_order{0};    int a_bus{0};
  bool b_is_dc{false};    int b_order{0};    int b_bus{0};
  Complex coeff{0.0, 0.0};
};

struct HybridNewtonInputs {
  std::vector<HarmonicCurrentSource> sources;   ///< AC (is_dc=false) and DC sources
  std::vector<HybridBilinearTerm>    bilinear;  ///< nonlinear cross-domain coupling
};

struct HPFHybridNewtonResult {
  bool        ok{false};
  std::string message;
  std::vector<int> ac_orders;
  std::vector<int> dc_orders;
  std::vector<HarmonicBusResult> ac_bus_results;
  std::vector<HarmonicBusResult> dc_bus_results;
  bool   converged{false};
  int    iterations{0};
  double final_residual{0.0};
  double max_ac_thd_pct{0.0};
  int    max_ac_thd_bus{-1};
  double max_dc_thd_pct{0.0};
  int    max_dc_thd_bus{-1};
  std::string summary() const;
};

/// Hybrid AC + DC Newton solve with bilinear NIC cross-domain coupling.
HPFHybridNewtonResult solve_harmonic_power_flow_hybrid_newton(
    const HybridPowerSystem& sys, const HybridNewtonInputs& inputs,
    const HPFOptions& opt = {});

}  // namespace hacdcpf::harmonics
