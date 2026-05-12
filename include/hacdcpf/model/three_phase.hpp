#pragma once

#include <array>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

#include "hacdcpf/model/enums/bus_types.hpp"

namespace hacdcpf {

enum class Phase : std::uint8_t {
  A = 0,
  B = 1,
  C = 2,
};

constexpr int phase_to_index(Phase phase) {
  return static_cast<int>(phase);
}

using PhaseValueMatrix3 = std::array<double, 9>;

constexpr int phase_matrix_offset(int row, int col) {
  return row * 3 + col;
}

inline double phase_matrix_get(
    const PhaseValueMatrix3& matrix,
    int row,
    int col) {
  return matrix[static_cast<std::size_t>(phase_matrix_offset(row, col))];
}

inline void phase_matrix_set(
    PhaseValueMatrix3& matrix,
    int row,
    int col,
    double value) {
  matrix[static_cast<std::size_t>(phase_matrix_offset(row, col))] = value;
}

struct PhaseMask {
  std::uint8_t bits{0x7};

  constexpr PhaseMask() = default;
  constexpr explicit PhaseMask(std::uint8_t raw_bits)
      : bits(static_cast<std::uint8_t>(raw_bits & 0x7)) {}

  static constexpr PhaseMask none() { return PhaseMask(0x0); }
  static constexpr PhaseMask a() { return PhaseMask(0x1); }
  static constexpr PhaseMask b() { return PhaseMask(0x2); }
  static constexpr PhaseMask c() { return PhaseMask(0x4); }
  static constexpr PhaseMask ab() { return PhaseMask(0x3); }
  static constexpr PhaseMask ac() { return PhaseMask(0x5); }
  static constexpr PhaseMask bc() { return PhaseMask(0x6); }
  static constexpr PhaseMask abc() { return PhaseMask(0x7); }

  constexpr bool empty() const { return bits == 0; }
  constexpr bool has(int phase_index) const {
    return phase_index >= 0 && phase_index < 3 &&
           (bits & static_cast<std::uint8_t>(1u << phase_index)) != 0;
  }
  constexpr bool has(Phase phase) const { return has(phase_to_index(phase)); }
  constexpr bool contains(PhaseMask other) const {
    return (bits & other.bits) == other.bits;
  }
  constexpr int count() const {
    return (has(0) ? 1 : 0) + (has(1) ? 1 : 0) + (has(2) ? 1 : 0);
  }
};

inline std::string phase_mask_to_string(PhaseMask mask) {
  std::string text;
  if (mask.has(Phase::A)) text.push_back('A');
  if (mask.has(Phase::B)) text.push_back('B');
  if (mask.has(Phase::C)) text.push_back('C');
  return text;
}

inline PhaseMask phase_mask_from_string(
    const std::string& text,
    PhaseMask fallback = PhaseMask::abc()) {
  if (text.empty()) return fallback;

  std::uint8_t bits = 0;
  for (unsigned char raw_ch : text) {
    const char ch = static_cast<char>(std::toupper(raw_ch));
    if (ch == 'A' || ch == '1') bits |= 0x1;
    if (ch == 'B' || ch == '2') bits |= 0x2;
    if (ch == 'C' || ch == '3') bits |= 0x4;
  }
  return bits == 0 ? fallback : PhaseMask(bits);
}

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase AC Bus
//
// Per-phase voltage, demand, and shunt data for unbalanced networks.
// Sequence decomposition: positive (1), negative (2), zero (0) can be
// derived from per-phase quantities via Fortescue transformation.
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseACBus {
  int index{0};
  BusType bus_type{BusType::PQ};
  std::string name;
  double base_kv{0.0};
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  // Per-phase voltage magnitude and angle
  double vm_a_pu{1.0};  double va_a_deg{0.0};
  double vm_b_pu{1.0};  double va_b_deg{-120.0};
  double vm_c_pu{1.0};  double va_c_deg{120.0};

  // Per-phase active and reactive demand
  double pd_a_mw{0.0};   double qd_a_mvar{0.0};
  double pd_b_mw{0.0};   double qd_b_mvar{0.0};
  double pd_c_mw{0.0};   double qd_c_mvar{0.0};

  // Per-phase shunt admittance
  double gs_a_mw{0.0};   double bs_a_mvar{0.0};
  double gs_b_mw{0.0};   double bs_b_mvar{0.0};
  double gs_c_mw{0.0};   double bs_c_mvar{0.0};

  // Voltage limits (applied to all phases)
  double vmin_pu{0.9};
  double vmax_pu{1.1};

  int area{1};
  int zone{1};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase AC Line
//
// Sequence impedance model with positive and zero sequence parameters.
// Negative sequence is typically equal to positive sequence for lines.
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseACLine {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  std::string name;
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  double length_km{0.0};
  int parallel{1};                     // number of parallel circuits

  // Positive sequence impedance per km
  double r1_ohm_per_km{0.0};
  double x1_ohm_per_km{0.0};
  double c1_nf_per_km{0.0};           // capacitance (nF/km)

  // Zero sequence impedance per km
  double r0_ohm_per_km{0.0};
  double x0_ohm_per_km{0.0};
  double c0_nf_per_km{0.0};

  // Per-unit impedance (computed from per-km values and system base)
  double r1_pu{0.0};  double x1_pu{0.0};  double b1_pu{0.0};
  double r0_pu{0.0};  double x0_pu{0.0};  double b0_pu{0.0};

  // Optional full phase-domain parameters (per-unit on system base).
  // When use_phase_matrix=true, NR consumes these matrices instead of the
  // sequence-derived circulant approximation above.
  bool use_phase_matrix{false};
  PhaseValueMatrix3 r_matrix_pu{};
  PhaseValueMatrix3 x_matrix_pu{};
  PhaseValueMatrix3 b_matrix_pu{};

  // Thermal rating
  double max_i_ka{0.0};               // maximum current (kA)
  double rate_a_mva{0.0};             // MVA rating

  // Reliability
  double failure_rate{0.0};           // failures per year per km
  double mttr_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase Transformer (two-winding)
//
// Vector group determines zero-sequence current path behavior.
// Supported groups: YNyn, Dyn, YNd, Yy, Yd, Dd, YNz, Dz, etc.
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseTransformer {
  int index{0};
  std::string name;
  int hv_bus{0};
  int lv_bus{0};
  bool in_service{true};
  PhaseMask hv_phase_mask{PhaseMask::abc()};
  PhaseMask lv_phase_mask{PhaseMask::abc()};

  double sn_mva{0.0};                 // rated power
  double vn_hv_kv{0.0};               // rated HV voltage
  double vn_lv_kv{0.0};               // rated LV voltage

  // Positive sequence short-circuit parameters
  double vk_percent{0.0};             // short-circuit voltage (%)
  double vkr_percent{0.0};            // real part of vk (%)
  double pfe_kw{0.0};                 // iron losses
  double i0_percent{0.0};             // no-load current (%)

  // Vector group (determines zero-sequence behavior)
  std::string vector_group;            // e.g. "YNyn0", "Dyn11"

  // Optional winding-topology contract.
  // Empty => infer from vector_group + phase_mask.
  // Wye-like sides use phase labels such as "A,B,C".
  // Delta sides use winding labels such as "AB,BC,CA".
  std::string hv_winding_topology;
  std::string lv_winding_topology;

  // Zero-sequence parameters
  double vk0_percent{0.0};            // zero-sequence short-circuit voltage
  double vkr0_percent{0.0};           // zero-sequence real part
  double mag0_percent{0.0};           // zero-sequence magnetizing impedance
  double mag0_rx{0.0};                // zero-sequence magnetizing R/X ratio
  double si0_hv_partial{0.5};         // HV-side zero-sequence current share

  // Tap changer
  int tap_side{0};                     // 0=HV, 1=LV
  int tap_pos{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  double tap_step_percent{0.0};

  // Phase shift
  double shift_deg{0.0};

  // Reliability
  double mtbf_hr{0.0};
  double mttr_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase Regulator control (minimal DSS-aligned control object)
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseRegulatorControl {
  int index{0};
  std::string name;

  int transformer_index{0};
  std::string transformer_name;

  int winding{0};                  // monitored winding
  int tap_winding{0};              // winding carrying the actual taps
  int monitored_bus{0};            // 0 => monitor the controlled winding bus
  int monitored_node{1};           // OpenDSS-style node number: 1=A, 2=B, 3=C
  double vreg_volts{0.0};          // PT-secondary target voltage
  double band_volts{0.0};          // PT-secondary bandwidth
  double ptratio{0.0};             // local PT ratio
  double remote_ptratio{0.0};      // 0 => fall back to ptratio
  double ct_primary_amps{0.0};     // CT primary current used by LDC scaling
  double r_volts{0.0};             // line-drop compensation R setting on PT secondary
  double x_volts{0.0};             // line-drop compensation X setting on PT secondary
  int max_tap_change{1};           // maximum discrete tap steps per control iteration
  bool reversible{false};          // currently unsupported if true
  bool enabled{true};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase Load
//
// Per-phase power with wye/delta connection and ZIP model support.
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseLoad {
  int index{0};
  int bus{0};
  std::string name;
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  std::string connection{"wye"};       // "wye" or "delta"
  bool grounded{true};                 // wye: true=grounded neutral, false=open/floating neutral
  double r_neut_ohm{0.0};             // neutral grounding resistance (OpenDSS RNeut)
  double x_neut_ohm{0.0};             // neutral grounding reactance (OpenDSS XNeut)

  // Per-phase power
  double p_a_mw{0.0};  double q_a_mvar{0.0};
  double p_b_mw{0.0};  double q_b_mvar{0.0};
  double p_c_mw{0.0};  double q_c_mvar{0.0};

  // Voltage window / cutoff semantics.
  double vmin_pu{0.95};
  double vmax_pu{1.05};
  double zipv_cutoff_pu{0.0};         // 0 => disabled

  // Legacy shared ZIP model composition (% of nominal demand).
  double const_z_percent{0.0};        // constant impedance (%)
  double const_i_percent{0.0};        // constant current (%)
  double const_p_percent{100.0};      // constant power (%)

  // Explicit OpenDSS-style split ZIP weights for active/reactive demand.
  // Negative values mean "unset", in which case the legacy shared ZIP fields
  // above are applied to both active and reactive demand.
  double p_const_z_percent{-1.0};
  double p_const_i_percent{-1.0};
  double p_const_p_percent{-1.0};
  double q_const_z_percent{-1.0};
  double q_const_i_percent{-1.0};
  double q_const_p_percent{-1.0};

  // Motor load fraction (for short-circuit / transient analysis)
  double motor_percent{0.0};          // percentage of motor load
  double lrc_pu{0.0};                 // locked rotor current (pu)
  double x_r_ratio{0.0};              // X/R ratio of motor load
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase Generator
//
// Generator with sequence impedance data for unbalanced analysis.
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseGenerator {
  int index{0};
  int bus{0};
  std::string name;
  bool in_service{true};
  bool is_slack{false};
  PhaseMask phase_mask{PhaseMask::abc()};

  // Legacy aggregate dispatch. Used only when all per-phase dispatch fields
  // remain zero.
  double p_mw{0.0};
  double q_mvar{0.0};

  // Per-phase dispatch override. When any of these fields is non-zero, the
  // NR path consumes them directly instead of splitting p_mw/q_mvar equally.
  double p_a_mw{0.0};  double q_a_mvar{0.0};
  double p_b_mw{0.0};  double q_b_mvar{0.0};
  double p_c_mw{0.0};  double q_c_mvar{0.0};

  double vm_pu{1.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double mbase_mva{0.0};              // machine base MVA

  // Sequence impedances (pu on machine base)
  double xd_pu{0.0};                  // positive sequence (d-axis synchronous)
  double xdpp_pu{0.0};                // positive sequence subtransient
  double x2_pu{0.0};                  // negative sequence reactance
  double x0_pu{0.0};                  // zero sequence reactance
  double r0_pu{0.0};                  // zero sequence resistance
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase External Grid
//
// Grid equivalent with sequence impedances for fault analysis.
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseExternalGrid {
  int index{0};
  int bus{0};
  std::string name;
  bool in_service{true};
  PhaseMask phase_mask{PhaseMask::abc()};

  // Legacy balanced source voltage. Used when use_phase_voltage_setpoint=false.
  double vm_pu{1.0};
  double va_deg{0.0};

  // Optional per-phase source voltage setpoint. When enabled, active phases
  // consume these explicit phasors instead of splitting vm_pu/va_deg.
  // When source_topology is delta-like, the a/b/c slots map to AB/BC/CA.
  bool use_phase_voltage_setpoint{false};
  double vm_a_pu{0.0};  double va_a_deg{0.0};
  double vm_b_pu{0.0};  double va_b_deg{0.0};
  double vm_c_pu{0.0};  double va_c_deg{0.0};

  // Optional voltage-source topology contract.
  // Empty => direct phase-domain source in the bus phase basis.
  // Wye-like labels use "A,B,C"; delta-like labels use "AB,BC,CA".
  std::string source_topology;

  // Short-circuit capacity
  double s_sc_max_mva{0.0};
  double s_sc_min_mva{0.0};
  double rx_max{0.0};
  double rx_min{0.0};

  // Positive sequence impedance (pu)
  double r1_pu{0.0};
  double x1_pu{0.0};

  // Negative sequence impedance (pu)
  double r2_pu{0.0};
  double x2_pu{0.0};

  // Zero sequence impedance (pu)
  double r0_pu{0.0};
  double x0_pu{0.0};

  // Optional per-phase Thevenin impedance override. When enabled, active
  // slots consume these diagonal impedances instead of the
  // sequence-derived coupled impedance model above. When source_topology is
  // delta-like, the a/b/c slots map to AB/BC/CA.
  bool use_phase_impedance{false};
  double r_a_pu{0.0};  double x_a_pu{0.0};
  double r_b_pu{0.0};  double x_b_pu{0.0};
  double r_c_pu{0.0};  double x_c_pu{0.0};

  // Optional full phase-domain Thevenin impedance matrix override. When
  // enabled, NR consumes this coupled slot matrix before falling back to the
  // diagonal or sequence contracts above. When source_topology is empty or
  // wye-like, slots map to A/B/C; when delta-like, slots map to AB/BC/CA.
  bool use_phase_impedance_matrix{false};
  PhaseValueMatrix3 r_matrix_pu{};
  PhaseValueMatrix3 x_matrix_pu{};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase AC System (container)
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseACSystem {
  std::vector<ThreePhaseACBus> buses;
  std::vector<ThreePhaseACLine> lines;
  std::vector<ThreePhaseTransformer> transformers;
  std::vector<ThreePhaseLoad> loads;
  std::vector<ThreePhaseGenerator> generators;
  std::vector<ThreePhaseExternalGrid> external_grids;
  std::vector<ThreePhaseRegulatorControl> regulator_controls;

  double base_mva{100.0};
  double base_freq_hz{50.0};
  std::string name{"Three-Phase AC System"};
};

// ═══════════════════════════════════════════════════════════════════════
// Three-Phase Bus Result (per-phase voltage and power output)
// ═══════════════════════════════════════════════════════════════════════
struct ThreePhaseBusResult {
  int bus_id{0};

  // Per-phase voltage
  double vm_a_pu{0.0};  double va_a_deg{0.0};
  double vm_b_pu{0.0};  double va_b_deg{0.0};
  double vm_c_pu{0.0};  double va_c_deg{0.0};

  // Per-phase power injection
  double p_a_mw{0.0};   double q_a_mvar{0.0};
  double p_b_mw{0.0};   double q_b_mvar{0.0};
  double p_c_mw{0.0};   double q_c_mvar{0.0};

  bool is_unbalanced{false};
};

}  // namespace hacdcpf
