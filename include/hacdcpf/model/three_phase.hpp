#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/enums/bus_types.hpp"

namespace hacdcpf {

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
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
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

  std::string connection{"wye"};       // "wye" or "delta"
  bool grounded{true};

  // Per-phase power
  double p_a_mw{0.0};  double q_a_mvar{0.0};
  double p_b_mw{0.0};  double q_b_mvar{0.0};
  double p_c_mw{0.0};  double q_c_mvar{0.0};

  // ZIP model composition
  double const_z_percent{0.0};        // constant impedance (%)
  double const_i_percent{0.0};        // constant current (%)
  double const_p_percent{100.0};      // constant power (%)

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

  double p_mw{0.0};
  double q_mvar{0.0};
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

  double vm_pu{1.0};
  double va_deg{0.0};

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
