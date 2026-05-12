#pragma once

#include <string>

#include "hacdcpf/model/enums/bus_types.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// AC Bus
// ═══════════════════════════════════════════════════════════════════════
struct ACBus {
  int index{0};
  BusType bus_type{BusType::PQ};
  double pd_mw{0.0};
  double qd_mvar{0.0};
  double vm_pu{1.0};
  double va_deg{0.0};
  int area{1};
  double base_kv{110.0};
  double vmax_pu{1.1};
  double vmin_pu{0.9};
  double gs_mw{0.0};
  double bs_mvar{0.0};
  int zone{1};
  bool in_service{true};
  std::string name;

  // Resilience / customer data (for planning, load priority)
  int n_customers{0};        // number of customers at this bus
  double importance{1.0};    // weight for resilience metrics

  // Bearing capability assessment (DL/T 2041)
  double i_breaker_ka{0.0};   // breaker interrupting capacity at this bus [kA]

  // Geographical (for planning / visualization)
  double latitude{0.0};
  double longitude{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// AC Branch
// ═══════════════════════════════════════════════════════════════════════
struct ACBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  double r_pu{0.0};
  double x_pu{0.0};
  double b_pu{0.0};
  double tap{1.0};
  double shift_deg{0.0};
  double rate_a_mva{0.0};
  bool in_service{true};
  std::string name;

  // Additional thermal ratings
  double rate_b_mva{0.0};    // short-term rating
  double rate_c_mva{0.0};    // emergency rating

  // Physical parameters (for stability / EMT)
  double length_km{0.0};

  // Per-km electrical parameters (for line impedance calculation)
  // When set (>0), they determine r_pu, x_pu, b_pu via:
  //   r_pu = r_ohm_per_km * length_km / z_base
  //   x_pu = x_ohm_per_km * length_km / z_base
  //   b_pu = b_us_per_km * length_km * z_base * 1e-6
  // where z_base = base_kv² / base_mva
  double r_ohm_per_km{0.0};  // resistance (Ω/km)
  double x_ohm_per_km{0.0};  // reactance (Ω/km)
  double b_us_per_km{0.0};   // susceptance (μS/km)
  double c_nf_per_km{0.0};   // capacitance (nF/km), alternative to b_us_per_km

  // Reliability (for planning)
  double failure_rate{0.0};  // failures per year per km
  double mttr_hr{0.0};       // mean time to repair (hours)
  double t_scheduled_hr{0.0}; // scheduled maintenance (hours/year)

  // Parallel circuits
  int n_parallel{1};          // number of parallel circuits

  // Zero-sequence parameters (for short-circuit analysis)
  double r0_pu{0.0};         // zero-sequence resistance (pu)
  double x0_pu{0.0};         // zero-sequence reactance (pu)
  double b0_pu{0.0};         // zero-sequence susceptance (pu)

  // Transformer nameplate (used by SC correction factors)
  double vn_hv_kv{0.0};      // rated HV-side voltage (kV)
  double vn_lv_kv{0.0};      // rated LV-side voltage (kV)
  double sn_mva{0.0};        // rated apparent power (MVA)
};

// ═══════════════════════════════════════════════════════════════════════
// Two-winding Transformer (explicit model, separate from ACBranch)
// ═══════════════════════════════════════════════════════════════════════
struct Transformer2W {
  int index{0};
  std::string name;
  std::string std_type;            // standard type name
  int hv_bus{0};
  int lv_bus{0};
  bool in_service{true};

  double sn_mva{0.0};             // rated power
  double vn_hv_kv{0.0};           // rated HV voltage
  double vn_lv_kv{0.0};           // rated LV voltage

  // Short-circuit parameters (percentage)
  double vk_percent{0.0};         // short-circuit voltage
  double vkr_percent{0.0};        // real part of vk
  double pk_kw{0.0};              // short-circuit losses

  // No-load parameters
  double pfe_kw{0.0};             // iron losses
  double i0_percent{0.0};         // no-load current

  // Tap changer
  int tap_side{0};                // 0=HV, 1=LV
  int tap_pos{0};
  int tap_min{0};
  int tap_max{0};
  int tap_neutral{0};
  double tap_step_percent{0.0};

  // Phase shift
  double shift_deg{0.0};
  std::string vector_group;        // e.g. "Yyn0", "Dyn11"

  // Zero-sequence
  double z0_percent{0.0};
  double x0_r0{0.0};

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};  // scheduled maintenance (hours/year)

  // Parallel transformers
  int n_parallel{1};

  // When auto-extracted from a MATPOWER branch, stores the original
  // branch index so that project_to_canonical_models can update the
  // existing branch's tap rather than creating a duplicate with
  // round-trip impedance loss.  0 means this is a standalone
  // transformer (not derived from a branch).
  int source_branch_idx{0};
};
struct Transformer3W {
  int index{0};
  std::string name;
  std::string std_type;
  int hv_bus{0};
  int mv_bus{0};
  int lv_bus{0};
  bool in_service{true};

  double sn_hv_mva{0.0};
  double sn_mv_mva{0.0};
  double sn_lv_mva{0.0};
  double vn_hv_kv{0.0};
  double vn_mv_kv{0.0};
  double vn_lv_kv{0.0};

  // Pair-wise short-circuit impedances
  double vk_hv_mv_percent{0.0};
  double vk_hv_lv_percent{0.0};
  double vk_mv_lv_percent{0.0};
  double vkr_hv_mv_percent{0.0};
  double vkr_hv_lv_percent{0.0};
  double vkr_mv_lv_percent{0.0};

  // No-load
  double pfe_kw{0.0};
  double i0_percent{0.0};

  // Tap changer
  int tap_side{0};
  int tap_pos{0};
  double tap_step_percent{0.0};

  // Phase shifts
  double shift_mv_deg{0.0};
  double shift_lv_deg{0.0};

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};  // scheduled maintenance (hours/year)
};

// ═══════════════════════════════════════════════════════════════════════
// External Grid (grid equivalent / infinite bus)
// ═══════════════════════════════════════════════════════════════════════
struct ExternalGrid {
  int index{0};
  std::string name;
  int bus{0};
  bool in_service{true};

  double vm_pu{1.0};
  double va_deg{0.0};

  // Short-circuit capacity
  double s_sc_max_mva{0.0};
  double s_sc_min_mva{0.0};
  double rx_max{0.0};
  double rx_min{0.0};

  // Impedance (pu on system base)
  double r_pu{0.0};
  double x_pu{0.0};
  double r0_pu{0.0};
  double x0_pu{0.0};

  double vn_kv{0.0};
  double ikq_ka{0.0};         // initial three-phase SC current from grid (kA, IEC 60909)
  double x_r{0.0};            // X/R ratio of grid impedance
  bool controllable{true};
};

// ═══════════════════════════════════════════════════════════════════════
// Asynchronous Motor (for IEC 60909 short circuit analysis)
// ═══════════════════════════════════════════════════════════════════════
struct AsynchronousMotor {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double vn_kv{0.0};          // rated voltage (kV)
  double sn_mva{0.0};         // rated apparent power (MVA)
  double r_pu{0.0};           // resistance (ohm)
  double x_pu{0.0};           // reactance (ohm)
  double x_r{0.0};            // X/R ratio
  double lrc{0.0};            // locked-rotor current ratio
  int    poles{2};            // number of pole pairs
  double cos_phi{0.85};       // rated power factor
  double efficiency{0.95};    // rated efficiency
  double r0_pu{0.0};          // zero-sequence resistance (ohm)
  double x0_pu{0.0};          // zero-sequence reactance (ohm)
};

}  // namespace hacdcpf
