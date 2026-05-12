#pragma once

#include <string>

#include "hacdcpf/model/enums/generator_enums.hpp"
#include "hacdcpf/model/enums/renewable_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Generator (enriched for UC / planning / stability / carbon)
// ═══════════════════════════════════════════════════════════════════════
struct Generator {
  int index{0};
  int bus{0};
  bool in_service{true};
  double pg_mw{0.0};               // active generation (positive = injection into bus)
  double qg_mvar{0.0};             // reactive generation (positive = injection into bus)
  double vg_pu{1.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  bool is_slack{false};

  // MATPOWER polynomial generation cost:
  // f(Pg_MW) = cost_c2 * Pg_MW^2 + cost_c1 * Pg_MW + cost_c0
  double cost_c2{0.0};
  double cost_c1{0.0};
  double cost_c0{0.0};

  std::string name;

  // Unit commitment parameters
  FuelType fuel_type{FuelType::Unknown};
  double startup_cost{0.0};    // $/start
  double shutdown_cost{0.0};   // $/stop
  double min_up_time_hr{0.0};  // minimum on time (hours)
  double min_dn_time_hr{0.0};  // minimum off time (hours)
  double ramp_up_mw_min{0.0};  // ramp rate up (MW/min)
  double ramp_dn_mw_min{0.0};  // ramp rate down (MW/min)
  int max_startups_per_day{0};   // §2.6.3.13 max startups per day (0 = unlimited)
  int max_shutdowns_per_day{0};  // §2.6.3.13 max shutdowns per day (0 = unlimited)
  double mbase_mva{0.0};       // machine base MVA

  // Stability parameters
  double inertia_h{0.0};       // inertia constant H (s), on mbase
  double droop_r{0.05};        // governor droop (pu)
  double xd_pu{0.0};           // d-axis synchronous reactance (pu)
  double xdp_pu{0.0};          // d-axis transient reactance (pu)
  double xdpp_pu{0.0};         // d-axis subtransient reactance (pu)
  double td0p_s{0.0};          // d-axis open-circuit transient time constant (s)
  double td0pp_s{0.0};         // d-axis open-circuit subtransient time constant (s)

  // Short-circuit parameters (IEC 60909)
  double ra_pu{0.0};           // armature resistance (pu on mbase)
  double vn_kv{0.0};           // rated voltage (kV)
  double cos_phi{1.0};         // rated power factor
  double xq_pu{0.0};           // q-axis synchronous reactance (pu)
  double xd_xq{1.0};           // Xd/Xq ratio (for steady-state SC, salient/hidden pole)
  double x0_pu{0.0};           // zero-sequence reactance (pu on mbase)
  double r0_pu{0.0};           // zero-sequence resistance (pu on mbase)

  // Carbon emission
  double emission_factor_tco2_mwh{0.0};  // tCO2/MWh
  double nox_factor_kg_mwh{0.0};         // kg NOx/MWh
  double so2_factor_kg_mwh{0.0};         // kg SO2/MWh

  // Time-series hook (index into external profile, -1 = none)
  int profile_id{-1};

  // Reliability
  double forced_outage_rate{0.0};  // probability of forced outage
  double mttr_hr{0.0};             // mean time to repair (hours)
  double t_scheduled_hr{0.0};      // scheduled maintenance (hours/year)
};

// ═══════════════════════════════════════════════════════════════════════
// Static Generator (distributed generation: PV inverter, wind, CHP, ...)
// ═══════════════════════════════════════════════════════════════════════
struct StaticGenerator {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  SgenType sgen_type{SgenType::Other};

  double p_mw{0.0};               // active output (positive = injection into bus)
  double q_mvar{0.0};             // reactive output (positive = injection into bus)
  double p_rated_mw{0.0};
  double sn_mva{0.0};             // rated apparent power
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double scaling{1.0};            // scaling factor [0,1]

  bool controllable{false};
  double v_ref_pu{1.0};           // voltage setpoint
  double k_p{0.0};                // droop gain (active)
  double k_q{0.0};                // droop gain (reactive)

  // Short-circuit
  double k{1.0};                  // short-circuit current ratio
  double rx{0.0};                 // R/X ratio

  // Carbon
  double co2_emission_rate{0.0};  // tCO2/MWh

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};     // scheduled maintenance (hours/year)

  // Frequency reference (droop control)
  double f_ref_hz{50.0};          // reference frequency (Hz)
};

// ═══════════════════════════════════════════════════════════════════════
// Renewable Generator (wind / solar, non-dispatchable)
// ═══════════════════════════════════════════════════════════════════════
struct RenewableGen {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  RenewableType type{RenewableType::Wind};

  double p_mw{0.0};               // current output (positive = injection into bus)
  double q_mvar{0.0};             // reactive output (positive = injection into bus)
  double p_rated_mw{0.0};         // rated capacity
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  // Curtailability
  bool curtailable{true};
  double cost_curtail_mwh{0.0};   // opportunity cost of curtailment

  // Capacity factor (for planning)
  double capacity_factor{0.3};

  // Time-series hook (index into external profile, -1 = none)
  int profile_id{-1};

  // Carbon offset
  double emission_offset_tco2_mwh{0.0};  // avoided tCO2/MWh

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};     // scheduled maintenance (hours/year)
};

// ═══════════════════════════════════════════════════════════════════════
// PV System (solar PV with inverter — more detailed than RenewableGen)
// ═══════════════════════════════════════════════════════════════════════
struct PVSystem {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  // Power output (positive = injection into bus)
  double p_mw{0.0};
  double q_mvar{0.0};
  double sn_mva{0.0};             // rated apparent power
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  // Control
  PVControlMode control_mode{PVControlMode::MPPT};
  bool controllable{false};
  double v_ac_set_pu{1.0};
  double v_dc_set_pu{1.0};

  // Inverter
  double inverter_eff{0.97};
  double loss_percent{0.0};

  // PV array parameters
  int num_series{0};
  int num_parallel{0};
  double vmpp{0.0};               // voltage at MPP (V)
  double impp{0.0};               // current at MPP (A)
  double voc{0.0};                // open-circuit voltage (V)
  double isc{0.0};                // short-circuit current (A)
  double alpha_isc{0.0};          // temp coefficient of Isc (%/C)
  double beta_voc{0.0};           // temp coefficient of Voc (%/C)

  // Environmental
  double irradiance{1000.0};      // W/m2
  double temperature{25.0};       // C

  // Time-series hook (index into external profile, -1 = none)
  int profile_id{-1};

  // System-level reliability
  double mtbf_hours{0.0};            // system-level MTBF (hours)
  double mttr_hours{0.0};            // system-level MTTR (hours)
  double t_scheduled_hr{0.0};        // scheduled maintenance (hours/year)

  // Sub-system reliability (panel + inverter)
  double mtbf_panel_hours{0.0};
  double mttr_panel_hours{0.0};
  double mtbf_inverter_hours{0.0};
  double mttr_inverter_hours{0.0};
};

}  // namespace hacdcpf
