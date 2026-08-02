#pragma once

/// model/dc_components.hpp
/// =======================
/// Consolidated DC component types for the hacdcpf power system model.
/// Replaces: dc_network.hpp, loads.hpp (DCLoad), switching.hpp (DCCircuitBreaker).

#include <string>

#include "hacdcpf/model/dynamic_model_profile.hpp"
#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/enums/load_enums.hpp"
#include "hacdcpf/model/enums/switching_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// DC Bus
// ═══════════════════════════════════════════════════════════════════════
struct DCBus {
  int index{0};
  DCBusType bus_type{DCBusType::DC_P};
  double vm_pu{1.0};
  double vmax_pu{1.1};
  double vmin_pu{0.9};
  // Additive bus-level DC demand; explicit DCLoad rows are added on top.
  double pd_mw{0.0};
  // Carbon factor of the ideal balancing source represented by a DC_V bus.
  // Internal unit: tCO2/MWh (numerically equivalent to kgCO2/kWh).
  double emission_factor_tco2_mwh{0.0};
  bool in_service{true};
  std::string name;

  double base_kv{0.0};
  int area{0};
  int zone{0};

  int n_customers{0};
  double importance{1.0};
  bool is_load{false};

  double latitude{0.0};
  double longitude{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Branch
// ═══════════════════════════════════════════════════════════════════════
struct DCBranch {
  int index{0};
  int from_bus{0};
  int to_bus{0};
  double r_pu{0.0};
  bool in_service{true};
  std::string name;

  double rate_a_mva{0.0};
  double length_km{0.0};
  double base_kv{0.0};
  double s_max_mva{0.0};
  int n_parallel{1};

  // Actual (engineering) resistance per unit length.  When provided together
  // with length_km and a positive base voltage, convert_actual_to_per_unit()
  // fills r_pu from it (used only when r_pu is still zero).
  double r_ohm_per_km{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// DC Static Generator (PV, wind, ESS on DC bus)
// ═══════════════════════════════════════════════════════════════════════
struct StaticGeneratorDC {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;
  std::string type;

  double p_set_mw{0.0};
  double scaling{1.0};
  int profile_id{-1};
  double pmax_mw{0.0};
  double pmin_mw{0.0};

  bool controllable{false};

  double cost_c1{0.0};
  double emission_factor_tco2_mwh{0.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// DC PV Array
// ═══════════════════════════════════════════════════════════════════════
struct PVArrayDC {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  double p_set_mw{0.0};
  int profile_id{-1};
  double cost_c1{0.0};

  int num_series{0};
  int num_parallel{0};
  double vmpp{0.0};
  double impp{0.0};
  double voc{0.0};
  double isc{0.0};
  double alpha_isc{0.0};
  double beta_voc{0.0};

  double temperature{25.0};
  double irradiance{1000.0};

  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// DC Load
// ═══════════════════════════════════════════════════════════════════════
struct DCLoad {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;
  std::string type;

  double p_mw{0.0};
  double p_rated_mw{0.0};
  double scaling{1.0};

  double z_percent{0.0};
  double i_percent{0.0};
  double p_percent{100.0};

  bool controllable{false};
  double p_min_mw{0.0};
  double cost_mw{0.0};
  int profile_id{-1};
  LoadPriority priority{LoadPriority::Medium};

  // Reliability / planning: number of served customers at this DC load, used
  // for hybrid SAIFI/SAIDI customer weighting (mirrors AC Load::n_customers).
  int n_customers{0};

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// DC Energy Storage (battery, etc.) — DC-side, no reactive power
// ═══════════════════════════════════════════════════════════════════════
// Mirrors the AC `Storage` struct but omits all reactive-power fields, since a
// DC bus has no reactive power. Participates in the DC power flow as an active
// power injection (positive p_mw = discharge = generation at its DC bus) and in
// time-series power flow with the same SOC dynamics as AC storage.
struct DCStorage {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;
  std::string type;

  double p_mw{0.0};
  double p_rated_mw{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};

  double e_rated_mwh{0.0};
  double soc_init{0.5};
  double soc_min{0.1};
  double soc_max{0.9};
  double soc_carbon_intensity_tco2_mwh{0.0};

  double eta_charge{0.95};
  double eta_discharge{0.95};
  double self_discharge_pct{0.0};  // percent per hour

  int max_cycles{5000};
  int current_cycles{0};
  double soh{1.0};
  double l_calendar_yr{15.0};
  // Backward-compatible EOL threshold: <=1 is a fraction; >1 is percent.
  double eol_percent{0.8};
  double replacement_cost{0.0};

  double e_mwh{0.0};

  int profile_id{-1};

  bool controllable{true};

  double charge_bid_price{0.0};
  double discharge_bid_price{0.0};
  double daily_cycle_limit{0.0};

  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};

  // Hosting-capacity assessment (DL/T 2041-2025): charging strategy.
  // "opf"    = optimized in OPF/UC (default, as today);
  // "static" = fixed injection, excluded from optimization, held at cap_static_charging_mw.
  std::string cap_charging_strategy{"opf"};
  double cap_static_charging_mw{0.0};        // charging power (positive) used when strategy=="static" (P_ESS)

  DynamicModelProfile dynamic_model;
};

// ═══════════════════════════════════════════════════════════════════════
// DC Circuit Breaker
// ═══════════════════════════════════════════════════════════════════════
struct DCCircuitBreaker {
  int index{0};
  std::string name;
  int bus_from{0};
  int bus_to{0};
  bool in_service{true};

  BreakerType breaker_type{BreakerType::CB};
  bool closed{true};

  double r_ohm{0.0};
  double rated_voltage_kv{0.0};
  double i_rated_ka{0.0};
  double i_breaking_ka{0.0};

  std::string element_type;
  int element_id{0};
};

}  // namespace hacdcpf
