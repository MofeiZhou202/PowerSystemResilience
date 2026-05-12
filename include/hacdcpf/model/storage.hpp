#pragma once

#include <string>

#include "hacdcpf/model/enums/storage_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Energy Storage (battery, pumped hydro, etc.)
// ═══════════════════════════════════════════════════════════════════════
struct Storage {
  int index{0};
  int bus{0};
  bool in_service{true};
  std::string name;

  // Power capacity
  double p_mw{0.0};               // current dispatch (positive = discharge/injection, negative = charge/consumption)
  double q_mvar{0.0};             // reactive power (positive = injection into bus)
  double p_rated_mw{0.0};         // rated power
  double pmax_mw{0.0};            // max discharge
  double pmin_mw{0.0};            // max charge (negative)
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};

  // Energy capacity
  double e_rated_mwh{0.0};        // rated energy capacity
  double soc_init{0.5};           // initial state of charge [0,1]
  double soc_min{0.1};            // minimum SOC
  double soc_max{0.9};            // maximum SOC
  double soc_carbon_intensity_tco2_mwh{0.0};  // carbon intensity of stored energy at the terminal SOC state

  // Efficiency
  double eta_charge{0.95};        // charging efficiency
  double eta_discharge{0.95};     // discharging efficiency
  double self_discharge_pct{0.0}; // self-discharge rate (%/hour)

  // Lifecycle (for planning)
  int max_cycles{5000};           // rated cycle life
  int current_cycles{0};          // accumulated cycles
  double soh{1.0};                // state of health [0,1]
  double soh_cycle{1.0};          // cycle-based SOH
  double soh_calendar{1.0};       // calendar-based SOH
  double l_calendar_yr{15.0};     // calendar life (years)
  double eol_percent{0.8};        // end-of-life SOH threshold
  double replacement_cost{0.0};   // $/kWh replacement

  // Current energy state
  double e_mwh{0.0};              // current stored energy (MWh)

  // Time-series hook (index into external profile, -1 = none)
  int profile_id{-1};

  // Control
  bool controllable{true};
  std::string control_mode;       // e.g. "self_consumption", "peak_shaving", "market"
  std::string type;               // e.g. "Li-ion", "flow", "pumped_hydro"

  // Market bidding (§2.6.3)
  double charge_bid_price{0.0};      // λ^ch: charging bid price [$/MWh]
  double discharge_bid_price{0.0};   // λ^dis: discharging bid price [$/MWh]
  double daily_cycle_limit{0.0};     // §2.6.3.16(5): max daily cycles (0 = unlimited)

  // System-level reliability
  double forced_outage_rate{0.0};
  double mttr_hr{0.0};
  double t_scheduled_hr{0.0};

  // Sub-system reliability (battery cell/pack, PCS, BMS)
  double mtbf_battery_hr{0.0};
  double mttr_battery_hr{0.0};
  double mtbf_pcs_hr{0.0};
  double mttr_pcs_hr{0.0};
  double mtbf_bms_hr{0.0};
  double mttr_bms_hr{0.0};
};

// ═══════════════════════════════════════════════════════════════════════
// Mobile Storage (EV battery trailer / transportable ESS)
// ═══════════════════════════════════════════════════════════════════════
struct MobileStorage {
  int index{0};
  int bus{0};                      // current connected bus
  bool in_service{true};
  std::string name;

  // Same sign convention as Storage
  double p_mw{0.0};               // current dispatch (positive = discharge/injection, negative = charge/consumption)
  double q_mvar{0.0};             // reactive power (positive = injection into bus)
  double p_rated_mw{0.0};
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double qmax_mvar{0.0};
  double qmin_mvar{0.0};
  double e_rated_mwh{0.0};
  double soc_init{0.5};
  double soc_min{0.1};
  double soc_max{0.9};
  double eta_charge{0.95};
  double eta_discharge{0.95};

  // Health
  int max_cycles{5000};
  int current_cycles{0};
  double soh{1.0};
  double soh_cycle{1.0};
  double soh_calendar{1.0};
  double l_calendar_yr{10.0};
  double eol_percent{0.8};

  // Current energy state
  double e_mwh{0.0};

  // Control
  bool controllable{true};
  std::string type;               // e.g. "Li-ion", "NMC"

  // Mobility
  bool is_mobile{true};
  MobileStorageStatus status{MobileStorageStatus::Stationary};
  std::string current_location;
  int target_bus{0};
  double e_consumption_mwh_km{0.0};  // energy cost of transport
  double max_travel_distance_km{0.0};
  double arrival_time{0.0};          // hours from start of horizon
  double departure_time{0.0};
  double t_stay_min_hr{0.0};
  double t_stay_max_hr{0.0};

  // System-level reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};

  // Sub-system reliability
  double mtbf_battery_hr{0.0};
  double mttr_battery_hr{0.0};
  double mtbf_pcs_hr{0.0};
  double mttr_pcs_hr{0.0};
  double mtbf_bms_hr{0.0};
  double mttr_bms_hr{0.0};
  double mtbf_vehicle_hr{0.0};
  double mttr_vehicle_hr{0.0};
};

}  // namespace hacdcpf
