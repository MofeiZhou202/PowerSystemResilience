#pragma once

#include <string>
#include <vector>

#include "hacdcpf/model/enums/grid_enums.hpp"

namespace hacdcpf {

// ═══════════════════════════════════════════════════════════════════════
// Virtual Power Plant (aggregation of DERs)
// ═══════════════════════════════════════════════════════════════════════
struct VirtualPowerPlant {
  int index{0};
  std::string name;
  std::string description;
  int aggregation_bus{0};
  bool in_service{true};

  // Fleet composition
  int n_pv_systems{0};
  int n_wind_turbines{0};
  int n_battery_systems{0};
  int n_ev_chargers{0};
  int n_controllable_loads{0};
  int n_chp{0};
  int n_biomass{0};
  int n_thermal_storage{0};
  int n_hvac{0};
  int n_industrial{0};

  // Aggregated capacity
  double p_generation_sum_mw{0.0};
  double e_storage_sum_mwh{0.0};
  double p_load_controllable_mw{0.0};
  double p_pv_sum_mw{0.0};
  double p_wind_sum_mw{0.0};

  // Regulation capability
  double p_regulation_up_mw{0.0};
  double p_regulation_down_mw{0.0};

  // Operational
  double p_output_mw{0.0};         // net active output (positive = injection into aggregation bus)
  double q_output_mvar{0.0};       // net reactive output (positive = injection into aggregation bus)
  double pmax_mw{0.0};
  double pmin_mw{0.0};
  double ramp_up_max_mw_min{0.0};
  double ramp_down_max_mw_min{0.0};

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};  double t_scheduled_hr{0.0};};

// ═══════════════════════════════════════════════════════════════════════
// Microgrid
// ═══════════════════════════════════════════════════════════════════════
struct Microgrid {
  int index{0};
  std::string name;
  std::string description;
  bool in_service{true};

  int aggregation_bus{0};          // point of common coupling bus
  std::vector<int> internal_buses;

  MicrogridMode operating_mode{MicrogridMode::GridConnected};
  bool islanding_capability{false};
  bool auto_reconnection{false};

  // Power exchange limits
  double p_exchange_max_mw{0.0};
  double p_exchange_min_mw{0.0};
  double p_import_max_mw{0.0};
  double p_export_max_mw{0.0};
  double p_exchange_mw{0.0};      // current exchange (positive = export from microgrid to main grid)

  // Aggregated capacity
  double total_generation_mw{0.0};
  double total_storage_mwh{0.0};
  double total_load_mw{0.0};
  double capacity_mw{0.0};
  double peak_load_mw{0.0};
  double total_dg_capacity_mw{0.0};
  double total_diesel_capacity_mw{0.0};

  // Control setpoints
  double f_set_hz{50.0};
  double v_set_pu{1.0};
  double k_droop{0.0};
  double p_set_mw{0.0};            // net power exchange setpoint
  std::string control_area;        // identifier for the control area

  // Protection limits
  double f_max_hz{52.0};
  double f_min_hz{47.0};
  double v_max_pu{1.1};
  double v_min_pu{0.9};

  int area{0};

  // Reliability
  double mtbf_hours{0.0};
  double mttr_hours{0.0};
  double t_scheduled_hr{0.0};
};

}  // namespace hacdcpf
