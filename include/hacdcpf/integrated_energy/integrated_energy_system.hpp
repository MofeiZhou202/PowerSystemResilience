#pragma once

#include <string>
#include <vector>

namespace hacdcpf::integrated_energy {

struct CampusIESData {
  int num_steps{24};
  double step_duration_hr{1.0};

  /// Stable AC bus label used only to attribute the aggregate PCC exchange.
  /// solve_campus_ies() does not receive or solve an electrical network.
  int pcc_ac_bus{0};
  double import_limit_mw{20.0};
  double export_limit_mw{20.0};
  /// Reserved compatibility field. Values other than unity are rejected until
  /// reactive-power and voltage constraints are part of the solved model.
  double fixed_power_factor{1.0};

  std::vector<double> electric_load_mw;
  std::vector<double> heat_load_mw;
  std::vector<double> hydrogen_load_mw;
  std::vector<double> fuel_load_mw;
  std::vector<double> transport_demand_km;
  std::vector<double> solar_available_mw;
  std::vector<double> wind_available_mw;
  std::vector<double> grid_buy_price;
  std::vector<double> grid_sell_price;
  std::vector<double> grid_carbon_tco2_mwh;

  double solar_rated_mw{8.0};
  double wind_rated_mw{4.0};
  double chp_power_max_mw{4.0};
  double chp_heat_max_mw{6.0};
  double heat_pump_power_max_mw{3.0};
  double electrolyzer_power_max_mw{3.0};
  double fuel_cell_power_max_mw{2.0};
  double fuel_purchase_limit_mw{30.0};

  double electric_storage_capacity_mwh{10.0};
  double electric_storage_min_mwh{0.0};
  double electric_storage_initial_mwh{2.0};
  double electric_storage_charge_max_mw{4.0};
  double electric_storage_discharge_max_mw{4.0};

  double thermal_storage_capacity_mwh{8.0};
  double thermal_storage_min_mwh{0.0};
  double thermal_storage_initial_mwh{2.0};
  double thermal_storage_charge_max_mw{3.0};
  double thermal_storage_discharge_max_mw{3.0};

  double hydrogen_storage_capacity_mwh{8.0};
  double hydrogen_storage_min_mwh{0.0};
  double hydrogen_storage_initial_mwh{2.0};
  double hydrogen_storage_charge_max_mw{2.0};
  double hydrogen_storage_discharge_max_mw{2.0};

  double weekly_hydrogen_storage_capacity_mwh{20.0};
  double weekly_hydrogen_storage_min_mwh{0.0};
  double weekly_hydrogen_storage_initial_mwh{6.0};
  double weekly_hydrogen_storage_charge_max_mw{1.0};
  double weekly_hydrogen_storage_discharge_max_mw{1.0};

  double seasonal_hydrogen_storage_capacity_mwh{60.0};
  double seasonal_hydrogen_storage_min_mwh{0.0};
  double seasonal_hydrogen_storage_initial_mwh{18.0};
  double seasonal_hydrogen_storage_charge_max_mw{0.8};
  double seasonal_hydrogen_storage_discharge_max_mw{0.8};

  double daily_weekly_hydrogen_transfer_max_mw{0.8};
  double weekly_seasonal_hydrogen_transfer_max_mw{0.5};

  double eta_electrolysis{0.65};
  double eta_power_to_fuel{0.50};
  double eta_fuelcell{0.52};
  double cop_heatpump{3.2};
  double eta_wasteheat{0.0};
  double eta_chp_elec{0.35};
  double eta_chp_heat{0.45};
  double eta_chp_total{0.82};

  double eta_storage_charge{0.95};
  double eta_storage_discharge{0.95};
  double electric_storage_retention{0.999};
  double thermal_storage_retention{0.995};
  double hydrogen_storage_retention{0.999};
  double weekly_hydrogen_storage_retention{0.9995};
  double seasonal_hydrogen_storage_retention{0.9998};

  double eta_daily_to_weekly{0.95};
  double eta_weekly_to_daily{0.94};
  double eta_weekly_to_seasonal{0.93};
  double eta_seasonal_to_weekly{0.92};

  double ev_ratio{0.45};
  double hv_ratio{0.20};
  double icv_ratio{0.35};
  double alpha_ev_mwh_per_km{0.00018};
  double alpha_hv_mwh_per_km{0.00060};
  double alpha_icv_mwh_per_km{0.00075};

  double fuel_carbon_tco2_mwh{0.27};
  double co2_budget_tco2{80.0};
  double ccus_capture_fraction{0.85};
  double ccus_max_tco2_per_h{3.0};
  double ccus_power_mwh_per_tco2{0.12};
  double eta_carbon_to_fuel{0.0};

  double fuel_cost_per_mwh{38.0};
  double solar_om_cost_per_mwh{2.0};
  double wind_om_cost_per_mwh{3.0};
  double chp_om_cost_per_mwh{5.0};
  double heat_pump_om_cost_per_mwh{1.0};
  double electrolyzer_om_cost_per_mwh{2.0};
  double fuel_cell_om_cost_per_mwh{4.0};
  double storage_throughput_cost_per_mwh{1.0};
  double hydrogen_storage_throughput_cost_per_mwh{1.5};
  double curtailment_cost_per_mwh{15.0};
  double carbon_penalty_per_tco2{80.0};
  double ccus_cost_per_tco2{35.0};
  double ccus_power_cost_per_mwh{0.0};

  double renewable_mandate_fraction{0.0};
};

CampusIESData make_sample_campus_ies_data(int num_steps = 24);

}  // namespace hacdcpf::integrated_energy
