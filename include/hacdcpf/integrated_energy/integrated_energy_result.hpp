#pragma once

#include <string>
#include <vector>

namespace hacdcpf::integrated_energy {

struct CampusIESSankeyFlow {
  std::string source;
  std::string target;
  double value_mwh{0.0};
  std::string carrier;
};

struct CampusIESValidity {
  bool multi_carrier_balances_modelled{true};
  bool aggregate_pcc_active_power_modelled{true};
  bool electrical_network_coupled{false};
  bool reactive_power_modelled{false};
  bool voltage_and_branch_limits_enforced{false};
};

struct CampusIESResult {
  bool feasible{false};
  bool optimal{false};
  std::string solver_name;
  std::string status;
  double objective{0.0};
  double solve_time_sec{0.0};
  std::string model_scope{"isolated-campus-multi-carrier-milp"};
  CampusIESValidity validity;
  std::vector<std::string> model_limitations;
  int pcc_ac_bus{0};

  std::vector<double> p_grid_import_mw;
  std::vector<double> p_grid_export_mw;
  std::vector<double> p_pcc_mw;
  std::vector<double> p_solar_mw;
  std::vector<double> p_wind_mw;
  std::vector<double> p_solar_curtail_mw;
  std::vector<double> p_wind_curtail_mw;

  std::vector<double> p_chp_mw;
  std::vector<double> q_chp_mw;
  std::vector<double> f_chp_mw;
  std::vector<double> f_external_mw;
  std::vector<double> f_synthetic_mw;
  std::vector<double> p_fuelcell_mw;
  std::vector<double> h_fuelcell_mw;

  std::vector<double> p_heatpump_mw;
  std::vector<double> q_heatpump_mw;
  std::vector<double> p_electrolysis_h2_mw;
  std::vector<double> p_electrolysis_fuel_mw;
  std::vector<double> h_electrolysis_mw;

  std::vector<double> p_storage_charge_mw;
  std::vector<double> p_storage_discharge_mw;
  std::vector<double> e_storage_mwh;
  std::vector<double> q_storage_charge_mw;
  std::vector<double> q_storage_discharge_mw;
  std::vector<double> q_storage_mwh;
  std::vector<double> h_storage_charge_mw;
  std::vector<double> h_storage_discharge_mw;
  std::vector<double> h_storage_mwh;

  std::vector<double> h_weekly_charge_mw;
  std::vector<double> h_weekly_discharge_mw;
  std::vector<double> h_weekly_storage_mwh;
  std::vector<double> h_seasonal_charge_mw;
  std::vector<double> h_seasonal_discharge_mw;
  std::vector<double> h_seasonal_storage_mwh;
  std::vector<double> h_daily_to_weekly_mw;
  std::vector<double> h_weekly_to_daily_mw;
  std::vector<double> h_weekly_to_seasonal_mw;
  std::vector<double> h_seasonal_to_weekly_mw;

  std::vector<double> d_ev_km;
  std::vector<double> d_hv_km;
  std::vector<double> d_icv_km;
  std::vector<double> p_ev_charge_mw;
  std::vector<double> p_ev_v2g_mw;
  std::vector<double> h_hv_refuel_mw;
  std::vector<double> f_icv_refuel_mw;

  std::vector<double> emissions_tco2;
  std::vector<double> co2_captured_tco2;
  std::vector<double> carbon_residual_tco2;
  std::vector<double> p_ccus_mw;

  double total_cost{0.0};
  double total_grid_import_mwh{0.0};
  double total_grid_export_mwh{0.0};
  double total_renewable_mwh{0.0};
  double total_renewable_available_mwh{0.0};
  double total_curtailment_mwh{0.0};
  double total_electric_load_mwh{0.0};
  double total_heat_load_mwh{0.0};
  double total_hydrogen_load_mwh{0.0};
  double total_fuel_load_mwh{0.0};
  double total_transport_km{0.0};
  double total_emissions_tco2{0.0};
  double total_co2_captured_tco2{0.0};
  double total_carbon_residual_tco2{0.0};
  double renewable_utilization{0.0};

  std::vector<CampusIESSankeyFlow> sankey_flows;
};

}  // namespace hacdcpf::integrated_energy
