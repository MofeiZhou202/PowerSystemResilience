#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/integrated_energy/integrated_energy_optimizer.hpp"

namespace fs = std::filesystem;
using nlohmann::json;
using namespace hacdcpf::integrated_energy;

namespace {

void disable_non_electric_assets(CampusIESData& d) {
  d.heat_load_mw.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.hydrogen_load_mw.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.fuel_load_mw.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.transport_demand_km.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.wind_available_mw.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.grid_sell_price.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.grid_carbon_tco2_mwh.assign(static_cast<std::size_t>(d.num_steps), 0.0);
  d.wind_rated_mw = d.chp_power_max_mw = d.chp_heat_max_mw = 0.0;
  d.heat_pump_power_max_mw = d.electrolyzer_power_max_mw = 0.0;
  d.fuel_cell_power_max_mw = d.fuel_purchase_limit_mw = 0.0;
  d.thermal_storage_capacity_mwh = d.thermal_storage_min_mwh = 0.0;
  d.thermal_storage_initial_mwh = d.thermal_storage_charge_max_mw = 0.0;
  d.thermal_storage_discharge_max_mw = 0.0;
  d.hydrogen_storage_capacity_mwh = d.hydrogen_storage_min_mwh = 0.0;
  d.hydrogen_storage_initial_mwh = d.hydrogen_storage_charge_max_mw = 0.0;
  d.hydrogen_storage_discharge_max_mw = 0.0;
  d.weekly_hydrogen_storage_capacity_mwh = 0.0;
  d.weekly_hydrogen_storage_min_mwh = d.weekly_hydrogen_storage_initial_mwh = 0.0;
  d.weekly_hydrogen_storage_charge_max_mw = 0.0;
  d.weekly_hydrogen_storage_discharge_max_mw = 0.0;
  d.seasonal_hydrogen_storage_capacity_mwh = 0.0;
  d.seasonal_hydrogen_storage_min_mwh = d.seasonal_hydrogen_storage_initial_mwh = 0.0;
  d.seasonal_hydrogen_storage_charge_max_mw = 0.0;
  d.seasonal_hydrogen_storage_discharge_max_mw = 0.0;
  d.daily_weekly_hydrogen_transfer_max_mw = 0.0;
  d.weekly_seasonal_hydrogen_transfer_max_mw = 0.0;
  d.ccus_max_tco2_per_h = d.ccus_power_mwh_per_tco2 = 0.0;
  d.fuel_carbon_tco2_mwh = d.carbon_penalty_per_tco2 = 0.0;
  d.fuel_cost_per_mwh = d.wind_om_cost_per_mwh = 0.0;
  d.chp_om_cost_per_mwh = d.heat_pump_om_cost_per_mwh = 0.0;
  d.electrolyzer_om_cost_per_mwh = d.fuel_cell_om_cost_per_mwh = 0.0;
  d.hydrogen_storage_throughput_cost_per_mwh = 0.0;
  d.curtailment_cost_per_mwh = d.ccus_cost_per_tco2 = 0.0;
  d.ccus_power_cost_per_mwh = 0.0;
}

json result_json(const CampusIESResult& r) {
  return {
      {"feasible", r.feasible}, {"optimal", r.optimal},
      {"solver", r.solver_name}, {"status", r.status},
      {"objective", r.objective}, {"total_cost", r.total_cost},
      {"grid_import", r.p_grid_import_mw}, {"grid_export", r.p_grid_export_mw},
      {"solar", r.p_solar_mw}, {"wind", r.p_wind_mw},
      {"solar_curtail", r.p_solar_curtail_mw}, {"wind_curtail", r.p_wind_curtail_mw},
      {"chp_p", r.p_chp_mw}, {"chp_q", r.q_chp_mw}, {"chp_fuel", r.f_chp_mw},
      {"external_fuel", r.f_external_mw}, {"synthetic_fuel", r.f_synthetic_mw},
      {"fuelcell_p", r.p_fuelcell_mw}, {"fuelcell_h", r.h_fuelcell_mw},
      {"heatpump_p", r.p_heatpump_mw}, {"heatpump_q", r.q_heatpump_mw},
      {"electrolyzer_h2_p", r.p_electrolysis_h2_mw},
      {"electrolyzer_fuel_p", r.p_electrolysis_fuel_mw},
      {"electrolyzer_h", r.h_electrolysis_mw},
      {"e_charge", r.p_storage_charge_mw}, {"e_discharge", r.p_storage_discharge_mw},
      {"e_state", r.e_storage_mwh},
      {"q_charge", r.q_storage_charge_mw}, {"q_discharge", r.q_storage_discharge_mw},
      {"q_state", r.q_storage_mwh},
      {"h_charge", r.h_storage_charge_mw}, {"h_discharge", r.h_storage_discharge_mw},
      {"h_state", r.h_storage_mwh},
      {"hw_charge", r.h_weekly_charge_mw}, {"hw_discharge", r.h_weekly_discharge_mw},
      {"hw_state", r.h_weekly_storage_mwh},
      {"hs_charge", r.h_seasonal_charge_mw}, {"hs_discharge", r.h_seasonal_discharge_mw},
      {"hs_state", r.h_seasonal_storage_mwh},
      {"h_d2w", r.h_daily_to_weekly_mw}, {"h_w2d", r.h_weekly_to_daily_mw},
      {"h_w2s", r.h_weekly_to_seasonal_mw}, {"h_s2w", r.h_seasonal_to_weekly_mw},
      {"ev_km", r.d_ev_km}, {"hv_km", r.d_hv_km}, {"icv_km", r.d_icv_km},
      {"ev_p", r.p_ev_charge_mw}, {"hv_h", r.h_hv_refuel_mw},
      {"icv_f", r.f_icv_refuel_mw}, {"ccus_p", r.p_ccus_mw},
      {"emissions", r.emissions_tco2}, {"captured", r.co2_captured_tco2},
      {"residual_carbon", r.carbon_residual_tco2},
      {"total_import_mwh", r.total_grid_import_mwh},
      {"total_renewable_mwh", r.total_renewable_mwh},
      {"total_emissions", r.total_emissions_tco2},
      {"total_residual_carbon", r.total_carbon_residual_tco2}};
}

json input_json(const CampusIESData& d) {
  return {
      {"n", d.num_steps}, {"dt", d.step_duration_hr},
      {"electric_load", d.electric_load_mw}, {"heat_load", d.heat_load_mw},
      {"hydrogen_load", d.hydrogen_load_mw}, {"fuel_load", d.fuel_load_mw},
      {"transport_km", d.transport_demand_km}, {"solar_available", d.solar_available_mw},
      {"wind_available", d.wind_available_mw}, {"buy_price", d.grid_buy_price},
      {"sell_price", d.grid_sell_price}, {"grid_carbon", d.grid_carbon_tco2_mwh},
      {"eta_electrolysis", d.eta_electrolysis}, {"eta_power_to_fuel", d.eta_power_to_fuel},
      {"eta_fuelcell", d.eta_fuelcell}, {"cop_heatpump", d.cop_heatpump},
      {"eta_wasteheat", d.eta_wasteheat}, {"eta_chp_elec", d.eta_chp_elec},
      {"eta_chp_heat", d.eta_chp_heat}, {"eta_chp_total", d.eta_chp_total},
      {"eta_charge", d.eta_storage_charge}, {"eta_discharge", d.eta_storage_discharge},
      {"rho_e", d.electric_storage_retention}, {"rho_q", d.thermal_storage_retention},
      {"rho_h", d.hydrogen_storage_retention},
      {"rho_hw", d.weekly_hydrogen_storage_retention},
      {"rho_hs", d.seasonal_hydrogen_storage_retention},
      {"eta_d2w", d.eta_daily_to_weekly}, {"eta_w2d", d.eta_weekly_to_daily},
      {"eta_w2s", d.eta_weekly_to_seasonal}, {"eta_s2w", d.eta_seasonal_to_weekly},
      {"ev_ratio", d.ev_ratio}, {"hv_ratio", d.hv_ratio}, {"icv_ratio", d.icv_ratio},
      {"alpha_ev", d.alpha_ev_mwh_per_km}, {"alpha_hv", d.alpha_hv_mwh_per_km},
      {"alpha_icv", d.alpha_icv_mwh_per_km}, {"fuel_carbon", d.fuel_carbon_tco2_mwh},
      {"capture_fraction", d.ccus_capture_fraction},
      {"ccus_energy", d.ccus_power_mwh_per_tco2},
      {"fuel_cost", d.fuel_cost_per_mwh}, {"solar_om", d.solar_om_cost_per_mwh},
      {"wind_om", d.wind_om_cost_per_mwh}, {"chp_om", d.chp_om_cost_per_mwh},
      {"heatpump_om", d.heat_pump_om_cost_per_mwh},
      {"electrolyzer_om", d.electrolyzer_om_cost_per_mwh},
      {"fuelcell_om", d.fuel_cell_om_cost_per_mwh},
      {"storage_cost", d.storage_throughput_cost_per_mwh},
      {"h_storage_cost", d.hydrogen_storage_throughput_cost_per_mwh},
      {"curtailment_cost", d.curtailment_cost_per_mwh},
      {"carbon_penalty", d.carbon_penalty_per_tco2}, {"ccus_cost", d.ccus_cost_per_tco2},
      {"ccus_power_cost", d.ccus_power_cost_per_mwh}};
}

CampusIESOptions validation_options() {
  CampusIESOptions o;
  o.objective = CampusIESObjective::Cost;
  o.enable_grid_exchange_exclusivity = true;
  o.enable_storage_exclusivity = true;
  return o;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) throw std::runtime_error("usage: validate_integrated_energy_xref OUTPUT.json");

    CampusIESData one = make_sample_campus_ies_data(1);
    disable_non_electric_assets(one);
    one.electric_load_mw = {5.0};
    one.solar_available_mw = {2.0};
    one.grid_buy_price = {100.0};
    one.solar_rated_mw = 2.0;
    one.import_limit_mw = 10.0;
    one.export_limit_mw = 0.0;
    one.electric_storage_capacity_mwh = one.electric_storage_min_mwh = 0.0;
    one.electric_storage_initial_mwh = one.electric_storage_charge_max_mw = 0.0;
    one.electric_storage_discharge_max_mw = 0.0;
    one.solar_om_cost_per_mwh = 2.0;
    one.storage_throughput_cost_per_mwh = 0.0;

    CampusIESData storage = make_sample_campus_ies_data(2);
    disable_non_electric_assets(storage);
    storage.electric_load_mw = {1.0, 1.0};
    storage.solar_available_mw = {0.0, 0.0};
    storage.grid_buy_price = {20.0, 120.0};
    storage.solar_rated_mw = 0.0;
    storage.import_limit_mw = 5.0;
    storage.export_limit_mw = 0.0;
    storage.electric_storage_capacity_mwh = 1.0;
    storage.electric_storage_min_mwh = 0.0;
    storage.electric_storage_initial_mwh = 0.0;
    storage.electric_storage_charge_max_mw = 2.0;
    storage.electric_storage_discharge_max_mw = 2.0;
    storage.electric_storage_retention = 1.0;
    storage.eta_storage_charge = 0.9;
    storage.eta_storage_discharge = 0.9;
    storage.solar_om_cost_per_mwh = 0.0;
    storage.storage_throughput_cost_per_mwh = 0.0;

    const CampusIESData sample = make_sample_campus_ies_data(24);
    const auto options = validation_options();
    const auto r_one = solve_campus_ies(one, options);
    const auto r_storage = solve_campus_ies(storage, options);
    const auto r_sample = solve_campus_ies(sample, options);
    if (!r_one.feasible || !r_storage.feasible || !r_sample.feasible)
      throw std::runtime_error("one or more validation cases had no primal solution");

    json out;
    out["schema"] = "hysim-integrated-energy-xref-v1";
    out["cases"] = {
        {"one_step_analytic", {{"input", input_json(one)}, {"result", result_json(r_one)}}},
        {"two_step_storage", {{"input", input_json(storage)}, {"result", result_json(r_storage)}}},
        {"campus_24h", {{"input", input_json(sample)}, {"result", result_json(r_sample)}}}};

    const fs::path output = argv[1];
    if (!output.parent_path().empty()) fs::create_directories(output.parent_path());
    std::ofstream stream(output);
    if (!stream) throw std::runtime_error("cannot open output file: " + output.string());
    stream << out.dump(2) << '\n';
    std::cout << output.string() << '\n';
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "integrated-energy evidence generation failed: " << e.what() << '\n';
    return 2;
  }
}
