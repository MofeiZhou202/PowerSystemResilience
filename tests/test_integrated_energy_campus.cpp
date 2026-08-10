#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>

#include "hacdcpf/integrated_energy/integrated_energy_optimizer.hpp"

namespace {

using hacdcpf::integrated_energy::CampusIESData;
using hacdcpf::integrated_energy::CampusIESObjective;
using hacdcpf::integrated_energy::CampusIESOptions;
using hacdcpf::integrated_energy::solve_campus_ies;
using hacdcpf::integrated_energy::make_sample_campus_ies_data;

using Approx = Catch::Approx;

double at(const std::vector<double>& values, int i) {
  return values[static_cast<std::size_t>(i)];
}

CampusIESData without_storage(CampusIESData data) {
  data.electric_storage_capacity_mwh = 0.0;
  data.electric_storage_min_mwh = 0.0;
  data.electric_storage_initial_mwh = 0.0;
  data.electric_storage_charge_max_mw = 0.0;
  data.electric_storage_discharge_max_mw = 0.0;

  data.thermal_storage_capacity_mwh = 0.0;
  data.thermal_storage_min_mwh = 0.0;
  data.thermal_storage_initial_mwh = 0.0;
  data.thermal_storage_charge_max_mw = 0.0;
  data.thermal_storage_discharge_max_mw = 0.0;

  data.hydrogen_storage_capacity_mwh = 0.0;
  data.hydrogen_storage_min_mwh = 0.0;
  data.hydrogen_storage_initial_mwh = 0.0;
  data.hydrogen_storage_charge_max_mw = 0.0;
  data.hydrogen_storage_discharge_max_mw = 0.0;

  data.weekly_hydrogen_storage_capacity_mwh = 0.0;
  data.weekly_hydrogen_storage_min_mwh = 0.0;
  data.weekly_hydrogen_storage_initial_mwh = 0.0;
  data.weekly_hydrogen_storage_charge_max_mw = 0.0;
  data.weekly_hydrogen_storage_discharge_max_mw = 0.0;

  data.seasonal_hydrogen_storage_capacity_mwh = 0.0;
  data.seasonal_hydrogen_storage_min_mwh = 0.0;
  data.seasonal_hydrogen_storage_initial_mwh = 0.0;
  data.seasonal_hydrogen_storage_charge_max_mw = 0.0;
  data.seasonal_hydrogen_storage_discharge_max_mw = 0.0;

  data.daily_weekly_hydrogen_transfer_max_mw = 0.0;
  data.weekly_seasonal_hydrogen_transfer_max_mw = 0.0;
  return data;
}

bool has_carrier(const hacdcpf::integrated_energy::CampusIESResult& result,
                 const std::string& carrier) {
  return std::any_of(result.sankey_flows.begin(), result.sankey_flows.end(),
                     [&](const auto& flow) { return flow.carrier == carrier; });
}

}  // namespace

TEST_CASE("Campus integrated energy sample solves and balances all carriers",
          "[integrated_energy][campus_ies]") {
  const CampusIESData data = make_sample_campus_ies_data(24);

  CampusIESOptions opts;
  opts.objective = CampusIESObjective::Cost;

  const auto result = solve_campus_ies(data, opts);
  REQUIRE(result.feasible);
  CHECK(result.optimal);
  REQUIRE(result.p_grid_import_mw.size() == 24U);
  REQUIRE(result.e_storage_mwh.size() == 25U);
  CHECK_FALSE(result.sankey_flows.empty());

  for (int t = 0; t < data.num_steps; ++t) {
    const double electric_supply =
        at(result.p_grid_import_mw, t) - at(result.p_grid_export_mw, t) +
        at(result.p_solar_mw, t) + at(result.p_wind_mw, t) +
        at(result.p_chp_mw, t) + at(result.p_fuelcell_mw, t) +
        at(result.p_storage_discharge_mw, t) + at(result.p_ev_v2g_mw, t);
    const double electric_demand =
        at(data.electric_load_mw, t) + at(result.p_electrolysis_h2_mw, t) +
        at(result.p_electrolysis_fuel_mw, t) + at(result.p_heatpump_mw, t) +
        at(result.p_storage_charge_mw, t) + at(result.p_ev_charge_mw, t) +
        at(result.p_ccus_mw, t);
    CHECK(electric_supply == Approx(electric_demand).margin(1e-5));

    const double heat_supply =
        at(result.q_chp_mw, t) + at(result.q_heatpump_mw, t) +
        at(result.q_storage_discharge_mw, t) + data.eta_wasteheat * at(result.p_heatpump_mw, t);
    const double heat_demand = at(data.heat_load_mw, t) + at(result.q_storage_charge_mw, t);
    CHECK(heat_supply == Approx(heat_demand).margin(1e-5));

    const double h_supply =
        at(result.h_electrolysis_mw, t) + at(result.h_storage_discharge_mw, t) +
        at(result.h_weekly_discharge_mw, t) + at(result.h_seasonal_discharge_mw, t);
    const double h_demand =
        at(data.hydrogen_load_mw, t) + at(result.h_fuelcell_mw, t) +
        at(result.h_hv_refuel_mw, t) + at(result.h_storage_charge_mw, t) +
        at(result.h_weekly_charge_mw, t) + at(result.h_seasonal_charge_mw, t);
    CHECK(h_supply == Approx(h_demand).margin(1e-5));
  }

  CHECK(has_carrier(result, "electricity"));
  CHECK(has_carrier(result, "heat"));
  CHECK(has_carrier(result, "hydrogen"));
  CHECK(has_carrier(result, "fuel"));
}

TEST_CASE("Campus integrated energy storage option is never worse than no storage",
          "[integrated_energy][campus_ies][storage]") {
  CampusIESData integrated = make_sample_campus_ies_data(24);
  integrated.electric_storage_initial_mwh = 0.0;
  integrated.thermal_storage_initial_mwh = 0.0;
  integrated.hydrogen_storage_initial_mwh = 0.0;
  integrated.weekly_hydrogen_storage_initial_mwh = 0.0;
  integrated.seasonal_hydrogen_storage_initial_mwh = 0.0;

  CampusIESOptions opts;
  opts.objective = CampusIESObjective::Cost;

  const auto with_storage = solve_campus_ies(integrated, opts);
  REQUIRE(with_storage.feasible);

  const auto no_storage = solve_campus_ies(without_storage(integrated), opts);
  REQUIRE(no_storage.feasible);

  CHECK(with_storage.total_cost <= no_storage.total_cost + 1e-4);
  CHECK(with_storage.total_curtailment_mwh <= no_storage.total_curtailment_mwh + 1e-4);
}

TEST_CASE("Campus integrated energy carbon budget activates CCUS",
          "[integrated_energy][campus_ies][carbon]") {
  CampusIESData data = make_sample_campus_ies_data(24);
  data.fuel_load_mw.assign(24U, 0.8);
  data.carbon_penalty_per_tco2 = 0.0;
  data.ccus_cost_per_tco2 = 1000.0;
  data.ccus_capture_fraction = 0.90;
  data.ccus_max_tco2_per_h = 10.0;

  CampusIESOptions base_opts;
  base_opts.objective = CampusIESObjective::Cost;
  const auto baseline = solve_campus_ies(data, base_opts);
  REQUIRE(baseline.feasible);

  data.co2_budget_tco2 = 0.75 * baseline.total_emissions_tco2;

  CampusIESOptions budget_opts;
  budget_opts.objective = CampusIESObjective::Cost;
  budget_opts.enable_carbon_budget = true;
  const auto budgeted = solve_campus_ies(data, budget_opts);

  REQUIRE(budgeted.feasible);
  CHECK(budgeted.total_carbon_residual_tco2 <= data.co2_budget_tco2 + 1e-5);
  CHECK(budgeted.total_co2_captured_tco2 > baseline.total_co2_captured_tco2 + 1e-5);
}

TEST_CASE("Campus integrated energy declares its isolated electrical scope",
          "[integrated_energy][campus_ies][scope]") {
  CampusIESData data = make_sample_campus_ies_data(4);
  data.pcc_ac_bus = 27;
  const auto result = solve_campus_ies(data, {});
  REQUIRE(result.feasible);
  CHECK(result.pcc_ac_bus == 27);
  CHECK(result.model_scope == "isolated-campus-multi-carrier-milp");
  CHECK(result.validity.multi_carrier_balances_modelled);
  CHECK(result.validity.aggregate_pcc_active_power_modelled);
  CHECK_FALSE(result.validity.electrical_network_coupled);
  CHECK_FALSE(result.validity.reactive_power_modelled);
  CHECK_FALSE(result.validity.voltage_and_branch_limits_enforced);
  CHECK_FALSE(result.model_limitations.empty());
}

TEST_CASE("Campus integrated energy rejects unmodelled power factor and active efficiencies",
          "[integrated_energy][campus_ies][validation]") {
  CampusIESData data = make_sample_campus_ies_data(2);
  data.fixed_power_factor = 0.95;
  CHECK_THROWS_AS(solve_campus_ies(data, {}), std::invalid_argument);

  data = make_sample_campus_ies_data(2);
  data.eta_fuelcell = 1.01;
  CHECK_THROWS_AS(solve_campus_ies(data, {}), std::invalid_argument);

  data = make_sample_campus_ies_data(2);
  data.electric_storage_retention = 1.0001;
  CHECK_THROWS_AS(solve_campus_ies(data, {}), std::invalid_argument);
}

TEST_CASE("Campus transport total reports solved activity",
          "[integrated_energy][campus_ies][transport]") {
  CampusIESData data = make_sample_campus_ies_data(4);
  CampusIESOptions options;
  options.enable_transport = false;
  const auto disabled = solve_campus_ies(data, options);
  REQUIRE(disabled.feasible);
  const auto total = [](const std::vector<double>& values) {
    return std::accumulate(values.begin(), values.end(), 0.0);
  };
  CHECK(disabled.total_transport_km == Approx(0.0).margin(1e-9));
  CHECK(total(disabled.d_ev_km) == Approx(0.0).margin(1e-9));
  CHECK(total(disabled.d_hv_km) == Approx(0.0).margin(1e-9));
  CHECK(total(disabled.d_icv_km) == Approx(0.0).margin(1e-9));
}
