#include <algorithm>
#include <filesystem>
#include <set>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/io/component_io_mapping.hpp"
#include "hacdcpf/model/model_semantics.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace {

using Catch::Approx;

TEST_CASE("Every rich IO collection has exactly one runtime semantics contract",
          "[model][semantics][registry]") {
  const auto& io_rows = hacdcpf::io::component_io_mappings();
  const auto& runtime_rows = hacdcpf::model::component_runtime_semantics();
  REQUIRE(runtime_rows.size() == io_rows.size());

  std::set<std::string> paths;
  const std::set<std::string> required_workflows{
      "power_flow", "optimal_power_flow", "dynamics", "time_series",
      "reliability_resilience", "market_carbon"};
  for (const auto& row : runtime_rows) {
    INFO(row.collection_path);
    CHECK(paths.insert(row.collection_path).second);
    CHECK_FALSE(row.component_type.empty());
    CHECK_FALSE(row.identity_contract.empty());
    CHECK_FALSE(row.terminal_contract.empty());
    CHECK_FALSE(row.unit_contract.empty());
    CHECK_FALSE(row.sign_contract.empty());
    CHECK_FALSE(row.parameter_owner.empty());
    CHECK_FALSE(row.result_contract.empty());
    CHECK_FALSE(row.fidelity_boundary.empty());
    std::set<std::string> workflows;
    for (const auto& workflow : row.workflows) {
      CHECK(workflows.insert(workflow.workflow).second);
      CHECK_FALSE(workflow.equations_or_role.empty());
      CHECK_FALSE(hacdcpf::model::to_string(workflow.fidelity).empty());
    }
    CHECK(workflows == required_workflows);
  }

  for (const auto& io_row : io_rows) {
    INFO(io_row.collection_path);
    const auto* runtime = hacdcpf::model::find_component_runtime_semantics(
        io_row.collection_path);
    REQUIRE(runtime != nullptr);
    CHECK(runtime->component_type == io_row.component_type);
  }
}

TEST_CASE("Every production source module has an input-output data contract",
          "[model][semantics][modules]") {
  const std::filesystem::path source_root =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) / "src";
  std::set<std::string> registered;
  for (const auto& contract : hacdcpf::model::module_data_contracts()) {
    INFO(contract.source_module);
    CHECK(registered.insert(contract.source_module).second);
    CHECK_FALSE(contract.authoritative_input.empty());
    CHECK_FALSE(contract.execution_view.empty());
    CHECK_FALSE(contract.public_output.empty());
    CHECK_FALSE(contract.identity_space.empty());
    CHECK_FALSE(contract.unit_contract.empty());
    CHECK_FALSE(contract.fidelity_boundary.empty());
  }

  std::set<std::string> source_modules;
  for (const auto& entry : std::filesystem::directory_iterator(source_root)) {
    if (entry.is_directory()) {
      source_modules.insert(entry.path().filename().string());
    }
  }
  CHECK(registered == source_modules);
}

TEST_CASE("DCStorage materialization preserves every shared physical field and is idempotent",
          "[model][semantics][storage]") {
  hacdcpf::DCStorage source;
  source.index = 71;
  source.bus = 9;
  source.in_service = false;
  source.name = "dc battery";
  source.type = "LFP";
  source.p_mw = -3.2;
  source.p_rated_mw = 8.0;
  source.pmax_mw = 7.5;
  source.pmin_mw = -6.5;
  source.e_rated_mwh = 24.0;
  source.soc_init = 0.63;
  source.soc_min = 0.12;
  source.soc_max = 0.91;
  source.soc_carbon_intensity_tco2_mwh = 0.27;
  source.eta_charge = 0.93;
  source.eta_discharge = 0.94;
  source.self_discharge_pct = 0.03;
  source.max_cycles = 4321;
  source.current_cycles = 123;
  source.soh = 0.88;
  source.l_calendar_yr = 11.0;
  source.eol_percent = 0.76;
  source.replacement_cost = 123456.0;
  source.e_mwh = 15.2;
  source.profile_id = 19;
  source.controllable = false;
  source.charge_bid_price = 17.0;
  source.discharge_bid_price = 42.0;
  source.daily_cycle_limit = 1.7;
  source.forced_outage_rate = 0.04;
  source.mttr_hr = 6.0;
  source.t_scheduled_hr = 3.0;
  source.cap_charging_strategy = "static";
  source.cap_static_charging_mw = 2.4;
  source.dynamic_model.standard = "IEC";
  source.dynamic_model.model_name = "BatteryModel";
  source.dynamic_model.parameters["tau"] = 0.2;

  const auto view = hacdcpf::dc_storage_execution_view(source);
  CHECK(view.index == source.index);
  CHECK(view.bus == source.bus);
  CHECK(view.in_service == source.in_service);
  CHECK(view.name == source.name);
  CHECK(view.type == source.type);
  CHECK(view.p_mw == source.p_mw);
  CHECK(view.q_mvar == 0.0);
  CHECK(view.p_rated_mw == source.p_rated_mw);
  CHECK(view.pmax_mw == source.pmax_mw);
  CHECK(view.pmin_mw == source.pmin_mw);
  CHECK(view.qmax_mvar == 0.0);
  CHECK(view.qmin_mvar == 0.0);
  CHECK(view.e_rated_mwh == source.e_rated_mwh);
  CHECK(view.soc_init == source.soc_init);
  CHECK(view.soc_min == source.soc_min);
  CHECK(view.soc_max == source.soc_max);
  CHECK(view.soc_carbon_intensity_tco2_mwh ==
        source.soc_carbon_intensity_tco2_mwh);
  CHECK(view.eta_charge == source.eta_charge);
  CHECK(view.eta_discharge == source.eta_discharge);
  CHECK(view.self_discharge_pct == source.self_discharge_pct);
  CHECK(view.max_cycles == source.max_cycles);
  CHECK(view.current_cycles == source.current_cycles);
  CHECK(view.soh == source.soh);
  CHECK(view.l_calendar_yr == source.l_calendar_yr);
  CHECK(view.eol_percent == source.eol_percent);
  CHECK(view.replacement_cost == source.replacement_cost);
  CHECK(view.e_mwh == source.e_mwh);
  CHECK(view.profile_id == source.profile_id);
  CHECK(view.controllable == source.controllable);
  CHECK(view.charge_bid_price == source.charge_bid_price);
  CHECK(view.discharge_bid_price == source.discharge_bid_price);
  CHECK(view.daily_cycle_limit == source.daily_cycle_limit);
  CHECK(view.forced_outage_rate == source.forced_outage_rate);
  CHECK(view.mttr_hr == source.mttr_hr);
  CHECK(view.t_scheduled_hr == source.t_scheduled_hr);
  CHECK(view.cap_charging_strategy == source.cap_charging_strategy);
  CHECK(view.cap_static_charging_mw == source.cap_static_charging_mw);
  CHECK(view.dynamic_model.standard == source.dynamic_model.standard);
  CHECK(view.dynamic_model.model_name == source.dynamic_model.model_name);
  CHECK(view.dynamic_model.parameters == source.dynamic_model.parameters);

  hacdcpf::HybridPowerSystem system;
  system.dc.dc_storage.push_back(source);
  hacdcpf::materialize_dc_storage(system);
  REQUIRE(system.dc.storage.size() == 1);
  CHECK(system.dc.dc_storage.empty());
  hacdcpf::materialize_dc_storage(system);
  CHECK(system.dc.storage.size() == 1);
}

TEST_CASE("StaticGeneratorDC PF view retains injection limits cost and carbon semantics",
          "[model][semantics][dc_generator]") {
  hacdcpf::StaticGeneratorDC source;
  source.index = 81;
  source.bus = 4;
  source.in_service = false;
  source.name = "dc source";
  source.type = "fuel-cell";
  source.p_set_mw = 12.5;
  source.scaling = 0.7;
  source.profile_id = 14;
  source.pmax_mw = 22.0;
  source.pmin_mw = -2.0;
  source.controllable = true;
  source.cost_c1 = 31.0;
  source.emission_factor_tco2_mwh = 0.38;
  source.mtbf_hours = 12000.0;
  source.dynamic_model.model_name = "FuelCellDynamic";

  const auto view = hacdcpf::dc_static_generator_pf_view(source);
  CHECK(view.index == source.index);
  CHECK(view.bus == source.bus);
  CHECK(view.in_service == source.in_service);
  CHECK(view.name == source.name);
  CHECK(view.p_mw == source.p_set_mw);
  CHECK(view.q_mvar == 0.0);
  CHECK(view.scaling == source.scaling);
  CHECK(view.pmax_mw == source.pmax_mw);
  CHECK(view.pmin_mw == source.pmin_mw);
  CHECK(view.controllable == source.controllable);
  CHECK(view.cost_c1 == source.cost_c1);
  CHECK(view.co2_emission_rate == source.emission_factor_tco2_mwh);
  CHECK(view.dynamic_model.model_name.empty());
  CHECK(view.mtbf_hours == 0.0);
}

TEST_CASE("SolverData constructors share DC compatibility views and additive load semantics",
          "[model][semantics][solver_data]") {
  hacdcpf::HybridPowerSystem system;
  system.base_mva = 100.0;
  system.ac.base_mva = 100.0;
  system.dc.base_mva = 100.0;

  hacdcpf::ACBus ac_bus;
  ac_bus.index = 1;
  ac_bus.bus_type = hacdcpf::BusType::SLACK;
  ac_bus.pd_mw = 10.0;
  ac_bus.qd_mvar = 3.0;
  system.ac.buses.push_back(ac_bus);
  hacdcpf::Load load;
  load.index = 101;
  load.bus = 1;
  load.p_mw = 20.0;
  load.q_mvar = 8.0;
  load.scaling = 0.5;
  system.ac.loads.push_back(load);

  hacdcpf::DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.bus_type = hacdcpf::DCBusType::DC_V;
  system.dc.buses.push_back(dc_bus);
  hacdcpf::DCStorage storage;
  storage.index = 71;
  storage.bus = 1;
  storage.p_mw = 4.0;
  storage.e_rated_mwh = 12.0;
  system.dc.dc_storage.push_back(storage);
  hacdcpf::StaticGeneratorDC generator;
  generator.index = 81;
  generator.bus = 1;
  generator.p_set_mw = 6.0;
  generator.scaling = 0.8;
  generator.cost_c1 = 27.0;
  generator.emission_factor_tco2_mwh = 0.41;
  system.dc.dc_static_generators.push_back(generator);

  const auto direct = hacdcpf::powerflow::make_solver_data(system);
  auto canonical = hacdcpf::project_to_canonical_models(system);
  const auto projected = hacdcpf::powerflow::make_solver_data_projected(
      std::move(canonical));

  REQUIRE(direct.dc_storage.size() == 1);
  REQUIRE(projected.dc_storage.size() == 1);
  REQUIRE(direct.dc_static_generators.size() == 1);
  REQUIRE(projected.dc_static_generators.size() == 1);
  CHECK(direct.dc_storage.front().index == 71);
  CHECK(projected.dc_storage.front().index == 71);
  CHECK(direct.dc_static_generators.front().index == 81);
  CHECK(projected.dc_static_generators.front().index == 81);
  CHECK(direct.dc_static_generators.front().p_mw == 6.0);
  CHECK(projected.dc_static_generators.front().p_mw == 6.0);
  CHECK(direct.dc_static_generators.front().cost_c1 == 27.0);
  CHECK(projected.dc_static_generators.front().cost_c1 == 27.0);
  CHECK(direct.dc_static_generators.front().co2_emission_rate == 0.41);
  CHECK(projected.dc_static_generators.front().co2_emission_rate == 0.41);

  REQUIRE(direct.has_component_loads);
  REQUIRE(direct.pd_pu.size() == 1);
  REQUIRE(direct.qd_pu.size() == 1);
  CHECK(direct.pd_pu[0] == Approx(0.20));
  CHECK(direct.qd_pu[0] == Approx(0.07));
  CHECK(projected.pd_pu[0] == Approx(direct.pd_pu[0]));
  CHECK(projected.qd_pu[0] == Approx(direct.qd_pu[0]));
}

}  // namespace
