#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <stdexcept>

namespace {

using Approx = Catch::Approx;

hacdcpf::HybridPowerSystem build_single_bus_storage_case() {
  using namespace hacdcpf;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;

  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.vm_pu = 1.0;
  bus.va_deg = 0.0;
  sys.ac.buses.push_back(bus);

  Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.in_service = true;
  gen.is_slack = true;
  gen.pg_mw = 12.0;
  gen.pmin_mw = 0.0;
  gen.pmax_mw = 100.0;
  gen.emission_factor_tco2_mwh = 0.8;
  sys.ac.generators.push_back(gen);

  Load load;
  load.index = 1;
  load.bus = 1;
  load.in_service = true;
  load.p_mw = 10.0;
  load.q_mvar = 0.0;
  sys.ac.loads.push_back(load);

  Storage storage;
  storage.index = 1;
  storage.bus = 1;
  storage.in_service = true;
  storage.p_rated_mw = 20.0;
  storage.pmin_mw = -20.0;
  storage.pmax_mw = 20.0;
  storage.e_rated_mwh = 100.0;
  storage.eta_charge = 1.0;
  storage.eta_discharge = 1.0;
  storage.soc_init = 0.5;
  storage.e_mwh = 50.0;
  storage.soc_carbon_intensity_tco2_mwh = 0.2;
  sys.ac.storage.push_back(storage);

  return sys;
}

hacdcpf::PowerFlowResult converged_pf() {
  hacdcpf::PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};
  return pf;
}

}  // namespace

TEST_CASE("time-series snapshot helper stores terminal storage SOC",
          "[carbonflow][storage][time-series]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();

  TimeSeriesData ts;
  ts.num_steps = 2;
  ts.step_duration_hr = 1.0;
  ts.profiles = {{0, "load", {1.0, 1.0}}};

  UCSchedule schedule;
  schedule.feasible = true;
  schedule.gen_dispatch = {{12.0, 10.0}};
  schedule.gen_commit = {{1, 1}};
  schedule.ess_dispatch = {{-10.0, 5.0}};
  schedule.ess_soc = {{0.60, 0.55}};

  TimeSeriesPFOptions opts;
  opts.skip_uc = false;

  const HybridPowerSystem step0 =
      build_time_series_system_snapshot(base, ts, schedule, 0, opts);
  const HybridPowerSystem step1 =
      build_time_series_system_snapshot(base, ts, schedule, 1, opts);

  REQUIRE(step0.ac.storage.size() == 1);
  CHECK(step0.ac.storage[0].p_mw == Approx(-10.0));
  CHECK(step0.ac.storage[0].soc_init == Approx(0.60));
  CHECK(step0.ac.storage[0].e_mwh == Approx(60.0));

  REQUIRE(step1.ac.storage.size() == 1);
  CHECK(step1.ac.storage[0].p_mw == Approx(5.0));
  CHECK(step1.ac.storage[0].soc_init == Approx(0.55));
  CHECK(step1.ac.storage[0].e_mwh == Approx(55.0));
}

TEST_CASE("time-series PF replays a precomputed UC schedule without resolving UC",
          "[time_series][replay][storage]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();

  TimeSeriesData ts;
  ts.num_steps = 2;
  ts.step_duration_hr = 1.0;
  base.ac.loads[0].profile_id = 0;
  ts.profiles = {{0, "load", {0.5, 1.5}}};

  UCSchedule schedule;
  schedule.feasible = true;
  schedule.gen_dispatch = {{20.0, 5.0}};
  schedule.gen_commit = {{1, 1}};
  schedule.ess_dispatch = {{-10.0, 5.0}};
  schedule.ess_soc = {{0.60, 0.55}};

  TimeSeriesPFOptions opts;
  opts.skip_uc = true;
  opts.run_opf = false;
  opts.keep_system_snapshots = true;
  opts.precomputed_uc_schedule = &schedule;

  const TimeSeriesPFResult result = solve_time_series_pf(base, ts, opts);

  REQUIRE(result.uc_solve_sec == Approx(0.0));
  REQUIRE(result.pf_system_snapshots.size() == 2);
  CHECK(result.pf_system_snapshots[0].ac.generators[0].pg_mw == Approx(20.0));
  CHECK(result.pf_system_snapshots[1].ac.generators[0].pg_mw == Approx(5.0));
  CHECK(result.pf_system_snapshots[0].ac.storage[0].p_mw == Approx(-10.0));
  CHECK(result.pf_system_snapshots[1].ac.storage[0].p_mw == Approx(5.0));
  CHECK(result.pf_system_snapshots[0].ac.storage[0].soc_init == Approx(0.60));
  CHECK(result.pf_system_snapshots[1].ac.storage[0].soc_init == Approx(0.55));
  CHECK(result.pf_system_snapshots[0].ac.loads[0].p_mw == Approx(5.0));
  CHECK(result.pf_system_snapshots[1].ac.loads[0].p_mw == Approx(15.0));
  REQUIRE(result.rich_results.size() == 2);
  for (const auto& rich_result : result.rich_results) {
    CHECK(rich_result.coverage.total());
    CHECK(rich_result.coverage.strong_components ==
          rich_result.coverage.rich_components);
    CHECK(rich_result.coverage.unsupported_components == 0);
  }
}

TEST_CASE("external-grid price profile is an absolute time series",
          "[time_series][profiles][price]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();
  ExternalGrid grid;
  grid.index = 7;
  grid.bus = 1;
  grid.in_service = true;
  grid.cost_c1 = 5.0;
  grid.price_profile_id = 9;
  base.ac.external_grids.push_back(grid);

  TimeSeriesData ts;
  ts.num_steps = 2;
  ts.step_duration_hr = 1.0;
  ts.profiles = {{9, "price", {100.0, 250.0}}};
  UCSchedule schedule;
  schedule.feasible = false;
  TimeSeriesPFOptions opts;
  opts.skip_uc = true;

  const auto step0 = build_time_series_system_snapshot(base, ts, schedule, 0, opts);
  const auto step1 = build_time_series_system_snapshot(base, ts, schedule, 1, opts);
  REQUIRE(step0.ac.external_grids.size() == 1);
  REQUIRE(step1.ac.external_grids.size() == 1);
  CHECK(step0.ac.external_grids[0].cost_c1 == Approx(100.0));
  CHECK(step1.ac.external_grids[0].cost_c1 == Approx(250.0));
}

TEST_CASE("time-series attribution preserves native DCStorage identity and dispatch",
          "[time_series][projection][storage][dc]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();
  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.bus_type = DCBusType::DC_V;
  base.dc.buses.push_back(dc_bus);

  DCStorage storage;
  storage.index = 91;
  storage.bus = 1;
  storage.name = "Native DC BESS";
  storage.p_rated_mw = 2.0;
  storage.pmin_mw = -2.0;
  storage.pmax_mw = 2.0;
  storage.e_rated_mwh = 10.0;
  storage.soc_init = 0.5;
  storage.e_mwh = 5.0;
  base.dc.dc_storage.push_back(storage);

  TimeSeriesData ts;
  ts.num_steps = 2;
  ts.step_duration_hr = 1.0;
  UCSchedule schedule;
  schedule.feasible = true;
  schedule.gen_dispatch = {{10.0, 10.0}};
  schedule.gen_commit = {{1, 1}};
  schedule.dc_ess_dispatch = {{-1.0, 1.0}};
  schedule.dc_ess_soc = {{0.6, 0.5}};

  TimeSeriesPFOptions opts;
  opts.skip_uc = true;
  opts.run_opf = false;
  opts.keep_system_snapshots = true;
  opts.precomputed_uc_schedule = &schedule;
  const auto result = solve_time_series_pf(base, ts, opts);

  REQUIRE(result.rich_results.size() == 2);
  for (size_t t = 0; t < result.rich_results.size(); ++t) {
    const auto& components = result.rich_results[t].components;
    const auto it = std::find_if(
        components.begin(), components.end(), [](const auto& component) {
          return component.component_type == "dc_storage" &&
                 component.domain == "DC" && component.component_index == 91;
        });
    REQUIRE(it != components.end());
    const auto p = std::find_if(it->values.begin(), it->values.end(),
                                [](const auto& value) {
                                  return value.name == "p_mw";
                                });
    REQUIRE(p != it->values.end());
    CHECK(p->value == Approx(t == 0 ? -1.0 : 1.0));
    CHECK(std::none_of(components.begin(), components.end(),
                       [](const auto& component) {
                         return component.component_type == "storage" &&
                                component.domain == "DC" &&
                                component.component_index == 91;
                       }));
  }
}

TEST_CASE("skip-UC fallback schedules DC storage with SOC transitions",
          "[time_series][storage][dc]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();
  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.bus_type = DCBusType::DC_V;
  base.dc.buses.push_back(dc_bus);

  Storage dc_storage;
  dc_storage.index = 7;
  dc_storage.bus = 1;
  dc_storage.in_service = true;
  dc_storage.p_rated_mw = 1.0;
  dc_storage.pmin_mw = -1.0;
  dc_storage.pmax_mw = 1.0;
  dc_storage.e_rated_mwh = 10.0;
  dc_storage.soc_init = 0.5;
  dc_storage.soc_min = 0.1;
  dc_storage.soc_max = 0.9;
  dc_storage.eta_charge = 0.95;
  dc_storage.eta_discharge = 0.95;
  base.dc.storage.push_back(dc_storage);

  base.ac.loads[0].profile_id = 0;
  TimeSeriesData ts;
  ts.num_steps = 4;
  ts.step_duration_hr = 1.0;
  ts.profiles = {{0, "load", {0.5, 1.5, 1.5, 0.5}}};

  TimeSeriesPFOptions opts;
  opts.skip_uc = true;
  opts.run_opf = false;
  opts.keep_system_snapshots = true;
  opts.enforce_terminal_soc_cyclic = true;
  const TimeSeriesPFResult result = solve_time_series_pf(base, ts, opts);

  REQUIRE(result.uc_schedule.dc_ess_dispatch.size() == 1);
  REQUIRE(result.uc_schedule.dc_ess_soc.size() == 1);
  const auto& dispatch = result.uc_schedule.dc_ess_dispatch[0];
  const auto& soc = result.uc_schedule.dc_ess_soc[0];
  REQUIRE(dispatch.size() == 4);
  REQUIRE(soc.size() == 4);
  CHECK(*std::min_element(dispatch.begin(), dispatch.end()) < 0.0);
  CHECK(*std::max_element(dispatch.begin(), dispatch.end()) > 0.0);
  CHECK(soc.back() == Approx(dc_storage.soc_init).margin(1e-9));

  double previous_soc = dc_storage.soc_init;
  for (size_t t = 0; t < dispatch.size(); ++t) {
    const double expected = dispatch[t] >= 0.0
        ? previous_soc - dispatch[t] /
              (dc_storage.eta_discharge * dc_storage.e_rated_mwh)
        : previous_soc - dispatch[t] * dc_storage.eta_charge /
              dc_storage.e_rated_mwh;
    CHECK(soc[t] == Approx(expected).margin(1e-10));
    previous_soc = soc[t];
  }
}

TEST_CASE("dynamic carbon materializes native DC storage for energy recurrence",
          "[time_series][storage][dc][carbonflow][regression]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();
  base.ac.storage.clear();
  base.ac.loads[0].profile_id = 0;

  DCBus dc_bus;
  dc_bus.index = 1;
  dc_bus.bus_type = DCBusType::DC_V;
  base.dc.buses.push_back(dc_bus);

  DCStorage storage;
  storage.index = 17;
  storage.bus = 1;
  storage.in_service = true;
  storage.p_rated_mw = 1.0;
  storage.pmin_mw = -1.0;
  storage.pmax_mw = 1.0;
  storage.e_rated_mwh = 10.0;
  storage.soc_init = 0.5;
  storage.soc_min = 0.1;
  storage.soc_max = 0.9;
  storage.eta_charge = 0.9;
  storage.eta_discharge = 0.8;
  storage.self_discharge_pct = 1.0;
  storage.soc_carbon_intensity_tco2_mwh = 0.2;
  base.dc.dc_storage.push_back(storage);

  TimeSeriesData ts;
  ts.num_steps = 4;
  ts.step_duration_hr = 1.0;
  ts.profiles = {{0, "load", {0.5, 1.5, 1.5, 0.5}}};

  TimeSeriesPFOptions opts;
  opts.skip_uc = true;
  opts.run_opf = false;
  opts.keep_system_snapshots = true;
  const TimeSeriesPFResult result = solve_time_series_pf(base, ts, opts);

  REQUIRE(result.pf_system_snapshots.size() == 4);
  REQUIRE(result.pf_system_snapshots.front().dc.storage.empty());
  REQUIRE(result.pf_system_snapshots.front().dc.dc_storage.size() == 1);

  analysis::AnnualCarbonAnalysisOptions carbon_opts;
  const auto annual = analysis::compute_annual_carbon_analysis(
      result, ts.step_duration_hr, carbon_opts);
  REQUIRE(annual.terminal_storage_states.size() == 1);
  CHECK(annual.terminal_storage_states[0].storage_index == 17);
  CHECK(annual.terminal_storage_states[0].is_dc);
  CHECK(std::abs(annual.storage_energy_balance_error_mwh) < 1e-8);
  CHECK(annual.storage_energy_balance_abs_error_mwh < 1e-8);
  CHECK(std::abs(annual.storage_inventory_balance_error_tco2) < 1e-8);
}

TEST_CASE("skip-UC fallback replaces inactive AC schedule and closes SOC",
          "[time_series][storage][ac][carbonflow][regression]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();
  base.ac.loads[0].profile_id = 0;
  base.ac.storage[0].eta_charge = 0.9;
  base.ac.storage[0].eta_discharge = 0.8;
  base.ac.storage[0].self_discharge_pct = 1.0;

  TimeSeriesData ts;
  ts.num_steps = 4;
  ts.step_duration_hr = 1.0;
  ts.profiles = {{0, "load", {0.5, 1.5, 1.5, 0.5}}};

  TimeSeriesPFOptions opts;
  opts.skip_uc = true;
  opts.run_opf = false;
  opts.keep_system_snapshots = true;
  const TimeSeriesPFResult result = solve_time_series_pf(base, ts, opts);

  REQUIRE(result.uc_schedule.ess_dispatch.size() == 1);
  REQUIRE(result.uc_schedule.ess_soc.size() == 1);
  const auto& dispatch = result.uc_schedule.ess_dispatch[0];
  const auto& soc = result.uc_schedule.ess_soc[0];
  REQUIRE(dispatch.size() == 4);
  REQUIRE(soc.size() == 4);
  REQUIRE(result.pf_system_snapshots.size() == 4);

  double previous_soc = base.ac.storage[0].soc_init;
  for (size_t t = 0; t < dispatch.size(); ++t) {
    const double expected = previous_soc * 0.99 +
        (dispatch[t] < 0.0
             ? -dispatch[t] * base.ac.storage[0].eta_charge /
                   base.ac.storage[0].e_rated_mwh
             : -dispatch[t] /
                   (base.ac.storage[0].eta_discharge *
                    base.ac.storage[0].e_rated_mwh));
    CHECK(soc[t] == Approx(expected).margin(1e-10));
    CHECK(result.pf_system_snapshots[t].ac.storage[0].soc_init ==
          Approx(soc[t]).margin(1e-10));
    CHECK(result.pf_system_snapshots[t].ac.storage[0].e_mwh ==
          Approx(soc[t] * base.ac.storage[0].e_rated_mwh).margin(1e-10));
    previous_soc = soc[t];
  }

  analysis::AnnualCarbonAnalysisOptions carbon_opts;
  const auto annual = analysis::compute_annual_carbon_analysis(
      result, ts.step_duration_hr, carbon_opts);
  CHECK(std::abs(annual.storage_energy_balance_error_mwh) < 1e-8);
  CHECK(annual.storage_energy_balance_abs_error_mwh < 1e-8);
  CHECK(std::abs(annual.storage_inventory_balance_error_tco2) < 1e-8);
}

TEST_CASE("carbon flow repairs radial DC transfer hidden by equal boundary voltages",
          "[carbonflow][dc][kcl]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  ACBus ac_bus;
  ac_bus.index = 1;
  ac_bus.bus_type = BusType::SLACK;
  sys.ac.buses.push_back(ac_bus);
  Load ac_load;
  ac_load.index = 1;
  ac_load.bus = 1;
  ac_load.p_mw = 0.392;
  sys.ac.loads.push_back(ac_load);
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.in_service = true;
  sys.ac.external_grids.push_back(grid);

  DCBus dc_p;
  dc_p.index = 1;
  dc_p.bus_type = DCBusType::DC_P;
  DCBus dc_v;
  dc_v.index = 2;
  dc_v.bus_type = DCBusType::DC_V;
  dc_v.emission_factor_tco2_mwh = 0.5;
  sys.dc.buses = {dc_p, dc_v};
  DCBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.in_service = true;
  sys.dc.branches.push_back(branch);

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};
  pf.vdc = {1.0, 1.0};
  pf.vsc_transfers.push_back(
      VSCTransfer{1, 1, 1, 0.392, 0.0, -0.4, 0.008});

  const CarbonAnalysisResult carbon = compute_carbon_analysis(sys, pf, {});
  CHECK(carbon.power_balance_verified);
  CHECK(carbon.matrix_solved);
  CHECK(carbon.max_node_power_balance_error_mw == Approx(0.0).margin(1e-10));
}

TEST_CASE("time-series solvers reject non-positive step durations",
          "[time_series][robustness]") {
  using namespace hacdcpf;

  HybridPowerSystem base = build_single_bus_storage_case();
  TimeSeriesData ts;
  ts.num_steps = 1;
  ts.step_duration_hr = 0.0;

  TimeSeriesPFOptions opts;
  opts.skip_uc = true;

  CHECK_THROWS_AS(solve_time_series_pf(base, ts, opts), std::invalid_argument);
  CHECK_THROWS_AS(solve_unit_commitment(base, ts, TimeSeriesPFOptions{}),
                  std::invalid_argument);
}

TEST_CASE("TimeSeriesPFResult annual carbon uses storage snapshots dynamically",
          "[carbonflow][storage][annual]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem step0 = build_single_bus_storage_case();
  step0.ac.generators[0].pg_mw = 20.0;
  step0.ac.storage[0].p_mw = -10.0;
  step0.ac.storage[0].soc_init = 0.60;
  step0.ac.storage[0].e_mwh = 60.0;

  HybridPowerSystem step1 = build_single_bus_storage_case();
  step1.ac.generators[0].pg_mw = 5.0;
  step1.ac.storage[0].p_mw = 5.0;
  step1.ac.storage[0].soc_init = 0.55;
  step1.ac.storage[0].e_mwh = 55.0;
  step1.ac.storage[0].soc_carbon_intensity_tco2_mwh = 0.2;

  std::vector<PowerFlowResult> pf_results{converged_pf(), converged_pf()};

  AnnualCarbonAnalysisOptions opts;
  opts.keep_hourly_load_emissions = true;
  opts.keep_hourly_load_energy = true;

  const CarbonAnalysisResult ca0 =
      compute_carbon_analysis(step0, pf_results[0], opts.carbon_options);
  REQUIRE(ca0.matrix_solved);
  REQUIRE_FALSE(ca0.storage_carbon.empty());
  const double charged_storage_intensity =
      ca0.storage_carbon[0].carbon_intensity_tco2_mwh;

  HybridPowerSystem expected_step1 = step1;
  expected_step1.ac.storage[0].soc_carbon_intensity_tco2_mwh =
      (50.0 * 0.2 + 10.0 * charged_storage_intensity) / 60.0;
  const CarbonAnalysisResult expected_ca1 =
      compute_carbon_analysis(expected_step1, pf_results[1], opts.carbon_options);
  REQUIRE_FALSE(expected_ca1.storage_carbon.empty());

  TimeSeriesPFResult ts_result;
  ts_result.num_steps = 2;
  ts_result.pf_results = pf_results;
  ts_result.pf_system_snapshots = {step0, step1};

  const AnnualCarbonAnalysisResult annual =
      compute_annual_carbon_analysis(ts_result, 1.0, opts);

  REQUIRE(annual.num_steps == 2);
  REQUIRE(annual.num_pf_converged == 2);
  REQUIRE(annual.num_carbon_verified == 2);
  REQUIRE(annual.step_results.size() == 2);
  CHECK(annual.step_results[1].total_generation_emissions_tco2 ==
        Approx(expected_ca1.matrix_summary.total_generation_emissions_tco2));
  CHECK(annual.step_results[1].total_load_emissions_tco2 ==
        Approx(expected_ca1.matrix_summary.total_load_emissions_tco2));

  HybridPowerSystem no_dynamic_update_step1 = step1;
  const CarbonAnalysisResult no_dynamic_ca1 =
      compute_carbon_analysis(no_dynamic_update_step1, pf_results[1], opts.carbon_options);
  REQUIRE_FALSE(no_dynamic_ca1.storage_carbon.empty());
  CHECK(expected_ca1.storage_carbon[0].total_emissions_tco2 !=
        Approx(no_dynamic_ca1.storage_carbon[0].total_emissions_tco2));
}

TEST_CASE("TimeSeriesPFResult dynamic annual carbon requires complete snapshots",
          "[carbonflow][annual]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  TimeSeriesPFResult ts_result;
  ts_result.pf_results = {converged_pf(), converged_pf()};
  ts_result.pf_system_snapshots = {build_single_bus_storage_case()};

  CHECK_THROWS_AS(
      compute_annual_carbon_analysis(ts_result, 1.0, AnnualCarbonAnalysisOptions{}),
      std::invalid_argument);
}

TEST_CASE("Dynamic storage ledger computes efficiency and self-discharge losses forward",
          "[carbonflow][storage][ledger][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem charge = build_single_bus_storage_case();
  charge.ac.storage[0].eta_charge = 0.8;
  charge.ac.storage[0].eta_discharge = 0.9;
  charge.ac.storage[0].self_discharge_pct = 10.0;
  charge.ac.storage[0].p_mw = -10.0;
  charge.ac.storage[0].soc_init = 0.53;
  charge.ac.storage[0].e_mwh = 53.0;
  charge.ac.storage[0].soc_carbon_intensity_tco2_mwh = 0.2;

  HybridPowerSystem discharge = charge;
  discharge.ac.storage[0].p_mw = 9.0;
  discharge.ac.storage[0].soc_init = 0.377;
  discharge.ac.storage[0].e_mwh = 37.7;

  TimeSeriesPFResult ts;
  ts.num_steps = 2;
  ts.pf_results = {converged_pf(), converged_pf()};
  ts.pf_system_snapshots = {charge, discharge};

  const AnnualCarbonAnalysisResult annual =
      compute_annual_carbon_analysis(ts, 1.0);

  const double initial_inventory = 50.0 * 0.2;
  const double charge_bus_intensity = 0.8;
  const double inventory_after_charge =
      initial_inventory - 5.0 * 0.2 + 8.0 * charge_bus_intensity;
  const double stored_intensity = inventory_after_charge / 53.0;
  const double expected_terminal_inventory =
      inventory_after_charge - 5.3 * stored_intensity -
      10.0 * stored_intensity;

  CHECK(annual.initial_storage_carbon_inventory_tco2 ==
        Approx(initial_inventory).margin(1e-10));
  CHECK(annual.storage_charge_loss_emissions_tco2 ==
        Approx(2.0 * charge_bus_intensity).margin(1e-10));
  CHECK(annual.storage_self_discharge_loss_emissions_tco2 ==
        Approx(5.0 * 0.2 + 5.3 * stored_intensity).margin(1e-10));
  CHECK(annual.storage_discharge_loss_emissions_tco2 ==
        Approx(1.0 * stored_intensity).margin(1e-10));
  CHECK(annual.storage_internal_loss_emissions_tco2 ==
        Approx(annual.storage_charge_loss_emissions_tco2 +
               annual.storage_self_discharge_loss_emissions_tco2 +
               annual.storage_discharge_loss_emissions_tco2)
            .margin(1e-10));
  CHECK(annual.terminal_storage_carbon_inventory_tco2 ==
        Approx(expected_terminal_inventory).margin(1e-10));
  CHECK(annual.storage_energy_balance_error_mwh == Approx(0.0).margin(1e-10));
  CHECK(annual.max_storage_step_energy_balance_error_mwh ==
        Approx(0.0).margin(1e-10));
  CHECK(annual.storage_inventory_balance_error_tco2 ==
        Approx(0.0).margin(1e-10));
  CHECK(annual.storage_inventory_balance_error_pct ==
        Approx(0.0).margin(1e-10));
  CHECK(annual.balance_error_storage_adjusted_tco2 ==
        Approx(0.0).margin(1e-10));
  REQUIRE(annual.step_results.size() == 2);
  CHECK(annual.step_results[0].storage_carbon_inventory_delta_tco2 ==
        Approx(inventory_after_charge - initial_inventory).margin(1e-10));
  CHECK(annual.step_results[0].storage_inventory_balance_error_tco2 ==
        Approx(0.0).margin(1e-10));
  CHECK(annual.step_results[1].storage_carbon_inventory_delta_tco2 ==
        Approx(expected_terminal_inventory - inventory_after_charge).margin(1e-10));
  CHECK(annual.step_results[1].storage_inventory_balance_error_tco2 ==
        Approx(0.0).margin(1e-10));
  CHECK(annual.storage_inventory_balance_abs_error_tco2 ==
        Approx(0.0).margin(1e-10));
}

TEST_CASE("Dynamic storage ledger exposes inconsistent SOC snapshots",
          "[carbonflow][storage][energy-balance][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem step = build_single_bus_storage_case();
  step.ac.storage[0].eta_charge = 0.8;
  step.ac.storage[0].self_discharge_pct = 0.0;
  step.ac.storage[0].p_mw = -10.0;
  step.ac.storage[0].soc_init = 0.59;
  step.ac.storage[0].e_mwh = 59.0;

  TimeSeriesPFResult ts;
  ts.num_steps = 1;
  ts.pf_results = {converged_pf()};
  ts.pf_system_snapshots = {step};

  const AnnualCarbonAnalysisResult annual =
      compute_annual_carbon_analysis(ts, 1.0);
  // Inverting the first terminal snapshot yields a consistent inferred start,
  // so a one-step series alone cannot audit an externally supplied initial SOC.
  CHECK(annual.storage_energy_balance_error_mwh == Approx(0.0).margin(1e-10));

  HybridPowerSystem next = step;
  next.ac.storage[0].p_mw = 0.0;
  next.ac.storage[0].soc_init = 0.60;
  next.ac.storage[0].e_mwh = 60.0;
  ts.num_steps = 2;
  ts.pf_results.push_back(converged_pf());
  ts.pf_system_snapshots.push_back(next);
  const AnnualCarbonAnalysisResult inconsistent =
      compute_annual_carbon_analysis(ts, 1.0);
  CHECK(inconsistent.storage_energy_balance_error_mwh ==
        Approx(-1.0).margin(1e-10));
  CHECK(inconsistent.storage_energy_balance_abs_error_mwh ==
        Approx(1.0).margin(1e-10));
  CHECK(inconsistent.max_storage_step_energy_balance_error_mwh ==
        Approx(1.0).margin(1e-10));
}
