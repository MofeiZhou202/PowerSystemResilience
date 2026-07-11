#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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
  ts.profiles = {{0, "load", {1.0, 1.0}}};

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
