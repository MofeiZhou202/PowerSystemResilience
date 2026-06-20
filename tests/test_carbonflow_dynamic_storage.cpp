#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

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
