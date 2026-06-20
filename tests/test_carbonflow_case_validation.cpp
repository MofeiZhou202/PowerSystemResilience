#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <vector>

namespace {

using Approx = Catch::Approx;

constexpr double kThermalEfTco2Mwh = 0.9;  // 900 kgCO2/MWh
constexpr double kZeroEfTco2Mwh = 0.0;

hacdcpf::ACBus make_bus(int index, hacdcpf::BusType type) {
  hacdcpf::ACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.in_service = true;
  return bus;
}

hacdcpf::Generator make_thermal_generator(int index,
                                          int bus,
                                          double p_mw,
                                          bool is_slack = false) {
  hacdcpf::Generator gen;
  gen.index = index;
  gen.bus = bus;
  gen.in_service = true;
  gen.is_slack = is_slack;
  gen.pg_mw = p_mw;
  gen.pmin_mw = 0.0;
  gen.pmax_mw = 500.0;
  gen.fuel_type = hacdcpf::FuelType::Coal;
  gen.emission_factor_tco2_mwh = kThermalEfTco2Mwh;
  return gen;
}

hacdcpf::RenewableGen make_wind_generator(int index, int bus, double p_mw) {
  hacdcpf::RenewableGen gen;
  gen.index = index;
  gen.bus = bus;
  gen.in_service = true;
  gen.type = hacdcpf::RenewableType::Wind;
  gen.p_mw = p_mw;
  return gen;
}

hacdcpf::PVSystem make_pv_system(int index, int bus, double p_mw) {
  hacdcpf::PVSystem pv;
  pv.index = index;
  pv.bus = bus;
  pv.in_service = true;
  pv.p_mw = p_mw;
  return pv;
}

hacdcpf::Load make_load(int index, int bus, double p_mw) {
  hacdcpf::Load load;
  load.index = index;
  load.bus = bus;
  load.in_service = true;
  load.p_mw = p_mw;
  return load;
}

hacdcpf::ACBranch make_branch(int index, int from_bus, int to_bus) {
  hacdcpf::ACBranch branch;
  branch.index = index;
  branch.from_bus = from_bus;
  branch.to_bus = to_bus;
  branch.in_service = true;
  return branch;
}

void assign_carbon_factors(hacdcpf::HybridPowerSystem& sys) {
  for (auto& gen : sys.ac.generators) {
    switch (gen.fuel_type) {
      case hacdcpf::FuelType::Wind:
      case hacdcpf::FuelType::Solar:
      case hacdcpf::FuelType::Hydro:
        gen.emission_factor_tco2_mwh = kZeroEfTco2Mwh;
        break;
      default:
        gen.emission_factor_tco2_mwh = kThermalEfTco2Mwh;
        break;
    }
  }

  for (auto& gen : sys.ac.static_generators) {
    if (gen.sgen_type == hacdcpf::SgenType::PV ||
        gen.sgen_type == hacdcpf::SgenType::Wind) {
      gen.co2_emission_rate = kZeroEfTco2Mwh;
    } else {
      gen.co2_emission_rate = kThermalEfTco2Mwh;
    }
  }

  for (auto& gen : sys.dc.static_generators) {
    if (gen.sgen_type == hacdcpf::SgenType::PV ||
        gen.sgen_type == hacdcpf::SgenType::Wind) {
      gen.co2_emission_rate = kZeroEfTco2Mwh;
    } else {
      gen.co2_emission_rate = kThermalEfTco2Mwh;
    }
  }
}

void require_carbon_balance(const hacdcpf::analysis::EmissionsSummary& summary,
                            double tolerance_pct = 1e-6) {
  const double left = summary.total_generation_emissions_tco2;
  const double right =
      summary.total_load_emissions_tco2 + summary.total_loss_emissions_tco2;
  CHECK(left == Approx(right).epsilon(1e-8).margin(1e-8));
  CHECK(summary.balance_error_pct < tolerance_pct);
}

hacdcpf::PowerFlowResult one_bus_pf() {
  hacdcpf::PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};
  return pf;
}

hacdcpf::HybridPowerSystem make_storage_step(double gen_mw,
                                             double storage_p_mw,
                                             double soc,
                                             double storage_intensity) {
  using namespace hacdcpf;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK)};
  sys.ac.generators = {make_thermal_generator(1, 1, gen_mw, true)};
  sys.ac.loads = {make_load(1, 1, 10.0)};

  Storage storage;
  storage.index = 1;
  storage.bus = 1;
  storage.in_service = true;
  storage.p_mw = storage_p_mw;
  storage.e_rated_mwh = 100.0;
  storage.soc_init = soc;
  storage.e_mwh = soc * storage.e_rated_mwh;
  storage.eta_charge = 1.0;
  storage.eta_discharge = 1.0;
  storage.soc_carbon_intensity_tco2_mwh = storage_intensity;
  sys.ac.storage = {storage};
  return sys;
}

}  // namespace

TEST_CASE("Downloaded MATPOWER cases keep carbon balance with thermal 900 kg/MWh",
          "[carbonflow][matpower][balance]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  const std::vector<std::string> cases = {"case14.m", "case33bw.m"};
  for (const auto& name : cases) {
    const std::string path = std::string(HACDCPF_TEST_DATA_DIR) + "/" + name;
    INFO("MATPOWER case: " << path);

    HybridPowerSystem sys = io::parse_matpower(path);
    assign_carbon_factors(sys);

    PowerFlowOptions pf_opt;
    pf_opt.max_iter = 200;
    pf_opt.tol = 1e-8;
    const PowerFlowResult pf = solve_power_flow(sys, pf_opt);
    REQUIRE(pf.converged);

    const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
    REQUIRE(ca.tracing_verified);
    REQUIRE(ca.matrix_solved);
    require_carbon_balance(ca.tracing_summary, 1e-4);
    require_carbon_balance(ca.matrix_summary, 1e-4);
  }
}

TEST_CASE("Wind and PV are zero-carbon while thermal is 900 kg/MWh",
          "[carbonflow][renewable][tracing]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PV),
      make_bus(3, BusType::PV),
      make_bus(4, BusType::PQ),
  };
  sys.ac.generators = {make_thermal_generator(1, 1, 90.0)};
  sys.ac.renewable_gens = {make_wind_generator(1, 2, 30.0)};
  sys.ac.pv_systems = {make_pv_system(1, 3, 20.0)};
  sys.ac.loads = {make_load(1, 4, 140.0)};
  sys.ac.branches = {
      make_branch(1, 1, 4),
      make_branch(2, 2, 4),
      make_branch(3, 3, 4),
  };

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0, 0.0};
  pf.branch_flows = {
      BranchFlow{90.0, 0.0, -90.0, 0.0},
      BranchFlow{30.0, 0.0, -30.0, 0.0},
      BranchFlow{20.0, 0.0, -20.0, 0.0},
  };

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 1);
  const auto& load = ca.load_carbon[0];
  CHECK(load.generator_supply_mw.size() == 3);
  CHECK(load.generator_supply_mw.at(0) == Approx(90.0));
  CHECK(load.generator_supply_mw.at(1) == Approx(30.0));
  CHECK(load.generator_supply_mw.at(2) == Approx(20.0));
  CHECK(load.carbon_intensity_tco2_mwh == Approx((90.0 * kThermalEfTco2Mwh) / 140.0));
  require_carbon_balance(ca.tracing_summary);
  require_carbon_balance(ca.matrix_summary);
}

TEST_CASE("Three-step storage recursion preserves carbon balance",
          "[carbonflow][storage][annual][balance]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  TimeSeriesPFResult ts_result;
  ts_result.num_steps = 3;
  ts_result.pf_results = {one_bus_pf(), one_bus_pf(), one_bus_pf()};
  ts_result.pf_system_snapshots = {
      make_storage_step(20.0, -10.0, 0.60, 0.2),
      make_storage_step(5.0, 5.0, 0.55, 0.0),
      make_storage_step(22.0, -12.0, 0.67, 0.0),
  };

  AnnualCarbonAnalysisOptions opts;
  opts.keep_hourly_load_emissions = true;
  opts.keep_hourly_load_energy = true;

  const AnnualCarbonAnalysisResult annual =
      compute_annual_carbon_analysis(ts_result, 1.0, opts);

  REQUIRE(annual.num_steps == 3);
  REQUIRE(annual.num_pf_converged == 3);
  REQUIRE(annual.step_results.size() == 3);

  const double charged_intensity = (50.0 * 0.2 + 10.0 * kThermalEfTco2Mwh) / 60.0;
  CHECK(annual.step_results[0].total_generation_emissions_tco2 ==
        Approx(20.0 * kThermalEfTco2Mwh));
  CHECK(annual.step_results[1].total_generation_emissions_tco2 ==
        Approx(5.0 * kThermalEfTco2Mwh + 5.0 * charged_intensity));
  CHECK(annual.step_results[2].total_generation_emissions_tco2 ==
        Approx(22.0 * kThermalEfTco2Mwh));

  for (const auto& step : annual.step_results) {
    CHECK(step.total_generation_emissions_tco2 ==
          Approx(step.total_load_emissions_tco2 + step.total_loss_emissions_tco2)
              .epsilon(1e-8)
              .margin(1e-8));
  }
  CHECK(annual.total_generation_emissions_tco2 ==
        Approx(annual.total_load_emissions_tco2 + annual.total_loss_emissions_tco2)
            .epsilon(1e-8)
            .margin(1e-8));
}
