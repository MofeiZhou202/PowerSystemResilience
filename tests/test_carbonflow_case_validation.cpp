#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <iostream>

#include <cmath>
#include <string>
#include <vector>

#ifndef HACDCPF_MATPOWER_DATA_DIR
#define HACDCPF_MATPOWER_DATA_DIR "../../external_data/matpower"
#endif

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
  CHECK(left == Approx(right).epsilon(1e-8).margin(1e-7));
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

hacdcpf::TimeSeriesData make_carbon_24h_profiles() {
  using namespace hacdcpf;
  const std::vector<double> load = {
      0.50, 0.45, 0.42, 0.40, 0.42, 0.50, 0.60, 0.72,
      0.80, 0.85, 0.88, 0.90, 0.88, 0.85, 0.82, 0.85,
      0.90, 1.00, 1.10, 1.05, 0.95, 0.85, 0.72, 0.60};
  const std::vector<double> wind = {
      0.65, 0.70, 0.75, 0.80, 0.72, 0.55, 0.35, 0.20,
      0.15, 0.10, 0.18, 0.25, 0.30, 0.40, 0.55, 0.60,
      0.50, 0.35, 0.25, 0.30, 0.45, 0.55, 0.60, 0.65};
  const std::vector<double> solar = {
      0.00, 0.00, 0.00, 0.00, 0.00, 0.02, 0.10, 0.30,
      0.55, 0.80, 0.92, 1.00, 0.98, 0.90, 0.75, 0.55,
      0.30, 0.10, 0.02, 0.00, 0.00, 0.00, 0.00, 0.00};

  TimeSeriesData ts;
  ts.num_steps = 24;
  ts.step_duration_hr = 1.0;
  ts.profiles = {
      TimeSeriesProfile{0, "daily_load", load},
      TimeSeriesProfile{1, "wind_daily", wind},
      TimeSeriesProfile{2, "solar_daily", solar},
      TimeSeriesProfile{3, "hydro_daily", std::vector<double>(24, 0.6)},
  };
  return ts;
}

void bind_default_profiles(hacdcpf::HybridPowerSystem& sys) {
  using namespace hacdcpf;
  for (auto& load : sys.ac.loads) {
    if (load.profile_id < 0) load.profile_id = 0;
  }
  for (auto& load : sys.dc.loads) {
    if (load.profile_id < 0) load.profile_id = 0;
  }
  for (auto& gen : sys.ac.renewable_gens) {
    if (gen.profile_id >= 0) continue;
    gen.profile_id = (gen.type == RenewableType::SolarPV ||
                      gen.type == RenewableType::SolarCSP)
                         ? 2
                         : 1;
  }
  for (auto& pv : sys.ac.pv_systems) {
    if (pv.profile_id < 0) pv.profile_id = 2;
  }
  for (auto& pv : sys.dc.pv_arrays) {
    if (pv.profile_id < 0) pv.profile_id = 2;
  }
}

}  // namespace

TEST_CASE("Downloaded MATPOWER cases keep carbon balance with thermal 900 kg/MWh",
          "[carbonflow][matpower][balance]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  const std::vector<std::string> cases = {"case14.m", "case33bw.m"};
  for (const auto& name : cases) {
    const std::string path = std::string(HACDCPF_MATPOWER_DATA_DIR) + "/" + name;
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

TEST_CASE("Carbon flow uses recovered branch-flow order on switched dist33 DER case",
          "[carbonflow][branch-mapping][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys = io::build_dist33_microgrid_der();
  assign_carbon_factors(sys);

  PowerFlowOptions pf_opt;
  pf_opt.max_iter = 100;
  pf_opt.tol = 1e-8;
  const PowerFlowResult pf = solve_power_flow(sys, pf_opt);
  REQUIRE(pf.converged);
  REQUIRE(pf.branch_flows.size() >= sys.ac.branches.size());
  for (const auto& transfer : pf.vsc_transfers) {
    UNSCOPED_INFO("VSC " << transfer.index << ": Pac=" << transfer.p_ac_mw
                  << " MW, Pdc=" << transfer.p_dc_mw
                  << " MW, loss=" << transfer.loss_mw << " MW");
  }

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
  INFO("max nodal power mismatch = " << ca.max_node_power_balance_error_mw << " MW");
  for (const auto& error : ca.node_power_balance_errors) {
    UNSCOPED_INFO((error.is_dc ? "DC" : "AC") << " bus " << error.bus_index
                  << ": source=" << error.source_mw
                  << " MW, required=" << error.required_mw
                  << " MW, mismatch=" << error.mismatch_mw << " MW");
  }
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.branch_carbon.size() == sys.ac.branches.size());

  for (size_t i = 0; i < sys.ac.branches.size(); ++i) {
    INFO("AC branch position " << i);
    const auto& br = sys.ac.branches[i];
    const auto& bf = pf.branch_flows[i];
    const auto& bc = ca.branch_carbon[i];
    CHECK(bc.branch_index == br.index);
    CHECK(bc.from_bus == br.from_bus);
    CHECK(bc.to_bus == br.to_bus);
    CHECK(bc.loss_mw == Approx(std::max(bf.pf_mw + bf.pt_mw, 0.0)).margin(1e-9));
  }
}

TEST_CASE("Market 5-bus AC/DC toy has angle reference in each AC island",
          "[powerflow][case-builder][regression]") {
  using namespace hacdcpf;

  HybridPowerSystem sys = io::build_market_5bus_acdc_toy();

  PowerFlowOptions pf_opt;
  pf_opt.max_iter = 100;
  pf_opt.tol = 1e-8;
  const PowerFlowResult pf = solve_power_flow(sys, pf_opt);

  REQUIRE(pf.converged);
  CHECK(pf.iterations <= 20);
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
  REQUIRE(annual.num_carbon_verified == 3);
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

  const double initial_inventory = 50.0 * 0.2;
  const double terminal_inventory =
      55.0 * charged_intensity + 12.0 * kThermalEfTco2Mwh;
  CHECK(annual.initial_storage_carbon_inventory_tco2 ==
        Approx(initial_inventory).margin(1e-10));
  CHECK(annual.terminal_storage_carbon_inventory_tco2 ==
        Approx(terminal_inventory).margin(1e-10));
  CHECK(annual.storage_carbon_inventory_delta_tco2 ==
        Approx(terminal_inventory - initial_inventory).margin(1e-10));
  CHECK(annual.storage_internal_loss_emissions_tco2 ==
        Approx(0.0).margin(1e-10));

  const double external_generation =
      annual.total_generation_emissions_tco2 -
      annual.total_storage_discharge_emissions_tco2;
  const double terminal_load =
      annual.total_load_emissions_tco2 -
      annual.total_storage_charge_emissions_tco2;
  CHECK(external_generation + initial_inventory ==
        Approx(terminal_load + annual.total_loss_emissions_tco2 +
               annual.storage_internal_loss_emissions_tco2 +
               terminal_inventory)
            .epsilon(1e-8)
            .margin(1e-8));
  CHECK(annual.balance_error_storage_adjusted_tco2 ==
        Approx(0.0).margin(1e-10));
  CHECK(annual.balance_error_storage_adjusted_pct ==
        Approx(0.0).margin(1e-10));

  REQUIRE(annual.terminal_storage_states.size() == 1);
  CHECK(annual.terminal_storage_states[0].stored_energy_mwh == Approx(67.0));
  CHECK(annual.terminal_storage_states[0].soc_carbon_intensity_tco2_mwh ==
        Approx(terminal_inventory / 67.0).margin(1e-10));
}

TEST_CASE("Built-in hybrid cases preserve dynamic carbon over 24 hours",
          "[.][carbonflow][24h][integration]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  struct CaseSpec {
    const char* name;
    HybridPowerSystem (*build)();
    bool apply_grid_profile;
  };
  const std::vector<CaseSpec> cases = {
      {"dist33_microgrid_der", &io::build_dist33_microgrid_der, false},
      {"comprehensive_hybrid_acdc", &io::build_comprehensive_hybrid_acdc, true},
  };

  for (const auto& spec : cases) {
    DYNAMIC_SECTION(spec.name) {
      HybridPowerSystem sys = spec.build();
      assign_carbon_factors(sys);
      bind_default_profiles(sys);

      TimeSeriesPFOptions pf_options;
      pf_options.uc_solver = UCSolverChoice::HiGHS;
      pf_options.run_opf = false;
      pf_options.keep_system_snapshots = true;
      pf_options.enforce_terminal_soc_cyclic = true;

      TimeSeriesPFResult ts =
          solve_time_series_pf(sys, make_carbon_24h_profiles(), pf_options);
      REQUIRE(ts.num_steps == 24);
      REQUIRE(ts.num_converged == 24);
      REQUIRE(ts.pf_system_snapshots.size() == 24);

      if (spec.apply_grid_profile) {
        for (size_t t = 0; t < ts.pf_system_snapshots.size(); ++t) {
          const double factor = 0.35 + 0.55 * static_cast<double>(t) / 23.0;
          for (auto& grid : ts.pf_system_snapshots[t].ac.external_grids) {
            grid.emission_factor_tco2_mwh = factor;
          }
        }
      }

      AnnualCarbonAnalysisOptions carbon_options;
      carbon_options.keep_hourly_bus_intensity = true;
      const AnnualCarbonAnalysisResult annual =
          compute_annual_carbon_analysis(ts, 1.0, carbon_options);

      std::cout << spec.name
                << ": PF=" << annual.num_pf_converged << "/24"
                << ", carbon=" << annual.num_carbon_verified << "/24"
                << ", max_KCL_MW=" << annual.max_node_power_balance_error_mw
                << ", max_rel_res=" << annual.max_matrix_relative_residual
                << ", max_cond=" << annual.max_matrix_condition_estimate
                << ", storage_E_abs_MWh="
                << annual.storage_energy_balance_abs_error_mwh
                << ", storage_C_abs_t="
                << annual.storage_inventory_balance_abs_error_tco2
                << ", adjusted_balance_pct="
                << annual.balance_error_storage_adjusted_pct << '\n';

      if (annual.num_carbon_verified != 24) {
        if (!ts.pf_results.empty()) {
          const auto& first_pf = ts.pf_results.front();
          std::cout << "  first-step VSC transfers:\n";
          for (const auto& transfer : first_pf.vsc_transfers) {
            std::cout << "    VSC-" << transfer.index
                      << ": AC=" << transfer.p_ac_mw
                      << ", DC=" << transfer.p_dc_mw
                      << ", loss=" << transfer.loss_mw << '\n';
          }
          std::cout << "  first-step DCDC transfers:\n";
          for (const auto& transfer : first_pf.dcdc_transfers) {
            std::cout << "    DCDC-" << transfer.index
                      << ": in=" << transfer.p_in_mw
                      << ", out=" << transfer.p_out_mw
                      << ", loss=" << transfer.loss_mw << '\n';
          }
          std::cout << "  first-step effective VSC modes:\n";
          for (const auto& converter : first_pf.diagnostics.effective_converters) {
            std::cout << "    VSC-" << converter.index
                      << ": mode=" << static_cast<int>(converter.control_mode)
                      << ", AC-bus=" << converter.bus_ac
                      << ", DC-bus=" << converter.bus_dc << '\n';
          }
        }
        for (size_t t = 0; t < ts.pf_results.size(); ++t) {
          const CarbonAnalysisResult ca = compute_carbon_analysis(
              ts.pf_system_snapshots[t], ts.pf_results[t]);
          if (ca.tracing_verified) continue;
          std::cout << "  step " << t
                    << ": power_ok=" << ca.power_balance_verified
                    << ", matrix_ok=" << ca.matrix_solved
                    << ", max_KCL_MW=" << ca.max_node_power_balance_error_mw
                    << '\n';
          for (const auto& error : ca.node_power_balance_errors) {
            std::cout << "    " << (error.is_dc ? "DC-" : "AC-")
                      << error.bus_index << ": source=" << error.source_mw
                      << ", required=" << error.required_mw
                      << ", mismatch=" << error.mismatch_mw << '\n';
          }
        }
      }

      REQUIRE(annual.num_pf_converged == 24);
      REQUIRE(annual.num_carbon_power_balance_verified == 24);
      REQUIRE(annual.num_carbon_matrix_solved == 24);
      REQUIRE(annual.num_carbon_verified == 24);
      CHECK(annual.storage_energy_balance_abs_error_mwh < 1e-7);
      CHECK(annual.storage_inventory_balance_abs_error_tco2 < 1e-7);
      CHECK(annual.balance_error_storage_adjusted_pct < 1e-6);
    }
  }
}
