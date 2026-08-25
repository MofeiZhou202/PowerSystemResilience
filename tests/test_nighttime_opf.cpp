/// @brief Quick smoke test: nansha_full_network OPF at nighttime (no PV).
///
/// Without the fix, solve_ac_opf immediately returns failure when
/// sys.ac.generators is empty (ng == 0). With the fix, external grids
/// with cost_c2 > 0 are injected as generator variables, so OPF converges.

#include <cmath>
#include <filesystem>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/time_series/time_series_pf.hpp"

namespace fs = std::filesystem;

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "../../"
#endif

static hacdcpf::HybridPowerSystem load_nansha() {
  const fs::path json_path =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "external_data/classical_example/nansha_full_network.json";
  REQUIRE(fs::exists(json_path));
  auto result = hacdcpf::io::try_load_json(json_path.string());
  REQUIRE(result.has_value());
  return std::move(result.value());
}

static hacdcpf::HybridPowerSystem load_canvas_power_system() {
  const fs::path json_path =
      fs::path(HACDCPF_PROJECT_ROOT) /
      "external_data/classical_example/power_system.json";
  REQUIRE(fs::exists(json_path));
  auto result = hacdcpf::io::try_load_json(json_path.string());
  REQUIRE(result.has_value());
  return std::move(result.value());
}

TEST_CASE("Canvas power_system OPF converges with finite external-grid dispatch",
          "[opf][external_grid][hybrid][regression]") {
  const auto sys = load_canvas_power_system();
  REQUIRE(sys.ac.generators.empty());
  REQUIRE(sys.ac.external_grids.size() == 1);
  REQUIRE(sys.ac.transformers_2w.size() == 1);
  CHECK(sys.ac.transformers_2w[0].hv_bus == 1);
  CHECK(sys.ac.transformers_2w[0].lv_bus == 2);

  hacdcpf::opf::ACOPFOptions opts;
  opts.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
  opts.allow_fallback = false;
  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO(result.status);
  REQUIRE(result.converged);
  REQUIRE(result.external_grid_p_mw.size() == 1);
  REQUIRE(result.external_grid_q_mvar.size() == 1);
  CHECK(std::isfinite(result.external_grid_p_mw[0]));
  CHECK(std::isfinite(result.external_grid_q_mvar[0]));
  REQUIRE(result.pstor_mw.size() == 1);
  CHECK_THAT(result.pstor_mw[0],
             Catch::Matchers::WithinAbs(sys.dc.dc_storage[0].p_mw, 2e-4));
  CHECK(result.external_grid_p_mw[0] > 0.0);
  REQUIRE(result.pac_mw.size() == 1);
  REQUIRE(result.qac_mvar.size() == 1);
  CHECK_THAT(result.qac_mvar[0],
             Catch::Matchers::WithinAbs(sys.vsc_converters[0].q_set_mvar, 5e-2));
  CHECK(result.external_grid_q_mvar[0] > 0.0);
  CHECK(result.external_grid_q_mvar[0] < 0.20);
  REQUIRE(result.pdcdc_mw.size() == 1);
  REQUIRE(result.vdc.size() >= 4);
  const auto& dcdc = sys.dc.dcdc_converters[0];
  const double p_in_pu = result.pdcdc_mw[0] / sys.base_mva;
  const double p_out_pu =
      p_in_pu / dcdc.eta -
      dcdc.r_eq_pu * p_in_pu * p_in_pu /
          (result.vdc[static_cast<size_t>(dcdc.bus_in - 1)] *
           result.vdc[static_cast<size_t>(dcdc.bus_in - 1)]);
  CHECK_THAT(-p_out_pu * sys.base_mva,
             Catch::Matchers::WithinAbs(sys.dc.pv_arrays[0].p_set_mw, 2e-4));
  const double ac_load_mw = sys.ac.loads[0].p_mw * sys.ac.loads[0].scaling;
  const double ac_supply_mw =
      result.external_grid_p_mw[0] + result.pac_mw[0];
  // Grid1 supplies Bus1 through the transformer while the VSC supplies Bus3
  // through CB34. Their combined injection serves the Bus2 load plus losses;
  // neither source is a duplicate slack injection.
  CHECK(ac_supply_mw >= ac_load_mw);
  CHECK_THAT(ac_supply_mw,
             Catch::Matchers::WithinAbs(ac_load_mw, 1e-3));
}

static hacdcpf::TimeSeriesData canvas_daily_profiles(int steps) {
  static const std::vector<double> load24 = {
      0.50,0.45,0.42,0.40,0.42,0.50,0.60,0.72,0.80,0.85,0.88,0.90,
      0.88,0.85,0.82,0.85,0.90,1.00,1.10,1.05,0.95,0.85,0.72,0.60};
  static const std::vector<double> wind24 = {
      0.65,0.70,0.75,0.80,0.72,0.55,0.35,0.20,0.15,0.10,0.18,0.25,
      0.30,0.40,0.55,0.60,0.50,0.35,0.25,0.30,0.45,0.55,0.60,0.65};
  static const std::vector<double> solar24 = {
      0.00,0.00,0.00,0.00,0.00,0.02,0.10,0.30,0.55,0.80,0.92,1.00,
      0.98,0.90,0.75,0.55,0.30,0.10,0.02,0.00,0.00,0.00,0.00,0.00};
  hacdcpf::TimeSeriesData ts;
  ts.num_steps = steps;
  ts.step_duration_hr = 1.0;
  auto add = [&](int id, const char* name, const std::vector<double>& values) {
    REQUIRE(values.size() == 24);
    hacdcpf::TimeSeriesProfile profile;
    profile.id = id;
    profile.name = name;
    for (int t = 0; t < steps; ++t)
      profile.values.push_back(values[static_cast<size_t>(t % 24)]);
    ts.profiles.push_back(std::move(profile));
  };
  add(0, "daily_load", load24);
  add(1, "wind_daily", wind24);
  add(2, "solar_daily", solar24);
  return ts;
}

TEST_CASE("Canvas power_system time-series production paths converge",
          "[time_series][opf][external_grid][hybrid][regression]") {
  auto sys = load_canvas_power_system();
  for (auto& load : sys.ac.loads) if (load.profile_id < 0) load.profile_id = 0;
  for (auto& load : sys.dc.loads) if (load.profile_id < 0) load.profile_id = 0;
  for (auto& pv : sys.dc.pv_arrays) if (pv.profile_id < 0) pv.profile_id = 2;

  SECTION("24-step dynamic OPF") {
    hacdcpf::TimeSeriesPFOptions opts;
    opts.skip_uc = true;
    opts.run_opf = true;
    opts.enable_external_grid = true;
    opts.opf_options.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
    const auto result = hacdcpf::solve_time_series_pf(
        sys, canvas_daily_profiles(24), opts);
    INFO("PF=" << result.num_converged << "/24, OPF="
               << result.num_opf_converged << "/24");
    CHECK(result.num_opf_converged == 24);
    CHECK(result.num_converged == 24);
  }

  SECTION("default daily production UC to OPF to PF") {
    hacdcpf::TimeSeriesPFOptions opts;
    opts.skip_uc = false;
    opts.run_opf = true;
    opts.enable_external_grid = true;
    opts.enforce_terminal_soc_cyclic = true;
    opts.opf_options.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
    const auto result = hacdcpf::solve_time_series_pf(
        sys, canvas_daily_profiles(4), opts);
    INFO("UC feasible=" << result.uc_schedule.feasible
                         << ", PF=" << result.num_converged
                         << "/4, OPF=" << result.num_opf_converged << "/4");
    CHECK(result.uc_schedule.feasible);
    CHECK(result.num_opf_converged == 4);
    CHECK(result.num_converged == 4);
  }

  SECTION("parallel days keep rich DC storage cyclic") {
    REQUIRE(sys.dc.storage.empty());
    REQUIRE(sys.dc.dc_storage.size() == 1);

    hacdcpf::TimeSeriesPFOptions opts;
    opts.skip_uc = true;
    opts.run_opf = false;
    opts.parallel_daily = true;
    opts.parallel_threads = 2;
    const auto result = hacdcpf::solve_time_series_pf(
        sys, canvas_daily_profiles(48), opts);

    INFO(result.uc_schedule.solver_status);
    REQUIRE(result.parallel_daily_effective);
    REQUIRE(result.uc_schedule.feasible);
    REQUIRE(result.uc_schedule.dc_ess_soc.size() == 1);
    REQUIRE(result.uc_schedule.dc_ess_soc[0].size() == 48);
    const double initial_soc = sys.dc.dc_storage[0].soc_init;
    CHECK_THAT(result.uc_schedule.dc_ess_soc[0][23],
               Catch::Matchers::WithinAbs(initial_soc, 1e-8));
    CHECK_THAT(result.uc_schedule.dc_ess_soc[0][47],
               Catch::Matchers::WithinAbs(initial_soc, 1e-8));
  }
}

TEST_CASE("Nansha OPF: nighttime (no generators) converges via external grid cost",
          "[opf][nighttime][nansha]") {
  auto sys = load_nansha();

  // Verify precondition: nansha has no conventional generators
  INFO("Number of generators: " << sys.ac.generators.size());
  CHECK(sys.ac.generators.empty());

  // Verify precondition: external grids have cost set
  bool any_eg_cost = false;
  for (const auto& eg : sys.ac.external_grids)
    if (eg.cost_c2 > 0.0 || eg.cost_c1 > 0.0) any_eg_cost = true;
  INFO("Number of external grids with cost: checked");
  REQUIRE(any_eg_cost);

  // The loaded JSON already has no ac.generators — this is the nighttime condition.
  // External grids with cost_c2 > 0 must be promoted to OPF variables for convergence.

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false; // strict: do not fall back to PF

  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO("OPF status: " << result.status);
  INFO("OPF iterations: " << result.iterations);
  for (const auto& h : result.infeasibility_hints)
    INFO("Hint: " << h);

  CHECK(result.converged);
}

TEST_CASE("Nansha OPF: daytime (PV present) also converges",
          "[opf][daytime][nansha]") {
  auto sys = load_nansha();

  // Leave renewable gen power as-is from the JSON (daytime representative)
  // Just ensure the base case runs — this was working before the fix too.

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false;

  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO("OPF status: " << result.status);
  CHECK(result.converged);
}

TEST_CASE("AC OPF dispatches external grid independently on a shared bus",
          "[opf][external_grid]") {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  hacdcpf::ACBus bus;
  bus.index = 1;
  bus.bus_type = hacdcpf::BusType::SLACK;
  sys.ac.buses.push_back(bus);
  hacdcpf::Load load;
  load.index = 1;
  load.bus = 1;
  load.p_mw = 50.0;
  sys.ac.loads.push_back(load);
  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.is_slack = true;
  gen.pmax_mw = 100.0;
  gen.qmax_mvar = 100.0;
  gen.qmin_mvar = -100.0;
  gen.cost_c1 = 100.0;
  sys.ac.generators.push_back(gen);
  hacdcpf::ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.cost_c1 = 10.0;
  sys.ac.external_grids.push_back(grid);

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false;
  opts.enable_primal_dual = true;
  opts.use_parity_ipm = true;
  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO(result.status);
  REQUIRE(result.converged);
  REQUIRE(result.pg_mw.size() == 1);
  REQUIRE(result.gen_map.size() == 1);
  CHECK(result.gen_map[0].original_index == 0);
  REQUIRE(result.external_grid_p_mw.size() == 1);
  CHECK_THAT(result.pg_mw[0],
             Catch::Matchers::WithinAbs(0.0, 1e-2));
  CHECK_THAT(result.external_grid_p_mw[0],
             Catch::Matchers::WithinAbs(50.0, 1e-2));
}

TEST_CASE("AC OPF generator results remain in authored order after island stripping",
          "[opf][projection][generator_map][regression]") {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  for (int i = 1; i <= 3; ++i) {
    hacdcpf::ACBus bus;
    bus.index = i;
    bus.bus_type = i == 1 ? hacdcpf::BusType::SLACK
                          : hacdcpf::BusType::PQ;
    sys.ac.buses.push_back(bus);
  }
  hacdcpf::ACBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.r_pu = 0.01;
  branch.x_pu = 0.05;
  branch.rate_a_mva = 100.0;
  sys.ac.branches.push_back(branch);
  hacdcpf::Load load;
  load.index = 1;
  load.bus = 2;
  load.p_mw = 10.0;
  sys.ac.loads.push_back(load);
  hacdcpf::Generator live;
  live.index = 101;
  live.bus = 1;
  live.is_slack = true;
  live.pmax_mw = 100.0;
  live.qmin_mvar = -100.0;
  live.qmax_mvar = 100.0;
  hacdcpf::Generator stripped = live;
  stripped.index = 202;
  stripped.bus = 3;
  stripped.is_slack = false;
  stripped.pg_mw = 0.0;
  sys.ac.generators = {live, stripped};

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false;
  opts.enable_primal_dual = true;
  opts.use_parity_ipm = true;
  const auto result = hacdcpf::solve_ac_opf(sys, opts);

  INFO(result.status);
  REQUIRE(result.converged);
  REQUIRE(result.pg_mw.size() == 2);
  REQUIRE(result.gen_map.size() == 2);
  CHECK(result.gen_map[0].original_index == 0);
  CHECK(result.gen_map[1].original_index == 1);
  // The live generator supplies the 10 MW demand plus the positive loss of the
  // r=0.01 pu branch; authored ordering is certified by gen_map and the exact
  // zero restored for the stripped generator, not by assuming a lossless grid.
  CHECK(result.pg_mw[0] > load.p_mw);
  CHECK(result.pg_mw[0] - load.p_mw < 0.02);
  CHECK(result.pg_mw[1] == 0.0);
}

TEST_CASE("DC OPF dispatches external grid independently on a shared bus",
          "[opf][external_grid]") {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  hacdcpf::ACBus bus;
  bus.index = 1;
  bus.bus_type = hacdcpf::BusType::SLACK;
  sys.ac.buses.push_back(bus);
  hacdcpf::Load load;
  load.index = 1;
  load.bus = 1;
  load.p_mw = 50.0;
  sys.ac.loads.push_back(load);
  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.pmax_mw = 100.0;
  gen.cost_c1 = 100.0;
  sys.ac.generators.push_back(gen);
  hacdcpf::ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.cost_c1 = 10.0;
  sys.ac.external_grids.push_back(grid);

  hacdcpf::opf::DCOPFOptions opts;
  opts.solver = hacdcpf::opf::DCOPFSolverBackend::NativeQP;
  const auto result = hacdcpf::solve_dc_opf(sys, opts);

  INFO(result.status);
  REQUIRE(result.converged);
  REQUIRE(result.pg_mw.size() == 1);
  REQUIRE(result.external_grid_p_mw.size() == 1);
  CHECK_THAT(result.pg_mw[0],
             Catch::Matchers::WithinAbs(0.0, 1e-2));
  CHECK_THAT(result.external_grid_p_mw[0],
             Catch::Matchers::WithinAbs(50.0, 1e-2));
}

TEST_CASE("Canonical bus merge preserves attached OPF sources and loads",
          "[opf][external_grid][projection][switch]") {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  hacdcpf::ACBus source_bus;
  source_bus.index = 10;
  source_bus.bus_type = hacdcpf::BusType::SLACK;
  hacdcpf::ACBus load_bus;
  load_bus.index = 20;
  load_bus.bus_type = hacdcpf::BusType::PQ;
  sys.ac.buses = {source_bus, load_bus};
  hacdcpf::ExternalGrid grid;
  grid.index = 1;
  grid.bus = 10;
  grid.cost_c1 = 10.0;
  sys.ac.external_grids.push_back(grid);
  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 20;
  gen.pmax_mw = 100.0;
  gen.qmax_mvar = 100.0;
  gen.qmin_mvar = -100.0;
  gen.cost_c1 = 100.0;
  sys.ac.generators.push_back(gen);
  hacdcpf::Load load;
  load.index = 1;
  load.bus = 20;
  load.p_mw = 50.0;
  sys.ac.loads.push_back(load);
  hacdcpf::Switch sw;
  sw.index = 1;
  sw.bus_from = 10;
  sw.bus_to = 20;
  sw.closed = true;
  sys.ac.switches.push_back(sw);

  const auto canonical =
      hacdcpf::project_to_canonical_models(sys, /*strip_dead=*/false);
  REQUIRE(canonical.ac.buses.size() == 1);
  REQUIRE(canonical.ac.external_grids.size() == 1);
  REQUIRE(canonical.ac.generators.size() == 1);
  REQUIRE(canonical.ac.loads.size() == 1);
  CHECK(canonical.ac.external_grids[0].bus == 1);
  CHECK(canonical.ac.generators[0].bus == 1);
  CHECK(canonical.ac.loads[0].bus == 1);

  hacdcpf::opf::ACOPFOptions opts;
  opts.allow_fallback = false;
  opts.enable_primal_dual = true;
  opts.use_parity_ipm = true;
  const auto result = hacdcpf::solve_ac_opf(canonical, opts);
  INFO(result.status);
  REQUIRE(result.converged);
  REQUIRE(result.external_grid_p_mw.size() == 1);
  CHECK_THAT(result.external_grid_p_mw[0],
             Catch::Matchers::WithinAbs(50.0, 1e-2));
}
