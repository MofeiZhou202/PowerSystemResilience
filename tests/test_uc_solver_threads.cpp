#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "hacdcpf/time_series/time_series_pf.hpp"
#include "hacdcpf/time_series/annual_production_sim.hpp"
#include "hacdcpf/engine/external_adapters.hpp"

namespace {
hacdcpf::HybridPowerSystem thread_case() {
  using namespace hacdcpf;
  HybridPowerSystem sys;
  ACBus bus; bus.index = 1; bus.bus_type = BusType::SLACK; bus.base_kv = 110;
  sys.ac.buses.push_back(bus);
  Load load; load.index = 1; load.bus = 1; load.p_mw = 30; load.profile_id = 101;
  sys.ac.loads.push_back(load);
  for (int i = 0; i < 2; ++i) {
    Generator gen; gen.index = i + 1; gen.bus = 1; gen.is_slack = i == 0;
    gen.pg_mw = 15; gen.pmin_mw = 0; gen.pmax_mw = 40;
    gen.cost_c0 = 0; gen.cost_c1 = 10 * (i + 1); gen.cost_c2 = 0;
    gen.startup_cost = 0; gen.shutdown_cost = 0;
    gen.min_up_time_hr = 0; gen.min_dn_time_hr = 0;
    gen.ramp_up_mw_min = 100; gen.ramp_dn_mw_min = 100;
    sys.ac.generators.push_back(gen);
  }
  return sys;
}
hacdcpf::TimeSeriesData thread_series() {
  hacdcpf::TimeSeriesData ts; ts.num_steps = 4; ts.step_duration_hr = 1;
  hacdcpf::TimeSeriesProfile profile; profile.id = 101; profile.values = {1, 2, 1, 2};
  ts.profiles.push_back(profile); return ts;
}
}

TEST_CASE("UC thread limits preserve dispatch and reject unsupported controls", "[uc_threads]") {
  using namespace hacdcpf;
  const auto sys = thread_case(); const auto ts = thread_series();
  TimeSeriesPFOptions opts; opts.uc_solver = UCSolverChoice::Native;
  opts.enable_external_grid = false; opts.run_opf = false;
  // Merit-order oracle: 2*(30*10 + 40*10 + 20*20) = 2200.
  // Only concurrency changes; the feasible set and objective are identical.
  for (int threads : {1, 2}) {
    opts.uc_solver_threads = threads;
    const auto result = solve_unit_commitment(sys, ts, opts);
    INFO(result.solver_status);
    REQUIRE(result.feasible);
    CHECK(result.solver_threads_configured == threads);
    CHECK(result.total_cost == Catch::Approx(2200).margin(1e-5));
    for (int t = 0; t < 4; ++t) {
      CHECK(result.gen_dispatch[0][t] + result.gen_dispatch[1][t] ==
            Catch::Approx(t % 2 ? 60 : 30).margin(1e-5));
    }
  }
  for (auto solver : {UCSolverChoice::HiGHS, UCSolverChoice::SCIP}) {
    opts.uc_solver = solver;
    CHECK_THROWS_AS(solve_unit_commitment(sys, ts, opts), std::invalid_argument);
  }
  opts.uc_solver = UCSolverChoice::Native;
  for (int threads : {-1, 257}) {
    opts.uc_solver_threads = threads;
    CHECK_THROWS_AS(solve_unit_commitment(sys, ts, opts), std::invalid_argument);
  }
}

TEST_CASE("Auto and Gurobi keep explicit UC thread limits through fallback", "[uc_threads]") {
  using namespace hacdcpf;
  for (auto solver : {UCSolverChoice::Auto, UCSolverChoice::Gurobi}) {
    TimeSeriesPFOptions opts; opts.uc_solver = solver; opts.uc_solver_threads = 2;
    const auto result = solve_unit_commitment(thread_case(), thread_series(), opts);
    INFO(result.solver_name << ": " << result.solver_status);
    REQUIRE(result.feasible);
    CHECK(result.solver_threads_configured == 2);
    CHECK(result.total_cost == Catch::Approx(2200).margin(1e-5));
  }
  engine::GurobiOptions invalid_low; invalid_low.threads = -1;
  engine::GurobiOptions invalid_high; invalid_high.threads = 1025;
  CHECK_THROWS_AS(engine::GurobiAdapter(invalid_low), std::invalid_argument);
  CHECK_THROWS_AS(engine::GurobiAdapter(invalid_high), std::invalid_argument);
}

TEST_CASE("CPLEX UC selection reports its actual backend and any fallback", "[uc_solver][cplex]") {
  using namespace hacdcpf;
  engine::CplexAdapter cplex;
  TimeSeriesPFOptions opts;
  opts.uc_solver = UCSolverChoice::CPLEX;
  opts.uc_solver_threads = cplex.available() ? 1 : 0;
  opts.enable_external_grid = false;
  opts.run_opf = false;
  const auto result = solve_unit_commitment(thread_case(), thread_series(), opts);
  INFO(result.solver_name << ": " << result.solver_status);
  REQUIRE(result.feasible);
  CHECK(result.requested_solver == "cplex");
  CHECK(result.total_cost == Catch::Approx(2200).margin(1e-5));
  if (cplex.available()) {
    CHECK(result.solver_name == "CPLEX");
    CHECK_FALSE(result.solver_fallback_used);
    CHECK(result.solver_threads_configured == 1);
  } else {
    CHECK(result.solver_name != "CPLEX");
    CHECK(result.solver_fallback_used);
    CHECK(result.solver_fallback_reason.find("CPLEX") != std::string::npos);
  }
}

TEST_CASE("Annual coupled SCUC forwards solver threads independently of daily workers", "[uc_threads]") {
  using namespace hacdcpf;
  analysis::AnnualProductionSimOptions opts;
  opts.ts_pf_options.uc_solver = UCSolverChoice::Native;
  opts.ts_pf_options.uc_solver_threads = 2;
  opts.parallel_threads = 7;
  opts.enable_parallel_daily = false;
  opts.skip_replay = true;
  const auto result = analysis::solve_annual_production_simulation(thread_case(), thread_series(), opts);
  CHECK_FALSE(result.parallel_daily_effective);
  CHECK(result.uc_solver_threads_configured == 2);
  CHECK_FALSE(result.uc_solver_name.empty());
}
