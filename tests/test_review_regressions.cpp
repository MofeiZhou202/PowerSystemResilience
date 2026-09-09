#include <algorithm>
#include <atomic>
#include <cmath>
#include <future>
#include <limits>
#include <stdexcept>
#include <thread>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/carbon_analysis/annual_carbon_analysis.hpp"
#include "hacdcpf/graph/result_recovery.hpp"
#include "hacdcpf/power_flow/distributed_slack_solver.hpp"
#include "hacdcpf/util/atomic_flag_lease.hpp"

using namespace hacdcpf;
using Catch::Approx;

namespace {
HybridPowerSystem source_load_case() {
  HybridPowerSystem sys;
  sys.base_mva = sys.ac.base_mva = 100.0;
  ACBus bus;
  bus.index = 1;
  bus.bus_type = BusType::SLACK;
  bus.vm_pu = 1.0;
  sys.ac.buses.push_back(bus);
  Generator gen;
  gen.index = gen.bus = 1;
  gen.is_slack = true;
  gen.pg_mw = 10.0;
  gen.pmax_mw = 100.0;
  gen.emission_factor_tco2_mwh = 0.8;
  sys.ac.generators.push_back(gen);
  Load load;
  load.index = load.bus = 1;
  load.p_mw = 10.0;
  sys.ac.loads.push_back(load);
  return sys;
}

analysis::AnnualCarbonAnalysisResult annual_fixture() {
  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};
  analysis::AnnualCarbonAnalysisOptions options;
  options.keep_hourly_bus_intensity = true;
  options.keep_hourly_load_emissions = true;
  options.keep_hourly_load_energy = true;
  return analysis::compute_annual_carbon_analysis(source_load_case(), {pf, pf}, 1.0, options);
}
}

TEST_CASE("Review: computation leases preserve another request through exceptions",
          "[review][session][concurrency]") {
  std::atomic<bool> busy{false};
  std::promise<void> acquired, finish;
  auto release_worker = finish.get_future();
  std::thread worker([&] {
    util::AtomicFlagLease lease(busy);
    const bool ok = lease.try_acquire();
    acquired.set_value();
    if (ok) release_worker.wait();
  });
  acquired.get_future().wait();
  CHECK(busy.load());
  {
    util::AtomicFlagLease rejected(busy);
    CHECK_FALSE(rejected.try_acquire());
    rejected.release();
    CHECK(busy.load());
  }
  CHECK_THROWS_AS([&] {
    util::AtomicFlagLease malformed(busy);
    throw std::invalid_argument("invalid request before acquisition");
  }(), std::invalid_argument);
  CHECK(busy.load());
  finish.set_value();
  worker.join();
  CHECK_FALSE(busy.load());
  CHECK_THROWS_AS([&] {
    util::AtomicFlagLease owner(busy);
    CHECK(owner.try_acquire());
    throw std::runtime_error("failure after acquisition");
  }(), std::runtime_error);
  CHECK_FALSE(busy.load());
  util::AtomicFlagLease first(busy);
  CHECK(first.try_acquire());
  first.release();
  util::AtomicFlagLease next(busy);
  CHECK(next.try_acquire());
  first.release();
  CHECK(busy.load());
}

TEST_CASE("Review: distributed slack uses stable AC bus identities",
          "[review][distributed_slack]") {
  auto sys = source_load_case();
  sys.ac.loads.clear();
  for (int i = 2; i <= 3; ++i) {
    ACBus bus;
    bus.index = i;
    bus.bus_type = i == 2 ? BusType::PV : BusType::PQ;
    bus.pd_mw = i == 3 ? 5.0 : 0.0;
    sys.ac.buses.push_back(bus);
    ACBranch branch;
    branch.index = i - 1;
    branch.from_bus = i - 1;
    branch.to_bus = i;
    branch.r_pu = 0.01;
    branch.x_pu = 0.05;
    sys.ac.branches.push_back(branch);
  }
  sys.ac.generators[0].pg_mw = 3.0;
  auto gen = sys.ac.generators[0];
  gen.index = gen.bus = 2;
  gen.is_slack = false;
  gen.pg_mw = 2.0;
  sys.ac.generators.push_back(gen);
  const auto original = powerflow::create_participation_factors(sys);
  CHECK(original.participating_buses == std::vector<int>{1, 2});
  std::reverse(sys.ac.buses.begin(), sys.ac.buses.end());
  for (auto& bus : sys.ac.buses) bus.index *= 10;
  for (auto& g : sys.ac.generators) g.bus *= 10;
  for (auto& b : sys.ac.branches) { b.from_bus *= 10; b.to_bus *= 10; }
  for (const auto& mode : {"capacity", "equal", "droop"}) {
    const auto cfg = powerflow::create_participation_factors(sys, mode, {10, 20}, {{10, 2.0}, {20, 4.0}});
    CHECK(cfg.participating_buses == std::vector<int>{10, 20});
    CHECK(cfg.participation_factors[0] + cfg.participation_factors[1] == Approx(1.0));
    PowerFlowOptions options;
    powerflow::DistributedSlackSolver solver;
    CHECK(solver.solve_full_jacobian(sys, cfg, options).converged);
  }
  auto cfg = powerflow::create_participation_factors(sys);
  std::sort(cfg.participating_buses.begin(), cfg.participating_buses.end());
  CHECK(cfg.participating_buses == std::vector<int>{10, 20});
  sys.ac.buses.back().in_service = false;
  CHECK(powerflow::create_participation_factors(sys).participating_buses == std::vector<int>{20});
}

TEST_CASE("Review: missing domain voltages stay unavailable after recovery",
          "[review][graph][recovery]") {
  graph::ContractionResult contraction;
  contraction.ac_super_to_buses[1] = {1, 3};
  contraction.ac_bus_to_super = {{1, 1}, {3, 1}};
  contraction.dc_super_to_buses[1] = {1, 2};
  contraction.dc_bus_to_super = {{1, 1}, {2, 1}};
  SECTION("only AC known") {
    graph::FullNetworkVoltages values;
    values.ac_bus_voltage[1] = {1.0, 0.2};
    graph::recover_switch_contracted_buses(values, contraction);
    CHECK(values.dc_bus_voltage.empty());
    CHECK(values.ac_bus_voltage.at(3) == std::complex<double>(1.0, 0.2));
  }
  SECTION("only DC known, including legacy alias") {
    graph::FullNetworkVoltages values;
    values.dc_bus_voltage[1] = values.bus_voltage[1] = {0.95, 0.0};
    graph::recover_switch_contracted_buses(values, contraction);
    CHECK(values.ac_bus_voltage.empty());
    CHECK(values.dc_bus_voltage.at(2) == std::complex<double>(0.95, 0.0));
  }
  SECTION("legacy only collision is ambiguous") {
    graph::FullNetworkVoltages values;
    values.bus_voltage[1] = {1.0, 0.0};
    graph::recover_switch_contracted_buses(values, contraction);
    CHECK(values.ac_bus_voltage.empty());
    CHECK(values.dc_bus_voltage.empty());
  }
  SECTION("legacy disjoint domain remains supported") {
    contraction.ac_super_to_buses.clear();
    contraction.ac_bus_to_super.clear();
    graph::FullNetworkVoltages values;
    values.bus_voltage[1] = {0.95, 0.0};
    graph::recover_switch_contracted_buses(values, contraction);
    CHECK(values.dc_bus_voltage.at(2) == std::complex<double>(0.95, 0.0));
  }
}

TEST_CASE("Review: GEC accounting rejects incomplete annual evidence",
          "[review][carbon][gec]") {
  using namespace analysis;
  auto annual = annual_fixture();
  REQUIRE(annual.num_carbon_verified == 2);
  const AnnualUserGECInput user{7, {{1, false}}, 10.0, 0.0};
  const AnnualNodeGECInput node{1, false, 10.0, 0.0};
  const auto good = compute_annual_user_gec_accounting(annual, {user}, true);
  CHECK(good.user_stats[0].energy_mwh == Approx(20.0));
  CHECK(good.user_stats[0].net_emissions_tco2 == Approx(8.0));
  CHECK(validate_annual_user_gec_result(good));
  CHECK(validate_annual_node_gec_result(compute_annual_node_gec_accounting(annual, {node}, true)));
  SECTION("failed power flow") { annual.step_results[1].pf_converged = false; }
  SECTION("failed carbon verification") { annual.step_results[1].carbon_verified = false; }
  SECTION("unknown emissions with known energy") { annual.hourly_load_emissions_tco2[1][0] = NAN; }
  SECTION("unknown energy") { annual.hourly_load_energy_mwh[1][0] = NAN; }
  SECTION("partial row") { annual.hourly_load_energy_mwh[1].clear(); }
  SECTION("empty horizon") { annual.num_steps = 0; }
  SECTION("one missing load among several") {
    annual.load_stats.push_back({2, 1, false});
    for (auto& row : annual.hourly_load_energy_mwh) row.push_back(NAN);
    for (auto& row : annual.hourly_load_emissions_tco2) row.push_back(NAN);
  }
  CHECK_THROWS_AS(compute_annual_user_gec_accounting(annual, {user}, true), std::invalid_argument);
  CHECK_THROWS_AS(compute_annual_node_gec_accounting(annual, {node}, true), std::invalid_argument);
}

TEST_CASE("Review: GEC validators reject all nonfinite metric fields",
          "[review][carbon][gec]") {
  using namespace analysis;
  const auto annual = annual_fixture();
  for (bool hourly : {false, true}) {
    auto user = compute_annual_user_gec_accounting(annual, {{7, {{1, false}}, 10.0, 0.0}}, hourly);
    auto node = compute_annual_node_gec_accounting(annual, {{1, false, 10.0, 0.0}}, hourly);
    for (double bad : {NAN, INFINITY, -INFINITY}) {
      const auto check = [&](auto validate, auto& stats) {
        REQUIRE(validate());
        for (double* field : {&stats.energy_mwh, &stats.gross_emissions_tco2,
             &stats.allocated_gec_mwh, &stats.unused_gec_mwh, &stats.avoided_emissions_tco2,
             &stats.net_emissions_tco2, &stats.gross_intensity_tco2_mwh,
             &stats.net_intensity_tco2_mwh, &stats.gec_coverage_ratio}) {
          const double original = *field;
          *field = bad;
          CHECK_FALSE(validate());
          *field = original;
        }
      };
      check([&] { return validate_annual_user_gec_result(user); }, user.user_stats[0]);
      check([&] { return validate_annual_node_gec_result(node); }, node.node_stats[0]);
      if (hourly) {
        const double user_original = user.hourly_user_results[0][0].net_intensity_tco2_mwh;
        const double node_original = node.hourly_node_results[0][0].net_intensity_tco2_mwh;
        user.hourly_user_results[0][0].net_intensity_tco2_mwh = bad;
        node.hourly_node_results[0][0].net_intensity_tco2_mwh = bad;
        CHECK_FALSE(validate_annual_user_gec_result(user));
        CHECK_FALSE(validate_annual_node_gec_result(node));
        user.hourly_user_results[0][0].net_intensity_tco2_mwh = user_original;
        node.hourly_node_results[0][0].net_intensity_tco2_mwh = node_original;
      }
    }
  }
}

TEST_CASE("Review: hourly carbon padding retains late columns and failed rows",
          "[review][carbon][padding]") {
  auto first = source_load_case();
  auto second = first;
  second.ac.loads[0].index = 99;
  second.ac.loads[0].p_mw = 20.0;
  second.ac.generators[0].pg_mw = 20.0;
  PowerFlowResult good;
  good.converged = true; good.vm = {1.0}; good.va = {0.0};
  PowerFlowResult failed;
  failed.converged = false;
  analysis::AnnualCarbonAnalysisOptions opts;
  opts.keep_hourly_bus_intensity = opts.keep_hourly_load_energy = opts.keep_hourly_load_emissions = true;
  const auto annual = analysis::compute_annual_carbon_analysis(
      std::vector<HybridPowerSystem>{first, first, second}, {good, failed, good}, 1.0, opts);
  REQUIRE(annual.load_stats.size() == 2);
  CHECK(annual.load_stats[0].load_index == 1);
  CHECK(annual.load_stats[1].load_index == 99);
  for (const auto& table : {annual.hourly_load_energy_mwh, annual.hourly_load_emissions_tco2}) {
    REQUIRE(table.size() == 3);
    for (const auto& row : table) REQUIRE(row.size() == 2);
    CHECK(std::isnan(table[0][1]));
    CHECK(std::isnan(table[1][0]));
    CHECK(std::isnan(table[1][1]));
    CHECK(std::isnan(table[2][0]));
  }
  CHECK(annual.hourly_load_energy_mwh[0][0] == Approx(10.0));
  CHECK(annual.hourly_load_energy_mwh[2][1] == Approx(20.0));
  CHECK(annual.total_load_emissions_tco2 == Approx(24.0));
}
