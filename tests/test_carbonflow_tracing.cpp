#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

namespace {

using Approx = Catch::Approx;

hacdcpf::ACBus make_bus(int index, hacdcpf::BusType type) {
  hacdcpf::ACBus bus;
  bus.index = index;
  bus.bus_type = type;
  bus.in_service = true;
  return bus;
}

hacdcpf::Generator make_generator(int index,
                                  int bus,
                                  double p_mw,
                                  double emission_factor,
                                  bool is_slack = false) {
  hacdcpf::Generator gen;
  gen.index = index;
  gen.bus = bus;
  gen.in_service = true;
  gen.is_slack = is_slack;
  gen.pg_mw = p_mw;
  gen.pmin_mw = 0.0;
  gen.pmax_mw = 200.0;
  gen.emission_factor_tco2_mwh = emission_factor;
  return gen;
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

}  // namespace

TEST_CASE("Carbon tracing allocates a known two-source load proportionally",
          "[carbonflow][tracing][analytic]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PV),
      make_bus(3, BusType::PQ),
  };
  sys.ac.generators = {
      make_generator(1, 1, 30.0, 1.0),
      make_generator(2, 2, 70.0, 0.0),
  };
  sys.ac.loads = {make_load(1, 3, 100.0)};
  sys.ac.branches = {
      make_branch(1, 1, 3),
      make_branch(2, 2, 3),
  };

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0};
  pf.branch_flows = {
      BranchFlow{30.0, 0.0, -30.0, 0.0},
      BranchFlow{70.0, 0.0, -70.0, 0.0},
  };

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 1);

  const LoadCarbonResult& load = ca.load_carbon[0];
  CHECK(load.demand_mw == Approx(100.0));
  CHECK(load.generator_supply_mw.size() == 2);
  CHECK(load.generator_supply_mw.at(0) == Approx(30.0));
  CHECK(load.generator_supply_mw.at(1) == Approx(70.0));
  CHECK(load.carbon_intensity_tco2_mwh == Approx(0.3));
  CHECK(load.total_emissions_tco2 == Approx(30.0));

  REQUIRE(ca.bus_carbon.size() == 3);
  CHECK(ca.bus_carbon[0].carbon_intensity_tco2_mwh == Approx(1.0));
  CHECK(ca.bus_carbon[1].carbon_intensity_tco2_mwh == Approx(0.0));
  CHECK(ca.bus_carbon[2].carbon_intensity_tco2_mwh == Approx(0.3));
}

TEST_CASE("Carbon tracing assigns branch loss to the upstream source",
          "[carbonflow][tracing][analytic]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PQ),
  };
  sys.ac.generators = {make_generator(1, 1, 110.0, 0.5)};
  sys.ac.loads = {make_load(1, 2, 100.0)};
  sys.ac.branches = {make_branch(1, 1, 2)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {BranchFlow{110.0, 0.0, -100.0, 0.0}};

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 1);
  REQUIRE(ca.branch_carbon.size() == 1);

  CHECK(ca.load_carbon[0].generator_supply_mw.at(0) == Approx(100.0));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.load_carbon[0].total_emissions_tco2 == Approx(50.0));

  CHECK(ca.branch_carbon[0].loss_mw == Approx(10.0));
  CHECK(ca.branch_carbon[0].generator_loss_mw.at(0) == Approx(10.0));
  CHECK(ca.branch_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.branch_carbon[0].total_emissions_tco2 == Approx(5.0));

  CHECK(ca.tracing_summary.total_generation_emissions_tco2 == Approx(55.0));
  CHECK(ca.tracing_summary.total_load_emissions_tco2 == Approx(50.0));
  CHECK(ca.tracing_summary.total_loss_emissions_tco2 == Approx(5.0));
  CHECK(ca.tracing_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}
