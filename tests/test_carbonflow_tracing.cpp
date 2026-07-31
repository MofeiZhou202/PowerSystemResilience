#include "hacdcpf/carbon_analysis/carbon_analysis.hpp"
#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/power_flow/pv_power_curve.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

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
  CHECK(ca.matrix_rank == 3);
  CHECK(std::isfinite(ca.matrix_condition_estimate));
  CHECK(ca.matrix_condition_estimate >= 1.0);
  CHECK(ca.matrix_relative_residual < 1e-12);
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

TEST_CASE("Carbon matrix rejects a source-free circulating-flow singularity",
          "[carbonflow][matrix][singular][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PQ),
      make_bus(2, BusType::PQ),
  };
  sys.ac.branches = {
      make_branch(1, 1, 2),
      make_branch(2, 2, 1),
  };

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {
      BranchFlow{1.0, 0.0, -1.0, 0.0},
      BranchFlow{1.0, 0.0, -1.0, 0.0},
  };

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
  REQUIRE(ca.power_balance_verified);
  CHECK_FALSE(ca.matrix_solved);
  CHECK_FALSE(ca.tracing_verified);
  CHECK(ca.matrix_rank == 1);
  CHECK(ca.matrix_condition_estimate ==
        std::numeric_limits<double>::max());
}

TEST_CASE("Matrix carbon flow balances a lossy two-source merge analytically",
          "[carbonflow][matrix][loss][analytic]") {
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
      make_generator(1, 1, 60.0, 0.9),
      make_generator(2, 2, 50.0, 0.0),
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
      BranchFlow{60.0, 0.0, -50.0, 0.0},
      BranchFlow{50.0, 0.0, -50.0, 0.0},
  };

  CarbonAnalysisOptions opt;
  opt.loss_allocation_alpha = 0.5;
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf, opt);

  const double expected_load_intensity = (55.0 * 0.9) / 105.0;
  const double expected_loss_intensity =
      0.5 * (0.9 + expected_load_intensity);

  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 1);
  REQUIRE(ca.branch_carbon.size() == 2);

  CHECK(ca.load_carbon[0].generator_supply_mw.at(0) == Approx(50.0));
  CHECK(ca.load_carbon[0].generator_supply_mw.at(1) == Approx(50.0));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh ==
        Approx(expected_load_intensity).margin(1e-12));
  CHECK(ca.branch_carbon[0].loss_mw == Approx(10.0));
  CHECK(ca.branch_carbon[0].carbon_intensity_tco2_mwh ==
        Approx(expected_loss_intensity).margin(1e-12));
  CHECK(ca.branch_carbon[1].loss_mw == Approx(0.0));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 ==
        Approx(60.0 * 0.9));
  CHECK(ca.matrix_summary.total_load_emissions_tco2 ==
        Approx(100.0 * expected_load_intensity).margin(1e-12));
  CHECK(ca.matrix_summary.total_loss_emissions_tco2 ==
        Approx(10.0 * expected_loss_intensity).margin(1e-12));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
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

TEST_CASE("External grid emission factor is used for slack supply carbon",
          "[carbonflow][external-grid][tracing]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK),
      make_bus(2, BusType::PQ),
  };

  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.in_service = true;
  grid.emission_factor_tco2_mwh = 0.9;
  sys.ac.external_grids = {grid};
  sys.ac.loads = {make_load(1, 2, 100.0)};
  sys.ac.branches = {make_branch(1, 1, 2)};

  const HybridPowerSystem restored =
      hacdcpf::io::from_json(hacdcpf::io::to_json(sys));
  REQUIRE(restored.ac.external_grids.size() == 1);
  CHECK(restored.ac.external_grids[0].emission_factor_tco2_mwh ==
        Approx(0.9));

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {BranchFlow{110.0, 0.0, -100.0, 0.0}};

  const CarbonAnalysisResult ca = compute_carbon_analysis(restored, pf);

  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 1);
  REQUIRE(ca.branch_carbon.size() == 1);

  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.9));
  CHECK(ca.load_carbon[0].total_emissions_tco2 == Approx(90.0));
  CHECK(ca.branch_carbon[0].total_emissions_tco2 == Approx(9.0));
  CHECK(ca.tracing_summary.total_generation_emissions_tco2 == Approx(99.0));
  CHECK(ca.tracing_summary.total_load_emissions_tco2 == Approx(90.0));
  CHECK(ca.tracing_summary.total_loss_emissions_tco2 == Approx(9.0));
  CHECK(ca.tracing_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Matrix nodal intensity is not amplified by branch loss responsibility",
          "[carbonflow][matrix][loss]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PV),
  };
  sys.ac.generators = {
      make_generator(1, 1, 20.0, 1.0),
      make_generator(2, 2, 100.0, 0.0),
  };
  sys.ac.loads = {make_load(1, 1, 110.0)};
  sys.ac.branches = {make_branch(1, 1, 2)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {BranchFlow{-90.0, 0.0, 100.0, 0.0}};

  CarbonAnalysisOptions opt;
  opt.loss_allocation_alpha = 0.5;
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf, opt);

  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.branch_carbon.size() == 1);

  CHECK(ca.bus_carbon[0].carbon_intensity_tco2_mwh == Approx(20.0 / 115.0));
  CHECK(ca.bus_carbon[1].carbon_intensity_tco2_mwh == Approx(0.0));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh <= 1.0);
  CHECK(ca.branch_carbon[0].loss_mw == Approx(10.0));
  CHECK(ca.branch_carbon[0].carbon_intensity_tco2_mwh ==
        Approx(0.5 * (20.0 / 115.0)).margin(1e-12));
  CHECK(ca.branch_carbon[0].total_emissions_tco2 ==
        Approx(10.0 * 0.5 * (20.0 / 115.0)).margin(1e-12));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(20.0));
  CHECK(ca.matrix_summary.total_load_emissions_tco2 ==
        Approx(110.0 * (20.0 / 115.0)));
  CHECK(ca.matrix_summary.total_loss_emissions_tco2 ==
        Approx(10.0 * 0.5 * (20.0 / 115.0)).margin(1e-12));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Matrix loss allocation alpha changes branch loss carbon intensity",
          "[carbonflow][matrix][loss]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PV),
  };
  sys.ac.generators = {
      make_generator(1, 1, 20.0, 1.0),
      make_generator(2, 2, 100.0, 0.0),
  };
  sys.ac.loads = {make_load(1, 1, 110.0)};
  sys.ac.branches = {make_branch(1, 1, 2)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {BranchFlow{-90.0, 0.0, 100.0, 0.0}};

  CarbonAnalysisOptions sender_only;
  sender_only.loss_allocation_alpha = 1.0;
  const CarbonAnalysisResult ca_sender = compute_carbon_analysis(sys, pf, sender_only);
  REQUIRE(ca_sender.matrix_solved);
  REQUIRE(ca_sender.branch_carbon.size() == 1);
  CHECK(ca_sender.branch_carbon[0].carbon_intensity_tco2_mwh ==
        Approx(0.0).margin(1e-12));
  CHECK(ca_sender.matrix_summary.balance_error_pct ==
        Approx(0.0).margin(1e-10));

  CarbonAnalysisOptions split;
  split.loss_allocation_alpha = 0.5;
  const CarbonAnalysisResult ca_split = compute_carbon_analysis(sys, pf, split);
  REQUIRE(ca_split.matrix_solved);
  REQUIRE(ca_split.branch_carbon.size() == 1);
  CHECK(ca_split.branch_carbon[0].carbon_intensity_tco2_mwh >
        ca_sender.branch_carbon[0].carbon_intensity_tco2_mwh);
  CHECK(ca_split.branch_carbon[0].carbon_intensity_tco2_mwh ==
        Approx(0.5 * ca_split.bus_carbon[0].carbon_intensity_tco2_mwh)
            .margin(1e-12));
  CHECK(ca_split.matrix_summary.balance_error_pct ==
        Approx(0.0).margin(1e-10));
}

TEST_CASE("Matrix summary includes bus shunt active demand",
          "[carbonflow][matrix][load]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::PV)};
  sys.ac.buses[0].gs_mw = 5.0;
  sys.ac.generators = {make_generator(1, 1, 5.0, 0.8)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.matrix_solved);
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(4.0));
  CHECK(ca.matrix_summary.total_load_emissions_tco2 == Approx(0.0));
  CHECK(ca.matrix_summary.total_loss_emissions_tco2 == Approx(4.0));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Closed AC switch flow participates in carbon matrix tracing",
          "[carbonflow][matrix][switch]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PQ),
      make_bus(3, BusType::PQ),
  };
  sys.ac.generators = {make_generator(1, 1, 1.0, 0.5)};
  sys.ac.loads = {
      make_load(1, 2, 0.4),
      make_load(2, 3, 0.6),
  };

  sys.ac.branches = {
      make_branch(1, 1, 2),
      make_branch(2, 2, 3),
  };

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0};
  pf.ac_switch_flows = {DeviceTerminalFlow{1, 1, 2, true, 1.0, -1.0, 0.0, 0.0}};
  pf.branch_flows = {
      BranchFlow{0.0, 0.0, 0.0, 0.0},
      BranchFlow{0.6, 0.0, -0.6, 0.0},
  };

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 2);
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.load_carbon[1].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.bus_carbon[1].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(0.5));
  CHECK(ca.matrix_summary.total_load_emissions_tco2 == Approx(0.5));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Negative AC load is traced as a zero-emission injection source",
          "[carbonflow][matrix][load][signed-device]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK)};
  sys.ac.generators = {make_generator(1, 1, 0.0, 0.8, true)};
  sys.ac.loads = {
      make_load(1, 1, 10.0),
      make_load(2, 1, -4.0),
  };

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.carbon_sources.size() == 2);
  CHECK(ca.carbon_sources[0].source_id == 0);
  CHECK(ca.carbon_sources[0].source_type == "generator");
  CHECK(ca.carbon_sources[0].power_mw == Approx(6.0));
  CHECK(ca.carbon_sources[1].source_id == 1);
  CHECK(ca.carbon_sources[1].source_type == "negative_load_injection");
  CHECK(ca.carbon_sources[1].component_index == 2);
  CHECK(ca.carbon_sources[1].power_mw == Approx(4.0));
  REQUIRE(ca.load_carbon.size() == 1);
  CHECK(ca.load_carbon[0].demand_mw == Approx(10.0));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.48));
  CHECK(ca.load_carbon[0].total_emissions_tco2 == Approx(4.8));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(4.8));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Negative DC load is traced as a zero-emission injection source",
          "[carbonflow][matrix][dc][load][signed-device]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.dc.base_mva = 100.0;
  DCBus bus;
  bus.index = 1;
  bus.in_service = true;
  bus.bus_type = DCBusType::DC_V;
  bus.emission_factor_tco2_mwh = 0.5;
  sys.dc.buses = {bus};

  DCLoad demand;
  demand.index = 1;
  demand.bus = 1;
  demand.in_service = true;
  demand.p_mw = 1.0;
  DCLoad injection = demand;
  injection.index = 2;
  injection.p_mw = -0.4;
  sys.dc.loads = {demand, injection};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vdc = {1.0};
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.carbon_sources.size() == 2);
  CHECK(ca.carbon_sources[0].source_type ==
        "negative_dc_load_injection");
  CHECK(ca.carbon_sources[0].power_mw == Approx(0.4));
  CHECK(ca.carbon_sources[1].source_type == "dc_voltage_boundary");
  CHECK(ca.carbon_sources[1].power_mw == Approx(0.6));
  REQUIRE(ca.dc_load_carbon.size() == 1);
  CHECK(ca.dc_load_carbon[0].demand_mw == Approx(1.0));
  CHECK(ca.dc_load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.3));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(0.3));
  CHECK(ca.matrix_summary.total_load_emissions_tco2 == Approx(0.3));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Carbon flow maps reordered non-contiguous DC bus IDs by identity",
          "[carbonflow][matrix][dc][mapping][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 1.0;
  sys.dc.base_mva = 1.0;

  DCBus source_bus;
  source_bus.index = 20;
  source_bus.in_service = true;
  source_bus.bus_type = DCBusType::DC_V;
  source_bus.emission_factor_tco2_mwh = 0.5;
  DCBus load_bus;
  load_bus.index = 10;
  load_bus.in_service = true;
  load_bus.bus_type = DCBusType::DC_P;
  sys.dc.buses = {source_bus, load_bus};

  DCBranch branch;
  branch.index = 7;
  branch.from_bus = 20;
  branch.to_bus = 10;
  branch.in_service = true;
  branch.r_pu = 0.01;
  sys.dc.branches = {branch};

  DCLoad load;
  load.index = 3;
  load.bus = 10;
  load.in_service = true;
  load.p_mw = 0.99;
  sys.dc.loads = {load};

  PowerFlowResult pf;
  pf.converged = true;
  // Result vectors follow the bus-vector order [20, 10], not bus_id - 1.
  pf.vdc = {1.0, 0.99};
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.dc_load_carbon.size() == 1);
  REQUIRE(ca.dc_bus_carbon.size() == 2);
  CHECK(ca.dc_load_carbon[0].bus == 10);
  CHECK(ca.dc_load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.dc_bus_carbon[0].bus_index == 20);
  CHECK(ca.dc_bus_carbon[1].bus_index == 10);
  CHECK(ca.dc_bus_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.dc_bus_carbon[1].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(0.5));
  CHECK(ca.matrix_summary.total_load_emissions_tco2 == Approx(0.495));
  CHECK(ca.matrix_summary.total_loss_emissions_tco2 == Approx(0.005));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("DC bus demand and scaled component loads are additive in carbon flow",
          "[carbonflow][matrix][dc][load][integration][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.dc.base_mva = 100.0;

  DCBus source_bus;
  source_bus.index = 1;
  source_bus.in_service = true;
  source_bus.bus_type = DCBusType::DC_V;
  source_bus.vm_pu = 1.0;
  source_bus.emission_factor_tco2_mwh = 0.5;
  DCBus load_bus;
  load_bus.index = 2;
  load_bus.in_service = true;
  load_bus.bus_type = DCBusType::DC_P;
  load_bus.vm_pu = 1.0;
  load_bus.pd_mw = 0.75;
  sys.dc.buses = {source_bus, load_bus};

  DCBranch branch;
  branch.index = 1;
  branch.from_bus = 1;
  branch.to_bus = 2;
  branch.in_service = true;
  branch.r_pu = 0.01;
  sys.dc.branches = {branch};

  DCLoad load;
  load.index = 1;
  load.bus = 2;
  load.in_service = true;
  load.p_mw = 1.0;
  load.scaling = 0.5;
  sys.dc.loads = {load};

  const PowerFlowResult pf = solve_power_flow(sys);
  REQUIRE(pf.converged);
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.dc_load_carbon.size() == 2);
  CHECK(ca.dc_load_carbon[0].demand_mw == Approx(0.5));
  CHECK(ca.dc_load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
  CHECK(ca.dc_load_carbon[1].demand_mw == Approx(0.75));
  CHECK(ca.dc_load_carbon[1].carbon_intensity_tco2_mwh == Approx(0.5));
  REQUIRE(ca.carbon_sources.size() == 1);
  CHECK(ca.carbon_sources[0].source_type == "dc_voltage_boundary");
  CHECK(ca.carbon_sources[0].power_mw > 1.25);
  CHECK(ca.carbon_sources[0].power_mw < 1.251);
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("PV carbon source uses the same array-model output as power flow",
          "[carbonflow][pv][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK)};
  sys.ac.generators = {make_generator(1, 1, 0.0, 0.8, true)};
  sys.ac.loads = {make_load(1, 1, 5.0)};

  PVSystem pv;
  pv.index = 1;
  pv.bus = 1;
  pv.in_service = true;
  // Detailed array parameters are authoritative even when the fallback field
  // contains a stale signed value.
  pv.p_mw = -4.9;
  pv.voc = 50.0;
  pv.vmpp = 40.0;
  pv.isc = 10.0;
  pv.num_series = 10;
  pv.num_parallel = 1000;
  pv.irradiance = 1000.0;
  pv.inverter_eff = 1.0;
  sys.ac.pv_systems = {pv};

  const double actual_pv_mw = powerflow::compute_pv_power_mw(pv);
  REQUIRE(actual_pv_mw > 0.0);
  REQUIRE(actual_pv_mw < 5.0);
  REQUIRE(std::abs(actual_pv_mw - pv.p_mw) > 5.0);

  const PowerFlowResult pf = solve_power_flow(sys);
  REQUIRE(pf.converged);
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.carbon_sources.size() == 2);
  CHECK(ca.carbon_sources[0].source_type == "generator");
  CHECK(ca.carbon_sources[0].power_mw == Approx(5.0 - actual_pv_mw));
  CHECK(ca.carbon_sources[1].source_type == "pv_system");
  CHECK(ca.carbon_sources[1].scheduled_power_mw == Approx(actual_pv_mw));
  CHECK(ca.carbon_sources[1].power_mw == Approx(actual_pv_mw));
  REQUIRE(ca.load_carbon.size() == 1);
  const double expected_intensity = (5.0 - actual_pv_mw) * 0.8 / 5.0;
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh ==
        Approx(expected_intensity).margin(1e-10));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 ==
        Approx((5.0 - actual_pv_mw) * 0.8).margin(1e-10));
  CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-10));
}

TEST_CASE("Standalone two-winding transformer participates in carbon flow",
          "[carbonflow][transformer][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK),
      make_bus(2, BusType::PQ),
  };
  sys.ac.buses[0].base_kv = 110.0;
  sys.ac.buses[1].base_kv = 10.0;
  sys.ac.generators = {make_generator(1, 1, 0.0, 0.9, true)};
  sys.ac.loads = {make_load(1, 2, 1.0)};

  Transformer2W transformer;
  transformer.index = 1;
  transformer.hv_bus = 1;
  transformer.lv_bus = 2;
  transformer.in_service = true;
  transformer.sn_mva = 10.0;
  transformer.vn_hv_kv = 110.0;
  transformer.vn_lv_kv = 10.0;
  transformer.vk_percent = 6.0;
  transformer.vkr_percent = 1.0;
  sys.ac.transformers_2w = {transformer};

  PowerFlowOptions pf_opt;
  pf_opt.max_iter = 100;
  pf_opt.tol = 1e-10;
  const PowerFlowResult pf = solve_power_flow(sys, pf_opt);
  REQUIRE(pf.converged);

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.load_carbon.size() == 1);
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.9).margin(1e-8));
  CHECK(ca.matrix_summary.total_generation_emissions_tco2 ==
        Approx(ca.matrix_summary.total_load_emissions_tco2 +
               ca.matrix_summary.total_loss_emissions_tco2)
            .margin(1e-8));
  CHECK(ca.matrix_summary.total_loss_emissions_tco2 > 0.0);
}

TEST_CASE("ZIP load carbon uses solved voltage-dependent active power",
          "[carbonflow][zip][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PQ),
  };
  sys.ac.generators = {make_generator(1, 1, 1.805, 0.9)};
  Load zip = make_load(1, 2, 2.0);
  zip.p_percent_p = 0.0;
  zip.i_percent_p = 0.0;
  zip.z_percent_p = 100.0;
  sys.ac.loads = {zip};
  sys.ac.branches = {make_branch(1, 1, 2)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 0.95};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {BranchFlow{1.805, 0.0, -1.805, 0.0}};

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.load_carbon.size() == 1);
  CHECK(ca.load_carbon[0].demand_mw == Approx(2.0 * 0.95 * 0.95));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.9));
}

TEST_CASE("Flexible load projected into power flow also enters carbon flow",
          "[carbonflow][flexible-load][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK),
      make_bus(2, BusType::PQ),
  };
  sys.ac.generators = {make_generator(1, 1, 0.0, 0.5, true)};
  ACBranch branch = make_branch(1, 1, 2);
  branch.r_pu = 0.01;
  branch.x_pu = 0.03;
  sys.ac.branches = {branch};
  FlexibleLoad load;
  load.index = 7;
  load.bus = 2;
  load.in_service = true;
  load.p_mw = 1.0;
  load.q_mvar = 0.2;
  sys.ac.flexible_loads = {load};

  const PowerFlowResult pf = solve_power_flow(sys);
  REQUIRE(pf.converged);
  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);

  REQUIRE(ca.power_balance_verified);
  REQUIRE(ca.matrix_solved);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.load_carbon.size() == 1);
  CHECK(ca.load_carbon[0].demand_mw == Approx(1.0));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5).margin(1e-8));
}

TEST_CASE("Unmodelled non-boundary export fails carbon verification",
          "[carbonflow][power-balance][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::SLACK),
      make_bus(2, BusType::PQ),
  };
  sys.ac.generators = {make_generator(1, 1, 0.0, 0.9, true)};
  sys.ac.branches = {make_branch(1, 1, 2)};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0};
  pf.va = {0.0, 0.0};
  pf.branch_flows = {BranchFlow{1.0, 0.0, -1.0, 0.0}};

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
  CHECK_FALSE(ca.power_balance_verified);
  CHECK_FALSE(ca.matrix_solved);
  CHECK_FALSE(ca.tracing_verified);
  REQUIRE(ca.node_power_balance_errors.size() == 1);
  CHECK(ca.node_power_balance_errors[0].bus_index == 2);
  CHECK(ca.node_power_balance_errors[0].mismatch_mw == Approx(1.0));
  CHECK(ca.total_external_export_mw == Approx(0.0));
}

TEST_CASE("Matrix source allocation handles split and reconverging paths",
          "[carbonflow][source-allocation][mesh][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {
      make_bus(1, BusType::PV),
      make_bus(2, BusType::PQ),
      make_bus(3, BusType::PQ),
      make_bus(4, BusType::PQ),
      make_bus(5, BusType::PQ),
  };
  sys.ac.generators = {make_generator(1, 1, 1.0, 0.7)};
  sys.ac.loads = {make_load(1, 5, 1.0)};
  sys.ac.branches = {
      make_branch(1, 1, 2),
      make_branch(2, 2, 3),
      make_branch(3, 2, 4),
      make_branch(4, 3, 5),
      make_branch(5, 4, 5),
  };

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0, 1.0, 1.0, 1.0, 1.0};
  pf.va = {0.0, 0.0, 0.0, 0.0, 0.0};
  pf.branch_flows = {
      BranchFlow{1.0, 0.0, -1.0, 0.0},
      BranchFlow{0.5, 0.0, -0.5, 0.0},
      BranchFlow{0.5, 0.0, -0.5, 0.0},
      BranchFlow{0.5, 0.0, -0.5, 0.0},
      BranchFlow{0.5, 0.0, -0.5, 0.0},
  };

  const CarbonAnalysisResult ca = compute_carbon_analysis(sys, pf);
  REQUIRE(ca.tracing_verified);
  REQUIRE(ca.load_carbon.size() == 1);
  REQUIRE(ca.load_carbon[0].generator_supply_mw.size() == 1);
  CHECK(ca.load_carbon[0].generator_supply_mw.at(0) == Approx(1.0));
  CHECK(ca.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.7));
}

TEST_CASE("External grid owns balancing carbon factor at a shared slack bus",
          "[carbonflow][external-grid][slack][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.buses = {make_bus(1, BusType::SLACK)};
  sys.ac.generators = {make_generator(1, 1, 0.0, 0.45, true)};
  sys.ac.loads = {make_load(1, 1, 1.0)};
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.in_service = true;
  grid.emission_factor_tco2_mwh = 0.1;
  sys.ac.external_grids = {grid};

  PowerFlowResult pf;
  pf.converged = true;
  pf.vm = {1.0};
  pf.va = {0.0};

  const CarbonAnalysisResult low = compute_carbon_analysis(sys, pf);
  REQUIRE(low.matrix_solved);
  REQUIRE(low.carbon_sources.size() == 1);
  CHECK(low.carbon_sources[0].source_id == 0);
  CHECK(low.carbon_sources[0].source_type == "external_grid");
  CHECK(low.carbon_sources[0].component_index == 1);
  CHECK(low.carbon_sources[0].power_mw == Approx(1.0));
  CHECK(low.carbon_sources[0].emission_factor_tco2_mwh == Approx(0.1));
  REQUIRE(low.load_carbon.size() == 1);
  CHECK(low.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.1));

  sys.ac.external_grids[0].emission_factor_tco2_mwh = 0.9;
  const CarbonAnalysisResult high = compute_carbon_analysis(sys, pf);
  REQUIRE(high.matrix_solved);
  REQUIRE(high.load_carbon.size() == 1);
  CHECK(high.load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.9));
  CHECK(high.matrix_summary.total_generation_emissions_tco2 == Approx(0.9));
}

TEST_CASE("DC voltage boundary and signed native generator preserve carbon balance",
          "[carbonflow][dc][boundary][signed-device][regression]") {
  using namespace hacdcpf;
  using namespace hacdcpf::analysis;

  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.dc.base_mva = 100.0;

  DCBus bus;
  bus.index = 1;
  bus.bus_type = DCBusType::DC_V;
  bus.in_service = true;
  bus.emission_factor_tco2_mwh = 0.5;
  sys.dc.buses = {bus};

  DCLoad load;
  load.index = 1;
  load.bus = 1;
  load.in_service = true;
  load.p_mw = 1.0;
  sys.dc.loads = {load};

  StaticGeneratorDC native_generator;
  native_generator.index = 7;
  native_generator.bus = 1;
  native_generator.in_service = true;
  native_generator.type = "thermal";
  native_generator.p_set_mw = -0.1;
  native_generator.scaling = 1.0;
  native_generator.emission_factor_tco2_mwh = 0.9;
  sys.dc.dc_static_generators = {native_generator};

  const HybridPowerSystem restored = io::from_json(io::to_json(sys));
  REQUIRE(restored.dc.buses.size() == 1);
  REQUIRE(restored.dc.dc_static_generators.size() == 1);
  CHECK(restored.dc.buses[0].emission_factor_tco2_mwh == Approx(0.5));
  CHECK(restored.dc.dc_static_generators[0].emission_factor_tco2_mwh ==
        Approx(0.9));

  PowerFlowResult pf;
  pf.converged = true;
  pf.vdc = {1.0};

  SECTION("negative native generation is an explicit carbon sink") {
    const CarbonAnalysisResult ca = compute_carbon_analysis(restored, pf);
    REQUIRE(ca.power_balance_verified);
    REQUIRE(ca.matrix_solved);
    REQUIRE(ca.tracing_verified);
    REQUIRE(ca.dc_load_carbon.size() == 2);
    CHECK(ca.dc_load_carbon[0].demand_mw == Approx(1.0));
    CHECK(ca.dc_load_carbon[1].demand_mw == Approx(0.1));
    CHECK(ca.dc_load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.5));
    CHECK(ca.dc_load_carbon[1].carbon_intensity_tco2_mwh == Approx(0.5));
    CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(0.55));
    CHECK(ca.matrix_summary.total_load_emissions_tco2 == Approx(0.55));
    CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-12));
  }

  SECTION("positive native generation uses its configured carbon factor") {
    HybridPowerSystem generating = restored;
    generating.dc.dc_static_generators[0].p_set_mw = 0.4;
    const CarbonAnalysisResult ca = compute_carbon_analysis(generating, pf);
    REQUIRE(ca.power_balance_verified);
    REQUIRE(ca.matrix_solved);
    REQUIRE(ca.tracing_verified);
    REQUIRE(ca.dc_load_carbon.size() == 1);
    CHECK(ca.dc_load_carbon[0].carbon_intensity_tco2_mwh == Approx(0.66));
    CHECK(ca.matrix_summary.total_generation_emissions_tco2 == Approx(0.66));
    CHECK(ca.matrix_summary.total_load_emissions_tco2 == Approx(0.66));
    CHECK(ca.matrix_summary.balance_error_pct == Approx(0.0).margin(1e-12));
  }
}
