#include <algorithm>
#include <cmath>
#include <unordered_map>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/power_flow/distributed_slack_solver.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/io/case_builders.hpp"

namespace {

double sum_factors(const std::vector<double>& factors) {
  double sum = 0.0;
  for (double f : factors) sum += f;
  return sum;
}

}  // namespace

TEST_CASE("Island detection and adaptive solver", "[integration][enhanced][island]") {
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::build_ieee14_acdc();
  hacdcpf::PowerFlowOptions opt;
  opt.max_iter = 50;
  opt.tol = 1e-8;

  hacdcpf::HybridPowerSystem islanded_sys = sys;
  for (auto& br : islanded_sys.ac.branches) {
    if (br.from_bus == 14 || br.to_bus == 14) br.in_service = false;
  }

  const auto islands = hacdcpf::powerflow::detect_islands(islanded_sys);
  REQUIRE(islands.size() >= 2);

  const auto it14 = std::find_if(islands.begin(), islands.end(), [](const hacdcpf::IslandInfo& isl) {
    return std::find(isl.ac_buses.begin(), isl.ac_buses.end(), 14) != isl.ac_buses.end();
  });
  REQUIRE(it14 != islands.end());

  const hacdcpf::HybridPowerSystem sub =
      hacdcpf::powerflow::extract_island_subsystem(islanded_sys, *it14, 14);
  REQUIRE(sub.ac.buses.size() == 1);
  REQUIRE(sub.ac.branches.empty());

  SECTION("Adaptive solver") {
    const hacdcpf::AdaptiveSolveResult adaptive = hacdcpf::solve_power_flow_adaptive(islanded_sys, opt);
    REQUIRE(adaptive.vm.size() == 14);
    REQUIRE(adaptive.va.size() == 14);
    REQUIRE(adaptive.vdc.size() == 2);
    REQUIRE(adaptive.islands.size() == islands.size());
    REQUIRE(std::abs(adaptive.vm[13]) <= 1e-12);
  }

  SECTION("Islanded solver") {
    const hacdcpf::IslandedSolveResult islanded = hacdcpf::solve_power_flow_islanded(islanded_sys, opt);
    REQUIRE(islanded.vm.size() == 14);
    REQUIRE(islanded.islands.size() == islands.size());
    REQUIRE(std::abs(islanded.vm[13]) <= 1e-12);
  }
}

TEST_CASE("Distributed slack participation factors", "[integration][enhanced][distributed_slack]") {
  hacdcpf::HybridPowerSystem sys = hacdcpf::io::build_ieee14_acdc();
  hacdcpf::PowerFlowOptions opt;
  opt.max_iter = 50;
  opt.tol = 1e-8;

  SECTION("Capacity-based") {
    const hacdcpf::DistributedSlack cap_cfg =
        hacdcpf::powerflow::create_participation_factors(sys, "capacity");
    REQUIRE_FALSE(cap_cfg.participating_buses.empty());
    REQUIRE(std::abs(sum_factors(cap_cfg.participation_factors) - 1.0) <= 1e-12);
  }

  SECTION("Equal factors") {
    const hacdcpf::DistributedSlack eq_cfg =
        hacdcpf::powerflow::create_participation_factors(sys, "equal", {1, 2, 6});
    REQUIRE(eq_cfg.participation_factors.size() == 3);
    for (double f : eq_cfg.participation_factors) {
      REQUIRE(std::abs(f - (1.0 / 3.0)) <= 1e-12);
    }
  }

  SECTION("Droop factors ordering") {
    const std::unordered_map<int, double> droop = {{1, 0.05}, {2, 0.10}, {6, 0.20}};
    const hacdcpf::DistributedSlack droop_cfg =
        hacdcpf::powerflow::create_participation_factors(sys, "droop", {1, 2, 6}, droop);
    REQUIRE(droop_cfg.participation_factors[0] > droop_cfg.participation_factors[1]);
    REQUIRE(droop_cfg.participation_factors[1] > droop_cfg.participation_factors[2]);
  }

  SECTION("Distributed slack solvers converge") {
    const hacdcpf::DistributedSlack cap_cfg =
        hacdcpf::powerflow::create_participation_factors(sys, "capacity");
    const hacdcpf::DistributedSlackResult dist_simple =
        hacdcpf::solve_power_flow_distributed_slack(sys, cap_cfg, opt);
    const hacdcpf::DistributedSlackResult dist_full =
        hacdcpf::solve_power_flow_distributed_slack_full(sys, cap_cfg, opt);
    REQUIRE(dist_simple.converged);
    REQUIRE(dist_full.converged);
    REQUIRE_FALSE(dist_simple.distributed_slack_p.empty());
    REQUIRE_FALSE(dist_full.distributed_slack_p.empty());
  }
}
