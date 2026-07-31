#include <cmath>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/power_flow/fdpf_solver.hpp"
#include "hacdcpf/power_flow/helm_solver.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/sppt/certificate.hpp"
#include "power_flow_solver_test_utils.hpp"

namespace {

hacdcpf::HybridPowerSystem make_two_bus_hybrid_dc_case(int dc_ref_id,
                                                        int dc_load_id) {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.dc.base_mva = 100.0;

  hacdcpf::ACBus ac;
  ac.index = 1;
  ac.bus_type = hacdcpf::BusType::SLACK;
  ac.vm_pu = 1.0;
  sys.ac.buses = {ac};

  hacdcpf::Generator generator;
  generator.index = 1;
  generator.bus = 1;
  generator.is_slack = true;
  generator.pmin_mw = -100.0;
  generator.pmax_mw = 100.0;
  generator.qmin_mvar = -100.0;
  generator.qmax_mvar = 100.0;
  sys.ac.generators = {generator};

  hacdcpf::DCBus dc_ref;
  dc_ref.index = dc_ref_id;
  dc_ref.bus_type = hacdcpf::DCBusType::DC_V;
  dc_ref.vm_pu = 1.0;
  hacdcpf::DCBus dc_load;
  dc_load.index = dc_load_id;
  dc_load.bus_type = hacdcpf::DCBusType::DC_P;
  dc_load.vm_pu = 1.0;
  dc_load.pd_mw = 2.0;
  sys.dc.buses = {dc_ref, dc_load};

  hacdcpf::DCBranch branch;
  branch.index = 7;
  branch.from_bus = dc_ref_id;
  branch.to_bus = dc_load_id;
  branch.r_pu = 0.05;
  sys.dc.branches = {branch};

  hacdcpf::DCLoad load;
  load.index = 9;
  load.bus = dc_load_id;
  load.p_mw = 3.0;
  sys.dc.loads = {load};

  hacdcpf::VSCConverter converter;
  converter.index = 11;
  converter.bus_ac = 1;
  converter.bus_dc = dc_ref_id;
  converter.control_mode = hacdcpf::ConverterMode::VDC_Q;
  converter.v_dc_set_pu = 1.0;
  converter.k_vdc = 20.0;
  converter.pmin_mw = -100.0;
  converter.pmax_mw = 100.0;
  converter.qmin_mvar = -100.0;
  converter.qmax_mvar = 100.0;
  converter.p_rated_mw = 100.0;
  sys.vsc_converters = {converter};
  return sys;
}

}  // namespace

TEST_CASE("HELM matches Newton on a well-conditioned AC case", "[power_flow][helm]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;

  hacdcpf::powerflow::NewtonSolver newton;
  const auto nr = newton.solve(data, options);
  REQUIRE(nr.converged);

  hacdcpf::powerflow::HelmSolver helm;
  helm.helm_opts.max_coef = 120;
  helm.helm_opts.mismatch = 1e-8;
  helm.helm_opts.enforce_q_limits = false;
  const auto result = helm.solve(data, options);
  REQUIRE(result.converged);
  REQUIRE(result.vm.size() == nr.vm.size());
  CHECK(std::abs(result.vm[1] - nr.vm[1]) < 2e-5);
  CHECK(std::abs(result.va[1] - nr.va[1]) < 2e-5);
}

TEST_CASE("Canonical projection normalizes non-contiguous DC IDs without cross-domain aliasing",
          "[power_flow][projection][dc][noncontiguous]") {
  const auto contiguous = make_two_bus_hybrid_dc_case(1, 2);
  const auto noncontiguous = make_two_bus_hybrid_dc_case(10, 20);

  const auto projected = hacdcpf::project_to_canonical_models(noncontiguous);
  REQUIRE(projected.dc.buses.size() == 2);
  CHECK(projected.dc.buses[0].index == 1);
  CHECK(projected.dc.buses[1].index == 2);
  CHECK(projected.dc.branches[0].from_bus == 1);
  CHECK(projected.dc.branches[0].to_bus == 2);
  CHECK(projected.dc.loads[0].bus == 2);
  CHECK(projected.vsc_converters[0].bus_ac == 1);
  CHECK(projected.vsc_converters[0].bus_dc == 1);

  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  const auto a = hacdcpf::solve_power_flow(contiguous, options);
  const auto b = hacdcpf::solve_power_flow(noncontiguous, options);
  REQUIRE(a.converged);
  REQUIRE(b.converged);
  REQUIRE(a.vdc.size() == b.vdc.size());
  CHECK(a.vdc[0] == Catch::Approx(b.vdc[0]).margin(1e-10));
  CHECK(a.vdc[1] == Catch::Approx(b.vdc[1]).margin(1e-10));
  REQUIRE(b.vsc_transfers.size() == 1);
  CHECK(b.vsc_transfers[0].bus_ac == 1);
  CHECK(b.vsc_transfers[0].bus_dc == 10);
}

TEST_CASE("DC bus demand and explicit DCLoad are additive",
          "[power_flow][dc][load-accounting]") {
  auto split = make_two_bus_hybrid_dc_case(10, 20);
  auto combined = split;
  combined.dc.buses[1].pd_mw = 0.0;
  combined.dc.loads[0].p_mw = 5.0;

  const auto split_result = hacdcpf::solve_power_flow(split);
  const auto combined_result = hacdcpf::solve_power_flow(combined);
  REQUIRE(split_result.converged);
  REQUIRE(combined_result.converged);
  REQUIRE(split_result.vdc.size() == 2);
  CHECK(split_result.vdc[1] == Catch::Approx(combined_result.vdc[1]).margin(1e-10));

  const auto certificate =
      hacdcpf::sppt::certify_independent_hybrid_residual(
          split, split_result, 1e-8);
  REQUIRE(certificate.supported);
  CHECK(certificate.total_residual <= 1e-8);
}

TEST_CASE("Facade rebuilds SolverData after an un-hashed shunt field changes",
          "[power_flow][facade][cache]") {
  auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  const auto before = hacdcpf::solve_power_flow(sys);
  REQUIRE(before.converged);
  sys.ac.buses[1].gs_mw = 5.0;
  const auto after = hacdcpf::solve_power_flow(sys);
  REQUIRE(after.converged);
  CHECK(std::abs(before.vm[1] - after.vm[1]) > 1e-5);
}

TEST_CASE("Public HELM facade provides physical results and rejects hybrid scope",
          "[power_flow][helm][facade]") {
  const auto ac = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::powerflow::HelmOptions helm_options;
  helm_options.max_coef = 120;
  helm_options.mismatch = 1e-7;
  helm_options.enforce_q_limits = false;
  const auto result = hacdcpf::solve_power_flow_helm(ac, {}, helm_options);
  REQUIRE(result.converged);
  CHECK(result.residual <= 1e-7);
  REQUIRE(result.branch_flows.size() == ac.ac.branches.size());

  const auto rejected = hacdcpf::solve_power_flow_helm(
      make_two_bus_hybrid_dc_case(10, 20), {}, helm_options);
  CHECK_FALSE(rejected.converged);
  CHECK(rejected.diagnostics.termination_reason.find("AC subsystem") !=
        std::string::npos);
}

TEST_CASE("Explicit homotopy and Newton-Krylov facades are callable",
          "[power_flow][facade][homotopy][newton_krylov]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::PowerFlowOptions options;
  options.max_iter = 80;
  options.robust_nonlinear.homotopy_step0 = 0.2;
  const auto homotopy = hacdcpf::solve_power_flow_homotopy(sys, options);
  const auto krylov = hacdcpf::solve_power_flow_newton_krylov(sys, options);
  REQUIRE(homotopy.converged);
  REQUIRE(krylov.converged);
  REQUIRE(homotopy.branch_flows.size() == sys.ac.branches.size());
  REQUIRE(krylov.branch_flows.size() == sys.ac.branches.size());
}

TEST_CASE("FDPF sparse injection path remains numerically consistent",
          "[power_flow][fdpf][sparse]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-9;

  hacdcpf::powerflow::NewtonSolver newton;
  hacdcpf::powerflow::FDPFSolver fdpf;
  const auto nr = newton.solve(data, options);
  const auto result = fdpf.solve(data, options);
  REQUIRE(nr.converged);
  REQUIRE(result.converged);
  CHECK(std::abs(result.vm[1] - nr.vm[1]) < 2e-4);
  CHECK(std::abs(result.va[1] - nr.va[1]) < 2e-4);
}

TEST_CASE("FDPF uses aggregated component loads", "[power_flow][fdpf][aggregation]") {
  auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::Load component;
  component.index = 1;
  component.bus = 2;
  component.p_mw = 5.0;
  component.q_mvar = 2.0;
  component.in_service = true;
  sys.ac.loads.push_back(component);
  const auto data = hacdcpf::powerflow::make_solver_data(sys);

  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-9;
  hacdcpf::powerflow::NewtonSolver newton;
  hacdcpf::powerflow::FDPFSolver fdpf;
  const auto nr = newton.solve(data, options);
  const auto result = fdpf.solve(data, options);
  REQUIRE(nr.converged);
  REQUIRE(result.converged);
  CHECK(std::abs(result.vm[1] - nr.vm[1]) < 3e-4);
  CHECK(std::abs(result.va[1] - nr.va[1]) < 3e-4);
}
