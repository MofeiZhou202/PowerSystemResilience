#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/power_flow/fdpf_solver.hpp"
#include "hacdcpf/power_flow/helm_solver.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "power_flow_solver_test_utils.hpp"

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
