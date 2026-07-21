#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/power_flow/voltage_stability.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "power_flow_solver_test_utils.hpp"

TEST_CASE("Arc-length CPF passes the two-bus P-V nose",
          "[power_flow][cpf][arc_length]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  const auto data = hacdcpf::powerflow::make_solver_data(sys);

  hacdcpf::powerflow::CpfSolver solver;
  solver.opts.lambda_max = 1.0;
  solver.opts.step_init = 0.03;
  solver.opts.step_min = 1e-5;
  solver.opts.step_max = 0.04;
  solver.opts.step_grow = 1.2;
  solver.opts.corrector_tol = 1e-9;
  solver.opts.trace_all_buses = true;
  solver.opts.vm_min_pu = 0.1;
  solver.opts.lower_branch_steps = 3;

  const auto result = solver.solve(
      data, hacdcpf::powerflow::CpfDirection::proportional(data));
  REQUIRE(result.nose_found);
  REQUIRE(result.trace.size() >= 5);
  CHECK(result.lambda_max > 0.25);
  CHECK(result.lambda_max < 0.50);

  const auto nose = std::max_element(
      result.trace.begin(), result.trace.end(),
      [](const auto& a, const auto& b) { return a.lambda < b.lambda; });
  REQUIRE(nose != result.trace.end());
  REQUIRE(std::next(nose) != result.trace.end());
  CHECK(std::next(nose)->lambda < nose->lambda);
  CHECK(result.vm_at_nose < result.trace.front().vm_monitor);
}

TEST_CASE("CPF proportional direction includes explicit component loads",
          "[power_flow][cpf][aggregation]") {
  auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::Load component;
  component.index = 1;
  component.bus = 2;
  component.p_mw = 10.0;
  component.q_mvar = 4.0;
  component.scaling = 0.5;
  component.in_service = true;
  sys.ac.loads.push_back(component);

  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  REQUIRE(data.has_component_loads);
  const auto direction = hacdcpf::powerflow::CpfDirection::proportional(data);
  CHECK(std::abs(direction.dp_load[1] - 55.0) < 1e-12);
  CHECK(std::abs(direction.dq_load[1] - 22.0) < 1e-12);
}
