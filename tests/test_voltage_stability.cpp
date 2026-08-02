#include <algorithm>
#include <cmath>

#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/power_flow/voltage_stability.hpp"
#include "hacdcpf/power_flow/solver_factory.hpp"
#include "hacdcpf/power_flow/power_flow_problem.hpp"
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

  hacdcpf::powerflow::CpfResult synthetic;
  synthetic.monitor_bus = 1;
  synthetic.lambda_max = 0.25;
  synthetic.p_max_mw = 70.0;
  synthetic.vm_at_nose = 0.8;
  const auto vsi = hacdcpf::powerflow::compute_vsi(synthetic, data);
  CHECK(std::abs(vsi.p_margin_mw - 15.0) < 1e-12);
}

TEST_CASE("CPF uses Q-limit-aware natural continuation when PV layout can switch",
          "[power_flow][cpf][audit][q-limits]") {
  auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  sys.ac.buses[1].bus_type = hacdcpf::BusType::PV;
  sys.ac.buses[1].vm_pu = 1.0;
  hacdcpf::Generator pv;
  pv.index = 2;
  pv.bus = 2;
  pv.in_service = true;
  pv.pg_mw = 50.0;
  pv.vg_pu = 1.0;
  pv.qmin_mvar = -25.0;
  pv.qmax_mvar = 25.0;
  sys.ac.generators.push_back(pv);
  const auto data = hacdcpf::powerflow::make_solver_data(sys);

  hacdcpf::powerflow::CpfSolver solver;
  solver.opts.enable_arc_length = true;
  solver.opts.lambda_max = 0.04;
  solver.opts.step_init = 0.01;
  solver.opts.corrector_tol = 1e-9;
  const auto result = solver.solve(
      data, hacdcpf::powerflow::CpfDirection::proportional(data));
  CHECK_FALSE(result.arc_length_used);
  CHECK(result.q_limits_enforced);
  CHECK(result.model_scope.find("pv-pq-active-set") != std::string::npos);
  CHECK_FALSE(result.warnings.empty());
  CHECK(result.trace.size() >= 2);
}

TEST_CASE("CPF does not label an unobserved first-step failure as a nose",
          "[power_flow][cpf][audit][nose-semantics]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  hacdcpf::powerflow::CpfSolver solver;
  solver.opts.step_init = 100.0;
  solver.opts.step_min = 100.0;
  solver.opts.lambda_max = 100.0;
  solver.opts.corrector_max_iter = 80;
  const auto result = solver.solve(
      data, hacdcpf::powerflow::CpfDirection::proportional(data));
  CHECK_FALSE(result.nose_found);
  CHECK(result.trace.size() == 1);
  CHECK(result.termination_reason.find("no continuation step") !=
        std::string::npos);
}

TEST_CASE("Continuation factory adapter requires progress and reports residual",
          "[power_flow][cpf][audit][factory]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::PowerFlowOptions options;
  options.max_iter = 80;
  options.tol = 1e-9;
  const auto problem = hacdcpf::build_power_flow_problem(sys, options);
  auto solver = hacdcpf::PowerFlowSolverFactory::create(
      hacdcpf::PowerFlowMethod::Continuation);
  const auto result = solver->solve(problem);
  REQUIRE(result.converged);
  CHECK(result.residual <= options.tol);

  auto no_progress_problem = problem;
  no_progress_problem.options.max_iter = 0;
  const auto no_progress = solver->solve(no_progress_problem);
  CHECK_FALSE(no_progress.converged);
  CHECK(no_progress.diagnostics.warnings.back().find("did not advance") !=
        std::string::npos);
}
