#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/power_flow/homotopy_continuation.hpp"
#include "power_flow_solver_test_utils.hpp"

TEST_CASE("Homotopy continuation reaches the full loading endpoint",
          "[power_flow][homotopy]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::PowerFlowOptions options;
  options.max_iter = 80;
  options.tol = 1e-9;
  options.robust_nonlinear.homotopy_step0 = 0.1;
  options.robust_nonlinear.homotopy_step_min = 1e-4;
  options.robust_nonlinear.homotopy_step_max = 0.4;
  options.robust_nonlinear.homotopy_max_steps = 30;

  hacdcpf::powerflow::HomotopyContinuationSolver solver;
  hacdcpf::powerflow::HomotopyState state;
  const auto result = solver.solve(sys, options, state);

  REQUIRE(result.converged);
  CHECK_FALSE(state.failed);
  CHECK(state.lambda == 1.0);
  CHECK(state.accepted_steps >= 3);
  CHECK(result.vm[1] < 1.0);
}
