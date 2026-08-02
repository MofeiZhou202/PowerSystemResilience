#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

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

TEST_CASE("Homotopy scaling covers every assembled injection family",
          "[power_flow][homotopy][audit][scaling]") {
  hacdcpf::HybridPowerSystem sys;

  hacdcpf::PVSystem pv;
  pv.p_mw = 8.0;
  pv.q_mvar = 4.0;
  pv.irradiance = 800.0;
  sys.ac.pv_systems = {pv};

  hacdcpf::ChargingStation charging;
  charging.p_total_kw = 400.0;
  charging.q_total_kvar = 100.0;
  sys.ac.charging_stations = {charging};

  hacdcpf::VirtualPowerPlant vpp;
  vpp.p_output_mw = 12.0;
  vpp.q_output_mvar = 3.0;
  sys.vpps = {vpp};

  hacdcpf::Microgrid microgrid;
  microgrid.p_exchange_mw = 16.0;
  microgrid.p_set_mw = 20.0;
  sys.microgrids = {microgrid};

  hacdcpf::LCCConverter lcc;
  lcc.p_set_mw = 24.0;
  lcc.i_set_ka = 2.0;
  sys.lcc_converters = {lcc};

  hacdcpf::EnergyRouter router;
  hacdcpf::EnergyRouterPort port;
  port.p_mw = 28.0;
  port.q_mvar = 8.0;
  port.p_set_mw = 32.0;
  port.q_set_mvar = 12.0;
  router.ports = {port};
  sys.energy_routers = {router};

  hacdcpf::DCStorage dc_storage;
  dc_storage.p_mw = 36.0;
  sys.dc.dc_storage = {dc_storage};

  hacdcpf::powerflow::detail::scale_system_by_lambda(sys, 0.25);
  CHECK(sys.ac.pv_systems[0].p_mw == Catch::Approx(2.0));
  CHECK(sys.ac.pv_systems[0].q_mvar == Catch::Approx(1.0));
  CHECK(sys.ac.pv_systems[0].irradiance == Catch::Approx(200.0));
  CHECK(sys.ac.charging_stations[0].p_total_kw == Catch::Approx(100.0));
  CHECK(sys.ac.charging_stations[0].q_total_kvar == Catch::Approx(25.0));
  CHECK(sys.vpps[0].p_output_mw == Catch::Approx(3.0));
  CHECK(sys.vpps[0].q_output_mvar == Catch::Approx(0.75));
  CHECK(sys.microgrids[0].p_exchange_mw == Catch::Approx(4.0));
  CHECK(sys.microgrids[0].p_set_mw == Catch::Approx(5.0));
  CHECK(sys.lcc_converters[0].p_set_mw == Catch::Approx(6.0));
  CHECK(sys.lcc_converters[0].i_set_ka == Catch::Approx(0.5));
  CHECK(sys.energy_routers[0].ports[0].p_mw == Catch::Approx(7.0));
  CHECK(sys.energy_routers[0].ports[0].q_mvar == Catch::Approx(2.0));
  CHECK(sys.energy_routers[0].ports[0].p_set_mw == Catch::Approx(8.0));
  CHECK(sys.energy_routers[0].ports[0].q_set_mvar == Catch::Approx(3.0));
  CHECK(sys.dc.dc_storage[0].p_mw == Catch::Approx(9.0));
}

TEST_CASE("Homotopy class interface fails honestly before full loading",
          "[power_flow][homotopy][audit][failure]") {
  const auto sys = hacdcpf::test::make_two_bus_voltage_stability_case();
  hacdcpf::PowerFlowOptions options;
  options.max_iter = 80;
  options.robust_nonlinear.homotopy_max_steps = 0;
  hacdcpf::powerflow::HomotopyContinuationSolver solver;
  hacdcpf::powerflow::HomotopyState state;
  const auto result = solver.solve(sys, options, state);
  CHECK(state.failed);
  CHECK(state.lambda == 0.0);
  CHECK_FALSE(result.converged);
  CHECK(std::isinf(result.residual));
  CHECK(result.diagnostics.termination_reason.find("lambda=1") !=
        std::string::npos);
}
