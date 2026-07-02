#include <algorithm>
#include <cmath>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"

using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

HybridPowerSystem make_transient_2bus() {
  HybridPowerSystem sys;
  sys.name = "transient_2bus";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.vm_pu = 1.02;
  b1.va_deg = 0.0;
  b1.in_service = true;

  ACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.vm_pu = 1.0;
  b2.va_deg = 0.0;
  b2.in_service = true;

  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.05;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.02;
  g.pg_mw = 40.0;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  g.xdpp_pu = 0.20;
  sys.ac.generators = {g};

  Load ld;
  ld.index = 1;
  ld.bus = 2;
  ld.p_mw = 30.0;
  ld.q_mvar = 10.0;
  ld.in_service = true;
  sys.ac.loads = {ld};

  return sys;
}

HybridPowerSystem make_hybrid_dc_case() {
  auto sys = make_transient_2bus();

  DCBus d1;
  d1.index = 1;
  d1.bus_type = DCBusType::DC_V;
  d1.vm_pu = 1.0;
  d1.in_service = true;
  DCBus d2;
  d2.index = 2;
  d2.bus_type = DCBusType::DC_P;
  d2.vm_pu = 1.0;
  d2.in_service = true;
  sys.dc.buses = {d1, d2};

  DCBranch dcbr;
  dcbr.index = 1;
  dcbr.from_bus = 1;
  dcbr.to_bus = 2;
  dcbr.r_pu = 0.02;
  dcbr.in_service = true;
  sys.dc.branches = {dcbr};

  DCLoad dcl;
  dcl.index = 1;
  dcl.bus = 2;
  dcl.p_mw = 5.0;
  dcl.in_service = true;
  sys.dc.loads = {dcl};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 2;
  vsc.bus_dc = 1;
  vsc.in_service = true;
  vsc.control_mode = ConverterMode::PQ_MODE;
  vsc.p_set_mw = 8.0;
  vsc.p_schedule_mw = 8.0;
  vsc.q_set_mvar = 1.0;
  vsc.eta = 0.98;
  sys.vsc_converters = {vsc};

  DCDCConverter dcdc;
  dcdc.index = 1;
  dcdc.bus_in = 1;
  dcdc.bus_out = 2;
  dcdc.in_service = true;
  dcdc.p_ref_mw = 4.0;
  dcdc.eta = 0.97;
  sys.dc.dcdc_converters = {dcdc};

  return sys;
}

HybridPowerSystem make_asymmetric_ac_case() {
  auto sys = make_transient_2bus();
  sys.name = "transient_asymmetric_ac";
  sys.ac.loads.clear();

  AsymmetricLoad load;
  load.index = 1;
  load.bus = 2;
  load.name = "LV unbalance";
  load.pa_mw = 15.0;
  load.qa_mvar = 4.0;
  load.pb_mw = 6.0;
  load.qb_mvar = 2.0;
  load.pc_mw = 3.0;
  load.qc_mvar = 1.0;
  load.scaling = 1.0;
  load.in_service = true;
  sys.ac.asymmetric_loads = {load};

  return sys;
}

HybridPowerSystem make_unbalanced_three_phase_case() {
  HybridPowerSystem sys;
  sys.name = "transient_three_phase";
  sys.base_mva = 10.0;

  ThreePhaseACSystem tp;
  tp.base_mva = 10.0;
  tp.base_freq_hz = 50.0;

  ThreePhaseACBus b1;
  b1.index = 1;
  b1.bus_type = BusType::SLACK;
  b1.phase_mask = PhaseMask::abc();
  b1.in_service = true;

  ThreePhaseACBus b2;
  b2.index = 2;
  b2.bus_type = BusType::PQ;
  b2.phase_mask = PhaseMask::abc();
  b2.in_service = true;

  tp.buses = {b1, b2};

  ThreePhaseACLine line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.phase_mask = PhaseMask::abc();
  line.r1_pu = 0.03;
  line.x1_pu = 0.08;
  line.r0_pu = 0.05;
  line.x0_pu = 0.12;
  line.in_service = true;
  tp.lines = {line};

  ThreePhaseExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.vm_pu = 1.0;
  grid.va_deg = 0.0;
  grid.x1_pu = 0.02;
  grid.in_service = true;
  tp.external_grids = {grid};

  ThreePhaseLoad load;
  load.index = 1;
  load.bus = 2;
  load.phase_mask = PhaseMask::abc();
  load.p_a_mw = 0.50;
  load.q_a_mvar = 0.10;
  load.p_b_mw = 0.20;
  load.q_b_mvar = 0.05;
  load.p_c_mw = 0.10;
  load.q_c_mvar = 0.02;
  load.in_service = true;
  tp.loads = {load};

  sys.three_phase_ac = tp;
  return sys;
}

DynamicSolverOptions fast_options() {
  DynamicSolverOptions opt;
  opt.t_start_s = 0.0;
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  opt.run_power_flow_initialization = false;
  opt.singular_regularization_pu = 1e-7;
  return opt;
}

}  // namespace

TEST_CASE("DynamicModelBuilder creates canonical transient system", "[dynamics][transient]") {
  const auto sys = make_transient_2bus();
  const auto opt = fast_options();

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  CHECK(dyn.network.ac_bus_ids.size() == 2);
  CHECK(dyn.network.acPhaseNodeCount() == 6);
  CHECK(dyn.network.ac_branches.size() == 1);
  CHECK(dyn.devices.size() >= 2);
  CHECK(dyn.stateCount() > 0);
  CHECK(dyn.y.Vac_abc.size() == 6);
}

TEST_CASE("Transient solver runs partitioned phasor dynamics", "[dynamics][transient]") {
  const auto sys = make_transient_2bus();
  auto opt = fast_options();
  opt.solver_type = DynamicSolverType::PartitionedHeun;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.steps == 5);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->max_ac_voltage_pu > 0.1);
  CHECK(result.final_snapshot()->min_ac_voltage_pu > 0.1);
}

TEST_CASE("Transient events preserve branch type and apply topology changes", "[dynamics][events]") {
  const auto sys = make_transient_2bus();
  auto opt = fast_options();
  opt.t_end_s = 0.03;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent trip;
  trip.time_s = 0.01;
  trip.type = DynamicEventType::ACBranchTrip;
  trip.component_index = 1;
  trip.component_type = "AC";
  trip.label = "trip AC branch 1";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  REQUIRE(result.success);
  REQUIRE_FALSE(result.applied_events.empty());
  CHECK(result.applied_events.front() == "trip AC branch 1");
  REQUIRE_FALSE(dyn.network.ac_branches.empty());
  CHECK_FALSE(dyn.network.ac_branches.front().in_service);
}

TEST_CASE("Transient solver lands on off-grid event times", "[dynamics][events]") {
  const auto sys = make_transient_2bus();
  auto opt = fast_options();
  opt.t_end_s = 0.03;
  opt.dt_s = 0.01;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent event;
  event.time_s = 0.015;
  event.type = DynamicEventType::ACLoadScale;
  event.component_index = 1;
  event.value = 0.5;
  event.label = "mid-step load scale";
  dyn.events.push_back(event);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  REQUIRE(result.success);
  REQUIRE_FALSE(result.applied_events.empty());
  CHECK(result.applied_events.front() == "mid-step load scale");
  bool saw_event_time = false;
  for (const auto& snapshot : result.snapshots) {
    saw_event_time = saw_event_time || std::abs(snapshot.time_s - 0.015) < 1e-12;
  }
  CHECK(saw_event_time);
}

TEST_CASE("Canonical transient builder keeps asymmetric loads per phase once", "[dynamics][three_phase]") {
  const auto sys = make_asymmetric_ac_case();
  auto opt = fast_options();
  opt.project_to_canonical = true;
  opt.t_end_s = 0.02;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  int three_phase_loads = 0;
  int balanced_loads = 0;
  for (const auto& device : dyn.devices) {
    if (device->type() == "ThreePhaseLoad") ++three_phase_loads;
    if (device->type() == "ACLoad") ++balanced_loads;
  }
  CHECK(three_phase_loads == 1);
  CHECK(balanced_loads == 0);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& v = result.final_snapshot()->vac_abc;
  REQUIRE(v.size() >= 6);
  const double va = std::abs(v[3]);
  const double vb = std::abs(v[4]);
  const double vc = std::abs(v[5]);
  CHECK(std::max({va, vb, vc}) - std::min({va, vb, vc}) > 1e-4);
}

TEST_CASE("Hybrid AC/DC transient includes VSC and DC/DC coupling", "[dynamics][hybrid_acdc]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.02;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->vdc.size() == 2);
  CHECK(result.final_snapshot()->max_dc_voltage_pu > 0.1);
  CHECK(result.final_snapshot()->vdc[0] == Catch::Approx(1.0).margin(5e-3));
}

TEST_CASE("Transient VSC roles separate AC and DC grid forming", "[dynamics][hybrid_acdc]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].grid_forming = true;
  sys.vsc_converters[0].ac_grid_forming = false;
  sys.vsc_converters[0].control_mode = ConverterMode::PQ_MODE;
  sys.vsc_converters[0].v_dc_set_pu = 1.03;
  auto opt = fast_options();

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  bool saw_dc_source = false;
  bool saw_ac_gfm = false;
  for (const auto& device : dyn.devices) {
    saw_dc_source = saw_dc_source || device->type() == "DCVoltageSource";
    saw_ac_gfm = saw_ac_gfm || device->type() == "VSCGridForming";
  }
  CHECK(saw_dc_source);
  CHECK_FALSE(saw_ac_gfm);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->vdc[0] == Catch::Approx(1.03).margin(5e-3));
}

TEST_CASE("Explicit three-phase transient model keeps unbalanced phase loads", "[dynamics][three_phase]") {
  const auto sys = make_unbalanced_three_phase_case();
  auto opt = fast_options();
  opt.t_end_s = 0.02;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  bool saw_three_phase_load = false;
  for (const auto& device : dyn.devices) {
    saw_three_phase_load = saw_three_phase_load || device->type() == "ThreePhaseLoad";
  }
  CHECK(saw_three_phase_load);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& v = result.final_snapshot()->vac_abc;
  REQUIRE(v.size() >= 6);
  const double va = std::abs(v[3]);
  const double vb = std::abs(v[4]);
  const double vc = std::abs(v[5]);
  const double vmax = std::max({va, vb, vc});
  const double vmin = std::min({va, vb, vc});
  CHECK(vmax - vmin > 1e-4);
}

TEST_CASE("Updated GFL inverter exposes PLL current-limited positive-sequence telemetry",
          "[dynamics][gfl]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].i_ac_max_pu = 0.04;
  sys.vsc_converters[0].q_set_mvar = 6.0;
  auto opt = fast_options();
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;
  opt.solver_type = DynamicSolverType::PartitionedRK4;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("pll_frequency_hz") > 49.0);
  CHECK(it->values.at("pll_frequency_hz") < 51.0);
  CHECK(it->values.at("i_mag_pu") <= 0.045);
  CHECK(it->values.at("current_limit_active") == Catch::Approx(1.0));
  CHECK(std::abs(it->values.at("p_mw")) > 0.1);
}

TEST_CASE("Transient initialization uses solved power flow and exposes canvas metadata",
          "[dynamics][initialization][gui]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.01;
  opt.dt_s = 0.01;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_requested);
  CHECK(result.initialization.power_flow_converged);
  CHECK_FALSE(result.initialization.fallback_voltage_setpoints);
  CHECK(result.initialization.max_ac_voltage_pu > 0.9);
  CHECK(result.initialization.dynamic_trim_iterations > 0);
  CHECK(result.initialization.dynamic_fast_dxdt_inf_norm < 1e-5);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto vsc = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(vsc != outputs.end());
  CHECK(vsc->canvas_type == "vsc");
  CHECK(vsc->canvas_index == 1);
  CHECK(vsc->component_domain == "AC");
  CHECK(vsc->source_type == "vsc_grid_following");
}

TEST_CASE("Transient initialization trims GFL fast states to avoid artificial PLL settling",
          "[dynamics][initialization][trim]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  opt.record_every_step = true;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() >= 2);
  CHECK(result.initialization.dynamic_fast_dxdt_inf_norm < 1e-5);

  auto pll_freq = [](const DynamicSnapshot& snapshot) {
    const auto it = std::find_if(snapshot.device_outputs.begin(),
                                 snapshot.device_outputs.end(),
                                 [](const DynamicDeviceOutput& out) {
                                   return out.type == "VSCGridFollowing";
                                 });
    REQUIRE(it != snapshot.device_outputs.end());
    REQUIRE(it->values.count("pll_frequency_hz") == 1);
    return it->values.at("pll_frequency_hz");
  };
  CHECK(pll_freq(result.snapshots.front()) == Catch::Approx(pll_freq(result.snapshots.back())).margin(1e-4));
}

TEST_CASE("Updated GFL inverter can use dynamic DC-link voltage state",
          "[dynamics][gfl][dc_link]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].v_dc_set_pu = 1.02;
  auto opt = fast_options();
  opt.dynamic_dc_link = true;
  opt.dc_link_capacitance_s = 0.20;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  bool saw_dynamic_vsc = false;
  for (const auto& device : dyn.devices) {
    if (device->type() == "VSCGridFollowing") {
      const auto out = device->output(dyn.x, dyn.y);
      saw_dynamic_vsc = out.values.count("vdc_link_pu") == 1 &&
                        out.values.at("dc_link_dynamic") == Catch::Approx(1.0);
    }
  }
  CHECK(saw_dynamic_vsc);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("vdc_link_pu") > 0.2);
  CHECK(it->values.at("vdc_link_pu") < 2.0);
  CHECK(it->values.at("dc_link_dynamic") == Catch::Approx(1.0));
}

TEST_CASE("Updated GFM inverter stamps Norton voltage source and droop telemetry",
          "[dynamics][gfm]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].ac_grid_forming = true;
  sys.vsc_converters[0].grid_forming = false;
  sys.vsc_converters[0].control_mode = ConverterMode::AC_GRID_FORMING;
  sys.vsc_converters[0].pmax_mw = 12.0;
  sys.vsc_converters[0].i_ac_max_pu = 0.25;
  auto opt = fast_options();
  opt.t_end_s = 0.02;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridForming";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("e_internal_pu") > 0.2);
  CHECK(it->values.at("frequency_hz") > 45.0);
  CHECK(it->values.at("frequency_hz") < 55.0);
  CHECK(it->values.at("i_rms_pu") >= 0.0);
  CHECK(it->values.count("p_filtered_mw") == 1);
}

TEST_CASE("Updated GFM inverter exposes dynamic DC-link telemetry",
          "[dynamics][gfm][dc_link]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].ac_grid_forming = true;
  sys.vsc_converters[0].grid_forming = false;
  sys.vsc_converters[0].control_mode = ConverterMode::AC_GRID_FORMING;
  sys.vsc_converters[0].v_dc_set_pu = 1.01;
  auto opt = fast_options();
  opt.dynamic_dc_link = true;
  opt.dc_link_capacitance_s = 0.20;
  opt.t_end_s = 0.02;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridForming";
  });
  REQUIRE(it != outputs.end());
  CHECK(it->values.at("vdc_link_pu") > 0.2);
  CHECK(it->values.at("vdc_link_pu") < 2.0);
  CHECK(it->values.at("dc_link_dynamic") == Catch::Approx(1.0));
  CHECK(it->values.count("p_dc_mw") == 1);
}

TEST_CASE("Implicit transient Newton solvers run without Heun fallback", "[dynamics][solver]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.02;
  opt.dt_s = 0.01;
  opt.solver_type = DynamicSolverType::BackwardEulerNewton;
  opt.newton_tol = 1e-7;
  opt.max_newton_iters = 12;

  const DynamicResults be = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(be.success);
  CHECK(be.steps == 2);
  CHECK(be.newton_iterations > 0);
  CHECK(std::none_of(be.warnings.begin(), be.warnings.end(), [](const std::string& warning) {
    return warning.find("PartitionedHeun was used") != std::string::npos;
  }));

  opt.solver_type = DynamicSolverType::TrapezoidalNewton;
  const DynamicResults trap = hacdcpf::run_transient_simulation(sys, opt);
  REQUIRE(trap.success);
  CHECK(trap.newton_iterations > 0);
}

TEST_CASE("Documented transient device headers expose executable standard profiles",
          "[dynamics][devices][standards]") {
  DynamicState x;
  NetworkState y;
  PowerFlowResult pf;
  pf.va = {0.0};
  y.resize(3, 0);
  y.Vac_abc[0] = std::polar(1.0, 0.0);
  y.Vac_abc[1] = std::polar(1.0, -2.0 * 3.14159265358979323846 / 3.0);
  y.Vac_abc[2] = std::polar(1.0, 2.0 * 3.14159265358979323846 / 3.0);

  GovernorDynamicParams governor_params;
  governor_params.component_index = 7;
  governor_params.base_mva = 100.0;
  governor_params.p_ref_mw = 40.0;
  Governor governor(governor_params);
  ExciterDynamicParams exciter_params;
  exciter_params.component_index = 8;
  exciter_params.bus = 1;
  exciter_params.bus_pos = 0;
  Exciter exciter(exciter_params);
  PVDynamicParams pv_params;
  pv_params.component_index = 9;
  pv_params.bus = 1;
  pv_params.bus_pos = 0;
  pv_params.p_ref_mw = 1.5;
  pv_params.q_ref_mvar = 0.2;
  PVDynamic pv(pv_params);
  ProtectionRelayParams relay_params;
  relay_params.component_index = 10;
  relay_params.bus = 1;
  relay_params.bus_pos = 0;
  ProtectionRelay relay(relay_params);

  std::vector<DynamicDevice*> devices{&governor, &exciter, &pv, &relay};
  int offset = 0;
  for (auto* device : devices) device->assignStateIndices(offset);
  x.resize(static_cast<std::size_t>(offset));
  for (auto* device : devices) device->initializeFromPowerFlow(pf, x, y);

  Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(x.size());
  for (auto* device : devices) device->computeDerivatives(0.0, x, y, dxdt);
  CHECK(dxdt.allFinite());

  DynamicStamp stamp(3, 0);
  pv.stamp(0.0, x, y, stamp);
  CHECK(stamp.Iac.norm() > 0.0);

  CHECK(governor.output(x, y).model_standard == "IEEE");
  CHECK(exciter.output(x, y).model_standard == "IEEE4215");
  CHECK(pv.output(x, y).model_standard == "IEEE1547");
  CHECK(relay.output(x, y).model_name == "VoltageFrequencyRelay");
}

TEST_CASE("Documented integrators and algebraic solvers advance dynamic systems",
          "[dynamics][integration][solver]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.01;
  opt.dt_s = 0.01;
  opt.solver_type = DynamicSolverType::PartitionedRK4;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  AlgebraicNetworkSolver algebraic;
  const auto algebraic_result = algebraic.solve(dyn, opt.t_start_s);
  REQUIRE(algebraic_result.success);
  CHECK(dyn.y.Vac_abc.size() > 0);

  RK4 rk4;
  const auto rk4_step = rk4.step(dyn, opt.t_start_s, opt.dt_s);
  REQUIRE(rk4_step.success);
  CHECK(rk4_step.derivative_evaluations == 4);
  CHECK(dyn.x.dxdt.size() == dyn.x.x.size());

  DynamicSystem dyn_be = builder.build(sys, opt);
  BackwardEuler be;
  DaeSolver dae(be);
  const auto dae_step = dae.step(dyn_be, opt.t_start_s, opt.dt_s);
  REQUIRE(dae_step.success);
  CHECK(dae_step.nonlinear_iterations > 0);
  CHECK(dyn_be.x.time_s == Catch::Approx(opt.dt_s));

  Eigen::SparseMatrix<double> a(2, 2);
  a.insert(0, 0) = 4.0;
  a.insert(1, 1) = 2.0;
  a.makeCompressed();
  Eigen::VectorXd b(2);
  b << 8.0, 6.0;
  Eigen::VectorXd solved;
  SparseLinearSolver sparse;
  const auto solve_result = sparse.solve(a, b, solved);
  REQUIRE(solve_result.success);
  CHECK(solved[0] == Catch::Approx(2.0));
  CHECK(solved[1] == Catch::Approx(3.0));
}

TEST_CASE("Transient results support sampled recording and CSV export",
          "[dynamics][results][sampling]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.output_every_steps = 2;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() == 4);
  CHECK(result.snapshots.front().time_s == Catch::Approx(0.0));
  CHECK(result.snapshots.back().time_s == Catch::Approx(0.05));

  const std::string csv = to_csv(result);
  CHECK(csv.find("time_s") != std::string::npos);
  CHECK(csv.find("vac_0_mag_pu") != std::string::npos);
  CHECK(csv.find("VSCGridFollowing_1_p_mw") != std::string::npos);
}

TEST_CASE("Transient contingencies carry named parameters and structured records",
          "[dynamics][events][params]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.03;
  opt.dt_s = 0.01;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  DynamicEvent fault;
  fault.time_s = 0.01;
  fault.type = DynamicEventType::FaultShunt;
  fault.bus = 2;
  fault.component_type = "AC";
  fault.label = "parameterized AC fault";
  fault.params["r_pu"] = 0.001;
  fault.params["x_pu"] = 0.002;
  fault.params["duration_s"] = 0.01;
  dyn.events.push_back(fault);

  DynamicEvent storage;
  storage.time_s = 0.02;
  storage.type = DynamicEventType::StoragePowerStep;
  storage.component_index = 1;
  storage.label = "storage dispatch";
  storage.params["p_ref_mw"] = -2.0;
  dyn.events.push_back(storage);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  REQUIRE(result.success);
  REQUIRE(result.applied_event_records.size() == 2);
  CHECK(result.applied_event_records.front().label == "parameterized AC fault");
  CHECK(result.applied_event_records.front().params.at("r_pu") == Catch::Approx(0.001));
  CHECK(result.applied_event_records.back().params.at("p_ref_mw") == Catch::Approx(-2.0));
}

TEST_CASE("Dynamic sparse linear solver accepts optional backend requests",
          "[dynamics][solver][linear]") {
  Eigen::SparseMatrix<double> a(2, 2);
  a.insert(0, 0) = 3.0;
  a.insert(1, 1) = 5.0;
  a.makeCompressed();
  Eigen::VectorXd b(2);
  b << 6.0, 20.0;

  for (const auto type : {DynamicLinearSolverType::EigenSparseLU,
                          DynamicLinearSolverType::EigenBiCGSTAB,
                          DynamicLinearSolverType::KLU,
                          DynamicLinearSolverType::UMFPACK}) {
    Eigen::VectorXd x;
    SparseLinearSolver solver(type);
    const auto result = solver.solve(a, b, x);
    REQUIRE(result.success);
    CHECK(x[0] == Catch::Approx(2.0));
    CHECK(x[1] == Catch::Approx(4.0));
  }
}
