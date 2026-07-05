#include <algorithm>
#include <complex>
#include <cmath>
#include <map>
#include <string>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"

using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

const DynamicModelDescriptor& require_model(const std::string& name) {
  auto m = find_dynamic_model(name);
  REQUIRE(m.has_value());
  static thread_local DynamicModelDescriptor held;
  held = *m;
  return held;
}

// Build a 2-bus system with one non-slack machine carrying the given control
// blocks, run a short transient, and return the results so we can confirm the
// catalog parameter keys actually reach the device.
DynamicResults run_with_blocks(std::map<std::string, std::map<std::string, double>> blocks) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;
  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.02;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PV; b2.vm_pu = 1.0;
  sys.ac.buses = {b1, b2};
  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.r_pu = 0.01; br.x_pu = 0.05; br.tap = 1.0;
  sys.ac.branches = {br};
  Generator gs; gs.index = 1; gs.bus = 1; gs.is_slack = true; gs.vg_pu = 1.02; gs.pg_mw = 40;
  gs.xdpp_pu = 0.2; gs.qmax_mvar = 300; gs.qmin_mvar = -300;
  Generator g2; g2.index = 2; g2.bus = 2; g2.vg_pu = 1.0; g2.pg_mw = 50; g2.qg_mvar = 10;
  g2.xdpp_pu = 0.2; g2.inertia_h = 3.0; g2.pmax_mw = 200; g2.qmax_mvar = 300; g2.qmin_mvar = -300;
  g2.dynamic_model.model_name = "ClassicalMachine";
  for (auto& [type, params] : blocks) {
    hacdcpf::DynamicModelComponentProfile c;
    c.type = type;
    c.model = type == "governor" ? "TGOV1" : type == "exciter" ? "SEXS" : "PSS1A";
    c.parameters = params;
    g2.dynamic_model.components.push_back(std::move(c));
  }
  sys.ac.generators = {gs, g2};
  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = 30; ld.q_mvar = 10;
  sys.ac.loads = {ld};

  DynamicSolverOptions opt;
  opt.t_end_s = 0.1;
  opt.dt_s = 0.01;
  return hacdcpf::dynamics::run_transient_simulation(sys, opt);
}

DynamicResults run_with_vsc(ConverterMode mode) {
  HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.02;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ; b2.vm_pu = 1.0;
  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.01;
  br.x_pu = 0.05;
  br.tap = 1.0;
  sys.ac.branches = {br};

  Generator slack;
  slack.index = 1;
  slack.bus = 1;
  slack.is_slack = true;
  slack.vg_pu = 1.02;
  slack.pg_mw = 40.0;
  slack.xdpp_pu = 0.2;
  slack.qmax_mvar = 300.0;
  slack.qmin_mvar = -300.0;
  sys.ac.generators = {slack};

  Load ld;
  ld.index = 1;
  ld.bus = 2;
  ld.p_mw = 20.0;
  ld.q_mvar = 5.0;
  sys.ac.loads = {ld};

  VSCConverter vsc;
  vsc.index = 1;
  vsc.bus_ac = 2;
  vsc.control_mode = mode;
  vsc.in_service = true;
  vsc.p_set_mw = 5.0;
  vsc.p_schedule_mw = 5.0;
  vsc.q_set_mvar = 1.0;
  vsc.v_ac_set_pu = 1.0;
  vsc.eta = 0.98;
  sys.vsc_converters = {vsc};

  DynamicSolverOptions opt;
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;
  return hacdcpf::dynamics::run_transient_simulation(sys, opt);
}

double device_value(const DynamicResults& r, const std::string& type, const std::string& key) {
  REQUIRE_FALSE(r.snapshots.empty());
  for (const auto& d : r.snapshots.front().device_outputs) {
    if (d.type == type) {
      auto it = d.values.find(key);
      if (it != d.values.end()) return it->second;
    }
  }
  FAIL("device output not found: " + type + "/" + key);
  return 0.0;
}

double device_value(const DynamicResults& r,
                    const std::string& type,
                    int component_index,
                    const std::string& key) {
  REQUIRE_FALSE(r.snapshots.empty());
  for (const auto& d : r.snapshots.front().device_outputs) {
    if (d.type == type && d.component_index == component_index) {
      auto it = d.values.find(key);
      if (it != d.values.end()) return it->second;
    }
  }
  FAIL("device output not found: " + type + "/" + std::to_string(component_index) +
       "/" + key);
  return 0.0;
}

MachineControlLink make_test_machine_link(StateIndexRange& machine_range) {
  MachineControlLink link;
  link.range = &machine_range;
  link.valid = true;
  link.omega_local = 1;
  link.pm_local = 6;
  link.efd_local = 7;
  link.inner_vars.machine_range = &machine_range;
  link.inner_vars.valid = true;
  link.inner_vars.bus_pos = 0;
  link.inner_vars.omega_local = 1;
  link.inner_vars.state_local[static_cast<std::size_t>(
      static_cast<int>(GeneratorInnerVar::MechanicalTorque))] = 6;
  link.inner_vars.state_local[static_cast<std::size_t>(
      static_cast<int>(GeneratorInnerVar::FieldVoltage))] = 7;
  return link;
}

NetworkState single_bus_voltage(double vm) {
  constexpr double pi = 3.141592653589793238462643383279502884;
  NetworkState y;
  y.resize(3, 0);
  y.Vac_abc[0] = std::polar(vm, 0.0);
  y.Vac_abc[1] = std::polar(vm, -2.0 * pi / 3.0);
  y.Vac_abc[2] = std::polar(vm, 2.0 * pi / 3.0);
  return y;
}

}  // namespace

TEST_CASE("Dynamic model catalog covers the wired models", "[dynamics][catalog]") {
  const auto& catalog = dynamic_model_catalog();
  REQUIRE(catalog.size() >= 15);
  for (const char* name :
       {"GENROU", "GENROE", "GENSAL", "GENSAE", "OneDOneQMachine",
        "SimpleMarconatoMachine", "ClassicalMachine", "SingleMass",
        "FiveMassShaft", "TGOV1", "IEEEG1", "TGTypeI", "TGTypeII", "SEXS", "IEEET1", "AVRSimple",
        "AVRTypeI", "AVRTypeII", "PSS1A", "IEEEST", "STAB1",
        "DynamicRLLine", "REGC_REEC_GFL_Subset",
        "GridFormingNortonDroop", "ReducedOrderPLL", "KauraPLL", "FixedFrequency",
        "AverageConverter", "ConstantDCSource", "DynamicDCLink", "RLFilter",
        "LCLFilter", "GFLPQOuterControl", "GFMDroopOuterControl",
        "PIInnerCurrentControl", "VirtualImpedanceInnerControl",
        "FirstOrderDCDCConverter", "BatterySOCFirstOrder", "ZIP"}) {
    INFO("model " << name);
    CHECK(find_dynamic_model(name).has_value());
  }
}

TEST_CASE("Every block-composition model resolves in the catalog", "[dynamics][catalog]") {
  for (const auto& comp : dynamic_block_composition()) {
    for (const auto& slot : comp.slots) {
      for (const auto& model : slot.model_names) {
        INFO(comp.canvas_type << "/" << slot.slot << " -> " << model);
        CHECK(find_dynamic_model(model).has_value());
      }
      if (!slot.default_model.empty() && slot.default_model != "None") {
        INFO(comp.canvas_type << "/" << slot.slot << " default " << slot.default_model);
        CHECK(find_dynamic_model(slot.default_model).has_value());
      }
    }
  }
}

TEST_CASE("Catalog parameter bounds are self-consistent", "[dynamics][catalog]") {
  for (const auto& model : dynamic_model_catalog()) {
    for (const auto& p : model.parameters) {
      INFO(model.model_name << "/" << p.key);
      if (p.min_value && p.max_value) {
        CHECK(*p.min_value <= *p.max_value);
        CHECK(p.default_value >= *p.min_value);
        CHECK(p.default_value <= *p.max_value);
      }
      CHECK_FALSE(p.key.empty());
      CHECK_FALSE(p.label.empty());
    }
  }
}

TEST_CASE("Governor / exciter / PSS catalog keys are consumed by the builder",
          "[dynamics][catalog]") {
  // Set distinctive values via the catalog's canonical keys and confirm they
  // are echoed by the built device — proving the keys are not dead.
  const DynamicResults r = run_with_blocks({
      {"governor", {{"R", 0.037}}},
      {"exciter", {{"Ka", 42.0}}},
      {"pss", {{"Ks", 7.5}}},
  });
  REQUIRE(r.success);
  CHECK(device_value(r, "Governor", "droop_r") == Catch::Approx(0.037));
  CHECK(device_value(r, "Exciter", "ka") == Catch::Approx(42.0));
  CHECK(device_value(r, "PSS", "ks") == Catch::Approx(7.5));
}

TEST_CASE("Phase 3 generator composition publishes the inner-variable bus",
          "[dynamics][catalog]") {
  const DynamicResults r = run_with_blocks({
      {"governor", {{"R", 0.037}}},
      {"exciter", {{"Ka", 42.0}}},
      {"pss", {{"Ks", 7.5}}},
  });
  REQUIRE(r.success);
  CHECK(device_value(r, "SynchronousMachine", 2, "composed_generator") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "generator_inner_var_count") ==
        Catch::Approx(9.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "inner_tau_m_present") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "inner_vf_present") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "inner_tau_e_present") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "block_machine") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "block_shaft") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "block_turbine_governor") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "block_avr") ==
        Catch::Approx(1.0));
  CHECK(device_value(r, "SynchronousMachine", 2, "block_pss") ==
        Catch::Approx(1.0));
}

TEST_CASE("Phase 3 inverter composition publishes compact block snapshots",
          "[dynamics][catalog]") {
  const DynamicResults gfl = run_with_vsc(ConverterMode::PQ_MODE);
  REQUIRE(gfl.success);
  CHECK(device_value(gfl, "VSCGridFollowing", "composed_inverter") ==
        Catch::Approx(1.0));
  CHECK(device_value(gfl, "VSCGridFollowing", "inverter_inner_var_count") ==
        Catch::Approx(25.0));
  CHECK(device_value(gfl, "VSCGridFollowing", "block_converter") ==
        Catch::Approx(1.0));
  CHECK(device_value(gfl, "VSCGridFollowing", "block_frequency_estimator") ==
        Catch::Approx(1.0));
  CHECK(device_value(gfl, "VSCGridFollowing", "inner_theta_pll_present") ==
        Catch::Approx(1.0));
  CHECK(device_value(gfl, "VSCGridFollowing", "inner_id_ic_present") ==
        Catch::Approx(1.0));

  const DynamicResults gfm = run_with_vsc(ConverterMode::AC_GRID_FORMING);
  REQUIRE(gfm.success);
  CHECK(device_value(gfm, "VSCGridForming", "composed_inverter") ==
        Catch::Approx(1.0));
  CHECK(device_value(gfm, "VSCGridForming", "inverter_grid_following") ==
        Catch::Approx(0.0));
  CHECK(device_value(gfm, "VSCGridForming", "block_outer_control") ==
        Catch::Approx(1.0));
  CHECK(device_value(gfm, "VSCGridForming", "inner_v_ref_present") ==
        Catch::Approx(1.0));
}

TEST_CASE("Phase 4 controller derivatives match PSID transfer blocks",
          "[dynamics][catalog][numerical]") {
  StateIndexRange machine_range{0, 8};
  MachineControlLink link = make_test_machine_link(machine_range);
  NetworkState y = single_bus_voltage(0.98);
  PowerFlowResult pf;
  pf.vm = {0.98};

  SECTION("AVRSimple") {
    ExciterDynamicParams p;
    p.model = ExciterModel::AVRSimple;
    p.model_name = "AVRSimple";
    p.bus_pos = 0;
    p.v_ref_pu = 1.03;
    p.kv = 25.0;
    Exciter avr(p);
    avr.attachMachine(&link);
    int offset = 8;
    avr.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[7] = 1.1;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    avr.computeDerivatives(0.0, x, y, dxdt);

    CHECK(dxdt[7] == Catch::Approx(25.0 * (1.03 - 0.98)));
  }

  SECTION("AVRTypeI") {
    ExciterDynamicParams p;
    p.model = ExciterModel::AVRTypeI;
    p.model_name = "AVRTypeI";
    p.bus_pos = 0;
    p.v_ref_pu = 1.04;
    p.ka = 40.0;
    p.ta_s = 0.2;
    p.ke = 1.1;
    p.kf = 0.04;
    p.tf_s = 0.5;
    p.tr_s = 0.1;
    p.te_s = 0.6;
    p.ae = 0.02;
    p.be = 0.4;
    Exciter avr(p);
    avr.attachMachine(&link);
    int offset = 8;
    avr.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[7] = 1.2;
    x.x[8] = 1.1;
    x.x[9] = 0.02;
    x.x[10] = 0.97;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    avr.computeDerivatives(0.0, x, y, dxdt);

    const double yhp = 0.02 + (0.04 / 0.5) * 1.2;
    const double se = 0.02 * std::exp(0.4 * 1.2);
    CHECK(dxdt[10] == Catch::Approx((0.98 - 0.97) / 0.1));
    CHECK(dxdt[9] == Catch::Approx(-((0.04 / 0.5) * 1.2 + 0.02) / 0.5));
    CHECK(dxdt[8] == Catch::Approx((40.0 * (1.04 - 0.97 - yhp) - 1.1) / 0.2));
    CHECK(dxdt[7] == Catch::Approx((1.1 - (1.1 + se) * 1.2) / 0.6));
  }

  SECTION("AVRTypeII") {
    ExciterDynamicParams p;
    p.model = ExciterModel::AVRTypeII;
    p.model_name = "AVRTypeII";
    p.bus_pos = 0;
    p.v_ref_pu = 1.05;
    p.k0 = 30.0;
    p.t1_s = 0.5;
    p.t2_s = 0.2;
    p.t3_s = 0.4;
    p.t4_s = 0.1;
    p.tr_s = 0.2;
    p.te_s = 0.8;
    p.ae = 0.01;
    p.be = 0.3;
    p.va_min_pu = -5.0;
    p.va_max_pu = 5.0;
    Exciter avr(p);
    avr.attachMachine(&link);
    int offset = 8;
    avr.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[7] = 1.1;
    x.x[8] = 0.4;
    x.x[9] = -0.02;
    x.x[10] = 0.99;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    avr.computeDerivatives(0.0, x, y, dxdt);

    const double u = 1.05 - 0.99;
    const double yll1 = 0.4 + 30.0 * (0.2 / 0.5) * u;
    const double dvr1 = (30.0 * (1.0 - 0.2 / 0.5) * u - 0.4) / 0.5;
    const double yll2 = 30.0 * -0.02 + (30.0 * 0.1) / (30.0 * 0.4) * yll1;
    const double dvr2 = ((1.0 - (30.0 * 0.1) / (30.0 * 0.4)) * yll1 -
                         30.0 * -0.02) /
                        (30.0 * 0.4);
    const double se = 0.01 * std::exp(0.3 * 1.1);
    CHECK(dxdt[10] == Catch::Approx((0.98 - 0.99) / 0.2));
    CHECK(dxdt[8] == Catch::Approx(dvr1));
    CHECK(dxdt[9] == Catch::Approx(dvr2));
    CHECK(dxdt[7] == Catch::Approx((yll2 - (1.0 + se) * 1.1) / 0.8));
  }

  SECTION("TGTypeI") {
    GovernorDynamicParams p;
    p.model = GovernorModel::TGTypeI;
    p.model_name = "TGTypeI";
    p.base_mva = 100.0;
    p.p_ref_mw = 80.0;
    p.droop_r = 0.05;
    p.t_s = 0.2;
    p.tc_s = 0.4;
    p.t3_s = 0.1;
    p.t4_s = 0.3;
    p.t5_s = 5.0;
    Governor gov(p);
    gov.attachMachine(&link);
    int offset = 8;
    gov.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[1] = 0.99;
    x.x[6] = 0.8;
    x.x[8] = 0.8;
    x.x[9] = 0.6;
    x.x[10] = 0.752;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    gov.computeDerivatives(0.0, x, y, dxdt);

    CHECK(dxdt[8] == Catch::Approx(1.0));
    CHECK(dxdt[9] == Catch::Approx(0.0).margin(1e-12));
    CHECK(dxdt[10] == Catch::Approx(0.0).margin(1e-12));
    CHECK(dxdt[6] == Catch::Approx(0.015));
  }

  SECTION("TGTypeII") {
    GovernorDynamicParams p;
    p.model = GovernorModel::TGTypeII;
    p.model_name = "TGTypeII";
    p.base_mva = 100.0;
    p.p_ref_mw = 75.0;
    p.droop_r = 0.05;
    p.t_s = 0.3;
    p.turbine_t_s = 0.6;
    Governor gov(p);
    gov.attachMachine(&link);
    int offset = 8;
    gov.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[1] = 0.98;
    x.x[6] = 0.82;
    x.x[8] = 0.1;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    gov.computeDerivatives(0.0, x, y, dxdt);

    const double input = (1.0 - 0.98) / 0.05;
    const double tau_delta = 0.1 + (0.3 / 0.6) * input;
    const double dxg = ((1.0 - 0.3 / 0.6) * input - 0.1) / 0.6;
    CHECK(dxdt[8] == Catch::Approx(dxg));
    CHECK(dxdt[6] == Catch::Approx(dxg + (0.75 + tau_delta - 0.82) / 0.6));
  }

  SECTION("STAB1") {
    PSSDynamicParams p;
    p.model = PSSModel::STAB1;
    p.model_name = "STAB1";
    p.kt = 4.0;
    p.stab_t_s = 5.0;
    p.t1_over_t3 = 2.0;
    p.t3_s = 0.4;
    p.t2_over_t4 = 0.5;
    p.t4_s = 0.2;
    p.h_lim = 0.1;
    PowerSystemStabilizer pss(p);
    pss.attachMachine(&link);
    int offset = 8;
    pss.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[1] = 1.01;
    x.x[8] = 0.02;
    x.x[9] = -0.01;
    x.x[10] = 0.005;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    pss.computeDerivatives(0.0, x, y, dxdt);
    const DynamicDeviceOutput out = pss.output(x, y);

    CHECK(dxdt[8] == Catch::Approx(-0.012));
    CHECK(dxdt[9] == Catch::Approx(-0.125));
    CHECK(dxdt[10] == Catch::Approx(0.25));
    CHECK(out.values.at("vs_pu") == Catch::Approx(0.06));
  }

  SECTION("IEEEST") {
    PSSDynamicParams p;
    p.model = PSSModel::IEEEST;
    p.model_name = "IEEEST";
    p.a1 = 0.2;
    p.a2 = 0.5;
    p.a3 = 0.3;
    p.a4 = 0.7;
    p.a5 = 0.8;
    p.a6 = 0.1;
    p.t1_s = 0.2;
    p.t2_s = 0.4;
    p.t3_s = 0.3;
    p.t4_s = 0.6;
    p.t5_s = 0.5;
    p.t6_s = 0.25;
    p.ks = 2.0;
    p.vs_max_pu = 10.0;
    p.vs_min_pu = -10.0;
    PowerSystemStabilizer pss(p);
    pss.attachMachine(&link);
    int offset = 8;
    pss.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[1] = 1.02;
    x.x[8] = 0.01;
    x.x[9] = 0.02;
    x.x[10] = -0.03;
    x.x[11] = 0.04;
    x.x[12] = 0.05;
    x.x[13] = -0.02;
    x.x[14] = 0.01;
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    pss.computeDerivatives(0.0, x, y, dxdt);

    const double u = 0.02;
    const double y_f = (0.1 / 0.5) * 0.02 +
                       (0.8 - 0.2 * 0.1 / 0.5) * -0.03 +
                       (1.0 - 0.1 / 0.5) * 0.04;
    const double y_ll1 = 0.05 + (0.2 / 0.4) * y_f;
    const double y_ll2 = -0.02 + (0.3 / 0.6) * y_ll1;
    CHECK(dxdt[8] == Catch::Approx((u - 0.3 * 0.01 - 0.02) / 0.7));
    CHECK(dxdt[9] == Catch::Approx(0.01));
    CHECK(dxdt[10] == Catch::Approx((0.02 - 0.2 * -0.03 - 0.04) / 0.5));
    CHECK(dxdt[11] == Catch::Approx(-0.03));
    CHECK(dxdt[12] == Catch::Approx(((1.0 - 0.2 / 0.4) * y_f - 0.05) / 0.4));
    CHECK(dxdt[13] == Catch::Approx(((1.0 - 0.3 / 0.6) * y_ll1 + 0.02) / 0.6));
    CHECK(dxdt[14] == Catch::Approx(-(((2.0 * 0.5) / 0.25) * y_ll2 + 0.01) / 0.25));
  }
}

TEST_CASE("Phase 5 five-mass shaft derivatives match torsional equations",
          "[dynamics][catalog][numerical]") {
  StateIndexRange machine_range{0, 8};
  MachineControlLink link = make_test_machine_link(machine_range);
  link.inertia_h = 3.0;

  FiveMassShaftParams p;
  p.component_index = 2;
  p.machine_index = 2;
  p.frequency_hz = 50.0;
  p.inertia_h = {{0.5, 0.6, 0.7, 0.8, 1.0}};
  p.stiffness_pu = {{20.0, 18.0, 16.0, 14.0}};
  p.damping_pu = {{0.0, 0.0, 0.0, 0.0}};
  FiveMassShaft shaft(p);
  shaft.attachMachine(&link);

  int offset = 8;
  shaft.assignStateIndices(offset);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  x.x[0] = 0.15;  // machine rotor angle
  x.x[1] = 1.0;   // machine speed
  x.x[6] = 0.8;   // machine-owned mechanical torque
  NetworkState y = single_bus_voltage(1.0);
  PowerFlowResult pf;
  shaft.initializeFromPowerFlow(pf, x, y);

  Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
  shaft.computeDerivatives(0.0, x, y, dxdt);
  CHECK(dxdt.segment(8, 10).lpNorm<Eigen::Infinity>() ==
        Catch::Approx(0.0).margin(1e-12));

  x.x[8] += 0.01;
  dxdt.setZero();
  shaft.computeDerivatives(0.0, x, y, dxdt);
  CHECK(dxdt[13] == Catch::Approx(-(20.0 * 0.01) / (2.0 * 0.5)));
  CHECK(dxdt[14] == Catch::Approx((20.0 * 0.01) / (2.0 * 0.6)));
  CHECK(dxdt[1] == Catch::Approx(0.0).margin(1e-12));
}
