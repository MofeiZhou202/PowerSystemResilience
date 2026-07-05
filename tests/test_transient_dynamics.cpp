#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/dynamics.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

using namespace hacdcpf;
using namespace hacdcpf::dynamics;

namespace {

using Complex = std::complex<double>;
using Json = nlohmann::json;

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

HybridPowerSystem materialize_canvas_loads_from_bus_demand(HybridPowerSystem sys) {
  sys.name = "Canvas System";
  if (!sys.ac.loads.empty()) return sys;
  for (const auto& bus : sys.ac.buses) {
    if (!bus.in_service) continue;
    if (std::abs(bus.pd_mw) < 1e-12 && std::abs(bus.qd_mvar) < 1e-12) continue;
    Load load;
    load.index = bus.index;
    load.bus = bus.index;
    load.name = "Load " + std::to_string(bus.index);
    load.p_mw = bus.pd_mw;
    load.q_mvar = bus.qd_mvar;
    load.scaling = 1.0;
    load.model = LoadModel::ConstantPower;
    load.in_service = true;
    sys.ac.loads.push_back(load);
  }
  for (auto& bus : sys.ac.buses) {
    bus.pd_mw = 0.0;
    bus.qd_mvar = 0.0;
  }
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

HybridPowerSystem make_psd_genrou_three_bus_subset_case() {
  HybridPowerSystem sys;
  sys.name = "psd_genrou_three_bus_subset";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 60.0;

  ACBus b101;
  b101.index = 101;
  b101.name = "BUS 1";
  b101.bus_type = BusType::SLACK;
  b101.base_kv = 138.0;
  b101.vm_pu = 1.05;
  b101.va_deg = 0.0;
  b101.in_service = true;

  ACBus b102;
  b102.index = 102;
  b102.name = "BUS 2";
  b102.bus_type = BusType::PV;
  b102.base_kv = 138.0;
  b102.vm_pu = 1.02;
  b102.va_deg = -0.9440;
  b102.in_service = true;

  ACBus b103;
  b103.index = 103;
  b103.name = "BUS 3";
  b103.bus_type = BusType::PQ;
  b103.base_kv = 138.0;
  b103.vm_pu = 0.99341;
  b103.va_deg = -8.7697;
  b103.in_service = true;

  sys.ac.buses = {b101, b102, b103};

  auto branch = [](int index, int from, int to) {
    ACBranch br;
    br.index = index;
    br.from_bus = from;
    br.to_bus = to;
    br.r_pu = 0.01;
    br.x_pu = 0.12;
    br.tap = 1.0;
    br.in_service = true;
    br.name = "BUS " + std::to_string(from - 100) + "-BUS " +
              std::to_string(to - 100) + "-i_1";
    return br;
  };
  sys.ac.branches = {
      branch(1, 101, 102),
      branch(2, 101, 103),
      branch(3, 102, 103),
  };

  Generator slack;
  slack.index = 1;
  slack.bus = 101;
  slack.name = "generator-101-1";
  slack.is_slack = true;
  slack.in_service = true;
  slack.pg_mw = 153.335;
  slack.qg_mvar = 73.271;
  slack.vg_pu = 1.05;
  slack.pmax_mw = 318.0;
  slack.pmin_mw = 0.0;
  slack.qmax_mvar = 100.0;
  slack.qmin_mvar = -100.0;
  slack.xdpp_pu = 1.0e-5;
  slack.dynamic_model.standard = "PSS/E";
  slack.dynamic_model.model_name = "GENCLS";
  slack.dynamic_model.source_id = "PowerSimulationsDynamics:test_case15_genrou";

  Generator genrou;
  genrou.index = 2;
  genrou.bus = 102;
  genrou.name = "generator-102-1";
  genrou.is_slack = false;
  genrou.in_service = true;
  genrou.pg_mw = 100.0;
  genrou.qg_mvar = -3.247;
  genrou.vg_pu = 1.02;
  genrou.pmax_mw = 318.0;
  genrou.pmin_mw = 0.0;
  genrou.qmax_mvar = 100.0;
  genrou.qmin_mvar = -100.0;
  genrou.ra_pu = 0.0;
  genrou.xd_pu = 1.8;
  genrou.xq_pu = 1.7;
  genrou.xdp_pu = 0.30;
  genrou.xdpp_pu = 0.25;
  genrou.td0p_s = 8.0;
  genrou.td0pp_s = 0.03;
  genrou.inertia_h = 6.175;
  genrou.droop_r = 0.05;
  genrou.dynamic_model.standard = "PSS/E";
  genrou.dynamic_model.model_name = "GENROU";
  genrou.dynamic_model.source_id = "PowerSimulationsDynamics:test_case15_genrou";
  genrou.dynamic_model.parameters = {
      {"H", 6.175},
      {"D", 0.05},
      {"Xd", 1.8},
      {"Xq", 1.7},
      {"Xd_p", 0.30},
      {"Xq_p", 0.55},
      {"Xd_pp", 0.25},
      {"Xl", 0.20},
      {"Td0_p", 8.0},
      {"Td0_pp", 0.03},
      {"Tq0_p", 0.4},
      {"Tq0_pp", 0.05},
      {"Sat_A", 0.904688681931025},
      {"Sat_B", 11.008066615170353},
  };

  sys.ac.generators = {slack, genrou};

  Load load;
  load.index = 1;
  load.bus = 103;
  load.name = "load1031";
  load.p_mw = 250.0;
  load.q_mvar = 30.0;
  load.in_service = true;
  load.dynamic_model.standard = "PSS/E";
  load.dynamic_model.model_name = "ConstantImpedanceLoad";
  load.dynamic_model.source_id = "PowerSimulationsDynamics:test_case15_genrou";
  sys.ac.loads = {load};

  return sys;
}

HybridPowerSystem make_psd_onedoneq_three_bus_subset_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_onedoneq_three_bus_subset";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.standard = "PowerSystems";
  machine.dynamic_model.model_name = "OneDOneQMachine";
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test_case02_onedoneq";
  machine.ra_pu = 0.0;
  machine.xd_pu = 1.3125;
  machine.xq_pu = 1.2578;
  machine.xdp_pu = 0.1813;
  machine.xdpp_pu = 0.0;
  machine.td0p_s = 5.89;
  machine.td0pp_s = 0.0;
  machine.inertia_h = 3.01;
  machine.dynamic_model.parameters = {
      {"H", 3.01},
      {"D", 0.0},
      {"R", 0.0},
      {"Xd", 1.3125},
      {"Xq", 1.2578},
      {"Xd_p", 0.1813},
      {"Xq_p", 0.25},
      {"Td0_p", 5.89},
      {"Tq0_p", 0.6},
  };
  return sys;
}

void apply_psd_simple_marconato_profile(Generator& machine) {
  machine.ra_pu = 0.0;
  machine.xd_pu = 1.3125;
  machine.xq_pu = 1.2578;
  machine.xdp_pu = 0.1813;
  machine.xdpp_pu = 0.14;
  machine.td0p_s = 5.89;
  machine.td0pp_s = 0.5;
  machine.inertia_h = 3.01;
  machine.droop_r = 0.05;
  machine.dynamic_model.standard = "PowerSystems";
  machine.dynamic_model.model_name = "SimpleMarconatoMachine";
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test_case03_simple_marconato";
  machine.dynamic_model.parameters = {
      {"H", 3.01},
      {"D", 0.0},
      {"R", 0.0},
      {"Xd", 1.3125},
      {"Xq", 1.2578},
      {"Xd_p", 0.1813},
      {"Xq_p", 0.25},
      {"Xd_pp", 0.14},
      {"Xq_pp", 0.18},
      {"Td0_p", 5.89},
      {"Tq0_p", 0.6},
      {"Td0_pp", 0.5},
      {"Tq0_pp", 0.023},
      {"T_AA", 0.0},
  };

  hacdcpf::DynamicModelComponentProfile avr;
  avr.type = "exciter";
  avr.model = "AVRTypeI";
  avr.standard = "PowerSystems";
  avr.parameter_set = "PowerSimulationsDynamics:test_case03_simple_marconato";
  avr.parameters = {
      {"Ka", 20.0},
      {"Ke", 0.01},
      {"Kf", 0.063},
      {"Ta", 0.2},
      {"Te", 0.314},
      {"Tf", 0.35},
      {"Tr", 0.001},
      {"Va_min", -5.0},
      {"Va_max", 5.0},
      {"Ae", 0.0039},
      {"Be", 1.555},
  };
  machine.dynamic_model.components.push_back(std::move(avr));
}

HybridPowerSystem make_psd_simple_marconato_three_bus_case() {
  HybridPowerSystem sys;
  sys.name = "psd_simple_marconato_three_bus";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 60.0;

  ACBus b101;
  b101.index = 101;
  b101.name = "BUS 1";
  b101.bus_type = BusType::SLACK;
  b101.base_kv = 138.0;
  b101.vm_pu = 1.02;
  b101.va_deg = 0.0;
  b101.in_service = true;

  ACBus b102;
  b102.index = 102;
  b102.name = "BUS 2";
  b102.bus_type = BusType::PV;
  b102.base_kv = 138.0;
  b102.vm_pu = 1.0142;
  b102.va_deg = 0.0;
  b102.in_service = true;

  ACBus b103;
  b103.index = 103;
  b103.name = "BUS 3";
  b103.bus_type = BusType::PV;
  b103.base_kv = 138.0;
  b103.vm_pu = 1.0059;
  b103.va_deg = 0.0;
  b103.in_service = true;

  sys.ac.buses = {b101, b102, b103};

  auto branch = [](int index, int from, int to) {
    ACBranch br;
    br.index = index;
    br.from_bus = from;
    br.to_bus = to;
    br.r_pu = 0.01;
    br.x_pu = 0.12;
    br.tap = 1.0;
    br.in_service = true;
    br.name = "BUS " + std::to_string(from - 100) + "-BUS " +
              std::to_string(to - 100) + "-i_1";
    return br;
  };
  sys.ac.branches = {
      branch(1, 101, 103),
      branch(2, 101, 102),
      branch(3, 102, 103),
  };

  ExternalGrid source;
  source.index = 1;
  source.bus = 101;
  source.name = "InfBus";
  source.in_service = true;
  source.vm_pu = 1.02;
  source.va_deg = 0.0;
  source.r_pu = 0.0;
  source.x_pu = 5.0e-6;
  sys.ac.external_grids = {source};

  Generator g102;
  g102.index = 2;
  g102.bus = 102;
  g102.name = "generator-102-1";
  g102.is_slack = false;
  g102.in_service = true;
  g102.pg_mw = 100.0;
  g102.qg_mvar = 79.44866697732391;
  g102.vg_pu = 1.0142;
  g102.pmax_mw = 318.0;
  g102.pmin_mw = 0.0;
  g102.qmax_mvar = 100.0;
  g102.qmin_mvar = -100.0;
  apply_psd_simple_marconato_profile(g102);

  Generator g103;
  g103.index = 3;
  g103.bus = 103;
  g103.name = "generator-103-1";
  g103.is_slack = false;
  g103.in_service = true;
  g103.pg_mw = 100.0;
  g103.qg_mvar = 8.108131399904096;
  g103.vg_pu = 1.0059;
  g103.pmax_mw = 318.0;
  g103.pmin_mw = 0.0;
  g103.qmax_mvar = 100.0;
  g103.qmin_mvar = -100.0;
  apply_psd_simple_marconato_profile(g103);

  sys.ac.generators = {g102, g103};

  auto load = [](int index, int bus, double p_mw, double q_mvar) {
    Load ld;
    ld.index = index;
    ld.bus = bus;
    ld.name = "load" + std::to_string(bus) + "1";
    ld.p_mw = p_mw;
    ld.q_mvar = q_mvar;
    ld.in_service = true;
    ld.dynamic_model.standard = "PowerSystems";
    ld.dynamic_model.model_name = "ConstantImpedanceLoad";
    ld.dynamic_model.source_id = "PowerSimulationsDynamics:test_case03_simple_marconato";
    ld.dynamic_model.parameters["phase_power_scale"] = 1.0;
    return ld;
  };
  sys.ac.loads = {
      load(1, 101, 150.0, 80.0),
      load(2, 102, 170.0, 70.0),
      load(3, 103, 50.0, 30.0),
  };

  return sys;
}

HybridPowerSystem make_psd_genroe_three_bus_subset_case(bool high_saturation = false) {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = high_saturation ? "psd_genroe_high_sat_three_bus_subset"
                             : "psd_genroe_three_bus_subset";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.standard = "PSS/E";
  machine.dynamic_model.model_name = "GENROE";
  machine.dynamic_model.source_id = high_saturation
                                        ? "PowerSimulationsDynamics:test_case16_genroe_high_sat"
                                        : "PowerSimulationsDynamics:test_case16_genroe";
  machine.ra_pu = 0.0;
  machine.xd_pu = 1.8;
  machine.xq_pu = 1.7;
  machine.xdp_pu = 0.30;
  machine.xdpp_pu = 0.25;
  machine.td0p_s = 8.0;
  machine.td0pp_s = 0.03;
  machine.inertia_h = 6.175;
  machine.dynamic_model.parameters = {
      {"H", 6.175},
      {"D", 0.05},
      {"R", 0.0},
      {"Xd", 1.8},
      {"Xq", 1.7},
      {"Xd_p", 0.30},
      {"Xq_p", 0.55},
      {"Xd_pp", 0.25},
      {"Xl", 0.20},
      {"Td0_p", 8.0},
      {"Td0_pp", 0.03},
      {"Tq0_p", 0.4},
      {"Tq0_pp", 0.05},
      {"Sat_A", high_saturation ? 16.431037153437266 : 10.52711883672175},
      {"Sat_B", high_saturation ? 0.04 : 0.0392},
  };
  return sys;
}

HybridPowerSystem make_psd_gensal_three_bus_subset_case(bool exponential) {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = exponential ? "psd_gensae_three_bus_subset" : "psd_gensal_three_bus_subset";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.standard = "PSS/E";
  machine.dynamic_model.model_name = exponential ? "GENSAE" : "GENSAL";
  machine.dynamic_model.source_id = exponential ? "PowerSimulationsDynamics:test_case19_gensae"
                                                : "PowerSimulationsDynamics:test_case18_gensal";
  machine.ra_pu = 0.0;
  machine.xd_pu = 1.0;
  machine.xq_pu = 0.75;
  machine.xdp_pu = 0.40;
  machine.xdpp_pu = 0.25;
  machine.td0p_s = 5.0;
  machine.td0pp_s = 0.05;
  machine.inertia_h = 5.0;
  machine.dynamic_model.parameters = {
      {"H", 5.0},
      {"D", 0.0},
      {"R", 0.0},
      {"Xd", 1.0},
      {"Xq", 0.75},
      {"Xd_p", 0.40},
      {"Xd_pp", 0.25},
      {"Xl", 0.10},
      {"Td0_p", 5.0},
      {"Td0_pp", 0.05},
      {"Tq0_pp", 0.20},
      {"Sat_A", exponential ? 9.484556531079702 : 0.8750546016608771},
      {"Sat_B", exponential ? 0.11 : 7.046154363249024},
  };
  return sys;
}

HybridPowerSystem make_psd_zip_load_three_bus_subset_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_zip_load_three_bus_subset";
  sys.ac.loads.clear();

  Load l102;
  l102.index = 1;
  l102.bus = 102;
  l102.name = "load1021";
  l102.p_mw = 50.0;
  l102.q_mvar = 30.0;
  l102.in_service = true;
  l102.dynamic_model.standard = "PSS/E";
  l102.dynamic_model.model_name = "ConstantPowerLoad";
  l102.dynamic_model.source_id = "PowerSimulationsDynamics:test_case33_zip_load";

  Load l103a;
  l103a.index = 2;
  l103a.bus = 103;
  l103a.name = "load1031";
  l103a.p_mw = 150.0;
  l103a.q_mvar = 30.0;
  l103a.in_service = true;
  l103a.dynamic_model = l102.dynamic_model;

  Load l103b;
  l103b.index = 3;
  l103b.bus = 103;
  l103b.name = "load1032";
  l103b.p_mw = 50.0;
  l103b.q_mvar = 30.0;
  l103b.in_service = true;
  l103b.dynamic_model = l102.dynamic_model;

  sys.ac.loads = {l102, l103a, l103b};
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

void set_balanced_voltage(NetworkState& y, int bus_pos, Complex v) {
  constexpr double pi = 3.141592653589793238462643383279502884;
  const int base = 3 * bus_pos;
  REQUIRE(base + 2 < y.Vac_abc.size());
  const double vm = std::abs(v);
  const double va = std::arg(v);
  y.Vac_abc[base + 0] = std::polar(vm, va);
  y.Vac_abc[base + 1] = std::polar(vm, va - 2.0 * pi / 3.0);
  y.Vac_abc[base + 2] = std::polar(vm, va + 2.0 * pi / 3.0);
}

double vsc_p_mw(const DynamicSnapshot& snapshot) {
  const auto& outputs = snapshot.device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  REQUIRE(it->values.count("p_mw") == 1);
  return it->values.at("p_mw");
}

double vsc_value(const DynamicSnapshot& snapshot, const std::string& key) {
  const auto& outputs = snapshot.device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  REQUIRE(it->values.count(key) == 1);
  return it->values.at(key);
}

double final_vsc_p_mw(const DynamicResults& result) {
  REQUIRE(result.final_snapshot() != nullptr);
  return vsc_p_mw(*result.final_snapshot());
}

struct CsvSeries {
  std::vector<double> t;
  std::vector<double> y;
};

std::string shell_quote(const std::string& value) {
  std::string out = "'";
  for (char ch : value) {
    if (ch == '\'') {
      out += "'\\''";
    } else {
      out += ch;
    }
  }
  out += "'";
  return out;
}

std::string safe_artifact_token(std::string value) {
  for (char& ch : value) {
    if (!(std::isalnum(static_cast<unsigned char>(ch)) || ch == '-' || ch == '_')) {
      ch = '_';
    }
  }
  return value;
}

std::filesystem::path psd_repo_path() {
  if (const char* env = std::getenv("HACDCPF_PSD_REPO")) {
    return env;
  }
  return "/Users/tianyangzhao/Codes/PowerSimulationsDynamics.jl";
}

std::string julia_bin() {
  if (const char* env = std::getenv("HACDCPF_JULIA_BIN")) {
    return env;
  }
  return "julia";
}

CsvSeries read_csv_series(const std::filesystem::path& path) {
  CsvSeries series;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    double t = 0.0;
    double y = 0.0;
    if (iss >> t >> y) {
      series.t.push_back(t);
      series.y.push_back(y);
    }
  }
  return series;
}

void write_csv_series(const std::filesystem::path& path,
                      const CsvSeries& series) {
  REQUIRE(series.t.size() == series.y.size());
  std::ofstream out(path);
  REQUIRE(out.good());
  for (std::size_t i = 0; i < series.t.size(); ++i) {
    out << series.t[i] << "," << series.y[i] << "\n";
  }
}

double interpolate_series(const CsvSeries& series, double t) {
  REQUIRE(series.t.size() >= 2);
  if (t <= series.t.front()) return series.y.front();
  if (t >= series.t.back()) return series.y.back();
  const auto hi = std::lower_bound(series.t.begin(), series.t.end(), t);
  const std::size_t i = static_cast<std::size_t>(std::distance(series.t.begin(), hi));
  REQUIRE(i > 0);
  const double t0 = series.t[i - 1];
  const double t1 = series.t[i];
  const double y0 = series.y[i - 1];
  const double y1 = series.y[i];
  const double a = (t - t0) / std::max(1e-12, t1 - t0);
  return y0 + a * (y1 - y0);
}

double rms_common_error(const CsvSeries& lhs,
                        const CsvSeries& rhs,
                        double t_start,
                        double t_end) {
  double sum_sq = 0.0;
  std::size_t n = 0;
  for (double t : lhs.t) {
    if (t < t_start - 1e-12 || t > t_end + 1e-12) continue;
    const double e = interpolate_series(lhs, t) - interpolate_series(rhs, t);
    sum_sq += e * e;
    ++n;
  }
  REQUIRE(n > 0);
  return std::sqrt(sum_sq / static_cast<double>(n));
}

double max_common_abs_error(const CsvSeries& lhs,
                            const CsvSeries& rhs,
                            double t_start,
                            double t_end) {
  double max_abs = 0.0;
  std::size_t n = 0;
  for (double t : lhs.t) {
    if (t < t_start - 1e-12 || t > t_end + 1e-12) continue;
    const double e = interpolate_series(lhs, t) - interpolate_series(rhs, t);
    max_abs = std::max(max_abs, std::abs(e));
    ++n;
  }
  REQUIRE(n > 0);
  return max_abs;
}

CsvSeries relative_to_initial(CsvSeries series) {
  if (series.y.empty()) return series;
  const double y0 = series.y.front();
  for (double& value : series.y) value -= y0;
  return series;
}

const DynamicDeviceOutput& require_device_output(const DynamicSnapshot& snapshot,
                                                 const std::string& type,
                                                 int component_index = 0) {
  const auto& outputs = snapshot.device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [&](const DynamicDeviceOutput& out) {
    return out.type == type &&
           (component_index == 0 || out.component_index == component_index);
  });
  REQUIRE(it != outputs.end());
  return *it;
}

CsvSeries device_output_series(const DynamicResults& result,
                               const std::string& type,
                               int component_index,
                               const std::string& key,
                               double scale = 1.0) {
  CsvSeries series;
  for (const auto& snapshot : result.snapshots) {
    const auto& out = require_device_output(snapshot, type, component_index);
    REQUIRE(out.values.count(key) == 1);
    series.t.push_back(snapshot.time_s);
    series.y.push_back(out.values.at(key) * scale);
  }
  return series;
}

CsvSeries bus_voltage_mag_series(const DynamicResults& result, int bus_position) {
  CsvSeries series;
  for (const auto& snapshot : result.snapshots) {
    const int node = 3 * bus_position;
    REQUIRE(node >= 0);
    REQUIRE(node < snapshot.vac_abc.size());
    series.t.push_back(snapshot.time_s);
    series.y.push_back(std::abs(snapshot.vac_abc[node]));
  }
  return series;
}

double series_range(const CsvSeries& series) {
  REQUIRE_FALSE(series.y.empty());
  const auto [min_it, max_it] = std::minmax_element(series.y.begin(), series.y.end());
  return *max_it - *min_it;
}

struct ValidationMetric {
  std::string level;
  std::string case_name;
  std::string reference;
  std::string signal;
  double rms_error{0.0};
  double max_abs_error{0.0};
  double rms_tolerance{0.0};
  double max_tolerance{0.0};
  bool passed{false};
};

void write_validation_summary(const std::filesystem::path& path,
                              const std::vector<ValidationMetric>& metrics) {
  std::ofstream out(path);
  REQUIRE(out.good());
  out << "level,case,reference,signal,rms_error,max_abs_error,"
         "rms_tolerance,max_tolerance,passed\n";
  for (const auto& metric : metrics) {
    out << metric.level << ","
        << metric.case_name << ","
        << metric.reference << ","
        << metric.signal << ","
        << metric.rms_error << ","
        << metric.max_abs_error << ","
        << metric.rms_tolerance << ","
        << metric.max_tolerance << ","
        << (metric.passed ? "true" : "false") << "\n";
  }
}

void append_pointwise_comparison_rows(std::ostream& out,
                                      const std::string& level,
                                      const std::string& case_name,
                                      const std::string& signal,
                                      const CsvSeries& local,
                                      const CsvSeries& psd,
                                      double t_start,
                                      double t_end,
                                      bool compare_relative) {
  REQUIRE_FALSE(local.t.empty());
  REQUIRE_FALSE(local.y.empty());
  REQUIRE_FALSE(psd.t.empty());
  REQUIRE_FALSE(psd.y.empty());
  const double local0 = local.y.front();
  const double psd0 = psd.y.front();
  const char* alignment = compare_relative ? "relative_to_initial" : "absolute";
  out << std::setprecision(17);
  for (double t : local.t) {
    if (t < t_start - 1e-12 || t > t_end + 1e-12) continue;
    const double local_raw = interpolate_series(local, t);
    const double psd_raw = interpolate_series(psd, t);
    const double local_cmp = compare_relative ? local_raw - local0 : local_raw;
    const double psd_cmp = compare_relative ? psd_raw - psd0 : psd_raw;
    const double error = local_cmp - psd_cmp;
    out << level << ","
        << case_name << ","
        << signal << ","
        << alignment << ","
        << t << ","
        << local_raw << ","
        << psd_raw << ","
        << local_cmp << ","
        << psd_cmp << ","
        << error << ","
        << std::abs(error) << "\n";
  }
}

struct PsdGflCase {
  std::string name;
  std::string psd_case;
  std::string psd_test_file;
  std::string pll_model;
  double pll_kp{0.0};
  double pll_ki{0.0};
  double pll_lpf_t_s{0.0};
};

DynamicResults run_hacdcpf_psd_gfl_case(const PsdGflCase& spec,
                                        double base_mva) {
  auto sys = make_hybrid_dc_case();
  sys.base_mva = base_mva;
  sys.ac.base_mva = base_mva;
  sys.vsc_converters[0].p_set_mw = 50.0;
  sys.vsc_converters[0].p_schedule_mw = 50.0;
  sys.vsc_converters[0].q_set_mvar = 0.0;
  sys.vsc_converters[0].dynamic_model.standard = "NERC";
  sys.vsc_converters[0].dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  sys.vsc_converters[0].dynamic_model.components.push_back(
      {"pll", spec.pll_model, "PowerSimulationsDynamics", spec.psd_case,
       {{"kp_pll", spec.pll_kp},
        {"ki_pll", spec.pll_ki},
        {"pll_lpf_t_s", spec.pll_lpf_t_s}}});

  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent pref_step;
  pref_step.time_s = 1.0;
  pref_step.type = DynamicEventType::Custom;
  pref_step.component_type = "VSC";
  pref_step.component_index = 1;
  pref_step.label = "PSD active-power reference step";
  pref_step.params["p_ref_mw"] = 70.0;
  dyn.events.push_back(pref_step);

  DynamicSolver solver;
  return solver.solve(dyn);
}

CsvSeries hacdcpf_vsc_filtered_power_series(const DynamicResults& result,
                                            double base_mva) {
  CsvSeries local;
  for (const auto& snapshot : result.snapshots) {
    local.t.push_back(snapshot.time_s);
    local.y.push_back(vsc_value(snapshot, "p_filtered_mw") / base_mva);
  }
  return local;
}

struct PsdTraceExportRequest {
  std::string case_name;
  std::string signal;
  std::filesystem::path out_csv;
};

bool export_psd_traces_batch(const std::vector<PsdTraceExportRequest>& requests,
                             const std::filesystem::path& log_file) {
  if (requests.empty()) return true;

  const std::filesystem::path repo = psd_repo_path();
  const std::filesystem::path script =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) /
      "tools" / "psd_validation" / "export_trace.jl";

  INFO("PSD repo: " << repo);
  INFO("PSD generic exporter: " << script);
  INFO("PSD batch request count: " << requests.size());
  INFO("PSD batch log: " << log_file);
  REQUIRE(std::filesystem::exists(repo / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / "Project.toml"));
  REQUIRE(std::filesystem::exists(script));

  std::string command =
      "cd " + shell_quote(repo.string()) + " && " +
      shell_quote(julia_bin()) + " --project=" +
      shell_quote((repo / "test").string()) + " " +
      shell_quote(script.string()) + " " +
      shell_quote(repo.string()) + " --batch";
  for (const auto& request : requests) {
    command += " " + shell_quote(request.case_name + "|" + request.signal +
                                 "=" + request.out_csv.string());
  }
  command += " > " + shell_quote(log_file.string()) + " 2>&1";
  return std::system(command.c_str()) == 0;
}

struct PsdInternalDiagnostics {
  std::map<std::string, std::string> metadata;
  std::vector<std::string> labels;
  Eigen::VectorXd state;
  Eigen::VectorXd residual;
  Eigen::VectorXd mass_diag;
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd reduced_jacobian;
  std::vector<Complex> eigenvalues;
};

bool export_psd_internal_diagnostics(const std::string& case_name,
                                     const std::filesystem::path& out_dir,
                                     const std::filesystem::path& log_file) {
  const std::filesystem::path repo = psd_repo_path();
  const std::filesystem::path script =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) /
      "tools" / "psd_validation" / "export_internal_diagnostics.jl";

  INFO("PSD repo: " << repo);
  INFO("PSD internal exporter: " << script);
  INFO("PSD internal output dir: " << out_dir);
  INFO("PSD internal log: " << log_file);
  REQUIRE(std::filesystem::exists(repo / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / "Project.toml"));
  REQUIRE(std::filesystem::exists(script));
  std::filesystem::create_directories(out_dir);

  const std::string command =
      "cd " + shell_quote(repo.string()) + " && " +
      shell_quote(julia_bin()) + " --project=" +
      shell_quote((repo / "test").string()) + " " +
      shell_quote(script.string()) + " " +
      shell_quote(repo.string()) + " " +
      shell_quote(case_name) + " " +
      shell_quote(out_dir.string()) + " > " +
      shell_quote(log_file.string()) + " 2>&1";
  return std::system(command.c_str()) == 0;
}

std::map<std::string, std::string> read_key_value_csv(
    const std::filesystem::path& path) {
  std::map<std::string, std::string> values;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (first) {
      first = false;
      if (line.rfind("key,", 0) == 0) continue;
    }
    const auto comma = line.find(',');
    REQUIRE(comma != std::string::npos);
    values[line.substr(0, comma)] = line.substr(comma + 1);
  }
  return values;
}

double metadata_double(const std::map<std::string, std::string>& metadata,
                       const std::string& key) {
  const auto it = metadata.find(key);
  REQUIRE(it != metadata.end());
  return std::stod(it->second);
}

int metadata_int(const std::map<std::string, std::string>& metadata,
                 const std::string& key) {
  return static_cast<int>(std::llround(metadata_double(metadata, key)));
}

Eigen::VectorXd read_index_value_vector(const std::filesystem::path& path) {
  std::vector<double> values;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (first) {
      first = false;
      if (line.rfind("index,", 0) == 0) continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    int index = 0;
    double value = 0.0;
    REQUIRE(iss >> index >> value);
    values.push_back(value);
  }
  Eigen::VectorXd out(static_cast<Eigen::Index>(values.size()));
  for (Eigen::Index i = 0; i < out.size(); ++i) {
    out[i] = values[static_cast<std::size_t>(i)];
  }
  return out;
}

Eigen::MatrixXd read_numeric_matrix_csv(const std::filesystem::path& path) {
  std::vector<std::vector<double>> rows;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    std::vector<double> row;
    double value = 0.0;
    while (iss >> value) row.push_back(value);
    if (!row.empty()) rows.push_back(std::move(row));
  }
  REQUIRE_FALSE(rows.empty());
  const std::size_t cols = rows.front().size();
  REQUIRE(cols > 0);
  Eigen::MatrixXd out(static_cast<Eigen::Index>(rows.size()),
                      static_cast<Eigen::Index>(cols));
  for (std::size_t r = 0; r < rows.size(); ++r) {
    REQUIRE(rows[r].size() == cols);
    for (std::size_t c = 0; c < cols; ++c) {
      out(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
          rows[r][c];
    }
  }
  return out;
}

std::vector<Complex> read_eigenvalue_csv(const std::filesystem::path& path) {
  std::vector<Complex> values;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (first) {
      first = false;
      if (line.rfind("index,", 0) == 0) continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    int index = 0;
    double real = 0.0;
    double imag = 0.0;
    REQUIRE(iss >> index >> real >> imag);
    values.emplace_back(real, imag);
  }
  return values;
}

std::vector<std::string> read_state_labels(const std::filesystem::path& path) {
  std::vector<std::string> labels;
  std::ifstream in(path);
  REQUIRE(in.good());
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (first) {
      first = false;
      if (line.rfind("index,", 0) == 0) continue;
    }
    const auto first_comma = line.find(',');
    REQUIRE(first_comma != std::string::npos);
    const auto second_comma = line.find(',', first_comma + 1);
    REQUIRE(second_comma != std::string::npos);
    labels.push_back(line.substr(first_comma + 1,
                                 second_comma - first_comma - 1));
  }
  return labels;
}

PsdInternalDiagnostics read_psd_internal_diagnostics(
    const std::filesystem::path& dir) {
  PsdInternalDiagnostics diagnostics;
  diagnostics.metadata = read_key_value_csv(dir / "metadata.csv");
  diagnostics.labels = read_state_labels(dir / "state_table.csv");
  diagnostics.residual = read_index_value_vector(dir / "residual.csv");
  diagnostics.mass_diag = read_index_value_vector(dir / "mass_diag.csv");
  diagnostics.jacobian = read_numeric_matrix_csv(dir / "jacobian.csv");
  diagnostics.reduced_jacobian =
      read_numeric_matrix_csv(dir / "reduced_jacobian.csv");
  diagnostics.eigenvalues = read_eigenvalue_csv(dir / "eigenvalues.csv");

  std::ifstream in(dir / "state_table.csv");
  REQUIRE(in.good());
  std::vector<double> state_values;
  std::string line;
  bool first = true;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    if (first) {
      first = false;
      continue;
    }
    std::vector<std::string> fields;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) fields.push_back(field);
    REQUIRE(fields.size() >= 6);
    state_values.push_back(std::stod(fields[2]));
  }
  diagnostics.state.resize(static_cast<Eigen::Index>(state_values.size()));
  for (Eigen::Index i = 0; i < diagnostics.state.size(); ++i) {
    diagnostics.state[i] = state_values[static_cast<std::size_t>(i)];
  }
  return diagnostics;
}

double max_abs_diff(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
  REQUIRE(lhs.size() == rhs.size());
  return lhs.size() == 0 ? 0.0 : (lhs - rhs).cwiseAbs().maxCoeff();
}

double relative_frobenius_error(const Eigen::MatrixXd& lhs,
                                const Eigen::MatrixXd& rhs) {
  REQUIRE(lhs.rows() == rhs.rows());
  REQUIRE(lhs.cols() == rhs.cols());
  const double denom = std::max(1.0, rhs.norm());
  return (lhs - rhs).norm() / denom;
}

double max_abs_matrix_diff(const Eigen::MatrixXd& lhs,
                           const Eigen::MatrixXd& rhs) {
  REQUIRE(lhs.rows() == rhs.rows());
  REQUIRE(lhs.cols() == rhs.cols());
  return lhs.size() == 0 ? 0.0 : (lhs - rhs).cwiseAbs().maxCoeff();
}

double nearest_eigenvalue_max_error(std::vector<Complex> lhs,
                                    const std::vector<Complex>& rhs) {
  REQUIRE(lhs.size() == rhs.size());
  double worst = 0.0;
  std::vector<bool> used(lhs.size(), false);
  for (const Complex target : rhs) {
    double best = std::numeric_limits<double>::infinity();
    std::size_t best_i = lhs.size();
    for (std::size_t i = 0; i < lhs.size(); ++i) {
      if (used[i]) continue;
      const double err = std::abs(lhs[i] - target);
      if (err < best) {
        best = err;
        best_i = i;
      }
    }
    REQUIRE(best_i < lhs.size());
    used[best_i] = true;
    worst = std::max(worst, best);
  }
  return worst;
}

void write_vector_artifact(const std::filesystem::path& path,
                           const Eigen::VectorXd& values) {
  std::ofstream out(path);
  REQUIRE(out.good());
  out << std::setprecision(17);
  out << "index,value\n";
  for (Eigen::Index i = 0; i < values.size(); ++i) {
    out << (i + 1) << "," << values[i] << "\n";
  }
}

void write_matrix_artifact(const std::filesystem::path& path,
                           const Eigen::MatrixXd& matrix) {
  std::ofstream out(path);
  REQUIRE(out.good());
  out << std::setprecision(17);
  for (Eigen::Index r = 0; r < matrix.rows(); ++r) {
    for (Eigen::Index c = 0; c < matrix.cols(); ++c) {
      if (c > 0) out << ",";
      out << matrix(r, c);
    }
    out << "\n";
  }
}

void write_eigen_artifact(const std::filesystem::path& path,
                          const Eigen::VectorXcd& eigenvalues) {
  std::ofstream out(path);
  REQUIRE(out.good());
  out << std::setprecision(17);
  out << "index,real,imag\n";
  for (Eigen::Index i = 0; i < eigenvalues.size(); ++i) {
    out << (i + 1) << ","
        << eigenvalues[i].real() << ","
        << eigenvalues[i].imag() << "\n";
  }
}

void write_hacdcpf_internal_diagnostics(
    const std::filesystem::path& dir,
    const DynamicDaeDiagnostics& diagnostics) {
  std::filesystem::create_directories(dir);
  {
    std::ofstream out(dir / "metadata.csv");
    REQUIRE(out.good());
    out << std::setprecision(17);
    out << "key,value\n";
    out << "case,simple_marconato\n";
    out << "formulation,MassMatrixDaeDiagnostic\n";
    out << "coordinate_system," << diagnostics.coordinate_system << "\n";
    out << "variable_count," << diagnostics.n_variables << "\n";
    out << "algebraic_count," << diagnostics.n_algebraic << "\n";
    out << "differential_count," << diagnostics.n_differential << "\n";
    out << "residual_inf_norm," << diagnostics.residual_inf_norm << "\n";
    out << "jacobian_rows," << diagnostics.jacobian.rows() << "\n";
    out << "jacobian_cols," << diagnostics.jacobian.cols() << "\n";
    out << "jacobian_inf_norm," << diagnostics.jacobian_inf_norm << "\n";
    out << "reduced_jacobian_rows," << diagnostics.reduced_jacobian.rows() << "\n";
    out << "reduced_jacobian_cols," << diagnostics.reduced_jacobian.cols() << "\n";
    out << "reduced_jacobian_inf_norm,"
        << diagnostics.reduced_jacobian_inf_norm << "\n";
    out << "eigenvalue_count," << diagnostics.eigenvalues.size() << "\n";
  }
  {
    std::ofstream out(dir / "state_table.csv");
    REQUIRE(out.good());
    out << std::setprecision(17);
    out << "index,label,value,residual,mass_diag,is_differential\n";
    for (std::size_t i = 0; i < diagnostics.states.size(); ++i) {
      const auto& state = diagnostics.states[i];
      out << (i + 1) << ","
          << state.label << ","
          << diagnostics.state[static_cast<Eigen::Index>(i)] << ","
          << diagnostics.residual[static_cast<Eigen::Index>(i)] << ","
          << diagnostics.mass_diag[static_cast<Eigen::Index>(i)] << ","
          << (state.kind == "differential" ? 1 : 0) << "\n";
    }
  }
  write_vector_artifact(dir / "mass_diag.csv", diagnostics.mass_diag);
  write_vector_artifact(dir / "residual.csv", diagnostics.residual);
  write_matrix_artifact(dir / "jacobian.csv", diagnostics.jacobian);
  write_matrix_artifact(dir / "reduced_jacobian.csv",
                        diagnostics.reduced_jacobian);
  write_eigen_artifact(dir / "eigenvalues.csv", diagnostics.eigenvalues);
}

double manifest_internal_tolerance(const Json& raw_case,
                                   const std::string& key,
                                   double fallback) {
  if (!raw_case.contains("internal_diagnostics")) return fallback;
  const auto& internal = raw_case.at("internal_diagnostics");
  if (!internal.contains("tolerances")) return fallback;
  const auto& tolerances = internal.at("tolerances");
  return tolerances.value(key, fallback);
}

struct ManifestSignalSpec {
  std::string local_key;
  std::string psd_quantity;
  double rms_tolerance{1.0};
  double max_tolerance{1.0};
  bool compare_relative{false};
};

struct ManifestDeviceSpec {
  std::string psd_ref;
  std::string local_component_type;
  int local_component_index{0};
  std::vector<ManifestSignalSpec> signals;
};

struct ManifestExecutableCase {
  std::string id;
  std::string psd_case;
  std::string psd_test;
  std::string scope;
  std::string gate_contract;
  std::string reference_package;
  std::string reference_formulation;
  std::string reference_integrator;
  std::string hacdcpf_fixture;
  std::string hacdcpf_solver;
  double t_start_s{0.0};
  double t_end_s{2.0};
  double dt_s{0.005};
  double compare_start_s{0.0};
  double compare_end_s{2.0};
  DynamicEvent event;
  std::vector<ManifestDeviceSpec> devices;
};

struct ManifestTraceComparison {
  std::string manifest_id;
  std::string psd_case;
  std::string level;
  std::string signal;
  CsvSeries local;
  double t_start{0.0};
  double t_end{2.0};
  double rms_tolerance{1.0};
  double max_tolerance{1.0};
  bool compare_relative{false};
  std::string reference_label;
};

std::filesystem::path psd_component_test_matrix_path() {
  return std::filesystem::path(HACDCPF_PROJECT_ROOT) /
         "tools" / "psd_validation" / "psd_component_test_matrix.json";
}

Json load_psd_component_test_matrix() {
  const std::filesystem::path matrix_path = psd_component_test_matrix_path();
  INFO("PSD component test matrix: " << matrix_path);
  std::ifstream in(matrix_path);
  REQUIRE(in.good());
  Json payload = Json::parse(in);
  REQUIRE(payload.value("format", "") == "hacdcpf_psd_component_test_matrix.v1");
  REQUIRE(payload.contains("rows"));
  REQUIRE(payload.at("rows").is_array());
  REQUIRE(payload.contains("execution_manifest"));
  REQUIRE(payload.at("execution_manifest").is_object());
  REQUIRE(payload.at("execution_manifest").value("version", 0) == 1);
  REQUIRE(payload.at("execution_manifest").contains("cases"));
  REQUIRE(payload.at("execution_manifest").at("cases").is_array());
  return payload;
}

const Json* find_manifest_case(const Json& payload, const std::string& id) {
  const auto& cases = payload.at("execution_manifest").at("cases");
  for (const auto& candidate : cases) {
    if (candidate.value("id", "") == id) return &candidate;
  }
  return nullptr;
}

bool manifest_row_selector_matches(const Json& row, const Json& selector) {
  for (auto it = selector.begin(); it != selector.end(); ++it) {
    if (!row.contains(it.key())) return false;
    if (row.at(it.key()) != it.value()) return false;
  }
  return true;
}

const Json* find_manifest_row(const Json& payload, const Json& selector) {
  for (const auto& row : payload.at("rows")) {
    if (manifest_row_selector_matches(row, selector)) return &row;
  }
  return nullptr;
}

DynamicSolverType manifest_solver_type(const std::string& solver) {
  if (solver == "PartitionedEuler") return DynamicSolverType::PartitionedEuler;
  if (solver == "PartitionedHeun") return DynamicSolverType::PartitionedHeun;
  if (solver == "PartitionedRK4") return DynamicSolverType::PartitionedRK4;
  if (solver == "BackwardEulerNewton") return DynamicSolverType::BackwardEulerNewton;
  if (solver == "TrapezoidalNewton") return DynamicSolverType::TrapezoidalNewton;
  if (solver == "RosenbrockEuler") return DynamicSolverType::RosenbrockEuler;
  REQUIRE(solver == "MassMatrixDae");
  return DynamicSolverType::MassMatrixDae;
}

ManifestExecutableCase parse_manifest_executable_case(const Json& raw) {
  REQUIRE(raw.value("enabled", false));
  ManifestExecutableCase out;
  out.id = raw.at("id").get<std::string>();
  out.psd_case = raw.at("psd_case").get<std::string>();
  out.psd_test = raw.at("psd_test").get<std::string>();
  out.scope = raw.value("scope", "component");
  out.gate_contract = raw.value("gate_contract", "");

  const auto& reference = raw.at("psd_reference");
  out.reference_package = reference.value("package", "PowerSimulationsDynamics.jl");
  out.reference_formulation = reference.at("formulation").get<std::string>();
  out.reference_integrator = reference.at("integrator").get<std::string>();

  const auto& hacdcpf = raw.at("hacdcpf");
  out.hacdcpf_fixture = hacdcpf.at("fixture").get<std::string>();
  out.hacdcpf_solver = hacdcpf.at("solver").get<std::string>();

  const auto& span = raw.at("time_span_s");
  out.t_start_s = span.at("start").get<double>();
  out.t_end_s = span.at("end").get<double>();
  out.dt_s = raw.at("step_s").get<double>();

  const auto& window = raw.at("comparison_window_s");
  out.compare_start_s = window.at("start").get<double>();
  out.compare_end_s = window.at("end").get<double>();

  const auto& event = hacdcpf.at("event");
  const std::string event_type = event.at("type").get<std::string>();
  REQUIRE(event_type == "ACBranchTrip");
  out.event.type = DynamicEventType::ACBranchTrip;
  out.event.time_s = event.at("time_s").get<double>();
  out.event.component_type = event.value("component_type", "AC");
  out.event.component_index = event.at("component_index").get<int>();
  out.event.label = event.value("label", out.id + " event");

  for (const auto& raw_device : raw.at("comparisons")) {
    ManifestDeviceSpec device;
    device.psd_ref = raw_device.at("psd_ref").get<std::string>();
    device.local_component_type =
        raw_device.value("local_component_type", "SynchronousMachine");
    device.local_component_index = raw_device.at("local_component_index").get<int>();
    for (const auto& raw_signal : raw_device.at("signals")) {
      ManifestSignalSpec signal;
      signal.local_key = raw_signal.at("local_key").get<std::string>();
      signal.psd_quantity = raw_signal.at("psd_quantity").get<std::string>();
      const std::string alignment = raw_signal.value("alignment", "absolute");
      REQUIRE((alignment == "absolute" || alignment == "relative_to_initial"));
      signal.compare_relative = alignment == "relative_to_initial";
      const auto& tolerances = raw_signal.at("tolerances");
      signal.rms_tolerance = tolerances.at("rms").get<double>();
      signal.max_tolerance = tolerances.at("max").get<double>();
      device.signals.push_back(signal);
    }
    REQUIRE_FALSE(device.signals.empty());
    out.devices.push_back(device);
  }
  REQUIRE_FALSE(out.devices.empty());
  return out;
}

HybridPowerSystem make_manifest_system(const ManifestExecutableCase& spec) {
  if (spec.hacdcpf_fixture == "make_psd_simple_marconato_three_bus_case") {
    return make_psd_simple_marconato_three_bus_case();
  }
  FAIL("Unsupported PSD executable manifest fixture: " << spec.hacdcpf_fixture);
  return HybridPowerSystem{};
}

DynamicResults run_manifest_hacdcpf_case(const ManifestExecutableCase& spec) {
  auto sys = make_manifest_system(spec);
  DynamicSolverOptions opt = fast_options();
  opt.solver_type = manifest_solver_type(spec.hacdcpf_solver);
  opt.run_power_flow_initialization = true;
  opt.t_start_s = spec.t_start_s;
  opt.t_end_s = spec.t_end_s;
  opt.dt_s = spec.dt_s;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  dyn.events.push_back(spec.event);

  DynamicSolver solver;
  return solver.solve(dyn);
}

std::vector<ManifestTraceComparison> build_manifest_trace_comparisons(
    const ManifestExecutableCase& spec,
    const DynamicResults& result,
    const std::filesystem::path& out_dir) {
  std::vector<ManifestTraceComparison> traces;
  REQUIRE(result.final_snapshot() != nullptr);
  for (const auto& device : spec.devices) {
    const auto& final_out =
        require_device_output(*result.final_snapshot(),
                              device.local_component_type,
                              device.local_component_index);
    CHECK(final_out.model_name == "SimpleMarconatoMachine");
    for (const auto& signal : device.signals) {
      auto local = device_output_series(result,
                                        device.local_component_type,
                                        device.local_component_index,
                                        signal.local_key);
      const std::string psd_signal = device.psd_ref + ":" + signal.psd_quantity;
      write_csv_series(out_dir /
                           ("hacdcpf_manifest_" + safe_artifact_token(spec.id) +
                            "_" + safe_artifact_token(psd_signal) + ".csv"),
                       local);
      traces.push_back({spec.id,
                        spec.psd_case,
                        spec.scope,
                        psd_signal,
                        std::move(local),
                        spec.compare_start_s,
                        spec.compare_end_s,
                        signal.rms_tolerance,
                        signal.max_tolerance,
                        signal.compare_relative,
                        spec.reference_package + " " +
                            spec.reference_formulation + "/" +
                            spec.reference_integrator});
    }
  }
  return traces;
}

bool export_psd_gridfollowing_trace(const PsdGflCase& spec,
                                    const std::filesystem::path& out_csv,
                                    const std::filesystem::path& log_file) {
  const std::filesystem::path repo = psd_repo_path();
  const std::filesystem::path script =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) /
      "tools" / "psd_validation" / "export_gridfollowing_trace.jl";

  INFO("PSD repo: " << repo);
  INFO("PSD exporter: " << script);
  INFO("PSD output CSV: " << out_csv);
  INFO("PSD log: " << log_file);
  REQUIRE(std::filesystem::exists(repo / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / "Project.toml"));
  REQUIRE(std::filesystem::exists(repo / "test" / spec.psd_test_file));
  REQUIRE(std::filesystem::exists(script));

  const std::string command =
      "cd " + shell_quote(repo.string()) + " && " +
      shell_quote(julia_bin()) + " --project=" +
      shell_quote((repo / "test").string()) + " " +
      shell_quote(script.string()) + " " +
      shell_quote(repo.string()) + " " +
      shell_quote(spec.psd_case) + " " +
      shell_quote(out_csv.string()) + " p_oc > " +
      shell_quote(log_file.string()) + " 2>&1";
  return std::system(command.c_str()) == 0;
}

// Two-bus case with a non-slack classical machine at bus 2 that optionally
// carries governor / AVR / PSS control blocks in its dynamic_model.
HybridPowerSystem make_controlled_machine_case(bool governor, bool avr, bool pss) {
  HybridPowerSystem sys;
  sys.name = "controlled_machine";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.02; b1.va_deg = 0.0;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PV; b2.vm_pu = 1.0; b2.va_deg = 0.0;
  sys.ac.buses = {b1, b2};

  ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2;
  br.r_pu = 0.01; br.x_pu = 0.05; br.tap = 1.0;
  sys.ac.branches = {br};

  Generator gs; gs.index = 1; gs.bus = 1; gs.is_slack = true; gs.vg_pu = 1.02;
  gs.pg_mw = 40.0; gs.xdpp_pu = 0.20; gs.qmax_mvar = 300; gs.qmin_mvar = -300;
  Generator g2; g2.index = 2; g2.bus = 2; g2.is_slack = false; g2.vg_pu = 1.0;
  g2.pg_mw = 50.0; g2.qg_mvar = 10.0; g2.xdpp_pu = 0.20; g2.inertia_h = 3.0;
  g2.pmax_mw = 200.0; g2.pmin_mw = 0.0; g2.qmax_mvar = 300; g2.qmin_mvar = -300;
  g2.dynamic_model.model_name = "ClassicalMachine";
  auto add_block = [&](const std::string& type, const std::string& model,
                       std::map<std::string, double> params) {
    hacdcpf::DynamicModelComponentProfile c;
    c.type = type; c.model = model; c.standard = "IEEE"; c.parameters = std::move(params);
    g2.dynamic_model.components.push_back(std::move(c));
  };
  if (governor) add_block("governor", "TGOV1", {{"R", 0.05}, {"T1", 0.5}, {"T3", 0.5}});
  if (avr) add_block("exciter", "SEXS", {{"Ka", 50.0}, {"Ta", 0.1}});
  if (pss) add_block("pss", "PSS1A",
                     {{"Ks", 10.0}, {"Tw", 10.0}, {"T1", 0.15}, {"T2", 0.03},
                      {"T3", 0.15}, {"T4", 0.03}});
  sys.ac.generators = {gs, g2};

  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = 30.0; ld.q_mvar = 10.0;
  sys.ac.loads = {ld};
  return sys;
}

// Peak-to-peak swing of a series after a given time (post-disturbance).
double series_swing_after(const CsvSeries& s, double t_after) {
  double lo = std::numeric_limits<double>::infinity();
  double hi = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < s.t.size(); ++i) {
    if (s.t[i] < t_after) continue;
    lo = std::min(lo, s.y[i]);
    hi = std::max(hi, s.y[i]);
  }
  return (hi >= lo) ? hi - lo : 0.0;
}

double series_mean_after(const CsvSeries& s, double t_after) {
  double sum = 0.0; int n = 0;
  for (std::size_t i = 0; i < s.t.size(); ++i) {
    if (s.t[i] < t_after) continue;
    sum += s.y[i]; ++n;
  }
  return n > 0 ? sum / n : 0.0;
}

DynamicResults run_controlled_case(bool governor, bool avr, bool pss) {
  DynamicSolverOptions opt;
  opt.t_end_s = 6.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;
  opt.use_consistent_dynamic_initialization = true;
  auto sys = make_controlled_machine_case(governor, avr, pss);
  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent ev;
  ev.time_s = 1.0;
  ev.type = DynamicEventType::ACLoadScale;
  ev.bus = 2;
  ev.value = 1.6;  // +60% load step at the machine bus
  dyn.events.push_back(ev);
  DynamicSolver solver;
  return solver.solve(dyn);
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
  auto sys = make_transient_2bus();
  sys.ac.loads.clear();
  auto opt = fast_options();
  opt.t_end_s = 0.03;
  opt.enforce_voltage_health_check = false;

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

  INFO(result.message);
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

  INFO(result.message);
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

TEST_CASE("GFL inverter supports KauraPLL profile and exposes dynamic model metadata",
          "[dynamics][gfl][pll][profile]") {
  auto sys = make_hybrid_dc_case();
  sys.vsc_converters[0].dynamic_model.standard = "NERC";
  sys.vsc_converters[0].dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  sys.vsc_converters[0].dynamic_model.parameter_set = "kaura_demo";
  sys.vsc_converters[0].dynamic_model.components.push_back(
      {"pll", "KauraPLL", "PSD", "default", {{"kp_pll", 0.015},
                                               {"ki_pll", 1.1},
                                               {"pll_lpf_t_s", 0.004}}});
  auto opt = fast_options();
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.final_snapshot() != nullptr);
  const auto& outputs = result.final_snapshot()->device_outputs;
  const auto it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(it != outputs.end());
  REQUIRE(it->values.count("pll_model") == 1);
  CHECK(it->values.at("pll_model") == Catch::Approx(1.0));
  CHECK(it->values.count("pll_vq_raw_pu") == 1);
  CHECK(it->values.count("pll_vd_pu") == 1);
  REQUIRE(it->model_profiles.size() >= 2);
  CHECK(it->model_profiles.front().model_name == "REGC_REEC_GFL_Subset");
  CHECK(std::any_of(it->model_profiles.begin(),
                    it->model_profiles.end(),
                    [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                      return profile.profile == "pll" &&
                             profile.model_name == "KauraPLL";
                    }));
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

TEST_CASE("No-event dynamic equilibrium residual is a hard benchmark gate",
          "[dynamics][benchmark][equilibrium]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
  REQUIRE(result.snapshots.size() >= 2);
  const double p0 = vsc_p_mw(result.snapshots.front());
  const double p1 = final_vsc_p_mw(result);
  CHECK(p1 == Catch::Approx(p0).margin(5e-4));
}

TEST_CASE("Dynamic DC link trim balances converter power at the pre-event point",
          "[dynamics][benchmark][equilibrium][dc-link]") {
  const auto sys = make_hybrid_dc_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.dynamic_dc_link = true;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
  REQUIRE_FALSE(result.snapshots.empty());
  const auto& outputs = result.snapshots.front().device_outputs;
  const auto vsc = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "VSCGridFollowing";
  });
  REQUIRE(vsc != outputs.end());
  REQUIRE(vsc->values.count("dc_link_dynamic") == 1);
  CHECK(vsc->values.at("dc_link_dynamic") == Catch::Approx(1.0));
}

TEST_CASE("Imported MATPOWER AC cases hold the no-event dynamic equilibrium",
          "[dynamics][benchmark][equilibrium][matpower]") {
  const std::filesystem::path matpower_dir =
      std::filesystem::path(HACDCPF_PROJECT_ROOT) / "external_data" / "matpower";
  for (const std::string filename : {"case9.m", "case30.m"}) {
    const auto raw_sys = hacdcpf::io::parse_matpower((matpower_dir / filename).string());
    for (const auto& [variant, sys] : {
             std::pair<std::string, HybridPowerSystem>{"direct", raw_sys},
             std::pair<std::string, HybridPowerSystem>{
                 "canvas-sync",
                 materialize_canvas_loads_from_bus_demand(raw_sys)}}) {
      DynamicSolverOptions opt = fast_options();
      opt.run_power_flow_initialization = true;
      opt.t_end_s = 1.0;
      opt.dt_s = 0.01;
      opt.record_every_step = true;
      opt.dynamic_trim_tol = 1e-7;
      opt.power_flow_options.tol = 1e-10;
      opt.power_flow_options.max_iter = 100;

      const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

      CAPTURE(filename, variant);
      REQUIRE(result.success);
      CHECK(result.initialization.power_flow_converged);
      CHECK(result.initialization.dynamic_trim_converged);
      REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
      REQUIRE(result.snapshots.size() >= 2);
      const auto& first = result.snapshots.front();
      const auto& last = result.snapshots.back();
      CHECK(last.min_ac_voltage_pu == Catch::Approx(first.min_ac_voltage_pu).margin(1e-8));
      CHECK(last.max_ac_voltage_pu == Catch::Approx(first.max_ac_voltage_pu).margin(1e-8));
      for (const auto& out : last.device_outputs) {
        if (out.type != "SynchronousMachine") continue;
        const auto omega_it = out.values.find("omega_pu");
        REQUIRE(omega_it != out.values.end());
        CHECK(omega_it->second == Catch::Approx(1.0).margin(1e-8));
      }
    }
  }
}

TEST_CASE("PSD validation ladder covers component, load, and system-level HACDCPF anchors",
          "[dynamics][benchmark][psd][validation]") {
  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_validation";
  std::filesystem::create_directories(out_dir);
  std::vector<ValidationMetric> metrics;

  {
    auto sys = make_psd_genrou_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;
    opt.max_dynamic_trim_iters = 20;
    opt.algebraic_network_max_iters = 8;
    opt.algebraic_network_tol = 1e-11;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    trip.label = "PSD GENROU fixture branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    REQUIRE(result.success);
    CHECK(result.initialization.power_flow_converged);
    CHECK(result.initialization.dynamic_trim_converged);
    CHECK(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
    REQUIRE_FALSE(result.applied_event_records.empty());
    REQUIRE(result.snapshots.size() > 100);

    const auto rotor = device_output_series(result,
                                            "SynchronousMachine",
                                            2,
                                            "angle_rad");
    const auto omega = device_output_series(result,
                                            "SynchronousMachine",
                                            2,
                                            "omega_pu");
    write_csv_series(out_dir / "hacdcpf_component_genrou_angle_rad.csv", rotor);
    write_csv_series(out_dir / "hacdcpf_component_genrou_omega_pu.csv", omega);

    CHECK(series_range(rotor) > 1e-4);
    CHECK(series_range(omega) > 1e-7);

    const auto& gen_out =
        require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
    CHECK(gen_out.model_standard == "IEEE");
    REQUIRE(gen_out.model_profiles.size() >= 1);
    CHECK(std::any_of(gen_out.model_profiles.begin(),
                      gen_out.model_profiles.end(),
                      [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.standard == "PSS/E" &&
                               profile.model_name == "GENROU";
                      }));

    metrics.push_back({"component",
                       "genrou_three_bus_subset",
                       "HACDCPF",
                       "no_event_initial_dxdt_inf",
                       result.initialization.dynamic_fast_dxdt_inf_norm,
                       result.initialization.dynamic_fast_dxdt_inf_norm,
                       opt.dynamic_trim_tol,
                       opt.dynamic_trim_tol,
                       result.initialization.dynamic_fast_dxdt_inf_norm <=
                           opt.dynamic_trim_tol});
  }

  {
    auto sys = make_psd_zip_load_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    trip.label = "PSD ZIP fixture branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    REQUIRE(result.success);
    CHECK(result.initialization.power_flow_converged);
    REQUIRE(result.snapshots.size() > 100);
    const auto v102 = bus_voltage_mag_series(result, 1);
    const auto v103 = bus_voltage_mag_series(result, 2);
    write_csv_series(out_dir / "hacdcpf_component_zip_v102.csv", v102);
    write_csv_series(out_dir / "hacdcpf_component_zip_v103.csv", v103);

    CHECK(series_range(v102) > 1e-5);
    CHECK(series_range(v103) > 1e-5);

    const auto& load_out = require_device_output(*result.final_snapshot(), "ACLoad", 1);
    REQUIRE(load_out.model_profiles.size() >= 1);
    CHECK(std::any_of(load_out.model_profiles.begin(),
                      load_out.model_profiles.end(),
                      [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.standard == "PSS/E" &&
                               profile.model_name == "ConstantPowerLoad";
                      }));

    metrics.push_back({"component",
                       "zip_load_three_bus_subset",
                       "HACDCPF",
                       "voltage_response_range_bus103",
                       series_range(v103),
                       series_range(v103),
                       1e-5,
                       1e-5,
                       series_range(v103) > 1e-5});
  }

  {
    auto sys = make_hybrid_dc_case();
    sys.ac.generators[0].dynamic_model.standard = "IEEE";
    sys.ac.generators[0].dynamic_model.model_name = "ClassicalMachine";
    sys.vsc_converters[0].dynamic_model.standard = "NERC";
    sys.vsc_converters[0].dynamic_model.model_name = "REGC_REEC_GFL_Subset";
    sys.vsc_converters[0].dynamic_model.components.push_back(
        {"pll", "FixedFrequency", "PowerSimulationsDynamics", "system_anchor", {}});

    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent pref_step;
    pref_step.time_s = 1.0;
    pref_step.type = DynamicEventType::Custom;
    pref_step.component_type = "VSC";
    pref_step.component_index = 1;
    pref_step.label = "system-level VSC reference step";
    pref_step.params["p_ref_mw"] = 12.0;
    dyn.events.push_back(pref_step);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    REQUIRE(result.success);
    CHECK(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
    REQUIRE(result.snapshots.size() > 100);
    const auto p_vsc = device_output_series(result,
                                            "VSCGridFollowing",
                                            1,
                                            "p_filtered_mw");
    const auto f_vsc = device_output_series(result,
                                            "VSCGridFollowing",
                                            1,
                                            "pll_frequency_hz");
    write_csv_series(out_dir / "hacdcpf_system_hybrid_vsc_p_filtered_mw.csv", p_vsc);
    write_csv_series(out_dir / "hacdcpf_system_hybrid_vsc_pll_frequency_hz.csv", f_vsc);

    CHECK(series_range(p_vsc) > 0.5);
    CHECK(*std::min_element(f_vsc.y.begin(), f_vsc.y.end()) > 45.0);
    CHECK(*std::max_element(f_vsc.y.begin(), f_vsc.y.end()) < 55.0);

    metrics.push_back({"system",
                       "hybrid_acdc_vsc_step",
                       "HACDCPF",
                       "p_filtered_response_range_mw",
                       series_range(p_vsc),
                       series_range(p_vsc),
                       0.5,
                       0.5,
                       series_range(p_vsc) > 0.5});
  }

  write_validation_summary(out_dir / "hacdcpf_psd_validation_summary.csv", metrics);
  for (const auto& metric : metrics) {
    INFO("Validation artifact directory: " << out_dir);
    INFO(metric.level << " " << metric.case_name << " " << metric.signal);
    CHECK(metric.passed);
  }
}

TEST_CASE("PSD OneDOneQ machine profile initializes and responds as a named model",
          "[dynamics][benchmark][psd][machine][onedoneq]") {
  auto sys = make_psd_onedoneq_three_bus_subset_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent trip;
  trip.time_s = 1.0;
  trip.type = DynamicEventType::ACBranchTrip;
  trip.component_index = 1;
  trip.component_type = "AC";
  trip.label = "PSD OneDOneQ fixture branch trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  const auto& gen_out =
      require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
  CHECK(gen_out.model_name == "OneDOneQMachine");
  CHECK(gen_out.values.count("psd_onedoneq") == 1);
  CHECK(gen_out.values.count("eq_p") == 1);
  CHECK(gen_out.values.count("ed_p") == 1);
  CHECK(gen_out.values.count("psi_kd") == 0);
  REQUIRE(gen_out.model_profiles.size() >= 1);
  CHECK(std::any_of(gen_out.model_profiles.begin(),
                    gen_out.model_profiles.end(),
                    [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                      return profile.model_name == "OneDOneQMachine";
                    }));

  const auto delta = device_output_series(result, "SynchronousMachine", 2, "angle_rad");
  const auto omega = device_output_series(result, "SynchronousMachine", 2, "omega_pu");
  const auto eqp = device_output_series(result, "SynchronousMachine", 2, "eq_p");
  const auto edp = device_output_series(result, "SynchronousMachine", 2, "ed_p");
  CHECK(series_range(delta) > 1e-4);
  CHECK(series_range(omega) > 1e-7);
  CHECK(series_range(eqp) > 1e-5);
  CHECK(series_range(edp) > 1e-5);
}

TEST_CASE("PSD Test 03 SimpleMarconato machine profile initializes and responds",
          "[dynamics][benchmark][psd][machine][simple-marconato]") {
  auto sys = make_psd_simple_marconato_three_bus_case();
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent trip;
  trip.time_s = 1.0;
  trip.type = DynamicEventType::ACBranchTrip;
  trip.component_index = 1;
  trip.component_type = "AC";
  trip.label = "PSD Test 03 SimpleMarconato BUS 1-BUS 3 branch trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  for (const int component_index : {2, 3}) {
    INFO("machine component " << component_index);
    const auto& gen_out =
        require_device_output(*result.final_snapshot(), "SynchronousMachine", component_index);
    CHECK(gen_out.model_name == "SimpleMarconatoMachine");
    CHECK(gen_out.values.count("psd_simple_marconato") == 1);
    CHECK(gen_out.values.count("eq_p") == 1);
    CHECK(gen_out.values.count("ed_p") == 1);
    CHECK(gen_out.values.count("eq_pp") == 1);
    CHECK(gen_out.values.count("ed_pp") == 1);
    CHECK(gen_out.values.count("psi_kd") == 0);
    REQUIRE(gen_out.model_profiles.size() >= 1);
    CHECK(std::any_of(gen_out.model_profiles.begin(),
                      gen_out.model_profiles.end(),
                      [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.model_name == "SimpleMarconatoMachine";
                      }));

    const auto delta =
        device_output_series(result, "SynchronousMachine", component_index, "angle_rad");
    const auto omega =
        device_output_series(result, "SynchronousMachine", component_index, "omega_pu");
    const auto eqp =
        device_output_series(result, "SynchronousMachine", component_index, "eq_p");
    const auto edp =
        device_output_series(result, "SynchronousMachine", component_index, "ed_p");
    CHECK(series_range(delta) > 1e-4);
    CHECK(series_range(omega) > 1e-7);
    CHECK(series_range(eqp) > 1e-5);
    CHECK(series_range(edp) > 1e-5);
  }
}

TEST_CASE("PSD PSSE machine profiles initialize and expose distinct states",
          "[dynamics][benchmark][psd][machine][psse]") {
  struct MachineSmokeCase {
    std::string name;
    HybridPowerSystem sys;
    std::vector<std::string> required_keys;
    std::vector<std::string> absent_keys;
  };
  std::vector<MachineSmokeCase> cases;
  cases.push_back({"GENROE",
                   make_psd_genroe_three_bus_subset_case(),
                   {"psd_genroe", "eq_p", "ed_p", "psi_kd", "psi_kq"},
                   {"psd_genrou", "psiq_pp"}});
  cases.push_back({"GENSAL",
                   make_psd_gensal_three_bus_subset_case(false),
                   {"psd_gensal", "eq_p", "psi_kd", "psiq_pp"},
                   {"ed_p", "psi_kq", "psd_gensae"}});
  cases.push_back({"GENSAE",
                   make_psd_gensal_three_bus_subset_case(true),
                   {"psd_gensae", "eq_p", "psi_kd", "psiq_pp"},
                   {"ed_p", "psi_kq", "psd_gensal"}});

  for (auto& spec : cases) {
    INFO(spec.name);
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;
    opt.max_dynamic_trim_iters = 20;
    opt.algebraic_network_max_iters = 8;
    opt.algebraic_network_tol = 1e-8;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(spec.sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    trip.label = spec.name + " fixture branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);

    INFO(result.message);
    REQUIRE(result.success);
    CHECK(result.initialization.power_flow_converged);
    CHECK(result.initialization.dynamic_trim_converged);
    REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= opt.dynamic_trim_tol);
    REQUIRE(result.snapshots.size() > 100);

    const auto& gen_out =
        require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
    CHECK(gen_out.model_name == spec.name);
    for (const auto& key : spec.required_keys) {
      INFO("required key " << key);
      CHECK(gen_out.values.count(key) == 1);
    }
    for (const auto& key : spec.absent_keys) {
      INFO("absent key " << key);
      CHECK(gen_out.values.count(key) == 0);
    }

    const auto delta = device_output_series(result, "SynchronousMachine", 2, "angle_rad");
    const auto omega = device_output_series(result, "SynchronousMachine", 2, "omega_pu");
    const auto eqp = device_output_series(result, "SynchronousMachine", 2, "eq_p");
    CHECK(series_range(delta) > 1e-4);
    CHECK(series_range(omega) > 1e-7);
    CHECK(series_range(eqp) > 1e-5);
    if (spec.name == "GENROE") {
      const auto edp = device_output_series(result, "SynchronousMachine", 2, "ed_p");
      CHECK(series_range(edp) > 1e-5);
    } else {
      const auto psiqpp = device_output_series(result, "SynchronousMachine", 2, "psiq_pp");
      CHECK(series_range(psiqpp) > 1e-5);
    }
  }
}

TEST_CASE("PSD component matrix executable manifest is internally consistent",
          "[dynamics][benchmark][psd][manifest]") {
  const Json payload = load_psd_component_test_matrix();
  const auto& cases = payload.at("execution_manifest").at("cases");
  REQUIRE_FALSE(cases.empty());

  std::size_t enabled_count = 0;
  bool found_simple_marconato_gate = false;
  for (const auto& raw_case : cases) {
    REQUIRE(raw_case.contains("id"));
    REQUIRE(raw_case.contains("enabled"));
    REQUIRE(raw_case.contains("row_selector"));
    REQUIRE(raw_case.at("row_selector").is_object());

    const Json* row = find_manifest_row(payload, raw_case.at("row_selector"));
    REQUIRE(row != nullptr);
    if (row->contains("execution_case_id")) {
      CHECK(row->at("execution_case_id") == raw_case.at("id"));
    }

    if (!raw_case.value("enabled", false)) continue;
    ++enabled_count;

    const ManifestExecutableCase spec = parse_manifest_executable_case(raw_case);
    CHECK_FALSE(spec.id.empty());
    CHECK_FALSE(spec.psd_case.empty());
    CHECK_FALSE(spec.gate_contract.empty());
    CHECK(spec.t_start_s <= spec.compare_start_s);
    CHECK(spec.compare_end_s <= spec.t_end_s);
    CHECK(spec.dt_s > 0.0);
    CHECK(row->value("status", "") == "compare-limited");

    std::size_t signal_count = 0;
    for (const auto& device : spec.devices) {
      CHECK_FALSE(device.psd_ref.empty());
      CHECK(device.local_component_index > 0);
      signal_count += device.signals.size();
      for (const auto& signal : device.signals) {
        CHECK_FALSE(signal.local_key.empty());
        CHECK_FALSE(signal.psd_quantity.empty());
        CHECK(signal.rms_tolerance > 0.0);
        CHECK(signal.max_tolerance > 0.0);
      }
    }
    CHECK(signal_count > 0);

    if (spec.id == "psd-test03-simple-marconato-residual-vs-mass-matrix") {
      found_simple_marconato_gate = true;
      CHECK(spec.psd_test == "Test 03");
      CHECK(spec.psd_case == "simple_marconato");
      CHECK(spec.reference_formulation == "ResidualModel");
      CHECK(spec.reference_integrator == "IDA");
      CHECK(spec.hacdcpf_fixture == "make_psd_simple_marconato_three_bus_case");
      CHECK(spec.hacdcpf_solver == "MassMatrixDae");
      CHECK(spec.devices.size() == 2);
      CHECK(signal_count == 8);
      REQUIRE(raw_case.contains("internal_diagnostics"));
      const auto& internal = raw_case.at("internal_diagnostics");
      CHECK(internal.value("enabled", false));
      CHECK(internal.value("coordinate_system", "") ==
            "positive_sequence_psd_order");
      REQUIRE(internal.contains("objects"));
      CHECK(internal.at("objects").size() >= 5);
    }
  }

  CHECK(enabled_count >= 1);
  CHECK(found_simple_marconato_gate);
}

TEST_CASE("PSD-order DynamicDaeDiagnostics exposes SimpleMarconato residual objects",
          "[dynamics][benchmark][psd][diagnostics]") {
  const Json payload = load_psd_component_test_matrix();
  const Json* raw_case =
      find_manifest_case(payload,
                         "psd-test03-simple-marconato-residual-vs-mass-matrix");
  REQUIRE(raw_case != nullptr);
  const ManifestExecutableCase spec = parse_manifest_executable_case(*raw_case);

  DynamicSolverOptions opt = fast_options();
  opt.solver_type = DynamicSolverType::MassMatrixDae;
  opt.run_power_flow_initialization = true;
  opt.t_start_s = spec.t_start_s;
  opt.t_end_s = spec.t_start_s;
  opt.dt_s = spec.dt_s;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(make_manifest_system(spec), opt);
  CHECK(dyn.initialization.power_flow_converged);
  CHECK(dyn.initialization.dynamic_trim_converged);
  std::string error;
  REQUIRE(dyn.solveNetwork(opt.t_start_s, error));

  DynamicDaeDiagnosticOptions diag_options;
  diag_options.positive_sequence_projection = true;
  diag_options.psd_simple_marconato_state_order = true;
  diag_options.build_jacobian = true;
  diag_options.finite_difference_step =
      raw_case->at("internal_diagnostics").value("finite_difference_step", 1e-4);
  const DynamicDaeDiagnostics diagnostics =
      dynamic_dae_diagnostics(dyn, diag_options);
  INFO(diagnostics.message);
  REQUIRE(diagnostics.success);
  CHECK(diagnostics.coordinate_system == "positive_sequence_psd_order");
  REQUIRE(diagnostics.n_variables == 26);
  REQUIRE(diagnostics.n_algebraic == 6);
  REQUIRE(diagnostics.n_differential == 20);
  REQUIRE(diagnostics.mass_diag.size() == 26);
  for (Eigen::Index i = 0; i < diagnostics.mass_diag.size(); ++i) {
    CHECK(diagnostics.mass_diag[i] == Catch::Approx(i < 6 ? 0.0 : 1.0).margin(1e-14));
  }
  CHECK(diagnostics.residual_inf_norm < 1e-6);
  CHECK(diagnostics.jacobian.rows() == 26);
  CHECK(diagnostics.jacobian.cols() == 26);
  CHECK(diagnostics.reduced_jacobian.rows() == 20);
  CHECK(diagnostics.reduced_jacobian.cols() == 20);
  CHECK(diagnostics.eigenvalues.size() == 20);
}

TEST_CASE("PSD executable manifest gates SimpleMarconato Test 03 residual vs MassMatrixDae",
          "[dynamics][benchmark][psd][manifest][external]") {
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  if (!run_external_psd) {
    SUCCEED("Set HACDCPF_RUN_PSD_COMPARE=1 to run executable PSD manifest gates");
    return;
  }

  const Json payload = load_psd_component_test_matrix();
  const Json* raw_case =
      find_manifest_case(payload,
                         "psd-test03-simple-marconato-residual-vs-mass-matrix");
  REQUIRE(raw_case != nullptr);
  const ManifestExecutableCase spec = parse_manifest_executable_case(*raw_case);
  REQUIRE(spec.reference_formulation == "ResidualModel");
  REQUIRE(spec.reference_integrator == "IDA");
  REQUIRE(spec.hacdcpf_solver == "MassMatrixDae");

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_manifest_validation";
  std::filesystem::create_directories(out_dir);

  const DynamicResults result = run_manifest_hacdcpf_case(spec);
  INFO(spec.id << ": " << result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= 1e-7);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  const std::vector<ManifestTraceComparison> traces =
      build_manifest_trace_comparisons(spec, result, out_dir);
  REQUIRE(traces.size() == 8);

  auto psd_csv_path = [&](const ManifestTraceComparison& trace) {
    return out_dir / ("psd_manifest_" + safe_artifact_token(trace.manifest_id) +
                      "_" + safe_artifact_token(trace.signal) + ".csv");
  };

  std::vector<PsdTraceExportRequest> export_requests;
  export_requests.reserve(traces.size());
  for (const auto& trace : traces) {
    export_requests.push_back({trace.psd_case, trace.signal, psd_csv_path(trace)});
  }
  const std::filesystem::path psd_batch_log = out_dir / "psd_manifest_batch.log";
  const bool exported = export_psd_traces_batch(export_requests, psd_batch_log);
  INFO("PSD manifest export failed. Inspect " << psd_batch_log
       << ". Run `cd " << psd_repo_path()
       << " && julia --project=test -e 'using Pkg; Pkg.instantiate()'` "
       << "to install missing PSD test dependencies.");
  REQUIRE(exported);

  const std::filesystem::path pointwise_path =
      out_dir / "psd_manifest_pointwise_comparison.csv";
  std::ofstream pointwise(pointwise_path);
  REQUIRE(pointwise.good());
  pointwise << "level,case,signal,alignment,time_s,hacdcpf_raw,psd_raw,"
               "hacdcpf_compare,psd_compare,error,abs_error\n";

  std::vector<ValidationMetric> metrics;
  for (const auto& trace : traces) {
    const CsvSeries psd = read_csv_series(psd_csv_path(trace));
    REQUIRE(psd.t.size() > 100);
    append_pointwise_comparison_rows(pointwise,
                                     trace.level,
                                     trace.manifest_id,
                                     trace.signal,
                                     trace.local,
                                     psd,
                                     trace.t_start,
                                     trace.t_end,
                                     trace.compare_relative);
    const CsvSeries local_cmp =
        trace.compare_relative ? relative_to_initial(trace.local) : trace.local;
    const CsvSeries psd_cmp =
        trace.compare_relative ? relative_to_initial(psd) : psd;
    const double rms =
        rms_common_error(local_cmp, psd_cmp, trace.t_start, trace.t_end);
    const double max_abs =
        max_common_abs_error(local_cmp, psd_cmp, trace.t_start, trace.t_end);
    metrics.push_back({trace.level,
                       trace.manifest_id,
                       trace.reference_label,
                       trace.compare_relative ? trace.signal + "_relative"
                                              : trace.signal,
                       rms,
                       max_abs,
                       trace.rms_tolerance,
                       trace.max_tolerance,
                       rms < trace.rms_tolerance &&
                           max_abs < trace.max_tolerance});
  }
  pointwise.close();

  const std::filesystem::path summary_path =
      out_dir / "psd_manifest_validation_summary.csv";
  write_validation_summary(summary_path, metrics);
  for (const auto& metric : metrics) {
    INFO("PSD manifest artifact directory: " << out_dir);
    INFO("PSD manifest summary CSV: " << summary_path);
    INFO("PSD manifest pointwise comparison CSV: " << pointwise_path);
    INFO(metric.level << " " << metric.case_name << " " << metric.signal
                      << " rms=" << metric.rms_error
                      << " max=" << metric.max_abs_error);
    CHECK(metric.passed);
  }
}

TEST_CASE("PSD executable manifest gates SimpleMarconato Test 03 internal DAE diagnostics",
          "[dynamics][benchmark][psd][manifest][internal]") {
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  if (!run_external_psd) {
    SUCCEED("Set HACDCPF_RUN_PSD_COMPARE=1 to run Julia-backed PSD internal diagnostics");
    return;
  }

  const Json payload = load_psd_component_test_matrix();
  const Json* raw_case =
      find_manifest_case(payload,
                         "psd-test03-simple-marconato-residual-vs-mass-matrix");
  REQUIRE(raw_case != nullptr);
  REQUIRE(raw_case->contains("internal_diagnostics"));
  REQUIRE(raw_case->at("internal_diagnostics").value("enabled", false));
  const ManifestExecutableCase spec = parse_manifest_executable_case(*raw_case);
  REQUIRE(spec.psd_case == "simple_marconato");
  REQUIRE(spec.hacdcpf_solver == "MassMatrixDae");

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_internal_validation";
  const std::filesystem::path psd_dir = out_dir / "psd";
  const std::filesystem::path local_dir = out_dir / "hacdcpf";
  std::filesystem::create_directories(out_dir);

  const std::filesystem::path psd_log = out_dir / "psd_internal_export.log";
  const bool exported =
      export_psd_internal_diagnostics(spec.psd_case, psd_dir, psd_log);
  INFO("PSD internal export failed. Inspect " << psd_log
       << ". Run `cd " << psd_repo_path()
       << " && julia --project=test -e 'using Pkg; Pkg.instantiate()'` "
       << "to install missing PSD test dependencies.");
  REQUIRE(exported);
  const PsdInternalDiagnostics psd = read_psd_internal_diagnostics(psd_dir);

  DynamicSolverOptions opt = fast_options();
  opt.solver_type = DynamicSolverType::MassMatrixDae;
  opt.run_power_flow_initialization = true;
  opt.t_start_s = spec.t_start_s;
  opt.t_end_s = spec.t_start_s;
  opt.dt_s = spec.dt_s;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(make_manifest_system(spec), opt);
  INFO("HACDCPF dynamic trim residual: "
       << dyn.initialization.dynamic_fast_dxdt_inf_norm);
  CHECK(dyn.initialization.power_flow_converged);
  CHECK(dyn.initialization.dynamic_trim_converged);

  std::string error;
  REQUIRE(dyn.solveNetwork(opt.t_start_s, error));

  DynamicDaeDiagnosticOptions diag_options;
  diag_options.positive_sequence_projection = true;
  diag_options.psd_simple_marconato_state_order = true;
  diag_options.build_jacobian = true;
  diag_options.finite_difference_step =
      raw_case->at("internal_diagnostics").value("finite_difference_step", 1e-4);
  DynamicDaeDiagnostics local =
      dynamic_dae_diagnostics(dyn, diag_options);
  INFO(local.message);
  REQUIRE(local.success);
  write_hacdcpf_internal_diagnostics(local_dir, local);

  const int psd_n = metadata_int(psd.metadata, "variable_count");
  const int psd_alg = metadata_int(psd.metadata, "algebraic_count");
  const int psd_diff = metadata_int(psd.metadata, "differential_count");
  REQUIRE(local.n_variables == psd_n);
  REQUIRE(local.n_algebraic == psd_alg);
  REQUIRE(local.n_differential == psd_diff);
  REQUIRE(psd.labels.size() == static_cast<std::size_t>(psd_n));
  REQUIRE(local.states.size() == static_cast<std::size_t>(psd_n));
  REQUIRE(psd.mass_diag.size() == local.mass_diag.size());
  REQUIRE(psd.residual.size() == local.residual.size());
  REQUIRE(psd.jacobian.rows() == local.jacobian.rows());
  REQUIRE(psd.jacobian.cols() == local.jacobian.cols());
  REQUIRE(psd.reduced_jacobian.rows() == local.reduced_jacobian.rows());
  REQUIRE(psd.reduced_jacobian.cols() == local.reduced_jacobian.cols());
  REQUIRE(psd.eigenvalues.size() ==
          static_cast<std::size_t>(local.eigenvalues.size()));

  std::vector<Complex> local_eigs;
  local_eigs.reserve(static_cast<std::size_t>(local.eigenvalues.size()));
  for (Eigen::Index i = 0; i < local.eigenvalues.size(); ++i) {
    local_eigs.push_back(local.eigenvalues[i]);
  }

  const double mass_diag_error = max_abs_diff(local.mass_diag, psd.mass_diag);
  const double psd_residual_inf = metadata_double(psd.metadata, "residual_inf_norm");
  const double local_residual_inf = local.residual_inf_norm;
  const double residual_pair_inf =
      std::max(local_residual_inf, psd_residual_inf);
  const double jacobian_rel =
      relative_frobenius_error(local.jacobian, psd.jacobian);
  const double jacobian_max =
      max_abs_matrix_diff(local.jacobian, psd.jacobian);
  const double reduced_rel =
      relative_frobenius_error(local.reduced_jacobian, psd.reduced_jacobian);
  const double reduced_max =
      max_abs_matrix_diff(local.reduced_jacobian, psd.reduced_jacobian);
  const double eigen_max =
      nearest_eigenvalue_max_error(local_eigs, psd.eigenvalues);

  const double mass_tol =
      manifest_internal_tolerance(*raw_case, "mass_diag_max_abs", 1e-12);
  const double residual_tol =
      manifest_internal_tolerance(*raw_case, "residual_inf_norm", 1e-5);
  const double jac_rel_tol =
      manifest_internal_tolerance(*raw_case, "jacobian_relative_fro", 10.0);
  const double reduced_rel_tol =
      manifest_internal_tolerance(*raw_case, "reduced_jacobian_relative_fro", 10.0);
  const double eigen_tol =
      manifest_internal_tolerance(*raw_case, "eigenvalue_nearest_max_abs", 1e4);

  const std::filesystem::path summary_path =
      out_dir / "internal_comparison_summary.csv";
  std::ofstream summary(summary_path);
  REQUIRE(summary.good());
  summary << "metric,value,tolerance,passed\n";
  auto write_metric = [&](const std::string& name,
                          double value,
                          double tolerance) {
    summary << name << "," << value << "," << tolerance << ","
            << (value <= tolerance ? "true" : "false") << "\n";
  };
  auto write_info_metric = [&](const std::string& name, double value) {
    summary << name << "," << value << ",informational,n/a\n";
  };
  write_metric("mass_diag_max_abs", mass_diag_error, mass_tol);
  write_metric("residual_inf_norm_pair", residual_pair_inf, residual_tol);
  write_metric("jacobian_relative_fro", jacobian_rel, jac_rel_tol);
  write_info_metric("jacobian_max_abs", jacobian_max);
  write_metric("reduced_jacobian_relative_fro", reduced_rel, reduced_rel_tol);
  write_info_metric("reduced_jacobian_max_abs", reduced_max);
  write_metric("eigenvalue_nearest_max_abs", eigen_max, eigen_tol);
  summary.close();

  INFO("PSD internal artifact directory: " << out_dir);
  INFO("PSD internal summary CSV: " << summary_path);
  INFO("mass_diag_error=" << mass_diag_error
                          << " residual_pair_inf=" << residual_pair_inf
                          << " jacobian_rel=" << jacobian_rel
                          << " reduced_rel=" << reduced_rel
                          << " eigen_max=" << eigen_max);

  CHECK(mass_diag_error <= mass_tol);
  CHECK(residual_pair_inf <= residual_tol);
  CHECK(jacobian_rel <= jac_rel_tol);
  CHECK(reduced_rel <= reduced_rel_tol);
  CHECK(eigen_max <= eigen_tol);
}

TEST_CASE("Opt-in PSD external comparisons cover generator, load, and system traces",
          "[dynamics][benchmark][psd][external]") {
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  if (!run_external_psd) {
    SUCCEED("Set HACDCPF_RUN_PSD_COMPARE=1 to run Julia-backed PSD trace comparisons");
    return;
  }

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_validation";
  std::filesystem::create_directories(out_dir);

  struct ExternalSpec {
    std::string case_name;
    std::string signal;
    CsvSeries local;
    double t_start{0.0};
    double t_end{2.0};
    double rms_tolerance{1.0};
    double max_tolerance{1.0};
    std::string level;
    bool compare_relative{false};
  };

  std::vector<ExternalSpec> specs;

  struct MachineTraceSpec {
    std::string local_key;
    std::string psd_quantity;
    double rms_tolerance{1.0};
    double max_tolerance{1.0};
  };
  constexpr double kPsseAngleRmsToleranceRad =
      5.0 * 3.14159265358979323846 / 180.0;
  constexpr double kPsseAngleMaxToleranceRad =
      12.0 * 3.14159265358979323846 / 180.0;

  auto append_psse_machine_specs =
      [&](const std::string& case_name,
          HybridPowerSystem sys,
          const std::vector<MachineTraceSpec>& traces) {
        DynamicSolverOptions opt = fast_options();
        opt.run_power_flow_initialization = true;
        opt.t_end_s = 2.0;
        opt.dt_s = 0.005;
        opt.record_every_step = true;
        opt.dynamic_trim_tol = 1e-7;
        opt.max_dynamic_trim_iters = 20;
        opt.algebraic_network_max_iters = 8;
        opt.algebraic_network_tol = 1e-8;

        DynamicModelBuilder builder;
        DynamicSystem dyn = builder.build(sys, opt);
        DynamicEvent trip;
        trip.time_s = 1.0;
        trip.type = DynamicEventType::ACBranchTrip;
        trip.component_index = 1;
        trip.component_type = "AC";
        trip.label = "PSD " + case_name + " BUS 1-BUS 2 branch trip";
        dyn.events.push_back(trip);

        DynamicSolver solver;
        const DynamicResults result = solver.solve(dyn);
        INFO(case_name << ": " << result.message);
        REQUIRE(result.success);

        for (const auto& trace : traces) {
          auto local = device_output_series(result,
                                            "SynchronousMachine",
                                            2,
                                            trace.local_key);
          write_csv_series(out_dir /
                               ("hacdcpf_psd_" + case_name + "_" +
                                safe_artifact_token(trace.psd_quantity) + ".csv"),
                           local);
          specs.push_back({case_name,
                           "generator-102-1:" + trace.psd_quantity,
                           std::move(local),
                           0.0,
                           2.0,
                           trace.rms_tolerance,
                           trace.max_tolerance,
                           "component",
                           true});
        }
      };

  {
    auto sys = make_psd_genrou_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    INFO(result.message);
    REQUIRE(result.success);
    auto delta = device_output_series(result,
                                      "SynchronousMachine",
                                      2,
                                      "angle_rad",
                                      180.0 / 3.14159265358979323846);
    write_csv_series(out_dir / "hacdcpf_psd_genrou_delta_deg.csv", delta);
    specs.push_back({"genrou",
                     "generator-102-1:delta_deg",
                     std::move(delta),
                     0.0,
                     2.0,
                     5.0,
                     12.0,
                     "component",
                     true});
    auto omega = device_output_series(result,
                                      "SynchronousMachine",
                                      2,
                                      "omega_pu");
    write_csv_series(out_dir / "hacdcpf_psd_genrou_omega_pu.csv", omega);
    specs.push_back({"genrou",
                     "generator-102-1:omega_pu",
                     std::move(omega),
                     0.0,
                     2.0,
                     0.002,
                     0.01,
                     "component",
                     true});
  }

  {
    auto sys = make_psd_onedoneq_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;
    opt.max_dynamic_trim_iters = 20;
    opt.algebraic_network_max_iters = 8;
    opt.algebraic_network_tol = 1e-8;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 2;
    trip.component_type = "AC";
    trip.label = "PSD Test 02 OneDOneQ BUS 1-BUS 3 branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    REQUIRE(result.success);

    auto delta = device_output_series(result,
                                      "SynchronousMachine",
                                      2,
                                      "angle_rad");
    write_csv_series(out_dir / "hacdcpf_psd_onedoneq_delta_rad.csv", delta);
    specs.push_back({"onedoneq",
                     "generator-102-1:delta_rad",
                     std::move(delta),
                     0.0,
                     2.0,
                     0.05,
                     0.12,
                     "component",
                     true});

    auto omega = device_output_series(result,
                                      "SynchronousMachine",
                                      2,
                                      "omega_pu");
    write_csv_series(out_dir / "hacdcpf_psd_onedoneq_omega_pu.csv", omega);
    specs.push_back({"onedoneq",
                     "generator-102-1:omega_pu",
                     std::move(omega),
                     0.0,
                     2.0,
                     0.002,
                     0.005,
                     "component",
                     true});

    auto eqp = device_output_series(result,
                                    "SynchronousMachine",
                                    2,
                                    "eq_p");
    write_csv_series(out_dir / "hacdcpf_psd_onedoneq_eq_p.csv", eqp);
    specs.push_back({"onedoneq",
                     "generator-102-1:eq_p",
                     std::move(eqp),
                     0.0,
                     2.0,
                     0.01,
                     0.02,
                     "component",
                     true});

    auto edp = device_output_series(result,
                                    "SynchronousMachine",
                                    2,
                                    "ed_p");
    write_csv_series(out_dir / "hacdcpf_psd_onedoneq_ed_p.csv", edp);
    specs.push_back({"onedoneq",
                     "generator-102-1:ed_p",
                     std::move(edp),
                     0.0,
                     2.0,
                     0.015,
                     0.03,
                     "component",
                     true});
  }

  {
    auto sys = make_psd_simple_marconato_three_bus_case();
    DynamicSolverOptions opt = fast_options();
    opt.solver_type = DynamicSolverType::MassMatrixDae;
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    opt.dynamic_trim_tol = 1e-7;
    opt.max_dynamic_trim_iters = 20;
    opt.algebraic_network_max_iters = 8;
    opt.algebraic_network_tol = 1e-8;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    trip.label = "PSD Test 03 SimpleMarconato BUS 1-BUS 3 branch trip";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    REQUIRE(result.success);

    struct SimpleMarconatoDeviceSpec {
      int component_index;
      std::string psd_ref;
    };
    const std::vector<SimpleMarconatoDeviceSpec> devices = {
        {2, "generator-102-1"},
        {3, "generator-103-1"},
    };
    const std::vector<MachineTraceSpec> traces = {
        {"angle_rad", "delta_rad", 0.05, 0.12},
        {"omega_pu", "omega_pu", 0.002, 0.005},
        {"eq_p", "eq_p", 0.02, 0.04},
        {"ed_p", "ed_p", 0.02, 0.04},
    };
    for (const auto& device : devices) {
      for (const auto& trace : traces) {
        auto local = device_output_series(result,
                                          "SynchronousMachine",
                                          device.component_index,
                                          trace.local_key);
        write_csv_series(out_dir /
                             ("hacdcpf_psd_simple_marconato_" +
                              safe_artifact_token(device.psd_ref) + "_" +
                              safe_artifact_token(trace.psd_quantity) + ".csv"),
                         local);
        specs.push_back({"simple_marconato",
                         device.psd_ref + ":" + trace.psd_quantity,
                         std::move(local),
                         0.0,
                         2.0,
                         trace.rms_tolerance,
                         trace.max_tolerance,
                         "component",
                         true});
      }
    }
  }

  append_psse_machine_specs(
      "genroe",
      make_psd_genroe_three_bus_subset_case(),
      {{"angle_rad", "delta_rad", kPsseAngleRmsToleranceRad, kPsseAngleMaxToleranceRad},
       {"omega_pu", "omega_pu", 0.002, 0.005},
       {"eq_p", "eq_p", 0.02, 0.04},
       {"ed_p", "ed_p", 0.02, 0.04}});

  append_psse_machine_specs(
      "genroe_high_sat",
      make_psd_genroe_three_bus_subset_case(true),
      {{"angle_rad", "delta_rad", kPsseAngleRmsToleranceRad, kPsseAngleMaxToleranceRad},
       {"omega_pu", "omega_pu", 0.002, 0.005},
       {"eq_p", "eq_p", 0.02, 0.04},
       {"ed_p", "ed_p", 0.02, 0.04}});

  append_psse_machine_specs(
      "gensal",
      make_psd_gensal_three_bus_subset_case(false),
      {{"angle_rad", "delta_rad", kPsseAngleRmsToleranceRad, kPsseAngleMaxToleranceRad},
       {"omega_pu", "omega_pu", 0.002, 0.005},
       {"eq_p", "eq_p", 0.02, 0.04},
       {"psiq_pp", "psiq_pp", 0.02, 0.04}});

  append_psse_machine_specs(
      "gensae",
      make_psd_gensal_three_bus_subset_case(true),
      {{"angle_rad", "delta_rad", kPsseAngleRmsToleranceRad, kPsseAngleMaxToleranceRad},
       {"omega_pu", "omega_pu", 0.002, 0.005},
       {"eq_p", "eq_p", 0.02, 0.04},
       {"psiq_pp", "psiq_pp", 0.02, 0.04}});

  {
    auto sys = make_psd_zip_load_three_bus_subset_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent trip;
    trip.time_s = 1.0;
    trip.type = DynamicEventType::ACBranchTrip;
    trip.component_index = 1;
    trip.component_type = "AC";
    dyn.events.push_back(trip);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    REQUIRE(result.success);
    auto v103 = bus_voltage_mag_series(result, 2);
    write_csv_series(out_dir / "hacdcpf_psd_zip_constant_power_v103.csv", v103);
    specs.push_back({"zip_constant_power",
                     "bus103:voltage_mag",
                     std::move(v103),
                     0.0,
                     2.0,
                     0.006,
                     0.015,
                     "component",
                     true});
    auto v102 = bus_voltage_mag_series(result, 1);
    write_csv_series(out_dir / "hacdcpf_psd_zip_constant_power_v102.csv", v102);
    specs.push_back({"zip_constant_power",
                     "bus102:voltage_mag",
                     std::move(v102),
                     0.0,
                     2.0,
                     0.008,
                     0.020,
                     "component",
                     true});
  }

  {
    const PsdGflCase psd_system_case{
        "reduced_pll_test24",
        "test24",
        "test_case_gridfollowing.jl",
        "ReducedOrderPLL",
        2.0,
        20.0,
        1.0 / (1.32 * 2.0 * 3.14159265358979323846 * 50.0)};
    const double base_mva = 100.0;
    const DynamicResults result =
        run_hacdcpf_psd_gfl_case(psd_system_case, base_mva);
    REQUIRE(result.success);
    auto local = hacdcpf_vsc_filtered_power_series(result, base_mva);
    write_csv_series(out_dir / "hacdcpf_psd_gridfollowing_reduced_p_oc.csv", local);
    specs.push_back({"test24",
                     "generator-102-1:p_oc",
                     std::move(local),
                     0.0,
                     2.0,
                     0.35,
                     0.35,
                     "system",
                     false});
  }

  auto psd_csv_path = [&](const ExternalSpec& spec) {
    return out_dir / ("psd_" + spec.case_name + "_" +
                      safe_artifact_token(spec.signal) + ".csv");
  };

  std::vector<PsdTraceExportRequest> export_requests;
  export_requests.reserve(specs.size());
  for (const auto& spec : specs) {
    export_requests.push_back({spec.case_name, spec.signal, psd_csv_path(spec)});
  }
  const std::filesystem::path psd_batch_log = out_dir / "psd_batch.log";
  const bool exported = export_psd_traces_batch(export_requests, psd_batch_log);
  INFO("PSD export failed. Inspect " << psd_batch_log
       << ". Run `cd " << psd_repo_path()
       << " && julia --project=test -e 'using Pkg; Pkg.instantiate()'` "
       << "to install missing PSD test dependencies.");
  REQUIRE(exported);

  const std::filesystem::path pointwise_path =
      out_dir / "psd_external_pointwise_comparison.csv";
  std::ofstream pointwise(pointwise_path);
  REQUIRE(pointwise.good());
  pointwise << "level,case,signal,alignment,time_s,hacdcpf_raw,psd_raw,"
               "hacdcpf_compare,psd_compare,error,abs_error\n";

  std::vector<ValidationMetric> metrics;
  for (const auto& spec : specs) {
    const CsvSeries psd = read_csv_series(psd_csv_path(spec));
    REQUIRE(psd.t.size() > 100);
    append_pointwise_comparison_rows(pointwise,
                                     spec.level,
                                     spec.case_name,
                                     spec.signal,
                                     spec.local,
                                     psd,
                                     spec.t_start,
                                     spec.t_end,
                                     spec.compare_relative);
    const CsvSeries local_cmp =
        spec.compare_relative ? relative_to_initial(spec.local) : spec.local;
    const CsvSeries psd_cmp =
        spec.compare_relative ? relative_to_initial(psd) : psd;
    const double rms = rms_common_error(local_cmp, psd_cmp, spec.t_start, spec.t_end);
    const double max_abs =
        max_common_abs_error(local_cmp, psd_cmp, spec.t_start, spec.t_end);
    metrics.push_back({spec.level,
                       spec.case_name,
                       "PowerSimulationsDynamics.jl",
                       spec.compare_relative ? spec.signal + "_relative" : spec.signal,
                       rms,
                       max_abs,
                       spec.rms_tolerance,
                       spec.max_tolerance,
                       rms < spec.rms_tolerance && max_abs < spec.max_tolerance});
  }
  pointwise.close();
  write_validation_summary(out_dir / "psd_external_validation_summary.csv", metrics);
  for (const auto& metric : metrics) {
    INFO("PSD validation artifact directory: " << out_dir);
    INFO("PSD pointwise comparison CSV: " << pointwise_path);
    INFO(metric.level << " " << metric.case_name << " " << metric.signal
                      << " rms=" << metric.rms_error
                      << " max=" << metric.max_abs_error);
    CHECK(metric.passed);
  }
}

TEST_CASE("PSD grid-following comparison harness is available",
          "[dynamics][benchmark][psd][gridfollowing]") {
  const std::vector<PsdGflCase> cases = {
      {"reduced_pll_test24",
       "test24",
       "test_case_gridfollowing.jl",
       "ReducedOrderPLL",
       2.0,
       20.0,
       1.0 / (1.32 * 2.0 * 3.14159265358979323846 * 50.0)},
      {"kaura_pll_test51",
       "test51",
       "test_case51_gridfollowing_kaura.jl",
       "KauraPLL",
       0.084,
       4.69,
       1.0 / 500.0},
  };

  const double base_mva = 100.0;
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_gridfollowing";
  std::filesystem::create_directories(out_dir);

  for (const auto& spec : cases) {
    INFO("PSD validation case: " << spec.name);
    const DynamicResults result = run_hacdcpf_psd_gfl_case(spec, base_mva);
    REQUIRE(result.success);
    REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm < 1e-5);
    REQUIRE(result.snapshots.size() > 100);
    REQUIRE(result.final_snapshot() != nullptr);
    const double p_final = final_vsc_p_mw(result);
    CHECK(std::isfinite(p_final));
    const double p_filtered_final =
        vsc_value(*result.final_snapshot(), "p_filtered_mw");
    CHECK(p_filtered_final > 40.0);
    CHECK(p_filtered_final < 80.0);

    const CsvSeries local =
        hacdcpf_vsc_filtered_power_series(result, base_mva);
    const std::filesystem::path hacdcpf_csv =
        out_dir / ("hacdcpf_" + spec.name + "_p_oc.csv");
    write_csv_series(hacdcpf_csv, local);

    if (!run_external_psd) continue;

    const std::filesystem::path psd_csv =
        out_dir / ("psd_" + spec.name + "_p_oc.csv");
    const std::filesystem::path psd_log =
        out_dir / ("psd_" + spec.name + ".log");
    const bool exported =
        export_psd_gridfollowing_trace(spec, psd_csv, psd_log);
    INFO("PSD export failed. Inspect " << psd_log
         << ". Run `cd " << psd_repo_path()
         << " && julia --project=test -e 'using Pkg; Pkg.instantiate()'` "
         << "to install missing PSD test dependencies.");
    REQUIRE(exported);
    const CsvSeries psd = read_csv_series(psd_csv);
    REQUIRE(psd.t.size() > 100);
    const double rms = rms_common_error(local, psd, 0.0, 2.0);
    CHECK(rms < 0.35);
  }
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

namespace {

// Two-bus case with a non-slack machine (model selectable) at bus 2, optionally
// carrying an AVR and PSS, for small-signal analysis.
HybridPowerSystem make_small_signal_case(const std::string& model, bool avr, bool pss) {
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
  g2.dynamic_model.model_name = model;
  auto add = [&](const std::string& type, const std::string& m,
                 std::map<std::string, double> p) {
    hacdcpf::DynamicModelComponentProfile c;
    c.type = type; c.model = m; c.standard = "IEEE"; c.parameters = std::move(p);
    g2.dynamic_model.components.push_back(std::move(c));
  };
  if (avr) add("exciter", "SEXS", {{"Ka", 50}, {"Ta", 0.1}});
  if (pss) add("pss", "PSS1A", {{"Ks", 15}, {"Tw", 10}, {"T1", 0.15}, {"T2", 0.03},
                                {"T3", 0.15}, {"T4", 0.03}});
  sys.ac.generators = {gs, g2};
  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = 30; ld.q_mvar = 10;
  sys.ac.loads = {ld};
  return sys;
}

SmallSignalResult analyze_small_signal(const std::string& model, bool avr, bool pss) {
  DynamicSolverOptions opt;
  opt.use_consistent_dynamic_initialization = true;
  auto sys = make_small_signal_case(model, avr, pss);
  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  return small_signal_analysis(dyn);
}

// The electromechanical mode: oscillatory, machine-dominant, sub-5 Hz, least damped.
SmallSignalMode electromechanical_mode(const SmallSignalResult& r) {
  SmallSignalMode best;
  best.frequency_hz = -1.0;
  best.damping_ratio = 1e30;
  for (const auto& m : r.modes) {
    if (!m.oscillatory || m.frequency_hz < 0.1 || m.frequency_hz > 5.0) continue;
    if (m.dominant_state.find("SynchronousMachine") == std::string::npos) continue;
    if (m.damping_ratio < best.damping_ratio) best = m;
  }
  return best;
}

// Independent time-domain estimate of the swing-mode frequency (peak spacing).
double measure_swing_frequency_hz(const std::string& model) {
  DynamicSolverOptions opt;
  opt.solver_type = DynamicSolverType::TrapezoidalNewton;
  opt.t_end_s = 8.0;
  opt.dt_s = 0.002;
  opt.use_consistent_dynamic_initialization = true;
  auto sys = make_small_signal_case(model, false, false);
  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent e; e.time_s = 1.0; e.type = DynamicEventType::ACLoadScale; e.bus = 2; e.value = 1.2;
  dyn.events.push_back(e);
  DynamicSolver solver;
  const DynamicResults r = solver.solve(dyn);
  const auto w = device_output_series(r, "SynchronousMachine", 2, "omega_pu");
  double wf = w.y.empty() ? 1.0 : w.y.back();
  std::vector<double> peak_t;
  for (std::size_t i = 1; i + 1 < w.y.size(); ++i) {
    if (w.t[i] < 1.0) continue;
    const double a = w.y[i] - wf, am = w.y[i - 1] - wf, ap = w.y[i + 1] - wf;
    if (a > am && a > ap && a > 1e-6) peak_t.push_back(w.t[i]);
  }
  if (peak_t.size() < 2) return -1.0;
  return 1.0 / ((peak_t.back() - peak_t.front()) / (peak_t.size() - 1));
}

}  // namespace

TEST_CASE("Small-signal analysis eigenvalues match time-domain and rank controls",
          "[dynamics][smallsignal]") {
  SECTION("eigenvalue frequency matches the independent time-domain oscillation") {
    const SmallSignalResult ss = analyze_small_signal("ClassicalMachine", false, false);
    REQUIRE(ss.success);
    CHECK(static_cast<int>(ss.modes.size()) == ss.n_differential);
    CHECK(ss.n_algebraic > 0);
    const SmallSignalMode em = electromechanical_mode(ss);
    REQUIRE(em.frequency_hz > 0.0);
    CHECK(em.dominant_state.find("SynchronousMachine") != std::string::npos);
    CHECK(em.oscillatory);
    const double f_time = measure_swing_frequency_hz("ClassicalMachine");
    REQUIRE(f_time > 0.0);
    CHECK(std::abs(em.frequency_hz - f_time) / f_time < 0.05);  // within 5%
    // participation of each mode is a normalized distribution over states.
    for (int i = 0; i < ss.n_differential; ++i) {
      CHECK(ss.participation.row(i).sum() == Catch::Approx(1.0).margin(1e-6));
    }
  }

  SECTION("a PSS increases the electromechanical damping ratio (GENROU)") {
    const SmallSignalResult no_pss = analyze_small_signal("GENROU", true, false);
    const SmallSignalResult with_pss = analyze_small_signal("GENROU", true, true);
    REQUIRE(no_pss.success);
    REQUIRE(with_pss.success);
    const SmallSignalMode em0 = electromechanical_mode(no_pss);
    const SmallSignalMode em1 = electromechanical_mode(with_pss);
    REQUIRE(em0.frequency_hz > 0.0);
    REQUIRE(em1.frequency_hz > 0.0);
    CHECK(em0.damping_ratio > 0.0);                     // stable without PSS
    CHECK(em1.damping_ratio > em0.damping_ratio + 0.05);  // PSS materially improves damping
  }
}

TEST_CASE("Mass-matrix DAE core matches the partitioned backward-Euler oracle",
          "[dynamics][dae]") {
  // The simultaneous DAE (bus voltages as algebraic states, one sparse Newton
  // solve per step) must reproduce the nested-network backward-Euler result,
  // since both are backward Euler on the same system.
  auto run_solver = [](DynamicSolverType type, bool event) {
    DynamicSolverOptions opt;
	    opt.solver_type = type;
	    opt.t_end_s = 3.0;
	    opt.dt_s = 0.005;
	    opt.algebraic_network_max_iters = 20;
	    opt.algebraic_network_tol = 1e-10;
	    opt.use_consistent_dynamic_initialization = true;
	    auto sys = make_controlled_machine_case(false, false, false);
    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    if (event) {
      DynamicEvent e;
      e.time_s = 1.0;
      e.type = DynamicEventType::ACLoadScale;
      e.bus = 2;
      e.value = 1.5;
      dyn.events.push_back(e);
    }
    DynamicSolver solver;
    return solver.solve(dyn);
  };

  SECTION("holds the scheduled equilibrium with no event") {
    const DynamicResults dae = run_solver(DynamicSolverType::MassMatrixDae, false);
    REQUIRE(dae.success);
    const auto omega = device_output_series(dae, "SynchronousMachine", 2, "omega_pu");
    const auto p = device_output_series(dae, "SynchronousMachine", 2, "p_mw");
    CHECK(series_range(omega) < 1e-5);          // stays at synchronous speed
    CHECK(p.y.front() == Catch::Approx(50.0).margin(0.5));  // delivers scheduled MW
    CHECK(series_range(p) < 1e-2);
  }

  SECTION("matches BackwardEulerNewton under a load step") {
    const DynamicResults dae = run_solver(DynamicSolverType::MassMatrixDae, true);
    const DynamicResults be = run_solver(DynamicSolverType::BackwardEulerNewton, true);
    REQUIRE(dae.success);
    REQUIRE(be.success);
    const auto od = device_output_series(dae, "SynchronousMachine", 2, "omega_pu");
    const auto ob = device_output_series(be, "SynchronousMachine", 2, "omega_pu");
    const auto pd = device_output_series(dae, "SynchronousMachine", 2, "p_mw");
    const auto pb = device_output_series(be, "SynchronousMachine", 2, "p_mw");
    REQUIRE(od.y.size() == ob.y.size());
    double omega_maxdiff = 0.0;
    double p_maxdiff = 0.0;
    for (std::size_t i = 0; i < od.y.size(); ++i) {
      omega_maxdiff = std::max(omega_maxdiff, std::abs(od.y[i] - ob.y[i]));
      p_maxdiff = std::max(p_maxdiff, std::abs(pd.y[i] - pb.y[i]));
    }
    CHECK(omega_maxdiff < 1e-6);
    CHECK(p_maxdiff < 1e-3);
  }
}

TEST_CASE("Machine governor / AVR / PSS control blocks are wired and effective",
          "[dynamics][controls]") {
  const DynamicResults base = run_controlled_case(false, false, false);
  const DynamicResults gov = run_controlled_case(true, false, false);
  const DynamicResults avr = run_controlled_case(true, true, false);
  const DynamicResults pss = run_controlled_case(true, true, true);

  REQUIRE(base.success);
  REQUIRE(gov.success);
  REQUIRE(avr.success);
  REQUIRE(pss.success);

  // Controllers must not disturb the pre-event equilibrium.
  CHECK(pss.initialization.dynamic_trim_converged);
  CHECK(pss.initialization.dynamic_fast_dxdt_inf_norm <= 1e-6);

  // Governor: mechanical power tracks the machine's scheduled dispatch at t0 and
  // then responds to the disturbance (it is no longer a dead constant).
  const auto pmech = device_output_series(gov, "Governor", 2, "p_mech_mw");
  CHECK(pmech.y.front() == Catch::Approx(50.0).margin(1.0));
  CHECK(series_swing_after(pmech, 1.0) > 1e-3);

  // AVR: with excitation control the terminal voltage is held closer to its
  // pre-event level than the uncontrolled machine, and the field voltage moves.
  const auto v_base = device_output_series(base, "SynchronousMachine", 2, "v_pos_pu");
  const auto v_avr = device_output_series(avr, "SynchronousMachine", 2, "v_pos_pu");
  const double v0 = v_base.y.front();
  CHECK(std::abs(series_mean_after(v_avr, 1.0) - v0) <
        std::abs(series_mean_after(v_base, 1.0) - v0));
  const auto efd = device_output_series(avr, "Exciter", 2, "efd_pu");
  CHECK(series_swing_after(efd, 1.0) > 1e-3);

  // PSS: the stabilizer improves damping of the electromechanical swing, so the
  // post-event frequency swing is smaller than with AVR alone.
  const auto f_avr = device_output_series(avr, "SynchronousMachine", 2, "frequency_hz");
  const auto f_pss = device_output_series(pss, "SynchronousMachine", 2, "frequency_hz");
  CHECK(series_swing_after(f_pss, 1.05) < series_swing_after(f_avr, 1.05));
  // PSS output is a bounded stabilizing signal that starts at ~0 (equilibrium).
  const auto vs = device_output_series(pss, "PSS", 2, "vs_pu");
  CHECK(std::abs(vs.y.front()) < 1e-6);
  CHECK(series_swing_after(vs, 1.0) > 1e-4);
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

TEST_CASE("DynamicModelBuilder wires Phase 5 five-mass shaft blocks",
          "[dynamics][shaft][transient]") {
  auto sys = make_controlled_machine_case(false, false, false);
  REQUIRE(sys.ac.generators.size() >= 2);
  hacdcpf::DynamicModelComponentProfile shaft;
  shaft.type = "shaft";
  shaft.model = "FiveMassShaft";
  shaft.standard = "PowerSimulationsDynamics";
  shaft.parameters = {
      {"H1", 0.5}, {"H2", 0.6}, {"H3", 0.7}, {"H4", 0.8}, {"H5", 1.0},
      {"K12", 20.0}, {"K23", 18.0}, {"K34", 16.0}, {"K45", 14.0},
  };
  sys.ac.generators[1].dynamic_model.components.push_back(shaft);

  DynamicSolverOptions opt;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.01;
  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  const auto shaft_it = std::find_if(dyn.devices.begin(),
                                     dyn.devices.end(),
                                     [](const std::unique_ptr<DynamicDevice>& device) {
                                       return device->type() == "Shaft" &&
                                              device->modelName() == "FiveMassShaft";
                                     });
  REQUIRE(shaft_it != dyn.devices.end());
  std::string error;
  REQUIRE(dyn.solveNetwork(opt.t_start_s, error));
  Eigen::VectorXd dxdt;
  REQUIRE(dyn.evaluateDerivatives(opt.t_start_s, dyn.x.x, dxdt, error));
  CHECK(dxdt.allFinite());

  DynamicSolver solver;
  DynamicResults r = solver.solve(dyn);
  REQUIRE(r.success);
  REQUIRE(r.final_snapshot() != nullptr);
  const auto& outputs = r.final_snapshot()->device_outputs;
  const auto out_it = std::find_if(outputs.begin(), outputs.end(), [](const DynamicDeviceOutput& out) {
    return out.type == "Shaft" && out.model_name == "FiveMassShaft";
  });
  REQUIRE(out_it != outputs.end());
  CHECK(out_it->values.at("state_count") == Catch::Approx(10.0));
  CHECK(out_it->values.at("mass_count") == Catch::Approx(5.0));
  CHECK(std::abs(out_it->values.at("omega_generator_pu") - 1.0) < 0.05);
}

TEST_CASE("Phase 5 dynamic RL line matches PSID branch current equations",
          "[dynamics][branch][numerical]") {
  DynamicRLLineParams params;
  params.component_index = 12;
  params.from_bus = 1;
  params.to_bus = 2;
  params.from_pos = 0;
  params.to_pos = 1;
  params.frequency_hz = 50.0;
  params.r_pu = 0.02;
  params.x_pu = 0.08;
  DynamicRLLine line(params);

  int offset = 0;
  line.assignStateIndices(offset);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  x.x[0] = 0.20;
  x.x[1] = -0.10;

  NetworkState y;
  y.resize(6, 0);
  const Complex vf = std::polar(1.04, 0.06);
  const Complex vt = std::polar(0.97, -0.03);
  set_balanced_voltage(y, 0, vf);
  set_balanced_voltage(y, 1, vt);

  Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
  line.computeDerivatives(0.0, x, y, dxdt);

  constexpr double pi = 3.141592653589793238462643383279502884;
  const Complex dv = vf - vt;
  const double omega_b = 2.0 * pi * 50.0;
  CHECK(dxdt[0] == Catch::Approx(
                      (omega_b / 0.08) * (dv.real() - (0.02 * 0.20 - 0.08 * -0.10))));
  CHECK(dxdt[1] == Catch::Approx(
                      (omega_b / 0.08) * (dv.imag() - (0.02 * -0.10 + 0.08 * 0.20))));

  DynamicStamp stamp(6, 0);
  line.stamp(0.0, x, y, stamp);
  const Complex ia(0.20, -0.10);
  CHECK(stamp.Iac[0].real() == Catch::Approx(-ia.real()));
  CHECK(stamp.Iac[0].imag() == Catch::Approx(-ia.imag()));
  CHECK(stamp.Iac[3].real() == Catch::Approx(ia.real()));
  CHECK(stamp.Iac[3].imag() == Catch::Approx(ia.imag()));

  DynamicState x_eq;
  x_eq.resize(static_cast<std::size_t>(offset));
  line.initializeFromPowerFlow(PowerFlowResult{}, x_eq, y);
  Eigen::VectorXd dxdt_eq = Eigen::VectorXd::Zero(offset);
  line.computeDerivatives(0.0, x_eq, y, dxdt_eq);
  CHECK(dxdt_eq[0] == Catch::Approx(0.0).margin(1e-9));
  CHECK(dxdt_eq[1] == Catch::Approx(0.0).margin(1e-9));

  DynamicEvent trip;
  trip.type = DynamicEventType::ACBranchTrip;
  trip.component_index = 12;
  line.handleEvent(trip, x_eq, y);
  CHECK(x_eq.x[0] == Catch::Approx(0.0));
  CHECK(x_eq.x[1] == Catch::Approx(0.0));
  DynamicEvent close;
  close.type = DynamicEventType::ACBranchClose;
  close.component_index = 12;
  line.handleEvent(close, x_eq, y);
  const Complex i_eq = dv / Complex(0.02, 0.08);
  CHECK(x_eq.x[0] == Catch::Approx(i_eq.real()));
  CHECK(x_eq.x[1] == Catch::Approx(i_eq.imag()));
}

TEST_CASE("DynamicModelBuilder wires Phase 5 dynamic RL branches",
          "[dynamics][branch][transient]") {
  auto sys = make_transient_2bus();
  REQUIRE_FALSE(sys.ac.branches.empty());
  sys.ac.branches.front().dynamic_rl = true;
  sys.ac.loads.clear();
  ExternalGrid remote_grid;
  remote_grid.index = 20;
  remote_grid.bus = 2;
  remote_grid.vm_pu = 1.0;
  remote_grid.va_deg = 0.0;
  remote_grid.in_service = true;
  sys.ac.external_grids = {remote_grid};

  auto opt = fast_options();
  opt.solver_type = DynamicSolverType::BackwardEulerNewton;
  opt.t_end_s = 2e-4;
  opt.dt_s = 1e-4;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  const auto line_it = std::find_if(dyn.devices.begin(),
                                    dyn.devices.end(),
                                    [](const std::unique_ptr<DynamicDevice>& device) {
                                      return device->type() == "DynamicRLLine";
                                    });
  REQUIRE(line_it != dyn.devices.end());
  REQUIRE_FALSE(dyn.network.ac_branches.empty());
  CHECK(dyn.network.ac_branches.front().y_ft.norm() == Catch::Approx(0.0));
  CHECK(dyn.network.ac_branches.front().y_tf.norm() == Catch::Approx(0.0));

  std::string error;
  REQUIRE(dyn.solveNetwork(opt.t_start_s, error));
  Eigen::VectorXd dxdt;
  REQUIRE(dyn.evaluateDerivatives(opt.t_start_s, dyn.x.x, dxdt, error));
  CHECK(dxdt.allFinite());

  const DynamicDeviceOutput out = (*line_it)->output(dyn.x, dyn.y);
  CHECK(out.type == "DynamicRLLine");
  CHECK(out.values.at("current_mag_pu") >= 0.0);
  CHECK(out.values.at("in_service") == Catch::Approx(1.0));
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

  INFO(result.message);
  REQUIRE(result.success);
  REQUIRE(result.applied_event_records.size() == 2);
  CHECK(result.applied_event_records.front().label == "parameterized AC fault");
  CHECK(result.applied_event_records.front().params.at("r_pu") == Catch::Approx(0.001));
  CHECK(result.applied_event_records.back().params.at("p_ref_mw") == Catch::Approx(-2.0));
}

TEST_CASE("Adaptive transient integration remains bounded after cleared AC fault",
          "[dynamics][events][fault][long-run]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 10.0;
  opt.dt_s = 0.02;
  opt.use_adaptive_step = true;
  opt.abs_tol = 1e-7;
  opt.rel_tol = 1e-5;
  opt.output_every_steps = 20;
  opt.dynamic_trim_tol = 1e-7;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  DynamicEvent fault;
  fault.time_s = 1.0;
  fault.type = DynamicEventType::FaultShunt;
  fault.bus = 2;
  fault.component_type = "AC";
  fault.label = "cleared AC fault";
  fault.params["r_pu"] = 0.02;
  fault.params["x_pu"] = 0.02;
  fault.params["duration_s"] = 0.06;
  dyn.events.push_back(fault);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.max_local_error_norm <= 1.0001);
  REQUIRE(result.final_snapshot() != nullptr);
  CHECK(result.final_snapshot()->time_s == Catch::Approx(10.0));
  CHECK(result.final_snapshot()->min_ac_voltage_pu > 0.70);
  CHECK(result.final_snapshot()->max_ac_voltage_pu < 1.30);

  for (const auto& snapshot : result.snapshots) {
    if (snapshot.time_s < 2.0) continue;
    CHECK(snapshot.min_ac_voltage_pu > 0.65);
    CHECK(snapshot.max_ac_voltage_pu < 1.35);
  }
}

TEST_CASE("Transient snapshots can skip per-device telemetry for fast UI runs",
          "[dynamics][performance][snapshots]") {
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.03;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.record_device_outputs = false;

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  INFO(result.message);
  REQUIRE(result.success);
  REQUIRE_FALSE(result.snapshots.empty());
  CHECK(result.final_snapshot() != nullptr);
  for (const auto& snapshot : result.snapshots) {
    CHECK(snapshot.device_outputs.empty());
    CHECK(snapshot.vac_abc.size() > 0);
  }
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
