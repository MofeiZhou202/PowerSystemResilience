#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
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

HybridPowerSystem make_psd_test01_omib_case() {
  HybridPowerSystem sys;
  sys.name = "psd_test01_omib";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 60.0;

  ACBus b101;
  b101.index = 101;
  b101.name = "BUS 1";
  b101.bus_type = BusType::SLACK;
  b101.base_kv = 230.0;
  b101.vm_pu = 1.05;
  b101.va_deg = 0.0;
  b101.in_service = true;

  ACBus b102;
  b102.index = 102;
  b102.name = "BUS 2";
  b102.bus_type = BusType::PV;
  b102.base_kv = 230.0;
  b102.vm_pu = 1.04;
  b102.va_deg = 1.3118;
  b102.in_service = true;

  sys.ac.buses = {b101, b102};

  auto branch = [](int index) {
    ACBranch br;
    br.index = index;
    br.from_bus = 101;
    br.to_bus = 102;
    br.r_pu = 0.0;
    br.x_pu = 0.1;
    br.b_pu = 0.0;
    br.tap = 1.0;
    br.name = "BUS 1-BUS 2-i_" + std::to_string(index);
    br.in_service = true;
    return br;
  };
  sys.ac.branches = {branch(1), branch(2)};

  ExternalGrid source;
  source.index = 1;
  source.bus = 101;
  source.name = "InfBus";
  source.vm_pu = 1.05;
  source.va_deg = 0.0;
  source.r_pu = 0.0;
  source.x_pu = 1.0e-5;
  source.in_service = true;
  sys.ac.external_grids = {source};

  Generator gen;
  gen.index = 1;
  gen.bus = 102;
  gen.name = "generator-102-1";
  gen.is_slack = false;
  gen.in_service = true;
  gen.pg_mw = 50.0;
  gen.qg_mvar = -20.228;
  gen.vg_pu = 1.04;
  gen.pmax_mw = 100.0;
  gen.pmin_mw = 0.0;
  gen.qmax_mvar = 100.0;
  gen.qmin_mvar = -100.0;
  gen.ra_pu = 0.0;
  gen.xd_pu = 0.2995;
  gen.xq_pu = 0.2995;
  gen.xdp_pu = 0.2995;
  gen.xdpp_pu = 0.2995;
  gen.td0p_s = 0.0;
  gen.td0pp_s = 0.0;
  gen.inertia_h = 3.148;
  gen.droop_r = 0.05;
  gen.dynamic_model.standard = "PowerSystems";
  gen.dynamic_model.model_name = "ClassicalMachine";
  gen.dynamic_model.source_id = "PowerSimulationsDynamics:test_case01_omib";
  gen.dynamic_model.parameters = {
      {"H", 3.148},
      {"D", 2.0},
      {"R", 0.0},
      {"Xd_p", 0.2995},
      {"eq_p", 0.7087},
  };
  sys.ac.generators = {gen};

  return sys;
}

HybridPowerSystem make_psd_simple_marconato_three_bus_case();
void apply_psd_onedoneq_profile(Generator& machine,
                                std::string source_id,
                                bool avr_type1);

HybridPowerSystem make_psd_onedoneq_three_bus_subset_case() {
  HybridPowerSystem sys = make_psd_simple_marconato_three_bus_case();
  sys.name = "psd_onedoneq_three_bus_subset";
  REQUIRE(sys.ac.generators.size() == 2);
  for (auto& machine : sys.ac.generators) {
    apply_psd_onedoneq_profile(
        machine, "PowerSimulationsDynamics:test_case02_onedoneq", true);
  }
  return sys;
}

void apply_psd_onedoneq_profile(Generator& machine,
                                std::string source_id,
                                bool avr_type1 = false) {
  machine.dynamic_model.standard = "PowerSystems";
  machine.dynamic_model.model_name = "OneDOneQMachine";
  machine.dynamic_model.source_id = std::move(source_id);
  machine.ra_pu = 0.0;
  machine.xd_pu = 1.3125;
  machine.xq_pu = 1.2578;
  machine.xdp_pu = 0.1813;
  machine.xdpp_pu = 0.0;
  machine.td0p_s = 5.89;
  machine.td0pp_s = 0.0;
  machine.inertia_h = 3.01;
  machine.droop_r = 0.05;
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
  machine.dynamic_model.components.clear();
  if (avr_type1) {
    hacdcpf::DynamicModelComponentProfile avr;
    avr.type = "exciter";
    avr.model = "AVRTypeI";
    avr.standard = "PowerSystems";
    avr.parameter_set = "PowerSimulationsDynamics:avr_type1";
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
}

HybridPowerSystem make_psd_test13_onedoneq_avr_tg_case() {
  HybridPowerSystem sys = make_psd_simple_marconato_three_bus_case();
  sys.name = "psd_test13_onedoneq_avr_tg";
  REQUIRE(sys.ac.generators.size() == 2);
  apply_psd_onedoneq_profile(sys.ac.generators[0],
                             "PowerSimulationsDynamics:test13_avrtype2_tgtype1");
  apply_psd_onedoneq_profile(sys.ac.generators[1],
                             "PowerSimulationsDynamics:test13_avrsimple");

  hacdcpf::DynamicModelComponentProfile avr2;
  avr2.type = "exciter";
  avr2.model = "AVRTypeII";
  avr2.standard = "PowerSystems";
  avr2.parameter_set = "PowerSimulationsDynamics:test13_avrtype2";
  avr2.parameters = {
      {"K0", 200.0},
      {"T1", 4.0},
      {"T2", 1.0},
      {"T3", 0.006},
      {"T4", 0.06},
      {"Te", 0.0001},
      {"Tr", 0.0001},
      {"Va_min", -50.0},
      {"Va_max", 50.0},
      {"Ae", 0.0},
      {"Be", 0.0},
  };
  sys.ac.generators[0].dynamic_model.components.push_back(std::move(avr2));

  hacdcpf::DynamicModelComponentProfile tg1;
  tg1.type = "governor";
  tg1.model = "TGTypeI";
  tg1.standard = "PowerSystems";
  tg1.parameter_set = "PowerSimulationsDynamics:test13_tgtype1";
  tg1.parameters = {
      {"R", 0.02},
      {"Ts", 0.1},
      {"Tc", 0.45},
      {"T3", 0.0},
      {"T4", 12.0},
      {"T5", 50.0},
      {"pmin_mw", 30.0},
      {"pmax_mw", 120.0},
  };
  sys.ac.generators[0].dynamic_model.components.push_back(std::move(tg1));

  hacdcpf::DynamicModelComponentProfile avrs;
  avrs.type = "exciter";
  avrs.model = "AVRSimple";
  avrs.standard = "PowerSystems";
  avrs.parameter_set = "PowerSimulationsDynamics:test13_avrsimple";
  avrs.parameters = {{"Kv", 500.0}};
  sys.ac.generators[1].dynamic_model.components.push_back(std::move(avrs));

  for (auto& load : sys.ac.loads) {
    load.dynamic_model.source_id = "PowerSimulationsDynamics:test13_avrs";
  }
  return sys;
}

HybridPowerSystem make_psd_test07_five_mass_shaft_case() {
  HybridPowerSystem sys = make_psd_simple_marconato_three_bus_case();
  sys.name = "psd_test07_five_mass_shaft";
  REQUIRE(sys.ac.generators.size() == 2);
  for (auto& machine : sys.ac.generators) {
    machine.pg_mw = 75.0;
    apply_psd_onedoneq_profile(machine,
                               "PowerSimulationsDynamics:test07_five_mass",
                               true);
  }

  hacdcpf::DynamicModelComponentProfile shaft;
  shaft.type = "shaft";
  shaft.model = "FiveMassShaft";
  shaft.standard = "PowerSystems";
  shaft.parameter_set = "PowerSimulationsDynamics:test07_five_mass";
  shaft.parameters = {
      {"H1", 0.3348},
      {"H2", 0.7306},
      {"H3", 0.8154},
      {"H4", 0.0452},
      {"H5", 3.01},
      {"D1", 0.5180},
      {"D2", 0.2240},
      {"D3", 0.2240},
      {"D4", 0.1450},
      {"D12", 0.0518},
      {"D23", 0.0224},
      {"D34", 0.0224},
      {"D45", 0.0145},
      {"K12", 33.07},
      {"K23", 28.59},
      {"K34", 44.68},
      {"K45", 21.984},
  };
  sys.ac.generators[1].dynamic_model.components.push_back(std::move(shaft));
  for (auto& load : sys.ac.loads) {
    load.dynamic_model.source_id = "PowerSimulationsDynamics:test07_five_mass";
  }
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

HybridPowerSystem make_psd_marconato_three_bus_case() {
  HybridPowerSystem sys = make_psd_simple_marconato_three_bus_case();
  sys.name = "psd_marconato_three_bus";
  for (auto& machine : sys.ac.generators) {
    machine.dynamic_model.standard = "PowerSystems";
    machine.dynamic_model.model_name = "MarconatoMachine";
    machine.dynamic_model.source_id = "PowerSimulationsDynamics:test_case04_marconato";
  }
  for (auto& load : sys.ac.loads) {
    load.dynamic_model.source_id = "PowerSimulationsDynamics:test_case04_marconato";
  }
  return sys;
}

void apply_psd_anderson_family_profile(Generator& machine,
                                       const std::string& model_name,
                                       const std::string& source_id) {
  machine.ra_pu = 0.0;
  machine.xd_pu = 0.8979;
  machine.xq_pu = 0.646;
  machine.xdp_pu = 0.2995;
  machine.xdpp_pu = 0.23;
  machine.td0p_s = 3.0;
  machine.td0pp_s = 0.01;
  machine.inertia_h = 3.01;
  machine.droop_r = 0.05;
  machine.dynamic_model.standard = "PowerSystems";
  machine.dynamic_model.model_name = model_name;
  machine.dynamic_model.source_id = source_id;
  machine.dynamic_model.parameters = {
      {"H", 3.01},
      {"D", 0.0},
      {"R", 0.0},
      {"Xd", 0.8979},
      {"Xq", 0.646},
      {"Xd_p", 0.2995},
      {"Xq_p", 0.646},
      {"Xd_pp", 0.23},
      {"Xq_pp", 0.4},
      {"Td0_p", 3.0},
      {"Tq0_p", 0.1},
      {"Td0_pp", 0.01},
      {"Tq0_pp", 0.033},
  };
}

HybridPowerSystem make_psd_simple_af_three_bus_case() {
  HybridPowerSystem sys = make_psd_simple_marconato_three_bus_case();
  sys.name = "psd_simple_af_three_bus";
  for (auto& machine : sys.ac.generators) {
    apply_psd_anderson_family_profile(machine,
                                      "SimpleAFMachine",
                                      "PowerSimulationsDynamics:test_case05_simple_af");
  }
  for (auto& load : sys.ac.loads) {
    load.dynamic_model.source_id = "PowerSimulationsDynamics:test_case05_simple_af";
  }
  return sys;
}

HybridPowerSystem make_psd_anderson_fouad_three_bus_case() {
  HybridPowerSystem sys = make_psd_simple_marconato_three_bus_case();
  sys.name = "psd_anderson_fouad_three_bus";
  for (auto& machine : sys.ac.generators) {
    apply_psd_anderson_family_profile(machine,
                                      "AndersonFouadMachine",
                                      "PowerSimulationsDynamics:test_case06_anderson_fouad");
  }
  for (auto& load : sys.ac.loads) {
    load.dynamic_model.source_id = "PowerSimulationsDynamics:test_case06_anderson_fouad";
  }
  return sys;
}

void apply_psd_test12_classical_profile(Generator& machine,
                                        double eq_p,
                                        bool with_tg_type2) {
  machine.ra_pu = 0.0;
  machine.xd_pu = 0.2995;
  machine.xq_pu = 0.2995;
  machine.xdp_pu = 0.2995;
  machine.xdpp_pu = 0.2995;
  machine.td0p_s = 0.0;
  machine.td0pp_s = 0.0;
  machine.inertia_h = 3.148;
  machine.droop_r = 0.05;
  machine.dynamic_model.standard = "PowerSystems";
  machine.dynamic_model.model_name = "ClassicalMachine";
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test_case12_multimachine";
  machine.dynamic_model.parameters = {
      {"H", 3.148},
      {"D", 2.0},
      {"R", 0.0},
      {"Xd_p", 0.2995},
      {"eq_p", eq_p},
  };
  machine.dynamic_model.components.clear();

  if (with_tg_type2) {
    hacdcpf::DynamicModelComponentProfile governor;
    governor.type = "governor";
    governor.model = "TGTypeII";
    governor.standard = "PowerSystems";
    governor.parameter_set = "PowerSimulationsDynamics:test_case12_multimachine";
    governor.parameters = {
        {"R", 0.05},
        {"T1", 1.0},
        {"T2", 2.0},
        {"Vmin", 0.1},
        {"Vmax", 1.5},
    };
    machine.dynamic_model.components.push_back(std::move(governor));
  }
}

HybridPowerSystem make_psd_test12_multimachine_tgtype2_case() {
  HybridPowerSystem sys;
  sys.name = "psd_test12_multimachine_tgtype2";
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
  b103.bus_type = BusType::PQ;
  b103.base_kv = 138.0;
  b103.vm_pu = 1.0;
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
    br.b_pu = 0.0;
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

  Generator g101;
  g101.index = 1;
  g101.bus = 101;
  g101.name = "generator-101-1";
  g101.is_slack = false;
  g101.in_service = true;
  g101.pg_mw = 0.0;
  g101.qg_mvar = 0.0;
  g101.vg_pu = 1.02;
  g101.pmax_mw = 318.0;
  g101.pmin_mw = 0.0;
  g101.qmax_mvar = 100.0;
  g101.qmin_mvar = -100.0;
  apply_psd_test12_classical_profile(g101, 1.0901, false);

  Generator g102;
  g102.index = 2;
  g102.bus = 102;
  g102.name = "generator-102-1";
  g102.is_slack = false;
  g102.in_service = true;
  g102.pg_mw = 0.0;
  g102.qg_mvar = 0.0;
  g102.vg_pu = 1.0142;
  g102.pmax_mw = 318.0;
  g102.pmin_mw = 0.0;
  g102.qmax_mvar = 100.0;
  g102.qmin_mvar = -100.0;
  apply_psd_test12_classical_profile(g102, 0.9516, true);

  sys.ac.generators = {g101, g102};
  return sys;
}

void apply_psd_sauerpai_profile(Generator& machine) {
  machine.ra_pu = 0.002;
  machine.xd_pu = 1.79;
  machine.xq_pu = 1.71;
  machine.xdp_pu = 0.169;
  machine.xdpp_pu = 0.135;
  machine.td0p_s = 4.3;
  machine.td0pp_s = 0.032;
  machine.inertia_h = 3.01;
  machine.droop_r = 0.05;
  machine.dynamic_model.standard = "PowerSystems";
  machine.dynamic_model.model_name = "SauerPaiMachine";
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test_case45_sauerpai";
  machine.dynamic_model.parameters = {
      {"H", 3.01},
      {"D", 0.0},
      {"R", 0.002},
      {"Xd", 1.79},
      {"Xq", 1.71},
      {"Xd_p", 0.169},
      {"Xq_p", 0.228},
      {"Xd_pp", 0.135},
      {"Xq_pp", 0.2},
      {"Xl", 0.13},
      {"Td0_p", 4.3},
      {"Tq0_p", 0.85},
      {"Td0_pp", 0.032},
      {"Tq0_pp", 0.05},
  };
  machine.dynamic_model.components.clear();

  hacdcpf::DynamicModelComponentProfile avr;
  avr.type = "exciter";
  avr.model = "AVRTypeI";
  avr.standard = "PowerSystems";
  avr.parameter_set = "PowerSimulationsDynamics:test_case45_sauerpai";
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

HybridPowerSystem make_psd_test45_sauerpai_machine_case() {
  HybridPowerSystem sys;
  sys.name = "psd_test45_sauerpai_machine_subset";
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
  b103.bus_type = BusType::PQ;
  b103.base_kv = 138.0;
  b103.vm_pu = 1.0;
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
    br.b_pu = 0.05;
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

  Generator g101;
  g101.index = 1;
  g101.bus = 101;
  g101.name = "generator-101-1";
  g101.is_slack = false;
  g101.in_service = true;
  g101.pg_mw = 50.0;
  g101.qg_mvar = 10.0;
  g101.vg_pu = 1.02;
  g101.pmax_mw = 318.0;
  g101.pmin_mw = 0.0;
  g101.qmax_mvar = 100.0;
  g101.qmin_mvar = -100.0;
  apply_psd_sauerpai_profile(g101);

  Generator g102;
  g102.index = 2;
  g102.bus = 102;
  g102.name = "generator-102-1";
  g102.is_slack = false;
  g102.in_service = true;
  g102.pg_mw = 50.0;
  g102.qg_mvar = 10.0;
  g102.vg_pu = 1.0142;
  g102.pmax_mw = 318.0;
  g102.pmin_mw = 0.0;
  g102.qmax_mvar = 100.0;
  g102.qmin_mvar = -100.0;
  apply_psd_test12_classical_profile(g102, 1.0, false);

  sys.ac.generators = {g101, g102};

  Load l103a;
  l103a.index = 1;
  l103a.bus = 103;
  l103a.name = "load-103-1";
  l103a.p_mw = 50.0;
  l103a.q_mvar = 5.0;
  l103a.in_service = true;
  l103a.dynamic_model.standard = "PowerSystems";
  l103a.dynamic_model.model_name = "ConstantImpedanceLoad";
  l103a.dynamic_model.source_id = "PowerSimulationsDynamics:test_case45_sauerpai";
  l103a.dynamic_model.parameters["phase_power_scale"] = 1.0;
  Load l103b = l103a;
  l103b.index = 2;
  l103b.name = "load-103-2";
  sys.ac.loads = {l103a, l103b};

  return sys;
}

void apply_psd_test25_marconato_profile(Generator& machine, bool with_tg_type2) {
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
  machine.dynamic_model.model_name = "MarconatoMachine";
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test_case25_dynamic_lines";
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
  machine.dynamic_model.components.clear();

  hacdcpf::DynamicModelComponentProfile avr;
  avr.type = "exciter";
  avr.model = "AVRSimple";
  avr.standard = "PowerSystems";
  avr.parameter_set = "PowerSimulationsDynamics:test_case25_dynamic_lines";
  avr.parameters = {{"Kv", 1.0}};
  machine.dynamic_model.components.push_back(std::move(avr));

  if (with_tg_type2) {
    hacdcpf::DynamicModelComponentProfile governor;
    governor.type = "governor";
    governor.model = "TGTypeII";
    governor.standard = "PowerSystems";
    governor.parameter_set = "PowerSimulationsDynamics:test_case25_dynamic_lines";
    governor.parameters = {
        {"R", 0.05},
        {"T1", 1.0},
        {"T2", 2.0},
        {"pmin_mw", 10.0},
        {"pmax_mw", 150.0},
    };
    machine.dynamic_model.components.push_back(std::move(governor));
  }
}

HybridPowerSystem make_psd_test25_dynamic_line_case() {
  HybridPowerSystem sys;
  sys.name = "psd_test25_marconato_dynamic_lines";
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
  b102.va_deg = -0.44941;
  b102.in_service = true;

  ACBus b103;
  b103.index = 103;
  b103.name = "BUS 3";
  b103.bus_type = BusType::PQ;
  b103.base_kv = 138.0;
  b103.vm_pu = 1.0;
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
    br.b_pu = 0.0;
    br.tap = 1.0;
    br.in_service = true;
    br.dynamic_rl = true;
    br.name = "BUS " + std::to_string(from - 100) + "-BUS " +
              std::to_string(to - 100) + "-i_1";
    return br;
  };
  sys.ac.branches = {
      branch(1, 101, 103),
      branch(2, 101, 102),
      branch(3, 102, 103),
  };

  Generator g101;
  g101.index = 1;
  g101.bus = 101;
  g101.name = "generator-101-1";
  g101.is_slack = false;
  g101.in_service = true;
  g101.pg_mw = 101.70567895312919;
  g101.qg_mvar = 24.501857920705278;
  g101.vg_pu = 1.02;
  g101.pmax_mw = 150.0;
  g101.pmin_mw = 0.0;
  g101.qmax_mvar = 100.0;
  g101.qmin_mvar = -100.0;
  apply_psd_test25_marconato_profile(g101, true);

  Generator g102;
  g102.index = 2;
  g102.bus = 102;
  g102.name = "generator-102-1";
  g102.is_slack = false;
  g102.in_service = true;
  g102.pg_mw = 80.0;
  g102.qg_mvar = 10.751144041475372;
  g102.vg_pu = 1.0142;
  g102.pmax_mw = 150.0;
  g102.pmin_mw = 0.0;
  g102.qmax_mvar = 100.0;
  g102.qmin_mvar = -100.0;
  apply_psd_test25_marconato_profile(g102, false);

  sys.ac.generators = {g101, g102};

  Load load;
  load.index = 1;
  load.bus = 103;
  load.name = "load-103-1";
  load.p_mw = 180.0;
  load.q_mvar = 30.0;
  load.in_service = true;
  load.dynamic_model.standard = "PowerSystems";
  load.dynamic_model.model_name = "ConstantImpedanceLoad";
  load.dynamic_model.source_id = "PowerSimulationsDynamics:test_case25_dynamic_lines";
  load.dynamic_model.parameters["phase_power_scale"] = 1.0;
  sys.ac.loads = {load};

  return sys;
}

HybridPowerSystem make_psd_csvgn1_three_bus_case() {
  HybridPowerSystem sys;
  sys.name = "psd_test49_csvgn1_three_bus";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1;
  b1.index = 1;
  b1.name = "GEN";
  b1.bus_type = BusType::SLACK;
  b1.base_kv = 20.0;
  b1.vm_pu = 1.0;
  b1.va_deg = 0.0;
  b1.in_service = true;

  ACBus b2;
  b2.index = 2;
  b2.name = "LOAD";
  b2.bus_type = BusType::PQ;
  b2.base_kv = 275.0;
  b2.vm_pu = 1.04785;
  b2.va_deg = -63.3086;
  b2.in_service = true;

  ACBus b3;
  b3.index = 3;
  b3.name = "LOAD_CSVGN1";
  b3.bus_type = BusType::PV;
  b3.base_kv = 275.0;
  b3.vm_pu = 1.015;
  b3.va_deg = -63.4538;
  b3.in_service = true;
  sys.ac.buses = {b1, b2, b3};

  ACBranch xfmr;
  xfmr.index = 1;
  xfmr.from_bus = 1;
  xfmr.to_bus = 2;
  xfmr.name = "GEN-LOAD-transformer";
  xfmr.r_pu = 0.0;
  xfmr.x_pu = 0.02550;
  xfmr.b_pu = 0.0;
  xfmr.tap = 0.95240;
  xfmr.in_service = true;

  auto line23 = [](int index, const std::string& circuit) {
    ACBranch br;
    br.index = index;
    br.from_bus = 2;
    br.to_bus = 3;
    br.name = "LOAD-LOAD_CSVGN1-" + circuit;
    br.r_pu = 0.02300;
    br.x_pu = 0.15000;
    br.b_pu = 0.56000;
    br.tap = 1.0;
    br.in_service = true;
    return br;
  };
  sys.ac.branches = {xfmr, line23(2, "1"), line23(3, "2")};

  ExternalGrid source;
  source.index = 1;
  source.bus = 1;
  source.name = "generator-1-1";
  source.vm_pu = 1.0;
  source.va_deg = 0.0;
  source.r_pu = 0.0;
  source.x_pu = 1.0e-5;
  source.in_service = true;

  ExternalGrid csvgn1_source;
  csvgn1_source.index = 2;
  csvgn1_source.bus = 3;
  csvgn1_source.name = "CSVGN1";
  csvgn1_source.vm_pu = 1.015;
  csvgn1_source.va_deg = -63.4538;
  csvgn1_source.r_pu = 0.0;
  csvgn1_source.x_pu = 1.0e-5;
  csvgn1_source.in_service = true;
  sys.ac.external_grids = {source, csvgn1_source};

  auto load = [](int index, int bus, const std::string& name,
                 double p_mw, double q_mvar) {
    Load ld;
    ld.index = index;
    ld.bus = bus;
    ld.name = name;
    ld.p_mw = p_mw;
    ld.q_mvar = q_mvar;
    ld.in_service = true;
    ld.dynamic_model.standard = "PowerSystems";
    ld.dynamic_model.model_name = "ConstantImpedanceLoad";
    ld.dynamic_model.source_id = "PowerSimulationsDynamics:test_case49_csvgn1";
    return ld;
  };
  sys.ac.loads = {
      load(1, 2, "load21", 455.0, 1.0),
      load(2, 3, "load31", 140.0, 95.0),
  };

  StaticGenerator csvgn1;
  csvgn1.index = 1;
  csvgn1.bus = 3;
  csvgn1.name = "CSVGN1";
  csvgn1.p_mw = 0.0;
  csvgn1.q_mvar = 0.0;
  csvgn1.sn_mva = 500.0;
  csvgn1.v_ref_pu = 1.015;
  csvgn1.in_service = true;
  csvgn1.dynamic_model.standard = "PSS/E";
  csvgn1.dynamic_model.model_name = "CSVGN1";
  csvgn1.dynamic_model.source_id = "PowerSimulationsDynamics:test_case49_csvgn1";
  csvgn1.dynamic_model.parameters = {
      {"K", 20.0},
      {"T1", 0.0},
      {"T2", 1.0},
      {"T3", 0.154833},
      {"T4", 1.0},
      {"T5", 0.005167},
      {"Rmin", 0.0},
      {"Vmax", 1.0},
      {"Vmin", 0.0},
      {"CBase", 60.0},
      {"base_power", 500.0},
  };
  sys.ac.static_generators = {csvgn1};

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

HybridPowerSystem make_psd_genrou_avrtype1_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_genrou_avrtype1";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test17_genrou_avrtype1";
  machine.dynamic_model.components.clear();
  hacdcpf::DynamicModelComponentProfile avr;
  avr.type = "exciter";
  avr.model = "AVRTypeI";
  avr.standard = "PowerSystems";
  avr.parameter_set = "PowerSimulationsDynamics:avr_type1";
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
  return sys;
}

HybridPowerSystem make_psd_genrou_sexs_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_genrou_sexs";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test26_sexs";
  machine.dynamic_model.components.clear();
  hacdcpf::DynamicModelComponentProfile avr;
  avr.type = "exciter";
  avr.model = "SEXS";
  avr.standard = "PSS/E";
  avr.parameter_set = "PowerSimulationsDynamics:test26_sexs";
  avr.parameters = {
      {"Ta_Tb", 0.4},
      {"Tb", 5.0},
      {"K", 20.0},
      {"Te", 1.0},
      {"Emin", -50.0},
      {"Emax", 50.0},
  };
  machine.dynamic_model.components.push_back(std::move(avr));
  return sys;
}

HybridPowerSystem make_psd_stab1_omib_case() {
  HybridPowerSystem sys;
  sys.name = "psd_test41_stab1_omib";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1;
  b1.index = 1;
  b1.name = "GENBUS";
  b1.bus_type = BusType::PV;
  b1.base_kv = 24.0;
  b1.vm_pu = 1.0;
  b1.va_deg = 25.0850;
  b1.in_service = true;

  ACBus b2;
  b2.index = 2;
  b2.name = "INFBUS";
  b2.bus_type = BusType::SLACK;
  b2.base_kv = 24.0;
  b2.vm_pu = 0.95512;
  b2.va_deg = 0.0;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.name = "GENBUS-INFBUS-i_1";
  br.r_pu = 0.0;
  br.x_pu = 0.45;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};

  ExternalGrid source;
  source.index = 2;
  source.bus = 2;
  source.name = "INFBUS";
  source.vm_pu = 0.95512;
  source.va_deg = 0.0;
  source.r_pu = 0.0;
  source.x_pu = 0.01;
  source.in_service = true;
  sys.ac.external_grids = {source};

  Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.name = "generator-1-1";
  gen.is_slack = false;
  gen.in_service = true;
  gen.pg_mw = 90.0;
  gen.qg_mvar = 29.993;
  gen.vg_pu = 1.0;
  gen.pmax_mw = 9999.0;
  gen.pmin_mw = -9999.0;
  gen.qmax_mvar = 9999.0;
  gen.qmin_mvar = -9999.0;
  gen.ra_pu = 0.0;
  gen.xd_pu = 2.2;
  gen.xq_pu = 2.2;
  gen.xdp_pu = 0.30;
  gen.xdpp_pu = 0.30;
  gen.td0p_s = 7.0;
  gen.td0pp_s = 999.0;
  gen.inertia_h = 4.0;
  gen.droop_r = 0.05;
  gen.dynamic_model.standard = "PSS/E";
  gen.dynamic_model.model_name = "GENROU";
  gen.dynamic_model.source_id = "PowerSimulationsDynamics:test41_stab1";
  gen.dynamic_model.parameters = {
      {"H", 4.0},
      {"D", 0.0},
      {"R", 0.0},
      {"Xd", 2.2},
      {"Xq", 2.2},
      {"Xd_p", 0.30},
      {"Xq_p", 0.30},
      {"Xd_pp", 0.30},
      {"Xl", 0.20},
      {"Td0_p", 7.0},
      {"Td0_pp", 999.0},
      {"Tq0_p", 0.4},
      {"Tq0_pp", 999.0},
      {"Sat_A", 0.0},
      {"Sat_B", 0.0},
  };

  hacdcpf::DynamicModelComponentProfile avr;
  avr.type = "exciter";
  avr.model = "SEXS";
  avr.standard = "PSS/E";
  avr.parameter_set = "PowerSimulationsDynamics:test41_stab1";
  avr.parameters = {
      {"Ta_Tb", 1.0},
      {"Tb", 1.0},
      {"K", 130.0},
      {"Te", 0.4},
      {"Emin", -10.0},
      {"Emax", 10.0},
  };
  gen.dynamic_model.components.push_back(std::move(avr));

  hacdcpf::DynamicModelComponentProfile pss;
  pss.type = "pss";
  pss.model = "STAB1";
  pss.standard = "PSS/E";
  pss.parameter_set = "PowerSimulationsDynamics:test41_stab1";
  pss.parameters = {
      {"KT", 6.0},
      {"T", 1.5},
      {"T1T3", 13.3},
      {"T3", 0.0447},
      {"T2T4", 13.3},
      {"T4", 0.0447},
      {"H_lim", 0.2},
  };
  gen.dynamic_model.components.push_back(std::move(pss));

  sys.ac.generators = {gen};
  return sys;
}

HybridPowerSystem make_psd_genrou_sexs_ieeest_case() {
  HybridPowerSystem sys = make_psd_genrou_sexs_case();
  sys.name = "psd_genrou_sexs_ieeest";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test30_ieeest";
  for (auto& comp : machine.dynamic_model.components) {
    comp.parameter_set = "PowerSimulationsDynamics:test30_ieeest";
  }

  hacdcpf::DynamicModelComponentProfile pss;
  pss.type = "pss";
  pss.model = "IEEEST";
  pss.standard = "PSS/E";
  pss.parameter_set = "PowerSimulationsDynamics:test30_ieeest_with_filter";
  pss.parameters = {
      {"input_code", 1.0},
      {"A1", 0.0},
      {"A2", 1.013},
      {"A3", 0.013},
      {"A4", 0.02},
      {"A5", 0.0},
      {"A6", 1.013},
      {"T1", 0.113},
      {"T2", 0.02},
      {"T3", 0.0},
      {"T4", 0.02},
      {"T5", 1.65},
      {"T6", 1.65},
      {"Ks", 3.0},
      {"Vsmax", 0.1},
      {"Vsmin", -0.1},
      {"Vcu", 0.0},
      {"Vcl", 0.0},
  };
  machine.dynamic_model.components.push_back(std::move(pss));
  return sys;
}

std::pair<double, double> psd_quadratic_saturation_coeffs(double se1,
                                                          double se12) {
  if (se1 == 0.0 || se12 == 0.0) return {0.0, 0.0};
  const double e1 = 1.0;
  const double e2 = 1.2;
  const double denom = se12 * e2 - se1 * e1;
  if (std::abs(denom) < 1e-12 || se12 <= se1) return {0.0, 0.0};
  const double radicand = e1 * e2 * se1 * se12 * (e1 - e2) * (e1 - e2);
  const double sat_a =
      (e1 * e2 * (se12 - se1) - std::sqrt(std::max(0.0, radicand))) / denom;
  const double sat_b = se12 * e2 / ((e2 - sat_a) * (e2 - sat_a));
  return {sat_a, sat_b};
}

void set_machine_saturation_from_psse_points(Generator& machine,
                                             double se1,
                                             double se12) {
  const auto [sat_a, sat_b] = psd_quadratic_saturation_coeffs(se1, se12);
  machine.dynamic_model.parameters["Sat_A"] = sat_a;
  machine.dynamic_model.parameters["Sat_B"] = sat_b;
}

hacdcpf::DynamicModelComponentProfile make_controller_component(
    std::string type,
    std::string model,
    std::string parameter_set,
    std::map<std::string, double> params,
    std::string standard = "PSS/E") {
  hacdcpf::DynamicModelComponentProfile c;
  c.type = std::move(type);
  c.model = std::move(model);
  c.standard = std::move(standard);
  c.parameter_set = std::move(parameter_set);
  c.parameters = std::move(params);
  return c;
}

hacdcpf::DynamicModelComponentProfile psd_esac1a_component(
    const std::string& parameter_set) {
  return make_controller_component(
      "exciter",
      "ESAC1A",
      parameter_set,
      {{"Tr", 0.01},
       {"Tc", 0.10},
       {"Tb", 0.0},
       {"Ka", 200.0},
       {"Ta", 0.05},
       {"Va_max", 7.0},
       {"Va_min", -7.0},
       {"Te", 1.333},
       {"Kf", 0.02},
       {"Tf", 0.8},
       {"Kc", 0.0},
       {"Kd", 0.0},
       {"Ke", 1.0},
       {"Emax", 10.0},
       {"Emin", -10.0}});
}

hacdcpf::DynamicModelComponentProfile psd_sexs_component(
    const std::string& parameter_set,
    double gain = 20.0) {
  return make_controller_component("exciter",
                                   "SEXS",
                                   parameter_set,
                                   {{"Ta_Tb", 0.4},
                                    {"Tb", 5.0},
                                    {"K", gain},
                                    {"Te", 1.0},
                                    {"Emin", -50.0},
                                    {"Emax", 50.0}});
}

HybridPowerSystem make_psd_genrou_esac1a_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_genrou_esac1a";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test20_esac1a";
  set_machine_saturation_from_psse_points(machine, 0.1, 0.8);
  machine.dynamic_model.components.clear();
  machine.dynamic_model.components.push_back(
      psd_esac1a_component("PowerSimulationsDynamics:test20_esac1a"));
  return sys;
}

HybridPowerSystem make_psd_genrou_tgov1_esac1a_case() {
  HybridPowerSystem sys = make_psd_genrou_esac1a_case();
  sys.name = "psd_genrou_tgov1_esac1a";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test22_tgov1";
  set_machine_saturation_from_psse_points(machine, 0.0, 1.0);
  machine.dynamic_model.components.insert(
      machine.dynamic_model.components.begin(),
      make_controller_component("governor",
                                "TGOV1",
                                "PowerSimulationsDynamics:test22_tgov1",
                                {{"R", 0.05},
                                 {"T1", 0.2},
                                 {"Vmax", 1.2},
                                 {"Vmin", 0.1},
                                 {"T2", 0.3},
                                 {"T3", 0.8},
                                 {"D_T", 0.0}}));
  return sys;
}

HybridPowerSystem make_psd_genrou_gast_case() {
  HybridPowerSystem sys = make_psd_genrou_esac1a_case();
  sys.name = "psd_genrou_gast";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test21_gast";
  set_machine_saturation_from_psse_points(machine, 0.0, 1.0);
  machine.dynamic_model.components.insert(
      machine.dynamic_model.components.begin(),
      make_controller_component("governor",
                                "GAST",
                                "PowerSimulationsDynamics:test21_gast",
                                {{"R", 0.05},
                                 {"T1", 0.2},
                                 {"T2", 0.2},
                                 {"T3", 2.0},
                                 {"Vmax", 1.1},
                                 {"Vmin", 0.01},
                                 {"D_T", 0.0}}));
  return sys;
}

HybridPowerSystem make_psd_genrou_hygov_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_genrou_hygov";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test31_hygov";
  set_machine_saturation_from_psse_points(machine, 0.0, 1.0);
  machine.dynamic_model.components.clear();
  machine.dynamic_model.components.push_back(
      psd_sexs_component("PowerSimulationsDynamics:test31_hygov"));
  machine.dynamic_model.components.push_back(
      make_controller_component("governor",
                                "HYGOV",
                                "PowerSimulationsDynamics:test31_hygov",
                                {{"R", 0.05},
                                 {"Tr", 12.0},
                                 {"Tf", 0.5},
                                 {"Tg", 20.5},
                                 {"Tw", 15.0},
                                 {"Gmax", 1.0},
                                 {"Gmin", 0.0},
                                 {"D_T", 0.0}}));
  return sys;
}

HybridPowerSystem make_psd_genrou_scrx_case() {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_genrou_scrx";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test47_scrx";
  set_machine_saturation_from_psse_points(machine, 0.1, 0.8);
  machine.dynamic_model.components.clear();
  machine.dynamic_model.components.push_back(
      make_controller_component("exciter",
                                "SCRX",
                                "PowerSimulationsDynamics:test47_scrx",
                                {{"Ta", 0.4},
                                 {"Tb", 5.0},
                                 {"K", 20.0},
                                 {"Te", 1.0},
                                 {"Emin", -50.0},
                                 {"Emax", 50.0}}));
  return sys;
}

HybridPowerSystem make_psd_genrou_pidgov_case(const std::string& model) {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_genrou_" + model;
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:" + model;
  set_machine_saturation_from_psse_points(machine, 0.0, 1.0);
  machine.dynamic_model.components.clear();
  machine.dynamic_model.components.push_back(
      psd_sexs_component("PowerSimulationsDynamics:" + model));
  machine.dynamic_model.components.push_back(
      make_controller_component("governor",
                                model,
                                "PowerSimulationsDynamics:" + model,
                                {{"R", 0.05},
                                 {"T_reg", 0.01},
                                 {"Ki", 5.0},
                                 {"Kd", 1.0},
                                 {"Ta", 0.1},
                                 {"Tb", 0.5},
                                 {"Tg", 0.02},
                                 {"Tw", 3.25},
                                 {"Gmax", 1.0},
                                 {"Gmin", 0.0},
                                 {"Vmax", 1.1},
                                 {"Vmin", 0.0}}));
  return sys;
}

HybridPowerSystem make_psd_gencls_degov_case(const std::string& model) {
  HybridPowerSystem sys = make_psd_genrou_three_bus_subset_case();
  sys.name = "psd_gencls_" + model;
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.standard = "PSS/E";
  machine.dynamic_model.model_name = "ClassicalMachine";
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:" + model;
  machine.inertia_h = 8.0;
  machine.dynamic_model.parameters = {
      {"H", 8.0},
      {"D", 0.03},
      {"R", 0.0},
      {"Xd_p", 0.25},
      {"eq_p", 1.0},
  };
  machine.dynamic_model.components.clear();
  machine.dynamic_model.components.push_back(
      make_controller_component("governor",
                                model,
                                "PowerSimulationsDynamics:" + model,
                                {{"R", model == "DEGOV1" ? 0.070 : 0.0},
                                 {"T1", model == "DEGOV1" ? 0.1905 : 0.0},
                                 {"Ta", model == "DEGOV1" ? 0.0476 : 0.0},
                                 {"Tb", model == "DEGOV1" ? 0.018 : 0.0},
                                 {"Tf", model == "DEGOV1" ? 5.1 : 12.0},
                                 {"Tg", model == "DEGOV1" ? 0.322 : 5.0},
                                 {"T3", model == "DEGOV1" ? 1.0 : 0.2},
                                 {"Vmax", 2.0},
                                 {"Vmin", -0.1}}));
  return sys;
}

HybridPowerSystem make_psd_tgsimple_case() {
  HybridPowerSystem sys = make_psd_onedoneq_three_bus_subset_case();
  sys.name = "psd_tgsimple";
  REQUIRE(sys.ac.generators.size() >= 2);
  auto& machine = sys.ac.generators[1];
  machine.dynamic_model.source_id = "PowerSimulationsDynamics:test62_tgsimple";
  machine.dynamic_model.components.clear();
  machine.dynamic_model.components.push_back(
      make_controller_component("governor",
                                "TGSimple",
                                "PowerSimulationsDynamics:test62_tgsimple",
                                {{"T1", 0.5}, {"Vmax", 1.1}, {"Vmin", 0.0}},
                                "PowerSystems"));
  return sys;
}

HybridPowerSystem make_psd_pss2_omib_case(const std::string& pss_model) {
  HybridPowerSystem sys;
  sys.name = "psd_" + pss_model + "_omib";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 60.0;

  ACBus b1;
  b1.index = 1;
  b1.name = "GENBUS";
  b1.bus_type = BusType::PV;
  b1.base_kv = 24.0;
  b1.vm_pu = 1.0;
  b1.va_deg = 25.0850;
  b1.in_service = true;

  ACBus b2;
  b2.index = 2;
  b2.name = "INFBUS";
  b2.bus_type = BusType::SLACK;
  b2.base_kv = 24.0;
  b2.vm_pu = 0.95512;
  b2.va_deg = 0.0;
  b2.in_service = true;
  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.name = "GENBUS-INFBUS-i_1";
  br.r_pu = 0.0;
  br.x_pu = 0.45;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};

  ExternalGrid source;
  source.index = 2;
  source.bus = 2;
  source.name = "INFBUS";
  source.vm_pu = 0.95512;
  source.r_pu = 0.0;
  source.x_pu = 0.01;
  source.in_service = true;
  sys.ac.external_grids = {source};

  Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.name = "generator-1-1";
  gen.is_slack = false;
  gen.in_service = true;
  gen.pg_mw = 90.0;
  gen.qg_mvar = 29.993;
  gen.vg_pu = 1.0;
  gen.pmax_mw = 9999.0;
  gen.pmin_mw = -9999.0;
  gen.qmax_mvar = 9999.0;
  gen.qmin_mvar = -9999.0;
  gen.ra_pu = 0.0;
  gen.xd_pu = 2.2;
  gen.xq_pu = 2.2;
  gen.xdp_pu = 0.30;
  gen.xdpp_pu = 0.30;
  gen.td0p_s = 7.0;
  gen.td0pp_s = 999.0;
  gen.inertia_h = 4.0;
  gen.dynamic_model.standard = "PSS/E";
  gen.dynamic_model.model_name = "GENROU";
  gen.dynamic_model.source_id = "PowerSimulationsDynamics:test_" + pss_model;
  gen.dynamic_model.parameters = {
      {"H", 4.0},
      {"D", 0.0},
      {"R", 0.0},
      {"Xd", 2.2},
      {"Xq", 2.2},
      {"Xd_p", 0.30},
      {"Xq_p", 0.30},
      {"Xd_pp", 0.30},
      {"Xl", 0.20},
      {"Td0_p", 7.0},
      {"Td0_pp", 999.0},
      {"Tq0_p", 0.4},
      {"Tq0_pp", 999.0},
      {"Sat_A", 0.0},
      {"Sat_B", 0.0},
  };
  gen.dynamic_model.components.push_back(make_controller_component(
      "exciter",
      "SEXS",
      "PowerSimulationsDynamics:test_" + pss_model,
      {{"Ta_Tb", 1.0}, {"Tb", 1.0}, {"K", 120.0}, {"Te", 0.4},
       {"Emin", -10.0}, {"Emax", 10.0}}));
  std::map<std::string, double> pss_params = {
      {"input_code_1", 1.0},
      {"input_code_2", 3.0},
      {"M_rtf", 5.0},
      {"N_rtf", 1.0},
      {"Tw1", 1.5},
      {"Tw2", 1.5},
      {"T6", 0.001},
      {"Tw3", 1.5},
      {"Tw4", 0.0},
      {"T7", 1.5},
      {"Ks2", 0.1875},
      {"Ks3", 1.0},
      {"T8", 0.5},
      {"T9", 0.1},
      {"Ks1", 2.0},
      {"T1", 0.59451},
      {"T2", 0.0447},
      {"T3", 0.59451},
      {"T4", 0.0447},
      {"T10", 1.0},
      {"T11", 1.0},
      {"Vstmax", 0.1},
      {"Vstmin", -0.1},
  };
  if (pss_model == "PSS2C") {
    pss_params["T12"] = 1.0;
    pss_params["T13"] = 1.0;
  }
  gen.dynamic_model.components.push_back(make_controller_component(
      "pss",
      pss_model,
      "PowerSimulationsDynamics:test_" + pss_model,
      std::move(pss_params)));
  sys.ac.generators = {gen};
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

DynamicJacobianContext test_jacobian_context(int n_x, int n_ac, int n_dc) {
  DynamicJacobianContext context;
  context.n_x = n_x;
  context.n_ac = n_ac;
  context.n_dc = n_dc;
  context.ac_real_offset = n_x;
  context.ac_imag_offset = n_x + n_ac;
  context.dc_offset = n_x + 2 * n_ac;
  context.total_size = n_x + 2 * n_ac + n_dc;
  context.dt = 0.01;
  context.theta = 1.0;
  return context;
}

Eigen::VectorXd stamped_current_vector(const DynamicDevice& device,
                                       const DynamicState& x,
                                       const NetworkState& y,
                                       const DynamicJacobianContext& context) {
  DynamicStamp stamp(context.n_ac, context.n_dc);
  device.stamp(0.0, x, y, stamp);
  Eigen::VectorXd out = Eigen::VectorXd::Zero(context.total_size);
  for (int node = 0; node < context.n_ac; ++node) {
    out[context.acRealRow(node)] = stamp.Iac[node].real();
    out[context.acImagRow(node)] = stamp.Iac[node].imag();
  }
  for (int node = 0; node < context.n_dc; ++node) {
    out[context.dcRow(node)] = stamp.Idc[node];
  }
  return out;
}

Eigen::VectorXd finite_difference_current_column(const DynamicDevice& device,
                                                 const DynamicState& x,
                                                 const NetworkState& y,
                                                 const DynamicJacobianContext& context,
                                                 int col) {
  DynamicState xp = x;
  DynamicState xm = x;
  NetworkState yp = y;
  NetworkState ym = y;
  double base = 0.0;
  if (col < context.n_x) {
    base = x.x[col];
  } else if (col < context.ac_imag_offset) {
    base = y.Vac_abc[col - context.ac_real_offset].real();
  } else if (col < context.dc_offset) {
    base = y.Vac_abc[col - context.ac_imag_offset].imag();
  } else {
    base = y.Vdc[col - context.dc_offset];
  }
  const double h = 1e-6 * std::max(1.0, std::abs(base));
  if (col < context.n_x) {
    xp.x[col] += h;
    xm.x[col] -= h;
  } else if (col < context.ac_imag_offset) {
    const int node = col - context.ac_real_offset;
    yp.Vac_abc[node] += Complex(h, 0.0);
    ym.Vac_abc[node] -= Complex(h, 0.0);
  } else if (col < context.dc_offset) {
    const int node = col - context.ac_imag_offset;
    yp.Vac_abc[node] += Complex(0.0, h);
    ym.Vac_abc[node] -= Complex(0.0, h);
  } else {
    const int node = col - context.dc_offset;
    yp.Vdc[node] += h;
    ym.Vdc[node] -= h;
  }
  return (stamped_current_vector(device, xp, yp, context) -
          stamped_current_vector(device, xm, ym, context)) /
         (2.0 * h);
}

Eigen::VectorXd analytic_current_column(const DynamicDevice& device,
                                        const DynamicState& x,
                                        const NetworkState& y,
                                        const DynamicJacobianContext& context,
                                        int col) {
  std::vector<Eigen::Triplet<double>> triplets;
  device.addJacobian(0.0, x, y, context, triplets);
  Eigen::SparseMatrix<double> jac(context.total_size, context.total_size);
  jac.setFromTriplets(triplets.begin(), triplets.end());
  Eigen::VectorXd out = Eigen::VectorXd::Zero(context.total_size);
  for (Eigen::SparseMatrix<double>::InnerIterator it(jac, col); it; ++it) {
    out[it.row()] = it.value();
  }
  return out;
}

void check_device_current_jacobian_column(const DynamicDevice& device,
                                          const DynamicState& x,
                                          const NetworkState& y,
                                          const DynamicJacobianContext& context,
                                          int col,
                                          double tolerance = 1e-7) {
  const Eigen::VectorXd analytic =
      analytic_current_column(device, x, y, context, col);
  const Eigen::VectorXd finite_difference =
      finite_difference_current_column(device, x, y, context, col);
  CHECK((analytic - finite_difference).lpNorm<Eigen::Infinity>() < tolerance);
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

HybridPowerSystem make_psd_gfm_omib_case(const std::string& model) {
  const bool vsm = model == "VSMGridForming";
  const bool voc = model == "VOCGridForming";
  HybridPowerSystem sys;
  sys.name = voc ? "psd_test44_voc_inverter"
                 : (vsm ? "psd_test08_vsm_inverter"
                        : "psd_test23_droop_inverter");
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b101;
  b101.index = 101;
  b101.name = "BUS 1";
  b101.bus_type = BusType::SLACK;
  b101.base_kv = 230.0;
  b101.vm_pu = 1.00001;
  b101.va_deg = 0.0;
  b101.in_service = true;

  ACBus b102;
  b102.index = 102;
  b102.name = "BUS 2";
  b102.bus_type = BusType::PQ;
  b102.base_kv = 230.0;
  b102.vm_pu = 1.0;
  b102.va_deg = 0.0;
  b102.in_service = true;
  sys.ac.buses = {b101, b102};

  ACBranch br;
  br.index = 1;
  br.from_bus = 101;
  br.to_bus = 102;
  br.name = "BUS 1-BUS 2-i_1";
  br.r_pu = 0.0;
  br.x_pu = 0.075;
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};

  ExternalGrid source;
  source.index = 1;
  source.bus = 101;
  source.name = "InfBus";
  source.vm_pu = 1.00001;
  source.va_deg = 0.0;
  source.r_pu = 0.0;
  source.x_pu = 1.0e-5;
  source.in_service = true;
  sys.ac.external_grids = {source};

  StaticGenerator inv;
  inv.index = 1;
  inv.bus = 102;
  inv.name = "generator-102-1";
  inv.p_mw = 50.0;
  inv.q_mvar = 0.0;
  inv.sn_mva = 100.0;
  inv.p_rated_mw = 100.0;
  inv.pmax_mw = 200.0;
  inv.pmin_mw = -200.0;
  inv.v_ref_pu = 1.0;
  inv.in_service = true;
  inv.dynamic_model.standard = "PowerSimulationsDynamics";
  inv.dynamic_model.model_name = model;
  inv.dynamic_model.source_id = voc ? "PowerSimulationsDynamics:test_case44_voc"
                                    : (vsm ? "PowerSimulationsDynamics:test_case08_vsm"
                                           : "PowerSimulationsDynamics:test_case23_droop");
  inv.dynamic_model.parameters = {
      {"virtual_x_pu", 0.20},
      {"virtual_r_pu", 0.0},
      {"q_droop_pu", 0.2},
      {"reactive_power_filter_t_s", 0.001},
      {"voltage_control_t_s", 0.02},
      {"v_ref_pu", 1.0},
  };
  if (voc) {
    inv.dynamic_model.parameters["k1"] = 0.0033;
    inv.dynamic_model.parameters["psi"] = 3.14159265358979323846 / 4.0;
    inv.dynamic_model.parameters["k2"] = 0.0796;
    inv.dynamic_model.components.push_back(
        {"outer_control", "VOCOuterControl", "PowerSimulationsDynamics",
         "PowerSimulationsDynamics:test_case44_voc",
         {{"k1", 0.0033},
          {"psi", 3.14159265358979323846 / 4.0},
          {"k2", 0.0796}}});
    inv.dynamic_model.components.push_back(
        {"filter", "LCLFilter", "PowerSimulationsDynamics",
         "PowerSimulationsDynamics:test_case44_voc",
         {{"virtual_x_pu", 0.20}, {"virtual_r_pu", 0.0}, {"Cf", 0.1086}}});
  } else if (vsm) {
    inv.dynamic_model.parameters["Ta"] = 2.0;
    inv.dynamic_model.parameters["kd"] = 400.0;
    inv.dynamic_model.parameters["komega"] = 20.0;
    inv.dynamic_model.components.push_back(
        {"outer_control", "VSMOuterControl", "PowerSimulationsDynamics",
         "PowerSimulationsDynamics:test_case08_vsm",
         {{"Ta", 2.0},
          {"kd", 400.0},
          {"komega", 20.0},
          {"q_droop_pu", 0.2},
          {"reactive_power_filter_t_s", 0.001}}});
    inv.dynamic_model.components.push_back(
        {"filter", "LCLFilter", "PowerSimulationsDynamics",
         "PowerSimulationsDynamics:test_case08_vsm",
         {{"virtual_x_pu", 0.20}, {"virtual_r_pu", 0.0}, {"Cf", 0.074}}});
  } else {
    inv.dynamic_model.parameters["p_droop_pu"] = 0.05;
    inv.dynamic_model.parameters["power_filter_t_s"] =
        1.0 / (2.0 * 3.14159265358979323846 * 5.0);
    inv.dynamic_model.components.push_back(
        {"outer_control", "GFMDroopOuterControl", "PowerSimulationsDynamics",
         "PowerSimulationsDynamics:test_case23_droop",
         {{"p_droop_pu", 0.05},
          {"q_droop_pu", 0.2},
          {"power_filter_t_s", 1.0 / (2.0 * 3.14159265358979323846 * 5.0)},
          {"reactive_power_filter_t_s", 0.001}}});
    inv.dynamic_model.components.push_back(
        {"filter", "LCLFilter", "PowerSimulationsDynamics",
         "PowerSimulationsDynamics:test_case23_droop",
         {{"virtual_x_pu", 0.20}, {"virtual_r_pu", 0.0}, {"Cf", 0.074}}});
  }
  inv.dynamic_model.components.push_back(
      {"converter", "AverageConverter", "PowerSimulationsDynamics",
       inv.dynamic_model.source_id, {}});
  inv.dynamic_model.components.push_back(
      {"inner_control", "VirtualImpedanceInnerControl", "HACDCPF",
       inv.dynamic_model.source_id, {{"virtual_x_pu", 0.20}, {"virtual_r_pu", 0.0}}});
  inv.dynamic_model.components.push_back(
      {"dc_source", "ConstantDCSource", "PowerSimulationsDynamics",
       inv.dynamic_model.source_id, {}});
  inv.dynamic_model.components.push_back(
      {"pll", "FixedFrequency", "PowerSimulationsDynamics",
       inv.dynamic_model.source_id, {}});
  sys.ac.static_generators = {inv};
  return sys;
}

HybridPowerSystem make_psd_gfl_case(const PsdGflCase& spec,
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
  return sys;
}

DynamicResults run_hacdcpf_psd_gfl_case(const PsdGflCase& spec,
                                        double base_mva) {
  auto sys = make_psd_gfl_case(spec, base_mva);

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

HybridPowerSystem make_psd_periodic_variable_source_case() {
  auto sys = make_transient_2bus();
  sys.ac.generators.clear();
  sys.ac.loads.clear();
  sys.ac.buses[0].vm_pu = 1.05;
  sys.ac.buses[1].vm_pu = 1.04;
  ExternalGrid grid;
  grid.index = 1;
  grid.bus = 1;
  grid.name = "InfBus";
  grid.vm_pu = 1.05;
  grid.va_deg = 0.0;
  grid.r_pu = 0.0;
  grid.x_pu = 0.05;
  grid.in_service = true;
  grid.dynamic_model.standard = "PowerSimulationsDynamics";
  grid.dynamic_model.model_name = "PeriodicVariableSource";
  grid.dynamic_model.parameters = {
      {"R_th", 0.0},
      {"X_th", 0.05},
      {"internal_voltage_bias", 1.0},
      {"internal_voltage_frequency_rad_s", 2.0 * 3.14159265358979323846},
      {"internal_voltage_sin_coeff", 1.0},
      {"internal_voltage_cos_coeff", 0.0},
      {"internal_angle_bias", 0.0},
      {"internal_angle_frequency_rad_s", 2.0 * 3.14159265358979323846},
      {"internal_angle_sin_coeff", 0.0},
      {"internal_angle_cos_coeff", 1.0}};
  sys.ac.external_grids = {grid};
  return sys;
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
  double local_scale{1.0};
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
  if (event_type == "ACBranchTrip") {
    out.event.type = DynamicEventType::ACBranchTrip;
  } else if (event_type == "ACBranchImpedanceScale") {
    out.event.type = DynamicEventType::ACBranchImpedanceScale;
  } else if (event_type == "ACLoadScale") {
    out.event.type = DynamicEventType::ACLoadScale;
  } else {
    REQUIRE(event_type == "Custom");
    out.event.type = DynamicEventType::Custom;
  }
  out.event.time_s = event.at("time_s").get<double>();
  out.event.component_type =
      event.value("component_type",
                  (event_type == "ACBranchTrip" ||
                   event_type == "ACBranchImpedanceScale")
                      ? "AC"
                      : "");
  out.event.component_index = event.at("component_index").get<int>();
  out.event.bus = event.value("bus", 0);
  out.event.value = event.value("value", 0.0);
  out.event.label = event.value("label", out.id + " event");
  if (event.contains("params")) {
    REQUIRE(event.at("params").is_object());
    for (auto it = event.at("params").begin(); it != event.at("params").end(); ++it) {
      out.event.params[it.key()] = it.value().get<double>();
    }
  }

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
      signal.local_scale = raw_signal.value("local_scale", 1.0);
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
  if (spec.hacdcpf_fixture == "make_psd_test01_omib_case") {
    return make_psd_test01_omib_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_onedoneq_three_bus_subset_case") {
    return make_psd_onedoneq_three_bus_subset_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_simple_marconato_three_bus_case") {
    return make_psd_simple_marconato_three_bus_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_marconato_three_bus_case") {
    return make_psd_marconato_three_bus_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_simple_af_three_bus_case") {
    return make_psd_simple_af_three_bus_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_anderson_fouad_three_bus_case") {
    return make_psd_anderson_fouad_three_bus_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_test07_five_mass_shaft_case") {
    return make_psd_test07_five_mass_shaft_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_test12_multimachine_tgtype2_case") {
    return make_psd_test12_multimachine_tgtype2_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_test13_onedoneq_avr_tg_case") {
    return make_psd_test13_onedoneq_avr_tg_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_test25_dynamic_line_case") {
    return make_psd_test25_dynamic_line_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_csvgn1_three_bus_case") {
    return make_psd_csvgn1_three_bus_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_three_bus_subset_case") {
    return make_psd_genrou_three_bus_subset_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_avrtype1_case") {
    return make_psd_genrou_avrtype1_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_esac1a_case") {
    return make_psd_genrou_esac1a_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_tgov1_esac1a_case") {
    return make_psd_genrou_tgov1_esac1a_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_gast_case") {
    return make_psd_genrou_gast_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_hygov_case") {
    return make_psd_genrou_hygov_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_scrx_case") {
    return make_psd_genrou_scrx_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_gencls_degov_case") {
    return make_psd_gencls_degov_case("DEGOV");
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_pidgov_case") {
    return make_psd_genrou_pidgov_case("PIDGOV");
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_wpidhy_case") {
    return make_psd_genrou_pidgov_case("WPIDHY");
  }
  if (spec.hacdcpf_fixture == "make_psd_tgsimple_case") {
    return make_psd_tgsimple_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_gencls_degov1_case") {
    return make_psd_gencls_degov_case("DEGOV1");
  }
  if (spec.hacdcpf_fixture == "make_psd_pss2a_omib_case") {
    return make_psd_pss2_omib_case("PSS2A");
  }
  if (spec.hacdcpf_fixture == "make_psd_pss2b_omib_case") {
    return make_psd_pss2_omib_case("PSS2B");
  }
  if (spec.hacdcpf_fixture == "make_psd_pss2c_omib_case") {
    return make_psd_pss2_omib_case("PSS2C");
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_sexs_case") {
    return make_psd_genrou_sexs_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_stab1_omib_case") {
    return make_psd_stab1_omib_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_sexs_ieeest_case") {
    return make_psd_genrou_sexs_ieeest_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genroe_three_bus_subset_case") {
    return make_psd_genroe_three_bus_subset_case();
  }
  if (spec.hacdcpf_fixture == "make_psd_genroe_high_sat_three_bus_subset_case") {
    return make_psd_genroe_three_bus_subset_case(true);
  }
  if (spec.hacdcpf_fixture == "make_psd_gensal_three_bus_subset_case") {
    return make_psd_gensal_three_bus_subset_case(false);
  }
  if (spec.hacdcpf_fixture == "make_psd_gensae_three_bus_subset_case") {
    return make_psd_gensal_three_bus_subset_case(true);
  }
  if (spec.hacdcpf_fixture == "make_psd_gfl_reduced_pll_case") {
    return make_psd_gfl_case({"reduced_pll_test24",
                              "test24",
                              "test_case_gridfollowing.jl",
                              "ReducedOrderPLL",
                              2.0,
                              20.0,
                              1.0 / (1.32 * 2.0 * 3.14159265358979323846 * 50.0)},
                             100.0);
  }
  if (spec.hacdcpf_fixture == "make_psd_gfl_kaura_pll_case") {
    return make_psd_gfl_case({"kaura_pll_test51",
                              "test51",
                              "test_case51_gridfollowing_kaura.jl",
                              "KauraPLL",
                              0.084,
                              4.69,
                              1.0 / 500.0},
                             100.0);
  }
  if (spec.hacdcpf_fixture == "make_psd_vsm_inverter_case") {
    return make_psd_gfm_omib_case("VSMGridForming");
  }
  if (spec.hacdcpf_fixture == "make_psd_droop_inverter_case") {
    return make_psd_gfm_omib_case("GridFormingNortonDroop");
  }
  if (spec.hacdcpf_fixture == "make_psd_voc_inverter_case") {
    return make_psd_gfm_omib_case("VOCGridForming");
  }
  if (spec.hacdcpf_fixture == "make_psd_periodic_variable_source_case") {
    return make_psd_periodic_variable_source_case();
  }
  FAIL("Unsupported PSD executable manifest fixture: " << spec.hacdcpf_fixture);
  return HybridPowerSystem{};
}

DynamicResults run_manifest_hacdcpf_case(const ManifestExecutableCase& spec) {
  auto sys = make_manifest_system(spec);
  DynamicSolverOptions opt = fast_options();
  opt.solver_type = manifest_solver_type(spec.hacdcpf_solver);
  opt.run_power_flow_initialization = true;
  const bool psd_raw_voltage_fixture =
      spec.hacdcpf_fixture.rfind("make_psd_genrou_", 0) == 0 ||
      spec.hacdcpf_fixture == "make_psd_onedoneq_three_bus_subset_case" ||
      spec.hacdcpf_fixture == "make_psd_test01_omib_case" ||
      spec.hacdcpf_fixture == "make_psd_test12_multimachine_tgtype2_case" ||
      spec.hacdcpf_fixture == "make_psd_test13_onedoneq_avr_tg_case";
  if (psd_raw_voltage_fixture) {
    opt.run_power_flow_initialization = false;
  }
  opt.t_start_s = spec.t_start_s;
  opt.t_end_s = spec.t_end_s;
  opt.dt_s = spec.dt_s;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;
  if (spec.hacdcpf_fixture == "make_psd_csvgn1_three_bus_case") {
    opt.algebraic_network_max_iters = 20;
    opt.algebraic_network_tol = 5e-5;
  }
  if (spec.hacdcpf_fixture == "make_psd_periodic_variable_source_case") {
    opt.enforce_voltage_health_check = false;
  }
  if (spec.hacdcpf_fixture == "make_psd_genrou_esac1a_case" ||
      spec.hacdcpf_fixture == "make_psd_genrou_gast_case" ||
      spec.hacdcpf_fixture == "make_psd_genrou_tgov1_esac1a_case") {
    opt.use_adaptive_step = true;
    opt.max_newton_iters = 30;
    opt.max_step_halving = 18;
    opt.min_accepted_step_s = 1e-9;
    opt.newton_tol = 1e-6;
  }

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
    const bool is_ac_bus_trace =
        device.local_component_type == "ACBus" ||
        device.local_component_type == "BusVoltage";
    if (!is_ac_bus_trace) {
      require_device_output(*result.final_snapshot(),
                            device.local_component_type,
                            device.local_component_index);
    }
    for (const auto& signal : device.signals) {
      CsvSeries local;
      if (is_ac_bus_trace) {
        REQUIRE(signal.local_key == "voltage_mag");
        local = bus_voltage_mag_series(result, device.local_component_index);
        for (double& value : local.y) value *= signal.local_scale;
      } else {
        local = device_output_series(result,
                                     device.local_component_type,
                                     device.local_component_index,
                                     signal.local_key,
                                     signal.local_scale);
      }
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
  opt.dynamic_trim_tol = 1e-6;
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
  // The AVR TypeI voltage transducer (Tr = 1 ms) makes this system stiff: its
  // eigenvalue sits at -1/Tr = -1000, far outside the explicit RK2 (Heun)
  // stability region for dt = 5 ms (which needs Re(lambda) > -2/dt = -400).
  // Integrate with an A-stable implicit method so the fast transducer mode is
  // resolved without a numerical blow-up.
  opt.solver_type = DynamicSolverType::TrapezoidalNewton;
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

TEST_CASE("PSD Test 04 Marconato machine profile initializes and responds",
          "[dynamics][benchmark][psd][machine][marconato]") {
  auto sys = make_psd_marconato_three_bus_case();
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
  trip.label = "PSD Test 04 Marconato BUS 1-BUS 3 branch trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= 1e-5);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  for (const int component_index : {2, 3}) {
    INFO("machine component " << component_index);
    const auto& gen_out =
        require_device_output(*result.final_snapshot(), "SynchronousMachine", component_index);
    CHECK(gen_out.model_name == "MarconatoMachine");
    CHECK(gen_out.values.count("psd_marconato") == 1);
    CHECK(gen_out.values.count("psi_q") == 1);
    CHECK(gen_out.values.count("psi_d") == 1);
    CHECK(gen_out.values.count("eq_p") == 1);
    CHECK(gen_out.values.count("ed_p") == 1);
    CHECK(gen_out.values.count("eq_pp") == 1);
    CHECK(gen_out.values.count("ed_pp") == 1);
    REQUIRE(gen_out.model_profiles.size() >= 1);
    CHECK(std::any_of(gen_out.model_profiles.begin(),
                      gen_out.model_profiles.end(),
                      [](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.model_name == "MarconatoMachine";
                      }));

    const auto delta =
        device_output_series(result, "SynchronousMachine", component_index, "angle_rad");
    const auto omega =
        device_output_series(result, "SynchronousMachine", component_index, "omega_pu");
    const auto psiq =
        device_output_series(result, "SynchronousMachine", component_index, "psi_q");
    const auto psid =
        device_output_series(result, "SynchronousMachine", component_index, "psi_d");
    CHECK(series_range(delta) > 1e-4);
    CHECK(series_range(omega) > 1e-7);
    CHECK(series_range(psiq) > 1e-5);
    CHECK(series_range(psid) > 1e-5);
  }
}

void run_anderson_family_smoke(HybridPowerSystem sys,
                               const std::string& expected_model_name,
                               const std::string& expected_flag,
                               const std::string& trip_label,
                               bool full_anderson) {
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
  trip.label = trip_label;
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= 1e-5);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  for (const int component_index : {2, 3}) {
    INFO("machine component " << component_index);
    const auto& gen_out =
        require_device_output(*result.final_snapshot(), "SynchronousMachine", component_index);
    CHECK(gen_out.model_name == expected_model_name);
    CHECK(gen_out.values.count(expected_flag) == 1);
    CHECK(gen_out.values.count("eq_p") == 1);
    CHECK(gen_out.values.count("ed_p") == 1);
    CHECK(gen_out.values.count("eq_pp") == 1);
    CHECK(gen_out.values.count("ed_pp") == 1);
    CHECK(gen_out.values.count("psi_q") == (full_anderson ? 1 : 0));
    CHECK(gen_out.values.count("psi_d") == (full_anderson ? 1 : 0));
    REQUIRE(gen_out.model_profiles.size() >= 1);
    CHECK(std::any_of(gen_out.model_profiles.begin(),
                      gen_out.model_profiles.end(),
                      [&](const hacdcpf::dynamics::DynamicModelProfile& profile) {
                        return profile.model_name == expected_model_name;
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
    REQUIRE_FALSE(edp.y.empty());
    CHECK(std::all_of(edp.y.begin(), edp.y.end(), [](double v) {
      return std::isfinite(v);
    }));
  }
}

TEST_CASE("PSD Test 05 SimpleAF machine profile initializes and responds",
          "[dynamics][benchmark][psd][machine][simple-af]") {
  run_anderson_family_smoke(make_psd_simple_af_three_bus_case(),
                            "SimpleAFMachine",
                            "psd_simple_af",
                            "PSD Test 05 SimpleAF BUS 1-BUS 3 branch trip",
                            false);
}

TEST_CASE("PSD Test 06 Anderson-Fouad machine profile initializes and responds",
          "[dynamics][benchmark][psd][machine][anderson-fouad]") {
  run_anderson_family_smoke(make_psd_anderson_fouad_three_bus_case(),
                            "AndersonFouadMachine",
                            "psd_anderson_fouad",
                            "PSD Test 06 Anderson-Fouad BUS 1-BUS 3 branch trip",
                            true);
}

TEST_CASE("PSD Test 12 multimachine TGTypeII fixture initializes with impedance switch",
          "[dynamics][benchmark][psd][machine][tgtype2][test12]") {
  auto sys = make_psd_test12_multimachine_tgtype2_case();
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
  DynamicEvent impedance_step;
  impedance_step.time_s = 1.0;
  impedance_step.type = DynamicEventType::ACBranchImpedanceScale;
  impedance_step.component_type = "AC";
  impedance_step.component_index = 2;
  impedance_step.label = "PSD Test 12 BUS 1-BUS 2 impedance x4";
  impedance_step.params["scale"] = 4.0;
  dyn.events.push_back(impedance_step);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  const auto& gen102 =
      require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
  CHECK(gen102.model_name == "ClassicalMachine");
  const auto& gov102 = require_device_output(*result.final_snapshot(), "Governor", 2);
  CHECK(gov102.model_name == "TGTypeII");
  const auto omega102 = device_output_series(result, "SynchronousMachine", 2, "omega_pu");
  CHECK(omega102.y.back() == Catch::Approx(1.0).margin(1e-8));
  CHECK(series_range(omega102) < 1e-8);
}

TEST_CASE("PSD Test 45 Sauer-Pai machine profile initializes and responds",
          "[dynamics][benchmark][psd][machine][sauerpai][test45]") {
  auto sys = make_psd_test45_sauerpai_machine_case();
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
  trip.component_type = "AC";
  trip.component_index = 2;
  trip.label = "PSD Test 45 BUS 1-BUS 2 branch trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  const auto& gen101 =
      require_device_output(*result.final_snapshot(), "SynchronousMachine", 1);
  CHECK(gen101.model_name == "SauerPaiMachine");
  for (const std::string key :
       {"psd_sauerpai", "psi_q", "psi_d", "eq_p", "ed_p",
        "psi_d_pp", "psi_q_pp", "vf_pu", "p_mw", "q_mvar"}) {
    INFO(key);
    REQUIRE(gen101.values.count(key) == 1);
    CHECK(std::isfinite(gen101.values.at(key)));
  }
  const auto delta = device_output_series(result, "SynchronousMachine", 1, "angle_rad");
  const auto omega = device_output_series(result, "SynchronousMachine", 1, "omega_pu");
  CHECK(series_range(delta) > 1e-4);
  CHECK(series_range(omega) > 1e-7);
}

TEST_CASE("PSD Test 30 IEEEST stack profile initializes and responds",
          "[dynamics][benchmark][psd][controller][ieeest][test30]") {
  auto sys = make_psd_genrou_sexs_ieeest_case();
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
  trip.component_type = "AC";
  trip.component_index = 1;
  trip.label = "PSD Test 30 BUS 1-BUS 2 branch trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  const auto& gen102 =
      require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
  CHECK(gen102.model_name == "GENROU");
  const auto& avr102 = require_device_output(*result.final_snapshot(), "Exciter", 2);
  CHECK(avr102.model_name == "SEXS");
  const auto& pss102 = require_device_output(*result.final_snapshot(), "PSS", 2);
  CHECK(pss102.model_name == "IEEEST");
  REQUIRE(pss102.values.count("vs_pu") == 1);
  CHECK(std::isfinite(pss102.values.at("vs_pu")));
  const auto vf = device_output_series(result, "SynchronousMachine", 2, "vf_pu");
  const auto vs = device_output_series(result, "PSS", 2, "vs_pu");
  CHECK(series_range(vf) > 1e-4);
  CHECK(series_range(vs) > 1e-8);
}

TEST_CASE("PSD Test 25 Marconato dynamic-line fixture responds to governor step",
          "[dynamics][benchmark][psd][branch][test25]") {
  auto sys = make_psd_test25_dynamic_line_case();
  DynamicSolverOptions opt = fast_options();
  opt.solver_type = DynamicSolverType::MassMatrixDae;
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.dynamic_trim_tol = 1e-7;
  opt.max_dynamic_trim_iters = 20;
  opt.algebraic_network_max_iters = 8;
  opt.algebraic_network_tol = 1e-8;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent pref_step;
  pref_step.time_s = 1.0;
  pref_step.type = DynamicEventType::Custom;
  pref_step.component_type = "SynchronousMachine";
  pref_step.component_index = 2;
  pref_step.label = "PSD Test 25 generator-102 P_ref step";
  pref_step.params["p_ref_mw"] = 90.0;
  dyn.events.push_back(pref_step);

  const auto dynamic_line_count =
      std::count_if(dyn.devices.begin(),
                    dyn.devices.end(),
                    [](const std::unique_ptr<DynamicDevice>& device) {
                      return device->type() == "DynamicRLLine";
                    });
  CHECK(dynamic_line_count == 3);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);
  CHECK(result.initialization.power_flow_converged);
  CHECK(result.initialization.dynamic_trim_converged);
  REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= 1e-5);
  REQUIRE_FALSE(result.applied_event_records.empty());
  REQUIRE(result.snapshots.size() > 100);

  const auto v102 = bus_voltage_mag_series(result, 1);
  CHECK(series_range(v102) > 1e-4);
  const auto& gen102 =
      require_device_output(*result.final_snapshot(), "SynchronousMachine", 2);
  CHECK(gen102.model_name == "MarconatoMachine");
  const auto& gov101 = require_device_output(*result.final_snapshot(), "Governor", 1);
  CHECK(gov101.model_name == "TGTypeII");
  CHECK(gen102.values.at("p_mech_mw") == Catch::Approx(90.0));
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
    if (row->contains("execution_case_ids")) {
      REQUIRE(row->at("execution_case_ids").is_array());
      CHECK(std::find(row->at("execution_case_ids").begin(),
                      row->at("execution_case_ids").end(),
                      raw_case.at("id")) != row->at("execution_case_ids").end());
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
    CHECK(row->value("status", "") == "full-gate");

    std::size_t signal_count = 0;
    for (const auto& device : spec.devices) {
      CHECK_FALSE(device.psd_ref.empty());
      const bool is_bus_trace = device.local_component_type == "ACBus" ||
                                device.local_component_type == "BusVoltage";
      if (is_bus_trace) {
        CHECK(device.local_component_index >= 0);
      } else {
        CHECK(device.local_component_index > 0);
      }
      signal_count += device.signals.size();
      for (const auto& signal : device.signals) {
        CHECK_FALSE(signal.local_key.empty());
        CHECK_FALSE(signal.psd_quantity.empty());
        CHECK(signal.rms_tolerance > 0.0);
        CHECK(signal.max_tolerance > 0.0);
        CHECK(signal.local_scale > 0.0);
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

  CHECK(enabled_count >= 9);
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

TEST_CASE("PSD executable manifest gates promoted full-contract rows vs MassMatrixDae",
          "[dynamics][benchmark][psd][manifest][external]") {
  const char* run_psd = std::getenv("HACDCPF_RUN_PSD_COMPARE");
  const bool run_external_psd = run_psd != nullptr && std::string(run_psd) == "1";
  if (!run_external_psd) {
    SUCCEED("Set HACDCPF_RUN_PSD_COMPARE=1 to run executable PSD manifest gates");
    return;
  }

  const std::filesystem::path out_dir =
      std::filesystem::temp_directory_path() / "hacdcpf_psd_manifest_validation";
  std::filesystem::create_directories(out_dir);

  const Json payload = load_psd_component_test_matrix();
  std::vector<ManifestTraceComparison> traces;
  for (const auto& raw_case : payload.at("execution_manifest").at("cases")) {
    if (!raw_case.value("enabled", false)) continue;
    const ManifestExecutableCase spec = parse_manifest_executable_case(raw_case);
    REQUIRE(spec.reference_formulation == "ResidualModel");
    REQUIRE(spec.reference_integrator == "IDA");
    REQUIRE(spec.hacdcpf_solver == "MassMatrixDae");

    const DynamicResults result = run_manifest_hacdcpf_case(spec);
    INFO(spec.id << ": " << result.message);
    INFO(spec.id << ": trim_converged="
                 << result.initialization.dynamic_trim_converged
                 << ", trim_iters="
                 << result.initialization.dynamic_trim_iterations
                 << ", fast_dxdt_inf_norm="
                 << result.initialization.dynamic_fast_dxdt_inf_norm);
    INFO(spec.id << ": steps=" << result.steps
                 << ", failed_step=" << result.failed_step
                 << ", rejected_steps=" << result.rejected_steps
                 << ", min_dt=" << result.min_accepted_step_s
                 << ", max_dt=" << result.max_accepted_step_s);
    for (const auto& diag : result.initialization.dynamic_residual_diagnostics) {
      INFO(spec.id << ": residual contributor "
                   << diag.device_type << "#" << diag.component_index
                   << " state[" << diag.state_index << "] = "
                   << diag.residual << " (" << diag.device_name << ")");
    }
    REQUIRE(result.success);
    CHECK(result.initialization.power_flow_converged);
    CHECK(result.initialization.dynamic_trim_converged);
    REQUIRE(result.initialization.dynamic_fast_dxdt_inf_norm <= 1e-5);
    if (spec.event.time_s <= spec.t_end_s) {
      REQUIRE_FALSE(result.applied_event_records.empty());
    }
    REQUIRE(result.snapshots.size() > 100);

    std::vector<ManifestTraceComparison> case_traces =
        build_manifest_trace_comparisons(spec, result, out_dir);
    traces.insert(traces.end(),
                  std::make_move_iterator(case_traces.begin()),
                  std::make_move_iterator(case_traces.end()));
  }
  REQUIRE(traces.size() >= 32);

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
    trip.component_index = 1;
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

TEST_CASE("PSD grid-forming VSM and VOC outer-control profiles run numerically",
          "[dynamics][gfm][psd][ibr]") {
  struct CaseSpec {
    std::string model;
    std::map<std::string, double> params;
  };
  const std::vector<CaseSpec> cases = {
      {"VSMGridForming",
       {{"Ta", 2.0},
        {"kd", 40.0},
        {"komega", 20.0},
        {"q_droop_pu", 0.2},
        {"reactive_power_filter_t_s", 0.001},
        {"virtual_x_pu", 0.12}}},
      {"VOCGridForming",
       {{"k1", 0.0033},
        {"psi", 3.14159265358979323846 / 4.0},
        {"k2", 0.0796},
        {"virtual_x_pu", 0.12}}},
  };

  for (const auto& spec : cases) {
    auto sys = make_hybrid_dc_case();
    sys.vsc_converters[0].ac_grid_forming = true;
    sys.vsc_converters[0].grid_forming = false;
    sys.vsc_converters[0].control_mode = ConverterMode::AC_GRID_FORMING;
    sys.vsc_converters[0].p_set_mw = 8.0;
    sys.vsc_converters[0].p_schedule_mw = 8.0;
    sys.vsc_converters[0].q_set_mvar = 1.0;
    sys.vsc_converters[0].dynamic_model.standard = "PowerSimulationsDynamics";
    sys.vsc_converters[0].dynamic_model.model_name = spec.model;
    sys.vsc_converters[0].dynamic_model.parameters = spec.params;

    auto opt = fast_options();
    opt.t_end_s = 0.10;
    opt.dt_s = 0.005;
    opt.run_power_flow_initialization = true;

    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent pref_step;
    pref_step.time_s = 0.04;
    pref_step.type = DynamicEventType::Custom;
    pref_step.component_type = "VSC";
    pref_step.component_index = 1;
    pref_step.params["p_ref_mw"] = 12.0;
    dyn.events.push_back(pref_step);

    DynamicSolver solver;
    const DynamicResults result = solver.solve(dyn);
    INFO(spec.model);
    REQUIRE(result.success);
    REQUIRE(result.final_snapshot() != nullptr);
    const auto& out = require_device_output(*result.final_snapshot(), "VSCGridForming", 1);
    CHECK(out.model_name == spec.model);
    CHECK(out.values.at("theta_oc_rad") == Catch::Approx(out.values.at("angle_rad")));
    CHECK(std::isfinite(out.values.at("omega_oc_pu")));
    CHECK(out.values.at("frequency_hz") > 45.0);
    CHECK(out.values.at("frequency_hz") < 55.0);
    CHECK(out.values.at("E_oc_pu") > 0.2);
    CHECK(out.values.at("E_oc_pu") < 1.5);
  }
}

TEST_CASE("PSD standalone IBR dynamic injectors initialize and run",
          "[dynamics][psd][ibr]") {
  SECTION("PeriodicVariableSource") {
    auto sys = make_transient_2bus();
    sys.ac.generators.clear();
    ExternalGrid grid;
    grid.index = 1;
    grid.bus = 1;
    grid.name = "InfBus";
    grid.vm_pu = 1.0;
    grid.x_pu = 0.05;
    grid.in_service = true;
    grid.dynamic_model.standard = "PowerSimulationsDynamics";
    grid.dynamic_model.model_name = "PeriodicVariableSource";
    grid.dynamic_model.parameters = {
        {"internal_voltage_bias", 1.0},
        {"internal_voltage_frequency_rad_s", 2.0 * 3.14159265358979323846},
        {"internal_voltage_sin_coeff", 0.02},
        {"internal_angle_frequency_rad_s", 2.0 * 3.14159265358979323846},
        {"internal_angle_cos_coeff", 0.02},
        {"X_th", 0.05}};
    sys.ac.external_grids = {grid};

    auto opt = fast_options();
    opt.t_end_s = 0.20;
    opt.dt_s = 0.01;
    const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);
    REQUIRE(result.success);
    REQUIRE(result.final_snapshot() != nullptr);
    const auto& out = require_device_output(*result.final_snapshot(), "PeriodicVariableSource", 1);
    CHECK(out.model_name == "PeriodicVariableSource");
    CHECK(out.values.count("Vt") == 1);
    CHECK(std::abs(out.values.at("Vt") - 1.0) > 1e-4);
    CHECK(std::isfinite(out.values.at("current_real_pu")));
  }

  SECTION("CSVGN1") {
    auto sys = make_transient_2bus();
    StaticGenerator shunt;
    shunt.index = 1;
    shunt.bus = 2;
    shunt.name = "CSVGN1";
    shunt.q_mvar = 5.0;
    shunt.in_service = true;
    shunt.dynamic_model.standard = "PSS/E";
    shunt.dynamic_model.model_name = "CSVGN1";
    shunt.dynamic_model.parameters = {{"K", 20.0}, {"T3", 0.154833}, {"T5", 0.005167},
                                      {"CBase", 60.0}, {"base_power", 500.0}};
    sys.ac.static_generators = {shunt};

    auto opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 0.08;
    opt.dt_s = 0.002;
    opt.algebraic_network_max_iters = 8;
    opt.algebraic_network_tol = 1e-5;
    const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);
    INFO(result.message);
    REQUIRE(result.success);
    REQUIRE(result.final_snapshot() != nullptr);
    const auto& out = require_device_output(*result.final_snapshot(), "CSVGN1", 1);
    CHECK(out.model_name == "CSVGN1");
    CHECK(std::isfinite(out.values.at("b_pu")));
    CHECK(std::isfinite(out.values.at("q_mvar")));
  }

  SECTION("AggregateDistributedGenerationA") {
    auto sys = make_transient_2bus();
    StaticGenerator dera;
    dera.index = 1;
    dera.bus = 2;
    dera.name = "DERA";
    dera.p_mw = 12.0;
    dera.q_mvar = 2.0;
    dera.sn_mva = 20.0;
    dera.v_ref_pu = 1.0;
    dera.in_service = true;
    dera.dynamic_model.standard = "PSS/E";
    dera.dynamic_model.model_name = "AggregateDistributedGenerationA";
    dera.dynamic_model.parameters = {{"base_power", 20.0}, {"Freq_Flag", 1.0},
                                     {"K_qv", 5.0}, {"Tg", 0.02},
                                     {"Tpord", 0.02}};
    sys.ac.static_generators = {dera};

    auto opt = fast_options();
    opt.t_end_s = 0.10;
    opt.dt_s = 0.005;
    const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);
    REQUIRE(result.success);
    REQUIRE(result.final_snapshot() != nullptr);
    const auto& out =
        require_device_output(*result.final_snapshot(), "AggregateDistributedGenerationA", 1);
    CHECK(out.model_name == "AggregateDistributedGenerationA");
    CHECK(out.values.at("freq_flag") == Catch::Approx(1.0));
    CHECK(std::isfinite(out.values.at("Ip")));
    CHECK(std::isfinite(out.values.at("Iq")));
  }
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
    // The time-domain estimate is peak-picked after a finite load step, so keep
    // this as a modal sanity bound rather than a precision eigentrace check.
    CHECK(std::abs(em.frequency_hz - f_time) / f_time < 0.12);
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

TEST_CASE("Small-signal screening of the new IBR and dynamic-load models",
          "[dynamics][smallsignal][newmodels]") {
  // Design doc §18: linearize about the initialized operating point and screen
  // the WECC renewable stack, the flux induction machine, and the active CPL for
  // small-signal stability. These are new device models (this session); the
  // screen catches an unstable equilibrium (right-half-plane eigenvalue) before
  // it shows up as a time-domain divergence.
  auto analyze = [](const HybridPowerSystem& sys) {
    DynamicSolverOptions opt;
    opt.use_consistent_dynamic_initialization = true;
    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    return small_signal_analysis(dyn);
  };
  // Least-damped mode dominated by the named device type. The global least-
  // damped mode is the reference machine's zero angle mode (Re~=0), which masks
  // the new device's own dynamics, so screen the device-dominated modes instead.
  auto device_worst = [](const SmallSignalResult& r, const std::string& type) {
    SmallSignalMode worst;
    worst.eigen_real = -1e30;
    bool found = false;
    for (const auto& m : r.modes) {
      if (m.dominant_state.find(type) == std::string::npos) continue;
      found = true;
      if (m.eigen_real > worst.eigen_real) worst = m;
    }
    return std::make_pair(found, worst);
  };

  SECTION("WECC REGC_A + REEC_A + REPC_A renewable plant") {
    HybridPowerSystem sys = make_transient_2bus();
    StaticGenerator der;
    der.index = 1;
    der.bus = 2;
    der.in_service = true;
    der.p_mw = 12.0;
    der.q_mvar = 0.0;
    der.sn_mva = 30.0;
    der.f_ref_hz = 50.0;
    der.dynamic_model.standard = "WECC";
    der.dynamic_model.model_name = "REGC_A";
    der.dynamic_model.parameters = {{"reec_enabled", 1.0}, {"repc_enabled", 1.0}};
    sys.ac.static_generators.push_back(der);
    const SmallSignalResult ss = analyze(sys);
    REQUIRE(ss.success);
    CHECK(static_cast<int>(ss.modes.size()) == ss.n_differential);
    const auto [found, worst] = device_worst(ss, "RenewableEnergyGeneratorA");
    REQUIRE(found);
    INFO("REGC_A device worst Re=" << worst.eigen_real << " f=" << worst.frequency_hz
         << " zeta=" << worst.damping_ratio << " dom=" << worst.dominant_state
         << " ndiff=" << ss.n_differential);
    // The renewable plant's own modes must sit in the left half-plane.
    CHECK(worst.eigen_real < 1e-6);
    CHECK(worst.damping_ratio > 0.0);
  }

  SECTION("flux single-cage induction motor") {
    HybridPowerSystem sys = make_transient_2bus();
    AsynchronousMotor m;
    m.index = 1;
    m.bus = 2;
    m.in_service = true;
    m.sn_mva = 10.0;
    m.cos_phi = 0.85;
    m.efficiency = 0.95;
    m.r_pu = 0.03;
    m.x_pu = 0.20;
    m.dynamic_model.model_name = "FluxInductionMachine";
    sys.ac.motors = {m};
    const SmallSignalResult ss = analyze(sys);
    REQUIRE(ss.success);
    CHECK(static_cast<int>(ss.modes.size()) == ss.n_differential);
    const auto [found, worst] = device_worst(ss, "SingleCageInductionMachine");
    REQUIRE(found);
    INFO("FluxMotor device worst Re=" << worst.eigen_real << " f=" << worst.frequency_hz
         << " zeta=" << worst.damping_ratio << " dom=" << worst.dominant_state
         << " ndiff=" << ss.n_differential);
    // A loaded induction motor at its stable slip is small-signal stable.
    CHECK(worst.eigen_real < 1e-6);
    CHECK(worst.damping_ratio > 0.0);
  }

  SECTION("active constant-power load") {
    HybridPowerSystem sys = make_transient_2bus();
    Load cpl;
    cpl.index = 2;
    cpl.bus = 2;
    cpl.p_mw = 10.0;
    cpl.q_mvar = 2.0;
    cpl.in_service = true;
    cpl.dynamic_model.model_name = "ActiveConstantPowerLoad";
    sys.ac.loads.push_back(cpl);
    const SmallSignalResult ss = analyze(sys);
    REQUIRE(ss.success);
    CHECK(static_cast<int>(ss.modes.size()) == ss.n_differential);
    const auto [found, worst] = device_worst(ss, "ActiveConstantPowerLoad");
    REQUIRE(found);
    INFO("CPL device worst Re=" << worst.eigen_real << " f=" << worst.frequency_hz
         << " zeta=" << worst.damping_ratio << " dom=" << worst.dominant_state
         << " ndiff=" << ss.n_differential);
    // The LC-filtered CPL settles (R damping dominates the negative-resistance
    // term at this loading), so its own mode is stable though lightly damped.
    CHECK(worst.eigen_real < 1e-6);
    CHECK(worst.damping_ratio > 0.0);
  }
}

TEST_CASE("compute_small_signal attaches a modal summary to the transient result",
          "[dynamics][smallsignal][transient]") {
  // Design doc §18: a transient run can optionally carry a small-signal screen
  // about its initialized operating point, so the GUI can plot eigenvalues /
  // damping alongside the time-domain trajectory without a second request.
  HybridPowerSystem sys = make_transient_2bus();
  StaticGenerator der;
  der.index = 1;
  der.bus = 2;
  der.in_service = true;
  der.p_mw = 12.0;
  der.q_mvar = 0.0;
  der.sn_mva = 30.0;
  der.f_ref_hz = 50.0;
  der.dynamic_model.standard = "WECC";
  der.dynamic_model.model_name = "REGC_A";
  der.dynamic_model.parameters = {{"reec_enabled", 1.0}, {"repc_enabled", 1.0}};
  sys.ac.static_generators.push_back(der);

  SECTION("off by default: no modal summary is computed") {
    auto opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 0.2;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    REQUIRE(result.success);
    CHECK_FALSE(result.modal.computed);
    CHECK(result.modal.modes.empty());
  }

  SECTION("opt-in: eigenvalues attached, trajectory unaffected") {
    auto opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 0.2;
    // Baseline trajectory without the modal screen.
    const auto base = hacdcpf::run_transient_simulation(sys, opt);
    REQUIRE(base.success);
    opt.compute_small_signal = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    REQUIRE(result.success);
    REQUIRE(result.modal.computed);
    REQUIRE(result.modal.success);
    CHECK(result.modal.n_differential > 0);
    CHECK(static_cast<int>(result.modal.modes.size()) == result.modal.n_differential);
    // Modes are least-damped first; min_damping_ratio matches the leading mode.
    CHECK(result.modal.min_damping_ratio ==
          Catch::Approx(result.modal.modes.front().damping_ratio));
    // The renewable device contributes at least one mode.
    bool has_regca = false;
    for (const auto& m : result.modal.modes)
      if (m.dominant_state.find("RenewableEnergyGeneratorA") != std::string::npos)
        has_regca = true;
    CHECK(has_regca);
    // Each mode carries its significant participation factors (descending, in
    // (0,1]) so the GUI can show which states drive the eigenvalue.
    const auto& lead = result.modal.modes.front();
    REQUIRE_FALSE(lead.participation.empty());
    double prev = 2.0;
    for (const auto& p : lead.participation) {
      CHECK(p.factor > 0.0);
      CHECK(p.factor <= 1.0 + 1e-9);
      CHECK(p.factor <= prev + 1e-9);  // sorted descending
      prev = p.factor;
    }
    // The leading contributor's label matches the mode's dominant state.
    CHECK(lead.participation.front().state_label == lead.dominant_state);
    // A human-readable Markdown report renders the screen for docs / sharing.
    const std::string report = hacdcpf::dynamics::to_modal_report(result.modal);
    CHECK(report.find("### Small-signal (modal) screen") != std::string::npos);
    CHECK((report.find("**Stable**") != std::string::npos ||
           report.find("**UNSTABLE**") != std::string::npos));
    CHECK(report.find("| # | Re(λ) [1/s]") != std::string::npos);
    CHECK(report.find("Dominant state") != std::string::npos);
    CHECK(report.find("participation") != std::string::npos);
    CHECK(report.find(":s") != std::string::npos);  // a device state label
    CHECK(report.find('%') != std::string::npos);   // participation percentages
    // An uncomputed screen produces no report.
    CHECK(hacdcpf::dynamics::to_modal_report(base.modal).empty());
    // The screen must not perturb the time-domain result: same final voltage.
    REQUIRE(result.final_snapshot() != nullptr);
    REQUIRE(base.final_snapshot() != nullptr);
    CHECK(result.final_snapshot()->min_ac_voltage_pu ==
          Catch::Approx(base.final_snapshot()->min_ac_voltage_pu).epsilon(1e-9));
  }
}

TEST_CASE("CPL small-signal stability boundary: filter damping vs loading",
          "[dynamics][smallsignal][cpl]") {
  // Design doc §13/§18: characterize the active constant-power load's small-signal
  // mode with the modal screen. In this RMS-phasor formulation the constant-power
  // stage current i_load = conj(S/Vc) is TRACE-FREE (d i_r/d a + d i_i/d b = 0), so
  // the negative-resistance term rotates the mode without shifting its real part.
  // The damping is therefore governed by the filter conductance
  // Re(Y_f) = R_f/(R_f^2 + X_f^2) and capacitance C, and is essentially
  // independent of the constant-power loading P. Lowering R_f or raising X_f
  // reduces Re(Y_f) and walks the mode toward the imaginary axis; loading does
  // not (the classic load-driven CPL instability needs a full dq/EMT model).
  auto cpl_mode_real = [](double filter_r_pu, double filter_x_pu, double p_mw,
                          double filter_c_s) -> double {
    HybridPowerSystem sys = make_transient_2bus();
    Load cpl;
    cpl.index = 2;
    cpl.bus = 2;
    cpl.p_mw = p_mw;
    cpl.q_mvar = 0.0;
    cpl.in_service = true;
    cpl.dynamic_model.model_name = "ActiveConstantPowerLoad";
    cpl.dynamic_model.parameters = {{"filter_r_pu", filter_r_pu},
                                    {"filter_x_pu", filter_x_pu},
                                    {"filter_c_s", filter_c_s}};
    sys.ac.loads.push_back(cpl);
    DynamicSolverOptions opt;
    opt.use_consistent_dynamic_initialization = true;
    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    const SmallSignalResult ss = small_signal_analysis(dyn);
    if (!ss.success) return std::numeric_limits<double>::quiet_NaN();
    double worst = -1e30;
    for (const auto& m : ss.modes)
      if (m.dominant_state.find("ActiveConstantPowerLoad") != std::string::npos)
        worst = std::max(worst, m.eigen_real);
    return worst;
  };

  SECTION("raising filter reactance walks the mode toward the boundary") {
    const double r = 0.005, p = 30.0, c = 0.02;
    const std::vector<double> xs = {0.05, 0.10, 0.20, 0.30, 0.50, 0.70};
    double prev = -1e30;
    for (double x : xs) {
      const double re = cpl_mode_real(r, x, p, c);
      INFO("X=" << x << " Re=" << re);
      REQUIRE(std::isfinite(re));
      CHECK(re < 0.0);               // stable across the physical range
      CHECK(re > prev - 1e-6);       // higher X_f => lower Re(Y_f) => less damped
      prev = re;
    }
    // The mode approaches (but does not cross) the imaginary axis.
    CHECK(cpl_mode_real(r, 0.70, p, c) > -3.0);
    CHECK(cpl_mode_real(r, 0.05, p, c) < -8.0);
  }

  SECTION("lowering filter resistance reduces damping monotonically") {
    const double x = 0.30, p = 30.0, c = 0.02;
    const std::vector<double> rs = {0.12, 0.08, 0.05, 0.03, 0.02, 0.01, 0.005, 0.002};
    double prev = -1e30;
    for (double r : rs) {
      const double re = cpl_mode_real(r, x, p, c);
      INFO("R=" << r << " Re=" << re);
      REQUIRE(std::isfinite(re));
      CHECK(re < 0.0);
      CHECK(re > prev - 1e-6);       // decreasing R_f => less damped
      prev = re;
    }
  }

  SECTION("constant-power loading barely moves the mode (trace-free stage)") {
    const double r = 0.005, x = 0.30, c = 0.02;
    const double base = cpl_mode_real(r, x, 2.0, c);
    REQUIRE(std::isfinite(base));
    for (double p : {2.0, 20.0, 40.0, 60.0}) {
      const double re = cpl_mode_real(r, x, p, c);
      INFO("P=" << p << " Re=" << re);
      CHECK(re < 0.0);
      // The trace-free power stage leaves the real part set by the filter: a
      // 30x loading change shifts it by < 1% of the filter-established damping.
      CHECK(std::abs(re - base) < 0.05);
    }
  }

  SECTION("capacitance scales the mode rate but not its stability") {
    // C only scales the eigenvalue magnitude (1/C), never its sign.
    const double r = 0.01, x = 0.20, p = 20.0;
    const double slow = cpl_mode_real(r, x, p, 0.05);
    const double fast = cpl_mode_real(r, x, p, 0.005);
    INFO("C=0.05 Re=" << slow << "  C=0.005 Re=" << fast);
    REQUIRE(std::isfinite(slow));
    REQUIRE(std::isfinite(fast));
    CHECK(slow < 0.0);
    CHECK(fast < 0.0);
    CHECK(fast < slow);              // smaller C => faster (more negative) mode
  }
}

TEST_CASE("Explicit-solver stiffness audit: fast AVR transducers need implicit integration",
          "[dynamics][smallsignal][stiffness]") {
  // Audit companion to the OneDOneQ fix. The PSD machine fixtures carry an AVR
  // TypeI with a 1 ms voltage transducer (Tr = 0.001 s); its eigenvalue at
  // -1/Tr = -1000 lies far outside the explicit RK2 (Heun) stability region for
  // the usual dt = 5 ms (which needs Re > -2/dt = -400). So explicit integration
  // over a non-trivial horizon diverges, while an A-stable implicit method does
  // not. This documents the stiffness and guards the solver-choice contract:
  // stiff fast-transducer models MUST be integrated implicitly.
  auto fastest_eigen_real = [](const HybridPowerSystem& sys) {
    DynamicSolverOptions opt;
    opt.run_power_flow_initialization = true;
    opt.use_consistent_dynamic_initialization = true;
    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    const auto ss = small_signal_analysis(dyn);
    double fastest = 0.0;
    for (const auto& m : ss.modes) fastest = std::min(fastest, m.eigen_real);
    return fastest;
  };

  SECTION("PSD machine fixtures are stiff (fast AVR transducer mode)") {
    // -1/Tr = -1000 for the 1 ms transducer; require a decade of margin so this
    // flags any future fixture change that reintroduces an ultra-fast lag.
    CHECK(fastest_eigen_real(make_psd_onedoneq_three_bus_subset_case()) <= -900.0);
    CHECK(fastest_eigen_real(make_psd_simple_marconato_three_bus_case()) <= -900.0);
  }

  SECTION("explicit Heun diverges where implicit integrators stay stable") {
    const auto sys = make_psd_simple_marconato_three_bus_case();
    auto run = [&](DynamicSolverType type) {
      DynamicSolverOptions opt = fast_options();
      opt.run_power_flow_initialization = true;
      opt.solver_type = type;
      opt.dt_s = 0.005;
      opt.t_end_s = 0.20;  // beyond the ~0.085 s explicit blow-up horizon
      return hacdcpf::run_transient_simulation(sys, opt);
    };
    // dt*lambda = -5 for the -1000 transducer mode: outside RK2's [-2, 0] region.
    CHECK_FALSE(run(DynamicSolverType::PartitionedHeun).success);
    // A-stable implicit integrators resolve the fast transducer cleanly.
    CHECK(run(DynamicSolverType::TrapezoidalNewton).success);
    CHECK(run(DynamicSolverType::MassMatrixDae).success);
  }

  SECTION("auto_select_stiff_solver rescues an explicit run on a stiff system") {
    const auto sys = make_psd_simple_marconato_three_bus_case();
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.solver_type = DynamicSolverType::PartitionedHeun;  // would diverge alone
    opt.dt_s = 0.005;
    opt.t_end_s = 0.20;
    opt.auto_select_stiff_solver = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    CHECK(result.success);  // auto-switched to an implicit method => stable
    bool warned = false;
    for (const auto& w : result.warnings)
      if (w.find("auto_select_stiff_solver") != std::string::npos) warned = true;
    CHECK(warned);  // the switch is reported
  }

  SECTION("auto_select_stiff_solver leaves a non-stiff explicit run untouched") {
    const auto sys = make_transient_2bus();  // no fast transducer mode
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.solver_type = DynamicSolverType::PartitionedHeun;
    opt.dt_s = 0.005;
    opt.t_end_s = 0.05;
    opt.auto_select_stiff_solver = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    CHECK(result.success);
    bool warned = false;
    for (const auto& w : result.warnings)
      if (w.find("auto_select_stiff_solver") != std::string::npos) warned = true;
    CHECK_FALSE(warned);  // Heun is already the cheapest stable choice: no change
  }

  SECTION("auto_select_stiff_solver downshifts a needlessly expensive explicit run") {
    const auto sys = make_transient_2bus();  // non-stiff: Heun suffices
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.solver_type = DynamicSolverType::PartitionedRK4;  // 4 evals/step, overkill here
    opt.dt_s = 0.005;
    opt.t_end_s = 0.05;
    opt.auto_select_stiff_solver = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    CHECK(result.success);
    bool downshift = false;
    for (const auto& w : result.warnings)
      if (w.find("PartitionedRK4 -> PartitionedHeun") != std::string::npos) downshift = true;
    CHECK(downshift);  // cheaper 2nd-order explicit is stable => downshift
  }
}

TEST_CASE("Mass-matrix DAE core matches the partitioned backward-Euler oracle",
          "[dynamics][dae]") {
  // The simultaneous DAE (bus voltages as algebraic states, one sparse Newton
  // solve per step) must reproduce the nested-network backward-Euler result,
  // since both are backward Euler on the same system.
  auto run_solver = [](DynamicSolverType type,
                       bool event,
                       bool dae_reuse_jacobian_factorization = true,
                       DynamicDaeStepMethod dae_step_method =
                           DynamicDaeStepMethod::BackwardEuler,
                       bool use_analytic_network_jacobian = true,
                       bool use_adaptive_step = false,
                       double dt_s = 0.005,
                       double t_end_s = 3.0) {
    DynamicSolverOptions opt;
    opt.solver_type = type;
    opt.t_end_s = t_end_s;
    opt.dt_s = dt_s;
    opt.algebraic_network_max_iters = 20;
    opt.algebraic_network_tol = 1e-10;
    opt.use_consistent_dynamic_initialization = true;
    opt.dae_reuse_jacobian_factorization = dae_reuse_jacobian_factorization;
    opt.dae_step_method = dae_step_method;
    opt.dae_use_analytic_network_jacobian = use_analytic_network_jacobian;
    opt.use_adaptive_step = use_adaptive_step;
    opt.abs_tol = 1e-9;
    opt.rel_tol = 1e-7;
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

  SECTION("reuses the MassMatrixDae Jacobian/factorization without changing the trace") {
    const DynamicResults fresh =
        run_solver(DynamicSolverType::MassMatrixDae, true, false);
    const DynamicResults reused =
        run_solver(DynamicSolverType::MassMatrixDae, true, true);
    REQUIRE(fresh.success);
    REQUIRE(reused.success);
    CHECK(fresh.jacobian_evaluations == fresh.linear_factorizations);
    CHECK(reused.jacobian_evaluations == reused.linear_factorizations);
    REQUIRE(fresh.linear_factorizations > 0);
    REQUIRE(reused.linear_factorizations > 0);
    CHECK(reused.linear_factorizations * 2 < fresh.linear_factorizations);

    const auto of = device_output_series(fresh, "SynchronousMachine", 2, "omega_pu");
    const auto oru = device_output_series(reused, "SynchronousMachine", 2, "omega_pu");
    const auto pf = device_output_series(fresh, "SynchronousMachine", 2, "p_mw");
    const auto pru = device_output_series(reused, "SynchronousMachine", 2, "p_mw");
    REQUIRE(of.y.size() == oru.y.size());
    double omega_maxdiff = 0.0;
    double p_maxdiff = 0.0;
    for (std::size_t i = 0; i < of.y.size(); ++i) {
      omega_maxdiff = std::max(omega_maxdiff, std::abs(of.y[i] - oru.y[i]));
      p_maxdiff = std::max(p_maxdiff, std::abs(pf.y[i] - pru.y[i]));
    }
    CHECK(omega_maxdiff < 1e-7);
    CHECK(p_maxdiff < 1e-4);
  }

  SECTION("analytic MassMatrixDae network block preserves the FD Jacobian contract") {
    const DynamicResults fd =
        run_solver(DynamicSolverType::MassMatrixDae, true, true,
                   DynamicDaeStepMethod::BackwardEuler, false);
    const DynamicResults analytic =
        run_solver(DynamicSolverType::MassMatrixDae, true, true,
                   DynamicDaeStepMethod::BackwardEuler, true);
    REQUIRE(fd.success);
    REQUIRE(analytic.success);
    const auto of = device_output_series(fd, "SynchronousMachine", 2, "omega_pu");
    const auto oa = device_output_series(analytic, "SynchronousMachine", 2, "omega_pu");
    const auto pf = device_output_series(fd, "SynchronousMachine", 2, "p_mw");
    const auto pa = device_output_series(analytic, "SynchronousMachine", 2, "p_mw");
    REQUIRE(of.y.size() == oa.y.size());
    double omega_maxdiff = 0.0;
    double p_maxdiff = 0.0;
    for (std::size_t i = 0; i < of.y.size(); ++i) {
      omega_maxdiff = std::max(omega_maxdiff, std::abs(of.y[i] - oa.y[i]));
      p_maxdiff = std::max(p_maxdiff, std::abs(pf.y[i] - pa.y[i]));
    }
    CHECK(omega_maxdiff < 1e-8);
    CHECK(p_maxdiff < 1e-5);
  }

  SECTION("trapezoidal MassMatrixDae converges under step refinement") {
    const DynamicResults coarse =
        run_solver(DynamicSolverType::MassMatrixDae, true, true,
                   DynamicDaeStepMethod::Trapezoidal, true, false, 0.02, 1.4);
    const DynamicResults medium =
        run_solver(DynamicSolverType::MassMatrixDae, true, true,
                   DynamicDaeStepMethod::Trapezoidal, true, false, 0.01, 1.4);
    const DynamicResults fine =
        run_solver(DynamicSolverType::MassMatrixDae, true, true,
                   DynamicDaeStepMethod::Trapezoidal, true, false, 0.005, 1.4);
    REQUIRE(coarse.success);
    REQUIRE(medium.success);
    REQUIRE(fine.success);
    const double oc =
        device_output_series(coarse, "SynchronousMachine", 2, "omega_pu").y.back();
    const double om =
        device_output_series(medium, "SynchronousMachine", 2, "omega_pu").y.back();
    const double of =
        device_output_series(fine, "SynchronousMachine", 2, "omega_pu").y.back();
    const double coarse_error = std::abs(oc - of);
    const double medium_error = std::abs(om - of);
    CHECK(coarse_error > medium_error);
    CHECK(medium_error < 2e-6);
  }

  SECTION("embedded BE/TR DAE pair rejects oversized adaptive steps") {
    DynamicResults adaptive =
        run_solver(DynamicSolverType::MassMatrixDae, true, true,
                   DynamicDaeStepMethod::Trapezoidal, true, true, 0.05, 1.25);
    REQUIRE(adaptive.success);
    CHECK(adaptive.rejected_steps > 0);
    CHECK(adaptive.max_local_error_norm <= 1.0);
    CHECK(adaptive.min_accepted_step_s < 0.05);
    CHECK(adaptive.max_accepted_step_s <= 0.05 + 1e-12);
  }
}

TEST_CASE("Analytic device current Jacobian blocks match finite-difference stamps",
          "[dynamics][dae][jacobian]") {
  SECTION("constant-power AC load voltage current derivatives") {
    ACLoadDynamicParams params;
    params.component_index = 1;
    params.bus = 1;
    params.bus_pos = 0;
    params.p_mw = 30.0;
    params.q_mvar = 10.0;
    params.base_mva = 100.0;
    params.model_kind = DynamicLoadModelKind::ConstantPower;
    DynamicLoad load(params);

    DynamicState x;
    x.resize(0);
    NetworkState y;
    y.resize(3, 0);
    set_balanced_voltage(y, 0, std::polar(1.03, 0.08));
    const DynamicJacobianContext context = test_jacobian_context(0, 3, 0);

    check_device_current_jacobian_column(load, x, y, context, context.acRealCol(0));
    check_device_current_jacobian_column(load, x, y, context, context.acImagCol(0));
  }

  SECTION("dynamic RL line state current derivatives") {
    DynamicRLLineParams params;
    params.component_index = 2;
    params.from_bus = 1;
    params.to_bus = 2;
    params.from_pos = 0;
    params.to_pos = 1;
    params.r_pu = 0.02;
    params.x_pu = 0.08;
    DynamicRLLine line(params);

    int offset = 0;
    line.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[0] = 0.18;
    x.x[1] = -0.06;
    NetworkState y;
    y.resize(6, 0);
    set_balanced_voltage(y, 0, std::polar(1.02, 0.04));
    set_balanced_voltage(y, 1, std::polar(0.98, -0.03));
    const DynamicJacobianContext context = test_jacobian_context(offset, 6, 0);

    check_device_current_jacobian_column(line, x, y, context, 0);
    check_device_current_jacobian_column(line, x, y, context, 1);
  }

  SECTION("grid-following inverter AC and DC current derivatives") {
    GridFollowingInverterParams params;
    params.component_index = 3;
    params.bus = 1;
    params.bus_pos = 0;
    params.dc_bus_pos = 0;
    params.stamp_dc_power = true;
    params.base_mva = 100.0;
    params.eta = 0.97;
    GridFollowingInverter inverter(params);

    int offset = 0;
    inverter.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[0] = 0.12;
    x.x[1] = 0.0;
    x.x[2] = 0.11;
    x.x[3] = -0.04;
    x.x[4] = 0.08;
    x.x[5] = 0.02;
    NetworkState y;
    y.resize(3, 1);
    set_balanced_voltage(y, 0, std::polar(1.01, 0.02));
    y.Vdc[0] = 1.04;
    const DynamicJacobianContext context = test_jacobian_context(offset, 3, 1);

    check_device_current_jacobian_column(inverter, x, y, context, 0);
    check_device_current_jacobian_column(inverter, x, y, context, 2);
    check_device_current_jacobian_column(inverter, x, y, context, 3);
    check_device_current_jacobian_column(inverter, x, y, context, 4);
    check_device_current_jacobian_column(inverter, x, y, context, context.dcCol(0));
  }

  SECTION("CSVGN1 current derivatives") {
    CSVGN1DynamicParams params;
    params.component_index = 4;
    params.bus = 1;
    params.bus_pos = 0;
    params.base_mva = 100.0;
    params.model_base_mva = 500.0;
    CSVGN1Dynamic csvgn1(params);

    int offset = 0;
    csvgn1.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[0] = 0.07;
    x.x[1] = 0.0;
    x.x[2] = 0.07;
    NetworkState y;
    y.resize(3, 0);
    set_balanced_voltage(y, 0, std::polar(0.99, -0.05));
    const DynamicJacobianContext context = test_jacobian_context(offset, 3, 0);

    check_device_current_jacobian_column(csvgn1, x, y, context, 0);
    check_device_current_jacobian_column(csvgn1, x, y, context, context.acRealCol(0));
    check_device_current_jacobian_column(csvgn1, x, y, context, context.acImagCol(0));
  }

  SECTION("DC/DC converter current derivatives") {
    DCDCConverterDynamicParams params;
    params.component_index = 5;
    params.bus_in = 1;
    params.bus_out = 2;
    params.bus_in_pos = 0;
    params.bus_out_pos = 1;
    params.eta = 0.96;
    DCDCConverterDynamic converter(params);

    int offset = 0;
    converter.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    x.x[0] = 0.06;
    NetworkState y;
    y.resize(0, 2);
    y.Vdc[0] = 1.05;
    y.Vdc[1] = 0.97;
    const DynamicJacobianContext context = test_jacobian_context(offset, 0, 2);

    check_device_current_jacobian_column(converter, x, y, context, 0);
    check_device_current_jacobian_column(converter, x, y, context, context.dcCol(0));
    check_device_current_jacobian_column(converter, x, y, context, context.dcCol(1));
  }
}

TEST_CASE("MassMatrixDae analytic device current block preserves hybrid AC/DC trace",
          "[dynamics][dae][jacobian][transient]") {
  auto run = [](bool use_device_jacobian) {
    DynamicSolverOptions opt = fast_options();
    opt.solver_type = DynamicSolverType::MassMatrixDae;
    opt.t_end_s = 0.03;
    opt.dt_s = 0.01;
    opt.newton_tol = 1e-8;
    opt.max_newton_iters = 16;
    opt.dae_use_analytic_device_jacobian = use_device_jacobian;
    opt.dae_use_fd_current_jacobian_corrections = true;
    return hacdcpf::run_transient_simulation(make_hybrid_dc_case(), opt);
  };

  const DynamicResults fd = run(false);
  const DynamicResults analytic = run(true);
  REQUIRE(fd.success);
  REQUIRE(analytic.success);
  REQUIRE(fd.final_snapshot() != nullptr);
  REQUIRE(analytic.final_snapshot() != nullptr);
  REQUIRE(fd.final_snapshot()->state.size() == analytic.final_snapshot()->state.size());
  REQUIRE(fd.final_snapshot()->vac_abc.size() == analytic.final_snapshot()->vac_abc.size());
  REQUIRE(fd.final_snapshot()->vdc.size() == analytic.final_snapshot()->vdc.size());

  double max_state_diff = 0.0;
  for (Eigen::Index i = 0; i < fd.final_snapshot()->state.size(); ++i) {
    max_state_diff = std::max(max_state_diff,
                              std::abs(fd.final_snapshot()->state[i] -
                                       analytic.final_snapshot()->state[i]));
  }
  double max_ac_diff = 0.0;
  for (Eigen::Index i = 0; i < fd.final_snapshot()->vac_abc.size(); ++i) {
    max_ac_diff = std::max(max_ac_diff,
                           std::abs(fd.final_snapshot()->vac_abc[i] -
                                    analytic.final_snapshot()->vac_abc[i]));
  }
  double max_dc_diff = 0.0;
  for (Eigen::Index i = 0; i < fd.final_snapshot()->vdc.size(); ++i) {
    max_dc_diff = std::max(max_dc_diff,
                           std::abs(fd.final_snapshot()->vdc[i] -
                                    analytic.final_snapshot()->vdc[i]));
  }
  CHECK(max_state_diff < 1e-10);
  CHECK(max_ac_diff < 1e-10);
  CHECK(max_dc_diff < 1e-10);
}

TEST_CASE("MassMatrixDae Jacobian modes preserve trace while colored FD reduces residual calls",
          "[dynamics][dae][jacobian][performance]") {
  auto run = [](DynamicDaeJacobianMode mode) {
    DynamicSolverOptions opt;
    opt.solver_type = DynamicSolverType::MassMatrixDae;
    opt.t_end_s = 0.04;
    opt.dt_s = 0.01;
    opt.newton_tol = 1e-8;
    opt.max_newton_iters = 16;
    opt.algebraic_network_max_iters = 20;
    opt.algebraic_network_tol = 1e-10;
    opt.use_consistent_dynamic_initialization = true;
    opt.dae_jacobian_mode = mode;
    opt.dae_use_analytic_network_jacobian =
        mode != DynamicDaeJacobianMode::FiniteDifference;
    opt.dae_use_analytic_device_jacobian =
        mode != DynamicDaeJacobianMode::FiniteDifference;
    opt.dae_use_fd_current_jacobian_corrections = true;
    auto sys = make_controlled_machine_case(false, false, false);
    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent event;
    event.time_s = 0.01;
    event.type = DynamicEventType::ACLoadScale;
    event.bus = 2;
    event.value = 1.05;
    dyn.events.push_back(event);
    DynamicSolver solver;
    return solver.solve(dyn);
  };

  const DynamicResults fd = run(DynamicDaeJacobianMode::FiniteDifference);
  const DynamicResults hybrid = run(DynamicDaeJacobianMode::HybridAnalytic);
  const DynamicResults colored = run(DynamicDaeJacobianMode::HybridAnalyticColored);
  REQUIRE(fd.success);
  REQUIRE(hybrid.success);
  REQUIRE(colored.success);
  REQUIRE(fd.final_snapshot() != nullptr);
  REQUIRE(hybrid.final_snapshot() != nullptr);
  REQUIRE(colored.final_snapshot() != nullptr);
  REQUIRE(fd.final_snapshot()->state.size() == colored.final_snapshot()->state.size());

  double max_state_diff = 0.0;
  for (Eigen::Index i = 0; i < fd.final_snapshot()->state.size(); ++i) {
    max_state_diff = std::max(max_state_diff,
                              std::abs(fd.final_snapshot()->state[i] -
                                       colored.final_snapshot()->state[i]));
  }
  double max_ac_diff = 0.0;
  for (Eigen::Index i = 0; i < fd.final_snapshot()->vac_abc.size(); ++i) {
    max_ac_diff = std::max(max_ac_diff,
                           std::abs(fd.final_snapshot()->vac_abc[i] -
                                    colored.final_snapshot()->vac_abc[i]));
  }
  double max_dc_diff = 0.0;
  for (Eigen::Index i = 0; i < fd.final_snapshot()->vdc.size(); ++i) {
    max_dc_diff = std::max(max_dc_diff,
                           std::abs(fd.final_snapshot()->vdc[i] -
                                    colored.final_snapshot()->vdc[i]));
  }

  CHECK(max_state_diff < 1e-7);
  CHECK(max_ac_diff < 1e-7);
  CHECK(max_dc_diff < 1e-7);
  CHECK(hybrid.jacobian_residual_evaluations == hybrid.jacobian_fd_columns);
  CHECK(colored.jacobian_colored_groups < colored.jacobian_fd_columns);
  CHECK(colored.jacobian_residual_evaluations < hybrid.jacobian_residual_evaluations);
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
  CHECK(std::abs(vs.y.front()) < 2e-6);
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
  opt.compute_small_signal = true;  // attach the modal screen for export

  const DynamicResults result = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() == 4);
  CHECK(result.snapshots.front().time_s == Catch::Approx(0.0));
  CHECK(result.snapshots.back().time_s == Catch::Approx(0.05));

  const std::string csv = to_csv(result);
  CHECK(csv.find("time_s") != std::string::npos);
  CHECK(csv.find("vac_0_mag_pu") != std::string::npos);
  CHECK(csv.find("VSCGridFollowing_1_p_mw") != std::string::npos);

  // The small-signal screen is exported as a leading '#'-commented block, and the
  // time-series header still follows it as plain CSV.
  REQUIRE(result.modal.computed);
  CHECK(csv.find("# small_signal") != std::string::npos);
  CHECK(csv.find("# mode,eigen_real_1_s") != std::string::npos);
  CHECK(csv.find("\ntime_s") != std::string::npos);
  if (result.modal.success && !result.modal.modes.empty()) {
    // At least one participation entry (label=factor) is present in the block.
    CHECK(csv.find(":s0=") != std::string::npos);
  }
  // Disabling the modal export drops the block but keeps the table.
  DynamicResultExportOptions no_modal;
  no_modal.include_modal = false;
  const std::string csv_plain = to_csv(result, no_modal);
  CHECK(csv_plain.find("# small_signal") == std::string::npos);
  CHECK(csv_plain.find("time_s") != std::string::npos);
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

TEST_CASE("Frequency observability: single-machine COI tracks the rotor speed",
          "[dynamics][frequency][coi]") {
  auto sys = make_psd_test01_omib_case();  // one classical machine + infinite bus
  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent trip;  // trip one of the two parallel lines to excite the rotor
  trip.time_s = 1.0;
  trip.type = DynamicEventType::ACBranchTrip;
  trip.component_index = 1;
  trip.component_type = "AC";
  trip.label = "OMIB line trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() > 100);

  // The only inertial device is the machine, so the system center-of-inertia
  // frequency must equal that machine's reported electrical frequency at every
  // recorded instant, and frequency_hz mirrors the COI (no longer the constant
  // nominal value).
  const auto machine_f =
      device_output_series(result, "SynchronousMachine", 1, "frequency_hz");
  REQUIRE(machine_f.y.size() == result.snapshots.size());
  std::vector<double> coi;
  coi.reserve(result.snapshots.size());
  for (std::size_t i = 0; i < result.snapshots.size(); ++i) {
    const auto& snap = result.snapshots[i];
    coi.push_back(snap.coi_frequency_hz);
    CHECK(snap.frequency_hz == Catch::Approx(snap.coi_frequency_hz).margin(1e-12));
    CHECK(snap.coi_frequency_hz ==
          Catch::Approx(machine_f.y[i]).margin(1e-9));
    REQUIRE(snap.island_frequencies.size() == 1);
    CHECK(snap.island_frequencies.front().has_source);
    CHECK(snap.island_frequencies.front().has_anchor);
  }
  const auto [lo, hi] = std::minmax_element(coi.begin(), coi.end());
  CHECK((*hi - *lo) > 1e-4);  // the disturbance actually moved the frequency

  CHECK(std::none_of(result.warnings.begin(), result.warnings.end(),
                     [](const std::string& w) {
                       return w.find("no frequency anchor") != std::string::npos;
                     }));
}

TEST_CASE("Frequency observability: two-machine COI is inertia-weighted",
          "[dynamics][frequency][coi]") {
  // Two machines with distinct inertia on a tie line carrying real power, so a
  // load step drives a differential inter-machine swing (the low-inertia unit
  // moves more). The center-of-inertia frequency must be the inertia-weighted
  // average of the two rotor frequencies.
  const double kH0 = 8.0;
  const double kH1 = 2.0;

  HybridPowerSystem sys;
  sys.name = "two_machine_coi";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  auto make_bus = [](int index, BusType type) {
    ACBus b;
    b.index = index;
    b.bus_type = type;
    b.base_kv = 138.0;
    b.vm_pu = 1.0;
    b.va_deg = 0.0;
    b.in_service = true;
    return b;
  };
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PV)};

  ACBranch tie;
  tie.index = 1;
  tie.from_bus = 1;
  tie.to_bus = 2;
  tie.r_pu = 0.005;
  tie.x_pu = 0.05;
  tie.b_pu = 0.0;
  tie.tap = 1.0;
  tie.in_service = true;
  sys.ac.branches = {tie};

  auto make_machine = [](int index, int bus, double H, double pg, bool slack) {
    Generator g;
    g.index = index;
    g.bus = bus;
    g.is_slack = slack;
    g.in_service = true;
    g.pg_mw = pg;
    g.qg_mvar = 0.0;
    g.vg_pu = 1.0;
    g.pmax_mw = 300.0;
    g.pmin_mw = 0.0;
    g.qmax_mvar = 150.0;
    g.qmin_mvar = -150.0;
    g.ra_pu = 0.0;
    g.xd_pu = 0.2;
    g.xq_pu = 0.2;
    g.xdp_pu = 0.2;
    g.xdpp_pu = 0.2;
    g.td0p_s = 0.0;
    g.td0pp_s = 0.0;
    g.inertia_h = H;
    g.droop_r = 0.05;
    g.dynamic_model.standard = "PowerSystems";
    g.dynamic_model.model_name = "ClassicalMachine";
    g.dynamic_model.parameters = {{"H", H}, {"D", 2.0}, {"R", 0.0}, {"Xd_p", 0.2}};
    return g;
  };
  // Bus 1 exports across the tie to a load at bus 2.
  sys.ac.generators = {make_machine(1, 1, kH0, 40.0, true),
                       make_machine(2, 2, kH1, 20.0, false)};

  Load ld;
  ld.index = 1;
  ld.bus = 2;
  ld.p_mw = 60.0;
  ld.q_mvar = 10.0;
  ld.in_service = true;
  sys.ac.loads = {ld};

  DynamicSolverOptions opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 3.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  DynamicEvent load_step;  // +50% load at bus 2 -> sustained imbalance & swing
  load_step.time_s = 1.0;
  load_step.type = DynamicEventType::ACLoadScale;
  load_step.bus = 2;
  load_step.value = 1.5;
  load_step.component_type = "AC";
  load_step.label = "bus-2 load step";
  dyn.events.push_back(load_step);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() > 100);

  const auto f0 = device_output_series(result, "SynchronousMachine", 1, "frequency_hz");
  const auto f1 = device_output_series(result, "SynchronousMachine", 2, "frequency_hz");
  REQUIRE(f0.y.size() == result.snapshots.size());
  REQUIRE(f1.y.size() == result.snapshots.size());

  double max_diff = 0.0;
  double max_spread = 0.0;
  for (std::size_t i = 0; i < result.snapshots.size(); ++i) {
    const double expected = (kH0 * f0.y[i] + kH1 * f1.y[i]) / (kH0 + kH1);
    max_diff = std::max(
        max_diff, std::abs(result.snapshots[i].coi_frequency_hz - expected));
    max_spread = std::max(max_spread, std::abs(f0.y[i] - f1.y[i]));
    // Both machines remain in one connected island (the tie stays in service).
    REQUIRE(result.snapshots[i].island_frequencies.size() == 1);
    CHECK(result.snapshots[i].island_frequencies.front().total_inertia_mws ==
          Catch::Approx((kH0 + kH1) * 100.0).epsilon(1e-6));
  }
  CHECK(max_diff < 1e-4);    // COI equals the inertia-weighted average
  CHECK(max_spread > 1e-4);  // the machines diverged, so weighting mattered

  // Measured per-bus frequency (design doc §7 role 4) is populated, stays within
  // a sane band, and actually responds to the disturbance.
  double meas_min = std::numeric_limits<double>::infinity();
  double meas_max = -std::numeric_limits<double>::infinity();
  for (const auto& snap : result.snapshots) {
    REQUIRE(snap.bus_frequency_hz.size() == sys.ac.buses.size());
    for (const double f : snap.bus_frequency_hz) {
      REQUIRE(std::isfinite(f));
      meas_min = std::min(meas_min, f);
      meas_max = std::max(meas_max, f);
    }
  }
  CHECK(meas_min > 45.0);
  CHECK(meas_max < 55.0);
  CHECK((meas_max - meas_min) > 1e-3);  // the measured signal tracked the event
}

TEST_CASE("Frequency observability: island detection reacts to a tie-line trip",
          "[dynamics][frequency][island]") {
  // Two self-balanced buses (generation == local load on each) joined by a
  // single tie line. Tripping the tie must split the system into two separately
  // anchored frequency islands.
  HybridPowerSystem sys;
  sys.name = "two_machine_tie";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  auto make_bus = [](int index, BusType type) {
    ACBus b;
    b.index = index;
    b.bus_type = type;
    b.base_kv = 138.0;
    b.vm_pu = 1.0;
    b.va_deg = 0.0;
    b.in_service = true;
    return b;
  };
  sys.ac.buses = {make_bus(1, BusType::SLACK), make_bus(2, BusType::PV)};

  ACBranch tie;
  tie.index = 1;
  tie.from_bus = 1;
  tie.to_bus = 2;
  tie.r_pu = 0.01;
  tie.x_pu = 0.05;
  tie.b_pu = 0.0;
  tie.tap = 1.0;
  tie.in_service = true;
  sys.ac.branches = {tie};

  auto make_machine = [](int index, int bus, double H, bool slack) {
    Generator g;
    g.index = index;
    g.bus = bus;
    g.is_slack = slack;
    g.in_service = true;
    g.pg_mw = 30.0;
    g.qg_mvar = 0.0;
    g.vg_pu = 1.0;
    g.pmax_mw = 200.0;
    g.pmin_mw = 0.0;
    g.qmax_mvar = 100.0;
    g.qmin_mvar = -100.0;
    g.ra_pu = 0.0;
    g.xd_pu = 0.2;
    g.xq_pu = 0.2;
    g.xdp_pu = 0.2;
    g.xdpp_pu = 0.2;
    g.td0p_s = 0.0;
    g.td0pp_s = 0.0;
    g.inertia_h = H;
    g.droop_r = 0.05;
    g.dynamic_model.standard = "PowerSystems";
    g.dynamic_model.model_name = "ClassicalMachine";
    g.dynamic_model.parameters = {{"H", H}, {"D", 2.0}, {"R", 0.0}, {"Xd_p", 0.2}};
    return g;
  };
  sys.ac.generators = {make_machine(1, 1, 5.0, true),
                       make_machine(2, 2, 3.0, false)};

  auto make_load = [](int index, int bus) {
    Load l;
    l.index = index;
    l.bus = bus;
    l.p_mw = 30.0;
    l.q_mvar = 5.0;
    l.in_service = true;
    return l;
  };
  sys.ac.loads = {make_load(1, 1), make_load(2, 2)};

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
  trip.label = "tie-line trip";
  dyn.events.push_back(trip);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);
  REQUIRE(result.success);
  REQUIRE(result.snapshots.size() > 100);

  // Before the trip: a single energized island spanning both buses.
  const auto& first = result.snapshots.front();
  REQUIRE(first.island_frequencies.size() == 1);
  CHECK(first.island_frequencies.front().n_ac_buses == 2);
  CHECK(first.island_frequencies.front().has_anchor);

  // After the trip: two separately anchored single-bus islands.
  const auto& last = result.snapshots.back();
  REQUIRE(last.island_frequencies.size() == 2);
  for (const auto& island : last.island_frequencies) {
    CHECK(island.n_ac_buses == 1);
    CHECK(island.has_source);
    CHECK(island.has_anchor);
    CHECK(island.coi_frequency_hz > 45.0);
    CHECK(island.coi_frequency_hz < 55.0);
  }

  // Both islands keep an anchor, so no anchorless-island warning is raised.
  CHECK(std::none_of(result.warnings.begin(), result.warnings.end(),
                     [](const std::string& w) {
                       return w.find("no frequency anchor") != std::string::npos;
                     }));
}

TEST_CASE("Unbalanced machine sequence interface: SLG fault braking torque",
          "[dynamics][unbalanced][sequence]") {
  // Design doc §8.8: a machine with sequence parameters presents Y0/Y1/Y2 to the
  // unbalanced network and feels a negative-sequence braking torque. A
  // single-line-to-ground (SLG) fault creates negative-sequence voltage; the
  // braking torque then appears only when the sequence path is modeled.
  auto make_case = [](bool sequence_enabled) {
    HybridPowerSystem sys;
    sys.name = "onedoneq_seq_omib";
    sys.base_mva = 100.0;
    sys.ac.base_mva = 100.0;
    sys.ac.freq_hz = 60.0;

    auto make_bus = [](int index, BusType type, double vm) {
      ACBus b;
      b.index = index;
      b.bus_type = type;
      b.base_kv = 230.0;
      b.vm_pu = vm;
      b.in_service = true;
      return b;
    };
    sys.ac.buses = {make_bus(101, BusType::SLACK, 1.05),
                    make_bus(102, BusType::PV, 1.02)};

    auto branch = [](int index) {
      ACBranch br;
      br.index = index;
      br.from_bus = 101;
      br.to_bus = 102;
      br.r_pu = 0.0;
      br.x_pu = 0.1;
      br.tap = 1.0;
      br.in_service = true;
      return br;
    };
    sys.ac.branches = {branch(1), branch(2)};

    ExternalGrid src;
    src.index = 1;
    src.bus = 101;
    src.vm_pu = 1.05;
    src.x_pu = 1.0e-5;
    src.in_service = true;
    sys.ac.external_grids = {src};

    Generator gen;
    gen.index = 1;
    gen.bus = 102;
    gen.is_slack = false;
    gen.in_service = true;
    gen.pg_mw = 40.0;
    gen.vg_pu = 1.02;
    gen.pmax_mw = 100.0;
    gen.qmax_mvar = 100.0;
    gen.qmin_mvar = -100.0;
    gen.ra_pu = 0.0;
    gen.xd_pu = 1.3;
    gen.xq_pu = 1.25;
    gen.xdp_pu = 0.18;
    gen.inertia_h = 4.0;
    gen.dynamic_model.standard = "PowerSystems";
    gen.dynamic_model.model_name = "OneDOneQMachine";
    gen.dynamic_model.parameters = {
        {"H", 4.0}, {"D", 2.0},    {"R", 0.0},      {"Xd", 1.3},
        {"Xq", 1.25}, {"Xd_p", 0.18}, {"Xq_p", 0.25}, {"Td0_p", 5.9},
        {"Tq0_p", 0.6}};
    if (sequence_enabled) {
      gen.dynamic_model.parameters["X2"] = 0.20;
      gen.dynamic_model.parameters["R2"] = 0.05;
      gen.dynamic_model.parameters["X0"] = 0.10;
    }
    sys.ac.generators = {gen};
    return sys;
  };

  auto run = [&](bool sequence_enabled) {
    auto sys = make_case(sequence_enabled);
    DynamicSolverOptions opt = fast_options();
    opt.run_power_flow_initialization = true;
    // Exercise the machine models directly: the canonical projection currently
    // re-synthesizes the network in a way that suppresses the terminal
    // negative-sequence voltage for this fixture, so the sequence interface is
    // validated on the un-projected system (device behavior is identical).
    opt.project_to_canonical = false;
    opt.t_end_s = 2.0;
    opt.dt_s = 0.005;
    opt.record_every_step = true;
    DynamicModelBuilder builder;
    DynamicSystem dyn = builder.build(sys, opt);
    DynamicEvent fault;
    fault.time_s = 1.0;
    fault.type = DynamicEventType::FaultShunt;
    fault.bus = 102;
    fault.phase = 0;  // single-line-to-ground on phase A
    fault.component_type = "AC";
    fault.params["x_pu"] = 0.02;
    fault.params["duration_s"] = 0.1;
    dyn.events.push_back(fault);
    DynamicSolver solver;
    return solver.solve(dyn);
  };

  const DynamicResults with_seq = run(true);
  const DynamicResults no_seq = run(false);
  INFO(with_seq.message);
  REQUIRE(with_seq.success);
  REQUIRE(no_seq.success);

  const auto vneg =
      device_output_series(with_seq, "SynchronousMachine", 1, "v_neg_seq_pu");
  const auto brake =
      device_output_series(with_seq, "SynchronousMachine", 1, "tau_brake_pu");
  const auto brake_off =
      device_output_series(no_seq, "SynchronousMachine", 1, "tau_brake_pu");

  // The single-phase fault creates negative-sequence voltage at the terminal.
  CHECK(*std::max_element(vneg.y.begin(), vneg.y.end()) > 0.01);
  // Braking torque is active only when the sequence path is modeled.
  CHECK(*std::max_element(brake.y.begin(), brake.y.end()) > 1e-3);
  CHECK(*std::max_element(brake_off.y.begin(), brake_off.y.end()) < 1e-9);

  // The braking torque perturbs the rotor speed relative to the sequence-free run.
  const auto w_seq =
      device_output_series(with_seq, "SynchronousMachine", 1, "omega_pu");
  const auto w_off =
      device_output_series(no_seq, "SynchronousMachine", 1, "omega_pu");
  REQUIRE(w_seq.y.size() == w_off.y.size());
  double max_dw = 0.0;
  for (std::size_t i = 0; i < w_seq.y.size(); ++i) {
    max_dw = std::max(max_dw, std::abs(w_seq.y[i] - w_off.y[i]));
  }
  CHECK(max_dw > 1e-4);
}

TEST_CASE("IEEE 1547 DER protection: undervoltage trip and ramped reconnect",
          "[dynamics][protection][ieee1547]") {
  // Design doc §11.7: a DER carries an IEEE 1547 ride-through protection block
  // evaluated on measured (filtered) terminal quantities (§7 role 4). A
  // sustained deep voltage sag past the must-trip clearing time trips the
  // inverter; once the terminal recovers into the continuous band for the
  // reconnect delay the inverter re-enters service and ramps its power back over
  // the soft-start window.
  GridFollowingInverterParams params;
  params.component_index = 7;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.p_ref_mw = 50.0;
  params.protection = make_default_ieee1547(IEEE1547Category::CategoryII, 60.0);
  params.protection.enabled = true;
  params.protection.reconnect_delay_s = 0.1;
  params.protection.power_ramp_s = 0.1;
  GridFollowingInverter inverter(params);

  int offset = 0;
  inverter.assignStateIndices(offset);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);

  const double dt = 0.01;
  double t = 0.0;
  std::vector<DynamicEvent> events;
  auto step_at = [&](double vmag) {
    set_balanced_voltage(y, 0, std::polar(vmag, 0.0));
    const bool rebuild = inverter.updateProtection(t, dt, x, y, events);
    t += dt;
    return rebuild;
  };

  // Healthy operation stays in service and never asks for a network rebuild.
  for (int i = 0; i < 10; ++i) CHECK_FALSE(step_at(1.0));
  {
    const auto out = inverter.output(x, y);
    CHECK(out.values.at("in_service") > 0.5);
    CHECK(out.values.at("protection_tripped") < 0.5);
    CHECK(out.values.at("protection_restore_scale") == Catch::Approx(1.0));
  }

  // A deep sag (0.40 pu, below the 0.50 pu / 0.16 s must-trip band) rides
  // through until the clearing time (~16 steps at dt = 0.01 s), then trips.
  bool tripped = false;
  int trip_steps = 0;
  for (; trip_steps < 40 && !tripped; ++trip_steps) tripped = step_at(0.4);
  REQUIRE(tripped);
  CHECK(trip_steps >= 14);  // rode through, did not trip instantly
  CHECK(trip_steps <= 20);
  {
    const auto out = inverter.output(x, y);
    CHECK(out.values.at("in_service") < 0.5);
    CHECK(out.values.at("protection_tripped") > 0.5);
    CHECK(out.values.at("protection_restore_scale") == Catch::Approx(0.0));
  }
  REQUIRE_FALSE(events.empty());
  CHECK(events.back().type == DynamicEventType::VSCTrip);
  CHECK(events.back().label.find("undervoltage") != std::string::npos);
  const std::size_t events_after_trip = events.size();

  // Recovery into the continuous band reconnects after the reconnect delay.
  bool reconnected = false;
  for (int i = 0; i < 40 && !reconnected; ++i) reconnected = step_at(1.0);
  REQUIRE(reconnected);
  {
    const auto out = inverter.output(x, y);
    CHECK(out.values.at("in_service") > 0.5);
    CHECK(out.values.at("protection_tripped") < 0.5);
    CHECK(out.values.at("protection_restore_scale") < 0.5);  // ramp just started
  }
  REQUIRE(events.size() > events_after_trip);
  CHECK(events.back().label.find("reconnect") != std::string::npos);

  // The soft-start ramp restores the injection scale to full over the window.
  for (int i = 0; i < 20; ++i) step_at(1.0);
  CHECK(inverter.output(x, y).values.at("protection_restore_scale") ==
        Catch::Approx(1.0));
}

TEST_CASE("IEEE 1547 protection hook is a live no-op without opted-in devices",
          "[dynamics][protection][ieee1547]") {
  // The solver-level enable flag alone must not change results: a device acts
  // only when its own per-device protection is enabled. Running with the hook on
  // exercises the per-step protection evaluation live across a full multi-device
  // transient and must reproduce the baseline exactly.
  const auto sys = make_hybrid_dc_case();
  auto opt = fast_options();
  opt.t_end_s = 0.05;
  opt.dt_s = 0.01;

  const auto baseline = hacdcpf::run_transient_simulation(sys, opt);
  opt.enable_der_protection = true;
  const auto with_hook = hacdcpf::run_transient_simulation(sys, opt);

  REQUIRE(baseline.success);
  REQUIRE(with_hook.success);
  REQUIRE(baseline.final_snapshot() != nullptr);
  REQUIRE(with_hook.final_snapshot() != nullptr);
  CHECK(with_hook.final_snapshot()->min_ac_voltage_pu ==
        Catch::Approx(baseline.final_snapshot()->min_ac_voltage_pu));
  CHECK(with_hook.final_snapshot()->max_ac_voltage_pu ==
        Catch::Approx(baseline.final_snapshot()->max_ac_voltage_pu));
  for (const auto& label : with_hook.applied_events) {
    CHECK(label.find("IEEE1547") == std::string::npos);
  }
}

TEST_CASE("IEEE 1547 protection trips grid-forming, VSC, and synchronous DERs",
          "[dynamics][protection][ieee1547]") {
  // Design doc §11.7 extended beyond the grid-following reference: the same
  // ride-through block trips a grid-forming inverter, a VSC converter, and a
  // synchronous DER on a sustained deep undervoltage, each emitting the correct
  // trip event type through the shared protection helper.
  IEEE1547Settings prot = make_default_ieee1547(IEEE1547Category::CategoryII, 60.0);
  prot.enabled = true;

  auto drive_trip = [](DynamicDevice& dev, int n_states,
                       std::vector<DynamicEvent>& events) {
    DynamicState x;
    x.resize(static_cast<std::size_t>(std::max(1, n_states)));
    NetworkState y;
    y.resize(3, 1);
    double t = 0.0;
    const double dt = 0.01;
    bool tripped = false;
    for (int i = 0; i < 60 && !tripped; ++i) {
      set_balanced_voltage(y, 0, std::polar(0.30, 0.0));  // deep sag
      tripped = dev.updateProtection(t, dt, x, y, events);
      t += dt;
    }
    return tripped;
  };

  SECTION("grid-forming inverter") {
    GridFormingInverterParams params;
    params.component_index = 11;
    params.bus = 1;
    params.bus_pos = 0;
    params.base_mva = 100.0;
    params.protection = prot;
    GridFormingInverter gfm(params);
    int offset = 0;
    gfm.assignStateIndices(offset);
    std::vector<DynamicEvent> events;
    REQUIRE(drive_trip(gfm, offset, events));
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().type == DynamicEventType::VSCTrip);
    CHECK(events.back().label.find("trip") != std::string::npos);
  }

  SECTION("VSC converter (grid-following)") {
    VSCConverterDynamicParams params;
    params.component_index = 12;
    params.bus = 1;
    params.bus_pos = 0;
    params.base_mva = 100.0;
    params.grid_forming = false;
    params.protection = prot;
    VSCConverterDynamic vsc(params);
    int offset = 0;
    vsc.assignStateIndices(offset);
    std::vector<DynamicEvent> events;
    REQUIRE(drive_trip(vsc, offset, events));
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().type == DynamicEventType::VSCTrip);
  }

  SECTION("synchronous machine") {
    VoltageSourceDynamicParams params;
    params.component_index = 13;
    params.bus = 1;
    params.bus_pos = 0;
    params.base_mva = 100.0;
    params.protection = prot;
    SynchronousMachine machine(params);
    int offset = 0;
    machine.assignStateIndices(offset);
    std::vector<DynamicEvent> events;
    REQUIRE(drive_trip(machine, offset, events));
    REQUIRE_FALSE(events.empty());
    CHECK(events.back().type == DynamicEventType::GeneratorTrip);
  }
}

TEST_CASE("IEEE 1547 volt-var and frequency-watt smart-inverter curves",
          "[dynamics][protection][ieee1547]") {
  // Design doc §11.7: piecewise-linear volt-var Q(V) and frequency-watt P(f)
  // characteristics with deadbands, superseding the simple linear droop.
  SECTION("volt-var Q(V) piecewise curve with deadband") {
    VoltVarSettings vv;  // defaults: V1=0.92 Q1=+0.44, deadband [0.98,1.02],
    vv.enabled = true;   //           V4=1.08 Q4=-0.44
    CHECK(volt_var_q_pu(vv, 1.00) == Catch::Approx(0.0));    // inside deadband
    CHECK(volt_var_q_pu(vv, 0.98) == Catch::Approx(0.0));    // lower edge
    CHECK(volt_var_q_pu(vv, 1.02) == Catch::Approx(0.0));    // upper edge
    CHECK(volt_var_q_pu(vv, 0.90) == Catch::Approx(0.44));   // clamp: full inject
    CHECK(volt_var_q_pu(vv, 1.10) == Catch::Approx(-0.44));  // clamp: full absorb
    CHECK(volt_var_q_pu(vv, 0.95) == Catch::Approx(0.22));   // linear midpoint
    CHECK(volt_var_q_pu(vv, 1.05) < 0.0);                    // partial absorb
    VoltVarSettings off;                                     // disabled
    CHECK(volt_var_q_pu(off, 0.90) == Catch::Approx(0.0));
  }

  SECTION("frequency-watt P(f) droop with deadband") {
    FreqWattSettings fw;
    fw.enabled = true;
    fw.nominal_frequency_hz = 60.0;  // deadband +/-0.036 Hz, droop 0.05
    CHECK(freq_watt_delta_pu(fw, 60.00) == Catch::Approx(0.0));  // nominal
    CHECK(freq_watt_delta_pu(fw, 60.03) == Catch::Approx(0.0));  // within deadband
    CHECK(freq_watt_delta_pu(fw, 60.50) < 0.0);                  // over-freq: curtail
    CHECK(freq_watt_delta_pu(fw, 59.50) > 0.0);                  // under-freq: raise
    CHECK(freq_watt_delta_pu(fw, 60.50) ==
          Catch::Approx(-(0.50 - 0.036) / 60.0 / 0.05));
    FreqWattSettings off;
    CHECK(freq_watt_delta_pu(off, 60.50) == Catch::Approx(0.0));
  }

  SECTION("grid-following inverter reports volt-var injection and absorption") {
    GridFollowingInverterParams params;
    params.component_index = 21;
    params.bus = 1;
    params.bus_pos = 0;
    params.base_mva = 100.0;
    params.p_ref_mw = 40.0;
    params.volt_var.enabled = true;
    GridFollowingInverter gfl(params);
    int offset = 0;
    gfl.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    NetworkState y;
    y.resize(3, 1);

    set_balanced_voltage(y, 0, std::polar(0.90, 0.0));  // below deadband: inject
    CHECK(gfl.output(x, y).values.at("volt_var_q_pu") > 0.0);
    set_balanced_voltage(y, 0, std::polar(1.05, 0.0));  // above deadband: absorb
    CHECK(gfl.output(x, y).values.at("volt_var_q_pu") < 0.0);
    set_balanced_voltage(y, 0, std::polar(1.00, 0.0));  // deadband: no vars
    CHECK(gfl.output(x, y).values.at("volt_var_q_pu") == Catch::Approx(0.0));
  }
}

TEST_CASE("IEEE 1547 protection trips and reconnects a DER through the solver",
          "[dynamics][protection][ieee1547][fault]") {
  // End-to-end (design doc §11.7 + §17): a grid-following DER with IEEE 1547
  // protection sits at a bus that suffers a deep, sustained voltage sag. The
  // solver's per-step protection hook trips the DER once the clearing time is
  // exceeded, then reconnects it after the terminal recovers into the continuous
  // band for the reconnect delay.
  HybridPowerSystem sys = make_transient_2bus();  // slack machine @bus1, load @bus2
  StaticGenerator der;
  der.index = 1;
  der.bus = 2;
  der.in_service = true;
  der.p_mw = 15.0;
  der.q_mvar = 0.0;
  der.sn_mva = 40.0;
  der.v_ref_pu = 1.0;
  der.f_ref_hz = 50.0;
  der.dynamic_model.standard = "IEEE";
  der.dynamic_model.model_name = "GridFollowingInverter";
  der.dynamic_model.parameters = {
      {"ieee1547_enabled", 1.0},
      {"ieee1547_category", 2.0},
      {"ieee1547_reconnect_delay_s", 0.2},
      {"ieee1547_power_ramp_s", 0.1},
  };
  sys.ac.static_generators.push_back(der);

  auto opt = fast_options();
  opt.run_power_flow_initialization = true;
  opt.enable_der_protection = true;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.01;
  opt.record_every_step = true;
  opt.enforce_voltage_health_check = false;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  DynamicEvent fault;
  fault.time_s = 0.20;
  fault.type = DynamicEventType::FaultShunt;
  fault.bus = 2;
  fault.component_type = "AC";
  fault.label = "deep AC fault at DER bus";
  fault.params["r_pu"] = 0.01;
  fault.params["x_pu"] = 0.01;
  fault.params["duration_s"] = 0.30;
  dyn.events.push_back(fault);

  DynamicSolver solver;
  const DynamicResults result = solver.solve(dyn);

  INFO(result.message);
  REQUIRE(result.success);

  bool tripped = false;
  bool reconnected = false;
  for (const auto& label : result.applied_events) {
    if (label.find("IEEE1547") != std::string::npos &&
        label.find("trip") != std::string::npos) {
      tripped = true;
    }
    if (label.find("IEEE1547 reconnect") != std::string::npos) {
      reconnected = true;
    }
  }
  CHECK(tripped);
  CHECK(reconnected);
}

TEST_CASE("IEEE 1547 smart-inverter filter and slew-rate limiter",
          "[dynamics][protection][ieee1547]") {
  // Design doc §11.7: the volt-var / frequency-watt references pass through a
  // T_qf/T_pf low-pass and a slew-rate limiter (a stateful smart-inverter block).
  SECTION("volt-var low-pass tracks toward the curve target") {
    VoltVarSettings vv;
    vv.enabled = true;
    vv.filter_t_s = 0.5;
    vv.ramp_rate_pu_per_s = 0.0;  // unlimited slew: pure low-pass
    FreqWattSettings fw;          // disabled
    SmartInverterState st;
    const double dt = 0.01;

    step_smart_inverter(vv, fw, st, 1.00, 0.0, dt);  // seed in deadband -> 0
    REQUIRE(st.initialized);
    CHECK(st.q_pu == Catch::Approx(0.0));

    const double target = volt_var_q_pu(vv, 0.90);  // 0.44 (clamped)
    for (int i = 0; i < 5; ++i) step_smart_inverter(vv, fw, st, 0.90, 0.0, dt);
    CHECK(st.q_pu > 0.0);     // rising
    CHECK(st.q_pu < target);  // but not yet at target (T_qf = 0.5 s)
    for (int i = 0; i < 1000; ++i) step_smart_inverter(vv, fw, st, 0.90, 0.0, dt);
    CHECK(st.q_pu == Catch::Approx(target).margin(1e-3));  // converged
  }

  SECTION("slew-rate limit caps the reference rate of change") {
    VoltVarSettings vv;
    vv.enabled = true;
    vv.filter_t_s = 0.0;           // no low-pass: isolate the slew limiter
    vv.ramp_rate_pu_per_s = 0.10;  // 0.10 pu/s
    FreqWattSettings fw;
    SmartInverterState st;
    const double dt = 0.01;  // max change per step = 0.001 pu

    step_smart_inverter(vv, fw, st, 1.00, 0.0, dt);  // seed at 0
    step_smart_inverter(vv, fw, st, 0.90, 0.0, dt);
    CHECK(st.q_pu == Catch::Approx(0.001).margin(1e-9));
    step_smart_inverter(vv, fw, st, 0.90, 0.0, dt);
    CHECK(st.q_pu == Catch::Approx(0.002).margin(1e-9));
  }

  SECTION("grid-following inverter smart controls update through the hook") {
    GridFollowingInverterParams params;
    params.component_index = 31;
    params.bus = 1;
    params.bus_pos = 0;
    params.base_mva = 100.0;
    params.volt_var.enabled = true;
    params.volt_var.filter_t_s = 0.2;
    GridFollowingInverter gfl(params);
    int offset = 0;
    gfl.assignStateIndices(offset);
    DynamicState x;
    x.resize(static_cast<std::size_t>(offset));
    NetworkState y;
    y.resize(3, 1);

    set_balanced_voltage(y, 0, std::polar(0.90, 0.0));  // below deadband
    const double dt = 0.01;
    for (int i = 0; i < 200; ++i) gfl.updateSmartControls(dt, y);
    // The filtered reactive command converges to the volt-var curve target.
    CHECK(gfl.output(x, y).values.at("smart_var_q_pu") > 0.0);
    CHECK(gfl.output(x, y).values.at("smart_var_q_pu") ==
          Catch::Approx(volt_var_q_pu(params.volt_var, 0.90)).margin(1e-2));
  }
}

TEST_CASE("WECC REGC_A renewable converter: LVACM, HVRCM, current limiting",
          "[dynamics][renewable][regca]") {
  // Design doc §11.6: the WECC generic renewable converter presents current-
  // command lags, a terminal-voltage filter, low-voltage active-current
  // management (LVACM) and high-voltage reactive-current management (HVRCM).
  REGCADynamicParams params;
  params.component_index = 41;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.model_base_mva = 100.0;
  params.p_ref_mw = 80.0;
  params.q_ref_mvar = 0.0;
  params.i_max_pu = 1.2;
  REGCADynamic regca(params);
  int offset = 0;
  regca.assignStateIndices(offset);
  REQUIRE(offset == 3);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);

  SECTION("nominal voltage injects the scheduled active power") {
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    const auto out = regca.output(x, y);
    CHECK(out.values.at("lvacm_gain") == Catch::Approx(1.0));
    CHECK(out.values.at("hvrcm_active") == Catch::Approx(0.0));
    CHECK(out.values.at("p_mw") == Catch::Approx(80.0).margin(1.0));
  }

  SECTION("LVACM ramps active current down under a sag") {
    set_balanced_voltage(y, 0, std::polar(0.5, 0.0));
    // Gain is linear between v_lvacm0=0.4 and v_lvacm1=0.8: (0.5-0.4)/0.4 = 0.25.
    CHECK(regca.output(x, y).values.at("lvacm_gain") ==
          Catch::Approx(0.25).margin(1e-6));
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);  // Ip at full command
    set_balanced_voltage(y, 0, std::polar(0.5, 0.0));
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    regca.computeDerivatives(0.0, x, y, dxdt);
    CHECK(dxdt[0] < 0.0);  // active-current command pulled down by LVACM
  }

  SECTION("HVRCM absorbs reactive current under a swell") {
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);  // Iq ~ 0
    set_balanced_voltage(y, 0, std::polar(1.30, 0.0));  // above v_hvrcm = 1.2
    CHECK(regca.output(x, y).values.at("hvrcm_active") == Catch::Approx(1.0));
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    regca.computeDerivatives(0.0, x, y, dxdt);
    CHECK(dxdt[1] < 0.0);  // reactive current driven negative (absorb)
  }

  SECTION("current-magnitude limit caps the injection") {
    REGCADynamicParams p2 = params;
    p2.p_ref_mw = 150.0;  // 1.5 pu at V = 1
    p2.q_ref_mvar = 80.0;  // 0.8 pu -> |I| = 1.7 > i_max
    REGCADynamic r2(p2);
    int off2 = 0;
    r2.assignStateIndices(off2);
    DynamicState x2;
    x2.resize(static_cast<std::size_t>(off2));
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    r2.initializeFromPowerFlow(PowerFlowResult{}, x2, y);
    const double ip0 = x2.x[0];
    const double iq0 = x2.x[1];
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(off2);
    r2.computeDerivatives(0.0, x2, y, dxdt);
    const double ip_lim = ip0 + dxdt[0] * p2.t_g_s;
    const double iq_lim = iq0 + dxdt[1] * p2.t_g_s;
    CHECK(std::hypot(ip_lim, iq_lim) <= p2.i_max_pu + 1e-6);
  }
}

TEST_CASE("WECC REEC_A electrical control: Q-V droop and active-power order",
          "[dynamics][renewable][reeca]") {
  // Design doc §11.6: REEC_A is the inner electrical control feeding REGC_A. It
  // adds a ramp-limited active-power order and a reactive command that combines a
  // deadband Q-V droop (dynamic voltage support) with the reactive schedule.
  REGCADynamicParams params;
  params.component_index = 42;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.model_base_mva = 100.0;
  params.p_ref_mw = 60.0;
  params.q_ref_mvar = 0.0;
  params.v_ref_pu = 1.0;
  params.reec.enabled = true;
  params.reec.dbd1 = -0.05;
  params.reec.dbd2 = 0.05;
  params.reec.kqv = 2.0;
  params.reec.tpord_s = 0.05;
  REGCADynamic regca(params);
  int offset = 0;
  regca.assignStateIndices(offset);
  REQUIRE(offset == 4);  // Ip, Iq, Vflt, Pord
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);

  SECTION("Q-V droop injects reactive current under a voltage dip") {
    set_balanced_voltage(y, 0, std::polar(0.80, 0.0));  // 0.20 pu error > deadband
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    CHECK(regca.output(x, y).values.at("reec_enabled") == Catch::Approx(1.0));
    // Iqinj = Kqv * ((Vref - Vt) - dbd2) = 2.0 * (0.20 - 0.05) = 0.30.
    CHECK(regca.output(x, y).values.at("Iqinj") ==
          Catch::Approx(0.30).margin(1e-3));
  }

  SECTION("no reactive injection inside the deadband") {
    set_balanced_voltage(y, 0, std::polar(0.98, 0.0));  // 0.02 pu error < deadband
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    CHECK(regca.output(x, y).values.at("Iqinj") == Catch::Approx(0.0));
  }

  SECTION("active-power order ramps toward the schedule") {
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);  // Pord init = 0.6
    x.x[3] = 0.3;  // perturb below the schedule
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    regca.computeDerivatives(0.0, x, y, dxdt);
    CHECK(dxdt[3] > 0.0);  // dPord/dt drives toward Pref = 0.6
  }
}

TEST_CASE("WECC REPC_A plant controller: deadband voltage-reactive PI",
          "[dynamics][renewable][repca]") {
  // Design doc §11.6: REPC_A is the outer plant loop. This single-unit plant
  // regulates the filtered terminal voltage with a deadband PI, producing the
  // reactive command Qext that feeds the REEC_A reference.
  REGCADynamicParams params;
  params.component_index = 43;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.model_base_mva = 100.0;
  params.p_ref_mw = 50.0;
  params.q_ref_mvar = 0.0;
  params.v_ref_pu = 1.0;
  params.reec.enabled = true;
  params.repc.enabled = true;
  params.repc.vref_pu = 1.0;
  params.repc.dbd1 = -0.01;
  params.repc.dbd2 = 0.01;
  params.repc.kp = 1.0;
  params.repc.ki = 5.0;
  REGCADynamic regca(params);
  int offset = 0;
  regca.assignStateIndices(offset);
  REQUIRE(offset == 5);  // Ip, Iq, Vflt, Pord, Qpi
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);

  SECTION("plant PI integrates up to inject vars under a low voltage") {
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);  // Qpi init = 0
    x.x[2] = 0.95;  // hold the filtered terminal voltage below Vref
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    regca.computeDerivatives(0.0, x, y, dxdt);
    CHECK(dxdt[4] > 0.0);  // dQpi/dt > 0 (raise reactive output)
    CHECK(regca.output(x, y).values.at("repc_enabled") == Catch::Approx(1.0));
    CHECK(regca.output(x, y).values.at("Qext") > 0.0);
  }

  SECTION("no plant action inside the voltage deadband") {
    set_balanced_voltage(y, 0, std::polar(1.005, 0.0));
    regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    x.x[2] = 1.005;  // within +/- 0.01 pu deadband of Vref = 1.0
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    regca.computeDerivatives(0.0, x, y, dxdt);
    CHECK(dxdt[4] == Catch::Approx(0.0));  // integrator frozen (no error)
  }
}

TEST_CASE("Active constant-power load: dynamic negative incremental resistance",
          "[dynamics][loads][cpl]") {
  // Design doc §13: a rectifier-behind-controls CPL. The constant-power stage
  // draws more current as the voltage falls (negative incremental resistance),
  // captured dynamically through the input filter rather than as a static
  // injection.
  ActiveConstantPowerLoadParams params;
  params.component_index = 51;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.p_mw = 40.0;
  params.q_mvar = 10.0;
  params.filter_r_pu = 0.02;
  params.filter_x_pu = 0.05;
  params.filter_c_s = 0.02;
  params.v_min_pu = 0.30;
  ActiveConstantPowerLoadDynamic cpl(params);
  int offset = 0;
  cpl.assignStateIndices(offset);
  REQUIRE(offset == 2);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);

  auto settle = [&](double vmag) {
    set_balanced_voltage(y, 0, std::polar(vmag, 0.0));
    cpl.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    const double dt = 0.0005;
    for (int i = 0; i < 2000; ++i) {
      Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
      cpl.computeDerivatives(0.0, x, y, dxdt);
      x.x += dt * dxdt;
    }
    return cpl.output(x, y);
  };

  SECTION("holds constant power and draws more current at lower voltage") {
    const auto hi = settle(1.0);
    const auto lo = settle(0.8);
    CHECK(hi.values.at("p_mw") == Catch::Approx(40.0).margin(1.5));
    CHECK(lo.values.at("p_mw") == Catch::Approx(40.0).margin(1.5));
    // Negative incremental resistance: lower voltage => higher current draw.
    CHECK(lo.values.at("i_draw_pu") > hi.values.at("i_draw_pu"));
  }

  SECTION("low-voltage floor rolls off to constant current") {
    set_balanced_voltage(y, 0, std::polar(0.20, 0.0));  // below v_min = 0.30
    cpl.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    CHECK(cpl.output(x, y).values.at("current_limited") == Catch::Approx(1.0));
  }
}

TEST_CASE("Flux-based single-cage induction machine: equilibrium and FIDVR stall",
          "[dynamics][loads][induction]") {
  // Design doc §13: the transient-EMF single-cage induction machine reaches a
  // torque-balanced operating point and, under a deep sustained voltage sag,
  // loses air-gap torque and decelerates (rising slip) -- the fault-induced
  // delayed voltage recovery (FIDVR) mechanism.
  InductionMachineDynamicParams params;
  params.component_index = 71;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.model_base_mva = 10.0;
  params.p_mech_mw = 8.0;  // 0.8 pu load on the machine base
  params.r_s_pu = 0.03;
  params.x_s_pu = 0.10;
  params.r_r_pu = 0.03;
  params.x_r_pu = 0.10;
  params.x_m_pu = 3.0;
  params.inertia_h = 1.0;
  params.torque_exponent = 2.0;
  params.flux_model = true;
  FluxInductionMachineDynamic motor(params);
  int offset = 0;
  motor.assignStateIndices(offset);
  REQUIRE(offset == 3);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);

  SECTION("torque-balanced equilibrium at nominal voltage") {
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    motor.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    const auto out = motor.output(x, y);
    CHECK(out.values.at("slip") > 0.0);    // motoring: positive slip
    CHECK(out.values.at("slip") < 0.10);   // stable low-slip point
    CHECK(out.values.at("p_mw") > 0.0);    // draws real power
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    motor.computeDerivatives(0.0, x, y, dxdt);
    CHECK(std::abs(dxdt[2]) < 1e-3);       // ds/dt ~ 0 at equilibrium
  }

  SECTION("stalls under a deep sustained voltage sag (FIDVR)") {
    set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
    motor.initializeFromPowerFlow(PowerFlowResult{}, x, y);
    set_balanced_voltage(y, 0, std::polar(0.5, 0.0));  // deep sag
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
    motor.computeDerivatives(0.0, x, y, dxdt);
    CHECK(dxdt[2] > 0.0);  // ds/dt > 0: rotor decelerating toward stall
  }
}

TEST_CASE("End-to-end transients with the new WECC, CPL, and flux-motor models",
          "[dynamics][transient][newmodels]") {
  // Exercise the new device models through the full builder + solver pipeline
  // (run_transient_simulation), not just as isolated unit tests.
  SECTION("WECC REGC_A + REEC_A + REPC_A renewable generator") {
    HybridPowerSystem sys = make_transient_2bus();
    StaticGenerator der;
    der.index = 1;
    der.bus = 2;
    der.in_service = true;
    der.p_mw = 12.0;
    der.q_mvar = 0.0;
    der.sn_mva = 30.0;
    der.f_ref_hz = 50.0;
    der.dynamic_model.standard = "WECC";
    der.dynamic_model.model_name = "REGC_A";
    der.dynamic_model.parameters = {{"reec_enabled", 1.0}, {"repc_enabled", 1.0}};
    sys.ac.static_generators.push_back(der);
    auto opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 0.5;
    opt.dt_s = 0.01;
    opt.record_device_outputs = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    INFO(result.message);
    REQUIRE(result.success);
    REQUIRE(result.final_snapshot() != nullptr);
    CHECK(result.final_snapshot()->min_ac_voltage_pu > 0.5);
    // Telemetry the GUI transient endpoint exposes for the renewable device.
    const auto lvacm =
        device_output_series(result, "RenewableEnergyGeneratorA", 0, "lvacm_gain");
    CHECK_FALSE(lvacm.y.empty());
    CHECK_FALSE(
        device_output_series(result, "RenewableEnergyGeneratorA", 0, "p_mw").y.empty());
  }

  SECTION("active constant-power load") {
    HybridPowerSystem sys = make_transient_2bus();
    Load cpl;
    cpl.index = 2;
    cpl.bus = 2;
    cpl.p_mw = 10.0;
    cpl.q_mvar = 2.0;
    cpl.in_service = true;
    cpl.dynamic_model.model_name = "ActiveConstantPowerLoad";
    sys.ac.loads.push_back(cpl);
    auto opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 0.5;
    opt.dt_s = 0.01;
    opt.record_device_outputs = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    INFO(result.message);
    REQUIRE(result.success);
    CHECK_FALSE(
        device_output_series(result, "ActiveConstantPowerLoad", 0, "i_draw_pu").y.empty());
  }

  SECTION("flux-based single-cage induction motor") {
    HybridPowerSystem sys = make_transient_2bus();
    AsynchronousMotor m;
    m.index = 1;
    m.bus = 2;
    m.in_service = true;
    m.sn_mva = 10.0;
    m.cos_phi = 0.85;
    m.efficiency = 0.95;
    m.r_pu = 0.03;
    m.x_pu = 0.20;
    m.dynamic_model.model_name = "FluxInductionMachine";
    sys.ac.motors = {m};
    auto opt = fast_options();
    opt.run_power_flow_initialization = true;
    opt.t_end_s = 0.5;
    opt.dt_s = 0.01;
    opt.record_device_outputs = true;
    const auto result = hacdcpf::run_transient_simulation(sys, opt);
    INFO(result.message);
    REQUIRE(result.success);
    CHECK_FALSE(
        device_output_series(result, "SingleCageInductionMachine", 0, "slip").y.empty());
  }
}



TEST_CASE("WECC REPC_A plant frequency droop curtails active power",
          "[dynamics][renewable][repca]") {
  // Design doc §11.6: REPC_A's second axis is a plant active-power / frequency
  // droop. The plant measures frequency from the terminal angle derivative and
  // curtails the active-power order under over-frequency.
  REGCADynamicParams params;
  params.component_index = 44;
  params.bus = 1;
  params.bus_pos = 0;
  params.base_mva = 100.0;
  params.model_base_mva = 100.0;
  params.p_ref_mw = 60.0;
  params.q_ref_mvar = 0.0;
  params.v_ref_pu = 1.0;
  params.reec.enabled = true;
  params.repc.enabled = true;
  params.repc.freq_control = true;
  params.repc.f_nominal_hz = 60.0;
  params.repc.f_dbd_hz = 0.017;
  params.repc.f_droop = 0.05;
  REGCADynamic regca(params);
  int offset = 0;
  regca.assignStateIndices(offset);
  REQUIRE(offset == 5);
  DynamicState x;
  x.resize(static_cast<std::size_t>(offset));
  NetworkState y;
  y.resize(3, 1);
  set_balanced_voltage(y, 0, std::polar(1.0, 0.0));
  regca.initializeFromPowerFlow(PowerFlowResult{}, x, y);

  // Advance the terminal angle each step to emulate a +0.5 Hz over-frequency
  // (f = f_nominal + dtheta/dt / 2pi).
  const double dt = 0.01;
  const double kTwoPiLocal = 2.0 * 3.14159265358979323846;
  const double dtheta = kTwoPiLocal * 0.5 * dt;
  double angle = 0.0;
  for (int i = 0; i < 100; ++i) {
    angle += dtheta;
    set_balanced_voltage(y, 0, std::polar(1.0, angle));
    regca.updateSmartControls(dt, y);
  }
  CHECK(regca.output(x, y).values.at("f_meas_hz") > 60.2);  // measured over-freq

  Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(offset);
  regca.computeDerivatives(0.0, x, y, dxdt);
  CHECK(dxdt[3] < 0.0);  // dPord/dt < 0: plant curtails under over-frequency
}

TEST_CASE("Unbalanced sequence braking torque extends to all machine models",
          "[dynamics][unbalanced][sequence]") {
  // Design doc §8.8 extended beyond OneDOneQ: any machine with sequence
  // parameters feels a negative-sequence braking torque under an unbalanced
  // terminal voltage. Comparing the same (classical) machine with and without
  // the sequence path at an identical unbalanced terminal isolates the torque.
  auto rotor_accel = [](bool sequence_modeled) {
    VoltageSourceDynamicParams p;
    p.component_index = 61;
    p.bus = 1;
    p.bus_pos = 0;
    p.base_mva = 100.0;
    p.inertia_h = 4.0;
    p.dynamic_angle = true;
    p.r_pu = 0.0;
    p.x_pu = 0.3;
    if (sequence_modeled) {
      p.x2_pu = 0.2;
      p.r2_pu = 0.02;
    }
    SynchronousMachine m(p);
    int off = 0;
    m.assignStateIndices(off);
    REQUIRE(off >= 4);
    DynamicState x;
    x.resize(static_cast<std::size_t>(off));
    x.x[0] = 0.0;  // delta
    x.x[1] = 1.0;  // omega
    x.x[2] = 1.0;  // e_mag
    x.x[3] = 0.5;  // pm
    NetworkState y;
    y.resize(3, 1);
    constexpr double pi = 3.14159265358979323846;
    y.Vac_abc[0] = std::polar(0.5, 0.0);            // phase-A sag (unbalanced)
    y.Vac_abc[1] = std::polar(1.0, -2.0 * pi / 3.0);
    y.Vac_abc[2] = std::polar(1.0, 2.0 * pi / 3.0);
    Eigen::VectorXd dxdt = Eigen::VectorXd::Zero(off);
    m.computeDerivatives(0.0, x, y, dxdt);
    return dxdt[1];  // rotor-speed derivative
  };
  // The sequence-modeled machine brakes: a lower (more negative) rotor accel.
  CHECK(rotor_accel(true) < rotor_accel(false));
}












