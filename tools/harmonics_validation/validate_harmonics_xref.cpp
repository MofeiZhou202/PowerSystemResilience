#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/harmonics_power_flow.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"

using json = nlohmann::json;
using hacdcpf::ACBranch;
using hacdcpf::ACBus;
using hacdcpf::BusType;
using hacdcpf::DCBranch;
using hacdcpf::DCBus;
using hacdcpf::DCBusType;
using hacdcpf::HybridPowerSystem;
using hacdcpf::PhaseMask;
using hacdcpf::ThreePhaseACBus;
using hacdcpf::ThreePhaseACLine;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::VSCConverter;
using hacdcpf::ConverterMode;
using hacdcpf::harmonics::Complex;
using hacdcpf::harmonics::HPFOptions;
using hacdcpf::harmonics::HPFResult;
using hacdcpf::harmonics::HarmonicCurrentSource;
using hacdcpf::harmonics::HarmonicNIC;
using hacdcpf::harmonics::HarmonicSpectrum;
using hacdcpf::harmonics::HarmonicSpectrumLine;
using hacdcpf::harmonics::HarmonicStudyInputs;
using hacdcpf::harmonics::HPF3phResult;
using hacdcpf::harmonics::PortBehavior;
using hacdcpf::harmonics::SkinEffectModel;
using hacdcpf::harmonics::ThreePhaseHarmonicInputs;
using hacdcpf::harmonics::ThreePhaseHarmonicSource;
using hacdcpf::harmonics::solve_harmonic_power_flow;
using hacdcpf::harmonics::solve_harmonic_power_flow_3ph;

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

template <typename T>
T get_or(const json& j, const char* key, T fallback) {
  auto it = j.find(key);
  return it == j.end() ? fallback : it->get<T>();
}

std::vector<int> int_vector_or(const json& j, const char* key,
                               std::vector<int> fallback) {
  auto it = j.find(key);
  return it == j.end() ? fallback : it->get<std::vector<int>>();
}

HarmonicSpectrum spectrum_from_json(const json& arr) {
  HarmonicSpectrum spec;
  for (const auto& row : arr) {
    spec.push_back({row.at("order").get<int>(),
                    get_or(row, "mag_percent", 0.0),
                    get_or(row, "phase_deg", 0.0)});
  }
  return spec;
}

json spectrum_to_json(const HarmonicSpectrum& spec) {
  json out = json::array();
  for (const auto& row : spec) {
    out.push_back({{"order", row.order},
                   {"mag_percent", row.mag_percent},
                   {"phase_deg", row.phase_deg}});
  }
  return out;
}

ACBus ac_bus(int id, BusType type, double base_kv) {
  ACBus b;
  b.index = id;
  b.bus_type = type;
  b.vm_pu = 1.0;
  b.va_deg = 0.0;
  b.base_kv = base_kv;
  b.in_service = true;
  return b;
}

ACBranch ac_line(int id, int from, int to, double r, double x) {
  ACBranch br;
  br.index = id;
  br.from_bus = from;
  br.to_bus = to;
  br.r_pu = r;
  br.x_pu = x;
  br.tap = 1.0;
  br.in_service = true;
  return br;
}

DCBus dc_bus(int id, DCBusType type, double base_kv) {
  DCBus b;
  b.index = id;
  b.bus_type = type;
  b.vm_pu = 1.0;
  b.base_kv = base_kv;
  b.in_service = true;
  return b;
}

DCBranch dc_line(int id, int from, int to, double r) {
  DCBranch br;
  br.index = id;
  br.from_bus = from;
  br.to_bus = to;
  br.r_pu = r;
  br.in_service = true;
  return br;
}

json phasor_json(const Complex& v) {
  return {{"real", v.real()},
          {"imag", v.imag()},
          {"mag", std::abs(v)},
          {"ang_deg", std::arg(v) * 180.0 / kPi}};
}

json result_to_json(const HPFResult& res) {
  json out;
  out["solver"] = "hacdcpf::harmonics::solve_harmonic_power_flow";
  out["ok"] = res.ok;
  out["message"] = res.message;
  out["base_pf_converged"] = res.base_pf_converged;
  out["max_ac_thd_pct"] = res.max_ac_thd_pct;
  out["max_dc_thd_pct"] = res.max_dc_thd_pct;
  out["max_ac_thd_bus"] = res.max_ac_thd_bus;
  out["max_dc_thd_bus"] = res.max_dc_thd_bus;
  out["ac_order_solved"] = res.ac_order_solved;
  out["dc_order_solved"] = res.dc_order_solved;

  out["buses"] = json::array();
  for (const auto& b : res.ac_bus_results) {
    json jb;
    jb["bus"] = b.bus;
    jb["is_dc"] = false;
    jb["v_fund_pu"] = b.v_fund_pu;
    jb["thd_pct"] = b.thd_pct;
    jb["orders"] = json::array();
    for (const auto& [ord, v] : b.v_by_order) {
      json row = phasor_json(v);
      row["order"] = ord;
      jb["orders"].push_back(row);
    }
    out["buses"].push_back(jb);
  }

  out["dc_buses"] = json::array();
  for (const auto& b : res.dc_bus_results) {
    json jb;
    jb["bus"] = b.bus;
    jb["is_dc"] = true;
    jb["v_fund_pu"] = b.v_fund_pu;
    jb["thd_pct"] = b.thd_pct;
    jb["orders"] = json::array();
    for (const auto& [ord, v] : b.v_by_order) {
      json row = phasor_json(v);
      row["order"] = ord;
      jb["orders"].push_back(row);
    }
    out["dc_buses"].push_back(jb);
  }

  out["branches"] = json::array();
  for (const auto& br : res.ac_branch_flows) {
    json jb;
    jb["branch_index"] = br.branch_index;
    jb["from"] = br.from_bus;
    jb["to"] = br.to_bus;
    jb["is_dc"] = false;
    jb["thd_i_pct"] = br.thd_i_pct;
    jb["orders"] = json::array();
    for (const auto& [ord, i] : br.i_by_order) {
      jb["orders"].push_back({{"order", ord}, {"i_pu", i},
                               {"i_to_pu", br.i_to_by_order.at(ord)},
                               {"i_series_pu", br.i_series_by_order.at(ord)}});
    }
    out["branches"].push_back(jb);
  }

  out["dc_branches"] = json::array();
  for (const auto& br : res.dc_branch_flows) {
    json jb;
    jb["branch_index"] = br.branch_index;
    jb["from"] = br.from_bus;
    jb["to"] = br.to_bus;
    jb["is_dc"] = true;
    jb["thd_i_pct"] = br.thd_i_pct;
    jb["orders"] = json::array();
    for (const auto& [ord, i] : br.i_by_order) {
      jb["orders"].push_back({{"order", ord}, {"i_pu", i}});
    }
    out["dc_branches"].push_back(jb);
  }

  return out;
}

ThreePhaseACBus three_phase_bus(int id, BusType type, double base_kv) {
  ThreePhaseACBus b;
  b.index = id;
  b.bus_type = type;
  b.phase_mask = PhaseMask::abc();
  b.vm_a_pu = 1.0;
  b.va_a_deg = 0.0;
  b.vm_b_pu = 1.0;
  b.va_b_deg = -120.0;
  b.vm_c_pu = 1.0;
  b.va_c_deg = 120.0;
  b.base_kv = base_kv;
  b.in_service = true;
  return b;
}

ThreePhaseACLine three_phase_line(int id, int from, int to, const json& line) {
  ThreePhaseACLine br;
  br.index = id;
  br.from_bus = from;
  br.to_bus = to;
  br.phase_mask = PhaseMask::abc();
  br.r1_pu = get_or(line, "r1_pu", 0.01);
  br.x1_pu = get_or(line, "x1_pu", 0.1);
  br.b1_pu = get_or(line, "b1_pu", 0.0);
  br.r0_pu = get_or(line, "r0_pu", br.r1_pu);
  br.x0_pu = get_or(line, "x0_pu", br.x1_pu);
  br.b0_pu = get_or(line, "b0_pu", br.b1_pu);
  br.in_service = true;
  return br;
}

json result_3ph_to_json(const HPF3phResult& res) {
  json out;
  out["solver"] = "hacdcpf::harmonics::solve_harmonic_power_flow_3ph";
  out["ok"] = res.ok;
  out["message"] = res.message;
  out["base_pf_converged"] = res.base_pf_converged;
  out["max_ac_thd_pct"] = res.max_thd_pct;
  out["max_ac_thd_bus"] = res.max_thd_bus;
  out["ac_order_solved"] = res.ac_order_solved;
  out["buses"] = json::array();
  for (const auto& b : res.bus_results) {
    json jb = {{"bus", b.bus},
               {"v_fund_pu", {b.v_fund_pu_a, b.v_fund_pu_b, b.v_fund_pu_c}},
               {"thd_pct", {b.thd_a_pct, b.thd_b_pct, b.thd_c_pct}},
               {"orders", json::array()}};
    for (const auto& [order, va] : b.v_by_order_a) {
      json phases = json::array();
      phases.push_back(phasor_json(va));
      phases.push_back(phasor_json(b.v_by_order_b.at(order)));
      phases.push_back(phasor_json(b.v_by_order_c.at(order)));
      jb["orders"].push_back({{"order", order}, {"phases", phases}});
    }
    out["buses"].push_back(jb);
  }
  return out;
}

void configure_frequency_models(const json& cfg, HPFOptions& opt) {
  const std::string skin = get_or<std::string>(cfg, "skin_effect", "none");
  if (skin == "sqrt_order") opt.skin_effect = SkinEffectModel::SqrtOrder;
  else if (skin == "proportional_sqrt") opt.skin_effect = SkinEffectModel::ProportionalSqrt;
  else opt.skin_effect = SkinEffectModel::None;
  opt.skin_coefficient = get_or(cfg, "skin_coefficient", 0.0);
  if (cfg.contains("dc_branch_x_pu"))
    opt.dc_ripple_model.branch_x_pu[1] = cfg["dc_branch_x_pu"].get<double>();
  if (cfg.contains("dc_bus_b_pu"))
    opt.dc_ripple_model.bus_b_pu[11] = cfg["dc_bus_b_pu"].get<double>();
}

json solve_canonical_case(const json& cfg) {
  const double base_mva = get_or(cfg, "base_mva", 100.0);
  const double base_kv = get_or(cfg, "base_kv", 10.0);
  const int load_bus = get_or(cfg, "load_bus", 2);
  const auto line = cfg.value("line", json::object());

  HybridPowerSystem sys;
  sys.base_mva = base_mva;
  sys.ac.base_mva = base_mva;
  sys.ac.buses = {ac_bus(1, BusType::SLACK, base_kv),
                  ac_bus(load_bus, BusType::PQ, base_kv)};
  sys.ac.branches = {ac_line(1, 1, load_bus,
                             get_or(line, "r_pu", 0.01),
                             get_or(line, "x_pu", 0.1))};

  HarmonicCurrentSource src;
  src.bus = load_bus;
  if (cfg.contains("source") && cfg["source"].is_number_integer()) {
    src.bus = cfg["source"].get<int>();
  }
  src.i_base_pu = get_or(cfg, "i_base_pu", 0.3);
  src.i_base_phase_deg = get_or(cfg, "i_base_phase_deg", 0.0);
  src.spectrum = spectrum_from_json(cfg.at("spectrum"));

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.compute_branch_flows = true;
  opt.ac_orders = int_vector_or(cfg, "ac_orders", {5, 7, 11, 13});
  opt.dc_orders.clear();
  opt.default_source_xpp_pu = get_or(cfg, "source_xpp_pu", 0.2);
  configure_frequency_models(cfg, opt);

  HPFResult res = solve_harmonic_power_flow(sys, HarmonicStudyInputs{{src}, {}}, opt);
  json out = result_to_json(res);
  out["case_type"] = "canonical_current_source";
  out["network"] = {{"base_mva", base_mva},
                    {"base_kv", base_kv},
                    {"source_bus", 1},
                    {"load_bus", load_bus},
                    {"line", line},
                    {"source_xpp_pu", opt.default_source_xpp_pu}};
  out["device"] = {{"type", "HarmonicCurrentSource"},
                   {"i_base_pu", src.i_base_pu},
                   {"i_base_phase_deg", src.i_base_phase_deg},
                   {"spectrum", spectrum_to_json(src.spectrum)}};
  return out;
}

json solve_canonical_dc_case(const json& cfg) {
  const double base_mva = get_or(cfg, "base_mva", 100.0);
  const double base_kv = get_or(cfg, "dc_base_kv", 1.0);
  const auto line = cfg.value("dc_line", json::object());
  HybridPowerSystem sys;
  sys.base_mva = base_mva;
  sys.dc.base_mva = base_mva;
  sys.dc.buses = {dc_bus(10, DCBusType::DC_V, base_kv),
                  dc_bus(11, DCBusType::DC_P, base_kv)};
  sys.dc.branches = {dc_line(1, 10, 11, get_or(line, "r_pu", 0.05))};

  HarmonicCurrentSource src;
  src.bus = 11;
  src.is_dc = true;
  src.i_base_pu = get_or(cfg, "i_base_pu", 0.5);
  src.i_base_phase_deg = get_or(cfg, "i_base_phase_deg", 0.0);
  src.spectrum = spectrum_from_json(cfg.at("spectrum"));

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.compute_branch_flows = true;
  opt.ac_orders.clear();
  opt.dc_orders = int_vector_or(cfg, "dc_orders", {6, 12, 18});
  opt.dc_source_impedance_pu = get_or(cfg, "dc_source_impedance_pu", 0.01);
  configure_frequency_models(cfg, opt);

  HPFResult res = solve_harmonic_power_flow(sys, HarmonicStudyInputs{{src}, {}}, opt);
  json out = result_to_json(res);
  out["case_type"] = "canonical_dc_current_source";
  out["network"] = {{"base_mva", base_mva},
                    {"dc_base_kv", base_kv},
                    {"source_bus", 10},
                    {"load_bus", 11},
                    {"dc_line", line},
                    {"dc_source_impedance_pu", opt.dc_source_impedance_pu},
                    {"dc_branch_x_pu", get_or(cfg, "dc_branch_x_pu", 0.0)},
                    {"dc_bus_b_pu", get_or(cfg, "dc_bus_b_pu", 0.0)}};
  out["device"] = {{"type", "DCRippleCurrentSource"},
                   {"i_base_pu", src.i_base_pu},
                   {"i_base_phase_deg", src.i_base_phase_deg},
                   {"spectrum", spectrum_to_json(src.spectrum)}};
  return out;
}

json solve_three_phase_case(const json& cfg) {
  const double base_mva = get_or(cfg, "base_mva", 100.0);
  const double base_kv = get_or(cfg, "base_kv", 10.0);
  const auto line = cfg.value("line", json::object());
  ThreePhaseACSystem sys;
  sys.base_mva = base_mva;
  sys.buses = {three_phase_bus(1, BusType::SLACK, base_kv),
               three_phase_bus(2, BusType::PQ, base_kv)};
  sys.lines = {three_phase_line(1, 1, 2, line)};

  ThreePhaseHarmonicSource src;
  src.bus = 2;
  src.balanced = get_or(cfg, "balanced", true);
  src.i_base_pu = get_or(cfg, "i_base_pu", 0.3);
  src.i_base_phase_deg = get_or(cfg, "i_base_phase_deg", 0.0);
  src.i_base_pu_a = get_or(cfg, "i_base_pu_a", src.i_base_pu);
  src.i_base_pu_b = get_or(cfg, "i_base_pu_b", src.i_base_pu);
  src.i_base_pu_c = get_or(cfg, "i_base_pu_c", src.i_base_pu);
  if (cfg.contains("i_base_pu_abc")) {
    const auto values = cfg["i_base_pu_abc"].get<std::vector<double>>();
    if (values.size() != 3)
      throw std::runtime_error("i_base_pu_abc must contain exactly three values");
    src.i_base_pu_a = values[0];
    src.i_base_pu_b = values[1];
    src.i_base_pu_c = values[2];
  }
  src.spectrum = spectrum_from_json(cfg.at("spectrum"));

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.ac_orders = int_vector_or(cfg, "ac_orders", {3, 5, 7, 11, 13});
  opt.default_source_xpp_pu = get_or(cfg, "source_xpp_pu", 0.2);
  configure_frequency_models(cfg, opt);
  HPF3phResult res = solve_harmonic_power_flow_3ph(
      sys, ThreePhaseHarmonicInputs{{src}, {}}, opt);
  json out = result_3ph_to_json(res);
  out["case_type"] = "canonical_three_phase_current_source";
  out["balanced"] = src.balanced;
  out["network"] = {{"base_mva", base_mva}, {"base_kv", base_kv},
                    {"source_bus", 1}, {"load_bus", 2}, {"line", line},
                    {"source_xpp_pu", opt.default_source_xpp_pu}};
  out["device"] = {{"type", "ThreePhaseHarmonicSource"},
                   {"i_base_pu", src.i_base_pu},
                   {"i_base_pu_abc", {src.i_base_pu_a, src.i_base_pu_b, src.i_base_pu_c}},
                   {"i_base_phase_deg", src.i_base_phase_deg},
                   {"spectrum", spectrum_to_json(src.spectrum)}};
  return out;
}

json solve_device_vsc_case(const json& cfg) {
  const double base_mva = get_or(cfg, "base_mva", 100.0);
  const double base_kv = get_or(cfg, "base_kv", 10.0);
  const double dc_base_kv = get_or(cfg, "dc_base_kv", 1.0);
  const double p_mw = get_or(cfg, "p_mw", 30.0);
  const double q_mvar = get_or(cfg, "q_mvar", 0.0);
  const double p_dc_mw = get_or(cfg, "p_dc_mw", -p_mw);
  const auto line = cfg.value("line", json::object());
  const auto dc_line_cfg = cfg.value("dc_line", json::object());

  HybridPowerSystem sys;
  sys.name = "harmonic VSC validation";
  sys.base_mva = base_mva;
  sys.ac.base_mva = base_mva;
  sys.dc.base_mva = base_mva;
  sys.ac.buses = {ac_bus(1, BusType::SLACK, base_kv),
                  ac_bus(2, BusType::PQ, base_kv)};
  sys.ac.branches = {ac_line(1, 1, 2,
                             get_or(line, "r_pu", 0.01),
                             get_or(line, "x_pu", 0.1))};
  sys.dc.buses = {dc_bus(10, DCBusType::DC_V, dc_base_kv),
                  dc_bus(11, DCBusType::DC_P, dc_base_kv)};
  sys.dc.branches = {dc_line(1, 10, 11, get_or(dc_line_cfg, "r_pu", 0.05))};

  VSCConverter vsc;
  vsc.index = 0;
  vsc.bus_ac = 2;
  vsc.bus_dc = 11;
  vsc.in_service = true;
  vsc.control_mode = ConverterMode::PQ_MODE;
  vsc.p_set_mw = p_mw;
  vsc.q_set_mvar = q_mvar;
  vsc.name = "validation_vsc";
  sys.vsc_converters = {vsc};

  HarmonicNIC nic;
  nic.vsc_index = 0;
  nic.bus_ac = 2;
  nic.bus_dc = 11;
  nic.name = "validation_vsc";
  nic.ac_port = PortBehavior::GridFollowing;
  nic.dc_port = PortBehavior::GridFollowing;
  nic.s_ac_p_mw = p_mw;
  nic.s_ac_q_mvar = q_mvar;
  nic.p_dc_mw = p_dc_mw;
  nic.ac_spectrum = cfg.contains("ac_spectrum")
                        ? spectrum_from_json(cfg["ac_spectrum"])
                        : hacdcpf::harmonics::default_six_pulse_ac_spectrum();
  nic.dc_spectrum = cfg.contains("dc_spectrum")
                        ? spectrum_from_json(cfg["dc_spectrum"])
                        : hacdcpf::harmonics::default_dc_ripple_spectrum();

  HPFOptions opt;
  opt.run_base_power_flow = false;
  opt.include_load_impedance = false;
  opt.compute_branch_flows = true;
  opt.auto_nic_from_vscs = false;
  opt.ac_orders = int_vector_or(cfg, "ac_orders", {5, 7, 11, 13});
  opt.dc_orders = int_vector_or(cfg, "dc_orders", {6, 12});
  opt.default_source_xpp_pu = get_or(cfg, "source_xpp_pu", 0.2);
  opt.dc_source_impedance_pu = get_or(cfg, "dc_source_impedance_pu", 0.01);
  configure_frequency_models(cfg, opt);

  HPFResult res = solve_harmonic_power_flow(sys, HarmonicStudyInputs{{}, {nic}}, opt);
  json out = result_to_json(res);
  out["case_type"] = "device_vsc_nic";
  out["network"] = {{"base_mva", base_mva},
                    {"base_kv", base_kv},
                    {"source_bus", 1},
                    {"converter_ac_bus", 2},
                    {"line", line},
                    {"source_xpp_pu", opt.default_source_xpp_pu},
                    {"dc_source_bus", 10},
                    {"converter_dc_bus", 11},
                    {"dc_line", dc_line_cfg},
                    {"dc_source_impedance_pu", opt.dc_source_impedance_pu}};
  const double i_ac1_mag = std::hypot(p_mw / base_mva, q_mvar / base_mva);
  const double i_dc0_mag = std::abs((p_dc_mw / base_mva) / 1.0);
  out["device"] = {{"type", "VSCConverter/HarmonicNIC"},
                   {"p_mw", p_mw},
                   {"q_mvar", q_mvar},
                   {"p_dc_mw", p_dc_mw},
                   {"i_ac1_mag_pu", i_ac1_mag},
                   {"i_dc0_mag_pu", i_dc0_mag},
                   {"ac_spectrum", spectrum_to_json(nic.ac_spectrum)},
                   {"dc_spectrum", spectrum_to_json(nic.dc_spectrum)}};
  out["device"]["ac_current_by_order"] = json::array();
  const double i_ac1_phase = std::atan2(-q_mvar, p_mw);
  for (const auto& row : nic.ac_spectrum) {
    out["device"]["ac_current_by_order"].push_back(
        {{"order", row.order},
         {"current", phasor_json(std::polar(i_ac1_mag * row.mag_percent / 100.0,
                                            row.phase_deg * kPi / 180.0 +
                                            i_ac1_phase))}});
  }
  out["device"]["dc_current_by_order"] = json::array();
  for (const auto& row : nic.dc_spectrum) {
    out["device"]["dc_current_by_order"].push_back(
        {{"order", row.order},
         {"current", phasor_json(std::polar(i_dc0_mag * row.mag_percent / 100.0,
                                            row.phase_deg * kPi / 180.0))}});
  }
  return out;
}

json solve_case(const json& cfg) {
  const std::string type = get_or<std::string>(cfg, "case_type", "canonical_current_source");
  if (type == "canonical_current_source") return solve_canonical_case(cfg);
  if (type == "canonical_dc_current_source") return solve_canonical_dc_case(cfg);
  if (type == "canonical_three_phase_current_source") return solve_three_phase_case(cfg);
  if (type == "device_vsc_nic") return solve_device_vsc_case(cfg);
  throw std::runtime_error("unsupported harmonics validation case_type: " + type);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: " << argv[0] << " <case.json> <out.json>\n";
    return 2;
  }

  try {
    std::ifstream in(argv[1]);
    if (!in) throw std::runtime_error("failed to open input JSON");
    json cfg = json::parse(in);
    json out = solve_case(cfg);
    std::ofstream fout(argv[2]);
    if (!fout) throw std::runtime_error("failed to open output JSON");
    fout << out.dump(2) << "\n";
    std::cout << "wrote " << argv[2] << " (ok=" << out.value("ok", false)
              << ", case_type=" << out.value("case_type", "") << ")\n";
    return out.value("ok", false) ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "validate_harmonics_xref: " << e.what() << "\n";
    return 1;
  }
}
