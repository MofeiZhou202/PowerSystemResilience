// tools/harmonics_validation/validate_harmonics_ieee13.cpp
// ---------------------------------------------------------
// Multi-bus feeder harmonic penetration emitter.
// Loads a real OpenDSS feeder (IEEE13 by default) through the phase-domain
// bridge, injects a balanced six-pulse harmonic current spectrum at a chosen
// bus, solves hacdcpf::harmonics::solve_harmonic_power_flow_3ph, and dumps the
// per-order per-phase complex bus voltages.  Companion driver:
// tools/harmonics_validation/compare_ieee13_opendss.py replays the identical
// current injections in OpenDSS (mode=harmonic) and compares.

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/harmonics_power_flow.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

using json = nlohmann::json;
namespace hh = hacdcpf::harmonics;

namespace {
constexpr double kPi = 3.14159265358979323846;

json phasor_json(const hh::Complex& v) {
  return {{"real", v.real()},
          {"imag", v.imag()},
          {"mag", std::abs(v)},
          {"ang_deg", std::arg(v) * 180.0 / kPi}};
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "usage: validate_harmonics_ieee13 <feeder.dss> <source_bus> "
                 "<i_ref_amps> [out.json]\n";
    return 2;
  }
  const std::filesystem::path dss_path = argv[1];
  const std::string src_name = argv[2];
  const double i_ref_amps = std::stod(argv[3]);
  const std::string out_path = argc > 4 ? argv[4] : "";

  try {
    auto sys = hacdcpf::analysis::load_three_phase_system_from_opendss(dss_path, 1.0);

    int src_index = -1;
    double src_kv = 0.0;
    for (const auto& b : sys.buses) {
      if (b.name == src_name) {
        src_index = b.index;
        src_kv = b.base_kv;
        break;
      }
    }
    if (src_index < 0) {
      std::cerr << "source bus not found: " << src_name << "\n";
      std::cerr << "available buses (" << sys.buses.size() << "):";
      for (const auto& b : sys.buses) std::cerr << ' ' << b.name;
      std::cerr << "\n";
      return 1;
    }
    const double i_base_amps =
        sys.base_mva * 1e6 / (std::sqrt(3.0) * src_kv * 1e3);

    hh::ThreePhaseHarmonicSource src;
    src.bus = src_index;
    src.name = "six_pulse_injection";
    src.balanced = true;
    src.i_base_pu = i_ref_amps / i_base_amps;
    src.spectrum = {{5, 20.0, 0.0},
                    {7, 100.0 / 7.0, 0.0},
                    {11, 100.0 / 11.0, 0.0},
                    {13, 100.0 / 13.0, 0.0}};

    hh::HPFOptions opt;
    opt.run_base_power_flow = false;
    opt.include_load_impedance = false;
    opt.compute_branch_flows = false;
    opt.auto_nic_from_vscs = false;
    opt.ac_orders = {5, 7, 11, 13};
    bool probe_mode = false;
    bool dump_lines = false;
    for (int i = 5; i < argc; ++i) {
      const std::string a = argv[i];
      if (a == "--loads") opt.include_load_impedance = true;
      else if (a == "--basepf") opt.run_base_power_flow = true;
      else if (a == "--source-xpp") opt.default_source_xpp_pu = std::stod(argv[++i]);
      else if (a == "--probe") probe_mode = true;
      else if (a == "--dump-lines") dump_lines = true;
      else if (a == "--no-shunts") {
        for (auto& b : sys.buses) {
          b.gs_a_mw = b.gs_b_mw = b.gs_c_mw = 0.0;
          b.bs_a_mvar = b.bs_b_mvar = b.bs_c_mvar = 0.0;
        }
      }
    }
    if (probe_mode) {
      // Driving-point impedance probe: inject i_ref_amps at the source bus
      // phase A on every order 2..25 (unbalanced, single-phase).
      src.balanced = false;
      src.i_base_pu_a = i_ref_amps / i_base_amps;
      src.i_base_pu_b = 0.0;
      src.i_base_pu_c = 0.0;
      src.spectrum.clear();
      opt.ac_orders.clear();
      for (int h = 2; h <= 25; ++h) {
        src.spectrum.push_back({h, 100.0, 0.0});
        opt.ac_orders.push_back(h);
      }
    }

    const auto res = hh::solve_harmonic_power_flow_3ph(
        sys, hh::ThreePhaseHarmonicInputs{{src}, {}}, opt);

    json out;
    out["solver"] = "hacdcpf::harmonics::solve_harmonic_power_flow_3ph";
    out["ok"] = res.ok;
    out["message"] = res.message;
    out["base_pf_converged"] = res.base_pf_converged;
    out["ac_order_solved"] = res.ac_order_solved;
    out["n_buses_loaded"] = sys.buses.size();
    out["n_lines_loaded"] = sys.lines.size();
    out["n_transformers_loaded"] = sys.transformers.size();
    if (dump_lines) {
      out["lines"] = json::array();
      auto mat3_to_json = [](const auto& m) {
        json j = json::array();
        for (int r = 0; r < 3; ++r) {
          json row = json::array();
          for (int c = 0; c < 3; ++c) row.push_back(hacdcpf::phase_matrix_get(m, r, c));
          j.push_back(row);
        }
        return j;
      };
      for (const auto& ln : sys.lines) {
        json jl = {{"name", ln.name},
                   {"from", ln.from_bus},
                   {"to", ln.to_bus},
                   {"phase_mask", static_cast<int>(ln.phase_mask.bits)},
                   {"use_phase_matrix", ln.use_phase_matrix},
                   {"length_km", ln.length_km},
                   {"r1_pu", ln.r1_pu}, {"x1_pu", ln.x1_pu}, {"b1_pu", ln.b1_pu},
                   {"r0_pu", ln.r0_pu}, {"x0_pu", ln.x0_pu}, {"b0_pu", ln.b0_pu}};
        if (ln.use_phase_matrix) {
          jl["r_matrix_pu"] = mat3_to_json(ln.r_matrix_pu);
          jl["x_matrix_pu"] = mat3_to_json(ln.x_matrix_pu);
          jl["b_matrix_pu"] = mat3_to_json(ln.b_matrix_pu);
        }
        out["lines"].push_back(jl);
      }
      out["transformers"] = json::array();
      for (const auto& tf : sys.transformers) {
        out["transformers"].push_back(
            {{"name", tf.name}, {"hv", tf.hv_bus}, {"lv", tf.lv_bus},
             {"sn_mva", tf.sn_mva}, {"vn_hv_kv", tf.vn_hv_kv},
             {"vn_lv_kv", tf.vn_lv_kv}, {"vk_percent", tf.vk_percent},
             {"vkr_percent", tf.vkr_percent},
             {"vector_group", tf.vector_group},
             {"hv_topo", tf.hv_winding_topology},
             {"lv_topo", tf.lv_winding_topology},
             {"tap_pos", tf.tap_pos}, {"tap_step_percent", tf.tap_step_percent},
             {"tap_side", tf.tap_side}});
      }
      out["external_grids"] = json::array();
      for (const auto& eg : sys.external_grids) {
        out["external_grids"].push_back(
            {{"bus", eg.bus}, {"r1_pu", eg.r1_pu}, {"x1_pu", eg.x1_pu},
             {"r0_pu", eg.r0_pu}, {"x0_pu", eg.x0_pu},
             {"s_sc_max_mva", eg.s_sc_max_mva}, {"rx_max", eg.rx_max}});
      }
      out["bus_shunts"] = json::array();
      for (const auto& b : sys.buses) {
        if (b.gs_a_mw != 0.0 || b.bs_a_mvar != 0.0 || b.gs_b_mw != 0.0 ||
            b.bs_b_mvar != 0.0 || b.gs_c_mw != 0.0 || b.bs_c_mvar != 0.0) {
          out["bus_shunts"].push_back(
              {{"name", b.name},
               {"bs_mvar", {b.bs_a_mvar, b.bs_b_mvar, b.bs_c_mvar}}});
        }
      }
    }
    out["bus_names"] = json::array();
    for (const auto& b : sys.buses) out["bus_names"].push_back(b.name);
    out["feeder"] = std::filesystem::path(dss_path).filename().string();
    out["base_mva"] = sys.base_mva;
    out["source"] = {{"bus_name", src_name},
                     {"bus_index", src_index},
                     {"base_kv", src_kv},
                     {"i_base_amps", i_base_amps},
                     {"i_ref_amps", i_ref_amps},
                     {"i_base_pu", src.i_base_pu}};
    out["spectrum"] = json::array();
    for (const auto& line : src.spectrum) {
      out["spectrum"].push_back({{"order", line.order},
                                 {"mag_percent", line.mag_percent},
                                 {"phase_deg", line.phase_deg}});
    }
    out["buses"] = json::array();
    for (const auto& br : res.bus_results) {
      const auto& b = sys.buses[static_cast<size_t>(br.bus - 1)];
      json jb = {{"bus", br.bus},
                 {"name", b.name},
                 {"base_kv", b.base_kv},
                 {"orders", json::array()}};
      for (const auto& [order, va] : br.v_by_order_a) {
        jb["orders"].push_back(
            {{"order", order},
             {"phases",
              {phasor_json(va),
               phasor_json(br.v_by_order_b.at(order)),
               phasor_json(br.v_by_order_c.at(order))}}});
      }
      out["buses"].push_back(jb);
    }

    const std::string text = out.dump(2);
    if (!out_path.empty()) {
      std::ofstream fout(out_path);
      fout << text << "\n";
      std::cout << "wrote " << out_path << " (ok=" << res.ok << ")\n";
    } else {
      std::cout << text << "\n";
    }
    return res.ok ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "validate_harmonics_ieee13: " << e.what() << "\n";
    return 1;
  }
}
