// tools/transformer_tap_crosscheck.cpp
// ------------------------------------
// Two-bus transformer test bench for cross-tool validation of the rich
// Transformer2W model (nameplate vk%/vkr%, tap side/pos/step, phase shift)
// against pandapower and MATPOWER (see tools/transformer_tap_crosscheck.py).
//
// Prints a compact JSON object with the solved LV-bus voltage and the
// from-side branch flow.

#include <cmath>
#include <iostream>
#include <string>

#include "hacdcpf/api/hacdcpf.hpp"

int main(int argc, char** argv) {
  // defaults: 230/115 kV on matching bus bases, neutral tap, no shift
  int tap_side = 0;
  int tap_pos = 0;
  double shift_deg = 0.0;
  double vn_hv = 230.0, vn_lv = 115.0;
  double bus_hv = 230.0, bus_lv = 115.0;
  double tap_step_percent = 1.25;
  double pd_mw = 50.0, qd_mvar = 10.0;

  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string k = argv[i];
    const std::string v = argv[i + 1];
    if (k == "--tap-side") tap_side = std::stoi(v);
    else if (k == "--tap-pos") tap_pos = std::stoi(v);
    else if (k == "--shift") shift_deg = std::stod(v);
    else if (k == "--vn-hv") vn_hv = std::stod(v);
    else if (k == "--vn-lv") vn_lv = std::stod(v);
    else if (k == "--bus-hv") bus_hv = std::stod(v);
    else if (k == "--bus-lv") bus_lv = std::stod(v);
    else if (k == "--tap-step") tap_step_percent = std::stod(v);
    else if (k == "--pd") pd_mw = std::stod(v);
    else if (k == "--qd") qd_mvar = std::stod(v);
  }

  try {
    hacdcpf::HybridPowerSystem sys;
    sys.base_mva = 100.0;

    hacdcpf::ACBus b1;
    b1.index = 1;
    b1.bus_type = hacdcpf::BusType::SLACK;
    b1.base_kv = bus_hv;
    b1.vm_pu = 1.0;
    b1.name = "HV";

    hacdcpf::ACBus b2;
    b2.index = 2;
    b2.bus_type = hacdcpf::BusType::PQ;
    b2.base_kv = bus_lv;
    b2.pd_mw = pd_mw;
    b2.qd_mvar = qd_mvar;
    b2.name = "LV";

    sys.ac.buses = {b1, b2};

    hacdcpf::Generator g;
    g.index = 1;
    g.bus = 1;
    g.in_service = true;
    g.pg_mw = 0.0;
    g.vg_pu = 1.0;
    g.qmax_mvar = 1e6;
    g.qmin_mvar = -1e6;
    g.pmax_mw = 1e6;
    g.pmin_mw = -1e6;
    g.is_slack = true;
    g.name = "GRID";
    sys.ac.generators = {g};

    hacdcpf::Transformer2W tr;
    tr.index = 1;
    tr.hv_bus = 1;
    tr.lv_bus = 2;
    tr.in_service = true;
    tr.sn_mva = 100.0;
    tr.vn_hv_kv = vn_hv;
    tr.vn_lv_kv = vn_lv;
    tr.vk_percent = 10.0;
    tr.vkr_percent = 0.5;
    tr.tap_side = tap_side;
    tr.tap_pos = tap_pos;
    tr.tap_neutral = 0;
    tr.tap_step_percent = tap_step_percent;
    tr.shift_deg = shift_deg;
    tr.name = "T1";
    sys.ac.transformers_2w = {tr};

    hacdcpf::PowerFlowOptions opt;
    opt.tol = 1e-10;
    opt.max_iter = 50;
    opt.enable_pv_pq_conversion = false;

    const auto pf = hacdcpf::solve_power_flow(sys, opt);

    std::cout.precision(12);
    std::cout << "{\"converged\":" << (pf.converged ? "true" : "false")
              << ",\"n_vm\":" << pf.vm.size()
              << ",\"n_flows\":" << pf.branch_flows.size();
    if (pf.converged && pf.vm.size() >= 2) {
      std::cout << ",\"vm_lv\":" << pf.vm[1]
                << ",\"va_lv_deg\":" << pf.va[1] * 180.0 / 3.14159265358979323846
                << ",\"vm_hv\":" << pf.vm[0];
      if (!pf.branch_flows.empty()) {
        std::cout << ",\"p_from_mw\":" << pf.branch_flows[0].pf_mw
                  << ",\"q_from_mvar\":" << pf.branch_flows[0].qf_mvar;
      }
    }
    std::cout << "}";
    return pf.converged ? 0 : 1;
  } catch (const std::exception& e) {
    std::cout << "{\"converged\":false,\"error\":\"" << e.what() << "\"}";
    return 1;
  }
}
