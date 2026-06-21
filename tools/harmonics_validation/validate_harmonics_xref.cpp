// External cross-validation driver for the harmonic power-flow solver.
// =====================================================================
// Builds the canonical two-bus AC harmonic case described in case.json, solves it
// with hacdcpf::harmonics::solve_harmonic_power_flow, and writes the per-bus /
// per-branch harmonic spectrum to JSON.  An independent numpy reference
// (reference_numpy.py) and an OpenDSS cross-check (validate_opendss.py) solve the
// SAME case.json and compare.py asserts agreement.
//
// Usage:  validate_harmonics_xref <case.json> <out.json>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

#include "hacdcpf/analysis/harmonics_power_flow.hpp"

using json = nlohmann::json;
using namespace hacdcpf;
using namespace hacdcpf::harmonics;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "usage: " << argv[0] << " <case.json> <out.json>\n";
    return 2;
  }
  std::ifstream f(argv[1]);
  if (!f) { std::cerr << "cannot open " << argv[1] << "\n"; return 2; }
  json c; f >> c;

  const double base_mva = c.value("base_mva", 100.0);
  const double base_kv  = c.value("base_kv", 10.0);
  const int    load_bus = c.value("load_bus", 2);

  // ── Build the two-bus system from case.json ──
  HybridPowerSystem sys;
  sys.base_mva = base_mva;
  sys.ac.base_mva = base_mva;

  ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.0;
  b1.va_deg = 0.0; b1.base_kv = base_kv; b1.in_service = true;
  ACBus b2; b2.index = 2; b2.bus_type = BusType::PQ; b2.vm_pu = 1.0;
  b2.va_deg = 0.0; b2.base_kv = base_kv; b2.in_service = true;
  sys.ac.buses = {b1, b2};

  ACBranch ln; ln.index = 1; ln.from_bus = 1; ln.to_bus = 2;
  ln.r_pu = c["line"].value("r_pu", 0.01);
  ln.x_pu = c["line"].value("x_pu", 0.1);
  ln.b_pu = c["line"].value("b_pu", 0.0);
  ln.tap = 1.0; ln.in_service = true;
  sys.ac.branches = {ln};

  HarmonicCurrentSource src;
  src.bus = load_bus;
  src.is_dc = false;
  src.i_base_pu = c.value("i_base_pu", 0.3);
  src.i_base_phase_deg = c.value("i_base_phase_deg", 0.0);
  for (const auto& s : c["spectrum"])
    src.spectrum.push_back({s.value("order", 0), s.value("mag_percent", 0.0),
                            s.value("phase_deg", 0.0)});

  HarmonicStudyInputs in;
  in.sources = {src};

  HPFOptions opt;
  opt.run_base_power_flow = false;          // stored flat voltages -> fundamental = 1
  opt.include_load_impedance = false;       // pure Norton current source
  opt.default_source_xpp_pu = c.value("source_xpp_pu", 0.2);
  opt.ac_orders = c["ac_orders"].get<std::vector<int>>();
  opt.dc_orders = {};
  opt.auto_nic_from_vscs = false;
  opt.compute_branch_flows = true;

  HPFResult r = solve_harmonic_power_flow(sys, in, opt);

  json out;
  out["ok"] = r.ok;
  out["message"] = r.message;
  out["solver"] = "hacdcpf::harmonics::solve_harmonic_power_flow";
  out["max_ac_thd_pct"] = r.max_ac_thd_pct;

  out["buses"] = json::array();
  for (const auto& bus : r.ac_bus_results) {
    json jb;
    jb["bus"] = bus.bus;
    jb["v_fund_pu"] = bus.v_fund_pu;
    jb["thd_pct"] = bus.thd_pct;
    jb["orders"] = json::array();
    for (const auto& [ord, v] : bus.v_by_order)
      jb["orders"].push_back({{"order", ord}, {"mag", std::abs(v)},
                              {"ang_deg", std::arg(v) * 180.0 / M_PI}});
    out["buses"].push_back(jb);
  }

  out["branches"] = json::array();
  for (const auto& bf : r.ac_branch_flows) {
    json jbf;
    jbf["from"] = bf.from_bus; jbf["to"] = bf.to_bus;
    jbf["thd_i_pct"] = bf.thd_i_pct;
    jbf["orders"] = json::array();
    for (const auto& [ord, m] : bf.i_by_order)
      jbf["orders"].push_back({{"order", ord}, {"i_pu", m}});
    out["branches"].push_back(jbf);
  }

  std::ofstream of(argv[2]);
  if (!of) { std::cerr << "cannot write " << argv[2] << "\n"; return 2; }
  of << out.dump(2) << "\n";
  std::cout << "wrote " << argv[2] << " (ok=" << r.ok
            << ", max AC THD=" << r.max_ac_thd_pct << "%)\n";
  return r.ok ? 0 : 1;
}
