// Reproduce: tests/transient_generator_trip_setting_check_3gen.json
// 3 generators on bus 1 (slack 15 MW + 2x 5 MW), 25 MW load on bus 2 via a
// very strong line. Trip generator index 2 at t=0.2 s and dump per-machine
// power / in-service state and bus voltages.
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <cstdio>
#include <string>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"

using namespace hacdcpf;
using hacdcpf::dynamics::DynamicModelBuilder;
using hacdcpf::dynamics::DynamicResults;
using hacdcpf::dynamics::DynamicSolver;
using hacdcpf::dynamics::DynamicSolverOptions;
using hacdcpf::dynamics::DynamicSystem;
using hacdcpf::dynamics::DynamicEvent;
using hacdcpf::dynamics::DynamicEventType;

static Generator make_gen(int idx, const char* name, double pg, bool slack) {
  extern bool g_dynamic_slack;
  Generator g;
  g.index = idx;
  g.name = name;
  g.bus = 1;
  g.pg_mw = pg;
  g.qg_mvar = 0.0;
  g.vg_pu = 1.0;
  g.pmax_mw = slack ? 80.0 : 30.0;
  g.qmax_mvar = slack ? 40.0 : 20.0;
  g.qmin_mvar = slack ? -40.0 : -20.0;
  g.is_slack = slack;
  g.in_service = true;
  g.dynamic_model.standard = "PSS/E";
  g.dynamic_model.model_name = "GENROU";
  g.dynamic_model.parameters = {
      {"H", 5.0}, {"D", 0.0},   {"Xd", 1.8},  {"Xq", 1.7},  {"Xd_p", 0.3},
      {"Xq_p", 0.55},           {"Xd_pp", 0.25}, {"Xq_pp", 0.25},
      {"Td0p", 8.0},            {"Tq0p", 0.4},  {"Td0pp", 0.03},
      {"Tq0pp", 0.05},          {"R", 0.0},
      {"slack_dynamic_angle", g_dynamic_slack ? 1.0 : 0.0},
  };
  return g;
}

bool g_dynamic_slack = true;
int main(int argc, char** argv) {
  if (argc > 1) g_dynamic_slack = std::string(argv[1]) != "0";
  HybridPowerSystem sys;
  sys.name = "trip3gen";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus b1;
  b1.index = 1; b1.bus_type = BusType::SLACK; b1.base_kv = 110.0;
  b1.vm_pu = 1.0; b1.va_deg = 0.0; b1.in_service = true;
  ACBus b2;
  b2.index = 2; b2.bus_type = BusType::PQ; b2.base_kv = 110.0;
  b2.vm_pu = 1.0; b2.va_deg = 0.0; b2.in_service = true;
  sys.ac.buses = {b1, b2};

  ACBranch br;
  br.index = 1; br.from_bus = 1; br.to_bus = 2;
  br.r_pu = 0.001; br.x_pu = 0.0; br.tap = 1.0; br.in_service = true;
  sys.ac.branches = {br};

  sys.ac.generators = {make_gen(0, "Slack_G0_pickup", 15.0, true),
                       make_gen(1, "G1_keep_5MW", 5.0, false),
                       make_gen(2, "G2_trip_5MW", 5.0, false)};

  Load ld;
  ld.index = 0; ld.bus = 2; ld.p_mw = 25.0; ld.q_mvar = 0.0;
  ld.in_service = true;
  sys.ac.loads = {ld};

  dynamics::DynamicSolverOptions opt;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;
  opt.run_power_flow_initialization = true;
  opt.solver_type = dynamics::DynamicSolverType::MassMatrixDae;

  dynamics::DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);

  DynamicEvent trip;
  trip.time_s = 0.2;
  trip.type = DynamicEventType::GeneratorTrip;
  trip.component_index = 2;
  trip.label = "trip G2";
  dyn.events.push_back(trip);

  dynamics::DynamicSolver solver;
  DynamicResults res = solver.solve(dyn);
  std::cout << "success: " << res.success << " msg: " << res.message << "\n";
  std::cout << "applied events: " << res.applied_event_records.size() << "\n";
  for (const auto& ev : res.applied_event_records) {
    std::cout << "  t=" << ev.time_s << " type=" << ev.type
              << " comp=" << ev.component_index << "\n";
  }

  std::cout << std::fixed << std::setprecision(5);
  const auto& snaps = res.snapshots;
  for (size_t k = 0; k < snaps.size(); k += 8) {
    const auto& s = snaps[k];
    auto gv = [&](const char* type, int comp, const char* key) {
      for (const auto& d : s.device_outputs) {
        if (d.type == type && d.component_index == comp) {
          auto it = d.values.find(key);
          return it != d.values.end() ? it->second : -999.0;
        }
      }
      return -999.0;
    };
    const double v1 = std::abs(s.vac_abc[0]);
    const double v2 = std::abs(s.vac_abc[3]);
    std::cout << "t=" << s.time_s
              << "  V1=" << v1 << " V2=" << v2
              << "  G0p=" << gv("SynchronousMachine", 0, "p_mw")
              << "  G1p=" << gv("SynchronousMachine", 1, "p_mw")
              << "  G2p=" << gv("SynchronousMachine", 2, "p_mw")
              << "  G2svc=" << gv("SynchronousMachine", 2, "in_service")
              << "  G1w=" << gv("SynchronousMachine", 1, "omega_pu")
              << "  G1d=" << gv("SynchronousMachine", 1, "delta_rad")
              << "  G0emf=" << gv("SynchronousMachine", 0, "vf_pu")
              << "  G0eq=" << gv("SynchronousMachine", 0, "eq_p")
              << "  G0i=" << gv("SynchronousMachine", 0, "i_rms_pu")
              << "  G0q=" << gv("SynchronousMachine", 0, "q_mvar")
              << "  G1q=" << gv("SynchronousMachine", 1, "q_mvar")
              << "  fbus1=" << s.bus_frequency_hz[0]
              << "  fcoi=" << s.coi_frequency_hz
              << "\n";
  }
  {
    std::filesystem::create_directories("/tmp/mm");
    const char* tag = g_dynamic_slack ? "hy_dyn" : "hy_pin";
    auto dump = [&](const std::string& fname, auto getter) {
      std::string path = std::string("/tmp/mm/") + tag + "_" + fname;
      FILE* f = std::fopen(path.c_str(), "w");
      std::fprintf(f, "time_s,value\n");
      for (const auto& s : res.snapshots) std::fprintf(f, "%.6f,%.10g\n", s.time_s, getter(s));
      std::fclose(f);
    };
    auto gv = [&](const char* type, int comp, const char* key) {
      return [=](const dynamics::DynamicSnapshot& s) {
        for (const auto& d : s.device_outputs)
          if (d.type == type && d.component_index == comp) {
            auto it = d.values.find(key);
            return it != d.values.end() ? it->second : -999.0;
          }
        return -999.0;
      };
    };
    dump("g0_delta.csv", gv("SynchronousMachine", 0, "delta_rad"));
    dump("g0_omega.csv", gv("SynchronousMachine", 0, "omega_pu"));
    dump("g0_p.csv", gv("SynchronousMachine", 0, "p_mw"));
    dump("g0_q.csv", gv("SynchronousMachine", 0, "q_mvar"));
    dump("g1_delta.csv", gv("SynchronousMachine", 1, "delta_rad"));
    dump("g1_omega.csv", gv("SynchronousMachine", 1, "omega_pu"));
    dump("g1_p.csv", gv("SynchronousMachine", 1, "p_mw"));
    dump("g1_q.csv", gv("SynchronousMachine", 1, "q_mvar"));
    dump("g2_p.csv", gv("SynchronousMachine", 2, "p_mw"));
    dump("g2_delta.csv", gv("SynchronousMachine", 2, "delta_rad"));
    dump("bus1_vm.csv", [](const dynamics::DynamicSnapshot& s) { return std::abs(s.vac_abc[0]); });
    dump("bus2_vm.csv", [](const dynamics::DynamicSnapshot& s) { return std::abs(s.vac_abc[3]); });
    dump("bus1_f.csv", [](const dynamics::DynamicSnapshot& s) { return s.bus_frequency_hz[0]; });
  }
  return res.success ? 0 : 1;
}
