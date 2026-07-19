// Verify the JSON case file loads with slack_dynamic_angle honored.
#include <iostream>
#include <iomanip>

#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"

using namespace hacdcpf;

int main(int argc, char** argv) {
  if (argc < 2) return 2;
  auto sys = io::load_json(argv[1]);
  dynamics::DynamicSolverOptions opt;
  opt.t_end_s = 0.8;
  opt.dt_s = 0.005;
  opt.record_every_step = true;
  opt.run_power_flow_initialization = true;
  opt.solver_type = dynamics::DynamicSolverType::MassMatrixDae;

  dynamics::DynamicModelBuilder builder;
  auto dyn = builder.build(sys, opt);

  dynamics::DynamicEvent trip;
  trip.time_s = 0.2;
  trip.type = dynamics::DynamicEventType::GeneratorTrip;
  trip.component_index = 2;
  dyn.events.push_back(trip);

  dynamics::DynamicSolver solver;
  auto res = solver.solve(dyn);
  std::cout << "success: " << res.success << " msg: " << res.message << "\n";
  std::cout << std::fixed << std::setprecision(5);
  for (size_t k = 0; k < res.snapshots.size(); k += 20) {
    const auto& s = res.snapshots[k];
    auto gv = [&](const char* type, int comp, const char* key) {
      for (const auto& d : s.device_outputs)
        if (d.type == type && d.component_index == comp) {
          auto it = d.values.find(key);
          return it != d.values.end() ? it->second : -999.0;
        }
      return -999.0;
    };
    std::cout << "t=" << s.time_s
              << "  V1=" << std::abs(s.vac_abc[0]) << " V2=" << std::abs(s.vac_abc[3])
              << "  G0p=" << gv("SynchronousMachine", 0, "p_mw")
              << "  G1p=" << gv("SynchronousMachine", 1, "p_mw")
              << "  G2p=" << gv("SynchronousMachine", 2, "p_mw")
              << "  G2svc=" << gv("SynchronousMachine", 2, "in_service")
              << "  G0w=" << gv("SynchronousMachine", 0, "omega_pu")
              << "  fbus1=" << s.bus_frequency_hz[0] << "\n";
  }
  return res.success ? 0 : 1;
}
