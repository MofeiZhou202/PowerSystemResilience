// Probe the controlled-machine 2-machine case under the multi-machine rule.
#include <iostream>
#include <iomanip>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"

using namespace hacdcpf;
using hacdcpf::dynamics::DynamicModelBuilder;
using hacdcpf::dynamics::DynamicResults;
using hacdcpf::dynamics::DynamicSolver;
using hacdcpf::dynamics::DynamicSolverOptions;
using hacdcpf::dynamics::DynamicSystem;

int main() {
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
  gs.in_service = true;
  gs.dynamic_model.parameters["pin_slack"] = 1.0;
  Generator g2; g2.index = 2; g2.bus = 2; g2.is_slack = false; g2.vg_pu = 1.0;
  g2.pg_mw = 50.0; g2.qg_mvar = 10.0; g2.xdpp_pu = 0.20; g2.inertia_h = 3.0;
  g2.pmax_mw = 200.0; g2.pmin_mw = 0.0; g2.qmax_mvar = 300; g2.qmin_mvar = -300;
  g2.in_service = true;
  g2.dynamic_model.model_name = "ClassicalMachine";
  sys.ac.generators = {gs, g2};

  Load ld; ld.index = 1; ld.bus = 2; ld.p_mw = 30.0; ld.q_mvar = 10.0; ld.in_service = true;
  sys.ac.loads = {ld};

  dynamics::DynamicSolverOptions opt;
  opt.t_end_s = 2.0;
  opt.dt_s = 0.005;
  opt.record_every_step = true;
  opt.run_power_flow_initialization = true;
  opt.solver_type = dynamics::DynamicSolverType::MassMatrixDae;

  {
    auto pf = hacdcpf::solve_power_flow(sys, {});
    std::cout << "PF converged: " << pf.converged << "\n";
    for (size_t i = 0; i < pf.vm.size(); ++i) {
      std::cout << "  bus " << (i + 1) << " vm=" << pf.vm[i]
                << " va_deg=" << pf.va[i] * 57.2958 << "\n";
    }
    for (const auto& f : pf.branch_flows) {
      std::cout << "  branch pf_mw=" << f.pf_mw << " pt_mw=" << f.pt_mw << "\n";
    }
  }
  dynamics::DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  std::cout << "builder initial PF vm: ";
  for (double v : dyn.initial_power_flow.vm) std::cout << v << " ";
  std::cout << "\n";
  dynamics::DynamicSolver solver;
  DynamicResults res = solver.solve(dyn);
  std::cout << "success: " << res.success << " msg: " << res.message << "\n";
  std::cout << "init dxdt inf norm: " << res.initialization.dynamic_fast_dxdt_inf_norm << "\n";
  std::cout << std::fixed << std::setprecision(6);
  for (size_t k = 0; k < res.snapshots.size(); k += 20) {
    const auto& s = res.snapshots[k];
    auto gv = [&](int comp, const char* key) {
      for (const auto& d : s.device_outputs)
        if (d.type == "SynchronousMachine" && d.component_index == comp) {
          auto it = d.values.find(key);
          return it != d.values.end() ? it->second : -999.0;
        }
      return -999.0;
    };
    std::cout << "t=" << s.time_s
              << "  gsp=" << gv(1, "p_mw") << " gsw=" << gv(1, "omega_pu")
              << "  g2p=" << gv(2, "p_mw") << " g2w=" << gv(2, "omega_pu")
              << "  g2d=" << gv(2, "delta_rad")
              << "  V2=" << std::abs(s.vac_abc[3]) << "\n";
  }
  return 0;
}
