// Debug probe: dump per-state dxdt at t=0 for one PSD GFL fixture case.
#include <iostream>
#include <iomanip>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"

using namespace hacdcpf;
using hacdcpf::dynamics::DynamicModelBuilder;
using hacdcpf::dynamics::DynamicSolver;
using hacdcpf::dynamics::DynamicResults;
using hacdcpf::dynamics::DynamicSolverOptions;
using hacdcpf::dynamics::DynamicSystem;

int main() {
  HybridPowerSystem sys;
  sys.name = "psd_probe";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  ACBus source_bus;
  source_bus.index = 101; source_bus.name = "BUS 1";
  source_bus.bus_type = BusType::SLACK; source_bus.base_kv = 230.0;
  source_bus.vm_pu = 1.00001; source_bus.va_deg = 0.0; source_bus.in_service = true;
  ACBus inverter_bus;
  inverter_bus.index = 102; inverter_bus.name = "BUS 2";
  inverter_bus.bus_type = BusType::PQ; inverter_bus.base_kv = 230.0;
  inverter_bus.vm_pu = 1.0; inverter_bus.va_deg = 0.0; inverter_bus.in_service = true;
  sys.ac.buses = {source_bus, inverter_bus};

  ACBranch branch;
  branch.index = 1; branch.from_bus = 101; branch.to_bus = 102;
  branch.r_pu = 0.0; branch.x_pu = 0.075; branch.tap = 1.0; branch.in_service = true;
  sys.ac.branches = {branch};

  ExternalGrid source;
  source.index = 1; source.bus = 101; source.name = "InfBus";
  source.vm_pu = 1.00001; source.va_deg = 0.0;
  source.r_pu = 0.0; source.x_pu = 1.0e-5; source.in_service = true;
  sys.ac.external_grids = {source};

  VSCConverter inverter;
  inverter.index = 1; inverter.name = "generator-102-1";
  inverter.bus_ac = 102; inverter.bus_dc = 0; inverter.in_service = true;
  inverter.control_mode = ConverterMode::PQ_MODE;
  inverter.p_set_mw = 50.0; inverter.q_set_mvar = 0.0;
  inverter.v_dc_set_pu = 1.0; inverter.v_ac_set_pu = 1.0;
  inverter.eta = 1.0; inverter.p_rated_mw = 100.0;
  inverter.pmax_mw = 200.0; inverter.pmin_mw = -200.0;
  inverter.dynamic_model.standard = "NERC";
  inverter.dynamic_model.model_name = "REGC_REEC_GFL_Subset";
  const double pi = 3.14159265358979323846;
  inverter.dynamic_model.components.push_back(
      {"pll", "ReducedOrderPLL", "PowerSimulationsDynamics", "test24",
       {{"kp_pll", 2.0}, {"ki_pll", 20.0}, {"pll_lpf_t_s", 1.0 / (1.32 * 2 * pi * 50)}}});
  inverter.dynamic_model.components.push_back(
      {"outer_control", "ActivePowerPI", "PowerSimulationsDynamics", "test24",
       {{"Kp_p", 2.0}, {"Ki_p", 30.0}, {"omega_z", 0.132 * 2 * pi * 50}}});
  inverter.dynamic_model.components.push_back(
      {"outer_control", "ReactivePowerPI", "PowerSimulationsDynamics", "test24",
       {{"Kp_q", 2.0}, {"Ki_q", 30.0}, {"omega_f", 0.132 * 2 * pi * 50}}});
  inverter.dynamic_model.components.push_back(
      {"inner_control", "CurrentModeControl", "PowerSimulationsDynamics", "test24",
       {{"kpc", 0.37}, {"kic", 0.7}, {"kffv", 1.0}}});
  inverter.dynamic_model.components.push_back(
      {"filter", "LCLFilter", "PowerSimulationsDynamics", "test24",
       {{"lf", 0.009}, {"rf", 0.016}, {"cf", 2.5}, {"lg", 0.002}, {"rg", 0.003}}});
  sys.vsc_converters = {inverter};

  DynamicSolverOptions opt;
  opt.run_power_flow_initialization = true;
  opt.t_end_s = 0.02;
  opt.dt_s = 0.005;
  opt.solver_type = dynamics::DynamicSolverType::MassMatrixDae;

  DynamicModelBuilder builder;
  DynamicSystem dyn = builder.build(sys, opt);
  const int n = static_cast<int>(dyn.x.x.size());
  std::cout << "states: " << n << "\nx after build:\n";
  for (int i = 0; i < n; ++i) {
    std::cout << "  x[" << i << "] = " << std::setprecision(10) << dyn.x.x[i] << "\n";
  }
  for (const auto& dev : dyn.devices) {
    std::cout << "device: " << dev->name() << "\n";
    Eigen::VectorXd d1 = Eigen::VectorXd::Zero(n);
    dev->computeDerivatives(0.0, dyn.x, dyn.y, d1);
    for (int i = 0; i < n; ++i) {
      if (d1[i] != 0.0) std::cout << "  dxdt[" << i << "] = " << d1[i] << "\n";
    }
  }

  DynamicSolver solver;
  DynamicResults res = solver.solve(dyn);
  std::cout << "success: " << res.success << " message: " << res.message << "\n";
  std::cout << "init fast_dxdt_inf_norm: " << res.initialization.dynamic_fast_dxdt_inf_norm << "\n";
  std::cout << "init dxdt inf norm: " << res.initialization.dynamic_initial_dxdt_inf_norm << "\n";
  for (const auto& w : res.initialization.warnings) {
    std::cout << "warning: " << w << "\n";
  }
  for (const auto& d : res.initialization.dynamic_residual_diagnostics) {
    std::cout << "residual: dev=" << d.device_name << " type=" << d.device_type
              << " comp=" << d.component_index << " state=" << d.state_index
              << " r=" << d.residual << "\n";
  }
  return res.success ? 0 : 1;
}
