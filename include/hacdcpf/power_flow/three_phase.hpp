#pragma once

// Three-phase unbalanced power flow solver.
//
// Primary solver: Newton-Raphson on abc-domain 3n×3n admittance matrix.
//   - Handles radial and meshed topologies, multiple sources.
//   - Uses polar-form state variables and sparse Jacobian.
//   - Delegates to hacdcpf::analysis::solve_three_phase_nr().
//
// Legacy backward-forward sweep APIs are intentionally not exposed; distribution
// feeders use the three-phase Newton-Raphson path.

#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace hacdcpf::powerflow {

// Three-phase power flow options
struct ThreePhaseFlowOptions {
  int max_iter{50};
  double tol{1e-6};
  bool verbose{false};
  bool include_shunts{true};
};

// Three-phase power flow result
struct ThreePhaseFlowResult {
  std::vector<ThreePhaseBusResult> bus_results;
  bool converged{false};
  int iterations{0};
  double residual{0.0};
  double total_p_loss_mw{0.0};
  double total_q_loss_mvar{0.0};
  double max_vuf_percent{0.0};
};

// Solve three-phase power flow using Newton-Raphson (abc-domain).
// This is the primary solver for general networks (meshed, multi-source).
inline ThreePhaseFlowResult solve_three_phase(
    const ThreePhaseACSystem& sys,
    const ThreePhaseFlowOptions& opt = {}) {

  analysis::ThreePhaseNROptions nr_opt;
  nr_opt.max_iter       = opt.max_iter;
  nr_opt.tol            = opt.tol;
  nr_opt.verbose        = opt.verbose;
  nr_opt.include_shunts = opt.include_shunts;

  auto tp_result = analysis::solve_three_phase_nr(sys, nr_opt);

  ThreePhaseFlowResult result;
  result.converged         = tp_result.converged;
  result.iterations        = tp_result.iterations;
  result.residual          = tp_result.residual;
  result.total_p_loss_mw   = tp_result.total_p_loss_mw;
  result.total_q_loss_mvar = tp_result.total_q_loss_mvar;
  result.max_vuf_percent   = tp_result.max_vuf_percent;

  // Convert ThreePhaseBusVoltage → ThreePhaseBusResult
  result.bus_results.resize(tp_result.bus_voltages.size());
  for (size_t i = 0; i < tp_result.bus_voltages.size(); ++i) {
    auto& br = result.bus_results[i];
    const auto& bv = tp_result.bus_voltages[i];
    br.bus_id  = bv.bus_id;
    br.vm_a_pu = bv.vm_a_pu;  br.va_a_deg = bv.va_a_deg;
    br.vm_b_pu = bv.vm_b_pu;  br.va_b_deg = bv.va_b_deg;
    br.vm_c_pu = bv.vm_c_pu;  br.va_c_deg = bv.va_c_deg;
  }
  return result;
}

}  // namespace hacdcpf::powerflow
