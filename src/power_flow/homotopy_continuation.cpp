#include "hacdcpf/power_flow/homotopy_continuation.hpp"

#include <algorithm>
#include <cmath>

#include "hacdcpf/power_flow/hybrid.hpp"

namespace hacdcpf::powerflow {

namespace {

/// Scale all active/reactive injections and setpoints by λ ∈ [0,1].
/// Voltage setpoints and network parameters are left unchanged.
static void scale_system_by_lambda(HybridPowerSystem& sys, double lambda) {
  // AC bus lumped load (inline).
  for (auto& bus : sys.ac.buses) {
    bus.pd_mw   *= lambda;
    bus.qd_mvar *= lambda;
  }
  // Explicit AC loads.
  for (auto& ld : sys.ac.loads) {
    ld.p_mw   *= lambda;
    ld.q_mvar *= lambda;
  }
  // Flexible AC loads.
  for (auto& fl : sys.ac.flexible_loads) {
    fl.p_mw   *= lambda;
    fl.q_mvar *= lambda;
  }
  // AC generators (dispatch setpoint).
  for (auto& g : sys.ac.generators) {
    g.pg_mw *= lambda;
  }
  // Static generators / SGEN (PV plants, etc.).
  for (auto& sg : sys.ac.static_generators) {
    sg.p_mw   *= lambda;
    sg.q_mvar *= lambda;
  }
  // Renewable gens.
  for (auto& rg : sys.ac.renewable_gens) {
    rg.p_mw   *= lambda;
    rg.q_mvar *= lambda;
  }
  // AC storage (dispatch setpoint).
  for (auto& st : sys.ac.storage) {
    st.p_mw *= lambda;
  }
  // DC bus lumped load.
  for (auto& bus : sys.dc.buses) {
    bus.pd_mw *= lambda;
  }
  // Explicit DC loads.
  for (auto& ld : sys.dc.loads) {
    ld.p_mw *= lambda;
  }
  // DC static generators.
  for (auto& sg : sys.dc.static_generators) {
    sg.p_mw   *= lambda;
    sg.q_mvar *= lambda;
  }
  for (auto& sg : sys.dc.dc_static_generators) {
    sg.p_set_mw *= lambda;
  }
  // PV arrays on DC bus.
  for (auto& pv : sys.dc.pv_arrays) {
    pv.p_set_mw *= lambda;
  }
  // DC storage.
  for (auto& st : sys.dc.storage) {
    st.p_mw *= lambda;
  }
  // VSC active-power setpoints (voltage control setpoints kept as-is).
  for (auto& vsc : sys.vsc_converters) {
    vsc.p_set_mw *= lambda;
  }
  // DC/DC converter power reference.
  for (auto& dcdc : sys.dc.dcdc_converters) {
    dcdc.p_ref_mw *= lambda;
  }
  // Mobile storage.
  for (auto& ms : sys.mobile_storage) {
    ms.p_mw *= lambda;
  }
}

}  // namespace

hacdcpf::PowerFlowResult HomotopyContinuationSolver::solve(
    const hacdcpf::HybridPowerSystem& sys,
    const hacdcpf::PowerFlowOptions& opt) const {
  HomotopyState state;
  return solve(sys, opt, state);
}

hacdcpf::PowerFlowResult HomotopyContinuationSolver::solve(
    const hacdcpf::HybridPowerSystem& sys,
    const hacdcpf::PowerFlowOptions& opt,
    HomotopyState& homotopy_out) const {

  const auto& hopts = opt.robust_nonlinear;
  homotopy_out = HomotopyState{};
  homotopy_out.step = hopts.homotopy_step0;

  // Disable homotopy recursion in sub-solves (avoid infinite recursion).
  hacdcpf::PowerFlowOptions sub_opt = opt;
  sub_opt.robust_nonlinear.enable_homotopy = false;

  // Use a tighter iteration budget per homotopy step to keep total time
  // bounded.  The full budget is used for the final λ = 1 solve.
  hacdcpf::PowerFlowOptions step_opt = sub_opt;
  step_opt.max_iter = std::max(10, opt.max_iter / 2);

  // Initial state: flat start (all voltages = 1.0, zero injections).
  // λ = 0 is trivially solved by the existing solver.
  hacdcpf::PowerFlowResult current;
  {
    hacdcpf::HybridPowerSystem base = sys;
    scale_system_by_lambda(base, 0.0);  // zero all injections
    current = solve_hybrid(base, sub_opt);
    // For a zero-injection system the solver always converges trivially.
    // If it doesn't, something is structurally wrong; bail out early.
    if (!current.converged) {
      homotopy_out.failed = true;
      return current;
    }
  }

  homotopy_out.lambda = 0.0;

  while (homotopy_out.lambda < 1.0 - 1e-12) {
    if (homotopy_out.accepted_steps + homotopy_out.rejected_steps >=
        hopts.homotopy_max_steps) {
      homotopy_out.failed = true;
      break;
    }
    if (homotopy_out.step < hopts.homotopy_step_min) {
      homotopy_out.failed = true;
      break;
    }

    const double lambda_next =
        std::min(1.0, homotopy_out.lambda + homotopy_out.step);

    // Build system at λ_next.
    hacdcpf::HybridPowerSystem trial_sys = sys;
    scale_system_by_lambda(trial_sys, lambda_next);

    // Warm-start from the last accepted solution.
    hacdcpf::PowerFlowOptions trial_opt =
        (lambda_next >= 1.0 - 1e-12) ? sub_opt : step_opt;
    trial_opt.initial_state =
        hacdcpf::InitialState{current.vm, current.va, current.vdc};

    hacdcpf::PowerFlowResult trial = solve_hybrid(trial_sys, trial_opt);

    if (trial.converged) {
      current = trial;
      homotopy_out.lambda = lambda_next;
      homotopy_out.accepted_steps += 1;
      // Expand step on success (capped at step_max).
      homotopy_out.step =
          std::min(2.0 * homotopy_out.step, hopts.homotopy_step_max);
    } else {
      homotopy_out.rejected_steps += 1;
      // Halve step on failure.
      homotopy_out.step *= 0.5;
    }
  }

  if (!homotopy_out.failed && std::abs(homotopy_out.lambda - 1.0) < 1e-12) {
    // Final solve at full loading (possibly already done above if the last
    // step landed exactly at λ = 1).
    if (!current.converged) {
      homotopy_out.failed = true;
    }
  }

  return current;
}

}  // namespace hacdcpf::powerflow
