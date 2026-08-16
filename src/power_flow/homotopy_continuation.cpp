#include "hacdcpf/power_flow/homotopy_continuation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "hacdcpf/power_flow/hybrid.hpp"

namespace hacdcpf::powerflow {

namespace detail {

/// Scale all active/reactive injections and setpoints by λ ∈ [0,1].
/// Voltage setpoints and network parameters are left unchanged.
void scale_system_by_lambda(HybridPowerSystem& sys, double lambda) {
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
    g.qg_mvar *= lambda;
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
    st.q_mvar *= lambda;
  }
  for (auto& pv : sys.ac.pv_systems) {
    pv.p_mw *= lambda;
    pv.q_mvar *= lambda;
    pv.irradiance *= lambda;
  }
  for (auto& cs : sys.ac.charging_stations) {
    cs.p_total_kw *= lambda;
    cs.q_total_kvar *= lambda;
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
  for (auto& st : sys.dc.dc_storage) {
    st.p_mw *= lambda;
  }
  // VSC active-power setpoints (voltage control setpoints kept as-is).
  for (auto& vsc : sys.vsc_converters) {
    vsc.p_set_mw *= lambda;
    vsc.p_schedule_mw *= lambda;
    vsc.p_initial_mw *= lambda;
    vsc.q_set_mvar *= lambda;
  }
  for (auto& lcc : sys.lcc_converters) {
    lcc.p_set_mw *= lambda;
    lcc.i_set_ka *= lambda;
  }
  // DC/DC converter power reference.
  for (auto& dcdc : sys.dc.dcdc_converters) {
    dcdc.p_ref_mw *= lambda;
  }
  // Mobile storage.
  for (auto& ms : sys.mobile_storage) {
    ms.p_mw *= lambda;
    ms.q_mvar *= lambda;
  }
  for (auto& vpp : sys.vpps) {
    vpp.p_output_mw *= lambda;
    vpp.q_output_mvar *= lambda;
  }
  for (auto& mg : sys.microgrids) {
    mg.p_exchange_mw *= lambda;
    mg.p_set_mw *= lambda;
  }
  for (auto& router : sys.energy_routers) {
    for (auto& port : router.ports) {
      port.p_mw *= lambda;
      port.q_mvar *= lambda;
      port.p_set_mw *= lambda;
      port.q_set_mvar *= lambda;
    }
  }
}

}  // namespace detail

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
  sub_opt.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
  sub_opt.initial_state.reset();

  // Use a tighter iteration budget per homotopy step to keep total time
  // bounded. Intermediate continuation points solve the smooth PV/PQ
  // electrical equations without changing the Q-limit active set. The
  // caller's complete Q-limit formulation and full budget are restored for
  // the final lambda = 1 solve, so only the requested endpoint can be
  // certified. See Allgower & Georg (1990), sec. 2.1, on regular homotopy
  // paths: discontinuous active-set changes are deferred to the endpoint.
  hacdcpf::PowerFlowOptions step_opt = sub_opt;
  step_opt.max_iter = std::max(10, opt.max_iter / 2);
  step_opt.enable_pv_pq_conversion = false;
  step_opt.enable_semi_smooth_newton = false;

  // Initial state: use the authored voltage profile at zero injections. Do not
  // inherit a caller-supplied failed state; continuation owns its path seed.
  hacdcpf::PowerFlowResult current;
  {
    hacdcpf::HybridPowerSystem base = sys;
    detail::scale_system_by_lambda(base, 0.0);  // zero all injections
    current = solve_hybrid(base, step_opt);
    // If the unconstrained electrical base point does not converge, the
    // continuation path cannot be initialized; bail out early.
    if (!current.converged) {
      homotopy_out.failed = true;
      return current;
    }
  }

  homotopy_out.lambda = 0.0;
  bool endpoint_certification_failed = false;

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
    detail::scale_system_by_lambda(trial_sys, lambda_next);

    // Warm-start from the last accepted solution.
    hacdcpf::PowerFlowOptions trial_opt = step_opt;
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
    // The regular path reaches the unconstrained full-loading root first.
    // Activate Q limits only as a separate endpoint solve so a rejected
    // active-set step cannot masquerade as a failed continuation increment.
    hacdcpf::PowerFlowOptions endpoint_opt = sub_opt;
    endpoint_opt.initial_state =
        hacdcpf::InitialState{current.vm, current.va, current.vdc};
    current = solve_hybrid(sys, endpoint_opt);
    if (!current.converged) {
      endpoint_certification_failed = true;
      homotopy_out.failed = true;
    }
  }

  if (endpoint_certification_failed) {
    current.converged = false;
    current.diagnostics.converged = false;
    current.diagnostics.termination_reason =
        "Homotopy reached lambda=1, but the Q-limit endpoint solve did not "
        "converge";
    current.diagnostics.warnings.push_back(
        "The unconstrained full-loading root was reached, but the returned "
        "state is not a Q-limit-certified solution of the requested system.");
  } else if (homotopy_out.failed || homotopy_out.lambda < 1.0 - 1e-12) {
    current.converged = false;
    current.residual = std::numeric_limits<double>::infinity();
    current.diagnostics.converged = false;
    current.diagnostics.final_mismatch_norm = current.residual;
    current.diagnostics.termination_reason =
        "Homotopy failed before reaching lambda=1 (lambda=" +
        std::to_string(homotopy_out.lambda) + ")";
    current.diagnostics.warnings.push_back(
        "The returned voltage is only the last accepted partial-loading state "
        "and is not a solution of the requested full system.");
  }
  return current;
}

}  // namespace hacdcpf::powerflow
