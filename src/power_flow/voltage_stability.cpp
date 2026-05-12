#include "hacdcpf/power_flow/voltage_stability.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include "hacdcpf/model/options.hpp"
#include "hacdcpf/model/results.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/solver_data.hpp"

namespace hacdcpf::powerflow {

// ─────────────────────────────────────────────────────────────────────────────
// CpfDirection factory methods
// ─────────────────────────────────────────────────────────────────────────────

CpfDirection CpfDirection::proportional(const SolverData& data) {
  const int N = static_cast<int>(data.ac_buses.size());
  CpfDirection dir;
  dir.dp_load.resize(static_cast<size_t>(N), 0.0);
  dir.dq_load.resize(static_cast<size_t>(N), 0.0);
  dir.dp_gen.resize(static_cast<size_t>(N), 0.0);

  // Load direction: proportional to base-case demand at each bus.
  for (int i = 0; i < N; ++i) {
    dir.dp_load[static_cast<size_t>(i)] = data.ac_buses[static_cast<size_t>(i)].pd_mw;
    dir.dq_load[static_cast<size_t>(i)] = data.ac_buses[static_cast<size_t>(i)].qd_mvar;
  }

  // Generation direction: all non-slack generators scale proportionally to
  // their base-case output so that the total generation increase matches the
  // total load increase (slack picks up the remainder).
  double total_nonslack_gen = 0.0;
  for (const auto& g : data.generators) {
    if (!g.in_service || g.is_slack) continue;
    total_nonslack_gen += g.pg_mw;
  }
  const double total_load = std::accumulate(
      data.ac_buses.begin(), data.ac_buses.end(), 0.0,
      [](double s, const ACBus& b) { return s + b.pd_mw; });

  for (const auto& g : data.generators) {
    if (!g.in_service || g.is_slack) continue;
    const int bi = g.bus - 1;
    if (bi < 0 || bi >= N) continue;
    // Scale each generator by the same fraction as the load increase.
    const double frac =
        (total_nonslack_gen > 0.0) ? (g.pg_mw / total_nonslack_gen) : 0.0;
    dir.dp_gen[static_cast<size_t>(bi)] += frac * total_load;
  }
  return dir;
}

CpfDirection CpfDirection::at_buses(const SolverData& data,
                                     const std::vector<int>& bus_indices) {
  const int N = static_cast<int>(data.ac_buses.size());
  CpfDirection dir;
  dir.dp_load.resize(static_cast<size_t>(N), 0.0);
  dir.dq_load.resize(static_cast<size_t>(N), 0.0);
  dir.dp_gen.resize(static_cast<size_t>(N), 0.0);
  for (int bi : bus_indices) {
    if (bi < 0 || bi >= N) continue;
    dir.dp_load[static_cast<size_t>(bi)] = 1.0;
    dir.dq_load[static_cast<size_t>(bi)] = 1.0;
  }
  return dir;
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Apply the CPF direction at loading factor lambda to a copy of base_data.
static SolverData apply_direction(const SolverData& base,
                                   double lambda,
                                   const CpfDirection& dir) {
  SolverData d = base;
  const int N = static_cast<int>(d.ac_buses.size());
  for (int i = 0; i < N; ++i) {
    const size_t si = static_cast<size_t>(i);
    d.ac_buses[si].pd_mw   += lambda * dir.dp_load[si];
    d.ac_buses[si].qd_mvar += lambda * dir.dq_load[si];
  }
  for (auto& g : d.generators) {
    if (!g.in_service || g.is_slack) continue;
    const int bi = g.bus - 1;
    if (bi < 0 || bi >= N) continue;
    g.pg_mw += lambda * dir.dp_gen[static_cast<size_t>(bi)];
  }
  aggregate_generation(d);
  return d;
}

/// Build a PowerFlowOptions suitable for CPF corrector solves.
static PowerFlowOptions make_corrector_opts(const CpfOptions& o) {
  PowerFlowOptions pfo;
  pfo.max_iter                = o.corrector_max_iter;
  pfo.tol                     = o.corrector_tol;
  pfo.enable_pv_pq_conversion = true;
  return pfo;
}

/// Build an InitialState warm-start from a previous PowerFlowResult.
static InitialState make_init(const PowerFlowResult& pf,
                               std::size_t n_dc_buses) {
  InitialState init;
  init.vm  = pf.vm;
  init.va  = pf.va;
  init.vdc = pf.vdc;
  // Ensure vdc vector is sized correctly even for pure-AC cases.
  if (init.vdc.empty() && n_dc_buses > 0)
    init.vdc.assign(n_dc_buses, 1.0);
  return init;
}

/// Compute total active and reactive load at loading factor lambda.
static std::pair<double, double> total_load(const SolverData& base,
                                             double lambda,
                                             const CpfDirection& dir) {
  double p = 0.0, q = 0.0;
  const int N = static_cast<int>(base.ac_buses.size());
  for (int i = 0; i < N; ++i) {
    const size_t si = static_cast<size_t>(i);
    p += base.ac_buses[si].pd_mw   + lambda * dir.dp_load[si];
    q += base.ac_buses[si].qd_mvar + lambda * dir.dq_load[si];
  }
  return {p, q};
}

// ─────────────────────────────────────────────────────────────────────────────
// CpfSolver::solve
// ─────────────────────────────────────────────────────────────────────────────

CpfResult CpfSolver::solve(const SolverData& base_data,
                            const CpfDirection& dir) const {
  CpfResult result;
  const int N    = static_cast<int>(base_data.ac_buses.size());
  const size_t NDC = base_data.dc_buses.size();

  if (N == 0) {
    result.termination_reason = "empty system";
    return result;
  }

  // Validate direction vector sizes.
  if (static_cast<int>(dir.dp_load.size()) != N ||
      static_cast<int>(dir.dq_load.size()) != N ||
      static_cast<int>(dir.dp_gen.size())  != N) {
    throw std::invalid_argument(
        "CpfDirection vectors must be of length equal to the number of AC buses.");
  }

  // ── Select monitor bus ────────────────────────────────────────────────────
  int monitor_bus = opts.monitor_bus;
  if (monitor_bus < 0) {
    double max_pd = -1.0;
    for (int i = 0; i < N; ++i) {
      const double pd = base_data.ac_buses[static_cast<size_t>(i)].pd_mw;
      if (pd > max_pd) {
        max_pd      = pd;
        monitor_bus = i;
      }
    }
  }
  monitor_bus         = std::max(0, std::min(N - 1, monitor_bus));
  result.monitor_bus  = monitor_bus;

  const PowerFlowOptions pfo = make_corrector_opts(opts);
  NewtonSolver nr_solver;

  // ── Lambda that runs NR and increments the solve counter ─────────────────
  auto run_nr = [&](const SolverData& d,
                     const InitialState* init) -> PowerFlowResult {
    ++result.total_pf_solves;
    return nr_solver.solve(d, pfo, init);
  };

  // ── Lambda that appends a point to the trace ──────────────────────────────
  auto record = [&](double lambda, const PowerFlowResult& pf) {
    CpfPoint pt;
    pt.lambda      = lambda;
    pt.vm_monitor  = (monitor_bus < static_cast<int>(pf.vm.size()))
                         ? pf.vm[static_cast<size_t>(monitor_bus)]
                         : 0.0;
    auto [p, q]    = total_load(base_data, lambda, dir);
    pt.p_total_mw  = p;
    pt.q_total_mvar = q;
    if (opts.trace_all_buses) {
      pt.vm = pf.vm;
      pt.va = pf.va;
    }
    result.trace.push_back(std::move(pt));
  };

  // ─────────────────────────────────────────────────────────────────────────
  // Step 0: solve the base case (λ = 0).
  // ─────────────────────────────────────────────────────────────────────────
  PowerFlowResult pf_base = run_nr(base_data, nullptr);
  if (!pf_base.converged) {
    result.termination_reason = "base case (lambda=0) did not converge";
    return result;
  }
  record(0.0, pf_base);

  // ─────────────────────────────────────────────────────────────────────────
  // State for the secant predictor: keep the two most recent accepted points.
  // ─────────────────────────────────────────────────────────────────────────
  struct SolPoint {
    double lambda{0.0};
    std::vector<double> vm;
    std::vector<double> va;
    std::vector<double> vdc;
  };

  SolPoint prev{ 0.0, pf_base.vm, pf_base.va, pf_base.vdc };
  SolPoint curr = prev;

  double lambda  = 0.0;
  double step    = opts.step_init;
  bool   first_step = true;  // use constant predictor for the first step

  // ─────────────────────────────────────────────────────────────────────────
  // Main predictor–corrector loop.
  // ─────────────────────────────────────────────────────────────────────────
  for (int iter = 0; iter < opts.max_steps; ++iter) {

    if (lambda + step * 0.5 > opts.lambda_max) {
      // Clamp so that we land exactly on lambda_max, then stop.
      step = opts.lambda_max - lambda;
      if (step < opts.step_min) {
        result.termination_reason = "lambda_max reached";
        break;
      }
    }

    const double lam_pred = lambda + step;

    // ── Predictor ──────────────────────────────────────────────────────────
    InitialState init_pred;
    if (!first_step && opts.use_secant_predictor) {
      const double dlam = curr.lambda - prev.lambda;
      if (dlam > 1e-14) {
        const double scale = step / dlam;
        init_pred.vm.resize(static_cast<size_t>(N));
        init_pred.va.resize(static_cast<size_t>(N));
        for (int i = 0; i < N; ++i) {
          const size_t si = static_cast<size_t>(i);
          init_pred.vm[si] = curr.vm[si] + scale * (curr.vm[si] - prev.vm[si]);
          init_pred.va[si] = curr.va[si] + scale * (curr.va[si] - prev.va[si]);
          // Clamp to physically reasonable range.
          init_pred.vm[si] = std::max(0.05, std::min(1.5, init_pred.vm[si]));
        }
        // DC buses: secant on DC voltages if available.
        if (!curr.vdc.empty()) {
          init_pred.vdc.resize(curr.vdc.size());
          for (size_t k = 0; k < curr.vdc.size(); ++k) {
            const double dv = curr.vdc[k] - prev.vdc[k];
            init_pred.vdc[k] = std::max(0.1, curr.vdc[k] + scale * dv);
          }
        } else if (NDC > 0) {
          init_pred.vdc.assign(NDC, 1.0);
        }
      } else {
        init_pred = make_init(pf_base, NDC);
        // Overwrite with current vm/va.
        init_pred.vm  = curr.vm;
        init_pred.va  = curr.va;
        init_pred.vdc = curr.vdc;
        if (init_pred.vdc.empty() && NDC > 0)
          init_pred.vdc.assign(NDC, 1.0);
      }
    } else {
      // Constant predictor: warm-start from current solution.
      init_pred.vm  = curr.vm;
      init_pred.va  = curr.va;
      init_pred.vdc = curr.vdc;
      if (init_pred.vdc.empty() && NDC > 0)
        init_pred.vdc.assign(NDC, 1.0);
    }

    // ── Corrector ─────────────────────────────────────────────────────────
    SolverData d_corr = apply_direction(base_data, lam_pred, dir);
    PowerFlowResult pf_corr = run_nr(d_corr, &init_pred);

    if (pf_corr.converged) {
      // Check voltage floor.
      bool vm_ok = true;
      for (double v : pf_corr.vm) {
        if (v < opts.vm_min_pu) { vm_ok = false; break; }
      }
      if (!vm_ok) {
        result.nose_found         = true;
        result.termination_reason = "voltage below vm_min_pu";
        break;
      }

      record(lam_pred, pf_corr);

      prev  = curr;
      curr  = { lam_pred, pf_corr.vm, pf_corr.va, pf_corr.vdc };
      lambda = lam_pred;
      step   = std::min(step * opts.step_grow, opts.step_max);
      first_step = false;

      if (lambda >= opts.lambda_max) {
        result.termination_reason = "lambda_max reached";
        break;
      }
    } else {
      // Corrector failed: shrink the step.
      step *= opts.step_shrink;
      if (step < opts.step_min) {
        result.nose_found         = true;
        result.termination_reason = "nose point found (step below step_min)";
        break;
      }
    }
  }

  if (result.termination_reason.empty())
    result.termination_reason = "max_steps reached";

  // ── Fill summary fields ───────────────────────────────────────────────────
  if (!result.trace.empty()) {
    const CpfPoint& last = result.trace.back();
    result.lambda_max   = last.lambda;
    result.p_max_mw     = last.p_total_mw;
    result.vm_at_nose   = last.vm_monitor;
  }

  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_vsi
// ─────────────────────────────────────────────────────────────────────────────

VoltageStabilityIndex compute_vsi(const CpfResult& result,
                                   const SolverData& base_data) {
  VoltageStabilityIndex vsi;
  vsi.bus       = result.monitor_bus;
  vsi.lambda_max = result.lambda_max;
  vsi.p_max_mw   = result.p_max_mw;
  vsi.vm_at_nose = result.vm_at_nose;

  // Base-case loading.
  double p_base = 0.0;
  for (const auto& b : base_data.ac_buses) p_base += b.pd_mw;
  vsi.p_margin_mw = vsi.p_max_mw - p_base;

  // Monitor-bus base voltage.
  if (vsi.bus >= 0 && vsi.bus < static_cast<int>(base_data.ac_buses.size())) {
    vsi.vm_base = base_data.ac_buses[static_cast<size_t>(vsi.bus)].vm_pu;
  }
  if (!result.trace.empty()) {
    vsi.vm_base = result.trace.front().vm_monitor;
  }

  return vsi;
}

}  // namespace hacdcpf::powerflow
