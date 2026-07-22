#include "hacdcpf/power_flow/voltage_stability.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "hacdcpf/power_flow/power_flow_options.hpp"
#include "hacdcpf/power_flow/power_flow_result.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/pf_utils.hpp"
#include "hacdcpf/power_flow/residual_evaluator.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

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
    dir.dp_load[static_cast<size_t>(i)] = data.has_component_loads
        ? data.pd_pu[i] * data.base_mva
        : data.ac_buses[static_cast<size_t>(i)].pd_mw;
    dir.dq_load[static_cast<size_t>(i)] = data.has_component_loads
        ? data.qd_pu[i] * data.base_mva
        : data.ac_buses[static_cast<size_t>(i)].qd_mvar;
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
      dir.dp_load.begin(), dir.dp_load.end(), 0.0);

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
    if (d.has_component_loads) {
      d.pd_pu[i] += lambda * dir.dp_load[si] / d.base_mva;
      d.qd_pu[i] += lambda * dir.dq_load[si] / d.base_mva;
    }
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
    const double p0 = base.has_component_loads
        ? base.pd_pu[i] * base.base_mva
        : base.ac_buses[si].pd_mw;
    const double q0 = base.has_component_loads
        ? base.qd_pu[i] * base.base_mva
        : base.ac_buses[si].qd_mvar;
    p += p0 + lambda * dir.dp_load[si];
    q += q0 + lambda * dir.dq_load[si];
  }
  return {p, q};
}

struct CpfStateLayout {
  std::vector<int> non_slack;
  std::vector<int> pq;
  std::vector<int> dc_non_slack;

  [[nodiscard]] int size() const noexcept {
    return static_cast<int>(non_slack.size() + pq.size() + dc_non_slack.size());
  }
};

static CpfStateLayout make_state_layout(const SolverData& data) {
  CpfStateLayout layout;
  const AcBusSets ac = classify_ac_buses(data);
  layout.non_slack = ac.non_slack;
  layout.pq = ac.pq;
  const int dc_slack = first_dc_slack_or_default(data);
  for (int i = 0; i < static_cast<int>(data.dc_buses.size()); ++i) {
    if (i != dc_slack &&
        data.dc_buses[static_cast<size_t>(i)].bus_type != DCBusType::DC_ISOLATED) {
      layout.dc_non_slack.push_back(i);
    }
  }
  return layout;
}

static Eigen::VectorXd pack_augmented_state(const CpfStateLayout& layout,
                                            const std::vector<double>& vm,
                                            const std::vector<double>& va,
                                            const std::vector<double>& vdc,
                                            double lambda) {
  Eigen::VectorXd y(layout.size() + 1);
  int k = 0;
  for (int bus : layout.non_slack) y[k++] = va[static_cast<size_t>(bus)];
  for (int bus : layout.pq) y[k++] = vm[static_cast<size_t>(bus)];
  for (int bus : layout.dc_non_slack) y[k++] = vdc[static_cast<size_t>(bus)];
  y[k] = lambda;
  return y;
}

static void unpack_augmented_state(const SolverData& base,
                                   const CpfStateLayout& layout,
                                   const Eigen::VectorXd& y,
                                   Eigen::VectorXd& vm,
                                   Eigen::VectorXd& va,
                                   Eigen::VectorXd& vdc) {
  const int n = static_cast<int>(base.ac_buses.size());
  const int ndc = static_cast<int>(base.dc_buses.size());
  vm.resize(n);
  va.resize(n);
  vdc.resize(ndc);
  for (int i = 0; i < n; ++i) {
    vm[i] = base.ac_buses[static_cast<size_t>(i)].vm_pu;
    va[i] = base.ac_buses[static_cast<size_t>(i)].va_deg *
            3.14159265358979323846 / 180.0;
  }
  for (int i = 0; i < ndc; ++i) {
    vdc[i] = base.dc_buses[static_cast<size_t>(i)].vm_pu;
  }
  int k = 0;
  for (int bus : layout.non_slack) va[bus] = y[k++];
  for (int bus : layout.pq) vm[bus] = std::max(0.02, y[k++]);
  for (int bus : layout.dc_non_slack) vdc[bus] = std::max(0.02, y[k++]);
}

static Eigen::VectorXd evaluate_cpf_residual(const SolverData& base,
                                             const CpfDirection& dir,
                                             const CpfStateLayout& layout,
                                             const Eigen::VectorXd& y) {
  Eigen::VectorXd vm, va, vdc;
  unpack_augmented_state(base, layout, y, vm, va, vdc);
  const SolverData parameterized = apply_direction(base, y[layout.size()], dir);
  return evaluate_power_flow_residual(parameterized, vm, va, vdc).full;
}

static bool arc_length_correct(const SolverData& base,
                               const CpfDirection& dir,
                               const CpfStateLayout& layout,
                               const Eigen::VectorXd& tangent,
                               const Eigen::VectorXd& predicted,
                               const CpfOptions& opts,
                               Eigen::VectorXd& corrected) {
  const int nstate = layout.size();
  corrected = predicted;
  for (int iter = 0; iter < opts.corrector_max_iter; ++iter) {
    const Eigen::VectorXd f = evaluate_cpf_residual(base, dir, layout, corrected);
    if (f.size() != nstate || !f.allFinite()) return false;

    Eigen::VectorXd augmented(nstate + 1);
    augmented.head(nstate) = f;
    augmented[nstate] = tangent.dot(corrected - predicted);
    if (augmented.cwiseAbs().maxCoeff() < opts.corrector_tol) return true;

    Eigen::MatrixXd jacobian(nstate + 1, nstate + 1);
    for (int col = 0; col <= nstate; ++col) {
      Eigen::VectorXd perturbed = corrected;
      const double h = 1e-6 * std::max(1.0, std::abs(corrected[col]));
      perturbed[col] += h;
      const Eigen::VectorXd f_perturbed =
          evaluate_cpf_residual(base, dir, layout, perturbed);
      if (f_perturbed.size() != nstate || !f_perturbed.allFinite()) return false;
      jacobian.block(0, col, nstate, 1) = (f_perturbed - f) / h;
    }
    jacobian.row(nstate) = tangent.transpose();

    Eigen::FullPivLU<Eigen::MatrixXd> lu(jacobian);
    if (!lu.isInvertible()) return false;
    const Eigen::VectorXd delta = lu.solve(-augmented);
    if (!delta.allFinite()) return false;

    // Backtrack on the augmented residual to keep difficult nose corrections
    // inside the local convergence basin.
    const double base_norm = augmented.cwiseAbs().maxCoeff();
    bool accepted = false;
    for (int ls = 0; ls < 10; ++ls) {
      const double alpha = std::ldexp(1.0, -ls);
      Eigen::VectorXd trial = corrected + alpha * delta;
      Eigen::VectorXd f_trial = evaluate_cpf_residual(base, dir, layout, trial);
      if (f_trial.size() != nstate || !f_trial.allFinite()) continue;
      Eigen::VectorXd aug_trial(nstate + 1);
      aug_trial.head(nstate) = f_trial;
      aug_trial[nstate] = tangent.dot(trial - predicted);
      if (aug_trial.cwiseAbs().maxCoeff() < base_norm) {
        corrected = std::move(trial);
        accepted = true;
        break;
      }
    }
    if (!accepted) return false;
  }
  return false;
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
      pt.vdc = pf.vdc;
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

  // Obtain a second point with the ordinary Newton solver. It establishes the
  // first secant tangent; all following arc-length corrections solve lambda as
  // part of the augmented system.
  double step = std::min(opts.step_init, opts.lambda_max);
  double lambda_first = 0.0;
  PowerFlowResult pf_first;
  while (step >= opts.step_min) {
    lambda_first = step;
    SolverData first_data = apply_direction(base_data, lambda_first, dir);
    const InitialState init = make_init(pf_base, NDC);
    pf_first = run_nr(first_data, &init);
    if (pf_first.converged) break;
    step *= opts.step_shrink;
  }
  if (!pf_first.converged) {
    result.nose_found = true;
    result.termination_reason = "no continuation step converged from base case";
  } else {
    record(lambda_first, pf_first);
  }

  const CpfStateLayout layout = make_state_layout(base_data);
  if (pf_first.converged && layout.size() > 0) {
    Eigen::VectorXd y_prev = pack_augmented_state(
        layout, pf_base.vm, pf_base.va, pf_base.vdc, 0.0);
    Eigen::VectorXd y_curr = pack_augmented_state(
        layout, pf_first.vm, pf_first.va, pf_first.vdc, lambda_first);

    auto record_state = [&](const Eigen::VectorXd& y) {
      Eigen::VectorXd vm, va, vdc;
      unpack_augmented_state(base_data, layout, y, vm, va, vdc);
      CpfPoint pt;
      pt.lambda = y[layout.size()];
      pt.vm_monitor = vm[monitor_bus];
      auto [p, q] = total_load(base_data, pt.lambda, dir);
      pt.p_total_mw = p;
      pt.q_total_mvar = q;
      if (opts.trace_all_buses) {
        pt.vm.assign(vm.data(), vm.data() + vm.size());
        pt.va.assign(va.data(), va.data() + va.size());
        if (vdc.size() > 0) {
          pt.vdc.assign(vdc.data(), vdc.data() + vdc.size());
        }
      }
      result.trace.push_back(std::move(pt));
    };

    if (!opts.enable_arc_length) {
      // Explicit legacy mode: retain natural parameterization for callers that
      // require strictly increasing lambda samples.
      double lambda = lambda_first;
      PowerFlowResult current = pf_first;
      for (int iter = 1; iter < opts.max_steps && lambda < opts.lambda_max; ++iter) {
        const double trial_lambda = std::min(opts.lambda_max, lambda + step);
        const InitialState init = make_init(current, NDC);
        PowerFlowResult trial = run_nr(apply_direction(base_data, trial_lambda, dir), &init);
        if (!trial.converged) {
          step *= opts.step_shrink;
          if (step < opts.step_min) {
            result.nose_found = true;
            result.termination_reason = "natural parameterization stalled";
            break;
          }
          continue;
        }
        record(trial_lambda, trial);
        current = std::move(trial);
        lambda = trial_lambda;
        step = std::min(step * opts.step_grow, opts.step_max);
      }
      if (result.termination_reason.empty() && lambda >= opts.lambda_max)
        result.termination_reason = "lambda_max reached";
    } else {
      step = std::clamp((y_curr - y_prev).norm(), opts.step_min, opts.step_max);
      Eigen::VectorXd previous_tangent = (y_curr - y_prev).normalized();
      bool lower_branch = false;
      int lower_branch_points = 0;

      for (int iter = 1; iter < opts.max_steps; ++iter) {
        Eigen::VectorXd tangent = opts.use_secant_predictor
            ? (y_curr - y_prev).normalized()
            : previous_tangent;
        if (tangent.dot(previous_tangent) < 0.0) tangent = -tangent;
        previous_tangent = tangent;

        const Eigen::VectorXd predicted = y_curr + step * tangent;
        if (!lower_branch && predicted[layout.size()] > opts.lambda_max) {
          result.termination_reason = "lambda_max reached";
          break;
        }

        Eigen::VectorXd corrected;
        ++result.total_pf_solves;
        if (!arc_length_correct(base_data, dir, layout, tangent, predicted,
                                opts, corrected)) {
          step *= opts.step_shrink;
          if (step < opts.step_min) {
            result.termination_reason = "arc-length corrector step below step_min";
            break;
          }
          continue;
        }

        Eigen::VectorXd vm, va, vdc;
        unpack_augmented_state(base_data, layout, corrected, vm, va, vdc);
        if (vm.minCoeff() < opts.vm_min_pu) {
          result.termination_reason = "voltage below vm_min_pu";
          break;
        }

        record_state(corrected);
        if (!lower_branch && corrected[layout.size()] < y_curr[layout.size()]) {
          lower_branch = true;
          result.nose_found = true;
          lower_branch_points = 1;
        } else if (lower_branch) {
          ++lower_branch_points;
        }

        y_prev = std::move(y_curr);
        y_curr = std::move(corrected);
        step = std::min(step * opts.step_grow, opts.step_max);

        if (lower_branch && lower_branch_points >= std::max(1, opts.lower_branch_steps)) {
          result.termination_reason = "nose point passed by arc-length continuation";
          break;
        }
      }
    }
  }

  if (result.termination_reason.empty())
    result.termination_reason = "max_steps reached";

  // ── Fill summary fields ───────────────────────────────────────────────────
  if (!result.trace.empty()) {
    const auto nose = std::max_element(
        result.trace.begin(), result.trace.end(),
        [](const CpfPoint& a, const CpfPoint& b) { return a.lambda < b.lambda; });
    result.lambda_max   = nose->lambda;
    result.p_max_mw     = nose->p_total_mw;
    result.vm_at_nose   = nose->vm_monitor;
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
