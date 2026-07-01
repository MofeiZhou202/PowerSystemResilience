#include "hacdcpf/dynamics/DynamicSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include <Eigen/Dense>

#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"

namespace hacdcpf::dynamics {
namespace {

DynamicSnapshot make_snapshot(const DynamicSystem& sys) {
  DynamicSnapshot s;
  s.time_s = sys.x.time_s;
  s.state = sys.x.x;
  s.vac_abc = sys.y.Vac_abc;
  s.vdc = sys.y.Vdc;
  s.frequency_hz = sys.network.frequency_hz;

  s.max_ac_voltage_pu = 0.0;
  s.min_ac_voltage_pu = sys.y.Vac_abc.size() > 0
                            ? std::numeric_limits<double>::infinity()
                            : 0.0;
  for (Eigen::Index i = 0; i < sys.y.Vac_abc.size(); ++i) {
    const double v = std::abs(sys.y.Vac_abc[i]);
    s.max_ac_voltage_pu = std::max(s.max_ac_voltage_pu, v);
    s.min_ac_voltage_pu = std::min(s.min_ac_voltage_pu, v);
  }
  if (!std::isfinite(s.min_ac_voltage_pu)) s.min_ac_voltage_pu = 0.0;

  s.max_dc_voltage_pu = 0.0;
  s.min_dc_voltage_pu = sys.y.Vdc.size() > 0
                            ? std::numeric_limits<double>::infinity()
                            : 0.0;
  for (Eigen::Index i = 0; i < sys.y.Vdc.size(); ++i) {
    const double v = std::abs(sys.y.Vdc[i]);
    s.max_dc_voltage_pu = std::max(s.max_dc_voltage_pu, v);
    s.min_dc_voltage_pu = std::min(s.min_dc_voltage_pu, v);
  }
  if (!std::isfinite(s.min_dc_voltage_pu)) s.min_dc_voltage_pu = 0.0;

  s.device_outputs.reserve(sys.devices.size());
  for (const auto& device : sys.devices) {
    s.device_outputs.push_back(device->output(sys.x, sys.y));
  }
  return s;
}

std::string event_label(const DynamicEvent& event) {
  if (!event.label.empty()) return event.label;
  std::ostringstream os;
  os << "event@" << event.time_s << " component=" << event.component_index;
  return os.str();
}

bool evaluate_derivatives(DynamicSystem& sys,
                          double t,
                          const Eigen::VectorXd& state,
                          Eigen::VectorXd& dxdt,
                          std::string& error) {
  return sys.evaluateDerivatives(t, state, dxdt, error);
}

bool clear_expired_faults(DynamicSystem& sys, double t) {
  bool changed = false;
  for (auto& fault : sys.network.fault_shunts) {
    if (!fault.active || fault.clear_time_s <= 0.0) continue;
    if (t + 1e-12 >= fault.clear_time_s) {
      fault.active = false;
      changed = true;
    }
  }
  return changed;
}

double next_discontinuity_time(const DynamicSystem& sys, double t) {
  double next = std::numeric_limits<double>::infinity();
  for (const auto& event : sys.events) {
    if (!event.applied && event.time_s > t + 1e-12) {
      next = std::min(next, event.time_s);
    }
  }
  for (const auto& fault : sys.network.fault_shunts) {
    if (fault.active && fault.clear_time_s > t + 1e-12) {
      next = std::min(next, fault.clear_time_s);
    }
  }
  return next;
}

bool apply_event_to_network(DynamicSystem& sys, const DynamicEvent& event) {
  bool rebuild = false;
  switch (event.type) {
    case DynamicEventType::ACBranchTrip:
    case DynamicEventType::ACBranchClose:
      for (auto& branch : sys.network.ac_branches) {
        if (branch.index == event.component_index) {
          branch.in_service = event.type == DynamicEventType::ACBranchClose;
          rebuild = true;
        }
      }
      break;
    case DynamicEventType::DCBranchTrip:
    case DynamicEventType::DCBranchClose:
      for (auto& branch : sys.network.dc_branches) {
        if (branch.index == event.component_index) {
          branch.in_service = event.type == DynamicEventType::DCBranchClose;
          rebuild = true;
        }
      }
      break;
    case DynamicEventType::FaultShunt: {
      const bool is_dc = event.component_type == "DC" || event.component_type == "dc";
      DynamicFaultShunt fault;
      fault.is_ac = !is_dc;
      fault.bus = event.bus != 0 ? event.bus : event.component_index;
      fault.bus_pos = is_dc ? sys.network.dcBusPosition(fault.bus)
                            : sys.network.acBusPosition(fault.bus);
      fault.g_pu = event.value > 0.0
                       ? event.value
                       : 1.0 / std::max(1e-9, sys.options.min_branch_impedance_pu);
      fault.b_pu = 0.0;
      fault.active = fault.bus_pos >= 0;
      fault.clear_time_s = event.duration_s > 0.0 ? event.time_s + event.duration_s : 0.0;
      if (fault.active) {
        sys.network.fault_shunts.push_back(fault);
        rebuild = true;
      }
      break;
    }
    case DynamicEventType::ClearFault: {
      const int bus = event.bus != 0 ? event.bus : event.component_index;
      for (auto& fault : sys.network.fault_shunts) {
        if (bus == 0 || fault.bus == bus) {
          fault.active = false;
          rebuild = true;
        }
      }
      break;
    }
    default:
      break;
  }
  return rebuild;
}

bool apply_events(DynamicSystem& sys, double t, DynamicResults& results) {
  bool rebuild = clear_expired_faults(sys, t);
  bool changed = rebuild;
  for (auto& event : sys.events) {
    if (event.applied || event.time_s > t + 1e-12) continue;
    rebuild = apply_event_to_network(sys, event) || rebuild;
    for (auto& device : sys.devices) {
      device->handleEvent(event, sys.x, sys.y);
    }
    event.applied = true;
    changed = true;
    results.applied_events.push_back(event_label(event));
  }
  if (rebuild) {
    sys.network.rebuildBaseMatrices(sys.options.singular_regularization_pu);
  }
  return changed;
}

DynamicSolverType effective_solver_type(const DynamicSolverOptions& options,
                                        DynamicResults& results) {
  if (options.use_analytic_jacobian) {
    results.warnings.push_back(
        "Analytic dynamic-device Jacobians are not complete yet; transient Newton "
        "solvers used numerical state Jacobians");
  }
  return options.solver_type;
}

struct StepOutcome {
  bool ok{false};
  int iterations{0};
};

StepOutcome step_euler(DynamicSystem& sys, double t, double dt, std::string& error) {
  Eigen::VectorXd k1;
  if (!evaluate_derivatives(sys, t, sys.x.x, k1, error)) return {};
  sys.x.x += dt * k1;
  sys.x.dxdt = k1;
  return {sys.solveNetwork(t + dt, error), 1};
}

StepOutcome step_heun(DynamicSystem& sys, double t, double dt, std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  Eigen::VectorXd k1;
  if (!evaluate_derivatives(sys, t, x0, k1, error)) return {};
  Eigen::VectorXd k2;
  if (!evaluate_derivatives(sys, t + dt, x0 + dt * k1, k2, error)) return {};
  sys.x.x = x0 + 0.5 * dt * (k1 + k2);
  sys.x.dxdt = 0.5 * (k1 + k2);
  return {sys.solveNetwork(t + dt, error), 2};
}

StepOutcome step_rk4(DynamicSystem& sys, double t, double dt, std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  Eigen::VectorXd k1, k2, k3, k4;
  if (!evaluate_derivatives(sys, t, x0, k1, error)) return {};
  if (!evaluate_derivatives(sys, t + 0.5 * dt, x0 + 0.5 * dt * k1, k2, error)) return {};
  if (!evaluate_derivatives(sys, t + 0.5 * dt, x0 + 0.5 * dt * k2, k3, error)) return {};
  if (!evaluate_derivatives(sys, t + dt, x0 + dt * k3, k4, error)) return {};
  sys.x.x = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  sys.x.dxdt = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
  return {sys.solveNetwork(t + dt, error), 4};
}

bool numerical_state_jacobian(DynamicSystem& sys,
                              double t,
                              const Eigen::VectorXd& state,
                              const Eigen::VectorXd& f0,
                              Eigen::MatrixXd& jac,
                              std::string& error) {
  const Eigen::Index n = state.size();
  jac = Eigen::MatrixXd::Zero(n, n);
  if (n == 0) return true;
  const double eps0 = std::sqrt(std::numeric_limits<double>::epsilon());
  for (Eigen::Index col = 0; col < n; ++col) {
    Eigen::VectorXd perturbed = state;
    const double h = eps0 * std::max(1.0, std::abs(state[col]));
    perturbed[col] += h;
    Eigen::VectorXd fp;
    if (!evaluate_derivatives(sys, t, perturbed, fp, error)) return false;
    jac.col(col) = (fp - f0) / h;
  }
  return true;
}

StepOutcome step_backward_euler_newton(DynamicSystem& sys,
                                       double t,
                                       double dt,
                                       std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  Eigen::VectorXd f0;
  if (!evaluate_derivatives(sys, t, x0, f0, error)) return {};
  Eigen::VectorXd x = x0 + dt * f0;
  Eigen::VectorXd f = f0;
  Eigen::VectorXd residual = x - x0 - dt * f;
  int iter = 0;

  for (; iter < sys.options.max_newton_iters; ++iter) {
    if (!evaluate_derivatives(sys, t + dt, x, f, error)) return {};
    residual = x - x0 - dt * f;
    const double res_norm = residual.lpNorm<Eigen::Infinity>();
    if (res_norm <= sys.options.newton_tol) {
      sys.x.x = x;
      sys.x.dxdt = f;
      return {sys.solveNetwork(t + dt, error), iter + 1};
    }
    Eigen::MatrixXd jf;
    if (!numerical_state_jacobian(sys, t + dt, x, f, jf, error)) return {};
    Eigen::MatrixXd jr = Eigen::MatrixXd::Identity(x.size(), x.size()) - dt * jf;
    Eigen::VectorXd delta = jr.colPivHouseholderQr().solve(-residual);
    if (!delta.allFinite()) {
      error = "Backward Euler Newton produced a non-finite correction";
      return {};
    }
    double alpha = 1.0;
    bool accepted = false;
    const double min_alpha = std::max(1e-9, sys.options.newton_damping_min);
    while (alpha >= min_alpha) {
      const Eigen::VectorXd trial = x + alpha * delta;
      Eigen::VectorXd f_trial;
      if (!evaluate_derivatives(sys, t + dt, trial, f_trial, error)) return {};
      const Eigen::VectorXd r_trial = trial - x0 - dt * f_trial;
      if (r_trial.lpNorm<Eigen::Infinity>() <=
          (1.0 - 1e-4 * alpha) * std::max(res_norm, sys.options.newton_tol)) {
        x = trial;
        f = f_trial;
        residual = r_trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) {
      error = "Backward Euler Newton line search failed";
      return {};
    }
  }
  error = "Backward Euler Newton did not converge";
  return {};
}

StepOutcome step_trapezoidal_newton(DynamicSystem& sys,
                                    double t,
                                    double dt,
                                    std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  Eigen::VectorXd f0;
  if (!evaluate_derivatives(sys, t, x0, f0, error)) return {};
  Eigen::VectorXd x = x0 + dt * f0;
  Eigen::VectorXd f = f0;
  Eigen::VectorXd residual = x - x0 - 0.5 * dt * (f0 + f);
  int iter = 0;

  for (; iter < sys.options.max_newton_iters; ++iter) {
    if (!evaluate_derivatives(sys, t + dt, x, f, error)) return {};
    residual = x - x0 - 0.5 * dt * (f0 + f);
    const double res_norm = residual.lpNorm<Eigen::Infinity>();
    if (res_norm <= sys.options.newton_tol) {
      sys.x.x = x;
      sys.x.dxdt = f;
      return {sys.solveNetwork(t + dt, error), iter + 1};
    }
    Eigen::MatrixXd jf;
    if (!numerical_state_jacobian(sys, t + dt, x, f, jf, error)) return {};
    Eigen::MatrixXd jr = Eigen::MatrixXd::Identity(x.size(), x.size()) - 0.5 * dt * jf;
    Eigen::VectorXd delta = jr.colPivHouseholderQr().solve(-residual);
    if (!delta.allFinite()) {
      error = "Trapezoidal Newton produced a non-finite correction";
      return {};
    }
    double alpha = 1.0;
    bool accepted = false;
    const double min_alpha = std::max(1e-9, sys.options.newton_damping_min);
    while (alpha >= min_alpha) {
      const Eigen::VectorXd trial = x + alpha * delta;
      Eigen::VectorXd f_trial;
      if (!evaluate_derivatives(sys, t + dt, trial, f_trial, error)) return {};
      const Eigen::VectorXd r_trial = trial - x0 - 0.5 * dt * (f0 + f_trial);
      if (r_trial.lpNorm<Eigen::Infinity>() <=
          (1.0 - 1e-4 * alpha) * std::max(res_norm, sys.options.newton_tol)) {
        x = trial;
        f = f_trial;
        residual = r_trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) {
      error = "Trapezoidal Newton line search failed";
      return {};
    }
  }
  error = "Trapezoidal Newton did not converge";
  return {};
}

StepOutcome step_rosenbrock_euler(DynamicSystem& sys,
                                  double t,
                                  double dt,
                                  std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  Eigen::VectorXd f0;
  if (!evaluate_derivatives(sys, t, x0, f0, error)) return {};
  Eigen::MatrixXd jf;
  if (!numerical_state_jacobian(sys, t, x0, f0, jf, error)) return {};
  const Eigen::MatrixXd a =
      Eigen::MatrixXd::Identity(x0.size(), x0.size()) - dt * jf;
  const Eigen::VectorXd delta = a.colPivHouseholderQr().solve(dt * f0);
  if (!delta.allFinite()) {
    error = "Rosenbrock-Euler produced a non-finite correction";
    return {};
  }
  sys.x.x = x0 + delta;
  Eigen::VectorXd f1;
  if (!evaluate_derivatives(sys, t + dt, sys.x.x, f1, error)) return {};
  sys.x.dxdt = f1;
  return {sys.solveNetwork(t + dt, error), 1};
}

}  // namespace

DynamicResults DynamicSolver::solve(DynamicSystem& system) const {
  DynamicResults results;
  results.initialization = system.initialization;
  results.warnings.insert(results.warnings.end(),
                          system.warnings.begin(),
                          system.warnings.end());

  if (system.options.dt_s <= 0.0) {
    results.success = false;
    results.message = "DynamicSolverOptions::dt_s must be positive";
    return results;
  }
  if (system.options.t_end_s < system.options.t_start_s) {
    results.success = false;
    results.message = "DynamicSolverOptions::t_end_s is before t_start_s";
    return results;
  }

  std::sort(system.events.begin(), system.events.end(),
            [](const DynamicEvent& a, const DynamicEvent& b) {
              return a.time_s < b.time_s;
            });

  system.x.time_s = system.options.t_start_s;
  std::string error;
  apply_events(system, system.x.time_s, results);
  if (!system.solveNetwork(system.x.time_s, error)) {
    results.success = false;
    results.message = error;
    results.failed_step = 0;
    return results;
  }

  if (system.options.record_every_step) {
    results.snapshots.push_back(make_snapshot(system));
  }

  const DynamicSolverType solver_type = effective_solver_type(system.options, results);
  const double t_end = system.options.t_end_s;
  const double base_dt = system.options.dt_s;
  int step = 0;

  while (system.x.time_s < t_end - 1e-12) {
    const double t = system.x.time_s;
    if (apply_events(system, t, results) && !system.solveNetwork(t, error)) {
      results.success = false;
      results.message = error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    const double next_discontinuity = next_discontinuity_time(system, t);
    double next_t = std::min(t + base_dt, t_end);
    if (std::isfinite(next_discontinuity)) {
      next_t = std::min(next_t, next_discontinuity);
    }
    if (next_t <= t + 1e-12) {
      next_t = std::min(t + base_dt, t_end);
    }
    const double dt = next_t - t;
    const Eigen::VectorXd x_before = system.x.x;
    const NetworkState y_before = system.y;

    StepOutcome outcome;
    double attempted_dt = dt;
    double accepted_next_t = next_t;
    for (int retry = 0; retry <= system.options.max_step_halving; ++retry) {
      system.x.x = x_before;
      system.y = y_before;
      error.clear();
      switch (solver_type) {
        case DynamicSolverType::PartitionedEuler:
          outcome = step_euler(system, t, attempted_dt, error);
          break;
        case DynamicSolverType::PartitionedRK4:
          outcome = step_rk4(system, t, attempted_dt, error);
          break;
        case DynamicSolverType::PartitionedHeun:
          outcome = step_heun(system, t, attempted_dt, error);
          break;
        case DynamicSolverType::BackwardEulerNewton:
          outcome = step_backward_euler_newton(system, t, attempted_dt, error);
          break;
        case DynamicSolverType::TrapezoidalNewton:
          outcome = step_trapezoidal_newton(system, t, attempted_dt, error);
          break;
        case DynamicSolverType::RosenbrockEuler:
          outcome = step_rosenbrock_euler(system, t, attempted_dt, error);
          break;
      }
      if (outcome.ok) {
        accepted_next_t = t + attempted_dt;
        break;
      }
      if (!system.options.use_adaptive_step ||
          retry == system.options.max_step_halving ||
          std::isfinite(next_discontinuity)) {
        break;
      }
      attempted_dt *= 0.5;
      results.rejected_steps += 1;
    }

    ++step;
    system.x.time_s = accepted_next_t;
    results.newton_iterations += outcome.iterations;
    if (!outcome.ok) {
      results.success = false;
      results.message = error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }

    if (apply_events(system, system.x.time_s, results) &&
        !system.solveNetwork(system.x.time_s, error)) {
      results.success = false;
      results.message = error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    if (system.options.record_every_step) {
      results.snapshots.push_back(make_snapshot(system));
    }
  }

  if (!system.options.record_every_step) {
    results.snapshots.push_back(make_snapshot(system));
  }
  results.success = true;
  results.message = "Transient simulation completed";
  results.steps = step;
  return results;
}

DynamicResults run_transient_simulation(const HybridPowerSystem& sys,
                                        const DynamicSolverOptions& options) {
  DynamicModelBuilder builder;
  DynamicSystem dynamic_system = builder.build(sys, options);
  DynamicSolver solver;
  return solver.solve(dynamic_system);
}

}  // namespace hacdcpf::dynamics
