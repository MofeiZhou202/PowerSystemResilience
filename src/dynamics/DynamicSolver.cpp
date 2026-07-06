#include "hacdcpf/dynamics/DynamicSolver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"

namespace hacdcpf::dynamics {
namespace {

DynamicSnapshot make_snapshot(const DynamicSystem& sys, bool include_device_outputs) {
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

  if (include_device_outputs) {
    s.device_outputs.reserve(sys.devices.size());
    for (const auto& device : sys.devices) {
      s.device_outputs.push_back(device->output(sys.x, sys.y));
    }
  }
  return s;
}

std::string event_label(const DynamicEvent& event) {
  if (!event.label.empty()) return event.label;
  std::ostringstream os;
  os << "event@" << event.time_s << " component=" << event.component_index;
  return os.str();
}

std::string event_type_name(DynamicEventType type) {
  switch (type) {
    case DynamicEventType::ACBranchTrip: return "ACBranchTrip";
    case DynamicEventType::ACBranchClose: return "ACBranchClose";
    case DynamicEventType::ACBranchImpedanceScale: return "ACBranchImpedanceScale";
    case DynamicEventType::DCBranchTrip: return "DCBranchTrip";
    case DynamicEventType::DCBranchClose: return "DCBranchClose";
    case DynamicEventType::ACLoadScale: return "ACLoadScale";
    case DynamicEventType::DCLoadScale: return "DCLoadScale";
    case DynamicEventType::GeneratorTrip: return "GeneratorTrip";
    case DynamicEventType::VSCTrip: return "VSCTrip";
    case DynamicEventType::DCDCTrip: return "DCDCTrip";
    case DynamicEventType::StoragePowerStep: return "StoragePowerStep";
    case DynamicEventType::DCStoragePowerStep: return "DCStoragePowerStep";
    case DynamicEventType::FaultShunt: return "FaultShunt";
    case DynamicEventType::ClearFault: return "ClearFault";
    case DynamicEventType::Custom: return "Custom";
  }
  return "Custom";
}

DynamicAppliedEventRecord event_record(const DynamicEvent& event, double applied_time_s) {
  DynamicAppliedEventRecord record;
  record.time_s = applied_time_s;
  record.type = event_type_name(event.type);
  record.label = event_label(event);
  record.component_index = event.component_index;
  record.target_id = event.target_id;
  record.bus = event.bus;
  record.phase = event.phase;
  record.value = event.value;
  record.duration_s = event.duration_s;
  record.component_type = event.component_type;
  record.target_type = event.target_type;
  record.params = event.params;
  return record;
}

double event_param(const DynamicEvent& event,
                   const std::string& key,
                   double fallback) {
  const auto it = event.params.find(key);
  return it == event.params.end() ? fallback : it->second;
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
    case DynamicEventType::ACBranchImpedanceScale: {
      const double scale = event_param(event, "scale", event.value > 0.0 ? event.value : 1.0);
      const double r_scale = event_param(event, "r_scale", scale);
      const double x_scale = event_param(event, "x_scale", scale);
      const double y_scale =
          1.0 / std::max(1e-9, std::max(std::abs(r_scale), std::abs(x_scale)));
      for (auto& branch : sys.network.ac_branches) {
        if (branch.index == event.component_index) {
          branch.y_ff *= y_scale;
          branch.y_ft *= y_scale;
          branch.y_tf *= y_scale;
          branch.y_tt *= y_scale;
          rebuild = true;
        }
      }
      break;
    }
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
      fault.bus = event.bus != 0 ? event.bus : (event.target_id != 0 ? event.target_id
                                                                       : event.component_index);
      fault.bus_pos = is_dc ? sys.network.dcBusPosition(fault.bus)
                            : sys.network.acBusPosition(fault.bus);
      const double r_pu = event_param(event, "r_pu", 0.0);
      const double x_pu = event_param(event, "x_pu", 0.0);
      if (r_pu > 0.0 || x_pu != 0.0) {
        const std::complex<double> z(std::max(0.0, r_pu), x_pu);
        const std::complex<double> y =
            std::abs(z) > 0.0
                ? std::complex<double>(1.0, 0.0) / z
                : std::complex<double>(1.0 / std::max(1e-9, sys.options.min_branch_impedance_pu),
                                       0.0);
        fault.g_pu = y.real();
        fault.b_pu = y.imag();
      } else {
        fault.g_pu = event_param(
            event,
            "g_pu",
            event.value > 0.0
                ? event.value
                : 1.0 / std::max(1e-9, sys.options.min_branch_impedance_pu));
        fault.b_pu = event_param(event, "b_pu", 0.0);
      }
      fault.active = fault.bus_pos >= 0;
      const double duration = event_param(event, "duration_s", event.duration_s);
      fault.clear_time_s = duration > 0.0 ? event.time_s + duration : 0.0;
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
    case DynamicEventType::ACLoadScale: {
      for (auto& load : sys.network.ac_bus_loads) {
        if (event.bus != 0 && load.bus != event.bus) continue;
        if (event.bus == 0 && event.component_index != 0 && load.bus != event.component_index) continue;
        load.scale = std::max(0.0, event_param(event, "scale", event.value));
        rebuild = true;
      }
      break;
    }
    case DynamicEventType::DCLoadScale: {
      for (auto& load : sys.network.dc_bus_loads) {
        if (event.bus != 0 && load.bus != event.bus) continue;
        if (event.bus == 0 && event.component_index != 0 && load.bus != event.component_index) continue;
        load.scale = std::max(0.0, event_param(event, "scale", event.value));
        rebuild = true;
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
    results.applied_event_records.push_back(event_record(event, t));
  }
  if (rebuild) {
    sys.network.rebuildBaseMatrices(sys.options.singular_regularization_pu);
    sys.network_cache.reset();
  }
  return changed;
}

bool should_record_snapshot(const DynamicSystem& system,
                            const DynamicResults& results,
                            int step,
                            double t) {
  const auto& opt = system.options;
  if (!opt.record_every_step) return false;
  if (opt.max_recorded_snapshots > 0 &&
      static_cast<int>(results.snapshots.size()) >= opt.max_recorded_snapshots) {
    return false;
  }
  const int stride = std::max(1, opt.output_every_steps);
  if (step == 0) return opt.record_initial_state;
  if (step % stride == 0) return true;
  if (opt.output_interval_s > 0.0) {
    const double k = std::round((t - opt.t_start_s) / opt.output_interval_s);
    const double target = opt.t_start_s + k * opt.output_interval_s;
    if (std::abs(t - target) <= 1e-9 * std::max(1.0, std::abs(t))) return true;
  }
  return false;
}

void record_snapshot_if_needed(DynamicSystem& system,
                               DynamicResults& results,
                               int step) {
  if (should_record_snapshot(system, results, step, system.x.time_s)) {
    results.snapshots.push_back(
        make_snapshot(system, system.options.record_device_outputs));
  }
}

DynamicSolverType effective_solver_type(const DynamicSolverOptions& options,
                                        DynamicResults& results) {
  if (options.use_analytic_jacobian) {
    results.warnings.push_back(
        "MassMatrixDae uses the selected DAE Jacobian mode; finite-difference "
        "fallback remains available for uncovered dynamic-device couplings");
  }
  return options.solver_type;
}

struct StepOutcome {
  bool ok{false};
  int iterations{0};
  int jacobian_evaluations{0};
  int jacobian_residual_evaluations{0};
  int jacobian_fd_columns{0};
  int jacobian_colored_groups{0};
  int linear_factorizations{0};
  double error_norm{0.0};
  bool error_estimated{false};
};

double scaled_error_norm(const Eigen::VectorXd& reference,
                         const Eigen::VectorXd& trial,
                         const Eigen::VectorXd& error,
                         const DynamicSolverOptions& opt) {
  if (error.size() == 0) return 0.0;
  double norm = 0.0;
  const double abs_tol = std::max(0.0, opt.abs_tol);
  const double rel_tol = std::max(0.0, opt.rel_tol);
  for (Eigen::Index i = 0; i < error.size(); ++i) {
    const double scale =
        abs_tol + rel_tol * std::max(std::abs(reference[i]), std::abs(trial[i]));
    norm = std::max(norm, std::abs(error[i]) / std::max(scale, 1e-16));
  }
  return norm;
}

Eigen::SparseMatrix<double> dense_to_sparse(const Eigen::MatrixXd& dense) {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(dense.size()));
  for (Eigen::Index j = 0; j < dense.cols(); ++j) {
    for (Eigen::Index i = 0; i < dense.rows(); ++i) {
      const double v = dense(i, j);
      if (v != 0.0) triplets.emplace_back(i, j, v);
    }
  }
  Eigen::SparseMatrix<double> sparse(dense.rows(), dense.cols());
  sparse.setFromTriplets(triplets.begin(), triplets.end());
  return sparse;
}

bool sparse_solve_dense_system(const Eigen::MatrixXd& dense,
                               const Eigen::VectorXd& rhs,
                               Eigen::VectorXd& x,
                               std::string& error) {
  Eigen::SparseMatrix<double> sparse = dense_to_sparse(dense);
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
  lu.compute(sparse);
  if (lu.info() != Eigen::Success) {
    error = "Sparse Newton correction factorization failed";
    return false;
  }
  x = lu.solve(rhs);
  if (lu.info() != Eigen::Success || !x.allFinite()) {
    error = "Sparse Newton correction solve failed";
    return false;
  }
  return true;
}

bool has_active_fault(const DynamicSystem& sys) {
  return std::any_of(sys.network.fault_shunts.begin(),
                     sys.network.fault_shunts.end(),
                     [](const DynamicFaultShunt& fault) { return fault.active; });
}

bool check_numerical_health(const DynamicSystem& sys,
                            double t,
                            std::string& error) {
  const auto& opt = sys.options;
  if (!sys.x.x.allFinite()) {
    error = "Transient state contains non-finite values at t=" + std::to_string(t) + "s";
    return false;
  }
  if (!opt.enforce_voltage_health_check) return true;

  const bool skip_low_voltage =
      opt.allow_low_voltage_during_active_fault && has_active_fault(sys);
  if (sys.y.Vac_abc.size() > 0) {
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = 0.0;
    for (Eigen::Index i = 0; i < sys.y.Vac_abc.size(); ++i) {
      const double v = std::abs(sys.y.Vac_abc[i]);
      if (!std::isfinite(v)) {
        error = "AC transient voltage contains non-finite values at t=" +
                std::to_string(t) + "s";
        return false;
      }
      min_v = std::min(min_v, v);
      max_v = std::max(max_v, v);
    }
    if (!skip_low_voltage && min_v < opt.voltage_collapse_min_ac_pu) {
      error = "AC voltage health check failed at t=" + std::to_string(t) +
              "s; min |V|=" + std::to_string(min_v) + " pu < " +
              std::to_string(opt.voltage_collapse_min_ac_pu) + " pu";
      return false;
    }
    if (max_v > opt.voltage_blowup_max_ac_pu) {
      error = "AC voltage health check failed at t=" + std::to_string(t) +
              "s; max |V|=" + std::to_string(max_v) + " pu > " +
              std::to_string(opt.voltage_blowup_max_ac_pu) + " pu";
      return false;
    }
  }

  if (sys.y.Vdc.size() > 0) {
    double min_v = std::numeric_limits<double>::infinity();
    double max_v = 0.0;
    for (Eigen::Index i = 0; i < sys.y.Vdc.size(); ++i) {
      const double v = sys.y.Vdc[i];
      if (!std::isfinite(v)) {
        error = "DC transient voltage contains non-finite values at t=" +
                std::to_string(t) + "s";
        return false;
      }
      min_v = std::min(min_v, std::abs(v));
      max_v = std::max(max_v, std::abs(v));
    }
    if (!skip_low_voltage && min_v < opt.voltage_collapse_min_dc_pu) {
      error = "DC voltage health check failed at t=" + std::to_string(t) +
              "s; min |V|=" + std::to_string(min_v) + " pu < " +
              std::to_string(opt.voltage_collapse_min_dc_pu) + " pu";
      return false;
    }
    if (max_v > opt.voltage_blowup_max_dc_pu) {
      error = "DC voltage health check failed at t=" + std::to_string(t) +
              "s; max |V|=" + std::to_string(max_v) + " pu > " +
              std::to_string(opt.voltage_blowup_max_dc_pu) + " pu";
      return false;
    }
  }
  return true;
}

StepOutcome step_euler(DynamicSystem& sys, double t, double dt, std::string& error) {
  Eigen::VectorXd k1;
  const Eigen::VectorXd x0 = sys.x.x;
  if (!evaluate_derivatives(sys, t, x0, k1, error)) return {};
  StepOutcome outcome;
  outcome.iterations = 1;
  if (sys.options.use_adaptive_step && x0.size() > 0) {
    const Eigen::VectorXd x_full = x0 + dt * k1;
    const Eigen::VectorXd x_half = x0 + 0.5 * dt * k1;
    Eigen::VectorXd k_half;
    if (!evaluate_derivatives(sys, t + 0.5 * dt, x_half, k_half, error)) return {};
    const Eigen::VectorXd x_two_half = x_half + 0.5 * dt * k_half;
    outcome.iterations = 2;
    outcome.error_estimated = true;
    outcome.error_norm =
        scaled_error_norm(x0, x_two_half, x_two_half - x_full, sys.options);
    sys.x.x = x_two_half;
    sys.x.dxdt = (x_two_half - x0) / dt;
  } else {
    sys.x.x = x0 + dt * k1;
    sys.x.dxdt = k1;
  }
  outcome.ok = sys.solveNetwork(t + dt, error);
  return outcome;
}

StepOutcome step_heun(DynamicSystem& sys, double t, double dt, std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  Eigen::VectorXd k1;
  if (!evaluate_derivatives(sys, t, x0, k1, error)) return {};
  Eigen::VectorXd k2;
  if (!evaluate_derivatives(sys, t + dt, x0 + dt * k1, k2, error)) return {};
  const Eigen::VectorXd x_euler = x0 + dt * k1;
  const Eigen::VectorXd x_heun = x0 + 0.5 * dt * (k1 + k2);
  sys.x.x = x_heun;
  sys.x.dxdt = 0.5 * (k1 + k2);
  StepOutcome outcome;
  outcome.ok = sys.solveNetwork(t + dt, error);
  outcome.iterations = 2;
  outcome.error_estimated = x0.size() > 0;
  if (outcome.error_estimated) {
    outcome.error_norm =
        scaled_error_norm(x0, x_heun, x_heun - x_euler, sys.options);
  }
  return outcome;
}

bool rk4_state(DynamicSystem& sys,
               double t,
               const Eigen::VectorXd& x0,
               double dt,
               Eigen::VectorXd& x1,
               std::string& error) {
  Eigen::VectorXd k1, k2, k3, k4;
  if (!evaluate_derivatives(sys, t, x0, k1, error)) return false;
  if (!evaluate_derivatives(sys, t + 0.5 * dt, x0 + 0.5 * dt * k1, k2, error)) return false;
  if (!evaluate_derivatives(sys, t + 0.5 * dt, x0 + 0.5 * dt * k2, k3, error)) return false;
  if (!evaluate_derivatives(sys, t + dt, x0 + dt * k3, k4, error)) return false;
  x1 = x0 + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
  return true;
}

StepOutcome step_rk4(DynamicSystem& sys, double t, double dt, std::string& error) {
  const Eigen::VectorXd x0 = sys.x.x;
  StepOutcome outcome;
  Eigen::VectorXd x_full;
  if (!rk4_state(sys, t, x0, dt, x_full, error)) return {};
  outcome.iterations = 4;
  Eigen::VectorXd x_next = x_full;
  if (sys.options.use_adaptive_step && x0.size() > 0) {
    Eigen::VectorXd x_mid;
    if (!rk4_state(sys, t, x0, 0.5 * dt, x_mid, error)) return {};
    Eigen::VectorXd x_half;
    if (!rk4_state(sys, t + 0.5 * dt, x_mid, 0.5 * dt, x_half, error)) return {};
    outcome.iterations = 12;
    outcome.error_estimated = true;
    outcome.error_norm =
        scaled_error_norm(x0, x_half, (x_half - x_full) / 15.0, sys.options);
    x_next = x_half;
  }
  sys.x.x = x_next;
  if (dt > 0.0) {
    sys.x.dxdt = (x_next - x0) / dt;
  } else {
    sys.x.dxdt = Eigen::VectorXd::Zero(x0.size());
  }
  outcome.ok = sys.solveNetwork(t + dt, error);
  return outcome;
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
  Eigen::VectorXd restored;
  return evaluate_derivatives(sys, t, state, restored, error);
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
    Eigen::VectorXd delta;
    if (!sparse_solve_dense_system(jr, -residual, delta, error)) {
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
    Eigen::VectorXd delta;
    if (!sparse_solve_dense_system(jr, -residual, delta, error)) {
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
  Eigen::VectorXd delta;
  if (!sparse_solve_dense_system(a, dt * f0, delta, error)) {
    return {};
  }
  sys.x.x = x0 + delta;
  Eigen::VectorXd f1;
  if (!evaluate_derivatives(sys, t + dt, sys.x.x, f1, error)) return {};
  sys.x.dxdt = f1;
  return {sys.solveNetwork(t + dt, error), 1};
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase 1 — simultaneous mass-matrix DAE core
//
// Unknown vector u = [ device states x ; Re(V_ac) ; Im(V_ac) ; V_dc ].
// Backward-Euler residual (mass matrix M = 1 for device rows, 0 for network rows):
//   device rows:  (x − x_prev) − dt·f(x,V) = 0
//   network rows: g(x,V) = I_inj(x,V) − Y_eff·V = 0        (algebraic, no dt)
// One sparse Jacobian is factored per Newton iteration for the whole coupled
// system — there is no nested network solve inside the derivative evaluation.
// ─────────────────────────────────────────────────────────────────────────────
struct DaeLayout {
  int n_x{0};
  int n_ac{0};
  int n_dc{0};
  int ac_off{0};
  int ai_off{0};
  int dc_off{0};
  int n{0};
};

DaeLayout make_dae_layout(const DynamicSystem& sys) {
  DaeLayout L;
  L.n_x = sys.x.size();
  L.n_ac = sys.network.acPhaseNodeCount();
  L.n_dc = sys.network.dcBusCount();
  L.ac_off = L.n_x;
  L.ai_off = L.n_x + L.n_ac;
  L.dc_off = L.n_x + 2 * L.n_ac;
  L.n = L.n_x + 2 * L.n_ac + L.n_dc;
  return L;
}

DynamicJacobianContext make_dae_jacobian_context(const DaeLayout& L,
                                                 double dt,
                                                 double theta) {
  DynamicJacobianContext context;
  context.n_x = L.n_x;
  context.n_ac = L.n_ac;
  context.n_dc = L.n_dc;
  context.ac_real_offset = L.ac_off;
  context.ac_imag_offset = L.ai_off;
  context.dc_offset = L.dc_off;
  context.total_size = L.n;
  context.dt = dt;
  context.theta = theta;
  context.include_differential_derivatives = true;
  return context;
}

struct DaeJacobianAssemblySettings {
  bool use_analytic_network{false};
  bool use_analytic_device{false};
  bool use_colored_fd{false};
};

DaeJacobianAssemblySettings dae_jacobian_settings(const DynamicSolverOptions& options) {
  DaeJacobianAssemblySettings settings;
  if (options.dae_jacobian_mode == DynamicDaeJacobianMode::FiniteDifference) {
    return settings;
  }
  settings.use_analytic_network = options.dae_use_analytic_network_jacobian;
  settings.use_analytic_device = options.dae_use_analytic_device_jacobian;
  settings.use_colored_fd =
      options.dae_jacobian_mode == DynamicDaeJacobianMode::HybridAnalyticColored &&
      settings.use_analytic_network &&
      settings.use_analytic_device;
  return settings;
}

bool mass_matrix_needs_coupled_network_startup(const DynamicSystem& sys) {
  return std::any_of(sys.devices.begin(),
                     sys.devices.end(),
                     [](const std::unique_ptr<DynamicDevice>& device) {
                       return device->type() == "DynamicRLLine";
                     });
}

void dae_pack(const DynamicSystem& sys, const DaeLayout& L, Eigen::VectorXd& u) {
  u.resize(L.n);
  for (int i = 0; i < L.n_x; ++i) u[i] = sys.x.x[i];
  for (int i = 0; i < L.n_ac; ++i) {
    u[L.ac_off + i] = sys.y.Vac_abc[i].real();
    u[L.ai_off + i] = sys.y.Vac_abc[i].imag();
  }
  for (int i = 0; i < L.n_dc; ++i) u[L.dc_off + i] = sys.y.Vdc[i];
}

void dae_unpack(DynamicSystem& sys, const DaeLayout& L, const Eigen::VectorXd& u) {
  for (int i = 0; i < L.n_x; ++i) sys.x.x[i] = u[i];
  for (int i = 0; i < L.n_ac; ++i) {
    sys.y.Vac_abc[i] = std::complex<double>(u[L.ac_off + i], u[L.ai_off + i]);
  }
  for (int i = 0; i < L.n_dc; ++i) sys.y.Vdc[i] = u[L.dc_off + i];
}

// Assemble the effective network matrices/currents for the current (x, V).
void dae_assemble(DynamicSystem& sys, const DaeLayout& L, double t,
                  Eigen::SparseMatrix<std::complex<double>>& Yac, Eigen::VectorXcd& Iac,
                  Eigen::SparseMatrix<double>& Gdc, Eigen::VectorXd& Idc) {
  DynamicStamp stamp(L.n_ac, L.n_dc);
  for (const auto& device : sys.devices) device->stamp(t, sys.x, sys.y, stamp);
  sys.network.assembleEffectiveMatrices(stamp, sys.options.singular_regularization_pu,
                                        Yac, Iac, Gdc, Idc);
}

bool dae_compute_derivatives(DynamicSystem& sys,
                             const DaeLayout& L,
                             double t,
                             Eigen::VectorXd& f,
                             std::string& error) {
  f = Eigen::VectorXd::Zero(L.n_x);
  for (const auto& device : sys.devices) device->computeDerivatives(t, sys.x, sys.y, f);
  if (!f.allFinite()) {
    error = "DAE device derivatives produced a non-finite value";
    return false;
  }
  return true;
}

struct DaeStepFormula {
  double theta{1.0};
  const Eigen::VectorXd* f_prev{nullptr};
};

// R = [ (x−x_prev) − dt·((1−θ)·f_prev + θ·f) ; g_ac_re ; g_ac_im ; g_dc ].
bool dae_residual(DynamicSystem& sys, const DaeLayout& L, double t, double dt,
                  const Eigen::VectorXd& u, const Eigen::VectorXd& x_prev,
                  const DaeStepFormula& formula,
                  Eigen::VectorXd& R, std::string& error) {
  dae_unpack(sys, L, u);
  Eigen::SparseMatrix<std::complex<double>> Yac;
  Eigen::VectorXcd Iac;
  Eigen::SparseMatrix<double> Gdc;
  Eigen::VectorXd Idc;
  dae_assemble(sys, L, t, Yac, Iac, Gdc, Idc);
  Eigen::VectorXd f;
  if (!dae_compute_derivatives(sys, L, t, f, error)) return false;
  if (formula.theta < 1.0 &&
      (formula.f_prev == nullptr || formula.f_prev->size() != L.n_x)) {
    error = "DAE theta residual requires a previous derivative vector";
    return false;
  }

  R.resize(L.n);
  for (int i = 0; i < L.n_x; ++i) {
    const double f_weighted =
        formula.theta * f[i] +
        (1.0 - formula.theta) *
            (formula.f_prev != nullptr ? (*formula.f_prev)[i] : 0.0);
    R[i] = (u[i] - x_prev[i]) - dt * f_weighted;
  }
  if (L.n_ac > 0) {
    Eigen::VectorXcd Vac(L.n_ac);
    for (int i = 0; i < L.n_ac; ++i) {
      Vac[i] = std::complex<double>(u[L.ac_off + i], u[L.ai_off + i]);
    }
    const Eigen::VectorXcd g = Iac - Yac * Vac;
    for (int i = 0; i < L.n_ac; ++i) {
      R[L.ac_off + i] = g[i].real();
      R[L.ai_off + i] = g[i].imag();
    }
  }
  if (L.n_dc > 0) {
    Eigen::VectorXd Vdc(L.n_dc);
    for (int i = 0; i < L.n_dc; ++i) Vdc[i] = u[L.dc_off + i];
    const Eigen::VectorXd g = Idc - Gdc * Vdc;
    for (int i = 0; i < L.n_dc; ++i) R[L.dc_off + i] = g[i];
  }
  if (!R.allFinite()) {
    error = "DAE residual produced a non-finite value";
    return false;
  }
  return true;
}

struct DaeNewtonCache {
  Eigen::SparseMatrix<double> jacobian;
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
  bool valid{false};
  bool force_rebuild_next{false};
  int n{0};
  double dt{0.0};
  double theta{1.0};
  int accepted_steps_since_rebuild{0};

  void invalidate() {
    valid = false;
    force_rebuild_next = false;
    n = 0;
    dt = 0.0;
    theta = 1.0;
    accepted_steps_since_rebuild = 0;
    jacobian.resize(0, 0);
  }
};

bool dae_cache_usable(const DaeNewtonCache& cache,
                      const DaeLayout& L,
                      double dt,
                      const DaeStepFormula& formula,
                      const DynamicSolverOptions& options) {
  if (!options.dae_reuse_jacobian_factorization) return false;
  if (!cache.valid || cache.force_rebuild_next) return false;
  if (cache.n != L.n) return false;
  if (std::abs(cache.theta - formula.theta) > 1e-14) return false;
  if (std::abs(cache.dt - dt) >
      1e-12 * std::max({1.0, std::abs(cache.dt), std::abs(dt)})) {
    return false;
  }
  const int max_reuse_steps = std::max(0, options.dae_jacobian_max_reuse_steps);
  return cache.accepted_steps_since_rebuild < max_reuse_steps;
}

bool dae_is_algebraic_row(const DaeLayout& L, int row) {
  return row >= L.ac_off && row < L.n;
}

void dae_add_analytic_network_block(
    const DaeLayout& L,
    const Eigen::SparseMatrix<std::complex<double>>& Yac,
    const Eigen::SparseMatrix<double>& Gdc,
    std::vector<Eigen::Triplet<double>>& triplets) {
  for (int col = 0; col < Yac.outerSize(); ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(Yac, col); it; ++it) {
      const int row = static_cast<int>(it.row());
      const auto y = it.value();
      if (y.real() != 0.0) {
        triplets.emplace_back(L.ac_off + row, L.ac_off + col, -y.real());
        triplets.emplace_back(L.ai_off + row, L.ai_off + col, -y.real());
      }
      if (y.imag() != 0.0) {
        triplets.emplace_back(L.ac_off + row, L.ai_off + col, y.imag());
        triplets.emplace_back(L.ai_off + row, L.ac_off + col, -y.imag());
      }
    }
  }
  for (int col = 0; col < Gdc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(Gdc, col); it; ++it) {
      if (it.value() != 0.0) {
        triplets.emplace_back(L.dc_off + static_cast<int>(it.row()),
                              L.dc_off + col,
                              -it.value());
      }
    }
  }
}

void dae_add_mass_identity_block(const DaeLayout& L,
                                 std::vector<Eigen::Triplet<double>>& triplets) {
  for (int i = 0; i < L.n_x; ++i) triplets.emplace_back(i, i, 1.0);
}

std::vector<int> dae_dense_row_pattern(const DaeLayout& L) {
  std::vector<int> rows;
  rows.reserve(static_cast<std::size_t>(L.n));
  for (int i = 0; i < L.n; ++i) rows.push_back(i);
  return rows;
}

std::vector<int> dae_column_pattern_from_analytic(
    const DaeLayout& L,
    const Eigen::SparseMatrix<double>& analytic_jacobian,
    int col) {
  std::vector<int> rows;
  if (analytic_jacobian.nonZeros() > 0) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(analytic_jacobian, col); it; ++it) {
      rows.push_back(static_cast<int>(it.row()));
    }
  }
  std::sort(rows.begin(), rows.end());
  rows.erase(std::unique(rows.begin(), rows.end()), rows.end());

  if (rows.empty()) return dae_dense_row_pattern(L);

  if (col < L.n_x && rows.size() == 1 && rows.front() == col &&
      std::abs(analytic_jacobian.coeff(col, col) - 1.0) <= 1e-12) {
    return dae_dense_row_pattern(L);
  }
  return rows;
}

struct DaeFdColorGroup {
  std::vector<int> columns;
  std::vector<char> occupied_rows;
};

std::vector<DaeFdColorGroup> dae_color_fd_columns(
    const DaeLayout& L,
    const std::vector<std::vector<int>>& patterns) {
  std::vector<DaeFdColorGroup> groups;
  for (int col = 0; col < L.n; ++col) {
    const auto& pattern = patterns[static_cast<std::size_t>(col)];
    bool placed = false;
    for (auto& group : groups) {
      bool intersects = false;
      for (int row : pattern) {
        if (row >= 0 && row < L.n && group.occupied_rows[static_cast<std::size_t>(row)]) {
          intersects = true;
          break;
        }
      }
      if (intersects) continue;
      group.columns.push_back(col);
      for (int row : pattern) {
        if (row >= 0 && row < L.n) group.occupied_rows[static_cast<std::size_t>(row)] = 1;
      }
      placed = true;
      break;
    }
    if (!placed) {
      DaeFdColorGroup group;
      group.columns.push_back(col);
      group.occupied_rows.assign(static_cast<std::size_t>(L.n), 0);
      for (int row : pattern) {
        if (row >= 0 && row < L.n) group.occupied_rows[static_cast<std::size_t>(row)] = 1;
      }
      groups.push_back(std::move(group));
    }
  }
  return groups;
}

bool dae_build_and_factor_jacobian(DynamicSystem& sys,
                                   const DaeLayout& L,
                                   double t,
                                   double dt,
                                   const Eigen::VectorXd& u,
                                   const Eigen::VectorXd& x_prev,
                                   const DaeStepFormula& formula,
                                   const Eigen::VectorXd& R,
                                   DaeNewtonCache& cache,
                                   StepOutcome& outcome,
                                   std::string& error) {
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<std::size_t>(L.n) * 8);
  Eigen::SparseMatrix<std::complex<double>> Yac;
  Eigen::VectorXcd Iac;
  Eigen::SparseMatrix<double> Gdc;
  Eigen::VectorXd Idc;
  dae_unpack(sys, L, u);
  dae_assemble(sys, L, t, Yac, Iac, Gdc, Idc);
  const DaeJacobianAssemblySettings settings = dae_jacobian_settings(sys.options);
  const bool analytic_mode =
      sys.options.dae_jacobian_mode != DynamicDaeJacobianMode::FiniteDifference;
  std::vector<Eigen::Triplet<double>> analytic_triplets;
  analytic_triplets.reserve(static_cast<std::size_t>(L.n) * 6);
  if (analytic_mode) {
    dae_add_mass_identity_block(L, analytic_triplets);
  }
  if (settings.use_analytic_network) {
    dae_add_analytic_network_block(L, Yac, Gdc, analytic_triplets);
  }
  if (settings.use_analytic_device) {
    std::vector<Eigen::Triplet<double>> device_triplets;
    device_triplets.reserve(static_cast<std::size_t>(sys.devices.size()) * 16);
    const DynamicJacobianContext context =
        make_dae_jacobian_context(L, dt, formula.theta);
    for (const auto& device : sys.devices) {
      device->addJacobian(t, sys.x, sys.y, context, device_triplets);
    }
    analytic_triplets.insert(analytic_triplets.end(),
                             device_triplets.begin(),
                             device_triplets.end());
  }

  Eigen::SparseMatrix<double> analytic_jacobian(L.n, L.n);
  if (!analytic_triplets.empty()) {
    analytic_jacobian.setFromTriplets(analytic_triplets.begin(),
                                      analytic_triplets.end());
    analytic_jacobian.makeCompressed();
    triplets.insert(triplets.end(), analytic_triplets.begin(), analytic_triplets.end());
  }

  const auto add_fd_remainder_value = [&](int row, int col, double d) {
    const double analytic_d =
        analytic_jacobian.nonZeros() > 0 ? analytic_jacobian.coeff(row, col) : 0.0;
    d -= analytic_d;
    if (!sys.options.dae_use_fd_current_jacobian_corrections &&
        dae_is_algebraic_row(L, row) &&
        std::abs(analytic_d) > 0.0) {
      return;
    }
    if (std::abs(d) > 1e-14) triplets.emplace_back(row, col, d);
  };

  Eigen::VectorXd rp;
  std::vector<double> h(static_cast<std::size_t>(L.n), 0.0);
  for (int j = 0; j < L.n; ++j) {
    h[static_cast<std::size_t>(j)] = eps * std::max(1.0, std::abs(u[j]));
  }

  if (!settings.use_colored_fd) {
    outcome.jacobian_fd_columns += L.n;
    outcome.jacobian_colored_groups += L.n;
    for (int j = 0; j < L.n; ++j) {
      Eigen::VectorXd up = u;
      up[j] += h[static_cast<std::size_t>(j)];
      if (!dae_residual(sys, L, t, dt, up, x_prev, formula, rp, error)) return false;
      ++outcome.jacobian_residual_evaluations;
      for (int i = 0; i < L.n; ++i) {
        const double d = (rp[i] - R[i]) / h[static_cast<std::size_t>(j)];
        add_fd_remainder_value(i, j, d);
      }
    }
  } else {
    std::vector<std::vector<int>> patterns(static_cast<std::size_t>(L.n));
    for (int j = 0; j < L.n; ++j) {
      patterns[static_cast<std::size_t>(j)] =
          dae_column_pattern_from_analytic(L, analytic_jacobian, j);
    }
    const std::vector<DaeFdColorGroup> groups = dae_color_fd_columns(L, patterns);
    outcome.jacobian_fd_columns += L.n;
    outcome.jacobian_colored_groups += static_cast<int>(groups.size());
    for (const auto& group : groups) {
      Eigen::VectorXd up = u;
      for (int col : group.columns) up[col] += h[static_cast<std::size_t>(col)];
      if (!dae_residual(sys, L, t, dt, up, x_prev, formula, rp, error)) return false;
      ++outcome.jacobian_residual_evaluations;
      for (int col : group.columns) {
        const double hj = h[static_cast<std::size_t>(col)];
        for (int row : patterns[static_cast<std::size_t>(col)]) {
          const double d = (rp[row] - R[row]) / hj;
          add_fd_remainder_value(row, col, d);
        }
      }
    }
  }
  dae_unpack(sys, L, u);

  cache.jacobian.resize(L.n, L.n);
  cache.jacobian.setFromTriplets(triplets.begin(), triplets.end());
  cache.jacobian.makeCompressed();
  cache.lu.compute(cache.jacobian);
  ++outcome.jacobian_evaluations;
  ++outcome.linear_factorizations;
  if (cache.lu.info() != Eigen::Success) {
    cache.invalidate();
    error = "DAE Jacobian factorization failed";
    return false;
  }
  cache.valid = true;
  cache.force_rebuild_next = false;
  cache.n = L.n;
  cache.dt = dt;
  cache.theta = formula.theta;
  cache.accepted_steps_since_rebuild = 0;
  return true;
}

// Refresh telemetry-facing algebraic outputs and stamped currents after a step.
void dae_finalize(DynamicSystem& sys, const DaeLayout& L, double t) {
  Eigen::SparseMatrix<std::complex<double>> Yac;
  Eigen::VectorXcd Iac;
  Eigen::SparseMatrix<double> Gdc;
  Eigen::VectorXd Idc;
  dae_assemble(sys, L, t, Yac, Iac, Gdc, Idc);
  if (L.n_ac > 0) sys.y.Iac_abc = Iac;
  if (L.n_dc > 0) sys.y.Idc = Idc;
  for (auto& device : sys.devices) device->updateAlgebraicOutputs(sys.x, sys.y);
}

// One theta-method DAE step with a modified-Newton sparse Jacobian/LU cache.
// Advances sys.x / sys.y to t+dt on success.
StepOutcome dae_theta_step(DynamicSystem& sys,
                           const DaeLayout& L,
                           double t,
                           double dt,
                           const Eigen::VectorXd& x_prev,
                           const DaeStepFormula& formula,
                           DaeNewtonCache& cache,
                           std::string& error) {
  StepOutcome outcome;
  Eigen::VectorXd u;
  dae_pack(sys, L, u);
  Eigen::VectorXd R;
  if (!dae_residual(sys, L, t + dt, dt, u, x_prev, formula, R, error)) return outcome;

  for (int iter = 0; iter < sys.options.max_newton_iters; ++iter) {
    const double res_norm = R.lpNorm<Eigen::Infinity>();
    if (res_norm <= sys.options.newton_tol) {
      dae_unpack(sys, L, u);
      outcome.ok = true;
      outcome.iterations = iter + 1;
      return outcome;
    }

    bool retried_with_fresh_jacobian = false;
    while (true) {
      bool rebuilt_for_direction = false;
      if (!dae_cache_usable(cache, L, dt, formula, sys.options)) {
        if (!dae_build_and_factor_jacobian(sys, L, t + dt, dt, u, x_prev,
                                           formula, R, cache, outcome, error)) {
          return outcome;
        }
        rebuilt_for_direction = true;
      }

      const Eigen::VectorXd du = cache.lu.solve(-R);
      if (cache.lu.info() != Eigen::Success || !du.allFinite()) {
        if (sys.options.dae_reuse_jacobian_factorization &&
            !rebuilt_for_direction && !retried_with_fresh_jacobian) {
          cache.invalidate();
          retried_with_fresh_jacobian = true;
          continue;
        }
        error = "DAE Newton produced a non-finite correction";
        return outcome;
      }

      double alpha = 1.0;
      bool accepted = false;
      Eigen::VectorXd accepted_R;
      const double min_alpha = std::max(1e-9, sys.options.newton_damping_min);
      while (alpha >= min_alpha) {
        const Eigen::VectorXd trial = u + alpha * du;
        Eigen::VectorXd r_trial;
        if (!dae_residual(sys, L, t + dt, dt, trial, x_prev, formula, r_trial, error)) {
          return outcome;
        }
        if (r_trial.lpNorm<Eigen::Infinity>() <=
            (1.0 - 1e-4 * alpha) * std::max(res_norm, sys.options.newton_tol)) {
          u = trial;
          accepted_R = r_trial;
          accepted = true;
          break;
        }
        alpha *= 0.5;
      }

      if (!accepted) {
        if (sys.options.dae_reuse_jacobian_factorization &&
            !rebuilt_for_direction && !retried_with_fresh_jacobian) {
          cache.invalidate();
          retried_with_fresh_jacobian = true;
          continue;
        }
        error = "DAE Newton line search failed";
        return outcome;
      }

      R = std::move(accepted_R);
      const double next_norm = R.lpNorm<Eigen::Infinity>();
      const double stale_ratio =
          std::clamp(sys.options.dae_jacobian_stale_residual_ratio, 0.0, 0.999);
      cache.force_rebuild_next =
          next_norm > sys.options.newton_tol &&
          next_norm > stale_ratio * std::max(res_norm, sys.options.newton_tol);
      if (!sys.options.dae_reuse_jacobian_factorization) cache.invalidate();
      break;
    }
  }
  error = "DAE Newton did not converge";
  return outcome;
}

void add_step_counters(StepOutcome& total, const StepOutcome& part) {
  total.iterations += part.iterations;
  total.jacobian_evaluations += part.jacobian_evaluations;
  total.jacobian_residual_evaluations += part.jacobian_residual_evaluations;
  total.jacobian_fd_columns += part.jacobian_fd_columns;
  total.jacobian_colored_groups += part.jacobian_colored_groups;
  total.linear_factorizations += part.linear_factorizations;
}

StepOutcome dae_backward_euler_step(DynamicSystem& sys,
                                    const DaeLayout& L,
                                    double t,
                                    double dt,
                                    const Eigen::VectorXd& x_prev,
                                    DaeNewtonCache& cache,
                                    std::string& error) {
  const DaeStepFormula formula{1.0, nullptr};
  return dae_theta_step(sys, L, t, dt, x_prev, formula, cache, error);
}

StepOutcome dae_trapezoidal_step(DynamicSystem& sys,
                                 const DaeLayout& L,
                                 double t,
                                 double dt,
                                 const Eigen::VectorXd& x_prev,
                                 DaeNewtonCache& trapezoidal_cache,
                                 DaeNewtonCache& predictor_cache,
                                 bool estimate_error,
                                 std::string& error) {
  StepOutcome outcome;
  Eigen::VectorXd f_prev;
  if (!dae_compute_derivatives(sys, L, t, f_prev, error)) return outcome;
  Eigen::VectorXd u_start;
  dae_pack(sys, L, u_start);

  StepOutcome predictor =
      dae_backward_euler_step(sys, L, t, dt, x_prev, predictor_cache, error);
  add_step_counters(outcome, predictor);
  if (!predictor.ok) return outcome;

  Eigen::VectorXd u_be;
  dae_pack(sys, L, u_be);
  const DaeStepFormula trapezoidal{0.5, &f_prev};
  StepOutcome corrector =
      dae_theta_step(sys, L, t, dt, x_prev, trapezoidal, trapezoidal_cache, error);
  add_step_counters(outcome, corrector);
  if (!corrector.ok) return outcome;

  outcome.ok = true;
  if (estimate_error) {
    Eigen::VectorXd u_tr;
    dae_pack(sys, L, u_tr);
    outcome.error_estimated = true;
    outcome.error_norm =
        scaled_error_norm(u_start, u_tr, u_tr - u_be, sys.options);
  }
  return outcome;
}

DynamicResults solve_mass_matrix_dae(DynamicSystem& system) {
  DynamicResults results;
  results.initialization = system.initialization;
  results.warnings.insert(results.warnings.end(), system.warnings.begin(),
                          system.warnings.end());

  std::sort(system.events.begin(), system.events.end(),
            [](const DynamicEvent& a, const DynamicEvent& b) {
              return a.time_s < b.time_s;
            });

  system.x.time_s = system.options.t_start_s;
  std::string error;
  apply_events(system, system.x.time_s, results);
  DaeLayout layout = make_dae_layout(system);
  const bool coupled_network_startup =
      mass_matrix_needs_coupled_network_startup(system);
  if (coupled_network_startup) {
    dae_finalize(system, layout, system.x.time_s);
  } else if (!system.solveNetwork(system.x.time_s, error)) {
    results.success = false;
    results.message = error;
    results.failed_step = 0;
    return results;
  }
  if (!check_numerical_health(system, system.x.time_s, error)) {
    results.success = false;
    results.message = error;
    results.failed_step = 0;
    return results;
  }
  record_snapshot_if_needed(system, results, 0);

  const double t_end = system.options.t_end_s;
  const double base_dt = system.options.dt_s;
  DaeNewtonCache dae_cache;
  DaeNewtonCache dae_predictor_cache;
  int step = 0;

  while (system.x.time_s < t_end - 1e-12) {
    const double t = system.x.time_s;
    if (apply_events(system, t, results)) {
      layout = make_dae_layout(system);
      dae_cache.invalidate();
      dae_predictor_cache.invalidate();
      if (coupled_network_startup) {
        dae_finalize(system, layout, t);
      } else if (!system.solveNetwork(t, error)) {
        results.success = false;
        results.message = error;
        results.failed_step = step;
        results.steps = step;
        return results;
      }
    }
    const double next_discontinuity = next_discontinuity_time(system, t);
    double next_t = std::min(t + base_dt, t_end);
    if (std::isfinite(next_discontinuity)) next_t = std::min(next_t, next_discontinuity);
    if (next_t <= t + 1e-12) next_t = std::min(t + base_dt, t_end);
    const double dt = next_t - t;

    const Eigen::VectorXd x_before = system.x.x;
    const NetworkState y_before = system.y;
    StepOutcome outcome;
    double attempted_dt = dt;
    double accepted_next_t = next_t;
    std::string last_rejection;
    bool step_accepted = false;
    for (int retry = 0; retry <= system.options.max_step_halving; ++retry) {
      system.x.x = x_before;
      system.y = y_before;
      error.clear();
      if (system.options.dae_step_method == DynamicDaeStepMethod::Trapezoidal) {
        outcome = dae_trapezoidal_step(system,
                                       layout,
                                       t,
                                       attempted_dt,
                                       x_before,
                                       dae_cache,
                                       dae_predictor_cache,
                                       system.options.use_adaptive_step,
                                       error);
      } else {
        outcome = dae_backward_euler_step(system, layout, t, attempted_dt,
                                          x_before, dae_cache, error);
      }
      bool accepted = outcome.ok;
      if (accepted && system.options.use_adaptive_step &&
          outcome.error_estimated && outcome.error_norm > 1.0) {
        accepted = false;
        std::ostringstream os;
        os << "DAE embedded local error check failed at t=" << t
           << "s with dt=" << attempted_dt
           << "s; scaled error=" << outcome.error_norm
           << " > 1";
        last_rejection = os.str();
      }
      if (accepted) {
        accepted_next_t = t + attempted_dt;
        last_rejection.clear();
        step_accepted = true;
        break;
      }
      if (last_rejection.empty()) last_rejection = error;
      dae_cache.invalidate();
      dae_predictor_cache.invalidate();
      if (!system.options.use_adaptive_step ||
          retry == system.options.max_step_halving ||
          attempted_dt <= system.options.min_accepted_step_s * (1.0 + 1e-12)) {
        break;
      }
      attempted_dt *= 0.5;
      results.rejected_steps += 1;
    }

    ++step;
    system.x.time_s = step_accepted ? accepted_next_t : t + dt;
    results.newton_iterations += outcome.iterations;
    results.jacobian_evaluations += outcome.jacobian_evaluations;
    results.jacobian_residual_evaluations += outcome.jacobian_residual_evaluations;
    results.jacobian_fd_columns += outcome.jacobian_fd_columns;
    results.jacobian_colored_groups += outcome.jacobian_colored_groups;
    results.linear_factorizations += outcome.linear_factorizations;
    if (!step_accepted) {
      results.success = false;
      results.message = !last_rejection.empty() ? last_rejection : error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    results.max_local_error_norm =
        std::max(results.max_local_error_norm, outcome.error_norm);
    results.min_accepted_step_s =
        results.min_accepted_step_s == 0.0
            ? attempted_dt
            : std::min(results.min_accepted_step_s, attempted_dt);
    results.max_accepted_step_s =
        std::max(results.max_accepted_step_s, attempted_dt);
    dae_finalize(system, layout, system.x.time_s);
    if (dae_cache.valid) ++dae_cache.accepted_steps_since_rebuild;
    if (dae_predictor_cache.valid) ++dae_predictor_cache.accepted_steps_since_rebuild;
    if (apply_events(system, system.x.time_s, results)) {
      layout = make_dae_layout(system);
      dae_cache.invalidate();
      dae_predictor_cache.invalidate();
      if (coupled_network_startup) {
        dae_finalize(system, layout, system.x.time_s);
      } else if (!system.solveNetwork(system.x.time_s, error)) {
        results.success = false;
        results.message = error;
        results.failed_step = step;
        results.steps = step;
        return results;
      }
    }
    if (!check_numerical_health(system, system.x.time_s, error)) {
      results.success = false;
      results.message = error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    record_snapshot_if_needed(system, results, step);
  }

  if (results.snapshots.empty() ||
      std::abs(results.snapshots.back().time_s - system.x.time_s) > 1e-12) {
    results.snapshots.push_back(
        make_snapshot(system, system.options.record_device_outputs));
  }
  results.success = true;
  results.message = "Transient simulation completed (mass-matrix DAE)";
  results.steps = step;
  return results;
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

  if (system.options.solver_type == DynamicSolverType::MassMatrixDae) {
    return solve_mass_matrix_dae(system);
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
  if (!check_numerical_health(system, system.x.time_s, error)) {
    results.success = false;
    results.message = error;
    results.failed_step = 0;
    return results;
  }

  record_snapshot_if_needed(system, results, 0);

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
    if (!check_numerical_health(system, t, error)) {
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
    std::string last_rejection;
    bool step_accepted = false;
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
        case DynamicSolverType::MassMatrixDae:
          // Dispatched by an early return in solve(); never reached here.
          error = "internal error: MassMatrixDae reached the partitioned stepper";
          outcome = {};
          break;
      }
      bool accepted = outcome.ok;
      if (accepted && system.options.use_adaptive_step &&
          outcome.error_estimated &&
          outcome.error_norm > 1.0) {
        accepted = false;
        std::ostringstream os;
        os << "adaptive local error check failed at t=" << t
           << "s with dt=" << attempted_dt
           << "s; scaled error=" << outcome.error_norm
           << " > 1";
        last_rejection = os.str();
      }
      if (accepted && !check_numerical_health(system, t + attempted_dt, error)) {
        accepted = false;
        last_rejection = error;
      }
      if (accepted) {
        accepted_next_t = t + attempted_dt;
        last_rejection.clear();
        step_accepted = true;
        break;
      }
      if (last_rejection.empty()) last_rejection = error;
      if (!system.options.use_adaptive_step ||
          retry == system.options.max_step_halving ||
          attempted_dt <= system.options.min_accepted_step_s * (1.0 + 1e-12)) {
        break;
      }
      attempted_dt *= 0.5;
      results.rejected_steps += 1;
    }

    ++step;
    system.x.time_s = accepted_next_t;
    results.newton_iterations += outcome.iterations;
    if (!step_accepted) {
      results.success = false;
      results.message = !last_rejection.empty() ? last_rejection : error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    results.max_local_error_norm =
        std::max(results.max_local_error_norm, outcome.error_norm);
    results.min_accepted_step_s =
        results.min_accepted_step_s == 0.0
            ? attempted_dt
            : std::min(results.min_accepted_step_s, attempted_dt);
    results.max_accepted_step_s =
        std::max(results.max_accepted_step_s, attempted_dt);

    if (apply_events(system, system.x.time_s, results) &&
        !system.solveNetwork(system.x.time_s, error)) {
      results.success = false;
      results.message = error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    if (!check_numerical_health(system, system.x.time_s, error)) {
      results.success = false;
      results.message = error;
      results.failed_step = step;
      results.steps = step;
      return results;
    }
    record_snapshot_if_needed(system, results, step);
  }

  if (results.snapshots.empty() ||
      std::abs(results.snapshots.back().time_s - system.x.time_s) > 1e-12) {
    results.snapshots.push_back(
        make_snapshot(system, system.options.record_device_outputs));
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
