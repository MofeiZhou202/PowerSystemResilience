#include "hacdcpf/resilience/certified_restoration.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <numeric>
#include <string>
#include <thread>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

#include "hacdcpf/dynamics/DynamicFrequency.hpp"
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/engine/engine.hpp"
#include "hacdcpf/engine/solver/external/adapters.hpp"

namespace hacdcpf::analysis {
namespace {

using dynamics::DynamicSnapshot;
using dynamics::DynamicSystem;
using dynamics::NetworkState;
namespace engine = mipsolvers::engine;

double vector_inf(const Eigen::VectorXd& value) {
  return value.size() > 0 ? value.lpNorm<Eigen::Infinity>() : 0.0;
}

double matrix_inf(const Eigen::MatrixXd& value) {
  if (value.size() == 0) return 0.0;
  return value.cwiseAbs().rowwise().sum().maxCoeff();
}

double logarithmic_norm_inf(const Eigen::MatrixXd& value) {
  if (value.rows() == 0) return 0.0;
  double measure = -std::numeric_limits<double>::infinity();
  for (Eigen::Index row = 0; row < value.rows(); ++row) {
    double candidate = value(row, row);
    for (Eigen::Index col = 0; col < value.cols(); ++col) {
      if (col != row) candidate += std::abs(value(row, col));
    }
    measure = std::max(measure, candidate);
  }
  return std::max(0.0, measure);
}

double saturating_product(double lhs, double rhs) {
  const double magnitude = std::abs(lhs);
  const double factor = std::abs(rhs);
  if (!std::isfinite(magnitude) || !std::isfinite(factor)) {
    return std::numeric_limits<double>::max();
  }
  if (factor > 0.0 &&
      magnitude > std::numeric_limits<double>::max() / factor) {
    return std::numeric_limits<double>::max();
  }
  return magnitude * factor;
}

double saturating_sum(double lhs, double rhs) {
  if (!std::isfinite(lhs) || !std::isfinite(rhs) ||
      lhs > std::numeric_limits<double>::max() - rhs) {
    return std::numeric_limits<double>::max();
  }
  return lhs + rhs;
}

Eigen::VectorXd flatten_algebraic(const NetworkState& y) {
  const int n_ac = static_cast<int>(y.Vac_abc.size());
  const int n_dc = static_cast<int>(y.Vdc.size());
  Eigen::VectorXd flat(2 * n_ac + n_dc);
  for (int i = 0; i < n_ac; ++i) {
    flat[i] = y.Vac_abc[i].real();
    flat[n_ac + i] = y.Vac_abc[i].imag();
  }
  if (n_dc > 0) flat.tail(n_dc) = y.Vdc;
  return flat;
}

NetworkState unflatten_algebraic(const Eigen::VectorXd& flat, int n_ac,
                                 int n_dc) {
  NetworkState y;
  y.resize(n_ac, n_dc);
  for (int i = 0; i < n_ac; ++i) {
    y.Vac_abc[i] = {flat[i], flat[n_ac + i]};
  }
  if (n_dc > 0) y.Vdc = flat.tail(n_dc);
  return y;
}

NetworkState snapshot_algebraic(const DynamicSnapshot& snapshot) {
  NetworkState y;
  y.resize(static_cast<int>(snapshot.vac_abc.size()),
           static_cast<int>(snapshot.vdc.size()));
  y.Vac_abc = snapshot.vac_abc;
  y.Vdc = snapshot.vdc;
  return y;
}

bool event_between(const std::vector<dynamics::DynamicEvent>& events,
                   double left, double right) {
  for (const auto& event : events) {
    if (event.time_s > left + 1e-12 && event.time_s <= right + 1e-12) {
      return true;
    }
  }
  return false;
}

Eigen::VectorXd snapshot_derivative(
    const std::vector<DynamicSnapshot>& snapshots, std::size_t index,
    const std::vector<dynamics::DynamicEvent>& events) {
  const auto& current = snapshots[index];
  if (snapshots.size() < 2) return Eigen::VectorXd::Zero(current.state.size());
  std::size_t left = index > 0 ? index - 1 : index;
  std::size_t right = index + 1 < snapshots.size() ? index + 1 : index;
  if (left < index && event_between(events, snapshots[left].time_s,
                                    current.time_s)) {
    left = index;
  }
  if (right > index && event_between(events, current.time_s,
                                     snapshots[right].time_s)) {
    right = index;
  }
  if (left == right) return Eigen::VectorXd::Zero(current.state.size());
  const double dt = snapshots[right].time_s - snapshots[left].time_s;
  if (!(dt > 0.0)) return Eigen::VectorXd::Zero(current.state.size());
  return (snapshots[right].state - snapshots[left].state) / dt;
}

std::vector<std::size_t> sample_indices(std::size_t size, int maximum) {
  if (size == 0 || maximum <= 0) return {};
  const std::size_t count = std::min(size, static_cast<std::size_t>(maximum));
  std::vector<std::size_t> out;
  out.reserve(count);
  if (count == 1) return {size - 1};
  for (std::size_t k = 0; k < count; ++k) {
    out.push_back(k * (size - 1) / (count - 1));
  }
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

std::vector<Eigen::Index> probe_columns(Eigen::Index size, int maximum) {
  if (size <= 0 || maximum <= 0) return {};
  const Eigen::Index count =
      std::min(size, static_cast<Eigen::Index>(maximum));
  std::vector<Eigen::Index> out;
  out.reserve(static_cast<std::size_t>(count));
  if (count == 1) return {size - 1};
  for (Eigen::Index k = 0; k < count; ++k) {
    out.push_back(k * (size - 1) / (count - 1));
  }
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

Eigen::VectorXd trajectory_state_scale(
    const dynamics::DynamicResults& trajectory, double floor) {
  if (trajectory.snapshots.empty()) return {};
  Eigen::VectorXd scale = Eigen::VectorXd::Constant(
      trajectory.snapshots.front().state.size(), std::max(1.0e-12, floor));
  for (const auto& snapshot : trajectory.snapshots) {
    if (snapshot.state.size() != scale.size()) continue;
    scale = scale.cwiseMax(snapshot.state.cwiseAbs());
  }
  return scale;
}

struct ObservedMargins {
  double frequency{std::numeric_limits<double>::infinity()};
  double voltage{std::numeric_limits<double>::infinity()};
  double rocof{std::numeric_limits<double>::infinity()};
  bool frequency_observed{false};
  bool voltage_observed{false};
  bool rocof_observed{false};
  bool converter_current_observed{false};
  bool current_limit_active{false};
  std::size_t frequency_index{0};
  std::size_t voltage_index{0};
  std::size_t rocof_index{0};
  Eigen::Index voltage_phase_index{0};
};

struct ReducedPropagator {
  Eigen::MatrixXd matrix;
  Eigen::MatrixXcd eigenvectors;
  Eigen::MatrixXcd eigenvectors_inverse;
  Eigen::VectorXcd eigenvalues;
  bool modal_valid{false};
};

ReducedPropagator make_reduced_propagator(const Eigen::MatrixXd& matrix) {
  ReducedPropagator out;
  out.matrix = matrix;
  if (matrix.rows() == 0 || matrix.rows() != matrix.cols()) return out;
  Eigen::ComplexEigenSolver<Eigen::MatrixXd> solver(matrix);
  if (solver.info() != Eigen::Success) return out;
  out.eigenvalues = solver.eigenvalues();
  out.eigenvectors = solver.eigenvectors();
  Eigen::FullPivLU<Eigen::MatrixXcd> lu(out.eigenvectors);
  if (!lu.isInvertible()) return out;
  out.eigenvectors_inverse = lu.inverse();
  out.modal_valid = out.eigenvectors_inverse.allFinite();
  return out;
}

double propagated_output_gain(const Eigen::VectorXd& output_gradient,
                              const ReducedPropagator& propagator,
                              double tau) {
  if (propagator.matrix.rows() != output_gradient.size()) {
    return std::numeric_limits<double>::infinity();
  }
  if (propagator.matrix.rows() <= 100 || !propagator.modal_valid) {
    const Eigen::RowVectorXd propagated = output_gradient.transpose() *
        (propagator.matrix * tau).exp();
    return propagated.allFinite()
        ? propagated.lpNorm<1>()
        : std::numeric_limits<double>::infinity();
  }
  Eigen::RowVectorXcd modal =
      output_gradient.cast<std::complex<double>>().transpose() *
      propagator.eigenvectors;
  for (Eigen::Index i = 0; i < modal.size(); ++i) {
    modal[i] *= std::exp(propagator.eigenvalues[i] * tau);
  }
  const Eigen::RowVectorXcd propagated =
      modal * propagator.eigenvectors_inverse;
  return propagated.allFinite()
      ? propagated.cwiseAbs().sum()
      : std::numeric_limits<double>::infinity();
}

bool converter_like(const dynamics::DynamicDeviceOutput& device) {
  return device.type.find("Inverter") != std::string::npos ||
         device.type.find("VSC") != std::string::npos ||
         device.type.find("Converter") != std::string::npos ||
         device.type.find("GridForming") != std::string::npos ||
         device.type == "PVSystem";
}

ObservedMargins observed_margins(
    const dynamics::DynamicResults& result,
    const MultiFidelityCertificateOptions& options) {
  ObservedMargins margins;
  for (std::size_t index = 0; index < result.snapshots.size(); ++index) {
    const auto& snapshot = result.snapshots[index];
    if (std::isfinite(snapshot.coi_frequency_hz) &&
        snapshot.coi_frequency_hz > 0.0) {
      margins.frequency_observed = true;
      const double value = snapshot.coi_frequency_hz - options.min_frequency_hz;
      if (value < margins.frequency) {
        margins.frequency = value;
        margins.frequency_index = index;
      }
    }
    if (snapshot.vac_abc.size() > 0) {
      margins.voltage_observed = true;
      double vmin = std::numeric_limits<double>::infinity();
      Eigen::Index vmin_index = 0;
      for (Eigen::Index i = 0; i < snapshot.vac_abc.size(); ++i) {
        const double magnitude = std::abs(snapshot.vac_abc[i]);
        if (magnitude < vmin) {
          vmin = magnitude;
          vmin_index = i;
        }
      }
      const double value = vmin - options.min_ac_voltage_pu;
      if (value < margins.voltage) {
        margins.voltage = value;
        margins.voltage_index = index;
        margins.voltage_phase_index = vmin_index;
      }
    }
    if (std::isfinite(snapshot.coi_rocof_hz_s)) {
      const double value =
          options.max_rocof_hz_s - std::abs(snapshot.coi_rocof_hz_s);
      if (value < margins.rocof) {
        margins.rocof = value;
        margins.rocof_index = index;
      }
      margins.rocof_observed = true;
    }
    for (const auto& device : snapshot.device_outputs) {
      if (!converter_like(device)) continue;
      for (const std::string& key : {"i_rms_pu", "i_mag_pu",
                                     "current_mag_pu"}) {
        const auto current = device.values.find(key);
        if (current != device.values.end() && std::isfinite(current->second)) {
          margins.converter_current_observed = true;
        }
      }
      const auto limited = device.values.find("current_limit_active");
      if (limited != device.values.end() && limited->second > 0.5) {
        margins.current_limit_active = true;
      }
    }
  }
  return margins;
}

bool evaluate_fields(const DynamicSystem& system, double t,
                     const Eigen::VectorXd& x, const Eigen::VectorXd& yflat,
                     Eigen::VectorXd& f, Eigen::VectorXd& g) {
  const int n_ac = system.network.acPhaseNodeCount();
  const int n_dc = system.network.dcBusCount();
  const NetworkState y = unflatten_algebraic(yflat, n_ac, n_dc);
  Eigen::VectorXd rf;
  std::string error;
  if (!system.evaluateDaeResidual(t, x, y, Eigen::VectorXd::Zero(x.size()),
                                  rf, g, error)) {
    return false;
  }
  f = -rf;
  return true;
}

bool sampled_constants(DynamicSystem& replay,
                       const dynamics::DynamicResults& trajectory,
                       const MultiFidelityCertificateOptions& options,
                       double& kappa_g, double& l_f,
                       bool& used_directional_proxy,
                       Eigen::MatrixXd& comparison_matrix,
                       std::vector<ReducedPropagator>& reduced_samples,
                       double& transition_gain,
                       double& transition_growth_rate,
                       const Eigen::VectorXd& state_scale,
                       double certificate_horizon) {
  kappa_g = 0.0;
  l_f = 0.0;
  used_directional_proxy = false;
  comparison_matrix.resize(0, 0);
  reduced_samples.clear();
  transition_gain = 0.0;
  transition_growth_rate =
      -std::numeric_limits<double>::infinity();
  bool evaluated = false;
  const auto indices =
      sample_indices(trajectory.snapshots.size(), options.max_jacobian_samples);
  for (const std::size_t index : indices) {
    const auto& snapshot = trajectory.snapshots[index];
    replay.x.x = snapshot.state;
    replay.y = snapshot_algebraic(snapshot);
    dynamics::applyDynamicEventsThrough(replay, snapshot.time_s);

    const Eigen::VectorXd x0 = snapshot.state;
    const Eigen::VectorXd y0 = flatten_algebraic(replay.y);
    Eigen::VectorXd f0;
    Eigen::VectorXd g0;
    if (!evaluate_fields(replay, snapshot.time_s, x0, y0, f0, g0)) return false;
    const bool use_dense =
        std::max(x0.size(), y0.size()) <=
        options.dense_jacobian_dimension_limit;
    if (!use_dense) {
      used_directional_proxy = true;
      double fx_proxy = 0.0;
      double gx_proxy = 0.0;
      double fy_proxy = 0.0;
      double gy_min_gain = std::numeric_limits<double>::infinity();
      for (const Eigen::Index col :
           probe_columns(x0.size(), options.directional_jacobian_probes)) {
        Eigen::VectorXd xp = x0;
        const double h = options.finite_difference_relative_step *
                         std::max(1.0, std::abs(x0[col] / state_scale[col]));
        xp[col] += h * state_scale[col];
        Eigen::VectorXd fp;
        Eigen::VectorXd gp;
        if (!evaluate_fields(replay, snapshot.time_s, xp, y0, fp, gp)) {
          return false;
        }
        fx_proxy = std::max(
            fx_proxy,
            vector_inf(((fp - f0) / h).cwiseQuotient(state_scale)));
        gx_proxy = std::max(gx_proxy, vector_inf((gp - g0) / h));
      }
      for (const Eigen::Index col :
           probe_columns(y0.size(), options.directional_jacobian_probes)) {
        Eigen::VectorXd yp = y0;
        const double h = options.finite_difference_relative_step *
                         std::max(1.0, std::abs(y0[col]));
        yp[col] += h;
        Eigen::VectorXd fp;
        Eigen::VectorXd gp;
        if (!evaluate_fields(replay, snapshot.time_s, x0, yp, fp, gp)) {
          return false;
        }
        fy_proxy = std::max(
            fy_proxy,
            vector_inf(((fp - f0) / h).cwiseQuotient(state_scale)));
        const double gy_gain = vector_inf((gp - g0) / h);
        if (gy_gain > 1.0e-12) {
          gy_min_gain = std::min(gy_min_gain, gy_gain);
        }
      }
      if (!std::isfinite(gy_min_gain)) return false;
      const double inverse_proxy = 1.0 / gy_min_gain;
      kappa_g = std::max(kappa_g, inverse_proxy * (1.0 + gx_proxy));
      l_f = std::max(l_f, fx_proxy + fy_proxy * inverse_proxy * gx_proxy);
      if (x0.size() <= options.dense_jacobian_dimension_limit) {
        Eigen::VectorXd reduced_f0;
        std::string reduced_error;
        if (!replay.evaluateDerivatives(snapshot.time_s, x0, reduced_f0,
                                        reduced_error)) {
          return false;
        }
        Eigen::MatrixXd reduced =
            Eigen::MatrixXd::Zero(x0.size(), x0.size());
        for (Eigen::Index col = 0; col < x0.size(); ++col) {
          Eigen::VectorXd xp = x0;
          const double h = options.finite_difference_relative_step *
                           std::max(1.0, std::abs(x0[col] / state_scale[col]));
          xp[col] += h * state_scale[col];
          Eigen::VectorXd reduced_fp;
          if (!replay.evaluateDerivatives(snapshot.time_s, xp, reduced_fp,
                                          reduced_error)) {
            return false;
          }
          reduced.col(col) =
              ((reduced_fp - reduced_f0) / h).cwiseQuotient(state_scale);
        }
        ReducedPropagator propagator = make_reduced_propagator(reduced);
        if (propagator.eigenvalues.size() > 0) {
          transition_growth_rate = std::max(
              transition_growth_rate,
              propagator.eigenvalues.real().maxCoeff());
        }
        reduced_samples.push_back(std::move(propagator));
        l_f = std::max(l_f, logarithmic_norm_inf(reduced));
      }
      evaluated = true;
      continue;
    }
    Eigen::MatrixXd fx(f0.size(), x0.size());
    Eigen::MatrixXd gx(g0.size(), x0.size());
    Eigen::MatrixXd fy(f0.size(), y0.size());
    Eigen::MatrixXd gy(g0.size(), y0.size());
    for (Eigen::Index col = 0; col < x0.size(); ++col) {
      Eigen::VectorXd xp = x0;
      const double h = options.finite_difference_relative_step *
                       std::max(1.0, std::abs(x0[col]));
      xp[col] += h;
      Eigen::VectorXd fp;
      Eigen::VectorXd gp;
      if (!evaluate_fields(replay, snapshot.time_s, xp, y0, fp, gp)) return false;
      fx.col(col) = (fp - f0) / h;
      gx.col(col) = (gp - g0) / h;
    }
    for (Eigen::Index col = 0; col < y0.size(); ++col) {
      Eigen::VectorXd yp = y0;
      const double h = options.finite_difference_relative_step *
                       std::max(1.0, std::abs(y0[col]));
      yp[col] += h;
      Eigen::VectorXd fp;
      Eigen::VectorXd gp;
      if (!evaluate_fields(replay, snapshot.time_s, x0, yp, fp, gp)) return false;
      fy.col(col) = (fp - f0) / h;
      gy.col(col) = (gp - g0) / h;
    }
    Eigen::FullPivLU<Eigen::MatrixXd> lu(gy);
    if (!lu.isInvertible()) return false;
    const Eigen::MatrixXd gy_inv = lu.inverse();
    const Eigen::MatrixXd dx = state_scale.asDiagonal();
    const Eigen::MatrixXd dx_inv =
        state_scale.cwiseInverse().asDiagonal();
    const Eigen::MatrixXd fx_scaled = dx_inv * fx * dx;
    const Eigen::MatrixXd gx_scaled = gx * dx;
    const Eigen::MatrixXd fy_scaled = dx_inv * fy;
    const double local_kappa =
        matrix_inf(gy_inv) * (1.0 + matrix_inf(gx_scaled));
    const Eigen::MatrixXd reduced =
        fx_scaled - fy_scaled * gy_inv * gx_scaled;
    reduced_samples.push_back(make_reduced_propagator(reduced));
    Eigen::ComplexEigenSolver<Eigen::MatrixXd> modal_solver(reduced);
    if (modal_solver.info() == Eigen::Success) {
      transition_growth_rate = std::max(
          transition_growth_rate,
          modal_solver.eigenvalues().real().maxCoeff());
    }
    const int time_probes = std::max(1, options.transition_time_probes);
    for (int probe = 0; probe <= time_probes; ++probe) {
      const double tau = certificate_horizon *
                         static_cast<double>(probe) /
                         static_cast<double>(time_probes);
      const Eigen::MatrixXd transition = (reduced * tau).exp();
      if (transition.allFinite()) {
        transition_gain =
            std::max(transition_gain, matrix_inf(transition));
      }
    }
    Eigen::MatrixXd local_comparison = reduced.cwiseAbs();
    local_comparison.diagonal() = reduced.diagonal();
    if (comparison_matrix.size() == 0) {
      comparison_matrix = std::move(local_comparison);
    } else {
      comparison_matrix = comparison_matrix.cwiseMax(local_comparison);
    }
    kappa_g = std::max(kappa_g, local_kappa);
    l_f = std::max(l_f, logarithmic_norm_inf(reduced));
    evaluated = true;
  }
  kappa_g *= options.jacobian_tube_factor;
  l_f *= options.jacobian_tube_factor;
  return evaluated && std::isfinite(kappa_g) && std::isfinite(l_f);
}

enum class SecurityOutput { Frequency, Voltage, Rocof };

bool evaluate_security_output(DynamicSystem& replay,
                              const DynamicSnapshot& snapshot,
                              SecurityOutput output,
                              Eigen::Index voltage_phase_index,
                              double& value) {
  replay.x.x = snapshot.state;
  replay.y = snapshot_algebraic(snapshot);
  dynamics::applyDynamicEventsThrough(replay, snapshot.time_s);
  Eigen::VectorXd dxdt;
  std::string error;
  if (!replay.evaluateDerivatives(snapshot.time_s, snapshot.state, dxdt,
                                  error)) {
    return false;
  }
  replay.x.dxdt = dxdt;
  if (output == SecurityOutput::Frequency) {
    value = dynamics::computeFrequencyReport(replay).system_coi_frequency_hz;
  } else if (output == SecurityOutput::Rocof) {
    value = std::abs(
        dynamics::computeFrequencyReport(replay).system_coi_rocof_hz_s);
  } else {
    if (voltage_phase_index < 0 ||
        voltage_phase_index >= replay.y.Vac_abc.size()) {
      return false;
    }
    value = std::abs(replay.y.Vac_abc[voltage_phase_index]);
  }
  return std::isfinite(value);
}

bool security_output_gradient(DynamicSystem& replay,
                              const DynamicSnapshot& snapshot,
                              SecurityOutput output,
                              Eigen::Index voltage_phase_index,
                              const Eigen::VectorXd& state_scale,
                              double relative_step,
                              Eigen::VectorXd& gradient) {
  if (snapshot.state.size() == 0 ||
      state_scale.size() != snapshot.state.size()) {
    return false;
  }
  double base = 0.0;
  if (!evaluate_security_output(replay, snapshot, output,
                                voltage_phase_index, base)) {
    return false;
  }
  gradient = Eigen::VectorXd::Zero(snapshot.state.size());
  for (Eigen::Index col = 0; col < snapshot.state.size(); ++col) {
    Eigen::VectorXd perturbed = snapshot.state;
    const double h = relative_step *
                     std::max(1.0, std::abs(snapshot.state[col] /
                                                state_scale[col]));
    perturbed[col] += h * state_scale[col];
    DynamicSnapshot perturbed_snapshot = snapshot;
    perturbed_snapshot.state = std::move(perturbed);
    double shifted = 0.0;
    if (!evaluate_security_output(replay, perturbed_snapshot, output,
                                  voltage_phase_index, shifted)) {
      return false;
    }
    gradient[col] = (shifted - base) / h;
  }
  return gradient.allFinite();
}

double output_transition_gain(
    const Eigen::VectorXd& output_gradient,
    const std::vector<ReducedPropagator>& reduced_samples,
    double horizon, int time_probes) {
  if (output_gradient.size() == 0 || reduced_samples.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double gain = output_gradient.lpNorm<1>();
  const int probes = std::max(1, time_probes);
  for (const auto& reduced : reduced_samples) {
    for (int probe = 0; probe <= probes; ++probe) {
      const double tau = horizon * static_cast<double>(probe) /
                         static_cast<double>(probes);
      gain = std::max(gain,
                      propagated_output_gain(output_gradient, reduced, tau));
    }
  }
  return gain;
}

struct DefectImpulse {
  double time_s{0.0};
  double magnitude{0.0};
};

bool hermite_reconstruction_defect(
    DynamicSystem& replay, const dynamics::DynamicResults& trajectory,
    const std::vector<dynamics::DynamicEvent>& events,
    const Eigen::VectorXd& state_scale, double& defect_integral,
    std::vector<DefectImpulse>& defect_profile) {
  defect_integral = 0.0;
  defect_profile.clear();
  if (trajectory.snapshots.size() < 2 || state_scale.size() == 0) {
    return false;
  }
  dynamics::applyDynamicEventsThrough(
      replay, trajectory.snapshots.front().time_s);
  for (std::size_t index = 1; index < trajectory.snapshots.size(); ++index) {
    const auto& left = trajectory.snapshots[index - 1];
    const auto& right = trajectory.snapshots[index];
    const double dt = right.time_s - left.time_s;
    if (!(dt > 0.0) || left.state.size() != state_scale.size() ||
        right.state.size() != state_scale.size()) {
      return false;
    }

    Eigen::VectorXd f_left;
    Eigen::VectorXd f_right;
    std::string error;
    replay.y = snapshot_algebraic(left);
    if (!replay.evaluateDerivatives(left.time_s, left.state, f_left, error)) {
      return false;
    }

    // A solver step ending at an event is advanced with the pre-event field;
    // the reset/topology update is applied only after the step is accepted.
    replay.y = snapshot_algebraic(right);
    if (!replay.evaluateDerivatives(right.time_s, right.state, f_right,
                                    error)) {
      return false;
    }
    constexpr double theta_a = 0.21132486540518713;
    constexpr double theta_b = 0.7886751345948129;
    double interval_residual_sum = 0.0;
    for (const double theta : {theta_a, theta_b}) {
      const double theta2 = theta * theta;
      const double theta3 = theta2 * theta;
      const double h00 = 2.0 * theta3 - 3.0 * theta2 + 1.0;
      const double h10 = theta3 - 2.0 * theta2 + theta;
      const double h01 = -2.0 * theta3 + 3.0 * theta2;
      const double h11 = theta3 - theta2;
      const Eigen::VectorXd interpolated =
          h00 * left.state + h10 * dt * f_left + h01 * right.state +
          h11 * dt * f_right;
      const Eigen::VectorXd interpolated_derivative =
          ((6.0 * theta2 - 6.0 * theta) / dt) * left.state +
          (3.0 * theta2 - 4.0 * theta + 1.0) * f_left +
          ((-6.0 * theta2 + 6.0 * theta) / dt) * right.state +
          (3.0 * theta2 - 2.0 * theta) * f_right;
      Eigen::VectorXd field;
      replay.y = snapshot_algebraic(left);
      if (!replay.evaluateDerivatives(left.time_s + theta * dt,
                                      interpolated, field, error)) {
        return false;
      }
      interval_residual_sum += vector_inf(
          (interpolated_derivative - field).cwiseQuotient(state_scale));
    }
    const double interval_defect = 0.5 * dt * interval_residual_sum;
    defect_integral += interval_defect;
    defect_profile.push_back(
        {left.time_s + 0.5 * dt, interval_defect});

    if (event_between(events, left.time_s, right.time_s)) {
      replay.x.x = right.state;
      replay.y = snapshot_algebraic(right);
      dynamics::applyDynamicEventsThrough(replay, right.time_s);
    }
  }
  return std::isfinite(defect_integral);
}

double output_defect_convolution(
    const Eigen::VectorXd& output_gradient,
    const std::vector<ReducedPropagator>& reduced_samples,
    const std::vector<DefectImpulse>& defect_profile,
    double output_time_s, double horizon, int time_probes) {
  if (output_gradient.size() == 0 || reduced_samples.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const int probes = std::max(1, time_probes);
  std::vector<double> sampled_gains(static_cast<std::size_t>(probes + 1),
                                    output_gradient.lpNorm<1>());
  for (int probe = 0; probe <= probes; ++probe) {
    const double tau = horizon * static_cast<double>(probe) /
                       static_cast<double>(probes);
    for (const auto& reduced : reduced_samples) {
      sampled_gains[static_cast<std::size_t>(probe)] = std::max(
          sampled_gains[static_cast<std::size_t>(probe)],
          propagated_output_gain(output_gradient, reduced, tau));
    }
  }
  double bound = 0.0;
  for (const auto& impulse : defect_profile) {
    if (impulse.time_s > output_time_s + 1.0e-12) break;
    const double tau = std::max(0.0, output_time_s - impulse.time_s);
    const double scaled_probe = horizon > 0.0
        ? std::clamp(tau / horizon * static_cast<double>(probes),
                     0.0, static_cast<double>(probes))
        : 0.0;
    const int lower_probe = static_cast<int>(std::floor(scaled_probe));
    const int upper_probe = std::min(probes, lower_probe + 1);
    const double local_gain = std::max(
        sampled_gains[static_cast<std::size_t>(lower_probe)],
        sampled_gains[static_cast<std::size_t>(upper_probe)]);
    bound = saturating_sum(
        bound, saturating_product(local_gain, impulse.magnitude));
  }
  return bound;
}

bool materialize_mess(const CertifiedRestorationAction& action,
                      HybridPowerSystem& system, int& dynamic_storage_index,
                      std::string& error) {
  dynamic_storage_index = -1;
  if (!action.requires_mess) return true;
  const auto mobile = std::find_if(
      system.mobile_storage.begin(), system.mobile_storage.end(),
      [&](const MobileStorage& storage) {
        return storage.in_service &&
               (action.mess_storage_index == 0 ||
                storage.index == action.mess_storage_index);
      });
  if (mobile == system.mobile_storage.end()) {
    error = "executable MESS action references no in-service mobile storage";
    return false;
  }
  const int target_bus = action.mess_target_ac_bus != 0
                             ? action.mess_target_ac_bus
                             : mobile->target_bus;
  const bool target_exists = std::any_of(
      system.ac.buses.begin(), system.ac.buses.end(),
      [&](const ACBus& bus) { return bus.in_service && bus.index == target_bus; });
  if (!target_exists) {
    error = "executable MESS action target is not an in-service AC bus";
    return false;
  }

  int next_index = 1;
  for (const auto& storage : system.ac.storage) {
    next_index = std::max(next_index, storage.index + 1);
  }
  Storage dynamic_storage;
  dynamic_storage.index = next_index;
  dynamic_storage.bus = target_bus;
  dynamic_storage.in_service = true;
  dynamic_storage.name = mobile->name.empty()
                             ? "Materialized MESS " + std::to_string(mobile->index)
                             : mobile->name + " (post-arrival)";
  dynamic_storage.p_mw = action.mess_dispatch_mw;
  dynamic_storage.q_mvar = action.mess_dispatch_mvar;
  dynamic_storage.p_rated_mw = mobile->p_rated_mw;
  dynamic_storage.pmax_mw = mobile->pmax_mw;
  dynamic_storage.pmin_mw = mobile->pmin_mw;
  dynamic_storage.qmax_mvar = mobile->qmax_mvar;
  dynamic_storage.qmin_mvar = mobile->qmin_mvar;
  dynamic_storage.e_rated_mwh = mobile->e_rated_mwh;
  dynamic_storage.e_mwh = action.mess_available_energy_mwh;
  dynamic_storage.soc_init = mobile->e_rated_mwh > 0.0
                                 ? std::clamp(action.mess_available_energy_mwh /
                                                  mobile->e_rated_mwh,
                                              mobile->soc_min, mobile->soc_max)
                                 : mobile->soc_init;
  dynamic_storage.soc_min = mobile->soc_min;
  dynamic_storage.soc_max = mobile->soc_max;
  dynamic_storage.eta_charge = mobile->eta_charge;
  dynamic_storage.eta_discharge = mobile->eta_discharge;
  dynamic_storage.grid_forming = action.requires_grid_forming_mess;
  dynamic_storage.grid_forming =
      dynamic_storage.grid_forming && mobile->grid_forming;
  dynamic_storage.anti_islanding = !dynamic_storage.grid_forming;
  dynamic_storage.control_mode = dynamic_storage.grid_forming
                                     ? "grid_forming"
                                     : "pq";
  dynamic_storage.type = mobile->type;
  dynamic_storage.dynamic_model = mobile->dynamic_model;
  dynamic_storage_index = dynamic_storage.index;
  system.ac.storage.push_back(std::move(dynamic_storage));
  return true;
}

bool componentwise_comparison_eta(const Eigen::MatrixXd& comparison,
                                  double horizon, double initial_error,
                                  double total_forcing, double& eta,
                                  double& spectral_abscissa) {
  if (comparison.rows() == 0 || comparison.rows() != comparison.cols() ||
      !(horizon > 0.0)) {
    return false;
  }
  Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(comparison, false);
  if (eigen_solver.info() == Eigen::Success) {
    spectral_abscissa =
        eigen_solver.eigenvalues().real().maxCoeff();
  }

  const Eigen::Index n = comparison.rows();
  Eigen::MatrixXd augmented = Eigen::MatrixXd::Zero(n + 1, n + 1);
  augmented.topLeftCorner(n, n) = comparison;
  augmented.topRightCorner(n, 1).setConstant(
      std::max(0.0, total_forcing) / horizon);
  Eigen::VectorXd initial = Eigen::VectorXd::Zero(n + 1);
  initial.head(n).setConstant(std::max(0.0, initial_error));
  initial[n] = 1.0;
  const Eigen::VectorXd terminal = (augmented * horizon).exp() * initial;
  if (!terminal.allFinite()) return false;
  eta = terminal.head(n).cwiseMax(0.0).maxCoeff();
  return std::isfinite(eta);
}

struct MasterSelection {
  bool feasible{false};
  bool optimal{false};
  int action_index{-1};
  double objective{-std::numeric_limits<double>::infinity()};
  std::string status;
};

MasterSelection solve_master(
    const std::vector<CertifiedRestorationAction>& actions,
    const std::vector<bool>& excluded,
    const std::vector<ExecutabilityReport>& executability) {
  std::vector<int> enabled_indices;
  for (int i = 0; i < static_cast<int>(actions.size()); ++i) {
    if (!excluded[static_cast<std::size_t>(i)] &&
        executability[static_cast<std::size_t>(i)].executable) {
      enabled_indices.push_back(i);
    }
  }
  if (enabled_indices.empty()) return {};
  if (enabled_indices.size() == 1) {
    const int index = enabled_indices.front();
    return {true, true, index,
            actions[static_cast<std::size_t>(index)].objective,
            "singleton action fixed by exact master-MIP presolve"};
  }
  engine::MIPModel model;
  auto& lp = model.linear_part;
  const int n = static_cast<int>(actions.size());
  lp.sense = engine::Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(n);
  lp.vars.reserve(actions.size());
  for (int i = 0; i < n; ++i) {
    const bool enabled = !excluded[static_cast<std::size_t>(i)] &&
                         executability[static_cast<std::size_t>(i)].executable;
    lp.vars.push_back({engine::VarType::Binary, 0.0, enabled ? 1.0 : 0.0,
                       "restore_action_" + actions[static_cast<std::size_t>(i)].id});
    lp.c[i] = -actions[static_cast<std::size_t>(i)].objective -
              1.0e-9 * actions[static_cast<std::size_t>(i)].tie_break_priority;
    model.binary_idx.push_back(i);
  }
  lp.A.resize(0, n);
  lp.b.resize(0);
  lp.Aeq.resize(1, n);
  std::vector<Eigen::Triplet<double>> equal;
  equal.reserve(actions.size());
  for (int i = 0; i < n; ++i) equal.emplace_back(0, i, 1.0);
  lp.Aeq.setFromTriplets(equal.begin(), equal.end());
  lp.beq = Eigen::VectorXd::Ones(1);
  model.initial_solution = Eigen::VectorXd::Zero(n);
  const int warm_index = *std::max_element(
      enabled_indices.begin(), enabled_indices.end(), [&](int lhs, int rhs) {
        const auto& a = actions[static_cast<std::size_t>(lhs)];
        const auto& b = actions[static_cast<std::size_t>(rhs)];
        if (a.objective != b.objective) return a.objective < b.objective;
        return a.tie_break_priority < b.tie_break_priority;
      });
  model.initial_solution[warm_index] = 1.0;

  engine::BCOptions options;
  options.gap_tol = 1.0e-9;
  // MIPSolvers adapters retain thread-local branch-and-cut scratch state.
  // A catalog solve must not affect a later restoration master in the same
  // process, so give this finite selection MIP a fresh, joined thread exactly
  // as the restoration MIP does for StrictHiGHS.
  std::packaged_task<engine::SolveResult()> task(
      [&model, options] {
        engine::StrictHighsBranchAndCutAdapter adapter(options);
        return adapter.solve_milp(model);
      });
  auto future = task.get_future();
  std::jthread worker(std::move(task));
  const engine::SolveResult solved = future.get();

  MasterSelection result;
  result.status = solved.stats.status;
  if (!solved.stats.success || solved.x.size() != n ||
      std::abs(solved.stats.mip_gap) > 1.0e-9) {
    return result;
  }
  Eigen::Index selected = -1;
  solved.x.maxCoeff(&selected);
  if (selected < 0 || solved.x[selected] < 0.5) return result;
  result.feasible = true;
  result.optimal = true;
  result.action_index = static_cast<int>(selected);
  result.objective = actions[static_cast<std::size_t>(selected)].objective;
  return result;
}

}  // namespace

const char* to_string(CertificateLabel label) noexcept {
  switch (label) {
    case CertificateLabel::Safe: return "safe";
    case CertificateLabel::Unsafe: return "unsafe";
    case CertificateLabel::Unresolved: return "unresolved";
    case CertificateLabel::Failed: return "failed";
  }
  return "failed";
}

MultiFidelityCertificateEngine::MultiFidelityCertificateEngine(
    HybridPowerSystem system, dynamics::DynamicSolverOptions oracle_options,
    MultiFidelityCertificateOptions certificate_options)
    : system_(std::move(system)),
      oracle_options_(std::move(oracle_options)),
      certificate_options_(std::move(certificate_options)) {}

ExecutabilityReport MultiFidelityCertificateEngine::checkExecutability(
    const CertifiedRestorationAction& action) const {
  ExecutabilityReport report;
  report.cyber_executable = action.command_path_available &&
                            action.authorization_valid &&
                            action.acknowledgement_available;
  if (!action.command_path_available)
    report.failed_constraints.push_back("cyber_command_path_unavailable");
  if (!action.authorization_valid)
    report.failed_constraints.push_back("cyber_authorization_invalid");
  if (!action.acknowledgement_available)
    report.failed_constraints.push_back("cyber_acknowledgement_unavailable");

  report.mess_executable = true;
  if (action.requires_mess) {
    if (action.mess_travel_time_s > action.mess_connection_deadline_s + 1e-12) {
      report.mess_executable = false;
      report.failed_constraints.push_back("mess_arrival_after_connection_deadline");
    }
    if (action.mess_required_energy_mwh >
        action.mess_available_energy_mwh + 1e-12) {
      report.mess_executable = false;
      report.failed_constraints.push_back("mess_soc_energy_insufficient");
    }
    if (action.requires_grid_forming_mess &&
        !action.mess_grid_forming_capable) {
      report.mess_executable = false;
      report.failed_constraints.push_back("mess_grid_forming_capability_missing");
    }
    const auto mobile = std::find_if(
        system_.mobile_storage.begin(), system_.mobile_storage.end(),
        [&](const MobileStorage& storage) {
          return storage.in_service &&
                 (action.mess_storage_index == 0 ||
                  storage.index == action.mess_storage_index);
        });
    if (mobile == system_.mobile_storage.end()) {
      report.mess_executable = false;
      report.failed_constraints.push_back("mess_asset_not_found");
    } else {
      if (action.requires_grid_forming_mess && !mobile->grid_forming) {
        report.mess_executable = false;
        report.failed_constraints.push_back(
            "mess_asset_grid_forming_capability_missing");
      }
      const int target_bus = action.mess_target_ac_bus != 0
                                 ? action.mess_target_ac_bus
                                 : mobile->target_bus;
      const bool target_exists = std::any_of(
          system_.ac.buses.begin(), system_.ac.buses.end(),
          [&](const ACBus& bus) {
            return bus.in_service && bus.index == target_bus;
          });
      if (!target_exists) {
        report.mess_executable = false;
        report.failed_constraints.push_back("mess_target_ac_bus_unavailable");
      }
    }
  }
  report.executable = report.cyber_executable && report.mess_executable;
  return report;
}

MultiFidelityCertificate MultiFidelityCertificateEngine::evaluate(
    const CertifiedRestorationAction& action, int fidelity_level) const {
  MultiFidelityCertificate certificate;
  certificate.action_id = action.id;
  certificate.fidelity_level = std::clamp(fidelity_level, 1, 3);
  auto simulation_options = oracle_options_;
  if (certificate.fidelity_level == 1) {
    simulation_options.dt_s *= certificate_options_.level1_step_multiplier;
    certificate.fidelity_scope = "coarse-step full-model DAE diagnostic";
  } else if (certificate.fidelity_level == 2) {
    simulation_options.dt_s *= certificate_options_.level2_step_multiplier;
    certificate.fidelity_scope = "fine-step full-model DAE diagnostic";
  } else {
    certificate.fidelity_scope = "configured full-DAE threshold oracle";
  }
  simulation_options.record_every_step = true;
  simulation_options.record_initial_state = true;
  simulation_options.record_device_outputs = true;
  simulation_options.output_every_steps = 1;
  simulation_options.max_recorded_snapshots = 0;

  HybridPowerSystem action_system = system_;
  int mess_dynamic_storage_index = -1;
  std::string mess_error;
  if (!materialize_mess(action, action_system, mess_dynamic_storage_index,
                        mess_error)) {
    certificate.label = CertificateLabel::Failed;
    certificate.limitations.push_back(mess_error);
    return certificate;
  }
  certificate.mess_materialized_in_dae = action.requires_mess;

  // A certificate oracle is fail-closed: model construction, power-flow
  // initialization, trajectory integration, and replay diagnostics may reject
  // an authored model, but that rejection must become an explicit failed
  // certificate rather than escape the resilience study. The mathematical
  // contract is documented in the resilience manual's certification chapter.
  std::string oracle_stage = "dynamic model construction";
  try {
  dynamics::DynamicModelBuilder builder;
  DynamicSystem simulation = builder.build(action_system, simulation_options);
  oracle_stage = "dynamic event installation";
  simulation.events = action.dynamic_events;
  oracle_stage = "DAE initialization and integration";
  dynamics::DynamicSolver solver;
  certificate.trajectory = solver.solve(simulation);
  oracle_stage = "trajectory post-processing";
  certificate.simulation_success = certificate.trajectory.success;
  for (const auto& snapshot : certificate.trajectory.snapshots) {
    certificate.mess_dynamic_device_observed =
        certificate.mess_dynamic_device_observed ||
        std::any_of(snapshot.device_outputs.begin(),
                    snapshot.device_outputs.end(),
                    [&](const dynamics::DynamicDeviceOutput& output) {
                      return output.component_index == mess_dynamic_storage_index &&
                             (output.type.find("GridForming") != std::string::npos ||
                              output.type.find("Battery") != std::string::npos);
                    });
  }
  if (!certificate.simulation_success || certificate.trajectory.snapshots.empty()) {
    certificate.label = CertificateLabel::Failed;
    certificate.limitations.push_back("dynamic simulation failed; no certificate is available");
    return certificate;
  }
  const Eigen::VectorXd state_scale = trajectory_state_scale(
      certificate.trajectory, certificate_options_.state_scale_floor);
  if (state_scale.size() > 0) {
    certificate.state_scale_min = state_scale.minCoeff();
    certificate.state_scale_max = state_scale.maxCoeff();
  }

  DynamicSystem replay = builder.build(action_system, simulation_options);
  replay.events = action.dynamic_events;
  const auto residual_indices = sample_indices(
      certificate.trajectory.snapshots.size(),
      certificate_options_.max_residual_samples);
  double previous_t = 0.0;
  double previous_rf = 0.0;
  double previous_scaled_rf = 0.0;
  double previous_rg = 0.0;
  double scaled_rf_integral = 0.0;
  bool first = true;
  for (const std::size_t index : residual_indices) {
    const auto& snapshot = certificate.trajectory.snapshots[index];
    replay.x.x = snapshot.state;
    replay.y = snapshot_algebraic(snapshot);
    dynamics::applyDynamicEventsThrough(replay, snapshot.time_s);
    Eigen::VectorXd rf;
    Eigen::VectorXd rg;
    std::string error;
    const Eigen::VectorXd xdot = snapshot_derivative(
        certificate.trajectory.snapshots, index, action.dynamic_events);
    if (!replay.evaluateDaeResidual(snapshot.time_s, snapshot.state, replay.y,
                                    xdot, rf, rg, error)) {
      certificate.label = CertificateLabel::Failed;
      certificate.limitations.push_back(error);
      return certificate;
    }
    const double rf_inf = vector_inf(rf);
    const double scaled_rf_inf =
        state_scale.size() == rf.size()
            ? vector_inf(rf.cwiseQuotient(state_scale))
            : rf_inf;
    const double rg_inf = vector_inf(rg);
    certificate.r_f.max_inf = std::max(certificate.r_f.max_inf, rf_inf);
    certificate.r_g.max_inf = std::max(certificate.r_g.max_inf, rg_inf);
    if (!first) {
      const double dt = snapshot.time_s - previous_t;
      if (dt > 0.0) {
        certificate.r_f.integral_inf += 0.5 * dt * (previous_rf + rf_inf);
        scaled_rf_integral +=
            0.5 * dt * (previous_scaled_rf + scaled_rf_inf);
        certificate.r_g.integral_inf += 0.5 * dt * (previous_rg + rg_inf);
      }
    }
    previous_t = snapshot.time_s;
    previous_rf = rf_inf;
    previous_scaled_rf = scaled_rf_inf;
    previous_rg = rg_inf;
    first = false;
    ++certificate.r_f.samples;
    ++certificate.r_g.samples;
  }

  DynamicSystem constants_replay = builder.build(action_system, simulation_options);
  constants_replay.events = action.dynamic_events;
  const double certificate_horizon = std::max(
      0.0, simulation_options.t_end_s - simulation_options.t_start_s);
  bool used_directional_proxy = false;
  Eigen::MatrixXd comparison_matrix;
  std::vector<ReducedPropagator> reduced_samples;
  certificate.constants_valid = sampled_constants(
      constants_replay, certificate.trajectory, certificate_options_,
      certificate.kappa_g, certificate.L_F, used_directional_proxy,
      comparison_matrix, reduced_samples, certificate.transition_gain,
      certificate.transition_growth_rate, state_scale,
      certificate_horizon);
  certificate.constant_scope = used_directional_proxy
      ? (reduced_samples.empty()
             ? "deterministic directional finite-difference sensitivity proxy on the computed trajectory; not an inverse norm or global bound"
             : "directional algebraic-conditioning proxy plus dense Jacobian of the network-solved reduced state field; sampled on the computed trajectory, not a global tube bound")
      : "sampled dense finite-difference Jacobians with infinity-norm logarithmic growth rate on the computed trajectory, inflated by declared tube factor; not global interval bounds";
  certificate.constants_are_global_bounds = false;
  if (!certificate.constants_valid) {
    certificate.limitations.push_back(
        "sampled algebraic Jacobian was singular or non-finite at one or more nodes");
  }

  const bool trapezoidal_method =
      simulation_options.solver_type ==
          dynamics::DynamicSolverType::TrapezoidalNewton ||
      (simulation_options.solver_type ==
           dynamics::DynamicSolverType::MassMatrixDae &&
       simulation_options.dae_step_method ==
           dynamics::DynamicDaeStepMethod::Trapezoidal);
  bool reconstruction_defect_valid = false;
  std::vector<DefectImpulse> defect_profile;
  if (trapezoidal_method) {
    DynamicSystem defect_replay =
        builder.build(action_system, simulation_options);
    defect_replay.events = action.dynamic_events;
    reconstruction_defect_valid = hermite_reconstruction_defect(
        defect_replay, certificate.trajectory, action.dynamic_events,
        state_scale, certificate.reconstruction_defect_integral,
        defect_profile);
  }
  if (reconstruction_defect_valid) {
    certificate.numerical_error =
        certificate.reconstruction_defect_integral;
    certificate.forcing_method =
        "cubic_hermite_continuous_defect_plus_algebraic_residual";
  } else {
    const double order = 2.0;
    certificate.numerical_error = std::pow(simulation_options.dt_s, order);
    certificate.forcing_method =
        "continuous_snapshot_residual_plus_nominal_order_term";
  }
  double total_forcing = std::numeric_limits<double>::infinity();
  if (certificate.constants_valid) {
    const double horizon = certificate_horizon;
    const double differential_forcing =
        reconstruction_defect_valid
            ? certificate.reconstruction_defect_integral
            : scaled_rf_integral;
    const double forcing = differential_forcing +
                           certificate.kappa_g * certificate.r_g.integral_inf +
                           (reconstruction_defect_valid
                                ? 0.0
                                : certificate.numerical_error);
    total_forcing = certificate.kappa_g * certificate.initial_error + forcing;
    const bool transition_envelope_available =
        !used_directional_proxy &&
        std::isfinite(certificate.transition_gain) &&
        certificate.transition_gain >= 1.0;
    if (transition_envelope_available) {
      certificate.eta = certificate.transition_gain * total_forcing;
      certificate.eta_method = "sampled_finite_horizon_semigroup_envelope";
    } else if (!used_directional_proxy && componentwise_comparison_eta(
            comparison_matrix, horizon,
            certificate.kappa_g * certificate.initial_error, forcing,
            certificate.eta, certificate.comparison_spectral_abscissa)) {
      certificate.eta_method =
          "componentwise_metzler_comparison_envelope";
    } else {
      const double exponent = std::min(700.0, certificate.L_F * horizon);
      certificate.eta =
          std::exp(exponent) *
          (certificate.kappa_g * certificate.initial_error + forcing);
      certificate.eta_method = "scalar_gronwall_fallback";
    }
  }

  const ObservedMargins observed =
      observed_margins(certificate.trajectory, certificate_options_);
  certificate.converter_current_observed =
      observed.converter_current_observed;
  certificate.current_limit_active = observed.current_limit_active;

  Eigen::VectorXd frequency_gradient;
  Eigen::VectorXd voltage_gradient;
  Eigen::VectorXd rocof_gradient;
  bool frequency_gradient_valid = false;
  bool voltage_gradient_valid = false;
  bool rocof_gradient_valid = false;
  if (certificate.constants_valid && !reduced_samples.empty() &&
      state_scale.size() > 0) {
    DynamicSystem frequency_replay =
        builder.build(action_system, simulation_options);
    frequency_replay.events = action.dynamic_events;
    frequency_gradient_valid = security_output_gradient(
        frequency_replay,
        certificate.trajectory.snapshots[observed.frequency_index],
        SecurityOutput::Frequency, 0, state_scale,
        certificate_options_.finite_difference_relative_step,
        frequency_gradient);

    DynamicSystem voltage_replay =
        builder.build(action_system, simulation_options);
    voltage_replay.events = action.dynamic_events;
    voltage_gradient_valid = security_output_gradient(
        voltage_replay,
        certificate.trajectory.snapshots[observed.voltage_index],
        SecurityOutput::Voltage, observed.voltage_phase_index, state_scale,
        certificate_options_.finite_difference_relative_step,
        voltage_gradient);

    if (observed.rocof_observed) {
      DynamicSystem rocof_replay =
          builder.build(action_system, simulation_options);
      rocof_replay.events = action.dynamic_events;
      rocof_gradient_valid = security_output_gradient(
          rocof_replay,
          certificate.trajectory.snapshots[observed.rocof_index],
          SecurityOutput::Rocof, 0, state_scale,
          certificate_options_.finite_difference_relative_step,
          rocof_gradient);
    }
  }

  const auto append_margin = [&](const std::string& name, double estimate,
                                 double fallback_lipschitz,
                                 const Eigen::VectorXd* output_gradient,
                                 const Eigen::VectorXd* second_gradient,
                                 double algebraic_direct_gain,
                                 double output_time_s) {
    MarginCertificate margin;
    margin.name = name;
    margin.estimate = estimate;
    margin.lipschitz = fallback_lipschitz;
    if (output_gradient != nullptr && output_gradient->size() > 0 &&
        std::isfinite(total_forcing)) {
      margin.lipschitz = output_gradient->lpNorm<1>();
      margin.output_transition_gain = output_transition_gain(
          *output_gradient, reduced_samples, certificate_horizon,
          certificate_options_.transition_time_probes);
      if (second_gradient != nullptr && second_gradient->size() > 0) {
        margin.lipschitz += second_gradient->lpNorm<1>();
        margin.output_transition_gain += output_transition_gain(
            *second_gradient, reduced_samples, certificate_horizon,
            certificate_options_.transition_time_probes);
      }
      margin.output_transition_gain *=
          certificate_options_.jacobian_tube_factor;
      margin.algebraic_direct_gain = algebraic_direct_gain;
      if (reconstruction_defect_valid) {
        double convolution = output_defect_convolution(
            *output_gradient, reduced_samples, defect_profile,
            output_time_s, certificate_horizon,
            certificate_options_.transition_time_probes);
        if (second_gradient != nullptr && second_gradient->size() > 0) {
          convolution = saturating_sum(
              convolution,
              output_defect_convolution(
                  *second_gradient, reduced_samples, defect_profile,
                  output_time_s, certificate_horizon,
                  certificate_options_.transition_time_probes));
        }
        convolution = saturating_product(
            certificate_options_.jacobian_tube_factor, convolution);
        const double algebraic_state_forcing =
            certificate.kappa_g *
            (certificate.initial_error + certificate.r_g.integral_inf);
        margin.delta_j = saturating_sum(
            convolution,
            saturating_sum(
                saturating_product(margin.output_transition_gain,
                                   algebraic_state_forcing),
                saturating_product(margin.algebraic_direct_gain,
                                   certificate.r_g.max_inf)));
        margin.bound_method = "sampled_output_defect_convolution";
      } else {
        margin.delta_j =
            saturating_product(margin.output_transition_gain, total_forcing);
        margin.bound_method = "sampled_output_semigroup_envelope";
      }
    } else {
      margin.delta_j =
          saturating_product(fallback_lipschitz, certificate.eta);
      margin.bound_method = "full_state_lipschitz_fallback";
    }
    margin.lower = margin.delta_j == std::numeric_limits<double>::max()
                       ? -std::numeric_limits<double>::max()
                       : estimate - margin.delta_j;
    margin.upper = margin.delta_j == std::numeric_limits<double>::max()
                       ? std::numeric_limits<double>::max()
                       : estimate + margin.delta_j;
    certificate.margins.push_back(std::move(margin));
  };
  append_margin("minimum_frequency", observed.frequency,
                50.0 * certificate.state_scale_max,
                frequency_gradient_valid ? &frequency_gradient : nullptr,
                nullptr, 0.0,
                certificate.trajectory.snapshots[observed.frequency_index]
                    .time_s);
  append_margin("minimum_ac_voltage", observed.voltage,
                certificate.constants_valid
                    ? std::max(1.0, certificate.kappa_g)
                    : std::numeric_limits<double>::infinity(),
                voltage_gradient_valid ? &voltage_gradient : nullptr,
                nullptr, certificate.kappa_g,
                certificate.trajectory.snapshots[observed.voltage_index]
                    .time_s);
  append_margin("maximum_rocof", observed.rocof,
                certificate.constants_valid
                    ? 50.0 * certificate.state_scale_max *
                          std::max(1.0, certificate.L_F)
                    : std::numeric_limits<double>::infinity(),
                rocof_gradient_valid ? &rocof_gradient : nullptr,
                nullptr,
                0.0,
                certificate.trajectory.snapshots[observed.rocof_index]
                    .time_s);
  bool interval_safe = certificate.constants_valid;
  bool interval_unsafe = false;
  for (const auto& margin : certificate.margins) {
    interval_safe = interval_safe && margin.lower >= 0.0;
    interval_unsafe = interval_unsafe || margin.upper < 0.0;
  }
  if (certificate.fidelity_level < 3) {
    certificate.label = interval_safe
                            ? CertificateLabel::Safe
                            : (interval_unsafe ? CertificateLabel::Unsafe
                                               : CertificateLabel::Unresolved);
    certificate.proof_valid = false;
    certificate.limitations.push_back(
        "L1/L2 labels are diagnostic because kappa_g and L_F are sampled estimates, not global enclosures");
    certificate.limitations.push_back(
        "output rows are sampled at nominal critical nodes; trajectory-tube and extremum-shift enclosures are not yet verified");
  } else {
    const bool current_observation_ok =
        !certificate_options_.require_converter_current_observation ||
        observed.converter_current_observed;
    const bool initial_state_qualified =
        certificate.trajectory.initialization.dynamic_trim_converged ||
        certificate.trajectory.initialization.dynamic_fast_dxdt_inf_norm <=
            certificate_options_.max_initial_dynamic_residual;
    const bool qualified =
        certificate.trajectory.initialization.power_flow_converged &&
        initial_state_qualified &&
        observed.frequency_observed && observed.voltage_observed &&
        observed.rocof_observed && current_observation_ok;
    const bool observed_safe = qualified &&
                               observed.frequency >= 0.0 &&
                               observed.voltage >= 0.0 && observed.rocof >= 0.0 &&
                               (!certificate_options_.require_converter_current_observation ||
                                !observed.current_limit_active);
    certificate.label = !qualified
                            ? CertificateLabel::Unresolved
                            : (observed_safe ? CertificateLabel::Safe
                                             : CertificateLabel::Unsafe);
    certificate.proof_valid = qualified;
    if (!qualified) {
      certificate.limitations.push_back(
          "L3 oracle is unresolved because initialization or required safety observations are incomplete");
    }
    certificate.limitations.push_back(
        "L3 validity is scenario- and horizon-specific full-DAE threshold verification, not a global analytic guarantee");
  }
  return certificate;
  } catch (const std::exception& error) {
    certificate.label = CertificateLabel::Failed;
    certificate.proof_valid = false;
    certificate.simulation_success = false;
    certificate.limitations.push_back(
        "DAE certificate oracle failed during " + oracle_stage + ": " +
        error.what());
    return certificate;
  } catch (const std::string& error) {
    certificate.label = CertificateLabel::Failed;
    certificate.proof_valid = false;
    certificate.simulation_success = false;
    certificate.limitations.push_back(
        "DAE certificate oracle failed during " + oracle_stage + ": " + error);
    return certificate;
  } catch (const char* error) {
    certificate.label = CertificateLabel::Failed;
    certificate.proof_valid = false;
    certificate.simulation_success = false;
    certificate.limitations.push_back(
        "DAE certificate oracle failed during " + oracle_stage + ": " +
        (error == nullptr ? std::string("null C-string exception")
                          : std::string(error)));
    return certificate;
  } catch (...) {
    certificate.label = CertificateLabel::Failed;
    certificate.proof_valid = false;
    certificate.simulation_success = false;
    certificate.limitations.push_back(
        "DAE certificate oracle failed during " + oracle_stage +
        " with an unrecognized non-standard or cross-ABI exception");
    return certificate;
  }
}

CertifiedRestorationResult CertifiedRestorationCoordinator::solve(
    const std::vector<CertifiedRestorationAction>& actions) const {
  CertifiedRestorationResult result;
  if (actions.empty()) {
    result.status = "empty restoration action catalog";
    return result;
  }
  std::vector<ExecutabilityReport> executability;
  executability.reserve(actions.size());
  std::vector<bool> excluded(actions.size(), false);
  for (const auto& action : actions) {
    executability.push_back(certificate_engine_.checkExecutability(action));
    if (!executability.back().executable) {
      result.master_filtered_actions.push_back(action.id);
    }
  }

  for (int iteration = 0; iteration < static_cast<int>(actions.size()); ++iteration) {
    const MasterSelection selected = solve_master(actions, excluded, executability);
    ++result.master_mip_solves;
    if (!selected.feasible) {
      result.optimality_proven_over_catalog =
          std::isfinite(result.incumbent_objective);
      result.success = std::isfinite(result.incumbent_objective);
      result.status = result.success
                          ? "catalog master exhausted; incumbent proven optimal over executable action catalog"
                          : "catalog master has no dynamically safe executable action: " +
                                selected.status;
      return result;
    }
    if (std::isfinite(result.incumbent_objective) &&
        selected.objective <= result.incumbent_objective + 1e-12) {
      result.optimality_proven_over_catalog = true;
      result.success = true;
      result.status =
          "master upper bound cannot improve incumbent; optimal over executable action catalog";
      return result;
    }

    const std::size_t index = static_cast<std::size_t>(selected.action_index);
    MasterIterationRecord record;
    record.iteration = iteration + 1;
    record.action_id = actions[index].id;
    record.master_objective = selected.objective;
    record.master_upper_bound = selected.objective;
    record.executability = executability[index];

    const MultiFidelityCertificate* decision = nullptr;
    for (int level = 1; level <= 2; ++level) {
      record.certificates.push_back(
          certificate_engine_.evaluate(actions[index], level));
      ++result.diagnostic_certificate_calls;
      if (record.certificates.back().label == CertificateLabel::Failed) break;
      if (record.certificates.back().proof_valid &&
          record.certificates.back().label != CertificateLabel::Unresolved) {
        decision = &record.certificates.back();
        break;
      }
    }
    if (decision == nullptr) {
      record.certificates.push_back(
          certificate_engine_.evaluate(actions[index], 3));
      ++result.dynamic_oracle_calls;
      decision = &record.certificates.back();
    }
    excluded[index] = true;
    if (decision->proof_valid && decision->label == CertificateLabel::Safe) {
      result.incumbent_objective = actions[index].objective;
      result.incumbent_action_id = actions[index].id;
      record.disposition =
          decision->fidelity_level < 3
              ? "verified lower-fidelity safe incumbent; full oracle skipped"
              : "safe incumbent; action excluded from next master solve";
    } else if (decision->proof_valid &&
               decision->label == CertificateLabel::Unsafe) {
      result.no_good_cuts.push_back(actions[index].id);
      record.disposition =
          decision->fidelity_level < 3
              ? "verified lower-fidelity unsafe; no-good cut z_action <= 0"
              : "dynamic-unsafe; no-good cut z_action <= 0";
    } else {
      record.disposition =
          "dynamic oracle unresolved; no exclusion cut is valid";
      result.iterations.push_back(std::move(record));
      result.success = std::isfinite(result.incumbent_objective);
      result.optimality_proven_over_catalog = false;
      result.status =
          "dynamic oracle unresolved for the current master optimum; catalog optimality is not proven";
      return result;
    }
    result.iterations.push_back(std::move(record));
  }
  result.success = std::isfinite(result.incumbent_objective);
  result.optimality_proven_over_catalog = result.success;
  result.status = result.success ? "all executable actions exhausted"
                                 : "no dynamically safe executable action";
  return result;
}

}  // namespace hacdcpf::analysis

// Kept in the certified-restoration translation unit so adding the MIP-to-DAE
// bridge does not create a second heavy Eigen matrix-functions compilation.
#include "resilience_dynamic_certification.inc"
