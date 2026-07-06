#include "hacdcpf/dynamics/devices/BasicDynamicDevices.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

namespace hacdcpf::dynamics {
namespace {

using Complex = std::complex<double>;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kMinVoltage = 1e-4;
constexpr double kMinTimeConstant = 1e-4;
constexpr double kMachinePowerBalanceTolPu = 1e-4;

int state_index(const StateIndexRange& range, int local) {
  return range.offset + local;
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

bool sparse_newton_step(const Eigen::MatrixXd& jac,
                        const Eigen::VectorXd& rhs,
                        Eigen::VectorXd& step) {
  Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
  lu.compute(dense_to_sparse(jac));
  if (lu.info() != Eigen::Success) return false;
  step = lu.solve(rhs);
  return lu.info() == Eigen::Success && step.allFinite();
}

Eigen::Vector3cd balanced_phasors(double vm, double theta_rad) {
  Eigen::Vector3cd v;
  v[0] = std::polar(vm, theta_rad);
  v[1] = std::polar(vm, theta_rad - 2.0 * kPi / 3.0);
  v[2] = std::polar(vm, theta_rad + 2.0 * kPi / 3.0);
  return v;
}

Eigen::Vector3cd bus_voltage(const NetworkState& y, int bus_pos) {
  Eigen::Vector3cd v = Eigen::Vector3cd::Ones();
  if (bus_pos < 0) return v;
  const int base = 3 * bus_pos;
  if (base + 2 >= y.Vac_abc.size()) return v;
  v[0] = y.Vac_abc[base + 0];
  v[1] = y.Vac_abc[base + 1];
  v[2] = y.Vac_abc[base + 2];
  return v;
}

double avg_voltage_mag(const Eigen::Vector3cd& v) {
  return std::sqrt((std::norm(v[0]) + std::norm(v[1]) + std::norm(v[2])) / 3.0);
}

Complex positive_sequence_voltage(const Eigen::Vector3cd& v) {
  const Complex a(-0.5, std::sqrt(3.0) / 2.0);
  return (v[0] + a * v[1] + a * a * v[2]) / 3.0;
}

std::pair<double, double> dq_from_phasor(Complex value, double theta_rad) {
  const Complex rotated = value * std::polar(1.0, -theta_rad);
  return {rotated.real(), rotated.imag()};
}

Complex phasor_from_dq(double d, double q, double theta_rad) {
  return Complex(d, q) * std::polar(1.0, theta_rad);
}

Eigen::Vector3cd balanced_current_from_dq(double id, double iq, double theta_rad) {
  const Complex ia = phasor_from_dq(id, iq, theta_rad);
  Eigen::Vector3cd current;
  current[0] = ia;
  current[1] = ia * std::polar(1.0, -2.0 * kPi / 3.0);
  current[2] = ia * std::polar(1.0, 2.0 * kPi / 3.0);
  return current;
}

Eigen::Vector3cd balanced_current_from_positive_sequence(Complex ia) {
  Eigen::Vector3cd current;
  current[0] = ia;
  current[1] = ia * std::polar(1.0, -2.0 * kPi / 3.0);
  current[2] = ia * std::polar(1.0, 2.0 * kPi / 3.0);
  return current;
}

Complex average_complex_power(const Eigen::Vector3cd& v,
                              const Eigen::Vector3cd& i) {
  Complex s{0.0, 0.0};
  for (int phase = 0; phase < 3; ++phase) {
    s += v[phase] * std::conj(i[phase]);
  }
  return s;
}

double phase_sum_to_total_power_scale(double phase_power_scale) {
  return 1.0 / std::max(1e-9, 3.0 * std::abs(phase_power_scale));
}

void add_balanced_current(DynamicStamp& stamp, int bus_pos, const Eigen::Vector3cd& current) {
  if (bus_pos < 0) return;
  for (int phase = 0; phase < 3; ++phase) {
    stamp.addAcCurrent(3 * bus_pos + phase, current[phase]);
  }
}

void add_balanced_admittance(DynamicStamp& stamp,
                             int bus_pos,
                             const Eigen::Matrix3cd& y_block) {
  if (bus_pos < 0) return;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      stamp.addAcAdmittance(3 * bus_pos + r, 3 * bus_pos + c, y_block(r, c));
    }
  }
}

bool valid_jacobian_index(const DynamicJacobianContext& context, int idx) {
  return idx >= 0 && idx < context.total_size;
}

void add_jacobian_triplet(const DynamicJacobianContext& context,
                          int row,
                          int col,
                          double value,
                          std::vector<Eigen::Triplet<double>>& triplets) {
  if (!valid_jacobian_index(context, row) ||
      !valid_jacobian_index(context, col) ||
      !std::isfinite(value) ||
      std::abs(value) <= 1e-14) {
    return;
  }
  triplets.emplace_back(row, col, value);
}

void add_ac_current_derivative(
    const DynamicJacobianContext& context,
    int ac_node,
    int col,
    Complex derivative,
    std::vector<Eigen::Triplet<double>>& triplets) {
  if (!context.validAcNode(ac_node) || !valid_jacobian_index(context, col)) return;
  add_jacobian_triplet(context, context.acRealRow(ac_node), col, derivative.real(), triplets);
  add_jacobian_triplet(context, context.acImagRow(ac_node), col, derivative.imag(), triplets);
}

void add_ac_current_voltage_derivative(
    const DynamicJacobianContext& context,
    int ac_node,
    Complex d_current_d_vr,
    Complex d_current_d_vi,
    std::vector<Eigen::Triplet<double>>& triplets) {
  if (!context.validAcNode(ac_node)) return;
  add_ac_current_derivative(context, ac_node, context.acRealCol(ac_node),
                            d_current_d_vr, triplets);
  add_ac_current_derivative(context, ac_node, context.acImagCol(ac_node),
                            d_current_d_vi, triplets);
}

void add_dc_current_derivative(
    const DynamicJacobianContext& context,
    int dc_node,
    int col,
    double derivative,
    std::vector<Eigen::Triplet<double>>& triplets) {
  if (!context.validDcNode(dc_node) || !valid_jacobian_index(context, col)) return;
  add_jacobian_triplet(context, context.dcRow(dc_node), col, derivative, triplets);
}

void add_balanced_current_derivative(
    const DynamicJacobianContext& context,
    int bus_pos,
    int col,
    const Eigen::Vector3cd& derivative,
    std::vector<Eigen::Triplet<double>>& triplets) {
  if (bus_pos < 0) return;
  for (int phase = 0; phase < 3; ++phase) {
    add_ac_current_derivative(context, 3 * bus_pos + phase, col, derivative[phase], triplets);
  }
}

template <typename CurrentFn>
void add_local_ac_voltage_current_derivative(
    const DynamicJacobianContext& context,
    int ac_node,
    Complex v,
    const CurrentFn& current,
    std::vector<Eigen::Triplet<double>>& triplets) {
  constexpr double eps = 1e-6;
  const double hr = eps * std::max(1.0, std::abs(v.real()));
  const double hi = eps * std::max(1.0, std::abs(v.imag()));
  const Complex d_vr =
      (current(Complex(v.real() + hr, v.imag())) -
       current(Complex(v.real() - hr, v.imag()))) /
      (2.0 * hr);
  const Complex d_vi =
      (current(Complex(v.real(), v.imag() + hi)) -
       current(Complex(v.real(), v.imag() - hi))) /
      (2.0 * hi);
  add_ac_current_voltage_derivative(context, ac_node, d_vr, d_vi, triplets);
}

double clamp_voltage_slope(double value) {
  return std::abs(value) > kMinVoltage ? (value >= 0.0 ? 1.0 : -1.0) : 0.0;
}

Eigen::Matrix3cd diagonal_admittance(Complex y) {
  Eigen::Matrix3cd matrix = Eigen::Matrix3cd::Zero();
  matrix(0, 0) = y;
  matrix(1, 1) = y;
  matrix(2, 2) = y;
  return matrix;
}

double clamp_voltage(double v) {
  if (!std::isfinite(v)) return 1.0;
  return std::max(kMinVoltage, std::abs(v));
}

double clamp_voltage_window(double v, double lo, double hi) {
  const double fallback = hi > lo ? std::clamp(1.0, lo, hi) : 1.0;
  if (!std::isfinite(v)) return fallback;
  if (hi > lo) return std::clamp(v, lo, hi);
  return std::max(kMinVoltage, std::abs(v));
}

double clamp01(double value, double lo, double hi) {
  return std::clamp(value, lo, hi);
}

double finite_value(double value, double fallback = 0.0) {
  return std::isfinite(value) ? value : fallback;
}

double positive_or(double value, double fallback) {
  return value > 0.0 ? value : fallback;
}

bool set_if_changed(Eigen::VectorXd& x, int idx, double value, double tol = 1e-10) {
  if (idx < 0 || idx >= x.size() || !std::isfinite(value)) return false;
  if (std::abs(x[idx] - value) <= tol * std::max({1.0, std::abs(x[idx]), std::abs(value)})) {
    return false;
  }
  x[idx] = value;
  return true;
}

double safe_base(double base_mva) {
  return std::max(1.0, base_mva);
}

double source_series_reactance_pu(double x_pu) {
  constexpr double kMinSourceReactance = 1e-8;
  if (!std::isfinite(x_pu)) return kMinSourceReactance;
  if (std::abs(x_pu) >= kMinSourceReactance) return x_pu;
  return x_pu < 0.0 ? -kMinSourceReactance : kMinSourceReactance;
}

double dc_link_network_power_pu(const NetworkState& y,
                                int dc_bus_pos,
                                double vdc_link,
                                double conductance_pu) {
  if (dc_bus_pos < 0 || dc_bus_pos >= y.Vdc.size() || conductance_pu <= 0.0) {
    return 0.0;
  }
  const double vdc_bus = finite_value(y.Vdc[dc_bus_pos], 1.0);
  return vdc_bus * conductance_pu * (vdc_link - vdc_bus);
}

double ac_to_dc_link_power_pu(double p_ac_pu, double eta) {
  const double eff = std::clamp(eta, 1e-6, 1.0);
  return p_ac_pu >= 0.0 ? p_ac_pu / eff : p_ac_pu * eff;
}

double dc_link_voltage_for_power_balance(const NetworkState& y,
                                         int dc_bus_pos,
                                         double p_ac_pu,
                                         double conductance_pu,
                                         double eta,
                                         double fallback_vdc,
                                         double vmin,
                                         double vmax) {
  if (dc_bus_pos < 0 || dc_bus_pos >= y.Vdc.size() || conductance_pu <= 0.0) {
    return clamp_voltage_window(fallback_vdc, vmin, vmax);
  }
  const double vdc_bus = clamp_voltage(y.Vdc[dc_bus_pos]);
  const double target_pdc = -ac_to_dc_link_power_pu(p_ac_pu, eta);
  const double vdc_link =
      vdc_bus + target_pdc / (conductance_pu * std::max(kMinVoltage, vdc_bus));
  return clamp_voltage_window(vdc_link, vmin, vmax);
}

double dc_link_voltage_derivative(double p_ac_pu,
                                  double p_dc_network_pu,
                                  double vdc_link,
                                  double capacitance_s,
                                  double eta,
                                  double vmin,
                                  double vmax) {
  const double c = std::max(kMinTimeConstant, capacitance_s);
  const double v = std::max(kMinVoltage, std::abs(vdc_link));
  double dvdt = -(ac_to_dc_link_power_pu(p_ac_pu, eta) + p_dc_network_pu) / (c * v);
  if (vdc_link <= vmin && dvdt < 0.0) dvdt = 0.0;
  if (vmax > vmin && vdc_link >= vmax && dvdt > 0.0) dvdt = 0.0;
  return finite_value(dvdt);
}

DynamicDeviceOutput make_output_base(const DynamicDevice& device,
                                     int bus,
                                     const std::string& canvas_type,
                                     const std::string& component_domain,
                                     const std::string& source_type) {
  DynamicDeviceOutput out;
  out.name = device.name();
  out.type = device.type();
  out.component_index = device.componentIndex();
  out.bus = bus;
  out.canvas_type = canvas_type;
  out.canvas_index = device.componentIndex();
  out.component_domain = component_domain;
  out.source_type = source_type;
  out.model_standard = device.modelStandard();
  out.model_name = device.modelName();
  out.parameter_set = device.parameterSet();
  out.model_profiles = device.modelProfiles();
  return out;
}

std::vector<DynamicModelProfile> profiles_or_default(
    const std::vector<DynamicModelProfile>& profiles,
    const DynamicDevice& device) {
  if (!profiles.empty()) return profiles;
  DynamicModelProfile profile;
  profile.standard = device.modelStandard();
  profile.model_name = device.modelName();
  profile.parameter_set = device.parameterSet();
  return {profile};
}

bool uses_kaura_pll(FrequencyEstimatorKind kind) {
  return kind == FrequencyEstimatorKind::KauraPLL;
}

int gfl_vdf_local(const GridFollowingInverterParams& params) {
  return uses_kaura_pll(params.frequency_estimator) ? 6 : -1;
}

int gfl_vqf_local(const GridFollowingInverterParams& params) {
  return uses_kaura_pll(params.frequency_estimator) ? 7 : -1;
}

int gfl_vdc_local(const GridFollowingInverterParams& params) {
  int local = 6;
  if (uses_kaura_pll(params.frequency_estimator)) local += 2;
  return params.dc_link_mode == DCLinkMode::DynamicDCVoltage &&
                 params.dc_bus_pos >= 0
             ? local
             : -1;
}

int gfl_state_count(const GridFollowingInverterParams& params) {
  int count = 6;
  if (uses_kaura_pll(params.frequency_estimator)) count += 2;
  if (gfl_vdc_local(params) >= 0) count += 1;
  return count;
}

std::pair<double, double> pll_measurement(const GridFollowingInverterParams& params,
                                          const DynamicState& x,
                                          const StateIndexRange& range,
                                          double vd_raw,
                                          double vq_raw) {
  if (uses_kaura_pll(params.frequency_estimator)) {
    return {x.x[state_index(range, gfl_vdf_local(params))],
            x.x[state_index(range, gfl_vqf_local(params))]};
  }
  return {vd_raw, vq_raw};
}

void add_voltage_metrics(DynamicDeviceOutput& out,
                         const NetworkState& y,
                         int ac_bus_pos,
                         int dc_bus_pos = -1) {
  if (ac_bus_pos >= 0) {
    const Eigen::Vector3cd v = bus_voltage(y, ac_bus_pos);
    out.values["v_avg_pu"] = avg_voltage_mag(v);
    out.values["v_pos_pu"] = std::abs(positive_sequence_voltage(v));
    out.values["v_a_pu"] = std::abs(v[0]);
    out.values["v_b_pu"] = std::abs(v[1]);
    out.values["v_c_pu"] = std::abs(v[2]);
  }
  if (dc_bus_pos >= 0 && dc_bus_pos < y.Vdc.size()) {
    out.values["vdc_pu"] = y.Vdc[dc_bus_pos];
  }
}

void add_generator_inner_outputs(DynamicDeviceOutput& out,
                                 const GeneratorInnerVariableSnapshot& snapshot) {
  out.values["composed_generator"] = 1.0;
  out.values["generator_inner_var_count"] =
      static_cast<double>(kGeneratorInnerVarCount);
  for (int i = 0; i < kGeneratorInnerVarCount; ++i) {
    const auto slot = static_cast<GeneratorInnerVar>(i);
    const std::string key = "inner_" + std::string(to_string(slot));
    out.values[key + "_present"] =
        snapshot.present[static_cast<std::size_t>(i)] ? 1.0 : 0.0;
    if (snapshot.present[static_cast<std::size_t>(i)]) {
      out.values[key] = snapshot.values[static_cast<std::size_t>(i)];
    }
  }
}

void add_generator_block_flags(DynamicDeviceOutput& out,
                               const GeneratorInnerVariableBus& bus) {
  out.values["block_machine"] = bus.has_machine ? 1.0 : 0.0;
  out.values["block_shaft"] = bus.has_shaft ? 1.0 : 0.0;
  out.values["block_avr"] = bus.has_avr ? 1.0 : 0.0;
  out.values["block_turbine_governor"] =
      bus.has_turbine_governor ? 1.0 : 0.0;
  out.values["block_pss"] = bus.has_pss ? 1.0 : 0.0;
}

GeneratorInnerVariableSnapshot generator_inner_base_snapshot(
    const GeneratorInnerVariableBus& bus,
    const DynamicState& x,
    const NetworkState& y) {
  GeneratorInnerVariableSnapshot snapshot;
  if (!bus.valid) return snapshot;
  const Complex v = positive_sequence_voltage(bus_voltage(y, bus.bus_pos));
  snapshot.set(GeneratorInnerVar::TerminalVoltageReal, v.real());
  snapshot.set(GeneratorInnerVar::TerminalVoltageImag, v.imag());
  snapshot.set(GeneratorInnerVar::StabilizerVoltage, 0.0);
  const int tau_idx = bus.mechanicalTorqueIndex();
  if (tau_idx >= 0 && tau_idx < x.x.size()) {
    snapshot.set(GeneratorInnerVar::MechanicalTorque, x.x[tau_idx]);
  }
  const int field_idx = bus.fieldVoltageIndex();
  if (field_idx >= 0 && field_idx < x.x.size()) {
    snapshot.set(GeneratorInnerVar::FieldVoltage, x.x[field_idx]);
  }
  return snapshot;
}

void add_inverter_inner_outputs(DynamicDeviceOutput& out,
                                const InverterInnerVariableSnapshot& snapshot) {
  out.values["composed_inverter"] = 1.0;
  out.values["inverter_inner_var_count"] =
      static_cast<double>(kInverterInnerVarCount);
  for (int i = 0; i < kInverterInnerVarCount; ++i) {
    const auto slot = static_cast<InverterInnerVar>(i);
    const std::string key = "inner_" + std::string(to_string(slot));
    out.values[key + "_present"] =
        snapshot.present[static_cast<std::size_t>(i)] ? 1.0 : 0.0;
    if (snapshot.present[static_cast<std::size_t>(i)]) {
      out.values[key] = snapshot.values[static_cast<std::size_t>(i)];
    }
  }
}

void add_inverter_block_flags(DynamicDeviceOutput& out,
                              const InverterInnerVariableBus& bus) {
  out.values["block_converter"] = bus.has_converter ? 1.0 : 0.0;
  out.values["block_dc_source"] = bus.has_dc_source ? 1.0 : 0.0;
  out.values["block_filter"] = bus.has_filter ? 1.0 : 0.0;
  out.values["block_frequency_estimator"] =
      bus.has_frequency_estimator ? 1.0 : 0.0;
  out.values["block_outer_control"] = bus.has_outer_control ? 1.0 : 0.0;
  out.values["block_inner_control"] = bus.has_inner_control ? 1.0 : 0.0;
  out.values["inverter_grid_following"] = bus.grid_following ? 1.0 : 0.0;
}

double control_time(double value) {
  return std::max(kMinTimeConstant, std::abs(value));
}

std::pair<double, double> low_pass_block(double u,
                                         double y,
                                         double k,
                                         double t) {
  const double tau = control_time(t);
  return {y, (k * u - y) / tau};
}

std::pair<double, double> low_pass_modified_block(double u,
                                                  double y,
                                                  double k,
                                                  double k_den,
                                                  double t) {
  const double tau = control_time(t);
  return {y, (k * u - k_den * y) / tau};
}

std::pair<double, double> high_pass_block(double u,
                                          double x,
                                          double k,
                                          double t) {
  const double tau = control_time(t);
  const double k_over_t = k / tau;
  return {x + k_over_t * u, -(k_over_t * u + x) / tau};
}

std::pair<double, double> lead_lag_block(double u,
                                         double x,
                                         double k,
                                         double t1,
                                         double t2) {
  const double denom = control_time(t2);
  const double ratio = t1 / denom;
  return {x + k * ratio * u, (k * (1.0 - ratio) * u - x) / denom};
}

struct SecondOrderLeadLagNonWindup {
  double output{0.0};
  double dx1{0.0};
  double dx2{0.0};
};

SecondOrderLeadLagNonWindup lead_lag_2nd_nonwindup(double u,
                                                   double x1,
                                                   double x2,
                                                   double t1,
                                                   double t2,
                                                   double t3,
                                                   double t4,
                                                   double y_min,
                                                   double y_max) {
  const double denom = control_time(t2);
  const double t4_over_t2 = std::abs(t2) < kMinTimeConstant ? 0.0 : t4 / t2;
  const double y = t4_over_t2 * u +
                   (t3 - t1 * t4_over_t2) * x1 +
                   (1.0 - t4_over_t2) * x2;
  const double y_sat = std::clamp(y, y_min, y_max);
  const double active = (y_min < y && y < y_max) ? 1.0 : 0.0;
  return {y_sat, active * (u - t1 * x1 - x2) / denom, active * x1};
}

std::pair<double, double> low_pass_nonwindup(double u,
                                             double y,
                                             double k,
                                             double t,
                                             double y_min,
                                             double y_max) {
  const double dydt_scaled = k * u - y;
  const double active =
      ((y >= y_max && dydt_scaled > 0.0) ||
       (y <= y_min && dydt_scaled < 0.0))
          ? 0.0
          : 1.0;
  return {std::clamp(y, y_min, y_max),
          active * dydt_scaled / control_time(t)};
}

double avr_saturation(double ae, double be, double vf) {
  if (ae == 0.0 && be == 0.0) return 0.0;
  return ae * std::exp(be * std::abs(vf));
}

double pss_output_limiter(double vss, double vct, double vcl, double vcu) {
  if (vcl == 0.0 || vcu == 0.0) return vss;
  return (vcl <= vct && vct <= vcu) ? vss : 0.0;
}

std::pair<double, double> limited_current(double id_ref,
                                          double iq_ref,
                                          double limit,
                                          bool reactive_priority) {
  if (limit <= 0.0) return {id_ref, iq_ref};
  const double imag = std::hypot(id_ref, iq_ref);
  if (imag <= limit || imag <= 1e-12) return {id_ref, iq_ref};
  if (!reactive_priority) {
    const double scale = limit / imag;
    return {id_ref * scale, iq_ref * scale};
  }
  const double iq = std::clamp(iq_ref, -limit, limit);
  const double id_abs = std::sqrt(std::max(0.0, limit * limit - iq * iq));
  const double id = id_ref < 0.0 ? -id_abs : id_abs;
  return {id, iq};
}

Complex load_power_pu(const ACLoadDynamicParams& params) {
  return Complex(params.p_mw / safe_base(params.base_mva) * params.scale,
                 params.q_mvar / safe_base(params.base_mva) * params.scale);
}

Complex load_consuming_current(Complex v,
                               double p_pu,
                               double q_pu,
                               double v0,
                               DynamicLoadModelKind kind,
                               const ACLoadDynamicParams& params) {
  const double vr = v.real();
  const double vi = v.imag();
  const double v0c = std::max(kMinVoltage, v0);
  const double v_floor = std::max(kMinVoltage, 0.20 * v0c);
  const double vmag = std::max(v_floor, std::abs(v));
  const double vmag2 = std::max(v_floor * v_floor, std::norm(v));
  auto current_from_weights = [&](double pz,
                                  double pi,
                                  double pp,
                                  double qz,
                                  double qi,
                                  double qp) {
    const double iz_re = (vr * pz + vi * qz) / (v0c * v0c);
    const double iz_im = (vi * pz - vr * qz) / (v0c * v0c);
    const double ii_re = (vr * pi + vi * qi) / (v0c * vmag);
    const double ii_im = (vi * pi - vr * qi) / (v0c * vmag);
    const double ip_re = (vr * pp + vi * qp) / vmag2;
    const double ip_im = (vi * pp - vr * qp) / vmag2;
    return Complex(iz_re + ii_re + ip_re, iz_im + ii_im + ip_im);
  };

  switch (kind) {
    case DynamicLoadModelKind::ConstantPower:
      return current_from_weights(0.0, 0.0, p_pu, 0.0, 0.0, q_pu);
    case DynamicLoadModelKind::ConstantCurrent:
      return current_from_weights(0.0, p_pu, 0.0, 0.0, q_pu, 0.0);
    case DynamicLoadModelKind::ConstantImpedance:
      return current_from_weights(p_pu, 0.0, 0.0, q_pu, 0.0, 0.0);
    case DynamicLoadModelKind::ZIP:
      return current_from_weights(p_pu * params.z_weight_p,
                                  p_pu * params.i_weight_p,
                                  p_pu * params.p_weight_p,
                                  q_pu * params.z_weight_q,
                                  q_pu * params.i_weight_q,
                                  q_pu * params.p_weight_q);
  }
  return current_from_weights(p_pu, 0.0, 0.0, q_pu, 0.0, 0.0);
}

struct GenrouParams {
  double r{0.0};
  double td0p{8.0};
  double td0pp{0.03};
  double tq0p{0.4};
  double tq0pp{0.05};
  double xd{1.8};
  double xq{1.7};
  double xdp{0.3};
  double xqp{0.55};
  double xdpp{0.25};
  double xl{0.2};
  double sat_a{0.0};
  double sat_b{0.0};
  bool exponential_saturation{false};
};

GenrouParams genrou_params(const VoltageSourceDynamicParams& params) {
  GenrouParams p;
  p.r = std::max(0.0, params.r_pu);
  p.xd = positive_or(params.xd_pu, 1.8);
  p.xq = positive_or(params.xq_pu, 1.7);
  p.xdp = positive_or(params.xdp_pu, 0.3);
  p.xqp = positive_or(params.xqp_pu, 0.55);
  p.xdpp = positive_or(params.xdpp_pu, positive_or(params.x_pu, 0.25));
  p.xl = positive_or(params.xl_pu, 0.2);
  p.td0p = std::max(kMinTimeConstant, positive_or(params.td0p_s, 8.0));
  p.td0pp = std::max(kMinTimeConstant, positive_or(params.td0pp_s, 0.03));
  p.tq0p = std::max(kMinTimeConstant, positive_or(params.tq0p_s, 0.4));
  p.tq0pp = std::max(kMinTimeConstant, positive_or(params.tq0pp_s, 0.05));
  p.sat_a = params.saturation_a;
  p.sat_b = params.saturation_b;
  p.exponential_saturation =
      params.machine_model == SynchronousMachineModelKind::GENROE;
  return p;
}

double genrou_gamma_d1(const GenrouParams& p) {
  return (p.xdpp - p.xl) / std::max(1e-9, p.xdp - p.xl);
}

double genrou_gamma_q1(const GenrouParams& p) {
  return (p.xdpp - p.xl) / std::max(1e-9, p.xqp - p.xl);
}

double genrou_gamma_d2(const GenrouParams& p) {
  const double denom = std::max(1e-9, p.xdp - p.xl);
  return (p.xdp - p.xdpp) / (denom * denom);
}

double genrou_gamma_q2(const GenrouParams& p) {
  const double denom = std::max(1e-9, p.xqp - p.xl);
  return (p.xqp - p.xdpp) / (denom * denom);
}

double genrou_gamma_qd(const GenrouParams& p) {
  return (p.xq - p.xl) / std::max(1e-9, p.xd - p.xl);
}

double genrou_saturation(const GenrouParams& p, double psi) {
  if (p.sat_a == 0.0 && p.sat_b == 0.0) return 0.0;
  const double x = std::max(kMinVoltage, psi);
  if (p.exponential_saturation) {
    return p.sat_b * std::pow(x, p.sat_a);
  }
  return p.sat_b * (x - p.sat_a) * (x - p.sat_a) / x;
}

std::pair<double, double> psd_ri_to_dq(double delta, Complex v) {
  const double vd = std::sin(delta) * v.real() - std::cos(delta) * v.imag();
  const double vq = std::cos(delta) * v.real() + std::sin(delta) * v.imag();
  return {vd, vq};
}

Complex psd_dq_to_ri(double delta, double id, double iq) {
  const double ir = std::sin(delta) * id + std::cos(delta) * iq;
  const double ii = -std::cos(delta) * id + std::sin(delta) * iq;
  return Complex(ir, ii);
}

bool machine_is_roundrotor(const VoltageSourceDynamicParams& params) {
  return params.machine_model == SynchronousMachineModelKind::GENROU ||
         params.machine_model == SynchronousMachineModelKind::GENROE ||
         params.psd_genrou_model;
}

bool machine_is_onedoneq(const VoltageSourceDynamicParams& params) {
  return params.machine_model == SynchronousMachineModelKind::OneDOneQ;
}

bool machine_is_simple_marconato(const VoltageSourceDynamicParams& params) {
  return params.machine_model == SynchronousMachineModelKind::SimpleMarconato ||
         params.machine_model == SynchronousMachineModelKind::SimpleAF;
}

bool machine_is_marconato(const VoltageSourceDynamicParams& params) {
  return params.machine_model == SynchronousMachineModelKind::Marconato ||
         params.machine_model == SynchronousMachineModelKind::AndersonFouad;
}

bool machine_is_sauerpai(const VoltageSourceDynamicParams& params) {
  return params.machine_model == SynchronousMachineModelKind::SauerPai;
}

bool machine_is_salient(const VoltageSourceDynamicParams& params) {
  return params.machine_model == SynchronousMachineModelKind::GENSAL ||
         params.machine_model == SynchronousMachineModelKind::GENSAE;
}

int synchronous_machine_state_count(const VoltageSourceDynamicParams& params) {
  if (machine_is_sauerpai(params)) return 10;
  if (machine_is_marconato(params)) return 10;
  if (machine_is_simple_marconato(params)) return 8;
  if (machine_is_roundrotor(params)) return 8;
  if (machine_is_salient(params)) return 7;
  if (machine_is_onedoneq(params)) return 6;
  return 4;
}

std::string synchronous_machine_model_name(const VoltageSourceDynamicParams& params) {
  if (machine_is_onedoneq(params)) return "OneDOneQMachine";
  if (params.machine_model == SynchronousMachineModelKind::AndersonFouad) {
    return "AndersonFouadMachine";
  }
  if (params.machine_model == SynchronousMachineModelKind::SimpleAF) {
    return "SimpleAFMachine";
  }
  if (machine_is_sauerpai(params)) return "SauerPaiMachine";
  if (machine_is_marconato(params)) return "MarconatoMachine";
  if (machine_is_simple_marconato(params)) return "SimpleMarconatoMachine";
  if (machine_is_roundrotor(params) || machine_is_salient(params)) {
    return (!params.machine_model_name.empty() &&
            params.machine_model_name != "ClassicalMachine")
               ? params.machine_model_name
               : (machine_is_salient(params) ? "GENSAL" : "GENROU");
  }
  if (!params.machine_model_name.empty()) return params.machine_model_name;
  return "ClassicalMachine";
}

struct OneDOneQParams {
  double r{0.0};
  double td0p{5.89};
  double tq0p{0.6};
  double xd{1.3125};
  double xq{1.2578};
  double xdp{0.1813};
  double xqp{0.25};
};

OneDOneQParams onedoneq_params(const VoltageSourceDynamicParams& params) {
  OneDOneQParams p;
  p.r = std::max(0.0, params.r_pu);
  p.xd = positive_or(params.xd_pu, 1.3125);
  p.xq = positive_or(params.xq_pu, 1.2578);
  p.xdp = positive_or(params.xdp_pu, positive_or(params.x_pu, 0.1813));
  p.xqp = positive_or(params.xqp_pu, 0.25);
  p.td0p = std::max(kMinTimeConstant, positive_or(params.td0p_s, 5.89));
  p.tq0p = std::max(kMinTimeConstant, positive_or(params.tq0p_s, 0.6));
  return p;
}

struct OneDOneQEval {
  double id{0.0};
  double iq{0.0};
  double pe{0.0};
  double qe{0.0};
  double tau_e{0.0};
  double vd{0.0};
  double vq{0.0};
  Complex current{0.0, 0.0};
};

OneDOneQEval evaluate_onedoneq(const OneDOneQParams& p,
                               Complex v,
                               double delta,
                               double eq_p,
                               double ed_p) {
  OneDOneQEval out;
  const auto [vd, vq] = psd_ri_to_dq(delta, v);
  out.vd = vd;
  out.vq = vq;
  const double denom = std::max(1e-9, p.r * p.r + p.xdp * p.xqp);
  out.id = (p.xqp * (eq_p - vq) + p.r * (ed_p - vd)) / denom;
  out.iq = (-p.xdp * (ed_p - vd) + p.r * (eq_p - vq)) / denom;
  out.pe = (vd + p.r * out.id) * out.id + (vq + p.r * out.iq) * out.iq;
  out.qe = vq * out.id - vd * out.iq;
  out.tau_e = out.pe;
  out.current = psd_dq_to_ri(delta, out.id, out.iq);
  return out;
}

Eigen::VectorXd onedoneq_initial_residual(const OneDOneQParams& p,
                                          Complex v,
                                          double p0,
                                          double q0,
                                          const Eigen::VectorXd& z) {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(5);
  const double delta = z[0];
  const double tau_m = z[1];
  const double vf = z[2];
  const double eq_p = z[3];
  const double ed_p = z[4];
  const OneDOneQEval e = evaluate_onedoneq(p, v, delta, eq_p, ed_p);
  r[0] = tau_m - e.tau_e;
  r[1] = p0 - e.pe;
  r[2] = q0 - e.qe;
  r[3] = -eq_p - (p.xd - p.xdp) * e.id + vf;
  r[4] = -ed_p + (p.xq - p.xqp) * e.iq;
  return r;
}

Eigen::VectorXd solve_onedoneq_initial_conditions(const OneDOneQParams& p,
                                                  Complex v,
                                                  double p0,
                                                  double q0,
                                                  Eigen::VectorXd z) {
  Eigen::VectorXd best = z;
  double best_norm =
      onedoneq_initial_residual(p, v, p0, q0, z).lpNorm<Eigen::Infinity>();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int iter = 0; iter < 30; ++iter) {
    const Eigen::VectorXd r = onedoneq_initial_residual(p, v, p0, q0, z);
    const double norm = r.lpNorm<Eigen::Infinity>();
    if (norm < best_norm) {
      best = z;
      best_norm = norm;
    }
    if (norm <= 1e-10) return z;
    Eigen::MatrixXd jac(5, 5);
    for (int col = 0; col < 5; ++col) {
      Eigen::VectorXd zp = z;
      const double h = eps * std::max(1.0, std::abs(z[col]));
      zp[col] += h;
      jac.col(col) = (onedoneq_initial_residual(p, v, p0, q0, zp) - r) / h;
    }
    Eigen::VectorXd step;
    if (!sparse_newton_step(jac, -r, step)) break;
    bool accepted = false;
    double alpha = 1.0;
    while (alpha >= 1.0 / 1024.0) {
      const Eigen::VectorXd trial = z + alpha * step;
      const double trial_norm =
          onedoneq_initial_residual(p, v, p0, q0, trial).lpNorm<Eigen::Infinity>();
      if (std::isfinite(trial_norm) && trial_norm < norm) {
        z = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return best;
}

struct SimpleMarconatoParams {
  double r{0.0};
  double td0p{5.89};
  double tq0p{0.6};
  double td0pp{0.5};
  double tq0pp{0.023};
  double xd{1.3125};
  double xq{1.2578};
  double xdp{0.1813};
  double xqp{0.25};
  double xdpp{0.14};
  double xqpp{0.18};
  double t_aa{0.0};
  double gamma_d{0.0};
  double gamma_q{0.0};
};

double marconato_gamma(double tpp,
                       double xpp,
                       double tp,
                       double xp,
                       double x,
                       double fallback) {
  const double denom = tp * xp;
  if (std::abs(denom) <= 1e-12) return fallback;
  return (tpp * xpp / denom) * (x - xp);
}

SimpleMarconatoParams simple_marconato_params(const VoltageSourceDynamicParams& params) {
  SimpleMarconatoParams p;
  p.r = std::max(0.0, params.r_pu);
  p.xd = positive_or(params.xd_pu, 1.3125);
  p.xq = positive_or(params.xq_pu, 1.2578);
  p.xdp = positive_or(params.xdp_pu, positive_or(params.x_pu, 0.1813));
  p.xqp = positive_or(params.xqp_pu, 0.25);
  p.xdpp = positive_or(params.xdpp_pu, 0.14);
  p.xqpp = positive_or(params.xqpp_pu, 0.18);
  p.td0p = std::max(kMinTimeConstant, positive_or(params.td0p_s, 5.89));
  p.tq0p = std::max(kMinTimeConstant, positive_or(params.tq0p_s, 0.6));
  p.td0pp = std::max(kMinTimeConstant, positive_or(params.td0pp_s, 0.5));
  p.tq0pp = std::max(kMinTimeConstant, positive_or(params.tq0pp_s, 0.023));
  p.t_aa = std::max(0.0, params.t_aa_s);
  const bool anderson_family =
      params.machine_model == SynchronousMachineModelKind::SimpleAF ||
      params.machine_model == SynchronousMachineModelKind::AndersonFouad;
  if (anderson_family) {
    p.t_aa = 0.0;
    p.gamma_d = 0.0;
    p.gamma_q = 0.0;
  } else {
    p.gamma_d = marconato_gamma(p.td0pp, p.xdpp, p.td0p, p.xdp, p.xd, 0.0);
    p.gamma_q = marconato_gamma(p.tq0pp, p.xqpp, p.tq0p, p.xqp, p.xq, 0.0);
  }
  return p;
}

struct SimpleMarconatoEval {
  double id{0.0};
  double iq{0.0};
  double pe{0.0};
  double qe{0.0};
  double tau_e{0.0};
  double vd{0.0};
  double vq{0.0};
  Complex current{0.0, 0.0};
};

SimpleMarconatoEval evaluate_simple_marconato(const SimpleMarconatoParams& p,
                                              Complex v,
                                              double delta,
                                              double eq_p,
                                              double ed_p,
                                              double eq_pp,
                                              double ed_pp) {
  (void)eq_p;
  (void)ed_p;
  SimpleMarconatoEval out;
  const auto [vd, vq] = psd_ri_to_dq(delta, v);
  out.vd = vd;
  out.vq = vq;
  const double denom = std::max(1e-9, p.r * p.r + p.xdpp * p.xqpp);
  out.id = (p.xqpp * (eq_pp - vq) + p.r * (ed_pp - vd)) / denom;
  out.iq = (-p.xdpp * (ed_pp - vd) + p.r * (eq_pp - vq)) / denom;
  out.tau_e = (vd + p.r * out.id) * out.id + (vq + p.r * out.iq) * out.iq;
  out.pe = vd * out.id + vq * out.iq;
  out.qe = vq * out.id - vd * out.iq;
  out.current = psd_dq_to_ri(delta, out.id, out.iq);
  return out;
}

Eigen::VectorXd simple_marconato_initial_residual(const SimpleMarconatoParams& p,
                                                  Complex v,
                                                  double p0,
                                                  double q0,
                                                  const Eigen::VectorXd& z) {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(7);
  const double delta = z[0];
  const double tau_m = z[1];
  const double vf = z[2];
  const double eq_p = z[3];
  const double ed_p = z[4];
  const double eq_pp = z[5];
  const double ed_pp = z[6];
  const SimpleMarconatoEval e =
      evaluate_simple_marconato(p, v, delta, eq_p, ed_p, eq_pp, ed_pp);
  r[0] = tau_m - e.tau_e;
  r[1] = p0 - e.pe;
  r[2] = q0 - e.qe;
  r[3] = -eq_p - (p.xd - p.xdp - p.gamma_d) * e.id +
         (1.0 - p.t_aa / p.td0p) * vf;
  r[4] = -ed_p + (p.xq - p.xqp - p.gamma_q) * e.iq;
  r[5] = -eq_pp + eq_p - (p.xdp - p.xdpp + p.gamma_d) * e.id +
         (p.t_aa / p.td0p) * vf;
  r[6] = -ed_pp + ed_p + (p.xqp - p.xqpp + p.gamma_q) * e.iq;
  return r;
}

Eigen::VectorXd solve_simple_marconato_initial_conditions(const SimpleMarconatoParams& p,
                                                          Complex v,
                                                          double p0,
                                                          double q0,
                                                          Eigen::VectorXd z) {
  Eigen::VectorXd best = z;
  double best_norm =
      simple_marconato_initial_residual(p, v, p0, q0, z).lpNorm<Eigen::Infinity>();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int iter = 0; iter < 30; ++iter) {
    const Eigen::VectorXd r = simple_marconato_initial_residual(p, v, p0, q0, z);
    const double norm = r.lpNorm<Eigen::Infinity>();
    if (norm < best_norm) {
      best = z;
      best_norm = norm;
    }
    if (norm <= 1e-10) return z;
    Eigen::MatrixXd jac(7, 7);
    for (int col = 0; col < 7; ++col) {
      Eigen::VectorXd zp = z;
      const double h = eps * std::max(1.0, std::abs(z[col]));
      zp[col] += h;
      jac.col(col) = (simple_marconato_initial_residual(p, v, p0, q0, zp) - r) / h;
    }
    Eigen::VectorXd step;
    if (!sparse_newton_step(jac, -r, step)) break;
    bool accepted = false;
    double alpha = 1.0;
    while (alpha >= 1.0 / 1024.0) {
      const Eigen::VectorXd trial = z + alpha * step;
      const double trial_norm =
          simple_marconato_initial_residual(p, v, p0, q0, trial).lpNorm<Eigen::Infinity>();
      if (std::isfinite(trial_norm) && trial_norm < norm) {
        z = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return best;
}

struct MarconatoEval {
  double id{0.0};
  double iq{0.0};
  double pe{0.0};
  double qe{0.0};
  double tau_e{0.0};
  double vd{0.0};
  double vq{0.0};
  Complex current{0.0, 0.0};
};

MarconatoEval evaluate_marconato(const SimpleMarconatoParams& p,
                                 Complex v,
                                 double delta,
                                 double psi_q,
                                 double psi_d,
                                 double eq_p,
                                 double ed_p,
                                 double eq_pp,
                                 double ed_pp) {
  (void)eq_p;
  (void)ed_p;
  MarconatoEval out;
  const auto [vd, vq] = psd_ri_to_dq(delta, v);
  out.vd = vd;
  out.vq = vq;
  out.id = (eq_pp - psi_d) / std::max(1e-9, p.xdpp);
  out.iq = (-ed_pp - psi_q) / std::max(1e-9, p.xqpp);
  out.tau_e = psi_d * out.iq - psi_q * out.id;
  out.pe = vd * out.id + vq * out.iq;
  out.qe = vq * out.id - vd * out.iq;
  out.current = psd_dq_to_ri(delta, out.id, out.iq);
  return out;
}

Eigen::VectorXd marconato_initial_residual(const SimpleMarconatoParams& p,
                                           Complex v,
                                           double p0,
                                           double q0,
                                           const Eigen::VectorXd& z) {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(9);
  const double delta = z[0];
  const double tau_m = z[1];
  const double vf = z[2];
  const double psi_q = z[3];
  const double psi_d = z[4];
  const double eq_p = z[5];
  const double ed_p = z[6];
  const double eq_pp = z[7];
  const double ed_pp = z[8];
  const MarconatoEval e =
      evaluate_marconato(p, v, delta, psi_q, psi_d, eq_p, ed_p, eq_pp, ed_pp);
  r[0] = tau_m - e.tau_e;
  r[1] = p0 - e.pe;
  r[2] = q0 - e.qe;
  r[3] = p.r * e.iq - psi_d + e.vq;
  r[4] = p.r * e.id + psi_q + e.vd;
  r[5] = -eq_p - (p.xd - p.xdp - p.gamma_d) * e.id +
         (1.0 - p.t_aa / p.td0p) * vf;
  r[6] = -ed_p + (p.xq - p.xqp - p.gamma_q) * e.iq;
  r[7] = -eq_pp + eq_p - (p.xdp - p.xdpp + p.gamma_d) * e.id +
         (p.t_aa / p.td0p) * vf;
  r[8] = -ed_pp + ed_p + (p.xqp - p.xqpp + p.gamma_q) * e.iq;
  return r;
}

Eigen::VectorXd solve_marconato_initial_conditions(const SimpleMarconatoParams& p,
                                                   Complex v,
                                                   double p0,
                                                   double q0,
                                                   Eigen::VectorXd z) {
  Eigen::VectorXd best = z;
  double best_norm =
      marconato_initial_residual(p, v, p0, q0, z).lpNorm<Eigen::Infinity>();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int iter = 0; iter < 40; ++iter) {
    const Eigen::VectorXd r = marconato_initial_residual(p, v, p0, q0, z);
    const double norm = r.lpNorm<Eigen::Infinity>();
    if (norm < best_norm) {
      best = z;
      best_norm = norm;
    }
    if (norm <= 1e-10) return z;
    Eigen::MatrixXd jac(9, 9);
    for (int col = 0; col < 9; ++col) {
      Eigen::VectorXd zp = z;
      const double h = eps * std::max(1.0, std::abs(z[col]));
      zp[col] += h;
      jac.col(col) = (marconato_initial_residual(p, v, p0, q0, zp) - r) / h;
    }
    Eigen::VectorXd step;
    if (!sparse_newton_step(jac, -r, step)) break;
    bool accepted = false;
    double alpha = 1.0;
    while (alpha >= 1.0 / 2048.0) {
      const Eigen::VectorXd trial = z + alpha * step;
      const double trial_norm =
          marconato_initial_residual(p, v, p0, q0, trial).lpNorm<Eigen::Infinity>();
      if (std::isfinite(trial_norm) && trial_norm < norm) {
        z = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return best;
}

struct SauerPaiParams {
  double r{0.002};
  double xd{1.79};
  double xq{1.71};
  double xdp{0.169};
  double xqp{0.228};
  double xdpp{0.135};
  double xqpp{0.2};
  double xl{0.13};
  double td0p{4.3};
  double tq0p{0.85};
  double td0pp{0.032};
  double tq0pp{0.05};
  double gamma_d1{0.0};
  double gamma_q1{0.0};
  double gamma_d2{0.0};
  double gamma_q2{0.0};
};

double sauerpai_ratio(double numerator, double denominator, double fallback) {
  return std::abs(denominator) > 1e-12 ? numerator / denominator : fallback;
}

SauerPaiParams sauerpai_params(const VoltageSourceDynamicParams& params) {
  SauerPaiParams p;
  p.r = std::max(0.0, positive_or(params.r_pu, 0.002));
  p.xd = positive_or(params.xd_pu, 1.79);
  p.xq = positive_or(params.xq_pu, 1.71);
  p.xdp = positive_or(params.xdp_pu, 0.169);
  p.xqp = positive_or(params.xqp_pu, 0.228);
  p.xdpp = positive_or(params.xdpp_pu, 0.135);
  p.xqpp = positive_or(params.xqpp_pu, 0.2);
  p.xl = positive_or(params.xl_pu, 0.13);
  p.td0p = std::max(kMinTimeConstant, positive_or(params.td0p_s, 4.3));
  p.tq0p = std::max(kMinTimeConstant, positive_or(params.tq0p_s, 0.85));
  p.td0pp = std::max(kMinTimeConstant, positive_or(params.td0pp_s, 0.032));
  p.tq0pp = std::max(kMinTimeConstant, positive_or(params.tq0pp_s, 0.05));
  p.gamma_d1 = sauerpai_ratio(p.xdpp - p.xl, p.xdp - p.xl, 0.0);
  p.gamma_q1 = sauerpai_ratio(p.xqpp - p.xl, p.xqp - p.xl, 0.0);
  p.gamma_d2 =
      sauerpai_ratio(p.xdp - p.xdpp, (p.xdp - p.xl) * (p.xdp - p.xl), 0.0);
  p.gamma_q2 =
      sauerpai_ratio(p.xqp - p.xqpp, (p.xqp - p.xl) * (p.xqp - p.xl), 0.0);
  return p;
}

struct SauerPaiEval {
  double id{0.0};
  double iq{0.0};
  double pe{0.0};
  double qe{0.0};
  double tau_e{0.0};
  double vd{0.0};
  double vq{0.0};
  Complex current{0.0, 0.0};
};

SauerPaiEval evaluate_sauerpai(const SauerPaiParams& p,
                               Complex v,
                               double delta,
                               double psi_q,
                               double psi_d,
                               double eq_p,
                               double ed_p,
                               double psi_d_pp,
                               double psi_q_pp) {
  SauerPaiEval out;
  const auto [vd, vq] = psd_ri_to_dq(delta, v);
  out.vd = vd;
  out.vq = vq;
  out.id = (p.gamma_d1 * eq_p - psi_d +
            (1.0 - p.gamma_d1) * psi_d_pp) /
           std::max(1e-9, p.xdpp);
  out.iq = (-p.gamma_q1 * ed_p - psi_q +
            (1.0 - p.gamma_q1) * psi_q_pp) /
           std::max(1e-9, p.xqpp);
  out.tau_e = psi_d * out.iq - psi_q * out.id;
  out.pe = vd * out.id + vq * out.iq;
  out.qe = vq * out.id - vd * out.iq;
  out.current = psd_dq_to_ri(delta, out.id, out.iq);
  return out;
}

Eigen::VectorXd sauerpai_initial_residual(const SauerPaiParams& p,
                                          Complex v,
                                          double p0,
                                          double q0,
                                          const Eigen::VectorXd& z) {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(9);
  const double delta = z[0];
  const double tau_m = z[1];
  const double vf = z[2];
  const double psi_q = z[3];
  const double psi_d = z[4];
  const double eq_p = z[5];
  const double ed_p = z[6];
  const double psi_d_pp = z[7];
  const double psi_q_pp = z[8];
  const SauerPaiEval e =
      evaluate_sauerpai(p, v, delta, psi_q, psi_d, eq_p, ed_p, psi_d_pp, psi_q_pp);
  r[0] = tau_m - e.tau_e;
  r[1] = p0 - e.pe;
  r[2] = q0 - e.qe;
  r[3] = p.r * e.iq - psi_d + e.vq;
  r[4] = p.r * e.id + psi_q + e.vd;
  r[5] = -eq_p -
         (p.xd - p.xdp) *
             (e.id - p.gamma_d2 * psi_d_pp -
              (1.0 - p.gamma_d1) * e.id + p.gamma_d2 * eq_p) +
         vf;
  r[6] = -ed_p +
         (p.xq - p.xqp) *
             (e.iq - p.gamma_q2 * psi_q_pp -
              (1.0 - p.gamma_q1) * e.iq - p.gamma_d2 * ed_p);
  r[7] = -psi_d_pp + eq_p - (p.xdp - p.xl) * e.id;
  r[8] = -psi_q_pp - ed_p - (p.xqp - p.xl) * e.iq;
  return r;
}

Eigen::VectorXd solve_sauerpai_initial_conditions(const SauerPaiParams& p,
                                                  Complex v,
                                                  double p0,
                                                  double q0,
                                                  Eigen::VectorXd z) {
  Eigen::VectorXd best = z;
  double best_norm =
      sauerpai_initial_residual(p, v, p0, q0, z).lpNorm<Eigen::Infinity>();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int iter = 0; iter < 50; ++iter) {
    const Eigen::VectorXd r = sauerpai_initial_residual(p, v, p0, q0, z);
    const double norm = r.lpNorm<Eigen::Infinity>();
    if (norm < best_norm) {
      best = z;
      best_norm = norm;
    }
    if (norm <= 1e-10) return z;
    Eigen::MatrixXd jac(9, 9);
    for (int col = 0; col < 9; ++col) {
      Eigen::VectorXd zp = z;
      const double h = eps * std::max(1.0, std::abs(z[col]));
      zp[col] += h;
      jac.col(col) = (sauerpai_initial_residual(p, v, p0, q0, zp) - r) / h;
    }
    Eigen::VectorXd step;
    if (!sparse_newton_step(jac, -r, step)) break;
    bool accepted = false;
    double alpha = 1.0;
    while (alpha >= 1.0 / 4096.0) {
      const Eigen::VectorXd trial = z + alpha * step;
      const double trial_norm =
          sauerpai_initial_residual(p, v, p0, q0, trial).lpNorm<Eigen::Infinity>();
      if (std::isfinite(trial_norm) && trial_norm < norm) {
        z = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return best;
}

struct GenrouEval {
  double id{0.0};
  double iq{0.0};
  double pe{0.0};
  double qe{0.0};
  double tau_e{0.0};
  double xad_ifd{0.0};
  double xaq_i1q{0.0};
  double psi_d_pp{0.0};
  double psi_q_pp{0.0};
  double vf{0.0};
  Complex current{0.0, 0.0};
};

GenrouEval evaluate_genrou(const GenrouParams& p,
                           Complex v,
                           double delta,
                           double eq_p,
                           double ed_p,
                           double psi_kd,
                           double psi_kq,
                           double vf) {
  GenrouEval out;
  const auto [vd, vq] = psd_ri_to_dq(delta, v);
  const double gd1 = genrou_gamma_d1(p);
  const double gq1 = genrou_gamma_q1(p);
  const double gd2 = genrou_gamma_d2(p);
  const double gq2 = genrou_gamma_q2(p);
  const double gqd = genrou_gamma_qd(p);
  out.psi_q_pp = gq1 * ed_p + psi_kq * (1.0 - gq1);
  out.psi_d_pp = gd1 * eq_p + gd2 * (p.xdp - p.xl) * psi_kd;
  const double denom = std::max(1e-9, p.r * p.r + p.xdpp * p.xdpp);
  out.id = (-p.r * (vd - out.psi_q_pp) + p.xdpp * (-vq + out.psi_d_pp)) / denom;
  out.iq = (p.xdpp * (vd - out.psi_q_pp) + p.r * (-vq + out.psi_d_pp)) / denom;
  const double psi_pp = std::hypot(out.psi_d_pp, out.psi_q_pp);
  const double se = genrou_saturation(p, psi_pp);
  out.xad_ifd = eq_p +
                (p.xd - p.xdp) * (gd1 * out.id - gd2 * psi_kd + gd2 * eq_p) +
                se * out.psi_d_pp;
  out.xaq_i1q =
      ed_p +
      (p.xq - p.xqp) * (gq2 * ed_p - gq2 * psi_kq - gq1 * out.iq) +
      se * out.psi_q_pp * gqd;
  out.tau_e = out.id * (vd + out.id * p.r) + out.iq * (vq + out.iq * p.r);
  out.pe = vd * out.id + vq * out.iq;
  out.qe = vq * out.id - vd * out.iq;
  out.current = psd_dq_to_ri(delta, out.id, out.iq);
  out.vf = vf;
  return out;
}

Eigen::VectorXd genrou_initial_residual(const GenrouParams& p,
                                        Complex v,
                                        double p0,
                                        double q0,
                                        const Eigen::VectorXd& z) {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(7);
  const double delta = z[0];
  const double tau_m = z[1];
  const double vf = z[2];
  const double eq_p = z[3];
  const double ed_p = z[4];
  const double psi_kd = z[5];
  const double psi_kq = z[6];
  const GenrouEval e = evaluate_genrou(p, v, delta, eq_p, ed_p, psi_kd, psi_kq, vf);
  r[0] = tau_m - e.tau_e;
  r[1] = p0 - e.pe;
  r[2] = q0 - e.qe;
  r[3] = vf - e.xad_ifd;
  r[4] = -e.xaq_i1q;
  r[5] = -psi_kd + eq_p - (p.xdp - p.xl) * e.id;
  r[6] = -psi_kq + ed_p + (p.xqp - p.xl) * e.iq;
  return r;
}

Eigen::VectorXd solve_genrou_initial_conditions(const GenrouParams& p,
                                                Complex v,
                                                double p0,
                                                double q0,
                                                Eigen::VectorXd z) {
  Eigen::VectorXd best = z;
  double best_norm =
      genrou_initial_residual(p, v, p0, q0, z).lpNorm<Eigen::Infinity>();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int iter = 0; iter < 30; ++iter) {
    const Eigen::VectorXd r = genrou_initial_residual(p, v, p0, q0, z);
    const double norm = r.lpNorm<Eigen::Infinity>();
    if (norm < best_norm) {
      best = z;
      best_norm = norm;
    }
    if (norm <= 1e-10) return z;
    Eigen::MatrixXd jac(7, 7);
    for (int col = 0; col < 7; ++col) {
      Eigen::VectorXd zp = z;
      const double h = eps * std::max(1.0, std::abs(z[col]));
      zp[col] += h;
      jac.col(col) = (genrou_initial_residual(p, v, p0, q0, zp) - r) / h;
    }
    Eigen::VectorXd step;
    if (!sparse_newton_step(jac, -r, step)) break;
    bool accepted = false;
    double alpha = 1.0;
    while (alpha >= 1.0 / 1024.0) {
      const Eigen::VectorXd trial = z + alpha * step;
      const double trial_norm =
          genrou_initial_residual(p, v, p0, q0, trial).lpNorm<Eigen::Infinity>();
      if (std::isfinite(trial_norm) && trial_norm < norm) {
        z = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return best;
}

struct SalientParams {
  double r{0.0};
  double td0p{5.0};
  double td0pp{0.05};
  double tq0pp{0.2};
  double xd{1.0};
  double xq{0.75};
  double xdp{0.4};
  double xdpp{0.25};
  double xl{0.1};
  double sat_a{0.0};
  double sat_b{0.0};
  bool exponential_saturation{false};
};

SalientParams salient_params(const VoltageSourceDynamicParams& params) {
  SalientParams p;
  p.r = std::max(0.0, params.r_pu);
  p.xd = positive_or(params.xd_pu, 1.0);
  p.xq = positive_or(params.xq_pu, 0.75);
  p.xdp = positive_or(params.xdp_pu, 0.4);
  p.xdpp = positive_or(params.xdpp_pu, positive_or(params.x_pu, 0.25));
  p.xl = positive_or(params.xl_pu, 0.1);
  p.td0p = std::max(kMinTimeConstant, positive_or(params.td0p_s, 5.0));
  p.td0pp = std::max(kMinTimeConstant, positive_or(params.td0pp_s, 0.05));
  p.tq0pp = std::max(kMinTimeConstant, positive_or(params.tq0pp_s, 0.2));
  p.sat_a = params.saturation_a;
  p.sat_b = params.saturation_b;
  p.exponential_saturation =
      params.machine_model == SynchronousMachineModelKind::GENSAE;
  return p;
}

double salient_gamma_d1(const SalientParams& p) {
  return (p.xdpp - p.xl) / std::max(1e-9, p.xdp - p.xl);
}

double salient_gamma_q1(const SalientParams& p) {
  return (p.xdp - p.xdpp) / std::max(1e-9, p.xdp - p.xl);
}

double salient_gamma_d2(const SalientParams& p) {
  const double denom = std::max(1e-9, p.xdp - p.xl);
  return (p.xdp - p.xdpp) / (denom * denom);
}

double salient_gamma_qd(const SalientParams& p) {
  return (p.xq - p.xl) / std::max(1e-9, p.xd - p.xl);
}

double salient_saturation(const SalientParams& p, double x) {
  if (p.sat_a == 0.0 && p.sat_b == 0.0) return 0.0;
  const double v = std::max(kMinVoltage, x);
  if (p.exponential_saturation) {
    return p.sat_b * std::pow(v, p.sat_a);
  }
  return p.sat_b * (v - p.sat_a) * (v - p.sat_a) / v;
}

struct SalientEval {
  double id{0.0};
  double iq{0.0};
  double pe{0.0};
  double qe{0.0};
  double tau_e{0.0};
  double xad_ifd{0.0};
  double psi_d_pp{0.0};
  double psi_q_source{0.0};
  double vd{0.0};
  double vq{0.0};
  Complex current{0.0, 0.0};
};

SalientEval evaluate_salient(const SalientParams& p,
                             Complex v,
                             double delta,
                             double eq_p,
                             double psi_kd,
                             double psiq_pp) {
  SalientEval out;
  const auto [vd, vq] = psd_ri_to_dq(delta, v);
  out.vd = vd;
  out.vq = vq;
  const double gd1 = salient_gamma_d1(p);
  const double gq1 = salient_gamma_q1(p);
  const double gd2 = salient_gamma_d2(p);
  out.psi_d_pp = gd1 * eq_p + gq1 * psi_kd;
  const double denom = std::max(1e-9, p.r * p.r + p.xdpp * p.xdpp);
  if (p.exponential_saturation) {
    out.id = (-p.r * (vd - psiq_pp) + p.xdpp * (-vq + out.psi_d_pp)) / denom;
    out.iq = (p.xdpp * (vd - psiq_pp) + p.r * (-vq + out.psi_d_pp)) / denom;
    const double psi_pp = std::hypot(out.psi_d_pp, psiq_pp);
    const double se = salient_saturation(p, psi_pp);
    out.xad_ifd =
        eq_p + se * out.psi_d_pp +
        (p.xd - p.xdp) * (out.id + gd2 * (eq_p - psi_kd - (p.xdp - p.xl) * out.id));
    out.psi_q_source = psiq_pp;
  } else {
    out.id = (-p.r * (vd + psiq_pp) + p.xdpp * (out.psi_d_pp - vq)) / denom;
    out.iq = (p.xdpp * (vd + psiq_pp) + p.r * (out.psi_d_pp - vq)) / denom;
    const double se = salient_saturation(p, eq_p);
    out.xad_ifd =
        eq_p + se * eq_p +
        (p.xd - p.xdp) * (out.id + gd2 * (eq_p - psi_kd - (p.xdp - p.xl) * out.id));
    out.psi_q_source = -psiq_pp;
  }
  out.tau_e = out.id * (vd + out.id * p.r) + out.iq * (vq + out.iq * p.r);
  out.pe = vd * out.id + vq * out.iq;
  out.qe = vq * out.id - vd * out.iq;
  out.current = psd_dq_to_ri(delta, out.id, out.iq);
  return out;
}

Eigen::VectorXd salient_initial_residual(const SalientParams& p,
                                         Complex v,
                                         double p0,
                                         double q0,
                                         const Eigen::VectorXd& z) {
  Eigen::VectorXd r = Eigen::VectorXd::Zero(6);
  const double delta = z[0];
  const double tau_m = z[1];
  const double vf = z[2];
  const double eq_p = z[3];
  const double psi_kd = z[4];
  const double psiq_pp = z[5];
  const SalientEval e = evaluate_salient(p, v, delta, eq_p, psi_kd, psiq_pp);
  r[0] = tau_m - e.tau_e;
  r[1] = p0 - e.pe;
  r[2] = q0 - e.qe;
  r[3] = (vf - e.xad_ifd) / p.td0p;
  r[4] = (-psi_kd + eq_p - (p.xdp - p.xl) * e.id) / p.td0pp;
  if (p.exponential_saturation) {
    const double psi_pp = std::hypot(e.psi_d_pp, psiq_pp);
    const double se = salient_saturation(p, psi_pp);
    r[5] = (-psiq_pp + (p.xq - p.xdpp) * e.iq -
            se * salient_gamma_qd(p) * psiq_pp) / p.tq0pp;
  } else {
    r[5] = (-psiq_pp - (p.xq - p.xdpp) * e.iq) / p.tq0pp;
  }
  return r;
}

Eigen::VectorXd solve_salient_initial_conditions(const SalientParams& p,
                                                 Complex v,
                                                 double p0,
                                                 double q0,
                                                 Eigen::VectorXd z) {
  Eigen::VectorXd best = z;
  double best_norm =
      salient_initial_residual(p, v, p0, q0, z).lpNorm<Eigen::Infinity>();
  const double eps = std::sqrt(std::numeric_limits<double>::epsilon());
  for (int iter = 0; iter < 30; ++iter) {
    const Eigen::VectorXd r = salient_initial_residual(p, v, p0, q0, z);
    const double norm = r.lpNorm<Eigen::Infinity>();
    if (norm < best_norm) {
      best = z;
      best_norm = norm;
    }
    if (norm <= 1e-10) return z;
    Eigen::MatrixXd jac(6, 6);
    for (int col = 0; col < 6; ++col) {
      Eigen::VectorXd zp = z;
      const double h = eps * std::max(1.0, std::abs(z[col]));
      zp[col] += h;
      jac.col(col) = (salient_initial_residual(p, v, p0, q0, zp) - r) / h;
    }
    Eigen::VectorXd step;
    if (!sparse_newton_step(jac, -r, step)) break;
    bool accepted = false;
    double alpha = 1.0;
    while (alpha >= 1.0 / 1024.0) {
      const Eigen::VectorXd trial = z + alpha * step;
      const double trial_norm =
          salient_initial_residual(p, v, p0, q0, trial).lpNorm<Eigen::Infinity>();
      if (std::isfinite(trial_norm) && trial_norm < norm) {
        z = trial;
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) break;
  }
  return best;
}

Eigen::VectorXd salient_initial_guess(const SalientParams& p,
                                      Complex v,
                                      Complex s) {
  const Complex i = std::conj(s / v);
  double delta = std::arg(v + Complex(p.r, p.xq) * i);
  double tau_m0 = s.real();
  double vf0 = 1.0;
  double eq_p0 = std::abs(v);
  double psi_kd0 = eq_p0;
  double psiq_pp0 = 0.0;
  if (p.exponential_saturation) {
    const Complex psi_pp0 = v + Complex(p.r, p.xdpp) * i;
    const double psi_abs = std::max(kMinVoltage, std::abs(psi_pp0));
    const double psi_ang = std::arg(psi_pp0);
    const double se0 = salient_saturation(p, psi_abs);
    const double a = psi_abs * (se0 * salient_gamma_qd(p) + 1.0);
    const double b = (p.xdpp - p.xq) * std::abs(i);
    const double theta_it = psi_ang - std::arg(i);
    const double delta_denom = b * std::sin(theta_it) - a;
    delta = psi_ang +
            std::atan((b * std::cos(theta_it)) /
                      (std::abs(delta_denom) > 1e-12 ? delta_denom : 1e-12));
  }
  if (!std::isfinite(delta)) {
    delta = std::arg(v + Complex(p.r, p.xdpp) * i);
  }
  const auto [id0, iq0] = psd_ri_to_dq(delta, i);
  const auto [vd0, vq0] = psd_ri_to_dq(delta, v);
  const double gd1 = salient_gamma_d1(p);
  const double gq1 = salient_gamma_q1(p);
  const double gd2 = salient_gamma_d2(p);
  if (p.exponential_saturation) {
    const double psiq_src0 = vd0 - p.r * id0 - p.xdpp * iq0;
    const double psidpp0 = vq0 + id0 * p.xdpp + iq0 * p.r;
    const double psi_d0 = vq0 + p.r * iq0;
    const double psi_q0 = -vd0 - p.r * id0;
    eq_p0 = (psidpp0 + id0 * (p.xdp - p.xl) * gq1) /
            std::max(1e-9, gd1 + gq1);
    psi_kd0 = eq_p0 - id0 * (p.xdp - p.xl);
    tau_m0 = psi_d0 * iq0 - psi_q0 * id0;
    const double se0 = salient_saturation(p, std::hypot(psidpp0, psiq_src0));
    vf0 = eq_p0 + id0 * (p.xd - p.xdp) + psidpp0 * se0;
    psiq_pp0 = psiq_src0;
  } else {
    const double psi_d0 = vq0 + p.r * iq0;
    const double psi_q0 = -vd0 - p.r * id0;
    const double psidpp0 = vq0 + id0 * p.xdpp + iq0 * p.r;
    psiq_pp0 = -(vd0 - iq0 * p.xdpp - id0 * p.r);
    eq_p0 = (psidpp0 + id0 * (p.xdp - p.xl) * gq1) /
            std::max(1e-9, gd1 + gq1);
    psi_kd0 = eq_p0 - id0 * (p.xdp - p.xl);
    tau_m0 = psi_d0 * iq0 - psi_q0 * id0;
    const double se0 = salient_saturation(p, eq_p0);
    vf0 = eq_p0 + id0 * (p.xd - p.xdp) + se0 * eq_p0;
    (void)gd2;
  }
  Eigen::VectorXd z(6);
  z << delta, tau_m0, vf0, eq_p0, psi_kd0, psiq_pp0;
  return z;
}

}  // namespace

DynamicLoad::DynamicLoad(ACLoadDynamicParams params) : params_(std::move(params)) {}

void DynamicLoad::assignStateIndices(int& offset) { (void)offset; }

void DynamicLoad::initializeFromPowerFlow(const PowerFlowResult&, DynamicState&, NetworkState&) {}

void DynamicLoad::computeDerivatives(double,
                                     const DynamicState&,
                                     const NetworkState&,
                                     Eigen::Ref<Eigen::VectorXd>) const {}

void DynamicLoad::stamp(double,
                        const DynamicState&,
                        const NetworkState& y,
                        DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || params_.base_mva <= 0.0) return;
  const Complex s_pu(params_.p_mw / params_.base_mva * params_.scale,
                     params_.q_mvar / params_.base_mva * params_.scale);
  const double phase_power_scale =
      std::max(0.0, std::abs(params_.phase_power_scale));
  if (params_.model_kind == DynamicLoadModelKind::ConstantImpedance) {
    const double v0 = std::max(1e-6, std::abs(params_.nominal_voltage_pu));
    const Complex y_load = std::conj(s_pu) * phase_power_scale / (v0 * v0);
    add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(y_load));
    return;
  }

  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const double p_phase = s_pu.real() * phase_power_scale;
  const double q_phase = s_pu.imag() * phase_power_scale;
  const double v0 = params_.nominal_voltage_pu > 0.0 ? params_.nominal_voltage_pu : 1.0;
  for (int phase = 0; phase < 3; ++phase) {
    const Complex consuming = load_consuming_current(v[phase],
                                                     p_phase,
                                                     q_phase,
                                                     v0,
                                                     params_.model_kind,
                                                     params_);
    stamp.addAcCurrent(3 * params_.bus_pos + phase, -consuming);
  }
}

void DynamicLoad::addJacobian(double,
                              const DynamicState&,
                              const NetworkState& y,
                              const DynamicJacobianContext& context,
                              std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_pos < 0 || params_.base_mva <= 0.0 ||
      params_.model_kind == DynamicLoadModelKind::ConstantImpedance) {
    return;
  }
  const Complex s_pu(params_.p_mw / params_.base_mva * params_.scale,
                     params_.q_mvar / params_.base_mva * params_.scale);
  const double phase_power_scale =
      std::max(0.0, std::abs(params_.phase_power_scale));
  const double p_phase = s_pu.real() * phase_power_scale;
  const double q_phase = s_pu.imag() * phase_power_scale;
  const double v0 = params_.nominal_voltage_pu > 0.0 ? params_.nominal_voltage_pu : 1.0;
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  for (int phase = 0; phase < 3; ++phase) {
    const int node = 3 * params_.bus_pos + phase;
    add_local_ac_voltage_current_derivative(
        context,
        node,
        v[phase],
        [&](Complex vv) {
          return -load_consuming_current(vv,
                                         p_phase,
                                         q_phase,
                                         v0,
                                         params_.model_kind,
                                         params_);
        },
        triplets);
  }
}

void DynamicLoad::handleEvent(const DynamicEvent& event, DynamicState&, NetworkState&) {
  if (event.type != DynamicEventType::ACLoadScale) return;
  const bool bus_targeted = event.bus != 0;
  if (bus_targeted && event.bus != params_.bus) return;
  if (!bus_targeted && event.component_index != 0 &&
      event.component_index != params_.component_index) {
    return;
  }
  const auto it = event.params.find("scale");
  params_.scale = std::max(0.0, it == event.params.end() ? event.value : it->second);
}

std::string DynamicLoad::name() const {
  return params_.label.empty() ? "AC load " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> DynamicLoad::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput DynamicLoad::output(const DynamicState&,
                                        const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  const Complex s = params_.in_service ? load_power_pu(params_) : Complex(0.0, 0.0);
  out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
  out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
  out.values["scale"] = params_.scale;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["constant_power"] =
      params_.model_kind == DynamicLoadModelKind::ConstantPower ? 1.0 : 0.0;
  out.values["constant_current"] =
      params_.model_kind == DynamicLoadModelKind::ConstantCurrent ? 1.0 : 0.0;
  out.values["constant_impedance"] =
      params_.model_kind == DynamicLoadModelKind::ConstantImpedance ? 1.0 : 0.0;
  add_voltage_metrics(out, y, params_.bus_pos);
  return out;
}

DynamicRLLine::DynamicRLLine(DynamicRLLineParams params) : params_(std::move(params)) {}

void DynamicRLLine::assignStateIndices(int& offset) {
  range_ = {offset, 2};
  offset += range_.size;
}

Complex DynamicRLLine::voltageDrop(const NetworkState& y) const {
  const Complex vf = positive_sequence_voltage(bus_voltage(y, params_.from_pos));
  const Complex vt = positive_sequence_voltage(bus_voltage(y, params_.to_pos));
  return vf - vt;
}

Complex DynamicRLLine::equilibriumCurrent(const NetworkState& y) const {
  const Complex z(params_.r_pu, params_.x_pu);
  if (std::abs(z) <= 1e-12) return Complex(0.0, 0.0);
  return voltageDrop(y) / z;
}

Complex DynamicRLLine::branchCurrent(const DynamicState& x) const {
  if (range_.empty() || state_index(range_, 1) >= x.x.size()) return Complex(0.0, 0.0);
  return Complex(x.x[state_index(range_, 0)], x.x[state_index(range_, 1)]);
}

void DynamicRLLine::initializeFromPowerFlow(const PowerFlowResult&,
                                            DynamicState& x,
                                            NetworkState& y) {
  if (range_.empty()) return;
  const Complex i0 = params_.in_service ? equilibriumCurrent(y) : Complex(0.0, 0.0);
  x.x[state_index(range_, 0)] = i0.real();
  x.x[state_index(range_, 1)] = i0.imag();
}

bool DynamicRLLine::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (range_.empty()) return false;
  const Complex i0 = params_.in_service ? equilibriumCurrent(y) : Complex(0.0, 0.0);
  bool changed = false;
  changed = set_if_changed(x.x, state_index(range_, 0), i0.real()) || changed;
  changed = set_if_changed(x.x, state_index(range_, 1), i0.imag()) || changed;
  return changed;
}

void DynamicRLLine::computeDerivatives(double,
                                       const DynamicState& x,
                                       const NetworkState& y,
                                       Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (range_.empty()) return;
  if (!params_.in_service ||
      params_.from_pos < 0 ||
      params_.to_pos < 0 ||
      std::abs(params_.x_pu) <= 1e-12) {
    dxdt[state_index(range_, 0)] = 0.0;
    dxdt[state_index(range_, 1)] = 0.0;
    return;
  }
  const Complex dv = voltageDrop(y);
  const Complex i = branchCurrent(x);
  const double omega_b = kTwoPi * positive_or(params_.frequency_hz, 50.0);
  const double l = std::max(1e-12, std::abs(params_.x_pu));
  dxdt[state_index(range_, 0)] =
      (omega_b / l) * (dv.real() - (params_.r_pu * i.real() - params_.x_pu * i.imag()));
  dxdt[state_index(range_, 1)] =
      (omega_b / l) * (dv.imag() - (params_.r_pu * i.imag() + params_.x_pu * i.real()));
}

void DynamicRLLine::stamp(double,
                          const DynamicState& x,
                          const NetworkState&,
                          DynamicStamp& stamp) const {
  if (!params_.in_service || params_.from_pos < 0 || params_.to_pos < 0) return;
  const Eigen::Vector3cd current =
      balanced_current_from_positive_sequence(branchCurrent(x));
  add_balanced_current(stamp, params_.from_pos, -current);
  add_balanced_current(stamp, params_.to_pos, current);
}

void DynamicRLLine::addJacobian(double,
                                const DynamicState&,
                                const NetworkState&,
                                const DynamicJacobianContext& context,
                                std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.from_pos < 0 || params_.to_pos < 0 ||
      range_.empty()) {
    return;
  }
  const int ir_col = state_index(range_, 0);
  const int ii_col = state_index(range_, 1);
  if (!context.validStateIndex(ir_col) || !context.validStateIndex(ii_col)) return;
  const Eigen::Vector3cd d_ir = balanced_current_from_positive_sequence(Complex(1.0, 0.0));
  const Eigen::Vector3cd d_ii = balanced_current_from_positive_sequence(Complex(0.0, 1.0));
  add_balanced_current_derivative(context, params_.from_pos, ir_col, -d_ir, triplets);
  add_balanced_current_derivative(context, params_.from_pos, ii_col, -d_ii, triplets);
  add_balanced_current_derivative(context, params_.to_pos, ir_col, d_ir, triplets);
  add_balanced_current_derivative(context, params_.to_pos, ii_col, d_ii, triplets);
}

void DynamicRLLine::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState& y) {
  if (event.type != DynamicEventType::ACBranchTrip &&
      event.type != DynamicEventType::ACBranchClose &&
      event.type != DynamicEventType::ACBranchImpedanceScale) {
    return;
  }
  if (event.component_index != 0 && event.component_index != params_.component_index) return;
  if (event.type == DynamicEventType::ACBranchImpedanceScale) {
    const auto scale_it = event.params.find("scale");
    const double scale = scale_it != event.params.end() ? scale_it->second
                         : (event.value > 0.0 ? event.value : 1.0);
    const auto r_it = event.params.find("r_scale");
    const auto x_it = event.params.find("x_scale");
    params_.r_pu *= r_it != event.params.end() ? r_it->second : scale;
    params_.x_pu *= x_it != event.params.end() ? x_it->second : scale;
  } else {
    params_.in_service = event.type == DynamicEventType::ACBranchClose;
  }
  if (!range_.empty()) {
    const Complex i = params_.in_service ? equilibriumCurrent(y) : Complex(0.0, 0.0);
    x.x[state_index(range_, 0)] = i.real();
    x.x[state_index(range_, 1)] = i.imag();
  }
}

std::string DynamicRLLine::name() const {
  if (!params_.label.empty()) return params_.label;
  return "Dynamic RL line " + std::to_string(params_.component_index);
}

DynamicDeviceOutput DynamicRLLine::output(const DynamicState& x,
                                          const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.from_bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  const Complex i = branchCurrent(x);
  const Complex vf = positive_sequence_voltage(bus_voltage(y, params_.from_pos));
  const Complex vt = positive_sequence_voltage(bus_voltage(y, params_.to_pos));
  const Complex s_from = vf * std::conj(i);
  out.values["from_bus"] = static_cast<double>(params_.from_bus);
  out.values["to_bus"] = static_cast<double>(params_.to_bus);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["r_pu"] = params_.r_pu;
  out.values["x_pu"] = params_.x_pu;
  out.values["current_r_pu"] = i.real();
  out.values["current_i_pu"] = i.imag();
  out.values["current_mag_pu"] = std::abs(i);
  out.values["voltage_drop_r_pu"] = (vf - vt).real();
  out.values["voltage_drop_i_pu"] = (vf - vt).imag();
  out.values["p_from_mw"] = s_from.real() * safe_base(params_.base_mva);
  out.values["q_from_mvar"] = s_from.imag() * safe_base(params_.base_mva);
  add_voltage_metrics(out, y, params_.from_pos);
  return out;
}

ThreePhaseDynamicLoad::ThreePhaseDynamicLoad(ThreePhaseLoadDynamicParams params)
    : params_(std::move(params)) {}

void ThreePhaseDynamicLoad::assignStateIndices(int& offset) { (void)offset; }

void ThreePhaseDynamicLoad::initializeFromPowerFlow(const PowerFlowResult&,
                                                   DynamicState&,
                                                   NetworkState&) {}

void ThreePhaseDynamicLoad::computeDerivatives(double,
                                               const DynamicState&,
                                               const NetworkState&,
                                               Eigen::Ref<Eigen::VectorXd>) const {}

void ThreePhaseDynamicLoad::stamp(double,
                                  const DynamicState&,
                                  const NetworkState&,
                                  DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || params_.base_mva <= 0.0) return;
  for (int phase = 0; phase < 3; ++phase) {
    if (!params_.phase_active[static_cast<std::size_t>(phase)]) continue;
    const Complex s_pu(params_.p_mw[static_cast<std::size_t>(phase)] / params_.base_mva *
                           params_.scale,
                       params_.q_mvar[static_cast<std::size_t>(phase)] / params_.base_mva *
                           params_.scale);
    stamp.addAcAdmittance(3 * params_.bus_pos + phase,
                          3 * params_.bus_pos + phase,
                          std::conj(s_pu));
  }
}

void ThreePhaseDynamicLoad::addJacobian(double,
                                        const DynamicState&,
                                        const NetworkState&,
                                        const DynamicJacobianContext&,
                                        std::vector<Eigen::Triplet<double>>&) const {}

void ThreePhaseDynamicLoad::handleEvent(const DynamicEvent& event,
                                        DynamicState&,
                                        NetworkState&) {
  if (event.type != DynamicEventType::ACLoadScale) return;
  const bool bus_targeted = event.bus != 0;
  if (bus_targeted && event.bus != params_.bus) return;
  if (!bus_targeted && event.component_index != 0 &&
      event.component_index != params_.component_index) {
    return;
  }
  const auto it = event.params.find("scale");
  params_.scale = std::max(0.0, it == event.params.end() ? event.value : it->second);
}

std::string ThreePhaseDynamicLoad::name() const {
  return params_.label.empty()
             ? "Three-phase load " + std::to_string(params_.component_index)
             : params_.label;
}

std::vector<DynamicModelProfile> ThreePhaseDynamicLoad::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput ThreePhaseDynamicLoad::output(const DynamicState&,
                                                  const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  double p = 0.0;
  double q = 0.0;
  for (int phase = 0; phase < 3; ++phase) {
    if (!params_.phase_active[static_cast<std::size_t>(phase)]) continue;
    const double pp = params_.in_service
                          ? params_.p_mw[static_cast<std::size_t>(phase)] * params_.scale
                          : 0.0;
    const double qq = params_.in_service
                          ? params_.q_mvar[static_cast<std::size_t>(phase)] * params_.scale
                          : 0.0;
    p += pp;
    q += qq;
    const char key = static_cast<char>('a' + phase);
    out.values[std::string("p_") + key + "_mw"] = pp;
    out.values[std::string("q_") + key + "_mvar"] = qq;
  }
  out.values["p_mw"] = p;
  out.values["q_mvar"] = q;
  out.values["scale"] = params_.scale;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  add_voltage_metrics(out, y, params_.bus_pos);
  return out;
}

DCDynamicLoad::DCDynamicLoad(DCLoadDynamicParams params) : params_(std::move(params)) {}

void DCDynamicLoad::assignStateIndices(int& offset) { (void)offset; }

void DCDynamicLoad::initializeFromPowerFlow(const PowerFlowResult&, DynamicState&, NetworkState&) {}

void DCDynamicLoad::computeDerivatives(double,
                                       const DynamicState&,
                                       const NetworkState&,
                                       Eigen::Ref<Eigen::VectorXd>) const {}

void DCDynamicLoad::stamp(double,
                          const DynamicState&,
                          const NetworkState&,
                          DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || params_.base_mva <= 0.0) return;
  const double p_pu = params_.p_mw / params_.base_mva * params_.scale;
  stamp.addDcConductance(params_.bus_pos, params_.bus_pos, std::max(0.0, p_pu));
}

void DCDynamicLoad::addJacobian(double,
                                const DynamicState&,
                                const NetworkState&,
                                const DynamicJacobianContext&,
                                std::vector<Eigen::Triplet<double>>&) const {}

void DCDynamicLoad::handleEvent(const DynamicEvent& event, DynamicState&, NetworkState&) {
  if (event.type != DynamicEventType::DCLoadScale) return;
  const bool bus_targeted = event.bus != 0;
  if (bus_targeted && event.bus != params_.bus) return;
  if (!bus_targeted && event.component_index != 0 &&
      event.component_index != params_.component_index) {
    return;
  }
  const auto it = event.params.find("scale");
  params_.scale = std::max(0.0, it == event.params.end() ? event.value : it->second);
}

std::string DCDynamicLoad::name() const {
  return params_.label.empty() ? "DC load " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> DCDynamicLoad::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput DCDynamicLoad::output(const DynamicState&,
                                          const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["p_mw"] = params_.in_service ? params_.p_mw * params_.scale : 0.0;
  out.values["scale"] = params_.scale;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  add_voltage_metrics(out, y, -1, params_.bus_pos);
  return out;
}

DCVoltageSourceDynamic::DCVoltageSourceDynamic(DCVoltageSourceDynamicParams params)
    : params_(std::move(params)) {}

void DCVoltageSourceDynamic::assignStateIndices(int& offset) { (void)offset; }

void DCVoltageSourceDynamic::initializeFromPowerFlow(const PowerFlowResult&,
                                                    DynamicState&,
                                                    NetworkState&) {}

void DCVoltageSourceDynamic::computeDerivatives(double,
                                                const DynamicState&,
                                                const NetworkState&,
                                                Eigen::Ref<Eigen::VectorXd>) const {}

void DCVoltageSourceDynamic::stamp(double,
                                   const DynamicState&,
                                   const NetworkState&,
                                   DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0) return;
  const double g = std::max(0.0, params_.conductance_pu);
  stamp.addDcConductance(params_.bus_pos, params_.bus_pos, g);
  stamp.addDcCurrent(params_.bus_pos, g * params_.v_ref_pu);
}

void DCVoltageSourceDynamic::addJacobian(double,
                                         const DynamicState&,
                                         const NetworkState&,
                                         const DynamicJacobianContext&,
                                         std::vector<Eigen::Triplet<double>>&) const {}

void DCVoltageSourceDynamic::handleEvent(const DynamicEvent& event, DynamicState&, NetworkState&) {
  if (params_.trip_on_vsc_event &&
      event.type == DynamicEventType::VSCTrip &&
      (event.component_index == 0 || event.component_index == params_.component_index)) {
    params_.in_service = false;
  }
}

std::string DCVoltageSourceDynamic::name() const {
  return params_.label.empty() ? "DC voltage source " + std::to_string(params_.component_index)
                               : params_.label;
}

DynamicDeviceOutput DCVoltageSourceDynamic::output(const DynamicState&,
                                                   const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["v_ref_pu"] = params_.v_ref_pu;
  out.values["conductance_pu"] = params_.conductance_pu;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  add_voltage_metrics(out, y, -1, params_.bus_pos);
  return out;
}

SynchronousMachine::SynchronousMachine(VoltageSourceDynamicParams params)
    : params_(std::move(params)) {}

void SynchronousMachine::assignStateIndices(int& offset) {
  range_ = {offset, synchronous_machine_state_count(params_)};
  offset += range_.size;
  const bool has_turbine_governor = governor_attached_ ||
                                    link_.inner_vars.has_turbine_governor;
  const bool has_avr = exciter_attached_ || link_.inner_vars.has_avr;
  const bool has_pss = link_.inner_vars.has_pss;
  // Publish the coupling link so attached controllers can address this machine's
  // speed / mechanical-power / field states. Local indices: omega=1 for both
  // models; classical pm=3, e_mag(field)=2; OneDOneQ tau_m=4, vf=5;
  // SimpleMarconato / round-rotor tau_m=6, vf=7; Marconato tau_m=8, vf=9;
  // salient-pole tau_m=5, vf=6.
  link_.range = &range_;
  link_.valid = true;
  link_.genrou = machine_is_roundrotor(params_) && range_.size >= 8;
  link_.bus_pos = params_.bus_pos;
  link_.base_mva = params_.base_mva;
  link_.frequency_hz = params_.frequency_hz;
  link_.inertia_h = std::max(0.01, params_.inertia_h);
  link_.omega_local = 1;
  if ((machine_is_marconato(params_) || machine_is_sauerpai(params_)) &&
      range_.size >= 10) {
    link_.pm_local = 8;
    link_.efd_local = 9;
  } else if (link_.genrou) {
    link_.pm_local = 6;
    link_.efd_local = 7;
  } else if (machine_is_simple_marconato(params_) && range_.size >= 8) {
    link_.pm_local = 6;
    link_.efd_local = 7;
  } else if (machine_is_salient(params_) && range_.size >= 7) {
    link_.pm_local = 5;
    link_.efd_local = 6;
  } else if (machine_is_onedoneq(params_) && range_.size >= 6) {
    link_.pm_local = 4;
    link_.efd_local = 5;
  } else {
    link_.pm_local = 3;
    link_.efd_local = 2;
  }
  link_.inner_vars = GeneratorInnerVariableBus{};
  link_.inner_vars.machine_range = &range_;
  link_.inner_vars.valid = true;
  link_.inner_vars.bus_pos = params_.bus_pos;
  link_.inner_vars.base_mva = params_.base_mva;
  link_.inner_vars.frequency_hz = params_.frequency_hz;
  link_.inner_vars.inertia_h = std::max(0.01, params_.inertia_h);
  link_.inner_vars.has_machine = true;
  link_.inner_vars.has_shaft = true;
  link_.inner_vars.has_turbine_governor = has_turbine_governor;
  link_.inner_vars.has_avr = has_avr;
  link_.inner_vars.has_pss = has_pss;
  link_.inner_vars.omega_local = link_.omega_local;
  link_.inner_vars.state_local[static_cast<std::size_t>(
      static_cast<int>(GeneratorInnerVar::MechanicalTorque))] = link_.pm_local;
  link_.inner_vars.state_local[static_cast<std::size_t>(
      static_cast<int>(GeneratorInnerVar::FieldVoltage))] = link_.efd_local;
}

void SynchronousMachine::initializeFromPowerFlow(const PowerFlowResult& pf,
                                                DynamicState& x,
                                                NetworkState& y) {
  if (range_.empty()) return;
  double angle = params_.angle_set_rad;
  if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.va.size())) {
    angle = pf.va[static_cast<std::size_t>(params_.bus_pos)];
  }
  if (machine_is_onedoneq(params_) && range_.size >= 6) {
    const OneDOneQParams mp = onedoneq_params(params_);
    double vm = params_.vm_set_pu;
    if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
      vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
    }
    const Complex v = std::polar(vm, angle);
    const Complex s(params_.p_mech_mw / safe_base(params_.base_mva),
                    params_.q_elec_mvar / safe_base(params_.base_mva));
    const Complex i_from_power = std::conj(s / v);
    double delta = std::arg(v + Complex(mp.r, mp.xq) * i_from_power);
    if (!std::isfinite(delta)) {
      delta = angle;
    }
    const auto [vd0, vq0] = psd_ri_to_dq(delta, v);
    Eigen::VectorXd z0(5);
    z0 << delta, s.real(), 1.0, vq0, vd0;
    const Eigen::VectorXd z =
        solve_onedoneq_initial_conditions(mp, v, s.real(), s.imag(), z0);

    x.x[state_index(range_, 0)] = finite_value(z[0], delta);
    x.x[state_index(range_, 1)] = 1.0;
    x.x[state_index(range_, 2)] = finite_value(z[3], vq0);
    x.x[state_index(range_, 3)] = finite_value(z[4], vd0);
    x.x[state_index(range_, 4)] = finite_value(z[1], s.real());
    x.x[state_index(range_, 5)] = finite_value(z[2], 1.0);
    if (params_.bus_pos >= 0 && y.Vac_abc.size() >= 3 * (params_.bus_pos + 1)) {
      y.Vac_abc[3 * params_.bus_pos + 0] = std::polar(vm, angle);
      y.Vac_abc[3 * params_.bus_pos + 1] = std::polar(vm, angle - 2.0 * kPi / 3.0);
      y.Vac_abc[3 * params_.bus_pos + 2] = std::polar(vm, angle + 2.0 * kPi / 3.0);
    }
    return;
  }
  if (machine_is_sauerpai(params_) && range_.size >= 10) {
    const SauerPaiParams mp = sauerpai_params(params_);
    double vm = params_.vm_set_pu;
    if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
      vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
    }
    const Complex v = std::polar(vm, angle);
    const Complex s(params_.p_mech_mw / safe_base(params_.base_mva),
                    params_.q_elec_mvar / safe_base(params_.base_mva));
    const Complex i_from_power = std::conj(s / v);
    double delta = std::arg(v + Complex(mp.r, mp.xq) * i_from_power);
    if (!std::isfinite(delta)) {
      delta = angle;
    }
    const auto [vd0, vq0] = psd_ri_to_dq(delta, v);
    Eigen::VectorXd z0(9);
    z0 << delta, s.real(), 1.0, vd0, vq0, vq0, vd0, vq0, vd0;
    const Eigen::VectorXd z =
        solve_sauerpai_initial_conditions(mp, v, s.real(), s.imag(), z0);

    x.x[state_index(range_, 0)] = finite_value(z[0], delta);
    x.x[state_index(range_, 1)] = 1.0;
    x.x[state_index(range_, 2)] = finite_value(z[3], vd0);
    x.x[state_index(range_, 3)] = finite_value(z[4], vq0);
    x.x[state_index(range_, 4)] = finite_value(z[5], vq0);
    x.x[state_index(range_, 5)] = finite_value(z[6], vd0);
    x.x[state_index(range_, 6)] = finite_value(z[7], vq0);
    x.x[state_index(range_, 7)] = finite_value(z[8], vd0);
    x.x[state_index(range_, 8)] = finite_value(z[1], s.real());
    x.x[state_index(range_, 9)] = finite_value(z[2], 1.0);
    if (params_.bus_pos >= 0 && y.Vac_abc.size() >= 3 * (params_.bus_pos + 1)) {
      y.Vac_abc[3 * params_.bus_pos + 0] = std::polar(vm, angle);
      y.Vac_abc[3 * params_.bus_pos + 1] = std::polar(vm, angle - 2.0 * kPi / 3.0);
      y.Vac_abc[3 * params_.bus_pos + 2] = std::polar(vm, angle + 2.0 * kPi / 3.0);
    }
    return;
  }
  if (machine_is_marconato(params_) && range_.size >= 10) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    double vm = params_.vm_set_pu;
    if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
      vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
    }
    const Complex v = std::polar(vm, angle);
    const Complex s(params_.p_mech_mw / safe_base(params_.base_mva),
                    params_.q_elec_mvar / safe_base(params_.base_mva));
    const Complex i_from_power = std::conj(s / v);
    double delta = std::arg(v + Complex(mp.r, mp.xq) * i_from_power);
    if (!std::isfinite(delta)) {
      delta = angle;
    }
    const auto [vd0, vq0] = psd_ri_to_dq(delta, v);
    Eigen::VectorXd z0(9);
    z0 << delta, s.real(), 1.0, vd0, vq0, vq0, vd0, vq0, vd0;
    const Eigen::VectorXd z =
        solve_marconato_initial_conditions(mp, v, s.real(), s.imag(), z0);

    x.x[state_index(range_, 0)] = finite_value(z[0], delta);
    x.x[state_index(range_, 1)] = 1.0;
    x.x[state_index(range_, 2)] = finite_value(z[3], vd0);
    x.x[state_index(range_, 3)] = finite_value(z[4], vq0);
    x.x[state_index(range_, 4)] = finite_value(z[5], vq0);
    x.x[state_index(range_, 5)] = finite_value(z[6], vd0);
    x.x[state_index(range_, 6)] = finite_value(z[7], vq0);
    x.x[state_index(range_, 7)] = finite_value(z[8], vd0);
    x.x[state_index(range_, 8)] = finite_value(z[1], s.real());
    x.x[state_index(range_, 9)] = finite_value(z[2], 1.0);
    if (params_.bus_pos >= 0 && y.Vac_abc.size() >= 3 * (params_.bus_pos + 1)) {
      y.Vac_abc[3 * params_.bus_pos + 0] = std::polar(vm, angle);
      y.Vac_abc[3 * params_.bus_pos + 1] = std::polar(vm, angle - 2.0 * kPi / 3.0);
      y.Vac_abc[3 * params_.bus_pos + 2] = std::polar(vm, angle + 2.0 * kPi / 3.0);
    }
    return;
  }
  if (machine_is_simple_marconato(params_) && range_.size >= 8) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    double vm = params_.vm_set_pu;
    if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
      vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
    }
    const Complex v = std::polar(vm, angle);
    const Complex s(params_.p_mech_mw / safe_base(params_.base_mva),
                    params_.q_elec_mvar / safe_base(params_.base_mva));
    const Complex i_from_power = std::conj(s / v);
    double delta = std::arg(v + Complex(mp.r, mp.xq) * i_from_power);
    if (!std::isfinite(delta)) {
      delta = angle;
    }
    const auto [vd0, vq0] = psd_ri_to_dq(delta, v);
    Eigen::VectorXd z0(7);
    z0 << delta, s.real(), 1.0, vq0, vd0, vq0, vd0;
    const Eigen::VectorXd z =
        solve_simple_marconato_initial_conditions(mp, v, s.real(), s.imag(), z0);

    x.x[state_index(range_, 0)] = finite_value(z[0], delta);
    x.x[state_index(range_, 1)] = 1.0;
    x.x[state_index(range_, 2)] = finite_value(z[3], vq0);
    x.x[state_index(range_, 3)] = finite_value(z[4], vd0);
    x.x[state_index(range_, 4)] = finite_value(z[5], vq0);
    x.x[state_index(range_, 5)] = finite_value(z[6], vd0);
    x.x[state_index(range_, 6)] = finite_value(z[1], s.real());
    x.x[state_index(range_, 7)] = finite_value(z[2], 1.0);
    if (params_.bus_pos >= 0 && y.Vac_abc.size() >= 3 * (params_.bus_pos + 1)) {
      y.Vac_abc[3 * params_.bus_pos + 0] = std::polar(vm, angle);
      y.Vac_abc[3 * params_.bus_pos + 1] = std::polar(vm, angle - 2.0 * kPi / 3.0);
      y.Vac_abc[3 * params_.bus_pos + 2] = std::polar(vm, angle + 2.0 * kPi / 3.0);
    }
    return;
  }
  if (machine_is_roundrotor(params_) && range_.size >= 8) {
    const GenrouParams gp = genrou_params(params_);
    double vm = params_.vm_set_pu;
    if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
      vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
    }
    const Complex v = std::polar(vm, angle);
    const Complex s(params_.p_mech_mw / safe_base(params_.base_mva),
                    params_.q_elec_mvar / safe_base(params_.base_mva));
    const Complex i_from_power = std::conj(s / v);
    const Complex psi_pp0 = v + Complex(gp.r, gp.xdpp) * i_from_power;
    const double psi_abs = std::max(kMinVoltage, std::abs(psi_pp0));
    const double psi_ang = std::arg(psi_pp0);
    const double se0 = genrou_saturation(gp, psi_abs);
    const double a = psi_abs * (se0 * genrou_gamma_qd(gp) + 1.0);
    const double b = (gp.xdpp - gp.xq) * std::abs(i_from_power);
    const double theta_it = psi_ang - std::arg(i_from_power);
    const double delta_denom = b * std::sin(theta_it) - a;
    double delta = psi_ang +
                   std::atan((b * std::cos(theta_it)) /
                             (std::abs(delta_denom) > 1e-12 ? delta_denom : 1e-12));
    if (!std::isfinite(delta)) {
      delta = std::arg(v + Complex(gp.r, gp.xdpp) * i_from_power);
    }
    const auto [id0, iq0] = psd_ri_to_dq(delta, i_from_power);
    const auto [vd0, vq0] = psd_ri_to_dq(delta, v);
    const double psi_q_pp0 = vd0 - gp.r * id0 - gp.xdpp * iq0;
    const double psi_d_pp0 = vq0 + gp.xdpp * id0 + gp.r * iq0;
    const double tau_m0 = id0 * (vd0 + id0 * gp.r) + iq0 * (vq0 + iq0 * gp.r);
    const double vf0 = id0 * (gp.xd - gp.xdpp) + psi_d_pp0 * (se0 + 1.0);
    const double eq_p0 = id0 * (gp.xdp - gp.xd) - se0 * psi_d_pp0 + vf0;
    const double ed_p0 = iq0 * (gp.xq - gp.xqp) - se0 * genrou_gamma_qd(gp) * psi_q_pp0;
    const double psi_kd0 = -id0 * (gp.xd - gp.xl) - se0 * psi_d_pp0 + vf0;
    const double psi_kq0 = iq0 * (gp.xq - gp.xl) - se0 * genrou_gamma_qd(gp) * psi_q_pp0;
    Eigen::VectorXd z0(7);
    z0 << delta, tau_m0, vf0, eq_p0, ed_p0, psi_kd0, psi_kq0;
    const Eigen::VectorXd z =
        solve_genrou_initial_conditions(gp, v, s.real(), s.imag(), z0);

    x.x[state_index(range_, 0)] = finite_value(z[0], delta);
    x.x[state_index(range_, 1)] = 1.0;
    x.x[state_index(range_, 2)] = finite_value(z[3], eq_p0);
    x.x[state_index(range_, 3)] = finite_value(z[4], ed_p0);
    x.x[state_index(range_, 4)] = finite_value(z[5], psi_kd0);
    x.x[state_index(range_, 5)] = finite_value(z[6], psi_kq0);
    x.x[state_index(range_, 6)] = finite_value(z[1], tau_m0);
    x.x[state_index(range_, 7)] = finite_value(z[2], vf0);
    if (params_.bus_pos >= 0 && y.Vac_abc.size() >= 3 * (params_.bus_pos + 1)) {
      y.Vac_abc[3 * params_.bus_pos + 0] = std::polar(vm, angle);
      y.Vac_abc[3 * params_.bus_pos + 1] = std::polar(vm, angle - 2.0 * kPi / 3.0);
      y.Vac_abc[3 * params_.bus_pos + 2] = std::polar(vm, angle + 2.0 * kPi / 3.0);
    }
    return;
  }
  if (machine_is_salient(params_) && range_.size >= 7) {
    const SalientParams sp = salient_params(params_);
    double vm = params_.vm_set_pu;
    if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
      vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
    }
    const Complex v = std::polar(vm, angle);
    const Complex s(params_.p_mech_mw / safe_base(params_.base_mva),
                    params_.q_elec_mvar / safe_base(params_.base_mva));
    const Eigen::VectorXd z0 = salient_initial_guess(sp, v, s);
    const Eigen::VectorXd z =
        solve_salient_initial_conditions(sp, v, s.real(), s.imag(), z0);

    x.x[state_index(range_, 0)] = finite_value(z[0], z0[0]);
    x.x[state_index(range_, 1)] = 1.0;
    x.x[state_index(range_, 2)] = finite_value(z[3], z0[3]);
    x.x[state_index(range_, 3)] = finite_value(z[4], z0[4]);
    x.x[state_index(range_, 4)] = finite_value(z[5], z0[5]);
    x.x[state_index(range_, 5)] = finite_value(z[1], z0[1]);
    x.x[state_index(range_, 6)] = finite_value(z[2], z0[2]);
    if (params_.bus_pos >= 0 && y.Vac_abc.size() >= 3 * (params_.bus_pos + 1)) {
      y.Vac_abc[3 * params_.bus_pos + 0] = std::polar(vm, angle);
      y.Vac_abc[3 * params_.bus_pos + 1] = std::polar(vm, angle - 2.0 * kPi / 3.0);
      y.Vac_abc[3 * params_.bus_pos + 2] = std::polar(vm, angle + 2.0 * kPi / 3.0);
    }
    return;
  }
  // Classical model: place the internal EMF *behind* the machine reactance so
  // that at t=0 the machine injects its scheduled P+jQ into a terminal held at
  // the power-flow voltage. Initializing E == Vt (as before) produces ~0 current
  // and therefore ~0 electrical power, leaving a standing Pm-Pe imbalance that
  // drives an undamped rotor swing (or, with equilibrium trimming enabled, a
  // wrong low-power operating point).
  double vm = params_.vm_set_pu;
  if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.vm.size())) {
    vm = positive_or(pf.vm[static_cast<std::size_t>(params_.bus_pos)], vm);
  }
  const double p_pu = params_.p_mech_mw / safe_base(params_.base_mva);
  const double q_pu = params_.q_elec_mvar / safe_base(params_.base_mva);
  const Complex vt = std::polar(clamp_voltage(vm), angle);
  const double phase_power_scale =
      std::max(0.0, std::abs(params_.phase_power_scale));
  const Complex s_phase(p_pu * phase_power_scale, q_pu * phase_power_scale);
  const Complex i_phase = std::conj(s_phase / vt);
  const Complex z(params_.r_pu, source_series_reactance_pu(params_.x_pu));
  const Complex e_internal = vt + z * i_phase;
  x.x[range_.offset + 0] = finite_value(std::arg(e_internal), angle);
  x.x[range_.offset + 1] = 1.0;
  x.x[range_.offset + 2] = finite_value(std::abs(e_internal), vm);
  x.x[range_.offset + 3] = p_pu;
}

bool SynchronousMachine::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service || range_.empty()) return false;
  bool changed = false;
  changed = set_if_changed(x.x, state_index(range_, 1), 1.0) || changed;
  if (machine_is_onedoneq(params_) && range_.size >= 6) {
    const OneDOneQParams mp = onedoneq_params(params_);
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const double p0 = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q0 = params_.q_elec_mvar / safe_base(params_.base_mva);
    Eigen::VectorXd z0(5);
    z0 << x.x[state_index(range_, 0)],
          x.x[state_index(range_, 4)],
          x.x[state_index(range_, 5)],
          x.x[state_index(range_, 2)],
          x.x[state_index(range_, 3)];
    const Eigen::VectorXd z = solve_onedoneq_initial_conditions(mp, v, p0, q0, z0);
    changed = set_if_changed(x.x, state_index(range_, 0), z[0]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2), z[3]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 3), z[4]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 4), z[1]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 5), z[2]) || changed;
    return changed;
  }
  if (machine_is_sauerpai(params_) && range_.size >= 10) {
    const SauerPaiParams mp = sauerpai_params(params_);
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const double p0 = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q0 = params_.q_elec_mvar / safe_base(params_.base_mva);
    Eigen::VectorXd z0(9);
    z0 << x.x[state_index(range_, 0)],
          x.x[state_index(range_, 8)],
          x.x[state_index(range_, 9)],
          x.x[state_index(range_, 2)],
          x.x[state_index(range_, 3)],
          x.x[state_index(range_, 4)],
          x.x[state_index(range_, 5)],
          x.x[state_index(range_, 6)],
          x.x[state_index(range_, 7)];
    const Eigen::VectorXd z =
        solve_sauerpai_initial_conditions(mp, v, p0, q0, z0);
    changed = set_if_changed(x.x, state_index(range_, 0), z[0]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2), z[3]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 3), z[4]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 4), z[5]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 5), z[6]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 6), z[7]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 7), z[8]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 8), z[1]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 9), z[2]) || changed;
    return changed;
  }
  if (machine_is_marconato(params_) && range_.size >= 10) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const double p0 = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q0 = params_.q_elec_mvar / safe_base(params_.base_mva);
    Eigen::VectorXd z0(9);
    z0 << x.x[state_index(range_, 0)],
          x.x[state_index(range_, 8)],
          x.x[state_index(range_, 9)],
          x.x[state_index(range_, 2)],
          x.x[state_index(range_, 3)],
          x.x[state_index(range_, 4)],
          x.x[state_index(range_, 5)],
          x.x[state_index(range_, 6)],
          x.x[state_index(range_, 7)];
    const Eigen::VectorXd z =
        solve_marconato_initial_conditions(mp, v, p0, q0, z0);
    changed = set_if_changed(x.x, state_index(range_, 0), z[0]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2), z[3]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 3), z[4]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 4), z[5]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 5), z[6]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 6), z[7]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 7), z[8]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 8), z[1]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 9), z[2]) || changed;
    return changed;
  }
  if (machine_is_simple_marconato(params_) && range_.size >= 8) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const double p0 = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q0 = params_.q_elec_mvar / safe_base(params_.base_mva);
    Eigen::VectorXd z0(7);
    z0 << x.x[state_index(range_, 0)],
          x.x[state_index(range_, 6)],
          x.x[state_index(range_, 7)],
          x.x[state_index(range_, 2)],
          x.x[state_index(range_, 3)],
          x.x[state_index(range_, 4)],
          x.x[state_index(range_, 5)];
    const Eigen::VectorXd z =
        solve_simple_marconato_initial_conditions(mp, v, p0, q0, z0);
    changed = set_if_changed(x.x, state_index(range_, 0), z[0]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2), z[3]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 3), z[4]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 4), z[5]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 5), z[6]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 6), z[1]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 7), z[2]) || changed;
    return changed;
  }
  if (machine_is_roundrotor(params_) && range_.size >= 8) {
    const GenrouParams gp = genrou_params(params_);
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const double p0 = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q0 = params_.q_elec_mvar / safe_base(params_.base_mva);
    Eigen::VectorXd z0(7);
    z0 << x.x[state_index(range_, 0)],
          x.x[state_index(range_, 6)],
          x.x[state_index(range_, 7)],
          x.x[state_index(range_, 2)],
          x.x[state_index(range_, 3)],
          x.x[state_index(range_, 4)],
          x.x[state_index(range_, 5)];
    const Eigen::VectorXd z = solve_genrou_initial_conditions(gp, v, p0, q0, z0);
    changed = set_if_changed(x.x, state_index(range_, 0), z[0]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2), z[3]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 3), z[4]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 4), z[5]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 5), z[6]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 6), z[1]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 7), z[2]) || changed;
    return changed;
  }
  if (machine_is_salient(params_) && range_.size >= 7) {
    const SalientParams sp = salient_params(params_);
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const double p0 = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q0 = params_.q_elec_mvar / safe_base(params_.base_mva);
    Eigen::VectorXd z0(6);
    z0 << x.x[state_index(range_, 0)],
          x.x[state_index(range_, 5)],
          x.x[state_index(range_, 6)],
          x.x[state_index(range_, 2)],
          x.x[state_index(range_, 3)],
          x.x[state_index(range_, 4)];
    const Eigen::VectorXd z = solve_salient_initial_conditions(sp, v, p0, q0, z0);
    changed = set_if_changed(x.x, state_index(range_, 0), z[0]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2), z[3]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 3), z[4]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 4), z[5]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 5), z[1]) || changed;
    changed = set_if_changed(x.x, state_index(range_, 6), z[2]) || changed;
    return changed;
  }
  if (params_.dynamic_angle) {
    // Re-derive the internal EMF from the network-solved terminal voltage and
    // the scheduled S, then anchor Pm to the scheduled real power. This drives
    // the machine to the *correct* equilibrium (delivering its scheduled MW with
    // dx/dt=0), rather than the previous behaviour of dragging Pm down to match
    // whatever near-zero power the E==Vt initial guess happened to produce.
    const double p_pu = params_.p_mech_mw / safe_base(params_.base_mva);
    const double q_pu = params_.q_elec_mvar / safe_base(params_.base_mva);
    const Complex vt_raw = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const Complex vt = std::abs(vt_raw) > kMinVoltage
                           ? vt_raw
                           : std::polar(clamp_voltage(params_.vm_set_pu), std::arg(vt_raw));
    const double phase_power_scale =
        std::max(0.0, std::abs(params_.phase_power_scale));
    const Complex s_phase(p_pu * phase_power_scale, q_pu * phase_power_scale);
    const Complex i_phase = std::conj(s_phase / vt);
    const Complex z(params_.r_pu, source_series_reactance_pu(params_.x_pu));
    const Complex e_internal = vt + z * i_phase;
    if (std::isfinite(std::abs(e_internal)) && std::abs(e_internal) > 0.0) {
      changed = set_if_changed(x.x, state_index(range_, 0), std::arg(e_internal)) || changed;
      changed = set_if_changed(x.x, state_index(range_, 2), std::abs(e_internal)) || changed;
    }
    changed = set_if_changed(x.x, state_index(range_, 3), p_pu) || changed;
  }
  return changed;
}

void SynchronousMachine::computeDerivatives(double,
                                           const DynamicState& x,
                                           const NetworkState& y,
                                           Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double theta = x.x[range_.offset + 0];
  const double omega = x.x[range_.offset + 1];
  const double e_mag = x.x[range_.offset + 2];
  const double pm = x.x[range_.offset + 3];
  if (machine_is_onedoneq(params_) && range_.size >= 6) {
    const OneDOneQParams mp = onedoneq_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double tau_m = x.x[state_index(range_, 4)];
    const double vf = x.x[state_index(range_, 5)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const OneDOneQEval e = evaluate_onedoneq(mp, v, delta, eq_p, ed_p);
    const double h = std::max(0.01, params_.inertia_h);
    const double wb = kTwoPi * params_.frequency_hz;
    dxdt[state_index(range_, 0)] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
    if (params_.dynamic_angle) {
      const double swing_power =
          tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 1)] =
          (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
           std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
              ? 0.0
              : swing_power / (2.0 * h);
    } else {
      dxdt[state_index(range_, 1)] = 0.0;
    }
    dxdt[state_index(range_, 2)] =
        (-eq_p - (mp.xd - mp.xdp) * e.id + vf) / mp.td0p;
    dxdt[state_index(range_, 3)] =
        (-ed_p + (mp.xq - mp.xqp) * e.iq) / mp.tq0p;
    if (!governor_attached_) dxdt[state_index(range_, 4)] = 0.0;
    if (!exciter_attached_) dxdt[state_index(range_, 5)] = 0.0;
    return;
  }
  if (machine_is_sauerpai(params_) && range_.size >= 10) {
    const SauerPaiParams mp = sauerpai_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double psi_q = x.x[state_index(range_, 2)];
    const double psi_d = x.x[state_index(range_, 3)];
    const double eq_p = x.x[state_index(range_, 4)];
    const double ed_p = x.x[state_index(range_, 5)];
    const double psi_d_pp = x.x[state_index(range_, 6)];
    const double psi_q_pp = x.x[state_index(range_, 7)];
    const double tau_m = x.x[state_index(range_, 8)];
    const double vf = x.x[state_index(range_, 9)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SauerPaiEval e =
        evaluate_sauerpai(mp, v, delta, psi_q, psi_d, eq_p, ed_p, psi_d_pp, psi_q_pp);
    const double h = std::max(0.01, params_.inertia_h);
    const double wb = kTwoPi * params_.frequency_hz;
    dxdt[state_index(range_, 0)] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
    if (params_.dynamic_angle) {
      const double swing_power =
          tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 1)] =
          (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
           std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
              ? 0.0
              : swing_power / (2.0 * h);
    } else {
      dxdt[state_index(range_, 1)] = 0.0;
    }
    dxdt[state_index(range_, 2)] =
        wb * (mp.r * e.iq - omega * psi_d + e.vq);
    dxdt[state_index(range_, 3)] =
        wb * (mp.r * e.id + omega * psi_q + e.vd);
    dxdt[state_index(range_, 4)] =
        (-eq_p -
         (mp.xd - mp.xdp) *
             (e.id - mp.gamma_d2 * psi_d_pp -
              (1.0 - mp.gamma_d1) * e.id + mp.gamma_d2 * eq_p) +
         vf) /
        mp.td0p;
    dxdt[state_index(range_, 5)] =
        (-ed_p +
         (mp.xq - mp.xqp) *
             (e.iq - mp.gamma_q2 * psi_q_pp -
              (1.0 - mp.gamma_q1) * e.iq - mp.gamma_d2 * ed_p)) /
        mp.tq0p;
    dxdt[state_index(range_, 6)] =
        (-psi_d_pp + eq_p - (mp.xdp - mp.xl) * e.id) / mp.td0pp;
    dxdt[state_index(range_, 7)] =
        (-psi_q_pp - ed_p - (mp.xqp - mp.xl) * e.iq) / mp.tq0pp;
    if (!governor_attached_) dxdt[state_index(range_, 8)] = 0.0;
    if (!exciter_attached_) dxdt[state_index(range_, 9)] = 0.0;
    return;
  }
  if (machine_is_marconato(params_) && range_.size >= 10) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double psi_q = x.x[state_index(range_, 2)];
    const double psi_d = x.x[state_index(range_, 3)];
    const double eq_p = x.x[state_index(range_, 4)];
    const double ed_p = x.x[state_index(range_, 5)];
    const double eq_pp = x.x[state_index(range_, 6)];
    const double ed_pp = x.x[state_index(range_, 7)];
    const double tau_m = x.x[state_index(range_, 8)];
    const double vf = x.x[state_index(range_, 9)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const MarconatoEval e =
        evaluate_marconato(mp, v, delta, psi_q, psi_d, eq_p, ed_p, eq_pp, ed_pp);
    const double h = std::max(0.01, params_.inertia_h);
    const double wb = kTwoPi * params_.frequency_hz;
    dxdt[state_index(range_, 0)] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
    if (params_.dynamic_angle) {
      const double swing_power =
          tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 1)] =
          (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
           std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
              ? 0.0
              : swing_power / (2.0 * h);
    } else {
      dxdt[state_index(range_, 1)] = 0.0;
    }
    dxdt[state_index(range_, 2)] =
        wb * (mp.r * e.iq - omega * psi_d + e.vq);
    dxdt[state_index(range_, 3)] =
        wb * (mp.r * e.id + omega * psi_q + e.vd);
    dxdt[state_index(range_, 4)] =
        (-eq_p - (mp.xd - mp.xdp - mp.gamma_d) * e.id +
         (1.0 - mp.t_aa / mp.td0p) * vf) / mp.td0p;
    dxdt[state_index(range_, 5)] =
        (-ed_p + (mp.xq - mp.xqp - mp.gamma_q) * e.iq) / mp.tq0p;
    dxdt[state_index(range_, 6)] =
        (-eq_pp + eq_p - (mp.xdp - mp.xdpp + mp.gamma_d) * e.id +
         (mp.t_aa / mp.td0p) * vf) / mp.td0pp;
    dxdt[state_index(range_, 7)] =
        (-ed_pp + ed_p + (mp.xqp - mp.xqpp + mp.gamma_q) * e.iq) / mp.tq0pp;
    if (!governor_attached_) dxdt[state_index(range_, 8)] = 0.0;
    if (!exciter_attached_) dxdt[state_index(range_, 9)] = 0.0;
    return;
  }
  if (machine_is_simple_marconato(params_) && range_.size >= 8) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double eq_pp = x.x[state_index(range_, 4)];
    const double ed_pp = x.x[state_index(range_, 5)];
    const double tau_m = x.x[state_index(range_, 6)];
    const double vf = x.x[state_index(range_, 7)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SimpleMarconatoEval e =
        evaluate_simple_marconato(mp, v, delta, eq_p, ed_p, eq_pp, ed_pp);
    const double h = std::max(0.01, params_.inertia_h);
    const double wb = kTwoPi * params_.frequency_hz;
    dxdt[state_index(range_, 0)] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
    if (params_.dynamic_angle) {
      const double swing_power =
          tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 1)] =
          (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
           std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
              ? 0.0
              : swing_power / (2.0 * h);
    } else {
      dxdt[state_index(range_, 1)] = 0.0;
    }
    dxdt[state_index(range_, 2)] =
        (-eq_p - (mp.xd - mp.xdp - mp.gamma_d) * e.id +
         (1.0 - mp.t_aa / mp.td0p) * vf) / mp.td0p;
    dxdt[state_index(range_, 3)] =
        (-ed_p + (mp.xq - mp.xqp - mp.gamma_q) * e.iq) / mp.tq0p;
    dxdt[state_index(range_, 4)] =
        (-eq_pp + eq_p - (mp.xdp - mp.xdpp + mp.gamma_d) * e.id +
         (mp.t_aa / mp.td0p) * vf) / mp.td0pp;
    dxdt[state_index(range_, 5)] =
        (-ed_pp + ed_p + (mp.xqp - mp.xqpp + mp.gamma_q) * e.iq) / mp.tq0pp;
    if (!governor_attached_) dxdt[state_index(range_, 6)] = 0.0;
    if (!exciter_attached_) dxdt[state_index(range_, 7)] = 0.0;
    return;
  }
  if (machine_is_roundrotor(params_) && range_.size >= 8) {
    const GenrouParams gp = genrou_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double psi_kd = x.x[state_index(range_, 4)];
    const double psi_kq = x.x[state_index(range_, 5)];
    const double tau_m = x.x[state_index(range_, 6)];
    const double vf = x.x[state_index(range_, 7)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const GenrouEval e = evaluate_genrou(gp, v, delta, eq_p, ed_p, psi_kd, psi_kq, vf);
    const double h = std::max(0.01, params_.inertia_h);
    const double wb = kTwoPi * params_.frequency_hz;
    dxdt[state_index(range_, 0)] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
    if (params_.dynamic_angle) {
      const double swing_power =
          tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 1)] =
          (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
           std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
              ? 0.0
              : swing_power / (2.0 * h);
    } else {
      dxdt[state_index(range_, 1)] = 0.0;
    }
    dxdt[state_index(range_, 2)] = (vf - e.xad_ifd) / gp.td0p;
    dxdt[state_index(range_, 3)] = -e.xaq_i1q / gp.tq0p;
    dxdt[state_index(range_, 4)] =
        (-psi_kd + eq_p - (gp.xdp - gp.xl) * e.id) / gp.td0pp;
    dxdt[state_index(range_, 5)] =
        (-psi_kq + ed_p + (gp.xqp - gp.xl) * e.iq) / gp.tq0pp;
    // tau_m (idx 6) and vf (idx 7) are constant unless a governor / exciter is
    // attached, in which case that controller owns their derivative.
    if (!governor_attached_) dxdt[state_index(range_, 6)] = 0.0;
    if (!exciter_attached_) dxdt[state_index(range_, 7)] = 0.0;
    return;
  }
  if (machine_is_salient(params_) && range_.size >= 7) {
    const SalientParams sp = salient_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double psi_kd = x.x[state_index(range_, 3)];
    const double psiq_pp = x.x[state_index(range_, 4)];
    const double tau_m = x.x[state_index(range_, 5)];
    const double vf = x.x[state_index(range_, 6)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SalientEval e = evaluate_salient(sp, v, delta, eq_p, psi_kd, psiq_pp);
    const double h = std::max(0.01, params_.inertia_h);
    const double wb = kTwoPi * params_.frequency_hz;
    dxdt[state_index(range_, 0)] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
    if (params_.dynamic_angle) {
      const double swing_power =
          tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 1)] =
          (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
           std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
              ? 0.0
              : swing_power / (2.0 * h);
    } else {
      dxdt[state_index(range_, 1)] = 0.0;
    }
    dxdt[state_index(range_, 2)] = (vf - e.xad_ifd) / sp.td0p;
    dxdt[state_index(range_, 3)] =
        (-psi_kd + eq_p - (sp.xdp - sp.xl) * e.id) / sp.td0pp;
    if (sp.exponential_saturation) {
      const double psi_pp = std::hypot(e.psi_d_pp, psiq_pp);
      const double se = salient_saturation(sp, psi_pp);
      dxdt[state_index(range_, 4)] =
          (-psiq_pp + (sp.xq - sp.xdpp) * e.iq -
           se * salient_gamma_qd(sp) * psiq_pp) / sp.tq0pp;
    } else {
      dxdt[state_index(range_, 4)] =
          (-psiq_pp - (sp.xq - sp.xdpp) * e.iq) / sp.tq0pp;
    }
    if (!governor_attached_) dxdt[state_index(range_, 5)] = 0.0;
    if (!exciter_attached_) dxdt[state_index(range_, 6)] = 0.0;
    return;
  }
  const Complex z(params_.r_pu, source_series_reactance_pu(params_.x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
  const Eigen::Vector3cd i = yv * (e - v);
  const double pe = average_complex_power(v, i).real() *
                    phase_sum_to_total_power_scale(params_.phase_power_scale);
  const double h = std::max(0.01, params_.inertia_h);
  const double wb = kTwoPi * params_.frequency_hz;

  dxdt[range_.offset + 0] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
  if (params_.dynamic_angle) {
    const double swing_power = pm - pe - params_.damping_d * (omega - 1.0);
    dxdt[range_.offset + 1] =
        (std::abs(swing_power) <= kMachinePowerBalanceTolPu &&
         std::abs(omega - 1.0) <= kMachinePowerBalanceTolPu)
            ? 0.0
            : swing_power / (2.0 * h);
  } else {
    dxdt[range_.offset + 1] = 0.0;
  }
  // e_mag (field, idx 2) and pm (idx 3) are constant unless an exciter / governor
  // is attached, in which case that controller owns their derivative.
  if (!exciter_attached_) dxdt[range_.offset + 2] = 0.0;
  if (!governor_attached_) dxdt[range_.offset + 3] = 0.0;
}

void SynchronousMachine::stamp(double,
                               const DynamicState& x,
                               const NetworkState& y,
                               DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  if (machine_is_onedoneq(params_) && range_.size >= 6) {
    const OneDOneQParams mp = onedoneq_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const OneDOneQEval e = evaluate_onedoneq(mp, v, delta, eq_p, ed_p);
    const double x_eff = std::sqrt(std::max(1e-8, mp.xdp * mp.xqp));
    const Complex z(mp.r, std::max(1e-5, x_eff));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Complex e_ri = v + z * e.current;
    const Eigen::Vector3cd e_src = balanced_phasors(std::abs(e_ri), std::arg(e_ri));
    add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
    add_balanced_current(stamp, params_.bus_pos, yv * e_src);
    return;
  }
  if (machine_is_marconato(params_) && range_.size >= 10) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double psi_q = x.x[state_index(range_, 2)];
    const double psi_d = x.x[state_index(range_, 3)];
    const double eq_p = x.x[state_index(range_, 4)];
    const double ed_p = x.x[state_index(range_, 5)];
    const double eq_pp = x.x[state_index(range_, 6)];
    const double ed_pp = x.x[state_index(range_, 7)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const MarconatoEval eval =
        evaluate_marconato(mp, v, delta, psi_q, psi_d, eq_p, ed_p, eq_pp, ed_pp);
    add_balanced_current(stamp,
                         params_.bus_pos,
                         balanced_current_from_positive_sequence(eval.current));
    return;
  }
  if (machine_is_sauerpai(params_) && range_.size >= 10) {
    const SauerPaiParams mp = sauerpai_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double psi_q = x.x[state_index(range_, 2)];
    const double psi_d = x.x[state_index(range_, 3)];
    const double eq_p = x.x[state_index(range_, 4)];
    const double ed_p = x.x[state_index(range_, 5)];
    const double psi_d_pp = x.x[state_index(range_, 6)];
    const double psi_q_pp = x.x[state_index(range_, 7)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SauerPaiEval eval =
        evaluate_sauerpai(mp, v, delta, psi_q, psi_d, eq_p, ed_p, psi_d_pp, psi_q_pp);
    add_balanced_current(stamp,
                         params_.bus_pos,
                         balanced_current_from_positive_sequence(eval.current));
    return;
  }
  if (machine_is_simple_marconato(params_) && range_.size >= 8) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double eq_pp = x.x[state_index(range_, 4)];
    const double ed_pp = x.x[state_index(range_, 5)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SimpleMarconatoEval eval =
        evaluate_simple_marconato(mp, v, delta, eq_p, ed_p, eq_pp, ed_pp);
    const double x_eff = std::sqrt(std::max(1e-8, mp.xdpp * mp.xqpp));
    const Complex z(mp.r, std::max(1e-5, x_eff));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Complex e_ri = v + z * eval.current;
    const Eigen::Vector3cd e_src = balanced_phasors(std::abs(e_ri), std::arg(e_ri));
    add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
    add_balanced_current(stamp, params_.bus_pos, yv * e_src);
    return;
  }
  if (machine_is_roundrotor(params_) && range_.size >= 8) {
    const GenrouParams gp = genrou_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double psi_kd = x.x[state_index(range_, 4)];
    const double psi_kq = x.x[state_index(range_, 5)];
    const double gd1 = genrou_gamma_d1(gp);
    const double gq1 = genrou_gamma_q1(gp);
    const double gd2 = genrou_gamma_d2(gp);
    const double psi_q_pp = gq1 * ed_p + psi_kq * (1.0 - gq1);
    const double psi_d_pp = gd1 * eq_p + gd2 * (gp.xdp - gp.xl) * psi_kd;
    const Complex e_ri = psd_dq_to_ri(delta, psi_q_pp, psi_d_pp);
    const Complex z(gp.r, gp.xdpp);
    const Complex yv = Complex(1.0, 0.0) / z;
    const Eigen::Vector3cd e = balanced_phasors(std::abs(e_ri), std::arg(e_ri));
    add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
    add_balanced_current(stamp, params_.bus_pos, yv * e);
    return;
  }
  if (machine_is_salient(params_) && range_.size >= 7) {
    const SalientParams sp = salient_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double psi_kd = x.x[state_index(range_, 3)];
    const double psiq_pp = x.x[state_index(range_, 4)];
    const double gd1 = salient_gamma_d1(sp);
    const double gq1 = salient_gamma_q1(sp);
    const double psi_d_pp = gd1 * eq_p + gq1 * psi_kd;
    const double psi_q_src = sp.exponential_saturation ? psiq_pp : -psiq_pp;
    const Complex e_ri = psd_dq_to_ri(delta, psi_q_src, psi_d_pp);
    const Complex z(sp.r, sp.xdpp);
    const Complex yv = Complex(1.0, 0.0) / z;
    const Eigen::Vector3cd e = balanced_phasors(std::abs(e_ri), std::arg(e_ri));
    add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
    add_balanced_current(stamp, params_.bus_pos, yv * e);
    return;
  }
  const double theta = x.x[range_.offset + 0];
  const double e_mag = x.x[range_.offset + 2];
  const Complex z(params_.r_pu, source_series_reactance_pu(params_.x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
  add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
  add_balanced_current(stamp, params_.bus_pos, yv * e);
}

void SynchronousMachine::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching =
      event.component_index == 0 || event.component_index == params_.component_index;
  if (!matching) return;
  if (event.type == DynamicEventType::GeneratorTrip) {
    params_.in_service = false;
  } else if (event.type == DynamicEventType::Custom &&
             (event.component_type == "SynchronousMachine" ||
              event.component_type == "Generator")) {
    const auto pref_it = event.params.find("p_ref_mw");
    const auto pmech_it = event.params.find("p_mech_mw");
    const double p_mw = pref_it != event.params.end()
                            ? pref_it->second
                            : (pmech_it != event.params.end() ? pmech_it->second
                                                              : event.value);
    if (!std::isfinite(p_mw)) return;
    params_.p_mech_mw = p_mw;
    const int pm_idx = link_.pmIndex();
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      x.x[pm_idx] = p_mw / safe_base(params_.base_mva);
    }
  }
}

std::string SynchronousMachine::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
}

std::string SynchronousMachine::modelName() const {
  return synchronous_machine_model_name(params_);
}

std::vector<DynamicModelProfile> SynchronousMachine::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput SynchronousMachine::output(const DynamicState& x,
                                               const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  GeneratorInnerVariableSnapshot inner =
      generator_inner_base_snapshot(link_.generatorBus(), x, y);
  if (machine_is_onedoneq(params_) && !range_.empty() && range_.offset + 5 < x.x.size()) {
    const OneDOneQParams mp = onedoneq_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double tau_m = x.x[state_index(range_, 4)];
    const double vf = x.x[state_index(range_, 5)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const OneDOneQEval e = evaluate_onedoneq(mp, v, delta, eq_p, ed_p);
    out.values["angle_rad"] = delta;
    out.values["delta_rad"] = delta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["eq_p"] = eq_p;
    out.values["ed_p"] = ed_p;
    out.values["vf_pu"] = vf;
    out.values["p_mech_mw"] = tau_m * safe_base(params_.base_mva);
    out.values["p_mw"] = e.pe * safe_base(params_.base_mva);
    out.values["q_mvar"] = e.qe * safe_base(params_.base_mva);
    out.values["i_rms_pu"] = std::abs(e.current);
    out.values["torque_e_pu"] = e.tau_e;
    out.values["psd_onedoneq"] = 1.0;
    inner.set(GeneratorInnerVar::ElectricalTorque, e.tau_e);
    inner.set(GeneratorInnerVar::PsiD, eq_p);
    inner.set(GeneratorInnerVar::PsiQ, ed_p);
    inner.set(GeneratorInnerVar::XadIfd, vf);
  } else if (machine_is_sauerpai(params_) && !range_.empty() &&
             range_.offset + 9 < x.x.size()) {
    const SauerPaiParams mp = sauerpai_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double psi_q = x.x[state_index(range_, 2)];
    const double psi_d = x.x[state_index(range_, 3)];
    const double eq_p = x.x[state_index(range_, 4)];
    const double ed_p = x.x[state_index(range_, 5)];
    const double psi_d_pp = x.x[state_index(range_, 6)];
    const double psi_q_pp = x.x[state_index(range_, 7)];
    const double tau_m = x.x[state_index(range_, 8)];
    const double vf = x.x[state_index(range_, 9)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SauerPaiEval e =
        evaluate_sauerpai(mp, v, delta, psi_q, psi_d, eq_p, ed_p, psi_d_pp, psi_q_pp);
    out.values["angle_rad"] = delta;
    out.values["delta_rad"] = delta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["psi_q"] = psi_q;
    out.values["psi_d"] = psi_d;
    out.values["eq_p"] = eq_p;
    out.values["ed_p"] = ed_p;
    out.values["psi_d_pp"] = psi_d_pp;
    out.values["psi_q_pp"] = psi_q_pp;
    out.values["psid_pp"] = psi_d_pp;
    out.values["psiq_pp"] = psi_q_pp;
    out.values["vf_pu"] = vf;
    out.values["p_mech_mw"] = tau_m * safe_base(params_.base_mva);
    out.values["p_mw"] = e.pe * safe_base(params_.base_mva);
    out.values["q_mvar"] = e.qe * safe_base(params_.base_mva);
    out.values["i_rms_pu"] = std::abs(e.current);
    out.values["torque_e_pu"] = e.tau_e;
    out.values["psd_sauerpai"] = 1.0;
    inner.set(GeneratorInnerVar::ElectricalTorque, e.tau_e);
    inner.set(GeneratorInnerVar::PsiD, psi_d);
    inner.set(GeneratorInnerVar::PsiQ, psi_q);
    inner.set(GeneratorInnerVar::XadIfd, vf);
  } else if (machine_is_marconato(params_) && !range_.empty() &&
             range_.offset + 9 < x.x.size()) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double psi_q = x.x[state_index(range_, 2)];
    const double psi_d = x.x[state_index(range_, 3)];
    const double eq_p = x.x[state_index(range_, 4)];
    const double ed_p = x.x[state_index(range_, 5)];
    const double eq_pp = x.x[state_index(range_, 6)];
    const double ed_pp = x.x[state_index(range_, 7)];
    const double tau_m = x.x[state_index(range_, 8)];
    const double vf = x.x[state_index(range_, 9)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const MarconatoEval e =
        evaluate_marconato(mp, v, delta, psi_q, psi_d, eq_p, ed_p, eq_pp, ed_pp);
    out.values["angle_rad"] = delta;
    out.values["delta_rad"] = delta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["psi_q"] = psi_q;
    out.values["psi_d"] = psi_d;
    out.values["psi_q_state"] = psi_q;
    out.values["psi_d_state"] = psi_d;
    out.values["eq_p"] = eq_p;
    out.values["ed_p"] = ed_p;
    out.values["eq_pp"] = eq_pp;
    out.values["ed_pp"] = ed_pp;
    out.values["vf_pu"] = vf;
    out.values["p_mech_mw"] = tau_m * safe_base(params_.base_mva);
    out.values["p_mw"] = e.pe * safe_base(params_.base_mva);
    out.values["q_mvar"] = e.qe * safe_base(params_.base_mva);
    out.values["i_rms_pu"] = std::abs(e.current);
    out.values["torque_e_pu"] = e.tau_e;
    out.values[params_.machine_model == SynchronousMachineModelKind::AndersonFouad
                   ? "psd_anderson_fouad"
                   : "psd_marconato"] = 1.0;
    inner.set(GeneratorInnerVar::ElectricalTorque, e.tau_e);
    inner.set(GeneratorInnerVar::PsiD, psi_d);
    inner.set(GeneratorInnerVar::PsiQ, psi_q);
    inner.set(GeneratorInnerVar::XadIfd, vf);
  } else if (machine_is_simple_marconato(params_) && !range_.empty() &&
             range_.offset + 7 < x.x.size()) {
    const SimpleMarconatoParams mp = simple_marconato_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double eq_pp = x.x[state_index(range_, 4)];
    const double ed_pp = x.x[state_index(range_, 5)];
    const double tau_m = x.x[state_index(range_, 6)];
    const double vf = x.x[state_index(range_, 7)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SimpleMarconatoEval e =
        evaluate_simple_marconato(mp, v, delta, eq_p, ed_p, eq_pp, ed_pp);
    out.values["angle_rad"] = delta;
    out.values["delta_rad"] = delta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["eq_p"] = eq_p;
    out.values["ed_p"] = ed_p;
    out.values["eq_pp"] = eq_pp;
    out.values["ed_pp"] = ed_pp;
    out.values["vf_pu"] = vf;
    out.values["p_mech_mw"] = tau_m * safe_base(params_.base_mva);
    out.values["p_mw"] = e.pe * safe_base(params_.base_mva);
    out.values["q_mvar"] = e.qe * safe_base(params_.base_mva);
    out.values["i_rms_pu"] = std::abs(e.current);
    out.values["torque_e_pu"] = e.tau_e;
    out.values[params_.machine_model == SynchronousMachineModelKind::SimpleAF
                   ? "psd_simple_af"
                   : "psd_simple_marconato"] = 1.0;
    inner.set(GeneratorInnerVar::ElectricalTorque, e.tau_e);
    inner.set(GeneratorInnerVar::PsiD, eq_pp);
    inner.set(GeneratorInnerVar::PsiQ, ed_pp);
    inner.set(GeneratorInnerVar::XadIfd, vf);
  } else if (machine_is_roundrotor(params_) && !range_.empty() && range_.offset + 7 < x.x.size()) {
    const GenrouParams gp = genrou_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double ed_p = x.x[state_index(range_, 3)];
    const double psi_kd = x.x[state_index(range_, 4)];
    const double psi_kq = x.x[state_index(range_, 5)];
    const double tau_m = x.x[state_index(range_, 6)];
    const double vf = x.x[state_index(range_, 7)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const GenrouEval e = evaluate_genrou(gp, v, delta, eq_p, ed_p, psi_kd, psi_kq, vf);
    out.values["angle_rad"] = delta;
    out.values["delta_rad"] = delta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["eq_p"] = eq_p;
    out.values["ed_p"] = ed_p;
    out.values["psi_kd"] = psi_kd;
    out.values["psi_kq"] = psi_kq;
    out.values["vf_pu"] = vf;
    out.values["p_mech_mw"] = tau_m * safe_base(params_.base_mva);
    out.values["p_mw"] = e.pe * safe_base(params_.base_mva);
    out.values["q_mvar"] = e.qe * safe_base(params_.base_mva);
    out.values["i_rms_pu"] = std::abs(e.current);
    out.values["torque_e_pu"] = e.tau_e;
    out.values[gp.exponential_saturation ? "psd_genroe" : "psd_genrou"] = 1.0;
    inner.set(GeneratorInnerVar::ElectricalTorque, e.tau_e);
    inner.set(GeneratorInnerVar::PsiD, e.psi_d_pp);
    inner.set(GeneratorInnerVar::PsiQ, e.psi_q_pp);
    inner.set(GeneratorInnerVar::XadIfd, e.xad_ifd);
  } else if (machine_is_salient(params_) && !range_.empty() && range_.offset + 6 < x.x.size()) {
    const SalientParams sp = salient_params(params_);
    const double delta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double eq_p = x.x[state_index(range_, 2)];
    const double psi_kd = x.x[state_index(range_, 3)];
    const double psiq_pp = x.x[state_index(range_, 4)];
    const double tau_m = x.x[state_index(range_, 5)];
    const double vf = x.x[state_index(range_, 6)];
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const SalientEval e = evaluate_salient(sp, v, delta, eq_p, psi_kd, psiq_pp);
    out.values["angle_rad"] = delta;
    out.values["delta_rad"] = delta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["eq_p"] = eq_p;
    out.values["psi_kd"] = psi_kd;
    out.values["psiq_pp"] = psiq_pp;
    out.values["psi_q_pp"] = psiq_pp;
    out.values["psi_d_pp"] = e.psi_d_pp;
    out.values["vf_pu"] = vf;
    out.values["p_mech_mw"] = tau_m * safe_base(params_.base_mva);
    out.values["p_mw"] = e.pe * safe_base(params_.base_mva);
    out.values["q_mvar"] = e.qe * safe_base(params_.base_mva);
    out.values["i_rms_pu"] = std::abs(e.current);
    out.values["torque_e_pu"] = e.tau_e;
    out.values[sp.exponential_saturation ? "psd_gensae" : "psd_gensal"] = 1.0;
    inner.set(GeneratorInnerVar::ElectricalTorque, e.tau_e);
    inner.set(GeneratorInnerVar::PsiD, e.psi_d_pp);
    inner.set(GeneratorInnerVar::PsiQ, e.psi_q_source);
    inner.set(GeneratorInnerVar::XadIfd, e.xad_ifd);
  } else if (!range_.empty() && range_.offset + 3 < x.x.size()) {
    const double theta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double e_mag = x.x[state_index(range_, 2)];
    const double pm = x.x[state_index(range_, 3)];
    out.values["angle_rad"] = theta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["e_internal_pu"] = e_mag;
    out.values["p_mech_mw"] = pm * safe_base(params_.base_mva);
    const Complex z(params_.r_pu, source_series_reactance_pu(params_.x_pu));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
    const Eigen::Vector3cd i = yv * (e - v);
    const Complex s = average_complex_power(v, i) *
                      phase_sum_to_total_power_scale(params_.phase_power_scale);
    out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
    out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
    out.values["i_rms_pu"] =
        std::sqrt((std::norm(i[0]) + std::norm(i[1]) + std::norm(i[2])) / 3.0);
    inner.set(GeneratorInnerVar::ElectricalTorque, s.real());
    inner.set(GeneratorInnerVar::PsiD, e_mag);
    inner.set(GeneratorInnerVar::PsiQ, 0.0);
    inner.set(GeneratorInnerVar::XadIfd, e_mag);
  }
  add_generator_inner_outputs(out, inner);
  add_generator_block_flags(out, link_.generatorBus());
  add_voltage_metrics(out, y, params_.bus_pos);
  return out;
}

FiveMassShaft::FiveMassShaft(FiveMassShaftParams params) : params_(std::move(params)) {}

void FiveMassShaft::assignStateIndices(int& offset) {
  range_ = {offset, 10};
  offset += range_.size;
}

double FiveMassShaft::mechanicalTorque(const DynamicState& x) const {
  if (machine_ != nullptr && machine_->valid) {
    const int pm = machine_->pmIndex();
    if (pm >= 0 && pm < x.x.size()) return x.x[pm];
  }
  return 0.0;
}

double FiveMassShaft::recoveredElectricalTorque(
    const DynamicState& x,
    Eigen::Ref<const Eigen::VectorXd> dxdt) const {
  const double tau_m = mechanicalTorque(x);
  if (machine_ == nullptr || !machine_->valid) return tau_m;
  const int omega = machine_->omegaIndex();
  if (omega < 0 || omega >= x.x.size() || omega >= dxdt.size()) return tau_m;
  const double domega = dxdt[omega];
  if (!std::isfinite(domega)) return tau_m;
  return tau_m - 2.0 * std::max(0.01, machine_->inertia_h) * domega;
}

void FiveMassShaft::setEquilibrium(DynamicState& x) const {
  if (range_.empty()) return;
  double delta = 0.0;
  double omega = 1.0;
  if (machine_ != nullptr && machine_->valid && machine_->range != nullptr) {
    const int d = machine_->range->offset;
    const int w = machine_->omegaIndex();
    if (d >= 0 && d < x.x.size()) delta = x.x[d];
    if (w >= 0 && w < x.x.size()) omega = x.x[w];
  }
  const double tau = mechanicalTorque(x);
  std::array<double, 5> theta{};
  theta[4] = delta;
  for (int k = 3; k >= 0; --k) {
    const double stiffness = std::max(1e-9, std::abs(params_.stiffness_pu[static_cast<std::size_t>(k)]));
    theta[static_cast<std::size_t>(k)] = theta[static_cast<std::size_t>(k + 1)] + tau / stiffness;
  }
  for (int i = 0; i < 5; ++i) {
    x.x[state_index(range_, i)] = theta[static_cast<std::size_t>(i)];
    x.x[state_index(range_, 5 + i)] = omega;
  }
}

void FiveMassShaft::initializeFromPowerFlow(const PowerFlowResult&,
                                            DynamicState& x,
                                            NetworkState&) {
  setEquilibrium(x);
}

bool FiveMassShaft::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  const Eigen::VectorXd before = x.x;
  setEquilibrium(x);
  return before.size() == x.x.size() && (before - x.x).cwiseAbs().maxCoeff() > 1e-12;
}

void FiveMassShaft::computeDerivatives(double,
                                       const DynamicState& x,
                                       const NetworkState&,
                                       Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  std::array<double, 5> theta{};
  std::array<double, 5> omega{};
  for (int i = 0; i < 5; ++i) {
    theta[static_cast<std::size_t>(i)] = x.x[state_index(range_, i)];
    omega[static_cast<std::size_t>(i)] = x.x[state_index(range_, 5 + i)];
  }
  std::array<double, 4> torque{};
  for (int i = 0; i < 4; ++i) {
    const double k = params_.stiffness_pu[static_cast<std::size_t>(i)];
    const double d = params_.damping_pu[static_cast<std::size_t>(i)];
    torque[static_cast<std::size_t>(i)] =
        k * (theta[static_cast<std::size_t>(i)] -
             theta[static_cast<std::size_t>(i + 1)]) +
        d * (omega[static_cast<std::size_t>(i)] -
             omega[static_cast<std::size_t>(i + 1)]);
  }
  const double tau_m = mechanicalTorque(x);
  const double tau_e = recoveredElectricalTorque(x, dxdt);
  const double wb = kTwoPi * positive_or(params_.frequency_hz, 50.0);
  for (int i = 0; i < 5; ++i) {
    dxdt[state_index(range_, i)] = wb * (omega[static_cast<std::size_t>(i)] - 1.0);
  }
  const auto h = [&](int i) {
    return std::max(0.01, params_.inertia_h[static_cast<std::size_t>(i)]);
  };
  dxdt[state_index(range_, 5)] = (tau_m - torque[0]) / (2.0 * h(0));
  dxdt[state_index(range_, 6)] = (torque[0] - torque[1]) / (2.0 * h(1));
  dxdt[state_index(range_, 7)] = (torque[1] - torque[2]) / (2.0 * h(2));
  dxdt[state_index(range_, 8)] = (torque[2] - torque[3]) / (2.0 * h(3));
  dxdt[state_index(range_, 9)] = (torque[3] - tau_e) / (2.0 * h(4));

  if (machine_ != nullptr && machine_->valid && machine_->range != nullptr) {
    const int delta = machine_->range->offset;
    const int omega_idx = machine_->omegaIndex();
    if (delta >= 0 && delta < dxdt.size()) dxdt[delta] = dxdt[state_index(range_, 4)];
    if (omega_idx >= 0 && omega_idx < dxdt.size()) {
      dxdt[omega_idx] = dxdt[state_index(range_, 9)];
    }
  }
}

void FiveMassShaft::stamp(double, const DynamicState&, const NetworkState&, DynamicStamp&) const {}

void FiveMassShaft::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index ||
                        event.component_index == params_.machine_index;
  if (!matching) return;
  if (event.type == DynamicEventType::GeneratorTrip) {
    params_.in_service = false;
    if (!range_.empty()) {
      for (int k = 0; k < range_.size; ++k) {
        if (state_index(range_, k) < x.x.size()) x.x[state_index(range_, k)] = 0.0;
      }
    }
  }
}

std::string FiveMassShaft::name() const {
  return params_.label.empty() ? "Five-mass shaft " + std::to_string(params_.component_index)
                               : params_.label;
}

DynamicDeviceOutput FiveMassShaft::output(const DynamicState& x, const NetworkState&) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.machine_index,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["mass_count"] = 5.0;
  out.values["state_count"] = 10.0;
  if (!range_.empty() && state_index(range_, 9) < x.x.size()) {
    for (int i = 0; i < 5; ++i) {
      out.values["theta_" + std::to_string(i + 1) + "_rad"] = x.x[state_index(range_, i)];
      out.values["omega_" + std::to_string(i + 1) + "_pu"] = x.x[state_index(range_, 5 + i)];
    }
    out.values["omega_generator_pu"] = x.x[state_index(range_, 9)];
    out.values["p_mech_input_mw"] = mechanicalTorque(x) * safe_base(params_.base_mva);
  }
  return out;
}

// Stabilizing signal Vs of a single-input speed PSS (washout + 2 lead-lag),
// as a pure algebraic function of the PSS states and the machine speed. Shared
// by PowerSystemStabilizer::output and Exciter::computeDerivatives so the AVR can
// read Vs without a mutable cache. At equilibrium (omega=1, PSS states=0) Vs=0.
static double pss_vs_output(const PSSOutputLink& L, const DynamicState& x) {
  if (!L.valid || L.range == nullptr || L.machine == nullptr || !L.machine->valid) {
    return 0.0;
  }
  const int o = L.range->offset;
  if (o < 0 || o + 2 >= x.x.size()) return 0.0;
  const int omega_idx = L.machine->omegaIndex();
  if (omega_idx < 0 || omega_idx >= x.x.size()) return 0.0;
  const double u = x.x[omega_idx] - 1.0;
  if (L.model == 1) {
    if (o + 6 >= x.x.size()) return 0.0;
    const double xp1 = x.x[o + 0];
    const double xp2 = x.x[o + 1];
    const double xp3 = x.x[o + 2];
    const double xp4 = x.x[o + 3];
    const double xp5 = x.x[o + 4];
    const double xp6 = x.x[o + 5];
    const double xp7 = x.x[o + 6];
    const double a2 = control_time(L.a2);
    const double t2 = control_time(L.t2_s);
    const double t4 = control_time(L.t4_s);
    const double t6 = control_time(L.t6_s);
    const double y_f =
        (L.a6 / a2) * xp2 +
        (L.a5 - L.a1 * (L.a6 / a2)) * xp3 +
        (1.0 - L.a6 / a2) * xp4;
    const double y_ll1 = xp5 + (L.t1_s / t2) * y_f;
    const double y_ll2 = xp6 + (L.t3_s / t4) * y_ll1;
    const double y_out = (L.ks * L.t5_s / t6) * (y_ll2 - xp7);
    const double vss = std::clamp(y_out, L.vs_min_pu, L.vs_max_pu);
    return pss_output_limiter(vss, 1.0, L.vcl, L.vcu);
  }
  if (L.model == 2) {
    const double y_hp = x.x[o + 0] + L.kt * u;
    const double y1 = x.x[o + 1] + L.t1_over_t3 * y_hp;
    const double y2 = x.x[o + 2] + L.t2_over_t4 * y1;
    return std::clamp(y2, -L.h_lim, L.h_lim);
  }
  const double t2 = std::max(kMinTimeConstant, L.t2_s);
  const double t4 = std::max(kMinTimeConstant, L.t4_s);
  const double y_w = L.ks * u - x.x[o + 0];
  const double y1 = (L.t1_s / t2) * y_w + x.x[o + 1];
  const double y2 = (L.t3_s / t4) * y1 + x.x[o + 2];
  return std::clamp(y2, L.vs_min_pu, L.vs_max_pu);
}

Governor::Governor(GovernorDynamicParams params) : params_(std::move(params)) {}

void Governor::assignStateIndices(int& offset) {
  const int count = params_.model == GovernorModel::TGTypeI ? 3
                    : (params_.model == GovernorModel::TGOV1 ? 2 : 1);
  range_ = {offset, count};  // machine owns the published tau_m/pm slot
  offset += range_.size;
}

void Governor::initializeFromPowerFlow(const PowerFlowResult&, DynamicState& x, NetworkState&) {
  if (range_.empty()) return;
  double p0 = params_.p_ref_mw / safe_base(params_.base_mva);
  if (machine_ != nullptr && machine_->valid) {
    // Anchor the reference to the machine's initialized mechanical power so the
    // valve/turbine train is at equilibrium (dx/dt = 0) at t0.
    const int pm_idx = machine_->pmIndex();
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      p0 = x.x[pm_idx];
      params_.p_ref_mw =
          (params_.model == GovernorModel::TGOV1 ? params_.droop_r * p0 : p0) *
          safe_base(params_.base_mva);
    }
  }
  if (params_.model == GovernorModel::TGTypeI && range_.size >= 3) {
    const double tc = control_time(params_.tc_s);
    const double t3 = params_.t3_s;
    const double t5 = control_time(params_.t5_s);
    const double t4 = params_.t4_s;
    x.x[state_index(range_, 0)] = p0;
    x.x[state_index(range_, 1)] = (1.0 - t3 / tc) * p0;
    x.x[state_index(range_, 2)] = (1.0 - t4 / t5) * p0;
  } else if (params_.model == GovernorModel::TGOV1 && range_.size >= 2) {
    const double t3 = control_time(params_.turbine_t_s);
    x.x[state_index(range_, 0)] = p0;
    x.x[state_index(range_, 1)] = (1.0 - params_.t2_s / t3) * p0;
  } else {
    x.x[range_.offset] =
        params_.model == GovernorModel::TGTypeII ? 0.0 : p0;
  }
}

bool Governor::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  double p0 = params_.p_ref_mw / safe_base(params_.base_mva);
  if (machine_ != nullptr && machine_->valid) {
    const int pm_idx = machine_->pmIndex();
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      p0 = x.x[pm_idx];
      params_.p_ref_mw =
          (params_.model == GovernorModel::TGOV1 ? params_.droop_r * p0 : p0) *
          safe_base(params_.base_mva);
    }
  }
  bool changed = false;
  if (params_.model == GovernorModel::TGTypeI && range_.size >= 3) {
    const double tc = control_time(params_.tc_s);
    const double t3 = params_.t3_s;
    const double t5 = control_time(params_.t5_s);
    const double t4 = params_.t4_s;
    changed = set_if_changed(x.x, state_index(range_, 0), p0) || changed;
    changed = set_if_changed(x.x, state_index(range_, 1),
                             (1.0 - t3 / tc) * p0) || changed;
    changed = set_if_changed(x.x, state_index(range_, 2),
                             (1.0 - t4 / t5) * p0) || changed;
    return changed;
  }
  if (params_.model == GovernorModel::TGOV1 && range_.size >= 2) {
    const double t3 = control_time(params_.turbine_t_s);
    changed = set_if_changed(x.x, state_index(range_, 0), p0) || changed;
    changed = set_if_changed(x.x, state_index(range_, 1),
                             (1.0 - params_.t2_s / t3) * p0) || changed;
    return changed;
  }
  return set_if_changed(x.x, range_.offset,
                        params_.model == GovernorModel::TGTypeII ? 0.0 : p0);
}

void Governor::computeDerivatives(double,
                                  const DynamicState& x,
                                  const NetworkState&,
                                  Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double base = safe_base(params_.base_mva);
  const double pmax = params_.pmax_mw > 0.0 ? params_.pmax_mw / base
                                            : std::numeric_limits<double>::infinity();
  const double pmin = params_.pmin_mw < 0.0 ? params_.pmin_mw / base
                                            : -std::numeric_limits<double>::infinity();
  const double pref = std::clamp(params_.p_ref_mw / base, pmin, pmax);
  const double t1 = control_time(params_.t_s);

  if (machine_ != nullptr && machine_->valid) {
    // Load-reference-set governor: valve responds to droop about the speed
    // deviation; the machine's pm/tau_m slot is the turbine output (lag of the
    // valve). Reheat models (IEEEG1) simply use a longer turbine time constant.
    const int omega_idx = machine_->omegaIndex();
    const int pm_idx = machine_->pmIndex();
    const double omega = (omega_idx >= 0 && omega_idx < x.x.size()) ? x.x[omega_idx] : 1.0;
    const double droop = params_.droop_r > 1e-9 ? params_.droop_r : 0.05;
    if (params_.model == GovernorModel::TGOV1 && range_.size >= 2) {
      const double xg1 = x.x[state_index(range_, 0)];
      const double xg2 = x.x[state_index(range_, 1)];
      const double ts = control_time(params_.t_s);
      const double t3 = control_time(params_.turbine_t_s);
      const double t2_over_t3 = params_.t2_s / t3;
      const double ref_in = (pref - (omega - 1.0)) / droop;
      const double xg1_sat = std::clamp(xg1, pmin, pmax);
      double dxg1 = (ref_in - xg1) / ts;
      if ((xg1 >= pmax && dxg1 > 0.0) || (xg1 <= pmin && dxg1 < 0.0)) {
        dxg1 = 0.0;
      }
      const double dxg2 = ((1.0 - t2_over_t3) * xg1_sat - xg2) / t3;
      const double tau_m =
          (xg2 + t2_over_t3 * xg1_sat - params_.damping_d_t * (omega - 1.0)) /
          std::max(kMinVoltage, std::abs(omega));
      dxdt[state_index(range_, 0)] = dxg1;
      dxdt[state_index(range_, 1)] = dxg2;
      if (pm_idx >= 0 && pm_idx < x.x.size()) {
        const double dtau_est = dxg2 + t2_over_t3 * dxg1;
        dxdt[pm_idx] = dtau_est + (tau_m - x.x[pm_idx]) / t3;
      }
      return;
    }
    const double valve_cmd = std::clamp(pref - (omega - 1.0) / droop, pmin, pmax);
    if (params_.model == GovernorModel::TGTypeI && range_.size >= 3) {
      const double xg1 = x.x[state_index(range_, 0)];
      const double xg2 = x.x[state_index(range_, 1)];
      const double xg3 = x.x[state_index(range_, 2)];
      const double ts = control_time(params_.t_s);
      const double tc = control_time(params_.tc_s);
      const double t3 = params_.t3_s;
      const double t5 = control_time(params_.t5_s);
      const double t4 = params_.t4_s;
      const double dxg1 = (valve_cmd - xg1) / ts;
      const double y_ll = xg2 + (t3 / tc) * xg1;
      const double dxg2 = ((1.0 - t3 / tc) * xg1 - xg2) / tc;
      const double tau_m = xg3 + (t4 / t5) * y_ll;
      const double dxg3 = ((1.0 - t4 / t5) * y_ll - xg3) / t5;
      dxdt[state_index(range_, 0)] = dxg1;
      dxdt[state_index(range_, 1)] = dxg2;
      dxdt[state_index(range_, 2)] = dxg3;
      if (pm_idx >= 0 && pm_idx < x.x.size()) {
        dxdt[pm_idx] = dxg3 + (t4 / t5) * (dxg2 + (t3 / tc) * dxg1);
        if (std::abs(tau_m - x.x[pm_idx]) > 1e-8) {
          dxdt[pm_idx] += (tau_m - x.x[pm_idx]) / t5;
        }
      }
      return;
    }
    if (params_.model == GovernorModel::TGTypeII) {
      const double xg = x.x[range_.offset];
      const double t2 = control_time(params_.turbine_t_s);
      const auto [tau_delta, dxg] =
          lead_lag_block((1.0 - omega) / droop, xg, 1.0, params_.t_s, t2);
      dxdt[range_.offset] = dxg;
      if (pm_idx >= 0 && pm_idx < x.x.size()) {
        const double tau_m = std::clamp(pref + tau_delta, pmin, pmax);
        dxdt[pm_idx] = dxg + (tau_m - x.x[pm_idx]) / t2;
      }
      return;
    }
    const double valve = x.x[range_.offset];
    dxdt[range_.offset] = (valve_cmd - valve) / t1;
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      const double t_turb = control_time(
          params_.model == GovernorModel::IEEEG1 ? params_.reheat_t_s : params_.turbine_t_s);
      dxdt[pm_idx] = (valve - x.x[pm_idx]) / t_turb;
    }
    return;
  }
  // Backward-compatible standalone behaviour: own state tracks the reference.
  dxdt[range_.offset] = (pref - x.x[range_.offset]) / t1;
}

void Governor::stamp(double, const DynamicState&, const NetworkState&, DynamicStamp&) const {}

void Governor::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index;
  if (!matching) return;
  if (event.type == DynamicEventType::GeneratorTrip) {
    params_.in_service = false;
    if (!range_.empty() && range_.offset < x.x.size()) x.x[range_.offset] = 0.0;
  } else if (event.type == DynamicEventType::Custom && event.component_type == "Governor") {
    const auto it = event.params.find("p_ref_mw");
    params_.p_ref_mw = it == event.params.end() ? event.value : it->second;
  }
}

std::string Governor::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
}

DynamicDeviceOutput Governor::output(const DynamicState& x, const NetworkState&) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             0,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["droop_r"] = params_.droop_r;
  out.values["p_ref_mw"] = params_.p_ref_mw;
  out.values["attached"] = machine_ != nullptr ? 1.0 : 0.0;
  if (!range_.empty() && range_.offset < x.x.size()) {
    out.values["valve_pu"] = x.x[range_.offset];
    if (params_.model == GovernorModel::TGOV1 && range_.size >= 2) {
      out.values["lead_lag_state_pu"] = x.x[state_index(range_, 1)];
      out.values["t2_s"] = params_.t2_s;
      out.values["d_t"] = params_.damping_d_t;
    }
    if (params_.model == GovernorModel::TGTypeI && range_.size >= 3) {
      out.values["servo_state_pu"] = x.x[state_index(range_, 1)];
      out.values["reheat_state_pu"] = x.x[state_index(range_, 2)];
    }
  }
  if (machine_ != nullptr && machine_->valid) {
    const int pm_idx = machine_->pmIndex();
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      out.values["p_mech_mw"] = x.x[pm_idx] * safe_base(params_.base_mva);
    }
  } else if (!range_.empty() && range_.offset < x.x.size()) {
    out.values["p_mech_mw"] = x.x[range_.offset] * safe_base(params_.base_mva);
  }
  return out;
}

Exciter::Exciter(ExciterDynamicParams params) : params_(std::move(params)) {}

void Exciter::assignStateIndices(int& offset) {
  int count = 1;
  if (machine_ != nullptr) {
    count = params_.model == ExciterModel::SEXS
                ? 1
                : (params_.model == ExciterModel::AVRTypeI ||
             params_.model == ExciterModel::AVRTypeII)
                      ? 3
                      : 0;
  } else if (params_.model == ExciterModel::AVRTypeI ||
             params_.model == ExciterModel::AVRTypeII) {
    count = 4;
  }
  range_ = {offset, count};
  offset += range_.size;
}

double Exciter::terminalVoltage(const NetworkState& y) const {
  const int bus = machine_ != nullptr && machine_->valid
                      ? machine_->generatorBus().bus_pos
                      : params_.bus_pos;
  if (bus < 0) return params_.v_ref_pu;
  return std::abs(positive_sequence_voltage(bus_voltage(y, bus)));
}

void Exciter::captureReference(const DynamicState& x, const NetworkState& y) {
  const double vt = terminalVoltage(y);
  double field0 = params_.v_ref_pu;
  if (machine_ != nullptr && machine_->valid) {
    const int f = machine_->fieldIndex();
    if (f >= 0 && f < x.x.size()) field0 = x.x[f];
  } else if (!range_.empty() && range_.offset < x.x.size()) {
    field0 = x.x[range_.offset];
  }
  field0_ = field0;
  if (params_.model == ExciterModel::SEXS) {
    const double k = std::max(1e-9, std::abs(params_.ka));
    v_ref_captured_ = vt + field0 / k;
  } else {
    // Hold the equilibrium terminal voltage as the AVR setpoint for compact AVRs.
    v_ref_captured_ = vt;
  }
  captured_ = true;
}

void Exciter::initializeFromPowerFlow(const PowerFlowResult& pf, DynamicState& x, NetworkState& y) {
  // Prefer the power-flow terminal voltage at init (network not yet solved here).
  double vt = params_.v_ref_pu;
  const int bus = machine_ != nullptr && machine_->valid
                      ? machine_->generatorBus().bus_pos
                      : params_.bus_pos;
  if (bus >= 0 && bus < static_cast<int>(pf.vm.size())) {
    vt = positive_or(pf.vm[static_cast<std::size_t>(bus)], vt);
  }
  double field0 = params_.v_ref_pu;
  if (machine_ != nullptr && machine_->valid) {
    const int f = machine_->fieldIndex();
    if (f >= 0 && f < x.x.size()) field0 = x.x[f];
  } else if (!range_.empty()) {
    field0 = std::clamp(vt, params_.efd_min_pu, params_.efd_max_pu);
    x.x[range_.offset] = field0;
  }
  field0_ = field0;
  v_ref_captured_ = vt;
  if (params_.model == ExciterModel::SEXS) {
    const double k = std::max(1e-9, std::abs(params_.ka));
    const double vin0 = field0 / k;
    v_ref_captured_ = vt + vin0;
    if (machine_ != nullptr && machine_->valid && !range_.empty()) {
      x.x[range_.offset] = (1.0 - params_.ta_over_tb) * vin0;
    }
  }
  if (params_.model == ExciterModel::AVRTypeI ||
      params_.model == ExciterModel::AVRTypeII) {
    const int base = range_.offset;
    const bool attached = machine_ != nullptr && machine_->valid;
    const int vr1_local = attached ? 0 : 1;
    const int vr2_local = attached ? 1 : 2;
    const int vm_local = attached ? 2 : 3;
    const double vf = field0;
    const double se = avr_saturation(params_.ae, params_.be, vf);
    if (!attached && !range_.empty()) x.x[base] = vf;
    if (params_.model == ExciterModel::AVRTypeI) {
      const double ka = std::max(1e-9, std::abs(params_.ka));
      const double ke = params_.ke;
      const double tf = control_time(params_.tf_s);
      const double vr1 = vf * (ke + se);
      const double vr2 = -(params_.kf / tf) * vf;
      x.x[base + vr1_local] = vr1;
      x.x[base + vr2_local] = vr2;
      x.x[base + vm_local] = vt;
      v_ref_captured_ = vt + vr1 / ka;
    } else {
      const double k0 = std::max(1e-9, std::abs(params_.k0));
      const double t1 = control_time(params_.t1_s);
      const double t2 = params_.t2_s;
      const double t3 = control_time(params_.t3_s);
      const double t4 = params_.t4_s;
      const double vr = std::clamp(vf * (1.0 + se),
                                   params_.va_min_pu,
                                   params_.va_max_pu);
      const double u = vr / k0;
      const double vr1 = k0 * (1.0 - t2 / t1) * u;
      const double vr2 = ((1.0 - t4 / t3) * vr) / k0;
      x.x[base + vr1_local] = vr1;
      x.x[base + vr2_local] = vr2;
      x.x[base + vm_local] = vt;
      v_ref_captured_ = vt + u;
    }
  }
  captured_ = true;
}

bool Exciter::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service) return false;
  captureReference(x, y);  // re-anchor to the network-solved equilibrium
  if (params_.model == ExciterModel::SEXS && machine_ != nullptr &&
      machine_->valid && !range_.empty()) {
    const int f = machine_->fieldIndex();
    const double vf = (f >= 0 && f < x.x.size()) ? x.x[f] : field0_;
    const double k = std::max(1e-9, std::abs(params_.ka));
    const double vin0 = vf / k;
    v_ref_captured_ = terminalVoltage(y) + vin0;
    captured_ = true;
    return set_if_changed(x.x, range_.offset,
                          (1.0 - params_.ta_over_tb) * vin0);
  }
  if (params_.model == ExciterModel::AVRTypeI ||
      params_.model == ExciterModel::AVRTypeII) {
    const bool attached = machine_ != nullptr && machine_->valid;
    if (range_.empty()) return false;
    const int base = range_.offset;
    const int vr1_local = attached ? 0 : 1;
    const int vr2_local = attached ? 1 : 2;
    const int vm_local = attached ? 2 : 3;
    const double vt = terminalVoltage(y);
    double vf = field0_;
    if (attached) {
      const int f = machine_->fieldIndex();
      if (f >= 0 && f < x.x.size()) vf = x.x[f];
    } else if (base < x.x.size()) {
      vf = x.x[base];
    }
    const double se = avr_saturation(params_.ae, params_.be, vf);
    bool changed = false;
    if (!attached) changed = set_if_changed(x.x, base, vf) || changed;
    if (params_.model == ExciterModel::AVRTypeI) {
      const double ka = std::max(1e-9, std::abs(params_.ka));
      const double tf = control_time(params_.tf_s);
      const double vr1 = vf * (params_.ke + se);
      const double vr2 = -(params_.kf / tf) * vf;
      v_ref_captured_ = vt + vr1 / ka;
      changed = set_if_changed(x.x, base + vr1_local, vr1) || changed;
      changed = set_if_changed(x.x, base + vr2_local, vr2) || changed;
      changed = set_if_changed(x.x, base + vm_local, vt) || changed;
    } else {
      const double k0 = std::max(1e-9, std::abs(params_.k0));
      const double t1 = control_time(params_.t1_s);
      const double t2 = params_.t2_s;
      const double t3 = control_time(params_.t3_s);
      const double t4 = params_.t4_s;
      const double vr = std::clamp(vf * (1.0 + se),
                                   params_.va_min_pu,
                                   params_.va_max_pu);
      const double u = vr / k0;
      const double vr1 = k0 * (1.0 - t2 / t1) * u;
      const double vr2 = ((1.0 - t4 / t3) * vr) / k0;
      v_ref_captured_ = vt + u;
      changed = set_if_changed(x.x, base + vr1_local, vr1) || changed;
      changed = set_if_changed(x.x, base + vr2_local, vr2) || changed;
      changed = set_if_changed(x.x, base + vm_local, vt) || changed;
    }
    captured_ = true;
    return changed;
  }
  if (machine_ != nullptr) return false;  // coupled exciter owns no state to trim
  if (range_.empty()) return false;
  return set_if_changed(x.x, range_.offset,
                        std::clamp(field0_, params_.efd_min_pu, params_.efd_max_pu));
}

void Exciter::computeDerivatives(double,
                                 const DynamicState& x,
                                 const NetworkState& y,
                                 Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service) return;
  const double vt = terminalVoltage(y);
  const double vref = captured_ ? v_ref_captured_ : params_.v_ref_pu;
  const double vs = pss_ != nullptr ? pss_vs_output(*pss_, x) : 0.0;
  const double t = control_time(
      params_.model == ExciterModel::IEEET1 ? params_.te_s : params_.ta_s);

  if (machine_ != nullptr && machine_->valid) {
    // Proportional AVR about the equilibrium field: efd_cmd = field0 +
    // Ka*(Vref - Vt + Vs). Drives the machine-owned field state through a lag.
    const int f = machine_->fieldIndex();
    if (f < 0 || f >= x.x.size()) return;
    if (params_.model == ExciterModel::SEXS && !range_.empty()) {
      const double vf = x.x[f];
      const double vr = x.x[range_.offset];
      const double tb = control_time(params_.tb_s);
      const double ta = params_.ta_over_tb * tb;
      const auto [v_ll, dvr] = lead_lag_block(vref + vs - vt, vr, 1.0, ta, tb);
      const double efd_cmd = std::clamp(params_.ka * v_ll,
                                        params_.efd_min_pu,
                                        params_.efd_max_pu);
      dxdt[range_.offset] = dvr;
      dxdt[f] = (efd_cmd - vf) / control_time(params_.te_s);
      return;
    }
    if (params_.model == ExciterModel::AVRSimple) {
      dxdt[f] = params_.kv * (vref - vt);
      return;
    }
    if ((params_.model == ExciterModel::AVRTypeI ||
         params_.model == ExciterModel::AVRTypeII) &&
        !range_.empty()) {
      const int base = range_.offset;
      const double vf = x.x[f];
      const double vr1 = x.x[base + 0];
      const double vr2 = x.x[base + 1];
      const double vm = x.x[base + 2];
      const auto vm_block = low_pass_block(vt, vm, 1.0, params_.tr_s);
      const double dvm = vm_block.second;
      dxdt[base + 2] = dvm;
      const double se = avr_saturation(params_.ae, params_.be, vf);
      if (params_.model == ExciterModel::AVRTypeI) {
        const auto [yhp, dvr2] = high_pass_block(vf, vr2, params_.kf, params_.tf_s);
        const auto [yr, dvr1] =
            low_pass_block(vref + vs - vm - yhp, vr1, params_.ka, params_.ta_s);
        (void)yr;
        dxdt[base + 0] = dvr1;
        dxdt[base + 1] = dvr2;
        dxdt[f] =
            low_pass_modified_block(vr1, vf, 1.0, params_.ke + se, params_.te_s).second;
      } else {
        const auto [yll1, dvr1] =
            lead_lag_block(vref + vs - vm, vr1, params_.k0, params_.t2_s, params_.t1_s);
        const auto [yll2, dvr2] =
            lead_lag_block(yll1,
                           params_.k0 * vr2,
                           1.0,
                           params_.k0 * params_.t4_s,
                           params_.k0 * params_.t3_s);
        const double vr = std::clamp(yll2, params_.va_min_pu, params_.va_max_pu);
        dxdt[base + 0] = dvr1;
        dxdt[base + 1] = dvr2;
        dxdt[f] =
            low_pass_modified_block(vr, vf, 1.0, 1.0 + se, params_.te_s).second;
      }
      return;
    }
    const double efd_cmd = std::clamp(field0_ + params_.ka * (vref - vt + vs),
                                      params_.efd_min_pu, params_.efd_max_pu);
    dxdt[f] = (efd_cmd - x.x[f]) / t;
    return;
  }
  if (range_.empty()) return;
  if (params_.model == ExciterModel::AVRSimple) {
    dxdt[range_.offset] = params_.kv * (vref - vt);
    return;
  }
  if ((params_.model == ExciterModel::AVRTypeI ||
       params_.model == ExciterModel::AVRTypeII) &&
      range_.size >= 4) {
    const int base = range_.offset;
    const double vf = x.x[base + 0];
    const double vr1 = x.x[base + 1];
    const double vr2 = x.x[base + 2];
    const double vm = x.x[base + 3];
    dxdt[base + 3] = low_pass_block(vt, vm, 1.0, params_.tr_s).second;
    const double se = avr_saturation(params_.ae, params_.be, vf);
    if (params_.model == ExciterModel::AVRTypeI) {
      const auto [yhp, dvr2] = high_pass_block(vf, vr2, params_.kf, params_.tf_s);
      dxdt[base + 1] =
          low_pass_block(vref + vs - vm - yhp, vr1, params_.ka, params_.ta_s).second;
      dxdt[base + 2] = dvr2;
      dxdt[base + 0] =
          low_pass_modified_block(vr1, vf, 1.0, params_.ke + se, params_.te_s).second;
    } else {
      const auto [yll1, dvr1] =
          lead_lag_block(vref + vs - vm, vr1, params_.k0, params_.t2_s, params_.t1_s);
      const auto [yll2, dvr2] =
          lead_lag_block(yll1,
                         params_.k0 * vr2,
                         1.0,
                         params_.k0 * params_.t4_s,
                         params_.k0 * params_.t3_s);
      const double vr = std::clamp(yll2, params_.va_min_pu, params_.va_max_pu);
      dxdt[base + 1] = dvr1;
      dxdt[base + 2] = dvr2;
      dxdt[base + 0] =
          low_pass_modified_block(vr, vf, 1.0, 1.0 + se, params_.te_s).second;
    }
    return;
  }
  const double efd_cmd = std::clamp(params_.ka * (vref - vt + vs) + vref,
                                    params_.efd_min_pu, params_.efd_max_pu);
  dxdt[range_.offset] = (efd_cmd - x.x[range_.offset]) / t;
}

void Exciter::stamp(double, const DynamicState&, const NetworkState&, DynamicStamp&) const {}

void Exciter::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index;
  if (!matching) return;
  if (event.type == DynamicEventType::GeneratorTrip) {
    params_.in_service = false;
    if (!range_.empty() && range_.offset < x.x.size()) x.x[range_.offset] = 0.0;
  } else if (event.type == DynamicEventType::Custom && event.component_type == "Exciter") {
    const auto it = event.params.find("v_ref_pu");
    params_.v_ref_pu = it == event.params.end() ? event.value : it->second;
    captured_ = false;  // re-capture the operating point on the next trim
  }
}

std::string Exciter::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
}

DynamicDeviceOutput Exciter::output(const DynamicState& x, const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["v_ref_pu"] = captured_ ? v_ref_captured_ : params_.v_ref_pu;
  out.values["ka"] = params_.ka;
  out.values["attached"] = machine_ != nullptr ? 1.0 : 0.0;
  double efd = field0_;
  if (machine_ != nullptr && machine_->valid) {
    const int f = machine_->fieldIndex();
    if (f >= 0 && f < x.x.size()) efd = x.x[f];
  } else if (!range_.empty() && range_.offset < x.x.size()) {
    efd = x.x[range_.offset];
  }
  out.values["efd_pu"] = efd;
  if (params_.model == ExciterModel::SEXS && !range_.empty() &&
      range_.offset < x.x.size()) {
    out.values["vr_pu"] = x.x[range_.offset];
    out.values["ta_over_tb"] = params_.ta_over_tb;
    out.values["tb_s"] = params_.tb_s;
    out.values["te_s"] = params_.te_s;
  }
  add_voltage_metrics(out, y, machine_ != nullptr && machine_->valid
                                  ? machine_->generatorBus().bus_pos
                                  : params_.bus_pos);
  return out;
}

// ── Power System Stabilizer (PSS1A single-input speed stabilizer) ──
PowerSystemStabilizer::PowerSystemStabilizer(PSSDynamicParams params)
    : params_(std::move(params)) {}

void PowerSystemStabilizer::attachMachine(const MachineControlLink* link) {
  machine_ = link;
  link_.machine = link;
}

void PowerSystemStabilizer::assignStateIndices(int& offset) {
  range_ = {offset, params_.model == PSSModel::IEEEST ? 7 : 3};
  offset += range_.size;
  link_.range = &range_;
  link_.machine = machine_;
  link_.valid = machine_ != nullptr && machine_->valid;
  link_.model = params_.model == PSSModel::IEEEST ? 1
                 : (params_.model == PSSModel::STAB1 ? 2 : 0);
  link_.ks = params_.ks;
  link_.tw_s = params_.tw_s;
  link_.t1_s = params_.t1_s;
  link_.t2_s = params_.t2_s;
  link_.t3_s = params_.t3_s;
  link_.t4_s = params_.t4_s;
  link_.vs_max_pu = params_.vs_max_pu;
  link_.vs_min_pu = params_.vs_min_pu;
  link_.a1 = params_.a1;
  link_.a2 = params_.a2;
  link_.a3 = params_.a3;
  link_.a4 = params_.a4;
  link_.a5 = params_.a5;
  link_.a6 = params_.a6;
  link_.t5_s = params_.t5_s;
  link_.t6_s = params_.t6_s;
  link_.vcu = params_.vcu;
  link_.vcl = params_.vcl;
  link_.input_code = params_.input_code;
  link_.kt = params_.kt;
  link_.stab_t_s = params_.stab_t_s;
  link_.t1_over_t3 = params_.t1_over_t3;
  link_.t2_over_t4 = params_.t2_over_t4;
  link_.h_lim = params_.h_lim;
}

void PowerSystemStabilizer::initializeFromPowerFlow(const PowerFlowResult&,
                                                    DynamicState& x,
                                                    NetworkState&) {
  // Zero states => Vs = 0 at the (omega = 1) equilibrium, so the PSS cannot
  // disturb the initialized operating point.
  if (range_.empty()) return;
  for (int k = 0; k < range_.size; ++k) x.x[state_index(range_, k)] = 0.0;
  link_.valid = machine_ != nullptr && machine_->valid;
}

bool PowerSystemStabilizer::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  bool changed = false;
  for (int k = 0; k < range_.size; ++k) {
    changed = set_if_changed(x.x, state_index(range_, k), 0.0) || changed;
  }
  return changed;
}

void PowerSystemStabilizer::computeDerivatives(double,
                                               const DynamicState& x,
                                               const NetworkState&,
                                               Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  if (machine_ == nullptr || !machine_->valid) {
    for (int k = 0; k < range_.size; ++k) dxdt[state_index(range_, k)] = 0.0;
    return;
  }
  const int omega_idx = machine_->omegaIndex();
  const double u = (omega_idx >= 0 && omega_idx < x.x.size()) ? x.x[omega_idx] - 1.0 : 0.0;
  if (params_.model == PSSModel::IEEEST && range_.size >= 7) {
    const double xp1 = x.x[state_index(range_, 0)];
    const double xp2 = x.x[state_index(range_, 1)];
    const double xp3 = x.x[state_index(range_, 2)];
    const double xp4 = x.x[state_index(range_, 3)];
    const double xp5 = x.x[state_index(range_, 4)];
    const double xp6 = x.x[state_index(range_, 5)];
    const double xp7 = x.x[state_index(range_, 6)];
    const double a2 = control_time(params_.a2);
    const double a4 = control_time(params_.a4);
    const double t2 = control_time(params_.t2_s);
    const double t4 = control_time(params_.t4_s);
    const double t6 = control_time(params_.t6_s);
    dxdt[state_index(range_, 0)] =
        (u - params_.a3 * xp1 - xp2) / a4;
    dxdt[state_index(range_, 1)] = xp1;
    dxdt[state_index(range_, 2)] =
        (xp2 - params_.a1 * xp3 - xp4) / a2;
    dxdt[state_index(range_, 3)] = xp3;
    const double y_f =
        (params_.a6 / a2) * xp2 +
        (params_.a5 - params_.a1 * (params_.a6 / a2)) * xp3 +
        (1.0 - params_.a6 / a2) * xp4;
    const auto [y_ll1, dxp5] =
        lead_lag_block(y_f, xp5, 1.0, params_.t1_s, t2);
    const auto [y_ll2, dxp6] =
        lead_lag_block(y_ll1, xp6, 1.0, params_.t3_s, t4);
    const auto hp = high_pass_block(y_ll2, xp7, params_.ks * params_.t5_s, t6);
    const double dxp7 = hp.second;
    dxdt[state_index(range_, 4)] = dxp5;
    dxdt[state_index(range_, 5)] = dxp6;
    dxdt[state_index(range_, 6)] = dxp7;
    return;
  }
  if (params_.model == PSSModel::STAB1) {
    const double t = control_time(params_.stab_t_s);
    const double t3 = control_time(params_.t3_s);
    const double t4 = control_time(params_.t4_s);
    const double x1 = x.x[state_index(range_, 0)];
    const double x2 = x.x[state_index(range_, 1)];
    const double x3 = x.x[state_index(range_, 2)];
    const double y_hp = x1 + params_.kt * u;
    const double y1 = x2 + params_.t1_over_t3 * y_hp;
    dxdt[state_index(range_, 0)] = -(params_.kt * u + x1) / t;
    dxdt[state_index(range_, 1)] =
        ((1.0 - params_.t1_over_t3) * y_hp - x2) / t3;
    dxdt[state_index(range_, 2)] =
        ((1.0 - params_.t2_over_t4) * y1 - x3) / t4;
    return;
  }
  const double tw = control_time(params_.tw_s);
  const double t2 = control_time(params_.t2_s);
  const double t4 = control_time(params_.t4_s);
  const double x1 = x.x[state_index(range_, 0)];
  const double x2 = x.x[state_index(range_, 1)];
  const double x3 = x.x[state_index(range_, 2)];
  const double y_w = params_.ks * u - x1;            // washout output
  const double y1 = (params_.t1_s / t2) * y_w + x2;  // lead-lag 1 output
  dxdt[state_index(range_, 0)] = (params_.ks * u - x1) / tw;
  dxdt[state_index(range_, 1)] = ((1.0 - params_.t1_s / t2) * y_w - x2) / t2;
  dxdt[state_index(range_, 2)] = ((1.0 - params_.t3_s / t4) * y1 - x3) / t4;
}

void PowerSystemStabilizer::stamp(double, const DynamicState&, const NetworkState&,
                                  DynamicStamp&) const {}

void PowerSystemStabilizer::handleEvent(const DynamicEvent& event, DynamicState& x,
                                        NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index;
  if (!matching) return;
  if (event.type == DynamicEventType::GeneratorTrip) {
    params_.in_service = false;
    if (!range_.empty()) {
      for (int k = 0; k < range_.size; ++k) {
        if (state_index(range_, k) < x.x.size()) x.x[state_index(range_, k)] = 0.0;
      }
    }
  }
}

std::string PowerSystemStabilizer::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
}

DynamicDeviceOutput PowerSystemStabilizer::output(const DynamicState& x,
                                                  const NetworkState&) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["ks"] = params_.ks;
  out.values["attached"] = machine_ != nullptr ? 1.0 : 0.0;
  out.values["vs_pu"] = pss_vs_output(link_, x);
  return out;
}

GridFormingInverter::GridFormingInverter(GridFormingInverterParams params)
    : params_(std::move(params)) {}

void GridFormingInverter::assignStateIndices(int& offset) {
  range_ = {offset,
            params_.dc_link_mode == DCLinkMode::DynamicDCVoltage && params_.dc_bus_pos >= 0
                ? 7
                : 6};
  offset += range_.size;
  inner_vars_ = InverterInnerVariableBus{};
  inner_vars_.range = &range_;
  inner_vars_.valid = true;
  inner_vars_.bus_pos = params_.bus_pos;
  inner_vars_.dc_bus_pos = params_.dc_bus_pos;
  inner_vars_.grid_following = false;
  inner_vars_.has_dc_source = params_.dc_bus_pos >= 0;
  inner_vars_.has_frequency_estimator = false;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::VoltageReference))] = 1;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::FilteredActivePower))] = 2;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::FilteredReactivePower))] = 3;
  if (range_.size > 6) {
    inner_vars_.state_local[static_cast<std::size_t>(
        static_cast<int>(InverterInnerVar::DCVoltage))] = 6;
  }
}

void GridFormingInverter::initializeFromPowerFlow(const PowerFlowResult& pf,
                                                 DynamicState& x,
                                                 NetworkState& y) {
  double angle = params_.angle_ref_rad;
  if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.va.size())) {
    angle = pf.va[static_cast<std::size_t>(params_.bus_pos)];
  }
  x.x[state_index(range_, 0)] = angle;
  x.x[state_index(range_, 1)] = std::clamp(params_.v_ref_pu,
                                           params_.vmin_internal_pu,
                                           params_.vmax_internal_pu);
  x.x[state_index(range_, 2)] =
      params_.control_kind == GridFormingControlKind::VirtualInertia
          ? 1.0
          : params_.p_ref_mw / safe_base(params_.base_mva);
  x.x[state_index(range_, 3)] = params_.q_ref_mvar / safe_base(params_.base_mva);
  x.x[state_index(range_, 4)] = 0.0;
  x.x[state_index(range_, 5)] = 0.0;
  if (range_.size > 6) {
    double vdc = params_.vdc_ref_pu;
    if (params_.dc_bus_pos >= 0 && params_.dc_bus_pos < y.Vdc.size()) {
      vdc = y.Vdc[params_.dc_bus_pos];
    }
    x.x[state_index(range_, 6)] =
        clamp_voltage_window(vdc, params_.vdc_min_pu, params_.vdc_max_pu);
  }
}

bool GridFormingInverter::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service || range_.empty() || params_.bus_pos < 0) return false;
  const Eigen::Vector3cd vabc = bus_voltage(y, params_.bus_pos);
  const Complex v = positive_sequence_voltage(vabc);
  const double vt = std::max(kMinVoltage, std::abs(v));
  const double v_angle = std::arg(v);
  const double p_ref = params_.p_ref_mw / safe_base(params_.base_mva);
  const double q_ref = params_.q_ref_mvar / safe_base(params_.base_mva);
  const Complex z(params_.virtual_r_pu, std::max(1e-5, params_.virtual_x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Complex s_ref(p_ref, q_ref);
  const Complex i_ref = std::conj(s_ref / v);
  const Complex e = v + z * i_ref;
  const double e_mag =
      clamp_voltage_window(std::abs(e), params_.vmin_internal_pu, params_.vmax_internal_pu);
  const double theta = std::arg(e);

  const double pmax = params_.pmax_mw > 0.0
                          ? params_.pmax_mw / safe_base(params_.base_mva)
                          : std::numeric_limits<double>::infinity();
  const double pmin = params_.pmin_mw < 0.0
                          ? params_.pmin_mw / safe_base(params_.base_mva)
                          : -std::numeric_limits<double>::infinity();
  const double overload =
      (std::isfinite(pmax) ? std::max(0.0, p_ref - pmax) : 0.0) -
      (std::isfinite(pmin) ? std::max(0.0, pmin - p_ref) : 0.0);
  const double e_droop = params_.v_ref_pu;
  const double voltage_error = e_droop - vt;
  double xi_v = x.x[state_index(range_, 4)];
  if (std::abs(params_.voltage_ki) > 1e-9) {
    xi_v = (e_mag - e_droop - params_.voltage_kp * voltage_error) / params_.voltage_ki;
  }
  double xi_ol = 0.0;
  if (std::abs(params_.overload_ki) > 1e-9) {
    xi_ol = -params_.overload_kp * overload / params_.overload_ki;
  }

  bool changed = false;
  changed = set_if_changed(x.x, state_index(range_, 0), theta) || changed;
  changed = set_if_changed(x.x, state_index(range_, 1), e_mag) || changed;
  changed = set_if_changed(x.x,
                           state_index(range_, 2),
                           params_.control_kind == GridFormingControlKind::VirtualInertia
                               ? 1.0
                               : p_ref) || changed;
  changed = set_if_changed(x.x, state_index(range_, 3), q_ref) || changed;
  changed = set_if_changed(x.x, state_index(range_, 4), finite_value(xi_v)) || changed;
  changed = set_if_changed(x.x, state_index(range_, 5), finite_value(xi_ol)) || changed;
  if (range_.size > 6) {
    double vdc = params_.vdc_ref_pu;
    if (params_.dc_bus_pos >= 0 && params_.dc_bus_pos < y.Vdc.size()) {
      vdc = y.Vdc[params_.dc_bus_pos];
    }
    const Eigen::Vector3cd i = yv * (balanced_phasors(e_mag, theta) - vabc);
    const double p_ac = finite_value(average_complex_power(vabc, i).real(), p_ref);
    vdc = dc_link_voltage_for_power_balance(y,
                                            params_.dc_bus_pos,
                                            p_ac,
                                            params_.dc_link_conductance_pu,
                                            params_.eta,
                                            vdc,
                                            params_.vdc_min_pu,
                                            params_.vdc_max_pu);
    changed = set_if_changed(x.x,
                             state_index(range_, 6),
                             vdc) || changed;
  }
  return changed;
}

void GridFormingInverter::computeDerivatives(double,
                                             const DynamicState& x,
                                             const NetworkState& y,
                                             Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double theta = x.x[state_index(range_, 0)];
  const double e_mag = x.x[state_index(range_, 1)];
  const Complex z(params_.virtual_r_pu, std::max(1e-5, params_.virtual_x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
  const Eigen::Vector3cd i = yv * (e - v);
  const Complex s = average_complex_power(v, i);
  const double p = finite_value(s.real());
  const double q = finite_value(s.imag());
  const double state2 = x.x[state_index(range_, 2)];
  const double pf =
      params_.control_kind == GridFormingControlKind::VirtualInertia ? p : state2;
  const double qf = x.x[state_index(range_, 3)];
  const double xi_v = x.x[state_index(range_, 4)];
  const double xi_ol = x.x[state_index(range_, 5)];
  const double t_filter = std::max(kMinTimeConstant, params_.power_filter_t_s);
  const double tq_filter =
      std::max(kMinTimeConstant,
               params_.reactive_power_filter_t_s > 0.0 ? params_.reactive_power_filter_t_s
                                                        : params_.power_filter_t_s);
  const double t_voltage = std::max(kMinTimeConstant, params_.voltage_control_t_s);
  const double p_ref = params_.p_ref_mw / safe_base(params_.base_mva);
  const double q_ref = params_.q_ref_mvar / safe_base(params_.base_mva);
  const double pmax = params_.pmax_mw > 0.0
                          ? params_.pmax_mw / safe_base(params_.base_mva)
                          : std::numeric_limits<double>::infinity();
  const double pmin = params_.pmin_mw < 0.0
                          ? params_.pmin_mw / safe_base(params_.base_mva)
                          : -std::numeric_limits<double>::infinity();
  const double e_pmax = std::isfinite(pmax) ? std::max(0.0, pf - pmax) : 0.0;
  const double e_pmin = std::isfinite(pmin) ? std::max(0.0, pmin - pf) : 0.0;
  const double overload = e_pmax - e_pmin;
  const double overload_correction =
      params_.overload_kp * overload + params_.overload_ki * xi_ol;

  const double vt = avg_voltage_mag(v);
  const double e_droop = params_.v_ref_pu - params_.q_droop_pu * (qf - q_ref);
  const double voltage_error = e_droop - vt;
  double e_cmd = e_droop + params_.voltage_kp * voltage_error + params_.voltage_ki * xi_v;
  if (params_.current_limit_pu > 0.0) {
    const double irms =
        std::sqrt((std::norm(i[0]) + std::norm(i[1]) + std::norm(i[2])) / 3.0);
    if (irms > params_.current_limit_pu) {
      const double scale = params_.current_limit_pu / std::max(irms, 1e-9);
      e_cmd = vt + scale * (e_cmd - vt);
    }
  }
  e_cmd = std::clamp(e_cmd, params_.vmin_internal_pu, params_.vmax_internal_pu);

  if (params_.control_kind == GridFormingControlKind::VirtualInertia) {
    const double omega = state2;
    const double ta = std::max(kMinTimeConstant, params_.vsm_ta_s);
    const double omega_ref = 1.0;
    const double omega_pll = 1.0;
    dxdt[state_index(range_, 0)] =
        params_.reference_frame_locked
            ? 0.0
            : kTwoPi * params_.frequency_hz * (omega - 1.0);
    dxdt[state_index(range_, 1)] = (e_cmd - e_mag) / t_voltage;
    dxdt[state_index(range_, 2)] =
        (p_ref - p -
         params_.vsm_damping_kd * (omega - omega_pll) -
         params_.vsm_frequency_droop_kw * (omega - omega_ref)) / ta;
    dxdt[state_index(range_, 3)] = (q - qf) / tq_filter;
    dxdt[state_index(range_, 4)] = voltage_error;
    dxdt[state_index(range_, 5)] = overload;
  } else if (params_.control_kind == GridFormingControlKind::VirtualOscillator) {
    const double e_safe = std::max(kMinVoltage, std::abs(e_mag));
    const double gamma = params_.voc_psi_rad - kPi / 2.0;
    const double p_error = p_ref - p;
    const double q_error = q_ref - q;
    const double omega_oc =
        1.0 +
        params_.voc_k1 / (e_safe * e_safe) *
            (std::cos(gamma) * p_error + std::sin(gamma) * q_error);
    dxdt[state_index(range_, 0)] =
        params_.reference_frame_locked
            ? 0.0
            : kTwoPi * params_.frequency_hz * (omega_oc - 1.0);
    dxdt[state_index(range_, 1)] =
        kTwoPi * params_.frequency_hz *
        (params_.voc_k1 / e_safe *
             (-std::sin(gamma) * p_error + std::cos(gamma) * q_error) +
         params_.voc_k2 * (params_.v_ref_pu * params_.v_ref_pu - e_safe * e_safe) *
             e_safe);
    dxdt[state_index(range_, 2)] = (p - state2) / t_filter;
    dxdt[state_index(range_, 3)] = (q - qf) / tq_filter;
    dxdt[state_index(range_, 4)] = 0.0;
    dxdt[state_index(range_, 5)] = 0.0;
  } else {
    const double angle_rate =
        kTwoPi * params_.frequency_hz * params_.p_droop_pu * (p_ref - pf) -
        overload_correction;
    dxdt[state_index(range_, 0)] =
        params_.reference_frame_locked ? 0.0 : angle_rate;
    dxdt[state_index(range_, 1)] = (e_cmd - e_mag) / t_voltage;
    dxdt[state_index(range_, 2)] = (p - pf) / t_filter;
    dxdt[state_index(range_, 3)] = (q - qf) / tq_filter;
    dxdt[state_index(range_, 4)] = voltage_error;
    dxdt[state_index(range_, 5)] = overload;
  }
  if (range_.size > 6) {
    const double vdc_link = x.x[state_index(range_, 6)];
    const double pdc_net = dc_link_network_power_pu(y,
                                                   params_.dc_bus_pos,
                                                   vdc_link,
                                                   params_.dc_link_conductance_pu);
    dxdt[state_index(range_, 6)] =
        dc_link_voltage_derivative(p,
                                   pdc_net,
                                   vdc_link,
                                   params_.dc_link_capacitance_s,
                                   params_.eta,
                                   params_.vdc_min_pu,
                                   params_.vdc_max_pu);
  }
}

void GridFormingInverter::stamp(double,
                                const DynamicState& x,
                                const NetworkState& y,
                                DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const Complex z(params_.virtual_r_pu, std::max(1e-5, params_.virtual_x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Eigen::Vector3cd e = balanced_phasors(x.x[state_index(range_, 1)],
                                              x.x[state_index(range_, 0)]);
  add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
  add_balanced_current(stamp, params_.bus_pos, yv * e);

  if (params_.dc_bus_pos >= 0) {
    if (params_.dc_link_mode == DCLinkMode::DynamicDCVoltage && range_.size > 6) {
      const double vdc_link =
          clamp_voltage_window(x.x[state_index(range_, 6)],
                               params_.vdc_min_pu,
                               params_.vdc_max_pu);
      const double g = std::max(0.0, params_.dc_link_conductance_pu);
      stamp.addDcConductance(params_.dc_bus_pos, params_.dc_bus_pos, g);
      stamp.addDcCurrent(params_.dc_bus_pos, g * vdc_link);
      return;
    }
    const double p_ac = x.x[state_index(range_, 2)] * safe_base(params_.base_mva);
    const double p_dc_pu = -p_ac / std::max(1e-6, params_.eta) / safe_base(params_.base_mva);
    const double vdc = (params_.dc_bus_pos < y.Vdc.size()) ? clamp_voltage(y.Vdc[params_.dc_bus_pos]) : 1.0;
    stamp.addDcCurrent(params_.dc_bus_pos, p_dc_pu / vdc);
  }
}

void GridFormingInverter::addJacobian(
    double,
    const DynamicState& x,
    const NetworkState& y,
    const DynamicJacobianContext& context,
    std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const int theta_col = state_index(range_, 0);
  const int e_mag_col = state_index(range_, 1);
  if (context.validStateIndex(theta_col) && context.validStateIndex(e_mag_col)) {
    const Complex z(params_.virtual_r_pu, std::max(1e-5, params_.virtual_x_pu));
    const Complex yv = Complex(1.0, 0.0) / z;
    const double theta = x.x[theta_col];
    const double e_mag = x.x[e_mag_col];
    const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
    const Eigen::Vector3cd d_i_dtheta = yv * Complex(0.0, 1.0) * e;
    const Eigen::Vector3cd d_i_de = yv * balanced_phasors(1.0, theta);
    add_balanced_current_derivative(context, params_.bus_pos, theta_col, d_i_dtheta, triplets);
    add_balanced_current_derivative(context, params_.bus_pos, e_mag_col, d_i_de, triplets);
  }

  if (params_.dc_bus_pos < 0 || !context.validDcNode(params_.dc_bus_pos)) return;
  if (params_.dc_link_mode == DCLinkMode::DynamicDCVoltage && range_.size > 6) {
    const int vdc_col = state_index(range_, 6);
    if (context.validStateIndex(vdc_col)) {
      add_dc_current_derivative(context,
                                params_.dc_bus_pos,
                                vdc_col,
                                std::max(0.0, params_.dc_link_conductance_pu),
                                triplets);
    }
    return;
  }
  const int p_col = state_index(range_, 2);
  if (!context.validStateIndex(p_col) ||
      params_.dc_bus_pos >= y.Vdc.size()) {
    return;
  }
  const double eta = std::max(1e-6, params_.eta);
  const double raw_vdc = y.Vdc[params_.dc_bus_pos];
  const double vdc = clamp_voltage(raw_vdc);
  add_dc_current_derivative(context, params_.dc_bus_pos, p_col, -1.0 / (eta * vdc), triplets);
  add_dc_current_derivative(context,
                            params_.dc_bus_pos,
                            context.dcCol(params_.dc_bus_pos),
                            x.x[p_col] * clamp_voltage_slope(raw_vdc) / (eta * vdc * vdc),
                            triplets);
}

void GridFormingInverter::handleEvent(const DynamicEvent& event, DynamicState&, NetworkState&) {
  if (event.type == DynamicEventType::VSCTrip &&
      (event.component_index == 0 || event.component_index == params_.component_index)) {
    params_.in_service = false;
  } else if (event.type == DynamicEventType::Custom &&
             event.component_type == "VSC" &&
             (event.component_index == 0 || event.component_index == params_.component_index)) {
    const auto p_it = event.params.find("p_ref_mw");
    if (p_it != event.params.end()) params_.p_ref_mw = p_it->second;
    const auto q_it = event.params.find("q_ref_mvar");
    if (q_it != event.params.end()) params_.q_ref_mvar = q_it->second;
    const auto v_it = event.params.find("v_ref_pu");
    if (v_it != event.params.end()) params_.v_ref_pu = v_it->second;
  }
}

std::string GridFormingInverter::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> GridFormingInverter::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput GridFormingInverter::output(const DynamicState& x,
                                                const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["control_mode"] = 1.0;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  InverterInnerVariableSnapshot inner;
  if (!range_.empty() && state_index(range_, 5) < x.x.size()) {
    const double theta = x.x[state_index(range_, 0)];
    const double e_mag = x.x[state_index(range_, 1)];
    const double state2 = x.x[state_index(range_, 2)];
    const bool is_vsm = params_.control_kind == GridFormingControlKind::VirtualInertia;
    const bool is_voc = params_.control_kind == GridFormingControlKind::VirtualOscillator;
    const double pf = is_vsm ? 0.0 : state2;
    const double qf = x.x[state_index(range_, 3)];
    const Complex z(params_.virtual_r_pu, std::max(1e-5, params_.virtual_x_pu));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
    const Eigen::Vector3cd i = yv * (e - v);
    const Complex s = average_complex_power(v, i);
    const double p_ref = params_.p_ref_mw / safe_base(params_.base_mva);
    const double pmax = params_.pmax_mw > 0.0
                            ? params_.pmax_mw / safe_base(params_.base_mva)
                            : std::numeric_limits<double>::infinity();
    const double pmin = params_.pmin_mw < 0.0
                            ? params_.pmin_mw / safe_base(params_.base_mva)
                            : -std::numeric_limits<double>::infinity();
    const double overload =
        (std::isfinite(pmax) ? std::max(0.0, pf - pmax) : 0.0) -
        (std::isfinite(pmin) ? std::max(0.0, pmin - pf) : 0.0);
    out.values["angle_rad"] = theta;
    out.values["theta_oc_rad"] = theta;
    out.values["e_internal_pu"] = e_mag;
    out.values["E_oc_pu"] = e_mag;
    out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
    out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
    out.values["p_filtered_mw"] =
        (is_vsm ? s.real() : pf) * safe_base(params_.base_mva);
    out.values["q_filtered_mvar"] = qf * safe_base(params_.base_mva);
    out.values["pm_pu"] = is_vsm ? s.real() : pf;
    out.values["qm_pu"] = qf;
    out.values["p_ref_mw"] = params_.p_ref_mw;
    out.values["q_ref_mvar"] = params_.q_ref_mvar;
    double omega_oc = 1.0 - params_.p_droop_pu * (pf - p_ref);
    if (is_vsm) {
      omega_oc = state2;
    } else if (is_voc) {
      const double e_safe = std::max(kMinVoltage, std::abs(e_mag));
      const double gamma = params_.voc_psi_rad - kPi / 2.0;
      omega_oc =
          1.0 +
          params_.voc_k1 / (e_safe * e_safe) *
              (std::cos(gamma) * (p_ref - s.real()) +
               std::sin(gamma) *
                   (params_.q_ref_mvar / safe_base(params_.base_mva) - s.imag()));
    }
    out.values["omega_oc_pu"] = omega_oc;
    out.values["frequency_hz"] = params_.frequency_hz * omega_oc;
    out.values["control_mode_id"] =
        is_vsm ? 2.0 : (is_voc ? 3.0 : 1.0);
    out.values["i_rms_pu"] =
        std::sqrt((std::norm(i[0]) + std::norm(i[1]) + std::norm(i[2])) / 3.0);
    out.values["current_limit_active"] =
        (params_.current_limit_pu > 0.0 && out.values["i_rms_pu"] > params_.current_limit_pu)
            ? 1.0
            : 0.0;
    out.values["overload_pu"] = overload;
    const Complex e_pos = std::polar(e_mag, theta);
    const Complex v_pos = positive_sequence_voltage(v);
    const Complex i_pos = positive_sequence_voltage(i);
    inner.set(InverterInnerVar::VoltageReference, e_mag);
    inner.set(InverterInnerVar::FrequencyReference, out.values["frequency_hz"]);
    inner.set(InverterInnerVar::ConverterVoltageReal, e_pos.real());
    inner.set(InverterInnerVar::ConverterVoltageImag, e_pos.imag());
    inner.set(InverterInnerVar::FilterVoltageReal, v_pos.real());
    inner.set(InverterInnerVar::FilterVoltageImag, v_pos.imag());
    inner.set(InverterInnerVar::FilteredActivePower, is_vsm ? s.real() : pf);
    inner.set(InverterInnerVar::FilteredReactivePower, qf);
    inner.set(InverterInnerVar::FilterCurrentReal, i_pos.real());
    inner.set(InverterInnerVar::FilterCurrentImag, i_pos.imag());
    inner.set(InverterInnerVar::ConverterCurrentReal, i_pos.real());
    inner.set(InverterInnerVar::ConverterCurrentImag, i_pos.imag());
    if (range_.size > 6 && state_index(range_, 6) < x.x.size()) {
      const double vdc_link = x.x[state_index(range_, 6)];
      out.values["vdc_link_pu"] = vdc_link;
      const double pdc_net =
          dc_link_network_power_pu(y,
                                   params_.dc_bus_pos,
                                   vdc_link,
                                   params_.dc_link_conductance_pu);
      out.values["p_dc_mw"] = pdc_net * safe_base(params_.base_mva);
      out.values["dc_link_dynamic"] = 1.0;
      inner.set(InverterInnerVar::DCVoltage, vdc_link);
    } else {
      out.values["dc_link_dynamic"] = 0.0;
    }
  }
  add_inverter_block_flags(out, inner_vars_);
  add_inverter_inner_outputs(out, inner);
  add_voltage_metrics(out, y, params_.bus_pos, params_.dc_bus_pos);
  return out;
}

GridFollowingInverter::GridFollowingInverter(GridFollowingInverterParams params)
    : params_(std::move(params)) {}

void GridFollowingInverter::assignStateIndices(int& offset) {
  range_ = {offset, gfl_state_count(params_)};
  offset += range_.size;
  inner_vars_ = InverterInnerVariableBus{};
  inner_vars_.range = &range_;
  inner_vars_.valid = true;
  inner_vars_.bus_pos = params_.bus_pos;
  inner_vars_.dc_bus_pos = params_.dc_bus_pos;
  inner_vars_.grid_following = true;
  inner_vars_.has_dc_source =
      params_.stamp_dc_power && params_.dc_bus_pos >= 0;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::PllAngle))] = 0;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::OuterCurrentD))] = 2;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::OuterCurrentQ))] = 3;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::InnerCurrentD))] = 2;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::InnerCurrentQ))] = 3;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::FilteredActivePower))] = 4;
  inner_vars_.state_local[static_cast<std::size_t>(
      static_cast<int>(InverterInnerVar::FilteredReactivePower))] = 5;
  if (uses_kaura_pll(params_.frequency_estimator)) {
    inner_vars_.state_local[static_cast<std::size_t>(
        static_cast<int>(InverterInnerVar::PllVoltageD))] =
        gfl_vdf_local(params_);
    inner_vars_.state_local[static_cast<std::size_t>(
        static_cast<int>(InverterInnerVar::PllVoltageQ))] =
        gfl_vqf_local(params_);
  }
  const int vdc_local = gfl_vdc_local(params_);
  if (vdc_local >= 0) {
    inner_vars_.state_local[static_cast<std::size_t>(
        static_cast<int>(InverterInnerVar::DCVoltage))] = vdc_local;
  }
}

void GridFollowingInverter::initializeFromPowerFlow(const PowerFlowResult& pf,
                                                   DynamicState& x,
                                                   NetworkState& y) {
  double angle = 0.0;
  if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.va.size())) {
    angle = pf.va[static_cast<std::size_t>(params_.bus_pos)];
  } else if (params_.bus_pos >= 0 && 3 * params_.bus_pos + 2 < y.Vac_abc.size()) {
    angle = std::arg(positive_sequence_voltage(bus_voltage(y, params_.bus_pos)));
  }
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const auto [vd, vq] = dq_from_phasor(positive_sequence_voltage(v), angle);
  const double p = params_.p_ref_mw / safe_base(params_.base_mva);
  const double q = params_.q_ref_mvar / safe_base(params_.base_mva);
  const double denom = std::max(vd * vd + vq * vq,
                                params_.v_min_current_pu * params_.v_min_current_pu);
  const double id_ref = (vd * p + vq * q) / denom;
  const double iq_ref = (vq * p - vd * q) / denom;
  const auto [id, iq] = limited_current(id_ref,
                                        iq_ref,
                                        params_.current_limit_pu,
                                        params_.reactive_current_priority);
  x.x[state_index(range_, 0)] = angle;
  x.x[state_index(range_, 1)] = 0.0;
  x.x[state_index(range_, 2)] = id;
  x.x[state_index(range_, 3)] = iq;
  x.x[state_index(range_, 4)] = p;
  x.x[state_index(range_, 5)] = q;
  if (uses_kaura_pll(params_.frequency_estimator)) {
    x.x[state_index(range_, gfl_vdf_local(params_))] = vd;
    x.x[state_index(range_, gfl_vqf_local(params_))] = vq;
  }
  const int vdc_local = gfl_vdc_local(params_);
  if (vdc_local >= 0) {
    double vdc = params_.vdc_ref_pu;
    if (params_.dc_bus_pos >= 0 && params_.dc_bus_pos < y.Vdc.size()) {
      vdc = y.Vdc[params_.dc_bus_pos];
    }
    x.x[state_index(range_, vdc_local)] =
        clamp_voltage_window(vdc, params_.vdc_min_pu, params_.vdc_max_pu);
  }
}

bool GridFollowingInverter::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service || range_.empty() || params_.bus_pos < 0) return false;
  const Eigen::Vector3cd vabc = bus_voltage(y, params_.bus_pos);
  const Complex vpos = positive_sequence_voltage(vabc);
  const double theta = std::arg(vpos);
  const auto [vd_raw, vq_raw] = dq_from_phasor(vpos, theta);
  bool changed = false;
  if (uses_kaura_pll(params_.frequency_estimator)) {
    changed = set_if_changed(x.x, state_index(range_, gfl_vdf_local(params_)), vd_raw) || changed;
    changed = set_if_changed(x.x, state_index(range_, gfl_vqf_local(params_)), vq_raw) || changed;
  }
  const auto [vd, vq] = pll_measurement(params_, x, range_, vd_raw, vq_raw);
  const double xi_pll = std::abs(params_.pll_ki) > 1e-12
                            ? -params_.pll_kp * vq / params_.pll_ki
                            : 0.0;
  const double freq_error_pu = params_.pll_kp * vq + params_.pll_ki * xi_pll;
  double p_ref = params_.p_ref_mw / safe_base(params_.base_mva);
  double q_ref = params_.q_ref_mvar / safe_base(params_.base_mva);
  if (params_.frequency_watt_droop_pu > 0.0) {
    p_ref -= params_.frequency_watt_droop_pu * freq_error_pu;
  }
  if (params_.volt_var_droop_pu > 0.0) {
    q_ref += params_.volt_var_droop_pu * (params_.v_ref_pu - std::abs(vpos));
  }
  const double denom = std::max(vd * vd + vq * vq,
                                params_.v_min_current_pu * params_.v_min_current_pu);
  const double id_ref = (vd * p_ref + vq * q_ref) / denom;
  const double iq_ref = (vq * p_ref - vd * q_ref) / denom;
  const auto [id, iq] = limited_current(id_ref,
                                        iq_ref,
                                        params_.current_limit_pu,
                                        params_.reactive_current_priority);
  changed = set_if_changed(x.x, state_index(range_, 0), theta) || changed;
  changed = set_if_changed(x.x, state_index(range_, 1), finite_value(xi_pll)) || changed;
  changed = set_if_changed(x.x, state_index(range_, 2), id) || changed;
  changed = set_if_changed(x.x, state_index(range_, 3), iq) || changed;
  changed = set_if_changed(x.x, state_index(range_, 4), p_ref) || changed;
  changed = set_if_changed(x.x, state_index(range_, 5), q_ref) || changed;
  const int vdc_local = gfl_vdc_local(params_);
  if (vdc_local >= 0) {
    double vdc = params_.vdc_ref_pu;
    if (params_.dc_bus_pos >= 0 && params_.dc_bus_pos < y.Vdc.size()) {
      vdc = y.Vdc[params_.dc_bus_pos];
    }
    const Eigen::Vector3cd current = balanced_current_from_dq(id, iq, theta);
    const double p_ac = finite_value(average_complex_power(vabc, current).real(), p_ref);
    vdc = dc_link_voltage_for_power_balance(y,
                                            params_.dc_bus_pos,
                                            p_ac,
                                            params_.dc_link_conductance_pu,
                                            params_.eta,
                                            vdc,
                                            params_.vdc_min_pu,
                                            params_.vdc_max_pu);
    changed = set_if_changed(x.x,
                             state_index(range_, vdc_local),
                             vdc) || changed;
  }
  return changed;
}

void GridFollowingInverter::computeDerivatives(double,
                                               const DynamicState& x,
                                               const NetworkState& y,
                                               Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double theta = x.x[state_index(range_, 0)];
  const double xi_pll = x.x[state_index(range_, 1)];
  const double id = x.x[state_index(range_, 2)];
  const double iq = x.x[state_index(range_, 3)];
  const double pf = x.x[state_index(range_, 4)];
  const double qf = x.x[state_index(range_, 5)];
  const Eigen::Vector3cd vabc = bus_voltage(y, params_.bus_pos);
  const Complex vpos = positive_sequence_voltage(vabc);
  const auto [vd_raw, vq_raw] = dq_from_phasor(vpos, theta);
  const auto [vd, vq] = pll_measurement(params_, x, range_, vd_raw, vq_raw);
  const double tau_i = std::max(kMinTimeConstant, params_.response_t_s);
  const double tau_p = std::max(kMinTimeConstant, params_.power_filter_t_s);
  const double p_nom = params_.p_ref_mw / safe_base(params_.base_mva);
  const double q_nom = params_.q_ref_mvar / safe_base(params_.base_mva);
  const double freq_error_pu =
      params_.frequency_estimator == FrequencyEstimatorKind::FixedFrequency
          ? 0.0
          : params_.pll_kp * vq + params_.pll_ki * xi_pll;
  double p_ref = p_nom;
  double q_ref = q_nom;
  if (params_.frequency_watt_droop_pu > 0.0) {
    p_ref -= params_.frequency_watt_droop_pu * freq_error_pu;
  }
  if (params_.volt_var_droop_pu > 0.0) {
    q_ref += params_.volt_var_droop_pu * (params_.v_ref_pu - std::abs(vpos));
  }
  const double denom = std::max(vd * vd + vq * vq,
                                params_.v_min_current_pu * params_.v_min_current_pu);
  const double id_ref = (vd * p_ref + vq * q_ref) / denom;
  const double iq_ref = (vq * p_ref - vd * q_ref) / denom;
  const auto [id_cmd, iq_cmd] = limited_current(id_ref,
                                                iq_ref,
                                                params_.current_limit_pu,
                                                params_.reactive_current_priority);

  dxdt[state_index(range_, 0)] = kTwoPi * params_.f_ref_hz * freq_error_pu;
  dxdt[state_index(range_, 1)] =
      params_.frequency_estimator == FrequencyEstimatorKind::FixedFrequency ? 0.0 : vq;
  dxdt[state_index(range_, 2)] = (id_cmd - id) / tau_i;
  dxdt[state_index(range_, 3)] = (iq_cmd - iq) / tau_i;
  dxdt[state_index(range_, 4)] = (p_ref - pf) / tau_p;
  dxdt[state_index(range_, 5)] = (q_ref - qf) / tau_p;
  if (uses_kaura_pll(params_.frequency_estimator)) {
    const double tau_pll_filter = std::max(kMinTimeConstant, params_.pll_lpf_t_s);
    dxdt[state_index(range_, gfl_vdf_local(params_))] =
        (vd_raw - x.x[state_index(range_, gfl_vdf_local(params_))]) / tau_pll_filter;
    dxdt[state_index(range_, gfl_vqf_local(params_))] =
        (vq_raw - x.x[state_index(range_, gfl_vqf_local(params_))]) / tau_pll_filter;
  }
  const int vdc_local = gfl_vdc_local(params_);
  if (vdc_local >= 0) {
    const Eigen::Vector3cd current = balanced_current_from_dq(id, iq, theta);
    const double p_ac = finite_value(average_complex_power(vabc, current).real());
    const double vdc_link = x.x[state_index(range_, vdc_local)];
    const double pdc_net =
        dc_link_network_power_pu(y,
                                 params_.dc_bus_pos,
                                 vdc_link,
                                 params_.dc_link_conductance_pu);
    dxdt[state_index(range_, vdc_local)] =
        dc_link_voltage_derivative(p_ac,
                                   pdc_net,
                                   vdc_link,
                                   params_.dc_link_capacitance_s,
                                   params_.eta,
                                   params_.vdc_min_pu,
                                   params_.vdc_max_pu);
  }
}

void GridFollowingInverter::stamp(double,
                                  const DynamicState& x,
                                  const NetworkState& y,
                                  DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const double theta = x.x[state_index(range_, 0)];
  const double id = x.x[state_index(range_, 2)];
  const double iq = x.x[state_index(range_, 3)];
  Eigen::Vector3cd current = balanced_current_from_dq(id, iq, theta);
  add_balanced_current(stamp, params_.bus_pos, current);
  if (params_.stabilizing_admittance_pu > 0.0) {
    add_balanced_admittance(stamp,
                            params_.bus_pos,
                            diagonal_admittance(Complex(params_.stabilizing_admittance_pu, 0.0)));
  }

  if (params_.stamp_dc_power && params_.dc_bus_pos >= 0) {
    const int vdc_local = gfl_vdc_local(params_);
    if (vdc_local >= 0) {
      const double vdc_link =
          clamp_voltage_window(x.x[state_index(range_, vdc_local)],
                               params_.vdc_min_pu,
                               params_.vdc_max_pu);
      const double g = std::max(0.0, params_.dc_link_conductance_pu);
      stamp.addDcConductance(params_.dc_bus_pos, params_.dc_bus_pos, g);
      stamp.addDcCurrent(params_.dc_bus_pos, g * vdc_link);
      return;
    }
    const double p_ac = x.x[state_index(range_, 4)];
    const double p_dc_pu = -p_ac / std::max(1e-6, params_.eta);
    const double vdc = (params_.dc_bus_pos < y.Vdc.size()) ? clamp_voltage(y.Vdc[params_.dc_bus_pos]) : 1.0;
    stamp.addDcCurrent(params_.dc_bus_pos, p_dc_pu / vdc);
  }
}

void GridFollowingInverter::addJacobian(
    double,
    const DynamicState& x,
    const NetworkState& y,
    const DynamicJacobianContext& context,
    std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const int theta_col = state_index(range_, 0);
  const int id_col = state_index(range_, 2);
  const int iq_col = state_index(range_, 3);
  if (context.validStateIndex(theta_col) &&
      context.validStateIndex(id_col) &&
      context.validStateIndex(iq_col)) {
    const double theta = x.x[theta_col];
    const double id = x.x[id_col];
    const double iq = x.x[iq_col];
    const Eigen::Vector3cd current = balanced_current_from_dq(id, iq, theta);
    const Eigen::Vector3cd d_i_dtheta = Complex(0.0, 1.0) * current;
    const Eigen::Vector3cd d_i_did = balanced_current_from_dq(1.0, 0.0, theta);
    const Eigen::Vector3cd d_i_diq = balanced_current_from_dq(0.0, 1.0, theta);
    add_balanced_current_derivative(context, params_.bus_pos, theta_col, d_i_dtheta, triplets);
    add_balanced_current_derivative(context, params_.bus_pos, id_col, d_i_did, triplets);
    add_balanced_current_derivative(context, params_.bus_pos, iq_col, d_i_diq, triplets);
  }

  if (!params_.stamp_dc_power ||
      params_.dc_bus_pos < 0 ||
      !context.validDcNode(params_.dc_bus_pos)) {
    return;
  }
  const int vdc_local = gfl_vdc_local(params_);
  if (vdc_local >= 0) {
    const int vdc_col = state_index(range_, vdc_local);
    if (context.validStateIndex(vdc_col)) {
      add_dc_current_derivative(context,
                                params_.dc_bus_pos,
                                vdc_col,
                                std::max(0.0, params_.dc_link_conductance_pu),
                                triplets);
    }
    return;
  }
  const int p_col = state_index(range_, 4);
  if (!context.validStateIndex(p_col) ||
      params_.dc_bus_pos >= y.Vdc.size()) {
    return;
  }
  const double eta = std::max(1e-6, params_.eta);
  const double raw_vdc = y.Vdc[params_.dc_bus_pos];
  const double vdc = clamp_voltage(raw_vdc);
  add_dc_current_derivative(context, params_.dc_bus_pos, p_col, -1.0 / (eta * vdc), triplets);
  add_dc_current_derivative(context,
                            params_.dc_bus_pos,
                            context.dcCol(params_.dc_bus_pos),
                            x.x[p_col] * clamp_voltage_slope(raw_vdc) / (eta * vdc * vdc),
                            triplets);
}

void GridFollowingInverter::handleEvent(const DynamicEvent& event,
                                        DynamicState& x,
                                        NetworkState&) {
  if (event.type == DynamicEventType::VSCTrip &&
      (event.component_index == 0 || event.component_index == params_.component_index)) {
    params_.in_service = false;
    if (!range_.empty() && state_index(range_, 5) < x.x.size()) {
      x.x.segment(range_.offset, range_.size).setZero();
    }
  } else if (event.type == DynamicEventType::Custom &&
             event.component_type == "VSC" &&
             (event.component_index == 0 || event.component_index == params_.component_index)) {
    const auto p_it = event.params.find("p_ref_mw");
    if (p_it != event.params.end()) params_.p_ref_mw = p_it->second;
    const auto q_it = event.params.find("q_ref_mvar");
    if (q_it != event.params.end()) params_.q_ref_mvar = q_it->second;
    const auto v_it = event.params.find("v_ref_pu");
    if (v_it != event.params.end()) params_.v_ref_pu = v_it->second;
  }
}

std::string GridFollowingInverter::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> GridFollowingInverter::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput GridFollowingInverter::output(const DynamicState& x,
                                                  const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["control_mode"] = 0.0;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  InverterInnerVariableSnapshot inner;
  if (!range_.empty() && state_index(range_, 5) < x.x.size()) {
    const double theta = x.x[state_index(range_, 0)];
    const double xi_pll = x.x[state_index(range_, 1)];
    const double id = x.x[state_index(range_, 2)];
    const double iq = x.x[state_index(range_, 3)];
    const Eigen::Vector3cd vabc = bus_voltage(y, params_.bus_pos);
    const Complex vpos = positive_sequence_voltage(vabc);
    const auto [vd_raw, vq_raw] = dq_from_phasor(vpos, theta);
    const auto [vd, vq] = pll_measurement(params_, x, range_, vd_raw, vq_raw);
    const Eigen::Vector3cd current = balanced_current_from_dq(id, iq, theta);
    const Complex s = average_complex_power(vabc, current);
    const double freq_error_pu =
        params_.frequency_estimator == FrequencyEstimatorKind::FixedFrequency
            ? 0.0
            : params_.pll_kp * vq + params_.pll_ki * xi_pll;
    const double denom = std::max(vd * vd + vq * vq,
                                  params_.v_min_current_pu * params_.v_min_current_pu);
    const double p_nom = params_.p_ref_mw / safe_base(params_.base_mva);
    const double q_nom = params_.q_ref_mvar / safe_base(params_.base_mva);
    const double id_ref = (vd * p_nom + vq * q_nom) / denom;
    const double iq_ref = (vq * p_nom - vd * q_nom) / denom;
    const auto [id_cmd, iq_cmd] = limited_current(id_ref,
                                                  iq_ref,
                                                  params_.current_limit_pu,
                                                  params_.reactive_current_priority);
    const double i_ref_mag = std::hypot(id_ref, iq_ref);
    out.values["pll_angle_rad"] = theta;
    out.values["pll_frequency_hz"] = params_.f_ref_hz * (1.0 + freq_error_pu);
    out.values["pll_vq_pu"] = vq;
    out.values["pll_vd_pu"] = vd;
    out.values["pll_vq_raw_pu"] = vq_raw;
    out.values["pll_vd_raw_pu"] = vd_raw;
    out.values["pll_model"] =
        params_.frequency_estimator == FrequencyEstimatorKind::KauraPLL
            ? 1.0
            : (params_.frequency_estimator == FrequencyEstimatorKind::FixedFrequency ? 2.0 : 0.0);
    out.values["pll_integrator"] = xi_pll;
    out.values["id_pu"] = id;
    out.values["iq_pu"] = iq;
    out.values["i_mag_pu"] = std::hypot(id, iq);
    out.values["i_ref_mag_pu"] = i_ref_mag;
    out.values["current_limit_active"] =
        (params_.current_limit_pu > 0.0 && i_ref_mag > params_.current_limit_pu) ? 1.0 : 0.0;
    out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
    out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
    out.values["p_ref_mw"] = params_.p_ref_mw;
    out.values["q_ref_mvar"] = params_.q_ref_mvar;
    out.values["p_filtered_mw"] = x.x[state_index(range_, 4)] * safe_base(params_.base_mva);
    out.values["q_filtered_mvar"] = x.x[state_index(range_, 5)] * safe_base(params_.base_mva);
    const Complex i_pos = phasor_from_dq(id, iq, theta);
    inner.set(InverterInnerVar::PllAngle, theta);
    inner.set(InverterInnerVar::PllOmega, 1.0 + freq_error_pu);
    inner.set(InverterInnerVar::FilterVoltageReal, vpos.real());
    inner.set(InverterInnerVar::FilterVoltageImag, vpos.imag());
    inner.set(InverterInnerVar::PllVoltageD, vd);
    inner.set(InverterInnerVar::PllVoltageQ, vq);
    inner.set(InverterInnerVar::CurrentReferenceD, id_ref);
    inner.set(InverterInnerVar::CurrentReferenceQ, iq_ref);
    inner.set(InverterInnerVar::OuterCurrentD, id_cmd);
    inner.set(InverterInnerVar::OuterCurrentQ, iq_cmd);
    inner.set(InverterInnerVar::InnerCurrentD, id);
    inner.set(InverterInnerVar::InnerCurrentQ, iq);
    inner.set(InverterInnerVar::ConverterCurrentReal, i_pos.real());
    inner.set(InverterInnerVar::ConverterCurrentImag, i_pos.imag());
    inner.set(InverterInnerVar::FilterCurrentReal, i_pos.real());
    inner.set(InverterInnerVar::FilterCurrentImag, i_pos.imag());
    inner.set(InverterInnerVar::FilteredActivePower, x.x[state_index(range_, 4)]);
    inner.set(InverterInnerVar::FilteredReactivePower, x.x[state_index(range_, 5)]);
    inner.set(InverterInnerVar::VoltageReference, params_.v_ref_pu);
    inner.set(InverterInnerVar::FrequencyReference, params_.f_ref_hz);
    const int vdc_local = gfl_vdc_local(params_);
    if (vdc_local >= 0 && state_index(range_, vdc_local) < x.x.size()) {
      const double vdc_link = x.x[state_index(range_, vdc_local)];
      out.values["vdc_link_pu"] = vdc_link;
      const double pdc_net =
          dc_link_network_power_pu(y,
                                   params_.dc_bus_pos,
                                   vdc_link,
                                   params_.dc_link_conductance_pu);
      out.values["p_dc_mw"] = pdc_net * safe_base(params_.base_mva);
      out.values["dc_link_dynamic"] = 1.0;
      inner.set(InverterInnerVar::DCVoltage, vdc_link);
    } else {
      out.values["dc_link_dynamic"] = 0.0;
    }
  }
  add_inverter_block_flags(out, inner_vars_);
  add_inverter_inner_outputs(out, inner);
  add_voltage_metrics(out, y, params_.bus_pos, params_.dc_bus_pos);
  return out;
}

namespace {

GridFormingInverterParams make_gfm_params(const VSCConverterDynamicParams& params) {
  GridFormingInverterParams gfm;
  gfm.component_index = params.component_index;
  gfm.bus = params.bus;
  gfm.bus_pos = params.bus_pos;
  gfm.dc_bus_pos = params.dc_bus_pos;
  gfm.label = params.label;
  gfm.device_type = "VSCGridForming";
  gfm.canvas_type = params.canvas_type;
  gfm.component_domain = params.component_domain;
  gfm.source_type = "vsc_grid_forming";
  gfm.model_name = params.model_name;
  gfm.model_profiles = params.model_profiles;
  gfm.control_kind = params.control_kind;
  gfm.base_mva = params.base_mva;
  gfm.p_ref_mw = params.p_ref_mw;
  gfm.q_ref_mvar = params.q_ref_mvar;
  gfm.v_ref_pu = params.v_ref_pu;
  gfm.angle_ref_rad = params.angle_ref_rad;
  gfm.frequency_hz = params.f_ref_hz;
  gfm.virtual_r_pu = params.virtual_r_pu;
  gfm.virtual_x_pu = params.virtual_x_pu;
  gfm.p_droop_pu = params.p_droop_pu;
  gfm.q_droop_pu = params.q_droop_pu;
  gfm.power_filter_t_s = params.power_filter_t_s;
  gfm.reactive_power_filter_t_s = params.reactive_power_filter_t_s;
  gfm.voltage_control_t_s = params.voltage_control_t_s;
  gfm.voltage_kp = params.voltage_kp;
  gfm.voltage_ki = params.voltage_ki;
  gfm.overload_kp = params.overload_kp;
  gfm.overload_ki = params.overload_ki;
  gfm.current_limit_pu = params.current_limit_pu;
  gfm.pmax_mw = params.pmax_mw;
  gfm.pmin_mw = params.pmin_mw;
  gfm.vmax_internal_pu = params.vmax_internal_pu;
  gfm.vmin_internal_pu = params.vmin_internal_pu;
  gfm.eta = params.eta;
  gfm.dc_link_capacitance_s = params.dc_link_capacitance_s;
  gfm.dc_link_conductance_pu = params.dc_link_conductance_pu;
  gfm.vdc_ref_pu = params.vdc_ref_pu;
  gfm.vdc_min_pu = params.vdc_min_pu;
  gfm.vdc_max_pu = params.vdc_max_pu;
  gfm.vsm_ta_s = params.vsm_ta_s;
  gfm.vsm_damping_kd = params.vsm_damping_kd;
  gfm.vsm_frequency_droop_kw = params.vsm_frequency_droop_kw;
  gfm.voc_k1 = params.voc_k1;
  gfm.voc_psi_rad = params.voc_psi_rad;
  gfm.voc_k2 = params.voc_k2;
  gfm.reference_frame_locked = params.reference_frame_locked;
  gfm.dc_link_mode = params.dc_link_mode;
  gfm.in_service = params.in_service;
  return gfm;
}

}  // namespace

VSCConverterDynamic::VSCConverterDynamic(VSCConverterDynamicParams params)
    : params_(std::move(params)),
      gfm_(make_gfm_params(params_)),
      gfl_(GridFollowingInverterParams(params_)) {}

void VSCConverterDynamic::assignStateIndices(int& offset) {
  if (params_.grid_forming) {
    gfm_.assignStateIndices(offset);
  } else {
    gfl_.assignStateIndices(offset);
  }
}

void VSCConverterDynamic::initializeFromPowerFlow(const PowerFlowResult& pf,
                                                 DynamicState& x,
                                                 NetworkState& y) {
  if (params_.grid_forming) {
    gfm_.initializeFromPowerFlow(pf, x, y);
  } else {
    gfl_.initializeFromPowerFlow(pf, x, y);
  }
}

bool VSCConverterDynamic::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  return params_.grid_forming ? gfm_.trimToNetworkEquilibrium(x, y)
                              : gfl_.trimToNetworkEquilibrium(x, y);
}

void VSCConverterDynamic::computeDerivatives(double t,
                                            const DynamicState& x,
                                            const NetworkState& y,
                                            Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (params_.grid_forming) {
    gfm_.computeDerivatives(t, x, y, dxdt);
  } else {
    gfl_.computeDerivatives(t, x, y, dxdt);
  }
}

void VSCConverterDynamic::maskSlowStateResidual(Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.grid_forming) gfl_.maskSlowStateResidual(dxdt);
}

void VSCConverterDynamic::stamp(double t,
                                const DynamicState& x,
                                const NetworkState& y,
                                DynamicStamp& stamp) const {
  if (params_.grid_forming) {
    gfm_.stamp(t, x, y, stamp);
  } else {
    gfl_.stamp(t, x, y, stamp);
  }
}

void VSCConverterDynamic::addJacobian(
    double t,
    const DynamicState& x,
    const NetworkState& y,
    const DynamicJacobianContext& context,
    std::vector<Eigen::Triplet<double>>& triplets) const {
  if (params_.grid_forming) {
    gfm_.addJacobian(t, x, y, context, triplets);
  } else {
    gfl_.addJacobian(t, x, y, context, triplets);
  }
}

void VSCConverterDynamic::handleEvent(const DynamicEvent& event,
                                      DynamicState& x,
                                      NetworkState& y) {
  if (params_.grid_forming) {
    gfm_.handleEvent(event, x, y);
  } else {
    gfl_.handleEvent(event, x, y);
  }
}

DynamicDeviceOutput VSCConverterDynamic::output(const DynamicState& x,
                                                const NetworkState& y) const {
  DynamicDeviceOutput out = params_.grid_forming ? gfm_.output(x, y) : gfl_.output(x, y);
  out.type = type();
  out.model_standard = modelStandard();
  out.model_name = modelName();
  out.model_profiles = modelProfiles();
  return out;
}

std::string VSCConverterDynamic::name() const {
  return params_.label.empty() ? type() + " " + std::to_string(params_.component_index)
                               : params_.label;
}

PeriodicVariableSourceDynamic::PeriodicVariableSourceDynamic(
    PeriodicVariableSourceDynamicParams params)
    : params_(std::move(params)) {}

void PeriodicVariableSourceDynamic::assignStateIndices(int& offset) {
  range_ = {offset, 2};
  offset += range_.size;
}

void PeriodicVariableSourceDynamic::initializeFromPowerFlow(const PowerFlowResult&,
                                                           DynamicState& x,
                                                           NetworkState& y) {
  double angle = params_.angle_bias_rad + params_.angle_cos_coeff_rad;
  double voltage = params_.voltage_bias_pu + params_.voltage_cos_coeff_pu;
  if (params_.bus_pos >= 0 && 3 * params_.bus_pos < y.Vac_abc.size()) {
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    voltage = std::abs(v);
    angle = std::arg(v);
  }
  params_.initial_voltage_pu = voltage;
  params_.initial_angle_rad = angle;
  x.x[state_index(range_, 0)] = std::max(kMinVoltage, voltage);
  x.x[state_index(range_, 1)] = angle;
}

namespace {

double periodic_prescribed_value(double initial,
                                 double t,
                                 double frequency,
                                 double sin_coeff,
                                 double cos_coeff) {
  return initial + sin_coeff * std::sin(frequency * t) +
         cos_coeff * (std::cos(frequency * t) - 1.0);
}

}  // namespace

void PeriodicVariableSourceDynamic::computeDerivatives(double t,
                                                       const DynamicState&,
                                                       const NetworkState&,
                                                       Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double wv = params_.voltage_frequency_rad_s;
  const double wa = params_.angle_frequency_rad_s;
  dxdt[state_index(range_, 0)] =
      t <= 0.0
          ? 0.0
          : wv * (params_.voltage_sin_coeff_pu * std::cos(wv * t) -
                  params_.voltage_cos_coeff_pu * std::sin(wv * t));
  dxdt[state_index(range_, 1)] =
      t <= 0.0
          ? 0.0
          : wa * (params_.angle_sin_coeff_rad * std::cos(wa * t) -
                  params_.angle_cos_coeff_rad * std::sin(wa * t));
}

void PeriodicVariableSourceDynamic::stamp(double,
                                          const DynamicState& x,
                                          const NetworkState&,
                                          DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const Complex z(params_.r_th_pu, std::max(1e-5, params_.x_th_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const double vt = periodic_prescribed_value(params_.initial_voltage_pu,
                                              x.time_s,
                                              params_.voltage_frequency_rad_s,
                                              params_.voltage_sin_coeff_pu,
                                              params_.voltage_cos_coeff_pu);
  const double theta =
      periodic_prescribed_value(params_.initial_angle_rad,
                                x.time_s,
                                params_.angle_frequency_rad_s,
                                params_.angle_sin_coeff_rad,
                                params_.angle_cos_coeff_rad) +
      (vt < 0.0 ? kPi : 0.0);
  const Eigen::Vector3cd e =
      balanced_phasors(std::abs(vt), theta);
  add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
  add_balanced_current(stamp, params_.bus_pos, yv * e);
}

std::string PeriodicVariableSourceDynamic::name() const {
  return params_.label.empty() ? "PeriodicVariableSource " +
                                     std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> PeriodicVariableSourceDynamic::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput PeriodicVariableSourceDynamic::output(const DynamicState& x,
                                                          const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  if (!range_.empty()) {
    const double vt = periodic_prescribed_value(params_.initial_voltage_pu,
                                                x.time_s,
                                                params_.voltage_frequency_rad_s,
                                                params_.voltage_sin_coeff_pu,
                                                params_.voltage_cos_coeff_pu);
    const double theta = periodic_prescribed_value(params_.initial_angle_rad,
                                                   x.time_s,
                                                   params_.angle_frequency_rad_s,
                                                   params_.angle_sin_coeff_rad,
                                                   params_.angle_cos_coeff_rad);
    const Complex z(params_.r_th_pu, std::max(1e-5, params_.x_th_pu));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Complex e = std::polar(std::abs(vt), theta + (vt < 0.0 ? kPi : 0.0));
    const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
    const Complex i = yv * (e - v);
    out.values["Vt"] = vt;
    out.values["theta_rad"] = theta;
    out.values["p_mw"] = (v * std::conj(i)).real() * safe_base(params_.base_mva);
    out.values["q_mvar"] = (v * std::conj(i)).imag() * safe_base(params_.base_mva);
    out.values["current_real_pu"] = i.real();
    out.values["current_imag_pu"] = i.imag();
  }
  add_voltage_metrics(out, y, params_.bus_pos, -1);
  return out;
}

CSVGN1Dynamic::CSVGN1Dynamic(CSVGN1DynamicParams params)
    : params_(std::move(params)) {}

void CSVGN1Dynamic::assignStateIndices(int& offset) {
  range_ = {offset, 3};
  offset += range_.size;
}

void CSVGN1Dynamic::initializeFromPowerFlow(const PowerFlowResult&,
                                            DynamicState& x,
                                            NetworkState& y) {
  if (range_.empty()) return;
  const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
  const double vm = std::max(kMinVoltage, std::abs(v));
  const double q_pu = params_.q_ref_mvar / safe_base(params_.base_mva);
  const double yq = q_pu / (vm * vm);
  if (params_.v_ref_pu <= 0.0 && params_.K > 1e-9) {
    params_.v_ref_pu =
        vm - (params_.CBase / safe_base(params_.base_mva) - yq) *
                 safe_base(params_.base_mva) /
                 std::max(1e-9, params_.K * params_.model_base_mva);
  }
  const double thy =
      params_.K * (vm - params_.v_ref_pu);
  x.x[state_index(range_, 0)] = thy;
  x.x[state_index(range_, 1)] = 0.0;
  x.x[state_index(range_, 2)] = thy;
}

bool CSVGN1Dynamic::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  params_.v_ref_pu = 0.0;
  initializeFromPowerFlow(PowerFlowResult{}, x, y);
  return true;
}

void CSVGN1Dynamic::computeDerivatives(double,
                                       const DynamicState& x,
                                       const NetworkState& y,
                                       Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double vm =
      std::max(kMinVoltage, std::abs(positive_sequence_voltage(bus_voltage(y, params_.bus_pos))));
  const double input = params_.K * (vm - params_.v_ref_pu);
  const double vr1 = x.x[state_index(range_, 1)];
  const double vr2 = x.x[state_index(range_, 2)];
  const SecondOrderLeadLagNonWindup regulator =
      lead_lag_2nd_nonwindup(input,
                             vr1,
                             vr2,
                             params_.T3 + params_.T4,
                             params_.T3 * params_.T4,
                             params_.T1 + params_.T2,
                             params_.T1 * params_.T2,
                             params_.Vmin,
                             params_.Vmax);
  const auto [thy_sat, dthy] =
      low_pass_nonwindup(regulator.output,
                         x.x[state_index(range_, 0)],
                         1.0,
                         params_.T5,
                         params_.Rmin / std::max(1e-9, params_.model_base_mva),
                         1.0);
  (void)thy_sat;
  dxdt[state_index(range_, 0)] = dthy;
  dxdt[state_index(range_, 1)] = regulator.dx1;
  dxdt[state_index(range_, 2)] = regulator.dx2;
}

double CSVGN1Dynamic::susceptancePu(const DynamicState& x) const {
  const double thy = range_.empty() ? 0.0 : x.x[state_index(range_, 0)];
  return params_.CBase / safe_base(params_.base_mva) -
         thy * params_.model_base_mva / safe_base(params_.base_mva);
}

void CSVGN1Dynamic::stamp(double,
                          const DynamicState& x,
                          const NetworkState& y,
                          DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const double b = susceptancePu(x);
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  Eigen::Vector3cd current;
  for (int phase = 0; phase < 3; ++phase) current[phase] = Complex(0.0, -b) * v[phase];
  add_balanced_current(stamp, params_.bus_pos, current);
}

void CSVGN1Dynamic::addJacobian(double,
                                const DynamicState& x,
                                const NetworkState& y,
                                const DynamicJacobianContext& context,
                                std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const int thy_col = state_index(range_, 0);
  const double b = susceptancePu(x);
  const double db_dthy = -params_.model_base_mva / safe_base(params_.base_mva);
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const Complex current_per_voltage(0.0, -b);
  for (int phase = 0; phase < 3; ++phase) {
    const int node = 3 * params_.bus_pos + phase;
    if (context.validStateIndex(thy_col)) {
      add_ac_current_derivative(context,
                                node,
                                thy_col,
                                Complex(0.0, -db_dthy) * v[phase],
                                triplets);
    }
    add_ac_current_voltage_derivative(context,
                                      node,
                                      current_per_voltage,
                                      current_per_voltage * Complex(0.0, 1.0),
                                      triplets);
  }
}

std::string CSVGN1Dynamic::name() const {
  return params_.label.empty() ? "CSVGN1 " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> CSVGN1Dynamic::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput CSVGN1Dynamic::output(const DynamicState& x,
                                          const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  const double b = susceptancePu(x);
  const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
  const Complex i = Complex(0.0, -b) * v;
  out.values["thyristor_state"] = range_.empty() ? 0.0 : x.x[state_index(range_, 0)];
  out.values["vr1"] = range_.empty() ? 0.0 : x.x[state_index(range_, 1)];
  out.values["vr2"] = range_.empty() ? 0.0 : x.x[state_index(range_, 2)];
  out.values["b_pu"] = b;
  out.values["q_mvar"] = (v * std::conj(i)).imag() * safe_base(params_.base_mva);
  out.values["v_ref_pu"] = params_.v_ref_pu;
  add_voltage_metrics(out, y, params_.bus_pos, -1);
  return out;
}

DERAADynamic::DERAADynamic(DERAADynamicParams params)
    : params_(std::move(params)) {}

void DERAADynamic::assignStateIndices(int& offset) {
  range_ = {offset, params_.freq_flag == 1 ? 10 : 7};
  offset += range_.size;
}

void DERAADynamic::initializeFromPowerFlow(const PowerFlowResult&,
                                           DynamicState& x,
                                           NetworkState& y) {
  const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
  const double vm = std::max(kMinVoltage, std::abs(v));
  const double theta = std::arg(v);
  const Complex s(params_.p_ref_mw / safe_base(params_.base_mva),
                  params_.q_ref_mvar / safe_base(params_.base_mva));
  const Complex i = std::conj(s / std::max(kMinVoltage, vm)) *
                    std::polar(1.0, theta);
  const Complex idq = i * std::polar(1.0, -theta);
  const double ip = idq.real() * safe_base(params_.base_mva) /
                    std::max(1e-9, params_.model_base_mva);
  const double iq = -idq.imag() * safe_base(params_.base_mva) /
                    std::max(1e-9, params_.model_base_mva);
  x.x[state_index(range_, 0)] = vm;
  x.x[state_index(range_, 1)] = params_.p_ref_mw / std::max(1e-9, params_.model_base_mva);
  x.x[state_index(range_, 2)] =
      params_.pf_flag == 1
          ? std::tan(params_.pf_angle_ref_rad) * x.x[state_index(range_, 1)] / vm
          : params_.q_ref_mvar / std::max(1e-9, params_.model_base_mva) / vm;
  x.x[state_index(range_, 3)] = iq;
  x.x[state_index(range_, 4)] = 1.0;
  x.x[state_index(range_, 5)] = 1.0;
  if (params_.freq_flag == 1) {
    x.x[state_index(range_, 6)] = 0.0;
    x.x[state_index(range_, 7)] = 0.0;
    x.x[state_index(range_, 8)] = x.x[state_index(range_, 1)];
    x.x[state_index(range_, 9)] = ip;
  } else {
    x.x[state_index(range_, 6)] = ip;
  }
}

namespace {

double dera_deadband(double value, double low, double high) {
  if (value > high) return value - high;
  if (value < low) return value - low;
  return 0.0;
}

}  // namespace

void DERAADynamic::computeDerivatives(double,
                                      const DynamicState& x,
                                      const NetworkState& y,
                                      Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double vm =
      std::max(kMinVoltage, std::abs(positive_sequence_voltage(bus_voltage(y, params_.bus_pos))));
  const double vmeas = x.x[state_index(range_, 0)];
  const double pmeas = x.x[state_index(range_, 1)];
  const double qv = x.x[state_index(range_, 2)];
  const double iq = x.x[state_index(range_, 3)];
  const double mult = x.x[state_index(range_, 4)];
  const double fmeas = x.x[state_index(range_, 5)];
  const double ip_index = params_.freq_flag == 1 ? 9 : 6;
  const double ip = x.x[state_index(range_, ip_index)];
  const double p_ref = params_.p_ref_mw / std::max(1e-9, params_.model_base_mva);
  const double q_ref = params_.q_ref_mvar / std::max(1e-9, params_.model_base_mva);
  dxdt[state_index(range_, 0)] = (vm - vmeas) / std::max(kMinTimeConstant, params_.T_rv);
  dxdt[state_index(range_, 1)] = (p_ref - pmeas) / std::max(kMinTimeConstant, params_.Tp);
  const double qv_ref =
      params_.pf_flag == 1
          ? std::tan(params_.pf_angle_ref_rad) * pmeas / std::max(kMinVoltage, vmeas)
          : q_ref / std::max(kMinVoltage, vmeas);
  dxdt[state_index(range_, 2)] = (qv_ref - qv) / std::max(kMinTimeConstant, params_.T_iq);
  const double iq_cmd =
      std::clamp(dera_deadband(params_.v_ref_pu - vmeas, params_.dbd1, params_.dbd2) *
                         params_.K_qv +
                     qv,
                 params_.Iq_min,
                 params_.Iq_max) *
      mult;
  dxdt[state_index(range_, 3)] = (iq_cmd - iq) / std::max(kMinTimeConstant, params_.Tg);
  dxdt[state_index(range_, 4)] = (1.0 - mult) / std::max(kMinTimeConstant, params_.Tv);
  dxdt[state_index(range_, 5)] = (1.0 - fmeas) / std::max(kMinTimeConstant, params_.Trf);
  double ip_ref = std::clamp(p_ref / std::max(kMinVoltage, vmeas),
                             params_.Ip_min,
                             params_.Ip_max) *
                  mult;
  if (params_.freq_flag == 1) {
    const double power_pi = x.x[state_index(range_, 6)];
    const double dpord = x.x[state_index(range_, 7)];
    const double pord = x.x[state_index(range_, 8)];
    const double freq_error = 1.0 - fmeas;
    dxdt[state_index(range_, 6)] = params_.Kig * freq_error;
    const double pord_cmd =
        std::clamp(p_ref + params_.Kpg * freq_error + power_pi,
                   params_.Ip_min,
                   params_.Ip_max);
    dxdt[state_index(range_, 7)] = (pord_cmd - dpord) / std::max(kMinTimeConstant, params_.Tpord);
    dxdt[state_index(range_, 8)] = dpord;
    ip_ref = std::clamp(pord / std::max(kMinVoltage, vmeas),
                        params_.Ip_min,
                        params_.Ip_max) *
             mult;
  }
  dxdt[state_index(range_, ip_index)] =
      (ip_ref - ip) / std::max(kMinTimeConstant, params_.Tg);
}

std::pair<double, double> DERAADynamic::currentDq(const DynamicState& x) const {
  if (range_.empty()) return {0.0, 0.0};
  const double ip = x.x[state_index(range_, params_.freq_flag == 1 ? 9 : 6)];
  const double iq = x.x[state_index(range_, 3)];
  const double ratio = params_.model_base_mva / safe_base(params_.base_mva);
  return {ip * ratio, iq * ratio};
}

void DERAADynamic::stamp(double,
                         const DynamicState& x,
                         const NetworkState& y,
                         DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
  const double theta = std::arg(v);
  const auto [ip, iq] = currentDq(x);
  add_balanced_current(stamp,
                       params_.bus_pos,
                       balanced_current_from_dq(ip, -iq, theta));
}

void DERAADynamic::addJacobian(double,
                               const DynamicState& x,
                               const NetworkState& y,
                               const DynamicJacobianContext& context,
                               std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
  const double theta = std::arg(v);
  const double ratio = params_.model_base_mva / safe_base(params_.base_mva);
  const int ip_col = state_index(range_, params_.freq_flag == 1 ? 9 : 6);
  const int iq_col = state_index(range_, 3);
  if (context.validStateIndex(ip_col)) {
    add_balanced_current_derivative(context,
                                    params_.bus_pos,
                                    ip_col,
                                    balanced_current_from_dq(ratio, 0.0, theta),
                                    triplets);
  }
  if (context.validStateIndex(iq_col)) {
    add_balanced_current_derivative(context,
                                    params_.bus_pos,
                                    iq_col,
                                    balanced_current_from_dq(0.0, -ratio, theta),
                                    triplets);
  }
}

std::string DERAADynamic::name() const {
  return params_.label.empty() ? "AggregateDistributedGenerationA " +
                                     std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> DERAADynamic::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput DERAADynamic::output(const DynamicState& x,
                                         const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  const Complex v = positive_sequence_voltage(bus_voltage(y, params_.bus_pos));
  const double theta = std::arg(v);
  const auto [ip, iq] = currentDq(x);
  const Complex i = phasor_from_dq(ip, -iq, theta);
  const Complex s = v * std::conj(i);
  out.values["Vmeas"] = range_.empty() ? 0.0 : x.x[state_index(range_, 0)];
  out.values["Pmeas"] = range_.empty() ? 0.0 : x.x[state_index(range_, 1)];
  out.values["Q_V"] = range_.empty() ? 0.0 : x.x[state_index(range_, 2)];
  out.values["Iq"] = range_.empty() ? 0.0 : x.x[state_index(range_, 3)];
  out.values["Mult"] = range_.empty() ? 0.0 : x.x[state_index(range_, 4)];
  out.values["Fmeas"] = range_.empty() ? 0.0 : x.x[state_index(range_, 5)];
  out.values["Ip"] = range_.empty() ? 0.0 : x.x[state_index(range_, params_.freq_flag == 1 ? 9 : 6)];
  out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
  out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
  out.values["freq_flag"] = static_cast<double>(params_.freq_flag);
  add_voltage_metrics(out, y, params_.bus_pos, -1);
  return out;
}

DCDCConverterDynamic::DCDCConverterDynamic(DCDCConverterDynamicParams params)
    : params_(std::move(params)) {}

void DCDCConverterDynamic::assignStateIndices(int& offset) {
  range_ = {offset, 1};
  offset += range_.size;
}

void DCDCConverterDynamic::initializeFromPowerFlow(const PowerFlowResult&,
                                                  DynamicState& x,
                                                  NetworkState&) {
  x.x[range_.offset] = params_.p_ref_mw / std::max(1.0, params_.base_mva);
}

bool DCDCConverterDynamic::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  return set_if_changed(x.x,
                        range_.offset,
                        params_.p_ref_mw / std::max(1.0, params_.base_mva));
}

void DCDCConverterDynamic::computeDerivatives(double,
                                              const DynamicState& x,
                                              const NetworkState&,
                                              Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double tau = std::max(1e-4, params_.response_t_s);
  dxdt[range_.offset] =
      (params_.p_ref_mw / std::max(1.0, params_.base_mva) - x.x[range_.offset]) / tau;
}

void DCDCConverterDynamic::stamp(double,
                                 const DynamicState& x,
                                 const NetworkState& y,
                                 DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_in_pos < 0 || params_.bus_out_pos < 0 ||
      range_.empty()) {
    return;
  }
  const double p_out = x.x[range_.offset];
  const double v_in = clamp_voltage(y.Vdc[params_.bus_in_pos]);
  const double v_out = clamp_voltage(y.Vdc[params_.bus_out_pos]);
  stamp.addDcCurrent(params_.bus_in_pos, -p_out / std::max(1e-6, params_.eta) / v_in);
  stamp.addDcCurrent(params_.bus_out_pos, p_out / v_out);
}

void DCDCConverterDynamic::addJacobian(
    double,
    const DynamicState& x,
    const NetworkState& y,
    const DynamicJacobianContext& context,
    std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_in_pos < 0 || params_.bus_out_pos < 0 ||
      range_.empty() ||
      params_.bus_in_pos >= y.Vdc.size() ||
      params_.bus_out_pos >= y.Vdc.size()) {
    return;
  }
  const int p_col = range_.offset;
  if (!context.validStateIndex(p_col)) return;
  const double eta = std::max(1e-6, params_.eta);
  const double p_out = x.x[p_col];
  const double raw_v_in = y.Vdc[params_.bus_in_pos];
  const double raw_v_out = y.Vdc[params_.bus_out_pos];
  const double v_in = clamp_voltage(raw_v_in);
  const double v_out = clamp_voltage(raw_v_out);
  add_dc_current_derivative(context, params_.bus_in_pos, p_col, -1.0 / (eta * v_in), triplets);
  add_dc_current_derivative(context, params_.bus_out_pos, p_col, 1.0 / v_out, triplets);
  add_dc_current_derivative(context,
                            params_.bus_in_pos,
                            context.dcCol(params_.bus_in_pos),
                            p_out * clamp_voltage_slope(raw_v_in) / (eta * v_in * v_in),
                            triplets);
  add_dc_current_derivative(context,
                            params_.bus_out_pos,
                            context.dcCol(params_.bus_out_pos),
                            -p_out * clamp_voltage_slope(raw_v_out) / (v_out * v_out),
                            triplets);
}

void DCDCConverterDynamic::handleEvent(const DynamicEvent& event, DynamicState&, NetworkState&) {
  if (event.type == DynamicEventType::DCDCTrip &&
      (event.component_index == 0 || event.component_index == params_.component_index)) {
    params_.in_service = false;
  }
}

std::string DCDCConverterDynamic::name() const {
  return params_.label.empty() ? "DC/DC converter " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> DCDCConverterDynamic::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput DCDCConverterDynamic::output(const DynamicState& x,
                                                 const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus_out,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  if (!range_.empty() && range_.offset < x.x.size()) {
    out.values["p_out_mw"] = x.x[range_.offset] * safe_base(params_.base_mva);
    out.values["p_in_mw"] =
        out.values["p_out_mw"] / std::max(1e-6, params_.eta);
  }
  out.values["eta"] = params_.eta;
  if (params_.bus_in_pos >= 0 && params_.bus_in_pos < y.Vdc.size()) {
    out.values["vdc_in_pu"] = y.Vdc[params_.bus_in_pos];
  }
  if (params_.bus_out_pos >= 0 && params_.bus_out_pos < y.Vdc.size()) {
    out.values["vdc_out_pu"] = y.Vdc[params_.bus_out_pos];
  }
  return out;
}

BatteryDynamic::BatteryDynamic(BatteryDynamicParams params) : params_(std::move(params)) {}

void BatteryDynamic::assignStateIndices(int& offset) {
  range_ = {offset, 2};
  offset += range_.size;
}

void BatteryDynamic::initializeFromPowerFlow(const PowerFlowResult&,
                                             DynamicState& x,
                                             NetworkState&) {
  x.x[range_.offset + 0] = params_.p_ref_mw / std::max(1.0, params_.base_mva);
  x.x[range_.offset + 1] = clamp01(params_.soc_init, params_.soc_min, params_.soc_max);
}

bool BatteryDynamic::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  return set_if_changed(x.x,
                        range_.offset + 0,
                        params_.p_ref_mw / std::max(1.0, params_.base_mva));
}

void BatteryDynamic::computeDerivatives(double,
                                        const DynamicState& x,
                                        const NetworkState&,
                                        Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double tau = std::max(1e-4, params_.response_t_s);
  const double p_ref_pu = params_.p_ref_mw / std::max(1.0, params_.base_mva);
  const double p_pu = x.x[range_.offset + 0];
  const double p_mw = p_pu * params_.base_mva;
  const double e = std::max(1e-6, params_.e_rated_mwh);
  const double eta = p_mw >= 0.0 ? std::max(1e-6, params_.eta_discharge)
                                 : std::max(1e-6, params_.eta_charge);
  const double dsoc_power = p_mw >= 0.0 ? -p_mw / (eta * e) : -p_mw * eta / e;
  const double dsoc_self = -std::max(0.0, params_.self_discharge_pct_per_h) / 100.0 *
                           x.x[range_.offset + 1] / 3600.0;
  dxdt[range_.offset + 0] = (p_ref_pu - p_pu) / tau;
  dxdt[range_.offset + 1] = dsoc_power / 3600.0 + dsoc_self;
}

void BatteryDynamic::maskSlowStateResidual(Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!range_.empty() && range_.offset + 1 < dxdt.size()) {
    dxdt[range_.offset + 1] = 0.0;
  }
}

void BatteryDynamic::stamp(double,
                           const DynamicState& x,
                           const NetworkState& y,
                           DynamicStamp& stamp) const {
  if (!params_.stamp_power || !params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const double soc = x.x[range_.offset + 1];
  if (soc <= params_.soc_min + 1e-9 && x.x[range_.offset + 0] > 0.0) return;
  if (soc >= params_.soc_max - 1e-9 && x.x[range_.offset + 0] < 0.0) return;
  const double p = x.x[range_.offset + 0];
  const double q = params_.q_ref_mvar / std::max(1.0, params_.base_mva);
  if (params_.is_ac) {
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    for (int phase = 0; phase < 3; ++phase) {
      const Complex vv = std::abs(v[phase]) > kMinVoltage
                             ? v[phase]
                             : balanced_phasors(1.0, 0.0)[phase];
      stamp.addAcCurrent(3 * params_.bus_pos + phase,
                          std::conj(Complex(p / 3.0, q / 3.0) / vv));
    }
  } else {
    const double vdc = clamp_voltage(y.Vdc[params_.bus_pos]);
    stamp.addDcCurrent(params_.bus_pos, p / vdc);
  }
}

void BatteryDynamic::addJacobian(double,
                                 const DynamicState& x,
                                 const NetworkState& y,
                                 const DynamicJacobianContext& context,
                                 std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.stamp_power || !params_.in_service || params_.bus_pos < 0 ||
      range_.empty() || range_.offset + 1 >= x.x.size()) {
    return;
  }
  const int p_col = range_.offset;
  const double p = x.x[p_col];
  const double soc = x.x[range_.offset + 1];
  if (soc <= params_.soc_min + 1e-9 && p > 0.0) return;
  if (soc >= params_.soc_max - 1e-9 && p < 0.0) return;

  if (params_.is_ac) {
    const double q = params_.q_ref_mvar / std::max(1.0, params_.base_mva);
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    const Eigen::Vector3cd fallback = balanced_phasors(1.0, 0.0);
    for (int phase = 0; phase < 3; ++phase) {
      const int node = 3 * params_.bus_pos + phase;
      const Complex vv = std::abs(v[phase]) > kMinVoltage ? v[phase] : fallback[phase];
      if (context.validStateIndex(p_col)) {
        add_ac_current_derivative(context,
                                  node,
                                  p_col,
                                  std::conj(Complex(1.0 / 3.0, 0.0) / vv),
                                  triplets);
      }
      add_local_ac_voltage_current_derivative(
          context,
          node,
          v[phase],
          [&](Complex trial_v) {
            const Complex v_used = std::abs(trial_v) > kMinVoltage ? trial_v : fallback[phase];
            return std::conj(Complex(p / 3.0, q / 3.0) / v_used);
          },
          triplets);
    }
    return;
  }

  if (params_.bus_pos >= y.Vdc.size()) return;
  const double raw_vdc = y.Vdc[params_.bus_pos];
  const double vdc = clamp_voltage(raw_vdc);
  add_dc_current_derivative(context, params_.bus_pos, p_col, 1.0 / vdc, triplets);
  add_dc_current_derivative(context,
                            params_.bus_pos,
                            context.dcCol(params_.bus_pos),
                            -p * clamp_voltage_slope(raw_vdc) / (vdc * vdc),
                            triplets);
}

void BatteryDynamic::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index;
  if (!matching) return;
  if ((params_.is_ac && event.type == DynamicEventType::StoragePowerStep) ||
      (!params_.is_ac && event.type == DynamicEventType::DCStoragePowerStep)) {
    const auto it = event.params.find("p_ref_mw");
    params_.p_ref_mw = it == event.params.end() ? event.value : it->second;
    if (!range_.empty() && range_.offset < x.x.size()) {
      x.x[range_.offset] = params_.p_ref_mw / std::max(1.0, params_.base_mva);
    }
  }
}

std::string BatteryDynamic::name() const {
  return params_.label.empty() ? type() + " " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> BatteryDynamic::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput BatteryDynamic::output(const DynamicState& x,
                                           const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["stamp_power"] = params_.stamp_power ? 1.0 : 0.0;
  if (!range_.empty() && range_.offset + 1 < x.x.size()) {
    out.values["p_mw"] = x.x[range_.offset + 0] * safe_base(params_.base_mva);
    out.values["soc"] = x.x[range_.offset + 1];
    out.values["soc_min"] = params_.soc_min;
    out.values["soc_max"] = params_.soc_max;
  }
  out.values["q_ref_mvar"] = params_.q_ref_mvar;
  if (params_.is_ac) {
    add_voltage_metrics(out, y, params_.bus_pos);
  } else {
    add_voltage_metrics(out, y, -1, params_.bus_pos);
  }
  return out;
}

PVDynamic::PVDynamic(PVDynamicParams params) : params_(std::move(params)) {}

void PVDynamic::assignStateIndices(int& offset) {
  range_ = {offset, 2};
  offset += range_.size;
}

void PVDynamic::initializeFromPowerFlow(const PowerFlowResult&, DynamicState& x, NetworkState&) {
  if (range_.empty()) return;
  x.x[range_.offset + 0] =
      params_.p_ref_mw * std::max(0.0, params_.irradiance_pu) / safe_base(params_.base_mva);
  x.x[range_.offset + 1] = params_.q_ref_mvar / safe_base(params_.base_mva);
}

bool PVDynamic::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  bool changed = false;
  changed = set_if_changed(
                x.x,
                range_.offset + 0,
                params_.p_ref_mw * std::max(0.0, params_.irradiance_pu) /
                    safe_base(params_.base_mva)) ||
            changed;
  changed = set_if_changed(x.x,
                           range_.offset + 1,
                           params_.q_ref_mvar / safe_base(params_.base_mva)) ||
            changed;
  return changed;
}

void PVDynamic::computeDerivatives(double,
                                   const DynamicState& x,
                                   const NetworkState&,
                                   Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  const double tau = std::max(kMinTimeConstant, params_.response_t_s);
  const double p_ref = params_.p_ref_mw * std::max(0.0, params_.irradiance_pu) /
                       safe_base(params_.base_mva);
  const double q_ref = params_.q_ref_mvar / safe_base(params_.base_mva);
  dxdt[range_.offset + 0] = (p_ref - x.x[range_.offset + 0]) / tau;
  dxdt[range_.offset + 1] = (q_ref - x.x[range_.offset + 1]) / tau;
}

void PVDynamic::stamp(double,
                      const DynamicState& x,
                      const NetworkState& y,
                      DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  Complex s_total(x.x[range_.offset + 0], x.x[range_.offset + 1]);
  if (params_.current_limit_pu > 0.0) {
    const double vavg = std::max(kMinVoltage, avg_voltage_mag(v));
    const double i_mag = std::abs(s_total) / vavg;
    if (i_mag > params_.current_limit_pu) {
      s_total *= params_.current_limit_pu / std::max(i_mag, 1e-9);
    }
  }
  for (int phase = 0; phase < 3; ++phase) {
    const Complex vv = std::abs(v[phase]) > kMinVoltage
                           ? v[phase]
                           : balanced_phasors(1.0, 0.0)[phase];
    stamp.addAcCurrent(3 * params_.bus_pos + phase,
                        std::conj((s_total / 3.0) / vv));
  }
}

void PVDynamic::addJacobian(double,
                            const DynamicState& x,
                            const NetworkState& y,
                            const DynamicJacobianContext& context,
                            std::vector<Eigen::Triplet<double>>& triplets) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty() ||
      range_.offset + 1 >= x.x.size()) {
    return;
  }
  const int p_col = range_.offset;
  const int q_col = range_.offset + 1;
  const double p = x.x[p_col];
  const double q = x.x[q_col];
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const double vavg = std::max(kMinVoltage, avg_voltage_mag(v));
  if (params_.current_limit_pu > 0.0 &&
      std::abs(Complex(p, q)) / vavg > params_.current_limit_pu) {
    return;
  }
  const Eigen::Vector3cd fallback = balanced_phasors(1.0, 0.0);
  for (int phase = 0; phase < 3; ++phase) {
    const int node = 3 * params_.bus_pos + phase;
    const Complex vv = std::abs(v[phase]) > kMinVoltage ? v[phase] : fallback[phase];
    if (context.validStateIndex(p_col)) {
      add_ac_current_derivative(context,
                                node,
                                p_col,
                                std::conj(Complex(1.0 / 3.0, 0.0) / vv),
                                triplets);
    }
    if (context.validStateIndex(q_col)) {
      add_ac_current_derivative(context,
                                node,
                                q_col,
                                std::conj(Complex(0.0, 1.0 / 3.0) / vv),
                                triplets);
    }
    add_local_ac_voltage_current_derivative(
        context,
        node,
        v[phase],
        [&](Complex trial_v) {
          const Complex v_used = std::abs(trial_v) > kMinVoltage ? trial_v : fallback[phase];
          return std::conj((Complex(p, q) / 3.0) / v_used);
        },
        triplets);
  }
}

void PVDynamic::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index;
  if (!matching) return;
  if (event.type == DynamicEventType::GeneratorTrip ||
      event.type == DynamicEventType::VSCTrip) {
    params_.in_service = false;
    if (!range_.empty() && range_.offset + 1 < x.x.size()) {
      x.x.segment(range_.offset, range_.size).setZero();
    }
  } else if (event.type == DynamicEventType::Custom && event.component_type == "PV") {
    const auto it = event.params.find("irradiance_pu");
    params_.irradiance_pu =
        std::max(0.0, it == event.params.end() ? event.value : it->second);
    const auto p_it = event.params.find("p_ref_mw");
    if (p_it != event.params.end()) params_.p_ref_mw = p_it->second;
    const auto q_it = event.params.find("q_ref_mvar");
    if (q_it != event.params.end()) params_.q_ref_mvar = q_it->second;
  }
}

std::string PVDynamic::name() const {
  return params_.label.empty() ? "PV dynamic " + std::to_string(params_.component_index)
                               : params_.label;
}

std::vector<DynamicModelProfile> PVDynamic::modelProfiles() const {
  return profiles_or_default(params_.model_profiles, *this);
}

DynamicDeviceOutput PVDynamic::output(const DynamicState& x, const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["irradiance_pu"] = params_.irradiance_pu;
  out.values["current_limit_pu"] = params_.current_limit_pu;
  if (!range_.empty() && range_.offset + 1 < x.x.size()) {
    out.values["p_mw"] = x.x[range_.offset + 0] * safe_base(params_.base_mva);
    out.values["q_mvar"] = x.x[range_.offset + 1] * safe_base(params_.base_mva);
  }
  add_voltage_metrics(out, y, params_.bus_pos);
  return out;
}

ProtectionRelay::ProtectionRelay(ProtectionRelayParams params) : params_(std::move(params)) {}

void ProtectionRelay::assignStateIndices(int& offset) {
  range_ = {offset, 1};
  offset += range_.size;
}

void ProtectionRelay::initializeFromPowerFlow(const PowerFlowResult&, DynamicState& x, NetworkState&) {
  if (!range_.empty()) x.x[range_.offset] = 0.0;
}

bool ProtectionRelay::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (range_.empty()) return false;
  return set_if_changed(x.x, range_.offset, 0.0);
}

void ProtectionRelay::computeDerivatives(double,
                                         const DynamicState& x,
                                         const NetworkState& y,
                                         Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || params_.tripped || range_.empty()) return;
  const double v = params_.bus_pos >= 0 ? avg_voltage_mag(bus_voltage(y, params_.bus_pos)) : 1.0;
  const bool voltage_violation =
      v < params_.undervoltage_pickup_pu || v > params_.overvoltage_pickup_pu;
  if (voltage_violation) {
    dxdt[range_.offset] = 1.0;
  } else {
    dxdt[range_.offset] = -x.x[range_.offset] / std::max(kMinTimeConstant, params_.trip_delay_s);
  }
}

void ProtectionRelay::stamp(double,
                            const DynamicState&,
                            const NetworkState&,
                            DynamicStamp&) const {}

void ProtectionRelay::handleEvent(const DynamicEvent& event, DynamicState& x, NetworkState&) {
  const bool matching = event.component_index == 0 ||
                        event.component_index == params_.component_index;
  if (!matching) return;
  if (event.type == DynamicEventType::Custom && event.component_type == "ProtectionRelay") {
    params_.tripped = event.value != 0.0;
    if (!range_.empty() && range_.offset < x.x.size() && !params_.tripped) {
      x.x[range_.offset] = 0.0;
    }
  }
}

std::string ProtectionRelay::name() const {
  return params_.label.empty()
             ? "Protection relay " + std::to_string(params_.component_index)
             : params_.label;
}

DynamicDeviceOutput ProtectionRelay::output(const DynamicState& x,
                                            const NetworkState& y) const {
  DynamicDeviceOutput out = make_output_base(*this,
                                             params_.bus,
                                             params_.canvas_type,
                                             params_.component_domain,
                                             params_.source_type);
  const double timer =
      (!range_.empty() && range_.offset < x.x.size()) ? std::max(0.0, x.x[range_.offset]) : 0.0;
  out.values["in_service"] = params_.in_service ? 1.0 : 0.0;
  out.values["tripped"] = params_.tripped ? 1.0 : 0.0;
  out.values["timer_s"] = timer;
  out.values["trip_delay_s"] = params_.trip_delay_s;
  out.values["pickup_active"] = timer >= params_.trip_delay_s ? 1.0 : 0.0;
  out.values["undervoltage_pickup_pu"] = params_.undervoltage_pickup_pu;
  out.values["overvoltage_pickup_pu"] = params_.overvoltage_pickup_pu;
  add_voltage_metrics(out, y, params_.bus_pos);
  return out;
}

}  // namespace hacdcpf::dynamics
