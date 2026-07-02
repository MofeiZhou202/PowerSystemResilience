#include "hacdcpf/dynamics/devices/BasicDynamicDevices.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <string>

namespace hacdcpf::dynamics {
namespace {

using Complex = std::complex<double>;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kMinVoltage = 1e-4;
constexpr double kMinTimeConstant = 1e-4;

int state_index(const StateIndexRange& range, int local) {
  return range.offset + local;
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

Complex average_complex_power(const Eigen::Vector3cd& v,
                              const Eigen::Vector3cd& i) {
  Complex s{0.0, 0.0};
  for (int phase = 0; phase < 3; ++phase) {
    s += v[phase] * std::conj(i[phase]);
  }
  return s;
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
                        const NetworkState&,
                        DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || params_.base_mva <= 0.0) return;
  const Complex s_pu(params_.p_mw / params_.base_mva * params_.scale,
                     params_.q_mvar / params_.base_mva * params_.scale);
  const Complex y_load = std::conj(s_pu) / 3.0;
  add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(y_load));
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
  add_voltage_metrics(out, y, params_.bus_pos);
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
  range_ = {offset, 4};
  offset += range_.size;
}

void SynchronousMachine::initializeFromPowerFlow(const PowerFlowResult& pf,
                                                DynamicState& x,
                                                NetworkState&) {
  if (range_.empty()) return;
  double angle = params_.angle_set_rad;
  if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.va.size())) {
    angle = pf.va[static_cast<std::size_t>(params_.bus_pos)];
  }
  x.x[range_.offset + 0] = angle;
  x.x[range_.offset + 1] = 1.0;
  x.x[range_.offset + 2] = params_.vm_set_pu;
  x.x[range_.offset + 3] = params_.p_mech_mw / std::max(1.0, params_.base_mva);
}

bool SynchronousMachine::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service || range_.empty()) return false;
  bool changed = false;
  changed = set_if_changed(x.x, state_index(range_, 1), 1.0) || changed;
  if (params_.dynamic_angle) {
    const double theta = x.x[state_index(range_, 0)];
    const double e_mag = x.x[state_index(range_, 2)];
    const Complex z(params_.r_pu, std::max(1e-5, params_.x_pu));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
    const double pe = finite_value(average_complex_power(v, yv * (e - v)).real());
    changed = set_if_changed(x.x, state_index(range_, 3), pe) || changed;
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
  const Complex z(params_.r_pu, std::max(1e-5, params_.x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
  const Eigen::Vector3cd i = yv * (e - v);
  const double pe = average_complex_power(v, i).real();
  const double h = std::max(0.01, params_.inertia_h);
  const double wb = kTwoPi * params_.frequency_hz;

  dxdt[range_.offset + 0] = params_.dynamic_angle ? wb * (omega - 1.0) : 0.0;
  dxdt[range_.offset + 1] =
      params_.dynamic_angle ? (pm - pe - params_.damping_d * (omega - 1.0)) / (2.0 * h) : 0.0;
  dxdt[range_.offset + 2] = 0.0;
  dxdt[range_.offset + 3] = 0.0;
}

void SynchronousMachine::stamp(double,
                               const DynamicState& x,
                               const NetworkState&,
                               DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  const double theta = x.x[range_.offset + 0];
  const double e_mag = x.x[range_.offset + 2];
  const Complex z(params_.r_pu, std::max(1e-5, params_.x_pu));
  const Complex yv = Complex(1.0, 0.0) / z;
  const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
  add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(yv));
  add_balanced_current(stamp, params_.bus_pos, yv * e);
}

void SynchronousMachine::handleEvent(const DynamicEvent& event, DynamicState&, NetworkState&) {
  if (event.type == DynamicEventType::GeneratorTrip &&
      (event.component_index == 0 || event.component_index == params_.component_index)) {
    params_.in_service = false;
  }
}

std::string SynchronousMachine::name() const {
  return params_.label.empty() ? params_.device_type + " " + std::to_string(params_.component_index)
                               : params_.label;
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
  if (!range_.empty() && range_.offset + 3 < x.x.size()) {
    const double theta = x.x[state_index(range_, 0)];
    const double omega = x.x[state_index(range_, 1)];
    const double e_mag = x.x[state_index(range_, 2)];
    const double pm = x.x[state_index(range_, 3)];
    out.values["angle_rad"] = theta;
    out.values["omega_pu"] = omega;
    out.values["frequency_hz"] = omega * params_.frequency_hz;
    out.values["e_internal_pu"] = e_mag;
    out.values["p_mech_mw"] = pm * safe_base(params_.base_mva);
    const Complex z(params_.r_pu, std::max(1e-5, params_.x_pu));
    const Complex yv = Complex(1.0, 0.0) / z;
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    const Eigen::Vector3cd e = balanced_phasors(e_mag, theta);
    const Eigen::Vector3cd i = yv * (e - v);
    const Complex s = average_complex_power(v, i);
    out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
    out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
    out.values["i_rms_pu"] =
        std::sqrt((std::norm(i[0]) + std::norm(i[1]) + std::norm(i[2])) / 3.0);
  }
  add_voltage_metrics(out, y, params_.bus_pos);
  return out;
}

Governor::Governor(GovernorDynamicParams params) : params_(std::move(params)) {}

void Governor::assignStateIndices(int& offset) {
  range_ = {offset, 1};
  offset += range_.size;
}

void Governor::initializeFromPowerFlow(const PowerFlowResult&, DynamicState& x, NetworkState&) {
  if (range_.empty()) return;
  x.x[range_.offset] = params_.p_ref_mw / safe_base(params_.base_mva);
}

bool Governor::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  return set_if_changed(x.x,
                        range_.offset,
                        params_.p_ref_mw / safe_base(params_.base_mva));
}

void Governor::computeDerivatives(double,
                                  const DynamicState& x,
                                  const NetworkState&,
                                  Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  double pref = params_.p_ref_mw / safe_base(params_.base_mva);
  const double pmax = params_.pmax_mw > 0.0
                          ? params_.pmax_mw / safe_base(params_.base_mva)
                          : std::numeric_limits<double>::infinity();
  const double pmin = params_.pmin_mw < 0.0
                          ? params_.pmin_mw / safe_base(params_.base_mva)
                          : -std::numeric_limits<double>::infinity();
  pref = std::clamp(pref, pmin, pmax);
  dxdt[range_.offset] = (pref - x.x[range_.offset]) /
                        std::max(kMinTimeConstant, params_.t_s);
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
  if (!range_.empty() && range_.offset < x.x.size()) {
    out.values["p_mech_mw"] = x.x[range_.offset] * safe_base(params_.base_mva);
  }
  return out;
}

Exciter::Exciter(ExciterDynamicParams params) : params_(std::move(params)) {}

void Exciter::assignStateIndices(int& offset) {
  range_ = {offset, 1};
  offset += range_.size;
}

void Exciter::initializeFromPowerFlow(const PowerFlowResult&, DynamicState& x, NetworkState& y) {
  if (range_.empty()) return;
  double vt = params_.v_ref_pu;
  if (params_.bus_pos >= 0) vt = avg_voltage_mag(bus_voltage(y, params_.bus_pos));
  x.x[range_.offset] = std::clamp(vt, params_.efd_min_pu, params_.efd_max_pu);
}

bool Exciter::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service || range_.empty()) return false;
  double vt = params_.v_ref_pu;
  if (params_.bus_pos >= 0) vt = avg_voltage_mag(bus_voltage(y, params_.bus_pos));
  return set_if_changed(x.x,
                        range_.offset,
                        std::clamp(vt, params_.efd_min_pu, params_.efd_max_pu));
}

void Exciter::computeDerivatives(double,
                                 const DynamicState& x,
                                 const NetworkState& y,
                                 Eigen::Ref<Eigen::VectorXd> dxdt) const {
  if (!params_.in_service || range_.empty()) return;
  double vt = params_.v_ref_pu;
  if (params_.bus_pos >= 0) vt = avg_voltage_mag(bus_voltage(y, params_.bus_pos));
  const double efd_cmd =
      std::clamp(params_.ka * (params_.v_ref_pu - vt) + params_.v_ref_pu,
                 params_.efd_min_pu,
                 params_.efd_max_pu);
  dxdt[range_.offset] = (efd_cmd - x.x[range_.offset]) /
                        std::max(kMinTimeConstant, params_.ta_s);
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
  out.values["v_ref_pu"] = params_.v_ref_pu;
  out.values["ka"] = params_.ka;
  if (!range_.empty() && range_.offset < x.x.size()) {
    out.values["efd_pu"] = x.x[range_.offset];
  }
  add_voltage_metrics(out, y, params_.bus_pos);
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
  x.x[state_index(range_, 2)] = params_.p_ref_mw / safe_base(params_.base_mva);
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
  changed = set_if_changed(x.x, state_index(range_, 2), p_ref) || changed;
  changed = set_if_changed(x.x, state_index(range_, 3), q_ref) || changed;
  changed = set_if_changed(x.x, state_index(range_, 4), finite_value(xi_v)) || changed;
  changed = set_if_changed(x.x, state_index(range_, 5), finite_value(xi_ol)) || changed;
  if (range_.size > 6) {
    double vdc = params_.vdc_ref_pu;
    if (params_.dc_bus_pos >= 0 && params_.dc_bus_pos < y.Vdc.size()) {
      vdc = y.Vdc[params_.dc_bus_pos];
    }
    changed = set_if_changed(x.x,
                             state_index(range_, 6),
                             clamp_voltage_window(vdc,
                                                  params_.vdc_min_pu,
                                                  params_.vdc_max_pu)) || changed;
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
  const double pf = x.x[state_index(range_, 2)];
  const double qf = x.x[state_index(range_, 3)];
  const double xi_v = x.x[state_index(range_, 4)];
  const double xi_ol = x.x[state_index(range_, 5)];
  const double t_filter = std::max(kMinTimeConstant, params_.power_filter_t_s);
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
  const double angle_rate =
      -kTwoPi * params_.frequency_hz * params_.p_droop_pu * (pf - p_ref) -
      overload_correction;

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

  dxdt[state_index(range_, 0)] = angle_rate;
  dxdt[state_index(range_, 1)] = (e_cmd - e_mag) / t_voltage;
  dxdt[state_index(range_, 2)] = (p - pf) / t_filter;
  dxdt[state_index(range_, 3)] = (q - qf) / t_filter;
  dxdt[state_index(range_, 4)] = voltage_error;
  dxdt[state_index(range_, 5)] = overload;
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
  if (!range_.empty() && state_index(range_, 5) < x.x.size()) {
    const double theta = x.x[state_index(range_, 0)];
    const double e_mag = x.x[state_index(range_, 1)];
    const double pf = x.x[state_index(range_, 2)];
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
    out.values["e_internal_pu"] = e_mag;
    out.values["p_mw"] = s.real() * safe_base(params_.base_mva);
    out.values["q_mvar"] = s.imag() * safe_base(params_.base_mva);
    out.values["p_filtered_mw"] = pf * safe_base(params_.base_mva);
    out.values["q_filtered_mvar"] = qf * safe_base(params_.base_mva);
    out.values["p_ref_mw"] = params_.p_ref_mw;
    out.values["q_ref_mvar"] = params_.q_ref_mvar;
    out.values["frequency_hz"] =
        params_.frequency_hz * (1.0 - params_.p_droop_pu * (pf - p_ref));
    out.values["i_rms_pu"] =
        std::sqrt((std::norm(i[0]) + std::norm(i[1]) + std::norm(i[2])) / 3.0);
    out.values["current_limit_active"] =
        (params_.current_limit_pu > 0.0 && out.values["i_rms_pu"] > params_.current_limit_pu)
            ? 1.0
            : 0.0;
    out.values["overload_pu"] = overload;
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
    } else {
      out.values["dc_link_dynamic"] = 0.0;
    }
  }
  add_voltage_metrics(out, y, params_.bus_pos, params_.dc_bus_pos);
  return out;
}

GridFollowingInverter::GridFollowingInverter(GridFollowingInverterParams params)
    : params_(std::move(params)) {}

void GridFollowingInverter::assignStateIndices(int& offset) {
  range_ = {offset, gfl_state_count(params_)};
  offset += range_.size;
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
    changed = set_if_changed(x.x,
                             state_index(range_, vdc_local),
                             clamp_voltage_window(vdc,
                                                  params_.vdc_min_pu,
                                                  params_.vdc_max_pu)) || changed;
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
    const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
    const double p_ac = average_complex_power(v, current).real();
    const double p_dc_pu = -p_ac / std::max(1e-6, params_.eta);
    const double vdc = (params_.dc_bus_pos < y.Vdc.size()) ? clamp_voltage(y.Vdc[params_.dc_bus_pos]) : 1.0;
    stamp.addDcCurrent(params_.dc_bus_pos, p_dc_pu / vdc);
  }
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
    } else {
      out.values["dc_link_dynamic"] = 0.0;
    }
  }
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
  gfm.model_profiles = params.model_profiles;
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
