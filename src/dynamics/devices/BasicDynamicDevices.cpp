#include "hacdcpf/dynamics/devices/BasicDynamicDevices.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <string>

#include <Eigen/Dense>

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

Complex load_consuming_current(Complex v,
                               double p_pu,
                               double q_pu,
                               double v0,
                               DynamicLoadModelKind kind,
                               const ACLoadDynamicParams& params) {
  const double vr = v.real();
  const double vi = v.imag();
  const double vmag = std::max(kMinVoltage, std::abs(v));
  const double vmag2 = std::max(kMinVoltage * kMinVoltage, std::norm(v));
  const double v0c = std::max(kMinVoltage, v0);
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
    const Eigen::VectorXd step = jac.colPivHouseholderQr().solve(-r);
    if (!step.allFinite()) break;
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
  if (params_.model_kind == DynamicLoadModelKind::ConstantImpedance) {
    const Complex y_load = std::conj(s_pu) / 3.0;
    add_balanced_admittance(stamp, params_.bus_pos, diagonal_admittance(y_load));
    return;
  }

  const Eigen::Vector3cd v = bus_voltage(y, params_.bus_pos);
  const double p_phase = s_pu.real() / 3.0;
  const double q_phase = s_pu.imag() / 3.0;
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
  range_ = {offset, params_.psd_genrou_model ? 8 : 4};
  offset += range_.size;
  // Publish the coupling link so attached controllers can address this machine's
  // speed / mechanical-power / field states. Local indices: omega=1 for both
  // models; classical pm=3, e_mag(field)=2; GENROU tau_m=6, vf(field)=7.
  link_.range = &range_;
  link_.valid = true;
  link_.genrou = params_.psd_genrou_model && range_.size >= 8;
  link_.bus_pos = params_.bus_pos;
  link_.base_mva = params_.base_mva;
  link_.frequency_hz = params_.frequency_hz;
  link_.inertia_h = std::max(0.01, params_.inertia_h);
  link_.omega_local = 1;
  link_.pm_local = link_.genrou ? 6 : 3;
  link_.efd_local = link_.genrou ? 7 : 2;
}

void SynchronousMachine::initializeFromPowerFlow(const PowerFlowResult& pf,
                                                DynamicState& x,
                                                NetworkState& y) {
  if (range_.empty()) return;
  double angle = params_.angle_set_rad;
  if (params_.bus_pos >= 0 && params_.bus_pos < static_cast<int>(pf.va.size())) {
    angle = pf.va[static_cast<std::size_t>(params_.bus_pos)];
  }
  if (params_.psd_genrou_model && range_.size >= 8) {
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
  const Complex s_phase(p_pu / 3.0, q_pu / 3.0);
  const Complex i_phase = std::conj(s_phase / vt);
  const Complex z(params_.r_pu, std::max(1e-5, params_.x_pu));
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
  if (params_.psd_genrou_model && range_.size >= 8) {
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
    const Complex s_phase(p_pu / 3.0, q_pu / 3.0);
    const Complex i_phase = std::conj(s_phase / vt);
    const Complex z(params_.r_pu, std::max(1e-5, params_.x_pu));
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
  if (params_.psd_genrou_model && range_.size >= 8) {
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
    dxdt[state_index(range_, 1)] =
        params_.dynamic_angle
            ? (tau_m - e.tau_e - params_.damping_d * (omega - 1.0) /
                                      std::max(kMinVoltage, std::abs(omega))) /
                  (2.0 * h)
            : 0.0;
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
  // e_mag (field, idx 2) and pm (idx 3) are constant unless an exciter / governor
  // is attached, in which case that controller owns their derivative.
  if (!exciter_attached_) dxdt[range_.offset + 2] = 0.0;
  if (!governor_attached_) dxdt[range_.offset + 3] = 0.0;
}

void SynchronousMachine::stamp(double,
                               const DynamicState& x,
                               const NetworkState&,
                               DynamicStamp& stamp) const {
  if (!params_.in_service || params_.bus_pos < 0 || range_.empty()) return;
  if (params_.psd_genrou_model && range_.size >= 8) {
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

std::string SynchronousMachine::modelName() const {
  return params_.psd_genrou_model ? "GENROU" : "ClassicalMachine";
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
  if (params_.psd_genrou_model && !range_.empty() && range_.offset + 7 < x.x.size()) {
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
    out.values["psd_genrou"] = 1.0;
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
  const double t2 = std::max(kMinTimeConstant, L.t2_s);
  const double t4 = std::max(kMinTimeConstant, L.t4_s);
  const double y_w = L.ks * u - x.x[o + 0];
  const double y1 = (L.t1_s / t2) * y_w + x.x[o + 1];
  const double y2 = (L.t3_s / t4) * y1 + x.x[o + 2];
  return std::clamp(y2, L.vs_min_pu, L.vs_max_pu);
}

Governor::Governor(GovernorDynamicParams params) : params_(std::move(params)) {}

void Governor::assignStateIndices(int& offset) {
  range_ = {offset, 1};  // one governor/valve state; machine owns the pm slot
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
      params_.p_ref_mw = p0 * safe_base(params_.base_mva);
    }
  }
  x.x[range_.offset] = p0;  // valve position = scheduled power
}

bool Governor::trimToNetworkEquilibrium(DynamicState& x, NetworkState&) {
  if (!params_.in_service || range_.empty()) return false;
  double p0 = params_.p_ref_mw / safe_base(params_.base_mva);
  if (machine_ != nullptr && machine_->valid) {
    const int pm_idx = machine_->pmIndex();
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      p0 = x.x[pm_idx];
      params_.p_ref_mw = p0 * safe_base(params_.base_mva);
    }
  }
  return set_if_changed(x.x, range_.offset, p0);
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
  const double t1 = std::max(kMinTimeConstant, params_.t_s);
  const double valve = x.x[range_.offset];

  if (machine_ != nullptr && machine_->valid) {
    // Load-reference-set governor: valve responds to droop about the speed
    // deviation; the machine's pm/tau_m slot is the turbine output (lag of the
    // valve). Reheat models (IEEEG1) simply use a longer turbine time constant.
    const int omega_idx = machine_->omegaIndex();
    const int pm_idx = machine_->pmIndex();
    const double omega = (omega_idx >= 0 && omega_idx < x.x.size()) ? x.x[omega_idx] : 1.0;
    const double droop = params_.droop_r > 1e-9 ? params_.droop_r : 0.05;
    const double valve_cmd = std::clamp(pref - (omega - 1.0) / droop, pmin, pmax);
    dxdt[range_.offset] = (valve_cmd - valve) / t1;
    if (pm_idx >= 0 && pm_idx < x.x.size()) {
      const double t_turb = std::max(
          kMinTimeConstant,
          params_.model == GovernorModel::IEEEG1 ? params_.reheat_t_s : params_.turbine_t_s);
      dxdt[pm_idx] = (valve - x.x[pm_idx]) / t_turb;
    }
    return;
  }
  // Backward-compatible standalone behaviour: own state tracks the reference.
  dxdt[range_.offset] = (pref - valve) / t1;
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
  // When coupled to a machine the exciter drives the machine-owned field state
  // and needs no state of its own; standalone it keeps a single efd state for
  // backward compatibility.
  range_ = {offset, machine_ != nullptr ? 0 : 1};
  offset += range_.size;
}

double Exciter::terminalVoltage(const NetworkState& y) const {
  const int bus = machine_ != nullptr && machine_->valid ? machine_->bus_pos : params_.bus_pos;
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
  v_ref_captured_ = vt;  // hold the equilibrium terminal voltage as the AVR setpoint
  captured_ = true;
}

void Exciter::initializeFromPowerFlow(const PowerFlowResult& pf, DynamicState& x, NetworkState& y) {
  // Prefer the power-flow terminal voltage at init (network not yet solved here).
  double vt = params_.v_ref_pu;
  const int bus = machine_ != nullptr && machine_->valid ? machine_->bus_pos : params_.bus_pos;
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
  captured_ = true;
}

bool Exciter::trimToNetworkEquilibrium(DynamicState& x, NetworkState& y) {
  if (!params_.in_service) return false;
  captureReference(x, y);  // re-anchor to the network-solved equilibrium
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
  const double t = std::max(
      kMinTimeConstant,
      params_.model == ExciterModel::IEEET1 ? params_.te_s : params_.ta_s);

  if (machine_ != nullptr && machine_->valid) {
    // Proportional AVR about the equilibrium field: efd_cmd = field0 +
    // Ka*(Vref - Vt + Vs). Drives the machine-owned field state through a lag.
    const int f = machine_->fieldIndex();
    if (f < 0 || f >= x.x.size()) return;
    const double efd_cmd = std::clamp(field0_ + params_.ka * (vref - vt + vs),
                                      params_.efd_min_pu, params_.efd_max_pu);
    dxdt[f] = (efd_cmd - x.x[f]) / t;
    return;
  }
  if (range_.empty()) return;
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
  add_voltage_metrics(out, y, machine_ != nullptr && machine_->valid ? machine_->bus_pos
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
  range_ = {offset, 3};  // washout + two lead-lag states
  offset += range_.size;
  link_.range = &range_;
  link_.machine = machine_;
  link_.valid = machine_ != nullptr && machine_->valid;
  link_.ks = params_.ks;
  link_.tw_s = params_.tw_s;
  link_.t1_s = params_.t1_s;
  link_.t2_s = params_.t2_s;
  link_.t3_s = params_.t3_s;
  link_.t4_s = params_.t4_s;
  link_.vs_max_pu = params_.vs_max_pu;
  link_.vs_min_pu = params_.vs_min_pu;
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
  const double tw = std::max(kMinTimeConstant, params_.tw_s);
  const double t2 = std::max(kMinTimeConstant, params_.t2_s);
  const double t4 = std::max(kMinTimeConstant, params_.t4_s);
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
