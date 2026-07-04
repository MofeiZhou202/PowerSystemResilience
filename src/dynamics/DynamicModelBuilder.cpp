#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <unordered_set>

#include <Eigen/LU>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/dynamics/devices/BasicDynamicDevices.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/projection/project_to_canonical.hpp"

namespace hacdcpf::dynamics {
namespace {

using Complex = std::complex<double>;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegToRad = kPi / 180.0;

template <typename T>
T positive_or(T value, T fallback) {
  return value > T{} ? value : fallback;
}

double scale_or_one(double value) {
  return value > 0.0 ? value : 1.0;
}

std::string label_or(const std::string& label, const std::string& fallback) {
  return label.empty() ? fallback : label;
}

std::vector<DynamicModelProfile> to_dynamic_profiles(
    const hacdcpf::DynamicModelProfile& profile) {
  std::vector<DynamicModelProfile> profiles;
  if (profile.empty()) return profiles;
  DynamicModelProfile root;
  root.standard = profile.standard;
  root.model_name = profile.model_name;
  root.parameter_set = profile.parameter_set;
  root.source_id = profile.source_id;
  root.notes = profile.notes;
  profiles.push_back(std::move(root));
  for (const auto& component : profile.components) {
    DynamicModelProfile child;
    child.standard = component.standard;
    child.profile = component.type;
    child.model_name = component.model;
    child.parameter_set = component.parameter_set;
    child.source_id = profile.source_id;
    child.notes = profile.notes;
    profiles.push_back(std::move(child));
  }
  return profiles;
}

bool iequals(std::string lhs, std::string rhs) {
  std::transform(lhs.begin(), lhs.end(), lhs.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  std::transform(rhs.begin(), rhs.end(), rhs.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return lhs == rhs;
}

// Alias-tolerant numeric lookup over a parameter map.
double param_or(const std::map<std::string, double>& m,
                std::initializer_list<const char*> keys,
                double fallback) {
  for (const char* key : keys) {
    const auto it = m.find(key);
    if (it != m.end()) return it->second;
  }
  return fallback;
}

void apply_gfl_profile(const hacdcpf::DynamicModelProfile& profile,
                       GridFollowingInverterParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  if (profile.empty()) return;
  auto apply_name = [&](const std::string& name) {
    if (iequals(name, "KauraPLL")) {
      params.frequency_estimator = FrequencyEstimatorKind::KauraPLL;
      params.frequency_estimator_name = "KauraPLL";
    } else if (iequals(name, "ReducedOrderPLL")) {
      params.frequency_estimator = FrequencyEstimatorKind::ReducedOrderPLL;
      params.frequency_estimator_name = "ReducedOrderPLL";
    } else if (iequals(name, "FixedFrequency")) {
      params.frequency_estimator = FrequencyEstimatorKind::FixedFrequency;
      params.frequency_estimator_name = "FixedFrequency";
    }
  };
  apply_name(profile.model_name);
  for (const auto& component : profile.components) {
    apply_name(component.model);
    if (!iequals(component.type, "pll")) continue;
    auto it = component.parameters.find("kp_pll");
    if (it != component.parameters.end()) params.pll_kp = it->second;
    it = component.parameters.find("ki_pll");
    if (it != component.parameters.end()) params.pll_ki = it->second;
    it = component.parameters.find("pll_lpf_t_s");
    if (it != component.parameters.end()) params.pll_lpf_t_s = it->second;
  }
  auto it = profile.parameters.find("pll_kp");
  if (it != profile.parameters.end()) params.pll_kp = it->second;
  it = profile.parameters.find("pll_ki");
  if (it != profile.parameters.end()) params.pll_ki = it->second;
  it = profile.parameters.find("pll_lpf_t_s");
  if (it != profile.parameters.end()) params.pll_lpf_t_s = it->second;
  const auto& p = profile.parameters;
  params.response_t_s = param_or(p, {"response_t_s", "Tg", "Trv"}, params.response_t_s);
  params.power_filter_t_s = param_or(p, {"power_filter_t_s", "Tp", "Tpf"}, params.power_filter_t_s);
  params.current_limit_pu = param_or(p, {"current_limit_pu", "Imax", "imax_pu"}, params.current_limit_pu);
  params.frequency_watt_droop_pu = param_or(p, {"frequency_watt_droop_pu", "Ddn", "kf"}, params.frequency_watt_droop_pu);
  params.volt_var_droop_pu = param_or(p, {"volt_var_droop_pu", "Dvv", "kq"}, params.volt_var_droop_pu);
}

template <typename P>
void apply_gfm_params(const std::map<std::string, double>& p, P& params) {
  params.virtual_r_pu = param_or(p, {"virtual_r_pu", "Rv", "rv"}, params.virtual_r_pu);
  params.virtual_x_pu = param_or(p, {"virtual_x_pu", "Xv", "xv"}, params.virtual_x_pu);
  params.p_droop_pu = param_or(p, {"p_droop_pu", "mp", "Dp"}, params.p_droop_pu);
  params.q_droop_pu = param_or(p, {"q_droop_pu", "mq", "Dq"}, params.q_droop_pu);
  params.power_filter_t_s = param_or(p, {"power_filter_t_s", "Tf", "Tpf"}, params.power_filter_t_s);
  params.voltage_control_t_s = param_or(p, {"voltage_control_t_s", "Tv"}, params.voltage_control_t_s);
  params.voltage_kp = param_or(p, {"voltage_kp", "Kpv"}, params.voltage_kp);
  params.voltage_ki = param_or(p, {"voltage_ki", "Kiv"}, params.voltage_ki);
  params.current_limit_pu = param_or(p, {"current_limit_pu", "Imax", "imax_pu"}, params.current_limit_pu);
}

void apply_gfm_profile(const hacdcpf::DynamicModelProfile& profile,
                       GridFormingInverterParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  if (!profile.empty()) apply_gfm_params(profile.parameters, params);
}

void apply_gfm_profile(const hacdcpf::DynamicModelProfile& profile,
                       VSCConverterDynamicParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  if (!profile.empty()) apply_gfm_params(profile.parameters, params);
}

void apply_voltage_source_profile(const hacdcpf::DynamicModelProfile& profile,
                                  VoltageSourceDynamicParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  if (profile.empty()) return;
  auto find_param = [&](const std::initializer_list<const char*> keys,
                        double fallback) {
    for (const char* key : keys) {
      const auto it = profile.parameters.find(key);
      if (it != profile.parameters.end()) return it->second;
    }
    return fallback;
  };
  if (iequals(profile.model_name, "GENROU") ||
      iequals(profile.model_name, "RoundRotorQuadratic") ||
      iequals(profile.model_name, "RoundRotorExponential")) {
    params.psd_genrou_model = true;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xl_pu = find_param({"Xl", "xl"}, params.xl_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
    params.saturation_a = find_param({"Sat_A", "saturation_a", "Se_A"}, params.saturation_a);
    params.saturation_b = find_param({"Sat_B", "saturation_b", "Se_B"}, params.saturation_b);
  }
}

// Build a Governor from a machine's dynamic_model "governor" component profile.
std::unique_ptr<Governor> make_governor(const hacdcpf::DynamicModelComponentProfile& comp,
                                        const VoltageSourceDynamicParams& mp) {
  GovernorDynamicParams g;
  g.component_index = mp.component_index;
  g.machine_index = mp.component_index;
  g.label = label_or(mp.label, "Machine " + std::to_string(mp.component_index)) + " governor";
  g.base_mva = mp.base_mva;
  g.model_name = comp.model.empty() ? "TGOV1" : comp.model;
  g.model = iequals(comp.model, "IEEEG1") ? GovernorModel::IEEEG1 : GovernorModel::TGOV1;
  g.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  const double base = std::max(1.0, mp.base_mva);
  g.droop_r = param_or(p, {"R", "droop_r", "droop"}, g.droop_r);
  g.t_s = param_or(p, {"T1", "t1", "Tg", "valve_t_s"}, g.t_s);
  g.turbine_t_s = param_or(p, {"T3", "t3", "Tt", "turbine_t_s"}, g.turbine_t_s);
  g.reheat_t_s = param_or(p, {"Tr", "T5", "reheat_t_s"}, g.reheat_t_s);
  g.reheat_k = param_or(p, {"K1", "Khp", "reheat_k"}, g.reheat_k);
  const double vmax = param_or(p, {"Vmax", "pmax_pu"}, 0.0);
  const double vmin = param_or(p, {"Vmin", "pmin_pu"}, 0.0);
  g.pmax_mw = vmax > 0.0 ? vmax * base : param_or(p, {"pmax_mw", "Pmax"}, g.pmax_mw);
  g.pmin_mw = vmin < 0.0 ? vmin * base : param_or(p, {"pmin_mw", "Pmin"}, g.pmin_mw);
  return std::make_unique<Governor>(g);
}

// Build an Exciter/AVR from a machine's dynamic_model "exciter" component profile.
std::unique_ptr<Exciter> make_exciter(const hacdcpf::DynamicModelComponentProfile& comp,
                                      const VoltageSourceDynamicParams& mp) {
  ExciterDynamicParams e;
  e.component_index = mp.component_index;
  e.machine_index = mp.component_index;
  e.bus = mp.bus;
  e.bus_pos = mp.bus_pos;
  e.label = label_or(mp.label, "Machine " + std::to_string(mp.component_index)) + " exciter";
  e.model_name = comp.model.empty() ? "SEXS" : comp.model;
  e.model = iequals(comp.model, "IEEET1") ? ExciterModel::IEEET1 : ExciterModel::SEXS;
  e.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  e.ka = param_or(p, {"Ka", "K", "ka"}, e.ka);
  e.ta_s = param_or(p, {"Ta", "ta", "Tb"}, e.ta_s);
  e.te_s = param_or(p, {"Te", "te"}, e.te_s);
  e.efd_max_pu = param_or(p, {"Emax", "Vrmax", "efd_max_pu"}, e.efd_max_pu);
  e.efd_min_pu = param_or(p, {"Emin", "Vrmin", "efd_min_pu"}, e.efd_min_pu);
  e.v_ref_pu = param_or(p, {"Vref", "v_ref_pu"}, e.v_ref_pu);
  return std::make_unique<Exciter>(e);
}

// Build a PSS from a machine's dynamic_model "pss" component profile.
std::unique_ptr<PowerSystemStabilizer> make_pss(
    const hacdcpf::DynamicModelComponentProfile& comp,
    const VoltageSourceDynamicParams& mp) {
  PSSDynamicParams s;
  s.component_index = mp.component_index;
  s.machine_index = mp.component_index;
  s.bus = mp.bus;
  s.bus_pos = mp.bus_pos;
  s.label = label_or(mp.label, "Machine " + std::to_string(mp.component_index)) + " PSS";
  s.model_name = comp.model.empty() ? "PSS1A" : comp.model;
  s.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  s.ks = param_or(p, {"Ks", "Ks1", "ks"}, s.ks);
  s.tw_s = param_or(p, {"Tw", "tw"}, s.tw_s);
  s.t1_s = param_or(p, {"T1", "t1"}, s.t1_s);
  s.t2_s = param_or(p, {"T2", "t2"}, s.t2_s);
  s.t3_s = param_or(p, {"T3", "t3"}, s.t3_s);
  s.t4_s = param_or(p, {"T4", "t4"}, s.t4_s);
  s.vs_max_pu = param_or(p, {"Vsmax", "vs_max_pu", "Vstmax"}, s.vs_max_pu);
  s.vs_min_pu = param_or(p, {"Vsmin", "vs_min_pu", "Vstmin"}, s.vs_min_pu);
  return std::make_unique<PowerSystemStabilizer>(s);
}

// Instantiate and wire governor / exciter / PSS control blocks listed in a
// machine's dynamic_model.components, attaching each to the just-created machine.
void wire_machine_controllers(std::vector<std::unique_ptr<DynamicDevice>>& devices,
                              SynchronousMachine* machine,
                              const VoltageSourceDynamicParams& mp,
                              const hacdcpf::DynamicModelProfile& dm,
                              std::vector<std::string>& warnings) {
  if (machine == nullptr) return;
  Exciter* exciter = nullptr;
  PowerSystemStabilizer* pss = nullptr;
  bool has_pss = false;
  for (const auto& comp : dm.components) {
    if (iequals(comp.type, "governor") || iequals(comp.type, "turbine_governor")) {
      auto g = make_governor(comp, mp);
      g->attachMachine(machine->controlLink());
      machine->markGovernorAttached();
      devices.push_back(std::move(g));
    } else if (iequals(comp.type, "exciter") || iequals(comp.type, "avr")) {
      auto e = make_exciter(comp, mp);
      e->attachMachine(machine->controlLink());
      machine->markExciterAttached();
      exciter = e.get();
      devices.push_back(std::move(e));
    } else if (iequals(comp.type, "pss") || iequals(comp.type, "stabilizer")) {
      auto s = make_pss(comp, mp);
      s->attachMachine(machine->controlLink());
      pss = s.get();
      has_pss = true;
      devices.push_back(std::move(s));
    }
  }
  if (has_pss && exciter != nullptr) {
    exciter->setPSS(pss->pssLink());
  } else if (has_pss) {
    warnings.push_back(
        "PSS on machine " + std::to_string(mp.component_index) +
        " has no exciter/AVR to feed; its stabilizing signal will not affect the machine");
  }
}

DynamicLoadModelKind load_model_kind_from_profile(
    const Load& load,
    const hacdcpf::DynamicModelProfile& profile) {
  if (!profile.empty()) {
    if (iequals(profile.model_name, "ConstantPowerLoad") ||
        iequals(profile.model_name, "ConstantPower") ||
        iequals(profile.model_name, "PowerLoad")) {
      return DynamicLoadModelKind::ConstantPower;
    }
    if (iequals(profile.model_name, "ConstantCurrentLoad") ||
        iequals(profile.model_name, "ConstantCurrent")) {
      return DynamicLoadModelKind::ConstantCurrent;
    }
    if (iequals(profile.model_name, "ConstantImpedanceLoad") ||
        iequals(profile.model_name, "ConstantImpedance")) {
      return DynamicLoadModelKind::ConstantImpedance;
    }
    if (iequals(profile.model_name, "ZIP") ||
        iequals(profile.model_name, "ZIPLoad") ||
        iequals(profile.model_name, "StandardLoad")) {
      return DynamicLoadModelKind::ZIP;
    }
  }
  return load.model == LoadModel::ConstantPower ? DynamicLoadModelKind::ConstantPower
                                                : DynamicLoadModelKind::ZIP;
}

double normalized_percent(double value, double fallback) {
  if (!std::isfinite(value)) return fallback;
  return value > 1.0 ? value / 100.0 : value;
}

void apply_load_profile(const Load& load, ACLoadDynamicParams& params) {
  params.model_profiles = to_dynamic_profiles(load.dynamic_model);
  params.model_kind = load_model_kind_from_profile(load, load.dynamic_model);
  const double zp = normalized_percent(load.z_percent_p, 0.0);
  const double ip = normalized_percent(load.i_percent_p, 0.0);
  const double pp = normalized_percent(load.p_percent_p, 1.0);
  const double zq = normalized_percent(load.z_percent_q, 0.0);
  const double iq = normalized_percent(load.i_percent_q, 0.0);
  const double pq = normalized_percent(load.p_percent_q, 1.0);
  const double sum_p = zp + ip + pp;
  const double sum_q = zq + iq + pq;
  params.z_weight_p = sum_p > 0.0 ? zp / sum_p : 0.0;
  params.i_weight_p = sum_p > 0.0 ? ip / sum_p : 0.0;
  params.p_weight_p = sum_p > 0.0 ? pp / sum_p : 1.0;
  params.z_weight_q = sum_q > 0.0 ? zq / sum_q : 0.0;
  params.i_weight_q = sum_q > 0.0 ? iq / sum_q : 0.0;
  params.p_weight_q = sum_q > 0.0 ? pq / sum_q : 1.0;
}

void apply_battery_profile(const hacdcpf::DynamicModelProfile& profile,
                           BatteryDynamicParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  if (profile.empty()) return;
  const auto& p = profile.parameters;
  params.response_t_s = param_or(p, {"response_t_s", "Tp", "tp"}, params.response_t_s);
  params.e_rated_mwh = param_or(p, {"e_rated_mwh", "Erated"}, params.e_rated_mwh);
  params.soc_init = param_or(p, {"soc_init", "SOC0", "soc0"}, params.soc_init);
  params.soc_min = param_or(p, {"soc_min", "SOCmin"}, params.soc_min);
  params.soc_max = param_or(p, {"soc_max", "SOCmax"}, params.soc_max);
  params.eta_charge = param_or(p, {"eta_charge", "eta_c"}, params.eta_charge);
  params.eta_discharge = param_or(p, {"eta_discharge", "eta_d"}, params.eta_discharge);
  params.self_discharge_pct_per_h =
      param_or(p, {"self_discharge_pct_per_h"}, params.self_discharge_pct_per_h);
}

void apply_dcdc_profile(const hacdcpf::DynamicModelProfile& profile,
                        DCDCConverterDynamicParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  if (profile.empty()) return;
  const auto& p = profile.parameters;
  params.eta = param_or(p, {"eta", "efficiency"}, params.eta);
  params.response_t_s = param_or(p, {"response_t_s", "Tp", "tp"}, params.response_t_s);
}

std::pair<double, double> finite_range(const std::vector<double>& values) {
  double lo = std::numeric_limits<double>::infinity();
  double hi = -std::numeric_limits<double>::infinity();
  for (const double value : values) {
    if (!std::isfinite(value)) continue;
    lo = std::min(lo, value);
    hi = std::max(hi, value);
  }
  if (!std::isfinite(lo) || !std::isfinite(hi)) return {0.0, 0.0};
  return {lo, hi};
}

DynamicInitializationSummary make_initialization_summary(
    const DynamicSolverOptions& options,
    const PowerFlowResult& pf) {
  DynamicInitializationSummary summary;
  summary.power_flow_requested = options.run_power_flow_initialization;
  summary.power_flow_converged = pf.converged;
  summary.fallback_voltage_setpoints = options.run_power_flow_initialization && !pf.converged;
  summary.iterations = pf.iterations;
  summary.residual = pf.residual;
  const auto [min_ac, max_ac] = finite_range(pf.vm);
  const auto [min_dc, max_dc] = finite_range(pf.vdc);
  summary.min_ac_voltage_pu = min_ac;
  summary.max_ac_voltage_pu = max_ac;
  summary.min_dc_voltage_pu = min_dc;
  summary.max_dc_voltage_pu = max_dc;
  summary.warnings = pf.diagnostics.warnings;
  return summary;
}

bool nearly_equal(double lhs, double rhs) {
  return std::abs(lhs - rhs) <= 1e-9 * std::max({1.0, std::abs(lhs), std::abs(rhs)});
}

bool is_projected_asymmetric_load_equivalent(
    const Load& load,
    const std::vector<AsymmetricLoad>& asymmetric_loads) {
  for (const auto& source : asymmetric_loads) {
    if (!source.in_service || source.bus == 0 || source.bus != load.bus) continue;
    const std::string expected_name =
        source.name.empty() ? "AsymmetricLoad_" + std::to_string(source.index)
                            : source.name + "_eq";
    if (load.name != expected_name) continue;

    const double scale = scale_or_one(source.scaling);
    const double p_mw = scale * (source.pa_mw + source.pb_mw + source.pc_mw);
    const double q_mvar = scale * (source.qa_mvar + source.qb_mvar + source.qc_mvar);
    if (nearly_equal(load.p_mw, p_mw) && nearly_equal(load.q_mvar, q_mvar)) {
      return true;
    }
  }
  return false;
}

Eigen::Matrix3cd sequence_to_phase_matrix(Complex z0, Complex z1, Complex z2) {
  const Complex a(-0.5, std::sqrt(3.0) / 2.0);
  Eigen::Matrix3cd A;
  A << Complex(1.0, 0.0), Complex(1.0, 0.0), Complex(1.0, 0.0),
       Complex(1.0, 0.0), a * a, a,
       Complex(1.0, 0.0), a, a * a;
  const Eigen::Matrix3cd Z012 =
      (Eigen::Vector3cd() << z0, z1, z2).finished().asDiagonal();
  return A * Z012 * A.inverse();
}

Eigen::Matrix3cd three_phase_line_impedance(const ThreePhaseACLine& line,
                                            double min_z) {
  if (line.use_phase_matrix) {
    Eigen::Matrix3cd z = Eigen::Matrix3cd::Zero();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        z(r, c) = Complex(phase_matrix_get(line.r_matrix_pu, r, c),
                          phase_matrix_get(line.x_matrix_pu, r, c));
      }
    }
    for (int p = 0; p < 3; ++p) {
      if (std::abs(z(p, p)) < min_z) z(p, p) = Complex(min_z, min_z);
    }
    return z;
  }

  Complex z1(line.r1_pu, line.x1_pu);
  Complex z0(line.r0_pu, line.x0_pu);
  if (std::abs(z1) < min_z) z1 = Complex(min_z, min_z);
  if (std::abs(z0) < min_z) z0 = z1;
  return sequence_to_phase_matrix(z0, z1, z1);
}

Eigen::Matrix3cd three_phase_line_shunt_half(const ThreePhaseACLine& line) {
  Eigen::Matrix3cd y = Eigen::Matrix3cd::Zero();
  if (line.use_phase_matrix) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        y(r, c) = Complex(0.0, phase_matrix_get(line.b_matrix_pu, r, c) / 2.0);
      }
    }
    return y;
  }
  const Complex y_half(0.0, line.b1_pu / 2.0);
  y(0, 0) = y_half;
  y(1, 1) = y_half;
  y(2, 2) = y_half;
  return y;
}

Eigen::Matrix3cd diag3(Complex value) {
  Eigen::Matrix3cd y = Eigen::Matrix3cd::Zero();
  y(0, 0) = value;
  y(1, 1) = value;
  y(2, 2) = value;
  return y;
}

DynamicACBranch make_balanced_ac_branch(const ACBranch& branch,
                                        const DynamicNetwork& network,
                                        double min_z,
                                        bool per_phase_equivalent) {
  DynamicACBranch dyn;
  dyn.index = branch.index;
  dyn.from_bus = branch.from_bus;
  dyn.to_bus = branch.to_bus;
  dyn.from_pos = network.acBusPosition(branch.from_bus);
  dyn.to_pos = network.acBusPosition(branch.to_bus);
  dyn.in_service = branch.in_service;
  Complex z(branch.r_pu, branch.x_pu);
  if (std::abs(z) < min_z) z = Complex(min_z, min_z);
  const double scale = per_phase_equivalent ? 1.0 / 3.0 : 1.0;
  const Complex y = (Complex(1.0, 0.0) / z) * scale;
  const Complex y_shunt(0.0, branch.b_pu / 2.0 * scale);
  const double tap_mag = branch.tap == 0.0 ? 1.0 : branch.tap;
  const Complex tap = std::polar(tap_mag, branch.shift_deg * kDegToRad);
  const Complex tap_conj = std::conj(tap);
  dyn.y_ff = diag3((y + y_shunt) / (tap * tap_conj));
  dyn.y_ft = diag3(-y / tap_conj);
  dyn.y_tf = diag3(-y / tap);
  dyn.y_tt = diag3(y + y_shunt);
  return dyn;
}

DynamicACBranch make_three_phase_line_branch(const ThreePhaseACLine& line,
                                             const DynamicNetwork& network,
                                             double min_z) {
  DynamicACBranch dyn;
  dyn.index = line.index;
  dyn.from_bus = line.from_bus;
  dyn.to_bus = line.to_bus;
  dyn.from_pos = network.acBusPosition(line.from_bus);
  dyn.to_pos = network.acBusPosition(line.to_bus);
  dyn.in_service = line.in_service;
  const Eigen::Matrix3cd z = three_phase_line_impedance(line, min_z);
  const Eigen::Matrix3cd y = z.inverse();
  const Eigen::Matrix3cd y_sh = three_phase_line_shunt_half(line);
  dyn.y_ff = y + y_sh;
  dyn.y_ft = -y;
  dyn.y_tf = -y;
  dyn.y_tt = y + y_sh;

  for (int phase = 0; phase < 3; ++phase) {
    if (line.phase_mask.has(phase)) continue;
    dyn.y_ff.row(phase).setZero();
    dyn.y_ff.col(phase).setZero();
    dyn.y_ft.row(phase).setZero();
    dyn.y_ft.col(phase).setZero();
    dyn.y_tf.row(phase).setZero();
    dyn.y_tf.col(phase).setZero();
    dyn.y_tt.row(phase).setZero();
    dyn.y_tt.col(phase).setZero();
  }
  return dyn;
}

DynamicACBranch make_three_phase_transformer_branch(const ThreePhaseTransformer& tr,
                                                    const DynamicNetwork& network,
                                                    double min_z) {
  DynamicACBranch dyn;
  dyn.index = tr.index;
  dyn.from_bus = tr.hv_bus;
  dyn.to_bus = tr.lv_bus;
  dyn.from_pos = network.acBusPosition(tr.hv_bus);
  dyn.to_pos = network.acBusPosition(tr.lv_bus);
  dyn.in_service = tr.in_service;
  const double z_base = positive_or(tr.sn_mva, network.base_mva);
  double r = tr.vkr_percent / 100.0 * network.base_mva / z_base;
  double zmag = tr.vk_percent / 100.0 * network.base_mva / z_base;
  if (zmag <= 0.0) zmag = min_z;
  if (r < 0.0) r = 0.0;
  const double x = std::sqrt(std::max(0.0, zmag * zmag - r * r));
  const Complex z(std::max(r, min_z), std::max(x, min_z));
  const Complex y = Complex(1.0, 0.0) / z;
  dyn.y_ff = diag3(y);
  dyn.y_ft = diag3(-y);
  dyn.y_tf = diag3(-y);
  dyn.y_tt = diag3(y);
  return dyn;
}

DynamicDCBranch make_dc_branch(const DCBranch& branch,
                               const DynamicNetwork& network,
                               double min_r) {
  DynamicDCBranch dyn;
  dyn.index = branch.index;
  dyn.from_bus = branch.from_bus;
  dyn.to_bus = branch.to_bus;
  dyn.from_pos = network.dcBusPosition(branch.from_bus);
  dyn.to_pos = network.dcBusPosition(branch.to_bus);
  dyn.in_service = branch.in_service;
  dyn.conductance_pu = 1.0 / std::max(min_r, std::abs(branch.r_pu));
  return dyn;
}

PowerFlowResult nominal_power_flow(const HybridPowerSystem& sys,
                                   const DynamicSolverOptions& options) {
  if (options.run_power_flow_initialization) {
    PowerFlowResult pf = solve_power_flow(sys, options.power_flow_options);
    if (pf.converged) return pf;
    pf.diagnostics.warnings.push_back(
        "Transient initialization used model voltage setpoints because static power flow did not converge");
    if (!pf.vm.empty() || !pf.vdc.empty()) return pf;
  }

  PowerFlowResult pf;
  pf.converged = !options.run_power_flow_initialization;
  pf.vm.reserve(sys.ac.buses.size());
  pf.va.reserve(sys.ac.buses.size());
  for (const auto& bus : sys.ac.buses) {
    pf.vm.push_back(bus.vm_pu > 0.0 ? bus.vm_pu : 1.0);
    pf.va.push_back(bus.va_deg * kDegToRad);
  }
  pf.vdc.reserve(sys.dc.buses.size());
  for (const auto& bus : sys.dc.buses) {
    pf.vdc.push_back(bus.vm_pu > 0.0 ? bus.vm_pu : 1.0);
  }
  return pf;
}

void initialize_network_voltages(const HybridPowerSystem& sys,
                                 const PowerFlowResult& pf,
                                 DynamicSystem& dyn) {
  dyn.y.resize(dyn.network.acPhaseNodeCount(), dyn.network.dcBusCount());
  for (int pos = 0; pos < static_cast<int>(dyn.network.ac_bus_ids.size()); ++pos) {
    double vm = 1.0;
    double va = 0.0;
    if (pos < static_cast<int>(pf.vm.size())) vm = pf.vm[static_cast<std::size_t>(pos)];
    if (pos < static_cast<int>(pf.va.size())) va = pf.va[static_cast<std::size_t>(pos)];
    const auto it = dyn.network.ac_bus_pos_by_id.find(dyn.network.ac_bus_ids[static_cast<std::size_t>(pos)]);
    (void)it;
    dyn.y.Vac_abc[3 * pos + 0] = std::polar(vm, va);
    dyn.y.Vac_abc[3 * pos + 1] = std::polar(vm, va - 2.0 * kPi / 3.0);
    dyn.y.Vac_abc[3 * pos + 2] = std::polar(vm, va + 2.0 * kPi / 3.0);
  }

  if (sys.three_phase_ac) {
    for (const auto& bus : sys.three_phase_ac->buses) {
      const int pos = dyn.network.acBusPosition(bus.index);
      if (pos < 0) continue;
      dyn.y.Vac_abc[3 * pos + 0] = std::polar(bus.vm_a_pu, bus.va_a_deg * kDegToRad);
      dyn.y.Vac_abc[3 * pos + 1] = std::polar(bus.vm_b_pu, bus.va_b_deg * kDegToRad);
      dyn.y.Vac_abc[3 * pos + 2] = std::polar(bus.vm_c_pu, bus.va_c_deg * kDegToRad);
    }
  }

  for (int pos = 0; pos < static_cast<int>(dyn.network.dc_bus_ids.size()); ++pos) {
    double v = 1.0;
    if (pos < static_cast<int>(pf.vdc.size())) v = pf.vdc[static_cast<std::size_t>(pos)];
    dyn.y.Vdc[pos] = v > 0.0 ? v : 1.0;
  }
}

std::unordered_map<int, Complex> ac_device_injection_from_network_pf(
    const DynamicNetwork& network,
    const NetworkState& y) {
  std::unordered_map<int, Complex> injections;
  if (network.acPhaseNodeCount() == 0 || y.Vac_abc.size() != network.acPhaseNodeCount()) {
    return injections;
  }
  const Eigen::VectorXcd currents = network.Yac_base * y.Vac_abc;
  for (int pos = 0; pos < static_cast<int>(network.ac_bus_ids.size()); ++pos) {
    Complex s{0.0, 0.0};
    for (int phase = 0; phase < 3; ++phase) {
      const int node = 3 * pos + phase;
      s += y.Vac_abc[node] * std::conj(currents[node]);
    }
    injections[network.ac_bus_ids[static_cast<std::size_t>(pos)]] =
        s * network.base_mva;
  }
  return injections;
}

std::unordered_map<int, Complex> explicit_ac_load_power_by_bus(
    const HybridPowerSystem& sys) {
  std::unordered_map<int, Complex> load_power;
  for (const auto& load : sys.ac.loads) {
    if (!load.in_service) continue;
    const double scale = scale_or_one(load.scaling);
    load_power[load.bus] += Complex(load.p_mw * scale, load.q_mvar * scale);
  }
  return load_power;
}

bool use_pf_bus_injection_machine_init(const HybridPowerSystem& sys) {
  const bool has_machine_dynamic_profiles =
      std::any_of(sys.ac.generators.begin(),
                  sys.ac.generators.end(),
                  [](const Generator& gen) {
                    return gen.in_service && !gen.dynamic_model.empty();
                  }) ||
      std::any_of(sys.ac.external_grids.begin(),
                  sys.ac.external_grids.end(),
                  [](const ExternalGrid& grid) {
                    return grid.in_service && !grid.dynamic_model.empty();
                  });
  return sys.dc.buses.empty() &&
         sys.vsc_converters.empty() &&
         sys.ac.static_generators.empty() &&
         sys.ac.pv_systems.empty() &&
         sys.ac.renewable_gens.empty() &&
         sys.ac.storage.empty() &&
         !has_machine_dynamic_profiles;
}

}  // namespace

DynamicSystem DynamicModelBuilder::build(const HybridPowerSystem& sys,
                                         const DynamicSolverOptions& options) const {
  DynamicSystem dyn;
  dyn.options = options;
  dyn.canonical_system = options.project_to_canonical ? project_to_canonical_models(sys) : sys;
  dyn.initial_power_flow = nominal_power_flow(
      options.project_to_canonical ? sys : dyn.canonical_system,
      options);
  dyn.initialization = make_initialization_summary(options, dyn.initial_power_flow);

  auto& network = dyn.network;
  network.base_mva = positive_or(dyn.canonical_system.base_mva, 100.0);
  network.frequency_hz = positive_or(dyn.canonical_system.ac.freq_hz, 50.0);

  if (dyn.canonical_system.three_phase_ac &&
      !dyn.canonical_system.three_phase_ac->buses.empty()) {
    network.frequency_hz =
        positive_or(dyn.canonical_system.three_phase_ac->base_freq_hz, network.frequency_hz);
    for (const auto& bus : dyn.canonical_system.three_phase_ac->buses) {
      if (!bus.in_service) continue;
      network.ac_bus_pos_by_id[bus.index] = static_cast<int>(network.ac_bus_ids.size());
      network.ac_bus_ids.push_back(bus.index);
    }
  } else if (options.synthesize_three_phase_if_absent) {
    for (const auto& bus : dyn.canonical_system.ac.buses) {
      if (!bus.in_service || bus.bus_type == BusType::ISOLATED) continue;
      network.ac_bus_pos_by_id[bus.index] = static_cast<int>(network.ac_bus_ids.size());
      network.ac_bus_ids.push_back(bus.index);
    }
  }

  for (const auto& bus : dyn.canonical_system.dc.buses) {
    if (!bus.in_service || bus.bus_type == DCBusType::DC_ISOLATED) continue;
    network.dc_bus_pos_by_id[bus.index] = static_cast<int>(network.dc_bus_ids.size());
    network.dc_bus_ids.push_back(bus.index);
  }

  const bool pf_bus_injection_init =
      use_pf_bus_injection_machine_init(dyn.canonical_system);

  for (const auto& bus : dyn.canonical_system.ac.buses) {
    if (!bus.in_service || bus.bus_type == BusType::ISOLATED) continue;
    const int bus_pos = network.acBusPosition(bus.index);
    if (bus_pos < 0) continue;
    if (std::abs(bus.pd_mw) <= 1e-12 && std::abs(bus.qd_mvar) <= 1e-12) continue;
    DynamicACBusLoad load;
    load.bus = bus.index;
    load.bus_pos = bus_pos;
    load.p_mw = bus.pd_mw;
    load.q_mvar = bus.qd_mvar;
    if (bus_pos >= 0 && bus_pos < static_cast<int>(dyn.initial_power_flow.vm.size())) {
      load.nominal_voltage_pu =
          positive_or(dyn.initial_power_flow.vm[static_cast<std::size_t>(bus_pos)], 1.0);
    } else {
      load.nominal_voltage_pu = positive_or(bus.vm_pu, 1.0);
    }
    network.ac_bus_loads.push_back(load);
  }

  for (const auto& bus : dyn.canonical_system.dc.buses) {
    if (!bus.in_service || bus.bus_type == DCBusType::DC_ISOLATED) continue;
    const int bus_pos = network.dcBusPosition(bus.index);
    if (bus_pos < 0 || std::abs(bus.pd_mw) <= 1e-12) continue;
    DynamicDCBusLoad load;
    load.bus = bus.index;
    load.bus_pos = bus_pos;
    load.p_mw = bus.pd_mw;
    network.dc_bus_loads.push_back(load);
  }

  if (dyn.canonical_system.three_phase_ac &&
      !dyn.canonical_system.three_phase_ac->buses.empty()) {
    for (const auto& line : dyn.canonical_system.three_phase_ac->lines) {
      network.ac_branches.push_back(
          make_three_phase_line_branch(line, network, options.min_branch_impedance_pu));
    }
    for (const auto& tr : dyn.canonical_system.three_phase_ac->transformers) {
      network.ac_branches.push_back(
          make_three_phase_transformer_branch(tr, network, options.min_branch_impedance_pu));
    }
  } else {
    for (const auto& branch : dyn.canonical_system.ac.branches) {
      network.ac_branches.push_back(
          make_balanced_ac_branch(branch,
                                  network,
                                  options.min_branch_impedance_pu,
                                  pf_bus_injection_init));
    }
  }

  for (const auto& branch : dyn.canonical_system.dc.branches) {
    network.dc_branches.push_back(make_dc_branch(branch, network, options.min_branch_impedance_pu));
  }
  network.rebuildBaseMatrices(options.singular_regularization_pu);
  initialize_network_voltages(dyn.canonical_system, dyn.initial_power_flow, dyn);
  auto pf_ac_device_injection_mva =
      pf_bus_injection_init ? ac_device_injection_from_network_pf(network, dyn.y)
                            : std::unordered_map<int, Complex>{};
  if (pf_bus_injection_init) {
    for (const auto& [bus, load_power] :
         explicit_ac_load_power_by_bus(dyn.canonical_system)) {
      pf_ac_device_injection_mva[bus] += load_power;
    }
  }

  const double base_mva = network.base_mva;

  std::unordered_set<int> explicit_dc_voltage_source_buses;
  for (const auto& conv : dyn.canonical_system.vsc_converters) {
    if (!conv.in_service) continue;
    const DeviceControlRole role = resolve_device_control_role(conv);
    if (role.is_dc_grid_forming && network.dcBusPosition(conv.bus_dc) >= 0) {
      explicit_dc_voltage_source_buses.insert(conv.bus_dc);
    }
  }

  for (const auto& bus : dyn.canonical_system.dc.buses) {
    if (!bus.in_service || bus.bus_type != DCBusType::DC_V) continue;
    const int bus_pos = network.dcBusPosition(bus.index);
    if (bus_pos < 0) continue;
    if (explicit_dc_voltage_source_buses.count(bus.index) != 0) {
      dyn.warnings.push_back(
          "DC_V bus " + std::to_string(bus.index) +
          " also has an explicit DC-forming converter; using the converter's "
          "dynamic voltage reference");
      continue;
    }
    DCVoltageSourceDynamicParams p;
    p.component_index = bus.index;
    p.bus = bus.index;
    p.bus_pos = bus_pos;
    p.label = label_or(bus.name, "DC voltage bus " + std::to_string(bus.index));
    p.canvas_type = "dc";
    p.source_type = "dc_bus_voltage";
    p.v_ref_pu = positive_or(bus.vm_pu, 1.0);
    p.conductance_pu = options.source_stiffness_pu;
    dyn.devices.push_back(std::make_unique<DCVoltageSourceDynamic>(p));
  }

  std::unordered_set<int> voltage_source_buses;
  const bool using_explicit_three_phase =
      dyn.canonical_system.three_phase_ac &&
      !dyn.canonical_system.three_phase_ac->buses.empty();

  if (using_explicit_three_phase) {
    for (const auto& grid : dyn.canonical_system.three_phase_ac->external_grids) {
      if (!grid.in_service) continue;
      const int bus_pos = network.acBusPosition(grid.bus);
      if (bus_pos < 0) continue;
      VoltageSourceDynamicParams p;
      p.component_index = grid.index;
      p.bus = grid.bus;
      p.bus_pos = bus_pos;
      p.label = label_or(grid.name, "Three-phase external grid " + std::to_string(grid.index));
      p.device_type = "ThreePhaseExternalGrid";
      p.canvas_type = "extGrid";
      p.source_type = "three_phase_external_grid";
      p.base_mva = base_mva;
      p.vm_set_pu = positive_or(grid.vm_pu, 1.0);
      p.angle_set_rad = grid.va_deg * kDegToRad;
      p.frequency_hz = network.frequency_hz;
      p.r_pu = grid.r1_pu;
      p.x_pu = positive_or(grid.x1_pu, 1.0 / std::max(1.0, options.source_stiffness_pu));
      p.in_service = true;
      apply_voltage_source_profile(grid.dynamic_model, p);
      dyn.devices.push_back(std::make_unique<SynchronousMachine>(p));
      voltage_source_buses.insert(grid.bus);
    }

    for (const auto& gen : dyn.canonical_system.three_phase_ac->generators) {
      if (!gen.in_service) continue;
      const int bus_pos = network.acBusPosition(gen.bus);
      if (bus_pos < 0) continue;
      if (gen.is_slack || gen.xdpp_pu > 0.0 || gen.xd_pu > 0.0) {
        VoltageSourceDynamicParams p;
        p.component_index = gen.index;
        p.bus = gen.bus;
        p.bus_pos = bus_pos;
        p.label = label_or(gen.name, "Three-phase generator " + std::to_string(gen.index));
        p.device_type = "ThreePhaseGenerator";
        p.canvas_type = "gen";
        p.source_type = "three_phase_generator";
        p.base_mva = base_mva;
        p.vm_set_pu = positive_or(gen.vm_pu, 1.0);
        p.frequency_hz = network.frequency_hz;
        p.x_pu = positive_or(gen.xdpp_pu, positive_or(gen.xd_pu, 0.2));
        p.p_mech_mw = gen.p_mw;
        p.q_elec_mvar = gen.q_mvar;
        p.xd_pu = gen.xd_pu;
        p.xdpp_pu = gen.xdpp_pu;
        p.inertia_h = gen.is_slack ? 5.0 : 1.0;
        p.dynamic_angle = !gen.is_slack;
        apply_voltage_source_profile(gen.dynamic_model, p);
        auto machine = std::make_unique<SynchronousMachine>(p);
        SynchronousMachine* machine_ptr = machine.get();
        dyn.devices.push_back(std::move(machine));
        wire_machine_controllers(dyn.devices, machine_ptr, p, gen.dynamic_model, dyn.warnings);
        voltage_source_buses.insert(gen.bus);
      } else {
        GridFollowingInverterParams p;
        p.component_index = gen.index;
        p.bus = gen.bus;
        p.bus_pos = bus_pos;
        p.label = label_or(gen.name, "Three-phase generator " + std::to_string(gen.index));
        p.device_type = "ThreePhaseGenerator";
        p.canvas_type = "gen";
        p.source_type = "three_phase_generator_gfl";
        p.base_mva = base_mva;
        p.p_ref_mw = gen.p_mw;
        p.q_ref_mvar = gen.q_mvar;
        p.f_ref_hz = network.frequency_hz;
        apply_gfl_profile(gen.dynamic_model, p);
        dyn.devices.push_back(std::make_unique<GridFollowingInverter>(p));
      }
    }

    for (const auto& load : dyn.canonical_system.three_phase_ac->loads) {
      if (!load.in_service) continue;
      const int bus_pos = network.acBusPosition(load.bus);
      if (bus_pos < 0) continue;
      ThreePhaseLoadDynamicParams p;
      p.component_index = load.index;
      p.bus = load.bus;
      p.bus_pos = bus_pos;
      p.label = label_or(load.name, "Three-phase load " + std::to_string(load.index));
      p.canvas_type = "load";
      p.source_type = "three_phase_load";
      p.p_mw = {load.p_a_mw, load.p_b_mw, load.p_c_mw};
      p.q_mvar = {load.q_a_mvar, load.q_b_mvar, load.q_c_mvar};
      p.phase_active = {load.phase_mask.has(0), load.phase_mask.has(1), load.phase_mask.has(2)};
      p.base_mva = base_mva;
      p.model_profiles = to_dynamic_profiles(load.dynamic_model);
      dyn.devices.push_back(std::make_unique<ThreePhaseDynamicLoad>(p));
    }
  }

  for (const auto& grid : dyn.canonical_system.ac.external_grids) {
    if (using_explicit_three_phase) break;
    if (!grid.in_service) continue;
    const int bus_pos = network.acBusPosition(grid.bus);
    if (bus_pos < 0) continue;
    VoltageSourceDynamicParams p;
    p.component_index = grid.index;
    p.bus = grid.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(grid.name, "External grid " + std::to_string(grid.index));
    p.device_type = "ExternalGrid";
    p.canvas_type = "extGrid";
    p.source_type = "external_grid";
    p.base_mva = base_mva;
    p.vm_set_pu = positive_or(grid.vm_pu, 1.0);
    p.angle_set_rad = grid.va_deg * kDegToRad;
    p.frequency_hz = network.frequency_hz;
    p.r_pu = grid.r_pu;
    p.x_pu = positive_or(grid.x_pu, 1.0 / std::max(1.0, options.source_stiffness_pu));
    p.in_service = true;
    apply_voltage_source_profile(grid.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<SynchronousMachine>(p));
    voltage_source_buses.insert(grid.bus);
  }

  for (const auto& gen : dyn.canonical_system.ac.generators) {
    if (using_explicit_three_phase) break;
    if (!gen.in_service) continue;
    const int bus_pos = network.acBusPosition(gen.bus);
    if (bus_pos < 0) continue;
    VoltageSourceDynamicParams p;
    p.component_index = gen.index;
    p.bus = gen.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(gen.name, "Generator " + std::to_string(gen.index));
    p.device_type = "SynchronousMachine";
    p.canvas_type = "gen";
    p.source_type = "generator";
    p.base_mva = base_mva;
    p.vm_set_pu = positive_or(gen.vg_pu, 1.0);
    p.frequency_hz = network.frequency_hz;
    p.r_pu = gen.ra_pu;
    p.x_pu = positive_or(gen.xdpp_pu, positive_or(gen.xdp_pu, positive_or(gen.xd_pu, 0.2)));
    p.p_mech_mw = gen.pg_mw;
    p.q_elec_mvar = gen.qg_mvar;
    if (pf_bus_injection_init &&
        std::count_if(dyn.canonical_system.ac.generators.begin(),
                      dyn.canonical_system.ac.generators.end(),
                      [&](const Generator& other) {
                        return other.in_service && other.bus == gen.bus;
                      }) == 1) {
      const auto inj = pf_ac_device_injection_mva.find(gen.bus);
      if (inj != pf_ac_device_injection_mva.end()) {
        p.p_mech_mw = inj->second.real();
        p.q_elec_mvar = inj->second.imag();
      }
    }
    p.xd_pu = gen.xd_pu;
    p.xq_pu = gen.xq_pu;
    p.xdp_pu = gen.xdp_pu;
    p.xdpp_pu = gen.xdpp_pu;
    p.td0p_s = gen.td0p_s;
    p.td0pp_s = gen.td0pp_s;
    p.inertia_h = positive_or(gen.inertia_h, gen.is_slack ? 5.0 : 1.0);
    p.droop_r = positive_or(gen.droop_r, 0.05);
    p.dynamic_angle = !gen.is_slack;
    apply_voltage_source_profile(gen.dynamic_model, p);
    auto machine = std::make_unique<SynchronousMachine>(p);
    SynchronousMachine* machine_ptr = machine.get();
    dyn.devices.push_back(std::move(machine));
    wire_machine_controllers(dyn.devices, machine_ptr, p, gen.dynamic_model, dyn.warnings);
    voltage_source_buses.insert(gen.bus);
  }

  for (const auto& load : dyn.canonical_system.ac.loads) {
    if (using_explicit_three_phase) break;
    if (!load.in_service) continue;
    if (is_projected_asymmetric_load_equivalent(
            load,
            dyn.canonical_system.ac.asymmetric_loads)) {
      continue;
    }
    const int bus_pos = network.acBusPosition(load.bus);
    if (bus_pos < 0) continue;
    ACLoadDynamicParams p;
    p.component_index = load.index;
    p.bus = load.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(load.name, "AC load " + std::to_string(load.index));
    p.canvas_type = "load";
    p.source_type = "ac_load";
    p.p_mw = load.p_mw * load.scaling;
    p.q_mvar = load.q_mvar * load.scaling;
    p.base_mva = base_mva;
    if (bus_pos >= 0 && bus_pos < static_cast<int>(dyn.initial_power_flow.vm.size())) {
      p.nominal_voltage_pu = positive_or(
          dyn.initial_power_flow.vm[static_cast<std::size_t>(bus_pos)],
          1.0);
    }
    apply_load_profile(load, p);
    dyn.devices.push_back(std::make_unique<DynamicLoad>(p));
  }

  for (const auto& load : dyn.canonical_system.ac.asymmetric_loads) {
    if (using_explicit_three_phase) break;
    if (!load.in_service) continue;
    const int bus_pos = network.acBusPosition(load.bus);
    if (bus_pos < 0) continue;
    ThreePhaseLoadDynamicParams p;
    p.component_index = load.index;
    p.bus = load.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(load.name, "Asymmetric load " + std::to_string(load.index));
    p.canvas_type = "asymLoad";
    p.source_type = "asymmetric_load";
    p.p_mw = {load.pa_mw, load.pb_mw, load.pc_mw};
    p.q_mvar = {load.qa_mvar, load.qb_mvar, load.qc_mvar};
    p.scale = scale_or_one(load.scaling);
    p.base_mva = base_mva;
    p.model_profiles = to_dynamic_profiles(load.dynamic_model);
    dyn.devices.push_back(std::make_unique<ThreePhaseDynamicLoad>(p));
  }

  for (const auto& gen : dyn.canonical_system.ac.static_generators) {
    if (!gen.in_service) continue;
    const int bus_pos = network.acBusPosition(gen.bus);
    if (bus_pos < 0) continue;
    GridFollowingInverterParams p;
    p.component_index = gen.index;
    p.bus = gen.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(gen.name, "Static generator " + std::to_string(gen.index));
    p.device_type = "StaticGenerator";
    p.canvas_type = "sgen";
    p.source_type = "static_generator";
    p.base_mva = base_mva;
    p.p_ref_mw = gen.p_mw * gen.scaling;
    p.q_ref_mvar = gen.q_mvar * gen.scaling;
    p.response_t_s = 0.05;
    p.power_filter_t_s = 0.05;
    p.f_ref_hz = positive_or(gen.f_ref_hz, network.frequency_hz);
    p.current_limit_pu = gen.sn_mva > 0.0
                             ? gen.sn_mva / base_mva
                             : (gen.p_rated_mw > 0.0 ? gen.p_rated_mw / base_mva : 0.0);
    p.v_ref_pu = positive_or(gen.v_ref_pu, 1.0);
    p.frequency_watt_droop_pu = gen.k_p;
    p.volt_var_droop_pu = gen.k_q;
    apply_gfl_profile(gen.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<GridFollowingInverter>(p));
  }

  for (const auto& pv : dyn.canonical_system.ac.pv_systems) {
    if (!pv.in_service) continue;
    const int bus_pos = network.acBusPosition(pv.bus);
    if (bus_pos < 0) continue;
    GridFollowingInverterParams p;
    p.component_index = pv.index;
    p.bus = pv.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(pv.name, "PV system " + std::to_string(pv.index));
    p.device_type = "PVSystem";
    p.canvas_type = "pv";
    p.source_type = "pv_system";
    p.base_mva = base_mva;
    p.p_ref_mw = pv.p_mw;
    p.q_ref_mvar = pv.q_mvar;
    p.eta = positive_or(pv.inverter_eff, 0.97);
    p.f_ref_hz = network.frequency_hz;
    p.v_ref_pu = positive_or(pv.v_ac_set_pu, 1.0);
    p.current_limit_pu = pv.sn_mva > 0.0
                             ? pv.sn_mva / base_mva
                             : (pv.pmax_mw > 0.0 ? pv.pmax_mw / base_mva : 0.0);
    apply_gfl_profile(pv.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<GridFollowingInverter>(p));
  }

  for (const auto& rg : dyn.canonical_system.ac.renewable_gens) {
    if (!rg.in_service) continue;
    const int bus_pos = network.acBusPosition(rg.bus);
    if (bus_pos < 0) continue;
    GridFollowingInverterParams p;
    p.component_index = rg.index;
    p.bus = rg.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(rg.name, "Renewable generator " + std::to_string(rg.index));
    p.device_type = "RenewableGen";
    p.canvas_type = "renGen";
    p.source_type = "renewable_gen";
    p.base_mva = base_mva;
    p.p_ref_mw = rg.p_mw;
    p.q_ref_mvar = rg.q_mvar;
    p.f_ref_hz = network.frequency_hz;
    p.current_limit_pu = rg.p_rated_mw > 0.0 ? rg.p_rated_mw / base_mva : 0.0;
    apply_gfl_profile(rg.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<GridFollowingInverter>(p));
  }

  for (const auto& st : dyn.canonical_system.ac.storage) {
    if (!st.in_service) continue;
    const int bus_pos = network.acBusPosition(st.bus);
    if (bus_pos < 0) continue;
    if (st.grid_forming) {
      GridFormingInverterParams p;
      p.component_index = st.index;
      p.bus = st.bus;
      p.bus_pos = bus_pos;
      p.label = label_or(st.name, "Grid-forming storage " + std::to_string(st.index));
      p.device_type = "GridFormingStorage";
      p.canvas_type = "storage";
      p.source_type = "grid_forming_storage";
      p.base_mva = base_mva;
      p.p_ref_mw = st.p_mw;
      p.q_ref_mvar = st.q_mvar;
      p.virtual_x_pu = options.inverter_virtual_reactance_pu;
      p.frequency_hz = network.frequency_hz;
      p.current_limit_pu = st.p_rated_mw > 0.0
                               ? st.p_rated_mw / base_mva
                               : (st.pmax_mw > 0.0 ? st.pmax_mw / base_mva : 0.0);
      p.pmax_mw = st.pmax_mw > 0.0 ? st.pmax_mw : st.p_rated_mw;
      p.pmin_mw = st.pmin_mw < 0.0 ? st.pmin_mw : -std::max(st.pmax_mw, st.p_rated_mw);
      p.dc_link_capacitance_s = options.dc_link_capacitance_s;
      apply_gfm_profile(st.dynamic_model, p);
      dyn.devices.push_back(std::make_unique<GridFormingInverter>(p));
      voltage_source_buses.insert(st.bus);
    }

    BatteryDynamicParams p;
    p.component_index = st.index;
    p.bus = st.bus;
    p.bus_pos = bus_pos;
    p.is_ac = true;
    p.label = label_or(st.name, "AC storage " + std::to_string(st.index));
    p.canvas_type = "storage";
    p.component_domain = "AC";
    p.source_type = "storage";
    p.base_mva = base_mva;
    p.p_ref_mw = st.p_mw;
    p.q_ref_mvar = st.grid_forming ? 0.0 : st.q_mvar;
    p.e_rated_mwh = positive_or(st.e_rated_mwh, positive_or(st.e_mwh, 1.0));
    p.soc_init = st.soc_init;
    p.soc_min = st.soc_min;
    p.soc_max = st.soc_max;
    p.eta_charge = positive_or(st.eta_charge, 0.95);
    p.eta_discharge = positive_or(st.eta_discharge, 0.95);
    p.self_discharge_pct_per_h = st.self_discharge_pct;
    p.stamp_power = !st.grid_forming;
    apply_battery_profile(st.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<BatteryDynamic>(p));
  }

  for (const auto& load : dyn.canonical_system.dc.loads) {
    if (!load.in_service) continue;
    const int bus_pos = network.dcBusPosition(load.bus);
    if (bus_pos < 0) continue;
    DCLoadDynamicParams p;
    p.component_index = load.index;
    p.bus = load.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(load.name, "DC load " + std::to_string(load.index));
    p.canvas_type = "dcLoad";
    p.source_type = "dc_load";
    p.p_mw = load.p_mw * load.scaling;
    p.base_mva = base_mva;
    p.model_profiles = to_dynamic_profiles(load.dynamic_model);
    dyn.devices.push_back(std::make_unique<DCDynamicLoad>(p));
  }

  for (const auto& gen : dyn.canonical_system.dc.dc_static_generators) {
    if (!gen.in_service) continue;
    const int bus_pos = network.dcBusPosition(gen.bus);
    if (bus_pos < 0) continue;
    BatteryDynamicParams p;
    p.component_index = gen.index;
    p.bus = gen.bus;
    p.bus_pos = bus_pos;
    p.is_ac = false;
    p.label = label_or(gen.name, "DC static generator " + std::to_string(gen.index));
    p.canvas_type = "dcSgen";
    p.component_domain = "DC";
    p.source_type = "dc_static_generator";
    p.base_mva = base_mva;
    p.p_ref_mw = gen.p_set_mw * gen.scaling;
    p.e_rated_mwh = 1e9;
    p.soc_init = 0.5;
    apply_battery_profile(gen.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<BatteryDynamic>(p));
  }

  for (const auto& pv : dyn.canonical_system.dc.pv_arrays) {
    if (!pv.in_service) continue;
    const int bus_pos = network.dcBusPosition(pv.bus);
    if (bus_pos < 0) continue;
    BatteryDynamicParams p;
    p.component_index = pv.index;
    p.bus = pv.bus;
    p.bus_pos = bus_pos;
    p.is_ac = false;
    p.label = label_or(pv.name, "DC PV array " + std::to_string(pv.index));
    p.canvas_type = "dcPv";
    p.component_domain = "DC";
    p.source_type = "dc_pv_array";
    p.base_mva = base_mva;
    p.p_ref_mw = pv.p_set_mw;
    p.e_rated_mwh = 1e9;
    p.soc_init = 0.5;
    apply_battery_profile(pv.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<BatteryDynamic>(p));
  }

  for (const auto& st : dyn.canonical_system.dc.storage) {
    if (!st.in_service) continue;
    const int bus_pos = network.dcBusPosition(st.bus);
    if (bus_pos < 0) continue;
    BatteryDynamicParams p;
    p.component_index = st.index;
    p.bus = st.bus;
    p.bus_pos = bus_pos;
    p.is_ac = false;
    p.label = label_or(st.name, "DC storage " + std::to_string(st.index));
    p.canvas_type = "dcStorage";
    p.component_domain = "DC";
    p.source_type = "dc_storage";
    p.base_mva = base_mva;
    p.p_ref_mw = st.p_mw;
    p.e_rated_mwh = positive_or(st.e_rated_mwh, positive_or(st.e_mwh, 1.0));
    p.soc_init = st.soc_init;
    p.soc_min = st.soc_min;
    p.soc_max = st.soc_max;
    p.eta_charge = positive_or(st.eta_charge, 0.95);
    p.eta_discharge = positive_or(st.eta_discharge, 0.95);
    p.self_discharge_pct_per_h = st.self_discharge_pct;
    apply_battery_profile(st.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<BatteryDynamic>(p));
  }

  for (const auto& conv : dyn.canonical_system.vsc_converters) {
    if (!conv.in_service) continue;
    const int ac_pos = network.acBusPosition(conv.bus_ac);
    const int dc_pos = network.dcBusPosition(conv.bus_dc);
    if (ac_pos < 0 && dc_pos < 0) continue;
    const DeviceControlRole role = resolve_device_control_role(conv);
    if (role.is_dc_grid_forming && dc_pos >= 0) {
      DCVoltageSourceDynamicParams p;
      p.component_index = conv.index;
      p.bus = conv.bus_dc;
      p.bus_pos = dc_pos;
      p.label = label_or(conv.name, "VSC DC voltage source " + std::to_string(conv.index));
      p.canvas_type = "vsc";
      p.component_domain = "DC";
      p.source_type = "vsc_dc_voltage_source";
      p.v_ref_pu = positive_or(conv.v_dc_set_pu, 1.0);
      p.conductance_pu = options.source_stiffness_pu;
      p.trip_on_vsc_event = true;
      dyn.devices.push_back(std::make_unique<DCVoltageSourceDynamic>(p));
    }
    if (role.is_ac_grid_forming && ac_pos >= 0) {
      VSCConverterDynamicParams p;
      p.grid_forming = true;
      p.component_index = conv.index;
      p.bus = conv.bus_ac;
      p.bus_pos = ac_pos;
      p.dc_bus_pos = dc_pos;
      p.label = label_or(conv.name, "VSC grid-forming " + std::to_string(conv.index));
      p.device_type = "VSCGridForming";
      p.canvas_type = "vsc";
      p.source_type = "vsc_grid_forming";
      p.base_mva = base_mva;
      p.p_ref_mw = conv.p_schedule_mw != 0.0 ? conv.p_schedule_mw : conv.p_set_mw;
      p.q_ref_mvar = conv.q_set_mvar;
      p.v_ref_pu = positive_or(conv.v_ac_set_pu, positive_or(conv.v_ref_pu, 1.0));
      p.angle_ref_rad = conv.v_ac_angle_set_deg * kDegToRad;
      p.virtual_r_pu = positive_or(conv.r_conv_ac_pu, 0.0);
      p.virtual_x_pu = options.inverter_virtual_reactance_pu;
      p.f_ref_hz = positive_or(conv.f_ref_hz, network.frequency_hz);
      p.current_limit_pu = conv.i_ac_max_pu;
      p.pmax_mw = conv.pmax_mw > 0.0 ? conv.pmax_mw
                                      : (conv.p_rated_mw > 0.0 ? conv.p_rated_mw : 0.0);
      p.pmin_mw = conv.pmin_mw < 0.0 ? conv.pmin_mw
                                      : (p.pmax_mw > 0.0 ? -p.pmax_mw : 0.0);
      p.p_droop_pu = positive_or(conv.k_p, p.p_droop_pu);
      p.q_droop_pu = positive_or(conv.k_q, p.q_droop_pu);
      p.eta = positive_or(conv.eta, 0.99);
      p.dc_link_capacitance_s = options.dc_link_capacitance_s;
      p.dc_link_conductance_pu = options.dc_link_coupling_conductance_pu;
      p.vdc_ref_pu = positive_or(conv.v_dc_set_pu, 1.0);
      p.dc_link_mode =
          (options.dynamic_dc_link && dc_pos >= 0)
              ? DCLinkMode::DynamicDCVoltage
              : DCLinkMode::ConstantDCVoltage;
      apply_gfm_profile(conv.dynamic_model, p);
      dyn.devices.push_back(std::make_unique<VSCConverterDynamic>(p));
      voltage_source_buses.insert(conv.bus_ac);
    } else if (ac_pos >= 0) {
      VSCConverterDynamicParams p;
      p.grid_forming = false;
      p.component_index = conv.index;
      p.bus = conv.bus_ac;
      p.bus_pos = ac_pos;
      p.dc_bus_pos = dc_pos;
      p.label = label_or(conv.name, "VSC " + std::to_string(conv.index));
      p.device_type = "VSCGridFollowing";
      p.canvas_type = "vsc";
      p.source_type = "vsc_grid_following";
      p.base_mva = base_mva;
      p.p_ref_mw = conv.p_schedule_mw != 0.0 ? conv.p_schedule_mw : conv.p_set_mw;
      p.q_ref_mvar = conv.q_set_mvar;
      p.current_limit_pu = conv.i_ac_max_pu;
      p.f_ref_hz = positive_or(conv.f_ref_hz, network.frequency_hz);
      p.v_ref_pu = positive_or(conv.v_ac_set_pu, positive_or(conv.v_ref_pu, 1.0));
      p.frequency_watt_droop_pu = conv.k_p;
      p.volt_var_droop_pu = conv.k_q;
      p.eta = positive_or(conv.eta, 0.99);
      p.stamp_dc_power = dc_pos >= 0;
      p.dc_link_capacitance_s = options.dc_link_capacitance_s;
      p.dc_link_conductance_pu = options.dc_link_coupling_conductance_pu;
      p.vdc_ref_pu = positive_or(conv.v_dc_set_pu, 1.0);
      p.dc_link_mode =
          (options.dynamic_dc_link && dc_pos >= 0)
              ? DCLinkMode::DynamicDCVoltage
              : DCLinkMode::ConstantDCVoltage;
      apply_gfl_profile(conv.dynamic_model, p);
      dyn.devices.push_back(std::make_unique<VSCConverterDynamic>(p));
    }
  }

  for (const auto& conv : dyn.canonical_system.dc.dcdc_converters) {
    if (!conv.in_service) continue;
    const int in_pos = network.dcBusPosition(conv.bus_in);
    const int out_pos = network.dcBusPosition(conv.bus_out);
    if (in_pos < 0 || out_pos < 0) continue;
    DCDCConverterDynamicParams p;
    p.component_index = conv.index;
    p.bus_in = conv.bus_in;
    p.bus_out = conv.bus_out;
    p.bus_in_pos = in_pos;
    p.bus_out_pos = out_pos;
    p.label = label_or(conv.name, "DC/DC converter " + std::to_string(conv.index));
    p.canvas_type = "dcdcConverter";
    p.source_type = "dcdc_converter";
    p.base_mva = base_mva;
    p.p_ref_mw = conv.p_ref_mw;
    p.eta = positive_or(conv.eta, 0.98);
    apply_dcdc_profile(conv.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<DCDCConverterDynamic>(p));
  }

  dyn.assignStateIndices();
  initialize_network_voltages(dyn.canonical_system, dyn.initial_power_flow, dyn);
  dyn.initializeStatesFromPowerFlow();

  if (network.acPhaseNodeCount() > 0 && voltage_source_buses.empty()) {
    dyn.warnings.push_back(
        "Transient AC network has no dynamic voltage source; algebraic solve may rely on regularization only");
  }

  for (const auto& warning : dyn.initial_power_flow.diagnostics.warnings) {
    dyn.warnings.push_back(warning);
  }
  return dyn;
}

}  // namespace hacdcpf::dynamics
