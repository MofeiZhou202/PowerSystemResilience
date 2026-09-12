#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_set>

#include <Eigen/LU>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/assembly/solver_data.hpp"
#include "hacdcpf/dynamics/devices/BasicDynamicDevices.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/gfm_norton_contract.hpp"
#include "hacdcpf/power_flow/newton_solver.hpp"
#include "hacdcpf/power_flow/island_detector.hpp"
#include "hacdcpf/projection/result_attribution.hpp"

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

bool profile_model_is(const hacdcpf::DynamicModelProfile& profile,
                      std::initializer_list<const char*> names) {
  for (const char* name : names) {
    if (iequals(profile.model_name, name)) return true;
    for (const auto& component : profile.components) {
      if (iequals(component.model, name)) return true;
    }
  }
  return false;
}

std::map<std::string, double> merged_profile_parameters(
    const hacdcpf::DynamicModelProfile& profile) {
  std::map<std::string, double> params = profile.parameters;
  for (const auto& component : profile.components) {
    for (const auto& [key, value] : component.parameters) {
      params[key] = value;
    }
  }
  return params;
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

InverterFilterKind filter_kind_from_name(const std::string& name,
                                         InverterFilterKind fallback) {
  if (iequals(name, "LCLFilter") || iequals(name, "LCFilter")) {
    return InverterFilterKind::LCL;
  }
  if (iequals(name, "RLFilter")) return InverterFilterKind::RL;
  return fallback;
}

CurrentLimiterKind limiter_kind_from_name(const std::string& name,
                                          CurrentLimiterKind fallback) {
  if (iequals(name, "PriorityOutputCurrentLimiter") ||
      iequals(name, "ReactivePriorityCurrentLimiter")) {
    return CurrentLimiterKind::ReactivePriority;
  }
  if (iequals(name, "ActivePriorityCurrentLimiter")) {
    return CurrentLimiterKind::ActivePriority;
  }
  if (iequals(name, "InstantaneousOutputCurrentLimiter")) {
    return CurrentLimiterKind::Instantaneous;
  }
  if (iequals(name, "SaturationOutputCurrentLimiter")) {
    return CurrentLimiterKind::Saturation;
  }
  if (iequals(name, "HybridOutputCurrentLimiter")) {
    return CurrentLimiterKind::Hybrid;
  }
  if (iequals(name, "MagnitudeOutputCurrentLimiter")) {
    return CurrentLimiterKind::Magnitude;
  }
  return fallback;
}

template <typename P>
void apply_inverter_filter_and_limiter_params(const std::map<std::string, double>& p,
                                              P& params) {
  params.current_limit_pu =
      param_or(p, {"current_limit_pu", "Imax", "imax_pu"}, params.current_limit_pu);
  params.filter_c_pu =
      param_or(p, {"filter_c_pu", "Cf", "C_f", "cf"}, params.filter_c_pu);
  params.filter_grid_r_pu =
      param_or(p, {"filter_grid_r_pu", "Rg", "Rgrid"}, params.filter_grid_r_pu);
  params.filter_grid_x_pu =
      param_or(p, {"filter_grid_x_pu", "Xg", "Xgrid"}, params.filter_grid_x_pu);
  if (param_or(p, {"reactive_current_priority", "iq_priority"}, 0.0) != 0.0) {
    params.limiter_kind = CurrentLimiterKind::ReactivePriority;
    params.reactive_current_priority = true;
  }
  const double forced_block =
      param_or(p, {"converter_forced_block", "control_forced_block"},
               params.control_failure_mode == ConverterControlFailureMode::ForcedBlock
                   ? 1.0
                   : 0.0);
  if (!std::isfinite(forced_block) || (forced_block != 0.0 && forced_block != 1.0)) {
    throw std::invalid_argument("converter_forced_block must be exactly 0 or 1");
  }
  params.control_failure_mode = forced_block == 1.0
                                    ? ConverterControlFailureMode::ForcedBlock
                                    : ConverterControlFailureMode::None;
}

// Reads IEEE 1547 ride-through / trip / reconnect settings (design doc §11.7)
// from a device parameter map. Opt-in: does nothing unless an enable flag is
// present. `nominal_hz` scales the default 60 Hz frequency bands to the system
// nominal, and the reconnect timing is optionally overridable.
void apply_ieee1547_params(const std::map<std::string, double>& p,
                           IEEE1547Settings& s,
                           double nominal_hz) {
  const bool enable =
      param_or(p, {"ieee1547_enabled", "der_ridethrough", "ieee1547"},
               s.enabled ? 1.0 : 0.0) != 0.0;
  if (!enable) return;
  const int cat = static_cast<int>(
      param_or(p, {"ieee1547_category", "der_category"}, 2.0));
  const IEEE1547Category category =
      cat <= 1 ? IEEE1547Category::CategoryI
               : (cat >= 3 ? IEEE1547Category::CategoryIII
                           : IEEE1547Category::CategoryII);
  s = make_default_ieee1547(category, nominal_hz > 0.0 ? nominal_hz : 60.0);
  s.enabled = true;
  s.allow_reconnect =
      param_or(p, {"ieee1547_allow_reconnect", "der_reconnect"},
               s.allow_reconnect ? 1.0 : 0.0) != 0.0;
  s.reconnect_delay_s = param_or(
      p, {"ieee1547_reconnect_delay_s", "reconnect_delay_s"}, s.reconnect_delay_s);
  s.power_ramp_s =
      param_or(p, {"ieee1547_power_ramp_s", "power_ramp_s"}, s.power_ramp_s);
}

// Reads IEEE 1547 volt-var and frequency-watt smart-inverter settings (design
// doc §11.7) from a device parameter map into the given curve settings. Opt-in
// per function; the enabled curve supersedes the simple linear droop. Includes
// the T_qf/T_pf low-pass and slew-rate limits. `nominal_hz` sets the
// frequency-watt base.
void apply_smart_inverter_params(const std::map<std::string, double>& p,
                                 VoltVarSettings& vv,
                                 FreqWattSettings& fw,
                                 double nominal_hz) {
  if (param_or(p, {"volt_var_enabled", "voltvar", "vv_enabled"}, 0.0) != 0.0) {
    vv.enabled = true;
    vv.v1_pu = param_or(p, {"vv_v1_pu", "volt_var_v1"}, vv.v1_pu);
    vv.q1_pu = param_or(p, {"vv_q1_pu", "volt_var_q1"}, vv.q1_pu);
    vv.v2_pu = param_or(p, {"vv_v2_pu", "volt_var_v2"}, vv.v2_pu);
    vv.v3_pu = param_or(p, {"vv_v3_pu", "volt_var_v3"}, vv.v3_pu);
    vv.v4_pu = param_or(p, {"vv_v4_pu", "volt_var_v4"}, vv.v4_pu);
    vv.q4_pu = param_or(p, {"vv_q4_pu", "volt_var_q4"}, vv.q4_pu);
    vv.filter_t_s = param_or(p, {"vv_filter_t_s", "T_qf"}, vv.filter_t_s);
    vv.ramp_rate_pu_per_s =
        param_or(p, {"vv_ramp_pu_per_s", "volt_var_ramp"}, vv.ramp_rate_pu_per_s);
  }
  if (param_or(p, {"freq_watt_enabled", "freqwatt", "fw_enabled"}, 0.0) != 0.0) {
    fw.enabled = true;
    fw.nominal_frequency_hz = nominal_hz > 0.0 ? nominal_hz : 60.0;
    fw.db_over_hz = param_or(p, {"fw_db_over_hz", "freq_watt_db_over"}, fw.db_over_hz);
    fw.db_under_hz = param_or(p, {"fw_db_under_hz", "freq_watt_db_under"}, fw.db_under_hz);
    fw.droop_over = param_or(p, {"fw_droop_over", "freq_watt_droop"}, fw.droop_over);
    fw.droop_under = param_or(p, {"fw_droop_under"}, fw.droop_under);
    fw.p_min_pu = param_or(p, {"fw_p_min_pu"}, fw.p_min_pu);
    fw.p_max_pu = param_or(p, {"fw_p_max_pu"}, fw.p_max_pu);
    fw.filter_t_s = param_or(p, {"fw_filter_t_s", "T_pf"}, fw.filter_t_s);
    fw.ramp_rate_pu_per_s =
        param_or(p, {"fw_ramp_pu_per_s", "freq_watt_ramp"}, fw.ramp_rate_pu_per_s);
  }
}

void apply_gfl_params(const std::map<std::string, double>& p,
                      GridFollowingInverterParams& params) {
  params.response_t_s = param_or(p, {"response_t_s", "Tg", "Trv"}, params.response_t_s);
  params.power_filter_t_s = param_or(p, {"power_filter_t_s", "Tp", "Tpf"}, params.power_filter_t_s);
  apply_inverter_filter_and_limiter_params(p, params);
  params.frequency_watt_droop_pu = param_or(p, {"frequency_watt_droop_pu", "Ddn", "kf"}, params.frequency_watt_droop_pu);
  params.volt_var_droop_pu = param_or(p, {"volt_var_droop_pu", "Dvv", "kq"}, params.volt_var_droop_pu);
  apply_ieee1547_params(p, params.protection, params.f_ref_hz);
  apply_smart_inverter_params(p, params.volt_var, params.freq_watt, params.f_ref_hz);
  // Full-fidelity chain (PSD.jl OuterControl/InnerControl/LCLFilter aliases).
  params.outer_kp_p = param_or(p, {"outer_kp_p", "Kp_p", "kp_p"}, params.outer_kp_p);
  params.outer_ki_p = param_or(p, {"outer_ki_p", "Ki_p", "ki_p"}, params.outer_ki_p);
  params.outer_omega_z = param_or(p, {"outer_omega_z", "omega_z", "wz"}, params.outer_omega_z);
  params.outer_kp_q = param_or(p, {"outer_kp_q", "Kp_q", "kp_q"}, params.outer_kp_q);
  params.outer_ki_q = param_or(p, {"outer_ki_q", "Ki_q", "ki_q"}, params.outer_ki_q);
  params.outer_omega_f = param_or(p, {"outer_omega_f", "omega_f", "wf"}, params.outer_omega_f);
  params.inner_kpc = param_or(p, {"inner_kpc", "kpc"}, params.inner_kpc);
  params.inner_kic = param_or(p, {"inner_kic", "kic"}, params.inner_kic);
  params.inner_kffv = param_or(p, {"inner_kffv", "kffv"}, params.inner_kffv);
  params.lcl_lf_pu = param_or(p, {"lcl_lf_pu", "lf"}, params.lcl_lf_pu);
  params.lcl_rf_pu = param_or(p, {"lcl_rf_pu", "rf"}, params.lcl_rf_pu);
  params.lcl_cf_pu = param_or(p, {"lcl_cf_pu", "cf"}, params.lcl_cf_pu);
  params.lcl_lg_pu = param_or(p, {"lcl_lg_pu", "lg"}, params.lcl_lg_pu);
  params.lcl_rg_pu = param_or(p, {"lcl_rg_pu", "rg"}, params.lcl_rg_pu);
  params.phase_domain_control =
      param_or(p, {"phase_domain_control", "per_phase_control"},
               params.phase_domain_control ? 1.0 : 0.0) > 0.5;
  params.allow_zero_sequence_current =
      param_or(p, {"allow_zero_sequence_current", "four_wire_converter"},
               params.allow_zero_sequence_current ? 1.0 : 0.0) > 0.5;
  const double dc_fault_enabled =
      param_or(p, {"dc_fault_control_enabled"},
               params.dc_fault_control.enabled ? 1.0 : 0.0);
  if (!std::isfinite(dc_fault_enabled) ||
      (dc_fault_enabled != 0.0 && dc_fault_enabled != 1.0)) {
    // Use std::runtime_error rather than std::invalid_argument: in this build the
    // std::logic_error-family RTTI typeinfo is not matched by catch(const
    // std::exception&) (Apple libc++abi pointer-based matching), so a
    // logic_error would be swallowed by the builder's catch(...) wrapper and its
    // message lost. runtime_error propagates with its message intact.
    throw std::runtime_error("dc_fault_control_enabled must be exactly 0 or 1");
  }
  params.dc_fault_control.enabled = dc_fault_enabled == 1.0;
  params.dc_fault_control.active_power_derate_start_pu = param_or(
      p, {"dc_active_power_derate_start_pu"},
      params.dc_fault_control.active_power_derate_start_pu);
  params.dc_fault_control.undervoltage_block_pu = param_or(
      p, {"dc_undervoltage_block_pu"},
      params.dc_fault_control.undervoltage_block_pu);
  params.dc_fault_control.undervoltage_block_delay_s = param_or(
      p, {"dc_undervoltage_block_delay_s"},
      params.dc_fault_control.undervoltage_block_delay_s);
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
    params.filter_kind = filter_kind_from_name(name, params.filter_kind);
    params.limiter_kind = limiter_kind_from_name(name, params.limiter_kind);
  };
  apply_name(profile.model_name);
  for (const auto& component : profile.components) {
    apply_name(component.model);
    if (iequals(component.model, "ActivePowerPI") ||
        iequals(component.model, "ReactivePowerPI") ||
        iequals(component.model, "CurrentModeControl")) {
      params.full_fidelity = true;
    }
    if (!iequals(component.type, "pll")) continue;
    auto it = component.parameters.find("kp_pll");
    if (it != component.parameters.end()) params.pll_kp = it->second;
    it = component.parameters.find("ki_pll");
    if (it != component.parameters.end()) params.pll_ki = it->second;
    it = component.parameters.find("pll_lpf_t_s");
    if (it != component.parameters.end()) params.pll_lpf_t_s = it->second;
  }
  for (const auto& component : profile.components) {
    apply_gfl_params(component.parameters, params);
  }
  auto it = profile.parameters.find("pll_kp");
  if (it != profile.parameters.end()) params.pll_kp = it->second;
  it = profile.parameters.find("pll_ki");
  if (it != profile.parameters.end()) params.pll_ki = it->second;
  it = profile.parameters.find("pll_lpf_t_s");
  if (it != profile.parameters.end()) params.pll_lpf_t_s = it->second;
  apply_gfl_params(profile.parameters, params);
}

template <typename P>
void apply_gfm_params(const std::map<std::string, double>& p, P& params) {
  params.virtual_r_pu = param_or(p, {"virtual_r_pu", "Rv", "rv"}, params.virtual_r_pu);
  params.virtual_x_pu = param_or(p, {"virtual_x_pu", "Xv", "xv"}, params.virtual_x_pu);
  params.p_droop_pu = param_or(p, {"p_droop_pu", "mp", "Dp"}, params.p_droop_pu);
  params.q_droop_pu = param_or(p, {"q_droop_pu", "mq", "Dq"}, params.q_droop_pu);
  params.power_filter_t_s = param_or(p, {"power_filter_t_s", "Tf", "Tpf"}, params.power_filter_t_s);
  params.reactive_power_filter_t_s =
      param_or(p, {"reactive_power_filter_t_s", "q_filter_t_s", "Tq"}, params.reactive_power_filter_t_s);
  params.voltage_control_t_s = param_or(p, {"voltage_control_t_s", "Tv"}, params.voltage_control_t_s);
  params.voltage_kp = param_or(p, {"voltage_kp", "Kpv"}, params.voltage_kp);
  params.voltage_ki = param_or(p, {"voltage_ki", "Kiv"}, params.voltage_ki);
  apply_inverter_filter_and_limiter_params(p, params);
  params.vsm_ta_s = param_or(p, {"Ta", "vsm_ta_s"}, params.vsm_ta_s);
  params.vsm_damping_kd = param_or(p, {"kd", "vsm_damping_kd"}, params.vsm_damping_kd);
  params.vsm_frequency_droop_kw =
      param_or(p, {"kω", "kw", "komega", "vsm_frequency_droop_kw"},
               params.vsm_frequency_droop_kw);
  params.voc_k1 = param_or(p, {"k1", "voc_k1"}, params.voc_k1);
  params.voc_psi_rad = param_or(p, {"ψ", "psi", "voc_psi_rad"}, params.voc_psi_rad);
  params.voc_k2 = param_or(p, {"k2", "voc_k2"}, params.voc_k2);
  // PSD marks a non-reference GFM with is_not_reference=1.  Absence of that
  // optional flag must preserve the native default; otherwise every GFM
  // profile is silently locked to the synchronous reference frame and its
  // active-power loop cannot move the internal-source angle.
  if (const auto it = p.find("is_not_reference"); it != p.end()) {
    params.reference_frame_locked = it->second <= 0.5;
  }
  if (const auto it = p.find("reference_frame_unlocked"); it != p.end()) {
    params.reference_frame_locked = it->second <= 0.5;
  }
  params.phase_domain_control =
      param_or(p, {"phase_domain_control", "per_phase_control"},
               params.phase_domain_control ? 1.0 : 0.0) > 0.5;
  params.allow_zero_sequence_current =
      param_or(p, {"allow_zero_sequence_current", "four_wire_converter"},
               params.allow_zero_sequence_current ? 1.0 : 0.0) > 0.5;
  apply_ieee1547_params(p, params.protection,
                        param_or(p, {"f_ref_hz", "frequency_hz", "fn"}, 60.0));
  apply_smart_inverter_params(p, params.volt_var, params.freq_watt,
                              param_or(p, {"f_ref_hz", "frequency_hz", "fn"}, 60.0));
}

void apply_gfm_profile(const hacdcpf::DynamicModelProfile& profile,
                       GridFormingInverterParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  auto apply_name = [&](const std::string& name) {
    if (iequals(name, "VirtualInertia") || iequals(name, "VSM") ||
        iequals(name, "VSMGridForming")) {
      params.control_kind = GridFormingControlKind::VirtualInertia;
      params.model_name = "VSMGridForming";
    } else if (iequals(name, "ActiveVirtualOscillator") ||
               iequals(name, "ReactiveVirtualOscillator") ||
               iequals(name, "VOC") || iequals(name, "VOCGridForming")) {
      params.control_kind = GridFormingControlKind::VirtualOscillator;
      params.model_name = "VOCGridForming";
    } else if (iequals(name, "ActivePowerDroop") ||
               iequals(name, "GFMDroopOuterControl") ||
               iequals(name, "GridFormingNortonDroop")) {
      params.control_kind = GridFormingControlKind::Droop;
      params.model_name = "GridFormingNortonDroop";
    }
    params.filter_kind = filter_kind_from_name(name, params.filter_kind);
    params.limiter_kind = limiter_kind_from_name(name, params.limiter_kind);
  };
  if (!profile.empty()) {
    apply_name(profile.model_name);
    apply_gfm_params(profile.parameters, params);
    for (const auto& component : profile.components) {
      apply_name(component.model);
      apply_gfm_params(component.parameters, params);
    }
  }
}

void apply_gfm_profile(const hacdcpf::DynamicModelProfile& profile,
                       VSCConverterDynamicParams& params) {
  params.model_profiles = to_dynamic_profiles(profile);
  auto apply_name = [&](const std::string& name) {
    if (iequals(name, "VirtualInertia") || iequals(name, "VSM") ||
        iequals(name, "VSMGridForming")) {
      params.control_kind = GridFormingControlKind::VirtualInertia;
      params.model_name = "VSMGridForming";
    } else if (iequals(name, "ActiveVirtualOscillator") ||
               iequals(name, "ReactiveVirtualOscillator") ||
               iequals(name, "VOC") || iequals(name, "VOCGridForming")) {
      params.control_kind = GridFormingControlKind::VirtualOscillator;
      params.model_name = "VOCGridForming";
    } else if (iequals(name, "ActivePowerDroop") ||
               iequals(name, "GFMDroopOuterControl") ||
               iequals(name, "GridFormingNortonDroop")) {
      params.control_kind = GridFormingControlKind::Droop;
      params.model_name = "GridFormingNortonDroop";
    }
    params.filter_kind = filter_kind_from_name(name, params.filter_kind);
    params.limiter_kind = limiter_kind_from_name(name, params.limiter_kind);
  };
  if (!profile.empty()) {
    apply_name(profile.model_name);
    apply_gfm_params(profile.parameters, params);
    for (const auto& component : profile.components) {
      apply_name(component.model);
      apply_gfm_params(component.parameters, params);
    }
  }
}

PeriodicVariableSourceDynamicParams make_periodic_source_params(
    const ExternalGrid& grid,
    int bus_pos,
    double base_mva) {
  const auto pmap = merged_profile_parameters(grid.dynamic_model);
  PeriodicVariableSourceDynamicParams p;
  p.component_index = grid.index;
  p.bus = grid.bus;
  p.bus_pos = bus_pos;
  p.label = label_or(grid.name, "Periodic source " + std::to_string(grid.index));
  p.base_mva = base_mva;
  p.r_th_pu = param_or(pmap, {"R_th", "r_th_pu", "R", "r"}, grid.r_pu);
  p.x_th_pu = param_or(pmap, {"X_th", "x_th_pu", "X", "x"}, positive_or(grid.x_pu, 0.05));
  p.voltage_bias_pu = param_or(pmap, {"internal_voltage_bias", "voltage_bias_pu", "V_bias"}, grid.vm_pu);
  p.voltage_frequency_rad_s =
      param_or(pmap, {"internal_voltage_frequency_rad_s", "voltage_frequency_rad_s", "omega_v"}, 2.0 * kPi);
  p.voltage_sin_coeff_pu =
      param_or(pmap, {"internal_voltage_sin_coeff", "voltage_sin_coeff_pu", "V_sin"}, 1.0);
  p.voltage_cos_coeff_pu =
      param_or(pmap, {"internal_voltage_cos_coeff", "voltage_cos_coeff_pu", "V_cos"}, 0.0);
  p.angle_bias_rad = param_or(pmap, {"internal_angle_bias", "angle_bias_rad", "theta_bias"}, grid.va_deg * kDegToRad);
  p.angle_frequency_rad_s =
      param_or(pmap, {"internal_angle_frequency_rad_s", "angle_frequency_rad_s", "omega_theta"}, 2.0 * kPi);
  p.angle_sin_coeff_rad =
      param_or(pmap, {"internal_angle_sin_coeff", "angle_sin_coeff_rad", "theta_sin"}, 0.0);
  p.angle_cos_coeff_rad =
      param_or(pmap, {"internal_angle_cos_coeff", "angle_cos_coeff_rad", "theta_cos"}, 1.0);
  p.model_profiles = to_dynamic_profiles(grid.dynamic_model);
  return p;
}

CSVGN1DynamicParams make_csvgn1_params(const StaticGenerator& gen,
                                       int bus_pos,
                                       double base_mva) {
  const auto pmap = merged_profile_parameters(gen.dynamic_model);
  CSVGN1DynamicParams p;
  p.component_index = gen.index;
  p.bus = gen.bus;
  p.bus_pos = bus_pos;
  p.label = label_or(gen.name, "CSVGN1 " + std::to_string(gen.index));
  p.base_mva = base_mva;
  p.q_ref_mvar = gen.q_mvar * gen.scaling;
  p.K = param_or(pmap, {"K"}, p.K);
  p.T1 = param_or(pmap, {"T1"}, p.T1);
  p.T2 = param_or(pmap, {"T2"}, p.T2);
  p.T3 = param_or(pmap, {"T3"}, p.T3);
  p.T4 = param_or(pmap, {"T4"}, p.T4);
  p.T5 = param_or(pmap, {"T5"}, p.T5);
  p.Rmin = param_or(pmap, {"Rmin"}, p.Rmin);
  p.Vmax = param_or(pmap, {"Vmax"}, p.Vmax);
  p.Vmin = param_or(pmap, {"Vmin"}, p.Vmin);
  p.CBase = param_or(pmap, {"CBase", "Cbase"}, p.CBase);
  p.model_base_mva = param_or(pmap, {"base_power", "Mbase", "model_base_mva"}, p.model_base_mva);
  p.v_ref_pu = param_or(pmap, {"V_ref", "v_ref_pu"}, p.v_ref_pu);
  p.model_profiles = to_dynamic_profiles(gen.dynamic_model);
  return p;
}

DERAADynamicParams make_deraa_params(const StaticGenerator& gen,
                                     int bus_pos,
                                     double base_mva) {
  const auto pmap = merged_profile_parameters(gen.dynamic_model);
  DERAADynamicParams p;
  p.component_index = gen.index;
  p.bus = gen.bus;
  p.bus_pos = bus_pos;
  p.label = label_or(gen.name, "DERA " + std::to_string(gen.index));
  p.base_mva = base_mva;
  p.model_base_mva =
      param_or(pmap, {"base_power", "Mbase", "model_base_mva"}, positive_or(gen.sn_mva, base_mva));
  p.p_ref_mw = gen.p_mw * gen.scaling;
  p.q_ref_mvar = gen.q_mvar * gen.scaling;
  p.v_ref_pu = param_or(pmap, {"V_ref", "v_ref_pu"}, positive_or(gen.v_ref_pu, 1.0));
  p.pf_angle_ref_rad = param_or(pmap, {"Pfa_ref", "pf_angle_ref_rad"}, p.pf_angle_ref_rad);
  p.pf_flag = static_cast<int>(param_or(pmap, {"Pf_Flag"}, static_cast<double>(p.pf_flag)));
  p.freq_flag = static_cast<int>(param_or(pmap, {"Freq_Flag"}, static_cast<double>(p.freq_flag)));
  p.pq_flag = static_cast<int>(param_or(pmap, {"PQ_Flag"}, static_cast<double>(p.pq_flag)));
  p.gen_flag = static_cast<int>(param_or(pmap, {"Gen_Flag"}, static_cast<double>(p.gen_flag)));
  p.T_rv = param_or(pmap, {"T_rv", "Trv"}, p.T_rv);
  p.Trf = param_or(pmap, {"Trf"}, p.Trf);
  p.dbd1 = param_or(pmap, {"dbd1", "dbd_low"}, p.dbd1);
  p.dbd2 = param_or(pmap, {"dbd2", "dbd_high"}, p.dbd2);
  p.K_qv = param_or(pmap, {"K_qv", "Kqv"}, p.K_qv);
  p.Tp = param_or(pmap, {"Tp"}, p.Tp);
  p.T_iq = param_or(pmap, {"T_iq", "Tiq"}, p.T_iq);
  p.Tg = param_or(pmap, {"Tg"}, p.Tg);
  p.Tv = param_or(pmap, {"Tv"}, p.Tv);
  p.Tpord = param_or(pmap, {"Tpord"}, p.Tpord);
  p.Kpg = param_or(pmap, {"Kpg"}, p.Kpg);
  p.Kig = param_or(pmap, {"Kig"}, p.Kig);
  p.D_dn = param_or(pmap, {"D_dn", "Ddn"}, p.D_dn);
  p.D_up = param_or(pmap, {"D_up", "Dup"}, p.D_up);
  p.fdbd1 = param_or(pmap, {"fdbd1", "fdbd_low"}, p.fdbd1);
  p.fdbd2 = param_or(pmap, {"fdbd2", "fdbd_high"}, p.fdbd2);
  p.fe_min = param_or(pmap, {"fe_min", "femin"}, p.fe_min);
  p.fe_max = param_or(pmap, {"fe_max", "femax"}, p.fe_max);
  p.p_min = param_or(pmap, {"P_min", "p_min"}, p.p_min);
  p.p_max = param_or(pmap, {"P_max", "p_max"}, p.p_max);
  p.dp_min = param_or(pmap, {"dP_min", "dp_min"}, p.dp_min);
  p.dp_max = param_or(pmap, {"dP_max", "dp_max"}, p.dp_max);
  p.I_max = param_or(pmap, {"I_max", "Imax"}, p.I_max);
  p.Iq_min = param_or(pmap, {"Iq_min"}, p.Iq_min);
  p.Iq_max = param_or(pmap, {"Iq_max"}, p.Iq_max);
  p.rr_pwr = param_or(pmap, {"rrpwr", "Rrpwr"}, p.rr_pwr);
  p.v_trip_low_pu = param_or(pmap, {"Vtrip_L", "v_trip_low_pu", "Vltrip"}, p.v_trip_low_pu);
  p.v_trip_high_pu = param_or(pmap, {"Vtrip_H", "v_trip_high_pu", "Vhtrip"}, p.v_trip_high_pu);
  p.f_trip_low_pu = param_or(pmap, {"Ftrip_L", "f_trip_low_pu", "Fltrip"}, p.f_trip_low_pu);
  p.f_trip_high_pu = param_or(pmap, {"Ftrip_H", "f_trip_high_pu", "Fhtrip"}, p.f_trip_high_pu);
  p.trip_delay_s = param_or(pmap, {"Ttrip", "trip_delay_s", "trip_delay"}, p.trip_delay_s);
  p.model_profiles = to_dynamic_profiles(gen.dynamic_model);
  return p;
}

// WECC generic renewable converter REGC_A (design doc §11.6) from a static
// generator's dynamic model.
REGCADynamicParams make_regca_params(const StaticGenerator& gen,
                                     int bus_pos,
                                     double base_mva) {
  const auto pmap = merged_profile_parameters(gen.dynamic_model);
  REGCADynamicParams p;
  p.component_index = gen.index;
  p.bus = gen.bus;
  p.bus_pos = bus_pos;
  p.label = label_or(gen.name, "REGC_A " + std::to_string(gen.index));
  p.base_mva = base_mva;
  p.model_base_mva = param_or(pmap, {"base_power", "Mbase", "model_base_mva"},
                              positive_or(gen.sn_mva, base_mva));
  p.p_ref_mw = gen.p_mw * gen.scaling;
  p.q_ref_mvar = gen.q_mvar * gen.scaling;
  p.v_ref_pu = param_or(pmap, {"V_ref", "v_ref_pu"}, positive_or(gen.v_ref_pu, 1.0));
  p.t_g_s = param_or(pmap, {"Tg", "t_g_s"}, p.t_g_s);
  p.t_fltr_s = param_or(pmap, {"Tfltr", "T_fltr", "t_fltr_s"}, p.t_fltr_s);
  p.v_lvacm0_pu = param_or(pmap, {"Volim", "Lvpl0", "v_lvacm0_pu"}, p.v_lvacm0_pu);
  p.v_lvacm1_pu = param_or(pmap, {"Lvpl1", "Brkpt", "v_lvacm1_pu"}, p.v_lvacm1_pu);
  p.v_hvrcm_pu = param_or(pmap, {"Volim_hi", "Vhvrcm", "v_hvrcm_pu"}, p.v_hvrcm_pu);
  p.k_hvrcm = param_or(pmap, {"Khv", "Iqrmax", "k_hvrcm"}, p.k_hvrcm);
  p.i_max_pu = param_or(pmap, {"Imax", "I_max", "i_max_pu"}, p.i_max_pu);
  if (param_or(pmap, {"reactive_current_priority", "iq_priority", "Qtrip"}, 0.0) != 0.0) {
    p.reactive_priority = true;
  }
  if (param_or(pmap, {"reec_enabled", "REECA", "reeca", "REEC_A"}, 0.0) != 0.0) {
    auto& r = p.reec;
    r.enabled = true;
    r.vref0_pu = param_or(pmap, {"Vref0", "vref0_pu"}, r.vref0_pu);
    r.dbd1 = param_or(pmap, {"dbd1", "reec_dbd1"}, r.dbd1);
    r.dbd2 = param_or(pmap, {"dbd2", "reec_dbd2"}, r.dbd2);
    r.kqv = param_or(pmap, {"Kqv", "kqv"}, r.kqv);
    r.iqh1 = param_or(pmap, {"Iqh1", "iqh1"}, r.iqh1);
    r.iql1 = param_or(pmap, {"Iql1", "iql1"}, r.iql1);
    r.tpord_s = param_or(pmap, {"Tpord", "tpord_s"}, r.tpord_s);
    r.p_rate_pu_per_s = param_or(pmap, {"rrpwr", "p_rate_pu_per_s"}, r.p_rate_pu_per_s);
    r.ip_min = param_or(pmap, {"Ipmin", "Ip_min", "ip_min"}, r.ip_min);
    r.ip_max = param_or(pmap, {"Ipmax", "Ip_max", "ip_max"}, r.ip_max);
    r.iq_min = param_or(pmap, {"Iqmin", "Iq_min", "iq_min"}, r.iq_min);
    r.iq_max = param_or(pmap, {"Iqmax", "Iq_max", "iq_max"}, r.iq_max);
  }
  if (param_or(pmap, {"repc_enabled", "REPCA", "repca", "REPC_A"}, 0.0) != 0.0) {
    p.reec.enabled = true;  // the plant controller feeds the REEC_A reference
    auto& pc = p.repc;
    pc.enabled = true;
    pc.vref_pu = param_or(pmap, {"Vref_plant", "repc_vref_pu"}, pc.vref_pu);
    pc.dbd1 = param_or(pmap, {"repc_dbd1", "dbd1_plant"}, pc.dbd1);
    pc.dbd2 = param_or(pmap, {"repc_dbd2", "dbd2_plant"}, pc.dbd2);
    pc.kp = param_or(pmap, {"Kp_plant", "Kpv_plant", "repc_kp"}, pc.kp);
    pc.ki = param_or(pmap, {"Ki_plant", "Kiv_plant", "repc_ki"}, pc.ki);
    pc.q_min = param_or(pmap, {"Qmin_plant", "repc_q_min"}, pc.q_min);
    pc.q_max = param_or(pmap, {"Qmax_plant", "repc_q_max"}, pc.q_max);
    if (param_or(pmap, {"repc_freq_control", "Freq_Flag_plant", "Pf_control"}, 0.0) != 0.0) {
      pc.freq_control = true;
      pc.f_nominal_hz = param_or(pmap, {"f_nominal_hz", "Fbase", "f_ref_hz"},
                                 positive_or(gen.f_ref_hz, 60.0));
      pc.f_dbd_hz = param_or(pmap, {"f_dbd_hz", "fdbd", "repc_fdbd"}, pc.f_dbd_hz);
      pc.f_droop = param_or(pmap, {"f_droop", "Ddn", "repc_droop"}, pc.f_droop);
      pc.p_min_pu = param_or(pmap, {"Pmin_plant", "repc_p_min"}, pc.p_min_pu);
      pc.p_max_pu = param_or(pmap, {"Pmax_plant", "repc_p_max"}, pc.p_max_pu);
    }
  }
  p.model_profiles = to_dynamic_profiles(gen.dynamic_model);
  return p;
}

InductionMachineDynamicParams make_induction_machine_params(
    const AsynchronousMotor& motor,
    int bus_pos,
    double base_mva) {
  const auto pmap = merged_profile_parameters(motor.dynamic_model);
  InductionMachineDynamicParams p;
  p.component_index = motor.index;
  p.bus = motor.bus;
  p.bus_pos = bus_pos;
  p.label = label_or(motor.name, "Induction machine " + std::to_string(motor.index));
  p.base_mva = base_mva;
  p.model_base_mva = positive_or(motor.sn_mva, base_mva);
  const double pf = std::clamp(positive_or(motor.cos_phi, 0.85), 0.01, 1.0);
  p.p_mech_mw =
      param_or(pmap,
               {"P_mech", "p_mech_mw", "P_load"},
               p.model_base_mva * pf * positive_or(motor.efficiency, 0.95));
  const double q_default =
      p.model_base_mva * std::sqrt(std::max(0.0, 1.0 - pf * pf));
  p.q_nom_mvar = param_or(pmap, {"Q_load", "q_nom_mvar"}, q_default);
  p.r_s_pu = param_or(pmap, {"Rs", "R_s", "r_s_pu", "R_stator"}, positive_or(motor.r_pu, p.r_s_pu));
  p.x_s_pu = param_or(pmap, {"Xs", "X_s", "x_s_pu", "X_stator"}, positive_or(motor.x_pu * 0.5, p.x_s_pu));
  p.r_r_pu = param_or(pmap, {"Rr", "R_r", "r_r_pu", "R_rotor"}, positive_or(motor.r_pu, p.r_r_pu));
  p.x_r_pu = param_or(pmap, {"Xr", "X_r", "x_r_pu", "X_rotor"}, positive_or(motor.x_pu * 0.5, p.x_r_pu));
  p.x_m_pu = param_or(pmap, {"Xm", "X_m", "x_m_pu", "X_magnetizing"}, p.x_m_pu);
  p.inertia_h = param_or(pmap, {"H", "h", "inertia_h"}, p.inertia_h);
  p.damping_d = param_or(pmap, {"D", "damping_d"}, p.damping_d);
  p.torque_exponent = param_or(pmap, {"torque_exponent", "N_exp"}, p.torque_exponent);
  p.fifth_order = profile_model_is(motor.dynamic_model,
                                   {"SingleCageInductionMachine"}) ||
                  param_or(pmap, {"fifth_order"}, 0.0) != 0.0;
  p.flux_model = profile_model_is(motor.dynamic_model,
                                  {"FluxInductionMachine",
                                   "FluxSingleCageInductionMachine",
                                   "SingleCageInductionMachineFlux"}) ||
                 param_or(pmap, {"flux_model"}, 0.0) != 0.0;
  p.model_profiles = to_dynamic_profiles(motor.dynamic_model);
  return p;
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
  // Sequence-network interface parameters (design doc §8.8), common to every
  // machine model; unset (0) reproduces balanced positive-sequence behavior.
  params.x2_pu = find_param({"X2", "Xnegative", "x2"}, params.x2_pu);
  params.r2_pu = find_param({"R2", "Rnegative", "r2"}, params.r2_pu);
  params.x0_pu = find_param({"X0", "Xzero", "x0"}, params.x0_pu);
  params.r0_pu = find_param({"R0", "Rzero", "r0"}, params.r0_pu);
  // IEEE 1547 ride-through / trip / reconnect protection (design doc §11.7) for a
  // synchronous DER; opt-in via the dynamic-model parameters.
  apply_ieee1547_params(profile.parameters, params.protection, params.frequency_hz);
  if (iequals(profile.model_name, "OneDOneQMachine") ||
      iequals(profile.model_name, "OneDOneQ")) {
    params.machine_model = SynchronousMachineModelKind::OneDOneQ;
    params.machine_model_name = "OneDOneQMachine";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
  } else if (iequals(profile.model_name, "AndersonFouadMachine") ||
             iequals(profile.model_name, "AndersonFouad")) {
    params.machine_model = SynchronousMachineModelKind::AndersonFouad;
    params.machine_model_name = "AndersonFouadMachine";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xqpp_pu = find_param({"Xq_pp", "Xqpp", "xq_pp", "xqpp"}, params.xqpp_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
  } else if (iequals(profile.model_name, "SimpleAFMachine") ||
             iequals(profile.model_name, "SimpleAndersonFouadMachine") ||
             iequals(profile.model_name, "SimpleAndersonFouad") ||
             iequals(profile.model_name, "SimpleAF")) {
    params.machine_model = SynchronousMachineModelKind::SimpleAF;
    params.machine_model_name = "SimpleAFMachine";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xqpp_pu = find_param({"Xq_pp", "Xqpp", "xq_pp", "xqpp"}, params.xqpp_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
  } else if (iequals(profile.model_name, "MarconatoMachine") ||
             iequals(profile.model_name, "Marconato")) {
    params.machine_model = SynchronousMachineModelKind::Marconato;
    params.machine_model_name = "MarconatoMachine";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xqpp_pu = find_param({"Xq_pp", "Xqpp", "xq_pp", "xqpp"}, params.xqpp_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
    params.t_aa_s = find_param({"T_AA", "TAA", "t_aa", "taa"}, params.t_aa_s);
  } else if (iequals(profile.model_name, "SauerPaiMachine") ||
             iequals(profile.model_name, "SauerPai")) {
    params.machine_model = SynchronousMachineModelKind::SauerPai;
    params.machine_model_name = "SauerPaiMachine";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xqpp_pu = find_param({"Xq_pp", "Xqpp", "xq_pp", "xqpp"}, params.xqpp_pu);
    params.xl_pu = find_param({"Xl", "xl"}, params.xl_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
  } else if (iequals(profile.model_name, "SimpleMarconatoMachine") ||
             iequals(profile.model_name, "SimpleMarconato")) {
    params.machine_model = SynchronousMachineModelKind::SimpleMarconato;
    params.machine_model_name = "SimpleMarconatoMachine";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xqp_pu = find_param({"Xq_p", "Xqp", "xq_p", "xqp"}, params.xqp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xqpp_pu = find_param({"Xq_pp", "Xqpp", "xq_pp", "xqpp"}, params.xqpp_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.tq0p_s = find_param({"Tq0_p", "Tq0p", "tq0_p", "tq0p"}, params.tq0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
    params.t_aa_s = find_param({"T_AA", "TAA", "t_aa", "taa"}, params.t_aa_s);
  } else if (iequals(profile.model_name, "GENROU") ||
             iequals(profile.model_name, "RoundRotorQuadratic")) {
    params.machine_model = SynchronousMachineModelKind::GENROU;
    params.machine_model_name = "GENROU";
    params.psd_genrou_model = true;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
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
  } else if (iequals(profile.model_name, "GENROE") ||
             iequals(profile.model_name, "RoundRotorExponential")) {
    params.machine_model = SynchronousMachineModelKind::GENROE;
    params.machine_model_name = "GENROE";
    params.psd_genrou_model = true;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
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
  } else if (iequals(profile.model_name, "GENSAL") ||
             iequals(profile.model_name, "SalientPoleQuadratic") ||
             iequals(profile.model_name, "GENSAE") ||
             iequals(profile.model_name, "SalientPoleExponential")) {
    const bool exponential = iequals(profile.model_name, "GENSAE") ||
                             iequals(profile.model_name, "SalientPoleExponential");
    params.machine_model = exponential ? SynchronousMachineModelKind::GENSAE
                                       : SynchronousMachineModelKind::GENSAL;
    params.machine_model_name = exponential ? "GENSAE" : "GENSAL";
    params.psd_genrou_model = false;
    params.inertia_h = find_param({"H", "h"}, params.inertia_h);
    params.damping_d = find_param({"D", "damping_d", "damping"}, params.damping_d);
    params.r_pu = find_param({"R", "Ra", "r"}, params.r_pu);
    params.xd_pu = find_param({"Xd", "xd"}, params.xd_pu);
    params.xq_pu = find_param({"Xq", "xq"}, params.xq_pu);
    params.xdp_pu = find_param({"Xd_p", "Xdp", "xd_p", "xdp"}, params.xdp_pu);
    params.xdpp_pu = find_param({"Xd_pp", "Xdpp", "xd_pp", "xdpp"}, params.xdpp_pu);
    params.xl_pu = find_param({"Xl", "xl"}, params.xl_pu);
    params.td0p_s = find_param({"Td0_p", "Td0p", "td0_p", "td0p"}, params.td0p_s);
    params.td0pp_s = find_param({"Td0_pp", "Td0pp", "td0_pp", "td0pp"}, params.td0pp_s);
    params.tq0pp_s = find_param({"Tq0_pp", "Tq0pp", "tq0_pp", "tq0pp"}, params.tq0pp_s);
    params.saturation_a = find_param({"Sat_A", "saturation_a", "Se_A"}, params.saturation_a);
    params.saturation_b = find_param({"Sat_B", "saturation_b", "Se_B"}, params.saturation_b);
  }
}

// Build a five-mass shaft from a machine's dynamic_model "shaft" component profile.
std::unique_ptr<FiveMassShaft> make_five_mass_shaft(
    const hacdcpf::DynamicModelComponentProfile& comp,
    const VoltageSourceDynamicParams& mp) {
  FiveMassShaftParams s;
  s.component_index = mp.component_index;
  s.machine_index = mp.component_index;
  s.label = label_or(mp.label, "Machine " + std::to_string(mp.component_index)) + " shaft";
  s.base_mva = mp.base_mva;
  s.frequency_hz = mp.frequency_hz;
  s.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  s.inertia_h[0] = param_or(p, {"H1", "H_hp", "h_hp"}, s.inertia_h[0]);
  s.inertia_h[1] = param_or(p, {"H2", "H_ip", "h_ip"}, s.inertia_h[1]);
  s.inertia_h[2] = param_or(p, {"H3", "H_lpa", "h_lpa"}, s.inertia_h[2]);
  s.inertia_h[3] = param_or(p, {"H4", "H_lpb", "h_lpb"}, s.inertia_h[3]);
  s.inertia_h[4] = param_or(p, {"H5", "H_gen", "h_gen"}, s.inertia_h[4]);
  s.stiffness_pu[0] = param_or(p, {"K12", "K_hp_ip", "k12"}, s.stiffness_pu[0]);
  s.stiffness_pu[1] = param_or(p, {"K23", "K_ip_lpa", "k23"}, s.stiffness_pu[1]);
  s.stiffness_pu[2] = param_or(p, {"K34", "K_lpa_lpb", "k34"}, s.stiffness_pu[2]);
  s.stiffness_pu[3] = param_or(p, {"K45", "K_lpb_gen", "k45"}, s.stiffness_pu[3]);
  s.damping_pu[0] = param_or(p, {"D12", "D_hp_ip", "d12"}, s.damping_pu[0]);
  s.damping_pu[1] = param_or(p, {"D23", "D_ip_lpa", "d23"}, s.damping_pu[1]);
  s.damping_pu[2] = param_or(p, {"D34", "D_lpa_lpb", "d34"}, s.damping_pu[2]);
  s.damping_pu[3] = param_or(p, {"D45", "D_lpb_gen", "d45"}, s.damping_pu[3]);
  return std::make_unique<FiveMassShaft>(s);
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
  if (iequals(comp.model, "IEEEG1")) {
    g.model = GovernorModel::IEEEG1;
  } else if (iequals(comp.model, "TGTypeI")) {
    g.model = GovernorModel::TGTypeI;
  } else if (iequals(comp.model, "TGTypeII")) {
    g.model = GovernorModel::TGTypeII;
  } else if (iequals(comp.model, "GAST") || iequals(comp.model, "GasTG")) {
    g.model = GovernorModel::GAST;
  } else if (iequals(comp.model, "HYGOV") || iequals(comp.model, "HydroTurbineGov")) {
    g.model = GovernorModel::HYGOV;
  } else if (iequals(comp.model, "DEGOV")) {
    g.model = GovernorModel::DEGOV;
  } else if (iequals(comp.model, "DEGOV1")) {
    g.model = GovernorModel::DEGOV1;
  } else if (iequals(comp.model, "PIDGOV")) {
    g.model = GovernorModel::PIDGOV;
  } else if (iequals(comp.model, "WPIDHY")) {
    g.model = GovernorModel::WPIDHY;
  } else if (iequals(comp.model, "TGSimple")) {
    g.model = GovernorModel::TGSimple;
  } else {
    g.model = GovernorModel::TGOV1;
  }
  g.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  const double base = std::max(1.0, mp.base_mva);
  g.droop_r = param_or(p, {"R", "droop_r", "droop"}, g.droop_r);
  g.t_s = param_or(p, {"T1", "t1", "Tg", "valve_t_s"}, g.t_s);
  g.t2_s = param_or(p, {"T2", "t2"}, g.t2_s);
  g.turbine_t_s = param_or(p, {"T3", "t3", "Tt", "turbine_t_s"}, g.turbine_t_s);
  g.damping_d_t = param_or(p, {"D_T", "Dt", "D_t", "d_t"}, g.damping_d_t);
  g.reheat_t_s = param_or(p, {"Tr", "T5", "reheat_t_s"}, g.reheat_t_s);
  g.reheat_k = param_or(p, {"K1", "Khp", "reheat_k"}, g.reheat_k);
  g.ta_s = param_or(p, {"Ta", "ta"}, g.ta_s);
  g.tb_s = param_or(p, {"Tb", "tb"}, g.tb_s);
  g.ki = param_or(p, {"Ki", "ki"}, g.ki);
  g.kd = param_or(p, {"Kd", "kd"}, g.kd);
  g.fuel_t_s = param_or(p, {"Tf", "T_fuel", "fuel_t_s", "T2"}, g.fuel_t_s);
  g.temperature_t_s =
      param_or(p, {"Tt", "T_temp", "temperature_t_s", "T3"}, g.temperature_t_s);
  g.water_t_s = param_or(p, {"Tw", "water_t_s"}, g.water_t_s);
  g.gate_t_s = param_or(p, {"Tg", "gate_t_s", "T_gate"}, g.gate_t_s);
  g.gate_max_pu = param_or(p, {"Gmax", "gate_max_pu"}, g.gate_max_pu);
  g.gate_min_pu = param_or(p, {"Gmin", "gate_min_pu"}, g.gate_min_pu);
  if (g.model == GovernorModel::TGTypeI) {
    g.t_s = param_or(p, {"Ts", "T_s", "T1", "t1"}, g.t_s);
    g.tc_s = param_or(p, {"Tc", "T_c", "servo_t_s"}, g.tc_s);
    g.t3_s = param_or(p, {"T3", "t3"}, g.t3_s);
    g.t4_s = param_or(p, {"T4", "t4"}, g.t4_s);
    g.t5_s = param_or(p, {"T5", "t5"}, g.t5_s);
  } else if (g.model == GovernorModel::TGTypeII) {
    g.t_s = param_or(p, {"T1", "t1"}, g.t_s);
    g.turbine_t_s = param_or(p, {"T2", "t2"}, g.turbine_t_s);
  } else if (g.model == GovernorModel::GAST) {
    g.t_s = param_or(p, {"T1", "t1", "Tg"}, g.t_s);
    g.fuel_t_s = param_or(p, {"T2", "t2", "Tf"}, g.fuel_t_s);
    g.temperature_t_s = param_or(p, {"T3", "t3", "Tt"}, g.temperature_t_s);
  } else if (g.model == GovernorModel::HYGOV) {
    g.gate_t_s = param_or(p, {"Tg", "gate_t_s"}, g.gate_t_s);
    g.ta_s = param_or(p, {"Tr", "Ta", "ta"}, g.ta_s);
    g.tb_s = param_or(p, {"Tf", "Tb", "tb"}, g.tb_s);
    g.water_t_s = param_or(p, {"Tw", "water_t_s"}, g.water_t_s);
  } else if (g.model == GovernorModel::PIDGOV ||
             g.model == GovernorModel::WPIDHY) {
    g.t_s = param_or(p, {"T_reg", "Treg", "t_reg"}, g.t_s);
    g.ki = param_or(p, {"Ki", "ki"}, g.ki);
    g.kd = param_or(p, {"Kd", "kd"}, g.kd);
    g.ta_s = param_or(p, {"Ta", "ta"}, g.ta_s);
    g.tb_s = param_or(p, {"Tb", "tb"}, g.tb_s);
    g.water_t_s = param_or(p, {"Tw", "water_t_s"}, g.water_t_s);
  }
  const double vmax = param_or(p, {"Vmax", "pmax_pu", "tau_max"}, 0.0);
  const double vmin = param_or(p, {"Vmin", "pmin_pu", "tau_min"}, 0.0);
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
  if (iequals(comp.model, "IEEET1")) {
    e.model = ExciterModel::IEEET1;
  } else if (iequals(comp.model, "AVRSimple")) {
    e.model = ExciterModel::AVRSimple;
  } else if (iequals(comp.model, "AVRTypeI")) {
    e.model = ExciterModel::AVRTypeI;
  } else if (iequals(comp.model, "AVRTypeII")) {
    e.model = ExciterModel::AVRTypeII;
  } else if (iequals(comp.model, "ESAC1A") || iequals(comp.model, "EXAC1A")) {
    e.model = ExciterModel::ESAC1A;
  } else if (iequals(comp.model, "EXAC1")) {
    e.model = ExciterModel::EXAC1;
  } else if (iequals(comp.model, "EXST1")) {
    e.model = ExciterModel::EXST1;
  } else if (iequals(comp.model, "SCRX")) {
    e.model = ExciterModel::SCRX;
  } else if (iequals(comp.model, "ESST1A")) {
    e.model = ExciterModel::ESST1A;
  } else if (iequals(comp.model, "ST6B")) {
    e.model = ExciterModel::ST6B;
  } else if (iequals(comp.model, "ST8C")) {
    e.model = ExciterModel::ST8C;
  } else {
    e.model = ExciterModel::SEXS;
  }
  e.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  e.ka = param_or(p, {"Ka", "K", "ka"}, e.ka);
  e.ta_s = param_or(p, {"Ta", "ta"}, e.ta_s);
  e.ta_over_tb = param_or(p,
                          {"Ta_Tb", "TaTb", "Ta_over_Tb", "TaOverTb", "ta_tb"},
                          e.ta_over_tb);
  e.tb_s = param_or(p, {"Tb", "tb"}, e.tb_s);
  if (e.model == ExciterModel::SEXS) {
    const bool has_legacy_ta = p.find("Ta") != p.end() || p.find("ta") != p.end();
    const bool has_explicit_tb = p.find("Tb") != p.end() || p.find("tb") != p.end();
    const bool has_explicit_ratio =
        p.find("Ta_Tb") != p.end() || p.find("TaTb") != p.end() ||
        p.find("Ta_over_Tb") != p.end() || p.find("TaOverTb") != p.end() ||
        p.find("ta_tb") != p.end();
    if (has_legacy_ta && !has_explicit_tb) {
      e.tb_s = e.ta_s;
    }
    if (has_legacy_ta && has_explicit_tb && !has_explicit_ratio &&
        std::abs(e.tb_s) > 1e-12) {
      e.ta_over_tb = e.ta_s / e.tb_s;
    }
    if (has_legacy_ta && p.find("Te") == p.end() && p.find("te") == p.end()) {
      e.te_s = e.ta_s;
    }
  }
  e.te_s = param_or(p, {"Te", "te"}, e.te_s);
  e.kv = param_or(p, {"Kv", "kv"}, e.kv);
  e.ke = param_or(p, {"Ke", "ke"}, e.ke);
  e.kf = param_or(p, {"Kf", "kf"}, e.kf);
  e.tf_s = param_or(p, {"Tf", "tf"}, e.tf_s);
  e.tr_s = param_or(p, {"Tr", "tr"}, e.tr_s);
  e.ae = param_or(p, {"Ae", "ae"}, e.ae);
  e.be = param_or(p, {"Be", "be"}, e.be);
  e.k0 = param_or(p, {"K0", "k0"}, e.k0);
  e.t1_s = param_or(p, {"T1", "t1"}, e.t1_s);
  e.t2_s = param_or(p, {"T2", "t2"}, e.t2_s);
  e.t3_s = param_or(p, {"T3", "t3"}, e.t3_s);
  e.t4_s = param_or(p, {"T4", "t4"}, e.t4_s);
  e.tc_s = param_or(p, {"Tc", "tc"}, e.tc_s);
  e.tb1_s = param_or(p, {"Tb1", "tb1"}, e.tb1_s);
  e.tc1_s = param_or(p, {"Tc1", "tc1"}, e.tc1_s);
  e.kg = param_or(p, {"Kg", "kg"}, e.kg);
  e.kc = param_or(p, {"Kc", "kc"}, e.kc);
  e.kd = param_or(p, {"Kd", "kd"}, e.kd);
  e.kp = param_or(p, {"Kp", "kp", "K_pr", "Kpa"}, e.kp);
  e.ki = param_or(p, {"Ki", "ki", "K_ir", "Kia"}, e.ki);
  e.va_max_pu = param_or(p, {"Va_max", "VaMax", "Vrmax", "Emax"}, e.va_max_pu);
  e.va_min_pu = param_or(p, {"Va_min", "VaMin", "Vrmin", "Emin"}, e.va_min_pu);
  e.efd_max_pu = param_or(p, {"Emax", "Vrmax", "efd_max_pu"}, e.efd_max_pu);
  e.efd_min_pu = param_or(p, {"Emin", "Vrmin", "efd_min_pu"}, e.efd_min_pu);
  e.v_ref_pu = param_or(p, {"Vref", "V_ref", "v_ref_pu"}, e.v_ref_pu);
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
  if (iequals(comp.model, "IEEEST")) {
    s.model = PSSModel::IEEEST;
  } else if (iequals(comp.model, "STAB1")) {
    s.model = PSSModel::STAB1;
  } else if (iequals(comp.model, "PSS2A")) {
    s.model = PSSModel::PSS2A;
  } else if (iequals(comp.model, "PSS2B")) {
    s.model = PSSModel::PSS2B;
  } else if (iequals(comp.model, "PSS2C")) {
    s.model = PSSModel::PSS2C;
  } else {
    s.model = PSSModel::PSS1A;
  }
  s.parameter_set = comp.parameter_set;
  const auto& p = comp.parameters;
  s.ks = param_or(p, {"Ks", "Ks1", "ks"}, s.ks);
  s.tw_s = param_or(p, {"Tw", "tw"}, s.tw_s);
  s.t1_s = param_or(p, {"T1", "t1"}, s.t1_s);
  s.t2_s = param_or(p, {"T2", "t2"}, s.t2_s);
  s.t3_s = param_or(p, {"T3", "t3"}, s.t3_s);
  s.t4_s = param_or(p, {"T4", "t4"}, s.t4_s);
  s.vs_max_pu = param_or(p, {"Vsmax", "Lsmax", "ls_max", "vs_max_pu", "Vstmax"}, s.vs_max_pu);
  s.vs_min_pu = param_or(p, {"Vsmin", "Lsmin", "ls_min", "vs_min_pu", "Vstmin"}, s.vs_min_pu);
  s.a1 = param_or(p, {"A1", "a1"}, s.a1);
  s.a2 = param_or(p, {"A2", "a2"}, s.a2);
  s.a3 = param_or(p, {"A3", "a3"}, s.a3);
  s.a4 = param_or(p, {"A4", "a4"}, s.a4);
  s.a5 = param_or(p, {"A5", "a5"}, s.a5);
  s.a6 = param_or(p, {"A6", "a6"}, s.a6);
  s.t5_s = param_or(p, {"T5", "t5"}, s.t5_s);
  s.t6_s = param_or(p, {"T6", "t6"}, s.t6_s);
  s.vcu = param_or(p, {"Vcu", "V_CU", "vcu"}, s.vcu);
  s.vcl = param_or(p, {"Vcl", "V_CL", "vcl"}, s.vcl);
  s.input_code = static_cast<int>(param_or(p, {"input_code", "InputCode"}, s.input_code));
  s.kt = param_or(p, {"KT", "Kt", "kt"}, s.kt);
  s.stab_t_s = param_or(p, {"T", "Tw", "tw"}, s.stab_t_s);
  s.t1_over_t3 = param_or(p, {"T1T3", "t1_over_t3"}, s.t1_over_t3);
  s.t2_over_t4 = param_or(p, {"T2T4", "t2_over_t4"}, s.t2_over_t4);
  s.h_lim = param_or(p, {"H_lim", "Hlim", "h_lim"}, s.h_lim);
  s.ks1 = param_or(p, {"Ks1", "ks1"}, s.ks1);
  s.ks2 = param_or(p, {"Ks2", "ks2"}, s.ks2);
  s.ks3 = param_or(p, {"Ks3", "ks3"}, s.ks3);
  s.m_rtf = param_or(p, {"M", "M_rtf", "m_rtf"}, s.m_rtf);
  s.n_rtf = param_or(p, {"N", "N_rtf", "n_rtf"}, s.n_rtf);
  s.tw1_s = param_or(p, {"Tw1", "tw1"}, s.tw1_s);
  s.tw2_s = param_or(p, {"Tw2", "tw2"}, s.tw2_s);
  s.tw3_s = param_or(p, {"Tw3", "tw3"}, s.tw3_s);
  s.tw4_s = param_or(p, {"Tw4", "tw4"}, s.tw4_s);
  s.t7_s = param_or(p, {"T7", "t7"}, s.t7_s);
  s.t8_s = param_or(p, {"T8", "t8"}, s.t8_s);
  s.t9_s = param_or(p, {"T9", "t9"}, s.t9_s);
  s.t10_s = param_or(p, {"T10", "t10"}, s.t10_s);
  s.t11_s = param_or(p, {"T11", "t11"}, s.t11_s);
  s.t12_s = param_or(p, {"T12", "t12"}, s.t12_s);
  s.t13_s = param_or(p, {"T13", "t13"}, s.t13_s);
  s.vs1_max_pu = param_or(p, {"Vs1max", "Vs1_max", "vs1_max_pu"}, s.vs1_max_pu);
  s.vs1_min_pu = param_or(p, {"Vs1min", "Vs1_min", "vs1_min_pu"}, s.vs1_min_pu);
  s.vs2_max_pu = param_or(p, {"Vs2max", "Vs2_max", "vs2_max_pu"}, s.vs2_max_pu);
  s.vs2_min_pu = param_or(p, {"Vs2min", "Vs2_min", "vs2_min_pu"}, s.vs2_min_pu);
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
    if (iequals(comp.type, "shaft")) {
      if (iequals(comp.model, "FiveMassShaft") || iequals(comp.model, "FiveMass") ||
          iequals(comp.model, "FiveMassShaftBlock")) {
        auto s = make_five_mass_shaft(comp, mp);
        s->attachMachine(machine->controlLink());
        devices.push_back(std::move(s));
      }
    } else if (iequals(comp.type, "governor") || iequals(comp.type, "turbine_governor")) {
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
      machine->markPssAttached();
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
  if (!load.dynamic_model.empty()) {
    params.phase_power_scale =
        param_or(load.dynamic_model.parameters,
                 {"phase_power_scale", "PhasePowerScale", "phase_scale"},
                 params.phase_power_scale);
    params.nominal_voltage_pu =
        param_or(load.dynamic_model.parameters,
                 {"nominal_voltage_pu", "V_nominal", "V0", "v0"},
                 params.nominal_voltage_pu);
  }
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

// Opt-in: a slack machine that swings like any other synchronous machine
// (dynamic_angle = true) instead of being pinned as an infinite angle
// reference.  Enable per generator via dynamic_model parameters
// {"slack_dynamic_angle", 1} (alias {"dynamic_slack", 1}).  Use for
// multi-machine transient studies where the slack is one of several real
// machines — pinning it would freeze the system frequency and create
// pathological reactive circulation with the dynamic machines.
bool slack_dynamic_angle_enabled(const hacdcpf::DynamicModelProfile& profile) {
  auto flag_in = [](const std::map<std::string, double>& p) {
    if (const auto it = p.find("slack_dynamic_angle"); it != p.end()) {
      return it->second != 0.0;
    }
    if (const auto it = p.find("dynamic_slack"); it != p.end()) {
      return it->second != 0.0;
    }
    return false;
  };
  if (flag_in(profile.parameters)) return true;
  for (const auto& component : profile.components) {
    if (flag_in(component.parameters)) return true;
  }
  return false;
}

// Opt-out counterpart of the above: forces a slack machine to stay pinned
// (dynamic_angle = false) even in a multi-machine system where the builder's
// default rule would let it swing.  dynamic_model parameters
// {"pin_slack", 1} (alias {"slack_pin_angle", 1}).
bool pin_slack_angle_enabled(const hacdcpf::DynamicModelProfile& profile) {
  auto flag_in = [](const std::map<std::string, double>& p) {
    if (const auto it = p.find("pin_slack"); it != p.end()) {
      return it->second != 0.0;
    }
    if (const auto it = p.find("slack_pin_angle"); it != p.end()) {
      return it->second != 0.0;
    }
    return false;
  };
  if (flag_in(profile.parameters)) return true;
  for (const auto& component : profile.components) {
    if (flag_in(component.parameters)) return true;
  }
  return false;
}

bool is_projected_asymmetric_load_equivalent(
    const Load& load,
    const std::vector<AsymmetricLoad>& asymmetric_loads) {
  for (const auto& source : asymmetric_loads) {
    if (!source.in_service || source.bus == 0 || source.bus != load.bus) continue;
    // The projection names the equivalent Load after the source: e4bc1c59
    // switched "name_eq" to the plain source name; accept both conventions so
    // systems projected before/after the switch are de-duplicated either way.
    const bool name_matches =
        source.name.empty()
            ? load.name == "AsymmetricLoad_" + std::to_string(source.index)
            : (load.name == source.name || load.name == source.name + "_eq");
    if (!name_matches) continue;

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

DynamicACBranch make_balanced_ac_branch_shunt(const ACBranch& branch,
                                              const DynamicNetwork& network,
                                              bool per_phase_equivalent) {
  DynamicACBranch dyn;
  dyn.index = branch.index;
  dyn.from_bus = branch.from_bus;
  dyn.to_bus = branch.to_bus;
  dyn.from_pos = network.acBusPosition(branch.from_bus);
  dyn.to_pos = network.acBusPosition(branch.to_bus);
  dyn.in_service = branch.in_service;
  const double scale = per_phase_equivalent ? 1.0 / 3.0 : 1.0;
  const Complex y_shunt(0.0, branch.b_pu / 2.0 * scale);
  const double tap_mag = branch.tap == 0.0 ? 1.0 : branch.tap;
  const Complex tap = std::polar(tap_mag, branch.shift_deg * kDegToRad);
  const Complex tap_conj = std::conj(tap);
  dyn.y_ff = diag3(y_shunt / (tap * tap_conj));
  dyn.y_tt = diag3(y_shunt);
  return dyn;
}

DynamicRLLineParams make_dynamic_rl_line_params(const ACBranch& branch,
                                                const DynamicNetwork& network,
                                                bool per_phase_equivalent) {
  DynamicRLLineParams p;
  p.component_index = branch.index;
  p.from_bus = branch.from_bus;
  p.to_bus = branch.to_bus;
  p.from_pos = network.acBusPosition(branch.from_bus);
  p.to_pos = network.acBusPosition(branch.to_bus);
  p.label = label_or(branch.name, "Dynamic RL line " + std::to_string(branch.index));
  p.base_mva = network.base_mva;
  p.frequency_hz = network.frequency_hz;
  const double scale = per_phase_equivalent ? 3.0 : 1.0;
  p.r_pu = branch.r_pu * scale;
  p.x_pu = branch.x_pu * scale;
  p.in_service = branch.in_service;
  return p;
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
  std::string pf_stage = "power-flow dispatch";
  try {
  if (options.run_power_flow_initialization) {
    // The PF facade owns rich-to-canonical projection and authored-space result
    // recovery. DynamicSolverOptions::project_to_canonical controls only the
    // later dynamic-network representation. A carried ProjectionCertificate is
    // the explicit proof that direct SolverData assembly is legal; without it,
    // bypassing the facade would reinterpret stable bus IDs as vector positions.
    PowerFlowResult pf;
    if (sys.projection_certificate.has_value()) {
      const bool has_isolated_bus =
          std::any_of(sys.ac.buses.begin(), sys.ac.buses.end(),
                      [](const ACBus& bus) {
                        return bus.bus_type == BusType::ISOLATED;
                      }) ||
          std::any_of(sys.dc.buses.begin(), sys.dc.buses.end(),
                      [](const DCBus& bus) {
                        return bus.bus_type == DCBusType::DC_ISOLATED;
                      });
      if (has_isolated_bus) {
        pf_stage = "adaptive projected-island power-flow solve";
        const auto adaptive =
            solve_power_flow_adaptive(sys, options.power_flow_options);
        pf.converged = adaptive.converged;
        pf.iterations = adaptive.iterations;
        pf.residual = adaptive.residual;
        pf.vm = adaptive.vm;
        pf.va = adaptive.va;
        pf.vdc = adaptive.vdc;
        pf.diagnostics = adaptive.diagnostics;
        pf.profiling = adaptive.profiling;
        pf.reactive_limits = adaptive.reactive_limits;
      } else {
        pf_stage = "projected SolverData assembly";
        auto data = powerflow::make_solver_data_projected(
            HybridPowerSystem(sys), options.power_flow_options.loss_model);
        pf_stage = "projected Newton solve";
        powerflow::NewtonSolver solver;
        const InitialState* initial = options.power_flow_options.initial_state
                                          ? &*options.power_flow_options.initial_state
                                          : nullptr;
        pf = solver.solve(data, options.power_flow_options, initial);
      }
    } else {
      pf_stage = "rich-model power-flow facade";
      pf = solve_power_flow(sys, options.power_flow_options);
    }
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
  } catch (const std::exception& error) {
    throw std::runtime_error("nominal power flow failed during " + pf_stage +
                             ": " + error.what());
  } catch (...) {
    throw std::runtime_error(
        "nominal power flow failed during " + pf_stage +
        " with an unrecognized non-standard or cross-ABI exception");
  }
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
    const double scale = std::max(0.0, load.scaling);
    load_power[load.bus] += Complex(load.p_mw * scale, load.q_mvar * scale);
  }
  return load_power;
}

int in_service_external_grid_count_at_bus(const HybridPowerSystem& sys, int bus) {
  return static_cast<int>(
      std::count_if(sys.ac.external_grids.begin(),
                    sys.ac.external_grids.end(),
                    [&](const ExternalGrid& grid) {
                      return grid.in_service && grid.bus == bus;
                    }));
}

bool has_non_external_ac_source_at_bus(const HybridPowerSystem& sys, int bus) {
  const auto at_bus = [&](const auto& device) {
    return device.in_service && device.bus == bus;
  };
  return std::any_of(sys.ac.generators.begin(), sys.ac.generators.end(), at_bus) ||
         std::any_of(sys.ac.static_generators.begin(),
                     sys.ac.static_generators.end(),
                     at_bus) ||
         std::any_of(sys.ac.pv_systems.begin(), sys.ac.pv_systems.end(), at_bus) ||
         std::any_of(sys.ac.renewable_gens.begin(),
                     sys.ac.renewable_gens.end(),
                     at_bus) ||
         std::any_of(sys.ac.storage.begin(), sys.ac.storage.end(), at_bus);
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
  std::string build_stage = "canonical projection";
  try {
  dyn.options = options;
  dyn.canonical_system =
      options.project_to_canonical
          ? projection::RichToCanonicalOperator::apply(sys).canonical
          : sys;
  // Authored -> canonical bus maps so transient events authored in the caller's
  // bus-id space resolve after canonical projection renumbers non-contiguous DC
  // bus ids and may merge AC buses. Mirrors resilience_dynamic_certification's
  // canonical_bus_ids: AC prefers the bus_merge_map, both domains fall back to
  // positional correspondence (DC canonicalization preserves bus order/count).
  {
    auto& ac_map = dyn.network.authored_to_canonical_ac_bus;
    auto& dc_map = dyn.network.authored_to_canonical_dc_bus;
    if (dyn.canonical_system.bus_merge_map) {
      for (const auto& [external, internal] :
           dyn.canonical_system.bus_merge_map->ext_to_int) {
        ac_map[external] = internal + 1;
      }
    } else {
      const std::size_t n =
          std::min(sys.ac.buses.size(), dyn.canonical_system.ac.buses.size());
      for (std::size_t i = 0; i < n; ++i)
        ac_map[sys.ac.buses[i].index] = dyn.canonical_system.ac.buses[i].index;
    }
    const std::size_t ndc =
        std::min(sys.dc.buses.size(), dyn.canonical_system.dc.buses.size());
    for (std::size_t i = 0; i < ndc; ++i)
      dc_map[sys.dc.buses[i].index] = dyn.canonical_system.dc.buses[i].index;
  }
  build_stage = "nominal power-flow initialization";
  dyn.initial_power_flow = nominal_power_flow(
      options.project_to_canonical ? sys : dyn.canonical_system, options);
  dyn.initialization = make_initialization_summary(options, dyn.initial_power_flow);
  build_stage = "dynamic network topology assembly";

  // Multi-machine rule (PSD semantics): with >= 2 in-service synchronous
  // machines a slack machine is one of several real machines and swings with
  // them — pinning it would freeze the system frequency and create
  // pathological reactive circulation.  Per-generator flags always win:
  // slack_dynamic_angle/dynamic_slack forces a swing, pin_slack/slack_pin_angle
  // forces a pin.  Single-machine systems keep the pinned infinite-bus
  // behavior by default.
  const int n_sync_machines = [&] {
    int n = 0;
    for (const auto& g : dyn.canonical_system.ac.generators) {
      if (g.in_service) ++n;
    }
    if (dyn.canonical_system.three_phase_ac) {
      for (const auto& g : dyn.canonical_system.three_phase_ac->generators) {
        if (g.in_service) ++n;
      }
    }
    return n;
  }();
  const bool multimachine_slack_swings = n_sync_machines >= 2;
  const auto dynamic_angle_for = [&](const auto& gen) {
    if (slack_dynamic_angle_enabled(gen.dynamic_model)) return true;
    if (pin_slack_angle_enabled(gen.dynamic_model)) return false;
    return !gen.is_slack || multimachine_slack_swings;
  };

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
      const bool dynamic_rl_supported =
          branch.dynamic_rl &&
          nearly_equal(branch.tap == 0.0 ? 1.0 : branch.tap, 1.0) &&
          nearly_equal(branch.shift_deg, 0.0) &&
          std::abs(branch.x_pu) > 1e-12;
      if (branch.dynamic_rl && !dynamic_rl_supported) {
        dyn.warnings.push_back(
            "AC branch " + std::to_string(branch.index) +
            " requested dynamic_rl but has unsupported tap/shift or zero reactance; using static admittance");
      }
      if (dynamic_rl_supported) {
        network.ac_branches.push_back(
            make_balanced_ac_branch_shunt(branch, network, pf_bus_injection_init));
        dyn.devices.push_back(std::make_unique<DynamicRLLine>(
            make_dynamic_rl_line_params(branch, network, pf_bus_injection_init)));
      } else {
        network.ac_branches.push_back(
            make_balanced_ac_branch(branch,
                                    network,
                                    options.min_branch_impedance_pu,
                                    pf_bus_injection_init));
      }
    }
  }

  for (const auto& branch : dyn.canonical_system.dc.branches) {
    network.dc_branches.push_back(make_dc_branch(branch, network, options.min_branch_impedance_pu));
  }
  build_stage = "dynamic network matrix assembly";
  network.rebuildBaseMatrices(options.singular_regularization_pu);
  initialize_network_voltages(dyn.canonical_system, dyn.initial_power_flow, dyn);
  auto pf_ac_bus_device_injection_mva =
      ac_device_injection_from_network_pf(network, dyn.y);
  for (const auto& [bus, load_power] :
       explicit_ac_load_power_by_bus(dyn.canonical_system)) {
    pf_ac_bus_device_injection_mva[bus] += load_power;
  }
  auto pf_ac_device_injection_mva =
      pf_bus_injection_init ? pf_ac_bus_device_injection_mva
                            : std::unordered_map<int, Complex>{};

  const double base_mva = network.base_mva;
  const double ac_phase_power_scale = pf_bus_injection_init ? (1.0 / 3.0) : 1.0;

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
  build_stage = "AC dynamic device assembly";
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
        p.dynamic_angle = dynamic_angle_for(gen);
        p.network_balance_reference = gen.is_slack;
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
        p.phase_domain_control = true;
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
    if (profile_model_is(grid.dynamic_model, {"PeriodicVariableSource"})) {
      dyn.devices.push_back(std::make_unique<PeriodicVariableSourceDynamic>(
          make_periodic_source_params(grid, bus_pos, base_mva)));
      voltage_source_buses.insert(grid.bus);
      continue;
    }
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
    p.phase_power_scale = ac_phase_power_scale;
    p.in_service = true;
    apply_voltage_source_profile(grid.dynamic_model, p);
    if (in_service_external_grid_count_at_bus(dyn.canonical_system, grid.bus) == 1 &&
        !has_non_external_ac_source_at_bus(dyn.canonical_system, grid.bus)) {
      const auto inj = pf_ac_bus_device_injection_mva.find(grid.bus);
      if (inj != pf_ac_bus_device_injection_mva.end()) {
        p.p_mech_mw = inj->second.real();
        p.q_elec_mvar = inj->second.imag();
      }
    }
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
    p.phase_power_scale = ac_phase_power_scale;
    // A slack machine's authored pg_mw is not its operating point — the power
    // flow adjusts the slack injection.  When the machine will actually swing
    // (multi-machine rule or explicit slack_dynamic_angle), seed p_mech from
    // the PF-solved injection at single-generator buses; at multi-generator
    // buses the injection cannot be attributed, so the authored value stands.
    // Pinned machines keep the legacy authored seed entirely (invisible in
    // their frozen swing anyway) so their calibrated behavior is unchanged.
    const bool machine_will_swing = dynamic_angle_for(gen);
    const bool single_gen_at_bus =
        std::count_if(dyn.canonical_system.ac.generators.begin(),
                      dyn.canonical_system.ac.generators.end(),
                      [&](const Generator& other) {
                        return other.in_service && other.bus == gen.bus;
                      }) == 1;
    if (gen.is_slack && machine_will_swing && single_gen_at_bus) {
      // pf_ac_bus_device_injection_mva is always populated (unlike
      // pf_ac_device_injection_mva, which is empty when the
      // pf_bus_injection_init style is off).
      const auto inj = pf_ac_bus_device_injection_mva.find(gen.bus);
      if (inj != pf_ac_bus_device_injection_mva.end()) {
        p.p_mech_mw = inj->second.real();
        p.q_elec_mvar = inj->second.imag();
      }
    } else if (pf_bus_injection_init && single_gen_at_bus) {
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
    p.dynamic_angle = dynamic_angle_for(gen);
    p.network_balance_reference = gen.is_slack;
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
    // Asynchronous motors are projected into canonical loads for the power-flow
    // network, but their electromechanical dynamics are rebuilt from the
    // original sys.ac.motors below. Skip the static-load stand-in here so the
    // motor demand is not counted twice.
    if (load.sc_source_type == "AsynchronousMotor") continue;
    const int bus_pos = network.acBusPosition(load.bus);
    if (bus_pos < 0) continue;
    if (profile_model_is(load.dynamic_model,
                         {"ActiveConstantPowerLoad", "ActiveCPL", "CPL"})) {
      ActiveConstantPowerLoadParams cp;
      cp.component_index = load.index;
      cp.bus = load.bus;
      cp.bus_pos = bus_pos;
      cp.label = label_or(load.name, "ActiveCPL " + std::to_string(load.index));
      cp.base_mva = base_mva;
      cp.p_mw = load.p_mw * load.scaling;
      cp.q_mvar = load.q_mvar * load.scaling;
      const auto cpmap = merged_profile_parameters(load.dynamic_model);
      cp.filter_r_pu = param_or(cpmap, {"filter_r_pu", "Rf", "rf"}, cp.filter_r_pu);
      cp.filter_x_pu = param_or(cpmap, {"filter_x_pu", "Xf", "xf"}, cp.filter_x_pu);
      cp.filter_c_s = param_or(cpmap, {"filter_c_s", "Cf", "cf", "Tc"}, cp.filter_c_s);
      cp.v_min_pu = param_or(cpmap, {"v_min_pu", "Vmin", "vmin"}, cp.v_min_pu);
      cp.model_profiles = to_dynamic_profiles(load.dynamic_model);
      dyn.devices.push_back(std::make_unique<ActiveConstantPowerLoadDynamic>(cp));
      continue;
    }
    ACLoadDynamicParams p;
    p.component_index = load.index;
    p.bus = load.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(load.name, "AC load " + std::to_string(load.index));
    p.canvas_type = "load";
    p.source_type = "ac_load";
    p.p_mw = load.p_mw;
    p.q_mvar = load.q_mvar;
    p.scale = std::max(0.0, load.scaling);
    if (const auto it = load.dynamic_model.parameters.find(
            "resilience_service_base_scale");
        it != load.dynamic_model.parameters.end()) {
      p.service_base_scale = std::max(0.0, it->second);
    } else {
      p.service_base_scale = p.scale;
    }
    p.base_mva = base_mva;
    p.phase_power_scale = ac_phase_power_scale;
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
    p.scale = std::max(0.0, load.scaling);
    if (const auto it = load.dynamic_model.parameters.find(
            "resilience_service_base_scale");
        it != load.dynamic_model.parameters.end()) {
      p.service_base_scale = std::max(0.0, it->second);
    } else {
      p.service_base_scale = p.scale;
    }
    p.base_mva = base_mva;
    p.model_profiles = to_dynamic_profiles(load.dynamic_model);
    dyn.devices.push_back(std::make_unique<ThreePhaseDynamicLoad>(p));
  }

  // Iterate the ORIGINAL motors: project_to_canonical_models() moves motors
  // into equivalent loads and clears ac.motors, so the canonical system never
  // carries them. The equivalent loads are skipped in the load loop above.
  for (const auto& motor : sys.ac.motors) {
    if (using_explicit_three_phase) break;
    if (!motor.in_service) continue;
    const int bus_pos = network.acBusPosition(motor.bus);
    if (bus_pos < 0) continue;
    auto motor_params = make_induction_machine_params(motor, bus_pos, base_mva);
    if (motor_params.flux_model) {
      dyn.devices.push_back(std::make_unique<FluxInductionMachineDynamic>(
          std::move(motor_params)));
    } else {
      dyn.devices.push_back(std::make_unique<InductionMachineDynamic>(
          std::move(motor_params)));
    }
  }

  for (const auto& gen : dyn.canonical_system.ac.static_generators) {
    if (!gen.in_service) continue;
    const int bus_pos = network.acBusPosition(gen.bus);
    if (bus_pos < 0) continue;
    if (profile_model_is(gen.dynamic_model, {"CSVGN1"})) {
      dyn.devices.push_back(std::make_unique<CSVGN1Dynamic>(
          make_csvgn1_params(gen, bus_pos, base_mva)));
      continue;
    }
    if (profile_model_is(gen.dynamic_model,
                         {"AggregateDistributedGenerationA", "DERA", "DERA_A"})) {
      dyn.devices.push_back(std::make_unique<DERAADynamic>(
          make_deraa_params(gen, bus_pos, base_mva)));
      continue;
    }
    if (profile_model_is(gen.dynamic_model,
                         {"RenewableEnergyGeneratorA", "REGCA", "REGC_A"})) {
      dyn.devices.push_back(std::make_unique<REGCADynamic>(
          make_regca_params(gen, bus_pos, base_mva)));
      continue;
    }
    if (gen.grid_forming || profile_model_is(gen.dynamic_model,
                         {"GridFormingNortonDroop",
                          "GFMDroopOuterControl",
                          "VirtualInertia",
                          "VSM",
                          "VSMGridForming",
                          "VSMOuterControl",
                          "ActiveVirtualOscillator",
                          "ReactiveVirtualOscillator",
                          "VOC",
                          "VOCGridForming",
                          "VOCOuterControl"})) {
      GridFormingInverterParams p;
      p.component_index = gen.index;
      p.bus = gen.bus;
      p.bus_pos = bus_pos;
      p.label = label_or(gen.name, "Grid-forming static generator " + std::to_string(gen.index));
      p.device_type = "VSCGridForming";
      p.canvas_type = "sgen";
      p.source_type = "static_generator_grid_forming";
      p.base_mva = base_mva;
      p.phase_domain_control = using_explicit_three_phase;
      p.p_ref_mw = gen.p_mw * gen.scaling;
      p.q_ref_mvar = gen.q_mvar * gen.scaling;
      p.v_ref_pu = positive_or(gen.v_ref_pu, 1.0);
      p.virtual_x_pu = options.inverter_virtual_reactance_pu;
      p.frequency_hz = positive_or(gen.f_ref_hz, network.frequency_hz);
      p.current_limit_pu = gen.sn_mva > 0.0
                               ? gen.sn_mva / base_mva
                               : (gen.p_rated_mw > 0.0 ? gen.p_rated_mw / base_mva : 0.0);
      p.pmax_mw = gen.pmax_mw > 0.0 ? gen.pmax_mw
                                     : (gen.p_rated_mw > 0.0 ? gen.p_rated_mw : 0.0);
      p.pmin_mw = gen.pmin_mw < 0.0 ? gen.pmin_mw
                                     : (p.pmax_mw > 0.0 ? -p.pmax_mw : 0.0);
      p.p_droop_pu = positive_or(gen.k_p, p.p_droop_pu);
      p.q_droop_pu = positive_or(gen.k_q, p.q_droop_pu);
      apply_gfm_profile(gen.dynamic_model, p);
      dyn.devices.push_back(std::make_unique<GridFormingInverter>(p));
      voltage_source_buses.insert(gen.bus);
      continue;
    }
    GridFollowingInverterParams p;
    p.component_index = gen.index;
    p.bus = gen.bus;
    p.bus_pos = bus_pos;
    p.label = label_or(gen.name, "Static generator " + std::to_string(gen.index));
    p.device_type = "StaticGenerator";
    p.canvas_type = "sgen";
    p.source_type = "static_generator";
    p.base_mva = base_mva;
    p.phase_domain_control = using_explicit_three_phase;
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
    p.phase_domain_control = using_explicit_three_phase;
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
    p.phase_domain_control = using_explicit_three_phase;
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
      p.phase_domain_control = using_explicit_three_phase;
      p.p_ref_mw = st.p_mw;
      p.q_ref_mvar = st.q_mvar;
      // The solver equations use the network MVA base.  The default virtual
      // impedance is a device-base inverter parameter, so convert it before
      // stamping.  An explicit dynamic-model value applied below retains the
      // existing system-base override semantics.
      const double device_base_mva =
          positive_or(st.p_rated_mw,
                      std::max({std::abs(st.pmax_mw), std::abs(st.pmin_mw), 1.0}));
      const double device_to_system_base = base_mva / device_base_mva;
      p.virtual_x_pu =
          options.inverter_virtual_reactance_pu * device_to_system_base;
      p.p_droop_pu *= device_to_system_base;
      p.q_droop_pu *= device_to_system_base;
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
    build_stage = "DC load device assembly";
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
    p.p_mw = load.p_mw;
    p.scale = std::max(0.0, load.scaling);
    if (const auto it = load.dynamic_model.parameters.find(
            "resilience_service_base_scale");
        it != load.dynamic_model.parameters.end()) {
      p.service_base_scale = std::max(0.0, it->second);
    } else {
      p.service_base_scale = p.scale;
    }
    p.base_mva = base_mva;
    p.model_profiles = to_dynamic_profiles(load.dynamic_model);
    dyn.devices.push_back(std::make_unique<DCDynamicLoad>(p));
  }

  for (const auto& gen : dyn.canonical_system.dc.dc_static_generators) {
    build_stage = "DC static-generator device assembly";
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
    build_stage = "DC PV device assembly";
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
    build_stage = "DC storage device assembly";
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
    build_stage = "VSC dynamic device assembly";
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
      p.phase_domain_control = using_explicit_three_phase;
      p.p_ref_mw = conv.p_schedule_mw != 0.0 ? conv.p_schedule_mw : conv.p_set_mw;
      p.q_ref_mvar = conv.q_set_mvar;
      p.v_ref_pu = positive_or(conv.v_ac_set_pu, positive_or(conv.v_ref_pu, 1.0));
      const auto gfm = model::resolve_gfm_norton_parameters(conv);
      p.v_ref_pu = gfm.internal_voltage_pu;
      p.angle_ref_rad = gfm.internal_angle_rad;
      p.virtual_r_pu = gfm.virtual_r_pu;
      p.virtual_x_pu = gfm.virtual_x_pu;
      switch (conv.current_limit_priority) {
        case VSCCurrentLimitPriority::Magnitude:
          p.limiter_kind = CurrentLimiterKind::Magnitude;
          p.reactive_current_priority = false;
          break;
        case VSCCurrentLimitPriority::ActivePower:
          p.limiter_kind = CurrentLimiterKind::ActivePriority;
          p.reactive_current_priority = false;
          break;
        case VSCCurrentLimitPriority::ReactivePower:
          p.limiter_kind = CurrentLimiterKind::ReactivePriority;
          p.reactive_current_priority = true;
          break;
      }
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
      p.phase_domain_control = using_explicit_three_phase;
      p.p_ref_mw = conv.p_schedule_mw != 0.0 ? conv.p_schedule_mw : conv.p_set_mw;
      p.q_ref_mvar = conv.q_set_mvar;
      p.current_limit_pu = conv.i_ac_max_pu;
      // VSCConverter::current_limit_priority is a system-base current-control
      // contract (converter_components.hpp). Preserve the same geometry used
      // by limited_current() for GFL and GFM dynamic ports.
      switch (conv.current_limit_priority) {
        case VSCCurrentLimitPriority::Magnitude:
          p.limiter_kind = CurrentLimiterKind::Magnitude;
          p.reactive_current_priority = false;
          break;
        case VSCCurrentLimitPriority::ActivePower:
          p.limiter_kind = CurrentLimiterKind::ActivePriority;
          p.reactive_current_priority = false;
          break;
        case VSCCurrentLimitPriority::ReactivePower:
          p.limiter_kind = CurrentLimiterKind::ReactivePriority;
          p.reactive_current_priority = true;
          break;
      }
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
    build_stage = "DC/DC dynamic device assembly";
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
    p.control_mode = conv.control_mode;
    p.p_ref_mw = conv.p_ref_mw;
    p.v_ref_pu = positive_or(conv.v_ref_pu, 1.0);
    p.sn_mva = conv.sn_mva;
    p.eta = positive_or(conv.eta, 0.98);
    p.r_eq_pu = std::max(0.0, conv.r_eq_pu);
    p.pmax_mw = conv.pmax_mw;
    p.pmin_mw = conv.pmin_mw;
    p.k_droop = conv.k_droop;
    apply_dcdc_profile(conv.dynamic_model, p);
    dyn.devices.push_back(std::make_unique<DCDCConverterDynamic>(p));
  }

  build_stage = "dynamic state indexing and initialization";
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
  } catch (const std::exception& error) {
    throw std::runtime_error("DynamicModelBuilder failed during " + build_stage +
                             ": " + error.what());
  } catch (const std::string& error) {
    throw std::runtime_error("DynamicModelBuilder failed during " + build_stage +
                             ": " + error);
  } catch (const char* error) {
    throw std::runtime_error(
        "DynamicModelBuilder failed during " + build_stage + ": " +
        (error == nullptr ? std::string("null C-string exception")
                          : std::string(error)));
  } catch (...) {
    throw std::runtime_error(
        "DynamicModelBuilder failed during " + build_stage +
        " with an unrecognized non-standard or cross-ABI exception");
  }
}

}  // namespace hacdcpf::dynamics
