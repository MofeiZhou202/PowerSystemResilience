#include "hacdcpf/power_flow/lcc_model.hpp"

#include <algorithm>
#include <cmath>

namespace hacdcpf::powerflow {

namespace {

constexpr double kPi = 3.14159265358979323846;
// 3*sqrt(2)/pi: DC voltage of a 6-pulse bridge per kV of valve-side line voltage.
constexpr double kUd0Factor = 4.24264068711928516 / kPi;  // 1.35057...
constexpr double kThreeOverPi = 3.0 / kPi;
// Floor for the commutation reactance in the voltage-source characteristic
/// modes: with X_c = 0 the CEA/constant-alpha station degenerates to an ideal
/// voltage source; the floor keeps the current (and its derivative) finite.
/// A missing X_c (converter transformer not identified) is reported by the
/// pre-solve readiness check, so this floor is only a numerical guard.
constexpr double kMinXCommOhm = 0.05;
constexpr double kMinUdKv = 1.0;
constexpr double kCurrentLimitDerivativeHysteresis = 1e-8;

double deg_to_rad(double deg) { return deg * kPi / 180.0; }
double rad_to_deg(double rad) { return rad * 180.0 / kPi; }

struct StationParams {
  double ud0_terminal_kv{0.0};     // fundamental-current/Q voltage
  double ud0_commutation_kv{0.0};  // alpha/gamma characteristic voltage
  double rc_ohm{0.0};   // (3/pi)*n_b*X_c equivalent commutation resistance
  double vdrop_kv{0.0}; // n_b * valve forward drop
  double id_max_ka{0.0};// rated DC current (0 = unlimited)
};

StationParams station_params(const LCCConverter& conv,
                             double terminal_e_kv,
                             double commutation_e_kv) {
  StationParams p;
  const double nb = static_cast<double>(std::max(1, conv.n_bridges));
  p.ud0_terminal_kv =
      kUd0Factor * nb * std::max(terminal_e_kv, 0.0);
  p.ud0_commutation_kv =
      kUd0Factor * nb * std::max(commutation_e_kv, 0.0);
  p.rc_ohm = kThreeOverPi * nb * std::max(conv.x_comm_ohm, kMinXCommOhm);
  p.vdrop_kv = nb * conv.v_drop_v * 1e-3;
  // BridgeIn is a bridge nameplate/reporting value, not a steady-state
  // current order. The actual order comes from the LD/DC/BM controls
  // (scheduled P, scheduled I, alpha/gamma, or Udc). Keep the LCC
  // unidirectional guard below, but do not silently replace the declared
  // control mode with constant-current operation at the nameplate value.
  p.id_max_ka = 0.0;
  return p;
}

// Enforce the physical unidirectional current range. id_max remains available
// to generic callers, but BPA/DSP BridgeIn does not populate it because that
// field is a nameplate rather than a steady-state current order.
double clamp_id(double id_ka, double id_max_ka, bool& at_limit) {
  at_limit = false;
  if (id_ka < 0.0) {
    at_limit = true;
    return 0.0;
  }
  if (id_max_ka > 0.0 &&
      id_ka >= id_max_ka * (1.0 - 1e-10)) {
    at_limit = true;
    return std::min(id_ka, id_max_ka);
  }
  return id_ka;
}

double requested_id_ka(const LCCConverter& conv,
                       const StationParams& p,
                       double ud_kv,
                       bool& supported) {
  supported = true;
  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  switch (conv.control_mode) {
    case LCCControlMode::ConstantPower:
      return (conv.p_set_mw > 0.0) ? conv.p_set_mw / ud_kv : 0.0;
    case LCCControlMode::ConstantCurrent:
      return conv.i_set_ka;
    case LCCControlMode::ConstantAlpha:
      if (!rectifier) {
        supported = false;
        return 0.0;
      }
      return (p.ud0_commutation_kv *
                  std::cos(deg_to_rad(conv.alpha_set_deg)) -
              p.vdrop_kv - ud_kv) /
             p.rc_ohm;
    case LCCControlMode::ConstantGamma:
      if (rectifier) {
        supported = false;
        return 0.0;
      }
      return (p.ud0_commutation_kv *
                  std::cos(deg_to_rad(conv.gamma_set_deg)) +
              p.vdrop_kv - ud_kv) /
             p.rc_ohm;
  }
  supported = false;
  return 0.0;
}

bool current_limit_strictly_binds(const LCCConverter& conv,
                                  const StationParams& p,
                                  double ud_kv) {
  bool supported = false;
  const double requested = requested_id_ka(conv, p, ud_kv, supported);
  if (!supported) return false;
  const double scale = std::max(1.0, p.id_max_ka);
  if (requested < -kCurrentLimitDerivativeHysteresis * scale) return true;
  return p.id_max_ka > 0.0 &&
         requested > p.id_max_ka *
                         (1.0 + kCurrentLimitDerivativeHysteresis);
}

// Reactive consumption: Q = |P| * tan(phi), cos(phi) ~= U_d/U_d0 (manual ch.4
// approximate formula — the same one the DSP reference uses).  The ratio is
// clamped to [-1,1] so transient iterates with U_d > U_d0 cannot produce NaN.
double lcc_q_mvar(double p_abs_mw, double ud_kv, double ud0_kv) {
  if (ud0_kv <= 1e-9 || p_abs_mw <= 0.0) return 0.0;
  const double cos_phi = std::clamp(ud_kv / ud0_kv, -1.0, 1.0);
  const double sin2 = 1.0 - cos_phi * cos_phi;
  if (sin2 <= 1e-12 || std::abs(cos_phi) <= 1e-9) return 0.0;
  return p_abs_mw * std::sqrt(sin2) / std::abs(cos_phi);
}

// Back-calculated firing angle from the rectifier external characteristic
//   U_d = U_d0*cos(alpha) - R_c*I_d - n_b*dU_v.
// Sets alpha_beyond_range when the argument of acos leaves [-1,1]: with the
// tap fixed the available U_d0 cannot reach the required U_d even at
// alpha = 0. The PF API's R-card outer loop may respond by changing the bound
// T-card tap; this local station evaluator itself remains fixed-tap.
double back_calc_alpha(double ud_kv, double id_ka, const StationParams& p,
                       bool& beyond) {
  if (p.ud0_commutation_kv <= 1e-9) {
    beyond = true;
    return 0.0;
  }
  const double arg =
      (ud_kv + p.rc_ohm * id_ka + p.vdrop_kv) /
      p.ud0_commutation_kv;
  beyond = arg < -1.0 || arg > 1.0;
  return rad_to_deg(std::acos(std::clamp(arg, -1.0, 1.0)));
}

// Back-calculated extinction angle from the inverter external characteristic
//   U_d = U_d0*cos(gamma) - R_c*I_d + n_b*dU_v.
double back_calc_gamma(double ud_kv, double id_ka, const StationParams& p,
                       bool& beyond) {
  if (p.ud0_commutation_kv <= 1e-9) {
    beyond = true;
    return 0.0;
  }
  const double arg =
      (ud_kv + p.rc_ohm * id_ka - p.vdrop_kv) /
      p.ud0_commutation_kv;
  beyond = arg < -1.0 || arg > 1.0;
  return rad_to_deg(std::acos(std::clamp(arg, -1.0, 1.0)));
}

}  // namespace

LCCOperatingPoint lcc_operating_point(const LCCConverter& conv,
                                      double e_kv,
                                      double ud_kv) {
  return lcc_operating_point(conv, e_kv, e_kv, ud_kv);
}

LCCOperatingPoint lcc_operating_point(const LCCConverter& conv,
                                      double terminal_e_kv,
                                      double commutation_e_kv,
                                      double ud_kv) {
  LCCOperatingPoint op;
  if (!std::isfinite(terminal_e_kv) ||
      !std::isfinite(commutation_e_kv) || !std::isfinite(ud_kv)) {
    return op;
  }
  if (conv.control_mode == LCCControlMode::ConstantPower &&
      (!std::isfinite(conv.p_set_mw) || conv.p_set_mw <= 0.0)) {
    return op;
  }
  op.ud_at_floor = ud_kv <= kMinUdKv;
  ud_kv = std::max(ud_kv, kMinUdKv);

  const StationParams p =
      station_params(conv, terminal_e_kv, commutation_e_kv);
  op.ud0_kv = p.ud0_terminal_kv;
  op.ud_kv = ud_kv;
  if (p.ud0_terminal_kv <= 1e-9 || p.ud0_commutation_kv <= 1e-9) {
    return op;
  }

  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  bool supported = false;
  const double requested = requested_id_ka(conv, p, ud_kv, supported);
  if (!supported) return op;
  const double id = clamp_id(requested, p.id_max_ka, op.id_at_limit);
  op.id_ka = id;

  // Power at the DC terminal (magnitude); the rectifier injects into the DC
  // network, the inverter draws from it.
  const double p_term_mw = ud_kv * id;
  op.p_dc_mw = rectifier ? p_term_mw : -p_term_mw;
  op.p_ac_mw = rectifier ? -p_term_mw : p_term_mw;
  op.q_ac_mvar =
      -lcc_q_mvar(p_term_mw, ud_kv, p.ud0_terminal_kv);

  bool beyond = false;
  op.alpha_deg = back_calc_alpha(ud_kv, id, p, beyond);
  op.alpha_beyond_range = beyond;
  op.gamma_deg = back_calc_gamma(ud_kv, id, p, beyond);
  op.gamma_beyond_range = beyond;
  op.valid = true;
  return op;
}

double lcc_required_valve_voltage_kv(const LCCConverter& conv,
                                     double ud_kv,
                                     double id_ka,
                                     double angle_deg) {
  if (!std::isfinite(ud_kv) || !std::isfinite(id_ka) ||
      !std::isfinite(angle_deg) || ud_kv <= 0.0 || id_ka < 0.0) {
    return 0.0;
  }
  const double nb = static_cast<double>(std::max(1, conv.n_bridges));
  const double xc = std::max(conv.x_comm_ohm, kMinXCommOhm);
  const double rc_ohm = kThreeOverPi * nb * xc;
  const double vdrop_kv = nb * conv.v_drop_v * 1e-3;
  const double cos_angle = std::cos(deg_to_rad(angle_deg));
  if (cos_angle <= 1e-9) return 0.0;

  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  const double numerator =
      ud_kv + rc_ohm * id_ka + (rectifier ? vdrop_kv : -vdrop_kv);
  const double ud0_kv = numerator / cos_angle;
  return ud0_kv / (kUd0Factor * nb);
}

namespace {

const ACBranch* lcc_transformer_branch(const SolverData& data,
                                       const LCCConverter& conv) {
  const auto it = std::find_if(
      data.ac_branches.begin(), data.ac_branches.end(),
      [&](const ACBranch& branch) {
        return branch.index == conv.converter_transformer_branch;
      });
  if (it == data.ac_branches.end() || !it->in_service ||
      !std::isfinite(it->tap) || !(it->tap > 0.0)) {
    return nullptr;
  }
  if (it->from_bus != conv.ac_bus && it->to_bus != conv.ac_bus) {
    return nullptr;
  }
  return &*it;
}

}  // namespace

int lcc_commutation_ac_bus(const SolverData& data,
                           const LCCConverter& conv) {
  const ACBranch* branch = lcc_transformer_branch(data, conv);
  if (branch == nullptr) return conv.ac_bus - 1;
  return (branch->to_bus == conv.ac_bus ? branch->from_bus : branch->to_bus) -
         1;
}

double lcc_commutation_voltage_sensitivity_kv_per_pu(
    const SolverData& data, const LCCConverter& conv) {
  if (!(conv.vn_ac_kv > 0.0)) return 0.0;
  const ACBranch* branch = lcc_transformer_branch(data, conv);
  if (branch == nullptr) return conv.vn_ac_kv;
  return branch->to_bus == conv.ac_bus
             ? conv.vn_ac_kv / branch->tap
             : conv.vn_ac_kv * branch->tap;
}

double lcc_commutation_voltage_kv(const SolverData& data,
                                  const LCCConverter& conv,
                                  const Eigen::VectorXd& vm) {
  const int bus = lcc_commutation_ac_bus(data, conv);
  if (bus < 0 || bus >= static_cast<int>(vm.size())) return 0.0;
  return std::max(vm[bus], 0.0) *
         lcc_commutation_voltage_sensitivity_kv_per_pu(data, conv);
}

double lcc_dc_base_kv(const SolverData& data, const LCCConverter& conv) {
  const int dc_pos = conv.dc_bus - 1;
  if (dc_pos >= 0 && dc_pos < static_cast<int>(data.dc_buses.size())) {
    const double base = data.dc_buses[static_cast<size_t>(dc_pos)].base_kv;
    if (base > 0.0) return base;
  }
  return (conv.rated_dc_kv > 0.0) ? conv.rated_dc_kv : 1.0;
}

namespace {

// Resolve the station's AC/DC bus references (canonical 1-based positions)
// and evaluate its operating point at the current iterate.  Returns false
// when the station cannot inject (out of service, unresolved bus, invalid
// config).
bool lcc_eval_at_state(const SolverData& data,
                       const LCCConverter& conv,
                       const Eigen::VectorXd& vm,
                       const Eigen::VectorXd& vdc,
                       LCCOperatingPoint& op) {
  if (!conv.in_service) return false;
  const int ac_pos = conv.ac_bus - 1;
  const int dc_pos = conv.dc_bus - 1;
  if (ac_pos < 0 || ac_pos >= static_cast<int>(vm.size())) return false;
  if (dc_pos < 0 || dc_pos >= static_cast<int>(vdc.size())) return false;
  if (conv.vn_ac_kv <= 0.0) return false;
  const double terminal_e_kv =
      std::max(vm[ac_pos], 0.0) * conv.vn_ac_kv;
  const double commutation_e_kv =
      lcc_commutation_voltage_kv(data, conv, vm);
  const double ud_kv = vdc[dc_pos] * lcc_dc_base_kv(data, conv);
  op = lcc_operating_point(conv, terminal_e_kv, commutation_e_kv, ud_kv);
  return op.valid;
}

}  // namespace

std::pair<double, double> lcc_ac_injection(const SolverData& data,
                                           const LCCConverter& conv,
                                           const Eigen::VectorXd& vm,
                                           const Eigen::VectorXd& vdc) {
  LCCOperatingPoint op;
  if (!lcc_eval_at_state(data, conv, vm, vdc, op)) return {0.0, 0.0};
  return {op.p_ac_mw / data.base_mva, op.q_ac_mvar / data.base_mva};
}

double lcc_dc_injection(const SolverData& data,
                        const LCCConverter& conv,
                        const Eigen::VectorXd& vm,
                        const Eigen::VectorXd& vdc) {
  LCCOperatingPoint op;
  if (!lcc_eval_at_state(data, conv, vm, vdc, op)) return 0.0;
  if (op.ud_at_floor) return 0.0;
  return op.p_dc_mw / data.base_mva;
}

double lcc_dc_jacobian_vdc(const SolverData& data,
                           const LCCConverter& conv,
                           const Eigen::VectorXd& vm,
                           const Eigen::VectorXd& vdc) {
  LCCOperatingPoint op;
  if (!lcc_eval_at_state(data, conv, vm, vdc, op)) return 0.0;
  if (op.ud_at_floor) return 0.0;

  const double base_kv = lcc_dc_base_kv(data, conv);
  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  const double terminal_e_kv =
      std::max(vm[conv.ac_bus - 1], 0.0) * conv.vn_ac_kv;
  const StationParams p = station_params(
      conv, terminal_e_kv, lcc_commutation_voltage_kv(data, conv, vm));
  // At a true current boundary, use the matching one-sided derivative.
  if (current_limit_strictly_binds(conv, p, op.ud_kv)) {
    // Current clamped at the rated/blocking limit: the injection degenerates
    // to constant current (pdc = ±U_d * I_lim), whose slope is ±I_lim.  A
    // blocked station (I_d = 0) has zero slope.
    return (rectifier ? op.id_ka : -op.id_ka) * base_kv / data.base_mva;
  }
  double dpdc_dud = 0.0;  // MW per kV
  switch (conv.control_mode) {
    case LCCControlMode::ConstantGamma: {
      // pdc = -U_d*(K - U_d)/R_c with K = U_d0*cos(gamma) + n_b*dU_v.
      const double k = p.ud0_commutation_kv *
                           std::cos(deg_to_rad(conv.gamma_set_deg)) +
                       p.vdrop_kv;
      dpdc_dud = (2.0 * op.ud_kv - k) / p.rc_ohm;
      break;
    }
    case LCCControlMode::ConstantAlpha: {
      // pdc = +U_d*(K' - U_d)/R_c with K' = U_d0*cos(alpha) - n_b*dU_v.
      const double k = p.ud0_commutation_kv *
                           std::cos(deg_to_rad(conv.alpha_set_deg)) -
                       p.vdrop_kv;
      dpdc_dud = (k - 2.0 * op.ud_kv) / p.rc_ohm;
      break;
    }
    case LCCControlMode::ConstantCurrent:
      dpdc_dud = rectifier ? op.id_ka : -op.id_ka;
      break;
    case LCCControlMode::ConstantPower:
      return 0.0;
  }
  // MW/kV -> pu(pdc)/pu(vdc): multiply by base_kv / base_mva.
  return dpdc_dud * base_kv / data.base_mva;
}

bool lcc_forms_dc_voltage(const LCCConverter& conv) {
  if (!conv.in_service) return false;
  // Characteristic (voltage-source-behind-X_c) control modes anchor the DC
  // voltage of their island.  ConstantGamma is the standard inverter CEA
  // mode; ConstantAlpha is the analogous rectifier characteristic mode.
  return conv.control_mode == LCCControlMode::ConstantGamma ||
         conv.control_mode == LCCControlMode::ConstantAlpha;
}

LCCJacobian lcc_ac_dc_jacobian(const SolverData& data,
                               const LCCConverter& conv,
                               const Eigen::VectorXd& vm,
                               const Eigen::VectorXd& vdc) {
  LCCJacobian jac;
  LCCOperatingPoint op;
  if (!lcc_eval_at_state(data, conv, vm, vdc, op)) return jac;

  const int ac_pos = conv.ac_bus - 1;
  const int commutation_ac_pos = lcc_commutation_ac_bus(data, conv);
  if (commutation_ac_pos < 0 ||
      commutation_ac_pos >= static_cast<int>(vm.size())) {
    return jac;
  }
  const double vm_terminal = std::max(vm[ac_pos], 1e-4);
  const double nb = static_cast<double>(std::max(1, conv.n_bridges));
  const double terminal_e_kv = vm_terminal * conv.vn_ac_kv;
  const double commutation_e_kv =
      lcc_commutation_voltage_kv(data, conv, vm);
  const double ud0_terminal = kUd0Factor * nb * terminal_e_kv;
  const double ud0_commutation = kUd0Factor * nb * commutation_e_kv;
  if (ud0_terminal <= 1e-9 || ud0_commutation <= 1e-9) return jac;
  const double dud0_terminal_dvm =
      kUd0Factor * nb * conv.vn_ac_kv;
  const double dud0_commutation_dvm =
      kUd0Factor * nb *
      lcc_commutation_voltage_sensitivity_kv_per_pu(data, conv);
  const bool shared_ac_voltage = commutation_ac_pos == ac_pos;
  const double base_kv = lcc_dc_base_kv(data, conv);
  const double ud = op.ud_kv;
  const double rc = kThreeOverPi * nb * std::max(conv.x_comm_ohm, kMinXCommOhm);
  const double vd = nb * conv.v_drop_v * 1e-3;
  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  const StationParams station =
      station_params(conv, terminal_e_kv, commutation_e_kv);
  // pac = sign * p_term (p_term >= 0 is the DC-terminal power magnitude);
  // pdc = -sign * p_term; qac = -p_term * tan(phi).
  const double sign = rectifier ? -1.0 : 1.0;

  // ∂p_term/∂(U_d, U_d0) [MW per kV] by control mode.
  double dpterm_dud = 0.0;
  double dpterm_dud0_commutation = 0.0;
  if (current_limit_strictly_binds(conv, station, op.ud_kv)) {
    dpterm_dud = op.id_ka;  // p_term = U_d * I_lim (constant-current region)
  } else {
    switch (conv.control_mode) {
      case LCCControlMode::ConstantPower:
        break;  // p_term = P_set (constant)
      case LCCControlMode::ConstantCurrent:
        dpterm_dud = op.id_ka;
        break;
      case LCCControlMode::ConstantAlpha: {
        const double k =
            ud0_commutation * std::cos(deg_to_rad(conv.alpha_set_deg)) - vd;
        dpterm_dud = (k - 2.0 * ud) / rc;
        dpterm_dud0_commutation =
            std::cos(deg_to_rad(conv.alpha_set_deg)) * ud / rc;
        break;
      }
      case LCCControlMode::ConstantGamma: {
        const double k =
            ud0_commutation * std::cos(deg_to_rad(conv.gamma_set_deg)) + vd;
        dpterm_dud = (k - 2.0 * ud) / rc;
        dpterm_dud0_commutation =
            std::cos(deg_to_rad(conv.gamma_set_deg)) * ud / rc;
        break;
      }
    }
  }
  const double dpterm_dvm_valve =
      shared_ac_voltage
          ? dpterm_dud0_commutation * dud0_commutation_dvm
          : 0.0;
  const double dpterm_dvm_comm =
      shared_ac_voltage
          ? 0.0
          : dpterm_dud0_commutation * dud0_commutation_dvm;
  const double dud_dvdc_scale = op.ud_at_floor ? 0.0 : 1.0;
  jac.dpac_dvdc =
      sign * dpterm_dud * base_kv / data.base_mva * dud_dvdc_scale;
  jac.dpac_dvm = sign * dpterm_dvm_valve / data.base_mva;
  jac.dpac_dvm_comm = sign * dpterm_dvm_comm / data.base_mva;
  jac.dpdc_dvdc =
      -sign * dpterm_dud * base_kv / data.base_mva * dud_dvdc_scale;
  jac.dpdc_dvm = -sign * dpterm_dvm_valve / data.base_mva;
  jac.dpdc_dvm_comm = -sign * dpterm_dvm_comm / data.base_mva;

  // qac = -p_term * S/U_d with S = sqrt(U_d0^2 - U_d^2).  Near the singular
  // point U_d = U_d0 the tan(phi) model itself breaks down; freeze the Q
  // derivatives there (Q is already ~0).
  const double s2 = ud0_terminal * ud0_terminal - ud * ud;
  if (s2 > 1e-6) {
    const double s = std::sqrt(s2);
    const double p_term = ud * op.id_ka;
    const double dq_dud =
        -s * (dpterm_dud * ud - p_term) / (ud * ud) + p_term / s;
    const double dq_dvm_valve =
        -dpterm_dvm_valve * s / ud -
        (p_term / ud) * (ud0_terminal / s) * dud0_terminal_dvm;
    const double dq_dvm_comm = -dpterm_dvm_comm * s / ud;
    jac.dqac_dvdc =
        dq_dud * base_kv / data.base_mva * dud_dvdc_scale;
    jac.dqac_dvm = dq_dvm_valve / data.base_mva;
    jac.dqac_dvm_comm = dq_dvm_comm / data.base_mva;
  }
  return jac;
}

bool lcc_control_supported(const LCCConverter& conv) {
  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  switch (conv.control_mode) {
    case LCCControlMode::ConstantPower:
    case LCCControlMode::ConstantCurrent:
      return true;  // valid for both roles
    case LCCControlMode::ConstantAlpha:
      return rectifier;
    case LCCControlMode::ConstantGamma:
      return !rectifier;
  }
  return false;
}

}  // namespace hacdcpf::powerflow
