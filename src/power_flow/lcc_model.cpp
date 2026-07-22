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

double deg_to_rad(double deg) { return deg * kPi / 180.0; }
double rad_to_deg(double rad) { return rad * 180.0 / kPi; }

struct StationParams {
  double ud0_kv{0.0};   // ideal no-load DC voltage
  double rc_ohm{0.0};   // (3/pi)*n_b*X_c equivalent commutation resistance
  double vdrop_kv{0.0}; // n_b * valve forward drop
  double id_max_ka{0.0};// rated DC current (0 = unlimited)
};

StationParams station_params(const LCCConverter& conv, double e_kv) {
  StationParams p;
  const double nb = static_cast<double>(std::max(1, conv.n_bridges));
  p.ud0_kv = kUd0Factor * nb * std::max(e_kv, 0.0);
  p.rc_ohm = kThreeOverPi * nb * std::max(conv.x_comm_ohm, kMinXCommOhm);
  p.vdrop_kv = nb * conv.v_drop_v * 1e-3;
  p.id_max_ka = (conv.rated_current_a > 0.0) ? conv.rated_current_a * 1e-3 : 0.0;
  return p;
}

// Clamp the DC current to the physical unidirectional range [0, id_max].
// LCC current cannot reverse (thyristor blocking); the rated-current clamp is
// mostly a transient-iteration guard (a constant-power rectifier at low U_d
// would otherwise demand unbounded current).
double clamp_id(double id_ka, double id_max_ka, bool& at_limit) {
  at_limit = false;
  if (id_ka < 0.0) {
    at_limit = true;
    return 0.0;
  }
  if (id_max_ka > 0.0 && id_ka > id_max_ka) {
    at_limit = true;
    return id_max_ka;
  }
  return id_ka;
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
// alpha = 0 — the physical response would be tap-changer action, which this
/// model deliberately does not simulate.
double back_calc_alpha(double ud_kv, double id_ka, const StationParams& p,
                       bool& beyond) {
  if (p.ud0_kv <= 1e-9) {
    beyond = true;
    return 0.0;
  }
  const double arg = (ud_kv + p.rc_ohm * id_ka + p.vdrop_kv) / p.ud0_kv;
  beyond = arg < -1.0 || arg > 1.0;
  return rad_to_deg(std::acos(std::clamp(arg, -1.0, 1.0)));
}

// Back-calculated extinction angle from the inverter external characteristic
//   U_d = U_d0*cos(gamma) - R_c*I_d + n_b*dU_v.
double back_calc_gamma(double ud_kv, double id_ka, const StationParams& p,
                       bool& beyond) {
  if (p.ud0_kv <= 1e-9) {
    beyond = true;
    return 0.0;
  }
  const double arg = (ud_kv + p.rc_ohm * id_ka - p.vdrop_kv) / p.ud0_kv;
  beyond = arg < -1.0 || arg > 1.0;
  return rad_to_deg(std::acos(std::clamp(arg, -1.0, 1.0)));
}

}  // namespace

LCCOperatingPoint lcc_operating_point(const LCCConverter& conv,
                                      double e_kv,
                                      double ud_kv) {
  LCCOperatingPoint op;
  if (!std::isfinite(e_kv) || !std::isfinite(ud_kv)) return op;
  ud_kv = std::max(ud_kv, kMinUdKv);

  const StationParams p = station_params(conv, e_kv);
  op.ud0_kv = p.ud0_kv;
  op.ud_kv = ud_kv;
  if (p.ud0_kv <= 1e-9) return op;  // no valve-side voltage base — unusable

  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  double id = 0.0;
  switch (conv.control_mode) {
    case LCCControlMode::ConstantPower: {
      // Scheduled DC power at this station's DC terminal: P = U_d * I_d.
      id = (conv.p_set_mw > 0.0) ? conv.p_set_mw / ud_kv : 0.0;
      id = clamp_id(id, p.id_max_ka, op.id_at_limit);
      break;
    }
    case LCCControlMode::ConstantCurrent: {
      id = clamp_id(conv.i_set_ka, p.id_max_ka, op.id_at_limit);
      break;
    }
    case LCCControlMode::ConstantAlpha: {
      if (!rectifier) return op;  // unsupported combination
      const double k = p.ud0_kv * std::cos(deg_to_rad(conv.alpha_set_deg)) - p.vdrop_kv;
      id = clamp_id((k - ud_kv) / p.rc_ohm, p.id_max_ka, op.id_at_limit);
      break;
    }
    case LCCControlMode::ConstantGamma: {
      if (rectifier) return op;  // unsupported combination
      const double k = p.ud0_kv * std::cos(deg_to_rad(conv.gamma_set_deg)) + p.vdrop_kv;
      id = clamp_id((k - ud_kv) / p.rc_ohm, p.id_max_ka, op.id_at_limit);
      break;
    }
  }
  op.id_ka = id;

  // Power at the DC terminal (magnitude); the rectifier injects into the DC
  // network, the inverter draws from it.
  const double p_term_mw = ud_kv * id;
  op.p_dc_mw = rectifier ? p_term_mw : -p_term_mw;
  op.p_ac_mw = rectifier ? -p_term_mw : p_term_mw;
  op.q_ac_mvar = -lcc_q_mvar(p_term_mw, ud_kv, p.ud0_kv);

  bool beyond = false;
  op.alpha_deg = back_calc_alpha(ud_kv, id, p, beyond);
  op.alpha_beyond_range = beyond;
  op.gamma_deg = back_calc_gamma(ud_kv, id, p, beyond);
  op.alpha_beyond_range = op.alpha_beyond_range || beyond;
  op.valid = true;
  return op;
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
  const double e_kv = std::max(vm[ac_pos], 0.0) * conv.vn_ac_kv;
  const double ud_kv = vdc[dc_pos] * lcc_dc_base_kv(data, conv);
  op = lcc_operating_point(conv, e_kv, ud_kv);
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
  return op.p_dc_mw / data.base_mva;
}

double lcc_dc_jacobian_vdc(const SolverData& data,
                           const LCCConverter& conv,
                           const Eigen::VectorXd& vm,
                           const Eigen::VectorXd& vdc) {
  LCCOperatingPoint op;
  if (!lcc_eval_at_state(data, conv, vm, vdc, op)) return 0.0;

  const double base_kv = lcc_dc_base_kv(data, conv);
  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  if (op.id_at_limit) {
    // Current clamped at the rated/blocking limit: the injection degenerates
    // to constant current (pdc = ±U_d * I_lim), whose slope is ±I_lim.  A
    // blocked station (I_d = 0) has zero slope.
    return (rectifier ? op.id_ka : -op.id_ka) * base_kv / data.base_mva;
  }
  const StationParams p = station_params(conv, std::max(vm[conv.ac_bus - 1], 0.0) * conv.vn_ac_kv);
  double dpdc_dud = 0.0;  // MW per kV
  switch (conv.control_mode) {
    case LCCControlMode::ConstantGamma: {
      // pdc = -U_d*(K - U_d)/R_c with K = U_d0*cos(gamma) + n_b*dU_v.
      const double k = p.ud0_kv * std::cos(deg_to_rad(conv.gamma_set_deg)) + p.vdrop_kv;
      dpdc_dud = (2.0 * op.ud_kv - k) / p.rc_ohm;
      break;
    }
    case LCCControlMode::ConstantAlpha: {
      // pdc = +U_d*(K' - U_d)/R_c with K' = U_d0*cos(alpha) - n_b*dU_v.
      const double k = p.ud0_kv * std::cos(deg_to_rad(conv.alpha_set_deg)) - p.vdrop_kv;
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
  const double vm_a = std::max(vm[ac_pos], 1e-4);
  const double nb = static_cast<double>(std::max(1, conv.n_bridges));
  const double ud0 = kUd0Factor * nb * vm_a * conv.vn_ac_kv;
  if (ud0 <= 1e-9) return jac;
  const double dud0_dvm = kUd0Factor * nb * conv.vn_ac_kv;  // kV per pu
  const double base_kv = lcc_dc_base_kv(data, conv);
  const double ud = op.ud_kv;
  const double rc = kThreeOverPi * nb * std::max(conv.x_comm_ohm, kMinXCommOhm);
  const double vd = nb * conv.v_drop_v * 1e-3;
  const bool rectifier = conv.station_role == LCCStationRole::Rectifier;
  // pac = sign * p_term (p_term >= 0 is the DC-terminal power magnitude);
  // pdc = -sign * p_term; qac = -p_term * tan(phi).
  const double sign = rectifier ? -1.0 : 1.0;

  // ∂p_term/∂(U_d, U_d0) [MW per kV] by control mode.
  double dpterm_dud = 0.0;
  double dpterm_dud0 = 0.0;
  if (op.id_at_limit) {
    dpterm_dud = op.id_ka;  // p_term = U_d * I_lim (constant-current region)
  } else {
    switch (conv.control_mode) {
      case LCCControlMode::ConstantPower:
        break;  // p_term = P_set (constant)
      case LCCControlMode::ConstantCurrent:
        dpterm_dud = op.id_ka;
        break;
      case LCCControlMode::ConstantAlpha: {
        const double k = ud0 * std::cos(deg_to_rad(conv.alpha_set_deg)) - vd;
        dpterm_dud = (k - 2.0 * ud) / rc;
        dpterm_dud0 = std::cos(deg_to_rad(conv.alpha_set_deg)) * ud / rc;
        break;
      }
      case LCCControlMode::ConstantGamma: {
        const double k = ud0 * std::cos(deg_to_rad(conv.gamma_set_deg)) + vd;
        dpterm_dud = (k - 2.0 * ud) / rc;
        dpterm_dud0 = std::cos(deg_to_rad(conv.gamma_set_deg)) * ud / rc;
        break;
      }
    }
  }
  jac.dpac_dvdc = sign * dpterm_dud * base_kv / data.base_mva;
  jac.dpac_dvm = sign * dpterm_dud0 * dud0_dvm / data.base_mva;
  jac.dpdc_dvdc = -sign * dpterm_dud * base_kv / data.base_mva;
  jac.dpdc_dvm = -sign * dpterm_dud0 * dud0_dvm / data.base_mva;

  // qac = -p_term * S/U_d with S = sqrt(U_d0^2 - U_d^2).  Near the singular
  // point U_d = U_d0 the tan(phi) model itself breaks down; freeze the Q
  // derivatives there (Q is already ~0).
  const double s2 = ud0 * ud0 - ud * ud;
  if (s2 > 1e-6) {
    const double s = std::sqrt(s2);
    const double p_term = ud * op.id_ka;
    const double dq_dud =
        -s * (dpterm_dud * ud - p_term) / (ud * ud) + p_term / s;
    const double dq_dud0 =
        -dpterm_dud0 * s / ud - (p_term / ud) * (ud0 / s);
    jac.dqac_dvdc = dq_dud * base_kv / data.base_mva;
    jac.dqac_dvm = dq_dud0 * dud0_dvm / data.base_mva;
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
