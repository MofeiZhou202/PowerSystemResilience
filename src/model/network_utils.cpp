#include "hacdcpf/projection/project_to_canonical.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include "hacdcpf/model/unit_conversion.hpp"

namespace hacdcpf {

namespace {

constexpr double kPi = 3.14159265358979323846;

template <typename Container>
int next_index_of(const Container& items) {
  int mx = 0;
  for (const auto& item : items) {
    mx = std::max(mx, item.index);
  }
  return mx + 1;
}

double bus_base_kv_or_default(const ACSystem& ac, int bus_id) {
  for (const auto& b : ac.buses) {
    if (b.index == bus_id) {
      return (b.base_kv > 1e-9) ? b.base_kv : 1.0;
    }
  }
  return 1.0;
}

double clamp_nonnegative(double x) {
  return (x > 0.0) ? x : 0.0;
}

DynamicModelProfile projected_dynamic_profile(DynamicModelProfile profile,
                                              const std::string& source_id,
                                              const std::string& notes) {
  if (profile.empty()) return profile;
  if (profile.source_id.empty()) profile.source_id = source_id;
  if (profile.notes.empty()) profile.notes = notes;
  return profile;
}

DynamicModelProfile energy_router_profile_for(const EnergyRouter& er,
                                              const EnergyRouterPort* port,
                                              const std::string& device_role) {
  if (port != nullptr && !port->dynamic_model.empty()) {
    return projected_dynamic_profile(
        port->dynamic_model,
        "EnergyRouterPort:" + std::to_string(er.index) + ":" +
            std::to_string(port->index),
        "Projected from energy router port into canonical " + device_role);
  }
  return projected_dynamic_profile(
      er.dynamic_model,
      "EnergyRouter:" + std::to_string(er.index),
      "Projected from energy router into canonical " + device_role);
}

std::pair<double, double> rx_from_vk_vkr(double vk_percent,
                                         double vkr_percent,
                                         double base_mva,
                                         double sn_mva) {
  if (sn_mva <= 1e-9 || base_mva <= 1e-9) {
    return {0.0, 0.0};
  }
  const double scale = base_mva / sn_mva;
  const double z = std::max(0.0, vk_percent / 100.0) * scale;
  const double r = std::max(0.0, vkr_percent / 100.0) * scale;
  const double x2 = std::max(0.0, z * z - r * r);
  return {r, std::sqrt(x2)};
}

std::pair<double, double> rx_from_z_percent_x_over_r(double z_percent,
                                                     double x_over_r,
                                                     double base_mva,
                                                     double sn_mva) {
  if (z_percent <= 0.0 || sn_mva <= 1e-9 || base_mva <= 1e-9) {
    return {0.0, 0.0};
  }
  const double z = (z_percent / 100.0) * (base_mva / sn_mva);
  if (x_over_r > 1e-9) {
    const double r = z / std::sqrt(1.0 + x_over_r * x_over_r);
    return {r, r * x_over_r};
  }
  return {0.0, z};
}

double tap_from_step(int tap_pos, int tap_neutral, double tap_step_percent) {
  if (std::abs(tap_step_percent) < 1e-12) {
    return 1.0;
  }
  return std::max(1e-6, 1.0 + (static_cast<double>(tap_pos - tap_neutral) * tap_step_percent / 100.0));
}

void add_equivalent_load_from_flexible(const FlexibleLoad& fl, ACSystem& ac, int& next_idx) {
  if (!fl.in_service) return;
  if (fl.bus == 0) return;

  Load ld;
  ld.index = next_idx++;
  ld.bus = fl.bus;
  ld.in_service = true;
  ld.name = fl.name.empty() ? ("FlexibleLoad_" + std::to_string(fl.index)) : fl.name;
  ld.p_mw = fl.p_mw;
  ld.q_mvar = fl.q_mvar;
  ld.model = LoadModel::ConstantPower;
  ld.p_percent_p = 100.0;
  ld.p_percent_q = 100.0;
  ld.controllable = fl.controllable;
  ld.p_min_mw = std::max(0.0, fl.p_mw - clamp_nonnegative(fl.flex_down_mw));
  ld.cost_mw = 0.0;
  ld.priority = fl.priority;
  ac.loads.push_back(ld);
}

void add_equivalent_load_from_asymmetric(const AsymmetricLoad& al, ACSystem& ac, int& next_idx) {
  if (!al.in_service) return;
  if (al.bus == 0) return;

  Load ld;
  ld.index = next_idx++;
  ld.bus = al.bus;
  ld.in_service = true;
  ld.name = al.name.empty() ? ("AsymmetricLoad_" + std::to_string(al.index)) : al.name;
  const double s = (al.scaling > 0.0) ? al.scaling : 1.0;
  ld.p_mw = s * (al.pa_mw + al.pb_mw + al.pc_mw);
  ld.q_mvar = s * (al.qa_mvar + al.qb_mvar + al.qc_mvar);
  ld.model = LoadModel::ConstantPower;
  ld.z_percent_p = al.const_z_percent;
  ld.i_percent_p = al.const_i_percent;
  ld.p_percent_p = al.const_p_percent;
  ld.z_percent_q = al.const_z_percent;
  ld.i_percent_q = al.const_i_percent;
  ld.p_percent_q = al.const_p_percent;
  ld.controllable = al.controllable;
  ld.priority = al.priority;
  ld.dynamic_model =
      projected_dynamic_profile(al.dynamic_model,
                                "AsymmetricLoad:" + std::to_string(al.index),
                                "Projected from asymmetric load into canonical load");
  ac.loads.push_back(ld);
}

void add_equivalent_branch_from_transformer2w(const Transformer2W& tr,
                                              ACSystem& ac,
                                              double base_mva,
                                              int& next_idx) {
  if (!tr.in_service) return;
  if (tr.hv_bus == 0 || tr.lv_bus == 0) return;

  auto [r_pu, x_pu] = rx_from_vk_vkr(tr.vk_percent, tr.vkr_percent, base_mva, tr.sn_mva);
  if (r_pu == 0.0 && x_pu == 0.0) {
    x_pu = 1e-4;
  }

  // Canonical ACBranch stores off-nominal tap on the from side. For an LV-side
  // physical tap, invert the tap and scale leakage impedance by tap^2.
  const double raw_tap =
      tap_from_step(tr.tap_pos, tr.tap_neutral, tr.tap_step_percent);
  const double clamped_tap = std::max(1e-6, raw_tap);
  const double tap_position =
      (tr.tap_side == 1) ? (1.0 / clamped_tap) : clamped_tap;
  const double impedance_scale =
      (tr.tap_side == 1) ? (clamped_tap * clamped_tap) : 1.0;

  // Include the nameplate-to-bus-base ratio when voltage bases do not match.
  const double base_kv_hv = bus_base_kv_or_default(ac, tr.hv_bus);
  const double base_kv_lv = bus_base_kv_or_default(ac, tr.lv_bus);
  double nominal_ratio_pu = 1.0;
  if (tr.vn_hv_kv > 1e-9 && tr.vn_lv_kv > 1e-9 &&
      base_kv_hv > 1e-9 && base_kv_lv > 1e-9) {
    nominal_ratio_pu =
        (tr.vn_hv_kv / tr.vn_lv_kv) / (base_kv_hv / base_kv_lv);
  }
  const double canonical_tap =
      std::max(1e-6, nominal_ratio_pu * tap_position);

  ACBranch br;
  br.index = next_idx++;
  br.from_bus = tr.hv_bus;
  br.to_bus = tr.lv_bus;
  br.r_pu = r_pu * impedance_scale;
  br.x_pu = x_pu * impedance_scale;
  br.b_pu = 0.0;
  br.tap = canonical_tap;
  br.shift_deg = tr.shift_deg;
  br.rate_a_mva = tr.sn_mva;
  br.in_service = true;
  br.name = tr.name.empty() ? ("Transformer2W_" + std::to_string(tr.index)) : tr.name;
  if (tr.z0_percent > 0.0) {
    auto [r0_pu, x0_pu] =
        rx_from_z_percent_x_over_r(tr.z0_percent, tr.x0_r0, base_mva, tr.sn_mva);
    br.r0_pu = r0_pu * impedance_scale;
    br.x0_pu = x0_pu * impedance_scale;
  } else {
    br.r0_pu = r_pu * impedance_scale;
    br.x0_pu = x_pu * impedance_scale;
  }
  br.b0_pu = 0.0;
  br.vn_hv_kv = tr.vn_hv_kv;
  br.vn_lv_kv = tr.vn_lv_kv;
  br.sn_mva = tr.sn_mva;
  ac.branches.push_back(br);
}

void add_equivalent_branches_from_transformer3w(const Transformer3W& tr,
                                                ACSystem& ac,
                                                double base_mva,
                                                int& next_idx) {
  if (!tr.in_service) return;
  if (tr.hv_bus == 0 || tr.mv_bus == 0 || tr.lv_bus == 0) return;

  using Complex = std::complex<double>;

  const double winding_tap_pu =
      std::max(1e-6, tap_from_step(tr.tap_pos, 0, tr.tap_step_percent));
  const double t = winding_tap_pu;
  const double inv_t = 1.0 / t;

  double tap_by_pair[3];
  double tau_winding[3] = {1.0, 1.0, 1.0};
  if (tr.tap_side == 0) {
    tap_by_pair[0] = t;
    tap_by_pair[1] = t;
    tap_by_pair[2] = 1.0;
    tau_winding[0] = t;
  } else if (tr.tap_side == 1) {
    tap_by_pair[0] = inv_t;
    tap_by_pair[1] = 1.0;
    tap_by_pair[2] = t;
    tau_winding[1] = t;
  } else {
    tap_by_pair[0] = 1.0;
    tap_by_pair[1] = inv_t;
    tap_by_pair[2] = inv_t;
    tau_winding[2] = t;
  }

  const double sn_hv_mv = std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_mv_mva));
  const double sn_hv_lv = std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_lv_mva));
  const double sn_mv_lv = std::max(1e-9, std::min(tr.sn_mv_mva, tr.sn_lv_mva));

  auto [r_hm, x_hm] =
      rx_from_vk_vkr(tr.vk_hv_mv_percent, tr.vkr_hv_mv_percent, base_mva, sn_hv_mv);
  auto [r_hl, x_hl] =
      rx_from_vk_vkr(tr.vk_hv_lv_percent, tr.vkr_hv_lv_percent, base_mva, sn_hv_lv);
  auto [r_ml, x_ml] =
      rx_from_vk_vkr(tr.vk_mv_lv_percent, tr.vkr_mv_lv_percent, base_mva, sn_mv_lv);

  const Complex z_hm(r_hm, x_hm);
  const Complex z_hl(r_hl, x_hl);
  const Complex z_ml(r_ml, x_ml);

  const Complex z_h = 0.5 * (z_hm + z_hl - z_ml);
  const Complex z_m = 0.5 * (z_hm + z_ml - z_hl);
  const Complex z_l = 0.5 * (z_hl + z_ml - z_hm);

  const double z_min = 1e-12;
  const Complex y_h =
      (std::abs(z_h) > z_min) ? (Complex{1.0, 0.0} / z_h)
                              : Complex{1.0 / z_min, 0.0};
  const Complex y_m =
      (std::abs(z_m) > z_min) ? (Complex{1.0, 0.0} / z_m)
                              : Complex{1.0 / z_min, 0.0};
  const Complex y_l =
      (std::abs(z_l) > z_min) ? (Complex{1.0, 0.0} / z_l)
                              : Complex{1.0 / z_min, 0.0};

  const Complex y_ss = y_h + y_m + y_l;
  const Complex y_winding[3] = {y_h, y_m, y_l};
  const int pair_from_winding[3] = {0, 0, 1};
  const int pair_to_winding[3] = {1, 2, 2};

  struct PairData {
    int from_bus;
    int to_bus;
    double sn_from;
    double sn_to;
    const char* suffix;
  };

  const PairData pairs[3] = {
      {tr.hv_bus, tr.mv_bus, tr.sn_hv_mva, tr.sn_mv_mva, "HV_MV"},
      {tr.hv_bus, tr.lv_bus, tr.sn_hv_mva, tr.sn_lv_mva, "HV_LV"},
      {tr.mv_bus, tr.lv_bus, tr.sn_mv_mva, tr.sn_lv_mva, "MV_LV"},
  };

  for (int pair_idx = 0; pair_idx < 3; ++pair_idx) {
    const auto& p = pairs[pair_idx];
    const int fw = pair_from_winding[pair_idx];
    const int tw = pair_to_winding[pair_idx];
    const double tau_from = tau_winding[fw];
    const double tau_to = tau_winding[tw];
    const double tau_pair = tap_by_pair[pair_idx];

    const Complex y_numerator =
        y_winding[fw] * y_winding[tw] * Complex{tau_pair, 0.0};
    const Complex z_denominator = Complex{tau_from * tau_to, 0.0} * y_ss;
    if (std::abs(y_numerator) < 1e-30) continue;

    const Complex z_branch = z_denominator / y_numerator;
    const double r_pu = z_branch.real();
    const double x_pu = z_branch.imag();
    if (r_pu == 0.0 && x_pu == 0.0) continue;

    const double sn_pair = std::max(1e-9, std::min(p.sn_from, p.sn_to));

    ACBranch br;
    br.index = next_idx++;
    br.from_bus = p.from_bus;
    br.to_bus = p.to_bus;
    br.r_pu = r_pu;
    br.x_pu = x_pu;
    br.b_pu = 0.0;
    br.tap = tap_by_pair[pair_idx];
    br.shift_deg =
        (p.from_bus == tr.hv_bus && p.to_bus == tr.mv_bus)
            ? tr.shift_mv_deg
            : (p.from_bus == tr.hv_bus && p.to_bus == tr.lv_bus)
                  ? tr.shift_lv_deg
                  : (tr.shift_lv_deg - tr.shift_mv_deg);
    br.rate_a_mva = sn_pair;
    br.in_service = true;
    br.name = tr.name.empty() ? ("Transformer3W_" + std::to_string(tr.index) + "_" + p.suffix)
                              : (tr.name + "_" + p.suffix + "_eq");
    br.r0_pu = r_pu;
    br.x0_pu = x_pu;
    br.b0_pu = 0.0;
    br.vn_hv_kv = tr.vn_hv_kv;
    br.vn_lv_kv = (p.to_bus == tr.mv_bus) ? tr.vn_mv_kv : tr.vn_lv_kv;
    br.sn_mva = sn_pair;
    ac.branches.push_back(br);
  }
}

void add_equivalent_branch_from_switch(const Switch& sw,
                                       ACSystem& ac,
                                       double base_mva,
                                       int& next_idx) {
  if (!sw.in_service) return;
  if (sw.bus_from == 0 || sw.bus_to == 0) return;

  ACBranch br;
  br.index = next_idx++;
  br.from_bus = sw.bus_from;
  br.to_bus = sw.bus_to;
  br.in_service = sw.closed;
  br.name = sw.name.empty() ? ("Switch_" + std::to_string(sw.index)) : sw.name;

  const double kv = bus_base_kv_or_default(ac, sw.bus_from);
  const double z_base = (kv * kv) / std::max(1e-9, base_mva);
  const double r_ohm = std::max(0.0, sw.r_contact_ohm);
  const double z_ohm = std::max(r_ohm, sw.z_ohm);
  double x_ohm = 0.0;
  if (z_ohm > r_ohm) {
    x_ohm = std::sqrt(std::max(0.0, z_ohm * z_ohm - r_ohm * r_ohm));
  }
  br.r_pu = r_ohm / z_base;
  br.x_pu = x_ohm / z_base;
  if (br.r_pu == 0.0 && br.x_pu == 0.0 && br.in_service) {
    br.r_pu = 1e-6;
    br.x_pu = 1e-6;
  }
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.shift_deg = 0.0;
  br.rate_a_mva = (sw.i_rated_ka > 0.0) ? (std::sqrt(3.0) * kv * sw.i_rated_ka) : 0.0;
  ac.branches.push_back(br);
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// CircuitBreaker 鈫?canonical ACBranch
//
// A circuit breaker is modelled identically to a Switch: an
// equivalent branch is always created, with in_service = cb.closed.
// A closed breaker becomes a near-zero-impedance branch that the
// downstream merge_zero_impedance_buses() step contracts into a
// single node.  An open breaker becomes an out-of-service branch,
// which electrically disconnects the two buses 鈥?the island detector
// and adaptive solver then handle the resulting dead island.
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void add_equivalent_branch_from_circuit_breaker(const CircuitBreaker& cb,
                                                 ACSystem& ac,
                                                 double base_mva,
                                                 int& next_idx) {
  if (!cb.in_service) return;
  if (cb.bus_from == 0 || cb.bus_to == 0) return;

  ACBranch br;
  br.index = next_idx++;
  br.from_bus = cb.bus_from;
  br.to_bus = cb.bus_to;
  br.in_service = cb.closed;   // open 鈫?out-of-service branch (same as Switch)
  br.name = cb.name.empty() ? ("CB_" + std::to_string(cb.index)) : cb.name;

  const double kv = bus_base_kv_or_default(ac, cb.bus_from);
  const double z_base = (kv * kv) / std::max(1e-9, base_mva);
  const double z_ohm = std::max(0.0, cb.z_ohm);
  // For a circuit breaker, z_ohm is typically pure resistance
  br.r_pu = z_ohm / z_base;
  br.x_pu = 0.0;
  // If both are zero (ideal CB) and the breaker is closed, assign tiny
  // impedance so the merge heuristic can identify this as a zero-Z link.
  if (br.r_pu < 1e-8 && br.x_pu < 1e-8 && br.in_service) {
    br.r_pu = 1e-6;
    br.x_pu = 1e-6;
  }
  br.b_pu = 0.0;
  br.tap = 1.0;
  br.shift_deg = 0.0;
  br.rate_a_mva = (cb.i_rated_ka > 0.0) ? (std::sqrt(3.0) * kv * cb.i_rated_ka) : 0.0;
  ac.branches.push_back(br);
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// Motor 鈫?canonical Load
//
// An AsynchronousMotor is a constant-power consumer.  Its rated power
// draw is:  P = sn_mva 脳 cos_phi,  Q = sn_mva 脳 sin(蠁).
// The SC subtransient impedance fields (sn_mva, r_sc_pu, x_sub_pu with
// motor_percent = 1) allow the Load-based SC path to model this motor.
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void add_equivalent_load_from_motor(const AsynchronousMotor& m,
                                    ACSystem& ac,
                                    int& next_idx) {
  if (!m.in_service) return;
  if (m.bus == 0) return;

  Load ld;
  ld.index = next_idx++;
  ld.bus = m.bus;
  ld.in_service = true;
  ld.name = m.name.empty() ? ("Motor_" + std::to_string(m.index)) : m.name;

  const double cos_phi = std::clamp(m.cos_phi, 0.01, 1.0);
  const double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
  ld.p_mw   = m.sn_mva * cos_phi;
  ld.q_mvar = m.sn_mva * sin_phi;
  ld.model  = LoadModel::ConstantPower;
  ld.p_percent_p = 100.0;
  ld.p_percent_q = 100.0;

  // SC subtransient impedance: convert from ohms to pu on motor base so that
  // the Load motor-fraction SC path (sn_mva, motor_percent, r_sc_pu, x_sub_pu)
  // produces the same admittance as the dedicated Motor path.
  ld.sn_mva       = m.sn_mva;
  ld.motor_percent = 1.0;
  ld.sc_source_type = "AsynchronousMotor";
  ld.sc_source_index = m.index;
  ld.motor_poles = m.poles;
  ld.motor_efficiency = m.efficiency;
  ld.dynamic_model =
      projected_dynamic_profile(m.dynamic_model,
                                "AsynchronousMotor:" + std::to_string(m.index),
                                "Projected from asynchronous motor into canonical load");
  if (m.vn_kv > 1e-6 && m.sn_mva > 1e-6) {
    const double z_base_motor = m.vn_kv * m.vn_kv / m.sn_mva;
    ld.r_sc_pu  = m.r_pu / z_base_motor;
    ld.x_sub_pu = m.x_pu / z_base_motor;
  } else {
    ld.r_sc_pu  = m.r_pu;
    ld.x_sub_pu = m.x_pu;
  }

  ac.loads.push_back(ld);
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// VirtualPowerPlant 鈫?canonical StaticGenerator
//
// The VPP pcc_bus receives the net injection from the fleet.
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void add_equivalent_static_gen_from_vpp(const VirtualPowerPlant& vpp,
                                        ACSystem& ac,
                                        int& next_idx) {
  if (!vpp.in_service) return;
  if (vpp.pcc_bus == 0) return;

  StaticGenerator sg;
  sg.index      = next_idx++;
  sg.bus        = vpp.pcc_bus;
  sg.in_service = true;
  sg.name       = vpp.name.empty() ? ("VPP_" + std::to_string(vpp.index)) : vpp.name;
  sg.sgen_type  = SgenType::Other;

  sg.p_mw      = vpp.p_output_mw;
  sg.q_mvar    = vpp.q_output_mvar;
  sg.pmax_mw   = vpp.pmax_mw;
  sg.pmin_mw   = vpp.pmin_mw;
  sg.controllable = true;

  // Regulation capability 鈫?reactive limits (symmetric if not given)
  sg.qmax_mvar = (vpp.q_output_mvar >= 0.0) ? vpp.q_output_mvar : 0.0;
  sg.qmin_mvar = (vpp.q_output_mvar < 0.0)  ? vpp.q_output_mvar : 0.0;
  sg.dynamic_model =
      projected_dynamic_profile(vpp.dynamic_model,
                                "VirtualPowerPlant:" + std::to_string(vpp.index),
                                "Projected from virtual power plant into canonical static generator");

  ac.static_generators.push_back(sg);
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// Microgrid 鈫?canonical StaticGenerator at PCC bus
//
// In grid-connected mode, the microgrid net exchange appears as a
// StaticGenerator at the point of common coupling.  Positive p_mw
// means the microgrid is exporting (injecting into the main grid).
// Islanded microgrids are skipped (they decouple from the main grid).
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void add_equivalent_static_gen_from_microgrid(const Microgrid& mg,
                                              ACSystem& ac,
                                              int& next_idx) {
  if (!mg.in_service) return;
  if (mg.pcc_bus == 0) return;
  if (mg.operating_mode == MicrogridMode::Islanded) return;

  StaticGenerator sg;
  sg.index      = next_idx++;
  sg.bus        = mg.pcc_bus;
  sg.in_service = true;
  sg.name       = mg.name.empty() ? ("Microgrid_" + std::to_string(mg.index)) : mg.name;
  sg.sgen_type  = SgenType::Other;

  // p_exchange_mw > 0 means export from microgrid 鈫?injection into main grid
  sg.p_mw      = mg.p_exchange_mw;
  sg.q_mvar    = 0.0;   // reactive exchange not tracked in Microgrid model
  // Limits: export is positive (pmax) and import negative (pmin)
  sg.pmax_mw   = mg.p_export_max_mw > 1e-9 ? mg.p_export_max_mw : mg.p_exchange_max_mw;
  sg.pmin_mw   = mg.p_import_max_mw > 1e-9 ? -mg.p_import_max_mw : mg.p_exchange_min_mw;
  sg.controllable = true;
  sg.dynamic_model =
      projected_dynamic_profile(mg.dynamic_model,
                                "Microgrid:" + std::to_string(mg.index),
                                "Projected from microgrid into canonical static generator");

  ac.static_generators.push_back(sg);
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// MobileStorage 鈫?canonical AC Storage at current bus
//
// When the unit is stationary and connected (bus != 0), it behaves
// identically to a fixed Storage unit.
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void add_equivalent_storage_from_mobile_storage(const MobileStorage& ms,
                                                ACSystem& ac,
                                                int& next_idx) {
  if (!ms.in_service) return;
  if (ms.bus == 0) return;
  // Only project if at rest 鈥?mobile/transit units are not yet connected.
  if (ms.status == MobileStorageStatus::InTransit) return;

  Storage st;
  st.index      = next_idx++;
  st.bus        = ms.bus;
  st.in_service = true;
  st.name       = ms.name.empty() ? ("MobileStorage_" + std::to_string(ms.index)) : ms.name;

  st.p_mw      = ms.p_mw;
  st.q_mvar    = ms.q_mvar;
  st.p_rated_mw = ms.p_rated_mw;
  st.pmax_mw   = ms.pmax_mw;
  st.pmin_mw   = ms.pmin_mw;
  st.qmax_mvar = ms.qmax_mvar;
  st.qmin_mvar = ms.qmin_mvar;
  st.e_rated_mwh = ms.e_rated_mwh;
  st.soc_init  = ms.soc_init;
  st.soc_min   = ms.soc_min;
  st.soc_max   = ms.soc_max;
  st.eta_charge    = ms.eta_charge;
  st.eta_discharge = ms.eta_discharge;
  st.e_mwh         = ms.e_mwh;
  st.controllable  = ms.controllable;
  st.type          = ms.type;
  st.dynamic_model =
      projected_dynamic_profile(ms.dynamic_model,
                                "MobileStorage:" + std::to_string(ms.index),
                                "Projected from mobile storage into canonical storage");

  ac.storage.push_back(st);
}

void project_chargers_into_stations(ACSystem& ac) {
  if (ac.chargers.empty()) return;

  std::unordered_map<int, size_t> station_pos;
  station_pos.reserve(ac.charging_stations.size());
  for (size_t i = 0; i < ac.charging_stations.size(); ++i) {
    station_pos.emplace(ac.charging_stations[i].index, i);
  }

  for (const auto& ch : ac.chargers) {
    if (!ch.in_service) continue;
    auto it = station_pos.find(ch.station_id);
    if (it == station_pos.end()) continue;
    auto& cs = ac.charging_stations[it->second];
    const double p_kw = std::max(0.0, ch.p_ch_max_kw);
    cs.p_total_kw += p_kw;
    const double pf = std::clamp(cs.power_factor, 1e-3, 1.0);
    const double tanphi = std::tan(std::acos(pf));
    cs.q_total_kvar += p_kw * tanphi;
    cs.num_chargers = std::max(cs.num_chargers, cs.n_fast + cs.n_slow);
    cs.num_chargers += 1;
  }
}

void project_three_phase_if_needed(HybridPowerSystem& out) {
  if (!out.three_phase_ac.has_value()) return;
  if (!out.ac.buses.empty()) return;

  const auto& tp = out.three_phase_ac.value();
  out.ac.base_mva = (tp.base_mva > 0.0) ? tp.base_mva : out.base_mva;
  out.ac.freq_hz = tp.base_freq_hz;

  out.ac.buses.reserve(tp.buses.size());
  for (const auto& b : tp.buses) {
    ACBus acb;
    acb.index = b.index;
    acb.bus_type = b.bus_type;
    acb.pd_mw = b.pd_a_mw + b.pd_b_mw + b.pd_c_mw;
    acb.qd_mvar = b.qd_a_mvar + b.qd_b_mvar + b.qd_c_mvar;
    acb.vm_pu = (b.vm_a_pu + b.vm_b_pu + b.vm_c_pu) / 3.0;
    acb.va_deg = b.va_a_deg;
    acb.area = b.area;
    acb.base_kv = b.base_kv;
    acb.vmax_pu = b.vmax_pu;
    acb.vmin_pu = b.vmin_pu;
    acb.gs_mw = b.gs_a_mw + b.gs_b_mw + b.gs_c_mw;
    acb.bs_mvar = b.bs_a_mvar + b.bs_b_mvar + b.bs_c_mvar;
    acb.zone = b.zone;
    acb.in_service = b.in_service;
    acb.name = b.name;
    out.ac.buses.push_back(acb);
  }

  int next_br = 1;
  for (const auto& line : tp.lines) {
    ACBranch br;
    br.index = next_br++;
    br.from_bus = line.from_bus;
    br.to_bus = line.to_bus;
    br.in_service = line.in_service;
    br.name = line.name.empty() ? ("ThreePhaseLine_" + std::to_string(line.index)) : line.name;
    br.r_pu = line.r1_pu;
    br.x_pu = line.x1_pu;
    br.b_pu = line.b1_pu;
    if ((br.r_pu == 0.0 && br.x_pu == 0.0) && line.length_km > 0.0) {
      const double kv = bus_base_kv_or_default(out.ac, line.from_bus);
      const double z_base = (kv * kv) / std::max(1e-9, out.base_mva);
      const double parallel = std::max(1, line.parallel);
      const double r_ohm = (line.r1_ohm_per_km * line.length_km) / static_cast<double>(parallel);
      const double x_ohm = (line.x1_ohm_per_km * line.length_km) / static_cast<double>(parallel);
      br.r_pu = r_ohm / z_base;
      br.x_pu = x_ohm / z_base;
      const double c_total = line.c1_nf_per_km * line.length_km * static_cast<double>(parallel) * 1e-9;
      const double b_siemens = 2.0 * kPi * std::max(1.0, out.ac.freq_hz) * c_total;
      br.b_pu = b_siemens * z_base;
    }
    br.r0_pu = line.r0_pu;
    br.x0_pu = line.x0_pu;
    br.b0_pu = line.b0_pu;
    br.rate_a_mva = line.rate_a_mva;
    out.ac.branches.push_back(br);
  }

  for (const auto& tr : tp.transformers) {
    Transformer2W eq;
    eq.index = tr.index;
    eq.name = tr.name.empty() ? ("ThreePhaseTransformer_" + std::to_string(tr.index)) : tr.name;
    eq.hv_bus = tr.hv_bus;
    eq.lv_bus = tr.lv_bus;
    eq.in_service = tr.in_service;
    eq.sn_mva = tr.sn_mva;
    eq.vn_hv_kv = tr.vn_hv_kv;
    eq.vn_lv_kv = tr.vn_lv_kv;
    eq.vk_percent = tr.vk_percent;
    eq.vkr_percent = tr.vkr_percent;
    eq.tap_pos = tr.tap_pos;
    eq.tap_neutral = tr.tap_neutral;
    eq.tap_step_percent = tr.tap_step_percent;
    eq.shift_deg = tr.shift_deg;
    eq.z0_percent = tr.vk0_percent;
    if (tr.vkr0_percent > 1e-9) {
      const double x0_sq =
          std::max(0.0, tr.vk0_percent * tr.vk0_percent -
                            tr.vkr0_percent * tr.vkr0_percent);
      eq.x0_r0 = std::sqrt(x0_sq) / tr.vkr0_percent;
    }
    out.ac.transformers_2w.push_back(eq);
  }

  int next_ld = 1;
  for (const auto& ld : tp.loads) {
    Load eq;
    eq.index = next_ld++;
    eq.bus = ld.bus;
    eq.in_service = ld.in_service;
    eq.name = ld.name.empty() ? ("ThreePhaseLoad_" + std::to_string(ld.index)) : ld.name;
    eq.p_mw = ld.p_a_mw + ld.p_b_mw + ld.p_c_mw;
    eq.q_mvar = ld.q_a_mvar + ld.q_b_mvar + ld.q_c_mvar;
    eq.z_percent_p = ld.const_z_percent;
    eq.i_percent_p = ld.const_i_percent;
    eq.p_percent_p = ld.const_p_percent;
    eq.z_percent_q = ld.const_z_percent;
    eq.i_percent_q = ld.const_i_percent;
    eq.p_percent_q = ld.const_p_percent;
    eq.motor_percent = ld.motor_percent;
    eq.x_sub_pu = ld.x_r_ratio;
    eq.dynamic_model =
        projected_dynamic_profile(ld.dynamic_model,
                                  "ThreePhaseLoad:" + std::to_string(ld.index),
                                  "Projected from three-phase load into canonical load");
    out.ac.loads.push_back(eq);
  }

  int next_gen = 1;
  for (const auto& g : tp.generators) {
    Generator eq;
    eq.index = next_gen++;
    eq.bus = g.bus;
    eq.in_service = g.in_service;
    eq.is_slack = g.is_slack;
    eq.name = g.name.empty() ? ("ThreePhaseGen_" + std::to_string(g.index)) : g.name;
    eq.pg_mw = g.p_mw;
    eq.qg_mvar = g.q_mvar;
    eq.vg_pu = g.vm_pu;
    eq.pmax_mw = g.pmax_mw;
    eq.pmin_mw = g.pmin_mw;
    eq.qmax_mvar = g.qmax_mvar;
    eq.qmin_mvar = g.qmin_mvar;
    eq.mbase_mva = g.mbase_mva;
    eq.xd_pu = g.xd_pu;
    eq.xdpp_pu = g.xdpp_pu;
    eq.x0_pu = g.x0_pu;
    eq.r0_pu = g.r0_pu;
    eq.dynamic_model =
        projected_dynamic_profile(g.dynamic_model,
                                  "ThreePhaseGenerator:" + std::to_string(g.index),
                                  "Projected from three-phase generator into canonical generator");
    out.ac.generators.push_back(eq);
  }

  int next_eg = 1;
  for (const auto& eg : tp.external_grids) {
    ExternalGrid eq;
    eq.index = next_eg++;
    eq.bus = eg.bus;
    eq.in_service = eg.in_service;
    eq.name = eg.name.empty() ? ("ThreePhaseExternalGrid_" + std::to_string(eg.index)) : eg.name;
    eq.vm_pu = eg.vm_pu;
    eq.va_deg = eg.va_deg;
    eq.s_sc_max_mva = eg.s_sc_max_mva;
    eq.s_sc_min_mva = eg.s_sc_min_mva;
    eq.rx_max = eg.rx_max;
    eq.rx_min = eg.rx_min;
    eq.r_pu = eg.r1_pu;
    eq.x_pu = eg.x1_pu;
    eq.r0_pu = eg.r0_pu;
    eq.x0_pu = eg.x0_pu;
    eq.dynamic_model =
        projected_dynamic_profile(eg.dynamic_model,
                                  "ThreePhaseExternalGrid:" + std::to_string(eg.index),
                                  "Projected from three-phase external grid into canonical external grid");
    out.ac.external_grids.push_back(eq);
  }
}

}  // namespace

int n_ac_buses(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.buses.size());
}
int n_dc_buses(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.dc.buses.size());
}
int n_buses(const HybridPowerSystem& sys) {
  return n_ac_buses(sys) + n_dc_buses(sys);
}
int n_ac_branches(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.branches.size());
}
int n_dc_branches(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.dc.branches.size());
}
int n_branches(const HybridPowerSystem& sys) {
  return n_ac_branches(sys) + n_dc_branches(sys);
}
int n_generators(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.generators.size());
}
int n_loads(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.loads.size());
}
int n_shunts(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.shunts.size());
}
int n_storage(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.storage.size()) +
         static_cast<int>(sys.dc.storage.size());
}
int n_renewable_gens(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.ac.renewable_gens.size());
}
int n_converters(const HybridPowerSystem& sys) {
  return static_cast<int>(sys.vsc_converters.size());
}

// Aggregation queries
double total_gen_capacity_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& g : sys.ac.generators) {
    if (g.in_service) total += g.pmax_mw;
  }
  return total;
}

double total_gen_pmin_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& g : sys.ac.generators) {
    if (g.in_service) total += g.pmin_mw;
  }
  return total;
}

double total_load_p_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  // Bus-level loads (legacy)
  for (const auto& b : sys.ac.buses) {
    if (b.in_service) total += b.pd_mw;
  }
  // Explicit Load components
  for (const auto& l : sys.ac.loads) {
    if (l.in_service) total += l.p_mw;
  }
  return total;
}

double total_load_q_mvar(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& b : sys.ac.buses) {
    if (b.in_service) total += b.qd_mvar;
  }
  for (const auto& l : sys.ac.loads) {
    if (l.in_service) total += l.q_mvar;
  }
  return total;
}

double total_renewable_capacity_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& r : sys.ac.renewable_gens) {
    if (r.in_service) total += r.p_rated_mw;
  }
  return total;
}

double total_storage_capacity_mwh(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& s : sys.ac.storage) {
    if (s.in_service) total += s.e_rated_mwh;
  }
  for (const auto& s : sys.dc.storage) {
    if (s.in_service) total += s.e_rated_mwh;
  }
  return total;
}

double total_storage_power_mw(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& s : sys.ac.storage) {
    if (s.in_service) total += s.p_rated_mw;
  }
  for (const auto& s : sys.dc.storage) {
    if (s.in_service) total += s.p_rated_mw;
  }
  return total;
}

double total_system_inertia_mws(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& g : sys.ac.generators) {
    if (g.in_service && g.inertia_h > 0.0) {
      double sbase = (g.mbase_mva > 0.0) ? g.mbase_mva : sys.base_mva;
      total += g.inertia_h * sbase;
    }
  }
  return total;
}

double total_emission_tco2_h(const HybridPowerSystem& sys) {
  double total = 0.0;
  for (const auto& g : sys.ac.generators) {
    if (g.in_service && g.pg_mw > 0.0) {
      total += g.emission_factor_tco2_mwh * g.pg_mw;
    }
  }
  return total;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Validation
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
ValidationResult validate(const HybridPowerSystem& sys) {
  ValidationResult res;

  // Collect valid AC bus indices
  std::unordered_set<int> ac_bus_idx;
  std::set<int> ac_dup_check;
  for (const auto& b : sys.ac.buses) {
    if (!ac_dup_check.insert(b.index).second) {
      res.errors.push_back("Duplicate AC bus index: " +
                           std::to_string(b.index));
    }
    ac_bus_idx.insert(b.index);
  }

  // Collect valid DC bus indices
  std::unordered_set<int> dc_bus_idx;
  std::set<int> dc_dup_check;
  for (const auto& b : sys.dc.buses) {
    if (!dc_dup_check.insert(b.index).second) {
      res.errors.push_back("Duplicate DC bus index: " +
                           std::to_string(b.index));
    }
    dc_bus_idx.insert(b.index);
  }

  // Check AC branches
  for (const auto& br : sys.ac.branches) {
    if (ac_bus_idx.find(br.from_bus) == ac_bus_idx.end()) {
      res.errors.push_back("AC branch " + std::to_string(br.index) +
                           " from_bus " + std::to_string(br.from_bus) +
                           " not found");
    }
    if (ac_bus_idx.find(br.to_bus) == ac_bus_idx.end()) {
      res.errors.push_back("AC branch " + std::to_string(br.index) +
                           " to_bus " + std::to_string(br.to_bus) +
                           " not found");
    }
    if (br.x_pu == 0.0 && br.r_pu == 0.0) {
      res.warnings.push_back("AC branch " + std::to_string(br.index) +
                             " has zero impedance");
    }
  }

  // Check DC branches
  for (const auto& br : sys.dc.branches) {
    if (dc_bus_idx.find(br.from_bus) == dc_bus_idx.end()) {
      res.errors.push_back("DC branch " + std::to_string(br.index) +
                           " from_bus " + std::to_string(br.from_bus) +
                           " not found");
    }
    if (dc_bus_idx.find(br.to_bus) == dc_bus_idx.end()) {
      res.errors.push_back("DC branch " + std::to_string(br.index) +
                           " to_bus " + std::to_string(br.to_bus) +
                           " not found");
    }
  }

  // Check generators
  for (const auto& g : sys.ac.generators) {
    if (ac_bus_idx.find(g.bus) == ac_bus_idx.end()) {
      res.errors.push_back("Generator " + std::to_string(g.index) +
                           " bus " + std::to_string(g.bus) + " not found");
    }
    if (g.pmin_mw > g.pmax_mw) {
      res.warnings.push_back("Generator " + std::to_string(g.index) +
                             " pmin > pmax");
    }
    if (g.qmin_mvar > g.qmax_mvar) {
      res.warnings.push_back("Generator " + std::to_string(g.index) +
                             " qmin > qmax");
    }
  }

  // Check loads
  for (const auto& l : sys.ac.loads) {
    if (ac_bus_idx.find(l.bus) == ac_bus_idx.end()) {
      res.errors.push_back("Load " + std::to_string(l.index) + " bus " +
                           std::to_string(l.bus) + " not found");
    }
  }

  // Check storage
  for (const auto& s : sys.ac.storage) {
    if (ac_bus_idx.find(s.bus) == ac_bus_idx.end()) {
      res.errors.push_back("AC Storage " + std::to_string(s.index) +
                           " bus " + std::to_string(s.bus) + " not found");
    }
    if (s.soc_min > s.soc_max) {
      res.warnings.push_back("Storage " + std::to_string(s.index) +
                             " soc_min > soc_max");
    }
  }
  for (const auto& s : sys.dc.storage) {
    if (dc_bus_idx.find(s.bus) == dc_bus_idx.end()) {
      res.errors.push_back("DC Storage " + std::to_string(s.index) +
                           " bus " + std::to_string(s.bus) + " not found");
    }
  }

  // Check renewable generators
  for (const auto& r : sys.ac.renewable_gens) {
    if (ac_bus_idx.find(r.bus) == ac_bus_idx.end()) {
      res.errors.push_back("RenewableGen " + std::to_string(r.index) +
                           " bus " + std::to_string(r.bus) + " not found");
    }
  }

  // Check VSC converters
  for (const auto& c : sys.vsc_converters) {
    if (ac_bus_idx.find(c.bus_ac) == ac_bus_idx.end()) {
      res.errors.push_back("VSC " + std::to_string(c.index) + " bus_ac " +
                           std::to_string(c.bus_ac) + " not found");
    }
    if (dc_bus_idx.find(c.bus_dc) == dc_bus_idx.end()) {
      res.errors.push_back("VSC " + std::to_string(c.index) + " bus_dc " +
                           std::to_string(c.bus_dc) + " not found");
    }
  }

  // Check at least one slack bus
  bool has_slack = false;
  for (const auto& b : sys.ac.buses) {
    if (b.bus_type == BusType::SLACK && b.in_service) {
      has_slack = true;
      break;
    }
  }
  if (!has_slack && !sys.ac.buses.empty()) {
    res.warnings.push_back("No AC slack bus found");
  }

  res.valid = res.errors.empty();
  return res;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Summary printing
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
void print_summary(const HybridPowerSystem& sys, std::ostream& os) {
  os << "鈺斺晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晽\n";
  os << "鈺? Power System: " << sys.name << "\n";
  os << "鈺? Base MVA: " << sys.base_mva << "\n";
  os << "鈺犫晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨暎\n";
  os << "鈺? AC buses:        " << n_ac_buses(sys) << "\n";
  os << "鈺? AC branches:     " << n_ac_branches(sys) << "\n";
  os << "鈺? DC buses:        " << n_dc_buses(sys) << "\n";
  os << "鈺? DC branches:     " << n_dc_branches(sys) << "\n";
  os << "鈺? Generators:      " << n_generators(sys) << "\n";
  os << "鈺? Loads:           " << n_loads(sys) << "\n";
  os << "鈺? Shunts:          " << n_shunts(sys) << "\n";
  os << "鈺? Storage:         " << n_storage(sys) << "\n";
  os << "鈺? Renewables:      " << n_renewable_gens(sys) << "\n";
  os << "鈺? VSC converters:  " << n_converters(sys) << "\n";
  os << "鈺犫晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨暎\n";
  os << "鈺? Total gen capacity:   " << total_gen_capacity_mw(sys)
     << " MW\n";
  os << "鈺? Total load (P):       " << total_load_p_mw(sys) << " MW\n";
  os << "鈺? Total load (Q):       " << total_load_q_mvar(sys)
     << " Mvar\n";

  double re_cap = total_renewable_capacity_mw(sys);
  double st_cap = total_storage_capacity_mwh(sys);
  if (re_cap > 0.0) {
    os << "鈺? Renewable capacity:   " << re_cap << " MW\n";
  }
  if (st_cap > 0.0) {
    os << "鈺? Storage energy:       " << st_cap << " MWh\n";
    os << "鈺? Storage power:        " << total_storage_power_mw(sys)
       << " MW\n";
  }

  double inertia = total_system_inertia_mws(sys);
  if (inertia > 0.0) {
    os << "鈺? System inertia:       " << inertia << " MW路s\n";
  }

  os << "鈺氣晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨暆\n";
}

std::string summary_string(const HybridPowerSystem& sys) {
  std::ostringstream oss;
  print_summary(sys, oss);
  return oss.str();
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Topology helpers
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
std::vector<std::vector<int>> ac_adjacency_list(const HybridPowerSystem& sys) {
  const int nb = n_ac_buses(sys);

  // Map bus index -> position
  std::unordered_map<int, int> idx_to_pos;
  idx_to_pos.reserve(static_cast<size_t>(nb));
  for (int i = 0; i < nb; ++i) {
    idx_to_pos[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }

  std::vector<std::vector<int>> adj(static_cast<size_t>(nb));
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    auto it_f = idx_to_pos.find(br.from_bus);
    auto it_t = idx_to_pos.find(br.to_bus);
    if (it_f != idx_to_pos.end() && it_t != idx_to_pos.end()) {
      adj[static_cast<size_t>(it_f->second)].push_back(it_t->second);
      adj[static_cast<size_t>(it_t->second)].push_back(it_f->second);
    }
  }
  return adj;
}

std::vector<int> generators_per_bus(const HybridPowerSystem& sys) {
  const int nb = n_ac_buses(sys);
  std::unordered_map<int, int> idx_to_pos;
  idx_to_pos.reserve(static_cast<size_t>(nb));
  for (int i = 0; i < nb; ++i) {
    idx_to_pos[sys.ac.buses[static_cast<size_t>(i)].index] = i;
  }

  std::vector<int> count(static_cast<size_t>(nb), 0);
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    auto it = idx_to_pos.find(g.bus);
    if (it != idx_to_pos.end()) {
      count[static_cast<size_t>(it->second)]++;
    }
  }
  return count;
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// Union-Find for bus merging
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
namespace {

class UnionFind {
 public:
  explicit UnionFind(int n) : parent_(n), rank_(n, 0) {
    for (int i = 0; i < n; ++i) parent_[i] = i;
  }
  int find(int x) {
    while (parent_[x] != x) {
      parent_[x] = parent_[parent_[x]];  // path halving
      x = parent_[x];
    }
    return x;
  }
  bool unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b) return false;
    if (rank_[a] < rank_[b]) std::swap(a, b);
    parent_[b] = a;
    if (rank_[a] == rank_[b]) ++rank_[a];
    return true;
  }

 private:
  std::vector<int> parent_;
  std::vector<int> rank_;
};

int bus_type_priority(BusType bt) {
  switch (bt) {
    case BusType::SLACK:    return 3;
    case BusType::PV:       return 2;
    case BusType::PQ:       return 1;
    default:                return 0;  // ISOLATED
  }
}

}  // namespace

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// strip_dead_islands
//
// After canonical projection (switch/CB expansion and bus merging),
// detect connected components via BFS on in-service AC branches.
// Components that have no generation source (no SLACK/PV bus, no
// in-service generator, external grid, static generator, renewable,
// PV system, or storage with non-zero output) are "dead islands".
//
// Dead island buses and all components attached to them are removed
// from the system, and surviving buses are renumbered 1..N_live.
// The BusMergeMap is updated (or created) so that unproject_bus_vector
// maps dead-island positions back to zero in the original bus space.
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
void strip_dead_islands(HybridPowerSystem& sys) {
  auto& buses = sys.ac.buses;
  auto& branches = sys.ac.branches;
  const int n = static_cast<int>(buses.size());
  if (n <= 1) return;

  // Build index鈫抪osition lookup and adjacency list (in-service branches only)
  std::unordered_map<int, int> idx_to_pos;
  idx_to_pos.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    idx_to_pos[buses[static_cast<size_t>(i)].index] = i;
  }

  std::vector<std::vector<int>> adj(static_cast<size_t>(n));
  for (const auto& br : branches) {
    if (!br.in_service) continue;
    auto it_f = idx_to_pos.find(br.from_bus);
    auto it_t = idx_to_pos.find(br.to_bus);
    if (it_f == idx_to_pos.end() || it_t == idx_to_pos.end()) continue;
    adj[static_cast<size_t>(it_f->second)].push_back(it_t->second);
    adj[static_cast<size_t>(it_t->second)].push_back(it_f->second);
  }

  // BFS to identify connected components
  std::vector<int> comp_id(static_cast<size_t>(n), -1);
  std::vector<std::vector<int>> components;  // component 鈫?list of positions
  for (int i = 0; i < n; ++i) {
    if (comp_id[static_cast<size_t>(i)] >= 0) continue;
    int cid = static_cast<int>(components.size());
    components.emplace_back();
    std::queue<int> q;
    q.push(i);
    comp_id[static_cast<size_t>(i)] = cid;
    while (!q.empty()) {
      int cur = q.front(); q.pop();
      components.back().push_back(cur);
      for (int nb : adj[static_cast<size_t>(cur)]) {
        if (comp_id[static_cast<size_t>(nb)] >= 0) continue;
        comp_id[static_cast<size_t>(nb)] = cid;
        q.push(nb);
      }
    }
  }

  if (components.size() <= 1) return;  // single component 鈫?nothing to strip

  // Check each component for generation sources
  // Collect bus indices (1-based .index) into a set per component for fast lookup
  std::vector<std::unordered_set<int>> comp_buses(components.size());
  for (size_t c = 0; c < components.size(); ++c) {
    for (int pos : components[c]) {
      comp_buses[c].insert(buses[static_cast<size_t>(pos)].index);
    }
  }

  auto has_generation = [&](size_t c) -> bool {
    const auto& bset = comp_buses[c];
    // Check bus types
    for (int pos : components[c]) {
      auto bt = buses[static_cast<size_t>(pos)].bus_type;
      if (bt == BusType::SLACK || bt == BusType::PV) return true;
    }
    // Check generators
    for (const auto& g : sys.ac.generators) {
      if (g.in_service && bset.count(g.bus) && (g.pg_mw > 1e-9 || g.is_slack)) return true;
    }
    // Check external grids
    for (const auto& eg : sys.ac.external_grids) {
      if (eg.in_service && bset.count(eg.bus)) return true;
    }
    // Check static generators
    for (const auto& sg : sys.ac.static_generators) {
      if (sg.in_service && bset.count(sg.bus) && sg.p_mw * sg.scaling > 1e-9) return true;
    }
    // Check renewable gens
    for (const auto& rg : sys.ac.renewable_gens) {
      if (rg.in_service && bset.count(rg.bus) && rg.p_mw > 1e-9) return true;
    }
    // Check PV systems
    for (const auto& pv : sys.ac.pv_systems) {
      if (pv.in_service && bset.count(pv.bus) && pv.p_mw > 1e-9) return true;
    }
    // Check storage with active dispatch
    for (const auto& st : sys.ac.storage) {
      if (st.in_service && bset.count(st.bus) && std::abs(st.p_mw) > 1e-9) return true;
    }
    // Check VSC converters (they inject into AC bus)
    for (const auto& conv : sys.vsc_converters) {
      if (conv.in_service && bset.count(conv.bus_ac)) return true;
    }
    return false;
  };

  // Collect dead bus indices (1-based) into a flat set
  std::unordered_set<int> dead_buses;
  for (size_t c = 0; c < components.size(); ++c) {
    if (!has_generation(c)) {
      for (int pos : components[c]) {
        dead_buses.insert(buses[static_cast<size_t>(pos)].index);
      }
    }
  }

  if (dead_buses.empty()) return;  // all islands are solvable

  // 鈹€鈹€ Update BusMergeMap: record dead bus entries 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
  // If no merge map exists yet, create an identity map so that
  // unproject_bus_vector can later expand the result back to full size.
  if (!sys.bus_merge_map) {
    BusMergeMap m;
    m.n_original = n;
    m.n_merged = n;
    m.int_to_ext.resize(static_cast<size_t>(n));
    m.groups.resize(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
      const int ext = buses[static_cast<size_t>(i)].index;
      m.ext_to_int[ext] = i;
      m.ext_to_orig_pos[ext] = i;
      m.extensive_participation[ext] = 1.0;
      m.int_to_ext[static_cast<size_t>(i)] = ext;
      m.groups[static_cast<size_t>(i)] = {ext};
    }
    sys.bus_merge_map = std::move(m);
  }

  auto& merge_map = *sys.bus_merge_map;
  const std::vector<int> old_int_to_ext = merge_map.int_to_ext;
  std::vector<char> dead_positions(static_cast<size_t>(n), 0);
  for (int pos = 0; pos < n; ++pos) {
    if (dead_buses.count(buses[static_cast<size_t>(pos)].index) != 0U) {
      dead_positions[static_cast<size_t>(pos)] = 1;
    }
  }

  // 鈹€鈹€ Remove dead buses 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
  std::vector<ACBus> live_buses;
  live_buses.reserve(buses.size() - dead_buses.size());
  std::vector<int> old_pos_to_new(static_cast<size_t>(n), -1);
  for (int pos = 0; pos < n; ++pos) {
    auto& bus = buses[static_cast<size_t>(pos)];
    if (dead_positions[static_cast<size_t>(pos)] != 0) continue;
    old_pos_to_new[static_cast<size_t>(pos)] =
        static_cast<int>(live_buses.size());
    live_buses.push_back(std::move(bus));
  }

  // Build renumber map: old bus index 鈫?new bus index (1-based contiguous)
  const int n_live = static_cast<int>(live_buses.size());
  std::unordered_map<int, int> renum;
  renum.reserve(static_cast<size_t>(n_live));
  for (int i = 0; i < n_live; ++i) {
    renum[live_buses[static_cast<size_t>(i)].index] = i + 1;
    live_buses[static_cast<size_t>(i)].index = i + 1;
  }
  buses = std::move(live_buses);

  // Compose the original-to-projected mapping with dead-bus removal by
  // projected position. The map keys are original bus IDs, whereas buses[]
  // has already been renumbered by zero-impedance merging.
  std::unordered_set<int> newly_dead_original_buses;
  for (auto& [original_bus, int_pos] : merge_map.ext_to_int) {
    if (int_pos < 0) continue;
    if (int_pos >= n || dead_positions[static_cast<size_t>(int_pos)] != 0) {
      int_pos = -1;
      newly_dead_original_buses.insert(original_bus);
      continue;
    }
    int_pos = old_pos_to_new[static_cast<size_t>(int_pos)];
  }
  merge_map.dead_bus_indices.insert(newly_dead_original_buses.begin(),
                                    newly_dead_original_buses.end());

  merge_map.int_to_ext.assign(static_cast<size_t>(n_live), 0);
  for (int old_pos = 0; old_pos < n; ++old_pos) {
    const int new_pos = old_pos_to_new[static_cast<size_t>(old_pos)];
    if (new_pos < 0 || old_pos >= static_cast<int>(old_int_to_ext.size())) {
      continue;
    }
    merge_map.int_to_ext[static_cast<size_t>(new_pos)] =
        old_int_to_ext[static_cast<size_t>(old_pos)];
  }
  merge_map.groups.assign(static_cast<size_t>(n_live), {});
  for (const auto& [original_bus, int_pos] : merge_map.ext_to_int) {
    if (int_pos >= 0 && int_pos < n_live) {
      merge_map.groups[static_cast<size_t>(int_pos)].push_back(original_bus);
    }
  }
  for (auto& group : merge_map.groups) {
    std::sort(group.begin(), group.end());
  }
  merge_map.n_merged = n_live;

  // 鈹€鈹€ Remap bus references in surviving components, remove dead 鈹€鈹€
  auto remap = [&](int old_bus) -> int {
    auto it = renum.find(old_bus);
    return (it != renum.end()) ? it->second : 0;
  };
  auto is_dead = [&](int bus) -> bool { return dead_buses.count(bus) > 0; };

  // Branches: remap and remove branches connecting to dead buses.
  //
  // Index-space discipline: at this point `branches` is in POST-MERGE
  // position space, while merge_map.branch_orig_to_proj (when the merge
  // stage ran) is keyed by PRE-MERGE (original/expanded) positions.  The
  // strip stage therefore builds its own post-merge→post-strip map and
  // COMPOSES it with any carried map instead of overwriting it, so
  // branch_orig_to_proj always maps original positions to final positions
  // and n_original_branches always counts the original space.
  const int n_branches_pre_strip = static_cast<int>(branches.size());
  std::vector<int> strip_pos_map(static_cast<size_t>(n_branches_pre_strip), -1);
  std::vector<ACBranch> live_br;
  live_br.reserve(branches.size());
  for (int pre_i = 0; pre_i < n_branches_pre_strip; ++pre_i) {
    auto& br = branches[static_cast<size_t>(pre_i)];
    if (is_dead(br.from_bus) || is_dead(br.to_bus)) continue;
    br.from_bus = remap(br.from_bus);
    br.to_bus = remap(br.to_bus);
    if (br.from_bus == 0 || br.to_bus == 0) continue;
    strip_pos_map[static_cast<size_t>(pre_i)] = static_cast<int>(live_br.size());
    live_br.push_back(std::move(br));
  }
  branches = std::move(live_br);

  if (merge_map.n_original_branches > 0 &&
      !merge_map.branch_orig_to_proj.empty()) {
    // Merge stage already recorded original→post-merge; compose with strip.
    for (auto& [orig_pos, proj_pos] : merge_map.branch_orig_to_proj) {
      (void)orig_pos;
      if (proj_pos < 0) continue;
      proj_pos = (proj_pos < n_branches_pre_strip)
                     ? strip_pos_map[static_cast<size_t>(proj_pos)]
                     : -1;
    }
  } else {
    merge_map.n_original_branches = n_branches_pre_strip;
    for (int pre_i = 0; pre_i < n_branches_pre_strip; ++pre_i) {
      merge_map.branch_orig_to_proj[pre_i] =
          strip_pos_map[static_cast<size_t>(pre_i)];
    }
  }

  // Helper: filter + remap for single-bus components
  auto filter_remap = [&](auto& vec) {
    using T = typename std::decay_t<decltype(vec)>::value_type;
    std::vector<T> kept;
    kept.reserve(vec.size());
    for (auto& item : vec) {
      if (is_dead(item.bus)) continue;
      item.bus = remap(item.bus);
      if (item.bus == 0) continue;
      kept.push_back(std::move(item));
    }
    vec = std::move(kept);
  };

  filter_remap(sys.ac.generators);
  filter_remap(sys.ac.loads);
  filter_remap(sys.ac.static_generators);
  filter_remap(sys.ac.renewable_gens);
  filter_remap(sys.ac.pv_systems);
  filter_remap(sys.ac.storage);
  filter_remap(sys.ac.external_grids);
  filter_remap(sys.ac.shunts);
  filter_remap(sys.ac.charging_stations);
  filter_remap(sys.ac.motors);
  filter_remap(sys.ac.flexible_loads);
  filter_remap(sys.ac.asymmetric_loads);

  // Switches and circuit breakers: two-bus components
  auto filter_remap_2bus = [&](auto& vec) {
    using T = typename std::decay_t<decltype(vec)>::value_type;
    std::vector<T> kept;
    kept.reserve(vec.size());
    for (auto& item : vec) {
      if (is_dead(item.bus_from) || is_dead(item.bus_to)) continue;
      item.bus_from = remap(item.bus_from);
      item.bus_to = remap(item.bus_to);
      if (item.bus_from == 0 || item.bus_to == 0) continue;
      kept.push_back(std::move(item));
    }
    vec = std::move(kept);
  };

  filter_remap_2bus(sys.ac.switches);
  filter_remap_2bus(sys.ac.circuit_breakers);

  // Transformers (if any remain after projection)
  {
    std::vector<Transformer2W> kept;
    for (auto& tr : sys.ac.transformers_2w) {
      if (is_dead(tr.hv_bus) || is_dead(tr.lv_bus)) continue;
      tr.hv_bus = remap(tr.hv_bus);
      tr.lv_bus = remap(tr.lv_bus);
      kept.push_back(std::move(tr));
    }
    sys.ac.transformers_2w = std::move(kept);
  }

  // VSC converters: AC side
  {
    std::vector<VSCConverter> kept;
    for (auto& conv : sys.vsc_converters) {
      if (is_dead(conv.bus_ac)) continue;
      conv.bus_ac = remap(conv.bus_ac);
      kept.push_back(std::move(conv));
    }
    sys.vsc_converters = std::move(kept);
  }

  // Energy routers: AC ports
  for (auto& er : sys.energy_routers) {
    for (auto& p : er.ports) {
      if (p.port_type == ERPortType::AC) p.bus = remap(p.bus);
    }
  }
}

// merge_zero_impedance_buses
static void merge_zero_impedance_buses_impl(
    HybridPowerSystem& sys,
    bool allow_merge,
    const std::unordered_map<int, MergeSemantics>* merge_branch_semantics,
    double impedance_threshold,
    ProjectionCertificate* certificate) {
  auto& buses = sys.ac.buses;
  auto& branches = sys.ac.branches;
  const int n = static_cast<int>(buses.size());
  if (n <= 1) return;

  // Build position lookup: bus.index 鈫?0-based position
  std::unordered_map<int, int> idx_to_pos;
  idx_to_pos.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    idx_to_pos[buses[static_cast<size_t>(i)].index] = i;
  }

  // Phase 1: identify zero-impedance branches and union buses.
  // When allow_merge is false the function performs canonical reindexing only
  // (no buses are merged) — used to guarantee 1-based contiguous indices for
  // systems that contain near-zero-Z real lines rather than switches.
  UnionFind uf(n);
  bool any_merged = false;
  if (allow_merge) {
    for (const auto& br : branches) {
      if (!br.in_service) continue;
      MergeSemantics semantics = MergeSemantics::ThresholdApproximate;
      if (merge_branch_semantics != nullptr) {
        const auto candidate = merge_branch_semantics->find(br.index);
        if (candidate == merge_branch_semantics->end()) continue;
        semantics = candidate->second;
      } else if (br.r_pu == 0.0 && br.x_pu == 0.0) {
        semantics = MergeSemantics::ExactIdeal;
      }
      // A true zero-impedance element (CB/switch) has both R and X tiny
      // AND negligible charging susceptance.  Short lines may have small
      // R and X but non-zero B 鈥?these must NOT be merged because merging
      // discards their charging and distorts the admittance model.
      if (semantics == MergeSemantics::ThresholdApproximate &&
          (std::abs(br.r_pu) >= impedance_threshold ||
           std::abs(br.x_pu) >= impedance_threshold)) continue;
      if (std::abs(br.b_pu) > 1e-6) continue;  // non-trivial charging 鈫?real line

      auto it_f = idx_to_pos.find(br.from_bus);
      auto it_t = idx_to_pos.find(br.to_bus);
      if (it_f == idx_to_pos.end() || it_t == idx_to_pos.end()) continue;

      if (uf.unite(it_f->second, it_t->second)) {
        any_merged = true;
        MergeRecord record;
        record.branch_index = br.index;
        record.bus_from = br.from_bus;
        record.bus_to = br.to_bus;
        record.r_pu = br.r_pu;
        record.x_pu = br.x_pu;
        record.semantics = semantics;
        if (certificate != nullptr) certificate->merge_records.push_back(record);
      }
    }
  }

  // The entire PF/OPF stack indexes buses positionally as (bus.index - 1)
  // (see build_admittance_matrix, newton_solver, branch_flow, ac_opf …), so the
  // canonical bus list must be 1-based contiguous before it reaches a solver.
  // When nothing was merged we can normally skip this pass, but only if the
  // buses already satisfy that invariant.  A sub-network that arrives with
  // non-canonical indices — e.g. a graph-contracted island that preserves the
  // original bus IDs — and has no zero-impedance branch to trigger a merge
  // must still be reindexed; otherwise its branches map to out-of-range
  // positions and are silently dropped, producing a singular Y-bus.
  bool already_canonical = true;
  for (int i = 0; i < n; ++i) {
    if (buses[static_cast<size_t>(i)].index != i + 1) { already_canonical = false; break; }
  }
  if (!any_merged && already_canonical) return;

  // Phase 2: build equivalence classes (use std::map for deterministic
  // iteration order 鈥?groups are visited in ascending root_pos, guaranteeing
  // stable bus index assignment across runs and platforms)
  std::map<int, std::vector<int>> group_map;  // root_pos 鈫?positions
  for (int i = 0; i < n; ++i) {
    group_map[uf.find(i)].push_back(i);
  }

  // Pick representative per group: highest bus-type priority, then lowest index
  // Build old-position 鈫?new-position mapping
  BusMergeMap merge_map;
  merge_map.n_original = n;
  merge_map.projection_mode = certificate != nullptr
                                  ? certificate->mode
                                  : ProjectionMode::ThresholdApproximate;
  merge_map.impedance_threshold = impedance_threshold;
  if (certificate != nullptr) merge_map.merge_records = certificate->merge_records;

  std::unordered_map<int, double> extensive_basis;
  for (const auto& bus : buses) {
    if (bus.in_service) {
      extensive_basis[bus.index] += std::max(0.0, bus.pd_mw);
    }
  }
  for (const auto& load : sys.ac.loads) {
    if (load.in_service) {
      extensive_basis[load.bus] +=
          std::max(0.0, load.p_mw * load.scaling);
    }
  }
  for (const auto& station : sys.ac.charging_stations) {
    if (station.in_service) {
      extensive_basis[station.bus] +=
          std::max(0.0, station.p_total_kw / 1000.0);
    }
  }

  // new_pos: representative position 鈫?new sequential position
  std::vector<int> old_pos_to_new(static_cast<size_t>(n), -1);
  std::vector<ACBus> merged_buses;
  merged_buses.reserve(group_map.size());

  for (const auto& [root, members] : group_map) {
    // Select representative: highest priority, then smallest index
    int rep = members[0];
    for (int m : members) {
      int pri_m = bus_type_priority(buses[static_cast<size_t>(m)].bus_type);
      int pri_r = bus_type_priority(buses[static_cast<size_t>(rep)].bus_type);
      if (pri_m > pri_r || (pri_m == pri_r && buses[static_cast<size_t>(m)].index < buses[static_cast<size_t>(rep)].index)) {
        rep = m;
      }
    }

    const int new_pos = static_cast<int>(merged_buses.size());
    ACBus merged = buses[static_cast<size_t>(rep)];

    // Aggregate demand / shunt from all members into representative
    if (members.size() > 1) {
      double pd = 0.0, qd = 0.0, gs = 0.0, bs = 0.0;
      int total_customers = 0;
      for (int m : members) {
        const auto& b = buses[static_cast<size_t>(m)];
        pd += b.pd_mw;
        qd += b.qd_mvar;
        gs += b.gs_mw;
        bs += b.bs_mvar;
        total_customers += b.n_customers;
      }
      merged.pd_mw = pd;
      merged.qd_mvar = qd;
      merged.gs_mw = gs;
      merged.bs_mvar = bs;
      merged.n_customers = total_customers;
    }

    // Assign new contiguous index (1-based)
    merged.index = new_pos + 1;

    // Record mapping for all members
    std::vector<int> group_ext;
    group_ext.reserve(members.size());
    double group_basis = 0.0;
    for (int m : members) {
      group_basis += extensive_basis[buses[static_cast<size_t>(m)].index];
    }
    for (int m : members) {
      old_pos_to_new[static_cast<size_t>(m)] = new_pos;
      const int ext = buses[static_cast<size_t>(m)].index;
      group_ext.push_back(ext);
      merge_map.extensive_participation[ext] =
          group_basis > 1e-12
              ? extensive_basis[ext] / group_basis
              : 1.0 / static_cast<double>(members.size());
    }
    merge_map.groups.push_back(std::move(group_ext));
    merge_map.int_to_ext.push_back(buses[static_cast<size_t>(rep)].index);

    merged_buses.push_back(std::move(merged));
  }

  merge_map.n_merged = static_cast<int>(merged_buses.size());

  // Build ext_to_int: original external bus index 鈫?new internal position
  for (int i = 0; i < n; ++i) {
    const int ext = buses[static_cast<size_t>(i)].index;
    merge_map.ext_to_int[ext] = old_pos_to_new[static_cast<size_t>(i)];
    merge_map.ext_to_orig_pos[ext] = i;
  }

  // Helper: reindex an AC bus reference (1-based external 鈫?new 1-based)
  auto remap_ac = [&](int old_bus) -> int {
    auto it = merge_map.ext_to_int.find(old_bus);
    if (it != merge_map.ext_to_int.end()) return it->second + 1;
    return old_bus;  // unknown bus unchanged (safety)
  };

  // Phase 3: replace buses
  buses = std::move(merged_buses);

  // Phase 4: reindex branches and remove self-loops
  merge_map.n_original_branches = static_cast<int>(branches.size());
  std::vector<ACBranch> kept_branches;
  kept_branches.reserve(branches.size());
  for (int orig_i = 0; orig_i < static_cast<int>(branches.size()); ++orig_i) {
    auto& br = branches[static_cast<size_t>(orig_i)];
    br.from_bus = remap_ac(br.from_bus);
    br.to_bus = remap_ac(br.to_bus);
    // Remove self-loops (branches within the same merged bus)
    if (br.from_bus == br.to_bus) {
      merge_map.branch_orig_to_proj[orig_i] = -1;
      continue;
    }
    merge_map.branch_orig_to_proj[orig_i] = static_cast<int>(kept_branches.size());
    kept_branches.push_back(std::move(br));
  }
  branches = std::move(kept_branches);

  // Phase 5: reindex all AC-bus-referencing components
  for (auto& g : sys.ac.generators) g.bus = remap_ac(g.bus);
  for (auto& sg : sys.ac.static_generators) sg.bus = remap_ac(sg.bus);
  for (auto& ld : sys.ac.loads) ld.bus = remap_ac(ld.bus);
  for (auto& fl : sys.ac.flexible_loads) fl.bus = remap_ac(fl.bus);
  for (auto& al : sys.ac.asymmetric_loads) al.bus = remap_ac(al.bus);
  for (auto& sh : sys.ac.shunts) sh.bus = remap_ac(sh.bus);
  for (auto& st : sys.ac.storage) st.bus = remap_ac(st.bus);
  for (auto& rg : sys.ac.renewable_gens) rg.bus = remap_ac(rg.bus);
  for (auto& pv : sys.ac.pv_systems) pv.bus = remap_ac(pv.bus);
  for (auto& eg : sys.ac.external_grids) eg.bus = remap_ac(eg.bus);
  for (auto& sw : sys.ac.switches) {
    sw.bus_from = remap_ac(sw.bus_from);
    sw.bus_to = remap_ac(sw.bus_to);
  }
  for (auto& cb : sys.ac.circuit_breakers) {
    cb.bus_from = remap_ac(cb.bus_from);
    cb.bus_to = remap_ac(cb.bus_to);
  }
  for (auto& cs : sys.ac.charging_stations) cs.bus = remap_ac(cs.bus);
  for (auto& mot : sys.ac.motors) mot.bus = remap_ac(mot.bus);

  // Transformers: only reindex the original rich types (not the branches
  // expanded from them 鈥?those were already handled above).
  for (auto& tr : sys.ac.transformers_2w) {
    tr.hv_bus = remap_ac(tr.hv_bus);
    tr.lv_bus = remap_ac(tr.lv_bus);
  }
  for (auto& tr : sys.ac.transformers_3w) {
    tr.hv_bus = remap_ac(tr.hv_bus);
    tr.mv_bus = remap_ac(tr.mv_bus);
    tr.lv_bus = remap_ac(tr.lv_bus);
  }

  // VSC converters: only the AC side
  for (auto& conv : sys.vsc_converters) conv.bus_ac = remap_ac(conv.bus_ac);

  // Energy routers: only AC ports
  for (auto& er : sys.energy_routers) {
    for (auto& p : er.ports) {
      if (p.port_type == ERPortType::AC) p.bus = remap_ac(p.bus);
    }
  }

  // Aggregation: VPPs, Microgrids
  for (auto& vpp : sys.vpps) vpp.pcc_bus = remap_ac(vpp.pcc_bus);
  for (auto& mg : sys.microgrids) {
    mg.pcc_bus = remap_ac(mg.pcc_bus);
    for (auto& ib : mg.internal_buses) ib = remap_ac(ib);
  }

  // Mobile storage: bus might be AC or DC 鈥?only remap if it could be AC.
  // Since MobileStorage lives at the HybridPowerSystem level and typically
  // references AC buses, remap both bus and target_bus.
  for (auto& ms : sys.mobile_storage) {
    ms.bus = remap_ac(ms.bus);
    ms.target_bus = remap_ac(ms.target_bus);
  }

  sys.bus_merge_map = std::move(merge_map);
}

void merge_zero_impedance_buses(HybridPowerSystem& sys, bool allow_merge) {
  merge_zero_impedance_buses_impl(sys, allow_merge, nullptr,
                                  kBusMergeZThreshold, nullptr);
}

// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
// unproject_bus_vector
// 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
std::vector<double> unproject_bus_vector(
    const std::vector<double>& merged,
    const BusMergeMap& map,
    BusVectorSemantics semantics) {
  // The input must live in the canonical (merged) bus space this map was
  // built for.  A silent partial broadcast here turns an index-space mixup
  // into plausible-looking zeros downstream, so reject it instead.
  if (map.n_merged > 0 && static_cast<int>(merged.size()) != map.n_merged) {
    throw std::invalid_argument(
        "unproject_bus_vector: input size " + std::to_string(merged.size()) +
        " does not match BusMergeMap.n_merged " + std::to_string(map.n_merged) +
        " (canonical/original index-space mismatch)");
  }
  std::vector<double> out(static_cast<size_t>(map.n_original), 0.0);
  for (const auto& [ext_bus, int_pos] : map.ext_to_int) {
    if (int_pos < 0 || int_pos >= static_cast<int>(merged.size())) continue;
    int orig_pos = -1;
    auto it_pos = map.ext_to_orig_pos.find(ext_bus);
    if (it_pos != map.ext_to_orig_pos.end()) {
      orig_pos = it_pos->second;
    }
    if (orig_pos < 0 || orig_pos >= map.n_original) continue;
    double value = merged[static_cast<size_t>(int_pos)];
    if (semantics == BusVectorSemantics::Extensive) {
      const auto weight = map.extensive_participation.find(ext_bus);
      if (weight == map.extensive_participation.end()) {
        throw std::invalid_argument(
            "unproject_bus_vector: extensive quantity has no participation "
            "factor for original bus " + std::to_string(ext_bus));
      }
      value *= weight->second;
    }
    out[static_cast<size_t>(orig_pos)] = value;
  }
  return out;
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// Energy Router 鈫?macro expansion into internal DC buses + VSCs + DCDC
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//
// Topology:
//   AC port (side A) 鈹€鈹€[VSC]鈹€鈹€ DC_伪 鈹€鈹€[DCDC]鈹€鈹€ DC_尾 鈹€鈹€[VSC]鈹€鈹€ AC port (side B)
//
// Control mode mapping (ERControlMode 鈫?ConverterMode / DCDCControlMode):
//   port VF  鈫?VSC VDC_Q_MODE  (establishes internal DC bus voltage)
//              + AC bus promoted to PV (holds AC voltage at v_set_pu)
//   port PQ  鈫?VSC PQ_MODE     (specified power transfer)
//   port Droop鈫?VSC DROOP_MODE (frequency/voltage droop)
//
// If no port on a side has VF control, the first in-service port on
// that side is auto-promoted to VDC_Q_MODE.
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
static void expand_energy_routers(HybridPowerSystem& sys) {
  if (sys.energy_routers.empty()) return;

  // Find the next available indices for new elements.
  int next_dc_bus = next_index_of(sys.dc.buses);
  int next_vsc    = next_index_of(sys.vsc_converters);
  int next_dcdc   = next_index_of(sys.dc.dcdc_converters);

  for (const auto& er : sys.energy_routers) {
    if (!er.in_service) continue;

    // Partition ports by side.
    std::vector<const EnergyRouterPort*> side_a, side_b;
    for (const auto& port : er.ports) {
      if (!port.in_service) continue;
      if (port.bus == 0) continue;
      if (port.side == 0)
        side_a.push_back(&port);
      else
        side_b.push_back(&port);
    }

    if (side_a.empty() && side_b.empty()) continue;

    // 鈹€鈹€ Create Internal DC Buses 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    const double dc_kv = (er.vn_dc_kv > 0.0) ? er.vn_dc_kv : 20.0;
    const double dcdc_eta = (er.loss_percent > 0.0 && er.loss_percent < 100.0)
                                ? (1.0 - er.loss_percent / 100.0)
                                : 0.98;

    DCBus dc_alpha;
    dc_alpha.index      = next_dc_bus++;
    dc_alpha.bus_type    = DCBusType::DC_P;
    dc_alpha.vm_pu       = 1.0;
    dc_alpha.base_kv     = dc_kv;
    dc_alpha.in_service  = true;
    dc_alpha.name        = er.name + "_DC_alpha";

    DCBus dc_beta;
    dc_beta.index       = next_dc_bus++;
    dc_beta.bus_type     = DCBusType::DC_P;
    dc_beta.vm_pu        = 1.0;
    dc_beta.base_kv      = dc_kv;
    dc_beta.in_service   = true;
    dc_beta.name         = er.name + "_DC_beta";

    sys.dc.buses.push_back(dc_alpha);
    sys.dc.buses.push_back(dc_beta);

    // --- Compute appropriate k_vdc for VDC_Q modes ---
    // The VDC_Q formula is p = k_vdc * (vdc^2 - v_set^2) [per-unit].
    // k_vdc must be large enough so that rated power transfers with
    // only a small DC-voltage deviation.
    const double base = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;
    const double p_rated_pu = er.p_rated_mw / base;
    // VF / auto-promote: large gain for tight voltage control (like DCDC test k=100)
    const double k_vdc_tight = std::max(p_rated_pu * 100.0, 25.0);
    // Droop: moderate gain derived from rated power and 5% droop bandwidth
    const double k_vdc_droop = std::max(p_rated_pu / (2.0 * 0.05), 1.0);

    // 鈹€鈹€ Create VSCs for Side A ports 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    // Check if any side A port has VF control.
    bool side_a_has_vf = false;
    for (const auto* p : side_a) {
      if (p->control_mode == ERControlMode::VF) { side_a_has_vf = true; break; }
    }
    for (size_t i = 0; i < side_a.size(); ++i) {
      const auto* p = side_a[i];
      VSCConverter vsc;
      vsc.index        = next_vsc++;
      vsc.bus_ac       = p->bus;
      vsc.bus_dc       = dc_alpha.index;
      vsc.in_service   = true;
      vsc.name         = er.name + "_VSC_A" + std::to_string(p->index);

      // Map control mode.
      if (p->control_mode == ERControlMode::VF) {
        vsc.control_mode = ConverterMode::VDC_Q;
        vsc.v_dc_set_pu  = p->v_set_pu;
        vsc.k_vdc        = k_vdc_tight;
        // VF (grid-forming) also controls AC voltage: promote AC bus to PV.
        if (p->port_type == ERPortType::AC) {
          for (auto& ab : sys.ac.buses) {
            if (ab.index == p->bus && ab.bus_type == BusType::PQ) {
              ab.bus_type = BusType::PV;
              ab.vm_pu    = p->v_set_pu;
              break;
            }
          }
        }
      } else if (p->control_mode == ERControlMode::Droop) {
        // Map Droop to VDC_Q with non-zero droop gain k_vdc.
        vsc.control_mode = ConverterMode::VDC_Q;
        vsc.v_dc_set_pu  = p->v_set_pu;
        vsc.k_vdc        = k_vdc_droop;
        side_a_has_vf = true;  // droop provides DC voltage reference
      } else {
        // PQ mode.
        // Auto-promote first port to VDC if no VF exists on this side.
        if (!side_a_has_vf && i == 0) {
          vsc.control_mode = ConverterMode::VDC_Q;
          vsc.v_dc_set_pu  = 1.0;
          vsc.k_vdc        = k_vdc_tight;
          side_a_has_vf = true;  // prevent further promotions
        } else {
          vsc.control_mode = ConverterMode::PQ_MODE;
        }
      }

      vsc.p_set_mw     = p->p_set_mw;
      vsc.q_set_mvar   = p->q_set_mvar;
      vsc.v_ac_set_pu  = p->v_set_pu;
      vsc.eta          = (p->eta > 1e-9 && p->eta <= 1.0) ? p->eta : 0.98;
      vsc.pmax_mw      = (p->pmax_mw != 0.0) ? p->pmax_mw : er.pmax_mw;
      vsc.pmin_mw      = (p->pmin_mw != 0.0) ? p->pmin_mw : er.pmin_mw;
      vsc.qmax_mvar    = (p->qmax_mvar != 0.0) ? p->qmax_mvar : er.qmax_mvar;
      vsc.qmin_mvar    = (p->qmin_mvar != 0.0) ? p->qmin_mvar : er.qmin_mvar;
      vsc.p_rated_mw   = er.p_rated_mw;
      vsc.vn_ac_kv     = (p->voltage_level_kv > 0.0) ? p->voltage_level_kv : er.vn_ac_kv;
      vsc.vn_dc_kv     = dc_kv;
      vsc.controllable = true;
      vsc.dynamic_model = energy_router_profile_for(er, p, "VSC converter");

      sys.vsc_converters.push_back(vsc);
    }

    // 鈹€鈹€ Create VSCs for Side B ports 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    bool side_b_has_vf = false;
    for (const auto* p : side_b) {
      if (p->control_mode == ERControlMode::VF) { side_b_has_vf = true; break; }
    }
    for (size_t i = 0; i < side_b.size(); ++i) {
      const auto* p = side_b[i];
      VSCConverter vsc;
      vsc.index        = next_vsc++;
      vsc.bus_ac       = p->bus;
      vsc.bus_dc       = dc_beta.index;
      vsc.in_service   = true;
      vsc.name         = er.name + "_VSC_B" + std::to_string(p->index);

      if (p->control_mode == ERControlMode::VF) {
        vsc.control_mode = ConverterMode::VDC_Q;
        vsc.v_dc_set_pu  = p->v_set_pu;
        vsc.k_vdc        = k_vdc_tight;
        // VF (grid-forming) also controls AC voltage: promote AC bus to PV.
        if (p->port_type == ERPortType::AC) {
          for (auto& ab : sys.ac.buses) {
            if (ab.index == p->bus && ab.bus_type == BusType::PQ) {
              ab.bus_type = BusType::PV;
              ab.vm_pu    = p->v_set_pu;
              break;
            }
          }
        }
      } else if (p->control_mode == ERControlMode::Droop) {
        vsc.control_mode = ConverterMode::VDC_Q;
        vsc.v_dc_set_pu  = p->v_set_pu;
        vsc.k_vdc        = k_vdc_droop;
      } else {
        // PQ — side B ports are normally PQ since DCDC controls DC_β.
        vsc.control_mode = ConverterMode::PQ_MODE;
      }

      vsc.p_set_mw     = p->p_set_mw;
      vsc.q_set_mvar   = p->q_set_mvar;
      vsc.v_ac_set_pu  = p->v_set_pu;
      vsc.eta          = (p->eta > 1e-9 && p->eta <= 1.0) ? p->eta : 0.98;
      vsc.pmax_mw      = (p->pmax_mw != 0.0) ? p->pmax_mw : er.pmax_mw;
      vsc.pmin_mw      = (p->pmin_mw != 0.0) ? p->pmin_mw : er.pmin_mw;
      vsc.qmax_mvar    = (p->qmax_mvar != 0.0) ? p->qmax_mvar : er.qmax_mvar;
      vsc.qmin_mvar    = (p->qmin_mvar != 0.0) ? p->qmin_mvar : er.qmin_mvar;
      vsc.p_rated_mw   = er.p_rated_mw;
      vsc.vn_ac_kv     = (p->voltage_level_kv > 0.0) ? p->voltage_level_kv : er.vn_ac_kv;
      vsc.vn_dc_kv     = dc_kv;
      vsc.controllable = true;
      vsc.dynamic_model = energy_router_profile_for(er, p, "VSC converter");

      sys.vsc_converters.push_back(vsc);
    }

    // ── Create DCDC converter between DC_α and DC_β ─────────────────
    // Check if side B has voltage-forming ports.  When side B has VF/Droop,
    // the expanded VSC already controls dc_beta via VDC_Q.  Using DCDC in
    // Voltage mode would ALSO clamp dc_beta → VDC_Q formula gives zero
    // power (k*(V²−V_set²)=0).  So switch DCDC to Power mode, with
    // p_ref estimated from the sum of side B PQ port setpoints.
    bool side_b_controls_dc = false;
    for (const auto* p : side_b) {
      if (p->control_mode == ERControlMode::VF ||
          p->control_mode == ERControlMode::Droop) {
        side_b_controls_dc = true;
        break;
      }
    }

    DCDCConverter dcdc;
    dcdc.index        = next_dcdc++;
    dcdc.bus_in       = dc_alpha.index;
    dcdc.bus_out      = dc_beta.index;
    dcdc.in_service   = true;
    dcdc.name         = er.name + "_DCDC";
    if (side_b_controls_dc) {
      // Side B VF/Droop controls dc_beta voltage → DCDC uses Power mode.
      dcdc.control_mode = DCDCControlMode::Power;
      double p_ref = 0.0;
      for (const auto* p : side_b) {
        if (p->control_mode == ERControlMode::PQ && p->p_set_mw > 0.0)
          p_ref += p->p_set_mw;
      }
      // Add headroom for the VF port (estimate: half of remaining rated capacity)
      double remaining = er.p_rated_mw - p_ref;
      if (remaining > 0.0) p_ref += remaining * 0.5;
      dcdc.p_ref_mw  = p_ref;
      dcdc.v_ref_pu  = 1.0;
    } else {
      dcdc.control_mode = DCDCControlMode::Voltage;
      dcdc.v_ref_pu     = 1.0;
    }
    dcdc.eta          = dcdc_eta;
    dcdc.sn_mva       = er.p_rated_mw;
    dcdc.vn_in_kv     = dc_kv;
    dcdc.vn_out_kv    = dc_kv;
    dcdc.pmax_mw      = er.pmax_mw;
    dcdc.pmin_mw      = er.pmin_mw;
    dcdc.controllable = true;
    dcdc.mtbf_hours   = er.mtbf_hours;
    dcdc.mttr_hours   = er.mttr_hours;
    dcdc.dynamic_model = energy_router_profile_for(er, nullptr, "DCDC converter");

    sys.dc.dcdc_converters.push_back(dcdc);
  }

  // All ERs have been expanded 鈥?clear the vector to prevent double-counting
  // in residual_evaluator / jacobian_builder which still iterate over
  // energy_routers for legacy fixed-injection paths.
  sys.energy_routers.clear();
}

// Internal helper: project a mutable HybridPowerSystem in place.
static void project_in_place(HybridPowerSystem& out,
                             const ProjectionOptions& options) {
  ProjectionCertificate projection_certificate;
  projection_certificate.mode = options.mode;
  projection_certificate.impedance_threshold = options.impedance_threshold;
  projection_certificate.switch_branches_preserved =
      options.preserve_switch_branches;
  projection_certificate.n_authored_dc_buses =
      static_cast<int>(out.dc.buses.size());
  ProjectionReport projection_report;

  auto record_mapping = [&](const std::string& source_type,
                            int source_index,
                            const std::string& canonical_type,
                            int canonical_index,
                            double participation = 1.0) {
    projection_report.mappings.push_back(
        {source_type, std::to_string(source_index), canonical_type,
         std::to_string(canonical_index), participation});
  };
  if (out.base_mva <= 1e-9) {
    out.base_mva = 100.0;
  }
  if (out.ac.base_mva <= 1e-9) {
    out.ac.base_mva = out.base_mva;
  }
  if (out.dc.base_mva <= 1e-9) {
    out.dc.base_mva = out.base_mva;
  }

  // Fill in per-unit branch impedances from any actual (ohm/km) engineering
  // data so that the rest of projection and every downstream solver can start
  // from real physical values.  No-op for systems already given in per-unit.
  convert_actual_to_per_unit(out);

  // Idempotence guard (sppt_theory.tex, Prop. "Idempotence of normalization
  // and re-projection"): a system carrying a projection certificate is
  // already a canonical model.  Expansion must not run again — kept rich
  // components (flexible/asymmetric loads, standalone transformers,
  // switches/CBs) would synthesise duplicate equivalent loads and branches,
  // silently doubling demand and admittance.  Only the idempotent stages run:
  // canonical reindexing, and dead-island stripping so a keep-dead-islands
  // projection can still be stripped later (the strip stage composes with the
  // carried provenance maps).  Carried provenance is preserved, not rebuilt.
  if (out.projection_certificate.has_value()) {
    const auto& carried = *out.projection_certificate;
    if (carried.mode != options.mode ||
        carried.switch_branches_preserved != options.preserve_switch_branches ||
        std::abs(carried.impedance_threshold - options.impedance_threshold) >
            1e-15) {
      throw std::invalid_argument(
          "project_to_canonical_models: cannot re-project a canonical model "
          "with different projection mode or impedance threshold");
    }
    merge_zero_impedance_buses_impl(out, /*allow_merge=*/false, nullptr,
                                    options.impedance_threshold, nullptr);
    if (options.strip_dead_islands) strip_dead_islands(out);
    return;
  }

  project_three_phase_if_needed(out);

  // When FlexibleLoads or AsymmetricLoads will be projected into the loads
  // table, first migrate any bus-level pd_mw/qd_mvar into Load entries so
  // that the loads table becomes the single authoritative source.  Consumers
  // (ONR, DPF, PF) use an exclusive if/else: if loads table is non-empty
  // they ignore bus pd_mw.  Without this normalisation step, adding a single
  // equivalent Load would cause all bus-level demand to be dropped.
  const bool will_add_loads =
      std::any_of(out.ac.flexible_loads.begin(), out.ac.flexible_loads.end(),
                  [](const FlexibleLoad& fl) { return fl.in_service && fl.bus != 0; }) ||
      std::any_of(out.ac.asymmetric_loads.begin(), out.ac.asymmetric_loads.end(),
                  [](const AsymmetricLoad& al) { return al.in_service && al.bus != 0; }) ||
      std::any_of(out.ac.motors.begin(), out.ac.motors.end(),
                  [](const AsynchronousMotor& m) { return m.in_service && m.bus != 0; });

  int next_ld = next_index_of(out.ac.loads);

  if (will_add_loads && out.ac.loads.empty()) {
    // Promote bus-level demand to explicit Load entries.
    for (auto& bus : out.ac.buses) {
      if (std::abs(bus.pd_mw) < 1e-15 && std::abs(bus.qd_mvar) < 1e-15) continue;
      Load ld;
      ld.index = next_ld++;
      ld.bus = bus.index;
      ld.in_service = true;
      ld.name = "BusLoad_" + std::to_string(bus.index);
      ld.p_mw = bus.pd_mw;
      ld.q_mvar = bus.qd_mvar;
      ld.model = LoadModel::ConstantPower;
      ld.p_percent_p = 100.0;
      ld.p_percent_q = 100.0;
      out.ac.loads.push_back(ld);
      bus.pd_mw = 0.0;
      bus.qd_mvar = 0.0;
    }
  }

  for (const auto& fl : out.ac.flexible_loads) {
    const int before = next_ld;
    add_equivalent_load_from_flexible(fl, out.ac, next_ld);
    if (next_ld > before) record_mapping("FlexibleLoad", fl.index, "Load", before);
  }
  for (const auto& al : out.ac.asymmetric_loads) {
    const int before = next_ld;
    add_equivalent_load_from_asymmetric(al, out.ac, next_ld);
    if (next_ld > before) record_mapping("AsymmetricLoad", al.index, "Load", before);
  }
  for (const auto& m : out.ac.motors) {
    const int before = next_ld;
    add_equivalent_load_from_motor(m, out.ac, next_ld);
    if (next_ld > before) record_mapping("AsynchronousMotor", m.index, "Load", before);
  }

  // EnergyRouters 鈫?macro expansion into internal DC buses + VSCs + DCDC
  const auto authored_energy_routers = out.energy_routers;
  const std::size_t vsc_before_router = out.vsc_converters.size();
  const std::size_t dcdc_before_router = out.dc.dcdc_converters.size();
  expand_energy_routers(out);
  for (const auto& er : authored_energy_routers) {
    const std::string prefix = er.name + "_";
    for (std::size_t i = vsc_before_router; i < out.vsc_converters.size(); ++i) {
      const auto& vsc = out.vsc_converters[i];
      if (vsc.name.rfind(prefix, 0) == 0)
        record_mapping("EnergyRouter", er.index, "VSCConverter", vsc.index);
    }
    for (std::size_t i = dcdc_before_router; i < out.dc.dcdc_converters.size(); ++i) {
      const auto& dcdc = out.dc.dcdc_converters[i];
      if (dcdc.name.rfind(prefix, 0) == 0)
        record_mapping("EnergyRouter", er.index, "DCDCConverter", dcdc.index);
    }
  }

  // VirtualPowerPlants 鈫?StaticGenerator at aggregation bus
  int next_sg = next_index_of(out.ac.static_generators);
  for (const auto& vpp : out.vpps) {
    const int before = next_sg;
    add_equivalent_static_gen_from_vpp(vpp, out.ac, next_sg);
    if (next_sg > before)
      record_mapping("VirtualPowerPlant", vpp.index, "StaticGenerator", before);
  }
  for (const auto& mg : out.microgrids) {
    const int before = next_sg;
    add_equivalent_static_gen_from_microgrid(mg, out.ac, next_sg);
    if (next_sg > before)
      record_mapping("Microgrid", mg.index, "StaticGenerator", before);
  }

  // MobileStorage 鈫?canonical AC Storage at current bus
  int next_st = next_index_of(out.ac.storage);
  for (const auto& ms : out.mobile_storage) {
    const int before = next_st;
    add_equivalent_storage_from_mobile_storage(ms, out.ac, next_st);
    if (next_st > before)
      record_mapping("MobileStorage", ms.index, "Storage", before);
  }

  // Track branch expansion provenance for Transformer2W/3W and Switch.
  // This lets callers map solver branch-flow indices back to original elements.
  BranchExpandMap bmap;
  std::unordered_map<int, MergeSemantics> switch_merge_branch_semantics;
  int next_br = next_index_of(out.ac.branches);

  for (const auto& tr : out.ac.transformers_2w) {
    if (tr.source_branch_idx > 0) {
      // This Transformer2W was auto-extracted from a MATPOWER branch
      // that is still in the branch list.  The original branch retains
      // exact pi-model data (r, x, b, tap) 鈥?do NOT overwrite with
      // round-tripped values.  Just record the provenance mapping.
      bmap.entries.push_back({tr.source_branch_idx, BranchOriginType::Transformer2W, tr.index, 0});
      record_mapping("Transformer2W", tr.index, "ACBranch", tr.source_branch_idx);
    } else {
      // Standalone Transformer2W (e.g. from Excel/JSON import) 鈥?create
      // equivalent branch as before.
      const int br_before = next_br;
      add_equivalent_branch_from_transformer2w(tr, out.ac, out.base_mva, next_br);
      if (next_br > br_before) {
        bmap.entries.push_back({br_before, BranchOriginType::Transformer2W, tr.index, 0});
        record_mapping("Transformer2W", tr.index, "ACBranch", br_before);
      }
    }
  }
  for (const auto& tr : out.ac.transformers_3w) {
    const int br_before = next_br;
    add_equivalent_branches_from_transformer3w(tr, out.ac, out.base_mva, next_br);
    // Record each pair branch in HV-MV / HV-LV / MV-LV order
    for (int pair = 0; br_before + pair < next_br; ++pair) {
      bmap.entries.push_back({br_before + pair, BranchOriginType::Transformer3W, tr.index, pair});
      record_mapping("Transformer3W", tr.index, "ACBranch", br_before + pair,
                     1.0 / 3.0);
    }
  }
  for (const auto& sw : out.ac.switches) {
    const int br_before = next_br;
    add_equivalent_branch_from_switch(sw, out.ac, out.base_mva, next_br);
    if (next_br > br_before) {
      BranchExpandEntry e;
      e.branch_index = br_before;
      e.origin_type = BranchOriginType::Switch;
      e.origin_index = sw.index;
      e.bus_from = sw.bus_from;
      e.bus_to = sw.bus_to;
      e.closed = sw.closed;
      bmap.entries.push_back(e);
      record_mapping("Switch", sw.index, "ACBranch", e.branch_index);
      if (sw.closed) {
        const bool authored_ideal = sw.r_contact_ohm == 0.0 && sw.z_ohm == 0.0;
        if (authored_ideal || options.mode == ProjectionMode::ThresholdApproximate) {
          switch_merge_branch_semantics[e.branch_index] =
              authored_ideal ? MergeSemantics::ExactIdeal
                             : MergeSemantics::ThresholdApproximate;
        }
      }
    }
  }
  for (const auto& cb : out.ac.circuit_breakers) {
    const int br_before = next_br;
    add_equivalent_branch_from_circuit_breaker(cb, out.ac, out.base_mva, next_br);
    if (next_br > br_before) {
      BranchExpandEntry e;
      e.branch_index = br_before;
      e.origin_type = BranchOriginType::CircuitBreaker;
      e.origin_index = cb.index;
      e.bus_from = cb.bus_from;
      e.bus_to = cb.bus_to;
      e.closed = cb.closed;
      bmap.entries.push_back(e);
      record_mapping("CircuitBreaker", cb.index, "ACBranch", e.branch_index);
      if (cb.closed) {
        const bool authored_ideal = cb.z_ohm == 0.0;
        if (authored_ideal || options.mode == ProjectionMode::ThresholdApproximate) {
          switch_merge_branch_semantics[e.branch_index] =
              authored_ideal ? MergeSemantics::ExactIdeal
                             : MergeSemantics::ThresholdApproximate;
        }
      }
    }
  }
  if (!bmap.empty()) out.branch_expand_map = std::move(bmap);

  // Remove rich elements that are fully represented by their canonical
  // equivalents inserted above.  This prevents double-counting in SC
  // admittance builders and simplifies downstream iteration.
  out.ac.motors.clear();
  out.ac.transformers_3w.clear();   // pair branches now live in ac.branches
  out.vpps.clear();
  out.microgrids.clear();
  out.mobile_storage.clear();

  // Merge buses connected by zero-impedance branches created from closed
  // switch/circuit-breaker expansion.  Use provenance as a whitelist: practical
  // GridLAB-D feeders can contain real very-short lines whose per-unit R/X are
  // below kBusMergeZThreshold, and those physical lines must remain branches.
  // In either case the bus list is canonicalized to 1-based contiguous indices,
  // which every positional PF/OPF builder (index-1) relies on.
  merge_zero_impedance_buses_impl(out,
                                  !options.preserve_switch_branches &&
                                      !switch_merge_branch_semantics.empty(),
                                  &switch_merge_branch_semantics,
                                  options.impedance_threshold,
                                  &projection_certificate);

  // Strip dead islands: remove buses with no path to any generation
  // source.  This ensures all downstream algorithms (PF, OPF, DPF,
  // time-series, reliability, etc.) receive a clean system without
  // isolated dead nodes that would cause singular admittance matrices.
  // Reconfiguration skips this so open ties into de-energised sections
  // remain valid reconnection candidates.
  if (options.strip_dead_islands) strip_dead_islands(out);
  if (out.bus_merge_map) {
    out.bus_merge_map->projection_mode = options.mode;
    out.bus_merge_map->impedance_threshold = options.impedance_threshold;
    out.bus_merge_map->merge_records = projection_certificate.merge_records;
  }
  out.projection_certificate = projection_certificate;
  out.projection_report = projection_report;

  if (!out.ac.chargers.empty()) {
    if (out.ac.charging_stations.empty()) {
      int next_cs = next_index_of(out.ac.charging_stations);
      std::unordered_map<int, std::vector<const Charger*>> by_station;
      for (const auto& ch : out.ac.chargers) {
        if (!ch.in_service) continue;
        by_station[ch.station_id].push_back(&ch);
      }
      for (const auto& [station_id, chargers] : by_station) {
        if (chargers.empty()) continue;
        ChargingStation cs;
        cs.index = (station_id != 0) ? station_id : next_cs++;
        cs.name = "StationFromChargers_" + std::to_string(cs.index);
        cs.in_service = true;
        cs.bus = 0;
        cs.num_chargers = static_cast<int>(chargers.size());
        for (const Charger* ch : chargers) {
          cs.p_total_kw += std::max(0.0, ch->p_ch_max_kw);
        }
        out.ac.charging_stations.push_back(cs);
      }
    }
    project_chargers_into_stations(out.ac);
    out.ac.chargers.clear();  // fully represented by charging_stations
  }
}

static DeviceTerminalFlows device_terminal_flows_from_net(
    const HybridPowerSystem& sys,
    const std::unordered_map<int, double>& net_p,
    const std::unordered_map<int, double>& net_q) {
  DeviceTerminalFlows out;
  auto terminal_residual = [&](int bus) {
    const auto active = net_p.find(bus);
    const auto reactive = net_q.find(bus);
    return std::pair<double, double>{
        active == net_p.end() ? 0.0 : active->second,
        reactive == net_q.end() ? 0.0 : reactive->second};
  };
  auto fill = [&](DeviceTerminalFlow& f, int from, int to, bool closed, double rate) {
    f.bus_from = from; f.bus_to = to; f.closed = closed; f.rate_mva = rate;
    if (!closed || !from || !to || from == to) return;
    // After authored line flows have been subtracted, the residual at the
    // receiving terminal is exactly the collapsed device's terminal injection.
    // Unlike a topology-cut BFS, this remains valid when another path connects
    // the two endpoint areas.
    auto [p, q] = terminal_residual(to);
    f.pf_mw = -p; f.pt_mw = p; f.qf_mvar = -q; f.qt_mvar = q;
    if (rate > 0) f.loading_pct = 100.0 * std::hypot(p, q) / rate;
  };
  for (const auto& sw : sys.ac.switches) {
    DeviceTerminalFlow f; f.index = sw.index;
    const double kv = bus_base_kv_or_default(sys.ac, sw.bus_from);
    fill(f, sw.bus_from, sw.bus_to, sw.in_service && sw.closed,
         sw.i_rated_ka > 0 ? std::sqrt(3.0) * kv * sw.i_rated_ka : 0.0);
    out.ac_switches.push_back(f);
  }
  for (const auto& cb : sys.ac.circuit_breakers) {
    DeviceTerminalFlow f; f.index = cb.index;
    const double kv = bus_base_kv_or_default(sys.ac, cb.bus_from);
    fill(f, cb.bus_from, cb.bus_to, cb.in_service && cb.closed,
         cb.i_rated_ka > 0 ? std::sqrt(3.0) * kv * cb.i_rated_ka : 0.0);
    out.ac_circuit_breakers.push_back(f);
  }
  return out;
}

DeviceTerminalFlows compute_device_terminal_flows(const HybridPowerSystem& sys,
                                                  const std::vector<BranchFlow>& ac_branch_flows) {
  // Residual KCL after subtracting authored line flows is the injection carried
  // by collapsed switches/CBs. This retains solved meshed-network flows while
  // preserving the actual rich-side load/generation distribution.
  std::unordered_map<int, double> net_p, net_q;
  auto add_p = [&](int bus, double value) {
    if (bus != 0) net_p[bus] += value;
  };
  auto add_q = [&](int bus, double value) {
    if (bus != 0) net_q[bus] += value;
  };
  for (const auto& bus : sys.ac.buses) {
    add_p(bus.index, -bus.pd_mw);
    add_q(bus.index, -bus.qd_mvar);
  }
  for (const auto& load : sys.ac.loads) {
    if (!load.in_service) continue;
    add_p(load.bus, -load.p_mw * load.scaling);
    add_q(load.bus, -load.q_mvar * load.scaling);
  }
  for (const auto& load : sys.ac.flexible_loads) {
    if (!load.in_service) continue;
    add_p(load.bus, -load.p_mw);
    add_q(load.bus, -load.q_mvar);
  }
  for (const auto& load : sys.ac.asymmetric_loads) {
    if (!load.in_service) continue;
    add_p(load.bus, -(load.pa_mw + load.pb_mw + load.pc_mw) * load.scaling);
    add_q(load.bus,
          -(load.qa_mvar + load.qb_mvar + load.qc_mvar) * load.scaling);
  }
  for (const auto& station : sys.ac.charging_stations) {
    if (!station.in_service) continue;
    add_p(station.bus, -station.p_total_kw / 1000.0);
    add_q(station.bus, -station.q_total_kvar / 1000.0);
  }
  for (const auto& gen : sys.ac.generators) {
    if (!gen.in_service) continue;
    add_p(gen.bus, gen.pg_mw);
    add_q(gen.bus, gen.qg_mvar);
  }
  for (const auto& gen : sys.ac.static_generators) {
    if (!gen.in_service) continue;
    add_p(gen.bus, gen.p_mw * gen.scaling);
    add_q(gen.bus, gen.q_mvar * gen.scaling);
  }
  for (const auto& gen : sys.ac.renewable_gens) {
    if (gen.in_service) add_p(gen.bus, gen.p_mw);
  }
  for (const auto& pv : sys.ac.pv_systems) {
    if (!pv.in_service) continue;
    add_p(pv.bus, pv.p_mw);
    add_q(pv.bus, pv.q_mvar);
  }
  for (const auto& storage : sys.ac.storage) {
    if (!storage.in_service) continue;
    add_p(storage.bus, storage.p_mw);
    add_q(storage.bus, storage.q_mvar);
  }
  const size_t n = std::min(ac_branch_flows.size(), sys.ac.branches.size());
  for (size_t i = 0; i < n; ++i) {
    const auto& br = sys.ac.branches[i];
    if (!br.in_service) continue;
    net_p[br.from_bus] -= ac_branch_flows[i].pf_mw;
    net_p[br.to_bus] -= ac_branch_flows[i].pt_mw;
    net_q[br.from_bus] -= ac_branch_flows[i].qf_mvar;
    net_q[br.to_bus] -= ac_branch_flows[i].qt_mvar;
  }
  return device_terminal_flows_from_net(sys, net_p, net_q);
}

ObservableAttribution evaluate_attribution(
    const HybridPowerSystem& original,
    const HybridPowerSystem& projected,
    ObservableKind observable) {
  ObservableAttribution out;
  out.observable = observable;

  const bool projection_exact =
      !projected.projection_certificate || projected.projection_certificate->exact();
  const RecoveryClass electrical_recovery =
      projection_exact ? RecoveryClass::Strong : RecoveryClass::Approximate;

  switch (observable) {
    case ObservableKind::ACBusVoltage: {
      out.canonical_entities = static_cast<int>(original.ac.buses.size());
      if (!projected.bus_merge_map) {
        out.attributed_entities =
            projected.ac.buses.size() == original.ac.buses.size()
                ? out.canonical_entities
                : 0;
      } else {
        out.attributed_entities =
            static_cast<int>(projected.bus_merge_map->ext_to_orig_pos.size());
      }
      out.recovery = out.attributed_entities == out.canonical_entities
                         ? electrical_recovery
                         : RecoveryClass::Unsupported;
      out.reason = projection_exact
                       ? "bus identity/broadcast map covers authored AC buses"
                       : "bus broadcast is total with threshold-merge semantics";
      return out;
    }
    case ObservableKind::DCBusVoltage:
      out.canonical_entities = static_cast<int>(original.dc.buses.size());
      out.attributed_entities =
          projected.dc.buses.size() == original.dc.buses.size()
              ? out.canonical_entities
              : 0;
      out.recovery = out.attributed_entities == out.canonical_entities
                         ? RecoveryClass::Strong
                         : RecoveryClass::Unsupported;
      out.reason = "DC buses retain stable component identities";
      return out;
    case ObservableKind::ACBranchTerminalFlow:
      out.canonical_entities = static_cast<int>(original.ac.branches.size());
      out.attributed_entities = out.canonical_entities;
      if (projected.bus_merge_map &&
          projected.bus_merge_map->n_original_branches > 0) {
        out.attributed_entities = 0;
        for (const auto& [original_position, projected_position] :
             projected.bus_merge_map->branch_orig_to_proj) {
          (void)projected_position;
          if (original_position >= 0 && original_position < out.canonical_entities)
            ++out.attributed_entities;
        }
      }
      out.recovery = out.attributed_entities == out.canonical_entities
                         ? electrical_recovery
                         : RecoveryClass::Unsupported;
      out.reason = "original branch positions map to projected branch positions";
      return out;
    case ObservableKind::SwitchTerminalFlow: {
      out.canonical_entities = static_cast<int>(original.ac.switches.size() +
                                                original.ac.circuit_breakers.size());
      int mapped = 0;
      if (projected.projection_report) {
        mapped += projected.projection_report->count_source("Switch");
        mapped += projected.projection_report->count_source("CircuitBreaker");
      }
      out.attributed_entities = mapped;
      out.recovery = mapped == out.canonical_entities
                         ? electrical_recovery
                         : RecoveryClass::Unsupported;
      out.reason = "expanded device origins and terminal cuts are recorded";
      return out;
    }
    case ObservableKind::ConverterTransfer:
      out.canonical_entities = static_cast<int>(original.vsc_converters.size());
      out.attributed_entities =
          projected.vsc_converters.size() >= original.vsc_converters.size()
              ? out.canonical_entities
              : 0;
      out.recovery = out.attributed_entities == out.canonical_entities
                         ? RecoveryClass::Strong
                         : RecoveryClass::Unsupported;
      out.reason = "authored VSC identities survive canonical projection";
      return out;
    case ObservableKind::AggregatedDeviceInjection:
      out.canonical_entities = projected.projection_report
                                   ? static_cast<int>(projected.projection_report->mappings.size())
                                   : 0;
      out.attributed_entities = out.canonical_entities;
      out.recovery = RecoveryClass::AuditOnly;
      out.reason = "source-to-canonical mappings are auditable; physical splits require a declared participation law";
      return out;
    case ObservableKind::NodalDual: {
      out = evaluate_attribution(original, projected, ObservableKind::ACBusVoltage);
      out.observable = ObservableKind::NodalDual;
      out.reason = "dual attribution follows surviving AC nodal constraints";
      return out;
    }
    case ObservableKind::ServiceLoss:
      out.recovery = RecoveryClass::Unsupported;
      out.reason = "service-loss attribution requires a temporal curtailment contract";
      return out;
  }
  return out;
}

HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys) {
  HybridPowerSystem out = sys;
  project_in_place(out, ProjectionOptions{});
  return out;
}

HybridPowerSystem project_to_canonical_models(HybridPowerSystem&& sys) {
  project_in_place(sys, ProjectionOptions{});
  return std::move(sys);
}

HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys,
                                              const ProjectionOptions& options) {
  HybridPowerSystem out = sys;
  project_in_place(out, options);
  return out;
}

HybridPowerSystem project_to_canonical_models(HybridPowerSystem&& sys,
                                              const ProjectionOptions& options) {
  project_in_place(sys, options);
  return std::move(sys);
}

HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys, bool strip_dead) {
  HybridPowerSystem out = sys;
  ProjectionOptions options;
  options.strip_dead_islands = strip_dead;
  project_in_place(out, options);
  return out;
}

}  // namespace hacdcpf
