#include "hacdcpf/model/network_utils.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

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
  ld.name = fl.name.empty() ? ("FlexibleLoad_" + std::to_string(fl.index)) : (fl.name + "_eq");
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
  ld.name = al.name.empty() ? ("AsymmetricLoad_" + std::to_string(al.index)) : (al.name + "_eq");
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

  // When the physical tap is on the LV winding (tap_side == 1), the canonical
  // branch model stores the tap as 1/tap_pu so that the branch equation
  //   V_lv = V_hv / branch.tap
  // correctly represents V_lv = V_hv * tap_pu (boost on LV side).
  // For HV-side tap (tap_side == 0), branch.tap == tap_pu directly.
  // The impedance values (expressed on the winding's own base) must also be
  // scaled by tap_pu² when the tap is on the LV side, matching the
  // CanonicalTapProjection logic in three_phase_nr.cpp.
  const double raw_tap = tap_from_step(tr.tap_pos, tr.tap_neutral, tr.tap_step_percent);
  const double clamped_tap = std::max(1e-6, raw_tap);
  const double canonical_tap = (tr.tap_side == 1) ? (1.0 / clamped_tap) : clamped_tap;
  const double impedance_scale = (tr.tap_side == 1) ? (clamped_tap * clamped_tap) : 1.0;

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
  br.name = tr.name.empty() ? ("Transformer2W_" + std::to_string(tr.index)) : (tr.name + "_eq");
  br.r0_pu = r_pu;
  br.x0_pu = (tr.z0_percent > 0.0) ? (tr.z0_percent / 100.0 * base_mva / std::max(1e-9, tr.sn_mva))
                                    : x_pu;
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

  // Compute the winding tap ratio.  The tap_neutral for Transformer3W uses
  // tap_pos relative to neutral position 0 (consistent with Transformer2W
  // conventions in this code-base).
  const double winding_tap_pu =
      std::max(1e-6, tap_from_step(tr.tap_pos, 0, tr.tap_step_percent));

  // Canonical branch tap assignment for three-winding Kron equivalent:
  //   branch.tap represents  V_to = V_from / branch.tap
  // We assign a per-winding scalar: w[HV]=1, w[MV]=1, w[LV]=1 for the
  // untapped windings; the tapped winding gets w=winding_tap_pu.
  // For a branch FROM winding A TO winding B:
  //   If tap is on the FROM side  →  branch.tap = winding_tap_pu  (raises V at to)
  //   If tap is on the TO   side  →  branch.tap = 1/winding_tap_pu (raises V at to)
  //   If tap is on neither  side  →  branch.tap = 1.0
  //
  // Mapping:  tap_side 0=HV, 1=MV, 2=LV
  // pairs:    0=HV_MV, 1=HV_LV, 2=MV_LV
  //
  // tap_side=0(HV): pair0=t, pair1=t,   pair2=1
  // tap_side=1(MV): pair0=1/t, pair1=1, pair2=t
  // tap_side=2(LV): pair0=1,  pair1=1/t, pair2=1/t
  const double t = winding_tap_pu;
  const double inv_t = 1.0 / t;
  double tap_by_pair[3];
  // Per-winding tap scalars (tau_H, tau_M, tau_L) — used for impedance
  // calculation.  The tapped winding carries tau=t; others carry tau=1.
  double tau_winding[3] = {1.0, 1.0, 1.0};
  if (tr.tap_side == 0) {        // HV tapped
    tap_by_pair[0] = t;
    tap_by_pair[1] = t;
    tap_by_pair[2] = 1.0;
    tau_winding[0] = t;
  } else if (tr.tap_side == 1) { // MV tapped
    tap_by_pair[0] = inv_t;
    tap_by_pair[1] = 1.0;
    tap_by_pair[2] = t;
    tau_winding[1] = t;
  } else {                       // LV tapped (tap_side == 2)
    tap_by_pair[0] = 1.0;
    tap_by_pair[1] = inv_t;
    tap_by_pair[2] = inv_t;
    tau_winding[2] = t;
  }

  // ─────────────────────────────────────────────────────────────────────────
  // Kron-correct impedance computation for three-winding equivalent branches
  //
  // The three-winding Kron equivalent is constructed by:
  //   1. Converting pair short-circuit data to star (T) impedances.
  //   2. Applying per-winding tap ratios to form the full 4-bus admittance
  //      matrix (3 external buses + 1 star node).
  //   3. Kron-eliminating the star node to get a 3×3 Y-bus.
  //
  // For each pair branch (from→to, tap=tau_pair) the contribution to the
  // off-diagonal element Y[from][to] is:
  //   Y_branch[from][to] = -y_branch / conj(tau_pair)
  //
  // The required element from the Kron Y-bus is:
  //   kron_y[from][to] = -(y_from * y_to) / (tau_from * tau_to * Y_ss)
  //
  // where tau_i is the per-winding tap (tau_winding[i]) and Y_ss = Σ y_i.
  // Equating the two and solving for y_branch (all taps are real here):
  //   y_branch = y_from * y_to * tau_pair / (tau_from * tau_to * Y_ss)
  //   z_branch = tau_from * tau_to * Y_ss / (y_from * y_to * tau_pair)
  //
  // Index mapping: 0=HV, 1=MV, 2=LV
  // pair indices:  pair0=(HV,MV)=winding(0,1), pair1=(HV,LV)=(0,2), pair2=(MV,LV)=(1,2)
  // ─────────────────────────────────────────────────────────────────────────

  // Step 1: Compute pair impedances (on system base) for each winding pair.
  const double sn_hv_mv = std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_mv_mva));
  const double sn_hv_lv = std::max(1e-9, std::min(tr.sn_hv_mva, tr.sn_lv_mva));
  const double sn_mv_lv = std::max(1e-9, std::min(tr.sn_mv_mva, tr.sn_lv_mva));

  auto [r_hm, x_hm] = rx_from_vk_vkr(tr.vk_hv_mv_percent, tr.vkr_hv_mv_percent, base_mva, sn_hv_mv);
  auto [r_hl, x_hl] = rx_from_vk_vkr(tr.vk_hv_lv_percent, tr.vkr_hv_lv_percent, base_mva, sn_hv_lv);
  auto [r_ml, x_ml] = rx_from_vk_vkr(tr.vk_mv_lv_percent, tr.vkr_mv_lv_percent, base_mva, sn_mv_lv);

  const Complex Z_hm(r_hm, x_hm);
  const Complex Z_hl(r_hl, x_hl);
  const Complex Z_ml(r_ml, x_ml);

  // Step 2: Convert to star (T-equivalent) impedances.
  const Complex Z_H = 0.5 * (Z_hm + Z_hl - Z_ml);
  const Complex Z_M = 0.5 * (Z_hm + Z_ml - Z_hl);
  const Complex Z_L = 0.5 * (Z_hl + Z_ml - Z_hm);

  // Guard against near-singular star impedances.
  const double kZmin = 1e-12;
  const Complex y_H = (std::abs(Z_H) > kZmin) ? (Complex{1.0, 0.0} / Z_H) : Complex{1.0 / kZmin, 0.0};
  const Complex y_M = (std::abs(Z_M) > kZmin) ? (Complex{1.0, 0.0} / Z_M) : Complex{1.0 / kZmin, 0.0};
  const Complex y_L = (std::abs(Z_L) > kZmin) ? (Complex{1.0, 0.0} / Z_L) : Complex{1.0 / kZmin, 0.0};

  // Step 3: Compute Y_ss (denominator for all pair impedances).
  const Complex Y_ss = y_H + y_M + y_L;

  // Star admittances indexed by winding [0=HV, 1=MV, 2=LV].
  const Complex y_winding[3] = {y_H, y_M, y_L};

  // Winding index pairs for each branch pair.
  const int pair_from_winding[3] = {0, 0, 1};  // HV, HV, MV
  const int pair_to_winding[3]   = {1, 2, 2};  // MV, LV, LV

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

    // Kron-correct pair impedance:
    //   z_branch = tau_from * tau_to * Y_ss / (y_from * y_to * tau_pair)
    // (all taps are real-valued here)
    const double tau_from = tau_winding[fw];
    const double tau_to   = tau_winding[tw];
    const double tau_pair = tap_by_pair[pair_idx];

    const Complex y_numerator = y_winding[fw] * y_winding[tw] * Complex{tau_pair, 0.0};
    const Complex z_denominator = Complex{tau_from * tau_to, 0.0} * Y_ss;

    if (std::abs(y_numerator) < 1e-30) continue;  // degenerate — skip

    const Complex Z_branch = z_denominator / y_numerator;
    const double r_pu = Z_branch.real();
    const double x_pu = Z_branch.imag();

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
    br.shift_deg = (p.from_bus == tr.hv_bus && p.to_bus == tr.mv_bus) ? tr.shift_mv_deg
                  : (p.from_bus == tr.hv_bus && p.to_bus == tr.lv_bus) ? tr.shift_lv_deg
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
  br.name = sw.name.empty() ? ("Switch_" + std::to_string(sw.index)) : (sw.name + "_eq");

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

// ─────────────────────────────────────────────────────────────────────
// Motor → canonical Load
//
// An AsynchronousMotor is a constant-power consumer.  Its rated power
// draw is:  P = sn_mva × cos_phi,  Q = sn_mva × sin(φ).
// The SC subtransient impedance fields (sn_mva, r_sc_pu, x_sub_pu with
// motor_percent = 1) allow the Load-based SC path to model this motor.
// ─────────────────────────────────────────────────────────────────────
void add_equivalent_load_from_motor(const AsynchronousMotor& m,
                                    ACSystem& ac,
                                    int& next_idx) {
  if (!m.in_service) return;
  if (m.bus == 0) return;

  Load ld;
  ld.index = next_idx++;
  ld.bus = m.bus;
  ld.in_service = true;
  ld.name = m.name.empty() ? ("Motor_" + std::to_string(m.index)) : (m.name + "_eq");

  const double cos_phi = std::clamp(m.cos_phi, 0.01, 1.0);
  const double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
  ld.p_mw   = m.sn_mva * cos_phi;
  ld.q_mvar = m.sn_mva * sin_phi;
  ld.model  = LoadModel::ConstantPower;
  ld.p_percent_p = 100.0;
  ld.p_percent_q = 100.0;

  // SC subtransient impedance — convert from ohms to pu on motor base so that
  // the Load motor-fraction SC path (sn_mva, motor_percent, r_sc_pu, x_sub_pu)
  // produces the same admittance as the dedicated Motor path.
  ld.sn_mva       = m.sn_mva;
  ld.motor_percent = 1.0;
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

// ─────────────────────────────────────────────────────────────────────
// VirtualPowerPlant → canonical StaticGenerator
//
// The VPP pcc_bus receives the net injection from the fleet.
// ─────────────────────────────────────────────────────────────────────
void add_equivalent_static_gen_from_vpp(const VirtualPowerPlant& vpp,
                                        ACSystem& ac,
                                        int& next_idx) {
  if (!vpp.in_service) return;
  if (vpp.pcc_bus == 0) return;

  StaticGenerator sg;
  sg.index      = next_idx++;
  sg.bus        = vpp.pcc_bus;
  sg.in_service = true;
  sg.name       = vpp.name.empty() ? ("VPP_" + std::to_string(vpp.index)) : (vpp.name + "_eq");
  sg.sgen_type  = SgenType::Other;

  sg.p_mw      = vpp.p_output_mw;
  sg.q_mvar    = vpp.q_output_mvar;
  sg.pmax_mw   = vpp.pmax_mw;
  sg.pmin_mw   = vpp.pmin_mw;
  sg.controllable = true;

  // Regulation capability → reactive limits (symmetric if not given)
  sg.qmax_mvar = (vpp.q_output_mvar >= 0.0) ? vpp.q_output_mvar : 0.0;
  sg.qmin_mvar = (vpp.q_output_mvar < 0.0)  ? vpp.q_output_mvar : 0.0;

  ac.static_generators.push_back(sg);
}

// ─────────────────────────────────────────────────────────────────────
// Microgrid → canonical StaticGenerator at PCC bus
//
// In grid-connected mode, the microgrid net exchange appears as a
// StaticGenerator at the point of common coupling.  Positive p_mw
// means the microgrid is exporting (injecting into the main grid).
// Islanded microgrids are skipped (they decouple from the main grid).
// ─────────────────────────────────────────────────────────────────────
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
  sg.name       = mg.name.empty() ? ("Microgrid_" + std::to_string(mg.index)) : (mg.name + "_eq");
  sg.sgen_type  = SgenType::Other;

  // p_exchange_mw > 0 means export from microgrid → injection into main grid
  sg.p_mw      = mg.p_exchange_mw;
  sg.q_mvar    = 0.0;   // reactive exchange not tracked in Microgrid model
  // Limits: export is positive (pmax) and import negative (pmin)
  sg.pmax_mw   = mg.p_export_max_mw > 1e-9 ? mg.p_export_max_mw : mg.p_exchange_max_mw;
  sg.pmin_mw   = mg.p_import_max_mw > 1e-9 ? -mg.p_import_max_mw : mg.p_exchange_min_mw;
  sg.controllable = true;

  ac.static_generators.push_back(sg);
}

// ─────────────────────────────────────────────────────────────────────
// MobileStorage → canonical AC Storage at current bus
//
// When the unit is stationary and connected (bus != 0), it behaves
// identically to a fixed Storage unit.
// ─────────────────────────────────────────────────────────────────────
void add_equivalent_storage_from_mobile_storage(const MobileStorage& ms,
                                                ACSystem& ac,
                                                int& next_idx) {
  if (!ms.in_service) return;
  if (ms.bus == 0) return;
  // Only project if at rest — mobile/transit units are not yet connected.
  if (ms.status == MobileStorageStatus::InTransit) return;

  Storage st;
  st.index      = next_idx++;
  st.bus        = ms.bus;
  st.in_service = true;
  st.name       = ms.name.empty() ? ("MobileStorage_" + std::to_string(ms.index)) : (ms.name + "_eq");

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
    br.name = line.name.empty() ? ("ThreePhaseLine_" + std::to_string(line.index)) : (line.name + "_eq");
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
    out.ac.transformers_2w.push_back(eq);
  }

  int next_ld = 1;
  for (const auto& ld : tp.loads) {
    Load eq;
    eq.index = next_ld++;
    eq.bus = ld.bus;
    eq.in_service = ld.in_service;
    eq.name = ld.name.empty() ? ("ThreePhaseLoad_" + std::to_string(ld.index)) : (ld.name + "_eq");
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
    out.ac.loads.push_back(eq);
  }

  int next_gen = 1;
  for (const auto& g : tp.generators) {
    Generator eq;
    eq.index = next_gen++;
    eq.bus = g.bus;
    eq.in_service = g.in_service;
    eq.is_slack = g.is_slack;
    eq.name = g.name.empty() ? ("ThreePhaseGen_" + std::to_string(g.index)) : (g.name + "_eq");
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
    out.ac.generators.push_back(eq);
  }

  int next_eg = 1;
  for (const auto& eg : tp.external_grids) {
    ExternalGrid eq;
    eq.index = next_eg++;
    eq.bus = eg.bus;
    eq.in_service = eg.in_service;
    eq.name = eg.name.empty() ? ("ThreePhaseExternalGrid_" + std::to_string(eg.index)) : (eg.name + "_eq");
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
    out.ac.external_grids.push_back(eq);
  }
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════
// Count queries
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
// Aggregation queries
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
// Validation
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
// Summary printing
// ═══════════════════════════════════════════════════════════════════════
void print_summary(const HybridPowerSystem& sys, std::ostream& os) {
  os << "╔══════════════════════════════════════════════════════════════╗\n";
  os << "║  Power System: " << sys.name << "\n";
  os << "║  Base MVA: " << sys.base_mva << "\n";
  os << "╠══════════════════════════════════════════════════════════════╣\n";
  os << "║  AC buses:        " << n_ac_buses(sys) << "\n";
  os << "║  AC branches:     " << n_ac_branches(sys) << "\n";
  os << "║  DC buses:        " << n_dc_buses(sys) << "\n";
  os << "║  DC branches:     " << n_dc_branches(sys) << "\n";
  os << "║  Generators:      " << n_generators(sys) << "\n";
  os << "║  Loads:           " << n_loads(sys) << "\n";
  os << "║  Shunts:          " << n_shunts(sys) << "\n";
  os << "║  Storage:         " << n_storage(sys) << "\n";
  os << "║  Renewables:      " << n_renewable_gens(sys) << "\n";
  os << "║  VSC converters:  " << n_converters(sys) << "\n";
  os << "╠══════════════════════════════════════════════════════════════╣\n";
  os << "║  Total gen capacity:   " << total_gen_capacity_mw(sys)
     << " MW\n";
  os << "║  Total load (P):       " << total_load_p_mw(sys) << " MW\n";
  os << "║  Total load (Q):       " << total_load_q_mvar(sys)
     << " Mvar\n";

  double re_cap = total_renewable_capacity_mw(sys);
  double st_cap = total_storage_capacity_mwh(sys);
  if (re_cap > 0.0) {
    os << "║  Renewable capacity:   " << re_cap << " MW\n";
  }
  if (st_cap > 0.0) {
    os << "║  Storage energy:       " << st_cap << " MWh\n";
    os << "║  Storage power:        " << total_storage_power_mw(sys)
       << " MW\n";
  }

  double inertia = total_system_inertia_mws(sys);
  if (inertia > 0.0) {
    os << "║  System inertia:       " << inertia << " MW·s\n";
  }

  os << "╚══════════════════════════════════════════════════════════════╝\n";
}

std::string summary_string(const HybridPowerSystem& sys) {
  std::ostringstream oss;
  print_summary(sys, oss);
  return oss.str();
}

// ═══════════════════════════════════════════════════════════════════════
// Topology helpers
// ═══════════════════════════════════════════════════════════════════════
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

// ─────────────────────────────────────────────────────────────────────
// Union-Find for bus merging
// ─────────────────────────────────────────────────────────────────────
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

// ─────────────────────────────────────────────────────────────────────
// merge_zero_impedance_buses
// ─────────────────────────────────────────────────────────────────────
void merge_zero_impedance_buses(HybridPowerSystem& sys) {
  auto& buses = sys.ac.buses;
  auto& branches = sys.ac.branches;
  const int n = static_cast<int>(buses.size());
  if (n <= 1) return;

  // Build position lookup: bus.index → 0-based position
  std::unordered_map<int, int> idx_to_pos;
  idx_to_pos.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    idx_to_pos[buses[static_cast<size_t>(i)].index] = i;
  }

  // Phase 1: identify zero-impedance branches and union buses
  UnionFind uf(n);
  bool any_merged = false;
  for (const auto& br : branches) {
    if (!br.in_service) continue;
    // A true zero-impedance element (CB/switch) has both R and X tiny
    // AND negligible charging susceptance.  Short lines may have small
    // R and X but non-zero B — these must NOT be merged because merging
    // discards their charging and distorts the admittance model.
    if (std::abs(br.r_pu) >= kBusMergeZThreshold ||
        std::abs(br.x_pu) >= kBusMergeZThreshold) continue;
    if (std::abs(br.b_pu) > 1e-6) continue;  // non-trivial charging → real line

    auto it_f = idx_to_pos.find(br.from_bus);
    auto it_t = idx_to_pos.find(br.to_bus);
    if (it_f == idx_to_pos.end() || it_t == idx_to_pos.end()) continue;

    if (uf.unite(it_f->second, it_t->second)) {
      any_merged = true;
    }
  }

  if (!any_merged) return;

  // Phase 2: build equivalence classes (use std::map for deterministic
  // iteration order — groups are visited in ascending root_pos, guaranteeing
  // stable bus index assignment across runs and platforms)
  std::map<int, std::vector<int>> group_map;  // root_pos → positions
  for (int i = 0; i < n; ++i) {
    group_map[uf.find(i)].push_back(i);
  }

  // Pick representative per group: highest bus-type priority, then lowest index
  // Build old-position → new-position mapping
  BusMergeMap merge_map;
  merge_map.n_original = n;

  // new_pos: representative position → new sequential position
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
    for (int m : members) {
      old_pos_to_new[static_cast<size_t>(m)] = new_pos;
      group_ext.push_back(buses[static_cast<size_t>(m)].index);
    }
    merge_map.groups.push_back(std::move(group_ext));
    merge_map.int_to_ext.push_back(buses[static_cast<size_t>(rep)].index);

    merged_buses.push_back(std::move(merged));
  }

  merge_map.n_merged = static_cast<int>(merged_buses.size());

  // Build ext_to_int: original external bus index → new internal position
  for (int i = 0; i < n; ++i) {
    const int ext = buses[static_cast<size_t>(i)].index;
    merge_map.ext_to_int[ext] = old_pos_to_new[static_cast<size_t>(i)];
    merge_map.ext_to_orig_pos[ext] = i;
  }

  // Helper: reindex an AC bus reference (1-based external → new 1-based)
  auto remap_ac = [&](int old_bus) -> int {
    auto it = merge_map.ext_to_int.find(old_bus);
    if (it != merge_map.ext_to_int.end()) return it->second + 1;
    return old_bus;  // unknown bus unchanged (safety)
  };

  // Phase 3: replace buses
  buses = std::move(merged_buses);

  // Phase 4: reindex branches and remove self-loops
  std::vector<ACBranch> kept_branches;
  kept_branches.reserve(branches.size());
  for (auto& br : branches) {
    br.from_bus = remap_ac(br.from_bus);
    br.to_bus = remap_ac(br.to_bus);
    // Remove self-loops (branches within the same merged bus)
    if (br.from_bus == br.to_bus) continue;
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
  for (auto& cs : sys.ac.charging_stations) cs.bus = remap_ac(cs.bus);
  for (auto& mot : sys.ac.motors) mot.bus = remap_ac(mot.bus);

  // Transformers: only reindex the original rich types (not the branches
  // expanded from them — those were already handled above).
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

  // Mobile storage: bus might be AC or DC — only remap if it could be AC.
  // Since MobileStorage lives at the HybridPowerSystem level and typically
  // references AC buses, remap both bus and target_bus.
  for (auto& ms : sys.mobile_storage) {
    ms.bus = remap_ac(ms.bus);
    ms.target_bus = remap_ac(ms.target_bus);
  }

  sys.bus_merge_map = std::move(merge_map);
}

// ─────────────────────────────────────────────────────────────────────
// unproject_bus_vector
// ─────────────────────────────────────────────────────────────────────
std::vector<double> unproject_bus_vector(
    const std::vector<double>& merged,
    const BusMergeMap& map) {
  std::vector<double> out(static_cast<size_t>(map.n_original), 0.0);
  for (const auto& [ext_bus, int_pos] : map.ext_to_int) {
    if (int_pos < 0 || int_pos >= static_cast<int>(merged.size())) continue;
    int orig_pos = -1;
    auto it_pos = map.ext_to_orig_pos.find(ext_bus);
    if (it_pos != map.ext_to_orig_pos.end()) {
      orig_pos = it_pos->second;
    } else {
      // Backward-compatibility fallback for legacy maps that did not store
      // explicit external-index → original-position mapping.
      orig_pos = ext_bus - 1;
    }
    if (orig_pos < 0 || orig_pos >= map.n_original) continue;
    out[static_cast<size_t>(orig_pos)] = merged[static_cast<size_t>(int_pos)];
  }
  return out;
}

// Internal helper: project a mutable HybridPowerSystem in place.
static void project_in_place(HybridPowerSystem& out) {
  if (out.base_mva <= 1e-9) {
    out.base_mva = 100.0;
  }
  if (out.ac.base_mva <= 1e-9) {
    out.ac.base_mva = out.base_mva;
  }
  if (out.dc.base_mva <= 1e-9) {
    out.dc.base_mva = out.base_mva;
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
    add_equivalent_load_from_flexible(fl, out.ac, next_ld);
  }
  for (const auto& al : out.ac.asymmetric_loads) {
    add_equivalent_load_from_asymmetric(al, out.ac, next_ld);
  }
  for (const auto& m : out.ac.motors) {
    add_equivalent_load_from_motor(m, out.ac, next_ld);
  }

  // VirtualPowerPlants → StaticGenerator at aggregation bus
  int next_sg = next_index_of(out.ac.static_generators);
  for (const auto& vpp : out.vpps) {
    add_equivalent_static_gen_from_vpp(vpp, out.ac, next_sg);
  }
  for (const auto& mg : out.microgrids) {
    add_equivalent_static_gen_from_microgrid(mg, out.ac, next_sg);
  }

  // MobileStorage → canonical AC Storage at current bus
  int next_st = next_index_of(out.ac.storage);
  for (const auto& ms : out.mobile_storage) {
    add_equivalent_storage_from_mobile_storage(ms, out.ac, next_st);
  }

  // Track branch expansion provenance for Transformer2W/3W and Switch.
  // This lets callers map solver branch-flow indices back to original elements.
  BranchExpandMap bmap;
  int next_br = next_index_of(out.ac.branches);

  for (const auto& tr : out.ac.transformers_2w) {
    if (tr.source_branch_idx > 0) {
      // This Transformer2W was auto-extracted from a MATPOWER branch
      // that is still in the branch list.  The original branch retains
      // exact pi-model data (r, x, b, tap) — do NOT overwrite with
      // round-tripped values.  Just record the provenance mapping.
      bmap.entries.push_back({tr.source_branch_idx, BranchOriginType::Transformer2W, tr.index, 0});
    } else {
      // Standalone Transformer2W (e.g. from Excel/JSON import) — create
      // equivalent branch as before.
      const int br_before = next_br;
      add_equivalent_branch_from_transformer2w(tr, out.ac, out.base_mva, next_br);
      if (next_br > br_before) {
        bmap.entries.push_back({br_before, BranchOriginType::Transformer2W, tr.index, 0});
      }
    }
  }
  for (const auto& tr : out.ac.transformers_3w) {
    const int br_before = next_br;
    add_equivalent_branches_from_transformer3w(tr, out.ac, out.base_mva, next_br);
    // Record each pair branch in HV-MV / HV-LV / MV-LV order
    for (int pair = 0; br_before + pair < next_br; ++pair) {
      bmap.entries.push_back({br_before + pair, BranchOriginType::Transformer3W, tr.index, pair});
    }
  }
  for (const auto& sw : out.ac.switches) {
    const int br_before = next_br;
    add_equivalent_branch_from_switch(sw, out.ac, out.base_mva, next_br);
    if (next_br > br_before) {
      bmap.entries.push_back({br_before, BranchOriginType::Switch, sw.index, 0});
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

  // Merge buses connected by zero-impedance branches created from switch
  // expansion.  Only perform merging when the system actually contains
  // switches; for pure MATPOWER imports the near-zero-Z branches are real
  // short lines and must not be merged.
  if (!out.ac.switches.empty()) {
    merge_zero_impedance_buses(out);
  }

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

HybridPowerSystem project_to_canonical_models(const HybridPowerSystem& sys) {
  HybridPowerSystem out = sys;
  project_in_place(out);
  return out;
}

HybridPowerSystem project_to_canonical_models(HybridPowerSystem&& sys) {
  project_in_place(sys);
  return std::move(sys);
}

}  // namespace hacdcpf
