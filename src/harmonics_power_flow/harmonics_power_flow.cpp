#define _USE_MATH_DEFINES  // M_PI on strict-conformance toolchains (MSVC / UCRT)

// Harmonic Power Flow (HPF) for hybrid AC/DC distribution networks.
// ================================================================
// Frequency-domain "harmonic penetration" form of the unified AC/DC harmonic
// power flow.  See include/hacdcpf/analysis/harmonics_power_flow.hpp and the
// docs/harmonic_*.md derivation for the modelling rationale.
//
// For each harmonic order h the AC network reduces to a linear nodal equation
//      Y_ac(h) . V_ac(h) = I_ac(h)
// and for each ripple order r the DC network reduces to
//      Y_dc(r) . V_dc(r) = I_dc(r).
// Resources (nonlinear loads, CIDERs, NICs) appear as Norton current injections
// with optional internal output admittance stamped into Y.  The two-port NIC
// bridges the AC and DC sub-systems through a consistent fundamental operating
// point taken from the base power flow.

#include "hacdcpf/analysis/harmonics_power_flow.hpp"

#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/projection/result_attribution.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <complex>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/power_flow/three_phase.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hacdcpf::harmonics {

namespace {

using Cx    = std::complex<double>;
using SpMat = Eigen::SparseMatrix<Cx>;
using Trip  = Eigen::Triplet<Cx>;

constexpr double kDeg2Rad = M_PI / 180.0;

// ───────────────────────────────────────────────────────────────────────────
// id → position maps (robust to non-contiguous bus identifiers)
// ───────────────────────────────────────────────────────────────────────────
template <typename BusVec>
std::unordered_map<int, int> build_id_map(const BusVec& buses) {
  std::unordered_map<int, int> m;
  m.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i)
    m[buses[i].index] = i;
  return m;
}

// Frequency-dependent series resistance (skin effect).  R(h) per HPFOptions.
// `h` is a (possibly fractional) frequency multiple of the fundamental.
double skin_r(double r1, double h, const HPFOptions& opt) {
  if (h <= 1.0 || r1 == 0.0 || opt.skin_effect == SkinEffectModel::None) return r1;
  const double sh = std::sqrt(h);
  if (opt.skin_effect == SkinEffectModel::SqrtOrder) return r1 * sh;
  return r1 * (1.0 + opt.skin_coefficient * sh);  // ProportionalSqrt
}

// Series + charging stamp for an AC pi-branch at harmonic frequency h.
struct BranchStamp {
  Cx yff, yft, ytf, ytt;
};

BranchStamp ac_branch_stamp(double r, double x, double b, double tap,
                            double shift_deg, double h) {
  const double hh = h;
  Cx z_series(r, hh * x);
  Cx ys = (std::abs(z_series) > 1e-12) ? (Cx(1.0, 0.0) / z_series) : Cx(0.0, 0.0);
  Cx ych(0.0, hh * b);  // total line charging at order h
  double tapm = (tap > 1e-9) ? tap : 1.0;
  Cx t = std::polar(tapm, shift_deg * kDeg2Rad);
  BranchStamp s;
  s.yff = (ys + 0.5 * ych) / (tapm * tapm);
  s.yft = -ys / std::conj(t);
  s.ytf = -ys / t;
  s.ytt = (ys + 0.5 * ych);
  return s;
}

// ───────────────────────────────────────────────────────────────────────────
// Operating point: fundamental AC voltages and steady DC voltages.
// Mirrors the GUI/PF convention: pf.vm[i] <-> sys.ac.buses[i] (position),
// pf.va[i] in radians, pf.vdc[i] <-> sys.dc.buses[i]; fall back to stored fields.
// ───────────────────────────────────────────────────────────────────────────
struct OperatingPoint {
  std::vector<Cx>     vac1;   ///< per AC bus position
  std::vector<double> vdc0;   ///< per DC bus position
  bool                pf_converged{false};
  std::unordered_map<int, VSCTransfer> vsc_by_index;
};

OperatingPoint extract_operating_point(const HybridPowerSystem& sys,
                                       const HPFOptions& opt) {
  OperatingPoint op;
  op.vac1.resize(sys.ac.buses.size(), Cx(1.0, 0.0));
  op.vdc0.resize(sys.dc.buses.size(), 1.0);

  PowerFlowResult pf;
  bool have_pf = false;
  if (opt.run_base_power_flow) {
    try {
      pf = hacdcpf::solve_power_flow(sys, opt.base_pf_options);
      have_pf = pf.converged;
      op.pf_converged = pf.converged;
    } catch (...) {
      have_pf = false;
    }
  }

  for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
    const auto& b = sys.ac.buses[i];
    double vm = (have_pf && i < pf.vm.size()) ? pf.vm[i] : b.vm_pu;
    double va = (have_pf && i < pf.va.size()) ? pf.va[i] : b.va_deg * kDeg2Rad;
    if (vm <= 1e-9) vm = 1.0;  // de-energised bus -> nominal reference for THD
    op.vac1[i] = std::polar(vm, va);
  }
  for (size_t k = 0; k < sys.dc.buses.size(); ++k) {
    const auto& b = sys.dc.buses[k];
    double vdc = (have_pf && k < pf.vdc.size()) ? pf.vdc[k] : b.vm_pu;
    if (std::abs(vdc) <= 1e-9) vdc = 1.0;
    op.vdc0[k] = vdc;
  }
  if (have_pf) {
    for (const auto& vt : pf.vsc_transfers) op.vsc_by_index[vt.index] = vt;
  }
  return op;
}

// ───────────────────────────────────────────────────────────────────────────
// AC harmonic admittance matrix at order h (over all AC bus positions).
// ───────────────────────────────────────────────────────────────────────────
SpMat build_ac_ybus(const HybridPowerSystem& sys, const OperatingPoint& op,
                    const std::unordered_map<int, int>& id2pos, double h,
                    const HPFOptions& opt) {
  const int n = static_cast<int>(sys.ac.buses.size());
  const double base = sys.base_mva > 0 ? sys.base_mva : 100.0;
  std::vector<Trip> trips;
  trips.reserve(sys.ac.branches.size() * 4 + n * 2);

  auto pos = [&](int id) -> int {
    auto it = id2pos.find(id);
    return it == id2pos.end() ? -1 : it->second;
  };

  // Branches (lines + transformers represented as ACBranch).
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    int f = pos(br.from_bus), t = pos(br.to_bus);
    if (f < 0 || t < 0) continue;
    BranchStamp s = ac_branch_stamp(skin_r(br.r_pu, h, opt), br.x_pu, br.b_pu,
                                    br.tap, br.shift_deg, h);
    trips.emplace_back(f, f, s.yff);
    trips.emplace_back(f, t, s.yft);
    trips.emplace_back(t, f, s.ytf);
    trips.emplace_back(t, t, s.ytt);
  }

  // Two-winding transformers (vk%, vkr% → series r + jx referred to system base).
  for (const auto& tf : sys.ac.transformers_2w) {
    if (!tf.in_service) continue;
    int f = pos(tf.hv_bus), t = pos(tf.lv_bus);
    if (f < 0 || t < 0 || tf.sn_mva <= 0) continue;
    double z_pu = (tf.vk_percent / 100.0) * (base / tf.sn_mva);
    double r_pu = (tf.vkr_percent / 100.0) * (base / tf.sn_mva);
    double x_pu = std::sqrt(std::max(z_pu * z_pu - r_pu * r_pu, 0.0));
    BranchStamp s = ac_branch_stamp(skin_r(r_pu, h, opt), x_pu, 0.0, 1.0, 0.0, h);
    trips.emplace_back(f, f, s.yff);
    trips.emplace_back(f, t, s.yft);
    trips.emplace_back(t, f, s.ytf);
    trips.emplace_back(t, t, s.ytt);
  }

  // Fixed bus shunts (ACBus gs/bs) and Shunt components: y(h) = g + j*h*b.
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.ac.buses[i];
    double g = b.gs_mw / base, bb = b.bs_mvar / base;
    if (g != 0.0 || bb != 0.0)
      trips.emplace_back(i, i, Cx(g, h * bb));
  }
  for (const auto& sh : sys.ac.shunts) {
    if (!sh.in_service) continue;
    int i = pos(sh.bus);
    if (i < 0) continue;
    double g = sh.gs_mw / base, bb = sh.bs_mvar / base;
    trips.emplace_back(i, i, Cx(g, h * bb));
  }

  // Load impedance model (parallel R // jX from fundamental P, Q).
  if (opt.include_load_impedance) {
    for (const auto& ld : sys.ac.loads) {
      if (!ld.in_service) continue;
      int i = pos(ld.bus);
      if (i < 0) continue;
      double p = ld.p_mw * ld.scaling / base;
      double q = ld.q_mvar * ld.scaling / base;
      double vm = std::abs(op.vac1[i]);
      if (vm < 1e-6) vm = 1.0;
      double v2 = vm * vm;
      // y_load(h) = P/V^2  +  1/(j h X1),  X1 = V^2/Q  (inductive Q>0).
      Cx yl(p / v2, 0.0);
      if (std::abs(q) > 1e-12)
        yl += Cx(0.0, -q / (static_cast<double>(h) * v2));
      trips.emplace_back(i, i, yl);
    }
  }

  // AC voltage sources grounded by their internal (sub-transient) impedance:
  //   y_src(h) = 1 / (r + j h x'').  A harmonic short behind Z_int.
  std::vector<Cx> source_y(n, Cx(0.0, 0.0));
  auto add_source = [&](int i, double r, double xpp) {
    if (i < 0) return;
    double x = (xpp > 1e-9) ? xpp : opt.default_source_xpp_pu;
    Cx z(r, static_cast<double>(h) * x);
    if (std::abs(z) > 1e-12) source_y[i] += Cx(1.0, 0.0) / z;
  };
  for (const auto& g : sys.ac.generators) {
    if (!g.in_service) continue;
    add_source(pos(g.bus), g.ra_pu, g.xdpp_pu);
  }
  for (const auto& eg : sys.ac.external_grids) {
    if (!eg.in_service) continue;
    int i = pos(eg.bus);
    if (i < 0) continue;
    double r = eg.r_pu, x = eg.x_pu;
    if (x <= 1e-9 && eg.s_sc_max_mva > 0) {
      double zsc = base / eg.s_sc_max_mva;  // |Z| pu
      double xr = eg.x_r > 0 ? eg.x_r : 10.0;
      x = zsc / std::sqrt(1.0 + 1.0 / (xr * xr));
      r = x / xr;
    }
    add_source(i, r, x);
  }
  // Slack / PV buses without an explicit machine still anchor the harmonic net.
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.ac.buses[i];
    if ((b.bus_type == BusType::SLACK || b.bus_type == BusType::PV) &&
        std::abs(source_y[i]) < 1e-12)
      add_source(i, 0.0, opt.default_source_xpp_pu);
  }
  for (int i = 0; i < n; ++i) {
    // Strongly ground out-of-service / isolated buses.
    if (!sys.ac.buses[i].in_service || sys.ac.buses[i].bus_type == BusType::ISOLATED)
      source_y[i] += Cx(1.0 / std::max(opt.min_shunt_pu, 1e-12), 0.0);
    // Tiny stray leakage keeps every node connected to ground (conditioning).
    source_y[i] += Cx(opt.min_shunt_pu, 0.0);
    trips.emplace_back(i, i, source_y[i]);
  }

  SpMat Y(n, n);
  Y.setFromTriplets(trips.begin(), trips.end());
  Y.makeCompressed();
  return Y;
}

// ───────────────────────────────────────────────────────────────────────────
// DC ripple admittance matrix at order r.
//   Branch: Z(r) = r_pu + j*r*x_pu (x from opt.dc_ripple_model.branch_x_pu)
//   Bus cap: Y(r) = j*r*b_pu       (b from opt.dc_ripple_model.bus_b_pu)
// ───────────────────────────────────────────────────────────────────────────
SpMat build_dc_ybus(const DCSystem& dc,
                    const std::unordered_map<int, int>& id2pos,
                    const std::vector<int>& nic_dc_pos, int order,
                    const HPFOptions& opt) {
  const int n = static_cast<int>(dc.buses.size());
  std::vector<Trip> trips;
  trips.reserve(dc.branches.size() * 4 + n * 2);
  auto pos = [&](int id) -> int {
    auto it = id2pos.find(id);
    return it == id2pos.end() ? -1 : it->second;
  };

  for (const auto& br : dc.branches) {
    if (!br.in_service) continue;
    int f = pos(br.from_bus), t = pos(br.to_bus);
    if (f < 0 || t < 0) continue;
    double x_pu = 0.0;
    auto xit = opt.dc_ripple_model.branch_x_pu.find(br.index);
    if (xit != opt.dc_ripple_model.branch_x_pu.end()) x_pu = xit->second;
    Cx z(br.r_pu, static_cast<double>(order) * x_pu);
    if (std::abs(z) < 1e-12) z = Cx(1e-6, 0.0);
    Cx y = Cx(1.0, 0.0) / z;
    trips.emplace_back(f, f, y);
    trips.emplace_back(f, t, -y);
    trips.emplace_back(t, f, -y);
    trips.emplace_back(t, t, y);
  }

  // DC voltage-forming nodes ground the ripple network (DC_V buses + NIC DC ports).
  std::vector<Cx> gy(n, Cx(0.0, 0.0));
  double zsrc = std::max(opt.dc_source_impedance_pu, 1e-9);
  for (int i = 0; i < n; ++i) {
    if (dc.buses[i].bus_type == DCBusType::DC_V)
      gy[i] += Cx(1.0 / zsrc, 0.0);
  }
  for (int p : nic_dc_pos)
    if (p >= 0 && p < n) gy[p] += Cx(1.0 / zsrc, 0.0);

  for (int i = 0; i < n; ++i) {
    auto bit = opt.dc_ripple_model.bus_b_pu.find(dc.buses[i].index);
    if (bit != opt.dc_ripple_model.bus_b_pu.end())
      gy[i] += Cx(0.0, static_cast<double>(order) * bit->second);
    if (!dc.buses[i].in_service ||
        dc.buses[i].bus_type == DCBusType::DC_ISOLATED)
      gy[i] += Cx(1.0 / std::max(opt.min_shunt_pu, 1e-12), 0.0);
    gy[i] += Cx(opt.min_shunt_pu, 0.0);
    trips.emplace_back(i, i, gy[i]);
  }

  SpMat Y(n, n);
  Y.setFromTriplets(trips.begin(), trips.end());
  Y.makeCompressed();
  return Y;
}

double thd_from_orders(const std::map<int, Cx>& v_by_order, int fundamental,
                       double v_fund) {
  if (v_fund < 1e-12) return 0.0;
  double acc = 0.0;
  for (const auto& [ord, v] : v_by_order)
    if (ord != fundamental) acc += std::norm(v);  // |v|^2
  return std::sqrt(acc) / v_fund * 100.0;
}

}  // namespace

// ───────────────────────────────────────────────────────────────────────────
// Default spectra
// ───────────────────────────────────────────────────────────────────────────
HarmonicSpectrum default_six_pulse_ac_spectrum() {
  // Characteristic six-pulse converter harmonics (≈ 1/h magnitude law).
  return {
      {5, 20.0, 0.0},  {7, 14.3, 0.0},  {11, 9.1, 0.0}, {13, 7.7, 0.0},
      {17, 5.9, 0.0},  {19, 5.3, 0.0},  {23, 4.3, 0.0}, {25, 4.0, 0.0},
  };
}

HarmonicSpectrum default_dc_ripple_spectrum() {
  // Characteristic six-pulse DC-side ripple (dominant 6th).
  return {{6, 4.5, 0.0}, {12, 2.0, 0.0}, {18, 1.2, 0.0}, {24, 0.8, 0.0}};
}

// ───────────────────────────────────────────────────────────────────────────
// Result summary
// ───────────────────────────────────────────────────────────────────────────
std::string HPFResult::summary() const {
  std::ostringstream os;
  os << "Harmonic power flow: " << (ok ? "OK" : "FAILED");
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  AC orders solved: " << ac_orders.size()
     << ", DC ripple orders solved: " << dc_orders.size();
  os << "\n  Max AC THD: " << max_ac_thd_pct << " % at bus " << max_ac_thd_bus;
  if (!dc_bus_results.empty())
    os << "\n  Max DC THD: " << max_dc_thd_pct << " % at bus " << max_dc_thd_bus;
  return os.str();
}

// ───────────────────────────────────────────────────────────────────────────
// Main solver
// ───────────────────────────────────────────────────────────────────────────
HPFResult solve_harmonic_power_flow(const HybridPowerSystem& rich_sys,
                                    const HarmonicStudyInputs& rich_inputs,
                                    const HPFOptions& opt) {
  const auto projection_bundle =
      projection::RichToCanonicalOperator::apply(rich_sys);
  const auto& sys = projection_bundle.canonical;
  HarmonicStudyInputs canonical_inputs = rich_inputs;
  if (sys.bus_merge_map) {
    const auto& bus_map = *sys.bus_merge_map;
    const auto canonical_ac_bus = [&](int rich_bus) {
      const auto it = bus_map.ext_to_int.find(rich_bus);
      return it == bus_map.ext_to_int.end() ? rich_bus : it->second + 1;
    };
    for (auto& source : canonical_inputs.sources) {
      if (!source.is_dc) source.bus = canonical_ac_bus(source.bus);
    }
    for (auto& nic : canonical_inputs.nics) {
      nic.bus_ac = canonical_ac_bus(nic.bus_ac);
    }
  }
  const auto& inputs = canonical_inputs;
  HPFResult res;
  const double base = sys.base_mva > 0 ? sys.base_mva : 100.0;
  const auto ac_id2pos = build_id_map(sys.ac.buses);
  const auto dc_id2pos = build_id_map(sys.dc.buses);
  const int n_ac = static_cast<int>(sys.ac.buses.size());
  const int n_dc = static_cast<int>(sys.dc.buses.size());

  if (n_ac == 0 && n_dc == 0) {
    res.message = "system has no buses";
    return res;
  }

  OperatingPoint op = extract_operating_point(sys, opt);
  res.base_pf_converged = op.pf_converged;

  auto ac_pos = [&](int id) -> int {
    auto it = ac_id2pos.find(id);
    return it == ac_id2pos.end() ? -1 : it->second;
  };
  auto dc_pos = [&](int id) -> int {
    auto it = dc_id2pos.find(id);
    return it == dc_id2pos.end() ? -1 : it->second;
  };

  // ── Assemble the effective NIC list (explicit overrides + auto from VSCs) ──
  std::vector<HarmonicNIC> nics = inputs.nics;
  std::vector<int> covered_vsc;
  for (const auto& nic : nics)
    if (nic.vsc_index >= 0) covered_vsc.push_back(nic.vsc_index);
  if (opt.auto_nic_from_vscs) {
    for (const auto& v : sys.vsc_converters) {
      if (!v.in_service) continue;
      if (std::find(covered_vsc.begin(), covered_vsc.end(), v.index) != covered_vsc.end())
        continue;
      HarmonicNIC nic;
      nic.vsc_index = v.index;
      nic.bus_ac = v.bus_ac;
      nic.bus_dc = v.bus_dc;
      nic.name = v.name.empty() ? ("VSC" + std::to_string(v.index)) : v.name;
      // Typical NIC: AC port grid-following, DC port grid-forming when the
      // converter regulates the DC-link voltage (VDC modes / grid-forming flag).
      // A pure PQ converter does not anchor the DC ripple network. Uses the
      // shared control-role resolver so all DC-voltage-forming modes (VDC_Q,
      // VDC_VAC, DC_V_DROOP_AC_V) are treated consistently with PF/OPF/transient.
      const bool dc_forms_voltage =
          resolve_device_control_role(v).provides_dc_v_reference;
      nic.ac_port = PortBehavior::GridFollowing;
      nic.dc_port = dc_forms_voltage ? PortBehavior::GridForming
                                     : PortBehavior::GridFollowing;
      nics.push_back(nic);
    }
  }

  // Resolve each NIC fundamental operating point (I_ac1, I_dc0) and fill defaults.
  struct NICOp {
    HarmonicNIC cfg;
    int ac_p{-1}, dc_p{-1};
    double i_ac1_mag{0.0}, i_ac1_ph{0.0};
    double i_dc0{0.0};
  };
  std::vector<NICOp> nic_ops;
  std::vector<int> nic_dc_positions;
  bool used_lossless_nic_guess = false;  // P_dc = -P_ac assumed (no converter loss)
  for (auto& nic : nics) {
    NICOp no;
    no.cfg = nic;
    if (nic.vsc_index >= 0 && nic.vsc_index < (int)sys.vsc_converters.size()) {
      // bus_ac/bus_dc may already be set; otherwise read from the converter.
    }
    // If a VSC index is given, prefer its declared AC/DC buses.
    for (const auto& v : sys.vsc_converters) {
      if (v.index == nic.vsc_index) {
        no.cfg.bus_ac = v.bus_ac;
        no.cfg.bus_dc = v.bus_dc;
        break;
      }
    }
    no.ac_p = ac_pos(no.cfg.bus_ac);
    no.dc_p = dc_pos(no.cfg.bus_dc);

    // Fundamental operating point: prefer base-PF VSC transfer, else setpoints.
    double p_ac = no.cfg.s_ac_p_mw, q_ac = no.cfg.s_ac_q_mvar, p_dc = no.cfg.p_dc_mw;
    auto itv = op.vsc_by_index.find(no.cfg.vsc_index);
    if (itv != op.vsc_by_index.end()) {
      if (p_ac == 0.0) p_ac = itv->second.p_ac_mw;
      if (q_ac == 0.0) q_ac = itv->second.q_ac_mvar;
      if (p_dc == 0.0) p_dc = itv->second.p_dc_mw;
    } else {
      for (const auto& v : sys.vsc_converters) {
        if (v.index == no.cfg.vsc_index) {
          if (p_ac == 0.0) p_ac = v.p_set_mw;
          if (q_ac == 0.0) q_ac = v.q_set_mvar;
          if (p_dc == 0.0) { p_dc = -v.p_set_mw; used_lossless_nic_guess = true; }  // lossless guess
          break;
        }
      }
    }
    // I_ac,1 = conj(S_ac)/conj(V_ac,1), bus-injection positive.
    if (no.ac_p >= 0) {
      Cx vac1 = op.vac1[no.ac_p];
      Cx sac(p_ac / base, q_ac / base);
      Cx iac1 = (std::abs(vac1) > 1e-9) ? std::conj(sac) / std::conj(vac1) : Cx(0.0, 0.0);
      no.i_ac1_mag = std::abs(iac1);
      no.i_ac1_ph = std::arg(iac1);
    }
    // I_dc,0 = P_dc / V_dc,0.
    if (no.dc_p >= 0) {
      double vdc0 = op.vdc0[no.dc_p];
      no.i_dc0 = (std::abs(vdc0) > 1e-9) ? (p_dc / base) / vdc0 : 0.0;
      if (no.cfg.dc_port == PortBehavior::GridForming)
        nic_dc_positions.push_back(no.dc_p);
    }
    if (no.cfg.ac_spectrum.empty()) no.cfg.ac_spectrum = default_six_pulse_ac_spectrum();
    if (no.cfg.dc_spectrum.empty()) no.cfg.dc_spectrum = default_dc_ripple_spectrum();
    nic_ops.push_back(std::move(no));
  }

  // ── Per-bus accumulators for the harmonic voltage spectra ──
  std::vector<HarmonicBusResult> ac_res(n_ac);
  std::vector<HarmonicBusResult> dc_res(n_dc);
  for (int i = 0; i < n_ac; ++i) {
    ac_res[i].bus = sys.ac.buses[i].index;
    ac_res[i].is_dc = false;
    ac_res[i].v_fund_pu = std::abs(op.vac1[i]);
    ac_res[i].v_by_order[1] = op.vac1[i];
  }
  for (int k = 0; k < n_dc; ++k) {
    dc_res[k].bus = sys.dc.buses[k].index;
    dc_res[k].is_dc = true;
    dc_res[k].v_fund_pu = std::abs(op.vdc0[k]);
    dc_res[k].v_by_order[0] = Cx(op.vdc0[k], 0.0);
  }

  // Helper: build a per-order injection vector and solve.
  auto solve_ac_order = [&](int h, Eigen::VectorXcd& V) -> bool {
    SpMat Y = build_ac_ybus(sys, op, ac_id2pos, h, opt);
    Eigen::VectorXcd I = Eigen::VectorXcd::Zero(n_ac);

    // User single-port AC current sources.
    for (const auto& src : inputs.sources) {
      if (src.is_dc) continue;
      int i = ac_pos(src.bus);
      if (i < 0) continue;
      double ibase = src.i_base_pu;
      if (ibase <= 0.0) {
        // Derive from operating-point nodal load current if not specified.
        ibase = 0.0;
      }
      for (const auto& line : src.spectrum) {
        if (line.order != h) continue;
        double mag = (line.mag_percent / 100.0) * ibase;
        I(i) += std::polar(mag, line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
      }
    }
    // NIC AC-port current injection + optional Norton output admittance.
    for (const auto& no : nic_ops) {
      if (no.ac_p < 0) continue;
      for (const auto& line : no.cfg.ac_spectrum) {
        if (line.order != h) continue;
        double mag = (line.mag_percent / 100.0) * no.i_ac1_mag;
        I(no.ac_p) += std::polar(mag, line.phase_deg * kDeg2Rad + no.i_ac1_ph);
      }
      if (std::abs(no.cfg.y_out_ac) > 0) {
        Cx yo(no.cfg.y_out_ac.real(), static_cast<double>(h) * no.cfg.y_out_ac.imag());
        Y.coeffRef(no.ac_p, no.ac_p) += yo;
      }
    }

    Eigen::SparseLU<SpMat> lu;
    lu.compute(Y);
    if (lu.info() != Eigen::Success) return false;
    V = lu.solve(I);
    return lu.info() == Eigen::Success;
  };

  auto solve_dc_order = [&](int r, Eigen::VectorXcd& V) -> bool {
    SpMat Y = build_dc_ybus(sys.dc, dc_id2pos, nic_dc_positions, r, opt);
    Eigen::VectorXcd I = Eigen::VectorXcd::Zero(n_dc);
    for (const auto& src : inputs.sources) {
      if (!src.is_dc) continue;
      int k = dc_pos(src.bus);
      if (k < 0) continue;
      for (const auto& line : src.spectrum) {
        if (line.order != r) continue;
        double mag = (line.mag_percent / 100.0) * src.i_base_pu;
        I(k) += std::polar(mag, line.phase_deg * kDeg2Rad);
      }
    }
    for (const auto& no : nic_ops) {
      if (no.dc_p < 0) continue;
      for (const auto& line : no.cfg.dc_spectrum) {
        if (line.order != r) continue;
        double mag = (line.mag_percent / 100.0) * std::abs(no.i_dc0);
        I(no.dc_p) += std::polar(mag, line.phase_deg * kDeg2Rad);
      }
    }
    if (n_dc == 0) { V = Eigen::VectorXcd::Zero(0); return true; }
    Eigen::SparseLU<SpMat> lu;
    lu.compute(Y);
    if (lu.info() != Eigen::Success) return false;
    V = lu.solve(I);
    return lu.info() == Eigen::Success;
  };

  // ── Solve every AC harmonic order ──
  if (n_ac > 0) {
    for (int h : opt.ac_orders) {
      if (h <= 1) continue;
      Eigen::VectorXcd V;
      bool okrow = solve_ac_order(h, V);
      res.ac_order_solved[h] = okrow;
      if (!okrow) continue;
      res.ac_orders.push_back(h);
      for (int i = 0; i < n_ac; ++i) ac_res[i].v_by_order[h] = V(i);
    }
  }
  // ── Solve every DC ripple order ──
  if (n_dc > 0) {
    for (int r : opt.dc_orders) {
      if (r <= 0) continue;
      Eigen::VectorXcd V;
      bool okrow = solve_dc_order(r, V);
      res.dc_order_solved[r] = okrow;
      if (!okrow) continue;
      res.dc_orders.push_back(r);
      for (int k = 0; k < n_dc; ++k) dc_res[k].v_by_order[r] = V(k);
    }
  }

  // ── THD post-processing ──
  for (auto& r : ac_res) {
    r.thd_pct = thd_from_orders(r.v_by_order, 1, r.v_fund_pu);
    if (r.thd_pct > res.max_ac_thd_pct) {
      res.max_ac_thd_pct = r.thd_pct;
      res.max_ac_thd_bus = r.bus;
    }
  }
  for (auto& r : dc_res) {
    r.thd_pct = thd_from_orders(r.v_by_order, 0, r.v_fund_pu);
    if (r.thd_pct > res.max_dc_thd_pct) {
      res.max_dc_thd_pct = r.thd_pct;
      res.max_dc_thd_bus = r.bus;
    }
  }

  // ── Optional branch harmonic current flows ──
  if (opt.compute_branch_flows) {
    for (const auto& br : sys.ac.branches) {
      if (!br.in_service) continue;
      int f = ac_pos(br.from_bus), t = ac_pos(br.to_bus);
      if (f < 0 || t < 0) continue;
      HarmonicBranchFlow bf;
      bf.from_bus = br.from_bus;
      bf.to_bus = br.to_bus;
      bf.is_dc = false;
      double i_fund = 0.0, acc = 0.0;
      // Series current at each order: I = ys(h) * (Vf - Vt).
      std::vector<int> orders{1};
      for (int h : res.ac_orders) orders.push_back(h);
      for (int h : orders) {
        auto itf = ac_res[f].v_by_order.find(h);
        auto itt = ac_res[t].v_by_order.find(h);
        if (itf == ac_res[f].v_by_order.end() || itt == ac_res[t].v_by_order.end())
          continue;
        Cx z(skin_r(br.r_pu, static_cast<double>(h), opt),
             static_cast<double>(h) * br.x_pu);
        Cx ys = (std::abs(z) > 1e-12) ? Cx(1.0, 0.0) / z : Cx(0.0, 0.0);
        double im = std::abs(ys * (itf->second - itt->second));
        bf.i_by_order[h] = im;
        if (h == 1) i_fund = im; else acc += im * im;
      }
      bf.thd_i_pct = (i_fund > 1e-12) ? std::sqrt(acc) / i_fund * 100.0 : 0.0;
      res.ac_branch_flows.push_back(std::move(bf));
    }
    for (const auto& br : sys.dc.branches) {
      if (!br.in_service) continue;
      int f = dc_pos(br.from_bus), t = dc_pos(br.to_bus);
      if (f < 0 || t < 0) continue;
      HarmonicBranchFlow bf;
      bf.from_bus = br.from_bus;
      bf.to_bus = br.to_bus;
      bf.is_dc = true;
      double r = std::max(br.r_pu, 1e-6);
      // Ripple reactance (if configured) so the reported branch current uses the
      // SAME order-dependent impedance Z(r)=r_pu + j*r*x_pu as build_dc_ybus().
      double x_pu = 0.0;
      auto xit = opt.dc_ripple_model.branch_x_pu.find(br.index);
      if (xit != opt.dc_ripple_model.branch_x_pu.end()) x_pu = xit->second;
      double i_fund = 0.0, acc = 0.0;
      std::vector<int> orders{0};
      for (int o : res.dc_orders) orders.push_back(o);
      for (int o : orders) {
        auto itf = dc_res[f].v_by_order.find(o);
        auto itt = dc_res[t].v_by_order.find(o);
        if (itf == dc_res[f].v_by_order.end() || itt == dc_res[t].v_by_order.end())
          continue;
        Cx z(r, static_cast<double>(o) * x_pu);
        Cx ys = (std::abs(z) > 1e-12) ? Cx(1.0, 0.0) / z : Cx(0.0, 0.0);
        double im = std::abs(ys * (itf->second - itt->second));
        bf.i_by_order[o] = im;
        if (o == 0) i_fund = im; else acc += im * im;
      }
      bf.thd_i_pct = (i_fund > 1e-12) ? std::sqrt(acc) / i_fund * 100.0 : 0.0;
      res.dc_branch_flows.push_back(std::move(bf));
    }
  }

  if (sys.bus_merge_map) {
    const auto positions =
        projection::CanonicalToRichOperator::ac_bus_reprojection_positions(
            rich_sys, projection_bundle);
    res.ac_bus_results.reserve(rich_sys.ac.buses.size());
    for (size_t i = 0; i < rich_sys.ac.buses.size(); ++i) {
      const auto& rich_bus = rich_sys.ac.buses[i];
      HarmonicBusResult attributed;
      attributed.bus = rich_bus.index;
      attributed.is_dc = false;
      attributed.v_fund_pu = 0.0;
      const int position = positions[i];
      if (position >= 0 && position < static_cast<int>(ac_res.size())) {
        attributed = ac_res[static_cast<size_t>(position)];
        attributed.bus = rich_bus.index;
      }
      res.ac_bus_results.push_back(std::move(attributed));
    }
    res.max_ac_thd_pct = 0.0;
    res.max_ac_thd_bus = -1;
    for (const auto& result : res.ac_bus_results) {
      if (result.thd_pct > res.max_ac_thd_pct) {
        res.max_ac_thd_pct = result.thd_pct;
        res.max_ac_thd_bus = result.bus;
      }
    }
  } else {
    res.ac_bus_results = std::move(ac_res);
  }
  res.dc_bus_results = std::move(dc_res);
  res.ok = true;
  if (!res.base_pf_converged && opt.run_base_power_flow)
    res.message = "base power flow did not converge; used stored/nominal voltages";
  if (used_lossless_nic_guess) {
    if (!res.message.empty()) res.message += "; ";
    res.message += "NIC DC operating point assumed lossless (P_dc = -P_ac); "
                   "supply p_dc_mw or run a base power flow for converter losses";
  }
  return res;
}

HPFResult solve_harmonic_power_flow(const HybridPowerSystem& sys,
                                    const HPFOptions& opt) {
  return solve_harmonic_power_flow(sys, HarmonicStudyInputs{}, opt);
}

// ═══════════════════════════════════════════════════════════════════════════
// Three-phase (abc-domain) harmonic power flow
// ═══════════════════════════════════════════════════════════════════════════
namespace {

using Mat3 = Eigen::Matrix3cd;

// Symmetrical-component -> phase impedance matrix:  Z_abc = A·diag(z0,z1,z2)·A⁻¹.
// Matches build_sequence_zabc() in src/power_flow/three_phase_nr.cpp.
Mat3 seq_to_phase_zabc(Cx z0, Cx z1, Cx z2) {
  const Cx a = std::polar(1.0, 2.0 * M_PI / 3.0);
  const Cx a2 = std::polar(1.0, 4.0 * M_PI / 3.0);
  Mat3 A;
  A << Cx(1, 0), Cx(1, 0), Cx(1, 0),
       Cx(1, 0), a2, a,
       Cx(1, 0), a, a2;
  Mat3 zs = Mat3::Zero();
  zs(0, 0) = z0;
  zs(1, 1) = z1;
  zs(2, 2) = z2;
  return A * zs * A.inverse();
}

// ── Three-phase transformer winding connections (vector groups) ──
enum class WindingConn { WyeG, Wye, Delta };

struct XfmrConn {
  WindingConn hv{WindingConn::WyeG};
  WindingConn lv{WindingConn::WyeG};
  int clock{11};  ///< vector-group clock number (Dyn11 default)
};

// Parse a vector-group string ("Dyn11", "YNd11", "Dd0", ...) into HV/LV winding
// types and the clock number, falling back to the per-winding topology strings.
XfmrConn parse_vector_group(const std::string& vg, const std::string& hv_topo,
                            const std::string& lv_topo) {
  XfmrConn c;
  int slot = 0;  // 0 -> assigning HV, 1 -> assigning LV, 2 -> done
  std::string digits;
  for (char ch : vg) {
    if (std::isdigit(static_cast<unsigned char>(ch))) { digits.push_back(ch); continue; }
    const char u = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    if (u == 'D' || u == 'Z' || u == 'Y') {
      WindingConn w = (u == 'Y') ? WindingConn::Wye : WindingConn::Delta;
      if (slot == 0) { c.hv = w; slot = 1; }
      else if (slot == 1) { c.lv = w; slot = 2; }
    } else if (u == 'N') {  // grounded neutral for the last-assigned wye winding
      if (slot == 1 && c.hv == WindingConn::Wye) c.hv = WindingConn::WyeG;
      else if (slot == 2 && c.lv == WindingConn::Wye) c.lv = WindingConn::WyeG;
    }
  }
  if (!digits.empty()) c.clock = std::stoi(digits) % 12;
  if (vg.empty()) {
    auto from_topo = [](const std::string& t) {
      std::string s;
      for (char x : t) s.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(x))));
      if (s.find("delta") != std::string::npos) return WindingConn::Delta;
      if (s.find("ground") != std::string::npos || s.find("wye-g") != std::string::npos)
        return WindingConn::WyeG;
      if (s.find("wye") != std::string::npos || s.find("star") != std::string::npos)
        return WindingConn::Wye;
      return WindingConn::WyeG;
    };
    c.hv = from_topo(hv_topo);
    c.lv = from_topo(lv_topo);
  }
  return c;
}

// Six-pulse-symmetric primitive connection blocks (Arrillaga).  yt = per-phase
// leakage admittance at the harmonic order.  Encodes zero-sequence blocking
// (delta -> singular for the [1,1,1] mode) and the ±30° vector-group shift.
struct XfmrBlocks { Mat3 ypp, yss, yps, ysp; };

XfmrBlocks transformer_blocks(Cx yt, const XfmrConn& c) {
  Mat3 YI = yt * Mat3::Identity();
  Mat3 M2;
  M2 << Cx(2,0), Cx(-1,0), Cx(-1,0),
        Cx(-1,0), Cx(2,0), Cx(-1,0),
        Cx(-1,0), Cx(-1,0), Cx(2,0);
  Mat3 YII = (yt / 3.0) * M2;
  Mat3 M3;
  M3 << Cx(-1,0), Cx(1,0), Cx(0,0),
        Cx(0,0), Cx(-1,0), Cx(1,0),
        Cx(1,0), Cx(0,0), Cx(-1,0);
  Mat3 YIII = (yt / std::sqrt(3.0)) * M3;
  // Clock parity selects the ±30° orientation (11,7,3 -> transpose).
  if (c.clock % 4 == 3) YIII = YIII.transpose().eval();

  const bool hv_delta = (c.hv == WindingConn::Delta);
  const bool lv_delta = (c.lv == WindingConn::Delta);
  XfmrBlocks b;
  if (!hv_delta && !lv_delta) {            // Yg-Yg
    b.ypp = YI;  b.yss = YI;  b.yps = -YI;             b.ysp = -YI;
  } else if (hv_delta && lv_delta) {       // D-D
    b.ypp = YII; b.yss = YII; b.yps = -YII;            b.ysp = -YII;
  } else if (hv_delta && !lv_delta) {      // D-Yg
    b.ypp = YII; b.yss = YI;  b.yps = YIII;            b.ysp = YIII.transpose().eval();
  } else {                                 // Yg-D
    b.ypp = YI;  b.yss = YII; b.yps = YIII;            b.ysp = YIII.transpose().eval();
  }
  return b;
}

// Rotate a balanced A-phase phasor into the (b, c) phases for a given sequence.
//   seq 1 (positive): b = a²·a_ph, c = a·a_ph
//   seq 2 (negative): b = a·a_ph,  c = a²·a_ph
//   seq 0 (zero)    : b = c = a_ph
void balanced_phase_set(Cx a_ph, int seq, Cx& b_ph, Cx& c_ph) {
  const Cx a = std::polar(1.0, 2.0 * M_PI / 3.0);   // e^{+j120}
  const Cx a2 = std::polar(1.0, 4.0 * M_PI / 3.0);  // e^{-j120}
  if (seq == 1) {        // positive
    b_ph = a2 * a_ph;
    c_ph = a * a_ph;
  } else if (seq == 2) {  // negative
    b_ph = a * a_ph;
    c_ph = a2 * a_ph;
  } else {                // zero
    b_ph = a_ph;
    c_ph = a_ph;
  }
}

// Build the 3N×3N abc-domain harmonic admittance matrix at order h.  Shared by
// the pure three-phase solver and the coupled hybrid AC/DC solver.
SpMat build_3ph_ac_ybus(const ThreePhaseACSystem& sys,
                        const std::vector<std::array<Cx, 3>>& vph1,
                        const std::unordered_map<int, int>& id2pos, double h,
                        const HPFOptions& opt) {
  const int n = static_cast<int>(sys.buses.size());
  const double base = sys.base_mva > 0 ? sys.base_mva : 100.0;
  const double hh = static_cast<double>(h);
  auto pos = [&](int id) -> int {
    auto it = id2pos.find(id);
    return it == id2pos.end() ? -1 : it->second;
  };
  auto node = [](int p, int ph) { return p * 3 + ph; };

  std::vector<Trip> trips;
  trips.reserve(sys.lines.size() * 36 + n * 6);

  auto stamp_block = [&](int pf, int pt, const Mat3& yser, const Mat3& yshh,
                         PhaseMask mf, PhaseMask mt, PhaseMask ml) {
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        if (!ml.has(r) || !ml.has(c)) continue;
        if (mf.has(r) && mf.has(c))
          trips.emplace_back(node(pf, r), node(pf, c), yser(r, c) + yshh(r, c));
        if (mt.has(r) && mt.has(c))
          trips.emplace_back(node(pt, r), node(pt, c), yser(r, c) + yshh(r, c));
        if (mf.has(r) && mt.has(c))
          trips.emplace_back(node(pf, r), node(pt, c), -yser(r, c));
        if (mt.has(r) && mf.has(c))
          trips.emplace_back(node(pt, r), node(pf, c), -yser(r, c));
      }
    }
  };

  for (const auto& ln : sys.lines) {
    if (!ln.in_service) continue;
    int pf = pos(ln.from_bus), pt = pos(ln.to_bus);
    if (pf < 0 || pt < 0) continue;
    Mat3 zabc, yshh;
    if (ln.use_phase_matrix) {
      Mat3 z = Mat3::Zero();
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          z(r, c) = Cx(skin_r(phase_matrix_get(ln.r_matrix_pu, r, c), h, opt),
                       hh * phase_matrix_get(ln.x_matrix_pu, r, c));
      zabc = z;
      yshh = Mat3::Zero();
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
          yshh(r, c) = Cx(0.0, hh * phase_matrix_get(ln.b_matrix_pu, r, c) / 2.0);
    } else {
      Cx z1(skin_r(ln.r1_pu, h, opt), hh * ln.x1_pu);
      Cx z0(skin_r(ln.r0_pu, h, opt), hh * ln.x0_pu);
      if (std::abs(z0) < 1e-20) z0 = z1;
      zabc = seq_to_phase_zabc(z0, z1, z1);
      yshh = Mat3::Zero();
      for (int ph = 0; ph < 3; ++ph) yshh(ph, ph) = Cx(0.0, hh * ln.b1_pu / 2.0);
    }
    Mat3 yser = zabc.inverse();
    stamp_block(pf, pt, yser, yshh, sys.buses[pf].phase_mask,
                sys.buses[pt].phase_mask, ln.phase_mask);
  }

  for (const auto& tf : sys.transformers) {
    if (!tf.in_service || tf.sn_mva <= 0) continue;
    int pf = pos(tf.hv_bus), pt = pos(tf.lv_bus);
    if (pf < 0 || pt < 0) continue;
    double zpu = (tf.vk_percent / 100.0) * (base / tf.sn_mva);
    double rpu = (tf.vkr_percent / 100.0) * (base / tf.sn_mva);
    double xpu = std::sqrt(std::max(zpu * zpu - rpu * rpu, 0.0));
    Cx yt = Cx(1.0, 0.0) / Cx(skin_r(rpu, h, opt), hh * xpu);
    // Vector-group aware connection blocks: encodes Dy/Yd ±30° shift and the
    // delta-winding zero-sequence (triplen) blocking.
    XfmrConn conn = parse_vector_group(tf.vector_group, tf.hv_winding_topology,
                                       tf.lv_winding_topology);
    XfmrBlocks blk = transformer_blocks(yt, conn);
    const PhaseMask mh = tf.hv_phase_mask, ml = tf.lv_phase_mask;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        if (mh.has(r) && mh.has(c)) trips.emplace_back(node(pf, r), node(pf, c), blk.ypp(r, c));
        if (ml.has(r) && ml.has(c)) trips.emplace_back(node(pt, r), node(pt, c), blk.yss(r, c));
        if (mh.has(r) && ml.has(c)) trips.emplace_back(node(pf, r), node(pt, c), blk.yps(r, c));
        if (ml.has(r) && mh.has(c)) trips.emplace_back(node(pt, r), node(pf, c), blk.ysp(r, c));
      }
    }
  }

  std::vector<std::array<Cx, 3>> diag(n, {Cx(0, 0), Cx(0, 0), Cx(0, 0)});
  auto add_source = [&](int p, double r, double xpp) {
    if (p < 0) return;
    double x = (xpp > 1e-9) ? xpp : opt.default_source_xpp_pu;
    Cx z(r, hh * x);
    if (std::abs(z) > 1e-12) {
      Cx y = Cx(1.0, 0.0) / z;
      for (int ph = 0; ph < 3; ++ph)
        if (sys.buses[p].phase_mask.has(ph)) diag[p][ph] += y;
    }
  };
  for (const auto& g : sys.generators)
    if (g.in_service) add_source(pos(g.bus), 0.0, g.xdpp_pu);
  for (const auto& eg : sys.external_grids) {
    if (!eg.in_service) continue;
    int p = pos(eg.bus);
    if (p < 0) continue;
    double r = eg.r1_pu, x = eg.x1_pu;
    if (x <= 1e-9 && eg.s_sc_max_mva > 0) {
      double zsc = base / eg.s_sc_max_mva;
      double xr = eg.rx_max > 0 ? (1.0 / eg.rx_max) : 10.0;
      x = zsc / std::sqrt(1.0 + 1.0 / (xr * xr));
      r = x / xr;
    }
    add_source(p, r, x);
  }
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    bool is_src = false;
    for (const auto& g : sys.generators)
      if (g.in_service && g.bus == b.index) is_src = true;
    for (const auto& eg : sys.external_grids)
      if (eg.in_service && eg.bus == b.index) is_src = true;
    if ((b.bus_type == BusType::SLACK || b.bus_type == BusType::PV) && !is_src)
      add_source(i, 0.0, opt.default_source_xpp_pu);
  }

  if (opt.include_load_impedance) {
    auto stamp_load = [&](int p, int ph, double pmw, double qmvar) {
      if (p < 0 || !sys.buses[p].phase_mask.has(ph)) return;
      double pp = pmw / base, qq = qmvar / base;
      double vm = std::abs(vph1[p][ph]);
      if (vm < 1e-6) vm = 1.0;
      double v2 = vm * vm;
      Cx yl(pp / v2, 0.0);
      if (std::abs(qq) > 1e-12) yl += Cx(0.0, -qq / (hh * v2));
      diag[p][ph] += yl;
    };
    for (const auto& ld : sys.loads) {
      if (!ld.in_service) continue;
      int p = pos(ld.bus);
      stamp_load(p, 0, ld.p_a_mw, ld.q_a_mvar);
      stamp_load(p, 1, ld.p_b_mw, ld.q_b_mvar);
      stamp_load(p, 2, ld.p_c_mw, ld.q_c_mvar);
    }
  }
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    const double gs[3] = {b.gs_a_mw, b.gs_b_mw, b.gs_c_mw};
    const double bs[3] = {b.bs_a_mvar, b.bs_b_mvar, b.bs_c_mvar};
    for (int ph = 0; ph < 3; ++ph) {
      if (gs[ph] != 0.0 || bs[ph] != 0.0)
        diag[i][ph] += Cx(gs[ph] / base, hh * bs[ph] / base);
    }
  }
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    for (int ph = 0; ph < 3; ++ph) {
      if (!b.in_service || b.bus_type == BusType::ISOLATED || !b.phase_mask.has(ph))
        diag[i][ph] += Cx(1.0 / std::max(opt.min_shunt_pu, 1e-12), 0.0);
      diag[i][ph] += Cx(opt.min_shunt_pu, 0.0);
      trips.emplace_back(node(i, ph), node(i, ph), diag[i][ph]);
    }
  }

  SpMat Y(3 * n, 3 * n);
  Y.setFromTriplets(trips.begin(), trips.end());
  Y.makeCompressed();
  return Y;
}

// Extract per-phase fundamental phasors for a three-phase system (base PF or
// stored).  Returns whether a base power flow converged.
bool extract_3ph_vph1(const ThreePhaseACSystem& sys, const HPFOptions& opt,
                      std::vector<std::array<Cx, 3>>& vph1) {
  const int n = static_cast<int>(sys.buses.size());
  vph1.assign(n, {Cx(1, 0), Cx(1, 0), Cx(1, 0)});
  powerflow::ThreePhaseFlowResult tp;
  bool have_pf = false;
  if (opt.run_base_power_flow) {
    try { tp = powerflow::solve_three_phase(sys, {}); have_pf = tp.converged; }
    catch (...) { have_pf = false; }
  }
  std::unordered_map<int, int> pfpos;
  if (have_pf)
    for (int i = 0; i < (int)tp.bus_results.size(); ++i) pfpos[tp.bus_results[i].bus_id] = i;
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    double vma = b.vm_a_pu, vaa = b.va_a_deg, vmb = b.vm_b_pu, vab = b.va_b_deg,
           vmc = b.vm_c_pu, vac = b.va_c_deg;
    if (have_pf) {
      auto it = pfpos.find(b.index);
      if (it != pfpos.end()) {
        const auto& r = tp.bus_results[it->second];
        if (r.vm_a_pu > 1e-9) { vma = r.vm_a_pu; vaa = r.va_a_deg; }
        if (r.vm_b_pu > 1e-9) { vmb = r.vm_b_pu; vab = r.va_b_deg; }
        if (r.vm_c_pu > 1e-9) { vmc = r.vm_c_pu; vac = r.va_c_deg; }
      }
    }
    if (vma <= 1e-9) vma = 1.0;
    if (vmb <= 1e-9) vmb = 1.0;
    if (vmc <= 1e-9) vmc = 1.0;
    vph1[i] = {std::polar(vma, vaa * kDeg2Rad), std::polar(vmb, vab * kDeg2Rad),
               std::polar(vmc, vac * kDeg2Rad)};
  }
  return have_pf;
}

}  // namespace

int harmonic_sequence_of_order(int h) {
  int m = ((h % 3) + 3) % 3;  // 0,1,2
  return m;                   // 1 = positive, 2 = negative, 0 = zero
}

std::string HPF3phResult::summary() const {
  std::ostringstream os;
  os << "Three-phase harmonic power flow: " << (ok ? "OK" : "FAILED");
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  AC orders solved: " << ac_orders.size();
  os << "\n  Max phase-voltage THD: " << max_thd_pct << " % at bus " << max_thd_bus;
  return os.str();
}

HPF3phResult solve_harmonic_power_flow_3ph(const ThreePhaseACSystem& sys,
                                           const ThreePhaseHarmonicInputs& inputs,
                                           const HPFOptions& opt) {
  HPF3phResult res;
  const int n = static_cast<int>(sys.buses.size());
  const double base = sys.base_mva > 0 ? sys.base_mva : 100.0;
  if (n == 0) {
    res.message = "system has no buses";
    return res;
  }

  std::unordered_map<int, int> id2pos;
  id2pos.reserve(n);
  for (int i = 0; i < n; ++i) id2pos[sys.buses[i].index] = i;
  auto pos = [&](int id) -> int {
    auto it = id2pos.find(id);
    return it == id2pos.end() ? -1 : it->second;
  };
  auto node = [](int p, int ph) { return p * 3 + ph; };

  // ── Operating point: per-phase fundamental phasors ──
  std::vector<std::array<Cx, 3>> vph1(n);
  powerflow::ThreePhaseFlowResult tp;
  bool have_pf = false;
  if (opt.run_base_power_flow) {
    try {
      tp = powerflow::solve_three_phase(sys, {});
      have_pf = tp.converged;
    } catch (...) {
      have_pf = false;
    }
  }
  res.base_pf_converged = have_pf;
  std::unordered_map<int, int> pfpos;
  if (have_pf)
    for (int i = 0; i < (int)tp.bus_results.size(); ++i)
      pfpos[tp.bus_results[i].bus_id] = i;

  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    double vma = b.vm_a_pu, vaa = b.va_a_deg;
    double vmb = b.vm_b_pu, vab = b.va_b_deg;
    double vmc = b.vm_c_pu, vac = b.va_c_deg;
    if (have_pf) {
      auto it = pfpos.find(b.index);
      if (it != pfpos.end()) {
        const auto& r = tp.bus_results[it->second];
        if (r.vm_a_pu > 1e-9) { vma = r.vm_a_pu; vaa = r.va_a_deg; }
        if (r.vm_b_pu > 1e-9) { vmb = r.vm_b_pu; vab = r.va_b_deg; }
        if (r.vm_c_pu > 1e-9) { vmc = r.vm_c_pu; vac = r.va_c_deg; }
      }
    }
    if (vma <= 1e-9) vma = 1.0;
    if (vmb <= 1e-9) vmb = 1.0;
    if (vmc <= 1e-9) vmc = 1.0;
    vph1[i] = {std::polar(vma, vaa * kDeg2Rad), std::polar(vmb, vab * kDeg2Rad),
               std::polar(vmc, vac * kDeg2Rad)};
  }

  // ── Effective NIC list (explicit + auto from converters is not available for
  //    a bare ThreePhaseACSystem, so only explicit NICs are used) ──
  std::vector<ThreePhaseHarmonicNIC> nics = inputs.nics;

  // Per-bus result accumulators.
  std::vector<ThreePhaseHarmonicBusResult> bus_res(n);
  for (int i = 0; i < n; ++i) {
    bus_res[i].bus = sys.buses[i].index;
    bus_res[i].phase_mask = sys.buses[i].phase_mask;
    bus_res[i].v_fund_pu_a = std::abs(vph1[i][0]);
    bus_res[i].v_fund_pu_b = std::abs(vph1[i][1]);
    bus_res[i].v_fund_pu_c = std::abs(vph1[i][2]);
    bus_res[i].v_by_order_a[1] = vph1[i][0];
    bus_res[i].v_by_order_b[1] = vph1[i][1];
    bus_res[i].v_by_order_c[1] = vph1[i][2];
  }

  // ── Build the 3N×3N harmonic admittance matrix at order h ──
  auto build_ybus = [&](int h) -> SpMat {
    return build_3ph_ac_ybus(sys, vph1, id2pos, h, opt);
  };

  // ── Injection vector at order h ──
  auto build_inj = [&](int h) -> Eigen::VectorXcd {
    Eigen::VectorXcd I = Eigen::VectorXcd::Zero(3 * n);
    const int seq = harmonic_sequence_of_order(h);

    for (const auto& src : inputs.sources) {
      int p = pos(src.bus);
      if (p < 0) continue;
      for (const auto& line : src.spectrum) {
        if (line.order != h) continue;
        if (src.balanced) {
          double mag = (line.mag_percent / 100.0) * src.i_base_pu;
          Cx ia = std::polar(mag, line.phase_deg * kDeg2Rad +
                                       src.i_base_phase_deg * kDeg2Rad);
          Cx ib, ic;
          balanced_phase_set(ia, seq, ib, ic);
          if (sys.buses[p].phase_mask.has(0)) I(node(p, 0)) += ia;
          if (sys.buses[p].phase_mask.has(1)) I(node(p, 1)) += ib;
          if (sys.buses[p].phase_mask.has(2)) I(node(p, 2)) += ic;
        } else {
          const double ib_[3] = {src.i_base_pu_a, src.i_base_pu_b, src.i_base_pu_c};
          for (int ph = 0; ph < 3; ++ph) {
            if (!sys.buses[p].phase_mask.has(ph)) continue;
            double mag = (line.mag_percent / 100.0) * ib_[ph];
            I(node(p, ph)) += std::polar(mag, line.phase_deg * kDeg2Rad);
          }
        }
      }
    }

    // Three-phase NIC AC port: balanced injection scaled by per-phase |I_φ,1|.
    for (const auto& nic : nics) {
      int p = pos(nic.bus_ac);
      if (p < 0) continue;
      HarmonicSpectrum spec = nic.ac_spectrum.empty()
                                  ? default_six_pulse_ac_spectrum()
                                  : nic.ac_spectrum;
      Cx sphi((nic.s_ac_p_mw / 3.0) / base, (nic.s_ac_q_mvar / 3.0) / base);
      Cx va1 = vph1[p][0];
      Cx ia1 = (std::abs(va1) > 1e-9) ? std::conj(sphi) / std::conj(va1) : Cx(0, 0);
      double iref = std::abs(ia1);
      double iph1 = std::arg(ia1);
      for (const auto& line : spec) {
        if (line.order != h) continue;
        double mag = (line.mag_percent / 100.0) * iref;
        Cx ia = std::polar(mag, line.phase_deg * kDeg2Rad + iph1);
        Cx ib, ic;
        balanced_phase_set(ia, seq, ib, ic);
        if (sys.buses[p].phase_mask.has(0)) I(node(p, 0)) += ia;
        if (sys.buses[p].phase_mask.has(1)) I(node(p, 1)) += ib;
        if (sys.buses[p].phase_mask.has(2)) I(node(p, 2)) += ic;
      }
    }
    return I;
  };

  // ── Solve each harmonic order ──
  for (int h : opt.ac_orders) {
    if (h <= 1) continue;
    SpMat Y = build_ybus(h);
    Eigen::VectorXcd I = build_inj(h);
    Eigen::SparseLU<SpMat> lu;
    lu.compute(Y);
    bool okrow = (lu.info() == Eigen::Success);
    Eigen::VectorXcd V;
    if (okrow) {
      V = lu.solve(I);
      okrow = (lu.info() == Eigen::Success);
    }
    res.ac_order_solved[h] = okrow;
    if (!okrow) continue;
    res.ac_orders.push_back(h);
    for (int i = 0; i < n; ++i) {
      bus_res[i].v_by_order_a[h] = V(node(i, 0));
      bus_res[i].v_by_order_b[h] = V(node(i, 1));
      bus_res[i].v_by_order_c[h] = V(node(i, 2));
    }
  }

  // ── THD post-processing (per phase) ──
  for (auto& r : bus_res) {
    r.thd_a_pct = thd_from_orders(r.v_by_order_a, 1, r.v_fund_pu_a);
    r.thd_b_pct = thd_from_orders(r.v_by_order_b, 1, r.v_fund_pu_b);
    r.thd_c_pct = thd_from_orders(r.v_by_order_c, 1, r.v_fund_pu_c);
    double mx = std::max({r.thd_a_pct, r.thd_b_pct, r.thd_c_pct});
    if (mx > res.max_thd_pct) {
      res.max_thd_pct = mx;
      res.max_thd_bus = r.bus;
    }
  }

  res.bus_results = std::move(bus_res);
  res.ok = true;
  if (!res.base_pf_converged && opt.run_base_power_flow)
    res.message = "base power flow did not converge; used stored/nominal voltages";
  return res;
}

HPF3phResult solve_harmonic_power_flow_3ph(const ThreePhaseACSystem& sys,
                                           const HPFOptions& opt) {
  return solve_harmonic_power_flow_3ph(sys, ThreePhaseHarmonicInputs{}, opt);
}

// ═══════════════════════════════════════════════════════════════════════════
// Harmonic distortion limit compliance (IEEE 519-2014 / GB-T 14549-1993)
// ═══════════════════════════════════════════════════════════════════════════
std::pair<double, double> harmonic_voltage_limits(HarmonicStandard standard,
                                                  double base_kv, int order) {
  if (standard == HarmonicStandard::IEEE519_2014) {
    // IEEE 519-2014 Table 1 — voltage distortion limits (individual, THD).
    if (base_kv <= 1.0)   return {5.0, 8.0};
    if (base_kv <= 69.0)  return {3.0, 5.0};
    if (base_kv <= 161.0) return {1.5, 2.5};
    return {1.0, 1.5};
  }
  // GB/T 14549-1993 — public-grid voltage harmonic limits.
  // Returns the individual-harmonic limit for the order's parity (odd/even).
  double thd, odd, even;
  if (base_kv <= 1.0)        { thd = 5.0; odd = 4.0; even = 2.0; }
  else if (base_kv <= 20.0)  { thd = 4.0; odd = 3.2; even = 1.6; }
  else if (base_kv <= 66.0)  { thd = 3.0; odd = 2.4; even = 1.2; }
  else if (base_kv <= 110.0) { thd = 2.0; odd = 1.6; even = 0.8; }
  else                       { thd = 1.5; odd = 1.2; even = 0.6; }
  const double ihd = (order % 2 == 0) ? even : odd;
  return {ihd, thd};
}

namespace {

// Evaluate one voltage spectrum against a standard.  fundamental = 1 (AC) or 0 (DC).
HarmonicLimitCheck eval_limit_check(const std::map<int, Cx>& v_by_order,
                                    int fundamental, double v_fund, double thd_pct,
                                    int bus, bool is_dc, int phase, double base_kv,
                                    HarmonicStandard standard) {
  HarmonicLimitCheck c;
  c.bus = bus;
  c.is_dc = is_dc;
  c.phase = phase;
  c.base_kv = base_kv;
  c.thd_pct = thd_pct;

  // THD limit: order-independent component of the standard (use order 3 as a
  // representative odd order to retrieve the THD column).
  c.thd_limit_pct = harmonic_voltage_limits(standard, base_kv, 3).second;
  c.thd_ok = thd_pct <= c.thd_limit_pct + 1e-9;

  bool ihd_ok = true;
  double worst = -1.0;
  int worst_order = 0;
  double worst_limit = 0.0;
  if (v_fund > 1e-12) {
    for (const auto& [ord, v] : v_by_order) {
      if (ord == fundamental) continue;
      const double ihd = std::abs(v) / v_fund * 100.0;
      const double lim = harmonic_voltage_limits(standard, base_kv, ord).first;
      if (ihd > lim + 1e-9) ihd_ok = false;
      if (ihd > worst) { worst = ihd; worst_order = ord; worst_limit = lim; }
    }
  }
  c.worst_ihd_order = worst_order;
  c.worst_ihd_pct = (worst < 0.0) ? 0.0 : worst;
  c.ihd_limit_pct = worst_limit;
  c.ihd_ok = ihd_ok;
  c.compliant = c.thd_ok && c.ihd_ok;
  return c;
}

void finalize_report(HarmonicComplianceReport& rep) {
  rep.all_compliant = true;
  rep.n_violations = 0;
  rep.worst_ratio = 0.0;
  rep.worst_bus = -1;
  for (const auto& c : rep.checks) {
    if (!c.compliant) {
      rep.all_compliant = false;
      ++rep.n_violations;
    }
    if (c.thd_limit_pct > 1e-9) {
      double ratio = c.thd_pct / c.thd_limit_pct;
      if (ratio > rep.worst_ratio) {
        rep.worst_ratio = ratio;
        rep.worst_bus = c.bus;
      }
    }
  }
}

}  // namespace

std::string HarmonicComplianceReport::summary() const {
  std::ostringstream os;
  os << "Harmonic limits ("
     << (standard == HarmonicStandard::IEEE519_2014 ? "IEEE 519-2014"
                                                    : "GB/T 14549-1993")
     << "): " << (all_compliant ? "COMPLIANT" : "VIOLATIONS")
     << "\n  checks: " << checks.size() << ", violations: " << n_violations
     << "\n  worst THD/limit ratio: " << worst_ratio << " at bus " << worst_bus;
  return os.str();
}

HarmonicComplianceReport check_harmonic_limits(const HPFResult& result,
                                               const HybridPowerSystem& sys,
                                               HarmonicStandard standard) {
  HarmonicComplianceReport rep;
  rep.standard = standard;
  std::unordered_map<int, double> ac_kv;
  for (const auto& b : sys.ac.buses) ac_kv[b.index] = b.base_kv;
  // Voltage distortion limits are defined for AC buses only.
  for (const auto& br : result.ac_bus_results) {
    double kv = ac_kv.count(br.bus) ? ac_kv.at(br.bus) : 0.0;
    rep.checks.push_back(eval_limit_check(br.v_by_order, 1, br.v_fund_pu,
                                          br.thd_pct, br.bus, false, -1, kv,
                                          standard));
  }
  finalize_report(rep);
  return rep;
}

HarmonicComplianceReport check_harmonic_limits(const HPF3phResult& result,
                                               const ThreePhaseACSystem& sys,
                                               HarmonicStandard standard) {
  HarmonicComplianceReport rep;
  rep.standard = standard;
  std::unordered_map<int, double> kv;
  for (const auto& b : sys.buses) kv[b.index] = b.base_kv;
  for (const auto& br : result.bus_results) {
    double bkv = kv.count(br.bus) ? kv.at(br.bus) : 0.0;
    rep.checks.push_back(eval_limit_check(br.v_by_order_a, 1, br.v_fund_pu_a,
                                          br.thd_a_pct, br.bus, false, 0, bkv,
                                          standard));
    rep.checks.push_back(eval_limit_check(br.v_by_order_b, 1, br.v_fund_pu_b,
                                          br.thd_b_pct, br.bus, false, 1, bkv,
                                          standard));
    rep.checks.push_back(eval_limit_check(br.v_by_order_c, 1, br.v_fund_pu_c,
                                          br.thd_c_pct, br.bus, false, 2, bkv,
                                          standard));
  }
  finalize_report(rep);
  return rep;
}

// ═══════════════════════════════════════════════════════════════════════════
// Coupled three-phase AC + DC harmonic power flow
// ═══════════════════════════════════════════════════════════════════════════
std::string HPFHybrid3phResult::summary() const {
  std::ostringstream os;
  os << "Coupled 3φ AC/DC harmonic power flow: " << (ok ? "OK" : "FAILED");
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  AC orders: " << ac_orders.size() << ", DC orders: " << dc_orders.size();
  os << "\n  Max AC phase THD: " << max_ac_thd_pct << " % at bus " << max_ac_thd_bus;
  os << "\n  Max DC ripple THD: " << max_dc_thd_pct << " % at bus " << max_dc_thd_bus;
  return os.str();
}

HPFHybrid3phResult solve_harmonic_power_flow_3ph_hybrid(
    const ThreePhaseACSystem& ac, const DCSystem& dc,
    const ThreePhaseHybridInputs& inputs, const HPFOptions& opt) {
  HPFHybrid3phResult res;
  const int nac = static_cast<int>(ac.buses.size());
  const int ndc = static_cast<int>(dc.buses.size());
  const double ac_base = ac.base_mva > 0 ? ac.base_mva : 100.0;
  const double dc_base = dc.base_mva > 0 ? dc.base_mva : 100.0;
  if (nac == 0) {
    res.message = "three-phase AC system has no buses";
    return res;
  }

  std::unordered_map<int, int> ac_id2pos, dc_id2pos;
  for (int i = 0; i < nac; ++i) ac_id2pos[ac.buses[i].index] = i;
  for (int k = 0; k < ndc; ++k) dc_id2pos[dc.buses[k].index] = k;
  auto ac_pos = [&](int id) { auto it = ac_id2pos.find(id); return it == ac_id2pos.end() ? -1 : it->second; };
  auto dc_pos = [&](int id) { auto it = dc_id2pos.find(id); return it == dc_id2pos.end() ? -1 : it->second; };

  // Operating point: AC per-phase fundamental (3φ PF or stored), DC steady (stored).
  std::vector<std::array<Cx, 3>> vph1(nac);
  powerflow::ThreePhaseFlowResult tp;
  bool have_pf = false;
  if (opt.run_base_power_flow) {
    try { tp = powerflow::solve_three_phase(ac, {}); have_pf = tp.converged; }
    catch (...) { have_pf = false; }
  }
  res.combined_solved = false;
  std::unordered_map<int, int> pfpos;
  if (have_pf)
    for (int i = 0; i < (int)tp.bus_results.size(); ++i) pfpos[tp.bus_results[i].bus_id] = i;
  for (int i = 0; i < nac; ++i) {
    const auto& b = ac.buses[i];
    double vma = b.vm_a_pu, vaa = b.va_a_deg, vmb = b.vm_b_pu, vab = b.va_b_deg,
           vmc = b.vm_c_pu, vac = b.va_c_deg;
    if (have_pf) {
      auto it = pfpos.find(b.index);
      if (it != pfpos.end()) {
        const auto& r = tp.bus_results[it->second];
        if (r.vm_a_pu > 1e-9) { vma = r.vm_a_pu; vaa = r.va_a_deg; }
        if (r.vm_b_pu > 1e-9) { vmb = r.vm_b_pu; vab = r.va_b_deg; }
        if (r.vm_c_pu > 1e-9) { vmc = r.vm_c_pu; vac = r.va_c_deg; }
      }
    }
    if (vma <= 1e-9) vma = 1.0;
    if (vmb <= 1e-9) vmb = 1.0;
    if (vmc <= 1e-9) vmc = 1.0;
    vph1[i] = {std::polar(vma, vaa * kDeg2Rad), std::polar(vmb, vab * kDeg2Rad),
               std::polar(vmc, vac * kDeg2Rad)};
  }
  std::vector<double> vdc0(ndc, 1.0);
  for (int k = 0; k < ndc; ++k)
    vdc0[k] = (std::abs(dc.buses[k].vm_pu) > 1e-9) ? dc.buses[k].vm_pu : 1.0;

  // Filter the harmonic orders.
  std::vector<int> hac, rdc;
  for (int h : opt.ac_orders) if (h > 1) hac.push_back(h);
  for (int r : opt.dc_orders) if (r > 0) rdc.push_back(r);
  const int n_h = static_cast<int>(hac.size());
  const int n_r = static_cast<int>(rdc.size());
  const int Nac = 3 * nac;                 // AC nodes per order
  const int dc_off = n_h * Nac;            // DC block offset
  const int M = n_h * Nac + n_r * ndc;     // total unknowns
  if (M == 0) { res.message = "no harmonic orders requested"; res.ok = true; return res; }
  auto ac_idx = [&](int ai, int p, int ph) { return ai * Nac + p * 3 + ph; };
  auto dc_idx = [&](int di, int k) { return dc_off + di * ndc + k; };
  auto h_index = [&](int h) { for (int i = 0; i < n_h; ++i) if (hac[i] == h) return i; return -1; };
  auto r_index = [&](int r) { for (int i = 0; i < n_r; ++i) if (rdc[i] == r) return i; return -1; };

  // Resolve NIC operating points and DC grid-forming port positions.
  struct NICOp { ThreePhaseHybridNIC cfg; int ac_p{-1}, dc_p{-1}; double iref{0}, iph{0}, idc0{0}; };
  std::vector<NICOp> nops;
  std::vector<int> dc_forming;
  for (const auto& nic : inputs.nics) {
    NICOp no; no.cfg = nic;
    no.ac_p = ac_pos(nic.bus_ac);
    no.dc_p = dc_pos(nic.bus_dc);
    if (no.ac_p >= 0) {
      Cx sphi((nic.s_ac_p_mw / 3.0) / ac_base, (nic.s_ac_q_mvar / 3.0) / ac_base);
      Cx va1 = vph1[no.ac_p][0];
      Cx ia1 = (std::abs(va1) > 1e-9) ? std::conj(sphi) / std::conj(va1) : Cx(0, 0);
      no.iref = std::abs(ia1);
      no.iph = std::arg(ia1);
    }
    if (no.dc_p >= 0) {
      no.idc0 = (std::abs(vdc0[no.dc_p]) > 1e-9) ? (nic.p_dc_mw / dc_base) / vdc0[no.dc_p] : 0.0;
      if (nic.dc_port == PortBehavior::GridForming) dc_forming.push_back(no.dc_p);
    }
    if (no.cfg.ac_spectrum.empty()) no.cfg.ac_spectrum = default_six_pulse_ac_spectrum();
    if (no.cfg.dc_spectrum.empty()) no.cfg.dc_spectrum = default_dc_ripple_spectrum();
    nops.push_back(std::move(no));
  }

  // ── Assemble the combined sparse system ──
  std::vector<Trip> T;
  Eigen::VectorXcd I = Eigen::VectorXcd::Zero(M);

  // AC diagonal blocks per order.
  for (int ai = 0; ai < n_h; ++ai) {
    const int h = hac[ai];
    SpMat Yac = build_3ph_ac_ybus(ac, vph1, ac_id2pos, h, opt);
    for (int kk = 0; kk < Yac.outerSize(); ++kk)
      for (SpMat::InnerIterator it(Yac, kk); it; ++it)
        T.emplace_back(ai * Nac + (int)it.row(), ai * Nac + (int)it.col(), it.value());
    // NIC AC-side Norton output admittance + spectrum injection.
    const int seq = harmonic_sequence_of_order(h);
    Cx rb, rc; balanced_phase_set(Cx(1, 0), seq, rb, rc);
    for (const auto& no : nops) {
      if (no.ac_p < 0) continue;
      if (std::abs(no.cfg.y_out_ac) > 0) {
        Cx yo(no.cfg.y_out_ac.real(), static_cast<double>(h) * no.cfg.y_out_ac.imag());
        for (int ph = 0; ph < 3; ++ph)
          T.emplace_back(ac_idx(ai, no.ac_p, ph), ac_idx(ai, no.ac_p, ph), yo);
      }
      for (const auto& line : no.cfg.ac_spectrum) {
        if (line.order != h) continue;
        double mag = (line.mag_percent / 100.0) * no.iref;
        Cx ia = std::polar(mag, line.phase_deg * kDeg2Rad + no.iph);
        I(ac_idx(ai, no.ac_p, 0)) += ia;
        I(ac_idx(ai, no.ac_p, 1)) += ia * rb;
        I(ac_idx(ai, no.ac_p, 2)) += ia * rc;
      }
    }
    // AC sources.
    for (const auto& src : inputs.ac_sources) {
      int p = ac_pos(src.bus);
      if (p < 0) continue;
      for (const auto& line : src.spectrum) {
        if (line.order != h) continue;
        if (src.balanced) {
          double mag = (line.mag_percent / 100.0) * src.i_base_pu;
          Cx ia = std::polar(mag, line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
          I(ac_idx(ai, p, 0)) += ia;
          I(ac_idx(ai, p, 1)) += ia * rb;
          I(ac_idx(ai, p, 2)) += ia * rc;
        } else {
          const double ib[3] = {src.i_base_pu_a, src.i_base_pu_b, src.i_base_pu_c};
          for (int ph = 0; ph < 3; ++ph)
            I(ac_idx(ai, p, ph)) += std::polar((line.mag_percent / 100.0) * ib[ph],
                                               line.phase_deg * kDeg2Rad);
        }
      }
    }
  }

  // DC diagonal blocks per order.
  for (int di = 0; di < n_r; ++di) {
    const int r = rdc[di];
    SpMat Ydc = build_dc_ybus(dc, dc_id2pos, dc_forming, r, opt);
    for (int kk = 0; kk < Ydc.outerSize(); ++kk)
      for (SpMat::InnerIterator it(Ydc, kk); it; ++it)
        T.emplace_back(dc_off + di * ndc + (int)it.row(), dc_off + di * ndc + (int)it.col(), it.value());
    for (const auto& no : nops) {
      if (no.dc_p < 0) continue;
      if (std::abs(no.cfg.y_out_dc) > 0)
        T.emplace_back(dc_idx(di, no.dc_p), dc_idx(di, no.dc_p), no.cfg.y_out_dc);
      for (const auto& line : no.cfg.dc_spectrum) {
        if (line.order != r) continue;
        double mag = (line.mag_percent / 100.0) * std::abs(no.idc0);
        I(dc_idx(di, no.dc_p)) += std::polar(mag, line.phase_deg * kDeg2Rad);
      }
    }
    for (const auto& src : inputs.dc_sources) {
      if (!src.is_dc) continue;
      int k = dc_pos(src.bus);
      if (k < 0) continue;
      for (const auto& line : src.spectrum) {
        if (line.order != r) continue;
        I(dc_idx(di, k)) += std::polar((line.mag_percent / 100.0) * src.i_base_pu,
                                       line.phase_deg * kDeg2Rad);
      }
    }
  }

  // NIC bidirectional cross-coupling at characteristic pairs |h - r| == 1.
  //   AC current row gets  -k_ad · V_dc(r)      (balanced by AC order sequence)
  //   DC current row gets  -k_da · V_ac,a(h)
  for (const auto& no : nops) {
    if (no.ac_p < 0 || no.dc_p < 0) continue;
    const bool has_ad = std::abs(no.cfg.k_ad) > 0;
    const bool has_da = std::abs(no.cfg.k_da) > 0;
    if (!has_ad && !has_da) continue;
    for (int ai = 0; ai < n_h; ++ai) {
      const int h = hac[ai];
      for (int di = 0; di < n_r; ++di) {
        const int r = rdc[di];
        if (std::abs(h - r) != 1) continue;
        if (has_ad) {
          const int seq = harmonic_sequence_of_order(h);
          Cx rb, rc; balanced_phase_set(Cx(1, 0), seq, rb, rc);
          const Cx rot[3] = {Cx(1, 0), rb, rc};
          for (int ph = 0; ph < 3; ++ph)
            T.emplace_back(ac_idx(ai, no.ac_p, ph), dc_idx(di, no.dc_p), -no.cfg.k_ad * rot[ph]);
        }
        if (has_da)
          T.emplace_back(dc_idx(di, no.dc_p), ac_idx(ai, no.ac_p, 0), -no.cfg.k_da);
      }
    }
  }

  SpMat A(M, M);
  A.setFromTriplets(T.begin(), T.end());
  A.makeCompressed();
  Eigen::SparseLU<SpMat> lu;
  lu.compute(A);
  if (lu.info() != Eigen::Success) {
    res.message = "combined AC/DC factorisation failed";
    return res;
  }
  Eigen::VectorXcd V = lu.solve(I);
  if (lu.info() != Eigen::Success) {
    res.message = "combined AC/DC solve failed";
    return res;
  }
  res.combined_solved = true;

  // ── Distribute the solution ──
  res.ac_orders = hac;
  res.dc_orders = rdc;
  res.ac_bus_results.resize(nac);
  for (int i = 0; i < nac; ++i) {
    auto& br = res.ac_bus_results[i];
    br.bus = ac.buses[i].index;
    br.phase_mask = ac.buses[i].phase_mask;
    br.v_fund_pu_a = std::abs(vph1[i][0]);
    br.v_fund_pu_b = std::abs(vph1[i][1]);
    br.v_fund_pu_c = std::abs(vph1[i][2]);
    br.v_by_order_a[1] = vph1[i][0];
    br.v_by_order_b[1] = vph1[i][1];
    br.v_by_order_c[1] = vph1[i][2];
    for (int ai = 0; ai < n_h; ++ai) {
      br.v_by_order_a[hac[ai]] = V(ac_idx(ai, i, 0));
      br.v_by_order_b[hac[ai]] = V(ac_idx(ai, i, 1));
      br.v_by_order_c[hac[ai]] = V(ac_idx(ai, i, 2));
    }
    br.thd_a_pct = thd_from_orders(br.v_by_order_a, 1, br.v_fund_pu_a);
    br.thd_b_pct = thd_from_orders(br.v_by_order_b, 1, br.v_fund_pu_b);
    br.thd_c_pct = thd_from_orders(br.v_by_order_c, 1, br.v_fund_pu_c);
    double mx = std::max({br.thd_a_pct, br.thd_b_pct, br.thd_c_pct});
    if (mx > res.max_ac_thd_pct) { res.max_ac_thd_pct = mx; res.max_ac_thd_bus = br.bus; }
  }
  res.dc_bus_results.resize(ndc);
  for (int k = 0; k < ndc; ++k) {
    auto& br = res.dc_bus_results[k];
    br.bus = dc.buses[k].index;
    br.is_dc = true;
    br.v_fund_pu = std::abs(vdc0[k]);
    br.v_by_order[0] = Cx(vdc0[k], 0.0);
    for (int di = 0; di < n_r; ++di) br.v_by_order[rdc[di]] = V(dc_idx(di, k));
    br.thd_pct = thd_from_orders(br.v_by_order, 0, br.v_fund_pu);
    if (br.thd_pct > res.max_dc_thd_pct) { res.max_dc_thd_pct = br.thd_pct; res.max_dc_thd_bus = br.bus; }
  }
  res.ok = true;
  return res;
}

// ═══════════════════════════════════════════════════════════════════════════
// Newton-Raphson harmonic power flow (nonlinear resources)
// ═══════════════════════════════════════════════════════════════════════════
std::string HPFNewtonResult::summary() const {
  std::ostringstream os;
  os << "Newton harmonic power flow: " << (ok ? "OK" : "FAILED");
  if (!converged) os << " (not converged)";
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  AC orders: " << ac_orders.size()
     << ", max Newton iterations: " << max_iterations_used;
  os << "\n  Max AC THD: " << max_ac_thd_pct << " % at bus " << max_ac_thd_bus;
  return os.str();
}

HPFNewtonResult solve_harmonic_power_flow_newton(
    const HybridPowerSystem& sys,
    const std::vector<HarmonicNonlinearSource>& nonlinear,
    const HarmonicStudyInputs& linear_inputs, const HPFOptions& opt) {
  HPFNewtonResult res;
  const int n = static_cast<int>(sys.ac.buses.size());
  if (n == 0) { res.message = "system has no AC buses"; return res; }
  const auto ac_id2pos = build_id_map(sys.ac.buses);
  auto ac_pos = [&](int id) { auto it = ac_id2pos.find(id); return it == ac_id2pos.end() ? -1 : it->second; };

  OperatingPoint op = extract_operating_point(sys, opt);

  std::vector<HarmonicBusResult> ac_res(n);
  for (int i = 0; i < n; ++i) {
    ac_res[i].bus = sys.ac.buses[i].index;
    ac_res[i].is_dc = false;
    ac_res[i].v_fund_pu = std::abs(op.vac1[i]);
    ac_res[i].v_by_order[1] = op.vac1[i];
  }

  res.converged = true;
  for (int h : opt.ac_orders) {
    if (h <= 1) continue;
    SpMat Y = build_ac_ybus(sys, op, ac_id2pos, h, opt);

    // Linear source currents (HarmonicCurrentSource) + nonlinear i_src / y_out / g2.
    Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(n);
    std::vector<Cx> g2(n, Cx(0.0, 0.0));
    for (const auto& src : linear_inputs.sources) {
      if (src.is_dc) continue;
      int i = ac_pos(src.bus);
      if (i < 0) continue;
      for (const auto& line : src.spectrum)
        if (line.order == h)
          Isrc(i) += std::polar((line.mag_percent / 100.0) * src.i_base_pu,
                                line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
    }
    for (const auto& nl : nonlinear) {
      int i = ac_pos(nl.bus);
      if (i < 0) continue;
      auto iy = nl.y_out.find(h);
      if (iy != nl.y_out.end()) Y.coeffRef(i, i) += iy->second;
      auto is = nl.i_src.find(h);
      if (is != nl.i_src.end()) Isrc(i) += is->second;
      auto ig = nl.g2.find(h);
      if (ig != nl.g2.end()) g2[i] += ig->second;
    }
    Y.makeCompressed();

    // Initial guess: the linear (one-shot) solution.
    Eigen::SparseLU<SpMat> lu0;
    lu0.compute(Y);
    if (lu0.info() != Eigen::Success) {
      res.converged = false;
      res.final_residual[h] = -1.0;
      continue;
    }
    Eigen::VectorXcd V = lu0.solve(Isrc);

    // Newton iteration on r(V) = Y·V − Isrc + g2∘V².
    int it = 0;
    double rn = 0.0;
    bool order_ok = true;
    for (; it < opt.newton_max_iter; ++it) {
      Eigen::VectorXcd r = Y * V - Isrc;
      for (int i = 0; i < n; ++i)
        if (std::abs(g2[i]) > 0) r(i) += g2[i] * V(i) * V(i);
      rn = 0.0;
      for (int i = 0; i < n; ++i) rn = std::max(rn, std::abs(r(i)));
      if (rn < opt.newton_tol) break;
      SpMat J = Y;
      for (int i = 0; i < n; ++i)
        if (std::abs(g2[i]) > 0) J.coeffRef(i, i) += 2.0 * g2[i] * V(i);
      J.makeCompressed();
      Eigen::SparseLU<SpMat> lu;
      lu.compute(J);
      if (lu.info() != Eigen::Success) { order_ok = false; break; }
      V += lu.solve(-r);
    }
    res.iterations[h] = it;
    res.final_residual[h] = rn;
    res.max_iterations_used = std::max(res.max_iterations_used, it);
    if (!order_ok || rn >= opt.newton_tol) res.converged = false;
    res.ac_orders.push_back(h);
    for (int i = 0; i < n; ++i) ac_res[i].v_by_order[h] = V(i);
  }

  for (auto& r : ac_res) {
    r.thd_pct = thd_from_orders(r.v_by_order, 1, r.v_fund_pu);
    if (r.thd_pct > res.max_ac_thd_pct) {
      res.max_ac_thd_pct = r.thd_pct;
      res.max_ac_thd_bus = r.bus;
    }
  }
  res.ac_bus_results = std::move(ac_res);
  res.ok = true;
  return res;
}

// ── Three-phase (abc-domain) Newton-Raphson harmonic power flow ──
HPF3phResult solve_harmonic_power_flow_3ph_newton(
    const ThreePhaseACSystem& sys,
    const std::vector<ThreePhaseNonlinearSource>& nonlinear,
    const ThreePhaseHarmonicInputs& linear_inputs, const HPFOptions& opt) {
  HPF3phResult res;
  const int n = static_cast<int>(sys.buses.size());
  if (n == 0) { res.message = "system has no buses"; return res; }
  std::unordered_map<int, int> id2pos;
  for (int i = 0; i < n; ++i) id2pos[sys.buses[i].index] = i;
  auto pos = [&](int id) { auto it = id2pos.find(id); return it == id2pos.end() ? -1 : it->second; };
  auto node = [](int p, int ph) { return p * 3 + ph; };

  // Operating point: per-phase fundamental phasors.
  std::vector<std::array<Cx, 3>> vph1(n);
  powerflow::ThreePhaseFlowResult tp;
  bool have_pf = false;
  if (opt.run_base_power_flow) {
    try { tp = powerflow::solve_three_phase(sys, {}); have_pf = tp.converged; }
    catch (...) { have_pf = false; }
  }
  res.base_pf_converged = have_pf;
  std::unordered_map<int, int> pfpos;
  if (have_pf)
    for (int i = 0; i < (int)tp.bus_results.size(); ++i) pfpos[tp.bus_results[i].bus_id] = i;
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    double vma = b.vm_a_pu, vaa = b.va_a_deg, vmb = b.vm_b_pu, vab = b.va_b_deg,
           vmc = b.vm_c_pu, vac = b.va_c_deg;
    if (have_pf) {
      auto it = pfpos.find(b.index);
      if (it != pfpos.end()) {
        const auto& r = tp.bus_results[it->second];
        if (r.vm_a_pu > 1e-9) { vma = r.vm_a_pu; vaa = r.va_a_deg; }
        if (r.vm_b_pu > 1e-9) { vmb = r.vm_b_pu; vab = r.va_b_deg; }
        if (r.vm_c_pu > 1e-9) { vmc = r.vm_c_pu; vac = r.va_c_deg; }
      }
    }
    if (vma <= 1e-9) vma = 1.0;
    if (vmb <= 1e-9) vmb = 1.0;
    if (vmc <= 1e-9) vmc = 1.0;
    vph1[i] = {std::polar(vma, vaa * kDeg2Rad), std::polar(vmb, vab * kDeg2Rad),
               std::polar(vmc, vac * kDeg2Rad)};
  }

  std::vector<ThreePhaseHarmonicBusResult> bus_res(n);
  for (int i = 0; i < n; ++i) {
    auto& br = bus_res[i];
    br.bus = sys.buses[i].index;
    br.phase_mask = sys.buses[i].phase_mask;
    br.v_fund_pu_a = std::abs(vph1[i][0]);
    br.v_fund_pu_b = std::abs(vph1[i][1]);
    br.v_fund_pu_c = std::abs(vph1[i][2]);
    br.v_by_order_a[1] = vph1[i][0];
    br.v_by_order_b[1] = vph1[i][1];
    br.v_by_order_c[1] = vph1[i][2];
  }

  res.newton_converged = true;
  for (int h : opt.ac_orders) {
    if (h <= 1) continue;
    SpMat Y = build_3ph_ac_ybus(sys, vph1, id2pos, h, opt);
    Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(3 * n);
    std::vector<Cx> g2(3 * n, Cx(0.0, 0.0));
    const int seq = harmonic_sequence_of_order(h);

    // Linear three-phase harmonic sources (balanced sequence-rotated / per-phase).
    for (const auto& src : linear_inputs.sources) {
      int p = pos(src.bus);
      if (p < 0) continue;
      for (const auto& line : src.spectrum) {
        if (line.order != h) continue;
        if (src.balanced) {
          double mag = (line.mag_percent / 100.0) * src.i_base_pu;
          Cx ia = std::polar(mag, line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
          Cx ib, ic; balanced_phase_set(ia, seq, ib, ic);
          if (sys.buses[p].phase_mask.has(0)) Isrc(node(p, 0)) += ia;
          if (sys.buses[p].phase_mask.has(1)) Isrc(node(p, 1)) += ib;
          if (sys.buses[p].phase_mask.has(2)) Isrc(node(p, 2)) += ic;
        } else {
          const double ib_[3] = {src.i_base_pu_a, src.i_base_pu_b, src.i_base_pu_c};
          for (int ph = 0; ph < 3; ++ph)
            if (sys.buses[p].phase_mask.has(ph))
              Isrc(node(p, ph)) += std::polar((line.mag_percent / 100.0) * ib_[ph],
                                              line.phase_deg * kDeg2Rad);
        }
      }
    }

    // Nonlinear resources: fold y_out into Y, i_src into RHS, collect g2.
    for (const auto& nl : nonlinear) {
      int p = pos(nl.bus);
      if (p < 0) continue;
      auto iy = nl.y_out.find(h);
      if (iy != nl.y_out.end())
        for (int ph = 0; ph < 3; ++ph) Y.coeffRef(node(p, ph), node(p, ph)) += iy->second[ph];
      auto is = nl.i_src.find(h);
      if (is != nl.i_src.end())
        for (int ph = 0; ph < 3; ++ph) Isrc(node(p, ph)) += is->second[ph];
      auto ig = nl.g2.find(h);
      if (ig != nl.g2.end())
        for (int ph = 0; ph < 3; ++ph) g2[node(p, ph)] += ig->second[ph];
    }
    Y.makeCompressed();

    Eigen::SparseLU<SpMat> lu0;
    lu0.compute(Y);
    if (lu0.info() != Eigen::Success) {
      res.newton_converged = false;
      res.ac_order_solved[h] = false;
      continue;
    }
    Eigen::VectorXcd V = lu0.solve(Isrc);

    int it = 0;
    double rn = 0.0;
    bool order_ok = true;
    for (; it < opt.newton_max_iter; ++it) {
      Eigen::VectorXcd r = Y * V - Isrc;
      for (int i = 0; i < 3 * n; ++i)
        if (std::abs(g2[i]) > 0) r(i) += g2[i] * V(i) * V(i);
      rn = 0.0;
      for (int i = 0; i < 3 * n; ++i) rn = std::max(rn, std::abs(r(i)));
      if (rn < opt.newton_tol) break;
      SpMat J = Y;
      for (int i = 0; i < 3 * n; ++i)
        if (std::abs(g2[i]) > 0) J.coeffRef(i, i) += 2.0 * g2[i] * V(i);
      J.makeCompressed();
      Eigen::SparseLU<SpMat> lu;
      lu.compute(J);
      if (lu.info() != Eigen::Success) { order_ok = false; break; }
      V += lu.solve(-r);
    }
    res.newton_iterations[h] = it;
    res.newton_residual[h] = rn;
    res.max_newton_iterations = std::max(res.max_newton_iterations, it);
    res.ac_order_solved[h] = order_ok && (rn < opt.newton_tol);
    if (!order_ok || rn >= opt.newton_tol) res.newton_converged = false;
    res.ac_orders.push_back(h);
    for (int i = 0; i < n; ++i) {
      bus_res[i].v_by_order_a[h] = V(node(i, 0));
      bus_res[i].v_by_order_b[h] = V(node(i, 1));
      bus_res[i].v_by_order_c[h] = V(node(i, 2));
    }
  }

  for (auto& r : bus_res) {
    r.thd_a_pct = thd_from_orders(r.v_by_order_a, 1, r.v_fund_pu_a);
    r.thd_b_pct = thd_from_orders(r.v_by_order_b, 1, r.v_fund_pu_b);
    r.thd_c_pct = thd_from_orders(r.v_by_order_c, 1, r.v_fund_pu_c);
    double mx = std::max({r.thd_a_pct, r.thd_b_pct, r.thd_c_pct});
    if (mx > res.max_thd_pct) { res.max_thd_pct = mx; res.max_thd_bus = r.bus; }
  }
  res.bus_results = std::move(bus_res);
  res.ok = true;
  return res;
}

// ── Real/imaginary (2N) Newton: non-holomorphic constant-power harmonic loads ──
HPFNewtonResult solve_harmonic_power_flow_newton_real(
    const HybridPowerSystem& sys,
    const std::vector<ConstantPowerHarmonicLoad>& cp_loads,
    const HarmonicStudyInputs& linear_inputs, const HPFOptions& opt) {
  using SpR = Eigen::SparseMatrix<double>;
  using TripR = Eigen::Triplet<double>;
  HPFNewtonResult res;
  const int n = static_cast<int>(sys.ac.buses.size());
  if (n == 0) { res.message = "system has no AC buses"; return res; }
  const auto id2pos = build_id_map(sys.ac.buses);
  auto pos = [&](int id) { auto it = id2pos.find(id); return it == id2pos.end() ? -1 : it->second; };
  OperatingPoint op = extract_operating_point(sys, opt);

  std::vector<HarmonicBusResult> ac_res(n);
  for (int i = 0; i < n; ++i) {
    ac_res[i].bus = sys.ac.buses[i].index;
    ac_res[i].is_dc = false;
    ac_res[i].v_fund_pu = std::abs(op.vac1[i]);
    ac_res[i].v_by_order[1] = op.vac1[i];
  }

  res.converged = true;
  for (int h : opt.ac_orders) {
    if (h <= 1) continue;
    SpMat Y = build_ac_ybus(sys, op, id2pos, h, opt);
    Y.makeCompressed();

    Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(n);
    for (const auto& src : linear_inputs.sources) {
      if (src.is_dc) continue;
      int i = pos(src.bus);
      if (i < 0) continue;
      for (const auto& line : src.spectrum)
        if (line.order == h)
          Isrc(i) += std::polar((line.mag_percent / 100.0) * src.i_base_pu,
                                line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
    }
    // c = conj(S) per bus for the constant-power term  Î = −conj(S)/conj(V),
    // so the residual carries  +c/conj(V).
    std::vector<Cx> cps(n, Cx(0.0, 0.0));
    for (const auto& cp : cp_loads) {
      int i = pos(cp.bus);
      if (i < 0) continue;
      auto it = cp.s_set.find(h);
      if (it != cp.s_set.end()) cps[i] += std::conj(it->second);
    }

    Eigen::SparseLU<SpMat> lu0;
    lu0.compute(Y);
    if (lu0.info() != Eigen::Success) {
      res.converged = false;
      res.final_residual[h] = -1.0;
      continue;
    }
    Eigen::VectorXcd V = lu0.solve(Isrc);

    // Constant real network Jacobian block  [[G, −B], [B, G]]  from Y = G + jB.
    std::vector<TripR> baseT;
    baseT.reserve(static_cast<size_t>(Y.nonZeros()) * 4);
    for (int k = 0; k < Y.outerSize(); ++k)
      for (SpMat::InnerIterator it(Y, k); it; ++it) {
        int i = static_cast<int>(it.row()), j = static_cast<int>(it.col());
        double g = it.value().real(), b = it.value().imag();
        baseT.emplace_back(i, j, g);
        baseT.emplace_back(n + i, n + j, g);
        baseT.emplace_back(i, n + j, -b);
        baseT.emplace_back(n + i, j, b);
      }

    int iter = 0;
    double rn = 0.0;
    bool ok = true;
    for (; iter < opt.newton_max_iter; ++iter) {
      // Residual Δ = Y·V − Isrc + Σ_cp conj(S)/conj(V).
      Eigen::VectorXcd D = Y * V - Isrc;
      for (int i = 0; i < n; ++i)
        if (std::abs(cps[i]) > 0) {
          Cx cv = std::conj(V(i));
          if (std::abs(cv) < 1e-12) cv = Cx(1e-12, 0.0);
          D(i) += cps[i] / cv;
        }
      rn = 0.0;
      for (int i = 0; i < n; ++i) rn = std::max(rn, std::abs(D(i)));
      if (rn < opt.newton_tol) break;

      std::vector<TripR> T = baseT;
      for (int i = 0; i < n; ++i)
        if (std::abs(cps[i]) > 0) {
          // d/dV of  f = c·(a + jb)/D ,  D = a² + b² ,  c = cr + j·ci.
          double a = V(i).real(), b = V(i).imag();
          double Dn = a * a + b * b;
          if (Dn < 1e-24) Dn = 1e-24;
          double cr = cps[i].real(), ci = cps[i].imag();
          double ReNum = cr * a - ci * b;   // = Re(f)·D
          double ImNum = cr * b + ci * a;   // = Im(f)·D
          double dRe_da = (cr * Dn - ReNum * 2.0 * a) / (Dn * Dn);
          double dRe_db = (-ci * Dn - ReNum * 2.0 * b) / (Dn * Dn);
          double dIm_da = (ci * Dn - ImNum * 2.0 * a) / (Dn * Dn);
          double dIm_db = (cr * Dn - ImNum * 2.0 * b) / (Dn * Dn);
          T.emplace_back(i, i, dRe_da);
          T.emplace_back(i, n + i, dRe_db);
          T.emplace_back(n + i, i, dIm_da);
          T.emplace_back(n + i, n + i, dIm_db);
        }
      SpR J(2 * n, 2 * n);
      J.setFromTriplets(T.begin(), T.end());
      J.makeCompressed();
      Eigen::VectorXd rr(2 * n);
      for (int i = 0; i < n; ++i) { rr(i) = D(i).real(); rr(n + i) = D(i).imag(); }
      Eigen::SparseLU<SpR> lu;
      lu.compute(J);
      if (lu.info() != Eigen::Success) { ok = false; break; }
      Eigen::VectorXd dx = lu.solve(-rr);
      if (lu.info() != Eigen::Success) { ok = false; break; }
      for (int i = 0; i < n; ++i) V(i) += Cx(dx(i), dx(n + i));
    }
    res.iterations[h] = iter;
    res.final_residual[h] = rn;
    res.max_iterations_used = std::max(res.max_iterations_used, iter);
    if (!ok || rn >= opt.newton_tol) res.converged = false;
    res.ac_orders.push_back(h);
    for (int i = 0; i < n; ++i) ac_res[i].v_by_order[h] = V(i);
  }

  for (auto& r : ac_res) {
    r.thd_pct = thd_from_orders(r.v_by_order, 1, r.v_fund_pu);
    if (r.thd_pct > res.max_ac_thd_pct) {
      res.max_ac_thd_pct = r.thd_pct;
      res.max_ac_thd_bus = r.bus;
    }
  }
  res.ac_bus_results = std::move(ac_res);
  res.ok = true;
  return res;
}

// ── Stacked all-orders Newton: cross-order (frequency-mixing) nonlinear resources ──
HPFNewtonResult solve_harmonic_power_flow_newton_coupled(
    const HybridPowerSystem& sys,
    const std::vector<CrossOrderNonlinearSource>& resources,
    const HarmonicStudyInputs& linear_inputs, const HPFOptions& opt) {
  HPFNewtonResult res;
  const int n = static_cast<int>(sys.ac.buses.size());
  if (n == 0) { res.message = "system has no AC buses"; return res; }
  const auto id2pos = build_id_map(sys.ac.buses);
  auto pos = [&](int id) { auto it = id2pos.find(id); return it == id2pos.end() ? -1 : it->second; };
  OperatingPoint op = extract_operating_point(sys, opt);

  std::vector<int> orders;
  for (int h : opt.ac_orders) if (h > 1) orders.push_back(h);
  const int no = static_cast<int>(orders.size());
  std::unordered_map<int, int> oidx;
  for (int oi = 0; oi < no; ++oi) oidx[orders[oi]] = oi;
  const int M = no * n;

  std::vector<HarmonicBusResult> ac_res(n);
  for (int i = 0; i < n; ++i) {
    ac_res[i].bus = sys.ac.buses[i].index;
    ac_res[i].is_dc = false;
    ac_res[i].v_fund_pu = std::abs(op.vac1[i]);
    ac_res[i].v_by_order[1] = op.vac1[i];
  }
  if (M == 0) { res.ac_bus_results = std::move(ac_res); res.ok = true; return res; }
  auto gidx = [&](int oi, int i) { return oi * n + i; };

  // Block-diagonal stacked admittance + stacked source vector.
  std::vector<Trip> baseT;
  Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(M);
  for (int oi = 0; oi < no; ++oi) {
    const int h = orders[oi];
    SpMat Y = build_ac_ybus(sys, op, id2pos, h, opt);
    for (int k = 0; k < Y.outerSize(); ++k)
      for (SpMat::InnerIterator it(Y, k); it; ++it)
        baseT.emplace_back(oi * n + (int)it.row(), oi * n + (int)it.col(), it.value());
    for (const auto& src : linear_inputs.sources) {
      if (src.is_dc) continue;
      int i = pos(src.bus);
      if (i < 0) continue;
      for (const auto& line : src.spectrum)
        if (line.order == h)
          Isrc(gidx(oi, i)) += std::polar((line.mag_percent / 100.0) * src.i_base_pu,
                                          line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
    }
    for (const auto& r : resources) {
      int i = pos(r.bus);
      if (i < 0) continue;
      auto it = r.i_src.find(h);
      if (it != r.i_src.end()) Isrc(gidx(oi, i)) += it->second;
    }
  }
  SpMat Ybase(M, M);
  Ybase.setFromTriplets(baseT.begin(), baseT.end());
  Ybase.makeCompressed();
  Eigen::SparseLU<SpMat> lu0;
  lu0.compute(Ybase);
  if (lu0.info() != Eigen::Success) {
    res.converged = false;
    res.message = "stacked factorisation failed";
    return res;
  }
  Eigen::VectorXcd V = lu0.solve(Isrc);

  // Resolve mixing terms whose three orders are all in the study set.
  struct MT { int oh, op, oq, bus_i; Cx k; };
  std::vector<MT> mts;
  for (const auto& r : resources) {
    int i = pos(r.bus);
    if (i < 0) continue;
    for (const auto& m : r.mixing) {
      auto a = oidx.find(m.out_order), b = oidx.find(m.p_order), c = oidx.find(m.q_order);
      if (a == oidx.end() || b == oidx.end() || c == oidx.end()) continue;
      mts.push_back({a->second, b->second, c->second, i, m.coeff});
    }
  }

  int iter = 0;
  double rn = 0.0;
  bool ok = true;
  for (; iter < opt.newton_max_iter; ++iter) {
    Eigen::VectorXcd D = Ybase * V - Isrc;
    for (const auto& m : mts)
      D(gidx(m.oh, m.bus_i)) += m.k * V(gidx(m.op, m.bus_i)) * V(gidx(m.oq, m.bus_i));
    rn = 0.0;
    for (int i = 0; i < M; ++i) rn = std::max(rn, std::abs(D(i)));
    if (rn < opt.newton_tol) break;
    std::vector<Trip> T = baseT;
    for (const auto& m : mts) {
      const int rh = gidx(m.oh, m.bus_i);
      Cx vp = V(gidx(m.op, m.bus_i)), vq = V(gidx(m.oq, m.bus_i));
      // ∂(k·Vp·Vq)/∂Vp = k·Vq ; ∂/∂Vq = k·Vp (summed when p == q).
      T.emplace_back(rh, gidx(m.op, m.bus_i), m.k * vq);
      T.emplace_back(rh, gidx(m.oq, m.bus_i), m.k * vp);
    }
    SpMat J(M, M);
    J.setFromTriplets(T.begin(), T.end());
    J.makeCompressed();
    Eigen::SparseLU<SpMat> lu;
    lu.compute(J);
    if (lu.info() != Eigen::Success) { ok = false; break; }
    V += lu.solve(-D);
  }
  res.iterations[0] = iter;       // single coupled solve (0 = global key)
  res.final_residual[0] = rn;
  res.max_iterations_used = iter;
  res.converged = ok && rn < opt.newton_tol;
  for (int oi = 0; oi < no; ++oi) {
    res.ac_orders.push_back(orders[oi]);
    for (int i = 0; i < n; ++i) ac_res[i].v_by_order[orders[oi]] = V(gidx(oi, i));
  }
  for (auto& r : ac_res) {
    r.thd_pct = thd_from_orders(r.v_by_order, 1, r.v_fund_pu);
    if (r.thd_pct > res.max_ac_thd_pct) {
      res.max_ac_thd_pct = r.thd_pct;
      res.max_ac_thd_bus = r.bus;
    }
  }
  res.ac_bus_results = std::move(ac_res);
  res.ok = true;
  return res;
}

// ═══════════════════════════════════════════════════════════════════════════
// Frequency scan / resonance analysis
// ═══════════════════════════════════════════════════════════════════════════
namespace {
// Local-extremum resonance detection on a |Z|(f) trace.
void detect_resonances(const std::vector<double>& zmag, const std::vector<double>& freqs,
                       int bus, int sequence, double min_pu,
                       std::vector<HarmonicResonance>& out) {
  for (size_t i = 1; i + 1 < zmag.size(); ++i) {
    if (zmag[i] > zmag[i - 1] && zmag[i] > zmag[i + 1] && zmag[i] >= min_pu)
      out.push_back({bus, freqs[i], zmag[i], true, sequence});
    else if (zmag[i] < zmag[i - 1] && zmag[i] < zmag[i + 1])
      out.push_back({bus, freqs[i], zmag[i], false, sequence});
  }
}
}  // namespace

std::string FrequencyScanResult::summary() const {
  std::ostringstream os;
  os << "Frequency scan: " << (ok ? "OK" : "FAILED");
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  " << freqs.size() << " frequency points, " << z_mag.size()
     << " bus(es), " << resonances.size() << " resonance(s)";
  return os.str();
}

FrequencyScanResult frequency_scan(const HybridPowerSystem& sys,
                                   const FrequencyScanOptions& sopt,
                                   const HPFOptions& opt) {
  FrequencyScanResult res;
  const int n = static_cast<int>(sys.ac.buses.size());
  if (n == 0) { res.message = "system has no AC buses"; return res; }
  const auto id2pos = build_id_map(sys.ac.buses);
  OperatingPoint op = extract_operating_point(sys, opt);

  std::vector<int> buses = sopt.buses;
  if (buses.empty())
    for (const auto& b : sys.ac.buses)
      if (b.in_service) buses.push_back(b.index);
  std::vector<int> kpos;
  for (int bus : buses) {
    auto it = id2pos.find(bus);
    if (it != id2pos.end()) { kpos.push_back(it->second); res.z_mag[bus]; res.z_ang_deg[bus]; }
  }

  const double step = (sopt.f_step > 1e-9) ? sopt.f_step : 0.1;
  for (double f = sopt.f_start; f <= sopt.f_end + 1e-9; f += step) res.freqs.push_back(f);

  for (double f : res.freqs) {
    SpMat Y = build_ac_ybus(sys, op, id2pos, f, opt);
    Eigen::SparseLU<SpMat> lu;
    lu.compute(Y);
    const bool ok = (lu.info() == Eigen::Success);
    for (size_t bi = 0; bi < buses.size(); ++bi) {
      double zm = 0.0, za = 0.0;
      if (ok) {
        Eigen::VectorXcd e = Eigen::VectorXcd::Zero(n);
        e(kpos[bi]) = Cx(1.0, 0.0);
        Eigen::VectorXcd V = lu.solve(e);
        if (lu.info() == Eigen::Success) {
          Cx Z = V(kpos[bi]);
          zm = std::abs(Z);
          za = std::arg(Z) * 180.0 / M_PI;
        }
      }
      res.z_mag[buses[bi]].push_back(zm);
      res.z_ang_deg[buses[bi]].push_back(za);
    }
  }

  if (sopt.detect_resonances)
    for (int bus : buses)
      detect_resonances(res.z_mag.at(bus), res.freqs, bus, -1, sopt.resonance_min_pu,
                        res.resonances);
  res.ok = true;
  return res;
}

std::string SequenceScanResult::summary() const {
  std::ostringstream os;
  os << "Sequence frequency scan at bus " << bus << ": " << (ok ? "OK" : "FAILED");
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  " << freqs.size() << " points, " << resonances.size() << " resonance(s)";
  return os.str();
}

SequenceScanResult sequence_frequency_scan(const ThreePhaseACSystem& sys, int bus,
                                           const FrequencyScanOptions& sopt,
                                           const HPFOptions& opt) {
  SequenceScanResult res;
  res.bus = bus;
  const int n = static_cast<int>(sys.buses.size());
  if (n == 0) { res.message = "system has no buses"; return res; }
  std::unordered_map<int, int> id2pos;
  for (int i = 0; i < n; ++i) id2pos[sys.buses[i].index] = i;
  auto it = id2pos.find(bus);
  if (it == id2pos.end()) { res.message = "bus not found"; return res; }
  const int k = it->second;

  // Operating point (for load impedances): stored phase voltages.
  std::vector<std::array<Cx, 3>> vph1(n);
  for (int i = 0; i < n; ++i) {
    const auto& b = sys.buses[i];
    double vma = b.vm_a_pu > 1e-9 ? b.vm_a_pu : 1.0;
    double vmb = b.vm_b_pu > 1e-9 ? b.vm_b_pu : 1.0;
    double vmc = b.vm_c_pu > 1e-9 ? b.vm_c_pu : 1.0;
    vph1[i] = {std::polar(vma, b.va_a_deg * kDeg2Rad),
               std::polar(vmb, b.va_b_deg * kDeg2Rad),
               std::polar(vmc, b.va_c_deg * kDeg2Rad)};
  }

  const double step = (sopt.f_step > 1e-9) ? sopt.f_step : 0.1;
  for (double f = sopt.f_start; f <= sopt.f_end + 1e-9; f += step) res.freqs.push_back(f);

  const Cx a = std::polar(1.0, 2.0 * M_PI / 3.0);
  const Cx a2 = std::polar(1.0, 4.0 * M_PI / 3.0);
  auto node = [](int p, int ph) { return p * 3 + ph; };

  for (double f : res.freqs) {
    SpMat Y = build_3ph_ac_ybus(sys, vph1, id2pos, f, opt);
    Eigen::SparseLU<SpMat> lu;
    lu.compute(Y);
    double z1 = 0.0, z2 = 0.0, z0 = 0.0;
    if (lu.info() == Eigen::Success) {
      // Positive sequence: inject I_abc = [1, a², a]; read V1 = (Va + a·Vb + a²·Vc)/3.
      Eigen::VectorXcd ep = Eigen::VectorXcd::Zero(3 * n);
      ep(node(k, 0)) = Cx(1, 0); ep(node(k, 1)) = a2; ep(node(k, 2)) = a;
      Eigen::VectorXcd Vp = lu.solve(ep);
      if (lu.info() == Eigen::Success)
        z1 = std::abs((Vp(node(k, 0)) + a * Vp(node(k, 1)) + a2 * Vp(node(k, 2))) / 3.0);
      // Negative sequence: I_abc = [1, a, a²]; V2 = (Va + a²·Vb + a·Vc)/3.
      Eigen::VectorXcd en = Eigen::VectorXcd::Zero(3 * n);
      en(node(k, 0)) = Cx(1, 0); en(node(k, 1)) = a; en(node(k, 2)) = a2;
      Eigen::VectorXcd Vn = lu.solve(en);
      if (lu.info() == Eigen::Success)
        z2 = std::abs((Vn(node(k, 0)) + a2 * Vn(node(k, 1)) + a * Vn(node(k, 2))) / 3.0);
      // Zero sequence: I_abc = [1, 1, 1]; V0 = (Va + Vb + Vc)/3.
      Eigen::VectorXcd e0 = Eigen::VectorXcd::Zero(3 * n);
      e0(node(k, 0)) = Cx(1, 0); e0(node(k, 1)) = Cx(1, 0); e0(node(k, 2)) = Cx(1, 0);
      Eigen::VectorXcd V0 = lu.solve(e0);
      if (lu.info() == Eigen::Success)
        z0 = std::abs((V0(node(k, 0)) + V0(node(k, 1)) + V0(node(k, 2))) / 3.0);
    }
    res.z1_mag.push_back(z1);
    res.z2_mag.push_back(z2);
    res.z0_mag.push_back(z0);
  }

  if (sopt.detect_resonances) {
    detect_resonances(res.z1_mag, res.freqs, bus, 1, sopt.resonance_min_pu, res.resonances);
    detect_resonances(res.z2_mag, res.freqs, bus, 2, sopt.resonance_min_pu, res.resonances);
    detect_resonances(res.z0_mag, res.freqs, bus, 0, sopt.resonance_min_pu, res.resonances);
  }
  res.ok = true;
  return res;
}

// ═══════════════════════════════════════════════════════════════════════════
// Three-phase Newton variants: non-holomorphic (constant-power) and cross-order
// ═══════════════════════════════════════════════════════════════════════════
namespace {
// Assemble the per-phase linear three-phase source injection at order h.
void assemble_3ph_linear_injection(const ThreePhaseACSystem& sys,
                                   const ThreePhaseHarmonicInputs& in, int h,
                                   const std::unordered_map<int, int>& id2pos,
                                   Eigen::VectorXcd& I, int block_off) {
  auto pos = [&](int id) { auto it = id2pos.find(id); return it == id2pos.end() ? -1 : it->second; };
  auto node = [](int p, int ph) { return p * 3 + ph; };
  const int seq = harmonic_sequence_of_order(h);
  for (const auto& src : in.sources) {
    int p = pos(src.bus);
    if (p < 0) continue;
    for (const auto& line : src.spectrum) {
      if (line.order != h) continue;
      if (src.balanced) {
        double mag = (line.mag_percent / 100.0) * src.i_base_pu;
        Cx ia = std::polar(mag, line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
        Cx ib, ic; balanced_phase_set(ia, seq, ib, ic);
        if (sys.buses[p].phase_mask.has(0)) I(block_off + node(p, 0)) += ia;
        if (sys.buses[p].phase_mask.has(1)) I(block_off + node(p, 1)) += ib;
        if (sys.buses[p].phase_mask.has(2)) I(block_off + node(p, 2)) += ic;
      } else {
        const double ib_[3] = {src.i_base_pu_a, src.i_base_pu_b, src.i_base_pu_c};
        for (int ph = 0; ph < 3; ++ph)
          if (sys.buses[p].phase_mask.has(ph))
            I(block_off + node(p, ph)) += std::polar((line.mag_percent / 100.0) * ib_[ph],
                                                     line.phase_deg * kDeg2Rad);
      }
    }
  }
}
}  // namespace

HPF3phResult solve_harmonic_power_flow_3ph_newton_real(
    const ThreePhaseACSystem& sys,
    const std::vector<ThreePhaseConstantPowerLoad>& cp_loads,
    const ThreePhaseHarmonicInputs& linear_inputs, const HPFOptions& opt) {
  using SpR = Eigen::SparseMatrix<double>;
  using TripR = Eigen::Triplet<double>;
  HPF3phResult res;
  const int n = static_cast<int>(sys.buses.size());
  if (n == 0) { res.message = "system has no buses"; return res; }
  std::unordered_map<int, int> id2pos;
  for (int i = 0; i < n; ++i) id2pos[sys.buses[i].index] = i;
  auto pos = [&](int id) { auto it = id2pos.find(id); return it == id2pos.end() ? -1 : it->second; };
  auto node = [](int p, int ph) { return p * 3 + ph; };
  std::vector<std::array<Cx, 3>> vph1;
  res.base_pf_converged = extract_3ph_vph1(sys, opt, vph1);

  std::vector<ThreePhaseHarmonicBusResult> bus_res(n);
  for (int i = 0; i < n; ++i) {
    auto& br = bus_res[i];
    br.bus = sys.buses[i].index;
    br.phase_mask = sys.buses[i].phase_mask;
    br.v_fund_pu_a = std::abs(vph1[i][0]);
    br.v_fund_pu_b = std::abs(vph1[i][1]);
    br.v_fund_pu_c = std::abs(vph1[i][2]);
    br.v_by_order_a[1] = vph1[i][0];
    br.v_by_order_b[1] = vph1[i][1];
    br.v_by_order_c[1] = vph1[i][2];
  }

  res.newton_converged = true;
  for (int h : opt.ac_orders) {
    if (h <= 1) continue;
    SpMat Y = build_3ph_ac_ybus(sys, vph1, id2pos, h, opt);
    Y.makeCompressed();
    const int N3 = 3 * n;
    Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(N3);
    assemble_3ph_linear_injection(sys, linear_inputs, h, id2pos, Isrc, 0);
    std::vector<Cx> cps(N3, Cx(0.0, 0.0));
    for (const auto& cp : cp_loads) {
      int p = pos(cp.bus);
      if (p < 0) continue;
      auto it = cp.s_set.find(h);
      if (it != cp.s_set.end())
        for (int ph = 0; ph < 3; ++ph) cps[node(p, ph)] += std::conj(it->second[ph]);
    }

    Eigen::SparseLU<SpMat> lu0;
    lu0.compute(Y);
    if (lu0.info() != Eigen::Success) {
      res.newton_converged = false;
      res.ac_order_solved[h] = false;
      continue;
    }
    Eigen::VectorXcd V = lu0.solve(Isrc);

    std::vector<TripR> baseT;
    baseT.reserve(static_cast<size_t>(Y.nonZeros()) * 4);
    for (int k = 0; k < Y.outerSize(); ++k)
      for (SpMat::InnerIterator it(Y, k); it; ++it) {
        int i = (int)it.row(), j = (int)it.col();
        double g = it.value().real(), b = it.value().imag();
        baseT.emplace_back(i, j, g);
        baseT.emplace_back(N3 + i, N3 + j, g);
        baseT.emplace_back(i, N3 + j, -b);
        baseT.emplace_back(N3 + i, j, b);
      }

    int iter = 0;
    double rn = 0.0;
    bool ok = true;
    for (; iter < opt.newton_max_iter; ++iter) {
      Eigen::VectorXcd D = Y * V - Isrc;
      for (int i = 0; i < N3; ++i)
        if (std::abs(cps[i]) > 0) {
          Cx cv = std::conj(V(i));
          if (std::abs(cv) < 1e-12) cv = Cx(1e-12, 0.0);
          D(i) += cps[i] / cv;
        }
      rn = 0.0;
      for (int i = 0; i < N3; ++i) rn = std::max(rn, std::abs(D(i)));
      if (rn < opt.newton_tol) break;
      std::vector<TripR> T = baseT;
      for (int i = 0; i < N3; ++i)
        if (std::abs(cps[i]) > 0) {
          double a = V(i).real(), b = V(i).imag();
          double Dn = a * a + b * b;
          if (Dn < 1e-24) Dn = 1e-24;
          double cr = cps[i].real(), ci = cps[i].imag();
          double ReN = cr * a - ci * b, ImN = cr * b + ci * a;
          T.emplace_back(i, i, (cr * Dn - ReN * 2 * a) / (Dn * Dn));
          T.emplace_back(i, N3 + i, (-ci * Dn - ReN * 2 * b) / (Dn * Dn));
          T.emplace_back(N3 + i, i, (ci * Dn - ImN * 2 * a) / (Dn * Dn));
          T.emplace_back(N3 + i, N3 + i, (cr * Dn - ImN * 2 * b) / (Dn * Dn));
        }
      SpR J(2 * N3, 2 * N3);
      J.setFromTriplets(T.begin(), T.end());
      J.makeCompressed();
      Eigen::VectorXd rr(2 * N3);
      for (int i = 0; i < N3; ++i) { rr(i) = D(i).real(); rr(N3 + i) = D(i).imag(); }
      Eigen::SparseLU<SpR> lu;
      lu.compute(J);
      if (lu.info() != Eigen::Success) { ok = false; break; }
      Eigen::VectorXd dx = lu.solve(-rr);
      if (lu.info() != Eigen::Success) { ok = false; break; }
      for (int i = 0; i < N3; ++i) V(i) += Cx(dx(i), dx(N3 + i));
    }
    res.newton_iterations[h] = iter;
    res.newton_residual[h] = rn;
    res.max_newton_iterations = std::max(res.max_newton_iterations, iter);
    res.ac_order_solved[h] = ok && (rn < opt.newton_tol);
    if (!ok || rn >= opt.newton_tol) res.newton_converged = false;
    res.ac_orders.push_back(h);
    for (int i = 0; i < n; ++i) {
      bus_res[i].v_by_order_a[h] = V(node(i, 0));
      bus_res[i].v_by_order_b[h] = V(node(i, 1));
      bus_res[i].v_by_order_c[h] = V(node(i, 2));
    }
  }
  for (auto& r : bus_res) {
    r.thd_a_pct = thd_from_orders(r.v_by_order_a, 1, r.v_fund_pu_a);
    r.thd_b_pct = thd_from_orders(r.v_by_order_b, 1, r.v_fund_pu_b);
    r.thd_c_pct = thd_from_orders(r.v_by_order_c, 1, r.v_fund_pu_c);
    double mx = std::max({r.thd_a_pct, r.thd_b_pct, r.thd_c_pct});
    if (mx > res.max_thd_pct) { res.max_thd_pct = mx; res.max_thd_bus = r.bus; }
  }
  res.bus_results = std::move(bus_res);
  res.ok = true;
  return res;
}

HPF3phResult solve_harmonic_power_flow_3ph_newton_coupled(
    const ThreePhaseACSystem& sys,
    const std::vector<ThreePhaseCrossOrderSource>& resources,
    const ThreePhaseHarmonicInputs& linear_inputs, const HPFOptions& opt) {
  HPF3phResult res;
  const int n = static_cast<int>(sys.buses.size());
  if (n == 0) { res.message = "system has no buses"; return res; }
  std::unordered_map<int, int> id2pos;
  for (int i = 0; i < n; ++i) id2pos[sys.buses[i].index] = i;
  auto pos = [&](int id) { auto it = id2pos.find(id); return it == id2pos.end() ? -1 : it->second; };
  auto node = [](int p, int ph) { return p * 3 + ph; };
  std::vector<std::array<Cx, 3>> vph1;
  res.base_pf_converged = extract_3ph_vph1(sys, opt, vph1);

  std::vector<int> orders;
  for (int h : opt.ac_orders) if (h > 1) orders.push_back(h);
  const int no = static_cast<int>(orders.size());
  std::unordered_map<int, int> oidx;
  for (int oi = 0; oi < no; ++oi) oidx[orders[oi]] = oi;
  const int N3 = 3 * n;
  const int M = no * N3;

  std::vector<ThreePhaseHarmonicBusResult> bus_res(n);
  for (int i = 0; i < n; ++i) {
    auto& br = bus_res[i];
    br.bus = sys.buses[i].index;
    br.phase_mask = sys.buses[i].phase_mask;
    br.v_fund_pu_a = std::abs(vph1[i][0]);
    br.v_fund_pu_b = std::abs(vph1[i][1]);
    br.v_fund_pu_c = std::abs(vph1[i][2]);
    br.v_by_order_a[1] = vph1[i][0];
    br.v_by_order_b[1] = vph1[i][1];
    br.v_by_order_c[1] = vph1[i][2];
  }
  if (M == 0) { res.bus_results = std::move(bus_res); res.ok = true; return res; }
  auto gidx = [&](int oi, int p, int ph) { return oi * N3 + node(p, ph); };

  std::vector<Trip> baseT;
  Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(M);
  for (int oi = 0; oi < no; ++oi) {
    const int h = orders[oi];
    SpMat Y = build_3ph_ac_ybus(sys, vph1, id2pos, h, opt);
    for (int k = 0; k < Y.outerSize(); ++k)
      for (SpMat::InnerIterator it(Y, k); it; ++it)
        baseT.emplace_back(oi * N3 + (int)it.row(), oi * N3 + (int)it.col(), it.value());
    assemble_3ph_linear_injection(sys, linear_inputs, h, id2pos, Isrc, oi * N3);
    for (const auto& r : resources) {
      int p = pos(r.bus);
      if (p < 0) continue;
      auto it = r.i_src.find(h);
      if (it != r.i_src.end())
        for (int ph = 0; ph < 3; ++ph) Isrc(gidx(oi, p, ph)) += it->second[ph];
    }
  }
  SpMat Ybase(M, M);
  Ybase.setFromTriplets(baseT.begin(), baseT.end());
  Ybase.makeCompressed();
  Eigen::SparseLU<SpMat> lu0;
  lu0.compute(Ybase);
  if (lu0.info() != Eigen::Success) {
    res.newton_converged = false;
    res.message = "stacked factorisation failed";
    return res;
  }
  Eigen::VectorXcd V = lu0.solve(Isrc);

  struct MT { int oh, op, oq, bus_p, ph; Cx k; };
  std::vector<MT> mts;
  for (const auto& r : resources) {
    int p = pos(r.bus);
    if (p < 0) continue;
    for (const auto& m : r.mixing) {
      auto a = oidx.find(m.out_order), b = oidx.find(m.p_order), c = oidx.find(m.q_order);
      if (a == oidx.end() || b == oidx.end() || c == oidx.end()) continue;
      if (m.phase < 0)
        for (int ph = 0; ph < 3; ++ph) mts.push_back({a->second, b->second, c->second, p, ph, m.coeff});
      else
        mts.push_back({a->second, b->second, c->second, p, m.phase, m.coeff});
    }
  }

  int iter = 0;
  double rn = 0.0;
  bool ok = true;
  for (; iter < opt.newton_max_iter; ++iter) {
    Eigen::VectorXcd D = Ybase * V - Isrc;
    for (const auto& m : mts)
      D(gidx(m.oh, m.bus_p, m.ph)) += m.k * V(gidx(m.op, m.bus_p, m.ph)) * V(gidx(m.oq, m.bus_p, m.ph));
    rn = 0.0;
    for (int i = 0; i < M; ++i) rn = std::max(rn, std::abs(D(i)));
    if (rn < opt.newton_tol) break;
    std::vector<Trip> T = baseT;
    for (const auto& m : mts) {
      const int rh = gidx(m.oh, m.bus_p, m.ph);
      Cx vp = V(gidx(m.op, m.bus_p, m.ph)), vq = V(gidx(m.oq, m.bus_p, m.ph));
      T.emplace_back(rh, gidx(m.op, m.bus_p, m.ph), m.k * vq);
      T.emplace_back(rh, gidx(m.oq, m.bus_p, m.ph), m.k * vp);
    }
    SpMat J(M, M);
    J.setFromTriplets(T.begin(), T.end());
    J.makeCompressed();
    Eigen::SparseLU<SpMat> lu;
    lu.compute(J);
    if (lu.info() != Eigen::Success) { ok = false; break; }
    V += lu.solve(-D);
  }
  res.newton_iterations[0] = iter;
  res.newton_residual[0] = rn;
  res.max_newton_iterations = iter;
  res.newton_converged = ok && rn < opt.newton_tol;
  for (int oi = 0; oi < no; ++oi) {
    res.ac_orders.push_back(orders[oi]);
    for (int i = 0; i < n; ++i) {
      bus_res[i].v_by_order_a[orders[oi]] = V(gidx(oi, i, 0));
      bus_res[i].v_by_order_b[orders[oi]] = V(gidx(oi, i, 1));
      bus_res[i].v_by_order_c[orders[oi]] = V(gidx(oi, i, 2));
    }
  }
  for (auto& r : bus_res) {
    r.thd_a_pct = thd_from_orders(r.v_by_order_a, 1, r.v_fund_pu_a);
    r.thd_b_pct = thd_from_orders(r.v_by_order_b, 1, r.v_fund_pu_b);
    r.thd_c_pct = thd_from_orders(r.v_by_order_c, 1, r.v_fund_pu_c);
    double mx = std::max({r.thd_a_pct, r.thd_b_pct, r.thd_c_pct});
    if (mx > res.max_thd_pct) { res.max_thd_pct = mx; res.max_thd_bus = r.bus; }
  }
  res.bus_results = std::move(bus_res);
  res.ok = true;
  return res;
}

// ═══════════════════════════════════════════════════════════════════════════
// Harmonic metrics: losses, K-factor, current THD / TDD
// ═══════════════════════════════════════════════════════════════════════════
std::string HarmonicMetricsResult::summary() const {
  std::ostringstream os;
  os << "Harmonic metrics: " << ac_branches.size() << " branch(es)";
  os << "\n  total loss " << total_loss_pu << " pu (" << total_harmonic_loss_pu
     << " pu harmonic, " << harmonic_loss_fraction * 100.0 << " %)";
  os << "\n  max K-factor " << max_k_factor << " at branch " << max_k_factor_branch
     << ", max current THD " << max_thd_i_pct << " %, max TDD " << max_tdd_pct << " %";
  return os.str();
}

HarmonicMetricsResult harmonic_metrics(const HybridPowerSystem& sys,
                                       const HPFResult& result,
                                       const HarmonicMetricsOptions& mopt,
                                       const HPFOptions& opt) {
  HarmonicMetricsResult res;
  std::unordered_map<int, const HarmonicBusResult*> busmap;
  for (const auto& br : result.ac_bus_results) busmap[br.bus] = &br;

  std::vector<int> orders{1};
  for (int h : result.ac_orders) orders.push_back(h);

  double tot_loss = 0.0, tot_hloss = 0.0;
  for (const auto& br : sys.ac.branches) {
    if (!br.in_service) continue;
    auto itf = busmap.find(br.from_bus), itt = busmap.find(br.to_bus);
    if (itf == busmap.end() || itt == busmap.end()) continue;
    const auto& vf = itf->second->v_by_order;
    const auto& vt = itt->second->v_by_order;

    BranchHarmonicMetrics m;
    m.from_bus = br.from_bus;
    m.to_bus = br.to_bus;
    double sum_i2 = 0.0, sum_h2i2 = 0.0, sum_harm_i2 = 0.0, loss = 0.0, hloss = 0.0;
    for (int h : orders) {
      auto a = vf.find(h), b = vt.find(h);
      if (a == vf.end() || b == vt.end()) continue;
      const double hh = static_cast<double>(h);
      const double R = skin_r(br.r_pu, hh, opt);
      Cx z(R, hh * br.x_pu);
      Cx ys = (std::abs(z) > 1e-12) ? (Cx(1.0, 0.0) / z) : Cx(0.0, 0.0);
      const double ih = std::abs(ys * (a->second - b->second));
      const double ih2 = ih * ih;
      sum_i2 += ih2;
      sum_h2i2 += hh * hh * ih2;
      loss += ih2 * R;
      if (h == 1) {
        m.i_fund_pu = ih;
      } else {
        sum_harm_i2 += ih2;
        hloss += ih2 * R;
      }
    }
    m.i_rms_pu = std::sqrt(sum_i2);
    m.thd_i_pct = (m.i_fund_pu > 1e-12) ? std::sqrt(sum_harm_i2) / m.i_fund_pu * 100.0 : 0.0;
    const double idem = (mopt.i_demand_pu > 1e-12) ? mopt.i_demand_pu : m.i_fund_pu;
    m.tdd_pct = (idem > 1e-12) ? std::sqrt(sum_harm_i2) / idem * 100.0 : 0.0;
    m.k_factor = (sum_i2 > 1e-12) ? sum_h2i2 / sum_i2 : 1.0;
    m.p_loss_pu = loss;
    m.p_loss_harmonic_pu = hloss;
    tot_loss += loss;
    tot_hloss += hloss;

    if (m.k_factor > res.max_k_factor) { res.max_k_factor = m.k_factor; res.max_k_factor_branch = br.index; }
    if (m.thd_i_pct > res.max_thd_i_pct) res.max_thd_i_pct = m.thd_i_pct;
    if (m.tdd_pct > res.max_tdd_pct) res.max_tdd_pct = m.tdd_pct;
    res.ac_branches.push_back(m);
  }
  res.total_loss_pu = tot_loss;
  res.total_harmonic_loss_pu = tot_hloss;
  res.harmonic_loss_fraction = (tot_loss > 1e-12) ? tot_hloss / tot_loss : 0.0;
  return res;
}

// ═══════════════════════════════════════════════════════════════════════════
// Hybrid AC + DC Newton with bilinear NIC cross-domain coupling
// ═══════════════════════════════════════════════════════════════════════════
std::string HPFHybridNewtonResult::summary() const {
  std::ostringstream os;
  os << "Hybrid AC/DC Newton: " << (ok ? "OK" : "FAILED");
  if (!converged) os << " (not converged)";
  if (!message.empty()) os << " (" << message << ")";
  os << "\n  AC orders " << ac_orders.size() << ", DC orders " << dc_orders.size()
     << ", iterations " << iterations << ", residual " << final_residual;
  os << "\n  Max AC THD " << max_ac_thd_pct << " %, max DC THD " << max_dc_thd_pct << " %";
  return os.str();
}

HPFHybridNewtonResult solve_harmonic_power_flow_hybrid_newton(
    const HybridPowerSystem& sys, const HybridNewtonInputs& inputs,
    const HPFOptions& opt) {
  HPFHybridNewtonResult res;
  const int nac = static_cast<int>(sys.ac.buses.size());
  const int ndc = static_cast<int>(sys.dc.buses.size());
  if (nac == 0 && ndc == 0) { res.message = "system has no buses"; return res; }

  const auto ac_id2pos = build_id_map(sys.ac.buses);
  std::unordered_map<int, int> dc_id2pos;
  for (int k = 0; k < ndc; ++k) dc_id2pos[sys.dc.buses[k].index] = k;
  auto ac_pos = [&](int id) { auto it = ac_id2pos.find(id); return it == ac_id2pos.end() ? -1 : it->second; };
  auto dc_pos = [&](int id) { auto it = dc_id2pos.find(id); return it == dc_id2pos.end() ? -1 : it->second; };

  OperatingPoint op = extract_operating_point(sys, opt);
  std::vector<double> vdc0(ndc, 1.0);
  for (int k = 0; k < ndc; ++k)
    vdc0[k] = (std::abs(sys.dc.buses[k].vm_pu) > 1e-9) ? sys.dc.buses[k].vm_pu : 1.0;

  std::vector<int> hac, rdc;
  for (int h : opt.ac_orders) if (h > 1) hac.push_back(h);
  for (int r : opt.dc_orders) if (r > 0) rdc.push_back(r);
  const int nh = static_cast<int>(hac.size()), nr = static_cast<int>(rdc.size());
  std::unordered_map<int, int> ac_oidx, dc_oidx;
  for (int i = 0; i < nh; ++i) ac_oidx[hac[i]] = i;
  for (int i = 0; i < nr; ++i) dc_oidx[rdc[i]] = i;
  const int dc_off = nh * nac;
  const int M = nh * nac + nr * ndc;

  // Per-bus result accumulators (fundamental / steady reference).
  res.ac_bus_results.resize(nac);
  for (int i = 0; i < nac; ++i) {
    res.ac_bus_results[i].bus = sys.ac.buses[i].index;
    res.ac_bus_results[i].is_dc = false;
    res.ac_bus_results[i].v_fund_pu = std::abs(op.vac1[i]);
    res.ac_bus_results[i].v_by_order[1] = op.vac1[i];
  }
  res.dc_bus_results.resize(ndc);
  for (int k = 0; k < ndc; ++k) {
    res.dc_bus_results[k].bus = sys.dc.buses[k].index;
    res.dc_bus_results[k].is_dc = true;
    res.dc_bus_results[k].v_fund_pu = std::abs(vdc0[k]);
    res.dc_bus_results[k].v_by_order[0] = Cx(vdc0[k], 0.0);
  }
  if (M == 0) { res.ok = true; res.converged = true; return res; }

  auto gid = [&](bool is_dc, int order, int bus) -> int {
    if (is_dc) {
      auto o = dc_oidx.find(order);
      int p = dc_pos(bus);
      if (o == dc_oidx.end() || p < 0) return -1;
      return dc_off + o->second * ndc + p;
    }
    auto o = ac_oidx.find(order);
    int p = ac_pos(bus);
    if (o == ac_oidx.end() || p < 0) return -1;
    return o->second * nac + p;
  };

  // Block-diagonal stacked admittance + linear source vector.
  std::vector<Trip> baseT;
  Eigen::VectorXcd Isrc = Eigen::VectorXcd::Zero(M);
  for (int oi = 0; oi < nh; ++oi) {
    const int h = hac[oi];
    SpMat Y = build_ac_ybus(sys, op, ac_id2pos, h, opt);
    for (int k = 0; k < Y.outerSize(); ++k)
      for (SpMat::InnerIterator it(Y, k); it; ++it)
        baseT.emplace_back(oi * nac + (int)it.row(), oi * nac + (int)it.col(), it.value());
  }
  std::vector<int> dc_forming;  // grounding via DC_V buses inside build_dc_ybus
  for (int oi = 0; oi < nr; ++oi) {
    const int r = rdc[oi];
    SpMat Y = build_dc_ybus(sys.dc, dc_id2pos, dc_forming, r, opt);
    for (int k = 0; k < Y.outerSize(); ++k)
      for (SpMat::InnerIterator it(Y, k); it; ++it)
        baseT.emplace_back(dc_off + oi * ndc + (int)it.row(),
                           dc_off + oi * ndc + (int)it.col(), it.value());
  }
  for (const auto& src : inputs.sources) {
    for (const auto& line : src.spectrum) {
      int g = gid(src.is_dc, line.order, src.bus);
      if (g < 0) continue;
      Isrc(g) += std::polar((line.mag_percent / 100.0) * src.i_base_pu,
                            line.phase_deg * kDeg2Rad + src.i_base_phase_deg * kDeg2Rad);
    }
  }

  SpMat Ybase(M, M);
  Ybase.setFromTriplets(baseT.begin(), baseT.end());
  Ybase.makeCompressed();
  Eigen::SparseLU<SpMat> lu0;
  lu0.compute(Ybase);
  if (lu0.info() != Eigen::Success) { res.message = "stacked factorisation failed"; return res; }
  Eigen::VectorXcd V = lu0.solve(Isrc);

  // Resolve bilinear terms to global indices.
  struct BT { int out_g, a_g, b_g; Cx k; };
  std::vector<BT> bts;
  for (const auto& t : inputs.bilinear) {
    int og = gid(t.out_is_dc, t.out_order, t.out_bus);
    int ag = gid(t.a_is_dc, t.a_order, t.a_bus);
    int bg = gid(t.b_is_dc, t.b_order, t.b_bus);
    if (og < 0 || ag < 0 || bg < 0) continue;
    bts.push_back({og, ag, bg, t.coeff});
  }

  int iter = 0;
  double rn = 0.0;
  bool ok = true;
  for (; iter < opt.newton_max_iter; ++iter) {
    Eigen::VectorXcd D = Ybase * V - Isrc;
    for (const auto& t : bts) D(t.out_g) += t.k * V(t.a_g) * V(t.b_g);
    rn = 0.0;
    for (int i = 0; i < M; ++i) rn = std::max(rn, std::abs(D(i)));
    if (rn < opt.newton_tol) break;
    std::vector<Trip> T = baseT;
    for (const auto& t : bts) {
      T.emplace_back(t.out_g, t.a_g, t.k * V(t.b_g));
      T.emplace_back(t.out_g, t.b_g, t.k * V(t.a_g));
    }
    SpMat J(M, M);
    J.setFromTriplets(T.begin(), T.end());
    J.makeCompressed();
    Eigen::SparseLU<SpMat> lu;
    lu.compute(J);
    if (lu.info() != Eigen::Success) { ok = false; break; }
    V += lu.solve(-D);
  }
  res.iterations = iter;
  res.final_residual = rn;
  res.converged = ok && rn < opt.newton_tol;

  res.ac_orders = hac;
  res.dc_orders = rdc;
  for (int oi = 0; oi < nh; ++oi)
    for (int i = 0; i < nac; ++i) res.ac_bus_results[i].v_by_order[hac[oi]] = V(oi * nac + i);
  for (int oi = 0; oi < nr; ++oi)
    for (int k = 0; k < ndc; ++k) res.dc_bus_results[k].v_by_order[rdc[oi]] = V(dc_off + oi * ndc + k);
  for (auto& r : res.ac_bus_results) {
    r.thd_pct = thd_from_orders(r.v_by_order, 1, r.v_fund_pu);
    if (r.thd_pct > res.max_ac_thd_pct) { res.max_ac_thd_pct = r.thd_pct; res.max_ac_thd_bus = r.bus; }
  }
  for (auto& r : res.dc_bus_results) {
    r.thd_pct = thd_from_orders(r.v_by_order, 0, r.v_fund_pu);
    if (r.thd_pct > res.max_dc_thd_pct) { res.max_dc_thd_pct = r.thd_pct; res.max_dc_thd_bus = r.bus; }
  }
  res.ok = true;
  return res;
}

}  // namespace hacdcpf::harmonics
