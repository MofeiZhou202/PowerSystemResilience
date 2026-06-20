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

#include <algorithm>
#include <cmath>
#include <complex>
#include <map>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/api/hacdcpf.hpp"

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

// Series + charging stamp for an AC pi-branch at harmonic order h.
struct BranchStamp {
  Cx yff, yft, ytf, ytt;
};

BranchStamp ac_branch_stamp(double r, double x, double b, double tap,
                            double shift_deg, int h) {
  const double hh = static_cast<double>(h);
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
                    const std::unordered_map<int, int>& id2pos, int h,
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
    BranchStamp s = ac_branch_stamp(br.r_pu, br.x_pu, br.b_pu, br.tap, br.shift_deg, h);
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
    BranchStamp s = ac_branch_stamp(r_pu, x_pu, 0.0, 1.0, 0.0, h);
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
// DC ripple admittance matrix at order r (resistive DC branches only).
// ───────────────────────────────────────────────────────────────────────────
SpMat build_dc_ybus(const HybridPowerSystem& sys,
                    const std::unordered_map<int, int>& id2pos,
                    const std::vector<int>& nic_dc_pos, int /*order*/,
                    const HPFOptions& opt) {
  const int n = static_cast<int>(sys.dc.buses.size());
  std::vector<Trip> trips;
  trips.reserve(sys.dc.branches.size() * 4 + n * 2);
  auto pos = [&](int id) -> int {
    auto it = id2pos.find(id);
    return it == id2pos.end() ? -1 : it->second;
  };

  for (const auto& br : sys.dc.branches) {
    if (!br.in_service) continue;
    int f = pos(br.from_bus), t = pos(br.to_bus);
    if (f < 0 || t < 0) continue;
    double r = std::max(br.r_pu, 1e-6);
    Cx y(1.0 / r, 0.0);
    trips.emplace_back(f, f, y);
    trips.emplace_back(f, t, -y);
    trips.emplace_back(t, f, -y);
    trips.emplace_back(t, t, y);
  }

  // DC voltage-forming nodes ground the ripple network (DC_V buses + NIC DC ports).
  std::vector<Cx> gy(n, Cx(0.0, 0.0));
  double zsrc = std::max(opt.dc_source_impedance_pu, 1e-9);
  for (int i = 0; i < n; ++i) {
    if (sys.dc.buses[i].bus_type == DCBusType::DC_V)
      gy[i] += Cx(1.0 / zsrc, 0.0);
  }
  for (int p : nic_dc_pos)
    if (p >= 0 && p < n) gy[p] += Cx(1.0 / zsrc, 0.0);

  for (int i = 0; i < n; ++i) {
    if (!sys.dc.buses[i].in_service ||
        sys.dc.buses[i].bus_type == DCBusType::DC_ISOLATED)
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
HPFResult solve_harmonic_power_flow(const HybridPowerSystem& sys,
                                    const HarmonicStudyInputs& inputs,
                                    const HPFOptions& opt) {
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
      // A pure PQ converter does not anchor the DC ripple network.
      const bool dc_forms_voltage =
          v.grid_forming || v.control_mode == ConverterMode::VDC_Q ||
          v.control_mode == ConverterMode::VDC_VAC;
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
          if (p_dc == 0.0) p_dc = -v.p_set_mw;  // lossless guess
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
    SpMat Y = build_dc_ybus(sys, dc_id2pos, nic_dc_positions, r, opt);
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
        Cx z(br.r_pu, static_cast<double>(h) * br.x_pu);
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
      double i_fund = 0.0, acc = 0.0;
      std::vector<int> orders{0};
      for (int o : res.dc_orders) orders.push_back(o);
      for (int o : orders) {
        auto itf = dc_res[f].v_by_order.find(o);
        auto itt = dc_res[t].v_by_order.find(o);
        if (itf == dc_res[f].v_by_order.end() || itt == dc_res[t].v_by_order.end())
          continue;
        double im = std::abs((itf->second - itt->second) / r);
        bf.i_by_order[o] = im;
        if (o == 0) i_fund = im; else acc += im * im;
      }
      bf.thd_i_pct = (i_fund > 1e-12) ? std::sqrt(acc) / i_fund * 100.0 : 0.0;
      res.dc_branch_flows.push_back(std::move(bf));
    }
  }

  res.ac_bus_results = std::move(ac_res);
  res.dc_bus_results = std::move(dc_res);
  res.ok = true;
  if (!res.base_pf_converged && opt.run_base_power_flow)
    res.message = "base power flow did not converge; used stored/nominal voltages";
  return res;
}

HPFResult solve_harmonic_power_flow(const HybridPowerSystem& sys,
                                    const HPFOptions& opt) {
  return solve_harmonic_power_flow(sys, HarmonicStudyInputs{}, opt);
}

}  // namespace hacdcpf::harmonics
