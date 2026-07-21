#define _USE_MATH_DEFINES  // M_PI on strict-conformance toolchains (MSYS2 UCRT, MSVC)
// Short Circuit Analysis — Z-bus method
// DistributionPowerFlow.jl ShortCircuit module equivalent (C++20)
//
// Algorithm: for three-phase faults, the fault admittance is added at the
// faulted bus and the bus impedance matrix Z_bus = Y_fault^{-1} is used.
// Z_kk gives the Thevenin impedance; I_f = c·V0 / Z_kk.
//
// For unsymmetrical faults the method of symmetrical components is used
// with a simplified assumption: the negative-sequence network equals the
// positive-sequence network (zero-sequence is not modelled for brevity).

#include "hacdcpf/analysis/short_circuit.hpp"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <complex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/model/components.hpp"
#include "hacdcpf/model/enum_strings.hpp"
#include "hacdcpf/projection/result_attribution.hpp"
#include "hacdcpf/model/device_control_role.hpp"
#include "hacdcpf/model/enums.hpp"
#include "hacdcpf/model/network_utils.hpp"
#include "hacdcpf/projection/canonical_network.hpp"

namespace hacdcpf::analysis {

namespace {

using Cx = std::complex<double>;
using SpMat = Eigen::SparseMatrix<Cx>;
using SpLU  = Eigen::SparseLU<SpMat>;

constexpr double kMinMotorContributionMw = 0.05;

// -------------------------------------------------------------------------
// Build bus-ID → local-index map
// -------------------------------------------------------------------------
std::unordered_map<int, int>
build_id_map(const std::vector<ACBus>& buses) {
  std::unordered_map<int, int> m;
  m.reserve(buses.size());
  for (int i = 0; i < static_cast<int>(buses.size()); ++i)
    m[buses[i].index] = i;
  return m;
}

// All IEC 60909 functions now implemented.

double detailed_voltage_factor(double vn_kv, const SCDetailedOptions& opt) {
  return (opt.c_factor > 0.0) ? opt.c_factor
                              : get_voltage_factor_sc(vn_kv, opt.calc_type);
}

double load_motor_fraction(double raw) {
  if (raw <= 0.0) return 0.0;
  return (raw > 1.0) ? std::clamp(raw / 100.0, 0.0, 1.0)
                     : std::clamp(raw, 0.0, 1.0);
}

bool is_projected_rich_motor_load(const Load& ld) {
  return ld.sc_source_type == "AsynchronousMotor";
}

bool is_ac_grid_forming_converter(const VSCConverter& conv) {
  return resolve_device_control_role(conv).is_ac_grid_forming;
}

double load_motor_active_power_mw(const Load& ld, double motor_fraction) {
  if (is_projected_rich_motor_load(ld)) {
    return std::abs(ld.p_mw);
  }
  if (ld.sn_mva > 1e-6) {
    const double pf = std::clamp(
        std::abs(ld.p_mw) / std::max(1e-9, std::hypot(ld.p_mw, ld.q_mvar)),
        0.0, 1.0);
    return ld.sn_mva * motor_fraction * ((pf > 1e-6) ? pf : 0.85);
  }
  return std::abs(ld.p_mw) * motor_fraction;
}

double load_motor_pn_mw(const Load& ld, double motor_fraction) {
  const double pn = load_motor_active_power_mw(ld, motor_fraction);
  if (is_projected_rich_motor_load(ld)) {
    const double eta = std::clamp(ld.motor_efficiency, 0.01, 1.0);
    return pn * eta;
  }
  return pn;
}

struct ExternalGridScImpedance {
  Cx z1{0.0, 0.0};
  Cx z0{0.0, 0.0};
};

struct TransformerBranchCorrection {
  double k1{1.0};
  double k0{1.0};
};

std::unordered_map<int, TransformerBranchCorrection>
build_transformer_branch_corrections(const ACSystem& ac,
                                     const std::optional<BranchExpandMap>& branch_map,
                                     double base_mva,
                                     const SCDetailedOptions& opt) {
  std::unordered_map<int, TransformerBranchCorrection> corrections;
  if (!branch_map || !opt.apply_iec_transformer_correction) return corrections;

  auto bus_kv = [&](int bus_id, double fallback) {
    for (const auto& bus : ac.buses) {
      if (bus.index == bus_id) return (bus.base_kv > 1e-6) ? bus.base_kv : fallback;
    }
    return fallback;
  };

  auto branch_by_index = [&](int branch_index) -> const ACBranch* {
    for (const auto& br : ac.branches) {
      if (br.index == branch_index) return &br;
    }
    return nullptr;
  };

  auto branch_based_correction = [&](const ACBranch& br) {
    TransformerBranchCorrection corr;
    if (br.sn_mva <= 1e-9) return corr;
    const double vn = (br.vn_hv_kv > 1e-6) ? br.vn_hv_kv
                     : bus_kv(br.from_bus, br.vn_lv_kv);
    const double c = detailed_voltage_factor(vn, opt);
    const double x_on_tr_base = std::abs(br.x_pu) * br.sn_mva / base_mva;
    corr.k1 = 0.95 * c / (1.0 + 0.6 * x_on_tr_base);
    if (std::abs(br.x0_pu) > 1e-12) {
      const double x0_on_tr_base = std::abs(br.x0_pu) * br.sn_mva / base_mva;
      corr.k0 = corr.k1 * (1.0 + 0.6 * x0_on_tr_base);
    } else {
      corr.k0 = corr.k1 * (1.0 + 0.6 * x_on_tr_base);
    }
    return corr;
  };

  for (const auto& entry : branch_map->entries) {
    if (entry.origin_type == BranchOriginType::Transformer2W) {
      const Transformer2W* tr = nullptr;
      for (const auto& candidate : ac.transformers_2w) {
        if (candidate.index == entry.origin_index) {
          tr = &candidate;
          break;
        }
      }
      if (!tr || tr->sn_mva <= 1e-9) continue;

      const double vn = (tr->vn_hv_kv > 1e-6) ? tr->vn_hv_kv
                       : bus_kv(tr->hv_bus, tr->vn_lv_kv);
      auto [r_pu, x_pu] = [&]() {
        const double scale = base_mva / tr->sn_mva;
        const double z = std::max(0.0, tr->vk_percent / 100.0) * scale;
        const double r = std::max(0.0, tr->vkr_percent / 100.0) * scale;
        return std::pair<double, double>{r, std::sqrt(std::max(0.0, z * z - r * r))};
      }();
      const double x_on_tr_base = x_pu * tr->sn_mva / base_mva;
      const double c = detailed_voltage_factor(vn, opt);
      TransformerBranchCorrection corr;
      corr.k1 = 0.95 * c / (1.0 + 0.6 * x_on_tr_base);
      if (tr->z0_percent > 0.0) {
        const double x0_on_tr_base =
            (tr->x0_r0 > 1e-9)
                ? (tr->z0_percent / 100.0) * tr->x0_r0 /
                      std::sqrt(1.0 + tr->x0_r0 * tr->x0_r0)
                : (tr->z0_percent / 100.0);
        corr.k0 = corr.k1 * (1.0 + 0.6 * x0_on_tr_base);
      } else {
        corr.k0 = corr.k1 * (1.0 + 0.6 * x_on_tr_base);
      }
      if (corr.k1 > 1e-9 && corr.k0 > 1e-9) {
        corrections[entry.branch_index] = corr;
      }
    } else if (entry.origin_type == BranchOriginType::Transformer3W) {
      if (const ACBranch* br = branch_by_index(entry.branch_index)) {
        corrections[entry.branch_index] = branch_based_correction(*br);
      }
    }
  }

  return corrections;
}

ExternalGridScImpedance external_grid_impedance_sc(const ExternalGrid& eg,
                                                   double eg_bus_kv,
                                                   double base_mva,
                                                   const SCDetailedOptions& opt) {
  const double c = detailed_voltage_factor(eg_bus_kv, opt);
  Cx z1(0.0, 0.0);

  if (eg.ikq_ka > 1e-6) {
    const double u_nq = (eg.vn_kv > 1e-6) ? eg.vn_kv : eg_bus_kv;
    const double z_ext_ohm = c * u_nq / (std::sqrt(3.0) * eg.ikq_ka);
    const double z_ext_pu = z_ext_ohm * (base_mva / (eg_bus_kv * eg_bus_kv));
    const double xr = (eg.x_r > 1e-6) ? eg.x_r : 10.0;
    const double r_ext = z_ext_pu / std::sqrt(1.0 + xr * xr);
    const double x_ext = r_ext * xr;
    z1 = Cx(r_ext, x_ext);
  } else if (std::abs(eg.r_pu) > 1e-12 || std::abs(eg.x_pu) > 1e-12) {
    z1 = Cx(eg.r_pu, eg.x_pu);
  } else {
    const double s_sc = (opt.calc_type == SCCalcType::Min && eg.s_sc_min_mva > 1e-6)
                            ? eg.s_sc_min_mva
                            : eg.s_sc_max_mva;
    if (s_sc > 1e-6) {
      const double z_ext_pu = c * base_mva / s_sc;
      const double rx = (opt.calc_type == SCCalcType::Min && eg.rx_min > 1e-9)
                            ? eg.rx_min
                            : ((eg.rx_max > 1e-9) ? eg.rx_max : 0.1);
      const double x_ext = z_ext_pu / std::sqrt(1.0 + rx * rx);
      const double r_ext = rx * x_ext;
      z1 = Cx(r_ext, x_ext);
    }
  }

  Cx z0 = (std::abs(eg.r0_pu) > 1e-12 || std::abs(eg.x0_pu) > 1e-12)
              ? Cx(eg.r0_pu, eg.x0_pu)
              : z1;
  return {z1, z0};
}

// -------------------------------------------------------------------------
// Build fault-network admittance matrix (positive sequence)
//
// Y_fault = Y_bus (from branches) + generator sub-transient shunts
//
// Tap-changer branches use the simplified π-model:
//   Y_ff = y/|t|², Y_tt = y,  Y_ft = -y/t*,  Y_tf = -y/t
// Regular lines (tap == 1, shift == 0):
//   standard π model with y_series and y_shunt/2 at each end.
// -------------------------------------------------------------------------
SpMat build_fault_ybus(const ACSystem& ac_sys,
                       const std::unordered_map<int, int>& id_map,
                       int n,
                       const SCOptions& opt) {
  using Trip = Eigen::Triplet<Cx>;
  std::vector<Trip> trips;
  trips.reserve(6 * ac_sys.branches.size() + 2 * ac_sys.generators.size());

  // --- branches ---
  for (const auto& br : ac_sys.branches) {
    if (!br.in_service) continue;
    auto it_f = id_map.find(br.from_bus);
    auto it_t = id_map.find(br.to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;

    // Avoid division by zero in very unusual cases
    if (std::abs(br.r_pu) < 1e-12 && std::abs(br.x_pu) < 1e-12) continue;

    Cx z_series(br.r_pu, br.x_pu);
    Cx y_series = Cx(1.0, 0.0) / z_series;
    Cx y_shunt(0.0, br.b_pu / 2.0);

    double tap = (br.tap > 1e-9) ? br.tap : 1.0;
    double shift_rad = br.shift_deg * M_PI / 180.0;
    Cx t(tap * std::cos(shift_rad), tap * std::sin(shift_rad));

    if (std::abs(t - Cx(1.0, 0.0)) < 1e-9) {
      // Regular line
      trips.emplace_back(fi, fi,  y_series + y_shunt);
      trips.emplace_back(ti, ti,  y_series + y_shunt);
      trips.emplace_back(fi, ti, -y_series);
      trips.emplace_back(ti, fi, -y_series);
    } else {
      // Transformer with tap
      double t2 = std::norm(t);  // |t|²
      trips.emplace_back(fi, fi,  y_series / t2 + y_shunt / t2);
      trips.emplace_back(ti, ti,  y_series + y_shunt);
      trips.emplace_back(fi, ti, -y_series / std::conj(t));
      trips.emplace_back(ti, fi, -y_series / t);
    }
  }

  // --- generator sub-transient shunts (for fault network) ---
  for (const auto& g : ac_sys.generators) {
    if (!g.in_service) continue;
    auto it = id_map.find(g.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;
    double xdpp = (g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp;
    // Y_gen = 1 / (j·x_d'')
    Cx y_gen(0.0, -1.0 / xdpp);   // 1/(jX) = -j/X
    trips.emplace_back(gi, gi, y_gen);
  }

  SpMat Y(n, n);
  Y.setFromTriplets(trips.begin(), trips.end());
  Y.makeCompressed();
  return Y;
}

// -------------------------------------------------------------------------
// Factorise Y_fault once and solve for multiple right-hand sides.
// Returns the diagonal element Z_kk = (Y^{-1})_{kk} for bus k by solving
//   Y · z = e_k
// and reading z[k].
// -------------------------------------------------------------------------
Cx compute_zkk(SpLU& lu, int k, int n) {
  Eigen::VectorXcd e_k = Eigen::VectorXcd::Zero(n);
  e_k[k] = Cx(1.0, 0.0);
  Eigen::VectorXcd z_col = lu.solve(e_k);
  return z_col[k];
}

// -------------------------------------------------------------------------
// Single-bus fault calculation
// -------------------------------------------------------------------------
BusFaultResult fault_at_bus(SpLU& lu,
                             int k, int bus_id,
                             int n,
                             double base_mva,
                             double base_kv,
                             const SCOptions& opt) {
  BusFaultResult r;
  r.bus_id = bus_id;

  Cx Z_kk = compute_zkk(lu, k, n);
  r.z_thevenin = Z_kk;

  // Pre-fault voltage assumed 1.0∠0° p.u. (flat profile, conservative)
  Cx V0(opt.c_factor, 0.0);  // IEC 60909: multiply by c factor

  if (std::abs(Z_kk) < 1e-15) {
    // Degenerate (infinite-bus bus with zero impedance) — leave zeros
    return r;
  }

  if (opt.fault_type == FaultType::ThreePhase) {
    r.i_fault_pu = V0 / Z_kk;
  } else if (opt.fault_type == FaultType::SinglePhaseGround) {
    // Simplified: assumes negative-sequence = positive-sequence
    // Zero-sequence not modelled → use Z1 = Z2 = Z_kk, Z0 = Z_kk (approx)
    Cx Z_total = Z_kk + Z_kk + Z_kk;
    r.i_fault_pu = Cx(3.0, 0.0) * V0 / Z_total;
  } else if (opt.fault_type == FaultType::TwoPhase) {
    Cx Z_total = Z_kk + Z_kk;
    r.i_fault_pu = Cx(1.0, 0.0) * V0 / Z_total;
  } else if (opt.fault_type == FaultType::TwoPhaseGround) {
    // DLG: I_a1 = V0 / (Z1 + Z2||Z0) ≈ V0 / (Z_kk + Z_kk/2) = 2V0/(3Z_kk)
    Cx Z_total = Z_kk + Z_kk * Z_kk / (Z_kk + Z_kk);
    r.i_fault_pu = V0 / Z_total;
  }

  // Short-circuit apparent power on the system base. For a 3-phase fault this
  // is S_k'' = S_base * c / |Z_k|; for unbalanced overview rows it follows the
  // reported fault-current magnitude.
  r.sk_mva = base_mva * std::abs(r.i_fault_pu);
  // Initial symmetrical short-circuit current in kA
  double v_base_kv = (base_kv > 0.0) ? base_kv : 1.0;
  double i_base_ka = base_mva / (std::sqrt(3.0) * v_base_kv);  // [kA]
  r.ikpp_ka  = std::abs(r.i_fault_pu) * i_base_ka;

  return r;
}

// -------------------------------------------------------------------------
// IEC 60909: Build subtransient admittance matrices
//   pos-seq (Ybus), neg-seq (Ybus2), zero-seq (Ybus0)
// -------------------------------------------------------------------------
struct YbusTriplet {
  Eigen::MatrixXcd Ybus;   // positive-sequence dense
  Eigen::MatrixXcd Ybus2;  // negative-sequence dense (differs from Ybus for converters)
  Eigen::MatrixXcd Ybus0;  // zero-sequence dense
};

YbusTriplet build_sc_admittance_matrices(const ACSystem& ac,
                                         const std::vector<VSCConverter>& vsc_converters,
                                         const std::unordered_map<int, int>& id_map,
                                         int n,
                                         const SCDetailedOptions& opt,
                                         const std::unordered_map<int, TransformerBranchCorrection>& transformer_corrections,
                                         bool steady_state,
                                         bool machine_shunts = true) {
  Eigen::MatrixXcd Ybus  = Eigen::MatrixXcd::Zero(n, n);
  Eigen::MatrixXcd Ybus2 = Eigen::MatrixXcd::Zero(n, n);  // negative-sequence
  Eigen::MatrixXcd Ybus0 = Eigen::MatrixXcd::Zero(n, n);

  const double base_mva = ac.base_mva > 0.0 ? ac.base_mva : 100.0;

  // --- branches (lines + transformers-as-branches) ---
  for (const auto& br : ac.branches) {
    if (!br.in_service) continue;
    auto it_f = id_map.find(br.from_bus);
    auto it_t = id_map.find(br.to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;

    if (std::abs(br.r_pu) < 1e-12 && std::abs(br.x_pu) < 1e-12) continue;

    const bool transformer_like =
        (br.sn_mva > 1e-9) && (br.vn_hv_kv > 1e-9) && (br.vn_lv_kv > 1e-9);

    Cx z_series(br.r_pu, br.x_pu);
    if (transformer_like) {
      const double c_tr = detailed_voltage_factor(br.vn_hv_kv, opt);
      const double x_t_rel = std::abs(br.x_pu) *
          (br.sn_mva / std::max(1e-9, base_mva));
      const double k_t = 0.95 * c_tr / (1.0 + 0.6 * x_t_rel);
      z_series *= k_t;

      const double hv_base_kv = (ac.buses[static_cast<size_t>(fi)].base_kv > 1e-9)
                                    ? ac.buses[static_cast<size_t>(fi)].base_kv
                                    : br.vn_hv_kv;
      const double lv_base_kv = (ac.buses[static_cast<size_t>(ti)].base_kv > 1e-9)
                                    ? ac.buses[static_cast<size_t>(ti)].base_kv
                                    : br.vn_lv_kv;
      double k_pu = 1.0;
      if (br.vn_lv_kv > 1e-9 && hv_base_kv > 1e-9 && lv_base_kv > 1e-9) {
        k_pu = (br.vn_hv_kv / br.vn_lv_kv) / (hv_base_kv / lv_base_kv);
      }

      const Cx y = Cx(k_pu, 0.0) / z_series;
      const Cx y_hv = Cx(1.0 - k_pu, 0.0) / z_series;
      const Cx y_lv = Cx(k_pu * (k_pu - 1.0), 0.0) / z_series;

      Ybus(fi, fi) += y + y_hv;
      Ybus(ti, ti) += y + y_lv;
      Ybus(fi, ti) += -y;
      Ybus(ti, fi) += -y;
      Ybus2(fi, fi) += y + y_hv;
      Ybus2(ti, ti) += y + y_lv;
      Ybus2(fi, ti) += -y;
      Ybus2(ti, fi) += -y;
    } else {
      if (auto it_corr = transformer_corrections.find(br.index);
          it_corr != transformer_corrections.end()) {
        z_series *= it_corr->second.k1;
      }
      Cx y_series = Cx(1.0, 0.0) / z_series;
      Cx y_shunt(0.0, br.b_pu / 2.0);

      double tap = (br.tap > 1e-9) ? br.tap : 1.0;
      double shift_rad = br.shift_deg * M_PI / 180.0;
      Cx t(tap * std::cos(shift_rad), tap * std::sin(shift_rad));

    if (std::abs(t - Cx(1.0, 0.0)) < 1e-9) {
      Ybus(fi, fi) += y_series + y_shunt;
      Ybus(ti, ti) += y_series + y_shunt;
      Ybus(fi, ti) += -y_series;
      Ybus(ti, fi) += -y_series;
      // Negative-seq = positive-seq for passive elements
      Ybus2(fi, fi) += y_series + y_shunt;
      Ybus2(ti, ti) += y_series + y_shunt;
      Ybus2(fi, ti) += -y_series;
      Ybus2(ti, fi) += -y_series;
    } else {
      double t2 = std::norm(t);
      Ybus(fi, fi) += y_series / t2 + y_shunt / t2;
      Ybus(ti, ti) += y_series + y_shunt;
      Ybus(fi, ti) += -y_series / std::conj(t);
      Ybus(ti, fi) += -y_series / t;
      Ybus2(fi, fi) += y_series / t2 + y_shunt / t2;
      Ybus2(ti, ti) += y_series + y_shunt;
      Ybus2(fi, ti) += -y_series / std::conj(t);
      Ybus2(ti, fi) += -y_series / t;
    }

    // Zero-sequence branch
    if (std::abs(br.r0_pu) > 1e-12 || std::abs(br.x0_pu) > 1e-12) {
      Cx z0(br.r0_pu, br.x0_pu);
      if (auto it_corr = transformer_corrections.find(br.index);
          it_corr != transformer_corrections.end()) {
        z0 *= it_corr->second.k0;
      }
      Cx y0 = Cx(1.0, 0.0) / z0;
      Cx y0_shunt(0.0, br.b0_pu / 2.0);
      if (std::abs(t - Cx(1.0, 0.0)) < 1e-9) {
        Ybus0(fi, fi) += y0 + y0_shunt;
        Ybus0(ti, ti) += y0 + y0_shunt;
        Ybus0(fi, ti) += -y0;
        Ybus0(ti, fi) += -y0;
      } else {
        double t2 = std::norm(t);
        Ybus0(fi, fi) += y0 / t2 + y0_shunt / t2;
        Ybus0(ti, ti) += y0 + y0_shunt;
        Ybus0(fi, ti) += -y0 / std::conj(t);
        Ybus0(ti, fi) += -y0 / t;
      }
    }
  }
  }

  // --- generators (subtransient or steady-state shunt with KG correction) ---
  for (const auto& g : ac.generators) {
    if (!machine_shunts) break;  // network-only build: skip all machine shunts
    if (!g.in_service) continue;
    auto it = id_map.find(g.bus);
    if (it == id_map.end()) continue;
    int gi = it->second;

    // Use xd_pu (steady-state reactance) for steady-state, xdpp_pu (subtransient) otherwise
    double xdpp = steady_state
        ? ((g.xd_pu > 1e-6) ? g.xd_pu : ((g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp))
        : ((g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp);
    double ra = g.ra_pu;
    double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;

    // Convert to system pu
    Cx z_gen = Cx(ra, xdpp) * (base_mva / mbase);

    // KG correction factor
    double bus_kv = 1.0;
    for (const auto& bus : ac.buses) {
      if (bus.index == g.bus) { bus_kv = bus.base_kv; break; }
    }
    double c = detailed_voltage_factor(bus_kv, opt);
    double cos_phi = std::clamp(g.cos_phi, 0.01, 1.0);
    double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
    double KG = c / (1.0 + xdpp * sin_phi);
    Cx z_gen_corr = z_gen * KG;

    if (std::abs(z_gen_corr) > 1e-15) {
      Cx y_gen = Cx(1.0, 0.0) / z_gen_corr;
      Ybus(gi, gi) += y_gen;
      Ybus2(gi, gi) += y_gen;  // Z2 ≈ Z1 for synchronous generators
    }

    // Zero-sequence generator
    double x0 = g.x0_pu;
    double r0 = g.r0_pu;
    if (x0 > 1e-12 || r0 > 1e-12) {
      Cx z_gen_0 = Cx(r0, x0) * (base_mva / mbase);
      double KG0 = c / (1.0 + x0 * sin_phi);
      Cx z_gen_0_corr = z_gen_0 * KG0;
      if (std::abs(z_gen_0_corr) > 1e-15) {
        Ybus0(gi, gi) += Cx(1.0, 0.0) / z_gen_0_corr;
      }
    }
  }

  // --- asynchronous motors (subtransient only, not in steady-state) ---
  if (!steady_state && machine_shunts) {
    for (const auto& m : ac.motors) {
      if (!m.in_service) continue;
      const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
      if (motor_p_mw < kMinMotorContributionMw) continue;
      auto it = id_map.find(m.bus);
      if (it == id_map.end()) continue;
      int mi = it->second;

      Cx z_motor_ohm(m.r_pu, m.x_pu);
      if (std::abs(z_motor_ohm) < 1e-15) continue;
      double motor_vn = (m.vn_kv > 1e-6) ? m.vn_kv : 1.0;
      double z_base = motor_vn * motor_vn / base_mva;
      Cx z_motor = z_motor_ohm / z_base;
      Cx y_motor = Cx(1.0, 0.0) / z_motor;
      Ybus(mi, mi) += y_motor;
      Ybus2(mi, mi) += y_motor;  // Z2 ≈ Z1 for asynchronous motors

      // Zero-sequence motor
      if (std::abs(m.r0_pu) > 1e-12 || std::abs(m.x0_pu) > 1e-12) {
        Cx z_m0_ohm(m.r0_pu, m.x0_pu);
        Cx z_m0 = z_m0_ohm / z_base;
        if (std::abs(z_m0) > 1e-15) {
          Ybus0(mi, mi) += Cx(1.0, 0.0) / z_m0;
        }
      }
    }

    // --- loads with motor fraction (subtransient only) ---
    for (const auto& ld : ac.loads) {
      if (!ld.in_service) continue;
      const double motor_fraction = load_motor_fraction(ld.motor_percent);
      if (motor_fraction <= 1e-6 || ld.sn_mva <= 1e-6) continue;
      if (load_motor_active_power_mw(ld, motor_fraction) < kMinMotorContributionMw) continue;
      auto it = id_map.find(ld.bus);
      if (it == id_map.end()) continue;
      int li = it->second;

      double actual_s = ld.sn_mva * motor_fraction;
      if (actual_s < 1e-12) continue;
      Cx z_load = Cx(ld.r_sc_pu, ld.x_sub_pu) * (base_mva / actual_s);
      if (std::abs(z_load) > 1e-15) {
        Cx y_load = Cx(1.0, 0.0) / z_load;
        Ybus(li, li) += y_load;
        Ybus2(li, li) += y_load;  // Z2 ≈ Z1 for load motor fraction
      }
    }
  }

  // --- external grids ---
  for (const auto& eg : ac.external_grids) {
    if (!eg.in_service) continue;
    auto it = id_map.find(eg.bus);
    if (it == id_map.end()) continue;
    int ei = it->second;

    double bus_kv = 1.0;
    for (const auto& bus : ac.buses) {
      if (bus.index == eg.bus) { bus_kv = bus.base_kv; break; }
    }
    const auto z_ext = external_grid_impedance_sc(eg, bus_kv, base_mva, opt);
    if (std::abs(z_ext.z1) > 1e-15) {
      Cx y_ext = Cx(1.0, 0.0) / z_ext.z1;
      Ybus(ei, ei) += y_ext;
      Ybus2(ei, ei) += y_ext;
    }

    // Zero-sequence external grid
    if (std::abs(z_ext.z0) > 1e-15) {
      Ybus0(ei, ei) += Cx(1.0, 0.0) / z_ext.z0;
    }
  }

  // --- VSC converters (AC grid-forming contributes to Ybus; grid-following = current source) ---
  if (!steady_state && machine_shunts) {
    for (const auto& conv : vsc_converters) {
      if (!conv.in_service) continue;
      auto it = id_map.find(conv.bus_ac);
      if (it == id_map.end()) continue;
      int ci = it->second;

      if (is_ac_grid_forming_converter(conv)) {
        // Grid-forming: voltage source behind impedance Z_filter + Z_virtual
        double r1 = conv.r_sc_pu;
        double x1 = (conv.x_sc_pu > 1e-12) ? conv.x_sc_pu : 0.15;

        // Convert from converter base to system base
        double s_rated = (conv.p_rated_mw > 1e-6) ? conv.p_rated_mw : base_mva;
        Cx z_conv = Cx(r1, x1) * (base_mva / s_rated);

        if (std::abs(z_conv) > 1e-15) {
          Ybus(ci, ci) += Cx(1.0, 0.0) / z_conv;
        }

        // Negative-sequence: converters can have different Z2
        double r2 = (conv.r2_sc_pu > 1e-12 || conv.x2_sc_pu > 1e-12)
                         ? conv.r2_sc_pu : r1;
        double x2 = (conv.r2_sc_pu > 1e-12 || conv.x2_sc_pu > 1e-12)
                         ? conv.x2_sc_pu : x1;
        Cx z_conv2 = Cx(r2, x2) * (base_mva / s_rated);

        if (std::abs(z_conv2) > 1e-15) {
          Ybus2(ci, ci) += Cx(1.0, 0.0) / z_conv2;
        }
      }
      // Grid-following converters: handled as current injection in the main computation
    }
  }

  // --- 3-winding transformers (star-branch equivalent) ---
  for (const auto& t3 : ac.transformers_3w) {
    if (!t3.in_service) continue;
    auto it_h = id_map.find(t3.hv_bus);
    auto it_m = id_map.find(t3.mv_bus);
    auto it_l = id_map.find(t3.lv_bus);
    if (it_h == id_map.end() || it_m == id_map.end() || it_l == id_map.end()) continue;
    int hi = it_h->second, mi = it_m->second, li = it_l->second;

    // Convert from percent impedances to pu on system base
    // vk_percent = 100 * Z_pu * S_base / S_rated
    double sn = (t3.sn_hv_mva > 1e-6) ? t3.sn_hv_mva : base_mva;

    // Pair-wise impedances in pu on system base
    auto pct_to_pu = [&](double vkr_pct, double vk_pct, double sn_pair) -> Cx {
      double s = (sn_pair > 1e-6) ? sn_pair : sn;
      double r_pu = (vkr_pct / 100.0) * (base_mva / s);
      double z_pu = (vk_pct / 100.0) * (base_mva / s);
      double x_pu = std::sqrt(std::max(0.0, z_pu * z_pu - r_pu * r_pu));
      return Cx(r_pu, x_pu);
    };

    Cx z_hm = pct_to_pu(t3.vkr_hv_mv_percent, t3.vk_hv_mv_percent, t3.sn_hv_mva);
    Cx z_hl = pct_to_pu(t3.vkr_hv_lv_percent, t3.vk_hv_lv_percent, t3.sn_hv_mva);
    Cx z_ml = pct_to_pu(t3.vkr_mv_lv_percent, t3.vk_mv_lv_percent, t3.sn_mv_mva);

    // Star-branch impedances from measured pair impedances:
    //   Z_H = 0.5 * (Z_HM + Z_HL - Z_ML)
    //   Z_M = 0.5 * (Z_HM + Z_ML - Z_HL)
    //   Z_L = 0.5 * (Z_HL + Z_ML - Z_HM)
    Cx z_h = 0.5 * (z_hm + z_hl - z_ml);
    Cx z_m = 0.5 * (z_hm + z_ml - z_hl);
    Cx z_l = 0.5 * (z_hl + z_ml - z_hm);

    // Ensure minimum impedance to avoid singularity
    auto safe_y = [](Cx z) -> Cx {
      return (std::abs(z) > 1e-15) ? Cx(1.0, 0.0) / z : Cx(0.0, 0.0);
    };

    Cx y_h = safe_y(z_h);
    Cx y_m = safe_y(z_m);
    Cx y_l = safe_y(z_l);

    // Add star-branch admittance entries (virtual star node eliminated)
    // Y_HM = y_h * y_m / (y_h + y_m + y_l), etc.
    Cx y_sum = y_h + y_m + y_l;
    if (std::abs(y_sum) > 1e-15) {
      Cx Y_HM = y_h * y_m / y_sum;
      Cx Y_HL = y_h * y_l / y_sum;
      Cx Y_ML = y_m * y_l / y_sum;

      Ybus(hi, hi) += Y_HM + Y_HL;
      Ybus(mi, mi) += Y_HM + Y_ML;
      Ybus(li, li) += Y_HL + Y_ML;
      Ybus(hi, mi) += -Y_HM;
      Ybus(mi, hi) += -Y_HM;
      Ybus(hi, li) += -Y_HL;
      Ybus(li, hi) += -Y_HL;
      Ybus(mi, li) += -Y_ML;
      Ybus(li, mi) += -Y_ML;

      // Negative-seq same as positive for passive 3W transformer
      Ybus2(hi, hi) += Y_HM + Y_HL;
      Ybus2(mi, mi) += Y_HM + Y_ML;
      Ybus2(li, li) += Y_HL + Y_ML;
      Ybus2(hi, mi) += -Y_HM;
      Ybus2(mi, hi) += -Y_HM;
      Ybus2(hi, li) += -Y_HL;
      Ybus2(li, hi) += -Y_HL;
      Ybus2(mi, li) += -Y_ML;
      Ybus2(li, mi) += -Y_ML;
    }

    // Zero-sequence 3W transformer — uses same star-branch model
    // (zero-sequence pair impedances not separately specified; use pos-seq as default)
    // In a full implementation, vector groups (Yy, Dy, Yd) affect zero-sequence paths
  }

  return {Ybus, Ybus2, Ybus0};
}

// -------------------------------------------------------------------------
// Safely invert a dense matrix, returning zero matrix on failure
// -------------------------------------------------------------------------
Eigen::MatrixXcd safe_inverse(const Eigen::MatrixXcd& M) {
  const int n = static_cast<int>(M.rows());
  // Check for NaN/Inf
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j)
      if (std::isnan(std::abs(M(i,j))) || std::isinf(std::abs(M(i,j))))
        return Eigen::MatrixXcd::Zero(n, n);
  // Use FullPivLU for robust inversion
  Eigen::FullPivLU<Eigen::MatrixXcd> lu(M);
  if (!lu.isInvertible())
    return Eigen::MatrixXcd::Zero(n, n);
  return lu.inverse();
}

// -------------------------------------------------------------------------
// Compute equivalent Zk for different fault types (with separate Z2)
// Returns effective impedance Zk such that I"k = c / |Zk|
// gives the actual fault current per IEC 60909.
// -------------------------------------------------------------------------
Cx compute_Zk(FaultType ft, Cx Z1, Cx Z2, Cx Z0, Cx Zf) {
  switch (ft) {
    case FaultType::ThreePhase:
      return Z1 + Zf;
    case FaultType::SinglePhaseGround:
      // I"k_1 = 3c/(Z1+Z2+Z0+3Zf) → Zk_eff = (Z1+Z2+Z0+3Zf)/3
      return (Z1 + Z2 + Z0 + Cx(3.0, 0.0) * Zf) / 3.0;
    case FaultType::TwoPhase:
      // I"k_2 = sqrt(3)*c/(Z1+Z2) → Zk_eff = (Z1+Z2)/sqrt(3)
      return (Z1 + Z2) / std::sqrt(3.0);
    case FaultType::TwoPhaseGround: {
      // I"k_2E ≈ c/(Z1 + Z2||Z0 + Zf)  (positive-seq current magnitude)
      Cx Z2_par_Z0 = (std::abs(Z2 + Z0) > 1e-15)
          ? (Z2 * Z0) / (Z2 + Z0) : Cx(0.0, 0.0);
      return Z1 + Z2_par_Z0 + Zf;
    }
    default:
      return Z1 + Zf;
  }
}

// Legacy overload (Z2 = Z1)
Cx compute_Zk(FaultType ft, Cx Z1, Cx Z0, Cx Zf) {
  return compute_Zk(ft, Z1, Z1, Z0, Zf);
}

// -------------------------------------------------------------------------
// Transfer impedance ratio for other buses
// -------------------------------------------------------------------------
struct TransferRatios {
  Cx Zk_self;   // equivalent Zk at this bus (for self-impedance)
  Cx Zk_xfer;   // transfer impedance to fault bus
};

TransferRatios compute_transfer(FaultType ft,
                                const Eigen::MatrixXcd& Zbus,
                                const Eigen::MatrixXcd& Zbus2,
                                const Eigen::MatrixXcd& Zbus0,
                                int bus_idx, int fault_idx) {
  TransferRatios tr;
  switch (ft) {
    case FaultType::ThreePhase:
      tr.Zk_xfer = Zbus(bus_idx, fault_idx);
      tr.Zk_self = Zbus(bus_idx, bus_idx);
      break;
    case FaultType::SinglePhaseGround:
      tr.Zk_xfer = (Zbus(bus_idx, fault_idx) + Zbus2(bus_idx, fault_idx) + Zbus0(bus_idx, fault_idx)) / 3.0;
      tr.Zk_self = (Zbus(bus_idx, bus_idx) + Zbus2(bus_idx, bus_idx) + Zbus0(bus_idx, bus_idx)) / 3.0;
      break;
    case FaultType::TwoPhase:
      tr.Zk_xfer = (Zbus(bus_idx, fault_idx) + Zbus2(bus_idx, fault_idx)) / std::sqrt(3.0);
      tr.Zk_self = (Zbus(bus_idx, bus_idx) + Zbus2(bus_idx, bus_idx)) / std::sqrt(3.0);
      break;
    case FaultType::TwoPhaseGround: {
      auto z_eff = [](Cx z1, Cx z2, Cx z0) -> Cx {
        Cx z2_par_z0 = (std::abs(z2 + z0) > 1e-15)
            ? (z2 * z0) / (z2 + z0) : Cx(0.0, 0.0);
        return z1 + z2_par_z0;
      };
      tr.Zk_xfer = z_eff(Zbus(bus_idx, fault_idx), Zbus2(bus_idx, fault_idx), Zbus0(bus_idx, fault_idx));
      tr.Zk_self = z_eff(Zbus(bus_idx, bus_idx), Zbus2(bus_idx, bus_idx), Zbus0(bus_idx, bus_idx));
      break;
    }
    default:
      tr.Zk_xfer = Zbus(bus_idx, fault_idx);
      tr.Zk_self = Zbus(bus_idx, bus_idx);
      break;
  }
  return tr;
}

// -------------------------------------------------------------------------
// IEC 60909 mu factor (for breaking current decay)
// -------------------------------------------------------------------------
double compute_mu(double Ik_ratio, double t_break) {
  if (t_break <= 0.02)
    return 0.84 + 0.26 * std::exp(-0.26 * Ik_ratio);
  if (t_break <= 0.05)
    return 0.71 + 0.51 * std::exp(-0.30 * Ik_ratio);
  if (t_break <= 0.10)
    return 0.62 + 0.72 * std::exp(-0.32 * Ik_ratio);
  return 0.56 + 0.94 * std::exp(-0.38 * Ik_ratio);
}

// -------------------------------------------------------------------------
// IEC 60909 q factor (for motor breaking current)
// -------------------------------------------------------------------------
double compute_q(double pn_mw, int poles, double t_break) {
  double ratio = (poles > 0) ? (pn_mw / poles) : pn_mw;
  if (ratio < 1e-12) return 1.0;
  double log_r = std::log(ratio);
  double q;
  if (t_break <= 0.02)
    q = 1.03 + 0.12 * log_r;
  else if (t_break <= 0.05)
    q = 0.79 + 0.12 * log_r;
  else if (t_break <= 0.10)
    q = 0.57 + 0.12 * log_r;
  else
    q = 0.26 + 0.10 * log_r;
  return std::clamp(q, 0.0, 1.0);
}

}  // anonymous namespace

// =========================================================================
// SCResult::summary
// =========================================================================
std::string SCResult::summary() const {
  std::ostringstream ss;
  ss << "Short Circuit Analysis — " << bus_results.size() << " buses\n";
  ss << "  Base MVA: " << base_mva << "\n";
  double max_sk = 0.0, min_sk = 1e30;
  int max_bus = -1, min_bus = -1;
  for (const auto& r : bus_results) {
    if (r.sk_mva > max_sk) { max_sk = r.sk_mva; max_bus = r.bus_id; }
    if (r.sk_mva < min_sk) { min_sk = r.sk_mva; min_bus = r.bus_id; }
  }
  ss << "  Max Sk = " << max_sk << " MVA  (bus " << max_bus << ")\n";
  ss << "  Min Sk = " << min_sk << " MVA  (bus " << min_bus << ")\n";
  return ss.str();
}

// =========================================================================
// compute_short_circuit — all buses
// =========================================================================
SCResult compute_short_circuit(const HybridPowerSystem& sys,
                               const SCOptions& opt) {
  const auto projection_bundle =
      projection::RichToCanonicalOperator::apply(sys);
  const auto& projected = projection_bundle.canonical;
  const auto& ac = projected.ac;
  const int n = static_cast<int>(ac.buses.size());
  SCResult result;
  result.base_mva = projected.base_mva;

  if (n == 0) return result;

  const auto id_map = build_id_map(ac.buses);

  // Build and factorise Y_fault
  SpMat Y_fault = build_fault_ybus(ac, id_map, n, opt);
  SpLU lu;
  lu.analyzePattern(Y_fault);
  lu.factorize(Y_fault);
  if (lu.info() != Eigen::Success)
    throw std::runtime_error("compute_short_circuit: Y_fault factorisation failed");

  std::vector<BusFaultResult> canonical_bus_results;
  canonical_bus_results.reserve(n);
  for (int k = 0; k < n; ++k) {
    // Canonical projection reindexes buses to a 1..n_merged sequence (and may
    // merge zero-impedance buses).  Whenever a BusMergeMap is present it records
    // the original external bus index for each internal position, so use it to
    // report ids that align with the pre-projection system the caller built.
    // NOTE: gate on map presence, not has_merges() — pure reindexing (no
    // merges) still renumbers non-contiguous ids and must be translated back.
    int bus_id = ac.buses[k].index;
    if (projected.bus_merge_map) {
      const auto& mmap = *projected.bus_merge_map;
      if (k < static_cast<int>(mmap.int_to_ext.size())) {
        bus_id = mmap.int_to_ext[static_cast<size_t>(k)];
      }
    }
    double base_kv = ac.buses[k].base_kv;
    canonical_bus_results.push_back(
        fault_at_bus(lu, k, bus_id, n, projected.base_mva, base_kv, opt));
  }
  if (projected.bus_merge_map) {
    const auto positions =
        projection::CanonicalToRichOperator::ac_bus_reprojection_positions(
            sys, projection_bundle);
    result.bus_results.reserve(sys.ac.buses.size());
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const int position = positions[i];
      if (position < 0 ||
          position >= static_cast<int>(canonical_bus_results.size())) {
        continue;
      }
      auto attributed = canonical_bus_results[static_cast<size_t>(position)];
      attributed.bus_id = sys.ac.buses[i].index;
      result.bus_results.push_back(std::move(attributed));
    }
  } else {
    result.bus_results = std::move(canonical_bus_results);
  }
  return result;
}

// =========================================================================
// compute_fault_at_bus — single bus
// =========================================================================
BusFaultResult compute_fault_at_bus(const HybridPowerSystem& sys,
                                    int bus_id,
                                    const SCOptions& opt) {
  const auto projection_bundle =
      projection::RichToCanonicalOperator::apply(sys);
  const auto& projected = projection_bundle.canonical;
  const auto& ac = projected.ac;
  const int n = static_cast<int>(ac.buses.size());

  const auto id_map = build_id_map(ac.buses);

  // Canonical projection reindexes buses to 1..N; translate the caller's
  // original external bus id to the reindexed internal id via the merge map.
  // Gate on map presence (pure reindexing has no "merges" but still needs
  // translation), mirroring run_short_circuit_detailed.
  int resolved_bus_id = bus_id;
  if (projected.bus_merge_map) {
    const auto& mmap = *projected.bus_merge_map;
    auto it_m = mmap.ext_to_int.find(bus_id);
    if (it_m != mmap.ext_to_int.end())
      resolved_bus_id = static_cast<int>(it_m->second) + 1;  // 0-based pos → 1-based
  }
  auto it = id_map.find(resolved_bus_id);
  if (it == id_map.end())
    throw std::invalid_argument("compute_fault_at_bus: bus_id not found");
  int k = it->second;

  SpMat Y_fault = build_fault_ybus(ac, id_map, n, opt);
  SpLU lu;
  lu.analyzePattern(Y_fault);
  lu.factorize(Y_fault);
  if (lu.info() != Eigen::Success)
    throw std::runtime_error("compute_fault_at_bus: Y_fault factorisation failed");

  double base_kv = ac.buses[k].base_kv;
  return fault_at_bus(lu, k, bus_id, n, projected.base_mva, base_kv, opt);
}

double get_voltage_factor_sc(double vn_kv, SCCalcType calc_type) {
  if (calc_type == SCCalcType::Max) {
    if (vn_kv <= 1.0) return (vn_kv > 0.1) ? 1.10 : 1.05;
    return 1.10;
  }
  if (vn_kv <= 1.0) return (vn_kv > 0.1) ? 0.90 : 0.95;
  return 1.00;
}

double calculate_kappa_basic_sc(double rx_ratio) {
  return 1.02 + 0.98 * std::exp(-3.0 * rx_ratio);
}

double calculate_generator_correction_factor_sc(const SCGeneratorParams& p,
                                                double vn_kv,
                                                SCCalcType calc_type) {
  const double c = get_voltage_factor_sc(vn_kv, calc_type);
  const double cos_phi = std::clamp(p.cos_phi, -1.0, 1.0);
  const double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
  return c / (1.0 + p.xd_sub_pu * sin_phi);
}

double calculate_zero_sequence_generator_correction_factor_sc(const SCGeneratorParams& p,
                                                              double vn_kv,
                                                              SCCalcType calc_type) {
  const double c = get_voltage_factor_sc(vn_kv, calc_type);
  const double cos_phi = std::clamp(p.cos_phi, -1.0, 1.0);
  const double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
  return c / (1.0 + p.x0_pu * sin_phi);
}

double calculate_transformer_correction_factor_sc(const SCTransformerParams& p,
                                                  double vn_kv,
                                                  SCCalcType calc_type,
                                                  double base_mva) {
  const double c = get_voltage_factor_sc(vn_kv, calc_type);
  const double x_t = p.x_pu * p.sn_mva / ((base_mva > 0.0) ? base_mva : 100.0);
  return 0.95 * c / (1.0 + 0.6 * x_t);
}

double calculate_transformer_zero_sequence_correction_factor_sc(const SCTransformerParams& p,
                                                                double vn_kv,
                                                                SCCalcType calc_type,
                                                                double base_mva) {
  const double c = get_voltage_factor_sc(vn_kv, calc_type);
  const double x_t0 = p.x0_pu * p.sn_mva / ((base_mva > 0.0) ? base_mva : 100.0);
  return 0.95 * c / (1.0 + 0.6 * x_t0);
}

std::complex<double> calculate_motor_impedance_sc(const SCMotorParams& p) {
  return {p.r_ohm, p.x_ohm};
}

SCDetailedResult run_short_circuit_detailed(const HybridPowerSystem& sys,
                                            int fault_bus_id,
                                            const SCDetailedOptions& opt) {
  SCDetailedResult out;
  out.fault_bus_id = fault_bus_id;
  out.solved = false;

  const auto projection_bundle =
      projection::RichToCanonicalOperator::apply(sys);
  const auto& projected = projection_bundle.canonical;
  const auto& ac = projected.ac;
  const int n = static_cast<int>(ac.buses.size());
  if (n == 0) return out;

  const double base_mva = ac.base_mva > 0.0 ? ac.base_mva : 100.0;
  const auto id_map = build_id_map(ac.buses);

  // Helper: map post-merge internal 1-based sequential bus ID → original
  // external bus ID.  Gate on map presence (canonical projection reindexes
  // even when nothing is merged), not has_merges().
  const auto ext_bus_id = [&](int internal_id) -> int {
    if (projected.bus_merge_map) {
      const auto& mmap = *projected.bus_merge_map;
      auto pos = static_cast<size_t>(internal_id - 1);
      if (pos < mmap.int_to_ext.size()) return mmap.int_to_ext[pos];
    }
    return internal_id;
  };

  // Resolve fault_bus_id from original external ID → post-merge internal
  // sequential ID.  Gate on map presence so non-contiguous ids (e.g. 1-5,
  // 10-14, 20-21) are translated to the reindexed 1..N space rather than
  // being looked up verbatim (which silently faulted the wrong bus or threw).
  int resolved_fault_bus_id = fault_bus_id;
  if (projected.bus_merge_map) {
    const auto& mmap = *projected.bus_merge_map;
    auto it_m = mmap.ext_to_int.find(fault_bus_id);
    if (it_m != mmap.ext_to_int.end())
      resolved_fault_bus_id = static_cast<int>(it_m->second) + 1;  // 0-based pos → 1-based
  }

  auto it_fault = id_map.find(resolved_fault_bus_id);
  if (it_fault == id_map.end())
    throw std::invalid_argument("run_short_circuit_detailed: fault_bus_id not found");
  const int fault_idx = it_fault->second;

  const double fault_kv = ac.buses[fault_idx].base_kv;
  const double c = detailed_voltage_factor(fault_kv, opt);
  const double I_base = base_mva / (std::sqrt(3.0) * fault_kv);  // kA
  const Cx Zf(opt.fault_impedance_pu, 0.0);
  const auto transformer_corrections = build_transformer_branch_corrections(
      ac, projected.branch_expand_map, base_mva, opt);

  auto bus_kv = [&](int bus_id) -> double {
    for (const auto& bus : ac.buses) {
      if (bus.index == bus_id) return (bus.base_kv > 1e-6) ? bus.base_kv : fault_kv;
    }
    return fault_kv;
  };

  auto converter_current_multiplier = [](const VSCConverter& conv) -> double {
    if (conv.i_max_pu > 1e-6 && std::abs(conv.i_max_pu - 1.0) > 1e-9) {
      return conv.i_max_pu;
    }
    if (conv.i_ac_max_pu > 1e-6) return conv.i_ac_max_pu;
    return (conv.i_max_pu > 1e-6) ? conv.i_max_pu : 1.0;
  };

  auto external_grid_impedances = [&](const ExternalGrid& eg) -> std::pair<Cx, Cx> {
    const double eg_bus_kv = bus_kv(eg.bus);
    const auto z = external_grid_impedance_sc(eg, eg_bus_kv, base_mva, opt);
    return {z.z1, z.z0};
  };

  // ====== Step 1: Build subtransient Ybus & Zbus ======
  auto [Ybus, Ybus2, Ybus0] = build_sc_admittance_matrices(
      ac, projected.vsc_converters, id_map, n, opt, transformer_corrections, false);
  Eigen::MatrixXcd Zbus  = safe_inverse(Ybus);
  Eigen::MatrixXcd Zbus2 = safe_inverse(Ybus2);
  Eigen::MatrixXcd Zbus0 = safe_inverse(Ybus0);

  // ====== Step 2: Compute initial SC current (Ikss) at fault bus ======
  Cx Z1_fault = Zbus(fault_idx, fault_idx);
  Cx Z2_fault = Zbus2(fault_idx, fault_idx);
  Cx Z0_fault = Zbus0(fault_idx, fault_idx);
  Cx Zk = compute_Zk(opt.fault_type, Z1_fault, Z2_fault, Z0_fault, Zf);

  double I_kss_pu = (std::abs(Zk) > 1e-15) ? (c / std::abs(Zk)) : 0.0;
  double I_kss_kA = I_kss_pu * I_base;

  // ====== Step 3: Compute per-source contributions at fault bus ======
  double gen_contrib = 0.0, motor_contrib = 0.0, load_contrib = 0.0;
  // Per-contribution peak data: (current [kA], source impedance [pu]) for
  // every voltage-source contribution; consumed by the formula-(59) peak
  // summation at the fault bus in Step 5.
  std::vector<std::pair<double, Cx>> vs_peak_contribs;

  auto source_transfer_abs = [&](int source_bus_id) -> double {
    auto it = id_map.find(source_bus_id);
    if (it == id_map.end() || std::abs(Zk) <= 1e-15) return 0.0;
    const auto tr = compute_transfer(opt.fault_type, Zbus, Zbus2, Zbus0,
                                     it->second, fault_idx);
    return std::abs(tr.Zk_xfer) / std::abs(Zk);
  };

  auto voltage_source_contribution_ka = [&](int source_bus_id, Cx source_zk) -> double {
    if (std::abs(source_zk) <= 1e-15) return 0.0;
    return (c / std::abs(source_zk)) * source_transfer_abs(source_bus_id) * I_base;
  };

  auto current_source_contribution_ka = [&](int source_bus_id, double source_current_ka) -> double {
    if (source_current_ka <= 1e-15) return 0.0;
    auto it = id_map.find(source_bus_id);
    if (it != id_map.end() && it->second == fault_idx) return source_current_ka;
    const double source_i_base = base_mva / (std::sqrt(3.0) * bus_kv(source_bus_id));
    if (source_i_base <= 1e-15) return 0.0;
    return (source_current_ka / source_i_base) * source_transfer_abs(source_bus_id) * I_base;
  };

  // Generator contributions
  for (const auto& g : ac.generators) {
    if (!g.in_service) continue;
    double xdpp = (g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp;
    double ra = g.ra_pu;
    double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
    Cx z_gen = Cx(ra, xdpp) * (base_mva / mbase);
    double cos_phi = std::clamp(g.cos_phi, 0.01, 1.0);
    double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
    double KG = c / (1.0 + xdpp * sin_phi);
    Cx z_gen_corr = z_gen * KG;

    double gen_ci = 0.0;
    Cx gen_z = z_gen_corr;
    if (opt.fault_type == FaultType::ThreePhase) {
      gen_ci = voltage_source_contribution_ka(g.bus, z_gen_corr);
    } else if (opt.fault_type == FaultType::SinglePhaseGround) {
      double x0 = g.x0_pu, r0 = g.r0_pu;
      Cx z_gen_0 = Cx(r0, x0) * (base_mva / mbase);
      double KG0 = c / (1.0 + x0 * sin_phi);
      Cx z_gen_0_corr = z_gen_0 * KG0;
      gen_z = (z_gen_corr * 2.0 + z_gen_0_corr) / 3.0;
      gen_ci = voltage_source_contribution_ka(g.bus, gen_z);
    } else if (opt.fault_type == FaultType::TwoPhase) {
      gen_z = z_gen_corr * 2.0 / std::sqrt(3.0);
      gen_ci = voltage_source_contribution_ka(g.bus, gen_z);
    } else if (opt.fault_type == FaultType::TwoPhaseGround) {
      double x0 = g.x0_pu, r0 = g.r0_pu;
      Cx z_gen_0 = Cx(r0, x0) * (base_mva / mbase);
      double KG0 = c / (1.0 + x0 * sin_phi);
      Cx z_gen_0_corr = z_gen_0 * KG0;
      gen_z = (2.0 * z_gen_corr * z_gen_0_corr + z_gen_corr * z_gen_corr) / (3.0 * z_gen_corr);
      gen_ci = voltage_source_contribution_ka(g.bus, gen_z);
    }
    gen_contrib += gen_ci;
    vs_peak_contribs.emplace_back(gen_ci, gen_z);
  }

  // Motor contributions at fault bus
  for (const auto& m : ac.motors) {
    if (!m.in_service) continue;
    const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
    if (motor_p_mw < kMinMotorContributionMw) continue;
    Cx z_motor_ohm(m.r_pu, m.x_pu);
    if (std::abs(z_motor_ohm) < 1e-15) continue;
    double mvn = (m.vn_kv > 1e-6) ? m.vn_kv : 1.0;
    double z_base = mvn * mvn / base_mva;
    Cx z_motor = z_motor_ohm / z_base;

    double motor_ci = 0.0;
    Cx motor_z = z_motor;
    if (opt.fault_type == FaultType::ThreePhase) {
      motor_ci = voltage_source_contribution_ka(m.bus, z_motor);
    } else if (opt.fault_type == FaultType::SinglePhaseGround) {
      Cx z_m0_ohm(m.r0_pu, m.x0_pu);
      Cx z_m0 = z_m0_ohm / z_base;
      motor_z = (2.0 * z_motor + z_m0) / 3.0;
      motor_ci = voltage_source_contribution_ka(m.bus, motor_z);
    } else if (opt.fault_type == FaultType::TwoPhase) {
      motor_z = z_motor * 2.0 / std::sqrt(3.0);
      motor_ci = voltage_source_contribution_ka(m.bus, motor_z);
    } else if (opt.fault_type == FaultType::TwoPhaseGround) {
      Cx z_m0_ohm(m.r0_pu, m.x0_pu);
      Cx z_m0 = z_m0_ohm / z_base;
      motor_z = (2.0 * z_motor * z_m0 + z_motor * z_motor) / (3.0 * z_motor);
      motor_ci = voltage_source_contribution_ka(m.bus, motor_z);
    }
    motor_contrib += motor_ci;
    vs_peak_contribs.emplace_back(motor_ci, motor_z);
  }

  // Load motor-fraction contributions at fault bus
  for (const auto& ld : ac.loads) {
    if (!ld.in_service) continue;
    const double motor_fraction = load_motor_fraction(ld.motor_percent);
    if (motor_fraction <= 1e-6 || ld.sn_mva <= 1e-6) continue;
    if (load_motor_active_power_mw(ld, motor_fraction) < kMinMotorContributionMw) continue;
    double actual_s = ld.sn_mva * motor_fraction;
    if (actual_s < 1e-12) continue;
    Cx z_load = Cx(ld.r_sc_pu, ld.x_sub_pu) * (base_mva / actual_s);
    const double contrib = voltage_source_contribution_ka(ld.bus, z_load);
    vs_peak_contribs.emplace_back(contrib, z_load);
    if (is_projected_rich_motor_load(ld)) {
      motor_contrib += contrib;
    } else {
      load_contrib += contrib;
    }
  }

  // Static generator (sgen) contributions at fault bus via IEC 60909 §6.7
  double sgen_contrib = 0.0;
  for (const auto& sg : ac.static_generators) {
    if (!sg.in_service) continue;
    if (sg.sn_mva < 1e-12 || sg.k < 1e-12) continue;
    double I_rated = sg.sn_mva / (std::sqrt(3.0) * bus_kv(sg.bus));
    sgen_contrib += current_source_contribution_ka(sg.bus, sg.k * I_rated);
  }

  // External grid contributions at fault bus
  double extgrid_contrib = 0.0;
  for (const auto& eg : ac.external_grids) {
    if (!eg.in_service) continue;
    const auto [z1, z0] = external_grid_impedances(eg);
    if (std::abs(z1) <= 1e-15) continue;
    const Cx source_zk = compute_Zk(opt.fault_type, z1, z1, z0, Cx(0.0, 0.0));
    extgrid_contrib += voltage_source_contribution_ka(eg.bus, source_zk);
  }

  // Converter contributions. Grid-following converters are current sources;
  // AC grid-forming converters are represented by the same voltage-source shunt
  // that build_sc_admittance_matrices() adds to the fault network.
  double converter_contrib = 0.0;
  double converter_current_source_contrib = 0.0;
  for (const auto& conv : projected.vsc_converters) {
    if (!conv.in_service) continue;
    SCConverterContributionResult conv_row;
    const auto role = resolve_device_control_role(conv);
    conv_row.converter_index = conv.index;
    conv_row.bus_id = ext_bus_id(conv.bus_ac);
    conv_row.name = conv.name;
    conv_row.ac_grid_forming = role.is_ac_grid_forming;
    conv_row.dc_grid_forming = role.is_dc_grid_forming;
    conv_row.p_rated_mw = conv.p_rated_mw;
    conv_row.i_limit_pu = converter_current_multiplier(conv);
    if (is_ac_grid_forming_converter(conv)) {
      const double s_rated = (conv.p_rated_mw > 1e-6) ? conv.p_rated_mw : base_mva;
      const Cx z_conv = Cx(conv.r_sc_pu, (conv.x_sc_pu > 1e-12) ? conv.x_sc_pu : 0.15)
                        * (base_mva / s_rated);
      const double contrib = voltage_source_contribution_ka(conv.bus_ac, z_conv);
      converter_contrib += contrib;
      vs_peak_contribs.emplace_back(contrib, z_conv);
      conv_row.model = "ac_grid_forming_voltage_source";
      conv_row.contribution_ka = contrib;
      out.converter_contributions.push_back(std::move(conv_row));
      continue;
    }
    const double s_rated = (conv.p_rated_mw > 1e-6) ? conv.p_rated_mw : 0.0;
    if (s_rated < 1e-12) {
      conv_row.model = role.is_dc_grid_forming
          ? "dc_side_forming_not_ac_source_missing_rating"
          : "grid_following_current_source_missing_rating";
      out.converter_contributions.push_back(std::move(conv_row));
      continue;
    }
    const double i_rated = s_rated / (std::sqrt(3.0) * bus_kv(conv.bus_ac));
    const double contrib = current_source_contribution_ka(
        conv.bus_ac, conv_row.i_limit_pu * i_rated);
    converter_contrib += contrib;
    converter_current_source_contrib += contrib;
    conv_row.model = role.is_dc_grid_forming
        ? "dc_side_forming_current_limited_source"
        : "grid_following_current_source";
    conv_row.contribution_ka = contrib;
    out.converter_contributions.push_back(std::move(conv_row));
  }
  const double total_ikss_kA = I_kss_kA + sgen_contrib + converter_current_source_contrib;
  const double no_motor = std::max(0.0, total_ikss_kA - gen_contrib - motor_contrib - load_contrib);

  Eigen::VectorXcd V_fault(n);
  const Cx Z_voltage_denom = Zbus(fault_idx, fault_idx) + Zf;
  for (int k = 0; k < n; ++k) {
    if (k == fault_idx) {
      V_fault[k] = Cx(0.0, 0.0);
    } else if (std::abs(Z_voltage_denom) > 1e-15) {
      V_fault[k] = Cx(c, 0.0) *
                   (Cx(1.0, 0.0) - Zbus(k, fault_idx) / Z_voltage_denom);
    } else {
      V_fault[k] = Cx(c, 0.0);
    }
  }

  // ====== Step 4: Build result vector for all buses ======
  out.bus_results.resize(n);
  for (int k = 0; k < n; ++k) {
    auto& row = out.bus_results[k];
    row.bus_id = (k == fault_idx) ? fault_bus_id : ext_bus_id(ac.buses[k].index);

    if (k == fault_idx) {
      row.ikss_ka = total_ikss_kA;
      row.ikss_1_ka = I_kss_kA;
      // Negative-sequence current uses Zbus2
      row.ikss_2_ka = (opt.fault_type == FaultType::ThreePhase) ? 0.0
          : ((std::abs(Z2_fault) > 1e-15) ? (c / std::abs(Z2_fault)) * I_base : 0.0);
      row.ikss_gen_contrib_ka = gen_contrib;
      row.ikss_motor_contrib_ka = motor_contrib;
      row.ikss_load_contrib_ka = load_contrib;
      row.ikss_sgen_contrib_ka = sgen_contrib;
      row.ikss_extgrid_contrib_ka = extgrid_contrib;
      row.ikss_converter_contrib_ka = converter_contrib;
      row.ikss_no_motor_ka = no_motor;
      row.v_remaining_pu = 0.0;  // Voltage at fault point is zero
    } else {
      // Transfer impedance based contribution
      auto tr = compute_transfer(opt.fault_type, Zbus, Zbus2, Zbus0, k, fault_idx);
      double I_pu = (std::abs(Zk) > 1e-15 && std::abs(tr.Zk_self) > 1e-15)
                        ? c * std::abs(tr.Zk_xfer) / (std::abs(Zk) * std::abs(tr.Zk_self))
                        : 0.0;
      double I_kA = I_pu * I_base;
      row.ikss_ka = I_kA;
      row.ikss_1_ka = I_kA;
      // Negative-sequence current at non-fault buses via Zbus2 transfer
      if (opt.fault_type == FaultType::ThreePhase) {
        row.ikss_2_ka = 0.0;
      } else {
        double Z2_self = std::abs(Zbus2(k, k));
        double Z2_xfer = std::abs(Zbus2(k, fault_idx));
        row.ikss_2_ka = (Z2_self > 1e-15 && std::abs(Zk) > 1e-15)
            ? c * Z2_xfer / (std::abs(Zk) * Z2_self) * I_base : 0.0;
      }

      // Voltage drop at non-fault bus: V_remaining = c * (1 - Z_kf / (Z_ff + Zf)).
      if (opt.compute_voltage_drops) {
        row.v_remaining_pu = std::abs(V_fault[k]);
      }

      // Per-source contributions at this bus via transfer ratio
      double bus_gen = 0.0, bus_motor = 0.0, bus_load = 0.0;
      int bus_id = ac.buses[k].index;

      for (const auto& g : ac.generators) {
        if (!g.in_service || g.bus != bus_id) continue;
        double xdpp = (g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp;
        double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
        double cos_phi = std::clamp(g.cos_phi, 0.01, 1.0);
        double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
        double KG = c / (1.0 + xdpp * sin_phi);
        Cx z_gen_corr = Cx(g.ra_pu, xdpp) * (base_mva / mbase) * KG;

        if (opt.fault_type == FaultType::ThreePhase) {
          if (std::abs(z_gen_corr) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += c * std::abs(Zbus(k, fault_idx)) / (std::abs(z_gen_corr) * std::abs(Zk)) * I_base;
        } else if (opt.fault_type == FaultType::SinglePhaseGround) {
          Cx z_gen_0 = Cx(g.r0_pu, g.x0_pu) * (base_mva / mbase);
          double KG0 = c / (1.0 + g.x0_pu * sin_phi);
          Cx z_gen_0_corr = z_gen_0 * KG0;
          Cx z_k = (2.0 * z_gen_corr + z_gen_0_corr) / 3.0;
          if (std::abs(z_k) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += (c / std::abs(z_k)) * std::abs(tr.Zk_xfer) / std::abs(Zk) * I_base;
        } else if (opt.fault_type == FaultType::TwoPhase) {
          Cx z_k = z_gen_corr * 2.0 / std::sqrt(3.0);
          if (std::abs(z_k) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += (c / std::abs(z_k)) * std::abs(Zbus(k, fault_idx) * 2.0 / std::sqrt(3.0)) / std::abs(Zk) * I_base;
        } else if (opt.fault_type == FaultType::TwoPhaseGround) {
          Cx z_gen_0 = Cx(g.r0_pu, g.x0_pu) * (base_mva / mbase);
          double KG0 = c / (1.0 + g.x0_pu * sin_phi);
          Cx z_gen_0_corr = z_gen_0 * KG0;
          Cx z_k = (2.0 * z_gen_corr * z_gen_0_corr + z_gen_corr * z_gen_corr) / (3.0 * z_gen_corr);
          if (std::abs(z_k) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += (c / std::abs(z_k)) * std::abs(tr.Zk_xfer) / std::abs(Zk) * I_base;
        }
      }

      for (const auto& m : ac.motors) {
        if (!m.in_service || m.bus != bus_id) continue;
        const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
        if (motor_p_mw < kMinMotorContributionMw) continue;
        Cx z_motor_ohm(m.r_pu, m.x_pu);
        if (std::abs(z_motor_ohm) < 1e-15) continue;
        double mvn = (m.vn_kv > 1e-6) ? m.vn_kv : 1.0;
        double z_base = mvn * mvn / base_mva;
        Cx z_motor = z_motor_ohm / z_base;

        if (opt.fault_type == FaultType::ThreePhase) {
          double m_pu = (c / std::abs(z_motor)) * std::abs(Zbus(k, fault_idx)) /
                        (std::abs(Zk) * std::abs(Zbus(k, k)));
          bus_motor += m_pu * I_base;
        } else {
          // For unbalanced faults, use transfer ratio
          double m_pu = (c / std::abs(z_motor)) * std::abs(tr.Zk_xfer) / std::abs(Zk);
          bus_motor += m_pu * I_base;
        }
      }

      for (const auto& ld : ac.loads) {
        if (!ld.in_service || ld.bus != bus_id) continue;
        const double motor_fraction = load_motor_fraction(ld.motor_percent);
        if (motor_fraction <= 1e-6 || ld.sn_mva <= 1e-6) continue;
        if (load_motor_active_power_mw(ld, motor_fraction) < kMinMotorContributionMw) continue;
        const double actual_s = ld.sn_mva * motor_fraction;
        if (actual_s < 1e-12) continue;
        const Cx z_load = Cx(ld.r_sc_pu, ld.x_sub_pu) * (base_mva / actual_s);
        if (std::abs(z_load) <= 1e-15 || std::abs(Zk) <= 1e-15) continue;
        const double contrib =
            (c / std::abs(z_load)) * std::abs(tr.Zk_xfer) / std::abs(Zk) * I_base;
        if (is_projected_rich_motor_load(ld)) {
          bus_motor += contrib;
        } else {
          bus_load += contrib;
        }
      }

      row.ikss_gen_contrib_ka = bus_gen;
      row.ikss_motor_contrib_ka = bus_motor;
      row.ikss_load_contrib_ka = bus_load;

      // Static generator contribution at non-fault buses via transfer ratio
      double bus_sgen = 0.0;
      for (const auto& sg : ac.static_generators) {
        if (!sg.in_service || sg.bus != bus_id) continue;
        if (sg.sn_mva < 1e-12 || sg.k < 1e-12) continue;
        double bus_kv_local = ac.buses[static_cast<size_t>(k)].base_kv;
        double I_rated = sg.sn_mva / (std::sqrt(3.0) * bus_kv_local);
        double sgen_fault_contrib = sg.k * I_rated;
        double tr_abs = (std::abs(Zk) > 1e-15)
            ? std::abs(tr.Zk_xfer) / std::abs(Zk) : 0.0;
        bus_sgen += sgen_fault_contrib * tr_abs;
      }
      row.ikss_sgen_contrib_ka = bus_sgen;

      // External grid contribution at non-fault buses via transfer ratio
      double bus_extgrid = 0.0;
      for (const auto& eg : ac.external_grids) {
        if (!eg.in_service || eg.bus != bus_id) continue;
        const Cx z_ext = external_grid_impedances(eg).first;
        if (std::abs(z_ext) > 1e-15) {
          double tr_abs = (std::abs(Zk) > 1e-15)
              ? std::abs(tr.Zk_xfer) / std::abs(Zk) : 0.0;
          bus_extgrid += (c / std::abs(z_ext)) * I_base * tr_abs;
        }
      }
      row.ikss_extgrid_contrib_ka = bus_extgrid;

      // Grid-following converter contribution at non-fault buses
      double bus_converter = 0.0;
      int bus_id_conv = ac.buses[k].index;
      for (const auto& conv : projected.vsc_converters) {
        if (!conv.in_service || is_ac_grid_forming_converter(conv)) continue;
        if (conv.bus_ac != bus_id_conv) continue;
        double s_rated = (conv.p_rated_mw > 1e-6) ? conv.p_rated_mw : 0.0;
        if (s_rated < 1e-12) continue;
        double conv_bus_kv = ac.buses[static_cast<size_t>(k)].base_kv;
        double i_rated = s_rated / (std::sqrt(3.0) * conv_bus_kv);
        double tr_abs = (std::abs(Zk) > 1e-15)
            ? std::abs(tr.Zk_xfer) / std::abs(Zk) : 0.0;
        bus_converter += converter_current_multiplier(conv) * i_rated * tr_abs;
      }
      row.ikss_converter_contrib_ka = bus_converter;

      row.ikss_no_motor_ka = std::max(0.0, I_kA - bus_gen - bus_motor - bus_load);
    }
  }

  // ====== Step 5: Peak current (ip) ======
  // Fault bus: IEC 60909-0 formula (59) — sum of per-contribution peaks
  //   ip = Σ κ_i·√2·I"k_i
  // with κ_i derived from each contribution's own R/X ratio. The network
  // part uses the Thevenin impedance of the network without machine shunts;
  // current sources (static generators, grid-following converters) have no
  // decaying DC component and are added without κ. Non-fault buses keep the
  // single-κ approximation on the transferred current (method A/B).
  Cx Z_k_kappa = compute_Zk(opt.fault_type,
                            Z1_fault, Z2_fault, Z0_fault, Zf);
  double rx_ratio = (std::abs(std::imag(Z_k_kappa)) > 1e-15)
                        ? std::abs(std::real(Z_k_kappa) / std::imag(Z_k_kappa))
                        : 0.0;
  double kappa = calculate_kappa_basic_sc(rx_ratio);
  if (opt.kappa_method == SCKappaMethod::B && opt.topology == SCTopology::Meshed) {
    kappa = std::min(1.8, 1.15 * kappa);
  } else {
    kappa = std::clamp(kappa, 1.0, 2.0);
  }

  auto kappa_of = [](Cx z) {
    const double rx = (std::abs(std::imag(z)) > 1e-15)
                          ? std::abs(std::real(z) / std::imag(z))
                          : 0.0;
    return std::clamp(calculate_kappa_basic_sc(rx), 1.0, 2.0);
  };

  // Network-only Thevenin impedance (branches + external grids, no machine
  // shunts) feeding the fault, for the network part of the peak summation.
  double kappa_net = kappa;
  {
    auto [Ybus_net, Ybus2_net, Ybus0_net] = build_sc_admittance_matrices(
        ac, projected.vsc_converters, id_map, n, opt, transformer_corrections,
        false, false);
    Eigen::MatrixXcd Zbus_net = safe_inverse(Ybus_net);
    Cx Zk_net;
    if (opt.fault_type == FaultType::ThreePhase) {
      Zk_net = compute_Zk(opt.fault_type,
                          Zbus_net(fault_idx, fault_idx),
                          Zbus_net(fault_idx, fault_idx),
                          Cx(0.0, 0.0), Zf);
    } else {
      Eigen::MatrixXcd Zbus0_net = safe_inverse(Ybus0_net);
      Zk_net = compute_Zk(opt.fault_type,
                          Zbus_net(fault_idx, fault_idx),
                          Zbus_net(fault_idx, fault_idx),
                          Zbus0_net(fault_idx, fault_idx), Zf);
    }
    kappa_net = kappa_of(Zk_net);
  }

  const double conv_gfm_contrib =
      std::max(0.0, converter_contrib - converter_current_source_contrib);
  const double i_network_ka =
      std::max(0.0, I_kss_kA - gen_contrib - motor_contrib - load_contrib -
                        conv_gfm_contrib);

  for (int k = 0; k < n; ++k) {
    auto& row = out.bus_results[k];
    if (k != fault_idx) {
      const double current_source_ka =
          std::max(0.0, row.ikss_ka - row.ikss_1_ka);
      row.ip_ka = std::sqrt(2.0) *
                  (kappa * row.ikss_1_ka + current_source_ka);
      continue;
    }
    double ip_sum = kappa_net * i_network_ka;
    for (const auto& [i_ka, z_src] : vs_peak_contribs) {
      ip_sum += kappa_of(z_src) * i_ka;
    }
    ip_sum += sgen_contrib + converter_current_source_contrib;  // κ = 1
    row.ip_ka = std::sqrt(2.0) * ip_sum;
  }

  // ====== Step 6: Breaking current (Ib) ======
  {
    const double t_break = opt.breaking_time_s;

    for (int k = 0; k < n; ++k) {
      auto& row = out.bus_results[k];
      double Ib = 0.0;

      // Generator contribution to breaking current
      for (const auto& g : ac.generators) {
        if (!g.in_service) continue;
        auto git = id_map.find(g.bus);
        if (git == id_map.end()) continue;
        int gi = git->second;

        double xdpp = (g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp;
        double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
        double u_rg = (g.vn_kv > 1e-6) ? g.vn_kv : fault_kv;
        double cos_phi = std::clamp(g.cos_phi, 0.01, 1.0);
        double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));

        double bus_kv = fault_kv;
        for (const auto& bus : ac.buses) {
          if (bus.index == g.bus) { bus_kv = bus.base_kv; break; }
        }

        double KG = c / (1.0 + xdpp * sin_phi);
        Cx z_gen_ohm = Cx(g.ra_pu, xdpp) * (u_rg * u_rg / mbase);
        double z_base_sys = bus_kv * bus_kv / base_mva;
        Cx z_gen = z_gen_ohm / z_base_sys;
        Cx z_gen_corr = z_gen * KG;

        double gen_current = 0.0, gen_current_tr = 0.0;
        if (opt.fault_type == FaultType::ThreePhase && std::abs(z_gen_corr) > 1e-15) {
          gen_current = (c / std::abs(z_gen_corr)) * I_base;
          Cx Z_k_b = Zbus(fault_idx, fault_idx) + Zf;
          gen_current_tr = (std::abs(Z_k_b) > 1e-15)
              ? (c * std::abs(Zbus(gi, fault_idx))) / (std::abs(z_gen_corr) * std::abs(Z_k_b)) * I_base
              : 0.0;
        }

        if (gen_current > 1e-15) {
          double Ik_ratio = gen_current * std::sqrt(3.0) * u_rg / mbase;
          double mu = compute_mu(Ik_ratio, t_break);
          Ib += gen_current_tr * mu;
        }
      }

      // Motor contribution to breaking current at all buses
      for (const auto& m : ac.motors) {
        if (!m.in_service) continue;
        const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
        if (motor_p_mw < kMinMotorContributionMw) continue;
        auto mit = id_map.find(m.bus);
        if (mit == id_map.end()) continue;
        Cx z_motor_ohm(m.r_pu, m.x_pu);
        if (std::abs(z_motor_ohm) < 1e-15) continue;
        double mvn = (m.vn_kv > 1e-6) ? m.vn_kv : 1.0;
        double z_base = mvn * mvn / base_mva;
        Cx z_motor = z_motor_ohm / z_base;
        double motor_current = (c / std::abs(z_motor)) * I_base;

        int mi = mit->second;
        double Zk_abs = std::abs(Zbus(fault_idx, fault_idx) + Zf);
        double motor_current_tr = (Zk_abs > 1e-15)
            ? motor_current * std::abs(Zbus(mi, fault_idx)) / Zk_abs
            : 0.0;

        if (motor_current > 1e-15) {
          double sn = (m.sn_mva > 1e-6) ? m.sn_mva : 1.0;
          double Ik_ratio = motor_current * std::sqrt(3.0) * mvn / sn;
          double mu = compute_mu(Ik_ratio, t_break);
          double pn_mw = sn * m.cos_phi * m.efficiency;
          // IEC 60909 q factor uses PrM/p with p = pole pairs per motor
          // (AsynchronousMotor::poles carries the IEC pole-pair count).
          double q = compute_q(pn_mw, m.poles, t_break);
          Ib += mu * q * motor_current_tr;
        }
      }

      for (const auto& ld : ac.loads) {
        if (!ld.in_service) continue;
        const double motor_fraction = load_motor_fraction(ld.motor_percent);
        if (motor_fraction <= 1e-6 || ld.sn_mva <= 1e-6) continue;
        if (load_motor_active_power_mw(ld, motor_fraction) < kMinMotorContributionMw) continue;
        auto lit = id_map.find(ld.bus);
        if (lit == id_map.end()) continue;
        const double actual_s = ld.sn_mva * motor_fraction;
        if (actual_s < 1e-12) continue;
        const Cx z_load = Cx(ld.r_sc_pu, ld.x_sub_pu) * (base_mva / actual_s);
        if (std::abs(z_load) <= 1e-15) continue;
        const double motor_current = (c / std::abs(z_load)) * I_base;
        const double Zk_abs = std::abs(Zbus(fault_idx, fault_idx) + Zf);
        const double motor_current_tr =
            (Zk_abs > 1e-15)
                ? motor_current * std::abs(Zbus(lit->second, fault_idx)) / Zk_abs
                : 0.0;
        if (motor_current <= 1e-15) continue;
        const double sn = (actual_s > 1e-6) ? actual_s : 1.0;
        const double Ik_ratio = motor_current * std::sqrt(3.0) * bus_kv(ld.bus) / sn;
        const double mu = compute_mu(Ik_ratio, t_break);
        const double q = compute_q(
            load_motor_pn_mw(ld, motor_fraction),
            ld.motor_poles,
            t_break);
        Ib += mu * q * motor_current_tr;
      }

      // Network (no-motor) contribution passes through directly
      Ib += row.ikss_no_motor_ka;

      row.ib_ka = Ib;
    }
  }

  // ====== Step 7: Steady-state current (Ik) ======
  {
    auto [Ybus_s, Ybus2_s, Ybus0_s] = build_sc_admittance_matrices(
        ac, projected.vsc_converters, id_map, n, opt, transformer_corrections, true);
    Eigen::MatrixXcd Zbus_s = safe_inverse(Ybus_s);

    Cx Zk_s = compute_Zk(opt.fault_type,
                          Zbus_s(fault_idx, fault_idx),
                          Zbus0(fault_idx, fault_idx), Zf);
    double Ik_1_pu = (std::abs(Zk_s) > 1e-15) ? (c / std::abs(Zk_s)) : 0.0;
    double Ik_1_kA = Ik_1_pu * I_base;

    for (int k = 0; k < n; ++k) {
      auto& row = out.bus_results[k];
      double Ik = 0.0;

      // Generator lambda_max contribution
      for (const auto& g : ac.generators) {
        if (!g.in_service) continue;
        auto git = id_map.find(g.bus);
        if (git == id_map.end()) continue;
        int gi = git->second;

        double u_rg = (g.vn_kv > 1e-6) ? g.vn_kv : fault_kv;
        double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
        double I_rG = mbase / (std::sqrt(3.0) * u_rg);  // kA

        double xd_xq = (g.xd_xq > 1e-6) ? g.xd_xq : 1.0;
        double lambda_max;
        if (xd_xq <= 1.2)
          lambda_max = 2.8;
        else if (xd_xq >= 1.5)
          lambda_max = 5.0;
        else
          lambda_max = 2.8 + (5.0 - 2.8) * (xd_xq - 1.2) / (1.5 - 1.2);

        double tr_ratio = (std::abs(Zk_s) > 1e-15)
            ? std::abs(Zbus_s(gi, fault_idx)) / std::abs(Zk_s)
            : 0.0;

        Ik += lambda_max * I_rG * tr_ratio;
      }

      // Network (no-motor) part from steady-state Zbus
      if (k == fault_idx) {
        double gen_ss_contrib = 0.0;
        for (const auto& g : ac.generators) {
          if (!g.in_service) continue;
          auto git = id_map.find(g.bus);
          if (git == id_map.end()) continue;

          double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
          double xd = (g.xd_pu > 1e-6) ? g.xd_pu : opt.default_xdpp;
          double cos_phi = std::clamp(g.cos_phi, 0.01, 1.0);
          double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
          double KG = c / (1.0 + xd * sin_phi);
          Cx z_gen = Cx(g.ra_pu, xd) * (base_mva / mbase) * KG;
          if (std::abs(z_gen) > 1e-15)
            gen_ss_contrib += (c / std::abs(z_gen)) * I_base;
        }
        Ik += (Ik_1_kA - gen_ss_contrib);
      } else {
        // Other buses: transfer from steady-state
        double I_ss_pu = (std::abs(Zk_s) > 1e-15 && std::abs(Zbus_s(k, k)) > 1e-15)
            ? c * std::abs(Zbus_s(k, fault_idx)) / (std::abs(Zk_s) * std::abs(Zbus_s(k, k)))
            : 0.0;
        double gen_ss_k = 0.0;
        int bus_id = ac.buses[k].index;
        for (const auto& g : ac.generators) {
          if (!g.in_service || g.bus != bus_id) continue;
          auto git = id_map.find(g.bus);
          if (git == id_map.end()) continue;
          double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
          double xd = (g.xd_pu > 1e-6) ? g.xd_pu : opt.default_xdpp;
          double cos_phi = std::clamp(g.cos_phi, 0.01, 1.0);
          double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
          double KG = c / (1.0 + xd * sin_phi);
          Cx z_gen = Cx(g.ra_pu, xd) * (base_mva / mbase) * KG;
          if (std::abs(z_gen) > 1e-15 && std::abs(Zk_s) > 1e-15)
            gen_ss_k += c * std::abs(Zbus_s(k, fault_idx)) / (std::abs(z_gen) * std::abs(Zk_s)) * I_base;
        }
        Ik += (I_ss_pu * I_base - gen_ss_k);
      }

      row.ik_ka = std::max(0.0, Ik);
    }
  }

  // ====== Step 8: Thermal equivalent current I_th (IEC 60909 §8) ======
  if (opt.compute_ith) {
    const double Tk = opt.ith_duration_s;
    for (int k = 0; k < n; ++k) {
      auto& row = out.bus_results[k];

      // I_th = I_kss * sqrt(m + n)
      // m: DC component heat factor, n: AC decay heat factor
      // Simplified: m = (1/(2*f*Tk)) * (exp(-4*pi*f*Tk*R/X) - 1) / (-4*pi*f*R/X) + 1
      //             n ≈ 1 for constant AC component

      Cx Z_k_th = (k == fault_idx) ? Z1_fault : Zbus(k, k);
      double rx_ratio = (std::abs(std::imag(Z_k_th)) > 1e-15)
                            ? std::abs(std::real(Z_k_th) / std::imag(Z_k_th))
                            : 0.0;

      double f = opt.base_frequency_hz;
      double omega = 2.0 * M_PI * f;

      // m factor: accounts for DC component decay
      double m_factor;
      if (rx_ratio < 1e-12 || Tk < 1e-12) {
        m_factor = 1.0;  // no DC decay
      } else {
        double tau = 1.0 / (omega * rx_ratio);  // DC time constant = X/(omega*R) = L/R
        double exp_term = std::exp(-2.0 * Tk / tau);
        m_factor = (1.0 / (2.0 * f * Tk)) * (1.0 - exp_term) / (2.0 / tau) + 1.0;
        // Simplified IEC formula: m = 1/(2fT) * [1 - e^(-4πfTR/X)]/(4πfR/X)
        double arg = 4.0 * M_PI * f * Tk * rx_ratio;
        if (arg > 1e-6) {
          m_factor = 1.0 / (2.0 * f * Tk) * (1.0 - std::exp(-arg)) / arg;
        }
      }

      // n factor: accounts for AC current decay (generators, motors)
      // Simplified: n ≈ 1 for most practical cases
      double n_factor = 1.0;

      row.ith_ka = row.ikss_ka * std::sqrt(m_factor + n_factor);
    }
  }

  // ====== Step 9: Branch fault currents and power flows ======
  if (opt.compute_branch_flows) {
    // Branch currents from voltage differences
    for (const auto& br : ac.branches) {
      if (!br.in_service) continue;
      auto it_f = id_map.find(br.from_bus);
      auto it_t = id_map.find(br.to_bus);
      if (it_f == id_map.end() || it_t == id_map.end()) continue;
      int fi = it_f->second, ti = it_t->second;

      if (std::abs(br.r_pu) < 1e-12 && std::abs(br.x_pu) < 1e-12) continue;

      Cx z_series(br.r_pu, br.x_pu);
      Cx y_series = Cx(1.0, 0.0) / z_series;

      double tap = (br.tap > 1e-9) ? br.tap : 1.0;
      double shift_rad = br.shift_deg * M_PI / 180.0;
      Cx t(tap * std::cos(shift_rad), tap * std::sin(shift_rad));

      // Current from-end: I_ft = y_series * (V_f/t - V_t) + y_shunt * V_f / |t|²
      Cx y_shunt(0.0, br.b_pu / 2.0);
      Cx V_f = V_fault[fi];
      Cx V_t = V_fault[ti];

      Cx I_ft, I_tf;
      if (std::abs(t - Cx(1.0, 0.0)) < 1e-9) {
        I_ft = y_series * (V_f - V_t) + y_shunt * V_f;
        I_tf = y_series * (V_t - V_f) + y_shunt * V_t;
      } else {
        I_ft = y_series * (V_f / t - V_t) + y_shunt * V_f / std::norm(t);
        I_tf = y_series * (V_t - V_f / t) + y_shunt * V_t;
      }

      SCDetailedBranchResult br_res;
      br_res.from_bus = ext_bus_id(br.from_bus);
      br_res.to_bus = ext_bus_id(br.to_bus);
      br_res.branch_index = br.index;
      br_res.i_from_ka = std::abs(I_ft) * I_base;
      br_res.i_to_ka = std::abs(I_tf) * I_base;
      br_res.i_branch_ka = std::max(br_res.i_from_ka, br_res.i_to_ka);

      // Apparent power: S = V * I* * S_base
      double s_base = base_mva;
      br_res.s_branch_mva = std::max(
          std::abs(V_f * std::conj(I_ft)) * s_base,
          std::abs(V_t * std::conj(I_tf)) * s_base);

      out.branch_results.push_back(br_res);
    }
  }

  if (projected.bus_merge_map) {
    const auto positions =
        projection::CanonicalToRichOperator::ac_bus_reprojection_positions(
            sys, projection_bundle);
    std::vector<SCDetailedBusResult> attributed_results;
    attributed_results.reserve(sys.ac.buses.size());
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      const int position = positions[i];
      if (position < 0 || position >= static_cast<int>(out.bus_results.size())) {
        continue;
      }
      auto attributed = out.bus_results[static_cast<size_t>(position)];
      attributed.bus_id = sys.ac.buses[i].index;
      attributed_results.push_back(std::move(attributed));
    }
    out.bus_results = std::move(attributed_results);
  }

  out.solved = true;
  return out;
}

std::vector<SCDetailedResult> run_short_circuit_detailed_batch(const HybridPowerSystem& sys,
                                                               const std::vector<int>& fault_bus_ids,
                                                               const SCDetailedOptions& opt) {
  std::vector<SCDetailedResult> out;
  out.reserve(fault_bus_ids.size());
  for (const int bus_id : fault_bus_ids) {
    out.push_back(run_short_circuit_detailed(sys, bus_id, opt));
  }
  return out;
}

}  // namespace hacdcpf::analysis
