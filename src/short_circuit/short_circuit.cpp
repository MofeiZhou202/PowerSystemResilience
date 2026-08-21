#define _USE_MATH_DEFINES  // M_PI on strict-conformance toolchains (MSYS2 UCRT, MSVC)
// Short Circuit Analysis — Z-bus method
// DistributionPowerFlow.jl ShortCircuit module equivalent (C++20)
//
// Algorithm: for three-phase faults, the fault admittance is added at the
// faulted bus and the bus impedance matrix Z_bus = Y_fault^{-1} is used.
// Z_kk gives the Thevenin impedance; I_f = c·V0 / Z_kk.
//
// The overview path uses a documented Z1=Z2=Z0 approximation for
// unsymmetrical faults.  The detailed IEC 60909 path assembles independent
// positive-, negative-, and componentwise zero-sequence networks and
// reconstructs the fault-point phase and ground currents.

#include "hacdcpf/analysis/short_circuit.hpp"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <complex>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#if defined(HACDCPF_OPF_HAVE_KLU)
#include <Eigen/KLUSupport>
#endif

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
using RowSpMat = Eigen::SparseMatrix<Cx, Eigen::RowMajor>;
#if defined(HACDCPF_OPF_HAVE_KLU)
using SpLU = Eigen::KLU<SpMat>;
#else
using SpLU  = Eigen::SparseLU<SpMat>;
#endif

class RowSparseAccumulator : public RowSpMat {
 public:
  using RowSpMat::RowSpMat;

  Cx& operator()(Eigen::Index row, Eigen::Index col) {
    return coeffRef(row, col);
  }
};

constexpr double kMinMotorContributionMw = 0.05;
// IEC 60909-0:2016, 4.3.1.2 method C uses fc = 20 Hz at 50 Hz and
// fc = 24 Hz at 60 Hz, hence the invariant ratio fc/f = 0.4.
constexpr double kMethodCFrequencyRatio = 0.4;
// Backward-error admission threshold for selected sparse solves. Higham,
// Accuracy and Stability of Numerical Algorithms (2nd ed.), sec. 7.2; the
// module validation contract fixes 1e-9 in docs/modules/short_circuit.
constexpr double kLinearResidualTolerance = 1e-9;

void validate_detailed_options(const SCDetailedOptions& opt) {
  const auto finite = [](double value) { return std::isfinite(value); };
  if (!finite(opt.c_factor) || opt.c_factor < 0.0 || opt.c_factor > 2.0)
    throw std::invalid_argument("SCDetailedOptions.c_factor must be finite and in [0, 2]");
  if (!finite(opt.fault_impedance_pu) || opt.fault_impedance_pu < 0.0)
    throw std::invalid_argument("SCDetailedOptions.fault_impedance_pu must be finite and non-negative");
  if (!finite(opt.breaking_time_s) || opt.breaking_time_s <= 0.0)
    throw std::invalid_argument("SCDetailedOptions.breaking_time_s must be finite and positive");
  if (!finite(opt.base_frequency_hz) || opt.base_frequency_hz <= 0.0)
    throw std::invalid_argument("SCDetailedOptions.base_frequency_hz must be finite and positive");
  if (!finite(opt.default_xdpp) || opt.default_xdpp <= 0.0)
    throw std::invalid_argument("SCDetailedOptions.default_xdpp must be finite and positive");
  if (!finite(opt.ith_duration_s) || (opt.compute_ith && opt.ith_duration_s <= 0.0))
    throw std::invalid_argument("SCDetailedOptions.ith_duration_s must be finite and positive when I_th is requested");
  if (opt.inverse_rhs_batch_size == 0 || opt.inverse_rhs_batch_size > 256)
    throw std::invalid_argument("SCDetailedOptions.inverse_rhs_batch_size must be in [1, 256]");
}

void validate_options(const SCOptions& opt) {
  if (!std::isfinite(opt.c_factor) || opt.c_factor <= 0.0 || opt.c_factor > 2.0)
    throw std::invalid_argument("SCOptions.c_factor must be finite and in (0, 2]");
  if (!std::isfinite(opt.default_xdpp) || opt.default_xdpp <= 0.0)
    throw std::invalid_argument("SCOptions.default_xdpp must be finite and positive");
  if (opt.inverse_rhs_batch_size == 0 || opt.inverse_rhs_batch_size > 256)
    throw std::invalid_argument("SCOptions.inverse_rhs_batch_size must be in [1, 256]");
}

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

double converter_current_multiplier_sc(const VSCConverter& conv) {
  if (conv.i_max_pu > 1e-6 && std::abs(conv.i_max_pu - 1.0) > 1e-9)
    return conv.i_max_pu;
  if (conv.i_ac_max_pu > 1e-6) return conv.i_ac_max_pu;
  return conv.i_max_pu > 1e-6 ? conv.i_max_pu : 1.0;
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
  std::optional<Cx> z1_override;
};

struct SequenceAdmittanceStamp {
  int row{0};
  int col{0};
  Cx value{0.0, 0.0};
};

struct IecNetworkCorrections {
  std::unordered_map<int, TransformerBranchCorrection> transformer_branches;
  std::unordered_map<int, double> generator_impedance_factors;
  std::unordered_set<int> replaced_positive_sequence_branches;
  std::unordered_set<int> replaced_zero_sequence_branches;
  std::vector<SequenceAdmittanceStamp> positive_sequence_stamps;
  std::vector<SequenceAdmittanceStamp> zero_sequence_stamps;
  bool valid{true};
  std::string message;
};

double generator_voltage_scale(const Generator& generator, double bus_kv) {
  if (generator.vn_kv <= 1e-9 || bus_kv <= 1e-9) return 1.0;
  return std::pow(generator.vn_kv / bus_kv, 2.0);
}

double generator_kg(const Generator& generator,
                    double bus_kv,
                    double c,
                    double xdpp) {
  const double cos_phi = std::clamp(generator.cos_phi, 0.01, 1.0);
  const double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
  const double voltage_ratio = generator.vn_kv > 1e-9
      ? bus_kv / (generator.vn_kv * (1.0 + generator.pg_percent / 100.0))
      : 1.0;
  // IEC 60909-0:2016, eq. (18): generator impedance correction including
  // permanent terminal-voltage deviation from rated voltage.
  return voltage_ratio * c / (1.0 + xdpp * sin_phi);
}

std::string vector_group_without_clock(std::string value) {
  value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char ch) {
                return std::isdigit(ch) || std::isspace(ch);
              }), value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  // A two-winding unit may declare a stabilizing delta tertiary as "+d".
  // It does not add an external terminal; the two authored winding
  // connectivity remains the prefix (IEC 60909-0:2016, 6.3.2).
  if (const auto tertiary = value.find("+d"); tertiary != std::string::npos)
    value.erase(tertiary);
  return value;
}

enum class ZeroSequenceWinding { Ungrounded, GroundedWye, Delta, Unsupported };

std::vector<ZeroSequenceWinding> parse_vector_group_windings(
    const std::string& authored) {
  std::string value;
  value.reserve(authored.size());
  for (unsigned char ch : authored) {
    if (!std::isdigit(ch) && !std::isspace(ch)) value.push_back(static_cast<char>(ch));
  }
  std::vector<ZeroSequenceWinding> result;
  for (size_t i = 0; i < value.size();) {
    const char ch = static_cast<char>(std::tolower(static_cast<unsigned char>(value[i])));
    if (ch != 'y' && ch != 'd' && ch != 'z') return {};
    const bool neutral = i + 1 < value.size() &&
        std::tolower(static_cast<unsigned char>(value[i + 1])) == 'n';
    if (ch == 'd') result.push_back(ZeroSequenceWinding::Delta);
    else if (ch == 'y' || ch == 'z') result.push_back(
        neutral ? ZeroSequenceWinding::GroundedWye
                : ZeroSequenceWinding::Ungrounded);
    i += neutral ? 2 : 1;
  }
  return result;
}

Cx impedance_from_percent(double vk_percent,
                          double vkr_percent,
                          double base_mva,
                          double rating_mva,
                          double voltage_base_scale) {
  const double scale = base_mva / std::max(1e-9, rating_mva);
  const double z = std::max(0.0, vk_percent / 100.0) * scale;
  const double r = std::max(0.0, vkr_percent / 100.0) * scale;
  return Cx(r, std::sqrt(std::max(0.0, z * z - r * r))) *
         voltage_base_scale;
}

double transformer_kt(double vk_percent,
                      double vkr_percent,
                      double c) {
  const double z = std::max(0.0, vk_percent / 100.0);
  const double r = std::max(0.0, vkr_percent / 100.0);
  const double x = std::sqrt(std::max(0.0, z * z - r * r));
  // IEC 60909-0:2016, 3.3.3 and eq. (12): K_T is applied to the
  // pair short-circuit impedance before the 3W star/delta conversion.
  return 0.95 * c / (1.0 + 0.6 * x);
}

std::array<Cx, 3> three_winding_star_from_pair(const std::array<Cx, 3>& pair) {
  // IEC 60909-0:2016, 3.3.3. Pair order is HV-MV, HV-LV, MV-LV.
  return {0.5 * (pair[0] + pair[1] - pair[2]),
          0.5 * (pair[0] + pair[2] - pair[1]),
          0.5 * (pair[1] + pair[2] - pair[0])};
}

IecNetworkCorrections build_iec_network_corrections(
    const HybridPowerSystem& authored,
    const HybridPowerSystem& projected,
    const SCDetailedOptions& opt,
    int station_internal_fault_bus_id = 0,
    double positive_sequence_reactance_scale = 1.0) {
  IecNetworkCorrections result;
  if (!projected.branch_expand_map) return result;
  const double base_mva = projected.ac.base_mva > 0.0
                              ? projected.ac.base_mva : 100.0;

  const auto authored_bus_kv = [&](int bus_id, double fallback) {
    for (const auto& bus : authored.ac.buses)
      if (bus.index == bus_id) return bus.base_kv > 1e-6 ? bus.base_kv : fallback;
    return fallback;
  };
  const auto projected_position = [&](int authored_bus_id) -> std::optional<int> {
    if (projected.bus_merge_map) {
      const auto it = projected.bus_merge_map->ext_to_int.find(authored_bus_id);
      if (it == projected.bus_merge_map->ext_to_int.end()) return std::nullopt;
      return it->second;
    }
    for (size_t i = 0; i < projected.ac.buses.size(); ++i)
      if (projected.ac.buses[i].index == authored_bus_id)
        return static_cast<int>(i);
    return std::nullopt;
  };
  const auto branch_by_index = [&](int branch_index) -> const ACBranch* {
    for (const auto& branch : projected.ac.branches)
      if (branch.index == branch_index) return &branch;
    return nullptr;
  };
  const auto origin_entries = [&](BranchOriginType type, int origin_index) {
    std::vector<const BranchExpandEntry*> entries;
    for (const auto& entry : projected.branch_expand_map->entries)
      if (entry.origin_type == type && entry.origin_index == origin_index)
        entries.push_back(&entry);
    std::sort(entries.begin(), entries.end(), [](const auto* lhs, const auto* rhs) {
      return lhs->pair_number < rhs->pair_number;
    });
    return entries;
  };
  const auto add_stamp = [&](int row, int col, Cx value) {
    if (std::abs(value) > 1e-30)
      result.zero_sequence_stamps.push_back({row, col, value});
  };
  const auto add_positive_stamp = [&](int row, int col, Cx value) {
    if (std::abs(value) > 1e-30)
      result.positive_sequence_stamps.push_back({row, col, value});
  };
  const auto add_series = [&](int from, int to, Cx y, double tap) {
    const double ratio = std::max(1e-9, std::abs(tap));
    add_stamp(from, from, y / (ratio * ratio));
    add_stamp(to, to, y);
    add_stamp(from, to, -y / ratio);
    add_stamp(to, from, -y / ratio);
  };

  for (const auto& transformer : authored.ac.transformers_2w) {
    if (!transformer.in_service || transformer.sn_mva <= 1e-9) continue;
    const auto entries = origin_entries(BranchOriginType::Transformer2W,
                                        transformer.index);
    if (entries.empty()) continue;
    const auto* branch = branch_by_index(entries.front()->branch_index);
    if (!branch) continue;

    TransformerBranchCorrection correction;
    const double lv_kv = authored_bus_kv(transformer.lv_bus,
                                         transformer.vn_lv_kv);
    const double c = get_voltage_factor_sc(lv_kv, SCCalcType::Max);
    if (opt.apply_iec_transformer_correction)
      correction.k1 = transformer_kt(transformer.vk_percent,
                                     transformer.vkr_percent, c);
    if (transformer.power_station_unit) {
      const Generator* station_generator = nullptr;
      for (const auto& generator : authored.ac.generators) {
        if (!generator.in_service ||
            generator.power_station_transformer_index != transformer.index)
          continue;
        if (station_generator) {
          result.valid = false;
          result.message = "Transformer2W " + std::to_string(transformer.index) +
              " is associated with more than one in-service generator";
          return result;
        }
        station_generator = &generator;
      }
      if (!station_generator) {
        result.valid = false;
        result.message = "power-station Transformer2W " +
            std::to_string(transformer.index) +
            " has no associated in-service generator";
        return result;
      }
      const double vq = authored_bus_kv(transformer.hv_bus,
                                        transformer.vn_hv_kv);
      const double vg = station_generator->vn_kv > 1e-9
          ? station_generator->vn_kv : lv_kv;
      const double cos_phi = std::clamp(station_generator->cos_phi, 0.01, 1.0);
      const double sin_phi = std::sqrt(std::max(0.0, 1.0 - cos_phi * cos_phi));
      const double xg = station_generator->xdpp_pu > 1e-9
          ? station_generator->xdpp_pu : opt.default_xdpp;
      const double zt = transformer.vk_percent / 100.0;
      const double rt = transformer.vkr_percent / 100.0;
      const double xt = std::sqrt(std::max(0.0, zt * zt - rt * rt));
      double ks = 1.0;
      if (transformer.oltc) {
        // IEC 60909-0:2016, eq. (21), power-station unit with OLTC.
        ks = std::pow(vq / vg, 2.0) *
             std::pow(transformer.vn_lv_kv / transformer.vn_hv_kv, 2.0) *
             c / (1.0 + std::abs(xg - xt) * sin_phi);
      } else {
        // IEC 60909-0:2016, eq. (22), power-station unit without OLTC.
        const double pg = station_generator->pg_percent / 100.0;
        const double pt = transformer.pt_percent / 100.0;
        ks = (vq / (vg * (1.0 + pg))) *
             (transformer.vn_lv_kv / transformer.vn_hv_kv) *
             (1.0 - pt) * c / (1.0 + xg * sin_phi);
      }
      if (!(ks > 0.0) || !std::isfinite(ks)) {
        result.valid = false;
        result.message = "power-station correction K_S is invalid for Transformer2W " +
            std::to_string(transformer.index);
        return result;
      }
      if (station_generator->bus == station_internal_fault_bus_id) {
        correction.k1 = 1.0;
        const double kg_inside = transformer.oltc
            ? c / (1.0 + xg * sin_phi)
            : c / ((1.0 + station_generator->pg_percent / 100.0) *
                   (1.0 + xg * sin_phi));
        result.generator_impedance_factors[station_generator->index] =
            kg_inside;
      } else {
        correction.k1 = ks;
        result.generator_impedance_factors[station_generator->index] = ks;
      }
    }
    correction.k0 = correction.k1;
    result.transformer_branches[branch->index] = correction;

    if (opt.fault_type != FaultType::SinglePhaseGround &&
        opt.fault_type != FaultType::TwoPhaseGround) continue;
    const std::string group = vector_group_without_clock(transformer.vector_group);
    if (group.empty()) continue;  // Backward-compatible authored branch Z0.
    const auto windings = parse_vector_group_windings(group);
    if (windings.size() != 2 ||
        std::find(windings.begin(), windings.end(),
                  ZeroSequenceWinding::Unsupported) != windings.end()) {
      result.valid = false;
      result.message = "Transformer2W " + std::to_string(transformer.index) +
          " uses unsupported zero-sequence vector group " +
          transformer.vector_group;
      return result;
    }
    result.replaced_zero_sequence_branches.insert(branch->index);

    const auto hv = projected_position(transformer.hv_bus);
    const auto lv = projected_position(transformer.lv_bus);
    if (!hv || !lv) continue;
    const bool grounded_hv = windings[0] == ZeroSequenceWinding::GroundedWye;
    const bool grounded_lv = windings[1] == ZeroSequenceWinding::GroundedWye;
    if (!grounded_hv && !grounded_lv) continue;
    if (transformer.z0_percent <= 0.0) {
      result.valid = false;
      result.message = "Transformer2W " + std::to_string(transformer.index) +
          " has grounded vector group " + transformer.vector_group +
          " but no positive z0_percent";
      return result;
    }

    const double xr = transformer.x0_r0;
    const double r0_percent = xr > 1e-9
        ? transformer.z0_percent / std::sqrt(1.0 + xr * xr) : 0.0;
    auto side_impedance = [&](bool high_voltage_side) {
      const double nameplate = high_voltage_side ? transformer.vn_hv_kv
                                                  : transformer.vn_lv_kv;
      const double bus_kv = authored_bus_kv(
          high_voltage_side ? transformer.hv_bus : transformer.lv_bus,
          nameplate);
      const double voltage_scale = nameplate > 1e-9 && bus_kv > 1e-9
          ? std::pow(nameplate / bus_kv, 2.0) : 1.0;
      return impedance_from_percent(transformer.z0_percent, r0_percent,
                                     base_mva, transformer.sn_mva,
                                     voltage_scale) * correction.k0;
    };
    if (grounded_hv && grounded_lv) {
      Cx z0(branch->r0_pu, branch->x0_pu);
      z0 *= correction.k0;
      if (std::abs(z0) > 1e-15)
        add_series(*hv, *lv, Cx(1.0, 0.0) / z0, branch->tap);
      continue;
    }

    Cx z0 = side_impedance(grounded_hv);
    const auto opposite = windings[grounded_hv ? 1U : 0U];
    if (opposite == ZeroSequenceWinding::Ungrounded &&
        transformer.mag0_percent <= 0.0) {
      result.valid = false;
      result.message = "Transformer2W " + std::to_string(transformer.index) +
          " requires mag0_percent for an ungrounded-wye return path";
      return result;
    }
    if (opposite == ZeroSequenceWinding::Ungrounded)
      z0 *= 1.0 + transformer.mag0_percent / 100.0;
    if (transformer.xn_ohm != 0.0) {
      const double bus_kv = authored_bus_kv(
          grounded_hv ? transformer.hv_bus : transformer.lv_bus,
          grounded_hv ? transformer.vn_hv_kv : transformer.vn_lv_kv);
      z0 += Cx(0.0, 3.0 * transformer.xn_ohm * base_mva /
                         std::max(1e-9, bus_kv * bus_kv));
    }
    if (std::abs(z0) > 1e-15)
      add_stamp(grounded_hv ? *hv : *lv, grounded_hv ? *hv : *lv,
                Cx(1.0, 0.0) / z0);
  }

  for (const auto& transformer : authored.ac.transformers_3w) {
    if (!transformer.in_service) continue;
    const auto entries = origin_entries(BranchOriginType::Transformer3W,
                                        transformer.index);
    if (entries.size() != 3) continue;
    const std::array<double, 3> ratings = {
        std::min(transformer.sn_hv_mva, transformer.sn_mv_mva),
        std::min(transformer.sn_hv_mva, transformer.sn_lv_mva),
        std::min(transformer.sn_mv_mva, transformer.sn_lv_mva)};
    const std::array<double, 3> vk = {transformer.vk_hv_mv_percent,
                                      transformer.vk_hv_lv_percent,
                                      transformer.vk_mv_lv_percent};
    const std::array<double, 3> vkr = {transformer.vkr_hv_mv_percent,
                                       transformer.vkr_hv_lv_percent,
                                       transformer.vkr_mv_lv_percent};
    const std::array<double, 3> winding_nameplate = {
        transformer.vn_hv_kv, transformer.vn_mv_kv, transformer.vn_lv_kv};
    const std::array<int, 3> winding_bus = {
        transformer.hv_bus, transformer.mv_bus, transformer.lv_bus};
    std::array<double, 3> kt{1.0, 1.0, 1.0};
    std::array<Cx, 3> corrected_pairs{};
    for (size_t i = 0; i < 3; ++i) {
      const double bus_kv = authored_bus_kv(winding_bus[std::min(i + 1, size_t{2})],
                                            winding_nameplate[std::min(i + 1, size_t{2})]);
      if (opt.apply_iec_transformer_correction)
        kt[i] = transformer_kt(vk[i], vkr[i],
            get_voltage_factor_sc(bus_kv, SCCalcType::Max));
      corrected_pairs[i] = impedance_from_percent(
          vk[i], vkr[i], base_mva, ratings[i], 1.0) * kt[i];
      corrected_pairs[i] = Cx(std::real(corrected_pairs[i]),
                              std::imag(corrected_pairs[i]) *
                                  positive_sequence_reactance_scale);
    }
    try {
      auto corrected_star = three_winding_star_from_pair(corrected_pairs);
      for (size_t i = 0; i < corrected_star.size(); ++i) {
        const double bus_kv = authored_bus_kv(winding_bus[i], winding_nameplate[i]);
        if (winding_nameplate[i] > 1e-9 && bus_kv > 1e-9)
          corrected_star[i] *= std::pow(winding_nameplate[i] / bus_kv, 2.0);
      }
      const std::array<std::optional<int>, 3> buses = {
          projected_position(transformer.hv_bus),
          projected_position(transformer.mv_bus),
          projected_position(transformer.lv_bus)};
      if (!buses[0] || !buses[1] || !buses[2]) continue;

      Eigen::Matrix4cd local = Eigen::Matrix4cd::Zero();
      const auto stamp_branch = [&](int from, int to, Cx impedance,
                                    double tap) {
        if (std::abs(impedance) <= 1e-15)
          throw std::invalid_argument("Transformer3W equivalent winding impedance is zero");
        const Cx y = Cx(1.0, 0.0) / impedance;
        const double ratio = std::max(1e-9, std::abs(tap));
        local(from, from) += y / (ratio * ratio);
        local(to, to) += y;
        local(from, to) -= y / ratio;
        local(to, from) -= y / ratio;
      };
      const double winding_tap = std::max(
          1e-9, 1.0 + transformer.tap_pos * transformer.tap_step_percent / 100.0);
      const double hv_tap = transformer.tap_side == 0 ? winding_tap : 1.0;
      const auto* hm_branch = branch_by_index(entries[0]->branch_index);
      const auto* hl_branch = branch_by_index(entries[1]->branch_index);
      if (!hm_branch || !hl_branch)
        throw std::invalid_argument("Transformer3W canonical pair branch is missing");
      stamp_branch(0, 3, corrected_star[0], hv_tap);
      stamp_branch(3, 1, corrected_star[1], hm_branch->tap / hv_tap);
      stamp_branch(3, 2, corrected_star[2], hl_branch->tap / hv_tap);
      if (std::abs(local(3, 3)) <= 1e-15)
        throw std::invalid_argument("Transformer3W internal star admittance is singular");
      const Eigen::Matrix3cd reduced = local.topLeftCorner<3, 3>() -
          local.topRightCorner<3, 1>() * local.bottomLeftCorner<1, 3>() /
              local(3, 3);
      for (const auto* entry : entries)
        result.replaced_positive_sequence_branches.insert(entry->branch_index);
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          add_positive_stamp(*buses[static_cast<size_t>(i)],
                             *buses[static_cast<size_t>(j)], reduced(i, j));
      for (size_t i = 0; i < entries.size(); ++i) {
        TransformerBranchCorrection correction;
        correction.k0 = kt[i];
        result.transformer_branches[entries[i]->branch_index] = correction;
      }
    } catch (const std::invalid_argument& error) {
      result.valid = false;
      result.message = "Transformer3W " + std::to_string(transformer.index) +
          ": " + error.what();
      return result;
    }

    if (opt.fault_type != FaultType::SinglePhaseGround &&
        opt.fault_type != FaultType::TwoPhaseGround) continue;
    const auto windings = parse_vector_group_windings(transformer.vector_group);
    if (windings.empty() && transformer.vector_group.empty()) continue;
    if (windings.size() != 3 ||
        std::find(windings.begin(), windings.end(),
                  ZeroSequenceWinding::Unsupported) != windings.end()) {
      result.valid = false;
      result.message = "Transformer3W " + std::to_string(transformer.index) +
          " uses unsupported zero-sequence vector group " +
          transformer.vector_group;
      return result;
    }
    for (const auto* entry : entries)
      result.replaced_zero_sequence_branches.insert(entry->branch_index);

    const std::array<double, 3> vk0 = {transformer.vk0_hv_mv_percent,
                                       transformer.vk0_hv_lv_percent,
                                       transformer.vk0_mv_lv_percent};
    const std::array<double, 3> vkr0 = {transformer.vkr0_hv_mv_percent,
                                        transformer.vkr0_hv_lv_percent,
                                        transformer.vkr0_mv_lv_percent};
    if (std::any_of(vk0.begin(), vk0.end(), [](double value) { return value <= 0.0; })) {
      result.valid = false;
      result.message = "Transformer3W " + std::to_string(transformer.index) +
          " has a grounded/delta vector group but incomplete zero-sequence pair data";
      return result;
    }
    std::array<Cx, 3> zero_pairs{};
    for (size_t i = 0; i < 3; ++i) {
      zero_pairs[i] = impedance_from_percent(
          vk0[i], vkr0[i], base_mva, ratings[i], 1.0) * kt[i];
    }
    auto star = three_winding_star_from_pair(zero_pairs);
    for (size_t i = 0; i < star.size(); ++i) {
      const double bus_kv = authored_bus_kv(winding_bus[i], winding_nameplate[i]);
      if (winding_nameplate[i] > 1e-9 && bus_kv > 1e-9)
        star[i] *= std::pow(winding_nameplate[i] / bus_kv, 2.0);
    }
    const std::array<std::optional<int>, 3> buses = {
        projected_position(transformer.hv_bus),
        projected_position(transformer.mv_bus),
        projected_position(transformer.lv_bus)};
    if (!buses[0] || !buses[1] || !buses[2]) continue;
    const auto* hm_branch = branch_by_index(entries[0]->branch_index);
    const auto* hl_branch = branch_by_index(entries[1]->branch_index);
    if (!hm_branch || !hl_branch) {
      result.valid = false;
      result.message = "Transformer3W " + std::to_string(transformer.index) +
          " canonical zero-sequence pair branch is missing";
      return result;
    }
    const double winding_tap = std::max(
        1e-9, 1.0 + transformer.tap_pos * transformer.tap_step_percent / 100.0);
    const double hv_tap = transformer.tap_side == 0 ? winding_tap : 1.0;
    const std::array<double, 3> star_side_taps = {
        hv_tap, hm_branch->tap / hv_tap, hl_branch->tap / hv_tap};

    Eigen::Matrix4cd local = Eigen::Matrix4cd::Zero();
    const auto stamp_grounded_winding = [&](size_t winding, bool hv_side) {
      if (std::abs(star[winding]) <= 1e-15)
        throw std::invalid_argument(
            "Transformer3W zero-sequence winding impedance is zero");
      const Cx y = Cx(1.0, 0.0) / star[winding];
      const double ratio = std::max(1e-9, std::abs(star_side_taps[winding]));
      const int terminal = static_cast<int>(winding);
      if (hv_side) {
        local(terminal, terminal) += y / (ratio * ratio);
        local(3, 3) += y;
        local(terminal, 3) -= y / ratio;
        local(3, terminal) -= y / ratio;
      } else {
        local(3, 3) += y / (ratio * ratio);
        local(terminal, terminal) += y;
        local(3, terminal) -= y / ratio;
        local(terminal, 3) -= y / ratio;
      }
    };
    try {
      for (size_t i = 0; i < 3; ++i) {
        if (windings[i] == ZeroSequenceWinding::GroundedWye) {
          stamp_grounded_winding(i, i == 0);
        } else if (windings[i] == ZeroSequenceWinding::Delta) {
          if (std::abs(star[i]) <= 1e-15)
            throw std::invalid_argument(
                "Transformer3W delta zero-sequence impedance is zero");
          const Cx y = Cx(1.0, 0.0) / star[i];
          const double ratio = std::max(1e-9, std::abs(star_side_taps[i]));
          // IEC 60909-0:2016, 6.3.2: a delta winding supplies a closed
          // zero-sequence circulation path at the internal star point while
          // remaining open at its external line terminals.
          local(3, 3) += i == 0 ? y : y / (ratio * ratio);
        }
      }
    } catch (const std::invalid_argument& error) {
      result.valid = false;
      result.message = "Transformer3W " + std::to_string(transformer.index) +
          ": " + error.what();
      return result;
    }
    if (std::abs(local(3, 3)) <= 1e-15) continue;
    const Eigen::Matrix3cd reduced = local.topLeftCorner<3, 3>() -
        local.topRightCorner<3, 1>() * local.bottomLeftCorner<1, 3>() /
            local(3, 3);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        add_stamp(*buses[static_cast<size_t>(i)],
                  *buses[static_cast<size_t>(j)], reduced(i, j));
  }
  return result;
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
  trips.reserve(6 * ac_sys.branches.size() + 2 * ac_sys.generators.size() +
                ac_sys.external_grids.size());

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

  // IEC 60909 equivalent-voltage-source model: an external grid is a source
  // behind its short-circuit impedance, just like the detailed sequence path.
  SCDetailedOptions detailed_opt;
  detailed_opt.c_factor = opt.c_factor;
  detailed_opt.default_xdpp = opt.default_xdpp;
  const double base_mva = ac_sys.base_mva > 0.0 ? ac_sys.base_mva : 100.0;
  for (const auto& eg : ac_sys.external_grids) {
    if (!eg.in_service) continue;
    const auto it = id_map.find(eg.bus);
    if (it == id_map.end()) continue;
    double bus_kv = 1.0;
    for (const auto& bus : ac_sys.buses) {
      if (bus.index == eg.bus) {
        bus_kv = bus.base_kv > 1e-6 ? bus.base_kv : 1.0;
        break;
      }
    }
    const auto impedance =
        external_grid_impedance_sc(eg, bus_kv, base_mva, detailed_opt);
    if (std::abs(impedance.z1) > 1e-15) {
      trips.emplace_back(
          it->second, it->second, Cx(1.0, 0.0) / impedance.z1);
    }
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

class SparseInverseSolver {
 public:
  explicit SparseInverseSolver(const SpMat& matrix)
      : matrix_(matrix), size_(matrix.rows()) {
    for (int col = 0; col < matrix_.outerSize(); ++col) {
      for (SpMat::InnerIterator it(matrix_, col); it; ++it) {
        if (!std::isfinite(std::real(it.value())) ||
            !std::isfinite(std::imag(it.value()))) {
          return;
        }
      }
    }
    if (matrix_.nonZeros() == 0) {
      // Nothing to factor (e.g. a network-only matrix with no
      // branches/external grids). KLU's analyzePattern fails on an empty
      // pattern and its factorize()/info() then assert in debug builds,
      // so bail out here and leave the solver invalid.
      return;
    }
    lu_.analyzePattern(matrix_);
    lu_.factorize(matrix_);
    valid_ = lu_.info() == Eigen::Success;
  }

  [[nodiscard]] bool valid() const noexcept { return valid_; }
  [[nodiscard]] double max_residual() const noexcept { return max_residual_; }

  Eigen::VectorXcd inverse_column(int column) {
    Eigen::VectorXcd result = Eigen::VectorXcd::Zero(size_);
    if (!valid_ || column < 0 || column >= size_)
      throw std::runtime_error("invalid sparse short-circuit factor or inverse-column index");
    Eigen::VectorXcd rhs = Eigen::VectorXcd::Zero(size_);
    rhs[column] = Cx(1.0, 0.0);
    result = lu_.solve(rhs);
    if (lu_.info() != Eigen::Success || !result.allFinite())
      throw std::runtime_error("sparse short-circuit inverse-column solve failed");
    const double residual = (matrix_ * result - rhs).norm() /
        std::max(1.0, matrix_.norm() * result.norm() + rhs.norm());
    max_residual_ = std::max(max_residual_, residual);
    if (!std::isfinite(residual) || residual > kLinearResidualTolerance)
      throw std::runtime_error("sparse short-circuit inverse-column residual exceeds 1e-9");
    return result;
  }

  std::vector<Cx> inverse_diagonal(
      size_t rhs_batch_size,
      const std::function<bool()>& cancellation_requested) {
    std::vector<Cx> diagonal(static_cast<size_t>(size_), Cx(0.0, 0.0));
    if (!valid_) throw std::runtime_error("invalid sparse short-circuit factor");

    const int batch_size = static_cast<int>(
        std::clamp<size_t>(rhs_batch_size, 1, 256));
    for (int start = 0; start < size_; start += batch_size) {
      if (cancellation_requested && cancellation_requested()) {
        throw std::runtime_error("short-circuit analysis cancelled");
      }
      const int width = std::min(batch_size, size_ - start);
      Eigen::MatrixXcd rhs = Eigen::MatrixXcd::Zero(size_, width);
      for (int j = 0; j < width; ++j) rhs(start + j, j) = Cx(1.0, 0.0);
      // Y^{-1}e_k is exactly column k of Zbus.  Only its kth entry is retained;
      // see Davis (2006), Ch. 3, and docs/short_circuit_rich_acdc_derivation.md.
      const Eigen::MatrixXcd columns = lu_.solve(rhs);
      if (lu_.info() != Eigen::Success || !columns.allFinite())
        throw std::runtime_error("sparse short-circuit inverse-diagonal solve failed");
      const double residual = (matrix_ * columns - rhs).norm() /
          std::max(1.0, matrix_.norm() * columns.norm() + rhs.norm());
      max_residual_ = std::max(max_residual_, residual);
      if (!std::isfinite(residual) || residual > kLinearResidualTolerance)
        throw std::runtime_error("sparse short-circuit inverse-diagonal residual exceeds 1e-9");
      for (int j = 0; j < width; ++j) {
        diagonal[static_cast<size_t>(start + j)] = columns(start + j, j);
      }
    }
    return diagonal;
  }

 private:
  SpMat matrix_;
  SpLU lu_;
  int size_{0};
  bool valid_{false};
  double max_residual_{0.0};
};

// A zero-sequence network may contain both grounded components and physically
// open, ungrounded components. Factoring the full matrix would let one
// ungrounded island invalidate every SLG result, so IEC sequence networks are
// solved component by component (IEC 60909-0:2016, 4.2 and 4.3.2).
class ComponentwiseSparseInverseSolver {
 public:
  explicit ComponentwiseSparseInverseSolver(const SpMat& matrix)
      : size_(matrix.rows()), component_of_(static_cast<size_t>(matrix.rows()), -1),
        local_position_(static_cast<size_t>(matrix.rows()), -1) {
    if (matrix.rows() != matrix.cols() || matrix.rows() < 0) return;
    std::vector<std::vector<int>> adjacency(static_cast<size_t>(size_));
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (SpMat::InnerIterator it(matrix, col); it; ++it) {
        if (!std::isfinite(std::real(it.value())) ||
            !std::isfinite(std::imag(it.value()))) return;
        if (it.row() != it.col() && std::abs(it.value()) > 1e-30) {
          adjacency[static_cast<size_t>(it.row())].push_back(it.col());
          adjacency[static_cast<size_t>(it.col())].push_back(it.row());
        }
      }
    }

    for (int root = 0; root < size_; ++root) {
      if (component_of_[static_cast<size_t>(root)] >= 0) continue;
      const int component = static_cast<int>(components_.size());
      components_.push_back(Component{});
      std::queue<int> pending;
      pending.push(root);
      component_of_[static_cast<size_t>(root)] = component;
      while (!pending.empty()) {
        const int node = pending.front();
        pending.pop();
        auto& nodes = components_[static_cast<size_t>(component)].nodes;
        local_position_[static_cast<size_t>(node)] =
            static_cast<int>(nodes.size());
        nodes.push_back(node);
        for (const int neighbor : adjacency[static_cast<size_t>(node)]) {
          if (component_of_[static_cast<size_t>(neighbor)] >= 0) continue;
          component_of_[static_cast<size_t>(neighbor)] = component;
          pending.push(neighbor);
        }
      }
    }

    std::vector<std::vector<Eigen::Triplet<Cx>>> triplets(components_.size());
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (SpMat::InnerIterator it(matrix, col); it; ++it) {
        const int component = component_of_[static_cast<size_t>(it.row())];
        if (component != component_of_[static_cast<size_t>(it.col())]) return;
        triplets[static_cast<size_t>(component)].emplace_back(
            local_position_[static_cast<size_t>(it.row())],
            local_position_[static_cast<size_t>(it.col())], it.value());
      }
    }
    for (size_t i = 0; i < components_.size(); ++i) {
      const int width = static_cast<int>(components_[i].nodes.size());
      SpMat local(width, width);
      local.setFromTriplets(triplets[i].begin(), triplets[i].end());
      local.makeCompressed();
      components_[i].solver = std::make_unique<SparseInverseSolver>(local);
    }
    valid_ = true;
  }

  [[nodiscard]] bool valid() const noexcept { return valid_; }

  [[nodiscard]] bool has_finite_driving_point(int column) const {
    if (!valid_ || column < 0 || column >= size_) return false;
    const int component = component_of_[static_cast<size_t>(column)];
    return component >= 0 &&
           components_[static_cast<size_t>(component)].solver->valid();
  }

  Eigen::VectorXcd inverse_column(int column) {
    if (!has_finite_driving_point(column))
      throw std::runtime_error(
          "zero-sequence fault bus belongs to an ungrounded component");
    const int component = component_of_[static_cast<size_t>(column)];
    const auto& nodes = components_[static_cast<size_t>(component)].nodes;
    const auto local = components_[static_cast<size_t>(component)].solver->inverse_column(
        local_position_[static_cast<size_t>(column)]);
    Eigen::VectorXcd result = Eigen::VectorXcd::Zero(size_);
    for (size_t i = 0; i < nodes.size(); ++i) result[nodes[i]] = local[i];
    return result;
  }

  std::vector<Cx> inverse_diagonal(
      size_t,
      const std::function<bool()>& cancellation_requested) {
    std::vector<Cx> result(static_cast<size_t>(size_), Cx(0.0, 0.0));
    if (!valid_) throw std::runtime_error("invalid componentwise sequence factor");
    for (auto& component : components_) {
      if (cancellation_requested && cancellation_requested())
        throw std::runtime_error("short-circuit analysis cancelled");
      if (!component.solver->valid()) continue;
      for (size_t local = 0; local < component.nodes.size(); ++local) {
        const auto column = component.solver->inverse_column(
            static_cast<int>(local));
        result[static_cast<size_t>(component.nodes[local])] = column[local];
      }
    }
    return result;
  }

  [[nodiscard]] double max_residual() const {
    double value = 0.0;
    for (const auto& component : components_)
      if (component.solver)
        value = std::max(value, component.solver->max_residual());
    return value;
  }

 private:
  struct Component {
    std::vector<int> nodes;
    std::unique_ptr<SparseInverseSolver> solver;
  };

  int size_{0};
  std::vector<int> component_of_;
  std::vector<int> local_position_;
  std::vector<Component> components_;
  bool valid_{false};
};

// -------------------------------------------------------------------------
// Single-bus fault calculation
// -------------------------------------------------------------------------
BusFaultResult fault_at_bus_with_zkk(Cx Z_kk,
                                     int bus_id,
                                     double base_mva,
                                     double base_kv,
                                     const SCOptions& opt) {
  BusFaultResult r;
  r.bus_id = bus_id;
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

BusFaultResult fault_at_bus(SpLU& lu,
                            int k, int bus_id,
                            int n,
                            double base_mva,
                            double base_kv,
                            const SCOptions& opt) {
  return fault_at_bus_with_zkk(
      compute_zkk(lu, k, n), bus_id, base_mva, base_kv, opt);
}

// -------------------------------------------------------------------------
// IEC 60909: Build subtransient admittance matrices
//   pos-seq (Ybus), neg-seq (Ybus2), zero-seq (Ybus0)
// -------------------------------------------------------------------------
struct YbusTriplet {
  SpMat Ybus;   // positive-sequence sparse
  SpMat Ybus2;  // negative-sequence sparse (differs from Ybus for converters)
  SpMat Ybus0;  // zero-sequence sparse
};

YbusTriplet build_sc_admittance_matrices(const ACSystem& ac,
                                         const std::vector<VSCConverter>& vsc_converters,
                                         const std::unordered_map<int, int>& id_map,
                                         int n,
                                         const SCDetailedOptions& opt,
                                         const IecNetworkCorrections& iec_corrections,
                                         bool steady_state,
                                         bool machine_shunts = true,
                                         double positive_sequence_reactance_scale = 1.0,
                                         bool use_generator_peak_resistance = false) {
  RowSparseAccumulator Ybus(n, n);
  RowSparseAccumulator Ybus2(n, n);  // negative-sequence
  RowSparseAccumulator Ybus0(n, n);
  const int estimated_row_nnz = 12;
  Ybus.reserve(Eigen::VectorXi::Constant(n, estimated_row_nnz));
  Ybus2.reserve(Eigen::VectorXi::Constant(n, estimated_row_nnz));
  Ybus0.reserve(Eigen::VectorXi::Constant(n, estimated_row_nnz));

  const double base_mva = ac.base_mva > 0.0 ? ac.base_mva : 100.0;
  const auto scale_positive_reactance =
      [positive_sequence_reactance_scale](Cx impedance) {
        return Cx(std::real(impedance),
                  std::imag(impedance) * positive_sequence_reactance_scale);
      };

  // --- branches (lines + transformers-as-branches) ---
  for (const auto& br : ac.branches) {
    if (!br.in_service) continue;
    auto it_f = id_map.find(br.from_bus);
    auto it_t = id_map.find(br.to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    int fi = it_f->second, ti = it_t->second;

    if (std::abs(br.r_pu) < 1e-12 && std::abs(br.x_pu) < 1e-12) continue;

    double temperature_factor = 1.0;
    if (opt.calc_type == SCCalcType::Min && br.sc_alpha_per_c > 0.0) {
      // IEC 60909-0:2016, eq. (14): R_Lmin = [1 + alpha *
      // (theta_e - theta_ref)] R_L, with explicit authored material data.
      temperature_factor = 1.0 + br.sc_alpha_per_c *
          (br.sc_end_temperature_c - br.sc_r_reference_temperature_c);
      if (!(temperature_factor > 0.0) || !std::isfinite(temperature_factor))
        throw std::invalid_argument("ACBranch " + std::to_string(br.index) +
                                    " has invalid IEC temperature correction");
    }
    Cx z_series(br.r_pu * temperature_factor, br.x_pu);
    if (auto it_corr = iec_corrections.transformer_branches.find(br.index);
        it_corr != iec_corrections.transformer_branches.end()) {
      z_series = it_corr->second.z1_override.value_or(
          z_series * it_corr->second.k1);
    }
    z_series = scale_positive_reactance(z_series);
    Cx y_series = Cx(1.0, 0.0) / z_series;
    Cx y_shunt(0.0, br.b_pu / 2.0);

    double tap = (br.tap > 1e-9) ? br.tap : 1.0;
    double shift_rad = br.shift_deg * M_PI / 180.0;
    Cx t(tap * std::cos(shift_rad), tap * std::sin(shift_rad));

    if (iec_corrections.replaced_positive_sequence_branches.count(br.index) == 0 &&
        std::abs(t - Cx(1.0, 0.0)) < 1e-9) {
      Ybus(fi, fi) += y_series + y_shunt;
      Ybus(ti, ti) += y_series + y_shunt;
      Ybus(fi, ti) += -y_series;
      Ybus(ti, fi) += -y_series;
      // Negative-seq = positive-seq for passive elements
      Ybus2(fi, fi) += y_series + y_shunt;
      Ybus2(ti, ti) += y_series + y_shunt;
      Ybus2(fi, ti) += -y_series;
      Ybus2(ti, fi) += -y_series;
    } else if (iec_corrections.replaced_positive_sequence_branches.count(
                   br.index) == 0) {
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
    if (iec_corrections.replaced_zero_sequence_branches.count(br.index) == 0 &&
        (std::abs(br.r0_pu) > 1e-12 || std::abs(br.x0_pu) > 1e-12)) {
      Cx z0(br.r0_pu * temperature_factor, br.x0_pu);
      if (auto it_corr = iec_corrections.transformer_branches.find(br.index);
          it_corr != iec_corrections.transformer_branches.end()) {
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
  }  // for (const auto& br : ac.branches)

  for (const auto& stamp : iec_corrections.positive_sequence_stamps) {
    Ybus(stamp.row, stamp.col) += stamp.value;
    Ybus2(stamp.row, stamp.col) += stamp.value;
  }
  for (const auto& stamp : iec_corrections.zero_sequence_stamps)
    Ybus0(stamp.row, stamp.col) += stamp.value;

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
    double bus_kv = 1.0;
    for (const auto& bus : ac.buses) {
      if (bus.index == g.bus) { bus_kv = bus.base_kv; break; }
    }
    if (use_generator_peak_resistance) {
      const double rated_kv = g.vn_kv > 1e-9 ? g.vn_kv : bus_kv;
      const double peak_r_over_x = rated_kv <= 1.0
          ? 0.15 : (mbase < 100.0 ? 0.07 : 0.05);
      // IEC 60909-0:2016, 4.3.1.2 method C: R_Gf is 0.15 Xd'' for
      // LV generators, 0.07 Xd'' below 100 MVA, and 0.05 Xd'' otherwise.
      ra = peak_r_over_x * xdpp;
    }
    const double c = get_voltage_factor_sc(bus_kv, SCCalcType::Max);
    const double voltage_scale = generator_voltage_scale(g, bus_kv);
    Cx z_gen = scale_positive_reactance(
        Cx(ra, xdpp) * (base_mva / mbase) * voltage_scale);
    const auto station_factor =
        iec_corrections.generator_impedance_factors.find(g.index);
    const double KG = station_factor !=
            iec_corrections.generator_impedance_factors.end()
        ? station_factor->second : generator_kg(g, bus_kv, c, xdpp);
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
      Cx z_gen_0 = Cx(r0, x0) * (base_mva / mbase) * voltage_scale;
      const double KG0 = station_factor !=
              iec_corrections.generator_impedance_factors.end()
          ? station_factor->second : generator_kg(g, bus_kv, c, x0);
      Cx z_gen_0_corr = z_gen_0 * KG0;
      if (std::abs(z_gen_0_corr) > 1e-15) {
        Ybus0(gi, gi) += Cx(1.0, 0.0) / z_gen_0_corr;
      }
    }
  }

  // --- asynchronous motors (subtransient only, not in steady-state) ---
  if (!steady_state && machine_shunts && opt.calc_type == SCCalcType::Max) {
    for (const auto& m : ac.motors) {
      if (!m.in_service) continue;
      const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
      if (motor_p_mw < kMinMotorContributionMw) continue;
      auto it = id_map.find(m.bus);
      if (it == id_map.end()) continue;
      int mi = it->second;

      Cx z_motor_nameplate(m.r_pu, m.x_pu);
      if (std::abs(z_motor_nameplate) < 1e-15 || m.sn_mva <= 1e-9) continue;
      const double motor_to_system_base = base_mva / m.sn_mva;
      Cx z_motor = scale_positive_reactance(
          z_motor_nameplate * motor_to_system_base);
      Cx y_motor = Cx(1.0, 0.0) / z_motor;
      Ybus(mi, mi) += y_motor;
      Ybus2(mi, mi) += y_motor;  // Z2 ≈ Z1 for asynchronous motors

      // Zero-sequence motor
      if (std::abs(m.r0_pu) > 1e-12 || std::abs(m.x0_pu) > 1e-12) {
        Cx z_m0 = Cx(m.r0_pu, m.x0_pu) * motor_to_system_base;
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
      Cx z_load = scale_positive_reactance(
          Cx(ld.r_sc_pu, ld.x_sub_pu) * (base_mva / actual_s));
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
    const Cx z_ext_positive = scale_positive_reactance(z_ext.z1);
    if (std::abs(z_ext_positive) > 1e-15) {
      Cx y_ext = Cx(1.0, 0.0) / z_ext_positive;
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
        Cx z_conv = scale_positive_reactance(
            Cx(r1, x1) * (base_mva / s_rated));

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

  // Transformer3W is intentionally absent here. RichToCanonicalOperator
  // expands each authored unit exactly once into its three Kron-eliminated pair
  // branches (IEC 60909-0:2016, 3.3.3; derivation in
  // docs/modules/short_circuit/chapters/projection_identity_contract.tex).
  // Re-adding the retained rich metadata would place a second transformer in
  // parallel with the canonical equivalent and overstate fault current.

  Ybus.makeCompressed();
  Ybus2.makeCompressed();
  Ybus0.makeCompressed();
  return {SpMat(Ybus), SpMat(Ybus2), SpMat(Ybus0)};
}

struct DetailedSparseContext {
  std::unique_ptr<SparseInverseSolver> subtransient_positive;
  std::unique_ptr<SparseInverseSolver> subtransient_negative;
  std::unique_ptr<ComponentwiseSparseInverseSolver> subtransient_zero;
  std::unique_ptr<SparseInverseSolver> network_positive;
  std::unique_ptr<SparseInverseSolver> network_zero;
  std::unique_ptr<SparseInverseSolver> method_c_positive;
  std::unique_ptr<SparseInverseSolver> steady_positive;
  std::vector<Cx> subtransient_positive_diagonal;
  std::vector<Cx> subtransient_negative_diagonal;
  std::vector<Cx> subtransient_zero_diagonal;
  std::vector<Cx> steady_positive_diagonal;
  std::unordered_map<int, double> generator_impedance_factors;
  std::unordered_map<int, TransformerBranchCorrection>
      transformer_branch_corrections;
  bool valid{true};
  std::string failure_status{"numerical_failure"};
  std::string failure_message;

  [[nodiscard]] double max_residual() const {
    double value = 0.0;
    const SparseInverseSolver* solvers[] = {
        subtransient_positive.get(), subtransient_negative.get(),
        network_positive.get(), network_zero.get(),
        method_c_positive.get(), steady_positive.get()};
    for (const auto* solver : solvers)
      if (solver) value = std::max(value, solver->max_residual());
    if (subtransient_zero)
      value = std::max(value, subtransient_zero->max_residual());
    return value;
  }
};

DetailedSparseContext build_detailed_sparse_context(
    const HybridPowerSystem& projected,
    const std::unordered_map<int, int>& id_map,
    const SCDetailedOptions& opt,
    const IecNetworkCorrections& iec_corrections,
    const IecNetworkCorrections* method_c_corrections = nullptr) {
  const auto& ac = projected.ac;
  const int n = static_cast<int>(ac.buses.size());
  DetailedSparseContext context;
  context.generator_impedance_factors =
      iec_corrections.generator_impedance_factors;
  context.transformer_branch_corrections =
      iec_corrections.transformer_branches;
  if (!iec_corrections.valid) {
    context.valid = false;
    context.failure_status = "invalid_network";
    context.failure_message = iec_corrections.message;
    return context;
  }

  auto [sub1, sub2, sub0] = build_sc_admittance_matrices(
      ac, projected.vsc_converters, id_map, n, opt,
      iec_corrections, false);
  context.subtransient_positive =
      std::make_unique<SparseInverseSolver>(sub1);
  if (!context.subtransient_positive->valid()) {
    context.valid = false;
    context.failure_message = "positive-sequence sparse factorization failed";
    return context;
  }
  context.subtransient_positive_diagonal.assign(
      static_cast<size_t>(n), Cx(0.0, 0.0));
  if (opt.compute_nonfault_currents) {
    context.subtransient_positive_diagonal =
        context.subtransient_positive->inverse_diagonal(
            opt.inverse_rhs_batch_size, opt.cancellation_requested);
  }
  context.subtransient_negative_diagonal.assign(
      static_cast<size_t>(n), Cx(0.0, 0.0));
  context.subtransient_zero_diagonal.assign(
      static_cast<size_t>(n), Cx(0.0, 0.0));
  if (opt.fault_type != FaultType::ThreePhase) {
    context.subtransient_negative =
        std::make_unique<SparseInverseSolver>(sub2);
    if (!context.subtransient_negative->valid()) {
      context.valid = false;
      context.failure_message = "negative-sequence sparse factorization failed";
      return context;
    }
    if (opt.compute_nonfault_currents) {
      context.subtransient_negative_diagonal =
          context.subtransient_negative->inverse_diagonal(
              opt.inverse_rhs_batch_size, opt.cancellation_requested);
    }
  }
  if (opt.fault_type == FaultType::SinglePhaseGround ||
      opt.fault_type == FaultType::TwoPhaseGround) {
    context.subtransient_zero =
        std::make_unique<ComponentwiseSparseInverseSolver>(sub0);
    if (!context.subtransient_zero->valid()) {
      context.valid = false;
      context.failure_message = "zero-sequence sparse factorization failed";
      return context;
    }
    if (opt.compute_nonfault_currents) {
      context.subtransient_zero_diagonal =
          context.subtransient_zero->inverse_diagonal(
              opt.inverse_rhs_batch_size, opt.cancellation_requested);
    }
  }

  auto [network1, network2, network0] = build_sc_admittance_matrices(
      ac, projected.vsc_converters, id_map, n, opt,
      iec_corrections, false, false);
  (void)network2;
  context.network_positive =
      std::make_unique<SparseInverseSolver>(network1);
  if (opt.fault_type == FaultType::SinglePhaseGround ||
      opt.fault_type == FaultType::TwoPhaseGround) {
    context.network_zero =
        std::make_unique<SparseInverseSolver>(network0);
  }

  if (opt.kappa_method == SCKappaMethod::C) {
    if (!method_c_corrections || !method_c_corrections->valid) {
      context.valid = false;
      context.failure_status = "invalid_network";
      context.failure_message = method_c_corrections
          ? method_c_corrections->message
          : "IEC 60909 method C corrections are unavailable";
      return context;
    }
    auto [method_c_1, method_c_2, method_c_0] =
        build_sc_admittance_matrices(
            ac, projected.vsc_converters, id_map, n, opt,
            *method_c_corrections, false, true,
            kMethodCFrequencyRatio, true);
    (void)method_c_2;
    (void)method_c_0;
    context.method_c_positive =
        std::make_unique<SparseInverseSolver>(method_c_1);
    if (!context.method_c_positive->valid()) {
      context.valid = false;
      context.failure_message =
          "IEC 60909 method C positive-sequence factorization failed";
      return context;
    }
  }

  auto [steady1, steady2, steady0] = build_sc_admittance_matrices(
      ac, projected.vsc_converters, id_map, n, opt,
      iec_corrections, true);
  (void)steady2;
  (void)steady0;
  context.steady_positive =
      std::make_unique<SparseInverseSolver>(steady1);
  context.steady_positive_diagonal.assign(
      static_cast<size_t>(n), Cx(0.0, 0.0));
  if (opt.compute_nonfault_currents && context.steady_positive->valid()) {
    context.steady_positive_diagonal =
        context.steady_positive->inverse_diagonal(
            opt.inverse_rhs_batch_size, opt.cancellation_requested);
  }
  return context;
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
      // IEC 60909-0:2016, 4.3.3: a finite line-to-line fault impedance is
      // in series with the opposed positive/negative-sequence networks.
      return (Z1 + Z2 + Zf) / std::sqrt(3.0);
    case FaultType::TwoPhaseGround: {
      // IEC 60909-0:2016, 4.3.4: Zf is threefold in the zero-sequence
      // branch for a double-line-to-ground fault.
      const Cx z0f = Z0 + Cx(3.0, 0.0) * Zf;
      Cx Z2_par_Z0f = (std::abs(Z2 + z0f) > 1e-15)
          ? (Z2 * z0f) / (Z2 + z0f) : Cx(0.0, 0.0);
      return Z1 + Z2_par_Z0f;
    }
    default:
      return Z1 + Zf;
  }
}

struct FaultSequenceCurrents {
  Cx i0{0.0, 0.0};
  Cx i1{0.0, 0.0};
  Cx i2{0.0, 0.0};
  Cx ia{0.0, 0.0};
  Cx ib{0.0, 0.0};
  Cx ic{0.0, 0.0};
};

FaultSequenceCurrents fault_sequence_currents(
    FaultType type, Cx z1, Cx z2, Cx z0, Cx zf, double c) {
  FaultSequenceCurrents out;
  const Cx a(-0.5, std::sqrt(3.0) / 2.0);
  const Cx a2 = a * a;
  switch (type) {
    case FaultType::ThreePhase:
      if (std::abs(z1 + zf) > 1e-15) out.i1 = c / (z1 + zf);
      break;
    case FaultType::SinglePhaseGround: {
      const Cx denom = z1 + z2 + z0 + Cx(3.0, 0.0) * zf;
      if (std::abs(denom) > 1e-15) out.i0 = out.i1 = out.i2 = c / denom;
      break;
    }
    case FaultType::TwoPhase: {
      const Cx denom = z1 + z2 + zf;
      if (std::abs(denom) > 1e-15) {
        out.i1 = c / denom;
        out.i2 = -out.i1;
      }
      break;
    }
    case FaultType::TwoPhaseGround: {
      // Fortescue sequence interconnection for a b-c-ground fault;
      // IEC 60909-0:2016, 4.3.4 and project derivation sec. 3.4.
      const Cx z0f = z0 + Cx(3.0, 0.0) * zf;
      const Cx parallel_denom = z2 + z0f;
      if (std::abs(parallel_denom) > 1e-15) {
        const Cx denom = z1 + z2 * z0f / parallel_denom;
        if (std::abs(denom) > 1e-15) {
          out.i1 = c / denom;
          out.i2 = -out.i1 * z0f / parallel_denom;
          out.i0 = -out.i1 * z2 / parallel_denom;
        }
      }
      break;
    }
  }
  out.ia = out.i0 + out.i1 + out.i2;
  out.ib = out.i0 + a2 * out.i1 + a * out.i2;
  out.ic = out.i0 + a * out.i1 + a2 * out.i2;
  return out;
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
                                const Eigen::VectorXcd& zbus_column,
                                const Eigen::VectorXcd& zbus2_column,
                                const Eigen::VectorXcd& zbus0_column,
                                const std::vector<Cx>& zbus_diagonal,
                                const std::vector<Cx>& zbus2_diagonal,
                                const std::vector<Cx>& zbus0_diagonal,
                                int bus_idx) {
  const auto diag_at = [bus_idx](const std::vector<Cx>& diagonal) -> Cx {
    return bus_idx >= 0 && bus_idx < static_cast<int>(diagonal.size())
               ? diagonal[static_cast<size_t>(bus_idx)]
               : Cx(0.0, 0.0);
  };
  TransferRatios tr;
  switch (ft) {
    case FaultType::ThreePhase:
      tr.Zk_xfer = zbus_column(bus_idx);
      tr.Zk_self = diag_at(zbus_diagonal);
      break;
    case FaultType::SinglePhaseGround:
      tr.Zk_xfer = (zbus_column(bus_idx) + zbus2_column(bus_idx) +
                    zbus0_column(bus_idx)) / 3.0;
      tr.Zk_self = (diag_at(zbus_diagonal) + diag_at(zbus2_diagonal) +
                    diag_at(zbus0_diagonal)) / 3.0;
      break;
    case FaultType::TwoPhase:
      tr.Zk_xfer = (zbus_column(bus_idx) + zbus2_column(bus_idx)) / std::sqrt(3.0);
      tr.Zk_self = (diag_at(zbus_diagonal) + diag_at(zbus2_diagonal)) / std::sqrt(3.0);
      break;
    case FaultType::TwoPhaseGround: {
      auto z_eff = [](Cx z1, Cx z2, Cx z0) -> Cx {
        Cx z2_par_z0 = (std::abs(z2 + z0) > 1e-15)
            ? (z2 * z0) / (z2 + z0) : Cx(0.0, 0.0);
        return z1 + z2_par_z0;
      };
      tr.Zk_xfer = z_eff(zbus_column(bus_idx), zbus2_column(bus_idx),
                         zbus0_column(bus_idx));
      tr.Zk_self = z_eff(diag_at(zbus_diagonal), diag_at(zbus2_diagonal),
                         diag_at(zbus0_diagonal));
      break;
    }
    default:
      tr.Zk_xfer = zbus_column(bus_idx);
      tr.Zk_self = diag_at(zbus_diagonal);
      break;
  }
  return tr;
}

// -------------------------------------------------------------------------
// IEC 60909 mu factor (for breaking current decay)
// -------------------------------------------------------------------------
double compute_mu(double Ik_ratio, double t_break) {
  // IEC 60909-0:2016, eq. (67): no AC-component decay is applied when the
  // partial initial current is not greater than twice rated current.
  if (Ik_ratio <= 2.0) return 1.0;
  const auto curve = [Ik_ratio](double time_s) {
    if (time_s <= 0.02)
      return 0.84 + 0.26 * std::exp(-0.26 * Ik_ratio);
    if (time_s <= 0.05)
      return 0.71 + 0.51 * std::exp(-0.30 * Ik_ratio);
    if (time_s <= 0.10)
      return 0.62 + 0.72 * std::exp(-0.32 * Ik_ratio);
    return 0.56 + 0.94 * std::exp(-0.38 * Ik_ratio);
  };
  double mu = 0.0;
  if (t_break <= 0.02 || t_break >= 0.25) {
    mu = curve(t_break);
  } else {
    const std::array<double, 4> times{0.02, 0.05, 0.10, 0.25};
    const auto upper = std::upper_bound(times.begin(), times.end(), t_break);
    const double hi = *upper;
    const double lo = *(upper - 1);
    const double weight = (t_break - lo) / (hi - lo);
    mu = curve(lo) + weight * (curve(hi) - curve(lo));
  }
  // IEC 60909-0:2016, Figure 13: mu is a decay factor and cannot exceed 1.
  return std::clamp(mu, 0.0, 1.0);
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
  validate_options(opt);
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
  SparseInverseSolver inverse(Y_fault);
  if (!inverse.valid())
    throw std::runtime_error("compute_short_circuit: Y_fault factorisation failed");
  const auto zbus_diagonal = inverse.inverse_diagonal(
      opt.inverse_rhs_batch_size, opt.cancellation_requested);

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
    canonical_bus_results.push_back(fault_at_bus_with_zkk(
        zbus_diagonal[static_cast<size_t>(k)], bus_id,
        projected.base_mva, base_kv, opt));
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
  validate_options(opt);
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
  bool factored = false;
  if (Y_fault.nonZeros() > 0) {
    lu.analyzePattern(Y_fault);
    lu.factorize(Y_fault);
    factored = lu.info() == Eigen::Success;
  }
  if (!factored)
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

SCThermalFactors calculate_thermal_factors_sc(double kappa,
                                              double ikss_ka,
                                              double ik_ka,
                                              double frequency_hz,
                                              double duration_s) {
  if (!std::isfinite(kappa) || kappa < 1.0 || kappa > 2.0)
    throw std::invalid_argument("IEC thermal kappa must be finite and in [1, 2]");
  if (!std::isfinite(ikss_ka) || ikss_ka < 0.0 ||
      !std::isfinite(ik_ka) || ik_ka < 0.0)
    throw std::invalid_argument("IEC thermal currents must be finite and non-negative");
  if (!std::isfinite(frequency_hz) || frequency_hz <= 0.0 ||
      !std::isfinite(duration_s) || duration_s <= 0.0)
    throw std::invalid_argument("IEC thermal frequency and duration must be finite and positive");

  SCThermalFactors factors;
  // IEC 60909-0:2016, Annex A. expm1 preserves the kappa -> 2 limit;
  // the kappa -> 1 limit is zero because ln(kappa - 1) -> -infinity.
  if (kappa <= 1.0 + 1e-15) {
    factors.m = 0.0;
  } else if (kappa >= 2.0 - 1e-15) {
    factors.m = 2.0;
  } else {
    const double log_term = std::log(kappa - 1.0);
    factors.m = std::expm1(4.0 * frequency_hz * duration_s * log_term) /
                (2.0 * frequency_hz * duration_s * log_term);
  }

  if (ikss_ka <= 1e-15) {
    factors.n = 1.0;
    return factors;
  }

  const auto annex_a_n = [duration_s](double ratio) {
    const double ratio_prime = ratio / (0.88 + 0.17 * ratio);
    const double td_prime = 3.1 / ratio_prime;
    const double a = ratio - ratio_prime;
    const double b = ratio_prime - 1.0;
    const auto one_minus_exp = [duration_s, td_prime](double multiple) {
      return -std::expm1(-multiple * duration_s / td_prime);
    };
    // IEC 60909-0:2016, Annex A. The 5*T term contains a (not a^2), the
    // 2*T term contains b^2, and the 5.5*T term contains a*b exactly as
    // printed in the normative formula.
    const double bracket =
        1.0 +
        td_prime / (20.0 * duration_s) * one_minus_exp(20.0) * a * a +
        td_prime / (2.0 * duration_s) * one_minus_exp(2.0) * b * b +
        td_prime / (5.0 * duration_s) * one_minus_exp(10.0) * a +
        2.0 * td_prime / duration_s * one_minus_exp(1.0) * b * b +
        td_prime / (5.5 * duration_s) * one_minus_exp(11.0) * a * b;
    return bracket / (ratio * ratio);
  };

  if (ik_ka <= 1e-15) {
    // q -> infinity: only the a^2 term survives after division by q^2.
    constexpr double ratio_prime_limit = 1.0 / 0.17;
    const double td_prime = 3.1 / ratio_prime_limit;
    factors.n = td_prime / (20.0 * duration_s) *
                (-std::expm1(-20.0 * duration_s / td_prime));
    return factors;
  }

  const double ratio = ikss_ka / ik_ka;
  if (ratio <= 1.0) {
    // Figure 19 starts at q=1. A non-decaying or increasing AC component has
    // n=1, which is the conservative IEC endpoint for this input regime.
    factors.n = 1.0;
  } else if (ratio < 1.25) {
    // IEC Figure 19 interpolation between its q=1 endpoint and the Annex A
    // analytic family, whose stated domain begins at q=1.25.
    const double n_at_125 = annex_a_n(1.25);
    factors.n = 1.0 + (ratio - 1.0) / 0.25 * (n_at_125 - 1.0);
  } else {
    factors.n = annex_a_n(ratio);
  }
  factors.m = std::clamp(factors.m, 0.0, 2.0);
  factors.n = std::max(0.0, factors.n);
  return factors;
}

SCDetailedResult run_short_circuit_detailed_impl(
    const HybridPowerSystem& sys,
    int fault_bus_id,
    const SCDetailedOptions& opt,
    const projection::ProjectionBundle& projection_bundle,
    DetailedSparseContext& sparse_context) {
  SCDetailedResult out;
  out.fault_bus_id = fault_bus_id;
  out.solved = false;
  out.status = "not_run";
  out.model_limitations = {
      "IEC 60909 quasi-static fault levels do not include EMT capacitor discharge, line inductance transients, converter-control switching, or breaker opening time histories."};

  const auto& projected = projection_bundle.canonical;
  const auto& ac = projected.ac;
  const int n = static_cast<int>(ac.buses.size());
  if (n == 0) {
    out.status = "invalid_network";
    out.message = "no energized AC buses after canonical projection";
    return out;
  }
  if (!sparse_context.valid) {
    out.status = sparse_context.failure_status;
    out.message = sparse_context.failure_message;
    out.numerical_quality.factorization_valid = false;
    return out;
  }
  out.numerical_quality.factorization_valid = true;

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
  double c = detailed_voltage_factor(fault_kv, opt);
  for (const auto& generator : ac.generators) {
    if (generator.in_service && generator.bus == resolved_fault_bus_id &&
        generator.power_station_transformer_index > 0 &&
        generator.vn_kv > 1e-9 && fault_kv > 1e-9) {
      // IEC 60909-0:2016, 6.3.1: inside a power-station unit, the
      // equivalent source is referred to the generator rated voltage.
      c *= generator.vn_kv / fault_kv;
      break;
    }
  }
  out.effective_c_factor = c;
  const double I_base = base_mva / (std::sqrt(3.0) * fault_kv);  // kA
  const Cx Zf(opt.fault_impedance_pu, 0.0);

  if ((opt.fault_type == FaultType::SinglePhaseGround ||
       opt.fault_type == FaultType::TwoPhaseGround) &&
      !sparse_context.subtransient_zero->has_finite_driving_point(fault_idx)) {
    // IEC 60909-0:2016, 4.3.2/4.3.4: without a zero-sequence return path the
    // ideal quasi-static network carries no earth-fault current. Capacitance-
    // supplied earth current and neutral displacement require a phase-domain
    // or shunt-capacitance model and are outside this sequence-network result.
    out.model_limitations.push_back(
        "The fault bus is in an ungrounded zero-sequence component: earth-fault current is zero in the IEC quasi-static impedance model; capacitive earth current and neutral-voltage displacement are not represented.");
    out.bus_results.reserve(sys.ac.buses.size());
    for (const auto& bus : sys.ac.buses) {
      SCDetailedBusResult row;
      row.bus_id = bus.index;
      row.v_remaining_pu = bus.index == fault_bus_id ? 0.0 : 1.0;
      out.bus_results.push_back(std::move(row));
    }
    out.numerical_quality.max_linear_residual = sparse_context.max_residual();
    out.numerical_quality.all_finite = true;
    out.solved = true;
    out.status = "solved_zero_sequence_open";
    out.message = "ok: no zero-sequence return path";
    return out;
  }

  auto bus_kv = [&](int bus_id) -> double {
    for (const auto& bus : ac.buses) {
      if (bus.index == bus_id) return (bus.base_kv > 1e-6) ? bus.base_kv : fault_kv;
    }
    return fault_kv;
  };

  const auto generator_factor = [&](const Generator& generator,
                                    double reactance) {
    const auto station = sparse_context.generator_impedance_factors.find(
        generator.index);
    if (station != sparse_context.generator_impedance_factors.end())
      return station->second;
    const double kv = bus_kv(generator.bus);
    return generator_kg(generator, kv,
                        get_voltage_factor_sc(kv, SCCalcType::Max),
                        reactance);
  };

  const auto generator_impedance = [&](const Generator& generator,
                                       double reactance,
                                       double resistance) {
    const double mbase = generator.mbase_mva > 1e-6
        ? generator.mbase_mva : base_mva;
    return Cx(resistance, reactance) * (base_mva / mbase) *
           generator_voltage_scale(generator, bus_kv(generator.bus)) *
           generator_factor(generator, reactance);
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

  // ====== Step 1: Selected columns from shared sparse sequence factors ======
  // IEC 60909 only requires Z[:,k] and selected diagonal entries for a fault
  // at k.  The batch API constructs sparse_context once and reuses its
  // symbolic/numeric factorizations for every requested fault location.
  const auto& Zbus_diag = sparse_context.subtransient_positive_diagonal;
  const auto& Zbus2_diag = sparse_context.subtransient_negative_diagonal;
  const auto& Zbus0_diag = sparse_context.subtransient_zero_diagonal;
  const Eigen::VectorXcd Zbus_col =
      sparse_context.subtransient_positive->inverse_column(fault_idx);
  Eigen::VectorXcd Zbus2_col = Eigen::VectorXcd::Zero(n);
  Eigen::VectorXcd Zbus0_col = Eigen::VectorXcd::Zero(n);
  if (sparse_context.subtransient_negative) {
    Zbus2_col = sparse_context.subtransient_negative->inverse_column(fault_idx);
  }
  if (sparse_context.subtransient_zero) {
    Zbus0_col = sparse_context.subtransient_zero->inverse_column(fault_idx);
  }

  // ====== Step 2: Compute initial SC current (Ikss) at fault bus ======
  Cx Z1_fault = Zbus_col(fault_idx);
  Cx Z2_fault = Zbus2_col(fault_idx);
  Cx Z0_fault = Zbus0_col(fault_idx);
  Cx Zk = compute_Zk(opt.fault_type, Z1_fault, Z2_fault, Z0_fault, Zf);
  const auto fault_currents = fault_sequence_currents(
      opt.fault_type, Z1_fault, Z2_fault, Z0_fault, Zf, c);
  double I_kss_pu = std::max({std::abs(fault_currents.ia),
                              std::abs(fault_currents.ib),
                              std::abs(fault_currents.ic)});
  double I_kss_kA = I_kss_pu * I_base;

  // ====== Step 3: Compute per-source contributions at fault bus ======
  double gen_contrib = 0.0, motor_contrib = 0.0, load_contrib = 0.0;
  struct VoltageSourcePath {
    int bus_index{-1};
    double current_ka{0.0};
    Cx impedance;
  };
  std::vector<VoltageSourcePath> voltage_source_paths;
  struct GeneratorFaultContribution {
    const Generator* generator{nullptr};
    double initial_ka{0.0};
    double rated_current_at_fault_kv_ka{0.0};
  };
  std::vector<GeneratorFaultContribution> generator_fault_contributions;
  int external_grid_source_count = 0;
  int grid_forming_converter_source_count = 0;

  auto source_transfer_abs = [&](int source_bus_id) -> double {
    auto it = id_map.find(source_bus_id);
    if (it == id_map.end() || std::abs(Zk) <= 1e-15) return 0.0;
    const auto tr = compute_transfer(
        opt.fault_type, Zbus_col, Zbus2_col, Zbus0_col,
        Zbus_diag, Zbus2_diag, Zbus0_diag, it->second);
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
    Cx z_gen_corr = generator_impedance(g, xdpp, g.ra_pu);

    double gen_ci = 0.0;
    Cx gen_z = z_gen_corr;
    if (opt.fault_type == FaultType::ThreePhase) {
      gen_ci = voltage_source_contribution_ka(g.bus, z_gen_corr);
    } else if (opt.fault_type == FaultType::SinglePhaseGround) {
      double x0 = g.x0_pu, r0 = g.r0_pu;
      Cx z_gen_0_corr = generator_impedance(g, x0, r0);
      gen_z = (z_gen_corr * 2.0 + z_gen_0_corr) / 3.0;
      gen_ci = voltage_source_contribution_ka(g.bus, gen_z);
    } else if (opt.fault_type == FaultType::TwoPhase) {
      gen_z = z_gen_corr * 2.0 / std::sqrt(3.0);
      gen_ci = voltage_source_contribution_ka(g.bus, gen_z);
    } else if (opt.fault_type == FaultType::TwoPhaseGround) {
      double x0 = g.x0_pu, r0 = g.r0_pu;
      Cx z_gen_0_corr = generator_impedance(g, x0, r0);
      gen_z = (2.0 * z_gen_corr * z_gen_0_corr + z_gen_corr * z_gen_corr) / (3.0 * z_gen_corr);
      gen_ci = voltage_source_contribution_ka(g.bus, gen_z);
    }
    gen_contrib += gen_ci;
    voltage_source_paths.push_back({id_map.at(g.bus), gen_ci, gen_z});
    if (gen_ci > 1e-12) {
      const double mbase = g.mbase_mva > 1e-6 ? g.mbase_mva : base_mva;
      generator_fault_contributions.push_back(
          {&g, gen_ci, mbase / (std::sqrt(3.0) * fault_kv)});
    }
  }

  // Motor contributions at fault bus
  if (opt.calc_type == SCCalcType::Max) for (const auto& m : ac.motors) {
    if (!m.in_service) continue;
    const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
    if (motor_p_mw < kMinMotorContributionMw) continue;
    Cx z_motor_nameplate(m.r_pu, m.x_pu);
    if (std::abs(z_motor_nameplate) < 1e-15 || m.sn_mva <= 1e-9) continue;
    const double motor_to_system_base = base_mva / m.sn_mva;
    Cx z_motor = z_motor_nameplate * motor_to_system_base;

    double motor_ci = 0.0;
    Cx motor_z = z_motor;
    if (opt.fault_type == FaultType::ThreePhase) {
      motor_ci = voltage_source_contribution_ka(m.bus, z_motor);
    } else if (opt.fault_type == FaultType::SinglePhaseGround) {
      Cx z_m0 = Cx(m.r0_pu, m.x0_pu) * motor_to_system_base;
      motor_z = (2.0 * z_motor + z_m0) / 3.0;
      motor_ci = voltage_source_contribution_ka(m.bus, motor_z);
    } else if (opt.fault_type == FaultType::TwoPhase) {
      motor_z = z_motor * 2.0 / std::sqrt(3.0);
      motor_ci = voltage_source_contribution_ka(m.bus, motor_z);
    } else if (opt.fault_type == FaultType::TwoPhaseGround) {
      Cx z_m0 = Cx(m.r0_pu, m.x0_pu) * motor_to_system_base;
      motor_z = (2.0 * z_motor * z_m0 + z_motor * z_motor) / (3.0 * z_motor);
      motor_ci = voltage_source_contribution_ka(m.bus, motor_z);
    }
    motor_contrib += motor_ci;
    voltage_source_paths.push_back({id_map.at(m.bus), motor_ci, motor_z});
  }

  // Load motor-fraction contributions at fault bus
  if (opt.calc_type == SCCalcType::Max) for (const auto& ld : ac.loads) {
    if (!ld.in_service) continue;
    const double motor_fraction = load_motor_fraction(ld.motor_percent);
    if (motor_fraction <= 1e-6 || ld.sn_mva <= 1e-6) continue;
    if (load_motor_active_power_mw(ld, motor_fraction) < kMinMotorContributionMw) continue;
    double actual_s = ld.sn_mva * motor_fraction;
    if (actual_s < 1e-12) continue;
    Cx z_load = Cx(ld.r_sc_pu, ld.x_sub_pu) * (base_mva / actual_s);
    const double contrib = voltage_source_contribution_ka(ld.bus, z_load);
    voltage_source_paths.push_back({id_map.at(ld.bus), contrib, z_load});
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
    const double contribution = voltage_source_contribution_ka(eg.bus, source_zk);
    extgrid_contrib += contribution;
    voltage_source_paths.push_back({id_map.at(eg.bus), contribution, source_zk});
    if (contribution > 1e-12) ++external_grid_source_count;
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
      voltage_source_paths.push_back({id_map.at(conv.bus_ac), contrib, z_conv});
      if (contrib > 1e-12) ++grid_forming_converter_source_count;
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
  const Cx Z_voltage_denom = Z1_fault + Zf;
  for (int k = 0; k < n; ++k) {
    if (k == fault_idx) {
      V_fault[k] = Cx(0.0, 0.0);
    } else if (std::abs(Z_voltage_denom) > 1e-15) {
      V_fault[k] = Cx(c, 0.0) *
                   (Cx(1.0, 0.0) - Zbus_col(k) / Z_voltage_denom);
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
      row.ikss_1_ka = std::abs(fault_currents.i1) * I_base;
      row.ikss_2_ka = std::abs(fault_currents.i2) * I_base;
      row.i_phase_a_ka = std::abs(fault_currents.ia) * I_base;
      row.i_phase_b_ka = std::abs(fault_currents.ib) * I_base;
      row.i_phase_c_ka = std::abs(fault_currents.ic) * I_base;
      row.i_ground_ka = 3.0 * std::abs(fault_currents.i0) * I_base;
      row.ikss_gen_contrib_ka = gen_contrib;
      row.ikss_motor_contrib_ka = motor_contrib;
      row.ikss_load_contrib_ka = load_contrib;
      row.ikss_sgen_contrib_ka = sgen_contrib;
      row.ikss_extgrid_contrib_ka = extgrid_contrib;
      row.ikss_converter_contrib_ka = converter_contrib;
      row.ikss_no_motor_ka = no_motor;
      row.v_remaining_pu = 0.0;  // Voltage at fault point is zero
    } else {
      if (!opt.compute_nonfault_currents) {
        if (opt.compute_voltage_drops) {
          row.v_remaining_pu = std::abs(V_fault[k]);
        }
        continue;
      }
      // Transfer impedance based contribution
      auto tr = compute_transfer(
          opt.fault_type, Zbus_col, Zbus2_col, Zbus0_col,
          Zbus_diag, Zbus2_diag, Zbus0_diag, k);
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
        double Z2_self = std::abs(Zbus2_diag[static_cast<size_t>(k)]);
        double Z2_xfer = std::abs(Zbus2_col(k));
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
        Cx z_gen_corr = generator_impedance(g, xdpp, g.ra_pu);

        if (opt.fault_type == FaultType::ThreePhase) {
          if (std::abs(z_gen_corr) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += c * std::abs(Zbus_col(k)) / (std::abs(z_gen_corr) * std::abs(Zk)) * I_base;
        } else if (opt.fault_type == FaultType::SinglePhaseGround) {
          Cx z_gen_0_corr = generator_impedance(g, g.x0_pu, g.r0_pu);
          Cx z_k = (2.0 * z_gen_corr + z_gen_0_corr) / 3.0;
          if (std::abs(z_k) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += (c / std::abs(z_k)) * std::abs(tr.Zk_xfer) / std::abs(Zk) * I_base;
        } else if (opt.fault_type == FaultType::TwoPhase) {
          Cx z_k = z_gen_corr * 2.0 / std::sqrt(3.0);
          if (std::abs(z_k) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += (c / std::abs(z_k)) * std::abs(Zbus_col(k) * 2.0 / std::sqrt(3.0)) / std::abs(Zk) * I_base;
        } else if (opt.fault_type == FaultType::TwoPhaseGround) {
          Cx z_gen_0_corr = generator_impedance(g, g.x0_pu, g.r0_pu);
          Cx z_k = (2.0 * z_gen_corr * z_gen_0_corr + z_gen_corr * z_gen_corr) / (3.0 * z_gen_corr);
          if (std::abs(z_k) > 1e-15 && std::abs(Zk) > 1e-15)
            bus_gen += (c / std::abs(z_k)) * std::abs(tr.Zk_xfer) / std::abs(Zk) * I_base;
        }
      }

      if (opt.calc_type == SCCalcType::Max) for (const auto& m : ac.motors) {
        if (!m.in_service || m.bus != bus_id) continue;
        const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
        if (motor_p_mw < kMinMotorContributionMw) continue;
        Cx z_motor_nameplate(m.r_pu, m.x_pu);
        if (std::abs(z_motor_nameplate) < 1e-15 || m.sn_mva <= 1e-9) continue;
        Cx z_motor = z_motor_nameplate * (base_mva / m.sn_mva);

        if (opt.fault_type == FaultType::ThreePhase) {
          double m_pu = (c / std::abs(z_motor)) * std::abs(Zbus_col(k)) /
                        (std::abs(Zk) * std::abs(Zbus_diag[static_cast<size_t>(k)]));
          bus_motor += m_pu * I_base;
        } else {
          // For unbalanced faults, use transfer ratio
          double m_pu = (c / std::abs(z_motor)) * std::abs(tr.Zk_xfer) / std::abs(Zk);
          bus_motor += m_pu * I_base;
        }
      }

      if (opt.calc_type == SCCalcType::Max) for (const auto& ld : ac.loads) {
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
  // IEC 60909-0:2016, 4.3.1.2 and eq. (59): methods A/B/C determine one
  // fault-point equivalent kappa for the total equivalent-voltage-source
  // current. Static generators and grid-following converters are current
  // sources without a decaying dc component and therefore use kappa = 1.
  // Method B applies its topology correction and voltage-level cap; methods A
  // and C use their own network constructions below. Non-fault rows retain the
  // same single-kappa transferred-current indicator.
  double method_b_multiplier = 1.0;
  if (opt.kappa_method == SCKappaMethod::B) {
    if (opt.topology == SCTopology::Meshed) {
      method_b_multiplier = 1.15;
    } else if (opt.topology == SCTopology::Auto) {
      struct PathEdge { int to; double r; double x; };
      std::vector<std::vector<PathEdge>> adjacency(static_cast<size_t>(n));
      for (const auto& branch : ac.branches) {
        if (!branch.in_service) continue;
        const auto from = id_map.find(branch.from_bus);
        const auto to = id_map.find(branch.to_bus);
        if (from == id_map.end() || to == id_map.end()) continue;
        double temperature_factor = 1.0;
        if (opt.calc_type == SCCalcType::Min && branch.sc_alpha_per_c > 0.0) {
          temperature_factor = 1.0 + branch.sc_alpha_per_c *
              (branch.sc_end_temperature_c - branch.sc_r_reference_temperature_c);
        }
        Cx impedance(branch.r_pu * temperature_factor, branch.x_pu);
        if (const auto correction =
                sparse_context.transformer_branch_corrections.find(branch.index);
            correction != sparse_context.transformer_branch_corrections.end()) {
          impedance = correction->second.z1_override.value_or(
              impedance * correction->second.k1);
        }
        const PathEdge forward{to->second, std::abs(std::real(impedance)),
                               std::abs(std::imag(impedance))};
        const PathEdge reverse{from->second, forward.r, forward.x};
        adjacency[static_cast<size_t>(from->second)].push_back(forward);
        adjacency[static_cast<size_t>(to->second)].push_back(reverse);
      }

      std::vector<std::vector<const VoltageSourcePath*>> sources_at_bus(
          static_cast<size_t>(n));
      for (const auto& source : voltage_source_paths) {
        if (source.current_ka > 1e-12 && source.bus_index >= 0 &&
            source.bus_index < n) {
          sources_at_bus[static_cast<size_t>(source.bus_index)].push_back(&source);
        }
      }
      size_t path_count = 0;
      bool low_rx_path = false;
      std::vector<bool> visited(static_cast<size_t>(n), false);
      const auto enumerate = [&](const auto& self, int node,
                                 double path_r, double path_x) -> void {
        if (path_count >= 2 && low_rx_path) return;
        visited[static_cast<size_t>(node)] = true;
        for (const auto* source : sources_at_bus[static_cast<size_t>(node)]) {
          ++path_count;
          const double r = path_r + std::abs(std::real(source->impedance));
          const double x = path_x + std::abs(std::imag(source->impedance));
          if (x > 1e-15 && r / x < 0.3) low_rx_path = true;
        }
        for (const auto& edge : adjacency[static_cast<size_t>(node)]) {
          if (!visited[static_cast<size_t>(edge.to)])
            self(self, edge.to, path_r + edge.r, path_x + edge.x);
          if (path_count >= 2 && low_rx_path) break;
        }
        visited[static_cast<size_t>(node)] = false;
      };
      enumerate(enumerate, fault_idx, 0.0, 0.0);
      if (path_count > 1 && !low_rx_path) method_b_multiplier = 1.15;
    }
  }
  Cx Z_k_kappa = compute_Zk(opt.fault_type,
                            Z1_fault, Z2_fault, Z0_fault, Zf);
  double rx_ratio = (std::abs(std::imag(Z_k_kappa)) > 1e-15)
                        ? std::abs(std::real(Z_k_kappa) / std::imag(Z_k_kappa))
                        : 0.0;
  if (opt.kappa_method == SCKappaMethod::A) {
    double minimum_participating_rx = std::numeric_limits<double>::infinity();
    const auto consider_impedance = [&](Cx impedance, double current) {
      if (current <= 1e-12 || std::abs(impedance) <= 1e-15) return;
      if (std::abs(std::imag(impedance)) <= 1e-15) return;
      minimum_participating_rx = std::min(
          minimum_participating_rx,
          std::abs(std::real(impedance) / std::imag(impedance)));
    };
    for (const auto& source : voltage_source_paths)
      consider_impedance(source.impedance, source.current_ka);

    std::unordered_set<int> transformer_branch_indices;
    if (projected.branch_expand_map) {
      for (const auto& entry : projected.branch_expand_map->entries) {
        if (entry.origin_type == BranchOriginType::Transformer2W ||
            entry.origin_type == BranchOriginType::Transformer3W)
          transformer_branch_indices.insert(entry.branch_index);
      }
    }

    const auto same_voltage_level = [fault_kv](double value) {
      return std::abs(value - fault_kv) <=
             1e-6 * std::max({1.0, std::abs(value), std::abs(fault_kv)});
    };
    for (const auto& branch : ac.branches) {
      if (!branch.in_service) continue;
      const auto from = id_map.find(branch.from_bus);
      const auto to = id_map.find(branch.to_bus);
      if (from == id_map.end() || to == id_map.end()) continue;
      const bool transformer =
          transformer_branch_indices.count(branch.index) > 0 ||
          !same_voltage_level(ac.buses[static_cast<size_t>(from->second)].base_kv) ||
          !same_voltage_level(ac.buses[static_cast<size_t>(to->second)].base_kv);
      if (!transformer &&
          (!same_voltage_level(ac.buses[static_cast<size_t>(from->second)].base_kv) ||
           !same_voltage_level(ac.buses[static_cast<size_t>(to->second)].base_kv)))
        continue;

      double temperature_factor = 1.0;
      if (opt.calc_type == SCCalcType::Min && branch.sc_alpha_per_c > 0.0) {
        temperature_factor = 1.0 + branch.sc_alpha_per_c *
            (branch.sc_end_temperature_c - branch.sc_r_reference_temperature_c);
      }
      Cx impedance(branch.r_pu * temperature_factor, branch.x_pu);
      if (const auto correction =
              sparse_context.transformer_branch_corrections.find(branch.index);
          correction != sparse_context.transformer_branch_corrections.end()) {
        impedance = correction->second.z1_override.value_or(
            impedance * correction->second.k1);
      }
      if (std::abs(impedance) <= 1e-15) continue;
      const double tap = branch.tap > 1e-9 ? branch.tap : 1.0;
      const double shift = branch.shift_deg * M_PI / 180.0;
      const Cx complex_tap(tap * std::cos(shift), tap * std::sin(shift));
      const double partial_current = std::abs(
          (V_fault(from->second) / complex_tap - V_fault(to->second)) /
          impedance);
      if (partial_current > std::max(1e-10, I_kss_pu * 1e-9))
        consider_impedance(impedance, partial_current);
    }
    if (!std::isfinite(minimum_participating_rx)) {
      out.status = "invalid_network";
      out.message =
          "IEC 60909 method A found no participating branch with finite R/X";
      return out;
    }
    // IEC 60909-0:2016, 4.3.1.2 method A: use one uniform kappa based on
    // the smallest R/X of every branch carrying partial fault current at the
    // fault voltage level and the transformer branches feeding that level.
    rx_ratio = minimum_participating_rx;
  }
  double kappa = calculate_kappa_basic_sc(rx_ratio);
  if (opt.kappa_method == SCKappaMethod::B) {
    const double kappa_max = fault_kv < 1.0 ? 1.8 : 2.0;
    kappa = std::clamp(method_b_multiplier * kappa, 1.0, kappa_max);
  } else {
    kappa = std::clamp(kappa, 1.0, 2.0);
  }

  double method_c_kappa = kappa;
  if (opt.kappa_method == SCKappaMethod::C) {
    const Eigen::VectorXcd method_c_column =
        sparse_context.method_c_positive->inverse_column(fault_idx);
    const Cx method_c_zkk = method_c_column(fault_idx);
    const double method_c_rx = std::abs(std::imag(method_c_zkk)) > 1e-15
        ? std::abs(std::real(method_c_zkk) / std::imag(method_c_zkk)) *
              kMethodCFrequencyRatio
        : 0.0;
    // IEC 60909-0:2016, 4.3.1.2 method C: determine R/X from the
    // fc-frequency equivalent and refer it back by fc/f before applying
    // kappa = 1.02 + 0.98 exp(-3 R/X).
    method_c_kappa = std::clamp(
        calculate_kappa_basic_sc(method_c_rx), 1.0, 2.0);
  }

  const double conv_gfm_contrib =
      std::max(0.0, converter_contrib - converter_current_source_contrib);

  for (int k = 0; k < n; ++k) {
    auto& row = out.bus_results[k];
    if (k != fault_idx) {
      if (!opt.compute_nonfault_currents) continue;
      const double current_source_ka =
          std::max(0.0, row.ikss_ka - row.ikss_1_ka);
      row.ip_ka = std::sqrt(2.0) *
                  (kappa * row.ikss_1_ka + current_source_ka);
      continue;
    }
    if (opt.kappa_method == SCKappaMethod::C) {
      // IEC 60909-0:2016, eq. (59): method C applies its single network
      // kappa to the equivalent-voltage-source current; converter/static-
      // generator current sources have no decaying dc component (kappa = 1).
      row.ip_ka = std::sqrt(2.0) *
          (method_c_kappa * I_kss_kA + sgen_contrib +
           converter_current_source_contrib);
      continue;
    }
    // IEC 60909-0:2016, 4.3.1.2 methods A/B: one method-specific kappa
    // applies to the total equivalent-voltage-source current. Static and
    // grid-following converter sources have no decaying dc component.
    row.ip_ka = std::sqrt(2.0) *
        (kappa * I_kss_kA + sgen_contrib +
         converter_current_source_contrib);
  }

  // ====== Step 6: Breaking current (Ib) ======
  std::vector<double> breaking_without_motors_ka(static_cast<size_t>(n), 0.0);
  {
    const double t_break = opt.breaking_time_s;

    for (int k = 0; k < n; ++k) {
      auto& row = out.bus_results[k];
      if (k != fault_idx && !opt.compute_nonfault_currents) continue;
      double Ib = 0.0;
      const bool formula_77_at_fault =
          k == fault_idx && opt.fault_type == FaultType::ThreePhase;
      if (formula_77_at_fault) Ib = row.ikss_ka;
      // IEC 60909-0:2016, 11.2.7: the steady current of a multiple-fed
      // near-to-generator fault is Ibmo, the breaking current evaluated after
      // removing every asynchronous-motor contribution.
      double Ibmo = formula_77_at_fault
          ? std::max(0.0, row.ikss_ka - row.ikss_motor_contrib_ka -
                               row.ikss_load_contrib_ka)
          : 0.0;

      // Generator contribution to breaking current
      for (const auto& g : ac.generators) {
        if (!g.in_service) continue;
        auto git = id_map.find(g.bus);
        if (git == id_map.end()) continue;
        int gi = git->second;

        double xdpp = (g.xdpp_pu > 1e-6) ? g.xdpp_pu : opt.default_xdpp;
        double mbase = (g.mbase_mva > 1e-6) ? g.mbase_mva : base_mva;
        Cx z_gen_corr = generator_impedance(g, xdpp, g.ra_pu);

        double gen_current = 0.0, gen_current_tr = 0.0;
        const Cx Z_k_b = Z1_fault + Zf;
        if (opt.fault_type == FaultType::ThreePhase && std::abs(z_gen_corr) > 1e-15) {
          gen_current = (c / std::abs(z_gen_corr)) * I_base;
          gen_current_tr = (std::abs(Z_k_b) > 1e-15)
              ? (c * std::abs(Zbus_col(gi))) / (std::abs(z_gen_corr) * std::abs(Z_k_b)) * I_base
              : 0.0;
        }

        if (gen_current > 1e-15) {
          // IEC 60909-0:2016, 9.1.1 and eq. (67): mu uses the generator
          // terminal partial current referred by the rated transformation
          // ratio, divided by the generator nameplate current. Computing it
          // from the actual source-port voltage drop preserves a generator
          // rated voltage that differs from the connected bus nominal value.
          const double source_bus_kv = bus_kv(g.bus);
          const double source_base_ka =
              base_mva / (std::sqrt(3.0) * source_bus_kv);
          const double terminal_current_ka =
              std::abs((Cx(c, 0.0) - V_fault(gi)) / z_gen_corr) *
              source_base_ka;
          const double generator_rated_kv =
              g.vn_kv > 1e-6 ? g.vn_kv : source_bus_kv;
          const double rated_terminal_ka =
              mbase / (std::sqrt(3.0) * generator_rated_kv);
          double Ik_ratio = rated_terminal_ka > 1e-15
              ? terminal_current_ka / rated_terminal_ka : 0.0;
          double mu = compute_mu(Ik_ratio, t_break);
          if (formula_77_at_fault) {
            // IEC 60909-0:2016, eq. (77): weight the decaying partial current
            // by the machine terminal-voltage depression
            // |Z_G I"_kG|/(c U_n/sqrt(3)). In per unit this ratio is
            // |Z_G| I"_kG,pu / c.
            const double partial_pu = gen_current_tr / I_base;
            const double voltage_depression =
                std::abs(z_gen_corr) * partial_pu / std::max(c, 1e-15);
            Ib -= voltage_depression * (1.0 - mu) * gen_current_tr;
            Ibmo -= voltage_depression * (1.0 - mu) * gen_current_tr;
          } else {
            Ib += gen_current_tr * mu;
            Ibmo += gen_current_tr * mu;
          }
        }
      }

      // Motor contribution to breaking current at all buses
      if (opt.calc_type == SCCalcType::Max) for (const auto& m : ac.motors) {
        if (!m.in_service) continue;
        const double motor_p_mw = m.sn_mva * std::clamp(m.cos_phi, 0.01, 1.0);
        if (motor_p_mw < kMinMotorContributionMw) continue;
        auto mit = id_map.find(m.bus);
        if (mit == id_map.end()) continue;
        Cx z_motor_nameplate(m.r_pu, m.x_pu);
        if (std::abs(z_motor_nameplate) < 1e-15 || m.sn_mva <= 1e-9) continue;
        Cx z_motor = z_motor_nameplate * (base_mva / m.sn_mva);
        double motor_current = (c / std::abs(z_motor)) * I_base;

        int mi = mit->second;
        double Zk_abs = std::abs(Z1_fault + Zf);
        double motor_current_tr = (Zk_abs > 1e-15)
            ? motor_current * std::abs(Zbus_col(mi)) / Zk_abs
            : 0.0;

        if (motor_current > 1e-15) {
          double sn = (m.sn_mva > 1e-6) ? m.sn_mva : 1.0;
          const double source_bus_kv = bus_kv(m.bus);
          const double source_base_ka =
              base_mva / (std::sqrt(3.0) * source_bus_kv);
          const double terminal_current_ka =
              std::abs((Cx(c, 0.0) - V_fault(mi)) / z_motor) * source_base_ka;
          const double motor_rated_kv =
              m.vn_kv > 1e-6 ? m.vn_kv : source_bus_kv;
          const double rated_terminal_ka =
              sn / (std::sqrt(3.0) * motor_rated_kv);
          double Ik_ratio = rated_terminal_ka > 1e-15
              ? terminal_current_ka / rated_terminal_ka : 0.0;
          double mu = compute_mu(Ik_ratio, t_break);
          double pn_mw = sn * m.cos_phi * m.efficiency;
          // IEC 60909 q factor uses PrM/p with p = pole pairs per motor
          // (AsynchronousMotor::poles carries the IEC pole-pair count).
          double q = compute_q(pn_mw, m.poles, t_break);
          if (formula_77_at_fault) {
            const double partial_pu = motor_current_tr / I_base;
            const double voltage_depression =
                std::abs(z_motor) * partial_pu / std::max(c, 1e-15);
            Ib -= voltage_depression * (1.0 - mu * q) * motor_current_tr;
          } else {
            Ib += mu * q * motor_current_tr;
          }
        }
      }

      if (opt.calc_type == SCCalcType::Max) for (const auto& ld : ac.loads) {
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
        const double Zk_abs = std::abs(Z1_fault + Zf);
        const double motor_current_tr =
            (Zk_abs > 1e-15)
                ? motor_current * std::abs(Zbus_col(lit->second)) / Zk_abs
                : 0.0;
        if (motor_current <= 1e-15) continue;
        const double sn = (actual_s > 1e-6) ? actual_s : 1.0;
        const double source_bus_kv = bus_kv(ld.bus);
        const double source_base_ka =
            base_mva / (std::sqrt(3.0) * source_bus_kv);
        const double terminal_current_ka =
            std::abs((Cx(c, 0.0) - V_fault(lit->second)) / z_load) *
            source_base_ka;
        const double rated_terminal_ka =
            sn / (std::sqrt(3.0) * source_bus_kv);
        const double Ik_ratio = rated_terminal_ka > 1e-15
            ? terminal_current_ka / rated_terminal_ka : 0.0;
        const double mu = compute_mu(Ik_ratio, t_break);
        const double q = compute_q(
            load_motor_pn_mw(ld, motor_fraction),
            ld.motor_poles,
            t_break);
        if (formula_77_at_fault) {
          const double partial_pu = motor_current_tr / I_base;
          const double voltage_depression =
              std::abs(z_load) * partial_pu / std::max(c, 1e-15);
          Ib -= voltage_depression * (1.0 - mu * q) * motor_current_tr;
        } else {
          Ib += mu * q * motor_current_tr;
        }
      }

      if (!formula_77_at_fault) {
        // Transferred non-fault indicators retain their scalar decomposition.
        Ib += row.ikss_no_motor_ka;
        Ibmo += row.ikss_no_motor_ka;
      }

      // IEC 60909-0:2016, 9.2, eqs. (78)-(80): unbalanced breaking current
      // equals the corresponding initial current; generator flux decay is not
      // applied to unbalanced faults.
      row.ib_ka = opt.fault_type == FaultType::ThreePhase
          ? std::max(0.0, Ib) : row.ikss_ka;
      breaking_without_motors_ka[static_cast<size_t>(k)] =
          opt.fault_type == FaultType::ThreePhase
              ? std::max(0.0, Ibmo)
              : row.ikss_ka;
    }
  }

  // ====== Step 7: Steady-state current (Ik) ======
  // IEC 60909-0:2016, 11.2: classify the fault before evaluating Ik. A
  // generator is near when its initial contribution exceeds 2*IrG. A
  // single-fed near-generator fault uses manufacturer/curve lambda; a
  // multiple-fed near fault uses Ibmo; network feeders retain Ik'', while
  // asynchronous motors contribute zero in steady state.
  std::vector<const GeneratorFaultContribution*> near_generators;
  for (const auto& contribution : generator_fault_contributions) {
    if (contribution.initial_ka >
        2.0 * contribution.rated_current_at_fault_kv_ka)
      near_generators.push_back(&contribution);
  }
  const int voltage_source_count =
      static_cast<int>(generator_fault_contributions.size()) +
      external_grid_source_count + grid_forming_converter_source_count;

  double grid_forming_converter_steady_ka = 0.0;
  for (const auto& conv : projected.vsc_converters) {
    if (!conv.in_service || !is_ac_grid_forming_converter(conv) ||
        conv.p_rated_mw <= 1e-6)
      continue;
    const double rated_ka =
        conv.p_rated_mw / (std::sqrt(3.0) * bus_kv(conv.bus_ac));
    grid_forming_converter_steady_ka += current_source_contribution_ka(
        conv.bus_ac, converter_current_multiplier(conv) * rated_ka);
  }

  const bool single_fed_near =
      near_generators.size() == 1 && voltage_source_count == 1;
  double single_generator_steady_fault_ka = 0.0;
  if (single_fed_near) {
    const auto& contribution = *near_generators.front();
    const Generator& generator = *contribution.generator;
    const bool terminal_static_excitation =
        generator.sc_terminal_fed_static_excitation &&
        generator.bus == resolved_fault_bus_id;
    // IEC 60909-0:2016, 11.2.1.2: for a terminal short circuit of a
    // generator with terminal-fed static excitation, lambda_max = lambda_min.
    const double lambda = terminal_static_excitation
        ? generator.sc_lambda_min
        : (opt.calc_type == SCCalcType::Max
               ? generator.sc_lambda_max
               : generator.sc_lambda_min);
    if (!std::isfinite(lambda) || lambda <= 0.0) {
      out.status = "invalid_generator_data";
      out.message = "Generator " + std::to_string(generator.index) +
          " requires a positive " +
          (terminal_static_excitation || opt.calc_type == SCCalcType::Min
               ? std::string("sc_lambda_min")
               : std::string("sc_lambda_max")) +
          " for an IEC single-fed near-to-generator fault";
      return out;
    }
    single_generator_steady_fault_ka =
        lambda * contribution.rated_current_at_fault_kv_ka;
  }

  for (int k = 0; k < n; ++k) {
    auto& row = out.bus_results[k];
    if (k != fault_idx && !opt.compute_nonfault_currents) continue;
    if (single_fed_near) {
      const double transfer = gen_contrib > 1e-15
          ? row.ikss_gen_contrib_ka / gen_contrib : 0.0;
      row.ik_ka = std::max(
          0.0, single_generator_steady_fault_ka * transfer +
                   row.ikss_sgen_contrib_ka + row.ikss_converter_contrib_ka);
    } else if (!near_generators.empty() && voltage_source_count > 1) {
      row.ik_ka = breaking_without_motors_ka[static_cast<size_t>(k)];
      if (k == fault_idx) {
        row.ik_ka = std::max(
            0.0, row.ik_ka - conv_gfm_contrib +
                     grid_forming_converter_steady_ka);
      }
    } else {
      row.ik_ka = std::max(
          0.0, row.ikss_ka - row.ikss_motor_contrib_ka -
                   row.ikss_load_contrib_ka);
      if (k == fault_idx) {
        row.ik_ka = std::max(
            0.0, row.ik_ka - conv_gfm_contrib +
                     grid_forming_converter_steady_ka);
      }
    }
  }

  // ====== Step 8: Thermal equivalent current I_th (IEC 60909 §8) ======
  if (opt.compute_ith) {
    const double Tk = opt.ith_duration_s;
    for (int k = 0; k < n; ++k) {
      auto& row = out.bus_results[k];
      if (k != fault_idx && !opt.compute_nonfault_currents) continue;

      // IEC 60909-0:2016, Annex A defines m from the peak factor belonging to
      // the reported total current. Methods A/B may sum source-specific peak
      // factors and method C owns a separate fc/f network, so the fault-point
      // R/X factor is not generally the factor represented by row.ip_ka.
      const double thermal_kappa = row.ikss_ka > 1e-15
          ? std::clamp(row.ip_ka /
                           (std::sqrt(2.0) * row.ikss_ka),
                       1.0, 2.0)
          : 1.0;
      const auto factors = calculate_thermal_factors_sc(
          thermal_kappa, row.ikss_ka, row.ik_ka,
          opt.base_frequency_hz, Tk);
      row.thermal_m = factors.m;
      row.thermal_n = factors.n;
      row.ith_ka = row.ikss_ka * std::sqrt(factors.m + factors.n);
    }
  }

  // ====== Step 9: Branch fault currents and power flows ======
  if (opt.compute_branch_flows) {
    // Branch currents from voltage differences
    std::unordered_map<int, bool> emitted_canonical_branches;
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
      br_res.component_index = br.index;
      br_res.i_from_ka = std::abs(I_ft) * I_base;
      br_res.i_to_ka = std::abs(I_tf) * I_base;
      br_res.i_branch_ka = std::max(br_res.i_from_ka, br_res.i_to_ka);

      // Apparent power: S = V * I* * S_base
      double s_base = base_mva;
      br_res.s_branch_mva = std::max(
          std::abs(V_f * std::conj(I_ft)) * s_base,
          std::abs(V_t * std::conj(I_tf)) * s_base);

      bool attributed = false;
      if (projected.branch_expand_map) {
        for (const auto& entry : projected.branch_expand_map->entries) {
          if (entry.branch_index != br.index) continue;
          auto authored = br_res;
          authored.component_index = entry.origin_index;
          authored.pair_number = entry.origin_type == BranchOriginType::Transformer3W
                                     ? entry.pair_number : -1;
          switch (entry.origin_type) {
            case BranchOriginType::Transformer2W: authored.component_kind = "Transformer2W"; break;
            case BranchOriginType::Transformer3W: authored.component_kind = "Transformer3W"; break;
            case BranchOriginType::Switch: authored.component_kind = "Switch"; break;
            case BranchOriginType::CircuitBreaker: authored.component_kind = "CircuitBreaker"; break;
          }
          out.branch_results.push_back(std::move(authored));
          attributed = true;
        }
      }
      if (!attributed) out.branch_results.push_back(std::move(br_res));
      emitted_canonical_branches[br.index] = true;
    }
    if (projected.branch_expand_map) {
      for (const auto& entry : projected.branch_expand_map->entries) {
        if (emitted_canonical_branches.count(entry.branch_index) != 0) continue;
        if (entry.origin_type != BranchOriginType::Switch &&
            entry.origin_type != BranchOriginType::CircuitBreaker) continue;
        SCDetailedBranchResult row;
        row.branch_index = entry.branch_index;
        row.component_kind = entry.origin_type == BranchOriginType::Switch
                                 ? "Switch" : "CircuitBreaker";
        row.component_index = entry.origin_index;
        row.from_bus = entry.bus_from;
        row.to_bus = entry.bus_to;
        row.electrical_value_available = false;
        row.message = "ideal device was contracted; individual current is not identifiable without a current-sharing model";
        out.branch_results.push_back(std::move(row));
      }
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

  auto finite_result = [](const SCDetailedResult& result) {
    if (!std::isfinite(result.effective_c_factor)) return false;
    for (const auto& row : result.bus_results) {
      const double values[] = {row.ikss_ka, row.ip_ka, row.ib_ka, row.ik_ka,
                               row.ith_ka, row.thermal_m, row.thermal_n,
                               row.v_remaining_pu};
      for (double value : values) if (!std::isfinite(value)) return false;
    }
    for (const auto& row : result.branch_results) {
      const double values[] = {row.i_branch_ka, row.i_from_ka, row.i_to_ka,
                               row.s_branch_mva};
      for (double value : values) if (!std::isfinite(value)) return false;
    }
    return true;
  };
  out.numerical_quality.max_linear_residual = sparse_context.max_residual();
  out.numerical_quality.all_finite = finite_result(out);
  if (!out.numerical_quality.all_finite) {
    out.status = "numerical_failure";
    out.message = "non-finite short-circuit result";
    return out;
  }
  out.solved = true;
  out.status = "solved";
  out.message = "ok";
  return out;
}

std::optional<SCDetailedResult> run_current_source_only_fault(
    const HybridPowerSystem& sys, int fault_bus_id,
    const SCDetailedOptions& opt,
    const projection::ProjectionBundle& projection_bundle) {
  const auto& projected = projection_bundle.canonical;
  const auto& ac = projected.ac;
  if (opt.fault_type != FaultType::ThreePhase) return std::nullopt;

  bool has_voltage_source = false;
  for (const auto& generator : ac.generators)
    has_voltage_source = has_voltage_source || generator.in_service;
  for (const auto& grid : ac.external_grids)
    has_voltage_source = has_voltage_source || grid.in_service;
  for (const auto& motor : ac.motors)
    has_voltage_source = has_voltage_source || motor.in_service;
  for (const auto& converter : projected.vsc_converters)
    has_voltage_source = has_voltage_source ||
        (converter.in_service && is_ac_grid_forming_converter(converter));
  if (has_voltage_source) return std::nullopt;

  const auto id_map = build_id_map(ac.buses);
  int resolved_fault_bus = fault_bus_id;
  if (projected.bus_merge_map) {
    const auto found = projected.bus_merge_map->ext_to_int.find(fault_bus_id);
    if (found != projected.bus_merge_map->ext_to_int.end())
      resolved_fault_bus = found->second + 1;
  }
  const auto fault_it = id_map.find(resolved_fault_bus);
  if (fault_it == id_map.end()) return std::nullopt;

  std::vector<std::vector<int>> adjacency(ac.buses.size());
  for (const auto& branch : ac.branches) {
    if (!branch.in_service) continue;
    const auto from = id_map.find(branch.from_bus);
    const auto to = id_map.find(branch.to_bus);
    if (from == id_map.end() || to == id_map.end()) continue;
    adjacency[static_cast<size_t>(from->second)].push_back(to->second);
    adjacency[static_cast<size_t>(to->second)].push_back(from->second);
  }
  std::vector<char> connected(ac.buses.size(), 0);
  std::vector<int> frontier{fault_it->second};
  connected[static_cast<size_t>(fault_it->second)] = 1;
  for (size_t cursor = 0; cursor < frontier.size(); ++cursor) {
    for (int next : adjacency[static_cast<size_t>(frontier[cursor])]) {
      if (connected[static_cast<size_t>(next)]) continue;
      connected[static_cast<size_t>(next)] = 1;
      frontier.push_back(next);
    }
  }

  const double base_mva = ac.base_mva > 0.0 ? ac.base_mva : 100.0;
  double total_ka = 0.0;
  double static_ka = 0.0;
  double converter_ka = 0.0;
  SCDetailedResult out;
  out.fault_bus_id = fault_bus_id;
  out.effective_c_factor = detailed_voltage_factor(
      ac.buses[static_cast<size_t>(fault_it->second)].base_kv, opt);
  out.model_scope = "iec60909-current-source-only-v1";
  out.model_limitations = {
      "With no AC voltage source, the bolted fault grounds the connected passive component and all aligned IEC current-source injections in that component feed the fault.",
      "Voltage profile, passive-branch currents, peak DC offset, and unbalanced converter sequence controls are indeterminate in the current-source-only model and are not returned."};

  for (const auto& source : ac.static_generators) {
    if (!source.in_service || source.sn_mva <= 0.0 || source.k <= 0.0) continue;
    const auto position = id_map.find(source.bus);
    if (position == id_map.end() || !connected[static_cast<size_t>(position->second)]) continue;
    const double kv = ac.buses[static_cast<size_t>(position->second)].base_kv;
    const double current = source.k * source.sn_mva / (std::sqrt(3.0) * kv);
    static_ka += current;
    total_ka += current;
  }
  for (const auto& converter : projected.vsc_converters) {
    if (!converter.in_service || is_ac_grid_forming_converter(converter) ||
        converter.p_rated_mw <= 0.0) continue;
    const auto position = id_map.find(converter.bus_ac);
    if (position == id_map.end() || !connected[static_cast<size_t>(position->second)]) continue;
    const double kv = ac.buses[static_cast<size_t>(position->second)].base_kv;
    const double limit = converter_current_multiplier_sc(converter);
    const double current = limit * converter.p_rated_mw / (std::sqrt(3.0) * kv);
    converter_ka += current;
    total_ka += current;
    SCConverterContributionResult row;
    row.converter_index = converter.index;
    row.bus_id = projected.bus_merge_map
        ? projected.bus_merge_map->int_to_ext[static_cast<size_t>(position->second)]
        : converter.bus_ac;
    row.name = converter.name;
    row.model = resolve_device_control_role(converter).is_dc_grid_forming
        ? "dc_side_forming_current_limited_source"
        : "grid_following_current_source";
    row.ac_grid_forming = false;
    row.dc_grid_forming = resolve_device_control_role(converter).is_dc_grid_forming;
    row.p_rated_mw = converter.p_rated_mw;
    row.i_limit_pu = limit;
    row.contribution_ka = current;
    out.converter_contributions.push_back(std::move(row));
  }
  if (!(total_ka > 0.0) || !std::isfinite(total_ka)) return std::nullopt;

  out.bus_results.reserve(sys.ac.buses.size());
  for (const auto& bus : sys.ac.buses) {
    SCDetailedBusResult row;
    row.bus_id = bus.index;
    row.v_remaining_pu = bus.index == fault_bus_id ? 0.0 : 1.0;
    if (bus.index == fault_bus_id) {
      row.ikss_ka = total_ka;
      row.ikss_1_ka = total_ka;
      row.ikss_sgen_contrib_ka = static_ka;
      row.ikss_converter_contrib_ka = converter_ka;
      row.i_phase_a_ka = total_ka;
      row.i_phase_b_ka = total_ka;
      row.i_phase_c_ka = total_ka;
      row.ip_ka = std::sqrt(2.0) * total_ka;
      row.ib_ka = total_ka;
      row.ik_ka = total_ka;
      row.ith_ka = opt.compute_ith ? total_ka : 0.0;
    }
    out.bus_results.push_back(std::move(row));
  }
  out.numerical_quality.factorization_valid = true;
  out.numerical_quality.all_finite = true;
  out.solved = true;
  out.status = "solved_with_limitations";
  out.message = "ok: current-source-only fault model";
  return out;
}

SCDetailedResult run_short_circuit_detailed(const HybridPowerSystem& sys,
                                            int fault_bus_id,
                                            const SCDetailedOptions& opt) {
  validate_detailed_options(opt);
  const auto projection_bundle =
      projection::RichToCanonicalOperator::apply(sys);
  const auto& projected = projection_bundle.canonical;
  const auto id_map = build_id_map(projected.ac.buses);
  const auto iec_corrections = build_iec_network_corrections(
      sys, projected, opt, fault_bus_id);
  std::optional<IecNetworkCorrections> method_c_corrections;
  if (opt.kappa_method == SCKappaMethod::C) {
    method_c_corrections = build_iec_network_corrections(
        sys, projected, opt, fault_bus_id, kMethodCFrequencyRatio);
  }
  auto sparse_context = build_detailed_sparse_context(
      projected, id_map, opt, iec_corrections,
      method_c_corrections ? &*method_c_corrections : nullptr);
  if (!sparse_context.valid &&
      sparse_context.failure_status == "numerical_failure") {
    if (auto current_only = run_current_source_only_fault(
            sys, fault_bus_id, opt, projection_bundle))
      return std::move(*current_only);
  }
  return run_short_circuit_detailed_impl(
      sys, fault_bus_id, opt, projection_bundle, sparse_context);
}

std::vector<SCDetailedResult> run_short_circuit_detailed_batch(const HybridPowerSystem& sys,
                                                               const std::vector<int>& fault_bus_ids,
                                                               const SCDetailedOptions& opt) {
  validate_detailed_options(opt);
  const auto projection_bundle =
      projection::RichToCanonicalOperator::apply(sys);
  const auto& projected = projection_bundle.canonical;
  const auto id_map = build_id_map(projected.ac.buses);
  const auto iec_corrections = build_iec_network_corrections(
      sys, projected, opt);
  std::optional<IecNetworkCorrections> method_c_corrections;
  if (opt.kappa_method == SCKappaMethod::C) {
    method_c_corrections = build_iec_network_corrections(
        sys, projected, opt, 0, kMethodCFrequencyRatio);
  }
  auto sparse_context = build_detailed_sparse_context(
      projected, id_map, opt, iec_corrections,
      method_c_corrections ? &*method_c_corrections : nullptr);

  std::vector<SCDetailedResult> out;
  out.reserve(fault_bus_ids.size());
  for (const int bus_id : fault_bus_ids) {
    if (opt.cancellation_requested && opt.cancellation_requested()) {
      throw std::runtime_error("short-circuit analysis cancelled");
    }
    const bool station_internal_fault = std::any_of(
        sys.ac.generators.begin(), sys.ac.generators.end(),
        [bus_id](const Generator& generator) {
          return generator.in_service && generator.bus == bus_id &&
                 generator.power_station_transformer_index > 0;
        });
    if (station_internal_fault) {
      const auto local_corrections = build_iec_network_corrections(
          sys, projected, opt, bus_id);
      std::optional<IecNetworkCorrections> local_method_c_corrections;
      if (opt.kappa_method == SCKappaMethod::C) {
        local_method_c_corrections = build_iec_network_corrections(
            sys, projected, opt, bus_id, kMethodCFrequencyRatio);
      }
      auto local_context = build_detailed_sparse_context(
          projected, id_map, opt, local_corrections,
          local_method_c_corrections ? &*local_method_c_corrections : nullptr);
      out.push_back(run_short_circuit_detailed_impl(
          sys, bus_id, opt, projection_bundle, local_context));
      continue;
    }
    if (!sparse_context.valid &&
        sparse_context.failure_status == "numerical_failure") {
      if (auto current_only = run_current_source_only_fault(
              sys, bus_id, opt, projection_bundle)) {
        out.push_back(std::move(*current_only));
        continue;
      }
    }
    out.push_back(run_short_circuit_detailed_impl(
        sys, bus_id, opt, projection_bundle, sparse_context));
  }
  return out;
}

}  // namespace hacdcpf::analysis
