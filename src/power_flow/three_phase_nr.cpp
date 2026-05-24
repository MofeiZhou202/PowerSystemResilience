// Three-Phase Newton-Raphson Power Flow Solver (abc domain)
//
// Polar-form NR on a compact abc-domain admittance matrix built only on
// active phase nodes. ThreePhaseACLine may contribute either
// sequence-derived circulant impedances or explicit full phase-domain
// matrices.
//
// State vector: x = [θ_{a1}, θ_{b1}, θ_{c1}, ..., θ_{aN}, θ_{bN}, θ_{cN},
//                     Vm_{a1}, Vm_{b1}, Vm_{c1}, ..., Vm_{aN}, Vm_{bN}, Vm_{cN}]
// (slack phases excluded from θ; PQ phases included in Vm mismatch)
//
// Mismatch equations:
//   ΔP_φi = P_spec_φi − Σ_j Σ_ψ |Vi_φ||Vj_ψ|(G_φψ_ij cos(θ_φi − θ_ψj) +
//                                                B_φψ_ij sin(θ_φi − θ_ψj))
//   ΔQ_φi = Q_spec_φi − Σ_j Σ_ψ |Vi_φ||Vj_ψ|(G_φψ_ij sin(θ_φi − θ_ψj) −
//                                                B_φψ_ij cos(θ_φi − θ_ψj))
//
// Reference: Arrillaga & Watson, "Computer Modelling of Electrical Power
// Systems", 2nd ed., Wiley, 2001, Chapter 7.

#include "hacdcpf/power_flow/distribution_power_flow.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/SparseLU>

#include "hacdcpf/detail/logging.hpp"
#include <Eigen/Sparse>

#include "hacdcpf/engine/kernel/linear_algebra/linear_solver.hpp"
#include "hacdcpf/engine/native_adapters.hpp"
#include "hacdcpf/model/ac_components.hpp"

namespace hacdcpf::analysis {

PhaseNodeIndexer PhaseNodeIndexer::build(const ThreePhaseACSystem& sys) {
  PhaseNodeIndexer indexer;
  indexer.bus_phase_to_node.resize(sys.buses.size(), std::array<int, 3>{-1, -1, -1});
  indexer.bus_phase_masks.reserve(sys.buses.size());

  for (size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    const auto& bus = sys.buses[bus_offset];
    const PhaseMask mask = bus.phase_mask;
    if (mask.empty()) {
      throw std::runtime_error(
          "PhaseNodeIndexer: bus " + std::to_string(bus.index) +
          " has empty phase_mask");
    }

    indexer.bus_phase_masks.push_back(mask);
    for (int phase = 0; phase < 3; ++phase) {
      if (!mask.has(phase)) continue;
      const int compact_index = indexer.total_nodes++;
      indexer.bus_phase_to_node[bus_offset][phase] = compact_index;
      indexer.nodes.push_back(
          PhaseNodeRef{compact_index, static_cast<int>(bus_offset), bus.index, phase});
    }
  }

  return indexer;
}

bool PhaseNodeIndexer::has_node(int bus_offset, int phase_index) const {
  return bus_offset >= 0 &&
         bus_offset < static_cast<int>(bus_phase_to_node.size()) &&
         phase_index >= 0 && phase_index < 3 &&
         bus_phase_to_node[bus_offset][phase_index] >= 0;
}

int PhaseNodeIndexer::node_index(int bus_offset, int phase_index) const {
  if (!has_node(bus_offset, phase_index)) {
    throw std::runtime_error(
        "PhaseNodeIndexer: missing node for bus_offset=" +
        std::to_string(bus_offset) + " phase=" + std::to_string(phase_index));
  }
  return bus_phase_to_node[bus_offset][phase_index];
}

const PhaseNodeRef& PhaseNodeIndexer::node_ref(int compact_index) const {
  if (compact_index < 0 || compact_index >= static_cast<int>(nodes.size())) {
    throw std::runtime_error(
        "PhaseNodeIndexer: compact index out of range: " +
        std::to_string(compact_index));
  }
  return nodes[compact_index];
}

PhaseMask PhaseNodeIndexer::bus_phase_mask(int bus_offset) const {
  if (bus_offset < 0 || bus_offset >= static_cast<int>(bus_phase_masks.size())) {
    throw std::runtime_error(
        "PhaseNodeIndexer: bus offset out of range: " +
        std::to_string(bus_offset));
  }
  return bus_phase_masks[bus_offset];
}

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPhaseValueTol = 1e-12;

using Complex = std::complex<double>;
using ComplexTriplet = Eigen::Triplet<Complex>;
using ComplexMatrixX = Eigen::Matrix<Complex, Eigen::Dynamic, Eigen::Dynamic>;
using ComplexVectorX = Eigen::Matrix<Complex, Eigen::Dynamic, 1>;

struct PhaseTerminal {
  int bus_offset{-1};
  PhaseMask mask{PhaseMask::none()};
  std::vector<int> phases;
  std::vector<int> nodes;
};

struct CanonicalTapProjection {
  double branch_tap_pu{1.0};
  double impedance_scale{1.0};
};

struct TransformerPrimitiveRecord;

struct PhaseNetworkModel {
  Eigen::SparseMatrix<Complex> ybus;
  ComplexVectorX fixed_current;
  std::vector<TransformerPrimitiveRecord> transformer_primitives;
};

enum class LoadNeutralKind {
  SolidGrounded,
  OpenNeutral,
  ImpedanceGrounded,
};

struct LoadZipWeights {
  double z_fraction{0.0};
  double i_fraction{0.0};
  double p_fraction{1.0};
};

struct PreparedWyeLoadContribution {
  int load_index{0};
  std::string load_name;
  int bus_offset{-1};
  std::vector<int> phase_indices;
  std::vector<int> nodes;
  LoadNeutralKind neutral_kind{LoadNeutralKind::SolidGrounded};
  Complex y_neutral_pu{0.0, 0.0};
  ComplexVectorX y_shunt_pu;
  ComplexVectorX s_current_nominal_pu;
  ComplexVectorX s_power_nominal_pu;
  ComplexMatrixX reduced_y_shunt_pu;
  double vmin_pu{0.95};
  double vmax_pu{1.05};
  double zipv_cutoff_pu{0.0};
};

struct PreparedBranchLoadContribution {
  int load_index{0};
  std::string load_name;
  int bus_offset{-1};
  int from_phase{-1};
  int to_phase{-1};
  int from_node{-1};
  int to_node{-1};
  Complex y_branch_pu{0.0, 0.0};
  Complex s_current_nominal_pu{0.0, 0.0};
  Complex s_power_nominal_pu{0.0, 0.0};
  double vmin_pu{0.95};
  double vmax_pu{1.05};
  double zipv_cutoff_pu{0.0};
};

struct PreparedLoadSet {
  std::vector<PreparedWyeLoadContribution> wye_loads;
  std::vector<PreparedBranchLoadContribution> branch_loads;
};

void stamp_prepared_constant_impedance_loads(
    std::vector<ComplexTriplet>& triplets,
    const PreparedLoadSet& prepared_loads);

struct PrimitiveBranchModel {
  PhaseTerminal from_terminal;
  PhaseTerminal to_terminal;
  ComplexMatrixX y_ff;
  ComplexMatrixX y_ft;
  ComplexMatrixX y_tf;
  ComplexMatrixX y_tt;
};

struct PrimitiveBranchObservation {
  ComplexVectorX from_voltage;
  ComplexVectorX to_voltage;
  ComplexVectorX from_current;
  ComplexVectorX to_current;
};

struct TransformerPrimitiveRecord {
  int transformer_index{0};
  std::string transformer_name;
  int hv_bus{0};
  int lv_bus{0};
  PhaseMask hv_phase_mask{PhaseMask::none()};
  PhaseMask lv_phase_mask{PhaseMask::none()};
  double hv_base_current_amps{0.0};
  double lv_base_current_amps{0.0};
  PrimitiveBranchModel primitive;
};

struct ThreePhaseSolveSnapshot {
  ThreePhaseDPFResult result;
  std::unordered_map<int, int> id_map;
};

enum class TransformerConnectionKind {
  GroundedWye,
  Wye,
  Delta,
  Unsupported,
};

struct ParsedVectorGroup {
  TransformerConnectionKind hv{TransformerConnectionKind::Unsupported};
  TransformerConnectionKind lv{TransformerConnectionKind::Unsupported};
  bool hv_grounded{false};
  bool lv_grounded{false};
  int clock{0};
};

struct TransformerConnectionEmbedding {
  ComplexMatrixX matrix;
  std::vector<int> winding_labels;
  bool explicit_topology{false};
};

enum class SourceConnectionKind {
  Wye,
  Delta,
};

struct SourceConnectionEmbedding {
  TransformerConnectionEmbedding embedding;
  SourceConnectionKind kind{SourceConnectionKind::Wye};
};

struct SourceNortonPrimitive {
  ComplexMatrixX y_phase;
  ComplexVectorX fixed_current;
  ComplexVectorX open_circuit_phase_voltage;
};

struct MixedConnectionClockProjection {
  bool delta_positive_thirty_deg{false};
  double phase_polarity{1.0};
};

struct ZeroSequencePortAdmittance {
  Complex y_ff{0.0, 0.0};
  Complex y_ft{0.0, 0.0};
  Complex y_tf{0.0, 0.0};
  Complex y_tt{0.0, 0.0};
};

SourceNortonPrimitive build_source_norton_primitive(
    const ThreePhaseExternalGrid& source,
    const PhaseTerminal& terminal,
    double base_mva);

std::string phase_name(int phase_index) {
  if (phase_index == 0) return "A";
  if (phase_index == 1) return "B";
  if (phase_index == 2) return "C";
  return "?";
}

std::vector<int> active_phase_indices(PhaseMask mask) {
  std::vector<int> phases;
  phases.reserve(mask.count());
  for (int phase = 0; phase < 3; ++phase) {
    if (mask.has(phase)) phases.push_back(phase);
  }
  return phases;
}

void require_mask_not_empty(
    const std::string& entity,
    int entity_index,
    PhaseMask mask) {
  if (mask.empty()) {
    throw std::runtime_error(
        entity + " " + std::to_string(entity_index) +
        " has empty phase_mask");
  }
}

void require_phase_values_on_mask(
    const std::string& entity,
    int entity_index,
    PhaseMask mask,
    const std::array<double, 3>& p_values,
    const std::array<double, 3>& q_values) {
  for (int phase = 0; phase < 3; ++phase) {
    if (mask.has(phase)) continue;
    if (std::abs(p_values[phase]) > kPhaseValueTol ||
        std::abs(q_values[phase]) > kPhaseValueTol) {
      throw std::runtime_error(
          entity + " " + std::to_string(entity_index) +
          " carries non-zero " + phase_name(phase) +
          "-phase values outside its phase_mask");
    }
  }
}

int single_phase_index_from_mask(
    const std::string& entity,
    int entity_index,
    PhaseMask mask) {
  if (mask.count() != 1) {
    throw std::runtime_error(
        entity + " " + std::to_string(entity_index) +
        " requires a single active phase");
  }
  for (int phase = 0; phase < 3; ++phase) {
    if (mask.has(phase)) return phase;
  }
  throw std::runtime_error(
      entity + " " + std::to_string(entity_index) +
      " has empty phase_mask");
}

double tap_from_step(int tap_pos, int tap_neutral, double tap_step_percent) {
  if (std::abs(tap_step_percent) < kPhaseValueTol) {
    return 1.0;
  }
  return std::max(
      1e-6,
      1.0 +
          (static_cast<double>(tap_pos - tap_neutral) * tap_step_percent / 100.0));
}

double transformer_tap_pu(const ThreePhaseTransformer& transformer) {
  return tap_from_step(
      transformer.tap_pos,
      transformer.tap_neutral,
      transformer.tap_step_percent);
}

int transformer_tap_number(const ThreePhaseTransformer& transformer) {
  return transformer.tap_pos - transformer.tap_neutral;
}

int transformer_winding_bus(const ThreePhaseTransformer& transformer, int winding) {
  if (winding == 1) return transformer.hv_bus;
  if (winding == 2) return transformer.lv_bus;
  throw std::runtime_error(
      "Unsupported three-phase transformer winding " + std::to_string(winding));
}

bool tap_side_matches_winding(const ThreePhaseTransformer& transformer, int tap_winding) {
  return (tap_winding == 1 && transformer.tap_side == 0) ||
         (tap_winding == 2 && transformer.tap_side == 1);
}

int tap_pos_delta_for_raise_voltage(const ThreePhaseTransformer& transformer) {
  return (transformer.tap_side == 1) ? 1 : -1;
}

double bus_base_voltage_volts(const ThreePhaseACBus& bus) {
  if (bus.base_kv <= kPhaseValueTol) {
    throw std::runtime_error(
        "Bus " + std::to_string(bus.index) +
        " is missing base_kv for three-phase regulator voltage control");
  }
  return bus.base_kv * 1000.0;
}

double bus_base_current_amps(double base_mva, const ThreePhaseACBus& bus) {
  return base_mva * 1000.0 / std::max(kPhaseValueTol, bus.base_kv);
}

int monitored_phase_index_from_node(int monitored_node) {
  if (monitored_node < 1 || monitored_node > 3) {
    throw std::runtime_error("unsupported_monitored_node");
  }
  return monitored_node - 1;
}

CanonicalTapProjection normalize_transformer_tap_to_from_side(
    int tap_side,
    double winding_tap_pu) {
  const double clamped = std::max(1e-6, winding_tap_pu);
  if (tap_side == 1) {
    return {
        .branch_tap_pu = 1.0 / clamped,
        .impedance_scale = clamped * clamped,
    };
  }
  return {
      .branch_tap_pu = clamped,
      .impedance_scale = 1.0,
  };
}

double shift_deg_from_clock(int clock) {
  const int normalized = ((12 - (clock % 12)) % 12);
  double shift = static_cast<double>(normalized) * 30.0;
  if (shift > 180.0) shift -= 360.0;
  return shift;
}

Complex same_connection_clock_factor(
    const ThreePhaseTransformer& transformer,
    const std::string& path_label,
    double effective_shift_deg) {
  auto matches = [&](double target_deg) {
    return std::abs(effective_shift_deg - target_deg) <= 1e-6;
  };

  if (matches(0.0) ||
      matches(30.0) ||
      matches(-30.0) ||
      matches(150.0) ||
      matches(-150.0) ||
      matches(180.0)) {
    return std::polar(1.0, -effective_shift_deg * kPi / 180.0);
  }
  throw std::runtime_error(
      "transformer " + std::to_string(transformer.index) + " " + path_label +
      " only supports 0, +/-30, +/-150, and 180 degree clock groups");
}

MixedConnectionClockProjection resolve_mixed_connection_clock(
    const ThreePhaseTransformer& transformer,
    const std::string& path_label,
    double effective_shift_deg,
    bool delta_on_from_side) {
  auto matches = [&](double target_deg) {
    return std::abs(effective_shift_deg - target_deg) <= 1e-6;
  };

  if (delta_on_from_side) {
    if (matches(30.0)) return {.delta_positive_thirty_deg = true, .phase_polarity = 1.0};
    if (matches(-30.0)) return {.delta_positive_thirty_deg = false, .phase_polarity = 1.0};
    if (matches(150.0)) return {.delta_positive_thirty_deg = false, .phase_polarity = -1.0};
    if (matches(-150.0)) return {.delta_positive_thirty_deg = true, .phase_polarity = -1.0};
  } else {
    if (matches(-30.0)) return {.delta_positive_thirty_deg = true, .phase_polarity = 1.0};
    if (matches(30.0)) return {.delta_positive_thirty_deg = false, .phase_polarity = 1.0};
    if (matches(150.0)) return {.delta_positive_thirty_deg = true, .phase_polarity = -1.0};
    if (matches(-150.0)) return {.delta_positive_thirty_deg = false, .phase_polarity = -1.0};
  }

  throw std::runtime_error(
      "transformer " + std::to_string(transformer.index) + " " + path_label +
      " only supports +/-30 and +/-150 degree clock groups");
}

bool is_wye_like_connection(TransformerConnectionKind kind) {
  return kind == TransformerConnectionKind::GroundedWye ||
         kind == TransformerConnectionKind::Wye;
}

bool is_delta_connection(TransformerConnectionKind kind) {
  return kind == TransformerConnectionKind::Delta;
}

std::string uppercase_ascii(std::string text) {
  std::transform(
      text.begin(),
      text.end(),
      text.begin(),
      [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });
  return text;
}

ParsedVectorGroup parse_vector_group(const std::string& raw_group) {
  ParsedVectorGroup parsed;
  std::string letters;
  std::string digits;
  for (char ch : uppercase_ascii(raw_group)) {
    if (std::isdigit(static_cast<unsigned char>(ch))) {
      digits.push_back(ch);
    } else if (std::isalpha(static_cast<unsigned char>(ch))) {
      letters.push_back(ch);
    }
  }

  if (!digits.empty()) {
    parsed.clock = std::stoi(digits) % 12;
  }

  size_t pos = 0;
  auto parse_side = [&](TransformerConnectionKind& kind, bool& grounded) {
    if (pos >= letters.size()) return;
    const char conn = letters[pos++];
    if (conn == 'D') {
      kind = TransformerConnectionKind::Delta;
      grounded = false;
      return;
    }
    if (conn == 'Y') {
      grounded = pos < letters.size() && letters[pos] == 'N';
      if (grounded) ++pos;
      kind = grounded ? TransformerConnectionKind::GroundedWye
                      : TransformerConnectionKind::Wye;
      return;
    }
    kind = TransformerConnectionKind::Unsupported;
  };

  parse_side(parsed.hv, parsed.hv_grounded);
  parse_side(parsed.lv, parsed.lv_grounded);
  return parsed;
}

bool generator_has_per_phase_dispatch(const ThreePhaseGenerator& g) {
  return std::abs(g.p_a_mw) > 1e-12 || std::abs(g.q_a_mvar) > 1e-12 ||
         std::abs(g.p_b_mw) > 1e-12 || std::abs(g.q_b_mvar) > 1e-12 ||
         std::abs(g.p_c_mw) > 1e-12 || std::abs(g.q_c_mvar) > 1e-12;
}

bool generator_has_aggregate_dispatch(const ThreePhaseGenerator& g) {
  return std::abs(g.p_mw) > kPhaseValueTol ||
         std::abs(g.q_mvar) > kPhaseValueTol;
}

void validate_generator_dispatch_semantics(const ThreePhaseGenerator& g) {
  require_mask_not_empty("generator", g.index, g.phase_mask);
  require_phase_values_on_mask(
      "generator",
      g.index,
      g.phase_mask,
      {g.p_a_mw, g.p_b_mw, g.p_c_mw},
      {g.q_a_mvar, g.q_b_mvar, g.q_c_mvar});

  if (generator_has_aggregate_dispatch(g) && generator_has_per_phase_dispatch(g)) {
    throw std::runtime_error(
        "generator " + std::to_string(g.index) +
        " mixes aggregate and per-phase dispatch fields");
  }
}

Eigen::Matrix3cd build_sequence_zabc(
    Complex z0,
    Complex z1,
    Complex z2) {
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = std::polar(1.0, 4.0 * kPi / 3.0);

  Eigen::Matrix3cd transform;
  transform << Complex(1.0, 0.0), Complex(1.0, 0.0), Complex(1.0, 0.0),
      Complex(1.0, 0.0), a2, a,
      Complex(1.0, 0.0), a, a2;

  Eigen::Matrix3cd z_seq = Eigen::Matrix3cd::Zero();
  z_seq(0, 0) = z0;
  z_seq(1, 1) = z1;
  z_seq(2, 2) = z2;
  return transform * z_seq * transform.inverse();
}

Eigen::Matrix3cd build_line_zabc(const ThreePhaseACLine& line) {
  const Complex z1(line.r1_pu, line.x1_pu);
  Complex z0(line.r0_pu, line.x0_pu);
  if (std::abs(z0) < 1e-20) z0 = z1;
  return build_sequence_zabc(z0, z1, z1);
}

Eigen::Matrix3cd build_phase_matrix(
    const PhaseValueMatrix3& real_matrix,
    const PhaseValueMatrix3& imag_matrix) {
  Eigen::Matrix3cd matrix = Eigen::Matrix3cd::Zero();
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      matrix(row, col) = Complex(
          phase_matrix_get(real_matrix, row, col),
          phase_matrix_get(imag_matrix, row, col));
    }
  }
  return matrix;
}

void validate_line_phase_matrix_against_mask(const ThreePhaseACLine& line) {
  if (!line.use_phase_matrix) return;

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (line.phase_mask.has(row) && line.phase_mask.has(col)) continue;
      const double r_value = phase_matrix_get(line.r_matrix_pu, row, col);
      const double x_value = phase_matrix_get(line.x_matrix_pu, row, col);
      const double b_value = phase_matrix_get(line.b_matrix_pu, row, col);
      if (std::abs(r_value) > kPhaseValueTol ||
          std::abs(x_value) > kPhaseValueTol ||
          std::abs(b_value) > kPhaseValueTol) {
        throw std::runtime_error(
            "line " + std::to_string(line.index) +
            " full phase-domain matrix carries non-zero entries outside phase_mask");
      }
    }
  }
}

Eigen::Matrix3cd build_line_series_matrix(const ThreePhaseACLine& line) {
  if (line.use_phase_matrix) {
    validate_line_phase_matrix_against_mask(line);
    return build_phase_matrix(line.r_matrix_pu, line.x_matrix_pu);
  }
  return build_line_zabc(line);
}

Eigen::Matrix3cd build_line_shunt_half_matrix(const ThreePhaseACLine& line) {
  if (line.use_phase_matrix) {
    validate_line_phase_matrix_against_mask(line);
    Eigen::Matrix3cd y_shunt = Eigen::Matrix3cd::Zero();
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        y_shunt(row, col) =
            Complex(0.0, phase_matrix_get(line.b_matrix_pu, row, col) / 2.0);
      }
    }
    return y_shunt;
  }

  Eigen::Matrix3cd y_shunt = Eigen::Matrix3cd::Zero();
  for (int phase = 0; phase < 3; ++phase) {
    y_shunt(phase, phase) = Complex(0.0, line.b1_pu / 2.0);
  }
  return y_shunt;
}

ComplexMatrixX select_phase_submatrix(
    const Eigen::Matrix3cd& full_matrix,
    const std::vector<int>& phases) {
  ComplexMatrixX sub(phases.size(), phases.size());
  for (size_t i = 0; i < phases.size(); ++i) {
    for (size_t j = 0; j < phases.size(); ++j) {
      sub(static_cast<int>(i), static_cast<int>(j)) =
          full_matrix(phases[i], phases[j]);
    }
  }
  return sub;
}

ComplexVectorX select_phase_subvector(
    const std::array<Complex, 3>& full_vector,
    const std::vector<int>& phases) {
  ComplexVectorX sub(phases.size());
  for (size_t i = 0; i < phases.size(); ++i) {
    sub(static_cast<int>(i)) = full_vector[phases[i]];
  }
  return sub;
}

ComplexMatrixX identity_matrix(int size) {
  return ComplexMatrixX::Identity(size, size);
}

double stable_real_scale(double value) {
  if (std::abs(value) > 1e-12) {
    return value;
  }
  return (value < 0.0) ? -1e-12 : 1e-12;
}

Complex stable_complex_scale(const Complex& value) {
  if (std::abs(value) > 1e-12) {
    return value;
  }
  return Complex(1e-12, 0.0);
}

ComplexMatrixX inverse_or_throw(
    const ComplexMatrixX& matrix,
    const std::string& label) {
  Eigen::FullPivLU<ComplexMatrixX> lu(matrix);
  if (!lu.isInvertible()) {
    throw std::runtime_error(label + " is singular");
  }
  return lu.inverse();
}

ComplexMatrixX pseudoinverse(
    const ComplexMatrixX& matrix,
    const std::string& label) {
  if (matrix.size() == 0) {
    return ComplexMatrixX::Zero(matrix.cols(), matrix.rows());
  }

  Eigen::JacobiSVD<ComplexMatrixX> svd(
      matrix,
      Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& singular_values = svd.singularValues();
  const double max_sv =
      singular_values.size() > 0 ? singular_values.maxCoeff() : 0.0;
  if (max_sv <= kPhaseValueTol) {
    throw std::runtime_error(label + " is rank deficient");
  }

  const double threshold =
      std::max(matrix.rows(), matrix.cols()) * max_sv * 1e-12;
  ComplexMatrixX sigma_pinv = ComplexMatrixX::Zero(
      svd.matrixV().cols(),
      svd.matrixU().cols());
  for (int idx = 0; idx < singular_values.size(); ++idx) {
    if (singular_values(idx) > threshold) {
      sigma_pinv(idx, idx) = Complex(1.0 / singular_values(idx), 0.0);
    }
  }
  return svd.matrixV() * sigma_pinv * svd.matrixU().adjoint();
}

[[maybe_unused]]
ComplexMatrixX projector_from_basis(
    const ComplexMatrixX& basis,
    const std::string& label) {
  if (basis.cols() == 0) {
    return ComplexMatrixX::Zero(basis.rows(), basis.rows());
  }
  const ComplexMatrixX gram = basis.adjoint() * basis;
  return basis * inverse_or_throw(gram, label + " gram") * basis.adjoint();
}

PhaseTerminal collect_terminal_nodes(
    const PhaseNodeIndexer& indexer,
    int bus_offset,
    PhaseMask mask,
    const std::string& entity,
    int entity_index,
    const std::string& terminal_label) {
  require_mask_not_empty(entity, entity_index, mask);
  if (!indexer.bus_phase_mask(bus_offset).contains(mask)) {
    throw std::runtime_error(
        entity + " " + std::to_string(entity_index) + " " + terminal_label +
        " mask is not a subset of the endpoint bus phase_mask");
  }

  PhaseTerminal terminal;
  terminal.bus_offset = bus_offset;
  terminal.mask = mask;
  terminal.phases = active_phase_indices(mask);
  terminal.nodes.reserve(terminal.phases.size());
  for (int phase : terminal.phases) {
    terminal.nodes.push_back(indexer.node_index(bus_offset, phase));
  }
  return terminal;
}

void add_dense_block(
    std::vector<ComplexTriplet>& triplets,
    const std::vector<int>& row_nodes,
    const std::vector<int>& col_nodes,
    const ComplexMatrixX& block,
    const std::string& label) {
  if (block.rows() != static_cast<int>(row_nodes.size()) ||
      block.cols() != static_cast<int>(col_nodes.size())) {
    throw std::runtime_error(label + " dimensions do not match compact terminals");
  }

  for (int row = 0; row < block.rows(); ++row) {
    for (int col = 0; col < block.cols(); ++col) {
      if (std::abs(block(row, col)) <= kPhaseValueTol) continue;
      triplets.emplace_back(row_nodes[static_cast<size_t>(row)],
                            col_nodes[static_cast<size_t>(col)],
                            block(row, col));
    }
  }
}

void stamp_primitive_branch(
    std::vector<ComplexTriplet>& triplets,
    const PhaseTerminal& from_terminal,
    const PhaseTerminal& to_terminal,
    const ComplexMatrixX& y_ff,
    const ComplexMatrixX& y_ft,
    const ComplexMatrixX& y_tf,
    const ComplexMatrixX& y_tt,
    const std::string& label) {
  add_dense_block(triplets, from_terminal.nodes, from_terminal.nodes, y_ff, label + ".y_ff");
  add_dense_block(triplets, from_terminal.nodes, to_terminal.nodes, y_ft, label + ".y_ft");
  add_dense_block(triplets, to_terminal.nodes, from_terminal.nodes, y_tf, label + ".y_tf");
  add_dense_block(triplets, to_terminal.nodes, to_terminal.nodes, y_tt, label + ".y_tt");
}

void stamp_terminal_shunt(
    std::vector<ComplexTriplet>& triplets,
    const PhaseTerminal& terminal,
    const ComplexMatrixX& y_shunt,
    const std::string& label) {
  add_dense_block(triplets, terminal.nodes, terminal.nodes, y_shunt, label);
}

void add_fixed_current(
    ComplexVectorX& fixed_current,
    const PhaseTerminal& terminal,
    const ComplexVectorX& current,
    const std::string& label) {
  if (current.rows() != static_cast<int>(terminal.nodes.size())) {
    throw std::runtime_error(label + " dimensions do not match compact terminal");
  }
  for (int i = 0; i < current.rows(); ++i) {
    fixed_current[terminal.nodes[static_cast<size_t>(i)]] += current(i);
  }
}

ComplexVectorX collect_terminal_phasor(
    const PhaseTerminal& terminal,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va) {
  ComplexVectorX voltage(static_cast<int>(terminal.nodes.size()));
  for (int i = 0; i < static_cast<int>(terminal.nodes.size()); ++i) {
    const int node = terminal.nodes[static_cast<size_t>(i)];
    voltage(i) = std::polar(vm[node], va[node]);
  }
  return voltage;
}

Complex default_phase_voltage_guess_pu(int phase_index) {
  if (phase_index == 0) return {1.0, 0.0};
  if (phase_index == 1) return std::polar(1.0, -2.0 * kPi / 3.0);
  return std::polar(1.0, 2.0 * kPi / 3.0);
}

double wrap_angle_rad(double angle_rad) {
  while (angle_rad > kPi) angle_rad -= 2.0 * kPi;
  while (angle_rad < -kPi) angle_rad += 2.0 * kPi;
  return angle_rad;
}

double bus_phase_vm_value(const ThreePhaseACBus& bus, int phase_index) {
  if (phase_index == 0) return bus.vm_a_pu;
  if (phase_index == 1) return bus.vm_b_pu;
  return bus.vm_c_pu;
}

double bus_phase_va_rad_value(const ThreePhaseACBus& bus, int phase_index) {
  if (phase_index == 0) return bus.va_a_deg * kPi / 180.0;
  if (phase_index == 1) return bus.va_b_deg * kPi / 180.0;
  return bus.va_c_deg * kPi / 180.0;
}

Complex bus_phase_voltage_guess_pu(
    const ThreePhaseACBus& bus,
    int phase_index) {
  return std::polar(
      bus_phase_vm_value(bus, phase_index),
      bus_phase_va_rad_value(bus, phase_index));
}

bool bus_phase_has_explicit_voltage_guess(
    const ThreePhaseACBus& bus,
    int phase_index) {
  const Complex specified = bus_phase_voltage_guess_pu(bus, phase_index);
  const Complex nominal = default_phase_voltage_guess_pu(phase_index);
  return std::abs(std::abs(specified) - std::abs(nominal)) > 1e-9 ||
         std::abs(wrap_angle_rad(
             std::arg(specified) - std::arg(nominal))) > 1e-9;
}

bool terminal_nodes_seeded(
    const PhaseTerminal& terminal,
    const std::vector<bool>& seeded_nodes) {
  for (const int node : terminal.nodes) {
    if (node < 0 || node >= static_cast<int>(seeded_nodes.size()) ||
        !seeded_nodes[static_cast<std::size_t>(node)]) {
      return false;
    }
  }
  return true;
}

void seed_terminal_voltage(
    const PhaseTerminal& terminal,
    const ComplexVectorX& voltage,
    Eigen::VectorXd& vm,
    Eigen::VectorXd& va,
    std::vector<bool>& seeded_nodes) {
  if (voltage.rows() != static_cast<int>(terminal.nodes.size())) {
    throw std::runtime_error("seed_terminal_voltage dimensions do not match compact terminal");
  }
  for (int i = 0; i < voltage.rows(); ++i) {
    const int node = terminal.nodes[static_cast<std::size_t>(i)];
    vm[node] = std::abs(voltage(i));
    va[node] = std::arg(voltage(i));
    seeded_nodes[static_cast<std::size_t>(node)] = true;
  }
}

bool try_seed_transformer_terminal(
    const PrimitiveBranchModel& primitive,
    bool seed_to_side,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    Eigen::VectorXd& vm_mutable,
    Eigen::VectorXd& va_mutable,
    std::vector<bool>& seeded_nodes) {
  const PhaseTerminal& known_terminal =
      seed_to_side ? primitive.from_terminal : primitive.to_terminal;
  const PhaseTerminal& target_terminal =
      seed_to_side ? primitive.to_terminal : primitive.from_terminal;
  const ComplexMatrixX& self_y = seed_to_side ? primitive.y_tt : primitive.y_ff;
  const ComplexMatrixX& cross_y = seed_to_side ? primitive.y_tf : primitive.y_ft;

  if (!terminal_nodes_seeded(known_terminal, seeded_nodes) ||
      terminal_nodes_seeded(target_terminal, seeded_nodes)) {
    return false;
  }

  Eigen::FullPivLU<ComplexMatrixX> lu(self_y);
  if (!lu.isInvertible()) {
    return false;
  }

  const ComplexVectorX known_voltage = collect_terminal_phasor(known_terminal, vm, va);
  const ComplexVectorX target_voltage = lu.solve(-cross_y * known_voltage);
  if (!target_voltage.allFinite()) {
    return false;
  }
  seed_terminal_voltage(target_terminal, target_voltage, vm_mutable, va_mutable, seeded_nodes);
  return true;
}

void seed_transformer_voltage_guesses(
    const std::vector<TransformerPrimitiveRecord>& transformer_primitives,
    const ThreePhaseACSystem& sys,
    Eigen::VectorXd& vm,
    Eigen::VectorXd& va) {
  std::vector<bool> seeded_nodes(static_cast<std::size_t>(vm.size()), false);
  for (const auto& primitive_record : transformer_primitives) {
    const auto& from_bus =
        sys.buses[static_cast<std::size_t>(primitive_record.primitive.from_terminal.bus_offset)];
    for (int local = 0;
         local < static_cast<int>(primitive_record.primitive.from_terminal.nodes.size());
         ++local) {
      const int node =
          primitive_record.primitive.from_terminal.nodes[static_cast<std::size_t>(local)];
      const int phase =
          primitive_record.primitive.from_terminal.phases[static_cast<std::size_t>(local)];
      if (from_bus.bus_type == BusType::SLACK ||
          bus_phase_has_explicit_voltage_guess(from_bus, phase)) {
        seeded_nodes[static_cast<std::size_t>(node)] = true;
      }
    }
    const auto& to_bus =
        sys.buses[static_cast<std::size_t>(primitive_record.primitive.to_terminal.bus_offset)];
    for (int local = 0;
         local < static_cast<int>(primitive_record.primitive.to_terminal.nodes.size());
         ++local) {
      const int node =
          primitive_record.primitive.to_terminal.nodes[static_cast<std::size_t>(local)];
      const int phase =
          primitive_record.primitive.to_terminal.phases[static_cast<std::size_t>(local)];
      if (to_bus.bus_type == BusType::SLACK ||
          bus_phase_has_explicit_voltage_guess(to_bus, phase)) {
        seeded_nodes[static_cast<std::size_t>(node)] = true;
      }
    }
  }

  bool progress = true;
  while (progress) {
    progress = false;
    for (const auto& primitive_record : transformer_primitives) {
      progress = try_seed_transformer_terminal(
                     primitive_record.primitive,
                     true,
                     vm,
                     va,
                     vm,
                     va,
                     seeded_nodes) ||
                 progress;
      progress = try_seed_transformer_terminal(
                     primitive_record.primitive,
                     false,
                     vm,
                     va,
                     vm,
                     va,
                     seeded_nodes) ||
                 progress;
    }
  }
}

void seed_source_voltage_guesses(
    const std::vector<ThreePhaseExternalGrid>& sources,
    const std::unordered_map<int, int>& id_map,
    const PhaseNodeIndexer& indexer,
    const ThreePhaseACSystem& sys,
    Eigen::VectorXd& vm,
    Eigen::VectorXd& va) {
  std::vector<bool> seeded_nodes(static_cast<std::size_t>(vm.size()), false);
  for (size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    const auto& bus = sys.buses[bus_offset];
    const PhaseMask mask = bus.phase_mask;
    for (int phase = 0; phase < 3; ++phase) {
      if (!mask.has(phase)) continue;
      if (bus.bus_type != BusType::SLACK &&
          !bus_phase_has_explicit_voltage_guess(bus, phase)) {
        continue;
      }
      const int node = indexer.node_index(static_cast<int>(bus_offset), phase);
      seeded_nodes[static_cast<std::size_t>(node)] = true;
    }
  }

  for (const auto& source : sources) {
    if (!source.in_service) continue;
    auto it_bus = id_map.find(source.bus);
    if (it_bus == id_map.end()) continue;
    if (sys.buses[static_cast<std::size_t>(it_bus->second)].bus_type == BusType::SLACK) {
      continue;
    }

    const PhaseTerminal terminal = collect_terminal_nodes(
        indexer,
        it_bus->second,
        source.phase_mask,
        "external_grid",
        source.index,
        "terminal");
    const SourceNortonPrimitive primitive = build_source_norton_primitive(
        source,
        terminal,
        sys.base_mva);

    for (int local = 0; local < primitive.open_circuit_phase_voltage.rows(); ++local) {
      const int node = terminal.nodes[static_cast<std::size_t>(local)];
      if (seeded_nodes[static_cast<std::size_t>(node)]) continue;
      vm[node] = std::abs(primitive.open_circuit_phase_voltage(local));
      va[node] = std::arg(primitive.open_circuit_phase_voltage(local));
      seeded_nodes[static_cast<std::size_t>(node)] = true;
    }
  }
}

PrimitiveBranchObservation observe_primitive_branch(
    const PrimitiveBranchModel& primitive,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va) {
  PrimitiveBranchObservation observation;
  observation.from_voltage = collect_terminal_phasor(primitive.from_terminal, vm, va);
  observation.to_voltage = collect_terminal_phasor(primitive.to_terminal, vm, va);
  observation.from_current =
      primitive.y_ff * observation.from_voltage +
      primitive.y_ft * observation.to_voltage;
  observation.to_current =
      primitive.y_tf * observation.from_voltage +
      primitive.y_tt * observation.to_voltage;
  return observation;
}

PrimitiveBranchModel build_line_primitive(
    const PhaseNodeIndexer& indexer,
    int from_bus_offset,
    int to_bus_offset,
    const ThreePhaseACLine& line) {
  PrimitiveBranchModel primitive;
  primitive.from_terminal = collect_terminal_nodes(
      indexer,
      from_bus_offset,
      line.phase_mask,
      "line",
      line.index,
      "from");
  primitive.to_terminal = collect_terminal_nodes(
      indexer,
      to_bus_offset,
      line.phase_mask,
      "line",
      line.index,
      "to");

  if (!line.use_phase_matrix) {
    const Complex z1(line.r1_pu, line.x1_pu);
    if (std::abs(z1) < 1e-20) return primitive;
  }

  const ComplexMatrixX z_sub = select_phase_submatrix(
      build_line_series_matrix(line),
      primitive.from_terminal.phases);
  const ComplexMatrixX y_series = inverse_or_throw(
      z_sub,
      "line " + std::to_string(line.index) + " phase submatrix");
  const ComplexMatrixX y_shunt_half = select_phase_submatrix(
      build_line_shunt_half_matrix(line),
      primitive.from_terminal.phases);

  primitive.y_ff = y_series + y_shunt_half;
  primitive.y_tt = y_series + y_shunt_half;
  primitive.y_ft = -y_series;
  primitive.y_tf = -y_series;
  return primitive;
}

Complex transformer_impedance_from_vk_vkr(
    double vk_percent,
    double vkr_percent,
    double base_mva,
    double sn_mva) {
  if (sn_mva <= kPhaseValueTol || base_mva <= kPhaseValueTol) {
    throw std::runtime_error("transformer series impedance requires positive sn_mva/base_mva");
  }
  const double scale = base_mva / sn_mva;
  const double z = std::max(0.0, vk_percent / 100.0) * scale;
  const double r = std::max(0.0, vkr_percent / 100.0) * scale;
  const double x_sq = std::max(0.0, z * z - r * r);
  return Complex(r, std::sqrt(x_sq));
}

Complex transformer_magnetizing_admittance(
    const ThreePhaseTransformer& transformer,
    double base_mva) {
  if (std::abs(transformer.pfe_kw) <= kPhaseValueTol &&
      std::abs(transformer.i0_percent) <= kPhaseValueTol) {
    return Complex(0.0, 0.0);
  }
  if (transformer.sn_mva <= kPhaseValueTol || base_mva <= kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " magnetizing branch requires positive sn_mva/base_mva");
  }

  const double g_tr = std::max(0.0, transformer.pfe_kw / 1000.0 / transformer.sn_mva);
  const double y_mag_tr = std::max(0.0, transformer.i0_percent / 100.0);
  const double b_tr = std::sqrt(std::max(0.0, y_mag_tr * y_mag_tr - g_tr * g_tr));
  return Complex(g_tr, -b_tr) * (transformer.sn_mva / base_mva);
}

bool transformer_has_zero_sequence_magnetizing_branch(
    const ThreePhaseTransformer& transformer) {
  return std::abs(transformer.mag0_percent) > kPhaseValueTol ||
         std::abs(transformer.mag0_rx) > kPhaseValueTol;
}

Complex transformer_zero_sequence_impedance(
    const ThreePhaseTransformer& transformer,
    double base_mva) {
  const double vk0_percent =
      std::abs(transformer.vk0_percent) > kPhaseValueTol
          ? transformer.vk0_percent
          : transformer.vk_percent;
  const double vkr0_percent =
      std::abs(transformer.vkr0_percent) > kPhaseValueTol
          ? transformer.vkr0_percent
          : transformer.vkr_percent;
  return transformer_impedance_from_vk_vkr(
      vk0_percent,
      vkr0_percent,
      base_mva,
      transformer.sn_mva);
}

ComplexMatrixX common_mode_projector(
    const PhaseTerminal& terminal,
    const std::string& label) {
  if (terminal.nodes.empty()) {
    throw std::runtime_error(label + " requires at least one active phase");
  }
  return ComplexMatrixX::Constant(
      static_cast<int>(terminal.nodes.size()),
      static_cast<int>(terminal.nodes.size()),
      Complex(1.0 / static_cast<double>(terminal.nodes.size()), 0.0));
}

ZeroSequencePortAdmittance build_transformer_zero_sequence_port_admittance(
    const ThreePhaseTransformer& transformer,
    double base_mva,
    double impedance_scale) {
  if (transformer.mag0_percent < -kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " requires non-negative mag0_percent");
  }
  if (std::abs(transformer.mag0_rx) > kPhaseValueTol &&
      std::abs(transformer.mag0_percent) <= kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " mag0_rx requires positive mag0_percent");
  }
  if (transformer.si0_hv_partial < -kPhaseValueTol ||
      transformer.si0_hv_partial > 1.0 + kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " si0_hv_partial must lie within [0, 1]");
  }

  const Complex z0 =
      transformer_zero_sequence_impedance(transformer, base_mva) * impedance_scale;
  if (std::abs(z0) <= kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " has zero zero-sequence leakage impedance");
  }

  if (!transformer_has_zero_sequence_magnetizing_branch(transformer)) {
    const Complex y0 = Complex(1.0, 0.0) / z0;
    return {
        .y_ff = y0,
        .y_ft = -y0,
        .y_tf = -y0,
        .y_tt = y0,
    };
  }

  const double mag0_abs = std::abs(z0) * transformer.mag0_percent;
  const double x0_mag = mag0_abs / std::sqrt(transformer.mag0_rx * transformer.mag0_rx + 1.0);
  const double r0_mag = x0_mag * transformer.mag0_rx;
  const Complex z_hv = transformer.si0_hv_partial * z0;
  const Complex z_lv = (1.0 - transformer.si0_hv_partial) * z0;
  const Complex z_mag(r0_mag, x0_mag);
  const Complex denom = z_hv * z_lv + z_hv * z_mag + z_lv * z_mag;
  if (std::abs(denom) <= kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " zero-sequence magnetizing T-model is singular");
  }

  return {
      .y_ff = (z_lv + z_mag) / denom,
      .y_ft = -z_mag / denom,
      .y_tf = -z_mag / denom,
      .y_tt = (z_hv + z_mag) / denom,
  };
}

PrimitiveBranchModel build_grounded_wye_grounded_wye_primitive(
    const PrimitiveBranchModel& primitive_template,
    const Complex& y_positive_negative,
    const ZeroSequencePortAdmittance& zero_sequence,
    const Complex& tap,
    const Complex& y_mag) {
  PrimitiveBranchModel primitive = primitive_template;
  const ComplexMatrixX p0 =
      common_mode_projector(primitive.from_terminal, "grounded-wye/wye zero-sequence model");
  const ComplexMatrixX p12 = identity_matrix(p0.rows()) - p0;

  const ComplexMatrixX y_ff_winding =
      y_positive_negative * p12 + zero_sequence.y_ff * p0;
  const ComplexMatrixX y_ft_winding =
      -y_positive_negative * p12 + zero_sequence.y_ft * p0;
  const ComplexMatrixX y_tf_winding =
      -y_positive_negative * p12 + zero_sequence.y_tf * p0;
  const ComplexMatrixX y_tt_winding =
      y_positive_negative * p12 + zero_sequence.y_tt * p0;

  const ComplexMatrixX k_hv =
      identity_matrix(p0.rows()) / stable_complex_scale(tap);
  const ComplexMatrixX k_lv = identity_matrix(p0.rows());

  primitive.y_ff = k_hv.adjoint() * y_ff_winding * k_hv;
  primitive.y_ft = k_hv.adjoint() * y_ft_winding * k_lv;
  primitive.y_tf = k_lv.adjoint() * y_tf_winding * k_hv;
  primitive.y_tt = k_lv.adjoint() * y_tt_winding * k_lv;
  if (std::abs(y_mag) > kPhaseValueTol) {
    primitive.y_ff +=
        k_hv.adjoint() * (y_mag * identity_matrix(3)) * k_hv;
  }
  return primitive;
}

Complex eliminate_open_terminal_from_port_admittance(
    const ZeroSequencePortAdmittance& port,
    bool retain_from_side) {
  const Complex y_self = retain_from_side ? port.y_ff : port.y_tt;
  const Complex y_couple_out = retain_from_side ? port.y_ft : port.y_tf;
  const Complex y_couple_back = retain_from_side ? port.y_tf : port.y_ft;
  const Complex y_open = retain_from_side ? port.y_tt : port.y_ff;
  if (std::abs(y_open) <= kPhaseValueTol) {
    return Complex(0.0, 0.0);
  }
  return y_self - (y_couple_out * y_couple_back) / y_open;
}

void add_common_mode_shunt_to_terminal(
    PrimitiveBranchModel& primitive,
    bool on_from_side,
    double terminal_scale_real,
    const Complex& y_shunt,
    const std::string& label) {
  if (std::abs(y_shunt) <= kPhaseValueTol) {
    return;
  }

  const PhaseTerminal& terminal =
      on_from_side ? primitive.from_terminal : primitive.to_terminal;
  const double scale = stable_real_scale(terminal_scale_real);
  const ComplexMatrixX p0 = common_mode_projector(terminal, label);
  const ComplexMatrixX y_block =
      (y_shunt / Complex(scale * scale, 0.0)) * p0;
  if (on_from_side) {
    primitive.y_ff += y_block;
  } else {
    primitive.y_tt += y_block;
  }
}

void add_transformer_zero_sequence_shunts_with_blocked_ports(
    PrimitiveBranchModel& primitive,
    const ThreePhaseTransformer& transformer,
    double base_mva,
    double impedance_scale,
    bool from_port_conducting,
    double from_terminal_scale_real,
    bool to_port_conducting,
    double to_terminal_scale_real,
    const std::string& path_label) {
  if (!transformer_has_zero_sequence_magnetizing_branch(transformer)) {
    return;
  }

  const ZeroSequencePortAdmittance zero_sequence =
      build_transformer_zero_sequence_port_admittance(
          transformer,
          base_mva,
          impedance_scale);
  if (from_port_conducting) {
    add_common_mode_shunt_to_terminal(
        primitive,
        true,
        from_terminal_scale_real,
        eliminate_open_terminal_from_port_admittance(zero_sequence, true),
        path_label + " zero-sequence model");
  }
  if (to_port_conducting) {
    add_common_mode_shunt_to_terminal(
        primitive,
        false,
        to_terminal_scale_real,
        eliminate_open_terminal_from_port_admittance(zero_sequence, false),
        path_label + " zero-sequence model");
  }
}

[[maybe_unused]]
double nominal_transformer_ratio_pu(
    const ThreePhaseTransformer& transformer,
    const ThreePhaseACBus& hv_bus,
    const ThreePhaseACBus& lv_bus) {
  if (transformer.vn_hv_kv <= kPhaseValueTol ||
      transformer.vn_lv_kv <= kPhaseValueTol ||
      hv_bus.base_kv <= kPhaseValueTol ||
      lv_bus.base_kv <= kPhaseValueTol) {
    return 1.0;
  }
  return (transformer.vn_hv_kv / hv_bus.base_kv) /
         (transformer.vn_lv_kv / lv_bus.base_kv);
}

ComplexMatrixX delta_connection_matrix(bool positive_thirty_deg) {
  ComplexMatrixX matrix = ComplexMatrixX::Zero(3, 3);
  if (positive_thirty_deg) {
    matrix(0, 0) = Complex(1.0, 0.0);
    matrix(0, 1) = Complex(-1.0, 0.0);
    matrix(1, 1) = Complex(1.0, 0.0);
    matrix(1, 2) = Complex(-1.0, 0.0);
    matrix(2, 2) = Complex(1.0, 0.0);
    matrix(2, 0) = Complex(-1.0, 0.0);
    return matrix;
  }

  matrix(0, 0) = Complex(1.0, 0.0);
  matrix(0, 2) = Complex(-1.0, 0.0);
  matrix(1, 1) = Complex(1.0, 0.0);
  matrix(1, 0) = Complex(-1.0, 0.0);
  matrix(2, 2) = Complex(1.0, 0.0);
  matrix(2, 1) = Complex(-1.0, 0.0);
  return matrix;
}

Complex delta_antifloat_conductance(const Complex& y_winding_scalar) {
  return Complex(std::abs(y_winding_scalar) * 1e-6, 0.0);
}

ComplexMatrixX delta_antifloat_shunt(
    const TransformerConnectionEmbedding& embedding,
    const Complex& y_winding_scalar) {
  return delta_antifloat_conductance(y_winding_scalar) *
         ComplexMatrixX::Identity(
             embedding.matrix.cols(),
             embedding.matrix.cols());
}

TransformerConnectionEmbedding build_wye_connection_embedding(
    const PhaseTerminal& terminal) {
  TransformerConnectionEmbedding embedding;
  embedding.matrix = ComplexMatrixX::Identity(
      static_cast<int>(terminal.phases.size()),
      static_cast<int>(terminal.phases.size()));
  embedding.winding_labels = terminal.phases;
  return embedding;
}

TransformerConnectionEmbedding build_delta_connection_embedding(
    const PhaseTerminal& terminal,
    bool positive_thirty_deg) {
  if (terminal.phases.size() < 2) {
    throw std::runtime_error("delta terminal requires at least two active phases");
  }

  const ComplexMatrixX full = delta_connection_matrix(positive_thirty_deg);
  std::vector<int> kept_rows;
  kept_rows.reserve(3);
  for (int row = 0; row < full.rows(); ++row) {
    bool uses_missing_phase = false;
    bool uses_any_terminal = false;
    for (int phase = 0; phase < 3; ++phase) {
      if (std::abs(full(row, phase)) <= kPhaseValueTol) continue;
      uses_any_terminal = true;
      if (!terminal.mask.has(phase)) {
        uses_missing_phase = true;
        break;
      }
    }
    if (uses_any_terminal && !uses_missing_phase) {
      kept_rows.push_back(row);
    }
  }

  if (kept_rows.empty()) {
    throw std::runtime_error("delta terminal winding set is empty for the given phase_mask");
  }

  TransformerConnectionEmbedding embedding;
  embedding.matrix = ComplexMatrixX::Zero(
      static_cast<int>(kept_rows.size()),
      static_cast<int>(terminal.phases.size()));
  embedding.winding_labels = kept_rows;
  for (int row_local = 0; row_local < static_cast<int>(kept_rows.size()); ++row_local) {
    const int row = kept_rows[static_cast<std::size_t>(row_local)];
    for (int col_local = 0; col_local < static_cast<int>(terminal.phases.size()); ++col_local) {
      const int phase = terminal.phases[static_cast<std::size_t>(col_local)];
      embedding.matrix(row_local, col_local) = full(row, phase);
    }
  }
  return embedding;
}

std::vector<std::string> split_winding_topology_tokens(
    const std::string& raw_topology,
    bool split_compact_phase_string) {
  std::vector<std::string> tokens;
  std::string current;
  auto flush = [&]() {
    if (current.empty()) return;
    tokens.push_back(uppercase_ascii(current));
    current.clear();
  };

  for (char ch : raw_topology) {
    const unsigned char raw = static_cast<unsigned char>(ch);
    if (std::isalpha(raw)) {
      current.push_back(static_cast<char>(std::toupper(raw)));
      continue;
    }
    flush();
  }
  flush();

  if (split_compact_phase_string &&
      tokens.size() == 1 &&
      tokens.front().size() > 1) {
    const std::string compact = tokens.front();
    const bool phase_letters_only = std::all_of(
        compact.begin(),
        compact.end(),
        [](unsigned char ch) {
          return ch == 'A' || ch == 'B' || ch == 'C';
        });
    if (phase_letters_only) {
      tokens.clear();
      for (char ch : compact) {
        tokens.emplace_back(1, ch);
      }
    }
  }

  return tokens;
}

int wye_winding_slot_from_token(
    const ThreePhaseTransformer& transformer,
    const std::string& side_label,
    const std::string& token) {
  if (token == "A") return 0;
  if (token == "B") return 1;
  if (token == "C") return 2;
  throw std::runtime_error(
      "transformer " + std::to_string(transformer.index) + " " + side_label +
      "_winding_topology contains unsupported wye token " + token);
}

int delta_winding_slot_from_token(
    const ThreePhaseTransformer& transformer,
    const std::string& side_label,
    const std::string& token,
    bool positive_thirty_deg) {
  int canonical_slot = -1;
  if (token == "AB" || token == "BA") canonical_slot = 0;
  if (token == "BC" || token == "CB") canonical_slot = 1;
  if (token == "CA" || token == "AC") canonical_slot = 2;
  if (canonical_slot < 0) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) + " " + side_label +
        "_winding_topology contains unsupported delta token " + token);
  }
  if (positive_thirty_deg) return canonical_slot;
  if (canonical_slot == 0) return 1;
  if (canonical_slot == 1) return 2;
  return 0;
}

std::vector<int> parse_transformer_winding_topology_slots(
    const ThreePhaseTransformer& transformer,
    const std::string& side_label,
    TransformerConnectionKind kind,
    bool positive_thirty_deg,
    const std::string& raw_topology) {
  if (raw_topology.empty()) return {};

  const bool wye_like = is_wye_like_connection(kind);
  const std::vector<std::string> tokens = split_winding_topology_tokens(
      raw_topology,
      wye_like);
  if (tokens.empty()) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) + " " + side_label +
        "_winding_topology is empty after parsing");
  }

  std::vector<int> slots;
  slots.reserve(tokens.size());
  for (const std::string& token : tokens) {
    const int slot = wye_like
                         ? wye_winding_slot_from_token(
                               transformer,
                               side_label,
                               token)
                         : delta_winding_slot_from_token(
                               transformer,
                               side_label,
                               token,
                               positive_thirty_deg);
    if (std::find(slots.begin(), slots.end(), slot) != slots.end()) {
      throw std::runtime_error(
          "transformer " + std::to_string(transformer.index) + " " + side_label +
          "_winding_topology repeats token " + token);
    }
    slots.push_back(slot);
  }
  return slots;
}

TransformerConnectionEmbedding build_transformer_connection_embedding(
    const ThreePhaseTransformer& transformer,
    const PhaseTerminal& terminal,
    TransformerConnectionKind kind,
    bool positive_thirty_deg,
    const std::string& raw_topology,
    const std::string& side_label) {
  TransformerConnectionEmbedding inferred;
  if (is_wye_like_connection(kind)) {
    inferred = build_wye_connection_embedding(terminal);
  } else if (is_delta_connection(kind)) {
    inferred = build_delta_connection_embedding(terminal, positive_thirty_deg);
  } else {
    throw std::runtime_error("unsupported transformer connection kind");
  }

  if (raw_topology.empty()) {
    return inferred;
  }

  const std::vector<int> requested = parse_transformer_winding_topology_slots(
      transformer,
      side_label,
      kind,
      positive_thirty_deg,
      raw_topology);
  TransformerConnectionEmbedding filtered;
  filtered.matrix = ComplexMatrixX::Zero(
      static_cast<int>(requested.size()),
      inferred.matrix.cols());
  filtered.winding_labels = requested;
  filtered.explicit_topology = true;

  for (int row_local = 0; row_local < static_cast<int>(requested.size()); ++row_local) {
    const int slot = requested[static_cast<std::size_t>(row_local)];
    const auto it = std::find(
        inferred.winding_labels.begin(),
        inferred.winding_labels.end(),
        slot);
    if (it == inferred.winding_labels.end()) {
      throw std::runtime_error(
          "transformer " + std::to_string(transformer.index) + " " + side_label +
          "_winding_topology is incompatible with current phase_mask");
    }
    const int inferred_row = static_cast<int>(
        std::distance(inferred.winding_labels.begin(), it));
    filtered.matrix.row(row_local) = inferred.matrix.row(inferred_row);
  }
  return filtered;
}

bool same_winding_label_set(
    const std::vector<int>& lhs,
    const std::vector<int>& rhs) {
  if (lhs.size() != rhs.size()) {
    return false;
  }
  std::vector<int> lhs_sorted = lhs;
  std::vector<int> rhs_sorted = rhs;
  std::sort(lhs_sorted.begin(), lhs_sorted.end());
  std::sort(rhs_sorted.begin(), rhs_sorted.end());
  return lhs_sorted == rhs_sorted;
}

bool can_pair_winding_rows_by_order(
    const TransformerConnectionEmbedding& from_embedding,
    const TransformerConnectionEmbedding& to_embedding,
    bool allow_explicit_order_pairing) {
  if (!allow_explicit_order_pairing) {
    return false;
  }
  if (from_embedding.matrix.rows() != to_embedding.matrix.rows()) {
    return false;
  }
  return from_embedding.explicit_topology || to_embedding.explicit_topology;
}

bool contains_all_winding_labels(
    const std::vector<int>& available,
    const std::vector<int>& requested) {
  return std::all_of(
      requested.begin(),
      requested.end(),
      [&](int label) {
        return std::find(available.begin(), available.end(), label) != available.end();
      });
}

TransformerConnectionEmbedding filter_embedding_to_winding_labels(
    const TransformerConnectionEmbedding& embedding,
    const std::vector<int>& requested_labels) {
  TransformerConnectionEmbedding filtered;
  filtered.matrix = ComplexMatrixX::Zero(
      static_cast<int>(requested_labels.size()),
      embedding.matrix.cols());
  filtered.winding_labels = requested_labels;
  filtered.explicit_topology = embedding.explicit_topology;

  for (int row_local = 0;
       row_local < static_cast<int>(requested_labels.size());
       ++row_local) {
    const int label = requested_labels[static_cast<std::size_t>(row_local)];
    const auto it = std::find(
        embedding.winding_labels.begin(),
        embedding.winding_labels.end(),
        label);
    if (it == embedding.winding_labels.end()) {
      throw std::runtime_error("internal winding label filtering mismatch");
    }
    const int source_row = static_cast<int>(
        std::distance(embedding.winding_labels.begin(), it));
    filtered.matrix.row(row_local) = embedding.matrix.row(source_row);
  }
  return filtered;
}

std::pair<TransformerConnectionEmbedding, TransformerConnectionEmbedding>
resolve_transformer_winding_correspondence(
    const ThreePhaseTransformer& transformer,
    const std::string& path_label,
    const TransformerConnectionEmbedding& from_embedding,
    const TransformerConnectionEmbedding& to_embedding,
    bool allow_explicit_order_pairing = false,
    bool allow_phase_mask_subset_inference = false) {
  if (from_embedding.matrix.rows() == to_embedding.matrix.rows() &&
      same_winding_label_set(
          from_embedding.winding_labels,
          to_embedding.winding_labels)) {
    return {from_embedding, to_embedding};
  }
  if (can_pair_winding_rows_by_order(
          from_embedding,
          to_embedding,
          allow_explicit_order_pairing)) {
    return {from_embedding, to_embedding};
  }

  if (allow_phase_mask_subset_inference) {
    if (!from_embedding.explicit_topology &&
        to_embedding.matrix.rows() < from_embedding.matrix.rows() &&
        contains_all_winding_labels(
            from_embedding.winding_labels,
            to_embedding.winding_labels)) {
      return {
          filter_embedding_to_winding_labels(
              from_embedding,
              to_embedding.winding_labels),
          to_embedding,
      };
    }
    if (!to_embedding.explicit_topology &&
        from_embedding.matrix.rows() < to_embedding.matrix.rows() &&
        contains_all_winding_labels(
            to_embedding.winding_labels,
            from_embedding.winding_labels)) {
      return {
          from_embedding,
          filter_embedding_to_winding_labels(
              to_embedding,
              from_embedding.winding_labels),
      };
    }
  }

  throw std::runtime_error(
      "transformer " + std::to_string(transformer.index) + " " + path_label +
      " cannot infer winding correspondence from phase masks");
}

std::array<const char*, 3> source_slot_labels(SourceConnectionKind kind) {
  if (kind == SourceConnectionKind::Delta) {
    return {"AB", "BC", "CA"};
  }
  return {"A", "B", "C"};
}

std::string source_slot_label(SourceConnectionKind kind, int slot) {
  return source_slot_labels(kind)[static_cast<std::size_t>(slot)];
}

TransformerConnectionEmbedding filter_embedding_rows(
    const std::string& entity,
    int entity_index,
    const std::string& contract_name,
    const TransformerConnectionEmbedding& inferred,
    const std::vector<int>& requested) {
  TransformerConnectionEmbedding filtered;
  filtered.matrix = ComplexMatrixX::Zero(
      static_cast<int>(requested.size()),
      inferred.matrix.cols());
  filtered.winding_labels = requested;

  for (int row_local = 0; row_local < static_cast<int>(requested.size()); ++row_local) {
    const int slot = requested[static_cast<std::size_t>(row_local)];
    const auto it = std::find(
        inferred.winding_labels.begin(),
        inferred.winding_labels.end(),
        slot);
    if (it == inferred.winding_labels.end()) {
      throw std::runtime_error(
          entity + " " + std::to_string(entity_index) + " " + contract_name +
          " is incompatible with current phase_mask");
    }
    const int inferred_row = static_cast<int>(
        std::distance(inferred.winding_labels.begin(), it));
    filtered.matrix.row(row_local) = inferred.matrix.row(inferred_row);
  }
  return filtered;
}

SourceConnectionEmbedding build_source_connection_embedding(
    const ThreePhaseExternalGrid& source,
    const PhaseTerminal& terminal) {
  if (source.source_topology.empty()) {
    return {
        .embedding = build_wye_connection_embedding(terminal),
        .kind = SourceConnectionKind::Wye,
    };
  }

  const std::vector<std::string> tokens = split_winding_topology_tokens(
      source.source_topology,
      true);
  if (tokens.empty()) {
    throw std::runtime_error(
        "external_grid " + std::to_string(source.index) +
        " source_topology is empty after parsing");
  }

  bool has_wye_token = false;
  bool has_delta_token = false;
  for (const std::string& token : tokens) {
    if (token.size() == 1) {
      has_wye_token = true;
    } else if (token.size() == 2) {
      has_delta_token = true;
    } else {
      throw std::runtime_error(
          "external_grid " + std::to_string(source.index) +
          " source_topology contains unsupported token " + token);
    }
  }
  if (has_wye_token && has_delta_token) {
    throw std::runtime_error(
        "external_grid " + std::to_string(source.index) +
        " source_topology mixes wye and delta labels");
  }

  const SourceConnectionKind kind =
      has_delta_token ? SourceConnectionKind::Delta : SourceConnectionKind::Wye;
  const TransformerConnectionEmbedding inferred =
      (kind == SourceConnectionKind::Delta)
          ? build_delta_connection_embedding(terminal, true)
          : build_wye_connection_embedding(terminal);

  std::vector<int> requested;
  requested.reserve(tokens.size());
  for (const std::string& token : tokens) {
    int slot = -1;
    if (kind == SourceConnectionKind::Delta) {
      if (token == "AB" || token == "BA") slot = 0;
      if (token == "BC" || token == "CB") slot = 1;
      if (token == "CA" || token == "AC") slot = 2;
      if (slot < 0) {
        throw std::runtime_error(
            "external_grid " + std::to_string(source.index) +
            " source_topology contains unsupported delta token " + token);
      }
    } else {
      if (token == "A") slot = 0;
      if (token == "B") slot = 1;
      if (token == "C") slot = 2;
      if (slot < 0) {
        throw std::runtime_error(
            "external_grid " + std::to_string(source.index) +
            " source_topology contains unsupported wye token " + token);
      }
    }
    if (std::find(requested.begin(), requested.end(), slot) != requested.end()) {
      throw std::runtime_error(
          "external_grid " + std::to_string(source.index) +
          " source_topology repeats token " + token);
    }
    requested.push_back(slot);
  }
  std::sort(requested.begin(), requested.end());

  return {
      .embedding = filter_embedding_rows(
          "external_grid",
          source.index,
          "source_topology",
          inferred,
          requested),
      .kind = kind,
  };
}

PrimitiveBranchModel build_transformer_linear_primitive(
    const PrimitiveBranchModel& primitive_template,
    const TransformerConnectionEmbedding& from_embedding,
    const TransformerConnectionEmbedding& to_embedding,
    const Complex& y_winding_scalar,
    const Complex& from_scale,
    const Complex& to_scale,
    const Complex& y_mag,
    bool add_from_delta_antifloat,
    bool add_to_delta_antifloat) {
  PrimitiveBranchModel primitive = primitive_template;
  const ComplexMatrixX y_winding =
      y_winding_scalar * ComplexMatrixX::Identity(
                             from_embedding.matrix.rows(),
                             from_embedding.matrix.rows());
  const ComplexMatrixX k_from =
      from_embedding.matrix / stable_complex_scale(from_scale);
  const ComplexMatrixX k_to =
      to_embedding.matrix / stable_complex_scale(to_scale);

  primitive.y_ff = k_from.adjoint() * y_winding * k_from;
  primitive.y_ft = -k_from.adjoint() * y_winding * k_to;
  primitive.y_tf = -k_to.adjoint() * y_winding * k_from;
  primitive.y_tt = k_to.adjoint() * y_winding * k_to;

  if (add_from_delta_antifloat) {
    primitive.y_ff += delta_antifloat_shunt(from_embedding, y_winding_scalar);
  }
  if (add_to_delta_antifloat) {
    primitive.y_tt += delta_antifloat_shunt(to_embedding, y_winding_scalar);
  }
  if (std::abs(y_mag) > kPhaseValueTol) {
    primitive.y_ff +=
        k_from.adjoint() *
        (y_mag * ComplexMatrixX::Identity(y_winding.rows(), y_winding.rows())) *
        k_from;
  }
  return primitive;
}

std::array<Complex, 3> balanced_source_voltage_setpoint(
    const ThreePhaseExternalGrid& source) {
  const double deg2rad = kPi / 180.0;
  return {
      std::polar(source.vm_pu, (source.va_deg + 0.0) * deg2rad),
      std::polar(source.vm_pu, (source.va_deg - 120.0) * deg2rad),
      std::polar(source.vm_pu, (source.va_deg + 120.0) * deg2rad),
  };
}

std::array<double, 3> source_phase_vm_values_pu(
    const ThreePhaseExternalGrid& source) {
  return {source.vm_a_pu, source.vm_b_pu, source.vm_c_pu};
}

std::array<double, 3> source_phase_va_values_deg(
    const ThreePhaseExternalGrid& source) {
  return {source.va_a_deg, source.va_b_deg, source.va_c_deg};
}

std::array<double, 3> source_phase_r_values_pu(
    const ThreePhaseExternalGrid& source) {
  return {source.r_a_pu, source.r_b_pu, source.r_c_pu};
}

std::array<double, 3> source_phase_x_values_pu(
    const ThreePhaseExternalGrid& source) {
  return {source.x_a_pu, source.x_b_pu, source.x_c_pu};
}

std::string source_active_contract_label(
    SourceConnectionKind kind,
    int slot) {
  return std::string(kind == SourceConnectionKind::Delta ? "slot " : "phase ") +
         source_slot_label(kind, slot);
}

std::string source_contract_support_boundary(
    const ThreePhaseExternalGrid& source) {
  return source.source_topology.empty() ? "phase_mask" : "source_topology";
}

std::array<bool, 3> active_source_slots(
    const SourceConnectionEmbedding& connection) {
  std::array<bool, 3> active_slots{false, false, false};
  for (const int slot : connection.embedding.winding_labels) {
    active_slots[static_cast<std::size_t>(slot)] = true;
  }
  return active_slots;
}

void validate_source_slot_values_against_support(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection,
    const std::array<double, 3>& primary_values,
    const std::array<double, 3>& secondary_values) {
  const auto active_slots = active_source_slots(connection);
  const std::string boundary = source_contract_support_boundary(source);
  for (int slot = 0; slot < 3; ++slot) {
    if (active_slots[static_cast<std::size_t>(slot)]) continue;
    if (std::abs(primary_values[slot]) > kPhaseValueTol ||
        std::abs(secondary_values[slot]) > kPhaseValueTol) {
      throw std::runtime_error(
          "external_grid " + std::to_string(source.index) +
          " carries non-zero " +
          source_active_contract_label(connection.kind, slot) +
          " values outside " + boundary);
    }
  }
}

double max_abs_entry(const ComplexMatrixX& matrix) {
  double max_value = 0.0;
  for (int row = 0; row < matrix.rows(); ++row) {
    for (int col = 0; col < matrix.cols(); ++col) {
      max_value = std::max(max_value, std::abs(matrix(row, col)));
    }
  }
  return max_value;
}

double max_abs_entry(const ComplexVectorX& vector) {
  if (vector.size() == 0) return 0.0;
  return vector.cwiseAbs().maxCoeff();
}

void validate_source_phase_voltage_contract(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  if (!source.use_phase_voltage_setpoint) return;

  const auto vm_values = source_phase_vm_values_pu(source);
  const auto va_values = source_phase_va_values_deg(source);
  validate_source_slot_values_against_support(
      source,
      connection,
      vm_values,
      va_values);
  const auto active_slots = active_source_slots(connection);
  for (int phase = 0; phase < 3; ++phase) {
    if (!active_slots[static_cast<std::size_t>(phase)]) continue;
    if (vm_values[phase] <= kPhaseValueTol) {
      throw std::runtime_error(
          "external_grid " + std::to_string(source.index) +
          " requires explicit source voltage magnitude on active " +
          source_active_contract_label(connection.kind, phase));
    }
  }
}

void validate_source_phase_impedance_contract(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  if (!source.use_phase_impedance) return;

  const auto r_values = source_phase_r_values_pu(source);
  const auto x_values = source_phase_x_values_pu(source);
  validate_source_slot_values_against_support(
      source,
      connection,
      r_values,
      x_values);
  const auto active_slots = active_source_slots(connection);
  for (int phase = 0; phase < 3; ++phase) {
    if (!active_slots[static_cast<std::size_t>(phase)]) continue;
    if (std::abs(Complex(r_values[phase], x_values[phase])) <= kPhaseValueTol) {
      throw std::runtime_error(
          "external_grid " + std::to_string(source.index) +
          " requires explicit source impedance on active " +
          source_active_contract_label(connection.kind, phase));
    }
  }
}

void validate_source_phase_impedance_matrix_contract(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  if (!source.use_phase_impedance_matrix) return;

  const auto active_slots = active_source_slots(connection);
  const std::string boundary = source_contract_support_boundary(source);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (active_slots[static_cast<std::size_t>(row)] &&
          active_slots[static_cast<std::size_t>(col)]) {
        continue;
      }
      const double r_value = phase_matrix_get(source.r_matrix_pu, row, col);
      const double x_value = phase_matrix_get(source.x_matrix_pu, row, col);
      if (std::abs(r_value) > kPhaseValueTol ||
          std::abs(x_value) > kPhaseValueTol) {
        throw std::runtime_error(
            "external_grid " + std::to_string(source.index) +
            " full phase-domain impedance matrix carries non-zero entries outside " +
            boundary);
      }
    }
  }
}

std::array<Complex, 3> explicit_source_voltage_setpoint(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  validate_source_phase_voltage_contract(source, connection);
  const double deg2rad = kPi / 180.0;
  return {
      std::polar(source.vm_a_pu, source.va_a_deg * deg2rad),
      std::polar(source.vm_b_pu, source.va_b_deg * deg2rad),
      std::polar(source.vm_c_pu, source.va_c_deg * deg2rad),
  };
}

std::array<Complex, 3> build_source_slot_voltage_setpoint(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  return source.use_phase_voltage_setpoint
             ? explicit_source_voltage_setpoint(source, connection)
             : balanced_source_voltage_setpoint(source);
}

ComplexVectorX build_source_winding_voltage_setpoint(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  return select_phase_subvector(
      build_source_slot_voltage_setpoint(source, connection),
      connection.embedding.winding_labels);
}

Complex source_sequence_impedance_from_short_circuit(
    double s_sc_mva,
    double rx_ratio,
    double base_mva) {
  if (s_sc_mva <= kPhaseValueTol || base_mva <= kPhaseValueTol) {
    return Complex(0.0, 0.0);
  }
  const double z_mag = base_mva / s_sc_mva;
  const double rx = std::max(0.0, rx_ratio);
  const double x = z_mag / std::sqrt(1.0 + rx * rx);
  const double r = rx * x;
  return Complex(r, x);
}

Complex build_source_positive_sequence_impedance(
    const ThreePhaseExternalGrid& source,
    double base_mva) {
  const Complex explicit_z(source.r1_pu, source.x1_pu);
  if (std::abs(explicit_z) > kPhaseValueTol) {
    return explicit_z;
  }
  Complex derived = source_sequence_impedance_from_short_circuit(
      source.s_sc_max_mva,
      source.rx_max,
      base_mva);
  if (std::abs(derived) > kPhaseValueTol) {
    return derived;
  }
  return source_sequence_impedance_from_short_circuit(
      source.s_sc_min_mva,
      source.rx_min,
      base_mva);
}

Complex build_source_negative_sequence_impedance(
    const ThreePhaseExternalGrid& source,
    double base_mva,
    const Complex& z1) {
  const Complex explicit_z(source.r2_pu, source.x2_pu);
  if (std::abs(explicit_z) > kPhaseValueTol) {
    return explicit_z;
  }
  Complex derived = source_sequence_impedance_from_short_circuit(
      source.s_sc_min_mva,
      (std::abs(source.rx_min) > kPhaseValueTol) ? source.rx_min : source.rx_max,
      base_mva);
  if (std::abs(derived) > kPhaseValueTol) {
    return derived;
  }
  return z1;
}

Complex build_source_zero_sequence_impedance(
    const ThreePhaseExternalGrid& source,
    double base_mva,
    const Complex& z1) {
  const Complex explicit_z(source.r0_pu, source.x0_pu);
  if (std::abs(explicit_z) > kPhaseValueTol) {
    return explicit_z;
  }
  Complex derived = source_sequence_impedance_from_short_circuit(
      source.s_sc_min_mva,
      (std::abs(source.rx_min) > kPhaseValueTol) ? source.rx_min : source.rx_max,
      base_mva);
  if (std::abs(derived) > kPhaseValueTol) {
    return derived;
  }
  return z1;
}

ComplexMatrixX build_source_diagonal_winding_impedance_submatrix(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  validate_source_phase_impedance_contract(source, connection);
  const auto r_values = source_phase_r_values_pu(source);
  const auto x_values = source_phase_x_values_pu(source);
  ComplexMatrixX z_phase = ComplexMatrixX::Zero(
      static_cast<int>(connection.embedding.winding_labels.size()),
      static_cast<int>(connection.embedding.winding_labels.size()));
  for (int local = 0;
       local < static_cast<int>(connection.embedding.winding_labels.size());
       ++local) {
    const int slot = connection.embedding.winding_labels[static_cast<std::size_t>(local)];
    z_phase(local, local) = Complex(r_values[slot], x_values[slot]);
  }
  return z_phase;
}

ComplexMatrixX build_source_matrix_winding_impedance_submatrix(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  validate_source_phase_impedance_matrix_contract(source, connection);
  return select_phase_submatrix(
      build_phase_matrix(source.r_matrix_pu, source.x_matrix_pu),
      connection.embedding.winding_labels);
}

Complex build_source_admittance_from_impedance_or_throw(
    const Complex& impedance,
    const std::string& label) {
  if (std::abs(impedance) <= kPhaseValueTol) {
    throw std::runtime_error(label + " is zero");
  }
  return Complex(1.0, 0.0) / impedance;
}

ComplexMatrixX build_source_sequence_phase_admittance_submatrix(
    const ThreePhaseExternalGrid& source,
    const PhaseTerminal& terminal,
    const SourceConnectionEmbedding& connection,
    double base_mva) {
  const Complex z1 = build_source_positive_sequence_impedance(source, base_mva);
  if (std::abs(z1) <= kPhaseValueTol) {
    throw std::runtime_error(
        "external_grid " + std::to_string(source.index) +
        " requires explicit impedance or short-circuit capacity");
  }
  const Complex z2 = build_source_negative_sequence_impedance(source, base_mva, z1);
  const Complex z0 = build_source_zero_sequence_impedance(source, base_mva, z1);
  const Complex y1 = build_source_admittance_from_impedance_or_throw(
      z1,
      "external_grid " + std::to_string(source.index) + " positive-sequence impedance");
  const Complex y2 = build_source_admittance_from_impedance_or_throw(
      z2,
      "external_grid " + std::to_string(source.index) + " negative-sequence impedance");
  const Complex y0 =
      (connection.kind == SourceConnectionKind::Delta)
          ? Complex(0.0, 0.0)
          : build_source_admittance_from_impedance_or_throw(
                z0,
                "external_grid " + std::to_string(source.index) +
                    " zero-sequence impedance");
  return select_phase_submatrix(
      build_sequence_zabc(y0, y1, y2),
      terminal.phases);
}

ComplexMatrixX build_source_sequence_winding_admittance_submatrix(
    const ThreePhaseExternalGrid& source,
    const PhaseTerminal& terminal,
    const SourceConnectionEmbedding& connection,
    double base_mva) {
  const ComplexMatrixX y_phase = build_source_sequence_phase_admittance_submatrix(
      source,
      terminal,
      connection,
      base_mva);
  if (connection.kind == SourceConnectionKind::Wye) {
    return y_phase;
  }

  const std::string label =
      "external_grid " + std::to_string(source.index) + " source_topology embedding";
  const ComplexMatrixX embedding_pinv =
      pseudoinverse(connection.embedding.matrix, label);
  const ComplexMatrixX y_winding =
      embedding_pinv.adjoint() * y_phase * embedding_pinv;
  const ComplexMatrixX reconstructed =
      connection.embedding.matrix.adjoint() * y_winding * connection.embedding.matrix;
  const ComplexMatrixX reconstruction_error = reconstructed - y_phase;
  const double reference = std::max(1.0, max_abs_entry(y_phase));
  if (max_abs_entry(reconstruction_error) > 1e-8 * reference) {
    throw std::runtime_error(label + " cannot reconstruct sequence fallback");
  }
  return y_winding;
}

ComplexMatrixX build_source_winding_impedance_submatrix(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection) {
  if (source.use_phase_impedance_matrix) {
    return build_source_matrix_winding_impedance_submatrix(source, connection);
  }
  if (source.use_phase_impedance) {
    return build_source_diagonal_winding_impedance_submatrix(source, connection);
  }
  throw std::runtime_error(
      "build_source_winding_impedance_submatrix requires explicit source impedance contract");
}

Complex source_delta_antifloat_anchor_admittance(
    const ComplexMatrixX& y_winding) {
  const double reference = std::max(max_abs_entry(y_winding), 1.0);
  return Complex(reference, 0.0);
}

ComplexVectorX solve_source_open_circuit_phase_voltage(
    const ThreePhaseExternalGrid& source,
    const SourceConnectionEmbedding& connection,
    const ComplexVectorX& e_winding) {
  Eigen::JacobiSVD<ComplexMatrixX> svd(
      connection.embedding.matrix,
      Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(1e-10);
  const ComplexVectorX v_phase = svd.solve(e_winding);
  const ComplexVectorX residual =
      connection.embedding.matrix * v_phase - e_winding;
  const double rhs_scale = std::max(1.0, max_abs_entry(e_winding));
  if (max_abs_entry(residual) > 1e-8 * rhs_scale) {
    throw std::runtime_error(
        "external_grid " + std::to_string(source.index) +
        " source voltage setpoint is incompatible with source_topology");
  }
  return v_phase;
}

SourceNortonPrimitive build_source_norton_primitive(
    const ThreePhaseExternalGrid& source,
    const PhaseTerminal& terminal,
    double base_mva) {
  const SourceConnectionEmbedding connection =
      build_source_connection_embedding(source, terminal);
  const ComplexVectorX e_winding =
      build_source_winding_voltage_setpoint(source, connection);
  ComplexMatrixX y_winding;
  if (source.use_phase_impedance_matrix || source.use_phase_impedance) {
    const ComplexMatrixX z_winding =
        build_source_winding_impedance_submatrix(source, connection);
    y_winding = inverse_or_throw(
        z_winding,
        "external_grid " + std::to_string(source.index) + " winding submatrix");
  } else {
    y_winding = build_source_sequence_winding_admittance_submatrix(
        source,
        terminal,
        connection,
        base_mva);
  }

  SourceNortonPrimitive primitive;
  primitive.y_phase =
      connection.embedding.matrix.adjoint() * y_winding * connection.embedding.matrix;
  if (connection.kind == SourceConnectionKind::Delta) {
    primitive.y_phase += delta_antifloat_shunt(
        connection.embedding,
        source_delta_antifloat_anchor_admittance(y_winding));
  }
  primitive.fixed_current =
      -(connection.embedding.matrix.adjoint() * y_winding * e_winding);
  primitive.open_circuit_phase_voltage = solve_source_open_circuit_phase_voltage(
      source,
      connection,
      e_winding);
  return primitive;
}

std::array<double, 3> bus_vm_values(const ThreePhaseACBus& bus) {
  return {bus.vm_a_pu, bus.vm_b_pu, bus.vm_c_pu};
}

std::array<double, 3> bus_va_values_deg(const ThreePhaseACBus& bus) {
  return {bus.va_a_deg, bus.va_b_deg, bus.va_c_deg};
}

void set_bus_voltage_phase(
    ThreePhaseBusVoltage& voltage,
    int phase_index,
    double vm_value,
    double va_value_deg) {
  if (phase_index == 0) {
    voltage.vm_a_pu = vm_value;
    voltage.va_a_deg = va_value_deg;
    return;
  }
  if (phase_index == 1) {
    voltage.vm_b_pu = vm_value;
    voltage.va_b_deg = va_value_deg;
    return;
  }
  voltage.vm_c_pu = vm_value;
  voltage.va_c_deg = va_value_deg;
}

void set_branch_power_phase(
    ThreePhaseBranchPower& branch_power,
    int phase_index,
    double p_mw,
    double q_mvar) {
  if (phase_index == 0) {
    branch_power.p_a_mw = p_mw;
    branch_power.q_a_mvar = q_mvar;
    return;
  }
  if (phase_index == 1) {
    branch_power.p_b_mw = p_mw;
    branch_power.q_b_mvar = q_mvar;
    return;
  }
  branch_power.p_c_mw = p_mw;
  branch_power.q_c_mvar = q_mvar;
}

Complex phase_phasor(
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    const PhaseNodeIndexer& indexer,
    int bus_offset,
    int phase_index) {
  if (!indexer.has_node(bus_offset, phase_index)) return Complex(0.0, 0.0);
  const int node = indexer.node_index(bus_offset, phase_index);
  return std::polar(vm[node], va[node]);
}

void stamp_shunt(
    std::vector<ComplexTriplet>& triplets,
    const PhaseNodeIndexer& indexer,
    int bus_offset,
    PhaseMask mask,
    const std::array<Complex, 3>& y_diag) {
  const PhaseTerminal terminal = collect_terminal_nodes(
      indexer,
      bus_offset,
      mask,
      "bus",
      bus_offset,
      "shunt");
  ComplexMatrixX shunt = ComplexMatrixX::Zero(
      static_cast<int>(terminal.nodes.size()),
      static_cast<int>(terminal.nodes.size()));
  for (int local = 0; local < static_cast<int>(terminal.phases.size()); ++local) {
    shunt(local, local) = y_diag[terminal.phases[static_cast<size_t>(local)]];
  }
  stamp_terminal_shunt(triplets, terminal, shunt, "stamp_shunt");
}

void stamp_line(
    std::vector<ComplexTriplet>& triplets,
    const PhaseNodeIndexer& indexer,
    int from_bus_offset,
    int to_bus_offset,
    const ThreePhaseACLine& line) {
  const PrimitiveBranchModel primitive =
      build_line_primitive(indexer, from_bus_offset, to_bus_offset, line);
  if (primitive.y_ff.rows() == 0) return;
  stamp_primitive_branch(
      triplets,
      primitive.from_terminal,
      primitive.to_terminal,
      primitive.y_ff,
      primitive.y_ft,
      primitive.y_tf,
      primitive.y_tt,
      "line");
}

double phase_domain_power_base_scale(double base_mva);

void stamp_injection(
    Eigen::VectorXd& p_spec,
    Eigen::VectorXd& q_spec,
    const PhaseNodeIndexer& indexer,
    int bus_offset,
    PhaseMask phase_mask,
    const std::array<double, 3>& p_values_mw,
    const std::array<double, 3>& q_values_mvar,
    double base_mva,
    double sign) {
  require_mask_not_empty("injection bus", bus_offset, phase_mask);
  require_phase_values_on_mask(
      "injection bus",
      bus_offset,
      phase_mask,
      p_values_mw,
      q_values_mvar);
  if (!indexer.bus_phase_mask(bus_offset).contains(phase_mask)) {
    throw std::runtime_error(
        "stamp_injection: component phase_mask is not a subset of bus phase mask");
  }

  const double scale = phase_domain_power_base_scale(base_mva);
  for (int phase = 0; phase < 3; ++phase) {
    if (!phase_mask.has(phase)) continue;
    const int node = indexer.node_index(bus_offset, phase);
    p_spec[node] += sign * p_values_mw[phase] * scale;
    q_spec[node] += sign * q_values_mvar[phase] * scale;
  }
}

TransformerPrimitiveRecord build_transformer_primitive(
    const PhaseNodeIndexer& indexer,
    const ThreePhaseTransformer& transformer,
    const std::unordered_map<int, int>& id_map,
    const ThreePhaseACSystem& sys,
    double base_mva) {
  auto it_hv = id_map.find(transformer.hv_bus);
  auto it_lv = id_map.find(transformer.lv_bus);
  if (it_hv == id_map.end() || it_lv == id_map.end()) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " endpoint bus is missing from three-phase NR bus map");
  }

  if (transformer.tap_side != 0 && transformer.tap_side != 1) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " only supports tap_side 0/1 in three-phase NR");
  }

  const ParsedVectorGroup group = parse_vector_group(transformer.vector_group);
  const double vector_group_shift = shift_deg_from_clock(group.clock);
  const bool explicit_shift = std::abs(transformer.shift_deg) > kPhaseValueTol;
  const double effective_shift_deg =
      explicit_shift ? transformer.shift_deg : vector_group_shift;
  if (explicit_shift &&
      !transformer.vector_group.empty() &&
      std::abs(transformer.shift_deg - vector_group_shift) > 1e-6) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " shift_deg conflicts with vector_group clock");
  }

  const double winding_tap_pu = tap_from_step(
      transformer.tap_pos,
      transformer.tap_neutral,
      transformer.tap_step_percent);
  const CanonicalTapProjection tap_projection =
      normalize_transformer_tap_to_from_side(transformer.tap_side, winding_tap_pu);
  const Complex z1 = transformer_impedance_from_vk_vkr(
      transformer.vk_percent,
      transformer.vkr_percent,
      base_mva,
      transformer.sn_mva);
  if (std::abs(z1) <= kPhaseValueTol) {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " has zero leakage impedance");
  }
  const Complex y_mag = transformer_magnetizing_admittance(transformer, base_mva);

  PrimitiveBranchModel primitive;

  const int hv_bus_offset = it_hv->second;
  const int lv_bus_offset = it_lv->second;
  const auto& hv_bus = sys.buses[static_cast<size_t>(hv_bus_offset)];
  const auto& lv_bus = sys.buses[static_cast<size_t>(lv_bus_offset)];

  primitive.from_terminal = collect_terminal_nodes(
      indexer,
      hv_bus_offset,
      transformer.hv_phase_mask,
      "transformer",
      transformer.index,
      "hv");
  primitive.to_terminal = collect_terminal_nodes(
      indexer,
      lv_bus_offset,
      transformer.lv_phase_mask,
      "transformer",
      transformer.index,
      "lv");

  const bool hv_wye_like = is_wye_like_connection(group.hv);
  const bool lv_wye_like = is_wye_like_connection(group.lv);
  const bool hv_delta = is_delta_connection(group.hv);
  const bool lv_delta = is_delta_connection(group.lv);

  if (hv_wye_like && lv_wye_like) {
    const Complex clock_factor = same_connection_clock_factor(
        transformer,
        "wye/wye path",
        effective_shift_deg);
    const auto hv_embedding =
        build_transformer_connection_embedding(
            transformer,
            primitive.from_terminal,
            group.hv,
            false,
            transformer.hv_winding_topology,
            "hv");
    const auto lv_embedding =
        build_transformer_connection_embedding(
            transformer,
            primitive.to_terminal,
            group.lv,
            false,
            transformer.lv_winding_topology,
            "lv");
    const auto [hv_aligned, lv_aligned] =
        resolve_transformer_winding_correspondence(
            transformer,
            "wye/wye path",
            hv_embedding,
            lv_embedding);
    if (group.hv == TransformerConnectionKind::GroundedWye &&
        group.lv == TransformerConnectionKind::GroundedWye) {
      const Complex tap =
          Complex(tap_projection.branch_tap_pu, 0.0) *
          clock_factor;
      const ZeroSequencePortAdmittance zero_sequence =
          build_transformer_zero_sequence_port_admittance(
              transformer,
              base_mva,
              tap_projection.impedance_scale);
      primitive = build_grounded_wye_grounded_wye_primitive(
          primitive,
          Complex(1.0, 0.0) / (z1 * tap_projection.impedance_scale),
          zero_sequence,
          tap,
          y_mag);
    } else {
      const Complex y_winding_scalar =
          Complex(1.0, 0.0) / (z1 * tap_projection.impedance_scale);
      primitive = build_transformer_linear_primitive(
          primitive,
          hv_aligned,
          lv_aligned,
          y_winding_scalar,
          Complex(tap_projection.branch_tap_pu, 0.0) *
              clock_factor,
          Complex(1.0, 0.0),
          y_mag,
          false,
          false);
      add_transformer_zero_sequence_shunts_with_blocked_ports(
          primitive,
          transformer,
          base_mva,
          tap_projection.impedance_scale,
          group.hv == TransformerConnectionKind::GroundedWye,
          tap_projection.branch_tap_pu,
          group.lv == TransformerConnectionKind::GroundedWye,
          1.0,
          "wye/wye path");
    }
  } else if (hv_delta && lv_wye_like) {
    const MixedConnectionClockProjection clock_projection =
        resolve_mixed_connection_clock(
            transformer,
            "delta/wye path",
            effective_shift_deg,
            true);

    const auto hv_embedding = build_transformer_connection_embedding(
        transformer,
        primitive.from_terminal,
        group.hv,
        clock_projection.delta_positive_thirty_deg,
        transformer.hv_winding_topology,
        "hv");
    const auto lv_embedding =
        build_transformer_connection_embedding(
            transformer,
            primitive.to_terminal,
            group.lv,
            false,
            transformer.lv_winding_topology,
            "lv");
    const auto [hv_aligned, lv_aligned] =
        resolve_transformer_winding_correspondence(
            transformer,
            "delta/wye path",
            hv_embedding,
            lv_embedding,
            true,
            true);

    const Complex y_winding_scalar =
        Complex(1.0, 0.0) / (z1 * tap_projection.impedance_scale);
    primitive = build_transformer_linear_primitive(
        primitive,
        hv_aligned,
        lv_aligned,
        y_winding_scalar,
        Complex(
            tap_projection.branch_tap_pu * std::sqrt(3.0) *
                clock_projection.phase_polarity,
            0.0),
        Complex(1.0, 0.0),
        y_mag,
        true,
        false);
    add_transformer_zero_sequence_shunts_with_blocked_ports(
        primitive,
        transformer,
        base_mva,
        tap_projection.impedance_scale,
        false,
        tap_projection.branch_tap_pu * std::sqrt(3.0),
        group.lv == TransformerConnectionKind::GroundedWye,
        1.0,
        "delta/wye path");
  } else if (hv_wye_like && lv_delta) {
    const MixedConnectionClockProjection clock_projection =
        resolve_mixed_connection_clock(
            transformer,
            "wye/delta path",
            effective_shift_deg,
            false);

    const auto hv_embedding =
        build_transformer_connection_embedding(
            transformer,
            primitive.from_terminal,
            group.hv,
            false,
            transformer.hv_winding_topology,
            "hv");
    const auto lv_embedding = build_transformer_connection_embedding(
        transformer,
        primitive.to_terminal,
        group.lv,
        clock_projection.delta_positive_thirty_deg,
        transformer.lv_winding_topology,
        "lv");
    const auto [hv_aligned, lv_aligned] =
        resolve_transformer_winding_correspondence(
            transformer,
            "wye/delta path",
            hv_embedding,
            lv_embedding,
            true,
            true);

    const Complex y_winding_scalar =
        Complex(1.0, 0.0) / (z1 * tap_projection.impedance_scale);
    primitive = build_transformer_linear_primitive(
        primitive,
        hv_aligned,
        lv_aligned,
        y_winding_scalar,
        Complex(tap_projection.branch_tap_pu, 0.0),
        Complex(std::sqrt(3.0) * clock_projection.phase_polarity, 0.0),
        y_mag,
        false,
        true);
    add_transformer_zero_sequence_shunts_with_blocked_ports(
        primitive,
        transformer,
        base_mva,
        tap_projection.impedance_scale,
        group.hv == TransformerConnectionKind::GroundedWye,
        tap_projection.branch_tap_pu,
        false,
        std::sqrt(3.0),
        "wye/delta path");
  } else if (hv_delta && lv_delta) {
    const Complex clock_factor = same_connection_clock_factor(
        transformer,
        "delta-delta path",
        effective_shift_deg);

    const auto hv_embedding =
        build_transformer_connection_embedding(
            transformer,
            primitive.from_terminal,
            group.hv,
            true,
            transformer.hv_winding_topology,
            "hv");
    const auto lv_embedding =
        build_transformer_connection_embedding(
            transformer,
            primitive.to_terminal,
            group.lv,
            true,
            transformer.lv_winding_topology,
            "lv");
    const auto [hv_aligned, lv_aligned] =
        resolve_transformer_winding_correspondence(
            transformer,
            "delta/delta path",
            hv_embedding,
            lv_embedding);

    const Complex y_winding_scalar =
        Complex(1.0, 0.0) / (z1 * tap_projection.impedance_scale);
    primitive = build_transformer_linear_primitive(
        primitive,
        hv_aligned,
        lv_aligned,
        y_winding_scalar,
        Complex(
            tap_projection.branch_tap_pu * std::sqrt(3.0),
            0.0) *
            clock_factor,
        Complex(std::sqrt(3.0), 0.0),
        y_mag,
        true,
        true);
    add_transformer_zero_sequence_shunts_with_blocked_ports(
        primitive,
        transformer,
        base_mva,
        tap_projection.impedance_scale,
        false,
        tap_projection.branch_tap_pu * std::sqrt(3.0),
        false,
        std::sqrt(3.0),
        "delta/delta path");
  } else {
    throw std::runtime_error(
        "transformer " + std::to_string(transformer.index) +
        " vector_group " + transformer.vector_group +
        " is unsupported in three-phase NR");
  }

  return {
      .transformer_index = transformer.index,
      .transformer_name = transformer.name,
      .hv_bus = transformer.hv_bus,
      .lv_bus = transformer.lv_bus,
      .hv_phase_mask = transformer.hv_phase_mask,
      .lv_phase_mask = transformer.lv_phase_mask,
      .hv_base_current_amps = bus_base_current_amps(base_mva, hv_bus),
      .lv_base_current_amps = bus_base_current_amps(base_mva, lv_bus),
      .primitive = std::move(primitive),
  };
}

void stamp_transformer(
    std::vector<ComplexTriplet>& triplets,
    std::vector<TransformerPrimitiveRecord>& transformer_primitives,
    const PhaseNodeIndexer& indexer,
    const ThreePhaseTransformer& transformer,
    const std::unordered_map<int, int>& id_map,
    const ThreePhaseACSystem& sys,
    double base_mva) {
  TransformerPrimitiveRecord primitive =
      build_transformer_primitive(indexer, transformer, id_map, sys, base_mva);
  stamp_primitive_branch(
      triplets,
      primitive.primitive.from_terminal,
      primitive.primitive.to_terminal,
      primitive.primitive.y_ff,
      primitive.primitive.y_ft,
      primitive.primitive.y_tf,
      primitive.primitive.y_tt,
      "transformer");
  transformer_primitives.push_back(std::move(primitive));
}

void stamp_source(
    std::vector<ComplexTriplet>& triplets,
    const PhaseNodeIndexer& indexer,
    const ThreePhaseExternalGrid& source,
    const std::unordered_map<int, int>& id_map,
    ComplexVectorX& fixed_current,
    double base_mva) {
  auto it_bus = id_map.find(source.bus);
  if (it_bus == id_map.end()) return;

  const PhaseTerminal terminal = collect_terminal_nodes(
      indexer,
      it_bus->second,
      source.phase_mask,
      "external_grid",
      source.index,
      "terminal");
  const SourceNortonPrimitive primitive = build_source_norton_primitive(
      source,
      terminal,
      base_mva);
  stamp_terminal_shunt(triplets, terminal, primitive.y_phase, "external_grid");
  add_fixed_current(
      fixed_current,
      terminal,
      primitive.fixed_current,
      "external_grid");
}

PhaseNetworkModel build_phase_network_model(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<int, int>& id_map,
    const PhaseNodeIndexer& indexer,
    const PreparedLoadSet& prepared_loads,
    bool include_shunts,
    double base_mva) {
  std::vector<ComplexTriplet> triplets;
  triplets.reserve(static_cast<size_t>(
      18 * sys.lines.size() +
      36 * sys.transformers.size() +
      9 * sys.external_grids.size() +
      3 * sys.buses.size() +
      9 * prepared_loads.wye_loads.size() +
      4 * prepared_loads.branch_loads.size()));
  ComplexVectorX fixed_current = ComplexVectorX::Zero(indexer.total_nodes);
  std::vector<TransformerPrimitiveRecord> transformer_primitives;
  transformer_primitives.reserve(sys.transformers.size());

  if (include_shunts) {
    const double phase_scale = phase_domain_power_base_scale(base_mva);
    for (size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
      const auto& bus = sys.buses[bus_offset];
      if (!bus.in_service) continue;
      require_phase_values_on_mask(
          "bus",
          bus.index,
          bus.phase_mask,
          {bus.gs_a_mw, bus.gs_b_mw, bus.gs_c_mw},
          {bus.bs_a_mvar, bus.bs_b_mvar, bus.bs_c_mvar});
      stamp_shunt(
          triplets,
          indexer,
          static_cast<int>(bus_offset),
          bus.phase_mask,
          {Complex(bus.gs_a_mw * phase_scale, bus.bs_a_mvar * phase_scale),
           Complex(bus.gs_b_mw * phase_scale, bus.bs_b_mvar * phase_scale),
           Complex(bus.gs_c_mw * phase_scale, bus.bs_c_mvar * phase_scale)});
    }
  }

  stamp_prepared_constant_impedance_loads(triplets, prepared_loads);

  for (const auto& line : sys.lines) {
    if (!line.in_service) continue;
    auto it_f = id_map.find(line.from_bus);
    auto it_t = id_map.find(line.to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    stamp_line(triplets, indexer, it_f->second, it_t->second, line);
  }

  for (const auto& transformer : sys.transformers) {
    if (!transformer.in_service) continue;
    stamp_transformer(
        triplets,
        transformer_primitives,
        indexer,
        transformer,
        id_map,
        sys,
        base_mva);
  }

  for (const auto& source : sys.external_grids) {
    if (!source.in_service) continue;
    stamp_source(
        triplets,
        indexer,
        source,
        id_map,
        fixed_current,
        base_mva);
  }

  Eigen::SparseMatrix<Complex> ybus(indexer.total_nodes, indexer.total_nodes);
  ybus.setFromTriplets(
      triplets.begin(),
      triplets.end(),
      [](const Complex& a, const Complex& b) { return a + b; });
  return {
      .ybus = std::move(ybus),
      .fixed_current = std::move(fixed_current),
      .transformer_primitives = std::move(transformer_primitives),
  };
}

// =========================================================================
// Compute per-phase net specified power: P_spec_φ, Q_spec_φ
// Convention: P_spec = P_gen − P_load  (bus injection positive)
// =========================================================================
struct PhaseSpec {
  Eigen::VectorXd p_spec;  // size = total compact phase nodes
  Eigen::VectorXd q_spec;  // size = total compact phase nodes
};

std::string lowercase_copy(const std::string& text) {
  std::string lower = text;
  std::transform(
      lower.begin(),
      lower.end(),
      lower.begin(),
      [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
      });
  return lower;
}

bool legacy_zip_is_default(const ThreePhaseLoad& load) {
  return std::abs(load.const_z_percent) <= 1e-9 &&
         std::abs(load.const_i_percent) <= 1e-9 &&
         std::abs(load.const_p_percent - 100.0) <= 1e-9;
}

bool explicit_zip_is_unset(const ThreePhaseLoad& load) {
  return load.p_const_z_percent < 0.0 &&
         load.p_const_i_percent < 0.0 &&
         load.p_const_p_percent < 0.0 &&
         load.q_const_z_percent < 0.0 &&
         load.q_const_i_percent < 0.0 &&
         load.q_const_p_percent < 0.0;
}

void validate_zip_weights(
    const std::string& label,
    int load_index,
    double z_percent,
    double i_percent,
    double p_percent) {
  if (z_percent < -1e-9 || i_percent < -1e-9 || p_percent < -1e-9) {
    throw std::runtime_error(
        label + " for load " + std::to_string(load_index) +
        " must be non-negative");
  }
  const double total = z_percent + i_percent + p_percent;
  if (std::abs(total - 100.0) > 1e-6) {
    throw std::runtime_error(
        label + " for load " + std::to_string(load_index) +
        " must sum to 100%");
  }
}

LoadZipWeights resolve_active_zip_weights(const ThreePhaseLoad& load) {
  const bool explicit_set = !explicit_zip_is_unset(load);
  if (explicit_set) {
    if (load.p_const_z_percent < 0.0 || load.p_const_i_percent < 0.0 ||
        load.p_const_p_percent < 0.0 || load.q_const_z_percent < 0.0 ||
        load.q_const_i_percent < 0.0 || load.q_const_p_percent < 0.0) {
      throw std::runtime_error(
          "three-phase load " + std::to_string(load.index) +
          " has partially specified explicit ZIP weights");
    }
    if (!legacy_zip_is_default(load)) {
      throw std::runtime_error(
          "three-phase load " + std::to_string(load.index) +
          " mixes legacy shared ZIP weights with explicit active/reactive ZIP weights");
    }
    validate_zip_weights(
        "active ZIP weights",
        load.index,
        load.p_const_z_percent,
        load.p_const_i_percent,
        load.p_const_p_percent);
    return {
        load.p_const_z_percent / 100.0,
        load.p_const_i_percent / 100.0,
        load.p_const_p_percent / 100.0,
    };
  }

  validate_zip_weights(
      "legacy shared ZIP weights",
      load.index,
      load.const_z_percent,
      load.const_i_percent,
      load.const_p_percent);
  return {
      load.const_z_percent / 100.0,
      load.const_i_percent / 100.0,
      load.const_p_percent / 100.0,
  };
}

LoadZipWeights resolve_reactive_zip_weights(const ThreePhaseLoad& load) {
  const bool explicit_set = !explicit_zip_is_unset(load);
  if (explicit_set) {
    validate_zip_weights(
        "reactive ZIP weights",
        load.index,
        load.q_const_z_percent,
        load.q_const_i_percent,
        load.q_const_p_percent);
    return {
        load.q_const_z_percent / 100.0,
        load.q_const_i_percent / 100.0,
        load.q_const_p_percent / 100.0,
    };
  }

  validate_zip_weights(
      "legacy shared ZIP weights",
      load.index,
      load.const_z_percent,
      load.const_i_percent,
      load.const_p_percent);
  return {
      load.const_z_percent / 100.0,
      load.const_i_percent / 100.0,
      load.const_p_percent / 100.0,
  };
}

LoadNeutralKind resolve_load_neutral_kind(const ThreePhaseLoad& load) {
  if (load.connection == "delta") {
    return LoadNeutralKind::OpenNeutral;
  }
  if (!load.grounded) return LoadNeutralKind::OpenNeutral;
  if (std::abs(load.r_neut_ohm) > kPhaseValueTol ||
      std::abs(load.x_neut_ohm) > kPhaseValueTol) {
    return LoadNeutralKind::ImpedanceGrounded;
  }
  return LoadNeutralKind::SolidGrounded;
}

void validate_load_voltage_contract(const ThreePhaseLoad& load) {
  if (load.vmin_pu <= kPhaseValueTol || load.vmax_pu <= kPhaseValueTol) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load.index) +
        " requires positive vmin_pu/vmax_pu");
  }
  if (load.vmax_pu < load.vmin_pu) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load.index) +
        " has vmax_pu < vmin_pu");
  }
  if (load.zipv_cutoff_pu < -kPhaseValueTol) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load.index) +
        " has negative zipv_cutoff_pu");
  }
}

double phase_domain_power_base_scale(double base_mva) {
  if (base_mva <= kPhaseValueTol) {
    throw std::runtime_error(
        "phase-domain solver requires positive base_mva for per-phase scaling");
  }
  return 3.0 / base_mva;
}

Complex load_nominal_power_pu(
    double p_mw,
    double q_mvar,
    double base_mva) {
  const double scale = phase_domain_power_base_scale(base_mva);
  return {p_mw * scale, q_mvar * scale};
}

double three_phase_bus_base_impedance_ohm(
    const ThreePhaseACBus& bus,
    double base_mva) {
  if (bus.base_kv <= kPhaseValueTol || base_mva <= kPhaseValueTol) {
    throw std::runtime_error(
        "three-phase bus " + std::to_string(bus.index) +
        " is missing base_kv/base_mva for neutral impedance conversion");
  }
  const double base_kv_ll =
      bus.phase_mask.count() == 3 ? bus.base_kv
                                  : bus.base_kv * std::sqrt(3.0);
  return (base_kv_ll * base_kv_ll) / base_mva;
}

Complex load_neutral_admittance_pu(
    const ThreePhaseLoad& load,
    LoadNeutralKind neutral_kind,
    const ThreePhaseACBus& bus,
    double base_mva) {
  if (neutral_kind != LoadNeutralKind::ImpedanceGrounded) {
    return {0.0, 0.0};
  }
  const Complex z_neutral_ohm{load.r_neut_ohm, load.x_neut_ohm};
  const double z_base_ohm = three_phase_bus_base_impedance_ohm(bus, base_mva);
  const Complex z_neutral_pu = z_neutral_ohm / z_base_ohm;
  if (std::abs(z_neutral_pu) <= kPhaseValueTol) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load.index) +
        " has singular neutral impedance");
  }
  return Complex(1.0, 0.0) / z_neutral_pu;
}

Complex nominal_phase_terminal_voltage_pu(int phase_index) {
  if (phase_index == 0) return {1.0, 0.0};
  if (phase_index == 1) return std::polar(1.0, -2.0 * kPi / 3.0);
  return std::polar(1.0, 2.0 * kPi / 3.0);
}

ComplexVectorX resolve_wye_branch_shunt_admittances(
    const std::vector<int>& phase_indices,
    const ComplexVectorX& terminal_shunt_pu,
    LoadNeutralKind neutral_kind,
    Complex y_neutral_pu,
    int load_index) {
  if (neutral_kind != LoadNeutralKind::ImpedanceGrounded) {
    return terminal_shunt_pu;
  }
  if (std::abs(y_neutral_pu) <= kPhaseValueTol) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load_index) +
        " has impedance-grounded constant-Z load with singular neutral admittance");
  }

  ComplexVectorX branch_shunt = terminal_shunt_pu;
  Complex total_terminal_current{0.0, 0.0};
  for (int local = 0; local < terminal_shunt_pu.size(); ++local) {
    const Complex nominal_voltage =
        nominal_phase_terminal_voltage_pu(phase_indices[static_cast<size_t>(local)]);
    total_terminal_current += terminal_shunt_pu(local) * nominal_voltage;
  }
  const Complex neutral_voltage = total_terminal_current / y_neutral_pu;

  for (int local = 0; local < terminal_shunt_pu.size(); ++local) {
    if (std::abs(terminal_shunt_pu(local)) <= kPhaseValueTol) {
      branch_shunt(local) = Complex(0.0, 0.0);
      continue;
    }
    const Complex nominal_voltage =
        nominal_phase_terminal_voltage_pu(phase_indices[static_cast<size_t>(local)]);
    const Complex branch_voltage = nominal_voltage - neutral_voltage;
    if (std::abs(branch_voltage) <= kPhaseValueTol) {
      throw std::runtime_error(
          "three-phase load " + std::to_string(load_index) +
          " produced singular branch voltage during impedance-grounded constant-Z reduction");
    }
    branch_shunt(local) =
        terminal_shunt_pu(local) * nominal_voltage / branch_voltage;
  }
  return branch_shunt;
}

ComplexMatrixX reduce_wye_shunt_admittance(
    const ComplexVectorX& phase_admittances,
    LoadNeutralKind neutral_kind,
    Complex y_neutral_pu,
    int load_index) {
  const int phase_count = static_cast<int>(phase_admittances.size());
  ComplexMatrixX reduced = ComplexMatrixX::Zero(phase_count, phase_count);
  if (phase_count == 0) return reduced;

  if (neutral_kind == LoadNeutralKind::SolidGrounded) {
    reduced.diagonal() = phase_admittances;
    return reduced;
  }

  const Complex denominator = phase_admittances.sum() + y_neutral_pu;
  if (std::abs(denominator) <= kPhaseValueTol) {
    if (phase_admittances.cwiseAbs().maxCoeff() <= kPhaseValueTol) {
      return reduced;
    }
    throw std::runtime_error(
        "three-phase load " + std::to_string(load_index) +
        " produced singular local-neutral reduction");
  }

  reduced = phase_admittances.asDiagonal();
  reduced -= (phase_admittances * phase_admittances.transpose()) / denominator;
  return reduced;
}

PreparedLoadSet prepare_load_models(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<int, int>& id_map,
    const PhaseNodeIndexer& indexer,
    double base_mva) {
  PreparedLoadSet prepared;

  for (const auto& load : sys.loads) {
    if (!load.in_service) continue;
    require_mask_not_empty("three-phase load", load.index, load.phase_mask);
    require_phase_values_on_mask(
        "three-phase load",
        load.index,
        load.phase_mask,
        {load.p_a_mw, load.p_b_mw, load.p_c_mw},
        {load.q_a_mvar, load.q_b_mvar, load.q_c_mvar});
    validate_load_voltage_contract(load);
    const LoadZipWeights p_zip = resolve_active_zip_weights(load);
    const LoadZipWeights q_zip = resolve_reactive_zip_weights(load);

    auto it = id_map.find(load.bus);
    if (it == id_map.end()) continue;
    const int bus_offset = it->second;
    if (!indexer.bus_phase_mask(bus_offset).contains(load.phase_mask)) {
      throw std::runtime_error(
          "three-phase load " + std::to_string(load.index) +
          " phase_mask is not a subset of the attached bus phase_mask");
    }

    const std::string connection = lowercase_copy(load.connection);
    if (connection == "wye") {
      const LoadNeutralKind neutral_kind = resolve_load_neutral_kind(load);
      const auto& bus = sys.buses[bus_offset];
      const std::array<double, 3> p_values = {load.p_a_mw, load.p_b_mw, load.p_c_mw};
      const std::array<double, 3> q_values = {load.q_a_mvar, load.q_b_mvar, load.q_c_mvar};
      PreparedWyeLoadContribution contribution;
      contribution.load_index = load.index;
      contribution.load_name = load.name;
      contribution.bus_offset = bus_offset;
      contribution.neutral_kind = neutral_kind;
      contribution.vmin_pu = load.vmin_pu;
      contribution.vmax_pu = load.vmax_pu;
      contribution.zipv_cutoff_pu = load.zipv_cutoff_pu;
      contribution.y_neutral_pu =
          load_neutral_admittance_pu(load, neutral_kind, bus, base_mva);

      const int phase_count = load.phase_mask.count();
      contribution.phase_indices.reserve(phase_count);
      contribution.nodes.reserve(phase_count);
      contribution.y_shunt_pu = ComplexVectorX::Zero(phase_count);
      contribution.s_current_nominal_pu = ComplexVectorX::Zero(phase_count);
      contribution.s_power_nominal_pu = ComplexVectorX::Zero(phase_count);
      ComplexVectorX terminal_shunt_pu = ComplexVectorX::Zero(phase_count);

      int local = 0;
      for (int phase = 0; phase < 3; ++phase) {
        if (!load.phase_mask.has(phase)) continue;
        const Complex nominal =
            load_nominal_power_pu(p_values[phase], q_values[phase], base_mva);
        contribution.phase_indices.push_back(phase);
        contribution.nodes.push_back(indexer.node_index(bus_offset, phase));

        const Complex s_z{nominal.real() * p_zip.z_fraction,
                          nominal.imag() * q_zip.z_fraction};
        terminal_shunt_pu(local) = {s_z.real(), -s_z.imag()};
        contribution.s_current_nominal_pu(local) = {
            nominal.real() * p_zip.i_fraction,
            nominal.imag() * q_zip.i_fraction,
        };
        contribution.s_power_nominal_pu(local) = {
            nominal.real() * p_zip.p_fraction,
            nominal.imag() * q_zip.p_fraction,
        };
        ++local;
      }

      contribution.y_shunt_pu =
          resolve_wye_branch_shunt_admittances(
              contribution.phase_indices,
              terminal_shunt_pu,
              neutral_kind,
              contribution.y_neutral_pu,
              load.index);

      contribution.reduced_y_shunt_pu =
          reduce_wye_shunt_admittance(
              contribution.y_shunt_pu,
              neutral_kind,
              contribution.y_neutral_pu,
              load.index);
      prepared.wye_loads.push_back(std::move(contribution));
      continue;
    }

    if (connection != "delta") {
      throw std::runtime_error(
          "three-phase load " + std::to_string(load.index) +
          " has unsupported connection=" + load.connection);
    }
    if (load.phase_mask.count() < 2) {
      throw std::runtime_error(
          "three-phase delta load " + std::to_string(load.index) +
          " requires at least two active phases");
    }

    auto add_delta_branch =
        [&](int from_phase,
            int to_phase,
            double p_mw,
            double q_mvar) {
          if (!load.phase_mask.has(from_phase) || !load.phase_mask.has(to_phase)) {
            return;
          }
          const Complex nominal =
              load_nominal_power_pu(p_mw, q_mvar, base_mva);
          PreparedBranchLoadContribution contribution;
          contribution.load_index = load.index;
          contribution.load_name = load.name;
          contribution.bus_offset = bus_offset;
          contribution.from_phase = from_phase;
          contribution.to_phase = to_phase;
          contribution.from_node = indexer.node_index(bus_offset, from_phase);
          contribution.to_node = indexer.node_index(bus_offset, to_phase);
          const Complex s_z{nominal.real() * p_zip.z_fraction,
                            nominal.imag() * q_zip.z_fraction};
          contribution.y_branch_pu = {s_z.real() / 3.0, -s_z.imag() / 3.0};
          contribution.s_current_nominal_pu = {
              nominal.real() * p_zip.i_fraction,
              nominal.imag() * q_zip.i_fraction,
          };
          contribution.s_power_nominal_pu = {
              nominal.real() * p_zip.p_fraction,
              nominal.imag() * q_zip.p_fraction,
          };
          contribution.vmin_pu = load.vmin_pu;
          contribution.vmax_pu = load.vmax_pu;
          contribution.zipv_cutoff_pu = load.zipv_cutoff_pu;
          prepared.branch_loads.push_back(std::move(contribution));
        };

    const std::array<double, 3> p_values = {load.p_a_mw, load.p_b_mw, load.p_c_mw};
    const std::array<double, 3> q_values = {load.q_a_mvar, load.q_b_mvar, load.q_c_mvar};

    if (load.phase_mask.bits == PhaseMask::abc().bits) {
      add_delta_branch(0, 1, p_values[0], q_values[0]);
      add_delta_branch(1, 2, p_values[1], q_values[1]);
      add_delta_branch(2, 0, p_values[2], q_values[2]);
    } else if (load.phase_mask.bits == PhaseMask::ab().bits) {
      add_delta_branch(0, 1, p_values[0] + p_values[1], q_values[0] + q_values[1]);
    } else if (load.phase_mask.bits == PhaseMask::bc().bits) {
      add_delta_branch(1, 2, p_values[1] + p_values[2], q_values[1] + q_values[2]);
    } else if (load.phase_mask.bits == PhaseMask::ac().bits) {
      add_delta_branch(2, 0, p_values[2] + p_values[0], q_values[2] + q_values[0]);
    } else {
      throw std::runtime_error(
          "three-phase delta load " + std::to_string(load.index) +
          " has unsupported phase_mask for delta semantics");
    }
  }

  return prepared;
}

void stamp_prepared_constant_impedance_loads(
    std::vector<ComplexTriplet>& triplets,
    const PreparedLoadSet& prepared_loads) {
  for (const auto& load : prepared_loads.wye_loads) {
    for (int row = 0; row < load.reduced_y_shunt_pu.rows(); ++row) {
      for (int col = 0; col < load.reduced_y_shunt_pu.cols(); ++col) {
        const Complex value = load.reduced_y_shunt_pu(row, col);
        if (std::abs(value) <= kPhaseValueTol) continue;
        triplets.emplace_back(load.nodes[row], load.nodes[col], value);
      }
    }
  }

  for (const auto& load : prepared_loads.branch_loads) {
    if (std::abs(load.y_branch_pu) <= kPhaseValueTol) continue;
    triplets.emplace_back(load.from_node, load.from_node, load.y_branch_pu);
    triplets.emplace_back(load.to_node, load.to_node, load.y_branch_pu);
    triplets.emplace_back(load.from_node, load.to_node, -load.y_branch_pu);
    triplets.emplace_back(load.to_node, load.from_node, -load.y_branch_pu);
  }
}

struct DynamicLoadPowerEvaluation {
  Complex s_dynamic_pu{0.0, 0.0};
  Complex ds_dv_pu{0.0, 0.0};
};

struct WyeDynamicBranchState {
  DynamicLoadPowerEvaluation power;
  Complex current_pu{0.0, 0.0};
};

struct LocalNeutralEquationEvaluation {
  Complex residual{0.0, 0.0};
  Eigen::Matrix2d jacobian = Eigen::Matrix2d::Zero();
  ComplexVectorX branch_voltages;
  std::vector<WyeDynamicBranchState> dynamic_branches;
};

struct LocalNeutralSolution {
  Complex neutral_voltage_pu{0.0, 0.0};
  Eigen::Matrix2d jacobian = Eigen::Matrix2d::Identity();
  ComplexVectorX branch_voltages;
  std::vector<WyeDynamicBranchState> dynamic_branches;
};

DynamicLoadPowerEvaluation evaluate_dynamic_load_power(
    double v_pu,
    double vmin_pu,
    double vmax_pu,
    double cutoff_pu,
    Complex s_current_nominal_pu,
    Complex s_power_nominal_pu) {
  DynamicLoadPowerEvaluation evaluation;
  if (cutoff_pu > kPhaseValueTol && v_pu < cutoff_pu) {
    return evaluation;
  }

  double current_scale = 0.0;
  double power_scale = 0.0;
  double d_current_scale = 0.0;
  double d_power_scale = 0.0;

  if (v_pu < vmin_pu) {
    current_scale = (v_pu * v_pu) / vmin_pu;
    power_scale = (v_pu * v_pu) / (vmin_pu * vmin_pu);
    d_current_scale = 2.0 * v_pu / vmin_pu;
    d_power_scale = 2.0 * v_pu / (vmin_pu * vmin_pu);
  } else if (v_pu > vmax_pu) {
    current_scale = (v_pu * v_pu) / vmax_pu;
    power_scale = (v_pu * v_pu) / (vmax_pu * vmax_pu);
    d_current_scale = 2.0 * v_pu / vmax_pu;
    d_power_scale = 2.0 * v_pu / (vmax_pu * vmax_pu);
  } else {
    current_scale = v_pu;
    power_scale = 1.0;
    d_current_scale = 1.0;
    d_power_scale = 0.0;
  }

  evaluation.s_dynamic_pu =
      s_current_nominal_pu * current_scale +
      s_power_nominal_pu * power_scale;
  evaluation.ds_dv_pu =
      s_current_nominal_pu * d_current_scale +
      s_power_nominal_pu * d_power_scale;
  return evaluation;
}

Complex dynamic_branch_current_from_power(
    Complex branch_voltage,
    const DynamicLoadPowerEvaluation& evaluation) {
  if (std::abs(branch_voltage) <= 1e-9) {
    return Complex(0.0, 0.0);
  }
  return std::conj(evaluation.s_dynamic_pu) / std::conj(branch_voltage);
}

Complex dynamic_branch_current_derivative(
    Complex branch_voltage,
    const DynamicLoadPowerEvaluation& evaluation,
    Complex d_branch_voltage) {
  const double branch_voltage_abs = std::abs(branch_voltage);
  if (branch_voltage_abs <= 1e-9) {
    return Complex(0.0, 0.0);
  }

  const double dm =
      std::real(std::conj(branch_voltage) * d_branch_voltage) / branch_voltage_abs;
  const Complex c_value = std::conj(evaluation.s_dynamic_pu);
  const Complex dc_dm = std::conj(evaluation.ds_dv_pu);
  return (dc_dm * dm) / std::conj(branch_voltage) -
         c_value * std::conj(d_branch_voltage) /
             (std::conj(branch_voltage) * std::conj(branch_voltage));
}

Complex solve_local_neutral_linearized(
    const Eigen::Matrix2d& jacobian,
    Complex rhs,
    int load_index) {
  Eigen::FullPivLU<Eigen::Matrix2d> lu(jacobian);
  if (!lu.isInvertible()) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load_index) +
        " produced singular local-neutral Jacobian");
  }
  const Eigen::Vector2d solution =
      lu.solve(Eigen::Vector2d(rhs.real(), rhs.imag()));
  return {solution[0], solution[1]};
}

double local_neutral_residual_norm(Complex residual) {
  return std::max(std::abs(residual.real()), std::abs(residual.imag()));
}

int count_effective_wye_branches(const PreparedWyeLoadContribution& load) {
  int count = 0;
  for (int local = 0; local < load.y_shunt_pu.size(); ++local) {
    if (std::abs(load.y_shunt_pu(local)) > kPhaseValueTol ||
        std::abs(load.s_current_nominal_pu(local)) > kPhaseValueTol ||
        std::abs(load.s_power_nominal_pu(local)) > kPhaseValueTol) {
      ++count;
    }
  }
  return count;
}

int first_effective_wye_branch(const PreparedWyeLoadContribution& load) {
  for (int local = 0; local < load.y_shunt_pu.size(); ++local) {
    if (std::abs(load.y_shunt_pu(local)) > kPhaseValueTol ||
        std::abs(load.s_current_nominal_pu(local)) > kPhaseValueTol ||
        std::abs(load.s_power_nominal_pu(local)) > kPhaseValueTol) {
      return local;
    }
  }
  return -1;
}

Complex solve_single_branch_impedance_grounded_wye(
    const PreparedWyeLoadContribution& load,
    const ComplexVectorX& phase_voltages) {
  const int local = first_effective_wye_branch(load);
  if (local < 0) {
    return Complex(0.0, 0.0);
  }

  const Complex phase_voltage = phase_voltages(local);
  Complex branch_voltage = phase_voltage;
  const Complex denominator = load.y_neutral_pu + load.y_shunt_pu(local);
  if (std::abs(denominator) > kPhaseValueTol) {
    branch_voltage = (load.y_neutral_pu / denominator) * phase_voltage;
  }

  for (int iter = 0; iter < 40; ++iter) {
    const DynamicLoadPowerEvaluation power =
        evaluate_dynamic_load_power(
            std::abs(branch_voltage),
            load.vmin_pu,
            load.vmax_pu,
            load.zipv_cutoff_pu,
            load.s_current_nominal_pu(local),
            load.s_power_nominal_pu(local));
    const Complex dynamic_current =
        dynamic_branch_current_from_power(branch_voltage, power);
    const Complex residual =
        load.y_neutral_pu * (phase_voltage - branch_voltage) -
        load.y_shunt_pu(local) * branch_voltage -
        dynamic_current;
    if (local_neutral_residual_norm(residual) < 1e-10) {
      return phase_voltage - branch_voltage;
    }

    Eigen::Matrix2d jacobian = Eigen::Matrix2d::Zero();
    const Complex dr_dre =
        -(load.y_neutral_pu + load.y_shunt_pu(local)) -
        dynamic_branch_current_derivative(
            branch_voltage,
            power,
            Complex(1.0, 0.0));
    const Complex dr_dim =
        -(load.y_neutral_pu + load.y_shunt_pu(local)) * Complex(0.0, 1.0) -
        dynamic_branch_current_derivative(
            branch_voltage,
            power,
            Complex(0.0, 1.0));
    jacobian(0, 0) = dr_dre.real();
    jacobian(1, 0) = dr_dre.imag();
    jacobian(0, 1) = dr_dim.real();
    jacobian(1, 1) = dr_dim.imag();

    const Complex delta = solve_local_neutral_linearized(
        jacobian,
        -residual,
        load.load_index);
    double step = 1.0;
    bool accepted = false;
    while (step >= 1.0 / 128.0) {
      const Complex candidate = branch_voltage + step * delta;
      const DynamicLoadPowerEvaluation candidate_power =
          evaluate_dynamic_load_power(
              std::abs(candidate),
              load.vmin_pu,
              load.vmax_pu,
              load.zipv_cutoff_pu,
              load.s_current_nominal_pu(local),
              load.s_power_nominal_pu(local));
      const Complex candidate_current =
          dynamic_branch_current_from_power(candidate, candidate_power);
      const Complex candidate_residual =
          load.y_neutral_pu * (phase_voltage - candidate) -
          load.y_shunt_pu(local) * candidate -
          candidate_current;
      if (local_neutral_residual_norm(candidate_residual) <
          local_neutral_residual_norm(residual)) {
        branch_voltage = candidate;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) {
      branch_voltage += delta;
    }
  }

  throw std::runtime_error(
      "three-phase load " + std::to_string(load.load_index) +
      " failed to solve single-branch impedance-grounded local-neutral state");
}

ComplexVectorX collect_wye_phase_voltages(
    const PreparedWyeLoadContribution& load,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va) {
  ComplexVectorX voltages(static_cast<int>(load.nodes.size()));
  for (int local = 0; local < static_cast<int>(load.nodes.size()); ++local) {
    const int node = load.nodes[local];
    voltages(local) = std::polar(vm[node], va[node]);
  }
  return voltages;
}

Complex initial_neutral_voltage_guess(
    const PreparedWyeLoadContribution& load,
    const ComplexVectorX& phase_voltages) {
  if (load.neutral_kind == LoadNeutralKind::SolidGrounded) {
    return {0.0, 0.0};
  }
  if (load.neutral_kind == LoadNeutralKind::OpenNeutral &&
      phase_voltages.size() == 1) {
    return phase_voltages(0);
  }

  const Complex denominator = load.y_shunt_pu.sum() + load.y_neutral_pu;
  if (std::abs(denominator) > kPhaseValueTol) {
    Complex weighted{0.0, 0.0};
    for (int local = 0; local < phase_voltages.size(); ++local) {
      weighted += load.y_shunt_pu(local) * phase_voltages(local);
    }
    return weighted / denominator;
  }

  Complex average{0.0, 0.0};
  for (int local = 0; local < phase_voltages.size(); ++local) {
    average += phase_voltages(local);
  }
  return average / static_cast<double>(phase_voltages.size());
}

LocalNeutralEquationEvaluation evaluate_local_neutral_equation(
    const PreparedWyeLoadContribution& load,
    const ComplexVectorX& phase_voltages,
    Complex neutral_voltage) {
  LocalNeutralEquationEvaluation evaluation;
  const int phase_count = phase_voltages.size();
  evaluation.branch_voltages = ComplexVectorX::Zero(phase_count);
  evaluation.dynamic_branches.resize(static_cast<size_t>(phase_count));
  evaluation.residual = load.y_neutral_pu * neutral_voltage;

  for (int local = 0; local < phase_count; ++local) {
    const Complex branch_voltage = phase_voltages(local) - neutral_voltage;
    evaluation.branch_voltages(local) = branch_voltage;

    auto& dynamic = evaluation.dynamic_branches[static_cast<size_t>(local)];
    dynamic.power = evaluate_dynamic_load_power(
        std::abs(branch_voltage),
        load.vmin_pu,
        load.vmax_pu,
        load.zipv_cutoff_pu,
        load.s_current_nominal_pu(local),
        load.s_power_nominal_pu(local));
    dynamic.current_pu =
        dynamic_branch_current_from_power(branch_voltage, dynamic.power);

    evaluation.residual -= load.y_shunt_pu(local) * branch_voltage;
    evaluation.residual -= dynamic.current_pu;

    const Complex dg_dre =
        load.y_neutral_pu -
        load.y_shunt_pu(local) * Complex(-1.0, 0.0) -
        dynamic_branch_current_derivative(
            branch_voltage,
            dynamic.power,
            Complex(-1.0, 0.0));
    const Complex dg_dim =
        load.y_neutral_pu * Complex(0.0, 1.0) -
        load.y_shunt_pu(local) * Complex(0.0, -1.0) -
        dynamic_branch_current_derivative(
            branch_voltage,
            dynamic.power,
            Complex(0.0, -1.0));

    evaluation.jacobian(0, 0) += dg_dre.real();
    evaluation.jacobian(1, 0) += dg_dre.imag();
    evaluation.jacobian(0, 1) += dg_dim.real();
    evaluation.jacobian(1, 1) += dg_dim.imag();
  }

  return evaluation;
}

LocalNeutralSolution solve_local_neutral_state(
    const PreparedWyeLoadContribution& load,
    const ComplexVectorX& phase_voltages) {
  LocalNeutralSolution solution;
  const int phase_count = phase_voltages.size();
  solution.branch_voltages = ComplexVectorX::Zero(phase_count);
  solution.dynamic_branches.resize(static_cast<size_t>(phase_count));

  if (load.neutral_kind == LoadNeutralKind::SolidGrounded) {
    const auto evaluation =
        evaluate_local_neutral_equation(load, phase_voltages, Complex(0.0, 0.0));
    solution.neutral_voltage_pu = {0.0, 0.0};
    solution.jacobian = Eigen::Matrix2d::Identity();
    solution.branch_voltages = evaluation.branch_voltages;
    solution.dynamic_branches = evaluation.dynamic_branches;
    return solution;
  }

  if (load.neutral_kind == LoadNeutralKind::OpenNeutral && phase_count == 1) {
    solution.neutral_voltage_pu = phase_voltages(0);
    solution.jacobian = Eigen::Matrix2d::Identity();
    solution.branch_voltages(0) = Complex(0.0, 0.0);
    return solution;
  }

  if (load.neutral_kind == LoadNeutralKind::OpenNeutral &&
      count_effective_wye_branches(load) <= 1) {
    const int local = first_effective_wye_branch(load);
    if (local >= 0) {
      solution.neutral_voltage_pu = phase_voltages(local);
    } else {
      solution.neutral_voltage_pu = initial_neutral_voltage_guess(load, phase_voltages);
    }
    solution.jacobian = Eigen::Matrix2d::Identity();
    for (int idx = 0; idx < phase_count; ++idx) {
      solution.branch_voltages(idx) = phase_voltages(idx) - solution.neutral_voltage_pu;
    }
    return solution;
  }

  if (load.neutral_kind == LoadNeutralKind::ImpedanceGrounded &&
      count_effective_wye_branches(load) == 1) {
    const Complex neutral_voltage =
        solve_single_branch_impedance_grounded_wye(load, phase_voltages);
    const auto evaluation =
        evaluate_local_neutral_equation(load, phase_voltages, neutral_voltage);
    solution.neutral_voltage_pu = neutral_voltage;
    solution.jacobian = evaluation.jacobian;
    solution.branch_voltages = evaluation.branch_voltages;
    solution.dynamic_branches = evaluation.dynamic_branches;
    return solution;
  }

  Complex neutral_voltage = initial_neutral_voltage_guess(load, phase_voltages);
  for (int iter = 0; iter < 40; ++iter) {
    const auto evaluation =
        evaluate_local_neutral_equation(load, phase_voltages, neutral_voltage);
    if (local_neutral_residual_norm(evaluation.residual) < 1e-10) {
      solution.neutral_voltage_pu = neutral_voltage;
      solution.jacobian = evaluation.jacobian;
      solution.branch_voltages = evaluation.branch_voltages;
      solution.dynamic_branches = evaluation.dynamic_branches;
      return solution;
    }

    const Complex delta = solve_local_neutral_linearized(
        evaluation.jacobian,
        -evaluation.residual,
        load.load_index);
    double step = 1.0;
    bool accepted = false;
    while (step >= 1.0 / 128.0) {
      const Complex candidate = neutral_voltage + step * delta;
      const auto candidate_evaluation =
          evaluate_local_neutral_equation(load, phase_voltages, candidate);
      if (local_neutral_residual_norm(candidate_evaluation.residual) <
          local_neutral_residual_norm(evaluation.residual)) {
        neutral_voltage = candidate;
        accepted = true;
        break;
      }
      step *= 0.5;
    }
    if (!accepted) {
      neutral_voltage += delta;
    }
  }

  const auto final_evaluation =
      evaluate_local_neutral_equation(load, phase_voltages, neutral_voltage);
  if (local_neutral_residual_norm(final_evaluation.residual) >= 1e-9) {
    throw std::runtime_error(
        "three-phase load " + std::to_string(load.load_index) +
        " failed to solve local-neutral state");
  }

  solution.neutral_voltage_pu = neutral_voltage;
  solution.jacobian = final_evaluation.jacobian;
  solution.branch_voltages = final_evaluation.branch_voltages;
  solution.dynamic_branches = final_evaluation.dynamic_branches;
  return solution;
}

PhaseSpec compute_phase_spec(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<int, int>& id_map,
    const PhaseNodeIndexer& indexer,
    double base_mva,
    double load_scale = 1.0) {

  PhaseSpec spec;
  spec.p_spec = Eigen::VectorXd::Zero(indexer.total_nodes);
  spec.q_spec = Eigen::VectorXd::Zero(indexer.total_nodes);

  // Bus-level demand (load positive → subtract from injection)
  for (size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    const auto& bus = sys.buses[bus_offset];
    if (!bus.in_service) continue;
    stamp_injection(
        spec.p_spec,
        spec.q_spec,
        indexer,
        static_cast<int>(bus_offset),
        bus.phase_mask,
        {bus.pd_a_mw * load_scale,
         bus.pd_b_mw * load_scale,
         bus.pd_c_mw * load_scale},
        {bus.qd_a_mvar * load_scale,
         bus.qd_b_mvar * load_scale,
         bus.qd_c_mvar * load_scale},
        base_mva,
        -1.0);
  }

  // Generator injection
  for (const auto& g : sys.generators) {
    if (!g.in_service) continue;
    validate_generator_dispatch_semantics(g);
    auto it = id_map.find(g.bus);
    if (it == id_map.end()) continue;
    const int gi = it->second;
    const auto& bus = sys.buses[gi];
    if (bus.bus_type == BusType::SLACK) continue;  // slack bus P is free
    if (generator_has_per_phase_dispatch(g)) {
      stamp_injection(
          spec.p_spec,
          spec.q_spec,
          indexer,
          gi,
          g.phase_mask,
          {g.p_a_mw, g.p_b_mw, g.p_c_mw},
          {g.q_a_mvar, g.q_b_mvar, g.q_c_mvar},
          base_mva,
          1.0);
      continue;
    }

    const int phase_count = g.phase_mask.count();
    const double p_per_phase = g.p_mw / static_cast<double>(phase_count);
    const double q_per_phase = g.q_mvar / static_cast<double>(phase_count);
    stamp_injection(
        spec.p_spec,
        spec.q_spec,
        indexer,
        gi,
        g.phase_mask,
        {g.phase_mask.has(0) ? p_per_phase : 0.0,
         g.phase_mask.has(1) ? p_per_phase : 0.0,
         g.phase_mask.has(2) ? p_per_phase : 0.0},
        {g.phase_mask.has(0) ? q_per_phase : 0.0,
         g.phase_mask.has(1) ? q_per_phase : 0.0,
         g.phase_mask.has(2) ? q_per_phase : 0.0},
        base_mva,
        1.0);
  }

  return spec;
}

// =========================================================================
// Build Jacobian context for compact phase-node NR.
//
// For SLACK bus: all active phase angles and magnitudes are fixed.
// For PQ bus:    all active phase angles and magnitudes are unknowns.
// For PV bus:    all active phase angles are unknowns, Vm fixed.
// =========================================================================
struct ThreePhaseNRContext {
  int node_count{0};    // total compact phase nodes
  int np{0};            // number of angle unknowns
  int nq{0};            // number of Vm unknowns
  int nvar{0};          // np + nq

  std::vector<int> angle_nodes;  // phase-node indices with angle unknowns
  std::vector<int> vm_nodes;     // phase-node indices with Vm unknowns

  // Reverse maps: phase_node → position in angle_nodes / vm_nodes (-1 if not)
  std::vector<int> angle_pos;
  std::vector<int> vm_pos;
};

ThreePhaseNRContext build_tp_nr_context(
    const ThreePhaseACSystem& sys,
    const PhaseNodeIndexer& indexer) {
  ThreePhaseNRContext ctx;
  ctx.node_count = indexer.total_nodes;
  ctx.angle_pos.assign(indexer.total_nodes, -1);
  ctx.vm_pos.assign(indexer.total_nodes, -1);

  for (size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    const auto& bus = sys.buses[bus_offset];
    if (bus.bus_type == BusType::SLACK) {
      // Slack: all active phases have fixed angle and Vm.
      continue;
    }
    const PhaseMask mask = indexer.bus_phase_mask(static_cast<int>(bus_offset));
    for (int phase = 0; phase < 3; ++phase) {
      if (!mask.has(phase)) continue;
      const int node = indexer.node_index(static_cast<int>(bus_offset), phase);
      ctx.angle_pos[node] = static_cast<int>(ctx.angle_nodes.size());
      ctx.angle_nodes.push_back(node);
    }
    if (bus.bus_type == BusType::PQ) {
      for (int phase = 0; phase < 3; ++phase) {
        if (!mask.has(phase)) continue;
        const int node = indexer.node_index(static_cast<int>(bus_offset), phase);
        ctx.vm_pos[node] = static_cast<int>(ctx.vm_nodes.size());
        ctx.vm_nodes.push_back(node);
      }
    }
  }

  ctx.np = static_cast<int>(ctx.angle_nodes.size());
  ctx.nq = static_cast<int>(ctx.vm_nodes.size());
  ctx.nvar = ctx.np + ctx.nq;
  return ctx;
}

void add_power_injection_to_mismatch(
    const ThreePhaseNRContext& ctx,
    int node,
    Complex s_injection_pu,
    Eigen::VectorXd& mismatch) {
  const int p_row = ctx.angle_pos[node];
  if (p_row >= 0) {
    mismatch[p_row] += s_injection_pu.real();
  }
  const int q_row = ctx.vm_pos[node];
  if (q_row >= 0) {
    mismatch[ctx.np + q_row] += s_injection_pu.imag();
  }
}

void add_power_injection_derivative(
    const ThreePhaseNRContext& ctx,
    int row_node,
    int col_node,
    Complex ds_dx_pu,
    bool angle_column,
    std::vector<Eigen::Triplet<double>>& trips) {
  const int p_row = ctx.angle_pos[row_node];
  const int q_row = ctx.vm_pos[row_node];
  const int column =
      angle_column ? ctx.angle_pos[col_node]
                   : ctx.vm_pos[col_node];
  if (column < 0) return;
  const int actual_column = angle_column ? column : ctx.np + column;
  if (p_row >= 0 && std::abs(ds_dx_pu.real()) > kPhaseValueTol) {
    trips.emplace_back(p_row, actual_column, ds_dx_pu.real());
  }
  if (q_row >= 0 && std::abs(ds_dx_pu.imag()) > kPhaseValueTol) {
    trips.emplace_back(ctx.np + q_row, actual_column, ds_dx_pu.imag());
  }
}

Complex complex_state_voltage_partial(
    Complex voltage,
    double vm,
    bool angle_column,
    bool positive_sign);

void add_wye_load_contributions(
    const PreparedLoadSet& prepared_loads,
    const ThreePhaseNRContext& ctx,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    Eigen::VectorXd& mismatch,
    std::vector<Eigen::Triplet<double>>& trips) {
  for (const auto& load : prepared_loads.wye_loads) {
    const ComplexVectorX phase_voltages = collect_wye_phase_voltages(load, vm, va);
    const LocalNeutralSolution neutral_state =
        solve_local_neutral_state(load, phase_voltages);

    bool has_dynamic = false;
    for (size_t local = 0; local < load.nodes.size(); ++local) {
      const auto& dynamic = neutral_state.dynamic_branches[local];
      if (std::abs(dynamic.current_pu) > kPhaseValueTol ||
          std::abs(dynamic.power.s_dynamic_pu) > kPhaseValueTol ||
          std::abs(dynamic.power.ds_dv_pu) > kPhaseValueTol) {
        has_dynamic = true;
      }

      const Complex s_injection =
          -phase_voltages(static_cast<int>(local)) * std::conj(dynamic.current_pu);
      if (std::abs(s_injection) <= kPhaseValueTol) continue;
      add_power_injection_to_mismatch(
          ctx,
          load.nodes[local],
          s_injection,
          mismatch);
    }

    if (!has_dynamic) continue;

    auto apply_column =
        [&](int source_local, bool angle_column) {
          const int column_node = load.nodes[static_cast<size_t>(source_local)];
          const Complex d_phase_source = complex_state_voltage_partial(
              phase_voltages(source_local),
              vm[column_node],
              angle_column,
              true);

          const Complex dg_explicit =
              -load.y_shunt_pu(source_local) * d_phase_source -
              dynamic_branch_current_derivative(
                  neutral_state.branch_voltages(source_local),
                  neutral_state.dynamic_branches[static_cast<size_t>(source_local)].power,
                  d_phase_source);
          const Complex d_neutral = solve_local_neutral_linearized(
              neutral_state.jacobian,
              -dg_explicit,
              load.load_index);

          for (size_t local = 0; local < load.nodes.size(); ++local) {
            const bool same_phase = static_cast<int>(local) == source_local;
            const Complex d_phase =
                same_phase ? d_phase_source : Complex(0.0, 0.0);
            const Complex d_branch_voltage = d_phase - d_neutral;
            const Complex d_dynamic_current =
                dynamic_branch_current_derivative(
                    neutral_state.branch_voltages(static_cast<int>(local)),
                    neutral_state.dynamic_branches[local].power,
                    d_branch_voltage);
            const Complex ds =
                -d_phase * std::conj(neutral_state.dynamic_branches[local].current_pu) -
                phase_voltages(static_cast<int>(local)) *
                    std::conj(d_dynamic_current);
            add_power_injection_derivative(
                ctx,
                load.nodes[local],
                column_node,
                ds,
                angle_column,
                trips);
          }
        };

    for (int source_local = 0; source_local < static_cast<int>(load.nodes.size());
         ++source_local) {
      apply_column(source_local, true);
      apply_column(source_local, false);
    }
  }
}

Complex complex_state_voltage_partial(
    Complex voltage,
    double vm,
    bool angle_column,
    bool positive_sign) {
  if (angle_column) {
    return positive_sign ? Complex(0.0, 1.0) * voltage
                         : -Complex(0.0, 1.0) * voltage;
  }
  if (vm <= kPhaseValueTol) {
    return Complex(0.0, 0.0);
  }
  return positive_sign ? voltage / vm : -voltage / vm;
}

void add_delta_load_contributions(
    const PreparedLoadSet& prepared_loads,
    const ThreePhaseNRContext& ctx,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    Eigen::VectorXd& mismatch,
    std::vector<Eigen::Triplet<double>>& trips) {
  for (const auto& load : prepared_loads.branch_loads) {
    const Complex v_from = std::polar(vm[load.from_node], va[load.from_node]);
    const Complex v_to = std::polar(vm[load.to_node], va[load.to_node]);
    const Complex branch_voltage = v_from - v_to;
    const double branch_voltage_abs = std::abs(branch_voltage);
    if (branch_voltage_abs <= 1e-9) {
      continue;
    }

    const double branch_voltage_pu = branch_voltage_abs / std::sqrt(3.0);
    const DynamicLoadPowerEvaluation evaluation =
        evaluate_dynamic_load_power(
            branch_voltage_pu,
            load.vmin_pu,
            load.vmax_pu,
            load.zipv_cutoff_pu,
            load.s_current_nominal_pu,
            load.s_power_nominal_pu);
    if (std::abs(evaluation.s_dynamic_pu) <= kPhaseValueTol &&
        std::abs(evaluation.ds_dv_pu) <= kPhaseValueTol) {
      continue;
    }

    const Complex c_value = std::conj(evaluation.s_dynamic_pu);
    const Complex branch_current = c_value / std::conj(branch_voltage);
    const Complex s_injection_from = -v_from * std::conj(branch_current);
    const Complex s_injection_to = v_to * std::conj(branch_current);

    add_power_injection_to_mismatch(
        ctx,
        load.from_node,
        s_injection_from,
        mismatch);
    add_power_injection_to_mismatch(
        ctx,
        load.to_node,
        s_injection_to,
        mismatch);

    const Complex dc_dm = std::conj(evaluation.ds_dv_pu) / std::sqrt(3.0);

    auto apply_column =
        [&](int column_node, bool angle_column) {
          const bool same_from = column_node == load.from_node;
          const bool same_to = column_node == load.to_node;
          if (!same_from && !same_to) return;

          const Complex dv_from = same_from
                                      ? complex_state_voltage_partial(
                                            v_from,
                                            vm[load.from_node],
                                            angle_column,
                                            true)
                                      : Complex(0.0, 0.0);
          const Complex dv_to = same_to
                                    ? complex_state_voltage_partial(
                                          v_to,
                                          vm[load.to_node],
                                          angle_column,
                                          true)
                                    : Complex(0.0, 0.0);
          const Complex d_branch_voltage = dv_from - dv_to;
          const double dm =
              std::real(std::conj(branch_voltage) * d_branch_voltage) /
              branch_voltage_abs;
          const Complex d_branch_current =
              (dc_dm * dm) / std::conj(branch_voltage) -
              c_value * std::conj(d_branch_voltage) /
                  (std::conj(branch_voltage) * std::conj(branch_voltage));

          const Complex ds_from =
              -dv_from * std::conj(branch_current) -
              v_from * std::conj(d_branch_current);
          const Complex ds_to =
              dv_to * std::conj(branch_current) +
              v_to * std::conj(d_branch_current);

          add_power_injection_derivative(
              ctx,
              load.from_node,
              column_node,
              ds_from,
              angle_column,
              trips);
          add_power_injection_derivative(
              ctx,
              load.to_node,
              column_node,
              ds_to,
              angle_column,
              trips);
        };

    apply_column(load.from_node, true);
    apply_column(load.from_node, false);
    apply_column(load.to_node, true);
    apply_column(load.to_node, false);
  }
}

// =========================================================================
// Evaluate mismatch and build Jacobian (polar form)
//
// P_calc_φi = Σ_{j,ψ} |Vi_φ|·|Vj_ψ|·(G_φψ_ij·cos(θ_φi−θ_ψj) +
//                                       B_φψ_ij·sin(θ_φi−θ_ψj))
// Q_calc_φi = Σ_{j,ψ} |Vi_φ|·|Vj_ψ|·(G_φψ_ij·sin(θ_φi−θ_ψj) −
//                                       B_φψ_ij·cos(θ_φi−θ_ψj))
//
// Jacobian blocks (for i ≠ j phase-nodes):
//   ∂P_r/∂θ_s = |Vr|·|Vs|·(G_rs·sin(θr−θs) − B_rs·cos(θr−θs))
//   ∂P_r/∂|Vs|= |Vr|·(G_rs·cos(θr−θs) + B_rs·sin(θr−θs))
//   ∂Q_r/∂θ_s = −|Vr|·|Vs|·(G_rs·cos(θr−θs) + B_rs·sin(θr−θs))
//   ∂Q_r/∂|Vs|= |Vr|·(G_rs·sin(θr−θs) − B_rs·cos(θr−θs))
//
// Diagonal terms accumulate self-terms from all connected phases.
// =========================================================================
double evaluate_tp_mismatch_and_jacobian(
    const Eigen::SparseMatrix<std::complex<double>>& ybus,
    const ComplexVectorX& fixed_current,
    const ThreePhaseNRContext& ctx,
    const PhaseSpec& spec,
    const PreparedLoadSet& prepared_loads,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    Eigen::VectorXd& mismatch,
    Eigen::SparseMatrix<double>& jac) {

  const int node_count = ctx.node_count;
  const int np = ctx.np;
  const int nq = ctx.nq;
  const int nvar = ctx.nvar;

  // Compute P_calc, Q_calc for all phase nodes
  Eigen::VectorXd p_calc = Eigen::VectorXd::Zero(node_count);
  Eigen::VectorXd q_calc = Eigen::VectorXd::Zero(node_count);

  // Build Jacobian via triplets
  std::vector<Eigen::Triplet<double>> trips;
  trips.reserve(static_cast<size_t>(ybus.nonZeros() * 4));

  // Iterate over Ybus entries
  for (int col = 0; col < node_count; ++col) {
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(ybus, col); it; ++it) {
      const int r = static_cast<int>(it.row());  // phase node r
      const int s = static_cast<int>(it.col());  // phase node s

      const double g_rs = it.value().real();
      const double b_rs = it.value().imag();
      const double Vi = vm[r];
      const double Vj = vm[s];
      const double theta_rs = va[r] - va[s];
      const double cos_t = std::cos(theta_rs);
      const double sin_t = std::sin(theta_rs);

      // Power flow calculation
      p_calc[r] += Vi * Vj * (g_rs * cos_t + b_rs * sin_t);
      q_calc[r] += Vi * Vj * (g_rs * sin_t - b_rs * cos_t);

      if (r == s) {
        // Self terms contribute to diagonal Jacobian later
        // dP_r/dθ_r += −Vi*Vj*(G·sin − B·cos)  → accumulated from off-diag
        // But the diagonal self-derivative is different:
        //   dP_r/dθ_r = −Q_calc_r − Vi²·B_rr  (standard form)
        //   dP_r/dVm_r = P_calc_r/Vm_r + Vi·G_rr  (standard form)
        //   dQ_r/dθ_r = P_calc_r − Vi²·G_rr
        //   dQ_r/dVm_r = Q_calc_r/Vm_r − Vi·B_rr
        // We'll compute these after the full P,Q calculation.
        continue;
      }

      // Off-diagonal Jacobian entries
      const int a_pos_r = ctx.angle_pos[r];
      const int a_pos_s = ctx.angle_pos[s];
      const int v_pos_s = ctx.vm_pos[s];

      // dP_r/dθ_s = Vi·Vj·(G·sin − B·cos)
      if (a_pos_r >= 0 && a_pos_s >= 0) {
        trips.emplace_back(a_pos_r, a_pos_s,
            Vi * Vj * (g_rs * sin_t - b_rs * cos_t));
      }
      // dP_r/dVm_s = Vi·(G·cos + B·sin)
      if (a_pos_r >= 0 && v_pos_s >= 0) {
        trips.emplace_back(a_pos_r, np + v_pos_s,
            Vi * (g_rs * cos_t + b_rs * sin_t));
      }
      // dQ_r/dθ_s = −Vi·Vj·(G·cos + B·sin)
      if (a_pos_r >= 0 && a_pos_s >= 0) {
        // Q row is at np + vm_pos[r] but only if r is a Vm unknown (PQ bus)
        const int q_row = ctx.vm_pos[r];
        if (q_row >= 0) {
          trips.emplace_back(np + q_row, a_pos_s,
              -Vi * Vj * (g_rs * cos_t + b_rs * sin_t));
        }
      }
      // dQ_r/dVm_s = Vi·(G·sin − B·cos)
      {
        const int q_row = ctx.vm_pos[r];
        if (q_row >= 0 && v_pos_s >= 0) {
          trips.emplace_back(np + q_row, np + v_pos_s,
              Vi * (g_rs * sin_t - b_rs * cos_t));
        }
      }
    }
  }

  for (int node = 0; node < node_count; ++node) {
    const Complex current = fixed_current(node);
    if (std::abs(current) <= kPhaseValueTol) continue;
    const double Vi = vm[node];
    const double theta = va[node];
    const double g = current.real();
    const double b = current.imag();
    p_calc[node] += Vi * (g * std::cos(theta) + b * std::sin(theta));
    q_calc[node] += Vi * (g * std::sin(theta) - b * std::cos(theta));
  }

  // Now build mismatch and diagonal Jacobian entries
  mismatch.resize(nvar);
  double max_mismatch = 0.0;

  for (int k = 0; k < np; ++k) {
    const int r = ctx.angle_nodes[k];
    double dp = spec.p_spec[r] - p_calc[r];
    mismatch[k] = dp;
    max_mismatch = std::max(max_mismatch, std::abs(dp));
  }
  for (int k = 0; k < nq; ++k) {
    const int r = ctx.vm_nodes[k];
    double dq = spec.q_spec[r] - q_calc[r];
    mismatch[np + k] = dq;
    max_mismatch = std::max(max_mismatch, std::abs(dq));
  }

  add_wye_load_contributions(
      prepared_loads,
      ctx,
      vm,
      va,
      mismatch,
      trips);
  add_delta_load_contributions(
      prepared_loads,
      ctx,
      vm,
      va,
      mismatch,
      trips);

  if (mismatch.size() > 0) {
    max_mismatch = mismatch.cwiseAbs().maxCoeff();
  }

  // Diagonal Jacobian: use standard polar-form formulas
  for (int k = 0; k < np; ++k) {
    const int r = ctx.angle_nodes[k];
    // dP_r/dθ_r = −Q_calc_r − Vm_r² · B_rr
    // (where B_rr is the imaginary part of Ybus(r,r))
    double Brr = 0.0, Grr = 0.0;
    // Look up Ybus diagonal
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(ybus, r); it; ++it) {
      if (it.row() == r) {
        Grr = it.value().real();
        Brr = it.value().imag();
        break;
      }
    }
    double Vi = vm[r];

    // dP_r/dθ_r
    trips.emplace_back(k, k, -q_calc[r] - Vi * Vi * Brr);

    // dP_r/dVm_r (if r is also a Vm unknown, i.e. PQ bus)
    int vk = ctx.vm_pos[r];
    if (vk >= 0) {
      trips.emplace_back(k, np + vk,
          p_calc[r] / Vi + Vi * Grr);
    }

    // dQ_r/dθ_r (if r is PQ)
    if (vk >= 0) {
      trips.emplace_back(np + vk, k,
          p_calc[r] - Vi * Vi * Grr);
    }
  }

  // dQ_r/dVm_r diagonal
  for (int k = 0; k < nq; ++k) {
    const int r = ctx.vm_nodes[k];
    double Brr = 0.0;
    for (Eigen::SparseMatrix<std::complex<double>>::InnerIterator it(ybus, r); it; ++it) {
      if (it.row() == r) { Brr = it.value().imag(); break; }
    }
    double Vi = vm[r];
    trips.emplace_back(np + k, np + k,
        q_calc[r] / Vi - Vi * Brr);
  }

  jac.resize(nvar, nvar);
  jac.setFromTriplets(trips.begin(), trips.end(),
      [](const double& a, const double& b) { return a + b; });

  return max_mismatch;
}

// =========================================================================
// Compute VUF from per-phase voltage phasors
// =========================================================================
double compute_max_vuf(
    const PhaseNodeIndexer& indexer,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va) {
  const std::complex<double> a_op = std::polar(1.0, 2.0 * kPi / 3.0);
  const std::complex<double> a2_op = std::polar(1.0, 4.0 * kPi / 3.0);
  double max_vuf = 0.0;
  for (int bus_offset = 0;
       bus_offset < static_cast<int>(indexer.bus_phase_masks.size());
       ++bus_offset) {
    if (indexer.bus_phase_mask(bus_offset).bits != PhaseMask::abc().bits) continue;
    std::complex<double> Va = phase_phasor(vm, va, indexer, bus_offset, 0);
    std::complex<double> Vb = phase_phasor(vm, va, indexer, bus_offset, 1);
    std::complex<double> Vc = phase_phasor(vm, va, indexer, bus_offset, 2);
    std::complex<double> V_pos = (Va + a_op * Vb + a2_op * Vc) / 3.0;
    std::complex<double> V_neg = (Va + a2_op * Vb + a_op * Vc) / 3.0;
    double vpos_mag = std::abs(V_pos);
    if (vpos_mag > 1e-12) {
      max_vuf = std::max(max_vuf, std::abs(V_neg) / vpos_mag * 100.0);
    }
  }
  return max_vuf;
}

void set_phasor_observation_phase(
    ThreePhasePhasorObservation& observation,
    int phase_index,
    Complex value) {
  observation.real[static_cast<size_t>(phase_index)] = value.real();
  observation.imag[static_cast<size_t>(phase_index)] = value.imag();
}

Complex phase_from_observation(
    const ThreePhasePhasorObservation& observation,
    int phase_index) {
  return {
      observation.real[static_cast<size_t>(phase_index)],
      observation.imag[static_cast<size_t>(phase_index)],
  };
}

Complex phase_from_bus_voltage_result(
    const ThreePhaseBusVoltage& voltage,
    int phase_index) {
  if (phase_index == 0) {
    return std::polar(voltage.vm_a_pu, voltage.va_a_deg * kPi / 180.0);
  }
  if (phase_index == 1) {
    return std::polar(voltage.vm_b_pu, voltage.va_b_deg * kPi / 180.0);
  }
  return std::polar(voltage.vm_c_pu, voltage.va_c_deg * kPi / 180.0);
}

std::vector<ThreePhaseTransformerTerminalObservation> build_transformer_terminal_observations(
    const std::vector<TransformerPrimitiveRecord>& transformer_primitives,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va) {
  std::vector<ThreePhaseTransformerTerminalObservation> observations;
  observations.reserve(transformer_primitives.size());
  for (const auto& primitive : transformer_primitives) {
    const PrimitiveBranchObservation measurement =
        observe_primitive_branch(primitive.primitive, vm, va);

    ThreePhaseTransformerTerminalObservation observation;
    observation.transformer_index = primitive.transformer_index;
    observation.transformer_name = primitive.transformer_name;
    observation.hv_bus = primitive.hv_bus;
    observation.lv_bus = primitive.lv_bus;
    observation.hv_phase_mask = primitive.hv_phase_mask;
    observation.lv_phase_mask = primitive.lv_phase_mask;

    for (int local = 0;
         local < static_cast<int>(primitive.primitive.from_terminal.phases.size());
         ++local) {
      const int phase =
          primitive.primitive.from_terminal.phases[static_cast<size_t>(local)];
      set_phasor_observation_phase(
          observation.hv_voltage_pu,
          phase,
          measurement.from_voltage(local));
      set_phasor_observation_phase(
          observation.hv_current_amps,
          phase,
          measurement.from_current(local) * primitive.hv_base_current_amps);
    }
    for (int local = 0;
         local < static_cast<int>(primitive.primitive.to_terminal.phases.size());
         ++local) {
      const int phase =
          primitive.primitive.to_terminal.phases[static_cast<size_t>(local)];
      set_phasor_observation_phase(
          observation.lv_voltage_pu,
          phase,
          measurement.to_voltage(local));
      set_phasor_observation_phase(
          observation.lv_current_amps,
          phase,
          measurement.to_current(local) * primitive.lv_base_current_amps);
    }
    observations.push_back(std::move(observation));
  }
  return observations;
}

std::vector<int> build_fixed_phase_nodes(
    const ThreePhaseACSystem& sys,
    const PhaseNodeIndexer& indexer) {
  std::vector<int> fixed;
  for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    if (sys.buses[bus_offset].bus_type != BusType::SLACK) continue;
    const PhaseMask mask = indexer.bus_phase_mask(static_cast<int>(bus_offset));
    for (int phase = 0; phase < 3; ++phase) {
      if (!mask.has(phase)) continue;
      fixed.push_back(indexer.node_index(static_cast<int>(bus_offset), phase));
    }
  }
  return fixed;
}

std::vector<int> build_variable_phase_nodes(
    int node_count,
    const std::vector<int>& fixed_indices) {
  std::vector<bool> is_fixed(static_cast<std::size_t>(node_count), false);
  for (const int node : fixed_indices) {
    if (node >= 0 && node < node_count) {
      is_fixed[static_cast<std::size_t>(node)] = true;
    }
  }

  std::vector<int> variable;
  variable.reserve(static_cast<std::size_t>(node_count));
  for (int node = 0; node < node_count; ++node) {
    if (!is_fixed[static_cast<std::size_t>(node)]) {
      variable.push_back(node);
    }
  }
  return variable;
}

std::vector<int> build_dense_index_lookup(
    int node_count,
    const std::vector<int>& indices) {
  std::vector<int> lookup(static_cast<std::size_t>(node_count), -1);
  for (std::size_t pos = 0; pos < indices.size(); ++pos) {
    lookup[static_cast<std::size_t>(indices[pos])] = static_cast<int>(pos);
  }
  return lookup;
}

ComplexVectorX gather_complex_entries(
    const ComplexVectorX& vector,
    const std::vector<int>& indices) {
  ComplexVectorX gathered(static_cast<int>(indices.size()));
  for (std::size_t pos = 0; pos < indices.size(); ++pos) {
    gathered(static_cast<int>(pos)) = vector(indices[pos]);
  }
  return gathered;
}

Eigen::SparseMatrix<Complex> extract_sparse_submatrix(
    const Eigen::SparseMatrix<Complex>& matrix,
    const std::vector<int>& row_indices,
    const std::vector<int>& col_indices) {
  std::vector<int> row_lookup = build_dense_index_lookup(matrix.rows(), row_indices);
  std::vector<int> col_lookup = build_dense_index_lookup(matrix.cols(), col_indices);
  std::vector<ComplexTriplet> triplets;
  triplets.reserve(static_cast<std::size_t>(matrix.nonZeros()));

  for (int col = 0; col < matrix.outerSize(); ++col) {
    const int local_col = (col >= 0 && col < static_cast<int>(col_lookup.size()))
                              ? col_lookup[static_cast<std::size_t>(col)]
                              : -1;
    if (local_col < 0) continue;
    for (Eigen::SparseMatrix<Complex>::InnerIterator it(matrix, col); it; ++it) {
      const int row = static_cast<int>(it.row());
      const int local_row = (row >= 0 && row < static_cast<int>(row_lookup.size()))
                                ? row_lookup[static_cast<std::size_t>(row)]
                                : -1;
      if (local_row < 0) continue;
      if (std::abs(it.value()) <= kPhaseValueTol) continue;
      triplets.emplace_back(local_row, local_col, it.value());
    }
  }

  Eigen::SparseMatrix<Complex> submatrix(
      static_cast<int>(row_indices.size()),
      static_cast<int>(col_indices.size()));
  submatrix.setFromTriplets(
      triplets.begin(),
      triplets.end(),
      [](const Complex& a, const Complex& b) { return a + b; });
  return submatrix;
}

std::vector<PhaseDomainSparseEntry> sparse_entries_from_matrix(
    const Eigen::SparseMatrix<Complex>& matrix) {
  std::vector<PhaseDomainSparseEntry> entries;
  entries.reserve(static_cast<std::size_t>(matrix.nonZeros()));
  for (int col = 0; col < matrix.outerSize(); ++col) {
    for (Eigen::SparseMatrix<Complex>::InnerIterator it(matrix, col); it; ++it) {
      entries.push_back(PhaseDomainSparseEntry{
          .row = static_cast<int>(it.row()),
          .col = static_cast<int>(it.col()),
          .value = it.value(),
      });
    }
  }
  return entries;
}

int full_phase_index(int bus_offset, int phase_index) {
  return bus_offset * 3 + phase_index;
}

std::vector<int> full_terminal_indices(const PhaseTerminal& terminal) {
  std::vector<int> indices;
  indices.reserve(terminal.phases.size());
  for (const int phase : terminal.phases) {
    indices.push_back(full_phase_index(terminal.bus_offset, phase));
  }
  return indices;
}

void stamp_full_terminal_shunt(
    std::vector<ComplexTriplet>& triplets,
    const PhaseTerminal& terminal,
    const ComplexMatrixX& shunt,
    const std::string& label) {
  if (shunt.rows() != static_cast<int>(terminal.nodes.size()) ||
      shunt.cols() != static_cast<int>(terminal.nodes.size())) {
    throw std::runtime_error(
        label + " full-matrix stamp dimension mismatch");
  }
  const std::vector<int> indices = full_terminal_indices(terminal);
  for (int row = 0; row < shunt.rows(); ++row) {
    for (int col = 0; col < shunt.cols(); ++col) {
      const Complex value = shunt(row, col);
      if (std::abs(value) <= kPhaseValueTol) continue;
      triplets.emplace_back(indices[static_cast<std::size_t>(row)],
                            indices[static_cast<std::size_t>(col)],
                            value);
    }
  }
}

void stamp_full_primitive_branch(
    std::vector<ComplexTriplet>& triplets,
    const PrimitiveBranchModel& primitive,
    const std::string& label) {
  if (primitive.y_ff.rows() == 0) return;

  const std::vector<int> from_indices = full_terminal_indices(primitive.from_terminal);
  const std::vector<int> to_indices = full_terminal_indices(primitive.to_terminal);

  auto stamp_block = [&](const std::vector<int>& rows,
                         const std::vector<int>& cols,
                         const ComplexMatrixX& block,
                         const char* block_name) {
    if (block.rows() != static_cast<int>(rows.size()) ||
        block.cols() != static_cast<int>(cols.size())) {
      throw std::runtime_error(
          label + " " + block_name + " full-matrix stamp dimension mismatch");
    }
    for (int row = 0; row < block.rows(); ++row) {
      for (int col = 0; col < block.cols(); ++col) {
        const Complex value = block(row, col);
        if (std::abs(value) <= kPhaseValueTol) continue;
        triplets.emplace_back(rows[static_cast<std::size_t>(row)],
                              cols[static_cast<std::size_t>(col)],
                              value);
      }
    }
  };

  stamp_block(from_indices, from_indices, primitive.y_ff, "y_ff");
  stamp_block(from_indices, to_indices, primitive.y_ft, "y_ft");
  stamp_block(to_indices, from_indices, primitive.y_tf, "y_tf");
  stamp_block(to_indices, to_indices, primitive.y_tt, "y_tt");
}

void add_full_fixed_current(
    ComplexVectorX& fixed_current,
    const PhaseTerminal& terminal,
    const ComplexVectorX& values,
    const std::string& label) {
  if (values.rows() != static_cast<int>(terminal.nodes.size())) {
    throw std::runtime_error(
        label + " full-matrix fixed current dimension mismatch");
  }
  for (int local = 0; local < values.rows(); ++local) {
    const int phase = terminal.phases[static_cast<std::size_t>(local)];
    fixed_current(full_phase_index(terminal.bus_offset, phase)) += values(local);
  }
}

void stamp_full_prepared_constant_impedance_loads(
    std::vector<ComplexTriplet>& triplets,
    const PreparedLoadSet& prepared_loads,
    const PhaseNodeIndexer& indexer) {
  for (const auto& load : prepared_loads.wye_loads) {
    for (int row = 0; row < load.reduced_y_shunt_pu.rows(); ++row) {
      for (int col = 0; col < load.reduced_y_shunt_pu.cols(); ++col) {
        const Complex value = load.reduced_y_shunt_pu(row, col);
        if (std::abs(value) <= kPhaseValueTol) continue;
        const PhaseNodeRef& row_ref =
            indexer.node_ref(load.nodes[static_cast<std::size_t>(row)]);
        const PhaseNodeRef& col_ref =
            indexer.node_ref(load.nodes[static_cast<std::size_t>(col)]);
        triplets.emplace_back(
            full_phase_index(row_ref.bus_offset, row_ref.phase_index),
            full_phase_index(col_ref.bus_offset, col_ref.phase_index),
            value);
      }
    }
  }

  for (const auto& load : prepared_loads.branch_loads) {
    if (std::abs(load.y_branch_pu) <= kPhaseValueTol) continue;
    const PhaseNodeRef& from_ref = indexer.node_ref(load.from_node);
    const PhaseNodeRef& to_ref = indexer.node_ref(load.to_node);
    const int from_index = full_phase_index(from_ref.bus_offset, from_ref.phase_index);
    const int to_index = full_phase_index(to_ref.bus_offset, to_ref.phase_index);
    triplets.emplace_back(from_index, from_index, load.y_branch_pu);
    triplets.emplace_back(to_index, to_index, load.y_branch_pu);
    triplets.emplace_back(from_index, to_index, -load.y_branch_pu);
    triplets.emplace_back(to_index, from_index, -load.y_branch_pu);
  }
}

ComplexVectorX collect_wye_phase_voltages(
    const PreparedWyeLoadContribution& load,
    const ComplexVectorX& voltage_state) {
  ComplexVectorX voltages(static_cast<int>(load.nodes.size()));
  for (int local = 0; local < static_cast<int>(load.nodes.size()); ++local) {
    voltages(local) = voltage_state(load.nodes[static_cast<std::size_t>(local)]);
  }
  return voltages;
}

struct CompactFixedPointContext {
  double base_mva{100.0};
  std::unordered_map<int, int> id_map;
  PhaseNodeIndexer indexer;
  PreparedLoadSet prepared_loads;
  PhaseNetworkModel network_model;
  PhaseSpec spec;
  std::vector<int> fixed_indices;
  std::vector<int> variable_indices;
  std::vector<int> fixed_lookup;
  std::vector<int> variable_lookup;
  ComplexVectorX fixed_voltage;
  Eigen::SparseMatrix<Complex> y_vv;
  Eigen::SparseMatrix<Complex> y_vf;
};

CompactFixedPointContext build_compact_fixed_point_context(
    const ThreePhaseACSystem& sys,
    bool include_shunts) {
  CompactFixedPointContext context;
  context.base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;

  context.id_map.reserve(sys.buses.size());
  for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    context.id_map[sys.buses[bus_offset].index] = static_cast<int>(bus_offset);
  }

  context.indexer = PhaseNodeIndexer::build(sys);
  context.prepared_loads =
      prepare_load_models(sys, context.id_map, context.indexer, context.base_mva);
  context.network_model =
      build_phase_network_model(
          sys,
          context.id_map,
          context.indexer,
          context.prepared_loads,
          include_shunts,
          context.base_mva);
  context.spec =
      compute_phase_spec(sys, context.id_map, context.indexer, context.base_mva);
  context.fixed_indices = build_fixed_phase_nodes(sys, context.indexer);
  context.variable_indices =
      build_variable_phase_nodes(context.indexer.total_nodes, context.fixed_indices);
  context.fixed_lookup =
      build_dense_index_lookup(context.indexer.total_nodes, context.fixed_indices);
  context.variable_lookup =
      build_dense_index_lookup(context.indexer.total_nodes, context.variable_indices);

  context.fixed_voltage = ComplexVectorX::Zero(static_cast<int>(context.fixed_indices.size()));
  for (std::size_t pos = 0; pos < context.fixed_indices.size(); ++pos) {
    const PhaseNodeRef& ref = context.indexer.node_ref(context.fixed_indices[pos]);
    const auto vm_values = bus_vm_values(sys.buses[static_cast<std::size_t>(ref.bus_offset)]);
    const auto va_values = bus_va_values_deg(sys.buses[static_cast<std::size_t>(ref.bus_offset)]);
    context.fixed_voltage(static_cast<int>(pos)) =
        std::polar(
            vm_values[static_cast<std::size_t>(ref.phase_index)],
            va_values[static_cast<std::size_t>(ref.phase_index)] * kPi / 180.0);
  }

  context.y_vv = extract_sparse_submatrix(
      context.network_model.ybus,
      context.variable_indices,
      context.variable_indices);
  context.y_vf = extract_sparse_submatrix(
      context.network_model.ybus,
      context.variable_indices,
      context.fixed_indices);
  return context;
}

ComplexVectorX evaluate_fixed_point_current_injections(
    const PreparedLoadSet& prepared_loads,
    const PhaseSpec& spec,
    const ComplexVectorX& fixed_current,
    const ComplexVectorX& voltage_state) {
  ComplexVectorX current = fixed_current;

  for (int node = 0; node < spec.p_spec.size(); ++node) {
    const Complex s_spec(spec.p_spec[node], spec.q_spec[node]);
    if (std::abs(s_spec) <= kPhaseValueTol) continue;
    const Complex voltage = voltage_state(node);
    if (std::abs(voltage) <= 1e-9) continue;
    current(node) += std::conj(s_spec / voltage);
  }

  for (const auto& load : prepared_loads.wye_loads) {
    const ComplexVectorX phase_voltages =
        collect_wye_phase_voltages(load, voltage_state);
    const LocalNeutralSolution neutral_state =
        solve_local_neutral_state(load, phase_voltages);
    for (int local = 0; local < static_cast<int>(load.nodes.size()); ++local) {
      const Complex dynamic_current =
          neutral_state.dynamic_branches[static_cast<std::size_t>(local)].current_pu;
      if (std::abs(dynamic_current) <= kPhaseValueTol) continue;
      current(load.nodes[static_cast<std::size_t>(local)]) -= dynamic_current;
    }
  }

  for (const auto& load : prepared_loads.branch_loads) {
    const Complex v_from = voltage_state(load.from_node);
    const Complex v_to = voltage_state(load.to_node);
    const Complex branch_voltage = v_from - v_to;
    const double branch_voltage_abs = std::abs(branch_voltage);
    if (branch_voltage_abs <= 1e-9) continue;

    const DynamicLoadPowerEvaluation evaluation =
        evaluate_dynamic_load_power(
            branch_voltage_abs / std::sqrt(3.0),
            load.vmin_pu,
            load.vmax_pu,
            load.zipv_cutoff_pu,
            load.s_current_nominal_pu,
            load.s_power_nominal_pu);
    if (std::abs(evaluation.s_dynamic_pu) <= kPhaseValueTol) continue;

    const Complex branch_current =
        dynamic_branch_current_from_power(branch_voltage, evaluation);
    if (std::abs(branch_current) <= kPhaseValueTol) continue;
    current(load.from_node) -= branch_current;
    current(load.to_node) += branch_current;
  }

  return current;
}

ComplexVectorX evaluate_fixed_point_current_injections(
    const CompactFixedPointContext& context,
    const ComplexVectorX& voltage_state) {
  return evaluate_fixed_point_current_injections(
      context.prepared_loads,
      context.spec,
      context.network_model.fixed_current,
      voltage_state);
}

PreparedLoadSet scale_dynamic_load_set(
    const PreparedLoadSet& base,
    double load_scale) {
  PreparedLoadSet scaled = base;
  for (auto& load : scaled.wye_loads) {
    load.s_current_nominal_pu *= load_scale;
    load.s_power_nominal_pu *= load_scale;
  }
  for (auto& load : scaled.branch_loads) {
    load.s_current_nominal_pu *= load_scale;
    load.s_power_nominal_pu *= load_scale;
  }
  return scaled;
}

struct FullPhaseMatrixModel {
  double base_mva{100.0};
  std::unordered_map<int, int> id_map;
  PhaseNodeIndexer indexer;
  PreparedLoadSet prepared_loads;
  PhaseSpec spec;
  Eigen::SparseMatrix<Complex> ybus;
  ComplexVectorX fixed_current;
  std::vector<int> compact_to_full;
  std::vector<int> full_to_compact;
  std::vector<int> fixed_indices;
  std::vector<int> variable_indices;
  ComplexVectorX fixed_voltage;
};

std::vector<int> build_full_fixed_phase_nodes(
    const ThreePhaseACSystem& sys,
    const PhaseNodeIndexer& indexer) {
  std::vector<int> fixed;
  fixed.reserve(sys.buses.size() * 3);
  for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    const bool is_slack = sys.buses[bus_offset].bus_type == BusType::SLACK;
    for (int phase = 0; phase < 3; ++phase) {
      if (!indexer.has_node(static_cast<int>(bus_offset), phase)) {
        fixed.push_back(full_phase_index(static_cast<int>(bus_offset), phase));
        continue;
      }
      if (is_slack) {
        fixed.push_back(full_phase_index(static_cast<int>(bus_offset), phase));
      }
    }
  }
  std::sort(fixed.begin(), fixed.end());
  fixed.erase(std::unique(fixed.begin(), fixed.end()), fixed.end());
  return fixed;
}

std::vector<int> build_full_variable_phase_nodes(
    const ThreePhaseACSystem& sys,
    const PhaseNodeIndexer& indexer) {
  std::vector<int> variable;
  variable.reserve(sys.buses.size() * 3);
  for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    if (sys.buses[bus_offset].bus_type == BusType::SLACK) continue;
    for (int phase = 0; phase < 3; ++phase) {
      if (!indexer.has_node(static_cast<int>(bus_offset), phase)) continue;
      variable.push_back(full_phase_index(static_cast<int>(bus_offset), phase));
    }
  }
  return variable;
}

ComplexVectorX build_full_fixed_voltage(
    const ThreePhaseACSystem& sys,
    const PhaseNodeIndexer& indexer,
    const std::vector<int>& fixed_indices) {
  ComplexVectorX fixed_voltage = ComplexVectorX::Zero(
      static_cast<int>(fixed_indices.size()));
  for (std::size_t pos = 0; pos < fixed_indices.size(); ++pos) {
    const int full_index = fixed_indices[pos];
    const int bus_offset = full_index / 3;
    const int phase = full_index % 3;
    if (!indexer.has_node(bus_offset, phase)) {
      fixed_voltage(static_cast<int>(pos)) = Complex(0.0, 0.0);
      continue;
    }
    const auto vm_values = bus_vm_values(sys.buses[static_cast<std::size_t>(bus_offset)]);
    const auto va_values = bus_va_values_deg(sys.buses[static_cast<std::size_t>(bus_offset)]);
    fixed_voltage(static_cast<int>(pos)) = std::polar(
        vm_values[static_cast<std::size_t>(phase)],
        va_values[static_cast<std::size_t>(phase)] * kPi / 180.0);
  }
  return fixed_voltage;
}

FullPhaseMatrixModel build_full_phase_matrix_model(
    const ThreePhaseACSystem& sys,
    bool include_shunts) {
  FullPhaseMatrixModel model;
  model.base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;

  model.id_map.reserve(sys.buses.size());
  for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    model.id_map[sys.buses[bus_offset].index] = static_cast<int>(bus_offset);
  }

  model.indexer = PhaseNodeIndexer::build(sys);
  model.prepared_loads =
      prepare_load_models(sys, model.id_map, model.indexer, model.base_mva);
  model.spec =
      compute_phase_spec(sys, model.id_map, model.indexer, model.base_mva);

  const int full_node_count = static_cast<int>(sys.buses.size()) * 3;
  model.fixed_current = ComplexVectorX::Zero(full_node_count);
  model.compact_to_full.resize(static_cast<std::size_t>(model.indexer.total_nodes), -1);
  model.full_to_compact.resize(static_cast<std::size_t>(full_node_count), -1);
  for (int node = 0; node < model.indexer.total_nodes; ++node) {
    const PhaseNodeRef& ref = model.indexer.node_ref(node);
    const int full_index = full_phase_index(ref.bus_offset, ref.phase_index);
    model.compact_to_full[static_cast<std::size_t>(node)] = full_index;
    model.full_to_compact[static_cast<std::size_t>(full_index)] = node;
  }

  std::vector<ComplexTriplet> triplets;
  triplets.reserve(static_cast<std::size_t>(
      18 * sys.lines.size() +
      36 * sys.transformers.size() +
      9 * sys.external_grids.size() +
      3 * sys.buses.size() +
      9 * model.prepared_loads.wye_loads.size() +
      4 * model.prepared_loads.branch_loads.size() +
      sys.buses.size() * 3));

  if (include_shunts) {
    const double phase_scale = phase_domain_power_base_scale(model.base_mva);
    for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
      const auto& bus = sys.buses[bus_offset];
      if (!bus.in_service) continue;
      require_phase_values_on_mask(
          "bus",
          bus.index,
          bus.phase_mask,
          {bus.gs_a_mw, bus.gs_b_mw, bus.gs_c_mw},
          {bus.bs_a_mvar, bus.bs_b_mvar, bus.bs_c_mvar});
      const PhaseTerminal terminal = collect_terminal_nodes(
          model.indexer,
          static_cast<int>(bus_offset),
          bus.phase_mask,
          "bus",
          bus.index,
          "shunt");
      ComplexMatrixX shunt = ComplexMatrixX::Zero(
          static_cast<int>(terminal.nodes.size()),
          static_cast<int>(terminal.nodes.size()));
      for (int local = 0; local < static_cast<int>(terminal.phases.size()); ++local) {
        const int phase = terminal.phases[static_cast<std::size_t>(local)];
        if (phase == 0) {
          shunt(local, local) = Complex(bus.gs_a_mw * phase_scale, bus.bs_a_mvar * phase_scale);
        } else if (phase == 1) {
          shunt(local, local) = Complex(bus.gs_b_mw * phase_scale, bus.bs_b_mvar * phase_scale);
        } else {
          shunt(local, local) = Complex(bus.gs_c_mw * phase_scale, bus.bs_c_mvar * phase_scale);
        }
      }
      stamp_full_terminal_shunt(triplets, terminal, shunt, "bus");
    }
  }

  stamp_full_prepared_constant_impedance_loads(
      triplets,
      model.prepared_loads,
      model.indexer);

  for (const auto& line : sys.lines) {
    if (!line.in_service) continue;
    auto it_f = model.id_map.find(line.from_bus);
    auto it_t = model.id_map.find(line.to_bus);
    if (it_f == model.id_map.end() || it_t == model.id_map.end()) continue;
    stamp_full_primitive_branch(
        triplets,
        build_line_primitive(model.indexer, it_f->second, it_t->second, line),
        "line");
  }

  for (const auto& transformer : sys.transformers) {
    if (!transformer.in_service) continue;
    const TransformerPrimitiveRecord primitive =
        build_transformer_primitive(
            model.indexer,
            transformer,
            model.id_map,
            sys,
            model.base_mva);
    stamp_full_primitive_branch(triplets, primitive.primitive, "transformer");
  }

  for (const auto& source : sys.external_grids) {
    if (!source.in_service) continue;
    auto it_bus = model.id_map.find(source.bus);
    if (it_bus == model.id_map.end()) continue;
    const PhaseTerminal terminal = collect_terminal_nodes(
        model.indexer,
        it_bus->second,
        source.phase_mask,
        "external_grid",
        source.index,
        "terminal");
    const SourceNortonPrimitive primitive =
        build_source_norton_primitive(source, terminal, model.base_mva);
    stamp_full_terminal_shunt(triplets, terminal, primitive.y_phase, "external_grid");
    add_full_fixed_current(model.fixed_current, terminal, primitive.fixed_current, "external_grid");
  }

  for (std::size_t bus_offset = 0; bus_offset < sys.buses.size(); ++bus_offset) {
    for (int phase = 0; phase < 3; ++phase) {
      if (model.indexer.has_node(static_cast<int>(bus_offset), phase)) continue;
      triplets.emplace_back(
          full_phase_index(static_cast<int>(bus_offset), phase),
          full_phase_index(static_cast<int>(bus_offset), phase),
          Complex(1e-6, 1e-6));
    }
  }

  model.ybus.resize(full_node_count, full_node_count);
  model.ybus.setFromTriplets(
      triplets.begin(),
      triplets.end(),
      [](const Complex& a, const Complex& b) { return a + b; });

  model.fixed_indices = build_full_fixed_phase_nodes(sys, model.indexer);
  model.variable_indices = build_full_variable_phase_nodes(sys, model.indexer);
  model.fixed_voltage =
      build_full_fixed_voltage(sys, model.indexer, model.fixed_indices);
  return model;
}

ComplexVectorX gather_full_voltage_to_compact(
    const FullPhaseMatrixModel& model,
    const ComplexVectorX& full_voltage) {
  ComplexVectorX compact_voltage = ComplexVectorX::Zero(model.indexer.total_nodes);
  for (int node = 0; node < model.indexer.total_nodes; ++node) {
    compact_voltage(node) =
        full_voltage(model.compact_to_full[static_cast<std::size_t>(node)]);
  }
  return compact_voltage;
}

ComplexVectorX scatter_compact_current_to_full(
    const FullPhaseMatrixModel& model,
    const ComplexVectorX& compact_current) {
  ComplexVectorX full_current = ComplexVectorX::Zero(model.ybus.rows());
  for (int node = 0; node < compact_current.size(); ++node) {
    full_current(model.compact_to_full[static_cast<std::size_t>(node)]) =
        compact_current(node);
  }
  return full_current;
}

ThreePhaseDPFResult finalize_three_phase_result_from_state(
    const ThreePhaseACSystem& sys,
    const std::unordered_map<int, int>& id_map,
    const PhaseNodeIndexer& indexer,
    const std::vector<TransformerPrimitiveRecord>& transformer_primitives,
    const Eigen::VectorXd& vm,
    const Eigen::VectorXd& va,
    bool converged,
    int iterations,
    double residual,
    double base_mva) {
  const int n = static_cast<int>(sys.buses.size());
  const int m = static_cast<int>(sys.lines.size());

  ThreePhaseDPFResult result;
  result.bus_voltages.resize(n);
  result.branch_powers.resize(m);
  result.converged = converged;
  result.iterations = iterations;
  result.residual = residual;

  const double rad2deg = 180.0 / kPi;
  for (int bus_offset = 0; bus_offset < n; ++bus_offset) {
    auto& bv = result.bus_voltages[static_cast<std::size_t>(bus_offset)];
    bv.bus_id = sys.buses[static_cast<std::size_t>(bus_offset)].index;
    bv.vm_a_pu = 0.0; bv.va_a_deg = 0.0;
    bv.vm_b_pu = 0.0; bv.va_b_deg = 0.0;
    bv.vm_c_pu = 0.0; bv.va_c_deg = 0.0;
    for (int phase = 0; phase < 3; ++phase) {
      if (!indexer.has_node(bus_offset, phase)) continue;
      const int node = indexer.node_index(bus_offset, phase);
      set_bus_voltage_phase(bv, phase, vm[node], va[node] * rad2deg);
    }
  }

  result.transformer_terminal_observations =
      build_transformer_terminal_observations(transformer_primitives, vm, va);

  double p_loss_a = 0.0, q_loss_a = 0.0;
  double p_loss_b = 0.0, q_loss_b = 0.0;
  double p_loss_c = 0.0, q_loss_c = 0.0;

  for (int line_pos = 0; line_pos < m; ++line_pos) {
    const auto& line = sys.lines[static_cast<std::size_t>(line_pos)];
    if (!line.in_service) continue;
    auto it_f = id_map.find(line.from_bus);
    auto it_t = id_map.find(line.to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;

    auto& bp = result.branch_powers[static_cast<std::size_t>(line_pos)];
    bp.line_index = line.index;

    const PrimitiveBranchModel primitive =
        build_line_primitive(indexer, it_f->second, it_t->second, line);
    if (primitive.y_ff.rows() == 0) continue;

    const PrimitiveBranchObservation observation =
        observe_primitive_branch(primitive, vm, va);

    for (int local = 0;
         local < static_cast<int>(primitive.from_terminal.phases.size());
         ++local) {
      const int phase = primitive.from_terminal.phases[static_cast<std::size_t>(local)];
      // NR per-phase p.u. system: S_base_1ph = base_mva / 3
      const double s_base_1ph = base_mva / 3.0;
      const Complex s_from =
          observation.from_voltage(local) *
          std::conj(observation.from_current(local)) * s_base_1ph;
      const Complex s_to =
          observation.to_voltage(local) *
          std::conj(observation.to_current(local)) * s_base_1ph;
      const Complex s_loss = s_from + s_to;
      set_branch_power_phase(bp, phase, s_from.real(), s_from.imag());
      if (phase == 0) {
        p_loss_a += s_loss.real();
        q_loss_a += s_loss.imag();
      } else if (phase == 1) {
        p_loss_b += s_loss.real();
        q_loss_b += s_loss.imag();
      } else {
        p_loss_c += s_loss.real();
        q_loss_c += s_loss.imag();
      }
    }
  }

  result.p_loss_a_mw = p_loss_a;
  result.q_loss_a_mvar = q_loss_a;
  result.p_loss_b_mw = p_loss_b;
  result.q_loss_b_mvar = q_loss_b;
  result.p_loss_c_mw = p_loss_c;
  result.q_loss_c_mvar = q_loss_c;
  result.total_p_loss_mw = p_loss_a + p_loss_b + p_loss_c;
  result.total_q_loss_mvar = q_loss_a + q_loss_b + q_loss_c;
  result.max_vuf_percent = compute_max_vuf(indexer, vm, va);
  return result;
}

Eigen::SparseMatrix<Complex> regularize_sparse_diagonal(
    const Eigen::SparseMatrix<Complex>& matrix,
    double diagonal_shift) {
  Eigen::SparseMatrix<Complex> regularized = matrix;
  regularized.makeCompressed();
  for (int idx = 0; idx < regularized.rows(); ++idx) {
    regularized.coeffRef(idx, idx) += Complex(diagonal_shift, diagonal_shift);
  }
  regularized.makeCompressed();
  return regularized;
}

void factorize_compact_matrix(
    const Eigen::SparseMatrix<Complex>& matrix,
    const std::string& label,
    Eigen::SparseLU<Eigen::SparseMatrix<Complex>>& lu) {
  auto try_factorize = [&](double diagonal_shift) {
    Eigen::SparseMatrix<Complex> regularized =
        regularize_sparse_diagonal(matrix, diagonal_shift);
    lu.analyzePattern(regularized);
    lu.factorize(regularized);
    return lu.info() == Eigen::Success;
  };

  if (!try_factorize(1e-8) && !try_factorize(1e-4)) {
    throw std::runtime_error(label + " factorization failed");
  }
}

int find_transformer_position_by_index(
    const std::vector<ThreePhaseTransformer>& transformers,
    int transformer_index) {
  for (std::size_t pos = 0; pos < transformers.size(); ++pos) {
    if (transformers[pos].index == transformer_index) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error(
      "Missing ThreePhaseTransformer with index " + std::to_string(transformer_index));
}

int find_transformer_observation_position(
    const std::vector<ThreePhaseTransformerTerminalObservation>& observations,
    int transformer_index) {
  for (std::size_t pos = 0; pos < observations.size(); ++pos) {
    if (observations[pos].transformer_index == transformer_index) {
      return static_cast<int>(pos);
    }
  }
  throw std::runtime_error(
      "Missing three-phase transformer observation for index " +
      std::to_string(transformer_index));
}

bool has_enabled_three_phase_regulator(const ThreePhaseACSystem& sys) {
  return std::any_of(
      sys.regulator_controls.begin(),
      sys.regulator_controls.end(),
      [](const ThreePhaseRegulatorControl& control) { return control.enabled; });
}

}  // anonymous namespace


// =========================================================================
// solve_three_phase_nr (ThreePhaseACSystem)
// =========================================================================
namespace {

ThreePhaseSolveSnapshot solve_three_phase_nr_snapshot(
    const ThreePhaseACSystem& sys,
    const ThreePhaseNROptions& opt) {
  const int n = static_cast<int>(sys.buses.size());
  const int m = static_cast<int>(sys.lines.size());
  const double base_mva = (sys.base_mva > 0.0) ? sys.base_mva : 100.0;

  ThreePhaseSolveSnapshot snapshot;
  ThreePhaseDPFResult result;
  result.bus_voltages.resize(n);
  result.branch_powers.resize(m);

  if (n == 0) {
    snapshot.result = std::move(result);
    return snapshot;
  }

  // Build bus ID → local index map
  std::unordered_map<int, int> id_map;
  id_map.reserve(n);
  for (int i = 0; i < n; ++i)
    id_map[sys.buses[i].index] = i;

  // A fixed-voltage SLACK bus is still valid, but external grids can now
  // provide the voltage reference through their internal source phasors.
  bool has_slack = false;
  for (int i = 0; i < n; ++i)
    if (sys.buses[i].bus_type == BusType::SLACK) { has_slack = true; break; }
  bool has_external_source = false;
  for (const auto& source : sys.external_grids) {
    if (source.in_service) {
      has_external_source = true;
      break;
    }
  }
  if (!has_slack && !has_external_source) {
    throw std::runtime_error(
        "solve_three_phase_nr: requires at least one SLACK bus or in-service external_grid");
  }

  const PhaseNodeIndexer indexer = PhaseNodeIndexer::build(sys);
  const PreparedLoadSet prepared_loads =
      prepare_load_models(sys, id_map, indexer, base_mva);
  auto ctx = build_tp_nr_context(sys, indexer);
  auto network_model =
      build_phase_network_model(
          sys,
          id_map,
          indexer,
          prepared_loads,
          opt.include_shunts,
          base_mva);
  auto spec = compute_phase_spec(sys, id_map, indexer, base_mva);

  // Initialise state vectors: Vm, Va for all active phase nodes
  Eigen::VectorXd vm = Eigen::VectorXd::Ones(indexer.total_nodes);
  Eigen::VectorXd va = Eigen::VectorXd::Zero(indexer.total_nodes);
  const double deg2rad = kPi / 180.0;
  for (int bus_offset = 0; bus_offset < n; ++bus_offset) {
    const auto vm_values = bus_vm_values(sys.buses[bus_offset]);
    const auto va_values = bus_va_values_deg(sys.buses[bus_offset]);
    for (int phase = 0; phase < 3; ++phase) {
      if (!indexer.has_node(bus_offset, phase)) continue;
      const int node = indexer.node_index(bus_offset, phase);
      vm[node] = vm_values[phase];
      va[node] = va_values[phase] * deg2rad;
    }
  }
  seed_source_voltage_guesses(sys.external_grids, id_map, indexer, sys, vm, va);
  seed_transformer_voltage_guesses(network_model.transformer_primitives, sys, vm, va);

  if (ctx.nvar == 0) {
    result.converged = true;
    result.iterations = 0;
    result.residual = 0.0;
  } else {
    // ── Pack/unpack helpers: vm+va ↔ single x vector ──────────────────
    // x = [va(angle_nodes[0..np-1]), vm(vm_nodes[0..nq-1])]
    auto pack_state = [&](const Eigen::VectorXd& vm_vec,
                          const Eigen::VectorXd& va_vec) -> Eigen::VectorXd {
      Eigen::VectorXd x(ctx.nvar);
      for (int k = 0; k < ctx.np; ++k)
        x[k] = va_vec[ctx.angle_nodes[k]];
      for (int k = 0; k < ctx.nq; ++k)
        x[ctx.np + k] = vm_vec[ctx.vm_nodes[k]];
      return x;
    };
    auto unpack_state = [&](const Eigen::VectorXd& x,
                            Eigen::VectorXd& vm_vec,
                            Eigen::VectorXd& va_vec) {
      for (int k = 0; k < ctx.np; ++k)
        va_vec[ctx.angle_nodes[k]] = x[k];
      for (int k = 0; k < ctx.nq; ++k)
        vm_vec[ctx.vm_nodes[k]] = x[ctx.np + k];
    };

    // ── Build NonlinearSystem for NativeNewtonAdapter ─────────────────
    // Convention: NativeNewtonAdapter solves  J·dx = −f  then  x += dx
    // Our mismatch = P_spec − P_calc (positive when injection shortfall).
    // So we set f = −mismatch, which gives  J·dx = −(−mismatch) = mismatch,
    // matching the original update direction  va += dx, vm += dx.
    engine::NonlinearSystem nle;
    nle.n = ctx.nvar;
    nle.x0 = pack_state(vm, va);

    // Shared Jacobian storage: residual-only path skips Jacobian assembly
    // by passing a dummy; the Jacobian callback fills it properly.
    Eigen::SparseMatrix<double> jac_shared;
    Eigen::VectorXd mismatch_shared;

    nle.residual = [&](const Eigen::VectorXd& x, Eigen::VectorXd& f) {
      unpack_state(x, vm, va);
      evaluate_tp_mismatch_and_jacobian(
          network_model.ybus, network_model.fixed_current,
          ctx, spec, prepared_loads, vm, va,
          mismatch_shared, jac_shared);
      f = -mismatch_shared;  // f = −mismatch  →  −f = mismatch = update direction
    };

    nle.jacobian = [&](const Eigen::VectorXd& x,
                       Eigen::SparseMatrix<double>& j_out) {
      unpack_state(x, vm, va);
      evaluate_tp_mismatch_and_jacobian(
          network_model.ybus, network_model.fixed_current,
          ctx, spec, prepared_loads, vm, va,
          mismatch_shared, jac_shared);
      j_out = jac_shared;
    };

    // Projection: clamp Vm >= 0.05 (same as original hard lower bound)
    nle.project = [&](Eigen::VectorXd& x) {
      for (int k = 0; k < ctx.nq; ++k) {
        double& v = x[ctx.np + k];
        if (v < 0.05) v = 0.05;
      }
    };

    // ── Solve via engine NR framework ─────────────────────────────────
    engine::NativeNLEOptions nle_opts;
    nle_opts.max_iter = opt.max_iter;
    nle_opts.tol = opt.tol;
    // Use conservative settings: small regularization, moderate line search
    nle_opts.regularization0 = 1e-10;
    nle_opts.step_backoff = 0.5;
    nle_opts.max_line_search_steps = 20;

    const engine::NativeNewtonAdapter adapter(nle_opts);
    const auto sol = adapter.solve_nle(nle);

    // ── Unpack solution back into vm/va ───────────────────────────────
    if (sol.x.size() == ctx.nvar) {
      unpack_state(sol.x, vm, va);
    }

    result.converged = sol.stats.success;
    result.iterations = sol.stats.iterations;
    result.residual = sol.stats.residual_inf;

    if (opt.verbose) {
      HACDCPF_LOG_DEBUG("  3ph-NR via NativeNewtonAdapter: {} in {} iter, residual={}",
                        sol.stats.status, sol.stats.iterations, sol.stats.residual_inf);
    }
  }

  // Extract per-phase bus voltages
  const double rad2deg = 180.0 / kPi;
  for (int bus_offset = 0; bus_offset < n; ++bus_offset) {
    auto& bv = result.bus_voltages[bus_offset];
    bv.bus_id = sys.buses[bus_offset].index;
    bv.vm_a_pu = 0.0; bv.va_a_deg = 0.0;
    bv.vm_b_pu = 0.0; bv.va_b_deg = 0.0;
    bv.vm_c_pu = 0.0; bv.va_c_deg = 0.0;
    for (int phase = 0; phase < 3; ++phase) {
      if (!indexer.has_node(bus_offset, phase)) continue;
      const int node = indexer.node_index(bus_offset, phase);
      set_bus_voltage_phase(bv, phase, vm[node], va[node] * rad2deg);
    }
  }

  result.transformer_terminal_observations = build_transformer_terminal_observations(
      network_model.transformer_primitives,
      vm,
      va);

  // Compute per-phase branch power and losses
  double p_loss_a = 0.0, q_loss_a = 0.0;
  double p_loss_b = 0.0, q_loss_b = 0.0;
  double p_loss_c = 0.0, q_loss_c = 0.0;

  for (int b = 0; b < m; ++b) {
    if (!sys.lines[b].in_service) continue;
    auto it_f = id_map.find(sys.lines[b].from_bus);
    auto it_t = id_map.find(sys.lines[b].to_bus);
    if (it_f == id_map.end() || it_t == id_map.end()) continue;
    const int fi = it_f->second;
    const int ti = it_t->second;

    auto& bp = result.branch_powers[b];
    bp.line_index = sys.lines[b].index;
    const PrimitiveBranchModel primitive =
        build_line_primitive(indexer, fi, ti, sys.lines[b]);
    if (primitive.y_ff.rows() == 0) continue;

    const PrimitiveBranchObservation observation =
        observe_primitive_branch(primitive, vm, va);

    for (int local = 0;
         local < static_cast<int>(primitive.from_terminal.phases.size());
         ++local) {
      const int phase = primitive.from_terminal.phases[static_cast<size_t>(local)];
      // NR per-phase p.u. system: S_base_1ph = base_mva / 3
      const double s_base_1ph = base_mva / 3.0;
      const Complex s_from =
          observation.from_voltage(local) *
          std::conj(observation.from_current(local)) * s_base_1ph;
      const Complex s_to =
          observation.to_voltage(local) *
          std::conj(observation.to_current(local)) * s_base_1ph;
      const Complex s_loss = s_from + s_to;
      set_branch_power_phase(bp, phase, s_from.real(), s_from.imag());
      if (phase == 0) {
        p_loss_a += s_loss.real();
        q_loss_a += s_loss.imag();
      } else if (phase == 1) {
        p_loss_b += s_loss.real();
        q_loss_b += s_loss.imag();
      } else {
        p_loss_c += s_loss.real();
        q_loss_c += s_loss.imag();
      }
    }
  }

  result.p_loss_a_mw = p_loss_a;   result.q_loss_a_mvar = q_loss_a;
  result.p_loss_b_mw = p_loss_b;   result.q_loss_b_mvar = q_loss_b;
  result.p_loss_c_mw = p_loss_c;   result.q_loss_c_mvar = q_loss_c;
  result.total_p_loss_mw   = p_loss_a + p_loss_b + p_loss_c;
  result.total_q_loss_mvar = q_loss_a + q_loss_b + q_loss_c;

  // VUF
  result.max_vuf_percent = compute_max_vuf(indexer, vm, va);

  snapshot.result = std::move(result);
  snapshot.id_map = std::move(id_map);
  return snapshot;
}

}  // anonymous namespace

ThreePhaseDPFResult solve_three_phase_nr(
    const ThreePhaseACSystem& sys,
    const ThreePhaseNROptions& opt) {
  if (!has_enabled_three_phase_regulator(sys)) {
    return solve_three_phase_nr_snapshot(sys, opt).result;
  }

  ThreePhaseACSystem working = sys;
  std::vector<ThreePhaseRegulatorControlTraceEntry> accumulated_trace;
  std::set<std::vector<int>> seen_tap_states;

  auto capture_tap_state = [&working]() {
    std::vector<int> state;
    state.reserve(working.regulator_controls.size());
    for (const auto& control : working.regulator_controls) {
      if (!control.enabled) continue;
      const int transformer_pos = find_transformer_position_by_index(
          working.transformers,
          control.transformer_index);
      state.push_back(
          working.transformers[static_cast<std::size_t>(transformer_pos)].tap_pos);
    }
    return state;
  };
  seen_tap_states.insert(capture_tap_state());

  ThreePhaseDPFResult final_result;
  bool finalized = false;

  for (int control_iter = 1; control_iter <= opt.max_control_iter; ++control_iter) {
    ThreePhaseSolveSnapshot snapshot = solve_three_phase_nr_snapshot(working, opt);
    ThreePhaseDPFResult current_result = snapshot.result;
    std::vector<ThreePhaseRegulatorControlState> regulator_states;
    regulator_states.reserve(working.regulator_controls.size());

    bool any_tap_changed = false;
    bool control_failed = !current_result.converged;
    std::vector<std::size_t> iteration_trace_positions;

    for (const auto& control : working.regulator_controls) {
      if (!control.enabled) continue;

      ThreePhaseRegulatorControlState state;
      state.regulator_name = control.name;
      state.transformer_name = control.transformer_name;
      state.transformer_index = control.transformer_index;
      state.winding = control.winding;
      state.tap_winding = control.tap_winding;
      state.monitored_bus = control.monitored_bus;
      state.monitored_node = control.monitored_node;
      state.used_remote_bus = (control.monitored_bus != 0);
      state.used_line_drop_compensation =
          std::abs(control.r_volts) > kPhaseValueTol ||
          std::abs(control.x_volts) > kPhaseValueTol;
      state.target_vreg_volts = control.vreg_volts;
      state.band_volts = control.band_volts;
      state.ptratio = control.ptratio;
      state.remote_ptratio =
          (control.remote_ptratio > 0.0) ? control.remote_ptratio : control.ptratio;
      state.ct_primary_amps = control.ct_primary_amps;
      state.r_volts = control.r_volts;
      state.x_volts = control.x_volts;
      state.max_tap_change = control.max_tap_change;
      state.control_iterations = control_iter;

      const std::size_t trace_pos = accumulated_trace.size();
      iteration_trace_positions.push_back(trace_pos);
      accumulated_trace.push_back(ThreePhaseRegulatorControlTraceEntry{
          .regulator_name = control.name,
          .iteration = control_iter,
          .target_vreg_volts = control.vreg_volts,
          .band_half_volts = control.band_volts / 2.0,
          .decision_reason = "",
          .stop_reason = "",
      });
      auto& trace = accumulated_trace.back();

      try {
        const int transformer_pos = find_transformer_position_by_index(
            working.transformers,
            control.transformer_index);
        const auto& transformer =
            working.transformers[static_cast<std::size_t>(transformer_pos)];
        trace.current_tap_pos = transformer.tap_pos;
        trace.current_tap_number = transformer_tap_number(transformer);
        trace.current_tap_pu = transformer_tap_pu(transformer);

        if (control.reversible) {
          throw std::runtime_error("unsupported_reversible_control");
        }
        if (control.winding < 1 || control.winding > 2 ||
            control.tap_winding < 1 || control.tap_winding > 2) {
          throw std::runtime_error("unsupported_winding_reference");
        }
        if (!tap_side_matches_winding(transformer, control.tap_winding)) {
          throw std::runtime_error("tap_side_tap_winding_mismatch");
        }
        if (transformer.tap_max < transformer.tap_min ||
            std::abs(transformer.tap_step_percent) < kPhaseValueTol) {
          throw std::runtime_error("transformer_missing_discrete_tap_grid");
        }
        if (control.max_tap_change != 1) {
          throw std::runtime_error("unsupported_max_tap_change");
        }

        const ParsedVectorGroup group = parse_vector_group(transformer.vector_group);
        const double expected_shift_deg = shift_deg_from_clock(group.clock);
        const double effective_shift_deg =
            (std::abs(transformer.shift_deg) > kPhaseValueTol)
                ? transformer.shift_deg
                : expected_shift_deg;
        if (group.hv != TransformerConnectionKind::GroundedWye ||
            group.lv != TransformerConnectionKind::GroundedWye ||
            std::abs(effective_shift_deg) > 1e-6) {
          throw std::runtime_error("unsupported_regulator_transformer_connection");
        }
        if (transformer.hv_phase_mask.bits != transformer.lv_phase_mask.bits) {
          throw std::runtime_error("regulator_transformer_phase_mask_mismatch");
        }
        const int transformer_phase = single_phase_index_from_mask(
            "regulator transformer",
            transformer.index,
            transformer.hv_phase_mask);
        const int monitored_phase = monitored_phase_index_from_node(control.monitored_node);
        if (monitored_phase != transformer_phase) {
          throw std::runtime_error("unsupported_monitored_node");
        }

        const int monitored_bus =
            (control.monitored_bus != 0)
                ? control.monitored_bus
                : transformer_winding_bus(transformer, control.winding);
        auto monitored_it = snapshot.id_map.find(monitored_bus);
        if (monitored_it == snapshot.id_map.end()) {
          throw std::runtime_error("monitored_bus_lookup_failed");
        }
        const int monitored_bus_pos = monitored_it->second;
        const auto& monitored_ac_bus =
            working.buses[static_cast<std::size_t>(monitored_bus_pos)];
        if (!monitored_ac_bus.phase_mask.has(monitored_phase)) {
          throw std::runtime_error("monitored_phase_missing_from_bus");
        }

        const Complex monitored_voltage_pu = phase_from_bus_voltage_result(
            current_result.bus_voltages[static_cast<std::size_t>(monitored_bus_pos)],
            monitored_phase);
        const Complex monitored_voltage =
            monitored_voltage_pu * bus_base_voltage_volts(monitored_ac_bus);
        const double monitored_voltage_volts = std::abs(monitored_voltage);
        const double effective_ptratio =
            (control.monitored_bus != 0 && control.remote_ptratio > 0.0)
                ? control.remote_ptratio
                : control.ptratio;
        if (effective_ptratio <= kPhaseValueTol) {
          throw std::runtime_error("invalid_ptratio");
        }

        const int observation_pos = find_transformer_observation_position(
            current_result.transformer_terminal_observations,
            transformer.index);
        const auto& observation =
            current_result.transformer_terminal_observations[static_cast<std::size_t>(observation_pos)];

        const Complex tap_winding_current_amps =
            (control.tap_winding == 1)
                ? phase_from_observation(observation.hv_current_amps, transformer_phase)
                : phase_from_observation(observation.lv_current_amps, transformer_phase);

        Complex ldc_term{0.0, 0.0};
        if (std::abs(control.r_volts) > kPhaseValueTol ||
            std::abs(control.x_volts) > kPhaseValueTol) {
          if (control.ct_primary_amps <= kPhaseValueTol) {
            throw std::runtime_error("invalid_ct_primary_amps");
          }
          ldc_term = (tap_winding_current_amps / control.ct_primary_amps) *
                     Complex(control.r_volts, control.x_volts);
        }

        const Complex control_voltage =
            monitored_voltage / effective_ptratio + ldc_term;
        const double monitored_voltage_magnitude_pu = std::abs(monitored_voltage_pu);
        const double control_voltage_volts = std::abs(control_voltage);
        const double band_half = control.band_volts / 2.0;
        const double lower_band = control.vreg_volts - band_half;
        const double upper_band = control.vreg_volts + band_half;

        state.final_tap_pos = transformer.tap_pos;
        state.final_tap_number = transformer_tap_number(transformer);
        state.final_tap_pu = transformer_tap_pu(transformer);
        state.monitored_voltage_pu = monitored_voltage_magnitude_pu;
        state.monitored_voltage_volts = monitored_voltage_volts;
        state.control_voltage_volts = control_voltage_volts;
        state.tap_winding_current_amps = std::abs(tap_winding_current_amps);
        state.line_drop_compensation_real_volts = ldc_term.real();
        state.line_drop_compensation_imag_volts = ldc_term.imag();
        state.line_drop_compensation_magnitude_volts = std::abs(ldc_term);

        trace.monitored_voltage_pu = monitored_voltage_magnitude_pu;
        trace.monitored_voltage_volts = monitored_voltage_volts;
        trace.control_voltage_volts = control_voltage_volts;
        trace.tap_winding_current_amps = std::abs(tap_winding_current_amps);
        trace.line_drop_compensation_real_volts = ldc_term.real();
        trace.line_drop_compensation_imag_volts = ldc_term.imag();
        trace.line_drop_compensation_magnitude_volts = std::abs(ldc_term);

        int next_tap_pos = transformer.tap_pos;
        if (control_voltage_volts < lower_band - 1e-6) {
          trace.decision_reason = "raise_voltage";
          next_tap_pos = std::clamp(
              transformer.tap_pos + tap_pos_delta_for_raise_voltage(transformer),
              transformer.tap_min,
              transformer.tap_max);
          if (next_tap_pos == transformer.tap_pos) {
            throw std::runtime_error("tap_limit_reached_below_band");
          }
        } else if (control_voltage_volts > upper_band + 1e-6) {
          trace.decision_reason = "lower_voltage";
          next_tap_pos = std::clamp(
              transformer.tap_pos - tap_pos_delta_for_raise_voltage(transformer),
              transformer.tap_min,
              transformer.tap_max);
          if (next_tap_pos == transformer.tap_pos) {
            throw std::runtime_error("tap_limit_reached_above_band");
          }
        } else {
          trace.decision_reason = "within_band";
          trace.next_tap_pos = transformer.tap_pos;
          trace.next_tap_number = transformer_tap_number(transformer);
          trace.stop_reason = "within_band";
          state.converged = true;
          state.stop_reason = "within_band";
          regulator_states.push_back(state);
          continue;
        }

        trace.next_tap_pos = next_tap_pos;
        trace.next_tap_number = next_tap_pos - transformer.tap_neutral;
        trace.stop_reason = "continue_control_loop";
        state.converged = false;
        state.stop_reason = "tap_change_requested";

        working.transformers[static_cast<std::size_t>(transformer_pos)].tap_pos =
            next_tap_pos;
        any_tap_changed = true;
      } catch (const std::exception& ex) {
        control_failed = true;
        trace.decision_reason = "blocked";
        trace.next_tap_pos = trace.current_tap_pos;
        trace.next_tap_number = trace.current_tap_number;
        trace.stop_reason = ex.what();
        state.converged = false;
        state.stop_reason = ex.what();
      }

      regulator_states.push_back(state);
    }

    current_result.regulator_trace = accumulated_trace;
    current_result.regulator_states = regulator_states;

    if (control_failed) {
      current_result.control_converged = false;
      current_result.converged = false;
      final_result = std::move(current_result);
      finalized = true;
      break;
    }

    if (!any_tap_changed) {
      current_result.control_converged = true;
      final_result = std::move(current_result);
      finalized = true;
      break;
    }

    const auto next_tap_state = capture_tap_state();
    if (!seen_tap_states.insert(next_tap_state).second) {
      for (const auto trace_pos : iteration_trace_positions) {
        if (accumulated_trace[trace_pos].stop_reason == "continue_control_loop") {
          accumulated_trace[trace_pos].stop_reason = "cycle_detected";
        }
      }
      for (auto& state : current_result.regulator_states) {
        if (state.stop_reason == "tap_change_requested") {
          state.stop_reason = "cycle_detected";
          state.converged = false;
        }
      }
      current_result.regulator_trace = accumulated_trace;
      current_result.control_converged = false;
      current_result.converged = false;
      final_result = std::move(current_result);
      finalized = true;
      break;
    }

    if (control_iter == opt.max_control_iter) {
      for (const auto trace_pos : iteration_trace_positions) {
        if (accumulated_trace[trace_pos].stop_reason == "continue_control_loop") {
          accumulated_trace[trace_pos].stop_reason = "max_control_iter_reached";
        }
      }
      for (auto& state : current_result.regulator_states) {
        if (state.stop_reason == "tap_change_requested") {
          state.stop_reason = "max_control_iter_reached";
          state.converged = false;
        }
      }
      current_result.regulator_trace = accumulated_trace;
      current_result.control_converged = false;
      current_result.converged = false;
      final_result = std::move(current_result);
      finalized = true;
      break;
    }
  }

  if (!finalized) {
    throw std::runtime_error(
        "solve_three_phase_nr: regulator control loop did not finalize");
  }

  return final_result;
}

// =========================================================================
// solve_three_phase_nr (HybridPowerSystem)
// =========================================================================
ThreePhaseDPFResult solve_three_phase_nr(
    const HybridPowerSystem& sys,
    const ThreePhaseNROptions& opt) {
  if (!sys.three_phase_ac.has_value())
    throw std::runtime_error(
        "solve_three_phase_nr: no three_phase_ac subsystem present");
  return solve_three_phase_nr(sys.three_phase_ac.value(), opt);
}

std::vector<PhaseDomainSparseEntry> build_full_ybus_phase_entries(
    const ThreePhaseACSystem& sys,
    bool include_shunts) {
  return sparse_entries_from_matrix(
      build_full_phase_matrix_model(sys, include_shunts).ybus);
}

std::vector<PhaseDomainSparseEntry> build_full_ybus_phase_entries(
    const HybridPowerSystem& sys,
    bool include_shunts) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "build_full_ybus_phase_entries: no three_phase_ac subsystem present");
  }
  return build_full_ybus_phase_entries(*sys.three_phase_ac, include_shunts);
}

ThreePhaseCompactPFData build_compact_pf_data(
    const ThreePhaseACSystem& sys,
    bool include_shunts) {
  const CompactFixedPointContext context =
      build_compact_fixed_point_context(sys, include_shunts);

  ThreePhaseCompactPFData data;
  data.indexer = context.indexer;
  data.fixed_indices = context.fixed_indices;
  data.variable_indices = context.variable_indices;
  data.fixed_voltage.assign(
      context.fixed_voltage.data(),
      context.fixed_voltage.data() + context.fixed_voltage.size());
  data.fixed_current.assign(
      context.network_model.fixed_current.data(),
      context.network_model.fixed_current.data() +
          context.network_model.fixed_current.size());
  data.ybus_entries = sparse_entries_from_matrix(context.network_model.ybus);
  data.y_vv_entries = sparse_entries_from_matrix(context.y_vv);
  data.y_vf_entries = sparse_entries_from_matrix(context.y_vf);
  return data;
}

ThreePhaseCompactPFData build_compact_pf_data(
    const HybridPowerSystem& sys,
    bool include_shunts) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "build_compact_pf_data: no three_phase_ac subsystem present");
  }
  return build_compact_pf_data(*sys.three_phase_ac, include_shunts);
}

ThreePhaseDPFResult solve_three_phase_compact_pf(
    const ThreePhaseACSystem& sys,
    const ThreePhaseFixedPointOptions& opt) {
  const CompactFixedPointContext context =
      build_compact_fixed_point_context(sys, opt.include_shunts);
  if (context.variable_indices.empty()) {
    Eigen::VectorXd vm = Eigen::VectorXd::Zero(context.indexer.total_nodes);
    Eigen::VectorXd va = Eigen::VectorXd::Zero(context.indexer.total_nodes);
    for (int node = 0; node < context.indexer.total_nodes; ++node) {
      const PhaseNodeRef& ref = context.indexer.node_ref(node);
      const auto vm_values = bus_vm_values(sys.buses[static_cast<std::size_t>(ref.bus_offset)]);
      const auto va_values = bus_va_values_deg(sys.buses[static_cast<std::size_t>(ref.bus_offset)]);
      vm[node] = vm_values[static_cast<std::size_t>(ref.phase_index)];
      va[node] = va_values[static_cast<std::size_t>(ref.phase_index)] * kPi / 180.0;
    }
    return finalize_three_phase_result_from_state(
        sys,
        context.id_map,
        context.indexer,
        context.network_model.transformer_primitives,
        vm,
        va,
        true,
        0,
        0.0,
        context.base_mva);
  }

  Eigen::SparseLU<Eigen::SparseMatrix<Complex>> lu;
  factorize_compact_matrix(
      context.y_vv,
      "solve_three_phase_compact_pf",
      lu);

  ComplexVectorX rhs =
      gather_complex_entries(
          context.network_model.fixed_current,
          context.variable_indices) -
      context.y_vf * context.fixed_voltage;
  ComplexVectorX no_load_voltage = lu.solve(rhs);
  if (lu.info() != Eigen::Success || no_load_voltage.size() != rhs.size()) {
    throw std::runtime_error(
        "solve_three_phase_compact_pf: failed to compute no-load initial guess");
  }
  ComplexVectorX variable_voltage = no_load_voltage;
  for (std::size_t pos = 0; pos < context.variable_indices.size(); ++pos) {
    const int node = context.variable_indices[pos];
    const PhaseNodeRef& ref = context.indexer.node_ref(node);
    const auto& bus = sys.buses[static_cast<std::size_t>(ref.bus_offset)];
    if (!bus_phase_has_explicit_voltage_guess(bus, ref.phase_index)) continue;
    variable_voltage(static_cast<int>(pos)) =
        bus_phase_voltage_guess_pu(bus, ref.phase_index);
  }

  bool converged = false;
  int iterations = 0;
  double residual = 0.0;

  for (int iter = 1; iter <= opt.max_iter; ++iter) {
    ComplexVectorX full_voltage = ComplexVectorX::Zero(context.indexer.total_nodes);
    for (std::size_t pos = 0; pos < context.fixed_indices.size(); ++pos) {
      full_voltage(context.fixed_indices[pos]) =
          context.fixed_voltage(static_cast<int>(pos));
    }
    for (std::size_t pos = 0; pos < context.variable_indices.size(); ++pos) {
      full_voltage(context.variable_indices[pos]) =
          variable_voltage(static_cast<int>(pos));
    }

    const ComplexVectorX current =
        evaluate_fixed_point_current_injections(context, full_voltage);
    const ComplexVectorX variable_rhs =
        gather_complex_entries(current, context.variable_indices) -
        context.y_vf * context.fixed_voltage;
    const ComplexVectorX variable_candidate = lu.solve(variable_rhs);
    if (lu.info() != Eigen::Success) {
      throw std::runtime_error(
          "solve_three_phase_compact_pf: linear solve failed during iteration");
    }

    residual = 0.0;
    for (int local = 0; local < variable_candidate.size(); ++local) {
      residual = std::max(
          residual,
          std::abs(variable_candidate(local) - variable_voltage(local)));
    }

    const double alpha = (iter <= 5) ? 0.3 : ((iter <= 20) ? 0.5 : 0.8);
    variable_voltage =
        alpha * variable_candidate + (1.0 - alpha) * variable_voltage;
    iterations = iter;

    if (residual < opt.tol && iter > 1) {
      converged = true;
      break;
    }
  }

  ComplexVectorX full_voltage = ComplexVectorX::Zero(context.indexer.total_nodes);
  for (std::size_t pos = 0; pos < context.fixed_indices.size(); ++pos) {
    full_voltage(context.fixed_indices[pos]) =
        context.fixed_voltage(static_cast<int>(pos));
  }
  for (std::size_t pos = 0; pos < context.variable_indices.size(); ++pos) {
    full_voltage(context.variable_indices[pos]) =
        variable_voltage(static_cast<int>(pos));
  }

  Eigen::VectorXd vm(context.indexer.total_nodes);
  Eigen::VectorXd va(context.indexer.total_nodes);
  for (int node = 0; node < context.indexer.total_nodes; ++node) {
    vm[node] = std::abs(full_voltage(node));
    va[node] = std::arg(full_voltage(node));
  }

  return finalize_three_phase_result_from_state(
      sys,
      context.id_map,
      context.indexer,
      context.network_model.transformer_primitives,
      vm,
      va,
      converged,
      iterations,
      residual,
      context.base_mva);
}

ThreePhaseDPFResult solve_three_phase_compact_pf(
    const HybridPowerSystem& sys,
    const ThreePhaseFixedPointOptions& opt) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "solve_three_phase_compact_pf: no three_phase_ac subsystem present");
  }
  return solve_three_phase_compact_pf(*sys.three_phase_ac, opt);
}

ThreePhaseDPFResult solve_three_phase_fixed_point(
    const ThreePhaseACSystem& sys,
    const ThreePhaseFixedPointOptions& opt) {
  const CompactFixedPointContext compact_context =
      build_compact_fixed_point_context(sys, opt.include_shunts);
  const FullPhaseMatrixModel full_model =
      build_full_phase_matrix_model(sys, opt.include_shunts);

  if (full_model.variable_indices.empty()) {
    Eigen::VectorXd vm = Eigen::VectorXd::Zero(compact_context.indexer.total_nodes);
    Eigen::VectorXd va = Eigen::VectorXd::Zero(compact_context.indexer.total_nodes);
    for (int node = 0; node < compact_context.indexer.total_nodes; ++node) {
      const PhaseNodeRef& ref = compact_context.indexer.node_ref(node);
      const auto vm_values = bus_vm_values(sys.buses[static_cast<std::size_t>(ref.bus_offset)]);
      const auto va_values = bus_va_values_deg(sys.buses[static_cast<std::size_t>(ref.bus_offset)]);
      vm[node] = vm_values[static_cast<std::size_t>(ref.phase_index)];
      va[node] = va_values[static_cast<std::size_t>(ref.phase_index)] * kPi / 180.0;
    }
    return finalize_three_phase_result_from_state(
        sys,
        compact_context.id_map,
        compact_context.indexer,
        compact_context.network_model.transformer_primitives,
        vm,
        va,
        true,
        0,
        0.0,
        compact_context.base_mva);
  }

  const Eigen::SparseMatrix<Complex> y_vv = extract_sparse_submatrix(
      full_model.ybus,
      full_model.variable_indices,
      full_model.variable_indices);
  const Eigen::SparseMatrix<Complex> y_vf = extract_sparse_submatrix(
      full_model.ybus,
      full_model.variable_indices,
      full_model.fixed_indices);

  Eigen::SparseLU<Eigen::SparseMatrix<Complex>> lu;
  factorize_compact_matrix(
      y_vv,
      "solve_three_phase_fixed_point",
      lu);

  ComplexVectorX rhs =
      gather_complex_entries(full_model.fixed_current, full_model.variable_indices) -
      y_vf * full_model.fixed_voltage;
  ComplexVectorX no_load_voltage = lu.solve(rhs);
  if (lu.info() != Eigen::Success || no_load_voltage.size() != rhs.size()) {
    throw std::runtime_error(
        "solve_three_phase_fixed_point: failed to compute no-load initial guess");
  }
  ComplexVectorX variable_voltage = no_load_voltage;
  for (std::size_t pos = 0; pos < full_model.variable_indices.size(); ++pos) {
    const int full_index = full_model.variable_indices[pos];
    const int bus_offset = full_index / 3;
    const int phase_index = full_index % 3;
    const auto& bus = sys.buses[static_cast<std::size_t>(bus_offset)];
    if (!bus_phase_has_explicit_voltage_guess(bus, phase_index)) continue;
    variable_voltage(static_cast<int>(pos)) =
        bus_phase_voltage_guess_pu(bus, phase_index);
  }

  const std::array<double, 7> load_scales = {0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0};
  bool converged = false;
  int total_iterations = 0;
  double residual = 0.0;
  double last_converged_scale = 0.0;

  for (const double load_scale : load_scales) {
    const PhaseSpec stage_spec = compute_phase_spec(
        sys,
        compact_context.id_map,
        compact_context.indexer,
        compact_context.base_mva,
        load_scale);
    const PreparedLoadSet stage_loads =
        scale_dynamic_load_set(compact_context.prepared_loads, load_scale);

    const int stage_max_iter =
        (load_scale >= 0.999) ? opt.max_iter : std::min(500, opt.max_iter);
    bool stage_converged = false;

    for (int iter = 1; iter <= stage_max_iter; ++iter) {
      ComplexVectorX full_voltage = ComplexVectorX::Zero(full_model.ybus.rows());
      for (std::size_t pos = 0; pos < full_model.fixed_indices.size(); ++pos) {
        full_voltage(full_model.fixed_indices[pos]) =
            full_model.fixed_voltage(static_cast<int>(pos));
      }
      for (std::size_t pos = 0; pos < full_model.variable_indices.size(); ++pos) {
        full_voltage(full_model.variable_indices[pos]) =
            variable_voltage(static_cast<int>(pos));
      }

      const ComplexVectorX compact_voltage =
          gather_full_voltage_to_compact(full_model, full_voltage);
      const ComplexVectorX compact_current =
          evaluate_fixed_point_current_injections(
              stage_loads,
              stage_spec,
              compact_context.network_model.fixed_current,
              compact_voltage);
      const ComplexVectorX full_current =
          scatter_compact_current_to_full(full_model, compact_current);

      const ComplexVectorX variable_rhs =
          gather_complex_entries(full_current, full_model.variable_indices) -
          y_vf * full_model.fixed_voltage;
      const ComplexVectorX variable_candidate = lu.solve(variable_rhs);
      if (lu.info() != Eigen::Success) {
        throw std::runtime_error(
            "solve_three_phase_fixed_point: linear solve failed during iteration");
      }

      residual = 0.0;
      for (int local = 0; local < variable_candidate.size(); ++local) {
        residual = std::max(
            residual,
            std::abs(variable_candidate(local) - variable_voltage(local)));
      }

      const double alpha =
          (iter <= 10) ? 0.2 : ((iter <= 30) ? 0.3 : ((iter <= 60) ? 0.5 : 0.7));
      ComplexVectorX variable_update =
          alpha * variable_candidate + (1.0 - alpha) * variable_voltage;

      const double vm_min =
          (load_scale > 0.3) ? 0.1 : ((load_scale > 0.1) ? 0.2 : 0.3);
      const double vm_max = 1.5;
      for (int local = 0; local < variable_update.size(); ++local) {
        const double vm_mag = std::abs(variable_update(local));
        if (vm_mag < 1e-12) {
          const int full_index = full_model.variable_indices[static_cast<std::size_t>(local)];
          const int phase = full_index % 3;
          const double angle =
              (phase == 0) ? 0.0 : ((phase == 1) ? -2.0 * kPi / 3.0 : 2.0 * kPi / 3.0);
          variable_update(local) = std::polar(vm_min, angle);
          continue;
        }
        if (vm_mag < vm_min) {
          variable_update(local) *= vm_min / vm_mag;
        } else if (vm_mag > vm_max) {
          variable_update(local) *= vm_max / vm_mag;
        }
      }

      variable_voltage = variable_update;
      ++total_iterations;
      if (residual < opt.tol) {
        stage_converged = true;
        break;
      }
    }

    if (!stage_converged) {
      converged = false;
      break;
    }

    last_converged_scale = load_scale;
    converged = load_scale >= 0.999;
    if (converged) break;
  }

  ComplexVectorX final_full_voltage = ComplexVectorX::Zero(full_model.ybus.rows());
  for (std::size_t pos = 0; pos < full_model.fixed_indices.size(); ++pos) {
    final_full_voltage(full_model.fixed_indices[pos]) =
        full_model.fixed_voltage(static_cast<int>(pos));
  }
  for (std::size_t pos = 0; pos < full_model.variable_indices.size(); ++pos) {
    final_full_voltage(full_model.variable_indices[pos]) =
        variable_voltage(static_cast<int>(pos));
  }

  const ComplexVectorX final_compact_voltage =
      gather_full_voltage_to_compact(full_model, final_full_voltage);
  Eigen::VectorXd vm(compact_context.indexer.total_nodes);
  Eigen::VectorXd va(compact_context.indexer.total_nodes);
  for (int node = 0; node < compact_context.indexer.total_nodes; ++node) {
    vm[node] = std::abs(final_compact_voltage(node));
    va[node] = std::arg(final_compact_voltage(node));
  }

  if (!converged && last_converged_scale > 0.0 && opt.verbose) {
    HACDCPF_LOG_WARN(
        "solve_three_phase_fixed_point stopped at {:.1f}% load continuation",
        last_converged_scale * 100.0);
  }

  return finalize_three_phase_result_from_state(
      sys,
      compact_context.id_map,
      compact_context.indexer,
      compact_context.network_model.transformer_primitives,
      vm,
      va,
      converged,
      total_iterations,
      residual,
      compact_context.base_mva);
}

ThreePhaseDPFResult solve_three_phase_fixed_point(
    const HybridPowerSystem& sys,
    const ThreePhaseFixedPointOptions& opt) {
  if (!sys.three_phase_ac.has_value()) {
    throw std::runtime_error(
        "solve_three_phase_fixed_point: no three_phase_ac subsystem present");
  }
  return solve_three_phase_fixed_point(*sys.three_phase_ac, opt);
}

}  // namespace hacdcpf::analysis
