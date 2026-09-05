#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <Eigen/SparseQR>

#include "hacdcpf/engine/engine.hpp"
#include <mipsolvers/engine/kernel/ipm/ipm_solver.hpp>

namespace hacdcpf::opf::phase_hybrid {
namespace {

using Complex = std::complex<double>;
using Triplet = Eigen::Triplet<double>;

constexpr double kPi = 3.14159265358979323846;

struct Layout {
  int nv{0};
  int ng{0};
  int ndc{0};
  int ncp{0};
  int nc{0};
  int ngfm{0};
  int i_e{0};
  int i_f{0};
  int i_pg{0};
  int i_qg{0};
  int i_udc{0};
  int i_pac{0};
  int i_qac{0};
  int i_gfm_e{0};
  int i_gfm_f{0};
  int i_pdc{0};
  int nvar{0};
  int neq{0};
  int nineq{0};
};

struct ConverterMap {
  std::vector<int> model_phase_nodes;
  std::vector<int> phase_var_positions;
  std::array<int, 3> model_node_by_phase{{-1, -1, -1}};
  std::array<int, 3> phase_var_by_phase{{-1, -1, -1}};
  int gfm_var_position{-1};
  int dc_terminal{-1};
  double efficiency{0.98};
  double s_max_pu{0.0};
  double phase_current_max_pu{0.0};
  bool fixed_unity_power_factor{false};
  PhaseVSCControlMode control_mode{PhaseVSCControlMode::EqualPhasePower};
  double nominal_frequency_hz{50.0};
  double pll_kp{0.01};
  double pll_ki{1.0};
  double virtual_r_pu{0.0};
  double virtual_x_pu{0.10};
  double p_droop_pu{0.01};
  double q_droop_pu{0.05};
  double voltage_reference_pu{1.0};
  double voltage_reference_angle_rad{0.0};
  double voltage_integral_gain{10.0};
};

struct ParametricHessianTerm {
  int row{0};
  int col{0};
  int multiplier_row_1{0};
  int multiplier_row_2{-1};
  double coefficient_1{0.0};
  double coefficient_2{0.0};
};

struct ModelData {
  const ThreePhaseHybridOPFCase* source{nullptr};
  bool verbose{false};
  graph::SparseComplexMatrix y;
  graph::SparseKronResult reduction;
  Eigen::VectorXcd fixed_current;
  Eigen::SparseMatrix<Complex> recovery;
  std::vector<std::vector<std::pair<int, Complex>>> recovery_rows;
  std::vector<int> full_to_model;
  std::vector<int> generator_nodes;
  std::vector<ConverterMap> converters;
  std::vector<int> converter_for_phase_var;
  std::vector<int> converter_local_phase;
  std::vector<int> reference_model_nodes;
  std::vector<int> enforced_inequality_rows;
  std::vector<ParametricHessianTerm> inequality_hessian_terms;
  std::vector<ParametricHessianTerm> enforced_inequality_hessian_terms;
  Eigen::VectorXd equality_scale;
  Layout layout;
};

void validate_case(const ThreePhaseHybridOPFCase& c) {
  const int n = static_cast<int>(c.y_ac.rows());
  if (n <= 0 || c.y_ac.cols() != n) {
    throw std::invalid_argument("three-phase hybrid OPF requires a square AC Y-bus");
  }
  const auto require_n = [n](const Eigen::VectorXd& v, const char* name) {
    if (v.size() != n) {
      throw std::invalid_argument(std::string(name) + " dimension mismatch");
    }
  };
  require_n(c.p_load_pu, "p_load_pu");
  require_n(c.q_load_pu, "q_load_pu");
  require_n(c.v_min_pu, "v_min_pu");
  require_n(c.v_max_pu, "v_max_pu");
  if (c.i_ac_fixed.size() != n) {
    throw std::invalid_argument("i_ac_fixed dimension mismatch");
  }
  if (c.voltage_start.size() != n) {
    throw std::invalid_argument("voltage_start dimension mismatch");
  }
  if (!c.ac_phase_index.empty()) {
    if (static_cast<int>(c.ac_phase_index.size()) != n ||
        std::any_of(c.ac_phase_index.begin(), c.ac_phase_index.end(),
                    [](int phase) { return phase < 0 || phase > 2; })) {
      throw std::invalid_argument("ac_phase_index is invalid");
    }
  }
  if (c.reference_voltage.size() !=
      static_cast<int>(c.reference_nodes.size())) {
    throw std::invalid_argument("reference voltage dimension mismatch");
  }
  const int ndc = static_cast<int>(c.g_dc.rows());
  if (c.g_dc.cols() != ndc || c.p_dc_load_pu.size() != ndc ||
      c.v_dc_start.size() != ndc || c.v_dc_min_pu.size() != ndc ||
      c.v_dc_max_pu.size() != ndc) {
    throw std::invalid_argument("DC model dimension mismatch");
  }
  if (c.dc_reference_voltage_pu.size() !=
      static_cast<int>(c.dc_reference_terminals.size())) {
    throw std::invalid_argument("DC reference voltage dimension mismatch");
  }
  for (int terminal : c.dc_reference_terminals) {
    if (terminal < 0 || terminal >= ndc) {
      throw std::invalid_argument("DC reference terminal is out of range");
    }
  }
  for (const auto& generator : c.generators) {
    if (generator.phase_node < 0 || generator.phase_node >= n) {
      throw std::invalid_argument("generator phase node is out of range");
    }
  }
  for (const auto& converter : c.converters) {
    if (converter.phase_nodes.empty() || converter.dc_terminal < 0 ||
        converter.dc_terminal >= ndc) {
      throw std::invalid_argument("converter terminal definition is invalid");
    }
    for (int node : converter.phase_nodes) {
      if (node < 0 || node >= n) {
        throw std::invalid_argument("converter phase node is out of range");
      }
    }
    if (converter.control_mode != PhaseVSCControlMode::EqualPhasePower) {
      if (converter.phase_nodes.size() != 3 || c.ac_phase_index.empty()) {
        throw std::invalid_argument(
            "sequence-aware converter control requires one a/b/c terminal");
      }
      std::array<bool, 3> present{};
      for (int node : converter.phase_nodes) {
        present[static_cast<std::size_t>(
            c.ac_phase_index[static_cast<std::size_t>(node)])] = true;
      }
      if (!present[0] || !present[1] || !present[2]) {
        throw std::invalid_argument(
            "sequence-aware converter control requires distinct a/b/c phases");
      }
    }
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop &&
        std::abs(converter.virtual_r_pu) + std::abs(converter.virtual_x_pu) <=
            1e-8) {
      throw std::invalid_argument(
          "grid-forming converter requires nonzero virtual impedance");
    }
  }
}

Eigen::SparseMatrix<Complex> identity_recovery(int n) {
  Eigen::SparseMatrix<Complex> t(n, n);
  t.setIdentity();
  return t;
}

std::vector<std::pair<int, Complex>> complex_linear_map(
    const ModelData& d,
    const std::vector<int>& full_nodes,
    const std::vector<Complex>& coefficients) {
  std::unordered_map<int, Complex> accumulated;
  for (int k = 0; k < static_cast<int>(full_nodes.size()); ++k) {
    const int full_node = full_nodes[static_cast<std::size_t>(k)];
    const Complex factor = coefficients[static_cast<std::size_t>(k)];
    for (const auto& [model_node, recovery] :
         d.recovery_rows[static_cast<std::size_t>(full_node)]) {
      accumulated[d.layout.i_e + model_node] += factor * recovery;
      accumulated[d.layout.i_f + model_node] +=
          Complex{0.0, 1.0} * factor * recovery;
    }
  }
  std::vector<std::pair<int, Complex>> result;
  result.reserve(accumulated.size());
  for (const auto& entry : accumulated) {
    if (entry.second != Complex{0.0, 0.0}) result.push_back(entry);
  }
  std::sort(result.begin(), result.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.first < rhs.first;
            });
  return result;
}

void append_parametric_norm_hessian(
    std::vector<ParametricHessianTerm>& terms,
    const std::vector<std::pair<int, Complex>>& map,
    int multiplier_row_1,
    double multiplier_scale_1,
    int multiplier_row_2 = -1,
    double multiplier_scale_2 = 0.0) {
  for (auto first = map.begin(); first != map.end(); ++first) {
    for (auto second = first; second != map.end(); ++second) {
      const double norm_coefficient = 2.0 *
          std::real(std::conj(first->second) * second->second);
      if (norm_coefficient == 0.0) continue;
      terms.push_back(
          {first->first, second->first,
           multiplier_row_1, multiplier_row_2,
           multiplier_scale_1 * norm_coefficient,
           multiplier_scale_2 * norm_coefficient});
      if (first->first != second->first) {
        terms.push_back(
            {second->first, first->first,
             multiplier_row_1, multiplier_row_2,
             multiplier_scale_1 * norm_coefficient,
             multiplier_scale_2 * norm_coefficient});
      }
    }
  }
}

void build_inequality_hessian_template(ModelData& d) {
  const int full_n = d.reduction.original_size;
  const Complex one{1.0, 0.0};
  for (int node = 0; node < full_n; ++node) {
    const auto map = complex_linear_map(d, {node}, {one});
    append_parametric_norm_hessian(
        d.inequality_hessian_terms, map, node, -1.0,
        full_n + node, 1.0);
  }

  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  const std::vector<Complex> positive{
      Complex{1.0 / 3.0, 0.0}, a / 3.0, a2 / 3.0};
  const std::vector<Complex> negative{
      Complex{1.0 / 3.0, 0.0}, a2 / 3.0, a / 3.0};
  int inequality_row = 2 * full_n;
  for (const auto& nodes : d.source->three_phase_bus_nodes) {
    append_parametric_norm_hessian(
        d.inequality_hessian_terms,
        complex_linear_map(d, nodes, negative), inequality_row, 1.0);
    append_parametric_norm_hessian(
        d.inequality_hessian_terms,
        complex_linear_map(d, nodes, positive), inequality_row,
        -d.source->vuf_max * d.source->vuf_max);
    ++inequality_row;
  }
  for (const auto& converter : d.converters) {
    for (int pos : converter.phase_var_positions) {
      d.inequality_hessian_terms.push_back(
          {d.layout.i_pac + pos, d.layout.i_pac + pos,
           inequality_row, -1, 2.0, 0.0});
      d.inequality_hessian_terms.push_back(
          {d.layout.i_qac + pos, d.layout.i_qac + pos,
           inequality_row, -1, 2.0, 0.0});
    }
    ++inequality_row;
  }
  for (const auto& converter : d.converters) {
    for (int local = 0;
         local < static_cast<int>(converter.phase_var_positions.size());
         ++local) {
      const int pos =
          converter.phase_var_positions[static_cast<std::size_t>(local)];
      d.inequality_hessian_terms.push_back(
          {d.layout.i_pac + pos, d.layout.i_pac + pos,
           inequality_row, -1, 2.0, 0.0});
      d.inequality_hessian_terms.push_back(
          {d.layout.i_qac + pos, d.layout.i_qac + pos,
           inequality_row, -1, 2.0, 0.0});
      const int full_node = d.reduction.retained[static_cast<std::size_t>(
          converter.model_phase_nodes[static_cast<std::size_t>(local)])];
      append_parametric_norm_hessian(
          d.inequality_hessian_terms,
          complex_linear_map(d, {full_node}, {one}), inequality_row,
          -converter.phase_current_max_pu * converter.phase_current_max_pu);
      ++inequality_row;
    }
  }
  for (const auto& limit : d.source->ac_line_limits) {
    append_parametric_norm_hessian(
        d.inequality_hessian_terms,
        complex_linear_map(d, limit.nodes, limit.coefficients),
        inequality_row, 1.0);
    ++inequality_row;
  }
  for (const auto& limit : d.source->dc_line_limits) {
    const int fi = d.layout.i_udc + limit.from_node;
    const int ti = d.layout.i_udc + limit.to_node;
    const double g2 = 2.0 * limit.conductance_pu * limit.conductance_pu;
    d.inequality_hessian_terms.push_back({fi, fi, inequality_row, -1, g2, 0.0});
    d.inequality_hessian_terms.push_back({ti, ti, inequality_row, -1, g2, 0.0});
    d.inequality_hessian_terms.push_back({fi, ti, inequality_row, -1, -g2, 0.0});
    d.inequality_hessian_terms.push_back({ti, fi, inequality_row, -1, -g2, 0.0});
    ++inequality_row;
  }
}

void configure_enforced_inequalities(ModelData& data,
                                     const std::vector<int>& requested_rows) {
  const int nineq = data.layout.nineq;
  data.enforced_inequality_rows.clear();
  if (requested_rows.empty()) {
    data.enforced_inequality_rows.reserve(static_cast<std::size_t>(nineq));
    for (int row = 0; row < nineq; ++row) {
      data.enforced_inequality_rows.push_back(row);
    }
  } else {
    data.enforced_inequality_rows = requested_rows;
    std::sort(data.enforced_inequality_rows.begin(),
              data.enforced_inequality_rows.end());
    const auto duplicate = std::adjacent_find(
        data.enforced_inequality_rows.begin(),
        data.enforced_inequality_rows.end());
    if (duplicate != data.enforced_inequality_rows.end() ||
        data.enforced_inequality_rows.front() < 0 ||
        data.enforced_inequality_rows.back() >= nineq) {
      throw std::invalid_argument(
          "enforced nonlinear inequality rows are invalid");
    }
  }

  std::vector<bool> enforced_mask(static_cast<std::size_t>(nineq), false);
  for (int row : data.enforced_inequality_rows) {
    enforced_mask[static_cast<std::size_t>(row)] = true;
  }
  data.enforced_inequality_hessian_terms.clear();
  data.enforced_inequality_hessian_terms.reserve(
      data.inequality_hessian_terms.size());
  for (const auto& term : data.inequality_hessian_terms) {
    if (enforced_mask[static_cast<std::size_t>(term.multiplier_row_1)] ||
        (term.multiplier_row_2 >= 0 &&
         enforced_mask[static_cast<std::size_t>(term.multiplier_row_2)])) {
      data.enforced_inequality_hessian_terms.push_back(term);
    }
  }
}

std::shared_ptr<ModelData> build_model_data(
    const ThreePhaseHybridOPFCase& c,
    const ThreePhaseHybridOPFOptions& options) {
  validate_case(c);
  auto data = std::make_shared<ModelData>();
  data->source = &c;
  data->verbose = options.verbose;
  const int full_n = static_cast<int>(c.y_ac.rows());

  if (options.variant == ModelVariant::GraphReduced) {
    std::vector<bool> eligible(static_cast<std::size_t>(full_n), true);
    const auto retain = [&](int node) {
      eligible.at(static_cast<std::size_t>(node)) = false;
    };
    for (int node = 0; node < full_n; ++node) {
      if (std::abs(c.p_load_pu[node]) > 1e-12 ||
          std::abs(c.q_load_pu[node]) > 1e-12 ||
          std::abs(c.i_ac_fixed[node]) > 1e-12) {
        retain(node);
      }
    }
    for (const auto& generator : c.generators) retain(generator.phase_node);
    for (const auto& converter : c.converters) {
      for (int node : converter.phase_nodes) retain(node);
    }
    for (int node : c.reference_nodes) retain(node);
    data->reduction =
        graph::reduce_sparse_kron(c.y_ac, eligible, options.reduction_options);
    if (!data->reduction.valid()) {
      throw std::runtime_error("sparse graph reduction failed: " +
                               data->reduction.error);
    }
    data->y = data->reduction.reduced;
    data->recovery = graph::sparse_kron_recovery_operator(data->reduction);
  } else {
    data->y = c.y_ac;
    data->reduction.original_size = full_n;
    data->reduction.original_nonzeros = static_cast<int>(c.y_ac.nonZeros());
    data->reduction.reduced = c.y_ac;
    data->reduction.retained.resize(static_cast<std::size_t>(full_n));
    for (int i = 0; i < full_n; ++i) data->reduction.retained[i] = i;
    data->recovery = identity_recovery(full_n);
  }

  data->fixed_current = Eigen::VectorXcd::Zero(data->y.rows());
  for (int pos = 0; pos < static_cast<int>(data->reduction.retained.size()); ++pos) {
    data->fixed_current[pos] =
        c.i_ac_fixed[data->reduction.retained[static_cast<std::size_t>(pos)]];
  }

  data->full_to_model.assign(static_cast<std::size_t>(full_n), -1);
  for (int pos = 0; pos < static_cast<int>(data->reduction.retained.size()); ++pos) {
    data->full_to_model[static_cast<std::size_t>(
        data->reduction.retained[static_cast<std::size_t>(pos)])] = pos;
  }

  data->recovery_rows.resize(static_cast<std::size_t>(full_n));
  for (int col = 0; col < data->recovery.outerSize(); ++col) {
    for (Eigen::SparseMatrix<Complex>::InnerIterator it(data->recovery, col);
         it; ++it) {
      data->recovery_rows[static_cast<std::size_t>(it.row())].emplace_back(
          it.col(), it.value());
    }
  }
  for (int full_node = 0; full_node < full_n; ++full_node) {
    if (data->recovery_rows[static_cast<std::size_t>(full_node)].empty()) {
      throw std::runtime_error(
          "graph-reduction certificate has an empty voltage-recovery row at "
          "phase node " + std::to_string(full_node) +
          "; remove reference-disconnected passive components before OPF");
    }
  }

  for (const auto& generator : c.generators) {
    const int node = data->full_to_model[static_cast<std::size_t>(
        generator.phase_node)];
    if (node < 0) throw std::runtime_error("generator node was eliminated");
    data->generator_nodes.push_back(node);
  }

  int phase_var = 0;
  int gfm_var = 0;
  for (int ci = 0; ci < static_cast<int>(c.converters.size()); ++ci) {
    const auto& converter = c.converters[static_cast<std::size_t>(ci)];
    ConverterMap map;
    map.dc_terminal = converter.dc_terminal;
    map.efficiency = std::clamp(converter.efficiency, 0.01, 1.0);
    map.s_max_pu = converter.s_max_pu;
    map.phase_current_max_pu = converter.phase_current_max_pu > 0.0
        ? converter.phase_current_max_pu
        : converter.s_max_pu /
              std::sqrt(static_cast<double>(converter.phase_nodes.size()));
    map.fixed_unity_power_factor = converter.fixed_unity_power_factor;
    map.control_mode = converter.control_mode;
    map.nominal_frequency_hz = converter.nominal_frequency_hz;
    map.pll_kp = converter.pll_kp;
    map.pll_ki = converter.pll_ki;
    map.virtual_r_pu = converter.virtual_r_pu;
    map.virtual_x_pu = converter.virtual_x_pu;
    map.p_droop_pu = converter.p_droop_pu;
    map.q_droop_pu = converter.q_droop_pu;
    map.voltage_reference_pu = converter.voltage_reference_pu;
    map.voltage_reference_angle_rad =
        converter.voltage_reference_angle_rad;
    map.voltage_integral_gain = converter.voltage_integral_gain;
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      map.gfm_var_position = gfm_var++;
    }
    for (int full_node : converter.phase_nodes) {
      const int node = data->full_to_model[static_cast<std::size_t>(full_node)];
      if (node < 0) throw std::runtime_error("converter node was eliminated");
      map.model_phase_nodes.push_back(node);
      map.phase_var_positions.push_back(phase_var++);
      if (!c.ac_phase_index.empty()) {
        const int phase = c.ac_phase_index[static_cast<std::size_t>(full_node)];
        map.model_node_by_phase[static_cast<std::size_t>(phase)] = node;
        map.phase_var_by_phase[static_cast<std::size_t>(phase)] = phase_var - 1;
      }
      data->converter_for_phase_var.push_back(ci);
      data->converter_local_phase.push_back(
          static_cast<int>(map.model_phase_nodes.size()) - 1);
    }
    data->converters.push_back(std::move(map));
  }

  for (int full_node : c.reference_nodes) {
    const int node = data->full_to_model[static_cast<std::size_t>(full_node)];
    if (node < 0) throw std::runtime_error("reference node was eliminated");
    data->reference_model_nodes.push_back(node);
  }

  Layout& l = data->layout;
  l.nv = static_cast<int>(data->y.rows());
  l.ng = static_cast<int>(c.generators.size());
  l.ndc = static_cast<int>(c.g_dc.rows());
  l.ncp = phase_var;
  l.nc = static_cast<int>(c.converters.size());
  l.ngfm = gfm_var;
  l.i_e = 0;
  l.i_f = l.i_e + l.nv;
  l.i_pg = l.i_f + l.nv;
  l.i_qg = l.i_pg + l.ng;
  l.i_udc = l.i_qg + l.ng;
  l.i_pac = l.i_udc + l.ndc;
  l.i_qac = l.i_pac + l.ncp;
  l.i_gfm_e = l.i_qac + l.ncp;
  l.i_gfm_f = l.i_gfm_e + l.ngfm;
  l.i_pdc = l.i_gfm_f + l.ngfm;
  l.nvar = l.i_pdc + l.nc;
  int converter_control_equalities = 0;
  for (const auto& converter : data->converters) {
    converter_control_equalities +=
        converter.control_mode == PhaseVSCControlMode::GridFormingDroop
            ? 2 * static_cast<int>(converter.phase_var_positions.size())
            : 2 *
                  (static_cast<int>(converter.phase_var_positions.size()) - 1);
  }
  l.neq = 2 * l.nv + l.ndc + l.nc +
          static_cast<int>(c.dc_reference_terminals.size()) +
          converter_control_equalities +
          2 * static_cast<int>(c.reference_nodes.size());
  l.nineq = 2 * full_n + static_cast<int>(c.three_phase_bus_nodes.size()) +
            l.nc + l.ncp +
            static_cast<int>(c.ac_line_limits.size()) +
            static_cast<int>(c.dc_line_limits.size());
  data->equality_scale = Eigen::VectorXd::Ones(l.neq);
  build_inequality_hessian_template(*data);
  configure_enforced_inequalities(*data, options.enforced_inequality_rows);
  return data;
}

Eigen::VectorXcd model_voltage(const ModelData& d, const Eigen::VectorXd& x) {
  const Layout& l = d.layout;
  Eigen::VectorXcd v(l.nv);
  for (int i = 0; i < l.nv; ++i) {
    v[i] = Complex{x[l.i_e + i], x[l.i_f + i]};
  }
  return v;
}

Eigen::VectorXcd full_voltage(const ModelData& d, const Eigen::VectorXd& x) {
  return d.recovery * model_voltage(d, x);
}

void evaluate_equalities(const ModelData& d,
                         const Eigen::VectorXd& x,
                         Eigen::VectorXd& g) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Eigen::VectorXcd v = model_voltage(d, x);
  const Eigen::VectorXcd current = d.y * v + d.fixed_current;
  g = Eigen::VectorXd::Zero(l.neq);
  for (int i = 0; i < l.nv; ++i) {
    const double p = std::real(v[i] * std::conj(current[i]));
    const double q = std::imag(v[i] * std::conj(current[i]));
    const int full_node = d.reduction.retained[static_cast<std::size_t>(i)];
    g[i] = p + c.p_load_pu[full_node];
    g[l.nv + i] = q + c.q_load_pu[full_node];
  }
  for (int gi = 0; gi < l.ng; ++gi) {
    const int node = d.generator_nodes[static_cast<std::size_t>(gi)];
    g[node] -= x[l.i_pg + gi];
    g[l.nv + node] -= x[l.i_qg + gi];
  }
  for (const auto& converter : d.converters) {
    for (int local = 0;
         local < static_cast<int>(converter.model_phase_nodes.size()); ++local) {
      const int node = converter.model_phase_nodes[static_cast<std::size_t>(local)];
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      g[node] -= x[l.i_pac + pos];
      g[l.nv + node] -= x[l.i_qac + pos];
    }
  }

  const Eigen::VectorXd u = x.segment(l.i_udc, l.ndc);
  const Eigen::VectorXd dc_current = c.g_dc * u;
  const int dc_row = 2 * l.nv;
  for (int k = 0; k < l.ndc; ++k) {
    g[dc_row + k] = u[k] * dc_current[k] + c.p_dc_load_pu[k];
  }
  for (int ci = 0; ci < l.nc; ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    g[dc_row + converter.dc_terminal] -= x[l.i_pdc + ci];
    double ac_sum = 0.0;
    for (int pos : converter.phase_var_positions) ac_sum += x[l.i_pac + pos];
    g[dc_row + l.ndc + ci] =
        x[l.i_pdc + ci] + converter.efficiency * ac_sum;
  }

  int row = dc_row + l.ndc + l.nc;
  for (int ri = 0;
       ri < static_cast<int>(c.dc_reference_terminals.size()); ++ri) {
    g[row++] = x[l.i_udc + c.dc_reference_terminals[static_cast<std::size_t>(ri)]] -
               c.dc_reference_voltage_pu[ri];
  }
  for (const auto& converter : d.converters) {
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
      const Complex phase_rotation[3] = {
          Complex{1.0, 0.0}, a * a, a};
      const Complex z{converter.virtual_r_pu, converter.virtual_x_pu};
      const Complex e_a{x[l.i_gfm_e + converter.gfm_var_position],
                        x[l.i_gfm_f + converter.gfm_var_position]};
      for (int phase = 0; phase < 3; ++phase) {
        const int pos =
            converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
        const int node =
            converter.model_node_by_phase[static_cast<std::size_t>(phase)];
        const Complex s{x[l.i_pac + pos], x[l.i_qac + pos]};
        const Complex e_phase = phase_rotation[phase] * e_a;
        const Complex residual =
            std::conj(e_phase) * v[node] - std::norm(v[node]) -
            std::conj(z) * s;
        g[row++] = std::real(residual);
        g[row++] = std::imag(residual);
      }
    } else if (converter.control_mode ==
               PhaseVSCControlMode::GridFollowingPLL) {
      const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
      const Complex rotation[2] = {a, a * a};
      const int pos_a = converter.phase_var_by_phase[0];
      const int node_a = converter.model_node_by_phase[0];
      const Complex s_a{x[l.i_pac + pos_a], x[l.i_qac + pos_a]};
      for (int phase = 1; phase < 3; ++phase) {
        const int pos = converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
        const int node = converter.model_node_by_phase[static_cast<std::size_t>(phase)];
        const Complex s{x[l.i_pac + pos], x[l.i_qac + pos]};
        const Complex residual =
            s * v[node_a] - rotation[phase - 1] * v[node] * s_a;
        g[row++] = std::real(residual);
        g[row++] = std::imag(residual);
      }
    } else {
      const int first = converter.phase_var_positions.front();
      for (int local = 1;
           local < static_cast<int>(converter.phase_var_positions.size());
           ++local) {
        const int pos =
            converter.phase_var_positions[static_cast<std::size_t>(local)];
        g[row++] = x[l.i_pac + pos] - x[l.i_pac + first];
      }
      for (int local = 1;
           local < static_cast<int>(converter.phase_var_positions.size());
           ++local) {
        const int pos =
            converter.phase_var_positions[static_cast<std::size_t>(local)];
        g[row++] = x[l.i_qac + pos] - x[l.i_qac + first];
      }
    }
  }
  for (int ri = 0; ri < static_cast<int>(c.reference_nodes.size()); ++ri) {
    const int node = d.reference_model_nodes[static_cast<std::size_t>(ri)];
    g[row++] = x[l.i_e + node] - std::real(c.reference_voltage[ri]);
    g[row++] = x[l.i_f + node] - std::imag(c.reference_voltage[ri]);
  }
  g.array() *= d.equality_scale.array();
}

void equality_jacobian(const ModelData& d,
                       const Eigen::VectorXd& x,
                       Eigen::SparseMatrix<double>& jac) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Eigen::VectorXcd v = model_voltage(d, x);
  const Eigen::VectorXcd current = d.y * v + d.fixed_current;
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(8 * d.y.nonZeros() +
                                         2 * l.ng + 3 * l.ncp +
                                         6 * l.ndc + 3 * l.nc));
  for (int col = 0; col < d.y.outerSize(); ++col) {
    for (graph::SparseComplexMatrix::InnerIterator it(d.y, col); it; ++it) {
      const int i = it.row();
      const int j = it.col();
      const double gij = std::real(it.value());
      const double bij = std::imag(it.value());
      const double ei = std::real(v[i]);
      const double fi = std::imag(v[i]);
      trips.emplace_back(i, l.i_e + j, ei * gij + fi * bij);
      trips.emplace_back(i, l.i_f + j, -ei * bij + fi * gij);
      trips.emplace_back(l.nv + i, l.i_e + j, fi * gij - ei * bij);
      trips.emplace_back(l.nv + i, l.i_f + j, -fi * bij - ei * gij);
    }
  }
  for (int i = 0; i < l.nv; ++i) {
    trips.emplace_back(i, l.i_e + i, std::real(current[i]));
    trips.emplace_back(i, l.i_f + i, std::imag(current[i]));
    trips.emplace_back(l.nv + i, l.i_e + i, -std::imag(current[i]));
    trips.emplace_back(l.nv + i, l.i_f + i, std::real(current[i]));
  }
  for (int gi = 0; gi < l.ng; ++gi) {
    const int node = d.generator_nodes[static_cast<std::size_t>(gi)];
    trips.emplace_back(node, l.i_pg + gi, -1.0);
    trips.emplace_back(l.nv + node, l.i_qg + gi, -1.0);
  }
  for (const auto& converter : d.converters) {
    for (int local = 0;
         local < static_cast<int>(converter.model_phase_nodes.size()); ++local) {
      const int node = converter.model_phase_nodes[static_cast<std::size_t>(local)];
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      trips.emplace_back(node, l.i_pac + pos, -1.0);
      trips.emplace_back(l.nv + node, l.i_qac + pos, -1.0);
    }
  }

  const Eigen::VectorXd u = x.segment(l.i_udc, l.ndc);
  const Eigen::VectorXd dc_current = c.g_dc * u;
  const int dc_row = 2 * l.nv;
  for (int col = 0; col < c.g_dc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(c.g_dc, col); it; ++it) {
      const int i = it.row();
      const int j = it.col();
      trips.emplace_back(dc_row + i, l.i_udc + j, u[i] * it.value());
    }
  }
  for (int i = 0; i < l.ndc; ++i) {
    trips.emplace_back(dc_row + i, l.i_udc + i, dc_current[i]);
  }
  for (int ci = 0; ci < l.nc; ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    trips.emplace_back(dc_row + converter.dc_terminal, l.i_pdc + ci, -1.0);
    const int row = dc_row + l.ndc + ci;
    trips.emplace_back(row, l.i_pdc + ci, 1.0);
    for (int pos : converter.phase_var_positions) {
      trips.emplace_back(row, l.i_pac + pos, converter.efficiency);
    }
  }
  int row = dc_row + l.ndc + l.nc;
  for (int terminal : c.dc_reference_terminals) {
    trips.emplace_back(row++, l.i_udc + terminal, 1.0);
  }
  for (const auto& converter : d.converters) {
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
      const Complex phase_rotation[3] = {
          Complex{1.0, 0.0}, a * a, a};
      const Complex z{converter.virtual_r_pu, converter.virtual_x_pu};
      const Complex e_a{x[l.i_gfm_e + converter.gfm_var_position],
                        x[l.i_gfm_f + converter.gfm_var_position]};
      for (int phase = 0; phase < 3; ++phase) {
        const int pos =
            converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
        const int node =
            converter.model_node_by_phase[static_cast<std::size_t>(phase)];
        const Complex coefficient = std::conj(phase_rotation[phase]);
        const Complex common = coefficient * std::conj(e_a);
        const auto append = [&](int col, Complex derivative) {
          trips.emplace_back(row, col, std::real(derivative));
          trips.emplace_back(row + 1, col, std::imag(derivative));
        };
        append(l.i_gfm_e + converter.gfm_var_position,
               coefficient * v[node]);
        append(l.i_gfm_f + converter.gfm_var_position,
               -Complex{0.0, 1.0} * coefficient * v[node]);
        append(l.i_e + node,
               common - Complex{2.0 * std::real(v[node]), 0.0});
        append(l.i_f + node,
               Complex{0.0, 1.0} * common -
                   Complex{2.0 * std::imag(v[node]), 0.0});
        append(l.i_pac + pos, -std::conj(z));
        append(l.i_qac + pos,
               -Complex{0.0, 1.0} * std::conj(z));
        row += 2;
      }
    } else if (converter.control_mode ==
               PhaseVSCControlMode::GridFollowingPLL) {
      const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
      const Complex rotation[2] = {a, a * a};
      const int pos_a = converter.phase_var_by_phase[0];
      const int node_a = converter.model_node_by_phase[0];
      const Complex s_a{x[l.i_pac + pos_a], x[l.i_qac + pos_a]};
      for (int phase = 1; phase < 3; ++phase) {
        const int pos = converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
        const int node = converter.model_node_by_phase[static_cast<std::size_t>(phase)];
        const Complex s{x[l.i_pac + pos], x[l.i_qac + pos]};
        const Complex rot = rotation[phase - 1];
        const auto append = [&](int col, Complex derivative) {
          trips.emplace_back(row, col, std::real(derivative));
          trips.emplace_back(row + 1, col, std::imag(derivative));
        };
        append(l.i_pac + pos, v[node_a]);
        append(l.i_qac + pos, Complex{0.0, 1.0} * v[node_a]);
        append(l.i_e + node_a, s);
        append(l.i_f + node_a, Complex{0.0, 1.0} * s);
        append(l.i_e + node, -rot * s_a);
        append(l.i_f + node, -Complex{0.0, 1.0} * rot * s_a);
        append(l.i_pac + pos_a, -rot * v[node]);
        append(l.i_qac + pos_a,
               -Complex{0.0, 1.0} * rot * v[node]);
        row += 2;
      }
    } else {
      const int first = converter.phase_var_positions.front();
      for (int local = 1;
           local < static_cast<int>(converter.phase_var_positions.size());
           ++local) {
        const int pos =
            converter.phase_var_positions[static_cast<std::size_t>(local)];
        trips.emplace_back(row, l.i_pac + pos, 1.0);
        trips.emplace_back(row++, l.i_pac + first, -1.0);
      }
      for (int local = 1;
           local < static_cast<int>(converter.phase_var_positions.size());
           ++local) {
        const int pos =
            converter.phase_var_positions[static_cast<std::size_t>(local)];
        trips.emplace_back(row, l.i_qac + pos, 1.0);
        trips.emplace_back(row++, l.i_qac + first, -1.0);
      }
    }
  }
  for (int node : d.reference_model_nodes) {
    trips.emplace_back(row++, l.i_e + node, 1.0);
    trips.emplace_back(row++, l.i_f + node, 1.0);
  }
  jac.resize(l.neq, l.nvar);
  jac.setFromTriplets(trips.begin(), trips.end());
  jac.makeCompressed();
  for (int col = 0; col < jac.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(jac, col); it; ++it) {
      it.valueRef() *= d.equality_scale[it.row()];
    }
  }
}

void append_full_gradient(const ModelData& d,
                          int row,
                          const std::vector<std::pair<int, double>>& de,
                          const std::vector<std::pair<int, double>>& df,
                          std::vector<Triplet>& trips) {
  std::vector<std::pair<int, double>> combined_e;
  std::vector<std::pair<int, double>> combined_f;
  for (const auto& [full_node, value] : de) {
    for (const auto& [model_node, coeff] :
         d.recovery_rows[static_cast<std::size_t>(full_node)]) {
      combined_e.emplace_back(model_node, value * std::real(coeff));
      combined_f.emplace_back(model_node, -value * std::imag(coeff));
    }
  }
  for (const auto& [full_node, value] : df) {
    for (const auto& [model_node, coeff] :
         d.recovery_rows[static_cast<std::size_t>(full_node)]) {
      combined_e.emplace_back(model_node, value * std::imag(coeff));
      combined_f.emplace_back(model_node, value * std::real(coeff));
    }
  }
  for (const auto& [node, value] : combined_e) {
    trips.emplace_back(row, d.layout.i_e + node, value);
  }
  for (const auto& [node, value] : combined_f) {
    trips.emplace_back(row, d.layout.i_f + node, value);
  }
}

void append_voltage_norm_gradient(const ModelData& d,
                                  int row,
                                  int full_node,
                                  Complex voltage,
                                  double sign,
                                  std::vector<Triplet>& trips) {
  const double e = std::real(voltage);
  const double f = std::imag(voltage);
  for (const auto& [model_node, recovery] :
       d.recovery_rows[static_cast<std::size_t>(full_node)]) {
    const double cr = std::real(recovery);
    const double ci = std::imag(recovery);
    trips.emplace_back(row, d.layout.i_e + model_node,
                       2.0 * sign * (e * cr + f * ci));
    trips.emplace_back(row, d.layout.i_f + model_node,
                       2.0 * sign * (-e * ci + f * cr));
  }
}

std::pair<Complex, Complex> sequence_components(
    const Eigen::VectorXcd& v,
    const std::vector<int>& nodes) {
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  const Complex v1 = (v[nodes[0]] + a * v[nodes[1]] + a2 * v[nodes[2]]) / 3.0;
  const Complex v2 = (v[nodes[0]] + a2 * v[nodes[1]] + a * v[nodes[2]]) / 3.0;
  return {v1, v2};
}

void evaluate_inequalities(const ModelData& d,
                           const Eigen::VectorXd& x,
                           Eigen::VectorXd& h) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Eigen::VectorXcd v = full_voltage(d, x);
  const int full_n = static_cast<int>(v.size());
  h = Eigen::VectorXd::Zero(l.nineq);
  for (int i = 0; i < full_n; ++i) {
    const double vm2 = std::norm(v[i]);
    h[i] = c.v_min_pu[i] * c.v_min_pu[i] - vm2;
    h[full_n + i] = vm2 - c.v_max_pu[i] * c.v_max_pu[i];
  }
  int row = 2 * full_n;
  for (const auto& nodes : c.three_phase_bus_nodes) {
    const auto [v1, v2] = sequence_components(v, nodes);
    h[row++] = std::norm(v2) - c.vuf_max * c.vuf_max * std::norm(v1);
  }
  for (const auto& converter : d.converters) {
    double s2 = 0.0;
    for (int pos : converter.phase_var_positions) {
      s2 += x[l.i_pac + pos] * x[l.i_pac + pos] +
            x[l.i_qac + pos] * x[l.i_qac + pos];
    }
    h[row++] = s2 - converter.s_max_pu * converter.s_max_pu;
  }
  for (int ci = 0; ci < static_cast<int>(d.converters.size()); ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const auto& source_converter = c.converters[static_cast<std::size_t>(ci)];
    for (int local = 0;
         local < static_cast<int>(converter.phase_var_positions.size());
         ++local) {
      const int pos =
          converter.phase_var_positions[static_cast<std::size_t>(local)];
      const int full_node =
          source_converter.phase_nodes[static_cast<std::size_t>(local)];
      h[row++] = x[l.i_pac + pos] * x[l.i_pac + pos] +
                 x[l.i_qac + pos] * x[l.i_qac + pos] -
                 converter.phase_current_max_pu *
                     converter.phase_current_max_pu * std::norm(v[full_node]);
    }
  }
  for (const auto& limit : c.ac_line_limits) {
    Complex i_line{0.0, 0.0};
    for (std::size_t k = 0; k < limit.nodes.size(); ++k) {
      i_line += limit.coefficients[k] *
                v[limit.nodes[static_cast<std::size_t>(k)]];
    }
    h[row++] = std::norm(i_line) - limit.i_max_pu * limit.i_max_pu;
  }
  for (const auto& limit : c.dc_line_limits) {
    const double du =
        x[l.i_udc + limit.from_node] - x[l.i_udc + limit.to_node];
    const double current = limit.conductance_pu * du;
    h[row++] = current * current - limit.i_max_pu * limit.i_max_pu;
  }
}

void inequality_jacobian(const ModelData& d,
                         const Eigen::VectorXd& x,
                         Eigen::SparseMatrix<double>& jac,
                         const std::vector<int>* enforced_rows = nullptr) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Eigen::VectorXcd v = full_voltage(d, x);
  const int full_n = static_cast<int>(v.size());
  const int output_rows = enforced_rows == nullptr
      ? l.nineq : static_cast<int>(enforced_rows->size());
  std::vector<int> full_to_output(static_cast<std::size_t>(l.nineq), -1);
  if (enforced_rows == nullptr) {
    for (int row = 0; row < l.nineq; ++row) {
      full_to_output[static_cast<std::size_t>(row)] = row;
    }
  } else {
    for (int local = 0; local < output_rows; ++local) {
      full_to_output[static_cast<std::size_t>(
          (*enforced_rows)[static_cast<std::size_t>(local)])] = local;
    }
  }
  const auto output_row = [&](int full_row) {
    return full_to_output[static_cast<std::size_t>(full_row)];
  };
  std::vector<Triplet> trips;
  std::size_t reserve_count = 0;
  for (int node = 0; node < full_n; ++node) {
    if (output_row(node) >= 0) {
      reserve_count += 2 * d.recovery_rows[static_cast<std::size_t>(node)].size();
    }
    if (output_row(full_n + node) >= 0) {
      reserve_count += 2 * d.recovery_rows[static_cast<std::size_t>(node)].size();
    }
  }
  int full_row = 2 * full_n;
  for (const auto& nodes : c.three_phase_bus_nodes) {
    if (output_row(full_row++) < 0) continue;
    for (int node : nodes) {
      reserve_count += 2 * d.recovery_rows[static_cast<std::size_t>(node)].size();
    }
  }
  for (const auto& converter : d.converters) {
    if (output_row(full_row++) >= 0) {
      reserve_count += 2 * converter.phase_var_positions.size();
    }
  }
  for (int ci = 0; ci < static_cast<int>(d.converters.size()); ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const auto& source_converter = c.converters[static_cast<std::size_t>(ci)];
    for (int local = 0;
         local < static_cast<int>(converter.phase_var_positions.size());
         ++local) {
      const int row = output_row(full_row++);
      if (row < 0) continue;
      const int full_node =
          source_converter.phase_nodes[static_cast<std::size_t>(local)];
      reserve_count += 2 +
          2 * d.recovery_rows[static_cast<std::size_t>(full_node)].size();
    }
  }
  for (const auto& limit : c.ac_line_limits) {
    if (output_row(full_row++) < 0) continue;
    for (int node : limit.nodes) {
      reserve_count +=
          2 * d.recovery_rows[static_cast<std::size_t>(node)].size();
    }
  }
  for (std::size_t li = 0; li < c.dc_line_limits.size(); ++li) {
    if (output_row(full_row++) >= 0) reserve_count += 2;
  }
  trips.reserve(reserve_count);
  for (int i = 0; i < full_n; ++i) {
    const int lower_row = output_row(i);
    const int upper_row = output_row(full_n + i);
    if (lower_row >= 0) {
      append_voltage_norm_gradient(d, lower_row, i, v[i], -1.0, trips);
    }
    if (upper_row >= 0) {
      append_voltage_norm_gradient(d, upper_row, i, v[i], 1.0, trips);
    }
  }
  full_row = 2 * full_n;
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  const Complex c1[3] = {Complex{1.0 / 3.0, 0.0}, a / 3.0, a2 / 3.0};
  const Complex c2[3] = {Complex{1.0 / 3.0, 0.0}, a2 / 3.0, a / 3.0};
  for (const auto& nodes : c.three_phase_bus_nodes) {
    const int row = output_row(full_row++);
    if (row < 0) continue;
    const auto [v1, v2] = sequence_components(v, nodes);
    std::vector<std::pair<int, double>> de;
    std::vector<std::pair<int, double>> df;
    for (int phase = 0; phase < 3; ++phase) {
      const Complex z = std::conj(v2) * c2[phase] -
          c.vuf_max * c.vuf_max * std::conj(v1) * c1[phase];
      de.emplace_back(nodes[static_cast<std::size_t>(phase)], 2.0 * std::real(z));
      df.emplace_back(nodes[static_cast<std::size_t>(phase)],
                      2.0 * std::real(Complex{0.0, 1.0} * z));
    }
    append_full_gradient(d, row, de, df, trips);
  }
  for (const auto& converter : d.converters) {
    const int row = output_row(full_row++);
    if (row < 0) continue;
    for (int pos : converter.phase_var_positions) {
      trips.emplace_back(row, l.i_pac + pos, 2.0 * x[l.i_pac + pos]);
      trips.emplace_back(row, l.i_qac + pos, 2.0 * x[l.i_qac + pos]);
    }
  }
  for (int ci = 0; ci < static_cast<int>(d.converters.size()); ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const auto& source_converter = c.converters[static_cast<std::size_t>(ci)];
    for (int local = 0;
         local < static_cast<int>(converter.phase_var_positions.size());
         ++local) {
      const int row = output_row(full_row++);
      if (row < 0) continue;
      const int pos =
          converter.phase_var_positions[static_cast<std::size_t>(local)];
      const int full_node =
          source_converter.phase_nodes[static_cast<std::size_t>(local)];
      trips.emplace_back(row, l.i_pac + pos, 2.0 * x[l.i_pac + pos]);
      trips.emplace_back(row, l.i_qac + pos, 2.0 * x[l.i_qac + pos]);
      append_voltage_norm_gradient(
          d, row, full_node, v[full_node],
          -converter.phase_current_max_pu * converter.phase_current_max_pu,
          trips);
    }
  }
  for (const auto& limit : c.ac_line_limits) {
    const int row = output_row(full_row++);
    if (row < 0) continue;
    Complex i_line{0.0, 0.0};
    for (std::size_t k = 0; k < limit.nodes.size(); ++k) {
      i_line += limit.coefficients[k] *
                v[limit.nodes[static_cast<std::size_t>(k)]];
    }
    std::vector<std::pair<int, double>> de;
    std::vector<std::pair<int, double>> df;
    de.reserve(limit.nodes.size());
    df.reserve(limit.nodes.size());
    for (std::size_t k = 0; k < limit.nodes.size(); ++k) {
      const Complex z = std::conj(i_line) * limit.coefficients[k];
      de.emplace_back(limit.nodes[static_cast<std::size_t>(k)],
                      2.0 * std::real(z));
      df.emplace_back(limit.nodes[static_cast<std::size_t>(k)],
                      2.0 * std::real(Complex{0.0, 1.0} * z));
    }
    append_full_gradient(d, row, de, df, trips);
  }
  for (const auto& limit : c.dc_line_limits) {
    const int row = output_row(full_row++);
    if (row < 0) continue;
    const double du =
        x[l.i_udc + limit.from_node] - x[l.i_udc + limit.to_node];
    const double slope =
        2.0 * limit.conductance_pu * limit.conductance_pu * du;
    trips.emplace_back(row, l.i_udc + limit.from_node, slope);
    trips.emplace_back(row, l.i_udc + limit.to_node, -slope);
  }
  jac.resize(output_rows, l.nvar);
  jac.setFromTriplets(trips.begin(), trips.end());
  jac.makeCompressed();
}

double objective(const ModelData& d, const Eigen::VectorXd& x) {
  double value = 0.0;
  for (int gi = 0; gi < d.layout.ng; ++gi) {
    const auto& generator = d.source->generators[static_cast<std::size_t>(gi)];
    const double p_mw = x[d.layout.i_pg + gi] * d.source->base_mva;
    value += generator.cost_c2 * p_mw * p_mw + generator.cost_c1 * p_mw;
    value += 1e-4 * x[d.layout.i_qg + gi] * x[d.layout.i_qg + gi];
  }
  for (int pos = 0; pos < d.layout.ncp; ++pos) {
    value += 1e-3 * (x[d.layout.i_pac + pos] * x[d.layout.i_pac + pos] +
                     x[d.layout.i_qac + pos] * x[d.layout.i_qac + pos]);
  }
  for (int ci = 0; ci < d.layout.nc; ++ci) {
    value += 1e-4 * x[d.layout.i_pdc + ci] * x[d.layout.i_pdc + ci];
  }
  return value;
}

void objective_gradient(const ModelData& d,
                        const Eigen::VectorXd& x,
                        Eigen::VectorXd& grad) {
  grad = Eigen::VectorXd::Zero(d.layout.nvar);
  for (int gi = 0; gi < d.layout.ng; ++gi) {
    const auto& generator = d.source->generators[static_cast<std::size_t>(gi)];
    const double p = x[d.layout.i_pg + gi];
    grad[d.layout.i_pg + gi] =
        2.0 * generator.cost_c2 * d.source->base_mva * d.source->base_mva * p +
        generator.cost_c1 * d.source->base_mva;
    grad[d.layout.i_qg + gi] = 2e-4 * x[d.layout.i_qg + gi];
  }
  for (int pos = 0; pos < d.layout.ncp; ++pos) {
    grad[d.layout.i_pac + pos] = 2e-3 * x[d.layout.i_pac + pos];
    grad[d.layout.i_qac + pos] = 2e-3 * x[d.layout.i_qac + pos];
  }
  for (int ci = 0; ci < d.layout.nc; ++ci) {
    grad[d.layout.i_pdc + ci] = 2e-4 * x[d.layout.i_pdc + ci];
  }
}

void objective_hessian(const ModelData& d,
                       const Eigen::VectorXd&,
                       Eigen::SparseMatrix<double>& hessian) {
  std::vector<Triplet> trips;
  for (int gi = 0; gi < d.layout.ng; ++gi) {
    const auto& generator = d.source->generators[static_cast<std::size_t>(gi)];
    const double value = 2.0 * generator.cost_c2 * d.source->base_mva *
                         d.source->base_mva;
    if (value != 0.0) trips.emplace_back(d.layout.i_pg + gi,
                                         d.layout.i_pg + gi, value);
    trips.emplace_back(d.layout.i_qg + gi, d.layout.i_qg + gi, 2e-4);
  }
  for (int pos = 0; pos < d.layout.ncp; ++pos) {
    trips.emplace_back(d.layout.i_pac + pos, d.layout.i_pac + pos, 2e-3);
    trips.emplace_back(d.layout.i_qac + pos, d.layout.i_qac + pos, 2e-3);
  }
  for (int ci = 0; ci < d.layout.nc; ++ci) {
    trips.emplace_back(d.layout.i_pdc + ci, d.layout.i_pdc + ci, 2e-4);
  }
  hessian.resize(d.layout.nvar, d.layout.nvar);
  hessian.setFromTriplets(trips.begin(), trips.end());
}

void add_symmetric(std::vector<Triplet>& trips,
                   int row,
                   int col,
                   double value) {
  if (value == 0.0) return;
  if (row == col) {
    trips.emplace_back(row, col, 2.0 * value);
  } else {
    trips.emplace_back(row, col, value);
    trips.emplace_back(col, row, value);
  }
}

void add_weighted_complex_product_hessian(
    std::vector<Triplet>& trips,
    int x_real,
    int x_imag,
    int y_real,
    int y_imag,
    Complex coefficient,
    double lambda_real,
    double lambda_imag) {
  const double cr = std::real(coefficient);
  const double ci = std::imag(coefficient);
  const double same = lambda_real * cr + lambda_imag * ci;
  const double cross = -lambda_real * ci + lambda_imag * cr;
  add_symmetric(trips, x_real, y_real, same);
  add_symmetric(trips, x_real, y_imag, cross);
  add_symmetric(trips, x_imag, y_real, cross);
  add_symmetric(trips, x_imag, y_imag, -same);
}

void lagrangian_hessian(const ModelData& d,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd* nu,
                        Eigen::SparseMatrix<double>& hessian,
                        bool enforced_only = true) {
  (void)x;
  const Layout& l = d.layout;
  std::vector<Triplet> trips;
  const auto& inequality_terms = enforced_only
      ? d.enforced_inequality_hessian_terms : d.inequality_hessian_terms;
  trips.reserve(inequality_terms.size() +
                static_cast<std::size_t>(8 * d.y.nonZeros() +
                                         2 * d.source->g_dc.nonZeros() +
                                         l.nvar));
  Eigen::SparseMatrix<double> objective_part;
  objective_hessian(d, x, objective_part);
  for (int col = 0; col < objective_part.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(objective_part, col);
         it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }

  if (lambda.size() == l.neq) {
    for (int col = 0; col < d.y.outerSize(); ++col) {
      for (graph::SparseComplexMatrix::InnerIterator it(d.y, col); it; ++it) {
        const int i = it.row();
        const int j = it.col();
        const double g = std::real(it.value());
        const double b = std::imag(it.value());
        const double lp = lambda[i] * d.equality_scale[i];
        const double lq = lambda[l.nv + i] * d.equality_scale[l.nv + i];
        add_symmetric(trips, l.i_e + i, l.i_e + j, lp * g - lq * b);
        add_symmetric(trips, l.i_e + i, l.i_f + j, -lp * b - lq * g);
        add_symmetric(trips, l.i_f + i, l.i_e + j, lp * b + lq * g);
        add_symmetric(trips, l.i_f + i, l.i_f + j, lp * g - lq * b);
      }
    }
    const int dc_row = 2 * l.nv;
    for (int col = 0; col < d.source->g_dc.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(d.source->g_dc, col);
           it; ++it) {
        const int i = it.row();
        const int j = it.col();
        const double multiplier =
            lambda[dc_row + i] * d.equality_scale[dc_row + i];
        add_symmetric(trips, l.i_udc + i, l.i_udc + j,
                      multiplier * it.value());
      }
    }
    int control_row = dc_row + l.ndc + l.nc +
        static_cast<int>(d.source->dc_reference_terminals.size());
    const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
    const Complex rotation[2] = {a, a * a};
    for (const auto& converter : d.converters) {
      if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
        const Complex phase_rotation[3] = {
            Complex{1.0, 0.0}, a * a, a};
        const int er = l.i_gfm_e + converter.gfm_var_position;
        const int ei = l.i_gfm_f + converter.gfm_var_position;
        for (int phase = 0; phase < 3; ++phase) {
          const int node =
              converter.model_node_by_phase[static_cast<std::size_t>(phase)];
          const double lr =
              lambda[control_row] * d.equality_scale[control_row];
          const double li =
              lambda[control_row + 1] * d.equality_scale[control_row + 1];
          const Complex coefficient = std::conj(phase_rotation[phase]);
          const auto weighted = [&](Complex c) {
            return lr * std::real(c) + li * std::imag(c);
          };
          add_symmetric(trips, er, l.i_e + node, weighted(coefficient));
          add_symmetric(trips, er, l.i_f + node,
                        weighted(Complex{0.0, 1.0} * coefficient));
          add_symmetric(trips, ei, l.i_e + node,
                        weighted(-Complex{0.0, 1.0} * coefficient));
          add_symmetric(trips, ei, l.i_f + node, weighted(coefficient));
          add_symmetric(trips, l.i_e + node, l.i_e + node, -lr);
          add_symmetric(trips, l.i_f + node, l.i_f + node, -lr);
          control_row += 2;
        }
      } else if (converter.control_mode ==
                 PhaseVSCControlMode::GridFollowingPLL) {
        const int pos_a = converter.phase_var_by_phase[0];
        const int node_a = converter.model_node_by_phase[0];
        for (int phase = 1; phase < 3; ++phase) {
          const int pos =
              converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
          const int node =
              converter.model_node_by_phase[static_cast<std::size_t>(phase)];
          const double lr =
              lambda[control_row] * d.equality_scale[control_row];
          const double li =
              lambda[control_row + 1] * d.equality_scale[control_row + 1];
          add_weighted_complex_product_hessian(
              trips, l.i_pac + pos, l.i_qac + pos,
              l.i_e + node_a, l.i_f + node_a, Complex{1.0, 0.0}, lr, li);
          add_weighted_complex_product_hessian(
              trips, l.i_e + node, l.i_f + node,
              l.i_pac + pos_a, l.i_qac + pos_a,
              -rotation[phase - 1], lr, li);
          control_row += 2;
        }
      } else {
        control_row += 2 *
            (static_cast<int>(converter.phase_var_positions.size()) - 1);
      }
    }
  }

  if (nu != nullptr && nu->size() >= l.nineq) {
    for (const auto& term : inequality_terms) {
      double value = (*nu)[term.multiplier_row_1] * term.coefficient_1;
      if (term.multiplier_row_2 >= 0) {
        value += (*nu)[term.multiplier_row_2] * term.coefficient_2;
      }
      if (value != 0.0) {
        trips.emplace_back(term.row, term.col, value);
      }
    }
  }
  hessian.resize(l.nvar, l.nvar);
  hessian.setFromTriplets(trips.begin(), trips.end());
  hessian.makeCompressed();
}

bool initialize_ac_power_flow(const ModelData& d, Eigen::VectorXd& x) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const auto fail = [&](const std::string& reason) {
    if (d.verbose) {
      std::cerr << '[' << c.name << "] AC continuation initialization skipped: "
                << reason << "\n" << std::flush;
    }
    return false;
  };
  std::vector<bool> is_reference(static_cast<std::size_t>(l.nv), false);
  for (int node : d.reference_model_nodes) {
    is_reference[static_cast<std::size_t>(node)] = true;
  }
  for (int node : d.generator_nodes) {
    if (!is_reference[static_cast<std::size_t>(node)]) {
      return fail("non-reference generator requires a specified dispatch");
    }
  }

  for (int index = 0; index < static_cast<int>(d.reference_model_nodes.size()); ++index) {
    const int node = d.reference_model_nodes[static_cast<std::size_t>(index)];
    x[l.i_e + node] = std::real(c.reference_voltage[index]);
    x[l.i_f + node] = std::imag(c.reference_voltage[index]);
  }

  std::vector<int> unknown_nodes;
  std::vector<int> unknown_position(static_cast<std::size_t>(l.nv), -1);
  for (int node = 0; node < l.nv; ++node) {
    if (is_reference[static_cast<std::size_t>(node)]) continue;
    unknown_position[static_cast<std::size_t>(node)] =
        static_cast<int>(unknown_nodes.size());
    unknown_nodes.push_back(node);
  }
  if (unknown_nodes.empty()) return true;

  std::vector<Eigen::Triplet<Complex>> trips;
  for (int col = 0; col < d.y.outerSize(); ++col) {
    const int reduced_col = unknown_position[static_cast<std::size_t>(col)];
    if (reduced_col < 0) continue;
    for (graph::SparseComplexMatrix::InnerIterator it(d.y, col); it; ++it) {
      const int reduced_row =
          unknown_position[static_cast<std::size_t>(it.row())];
      if (reduced_row >= 0) {
        trips.emplace_back(reduced_row, reduced_col, it.value());
      }
    }
  }
  Eigen::SparseMatrix<Complex> y_uu(
      static_cast<int>(unknown_nodes.size()),
      static_cast<int>(unknown_nodes.size()));
  y_uu.setFromTriplets(trips.begin(), trips.end());
  y_uu.makeCompressed();
  Eigen::SparseLU<Eigen::SparseMatrix<Complex>> lu;
  lu.analyzePattern(y_uu);
  lu.factorize(y_uu);
  if (lu.info() != Eigen::Success) return fail("Y_UU factorization failed");

  Eigen::VectorXcd specified_power = Eigen::VectorXcd::Zero(l.nv);
  for (int node = 0; node < l.nv; ++node) {
    const int full_node = d.reduction.retained[static_cast<std::size_t>(node)];
    specified_power[node] =
        Complex{-c.p_load_pu[full_node], -c.q_load_pu[full_node]};
  }
  for (const auto& converter : d.converters) {
    for (int local = 0;
         local < static_cast<int>(converter.model_phase_nodes.size()); ++local) {
      const int node = converter.model_phase_nodes[static_cast<std::size_t>(local)];
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      specified_power[node] +=
          Complex{x[l.i_pac + pos], x[l.i_qac + pos]};
    }
  }

  const Eigen::VectorXd original_x = x;
  const auto ac_residual = [&](const Eigen::VectorXd& point,
                               Eigen::VectorXd* residual_vector) {
    const Eigen::VectorXcd voltage = model_voltage(d, point);
    const Eigen::VectorXcd current = d.y * voltage + d.fixed_current;
    Eigen::VectorXd residual(2 * static_cast<int>(unknown_nodes.size()));
    for (int local = 0; local < static_cast<int>(unknown_nodes.size()); ++local) {
      const int node = unknown_nodes[static_cast<std::size_t>(local)];
      const Complex mismatch =
          voltage[node] * std::conj(current[node]) - specified_power[node];
      residual[local] = std::real(mismatch);
      residual[static_cast<int>(unknown_nodes.size()) + local] =
          std::imag(mismatch);
    }
    const double norm = residual.cwiseAbs().maxCoeff();
    if (residual_vector != nullptr) *residual_vector = std::move(residual);
    return norm;
  };

  for (int iteration = 0; iteration < 20; ++iteration) {
    Eigen::VectorXd residual;
    const double norm = ac_residual(x, &residual);
    if (d.verbose) {
      const Eigen::VectorXcd recovered = full_voltage(d, x);
      std::cerr << '[' << c.name << "] AC Newton initialization: iteration="
                << iteration << ", residual=" << norm
                << ", Vmin=" << recovered.cwiseAbs().minCoeff()
                << ", Vmax=" << recovered.cwiseAbs().maxCoeff() << "\n"
                << std::flush;
    }
    if (norm <= 1e-10) return true;

    Eigen::SparseMatrix<double> full_jacobian;
    equality_jacobian(d, x, full_jacobian);
    std::vector<Eigen::Triplet<double>> jacobian_trips;
    for (int col = 0; col < full_jacobian.outerSize(); ++col) {
      int reduced_col = -1;
      if (col >= l.i_e && col < l.i_e + l.nv) {
        const int node = col - l.i_e;
        const int local = unknown_position[static_cast<std::size_t>(node)];
        if (local >= 0) reduced_col = local;
      } else if (col >= l.i_f && col < l.i_f + l.nv) {
        const int node = col - l.i_f;
        const int local = unknown_position[static_cast<std::size_t>(node)];
        if (local >= 0) {
          reduced_col = static_cast<int>(unknown_nodes.size()) + local;
        }
      }
      if (reduced_col < 0) continue;
      for (Eigen::SparseMatrix<double>::InnerIterator it(full_jacobian, col);
           it; ++it) {
        int reduced_row = -1;
        if (it.row() < l.nv) {
          reduced_row = unknown_position[static_cast<std::size_t>(it.row())];
        } else if (it.row() < 2 * l.nv) {
          const int local = unknown_position[static_cast<std::size_t>(
              it.row() - l.nv)];
          if (local >= 0) {
            reduced_row = static_cast<int>(unknown_nodes.size()) + local;
          }
        }
        if (reduced_row >= 0) {
          jacobian_trips.emplace_back(reduced_row, reduced_col, it.value());
        }
      }
    }
    const int newton_dimension = 2 * static_cast<int>(unknown_nodes.size());
    Eigen::SparseMatrix<double> jacobian(newton_dimension, newton_dimension);
    jacobian.setFromTriplets(jacobian_trips.begin(), jacobian_trips.end());
    Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> lu_newton;
    lu_newton.analyzePattern(jacobian);
    lu_newton.factorize(jacobian);
    if (lu_newton.info() != Eigen::Success) {
      if (d.verbose) std::cerr << '[' << c.name << "] AC Newton factorization failed\n";
      break;
    }
    const Eigen::VectorXd step = lu_newton.solve(-residual);
    if (lu_newton.info() != Eigen::Success || !step.allFinite()) {
      if (d.verbose) std::cerr << '[' << c.name << "] AC Newton solve failed\n";
      break;
    }

    bool accepted = false;
    double alpha = 1.0;
    for (int line_search = 0; line_search < 20; ++line_search) {
      Eigen::VectorXd trial = x;
      for (int local = 0; local < static_cast<int>(unknown_nodes.size()); ++local) {
        const int node = unknown_nodes[static_cast<std::size_t>(local)];
        trial[l.i_e + node] += alpha * step[local];
        trial[l.i_f + node] +=
            alpha * step[static_cast<int>(unknown_nodes.size()) + local];
      }
      const Eigen::VectorXcd recovered = full_voltage(d, trial);
      const double min_voltage = recovered.cwiseAbs().minCoeff();
      const double max_voltage = recovered.cwiseAbs().maxCoeff();
      if (trial.allFinite() && min_voltage > 0.5 && max_voltage < 1.3 &&
          ac_residual(trial, nullptr) < norm) {
        x = std::move(trial);
        accepted = true;
        break;
      }
      alpha *= 0.5;
    }
    if (!accepted) {
      if (d.verbose) {
        std::cerr << '[' << c.name
                  << "] AC Newton line search failed at residual=" << norm << "\n"
                  << std::flush;
      }
      break;
    }
  }
  x = original_x;

  Eigen::VectorXcd voltage = model_voltage(d, x);
  Eigen::VectorXcd reference_voltage = Eigen::VectorXcd::Zero(l.nv);
  for (int node : d.reference_model_nodes) reference_voltage[node] = voltage[node];
  const Eigen::VectorXcd known_current = d.y * reference_voltage + d.fixed_current;
  Eigen::VectorXcd no_load_rhs(static_cast<int>(unknown_nodes.size()));
  for (int local = 0; local < static_cast<int>(unknown_nodes.size()); ++local) {
    no_load_rhs[local] = -known_current[unknown_nodes[static_cast<std::size_t>(local)]];
  }
  const Eigen::VectorXcd no_load_voltage = lu.solve(no_load_rhs);
  if (lu.info() != Eigen::Success || !no_load_voltage.allFinite()) {
    return fail("zero-injection solve failed");
  }
  for (int local = 0; local < static_cast<int>(unknown_nodes.size()); ++local) {
    voltage[unknown_nodes[static_cast<std::size_t>(local)]] = no_load_voltage[local];
  }

  constexpr int kContinuationSteps = 20;
  for (int continuation = 1; continuation <= kContinuationSteps; ++continuation) {
    const double loading =
        static_cast<double>(continuation) / kContinuationSteps;
    for (int iteration = 0; iteration < 80; ++iteration) {
      Eigen::VectorXcd rhs(static_cast<int>(unknown_nodes.size()));
      for (int local = 0; local < static_cast<int>(unknown_nodes.size()); ++local) {
        const int node = unknown_nodes[static_cast<std::size_t>(local)];
        if (std::abs(voltage[node]) <= 0.2) {
          return fail("voltage collapsed below 0.2 p.u. at continuation step " +
                      std::to_string(continuation));
        }
        rhs[local] = loading * std::conj(specified_power[node] / voltage[node]) -
                     known_current[node];
      }
      const Eigen::VectorXcd candidate = lu.solve(rhs);
      if (lu.info() != Eigen::Success || !candidate.allFinite()) {
        return fail("Z-bus solve failed at continuation step " +
                    std::to_string(continuation));
      }
      double alpha = 0.65;
      Eigen::VectorXcd trial = voltage;
      while (alpha >= 1e-4) {
        bool acceptable = true;
        for (int local = 0; local < static_cast<int>(unknown_nodes.size()); ++local) {
          const int node = unknown_nodes[static_cast<std::size_t>(local)];
          trial[node] = (1.0 - alpha) * voltage[node] + alpha * candidate[local];
          acceptable = acceptable && std::abs(trial[node]) > 0.2;
        }
        if (acceptable) break;
        alpha *= 0.5;
      }
      if (alpha < 1e-4) {
        return fail("positive-voltage line search failed at continuation step " +
                    std::to_string(continuation));
      }
      double voltage_change = 0.0;
      for (int node : unknown_nodes) {
        voltage_change = std::max(voltage_change, std::abs(trial[node] - voltage[node]));
      }
      voltage = std::move(trial);
      if (voltage_change <= 1e-11) break;
    }
  }
  if (!voltage.allFinite()) return fail("continuation returned a nonfinite voltage");
  for (int node = 0; node < l.nv; ++node) {
    x[l.i_e + node] = std::real(voltage[node]);
    x[l.i_f + node] = std::imag(voltage[node]);
  }
  return true;
}

void project_converter_control(const ModelData& d, Eigen::VectorXd& x) {
  const Layout& l = d.layout;
  const Eigen::VectorXcd v = model_voltage(d, x);
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex current_conjugate_rotation[3] = {
      Complex{1.0, 0.0}, a, a * a};
  for (const auto& converter : d.converters) {
    Complex total_power{0.0, 0.0};
    for (int pos : converter.phase_var_positions) {
      total_power += Complex{x[l.i_pac + pos], x[l.i_qac + pos]};
    }
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      const Complex voltage_rotation[3] = {
          Complex{1.0, 0.0}, a * a, a};
      const Complex z{converter.virtual_r_pu, converter.virtual_x_pu};
      Complex denominator{0.0, 0.0};
      double voltage_norm_sum = 0.0;
      for (int phase = 0; phase < 3; ++phase) {
        const int node =
            converter.model_node_by_phase[static_cast<std::size_t>(phase)];
        denominator += v[node] * std::conj(voltage_rotation[phase]);
        voltage_norm_sum += std::norm(v[node]);
      }
      if (std::abs(denominator) <= 1e-12 || std::abs(z) <= 1e-12) continue;
      const Complex e_conjugate =
          (std::conj(z) * total_power + voltage_norm_sum) / denominator;
      const Complex e_a = std::conj(e_conjugate);
      x[l.i_gfm_e + converter.gfm_var_position] = std::real(e_a);
      x[l.i_gfm_f + converter.gfm_var_position] = std::imag(e_a);
      for (int phase = 0; phase < 3; ++phase) {
        const int pos =
            converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
        const int node =
            converter.model_node_by_phase[static_cast<std::size_t>(phase)];
        const Complex current =
            (voltage_rotation[phase] * e_a - v[node]) / z;
        const Complex power = v[node] * std::conj(current);
        x[l.i_pac + pos] = std::real(power);
        x[l.i_qac + pos] = std::imag(power);
      }
      continue;
    }
    if (converter.control_mode != PhaseVSCControlMode::GridFollowingPLL) {
      continue;
    }
    const Complex v1 =
        (v[converter.model_node_by_phase[0]] +
         a * v[converter.model_node_by_phase[1]] +
         a * a * v[converter.model_node_by_phase[2]]) /
        3.0;
    if (std::abs(v1) <= 1e-12) continue;
    const Complex ia_conjugate = total_power / (3.0 * v1);
    for (int phase = 0; phase < 3; ++phase) {
      const int pos =
          converter.phase_var_by_phase[static_cast<std::size_t>(phase)];
      const int node =
          converter.model_node_by_phase[static_cast<std::size_t>(phase)];
      const Complex power = v[node] *
          current_conjugate_rotation[phase] * ia_conjugate;
      x[l.i_pac + pos] = std::real(power);
      x[l.i_qac + pos] = std::imag(power);
    }
  }
}

Eigen::VectorXd build_initial_point(const ModelData& d) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(l.nvar);
  for (int i = 0; i < l.nv; ++i) {
    const int full_node = d.reduction.retained[static_cast<std::size_t>(i)];
    x[l.i_e + i] = std::real(c.voltage_start[full_node]);
    x[l.i_f + i] = std::imag(c.voltage_start[full_node]);
  }
  if (l.ndc > 0) x.segment(l.i_udc, l.ndc) = c.v_dc_start;
  for (int ci = 0; ci < l.nc; ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const Eigen::VectorXd u = x.segment(l.i_udc, l.ndc);
    const double pdc = u[converter.dc_terminal] *
                       (c.g_dc * u)[converter.dc_terminal];
    x[l.i_pdc + ci] = pdc;
    const double pac = -pdc /
        (converter.efficiency * converter.phase_var_positions.size());
    for (int pos : converter.phase_var_positions) x[l.i_pac + pos] = pac;
    if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      const Complex internal = std::polar(
          converter.voltage_reference_pu,
          converter.voltage_reference_angle_rad);
      x[l.i_gfm_e + converter.gfm_var_position] = std::real(internal);
      x[l.i_gfm_f + converter.gfm_var_position] = std::imag(internal);
    }
  }

  initialize_ac_power_flow(d, x);
  for (int pass = 0; pass < 3; ++pass) {
    project_converter_control(d, x);
    initialize_ac_power_flow(d, x);
  }
  project_converter_control(d, x);

  const Eigen::VectorXcd v = model_voltage(d, x);
  const Eigen::VectorXcd current = d.y * v + d.fixed_current;
  for (int gi = 0; gi < l.ng; ++gi) {
    const auto& generator = c.generators[static_cast<std::size_t>(gi)];
    const int node = d.generator_nodes[static_cast<std::size_t>(gi)];
    const int full_node = d.reduction.retained[static_cast<std::size_t>(node)];
    double p = std::real(v[node] * std::conj(current[node])) +
               c.p_load_pu[full_node];
    double q = std::imag(v[node] * std::conj(current[node])) +
               c.q_load_pu[full_node];
    for (const auto& converter : d.converters) {
      for (int local = 0;
           local < static_cast<int>(converter.model_phase_nodes.size()); ++local) {
        if (converter.model_phase_nodes[static_cast<std::size_t>(local)] != node) {
          continue;
        }
        const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
        p -= x[l.i_pac + pos];
        q -= x[l.i_qac + pos];
      }
    }
    x[l.i_pg + gi] = std::clamp(p, generator.p_min_pu, generator.p_max_pu);
    x[l.i_qg + gi] = std::clamp(q, generator.q_min_pu, generator.q_max_pu);
  }
  return x;
}

Eigen::VectorXd select_inequality_rows(
    const Eigen::VectorXd& full,
    const std::vector<int>& rows) {
  Eigen::VectorXd selected(static_cast<int>(rows.size()));
  for (int local = 0; local < static_cast<int>(rows.size()); ++local) {
    selected[local] = full[rows[static_cast<std::size_t>(local)]];
  }
  return selected;
}

Eigen::VectorXd expand_inequality_multipliers(
    const Eigen::VectorXd& selected,
    const std::vector<int>& rows,
    int full_count) {
  Eigen::VectorXd full = Eigen::VectorXd::Zero(full_count);
  const int count = std::min(
      static_cast<int>(rows.size()), static_cast<int>(selected.size()));
  for (int local = 0; local < count; ++local) {
    full[rows[static_cast<std::size_t>(local)]] = selected[local];
  }
  return full;
}

engine::NLPModel build_nlp(const std::shared_ptr<ModelData>& d,
                           const Eigen::VectorXd* primal_start = nullptr) {
  const Layout& l = d->layout;
  const auto& c = *d->source;
  engine::NLPModel nlp;
  nlp.sense = engine::Sense::Minimize;
  nlp.vars.resize(static_cast<std::size_t>(l.nvar));
  for (auto& var : nlp.vars) {
    var.type = engine::VarType::Continuous;
    var.lb = -1e20;
    var.ub = 1e20;
  }
  for (int i = 0; i < l.nv; ++i) {
    nlp.vars[static_cast<std::size_t>(l.i_e + i)].lb = -1.2;
    nlp.vars[static_cast<std::size_t>(l.i_e + i)].ub = 1.2;
    nlp.vars[static_cast<std::size_t>(l.i_f + i)].lb = -1.2;
    nlp.vars[static_cast<std::size_t>(l.i_f + i)].ub = 1.2;
  }
  for (int gi = 0; gi < l.ng; ++gi) {
    const auto& generator = c.generators[static_cast<std::size_t>(gi)];
    nlp.vars[static_cast<std::size_t>(l.i_pg + gi)].lb = generator.p_min_pu;
    nlp.vars[static_cast<std::size_t>(l.i_pg + gi)].ub = generator.p_max_pu;
    nlp.vars[static_cast<std::size_t>(l.i_qg + gi)].lb = generator.q_min_pu;
    nlp.vars[static_cast<std::size_t>(l.i_qg + gi)].ub = generator.q_max_pu;
  }
  for (int k = 0; k < l.ndc; ++k) {
    nlp.vars[static_cast<std::size_t>(l.i_udc + k)].lb = c.v_dc_min_pu[k];
    nlp.vars[static_cast<std::size_t>(l.i_udc + k)].ub = c.v_dc_max_pu[k];
  }
  for (int pos = 0; pos < l.ncp; ++pos) {
    const int ci = d->converter_for_phase_var[static_cast<std::size_t>(pos)];
    const double cap = d->converters[static_cast<std::size_t>(ci)].s_max_pu;
    nlp.vars[static_cast<std::size_t>(l.i_pac + pos)].lb = -cap;
    nlp.vars[static_cast<std::size_t>(l.i_pac + pos)].ub = cap;
    nlp.vars[static_cast<std::size_t>(l.i_qac + pos)].lb =
        d->converters[static_cast<std::size_t>(ci)].fixed_unity_power_factor
            ? 0.0 : -cap;
    nlp.vars[static_cast<std::size_t>(l.i_qac + pos)].ub =
        d->converters[static_cast<std::size_t>(ci)].fixed_unity_power_factor
            ? 0.0 : cap;
  }
  for (int pos = 0; pos < l.ngfm; ++pos) {
    nlp.vars[static_cast<std::size_t>(l.i_gfm_e + pos)].lb = -1.5;
    nlp.vars[static_cast<std::size_t>(l.i_gfm_e + pos)].ub = 1.5;
    nlp.vars[static_cast<std::size_t>(l.i_gfm_f + pos)].lb = -1.5;
    nlp.vars[static_cast<std::size_t>(l.i_gfm_f + pos)].ub = 1.5;
  }
  for (int ci = 0; ci < l.nc; ++ci) {
    const double cap = d->converters[static_cast<std::size_t>(ci)].s_max_pu;
    nlp.vars[static_cast<std::size_t>(l.i_pdc + ci)].lb = -cap;
    nlp.vars[static_cast<std::size_t>(l.i_pdc + ci)].ub = cap;
  }
  nlp.x0 = primal_start != nullptr &&
                   primal_start->size() == l.nvar &&
                   primal_start->allFinite()
      ? *primal_start : build_initial_point(*d);
  nlp.f = [d](const Eigen::VectorXd& x) { return objective(*d, x); };
  nlp.grad = [d](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    objective_gradient(*d, x, g);
  };
  nlp.hess = [d](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& h) {
    objective_hessian(*d, x, h);
  };
  nlp.lagrangian_hess =
      [d](const Eigen::VectorXd& x,
          const Eigen::VectorXd& lambda,
          const Eigen::VectorXd* nu,
          Eigen::SparseMatrix<double>& h) {
        if (nu == nullptr) {
          lagrangian_hessian(*d, x, lambda, nullptr, h);
          return;
        }
        const Eigen::VectorXd full_nu = expand_inequality_multipliers(
            *nu, d->enforced_inequality_rows, d->layout.nineq);
        lagrangian_hessian(*d, x, lambda, &full_nu, h);
      };
  nlp.g = [d](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    evaluate_equalities(*d, x, g);
  };
  nlp.jac_g = [d](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& j) {
    equality_jacobian(*d, x, j);
  };
  nlp.h = [d](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    Eigen::VectorXd full;
    evaluate_inequalities(*d, x, full);
    h = select_inequality_rows(full, d->enforced_inequality_rows);
  };
  nlp.jac_h = [d](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& j) {
    inequality_jacobian(*d, x, j, &d->enforced_inequality_rows);
  };
  // Wachter--Biegler (2006), Sections 2--3: a restored filter-IPM start is
  // preserved only after an independent max-norm feasibility audit. Undo the
  // internal equality row scaling so this callback reports the OPF's original
  // per-unit coordinates; MIPSolvers audits variable bounds independently.
  nlp.original_constraint_violation = [d](const Eigen::VectorXd& x) {
    Eigen::VectorXd equalities;
    Eigen::VectorXd full_inequalities;
    evaluate_equalities(*d, x, equalities);
    evaluate_inequalities(*d, x, full_inequalities);
    if (equalities.size() != d->equality_scale.size() ||
        !equalities.allFinite() || !full_inequalities.allFinite()) {
      return std::numeric_limits<double>::infinity();
    }
    equalities.array() /= d->equality_scale.array();
    const Eigen::VectorXd inequalities = select_inequality_rows(
        full_inequalities, d->enforced_inequality_rows);
    const double equality_violation = equalities.size() > 0
        ? equalities.cwiseAbs().maxCoeff() : 0.0;
    const double inequality_violation = inequalities.size() > 0
        ? std::max(0.0, inequalities.maxCoeff()) : 0.0;
    return std::max(equality_violation, inequality_violation);
  };
  const int equality_nullity = l.nvar - l.neq;
  if (equality_nullity == l.nc) {
    nlp.equality_free_columns.reserve(static_cast<std::size_t>(l.nc));
    for (const auto& converter : d->converters) {
      nlp.equality_free_columns.push_back(
          l.i_qac + converter.phase_var_positions.front());
    }
  }
  return nlp;
}

void initialize_primal_dual_start(const engine::NLPModel& nlp,
                                  const Eigen::VectorXd* equality_dual_seed,
                                  const Eigen::VectorXd* inequality_dual_seed,
                                  const Eigen::VectorXd* slack_seed,
                                  double* initial_dual_residual,
                                  double* dual_fit_residual,
                                  engine::IPMOptions& options,
                                  int max_factorizations =
                                      std::numeric_limits<int>::max(),
                                  int* factorization_count = nullptr) {
  const int n = static_cast<int>(nlp.vars.size());
  if (nlp.x0.size() != n || !nlp.x0.allFinite()) return;

  Eigen::VectorXd gradient;
  Eigen::VectorXd equalities;
  Eigen::VectorXd nonlinear_inequalities;
  Eigen::SparseMatrix<double> equality_jacobian;
  Eigen::SparseMatrix<double> nonlinear_jacobian;
  nlp.grad(nlp.x0, gradient);
  nlp.g(nlp.x0, equalities);
  nlp.jac_g(nlp.x0, equality_jacobian);
  nlp.h(nlp.x0, nonlinear_inequalities);
  nlp.jac_h(nlp.x0, nonlinear_jacobian);
  const double primal_neighborhood = std::max(
      equalities.size() > 0 ? equalities.cwiseAbs().maxCoeff() : 0.0,
      nonlinear_inequalities.size() > 0
          ? std::max(0.0, nonlinear_inequalities.maxCoeff())
          : 0.0);
  const double dual_initialization_neighborhood =
      options.central_warm_start
      ? options.central_warm_start_primal_tolerance : 1e-5;
  if (primal_neighborhood > dual_initialization_neighborhood) return;

  std::vector<int> lower_bound_variables;
  std::vector<int> upper_bound_variables;
  for (int col = 0; col < n; ++col) {
    const auto& variable = nlp.vars[static_cast<std::size_t>(col)];
    if (std::isfinite(variable.lb) && std::abs(variable.lb) < 1e19) {
      lower_bound_variables.push_back(col);
    }
    if (std::isfinite(variable.ub) && std::abs(variable.ub) < 1e19) {
      upper_bound_variables.push_back(col);
    }
  }

  const int nonlinear_count = static_cast<int>(nonlinear_inequalities.size());
  const int inequality_count =
      nonlinear_count + static_cast<int>(lower_bound_variables.size()) +
      static_cast<int>(upper_bound_variables.size());
  Eigen::VectorXd inequality_values = Eigen::VectorXd::Zero(inequality_count);
  inequality_values.head(nonlinear_count) = nonlinear_inequalities;
  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(
      nonlinear_jacobian.nonZeros() + lower_bound_variables.size() +
      upper_bound_variables.size()));
  for (int col = 0; col < nonlinear_jacobian.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(nonlinear_jacobian, col);
         it; ++it) {
      trips.emplace_back(it.row(), it.col(), it.value());
    }
  }
  int row = nonlinear_count;
  for (int col : lower_bound_variables) {
    inequality_values[row] =
        nlp.vars[static_cast<std::size_t>(col)].lb - nlp.x0[col];
    trips.emplace_back(row++, col, -1.0);
  }
  for (int col : upper_bound_variables) {
    inequality_values[row] =
        nlp.x0[col] - nlp.vars[static_cast<std::size_t>(col)].ub;
    trips.emplace_back(row++, col, 1.0);
  }
  Eigen::SparseMatrix<double> inequality_jacobian(inequality_count, n);
  inequality_jacobian.setFromTriplets(trips.begin(), trips.end());
  inequality_jacobian.makeCompressed();

  const bool has_inequality_dual_seed =
      inequality_dual_seed != nullptr &&
      inequality_dual_seed->size() == nonlinear_count &&
      inequality_dual_seed->allFinite();
  // Slack is a primal state tied to the current Full point by h(x)+s=0.  A
  // reduced-model slack cannot be copied after voltage recovery; reconstruct
  // it first, then transfer the inequality multiplier.
  const double max_positive_inequality = inequality_values.size() > 0
      ? std::max(0.0, inequality_values.maxCoeff()) : 0.0;
  // Use the remaining central-corridor margin to bound mu_i=mu_0/s_i while
  // retaining h_i+s_i<=epsilon_p. This is O(m) and adds no KKT factorization.
  const double slack_floor = options.central_warm_start
      ? std::max(2e-10,
          0.5 * (options.central_warm_start_primal_tolerance -
                 max_positive_inequality))
      : 1e-2;
  options.slack_start = has_inequality_dual_seed
      ? (-inequality_values.array()).max(2e-10).matrix()
      : (-inequality_values.array()).max(slack_floor).matrix();
  if (!has_inequality_dual_seed && slack_seed != nullptr &&
      slack_seed->size() == nonlinear_count && slack_seed->allFinite() &&
      (slack_seed->array() > 0.0).all()) {
    options.slack_start.head(nonlinear_count) = *slack_seed;
  }
  const double barrier = std::max(options.mu_min, options.mu_init);
  options.inequality_dual_start =
      (barrier / options.slack_start.array()).matrix();
  if (has_inequality_dual_seed) {
    options.inequality_dual_start.head(nonlinear_count) =
        inequality_dual_seed->cwiseMax(1e-12);
    const double recovered_barrier =
        options.slack_start.head(nonlinear_count).dot(
            options.inequality_dual_start.head(nonlinear_count)) /
        std::max(1, nonlinear_count);
    options.mu_init = std::clamp(recovered_barrier,
                                 options.mu_min, 0.1);
    // Project only complementarity outliers into the accepted central
    // neighborhood after slacks are reconstructed at the perturbed primal
    // point. This is the componentwise neighborhood
    // |s_i z_i / mu - 1| <= eta from the primal-dual path definition; duals
    // already inside it are retained exactly. Waechter--Biegler (2006),
    // Algorithm 1 and Section 2.2.
    // Keep a 10% interior margin so the subsequent independent audit cannot
    // reject a boundary value because of one rounding ulp.
    const double centrality_tolerance = 0.9 *
        std::max(0.0, options.central_warm_start_centrality_tolerance);
    for (int i = 0; i < nonlinear_count; ++i) {
      const double central_dual = options.mu_init / options.slack_start[i];
      const double lower = std::max(
          1e-12, (1.0 - centrality_tolerance) * central_dual);
      const double upper =
          (1.0 + centrality_tolerance) * central_dual;
      options.inequality_dual_start[i] = std::clamp(
          options.inequality_dual_start[i], lower, upper);
    }
    if (inequality_count > nonlinear_count) {
      options.inequality_dual_start.tail(
          inequality_count - nonlinear_count) =
          (options.mu_init /
           options.slack_start.tail(inequality_count - nonlinear_count).array())
              .matrix();
    }
  }

  Eigen::VectorXd stationarity_without_equalities =
      gradient + inequality_jacobian.transpose() *
                     options.inequality_dual_start;

  // If the model supplies independent control columns, recover equality
  // multipliers from stationarity on the complementary state variables:
  // J_B' lambda = -r_B. The remaining control components are exactly the
  // reduced gradient and must not be erased by a global least-squares fit.
  const int equality_count = static_cast<int>(equalities.size());
  if (equality_dual_seed == nullptr && equality_count <= n &&
      static_cast<int>(nlp.equality_free_columns.size()) ==
          n - equality_count) {
    std::vector<bool> is_free(static_cast<std::size_t>(n), false);
    bool valid_partition = true;
    for (int col : nlp.equality_free_columns) {
      if (col < 0 || col >= n || is_free[static_cast<std::size_t>(col)]) {
        valid_partition = false;
        break;
      }
      is_free[static_cast<std::size_t>(col)] = true;
    }
    std::vector<int> basic_columns;
    std::vector<int> variable_to_basic(static_cast<std::size_t>(n), -1);
    if (valid_partition) {
      basic_columns.reserve(static_cast<std::size_t>(equality_count));
      for (int col = 0; col < n; ++col) {
        if (!is_free[static_cast<std::size_t>(col)]) {
          variable_to_basic[static_cast<std::size_t>(col)] =
              static_cast<int>(basic_columns.size());
          basic_columns.push_back(col);
        }
      }
      valid_partition =
          static_cast<int>(basic_columns.size()) == equality_count;
    }
    if (valid_partition) {
      std::vector<Triplet> transpose_trips;
      transpose_trips.reserve(
          static_cast<std::size_t>(equality_jacobian.nonZeros()));
      for (int col = 0; col < equality_jacobian.outerSize(); ++col) {
        const int basic = variable_to_basic[static_cast<std::size_t>(col)];
        if (basic < 0) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(
                 equality_jacobian, col); it; ++it) {
          transpose_trips.emplace_back(basic, it.row(), it.value());
        }
      }
      Eigen::SparseMatrix<double> transpose_basis(
          equality_count, equality_count);
      transpose_basis.setFromTriplets(transpose_trips.begin(),
                                      transpose_trips.end());
      transpose_basis.makeCompressed();
      Eigen::SparseLU<Eigen::SparseMatrix<double>,
                      Eigen::COLAMDOrdering<int>> lu;
      if (max_factorizations <= 0) return;
      lu.analyzePattern(transpose_basis);
      lu.factorize(transpose_basis);
      --max_factorizations;
      if (factorization_count != nullptr) ++*factorization_count;
      if (lu.info() == Eigen::Success) {
        Eigen::VectorXd rhs(equality_count);
        for (int basic = 0; basic < equality_count; ++basic) {
          rhs[basic] = -stationarity_without_equalities[
              basic_columns[static_cast<std::size_t>(basic)]];
        }
        Eigen::VectorXd equality_dual = lu.solve(rhs);
        if (lu.info() == Eigen::Success && equality_dual.allFinite()) {
          options.equality_dual_start = std::move(equality_dual);
          if (initial_dual_residual != nullptr) {
            const Eigen::VectorXd stationarity =
                stationarity_without_equalities + equality_jacobian.transpose() *
                                                     options.equality_dual_start;
            const double multiplier_scale = 1.0 + std::max(
                options.equality_dual_start.cwiseAbs().maxCoeff(),
                options.inequality_dual_start.cwiseAbs().maxCoeff());
            *initial_dual_residual =
                stationarity.cwiseAbs().maxCoeff() / multiplier_scale;
            if (dual_fit_residual != nullptr) {
              double basic_residual = 0.0;
              for (int col : basic_columns) {
                basic_residual =
                    std::max(basic_residual, std::abs(stationarity[col]));
              }
              *dual_fit_residual = basic_residual / multiplier_scale;
            }
          }
          return;
        }
      }
    }
  }

  std::vector<int> unknown_equalities;
  Eigen::VectorXd equality_dual = Eigen::VectorXd::Zero(equalities.size());
  const bool seeded = equality_dual_seed != nullptr &&
      equality_dual_seed->size() == equalities.size();
  for (int row = 0; row < equalities.size(); ++row) {
    if (seeded && std::isfinite((*equality_dual_seed)[row])) {
      equality_dual[row] = (*equality_dual_seed)[row];
    } else {
      unknown_equalities.push_back(row);
    }
  }
  if (seeded) {
    stationarity_without_equalities +=
        equality_jacobian.transpose() * equality_dual;
    if (unknown_equalities.empty()) {
      options.equality_dual_start = std::move(equality_dual);
      if (initial_dual_residual != nullptr) {
        const double multiplier_scale = 1.0 + std::max(
            options.equality_dual_start.cwiseAbs().maxCoeff(),
            options.inequality_dual_start.cwiseAbs().maxCoeff());
        *initial_dual_residual =
            stationarity_without_equalities.cwiseAbs().maxCoeff() /
            multiplier_scale;
        if (dual_fit_residual != nullptr) {
          const Eigen::VectorXd normal_residual =
              equality_jacobian * stationarity_without_equalities;
          *dual_fit_residual =
              (normal_residual.size() > 0
                   ? normal_residual.cwiseAbs().maxCoeff() : 0.0) /
              multiplier_scale;
        }
      }
      return;
    }
  }

  const bool schur_recovery = seeded && std::all_of(
      unknown_equalities.begin(), unknown_equalities.end(),
      [n](int index) { return index >= 0 && index < n; });
  const int stationarity_rows = schur_recovery
      ? static_cast<int>(unknown_equalities.size()) : n;
  std::vector<int> variable_to_recovery_row(
      static_cast<std::size_t>(n), -1);
  if (schur_recovery) {
    for (int local = 0; local < static_cast<int>(unknown_equalities.size());
         ++local) {
      variable_to_recovery_row[static_cast<std::size_t>(
          unknown_equalities[static_cast<std::size_t>(local)])] = local;
    }
  }
  std::vector<Triplet> stationarity_trips;
  for (int col = 0; col < equality_jacobian.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(equality_jacobian, col);
         it; ++it) {
      auto found = std::lower_bound(unknown_equalities.begin(),
                                    unknown_equalities.end(), it.row());
      if (found != unknown_equalities.end() && *found == it.row()) {
        const int recovery_row = schur_recovery
            ? variable_to_recovery_row[static_cast<std::size_t>(it.col())]
            : it.col();
        if (recovery_row < 0) continue;
        stationarity_trips.emplace_back(
            recovery_row,
            static_cast<int>(found - unknown_equalities.begin()),
            it.value());
      }
    }
  }
  Eigen::SparseMatrix<double> stationarity_matrix(
      stationarity_rows, static_cast<int>(unknown_equalities.size()));
  stationarity_matrix.setFromTriplets(stationarity_trips.begin(),
                                      stationarity_trips.end());
  stationarity_matrix.makeCompressed();
  Eigen::VectorXd column_scale =
      Eigen::VectorXd::Ones(unknown_equalities.size());
  for (int col = 0; col < stationarity_matrix.outerSize(); ++col) {
    double norm = 0.0;
    for (Eigen::SparseMatrix<double>::InnerIterator it(stationarity_matrix, col);
         it; ++it) {
      norm = std::max(norm, std::abs(it.value()));
    }
    column_scale[col] = 1.0 / std::max(norm, 1e-12);
    for (Eigen::SparseMatrix<double>::InnerIterator it(stationarity_matrix, col);
         it; ++it) {
      it.valueRef() *= column_scale[col];
    }
  }
  Eigen::VectorXd recovery_rhs(stationarity_rows);
  if (schur_recovery) {
    for (int local = 0; local < stationarity_rows; ++local) {
      recovery_rhs[local] = stationarity_without_equalities[
          unknown_equalities[static_cast<std::size_t>(local)]];
    }
  } else {
    recovery_rhs = stationarity_without_equalities;
  }
  Eigen::VectorXd scaled_dual;
  bool solved_dual = false;
  if (max_factorizations > 0) {
    Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr;
    qr.compute(stationarity_matrix);
    if (factorization_count != nullptr) ++*factorization_count;
    if (qr.info() != Eigen::Success) return;
    scaled_dual = qr.solve(-recovery_rhs);
    solved_dual = qr.info() == Eigen::Success && scaled_dual.allFinite();
  }
  if (solved_dual) {
    const Eigen::VectorXd recovered = column_scale.cwiseProduct(scaled_dual);
    for (int local = 0; local < static_cast<int>(unknown_equalities.size());
         ++local) {
      equality_dual[unknown_equalities[static_cast<std::size_t>(local)]] =
          recovered[local];
    }
    options.equality_dual_start = std::move(equality_dual);
    if (initial_dual_residual != nullptr) {
      const Eigen::VectorXd stationarity =
          gradient + inequality_jacobian.transpose() *
                         options.inequality_dual_start +
          equality_jacobian.transpose() * options.equality_dual_start;
      const double multiplier_scale = 1.0 + std::max(
          options.equality_dual_start.cwiseAbs().maxCoeff(),
          options.inequality_dual_start.cwiseAbs().maxCoeff());
      *initial_dual_residual =
          stationarity.cwiseAbs().maxCoeff() / multiplier_scale;
      if (dual_fit_residual != nullptr) {
        const Eigen::VectorXd normal_residual =
            equality_jacobian * stationarity;
        *dual_fit_residual =
            (normal_residual.size() > 0
                 ? normal_residual.cwiseAbs().maxCoeff() : 0.0) /
            multiplier_scale;
      }
    }
  }
}

struct PhaseOneRestorationResult {
  bool converged{false};
  bool budget_exhausted{false};
  int iterations{0};
  int factorizations{0};
  int backtracks{0};
  double initial_violation{std::numeric_limits<double>::infinity()};
  double final_violation{std::numeric_limits<double>::infinity()};
  double runtime_ms{0.0};
  std::string termination{"invalid-start"};
  std::string linear_solver{"unselected"};
};

double phase_one_violation(const engine::NLPModel& nlp,
                           const Eigen::VectorXd& point) {
  double violation = 0.0;
  if (nlp.original_constraint_violation) {
    violation = nlp.original_constraint_violation(point);
  } else {
    Eigen::VectorXd equality;
    Eigen::VectorXd inequality;
    nlp.g(point, equality);
    nlp.h(point, inequality);
    const double eq = equality.size() > 0
        ? equality.cwiseAbs().maxCoeff() : 0.0;
    const double ineq = inequality.size() > 0
        ? std::max(0.0, inequality.maxCoeff()) : 0.0;
    violation = std::max(eq, ineq);
  }
  if (!std::isfinite(violation) || violation < 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  for (int col = 0; col < point.size(); ++col) {
    const auto& variable = nlp.vars[static_cast<std::size_t>(col)];
    if (engine::variable_has_finite_lower_bound(variable.lb)) {
      violation = std::max(
          violation, std::max(0.0, variable.lb - point[col]));
    }
    if (engine::variable_has_finite_upper_bound(variable.ub)) {
      violation = std::max(
          violation, std::max(0.0, point[col] - variable.ub));
    }
  }
  return violation;
}

PhaseOneRestorationResult restore_primal_feasibility(
    const engine::NLPModel& nlp,
    Eigen::VectorXd& x,
    double tolerance,
    int max_iterations,
    int max_factorizations,
    int max_backtracks,
    double time_limit_ms) {
  PhaseOneRestorationResult result;
  const auto started = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&]() {
    return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
  };
  if (x.size() != static_cast<int>(nlp.vars.size()) || !x.allFinite()) {
    return result;
  }

  double residual = phase_one_violation(nlp, x);
  result.initial_violation = residual;
  Eigen::VectorXd best_x = x;
  double best_residual = residual;

  const int n = static_cast<int>(nlp.vars.size());
  Eigen::VectorXd initial_equalities;
  nlp.g(x, initial_equalities);
  const int equality_count = static_cast<int>(initial_equalities.size());
  std::vector<int> basic_columns;
  std::vector<int> variable_to_basic(static_cast<std::size_t>(n), -1);
  bool valid_partition = equality_count <= n &&
      static_cast<int>(nlp.equality_free_columns.size()) == n - equality_count;
  if (valid_partition) {
    std::vector<bool> is_free(static_cast<std::size_t>(n), false);
    for (int col : nlp.equality_free_columns) {
      if (col < 0 || col >= n || is_free[static_cast<std::size_t>(col)]) {
        valid_partition = false;
        break;
      }
      is_free[static_cast<std::size_t>(col)] = true;
    }
    if (valid_partition) {
      basic_columns.reserve(static_cast<std::size_t>(equality_count));
      for (int col = 0; col < n; ++col) {
        if (!is_free[static_cast<std::size_t>(col)]) {
          variable_to_basic[static_cast<std::size_t>(col)] =
              static_cast<int>(basic_columns.size());
          basic_columns.push_back(col);
        }
      }
      valid_partition =
          static_cast<int>(basic_columns.size()) == equality_count;
    }
  }

  Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
      basis_solver;
  bool basis_pattern_analyzed = false;
  for (; result.iterations < std::max(0, max_iterations) &&
         residual > tolerance;) {
    if (result.factorizations >= std::max(0, max_factorizations)) {
      result.budget_exhausted = true;
      result.termination = "factorization-budget";
      break;
    }
    if (time_limit_ms > 0.0 && elapsed_ms() >= time_limit_ms) {
      result.budget_exhausted = true;
      result.termination = "time-budget";
      break;
    }

    Eigen::VectorXd equality;
    Eigen::SparseMatrix<double> jacobian;
    nlp.g(x, equality);
    nlp.jac_g(x, jacobian);
    if (!equality.allFinite() || jacobian.rows() != equality_count ||
        jacobian.cols() != n) {
      result.termination = "invalid-jacobian";
      break;
    }
    Eigen::VectorXd row_scale = Eigen::VectorXd::Ones(equality.size());
    for (int col = 0; col < jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jacobian, col);
           it; ++it) {
        row_scale[it.row()] =
            std::max(row_scale[it.row()], std::abs(it.value()));
      }
    }
    row_scale = row_scale.cwiseInverse();
    for (int col = 0; col < jacobian.outerSize(); ++col) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(jacobian, col);
           it; ++it) {
        it.valueRef() *= row_scale[it.row()];
      }
    }
    const Eigen::VectorXd scaled_equality =
        row_scale.cwiseProduct(equality);
    Eigen::VectorXd step = Eigen::VectorXd::Zero(n);
    bool solved_step = false;
    if (valid_partition) {
      // Nocedal--Wright (2006), Section 11.1: hold the independent control
      // variables fixed and solve the square state/basic Newton correction.
      std::vector<Triplet> basis_trips;
      basis_trips.reserve(static_cast<std::size_t>(jacobian.nonZeros()));
      for (int col = 0; col < jacobian.outerSize(); ++col) {
        const int basic = variable_to_basic[static_cast<std::size_t>(col)];
        if (basic < 0) continue;
        for (Eigen::SparseMatrix<double>::InnerIterator it(jacobian, col);
             it; ++it) {
          basis_trips.emplace_back(it.row(), basic, it.value());
        }
      }
      Eigen::SparseMatrix<double> basis(equality_count, equality_count);
      basis.setFromTriplets(basis_trips.begin(), basis_trips.end());
      basis.makeCompressed();
      if (!basis_pattern_analyzed) {
        basis_solver.analyzePattern(basis);
        // Eigen SparseLU reports numerical status after factorize()/compute();
        // analyzePattern() only prepares the symbolic ordering.  Gating the
        // numerical factorization on info() here silently disabled the
        // Nocedal--Wright (2006), Section 11.1 state/basic Newton path.
        basis_pattern_analyzed = true;
      }
      if (basis_pattern_analyzed) {
        basis_solver.factorize(basis);
        ++result.factorizations;
        if (basis_solver.info() == Eigen::Success) {
          const Eigen::VectorXd basic_step =
              basis_solver.solve(-scaled_equality);
          if (basis_solver.info() == Eigen::Success &&
              basic_step.allFinite()) {
            for (int basic = 0; basic < equality_count; ++basic) {
              step[basic_columns[static_cast<std::size_t>(basic)]] =
                  basic_step[basic];
            }
            solved_step = true;
            result.linear_solver = "sparse-basis-lu";
          }
        }
      }
    }
    if (!solved_step) {
      if (time_limit_ms > 0.0 && elapsed_ms() >= time_limit_ms) {
        result.budget_exhausted = true;
        result.termination = "time-budget";
        break;
      }
      if (result.factorizations >= std::max(0, max_factorizations)) {
        result.budget_exhausted = true;
        result.termination = "factorization-budget";
        break;
      }
      // Nocedal--Wright (2006), Section 11.1: solve the rectangular Newton
      // correction directly. Sparse QR avoids both dense materialization and
      // the squared conditioning/fill of J*J^T normal equations.
      Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>
          qr;
      qr.compute(jacobian);
      ++result.factorizations;
      if (qr.info() != Eigen::Success) {
        result.termination = "singular-jacobian";
        break;
      }
      step = qr.solve(-scaled_equality);
      solved_step = qr.info() == Eigen::Success && step.allFinite();
      if (solved_step) result.linear_solver = "sparse-qr";
    }
    ++result.iterations;
    if (!solved_step) {
      result.termination = "linear-solve-failed";
      break;
    }

    bool accepted = false;
    double alpha = 1.0;
    const int backtrack_limit = std::max(0, max_backtracks);
    for (int line_search = 0; line_search <= backtrack_limit;
         ++line_search) {
      Eigen::VectorXd trial = x + alpha * step;
      for (int col = 0; col < trial.size(); ++col) {
        const auto& variable = nlp.vars[static_cast<std::size_t>(col)];
        trial[col] = std::min(variable.ub, std::max(variable.lb, trial[col]));
      }
      const double trial_residual = phase_one_violation(nlp, trial);
      // Deuflhard (2011), Sections 2.2--2.3: monotone damped Newton accepts
      // only a strict residual improvement; no Phase-II filter is duplicated.
      if (std::isfinite(trial_residual) && trial_residual < residual) {
        x = std::move(trial);
        residual = trial_residual;
        if (residual < best_residual) {
          best_residual = residual;
          best_x = x;
        }
        accepted = true;
        break;
      }
      if (line_search < backtrack_limit) {
        ++result.backtracks;
        alpha *= 0.5;
      }
    }
    if (!accepted) {
      result.termination = "no-decreasing-step";
      break;
    }
  }
  x = std::move(best_x);
  result.final_violation = best_residual;
  result.converged = std::isfinite(best_residual) &&
                     best_residual <= tolerance;
  if (result.converged) {
    result.termination = "certified";
  } else if (result.termination == "invalid-start") {
    result.budget_exhausted =
        result.iterations >= std::max(0, max_iterations);
    result.termination = result.budget_exhausted
        ? "iteration-budget" : "incomplete";
  }
  result.runtime_ms = elapsed_ms();
  return result;
}

double max_vuf(const ThreePhaseHybridOPFCase& c,
               const Eigen::VectorXcd& voltage) {
  double result = 0.0;
  for (const auto& nodes : c.three_phase_bus_nodes) {
    const auto [v1, v2] = sequence_components(voltage, nodes);
    if (std::abs(v1) > 1e-12) result = std::max(result, std::abs(v2) / std::abs(v1));
  }
  return result;
}

double max_converter_current_vuf(const ModelData& d,
                                 const Eigen::VectorXd& x,
                                 const Eigen::VectorXcd& voltage) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  double result = 0.0;
  for (int ci = 0; ci < static_cast<int>(d.converters.size()); ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const auto& source_converter = c.converters[static_cast<std::size_t>(ci)];
    if (converter.phase_var_positions.size() != 3 ||
        source_converter.phase_nodes.size() != 3 || c.ac_phase_index.empty()) {
      continue;
    }
    std::array<Complex, 3> phase_current{};
    std::array<bool, 3> present{};
    for (int local = 0; local < 3; ++local) {
      const int full_node =
          source_converter.phase_nodes[static_cast<std::size_t>(local)];
      const int phase = c.ac_phase_index[static_cast<std::size_t>(full_node)];
      if (phase < 0 || phase > 2 || std::abs(voltage[full_node]) <= 1e-12) {
        continue;
      }
      const int pos =
          converter.phase_var_positions[static_cast<std::size_t>(local)];
      const Complex power{x[l.i_pac + pos], x[l.i_qac + pos]};
      phase_current[static_cast<std::size_t>(phase)] =
          std::conj(power / voltage[full_node]);
      present[static_cast<std::size_t>(phase)] = true;
    }
    if (!present[0] || !present[1] || !present[2]) continue;
    const Complex i1 =
        (phase_current[0] + a * phase_current[1] + a2 * phase_current[2]) /
        3.0;
    const Complex i2 =
        (phase_current[0] + a2 * phase_current[1] + a * phase_current[2]) /
        3.0;
    result = std::max(
        result, std::abs(i2) / std::max(1e-12, std::abs(i1)));
  }
  return result;
}

double max_converter_current_loading(const ModelData& d,
                                     const Eigen::VectorXd& x,
                                     const Eigen::VectorXcd& voltage) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  double result = 0.0;
  for (int ci = 0; ci < static_cast<int>(d.converters.size()); ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const auto& source_converter = c.converters[static_cast<std::size_t>(ci)];
    for (int local = 0;
         local < static_cast<int>(converter.phase_var_positions.size());
         ++local) {
      const int pos =
          converter.phase_var_positions[static_cast<std::size_t>(local)];
      const int full_node =
          source_converter.phase_nodes[static_cast<std::size_t>(local)];
      if (std::abs(voltage[full_node]) <= 1e-12) continue;
      const Complex power{x[l.i_pac + pos], x[l.i_qac + pos]};
      const double current = std::abs(power / voltage[full_node]);
      result = std::max(
          result, current / std::max(1e-12, converter.phase_current_max_pu));
    }
  }
  return result;
}

std::vector<PhaseVSCDynamicEquilibrium> recover_dynamic_equilibria(
    const ModelData& d,
    const Eigen::VectorXd& x,
    const Eigen::VectorXcd& voltage,
    double* max_residual) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  std::vector<PhaseVSCDynamicEquilibrium> result;
  result.reserve(d.converters.size());
  double maximum = 0.0;
  for (int ci = 0; ci < static_cast<int>(d.converters.size()); ++ci) {
    const auto& converter = d.converters[static_cast<std::size_t>(ci)];
    const auto& source_converter = c.converters[static_cast<std::size_t>(ci)];
    PhaseVSCDynamicEquilibrium equilibrium;
    equilibrium.control_mode = converter.control_mode;
    std::array<Complex, 3> phase_voltage{};
    std::array<Complex, 3> phase_current{};
    Complex total_power{0.0, 0.0};
    for (int local = 0;
         local < static_cast<int>(converter.phase_var_positions.size());
         ++local) {
      const int pos =
          converter.phase_var_positions[static_cast<std::size_t>(local)];
      const int full_node =
          source_converter.phase_nodes[static_cast<std::size_t>(local)];
      const int phase = c.ac_phase_index[static_cast<std::size_t>(full_node)];
      const Complex power{x[l.i_pac + pos], x[l.i_qac + pos]};
      phase_voltage[static_cast<std::size_t>(phase)] = voltage[full_node];
      phase_current[static_cast<std::size_t>(phase)] =
          std::conj(power / voltage[full_node]);
      total_power += power;
    }
    // The transient inverter uses a three-phase device base, so its dq power
    // state is the average of the three phase-domain per-unit powers.
    equilibrium.active_power_reference_pu = std::real(total_power) / 3.0;
    equilibrium.reactive_power_reference_pu = std::imag(total_power) / 3.0;
    equilibrium.filtered_active_power_pu = std::real(total_power) / 3.0;
    equilibrium.filtered_reactive_power_pu = std::imag(total_power) / 3.0;
    const Complex v1 =
        (phase_voltage[0] + a * phase_voltage[1] + a2 * phase_voltage[2]) /
        3.0;
    if (converter.control_mode == PhaseVSCControlMode::GridFollowingPLL) {
      equilibrium.pll_angle_rad = std::arg(v1);
      const Complex i0 =
          (phase_current[0] + phase_current[1] + phase_current[2]) / 3.0;
      const Complex i2 =
          (phase_current[0] + a2 * phase_current[1] + a * phase_current[2]) /
          3.0;
      const Complex v_dq = v1 * std::exp(
          Complex{0.0, -equilibrium.pll_angle_rad});
      const double vq = std::imag(v_dq);
      equilibrium.pll_integrator = std::abs(converter.pll_ki) > 1e-12
          ? -converter.pll_kp * vq / converter.pll_ki
          : 0.0;
      equilibrium.max_differential_residual = std::max(
          {std::abs(i0), std::abs(i2),
           std::abs(converter.pll_kp * vq +
                    converter.pll_ki * equilibrium.pll_integrator)});
    } else if (converter.control_mode ==
               PhaseVSCControlMode::GridFormingDroop) {
      const Complex e_a{x[l.i_gfm_e + converter.gfm_var_position],
                        x[l.i_gfm_f + converter.gfm_var_position]};
      equilibrium.internal_voltage_positive = e_a;
      const double vt =
          (std::abs(phase_voltage[0]) + std::abs(phase_voltage[1]) +
           std::abs(phase_voltage[2])) /
          3.0;
      equilibrium.voltage_reference_pu = vt;
      equilibrium.voltage_integrator =
          std::abs(converter.voltage_integral_gain) > 1e-12
          ? (std::abs(e_a) - vt) / converter.voltage_integral_gain
          : 0.0;
      const Complex z{converter.virtual_r_pu, converter.virtual_x_pu};
      const Complex rotation[3] = {Complex{1.0, 0.0}, a2, a};
      double residual = 0.0;
      for (int phase = 0; phase < 3; ++phase) {
        residual = std::max(
            residual,
            std::abs(rotation[phase] * e_a - phase_voltage[phase] -
                     z * phase_current[phase]));
      }
      equilibrium.max_differential_residual = residual;
    }
    maximum = std::max(maximum, equilibrium.max_differential_residual);
    result.push_back(equilibrium);
  }
  if (max_residual != nullptr) *max_residual = maximum;
  return result;
}

}  // namespace

ThreePhaseHybridOPFResult solve_three_phase_hybrid_opf_impl(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFOptions& options,
    const std::shared_ptr<ModelData>& prebuilt_data = nullptr) {
  const auto function_start = std::chrono::steady_clock::now();
  const auto log_stage = [&](const std::string& stage) {
    if (!options.verbose) return;
    std::cerr << '[' << problem.name << "] " << stage << " at "
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - function_start).count()
              << " s\n" << std::flush;
  };
  if (options.use_constraint_oracle) {
    ThreePhaseHybridOPFOptions screening_options = options;
    screening_options.use_constraint_oracle = false;
    screening_options.enforced_inequality_rows.clear();
    std::shared_ptr<ModelData> screening_data;
    if (prebuilt_data != nullptr) {
      screening_data = std::make_shared<ModelData>(*prebuilt_data);
      screening_data->source = &problem;
      configure_enforced_inequalities(*screening_data, {});
    } else {
      screening_data = build_model_data(problem, screening_options);
    }
    Eigen::VectorXd current_start =
        options.primal_start.size() == screening_data->layout.nvar &&
                options.primal_start.allFinite()
            ? options.primal_start : build_initial_point(*screening_data);
    Eigen::VectorXd initial_h;
    evaluate_inequalities(*screening_data, current_start, initial_h);
    const double initial_margin = std::max(
        options.oracle_activation_margin, options.oracle_initial_margin);
    std::vector<bool> enforced_mask(
        static_cast<std::size_t>(screening_data->layout.nineq), false);
    for (int row : options.oracle_seed_rows) {
      if (row < 0 || row >= screening_data->layout.nineq) {
        throw std::invalid_argument(
            "constraint-oracle seed row is outside the full inequality range");
      }
      enforced_mask[static_cast<std::size_t>(row)] = true;
    }
    const int converter_row =
        2 * screening_data->reduction.original_size +
        static_cast<int>(problem.three_phase_bus_nodes.size());
    // Converter capability and phase-current rows are always enforced; the
    // voltage, VUF, and recovered line-current rows are screened by the oracle.
    const int line_row_start =
        converter_row + screening_data->layout.nc + screening_data->layout.ncp;
    for (int row = 0; row < initial_h.size(); ++row) {
      if (initial_h[row] >= -initial_margin ||
          (row >= converter_row && row < line_row_start)) {
        enforced_mask[static_cast<std::size_t>(row)] = true;
      }
    }
    std::vector<int> enforced_rows;
    for (int row = 0; row < initial_h.size(); ++row) {
      if (enforced_mask[static_cast<std::size_t>(row)]) {
        enforced_rows.push_back(row);
      }
    }
    log_stage("constraint oracle: initial rows=" +
              std::to_string(enforced_rows.size()) + "/" +
              std::to_string(screening_data->layout.nineq));

    ThreePhaseHybridOPFResult latest;
    std::vector<ConstraintOracleRoundDiagnostic> oracle_trace;
    int total_iterations = 0;
    int total_added = 0;
    bool exact_corrector_required = false;
    const int max_rounds = std::max(1, options.oracle_max_rounds);
    for (int round = 0; round < max_rounds; ++round) {
      ThreePhaseHybridOPFOptions round_options = options;
      round_options.use_constraint_oracle = false;
      round_options.enforced_inequality_rows = enforced_rows;
      round_options.primal_start = current_start;
      const bool predictor_round =
          !exact_corrector_required && options.oracle_probe_iterations > 0 &&
          options.oracle_probe_iterations < options.max_iterations;
      if (predictor_round) {
        round_options.max_iterations = options.oracle_probe_iterations;
      }
      if (round > 0) {
        // The previous restricted solution remains equality-feasible.  Newly
        // activated inequalities are handled by the infeasible-start filter;
        // an Ipopt feasibility solve would discard the oracle continuation.
        round_options.warm_start_with_ipopt = false;
        round_options.equality_dual_start = latest.equality_dual;
        round_options.nonlinear_inequality_dual_start =
            latest.inequality_dual.size() >= screening_data->layout.nineq
                ? latest.inequality_dual.head(screening_data->layout.nineq)
                : Eigen::VectorXd{};
        round_options.nonlinear_slack_start =
            latest.inequality_slack.size() >= screening_data->layout.nineq
                ? latest.inequality_slack.head(screening_data->layout.nineq)
                : Eigen::VectorXd{};
        round_options.variable_lower_bound_dual_start =
            latest.variable_lower_bound_dual;
        round_options.variable_upper_bound_dual_start =
            latest.variable_upper_bound_dual;
      }
      log_stage("constraint oracle: round=" + std::to_string(round + 1) +
                ", rows=" + std::to_string(enforced_rows.size()));
      const int rows_before_solve = static_cast<int>(enforced_rows.size());
      const auto round_start = std::chrono::steady_clock::now();
      auto round_data = std::make_shared<ModelData>(*screening_data);
      configure_enforced_inequalities(*round_data, enforced_rows);
      latest = solve_three_phase_hybrid_opf_impl(
          problem, round_options, round_data);
      total_iterations += latest.iterations;
      if (latest.primal.size() != screening_data->layout.nvar ||
          !latest.primal.allFinite()) {
        latest.converged = false;
        latest.status = "Constraint oracle: restricted solve returned no "
                        "finite primal point";
        latest.constraint_oracle_rounds = round + 1;
        latest.constraint_oracle_added_rows = total_added;
        latest.runtime_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - function_start).count();
        return latest;
      }
      current_start = latest.primal;
      Eigen::VectorXd full_h;
      evaluate_inequalities(*screening_data, current_start, full_h);
      int added_this_round = 0;
      for (int row = 0; row < full_h.size(); ++row) {
        if (!enforced_mask[static_cast<std::size_t>(row)] &&
            full_h[row] >= -options.oracle_activation_margin) {
          enforced_mask[static_cast<std::size_t>(row)] = true;
          enforced_rows.push_back(row);
          ++added_this_round;
        }
      }
      std::sort(enforced_rows.begin(), enforced_rows.end());
      total_added += added_this_round;
      oracle_trace.push_back({
          round + 1,
          rows_before_solve,
          added_this_round,
          latest.iterations,
          latest.converged,
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - round_start).count(),
          full_h.maxCoeff()});
      log_stage("constraint oracle: checked, added=" +
                std::to_string(added_this_round) +
                ", full_max=" + std::to_string(full_h.maxCoeff()));
      if (predictor_round) {
        if (added_this_round == 0 && latest.converged) {
          latest.constraint_oracle_rounds = round + 1;
          latest.constraint_oracle_added_rows = total_added;
          latest.constraint_oracle_trace = oracle_trace;
          latest.iterations = total_iterations;
          latest.runtime_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - function_start).count();
          return latest;
        }
        exact_corrector_required = added_this_round == 0;
        if (exact_corrector_required) {
          log_stage("constraint oracle: active set stable; exact corrector "
                    "required");
        }
        continue;
      }
      if (added_this_round == 0) {
        latest.constraint_oracle_rounds = round + 1;
        latest.constraint_oracle_added_rows = total_added;
        latest.constraint_oracle_trace = oracle_trace;
        latest.iterations = total_iterations;
        latest.runtime_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - function_start).count();
        return latest;
      }
      exact_corrector_required = false;
    }
    latest.converged = false;
    latest.status = "Constraint oracle: enrichment round limit reached";
    latest.constraint_oracle_rounds = max_rounds;
    latest.constraint_oracle_added_rows = total_added;
    latest.constraint_oracle_trace = oracle_trace;
    latest.iterations = total_iterations;
    latest.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - function_start).count();
    return latest;
  }
  log_stage("model build: start");
  const auto data = prebuilt_data != nullptr
      ? prebuilt_data : build_model_data(problem, options);
  const Eigen::VectorXd* supplied_primal =
      options.primal_start.size() == data->layout.nvar &&
              options.primal_start.allFinite()
          ? &options.primal_start : nullptr;
  engine::NLPModel nlp = build_nlp(data, supplied_primal);
  log_stage("model build: done (n=" + std::to_string(data->layout.nvar) +
            ", meq=" + std::to_string(data->layout.neq) +
            ", mineq=" +
            std::to_string(data->enforced_inequality_rows.size()) + "/" +
            std::to_string(data->layout.nineq) + ")");
  if (options.primal_start.size() == data->layout.nvar &&
      options.primal_start.allFinite()) {
    nlp.x0 = options.primal_start;
  }
  if (options.verbose) {
    const Eigen::VectorXcd recovered_start = full_voltage(*data, nlp.x0);
    Eigen::Index min_voltage_node = 0;
    recovered_start.cwiseAbs().minCoeff(&min_voltage_node);
    const auto& recovery_row =
        data->recovery_rows[static_cast<std::size_t>(min_voltage_node)];
    double recovery_row_norm = 0.0;
    for (const auto& [unused, coefficient] : recovery_row) {
      (void)unused;
      recovery_row_norm += std::norm(coefficient);
    }
    recovery_row_norm = std::sqrt(recovery_row_norm);
    std::cerr << '[' << problem.name << "] recovery diagnostic: full-node="
              << min_voltage_node << ", retained-position="
              << data->full_to_model[static_cast<std::size_t>(min_voltage_node)]
              << ", original-start="
              << std::abs(problem.voltage_start[min_voltage_node])
              << ", recovered-start=" << std::abs(recovered_start[min_voltage_node])
              << ", row-nnz=" << recovery_row.size()
              << ", row-norm=" << recovery_row_norm << "\n"
              << std::flush;
  }
  nlp.solver_options.max_iterations = options.max_iterations;
  nlp.solver_options.tolerance = options.tolerance;
  // Kron-reduced OPF assembles dense recovered rows whose poor central-path
  // centrality inflates the monotone iteration count; the adaptive barrier
  // roughly halves it and is exact-preserving (verified on H13--H8500).
  nlp.solver_options.adaptive_barrier = true;
  nlp.solver_options.acceptable_tolerance = 10.0 * options.tolerance;
  nlp.solver_options.constraint_violation_tolerance = options.tolerance;
  nlp.solver_options.dual_infeasibility_tolerance = options.tolerance;
  nlp.solver_options.complementarity_tolerance = options.tolerance;
  nlp.solver_options.acceptable_constraint_violation_tolerance =
      10.0 * options.tolerance;
  nlp.solver_options.acceptable_dual_infeasibility_tolerance =
      10.0 * options.tolerance;
  nlp.solver_options.acceptable_complementarity_tolerance =
      10.0 * options.tolerance;

  engine::SolveResult solved;
  engine::IPMDetail detail;
  double initial_dual_residual = std::numeric_limits<double>::infinity();
  double phase_one_dual_fit_residual =
      std::numeric_limits<double>::infinity();
  double phase_one_constraint_violation =
      std::numeric_limits<double>::infinity();
  const double phase_one_barrier_mu = std::clamp(
      std::isfinite(options.phase_one_barrier_mu) &&
              options.phase_one_barrier_mu > 0.0
          ? options.phase_one_barrier_mu
          : 0.1,
      1e-12, 0.1);
  const double phase_one_primal_mu_factor =
      std::isfinite(options.phase_one_primal_mu_factor)
      ? std::max(0.0, options.phase_one_primal_mu_factor) : 0.1;
  const double phase_one_tolerance = std::max(
      options.tolerance,
      phase_one_primal_mu_factor * phase_one_barrier_mu);
  const double phase_one_admission_mu_factor =
      std::isfinite(options.phase_one_admission_mu_factor)
      ? std::max(0.0, options.phase_one_admission_mu_factor) : 1.0;
  bool phase_one_admitted = false;
  bool phase_one_dual_initialized = false;
  PhaseOneRestorationResult phase_one_result;
  const auto start = std::chrono::steady_clock::now();
  if (options.backend == SolverBackend::NativeIPM) {
    const auto phase_one_started = std::chrono::steady_clock::now();
    const auto phase_one_elapsed_ms = [&]() {
      return std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - phase_one_started).count();
    };
    double start_primal_residual = phase_one_violation(nlp, nlp.x0);
    phase_one_admitted = std::isfinite(start_primal_residual) &&
        start_primal_residual <=
            phase_one_admission_mu_factor * phase_one_barrier_mu;
    log_stage("initial residual=" + std::to_string(start_primal_residual));
    if (options.verbose) {
      Eigen::VectorXd debug_equalities;
      Eigen::VectorXd debug_inequalities;
      nlp.g(nlp.x0, debug_equalities);
      nlp.h(nlp.x0, debug_inequalities);
      Eigen::Index worst_eq = 0;
      Eigen::Index worst_ineq = 0;
      const double eq_residual = debug_equalities.size() > 0
          ? debug_equalities.cwiseAbs().maxCoeff(&worst_eq) : 0.0;
      const double ineq_residual = debug_inequalities.size() > 0
          ? std::max(0.0, debug_inequalities.maxCoeff(&worst_ineq)) : 0.0;
      std::cerr << '[' << problem.name << "] initial components: eq="
                << eq_residual << " (row " << worst_eq << "), ineq="
                << ineq_residual << " (row " << worst_ineq << ")\n"
                << std::flush;
      if (worst_eq >= 0 && worst_eq < 2 * data->layout.nv) {
        const bool reactive_row = worst_eq >= data->layout.nv;
        const int model_node = static_cast<int>(worst_eq) % data->layout.nv;
        const int full_node = data->reduction.retained[static_cast<std::size_t>(
            model_node)];
        const Eigen::VectorXcd voltage = model_voltage(*data, nlp.x0);
        const Eigen::VectorXcd current =
            data->y * voltage + data->fixed_current;
        const Complex power = voltage[model_node] * std::conj(current[model_node]);
        std::cerr << '[' << problem.name << "] worst AC balance: kind="
                  << (reactive_row ? 'Q' : 'P') << ", model-node=" << model_node
                  << ", full-node=" << full_node
                  << ", |V|=" << std::abs(voltage[model_node])
                  << ", Pnet=" << std::real(power)
                  << ", Qnet=" << std::imag(power)
                  << ", Pload=" << problem.p_load_pu[full_node]
                  << ", Qload=" << problem.q_load_pu[full_node] << "\n"
                  << std::flush;
      }
    }

    // Phase I enters an eta_p*mu_0 central corridor and fits multipliers on
    // the state/basic equations. It intentionally leaves the control-space
    // reduced gradient for Phase II. Final feasibility remains Phase II's
    // strict options.tolerance contract. Waechter--Biegler (2006), Sec. 2--3.
    log_stage("Phase I primal restoration: start");
    Eigen::VectorXd restored_start = nlp.x0;
    if (phase_one_admitted) {
      phase_one_result = restore_primal_feasibility(
          nlp, restored_start, phase_one_tolerance,
          options.phase_one_max_iterations,
          options.phase_one_max_factorizations,
          options.phase_one_max_backtracks,
          options.phase_one_time_limit_ms);
      if (restored_start.allFinite()) nlp.x0 = std::move(restored_start);
    } else {
      phase_one_result.initial_violation = start_primal_residual;
      phase_one_result.final_violation = start_primal_residual;
      phase_one_result.termination = "outside-phase-one-admission";
    }
    start_primal_residual = phase_one_violation(nlp, nlp.x0);
    log_stage("Phase I primal restoration: " +
              phase_one_result.termination + " (p=" +
              std::to_string(start_primal_residual) + ")");

    if (phase_one_admitted && options.warm_start_with_ipopt &&
        options.phase_one_time_limit_ms <= 0.0 &&
        start_primal_residual > 1e-5) {
      log_stage("Ipopt warm solve: start");
      engine::IpoptAdapter ipopt;
      engine::NLPModel feasibility_nlp = nlp;
      feasibility_nlp.solver_options.tolerance = 1e-9;
      feasibility_nlp.solver_options.acceptable_tolerance = 1e-8;
      feasibility_nlp.solver_options.constraint_violation_tolerance = 1e-9;
      feasibility_nlp.solver_options.acceptable_constraint_violation_tolerance =
          1e-8;
      const engine::SolveResult warm = ipopt.solve_nlp(feasibility_nlp);
      log_stage("Ipopt warm solve: done (status=" + warm.stats.status +
                ", p=" + std::to_string(warm.stats.unscaled_primal_feas) +
                ", d=" + std::to_string(warm.stats.unscaled_dual_feas) + ")");
      if (warm.x.size() == data->layout.nvar && warm.x.allFinite()) nlp.x0 = warm.x;
      log_stage("post-Ipopt Phase I primal restoration: start");
      restored_start = nlp.x0;
      const PhaseOneRestorationResult post_ipopt_result =
          restore_primal_feasibility(
              nlp, restored_start, phase_one_tolerance,
              std::max(0, options.phase_one_max_iterations -
                              phase_one_result.iterations),
              std::max(0, options.phase_one_max_factorizations -
                              phase_one_result.factorizations),
              options.phase_one_max_backtracks,
              options.phase_one_time_limit_ms);
      if (restored_start.allFinite()) nlp.x0 = std::move(restored_start);
      phase_one_result.converged = post_ipopt_result.converged;
      phase_one_result.budget_exhausted =
          phase_one_result.budget_exhausted ||
          post_ipopt_result.budget_exhausted;
      phase_one_result.iterations += post_ipopt_result.iterations;
      phase_one_result.factorizations += post_ipopt_result.factorizations;
      phase_one_result.backtracks += post_ipopt_result.backtracks;
      phase_one_result.final_violation = post_ipopt_result.final_violation;
      phase_one_result.termination = post_ipopt_result.termination;
      if (post_ipopt_result.linear_solver != "unselected") {
        phase_one_result.linear_solver = post_ipopt_result.linear_solver;
      }
      start_primal_residual = phase_one_violation(nlp, nlp.x0);
      log_stage("post-Ipopt Phase I primal restoration: " +
                phase_one_result.termination + " (p=" +
                std::to_string(start_primal_residual) + ")");
    }
    engine::IPMOptions ipm_options;
    ipm_options.max_iter = options.max_iterations;
    // Graph-reduced cold starts can follow an infeasible central path for
    // hundreds of factorizations even though the existing restoration phase
    // produces a rapidly convergent primal-dual retry. Trigger restoration
    // after bounded primary work, give restoration an independent work cap,
    // and preserve the retry's full user-requested budget. Waechter--Biegler (2006),
    // Sections 2.4 and 3.3; derivation and measurements are documented in
    // docs/modules/optimal_power_flow/chapters/three_phase.tex.
    const bool bound_reduced_restoration =
        options.variant == ModelVariant::GraphReduced;
    ipm_options.primary_max_iter_before_restoration =
        bound_reduced_restoration &&
                options.native_primary_max_iterations_before_restoration > 0
        ? std::min(options.max_iterations,
                   options.native_primary_max_iterations_before_restoration)
        : 0;
    ipm_options.restoration_max_iter = std::min(
        options.max_iterations,
        bound_reduced_restoration
        ? std::max(1, options.native_restoration_max_iterations)
        : 200);
    ipm_options.tol_primal = options.tolerance;
    ipm_options.tol_dual = options.tolerance;
    ipm_options.tol_complementarity = options.tolerance;
    ipm_options.tol_accept = 0.0;
    ipm_options.globalization = engine::Globalization::Filter;
    ipm_options.scale_problem = false;
    ipm_options.verbose = options.verbose;
    ipm_options.central_warm_start = phase_one_admitted;
    ipm_options.central_warm_start_primal_tolerance = phase_one_tolerance;
    ipm_options.central_warm_start_centrality_tolerance =
        std::isfinite(options.phase_one_centrality_tolerance)
        ? std::max(0.0, options.phase_one_centrality_tolerance) : 0.5;
    ipm_options.mu_init = phase_one_barrier_mu;
    const Eigen::VectorXd* equality_dual_seed =
        options.equality_dual_start.size() == data->layout.neq
            ? &options.equality_dual_start : nullptr;
    Eigen::VectorXd selected_inequality_dual_seed;
    Eigen::VectorXd selected_slack_seed;
    const int enforced_count =
        static_cast<int>(data->enforced_inequality_rows.size());
    if (options.nonlinear_inequality_dual_start.size() == data->layout.nineq) {
      selected_inequality_dual_seed = select_inequality_rows(
          options.nonlinear_inequality_dual_start,
          data->enforced_inequality_rows);
    } else if (options.nonlinear_inequality_dual_start.size() == enforced_count) {
      selected_inequality_dual_seed =
          options.nonlinear_inequality_dual_start;
    }
    if (options.nonlinear_slack_start.size() == data->layout.nineq) {
      selected_slack_seed = select_inequality_rows(
          options.nonlinear_slack_start, data->enforced_inequality_rows);
    } else if (options.nonlinear_slack_start.size() == enforced_count) {
      selected_slack_seed = options.nonlinear_slack_start;
    }
    const Eigen::VectorXd* inequality_dual_seed =
        selected_inequality_dual_seed.size() == enforced_count
            ? &selected_inequality_dual_seed : nullptr;
    const Eigen::VectorXd* slack_seed =
        selected_slack_seed.size() == enforced_count
            ? &selected_slack_seed : nullptr;
    log_stage("dual initialization: start");
    int dual_factorizations = 0;
    int remaining_factorizations = std::max(
        0, options.phase_one_max_factorizations -
               phase_one_result.factorizations);
    if (options.phase_one_time_limit_ms > 0.0 &&
        phase_one_elapsed_ms() >= options.phase_one_time_limit_ms) {
      remaining_factorizations = 0;
      phase_one_result.budget_exhausted = true;
      if (phase_one_result.converged) {
        phase_one_result.termination = "time-budget-after-primal";
      }
    }
    initialize_primal_dual_start(nlp, equality_dual_seed,
                                 inequality_dual_seed, slack_seed,
                                 &initial_dual_residual,
                                 &phase_one_dual_fit_residual, ipm_options,
                                 remaining_factorizations,
                                 &dual_factorizations);
    phase_one_result.factorizations += dual_factorizations;
    if (dual_factorizations > 0 &&
        phase_one_result.linear_solver == "unselected") {
      phase_one_result.linear_solver = "sparse-basis-lu";
    }
    phase_one_result.runtime_ms = phase_one_elapsed_ms();
    phase_one_constraint_violation = phase_one_violation(nlp, nlp.x0);
    // docs/OptimalPowerFlow/chapters/three_phase.tex,
    // eq. (phase-one-dual-fit): certify the state/basic multiplier fit, not
    // full stationarity, which retains Phase II's reduced-gradient signal.
    const double dual_quality_tolerance =
        std::max(1e-6, 10.0 * options.tolerance);
    phase_one_dual_initialized =
        std::isfinite(phase_one_dual_fit_residual) &&
        phase_one_dual_fit_residual <= dual_quality_tolerance &&
        ipm_options.equality_dual_start.size() == data->layout.neq &&
        ipm_options.equality_dual_start.allFinite() &&
        ipm_options.inequality_dual_start.size() > 0 &&
        ipm_options.inequality_dual_start.size() ==
            ipm_options.slack_start.size() &&
        ipm_options.inequality_dual_start.allFinite() &&
        ipm_options.slack_start.allFinite() &&
        (ipm_options.inequality_dual_start.array() > 0.0).all() &&
        (ipm_options.slack_start.array() > 0.0).all();
    if (!phase_one_dual_initialized &&
        phase_one_result.factorizations >=
            std::max(0, options.phase_one_max_factorizations)) {
      phase_one_result.budget_exhausted = true;
      if (phase_one_result.converged) {
        phase_one_result.termination = "factorization-budget-after-primal";
      }
    }
    log_stage("Phase I dual initialization: done (d=" +
              std::to_string(initial_dual_residual) + ")");
    engine::NativeIPMAdapter native(ipm_options);
    log_stage("Phase II NativeIPM: start");
    std::tie(solved, detail) = native.solve_nlp_detail(nlp);
    if (detail.central_warm_start_accepted) {
      phase_one_result.termination = "central-corridor-certified";
    }
    log_stage("Phase II NativeIPM: done (status=" + solved.stats.status + ")");
  } else {
    Eigen::VectorXd selected_inequality_dual;
    const int enforced_count =
        static_cast<int>(data->enforced_inequality_rows.size());
    if (options.nonlinear_inequality_dual_start.size() == data->layout.nineq) {
      selected_inequality_dual = select_inequality_rows(
          options.nonlinear_inequality_dual_start,
          data->enforced_inequality_rows);
    } else if (options.nonlinear_inequality_dual_start.size() == enforced_count) {
      selected_inequality_dual = options.nonlinear_inequality_dual_start;
    }
    const bool complete_multiplier_start =
        options.equality_dual_start.size() == data->layout.neq &&
        selected_inequality_dual.size() == enforced_count &&
        options.variable_lower_bound_dual_start.size() == data->layout.nvar &&
        options.variable_upper_bound_dual_start.size() == data->layout.nvar &&
        options.equality_dual_start.allFinite() &&
        selected_inequality_dual.allFinite() &&
        options.variable_lower_bound_dual_start.allFinite() &&
        options.variable_upper_bound_dual_start.allFinite() &&
        (selected_inequality_dual.array() >= 0.0).all() &&
        (options.variable_lower_bound_dual_start.array() >= 0.0).all() &&
        (options.variable_upper_bound_dual_start.array() >= 0.0).all();
    if (complete_multiplier_start) {
      // MIPSolvers uses [inequalities | equalities]; its Ipopt TNLP bridge
      // performs the row-order conversion and passes z_L/z_U. Ipopt rebuilds
      // its internal inequality slacks. Waechter--Biegler (2006), Sec. 3.1.
      nlp.constraint_dual_start.resize(enforced_count + data->layout.neq);
      nlp.constraint_dual_start << selected_inequality_dual,
          options.equality_dual_start;
      nlp.box_dual_lb_start = options.variable_lower_bound_dual_start;
      nlp.box_dual_ub_start = options.variable_upper_bound_dual_start;
      nlp.solver_options.primal_dual_warm_start = true;
      nlp.solver_options.warm_start_push = 1e-8;
    }
    engine::IpoptAdapter ipopt;
    solved = ipopt.solve_nlp(nlp);
  }
  const double runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

  if (options.backend == SolverBackend::NativeIPM &&
      detail.lambda_eq.size() != data->layout.neq &&
      solved.x.size() == data->layout.nvar && solved.x.allFinite()) {
    engine::NLPModel recovery_nlp = nlp;
    recovery_nlp.x0 = solved.x;
    engine::IPMOptions recovery_options;
    double recovered_dual_residual = 0.0;
    initialize_primal_dual_start(recovery_nlp, nullptr, nullptr, nullptr,
                                 &recovered_dual_residual,
                                 nullptr,
                                 recovery_options);
    if (recovered_dual_residual <= 1e-6 &&
        recovery_options.equality_dual_start.size() == data->layout.neq) {
      detail.lambda_eq = recovery_options.equality_dual_start;
      detail.mu_ineq = recovery_options.inequality_dual_start;
      detail.z_slack = recovery_options.slack_start;
    }
  }

  ThreePhaseHybridOPFResult result;
  result.converged = solved.stats.success;
  result.status = solved.stats.status;
  result.solver = solved.stats.solver_name;
  result.iterations = solved.stats.iterations;
  result.variables = data->layout.nvar;
  result.equalities = data->layout.neq;
  result.inequalities = data->layout.nineq;
  result.enforced_inequalities =
      static_cast<int>(data->enforced_inequality_rows.size());
  result.objective = solved.stats.objective;
  result.runtime_ms = runtime_ms;
  result.initial_dual_residual = initial_dual_residual;
  result.phase_one_constraint_violation = phase_one_constraint_violation;
  result.phase_one_initial_violation = phase_one_result.initial_violation;
  result.phase_one_dual_fit_residual = phase_one_dual_fit_residual;
  result.phase_one_primal_feasible =
      options.backend == SolverBackend::NativeIPM &&
      std::isfinite(phase_one_constraint_violation) &&
      phase_one_constraint_violation <= options.tolerance;
  result.phase_one_in_handoff_corridor =
      options.backend == SolverBackend::NativeIPM &&
      std::isfinite(phase_one_constraint_violation) &&
      phase_one_constraint_violation <= phase_one_tolerance;
  result.phase_one_dual_initialized = phase_one_dual_initialized;
  result.phase_one_handoff_primal_tolerance = phase_one_tolerance;
  result.phase_one_perturbed_primal_residual =
      detail.central_warm_start_requested
      ? detail.central_warm_start_primal_residual
      : std::numeric_limits<double>::infinity();
  result.phase_one_centrality = detail.central_warm_start_requested
      ? detail.central_warm_start_centrality
      : std::numeric_limits<double>::infinity();
  result.phase_one_barrier_mu = phase_one_barrier_mu;
  result.phase_one_budget_exhausted =
      phase_one_result.budget_exhausted ||
      (options.backend == SolverBackend::NativeIPM &&
       !phase_one_dual_initialized &&
       phase_one_result.factorizations >=
           std::max(0, options.phase_one_max_factorizations));
  result.phase_one_iterations = phase_one_result.iterations;
  result.phase_one_factorizations = phase_one_result.factorizations;
  result.phase_one_backtracks = phase_one_result.backtracks;
  result.phase_one_runtime_ms = phase_one_result.runtime_ms;
  result.phase_one_termination = phase_one_result.termination;
  result.phase_one_linear_solver = phase_one_result.linear_solver;
  result.phase_two_start_requested =
      detail.central_warm_start_requested;
  result.phase_two_start_accepted =
      detail.central_warm_start_accepted;
  result.phase_two_start_rejection_reason =
      detail.central_warm_start_rejection_reason;
  result.phase_two_linear_solver_backend = detail.linear_solver_backend;
  result.phase_two_initial_attempt_iterations =
      detail.initial_attempt_iterations;
  result.phase_two_initial_attempt_factorizations =
      detail.initial_attempt_factorizations;
  result.phase_two_total_factorizations = detail.numeric_factorizations;
  result.phase_two_restoration_factorizations =
      detail.restoration_factorizations;
  result.phase_two_retry_factorizations = detail.retry_factorizations;
  Eigen::VectorXd initial_g;
  Eigen::VectorXd initial_h;
  evaluate_equalities(*data, nlp.x0, initial_g);
  evaluate_inequalities(*data, nlp.x0, initial_h);
  if (initial_g.size() > 0) {
    initial_g.array() /= data->equality_scale.array();
    Eigen::Index worst = 0;
    result.initial_primal_residual = initial_g.cwiseAbs().maxCoeff(&worst);
    result.initial_worst_equality = static_cast<int>(worst);
  }
  if (initial_h.size() > 0) {
    Eigen::Index worst = 0;
    const double violation = initial_h.maxCoeff(&worst);
    if (violation > result.initial_primal_residual) {
      result.initial_primal_residual = violation;
      result.initial_worst_inequality = static_cast<int>(worst);
    }
  }
  result.primal_residual = solved.stats.unscaled_primal_feas > 0.0
      ? solved.stats.unscaled_primal_feas : solved.stats.primal_feas;
  result.dual_residual = solved.stats.unscaled_dual_feas > 0.0
      ? solved.stats.unscaled_dual_feas : solved.stats.dual_feas;
  result.complementarity = solved.stats.unscaled_complementarity > 0.0
      ? solved.stats.unscaled_complementarity
      : solved.stats.complementarity;
  result.primal = solved.x;
  result.primal_dual_warm_start_used = solved.stats.warm_start_used;
  result.variable_lower_bound_dual = solved.box_dual_lb;
  result.variable_upper_bound_dual = solved.box_dual_ub;
  if (options.backend == SolverBackend::Ipopt &&
      solved.constraint_duals.size() ==
          static_cast<int>(data->enforced_inequality_rows.size()) +
              data->layout.neq) {
    const int enforced_count =
        static_cast<int>(data->enforced_inequality_rows.size());
    detail.mu_ineq = solved.constraint_duals.head(enforced_count);
    detail.lambda_eq = solved.constraint_duals.tail(data->layout.neq);
  }
  result.equality_dual = detail.lambda_eq;
  const int enforced_count =
      static_cast<int>(data->enforced_inequality_rows.size());
  const int bound_multiplier_count = std::max(
      0, static_cast<int>(detail.mu_ineq.size()) - enforced_count);
  result.inequality_dual = Eigen::VectorXd::Zero(
      data->layout.nineq + bound_multiplier_count);
  if (detail.mu_ineq.size() >= enforced_count) {
    result.inequality_dual.head(data->layout.nineq) =
        expand_inequality_multipliers(
            detail.mu_ineq.head(enforced_count),
            data->enforced_inequality_rows, data->layout.nineq);
    if (bound_multiplier_count > 0) {
      result.inequality_dual.tail(bound_multiplier_count) =
          detail.mu_ineq.tail(bound_multiplier_count);
    }
  }
  result.inequality_slack = Eigen::VectorXd::Zero(
      data->layout.nineq + bound_multiplier_count);
  if (solved.x.size() == data->layout.nvar && solved.x.allFinite()) {
    Eigen::VectorXd full_h;
    evaluate_inequalities(*data, solved.x, full_h);
    result.inequality_slack.head(data->layout.nineq) =
        (-full_h.array()).max(1e-12).matrix();
  }
  if (detail.z_slack.size() >= enforced_count && bound_multiplier_count > 0) {
    result.inequality_slack.tail(bound_multiplier_count) =
        detail.z_slack.tail(bound_multiplier_count);
  }
  result.reduction = data->reduction;
  result.enforced_inequality_rows = data->enforced_inequality_rows;
  result.eliminated_phase_nodes =
      static_cast<int>(data->reduction.recovery_steps.size());

  if (options.verify_derivatives) {
    const Eigen::VectorXd x0 = nlp.x0;
    Eigen::SparseMatrix<double> jg;
    Eigen::SparseMatrix<double> jh;
    equality_jacobian(*data, x0, jg);
    inequality_jacobian(*data, x0, jh);
    const double step = 1e-5;
    for (int col = 0; col < data->layout.nvar; ++col) {
      Eigen::VectorXd xp = x0;
      Eigen::VectorXd xm = x0;
      xp[col] += step;
      xm[col] -= step;
      Eigen::VectorXd gp, gm, hp, hm;
      evaluate_equalities(*data, xp, gp);
      evaluate_equalities(*data, xm, gm);
      evaluate_inequalities(*data, xp, hp);
      evaluate_inequalities(*data, xm, hm);
      const Eigen::VectorXd fdg = (gp - gm) / (2.0 * step);
      const Eigen::VectorXd fdh = (hp - hm) / (2.0 * step);
      for (int row = 0; row < fdg.size(); ++row) {
        result.max_equality_jacobian_error = std::max(
            result.max_equality_jacobian_error,
            std::abs(fdg[row] - jg.coeff(row, col)) /
                std::max(1.0, std::abs(fdg[row])));
      }
      for (int row = 0; row < fdh.size(); ++row) {
        result.max_inequality_jacobian_error = std::max(
            result.max_inequality_jacobian_error,
            std::abs(fdh[row] - jh.coeff(row, col)) /
                std::max(1.0, std::abs(fdh[row])));
      }
    }

    Eigen::VectorXd lambda(data->layout.neq);
    Eigen::VectorXd nu(data->layout.nineq);
    for (int i = 0; i < lambda.size(); ++i) {
      lambda[i] = 0.01 * static_cast<double>((i % 7) - 3);
    }
    for (int i = 0; i < nu.size(); ++i) {
      nu[i] = 0.01 * static_cast<double>((i % 5) + 1);
    }
    Eigen::SparseMatrix<double> analytic_hessian;
    lagrangian_hessian(*data, x0, lambda, &nu, analytic_hessian, false);
    const auto lagrangian_gradient = [&](const Eigen::VectorXd& point) {
      Eigen::VectorXd gradient;
      objective_gradient(*data, point, gradient);
      Eigen::SparseMatrix<double> eq_jacobian;
      Eigen::SparseMatrix<double> ineq_jacobian;
      equality_jacobian(*data, point, eq_jacobian);
      inequality_jacobian(*data, point, ineq_jacobian);
      Eigen::VectorXd result = gradient + eq_jacobian.transpose() * lambda +
                               ineq_jacobian.transpose() * nu;
      return result;
    };
    for (int col = 0; col < data->layout.nvar; ++col) {
      Eigen::VectorXd xp = x0;
      Eigen::VectorXd xm = x0;
      xp[col] += step;
      xm[col] -= step;
      const Eigen::VectorXd finite_difference =
          (lagrangian_gradient(xp) - lagrangian_gradient(xm)) / (2.0 * step);
      for (int row = 0; row < finite_difference.size(); ++row) {
        result.max_lagrangian_hessian_error = std::max(
            result.max_lagrangian_hessian_error,
            std::abs(finite_difference[row] - analytic_hessian.coeff(row, col)) /
                std::max(1.0, std::abs(finite_difference[row])));
      }
    }
  }

  Eigen::SparseMatrix<double> jac;
  equality_jacobian(*data, nlp.x0, jac);
  result.equality_jacobian_nonzeros = static_cast<int>(jac.nonZeros());
  inequality_jacobian(*data, nlp.x0, jac,
                      &data->enforced_inequality_rows);
  result.inequality_jacobian_nonzeros = static_cast<int>(jac.nonZeros());
  if (solved.x.size() == data->layout.nvar && solved.x.allFinite()) {
    result.full_voltage = full_voltage(*data, solved.x);
    result.dc_voltage = solved.x.segment(data->layout.i_udc, data->layout.ndc);
    result.generator_active_power_pu.resize(
        static_cast<std::size_t>(data->layout.ng));
    result.generator_reactive_power_pu.resize(
        static_cast<std::size_t>(data->layout.ng));
    for (int gi = 0; gi < data->layout.ng; ++gi) {
      result.generator_active_power_pu[static_cast<std::size_t>(gi)] =
          solved.x[data->layout.i_pg + gi];
      result.generator_reactive_power_pu[static_cast<std::size_t>(gi)] =
          solved.x[data->layout.i_qg + gi];
    }
    result.converter_phase_power_pu.clear();
    result.converter_dc_power_pu.clear();
    result.converter_phase_power_pu.reserve(data->converters.size());
    result.converter_dc_power_pu.reserve(data->converters.size());
    for (int ci = 0; ci < static_cast<int>(data->converters.size()); ++ci) {
      const auto& converter = data->converters[static_cast<std::size_t>(ci)];
      std::vector<Complex> phase_power;
      phase_power.reserve(converter.phase_var_positions.size());
      for (int pos : converter.phase_var_positions) {
        phase_power.emplace_back(solved.x[data->layout.i_pac + pos],
                                 solved.x[data->layout.i_qac + pos]);
      }
      result.converter_phase_power_pu.push_back(std::move(phase_power));
      result.converter_dc_power_pu.push_back(solved.x[data->layout.i_pdc + ci]);
    }
    Eigen::VectorXd physical_g;
    evaluate_equalities(*data, solved.x, physical_g);
    physical_g.array() /= data->equality_scale.array();
    Eigen::VectorXd h;
    evaluate_inequalities(*data, solved.x, h);
    std::vector<bool> enforced_mask(
        static_cast<std::size_t>(data->layout.nineq), false);
    for (int row : data->enforced_inequality_rows) {
      enforced_mask[static_cast<std::size_t>(row)] = true;
    }
    for (int row = 0; row < h.size(); ++row) {
      if (!enforced_mask[static_cast<std::size_t>(row)]) {
        result.max_omitted_inequality =
            std::max(result.max_omitted_inequality, h[row]);
      }
    }
    const double equality_residual = physical_g.size() > 0
        ? physical_g.cwiseAbs().maxCoeff() : 0.0;
    const double inequality_residual = h.size() > 0
        ? std::max(0.0, h.maxCoeff()) : 0.0;
    result.primal_residual = std::max(equality_residual, inequality_residual);
    const int full_n = static_cast<int>(problem.y_ac.rows());
    for (int i = 0; i < full_n; ++i) {
      result.max_voltage_violation = std::max(
          result.max_voltage_violation, std::max(0.0, std::max(h[i], h[full_n + i])));
    }
    result.max_vuf = max_vuf(problem, result.full_voltage);
    result.max_converter_current_vuf =
        max_converter_current_vuf(*data, result.primal, result.full_voltage);
    result.max_converter_current_loading =
        max_converter_current_loading(*data, result.primal, result.full_voltage);
    result.converter_dynamic_equilibria = recover_dynamic_equilibria(
        *data, result.primal, result.full_voltage,
        &result.max_dynamic_equilibrium_residual);
    const int converter_row = 2 * full_n +
        static_cast<int>(problem.three_phase_bus_nodes.size());
    for (int ci = 0; ci < data->layout.nc + data->layout.ncp; ++ci) {
      result.max_converter_violation = std::max(
          result.max_converter_violation,
          std::max(0.0, h[converter_row + ci]));
    }
    const bool audited_kkt =
        result.primal_residual <= 1e-6 &&
        result.dual_residual <= 1e-6 &&
        result.complementarity <= 1e-6 &&
        result.max_voltage_violation <= 1e-6 &&
        result.max_converter_violation <= 1e-6;
    result.converged = solved.stats.success && audited_kkt;
  }
  return result;
}

ThreePhaseHybridOPFResult solve_three_phase_hybrid_opf(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFOptions& options) {
  return solve_three_phase_hybrid_opf_impl(problem, options);
}

powerflow::ThreePhaseHybridPFCase make_three_phase_hybrid_pf_case(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFResult& operating_point) {
  if (!operating_point.converged ||
      operating_point.full_voltage.size() != problem.y_ac.rows() ||
      operating_point.converter_phase_power_pu.size() !=
          problem.converters.size()) {
    throw std::invalid_argument(
        "make_three_phase_hybrid_pf_case requires a converged matching OPF point");
  }
  powerflow::ThreePhaseHybridPFCase pf;
  pf.name = problem.name + "_opf_dispatch_pf";
  pf.base_mva = problem.base_mva;
  pf.y_ac = problem.y_ac;
  pf.i_ac_fixed = problem.i_ac_fixed;
  pf.p_load_pu = problem.p_load_pu;
  pf.q_load_pu = problem.q_load_pu;
  pf.voltage_start = problem.voltage_start;
  pf.ac_phase_index = problem.ac_phase_index;
  pf.reference_nodes = problem.reference_nodes;
  pf.reference_voltage = problem.reference_voltage;
  pf.g_dc = problem.g_dc;
  pf.p_dc_load_pu = problem.p_dc_load_pu;
  pf.v_dc_start = problem.v_dc_start;

  const auto is_dc_reference = [&](int terminal) {
    return std::find(problem.dc_reference_terminals.begin(),
                     problem.dc_reference_terminals.end(), terminal) !=
           problem.dc_reference_terminals.end();
  };
  const auto dc_setpoint = [&](int terminal) {
    for (int k = 0; k < static_cast<int>(problem.dc_reference_terminals.size()); ++k) {
      if (problem.dc_reference_terminals[static_cast<std::size_t>(k)] == terminal) {
        return problem.dc_reference_voltage_pu[k];
      }
    }
    return pf.v_dc_start[terminal];
  };
  pf.converters.reserve(problem.converters.size());
  for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
    const auto& source = problem.converters[static_cast<std::size_t>(ci)];
    const auto& powers =
        operating_point.converter_phase_power_pu[static_cast<std::size_t>(ci)];
    Complex total{0.0, 0.0};
    for (const Complex value : powers) total += value;
    powerflow::ThreePhaseHybridPFConverter converter;
    converter.phase_nodes = source.phase_nodes;
    converter.dc_terminal = source.dc_terminal;
    converter.efficiency = source.efficiency;
    converter.p_set_pu = std::real(total);
    converter.q_set_pu = std::imag(total);
    converter.v_dc_set_pu = dc_setpoint(source.dc_terminal);
    converter.virtual_r_pu = source.virtual_r_pu;
    converter.virtual_x_pu = source.virtual_x_pu;
    const bool controls_vdc = is_dc_reference(source.dc_terminal);
    if (source.control_mode == PhaseVSCControlMode::GridFollowingPLL) {
      converter.control_mode = controls_vdc
          ? powerflow::ThreePhaseHybridPFControlMode::GridFollowingVdcQ
          : powerflow::ThreePhaseHybridPFControlMode::GridFollowingPQ;
    } else if (source.control_mode == PhaseVSCControlMode::GridFormingDroop) {
      converter.control_mode = controls_vdc
          ? powerflow::ThreePhaseHybridPFControlMode::GridFormingVdc
          : powerflow::ThreePhaseHybridPFControlMode::GridFormingVoltage;
      if (ci < static_cast<int>(operating_point.converter_dynamic_equilibria.size())) {
        converter.internal_voltage_positive =
            operating_point.converter_dynamic_equilibria[static_cast<std::size_t>(ci)]
                .internal_voltage_positive;
      }
    } else {
      converter.control_mode = controls_vdc
          ? powerflow::ThreePhaseHybridPFControlMode::EqualPhaseVdcQ
          : powerflow::ThreePhaseHybridPFControlMode::EqualPhasePQ;
    }
    pf.converters.push_back(std::move(converter));
  }
  return pf;
}

std::vector<ThreePhaseHybridOPFResult> solve_three_phase_hybrid_opf_sequence(
    const std::vector<ThreePhaseHybridOPFCase>& problems,
    const ThreePhaseHybridOPFOptions& options) {
  std::vector<ThreePhaseHybridOPFResult> results;
  if (problems.empty()) return results;

  ThreePhaseHybridOPFCase preparation_problem = problems.front();
  for (const auto& problem : problems) {
    validate_case(problem);
    if (problem.y_ac.rows() != problems.front().y_ac.rows() ||
        problem.y_ac.nonZeros() != problems.front().y_ac.nonZeros() ||
        problem.g_dc.rows() != problems.front().g_dc.rows() ||
        problem.g_dc.nonZeros() != problems.front().g_dc.nonZeros() ||
        problem.generators.size() != problems.front().generators.size() ||
        problem.converters.size() != problems.front().converters.size()) {
      throw std::invalid_argument(
          "hybrid OPF sequence requires a fixed network and device topology");
    }
    for (int node = 0; node < problem.p_load_pu.size(); ++node) {
      if (std::abs(problem.p_load_pu[node]) >
          std::abs(preparation_problem.p_load_pu[node])) {
        preparation_problem.p_load_pu[node] = problem.p_load_pu[node];
      }
      if (std::abs(problem.q_load_pu[node]) >
          std::abs(preparation_problem.q_load_pu[node])) {
        preparation_problem.q_load_pu[node] = problem.q_load_pu[node];
      }
      if (std::abs(problem.i_ac_fixed[node]) >
          std::abs(preparation_problem.i_ac_fixed[node])) {
        preparation_problem.i_ac_fixed[node] = problem.i_ac_fixed[node];
      }
    }
  }

  const auto preparation_start = std::chrono::steady_clock::now();
  ThreePhaseHybridOPFOptions preparation_options = options;
  preparation_options.use_constraint_oracle = false;
  preparation_options.enforced_inequality_rows.clear();
  const auto prepared =
      build_model_data(preparation_problem, preparation_options);
  const double preparation_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - preparation_start).count();

  results.reserve(problems.size());
  for (std::size_t index = 0; index < problems.size(); ++index) {
    const auto& problem = problems[index];
    auto step_data = std::make_shared<ModelData>(*prepared);
    step_data->source = &problem;
    for (int pos = 0;
         pos < static_cast<int>(step_data->reduction.retained.size()); ++pos) {
      step_data->fixed_current[pos] = problem.i_ac_fixed[
          step_data->reduction.retained[static_cast<std::size_t>(pos)]];
    }

    ThreePhaseHybridOPFOptions step_options = options;
    if (!results.empty()) {
      const auto& previous = results.back();
      step_options.warm_start_with_ipopt = false;
      step_options.oracle_seed_rows = previous.enforced_inequality_rows;
      step_options.primal_start = previous.primal;
      step_options.equality_dual_start = previous.equality_dual;
      if (previous.inequality_dual.size() >= previous.inequalities) {
        step_options.nonlinear_inequality_dual_start =
            previous.inequality_dual.head(previous.inequalities);
      }
      if (previous.inequality_slack.size() >= previous.inequalities) {
        step_options.nonlinear_slack_start =
            previous.inequality_slack.head(previous.inequalities);
      }
      step_options.variable_lower_bound_dual_start =
          previous.variable_lower_bound_dual;
      step_options.variable_upper_bound_dual_start =
          previous.variable_upper_bound_dual;
    }
    results.push_back(solve_three_phase_hybrid_opf_impl(
        problem, step_options, step_data));
  }
  results.front().runtime_ms += preparation_ms;
  return results;
}

std::vector<ThreePhaseHybridOPFResult> solve_three_phase_hybrid_opf_branches(
    const ThreePhaseHybridOPFCase& base_problem,
    const ThreePhaseHybridOPFResult& base_result,
    const std::vector<ThreePhaseHybridOPFCase>& perturbed_problems,
    const ThreePhaseHybridOPFOptions& options,
    ParametricWarmStartMode warm_start_mode) {
  if (!base_result.converged) {
    throw std::invalid_argument(
        "parametric OPF branches require a converged base OPF solution");
  }
  ThreePhaseHybridOPFOptions preparation_options = options;
  preparation_options.use_constraint_oracle = false;
  preparation_options.enforced_inequality_rows.clear();
  const auto prepared = build_model_data(base_problem, preparation_options);

  std::vector<ThreePhaseHybridOPFResult> results;
  results.reserve(perturbed_problems.size());
  for (const auto& problem : perturbed_problems) {
    validate_case(problem);
    if (problem.y_ac.rows() != base_problem.y_ac.rows() ||
        problem.y_ac.nonZeros() != base_problem.y_ac.nonZeros() ||
        problem.g_dc.rows() != base_problem.g_dc.rows() ||
        problem.g_dc.nonZeros() != base_problem.g_dc.nonZeros() ||
        problem.generators.size() != base_problem.generators.size() ||
        problem.converters.size() != base_problem.converters.size()) {
      throw std::invalid_argument(
          "hybrid OPF branches require a fixed network and device topology");
    }
    auto step_data = std::make_shared<ModelData>(*prepared);
    step_data->source = &problem;
    for (int pos = 0;
         pos < static_cast<int>(step_data->reduction.retained.size()); ++pos) {
      step_data->fixed_current[pos] = problem.i_ac_fixed[
          step_data->reduction.retained[static_cast<std::size_t>(pos)]];
    }
    ThreePhaseHybridOPFOptions step_options = options;
    step_options.warm_start_with_ipopt = false;
    step_options.oracle_seed_rows = base_result.enforced_inequality_rows;
    step_options.primal_start = base_result.primal;
    if (warm_start_mode == ParametricWarmStartMode::PrimalDual) {
      step_options.equality_dual_start = base_result.equality_dual;
      if (base_result.inequality_dual.size() >= base_result.inequalities) {
        step_options.nonlinear_inequality_dual_start =
            base_result.inequality_dual.head(base_result.inequalities);
      }
      if (base_result.inequality_slack.size() >= base_result.inequalities) {
        step_options.nonlinear_slack_start =
            base_result.inequality_slack.head(base_result.inequalities);
      }
      step_options.variable_lower_bound_dual_start =
          base_result.variable_lower_bound_dual;
      step_options.variable_upper_bound_dual_start =
          base_result.variable_upper_bound_dual;
    } else {
      // A primal-only adapter must not be credited with a newly factorized
      // Phase-I dual fit. Leave the supplied primal point in nlp.x0 and let
      // the backend initialize its ordinary dual/slack state.
      step_options.phase_one_max_factorizations = 0;
    }
    results.push_back(solve_three_phase_hybrid_opf_impl(
        problem, step_options, step_data));
  }
  return results;
}

}  // namespace hacdcpf::opf::phase_hybrid
