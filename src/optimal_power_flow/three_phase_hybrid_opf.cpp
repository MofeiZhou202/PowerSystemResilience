#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include <Eigen/Sparse>

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
  int i_e{0};
  int i_f{0};
  int i_pg{0};
  int i_qg{0};
  int i_udc{0};
  int i_pac{0};
  int i_qac{0};
  int i_pdc{0};
  int nvar{0};
  int neq{0};
  int nineq{0};
};

struct ConverterMap {
  std::vector<int> model_phase_nodes;
  std::vector<int> phase_var_positions;
  int dc_terminal{-1};
  double efficiency{0.98};
  double s_max_pu{0.0};
  bool fixed_unity_power_factor{false};
};

struct ModelData {
  const ThreePhaseHybridOPFCase* source{nullptr};
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
  }
}

Eigen::SparseMatrix<Complex> identity_recovery(int n) {
  Eigen::SparseMatrix<Complex> t(n, n);
  t.setIdentity();
  return t;
}

std::shared_ptr<ModelData> build_model_data(
    const ThreePhaseHybridOPFCase& c,
    const ThreePhaseHybridOPFOptions& options) {
  validate_case(c);
  auto data = std::make_shared<ModelData>();
  data->source = &c;
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

  for (const auto& generator : c.generators) {
    const int node = data->full_to_model[static_cast<std::size_t>(
        generator.phase_node)];
    if (node < 0) throw std::runtime_error("generator node was eliminated");
    data->generator_nodes.push_back(node);
  }

  int phase_var = 0;
  for (int ci = 0; ci < static_cast<int>(c.converters.size()); ++ci) {
    const auto& converter = c.converters[static_cast<std::size_t>(ci)];
    ConverterMap map;
    map.dc_terminal = converter.dc_terminal;
    map.efficiency = std::clamp(converter.efficiency, 0.01, 1.0);
    map.s_max_pu = converter.s_max_pu;
    map.fixed_unity_power_factor = converter.fixed_unity_power_factor;
    for (int full_node : converter.phase_nodes) {
      const int node = data->full_to_model[static_cast<std::size_t>(full_node)];
      if (node < 0) throw std::runtime_error("converter node was eliminated");
      map.model_phase_nodes.push_back(node);
      map.phase_var_positions.push_back(phase_var++);
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
  l.i_e = 0;
  l.i_f = l.i_e + l.nv;
  l.i_pg = l.i_f + l.nv;
  l.i_qg = l.i_pg + l.ng;
  l.i_udc = l.i_qg + l.ng;
  l.i_pac = l.i_udc + l.ndc;
  l.i_qac = l.i_pac + l.ncp;
  l.i_pdc = l.i_qac + l.ncp;
  l.nvar = l.i_pdc + l.nc;
  l.neq = 2 * l.nv + l.ndc + 2 * l.nc + 2 * (l.ncp - l.nc) +
          2 * static_cast<int>(c.reference_nodes.size());
  l.nineq = 2 * full_n + static_cast<int>(c.three_phase_bus_nodes.size()) + l.nc;
  data->equality_scale = Eigen::VectorXd::Ones(l.neq);
  Eigen::VectorXd ac_row_norm = Eigen::VectorXd::Zero(l.nv);
  for (int col = 0; col < data->y.outerSize(); ++col) {
    for (graph::SparseComplexMatrix::InnerIterator it(data->y, col); it; ++it) {
      ac_row_norm[it.row()] += std::abs(it.value());
    }
  }
  for (int i = 0; i < l.nv; ++i) {
    const double scale = 1.0 / std::max(1.0, ac_row_norm[i]);
    data->equality_scale[i] = scale;
    data->equality_scale[l.nv + i] = scale;
  }
  Eigen::VectorXd dc_row_norm = Eigen::VectorXd::Zero(l.ndc);
  for (int col = 0; col < c.g_dc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(c.g_dc, col); it; ++it) {
      dc_row_norm[it.row()] += std::abs(it.value());
    }
  }
  for (int i = 0; i < l.ndc; ++i) {
    data->equality_scale[2 * l.nv + i] =
        1.0 / std::max(1.0, dc_row_norm[i]);
  }
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
  for (const auto& converter : d.converters) {
    g[row++] = x[l.i_udc + converter.dc_terminal] -
               c.v_dc_start[converter.dc_terminal];
  }
  for (const auto& converter : d.converters) {
    const int first = converter.phase_var_positions.front();
    for (int local = 1;
         local < static_cast<int>(converter.phase_var_positions.size()); ++local) {
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      g[row++] = x[l.i_pac + pos] - x[l.i_pac + first];
    }
    for (int local = 1;
         local < static_cast<int>(converter.phase_var_positions.size()); ++local) {
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      g[row++] = x[l.i_qac + pos] - x[l.i_qac + first];
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
  for (const auto& converter : d.converters) {
    trips.emplace_back(row++, l.i_udc + converter.dc_terminal, 1.0);
  }
  for (const auto& converter : d.converters) {
    const int first = converter.phase_var_positions.front();
    for (int local = 1;
         local < static_cast<int>(converter.phase_var_positions.size()); ++local) {
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      trips.emplace_back(row, l.i_pac + pos, 1.0);
      trips.emplace_back(row++, l.i_pac + first, -1.0);
    }
    for (int local = 1;
         local < static_cast<int>(converter.phase_var_positions.size()); ++local) {
      const int pos = converter.phase_var_positions[static_cast<std::size_t>(local)];
      trips.emplace_back(row, l.i_qac + pos, 1.0);
      trips.emplace_back(row++, l.i_qac + first, -1.0);
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
}

void inequality_jacobian(const ModelData& d,
                         const Eigen::VectorXd& x,
                         Eigen::SparseMatrix<double>& jac) {
  const Layout& l = d.layout;
  const auto& c = *d.source;
  const Eigen::VectorXcd v = full_voltage(d, x);
  const int full_n = static_cast<int>(v.size());
  std::vector<Triplet> trips;
  for (int i = 0; i < full_n; ++i) {
    const double e = std::real(v[i]);
    const double f = std::imag(v[i]);
    append_full_gradient(d, i, {{i, -2.0 * e}}, {{i, -2.0 * f}}, trips);
    append_full_gradient(d, full_n + i, {{i, 2.0 * e}}, {{i, 2.0 * f}}, trips);
  }
  int row = 2 * full_n;
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  const Complex c1[3] = {Complex{1.0 / 3.0, 0.0}, a / 3.0, a2 / 3.0};
  const Complex c2[3] = {Complex{1.0 / 3.0, 0.0}, a2 / 3.0, a / 3.0};
  for (const auto& nodes : c.three_phase_bus_nodes) {
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
    append_full_gradient(d, row++, de, df, trips);
  }
  for (const auto& converter : d.converters) {
    for (int pos : converter.phase_var_positions) {
      trips.emplace_back(row, l.i_pac + pos, 2.0 * x[l.i_pac + pos]);
      trips.emplace_back(row, l.i_qac + pos, 2.0 * x[l.i_qac + pos]);
    }
    ++row;
  }
  jac.resize(l.nineq, l.nvar);
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

std::unordered_map<int, Complex> complex_linear_map(
    const ModelData& d,
    const std::vector<int>& full_nodes,
    const std::vector<Complex>& coefficients) {
  std::unordered_map<int, Complex> map;
  for (int k = 0; k < static_cast<int>(full_nodes.size()); ++k) {
    const int full_node = full_nodes[static_cast<std::size_t>(k)];
    const Complex factor = coefficients[static_cast<std::size_t>(k)];
    for (const auto& [model_node, recovery] :
         d.recovery_rows[static_cast<std::size_t>(full_node)]) {
      map[d.layout.i_e + model_node] += factor * recovery;
      map[d.layout.i_f + model_node] +=
          Complex{0.0, 1.0} * factor * recovery;
    }
  }
  return map;
}

void add_complex_norm_hessian(
    std::vector<Triplet>& trips,
    const std::unordered_map<int, Complex>& map,
    double multiplier) {
  for (auto first = map.begin(); first != map.end(); ++first) {
    auto second = first;
    for (; second != map.end(); ++second) {
      const double value = multiplier *
          2.0 * std::real(std::conj(first->second) * second->second);
      if (first->first == second->first) {
        trips.emplace_back(first->first, second->first, value);
      } else {
        trips.emplace_back(first->first, second->first, value);
        trips.emplace_back(second->first, first->first, value);
      }
    }
  }
}

void lagrangian_hessian(const ModelData& d,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXd& lambda,
                        const Eigen::VectorXd* nu,
                        Eigen::SparseMatrix<double>& hessian) {
  (void)x;
  const Layout& l = d.layout;
  std::vector<Triplet> trips;
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
  }

  if (nu != nullptr && nu->size() >= l.nineq) {
    const int full_n = d.reduction.original_size;
    for (int node = 0; node < full_n; ++node) {
      const double multiplier = -(*nu)[node] + (*nu)[full_n + node];
      const auto map = complex_linear_map(
          d, std::vector<int>{node}, std::vector<Complex>{Complex{1.0, 0.0}});
      add_complex_norm_hessian(trips, map, multiplier);
    }
    int row = 2 * full_n;
    const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
    const Complex a2 = a * a;
    const std::vector<Complex> positive{
        Complex{1.0 / 3.0, 0.0}, a / 3.0, a2 / 3.0};
    const std::vector<Complex> negative{
        Complex{1.0 / 3.0, 0.0}, a2 / 3.0, a / 3.0};
    for (const auto& nodes : d.source->three_phase_bus_nodes) {
      const double multiplier = (*nu)[row++];
      add_complex_norm_hessian(
          trips, complex_linear_map(d, nodes, negative), multiplier);
      add_complex_norm_hessian(
          trips, complex_linear_map(d, nodes, positive),
          -multiplier * d.source->vuf_max * d.source->vuf_max);
    }
    for (const auto& converter : d.converters) {
      const double multiplier = 2.0 * (*nu)[row++];
      for (int pos : converter.phase_var_positions) {
        trips.emplace_back(l.i_pac + pos, l.i_pac + pos, multiplier);
        trips.emplace_back(l.i_qac + pos, l.i_qac + pos, multiplier);
      }
    }
  }
  hessian.resize(l.nvar, l.nvar);
  hessian.setFromTriplets(trips.begin(), trips.end());
  hessian.makeCompressed();
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
  }

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

engine::NLPModel build_nlp(const std::shared_ptr<ModelData>& d) {
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
  for (int ci = 0; ci < l.nc; ++ci) {
    const double cap = d->converters[static_cast<std::size_t>(ci)].s_max_pu;
    nlp.vars[static_cast<std::size_t>(l.i_pdc + ci)].lb = -cap;
    nlp.vars[static_cast<std::size_t>(l.i_pdc + ci)].ub = cap;
  }
  nlp.x0 = build_initial_point(*d);
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
        lagrangian_hessian(*d, x, lambda, nu, h);
      };
  nlp.g = [d](const Eigen::VectorXd& x, Eigen::VectorXd& g) {
    evaluate_equalities(*d, x, g);
  };
  nlp.jac_g = [d](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& j) {
    equality_jacobian(*d, x, j);
  };
  nlp.h = [d](const Eigen::VectorXd& x, Eigen::VectorXd& h) {
    evaluate_inequalities(*d, x, h);
  };
  nlp.jac_h = [d](const Eigen::VectorXd& x, Eigen::SparseMatrix<double>& j) {
    inequality_jacobian(*d, x, j);
  };
  return nlp;
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

}  // namespace

ThreePhaseHybridOPFResult solve_three_phase_hybrid_opf(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFOptions& options) {
  const auto data = build_model_data(problem, options);
  engine::NLPModel nlp = build_nlp(data);
  if (options.primal_start.size() == data->layout.nvar &&
      options.primal_start.allFinite()) {
    nlp.x0 = options.primal_start;
  }
  nlp.solver_options.max_iterations = options.max_iterations;
  nlp.solver_options.tolerance = options.tolerance;
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
  const auto start = std::chrono::steady_clock::now();
  if (options.backend == SolverBackend::NativeIPM) {
    if (options.warm_start_with_ipopt) {
      engine::IpoptAdapter ipopt;
      const engine::SolveResult warm = ipopt.solve_nlp(nlp);
      if (warm.x.size() == data->layout.nvar && warm.x.allFinite()) nlp.x0 = warm.x;
    }
    engine::IPMOptions ipm_options;
    ipm_options.max_iter = options.max_iterations;
    ipm_options.tol_primal = options.tolerance;
    ipm_options.tol_dual = options.tolerance;
    ipm_options.tol_complementarity = options.tolerance;
    ipm_options.tol_accept = std::max(options.tolerance, 1e-5);
    ipm_options.globalization = engine::Globalization::Filter;
    ipm_options.scale_problem = false;
    engine::NativeIPMAdapter native(ipm_options);
    std::tie(solved, detail) = native.solve_nlp_detail(nlp);
  } else {
    engine::IpoptAdapter ipopt;
    solved = ipopt.solve_nlp(nlp);
  }
  const double runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();

  ThreePhaseHybridOPFResult result;
  result.converged = solved.stats.success;
  result.status = solved.stats.status;
  result.solver = solved.stats.solver_name;
  result.iterations = solved.stats.iterations;
  result.variables = data->layout.nvar;
  result.equalities = data->layout.neq;
  result.inequalities = data->layout.nineq;
  result.objective = solved.stats.objective;
  result.runtime_ms = runtime_ms;
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
  result.dual_residual = solved.stats.dual_feas;
  result.complementarity = solved.stats.complementarity;
  result.primal = solved.x;
  result.equality_dual = detail.lambda_eq;
  result.inequality_dual = detail.mu_ineq;
  result.reduction = data->reduction;
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
    lagrangian_hessian(*data, x0, lambda, &nu, analytic_hessian);
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
  if (solved.x.size() == data->layout.nvar && solved.x.allFinite()) {
    result.full_voltage = full_voltage(*data, solved.x);
    Eigen::VectorXd physical_g;
    evaluate_equalities(*data, solved.x, physical_g);
    physical_g.array() /= data->equality_scale.array();
    Eigen::VectorXd h;
    evaluate_inequalities(*data, solved.x, h);
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
    const int converter_row = 2 * full_n +
        static_cast<int>(problem.three_phase_bus_nodes.size());
    for (int ci = 0; ci < data->layout.nc; ++ci) {
      result.max_converter_violation = std::max(
          result.max_converter_violation, std::max(0.0, h[converter_row + ci]));
    }
    result.converged = result.converged && result.primal_residual <= 1e-7;
  }
  return result;
}

}  // namespace hacdcpf::opf::phase_hybrid
