#include "hacdcpf/power_flow/three_phase_hybrid.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_set>

#include <Eigen/SparseLU>

namespace hacdcpf::powerflow {
namespace {

using Complex = std::complex<double>;
using Triplet = Eigen::Triplet<double>;

constexpr double kPi = 3.14159265358979323846;

struct Layout {
  int nac{0};
  int ndc{0};
  int ncontrol{0};
  int i_e{0};
  int i_f{0};
  int i_u{0};
  int i_control{0};
  int nvar{0};
};

struct ConverterMap {
  std::array<int, 3> node_by_phase{{-1, -1, -1}};
  int dc_terminal{-1};
  int control_position{-1};
};

struct Model {
  const ThreePhaseHybridPFCase* source{nullptr};
  Layout layout;
  std::vector<ConverterMap> converters;
  std::vector<char> is_reference;
  Eigen::VectorXcd reference_by_node;
};

bool is_vdc_control(ThreePhaseHybridPFControlMode mode) {
  return mode == ThreePhaseHybridPFControlMode::EqualPhaseVdcQ ||
         mode == ThreePhaseHybridPFControlMode::GridFollowingVdcQ ||
         mode == ThreePhaseHybridPFControlMode::GridFormingVdc;
}

bool is_grid_following(ThreePhaseHybridPFControlMode mode) {
  return mode == ThreePhaseHybridPFControlMode::GridFollowingPQ ||
         mode == ThreePhaseHybridPFControlMode::GridFollowingVdcQ;
}

bool is_grid_forming(ThreePhaseHybridPFControlMode mode) {
  return mode == ThreePhaseHybridPFControlMode::GridFormingVoltage ||
         mode == ThreePhaseHybridPFControlMode::GridFormingVdc;
}

Model build_model(const ThreePhaseHybridPFCase& c) {
  Model model;
  model.source = &c;
  Layout& l = model.layout;
  l.nac = static_cast<int>(c.y_ac.rows());
  l.ndc = static_cast<int>(c.g_dc.rows());
  if (l.nac <= 0 || c.y_ac.cols() != l.nac) {
    throw std::invalid_argument("three-phase hybrid PF requires square nonempty y_ac");
  }
  if (c.i_ac_fixed.size() != l.nac || c.p_load_pu.size() != l.nac ||
      c.q_load_pu.size() != l.nac || c.voltage_start.size() != l.nac ||
      static_cast<int>(c.ac_phase_index.size()) != l.nac) {
    throw std::invalid_argument("three-phase hybrid PF AC vector size mismatch");
  }
  if (c.g_dc.cols() != l.ndc || c.p_dc_load_pu.size() != l.ndc ||
      c.v_dc_start.size() != l.ndc) {
    throw std::invalid_argument("three-phase hybrid PF DC vector size mismatch");
  }
  if (c.reference_nodes.empty() ||
      c.reference_voltage.size() != static_cast<int>(c.reference_nodes.size())) {
    throw std::invalid_argument("three-phase hybrid PF requires matched AC references");
  }
  model.is_reference.assign(static_cast<std::size_t>(l.nac), 0);
  model.reference_by_node = Eigen::VectorXcd::Zero(l.nac);
  for (int k = 0; k < static_cast<int>(c.reference_nodes.size()); ++k) {
    const int node = c.reference_nodes[static_cast<std::size_t>(k)];
    if (node < 0 || node >= l.nac) {
      throw std::invalid_argument("three-phase hybrid PF reference node is invalid");
    }
    model.is_reference[static_cast<std::size_t>(node)] = 1;
    model.reference_by_node[node] = c.reference_voltage[k];
  }

  std::unordered_set<int> controlled_dc_terminals;
  for (const auto& converter : c.converters) {
    if (converter.phase_nodes.size() != 3) {
      throw std::invalid_argument(
          "sequence-aware hybrid PF converters require exactly three phases");
    }
    if (converter.dc_terminal < 0 || converter.dc_terminal >= l.ndc) {
      throw std::invalid_argument("three-phase hybrid PF converter DC terminal is invalid");
    }
    if (converter.efficiency <= 0.0 || converter.efficiency > 1.0) {
      throw std::invalid_argument("three-phase hybrid PF converter efficiency is invalid");
    }
    if (is_grid_forming(converter.control_mode) &&
        std::hypot(converter.virtual_r_pu, converter.virtual_x_pu) <= 1e-10) {
      throw std::invalid_argument("three-phase hybrid PF GFM virtual impedance is zero");
    }
    ConverterMap map;
    map.dc_terminal = converter.dc_terminal;
    for (int node : converter.phase_nodes) {
      if (node < 0 || node >= l.nac) {
        throw std::invalid_argument("three-phase hybrid PF converter phase node is invalid");
      }
      const int phase = c.ac_phase_index[static_cast<std::size_t>(node)];
      if (phase < 0 || phase > 2 || map.node_by_phase[static_cast<std::size_t>(phase)] >= 0) {
        throw std::invalid_argument("three-phase hybrid PF converter phase mapping is invalid");
      }
      map.node_by_phase[static_cast<std::size_t>(phase)] = node;
    }
    if (is_vdc_control(converter.control_mode)) {
      if (!controlled_dc_terminals.insert(converter.dc_terminal).second) {
        throw std::invalid_argument("multiple converters regulate the same DC terminal");
      }
      map.control_position = l.ncontrol++;
    }
    model.converters.push_back(map);
  }
  if (l.ndc > 0 && l.ncontrol == 0) {
    throw std::invalid_argument(
        "a DC network requires at least one converter voltage controller");
  }
  l.i_e = 0;
  l.i_f = l.nac;
  l.i_u = 2 * l.nac;
  l.i_control = l.i_u + l.ndc;
  l.nvar = l.i_control + l.ncontrol;
  return model;
}

Eigen::VectorXcd voltage(const Model& model, const Eigen::VectorXd& x) {
  const Layout& l = model.layout;
  Eigen::VectorXcd v(l.nac);
  for (int i = 0; i < l.nac; ++i) {
    v[i] = Complex{x[l.i_e + i], x[l.i_f + i]};
  }
  return v;
}

struct PowerDerivative {
  int column{-1};
  Complex value{0.0, 0.0};
};

struct ConverterEvaluation {
  std::array<Complex, 3> power{};
  std::array<std::vector<PowerDerivative>, 3> derivatives;
  Complex internal_voltage{0.0, 0.0};
  double control_value{0.0};
};

bool evaluate_converter(const Model& model,
                        int converter_index,
                        const Eigen::VectorXd& x,
                        const Eigen::VectorXcd& v,
                        double minimum_v1,
                        ConverterEvaluation& out) {
  const auto& c = *model.source;
  const Layout& l = model.layout;
  const auto& converter = c.converters[static_cast<std::size_t>(converter_index)];
  const auto& map = model.converters[static_cast<std::size_t>(converter_index)];
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  const Complex sequence_coefficient[3] = {Complex{1.0, 0.0}, a, a2};
  const Complex current_conjugate_rotation[3] = {Complex{1.0, 0.0}, a, a2};
  const Complex voltage_rotation[3] = {Complex{1.0, 0.0}, a2, a};

  double p_total = converter.p_set_pu;
  if ((converter.control_mode == ThreePhaseHybridPFControlMode::EqualPhaseVdcQ ||
       converter.control_mode == ThreePhaseHybridPFControlMode::GridFollowingVdcQ) &&
      map.control_position >= 0) {
    p_total = x[l.i_control + map.control_position];
  }
  out.control_value = map.control_position >= 0
      ? x[l.i_control + map.control_position]
      : p_total;

  if (converter.control_mode == ThreePhaseHybridPFControlMode::EqualPhasePQ ||
      converter.control_mode == ThreePhaseHybridPFControlMode::EqualPhaseVdcQ) {
    const Complex phase_power{p_total / 3.0, converter.q_set_pu / 3.0};
    for (int phase = 0; phase < 3; ++phase) {
      out.power[static_cast<std::size_t>(phase)] = phase_power;
      if (map.control_position >= 0) {
        out.derivatives[static_cast<std::size_t>(phase)].push_back(
            {l.i_control + map.control_position, Complex{1.0 / 3.0, 0.0}});
      }
    }
    return true;
  }

  if (is_grid_following(converter.control_mode)) {
    Complex v1{0.0, 0.0};
    for (int phase = 0; phase < 3; ++phase) {
      v1 += sequence_coefficient[phase] *
          v[map.node_by_phase[static_cast<std::size_t>(phase)]] / 3.0;
    }
    if (std::abs(v1) < minimum_v1) return false;
    const Complex total_power{p_total, converter.q_set_pu};
    const Complex ia_conjugate = total_power / (3.0 * v1);
    for (int phase = 0; phase < 3; ++phase) {
      const int node = map.node_by_phase[static_cast<std::size_t>(phase)];
      const Complex rotation = current_conjugate_rotation[phase];
      out.power[static_cast<std::size_t>(phase)] = v[node] * rotation * ia_conjugate;
      for (int source_phase = 0; source_phase < 3; ++source_phase) {
        const int source_node =
            map.node_by_phase[static_cast<std::size_t>(source_phase)];
        Complex derivative = -v[node] * rotation * total_power *
            sequence_coefficient[source_phase] / (9.0 * v1 * v1);
        if (source_phase == phase) derivative += rotation * ia_conjugate;
        out.derivatives[static_cast<std::size_t>(phase)].push_back(
            {l.i_e + source_node, derivative});
        out.derivatives[static_cast<std::size_t>(phase)].push_back(
            {l.i_f + source_node, Complex{0.0, 1.0} * derivative});
      }
      if (map.control_position >= 0) {
        out.derivatives[static_cast<std::size_t>(phase)].push_back(
            {l.i_control + map.control_position, v[node] * rotation / (3.0 * v1)});
      }
    }
    return true;
  }

  Complex internal = converter.internal_voltage_positive;
  if (converter.control_mode == ThreePhaseHybridPFControlMode::GridFormingVdc &&
      map.control_position >= 0) {
    internal = std::polar(std::abs(converter.internal_voltage_positive),
                          x[l.i_control + map.control_position]);
  }
  out.internal_voltage = internal;
  out.control_value = std::arg(internal);
  const Complex z{converter.virtual_r_pu, converter.virtual_x_pu};
  const Complex z_conjugate = std::conj(z);
  for (int phase = 0; phase < 3; ++phase) {
    const int node = map.node_by_phase[static_cast<std::size_t>(phase)];
    const Complex e_phase = voltage_rotation[phase] * internal;
    const Complex common = std::conj(e_phase) - std::conj(v[node]);
    out.power[static_cast<std::size_t>(phase)] = v[node] * common / z_conjugate;
    const Complex d_de = (common - v[node]) / z_conjugate;
    const Complex d_df =
        (Complex{0.0, 1.0} * common + Complex{0.0, 1.0} * v[node]) /
        z_conjugate;
    out.derivatives[static_cast<std::size_t>(phase)].push_back(
        {l.i_e + node, d_de});
    out.derivatives[static_cast<std::size_t>(phase)].push_back(
        {l.i_f + node, d_df});
    if (map.control_position >= 0) {
      const Complex d_theta =
          -Complex{0.0, 1.0} * v[node] * std::conj(e_phase) / z_conjugate;
      out.derivatives[static_cast<std::size_t>(phase)].push_back(
          {l.i_control + map.control_position, d_theta});
    }
  }
  return true;
}

void append_complex_derivative(std::vector<Triplet>& trips,
                               int p_row,
                               int q_row,
                               int column,
                               Complex derivative,
                               double scale) {
  trips.emplace_back(p_row, column, scale * std::real(derivative));
  trips.emplace_back(q_row, column, scale * std::imag(derivative));
}

bool evaluate(const Model& model,
              const Eigen::VectorXd& x,
              const ThreePhaseHybridPFOptions& options,
              Eigen::VectorXd& residual,
              Eigen::SparseMatrix<double>* jacobian,
              std::vector<ConverterEvaluation>* converter_values = nullptr) {
  const auto& c = *model.source;
  const Layout& l = model.layout;
  const Eigen::VectorXcd v = voltage(model, x);
  const Eigen::VectorXcd current = c.y_ac * v + c.i_ac_fixed;
  residual = Eigen::VectorXd::Zero(l.nvar);
  for (int i = 0; i < l.nac; ++i) {
    if (model.is_reference[static_cast<std::size_t>(i)] != 0) {
      residual[l.i_e + i] = std::real(v[i] - model.reference_by_node[i]);
      residual[l.i_f + i] = std::imag(v[i] - model.reference_by_node[i]);
    } else {
      const Complex s = v[i] * std::conj(current[i]);
      residual[l.i_e + i] = std::real(s) + c.p_load_pu[i];
      residual[l.i_f + i] = std::imag(s) + c.q_load_pu[i];
    }
  }
  const Eigen::VectorXd u = x.segment(l.i_u, l.ndc);
  if (l.ndc > 0 && (u.array() <= options.minimum_dc_voltage_pu).any()) return false;
  const Eigen::VectorXd dc_current = c.g_dc * u;
  for (int i = 0; i < l.ndc; ++i) {
    residual[l.i_u + i] = u[i] * dc_current[i] + c.p_dc_load_pu[i];
  }

  std::vector<ConverterEvaluation> evaluations(c.converters.size());
  for (int ci = 0; ci < static_cast<int>(c.converters.size()); ++ci) {
    if (!evaluate_converter(model, ci, x, v,
                            options.minimum_positive_sequence_voltage_pu,
                            evaluations[static_cast<std::size_t>(ci)])) {
      return false;
    }
    const auto& converter = c.converters[static_cast<std::size_t>(ci)];
    const auto& map = model.converters[static_cast<std::size_t>(ci)];
    double p_ac = 0.0;
    for (int phase = 0; phase < 3; ++phase) {
      const int node = map.node_by_phase[static_cast<std::size_t>(phase)];
      const Complex s = evaluations[static_cast<std::size_t>(ci)]
                            .power[static_cast<std::size_t>(phase)];
      p_ac += std::real(s);
      if (model.is_reference[static_cast<std::size_t>(node)] == 0) {
        residual[l.i_e + node] -= std::real(s);
        residual[l.i_f + node] -= std::imag(s);
      }
    }
    residual[l.i_u + converter.dc_terminal] += converter.efficiency * p_ac;
    if (map.control_position >= 0) {
      residual[l.i_control + map.control_position] =
          u[converter.dc_terminal] - converter.v_dc_set_pu;
    }
  }
  if (converter_values != nullptr) *converter_values = evaluations;
  if (jacobian == nullptr) return true;

  std::vector<Triplet> trips;
  trips.reserve(static_cast<std::size_t>(8 * c.y_ac.nonZeros() +
                                         4 * c.g_dc.nonZeros() +
                                         80 * c.converters.size()));
  for (int col = 0; col < c.y_ac.outerSize(); ++col) {
    for (Eigen::SparseMatrix<Complex>::InnerIterator it(c.y_ac, col); it; ++it) {
      const int i = it.row();
      const int j = it.col();
      if (model.is_reference[static_cast<std::size_t>(i)] != 0) continue;
      const double g = std::real(it.value());
      const double b = std::imag(it.value());
      const double ei = std::real(v[i]);
      const double fi = std::imag(v[i]);
      trips.emplace_back(l.i_e + i, l.i_e + j, ei * g + fi * b);
      trips.emplace_back(l.i_e + i, l.i_f + j, -ei * b + fi * g);
      trips.emplace_back(l.i_f + i, l.i_e + j, fi * g - ei * b);
      trips.emplace_back(l.i_f + i, l.i_f + j, -fi * b - ei * g);
    }
  }
  for (int i = 0; i < l.nac; ++i) {
    if (model.is_reference[static_cast<std::size_t>(i)] != 0) {
      trips.emplace_back(l.i_e + i, l.i_e + i, 1.0);
      trips.emplace_back(l.i_f + i, l.i_f + i, 1.0);
    } else {
      trips.emplace_back(l.i_e + i, l.i_e + i, std::real(current[i]));
      trips.emplace_back(l.i_e + i, l.i_f + i, std::imag(current[i]));
      trips.emplace_back(l.i_f + i, l.i_e + i, -std::imag(current[i]));
      trips.emplace_back(l.i_f + i, l.i_f + i, std::real(current[i]));
    }
  }
  for (int col = 0; col < c.g_dc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(c.g_dc, col); it; ++it) {
      trips.emplace_back(l.i_u + it.row(), l.i_u + it.col(),
                         u[it.row()] * it.value());
    }
  }
  for (int i = 0; i < l.ndc; ++i) {
    trips.emplace_back(l.i_u + i, l.i_u + i, dc_current[i]);
  }
  for (int ci = 0; ci < static_cast<int>(c.converters.size()); ++ci) {
    const auto& converter = c.converters[static_cast<std::size_t>(ci)];
    const auto& map = model.converters[static_cast<std::size_t>(ci)];
    const auto& evaluation = evaluations[static_cast<std::size_t>(ci)];
    for (int phase = 0; phase < 3; ++phase) {
      const int node = map.node_by_phase[static_cast<std::size_t>(phase)];
      for (const auto& derivative :
           evaluation.derivatives[static_cast<std::size_t>(phase)]) {
        if (model.is_reference[static_cast<std::size_t>(node)] == 0) {
          append_complex_derivative(trips, l.i_e + node, l.i_f + node,
                                    derivative.column, derivative.value, -1.0);
        }
        trips.emplace_back(l.i_u + converter.dc_terminal, derivative.column,
                           converter.efficiency * std::real(derivative.value));
      }
    }
    if (map.control_position >= 0) {
      trips.emplace_back(l.i_control + map.control_position,
                         l.i_u + converter.dc_terminal, 1.0);
    }
  }
  jacobian->resize(l.nvar, l.nvar);
  jacobian->setFromTriplets(trips.begin(), trips.end());
  jacobian->makeCompressed();
  return true;
}

double sequence_ratio(const std::array<Complex, 3>& values) {
  const Complex a = std::polar(1.0, 2.0 * kPi / 3.0);
  const Complex a2 = a * a;
  const Complex positive = (values[0] + a * values[1] + a2 * values[2]) / 3.0;
  const Complex negative = (values[0] + a2 * values[1] + a * values[2]) / 3.0;
  return std::abs(negative) / std::max(1e-12, std::abs(positive));
}

Eigen::VectorXd initial_point(const Model& model) {
  const auto& c = *model.source;
  const Layout& l = model.layout;
  Eigen::VectorXd x = Eigen::VectorXd::Zero(l.nvar);
  for (int i = 0; i < l.nac; ++i) {
    x[l.i_e + i] = std::real(c.voltage_start[i]);
    x[l.i_f + i] = std::imag(c.voltage_start[i]);
  }
  if (l.ndc > 0) x.segment(l.i_u, l.ndc) = c.v_dc_start;
  for (int ci = 0; ci < static_cast<int>(c.converters.size()); ++ci) {
    const auto& converter = c.converters[static_cast<std::size_t>(ci)];
    const auto& map = model.converters[static_cast<std::size_t>(ci)];
    if (map.control_position < 0) continue;
    x[l.i_control + map.control_position] =
        converter.control_mode == ThreePhaseHybridPFControlMode::GridFormingVdc
            ? std::arg(converter.internal_voltage_positive)
            : converter.p_set_pu;
  }
  return x;
}

}  // namespace

ThreePhaseHybridPFResult solve_three_phase_hybrid_pf(
    const ThreePhaseHybridPFCase& problem,
    const ThreePhaseHybridPFOptions& options) {
  const auto start = std::chrono::steady_clock::now();
  ThreePhaseHybridPFResult result;
  try {
    const Model model = build_model(problem);
    const Layout& l = model.layout;
    result.variables = l.nvar;
    result.equations = l.nvar;
    Eigen::VectorXd x = initial_point(model);
    Eigen::VectorXd residual;
    Eigen::SparseMatrix<double> jacobian;
    double norm = std::numeric_limits<double>::infinity();
    for (int iteration = 0; iteration <= options.max_iterations; ++iteration) {
      if (!evaluate(model, x, options, residual, &jacobian)) {
        result.status = "invalid voltage encountered during Newton iteration";
        break;
      }
      norm = residual.cwiseAbs().maxCoeff();
      result.iterations = iteration;
      result.residual = norm;
      result.jacobian_nonzeros = jacobian.nonZeros();
      if (options.verbose) {
        std::cerr << '[' << problem.name << "] hybrid abc/dc PF iter="
                  << iteration << " residual=" << norm << '\n';
      }
      if (norm <= options.tolerance) {
        result.converged = true;
        result.status = "Converged";
        break;
      }
      if (iteration == options.max_iterations) {
        result.status = "maximum iterations reached";
        break;
      }
      Eigen::SparseLU<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> solver;
      solver.analyzePattern(jacobian);
      solver.factorize(jacobian);
      if (solver.info() != Eigen::Success) {
        result.status = "sparse Jacobian factorization failed";
        break;
      }
      const Eigen::VectorXd step = solver.solve(-residual);
      if (solver.info() != Eigen::Success || !step.allFinite()) {
        result.status = "sparse Newton solve failed";
        break;
      }
      const double merit = 0.5 * residual.squaredNorm();
      bool accepted = false;
      double alpha = 1.0;
      for (int line = 0; line < options.max_line_search_steps; ++line) {
        const Eigen::VectorXd trial = x + alpha * step;
        Eigen::VectorXd trial_residual;
        if (evaluate(model, trial, options, trial_residual, nullptr)) {
          const double trial_merit = 0.5 * trial_residual.squaredNorm();
          if (trial_merit <= (1.0 - options.armijo * alpha) * merit) {
            x = trial;
            accepted = true;
            break;
          }
        }
        alpha *= 0.5;
      }
      if (!accepted) {
        result.status = "Newton line search failed";
        break;
      }
    }

    result.voltage = voltage(model, x);
    result.dc_voltage = x.segment(l.i_u, l.ndc);
    std::vector<ConverterEvaluation> evaluations;
    Eigen::VectorXd final_residual;
    if (!evaluate(model, x, options, final_residual, nullptr, &evaluations)) {
      result.converged = false;
      if (result.status.empty()) result.status = "final voltage audit failed";
      result.runtime_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      return result;
    }
    result.residual = final_residual.cwiseAbs().maxCoeff();
    result.ac_active_balance_residual = 0.0;
    result.ac_reactive_balance_residual = 0.0;
    for (int i = 0; i < l.nac; ++i) {
      if (model.is_reference[static_cast<std::size_t>(i)] != 0) continue;
      result.ac_active_balance_residual = std::max(
          result.ac_active_balance_residual, std::abs(final_residual[l.i_e + i]));
      result.ac_reactive_balance_residual = std::max(
          result.ac_reactive_balance_residual, std::abs(final_residual[l.i_f + i]));
    }
    for (int i = 0; i < l.ndc; ++i) {
      result.dc_balance_residual = std::max(
          result.dc_balance_residual, std::abs(final_residual[l.i_u + i]));
    }
    for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
      const auto& converter = problem.converters[static_cast<std::size_t>(ci)];
      const auto& map = model.converters[static_cast<std::size_t>(ci)];
      const auto& evaluation = evaluations[static_cast<std::size_t>(ci)];
      ThreePhaseHybridPFConverterResult item;
      item.solved_control = evaluation.control_value;
      item.internal_voltage_positive = evaluation.internal_voltage;
      std::array<Complex, 3> currents{};
      double p_ac = 0.0;
      for (int phase = 0; phase < 3; ++phase) {
        const int node = map.node_by_phase[static_cast<std::size_t>(phase)];
        const Complex s = evaluation.power[static_cast<std::size_t>(phase)];
        const Complex current = std::conj(s / result.voltage[node]);
        item.phase_power_pu.push_back(s);
        item.phase_current_pu.push_back(current);
        currents[static_cast<std::size_t>(phase)] = current;
        p_ac += std::real(s);
      }
      item.p_dc_pu = -converter.efficiency * p_ac;
      item.current_vuf = sequence_ratio(currents);
      result.converters.push_back(std::move(item));
    }
    // Independently infer the aggregate converter injection required at each
    // DC terminal from the solved network and fixed DC load. This avoids the
    // tautological check pdc := -eta*pac followed by pdc + eta*pac == 0.
    if (l.ndc > 0) {
      const Eigen::VectorXd dc_current = problem.g_dc * result.dc_voltage;
      Eigen::VectorXd reported_converter_injection =
          Eigen::VectorXd::Zero(l.ndc);
      for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
        const int terminal =
            problem.converters[static_cast<size_t>(ci)].dc_terminal;
        reported_converter_injection[terminal] +=
            result.converters[static_cast<size_t>(ci)].p_dc_pu;
      }
      for (int terminal = 0; terminal < l.ndc; ++terminal) {
        const bool has_converter = std::any_of(
            problem.converters.begin(), problem.converters.end(),
            [&](const ThreePhaseHybridPFConverter& converter) {
              return converter.dc_terminal == terminal;
            });
        if (!has_converter) continue;
        const double required_injection =
            result.dc_voltage[terminal] * dc_current[terminal] +
            problem.p_dc_load_pu[terminal];
        result.converter_coupling_residual = std::max(
            result.converter_coupling_residual,
            std::abs(required_injection -
                     reported_converter_injection[terminal]));
      }
    }
    // Phase-node order need not be grouped by bus; converters provide the only
    // explicit grouping in this matrix API. Report the maximum terminal VUF.
    for (const auto& map : model.converters) {
      std::array<Complex, 3> terminal{};
      for (int phase = 0; phase < 3; ++phase) {
        terminal[static_cast<std::size_t>(phase)] =
            result.voltage[map.node_by_phase[static_cast<std::size_t>(phase)]];
      }
      result.max_vuf = std::max(result.max_vuf, sequence_ratio(terminal));
    }
    if (result.converged && result.residual > options.tolerance) {
      result.converged = false;
      result.status = "final residual audit failed";
    }
  } catch (const std::exception& error) {
    result.status = error.what();
  }
  result.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

}  // namespace hacdcpf::powerflow
