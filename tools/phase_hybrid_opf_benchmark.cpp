#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
// MinGW GCC in strict -std=c++20 mode does not define M_PI via <cmath>.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace fs = std::filesystem;
using hacdcpf::PhaseMask;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::graph::SparseComplexMatrix;
using namespace hacdcpf::opf::phase_hybrid;
using Complex = std::complex<double>;

namespace {

struct Input {
  std::string label;
  fs::path master;
  double dc_share{0.0};
  int vsc_count{0};
};

struct CandidateBus {
  int bus_offset{-1};
  double p_pu{0.0};
  std::vector<int> nodes;
};

struct PhysicalFigureMetadata {
  struct Edge {
    std::string from_bus;
    std::string to_bus;
    std::string phases;
    std::string element;
  };

  std::vector<std::string> phase_node_bus;
  std::vector<int> phase_node_phase;
  std::vector<std::string> converter_bus;
  std::vector<Edge> edges;
};

struct Audit {
  std::string label;
  bool built{false};
  int phase_nodes{0};
  int dc_nodes{0};
  int vscs{0};
  double requested_dc_share{0.0};
  double realized_dc_share{0.0};
  ThreePhaseHybridOPFResult full;
  ThreePhaseHybridOPFResult reduced;
  ThreePhaseHybridOPFResult continuation;
  ThreePhaseHybridOPFResult continuation_reference;
  double continuation_objective_relative_error{
      std::numeric_limits<double>::quiet_NaN()};
  double continuation_voltage_error{
      std::numeric_limits<double>::quiet_NaN()};
  double objective_relative_error{std::numeric_limits<double>::quiet_NaN()};
  double voltage_error{std::numeric_limits<double>::quiet_NaN()};
  double speedup{std::numeric_limits<double>::quiet_NaN()};
  std::string error;
};

struct SensitivityRow {
  std::string scenario;
  int repeat{0};
  bool continuation_converged{false};
  bool reference_converged{false};
  int iterations{0};
  int oracle_rounds{0};
  int added_rows{0};
  int enforced_rows{0};
  int total_rows{0};
  double continuation_ms{0.0};
  double reference_ms{0.0};
  double objective_error{std::numeric_limits<double>::quiet_NaN()};
  double voltage_error{std::numeric_limits<double>::quiet_NaN()};
  double primal_residual{0.0};
  double dual_residual{0.0};
  double complementarity{0.0};
  double max_vuf{0.0};
  double phase_load_unbalance{0.0};
  double max_converter_current_vuf{0.0};
  double max_converter_current_loading{0.0};
  double max_dynamic_equilibrium_residual{0.0};
  std::array<double, 3> phase_vmin{};
  std::array<double, 3> phase_vmax{};
};

struct SensitivityScenario {
  std::string label;
  ThreePhaseHybridOPFCase problem;
};

struct DriftTraceRow {
  int iteration_cap{0};
  ThreePhaseHybridOPFResult result;
  double min_phase_generation_pu{std::numeric_limits<double>::quiet_NaN()};
  double max_phase_generation_pu{std::numeric_limits<double>::quiet_NaN()};
};

struct NativePhaseGraph {
  SparseComplexMatrix y_pu;
  Eigen::VectorXcd voltage_start_pu;
  std::vector<int> bus_offset;
  std::vector<int> phase_index;
  std::vector<std::array<int, 3>> bus_phase_to_node;

  bool has_node(int bus, int phase) const {
    return bus >= 0 && bus < static_cast<int>(bus_phase_to_node.size()) &&
           phase >= 0 && phase < 3 &&
           bus_phase_to_node[static_cast<std::size_t>(bus)]
                            [static_cast<std::size_t>(phase)] >= 0;
  }

  int node_index(int bus, int phase) const {
    if (!has_node(bus, phase)) {
      throw std::runtime_error("native OpenDSS phase node is missing");
    }
    return bus_phase_to_node[static_cast<std::size_t>(bus)]
                            [static_cast<std::size_t>(phase)];
  }
};

fs::path project_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return fs::path(HACDCPF_PROJECT_ROOT);
#else
  return fs::current_path();
#endif
}

std::string lowercase_ascii(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

NativePhaseGraph build_native_phase_graph(
    const fs::path& master,
    const ThreePhaseACSystem& sys) {
  const auto snapshot = hacdcpf::analysis::build_opendss_sparse_y_matrix(
      master, {}, false, true, true);
  NativePhaseGraph graph;
  graph.bus_offset.resize(static_cast<std::size_t>(snapshot.dimension), -1);
  graph.phase_index.resize(static_cast<std::size_t>(snapshot.dimension), -1);
  graph.bus_phase_to_node.assign(
      sys.buses.size(), std::array<int, 3>{-1, -1, -1});

  std::unordered_map<std::string, int> bus_by_name;
  bus_by_name.reserve(sys.buses.size());
  for (int offset = 0; offset < static_cast<int>(sys.buses.size()); ++offset) {
    bus_by_name[lowercase_ascii(sys.buses[static_cast<std::size_t>(offset)].name)] =
        offset;
  }
  for (int node = 0; node < snapshot.dimension; ++node) {
    const std::string label = lowercase_ascii(
        snapshot.node_order[static_cast<std::size_t>(node)]);
    const std::size_t separator = label.rfind('.');
    if (separator == std::string::npos || separator + 1 >= label.size()) {
      throw std::runtime_error("invalid OpenDSS Y-node label: " + label);
    }
    const int conductor = std::stoi(label.substr(separator + 1));
    if (conductor < 1 || conductor > 3) {
      throw std::runtime_error("unsupported OpenDSS conductor in Y-node label: " + label);
    }
    const auto bus_it = bus_by_name.find(label.substr(0, separator));
    if (bus_it == bus_by_name.end()) {
      throw std::runtime_error("OpenDSS Y-node bus is absent from imported buses: " + label);
    }
    const int phase = conductor - 1;
    const int bus = bus_it->second;
    int& slot = graph.bus_phase_to_node[static_cast<std::size_t>(bus)]
                                       [static_cast<std::size_t>(phase)];
    if (slot >= 0) {
      throw std::runtime_error("duplicate OpenDSS Y-node label: " + label);
    }
    slot = node;
    graph.bus_offset[static_cast<std::size_t>(node)] = bus;
    graph.phase_index[static_cast<std::size_t>(node)] = phase;
  }

  std::vector<Eigen::Triplet<Complex>> triplets;
  triplets.reserve(snapshot.entries.size());
  const double base_mva = sys.base_mva > 0.0 ? sys.base_mva : 1.0;
  for (const auto& entry : snapshot.entries) {
    const auto& row_bus = sys.buses[static_cast<std::size_t>(
        graph.bus_offset[static_cast<std::size_t>(entry.row)])];
    const auto& col_bus = sys.buses[static_cast<std::size_t>(
        graph.bus_offset[static_cast<std::size_t>(entry.col)])];
    if (!(row_bus.base_kv > 0.0) || !(col_bus.base_kv > 0.0)) {
      throw std::runtime_error("OpenDSS Y-node has no positive voltage base");
    }
    const double row_phase_base_kv = row_bus.phase_mask.count() == 3
        ? row_bus.base_kv / std::sqrt(3.0) : row_bus.base_kv;
    const double col_phase_base_kv = col_bus.phase_mask.count() == 3
        ? col_bus.base_kv / std::sqrt(3.0) : col_bus.base_kv;
    const double scale =
        row_phase_base_kv * col_phase_base_kv / base_mva;
    triplets.emplace_back(entry.row, entry.col, entry.value * scale);
  }
  graph.y_pu.resize(snapshot.dimension, snapshot.dimension);
  graph.y_pu.setFromTriplets(triplets.begin(), triplets.end(),
                             [](Complex a, Complex b) { return a + b; });
  graph.y_pu.prune(Complex{}, 1e-13);
  graph.y_pu.makeCompressed();
  graph.voltage_start_pu.resize(snapshot.dimension);
  for (int node = 0; node < snapshot.dimension; ++node) {
    const auto& bus = sys.buses[static_cast<std::size_t>(
        graph.bus_offset[static_cast<std::size_t>(node)])];
    const double phase_base_kv = bus.phase_mask.count() == 3
        ? bus.base_kv / std::sqrt(3.0) : bus.base_kv;
    const Complex voltage = snapshot.node_voltage_volts.empty()
        ? Complex{}
        : snapshot.node_voltage_volts[static_cast<std::size_t>(node)];
    graph.voltage_start_pu[node] = voltage / (1000.0 * phase_base_kv);
  }
  return graph;
}

std::vector<int> restrict_to_reference_connected_phase_graph(
    ThreePhaseHybridOPFCase& c) {
  const int n = static_cast<int>(c.y_ac.rows());
  std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(n));
  for (int col = 0; col < c.y_ac.outerSize(); ++col) {
    for (SparseComplexMatrix::InnerIterator it(c.y_ac, col); it; ++it) {
      if (it.row() == it.col() || std::abs(it.value()) <= 1e-13) continue;
      adjacency[static_cast<std::size_t>(it.row())].push_back(it.col());
      adjacency[static_cast<std::size_t>(it.col())].push_back(it.row());
    }
  }

  std::vector<bool> reached(static_cast<std::size_t>(n), false);
  std::queue<int> frontier;
  for (int node : c.reference_nodes) {
    if (reached[static_cast<std::size_t>(node)]) continue;
    reached[static_cast<std::size_t>(node)] = true;
    frontier.push(node);
  }
  while (!frontier.empty()) {
    const int node = frontier.front();
    frontier.pop();
    for (int neighbor : adjacency[static_cast<std::size_t>(node)]) {
      if (reached[static_cast<std::size_t>(neighbor)]) continue;
      reached[static_cast<std::size_t>(neighbor)] = true;
      frontier.push(neighbor);
    }
  }

  std::vector<int> old_to_new(static_cast<std::size_t>(n), -1);
  int retained = 0;
  int disconnected_with_injection = 0;
  for (int node = 0; node < n; ++node) {
    if (reached[static_cast<std::size_t>(node)]) {
      old_to_new[static_cast<std::size_t>(node)] = retained++;
      continue;
    }
    if (std::abs(c.p_load_pu[node]) > 1e-12 ||
        std::abs(c.q_load_pu[node]) > 1e-12 ||
        std::abs(c.i_ac_fixed[node]) > 1e-12) {
      ++disconnected_with_injection;
    }
  }
  if (disconnected_with_injection > 0) {
    throw std::runtime_error(
        "reference-disconnected AC phase component contains " +
        std::to_string(disconnected_with_injection) + " injected phase nodes");
  }
  if (retained == n) return old_to_new;

  std::vector<Eigen::Triplet<Complex>> ytriplets;
  ytriplets.reserve(static_cast<std::size_t>(c.y_ac.nonZeros()));
  for (int col = 0; col < c.y_ac.outerSize(); ++col) {
    for (SparseComplexMatrix::InnerIterator it(c.y_ac, col); it; ++it) {
      const int row_new = old_to_new[static_cast<std::size_t>(it.row())];
      const int col_new = old_to_new[static_cast<std::size_t>(it.col())];
      if (row_new >= 0 && col_new >= 0) {
        ytriplets.emplace_back(row_new, col_new, it.value());
      }
    }
  }
  SparseComplexMatrix energized_y(retained, retained);
  energized_y.setFromTriplets(ytriplets.begin(), ytriplets.end());
  energized_y.makeCompressed();

  const auto restrict_real = [&](const Eigen::VectorXd& full) {
    Eigen::VectorXd energized(retained);
    for (int old = 0; old < n; ++old) {
      const int mapped = old_to_new[static_cast<std::size_t>(old)];
      if (mapped >= 0) energized[mapped] = full[old];
    }
    return energized;
  };
  const auto restrict_complex = [&](const Eigen::VectorXcd& full) {
    Eigen::VectorXcd energized(retained);
    for (int old = 0; old < n; ++old) {
      const int mapped = old_to_new[static_cast<std::size_t>(old)];
      if (mapped >= 0) energized[mapped] = full[old];
    }
    return energized;
  };

  c.y_ac = std::move(energized_y);
  c.i_ac_fixed = restrict_complex(c.i_ac_fixed);
  c.p_load_pu = restrict_real(c.p_load_pu);
  c.q_load_pu = restrict_real(c.q_load_pu);
  c.v_min_pu = restrict_real(c.v_min_pu);
  c.v_max_pu = restrict_real(c.v_max_pu);
  c.voltage_start = restrict_complex(c.voltage_start);
  if (!c.ac_phase_index.empty()) {
    std::vector<int> energized_phase(static_cast<std::size_t>(retained), -1);
    for (int old = 0; old < n; ++old) {
      const int mapped = old_to_new[static_cast<std::size_t>(old)];
      if (mapped >= 0) {
        energized_phase[static_cast<std::size_t>(mapped)] =
            c.ac_phase_index[static_cast<std::size_t>(old)];
      }
    }
    c.ac_phase_index = std::move(energized_phase);
  }
  for (int& node : c.reference_nodes) {
    node = old_to_new[static_cast<std::size_t>(node)];
  }
  return old_to_new;
}

std::vector<Input> inputs() {
  const fs::path refs = project_root() / "external_data" / "opendss_ieee_pes" /
                        "opendss_reference";
  return {
      {"H13", refs / "13_node" / "official_full" / "IEEE13Nodeckt.dss", 0.15, 1},
      {"H34", refs / "34_node" / "ieee34Mod2.dss", 0.20, 2},
      {"H123", refs / "123_node" / "IEEE123Master.dss", 0.25, 4},
      {"H8500", refs / "8500_node" / "Master.dss", 0.05, 12},
  };
}

double phase_value(const double values[3], int phase) {
  return values[phase];
}

// Per-conductor AC line thermal limits and DC branch limits, with ratings set
// from the network's own base OPF operating point times a headroom margin so
// that the base dispatch is strictly feasible and only stressed scenarios drive
// a few conductors to their limits. Series admittances are read from the
// assembled nodal matrices (an off-diagonal equals the negative of the series
// admittance), so no separate branch database is required.
void attach_line_current_limits(ThreePhaseHybridOPFCase& c, double margin) {
  const double tol = 1e-9;
  // Base operating point from a full OPF solve without line limits.
  ThreePhaseHybridOPFOptions base_options;
  base_options.variant = ModelVariant::Full;
  base_options.backend = SolverBackend::Ipopt;
  base_options.verbose = false;
  const ThreePhaseHybridOPFResult base = solve_three_phase_hybrid_opf(c, base_options);
  const bool have_ac_solution =
      base.converged && base.full_voltage.size() == c.y_ac.rows();
  const bool have_dc_solution =
      base.converged && base.dc_voltage.size() == c.g_dc.rows();
  const Eigen::VectorXcd& v_base =
      have_ac_solution ? base.full_voltage : c.voltage_start;
  for (int col = 0; col < c.y_ac.outerSize(); ++col) {
    for (SparseComplexMatrix::InnerIterator it(c.y_ac, col); it; ++it) {
      const int i = it.row();
      const int j = it.col();
      if (i >= j) continue;
      if (c.ac_phase_index[static_cast<std::size_t>(i)] !=
          c.ac_phase_index[static_cast<std::size_t>(j)]) {
        continue;  // same-phase off-diagonal => a series line conductor
      }
      const Complex y_series = -it.value();
      if (std::abs(y_series) <= tol) continue;
      const double base_current = std::abs(y_series * (v_base[i] - v_base[j]));
      if (base_current < 1e-3) continue;  // unloaded conductor: no thermal limit
      const double rating = margin * base_current;
      ACLineCurrentLimit limit;
      limit.nodes = {i, j};
      limit.coefficients = {y_series, -y_series};
      limit.i_max_pu = rating;
      c.ac_line_limits.push_back(std::move(limit));
    }
  }
  for (int col = 0; col < c.g_dc.outerSize(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(c.g_dc, col); it; ++it) {
      const int i = it.row();
      const int j = it.col();
      if (i >= j) continue;
      const double g_series = -it.value();
      if (std::abs(g_series) <= tol) continue;
      const double u_i = have_dc_solution ? base.dc_voltage[i] : c.v_dc_start[i];
      const double u_j = have_dc_solution ? base.dc_voltage[j] : c.v_dc_start[j];
      const double base_current = std::abs(g_series * (u_i - u_j));
      if (base_current < 1e-3) continue;  // unloaded DC branch: no thermal limit
      const double rating = margin * base_current;
      DCLineCurrentLimit limit;
      limit.from_node = i;
      limit.to_node = j;
      limit.conductance_pu = g_series;
      limit.i_max_pu = rating;
      c.dc_line_limits.push_back(std::move(limit));
    }
  }
  std::cerr << "[line-limits] " << c.name << " ac=" << c.ac_line_limits.size()
            << " dc=" << c.dc_line_limits.size()
            << " base=" << (base.converged ? 1 : 0) << "\n";
}

ThreePhaseHybridOPFCase hybridize(
    const Input& input, PhysicalFigureMetadata* figure_metadata = nullptr) {
  const ThreePhaseACSystem sys =
      hacdcpf::analysis::load_three_phase_system_from_opendss(input.master, 100.0);
  if (sys.buses.empty()) throw std::runtime_error("OpenDSS import returned no buses");
  const NativePhaseGraph phase_graph = build_native_phase_graph(input.master, sys);
  const int n = static_cast<int>(phase_graph.y_pu.rows());

  ThreePhaseHybridOPFCase c;
  c.name = input.label;
  c.base_mva = sys.base_mva > 0.0 ? sys.base_mva : 1.0;
  c.y_ac = phase_graph.y_pu;
  c.i_ac_fixed = Eigen::VectorXcd::Zero(n);
  c.p_load_pu = Eigen::VectorXd::Zero(n);
  c.q_load_pu = Eigen::VectorXd::Zero(n);
  c.v_min_pu = Eigen::VectorXd::Constant(n, 0.85);
  c.v_max_pu = Eigen::VectorXd::Constant(n, 1.10);
  c.voltage_start = Eigen::VectorXcd::Ones(n);
  c.ac_phase_index = phase_graph.phase_index;
  c.vuf_max = 0.03;

  for (int k = 0; k < n; ++k) {
    const int bus_offset = phase_graph.bus_offset[static_cast<std::size_t>(k)];
    const int phase_index = phase_graph.phase_index[static_cast<std::size_t>(k)];
    const auto& bus = sys.buses[static_cast<std::size_t>(bus_offset)];
    const double pd[3] = {bus.pd_a_mw, bus.pd_b_mw, bus.pd_c_mw};
    const double qd[3] = {bus.qd_a_mvar, bus.qd_b_mvar, bus.qd_c_mvar};
    c.voltage_start[k] = std::abs(phase_graph.voltage_start_pu[k]) > 0.0
        ? phase_graph.voltage_start_pu[k]
        : Complex{1.0, 0.0};
    c.p_load_pu[k] += pd[phase_index] / c.base_mva;
    c.q_load_pu[k] += qd[phase_index] / c.base_mva;
    c.v_min_pu[k] = bus.vmin_pu > 0.0 ? bus.vmin_pu : 0.85;
    c.v_max_pu[k] = bus.vmax_pu > 0.0 ? bus.vmax_pu : 1.10;
  }
  for (const auto& load : sys.loads) {
    if (!load.in_service) continue;
    const auto bus_it = std::find_if(
        sys.buses.begin(), sys.buses.end(),
        [&](const auto& bus) { return bus.index == load.bus; });
    if (bus_it == sys.buses.end()) continue;
    const int bus_offset = static_cast<int>(std::distance(sys.buses.begin(), bus_it));
    const double p[3] = {load.p_a_mw, load.p_b_mw, load.p_c_mw};
    const double q[3] = {load.q_a_mvar, load.q_b_mvar, load.q_c_mvar};
    for (int phase = 0; phase < 3; ++phase) {
      if (!load.phase_mask.has(phase) || !phase_graph.has_node(bus_offset, phase)) continue;
      const int node = phase_graph.node_index(bus_offset, phase);
      c.p_load_pu[node] += p[phase] / c.base_mva;
      c.q_load_pu[node] += q[phase] / c.base_mva;
    }
  }

  std::unordered_set<int> reference_node_set;
  for (const auto& grid : sys.external_grids) {
    if (!grid.in_service) continue;
    const auto bus_it = std::find_if(
        sys.buses.begin(), sys.buses.end(),
        [&](const auto& bus) { return bus.index == grid.bus; });
    if (bus_it == sys.buses.end()) continue;
    const int bus_offset = static_cast<int>(std::distance(sys.buses.begin(), bus_it));
    for (int phase = 0; phase < 3; ++phase) {
      if (!grid.phase_mask.has(phase) || !phase_graph.has_node(bus_offset, phase)) continue;
      const int node = phase_graph.node_index(bus_offset, phase);
      if (!reference_node_set.insert(node).second) continue;
      const double phase_shift[3] = {0.0, -120.0, 120.0};
      const double vm[3] = {grid.vm_a_pu, grid.vm_b_pu, grid.vm_c_pu};
      const double va[3] = {grid.va_a_deg, grid.va_b_deg, grid.va_c_deg};
      double magnitude = grid.use_phase_voltage_setpoint && vm[phase] > 0.0
          ? vm[phase] : (grid.vm_pu > 0.0 ? grid.vm_pu : 1.0);
      if (input.label == "H8500") magnitude = std::min(magnitude, 1.04);
      const double angle = grid.use_phase_voltage_setpoint
          ? va[phase] : grid.va_deg + phase_shift[phase];
      c.reference_nodes.push_back(node);
      c.reference_voltage.conservativeResize(c.reference_voltage.size() + 1);
      c.reference_voltage[c.reference_voltage.size() - 1] =
          std::polar(magnitude, angle * M_PI / 180.0);
    }
  }
  if (c.reference_nodes.empty()) {
    for (int bus_offset = 0; bus_offset < static_cast<int>(sys.buses.size()); ++bus_offset) {
      if (sys.buses[static_cast<std::size_t>(bus_offset)].bus_type !=
          hacdcpf::BusType::SLACK) continue;
      for (int phase = 0; phase < 3; ++phase) {
        if (!phase_graph.has_node(bus_offset, phase)) continue;
        const int node = phase_graph.node_index(bus_offset, phase);
        c.reference_nodes.push_back(node);
        c.reference_voltage.conservativeResize(c.reference_voltage.size() + 1);
        c.reference_voltage[c.reference_voltage.size() - 1] = c.voltage_start[node];
      }
      break;
    }
  }
  if (c.reference_nodes.empty()) throw std::runtime_error("no AC reference phase found");

  const std::vector<int> old_to_energized =
      restrict_to_reference_connected_phase_graph(c);
  const int excluded_phase_nodes =
      n - static_cast<int>(c.y_ac.rows());
  if (excluded_phase_nodes > 0) {
    std::cerr << '[' << input.label << "] excluded " << excluded_phase_nodes
              << " reference-disconnected zero-injection phase nodes\n";
  }
  reference_node_set.clear();
  reference_node_set.insert(c.reference_nodes.begin(), c.reference_nodes.end());

  for (int bus_offset = 0; bus_offset < static_cast<int>(sys.buses.size()); ++bus_offset) {
    if (phase_graph.has_node(bus_offset, 0) && phase_graph.has_node(bus_offset, 1) &&
        phase_graph.has_node(bus_offset, 2)) {
      const int a = old_to_energized[static_cast<std::size_t>(
          phase_graph.node_index(bus_offset, 0))];
      const int b = old_to_energized[static_cast<std::size_t>(
          phase_graph.node_index(bus_offset, 1))];
      const int phase_c = old_to_energized[static_cast<std::size_t>(
          phase_graph.node_index(bus_offset, 2))];
      if (a >= 0 && b >= 0 && phase_c >= 0) {
        c.three_phase_bus_nodes.push_back({a, b, phase_c});
      }
    }
  }

  std::vector<CandidateBus> candidates;
  for (int bus_offset = 0; bus_offset < static_cast<int>(sys.buses.size()); ++bus_offset) {
    CandidateBus candidate;
    candidate.bus_offset = bus_offset;
    for (int phase = 0; phase < 3; ++phase) {
      if (!phase_graph.has_node(bus_offset, phase)) continue;
      const int node = old_to_energized[static_cast<std::size_t>(
          phase_graph.node_index(bus_offset, phase))];
      if (node < 0) continue;
      candidate.nodes.push_back(node);
      candidate.p_pu += std::max(0.0, c.p_load_pu[node]);
    }
    if (candidate.p_pu > 1e-9 && !candidate.nodes.empty()) {
      const bool is_reference = std::any_of(
          candidate.nodes.begin(), candidate.nodes.end(),
          [&](int node) { return reference_node_set.count(node) != 0; });
      if (!is_reference) candidates.push_back(std::move(candidate));
    }
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
    if (a.p_pu != b.p_pu) return a.p_pu > b.p_pu;
    return a.bus_offset < b.bus_offset;
  });
  if (static_cast<int>(candidates.size()) < input.vsc_count) {
    throw std::runtime_error("not enough loaded buses for requested VSC count");
  }

  const double original_p = c.p_load_pu.sum();
  const double target_dc = input.dc_share * original_p;
  std::vector<std::vector<int>> adjacency(
      static_cast<std::size_t>(c.y_ac.rows()));
  for (int col = 0; col < c.y_ac.outerSize(); ++col) {
    for (SparseComplexMatrix::InnerIterator it(c.y_ac, col); it; ++it) {
      if (it.row() == it.col() || std::abs(it.value()) <= 1e-13) continue;
      adjacency[static_cast<std::size_t>(it.row())].push_back(it.col());
      adjacency[static_cast<std::size_t>(it.col())].push_back(it.row());
    }
  }
  const auto distances_from = [&](const std::vector<int>& sources) {
    const int unreachable = std::numeric_limits<int>::max() / 4;
    std::vector<int> distance(static_cast<std::size_t>(c.y_ac.rows()), unreachable);
    std::queue<int> frontier;
    for (int source : sources) {
      distance[static_cast<std::size_t>(source)] = 0;
      frontier.push(source);
    }
    while (!frontier.empty()) {
      const int node = frontier.front();
      frontier.pop();
      for (int neighbor : adjacency[static_cast<std::size_t>(node)]) {
        if (distance[static_cast<std::size_t>(neighbor)] <=
            distance[static_cast<std::size_t>(node)] + 1) {
          continue;
        }
        distance[static_cast<std::size_t>(neighbor)] =
            distance[static_cast<std::size_t>(node)] + 1;
        frontier.push(neighbor);
      }
    }
    return distance;
  };

  const int unreachable = std::numeric_limits<int>::max() / 4;
  std::vector<int> selected_candidates;
  std::vector<std::vector<int>> center_distances;
  std::vector<int> minimum_distance(
      static_cast<std::size_t>(c.y_ac.rows()), unreachable);
  for (int center = 0; center < input.vsc_count; ++center) {
    int selected = 0;
    if (center > 0) {
      int best_distance = -1;
      for (int candidate_index = 0;
           candidate_index < static_cast<int>(candidates.size()); ++candidate_index) {
        if (std::find(selected_candidates.begin(), selected_candidates.end(),
                      candidate_index) != selected_candidates.end()) {
          continue;
        }
        int candidate_distance = unreachable;
        for (int node : candidates[static_cast<std::size_t>(candidate_index)].nodes) {
          candidate_distance = std::min(
              candidate_distance,
              minimum_distance[static_cast<std::size_t>(node)]);
        }
        if (candidate_distance > best_distance) {
          best_distance = candidate_distance;
          selected = candidate_index;
        }
      }
    }
    selected_candidates.push_back(selected);
    center_distances.push_back(distances_from(
        candidates[static_cast<std::size_t>(selected)].nodes));
    const auto& latest = center_distances.back();
    for (int node = 0; node < static_cast<int>(latest.size()); ++node) {
      minimum_distance[static_cast<std::size_t>(node)] = std::min(
          minimum_distance[static_cast<std::size_t>(node)],
          latest[static_cast<std::size_t>(node)]);
    }
  }

  std::vector<double> bucket_load(static_cast<std::size_t>(input.vsc_count), 0.0);
  std::vector<std::vector<int>> terminal_nodes(static_cast<std::size_t>(input.vsc_count));
  const std::vector<int> reference_distance = distances_from(c.reference_nodes);
  for (int candidate_index = 0;
       candidate_index < static_cast<int>(candidates.size()); ++candidate_index) {
    const auto& candidate = candidates[static_cast<std::size_t>(candidate_index)];
    int bucket = 0;
    int best_distance = unreachable;
    for (int center = 0; center < input.vsc_count; ++center) {
      int distance = unreachable;
      for (int node : candidate.nodes) {
        distance = std::min(
            distance,
            center_distances[static_cast<std::size_t>(center)]
                            [static_cast<std::size_t>(node)]);
      }
      if (distance < best_distance) {
        best_distance = distance;
        bucket = center;
      }
    }
    for (int node : candidate.nodes) {
      const double moved = input.dc_share * std::max(0.0, c.p_load_pu[node]);
      c.p_load_pu[node] -= moved;
      c.q_load_pu[node] *= 1.0 - input.dc_share;
      bucket_load[static_cast<std::size_t>(bucket)] += moved;
    }
  }

  std::vector<int> energized_to_old(static_cast<std::size_t>(c.y_ac.rows()), -1);
  for (int old = 0; old < static_cast<int>(old_to_energized.size()); ++old) {
    const int energized = old_to_energized[static_cast<std::size_t>(old)];
    if (energized >= 0) {
      energized_to_old[static_cast<std::size_t>(energized)] = old;
    }
  }
  if (figure_metadata != nullptr) {
    figure_metadata->phase_node_bus.resize(
        static_cast<std::size_t>(c.y_ac.rows()));
    figure_metadata->phase_node_phase = c.ac_phase_index;
    for (int node = 0; node < c.y_ac.rows(); ++node) {
      const int old = energized_to_old[static_cast<std::size_t>(node)];
      const int bus_offset =
          phase_graph.bus_offset[static_cast<std::size_t>(old)];
      figure_metadata->phase_node_bus[static_cast<std::size_t>(node)] =
          sys.buses[static_cast<std::size_t>(bus_offset)].name;
    }
  }
  std::vector<int> cluster_root(static_cast<std::size_t>(input.vsc_count), -1);
  std::vector<int> cluster_root_distance(
      static_cast<std::size_t>(input.vsc_count), unreachable);
  for (int node = 0; node < static_cast<int>(c.y_ac.rows()); ++node) {
    int bucket = 0;
    int best_center_distance = unreachable;
    for (int center = 0; center < input.vsc_count; ++center) {
      const int distance = center_distances[static_cast<std::size_t>(center)]
                                           [static_cast<std::size_t>(node)];
      if (distance < best_center_distance) {
        best_center_distance = distance;
        bucket = center;
      }
    }
    if (reference_distance[static_cast<std::size_t>(node)] <
        cluster_root_distance[static_cast<std::size_t>(bucket)]) {
      cluster_root_distance[static_cast<std::size_t>(bucket)] =
          reference_distance[static_cast<std::size_t>(node)];
      cluster_root[static_cast<std::size_t>(bucket)] = node;
    }
  }
  for (int center = 0; center < input.vsc_count; ++center) {
    const int root = cluster_root[static_cast<std::size_t>(center)];
    if (root < 0) throw std::runtime_error("DC load cluster has no AC boundary root");
    const int old_root = energized_to_old[static_cast<std::size_t>(root)];
    const int bus_offset =
        phase_graph.bus_offset[static_cast<std::size_t>(old_root)];
    for (int phase = 0; phase < 3; ++phase) {
      if (!phase_graph.has_node(bus_offset, phase)) continue;
      const int old_node = phase_graph.node_index(bus_offset, phase);
      const int energized = old_to_energized[static_cast<std::size_t>(old_node)];
      if (energized >= 0) {
        terminal_nodes[static_cast<std::size_t>(center)].push_back(energized);
      }
    }
  }
  const double realized_target = std::accumulate(
      bucket_load.begin(), bucket_load.end(), 0.0);
  if (std::abs(realized_target - target_dc) >
      1e-8 * std::max(1.0, target_dc)) {
    throw std::runtime_error("unable to realize requested topology-local DC load share");
  }

  const int ndc = 2 * input.vsc_count;
  c.g_dc.resize(ndc, ndc);
  std::vector<Eigen::Triplet<double>> gtrip;
  c.p_dc_load_pu = Eigen::VectorXd::Zero(ndc);
  c.v_dc_start = Eigen::VectorXd::Ones(ndc);
  c.v_dc_min_pu = Eigen::VectorXd::Constant(ndc, 0.90);
  c.v_dc_max_pu = Eigen::VectorXd::Constant(ndc, 1.10);
  for (int k = 0; k < input.vsc_count; ++k) {
    const int terminal = 2 * k;
    const int load = terminal + 1;
    const double p = bucket_load[static_cast<std::size_t>(k)];
    const double conductance = 100.0 * std::max(1.0, p);
    gtrip.emplace_back(terminal, terminal, conductance);
    gtrip.emplace_back(terminal, load, -conductance);
    gtrip.emplace_back(load, terminal, -conductance);
    gtrip.emplace_back(load, load, conductance);
    c.p_dc_load_pu[load] = p;
    const double disc = std::max(0.0, 1.0 - 4.0 * p / conductance);
    c.v_dc_start[load] = 0.5 * (1.0 + std::sqrt(disc));
    c.dc_reference_terminals.push_back(terminal);
    PhaseVSC converter;
    converter.phase_nodes = terminal_nodes[static_cast<std::size_t>(k)];
    converter.dc_terminal = terminal;
    converter.efficiency = 0.98;
    converter.s_max_pu = 1.30 * p / converter.efficiency + 1e-3;
    converter.phase_current_max_pu = converter.s_max_pu /
        std::sqrt(static_cast<double>(converter.phase_nodes.size()));
    converter.fixed_unity_power_factor = false;
    if (converter.phase_nodes.size() == 3) {
      const bool gfm_anchor = input.label == "H8500"
          ? k == 0 : input.vsc_count > 1 && k % 3 == 0;
      converter.control_mode = gfm_anchor
          ? PhaseVSCControlMode::GridFormingDroop
          : PhaseVSCControlMode::GridFollowingPLL;
    }
    converter.nominal_frequency_hz = 50.0;
    converter.pll_kp = 0.01;
    converter.pll_ki = 1.0;
    converter.virtual_r_pu = 0.01;
    converter.virtual_x_pu = 0.10;
    converter.p_droop_pu = 0.01;
    converter.q_droop_pu = 0.05;
    converter.voltage_reference_pu = 1.0;
    converter.voltage_integral_gain = 10.0;
    c.converters.push_back(std::move(converter));
    if (figure_metadata != nullptr) {
      const int node = terminal_nodes[static_cast<std::size_t>(k)].front();
      figure_metadata->converter_bus.push_back(
          figure_metadata->phase_node_bus[static_cast<std::size_t>(node)]);
    }
  }
  c.g_dc.setFromTriplets(gtrip.begin(), gtrip.end());
  c.g_dc.makeCompressed();
  c.dc_reference_voltage_pu = Eigen::VectorXd::Ones(
      static_cast<int>(c.dc_reference_terminals.size()));

  const double total_capacity = 5.0 * (original_p + 1.0);
  for (int ri = 0; ri < static_cast<int>(c.reference_nodes.size()); ++ri) {
    PhaseGenerator generator;
    generator.phase_node = c.reference_nodes[static_cast<std::size_t>(ri)];
    generator.p_min_pu = -0.25 * original_p;
    generator.p_max_pu = total_capacity;
    generator.q_min_pu = -total_capacity;
    generator.q_max_pu = total_capacity;
    generator.cost_c2 = 0.01;
    generator.cost_c1 = 45.0 + 0.5 * ri;
    c.generators.push_back(generator);
  }
  if (figure_metadata != nullptr) {
    std::unordered_map<int, std::string> bus_name;
    for (const auto& bus : sys.buses) bus_name.emplace(bus.index, bus.name);
    for (const auto& line : sys.lines) {
      if (!line.in_service) continue;
      const auto from = bus_name.find(line.from_bus);
      const auto to = bus_name.find(line.to_bus);
      if (from == bus_name.end() || to == bus_name.end()) continue;
      figure_metadata->edges.push_back(
          {from->second, to->second,
           hacdcpf::phase_mask_to_string(line.phase_mask),
           "line"});
    }
    for (const auto& transformer : sys.transformers) {
      if (!transformer.in_service) continue;
      const auto from = bus_name.find(transformer.hv_bus);
      const auto to = bus_name.find(transformer.lv_bus);
      if (from == bus_name.end() || to == bus_name.end()) continue;
      figure_metadata->edges.push_back(
          {from->second, to->second,
           hacdcpf::phase_mask_to_string(transformer.hv_phase_mask),
           "transformer"});
    }
  }
  attach_line_current_limits(c, 1.25);
  return c;
}

double max_voltage_difference(const Eigen::VectorXcd& a,
                              const Eigen::VectorXcd& b) {
  if (a.size() != b.size() || a.size() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (a - b).cwiseAbs().maxCoeff();
}

std::vector<SensitivityScenario> make_sensitivity_scenarios(
    const ThreePhaseHybridOPFCase& base) {
  std::vector<SensitivityScenario> scenarios;
  scenarios.push_back({"base", base});
  const auto add = [&](std::string label, ThreePhaseHybridOPFCase problem) {
    problem.name = base.name + "_" + label;
    scenarios.push_back({std::move(label), std::move(problem)});
  };

  ThreePhaseHybridOPFCase problem = base;
  problem.p_load_pu *= 1.001;
  problem.q_load_pu *= 1.001;
  add("ac_uniform_up_0p1", std::move(problem));

  problem = base;
  problem.p_load_pu *= 1.02;
  problem.q_load_pu *= 1.02;
  add("ac_uniform_up_2", std::move(problem));

  problem = base;
  problem.p_dc_load_pu *= 1.05;
  add("dc_uniform_up_5", std::move(problem));

  problem = base;
  for (int node = 0; node < problem.p_load_pu.size(); ++node) {
    if (problem.ac_phase_index[static_cast<std::size_t>(node)] == 0) {
      problem.p_load_pu[node] *= 1.03;
      problem.q_load_pu[node] *= 1.03;
    }
  }
  add("phase_a_up_3", std::move(problem));

  problem = base;
  for (int node = 0; node < problem.p_load_pu.size(); ++node) {
    const int phase = problem.ac_phase_index[static_cast<std::size_t>(node)];
    const double scale = phase == 0 ? 1.03 : 0.985;
    problem.p_load_pu[node] *= scale;
    problem.q_load_pu[node] *= scale;
  }
  add("phase_transfer_3", std::move(problem));

  problem = base;
  problem.reference_voltage *= 0.995;
  add("source_voltage_down_0p5", std::move(problem));

  problem = base;
  problem.p_load_pu *= 1.005;
  problem.q_load_pu *= 1.005;
  problem.p_dc_load_pu *= 1.01;
  problem.reference_voltage *= 0.999;
  for (int node = 0; node < problem.p_load_pu.size(); ++node) {
    if (problem.ac_phase_index[static_cast<std::size_t>(node)] == 0) {
      problem.p_load_pu[node] *= 1.005;
      problem.q_load_pu[node] *= 1.005;
    }
  }
  add("combined_moderate", std::move(problem));
  return scenarios;
}

double phase_load_unbalance(const ThreePhaseHybridOPFCase& problem) {
  std::array<double, 3> totals{};
  for (int node = 0; node < problem.p_load_pu.size(); ++node) {
    totals[static_cast<std::size_t>(
        problem.ac_phase_index[static_cast<std::size_t>(node)])] +=
        problem.p_load_pu[node];
  }
  const double mean = (totals[0] + totals[1] + totals[2]) / 3.0;
  double deviation = 0.0;
  for (double total : totals) deviation = std::max(deviation, std::abs(total - mean));
  return deviation / std::max(1e-12, std::abs(mean));
}

std::string control_mode_name(PhaseVSCControlMode mode) {
  switch (mode) {
    case PhaseVSCControlMode::EqualPhasePower:
      return "Equal phase";
    case PhaseVSCControlMode::GridFollowingPLL:
      return "GFL";
    case PhaseVSCControlMode::GridFormingDroop:
      return "GFM";
  }
  return "Unknown";
}

double sequence_unbalance_percent(const std::array<Complex, 3>& phasors) {
  const Complex alpha = std::polar(1.0, 2.0 * M_PI / 3.0);
  const Complex positive =
      (phasors[0] + alpha * phasors[1] + alpha * alpha * phasors[2]) / 3.0;
  const Complex negative =
      (phasors[0] + alpha * alpha * phasors[1] + alpha * phasors[2]) / 3.0;
  return 100.0 * std::abs(negative) / std::max(1e-12, std::abs(positive));
}

void write_physical_figure_rows(
    const ThreePhaseHybridOPFCase& problem,
    const ThreePhaseHybridOPFResult& solved,
    const PhysicalFigureMetadata& metadata,
    const std::string& mode,
    std::ostream& node_out,
    std::ostream& branch_out) {
  std::map<std::string, std::array<int, 3>> bus_nodes;
  for (int node = 0; node < problem.y_ac.rows(); ++node) {
    auto [it, inserted] = bus_nodes.try_emplace(
        metadata.phase_node_bus[static_cast<std::size_t>(node)],
        std::array<int, 3>{-1, -1, -1});
    (void)inserted;
    const int phase =
        metadata.phase_node_phase[static_cast<std::size_t>(node)];
    it->second[static_cast<std::size_t>(phase)] = node;
  }

  std::vector<int> node_converter(static_cast<std::size_t>(problem.y_ac.rows()), -1);
  std::vector<int> node_converter_phase(
      static_cast<std::size_t>(problem.y_ac.rows()), -1);
  std::vector<double> converter_i_vuf(problem.converters.size(),
                                      std::numeric_limits<double>::quiet_NaN());
  for (int ci = 0; ci < static_cast<int>(problem.converters.size()); ++ci) {
    const auto& converter = problem.converters[static_cast<std::size_t>(ci)];
    std::array<Complex, 3> phase_current{};
    bool has_all_phases = converter.phase_nodes.size() == 3;
    for (int local = 0; local < static_cast<int>(converter.phase_nodes.size()); ++local) {
      const int node = converter.phase_nodes[static_cast<std::size_t>(local)];
      const int phase = problem.ac_phase_index[static_cast<std::size_t>(node)];
      node_converter[static_cast<std::size_t>(node)] = ci;
      node_converter_phase[static_cast<std::size_t>(node)] = local;
      if (solved.converged &&
          ci < static_cast<int>(solved.converter_phase_power_pu.size()) &&
          local < static_cast<int>(
              solved.converter_phase_power_pu[static_cast<std::size_t>(ci)].size())) {
        const Complex voltage = solved.full_voltage[node];
        const Complex power = solved.converter_phase_power_pu[
            static_cast<std::size_t>(ci)][static_cast<std::size_t>(local)];
        phase_current[static_cast<std::size_t>(phase)] =
            std::abs(voltage) > 1e-12 ? std::conj(power / voltage) : Complex{};
      }
    }
    if (solved.converged && has_all_phases) {
      converter_i_vuf[static_cast<std::size_t>(ci)] =
          sequence_unbalance_percent(phase_current);
    }
  }

  for (const auto& [bus, nodes] : bus_nodes) {
    std::array<Complex, 3> voltage{};
    bool has_all_phases = true;
    for (int phase = 0; phase < 3; ++phase) {
      const int node = nodes[static_cast<std::size_t>(phase)];
      has_all_phases = has_all_phases && node >= 0;
      if (solved.converged && node >= 0) {
        voltage[static_cast<std::size_t>(phase)] = solved.full_voltage[node];
      }
    }
    const double voltage_vuf = solved.converged && has_all_phases
        ? sequence_unbalance_percent(voltage)
        : std::numeric_limits<double>::quiet_NaN();
    for (int phase = 0; phase < 3; ++phase) {
      const int node = nodes[static_cast<std::size_t>(phase)];
      if (node < 0) continue;
      const int ci = node_converter[static_cast<std::size_t>(node)];
      const int local = node_converter_phase[static_cast<std::size_t>(node)];
      Complex converter_power{};
      double current_magnitude = std::numeric_limits<double>::quiet_NaN();
      if (solved.converged && ci >= 0 && local >= 0) {
        converter_power = solved.converter_phase_power_pu[
            static_cast<std::size_t>(ci)][static_cast<std::size_t>(local)];
        current_magnitude = std::abs(converter_power) /
            std::max(1e-12, std::abs(solved.full_voltage[node]));
      }
      const bool is_reference = std::find(
          problem.reference_nodes.begin(), problem.reference_nodes.end(), node) !=
          problem.reference_nodes.end();
      node_out << problem.name.substr(0, problem.name.find('_')) << ',' << mode
               << ',' << (solved.converged ? "yes" : "no") << ',' << bus << ','
               << static_cast<char>('A' + phase) << ','
               << (solved.converged ? std::abs(solved.full_voltage[node])
                                    : std::numeric_limits<double>::quiet_NaN())
               << ',' << voltage_vuf << ',' << (is_reference ? "yes" : "no")
               << ',' << (ci >= 0 ? ci + 1 : 0) << ','
               << (ci >= 0 ? control_mode_name(
                                  problem.converters[static_cast<std::size_t>(ci)]
                                      .control_mode)
                            : "none")
               << ',' << converter_power.real() * problem.base_mva << ','
               << converter_power.imag() * problem.base_mva << ','
               << current_magnitude << ','
               << (ci >= 0 ? converter_i_vuf[static_cast<std::size_t>(ci)]
                           : std::numeric_limits<double>::quiet_NaN())
               << '\n';
    }
  }

  if (!solved.converged) return;
  std::map<std::pair<std::string, std::string>, PhysicalFigureMetadata::Edge>
      unique_edges;
  for (const auto& edge : metadata.edges) {
    const auto key = std::minmax(edge.from_bus, edge.to_bus);
    unique_edges.try_emplace(
        std::make_pair(key.first, key.second), edge);
  }
  const auto phase_powers = [&](const std::string& from,
                                const std::string& to) {
    std::array<double, 3> active{};
    const auto from_it = bus_nodes.find(from);
    const auto to_it = bus_nodes.find(to);
    if (from_it == bus_nodes.end() || to_it == bus_nodes.end()) return active;
    for (int phase = 0; phase < 3; ++phase) {
      const int from_node = from_it->second[static_cast<std::size_t>(phase)];
      if (from_node < 0) continue;
      Complex current{};
      for (int coupled_phase = 0; coupled_phase < 3; ++coupled_phase) {
        const int from_coupled =
            from_it->second[static_cast<std::size_t>(coupled_phase)];
        const int to_coupled =
            to_it->second[static_cast<std::size_t>(coupled_phase)];
        if (from_coupled < 0 || to_coupled < 0) continue;
        const Complex series = -problem.y_ac.coeff(from_node, to_coupled);
        current += series *
            (solved.full_voltage[from_coupled] - solved.full_voltage[to_coupled]);
      }
      active[static_cast<std::size_t>(phase)] =
          (solved.full_voltage[from_node] * std::conj(current)).real() *
          problem.base_mva;
    }
    return active;
  };
  for (const auto& [key, edge] : unique_edges) {
    std::string from = edge.from_bus;
    std::string to = edge.to_bus;
    auto active = phase_powers(from, to);
    double total = active[0] + active[1] + active[2];
    if (total < 0.0) {
      std::swap(from, to);
      active = phase_powers(from, to);
      total = active[0] + active[1] + active[2];
    }
    const double magnitude =
        std::abs(active[0]) + std::abs(active[1]) + std::abs(active[2]);
    if (magnitude <= 1e-10) continue;
    branch_out << problem.name.substr(0, problem.name.find('_')) << ',' << mode
               << ',' << from << ',' << to << ',' << edge.element << ','
               << active[0] << ','
               << active[1] << ',' << active[2] << ',' << total << ','
               << magnitude << '\n';
  }
}

void fill_phase_voltage_ranges(const ThreePhaseHybridOPFCase& problem,
                               const Eigen::VectorXcd& voltage,
                               SensitivityRow& row) {
  row.phase_vmin.fill(std::numeric_limits<double>::infinity());
  row.phase_vmax.fill(0.0);
  for (int node = 0; node < voltage.size(); ++node) {
    const int phase = problem.ac_phase_index[static_cast<std::size_t>(node)];
    const double magnitude = std::abs(voltage[node]);
    row.phase_vmin[static_cast<std::size_t>(phase)] =
        std::min(row.phase_vmin[static_cast<std::size_t>(phase)], magnitude);
    row.phase_vmax[static_cast<std::size_t>(phase)] =
        std::max(row.phase_vmax[static_cast<std::size_t>(phase)], magnitude);
  }
}

int run_converter_mode_suite(const Input& input, SolverBackend backend) {
  PhysicalFigureMetadata figure_metadata;
  const ThreePhaseHybridOPFCase base = hybridize(input, &figure_metadata);
  struct Variant {
    std::string label;
    ThreePhaseHybridOPFCase problem;
  };
  std::vector<Variant> variants;
  const auto add_uniform = [&](const std::string& label,
                               PhaseVSCControlMode mode) {
    ThreePhaseHybridOPFCase problem = base;
    for (auto& converter : problem.converters) {
      converter.control_mode = converter.phase_nodes.size() == 3
          ? mode : PhaseVSCControlMode::EqualPhasePower;
    }
    problem.name += "_" + label;
    variants.push_back({label, std::move(problem)});
  };
  add_uniform("equal_phase_pq", PhaseVSCControlMode::EqualPhasePower);
  add_uniform("gfl_pll", PhaseVSCControlMode::GridFollowingPLL);
  if (input.label != "H8500") {
    add_uniform("gfm_droop", PhaseVSCControlMode::GridFormingDroop);
  }
  if (base.converters.size() > 1) variants.push_back({"mixed_gfl_gfm", base});

  ThreePhaseHybridOPFOptions options;
  options.backend = backend;
  options.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
  options.max_iterations = 500;
  options.tolerance = 1e-6;
  options.use_constraint_oracle = true;
  options.variant = ModelVariant::GraphReduced;
  options.reduction_options.max_front = 12;
  options.reduction_options.max_nnz_ratio = 2.0;

  const fs::path path = project_root() / "output" / "benchmarks" /
      ("paper_converter_modes_" + input.label + ".csv");
  const fs::path topology_path = project_root() / "output" / "benchmarks" /
      ("paper_converter_topology_" + input.label + ".csv");
  const fs::path node_path = project_root() / "output" / "benchmarks" /
      ("paper_converter_node_results_" + input.label + ".csv");
  const fs::path branch_path = project_root() / "output" / "benchmarks" /
      ("paper_converter_branch_results_" + input.label + ".csv");
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  std::ofstream topology_out(topology_path);
  std::ofstream node_out(node_path);
  std::ofstream branch_out(branch_path);
  if (!out || !topology_out || !node_out || !branch_out) {
    throw std::runtime_error("unable to open converter-mode output files");
  }
  out << "case,mode,converged,gfl_count,gfm_count,legacy_count,variables,"
         "equalities,inequalities,eliminated,oracle_rows,total_rows,iterations,"
         "runtime_ms,objective,primal_residual,dual_residual,complementarity,"
         "max_vuf,max_converter_current_vuf,max_converter_current_loading,"
         "max_dynamic_equilibrium_residual,status\n";
  out << std::setprecision(12);
  topology_out << "case,from_bus,to_bus,phases,element\n";
  for (const auto& edge : figure_metadata.edges) {
    topology_out << input.label << ',' << edge.from_bus << ',' << edge.to_bus
                 << ',' << edge.phases << ',' << edge.element << '\n';
  }
  node_out << "case,mode,converged,bus,phase,voltage_pu,voltage_vuf_pct,"
              "is_reference,converter_id,converter_control,p_mw,q_mvar,"
              "current_pu,converter_current_vuf_pct\n";
  branch_out << "case,mode,from_bus,to_bus,element,p_a_mw,p_b_mw,p_c_mw,"
                "p_total_mw,sum_abs_phase_p_mw\n";
  node_out << std::setprecision(12);
  branch_out << std::setprecision(12);
  for (const auto& variant : variants) {
    const auto solved = solve_three_phase_hybrid_opf(variant.problem, options);
    int gfl = 0;
    int gfm = 0;
    int legacy = 0;
    for (const auto& converter : variant.problem.converters) {
      if (converter.control_mode == PhaseVSCControlMode::GridFollowingPLL) {
        ++gfl;
      } else if (converter.control_mode ==
                 PhaseVSCControlMode::GridFormingDroop) {
        ++gfm;
      } else {
        ++legacy;
      }
    }
    out << input.label << ',' << variant.label << ','
        << (solved.converged ? "yes" : "no") << ',' << gfl << ',' << gfm
        << ',' << legacy << ',' << solved.variables << ',' << solved.equalities
        << ',' << solved.inequalities << ',' << solved.eliminated_phase_nodes
        << ',' << solved.enforced_inequalities << ',' << solved.inequalities
        << ',' << solved.iterations << ',' << solved.runtime_ms << ','
        << solved.objective << ',' << solved.primal_residual << ','
        << solved.dual_residual << ',' << solved.complementarity << ','
        << solved.max_vuf << ',' << solved.max_converter_current_vuf << ','
        << solved.max_converter_current_loading << ','
        << solved.max_dynamic_equilibrium_residual << ",\""
        << solved.status << "\"\n" << std::flush;
    write_physical_figure_rows(variant.problem, solved, figure_metadata,
                               variant.label, node_out, branch_out);
    std::cout << input.label << '/' << variant.label
              << ": converged=" << solved.converged
              << ", runtime_ms=" << solved.runtime_ms
              << ", dynamic_residual="
              << solved.max_dynamic_equilibrium_residual << '\n';
  }
  std::cout << "Wrote " << path << '\n';
  std::cout << "Wrote " << topology_path << '\n';
  std::cout << "Wrote " << node_path << '\n';
  std::cout << "Wrote " << branch_path << '\n';
  return 0;
}

int run_opf_pf_crosscheck_suite(const std::vector<Input>& selected,
                                SolverBackend backend) {
  const fs::path path = project_root() / "output" / "benchmarks" /
      "paper_opf_pf_crosscheck.csv";
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,opf_converged,pf_converged,phase_nodes,dc_nodes,converters,"
         "opf_runtime_ms,pf_runtime_ms,pf_iterations,pf_residual,"
         "max_voltage_error_pu,max_dc_voltage_error_pu,"
         "max_converter_total_power_error_pu,status\n";
  out << std::setprecision(12);
  for (const auto& input : selected) {
    const auto problem = hybridize(input);
    ThreePhaseHybridOPFOptions opf_options;
    opf_options.backend = backend;
    opf_options.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
    opf_options.max_iterations = 500;
    opf_options.tolerance = 1e-6;
    opf_options.use_constraint_oracle = true;
    opf_options.variant = ModelVariant::GraphReduced;
    opf_options.reduction_options.max_front = 12;
    opf_options.reduction_options.max_nnz_ratio = 2.0;
    const auto opf = solve_three_phase_hybrid_opf(problem, opf_options);
    hacdcpf::powerflow::ThreePhaseHybridPFResult pf;
    double voltage_error = std::numeric_limits<double>::quiet_NaN();
    double dc_error = std::numeric_limits<double>::quiet_NaN();
    double power_error = std::numeric_limits<double>::quiet_NaN();
    if (opf.converged) {
      const auto pf_case = make_three_phase_hybrid_pf_case(problem, opf);
      hacdcpf::powerflow::ThreePhaseHybridPFOptions pf_options;
      pf_options.max_iterations = 100;
      pf_options.tolerance = 1e-8;
      pf = hacdcpf::powerflow::solve_three_phase_hybrid_pf(pf_case, pf_options);
      if (pf.converged) {
        voltage_error = max_voltage_difference(opf.full_voltage, pf.voltage);
        dc_error = (opf.dc_voltage - pf.dc_voltage).cwiseAbs().maxCoeff();
        power_error = 0.0;
        for (int ci = 0; ci < static_cast<int>(pf.converters.size()); ++ci) {
          Complex opf_total{0.0, 0.0};
          Complex pf_total{0.0, 0.0};
          for (const Complex value :
               opf.converter_phase_power_pu[static_cast<std::size_t>(ci)]) {
            opf_total += value;
          }
          for (const Complex value :
               pf.converters[static_cast<std::size_t>(ci)].phase_power_pu) {
            pf_total += value;
          }
          power_error = std::max(power_error, std::abs(opf_total - pf_total));
        }
      }
    }
    const std::string status = !opf.converged ? opf.status : pf.status;
    out << input.label << ',' << (opf.converged ? "yes" : "no") << ','
        << (pf.converged ? "yes" : "no") << ',' << problem.y_ac.rows() << ','
        << problem.g_dc.rows() << ',' << problem.converters.size() << ','
        << opf.runtime_ms << ',' << pf.runtime_ms << ',' << pf.iterations << ','
        << pf.residual << ',' << voltage_error << ',' << dc_error << ','
        << power_error << ",\"" << status << "\"\n" << std::flush;
    std::cout << input.label << " OPF/PF: opf=" << opf.converged
              << ", pf=" << pf.converged << ", dV=" << voltage_error
              << ", dVdc=" << dc_error << ", dS=" << power_error << '\n';
  }
  std::cout << "Wrote " << path << '\n';
  return 0;
}

int run_sensitivity_suite(const Input& input,
                          SolverBackend backend,
                          int repeats) {
  const ThreePhaseHybridOPFCase base = hybridize(input);
  const auto scenarios = make_sensitivity_scenarios(base);
  ThreePhaseHybridOPFOptions options;
  options.backend = backend;
  options.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
  options.max_iterations = 500;
  options.tolerance = 1e-6;
  options.use_constraint_oracle = true;
  options.variant = ModelVariant::GraphReduced;
  options.reduction_options.max_front = 12;
  options.reduction_options.max_nnz_ratio = 2.0;

  const fs::path trace_path = project_root() / "output" / "benchmarks" /
      "paper_oracle_sensitivity_trace_raw.csv";
  const fs::path raw_path = project_root() / "output" / "benchmarks" /
      "paper_oracle_sensitivity_raw.csv";
  fs::create_directories(trace_path.parent_path());
  std::ofstream trace(trace_path);
  trace << "repeat,scenario,round,enforced_rows,added_rows,iterations,"
           "restricted_converged,runtime_ms,max_full_inequality\n";
  trace << std::setprecision(12);
  std::ofstream out(raw_path);
  out << "repeat,scenario,continuation_converged,reference_converged,iterations,"
         "oracle_rounds,added_rows,enforced_rows,total_rows,continuation_ms,"
         "reference_ms,speedup,objective_error,voltage_error,primal_residual,"
         "dual_residual,complementarity,max_vuf,phase_load_unbalance,"
         "max_converter_current_vuf,max_converter_current_loading,"
         "max_dynamic_equilibrium_residual,va_min,vb_min,vc_min,"
         "va_max,vb_max,vc_max\n";
  out << std::setprecision(12);

  for (int repeat = 1; repeat <= repeats; ++repeat) {
    const auto base_result = solve_three_phase_hybrid_opf(base, options);
    for (std::size_t index = 0; index < scenarios.size(); ++index) {
      ThreePhaseHybridOPFResult solved;
      if (index == 0) {
        solved = base_result;
      } else {
        solved = solve_three_phase_hybrid_opf_branches(
            base, base_result, {scenarios[index].problem}, options).front();
      }
      ThreePhaseHybridOPFOptions reference_options = options;
      reference_options.use_constraint_oracle = false;
      const auto reference = solve_three_phase_hybrid_opf(
          scenarios[index].problem, reference_options);
      SensitivityRow row;
      row.scenario = scenarios[index].label;
      row.repeat = repeat;
      row.continuation_converged = solved.converged;
      row.reference_converged = reference.converged;
      row.iterations = solved.iterations;
      row.oracle_rounds = solved.constraint_oracle_rounds;
      row.added_rows = solved.constraint_oracle_added_rows;
      row.enforced_rows = solved.enforced_inequalities;
      row.total_rows = solved.inequalities;
      row.continuation_ms = solved.runtime_ms;
      row.reference_ms = reference.runtime_ms;
      row.objective_error = std::abs(solved.objective - reference.objective) /
          std::max(1.0, std::abs(reference.objective));
      row.voltage_error = max_voltage_difference(
          solved.full_voltage, reference.full_voltage);
      row.primal_residual = solved.primal_residual;
      row.dual_residual = solved.dual_residual;
      row.complementarity = solved.complementarity;
      row.max_vuf = solved.max_vuf;
      row.phase_load_unbalance = phase_load_unbalance(scenarios[index].problem);
      row.max_converter_current_vuf = solved.max_converter_current_vuf;
      row.max_converter_current_loading =
          solved.max_converter_current_loading;
      row.max_dynamic_equilibrium_residual =
          solved.max_dynamic_equilibrium_residual;
      fill_phase_voltage_ranges(
          scenarios[index].problem, solved.full_voltage, row);
      out << row.repeat << ',' << row.scenario << ','
          << (row.continuation_converged ? "yes" : "no") << ','
          << (row.reference_converged ? "yes" : "no") << ','
          << row.iterations << ',' << row.oracle_rounds << ',' << row.added_rows << ','
          << row.enforced_rows << ',' << row.total_rows << ','
          << row.continuation_ms << ',' << row.reference_ms << ','
          << row.reference_ms / std::max(1e-12, row.continuation_ms) << ','
          << row.objective_error << ',' << row.voltage_error << ','
          << row.primal_residual << ',' << row.dual_residual << ','
          << row.complementarity << ',' << row.max_vuf << ','
          << row.phase_load_unbalance << ',' << row.max_converter_current_vuf
          << ',' << row.max_converter_current_loading << ','
          << row.max_dynamic_equilibrium_residual;
      for (double value : row.phase_vmin) out << ',' << value;
      for (double value : row.phase_vmax) out << ',' << value;
      out << '\n' << std::flush;
      for (const auto& round : solved.constraint_oracle_trace) {
        trace << repeat << ',' << row.scenario << ',' << round.round << ','
              << round.enforced_rows << ',' << round.added_rows << ','
              << round.iterations << ','
              << (round.restricted_converged ? "yes" : "no") << ','
              << round.runtime_ms << ',' << round.max_full_inequality << '\n';
      }
      trace.flush();
      std::cout << "repeat=" << repeat << ", scenario=" << row.scenario
                << ", continuation=" << row.continuation_converged
                << '/' << row.continuation_ms << " ms, reference="
                << row.reference_converged << '/' << row.reference_ms
                << " ms, added=" << row.added_rows << '\n' << std::flush;
    }
  }
  std::cout << "Wrote " << raw_path << "\nWrote " << trace_path << '\n';
  return 0;
}

Audit run_case(const Input& input,
               SolverBackend backend,
               bool solve_full,
               bool matched_full_start,
               bool build_only,
               bool use_constraint_oracle,
               bool run_continuation,
               bool verbose,
               std::vector<DriftTraceRow>* drift_trace) {
  Audit audit;
  audit.label = input.label;
  audit.requested_dc_share = input.dc_share;
  try {
    std::cerr << '[' << input.label << "] hybridization: start\n" << std::flush;
    const auto build_start = std::chrono::steady_clock::now();
    const ThreePhaseHybridOPFCase c = hybridize(input);
    std::cerr << '[' << input.label << "] hybridization: done in "
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - build_start).count()
              << " s\n" << std::flush;
    audit.built = true;
    audit.phase_nodes = static_cast<int>(c.y_ac.rows());
    audit.dc_nodes = static_cast<int>(c.g_dc.rows());
    audit.vscs = static_cast<int>(c.converters.size());
    const double ac_p = c.p_load_pu.sum();
    const double dc_p = c.p_dc_load_pu.sum();
    audit.realized_dc_share = dc_p / std::max(1e-12, ac_p + dc_p);
    if (build_only) return audit;

    ThreePhaseHybridOPFOptions options;
    options.backend = backend;
    options.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
    options.verify_derivatives = input.label == "H13" && !run_continuation;
    options.verbose = verbose;
    options.max_iterations = 500;
    options.tolerance = 1e-6;
    options.use_constraint_oracle = use_constraint_oracle;
    options.variant = ModelVariant::GraphReduced;
    options.reduction_options.max_front = 12;
    options.reduction_options.max_nnz_ratio = 2.0;
    std::cerr << '[' << input.label << "] GR-IPM: start\n" << std::flush;
    if (run_continuation) {
      ThreePhaseHybridOPFCase continued_case = c;
      continued_case.p_load_pu *= 1.001;
      continued_case.q_load_pu *= 1.001;
      continued_case.p_dc_load_pu *= 1.001;
      auto sequence = solve_three_phase_hybrid_opf_sequence(
          {c, continued_case}, options);
      audit.reduced = std::move(sequence[0]);
      audit.continuation = std::move(sequence[1]);
      ThreePhaseHybridOPFOptions reference_options = options;
      reference_options.use_constraint_oracle = false;
      audit.continuation_reference = solve_three_phase_hybrid_opf(
          continued_case, reference_options);
      audit.continuation_objective_relative_error =
          std::abs(audit.continuation.objective -
                   audit.continuation_reference.objective) /
          std::max(1.0, std::abs(audit.continuation_reference.objective));
      audit.continuation_voltage_error = max_voltage_difference(
          audit.continuation.full_voltage,
          audit.continuation_reference.full_voltage);
    } else {
      audit.reduced = solve_three_phase_hybrid_opf(c, options);
    }
    std::cerr << '[' << input.label << "] GR-IPM: done in "
              << audit.reduced.runtime_ms / 1000.0 << " s, status="
              << audit.reduced.status << "\n" << std::flush;
    if (run_continuation) {
      std::cerr << '[' << input.label << "] continuation GR-IPM: done in "
                << audit.continuation.runtime_ms / 1000.0 << " s, status="
                << audit.continuation.status << ", phase2-start="
                << audit.continuation.phase_two_start_requested << '/'
                << audit.continuation.phase_two_start_accepted
                << ", centrality=" << audit.continuation.phase_one_centrality
                << ", mu=" << audit.continuation.phase_one_barrier_mu;
      if (!audit.continuation.phase_two_start_rejection_reason.empty()) {
        std::cerr << " ("
                  << audit.continuation.phase_two_start_rejection_reason << ')';
      }
      std::cerr << "\n" << std::flush;
    }
    if (!solve_full) return audit;
    ThreePhaseHybridOPFCase full_case = c;
    options.variant = ModelVariant::Full;
    const int ng = static_cast<int>(c.generators.size());
    const int full_nv = static_cast<int>(c.y_ac.rows());
    options.primal_start.resize(0);
    options.equality_dual_start.resize(0);
    options.nonlinear_inequality_dual_start.resize(0);
    options.nonlinear_slack_start.resize(0);
    if (matched_full_start && audit.reduced.converged) {
      const int ndc = static_cast<int>(c.g_dc.rows());
      int ncp = 0;
      int ngfm = 0;
      for (const auto& converter : c.converters) {
        ncp += static_cast<int>(converter.phase_nodes.size());
        if (converter.control_mode == PhaseVSCControlMode::GridFormingDroop) {
          ++ngfm;
        }
      }
      const int nc = static_cast<int>(c.converters.size());
      const int reduced_nv =
          static_cast<int>(audit.reduced.reduction.retained.size());
      const int device_variables =
          2 * ng + ndc + 2 * ncp + 2 * ngfm + nc;
      if (audit.reduced.primal.size() == 2 * reduced_nv + device_variables &&
          audit.reduced.full_voltage.size() == full_nv) {
        full_case.voltage_start = audit.reduced.full_voltage;
        options.primal_start =
            Eigen::VectorXd::Zero(2 * full_nv + device_variables);
        for (int node = 0; node < full_nv; ++node) {
          options.primal_start[node] =
              std::real(audit.reduced.full_voltage[node]);
          options.primal_start[full_nv + node] =
              std::imag(audit.reduced.full_voltage[node]);
        }
        options.primal_start.tail(device_variables) =
            audit.reduced.primal.tail(device_variables);
      }
      if (audit.reduced.equality_dual.size() == audit.reduced.equalities) {
        const int full_equalities =
            audit.reduced.equalities + 2 * (full_nv - reduced_nv);
        options.equality_dual_start = Eigen::VectorXd::Constant(
            full_equalities, std::numeric_limits<double>::quiet_NaN());
        for (int pos = 0; pos < reduced_nv; ++pos) {
          const int node =
              audit.reduced.reduction.retained[static_cast<std::size_t>(pos)];
          options.equality_dual_start[node] = audit.reduced.equality_dual[pos];
          options.equality_dual_start[full_nv + node] =
              audit.reduced.equality_dual[reduced_nv + pos];
        }
        const int tail = audit.reduced.equalities - 2 * reduced_nv;
        options.equality_dual_start.tail(tail) =
            audit.reduced.equality_dual.tail(tail);
      }
      if (audit.reduced.inequality_dual.size() >= audit.reduced.inequalities) {
        options.nonlinear_inequality_dual_start =
            audit.reduced.inequality_dual.head(audit.reduced.inequalities);
      }
    }
    std::cerr << '[' << input.label << "] Full-IPM: start\n" << std::flush;
    audit.full = solve_three_phase_hybrid_opf(full_case, options);
    std::cerr << '[' << input.label << "] Full-IPM: done in "
              << audit.full.runtime_ms / 1000.0 << " s, status="
              << audit.full.status << "\n" << std::flush;
    if (drift_trace != nullptr) {
      const std::vector<int> caps{1, 2, 5, 10, 20, 50, 100, 200, 500};
      for (int cap : caps) {
        ThreePhaseHybridOPFOptions trace_options = options;
        trace_options.max_iterations = cap;
        trace_options.verify_derivatives = false;
        DriftTraceRow row;
        row.iteration_cap = cap;
        row.result = solve_three_phase_hybrid_opf(full_case, trace_options);
        if (row.result.primal.size() >= 2 * full_nv + ng) {
          row.min_phase_generation_pu =
              row.result.primal.segment(2 * full_nv, ng).minCoeff();
          row.max_phase_generation_pu =
              row.result.primal.segment(2 * full_nv, ng).maxCoeff();
        }
        drift_trace->push_back(std::move(row));
      }
    }

    audit.objective_relative_error =
        std::abs(audit.full.objective - audit.reduced.objective) /
        std::max(1.0, std::abs(audit.full.objective));
    audit.voltage_error = max_voltage_difference(
        audit.full.full_voltage, audit.reduced.full_voltage);
    if (audit.reduced.runtime_ms > 0.0) {
      audit.speedup = audit.full.runtime_ms / audit.reduced.runtime_ms;
    }
  } catch (const std::exception& ex) {
    audit.error = ex.what();
  }
  return audit;
}

void write_drift_trace(const fs::path& path,
                       const std::vector<DriftTraceRow>& rows) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "iteration_cap,solver_success,objective,physical_primal_residual,"
         "initial_physical_residual,scaled_dual_residual,complementarity,"
         "min_phase_generation_pu,max_phase_generation_pu,voltage_violation,"
         "vuf,runtime_ms,status\n";
  out << std::setprecision(12);
  for (const auto& row : rows) {
    const auto& r = row.result;
    out << row.iteration_cap << ',' << (r.converged ? "yes" : "no") << ','
        << r.objective << ',' << r.primal_residual << ','
        << r.initial_primal_residual << ',' << r.dual_residual << ','
        << r.complementarity << ',' << row.min_phase_generation_pu << ','
        << row.max_phase_generation_pu << ',' << r.max_voltage_violation << ','
        << r.max_vuf << ',' << r.runtime_ms << ",\"" << r.status << "\"\n";
  }
}

void write_csv(const fs::path& path, const std::vector<Audit>& audits) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,built,phase_nodes,dc_nodes,vscs,requested_dc_share,realized_dc_share,"
         "full_converged,gr_converged,full_variables,gr_variables,full_equalities,"
         "gr_equalities,gr_eliminated,objective_relative_error,voltage_error,"
         "full_objective,gr_objective,full_primal,gr_primal,full_dual,gr_dual,"
         "full_complementarity,gr_complementarity,full_runtime_ms,gr_runtime_ms,speedup,"
         "full_enforced_inequalities,gr_enforced_inequalities,"
         "full_oracle_rounds,gr_oracle_rounds,full_oracle_added,gr_oracle_added,"
         "full_inequality_jacobian_nnz,gr_inequality_jacobian_nnz,"
         "full_max_omitted_inequality,gr_max_omitted_inequality,"
         "continuation_converged,continuation_runtime_ms,continuation_iterations,"
         "continuation_rows,continuation_rounds,continuation_added,"
         "continuation_primal,continuation_dual,continuation_complementarity,"
         "continuation_reference_converged,continuation_reference_runtime_ms,"
         "continuation_objective_relative_error,continuation_voltage_error,"
         "full_status,gr_status,error\n";
  out << std::setprecision(12);
  for (const auto& a : audits) {
    out << a.label << ',' << (a.built ? "yes" : "no") << ',' << a.phase_nodes << ','
        << a.dc_nodes << ',' << a.vscs << ',' << a.requested_dc_share << ','
        << a.realized_dc_share << ',' << (a.full.converged ? "yes" : "no") << ','
        << (a.reduced.converged ? "yes" : "no") << ',' << a.full.variables << ','
        << a.reduced.variables << ',' << a.full.equalities << ','
        << a.reduced.equalities << ',' << a.reduced.eliminated_phase_nodes << ','
        << a.objective_relative_error << ',' << a.voltage_error << ','
        << a.full.objective << ',' << a.reduced.objective << ','
        << a.full.primal_residual << ',' << a.reduced.primal_residual << ','
        << a.full.dual_residual << ',' << a.reduced.dual_residual << ','
        << a.full.complementarity << ',' << a.reduced.complementarity << ','
        << a.full.runtime_ms << ',' << a.reduced.runtime_ms << ',' << a.speedup << ','
        << a.full.enforced_inequalities << ','
        << a.reduced.enforced_inequalities << ','
        << a.full.constraint_oracle_rounds << ','
        << a.reduced.constraint_oracle_rounds << ','
        << a.full.constraint_oracle_added_rows << ','
        << a.reduced.constraint_oracle_added_rows << ','
        << a.full.inequality_jacobian_nonzeros << ','
        << a.reduced.inequality_jacobian_nonzeros << ','
        << a.full.max_omitted_inequality << ','
        << a.reduced.max_omitted_inequality << ','
        << (a.continuation.converged ? "yes" : "no") << ','
        << a.continuation.runtime_ms << ',' << a.continuation.iterations << ','
        << a.continuation.enforced_inequalities << ','
        << a.continuation.constraint_oracle_rounds << ','
        << a.continuation.constraint_oracle_added_rows << ','
        << a.continuation.primal_residual << ','
        << a.continuation.dual_residual << ','
        << a.continuation.complementarity << ','
        << (a.continuation_reference.converged ? "yes" : "no") << ','
        << a.continuation_reference.runtime_ms << ','
        << a.continuation_objective_relative_error << ','
        << a.continuation_voltage_error
        << ",\"" << a.full.status << "\",\"" << a.reduced.status
        << "\",\"" << a.error << "\"\n";
  }
}

}  // namespace

// ===================== Claims experiments (Exp 1 / Exp 2) ====================
// Variants compared on each feeder:
//   Full : unreduced network, all limits enforced (ground truth).
//   RN   : Kron-reduced, limits enforced ONLY at retained nodes (drops the
//          recovered eliminated-node voltage/VUF rows; current practice).
//   RA   : Kron-reduced, ALL recovered limits enforced (this work's model,
//          oracle disabled so every recovered row is assembled).
namespace {

// Enforced-inequality set for the RN variant: retained-node voltage rows,
// VUF rows of fully retained buses, and all converter rows.  Eliminated-node
// voltage and VUF rows are omitted.  Row layout (see build_model_data):
// [0,n) voltage lower, [n,2n) voltage upper, [2n,2n+n3) VUF, then converters.
std::vector<int> retained_enforced_rows(
    const ThreePhaseHybridOPFCase& c,
    const hacdcpf::graph::SparseKronResult& red) {
  const int full_n = red.original_size;
  const int n3 = static_cast<int>(c.three_phase_bus_nodes.size());
  std::vector<char> retained(static_cast<std::size_t>(full_n), 0);
  for (int node : red.retained) retained[static_cast<std::size_t>(node)] = 1;
  std::vector<int> rows;
  for (int i = 0; i < full_n; ++i) {
    if (retained[static_cast<std::size_t>(i)]) {
      rows.push_back(i);
      rows.push_back(full_n + i);
    }
  }
  for (int b = 0; b < n3; ++b) {
    bool all_retained = true;
    for (int nd : c.three_phase_bus_nodes[static_cast<std::size_t>(b)]) {
      if (!retained[static_cast<std::size_t>(nd)]) all_retained = false;
    }
    if (all_retained) rows.push_back(2 * full_n + b);
  }
  int ncp = 0;
  for (const auto& cv : c.converters) {
    ncp += static_cast<int>(cv.phase_nodes.size());
  }
  const int nc = static_cast<int>(c.converters.size());
  const int nineq = 2 * full_n + n3 + nc + ncp;
  for (int r = 2 * full_n + n3; r < nineq; ++r) rows.push_back(r);
  std::sort(rows.begin(), rows.end());
  return rows;
}

struct EliminatedViolation {
  int violated{0};
  int total{0};
  double max_overshoot{0.0};
  int line_violated{0};
  int line_total{0};
  double line_max_overshoot{0.0};
};

// Evaluate the omitted (eliminated-node) voltage and VUF limits at a recovered
// voltage vector and count how many are violated.
EliminatedViolation eliminated_violations(
    const ThreePhaseHybridOPFCase& c,
    const hacdcpf::graph::SparseKronResult& red,
    const Eigen::VectorXcd& vfull,
    double tol) {
  const int full_n = red.original_size;
  if (vfull.size() != full_n) return {};
  std::vector<char> retained(static_cast<std::size_t>(full_n), 0);
  for (int node : red.retained) retained[static_cast<std::size_t>(node)] = 1;
  EliminatedViolation s;
  for (int i = 0; i < full_n; ++i) {
    if (retained[static_cast<std::size_t>(i)]) continue;
    ++s.total;
    const double vm2 = std::norm(vfull[i]);
    const double h = std::max(c.v_min_pu[i] * c.v_min_pu[i] - vm2,
                              vm2 - c.v_max_pu[i] * c.v_max_pu[i]);
    if (h > tol) {
      ++s.violated;
      s.max_overshoot = std::max(s.max_overshoot, h);
    }
  }
  const Complex a = std::polar(1.0, 2.0 * M_PI / 3.0);
  const Complex a2 = a * a;
  for (const auto& tri : c.three_phase_bus_nodes) {
    bool any_eliminated = false;
    for (int nd : tri) {
      if (!retained[static_cast<std::size_t>(nd)]) any_eliminated = true;
    }
    if (!any_eliminated) continue;
    ++s.total;
    const Complex v1 =
        (vfull[tri[0]] + a * vfull[tri[1]] + a2 * vfull[tri[2]]) / 3.0;
    const Complex v2 =
        (vfull[tri[0]] + a2 * vfull[tri[1]] + a * vfull[tri[2]]) / 3.0;
    const double h = std::norm(v2) - c.vuf_max * c.vuf_max * std::norm(v1);
    if (h > tol) {
      ++s.violated;
      s.max_overshoot = std::max(s.max_overshoot, h);
    }
  }
  // Recovered line-current rows incident to an eliminated node: count how many
  // would be violated when omitted, i.e., how many would bind at this point.
  for (const auto& limit : c.ac_line_limits) {
    bool any_eliminated = false;
    for (int nd : limit.nodes) {
      if (!retained[static_cast<std::size_t>(nd)]) any_eliminated = true;
    }
    if (!any_eliminated) continue;
    ++s.line_total;
    Complex i_line{0.0, 0.0};
    for (std::size_t k = 0; k < limit.nodes.size(); ++k) {
      i_line += limit.coefficients[k] *
                vfull[limit.nodes[static_cast<std::size_t>(k)]];
    }
    const double h = std::norm(i_line) - limit.i_max_pu * limit.i_max_pu;
    if (h > tol) {
      ++s.line_violated;
      s.line_max_overshoot = std::max(s.line_max_overshoot, h);
    }
  }
  return s;
}

// Scale every load by lambda and place a fraction pi of each three-phase bus
// load on phase a (the remainder split over b and c) to control unbalance.
ThreePhaseHybridOPFCase scale_loads(ThreePhaseHybridOPFCase c,
                                    double lambda, double pi) {
  c.p_load_pu *= lambda;
  c.q_load_pu *= lambda;
  const double w[3] = {pi, 0.5 * (1.0 - pi), 0.5 * (1.0 - pi)};
  for (const auto& tri : c.three_phase_bus_nodes) {
    double pt = 0.0, qt = 0.0;
    for (int nd : tri) {
      pt += c.p_load_pu[nd];
      qt += c.q_load_pu[nd];
    }
    for (int j = 0; j < 3; ++j) {
      c.p_load_pu[tri[static_cast<std::size_t>(j)]] = w[j] * pt;
      c.q_load_pu[tri[static_cast<std::size_t>(j)]] = w[j] * qt;
    }
  }
  return c;
}

void set_all_converters_gfl(ThreePhaseHybridOPFCase& c) {
  for (auto& cv : c.converters) {
    if (cv.phase_nodes.size() == 3) {
      cv.control_mode = PhaseVSCControlMode::GridFollowingPLL;
    }
  }
}

// Exp 1 (Claim B): omission of recovered limits leaves undetected violations.
int run_exp1_suite(const Input& input, SolverBackend backend) {
  ThreePhaseHybridOPFCase base = hybridize(input);
  set_all_converters_gfl(base);
  const bool big = input.label == "H8500";
  // On the smaller feeders, enforce the ANSI C84.1 Range A service band
  // (0.95--1.05 p.u.) so that the recovered eliminated-node limits are
  // operationally binding; the largest feeder is left at its native limits.
  if (!big) {
    for (int i = 0; i < base.v_min_pu.size(); ++i) {
      base.v_min_pu[i] = 0.95;
      base.v_max_pu[i] = 1.05;
    }
  }
  const std::vector<double> lambdas = big
      ? std::vector<double>{1.0, 1.2}
      : std::vector<double>{0.6, 0.8, 1.0, 1.2, 1.4};
  const std::vector<double> pis = big
      ? std::vector<double>{0.3, 0.5}
      : std::vector<double>{0.1, 0.3, 0.5, 0.7, 0.9};
  const fs::path path = project_root() / "output" / "benchmarks" /
      ("paper_exp1_" + input.label + ".csv");
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,lambda,pi,full_conv,rn_conv,ra_conv,elim_rows,elim_violated,"
         "violation_rate,max_overshoot,line_violated,line_total,"
         "line_max_overshoot,full_obj,rn_obj,obj_gap_rel\n";
  out << std::setprecision(10);
  ThreePhaseHybridOPFOptions base_opts;
  base_opts.backend = backend;
  base_opts.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
  base_opts.max_iterations = 500;
  base_opts.tolerance = 1e-6;
  base_opts.reduction_options.max_front = 12;
  base_opts.reduction_options.max_nnz_ratio = 4.0;
  for (double lambda : lambdas) {
    for (double pi : pis) {
      const ThreePhaseHybridOPFCase prob = scale_loads(base, lambda, pi);
      ThreePhaseHybridOPFOptions ra_opts = base_opts;
      ra_opts.variant = ModelVariant::GraphReduced;
      const auto ra = solve_three_phase_hybrid_opf(prob, ra_opts);
      // The exact all-row reduced model (RA) is the feasible reference; on the
      // largest feeder the unreduced Full solve is skipped for tractability.
      ThreePhaseHybridOPFResult full;
      if (!big) {
        ThreePhaseHybridOPFOptions full_opts = base_opts;
        full_opts.variant = ModelVariant::Full;
        full = solve_three_phase_hybrid_opf(prob, full_opts);
      }
      const auto rn_rows = retained_enforced_rows(prob, ra.reduction);
      ThreePhaseHybridOPFOptions rn_opts = ra_opts;
      rn_opts.enforced_inequality_rows = rn_rows;
      // Warm-start RN from the feasible all-row (RA) point so that the solver
      // reaches RN's relaxed optimum rather than stalling; any violation of the
      // dropped eliminated-node limits is then a converged-point property.
      if (ra.converged && ra.primal.size() > 0) {
        rn_opts.primal_start = ra.primal;
      }
      const auto rn = solve_three_phase_hybrid_opf(prob, rn_opts);
      const EliminatedViolation v =
          eliminated_violations(prob, ra.reduction, rn.full_voltage, 1e-6);
      const double rate =
          v.total > 0 ? static_cast<double>(v.violated) / v.total : 0.0;
      const double ref_obj = big ? ra.objective : full.objective;
      const double gap = std::abs(ref_obj) > 1e-9
          ? (ref_obj - rn.objective) / std::abs(ref_obj) : 0.0;
      out << input.label << ',' << lambda << ',' << pi << ','
          << (full.converged ? 1 : 0) << ',' << (rn.converged ? 1 : 0) << ','
          << (ra.converged ? 1 : 0) << ',' << v.total << ',' << v.violated
          << ',' << rate << ',' << v.max_overshoot << ',' << v.line_violated
          << ',' << v.line_total << ',' << v.line_max_overshoot << ','
          << full.objective << ',' << rn.objective << ',' << gap
          << '\n' << std::flush;
      std::cout << input.label << " exp1 lambda=" << lambda << " pi=" << pi
                << " viol_rate=" << rate << " max_over=" << v.max_overshoot
                << " line_viol=" << v.line_violated << '/' << v.line_total
                << " line_over=" << v.line_max_overshoot
                << " gap=" << gap << " ra/rn=" << ra.converged << '/'
                << rn.converged << '\n' << std::flush;
    }
  }
  std::cout << "Wrote " << path << '\n';
  return 0;
}

// Exp 2 (Claim A): enforcing all recovered limits densifies the KKT system as
// the elimination ratio grows.  max_front sweeps the elimination ratio.
int run_exp2_suite(const Input& input, SolverBackend backend) {
  ThreePhaseHybridOPFCase base = hybridize(input);
  set_all_converters_gfl(base);
  // nnz(J_h) is a structural quantity; on the largest feeder a minimal solve
  // suffices to populate it, so the iteration budget is capped there.
  const bool big = input.label == "H8500";
  const int iters = big ? 2 : 500;
  const std::vector<int> fronts = {1, 2, 3, 5, 8, 12, 20};
  const fs::path path = project_root() / "output" / "benchmarks" /
      ("paper_exp2_" + input.label + ".csv");
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,max_front,rho,eliminated,original,full_nnzJ,rn_nnzJ,ra_nnzJ,"
         "full_var,rn_var,ra_var,full_ms,rn_ms,ra_ms,"
         "full_conv,rn_conv,ra_conv\n";
  out << std::setprecision(10);
  ThreePhaseHybridOPFOptions full_opts;
  full_opts.backend = backend;
  full_opts.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
  full_opts.variant = ModelVariant::Full;
  full_opts.max_iterations = iters;
  full_opts.tolerance = 1e-6;
  if (big) full_opts.phase_one_time_limit_ms = 1000.0;
  const auto full = solve_three_phase_hybrid_opf(base, full_opts);
  for (int mf : fronts) {
    ThreePhaseHybridOPFOptions ra_opts;
    ra_opts.backend = backend;
    ra_opts.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
    ra_opts.variant = ModelVariant::GraphReduced;
    ra_opts.max_iterations = iters;
    ra_opts.tolerance = 1e-6;
    if (big) ra_opts.phase_one_time_limit_ms = 1000.0;
    ra_opts.reduction_options.max_front = mf;
    ra_opts.reduction_options.max_nnz_ratio = 50.0;
    const auto ra = solve_three_phase_hybrid_opf(base, ra_opts);
    const double rho = ra.reduction.original_size > 0
        ? static_cast<double>(ra.eliminated_phase_nodes) /
              ra.reduction.original_size
        : 0.0;
    const auto rn_rows = retained_enforced_rows(base, ra.reduction);
    ThreePhaseHybridOPFOptions rn_opts = ra_opts;
    rn_opts.enforced_inequality_rows = rn_rows;
    const auto rn = solve_three_phase_hybrid_opf(base, rn_opts);
    out << input.label << ',' << mf << ',' << rho << ','
        << ra.eliminated_phase_nodes << ',' << ra.reduction.original_size << ','
        << full.inequality_jacobian_nonzeros << ','
        << rn.inequality_jacobian_nonzeros << ','
        << ra.inequality_jacobian_nonzeros << ',' << full.variables << ','
        << rn.variables << ',' << ra.variables << ',' << full.runtime_ms << ','
        << rn.runtime_ms << ',' << ra.runtime_ms << ','
        << (full.converged ? 1 : 0) << ',' << (rn.converged ? 1 : 0) << ','
        << (ra.converged ? 1 : 0) << '\n' << std::flush;
    std::cout << input.label << " exp2 mf=" << mf << " rho=" << rho
              << " nnzJ full/rn/ra=" << full.inequality_jacobian_nonzeros << '/'
              << rn.inequality_jacobian_nonzeros << '/'
              << ra.inequality_jacobian_nonzeros << " ra_ms=" << ra.runtime_ms
              << '\n' << std::flush;
  }
  std::cout << "Wrote " << path << '\n';
  return 0;
}

int run_crosssolver_suite(const std::vector<Input>& selected) {
  // The acceptance threshold matches the OPF feasibility tolerance used in the
  // manuscript validation protocol; see docs/modules/optimal_power_flow/
  // chapters/three_phase.tex, "Independent solver cross-check".
  constexpr double kAgreementTolerance = 1e-6;
  constexpr double kSolverTolerance = 1e-6;
  const fs::path path = project_root() / "output" / "benchmarks" /
      "paper_native_ipm_ipopt_crosscheck.csv";
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,native_converged,ipopt_cold_converged,ipopt_verification_converged,"
         "native_solver,ipopt_solver,"
         "native_ipopt_initialization,native_objective,ipopt_objective,"
         "ipopt_verification_objective,cold_objective_relative_difference,"
         "cold_voltage_max_difference_pu,verification_objective_relative_difference,"
         "verification_voltage_max_difference_pu,native_primal_residual,"
         "ipopt_primal_residual,ipopt_verification_primal_residual,"
         "native_dual_residual,ipopt_dual_residual,native_complementarity,"
         "ipopt_complementarity,"
         "native_screen_rows,ipopt_screen_rows,common_enforced_rows,total_rows,"
         "native_max_omitted_inequality,ipopt_max_omitted_inequality,"
         "passes,native_status,ipopt_status,ipopt_verification_status\n";
  out << std::setprecision(12);

  bool all_cases_pass = true;
  for (const auto& input : selected) {
    ThreePhaseHybridOPFResult native;
    ThreePhaseHybridOPFResult ipopt;
    ThreePhaseHybridOPFResult ipopt_verification;
    int native_screen_rows = 0;
    int ipopt_screen_rows = 0;
    int common_enforced_rows = 0;
    double objective_difference = std::numeric_limits<double>::infinity();
    double voltage_difference = std::numeric_limits<double>::infinity();
    double verification_objective_difference =
        std::numeric_limits<double>::infinity();
    double verification_voltage_difference =
        std::numeric_limits<double>::infinity();
    try {
      const ThreePhaseHybridOPFCase problem = hybridize(input);
      ThreePhaseHybridOPFOptions native_options;
      native_options.backend = SolverBackend::NativeIPM;
      // This is the independence condition for the external-solver check.
      // The Native solve must not call the optional Ipopt Phase-I initializer.
      native_options.warm_start_with_ipopt = false;
      native_options.variant = ModelVariant::GraphReduced;
      native_options.max_iterations = 1000;
      native_options.tolerance = kSolverTolerance;
      native_options.use_constraint_oracle = true;
      native_options.reduction_options.max_front = 12;
      native_options.reduction_options.max_nnz_ratio = 2.0;

      ThreePhaseHybridOPFOptions ipopt_options = native_options;
      ipopt_options.backend = SolverBackend::Ipopt;

      const ThreePhaseHybridOPFResult native_screen =
          solve_three_phase_hybrid_opf(problem, native_options);
      const ThreePhaseHybridOPFResult ipopt_screen =
          solve_three_phase_hybrid_opf(problem, ipopt_options);
      native_screen_rows = native_screen.enforced_inequalities;
      ipopt_screen_rows = ipopt_screen.enforced_inequalities;
      if (!native_screen.converged || !ipopt_screen.converged) {
        native = native_screen;
        ipopt = ipopt_screen;
      } else {
        // Use the union of the independently screened Native and Ipopt rows
        // as one fixed restricted NLP. Both final solves also evaluate every
        // omitted physical inequality, so screening cannot hide a violation.
        std::vector<int> common_rows = native_screen.enforced_inequality_rows;
        common_rows.insert(common_rows.end(),
                           ipopt_screen.enforced_inequality_rows.begin(),
                           ipopt_screen.enforced_inequality_rows.end());
        std::sort(common_rows.begin(), common_rows.end());
        common_rows.erase(std::unique(common_rows.begin(), common_rows.end()),
                          common_rows.end());
        common_enforced_rows = static_cast<int>(common_rows.size());
        native_options.use_constraint_oracle = false;
        native_options.enforced_inequality_rows = common_rows;
        ipopt_options.use_constraint_oracle = false;
        ipopt_options.enforced_inequality_rows = common_rows;
        native = solve_three_phase_hybrid_opf(problem, native_options);
        ipopt = solve_three_phase_hybrid_opf(problem, ipopt_options);
        if (native.converged) {
          // External verification of the Native local solution: the current
          // Ipopt adapter receives only the Native primal point. It receives no
          // equality/inequality multipliers or nonlinear slacks and then runs
          // Ipopt to its own convergence test on the identical restricted NLP.
          ipopt_options.primal_start = native.primal;
          ipopt_verification =
              solve_three_phase_hybrid_opf(problem, ipopt_options);
        }
      }
      if (std::isfinite(native.objective) && std::isfinite(ipopt.objective)) {
        objective_difference = std::abs(native.objective - ipopt.objective) /
            std::max(1.0, std::abs(ipopt.objective));
      }
      if (native.full_voltage.size() > 0 &&
          native.full_voltage.size() == ipopt.full_voltage.size()) {
        voltage_difference = max_voltage_difference(
            native.full_voltage, ipopt.full_voltage);
      }
      if (std::isfinite(native.objective) &&
          std::isfinite(ipopt_verification.objective)) {
        verification_objective_difference =
            std::abs(native.objective - ipopt_verification.objective) /
            std::max(1.0, std::abs(ipopt_verification.objective));
      }
      if (native.full_voltage.size() > 0 &&
          native.full_voltage.size() == ipopt_verification.full_voltage.size()) {
        verification_voltage_difference = max_voltage_difference(
            native.full_voltage, ipopt_verification.full_voltage);
      }
    } catch (const std::exception& error) {
      native.status = std::string("cross-solver setup failed: ") + error.what();
    }

    const bool passes = native.converged && ipopt_verification.converged &&
        native.primal_residual <= kAgreementTolerance &&
        ipopt_verification.primal_residual <= kAgreementTolerance &&
        native.max_omitted_inequality <= kAgreementTolerance &&
        ipopt_verification.max_omitted_inequality <= kAgreementTolerance &&
        verification_objective_difference <= kAgreementTolerance &&
        verification_voltage_difference <= kAgreementTolerance;
    all_cases_pass = all_cases_pass && passes;
    out << input.label << ',' << (native.converged ? "yes" : "no") << ','
        << (ipopt.converged ? "yes" : "no") << ','
        << (ipopt_verification.converged ? "yes" : "no") << ",\""
        << native.solver
        << "\",\"" << ipopt.solver << "\",no," << native.objective << ','
        << ipopt.objective << ',' << ipopt_verification.objective << ','
        << objective_difference << ',' << voltage_difference << ','
        << verification_objective_difference << ','
        << verification_voltage_difference << ',' << native.primal_residual << ','
        << ipopt.primal_residual << ',' << ipopt_verification.primal_residual
        << ',' << native.dual_residual << ','
        << ipopt.dual_residual << ',' << native.complementarity << ','
        << ipopt.complementarity << ',' << native_screen_rows << ','
        << ipopt_screen_rows << ',' << common_enforced_rows << ','
        << native.inequalities << ','
        << native.max_omitted_inequality << ','
        << ipopt.max_omitted_inequality << ',' << (passes ? "yes" : "no")
        << ",\"" << native.status << "\",\"" << ipopt.status << "\",\""
        << ipopt_verification.status << "\"\n"
        << std::flush;
    std::cout << input.label << " CROSSSOLVER: native=" << native.converged
              << ", ipopt=" << ipopt.converged
              << ", cold_objective/voltage_difference="
              << objective_difference << '/' << voltage_difference
              << ", verification_objective/voltage_difference="
              << verification_objective_difference << '/'
              << verification_voltage_difference << ", primal_residuals="
              << native.primal_residual << '/' << ipopt.primal_residual << '/'
              << ipopt_verification.primal_residual << ", screen/common_rows="
              << native_screen_rows << '/' << ipopt_screen_rows << '/'
              << common_enforced_rows << ", passes=" << passes << '\n'
              << std::flush;
  }
  std::cout << "CROSSSOLVER four-case agreement=" << all_cases_pass
            << "\nWrote " << path << '\n';
  return all_cases_pass ? 0 : 1;
}

int run_parametric_warmbench_suite(const std::vector<Input>& selected,
                                   SolverBackend backend,
                                   int repeats) {
  constexpr double kEquivalenceTolerance = 1e-6;
  const double minimum_case_speedup =
      backend == SolverBackend::NativeIPM ? 1.05 : 1.02;
  const double minimum_geometric_mean_speedup =
      backend == SolverBackend::NativeIPM ? 1.20 : 1.02;
  const fs::path path = project_root() / "output" / "benchmarks" /
      (backend == SolverBackend::NativeIPM
           ? "paper_native_ipm_parametric_warmbench.csv"
           : "paper_ipopt_multiplier_warmbench.csv");
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,repeat,variant,converged,runtime_ms,iterations,"
         "factorizations,enforced_rows,total_rows,phase2_start_accepted,"
         "primal_residual,dual_residual,complementarity,objective,"
         "pair_objective_relative_error,pair_voltage_error_pu,status\n";
  out << std::setprecision(12);

  const auto median = [](std::vector<double> values) {
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
  };
  bool all_cases_pass = true;
  double log_speedup_sum = 0.0;
  int passed_cases = 0;
  for (const auto& input : selected) {
    const ThreePhaseHybridOPFCase base = hybridize(input);
    ThreePhaseHybridOPFOptions options;
    options.backend = backend;
    options.warm_start_with_ipopt = false;
    options.variant = ModelVariant::GraphReduced;
    options.max_iterations = 500;
    options.tolerance = kEquivalenceTolerance;
    options.use_constraint_oracle = true;
    options.native_primary_max_iterations_before_restoration = 0;
    options.reduction_options.max_front = 12;
    options.reduction_options.max_nnz_ratio = 2.0;
    const ThreePhaseHybridOPFResult base_result =
        solve_three_phase_hybrid_opf(base, options);
    if (!base_result.converged) {
      std::cerr << '[' << input.label
                << "] PARAMBENCH failed: base OPF did not converge: "
                << base_result.status << '\n';
      all_cases_pass = false;
      continue;
    }

    ThreePhaseHybridOPFCase perturbed = base;
    perturbed.p_load_pu *= 1.001;
    perturbed.q_load_pu *= 1.001;
    perturbed.p_dc_load_pu *= 1.001;
    std::vector<double> candidate_times;
    std::vector<double> reference_times;
    std::vector<double> candidate_factors;
    std::vector<double> reference_factors;
    std::vector<double> candidate_iterations;
    std::vector<double> reference_iterations;
    bool case_passes = true;
    const auto run_branch = [&](ParametricWarmStartMode mode) {
      return solve_three_phase_hybrid_opf_branches(
          base, base_result, {perturbed}, options, mode).front();
    };
    for (int repeat = 0; repeat < repeats; ++repeat) {
      ThreePhaseHybridOPFResult candidate;
      ThreePhaseHybridOPFResult reference;
      if (repeat % 2 == 0) {
        candidate = run_branch(ParametricWarmStartMode::PrimalDual);
        reference = run_branch(ParametricWarmStartMode::PrimalOnly);
      } else {
        reference = run_branch(ParametricWarmStartMode::PrimalOnly);
        candidate = run_branch(ParametricWarmStartMode::PrimalDual);
      }
      const double objective_error =
          std::abs(candidate.objective - reference.objective) /
          std::max(1.0, std::abs(reference.objective));
      const double voltage_error = max_voltage_difference(
          candidate.full_voltage, reference.full_voltage);
      const auto satisfies_tolerances = [=](
          const ThreePhaseHybridOPFResult& result) {
        return result.converged && result.primal_residual <=
                   kEquivalenceTolerance &&
               result.dual_residual <= kEquivalenceTolerance &&
               result.complementarity <= kEquivalenceTolerance;
      };
      const bool pair_passes = satisfies_tolerances(candidate) &&
          satisfies_tolerances(reference) &&
          objective_error <= kEquivalenceTolerance &&
          voltage_error <= kEquivalenceTolerance &&
          candidate.enforced_inequality_rows ==
              reference.enforced_inequality_rows &&
          (backend != SolverBackend::Ipopt ||
           (candidate.primal_dual_warm_start_used &&
            !reference.primal_dual_warm_start_used));
      case_passes = case_passes && pair_passes;
      candidate_times.push_back(candidate.runtime_ms);
      reference_times.push_back(reference.runtime_ms);
      candidate_factors.push_back(candidate.phase_two_total_factorizations);
      reference_factors.push_back(reference.phase_two_total_factorizations);
      candidate_iterations.push_back(candidate.iterations);
      reference_iterations.push_back(reference.iterations);
      const auto write = [&](const char* variant,
                             const ThreePhaseHybridOPFResult& result) {
        out << input.label << ',' << repeat + 1 << ',' << variant << ','
            << (result.converged ? "yes" : "no") << ','
            << result.runtime_ms << ',' << result.iterations << ','
            << result.phase_two_total_factorizations << ','
            << result.enforced_inequalities << ',' << result.inequalities << ','
            << (result.primal_dual_warm_start_used ||
                        result.phase_two_start_accepted ? "yes" : "no") << ','
            << result.primal_residual << ',' << result.dual_residual << ','
            << result.complementarity << ',' << result.objective << ','
            << objective_error << ',' << voltage_error << ",\""
            << result.status << "\"\n";
      };
      write("primal-dual", candidate);
      write("primal-only", reference);
      out.flush();
      std::cerr << '[' << input.label << "] PARAMBENCH rep " << repeat + 1
                << '/' << repeats << ": primal-dual="
                << candidate.runtime_ms << "ms/"
                << candidate.iterations << "iter, primal-only="
                << reference.runtime_ms << "ms/" << reference.iterations
                << "iter, passes=" << pair_passes << '\n';
    }
    const double candidate_ms = median(candidate_times);
    const double reference_ms = median(reference_times);
    const double candidate_factorizations = median(candidate_factors);
    const double reference_factorizations = median(reference_factors);
    const double candidate_iteration_count = median(candidate_iterations);
    const double reference_iteration_count = median(reference_iterations);
    const double speedup = reference_ms / std::max(1e-12, candidate_ms);
    // Runtime is the performance endpoint. Factorization counts remain a
    // diagnostic because constraint-generation rounds and terminal KKT
    // certification can trade more small sparse solves for less wall time.
    case_passes = case_passes && speedup >= minimum_case_speedup;
    all_cases_pass = all_cases_pass && case_passes;
    if (case_passes) {
      log_speedup_sum += std::log(speedup);
      ++passed_cases;
    }
    std::cout << input.label << " PARAMBENCH: base="
              << base_result.runtime_ms << "ms (excluded), primal-dual="
              << candidate_ms << "ms/" << candidate_iteration_count
              << "iter/" << candidate_factorizations
              << "fact, primal-only=" << reference_ms << "ms/"
              << reference_iteration_count << "iter/"
              << reference_factorizations << "fact, speedup=" << speedup
              << "x, passes=" << case_passes << '\n';
  }
  const double geometric_mean_speedup =
      passed_cases == static_cast<int>(selected.size())
          ? std::exp(log_speedup_sum / std::max(1, passed_cases)) : 0.0;
  all_cases_pass = all_cases_pass &&
      passed_cases == static_cast<int>(selected.size()) &&
      geometric_mean_speedup >= minimum_geometric_mean_speedup;
  std::cout << (backend == SolverBackend::NativeIPM ? "NATIVE" : "IPOPT")
            << " PARAMBENCH four-case geometric-mean speedup="
            << geometric_mean_speedup << "x, passes=" << all_cases_pass
            << "\nWrote " << path << '\n';
  return all_cases_pass ? 0 : 1;
}

// Median-of-N, same-process harness. Interleaves full-all and reduced-generation
// solves so multiplicative machine-state drift cancels within each repeat, then
// reports medians. This is the clean comparison the warm-start work is validated
// against (single-sample cross-run numbers vary ~2.6x from thermal state).
int run_warmbench_suite(const Input& input, SolverBackend backend, int repeats) {
  if (backend != SolverBackend::NativeIPM) {
    std::cerr << "warmbench mode requires the native backend\n";
    return 2;
  }
  const ThreePhaseHybridOPFCase base = hybridize(input);
  ThreePhaseHybridOPFOptions full_opts;
  full_opts.backend = backend;
  full_opts.warm_start_with_ipopt = backend == SolverBackend::NativeIPM;
  full_opts.variant = ModelVariant::Full;
  full_opts.max_iterations = 500;
  full_opts.tolerance = 1e-6;
  ThreePhaseHybridOPFOptions gen_opts = full_opts;
  gen_opts.variant = ModelVariant::GraphReduced;
  gen_opts.use_constraint_oracle = true;
  gen_opts.reduction_options.max_front = 12;
  gen_opts.reduction_options.max_nnz_ratio = 2.0;
  ThreePhaseHybridOPFOptions legacy_opts = gen_opts;
  legacy_opts.native_primary_max_iterations_before_restoration = 0;
  legacy_opts.native_restoration_max_iterations = 200;

  // Single-round ceiling: one reduced solve enforcing exactly the active set the
  // oracle discovers, with no rounds. This is the best case of A (single-round
  // seeding) -- if it beats full, a good a-priori seed is worth building.
  const ThreePhaseHybridOPFResult probe =
      solve_three_phase_hybrid_opf(base, gen_opts);
  ThreePhaseHybridOPFOptions oneshot_opts = gen_opts;
  oneshot_opts.use_constraint_oracle = false;
  oneshot_opts.enforced_inequality_rows = probe.enforced_inequality_rows;

  // Classify the discovered active set by row type to guide the seed heuristic.
  {
    const int full_n = static_cast<int>(base.y_ac.rows());
    const int volt_end = 2 * full_n;
    const int vuf_end =
        volt_end + static_cast<int>(base.three_phase_bus_nodes.size());
    int ncp = 0;
    for (const auto& conv : base.converters)
      ncp += static_cast<int>(conv.phase_nodes.size());
    const int convcap_end = vuf_end + static_cast<int>(base.converters.size());
    const int convcur_end = convcap_end + ncp;
    int cv = 0, cvuf = 0, ccap = 0, ccur = 0, cline = 0;
    for (const int r : probe.enforced_inequality_rows) {
      if (r < volt_end) ++cv;
      else if (r < vuf_end) ++cvuf;
      else if (r < convcap_end) ++ccap;
      else if (r < convcur_end) ++ccur;
      else ++cline;
    }
    std::cerr << '[' << input.label << "] active-set breakdown: voltage=" << cv
              << " vuf=" << cvuf << " convcap=" << ccap << " convcur=" << ccur
              << " line=" << cline << " (of " << base.ac_line_limits.size()
              << " line rows), total=" << probe.enforced_inequality_rows.size()
              << "\n" << std::flush;
  }

  std::vector<double> full_times;
  std::vector<double> legacy_times;
  std::vector<double> gen_times;
  std::vector<double> oneshot_times;
  ThreePhaseHybridOPFResult full_res;
  ThreePhaseHybridOPFResult legacy_res;
  ThreePhaseHybridOPFResult gen_res;
  ThreePhaseHybridOPFResult oneshot_res;
  bool all_full_converged = true;
  bool all_legacy_converged = true;
  bool all_bounded_converged = true;
  bool all_oneshot_converged = true;
  const fs::path path = project_root() / "output" / "benchmarks" /
      ("paper_native_ipm_warmbench_" + input.label + ".csv");
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,repeat,variant,converged,runtime_ms,iterations,oracle_rounds,"
         "enforced_rows,total_rows,initial_attempt_iterations,"
         "initial_attempt_factorizations,"
         "restoration_factorizations,retry_factorizations,total_factorizations,"
         "primal_residual,dual_residual,complementarity,objective\n";
  out << std::setprecision(12);
  const auto write_result = [&](int repeat, const char* variant,
                                const ThreePhaseHybridOPFResult& result) {
    out << input.label << ',' << repeat << ',' << variant << ','
        << (result.converged ? "yes" : "no") << ',' << result.runtime_ms << ','
        << result.iterations << ',' << result.constraint_oracle_rounds << ','
        << result.enforced_inequalities << ',' << result.inequalities << ','
        << result.phase_two_initial_attempt_iterations << ','
        << result.phase_two_initial_attempt_factorizations << ','
        << result.phase_two_restoration_factorizations << ','
        << result.phase_two_retry_factorizations << ','
        << result.phase_two_total_factorizations << ','
        << result.primal_residual << ',' << result.dual_residual << ','
        << result.complementarity << ',' << result.objective << '\n';
  };
  for (int i = 0; i < repeats; ++i) {
    full_res = solve_three_phase_hybrid_opf(base, full_opts);
    all_full_converged = all_full_converged && full_res.converged;
    full_times.push_back(full_res.runtime_ms);
    legacy_res = solve_three_phase_hybrid_opf(base, legacy_opts);
    all_legacy_converged = all_legacy_converged && legacy_res.converged;
    legacy_times.push_back(legacy_res.runtime_ms);
    gen_res = solve_three_phase_hybrid_opf(base, gen_opts);
    all_bounded_converged = all_bounded_converged && gen_res.converged;
    gen_times.push_back(gen_res.runtime_ms);
    oneshot_res = solve_three_phase_hybrid_opf(base, oneshot_opts);
    all_oneshot_converged = all_oneshot_converged && oneshot_res.converged;
    oneshot_times.push_back(oneshot_res.runtime_ms);
    write_result(i + 1, "full-bounded", full_res);
    write_result(i + 1, "reduced-legacy", legacy_res);
    write_result(i + 1, "reduced-bounded", gen_res);
    write_result(i + 1, "reduced-oneshot", oneshot_res);
    out.flush();
    std::cerr << '[' << input.label << "] rep " << (i + 1) << '/' << repeats
              << ": full=" << full_res.runtime_ms << "ms(it="
              << full_res.iterations << "), legacy=" << legacy_res.runtime_ms
              << "ms(fact=" << legacy_res.phase_two_total_factorizations
              << "), gen=" << gen_res.runtime_ms
              << "ms(rounds=" << gen_res.constraint_oracle_rounds
              << ",it=" << gen_res.iterations << ",fact="
              << gen_res.phase_two_total_factorizations << "), 1shot="
              << oneshot_res.runtime_ms << "ms(it=" << oneshot_res.iterations
              << ")\n" << std::flush;
  }
  const auto median = [](std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
  };
  const double full_ms = median(full_times);
  const double legacy_ms = median(legacy_times);
  const double gen_ms = median(gen_times);
  const double oneshot_ms = median(oneshot_times);
  const bool comparison_passes =
      all_legacy_converged && all_bounded_converged;
  std::cout << input.label << " WARMBENCH (median of " << repeats << "): "
            << "full-all=" << full_ms << "ms (vars=" << full_res.variables
            << ", enf=" << full_res.enforced_inequalities << '/'
            << full_res.inequalities << ", JhNnz="
            << full_res.inequality_jacobian_nonzeros << "); reduced-gen="
            << gen_ms << "ms (vars=" << gen_res.variables << ", enf="
            << gen_res.enforced_inequalities << '/' << gen_res.inequalities
            << ", JhNnz=" << gen_res.inequality_jacobian_nonzeros << ", rounds="
            << gen_res.constraint_oracle_rounds << "); reduced-1shot="
            << oneshot_ms << "ms (enf=" << oneshot_res.enforced_inequalities
            << ", JhNnz=" << oneshot_res.inequality_jacobian_nonzeros
            << ", conv=" << oneshot_res.converged << "); legacy-gen="
            << legacy_ms << "ms; speedup legacy/gen=";
  if (comparison_passes && gen_ms > 0.0) {
    std::cout << legacy_ms / gen_ms;
  } else {
    std::cout << "not-comparable";
  }
  std::cout << ", full/gen=";
  if (all_full_converged && all_bounded_converged && gen_ms > 0.0) {
    std::cout << full_ms / gen_ms;
  } else {
    std::cout << "not-comparable";
  }
  std::cout << ", full/1shot=";
  if (all_full_converged && all_oneshot_converged && oneshot_ms > 0.0) {
    std::cout << full_ms / oneshot_ms;
  } else {
    std::cout << "not-comparable";
  }
  std::cout << '\n' << std::flush;
  std::cout << "Wrote " << path << '\n';
  return comparison_passes ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<Input> selected;
  const auto all = inputs();
  if (argc > 1) {
    const std::string wanted = argv[1];
    if (wanted == "all") {
      selected = all;
    } else {
      for (const auto& input : all) {
        if (input.label == wanted) selected.push_back(input);
      }
    }
    if (selected.empty()) {
      std::cerr << "unknown case label: " << wanted << "\n";
      return 2;
    }
  } else {
    selected = all;
  }
  const bool trace_full = argc > 2 && std::string(argv[2]) == "trace";
  const bool gr_only = argc > 2 && std::string(argv[2]) == "gr-only";
  const bool matched_full = argc > 2 && std::string(argv[2]) == "matched";
  const bool build_only = argc > 2 && std::string(argv[2]) == "build-only";
  const bool run_continuation =
      argc > 2 && std::string(argv[2]) == "reuse";
  const bool run_sensitivity =
      argc > 2 && std::string(argv[2]) == "sensitivity";
  const bool run_converter_modes =
      argc > 2 && std::string(argv[2]) == "controls";
  const bool run_opf_pf_crosscheck =
      argc > 2 && std::string(argv[2]) == "pf-crosscheck";
  const bool run_exp1 = argc > 2 && std::string(argv[2]) == "exp1";
  const bool run_exp2 = argc > 2 && std::string(argv[2]) == "exp2";
  const bool run_warmbench = argc > 2 && std::string(argv[2]) == "warmbench";
  const bool run_parambench = argc > 2 && std::string(argv[2]) == "parambench";
  const bool run_crosssolver =
      argc > 2 && std::string(argv[2]) == "crosssolver";
  bool use_native = false;
  bool use_constraint_oracle = false;
  bool verbose = true;
  for (int arg = 2; arg < argc; ++arg) {
    use_native = use_native || std::string(argv[arg]) == "native";
    use_constraint_oracle =
        use_constraint_oracle || std::string(argv[arg]) == "oracle";
    if (std::string(argv[arg]) == "quiet") verbose = false;
  }
  const SolverBackend backend =
      use_native ? SolverBackend::NativeIPM : SolverBackend::Ipopt;
  if (run_sensitivity) {
    if (selected.size() != 1) {
      std::cerr << "sensitivity mode requires exactly one case\n";
      return 2;
    }
    return run_sensitivity_suite(selected.front(), backend, 3);
  }
  if (run_converter_modes) {
    if (selected.size() != 1) {
      std::cerr << "controls mode requires exactly one case\n";
      return 2;
    }
    return run_converter_mode_suite(selected.front(), backend);
  }
  if (run_opf_pf_crosscheck) {
    return run_opf_pf_crosscheck_suite(selected, backend);
  }
  if (run_exp1) {
    if (selected.size() != 1) {
      std::cerr << "exp1 mode requires exactly one case\n";
      return 2;
    }
    return run_exp1_suite(selected.front(), backend);
  }
  if (run_exp2) {
    if (selected.size() != 1) {
      std::cerr << "exp2 mode requires exactly one case\n";
      return 2;
    }
    return run_exp2_suite(selected.front(), backend);
  }
  if (run_warmbench) {
    if (selected.size() != 1) {
      std::cerr << "warmbench mode requires exactly one case\n";
      return 2;
    }
    return run_warmbench_suite(selected.front(), backend, 5);
  }
  if (run_parambench) {
    return run_parametric_warmbench_suite(selected, backend, 5);
  }
  if (run_crosssolver) {
    return run_crosssolver_suite(selected);
  }

  std::vector<Audit> audits;
  std::vector<DriftTraceRow> drift_trace;
  for (const auto& input : selected) {
    Audit audit = run_case(input, backend, !gr_only && !run_continuation,
                           matched_full, build_only,
                           use_constraint_oracle, run_continuation, verbose,
                           trace_full ? &drift_trace : nullptr);
    std::cout << audit.label << ": built=" << audit.built
              << ", backend=" << (backend == SolverBackend::Ipopt ? "ipopt" : "native")
              << ", full_solver=" << audit.full.solver
              << ", full=" << audit.full.converged
              << ", gr=" << audit.reduced.converged
              << ", eliminated=" << audit.reduced.eliminated_phase_nodes
              << ", objective_error=" << audit.objective_relative_error
              << ", voltage_error=" << audit.voltage_error
              << ", speedup=" << audit.speedup
              << ", full_residual=" << audit.full.primal_residual
              << ", gr_residual=" << audit.reduced.primal_residual
              << ", full_dual/comp=" << audit.full.dual_residual << '/'
              << audit.full.complementarity
              << ", gr_dual/comp=" << audit.reduced.dual_residual << '/'
              << audit.reduced.complementarity
              << ", full_rows=" << audit.full.enforced_inequalities << '/'
              << audit.full.inequalities
              << ", gr_rows=" << audit.reduced.enforced_inequalities << '/'
              << audit.reduced.inequalities
              << ", full_oracle=" << audit.full.constraint_oracle_rounds << '/'
              << audit.full.constraint_oracle_added_rows
              << ", gr_oracle=" << audit.reduced.constraint_oracle_rounds << '/'
              << audit.reduced.constraint_oracle_added_rows
              << ", full_Jh_nnz=" << audit.full.inequality_jacobian_nonzeros
              << ", gr_Jh_nnz=" << audit.reduced.inequality_jacobian_nonzeros
              << ", full_max_omitted=" << audit.full.max_omitted_inequality
              << ", gr_max_omitted=" << audit.reduced.max_omitted_inequality
              << ", continuation=" << audit.continuation.converged
              << '/' << audit.continuation.runtime_ms
              << "ms/" << audit.continuation.iterations
              << "it/" << audit.continuation.enforced_inequalities
              << "rows/" << audit.continuation.constraint_oracle_added_rows
              << "added, continuation_reference="
              << audit.continuation_reference.converged << '/'
              << audit.continuation_reference.runtime_ms << "ms"
              << ", continuation_error="
              << audit.continuation_objective_relative_error << '/'
              << audit.continuation_voltage_error
              << ", full_initial=" << audit.full.initial_primal_residual
              << "/" << audit.full.initial_dual_residual
              << "(eq=" << audit.full.initial_worst_equality
              << ",ineq=" << audit.full.initial_worst_inequality << ")"
              << ", gr_initial=" << audit.reduced.initial_primal_residual
              << "/" << audit.reduced.initial_dual_residual
              << "(eq=" << audit.reduced.initial_worst_equality
              << ",ineq=" << audit.reduced.initial_worst_inequality << ")"
              << ", full_status=" << audit.full.status
              << ", gr_status=" << audit.reduced.status
              << ", full_jac_err=" << audit.full.max_equality_jacobian_error
              << '/' << audit.full.max_inequality_jacobian_error
              << ", gr_jac_err=" << audit.reduced.max_equality_jacobian_error
              << '/' << audit.reduced.max_inequality_jacobian_error
              << ", full_hess_err=" << audit.full.max_lagrangian_hessian_error
              << ", gr_hess_err=" << audit.reduced.max_lagrangian_hessian_error;
    if (!audit.error.empty()) std::cout << ", error=" << audit.error;
    std::cout << "\n";
    audits.push_back(std::move(audit));
  }
  const std::string mode_suffix =
      (build_only ? "_build" :
       (run_continuation ? "_reuse" :
        (matched_full ? "_matched" : "_cold"))) +
      std::string(use_constraint_oracle ? "_oracle" : "");
  const std::string output_name = selected.size() == 1
      ? "phase_hybrid_opf_case_audit_" + selected.front().label +
            mode_suffix + ".csv"
      : "phase_hybrid_opf_case_audit" + mode_suffix + ".csv";
  const fs::path output =
      project_root() / "output" / "benchmarks" / output_name;
  write_csv(output, audits);
  std::cout << "Wrote " << output << "\n";
  if (trace_full) {
    const fs::path trace_output = project_root() / "output" / "benchmarks" /
                                  "phase_hybrid_opf_full_drift_trace.csv";
    write_drift_trace(trace_output, drift_trace);
    std::cout << "Wrote " << trace_output << "\n";
  }
  return 0;
}
