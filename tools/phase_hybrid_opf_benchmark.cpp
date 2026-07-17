#include <algorithm>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
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
using hacdcpf::analysis::PhaseNodeIndexer;
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
  double objective_relative_error{std::numeric_limits<double>::quiet_NaN()};
  double voltage_error{std::numeric_limits<double>::quiet_NaN()};
  double speedup{std::numeric_limits<double>::quiet_NaN()};
  std::string error;
};

fs::path project_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return fs::path(HACDCPF_PROJECT_ROOT);
#else
  return fs::current_path();
#endif
}

SparseComplexMatrix build_ybus(
    int dimension,
    const std::vector<hacdcpf::analysis::PhaseDomainSparseEntry>& entries) {
  std::vector<Eigen::Triplet<Complex>> triplets;
  triplets.reserve(entries.size());
  for (const auto& entry : entries) {
    triplets.emplace_back(entry.row, entry.col, entry.value);
  }
  SparseComplexMatrix y(dimension, dimension);
  y.setFromTriplets(triplets.begin(), triplets.end(),
                    [](Complex a, Complex b) { return a + b; });
  y.prune(Complex{}, 1e-13);
  y.makeCompressed();
  return y;
}

std::vector<Input> inputs() {
  const fs::path refs = project_root() / "external_data" / "opendss_ieee_pes" /
                        "opendss_reference";
  return {
      {"H13", refs / "13_node" / "official_full" / "IEEE13Nodeckt.dss", 0.01, 1},
      {"H34", refs / "34_node" / "ieee34Mod2.dss", 0.20, 2},
      {"H123", refs / "123_node" / "IEEE123Master.dss", 0.25, 4},
      {"H8500", refs / "8500_node" / "Master.dss", 0.30, 12},
  };
}

double phase_value(const double values[3], int phase) {
  return values[phase];
}

ThreePhaseHybridOPFCase hybridize(const Input& input) {
  const ThreePhaseACSystem sys =
      hacdcpf::analysis::load_three_phase_system_from_opendss(input.master, 100.0);
  if (sys.buses.empty()) throw std::runtime_error("OpenDSS import returned no buses");
  const auto compact = hacdcpf::analysis::build_compact_pf_data(sys, true);
  const PhaseNodeIndexer& indexer = compact.indexer;
  const int n = indexer.total_nodes;

  ThreePhaseHybridOPFCase c;
  c.name = input.label;
  c.base_mva = sys.base_mva > 0.0 ? sys.base_mva : 1.0;
  c.y_ac = build_ybus(n, compact.ybus_entries);
  c.i_ac_fixed = Eigen::VectorXcd::Zero(n);
  if (compact.fixed_current.size() != static_cast<std::size_t>(n)) {
    throw std::runtime_error("compact fixed-current dimension mismatch");
  }
  for (int node = 0; node < n; ++node) {
    c.i_ac_fixed[node] = compact.fixed_current[static_cast<std::size_t>(node)];
  }
  c.p_load_pu = Eigen::VectorXd::Zero(n);
  c.q_load_pu = Eigen::VectorXd::Zero(n);
  c.v_min_pu = Eigen::VectorXd::Constant(n, 0.85);
  c.v_max_pu = Eigen::VectorXd::Constant(n, 1.10);
  c.voltage_start = Eigen::VectorXcd::Ones(n);
  c.vuf_max = 0.03;

  for (const auto& node : indexer.nodes) {
    const auto& bus = sys.buses[static_cast<std::size_t>(node.bus_offset)];
    const double vm[3] = {bus.vm_a_pu, bus.vm_b_pu, bus.vm_c_pu};
    const double va[3] = {bus.va_a_deg, bus.va_b_deg, bus.va_c_deg};
    const double pd[3] = {bus.pd_a_mw, bus.pd_b_mw, bus.pd_c_mw};
    const double qd[3] = {bus.qd_a_mvar, bus.qd_b_mvar, bus.qd_c_mvar};
    const int k = node.compact_index;
    const double magnitude = vm[node.phase_index] > 0.0 ? vm[node.phase_index] : 1.0;
    c.voltage_start[k] = std::polar(magnitude, va[node.phase_index] * M_PI / 180.0);
    c.p_load_pu[k] += pd[node.phase_index] / c.base_mva;
    c.q_load_pu[k] += qd[node.phase_index] / c.base_mva;
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
      if (!load.phase_mask.has(phase) || !indexer.has_node(bus_offset, phase)) continue;
      const int node = indexer.node_index(bus_offset, phase);
      c.p_load_pu[node] += p[phase] / c.base_mva;
      c.q_load_pu[node] += q[phase] / c.base_mva;
    }
  }

  hacdcpf::analysis::ThreePhaseNROptions nr_options;
  nr_options.max_iter = 100;
  nr_options.tol = 1e-9;
  nr_options.include_shunts = true;
  const auto nr = hacdcpf::analysis::solve_three_phase_nr(sys, nr_options);
  if (nr.converged) {
    for (const auto& voltage : nr.bus_voltages) {
      const auto bus_it = std::find_if(
          sys.buses.begin(), sys.buses.end(),
          [&](const auto& bus) { return bus.index == voltage.bus_id; });
      if (bus_it == sys.buses.end()) continue;
      const int bus_offset = static_cast<int>(std::distance(sys.buses.begin(), bus_it));
      const double vm[3] = {voltage.vm_a_pu, voltage.vm_b_pu, voltage.vm_c_pu};
      const double va[3] = {voltage.va_a_deg, voltage.va_b_deg, voltage.va_c_deg};
      for (int phase = 0; phase < 3; ++phase) {
        if (!indexer.has_node(bus_offset, phase) || !(vm[phase] > 0.0)) continue;
        c.voltage_start[indexer.node_index(bus_offset, phase)] =
            std::polar(vm[phase], va[phase] * M_PI / 180.0);
      }
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
      if (!grid.phase_mask.has(phase) || !indexer.has_node(bus_offset, phase)) continue;
      const int node = indexer.node_index(bus_offset, phase);
      if (!reference_node_set.insert(node).second) continue;
      const double phase_shift[3] = {0.0, -120.0, 120.0};
      const double vm[3] = {grid.vm_a_pu, grid.vm_b_pu, grid.vm_c_pu};
      const double va[3] = {grid.va_a_deg, grid.va_b_deg, grid.va_c_deg};
      const double magnitude = grid.use_phase_voltage_setpoint && vm[phase] > 0.0
          ? vm[phase] : (grid.vm_pu > 0.0 ? grid.vm_pu : 1.0);
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
        if (!indexer.has_node(bus_offset, phase)) continue;
        const int node = indexer.node_index(bus_offset, phase);
        c.reference_nodes.push_back(node);
        c.reference_voltage.conservativeResize(c.reference_voltage.size() + 1);
        c.reference_voltage[c.reference_voltage.size() - 1] = c.voltage_start[node];
      }
      break;
    }
  }
  if (c.reference_nodes.empty()) throw std::runtime_error("no AC reference phase found");

  for (int bus_offset = 0; bus_offset < static_cast<int>(sys.buses.size()); ++bus_offset) {
    if (indexer.has_node(bus_offset, 0) && indexer.has_node(bus_offset, 1) &&
        indexer.has_node(bus_offset, 2)) {
      c.three_phase_bus_nodes.push_back({indexer.node_index(bus_offset, 0),
                                         indexer.node_index(bus_offset, 1),
                                         indexer.node_index(bus_offset, 2)});
    }
  }

  std::vector<CandidateBus> candidates;
  for (int bus_offset = 0; bus_offset < static_cast<int>(sys.buses.size()); ++bus_offset) {
    CandidateBus candidate;
    candidate.bus_offset = bus_offset;
    for (int phase = 0; phase < 3; ++phase) {
      if (!indexer.has_node(bus_offset, phase)) continue;
      const int node = indexer.node_index(bus_offset, phase);
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
  std::vector<double> bucket_load(static_cast<std::size_t>(input.vsc_count), 0.0);
  std::vector<std::vector<int>> terminal_nodes(static_cast<std::size_t>(input.vsc_count));
  for (int k = 0; k < input.vsc_count; ++k) {
    terminal_nodes[static_cast<std::size_t>(k)] =
        candidates[static_cast<std::size_t>(k)].nodes;
  }
  double remaining = target_dc;
  int cursor = 0;
  for (const auto& candidate : candidates) {
    if (remaining <= 1e-12) break;
    const int bucket = cursor++ % input.vsc_count;
    const double available = std::accumulate(
        candidate.nodes.begin(), candidate.nodes.end(), 0.0,
        [&](double sum, int node) { return sum + std::max(0.0, c.p_load_pu[node]); });
    const double moved = std::min(available, remaining);
    if (moved <= 0.0) continue;
    const double fraction = moved / available;
    for (int node : candidate.nodes) {
      c.p_load_pu[node] *= 1.0 - fraction;
      c.q_load_pu[node] *= 1.0 - fraction;
    }
    bucket_load[static_cast<std::size_t>(bucket)] += moved;
    remaining -= moved;
  }
  if (remaining > 1e-8 * std::max(1.0, target_dc)) {
    throw std::runtime_error("unable to realize requested DC load share");
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

    PhaseVSC converter;
    converter.phase_nodes = terminal_nodes[static_cast<std::size_t>(k)];
    converter.dc_terminal = terminal;
    converter.efficiency = 0.98;
    converter.s_max_pu = 1.30 * p / converter.efficiency + 1e-3;
    converter.fixed_unity_power_factor = false;
    c.converters.push_back(std::move(converter));
  }
  c.g_dc.setFromTriplets(gtrip.begin(), gtrip.end());
  c.g_dc.makeCompressed();

  const double total_capacity = 5.0 * (original_p + 1.0);
  for (int ri = 0; ri < static_cast<int>(c.reference_nodes.size()); ++ri) {
    PhaseGenerator generator;
    generator.phase_node = c.reference_nodes[static_cast<std::size_t>(ri)];
    generator.p_min_pu = -total_capacity;
    generator.p_max_pu = total_capacity;
    generator.q_min_pu = -total_capacity;
    generator.q_max_pu = total_capacity;
    generator.cost_c2 = 0.01;
    generator.cost_c1 = 45.0 + 0.5 * ri;
    c.generators.push_back(generator);
  }
  return c;
}

double max_voltage_difference(const Eigen::VectorXcd& a,
                              const Eigen::VectorXcd& b) {
  if (a.size() != b.size() || a.size() == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (a - b).cwiseAbs().maxCoeff();
}

Audit run_case(const Input& input) {
  Audit audit;
  audit.label = input.label;
  audit.requested_dc_share = input.dc_share;
  try {
    const ThreePhaseHybridOPFCase c = hybridize(input);
    audit.built = true;
    audit.phase_nodes = static_cast<int>(c.y_ac.rows());
    audit.dc_nodes = static_cast<int>(c.g_dc.rows());
    audit.vscs = static_cast<int>(c.converters.size());
    const double ac_p = c.p_load_pu.sum();
    const double dc_p = c.p_dc_load_pu.sum();
    audit.realized_dc_share = dc_p / std::max(1e-12, ac_p + dc_p);

    ThreePhaseHybridOPFOptions options;
    options.backend = SolverBackend::NativeIPM;
    options.warm_start_with_ipopt = true;
    options.verify_derivatives = input.label == "H13";
    options.max_iterations = 500;
    options.tolerance = 1e-9;
    options.variant = ModelVariant::GraphReduced;
    options.reduction_options.max_front = 12;
    options.reduction_options.max_nnz_ratio = 2.0;
    audit.reduced = solve_three_phase_hybrid_opf(c, options);
    ThreePhaseHybridOPFCase full_case = c;
    if (audit.reduced.full_voltage.size() == c.voltage_start.size() &&
        audit.reduced.full_voltage.allFinite()) {
      full_case.voltage_start = audit.reduced.full_voltage;
    }
    options.variant = ModelVariant::Full;
    const int ng = static_cast<int>(c.generators.size());
    const int ndc = static_cast<int>(c.g_dc.rows());
    int ncp = 0;
    for (const auto& converter : c.converters) {
      ncp += static_cast<int>(converter.phase_nodes.size());
    }
    const int nc = static_cast<int>(c.converters.size());
    const int reduced_nv = static_cast<int>(audit.reduced.reduction.retained.size());
    const int full_nv = static_cast<int>(c.y_ac.rows());
    const int device_variables = 2 * ng + ndc + 2 * ncp + nc;
    if (audit.reduced.primal.size() == 2 * reduced_nv + device_variables &&
        audit.reduced.full_voltage.size() == full_nv) {
      options.primal_start = Eigen::VectorXd::Zero(2 * full_nv + device_variables);
      for (int node = 0; node < full_nv; ++node) {
        options.primal_start[node] = std::real(audit.reduced.full_voltage[node]);
        options.primal_start[full_nv + node] =
            std::imag(audit.reduced.full_voltage[node]);
      }
      options.primal_start.tail(device_variables) =
          audit.reduced.primal.tail(device_variables);
    }
    audit.full = solve_three_phase_hybrid_opf(full_case, options);

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

void write_csv(const fs::path& path, const std::vector<Audit>& audits) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  out << "case,built,phase_nodes,dc_nodes,vscs,requested_dc_share,realized_dc_share,"
         "full_converged,gr_converged,full_variables,gr_variables,full_equalities,"
         "gr_equalities,gr_eliminated,objective_relative_error,voltage_error,"
         "full_primal,gr_primal,full_runtime_ms,gr_runtime_ms,speedup,"
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
        << a.full.primal_residual << ',' << a.reduced.primal_residual << ','
        << a.full.runtime_ms << ',' << a.reduced.runtime_ms << ',' << a.speedup
        << ",\"" << a.full.status << "\",\"" << a.reduced.status
        << "\",\"" << a.error << "\"\n";
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::vector<Input> selected;
  const auto all = inputs();
  if (argc > 1) {
    const std::string wanted = argv[1];
    for (const auto& input : all) {
      if (input.label == wanted) selected.push_back(input);
    }
    if (selected.empty()) {
      std::cerr << "unknown case label: " << wanted << "\n";
      return 2;
    }
  } else {
    selected = all;
  }

  std::vector<Audit> audits;
  for (const auto& input : selected) {
    Audit audit = run_case(input);
    std::cout << audit.label << ": built=" << audit.built
              << ", full=" << audit.full.converged
              << ", gr=" << audit.reduced.converged
              << ", eliminated=" << audit.reduced.eliminated_phase_nodes
              << ", objective_error=" << audit.objective_relative_error
              << ", voltage_error=" << audit.voltage_error
              << ", speedup=" << audit.speedup
              << ", full_residual=" << audit.full.primal_residual
              << ", gr_residual=" << audit.reduced.primal_residual
              << ", full_initial=" << audit.full.initial_primal_residual
              << "(eq=" << audit.full.initial_worst_equality
              << ",ineq=" << audit.full.initial_worst_inequality << ")"
              << ", gr_initial=" << audit.reduced.initial_primal_residual
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
  const fs::path output = project_root() / "output" / "benchmarks" /
                          "phase_hybrid_opf_case_audit.csv";
  write_csv(output, audits);
  std::cout << "Wrote " << output << "\n";
  return 0;
}
