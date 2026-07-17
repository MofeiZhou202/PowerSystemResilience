/// Reproducible phase-node graph-reduction benchmark for the IEEE 13, 34,
/// 123, and 8500-node distribution feeders.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <iostream>
#include <queue>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "hacdcpf/model/ac_components.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"

namespace fs = std::filesystem;

namespace {

using hacdcpf::PhaseMask;
using hacdcpf::ThreePhaseACSystem;
using hacdcpf::analysis::PhaseNodeIndexer;
using Complex = std::complex<double>;
using SparseComplex = Eigen::SparseMatrix<Complex>;

struct CaseInput {
  std::string label;
  fs::path master;
};

struct CaseAudit {
  std::string label;
  fs::path master;
  bool loaded{false};
  std::string error;
  int buses{0};
  int phase_nodes{0};
  int lines{0};
  int transformers{0};
  int loads{0};
  int generators{0};
  int external_grids{0};
  int zero_injection_phase_nodes{0};
  int ybus_nonzeros{0};
  int retained_phase_nodes{0};
  int eliminated_phase_nodes{0};
  int reduced_nonzeros{0};
  int max_elimination_front{0};
  double reduced_to_full_nnz{0.0};
  double reduction_ms{0.0};
  double max_interior_current_residual{0.0};
  double max_boundary_current_error{0.0};
  double max_solve_state_error{0.0};
  double full_solve_ms{0.0};
  double reduced_solve_ms{0.0};
  double solve_speedup{0.0};
};

struct RecoveryStep {
  int node{-1};
  std::vector<std::pair<int, Complex>> coefficients;
};

struct SparseReduction {
  SparseComplex reduced;
  std::vector<int> retained;
  std::vector<RecoveryStep> recovery_steps;
  int max_front{0};
  double elapsed_ms{0.0};
};

class MutableSparseMatrix {
 public:
  explicit MutableSparseMatrix(const SparseComplex& matrix)
      : rows_(static_cast<std::size_t>(matrix.rows())),
        cols_(static_cast<std::size_t>(matrix.cols())),
        active_(static_cast<std::size_t>(matrix.rows()), true) {
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (SparseComplex::InnerIterator it(matrix, col); it; ++it) {
        set(it.row(), it.col(), it.value());
      }
    }
    initial_nnz_ = nnz_;
  }

  int size() const { return static_cast<int>(rows_.size()); }
  bool active(int node) const { return active_.at(static_cast<std::size_t>(node)); }
  long long nonzeros() const { return nnz_; }
  long long initial_nonzeros() const { return initial_nnz_; }

  Complex value(int row, int col) const {
    const auto& entries = rows_.at(static_cast<std::size_t>(row));
    const auto it = entries.find(col);
    return it == entries.end() ? Complex{} : it->second;
  }

  std::vector<int> row_neighbors(int node) const {
    std::vector<int> result;
    for (const auto& [col, value] : rows_.at(static_cast<std::size_t>(node))) {
      if (col != node && active(col) && std::abs(value) > kDropTolerance) {
        result.push_back(col);
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  std::vector<int> col_neighbors(int node) const {
    std::vector<int> result;
    for (const auto& [row, value] : cols_.at(static_cast<std::size_t>(node))) {
      if (row != node && active(row) && std::abs(value) > kDropTolerance) {
        result.push_back(row);
      }
    }
    std::sort(result.begin(), result.end());
    return result;
  }

  int front_size(int node) const {
    return std::max(
        static_cast<int>(row_neighbors(node).size()),
        static_cast<int>(col_neighbors(node).size()));
  }

  long long predicted_nonzeros_after_elimination(int node) const {
    const auto row_nodes = row_neighbors(node);
    const auto col_nodes = col_neighbors(node);
    long long additions = 0;
    for (int row : col_nodes) {
      for (int col : row_nodes) {
        if (std::abs(value(row, col)) <= kDropTolerance) ++additions;
      }
    }
    long long removals = static_cast<long long>(
        rows_.at(static_cast<std::size_t>(node)).size() +
        cols_.at(static_cast<std::size_t>(node)).size());
    if (std::abs(value(node, node)) > kDropTolerance) --removals;
    return nnz_ - removals + additions;
  }

  RecoveryStep eliminate(int node) {
    const Complex pivot = value(node, node);
    if (std::abs(pivot) <= kPivotTolerance) {
      throw std::runtime_error("zero or near-zero Kron pivot");
    }
    const auto row_nodes = row_neighbors(node);
    const auto col_nodes = col_neighbors(node);

    RecoveryStep recovery;
    recovery.node = node;
    recovery.coefficients.reserve(row_nodes.size());
    for (int col : row_nodes) {
      recovery.coefficients.emplace_back(col, -value(node, col) / pivot);
    }

    for (int row : col_nodes) {
      const Complex left = value(row, node);
      for (int col : row_nodes) {
        set(row, col, value(row, col) - left * value(node, col) / pivot);
      }
    }

    std::vector<int> row_keys;
    row_keys.reserve(rows_.at(static_cast<std::size_t>(node)).size());
    for (const auto& [col, _] : rows_.at(static_cast<std::size_t>(node))) {
      row_keys.push_back(col);
    }
    for (int col : row_keys) erase(node, col);

    std::vector<int> col_keys;
    col_keys.reserve(cols_.at(static_cast<std::size_t>(node)).size());
    for (const auto& [row, _] : cols_.at(static_cast<std::size_t>(node))) {
      col_keys.push_back(row);
    }
    for (int row : col_keys) erase(row, node);

    active_.at(static_cast<std::size_t>(node)) = false;
    return recovery;
  }

  SparseComplex retained_matrix(std::vector<int>& retained) const {
    retained.clear();
    std::vector<int> position(rows_.size(), -1);
    for (int node = 0; node < size(); ++node) {
      if (!active(node)) continue;
      position[static_cast<std::size_t>(node)] = static_cast<int>(retained.size());
      retained.push_back(node);
    }

    std::vector<Eigen::Triplet<Complex>> triplets;
    triplets.reserve(static_cast<std::size_t>(std::max<long long>(0, nnz_)));
    for (int row : retained) {
      for (const auto& [col, entry] : rows_.at(static_cast<std::size_t>(row))) {
        const int reduced_col = position.at(static_cast<std::size_t>(col));
        if (reduced_col < 0 || std::abs(entry) <= kDropTolerance) continue;
        triplets.emplace_back(
            position.at(static_cast<std::size_t>(row)), reduced_col, entry);
      }
    }
    SparseComplex reduced(
        static_cast<int>(retained.size()), static_cast<int>(retained.size()));
    reduced.setFromTriplets(triplets.begin(), triplets.end());
    reduced.makeCompressed();
    return reduced;
  }

 private:
  static constexpr double kDropTolerance = 1e-13;
  static constexpr double kPivotTolerance = 1e-12;

  void set(int row, int col, Complex entry) {
    auto& row_map = rows_.at(static_cast<std::size_t>(row));
    auto& col_map = cols_.at(static_cast<std::size_t>(col));
    const auto existing = row_map.find(col);
    const bool had_value = existing != row_map.end();
    if (std::abs(entry) <= kDropTolerance) {
      if (had_value) {
        row_map.erase(existing);
        col_map.erase(row);
        --nnz_;
      }
      return;
    }
    row_map[col] = entry;
    col_map[row] = entry;
    if (!had_value) ++nnz_;
  }

  void erase(int row, int col) {
    auto& row_map = rows_.at(static_cast<std::size_t>(row));
    const auto it = row_map.find(col);
    if (it == row_map.end()) return;
    row_map.erase(it);
    cols_.at(static_cast<std::size_t>(col)).erase(row);
    --nnz_;
  }

  std::vector<std::unordered_map<int, Complex>> rows_;
  std::vector<std::unordered_map<int, Complex>> cols_;
  std::vector<bool> active_;
  long long nnz_{0};
  long long initial_nnz_{0};
};

fs::path project_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return fs::path(HACDCPF_PROJECT_ROOT);
#else
  return fs::current_path();
#endif
}

bool phase_has_independent_injection(
    const ThreePhaseACSystem& sys,
    int bus_offset,
    int phase) {
  const auto& bus = sys.buses.at(static_cast<std::size_t>(bus_offset));
  const auto nonzero = [](double value) { return std::abs(value) > 1e-12; };

  const double bus_p[3] = {bus.pd_a_mw, bus.pd_b_mw, bus.pd_c_mw};
  const double bus_q[3] = {bus.qd_a_mvar, bus.qd_b_mvar, bus.qd_c_mvar};
  if (nonzero(bus_p[phase]) || nonzero(bus_q[phase])) return true;

  for (const auto& load : sys.loads) {
    if (!load.in_service || load.bus != bus.index ||
        !load.phase_mask.has(phase)) {
      continue;
    }
    const double p[3] = {load.p_a_mw, load.p_b_mw, load.p_c_mw};
    const double q[3] = {load.q_a_mvar, load.q_b_mvar, load.q_c_mvar};
    if (nonzero(p[phase]) || nonzero(q[phase])) return true;
  }

  for (const auto& gen : sys.generators) {
    if (gen.in_service && gen.bus == bus.index && gen.phase_mask.has(phase)) {
      return true;
    }
  }
  for (const auto& grid : sys.external_grids) {
    if (grid.in_service && grid.bus == bus.index && grid.phase_mask.has(phase)) {
      return true;
    }
  }
  return false;
}

SparseComplex build_sparse_ybus(
    int dimension,
    const std::vector<hacdcpf::analysis::PhaseDomainSparseEntry>& entries) {
  std::vector<Eigen::Triplet<Complex>> triplets;
  triplets.reserve(entries.size());
  for (const auto& entry : entries) {
    if (entry.row < 0 || entry.row >= dimension ||
        entry.col < 0 || entry.col >= dimension) {
      throw std::runtime_error("compact Y-bus entry is out of range");
    }
    triplets.emplace_back(entry.row, entry.col, entry.value);
  }
  SparseComplex matrix(dimension, dimension);
  matrix.setFromTriplets(
      triplets.begin(), triplets.end(),
      [](const Complex& lhs, const Complex& rhs) { return lhs + rhs; });
  matrix.prune(Complex{}, 1e-13);
  matrix.makeCompressed();
  return matrix;
}

SparseReduction reduce_sparse_phase_graph(
    const SparseComplex& ybus,
    const std::vector<bool>& eligible,
    int max_front = 12,
    double max_nnz_ratio = 2.0) {
  const auto start = std::chrono::steady_clock::now();
  MutableSparseMatrix matrix(ybus);
  using QueueEntry = std::pair<int, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
                      std::greater<QueueEntry>> queue;
  for (int node = 0; node < matrix.size(); ++node) {
    if (eligible.at(static_cast<std::size_t>(node))) {
      queue.emplace(matrix.front_size(node), node);
    }
  }

  SparseReduction result;
  while (!queue.empty()) {
    const auto [queued_front, node] = queue.top();
    queue.pop();
    if (!matrix.active(node) || !eligible.at(static_cast<std::size_t>(node))) {
      continue;
    }
    const int actual_front = matrix.front_size(node);
    if (actual_front != queued_front) {
      queue.emplace(actual_front, node);
      continue;
    }
    if (actual_front > max_front ||
        std::abs(matrix.value(node, node)) <= 1e-12) {
      continue;
    }
    const long long predicted =
        matrix.predicted_nonzeros_after_elimination(node);
    if (predicted > static_cast<long long>(
                        std::ceil(max_nnz_ratio *
                                  static_cast<double>(matrix.initial_nonzeros())))) {
      continue;
    }

    const auto affected_rows = matrix.row_neighbors(node);
    const auto affected_cols = matrix.col_neighbors(node);
    result.max_front = std::max(result.max_front, actual_front);
    result.recovery_steps.push_back(matrix.eliminate(node));
    for (int neighbor : affected_rows) {
      if (matrix.active(neighbor) && eligible.at(static_cast<std::size_t>(neighbor))) {
        queue.emplace(matrix.front_size(neighbor), neighbor);
      }
    }
    for (int neighbor : affected_cols) {
      if (matrix.active(neighbor) && eligible.at(static_cast<std::size_t>(neighbor))) {
        queue.emplace(matrix.front_size(neighbor), neighbor);
      }
    }
  }

  result.reduced = matrix.retained_matrix(result.retained);
  result.elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start).count();
  return result;
}

Eigen::VectorXcd recover_full_state(
    int full_size,
    const std::vector<int>& retained,
    const std::vector<RecoveryStep>& steps,
    const Eigen::VectorXcd& boundary_state) {
  if (boundary_state.size() != static_cast<int>(retained.size())) {
    throw std::runtime_error("boundary state dimension mismatch");
  }
  Eigen::VectorXcd full = Eigen::VectorXcd::Zero(full_size);
  for (int pos = 0; pos < static_cast<int>(retained.size()); ++pos) {
    full[retained[static_cast<std::size_t>(pos)]] = boundary_state[pos];
  }
  for (auto it = steps.rbegin(); it != steps.rend(); ++it) {
    Complex value{};
    for (const auto& [neighbor, coefficient] : it->coefficients) {
      value += coefficient * full[neighbor];
    }
    full[it->node] = value;
  }
  return full;
}

Eigen::VectorXcd deterministic_boundary_state(int size) {
  Eigen::VectorXcd state(size);
  for (int idx = 0; idx < size; ++idx) {
    const double magnitude = 0.97 + 0.04 *
        static_cast<double>((idx * 37) % 101) / 100.0;
    const double angle = 0.001 * static_cast<double>((idx * 53) % 211 - 105);
    state[idx] = std::polar(magnitude, angle);
  }
  return state;
}

double max_abs(const Eigen::VectorXcd& values) {
  double result = 0.0;
  for (int idx = 0; idx < values.size(); ++idx) {
    result = std::max(result, std::abs(values[idx]));
  }
  return result;
}

void validate_reduction(
    const SparseComplex& full_ybus,
    const std::vector<bool>& eligible,
    const SparseReduction& reduction,
    CaseAudit& audit) {
  const Eigen::VectorXcd boundary =
      deterministic_boundary_state(static_cast<int>(reduction.retained.size()));
  const Eigen::VectorXcd full = recover_full_state(
      full_ybus.rows(), reduction.retained, reduction.recovery_steps, boundary);
  const Eigen::VectorXcd full_current = full_ybus * full;
  const Eigen::VectorXcd reduced_current = reduction.reduced * boundary;

  std::vector<int> retained_position(static_cast<std::size_t>(full_ybus.rows()), -1);
  for (int pos = 0; pos < static_cast<int>(reduction.retained.size()); ++pos) {
    retained_position[static_cast<std::size_t>(
        reduction.retained[static_cast<std::size_t>(pos)])] = pos;
  }
  for (int node = 0; node < full_ybus.rows(); ++node) {
    const int pos = retained_position[static_cast<std::size_t>(node)];
    if (pos >= 0) {
      audit.max_boundary_current_error = std::max(
          audit.max_boundary_current_error,
          std::abs(full_current[node] - reduced_current[pos]));
    } else if (eligible.at(static_cast<std::size_t>(node))) {
      audit.max_interior_current_residual = std::max(
          audit.max_interior_current_residual, std::abs(full_current[node]));
    }
  }
}

struct TimedSolve {
  bool success{false};
  double milliseconds{0.0};
  Eigen::VectorXcd solution;
};

std::pair<SparseComplex, Eigen::VectorXd> symmetric_equilibrate(
    const SparseComplex& matrix,
    int iterations = 4) {
  Eigen::VectorXd scale = Eigen::VectorXd::Ones(matrix.rows());
  for (int iteration = 0; iteration < iterations; ++iteration) {
    Eigen::VectorXd row_max = Eigen::VectorXd::Zero(matrix.rows());
    for (int col = 0; col < matrix.outerSize(); ++col) {
      for (SparseComplex::InnerIterator entry(matrix, col); entry; ++entry) {
        const double magnitude =
            scale[entry.row()] * std::abs(entry.value()) * scale[entry.col()];
        row_max[entry.row()] = std::max(row_max[entry.row()], magnitude);
      }
    }
    for (int row = 0; row < row_max.size(); ++row) {
      if (row_max[row] > 1e-30 && std::isfinite(row_max[row])) {
        scale[row] /= std::sqrt(row_max[row]);
      }
    }
  }

  std::vector<Eigen::Triplet<Complex>> triplets;
  triplets.reserve(static_cast<std::size_t>(matrix.nonZeros()));
  for (int col = 0; col < matrix.outerSize(); ++col) {
    for (SparseComplex::InnerIterator entry(matrix, col); entry; ++entry) {
      triplets.emplace_back(
          entry.row(), entry.col(),
          scale[entry.row()] * entry.value() * scale[entry.col()]);
    }
  }
  SparseComplex scaled(matrix.rows(), matrix.cols());
  scaled.setFromTriplets(triplets.begin(), triplets.end());
  scaled.makeCompressed();
  return {std::move(scaled), std::move(scale)};
}

TimedSolve timed_sparse_solve(
    const SparseComplex& matrix,
    const Eigen::VectorXcd& rhs,
    int repeats = 7) {
  TimedSolve result;
  std::vector<double> timings;
  timings.reserve(static_cast<std::size_t>(repeats));
  auto [scaled_matrix, scale] = symmetric_equilibrate(matrix);
  const Eigen::VectorXcd scaled_rhs = scale.cast<Complex>().cwiseProduct(rhs);
  for (int repeat = 0; repeat < repeats; ++repeat) {
    const auto start = std::chrono::steady_clock::now();
    Eigen::SparseLU<SparseComplex, Eigen::COLAMDOrdering<int>> solver;
    solver.analyzePattern(scaled_matrix);
    solver.factorize(scaled_matrix);
    if (solver.info() != Eigen::Success) return result;
    Eigen::VectorXcd scaled_solution = solver.solve(scaled_rhs);
    if (solver.info() != Eigen::Success || !scaled_solution.allFinite()) return result;
    Eigen::VectorXcd solution =
        scale.cast<Complex>().cwiseProduct(scaled_solution);
    for (int refinement = 0; refinement < 3; ++refinement) {
      const Eigen::VectorXcd residual = rhs - matrix * solution;
      if (max_abs(residual) <= 1e-12 * std::max(1.0, max_abs(rhs))) break;
      const Eigen::VectorXcd scaled_residual =
          scale.cast<Complex>().cwiseProduct(residual);
      const Eigen::VectorXcd scaled_correction = solver.solve(scaled_residual);
      if (solver.info() != Eigen::Success || !scaled_correction.allFinite()) {
        return result;
      }
      solution += scale.cast<Complex>().cwiseProduct(scaled_correction);
    }
    const double elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    timings.push_back(elapsed);
    result.solution = std::move(solution);
    result.success = true;
  }
  std::sort(timings.begin(), timings.end());
  result.milliseconds = timings[timings.size() / 2];
  return result;
}

void benchmark_solve(
    const SparseComplex& full_ybus,
    const SparseReduction& reduction,
    CaseAudit& audit) {
  Eigen::VectorXcd reduced_rhs(reduction.reduced.rows());
  for (int idx = 0; idx < reduced_rhs.size(); ++idx) {
    reduced_rhs[idx] = Complex(
        0.01 * static_cast<double>((idx * 17) % 19 - 9),
        0.01 * static_cast<double>((idx * 29) % 23 - 11));
  }
  Eigen::VectorXcd full_rhs = Eigen::VectorXcd::Zero(full_ybus.rows());
  for (int pos = 0; pos < static_cast<int>(reduction.retained.size()); ++pos) {
    full_rhs[reduction.retained[static_cast<std::size_t>(pos)]] = reduced_rhs[pos];
  }

  const TimedSolve full = timed_sparse_solve(full_ybus, full_rhs);
  const TimedSolve reduced = timed_sparse_solve(reduction.reduced, reduced_rhs);
  if (!full.success || !reduced.success) return;

  const Eigen::VectorXcd recovered = recover_full_state(
      full_ybus.rows(), reduction.retained, reduction.recovery_steps,
      reduced.solution);
  audit.max_solve_state_error =
      max_abs(full.solution - recovered) / std::max(1.0, max_abs(full.solution));
  audit.full_solve_ms = full.milliseconds;
  audit.reduced_solve_ms = reduced.milliseconds;
  if (reduced.milliseconds > 0.0) {
    audit.solve_speedup = full.milliseconds / reduced.milliseconds;
  }
}

CaseAudit audit_case(const CaseInput& input) {
  CaseAudit audit;
  audit.label = input.label;
  audit.master = input.master;
  try {
    if (!fs::exists(input.master)) {
      throw std::runtime_error("master file not found");
    }
    const ThreePhaseACSystem sys =
        hacdcpf::analysis::load_three_phase_system_from_opendss(input.master);
    if (sys.buses.empty()) {
      throw std::runtime_error(
          "OpenDSS import returned an empty system; rebuild with "
          "HACDCPF_HAVE_OPENDSS enabled");
    }
    const auto compact = hacdcpf::analysis::build_compact_pf_data(sys, true);
    const PhaseNodeIndexer& indexer = compact.indexer;

    audit.loaded = true;
    audit.buses = static_cast<int>(sys.buses.size());
    audit.phase_nodes = indexer.total_nodes;
    audit.lines = static_cast<int>(sys.lines.size());
    audit.transformers = static_cast<int>(sys.transformers.size());
    audit.loads = static_cast<int>(sys.loads.size());
    audit.generators = static_cast<int>(sys.generators.size());
    audit.external_grids = static_cast<int>(sys.external_grids.size());
    audit.ybus_nonzeros = static_cast<int>(compact.ybus_entries.size());

    std::vector<bool> eligible(static_cast<std::size_t>(indexer.total_nodes), false);
    for (const auto& node : indexer.nodes) {
      const auto& bus = sys.buses.at(static_cast<std::size_t>(node.bus_offset));
      if (bus.bus_type == hacdcpf::BusType::SLACK) continue;
      if (!phase_has_independent_injection(
              sys, node.bus_offset, node.phase_index)) {
        eligible[static_cast<std::size_t>(node.compact_index)] = true;
        ++audit.zero_injection_phase_nodes;
      }
    }

    const SparseComplex full_ybus = build_sparse_ybus(
        indexer.total_nodes, compact.ybus_entries);
    const SparseReduction reduction =
        reduce_sparse_phase_graph(full_ybus, eligible);
    audit.retained_phase_nodes = static_cast<int>(reduction.retained.size());
    audit.eliminated_phase_nodes =
        static_cast<int>(reduction.recovery_steps.size());
    audit.reduced_nonzeros = static_cast<int>(reduction.reduced.nonZeros());
    audit.max_elimination_front = reduction.max_front;
    audit.reduced_to_full_nnz = audit.ybus_nonzeros > 0
        ? static_cast<double>(audit.reduced_nonzeros) /
              static_cast<double>(audit.ybus_nonzeros)
        : 0.0;
    audit.reduction_ms = reduction.elapsed_ms;
    validate_reduction(full_ybus, eligible, reduction, audit);
    benchmark_solve(full_ybus, reduction, audit);
  } catch (const std::exception& ex) {
    audit.error = ex.what();
  }
  return audit;
}

std::vector<CaseInput> default_cases() {
  const fs::path root = project_root();
  const fs::path refs = root / "external_data" / "opendss_ieee_pes" /
                        "opendss_reference";
  return {
      {"IEEE13", refs / "13_node" / "official_full" / "IEEE13Nodeckt.dss"},
      {"IEEE34", refs / "34_node" / "ieee34Mod2.dss"},
      {"IEEE123", refs / "123_node" / "IEEE123Master.dss"},
      {"IEEE8500", refs / "8500_node" / "Master.dss"},
  };
}

void write_csv(const fs::path& path, const std::vector<CaseAudit>& audits) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open output CSV: " + path.string());
  out << "case,loaded,buses,phase_nodes,lines,transformers,loads,generators,"
         "external_grids,zero_injection_phase_nodes,ybus_nonzeros,retained_nodes,"
         "eliminated_nodes,reduced_nonzeros,max_front,reduced_to_full_nnz,"
         "reduction_ms,interior_current_residual,boundary_current_error,"
         "solve_state_error,full_solve_ms,reduced_solve_ms,solve_speedup,error,master\n";
  for (const auto& row : audits) {
    out << row.label << ',' << (row.loaded ? "yes" : "no") << ','
        << row.buses << ',' << row.phase_nodes << ',' << row.lines << ','
        << row.transformers << ',' << row.loads << ',' << row.generators << ','
        << row.external_grids << ',' << row.zero_injection_phase_nodes << ','
        << row.ybus_nonzeros << ',' << row.retained_phase_nodes << ','
        << row.eliminated_phase_nodes << ',' << row.reduced_nonzeros << ','
        << row.max_elimination_front << ',' << row.reduced_to_full_nnz << ','
        << row.reduction_ms << ',' << row.max_interior_current_residual << ','
        << row.max_boundary_current_error << ',' << row.max_solve_state_error << ','
        << row.full_solve_ms << ',' << row.reduced_solve_ms << ','
        << row.solve_speedup << ",\"" << row.error << "\",\""
        << row.master.string() << "\"\n";
  }
}

std::string latex_scientific(double value) {
  if (!std::isfinite(value)) return "--";
  if (value == 0.0) return "$0$";
  std::ostringstream stream;
  stream << std::scientific << std::setprecision(2) << value;
  const std::string encoded = stream.str();
  const std::size_t exponent_pos = encoded.find('e');
  if (exponent_pos == std::string::npos) return "$" + encoded + "$";
  const std::string mantissa = encoded.substr(0, exponent_pos);
  const int exponent = std::stoi(encoded.substr(exponent_pos + 1));
  return "$" + mantissa + "\\times10^{" + std::to_string(exponent) + "}$";
}

void write_latex(const fs::path& path, const std::vector<CaseAudit>& audits) {
  fs::create_directories(path.parent_path());
  std::ofstream out(path);
  if (!out) throw std::runtime_error("cannot open output TeX: " + path.string());
  out << "% Auto-generated by tools/phase_graph_reduction_benchmark.cpp\n"
         "\\begin{table*}[t]\n"
         "\\caption{Executable AC-Feeder Phase-Node Reduction Certificate "
         "(Not Monolithic Hybrid OPF Timing)}\n"
         "\\label{tab:reduction_certificate}\n"
         "\\centering\n\\footnotesize\n"
         "\\begin{tabular}{lrrrrrrrr}\n\\toprule\n"
         "Feeder & $n_{\\phi}$ & Eliminated & Retained & $\\mathrm{nnz}_r/"
         "\\mathrm{nnz}$ & Reduction (ms) & Boundary error & State error & "
         "LU speedup\\\\\n\\midrule\n";
  out << std::scientific;
  for (const auto& row : audits) {
    const double eliminated_pct = row.phase_nodes > 0
        ? 100.0 * static_cast<double>(row.eliminated_phase_nodes) /
              static_cast<double>(row.phase_nodes)
        : 0.0;
    out << row.label << " & " << row.phase_nodes << " & "
        << row.eliminated_phase_nodes << " (" << std::fixed
        << std::setprecision(1) << eliminated_pct << "\\%) & "
        << row.retained_phase_nodes << " & " << std::setprecision(3)
        << row.reduced_to_full_nnz << " & " << row.reduction_ms << " & "
        << latex_scientific(row.max_boundary_current_error) << " & "
        << latex_scientific(row.max_solve_state_error) << " & " << std::fixed
        << std::setprecision(2) << row.solve_speedup << "$\\times$\\\\\n";
  }
  out << "\\bottomrule\n\\end{tabular}\n"
         "\\end{table*}\n";
}

}  // namespace

int main(int argc, char** argv) {
  const fs::path output = argc > 1
      ? fs::path(argv[1])
      : project_root() / "output" / "benchmarks" /
            "phase_graph_reduction_case_audit.csv";

  std::vector<CaseAudit> audits;
  for (const auto& input : default_cases()) {
    CaseAudit audit = audit_case(input);
    std::cout << audit.label << ": loaded=" << (audit.loaded ? "yes" : "no")
              << ", buses=" << audit.buses
              << ", phase_nodes=" << audit.phase_nodes
              << ", zero_injection=" << audit.zero_injection_phase_nodes
              << ", eliminated=" << audit.eliminated_phase_nodes
              << ", reduction_ms=" << audit.reduction_ms
              << ", boundary_error=" << audit.max_boundary_current_error
              << ", speedup=" << audit.solve_speedup;
    if (!audit.error.empty()) std::cout << ", error=" << audit.error;
    std::cout << '\n';
    audits.push_back(std::move(audit));
  }
  write_csv(output, audits);
  write_latex(
      project_root() / "docs" / "papers" /
          "phase_graph_reduced_hybrid_opf" /
          "reduction_certificate_table.tex",
      audits);
  std::cout << "Wrote " << output << '\n';
  return std::all_of(
      audits.begin(), audits.end(), [](const CaseAudit& row) { return row.loaded; })
      ? 0
      : 2;
}
