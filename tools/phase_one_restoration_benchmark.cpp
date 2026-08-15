#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>

#include <Eigen/Sparse>

#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/three_phase_hybrid_opf.hpp"

namespace {

using hacdcpf::opf::phase_hybrid::ModelVariant;
using hacdcpf::opf::phase_hybrid::PhaseGenerator;
using hacdcpf::opf::phase_hybrid::PhaseVSC;
using hacdcpf::opf::phase_hybrid::SolverBackend;
using hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFCase;
using hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFOptions;
using hacdcpf::opf::phase_hybrid::ThreePhaseHybridOPFResult;
using Complex = std::complex<double>;

constexpr double kPi = 3.14159265358979323846;

ThreePhaseHybridOPFCase make_case(int bus_count) {
  ThreePhaseHybridOPFCase c;
  c.name = "phase_one_sparse_benchmark";
  c.base_mva = 1.0;
  const int n = 3 * bus_count;
  std::vector<Eigen::Triplet<Complex>> ytrip;
  ytrip.reserve(static_cast<std::size_t>(12 * bus_count));
  const Complex admittance{8.0, -16.0};
  for (int bus = 0; bus + 1 < bus_count; ++bus) {
    for (int phase = 0; phase < 3; ++phase) {
      const int from = 3 * bus + phase;
      const int to = 3 * (bus + 1) + phase;
      ytrip.emplace_back(from, from, admittance);
      ytrip.emplace_back(to, to, admittance);
      ytrip.emplace_back(from, to, -admittance);
      ytrip.emplace_back(to, from, -admittance);
    }
  }
  c.y_ac.resize(n, n);
  c.y_ac.setFromTriplets(ytrip.begin(), ytrip.end());
  c.y_ac.makeCompressed();
  c.i_ac_fixed = Eigen::VectorXcd::Zero(n);
  c.p_load_pu = Eigen::VectorXd::Zero(n);
  c.q_load_pu = Eigen::VectorXd::Zero(n);
  for (int phase = 0; phase < 3; ++phase) {
    c.p_load_pu[n - 3 + phase] = 2e-4;
    c.q_load_pu[n - 3 + phase] = 5e-5;
  }
  c.v_min_pu = Eigen::VectorXd::Constant(n, 0.85);
  c.v_max_pu = Eigen::VectorXd::Constant(n, 1.10);
  c.voltage_start.resize(n);
  c.ac_phase_index.resize(n);
  const std::array<double, 3> angles{
      0.0, -2.0 * kPi / 3.0, 2.0 * kPi / 3.0};
  for (int bus = 0; bus < bus_count; ++bus) {
    for (int phase = 0; phase < 3; ++phase) {
      const int node = 3 * bus + phase;
      c.voltage_start[node] = std::polar(0.98, angles[phase]);
      c.ac_phase_index[static_cast<std::size_t>(node)] = phase;
    }
    c.three_phase_bus_nodes.push_back(
        {3 * bus, 3 * bus + 1, 3 * bus + 2});
  }
  c.reference_nodes = {0, 1, 2};
  c.reference_voltage.resize(3);
  for (int phase = 0; phase < 3; ++phase) {
    c.reference_voltage[phase] = std::polar(1.0, angles[phase]);
    PhaseGenerator generator;
    generator.phase_node = phase;
    generator.p_min_pu = -0.2;
    generator.p_max_pu = 1.0;
    generator.q_min_pu = -1.0;
    generator.q_max_pu = 1.0;
    generator.cost_c2 = 0.1;
    generator.cost_c1 = 30.0;
    c.generators.push_back(generator);
  }
  c.vuf_max = 0.05;

  c.g_dc.resize(2, 2);
  const std::vector<Eigen::Triplet<double>> gtrip{
      {0, 0, 50.0}, {0, 1, -50.0}, {1, 0, -50.0}, {1, 1, 50.0}};
  c.g_dc.setFromTriplets(gtrip.begin(), gtrip.end());
  c.p_dc_load_pu = Eigen::VectorXd::Zero(2);
  c.p_dc_load_pu[1] = 1e-3;
  c.v_dc_start = Eigen::VectorXd::Ones(2);
  c.v_dc_min_pu = Eigen::VectorXd::Constant(2, 0.90);
  c.v_dc_max_pu = Eigen::VectorXd::Constant(2, 1.10);
  c.dc_reference_terminals = {0};
  c.dc_reference_voltage_pu = Eigen::VectorXd::Ones(1);
  PhaseVSC converter;
  converter.phase_nodes = {n - 3, n - 2, n - 1};
  converter.dc_terminal = 0;
  converter.efficiency = 0.98;
  converter.s_max_pu = 0.25;
  c.converters.push_back(converter);
  return c;
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  return values[values.size() / 2];
}

struct BalancedBenchmarkResult {
  double median_ms{0.0};
  hacdcpf::opf::ACOPFResult result;
};

BalancedBenchmarkResult benchmark_balanced(
    const hacdcpf::HybridPowerSystem& system, bool enable_phase_one,
    int max_iterations, double phase_one_barrier_mu = 0.1) {
  constexpr int kWarmups = 2;
  constexpr int kRepeats = 5;
  hacdcpf::opf::ACOPFOptions options;
  options.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.max_inner_iterations = max_iterations;
  options.enable_phase_one = enable_phase_one;
  options.phase_one_barrier_mu = phase_one_barrier_mu;

  std::vector<double> samples;
  samples.reserve(kRepeats);
  hacdcpf::opf::ACOPFResult last;
  for (int repeat = -kWarmups; repeat < kRepeats; ++repeat) {
    const auto started = std::chrono::steady_clock::now();
    last = hacdcpf::opf::solve_ac_opf(system, options);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (repeat >= 0) samples.push_back(elapsed_ms);
  }
  return {median(samples), std::move(last)};
}

void print_balanced_comparison(const char* name,
                               const hacdcpf::HybridPowerSystem& system,
                               int max_iterations,
                               bool sweep_barrier = false) {
  const BalancedBenchmarkResult disabled =
      benchmark_balanced(system, false, max_iterations);
  const BalancedBenchmarkResult enabled =
      benchmark_balanced(system, true, max_iterations);
  const auto print_run = [name](const char* mode,
                                const BalancedBenchmarkResult& run) {
    const auto& result = run.result;
    std::cout << "case=" << name << " mode=" << mode
              << " median_ms=" << run.median_ms
              << " converged=" << std::boolalpha << result.converged
              << " phase_two_iterations=" << result.iterations
              << " final_primal=" << result.max_constraint_violation
              << " final_dual=" << result.max_stationarity
              << " phase_one_runtime_ms="
              << result.profiling.phase_one_runtime_ms
              << " phase_one_initial="
              << result.profiling.phase_one_initial_violation
              << " phase_one_final="
              << result.profiling.phase_one_constraint_violation
              << " phase_one_dual_fit="
              << result.profiling.phase_one_dual_fit_residual
              << " phase_one_perturbed_primal="
              << result.profiling.phase_one_perturbed_primal_residual
              << " phase_one_centrality="
              << result.profiling.phase_one_centrality
              << " phase_one_mu=" << result.profiling.phase_one_barrier_mu
              << " phase_one_structure="
              << result.profiling.phase_one_structure
              << " structural_attempted="
              << result.profiling.phase_one_structural_step_attempted
              << " structural_accepted="
              << result.profiling.phase_one_structural_step_accepted
              << " structural_factorizations="
              << result.profiling.phase_one_structural_factorizations
              << " factorization_calls="
              << result.profiling.factorization_calls
              << " phase_two_accepted="
              << result.profiling.phase_two_start_accepted << '\n';
  };
  print_run("phase-one-off", disabled);
  print_run("phase-one-on", enabled);
  std::cout << "case=" << name << " end_to_end_speedup="
            << disabled.median_ms / enabled.median_ms << '\n';
  if (sweep_barrier) {
    for (double barrier_mu : {0.03, 0.01}) {
      const BalancedBenchmarkResult candidate =
          benchmark_balanced(system, true, max_iterations, barrier_mu);
      print_run(barrier_mu == 0.03 ? "phase-one-mu-0.03"
                                   : "phase-one-mu-0.01",
                candidate);
      std::cout << "case=" << name << " barrier_mu=" << barrier_mu
                << " end_to_end_speedup="
                << disabled.median_ms / candidate.median_ms << '\n';
    }
  }
}

void print_dcopf_comparison(const char* name,
                            const hacdcpf::HybridPowerSystem& system) {
  constexpr int kWarmups = 2;
  constexpr int kRepeats = 5;
  const auto run = [&](bool structural) {
    hacdcpf::opf::DCOPFOptions options;
    options.solver = hacdcpf::opf::DCOPFSolverBackend::NativeQP;
    options.compute_lmp = false;
    options.structural_warm_start = structural;
    std::vector<double> samples;
    hacdcpf::opf::DCOPFResult last;
    for (int repeat = -kWarmups; repeat < kRepeats; ++repeat) {
      const auto started = std::chrono::steady_clock::now();
      last = hacdcpf::opf::solve_dc_opf(system, options);
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      if (repeat >= 0) samples.push_back(elapsed_ms);
    }
    return std::pair{median(samples), std::move(last)};
  };
  const auto cold = run(false);
  const auto warm = run(true);
  const auto print = [name](const char* mode, const auto& sample) {
    const auto& result = sample.second;
    std::cout << "case=" << name << " mode=" << mode
              << " median_ms=" << sample.first
              << " converged=" << result.converged
              << " iterations=" << result.iterations
              << " projection_components="
              << result.structural_warm_start_components
              << " projection_factorizations="
              << result.structural_warm_start_factorizations
              << " projection_residual="
              << result.structural_warm_start_residual
              << " solver_initial_primal="
              << result.solver_initial_primal_residual
              << " warm_used=" << result.structural_warm_start_used
              << " status=" << result.structural_warm_start_status << '\n';
  };
  print("structural-off", cold);
  print("structural-on", warm);
  std::cout << "case=" << name << " end_to_end_speedup="
            << cold.first / warm.first << '\n';
}

}  // namespace

int main() {
  constexpr int kWarmups = 2;
  constexpr int kRepeats = 5;
  const ThreePhaseHybridOPFCase problem = make_case(360);
  ThreePhaseHybridOPFOptions options;
  options.variant = ModelVariant::Full;
  options.backend = SolverBackend::NativeIPM;
  options.max_iterations = 0;
  options.tolerance = 1e-6;
#ifdef HACDCPF_BOUNDED_PHASE_ONE
  options.phase_one_max_iterations = 2;
  options.phase_one_max_factorizations = 5;
  options.phase_one_max_backtracks = 4;
  options.phase_one_time_limit_ms = 5000.0;
#endif

  std::vector<double> samples;
  samples.reserve(kRepeats);
  ThreePhaseHybridOPFResult last;
  for (int repeat = -kWarmups; repeat < kRepeats; ++repeat) {
    const auto started = std::chrono::steady_clock::now();
    last = hacdcpf::opf::phase_hybrid::solve_three_phase_hybrid_opf(
        problem, options);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (repeat >= 0) samples.push_back(elapsed_ms);
  }

  std::cout << std::setprecision(12)
            << "variables=" << last.variables << '\n'
            << "initial_violation="
#ifdef HACDCPF_BOUNDED_PHASE_ONE
            << last.phase_one_initial_violation
#else
            << last.initial_primal_residual
#endif
            << '\n'
            << "final_violation=" << last.phase_one_constraint_violation << '\n'
            << "samples_ms=";
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (i > 0) std::cout << ',';
    std::cout << samples[i];
  }
  std::cout << '\n' << "median_ms=" << median(samples) << '\n';
#ifdef HACDCPF_BOUNDED_PHASE_ONE
  std::cout << "phase_one_runtime_ms=" << last.phase_one_runtime_ms << '\n'
            << "phase_one_iterations=" << last.phase_one_iterations << '\n'
            << "phase_one_factorizations=" << last.phase_one_factorizations
            << '\n'
            << "phase_one_dual_fit=" << last.phase_one_dual_fit_residual << '\n'
            << "phase_one_solver=" << last.phase_one_linear_solver << '\n';
#endif

  const hacdcpf::HybridPowerSystem ac_case = hacdcpf::io::parse_matpower(
      std::string(HACDCPF_MATPOWER_DATA_DIR) + "/case30.m");
  print_balanced_comparison("case30-ac", ac_case, 200, true);
  print_balanced_comparison(
      "hybrid-microgrid",
      hacdcpf::io::build_hybrid_acdc_microgrid_island(), 400, true);
  print_balanced_comparison(
      "case300-acdc", hacdcpf::io::build_case300_acdc(), 200);
  for (const char* name : {"case57", "case118", "case300"}) {
    const hacdcpf::HybridPowerSystem dc_case = hacdcpf::io::parse_matpower(
        std::string(HACDCPF_MATPOWER_DATA_DIR) + "/" + name + ".m");
    print_dcopf_comparison(name, dc_case);
  }
}
