#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#if defined(__APPLE__) || defined(__linux__)
#include <sys/resource.h>
#endif

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"

namespace {

enum class StartMode {
  Cold,
  PhaseOne,
  PowerFlow,
  PowerFlowPhaseOne,
  PreparedSession,
  PreparedPowerFlow,
  PreparedSweep,
  EconomicPowerFlow,
  StructuredPowerFlow,
  DcOpf,
  DcOpfPowerFlow,
  DcOpfHighs,
  DcOpfHighsPowerFlow,
  DcOpfGurobi,
  DcOpfGurobiPowerFlow
};

struct Arguments {
  std::filesystem::path case_path;
  StartMode mode{StartMode::PhaseOne};
  int warmups{1};
  int repeats{3};
  int max_iterations{300};
  int dc_max_iterations{0};
  double dc_time_limit_ms{2000.0};
  bool dc_time_limit_explicit{false};
  double phase_one_admission_factor{1.0};
  bool use_ipopt{false};
};

StartMode parse_mode(std::string_view value) {
  if (value == "cold") return StartMode::Cold;
  if (value == "phase-one") return StartMode::PhaseOne;
  if (value == "power-flow") return StartMode::PowerFlow;
  if (value == "power-flow-phase-one") return StartMode::PowerFlowPhaseOne;
  if (value == "prepared-session") return StartMode::PreparedSession;
  if (value == "prepared-power-flow") return StartMode::PreparedPowerFlow;
  if (value == "prepared-sweep") return StartMode::PreparedSweep;
  if (value == "economic-power-flow") return StartMode::EconomicPowerFlow;
  if (value == "structured-power-flow") return StartMode::StructuredPowerFlow;
  if (value == "dc-opf") return StartMode::DcOpf;
  if (value == "dc-opf-power-flow") return StartMode::DcOpfPowerFlow;
  if (value == "dc-opf-highs") return StartMode::DcOpfHighs;
  if (value == "dc-opf-highs-power-flow") {
    return StartMode::DcOpfHighsPowerFlow;
  }
  if (value == "dc-opf-gurobi") return StartMode::DcOpfGurobi;
  if (value == "dc-opf-gurobi-power-flow") {
    return StartMode::DcOpfGurobiPowerFlow;
  }
  throw std::invalid_argument("unknown start mode: " + std::string(value));
}

const char* mode_name(StartMode mode) {
  switch (mode) {
    case StartMode::Cold: return "cold";
    case StartMode::PhaseOne: return "phase-one";
    case StartMode::PowerFlow: return "power-flow";
    case StartMode::PowerFlowPhaseOne: return "power-flow-phase-one";
    case StartMode::PreparedSession: return "prepared-session";
    case StartMode::PreparedPowerFlow: return "prepared-power-flow";
    case StartMode::PreparedSweep: return "prepared-sweep";
    case StartMode::EconomicPowerFlow: return "economic-power-flow";
    case StartMode::StructuredPowerFlow: return "structured-power-flow";
    case StartMode::DcOpf: return "dc-opf";
    case StartMode::DcOpfPowerFlow: return "dc-opf-power-flow";
    case StartMode::DcOpfHighs: return "dc-opf-highs";
    case StartMode::DcOpfHighsPowerFlow: return "dc-opf-highs-power-flow";
    case StartMode::DcOpfGurobi: return "dc-opf-gurobi";
    case StartMode::DcOpfGurobiPowerFlow: return "dc-opf-gurobi-power-flow";
  }
  return "unknown";
}

int parse_nonnegative(const char* value, const char* option) {
  const int parsed = std::stoi(value);
  if (parsed < 0) {
    throw std::invalid_argument(std::string(option) + " must be nonnegative");
  }
  return parsed;
}

Arguments parse_arguments(int argc, char** argv) {
  if (argc < 2) {
    throw std::invalid_argument(
        "usage: opf_numerical_benchmark CASE [--mode MODE] [--warmups N] "
        "[--repeats N] [--max-iterations N] "
        "[--dc-max-iterations N] "
        "[--dc-time-limit-ms MS] "
        "[--phase-one-admission FACTOR]");
  }
  Arguments args;
  args.case_path = argv[1];
  for (int i = 2; i < argc; ++i) {
    const std::string_view option = argv[i];
    if (i + 1 >= argc) {
      throw std::invalid_argument("missing value for " + std::string(option));
    }
    const char* value = argv[++i];
    if (option == "--mode") {
      args.mode = parse_mode(value);
    } else if (option == "--warmups") {
      args.warmups = parse_nonnegative(value, "--warmups");
    } else if (option == "--repeats") {
      args.repeats = parse_nonnegative(value, "--repeats");
    } else if (option == "--max-iterations") {
      args.max_iterations = parse_nonnegative(value, "--max-iterations");
    } else if (option == "--dc-max-iterations") {
      args.dc_max_iterations = parse_nonnegative(value, "--dc-max-iterations");
    } else if (option == "--dc-time-limit-ms") {
      args.dc_time_limit_ms = std::stod(value);
      args.dc_time_limit_explicit = true;
      if (!std::isfinite(args.dc_time_limit_ms) ||
          args.dc_time_limit_ms < 0.0) {
        throw std::invalid_argument(
            "--dc-time-limit-ms must be nonnegative and finite");
      }
    } else if (option == "--phase-one-admission") {
      args.phase_one_admission_factor = std::stod(value);
      if (!std::isfinite(args.phase_one_admission_factor) ||
          args.phase_one_admission_factor < 0.0) {
        throw std::invalid_argument(
            "--phase-one-admission must be nonnegative and finite");
      }
    } else if (option == "--backend") {
      const std::string_view backend = value;
      if (backend == "ipopt") {
        args.use_ipopt = true;
      } else if (backend == "parity" || backend == "native") {
        args.use_ipopt = false;
      } else {
        throw std::invalid_argument(
            "--backend must be ipopt, parity, or native");
      }
    } else {
      throw std::invalid_argument("unknown option: " + std::string(option));
    }
  }
  if (args.repeats == 0 || args.max_iterations == 0) {
    throw std::invalid_argument("--repeats and --max-iterations must be positive");
  }
  return args;
}

double median(std::vector<double> samples) {
  std::sort(samples.begin(), samples.end());
  return samples[samples.size() / 2];
}

double peak_rss_mib() {
#if defined(__APPLE__) || defined(__linux__)
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
#if defined(__APPLE__)
  // getrusage(2): Darwin reports ru_maxrss in bytes; Linux reports KiB.
  constexpr double kNativeUnitsPerMiB = 1024.0 * 1024.0;
#else
  constexpr double kNativeUnitsPerMiB = 1024.0;
#endif
  return static_cast<double>(usage.ru_maxrss) / kNativeUnitsPerMiB;
#else
  return std::numeric_limits<double>::quiet_NaN();
#endif
}

hacdcpf::opf::ACOPFOptions options_for(const Arguments& args) {
  hacdcpf::opf::ACOPFOptions options;
  options.ac_solver_backend =
      args.use_ipopt ? hacdcpf::opf::ACOPFSolverBackend::Ipopt
                     : hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.max_inner_iterations = args.max_iterations;
  options.max_outer_iterations = 1;
  options.enable_phase_one =
      !args.use_ipopt &&
      (args.mode == StartMode::PhaseOne ||
       args.mode == StartMode::PowerFlowPhaseOne ||
       args.mode == StartMode::PreparedSession ||
       args.mode == StartMode::PreparedSweep);
  options.phase_one_dispatch_dual_predictor =
      args.mode == StartMode::PowerFlowPhaseOne;
  options.ac_pf_warm_start =
      !args.use_ipopt &&
      (args.mode == StartMode::PowerFlow ||
       args.mode == StartMode::PowerFlowPhaseOne ||
       args.mode == StartMode::PreparedPowerFlow ||
       args.mode == StartMode::EconomicPowerFlow ||
       args.mode == StartMode::StructuredPowerFlow ||
       args.mode == StartMode::DcOpfPowerFlow ||
       args.mode == StartMode::DcOpfHighsPowerFlow ||
       args.mode == StartMode::DcOpfGurobiPowerFlow);
  options.phase_one_admission_mu_factor = args.phase_one_admission_factor;
  options.ac_pf_dc_phase_one = args.mode == StartMode::StructuredPowerFlow;
  options.ac_pf_dc_phase_one_time_limit_ms = args.dc_time_limit_ms;
  if (args.dc_max_iterations > 0) {
    options.ac_pf_dc_phase_one_max_iterations = args.dc_max_iterations;
  }
  return options;
}

void print_result(const Arguments& args,
                  const std::vector<double>& samples,
                  const hacdcpf::opf::ACOPFResult& result,
                  const hacdcpf::HybridPowerSystem& system) {
  const auto& p = result.profiling;
  std::cout << std::setprecision(12)
            << "case=" << args.case_path.filename().string()
            << " mode=" << mode_name(args.mode)
            << " warmups=" << args.warmups
            << " repeats=" << args.repeats
            << " ac_buses=" << system.ac.buses.size()
            << " generators=" << system.ac.generators.size()
            << " ac_branches=" << system.ac.branches.size()
            << " phase_one_admission_factor="
            << args.phase_one_admission_factor
            << " requested_dc_time_limit_ms=" << args.dc_time_limit_ms
            << " median_ms=" << median(samples)
            << " prepared_session_used=" << p.prepared_session_used
            << " formulation_reused=" << p.formulation_reused
            << " mapping_reused=" << p.mapping_reused
            << " symbolic_reused=" << p.symbolic_reused
            << " continuation_state_reused="
            << p.continuation_state_reused
            << " numeric_refactor_attempted="
            << p.numeric_refactor_attempted
            << " numeric_refactor_accepted="
            << p.numeric_refactor_accepted
            << " numeric_refactor_status=" << p.numeric_refactor_status
            << " prepared_invalidation="
            << p.prepared_session_invalidation_reason
            << " samples_ms=";
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (i != 0) std::cout << ',';
    std::cout << samples[i];
  }
  std::cout << '\n'
            << "converged=" << std::boolalpha << result.converged
            << " iterations=" << result.iterations
            << " objective=" << result.objective
            << " primal=" << result.max_constraint_violation
            << " dual=" << result.max_stationarity
            << " backend=" << p.linear_solver_backend
            << " factorizations=" << p.factorization_calls
            << " solves=" << p.linear_solve_calls
            << " analyze_calls=" << p.analyze_calls
            << " scaling_rebuilds=" << p.scaling_rebuilds
            << " accepted_steps=" << p.accepted_steps
            << " rejected_steps=" << p.rejected_steps << '\n'
            << "initial_primal=" << p.initial_primal_residual
            << " initial_dual=" << p.initial_dual_residual
            << " dc_phase_one_requested=" << p.dc_phase_one_requested
            << " dc_phase_one_accepted=" << p.dc_phase_one_accepted
            << " dc_phase_one_runtime_ms=" << p.dc_phase_one_runtime_ms
            << " dc_phase_one_time_limit_ms=" << p.dc_phase_one_time_limit_ms
            << " dc_phase_one_budget_exhausted="
            << p.dc_phase_one_budget_exhausted
            << " dc_phase_one_budget_overshoot_ms="
            << p.dc_phase_one_budget_overshoot_ms
            << " dc_phase_one_symbolic_analyze_calls="
            << p.dc_phase_one_symbolic_analyze_calls
            << " parity_formulation_builds=" << p.parity_formulation_builds
            << " dc_phase_one_iterations=" << p.dc_phase_one_iterations
            << " dc_phase_one_residual=" << p.dc_phase_one_residual
            << " dc_candidate_primal=" << p.dc_phase_one_candidate_primal
            << " dc_candidate_dual=" << p.dc_phase_one_candidate_dual
            << " dc_baseline_primal=" << p.dc_phase_one_baseline_primal
            << " dc_baseline_dual=" << p.dc_phase_one_baseline_dual
            << " dc_phase_one_status=" << p.dc_phase_one_status
            << " phase_one_initial=" << p.phase_one_initial_violation
            << " phase_one_final=" << p.phase_one_constraint_violation
            << " phase_one_runtime_ms=" << p.phase_one_runtime_ms
            << " phase_one_iterations=" << p.phase_one_iterations
            << " phase_one_factorizations=" << p.phase_one_factorizations
            << " phase_one_accepted=" << p.phase_two_start_accepted
            << " phase_one_termination=" << p.phase_one_termination
            << " dispatch_predictor_attempted="
            << p.dispatch_dual_predictor_attempted
            << " dispatch_predictor_accepted="
            << p.dispatch_dual_predictor_accepted
            << " dispatch_predictor_runtime_ms="
            << p.dispatch_dual_predictor_runtime_ms
            << " dispatch_predictor_baseline_raw="
            << p.dispatch_dual_predictor_baseline_raw
            << " dispatch_predictor_candidate_raw="
            << p.dispatch_dual_predictor_candidate_raw
            << " dispatch_predictor_baseline_normalized="
            << p.dispatch_dual_predictor_baseline_normalized
            << " dispatch_predictor_candidate_normalized="
            << p.dispatch_dual_predictor_candidate_normalized
            << " dispatch_predictor_status="
            << p.dispatch_dual_predictor_status
            << " peak_rss_mib=" << peak_rss_mib()
            << " status=" << result.status << '\n';
}

int run_dc_opf_benchmark(const Arguments& args,
                         const hacdcpf::HybridPowerSystem& system) {
  hacdcpf::opf::DCOPFOptions options;
  options.solver = args.mode == StartMode::DcOpfHighs
                       ? hacdcpf::opf::DCOPFSolverBackend::HiGHS
                   : args.mode == StartMode::DcOpfGurobi
                       ? hacdcpf::opf::DCOPFSolverBackend::Gurobi
                       : hacdcpf::opf::DCOPFSolverBackend::NativeQP;
  options.compute_lmp = false;
  options.structural_warm_start = true;
  options.load_shedding = false;
  options.compact_quadratic_model =
      args.mode != StartMode::DcOpfHighs;
  if (args.dc_max_iterations > 0) {
    options.max_iterations = args.dc_max_iterations;
    options.accept_phase_one_iterate = true;
  }
  options.phase_one_linear_relaxation =
      args.mode == StartMode::DcOpfHighs;
  if (args.dc_time_limit_explicit && args.dc_time_limit_ms > 0.0) {
    options.accept_phase_one_iterate = true;
    options.phase_one_time_limit_ms = args.dc_time_limit_ms;
  }
  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(args.repeats));
  hacdcpf::opf::DCOPFResult result;
  for (int run = -args.warmups; run < args.repeats; ++run) {
    const auto started = std::chrono::steady_clock::now();
    result = hacdcpf::opf::solve_dc_opf(system, options);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (run >= 0) samples.push_back(elapsed_ms);
  }
  std::cout << std::setprecision(12)
            << "case=" << args.case_path.filename().string()
            << " mode=dc-opf"
            << " warmups=" << args.warmups
            << " repeats=" << args.repeats
            << " median_ms=" << median(samples)
            << " converged=" << std::boolalpha << result.converged
            << " iterations=" << result.iterations
            << " objective=" << result.objective
            << " projection_residual="
            << result.structural_warm_start_residual
            << " solver_initial_primal="
            << result.solver_initial_primal_residual
            << " phase_one_only=" << result.phase_one_warm_start_only
            << " phase_one_residual=" << result.phase_one_iterate_residual
            << " budget_exhausted=" << result.phase_one_budget_exhausted
            << " budget_overshoot_ms=" << result.phase_one_budget_overshoot_ms
            << " symbolic_analyze_calls="
            << result.native_qp_symbolic_analyze_calls
            << " peak_rss_mib=" << peak_rss_mib()
            << " status=" << result.status << '\n';
  return result.converged || result.phase_one_warm_start_only
             ? EXIT_SUCCESS : EXIT_FAILURE;
}

hacdcpf::HybridPowerSystem seed_from_dc_opf(
    const hacdcpf::HybridPowerSystem& system,
    const hacdcpf::opf::DCOPFResult& dc) {
  if (!dc.converged && !dc.phase_one_warm_start_only) {
    throw std::runtime_error("DCOPF seed did not converge: " + dc.status);
  }
  if (dc.pg_mw.size() != system.ac.generators.size()) {
    throw std::runtime_error("DCOPF generator result does not match authored generators");
  }
  if (dc.va.size() != system.ac.buses.size()) {
    throw std::runtime_error("DCOPF angle result does not match authored AC buses");
  }
  if (!system.ac.external_grids.empty() &&
      dc.external_grid_p_mw.size() != system.ac.external_grids.size()) {
    throw std::runtime_error("DCOPF external-grid result does not match authored sources");
  }

  hacdcpf::HybridPowerSystem seeded = system;
  for (std::size_t i = 0; i < seeded.ac.generators.size(); ++i) {
    seeded.ac.generators[i].pg_mw = dc.pg_mw[i];
  }
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  for (std::size_t i = 0; i < seeded.ac.buses.size(); ++i) {
    seeded.ac.buses[i].va_deg = dc.va[i] * kRadToDeg;
  }
  return seeded;
}

hacdcpf::HybridPowerSystem seed_from_economic_dispatch(
    const hacdcpf::HybridPowerSystem& system) {
  hacdcpf::HybridPowerSystem seeded = system;
  double target_mw = 0.0;
  for (const auto& bus : seeded.ac.buses) {
    if (bus.in_service) target_mw += bus.pd_mw;
  }
  for (const auto& load : seeded.ac.loads) {
    if (load.in_service) target_mw += load.p_mw * load.scaling;
  }
  for (const auto& generator : seeded.ac.static_generators) {
    if (generator.in_service) target_mw -= generator.p_mw * generator.scaling;
  }

  double sum_pmin = 0.0;
  double sum_pmax = 0.0;
  double lambda_lo = std::numeric_limits<double>::infinity();
  double lambda_hi = -std::numeric_limits<double>::infinity();
  for (const auto& generator : seeded.ac.generators) {
    if (!generator.in_service) continue;
    const double lo = std::min(generator.pmin_mw, generator.pmax_mw);
    const double hi = std::max(generator.pmin_mw, generator.pmax_mw);
    sum_pmin += lo;
    sum_pmax += hi;
    lambda_lo = std::min(
        lambda_lo, generator.cost_c1 -
                       2.0 * std::abs(generator.cost_c2) *
                           std::max(1.0, std::abs(hi)));
    lambda_hi = std::max(
        lambda_hi, generator.cost_c1 +
                       2.0 * std::abs(generator.cost_c2) *
                           std::max(1.0, std::abs(hi)));
  }
  target_mw = std::clamp(target_mw, sum_pmin, sum_pmax);
  if (!std::isfinite(lambda_lo) || !std::isfinite(lambda_hi) ||
      lambda_lo >= lambda_hi) {
    lambda_lo = -1.0e3;
    lambda_hi = 1.0e3;
  }
  const auto total_at = [&](double lambda, bool write) {
    double total = 0.0;
    for (auto& generator : seeded.ac.generators) {
      if (!generator.in_service) continue;
      const double lo = std::min(generator.pmin_mw, generator.pmax_mw);
      const double hi = std::max(generator.pmin_mw, generator.pmax_mw);
      const double pg = generator.cost_c2 > 1.0e-12
                            ? std::clamp(
                                  (lambda - generator.cost_c1) /
                                      (2.0 * generator.cost_c2),
                                  lo, hi)
                            : (lambda >= generator.cost_c1 ? hi : lo);
      total += pg;
      if (write) generator.pg_mw = pg;
    }
    return total;
  };
  for (int iteration = 0; iteration < 80; ++iteration) {
    const double middle = 0.5 * (lambda_lo + lambda_hi);
    if (total_at(middle, false) < target_mw) {
      lambda_lo = middle;
    } else {
      lambda_hi = middle;
    }
  }
  total_at(0.5 * (lambda_lo + lambda_hi), true);
  return seeded;
}

int run_dc_opf_power_flow_benchmark(
    const Arguments& args, const hacdcpf::HybridPowerSystem& system) {
  hacdcpf::opf::DCOPFOptions dc_options;
  dc_options.solver =
      args.mode == StartMode::DcOpfHighsPowerFlow
          ? hacdcpf::opf::DCOPFSolverBackend::HiGHS
      : args.mode == StartMode::DcOpfGurobiPowerFlow
          ? hacdcpf::opf::DCOPFSolverBackend::Gurobi
          : hacdcpf::opf::DCOPFSolverBackend::NativeQP;
  dc_options.compute_lmp = false;
  dc_options.structural_warm_start = true;
  dc_options.load_shedding = false;
  dc_options.compact_quadratic_model =
      args.mode != StartMode::DcOpfHighsPowerFlow;
  if (args.dc_max_iterations > 0) {
    dc_options.max_iterations = args.dc_max_iterations;
    dc_options.accept_phase_one_iterate = true;
  }
  dc_options.phase_one_linear_relaxation =
      args.mode == StartMode::DcOpfHighsPowerFlow;
  if (args.dc_time_limit_explicit && args.dc_time_limit_ms > 0.0) {
    dc_options.accept_phase_one_iterate = true;
    dc_options.phase_one_time_limit_ms = args.dc_time_limit_ms;
  }
  const hacdcpf::opf::ACOPFOptions ac_options = options_for(args);

  std::vector<double> samples;
  samples.reserve(static_cast<std::size_t>(args.repeats));
  hacdcpf::opf::DCOPFResult dc_result;
  hacdcpf::opf::ACOPFResult ac_result;
  for (int run = -args.warmups; run < args.repeats; ++run) {
    const auto started = std::chrono::steady_clock::now();
    dc_result = hacdcpf::opf::solve_dc_opf(system, dc_options);
    const hacdcpf::HybridPowerSystem seeded =
        seed_from_dc_opf(system, dc_result);
    ac_result = hacdcpf::opf::solve_ac_opf(seeded, ac_options);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    if (run >= 0) samples.push_back(elapsed_ms);
  }
  print_result(args, samples, ac_result, system);
  std::cout << std::setprecision(12)
            << "dc_converged=" << std::boolalpha << dc_result.converged
            << " dc_iterations=" << dc_result.iterations
            << " dc_objective=" << dc_result.objective
            << " dc_projection_residual="
            << dc_result.structural_warm_start_residual
            << " dc_solver_initial_primal="
            << dc_result.solver_initial_primal_residual
            << " dc_phase_one_only="
            << dc_result.phase_one_warm_start_only
            << " dc_phase_one_residual="
            << dc_result.phase_one_iterate_residual
            << " dc_budget_exhausted="
            << dc_result.phase_one_budget_exhausted
            << " dc_budget_overshoot_ms="
            << dc_result.phase_one_budget_overshoot_ms
            << " dc_symbolic_analyze_calls="
            << dc_result.native_qp_symbolic_analyze_calls
            << " dc_status=" << dc_result.status << '\n';
  return ac_result.converged ? EXIT_SUCCESS : EXIT_FAILURE;
}

// Load a case by path, or synthesize an internal hybrid AC/DC fixture when the
// path stem names one (so the benchmark can trace hybrid cases that have no
// MATPOWER file).
hacdcpf::HybridPowerSystem load_case_system(const std::filesystem::path& p) {
  const std::string stem = p.stem().string();
  if (stem == "case300_acdc") return hacdcpf::io::build_case300_acdc();
  if (stem == "case2000_acdc") return hacdcpf::io::build_case2000_acdc();
  if (!std::filesystem::is_regular_file(p)) {
    throw std::invalid_argument("case file not found: " + p.string());
  }
  return hacdcpf::io::parse_matpower(p.string());
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments args = parse_arguments(argc, argv);
    const hacdcpf::HybridPowerSystem system = load_case_system(args.case_path);
    if (args.mode == StartMode::DcOpf ||
        args.mode == StartMode::DcOpfHighs ||
        args.mode == StartMode::DcOpfGurobi) {
      return run_dc_opf_benchmark(args, system);
    }
    if (args.mode == StartMode::DcOpfPowerFlow ||
        args.mode == StartMode::DcOpfHighsPowerFlow ||
        args.mode == StartMode::DcOpfGurobiPowerFlow) {
      return run_dc_opf_power_flow_benchmark(args, system);
    }
    const hacdcpf::opf::ACOPFOptions options = options_for(args);
    const bool prepared_mode =
        args.mode == StartMode::PreparedSession ||
        args.mode == StartMode::PreparedPowerFlow ||
        args.mode == StartMode::PreparedSweep;
    std::unique_ptr<hacdcpf::opf::PreparedACOPFSession> prepared_session;
    if (prepared_mode) {
      prepared_session =
          std::make_unique<hacdcpf::opf::PreparedACOPFSession>(options);
    }
    std::vector<double> samples;
    samples.reserve(static_cast<std::size_t>(args.repeats));
    hacdcpf::opf::ACOPFResult result;
    for (int run = -args.warmups; run < args.repeats; ++run) {
      const auto started = std::chrono::steady_clock::now();
      if (prepared_mode) {
        hacdcpf::HybridPowerSystem scenario = system;
        if (args.mode == StartMode::PreparedSweep) {
          constexpr double kLoadFactors[] = {1.0, 1.005, 0.995, 1.02, 0.98};
          const int sequence_index = run + args.warmups;
          const double factor = kLoadFactors[
              static_cast<std::size_t>(sequence_index) %
              std::size(kLoadFactors)];
          for (auto& bus : scenario.ac.buses) {
            bus.pd_mw *= factor;
            bus.qd_mvar *= factor;
          }
          for (auto& load : scenario.ac.loads) {
            load.p_mw *= factor;
            load.q_mvar *= factor;
          }
        }
        result = prepared_session->solve(scenario);
      } else if (args.mode == StartMode::EconomicPowerFlow) {
        const hacdcpf::HybridPowerSystem seeded =
            seed_from_economic_dispatch(system);
        result = hacdcpf::opf::solve_ac_opf(seeded, options);
      } else {
        result = hacdcpf::opf::solve_ac_opf(system, options);
      }
      const double elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count();
      if (run >= 0) samples.push_back(elapsed_ms);
    }
    print_result(args, samples, result, system);
    return result.converged ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& error) {
    std::cerr << "opf_numerical_benchmark: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
