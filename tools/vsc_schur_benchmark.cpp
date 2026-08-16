#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/case_builders.hpp"

namespace {

using Clock = std::chrono::steady_clock;

struct Samples {
  std::vector<double> wall_ms;
  std::vector<double> linear_ms;
  hacdcpf::PowerFlowResult last;
};

double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  return values.size() % 2 == 0
             ? 0.5 * (values[middle - 1] + values[middle])
             : values[middle];
}

void run_sample(const hacdcpf::HybridPowerSystem& system,
                bool enable_schur,
                Samples& samples) {
  hacdcpf::PowerFlowOptions options;
  options.tol = 1e-10;
  options.max_iter = 180;
  options.enable_solver_profiling = true;
  options.robust_nonlinear.enable_vsc_local_schur = enable_schur;
  options.robust_nonlinear.vsc_schur_min_network_dimension = 0;

  const auto start = Clock::now();
  auto result = hacdcpf::solve_power_flow(system, options);
  const double wall_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - start).count();
  if (!result.converged) {
    throw std::runtime_error("VSC Schur benchmark sample did not converge");
  }
  samples.wall_ms.push_back(wall_ms);
  samples.linear_ms.push_back(result.profiling.linear_solve_ms_total);
  samples.last = std::move(result);
}

nlohmann::json summarize(const std::string& case_name,
                         const hacdcpf::HybridPowerSystem& system,
                         int warmups,
                         int repeats) {
  Samples full;
  Samples schur;
  full.wall_ms.reserve(static_cast<size_t>(repeats));
  full.linear_ms.reserve(static_cast<size_t>(repeats));
  schur.wall_ms.reserve(static_cast<size_t>(repeats));
  schur.linear_ms.reserve(static_cast<size_t>(repeats));
  Samples discarded;
  for (int warmup = 0; warmup < warmups; ++warmup) {
    if (warmup % 2 == 0) {
      run_sample(system, false, discarded);
      run_sample(system, true, discarded);
    } else {
      run_sample(system, true, discarded);
      run_sample(system, false, discarded);
    }
  }
  for (int repeat = 0; repeat < repeats; ++repeat) {
    if (repeat % 2 == 0) {
      run_sample(system, false, full);
      run_sample(system, true, schur);
    } else {
      run_sample(system, true, schur);
      run_sample(system, false, full);
    }
  }
  const double full_wall = median(full.wall_ms);
  const double schur_wall = median(schur.wall_ms);
  const double full_linear = median(full.linear_ms);
  const double schur_linear = median(schur.linear_ms);
  const auto& p = schur.last.profiling;
  return nlohmann::json{
      {"case", case_name},
      {"ac_buses", system.ac.buses.size()},
      {"dc_buses", system.dc.buses.size()},
      {"vsc_blocks", system.vsc_converters.size()},
      {"warmups", warmups},
      {"repeats", repeats},
      {"schur_admission", "forced-research-ablation"},
      {"full_wall_ms_median", full_wall},
      {"schur_wall_ms_median", schur_wall},
      {"wall_change_pct",
       full_wall > 0.0 ? 100.0 * (schur_wall / full_wall - 1.0) : 0.0},
      {"full_linear_ms_median", full_linear},
      {"schur_linear_ms_median", schur_linear},
      {"linear_change_pct",
       full_linear > 0.0 ? 100.0 * (schur_linear / full_linear - 1.0) : 0.0},
      {"full_dimension", p.vsc_schur_full_dimension},
      {"reduced_dimension", p.vsc_schur_reduced_dimension},
      {"full_structural_nnz", p.vsc_schur_full_structural_nnz},
      {"reduced_structural_nnz", p.vsc_schur_reduced_structural_nnz},
      {"reduced_factor_nonzeros", p.vsc_schur_reduced_factor_nonzeros},
      {"reduced_factor_work", p.vsc_schur_reduced_factor_work},
      {"full_lu_factor_nonzeros", full.last.profiling.full_lu_factor_nonzeros},
      {"full_lu_factor_work", full.last.profiling.full_lu_factor_work},
      {"factor_statistics_available",
       p.vsc_schur_reduced_factor_nonzeros >= 0 &&
           full.last.profiling.full_lu_factor_nonzeros >= 0},
      {"schur_attempts", p.vsc_schur_attempts},
      {"schur_accepted", p.vsc_schur_accepted},
      {"schur_fallbacks", p.vsc_schur_fallbacks},
      {"local_regular_rejections", p.vsc_schur_local_regular_rejections},
      {"reduced_solve_rejections", p.vsc_schur_reduced_solve_rejections},
      {"full_backward_error_rejections",
       p.vsc_schur_full_backward_error_rejections},
      {"minimum_observed_local_rcond", p.vsc_schur_minimum_local_rcond},
      {"minimum_accepted_local_rcond",
       p.vsc_schur_minimum_accepted_local_rcond},
      {"max_accepted_full_backward_error",
       p.max_vsc_schur_full_backward_error},
      {"semismooth_rate_samples", p.semismooth_rate_samples},
      {"semismooth_last_residual_ratio", p.semismooth_last_residual_ratio},
      {"semismooth_last_quadratic_ratio", p.semismooth_last_quadratic_ratio},
      {"final_residual", schur.last.residual},
      {"schur_iterations", schur.last.iterations},
      {"full_iterations", full.last.iterations},
      {"full_final_residual", full.last.residual}};
}

int parse_positive(const char* text, const char* option) {
  const int value = std::atoi(text);
  if (value <= 0) {
    throw std::invalid_argument(std::string(option) + " must be positive");
  }
  return value;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string selected_case = "all";
    int warmups = 2;
    int repeats = 5;
    for (int arg = 1; arg < argc; ++arg) {
      const std::string option = argv[arg];
      if (option == "--case" && arg + 1 < argc) {
        selected_case = argv[++arg];
      } else if (option == "--warmups" && arg + 1 < argc) {
        warmups = parse_positive(argv[++arg], "--warmups");
      } else if (option == "--repeats" && arg + 1 < argc) {
        repeats = parse_positive(argv[++arg], "--repeats");
      } else {
        throw std::invalid_argument("unknown or incomplete option: " + option);
      }
    }

    nlohmann::json output;
    output["schema"] = "vsc_local_schur_benchmark_v1";
    output["results"] = nlohmann::json::array();
    if (selected_case == "all" || selected_case == "case300") {
      const auto system = hacdcpf::io::build_case300_acdc_vsc_limit_ncp();
      output["results"].push_back(
          summarize("case300_acdc_vsc_limit_ncp", system, warmups, repeats));
    }
    if (selected_case == "all" || selected_case == "case2000") {
      const auto system = hacdcpf::io::build_case2000_acdc_vsc_limit_ncp();
      output["results"].push_back(
          summarize("case2000_acdc_vsc_limit_ncp", system, warmups, repeats));
    }
    if (output["results"].empty()) {
      throw std::invalid_argument("--case must be case300, case2000, or all");
    }
    std::cout << std::setprecision(12) << output.dump(2) << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "vsc_schur_benchmark: " << error.what() << '\n';
    return 1;
  }
}
