#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "hacdcpf/io/gridlabd_validation.hpp"

namespace {

void print_usage(const char* argv0) {
  std::cerr
      << "Usage: " << argv0 << " [options]\n"
      << "  --component-only       Run only internal component cases\n"
      << "  --no-transmission      Exclude IEEE/MATPOWER transmission diagnostics\n"
      << "  --large                Include large cases such as case300\n"
      << "  --require-gridlabd     Fail the exact gate if GridLAB-D is unavailable\n"
      << "  --gridlabd-bin PATH    Use a specific GridLAB-D executable\n"
      << "  --out PATH             Write JSON report to PATH\n"
      << "  --long-duration        Run 10s multi-event transient/GridLAB-D sample validation\n"
      << "  --t-end VALUE          Long-duration end time in seconds (default: 10)\n"
      << "  --sample-interval VALUE  Long-duration sample interval in seconds (default: 1)\n"
      << "  --fault-r-pu VALUE     Fault shunt resistance in pu (default: 50)\n"
      << "  --max-branch N         Max branch-trip scenarios per case\n"
      << "  --help                 Show this help\n";
}

double parse_double(const std::string& text, const std::string& flag) {
  try {
    return std::stod(text);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid numeric value for " + flag + ": " + text);
  }
}

int parse_int(const std::string& text, const std::string& flag) {
  try {
    return std::stoi(text);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid integer value for " + flag + ": " + text);
  }
}

}  // namespace

int main(int argc, char** argv) {
  hacdcpf::io::GridLABDValidationMatrixOptions options;
  hacdcpf::io::GridLABDLongDurationOptions long_options;
  std::filesystem::path out_path;
  bool run_long_duration = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string& flag) -> std::string {
      if (i + 1 >= argc) throw std::runtime_error(flag + " requires a value");
      return argv[++i];
    };

    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
    if (arg == "--component-only") {
      options.include_component_cases = true;
      options.include_distribution_cases = false;
      options.include_transmission_cases = false;
      continue;
    }
    if (arg == "--no-transmission") {
      options.include_transmission_cases = false;
      continue;
    }
    if (arg == "--large") {
      options.include_large_cases = true;
      continue;
    }
    if (arg == "--require-gridlabd") {
      options.require_gridlabd = true;
      continue;
    }
    if (arg == "--gridlabd-bin") {
      const auto path = std::filesystem::path(require_value(arg));
      options.comparison_options.run_options.executable = path;
      long_options.comparison_options.run_options.executable = path;
      continue;
    }
    if (arg == "--out") {
      out_path = require_value(arg);
      continue;
    }
    if (arg == "--fault-r-pu") {
      options.fault_r_pu = parse_double(require_value(arg), arg);
      continue;
    }
    if (arg == "--long-duration") {
      run_long_duration = true;
      continue;
    }
    if (arg == "--t-end") {
      long_options.t_end_s = parse_double(require_value(arg), arg);
      continue;
    }
    if (arg == "--sample-interval") {
      long_options.sample_interval_s = parse_double(require_value(arg), arg);
      continue;
    }
    if (arg == "--max-branch") {
      options.max_branch_trip_scenarios_per_case =
          parse_int(require_value(arg), arg);
      continue;
    }

    std::cerr << "Unknown argument: " << arg << "\n";
    print_usage(argv[0]);
    return 2;
  }

  try {
    if (run_long_duration) {
      long_options.require_gridlabd = options.require_gridlabd;
      long_options.comparison_options.export_options =
          options.comparison_options.export_options;
      long_options.comparison_options.vm_tolerance_pu =
          options.comparison_options.vm_tolerance_pu;
      long_options.comparison_options.va_tolerance_deg =
          options.comparison_options.va_tolerance_deg;
      long_options.comparison_options.branch_p_tolerance_mw =
          options.comparison_options.branch_p_tolerance_mw;
      long_options.comparison_options.branch_q_tolerance_mvar =
          options.comparison_options.branch_q_tolerance_mvar;
      long_options.dynamic_options.dt_s = 0.02;
      long_options.dynamic_options.use_adaptive_step = true;
      long_options.dynamic_options.abs_tol = 1e-7;
      long_options.dynamic_options.rel_tol = 1e-5;

      hacdcpf::io::GridLABDValidationMatrixOptions case_options = options;
      case_options.include_component_cases = true;
      case_options.include_distribution_cases = false;
      case_options.include_transmission_cases = false;
      const auto cases = hacdcpf::io::build_gridlabd_standard_cases(case_options);
      const auto it = std::find_if(
          cases.begin(), cases.end(),
          [](const hacdcpf::io::GridLABDStandardCase& c) {
            return c.name == "three_bus_meshed";
          });
      if (it == cases.end()) {
        throw std::runtime_error("three_bus_meshed validation case was not built");
      }
      const auto events =
          hacdcpf::io::build_gridlabd_long_duration_sequence(it->system,
                                                             long_options);
      const auto report =
          hacdcpf::io::run_gridlabd_long_duration_validation(*it, events,
                                                             long_options);
      const std::string json =
          hacdcpf::io::gridlabd_long_duration_report_to_json(report, 2);
      if (!out_path.empty()) {
        std::ofstream out(out_path);
        if (!out) throw std::runtime_error("Failed to open output path: " + out_path.string());
        out << json << "\n";
      } else {
        std::cout << json << "\n";
      }
      std::cerr << "GridLAB-D long-duration validation: exact gate "
                << report.exact_gate_passed_count << "/"
                << report.exact_gate_count
                << (report.exact_gate_passed ? " passed" : " failed")
                << ", dynamic=" << (report.dynamic_success ? "passed" : "failed")
                << ", diagnostics=" << report.diagnostic_count
                << ", skipped=" << report.skipped_count << "\n";
      return report.exact_gate_passed ? 0 : 1;
    }

    const auto report =
        hacdcpf::io::run_gridlabd_standard_validation_matrix(options);
    const std::string json =
        hacdcpf::io::gridlabd_validation_matrix_to_json(report, 2);
    if (!out_path.empty()) {
      std::ofstream out(out_path);
      if (!out) throw std::runtime_error("Failed to open output path: " + out_path.string());
      out << json << "\n";
    } else {
      std::cout << json << "\n";
    }

    std::cerr << "GridLAB-D validation matrix: exact gate "
              << report.exact_gate_passed_count << "/"
              << report.exact_gate_count
              << (report.exact_gate_passed ? " passed" : " failed")
              << ", diagnostics=" << report.diagnostic_count
              << ", skipped=" << report.skipped_count << "\n";
    return report.exact_gate_passed ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "gridlabd_validation_matrix failed: " << e.what() << "\n";
    return 2;
  }
}
