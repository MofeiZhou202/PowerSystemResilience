/// scuc_solve — Command-line SCUC/SCED/LMP solver
///
/// Usage:
///   scuc_solve <input.json> [output.json]  [--solver <name>] [--no-sced] [--no-lmp]
///
/// When output.json is omitted the result JSON is written to stdout.
/// The input file must conform to the schema described in include/mipsolvers/scuc/scuc.hpp.

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>

#include "mipsolvers/scuc/scuc.hpp"
#include "mipsolvers/engine/engine.hpp"

static void print_usage(const char* progname) {
  std::cerr
    << "Usage: " << progname
    << " <input.json> [output.json]"
    << " [--solver <Auto|StrictHiGHS|Gurobi|HiGHS|SCIP|NativeBranchAndCut>]"
    << " [--no-sced] [--no-lmp]"
    << " [--indent <n>]\n"
    << "\n"
    << "Solver choices (case-sensitive):\n"
    << "  Auto                — use the production MILP policy\n"
    << "  StrictHiGHS         — embedded HiGHS state machine with MIPSolvers contract\n"
    << "  HiGHS               — direct open-source LP/MILP (HiGHS)\n"
    << "  SCIP                — open-source MILP/MINLP (SCIP)\n"
    << "  NativeBranchAndCut  — built-in B&C in MIPSolvers\n"
    << "  Gurobi              — commercial MILP (requires licence)\n"
    << "\n"
    << "The input JSON must contain at minimum:\n"
    << "  config        — solver settings (optional; defaults shown below)\n"
    << "  generators    — list of dispatchable units\n"
    << "  branches      — AC transmission branches\n"
    << "  loads         — system loads\n"
    << "  profiles      — load/wind/solar time-series\n"
    << "  initial_status — generator and storage initial conditions\n"
    << "\n"
    << "Available solvers in this build:\n";
  mipsolvers::engine::SolverEngine eng;
  eng.register_default_adapters();
  for (const auto& s : eng.list_solvers(mipsolvers::engine::ProblemClass::MILP))
    std::cerr << "  " << s << "\n";
}

static std::string read_file(const std::string& path) {
  std::ifstream ifs(path);
  if (!ifs) throw std::runtime_error("Cannot open file: " + path);
  return {std::istreambuf_iterator<char>(ifs), {}};
}

int main(int argc, char** argv) {
  if (argc < 2) { print_usage(argv[0]); return EXIT_FAILURE; }

  std::string input_path;
  std::string output_path;
  std::string solver_override;
  bool no_sced = false;
  bool no_lmp  = false;
  int indent = 2;

  for (int i = 1; i < argc; ++i) {
    std::string_view arg{argv[i]};
    if (arg == "--solver" && i + 1 < argc) {
      solver_override = argv[++i];
    } else if (arg == "--no-sced") {
      no_sced = true;
    } else if (arg == "--no-lmp") {
      no_lmp = true;
    } else if (arg == "--indent" && i + 1 < argc) {
      try { indent = std::stoi(argv[++i]); }
      catch (...) { std::cerr << "Warning: invalid --indent value, using 2\n"; indent = 2; }
    } else if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]); return EXIT_SUCCESS;
    } else if (arg.starts_with("--")) {
      std::cerr << "Unknown option: " << arg << "\n";
      print_usage(argv[0]); return EXIT_FAILURE;
    } else if (input_path.empty()) {
      input_path = argv[i];
    } else if (output_path.empty()) {
      output_path = argv[i];
    } else {
      std::cerr << "Unexpected argument: " << arg << "\n";
      print_usage(argv[0]); return EXIT_FAILURE;
    }
  }

  if (input_path.empty()) { print_usage(argv[0]); return EXIT_FAILURE; }

  // ── Load input ───────────────────────────────────────────────────────────
  mipsolvers::scuc::SCUCInput input;
  try {
    const std::string json_str = read_file(input_path);
    input = mipsolvers::scuc::scuc_from_json(json_str);
  } catch (const std::exception& e) {
    std::cerr << "Error reading input: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  // Apply command-line overrides
  if (!solver_override.empty()) input.config.solver = solver_override;
  if (no_sced) input.config.solve_sced = false;
  if (no_lmp)  input.config.solve_lmp  = false;

  // Print configuration summary to stderr
  std::cerr << "SCUC Solve\n"
            << "  Input:   " << input_path << "\n"
            << "  Solver:  " << input.config.solver << "\n"
            << "  Periods: " << input.config.num_periods << "\n"
            << "  Buses:   " << input.num_buses << "\n"
            << "  Gens:    " << input.generators.size() << "\n"
            << "  Branches:" << input.branches.size() << "\n"
            << "  SCED:    " << (input.config.solve_sced ? "yes" : "no") << "\n"
            << "  LMP:     " << (input.config.solve_lmp  ? "yes" : "no") << "\n";

  // ── Solve ────────────────────────────────────────────────────────────────
  mipsolvers::scuc::SCUCOutput output;
  try {
    output = mipsolvers::scuc::scuc_solve(input);
  } catch (const std::exception& e) {
    std::cerr << "Error during solve: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  std::cerr << "  SCUC converged: " << (output.scuc.converged ? "YES" : "NO") << "\n"
            << "  Objective: "      << output.scuc.objective  << "\n"
            << "  Solver used: "    << output.scuc.solver_name << "\n"
            << "  MIP gap: "        << output.scuc.mip_gap     << "\n"
            << "  Solve time: "     << output.scuc.solve_time_sec << " s\n";
  if (input.config.solve_sced)
    std::cerr << "  SCED converged: " << (output.sced.converged ? "YES" : "NO") << "\n";
  if (input.config.solve_lmp)
    std::cerr << "  LMP converged: "  << (output.lmp.converged  ? "YES" : "NO") << "\n"
              << "  Avg LMP: "         << output.lmp.avg_lmp << " $/MWh\n";

  // ── Write output ─────────────────────────────────────────────────────────
  std::string json_out;
  try {
    json_out = mipsolvers::scuc::scuc_output_to_json(output, input, indent);
  } catch (const std::exception& e) {
    std::cerr << "Error serialising output: " << e.what() << "\n";
    return EXIT_FAILURE;
  }

  if (output_path.empty()) {
    std::cout << json_out << "\n";
  } else {
    std::ofstream ofs(output_path);
    if (!ofs) { std::cerr << "Cannot write to: " << output_path << "\n"; return EXIT_FAILURE; }
    ofs << json_out << "\n";
    std::cerr << "  Output:  " << output_path << "\n";
  }

  return output.scuc.converged ? EXIT_SUCCESS : EXIT_FAILURE;
}
