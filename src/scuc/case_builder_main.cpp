/// case_builder_main.cpp — CLI tool to generate SCUC JSON test cases
///
/// Usage:
///   scuc_case_builder [--case <name>] [--T <periods>] [--dt <hr>]
///                     [--wind] [--solar] [--storage] [--output <file>]
///
/// Available cases: 3bus, 6bus, ieee39

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mipsolvers/scuc/case_builder.hpp"

static void usage(const char* prog) {
  std::cerr
      << "Usage: " << prog << " [options]\n"
      << "  --case <name>    Case name: 3bus | 6bus | ieee39  (default: ieee39)\n"
      << "  --T <periods>    Number of time periods           (default: 24)\n"
      << "  --dt <hours>     Period length in hours           (default: 1.0)\n"
      << "  --wind           Include wind generation\n"
      << "  --solar          Include solar generation\n"
      << "  --storage        Include battery storage (6bus only)\n"
      << "  --output <file>  Output JSON file                 (default: stdout)\n"
      << "  --compact        Compact (non-pretty) JSON output\n"
      << "  --help           Show this message\n";
}

int main(int argc, char** argv) {
  std::string case_name = "ieee39";
  int    T         = 24;
  double dt        = 1.0;
  bool   with_wind    = false;
  bool   with_solar   = false;
  bool   with_storage = false;
  int    indent    = 2;
  std::string output_file;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--help" || arg == "-h") { usage(argv[0]); return 0; }
    else if (arg == "--case"   && i+1 < argc) { case_name  = argv[++i]; }
    else if (arg == "--T"      && i+1 < argc) { T          = std::stoi(argv[++i]); }
    else if (arg == "--dt"     && i+1 < argc) { dt         = std::stod(argv[++i]); }
    else if (arg == "--wind")                 { with_wind    = true; }
    else if (arg == "--solar")                { with_solar   = true; }
    else if (arg == "--storage")              { with_storage = true; }
    else if (arg == "--compact")              { indent       = -1; }
    else if (arg == "--output" && i+1 < argc) { output_file = argv[++i]; }
    else {
      std::cerr << "Unknown argument: " << arg << "\n";
      usage(argv[0]); return 1;
    }
  }

  // Basic validation
  if (T < 1 || T > 8760) {
    std::cerr << "Error: --T must be in [1, 8760]\n"; return 1;
  }
  if (dt <= 0.0 || dt > 24.0) {
    std::cerr << "Error: --dt must be in (0, 24]\n"; return 1;
  }

  // Build the requested case
  mipsolvers::scuc::SCUCInput inp;
  try {
    if (case_name == "3bus") {
      inp = mipsolvers::scuc::build_3bus_case(T, dt);
    } else if (case_name == "6bus") {
      inp = mipsolvers::scuc::build_6bus_case(T, dt, with_wind, with_storage);
    } else if (case_name == "ieee39") {
      inp = mipsolvers::scuc::build_ieee39_case(T, dt, with_wind, with_solar);
    } else {
      std::cerr << "Unknown case: " << case_name
                << "  (choose 3bus | 6bus | ieee39)\n";
      return 1;
    }
  } catch (const std::exception& e) {
    std::cerr << "Case builder error: " << e.what() << "\n";
    return 1;
  }

  const std::string json_str = mipsolvers::scuc::scuc_input_to_json(inp, indent);

  if (!output_file.empty()) {
    std::ofstream ofs(output_file);
    if (!ofs) {
      std::cerr << "Cannot open output file: " << output_file << "\n";
      return 1;
    }
    ofs << json_str << "\n";
    std::cout << "Written to " << output_file
              << "  (" << std::filesystem::file_size(output_file) << " bytes)\n";
  } else {
    std::cout << json_str << "\n";
  }

  return 0;
}
