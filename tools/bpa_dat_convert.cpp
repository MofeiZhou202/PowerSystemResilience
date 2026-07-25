/// tools/bpa_dat_convert.cpp
/// ==========================
/// Command-line converter: PSD-BPA / DSP card file (.dat) -> hacdcpf JSON.
///
/// Usage:
///   bpa_dat_convert <input.dat> [output.json] [--strict] [--q-ratio R]
///                   [--lcc] [--vsc-approx]
///
/// When output.json is omitted, the input path with a .json extension is
/// used.  The import report (skipped / coerced records) is printed to stdout.
/// BD/LD links import as native quasi-steady LCCConverter elements by
/// default; --vsc-approx selects the legacy VSC-approximation path
/// (--q-ratio sets its reactive estimate), --lcc is kept for compatibility.

#include <filesystem>
#include <iostream>
#include <string>

#include "hacdcpf/io/bpa_io.hpp"
#include "hacdcpf/io/import_report.hpp"
#include "hacdcpf/io/json_io.hpp"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: bpa_dat_convert <input.dat> [output.json] "
                 "[--strict] [--q-ratio R] [--lcc] [--vsc-approx]\n";
    return 2;
  }

  std::string input = argv[1];
  std::string output;
  hacdcpf::io::BpaImportOptions options;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--strict") {
      options.mode = hacdcpf::io::ImportMode::Strict;
    } else if (arg == "--lcc") {
      options.lcc_model = hacdcpf::io::BpaLccModel::LccQuasiSteady;
    } else if (arg == "--vsc-approx") {
      options.lcc_model = hacdcpf::io::BpaLccModel::VscApprox;
    } else if (arg == "--q-ratio" && i + 1 < argc) {
      options.lcc_q_ratio = std::stod(argv[++i]);
    } else if (output.empty() && arg.rfind("--", 0) != 0) {
      output = arg;
    } else {
      std::cerr << "Unknown / incomplete argument: " << arg << "\n";
      return 2;
    }
  }
  if (output.empty()) {
    output = fs::path(input).replace_extension(".json").string();
  }

  auto result = hacdcpf::io::parse_bpa_dat(input, options);
  const auto& sys = result.system;
  const auto& rep = result.report;

  if (rep.has_errors()) {
    std::cerr << "Import failed for " << input << ":\n";
    for (const auto& rec : rep.records) {
      if (rec.severity == hacdcpf::io::ImportSeverity::Error) {
        std::cerr << "  [error] " << rec.source_locator << ": " << rec.message
                  << "\n";
      }
    }
    return 1;
  }

  hacdcpf::io::save_json(sys, output, 2);

  std::cout << "Converted: " << input << " -> " << output << "\n"
            << "  case: " << sys.name << "  (base " << sys.base_mva << " MVA)\n"
            << "  AC:   " << sys.ac.buses.size() << " buses, "
            << sys.ac.branches.size() << " branches, "
            << sys.ac.generators.size() << " generators, "
            << sys.ac.loads.size() << " loads, " << sys.ac.shunts.size()
            << " shunts\n"
            << "  DC:   " << sys.dc.buses.size() << " buses, "
            << sys.dc.branches.size() << " branches, "
            << sys.vsc_converters.size() << " VSC + "
            << sys.lcc_converters.size() << " LCC converters\n"
            << "  report: " << rep.summary.accepted << " accepted, "
            << rep.summary.coerced << " coerced, " << rep.summary.skipped
            << " skipped, " << rep.summary.rejected << " rejected\n";
  for (const auto& rec : rep.records) {
    if (rec.severity != hacdcpf::io::ImportSeverity::Info) {
      std::cout << "  [warn] " << rec.source_locator << ": " << rec.message
                << "\n";
    }
  }
  return 0;
}
