#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "hacdcpf/io/opendss_bridge.hpp"
#include "opendss_compare/snapshot_contract.hpp"

namespace {

namespace fs = std::filesystem;

fs::path project_root() {
#ifdef HACDCPF_PROJECT_ROOT
  return fs::path(HACDCPF_PROJECT_ROOT);
#else
  return fs::current_path();
#endif
}

struct AdapterOptions {
  fs::path master_dss;
  std::optional<fs::path> out;
};

AdapterOptions parse_args(int argc, char** argv) {
  AdapterOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--master") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--master requires a path argument.");
      }
      options.master_dss = argv[++i];
      continue;
    }
    if (arg == "--out") {
      if (i + 1 >= argc) {
        throw std::runtime_error("--out requires a path argument.");
      }
      options.out = fs::path(argv[++i]);
      continue;
    }
    if (arg == "--help" || arg == "-h") {
      std::cout
          << "Usage: opendss_snapshot_adapter --master <Master.dss> [--out <json>]\n"
          << "Loads the OpenDSS Master.dss, solves a snapshot, and emits canonical JSON.\n";
      std::exit(0);
    }
    throw std::runtime_error("Unknown argument: " + arg);
  }

  if (options.master_dss.empty()) {
    throw std::runtime_error("Missing required --master <Master.dss> argument.");
  }
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = parse_args(argc, argv);
    const auto snapshot = hacdcpf::io::solve_opendss_snapshot(options.master_dss);
    const auto output =
        hacdcpf_compare::snapshot_contract::snapshot_to_json(
            snapshot,
            hacdcpf_compare::snapshot_contract::portable_path_string(
                options.master_dss, project_root()));

    if (options.out.has_value()) {
      if (!options.out->parent_path().empty()) {
        fs::create_directories(options.out->parent_path());
      }
      std::ofstream os(*options.out);
      if (!os.good()) {
        throw std::runtime_error("Failed to open output file: " +
                                 options.out->string());
      }
      os << output.dump(2) << '\n';
    } else {
      std::cout << output.dump(2) << '\n';
    }
    return 0;
  } catch (const std::exception& ex) {
    std::cerr << "opendss_snapshot_adapter failed: " << ex.what() << '\n';
    return 1;
  }
}
