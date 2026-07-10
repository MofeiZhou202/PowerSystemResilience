/// tools/sppt_benchmark.cpp
/// ========================
/// Self-contained Pillar-4 scale/accuracy benchmark of the SPPT-enabled platform
/// (docs/latex/sppt_theory.tex). Times projection, canonical assembly, and the
/// power-flow solve across a case ladder, and reports the MR3 commuting residual.
/// Writes sppt_benchmark.{tex,csv} and prints a summary.  No external tools.
///
/// Usage:  sppt_benchmark [output_dir] [repeats]

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/sppt/benchmark.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;

int main(int argc, char** argv) {
  const fs::path root = HACDCPF_PROJECT_ROOT;
  const fs::path data = root / "data";
  const fs::path outdir = (argc > 1) ? fs::path(argv[1]) : (root / "docs" / "latex");
  const int repeats = (argc > 2) ? std::stoi(argv[2]) : 3;

  // Scale ladder from small IEEE cases up to European PEGASE systems.
  const std::vector<std::string> ladder = {
      "case9.m",   "case14.m",  "case30.m",         "case57.m",
      "case118.m", "case300.m", "case1354pegase.m", "case2869pegase.m"};

  std::vector<std::pair<std::string, std::string>> cases;
  for (const auto& n : ladder) {
    const fs::path p = data / n;
    if (fs::exists(p)) cases.emplace_back(n, p.string());
  }
  if (cases.empty()) {
    std::cerr << "sppt_benchmark: no cases found under " << data << "\n";
    return 2;
  }

  const sppt::Benchmark bench = sppt::run_benchmark(cases, repeats);

  std::error_code ec;
  fs::create_directories(outdir, ec);
  { std::ofstream(outdir / "sppt_benchmark.csv") << bench.to_csv(); }
  { std::ofstream(outdir / "sppt_benchmark.tex") << bench.to_latex(); }

  std::cout << bench.to_csv() << "\n";
  int converged = 0;
  double worst_resid = 0.0;
  for (const auto& r : bench.rows) {
    if (r.converged) ++converged;
    if (r.commuting_residual > worst_resid) worst_resid = r.commuting_residual;
  }
  std::cout << "Benchmark: " << converged << "/" << bench.rows.size()
            << " converged; worst MR3 residual = " << worst_resid
            << "\nWrote " << (outdir / "sppt_benchmark.tex").string() << " and .csv\n";
  return 0;
}
