/// tools/sppt_ablation.cpp
/// =======================
/// Emits the Pillar-4 SPPT ablation study (docs/latex/sppt_theory.tex): disabling
/// provenance, role typing, and the merge fill-guard, and quantifying the damage.
/// Writes sppt_ablation.{tex,csv} and prints a console summary.
///
/// Usage:  sppt_ablation [output_dir]   (default: <project_root>/docs/latex)

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "hacdcpf/model/enums/bus_types.hpp"
#include "hacdcpf/model/hybrid_power_system.hpp"
#include "hacdcpf/sppt/ablation.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;

namespace {

// 3-bus feeder with a closed switch (bus 1--2) so the merge guard is exercised.
HybridPowerSystem make_switch_demo() {
  HybridPowerSystem sys;
  sys.name = "switch_demo";
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = 50.0;

  auto bus = [](int idx, BusType t) {
    ACBus b;
    b.index = idx;
    b.bus_type = t;
    b.vm_pu = 1.0;
    b.in_service = true;
    return b;
  };
  sys.ac.buses = {bus(1, BusType::SLACK), bus(2, BusType::PQ), bus(3, BusType::PQ)};

  ACBranch line;
  line.index = 1;
  line.from_bus = 2;
  line.to_bus = 3;
  line.r_pu = 0.01;
  line.x_pu = 0.05;
  line.tap = 1.0;
  line.in_service = true;
  sys.ac.branches = {line};

  Switch sw;
  sw.index = 1;
  sw.bus_from = 1;
  sw.bus_to = 2;
  sw.closed = true;
  sw.in_service = true;
  sys.ac.switches = {sw};

  Generator g;
  g.index = 1;
  g.bus = 1;
  g.is_slack = true;
  g.in_service = true;
  g.vg_pu = 1.0;
  g.pg_mw = 30.0;
  g.pmax_mw = 200.0;
  g.pmin_mw = 0.0;
  g.qmax_mvar = 100.0;
  g.qmin_mvar = -100.0;
  sys.ac.generators = {g};

  Load ld;
  ld.index = 1;
  ld.bus = 3;
  ld.p_mw = 30.0;
  ld.q_mvar = 10.0;
  ld.in_service = true;
  sys.ac.loads = {ld};
  return sys;
}

}  // namespace

int main(int argc, char** argv) {
  const fs::path root = HACDCPF_PROJECT_ROOT;
  const fs::path data = root / "data";
  const fs::path outdir = (argc > 1) ? fs::path(argv[1]) : (root / "docs" / "latex");

  std::vector<std::pair<std::string, std::string>> cases;
  for (const char* n : {"case9.m", "case14.m", "case30.m", "case118.m"}) {
    const fs::path p = data / n;
    if (fs::exists(p)) cases.emplace_back(n, p.string());
  }

  sppt::AblationStudy study = sppt::run_ablation_study(cases);
  // A switch case makes the provenance and merge-guard ablations non-vacuous.
  study.rows.push_back(sppt::run_ablation_case(make_switch_demo(), "switch_demo"));

  std::error_code ec;
  fs::create_directories(outdir, ec);
  { std::ofstream(outdir / "sppt_ablation.csv") << study.to_csv(); }
  { std::ofstream(outdir / "sppt_ablation.tex") << study.to_latex(); }

  std::cout << study.to_csv() << "\n"
            << "Ablation summary:\n"
            << "  remove provenance   -> " << study.total_unattributable_without_provenance()
            << " unattributable canonical entities\n"
            << "  remove role typing  -> guard caught every reference-less edit; "
            << study.silent_solves_without_role_typing()
            << " would have solved 'converged' unguarded\n"
            << "  remove merge guard  -> up to " << study.max_merge_blowup()
            << "x blow-up in max |Y_ii|\n"
            << "Wrote " << (outdir / "sppt_ablation.tex").string() << " and .csv\n";
  return 0;
}
