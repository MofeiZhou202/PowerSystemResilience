/// tools/sppt_agent_demo.cpp
/// =========================
/// Verified Intelligent Modeling demonstration (Alg. 2 / Thm. 8.9 of
/// docs/latex/sppt_theory.tex).  Runs a scripted sequence of model edits (a
/// stand-in for an LLM/agent) through the SPPT admissibility guard on a seed
/// case, printing each Accept/Reject verdict with attribution and the final
/// LLM-in-the-loop metrics.
///
/// Usage:  sppt_agent_demo [case.m|case.json]   (default: data/case9.m)

#include <filesystem>
#include <iostream>
#include <string>

#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/sppt/agent.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;

int main(int argc, char** argv) {
  const fs::path root = HACDCPF_PROJECT_ROOT;
  const std::string path =
      (argc > 1) ? argv[1] : (root / "data" / "case9.m").string();

  HybridPowerSystem sys;
  try {
    if (path.size() >= 2 && path.substr(path.size() - 2) == ".m")
      sys = io::parse_matpower(path);
    else
      sys = io::load_json(path);
  } catch (const std::exception& e) {
    std::cerr << "sppt_agent_demo: failed to load " << path << ": " << e.what()
              << "\n";
    return 2;
  }

  const sppt::AgentTrajectory traj =
      sppt::run_agent_loop(sys, sppt::default_agent_script());

  std::cout << "SPPT guarded agent loop on " << path << "\n"
            << "------------------------------------------------------------\n";
  int i = 1;
  for (const auto& s : traj.steps) {
    std::cout << "[" << i++ << "] " << s.edit_name << "\n"
              << "      verdict : " << (s.accepted ? "ACCEPT" : "REJECT")
              << (s.accepted ? "" : " (" + s.reason + ")") << "\n"
              << "      applied : " << (s.applied ? "yes" : "no");
    if (s.analysis_ran)
      std::cout << " | analysis " << (s.analysis_converged ? "converged" : "diverged")
                << " | V attributed to " << s.attributed_buses << " buses";
    std::cout << "\n";
  }

  const auto& m = traj.metrics;
  std::cout << "------------------------------------------------------------\n"
            << "Guard metrics: tp=" << m.tp << " tn=" << m.tn << " fp=" << m.fp
            << " fn=" << m.fn << "\n"
            << "  precision=" << m.precision() << " recall=" << m.recall()
            << " catch_rate=" << m.catch_rate() << " accuracy=" << m.accuracy()
            << "\n"
            << "Trajectory soundness (Thm. 8.9): "
            << (traj.sound ? "SOUND" : "VIOLATED") << "\n";
  return traj.sound ? 0 : 1;
}
