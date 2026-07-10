/// tools/sppt_agent_demo.cpp
/// =========================
/// Verified Intelligent Modeling demonstration (Alg. 2 / Thm. 8.9 of
/// docs/latex/sppt_theory.tex).  Runs a scripted sequence of model edits (a
/// stand-in for a future LLM/tool-call proposal) through the SPPT admissibility
/// guard on a seed case, printing each Accept/Reject verdict with attribution and the final
/// scripted agent-edit metrics for the LLM-ready interface.
///
/// Usage:
///   sppt_agent_demo [case.m|case.json]
///   sppt_agent_demo --actions id1,id2 [case.m|case.json]
///   sppt_agent_demo --list-actions

#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/sppt/agent.hpp"

#ifndef HACDCPF_PROJECT_ROOT
#define HACDCPF_PROJECT_ROOT "."
#endif

namespace fs = std::filesystem;
using namespace hacdcpf;

namespace {

std::vector<std::string> split_csv(const std::string& csv) {
  std::vector<std::string> out;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (!item.empty()) out.push_back(item);
  }
  return out;
}

void print_usage(const char* argv0) {
  std::cerr << "Usage:\n"
            << "  " << argv0 << " [case.m|case.json]\n"
            << "  " << argv0 << " --actions id1,id2 [case.m|case.json]\n"
            << "  " << argv0 << " --list-actions\n";
}

}  // namespace

int main(int argc, char** argv) {
  const fs::path root = HACDCPF_PROJECT_ROOT;
  std::string path = (root / "data" / "case9.m").string();
  std::vector<std::string> action_ids;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
    if (arg == "--list-actions") {
      for (const auto& id : sppt::agent_action_ids()) std::cout << id << "\n";
      return 0;
    }
    if (arg == "--actions") {
      if (i + 1 >= argc) {
        std::cerr << "sppt_agent_demo: --actions requires a comma-separated list\n";
        print_usage(argv[0]);
        return 2;
      }
      action_ids = split_csv(argv[++i]);
      continue;
    }
    path = arg;
  }

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

  std::vector<sppt::AgentEdit> script;
  try {
    script = action_ids.empty() ? sppt::default_agent_script()
                                : sppt::agent_script_from_action_ids(action_ids);
  } catch (const std::exception& e) {
    std::cerr << "sppt_agent_demo: " << e.what() << "\n";
    return 2;
  }

  const sppt::AgentTrajectory traj = sppt::run_agent_loop(sys, script);

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
