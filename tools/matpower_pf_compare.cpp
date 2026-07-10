// tools/matpower_pf_compare.cpp
// -----------------------------
// Minimal MATPOWER AC PF runner used by external cross-tool checks.
// Prints a compact JSON object with convergence flags and solved voltages.

#include <cmath>
#include <exception>
#include <iostream>
#include <string>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/matpower_parser.hpp"

static std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (char c : s) {
    if (c == '\\' || c == '"') {
      out.push_back('\\');
      out.push_back(c);
    } else if (c == '\n') {
      out += "\\n";
    } else {
      out.push_back(c);
    }
  }
  return out;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: matpower_pf_compare <case.m>\n";
    return 2;
  }

  try {
    const std::string case_path = argv[1];
    const auto sys = hacdcpf::io::parse_matpower(case_path);

    hacdcpf::PowerFlowOptions opt;
    opt.tol = 1e-8;
    opt.max_iter = 80;

    const auto pf = hacdcpf::solve_power_flow(sys, opt);

    std::cout << "{\"converged\":" << (pf.converged ? "true" : "false")
              << ",\"iterations\":" << pf.iterations
              << ",\"residual\":" << pf.residual
              << ",\"vm\":[";
    for (size_t i = 0; i < pf.vm.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << pf.vm[i];
    }
    std::cout << "],\"va\":[";
    for (size_t i = 0; i < pf.va.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << pf.va[i];
    }
    std::cout << "]}";

    return pf.converged ? 0 : 1;
  } catch (const std::exception& e) {
    std::cout << "{\"converged\":false,\"error\":\"" << json_escape(e.what())
              << "\"}";
    return 1;
  }
}
