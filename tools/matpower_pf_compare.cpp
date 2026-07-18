// tools/matpower_pf_compare.cpp
// -----------------------------
// Minimal MATPOWER AC PF runner used by external cross-tool checks.
// Prints a compact JSON object with convergence flags and solved voltages.
//
// Flags:
//   --no-pv-pq    disable PV->PQ switching (match MATPOWER default runpf)
//   --flows       append per-branch from/to flows and total losses
//   --repeat N    solve N extra times (warm cache) and report per-solve ms

#include <chrono>
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
    std::cerr << "usage: matpower_pf_compare <case.m> [--no-pv-pq] [--flows] "
                 "[--repeat N]\n";
    return 2;
  }
  bool no_pv_pq = false;
  bool flows = false;
  int repeat = 0;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--no-pv-pq") {
      no_pv_pq = true;
    } else if (a == "--flows") {
      flows = true;
    } else if (a == "--repeat" && i + 1 < argc) {
      repeat = std::max(0, std::atoi(argv[++i]));
    }
  }

  try {
    const std::string case_path = argv[1];
    const auto t_parse0 = std::chrono::steady_clock::now();
    const auto sys = hacdcpf::io::parse_matpower(case_path);
    const auto t_parse1 = std::chrono::steady_clock::now();

    hacdcpf::PowerFlowOptions opt;
    opt.tol = 1e-8;
    opt.max_iter = 80;
    opt.enable_solver_profiling = true;
    if (no_pv_pq) {
      opt.enable_pv_pq_conversion = false;
    }

    const auto ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };

    auto pf = hacdcpf::solve_power_flow(sys, opt);
    const auto t_solve1 = std::chrono::steady_clock::now();

    std::vector<double> repeat_ms;
    repeat_ms.reserve(static_cast<size_t>(repeat));
    for (int k = 0; k < repeat; ++k) {
      const auto t0 = std::chrono::steady_clock::now();
      pf = hacdcpf::solve_power_flow(sys, opt);
      repeat_ms.push_back(ms(t0, std::chrono::steady_clock::now()));
    }
    const auto t_end = std::chrono::steady_clock::now();

    std::cout << "{\"converged\":" << (pf.converged ? "true" : "false")
              << ",\"iterations\":" << pf.iterations
              << ",\"residual\":" << pf.residual
              << ",\"n_bus\":" << sys.ac.buses.size()
              << ",\"n_gen\":" << sys.ac.generators.size()
              << ",\"n_branch\":" << sys.ac.branches.size()
              << ",\"pd_total_mw\":" << [&] {
                   double s = 0.0;
                   for (const auto& b : sys.ac.buses) s += b.pd_mw;
                   return s;
                 }()
              << ",\"qd_total_mvar\":" << [&] {
                   double s = 0.0;
                   for (const auto& b : sys.ac.buses) s += b.qd_mvar;
                   return s;
                 }()
              << ",\"parse_ms\":" << ms(t_parse0, t_parse1)
              << ",\"solve_ms\":" << ms(t_parse1, t_solve1)
              << ",\"total_ms\":" << ms(t_parse1, t_end)
              << ",\"repeat_ms\":[";
    for (size_t i = 0; i < repeat_ms.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << repeat_ms[i];
    }
    std::cout << "],\"linear_backend\":\""
              << json_escape(pf.profiling.linear_solver_backend)
              << "\",\"factorization_calls\":" << pf.profiling.factorization_calls
              << ",\"eval_jacobian_ms_total\":" << pf.profiling.eval_jacobian_ms_total
              << ",\"linear_solve_ms_total\":" << pf.profiling.linear_solve_ms_total
              << ",\"line_search_ms_total\":" << pf.profiling.line_search_ms_total
              << ",\"pv_to_pq_switches\":" << pf.profiling.pv_to_pq_switches
              << ",\"pq_to_pv_switches\":" << pf.profiling.pq_to_pv_switches
              << ",\"bus_ids\":[";
    // parse_matpower renumbers buses to 1..N in .index and stores the
    // original MATPOWER bus number in .name as "Bus<id>"; report the
    // original number so cross-tool comparisons align on MATPOWER ids.
    for (size_t i = 0; i < sys.ac.buses.size(); ++i) {
      if (i) std::cout << ',';
      const auto& b = sys.ac.buses[i];
      int id = b.index;
      if (b.name.size() > 3 && b.name.rfind("Bus", 0) == 0) {
        try {
          id = std::stoi(b.name.substr(3));
        } catch (const std::exception&) {
        }
      }
      std::cout << id;
    }
    std::cout << "],\"vm\":[";
    for (size_t i = 0; i < pf.vm.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << pf.vm[i];
    }
    std::cout << "],\"va\":[";
    for (size_t i = 0; i < pf.va.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << pf.va[i];
    }
    std::cout << ']';
    if (flows) {
      double loss_p = 0.0;
      for (const auto& f : pf.branch_flows) loss_p += f.pf_mw + f.pt_mw;
      std::cout << ",\"loss_p_mw\":" << loss_p << ",\"branches\":[";
      for (size_t i = 0; i < sys.ac.branches.size() &&
                          i < pf.branch_flows.size(); ++i) {
        if (i) std::cout << ',';
        const auto& br = sys.ac.branches[i];
        const auto& f = pf.branch_flows[i];
        std::cout << "{\"from\":" << br.from_bus << ",\"to\":" << br.to_bus
                  << ",\"in_service\":" << (br.in_service ? 1 : 0)
                  << ",\"pf_mw\":" << f.pf_mw << ",\"pt_mw\":" << f.pt_mw
                  << '}';
      }
      std::cout << ']';
    }
    std::cout << "}";

    return pf.converged ? 0 : 1;
  } catch (const std::exception& e) {
    std::cout << "{\"converged\":false,\"error\":\"" << json_escape(e.what())
              << "\"}";
    return 1;
  }
}
