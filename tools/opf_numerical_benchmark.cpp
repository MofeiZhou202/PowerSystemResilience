// tools/opf_numerical_benchmark.cpp
// =================================================================
// Standalone numerical-performance probe for the AC OPF solver.
//
// Runs solve_ac_opf() on a sweep of MATPOWER cases (small -> very large,
// up to ~25k buses) through the parity full-space IPM (the self-developed
// "native" interior-point method) and prints a table of: convergence flag,
// iteration count, objective, wall time, KKT residual metrics, and the
// dense-vs-sparse KKT linear-algebra path actually taken.
//
// Purpose: characterise scalability of the native IPM to 10000+ buses and
// quantify the sparse-KKT linear-solver cost.
// =================================================================

#include <chrono>
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

using namespace hacdcpf;

namespace {

std::string solver_path_name(opf::OPFSolverPath p) {
  switch (p) {
    case opf::OPFSolverPath::NativeAC:  return "NativeAC";
    case opf::OPFSolverPath::ParityIPM: return "ParityIPM";
    default:                            return "Unknown";
  }
}

struct Row {
  std::string case_name;
  size_t nb{0}, ng{0}, nl{0};
  bool converged{false};
  int iters{0};
  double objective{0.0};
  double time_sec{0.0};
  double max_stat{0.0};
  double final_mu{0.0};
  std::string backend;
  std::string status;
};

Row run_one(const std::string& data_dir, const std::string& file,
            int max_inner, int max_outer) {
  Row row;
  row.case_name = file;

  HybridPowerSystem sys;
  try {
    sys = io::parse_matpower(data_dir + "/" + file);
  } catch (const std::exception& e) {
    row.status = std::string("parse-error: ") + e.what();
    return row;
  }
  row.nb = sys.ac.buses.size();
  row.ng = sys.ac.generators.size();
  row.nl = sys.ac.branches.size();

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.enable_primal_dual = true;
  opt.use_parity_ipm = true;
  opt.allow_fallback = false;
  opt.max_inner_iterations = max_inner;
  opt.max_outer_iterations = max_outer;
  opt.feasibility_tol = 1e-6;
  opt.stationarity_tol = 1e-6;
  opt.verbose = false;

  const auto t0 = std::chrono::high_resolution_clock::now();
  opf::ACOPFResult r;
  try {
    r = opf::solve_ac_opf(sys, opt);
  } catch (const std::exception& e) {
    row.status = std::string("throw: ") + e.what();
    return row;
  }
  const auto t1 = std::chrono::high_resolution_clock::now();
  row.time_sec = std::chrono::duration<double>(t1 - t0).count();
  row.converged = r.converged;
  row.iters = r.iterations;
  row.objective = r.objective;
  row.max_stat = r.max_stationarity;
  row.final_mu = r.profiling.final_barrier_mu;
  row.backend = r.profiling.linear_solver_backend;
  row.status = r.status + " [" + solver_path_name(r.solver_path) + "]";
  return row;
}

void print_header() {
  std::printf("%-22s %6s %5s %6s | %5s %5s %14s %10s %9s | %-26s %s\n",
              "case", "nb", "ng", "nl", "conv", "it",
              "objective", "max_stat", "time(s)", "kkt-backend", "status");
  std::printf("%s\n", std::string(160, '-').c_str());
}

void print_row(const Row& r) {
  std::printf("%-22s %6zu %5zu %6zu | %5s %5d %14.2f %10.2e %9.2f | %-26s %s\n",
              r.case_name.c_str(), r.nb, r.ng, r.nl,
              r.converged ? "YES" : "no", r.iters, r.objective,
              r.max_stat, r.time_sec, r.backend.c_str(), r.status.c_str());
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);  // unbuffered: show progress before any crash
  std::string data_dir = "../data";
  if (argc > 1) data_dir = argv[1];

  // Optional: pass case names as extra args to override the default sweep.
  std::vector<std::string> cases;
  for (int i = 2; i < argc; ++i) cases.emplace_back(argv[i]);
  if (cases.empty()) {
    cases = {
        "case300.m", "case1354pegase.m", "case_ACTIVSg2000.m",
        "case2869pegase.m", "case6515rte.m", "case8387pegase.m",
        "case9241pegase.m", "case_ACTIVSg10k.m",
    };
  }

  std::printf("OPF numerical benchmark (native parity IPM) — data_dir=%s\n\n", data_dir.c_str());
  print_header();
  for (const auto& c : cases) {
    print_row(run_one(data_dir, c, 600, 1));
  }
  return 0;
}
