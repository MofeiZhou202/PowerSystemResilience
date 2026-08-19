// tools/pf_doc_benchmark.cpp
// --------------------------
// Reproducible numerical benchmarks for the power-flow technical manual
// (docs/modules/power_flow/chapters/theory_*.tex). Emits honest, per-case
// convergence data used verbatim in the "数值算例与基准" sections. All numbers
// are produced by the platform's public solver API (hacdcpf.hpp) so the manual
// tables can be regenerated at any time.
//
// Usage:
//   pf_doc_benchmark [mode] [case1.m case2.m ...]
//
// Modes:
//   nr   (default)  Newton-Raphson convergence: iterations, final residual,
//                   condition proxy, LU factor non-zeros, timing, and the full
//                   per-iteration residual history (quadratic-convergence demo).
//
// With no case arguments a default IEEE case list under data/ is used.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "hacdcpf/api/hacdcpf.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/power_flow/distribution_power_flow.hpp"
#include "hacdcpf/power_flow/voltage_stability.hpp"
#include "hacdcpf/assembly/solver_data.hpp"

namespace {

std::string basename_of(const std::string& path) {
  const auto slash = path.find_last_of("/\\");
  return slash == std::string::npos ? path : path.substr(slash + 1);
}

struct NrRow {
  std::string name;
  int n_bus{0};
  int n_var{0};
  int iters{0};
  double residual{0.0};
  double cond{0.0};
  int pv_pq_switches{0};
  double solve_ms{0.0};
  std::vector<double> resid_by_iter;
};

NrRow run_nr(const std::string& case_path) {
  NrRow row;
  row.name = basename_of(case_path);
  auto sys = hacdcpf::io::parse_matpower(case_path);
  row.n_bus = static_cast<int>(sys.ac.buses.size());

  hacdcpf::PowerFlowOptions opt;
  opt.tol = 1e-10;
  opt.max_iter = 50;
  opt.enable_solver_profiling = true;
  opt.robust_nonlinear.enable_condition_monitor = true;

  const auto t0 = std::chrono::steady_clock::now();
  const auto pf = hacdcpf::solve_power_flow(sys, opt);
  const auto t1 = std::chrono::steady_clock::now();

  row.iters = pf.iterations;
  row.residual = pf.residual;
  row.cond = pf.profiling.condition_estimate;
  row.n_var = pf.diagnostics.n_variables;
  row.pv_pq_switches = pf.profiling.pv_to_pq_switches;
  row.solve_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
  row.resid_by_iter = pf.profiling.residual_by_iter;
  return row;
}

int mode_nr(const std::vector<std::string>& cases) {
  std::cout << "# Newton-Raphson convergence benchmark (flat start, tol=1e-10)\n";
  std::cout << "# columns: case n_bus n_var iters residual cond_proxy "
               "pv_pq solve_ms\n";
  std::vector<NrRow> rows;
  rows.reserve(cases.size());
  for (const auto& c : cases) {
    try {
      const NrRow r = run_nr(c);
      rows.push_back(r);
      std::cout << std::setprecision(6);
      std::cout << r.name << ' ' << r.n_bus << ' ' << r.n_var << ' '
                << r.iters << ' ' << r.residual << ' ' << r.cond << ' '
                << r.pv_pq_switches << ' ' << r.solve_ms << '\n';
    } catch (const std::exception& e) {
      std::cout << basename_of(c) << " ERROR " << e.what() << '\n';
    }
  }

  std::cout << "\n# residual_by_iter (max mismatch norm at each Newton iteration)\n";
  std::cout << std::scientific << std::setprecision(4);
  for (const auto& r : rows) {
    std::cout << r.name << ':';
    for (const double v : r.resid_by_iter) std::cout << ' ' << v;
    std::cout << '\n';
  }
  return 0;
}

// FDPF vs Newton: iteration counts on the same cases.
int mode_fdpf(const std::vector<std::string>& cases) {
  std::cout << "# FDPF vs Newton iteration counts (flat start, tol=1e-8)\n";
  std::cout << "# columns: case n_bus nr_iters nr_res fdpf_iters fdpf_res\n";
  for (const auto& c : cases) {
    try {
      auto sys = hacdcpf::io::parse_matpower(c);
      hacdcpf::PowerFlowOptions opt;
      opt.tol = 1e-8;
      opt.max_iter = 100;
      const auto nr = hacdcpf::solve_power_flow(sys, opt);
      const auto fd = hacdcpf::solve_power_flow_fdpf(sys, opt);
      std::cout << std::scientific << std::setprecision(3);
      std::cout << basename_of(c) << ' ' << sys.ac.buses.size() << ' '
                << nr.iterations << ' ' << nr.residual << ' ' << fd.iterations
                << ' ' << fd.residual << '\n';
    } catch (const std::exception& e) {
      std::cout << basename_of(c) << " ERROR " << e.what() << '\n';
    }
  }
  return 0;
}

// DC-linearized vs AC: from-end active branch-flow error (angle differences,
// not absolute angles, drive branch flows, so only flow error is robust).
int mode_dc(const std::vector<std::string>& cases) {
  std::cout << "# DC-linearized vs AC from-end active branch flows\n";
  std::cout << "# columns: case n_bus n_branch max_pf_err_MW mean_pf_err_MW "
               "rms_pf_err_MW\n";
  for (const auto& c : cases) {
    try {
      auto sys = hacdcpf::io::parse_matpower(c);
      hacdcpf::PowerFlowOptions opt;
      opt.tol = 1e-10;
      opt.max_iter = 50;
      const auto ac = hacdcpf::solve_power_flow(sys, opt);
      const auto dc = hacdcpf::solve_ac_dc_power_flow(sys, opt);
      const size_t nb = std::min(ac.branch_flows.size(), dc.pf_mw.size());
      double maxe = 0.0, sume = 0.0, sumsq = 0.0;
      for (size_t k = 0; k < nb; ++k) {
        const double e = std::abs(ac.branch_flows[k].pf_mw - dc.pf_mw[k]);
        maxe = std::max(maxe, e);
        sume += e;
        sumsq += e * e;
      }
      const double mean = nb ? sume / static_cast<double>(nb) : 0.0;
      const double rms = nb ? std::sqrt(sumsq / static_cast<double>(nb)) : 0.0;
      std::cout << std::setprecision(4);
      std::cout << basename_of(c) << ' ' << sys.ac.buses.size() << ' ' << nb
                << ' ' << maxe << ' ' << mean << ' ' << rms << '\n';
    } catch (const std::exception& e) {
      std::cout << basename_of(c) << " ERROR " << e.what() << '\n';
    }
  }
  return 0;
}

// BFS distribution PF vs Newton on radial feeders (min-voltage cross-check).
int mode_bfs(const std::vector<std::string>& cases) {
  std::cout << "# BFS distribution PF vs Newton on radial feeders (tol=1e-8)\n";
  std::cout << "# columns: case n_bus bfs_iters bfs_min_vm bfs_res nr_iters "
               "nr_min_vm min_vm_diff\n";
  const auto vmin = [](const std::vector<double>& v) {
    double m = 1e9;
    for (const double x : v)
      if (x > 1e-6) m = std::min(m, x);
    return m;
  };
  for (const auto& c : cases) {
    try {
      auto sys = hacdcpf::io::parse_matpower(c);
      hacdcpf::analysis::DPFOptions dopt;
      dopt.tol = 1e-8;
      dopt.max_iter = 300;
      const auto bfs = hacdcpf::analysis::solve_distribution_pf(sys, dopt);
      hacdcpf::PowerFlowOptions opt;
      opt.tol = 1e-8;
      opt.max_iter = 100;
      const auto nr = hacdcpf::solve_power_flow(sys, opt);
      const double bmin = vmin(bfs.vm_pu);
      const double nmin = vmin(nr.vm);
      std::cout << std::setprecision(6);
      std::cout << basename_of(c) << ' ' << sys.ac.buses.size() << ' '
                << bfs.iterations << ' ' << bmin << ' ' << bfs.residual << ' '
                << nr.iterations << ' ' << nmin << ' ' << std::abs(bmin - nmin)
                << '\n';
    } catch (const std::exception& e) {
      std::cout << basename_of(c) << " ERROR " << e.what() << '\n';
    }
  }
  return 0;
}

// Advanced solvers on standard cases: plain Newton, homotopy, HELM, Newton-Krylov.
int mode_robust(const std::vector<std::string>& cases) {
  std::cout << "# Advanced solvers on standard cases (flat start, tol=1e-8)\n";
  std::cout << "# columns: case n_bus nr_conv nr_res hom_conv hom_res "
               "helm_conv helm_res nk_conv nk_res\n";
  const auto yn = [](bool b) { return b ? "Y" : "N"; };
  for (const auto& c : cases) {
    try {
      auto sys = hacdcpf::io::parse_matpower(c);
      hacdcpf::PowerFlowOptions opt;
      opt.tol = 1e-8;
      opt.max_iter = 60;
      hacdcpf::PowerFlowOptions plain = opt;
      plain.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
      plain.robust_nonlinear.enable_nonmonotone_linesearch = false;
      const auto nr = hacdcpf::solve_power_flow(sys, plain);
      const auto ho = hacdcpf::solve_power_flow_homotopy(sys, opt);
      const auto he = hacdcpf::solve_power_flow_helm(sys, opt);
      const auto nk = hacdcpf::solve_power_flow_newton_krylov(sys, opt);
      std::cout << std::scientific << std::setprecision(2);
      std::cout << basename_of(c) << ' ' << sys.ac.buses.size() << ' '
                << yn(nr.converged) << ' ' << nr.residual << ' '
                << yn(ho.converged) << ' ' << ho.residual << ' '
                << yn(he.converged) << ' ' << he.residual << ' '
                << yn(nk.converged) << ' ' << nk.residual << '\n';
    } catch (const std::exception& e) {
      std::cout << basename_of(c) << " ERROR " << e.what() << '\n';
    }
  }
  return 0;
}

// Load-scaling stress: plain Newton vs robust escalation vs homotopy.
int mode_stress(const std::vector<std::string>& cases) {
  const std::string c = cases.empty() ? "data/case30.m" : cases.front();
  std::cout << "# Load-scaling robustness on " << basename_of(c)
            << " (flat start, tol=1e-8; load scaled by gamma, generation fixed)\n";
  std::cout << "# columns: gamma plain_NR robust_NR homotopy (Y=converged)\n";
  const auto yn = [](bool b) { return b ? "Y" : "N"; };
  for (double g = 1.0; g <= 3.01; g += 0.25) {
    try {
      auto sys = hacdcpf::io::parse_matpower(c);
      for (auto& b : sys.ac.buses) {
        b.pd_mw *= g;
        b.qd_mvar *= g;
      }
      hacdcpf::PowerFlowOptions plain;
      plain.tol = 1e-8;
      plain.max_iter = 60;
      plain.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
      plain.robust_nonlinear.enable_nonmonotone_linesearch = false;
      hacdcpf::PowerFlowOptions robust;
      robust.tol = 1e-8;
      robust.max_iter = 60;
      const auto pr = hacdcpf::solve_power_flow(sys, plain);
      const auto rb = hacdcpf::solve_power_flow(sys, robust);
      const auto ho = hacdcpf::solve_power_flow_homotopy(sys, robust);
      std::cout << std::fixed << std::setprecision(1) << g << ' '
                << yn(pr.converged) << ' ' << yn(rb.converged) << ' '
                << yn(ho.converged) << '\n';
    } catch (const std::exception& e) {
      std::cout << g << " ERROR " << e.what() << '\n';
    }
  }
  return 0;
}

// Two-bus continuation power flow vs the analytic nose point.
int mode_cpf() {
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  hacdcpf::ACBus slack;
  slack.index = 1;
  slack.bus_type = hacdcpf::BusType::SLACK;
  slack.vm_pu = 1.0;
  slack.vmin_pu = 0.1;
  slack.vmax_pu = 1.2;
  hacdcpf::ACBus load;
  load.index = 2;
  load.bus_type = hacdcpf::BusType::PQ;
  load.vm_pu = 0.8;
  load.pd_mw = 50.0;
  load.qd_mvar = 20.0;
  load.vmin_pu = 0.1;
  load.vmax_pu = 1.2;
  sys.ac.buses = {slack, load};
  hacdcpf::ACBranch line;
  line.index = 1;
  line.from_bus = 1;
  line.to_bus = 2;
  line.r_pu = 0.0;
  line.x_pu = 0.5;
  line.tap = 1.0;
  line.in_service = true;
  sys.ac.branches = {line};
  hacdcpf::Generator gen;
  gen.index = 1;
  gen.bus = 1;
  gen.is_slack = true;
  gen.in_service = true;
  gen.vg_pu = 1.0;
  gen.qmin_mvar = -1000.0;
  gen.qmax_mvar = 1000.0;
  gen.pmin_mw = -1000.0;
  gen.pmax_mw = 1000.0;
  sys.ac.generators = {gen};

  const auto data = hacdcpf::powerflow::make_solver_data(sys);
  hacdcpf::powerflow::CpfSolver solver;
  solver.opts.enable_arc_length = true;
  solver.opts.lambda_max = 1.0;
  solver.opts.step_init = 0.02;
  solver.opts.step_min = 1e-5;
  solver.opts.step_max = 0.03;
  solver.opts.step_grow = 1.2;
  solver.opts.corrector_tol = 1e-10;
  solver.opts.trace_all_buses = true;
  solver.opts.vm_min_pu = 0.1;
  solver.opts.lower_branch_steps = 3;
  const auto res =
      solver.solve(data, hacdcpf::powerflow::CpfDirection::proportional(data));

  std::cout << "# Two-bus CPF (E=1, X=0.5 pu, load 50+j20 MVA, base 100 MVA)\n";
  std::cout << std::fixed << std::setprecision(5);
  std::cout << "nose_found=" << (res.nose_found ? "Y" : "N")
            << " arc_length=" << (res.arc_length_used ? "Y" : "N")
            << " lambda_max=" << res.lambda_max
            << " vm_at_nose=" << res.vm_at_nose
            << " p_max_mw=" << res.p_max_mw
            << " n_trace=" << res.trace.size() << '\n';
  std::cout << "# analytic (Q!=0): s=1+lambda solves 0.25 s^2 + 0.4 s - 1 = 0 "
               "-> s=1.3541, lambda_max=0.3541, V_nose=0.6038\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string mode = "nr";
  std::vector<std::string> cases;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (i == 1 && (a == "nr" || a == "fdpf" || a == "dc" || a == "bfs" ||
                   a == "robust" || a == "stress" || a == "cpf")) {
      mode = a;
      continue;
    }
    cases.push_back(a);
  }
  if (cases.empty()) {
    if (mode == "bfs") {
      cases = {"data/case33bw.m", "data/case69.m", "data/case85.m",
               "data/case141.m"};
    } else {
      cases = {"data/case9.m",   "data/case14.m",  "data/case30.m",
               "data/case57.m",  "data/case118.m", "data/case300.m"};
    }
  }

  if (mode == "nr") return mode_nr(cases);
  if (mode == "fdpf") return mode_fdpf(cases);
  if (mode == "dc") return mode_dc(cases);
  if (mode == "bfs") return mode_bfs(cases);
  if (mode == "robust") return mode_robust(cases);
  if (mode == "stress") return mode_stress(cases);
  if (mode == "cpf") return mode_cpf();
  std::cerr << "unknown mode: " << mode << "\n";
  return 2;
}
