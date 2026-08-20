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
#include "hacdcpf/dynamics/DynamicModelBuilder.hpp"
#include "hacdcpf/dynamics/DynamicSolver.hpp"
#include "hacdcpf/dynamics/DynamicSystem.hpp"
#include "hacdcpf/dynamics/DynamicEvent.hpp"
#include "hacdcpf/reliability/reliability_assessment.hpp"

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

// AC OPF and DC OPF objective/convergence, plus the DC-vs-AC cost gap.
int mode_opf(const std::vector<std::string>& cases) {
  std::cout << "# AC OPF and DC OPF on standard cases (default options)\n";
  std::cout << "# columns: case n_bus ac_conv ac_obj ac_outer ac_stat ac_viol "
               "dc_conv dc_obj gap_pct\n";
  const auto yn = [](bool b) { return b ? "Y" : "N"; };
  for (const auto& c : cases) {
    try {
      auto sys = hacdcpf::io::parse_matpower(c);
      hacdcpf::opf::ACOPFOptions aopt;
      aopt.ac_solver_backend = hacdcpf::opf::ACOPFSolverBackend::ParityIPM;
      const auto ac = hacdcpf::solve_ac_opf(sys, aopt);
      const hacdcpf::opf::DCOPFOptions dopt;
      const auto dc = hacdcpf::solve_dc_opf(sys, dopt);
      const double gap =
          (ac.converged && dc.converged && std::abs(ac.objective) > 1e-9)
              ? 100.0 * (ac.objective - dc.objective) / ac.objective
              : 0.0;
      std::cout << basename_of(c) << ' ' << sys.ac.buses.size() << ' '
                << yn(ac.converged) << ' ' << std::fixed << std::setprecision(2)
                << ac.objective << ' ' << ac.outer_iterations << ' '
                << std::scientific << std::setprecision(2) << ac.max_stationarity
                << ' ' << ac.max_constraint_violation << ' ' << yn(dc.converged)
                << ' ' << std::fixed << std::setprecision(2) << dc.objective
                << ' ' << gap << '\n';
    } catch (const std::exception& e) {
      std::cout << basename_of(c) << " ERROR " << e.what() << '\n';
    }
  }
  return 0;
}

// Single-machine-infinite-bus small-signal: electromechanical mode + spectrum.
int mode_dyn() {
  const double H = 3.5, Xdp = 0.30, Xe = 0.20, Pg = 0.80, f0 = 50.0;
  hacdcpf::HybridPowerSystem sys;
  sys.base_mva = 100.0;
  sys.ac.base_mva = 100.0;
  sys.ac.freq_hz = f0;
  hacdcpf::ACBus b1;
  b1.index = 1;
  b1.bus_type = hacdcpf::BusType::SLACK;
  b1.vm_pu = 1.0;
  b1.vmin_pu = 0.5;
  b1.vmax_pu = 1.5;
  hacdcpf::ACBus b2;
  b2.index = 2;
  b2.bus_type = hacdcpf::BusType::PV;
  b2.vm_pu = 1.0;
  b2.vmin_pu = 0.5;
  b2.vmax_pu = 1.5;
  sys.ac.buses = {b1, b2};
  hacdcpf::ACBranch br;
  br.index = 1;
  br.from_bus = 1;
  br.to_bus = 2;
  br.r_pu = 0.0;
  br.x_pu = Xe;
  br.tap = 1.0;
  br.in_service = true;
  sys.ac.branches = {br};
  hacdcpf::Generator gs;
  gs.index = 1;
  gs.bus = 1;
  gs.is_slack = true;
  gs.vg_pu = 1.0;
  gs.in_service = true;
  gs.inertia_h = 1e6;  // frozen reference => effective infinite bus
  gs.xd_pu = 1.0;
  gs.xdp_pu = Xdp;
  gs.xdpp_pu = Xdp;
  gs.qmax_mvar = 1e4;
  gs.qmin_mvar = -1e4;
  gs.pmax_mw = 1e4;
  gs.pmin_mw = -1e4;
  hacdcpf::Generator g2;
  g2.index = 2;
  g2.bus = 2;
  g2.is_slack = false;
  g2.vg_pu = 1.0;
  g2.in_service = true;
  g2.pg_mw = Pg * 100.0;
  g2.inertia_h = H;
  g2.xd_pu = 1.0;
  g2.xdp_pu = Xdp;
  g2.xdpp_pu = Xdp;
  g2.pmax_mw = 1e4;
  g2.pmin_mw = -1e4;
  g2.qmax_mvar = 1e4;
  g2.qmin_mvar = -1e4;
  g2.dynamic_model.model_name = "ClassicalMachine";
  sys.ac.generators = {gs, g2};

  hacdcpf::dynamics::DynamicSolverOptions opt;
  opt.t_end_s = 0.1;
  opt.dt_s = 0.01;
  opt.compute_small_signal = true;
  opt.solver_type = hacdcpf::dynamics::DynamicSolverType::TrapezoidalNewton;
  const auto res = hacdcpf::run_transient_simulation(sys, opt);
  const auto pf = hacdcpf::solve_power_flow(sys, {});

  std::cout << "# SMIB classical machine small-signal (H=" << H << "s, Xdp=" << Xdp
            << ", Xe=" << Xe << ", Pg=" << Pg << "pu, f0=" << f0 << "Hz)\n";
  std::cout << "pf_converged=" << (pf.converged ? "Y" : "N");
  if (pf.vm.size() > 1)
    std::cout << " Vt2=" << pf.vm[1] << " theta2_deg=" << pf.va[1];
  std::cout << "\nmodal: computed=" << (res.modal.computed ? "Y" : "N")
            << " success=" << (res.modal.success ? "Y" : "N")
            << " n_diff=" << res.modal.n_differential
            << " stable=" << (res.modal.stable ? "Y" : "N") << '\n';
  std::cout << std::fixed << std::setprecision(4);
  for (const auto& m : res.modal.modes) {
    std::cout << "  lambda=" << m.eigen_real << (m.eigen_imag >= 0 ? "+" : "")
              << m.eigen_imag << "j f=" << m.frequency_hz << "Hz zeta="
              << m.damping_ratio << " osc=" << (m.oscillatory ? "Y" : "N")
              << " state=" << m.dominant_state << '\n';
  }
  return 0;
}

// Attach WSCC classical-machine data (H, Xd' on 100 MVA base) to case9 gens.
hacdcpf::HybridPowerSystem make_wscc9() {
  auto sys = hacdcpf::io::parse_matpower("data/case9.m");
  sys.ac.freq_hz = 60.0;  // WSCC 3-machine system is 60 Hz
  sys.ac.base_mva = 100.0;
  struct MD { int bus; double h; double xdp; };
  const MD md[] = {{1, 23.64, 0.0608}, {2, 6.40, 0.1198}, {3, 3.01, 0.1813}};
  for (auto& g : sys.ac.generators) {
    for (const auto& d : md) {
      if (g.bus == d.bus) {
        g.inertia_h = d.h;
        g.xdp_pu = d.xdp;
        g.xd_pu = d.xdp;    // classical: single reactance behind constant EMF
        g.xdpp_pu = d.xdp;
        g.dynamic_model.model_name = "ClassicalMachine";
      }
    }
  }
  return sys;
}

// Eigenvalue-spectrum stiffness ratio: max|Re λ| / min|Re λ| over damped modes.
double stiffness_ratio(const hacdcpf::dynamics::DynamicModalSummary& modal) {
  double lo = 1e30, hi = 0.0;
  for (const auto& m : modal.modes) {
    const double a = std::abs(m.eigen_real);
    if (a > 1e-6) { lo = std::min(lo, a); hi = std::max(hi, a); }
  }
  return (hi > 0.0 && lo < 1e29) ? hi / lo : 0.0;
}

// Multi-machine modal (WSCC 9-bus) + stiffness spectrum + integrator comparison
// under a bus fault. Real data for the dynamics manual "数值算例与基准" sections.
int mode_dyn2() {
  using namespace hacdcpf;
  std::cout << std::fixed << std::setprecision(4);

  // ---- Part A: WSCC 9-bus 3-machine electromechanical modes ----
  auto sys9 = make_wscc9();
  dynamics::DynamicSolverOptions mopt;
  mopt.t_end_s = 0.02;
  mopt.dt_s = 0.005;
  mopt.compute_small_signal = true;
  mopt.solver_type = dynamics::DynamicSolverType::TrapezoidalNewton;
  const auto r9 = hacdcpf::run_transient_simulation(sys9, mopt);
  std::cout << "# WSCC 9-bus 3-machine classical small-signal (60Hz, H={23.64,6.40,3.01})\n";
  std::cout << "modal_computed=" << (r9.modal.computed ? "Y" : "N")
            << " n_diff=" << r9.modal.n_differential
            << " stable=" << (r9.modal.stable ? "Y" : "N") << '\n';
  for (const auto& m : r9.modal.modes) {
    if (m.oscillatory && m.frequency_hz > 0.05)
      std::cout << "  em_mode lambda=" << m.eigen_real << (m.eigen_imag >= 0 ? "+" : "")
                << m.eigen_imag << "j f=" << m.frequency_hz << "Hz zeta="
                << m.damping_ratio << " state=" << m.dominant_state << '\n';
  }
  std::cout << "  stiffness_ratio=" << stiffness_ratio(r9.modal) << '\n';

  // ---- Part B: SMIB spectrum + stiffness (H=3.5, Xdp=0.30, Xe=0.20, Pg=0.80) ----
  {
    const double H = 3.5, Xdp = 0.30, Xe = 0.20, Pg = 0.80, f0 = 50.0;
    HybridPowerSystem sys;
    sys.base_mva = 100.0; sys.ac.base_mva = 100.0; sys.ac.freq_hz = f0;
    ACBus b1; b1.index = 1; b1.bus_type = BusType::SLACK; b1.vm_pu = 1.0;
    b1.vmin_pu = 0.5; b1.vmax_pu = 1.5;
    ACBus b2; b2.index = 2; b2.bus_type = BusType::PV; b2.vm_pu = 1.0;
    b2.vmin_pu = 0.5; b2.vmax_pu = 1.5;
    sys.ac.buses = {b1, b2};
    ACBranch br; br.index = 1; br.from_bus = 1; br.to_bus = 2; br.r_pu = 0.0;
    br.x_pu = Xe; br.tap = 1.0; br.in_service = true; sys.ac.branches = {br};
    Generator gs; gs.index = 1; gs.bus = 1; gs.is_slack = true; gs.vg_pu = 1.0;
    gs.in_service = true; gs.inertia_h = 1e6; gs.xd_pu = 1.0; gs.xdp_pu = Xdp;
    gs.xdpp_pu = Xdp; gs.qmax_mvar = 1e4; gs.qmin_mvar = -1e4; gs.pmax_mw = 1e4;
    gs.pmin_mw = -1e4;
    Generator g2; g2.index = 2; g2.bus = 2; g2.is_slack = false; g2.vg_pu = 1.0;
    g2.in_service = true; g2.pg_mw = Pg * 100.0; g2.inertia_h = H; g2.xd_pu = 1.0;
    g2.xdp_pu = Xdp; g2.xdpp_pu = Xdp; g2.pmax_mw = 1e4; g2.pmin_mw = -1e4;
    g2.qmax_mvar = 1e4; g2.qmin_mvar = -1e4;
    g2.dynamic_model.model_name = "ClassicalMachine";
    sys.ac.generators = {gs, g2};
    dynamics::DynamicSolverOptions opt; opt.t_end_s = 0.02; opt.dt_s = 0.005;
    opt.compute_small_signal = true;
    opt.solver_type = dynamics::DynamicSolverType::TrapezoidalNewton;
    const auto r = hacdcpf::run_transient_simulation(sys, opt);
    std::cout << "# SMIB spectrum (H=3.5,Xdp=0.30,Xe=0.20,Pg=0.80,f0=50)\n";
    for (const auto& m : r.modal.modes)
      std::cout << "  lambda=" << m.eigen_real << (m.eigen_imag >= 0 ? "+" : "")
                << m.eigen_imag << "j f=" << m.frequency_hz << "Hz zeta="
                << m.damping_ratio << '\n';
    std::cout << "  stiffness_ratio=" << stiffness_ratio(r.modal) << '\n';
  }

  // ---- Part C: integrator comparison under a bus fault (WSCC 9-bus) ----
  // Bolted-ish 3-phase shunt fault at bus 7, applied t=0.10s, cleared t=0.18s.
  std::cout << "# integrator comparison: WSCC 9-bus, 80ms fault at bus7\n";
  struct SolverSpec { const char* name; dynamics::DynamicSolverType t; };
  const SolverSpec solvers[] = {
      {"Trapezoidal", dynamics::DynamicSolverType::TrapezoidalNewton},
      {"BackwardEuler", dynamics::DynamicSolverType::BackwardEulerNewton},
      {"RK4", dynamics::DynamicSolverType::PartitionedRK4},
      {"Rosenbrock", dynamics::DynamicSolverType::RosenbrockEuler}};
  for (double dt : {0.010, 0.005}) {
    for (const auto& sp : solvers) {
      try {
        auto sysc = make_wscc9();
        dynamics::DynamicSolverOptions opt;
        opt.t_start_s = 0.0; opt.t_end_s = 3.0; opt.dt_s = dt;
        opt.compute_small_signal = false; opt.solver_type = sp.t;
        dynamics::DynamicModelBuilder builder;
        dynamics::DynamicSystem ds = builder.build(sysc, opt);
        dynamics::DynamicEvent f; f.time_s = 0.10;
        f.type = dynamics::DynamicEventType::FaultShunt; f.bus = 7; f.value = 1e3;
        dynamics::DynamicEvent c; c.time_s = 0.18;
        c.type = dynamics::DynamicEventType::ClearFault; c.bus = 7;
        ds.events.push_back(f); ds.events.push_back(c);
        dynamics::DynamicSolver solver;
        const auto r = solver.solve(ds);
        double peak = 0.0, ffin = 0.0;
        for (const auto& s : r.snapshots) {
          const double dfreq = std::abs(s.coi_frequency_hz - 60.0);
          peak = std::max(peak, dfreq);
          ffin = s.coi_frequency_hz;
        }
        std::cout << "  dt=" << dt << " " << sp.name
                  << " success=" << (r.success ? "Y" : "N")
                  << " peak_df=" << peak << "Hz coi_final=" << ffin
                  << "Hz newton=" << r.newton_iterations
                  << " rejected=" << r.rejected_steps << '\n';
      } catch (const std::exception& e) {
        std::cout << "  dt=" << dt << " " << sp.name << " ERROR " << e.what() << '\n';
      }
    }
  }
  return 0;
}

// IEEE RTS-24 reliability indices: non-sequential + sequential Monte Carlo.
int mode_rel() {
  using namespace hacdcpf;
  std::cout << std::fixed << std::setprecision(4);
  auto sys = io::parse_matpower("data/case24_ieee_rts.m");
  analysis::apply_ieee24_reliability_data(sys);
  std::cout << "# IEEE RTS-24 reliability (AC-only DC-OPF scope)\n";
  std::cout << "n_bus=" << sys.ac.buses.size() << " n_gen="
            << sys.ac.generators.size() << " n_branch="
            << sys.ac.branches.size() << '\n';

  // ---- Non-sequential (state-sampling) Monte Carlo ----
  {
    analysis::ReliabilityOptions opt;
    opt.max_iterations = 20000;
    opt.cov_threshold = 0.03;
    opt.seed = 12345;
    opt.enable_parallel = true;
    opt.compute_tail_risk = true;
    const auto r = analysis::run_nonsequential_mc(sys, opt);
    std::cout << "NSQ_MC converged=" << (r.converged ? "Y" : "N")
              << " iters=" << r.iterations_used << " cov=" << r.final_cov
              << " EENS=" << r.eens_mwh_yr << " LOLE=" << r.lole_hr_yr
              << " EDNS=" << r.edns_mw << " PLC=" << r.plc
              << " EENS_CVaR95=" << r.tail_risk.eens_cvar
              << " scope=" << r.model_scope << '\n';
  }

  // ---- Sequential (chronological) Monte Carlo ----
  {
    auto lp = analysis::build_ieee_rts24_load_profile(8736);
    analysis::ReliabilityOptions opt;
    opt.max_iterations = 400;
    opt.cov_threshold = 0.05;
    opt.seed = 12345;
    opt.hours_per_year = 8736;
    opt.enable_parallel = true;
    const auto r = analysis::run_sequential_mc(sys, lp, opt);
    std::cout << "SEQ_MC converged=" << (r.converged ? "Y" : "N")
              << " years=" << r.iterations_used << " cov=" << r.final_cov
              << " EENS=" << r.eens_mwh_yr << " LOLE=" << r.lole_hr_yr
              << " LOLF=" << r.lolf_occ_yr << '\n';
  }
  return 0;
}

// Deterministic N-1 distribution FMEA on radial feeders.  MATPOWER cases carry
// no reliability data, so per-branch screening-template rates (lambda=0.35/yr,
// r=10h) are used; results are a method demonstration, not a validated study.
int mode_relfmea(const std::vector<std::string>& cases) {
  using namespace hacdcpf;
  std::cout << std::fixed << std::setprecision(3);
  std::cout << "# Distribution N-1 FMEA (screening-template branch rates "
               "lambda=0.35/yr r=10h; template customers=10/MW)\n";
  std::cout << "# case n_bus n_branch n_cont n_loss EENS[MWh/yr] EDNS[MW] "
               "LOLE[hr/yr] LOLF[/yr] SAIFI SAIDI[hr/yr] scope\n";
  for (const auto& c : cases) {
    HybridPowerSystem sys;
    try {
      sys = io::parse_matpower(c);
    } catch (const std::exception& e) {
      std::cout << c << " parse_error " << e.what() << '\n';
      continue;
    }
    analysis::FMEAOptions opt;
    opt.verbose = false;
    const auto r = analysis::run_distribution_fmea(sys, opt);
    std::cout << c << ' ' << sys.ac.buses.size() << ' ' << sys.ac.branches.size()
              << ' ' << r.n_contingencies << ' ' << r.n_loss_contingencies
              << ' ' << r.eens_mwh_yr << ' ' << r.edns_mw << ' ' << r.lole_hr_yr
              << ' ' << r.lolf_occ_yr << ' ' << r.distribution_idx.saifi << ' '
              << r.distribution_idx.saidi << ' ' << r.model_scope << '\n';
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string mode = "nr";
  std::vector<std::string> cases;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (i == 1 && (a == "nr" || a == "fdpf" || a == "dc" || a == "bfs" ||
                   a == "robust" || a == "stress" || a == "cpf" ||
                   a == "opf" || a == "dyn" || a == "dyn2" || a == "rel" ||
                   a == "relfmea")) {
      mode = a;
      continue;
    }
    cases.push_back(a);
  }
  if (cases.empty()) {
    if (mode == "bfs" || mode == "relfmea") {
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
  if (mode == "opf") return mode_opf(cases);
  if (mode == "dyn") return mode_dyn();
  if (mode == "dyn2") return mode_dyn2();
  if (mode == "rel") return mode_rel();
  if (mode == "relfmea") return mode_relfmea(cases);
  std::cerr << "unknown mode: " << mode << "\n";
  return 2;
}
