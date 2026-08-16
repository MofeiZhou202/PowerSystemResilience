// tools/matpower_pf_compare.cpp
// -----------------------------
// Minimal MATPOWER AC PF runner used by external cross-tool checks.
// Prints a compact JSON object with convergence flags and solved voltages.
//
// Flags:
//   --no-pv-pq    disable PV->PQ switching (match MATPOWER default runpf)
//   --flows       append per-branch from/to flows and total losses
//   --repeat N    solve N extra times (warm cache) and report per-solve ms
//   --robust      enable the default NCP/DC-seed/homotopy escalation ladder
//   --no-robust   disable nonlinear escalation for direct-Newton baselines
//   --flat-start  reset all bus voltages to 1.0 pu / 0 deg before solving
//   --fixed-pv-pq keep one superset Jacobian pattern across Q-limit switches
//   --no-refactor disable KLU numeric refactorization
//   --prepared    use PreparedPowerFlowSession for repeated solves

#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <memory>
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
                 "[--repeat N] [--robust] [--flat-start] [--fixed-pv-pq]\n";
    return 2;
  }
  bool no_pv_pq = false;
  bool flows = false;
  bool robust = false;
  bool no_robust = false;
  bool flat_start = false;
  bool fixed_pv_pq = false;
  bool no_refactor = false;
  bool prepared = false;
  bool fdpf = false;
  bool dc_angle_seed = false;
  bool unconstrained_seed = false;
  bool ncp = false;
  bool smooth_ncp = false;
  double refactor_tol = 1e-8;
  int repeat = 0;
  for (int i = 2; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--no-pv-pq") {
      no_pv_pq = true;
    } else if (a == "--flows") {
      flows = true;
    } else if (a == "--robust") {
      robust = true;
    } else if (a == "--no-robust") {
      no_robust = true;
    } else if (a == "--flat-start") {
      flat_start = true;
    } else if (a == "--fixed-pv-pq") {
      fixed_pv_pq = true;
    } else if (a == "--no-refactor") {
      no_refactor = true;
    } else if (a == "--prepared") {
      prepared = true;
    } else if (a == "--fdpf") {
      fdpf = true;
    } else if (a == "--dc-angle-seed") {
      dc_angle_seed = true;
    } else if (a == "--unconstrained-seed") {
      unconstrained_seed = true;
    } else if (a == "--ncp") {
      ncp = true;
    } else if (a == "--smooth-ncp") {
      ncp = true;
      smooth_ncp = true;
    } else if (a == "--refactor-tol" && i + 1 < argc) {
      refactor_tol = std::stod(argv[++i]);
    } else if (a == "--repeat" && i + 1 < argc) {
      repeat = std::max(0, std::atoi(argv[++i]));
    }
  }

  try {
    const std::string case_path = argv[1];
    const auto t_parse0 = std::chrono::steady_clock::now();
    auto sys = hacdcpf::io::parse_matpower(case_path);
    const auto t_parse1 = std::chrono::steady_clock::now();
    if (flat_start) {
      for (auto& b : sys.ac.buses) {
        b.vm_pu = 1.0;
        b.va_deg = 0.0;
      }
    }

    hacdcpf::PowerFlowOptions opt;
    opt.tol = 1e-8;
    opt.max_iter = 80;
    opt.enable_solver_profiling = true;
    if (no_pv_pq) {
      opt.enable_pv_pq_conversion = false;
    }
    if (robust) {
      opt.robust_nonlinear.enable_homotopy_fallback_on_failure = true;
    }
    if (no_robust) {
      opt.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
    }
    if (fixed_pv_pq) {
      opt.robust_nonlinear.enable_fixed_pv_pq_layout = true;
    }
    opt.robust_nonlinear.enable_klu_numeric_refactor = !no_refactor;
    opt.robust_nonlinear.refactor_backward_error_tolerance = refactor_tol;
    opt.enable_semi_smooth_newton = ncp;
    opt.robust_nonlinear.enable_smooth_ncp = smooth_ncp;
    if (dc_angle_seed) {
      const auto dc_seed = hacdcpf::solve_ac_dc_power_flow(sys, opt);
      if (dc_seed.success && dc_seed.va.size() == sys.ac.buses.size()) {
        hacdcpf::InitialState initial;
        initial.va = dc_seed.va;
        initial.vm.reserve(sys.ac.buses.size());
        for (const auto& bus : sys.ac.buses) initial.vm.push_back(bus.vm_pu);
        initial.vdc.reserve(sys.dc.buses.size());
        for (const auto& bus : sys.dc.buses) initial.vdc.push_back(bus.vm_pu);
        opt.initial_state = std::move(initial);
      }
    }
    if (unconstrained_seed) {
      auto seed_options = opt;
      seed_options.enable_pv_pq_conversion = false;
      seed_options.robust_nonlinear.enable_homotopy_fallback_on_failure = false;
      const auto seed = hacdcpf::solve_power_flow(sys, seed_options);
      if (seed.converged) {
        opt.initial_state = hacdcpf::InitialState{seed.vm, seed.va, seed.vdc};
      }
    }

    const auto ms = [](auto a, auto b) {
      return std::chrono::duration<double, std::milli>(b - a).count();
    };

    std::unique_ptr<hacdcpf::PreparedPowerFlowSession> prepared_session;
    if (prepared) {
      prepared_session =
          std::make_unique<hacdcpf::PreparedPowerFlowSession>(opt);
    }
    auto solve_once = [&]() {
      if (fdpf) return hacdcpf::solve_power_flow_fdpf(sys, opt);
      return prepared_session ? prepared_session->solve(sys)
                              : hacdcpf::solve_power_flow(sys, opt);
    };
    auto pf = solve_once();
    const auto t_solve1 = std::chrono::steady_clock::now();

    std::vector<double> repeat_ms;
    repeat_ms.reserve(static_cast<size_t>(repeat));
    for (int k = 0; k < repeat; ++k) {
      const auto t0 = std::chrono::steady_clock::now();
      pf = solve_once();
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
              << "\",\"jacobian_pattern_rebuilds\":"
              << pf.profiling.jacobian_pattern_rebuilds
              << ",\"jacobian_analyze_calls\":"
              << pf.profiling.jacobian_analyze_calls
              << ",\"factorization_calls\":" << pf.profiling.factorization_calls
              << ",\"numeric_refactor_attempts\":"
              << pf.profiling.numeric_refactor_attempts
              << ",\"numeric_refactor_accepted\":"
              << pf.profiling.numeric_refactor_accepted
              << ",\"numeric_refactor_fallbacks\":"
              << pf.profiling.numeric_refactor_fallbacks
              << ",\"max_refactor_backward_error\":"
              << pf.profiling.max_refactor_backward_error
              << ",\"eval_jacobian_ms_total\":" << pf.profiling.eval_jacobian_ms_total
              << ",\"linear_solve_ms_total\":" << pf.profiling.linear_solve_ms_total
              << ",\"line_search_ms_total\":" << pf.profiling.line_search_ms_total
              << ",\"residual_evaluation_ms_total\":"
              << pf.profiling.residual_evaluation_ms_total
              << ",\"scaling_ms_total\":" << pf.profiling.scaling_ms_total
              << ",\"active_set_scan_ms_total\":"
              << pf.profiling.active_set_scan_ms_total
              << ",\"projection_ms_total\":" << pf.profiling.projection_ms_total
              << ",\"assembly_ms_total\":" << pf.profiling.assembly_ms_total
              << ",\"result_derivation_ms_total\":"
              << pf.profiling.result_derivation_ms_total
              << ",\"solver_core_ms_total\":" << pf.profiling.solver_core_ms_total
              << ",\"unclassified_core_ms_total\":"
              << pf.profiling.unclassified_core_ms_total
              << ",\"facade_ms_total\":" << pf.profiling.facade_ms_total
              << ",\"unclassified_facade_ms_total\":"
              << pf.profiling.unclassified_facade_ms_total
              << ",\"prepared_session_rebuilds\":"
              << pf.profiling.prepared_session_rebuilds
              << ",\"prepared_session_reuses\":"
              << pf.profiling.prepared_session_reuses
              << ",\"prepared_session_numeric_refreshes\":"
              << pf.profiling.prepared_session_numeric_refreshes
              << ",\"pv_to_pq_switches\":" << pf.profiling.pv_to_pq_switches
              << ",\"pq_to_pv_switches\":" << pf.profiling.pq_to_pv_switches
              << ",\"pv_pq_outer_iterations\":"
              << pf.profiling.pv_pq_outer_iterations
              << ",\"pv_pq_repeated_active_sets\":"
              << pf.profiling.pv_pq_repeated_active_sets
              << ",\"smooth_ncp_continuation_updates\":"
              << pf.profiling.smooth_ncp_continuation_updates
              << ",\"smooth_ncp_final_mu\":"
              << pf.profiling.smooth_ncp_final_mu
              << ",\"q_limit_enforcement_requested\":"
              << (pf.reactive_limits.enforcement_requested ? "true" : "false")
              << ",\"q_limit_certified\":"
              << (pf.reactive_limits.certified ? "true" : "false")
              << ",\"q_limit_cycle_detected\":"
              << (pf.reactive_limits.active_set_cycle_detected ? "true" : "false")
              << ",\"q_limit_outer_limit_reached\":"
              << (pf.reactive_limits.outer_iteration_limit_reached ? "true" : "false")
              << ",\"q_limit_active_buses\":"
              << pf.reactive_limits.active_limited_buses
              << ",\"q_limit_max_violation_pu\":"
              << pf.reactive_limits.max_violation_pu
              << ",\"stagnation_detected\":"
              << (pf.profiling.stagnation_detected ? "true" : "false")
              << ",\"stagnation_exit_iteration\":"
              << pf.profiling.stagnation_exit_iteration
              << ",\"homotopy_fallback_attempted\":"
              << (pf.profiling.homotopy_fallback_attempted ? "true" : "false")
              << ",\"homotopy_fallback_succeeded\":"
              << (pf.profiling.homotopy_fallback_succeeded ? "true" : "false")
              << ",\"nonlinear_escalation_attempts\":"
              << pf.profiling.nonlinear_escalation_attempts
              << ",\"ncp_fallback_attempted\":"
              << (pf.profiling.ncp_fallback_attempted ? "true" : "false")
              << ",\"dc_angle_seed_attempted\":"
              << (pf.profiling.dc_angle_seed_attempted ? "true" : "false")
              << ",\"successful_fallback_stage\":\""
              << json_escape(pf.profiling.successful_fallback_stage) << "\""
              << ",\"warnings\":[";
    for (size_t i = 0; i < pf.diagnostics.warnings.size(); ++i) {
      if (i) std::cout << ',';
      std::cout << '"' << json_escape(pf.diagnostics.warnings[i]) << '"';
    }
    std::cout << "],\"bus_ids\":[";
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
