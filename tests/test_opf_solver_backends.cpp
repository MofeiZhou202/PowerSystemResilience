// tests/test_opf_solver_backends.cpp
// =====================================================================
// Coverage for the AC OPF solver-backend selector (ACOPFSolverBackend),
// the gradient-based objective-scaling robustness improvement, and
// convergence on the internal hybrid AC/DC cases (case300_acdc /
// case2000_acdc).  See src/optimal_power_flow/ac_opf.cpp and
// src/optimal_power_flow/parity_ipm.cpp.
// =====================================================================

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/bpa_io.hpp"
#include "hacdcpf/io/json_io.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/dc_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"
#include "hacdcpf/power_flow/hybrid.hpp"
#include "hacdcpf/power_flow/lcc_model.hpp"

#ifndef HACDCPF_MATPOWER_DATA_DIR
#define HACDCPF_MATPOWER_DATA_DIR "../../external_data/matpower"
#endif

using namespace hacdcpf;

namespace {
std::string data_path(const std::string& f) {
  return std::string(HACDCPF_MATPOWER_DATA_DIR) + "/" + f;
}

void set_env_var(const char* key, const char* value) {
#if defined(_WIN32)
  _putenv_s(key, value);
#else
  setenv(key, value, 1);
#endif
}

void unset_env_var(const char* key) {
#if defined(_WIN32)
  _putenv_s(key, "");
#else
  unsetenv(key);
#endif
}

class ScopedEnvVar {
 public:
  ScopedEnvVar(const char* key, const char* value) : key_(key) {
    if (const char* old = std::getenv(key)) {
      had_old_ = true;
      old_value_ = old;
    }
    set_env_var(key, value);
  }

  ~ScopedEnvVar() {
    if (had_old_) {
      set_env_var(key_.c_str(), old_value_.c_str());
    } else {
      unset_env_var(key_.c_str());
    }
  }

 private:
  std::string key_;
  bool had_old_{false};
  std::string old_value_;
};
}  // namespace

// ── Internal hybrid AC/DC cases ──────────────────────────────────────────────

TEST_CASE("AC OPF converges on internal hybrid case300_acdc", "[opf][acdc]") {
  HybridPowerSystem sys = io::build_case300_acdc();
  REQUIRE_FALSE(sys.ac.buses.empty());
  REQUIRE_FALSE(sys.dc.buses.empty());  // genuinely hybrid

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 200;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  INFO("status=" << r.status);
  INFO("iterations=" << r.iterations
       << " max_constraint_violation=" << r.max_constraint_violation
       << " max_stationarity=" << r.max_stationarity
       << " linear_solver=" << r.profiling.linear_solver_backend
       << " factorization_calls=" << r.profiling.factorization_calls
       << " linear_solve_calls=" << r.profiling.linear_solve_calls
       << " accepted_steps=" << r.profiling.accepted_steps
       << " rejected_steps=" << r.profiling.rejected_steps);
  INFO("phase_one=" << r.profiling.phase_one_initial_violation << " -> "
       << r.profiling.phase_one_constraint_violation
       << " dual_fit=" << r.profiling.phase_one_dual_fit_residual
       << " iterations=" << r.profiling.phase_one_iterations
       << " factorizations=" << r.profiling.phase_one_factorizations
       << " backtracks=" << r.profiling.phase_one_backtracks
       << " solver=" << r.profiling.phase_one_linear_solver
       << " termination=" << r.profiling.phase_one_termination);
  CHECK(r.converged);
  CHECK(r.solver_path == opf::OPFSolverPath::ParityIPM);
  CHECK(r.objective > 0.0);
  CHECK_FALSE(r.vm.empty());
  CHECK(r.profiling.accepted_steps > 0);
  CHECK(r.profiling.accepted_steps <= r.iterations);
  CHECK(r.profiling.rejected_steps > 0);
  CHECK(std::isfinite(r.profiling.phase_one_initial_violation));
  CHECK(std::isfinite(r.profiling.phase_one_constraint_violation));
  CHECK(r.profiling.phase_one_constraint_violation <=
        r.profiling.phase_one_initial_violation);
  CHECK(r.profiling.phase_one_iterations <= opt.phase_one_max_iterations);
  CHECK(r.profiling.phase_one_factorizations <=
        opt.phase_one_max_factorizations);
  opf::ACOPFOptions phase_one_disabled = opt;
  phase_one_disabled.enable_phase_one = false;
  const opf::ACOPFResult baseline =
      opf::solve_ac_opf(sys, phase_one_disabled);
  REQUIRE(baseline.converged);
  CHECK_FALSE(r.profiling.phase_two_start_accepted);
  CHECK(std::abs(r.profiling.initial_primal_residual -
                 baseline.profiling.initial_primal_residual) <= 1e-12);
}

TEST_CASE("Bounded Phase I certifies a pure AC OPF warm start",
          "[opf][ac][phase_one]") {
  const HybridPowerSystem sys = io::parse_matpower(data_path("case30.m"));
  REQUIRE(sys.dc.buses.empty());

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.allow_fallback = false;
  opt.max_inner_iterations = 200;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  INFO("status=" << r.status
       << " phase_one=" << r.profiling.phase_one_initial_violation << " -> "
       << r.profiling.phase_one_constraint_violation
       << " dual_fit=" << r.profiling.phase_one_dual_fit_residual
       << " iterations=" << r.profiling.phase_one_iterations
       << " factorizations=" << r.profiling.phase_one_factorizations
       << " backtracks=" << r.profiling.phase_one_backtracks
       << " solver=" << r.profiling.phase_one_linear_solver
       << " termination=" << r.profiling.phase_one_termination);
  CHECK(r.converged);
  CHECK_FALSE(r.profiling.phase_one_primal_feasible);
  CHECK(r.profiling.phase_one_in_handoff_corridor);
  CHECK(r.profiling.phase_one_dual_initialized);
  CHECK(r.profiling.phase_one_constraint_violation <=
        r.profiling.phase_one_handoff_primal_tolerance);
  CHECK(r.profiling.phase_one_perturbed_primal_residual <=
        r.profiling.phase_one_handoff_primal_tolerance);
  CHECK(r.profiling.phase_one_centrality <=
        opt.phase_one_centrality_tolerance);
  CHECK(r.profiling.phase_one_dual_fit_residual <= 1e-6);
  CHECK(r.profiling.phase_one_iterations <= opt.phase_one_max_iterations);
  CHECK(r.profiling.phase_one_factorizations <=
        opt.phase_one_max_factorizations);
  CHECK(r.profiling.phase_one_structure == "ac-state-basic");
  CHECK_FALSE(r.profiling.phase_one_structural_step_attempted);
  CHECK(r.profiling.phase_one_backtracks <=
        opt.phase_one_max_iterations * opt.phase_one_max_backtracks);
  CHECK(r.profiling.phase_two_start_accepted);
}

TEST_CASE("Parity Phase I obeys a zero-factorization budget",
          "[opf][acdc][phase_one][budget]") {
  const HybridPowerSystem sys = io::build_case300_acdc();
  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.allow_fallback = false;
  opt.max_inner_iterations = 1;
  opt.max_outer_iterations = 1;
  opt.phase_one_max_factorizations = 0;
  opt.phase_one_admission_mu_factor = 20.0;
  opt.phase_one_max_backtracks = 4;
  opt.phase_one_time_limit_ms = 1000.0;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  CHECK(r.profiling.phase_one_factorizations == 0);
  CHECK(r.profiling.phase_one_iterations == 0);
  CHECK(r.profiling.phase_one_budget_exhausted);
  CHECK_FALSE(r.profiling.phase_one_dual_initialized);
  CHECK_FALSE(r.profiling.phase_two_start_accepted);
}

TEST_CASE("case300_acdc forwards iteration and tolerance options to Ipopt",
          "[opf][acdc][ipopt][options]") {
#ifndef HACDCPF_HAVE_IPOPT
  SKIP("embedded Ipopt is not available in this build");
#endif
  const HybridPowerSystem sys = io::build_case300_acdc();

  opf::ACOPFOptions capped_options;
  capped_options.ac_solver_backend = opf::ACOPFSolverBackend::Ipopt;
  capped_options.max_inner_iterations = 1;
  capped_options.max_outer_iterations = 1;
  capped_options.allow_fallback = false;
  capped_options.feasibility_tol = 1e-7;
  capped_options.stationarity_tol = 1e-7;
  capped_options.barrier_mu_min = 1e-9;
  const opf::ACOPFResult capped = opf::solve_ac_opf(sys, capped_options);

  INFO("capped status=" << capped.status
       << " iterations=" << capped.iterations
       << " primal=" << capped.max_constraint_violation
       << " dual=" << capped.max_stationarity);
  CHECK_FALSE(capped.converged);
  CHECK(capped.iterations <= 2);
  CHECK(capped.status.find("Max iterations exceeded") != std::string::npos);

  opf::ACOPFOptions full_options = capped_options;
  full_options.max_inner_iterations = 800;
  full_options.stationarity_tol = 1e-3;
  const opf::ACOPFResult full = opf::solve_ac_opf(sys, full_options);
  INFO("full status=" << full.status
       << " iterations=" << full.iterations
       << " primal=" << full.max_constraint_violation
       << " dual=" << full.max_stationarity);
  REQUIRE(full.converged);
  CHECK(full.iterations > capped.iterations);
  CHECK(full.iterations <= full_options.max_inner_iterations + 1);
  CHECK(full.max_constraint_violation < 1e-6);
  CHECK(full.max_stationarity < full_options.stationarity_tol);
}

TEST_CASE("case300_acdc reuses a compatible primal OPF warm start",
          "[opf][acdc][warm-start][performance]") {
  HybridPowerSystem sys = io::build_case300_acdc();

  opf::ACOPFOptions cold_options;
  cold_options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  cold_options.max_inner_iterations = 200;
  const opf::ACOPFResult cold = opf::solve_ac_opf(sys, cold_options);
  REQUIRE(cold.converged);
  REQUIRE_FALSE(cold.profiling.warm_start_used);

  opf::ACOPFOptions warm_options = cold_options;
  warm_options.warm_start = &cold;
  const opf::ACOPFResult warm = opf::solve_ac_opf(sys, warm_options);

  INFO("cold iterations=" << cold.iterations
       << " initial_primal=" << cold.profiling.initial_primal_residual);
  INFO("warm iterations=" << warm.iterations
       << " initial_primal=" << warm.profiling.initial_primal_residual);
  REQUIRE(warm.converged);
  CHECK(warm.profiling.warm_start_used);
  CHECK(warm.profiling.phase_one_termination ==
        "continuation-state-preserved");
  CHECK(warm.profiling.phase_two_start_accepted);
  CHECK(warm.profiling.initial_primal_residual <=
        cold.profiling.initial_primal_residual + 1e-10);
  CHECK(warm.iterations <= cold.iterations);
  CHECK(std::abs(warm.objective - cold.objective) <=
        1e-5 * std::max(1.0, std::abs(cold.objective)));
}

TEST_CASE("Prepared ACOPF session reuses exact formulation and compatible full state",
          "[opf][prepared-session][warm-start]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case118.m"));
  opf::ACOPFOptions options;
  options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.max_inner_iterations = 300;
  options.prepared_numeric_refactor = true;
  opf::PreparedACOPFSession session(options);

  const opf::ACOPFResult first = session.solve(sys);
  REQUIRE(first.converged);
  CHECK(first.profiling.prepared_session_used);
  CHECK_FALSE(first.profiling.formulation_reused);
  CHECK(first.profiling.parity_formulation_builds == 1);
  CHECK(first.ipm_layout_signature != 0);

  const opf::ACOPFResult repeated = session.solve(sys);
  REQUIRE(repeated.converged);
  CHECK(repeated.profiling.formulation_reused);
  CHECK(repeated.profiling.mapping_reused);
  CHECK(repeated.profiling.parity_formulation_builds == 0);
  CHECK(repeated.profiling.continuation_state_reused);
  CHECK(repeated.ipm_layout_signature == first.ipm_layout_signature);
  CHECK_FALSE(repeated.profiling.numeric_refactor_attempted);
  CHECK_FALSE(repeated.profiling.numeric_refactor_accepted);
  CHECK(repeated.profiling.numeric_refactor_status.find("unsupported") !=
        std::string::npos);
  CHECK(std::abs(repeated.objective - first.objective) <=
        1e-5 * std::max(1.0, std::abs(first.objective)));

  for (auto& bus : sys.ac.buses) {
    bus.pd_mw *= 1.005;
    bus.qd_mvar *= 1.005;
  }
  for (auto& load : sys.ac.loads) {
    load.p_mw *= 1.005;
    load.q_mvar *= 1.005;
  }
  const opf::ACOPFResult perturbed = session.solve(sys);
  REQUIRE(perturbed.converged);
  CHECK(perturbed.profiling.formulation_reused);
  CHECK(perturbed.profiling.mapping_reused);
  CHECK(perturbed.profiling.parity_formulation_builds == 0);
  CHECK(perturbed.ipm_layout_signature == first.ipm_layout_signature);
  CHECK(perturbed.profiling.continuation_state_reused);
  CHECK(perturbed.profiling.prepared_session_invalidation_reason.find(
            "numeric-parameters-refreshed") != std::string::npos);
  if (perturbed.profiling.factorization_calls > 0) {
    CHECK(perturbed.profiling.symbolic_reused);
    CHECK(perturbed.profiling.analyze_calls == 0);
  }
}

TEST_CASE("Prepared ACOPF session invalidates opaque state on layout change",
          "[opf][prepared-session][layout]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case30.m"));
  opf::ACOPFOptions options;
  options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.max_inner_iterations = 250;
  opf::PreparedACOPFSession session(options);

  const opf::ACOPFResult baseline = session.solve(sys);
  REQUIRE(baseline.converged);
  REQUIRE_FALSE(sys.ac.branches.empty());
  sys.ac.branches.front().in_service = false;

  const opf::ACOPFResult changed = session.solve(sys);
  CHECK(changed.ipm_layout_signature != 0);
  CHECK(changed.ipm_layout_signature != baseline.ipm_layout_signature);
  CHECK_FALSE(changed.profiling.continuation_state_reused);
  CHECK(changed.profiling.prepared_session_invalidation_reason ==
        "layout-signature-changed");
}

TEST_CASE("Prepared ACOPF session refreshes a hybrid AC/DC operating point",
          "[opf][acdc][prepared-session][warm-start]") {
  HybridPowerSystem sys = io::build_hybrid_acdc_microgrid_island();
  opf::ACOPFOptions options;
  options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.max_inner_iterations = 400;
  options.phase_one_admission_mu_factor = 2.0;
  opf::PreparedACOPFSession session(options);

  const opf::ACOPFResult first = session.solve(sys);
  REQUIRE(first.converged);
  REQUIRE(first.ipm_layout_signature != 0);

  REQUIRE(sys.ac.buses.size() > 3);
  sys.ac.buses[3].pd_mw *= 1.002;
  sys.ac.buses[3].qd_mvar *= 1.002;
  const opf::ACOPFResult refreshed = session.solve(sys);

  REQUIRE(refreshed.converged);
  CHECK(refreshed.profiling.formulation_reused);
  CHECK(refreshed.profiling.mapping_reused);
  CHECK(refreshed.profiling.parity_formulation_builds == 0);
  CHECK(refreshed.profiling.continuation_state_reused);
  CHECK(refreshed.ipm_layout_signature == first.ipm_layout_signature);
  CHECK(refreshed.profiling.prepared_session_invalidation_reason.find(
            "numeric-parameters-refreshed") != std::string::npos);
  if (refreshed.profiling.linear_solver_backend.find("sparse") !=
      std::string::npos) {
    CHECK(refreshed.profiling.symbolic_reused);
    CHECK(refreshed.profiling.analyze_calls == 0);
  }
}

TEST_CASE("Prepared parity structure distinguishes LCC mappings from parameters",
          "[opf][acdc][lcc][prepared-session][layout]") {
  const auto imported = io::parse_bpa_dat(HACDCPF_TEST_DATA_DIR "/dsp/2DC.dat");
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE_FALSE(imported.system.lcc_converters.empty());

  opf::parity::Problem problem = opf::parity::build_problem(imported.system);
  const std::uint64_t baseline_signature =
      opf::parity::problem_layout_signature(problem);

  HybridPowerSystem numeric_change = imported.system;
  numeric_change.lcc_converters.front().p_set_mw *= 0.999;
  CHECK(opf::parity::refresh_problem_numeric_data(problem, numeric_change));
  CHECK(opf::parity::problem_layout_signature(problem) == baseline_signature);

  HybridPowerSystem terminal_change = numeric_change;
  terminal_change.lcc_converters.front().ac_bus =
      terminal_change.lcc_converters.back().ac_bus;
  CHECK_FALSE(opf::parity::refresh_problem_numeric_data(problem,
                                                        terminal_change));

  HybridPowerSystem identity_change = numeric_change;
  ++identity_change.lcc_converters.front().index;
  CHECK_FALSE(opf::parity::refresh_problem_numeric_data(problem,
                                                        identity_change));
}

TEST_CASE("Hybrid microgrid OPF fixes DC reference voltage and balances DC generation",
          "[opf][acdc][slack][regression]") {
  HybridPowerSystem sys = io::build_hybrid_acdc_microgrid_island();
  REQUIRE(sys.ac.buses.front().bus_type == BusType::SLACK);
  REQUIRE(sys.dc.buses.front().bus_type == DCBusType::DC_V);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 400;
  opt.allow_fallback = false;
  // The production admission remains mu_0. This explicit research setting
  // exercises the audited structured corridor up to 2*mu_0.
  opt.phase_one_admission_mu_factor = 2.0;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  INFO("phase_one=" << r.profiling.phase_one_initial_violation << " -> "
       << r.profiling.phase_one_constraint_violation
       << " dual_fit=" << r.profiling.phase_one_dual_fit_residual
       << " structure=" << r.profiling.phase_one_structure
       << " structural_violation="
       << r.profiling.phase_one_structural_violation
       << " termination=" << r.profiling.phase_one_termination);
  REQUIRE(r.converged);
  REQUIRE(r.vm.size() == sys.ac.buses.size());
  REQUIRE(r.vdc.size() == sys.dc.buses.size());
  REQUIRE(r.pac_mw.size() == 1);
  CHECK(std::abs(r.vdc[0] - sys.dc.buses[0].vm_pu) < 2e-6);

  // DC demand is 0.85 MW and DC solar injects 0.60 MW, so the AC-side VSC
  // exchange is around 0.25 MW plus converter/DC losses, not 1.45 MW as when
  // the static-generator sign was incorrectly treated as additional demand.
  CHECK(std::abs(r.pac_mw[0]) < 0.5);
  CHECK(r.max_constraint_violation < 1e-5);
  CHECK(r.profiling.phase_one_structural_step_attempted);
  CHECK(r.profiling.phase_one_structural_step_accepted);
  CHECK(r.profiling.phase_one_structural_factorizations == 1);
  CHECK(r.profiling.phase_one_structure == "hybrid-dc-converter-schur");
  CHECK(r.profiling.phase_one_structural_violation <
        r.profiling.phase_one_initial_violation);
  CHECK_FALSE(r.profiling.phase_one_primal_feasible);
  CHECK(r.profiling.phase_one_in_handoff_corridor);
  CHECK(r.profiling.phase_one_dual_initialized);
  CHECK(r.profiling.phase_one_constraint_violation <=
        r.profiling.phase_one_handoff_primal_tolerance);
  CHECK(r.profiling.phase_one_perturbed_primal_residual <=
        r.profiling.phase_one_handoff_primal_tolerance);
  CHECK(r.profiling.phase_one_centrality <=
        opt.phase_one_centrality_tolerance);
  CHECK(r.profiling.phase_one_dual_fit_residual <= 1e-6);
  CHECK(r.profiling.phase_two_start_accepted);
}

TEST_CASE("Market 5-bus AC/DC toy OPF preserves 1-based DC references",
          "[opf][acdc][regression]") {
  const HybridPowerSystem sys = io::build_market_5bus_acdc_toy();
  REQUIRE(sys.dc.buses.size() == 2);
  REQUIRE(sys.dc.branches.size() == 1);
  CHECK(sys.dc.buses[0].index == 1);
  CHECK(sys.dc.buses[1].index == 2);
  CHECK(sys.dc.branches[0].from_bus == 1);
  CHECK(sys.dc.branches[0].to_bus == 2);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.allow_fallback = false;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  INFO("status=" << r.status << " backend="
                  << r.profiling.linear_solver_backend);
  CHECK(r.converged);
  CHECK(r.solver_path == opf::OPFSolverPath::ParityIPM);
}

TEST_CASE("ACOPF skips DC Phase I when baseline PF dual is acceptable",
          "[opf][ac][phase_one][admission]") {
  const HybridPowerSystem sys = io::parse_matpower(data_path("case9.m"));
  opf::ACOPFOptions options;
  options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.ac_pf_warm_start = true;
  options.ac_pf_dc_phase_one = true;
  options.ac_pf_dc_phase_one_min_buses = 0;
  options.ac_pf_dc_phase_one_baseline_dual_threshold =
      std::numeric_limits<double>::infinity();
  options.max_inner_iterations = 200;

  const opf::ACOPFResult result = opf::solve_ac_opf(sys, options);
  INFO("status=" << result.status
                 << " phase_one=" << result.profiling.dc_phase_one_status);
  CHECK(result.converged);
  CHECK(result.profiling.dc_phase_one_requested);
  CHECK_FALSE(result.profiling.dc_phase_one_accepted);
  CHECK(result.profiling.dc_phase_one_iterations == 0);
  CHECK(result.profiling.dc_phase_one_runtime_ms == 0.0);
  CHECK(result.profiling.parity_formulation_builds == 1);
  CHECK(result.profiling.dc_phase_one_symbolic_analyze_calls == 0);
  CHECK(result.profiling.dc_phase_one_status.find("skipped") !=
        std::string::npos);
}

TEST_CASE("AC-PF dispatch dual predictor is factorization-free and audited",
          "[opf][ac][phase_one][dual_predictor]") {
  const HybridPowerSystem sys = io::parse_matpower(data_path("case118.m"));
  opf::ACOPFOptions options;
  options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  options.allow_fallback = false;
  options.ac_pf_warm_start = true;
  options.ac_pf_dc_phase_one = false;
  options.phase_one_admission_mu_factor = 0.0;
  options.phase_one_dispatch_dual_predictor = true;
  options.phase_one_dispatch_dual_min_improvement = 0.0;
  options.max_inner_iterations = 300;

  const opf::ACOPFResult result = opf::solve_ac_opf(sys, options);
  INFO("status=" << result.status
                 << " predictor="
                 << result.profiling.dispatch_dual_predictor_status
                 << " raw="
                 << result.profiling.dispatch_dual_predictor_baseline_raw
                 << " -> "
                 << result.profiling.dispatch_dual_predictor_candidate_raw);
  REQUIRE(result.converged);
  CHECK(result.profiling.phase_one_factorizations == 0);
  CHECK(result.profiling.dispatch_dual_predictor_attempted);
  CHECK(result.profiling.dispatch_dual_predictor_accepted);
  CHECK(result.profiling.dispatch_dual_predictor_runtime_ms >= 0.0);
  CHECK(result.profiling.dispatch_dual_predictor_candidate_raw <=
        result.profiling.dispatch_dual_predictor_baseline_raw);
  CHECK(result.profiling.dispatch_dual_predictor_candidate_normalized <=
        result.profiling.dispatch_dual_predictor_baseline_normalized);
  CHECK_FALSE(result.profiling.phase_two_start_accepted);
}

TEST_CASE("ACOPF Phase I profiling survives JSON round trip",
          "[opf][ac][phase_one][json]") {
  opf::ACOPFResult result;
  result.profiling.warm_start_used = true;
  result.profiling.prepared_session_used = true;
  result.profiling.formulation_reused = true;
  result.profiling.mapping_reused = true;
  result.profiling.symbolic_reused = true;
  result.profiling.continuation_state_reused = true;
  result.profiling.numeric_refactor_attempted = true;
  result.profiling.numeric_refactor_accepted = false;
  result.profiling.numeric_refactor_relative_drift = 5e-4;
  result.profiling.numeric_refactor_backward_error = 2e-9;
  result.profiling.numeric_refactor_status = "rejected: audit";
  result.profiling.prepared_session_invalidation_reason = "none";
  result.profiling.initial_primal_residual = 0.125;
  result.profiling.initial_dual_residual = 42.0;
  result.profiling.dc_phase_one_requested = true;
  result.profiling.dc_phase_one_accepted = true;
  result.profiling.dc_phase_one_iterations = 5;
  result.profiling.dc_phase_one_runtime_ms = 17.5;
  result.profiling.dc_phase_one_time_limit_ms = 2000.0;
  result.profiling.dc_phase_one_budget_exhausted = true;
  result.profiling.dc_phase_one_budget_overshoot_ms = 3.5;
  result.profiling.dc_phase_one_symbolic_analyze_calls = 1;
  result.profiling.parity_formulation_builds = 1;
  result.profiling.dc_phase_one_residual = 2.5e-4;
  result.profiling.dc_phase_one_candidate_primal = 0.15;
  result.profiling.dc_phase_one_candidate_dual = 12.0;
  result.profiling.dc_phase_one_baseline_primal = 1.0e-3;
  result.profiling.dc_phase_one_baseline_dual = 84.0;
  result.profiling.dc_phase_one_status = "usable warm-start-only iterate";
  result.profiling.phase_one_in_handoff_corridor = true;
  result.profiling.phase_one_centrality = 0.25;
  result.profiling.phase_one_structure = "ac-state-basic";
  result.profiling.dispatch_dual_predictor_attempted = true;
  result.profiling.dispatch_dual_predictor_accepted = true;
  result.profiling.dispatch_dual_predictor_runtime_ms = 0.75;
  result.profiling.dispatch_dual_predictor_baseline_raw = 100.0;
  result.profiling.dispatch_dual_predictor_candidate_raw = 80.0;
  result.profiling.dispatch_dual_predictor_baseline_normalized = 10.0;
  result.profiling.dispatch_dual_predictor_candidate_normalized = 8.0;
  result.profiling.dispatch_dual_predictor_status = "accepted";
  result.profiling.phase_two_start_accepted = true;

  const auto round_trip =
      io::opf_result_from_json(io::opf_result_to_json(result, -1));
  const auto& profiling = round_trip.profiling;
  CHECK(profiling.warm_start_used);
  CHECK(profiling.prepared_session_used);
  CHECK(profiling.formulation_reused);
  CHECK(profiling.mapping_reused);
  CHECK(profiling.symbolic_reused);
  CHECK(profiling.continuation_state_reused);
  CHECK(profiling.numeric_refactor_attempted);
  CHECK_FALSE(profiling.numeric_refactor_accepted);
  CHECK(profiling.numeric_refactor_relative_drift == Catch::Approx(5e-4));
  CHECK(profiling.numeric_refactor_backward_error == Catch::Approx(2e-9));
  CHECK(profiling.numeric_refactor_status == "rejected: audit");
  CHECK(profiling.prepared_session_invalidation_reason == "none");
  CHECK(profiling.initial_primal_residual == Catch::Approx(0.125));
  CHECK(profiling.initial_dual_residual == Catch::Approx(42.0));
  CHECK(profiling.dc_phase_one_requested);
  CHECK(profiling.dc_phase_one_accepted);
  CHECK(profiling.dc_phase_one_iterations == 5);
  CHECK(profiling.dc_phase_one_runtime_ms == Catch::Approx(17.5));
  CHECK(profiling.dc_phase_one_time_limit_ms == Catch::Approx(2000.0));
  CHECK(profiling.dc_phase_one_budget_exhausted);
  CHECK(profiling.dc_phase_one_budget_overshoot_ms == Catch::Approx(3.5));
  CHECK(profiling.dc_phase_one_symbolic_analyze_calls == 1);
  CHECK(profiling.parity_formulation_builds == 1);
  CHECK(profiling.dc_phase_one_residual == Catch::Approx(2.5e-4));
  CHECK(profiling.dc_phase_one_candidate_primal == Catch::Approx(0.15));
  CHECK(profiling.dc_phase_one_candidate_dual == Catch::Approx(12.0));
  CHECK(profiling.dc_phase_one_baseline_primal == Catch::Approx(1.0e-3));
  CHECK(profiling.dc_phase_one_baseline_dual == Catch::Approx(84.0));
  CHECK(profiling.dc_phase_one_status == "usable warm-start-only iterate");
  CHECK(profiling.phase_one_in_handoff_corridor);
  CHECK(profiling.phase_one_centrality == Catch::Approx(0.25));
  CHECK(profiling.phase_one_structure == "ac-state-basic");
  CHECK(profiling.dispatch_dual_predictor_attempted);
  CHECK(profiling.dispatch_dual_predictor_accepted);
  CHECK(profiling.dispatch_dual_predictor_runtime_ms == Catch::Approx(0.75));
  CHECK(profiling.dispatch_dual_predictor_baseline_raw ==
        Catch::Approx(100.0));
  CHECK(profiling.dispatch_dual_predictor_candidate_raw ==
        Catch::Approx(80.0));
  CHECK(profiling.dispatch_dual_predictor_baseline_normalized ==
        Catch::Approx(10.0));
  CHECK(profiling.dispatch_dual_predictor_candidate_normalized ==
        Catch::Approx(8.0));
  CHECK(profiling.dispatch_dual_predictor_status == "accepted");
  CHECK(profiling.phase_two_start_accepted);
}

TEST_CASE("GUI Auto recovers the two built-in hybrid showcase OPFs",
          "[opf][acdc][gui][regression]") {
  for (const auto& named_case : {
           std::pair<const char*, HybridPowerSystem (*)(void)>{
               "ieee24_3area_acdc_expanded", io::build_ieee24_3area_acdc_expanded},
           {"multiscale_comprehensive_acdc", io::build_multiscale_comprehensive_acdc}}) {
    const HybridPowerSystem sys = named_case.second();
    opf::ACOPFOptions opt;
    opt.ac_solver_backend = opf::ACOPFSolverBackend::Auto;
    opt.enable_primal_dual = true;
    opt.use_parity_ipm = true;
    opt.allow_fallback = true;
    opt.ac_pf_warm_start = true;
    opt.max_inner_iterations = 400;

    const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
    INFO("case=" << named_case.first << " status=" << r.status
                 << " backend=" << r.profiling.linear_solver_backend
                 << " primal=" << r.max_constraint_violation
                 << " dual=" << r.max_stationarity);
    CHECK(r.converged);
    CHECK(r.max_constraint_violation < 1e-6);
    CHECK(r.max_stationarity < 1e-3);
  }
}

TEST_CASE("Bare DC_V bus is rejected before OPF formulation",
          "[opf][acdc][slack][regression]") {
  HybridPowerSystem sys = io::build_actual_value_demo_acdc();
  REQUIRE(sys.dc.buses.front().bus_type == DCBusType::DC_V);
  REQUIRE(sys.vsc_converters.empty());
  REQUIRE_FALSE(sys.dc.dc_storage.empty());
  sys.dc.dc_storage.clear();

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 400;
  opt.allow_fallback = false;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  CHECK_FALSE(r.converged);
  CHECK(r.iterations == 0);
  CHECK(r.status.find("reference-bus eligibility check failed") !=
        std::string::npos);
  REQUIRE_FALSE(r.infeasibility_hints.empty());
  CHECK(r.infeasibility_hints.front().find("DC voltage-reference bus 1") !=
        std::string::npos);
}

TEST_CASE("DC slack KCL Jacobian matches finite differences",
          "[opf][acdc][slack][jacobian][regression]") {
  const HybridPowerSystem sys = io::build_hybrid_acdc_microgrid_island();
  opf::parity::ParityOptions parity_opt;
  const opf::parity::Problem prob =
      opf::parity::build_problem(sys, parity_opt);

  Eigen::VectorXd xmin, xmax, x;
  opf::parity::build_variable_bounds(prob, xmin, xmax);
  opf::parity::build_initial_point(prob, xmin, xmax, x);

  opf::parity::EvalWorkspace ws;
  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> jacobian;
  opf::parity::equality_constraints(prob, x, ws, g);
  opf::parity::equality_jacobian(prob, x, ws, jacobian);

  constexpr double eps = 1e-7;
  std::vector<int> columns;
  for (int k = 0; k < prob.vidx.n_vdc; ++k)
    columns.push_back(prob.vidx.i_vdc + k);
  for (const int col : columns) {
    Eigen::VectorXd xp = x;
    Eigen::VectorXd xm = x;
    xp[col] += eps;
    xm[col] -= eps;
    opf::parity::EvalWorkspace wsp, wsm;
    Eigen::VectorXd gp, gm;
    opf::parity::equality_constraints(prob, xp, wsp, gp);
    opf::parity::equality_constraints(prob, xm, wsm, gm);
    const Eigen::VectorXd finite_difference = (gp - gm) / (2.0 * eps);
    const Eigen::VectorXd analytic = Eigen::VectorXd(jacobian.col(col));
    CHECK((analytic - finite_difference).lpNorm<Eigen::Infinity>() < 1e-6);
  }
}

TEST_CASE("LCC parity balance Jacobian matches finite differences",
          "[opf][acdc][lcc][jacobian]") {
  const auto imported =
      io::parse_bpa_dat(HACDCPF_TEST_DATA_DIR "/dsp/2DC.dat");
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE(imported.system.lcc_converters.size() == 2);

  const opf::parity::Problem prob = opf::parity::build_problem(imported.system);
  REQUIRE(prob.data.lcc_converters.size() == 2);
  Eigen::VectorXd xmin, xmax, x;
  opf::parity::build_variable_bounds(prob, xmin, xmax);
  opf::parity::build_initial_point(prob, xmin, xmax, x);

  const PowerFlowResult pf = powerflow::solve_hybrid(imported.system);
  REQUIRE(pf.converged);
  REQUIRE(pf.vm.size() == static_cast<size_t>(prob.vidx.n_vm));
  REQUIRE(pf.vdc.size() == static_cast<size_t>(prob.vidx.n_vdc));
  for (int i = 0; i < prob.vidx.n_vm; ++i)
    x[prob.vidx.i_vm + i] = pf.vm[static_cast<size_t>(i)];
  for (int i = 0; i < prob.vidx.n_vdc; ++i)
    x[prob.vidx.i_vdc + i] = pf.vdc[static_cast<size_t>(i)];
  // Both DSP schedules sit on I_rated. Move fixed-power stations just off the
  // clamp kink and place characteristic-controlled stations at 90% rated
  // current by solving their public inverse characteristic.
  for (const auto& lcc : prob.data.lcc_converters) {
    const int dc = lcc.dc_bus - 1;
    if (lcc.control_mode == LCCControlMode::ConstantPower) {
      x[prob.vidx.i_vdc + dc] += 1e-4;
      continue;
    }
    if (lcc.control_mode != LCCControlMode::ConstantGamma &&
        lcc.control_mode != LCCControlMode::ConstantAlpha) {
      continue;
    }
    REQUIRE(lcc.rated_current_a > 0.0);
    const int commutation_ac =
        powerflow::lcc_commutation_ac_bus(prob.data, lcc);
    const double desired_id_ka =
        0.9 * lcc.rated_current_a * 1e-3;
    const double ud_kv =
        x[prob.vidx.i_vdc + dc] *
        powerflow::lcc_dc_base_kv(prob.data, lcc);
    const double angle_deg =
        lcc.control_mode == LCCControlMode::ConstantGamma
            ? lcc.gamma_set_deg
            : lcc.alpha_set_deg;
    const double required_e_kv =
        powerflow::lcc_required_valve_voltage_kv(
            lcc, ud_kv, desired_id_ka, angle_deg);
    const double sensitivity =
        powerflow::lcc_commutation_voltage_sensitivity_kv_per_pu(
            prob.data, lcc);
    REQUIRE(commutation_ac >= 0);
    REQUIRE(required_e_kv > 0.0);
    REQUIRE(sensitivity > 0.0);
    x[prob.vidx.i_vm + commutation_ac] = required_e_kv / sensitivity;

    const double terminal_e_kv =
        x[prob.vidx.i_vm + lcc.ac_bus - 1] * lcc.vn_ac_kv;
    const auto operating_point = powerflow::lcc_operating_point(
        lcc, terminal_e_kv, required_e_kv, ud_kv);
    REQUIRE(operating_point.valid);
    REQUIRE_FALSE(operating_point.id_at_limit);
    CHECK(std::abs(operating_point.id_ka - desired_id_ka) < 1e-10);
  }

  opf::parity::EvalWorkspace ws;
  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> jacobian;
  opf::parity::equality_constraints(prob, x, ws, g);
  opf::parity::equality_jacobian(prob, x, ws, jacobian);

  constexpr double eps = 1e-7;
  bool saw_distinct_commutation_bus = false;
  bool saw_nonzero_commutation_derivative = false;
  for (const auto& lcc : prob.data.lcc_converters) {
    const int ac = lcc.ac_bus - 1;
    const int commutation_ac =
        powerflow::lcc_commutation_ac_bus(prob.data, lcc);
    const int dc = lcc.dc_bus - 1;
    std::vector<int> columns{prob.vidx.i_vm + ac};
    if (commutation_ac != ac) {
      saw_distinct_commutation_bus = true;
      columns.push_back(prob.vidx.i_vm + commutation_ac);
    }
    columns.push_back(prob.vidx.i_vdc + dc);
    for (const int col : columns) {
      Eigen::VectorXd xp = x;
      Eigen::VectorXd xm = x;
      xp[col] += eps;
      xm[col] -= eps;
      opf::parity::EvalWorkspace wsp, wsm;
      Eigen::VectorXd gp, gm;
      opf::parity::equality_constraints(prob, xp, wsp, gp);
      opf::parity::equality_constraints(prob, xm, wsm, gm);
      const Eigen::VectorXd fd = (gp - gm) / (2.0 * eps);
      for (const int row : {prob.cidx.i_pbal_ac + ac,
                            prob.cidx.i_qbal_ac + ac,
                            prob.cidx.i_pbal_dc + dc}) {
        INFO("lcc=" << lcc.index << " row=" << row << " col=" << col
                    << " analytic=" << jacobian.coeff(row, col)
                    << " fd=" << fd[row]);
        CHECK(std::abs(jacobian.coeff(row, col) - fd[row]) < 2e-5);
        if (commutation_ac != ac &&
            col == prob.vidx.i_vm + commutation_ac &&
            std::abs(jacobian.coeff(row, col)) > 1e-8) {
          saw_nonzero_commutation_derivative = true;
        }
      }
    }
  }
  CHECK(saw_distinct_commutation_bus);
  CHECK(saw_nonzero_commutation_derivative);
}

TEST_CASE("Transformer-aware LCC parity Hessian matches Jacobian differences",
          "[opf][acdc][lcc][hessian][derivatives]") {
  const auto imported =
      io::parse_bpa_dat(HACDCPF_TEST_DATA_DIR "/dsp/2DC.dat");
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE(imported.system.lcc_converters.size() == 2);

  const opf::parity::Problem prob =
      opf::parity::build_problem(imported.system);
  const auto target = std::find_if(
      prob.data.lcc_converters.begin(), prob.data.lcc_converters.end(),
      [&](const LCCConverter& lcc) {
        return lcc.in_service &&
               lcc.control_mode == LCCControlMode::ConstantGamma &&
               powerflow::lcc_commutation_ac_bus(prob.data, lcc) !=
                   lcc.ac_bus - 1;
      });
  REQUIRE(target != prob.data.lcc_converters.end());

  Eigen::VectorXd xmin, xmax, x;
  opf::parity::build_variable_bounds(prob, xmin, xmax);
  opf::parity::build_initial_point(prob, xmin, xmax, x);
  const PowerFlowResult pf = powerflow::solve_hybrid(imported.system);
  REQUIRE(pf.converged);
  REQUIRE(pf.vm.size() == static_cast<size_t>(prob.vidx.n_vm));
  REQUIRE(pf.vdc.size() == static_cast<size_t>(prob.vidx.n_vdc));
  for (int i = 0; i < prob.vidx.n_vm; ++i)
    x[prob.vidx.i_vm + i] = pf.vm[static_cast<size_t>(i)];
  for (int i = 0; i < prob.vidx.n_vdc; ++i)
    x[prob.vidx.i_vdc + i] = pf.vdc[static_cast<size_t>(i)];

  const int ac = target->ac_bus - 1;
  const int commutation_ac =
      powerflow::lcc_commutation_ac_bus(prob.data, *target);
  const int dc = target->dc_bus - 1;
  REQUIRE(commutation_ac >= 0);
  REQUIRE(commutation_ac != ac);
  // The fixed imported tap places the calibrated point on the current clamp.
  // Keep the physical 3 kA limit and use the public inverse characteristic to
  // construct a smooth ConstantGamma point at 90% rated current.
  REQUIRE(target->rated_current_a > 0.0);
  const double desired_id_ka =
      0.9 * target->rated_current_a * 1e-3;
  const double ud_kv =
      x[prob.vidx.i_vdc + dc] *
      powerflow::lcc_dc_base_kv(prob.data, *target);
  const double required_e_kv =
      powerflow::lcc_required_valve_voltage_kv(
          *target, ud_kv, desired_id_ka, target->gamma_set_deg);
  const double sensitivity =
      powerflow::lcc_commutation_voltage_sensitivity_kv_per_pu(
          prob.data, *target);
  REQUIRE(required_e_kv > 0.0);
  REQUIRE(sensitivity > 0.0);
  x[prob.vidx.i_vm + commutation_ac] = required_e_kv / sensitivity;

  const double terminal_e_kv =
      x[prob.vidx.i_vm + ac] * target->vn_ac_kv;
  const auto operating_point = powerflow::lcc_operating_point(
      *target, terminal_e_kv, required_e_kv, ud_kv);
  REQUIRE(operating_point.valid);
  REQUIRE_FALSE(operating_point.id_at_limit);
  CHECK(std::abs(operating_point.id_ka - desired_id_ka) < 1e-10);
  CHECK(std::abs(operating_point.gamma_deg - target->gamma_set_deg) <
        1e-10);
  REQUIRE(operating_point.ud0_kv > 0.0);
  CHECK(ud_kv / operating_point.ud0_kv < 0.95);

  HybridPowerSystem without_lcc = imported.system;
  for (auto& lcc : without_lcc.lcc_converters) lcc.in_service = false;
  const opf::parity::Problem baseline =
      opf::parity::build_problem(without_lcc);
  REQUIRE(baseline.vidx.n_total == prob.vidx.n_total);

  Eigen::VectorXd lambda =
      Eigen::VectorXd::Zero(prob.cidx.n_eq_total);
  Eigen::VectorXd baseline_lambda =
      Eigen::VectorXd::Zero(baseline.cidx.n_eq_total);
  const auto set_multipliers = [&](const opf::parity::Problem& problem,
                                   Eigen::VectorXd& values) {
    values[problem.cidx.i_pbal_ac + ac] = 0.7;
    values[problem.cidx.i_qbal_ac + ac] = -0.4;
    values[problem.cidx.i_pbal_dc + dc] = 0.3;
  };
  set_multipliers(prob, lambda);
  set_multipliers(baseline, baseline_lambda);

  Eigen::SparseMatrix<double> hessian, baseline_hessian;
  opf::parity::lagrangian_hessian(
      prob, x, lambda, nullptr, hessian);
  opf::parity::lagrangian_hessian(
      baseline, x, baseline_lambda, nullptr, baseline_hessian);
  const Eigen::MatrixXd isolated_hessian =
      Eigen::MatrixXd(hessian) - Eigen::MatrixXd(baseline_hessian);
  CHECK((isolated_hessian - isolated_hessian.transpose())
            .cwiseAbs()
            .maxCoeff() < 1e-10);

  const auto equality_gradient =
      [](const opf::parity::Problem& problem,
         const Eigen::VectorXd& state,
         const Eigen::VectorXd& multipliers) {
        opf::parity::EvalWorkspace workspace;
        Eigen::VectorXd constraints;
        Eigen::SparseMatrix<double> jacobian;
        opf::parity::equality_constraints(
            problem, state, workspace, constraints);
        opf::parity::equality_jacobian(
            problem, state, workspace, jacobian);
        Eigen::VectorXd gradient = jacobian.transpose() * multipliers;
        return gradient;
      };
  const auto isolated_gradient =
      [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
    const Eigen::VectorXd with_lcc =
        equality_gradient(prob, state, lambda);
    const Eigen::VectorXd without_lcc =
        equality_gradient(baseline, state, baseline_lambda);
    return with_lcc - without_lcc;
  };

  const std::vector<int> local_columns{
      prob.vidx.i_vm + ac,
      prob.vidx.i_vm + commutation_ac,
      prob.vidx.i_vdc + dc,
  };
  const Eigen::VectorXd centre_gradient = isolated_gradient(x);
  Eigen::VectorXd state_vm(prob.vidx.n_vm);
  Eigen::VectorXd state_vdc(prob.vidx.n_vdc);
  for (int i = 0; i < prob.vidx.n_vm; ++i)
    state_vm[i] = x[prob.vidx.i_vm + i];
  for (int i = 0; i < prob.vidx.n_vdc; ++i)
    state_vdc[i] = x[prob.vidx.i_vdc + i];
  const auto target_jacobian = powerflow::lcc_ac_dc_jacobian(
      prob.data, *target, state_vm, state_vdc);
  CHECK(std::abs(target_jacobian.dpac_dvm_comm) +
            std::abs(target_jacobian.dqac_dvm_comm) +
            std::abs(target_jacobian.dpdc_dvm_comm) >
        1e-4);
  INFO("isolated LCC gradient: valve="
       << centre_gradient[local_columns[0]]
       << " commutation=" << centre_gradient[local_columns[1]]
       << " vdc=" << centre_gradient[local_columns[2]]
       << " commutation jac=[" << target_jacobian.dpac_dvm_comm << ","
       << target_jacobian.dqac_dvm_comm << ","
       << target_jacobian.dpdc_dvm_comm << "]");
  double max_commutation_curvature = 0.0;
  for (const int col : local_columns) {
    const double step =
        2e-6 * std::max(1.0, std::abs(x[col]));
    Eigen::VectorXd xp = x;
    Eigen::VectorXd xm = x;
    xp[col] += step;
    xm[col] -= step;
    const Eigen::VectorXd finite_difference =
        (isolated_gradient(xp) - isolated_gradient(xm)) /
        (2.0 * step);
    for (const int row : local_columns) {
      const double analytic = isolated_hessian(row, col);
      const double numeric = finite_difference[row];
      const double tolerance =
          5e-3 * std::max({1.0, std::abs(analytic), std::abs(numeric)});
      INFO("row=" << row << " col=" << col
                   << " analytic=" << analytic
                   << " numeric=" << numeric
                   << " tolerance=" << tolerance);
      CHECK(std::abs(analytic - numeric) < tolerance);
      if (row == prob.vidx.i_vm + commutation_ac ||
          col == prob.vidx.i_vm + commutation_ac) {
        max_commutation_curvature =
            std::max(max_commutation_curvature, std::abs(analytic));
      }
    }
  }
  CHECK(max_commutation_curvature > 1e-4);
}

TEST_CASE("Raw CIGRE LCC OPF converges and a bounded variant fails safely",
          "[opf][acdc][lcc][regression][raw-cigre]") {
  const auto imported =
      io::parse_bpa_dat(HACDCPF_TEST_DATA_DIR "/dsp/cigre.dat");
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE(imported.system.dc.branches.size() == 1);
  REQUIRE(imported.system.lcc_converters.size() == 2);
  REQUIRE(imported.system.ac.generators.size() == 2);

  // The LD card is a 10 ohm link. On the 500 kV / 100 MVA base this is
  // r_pu=0.004 and produces the calibrated 1500 -> 1410 MW transfer. The OPF
  // failure contract must never be obtained by deleting that physical loss.
  CHECK(std::abs(imported.system.dc.branches[0].r_pu - 0.004) < 1e-12);
  const PowerFlowResult pf = powerflow::solve_hybrid(imported.system);
  REQUIRE(pf.converged);
  REQUIRE(pf.lcc_transfers.size() == 2);
  const double dc_loss_mw = pf.lcc_transfers[0].p_dc_mw +
                            pf.lcc_transfers[1].p_dc_mw;
  CHECK(std::abs(dc_loss_mw - 90.0) < 2.0);
  const opf::parity::Problem raw_problem =
      opf::parity::build_problem(imported.system);
  REQUIRE(raw_problem.ac_angle_reference_buses.size() == 2);
  REQUIRE(raw_problem.cidx.n_ac_ref == 2);

  // BS is a balancing-machine declaration: its card has no Pmin field and DSP
  // permits negative output. The PF-derived warm start must recover those
  // actual balancing injections instead of reusing the authored zero dispatch.
  CHECK(imported.system.ac.generators[0].pmin_mw == -9999.0);
  CHECK(imported.system.ac.generators[1].pmin_mw == -9999.0);
  opf::ACOPFOptions options;
  options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  options.ac_pf_warm_start = true;
  options.allow_fallback = false;
  options.max_inner_iterations = 200;
  const opf::ACOPFResult result =
      opf::solve_ac_opf(imported.system, options);
  INFO("status=" << result.status
                 << " primal=" << result.max_constraint_violation
                 << " dual=" << result.max_stationarity
                 << " iterations=" << result.iterations);

  REQUIRE(result.converged);
  REQUIRE(result.solver_path == opf::OPFSolverPath::ParityIPM);
  REQUIRE(result.vm.size() == imported.system.ac.buses.size());
  REQUIRE(result.va.size() == imported.system.ac.buses.size());
  REQUIRE(result.pg_mw.size() == imported.system.ac.generators.size());
  REQUIRE(result.qg_mvar.size() == imported.system.ac.generators.size());
  REQUIRE(result.vdc.size() == imported.system.dc.buses.size());
  REQUIRE(result.lcc_transfers.size() == 2);
  CHECK(result.profiling.warm_start_used);
  CHECK(result.converter_model_scope.validity.lcc_quasi_steady_modelled);
  CHECK(std::abs(result.pg_mw[0] - 1500.0) < 2.0);
  CHECK(std::abs(result.pg_mw[1] + 1410.0) < 2.0);
  CHECK(std::abs(result.lcc_transfers[0].p_dc_mw +
                 result.lcc_transfers[1].p_dc_mw - 90.0) < 2.0);
  CHECK(result.max_constraint_violation < 1e-6);
  CHECK(std::abs(imported.system.dc.branches[0].r_pu - 0.004) < 1e-12);

  std::string serialized;
  REQUIRE_NOTHROW(serialized = io::opf_result_to_json(result, -1));
  REQUIRE_FALSE(serialized.empty());
  opf::ACOPFResult round_trip;
  REQUIRE_NOTHROW(round_trip = io::opf_result_from_json(serialized));
  CHECK(round_trip.pg_mw.size() == result.pg_mw.size());
  CHECK(round_trip.vdc.size() == result.vdc.size());
  REQUIRE(round_trip.lcc_transfers.size() == result.lcc_transfers.size());
  for (size_t i = 0; i < result.lcc_transfers.size(); ++i) {
    CHECK(round_trip.lcc_transfers[i].index == result.lcc_transfers[i].index);
    CHECK(std::abs(round_trip.lcc_transfers[i].p_ac_mw -
                   result.lcc_transfers[i].p_ac_mw) < 1e-12);
    CHECK(std::abs(round_trip.lcc_transfers[i].p_dc_mw -
                   result.lcc_transfers[i].p_dc_mw) < 1e-12);
  }

  // Recreate the former importer bug explicitly. With the receiving-end
  // balancing machine unable to absorb 1410 MW, the model is truly infeasible.
  // The solver must return a finite, serializable failure result, never an
  // access violation or a fabricated lossless DC solution.
  HybridPowerSystem infeasible_system = imported.system;
  infeasible_system.ac.generators[1].pmin_mw = 0.0;
  const opf::ACOPFResult infeasible =
      opf::solve_ac_opf(infeasible_system, options);
  INFO("infeasible status=" << infeasible.status
                            << " primal="
                            << infeasible.max_constraint_violation
                            << " dual=" << infeasible.max_stationarity
                            << " iterations=" << infeasible.iterations);
  CHECK_FALSE(infeasible.converged);
  CHECK(infeasible.solver_path == opf::OPFSolverPath::ParityIPM);
  CHECK_FALSE(infeasible.status.empty());
  CHECK(infeasible.vm.size() == infeasible_system.ac.buses.size());
  CHECK(infeasible.va.size() == infeasible_system.ac.buses.size());
  CHECK(infeasible.pg_mw.size() ==
        infeasible_system.ac.generators.size());
  CHECK(infeasible.qg_mvar.size() ==
        infeasible_system.ac.generators.size());
  CHECK((infeasible.vdc.empty() ||
         infeasible.vdc.size() == infeasible_system.dc.buses.size()));
  const auto all_finite = [](const std::vector<double>& values) {
    return std::all_of(values.begin(), values.end(),
                       [](double value) { return std::isfinite(value); });
  };
  CHECK(all_finite(infeasible.vm));
  CHECK(all_finite(infeasible.va));
  CHECK(all_finite(infeasible.pg_mw));
  CHECK(all_finite(infeasible.qg_mvar));
  CHECK(all_finite(infeasible.vdc));
  CHECK(std::abs(infeasible_system.dc.branches[0].r_pu - 0.004) <
        1e-12);
  REQUIRE_NOTHROW(serialized = io::opf_result_to_json(infeasible, -1));
  CHECK_FALSE(serialized.empty());
}

TEST_CASE("LCC OPF uses parity model and unsupported linear backends reject it",
          "[opf][acdc][lcc][regression]") {
  // CIGRE carries usable generator P bounds. The 2DC card case intentionally
  // has pmin=pmax=0 and is therefore a PF/Jacobian reference, not OPF-ready.
  auto imported =
      io::parse_bpa_dat(HACDCPF_TEST_DATA_DIR "/dsp/cigre.dat");
  REQUIRE_FALSE(imported.report.has_errors());
  REQUIRE(imported.system.lcc_converters.size() == 2);

  // Additional calibrated variant: explicitly pin the inverter DC voltage and
  // add a small convex dispatch cost while retaining the imported BS bounds.
  REQUIRE(imported.system.ac.generators.size() == 2);
  REQUIRE(imported.system.dc.buses.size() == 2);
  imported.system.dc.buses[1].bus_type = DCBusType::DC_V;
  imported.system.dc.buses[1].vm_pu = 470.0 / 500.0;
  for (auto& generator : imported.system.ac.generators) {
    generator.cost_c1 = 0.0;
    generator.cost_c2 = 1e-3;
  }
  const PowerFlowResult calibration_pf =
      powerflow::solve_hybrid(imported.system);
  REQUIRE(calibration_pf.converged);
  REQUIRE(calibration_pf.lcc_transfers.size() == 2);
  for (const auto& transfer : calibration_pf.lcc_transfers) {
    const size_t generator_row =
        transfer.station_role == static_cast<int>(LCCStationRole::Rectifier)
            ? 0u : 1u;
    imported.system.ac.generators[generator_row].pg_mw = -transfer.p_ac_mw;
    imported.system.ac.generators[generator_row].qg_mvar = -transfer.q_ac_mvar;
  }

  opf::ACOPFOptions parity_options;
  parity_options.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  parity_options.ac_pf_warm_start = true;
  parity_options.allow_fallback = false;
  parity_options.max_inner_iterations = 500;
  const opf::ACOPFResult result =
      opf::solve_ac_opf(imported.system, parity_options);
  INFO("status=" << result.status
                 << " primal=" << result.max_constraint_violation
                 << " dual=" << result.max_stationarity
                 << " initial_primal="
                 << result.profiling.initial_primal_residual
                 << " warm=" << result.profiling.warm_start_used
                 << " iterations=" << result.iterations
                 << " accepted=" << result.profiling.accepted_steps
                 << " rejected=" << result.profiling.rejected_steps
                 << " ac_p=" << result.profiling.max_ac_p_balance_residual_pu
                 << " ac_q=" << result.profiling.max_ac_q_balance_residual_pu
                 << " dc=" << result.profiling.max_dc_balance_residual_pu);
  for (size_t i = 0; i < result.vdc.size(); ++i)
    INFO("vdc[" << i << "]=" << result.vdc[i]);
  for (size_t i = 0; i < result.dpd_mw.size(); ++i) {
    if (std::abs(result.dpd_mw[i]) > 1e-6)
      INFO("dpd_mw[" << i << "]=" << result.dpd_mw[i]);
  }
  REQUIRE(result.solver_path == opf::OPFSolverPath::ParityIPM);
  REQUIRE(result.converged);
  REQUIRE(result.converter_model_scope.validity.lcc_quasi_steady_modelled);
  REQUIRE(result.lcc_transfers.size() == 2);
  REQUIRE(result.max_constraint_violation < 1e-6);
  REQUIRE(result.profiling.max_ac_p_balance_residual_pu < 1e-6);
  REQUIRE(result.profiling.max_ac_q_balance_residual_pu < 1e-6);
  REQUIRE(result.profiling.max_dc_balance_residual_pu < 1e-6);
  const LCCTransfer* rectifier = nullptr;
  const LCCTransfer* inverter = nullptr;
  for (const auto& transfer : result.lcc_transfers) {
    CHECK(transfer.q_ac_mvar < 0.0);
    CHECK(transfer.id_ka >= 0.0);
    if (transfer.station_role == static_cast<int>(LCCStationRole::Rectifier))
      rectifier = &transfer;
    if (transfer.station_role == static_cast<int>(LCCStationRole::Inverter))
      inverter = &transfer;
    const auto authored = std::find_if(
        imported.system.lcc_converters.begin(),
        imported.system.lcc_converters.end(),
        [&](const LCCConverter& lcc) { return lcc.index == transfer.index; });
    REQUIRE(authored != imported.system.lcc_converters.end());
    CHECK(transfer.bus_ac == authored->ac_bus);
    CHECK(transfer.bus_dc == authored->dc_bus);
  }
  REQUIRE(rectifier != nullptr);
  REQUIRE(inverter != nullptr);
  REQUIRE(result.vdc.size() == 2);
  REQUIRE(result.pg_mw.size() == 2);
  REQUIRE(result.qg_mvar.size() == 2);
  INFO("calibration vdc=[" << result.vdc[0] << "," << result.vdc[1]
       << "] pg_mw=[" << result.pg_mw[0] << "," << result.pg_mw[1]
       << "] qg_mvar=[" << result.qg_mvar[0] << "," << result.qg_mvar[1]
       << "] rect={P=" << rectifier->p_ac_mw
       << ",Q=" << rectifier->q_ac_mvar << ",Pdc=" << rectifier->p_dc_mw
       << ",Ud=" << rectifier->ud_kv << ",Id=" << rectifier->id_ka
       << ",alpha=" << rectifier->alpha_deg << "} inv={P="
       << inverter->p_ac_mw << ",Q=" << inverter->q_ac_mvar
       << ",Pdc=" << inverter->p_dc_mw << ",Ud=" << inverter->ud_kv
       << ",Id=" << inverter->id_ka << ",gamma=" << inverter->gamma_deg
       << "}");
  CHECK(std::abs(rectifier->p_ac_mw + 1500.0) < 2.0);
  CHECK(std::abs(inverter->p_ac_mw - 1410.0) < 2.0);
  CHECK(std::abs(rectifier->id_ka - 3.0) < 0.02);
  CHECK(std::abs(inverter->id_ka - 3.0) < 0.02);
  CHECK(std::abs(rectifier->p_dc_mw + inverter->p_dc_mw - 90.0) < 2.0);

  HybridPowerSystem replay_system = imported.system;
  // OPF treats converter-transformer taps as fixed formulation inputs. The
  // replay must solve that same network, not run the PF-only R-card outer
  // loop and silently change Ybus before comparing terminal injections.
  for (auto& lcc : replay_system.lcc_converters)
    lcc.tap_control_modelled = false;
  REQUIRE(result.pg_mw.size() == replay_system.ac.generators.size());
  REQUIRE(result.qg_mvar.size() == replay_system.ac.generators.size());
  for (size_t i = 0; i < replay_system.ac.generators.size(); ++i) {
    replay_system.ac.generators[i].pg_mw = result.pg_mw[i];
    replay_system.ac.generators[i].qg_mvar = result.qg_mvar[i];
  }
  for (size_t i = 0; i < replay_system.ac.buses.size(); ++i) {
    replay_system.ac.buses[i].vm_pu = result.vm[i];
    replay_system.ac.buses[i].va_deg =
        result.va[i] * 180.0 / 3.14159265358979323846;
  }
  for (size_t i = 0; i < replay_system.dc.buses.size(); ++i)
    replay_system.dc.buses[i].vm_pu = result.vdc[i];
  const PowerFlowResult replay = powerflow::solve_hybrid(replay_system);
  REQUIRE(replay.converged);
  REQUIRE(replay.lcc_transfers.size() == result.lcc_transfers.size());
  for (const auto& opf_transfer : result.lcc_transfers) {
    const auto pf_transfer = std::find_if(
        replay.lcc_transfers.begin(), replay.lcc_transfers.end(),
        [&](const LCCTransfer& transfer) {
          return transfer.index == opf_transfer.index;
        });
    REQUIRE(pf_transfer != replay.lcc_transfers.end());
    CHECK(std::abs(pf_transfer->p_ac_mw - opf_transfer.p_ac_mw) < 0.5);
    CHECK(std::abs(pf_transfer->q_ac_mvar - opf_transfer.q_ac_mvar) < 0.5);
    CHECK(std::abs(pf_transfer->p_dc_mw - opf_transfer.p_dc_mw) < 0.5);
  }

  opf::ACOPFOptions economic_options;
  economic_options.ac_solver_backend =
      opf::ACOPFSolverBackend::EconomicDispatch;
  const opf::ACOPFResult economic =
      opf::solve_ac_opf(imported.system, economic_options);
  CHECK_FALSE(economic.converged);
  CHECK(economic.status.find("hybrid AC/DC/LCC") != std::string::npos);

  const opf::DCOPFResult dc = opf::solve_dc_opf(imported.system);
  CHECK_FALSE(dc.converged);
  CHECK(dc.status.find("DC OPF rejected") != std::string::npos);
  CHECK(dc.converter_model_scope.model_scope == "dc-opf:rejected-lcc");

  HybridPowerSystem unsupported_control = imported.system;
  unsupported_control.lcc_converters[0].control_mode =
      LCCControlMode::ConstantGamma;
  const opf::ACOPFResult unsupported =
      opf::solve_ac_opf(unsupported_control, parity_options);
  CHECK_FALSE(unsupported.converged);
  CHECK(unsupported.iterations == 0);
  CHECK(unsupported.status.find("LCC readiness check failed") !=
        std::string::npos);

  HybridPowerSystem missing_characteristic = imported.system;
  missing_characteristic.lcc_converters[1].x_comm_ohm = 0.0;
  const opf::ACOPFResult missing =
      opf::solve_ac_opf(missing_characteristic, parity_options);
  CHECK_FALSE(missing.converged);
  CHECK(missing.iterations == 0);
  CHECK(missing.status.find("positive x_comm_ohm") != std::string::npos);
}

TEST_CASE("AC OPF converges on internal hybrid case2000_acdc", "[opf][acdc]") {
  HybridPowerSystem sys = io::build_case2000_acdc();
  REQUIRE(sys.ac.buses.size() > 1000);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 400;
  const auto started = std::chrono::steady_clock::now();
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
  const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();

  // Gradient-based objective scaling is what lets a ~2000-bus hybrid system
  // converge in the native IPM (it stalled before the scaling fix).
  INFO("case2000 native IPM: buses=" << sys.ac.buses.size()
       << " vsc=" << sys.vsc_converters.size()
       << " status=" << r.status
       << " iterations=" << r.iterations
       << " elapsed_sec=" << elapsed_sec
       << " factorization_calls=" << r.profiling.factorization_calls
       << " linear_solve_calls=" << r.profiling.linear_solve_calls
       << " primal=" << r.max_constraint_violation
       << " dual=" << r.max_stationarity
       << " ac_p=" << r.profiling.max_ac_p_balance_residual_pu
       << " ac_q=" << r.profiling.max_ac_q_balance_residual_pu
       << " dc=" << r.profiling.max_dc_balance_residual_pu
       << " converter=" << r.profiling.max_converter_balance_residual_pu
       << " backend=" << r.profiling.linear_solver_backend);
  CHECK(r.converged);
  CHECK(r.objective > 0.0);
  CHECK(r.max_constraint_violation < 1e-5);
  CHECK(r.iterations < 160);
  CHECK(std::isfinite(r.profiling.phase_one_initial_violation));
  CHECK(std::isfinite(r.profiling.phase_one_constraint_violation));
  CHECK(r.profiling.phase_one_constraint_violation <=
        r.profiling.phase_one_initial_violation);
  CHECK(r.profiling.phase_one_iterations <= opt.phase_one_max_iterations);
  CHECK(r.profiling.phase_one_factorizations <=
        opt.phase_one_max_factorizations);
}

TEST_CASE("case2000 AC/DC Ipopt bounded performance benchmark",
          "[.performance][opf][acdc][ipopt][case2000]") {
  const HybridPowerSystem sys = io::build_case2000_acdc();
  REQUIRE(sys.ac.buses.size() == 2000);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::Ipopt;
  opt.max_inner_iterations = 100;
  opt.max_outer_iterations = 1;
  opt.allow_fallback = false;
  opt.feasibility_tol = 1e-6;
  opt.stationarity_tol = 1e-3;
  opt.barrier_mu_min = 1e-8;

  const auto started = std::chrono::steady_clock::now();
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
  const double elapsed_sec = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();

  INFO("case2000 Ipopt: converged=" << r.converged
       << " iterations=" << r.iterations
       << " elapsed_sec=" << elapsed_sec
       << " primal=" << r.max_constraint_violation
       << " dual=" << r.max_stationarity
       << " status=" << r.status);
  CHECK(r.iterations <= opt.max_inner_iterations + 1);
  CHECK(std::isfinite(r.objective));
  CHECK(std::isfinite(r.max_constraint_violation));
  CHECK(std::isfinite(r.max_stationarity));
}

TEST_CASE("Parity derivatives stay sparse at 70000 DC buses",
          "[.performance][opf][acdc][scale][case70000]") {
  constexpr int n = 70000;
  HybridPowerSystem sys = io::build_case300_acdc();
  sys.dc.buses.resize(n);
  sys.dc.branches.resize(n - 1);
  for (int i = 0; i < n; ++i) {
    sys.dc.buses[i].index = i + 1;
    sys.dc.buses[i].bus_type = i == 0 ? DCBusType::DC_V : DCBusType::DC_P;
    if (i + 1 < n) {
      sys.dc.branches[i].index = i + 1;
      sys.dc.branches[i].from_bus = i + 1;
      sys.dc.branches[i].to_bus = i + 2;
      sys.dc.branches[i].r_pu = 0.005;
    }
  }
  const opf::parity::Problem prob = opf::parity::build_problem(sys);
  Eigen::VectorXd x = Eigen::VectorXd::Zero(prob.vidx.n_total);
  x.segment(prob.vidx.i_vm, prob.vidx.n_vm).setOnes();
  x.segment(prob.vidx.i_vdc, prob.vidx.n_vdc).setOnes();
  opf::parity::EvalWorkspace ws;
  Eigen::VectorXd g;
  Eigen::SparseMatrix<double> jg, hess;
  opf::parity::equality_constraints(prob, x, ws, g);
  opf::parity::equality_jacobian(prob, x, ws, jg);
  opf::parity::lagrangian_hessian(
      prob, x, Eigen::VectorXd::Ones(prob.cidx.n_eq_total), nullptr, hess);
  CHECK(prob.vidx.n_vdc == n);
  CHECK(prob.data.gdc.nonZeros() < 3 * n);
  CHECK(std::max(jg.nonZeros(), hess.nonZeros()) < 6 * n);
  CHECK(g.allFinite());
  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::Ipopt;
  opt.max_inner_iterations = 200;
  opt.max_outer_iterations = 1;
  opt.allow_fallback = false;
  opt.stationarity_tol = 1e-3;
  const opf::ACOPFResult result = opf::solve_ac_opf(sys, opt);
  CHECK(result.converged);
  CHECK(result.max_constraint_violation < 1e-6);
}

TEST_CASE("Ipopt OPF parameter stability matrix",
          "[.performance][opf][ipopt][parameter-matrix]") {
  struct MatrixConfig {
    int budget;
    double feasibility_tolerance;
    double stationarity_tolerance;
    double complementarity_tolerance;
  };

  const auto run = [](const HybridPowerSystem& system,
                      const char* case_name,
                      const MatrixConfig& config) {
    opf::ACOPFOptions opt;
    opt.ac_solver_backend = opf::ACOPFSolverBackend::Ipopt;
    opt.max_inner_iterations = config.budget;
    opt.max_outer_iterations = 1;
    opt.allow_fallback = false;
    opt.feasibility_tol = config.feasibility_tolerance;
    opt.stationarity_tol = config.stationarity_tolerance;
    opt.barrier_mu_min = config.complementarity_tolerance;

    const auto started = std::chrono::steady_clock::now();
    const opf::ACOPFResult result = opf::solve_ac_opf(system, opt);
    const double elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    std::cout << "IPOPT_MATRIX case=" << case_name
              << " budget=" << config.budget
              << " feasibility_tol=" << config.feasibility_tolerance
              << " stationarity_tol=" << config.stationarity_tolerance
              << " complementarity_tol=" << config.complementarity_tolerance
              << " converged=" << result.converged
              << " iterations=" << result.iterations
              << " elapsed_sec=" << elapsed_sec
              << " primal=" << result.max_constraint_violation
              << " dual=" << result.max_stationarity
              << " status=\"" << result.status << "\"\n";

    CAPTURE(case_name, config.budget, config.feasibility_tolerance,
            config.stationarity_tolerance,
            config.complementarity_tolerance, result.status);
    CHECK(result.iterations <= config.budget + 1);
    CHECK(std::isfinite(result.objective));
    CHECK(std::isfinite(result.max_constraint_violation));
    CHECK(std::isfinite(result.max_stationarity));
    return result;
  };

  SECTION("case30 tolerance and budget sweep") {
    const HybridPowerSystem system = io::parse_matpower(data_path("case30.m"));
    const MatrixConfig configs[] = {
        {1, 1e-8, 1e-8, 1e-10},
        {20, 1e-4, 1e-4, 1e-8},
        {100, 1e-6, 1e-6, 1e-8},
        {200, 1e-8, 1e-8, 1e-10},
    };
    for (const MatrixConfig& config : configs) run(system, "case30", config);
  }

  SECTION("case300 tolerance and budget sweep") {
    const HybridPowerSystem system = io::build_case300_acdc();
    const MatrixConfig configs[] = {
        {100, 1e-7, 1e-3, 1e-9},
        {500, 1e-7, 1e-3, 1e-9},
        {800, 1e-7, 1e-2, 1e-9},
        {800, 1e-7, 1e-3, 1e-9},
        {800, 1e-7, 1e-4, 1e-9},
    };
    for (const MatrixConfig& config : configs) run(system, "case300", config);
  }

  SECTION("case2000 bounded budget sweep") {
    const HybridPowerSystem system = io::build_case2000_acdc();
    const MatrixConfig configs[] = {
        {20, 1e-6, 1e-3, 1e-8},
        {50, 1e-6, 1e-3, 1e-8},
        {100, 1e-6, 1e-3, 1e-8},
    };
    for (const MatrixConfig& config : configs) run(system, "case2000", config);
  }
}

// ── Objective-scaling regression guard ───────────────────────────────────────

TEST_CASE("Parity OPF warm start accepts fixed renewable reactive bounds",
          "[opf][acdc][regression]") {
  HybridPowerSystem sys = io::build_dist33_microgrid_der();
  REQUIRE_FALSE(sys.ac.renewable_gens.empty());
  REQUIRE_FALSE(sys.ac.pv_systems.empty());

  for (auto& rg : sys.ac.renewable_gens) {
    rg.qmin_mvar = 0.0;
    rg.qmax_mvar = 0.0;
  }
  for (auto& pv : sys.ac.pv_systems) {
    pv.qmin_mvar = 0.0;
    pv.qmax_mvar = 0.0;
  }

  const opf::parity::Problem prob = opf::parity::build_problem(sys);
  Eigen::VectorXd xmin;
  Eigen::VectorXd xmax;
  Eigen::VectorXd x0;
  opf::parity::build_variable_bounds(prob, xmin, xmax);

  REQUIRE_NOTHROW(opf::parity::build_initial_point(prob, xmin, xmax, x0));
  REQUIRE(x0.size() == prob.vidx.n_total);
  CHECK(x0.allFinite());
  for (int i = 0; i < x0.size(); ++i) {
    CHECK(x0[i] >= std::min(xmin[i], xmax[i]) - 1e-10);
    CHECK(x0[i] <= std::max(xmin[i], xmax[i]) + 1e-10);
  }
}

TEST_CASE("Objective scaling lets the parity IPM converge on case2383wp", "[opf][scaling]") {
  // case2383wp (Polish winter peak) diverged badly before gradient-based
  // objective scaling (max stationarity ~0.45 at the iteration cap); it now
  // converges.  This is the regression guard for that scaling.
  HybridPowerSystem sys = io::parse_matpower(data_path("case2383wp.m"));

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 400;
  opt.max_outer_iterations = 1;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  CHECK(r.converged);
}

// ── Backend selector semantics ───────────────────────────────────────────────

TEST_CASE("ACOPFSolverBackend selection on case30", "[opf][backend]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case30.m"));

  double dispatch_cost = 0.0;
  double opf_cost = 0.0;

  SECTION("EconomicDispatch backend uses the fast merit-order + PF path") {
    opf::ACOPFOptions opt;
    opt.ac_solver_backend = opf::ACOPFSolverBackend::EconomicDispatch;
    const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
    CHECK(r.converged);
    CHECK(r.status.find("economic-dispatch") != std::string::npos);
    dispatch_cost = r.objective;
  }

  SECTION("ParityIPM backend runs the full-space nonlinear OPF") {
    opf::ACOPFOptions opt;
    opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
    const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
    CHECK(r.converged);
    CHECK(r.solver_path == opf::OPFSolverPath::ParityIPM);
    opf_cost = r.objective;
    CHECK(opf_cost > 0.0);
  }

  SECTION("Ipopt backend degrades gracefully when embedded Ipopt is unavailable") {
    // The embedded Ipopt/MUMPS is gated off by default; the Ipopt backend must
    // therefore fall back to the native parity IPM rather than abort.
    opf::ACOPFOptions opt;
    opt.ac_solver_backend = opf::ACOPFSolverBackend::Ipopt;
    const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);
    CHECK(r.converged);
    CHECK(r.solver_path == opf::OPFSolverPath::ParityIPM);
  }

  (void)dispatch_cost;
  (void)opf_cost;
}

TEST_CASE("Real AC OPF beats economic-dispatch cost on case30", "[opf][backend]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case30.m"));

  opf::ACOPFOptions ed;
  ed.ac_solver_backend = opf::ACOPFSolverBackend::EconomicDispatch;
  const opf::ACOPFResult r_ed = opf::solve_ac_opf(sys, ed);

  opf::ACOPFOptions pi;
  pi.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  const opf::ACOPFResult r_pi = opf::solve_ac_opf(sys, pi);

  REQUIRE(r_ed.converged);
  REQUIRE(r_pi.converged);
  // The full nonlinear OPF should not cost more than the suboptimal
  // economic-dispatch + PF shortcut (small tolerance for model differences).
  CHECK(r_pi.objective <= r_ed.objective * 1.001 + 1.0);
}

TEST_CASE("Parity IPM dense KKT backend solves small MATPOWER OPF cases",
          "[opf][backend][windows]") {
  ScopedEnvVar force_dense("HACDCPF_OPF_LINEAR_SOLVER", "dense");

  for (const std::string case_name : {"case9.m", "case30.m"}) {
    HybridPowerSystem sys = io::parse_matpower(data_path(case_name));

    opf::ACOPFOptions opt;
    opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
    opt.max_inner_iterations = 400;
    const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

    INFO("case=" << case_name << " status=" << r.status
                 << " backend=" << r.profiling.linear_solver_backend);
    CHECK(r.converged);
    CHECK(r.objective > 0.0);
    CHECK(r.profiling.linear_solver_backend.find("dense_lu") != std::string::npos);
  }
}

// ── Large-scale sparse-KKT robustness ────────────────────────────────────────

TEST_CASE("Large OPF takes the sparse KKT path and converges (case2869pegase)",
          "[opf][sparse][large]") {
  HybridPowerSystem sys = io::parse_matpower(data_path("case2869pegase.m"));
  REQUIRE(sys.ac.buses.size() > 2500);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 200;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  INFO("status=" << r.status << " iterations=" << r.iterations
       << " objective=" << r.objective
       << " primal=" << r.max_constraint_violation
       << " dual=" << r.max_stationarity
       << " accepted=" << r.profiling.accepted_steps
       << " rejected=" << r.profiling.rejected_steps
       << " backend=" << r.profiling.linear_solver_backend);
  CHECK(r.converged);
  CHECK(std::isfinite(r.objective));
  CHECK(r.iterations < 100);
  // A KKT with far more than 1500 unknowns must use the sparse factorization.
  CHECK(r.profiling.linear_solver_backend.find("sparse") != std::string::npos);
}

TEST_CASE("Sparse KKT keeps a 6500-bus RTE OPF finite (no factorization blow-up)",
          "[opf][sparse][large]") {
  // case6515rte previously diverged to NaN with a "KKT factorization failed"
  // status under Eigen's SparseLU; the UMFPACK-first sparse backend keeps the
  // iterates finite.  This guards that robustness win — convergence on this
  // hard RTE case is not expected within the iteration budget.
  HybridPowerSystem sys = io::parse_matpower(data_path("case6515rte.m"));
  REQUIRE(sys.ac.buses.size() > 6000);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 40;
  opt.max_outer_iterations = 1;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  CHECK(std::isfinite(r.objective));
  CHECK(r.objective < 1e30);  // not the pre-fix divergence (objective ~ -3e48)
  CHECK(r.profiling.linear_solver_backend.find("sparse") != std::string::npos);
}
