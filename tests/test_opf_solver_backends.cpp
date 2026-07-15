// tests/test_opf_solver_backends.cpp
// =====================================================================
// Coverage for the AC OPF solver-backend selector (ACOPFSolverBackend),
// the gradient-based objective-scaling robustness improvement, and
// convergence on the internal hybrid AC/DC cases (case300_acdc /
// case2000_acdc).  See src/optimal_power_flow/ac_opf.cpp and
// src/optimal_power_flow/parity_ipm.cpp.
// =====================================================================

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <string>

#include "hacdcpf/io/case_builders.hpp"
#include "hacdcpf/io/matpower_parser.hpp"
#include "hacdcpf/optimal_power_flow/ac_opf_solver.hpp"
#include "hacdcpf/optimal_power_flow/formulation.hpp"
#include "hacdcpf/optimal_power_flow/opf_options.hpp"
#include "hacdcpf/optimal_power_flow/opf_result.hpp"

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
  CHECK(r.converged);
  CHECK(r.solver_path == opf::OPFSolverPath::ParityIPM);
  CHECK(r.objective > 0.0);
  CHECK_FALSE(r.vm.empty());
  CHECK(r.profiling.accepted_steps > 0);
  CHECK(r.profiling.accepted_steps <= r.iterations);
  CHECK(r.profiling.rejected_steps > 0);
}

TEST_CASE("case300_acdc forwards iteration and tolerance options to Ipopt",
          "[opf][acdc][ipopt][options]") {
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
  CHECK(warm.profiling.initial_primal_residual <=
        cold.profiling.initial_primal_residual + 1e-10);
  CHECK(warm.iterations <= cold.iterations);
  CHECK(std::abs(warm.objective - cold.objective) <=
        1e-5 * std::max(1.0, std::abs(cold.objective)));
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
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

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
       << " iterations=" << r.iterations
       << " elapsed_sec=" << elapsed_sec
       << " factorization_calls=" << r.profiling.factorization_calls
       << " linear_solve_calls=" << r.profiling.linear_solve_calls
       << " primal=" << r.max_constraint_violation
       << " dual=" << r.max_stationarity
       << " backend=" << r.profiling.linear_solver_backend);
  CHECK(r.converged);
  CHECK(r.objective > 0.0);
  CHECK(r.max_constraint_violation < 1e-5);
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

  CHECK(r.converged);
  CHECK(std::isfinite(r.objective));
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
