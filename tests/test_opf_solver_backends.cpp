// tests/test_opf_solver_backends.cpp
// =====================================================================
// Coverage for the AC OPF solver-backend selector (ACOPFSolverBackend),
// the gradient-based objective-scaling robustness improvement, and
// convergence on the internal hybrid AC/DC cases (case300_acdc /
// case2000_acdc).  See src/optimal_power_flow/ac_opf.cpp and
// src/optimal_power_flow/parity_ipm.cpp.
// =====================================================================

#include <catch2/catch_test_macros.hpp>

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

  CHECK(r.converged);
  CHECK(r.solver_path == opf::OPFSolverPath::ParityIPM);
  CHECK(r.objective > 0.0);
  CHECK_FALSE(r.vm.empty());
}

TEST_CASE("AC OPF converges on internal hybrid case2000_acdc", "[opf][acdc]") {
  HybridPowerSystem sys = io::build_case2000_acdc();
  REQUIRE(sys.ac.buses.size() > 1000);

  opf::ACOPFOptions opt;
  opt.ac_solver_backend = opf::ACOPFSolverBackend::ParityIPM;
  opt.max_inner_iterations = 400;
  const opf::ACOPFResult r = opf::solve_ac_opf(sys, opt);

  // Gradient-based objective scaling is what lets a ~2000-bus hybrid system
  // converge in the native IPM (it stalled before the scaling fix).
  CHECK(r.converged);
  CHECK(r.objective > 0.0);
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
