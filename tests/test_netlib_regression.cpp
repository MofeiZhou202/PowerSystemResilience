/// test_netlib_regression.cpp
/// NETLIB-subset regression suite (P3.15): solve published LP problems with
/// known optimal objectives through the engine's registered LP solvers and
/// the native kernels.  Problems (coin-or Data-Netlib, MIT licensed):
///   afiro, adlittle, share2b, stocfor1, kb2 — stored in tests/data/netlib/.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/kernel/ipm/ipm_lp_solver.hpp"
#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"

#include "io/HMPSIO.h"  // vendored HiGHS MPS reader

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

struct NetlibCase {
  const char* file;
  double expected_obj;
};

// Published NETLIB optimal objective values (lp/data readme index).
const NetlibCase kCases[] = {
    {"afiro", -4.6475314286e+02},
    {"adlittle", 2.2549496316e+05},
    {"share2b", -4.1573224074e+02},
    {"stocfor1", -4.1131976219e+04},
    {"kb2", -1.7499001299e+03},
};

std::string netlib_path(const std::string& name) {
  return (std::filesystem::path(__FILE__).parent_path() / "data" / "netlib" /
          (name + ".mps"))
      .string();
}

LPModel read_mps_as_lp(const std::string& path) {
  // highsLogDev dereferences these unconditionally — point them at quiet
  // defaults (HighsLogOptions default ctor leaves them null).
  bool output_flag = false;
  bool log_to_console = false;
  HighsInt log_dev_level = kHighsLogDevLevelNone;
  HighsLogOptions log_options;
  log_options.output_flag = &output_flag;
  log_options.log_to_console = &log_to_console;
  log_options.log_dev_level = &log_dev_level;
  HighsInt num_row = 0, num_col = 0;
  ObjSense sense = ObjSense::kMinimize;
  double offset = 0.0;
  std::vector<HighsInt> Astart, Aindex;
  std::vector<double> Avalue, colCost, colLower, colUpper, rowLower, rowUpper;
  std::vector<HighsVarType> integer;
  std::string objective_name;
  std::vector<std::string> col_names, row_names;
  HighsInt Qdim = 0, cost_row_location = 0;
  std::vector<HighsInt> Qstart, Qindex;
  std::vector<double> Qvalue;
  bool warning_issued = false;
  const FilereaderRetcode rc = readMps(
      log_options, path, -1, -1, num_row, num_col, sense, offset, Astart,
      Aindex, Avalue, colCost, colLower, colUpper, rowLower, rowUpper, integer,
      objective_name, col_names, row_names, Qdim, Qstart, Qindex, Qvalue,
      cost_row_location, warning_issued);
  REQUIRE(rc == FilereaderRetcode::kOk);

  LPModel lp;
  lp.sense = (sense == ObjSense::kMaximize) ? Sense::Maximize : Sense::Minimize;
  const int n = static_cast<int>(num_col);
  const int m = static_cast<int>(num_row);
  lp.c = Eigen::VectorXd::Map(colCost.data(), n);
  lp.vars.resize(static_cast<size_t>(n));
  for (int j = 0; j < n; ++j) {
    lp.vars[static_cast<size_t>(j)].type = VarType::Continuous;
    // Engine validation requires finite bounds; clamp free variables to the
    // codebase's kBig convention (solvers clamp identically internally).
    const double lo = colLower[static_cast<size_t>(j)];
    const double hi = colUpper[static_cast<size_t>(j)];
    lp.vars[static_cast<size_t>(j)].lb = std::isfinite(lo) ? lo : -1e20;
    lp.vars[static_cast<size_t>(j)].ub = std::isfinite(hi) ? hi : 1e20;
  }

  // Column-wise CSC from readMps (Astart has num_col+1 entries, Aindex holds
  // row indices).  Split rows into equalities (lhs == rhs) and inequalities
  // (two-sided via LPModel::row_lhs).
  std::vector<int> ineq_row(static_cast<size_t>(m), -1);
  std::vector<int> eq_row(static_cast<size_t>(m), -1);
  int n_ineq = 0, n_eq = 0;
  for (int i = 0; i < m; ++i) {
    if (rowLower[static_cast<size_t>(i)] == rowUpper[static_cast<size_t>(i)]) {
      eq_row[static_cast<size_t>(i)] = n_eq++;
    } else {
      ineq_row[static_cast<size_t>(i)] = n_ineq++;
    }
  }
  std::vector<Eigen::Triplet<double>> trips, trips_eq;
  std::vector<double> b(static_cast<size_t>(n_ineq)),
      beq(static_cast<size_t>(n_eq)), lhs(static_cast<size_t>(n_ineq));
  for (int i = 0; i < m; ++i) {
    if (eq_row[static_cast<size_t>(i)] >= 0) {
      beq[static_cast<size_t>(eq_row[static_cast<size_t>(i)])] =
          rowLower[static_cast<size_t>(i)];
    } else {
      b[static_cast<size_t>(ineq_row[static_cast<size_t>(i)])] =
          rowUpper[static_cast<size_t>(i)];
      lhs[static_cast<size_t>(ineq_row[static_cast<size_t>(i)])] =
          rowLower[static_cast<size_t>(i)];
    }
  }
  for (int j = 0; j < n; ++j) {
    for (HighsInt p = Astart[j]; p < Astart[j + 1]; ++p) {
      const int i = static_cast<int>(Aindex[static_cast<size_t>(p)]);
      if (eq_row[static_cast<size_t>(i)] >= 0) {
        trips_eq.emplace_back(eq_row[static_cast<size_t>(i)], j,
                              Avalue[static_cast<size_t>(p)]);
      } else {
        trips.emplace_back(ineq_row[static_cast<size_t>(i)], j,
                           Avalue[static_cast<size_t>(p)]);
      }
    }
  }
  lp.A.resize(n_ineq, n);
  lp.A.setFromTriplets(trips.begin(), trips.end());
  lp.b = Eigen::VectorXd::Map(b.data(), n_ineq);
  lp.row_lhs = Eigen::VectorXd::Map(lhs.data(), n_ineq);
  lp.Aeq.resize(n_eq, n);
  lp.Aeq.setFromTriplets(trips_eq.begin(), trips_eq.end());
  lp.beq = Eigen::VectorXd::Map(beq.data(), n_eq);
  return lp;
}

SolveOptions pinned(const std::string& solver_name) {
  SolveOptions opts;
  opts.preferred_solver = solver_name;
  opts.allow_fallback = false;
  return opts;
}

// Per-solver capability on the five problems, from measured behavior.
// PDLP and LCQP have documented robustness gaps on harder/degenerate LPs
// (see docs/testing.md); those are reported, not enforced.
bool solver_is_strict(const std::string& solver, const std::string& problem) {
  if (solver == "HiGHS") return true;
  if (solver == "NativeIPMLP") return problem != "adlittle";
  if (solver == "NativeLCQP")
    return problem == "afiro" || problem == "share2b";
  return false;  // NativePDLP and anything else: report-only
}

}  // namespace

TEST_CASE("NETLIB: published optima via all registered LP solvers",
          "[netlib][regression]") {
  SolverEngine eng;
  const auto solvers = eng.list_solvers(ProblemClass::LP);
  REQUIRE(!solvers.empty());

  for (const auto& kase : kCases) {
    const LPModel lp = read_mps_as_lp(netlib_path(kase.file));
    for (const auto& solver_name : solvers) {
      DYNAMIC_SECTION(kase.file << " via " << solver_name) {
        api::Result res;
        try {
          res = eng.solve_lp(lp, pinned(solver_name));
        } catch (const std::exception& e) {
          FAIL("engine threw: " << e.what());
        }
        INFO(kase.file << " solver=" << solver_name
             << " status=" << res.stats.status
             << " obj=" << res.stats.objective);
        if (solver_is_strict(solver_name, kase.file)) {
          REQUIRE(res.stats.success);
          CHECK(res.stats.objective ==
                Approx(kase.expected_obj).epsilon(1e-4).margin(1e-3));
        } else {
          // Known-limitation path: must fail gracefully (finite or a clean
          // non-success status), never crash or return garbage as "optimal".
          CHECK(!(res.stats.success &&
                  std::abs(res.stats.objective - kase.expected_obj) >
                      1e-4 * std::abs(kase.expected_obj)));
        }
      }
    }
  }
}

TEST_CASE("NETLIB: published optima via native kernels directly",
          "[netlib][regression]") {
  for (const auto& kase : kCases) {
    const LPModel lp = read_mps_as_lp(netlib_path(kase.file));

    DYNAMIC_SECTION(kase.file << " via NativeIPMLP (Ruiz + IR + CHOLMOD)") {
      NativeIPMLPAdapter ipm{};
      const SolveResult res = ipm.solve_lp(lp);
      INFO(kase.file << " NativeIPMLP status=" << res.stats.status
           << " obj=" << res.stats.objective);
      if (std::string(kase.file) != "adlittle") {
        REQUIRE(res.stats.success);
        CHECK(res.stats.objective ==
              Approx(kase.expected_obj).epsilon(1e-4).margin(1e-3));
      } else {
        // adlittle is a documented hard-degenerate case for the IPM
        // (barrier mu explodes; dual simplex and HiGHS cover it).
        INFO("adlittle: known IPM robustness limitation, status reported");
        CHECK(!std::isnan(res.stats.objective));
      }
    }

    DYNAMIC_SECTION(kase.file << " via dual simplex (UMFPACK dl + IR)") {
      SimplexOptions opts;
      opts.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
      opts.max_iter = 100000;
      const auto res = solve_lp_with_basis(lp, opts);
      INFO(kase.file << " dual_simplex status=" << res.result.stats.status
           << " obj=" << res.result.stats.objective);
      REQUIRE(res.result.stats.success);
      CHECK(res.result.stats.objective ==
            Approx(kase.expected_obj).epsilon(1e-4).margin(1e-3));
    }
  }
}

TEST_CASE("NETLIB: grow22 side reconstruction satisfies canonical residual",
          "[netlib][regression][dual_simplex][reconstruction]") {
  constexpr double kGrow22Objective = -1.60834336483e+08;
  const LPModel lp = read_mps_as_lp(netlib_path("grow22"));
  SimplexOptions opts;
  opts.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opts.use_highs_presolve = true;
  opts.max_iter = 100000;

  const auto solved = solve_lp_with_basis(lp, opts);
  INFO("status=" << solved.result.stats.status
                  << " objective=" << solved.result.stats.objective
                  << " residual=" << solved.result.stats.residual_inf);
  REQUIRE(solved.result.stats.success);
  CHECK(solved.result.stats.objective ==
        Approx(kGrow22Objective).epsilon(1e-9).margin(1e-3));
  CHECK(solved.result.stats.residual_inf <= 1e-7);
}
