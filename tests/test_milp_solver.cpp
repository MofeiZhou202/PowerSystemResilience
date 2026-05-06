/// test_milp_solver.cpp
/// Tests for MILP solving via the engine — exercises every registered MILP solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

// Return SolveOptions that hard-pins a specific solver.
static SolveOptions solver_opts(const std::string& name) {
  SolveOptions opts;
  opts.preferred_solver = name;
  opts.allow_fallback   = false;
  return opts;
}

// Helper: return SolveOptions that prefers Gurobi (if available) for MILP.
static SolveOptions milp_options(const SolverEngine& eng) {
  SolveOptions opts;
  auto solvers = eng.list_solvers(ProblemClass::MILP);
  for (auto& s : solvers) {
    if (s.find("urobi") != std::string::npos) {
      opts.preferred_solver = s;
      opts.allow_fallback = true;
      return opts;
    }
  }
  // Fallback to whatever is first
  opts.allow_fallback = true;
  return opts;
}

// ─── Simple binary knapsack ────────────────────────────────────────────────
// max  5x1 + 4x2 + 3x3
// s.t. 2x1 + 3x2 + x3 <= 5
//      x1,x2,x3 ∈ {0,1}
// Optimal: x1=1,x2=1,x3=0, obj=9
TEST_CASE("MILP: binary knapsack 3 items", "[milp]") {
  SolverEngine eng;

  MIPModel mip;
  mip.linear_part.sense = Sense::Maximize;
  mip.linear_part.c.resize(3);
  mip.linear_part.c << 5.0, 4.0, 3.0;

  Eigen::SparseMatrix<double> A(1, 3);
  A.insert(0, 0) = 2.0;
  A.insert(0, 1) = 3.0;
  A.insert(0, 2) = 1.0;
  A.makeCompressed();
  mip.linear_part.A = A;
  mip.linear_part.b.resize(1);
  mip.linear_part.b << 5.0;

  for (int i = 0; i < 3; ++i) {
    mip.linear_part.vars.push_back({VarType::Binary, 0.0, 1.0});
  }
  mip.binary_idx = {0, 1, 2};

  for (const auto& solver_name : eng.list_solvers(ProblemClass::MILP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_milp(mip, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        // Optimal: x1=1,x2=1,x3=0 → obj=9; constraint 2+3+0=5≤5
        CHECK(res.stats.objective == Approx(9.0).margin(1e-4));
        CHECK(res.x.size() == 3);
        CHECK(std::round(res.x[0]) == Approx(1.0).margin(0.01));
        CHECK(std::round(res.x[1]) == Approx(1.0).margin(0.01));
        CHECK(std::round(res.x[2]) == Approx(0.0).margin(0.01));
      }
    }
  }
}

// ─── Simple integer programming ───────────────────────────────────────────
// max  x + y
// s.t. 2x + y <= 14
//      x + 2y <= 14
//      x, y   >= 0,  integer
// MIP optimal: x=4,y=5 (or x=5,y=4), obj=9 (floor of LP opt 9.33)
TEST_CASE("MILP: integer 2-variable problem", "[milp]") {
  SolverEngine eng;

  MIPModel mip;
  mip.linear_part.sense = Sense::Maximize;
  mip.linear_part.c.resize(2);
  mip.linear_part.c << 1.0, 1.0;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 2.0; A.insert(0, 1) = 1.0;
  A.insert(1, 0) = 1.0; A.insert(1, 1) = 2.0;
  A.makeCompressed();
  mip.linear_part.A = A;
  mip.linear_part.b.resize(2);
  mip.linear_part.b << 14.0, 14.0;

  for (int i = 0; i < 2; ++i) {
    mip.linear_part.vars.push_back({VarType::Integer, 0.0, 1e20});
  }
  mip.integer_idx = {0, 1};

  for (const auto& solver_name : eng.list_solvers(ProblemClass::MILP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_milp(mip, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        CHECK(res.stats.objective == Approx(9.0).margin(1e-4));
        CHECK(res.x[0] + res.x[1] == Approx(9.0).margin(1e-4));
        // Both must be integer.
        CHECK(std::abs(res.x[0] - std::round(res.x[0])) < 0.01);
        CHECK(std::abs(res.x[1] - std::round(res.x[1])) < 0.01);
      }
    }
  }
}

// ─── MILP with equality constraints ───────────────────────────────────────
// min  x + 2y
// s.t. x + y  == 5    (equality)
//      x >= 0,  x integer
//      y >= 0,  y integer
// Optimal: x=5, y=0, obj=5
TEST_CASE("MILP: equality constraint with integers", "[milp]") {
  SolverEngine eng;

  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c.resize(2);
  mip.linear_part.c << 1.0, 2.0;

  mip.linear_part.A.resize(0, 2);
  mip.linear_part.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(1, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.makeCompressed();
  mip.linear_part.Aeq = Aeq;
  mip.linear_part.beq.resize(1);
  mip.linear_part.beq << 5.0;

  mip.linear_part.vars.push_back({VarType::Integer, 0.0, 1e20});
  mip.linear_part.vars.push_back({VarType::Integer, 0.0, 1e20});
  mip.integer_idx = {0, 1};

  for (const auto& solver_name : eng.list_solvers(ProblemClass::MILP)) {
    DYNAMIC_SECTION("solver=" << solver_name) {
      auto res = eng.solve_milp(mip, solver_opts(solver_name));
      CHECK(res.stats.success);
      if (res.stats.success) {
        CHECK(res.stats.objective == Approx(5.0).margin(1e-4));
        CHECK(res.x[0] == Approx(5.0).margin(1e-4));
        CHECK(res.x[1] == Approx(0.0).margin(1e-4));
      }
    }
  }
}

// ─── Warm start ────────────────────────────────────────────────────────────
TEST_CASE("MILP: warm start is accepted and maintains solution quality", "[milp]") {
  SolverEngine eng;

  // Binary knapsack from above.
  MIPModel mip;
  mip.linear_part.sense = Sense::Maximize;
  mip.linear_part.c.resize(3);
  mip.linear_part.c << 5.0, 4.0, 3.0;

  Eigen::SparseMatrix<double> A(1, 3);
  A.insert(0, 0) = 2.0; A.insert(0, 1) = 3.0; A.insert(0, 2) = 1.0;
  A.makeCompressed();
  mip.linear_part.A = A;
  mip.linear_part.b.resize(1);
  mip.linear_part.b << 5.0;
  for (int i = 0; i < 3; ++i) {
    mip.linear_part.vars.push_back({VarType::Binary, 0.0, 1.0});
  }
  mip.binary_idx = {0, 1, 2};

  // Provide warm start (a feasible but suboptimal solution).
  mip.initial_solution.resize(3);
  mip.initial_solution << 1.0, 0.0, 1.0;  // obj=8, not optimal

  auto res = eng.solve_milp(mip, milp_options(eng));
  REQUIRE(res.stats.success);
  // Solver should still find the global optimum (obj=9)
  CHECK(res.stats.objective == Approx(9.0).margin(1e-4));
}

#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

