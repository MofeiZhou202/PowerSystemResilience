/// test_branch_and_cut.cpp
/// Integration tests for native branch-and-cut internals that need
/// failure-injection seams (env hooks) rather than the public engine API.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <cstdlib>
#include <string>

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/detail/bc_conformance_trace.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

/// Sets an environment variable for the lifetime of the guard, restoring
/// "unset" on destruction (the seams below are opt-in flags, never set in a
/// normal environment).
struct EnvVarGuard {
  std::string name;
  EnvVarGuard(const char* n, const char* v) : name(n) { ::setenv(n, v, 1); }
  ~EnvVarGuard() { ::unsetenv(name.c_str()); }
};

// 10-item 0-1 knapsack (same instance as test_milp_solver.cpp):
//   maximise  10x0 + 9x1 + 8x2 + 7x3 + 6x4 + 5x5 + 4x6 + 3x7 + 7x8 + 2x9
//   s.t.       5x0 + 4x1 + 3x2 + 3x3 + 3x4 + 2x5 + 2x6 + 2x7 + 4x8 + 1x9 <= 14
MIPModel make_knapsack_10() {
  constexpr int N = 10;
  const double vals[N]    = {10.0, 9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 7.0, 2.0};
  const double weights[N] = { 5.0, 4.0, 3.0, 3.0, 3.0, 2.0, 2.0, 2.0, 4.0, 1.0};

  MIPModel mip;
  mip.linear_part.sense = Sense::Maximize;
  mip.linear_part.c.resize(N);
  for (int i = 0; i < N; ++i) mip.linear_part.c[i] = vals[i];

  Eigen::SparseMatrix<double> A(1, N);
  for (int i = 0; i < N; ++i) A.insert(0, i) = weights[i];
  A.makeCompressed();
  mip.linear_part.A = A;
  mip.linear_part.b.resize(1);
  mip.linear_part.b << 14.0;
  mip.linear_part.Aeq = Eigen::SparseMatrix<double>(0, N);
  mip.linear_part.beq.resize(0);

  for (int i = 0; i < N; ++i)
    mip.linear_part.vars.push_back({VarType::Binary, 0.0, 1.0});
  for (int i = 0; i < N; ++i) mip.binary_idx.push_back(i);

  return mip;
}

}  // namespace

// ─── P0(a): unpresolved-root fallback ───────────────────────────────────────
//
// MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL=1 marks the root relaxation of the
// *presolved* attempt as failed (the retry runs with presolve off, so the
// seam cannot re-fire), which makes the fallback deterministically testable.

TEST_CASE("B&C: root failure on presolved model retries without presolve",
          "[bc][presolve_fallback]") {
  MIPModel mip = make_knapsack_10();

  // Reference optimum from an unperturbed solve.
  BCOptions ref_opt;
  auto ref = solve_milp_bc(mip, ref_opt);
  REQUIRE(ref.stats.success);

  EnvVarGuard seam("MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL", "1");
  BCOptions opt;
  opt.use_papilo_presolve = true;   // seam only fires on the presolved attempt
  opt.root_presolve_fallback = true;
  auto res = solve_milp_bc(mip, opt);

  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(ref.stats.objective).margin(1e-6));
  CHECK(res.bc_stats.presolve_fallback_attempts == 1);
}

TEST_CASE("B&C: fallback disabled keeps the root-failure status",
          "[bc][presolve_fallback]") {
  MIPModel mip = make_knapsack_10();

  EnvVarGuard seam("MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL", "1");
  BCOptions opt;
  opt.use_papilo_presolve = true;
  opt.root_presolve_fallback = false;
  auto res = solve_milp_bc(mip, opt);

  CHECK(!res.stats.success);
  CHECK(res.stats.status == "Root relaxation failed");
  CHECK(res.bc_stats.presolve_fallback_attempts == 0);
}

TEST_CASE("B&C: fallback respects the minimum-remaining-time guard",
          "[bc][presolve_fallback]") {
  MIPModel mip = make_knapsack_10();

  EnvVarGuard seam("MIPSOLVERS_BC_TEST_FORCE_ROOT_FAIL", "1");
  BCOptions opt;
  opt.use_papilo_presolve = true;
  opt.root_presolve_fallback = true;
  opt.time_limit_sec = 30.0;
  // Remaining wall-clock (≤ 30 s) can never exceed this threshold, so the
  // retry must not be attempted even though the fallback is enabled.
  opt.root_presolve_fallback_min_remaining_sec = 1e9;
  auto res = solve_milp_bc(mip, opt);

  CHECK(!res.stats.success);
  CHECK(res.stats.status == "Root relaxation failed");
  CHECK(res.bc_stats.presolve_fallback_attempts == 0);
}

TEST_CASE("B&C: timed-out HiGHS presolve side state is not cached",
          "[bc][deadline]") {
  LPModel lp = make_knapsack_10().linear_part;
  // Make the cache signature unique to this contract test.
  lp.c[0] = 1234567.89;

  bool cache_hit = true;
  const auto& timed_out = detail::cached_highs_presolve_side_state(
      lp, 1e-12, &cache_hit);
  CHECK_FALSE(cache_hit);
  CHECK(timed_out.highs_status == "timeout");

  const auto& complete = detail::cached_highs_presolve_side_state(
      lp, 1.0, &cache_hit);
  CHECK_FALSE(cache_hit);
  CHECK(complete.highs_status != "timeout");

  (void)detail::cached_highs_presolve_side_state(lp, 1.0, &cache_hit);
  CHECK(cache_hit);
}
