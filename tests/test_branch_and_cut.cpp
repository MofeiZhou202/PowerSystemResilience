/// test_branch_and_cut.cpp
/// Integration tests for native branch-and-cut internals that need
/// failure-injection seams (env hooks) rather than the public engine API.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <cstdlib>
#include <algorithm>
#include <optional>
#include <string>
#include <thread>

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/detail/bc_conformance_trace.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_objective_propagation.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

/// Sets an environment variable for the lifetime of the guard, restoring
/// "unset" on destruction (the seams below are opt-in flags, never set in a
/// normal environment).
struct EnvVarGuard {
  std::string name;
  std::optional<std::string> previous;
  EnvVarGuard(const char* n, const char* v) : name(n) {
    if (const char* old = ::getenv(n)) previous = old;
    ::setenv(n, v, 1);
  }
  ~EnvVarGuard() {
    if (previous) {
      ::setenv(name.c_str(), previous->c_str(), 1);
    } else {
      ::unsetenv(name.c_str());
    }
  }
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

TEST_CASE("B&C: environment snapshot is immutable and shared across workers",
          "[bc][environment][reproducibility]") {
  EnvVarGuard guard("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "before");
  const auto snapshot = capture_bc_env_options();
  REQUIRE(snapshot);
  REQUIRE(snapshot->value("MIPSOLVERS_TEST_SNAPSHOT_VALUE") != nullptr);
  CHECK(std::string(snapshot->value("MIPSOLVERS_TEST_SNAPSHOT_VALUE")) ==
        "before");

  ::setenv("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "after", 1);
  ScopedBcEnvOptions scope(snapshot);
  CHECK(std::string(bc_env_options().value(
            "MIPSOLVERS_TEST_SNAPSHOT_VALUE")) == "before");

  std::string worker_value;
  std::thread worker([snapshot, &worker_value] {
    ScopedBcEnvOptions worker_scope(snapshot);
    const char* value =
        bc_env_options().value("MIPSOLVERS_TEST_SNAPSHOT_VALUE");
    worker_value = value == nullptr ? "" : value;
  });
  worker.join();
  CHECK(worker_value == "before");
}

TEST_CASE("B&C: result records the effective solve environment",
          "[bc][environment][provenance]") {
  EnvVarGuard guard("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "recorded");
  BCOptions opt;
  opt.num_threads = 1;
  const BCResult result = solve_milp_bc(make_knapsack_10(), opt);
  REQUIRE(result.stats.success);
  CHECK(std::find(result.effective_environment.begin(),
                  result.effective_environment.end(),
                  "MIPSOLVERS_TEST_SNAPSHOT_VALUE=recorded") !=
        result.effective_environment.end());

  ::setenv("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "next-solve", 1);
  const BCResult next_result = solve_milp_bc(make_knapsack_10(), opt);
  REQUIRE(next_result.stats.success);
  CHECK(std::find(next_result.effective_environment.begin(),
                  next_result.effective_environment.end(),
                  "MIPSOLVERS_TEST_SNAPSHOT_VALUE=next-solve") !=
        next_result.effective_environment.end());
}

TEST_CASE("B&C: perturbed fallback cannot certify the original node",
          "[bc][fallback][certificate]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(0, 1);
  lp.b.resize(0);
  lp.Aeq.resize(2, 1);
  lp.beq.resize(2);
  // L1's deterministic perturbation changes the second (odd) RHS from 1e-4
  // to zero. The perturbed LP is feasible, while the original zero=1e-4 row
  // remains infeasible and therefore must never yield a successful node bound.
  lp.beq << 0.0, 1e-4;
  lp.vars.push_back({VarType::Continuous, 0.0, 1.0});

  auto sf = build_standard_form_lp(lp);
  SimplexOptions simplex_opts;
  simplex_opts.allow_cold_start = true;
  simplex_opts.max_iter = 1000;
  detail::FallbackConfig fallback_config;
  fallback_config.l1_max_retries = 1;
  fallback_config.l1_perturbation = 1e-4;
  fallback_config.l3_max_retries = 0;
  detail::FallbackLogger logger(std::chrono::steady_clock::now());
  detail::FallbackManager fallback(fallback_config, logger, simplex_opts);

  bool deferred = false;
  const SimplexResult result = fallback.handle_failure(
      sf, nullptr, detail::LPFailureType::NumericalFailure, 0, 0, 1, deferred);
  CHECK(detail::classify_lp_result(result, 1) !=
        detail::LPFailureType::Success);
  CHECK(deferred);
  CHECK(logger.summary().l1_successes == 0);
}

TEST_CASE("B&C: indexed objective events propagate chained literals to a fixed point",
          "[bc][objective_propagation][index]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(3);
  lp.c << 0.0, 1.0, 1.0;
  lp.A.resize(0, 3);
  lp.b.resize(0);
  lp.Aeq.resize(0, 3);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0},
             {VarType::Continuous, 0.0, 10.0}};

  detail::ObjectivePropagationState state;
  state.setup(lp, nullptr, nullptr, nullptr,
              detail::ObjectivePropagationState::BuildPolicy::none());
  state.implied_events = {
      {1, 1.0, 1.0, true, {{0, true}}},
      {2, 3.0, 1.0, true, {{0, true}, {1, true}}},
  };
  state.implied_events_by_literal.assign(6, {});
  state.implied_events_by_literal[1] = {0, 1};
  state.implied_events_by_literal[3] = {1};

  Eigen::VectorXd lb(3), ub(3);
  lb << 1.0, 0.0, 0.0;
  ub << 1.0, 1.0, 10.0;
  int tightened = 0;
  int pruned = 0;
  REQUIRE(state.propagate(lp, lb, ub, 100.0, 1e-6, 0, 0, nullptr,
                          tightened, pruned));
  CHECK(pruned == 0);
  CHECK(tightened == 2);
  CHECK(lb[1] == Approx(1.0));
  CHECK(lb[2] == Approx(3.0));
}

TEST_CASE("B&C: objective-event propagation falls back when its index is absent",
          "[bc][objective_propagation][index]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 0.0, 1.0;
  lp.A.resize(0, 2);
  lp.b.resize(0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Continuous, 0.0, 10.0}};

  detail::ObjectivePropagationState state;
  state.setup(lp, nullptr, nullptr, nullptr,
              detail::ObjectivePropagationState::BuildPolicy::none());
  state.implied_events = {{1, 2.0, 1.0, true, {{0, true}}}};
  state.implied_events_by_literal.clear();

  Eigen::VectorXd lb(2), ub(2);
  lb << 1.0, 0.0;
  ub << 1.0, 10.0;
  int tightened = 0;
  int pruned = 0;
  REQUIRE(state.propagate(lp, lb, ub, 100.0, 1e-6, 0, 0, nullptr,
                          tightened, pruned));
  CHECK(pruned == 0);
  CHECK(tightened == 1);
  CHECK(lb[1] == Approx(2.0));
}

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

TEST_CASE("B&C: short global limits do not collapse the root budget",
          "[bc][presolve_fallback][time_limit]") {
  CHECK(detail::bc_root_lp_budget_sec(3.0, 3.0, 5.0, 0.5,
                                      /*fallback_available=*/true,
                                      /*short_budget_ipm_root=*/false) ==
        Approx(3.0));

  // Once the minimum fallback balance is reachable, reserve it explicitly.
  CHECK(detail::bc_root_lp_budget_sec(30.0, 30.0, 5.0, 0.5,
                                      /*fallback_available=*/true,
                                      /*short_budget_ipm_root=*/false) ==
        Approx(15.0));

  // No fallback path means no reserve, regardless of its configured size.
  CHECK(detail::bc_root_lp_budget_sec(3.0, 3.0, 1e9, 0.5,
                                      /*fallback_available=*/false,
                                      /*short_budget_ipm_root=*/false) ==
        Approx(3.0));
}

TEST_CASE("B&C: iteration limits are not global wall-clock deadlines",
          "[bc][deadline][regression]") {
  SimplexResult iteration_limit;
  iteration_limit.result.stats.status =
      "Native dual simplex: dual simplex iteration limit";
  CHECK(detail::classify_lp_result(iteration_limit, 0) ==
        detail::LPFailureType::Timeout);
  CHECK_FALSE(detail::lp_wall_time_limit_reached(iteration_limit));

  SimplexResult wall_limit;
  wall_limit.result.stats.status =
      "Time limit: dual simplex wall-clock deadline";
  CHECK(detail::classify_lp_result(wall_limit, 0) ==
        detail::LPFailureType::NumericalFailure);
  CHECK(detail::lp_wall_time_limit_reached(wall_limit));
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

TEST_CASE("B&C: reduced-cost fixing requires the matching active bound side",
          "[bc][reduced_cost][regression]") {
  std::vector<VariableMeta> vars{
      {VarType::Binary, 0.0, 1.0},
      {VarType::Binary, 0.0, 1.0}};
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd ub = Eigen::VectorXd::Ones(2);
  Eigen::VectorXd x(2);
  x << 0.5, 0.5;
  // reduced_cost_fixing receives scaled maximization reduced costs, so these
  // map to minimization col duals +20 and -20 respectively.
  Eigen::VectorXd reduced_costs(2);
  reduced_costs << -20.0, 20.0;
  const std::vector<int> no_basic_columns;

  CHECK(detail::reduced_cost_fixing(
            vars, x, reduced_costs, no_basic_columns,
            /*node_bound=*/0.0, /*incumbent_obj=*/5.0, /*int_tol=*/1e-6,
            lb, ub) == 0);
  CHECK(lb[0] == Approx(0.0));
  CHECK(ub[0] == Approx(1.0));
  CHECK(lb[1] == Approx(0.0));
  CHECK(ub[1] == Approx(1.0));

  x << 0.0, 1.0;
  CHECK(detail::reduced_cost_fixing(
            vars, x, reduced_costs, no_basic_columns,
            /*node_bound=*/0.0, /*incumbent_obj=*/5.0, /*int_tol=*/1e-6,
            lb, ub) == 2);
  CHECK(ub[0] == Approx(0.0));
  CHECK(lb[1] == Approx(1.0));
}
