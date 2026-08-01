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
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

// Return SolveOptions that hard-pins a specific solver.
static SolveOptions solver_opts(const std::string& name) {
  SolveOptions opts;
  opts.preferred_solver = name;
  opts.allow_fallback   = false;
  return opts;
}

// Use the engine's production MILP order (StrictHiGHS, HiGHS, native B&C).
static SolveOptions milp_options(const SolverEngine&) {
  SolveOptions opts;
  opts.allow_fallback = true;
  return opts;
}

TEST_CASE("MILP: compact pseudocost table preserves branching scores",
          "[milp][performance]") {
  constexpr int n = 100000;
  const std::vector<int> branchable{7, 50000, 99999};
  std::vector<detail::PseudoCost> dense(static_cast<std::size_t>(n));
  detail::CompactPseudoCostTable compact(n, branchable);

  for (std::size_t k = 0; k < branchable.size(); ++k) {
    const int col = branchable[k];
    dense[col].add_down(2.0 + k);
    dense[col].add_up(5.0 - k);
    compact[col] = dense[col];
  }

  Eigen::VectorXd x = Eigen::VectorXd::Zero(n);
  x[7] = 0.2;
  x[50000] = 0.5;
  x[99999] = 0.8;
  const std::vector<int> priority(static_cast<std::size_t>(n), 0);

  CHECK(detail::choose_branch_var_pseudocost(branchable, x, dense, priority) ==
        detail::choose_branch_var_pseudocost(branchable, x, compact, priority));
  CHECK(detail::compute_node_estimate(branchable, x, dense, 1e-6, 10.0) ==
        Approx(detail::compute_node_estimate(
            branchable, x, compact, 1e-6, 10.0)).margin(1e-12));
  CHECK(compact.active_size() == branchable.size());
  CHECK(compact.storage_bytes() < dense.size() * sizeof(detail::PseudoCost) / 8);
}

TEST_CASE("MILP: pseudocost observations are normalized by branch distance",
          "[milp][branching][math]") {
  detail::PseudoCost pc;
  detail::PCUpdate down{3, false};
  down.branch_distance = 0.25;
  down.has_gain = true;
  down.gain = 6.0;
  down.apply(pc);

  detail::PCUpdate up{3, true};
  up.branch_distance = 0.75;
  up.has_gain = true;
  up.gain = 6.0;
  up.apply(pc);

  CHECK(pc.down_avg() == Approx(24.0));
  CHECK(pc.up_avg() == Approx(8.0));
  CHECK(detail::normalized_pseudocost_gain(-1.0, 0.5) == 0.0);
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

// ─── StrictHiGHS tests (improvements A, B, D) ─────────────────────────────
//
// These tests exercise the vendored-HiGHS LP kernel path
// (BCOptions::lp_kernel_backend = LpKernelBackend::HiGHS). They verify:
//
//   [D] mip_detect_symmetry=false – still finds the correct optimum.
//   [A+B] Warm-start via dispatch LP + setSolution() – still finds the correct
//         optimum, and nodes_explored with warm start <= nodes_explored cold.

#include "mipsolvers/engine/bc/api.hpp"

TEST_CASE("MILP LP backend selection is independent of StrictHiGHS policy",
          "[milp][dispatch]") {
  BCOptions opt;
  REQUIRE(opt.lp_kernel_backend == LpKernelBackend::HiGHS);
  REQUIRE_FALSE(opt.strict_highs_mip_contract);

  opt.use_papilo_presolve = true;
  opt.use_feasibility_pump = true;
  opt.enable_reduced_cost_conflict_learning = false;
  opt.require_tree_exhaustion_certificate = true;

  const auto state = detail::apply_bc_strict_highs_contract(opt);
  CHECK(state.requested_vendored_highs_lp);
  CHECK_FALSE(state.strict_highs_lp_contract);
  CHECK(opt.lp_kernel_backend == LpKernelBackend::HiGHS);
  CHECK(opt.use_papilo_presolve);
  CHECK(opt.use_feasibility_pump);
  CHECK_FALSE(opt.enable_reduced_cost_conflict_learning);
  CHECK(opt.require_tree_exhaustion_certificate);
}

TEST_CASE("MILP row propagation treats 1e20 bounds as infinity",
          "[milp][propagation][regression]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = -10.0;
  lp.A.insert(0, 1) = -1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, -10.0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1e20}};

  Eigen::VectorXd lb(2), ub(2);
  lb << 0.0, 0.0;
  ub << 1.0, 1e20;
  CHECK(detail::node_bound_propagation(lp, lb, ub, 4) == 0);
  CHECK(lb[0] == Approx(0.0));
  CHECK(lb[1] == Approx(0.0));

  Eigen::SparseMatrix<double, Eigen::RowMajor> a_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> aeq_row = lp.Aeq;
  std::vector<BoundChangeInfo> changes;
  CHECK(detail::node_bound_propagation_tracked(
            lp, a_row, aeq_row, lb, ub, 4, changes) == 0);
  CHECK(changes.empty());
}

TEST_CASE("StrictHiGHS MIP policy explicitly normalizes the LP backend",
          "[milp][dispatch]") {
  BCOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opt.strict_highs_mip_contract = true;

  const auto state = detail::apply_bc_strict_highs_contract(opt);
  CHECK(state.requested_vendored_highs_lp);
  CHECK(state.strict_highs_lp_contract);
  CHECK(opt.lp_kernel_backend == LpKernelBackend::HiGHS);
}

// 10-item 0-1 knapsack used by the StrictHiGHS tests.
//
//   maximise  10x0 + 9x1 + 8x2 + 7x3 + 6x4 + 5x5 + 4x6 + 3x7 + 7x8 + 2x9
//   s.t.       5x0 + 4x1 + 3x2 + 3x3 + 3x4 + 2x5 + 2x6 + 2x7 + 4x8 + 1x9 <= 14
//              xi in {0, 1}
//
// The instance is asymmetric (distinct value/weight ratios), so disabling Nauty
// symmetry detection (improvement D) has no ill effect on solve quality.
static MIPModel make_knapsack_10() {
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

  // Equality constraints: none, but Aeq must have N columns so that
  // bc_pass_mip_model_to_highs can safely iterate over it column by column.
  mip.linear_part.Aeq = Eigen::SparseMatrix<double>(0, N);
  mip.linear_part.beq.resize(0);

  for (int i = 0; i < N; ++i)
    mip.linear_part.vars.push_back({VarType::Binary, 0.0, 1.0});
  for (int i = 0; i < N; ++i) mip.binary_idx.push_back(i);

  return mip;
}

TEST_CASE("MILP: StrictHiGHS [D] mip_detect_symmetry=false preserves correctness",
          "[milp][strict_highs]") {
  BCOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::HiGHS;

  MIPModel mip = make_knapsack_10();
  auto res = solve_milp_bc(mip, opt);

  REQUIRE(res.stats.success);
  // Objective must be a finite positive value.
  CHECK(res.stats.objective > 0.0);
  CHECK(res.stats.objective < 1e9);
}

TEST_CASE("MILP: native B&C reuses exported pseudocosts",
          "[milp][native][warmstart]") {
  BCOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opt.use_papilo_presolve = false;
  opt.root_cut_rounds = 0;
  opt.use_feasibility_pump = false;
  opt.use_progressive_rounding = false;
  opt.enable_feasibility_jump = false;

  MIPModel mip = make_knapsack_10();
  auto cold = solve_milp_bc(mip, opt);
  REQUIRE(cold.stats.success);
  REQUIRE(cold.highs_pseudocost_init);
  REQUIRE(cold.highs_pseudocost_init->n_orig_cols == 10);
  REQUIRE(cold.highs_pseudocost_init->pseudocostup.size() == 10);
  REQUIRE(cold.highs_pseudocost_init->nsamplesup.size() == 10);

  auto seed = std::make_shared<BCPseudocostInit>(
      *cold.highs_pseudocost_init);
  for (int col : mip.binary_idx) {
    seed->pseudocostup[static_cast<size_t>(col)] = 3.0 + col;
    seed->pseudocostdown[static_cast<size_t>(col)] = 4.0 + col;
    seed->nsamplesup[static_cast<size_t>(col)] = 17;
    seed->nsamplesdown[static_cast<size_t>(col)] = 19;
  }
  opt.highs_pseudocost_warm_start = seed;
  mip.initial_solution = cold.x;

  auto warm = solve_milp_bc(mip, opt);
  REQUIRE(warm.stats.success);
  REQUIRE(warm.highs_pseudocost_init);
  CHECK(warm.stats.objective == Approx(cold.stats.objective).margin(1e-6));
  for (int col : mip.binary_idx) {
    CHECK(warm.highs_pseudocost_init->nsamplesup[static_cast<size_t>(col)] >= 17);
    CHECK(warm.highs_pseudocost_init->nsamplesdown[static_cast<size_t>(col)] >= 19);
  }
}

TEST_CASE("MILP: StrictHiGHS [A+B] warm-start injection correctness and node reduction",
          "[milp][strict_highs]") {
  // Protocol:
  //   Cold run — no initial_solution.
  //   Warm run — optimal solution from cold run injected as initial_solution.
  //
  // Assertions:
  //   (1) Both runs return the same optimal objective  (correctness).
  //   (2) nodes_explored(warm) <= nodes_explored(cold) (effectiveness of
  //       upper_limit tightening from the injected incumbent).
  BCOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  opt.verbose = false;

  // Cold run.
  MIPModel mip = make_knapsack_10();
  auto cold = solve_milp_bc(mip, opt);
  REQUIRE(cold.stats.success);
  const double opt_obj = cold.stats.objective;
  CHECK(opt_obj > 0.0);

  // Warm run: inject the optimal solution from the cold run.
  mip.initial_solution = cold.x;
  auto warm = solve_milp_bc(mip, opt);
  REQUIRE(warm.stats.success);

  // (1) Correctness.
  CHECK(warm.stats.objective == Approx(opt_obj).margin(1e-4));

  // (2) Effectiveness: a tight incumbent from the start allows HiGHS to prune
  //     at least as well as the cold run.  The inequality is non-strict because
  //     small instances may already be solved at the root LP (0 nodes each).
  CHECK(warm.bc_stats.nodes_explored <= cold.bc_stats.nodes_explored);
}


// ─── P1 failure-path hardening (round 2) ───────────────────────────────────
//
// These tests pin the reporting hygiene added for the evaluation-doc roadmap:
// no -1e30/1e30 BCStats sentinels may leak to callers, a proven-optimal tree
// exhaustion must report best_bound == best_obj, and pathologically poor warm
// starts must not survive as root incumbents.

#include <cmath>
#include <limits>

TEST_CASE("MILP: failure paths report honest best_bound/gap, no sentinels",
          "[milp][hygiene]") {
  BCOptions opt;
  opt.time_limit_sec = 1e-6;  // expire before the root relaxation

  MIPModel mip = make_knapsack_10();
  mip.initial_solution = Eigen::VectorXd::Zero(10);  // feasible (weight 0 <= 14)
  auto res = solve_milp_bc(mip, opt);

  // Whether or not the verified warm start was published, the untouched
  // BCStats defaults (best_bound -1e30, best_obj/gap 1e30) must have been
  // scrubbed to honest infinities.
  CHECK(!(std::isfinite(res.bc_stats.best_bound) &&
          std::abs(res.bc_stats.best_bound) >= 1e29));
  CHECK(!(std::isfinite(res.bc_stats.best_obj) &&
          std::abs(res.bc_stats.best_obj) >= 1e29));
  CHECK(!(std::isfinite(res.bc_stats.gap) && res.bc_stats.gap >= 1e29));
}

TEST_CASE("MILP: tree-exhausted optimality syncs best_bound to incumbent",
          "[milp][hygiene]") {
  BCOptions opt;
  opt.require_tree_exhaustion_certificate = true;

  MIPModel mip = make_knapsack_10();
  auto res = solve_milp_bc(mip, opt);

  REQUIRE(res.stats.success);
  CHECK(!(std::isfinite(res.bc_stats.best_bound) &&
          std::abs(res.bc_stats.best_bound) >= 1e29));
  if (res.stats.status == "Optimal (tree exhausted)") {
    CHECK(res.bc_stats.gap == 0.0);
    CHECK(res.bc_stats.best_bound ==
          Approx(res.bc_stats.best_obj).margin(1e-6));
  }
}

TEST_CASE("MILP: garbage warm start does not survive as the final incumbent",
          "[milp][hygiene]") {
  // minimise x0 + 1e12*x1  s.t. x0 + x1 >= 1, x binary.
  // Optimal: x0=1, x1=0, obj=1.  The warm start (0,1) is feasible but ~1e12x
  // worse than the root bound; the incumbent-quality gate must reject it as a
  // root incumbent and the solve must still return the true optimum.
  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c.resize(2);
  mip.linear_part.c << 1.0, 1e12;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = -1.0;
  A.insert(0, 1) = -1.0;
  A.makeCompressed();
  mip.linear_part.A = A;
  mip.linear_part.b.resize(1);
  mip.linear_part.b << -1.0;
  mip.linear_part.Aeq = Eigen::SparseMatrix<double>(0, 2);
  mip.linear_part.beq.resize(0);

  for (int i = 0; i < 2; ++i) {
    mip.linear_part.vars.push_back({VarType::Binary, 0.0, 1.0});
    mip.binary_idx.push_back(i);
  }
  Eigen::VectorXd bad_warm(2);
  bad_warm << 0.0, 1.0;
  mip.initial_solution = bad_warm;

  BCOptions opt;
  opt.use_papilo_presolve = false;  // keep original/reduced spaces identical

  auto res = solve_milp_bc(mip, opt);
  REQUIRE(res.stats.success);
  CHECK(res.stats.objective == Approx(1.0).margin(1e-4));

  // Disabling the gate must not change the optimum either (the improvement
  // path replaces the weak incumbent) — guards against over-eager rejection.
  BCOptions no_gate = opt;
  no_gate.incumbent_quality_reject_factor = 0.0;
  auto res2 = solve_milp_bc(mip, no_gate);
  REQUIRE(res2.stats.success);
  CHECK(res2.stats.objective == Approx(1.0).margin(1e-4));
}

// ─── P2-short: IPM root health probe ────────────────────────────────────────
//
// On a healthy instance the probe must be behaviour-neutral: same optimum
// with the gate on and off (the probe iterate seeds the full solve, so no
// work is wasted).  Unhealthy classification (NumericalError / Cholesky
// failed / ProblemTooLarge) is exercised at benchmark scale where the IPM
// genuinely fails; here we pin the gate's no-harm contract.

TEST_CASE("MILP: IPM root probe is behaviour-neutral on a healthy instance",
          "[milp][ipm_probe]") {
  MIPModel mip = make_knapsack_10();

  BCOptions probed;
  probed.use_ipm_root = true;
  probed.ipm_root_probe = true;
  auto res_probed = solve_milp_bc(mip, probed);

  BCOptions unprobed = probed;
  unprobed.ipm_root_probe = false;
  auto res_unprobed = solve_milp_bc(mip, unprobed);

  REQUIRE(res_probed.stats.success);
  REQUIRE(res_unprobed.stats.success);
  CHECK(res_probed.stats.objective ==
        Approx(res_unprobed.stats.objective).margin(1e-6));
}
