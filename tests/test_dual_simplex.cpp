/// test_dual_simplex.cpp
/// Tests for the native dual-simplex LP kernel via solve_lp_with_basis().
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Sparse>

#include <limits>

#include "Highs.h"

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "mipsolvers/engine/kernel/linear_algebra/hfactor_backend.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual_core.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual/factor.hpp"
#include "../src/engine/kernel/lp_kernel/native_dual/state.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

double solve_with_highs(const LPModel& lp) {
  const int cols = static_cast<int>(lp.vars.size());
  const int ineq_rows = static_cast<int>(lp.A.rows());
  const int eq_rows = static_cast<int>(lp.Aeq.rows());
  const int rows = ineq_rows + eq_rows;
  std::vector<double> costs(static_cast<size_t>(cols));
  std::vector<double> lower(static_cast<size_t>(cols));
  std::vector<double> upper(static_cast<size_t>(cols));
  for (int j = 0; j < cols; ++j) {
    costs[static_cast<size_t>(j)] = lp.c[j];
    lower[static_cast<size_t>(j)] = lp.vars[static_cast<size_t>(j)].lb;
    upper[static_cast<size_t>(j)] = lp.vars[static_cast<size_t>(j)].ub;
  }
  std::vector<double> row_lower(static_cast<size_t>(rows), -kHighsInf);
  std::vector<double> row_upper(static_cast<size_t>(rows), kHighsInf);
  for (int i = 0; i < ineq_rows; ++i) {
    row_upper[static_cast<size_t>(i)] = lp.b[i];
  }
  for (int i = 0; i < eq_rows; ++i) {
    row_lower[static_cast<size_t>(ineq_rows + i)] = lp.beq[i];
    row_upper[static_cast<size_t>(ineq_rows + i)] = lp.beq[i];
  }
  std::vector<HighsInt> start(static_cast<size_t>(cols + 1));
  std::vector<HighsInt> index;
  std::vector<double> value;
  for (int j = 0; j < cols; ++j) {
    start[static_cast<size_t>(j)] = static_cast<HighsInt>(index.size());
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, j); it; ++it) {
      index.push_back(it.row());
      value.push_back(it.value());
    }
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, j); it; ++it) {
      index.push_back(ineq_rows + it.row());
      value.push_back(it.value());
    }
  }
  start[static_cast<size_t>(cols)] = static_cast<HighsInt>(index.size());

  Highs highs;
  highs.setOptionValue("output_flag", false);
  highs.setOptionValue("solver", "simplex");
  const auto pass = highs.passModel(
      cols, rows, static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMinimize), 0.0, costs.data(),
      lower.data(), upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data());
  REQUIRE(pass != HighsStatus::kError);
  REQUIRE(highs.run() == HighsStatus::kOk);
  REQUIRE(highs.getModelStatus() == HighsModelStatus::kOptimal);
  return highs.getInfo().objective_function_value;
}

LPModel make_degenerate_transport_lp(int side) {
  const int cols = side * side;
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(2 * side - 1, cols);
  lp.beq = Eigen::VectorXd::Ones(2 * side - 1);
  lp.vars.reserve(static_cast<size_t>(cols));
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(static_cast<size_t>(2 * cols));
  for (int source = 0; source < side; ++source) {
    for (int sink = 0; sink < side; ++sink) {
      const int col = source * side + sink;
      triplets.emplace_back(source, col, 1.0);
      if (sink < side - 1) triplets.emplace_back(side + sink, col, 1.0);
      lp.c[col] = source == sink ? 0.0
                                 : 1.0 + 1e-4 * ((source + 3 * sink) % side);
      // Keep the root optimum strictly inside the upper bound so every
      // positive transportation arc must be basic. Later bound reductions
      // therefore create a genuine primal violation in the inherited basis.
      lp.vars.push_back({VarType::Continuous, 0.0, 2.0});
    }
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();
  return lp;
}

SimplexOptions native_simplex_options() {
  SimplexOptions options;
  options.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  return options;
}

}  // namespace

TEST_CASE("LP kernel defaults to HiGHS", "[dual_simplex][dispatch]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(1);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 3.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 2.0, 3.0});

  const auto result = solve_lp_with_basis(lp, SimplexOptions{});
  REQUIRE(result.result.stats.success);
  CHECK(result.result.stats.solver_name == "VendoredHighsLpKernel");
  CHECK(result.result.stats.objective == Approx(2.0));
}

TEST_CASE("HiGHS infeasibility never falls through to native dual",
          "[dual_simplex][dispatch]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, -1.0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  const auto result = solve_lp_with_basis(lp, SimplexOptions{});
  CHECK_FALSE(result.result.stats.success);
  CHECK(result.result.stats.solver_name == "VendoredHighsLpKernel");
}

// ─── 2-variable LP solved via dual simplex ────────────────────────────────
// min -3x - 2y  s.t.  x+y <= 4,  x >= 0,  y >= 0
// Optimal: x=4, y=0, obj=-12
TEST_CASE("DualSimplex: 2-variable LP", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -3.0, -2.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 4.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  opts.max_iter = 500;

  auto res = solve_lp_with_basis(lp, opts);
  INFO("status=" << res.result.stats.status
                  << " iterations=" << res.result.stats.iterations);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(-12.0).margin(1e-5));
  CHECK(res.result.x[0] == Approx(4.0).margin(1e-5));
  CHECK(res.result.x[1] == Approx(0.0).margin(1e-5));
}

// ─── 3-variable LP ────────────────────────────────────────────────────────
// min  x1 + x2 + x3
// s.t. x1 + x2 >= 2
//      x2 + x3 >= 2
//      all >= 0
// Optimal: x1=0, x2=2, x3=0, obj=2
TEST_CASE("DualSimplex: 3-variable LP with >= constraints", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(3);
  lp.c << 1.0, 1.0, 1.0;

  // Convert x1+x2 >= 2 and x2+x3 >= 2 to -x <= -2
  Eigen::SparseMatrix<double> A(2, 3);
  A.insert(0, 0) = -1.0; A.insert(0, 1) = -1.0;
  A.insert(1, 1) = -1.0; A.insert(1, 2) = -1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << -2.0, -2.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  opts.max_iter = 500;

  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(2.0).margin(1e-5));
}

// ─── Equality constrained LP ──────────────────────────────────────────────
// min  x + y   s.t.  x + y = 4,  x,y >= 0
// Optimal: obj=4, x+y=4
TEST_CASE("DualSimplex: equality constrained LP", "[dual_simplex]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;

  lp.A.resize(0, 2);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(1, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(1);
  lp.beq << 4.0;

  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  auto res = solve_lp_with_basis(lp, opts);
  REQUIRE(res.result.stats.success);
  CHECK(res.result.stats.objective == Approx(4.0).margin(1e-5));
}

// ─── Basis round-trip: solve then warm-start ──────────────────────────────
TEST_CASE("DualSimplex: warm-start from previous basis", "[dual_simplex]") {
  // Same LP twice: second solve uses returned basis as hint.
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << -1.0, -2.0;

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 2.0;
  A.insert(1, 0) = 2.0; A.insert(1, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 14.0, 14.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  auto opts = native_simplex_options();
  auto first = solve_lp_with_basis(lp, opts);
  INFO("first status=" << first.result.stats.status
                        << " iterations=" << first.result.stats.iterations);
  REQUIRE(first.result.stats.success);

  // Re-solve with basis hint – should reach optimality in 0 pivots.
  auto second = solve_lp_with_basis(lp, opts, &first.basis);
  REQUIRE(second.result.stats.success);
  CHECK(second.result.stats.objective ==
        Approx(first.result.stats.objective).margin(1e-6));
}

TEST_CASE("DualSimplex: exact-breakpoint BFRT flips a complete prefix",
          "[dual_simplex][bfrt]") {
  // With x0 basic, x0=-2.5 is below its lower bound.  The three eligible
  // nonbasics have breakpoints 1, 2 and 3 and unit ranges.  A correct BFRT
  // flips x1 and x2 as complete earlier groups, then pivots x3 once.  A
  // one-at-a-time ratio test needs three basis exchanges on this model.
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(4);
  lp.c << 0.0, 1.0, 2.0, 3.0;
  lp.A.resize(0, 4);
  lp.b.resize(0);
  lp.Aeq.resize(1, 4);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = -1.0;
  lp.Aeq.insert(0, 2) = -1.0;
  lp.Aeq.insert(0, 3) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq.resize(1);
  lp.beq << -2.5;
  for (int j = 0; j < 4; ++j)
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});

  mipsolvers::engine::SimplexBasis hint;
  hint.rows = 1;
  hint.cols = 5;  // Four structural columns plus the equality artificial.
  hint.indices = {0};
  hint.at_upper.assign(5, 0);

  auto opts = native_simplex_options();
  opts.max_iter = 2;
  const auto solved = solve_lp_with_basis(lp, opts, &hint);
  INFO("status=" << solved.result.stats.status
                  << " iterations=" << solved.result.stats.iterations);
  REQUIRE(solved.result.stats.success);
  CHECK(solved.result.stats.iterations == 1);
  CHECK(solved.result.x[0] == Approx(0.0).margin(1e-10));
  CHECK(solved.result.x[1] == Approx(1.0).margin(1e-10));
  CHECK(solved.result.x[2] == Approx(1.0).margin(1e-10));
  CHECK(solved.result.x[3] == Approx(0.5).margin(1e-10));
  CHECK(solved.result.stats.objective == Approx(4.5).margin(1e-10));
}

TEST_CASE("DualSimplex: stabilized warm solve rebuilds and cleans original cost",
          "[dual_simplex][stabilization][cleanup]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(4);
  lp.c << 0.0, 1.0, 2.0, 3.0;
  lp.A.resize(0, 4);
  lp.b.resize(0);
  lp.Aeq.resize(1, 4);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = -1.0;
  lp.Aeq.insert(0, 2) = -1.0;
  lp.Aeq.insert(0, 3) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(1, -2.5);
  for (int j = 0; j < 4; ++j)
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});

  StandardFormLP sf = build_standard_form_lp(lp);
  mipsolvers::engine::SimplexBasis hint;
  hint.rows = 1;
  hint.cols = static_cast<int>(sf.A.cols());
  hint.indices = {0};
  hint.at_upper.assign(static_cast<std::size_t>(sf.A.cols()), 0);

  auto options = native_simplex_options();
  options.max_iter = 20;
  const auto result = native_dual::solve_phase2(sf, options, hint);
  INFO("status=" << native_dual::status_name(result.status)
                 << " message=" << result.message);
  REQUIRE(result.status == native_dual::Status::Optimal);
  CHECK(result.statistics.major_rebuilds >= 2);
  CHECK(result.statistics.max_cost_perturbation > 0.0);
  CHECK(result.statistics.devex_frameworks == 1);
  CHECK(result.statistics.dse_initialization_solves == 0);
  CHECK(result.statistics.harris_second_pass_candidates >= 1);
  CHECK(result.statistics.cleanup_passes == 1);
  CHECK(result.max_objective == Approx(-4.5).margin(1e-10));
}

TEST_CASE("DualSimplex: batched DSE uses one existing INVERT",
          "[dual_simplex][stabilization][dse][factor_differential]") {
  StandardFormLP sf;
  Eigen::Matrix<double, 3, 6> dense;
  dense << 2.0, 1.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 3.0, 1.0, 0.0, 1.0, 0.0,
           1.0, 0.0, 4.0, 0.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();
  std::vector<int> basis{0, 1, 2};
  mipsolvers::engine::native_dual::detail::BasisFactor factor(
      sf, {3, 4, 5});
  int repairs = 0;
  std::string failure;
  REQUIRE(factor.rebuild(basis, repairs, failure));
  const int rebuilds_before = factor.rebuild_count();

  const auto evidence = factor.compute_exact_edge_weights();
  INFO("residual=" << evidence.max_residual
                    << " limit=" << evidence.max_error_limit);
  REQUIRE(evidence.accepted);
  CHECK_FALSE(evidence.needs_rebuild);
  CHECK(factor.rebuild_count() == rebuilds_before);
  REQUIRE(evidence.weights.size() == 3);

  const Eigen::Matrix3d B = dense.leftCols<3>();
  const Eigen::Matrix3d inverse = B.inverse();
  for (int row = 0; row < 3; ++row) {
    CHECK(evidence.weights[static_cast<std::size_t>(row)] ==
          Approx(inverse.row(row).squaredNorm()).epsilon(2e-12));
  }
}

TEST_CASE("DualSimplex: reconstruction corrects canonical primal residual",
          "[dual_simplex][stabilization][refinement]") {
  using mipsolvers::engine::native_dual::detail::BasisFactor;
  using mipsolvers::engine::native_dual::detail::Move;
  using mipsolvers::engine::native_dual::detail::State;

  StandardFormLP sf;
  Eigen::Matrix<double, 2, 4> dense;
  dense << 1.0, 0.0, 1.0, 0.0,
           0.0, 1.0, 0.0, 1.0;
  sf.A = dense.sparseView();
  sf.A.makeCompressed();
  sf.b.resize(2);
  sf.b << 1.0, 2.0;
  sf.c_max = Eigen::VectorXd::Zero(4);
  sf.var_ub = Eigen::VectorXd::Constant(
      4, std::numeric_limits<double>::infinity());
  sf.n_original = 2;
  sf.row_to_artificial_col.assign(2, -1);
  sf.row_to_slack_col = {2, 3};

  auto options = native_simplex_options();
  options.feasibility_tol = 1e-8;
  State state;
  state.sf = &sf;
  state.options = &options;
  state.m = 2;
  state.n = 4;
  state.bounds.lower = Eigen::VectorXd::Zero(4);
  state.bounds.upper = Eigen::VectorXd::Constant(
      4, std::numeric_limits<double>::infinity());
  state.bounds.enterable.assign(4, 1);
  state.basis = {0, 1};
  state.move.assign(4, Move::Up);
  state.cost = sf.c_max;
  state.factor = std::make_shared<BasisFactor>(sf, std::vector<int>{2, 3});

  int repairs = 0;
  std::string failure;
  const bool rebuilt = state.factor->rebuild(state.basis, repairs, failure);
  INFO("factor failure=" << failure);
  REQUIRE(rebuilt);
  REQUIRE(mipsolvers::engine::native_dual::detail::rebuild_membership(
      state, failure));
  state.reduced_costs = Eigen::VectorXd::Zero(4);
  state.x_basic = sf.b;
  state.x_basic[0] += 1e-4;
  REQUIRE(
      mipsolvers::engine::native_dual::detail::correct_canonical_primal_residual(
          state, failure));
  INFO("failure=" << failure);
  CHECK(state.canonical_primal_corrections == 1);
  const Eigen::VectorXd x =
      mipsolvers::engine::native_dual::detail::full_primal(state);
  const double residual =
      mipsolvers::engine::native_dual::detail::equation_residual_inf(
          sf.A, x, sf.b);
  CHECK(residual <= options.feasibility_tol * sf.b.lpNorm<Eigen::Infinity>());
}

TEST_CASE("DualSimplex: repeated basis state taboos its prior outgoing edge",
          "[dual_simplex][stabilization][cycling][taboo]") {
  using mipsolvers::engine::native_dual::Statistics;
  using mipsolvers::engine::native_dual::detail::Move;
  using mipsolvers::engine::native_dual::detail::State;
  State state;
  state.m = 2;
  state.n = 4;
  state.basis = {0, 1};
  state.basic = {1, 1, 0, 0};
  state.move = {Move::Fixed, Move::Fixed, Move::Up, Move::Down};
  mipsolvers::engine::native_dual::detail::initialize_cycle_guard(state);

  mipsolvers::engine::native_dual::detail::record_cycle_departure(state, 0, 2);
  state.basis[0] = 2;
  state.basic = {0, 1, 1, 0};
  state.move = {Move::Up, Move::Fixed, Move::Fixed, Move::Down};
  Statistics statistics;
  CHECK_FALSE(mipsolvers::engine::native_dual::detail::record_cycle_arrival(
      state, statistics));

  mipsolvers::engine::native_dual::detail::record_cycle_departure(state, 2, 0);
  state.basis[0] = 0;
  state.basic = {1, 1, 0, 0};
  state.move = {Move::Fixed, Move::Fixed, Move::Up, Move::Down};
  REQUIRE(mipsolvers::engine::native_dual::detail::record_cycle_arrival(
      state, statistics));
  CHECK(statistics.cycles_detected == 1);
  CHECK(statistics.taboo_changes == 1);
  CHECK(mipsolvers::engine::native_dual::detail::is_taboo_change(state, 0, 2));
  CHECK_FALSE(
      mipsolvers::engine::native_dual::detail::is_taboo_change(state, 0, 3));
  mipsolvers::engine::native_dual::detail::add_taboo_row(state, 0,
                                                         statistics);
  CHECK(statistics.taboo_rows == 1);
  CHECK(mipsolvers::engine::native_dual::detail::is_taboo_row(state, 0));
  mipsolvers::engine::native_dual::detail::release_taboo_row(state, 0,
                                                             statistics);
  CHECK(statistics.taboo_row_releases == 1);
  CHECK_FALSE(mipsolvers::engine::native_dual::detail::is_taboo_row(state, 0));
}

TEST_CASE("DualSimplex: large LPModel warm-start stays sparse",
          "[dual_simplex][warm_start]") {
  constexpr int rows = 256;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Ones(rows);
  lp.A.resize(0, rows);
  lp.b.resize(0);
  lp.Aeq.resize(rows, rows);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(rows);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(rows);
  for (int i = 0; i < rows; ++i) {
    triplets.emplace_back(i, i, 1.0);
    lp.vars.push_back({VarType::Continuous, 0.0, 2.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 1000;
  opts.escalation_max_level = 0;

  const auto first = solve_lp_with_basis(lp, opts);
  REQUIRE(first.result.stats.success);
  REQUIRE(first.basis.cached_sparse_basis);

  const auto second = solve_lp_with_basis(lp, opts, &first.basis);
  REQUIRE(second.result.stats.success);
  CHECK(second.solved_from_hint);
  CHECK(second.dual_reoptimized);
  CHECK(second.result.stats.iterations <= 1);
  CHECK(second.basis_inverse.size() == 0);
  CHECK(second.basis.cached_sparse_basis);
  CHECK(second.result.stats.objective == Approx(rows).margin(1e-8));
}

TEST_CASE("DualSimplex: sparse warm-start honors the explicit iteration limit",
          "[dual_simplex][warm_start]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel root;
  root.sense = Sense::Minimize;
  root.c = Eigen::VectorXd::Zero(cols);
  root.A.resize(0, cols);
  root.b.resize(0);
  root.Aeq.resize(rows, cols);
  root.beq = Eigen::VectorXd::Ones(rows);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    triplets.emplace_back(row, 2 * row, 1.0);
    triplets.emplace_back(row, 2 * row + 1, 1.0);
    root.c[2 * row + 1] = 1.0;
    root.vars.push_back({VarType::Continuous, 0.0, 1.0});
    root.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  root.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  root.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 160;
  opts.escalation_max_level = 0;

  const auto first = solve_lp_with_basis(root, opts);
  REQUIRE(first.result.stats.success);
  REQUIRE(first.basis.cached_dse_weights);
  REQUIRE(first.basis.cached_dse_basis);

  mipsolvers::engine::SimplexBasis portable_hint = first.basis;
  portable_hint.cached_sparse_basis.reset();

  LPModel child = root;
  for (int row = 0; row < rows; ++row) {
    child.vars[2 * row].ub = 0.4;
  }
  const auto second = solve_lp_with_basis(child, opts, &portable_hint);
  INFO("status=" << second.result.stats.status
                  << " iterations=" << second.result.stats.iterations);
  CHECK_FALSE(second.result.stats.success);
  CHECK(second.result.stats.status.starts_with(
      "Native dual simplex: dual simplex iteration limit"));
  CHECK(second.result.stats.iterations == opts.max_iter);
}

TEST_CASE("DualSimplex: warm reoptimization excludes outage-fixed columns",
          "[dual_simplex][warm_start][fixed_bounds][highs_differential]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(cols);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    triplets.emplace_back(row, row, 1.0);
    triplets.emplace_back(row, rows + row, 1.0);
    lp.c[rows + row] = 1.0;
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  for (int row = 0; row < rows; ++row) {
    lp.vars.push_back({VarType::Continuous, 0.0, 1.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto options = native_simplex_options();
  options.max_iter = 1000;
  options.escalation_max_level = 0;

  auto root = solve_lp_with_basis(lp, options);
  REQUIRE(root.result.stats.success);
  CHECK(root.result.stats.objective == Approx(0.0).margin(1e-9));

  for (int row = 0; row < rows; ++row) lp.vars[row].ub = 0.0;
  const double highs_objective = solve_with_highs(lp);
  auto outage = solve_lp_with_basis(lp, options, &root.basis);
  INFO("status=" << outage.result.stats.status
                  << " iterations=" << outage.result.stats.iterations);
  REQUIRE(outage.result.stats.success);
  CHECK(outage.solved_from_hint);
  CHECK(outage.dual_reoptimized);
  CHECK(outage.result.stats.objective == Approx(highs_objective).margin(1e-8));
  CHECK((lp.Aeq * outage.result.x - lp.beq).lpNorm<Eigen::Infinity>() <= 1e-9);
}

TEST_CASE("DualSimplex: warm reoptimization repairs a rank-deficient outage basis",
          "[dual_simplex][warm_start][rank_repair][highs_differential]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel root;
  root.sense = Sense::Minimize;
  root.c = Eigen::VectorXd::Zero(cols);
  root.A.resize(0, cols);
  root.b.resize(0);
  root.Aeq.resize(rows, cols);
  root.beq = Eigen::VectorXd::Ones(rows);
  root.vars.reserve(cols);

  std::vector<Eigen::Triplet<double>> root_triplets;
  root_triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    root_triplets.emplace_back(row, row, 1.0);
    root_triplets.emplace_back(row, rows + row, 0.5);
    root.c[rows + row] = 1.0;
  }
  for (int col = 0; col < cols; ++col) {
    root.vars.push_back({VarType::Continuous, 0.0, 2.0});
  }
  root.Aeq.setFromTriplets(root_triplets.begin(), root_triplets.end());
  root.Aeq.makeCompressed();

  auto options = native_simplex_options();
  options.max_iter = 2000;
  options.escalation_max_level = 0;

  auto native = solve_lp_with_basis(root, options);
  REQUIRE(native.result.stats.success);
  CHECK(native.result.stats.objective == Approx(0.0).margin(1e-9));

  LPModel outage = root;
  std::vector<Eigen::Triplet<double>> outage_triplets;
  outage_triplets.reserve(cols + 2);
  outage_triplets.emplace_back(0, 0, 1.0);
  outage_triplets.emplace_back(1, 0, 1.0);
  outage_triplets.emplace_back(0, 1, 1.0);
  outage_triplets.emplace_back(1, 1, 1.0);
  for (int row = 2; row < rows; ++row) {
    outage_triplets.emplace_back(row, row, 1.0);
  }
  for (int row = 0; row < rows; ++row) {
    outage_triplets.emplace_back(row, rows + row, 0.5);
  }
  outage.Aeq.setZero();
  outage.Aeq.setFromTriplets(outage_triplets.begin(), outage_triplets.end());
  outage.Aeq.makeCompressed();

  const double highs_objective = solve_with_highs(outage);
  auto repaired = solve_lp_with_basis(outage, options, &native.basis);
  INFO("status=" << repaired.result.stats.status
                  << " iterations=" << repaired.result.stats.iterations);
  REQUIRE(repaired.result.stats.success);
  CHECK(repaired.solved_from_hint);
  CHECK(repaired.dual_reoptimized);
  CHECK(repaired.result.stats.objective == Approx(highs_objective).margin(1e-8));
  CHECK((outage.Aeq * repaired.result.x - outage.beq)
            .lpNorm<Eigen::Infinity>() <= 1e-9);
}

TEST_CASE("HFactor basis repair replaces only the column without a pivot",
          "[dual_simplex][rank_repair][hfactor]") {
  Eigen::SparseMatrix<double> A(2, 4);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 1.0}, {0, 1, 1.0}, {0, 2, 2.0}, {1, 3, 3.0}};
  A.setFromTriplets(entries.begin(), entries.end());
  A.makeCompressed();

  std::vector<int> basis{0, 1};
  const std::vector<int> logical{2, 3};
  std::vector<int> repaired;
  HFactorBackend factor;
  REQUIRE(factor.factorize_with_logicals(A, basis.data(), 2, logical,
                                         repaired));
  REQUIRE(factor.rank_deficiency == 1);
  REQUIRE(repaired.size() == basis.size());
  CHECK(std::inner_product(
            repaired.begin(), repaired.end(), basis.begin(), 0,
            std::plus<>(), std::not_equal_to<>()) == 1);

  const Eigen::Vector2d rhs(2.0, 6.0);
  Eigen::Vector2d solution;
  factor.ftran(rhs.data(), solution.data());
  CHECK((solution - Eigen::Vector2d(2.0, 2.0)).lpNorm<Eigen::Infinity>() ==
        Approx(0.0).margin(1e-12));
}

TEST_CASE("HFactor preserves caller basis positions and solve coordinates",
          "[dual_simplex][basis_order][hfactor]") {
  Eigen::SparseMatrix<double> A(3, 6);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 1.0}, {1, 1, 1.0}, {2, 2, 1.0},
      {0, 3, 2.0}, {1, 4, 3.0}, {2, 5, 4.0}};
  A.setFromTriplets(entries.begin(), entries.end());
  A.makeCompressed();

  const std::vector<int> basis{2, 0, 1};
  const std::vector<int> logical{3, 4, 5};
  std::vector<int> repaired;
  HFactorBackend factor;
  REQUIRE(factor.factorize_with_logicals(A, basis.data(), 3, logical,
                                         repaired));
  REQUIRE(factor.rank_deficiency == 0);
  CHECK(repaired == basis);

  const Eigen::Vector3d rhs(10.0, 20.0, 30.0);
  Eigen::Vector3d primal;
  factor.ftran(rhs.data(), primal.data());
  CHECK((primal - Eigen::Vector3d(30.0, 10.0, 20.0))
            .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(1e-12));

  const Eigen::Vector3d basic_cost(3.0, 5.0, 7.0);
  Eigen::Vector3d dual;
  factor.btran(basic_cost.data(), dual.data());
  CHECK((dual - Eigen::Vector3d(5.0, 7.0, 3.0))
            .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(1e-12));
}

TEST_CASE("HFactor fresh solves arbitrary non-diagonal bases",
          "[dual_simplex][basis_order][hfactor][factor_differential]") {
  Eigen::Matrix<double, 5, 10> dense;
  dense << 4.0, 1.0, 0.0, 2.0, 0.0, 1.0, 0.0, 2.0, -1.0, 3.0,
           1.0, 5.0, 2.0, 0.0, 0.0, 3.0, 1.0, 0.0,  2.0, 1.0,
           0.0, 2.0, 6.0, 1.0, 1.0, 0.0, 4.0, 1.0,  0.0, 2.0,
           2.0, 0.0, 1.0, 7.0, 2.0, 1.0, 0.0, 5.0,  1.0, 0.0,
           0.0, 0.0, 1.0, 2.0, 8.0, 2.0, 1.0, 0.0,  4.0, 3.0;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();
  const std::vector<std::vector<int>> bases{
      {0, 1, 2, 3, 4}, {3, 0, 4, 1, 2}, {5, 6, 7, 8, 9},
      {8, 2, 5, 0, 7}};
  const Eigen::Vector<double, 5> primal_rhs(0.5, -2.0, 3.5, 7.0, -1.25);
  const Eigen::Vector<double, 5> dual_rhs(-1.0, 4.0, 2.5, -3.0, 0.75);

  for (const std::vector<int>& basis : bases) {
    Eigen::Matrix<double, 5, 5> B;
    for (int position = 0; position < 5; ++position) {
      B.col(position) = dense.col(basis[static_cast<std::size_t>(position)]);
    }
    REQUIRE(B.fullPivLu().rank() == 5);
    HFactorBackend factor;
    REQUIRE(factor.factorize(A, basis.data(), 5));

    Eigen::Vector<double, 5> actual_primal;
    Eigen::Vector<double, 5> actual_dual;
    factor.ftran(primal_rhs.data(), actual_primal.data());
    factor.btran(dual_rhs.data(), actual_dual.data());
    CHECK((actual_primal - B.fullPivLu().solve(primal_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
    CHECK((actual_dual - B.transpose().fullPivLu().solve(dual_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
  }
}

TEST_CASE("HFactor product-form chain from a non-diagonal basis matches fresh LU",
          "[dual_simplex][factor_update][hfactor][factor_differential]") {
  Eigen::Matrix<double, 5, 10> dense;
  dense << 4.0, 1.0, 0.0, 2.0, 0.0, 1.0, 0.0, 2.0, -1.0, 3.0,
           1.0, 5.0, 2.0, 0.0, 0.0, 3.0, 1.0, 0.0,  2.0, 1.0,
           0.0, 2.0, 6.0, 1.0, 1.0, 0.0, 4.0, 1.0,  0.0, 2.0,
           2.0, 0.0, 1.0, 7.0, 2.0, 1.0, 0.0, 5.0,  1.0, 0.0,
           0.0, 0.0, 1.0, 2.0, 8.0, 2.0, 1.0, 0.0,  4.0, 3.0;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();
  std::vector<int> basis{3, 0, 4, 1, 2};
  HFactorBackend updated;
  REQUIRE(updated.factorize(A, basis.data(), 5));
  const std::vector<std::pair<int, int>> exchanges{
      {0, 5}, {1, 6}, {2, 7}, {3, 8}, {4, 9},
      {0, 3}, {1, 0}, {2, 4}, {3, 1}, {4, 2}};
  const Eigen::Vector<double, 5> primal_rhs(0.5, -2.0, 3.5, 7.0, -1.25);
  const Eigen::Vector<double, 5> dual_rhs(-1.0, 4.0, 2.5, -3.0, 0.75);
  HFactorBackend reused_fresh;

  auto matrix_for = [&](const std::vector<int>& index) {
    Eigen::Matrix<double, 5, 5> B;
    for (int position = 0; position < 5; ++position) {
      B.col(position) = dense.col(index[static_cast<std::size_t>(position)]);
    }
    return B;
  };
  for (const auto& [pivot_position, entering_col] : exchanges) {
    const auto before = matrix_for(basis);
    const Eigen::Vector<double, 5> aq =
        before.fullPivLu().solve(dense.col(entering_col));
    Eigen::Vector<double, 5> unit = Eigen::Vector<double, 5>::Zero();
    unit[pivot_position] = 1.0;
    const Eigen::Vector<double, 5> ep =
        before.transpose().fullPivLu().solve(unit);
    REQUIRE(std::abs(aq[pivot_position]) > 1e-12);
    Eigen::Vector<double, 5> captured_aq;
    Eigen::Vector<double, 5> captured_ep;
    updated.ftran_for_update(dense.col(entering_col).data(), captured_aq.data());
    updated.btran_for_update(unit.data(), captured_ep.data());
    REQUIRE(updated.update(pivot_position, entering_col, captured_aq.data(),
                           captured_ep.data()));
    basis[static_cast<std::size_t>(pivot_position)] = entering_col;

    const auto after = matrix_for(basis);
    REQUIRE(after.fullPivLu().rank() == 5);
    HFactorBackend fresh;
    REQUIRE(fresh.factorize(A, basis.data(), 5));
    REQUIRE(reused_fresh.factorize(A, basis.data(), 5));
    Eigen::Vector<double, 5> updated_primal, updated_dual;
    Eigen::Vector<double, 5> fresh_primal, fresh_dual;
    Eigen::Vector<double, 5> reused_primal, reused_dual;
    updated.ftran(primal_rhs.data(), updated_primal.data());
    updated.btran(dual_rhs.data(), updated_dual.data());
    fresh.ftran(primal_rhs.data(), fresh_primal.data());
    fresh.btran(dual_rhs.data(), fresh_dual.data());
    reused_fresh.ftran(primal_rhs.data(), reused_primal.data());
    reused_fresh.btran(dual_rhs.data(), reused_dual.data());
    CHECK((updated_primal - fresh_primal).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((updated_dual - fresh_dual).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((reused_primal - fresh_primal).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((reused_dual - fresh_dual).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((updated_primal - after.fullPivLu().solve(primal_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
    CHECK((updated_dual - after.transpose().fullPivLu().solve(dual_rhs))
              .lpNorm<Eigen::Infinity>() == Approx(0.0).margin(2e-11));
  }
}

TEST_CASE("HFactor FT update chain is algebraically identical to fresh bases",
          "[dual_simplex][factor_update][hfactor]") {
  Eigen::Matrix<double, 4, 8> dense;
  dense << 1, 0, 0, 0, 2, 1, 1, 1,
           0, 1, 0, 0, 1, 3, 1, 1,
           0, 0, 1, 0, 1, 1, 4, 1,
           0, 0, 0, 1, 1, 1, 1, 5;
  Eigen::SparseMatrix<double> A = dense.sparseView();
  A.makeCompressed();

  std::vector<int> basis{0, 1, 2, 3};
  HFactorBackend updated;
  REQUIRE(updated.factorize(A, basis.data(), 4));

  const std::vector<std::pair<int, int>> exchanges{
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
      {0, 0}, {1, 1}, {2, 2}, {3, 3}};
  const Eigen::Vector4d primal_rhs(0.5, -2.0, 3.5, 7.0);
  const Eigen::Vector4d dual_rhs(-1.0, 4.0, 2.5, -3.0);

  auto fresh_basis = [&]() {
    Eigen::Matrix4d B;
    for (int position = 0; position < 4; ++position) {
      B.col(position) = dense.col(basis[static_cast<std::size_t>(position)]);
    }
    return B;
  };

  for (const auto& [pivot_position, entering_col] : exchanges) {
    const Eigen::Matrix4d before = fresh_basis();
    const Eigen::Vector4d entering = dense.col(entering_col);
    const Eigen::Vector4d aq = before.fullPivLu().solve(entering);
    Eigen::Vector4d unit = Eigen::Vector4d::Zero();
    unit[pivot_position] = 1.0;
    const Eigen::Vector4d ep = before.transpose().fullPivLu().solve(unit);
    REQUIRE(std::abs(aq[pivot_position]) > 1e-12);
    Eigen::Vector4d captured_aq;
    Eigen::Vector4d captured_ep;
    updated.ftran_for_update(entering.data(), captured_aq.data());
    updated.btran_for_update(unit.data(), captured_ep.data());
    REQUIRE(updated.update(pivot_position, entering_col, captured_aq.data(),
                           captured_ep.data()));
    basis[static_cast<std::size_t>(pivot_position)] = entering_col;

    const Eigen::Matrix4d after = fresh_basis();
    REQUIRE(after.fullPivLu().rank() == 4);
    Eigen::Vector4d actual_primal;
    Eigen::Vector4d actual_dual;
    updated.ftran(primal_rhs.data(), actual_primal.data());
    updated.btran(dual_rhs.data(), actual_dual.data());
    const Eigen::Vector4d expected_primal =
        after.fullPivLu().solve(primal_rhs);
    const Eigen::Vector4d expected_dual =
        after.transpose().fullPivLu().solve(dual_rhs);
    CHECK((actual_primal - expected_primal).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
    CHECK((actual_dual - expected_dual).lpNorm<Eigen::Infinity>() ==
          Approx(0.0).margin(2e-11));
  }
}

TEST_CASE("DualSimplex: BFRT warm sequence agrees with HiGHS",
          "[dual_simplex][warm_start][bfrt][highs_differential]") {
  constexpr int side = 12;
  LPModel lp = make_degenerate_transport_lp(side);
  auto options = native_simplex_options();
  options.max_iter = 2000;
  options.escalation_max_level = 0;

  auto native = solve_lp_with_basis(lp, options);
  REQUIRE(native.result.stats.success);
  CHECK(native.result.stats.objective == Approx(solve_with_highs(lp)).margin(1e-8));

  auto check_dse_weights = [](const SimplexResult& result) {
    REQUIRE(result.basis.cached_sparse_basis);
    REQUIRE(result.basis.cached_dse_weights);
    REQUIRE(result.basis.cached_dse_weights->size() ==
            static_cast<std::size_t>(result.form.A.rows()));
    for (int row = 0; row < result.form.A.rows(); ++row) {
      Eigen::VectorXd unit = Eigen::VectorXd::Zero(result.form.A.rows());
      unit[row] = 1.0;
      const Eigen::VectorXd inverse_row =
          result.basis.cached_sparse_basis->btran(unit);
      REQUIRE(inverse_row.size() == result.form.A.rows());
      CHECK((*result.basis.cached_dse_weights)[static_cast<std::size_t>(row)] ==
            Approx(inverse_row.squaredNorm()).epsilon(2e-10).margin(2e-12));
    }
  };
  check_dse_weights(native);

  for (int step = 1; step <= 3; ++step) {
    int changed = 0;
    for (int col : native.basis.basis_indices()) {
      if (col < 0 || col >= side * side || native.result.x[col] <= 0.15) {
        continue;
      }
      auto& var = lp.vars[static_cast<size_t>(col)];
      const double next_ub = native.result.x[col] - 0.1;
      if (next_ub < var.ub - 1e-10) {
        var.ub = next_ub;
        ++changed;
        if (changed == 2) break;
      }
    }
    REQUIRE(changed > 0);
    const double highs_objective = solve_with_highs(lp);
    auto warm = solve_lp_with_basis(lp, options, &native.basis);
    INFO("step=" << step << " status=" << warm.result.stats.status
                 << " iterations=" << warm.result.stats.iterations);
    REQUIRE(warm.result.stats.success);
    CHECK(warm.solved_from_hint);
    CHECK(warm.dual_reoptimized);
    CHECK(warm.result.stats.iterations > 0);
    CHECK(warm.result.stats.objective == Approx(highs_objective).margin(1e-7));
    INFO("scaled_residual="
         << (warm.form.A * warm.x_std - warm.form.b)
                .lpNorm<Eigen::Infinity>());
    CHECK((warm.form.A * warm.x_std - warm.form.b)
              .lpNorm<Eigen::Infinity>() <= 1e-8);
    CHECK((lp.Aeq * warm.result.x - lp.beq).lpNorm<Eigen::Infinity>() <= 1e-8);
    native = std::move(warm);
    check_dse_weights(native);
  }
}

TEST_CASE("DualSimplex: cold Phase I honors the explicit pivot budget",
          "[dual_simplex][phase1]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows + 1;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(2 * rows);
  for (int row = 0; row < rows; ++row) {
    // The crash selects 2*x first, but x=0.5 violates x<=0.4.  Phase I
    // therefore needs a bound flip followed by y entering for every row.
    triplets.emplace_back(row, 2 * row, 2.0);
    triplets.emplace_back(row, 2 * row + 1, 1.0);
    lp.vars.push_back({VarType::Continuous, 0.0, 0.4});
    lp.vars.push_back({VarType::Continuous, 0.0, 0.2});
  }
  // A lower-only improving zero column makes a dual-feasible endpoint
  // assignment impossible for the cold basis, so the exact start
  // classification must select primal Phase I without affecting its pivots.
  lp.c[cols - 1] = -1.0;
  lp.vars.push_back({VarType::Continuous, 0.0,
                     std::numeric_limits<double>::infinity()});
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 300;
  opts.escalation_max_level = 0;

  const auto solved = solve_lp_with_basis(lp, opts);
  INFO("status=" << solved.result.stats.status
                  << " iterations=" << solved.result.stats.iterations);
  CHECK_FALSE(solved.result.stats.success);
  CHECK(solved.result.stats.status ==
        "Native dual simplex: primal Phase I: primal simplex iteration limit");
  CHECK(solved.result.stats.iterations == opts.max_iter);
}

TEST_CASE("DualSimplex: crash basis skips fixed-width structural columns",
          "[dual_simplex][phase1][crash]") {
  constexpr int rows = 256;
  constexpr int cols = 2 * rows;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.reserve(cols);

  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(cols);
  for (int row = 0; row < rows; ++row) {
    // The first, identity-like column is an attractive crash candidate but is
    // fixed at zero. The second column is the only one that can satisfy the
    // row and must be selected instead.
    triplets.emplace_back(row, row, -1.0);
    triplets.emplace_back(row, rows + row, 1.0);
    lp.vars.push_back({VarType::Continuous, 0.0, 0.0});
  }
  for (int row = 0; row < rows; ++row) {
    lp.vars.push_back({VarType::Continuous, 0.0, 2.0});
  }
  lp.Aeq.setFromTriplets(triplets.begin(), triplets.end());
  lp.Aeq.makeCompressed();

  auto opts = native_simplex_options();
  opts.max_iter = 1000;
  opts.escalation_max_level = 0;

  const auto solved = solve_lp_with_basis(lp, opts);
  INFO("status=" << solved.result.stats.status
                  << " iterations=" << solved.result.stats.iterations);
  REQUIRE(solved.result.stats.success);
  CHECK(solved.result.stats.iterations <= 1);
  for (int row = 0; row < rows; ++row) {
    CHECK(solved.result.x[row] == Approx(0.0).margin(1e-12));
    CHECK(solved.result.x[rows + row] == Approx(1.0).margin(1e-9));
  }
}
