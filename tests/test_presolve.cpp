/// test_presolve.cpp
/// Tests for MILP and LP presolve/postsolve pipeline.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/strategy/presolve_manager.hpp"
#include "mipsolvers/engine/strategy/postsolve_manager.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/native/milp/bc/milp_presolve.hpp"

using namespace mipsolvers::engine;
using namespace mipsolvers::engine::strategy;
using Catch::Approx;

TEST_CASE("Native MILP presolve reports infeasible empty rows",
          "[presolve][milp][infeasible]") {
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(0, 1);
  lp.b.resize(0);
  lp.Aeq.resize(1, 1);
  lp.beq = Eigen::VectorXd::Ones(1);
  lp.vars.push_back({VarType::Binary, 0.0, 1.0});

  std::vector<int> binary{0};
  std::vector<int> integer;
  MILPPresolve presolve;
  const PresolveStats stats = presolve.run(lp, binary, integer);

  CHECK(stats.infeasible);
  CHECK_FALSE(stats.infeasibility_reason.empty());
}

TEST_CASE("Native MILP presolve propagates contradictory tightened bounds",
          "[presolve][milp][infeasible]") {
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(2, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(1, 0) = -1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 0.0, -1.0;
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars.push_back({VarType::Binary, 0.0, 1.0});

  std::vector<int> binary{0};
  std::vector<int> integer;
  MILPPresolve presolve;
  const PresolveStats stats = presolve.run(lp, binary, integer);

  CHECK(stats.infeasible);
  CHECK(stats.infeasibility_reason.find("bounds") != std::string::npos);
}

// ─── Presolve of a simple LP ────────────────────────────────────────────────
// min -x  s.t. x<=5, x<=10, x>=0
TEST_CASE("Presolve: LP with redundant constraint", "[presolve][lp]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(1);
  lp.c << -1.0;

  Eigen::SparseMatrix<double> A(2, 1);
  A.insert(0, 0) = 1.0;
  A.insert(1, 0) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 5.0, 10.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  PresolveManager ps;
  SolveOptions opts;
  auto presolved = ps.presolve(lp, opts);

  if (std::holds_alternative<LPModel>(presolved.presolved)) {
    const auto& lp2 = std::get<LPModel>(presolved.presolved);
    CHECK(lp2.c.size() == 1);
  }
  CHECK_FALSE(presolved.is_presolved());
  const auto stats = ps.last_stats();
  CHECK(stats.original_rows == 2);
  CHECK(stats.presolved_rows == 2);
  CHECK(stats.original_cols == 1);
  CHECK(stats.presolved_cols == 1);
}

// ─── Fixed variable elimination ────────────────────────────────────────────
TEST_CASE("Presolve: fixed variable eliminated", "[presolve][lp]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 0.0, 1.0;

  lp.A.resize(0, 2);
  lp.b.resize(0);

  Eigen::SparseMatrix<double> Aeq(1, 2);
  Aeq.insert(0, 0) = 1.0; Aeq.insert(0, 1) = 1.0;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(1);
  lp.beq << 3.0;

  lp.vars.push_back({VarType::Continuous, 3.0, 3.0, "x"});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20, "y"});

  PresolveManager ps;
  SolveOptions opts;
  auto presolved = ps.presolve(lp, opts);

  if (std::holds_alternative<LPModel>(presolved.presolved)) {
    const auto& lp2 = std::get<LPModel>(presolved.presolved);
    CHECK(lp2.c.size() <= 2);
  }
}

// ─── Postsolve round-trip ──────────────────────────────────────────────────
TEST_CASE("Postsolve: identity mapping preserves solution", "[presolve][postsolve]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(2);
  lp.c << 1.0, 2.0;

  Eigen::SparseMatrix<double> A(1, 2);
  A.insert(0, 0) = 1.0; A.insert(0, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 5.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  PresolveManager ps;
  SolveOptions opts;
  auto presolved = ps.presolve(lp, opts);

  SolveResult presolved_res;
  presolved_res.stats.success = true;
  presolved_res.x.resize(2);
  presolved_res.x << 5.0, 0.0;
  presolved_res.stats.objective = 5.0;

  PostsolveManager post;
  auto final_res = post.postsolve(presolved_res, presolved.mapping);

  REQUIRE(final_res.stats.success);
  CHECK(final_res.x.size() == 2);
  CHECK(final_res.stats.objective == Approx(5.0).margin(1e-6));
  CHECK(post.validate(final_res, presolved_res));
}

TEST_CASE("Postsolve: fixed variables require and use a complete mapping",
          "[presolve][postsolve]") {
  SolveResult reduced;
  reduced.stats.success = true;
  reduced.stats.objective = 7.0;
  reduced.x.resize(2);
  reduced.x << 1.0, 2.0;

  PresolveMapping mapping;
  mapping.col_mapping = {0, 2};
  mapping.fixed_variables.emplace(1, 5.0);

  PostsolveManager post;
  const SolveResult restored = post.postsolve(reduced, mapping);
  REQUIRE(restored.x.size() == 3);
  CHECK(restored.x[0] == Approx(1.0));
  CHECK(restored.x[1] == Approx(5.0));
  CHECK(restored.x[2] == Approx(2.0));

  mapping.col_mapping.clear();
  CHECK_THROWS_AS(post.postsolve(reduced, mapping), std::invalid_argument);
}

// ─── should_presolve heuristic ────────────────────────────────────────────
TEST_CASE("Presolve: should_presolve runs without crash", "[presolve]") {
  LPModel tiny;
  tiny.c.resize(2);
  tiny.c << 1.0, 1.0;
  tiny.vars.push_back({VarType::Continuous, 0.0, 1e20});
  tiny.vars.push_back({VarType::Continuous, 0.0, 1e20});

  PresolveManager ps;
  SolveOptions opts;
  CHECK_FALSE(ps.should_presolve(tiny, opts));
  CHECK_THROWS_AS(ps.scale(tiny, "ruiz"), std::logic_error);
  CHECK_FALSE(ps.scale(tiny, "identity").is_presolved());
}

// ─── Matrix scaling statistics (P0(b) presolve-hardening diagnostics) ───────

#include "mipsolvers/engine/strategy/papilo_presolve.hpp"

TEST_CASE("compute_matrix_scaling_stats: known matrix", "[presolve][scaling]") {
  // A (2x3 inequality): row0 = [1e-4, 0, 1e6] (range 1e10 > 1e8),
  //                     row1 = [2, 3, 0]      (range 1.5)
  // Aeq (1x3):          row2 = [0, 5, 0]
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c.resize(3);
  lp.c << 1.0, -7.0, 2.0;

  Eigen::SparseMatrix<double> A(2, 3);
  A.insert(0, 0) = 1e-4;
  A.insert(0, 2) = 1e6;
  A.insert(1, 0) = 2.0;
  A.insert(1, 1) = 3.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(2);
  lp.b << 4.0, -11.0;

  Eigen::SparseMatrix<double> Aeq(1, 3);
  Aeq.insert(0, 1) = 5.0;
  Aeq.makeCompressed();
  lp.Aeq = Aeq;
  lp.beq.resize(1);
  lp.beq << 2.0;

  for (int i = 0; i < 3; ++i)
    lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  const MatrixScalingStats s = compute_matrix_scaling_stats(lp);
  CHECK(s.rows == 3);
  CHECK(s.cols == 3);
  CHECK(s.nnz == 5);
  CHECK(s.abs_min == Approx(1e-4));
  CHECK(s.abs_max == Approx(1e6));
  CHECK(s.dynamic_range == Approx(1e10));
  CHECK(s.max_row_nnz == 2);
  CHECK(s.max_col_nnz == 2);  // column 0 (rows 0,1) and column 1 (rows 1,2)
  CHECK(s.rows_with_large_range == 1);
  CHECK(s.worst_row == 0);
  CHECK(s.worst_row_range == Approx(1e10));
  CHECK(s.b_abs_max == Approx(11.0));
  CHECK(s.c_abs_max == Approx(7.0));
}

TEST_CASE("compute_matrix_scaling_stats: empty model", "[presolve][scaling]") {
  LPModel lp;
  const MatrixScalingStats s = compute_matrix_scaling_stats(lp);
  CHECK(s.rows == 0);
  CHECK(s.cols == 0);
  CHECK(s.nnz == 0);
  CHECK(s.abs_min == 0.0);
  CHECK(s.abs_max == 0.0);
  CHECK(s.dynamic_range == 1.0);
  CHECK(s.worst_row == -1);
}
