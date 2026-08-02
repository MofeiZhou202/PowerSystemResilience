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

TEST_CASE("Native MILP presolve preserves finite lower row sides",
          "[presolve][milp][ranged-row]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.row_lhs = Eigen::VectorXd::Constant(1, 1.0);
  lp.b = Eigen::VectorXd::Constant(1, 1e20);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.assign(2, {VarType::Continuous, 0.0, 1.0});

  PresolveOptions options;
  options.max_rounds = 1;
  options.do_probing = false;
  std::vector<int> binary;
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  REQUIRE_FALSE(stats.infeasible);
  REQUIRE(lp.A.rows() == 1);
  REQUIRE(lp.b.size() == 1);
  CHECK(lp.A.coeff(0, 0) == Approx(-1.0));
  CHECK(lp.A.coeff(0, 1) == Approx(-1.0));
  CHECK(lp.b[0] == Approx(-1.0));
  CHECK(lp.row_lhs.size() == 0);
}

TEST_CASE("Native MILP presolve maintains row activities incrementally",
          "[presolve][milp][activity]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(4);
  lp.A.resize(2, 4);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.insert(1, 2) = 1.0;
  lp.A.insert(1, 3) = 1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 5.0, 20.0;
  lp.Aeq.resize(0, 4);
  lp.beq.resize(0);
  lp.vars.assign(4, {VarType::Continuous, 0.0, 10.0});

  PresolveOptions options;
  options.max_rounds = 1;
  options.do_singleton_rows = false;
  options.do_forcing_rows = false;
  options.do_probing = false;
  options.do_doubleton = false;
  options.do_parallel_rows = false;
  options.do_dominated = false;

  std::vector<int> binary;
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  INFO(stats.infeasibility_reason);
  REQUIRE_FALSE(stats.infeasible);
  CHECK(stats.activity_rows_recomputed == 2);
  CHECK(stats.activity_delta_updates >= 2);
  CHECK(stats.bounds_tightened >= 2);
}

TEST_CASE("Native MILP presolve retains cancelled row contributions",
          "[presolve][milp][numerics]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(4);
  lp.A.resize(2, 4);
  for (int r = 0; r < 2; ++r) {
    lp.A.insert(r, 0) = 1e16;
    lp.A.insert(r, 1) = 1.0;
    lp.A.insert(r, 2) = -1e16;
    lp.A.insert(r, 3) = 1.0;
  }
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(2);
  lp.Aeq.resize(0, 4);
  lp.beq.resize(0);
  lp.vars = {{VarType::Continuous, 1.0, 2.0},
             {VarType::Continuous, 1.0, 2.0},
             {VarType::Continuous, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1.0}};

  PresolveOptions options;
  options.max_rounds = 1;
  options.do_singleton_rows = false;
  options.do_forcing_rows = false;
  options.do_probing = false;
  options.do_doubleton = false;
  options.do_parallel_rows = false;
  options.do_dominated = false;

  std::vector<int> binary;
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  INFO(stats.infeasibility_reason);
  REQUIRE_FALSE(stats.infeasible);
  REQUIRE(lp.vars.size() == 4);
  CHECK(lp.vars[3].lb == Approx(0.0));
  CHECK(lp.vars[3].ub <= 1e-8);
}

TEST_CASE("Native MILP probing uses a sparse trail and retains implications",
          "[presolve][milp][probing][incremental]") {
  constexpr int kChainCols = 3;
  constexpr int kUnrelatedCols = 100;
  constexpr int kCols = kChainCols + kUnrelatedCols;
  constexpr int kChainRows = 4;
  constexpr int kRows = kChainRows + 2 * kUnrelatedCols;

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(kCols);
  lp.A.resize(kRows, kCols);
  // Duplicate each chain row so no column is a singleton:
  // x0 <= x1 <= x2. Probing x0=1 must wake the second row family and
  // retain the transitive conditional bound x2>=1.
  for (int r = 0; r < 2; ++r) {
    lp.A.insert(r, 0) = 1.0;
    lp.A.insert(r, 1) = -1.0;
    lp.A.insert(2 + r, 1) = 1.0;
    lp.A.insert(2 + r, 2) = -1.0;
  }
  for (int k = 0; k < kUnrelatedCols; ++k) {
    const int col = kChainCols + k;
    lp.A.insert(kChainRows + 2 * k, col) = 1.0;
    lp.A.insert(kChainRows + 2 * k + 1, col) = 1.0;
  }
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(kRows);
  lp.b.head(kChainRows).setZero();
  lp.Aeq.resize(0, kCols);
  lp.beq.resize(0);
  lp.vars.assign(kCols, {VarType::Continuous, 0.0, 1.0});
  for (int j = 0; j < kChainCols; ++j)
    lp.vars[j].type = VarType::Binary;

  PresolveOptions options;
  options.max_rounds = 1;
  options.max_probing_candidates = kChainCols;
  options.probing_depth = 3;
  options.do_singleton_rows = false;
  options.do_forcing_rows = false;
  options.do_doubleton = false;
  options.do_parallel_rows = false;
  options.do_dominated = false;

  std::vector<int> binary{0, 1, 2};
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  INFO(stats.infeasibility_reason);
  REQUIRE_FALSE(stats.infeasible);
  CHECK(stats.rounds == 1);
  CHECK(stats.activity_rows_recomputed == kRows);
  CHECK(stats.probing_trail_pushes > 0);
  CHECK(stats.probing_rows_processed > 0);
  CHECK(stats.probing_max_touched_cols <= kChainCols);
  CHECK(stats.probing_max_touched_cols * 10 < kCols);
  CHECK(stats.probing_implications_learned > 0);
  REQUIRE(lp.vars.size() == kCols);

  bool found_transitive_implication = false;
  for (const auto& implication : presolve.probing_implications()) {
    if (implication.trigger_col == 0 && implication.trigger_value_one &&
        implication.implied_col == 2 && implication.implied_is_lb &&
        implication.implied_value >= 1.0 - 1e-9) {
      found_transitive_implication = true;
      break;
    }
  }
  CHECK(found_transitive_implication);
}

TEST_CASE("Native MILP probing publishes the feasible world after a conflict",
          "[presolve][milp][probing][rollback]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(3);
  lp.A.resize(3, 3);
  // x0=1 implies x1<=0 and x1>=1, so only x0=0 is feasible. In that
  // surviving world, x0+x2>=1 additionally implies x2=1.
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.insert(1, 0) = 1.0;
  lp.A.insert(1, 1) = -1.0;
  lp.A.insert(2, 0) = -1.0;
  lp.A.insert(2, 2) = -1.0;
  lp.A.makeCompressed();
  lp.b.resize(3);
  lp.b << 1.0, 0.0, -1.0;
  lp.Aeq.resize(0, 3);
  lp.beq.resize(0);
  lp.vars.assign(3, {VarType::Binary, 0.0, 1.0});

  PresolveOptions options;
  options.max_rounds = 1;
  options.max_probing_candidates = 1;
  options.probing_depth = 3;
  options.do_singleton_rows = false;
  options.do_forcing_rows = false;
  options.do_doubleton = false;
  options.do_parallel_rows = false;
  options.do_dominated = false;

  std::vector<int> binary{0, 1, 2};
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  INFO(stats.infeasibility_reason);
  REQUIRE_FALSE(stats.infeasible);
  CHECK(stats.probing_fixings == 1);
  CHECK(stats.activity_rows_recomputed == 3);
  CHECK(presolve.orig_to_reduced_col()[0] == -1);
  CHECK(presolve.orig_to_reduced_col()[2] == -1);
  REQUIRE(lp.vars.size() == 1);

  Eigen::VectorXd reduced(1);
  reduced[0] = 0.25;
  const Eigen::VectorXd original = presolve.postsolve(reduced);
  REQUIRE(original.size() == 3);
  CHECK(original[0] == Approx(0.0));
  CHECK(original[1] == Approx(0.25));
  CHECK(original[2] == Approx(1.0));
}

TEST_CASE("Native MILP probing rolls back a truncated world",
          "[presolve][milp][probing][budget]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(2, 2);
  for (int r = 0; r < 2; ++r) {
    lp.A.insert(r, 0) = 1.0;
    lp.A.insert(r, 1) = -1.0;
  }
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Zero(2);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.assign(2, {VarType::Binary, 0.0, 1.0});

  PresolveOptions options;
  options.max_rounds = 1;
  options.max_probing_candidates = 2;
  options.max_probing_row_visits = 1;
  options.max_probing_implications = 10;
  std::vector<int> binary{0, 1};
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  REQUIRE_FALSE(stats.infeasible);
  CHECK(stats.probing_truncated);
  CHECK(stats.probing_rows_processed <= 1);
  CHECK(stats.probing_implications_learned == 0);
  REQUIRE(lp.vars.size() == 2);
  CHECK(lp.vars[0].lb == Approx(0.0));
  CHECK(lp.vars[0].ub == Approx(1.0));
  CHECK(lp.vars[1].lb == Approx(0.0));
  CHECK(lp.vars[1].ub == Approx(1.0));
}

TEST_CASE("Native MILP probing remaps implications after column elimination",
          "[presolve][milp][probing][mapping]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(4);
  lp.A.resize(4, 4);
  // Original column 0 is fixed and removed before probing. The surviving
  // chain x1<=x2<=x3 must therefore be published as reduced indices 0..2.
  for (int r = 0; r < 2; ++r) {
    lp.A.insert(r, 1) = 1.0;
    lp.A.insert(r, 2) = -1.0;
    lp.A.insert(2 + r, 2) = 1.0;
    lp.A.insert(2 + r, 3) = -1.0;
  }
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Zero(4);
  lp.Aeq.resize(0, 4);
  lp.beq.resize(0);
  lp.vars.assign(4, {VarType::Binary, 0.0, 1.0});
  lp.vars[0].lb = 0.0;
  lp.vars[0].ub = 0.0;

  PresolveOptions options;
  options.max_rounds = 1;
  options.max_probing_candidates = 3;
  options.probing_depth = 3;
  options.do_singleton_rows = false;
  options.do_forcing_rows = false;
  options.do_doubleton = false;
  options.do_parallel_rows = false;
  options.do_dominated = false;

  std::vector<int> binary{0, 1, 2, 3};
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  INFO(stats.infeasibility_reason);
  REQUIRE_FALSE(stats.infeasible);
  REQUIRE(lp.vars.size() == 3);
  CHECK(presolve.orig_to_reduced_col()[0] == -1);
  CHECK(presolve.orig_to_reduced_col()[1] == 0);
  CHECK(presolve.orig_to_reduced_col()[2] == 1);
  CHECK(presolve.orig_to_reduced_col()[3] == 2);

  bool found_remapped_implication = false;
  for (const auto& implication : presolve.probing_implications()) {
    if (implication.trigger_col == 0 && implication.trigger_value_one &&
        implication.implied_col == 2 && implication.implied_is_lb &&
        implication.implied_value >= 1.0 - 1e-9) {
      found_remapped_implication = true;
      break;
    }
  }
  CHECK(found_remapped_implication);
}

TEST_CASE("Native MILP singleton substitution preserves integer divisibility",
          "[presolve][milp][integrality]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(0, 2);
  lp.b.resize(0);
  lp.Aeq.resize(1, 2);
  lp.Aeq.insert(0, 0) = 2.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Ones(1);
  lp.vars = {{VarType::Integer, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1.0}};

  PresolveOptions options;
  options.max_rounds = 1;
  options.do_singleton_rows = false;
  options.do_singleton_columns = true;
  options.do_forcing_rows = false;
  options.do_probing = false;

  std::vector<int> binary;
  std::vector<int> integer{0};
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  REQUIRE_FALSE(stats.infeasible);
  REQUIRE(lp.vars.empty());
  CHECK(stats.singletons_removed == 1);
  const Eigen::VectorXd original =
      presolve.postsolve(Eigen::VectorXd::Zero(0));
  REQUIRE(original.size() == 2);
  CHECK(original[0] == Approx(0.0));
  CHECK(original[1] == Approx(1.0));
}

TEST_CASE("Native MILP doubleton substitution preserves integer intercepts",
          "[presolve][milp][integrality][doubleton]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(0, 2);
  lp.b.resize(0);
  lp.Aeq.resize(1, 2);
  lp.Aeq.insert(0, 0) = 2.0;
  lp.Aeq.insert(0, 1) = -2.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Ones(1);
  lp.vars = {{VarType::Integer, -100.0, 100.0},
             {VarType::Integer, -100.0, 100.0}};

  PresolveOptions options;
  options.max_rounds = 1;
  options.do_doubleton = true;
  options.do_probing = false;

  std::vector<int> binary;
  std::vector<int> integer{0, 1};
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  REQUIRE_FALSE(stats.infeasible);
  CHECK(stats.doubletons_removed == 0);
  REQUIRE(lp.vars.size() == 2);
  REQUIRE(lp.Aeq.rows() == 1);
  CHECK(lp.Aeq.coeff(0, 0) == Approx(2.0));
  CHECK(lp.Aeq.coeff(0, 1) == Approx(-2.0));
  CHECK(lp.beq[0] == Approx(1.0));
}

TEST_CASE("Native MILP parallel row scaling multiplies by the coefficient ratio",
          "[presolve][milp][parallel]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(2, 2);
  lp.A.insert(0, 0) = 2.0;
  lp.A.insert(0, 1) = 2.0;
  lp.A.insert(1, 0) = 1.0;
  lp.A.insert(1, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 3.0, 2.0;
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars.assign(2, {VarType::Continuous, 0.0, 10.0});

  PresolveOptions options;
  options.max_rounds = 1;
  options.do_singleton_rows = false;
  options.do_forcing_rows = false;
  options.do_probing = false;
  options.do_parallel_rows = true;

  std::vector<int> binary;
  std::vector<int> integer;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  REQUIRE_FALSE(stats.infeasible);
  REQUIRE(lp.A.rows() == 1);
  REQUIRE(lp.b.size() == 1);
  // 2*x0+2*x1<=3 is tighter than x0+x1<=2.
  CHECK(lp.b[0] == Approx(3.0));
  CHECK(lp.A.coeff(0, 0) == Approx(2.0));
  CHECK(lp.A.coeff(0, 1) == Approx(2.0));
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
