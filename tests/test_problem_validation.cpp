/// test_problem_validation.cpp
/// Tests for the engine problem validation functions.
#include <catch2/catch_test_macros.hpp>

#include "mipsolvers/engine/util/problem_validation.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;

// ─── SparseLinSys validation ──────────────────────────────────────────────────
TEST_CASE("validate SparseLinSys: well-formed 2x2 system", "[validation][le]") {
  SparseLinSys sys;
  sys.A.resize(2, 2);
  sys.A.insert(0, 0) = 2.0;
  sys.A.insert(0, 1) = 1.0;
  sys.A.insert(1, 0) = 1.0;
  sys.A.insert(1, 1) = 3.0;
  sys.A.makeCompressed();
  sys.b.resize(2);
  sys.b << 4.0, 5.0;

  auto rep = validate(sys);
  CHECK(rep.valid);
  CHECK(rep.errors.empty());
}

TEST_CASE("validate SparseLinSys: size mismatch yields error", "[validation][le]") {
  SparseLinSys sys;
  sys.A.resize(3, 2);
  sys.b.resize(2);  // wrong: should be 3
  auto rep = validate(sys);
  CHECK_FALSE(rep.valid);
  CHECK_FALSE(rep.errors.empty());
}

// ─── LPModel validation ───────────────────────────────────────────────────────
TEST_CASE("validate LPModel: trivial 1-var LP passes", "[validation][lp]") {
  LPModel lp;
  lp.c.resize(1);
  lp.c << -1.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0, "x"});

  auto rep = validate(lp);
  CHECK(rep.valid);
}

TEST_CASE("validate LPModel: mismatched A column count yields error", "[validation][lp]") {
  LPModel lp;
  lp.c.resize(2);
  lp.c << -1.0, -2.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  // A with wrong number of columns
  Eigen::SparseMatrix<double> A(1, 3);  // 3 columns but c has 2
  A.insert(0, 0) = 1.0;
  A.insert(0, 1) = 1.0;
  A.insert(0, 2) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);
  lp.b << 1.0;

  auto rep = validate(lp);
  CHECK_FALSE(rep.valid);
}

TEST_CASE("validate LPModel: b size mismatch yields error", "[validation][lp]") {
  LPModel lp;
  lp.c.resize(2);
  lp.c << 1.0, 1.0;
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});
  lp.vars.push_back({VarType::Continuous, 0.0, 1e20});

  Eigen::SparseMatrix<double> A(2, 2);
  A.insert(0, 0) = 1.0;
  A.insert(1, 1) = 1.0;
  A.makeCompressed();
  lp.A = A;
  lp.b.resize(1);  // wrong: should be 2
  lp.b << 1.0;

  auto rep = validate(lp);
  CHECK_FALSE(rep.valid);
}

// ─── MIPModel validation ──────────────────────────────────────────────────────
TEST_CASE("validate MIPModel: single binary variable passes", "[validation][milp]") {
  MIPModel mip;
  mip.linear_part.c.resize(1);
  mip.linear_part.c << -1.0;
  mip.linear_part.vars.push_back({VarType::Binary, 0.0, 1.0, "b"});
  mip.binary_idx = {0};

  auto rep = validate(mip);
  CHECK(rep.valid);
}
