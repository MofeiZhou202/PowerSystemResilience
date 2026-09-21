/// test_lp_presolve.cpp
/// Tests for the native LP presolve.
/// P0: identity skeleton + env/telemetry contract.
/// P1: zero-fill rules (A&A 1995 §2.1-2.3) + postsolve stack roundtrip.
/// P2: equality substitution (A&A 1995 §2.4) + primal certificate undo.
/// Design: docs/archive/native_presolve_lp_2026-08-18.md §4.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cstdlib>
#include <cstring>
#include <limits>
#include <random>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include "mipsolvers/engine/presolve/lp_presolve.hpp"
#include "mipsolvers/engine/problem_types.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

void set_env(const char* name, const char* value) {
#ifdef _WIN32
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif
}

void unset_env(const char* name) {
#ifdef _WIN32
  _putenv_s(name, "");
#else
  unsetenv(name);
#endif
}

/// Small LP: 2 inequality rows (one ranged), 1 equality row, 3 columns.
LPModel make_small_lp() {
  LPModel lp;
  lp.c.resize(3);
  lp.c << 1.0, -2.0, 0.5;
  lp.A.resize(2, 3);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 2) = -1.0;
  lp.A.insert(1, 1) = 2.0;
  lp.A.makeCompressed();
  lp.row_lhs.resize(2);
  lp.row_lhs << -kInf, -1.0;
  lp.b.resize(2);
  lp.b << 4.0, 3.0;
  lp.Aeq.resize(1, 3);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq.resize(1);
  lp.beq << 2.0;
  lp.vars = {{VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, -5.0, 5.0},
             {VarType::Continuous, 0.0, 1e20}};
  return lp;
}

/// Blank n-column LP with all-free columns; fill c/rows/vars per test.
LPModel make_blank_lp(int n) {
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(n);
  lp.vars.assign(static_cast<std::size_t>(n), VariableMeta{});
  return lp;
}

void set_ineq_rows(LPModel& lp,
                   const std::vector<Eigen::Triplet<double>>& trips,
                   const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) {
  lp.A.resize(lhs.size(), lp.c.size());
  lp.A.setFromTriplets(trips.begin(), trips.end());
  lp.A.makeCompressed();
  lp.row_lhs = lhs;
  lp.b = rhs;
}

LpPresolveConfig enabled_cfg() {
  LpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.substitutions = false;
  cfg.propagate_bounds = false;
  return cfg;
}

LpPresolveConfig p2_cfg() {
  LpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.substitutions = true;
  cfg.propagate_bounds = false;
  return cfg;
}

LpPresolveConfig p3_cfg() {
  LpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.substitutions = false;
  cfg.propagate_bounds = true;
  return cfg;
}

}  // namespace

TEST_CASE("Native LP presolve reports disabled without touching statistics",
          "[presolve][lp][p0]") {
  const LPModel lp = make_small_lp();
  const LpPresolveResult res = lp_presolve_run(lp, LpPresolveConfig{});
  CHECK(res.status == "disabled");
  CHECK_FALSE(res.use_reduced);
  CHECK(res.orig_rows == 3);
  CHECK(res.orig_cols == 3);
  CHECK(res.orig_nnz == 5);
  // Reduced dimensions stay at the "did not run" sentinel.
  CHECK(res.reduced_rows == -1);
  CHECK(res.reduced_cols == -1);
  CHECK(res.reduced_nnz == -1);
}

TEST_CASE("lp_presolve_config_from_env honors the override convention",
          "[presolve][lp][p0][env]") {
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE");
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE");
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX");
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE_ADAPTIVE_STAGING");

  // No env: the base config passes through unchanged.
  LpPresolveConfig cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK_FALSE(cfg.enabled);
  CHECK(cfg.time_box_sec == Approx(2.0));
  CHECK_FALSE(cfg.verbose);
  CHECK_FALSE(cfg.adaptive_staging);

  // "0" disables, any other value enables.
  set_env("MIPSOLVERS_NATIVE_PRESOLVE", "0");
  cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK_FALSE(cfg.enabled);
  set_env("MIPSOLVERS_NATIVE_PRESOLVE", "1");
  cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK(cfg.enabled);

  // Verbose flag and time box parse.
  set_env("MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE", "1");
  set_env("MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX", "0.5");
  cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK(cfg.verbose);
  CHECK(cfg.time_box_sec == Approx(0.5));

  set_env("MIPSOLVERS_NATIVE_PRESOLVE_ADAPTIVE_STAGING", "1");
  cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK(cfg.adaptive_staging);
  set_env("MIPSOLVERS_NATIVE_PRESOLVE_ADAPTIVE_STAGING", "0");
  cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK_FALSE(cfg.adaptive_staging);

  // A non-numeric time box is ignored (same strtod guard as the HiGHS arm).
  set_env("MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX", "abc");
  cfg = lp_presolve_config_from_env(LpPresolveConfig{});
  CHECK(cfg.time_box_sec == Approx(2.0));

  unset_env("MIPSOLVERS_NATIVE_PRESOLVE");
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE_VERBOSE");
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE_TIME_BOX");
  unset_env("MIPSOLVERS_NATIVE_PRESOLVE_ADAPTIVE_STAGING");
}

TEST_CASE("Adaptive LP presolve skips small models before adjacency build",
          "[presolve][lp][adaptive]") {
  const LPModel lp = make_small_lp();
  LpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.substitutions = true;
  cfg.adaptive_staging = true;
  const LpPresolveResult res = lp_presolve_run(lp, cfg);
  CHECK(res.status == "small_model");
  CHECK_FALSE(res.use_reduced);
  CHECK(res.reduced_rows == res.orig_rows);
  CHECK(res.reduced_cols == res.orig_cols);
  CHECK(res.reduced_nnz == res.orig_nnz);
}

TEST_CASE("Adaptive LP presolve rejects saturated singleton-only P2 replay",
          "[presolve][lp][adaptive][p2-policy]") {
  constexpr int rows = 1000;
  constexpr int cols = 4000;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Ones(rows);
  lp.vars.assign(static_cast<std::size_t>(cols),
                 VariableMeta{VarType::Continuous, 0.0, 1.0});
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(cols);
  for (int col = 0; col < cols; ++col) {
    entries.emplace_back(col / 4, col, 1.0);
  }
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  LpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.substitutions = true;
  cfg.adaptive_staging = true;
  const LpPresolveResult res = lp_presolve_run(lp, cfg);
  CHECK(res.status == "no_reduction");
  CHECK_FALSE(res.use_reduced);
  CHECK(res.reduced_rows == rows);
  CHECK(res.reduced_cols == cols);
}

TEST_CASE("Adaptive LP presolve retains the intermediate singleton P2 cohort",
          "[presolve][lp][adaptive][p2-policy]") {
  constexpr int rows = 1001;
  constexpr int singleton_cols = 2002;
  constexpr int chain_cols = 998;
  constexpr int cols = singleton_cols + chain_cols;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Constant(rows, 2.0);
  lp.vars.assign(static_cast<std::size_t>(cols),
                 VariableMeta{VarType::Continuous, 0.0, 1.0});
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(singleton_cols + 2 * chain_cols);
  for (int row = 0; row < rows; ++row) {
    entries.emplace_back(row, 2 * row, 1.0);
    entries.emplace_back(row, 2 * row + 1, 1.0);
  }
  for (int k = 0; k < chain_cols; ++k) {
    const int col = singleton_cols + k;
    entries.emplace_back(k, col, 1.0);
    entries.emplace_back(k + 1, col, 1.0);
  }
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  LpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.substitutions = true;
  cfg.adaptive_staging = true;
  const LpPresolveResult res = lp_presolve_run(lp, cfg);
  CHECK(res.status == "reduced");
  CHECK(res.use_reduced);
  CHECK(res.reduced_rows < rows);
  CHECK(res.reduced_cols < cols);
}

TEST_CASE("Read-only P3 opportunity predicts direct projected fixings",
          "[presolve][lp][adaptive][opportunity]") {
  constexpr int rows = 1000;
  constexpr int cols = 4000;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Zero(rows);
  lp.vars.assign(static_cast<std::size_t>(cols),
                 VariableMeta{VarType::Continuous, 0.0, 1.0});
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(cols);
  for (int col = 0; col < cols; ++col)
    entries.emplace_back(col / 4, col, 1.0);
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  const LpPresolveOpportunityEstimate estimate =
      lp_presolve_estimate_adaptive_opportunity(lp);
  CHECK(estimate.valid);
  CHECK(estimate.should_run);
  CHECK(estimate.projected_fixed_cols == cols);
  CHECK(estimate.projectable_rows == rows);
  CHECK(estimate.structural_potential == Approx(1.0));
}

TEST_CASE("Read-only P3 opportunity rejects a nonforcing saturated pattern",
          "[presolve][lp][adaptive][opportunity]") {
  constexpr int rows = 1000;
  constexpr int cols = 4000;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Constant(rows, 2.0);
  lp.vars.assign(static_cast<std::size_t>(cols),
                 VariableMeta{VarType::Continuous, 0.0, 1.0});
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(cols);
  for (int col = 0; col < cols; ++col)
    entries.emplace_back(col / 4, col, 1.0);
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  const LpPresolveOpportunityEstimate estimate =
      lp_presolve_estimate_adaptive_opportunity(lp);
  CHECK(estimate.valid);
  CHECK_FALSE(estimate.should_run);
  CHECK(estimate.projected_fixed_cols == 0);
  CHECK(estimate.projectable_rows == 0);
  CHECK(estimate.singleton_equality_density == Approx(1.0));
}

TEST_CASE("Read-only P3 opportunity fails closed on activity overflow",
          "[presolve][lp][adaptive][opportunity]") {
  constexpr int cols = 4000;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(1, cols);
  lp.beq = Eigen::VectorXd::Zero(1);
  lp.vars.assign(static_cast<std::size_t>(cols),
                 VariableMeta{VarType::Continuous, 0.0, 10.0});
  std::vector<Eigen::Triplet<double>> entries;
  entries.emplace_back(0, 0, 1e308);
  entries.emplace_back(0, 1, 1e308);
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  const LpPresolveOpportunityEstimate estimate =
      lp_presolve_estimate_adaptive_opportunity(lp);
  CHECK_FALSE(estimate.valid);
  CHECK_FALSE(estimate.should_run);
}

TEST_CASE("Estimator activities feed incremental P3 across infinite bounds",
          "[presolve][lp][adaptive][opportunity][incremental-p3]") {
  constexpr int rows = 1000;
  constexpr int cols = 4000;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(rows, cols);
  lp.beq = Eigen::VectorXd::Zero(rows);
  lp.vars.resize(cols);
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(cols);
  for (int j = 0; j < cols; ++j) {
    const int row = j / 4;
    entries.emplace_back(row, j, 1.0);
    if ((row & 1) == 0) {
      lp.vars[static_cast<std::size_t>(j)] =
          VariableMeta{VarType::Continuous, 0.0, kVariableNoBound};
    } else {
      lp.vars[static_cast<std::size_t>(j)] =
          VariableMeta{VarType::Continuous, -kVariableNoBound, 0.0};
    }
  }
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  const LpPresolveOpportunityEstimate estimate =
      lp_presolve_estimate_adaptive_opportunity(lp);
  REQUIRE(estimate.valid);
  REQUIRE(estimate.activity_snapshot);

  LpPresolveConfig cfg = p3_cfg();
  const LpPresolveResult cold = lp_presolve_run(lp, cfg);
  const LpPresolveResult resident =
      lp_presolve_run(lp, cfg, estimate.activity_snapshot);
  CHECK_FALSE(cold.p3_activity_snapshot_used);
  CHECK(resident.p3_activity_snapshot_used);
  CHECK(resident.status == cold.status);
  CHECK(resident.infeasible == cold.infeasible);
  CHECK(resident.use_reduced == cold.use_reduced);
  CHECK(resident.reduced_rows == cold.reduced_rows);
  CHECK(resident.reduced_cols == cold.reduced_cols);
  CHECK(resident.reduced_nnz == cold.reduced_nnz);
  REQUIRE(resident.use_reduced);
  const Eigen::VectorXd x = postsolve_primal(
      resident, Eigen::VectorXd::Zero(resident.reduced_cols));
  REQUIRE(x.size() == cols);
  CHECK((lp.Aeq * x - lp.beq).lpNorm<Eigen::Infinity>() == Approx(0.0));
}

TEST_CASE("Incremental P3 rejects a stale matrix snapshot",
          "[presolve][lp][adaptive][opportunity][incremental-p3]") {
  constexpr int rows = 1000;
  constexpr int cols = 4000;
  auto make_lp = [](double first_coefficient) {
    LPModel lp;
    lp.c = Eigen::VectorXd::Zero(cols);
    lp.A.resize(0, cols);
    lp.b.resize(0);
    lp.row_lhs.resize(0);
    lp.Aeq.resize(rows, cols);
    lp.beq = Eigen::VectorXd::Zero(rows);
    lp.vars.assign(static_cast<std::size_t>(cols),
                   VariableMeta{VarType::Continuous, 0.0, 1.0});
    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(cols);
    for (int j = 0; j < cols; ++j)
      entries.emplace_back(j / 4, j, j == 0 ? first_coefficient : 1.0);
    lp.Aeq.setFromTriplets(entries.begin(), entries.end());
    lp.Aeq.makeCompressed();
    return lp;
  };

  const LPModel source = make_lp(1.0);
  const LPModel target = make_lp(2.0);
  const auto estimate = lp_presolve_estimate_adaptive_opportunity(source);
  REQUIRE(estimate.activity_snapshot);
  const LpPresolveResult cold = lp_presolve_run(target, p3_cfg());
  const LpPresolveResult rejected =
      lp_presolve_run(target, p3_cfg(), estimate.activity_snapshot);
  CHECK_FALSE(rejected.p3_activity_snapshot_used);
  CHECK(rejected.status == cold.status);
  CHECK(rejected.reduced_rows == cold.reduced_rows);
  CHECK(rejected.reduced_cols == cold.reduced_cols);
  CHECK(rejected.reduced_nnz == cold.reduced_nnz);
}

TEST_CASE("P2 commit invalidates resident P3 activities",
          "[presolve][lp][p2][incremental-p3]") {
  constexpr int cols = 3999;
  LPModel lp;
  lp.c = Eigen::VectorXd::Zero(cols);
  lp.A.resize(0, cols);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(1, cols);
  lp.beq = Eigen::VectorXd::Ones(1);
  lp.vars.assign(static_cast<std::size_t>(cols),
                 VariableMeta{VarType::Continuous, 0.0, 1.0});
  std::vector<Eigen::Triplet<double>> entries{{0, 0, 1.0}, {0, 1, 1.0}};
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();

  const auto estimate = lp_presolve_estimate_adaptive_opportunity(lp);
  REQUIRE(estimate.activity_snapshot);
  LpPresolveConfig cfg = p2_cfg();
  cfg.propagate_bounds = true;
  const LpPresolveResult cold = lp_presolve_run(lp, cfg);
  const LpPresolveResult resident =
      lp_presolve_run(lp, cfg, estimate.activity_snapshot);
  CHECK(resident.p3_activity_snapshot_used);
  CHECK(resident.status == cold.status);
  CHECK(resident.infeasible == cold.infeasible);
  CHECK(resident.reduced_rows == cold.reduced_rows);
  CHECK(resident.reduced_cols == cold.reduced_cols);
  CHECK(resident.reduced_nnz == cold.reduced_nnz);
}

TEST_CASE("Resident P1 degrees follow row and column deletion cascades",
          "[presolve][lp][p1][resident-degree]") {
  LPModel lp = make_blank_lp(3);
  lp.c = Eigen::VectorXd::Zero(3);
  lp.vars = {{VarType::Continuous, 0.0, 2.0},
             {VarType::Continuous, 0.0, 2.0},
             {VarType::Continuous, 0.0, 2.0}};
  lp.A.resize(0, 3);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(3, 3);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 1.0}, {1, 0, 1.0}, {1, 1, 1.0},
      {2, 1, 1.0}, {2, 2, 1.0}};
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(3, 1.0);
  lp.beq[1] = 2.0;
  lp.beq[2] = 2.0;

  LpPresolveConfig cfg = enabled_cfg();
  cfg.propagate_bounds = false;
  cfg.substitutions = false;
  const LpPresolveResult res = lp_presolve_run(lp, cfg);
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 0);
  CHECK(res.reduced_cols == 0);
  CHECK(res.reduced_nnz == 0);

  const Eigen::VectorXd x = postsolve_primal(res, Eigen::VectorXd());
  REQUIRE(x.size() == 3);
  CHECK((lp.Aeq * x - lp.beq).lpNorm<Eigen::Infinity>() == Approx(0.0));
  CHECK(x[0] == Approx(1.0));
  CHECK(x[1] == Approx(1.0));
  CHECK(x[2] == Approx(1.0));
}

TEST_CASE("P1 dirty queues revisit noncontiguous incidence cascades",
          "[presolve][lp][p1][dirty-queue]") {
  LPModel lp = make_blank_lp(5);
  lp.c = Eigen::VectorXd::Zero(5);
  lp.vars.assign(5, VariableMeta{VarType::Continuous, 0.0, 2.0});
  lp.A.resize(0, 5);
  lp.b.resize(0);
  lp.row_lhs.resize(0);
  lp.Aeq.resize(4, 5);
  std::vector<Eigen::Triplet<double>> entries{
      {0, 0, 1.0}, {0, 4, 1.0}, {3, 0, 1.0}};
  lp.Aeq.setFromTriplets(entries.begin(), entries.end());
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Zero(4);
  lp.beq[0] = 2.0;
  lp.beq[3] = 1.0;

  LpPresolveConfig cfg = enabled_cfg();
  cfg.propagate_bounds = true;
  cfg.substitutions = false;
  const LpPresolveResult res = lp_presolve_run(lp, cfg);
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 0);
  CHECK(res.reduced_cols == 0);
  CHECK(res.reduced_nnz == 0);

  const Eigen::VectorXd x = postsolve_primal(res, Eigen::VectorXd());
  REQUIRE(x.size() == 5);
  CHECK((lp.Aeq * x - lp.beq).lpNorm<Eigen::Infinity>() == Approx(0.0));
  CHECK(x[0] == Approx(1.0));
  CHECK(x[4] == Approx(1.0));
}

TEST_CASE("P1 reduces the small LP via the singleton-row rule (A&A §2.3)",
          "[presolve][lp][p1]") {
  const LPModel lp = make_small_lp();
  // Snapshot of the input; the pass takes a const reference and must leave
  // every byte of the model untouched.
  const Eigen::VectorXd c0 = lp.c;
  const Eigen::VectorXd b0 = lp.b;
  const Eigen::VectorXd lhs0 = lp.row_lhs;
  const Eigen::VectorXd beq0 = lp.beq;
  const Eigen::SparseMatrix<double> a0 = lp.A;
  const Eigen::SparseMatrix<double> aeq0 = lp.Aeq;

  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());

  // Row 1 (-1 <= 2 x1 <= 3) is a singleton: it tightens x1 to [-0.5, 1.5]
  // and is deleted (A&A §2.3, eq. (2.4)).  Nothing else reduces.
  CHECK(res.status == "reduced");
  CHECK(res.use_reduced);
  CHECK_FALSE(res.infeasible);
  CHECK_FALSE(res.unbounded_candidate);
  CHECK(res.orig_rows == 3);
  CHECK(res.orig_cols == 3);
  CHECK(res.orig_nnz == 5);
  CHECK(res.reduced_rows == 2);
  CHECK(res.reduced_cols == 3);
  CHECK(res.reduced_nnz == 4);
  CHECK(res.presolve_ms >= 0.0);
  CHECK(res.postsolve_stack.empty());  // no column was fixed
  CHECK(res.objective_offset == Approx(0.0));

  // Reduced-model output convention: exact equality rows go back to Aeq
  // (the IPM slack form cannot carry zero-width slacks); all other rows in
  // A with double bounds.  Here the surviving equality x0 + x1 = 2 sits in
  // Aeq and the inequality row0 in A.
  CHECK(res.reduced.Aeq.rows() == 1);
  CHECK(res.reduced.A.rows() == 1);
  CHECK(res.reduced.beq[0] == Approx(2.0));
  CHECK(res.reduced.row_lhs.size() == res.reduced.A.rows());
  CHECK(res.reduced.vars[1].lb == Approx(-0.5));
  CHECK(res.reduced.vars[1].ub == Approx(1.5));

  // Postsolve with no deleted columns is the identity through the map.
  const Eigen::VectorXd x_red = (Eigen::VectorXd(3) << 2.5, -0.5, 1.0).finished();
  const Eigen::VectorXd x_orig = postsolve_primal(res, x_red);
  REQUIRE(x_orig.size() == 3);
  CHECK((x_orig - x_red).norm() == Approx(0.0));

  CHECK(lp.c == c0);
  CHECK(lp.b == b0);
  CHECK(lp.row_lhs == lhs0);
  CHECK(lp.beq == beq0);
  CHECK(lp.A.nonZeros() == a0.nonZeros());
  CHECK(lp.Aeq.nonZeros() == aeq0.nonZeros());
  CHECK((lp.A - a0).norm() == Approx(0.0));
  CHECK((lp.Aeq - aeq0).norm() == Approx(0.0));
}

TEST_CASE("P1 empty row: feasible row deleted, infeasible row proves "
          "infeasibility (A&A §2.1)",
          "[presolve][lp][p1][empty-row]") {
  SECTION("feasible empty row is deleted") {
    LPModel lp = make_blank_lp(1);
    lp.c << 1.0;
    lp.vars = {{VarType::Continuous, 0.0, 1.0}};
    // Row has no entries; 0 in [-1, 1] -> deleted.  The column then has
    // degree 0 and positive cost -> fixed to its lower bound (A&A §2.2).
    set_ineq_rows(lp, {}, (Eigen::VectorXd(1) << -1.0).finished(),
                  (Eigen::VectorXd(1) << 1.0).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.status == "reduced");
    CHECK(res.use_reduced);
    CHECK(res.reduced_rows == 0);
    CHECK(res.reduced_cols == 0);
    REQUIRE(res.postsolve_stack.size() == 1);
    const auto* rec = std::get_if<LpPostsolveFixedCol>(&res.postsolve_stack[0]);
    REQUIRE(rec != nullptr);
    CHECK(rec->orig_col == 0);
    CHECK(rec->value == Approx(0.0));
    // Postsolve of the empty reduced primal recovers the fixed value.
    const Eigen::VectorXd x = postsolve_primal(res, Eigen::VectorXd());
    REQUIRE(x.size() == 1);
    CHECK(x[0] == Approx(0.0));
  }
  SECTION("infeasible empty row is detected outside the tolerance") {
    LPModel lp = make_blank_lp(1);
    lp.c << 1.0;
    lp.vars = {{VarType::Continuous, 0.0, 1.0}};
    // 0 <= -1 is false: infeasible (A&A §2.1).
    set_ineq_rows(lp, {}, (Eigen::VectorXd(1) << -kInf).finished(),
                  (Eigen::VectorXd(1) << -1.0).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.infeasible);
    CHECK(res.status == "infeasible");
    CHECK_FALSE(res.use_reduced);
  }
}

TEST_CASE("P1 empty column: zero cost fixes to a bound, nonzero cost without "
          "a bound in the improving direction flags an unbounded candidate "
          "(A&A §2.2)",
          "[presolve][lp][p1][empty-col]") {
  SECTION("zero-cost empty column fixed to lower bound") {
    LPModel lp = make_blank_lp(2);
    lp.c << 1.0, 0.0;
    lp.vars = {{VarType::Continuous, 0.0, 10.0},
               {VarType::Continuous, 3.0, 7.0}};
    // Column 1 has no entries anywhere.
    set_ineq_rows(lp, {Eigen::Triplet<double>(0, 0, 1.0)},
                  (Eigen::VectorXd(1) << -kInf).finished(),
                  (Eigen::VectorXd(1) << 5.0).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.use_reduced);
    // row0 is a singleton on x0 (tightens x0 to [-inf,5], deleted); both
    // columns then have degree 0 and are fixed: x0 to lb (c=1>0), x1 to 3.
    CHECK(res.reduced_cols == 0);
    bool found = false;
    for (const auto& entry : res.postsolve_stack) {
      const auto* rec = std::get_if<LpPostsolveFixedCol>(&entry);
      if (rec && rec->orig_col == 1) {
        found = true;
        CHECK(rec->value == Approx(3.0));  // lower bound chosen
      }
    }
    CHECK(found);
  }
  SECTION("positive cost with no lower bound is an unbounded candidate") {
    LPModel lp = make_blank_lp(1);
    lp.c << 1.0;  // min +x, x free below
    lp.vars = {{VarType::Continuous, -kInf, 5.0}};
    // No rows at all: the column is empty from the start.
    lp.A.resize(0, 1);
    lp.b.resize(0);
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.unbounded_candidate);
    CHECK(res.status == "unbounded_candidate");
    // Presolve never concludes: no reduced model is published (§4 P1).
    CHECK_FALSE(res.use_reduced);
    CHECK_FALSE(res.infeasible);
  }
  SECTION("negative cost with no upper bound is an unbounded candidate") {
    LPModel lp = make_blank_lp(1);
    lp.c << -2.0;  // min -2x, x free above
    lp.vars = {{VarType::Continuous, 0.0, kInf}};
    lp.A.resize(0, 1);
    lp.b.resize(0);
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.unbounded_candidate);
    CHECK_FALSE(res.use_reduced);
  }
}

TEST_CASE("P1 fixed column substitution shifts row sides and the objective "
          "offset exactly (A&A §2.2)",
          "[presolve][lp][p1][fixed-col]") {
  LPModel lp = make_blank_lp(3);
  lp.c << 5.0, 0.0, 0.0;
  lp.vars = {{VarType::Continuous, 2.0, 2.0},   // fixed
             {VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, 0.0, 10.0}};
  // row0: 3 x0 + x1 + x2 <= 10  ->  x1 + x2 <= 4 after substitution
  // row1: x1 + x2 >= 1          (keeps both columns alive)
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 3.0),
                 Eigen::Triplet<double>(0, 1, 1.0),
                 Eigen::Triplet<double>(0, 2, 1.0),
                 Eigen::Triplet<double>(1, 1, 1.0),
                 Eigen::Triplet<double>(1, 2, 1.0)},
                (Eigen::VectorXd(2) << -kInf, 1.0).finished(),
                (Eigen::VectorXd(2) << 10.0, kInf).finished());
  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 2);
  CHECK(res.reduced_cols == 2);
  // Substituted row sides are exact: rhs 10 - 3*2 = 4, lhs side untouched.
  CHECK(res.reduced.b[0] == Approx(4.0));
  CHECK(res.reduced.row_lhs[0] == -kInf);
  CHECK(res.reduced.row_lhs[1] == Approx(1.0));
  // Objective offset is c0 * 2 = 10 in the original (min) convention.
  CHECK(res.objective_offset == Approx(10.0));
  REQUIRE(res.postsolve_stack.size() == 1);
  const auto* rec = std::get_if<LpPostsolveFixedCol>(&res.postsolve_stack[0]);
  REQUIRE(rec != nullptr);
  CHECK(rec->orig_col == 0);
  CHECK(rec->value == Approx(2.0));
  // Postsolve restores the fixed column.
  const Eigen::VectorXd x_red = (Eigen::VectorXd(2) << 0.5, 0.5).finished();
  const Eigen::VectorXd x = postsolve_primal(res, x_red);
  REQUIRE(x.size() == 3);
  CHECK(x[0] == Approx(2.0));
  CHECK(x[1] == Approx(0.5));
  CHECK(x[2] == Approx(0.5));
  // c'x == c_red'x_red + offset.
  CHECK(lp.c.dot(x) ==
        Approx(res.reduced.c.dot(x_red) + res.objective_offset));
}

TEST_CASE("Presolve compaction publishes CSC-consistent resident CSR",
          "[presolve][lp][matrix-workspace]") {
  LPModel lp = make_blank_lp(5);
  lp.c = Eigen::VectorXd::Zero(5);
  lp.vars = {{VarType::Continuous, 1.0, 1.0},
             {VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, -10.0, 10.0},
             {VarType::Continuous, -10.0, 10.0}};
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 2.0),
                 Eigen::Triplet<double>(0, 1, 3.0),
                 Eigen::Triplet<double>(0, 2, -1.0),
                 Eigen::Triplet<double>(1, 1, 1.0),
                 Eigen::Triplet<double>(1, 2, 2.0)},
                (Eigen::VectorXd(2) << -kInf, 0.5).finished(),
                (Eigen::VectorXd(2) << 20.0, kInf).finished());
  lp.Aeq.resize(1, 5);
  lp.Aeq.insert(0, 0) = -4.0;
  lp.Aeq.insert(0, 3) = 2.0;
  lp.Aeq.insert(0, 4) = 5.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(1, 7.0);

  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  REQUIRE(res.use_reduced);
  REQUIRE(res.matrix_workspace);
  const auto& ws = *res.matrix_workspace;
  CHECK(ws.inequality_rows == res.reduced.A.rows());
  CHECK(ws.equality_rows == res.reduced.Aeq.rows());
  CHECK(ws.cols == res.reduced.c.size());

  auto check_block = [](const Eigen::SparseMatrix<double>& matrix,
                        const std::vector<int>& row_start,
                        const std::vector<int>& col,
                        const std::vector<double>& value,
                        const std::vector<int>& csc_to_csr) {
    REQUIRE(row_start.size() ==
            static_cast<std::size_t>(matrix.rows()) + 1);
    REQUIRE(col.size() == static_cast<std::size_t>(matrix.nonZeros()));
    REQUIRE(value.size() == col.size());
    REQUIRE(csc_to_csr.size() == col.size());
    for (int i = 0; i < matrix.rows(); ++i) {
      for (int p = row_start[static_cast<std::size_t>(i)];
           p < row_start[static_cast<std::size_t>(i) + 1]; ++p) {
        CHECK(matrix.coeff(i, col[static_cast<std::size_t>(p)]) ==
              Approx(value[static_cast<std::size_t>(p)]));
      }
    }
    for (int j = 0; j < matrix.cols(); ++j) {
      for (int p = matrix.outerIndexPtr()[j];
           p < matrix.outerIndexPtr()[j + 1]; ++p) {
        const int q = csc_to_csr[static_cast<std::size_t>(p)];
        CHECK(col[static_cast<std::size_t>(q)] == j);
        CHECK(value[static_cast<std::size_t>(q)] ==
              Approx(matrix.valuePtr()[p]));
      }
    }
  };
  check_block(res.reduced.A, ws.a_row_start, ws.a_col, ws.a_value,
              ws.a_csc_to_csr);
  check_block(res.reduced.Aeq, ws.aeq_row_start, ws.aeq_col, ws.aeq_value,
              ws.aeq_csc_to_csr);
}

TEST_CASE("P1 redundant row: activity inside the sides deletes, outside "
          "proves infeasibility (A&A §2.3)",
          "[presolve][lp][p1][redundant-row]") {
  SECTION("activity range within sides -> row deleted") {
    LPModel lp = make_blank_lp(2);
    lp.c << 1.0, 1.0;
    lp.vars = {{VarType::Continuous, 0.0, 1.0},
               {VarType::Continuous, 0.0, 1.0}};
    // row0: x0 + x1 <= 10 is implied by the box (max activity 2).
    // row1: x0 + x1 >= 0.5 is not redundant and keeps the columns alive.
    set_ineq_rows(lp,
                  {Eigen::Triplet<double>(0, 0, 1.0),
                   Eigen::Triplet<double>(0, 1, 1.0),
                   Eigen::Triplet<double>(1, 0, 1.0),
                   Eigen::Triplet<double>(1, 1, 1.0)},
                  (Eigen::VectorXd(2) << -kInf, 0.5).finished(),
                  (Eigen::VectorXd(2) << 10.0, kInf).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    REQUIRE(res.use_reduced);
    CHECK(res.reduced_rows == 1);
    CHECK(res.reduced_cols == 2);
    CHECK(res.reduced.row_lhs[0] == Approx(0.5));
  }
  SECTION("activity range exceeding the sides -> row kept, no reduction") {
    LPModel lp = make_blank_lp(2);
    lp.c << 1.0, 1.0;
    lp.vars = {{VarType::Continuous, 0.0, 10.0},
               {VarType::Continuous, 0.0, 10.0}};
    // x0 + x1 <= 10 is not implied by the box (max activity 20).
    set_ineq_rows(lp,
                  {Eigen::Triplet<double>(0, 0, 1.0),
                   Eigen::Triplet<double>(0, 1, 1.0)},
                  (Eigen::VectorXd(1) << -kInf).finished(),
                  (Eigen::VectorXd(1) << 10.0).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.status == "no_reduction");
    CHECK_FALSE(res.use_reduced);
  }
  SECTION("max activity below the lower side -> infeasible") {
    LPModel lp = make_blank_lp(2);
    lp.c << 1.0, 1.0;
    lp.vars = {{VarType::Continuous, 0.0, 1.0},
               {VarType::Continuous, 0.0, 1.0}};
    // x0 + x1 >= 20 but the largest attainable activity is 2.
    set_ineq_rows(lp,
                  {Eigen::Triplet<double>(0, 0, 1.0),
                   Eigen::Triplet<double>(0, 1, 1.0)},
                  (Eigen::VectorXd(1) << 20.0).finished(),
                  (Eigen::VectorXd(1) << kInf).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.infeasible);
    CHECK_FALSE(res.use_reduced);
  }
}

TEST_CASE("P1 singleton row implied bounds, all four sign/side combinations "
          "(A&A §2.3 eq. (2.4))",
          "[presolve][lp][p1][singleton-row]") {
  auto run_case = [](double a, double lhs, double rhs, double expect_lb,
                     double expect_ub) {
    LPModel lp = make_blank_lp(2);
    lp.c << 0.0, 0.0;
    lp.vars = {{VarType::Continuous, -10.0, 10.0},
               {VarType::Continuous, 0.0, 1000.0}};
    // row0: a*x0 in [lhs, rhs] (singleton, tightens x0);
    // row1: x0 + x1 <= 100 keeps x0 alive (not redundant: max activity is
    // 10 + 1000 before tightening and stays above 100 after it).
    set_ineq_rows(lp,
                  {Eigen::Triplet<double>(0, 0, a),
                   Eigen::Triplet<double>(1, 0, 1.0),
                   Eigen::Triplet<double>(1, 1, 1.0)},
                  (Eigen::VectorXd(2) << lhs, -kInf).finished(),
                  (Eigen::VectorXd(2) << rhs, 100.0).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    REQUIRE(res.use_reduced);
    REQUIRE(res.reduced_cols == 2);
    CHECK(res.reduced.vars[0].lb == Approx(expect_lb));
    CHECK(res.reduced.vars[0].ub == Approx(expect_ub));
  };
  SECTION("a > 0, upper side") { run_case(2.0, -kInf, 6.0, -10.0, 3.0); }
  SECTION("a > 0, lower side") { run_case(2.0, 4.0, kInf, 2.0, 10.0); }
  SECTION("a < 0, upper side") { run_case(-2.0, -kInf, 6.0, -3.0, 10.0); }
  SECTION("a < 0, lower side") { run_case(-2.0, 4.0, kInf, -10.0, -2.0); }
  SECTION("implied bound conflicting with the box -> infeasible") {
    LPModel lp = make_blank_lp(1);
    lp.c << 0.0;
    lp.vars = {{VarType::Continuous, 0.0, 1.0}};
    // 2 x0 >= 4 implies x0 >= 2, contradicting x0 <= 1 (A&A §2.3).
    set_ineq_rows(lp, {Eigen::Triplet<double>(0, 0, 2.0)},
                  (Eigen::VectorXd(1) << 4.0).finished(),
                  (Eigen::VectorXd(1) << kInf).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.infeasible);
    CHECK_FALSE(res.use_reduced);
  }
}

TEST_CASE("P1 fixpoint chains: fixed col -> singleton row -> empty col",
          "[presolve][lp][p1][fixpoint]") {
  LPModel lp = make_blank_lp(3);
  lp.c << 1.0, 1.0, 1.0;
  lp.vars = {{VarType::Continuous, 1.0, 1.0},   // fixed
             {VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, 0.0, 10.0}};
  // eq: x0 + x1 = 3  -> x1 forced to 2 once x0 = 1 substitutes in
  // row: x1 + x2 <= 10 -> x2 <= 8 once x1 = 2 propagates
  lp.Aeq.resize(1, 3);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq.resize(1);
  lp.beq << 3.0;
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 1, 1.0),
                 Eigen::Triplet<double>(0, 2, 1.0)},
                (Eigen::VectorXd(1) << -kInf).finished(),
                (Eigen::VectorXd(1) << 10.0).finished());
  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 0);
  CHECK(res.reduced_cols == 0);
  // The whole model reduced to fixed values: x = (1, 2, 0).
  const Eigen::VectorXd x = postsolve_primal(res, Eigen::VectorXd());
  REQUIRE(x.size() == 3);
  CHECK(x[0] == Approx(1.0));
  CHECK(x[1] == Approx(2.0));  // recovered via tightened bound, not via x0
  CHECK(x[2] == Approx(0.0));  // positive cost, lower bound
  // offset = 1*1 + 1*2 + 1*0 = 3 in the original convention.
  CHECK(res.objective_offset == Approx(3.0));
  CHECK(lp.c.dot(x) == Approx(res.objective_offset));
}

TEST_CASE("P1 maximize sense: offset is reported in the original sense "
          "convention",
          "[presolve][lp][p1][sense]") {
  LPModel lp = make_blank_lp(1);
  lp.sense = Sense::Maximize;
  lp.c << 2.0;
  lp.vars = {{VarType::Continuous, 3.0, 3.0}};  // fixed
  set_ineq_rows(lp, {Eigen::Triplet<double>(0, 0, 1.0)},
                (Eigen::VectorXd(1) << -kInf).finished(),
                (Eigen::VectorXd(1) << 5.0).finished());
  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_cols == 0);
  // max 2x with x = 3: the original-convention offset is +6, not -6.
  CHECK(res.objective_offset == Approx(6.0));
  CHECK(res.reduced.sense == Sense::Maximize);
  const Eigen::VectorXd x = postsolve_primal(res, Eigen::VectorXd());
  REQUIRE(x.size() == 1);
  CHECK(x[0] == Approx(3.0));
}

TEST_CASE("P1 random LP postsolve roundtrip (20 seeded instances)",
          "[presolve][lp][p1][roundtrip]") {
  // Construction: pick x* first, then build rows whose sides bracket the
  // row activity at x*, so x* is feasible by construction.  Column cost
  // signs are chosen against the missing bounds (a column missing its lower
  // bound gets c <= 0, etc.) so an emptied column can never flag an
  // unbounded candidate and the roundtrip always applies.
  for (int seed = 0; seed < 20; ++seed) {
    std::mt19937 rng(static_cast<unsigned>(1234 + seed));
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    const int n = 8 + static_cast<int>(rng() % 13);       // 8..20 columns
    const int m_ineq = 4 + static_cast<int>(rng() % 10);  // inequality rows
    const int m_eq = 1 + static_cast<int>(rng() % 4);     // equality rows

    Eigen::VectorXd x_star(n);
    for (int j = 0; j < n; ++j) x_star[j] = -5.0 + 10.0 * uni(rng);

    LPModel lp = make_blank_lp(n);
    for (int j = 0; j < n; ++j) {
      auto& v = lp.vars[static_cast<std::size_t>(j)];
      const double r1 = 0.1 + 5.0 * uni(rng);
      const double r2 = 0.1 + 5.0 * uni(rng);
      const int kind = static_cast<int>(rng() % 5);
      if (kind == 0) {  // fixed column
        v.lb = x_star[j];
        v.ub = x_star[j];
      } else if (kind == 1) {  // no lower bound
        v.lb = -kInf;
        v.ub = x_star[j] + r2;
        lp.c[j] = -uni(rng);  // c <= 0 pushes to the finite upper bound
      } else if (kind == 2) {  // no upper bound
        v.lb = x_star[j] - r1;
        v.ub = kInf;
        lp.c[j] = uni(rng);   // c >= 0 pushes to the finite lower bound
      } else if (kind == 3) {  // fully unbounded
        v.lb = -kInf;
        v.ub = kInf;
        lp.c[j] = 0.0;        // zero cost keeps any fixing optimal
      } else {                 // box
        v.lb = x_star[j] - r1;
        v.ub = x_star[j] + r2;
        lp.c[j] = -1.0 + 2.0 * uni(rng);
      }
    }

    std::vector<Eigen::Triplet<double>> trips;
    Eigen::VectorXd lhs(m_ineq), rhs(m_ineq);
    for (int i = 0; i < m_ineq; ++i) {
      double act = 0.0;
      const int entries = 1 + static_cast<int>(rng() % 4);
      for (int e = 0; e < entries; ++e) {
        const int j = static_cast<int>(rng() % n);
        double a = -3.0 + 6.0 * uni(rng);
        if (std::abs(a) < 0.1) a = a < 0.0 ? -0.5 : 0.5;
        trips.emplace_back(i, j, a);
        act += a * x_star[j];
      }
      const int rkind = static_cast<int>(rng() % 3);
      if (rkind == 0) {  // <= row
        lhs[i] = -kInf;
        rhs[i] = act + 5.0 * uni(rng);
      } else if (rkind == 1) {  // >= row
        lhs[i] = act - 5.0 * uni(rng);
        rhs[i] = kInf;
      } else {  // ranged row
        lhs[i] = act - 2.0 * uni(rng);
        rhs[i] = act + 2.0 * uni(rng);
      }
    }
    set_ineq_rows(lp, trips, lhs, rhs);

    std::vector<Eigen::Triplet<double>> eq_trips;
    lp.beq.resize(m_eq);
    for (int k = 0; k < m_eq; ++k) {
      double act = 0.0;
      const int entries = 1 + static_cast<int>(rng() % 3);
      for (int e = 0; e < entries; ++e) {
        const int j = static_cast<int>(rng() % n);
        double a = -3.0 + 6.0 * uni(rng);
        if (std::abs(a) < 0.1) a = a < 0.0 ? -0.5 : 0.5;
        eq_trips.emplace_back(k, j, a);
        act += a * x_star[j];
      }
      lp.beq[k] = act;
    }
    lp.Aeq.resize(m_eq, n);
    lp.Aeq.setFromTriplets(eq_trips.begin(), eq_trips.end());
    lp.Aeq.makeCompressed();

    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    INFO("seed ", seed, ": status ", res.status);
    CHECK_FALSE(res.infeasible);
    CHECK_FALSE(res.unbounded_candidate);
    if (!res.use_reduced) {
      // No rule fired on this instance; nothing to roundtrip.
      CHECK(res.status == "no_reduction");
      continue;
    }

    // x* restricted to the surviving columns is feasible for the reduced
    // model: deleted rows were certified against bounds that only tightened,
    // and fixed columns substituted values x* attains (up to the 1e-9 fixed
    // tolerance).
    Eigen::VectorXd x_red(res.reduced_cols);
    for (int j = 0; j < n; ++j) {
      const int r = res.orig_to_reduced_col[static_cast<std::size_t>(j)];
      if (r >= 0) x_red[r] = x_star[j];
    }
    const Eigen::VectorXd x = postsolve_primal(res, x_red);
    REQUIRE(x.size() == n);

    // Residual check in the original model (1e-6): all row sides, equality
    // rows, and column bounds.
    const Eigen::VectorXd ax = lp.A * x;
    for (int i = 0; i < ax.size(); ++i) {
      CHECK(ax[i] <= lp.b[i] + 1e-6);
      CHECK(ax[i] >= lp.row_lhs[i] - 1e-6);
    }
    const Eigen::VectorXd aeqx = lp.Aeq * x;
    for (int k = 0; k < aeqx.size(); ++k) {
      CHECK(std::abs(aeqx[k] - lp.beq[k]) <= 1e-6);
    }
    for (int j = 0; j < n; ++j) {
      const auto& v = lp.vars[static_cast<std::size_t>(j)];
      if (v.lb > -kVariableNoBound) CHECK(x[j] >= v.lb - 1e-6);
      if (v.ub < kVariableNoBound) CHECK(x[j] <= v.ub + 1e-6);
    }

    // Objective identity: c'x_orig == c_red'x_red + offset (1e-9 relative).
    const double obj_orig = lp.c.dot(x);
    const double obj_recon = res.reduced.c.dot(x_red) + res.objective_offset;
    CHECK(obj_orig ==
          Approx(obj_recon).epsilon(1e-9).margin(1e-9));
  }
}

TEST_CASE("postsolve_primal rejects contract violations loudly",
          "[presolve][lp][p1][contract]") {
  const LPModel lp = make_small_lp();
  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  REQUIRE(res.use_reduced);
  // Wrong reduced size -> empty vector, never a plausible solution.
  CHECK(postsolve_primal(res, Eigen::VectorXd::Ones(2)).size() == 0);
  CHECK(postsolve_primal(res, Eigen::VectorXd::Ones(4)).size() == 0);
  // use_reduced=false result -> empty vector.
  const LpPresolveResult disabled = lp_presolve_run(lp, LpPresolveConfig{});
  CHECK(postsolve_primal(disabled, Eigen::VectorXd()).size() == 0);
}

TEST_CASE("P1 fixed-column substitution is skipped when a shifted row side "
          "would leave the finite-side regime (1e19 sentinel guard)",
          "[presolve][lp][p1][sentinel]") {
  SECTION("finite shift crossing the sentinel keeps the column and the exact "
          "row sides") {
    LPModel lp = make_blank_lp(2);
    lp.c << 1.0, 1.0;
    lp.vars = {{VarType::Continuous, 1e18, 1e18},  // fixed
               {VarType::Continuous, 0.0, 1e12}};
    // row0: 9.9 x0 + x1 in [-9.9e18, 9.9e18 + 5e11].  Substituting
    // x0 = 1e18 (av = 9.9e18) would shift the lower side to -1.98e19,
    // beyond the 1e19 no-bound sentinel: the reduction must be skipped
    // entirely (the shifted side would read back as "no bound" in the
    // reduced model and in the residual audit).  The row itself is
    // non-redundant (its upper side binds: max activity 9.9e18+1e12
    // exceeds rhs) and survives the row sweep, so the guard — not the
    // activity test — is what keeps the substitution from running.
    // row1: x0 + x1 >= 1e18 + 5e11 is non-redundant and keeps both
    // columns alive (max activity attains the side, min activity does not).
    set_ineq_rows(lp,
                  {Eigen::Triplet<double>(0, 0, 9.9),
                   Eigen::Triplet<double>(0, 1, 1.0),
                   Eigen::Triplet<double>(1, 0, 1.0),
                   Eigen::Triplet<double>(1, 1, 1.0)},
                  (Eigen::VectorXd(2) << -9.9e18, 1e18 + 5e11).finished(),
                  (Eigen::VectorXd(2) << 9.9e18 + 5e11, kInf).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    // The substitution was skipped, so no rule fires at all: the model is
    // returned unreduced rather than with a sentinel-corrupted row side.
    CHECK(res.status == "no_reduction");
    CHECK_FALSE(res.use_reduced);
    CHECK(res.postsolve_stack.empty());
    CHECK(res.objective_offset == Approx(0.0));
  }
  SECTION("non-finite a*v (overflow to inf) skips the substitution") {
    LPModel lp = make_blank_lp(2);
    lp.c << 1.0, 1.0;
    lp.vars = {{VarType::Continuous, 1e18, 1e18},  // fixed
               {VarType::Continuous, 0.0, 1e12}};
    // row0: 1e291 x0 + x1 >= -1e18.  a*v = 1e309 overflows to +inf; the
    // substitution must be skipped rather than turning the lower side into
    // -inf (a silently dropped constraint).  The overflowed activity bounds
    // are treated as unknown, so the row is neither deleted nor declared
    // infeasible.
    // row1: x0 + x1 >= 1e18 + 1e12 is non-redundant and keeps both columns
    // alive (max activity attains the side, min activity does not).
    set_ineq_rows(lp,
                  {Eigen::Triplet<double>(0, 0, 1e291),
                   Eigen::Triplet<double>(0, 1, 1.0),
                   Eigen::Triplet<double>(1, 0, 1.0),
                   Eigen::Triplet<double>(1, 1, 1.0)},
                  (Eigen::VectorXd(2) << -1e18, 1e18 + 1e12).finished(),
                  (Eigen::VectorXd(2) << kInf, kInf).finished());
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    // No rule fires: the model is returned unreduced.
    CHECK(res.status == "no_reduction");
    CHECK_FALSE(res.use_reduced);
    CHECK(res.postsolve_stack.empty());
  }
}

TEST_CASE("P1 singleton implied bound beyond the 1e19 sentinel is not "
          "applied and the row is NOT deleted (no silent constraint loss)",
          "[presolve][lp][p1][sentinel]") {
  LPModel lp = make_blank_lp(1);
  lp.c << 1.0;
  lp.vars = {{VarType::Continuous, 0.0, kInf}};
  // 1e-5 x0 <= 9e18 implies x0 <= 9e23 — beyond the 1e19 sentinel, hence
  // unusable as a column bound.  Deleting the row anyway would drop the
  // constraint entirely, so the row must stay.
  set_ineq_rows(lp, {Eigen::Triplet<double>(0, 0, 1e-5)},
                (Eigen::VectorXd(1) << -kInf).finished(),
                (Eigen::VectorXd(1) << 9e18).finished());
  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  CHECK(res.status == "no_reduction");
  CHECK_FALSE(res.use_reduced);
  CHECK_FALSE(res.infeasible);
  CHECK(res.postsolve_stack.empty());
}

TEST_CASE("P1 activity accumulation overflow is treated as an unknown "
          "(unbounded) side: no false infeasibility, no deletion",
          "[presolve][lp][p1][overflow]") {
  LPModel lp = make_blank_lp(2);
  lp.c << 1.0, 1.0;
  lp.vars = {{VarType::Continuous, 1e19, 9e19},
             {VarType::Continuous, 1e19, 9e19}};
  // 1e290 x0 - 1e290 x1 in [-1e18, 1e18]: both activity accumulations
  // overflow to NaN (+inf and -inf terms).  The row must survive untouched;
  // the model is feasible (x0 = x1 attains activity 0).
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 1e290),
                 Eigen::Triplet<double>(0, 1, -1e290)},
                (Eigen::VectorXd(1) << -1e18).finished(),
                (Eigen::VectorXd(1) << 1e18).finished());
  const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
  CHECK_FALSE(res.infeasible);
  CHECK_FALSE(res.unbounded_candidate);
  // No rule may fire on unverifiable activity bounds.
  CHECK(res.status == "no_reduction");
  CHECK_FALSE(res.use_reduced);
}

TEST_CASE("lp_presolve_publication_tol_scale honors the audit-transfer "
          "invariant (scale <= 0.9 * scale_orig/scale_reduced, no "
          "gratuitous decade on zero-inflation models)",
          "[presolve][lp][p1][pubscale]") {
  // Zero inflation (modszk1/fffff800 shape: identical global side scales)
  // must publish essentially at the direct-solve tolerance — the previous
  // unconditional 0.1 factor demanded a decade tighter and stalled the
  // normal-equation path just above the 1e-8-relative target.
  {
    LPModel orig = make_small_lp();
    const LPModel& reduced = orig;
    CHECK(lp_presolve_publication_tol_scale(orig, reduced) == Approx(0.9));
  }
  // Reduced scale 100x the original (the kSideShiftCap boundary): the
  // scale tightens to 9e-3, still far above the 1e-4 floor, and stays
  // inside the invariant scale <= orig/reduced = 1e-2.
  {
    LPModel orig = make_small_lp();
    LPModel reduced = make_small_lp();
    reduced.beq[0] = 400.0;  // 100x the original max side (4.0)
    const double scale = lp_presolve_publication_tol_scale(orig, reduced);
    CHECK(scale == Approx(0.9e-2));
    CHECK(scale <= 1e-2);
  }
  // Reduced scale smaller than the original: the clamp caps at 1.0 (the
  // direct-solve tolerance); the invariant holds since ratio > 1.
  {
    LPModel orig = make_small_lp();
    LPModel reduced = make_small_lp();
    reduced.b[0] = 0.05;  // reduced max side 3.0 < original 4.0
    CHECK(lp_presolve_publication_tol_scale(orig, reduced) == Approx(1.0));
  }
  // Beyond-native-cap inflation (reachable through the HiGHS bridge) must not
  // be floored above the ratio: that would violate the transfer invariant.
  {
    LPModel orig = make_small_lp();
    LPModel reduced = make_small_lp();
    reduced.beq[0] = 4e8;  // ratio 1e-8 -> 0.9e-8
    const double scale = lp_presolve_publication_tol_scale(orig, reduced);
    CHECK(scale == Approx(0.9e-8));
    CHECK(scale <= 1e-8);
  }
  // Equality RHS values have no no-bound sentinel: even values above 1e19
  // remain part of the residual scale and therefore of the transfer ratio.
  {
    LPModel orig = make_small_lp();
    LPModel reduced = make_small_lp();
    reduced.beq[0] = 4e20;  // ratio 1e-20 -> 0.9e-20
    const double scale = lp_presolve_publication_tol_scale(orig, reduced);
    CHECK(scale == Approx(0.9e-20));
    CHECK(scale <= 1e-20);
  }
  // No-bound sentinels (|side| >= 1e19, +/-inf) never enter the scale, on
  // either side of the comparison.
  {
    LPModel orig = make_small_lp();
    orig.b[0] = 1e20;             // sentinel: no bound
    orig.row_lhs[0] = -kInf;
    LPModel reduced = orig;
    CHECK(lp_presolve_publication_tol_scale(orig, reduced) == Approx(0.9));
    // A model with no finite sides at all floors the scale at 1.
    LPModel bare = make_blank_lp(1);
    LPModel bare2 = make_blank_lp(1);
    CHECK(lp_presolve_publication_tol_scale(bare, bare2) == Approx(0.9));
  }
}

TEST_CASE("P1 meager-reduction gate: <1% row and column cuts are declined "
          "(negative expected value for the speculative reduced solve)",
          "[presolve][lp][p1][meager]") {
  // 200 one-sided rows, each coupling two consecutive columns (degree 2,
  // so no singleton-row or empty-column rule fires); one extra empty row.
  // The empty-row deletion removes 1/201 rows (<1%) and 0 columns, so the
  // reduction must be declined bit-identically (status + no stack + no
  // reduced model), exactly like "no_reduction".
  const int kRows = 200;
  auto make_lp = [&](int extra_empty_rows) {
    LPModel lp = make_blank_lp(kRows);
    for (int j = 0; j < kRows; ++j) lp.c[j] = 1.0;
    lp.vars.assign(static_cast<std::size_t>(kRows),
                   VariableMeta{VarType::Continuous, 0.0, 1.0});
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(2 * kRows);
    for (int i = 0; i < kRows; ++i) {
      trips.emplace_back(i, i, 1.0);
      trips.emplace_back(i, (i + 1) % kRows, 1.0);
    }
    Eigen::VectorXd lhs =
        Eigen::VectorXd::Constant(kRows + extra_empty_rows, -kInf);
    Eigen::VectorXd rhs = Eigen::VectorXd::Ones(kRows + extra_empty_rows);
    set_ineq_rows(lp, trips, lhs, rhs);
    return lp;
  };

  {
    LPModel lp = make_lp(1);  // 1/201 rows removable, 0% columns
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.status == "meager_reduction");
    CHECK_FALSE(res.use_reduced);
    CHECK_FALSE(res.infeasible);
    CHECK(res.postsolve_stack.empty());
    // Dimensions report what the (declined) reduction would have been.
    CHECK(res.reduced_rows == res.orig_rows - 1);
    CHECK(res.reduced_cols == res.orig_cols);
  }
  {
    // 4/204 rows (2.0%) removable: above the gate, the reduced model is
    // used normally.
    LPModel lp = make_lp(4);
    const LpPresolveResult res = lp_presolve_run(lp, enabled_cfg());
    CHECK(res.status == "reduced");
    CHECK(res.use_reduced);
    CHECK(res.reduced_rows == res.orig_rows - 4);
  }
}

TEST_CASE("P2 doubleton equation substitutes one column, transfers bounds, "
          "and postsolves the primal (A&A 1995 §2.4)",
          "[presolve][lp][p2][doubleton]") {
  LPModel lp = make_blank_lp(3);
  lp.c << 3.0, 4.0, -1.0;
  lp.vars = {{VarType::Continuous, 1.0, 3.0},
             {VarType::Continuous, -10.0, 10.0},
             {VarType::Continuous, -10.0, 10.0}};
  // 3*x0 + x1 = 7 -> x0 = (7-x1)/3.  Bounds 1 <= x0 <= 3 imply
  // -2 <= x1 <= 4.  Two inequalities keep both equation columns above
  // singleton degree so this specifically exercises the doubleton rule.
  lp.Aeq.resize(1, 3);
  lp.Aeq.insert(0, 0) = 3.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq = (Eigen::VectorXd(1) << 7.0).finished();
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 1, 1.0),
                 Eigen::Triplet<double>(0, 2, 1.0),
                 Eigen::Triplet<double>(1, 0, 1.0),
                 Eigen::Triplet<double>(1, 2, -1.0)},
                (Eigen::VectorXd(2) << -1.0, -2.0).finished(),
                (Eigen::VectorXd(2) << 4.0, 3.0).finished());

  const LpPresolveResult res = lp_presolve_run(lp, p2_cfg());
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 2);
  CHECK(res.reduced_cols == 2);
  CHECK(res.reduced.vars[0].lb == Approx(-2.0));
  CHECK(res.reduced.vars[0].ub == Approx(4.0));
  REQUIRE(res.postsolve_stack.size() == 1);
  REQUIRE(std::get_if<LpPostsolveDoubletonEquation>(
              &res.postsolve_stack.front()) != nullptr);

  const Eigen::VectorXd x_red =
      (Eigen::VectorXd(2) << 1.0, 0.0).finished();
  const Eigen::VectorXd x = postsolve_primal(res, x_red);
  REQUIRE(x.size() == 3);
  CHECK(x[0] == Approx(2.0));
  CHECK(x[1] == Approx(1.0));
  CHECK((lp.Aeq * x)[0] == Approx(lp.beq[0]));
  CHECK(lp.c.dot(x) ==
        Approx(res.reduced.c.dot(x_red) + res.objective_offset));
}

TEST_CASE("P2 free-column substitution creates bounded fill-in and restores "
          "the eliminated value (A&A 1995 §2.4)",
          "[presolve][lp][p2][free-column]") {
  LPModel lp = make_blank_lp(4);
  lp.c << 2.0, 1.0, -1.0, 0.5;
  lp.vars = {{VarType::Continuous, -kInf, kInf},
             {VarType::Continuous, -10.0, 10.0},
             {VarType::Continuous, -10.0, 10.0},
             {VarType::Continuous, -10.0, 10.0}};
  // x0 + 2*x1 - x2 = 6 eliminates the truly free x0.  In the second row,
  // x0 is replaced by 6 - 2*x1 + x2, creating one controlled fill-in.
  lp.Aeq.resize(1, 4);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 2.0;
  lp.Aeq.insert(0, 2) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq = (Eigen::VectorXd(1) << 6.0).finished();
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 1.0),
                 Eigen::Triplet<double>(0, 1, 1.0),
                 Eigen::Triplet<double>(0, 2, 1.0),
                 Eigen::Triplet<double>(0, 3, 1.0)},
                (Eigen::VectorXd(1) << -5.0).finished(),
                (Eigen::VectorXd(1) << 12.0).finished());

  const LpPresolveResult res = lp_presolve_run(lp, p2_cfg());
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 1);
  CHECK(res.reduced_cols == 3);
  CHECK(res.reduced_nnz == 3);
  REQUIRE(res.postsolve_stack.size() == 1);
  REQUIRE(std::get_if<LpPostsolveFreeColSubstitution>(
              &res.postsolve_stack.front()) != nullptr);

  const Eigen::VectorXd x_red =
      (Eigen::VectorXd(3) << 1.0, 2.0, 0.0).finished();
  const Eigen::VectorXd x = postsolve_primal(res, x_red);
  REQUIRE(x.size() == 4);
  CHECK(x[0] == Approx(6.0));
  CHECK((lp.Aeq * x)[0] == Approx(lp.beq[0]));
  CHECK(lp.c.dot(x) ==
        Approx(res.reduced.c.dot(x_red) + res.objective_offset));
}

TEST_CASE("P2 bounded singleton equality column projects its box to a ranged "
          "row and postsolves exactly (A&A 1995 §2.4)",
          "[presolve][lp][p2][singleton-column]") {
  LPModel lp = make_blank_lp(3);
  lp.c << 2.0, 1.0, -1.0;
  lp.vars = {{VarType::Continuous, 0.0, kInf},
             {VarType::Continuous, -10.0, 10.0},
             {VarType::Continuous, -10.0, 10.0}};
  // x0 + 2*x1 - x2 = 6 with x0 >= 0 projects to 2*x1-x2 <= 6.
  // x0 occurs nowhere else, while the second row keeps x1/x2 active.
  lp.Aeq.resize(1, 3);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 2.0;
  lp.Aeq.insert(0, 2) = -1.0;
  lp.Aeq.makeCompressed();
  lp.beq = (Eigen::VectorXd(1) << 6.0).finished();
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 1, 1.0),
                 Eigen::Triplet<double>(0, 2, 1.0)},
                (Eigen::VectorXd(1) << -3.0).finished(),
                (Eigen::VectorXd(1) << 5.0).finished());

  const LpPresolveResult res = lp_presolve_run(lp, p2_cfg());
  REQUIRE(res.use_reduced);
  CHECK(res.reduced_rows == 2);
  CHECK(res.reduced_cols == 2);
  REQUIRE(res.postsolve_stack.size() == 1);
  const auto* rec = std::get_if<LpPostsolveFreeColSubstitution>(
      &res.postsolve_stack.front());
  REQUIRE(rec != nullptr);
  CHECK(rec->col == 0);

  const Eigen::VectorXd x_red =
      (Eigen::VectorXd(2) << 1.0, 2.0).finished();
  const Eigen::VectorXd x = postsolve_primal(res, x_red);
  REQUIRE(x.size() == 3);
  CHECK(x[0] == Approx(6.0));
  CHECK((lp.Aeq * x)[0] == Approx(lp.beq[0]));
  CHECK(lp.c.dot(x) ==
        Approx(res.reduced.c.dot(x_red) + res.objective_offset));
}

TEST_CASE("P2 rejects a bounded column that is not implied free",
          "[presolve][lp][p2][implied-free][negative]") {
  LPModel lp = make_blank_lp(3);
  lp.c << 1.0, 1.0, 1.0;
  lp.vars = {{VarType::Continuous, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1.0},
             {VarType::Continuous, 0.0, 1.0}};
  lp.Aeq.resize(1, 3);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.insert(0, 2) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Zero(1);
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 1.0),
                 Eigen::Triplet<double>(0, 1, 1.0),
                 Eigen::Triplet<double>(0, 2, 1.0)},
                (Eigen::VectorXd(1) << -kInf).finished(),
                (Eigen::VectorXd(1) << 1.0).finished());

  const LpPresolveResult res = lp_presolve_run(lp, p2_cfg());
  CHECK(res.status == "no_reduction");
  CHECK_FALSE(res.use_reduced);
  CHECK(res.postsolve_stack.empty());
}

TEST_CASE("P3 row implied bounds handle coefficient signs and both row sides "
          "(A&A 1995 §3)",
          "[presolve][lp][p3][implied-bounds]") {
  LPModel lp = make_blank_lp(2);
  lp.c << 1.0, 1.0;
  lp.vars = {{VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, 1.0, 2.0}};
  // 4 <= 2*x0 - 3*x1 <= 10 and 1 <= x1 <= 2 imply
  // x0 >= (4 - max(-3*x1))/2 = 3.5 and
  // x0 <= (10 - min(-3*x1))/2 = 8.
  // The extra empty row makes the resulting bound-only reduction pass the
  // existing 1% speculative-use gate, without contributing any bound.
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 2.0),
                 Eigen::Triplet<double>(0, 1, -3.0)},
                (Eigen::VectorXd(2) << 4.0, -1.0).finished(),
                (Eigen::VectorXd(2) << 10.0, 1.0).finished());

  const LpPresolveResult res = lp_presolve_run(lp, p3_cfg());
  REQUIRE(res.use_reduced);
  REQUIRE(res.reduced_cols == 2);
  CHECK(res.reduced.vars[0].lb == Approx(3.5));
  CHECK(res.reduced.vars[0].ub == Approx(8.0));
}

TEST_CASE("P3 does not derive a bound from an unbounded residual activity",
          "[presolve][lp][p3][implied-bounds][infinite]") {
  LPModel lp = make_blank_lp(2);
  lp.c << 0.0, 0.0;
  lp.vars = {{VarType::Continuous, 0.0, 10.0},
             {VarType::Continuous, -kInf, 2.0}};
  // x0+x1 <= 5 cannot bound x0 above because x1 has no lower bound.
  set_ineq_rows(lp,
                {Eigen::Triplet<double>(0, 0, 1.0),
                 Eigen::Triplet<double>(0, 1, 1.0)},
                (Eigen::VectorXd(1) << -kInf).finished(),
                (Eigen::VectorXd(1) << 5.0).finished());

  const LpPresolveResult res = lp_presolve_run(lp, p3_cfg());
  CHECK(res.status == "no_reduction");
  CHECK_FALSE(res.use_reduced);
}
