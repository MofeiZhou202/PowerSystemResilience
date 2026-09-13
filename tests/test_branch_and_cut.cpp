/// test_branch_and_cut.cpp
/// Integration tests for native branch-and-cut internals that need
/// failure-injection seams (env hooks) rather than the public engine API.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <algorithm>
#include <limits>
#include <optional>
#include <string>
#include <thread>

#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/detail/bc_conformance_trace.hpp"
#include "mipsolvers/engine/detail/bc_clique_table.hpp"
#include "mipsolvers/engine/detail/bc_domain.hpp"
#include "mipsolvers/engine/detail/bc_domain_probe.hpp"
#include "mipsolvers/engine/detail/bc_env_options.hpp"
#include "mipsolvers/engine/detail/bc_fallback.hpp"
#include "mipsolvers/engine/detail/bc_highs_style_numerics.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_numerics.hpp"
#include "mipsolvers/engine/detail/bc_objective_propagation.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/native/milp/bc/milp_presolve.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

namespace {

void set_env_value(const char* name, const char* value) {
#ifdef _WIN32
  ::_putenv_s(name, value);
#else
  ::setenv(name, value, 1);
#endif
}

void unset_env_value(const char* name) {
#ifdef _WIN32
  ::_putenv_s(name, "");
#else
  ::unsetenv(name);
#endif
}

/// Sets an environment variable for the lifetime of the guard, restoring
/// "unset" on destruction (the seams below are opt-in flags, never set in a
/// normal environment).
struct EnvVarGuard {
  std::string name;
  std::optional<std::string> previous;
  EnvVarGuard(const char* n, const char* v) : name(n) {
    if (const char* old = ::getenv(n)) previous = old;
    set_env_value(n, v);
  }
  ~EnvVarGuard() {
    if (previous) {
      set_env_value(name.c_str(), previous->c_str());
    } else {
      unset_env_value(name.c_str());
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

TEST_CASE("B&C: explicit-bound feasibility rejects malformed vectors",
          "[bc][feasibility][dimensions]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(0, 1);
  lp.b.resize(0);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars = {{VarType::Continuous, 0.0, 1.0}};

  const Eigen::VectorXd bounds_lb = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd bounds_ub = Eigen::VectorXd::Ones(1);
  Eigen::VectorXd x = Eigen::VectorXd::Constant(1, 0.5);
  CHECK(detail::satisfies_with_bounds(lp, x, bounds_lb, bounds_ub, 1e-6));
  CHECK_FALSE(detail::satisfies_with_bounds(
      lp, Eigen::VectorXd{}, bounds_lb, bounds_ub, 1e-6));
  CHECK_FALSE(detail::satisfies_with_bounds(
      lp, x, Eigen::VectorXd{}, bounds_ub, 1e-6));
  x[0] = std::numeric_limits<double>::quiet_NaN();
  CHECK_FALSE(detail::satisfies_with_bounds(lp, x, bounds_lb, bounds_ub,
                                             1e-6));
}

TEST_CASE("B&C: sparse cut rows append without rebuilding prior entries",
          "[bc][cuts][matrix_append]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(4);
  lp.A.resize(2, 4);
  lp.A.insert(0, 0) = 2.0;
  lp.A.insert(0, 3) = -1.0;
  lp.A.insert(1, 1) = 3.0;
  lp.A.makeCompressed();
  lp.b.resize(2);
  lp.b << 5.0, 7.0;
  lp.row_lhs.resize(2);
  lp.row_lhs << -2.0, 1.0;
  lp.Aeq.resize(0, 4);
  lp.beq.resize(0);
  lp.vars.assign(4, VariableMeta{VarType::Continuous, 0.0, 10.0});

  auto sparse_row = [](int size,
                       std::initializer_list<std::pair<int, double>> terms) {
    Eigen::SparseVector<double> row(size);
    row.reserve(static_cast<int>(terms.size()));
    for (const auto& [col, value] : terms) row.insertBack(col) = value;
    return row;
  };

  std::vector<Eigen::SparseVector<double>> first_rows;
  first_rows.push_back(sparse_row(4, {{0, 4.0}, {2, -2.0}}));
  first_rows.push_back(sparse_row(3, {{0, 1.0}}));
  first_rows.push_back(sparse_row(4, {{1, 1.0}}));
  first_rows.push_back(sparse_row(4, {}));
  first_rows.push_back(sparse_row(
      4, {{3, std::numeric_limits<double>::infinity()}}));
  const std::vector<double> first_rhs = {
      11.0, 12.0, std::numeric_limits<double>::quiet_NaN(), 13.0, 14.0};

  detail::SeparatorStorageStats storage;
  detail::add_sparse_rows_to_lp(lp, first_rows, first_rhs, &storage);

  REQUIRE(lp.A.rows() == 3);
  CHECK_FALSE(lp.A.isCompressed());
  CHECK(lp.A.coeff(0, 0) == Approx(2.0));
  CHECK(lp.A.coeff(0, 3) == Approx(-1.0));
  CHECK(lp.A.coeff(1, 1) == Approx(3.0));
  CHECK(lp.A.coeff(2, 0) == Approx(4.0));
  CHECK(lp.A.coeff(2, 2) == Approx(-2.0));
  CHECK(lp.b[0] == Approx(5.0));
  CHECK(lp.b[1] == Approx(7.0));
  CHECK(lp.b[2] == Approx(11.0));
  CHECK(lp.row_lhs[0] == Approx(-2.0));
  CHECK(lp.row_lhs[1] == Approx(1.0));
  CHECK(std::isinf(lp.row_lhs[2]));
  CHECK(lp.row_lhs[2] < 0.0);
  CHECK(storage.matrix_append_calls == 1);
  CHECK(storage.matrix_appended_rows == 1);
  CHECK(storage.matrix_appended_entries == 2);
  CHECK(storage.matrix_prior_entries_bypassing_triplet_rebuild == 3);

  constexpr int kRepeatedAppends = 32;
  std::uint64_t expected_prior_entries = 3;
  int expected_entries = 5;
  for (int i = 0; i < kRepeatedAppends; ++i) {
    const int col = i % 4;
    const double value = 20.0 + static_cast<double>(i);
    const std::vector<Eigen::SparseVector<double>> rows = {
        sparse_row(4, {{col, value}})};
    const std::vector<double> rhs = {100.0 + static_cast<double>(i)};
    expected_prior_entries += static_cast<std::uint64_t>(expected_entries);
    detail::add_sparse_rows_to_lp(lp, rows, rhs, &storage);
    ++expected_entries;
    CHECK(lp.A.coeff(3 + i, col) == Approx(value));
    CHECK(lp.b[3 + i] == Approx(rhs[0]));
    CHECK(std::isinf(lp.row_lhs[3 + i]));
    CHECK(lp.row_lhs[3 + i] < 0.0);
  }

  CHECK(lp.A.rows() == 3 + kRepeatedAppends);
  CHECK(lp.A.nonZeros() == expected_entries);
  CHECK(storage.matrix_append_calls == 1 + kRepeatedAppends);
  CHECK(storage.matrix_appended_rows == 1 + kRepeatedAppends);
  CHECK(storage.matrix_appended_entries == 2 + kRepeatedAppends);
  CHECK(storage.matrix_prior_entries_bypassing_triplet_rebuild ==
        expected_prior_entries);
  CHECK(storage.matrix_storage_reallocations < storage.matrix_append_calls);
  CHECK(storage.matrix_peak_spare_entries > 0);
}

TEST_CASE("B&C: clique-table insertions touch only endpoint adjacency",
          "[bc][clique_table][incremental]") {
  LPModel lp;
  constexpr int kCols = 2048;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(kCols);
  lp.A.resize(0, kCols);
  lp.b.resize(0);
  lp.Aeq.resize(0, kCols);
  lp.beq.resize(0);
  lp.vars.assign(kCols, {VarType::Binary, 0.0, 1.0});

  detail::CliqueTable table;
  CHECK(table.build(lp) == 0);
  CHECK(table.add_edges(lp, {{0, 1}}) == 1);
  CHECK(table.n_edges() == 1);
  CHECK(table.n_literal_edges() == 1);
  CHECK(table.has_edge(0, 1));
  CHECK(table.has_literal_edge(0, true, 1, true));

  const auto untouched_variable_range = table.neighbours(0);
  const auto untouched_literal_range = table.literal_neighbours(0, true);
  REQUIRE(untouched_variable_range.first != nullptr);
  REQUIRE(untouched_literal_range.first != nullptr);

  CHECK(table.add_literal_edges(
            lp, {{{2, false}, {3, true}}, {{4, true}, {5, true}}}) == 2);
  CHECK(table.n_edges() == 2);
  CHECK(table.n_literal_edges() == 3);
  CHECK(table.has_literal_edge(2, false, 3, true));
  CHECK(table.has_edge(4, 5));

  // Updating unrelated endpoints must not rebuild every adjacency list.
  CHECK(table.neighbours(0).first == untouched_variable_range.first);
  CHECK(table.literal_neighbours(0, true).first ==
        untouched_literal_range.first);

  CHECK(table.add_edges(lp, {{1, 0}, {0, 1}}) == 0);
  CHECK(table.add_literal_edges(lp, {{{3, true}, {2, false}}}) == 0);
  CHECK(table.n_edges() == 2);
  CHECK(table.n_literal_edges() == 3);

  std::vector<std::pair<int, int>> batch;
  batch.reserve(4 * (kCols - 6));
  for (int j = 6; j < kCols; ++j) {
    batch.emplace_back(2, j);
    batch.emplace_back(j, 2);
    batch.emplace_back(2, j);
    batch.emplace_back(j, 2);
  }
  CHECK(table.add_edges(lp, batch) == static_cast<std::size_t>(kCols - 6));
  CHECK(table.degree(2) == kCols - 6);
  CHECK(table.degree(kCols - 1) == 1);
  CHECK(table.has_edge(2, kCols - 1));
  CHECK(table.has_literal_edge(2, true, kCols - 1, true));
  CHECK(table.add_edges(lp, batch) == 0);
}

TEST_CASE("B&C: domain restore rolls back infeasible activity deltas",
          "[bc][domain][restore]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(1);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  detail::BCDomain domain;
  const Eigen::VectorXd lb = Eigen::VectorXd::Zero(2);
  const Eigen::VectorXd ub = Eigen::VectorXd::Ones(2);
  domain.init(lp, lb, ub);
  REQUIRE(domain.propagate());
  CHECK(domain.propagation_complete());
  CHECK(domain.rows_processed() == 1);

  const detail::BCDomain::Savepoint sp = domain.savepoint();
  CHECK(domain.trail_size() == sp.trail_size);
  REQUIRE(domain.fix_col(0, 1.0));
  REQUIRE(domain.trail_size() == sp.trail_size + 1);
  CHECK(domain.trail_entry(sp.trail_size).col == 0);
  CHECK_FALSE(domain.fix_col(1, 1.0));
  CHECK(domain.infeasible());
  CHECK(domain.trail_size() == sp.trail_size + 2);
  CHECK(domain.trail_entry(sp.trail_size + 1).col == 1);

  domain.restore(sp);
  CHECK_FALSE(domain.infeasible());
  CHECK(domain.trail_size() == sp.trail_size);
  CHECK(domain.lb()[0] == Approx(0.0));
  CHECK(domain.lb()[1] == Approx(0.0));
  REQUIRE(domain.propagate());
  CHECK(domain.propagation_complete());

  REQUIRE(domain.fix_col(0, 1.0));
  REQUIRE(domain.propagate());
  CHECK(domain.ub()[1] == Approx(0.0));
}

TEST_CASE("B&C: domain restore rolls back infeasible upper-bound deltas",
          "[bc][domain][restore]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(
      1, std::numeric_limits<double>::infinity());
  lp.row_lhs = Eigen::VectorXd::Ones(1);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  detail::BCDomain domain;
  const Eigen::VectorXd lb = Eigen::VectorXd::Zero(2);
  const Eigen::VectorXd ub = Eigen::VectorXd::Ones(2);
  domain.init(lp, lb, ub);
  REQUIRE(domain.propagate());

  const detail::BCDomain::Savepoint sp = domain.savepoint();
  REQUIRE(domain.fix_col(0, 0.0));
  CHECK_FALSE(domain.fix_col(1, 0.0));
  CHECK(domain.infeasible());

  domain.restore(sp);
  CHECK_FALSE(domain.infeasible());
  CHECK(domain.ub()[0] == Approx(1.0));
  CHECK(domain.ub()[1] == Approx(1.0));
  REQUIRE(domain.fix_col(0, 0.0));
  REQUIRE(domain.propagate());
  CHECK(domain.lb()[1] == Approx(1.0));
}

TEST_CASE("B&C: integer preferences project through evolving domain",
          "[bc][domain][repair_projection]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(
      1, std::numeric_limits<double>::infinity());
  lp.row_lhs = Eigen::VectorXd::Ones(1);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  detail::BCDomain domain;
  domain.init(lp, Eigen::VectorXd::Zero(2), Eigen::VectorXd::Ones(2),
              detail::BCDomain::InitialPropagation::AlreadyClosed);
  CHECK(domain.propagation_complete());
  CHECK(domain.rows_processed() == 0);
  REQUIRE(domain.propagate());
  const detail::BCDomain::Savepoint sp = domain.savepoint();

  // x0=0 implies x1=1. Sequential projection must clamp the later stale
  // x1=0 preference to that propagated value. See the root-quality derivation.
  const Eigen::VectorXd preference = Eigen::VectorXd::Zero(2);
  REQUIRE(domain.project_integer_preferences({0, 1}, preference));
  CHECK(domain.lb()[0] == Approx(0.0));
  CHECK(domain.ub()[0] == Approx(0.0));
  CHECK(domain.lb()[1] == Approx(1.0));
  CHECK(domain.ub()[1] == Approx(1.0));

  domain.restore(sp);
  CHECK_FALSE(domain.infeasible());
  CHECK(domain.lb()[0] == Approx(0.0));
  CHECK(domain.ub()[0] == Approx(1.0));
  CHECK(domain.lb()[1] == Approx(0.0));
  CHECK(domain.ub()[1] == Approx(1.0));
  REQUIRE(domain.propagate());
}

TEST_CASE("B&C: domain probing exports and rolls back only trailed columns",
          "[bc][domain][probing]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(3);
  lp.A.resize(1, 3);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(1);
  lp.Aeq.resize(0, 3);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  detail::CliqueTable clique_table;
  REQUIRE(clique_table.add_literal_edges(
              lp, {{{1, false}, {2, false}}}) == 1);

  const Eigen::VectorXd lb = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd ub = Eigen::VectorXd::Ones(3);
  detail::BCDomainProbeWorkspace workspace(lp, lb, ub, &clique_table, 1e-9);
  std::vector<detail::BCDomainProbeDelta> baseline;
  REQUIRE(workspace.initialize(baseline) ==
          detail::BCDomainProbeStatus::Feasible);
  CHECK(baseline.empty());

  const auto outcome = workspace.probe(0, true);
  REQUIRE(outcome.status == detail::BCDomainProbeStatus::Feasible);
  REQUIRE(outcome.deltas.size() == 3);
  CHECK(outcome.deltas[0].col == 0);
  CHECK(outcome.deltas[1].col == 1);
  CHECK(outcome.deltas[2].col == 2);
  CHECK(outcome.deltas[1].new_ub == Approx(0.0));
  CHECK(outcome.deltas[2].new_lb == Approx(1.0));
  CHECK(workspace.lb().isApprox(lb));
  CHECK(workspace.ub().isApprox(ub));

  const auto& telemetry = workspace.telemetry();
  CHECK(telemetry.workspace_initializations == 1);
  CHECK(telemetry.worlds == 1);
  CHECK(telemetry.trail_pushes == 3);
  CHECK(telemetry.changed_columns == 3);
  CHECK(telemetry.rollbacks == 1);
  CHECK(telemetry.failures == 0);
  CHECK(telemetry.rows_processed > 0);

  std::vector<detail::BCDomainProbeDelta> committed;
  REQUIRE(workspace.commit(0, true, committed) ==
          detail::BCDomainProbeStatus::Feasible);
  CHECK(committed.size() == 3);
  CHECK(workspace.lb()[0] == Approx(1.0));
  CHECK(workspace.ub()[1] == Approx(0.0));
  CHECK(workspace.lb()[2] == Approx(1.0));
}

TEST_CASE("B&C: compensated row activity survives catastrophic cancellation",
          "[bc][domain][numerics]") {
  detail::StableActivitySum activity;
  activity.add_product(1e16, 1.0);
  activity.add_product(1.0, 1.0);
  activity.add_product(-1e16, 1.0);
  CHECK(activity.value() == Approx(1.0));

  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(4);
  lp.A.resize(1, 4);
  lp.A.insert(0, 0) = 1e16;
  lp.A.insert(0, 1) = 1.0;
  lp.A.insert(0, 2) = -1e16;
  lp.A.insert(0, 3) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(1);
  lp.Aeq.resize(0, 4);
  lp.beq.resize(0);
  lp.vars = {{VarType::Continuous, 1.0, 1.0},
             {VarType::Continuous, 1.0, 1.0},
             {VarType::Continuous, 1.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  Eigen::VectorXd lb(4), ub(4);
  lb << 1.0, 1.0, 1.0, 0.0;
  ub << 1.0, 1.0, 1.0, 1.0;
  detail::BCDomain domain;
  domain.init(lp, lb, ub);
  REQUIRE(domain.propagate());
  CHECK(domain.ub()[3] == Approx(0.0));
}

TEST_CASE("B&C: environment snapshot is immutable and shared across workers",
          "[bc][environment][reproducibility]") {
  EnvVarGuard guard("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "before");
  const auto snapshot = capture_bc_env_options();
  REQUIRE(snapshot);
  REQUIRE(snapshot->value("MIPSOLVERS_TEST_SNAPSHOT_VALUE") != nullptr);
  CHECK(std::string(snapshot->value("MIPSOLVERS_TEST_SNAPSHOT_VALUE")) ==
        "before");

  set_env_value("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "after");
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

  set_env_value("MIPSOLVERS_TEST_SNAPSHOT_VALUE", "next-solve");
  const BCResult next_result = solve_milp_bc(make_knapsack_10(), opt);
  REQUIRE(next_result.stats.success);
  CHECK(std::find(next_result.effective_environment.begin(),
                  next_result.effective_environment.end(),
                  "MIPSOLVERS_TEST_SNAPSHOT_VALUE=next-solve") !=
        next_result.effective_environment.end());
}

TEST_CASE("B&C: bound events retain audited user-space incumbents",
          "[bc][progress][audit]") {
  const MIPModel mip = make_knapsack_10();
  BCOptions opt;
  opt.num_threads = 1;
  opt.enable_lns = false;
  const BCResult result = solve_milp_bc(mip, opt);

  REQUIRE(result.stats.success);
  REQUIRE(result.bc_stats.bound_event_stream_available);
  REQUIRE_FALSE(result.bc_stats.bound_events.empty());

  double previous_time = 0.0;
  int primal_events = 0;
  Eigen::VectorXd lb(mip.linear_part.vars.size());
  Eigen::VectorXd ub(mip.linear_part.vars.size());
  for (int j = 0; j < lb.size(); ++j) {
    lb[j] = mip.linear_part.vars[static_cast<std::size_t>(j)].lb;
    ub[j] = mip.linear_part.vars[static_cast<std::size_t>(j)].ub;
  }
  for (const auto& event : result.bc_stats.bound_events) {
    CHECK(event.time_sec >= previous_time);
    CHECK(event.time_sec <= result.bc_stats.runtime_sec + 1e-9);
    previous_time = event.time_sec;
    if (!std::isfinite(event.primal_bound)) continue;
    ++primal_events;
    REQUIRE(event.incumbent.size() == mip.linear_part.c.size());
    CHECK(detail::satisfies_with_bounds(
        mip.linear_part, event.incumbent, lb, ub, 1e-6));
    CHECK(event.primal_bound ==
          Approx(mip.linear_part.c.dot(event.incumbent)).margin(1e-8));
  }
  CHECK(primal_events >= 1);
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

TEST_CASE("B&C: optional subsolves cannot consume the outer deadline reserve",
          "[bc][deadline][subsolve]") {
  // A 3 s outer solve with 0.3 s reserved for finalization cannot give a
  // nominally 5 s LNS child more than 2.7 s.
  CHECK(detail::bc_optional_subsolve_budget_sec(
            5.0, 3.0, 0.3, 15.0) == Approx(2.7));

  // The LNS stage allowance can be tighter than both local and global limits.
  CHECK(detail::bc_optional_subsolve_budget_sec(
            5.0, 20.0, 1.0, 0.75) == Approx(0.75));

  // No nested solve is started once the finalization reserve or the minimum
  // useful child budget has been consumed.
  CHECK(detail::bc_optional_subsolve_budget_sec(
            5.0, 0.3, 0.3, 15.0) == Approx(0.0));
  CHECK(detail::bc_optional_subsolve_budget_sec(
            5.0, 0.3005, 0.3, 15.0) == Approx(0.0));

  // An unlimited outer solve still respects the per-call and stage caps.
  CHECK(detail::bc_optional_subsolve_budget_sec(
            5.0, std::numeric_limits<double>::infinity(), 0.0, 15.0) ==
        Approx(5.0));
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

TEST_CASE("B&C: native presolve timeout does not publish a partial model",
          "[bc][deadline][native_presolve]") {
  MIPModel mip = make_knapsack_10();
  LPModel lp = mip.linear_part;
  const LPModel original = lp;
  std::vector<int> binary = mip.binary_idx;
  std::vector<int> integer;

  PresolveOptions options;
  options.time_limit_sec = 1e-12;
  MILPPresolve presolve(options);
  const PresolveStats stats = presolve.run(lp, binary, integer);

  CHECK(stats.timed_out);
  CHECK_FALSE(stats.infeasible);
  CHECK(lp.c.isApprox(original.c, 0.0));
  CHECK(lp.A.rows() == original.A.rows());
  CHECK(lp.A.cols() == original.A.cols());
  CHECK(lp.A.nonZeros() == original.A.nonZeros());
  CHECK(binary == mip.binary_idx);
}

TEST_CASE("B&C: HiGHS presolve forward map applies retained affine transforms",
          "[bc][presolve][highs][mapping]") {
  HighsLpPresolveResult ps;
  ps.orig_cols = 4;
  ps.reduced_to_orig_col = {2, 0};
  ps.reduced_col_scale = {-2.0, 0.5};
  ps.reduced_col_constant = {7.0, -1.0};
  ps.reduced_col_linearly_transformable = {1, 1};

  Eigen::VectorXd original(4);
  original << 2.0, 99.0, 1.0, -5.0;
  const Eigen::VectorXd reduced = highs_presolve_forward_map(ps, original);
  REQUIRE(reduced.size() == 2);
  CHECK(reduced[0] == Approx(3.0));
  CHECK(reduced[1] == Approx(6.0));

  ps.reduced_col_linearly_transformable[1] = 0;
  CHECK(highs_presolve_forward_map(ps, original).size() == 0);
  ps.reduced_col_linearly_transformable[1] = 1;
  ps.reduced_col_scale[1] = 0.0;
  CHECK(highs_presolve_forward_map(ps, original).size() == 0);
}

TEST_CASE("B&C: HiGHS side-state forward map transfers root primals",
          "[bc][presolve][highs][mapping][root-primal]") {
  HiGHSPresolvedModelStats side_state;
  side_state.side_state_available = true;
  side_state.presolved_col_orig = {2, 0};
  side_state.presolved_col_scale = {-2.0, 0.5};
  side_state.presolved_col_constant = {7.0, -1.0};
  side_state.presolved_col_linearly_transformable = {1, 1};

  Eigen::VectorXd original(4);
  original << 2.0, 99.0, 1.0, -5.0;
  const Eigen::VectorXd reduced =
      highs_presolve_forward_map(side_state, 4, original);
  REQUIRE(reduced.size() == 2);
  CHECK(reduced[0] == Approx(3.0));
  CHECK(reduced[1] == Approx(6.0));

  side_state.presolved_col_linearly_transformable[1] = 0;
  CHECK(highs_presolve_forward_map(side_state, 4, original).size() == 0);
}

TEST_CASE("B&C: HiGHS affine maps preserve variable-bound inequalities",
          "[bc][presolve][highs][mapping][varbound]") {
  bool upper = true;
  double coef = 3.0;
  double constant = 5.0;
  REQUIRE(detail::highs_style_map_variable_bound_to_reduced(
      upper, coef, constant, 2.0, 7.0, 4.0, -1.0));
  CHECK(upper);
  CHECK(coef == Approx(6.0));
  CHECK(constant == Approx(-2.5));

  upper = true;
  coef = 3.0;
  constant = 5.0;
  REQUIRE(detail::highs_style_map_variable_bound_to_reduced(
      upper, coef, constant, -2.0, 7.0, 4.0, -1.0));
  CHECK_FALSE(upper);
  CHECK(coef == Approx(-6.0));
  CHECK(constant == Approx(2.5));

  CHECK_FALSE(detail::highs_style_map_variable_bound_to_reduced(
      upper, coef, constant, 0.0, 0.0, 1.0, 0.0));
}

TEST_CASE("B&C: only LP-owned transformed cuts enter tree propagation",
          "[bc][cuts][highs][ownership]") {
  CHECK(detail::highs_style_cutpool_row_tree_owned(true, true));
  CHECK_FALSE(detail::highs_style_cutpool_row_tree_owned(true, false));
  CHECK_FALSE(detail::highs_style_cutpool_row_tree_owned(false, true));
  CHECK_FALSE(detail::highs_style_cutpool_row_tree_owned(false, false));

  CHECK(detail::highs_style_cutpool_candidate_stays_root_owned(true, true));
  CHECK(detail::highs_style_cutpool_candidate_stays_root_owned(true, false));
  CHECK(detail::highs_style_cutpool_candidate_stays_root_owned(false, true));
  CHECK_FALSE(
      detail::highs_style_cutpool_candidate_stays_root_owned(false, false));
}

TEST_CASE("B&C: retained HiGHS presolve postsolves eliminated columns",
          "[bc][presolve][highs][postsolve]") {
  LPModel lp = make_knapsack_10().linear_part;
  constexpr int original_cols = 11;
  Eigen::SparseMatrix<double> extended(1, original_cols);
  for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, 0); it; ++it) {
    extended.insert(it.row(), it.col()) = it.value();
  }
  for (int col = 1; col < lp.A.cols(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, col); it; ++it) {
      extended.insert(it.row(), it.col()) = it.value();
    }
  }
  extended.insert(0, 10) = 1.0;
  extended.makeCompressed();
  lp.A = std::move(extended);
  lp.b[0] = 16.0;
  lp.c.conservativeResize(original_cols);
  lp.c[10] = 3.0;
  lp.vars.push_back({VarType::Continuous, 2.0, 2.0});

  HighsLpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.nnz_floor = 0;
  cfg.nnz_cap = 0;
  cfg.min_shrink = 1.0;
  cfg.time_limit_sec = 1.0;
  const HighsLpPresolveResult ps = highs_presolve_lp(lp, cfg);
  REQUIRE(ps.use_reduced);
  REQUIRE(ps.reduced_cols < original_cols);
  REQUIRE(ps.side_state.side_state_available);
  REQUIRE(ps.side_state.cols == ps.reduced_cols);

  // This instance deliberately need not support forward projection: HiGHS
  // may mark surviving columns as non-linearly transformable. The all-zero
  // reduced point is feasible for the presolved <= knapsack and isolates the
  // retained HighsPostsolveStack reconstruction contract.
  const Eigen::VectorXd reduced = Eigen::VectorXd::Zero(ps.reduced_cols);

  Eigen::VectorXd recovered;
  double recovered_objective = 0.0;
  REQUIRE(highs_presolve_recover_primal(
      lp, ps, reduced, 1e-7, recovered, recovered_objective));
  REQUIRE(recovered.size() == original_cols);
  CHECK(recovered[10] == Approx(2.0));
  CHECK(recovered_objective == Approx(lp.c.dot(recovered)));
}

TEST_CASE("B&C: node propagation reports deadline interruption without conflict",
          "[bc][deadline][propagation]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 1.0;
  lp.A.insert(0, 1) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 1.0);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  Eigen::SparseMatrix<double, Eigen::RowMajor> a_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> aeq_row = lp.Aeq;
  detail::RowPropagationIndex row_index;
  row_index.build(2, a_row, aeq_row);
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd ub = Eigen::VectorXd::Ones(2);
  std::vector<BoundChangeInfo> changes{
      {0, 1.0, true, 0.0, 1.0}};
  lb[0] = 1.0;
  std::vector<detail::BranchDomainLiteral> learned_conflict;
  bool interrupted = false;
  int polls = 0;
  const std::function<bool()> stop_requested = [&]() {
    ++polls;
    return true;
  };

  CHECK(detail::propagate_node_domain(
      lp, a_row, aeq_row, row_index, lb, ub, 4,
      static_cast<const detail::ConflictPool*>(nullptr), nullptr, nullptr,
      changes, {}, &learned_conflict, nullptr, nullptr, nullptr, nullptr,
      nullptr, &stop_requested, &interrupted));
  CHECK(interrupted);
  CHECK(polls == 1);
  CHECK(learned_conflict.empty());
  CHECK(lb[0] == Approx(1.0));
  CHECK(ub[1] == Approx(1.0));
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

TEST_CASE("B&C: cover separator stores the exact cut sparsely",
          "[bc][cuts][separator_storage]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(3);
  lp.A.resize(1, 3);
  lp.A.insert(0, 0) = 2.0;
  lp.A.insert(0, 1) = 2.0;
  lp.A.insert(0, 2) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 3.0);
  lp.Aeq.resize(0, 3);
  lp.beq.resize(0);
  lp.vars.assign(3, VariableMeta{VarType::Binary, 0.0, 1.0});

  Eigen::VectorXd x(3);
  x << 0.8, 0.8, 0.0;
  BCOptions options;
  options.cuts = CutType::Cover;
  options.enable_projected_capacity_cuts = false;
  detail::SeparatorStorageStats storage;

  REQUIRE(detail::add_cuts(lp, x, nullptr, options, 1, nullptr, nullptr,
                           nullptr, &storage) == 1);
  REQUIRE(lp.A.rows() == 2);
  Eigen::SparseMatrix<double, Eigen::RowMajor> rows = lp.A;
  std::vector<std::pair<int, double>> terms;
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(rows, 1);
       it; ++it) {
    terms.emplace_back(static_cast<int>(it.col()), it.value());
  }
  REQUIRE(terms.size() == 2);
  CHECK(terms[0].first == 0);
  CHECK(terms[0].second == Approx(1.0));
  CHECK(terms[1].first == 1);
  CHECK(terms[1].second == Approx(1.0));
  CHECK(lp.b[1] == Approx(1.0));
  CHECK(storage.sparse_candidates_created == 1);
  CHECK(storage.sparse_candidate_entries_created == 2);
  CHECK(storage.peak_live_sparse_candidates == 1);
  CHECK(storage.peak_live_sparse_entries == 2);
  CHECK(storage.dense_workspace_materializations == 0);
  CHECK(storage.matrix_append_calls == 1);
  CHECK(storage.matrix_appended_rows == 1);
  CHECK(storage.matrix_appended_entries == 2);
  CHECK(storage.matrix_prior_entries_bypassing_triplet_rebuild == 3);

  for (int mask = 0; mask < 8; ++mask) {
    Eigen::VectorXd integer_x(3);
    for (int j = 0; j < 3; ++j) integer_x[j] = (mask >> j) & 1;
    const double source_activity =
        2.0 * integer_x[0] + 2.0 * integer_x[1] + integer_x[2];
    if (source_activity <= 3.0 + 1e-12) {
      CHECK(integer_x[0] + integer_x[1] <= lp.b[1] + 1e-12);
    }
  }
}

TEST_CASE("B&C: row MIR separator preserves coefficients without dense candidates",
          "[bc][cuts][separator_storage]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(1, 2);
  lp.A.insert(0, 0) = 0.8;
  lp.A.insert(0, 1) = -0.5;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Constant(1, 0.5);
  lp.Aeq.resize(0, 2);
  lp.beq.resize(0);
  lp.vars = {{VarType::Integer, 0.0, 2.0},
             {VarType::Continuous, 0.0, 10.0}};

  Eigen::VectorXd x(2);
  x << 0.8, 0.3;
  BCOptions options;
  options.cuts = CutType::MIR;
  detail::SeparatorStorageStats storage;

  REQUIRE(detail::add_cuts(lp, x, nullptr, options, 1, nullptr, nullptr,
                           nullptr, &storage) == 1);
  REQUIRE(lp.A.rows() == 2);
  Eigen::SparseMatrix<double, Eigen::RowMajor> rows = lp.A;
  std::vector<std::pair<int, double>> terms;
  for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(rows, 1);
       it; ++it) {
    terms.emplace_back(static_cast<int>(it.col()), it.value());
  }
  REQUIRE(terms.size() == 2);
  CHECK(terms[0].first == 0);
  CHECK(terms[0].second == Approx(0.6).margin(1e-12));
  CHECK(terms[1].first == 1);
  CHECK(terms[1].second == Approx(-1.0).margin(1e-12));
  CHECK(lp.b[1] == Approx(0.0).margin(1e-12));
  CHECK(storage.sparse_candidates_created == 1);
  CHECK(storage.sparse_candidate_entries_created == 2);
  CHECK(storage.dense_workspace_materializations == 0);

  for (int integer_x = 0; integer_x <= 2; ++integer_x) {
    const double minimum_y = std::max(0.0, 1.6 * integer_x - 1.0);
    CHECK(0.6 * integer_x - minimum_y <= lp.b[1] + 1e-12);
  }
}

TEST_CASE("B&C: extracted highs_style numeric helpers",
          "[bc][numerics][extraction]") {
  using detail::collect_changed_col_sides_with_tol;
  using detail::highs_style_gcd;
  using detail::highs_style_incumbent_upper_limit;
  using detail::highs_style_objective_integral_scale;
  using detail::highs_style_root_source_eligible;
  using detail::presolved_objective_value;

  SECTION("gcd matches Euclid on signed inputs and zero operands") {
    CHECK(highs_style_gcd(12, 18) == 6);
    CHECK(highs_style_gcd(-24, 36) == 12);
    CHECK(highs_style_gcd(0, 7) == 7);
    CHECK(highs_style_gcd(13, 0) == 13);
    CHECK(highs_style_gcd(17, 5) == 1);
  }

  SECTION("collect_changed_col_sides flags the tightened side only") {
    Eigen::VectorXd old_lb(3), old_ub(3), new_lb(3), new_ub(3);
    old_lb << 0.0, 0.0, 0.0;
    old_ub << 1.0, 1.0, 1.0;
    new_lb << 0.5, 0.0, 0.0;  // col 0 lower tightened
    new_ub << 1.0, 0.4, 1.0;  // col 1 upper tightened
    const auto changed =
        collect_changed_col_sides_with_tol(old_lb, old_ub, new_lb, new_ub, 1e-9);
    CHECK(changed.lower == std::vector<int>{0});
    CHECK(changed.upper == std::vector<int>{1});
    CHECK(changed.any == std::vector<int>{0, 1});
  }

  SECTION("integral objective scale distinguishes integer and continuous cost") {
    LPModel unit;
    unit.c = Eigen::VectorXd::Constant(2, 1.0);
    unit.vars.assign(2, VariableMeta{VarType::Integer, 0.0, 10.0});
    CHECK(highs_style_objective_integral_scale(unit, 1e-9) == Approx(1.0));

    LPModel half;  // UC-style 0.5 increments must recover an integral scale
    half.c = Eigen::VectorXd::Constant(2, 0.5);
    half.vars.assign(2, VariableMeta{VarType::Integer, 0.0, 10.0});
    CHECK(highs_style_objective_integral_scale(half, 1e-9) == Approx(2.0));

    LPModel cont;  // a continuous cost has no integral objective scale
    cont.c = Eigen::VectorXd::Constant(1, 1.0);
    cont.vars.assign(1, VariableMeta{VarType::Continuous, 0.0, 10.0});
    CHECK(highs_style_objective_integral_scale(cont, 1e-9) == Approx(0.0));
  }

  SECTION("incumbent upper limit mirrors HiGHS computeNewUpperLimit") {
    const double tol = 1e-9;

    LPModel unit;  // integer objective, scale 1 -> next integer below incumbent
    unit.c = Eigen::VectorXd::Constant(2, 1.0);
    unit.vars.assign(2, VariableMeta{VarType::Integer, 0.0, 10.0});
    CHECK(highs_style_incumbent_upper_limit(unit, 5.0, tol) ==
          Approx(4.0).margin(1e-7));

    LPModel half;  // scale 2 -> must improve by at least 0.5
    half.c = Eigen::VectorXd::Constant(2, 0.5);
    half.vars.assign(2, VariableMeta{VarType::Integer, 0.0, 10.0});
    CHECK(highs_style_incumbent_upper_limit(half, 5.5, tol) ==
          Approx(5.0).margin(1e-7));

    LPModel cont;  // no integral scale -> strictly-below-incumbent cutoff
    cont.c = Eigen::VectorXd::Constant(1, 1.0);
    cont.vars.assign(1, VariableMeta{VarType::Continuous, 0.0, 10.0});
    const double cont_cut = highs_style_incumbent_upper_limit(cont, 5.0, tol);
    CHECK(cont_cut < 5.0);
    CHECK(cont_cut == Approx(5.0 - tol).margin(1e-12));

    // A non-finite incumbent leaves the cutoff at infinity.
    CHECK(highs_style_incumbent_upper_limit(
              unit, std::numeric_limits<double>::infinity(), tol) ==
          std::numeric_limits<double>::infinity());
  }

  SECTION("root source separation accepts declared-only integer models") {
    CHECK(highs_style_root_source_eligible(271, 0, 14.90225894));
    CHECK(highs_style_root_source_eligible(0, 3, 14.90225894));
    CHECK_FALSE(highs_style_root_source_eligible(0, 0, 14.90225894));
    CHECK_FALSE(highs_style_root_source_eligible(
        271, 0, std::numeric_limits<double>::infinity()));
  }

  SECTION("presolved objectives remove the retained constant offset") {
    CHECK(presolved_objective_value(-97.0, -100.0) == Approx(3.0));
    CHECK(presolved_objective_value(12.5, 2.25) == Approx(10.25));
  }
}

TEST_CASE("B&C: HiGHS presolve offset cannot falsely close the root gap",
          "[bc][presolve][highs][objective]") {
  MIPModel mip = make_knapsack_10();
  constexpr int original_cols = 11;
  Eigen::SparseMatrix<double> extended(1, original_cols);
  for (int col = 0; col < mip.linear_part.A.cols(); ++col) {
    for (Eigen::SparseMatrix<double>::InnerIterator it(mip.linear_part.A, col);
         it; ++it) {
      extended.insert(it.row(), it.col()) = it.value();
    }
  }
  extended.insert(0, 10) = 1.0;
  extended.makeCompressed();
  mip.linear_part.A = std::move(extended);
  mip.linear_part.b[0] += 1.0;
  mip.linear_part.c.conservativeResize(original_cols);
  mip.linear_part.c[10] = 100.0;
  mip.linear_part.vars.push_back({VarType::Continuous, 1.0, 1.0});
  mip.initial_solution = Eigen::VectorXd::Zero(original_cols);
  mip.initial_solution[10] = 1.0;

  HighsLpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.nnz_floor = 0;
  cfg.nnz_cap = 0;
  cfg.min_shrink = 1.0;
  const HighsLpPresolveResult ps =
      highs_presolve_lp(mip.linear_part, cfg);
  REQUIRE(ps.use_reduced);
  REQUIRE(ps.objective_offset == Approx(-100.0));

  BCOptions opt;
  opt.strict_highs_mip_contract = false;
  opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  opt.cuts = CutType::None;
  opt.gap_tol = 0.1;
  opt.time_limit_sec = 2.0;
  const BCResult result = solve_milp_bc(mip, opt);

  REQUIRE(result.stats.success);
  CHECK(result.stats.objective > 100.0);
}

TEST_CASE("B&C: audited HiGHS reduced-to-empty result skips the root LP",
          "[bc][presolve][highs][empty]") {
  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c.resize(2);
  mip.linear_part.c << 3.0, -2.0;
  mip.linear_part.A.resize(0, 2);
  mip.linear_part.b.resize(0);
  mip.linear_part.Aeq.resize(1, 2);
  mip.linear_part.Aeq.insert(0, 0) = 1.0;
  mip.linear_part.Aeq.insert(0, 1) = 1.0;
  mip.linear_part.Aeq.makeCompressed();
  mip.linear_part.beq = Eigen::VectorXd::Constant(1, 1.0);
  mip.linear_part.vars = {
      {VarType::Binary, 0.0, 1.0},
      {VarType::Binary, 0.0, 1.0}};
  mip.binary_idx = {0, 1};

  HighsLpPresolveConfig cfg;
  cfg.enabled = true;
  cfg.nnz_floor = 0;
  cfg.nnz_cap = 0;
  const HighsLpPresolveResult ps =
      highs_presolve_lp(mip.linear_part, cfg);
  REQUIRE(ps.solved_by_presolve);

  BCOptions opt;
  opt.strict_highs_mip_contract = false;
  opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  const BCResult result = solve_milp_bc(mip, opt);

  REQUIRE(result.stats.success);
  CHECK(result.stats.objective == Approx(-2.0));
  CHECK(result.stats.iterations == 0);
  CHECK(result.bc_stats.nodes_explored == 0);
  CHECK(result.bc_stats.lp_solves == 0);
  REQUIRE(result.x.size() == 2);
  CHECK(result.x[0] == Approx(0.0));
  CHECK(result.x[1] == Approx(1.0));
}

TEST_CASE("ScopedJoinThread joins a running thread on every scope exit",
          "[bc][background-task]") {
  // ISO C++20 [thread.thread.destr]: a joinable std::thread destructor calls
  // std::terminate. The wrapper must convert every scope exit (including the
  // strict-root transactional early return) into a join. Derivation:
  // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md,
  // "Root Background-Task Ownership".
  std::atomic<bool> finished{false};
  {
    detail::ScopedJoinThread worker{std::thread([&finished]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      finished.store(true);
    })};
    REQUIRE(worker.joinable());
  }  // Destructor must join here; a terminate would abort the test binary.
  REQUIRE(finished.load());

  // Move-assignment over a running thread must join the previous thread
  // before adopting the new one, and destruction of a moved-from wrapper
  // must be a no-op.
  std::atomic<int> completed{0};
  detail::ScopedJoinThread first{std::thread([&completed]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    completed.fetch_add(1);
  })};
  first = detail::ScopedJoinThread(std::thread([&completed]() {
    completed.fetch_add(1);
  }));
  REQUIRE(completed.load() >= 1);  // previous thread joined by assignment
  first.join();
  REQUIRE(completed.load() == 2);
  REQUIRE_FALSE(first.joinable());
}

TEST_CASE("Retained side-state pass postsolves working-space primals",
          "[bc][strict-postsolve]") {
  // The 10-item knapsack (LP-fractional at its relaxation optimum, so
  // presolve cannot solve it) plus one continuous column x10 tied by the
  // doubleton equality 2 x10 - x0 = 3. HiGHS presolve eliminates x10 and
  // returns `reduced`; the retained presolve-only instance must reconstruct
  // it via Highs::postsolve. Derivation:
  // docs/native_milp_root_quality_restart_prerequisites_2026-08-13.md,
  // "Strict Tree-Incumbent Postsolve Ownership".
  MIPModel knapsack = make_knapsack_10();
  LPModel lp = knapsack.linear_part;
  const int n_orig = 11;
  lp.c.conservativeResize(n_orig);
  lp.c[10] = 1.0;
  Eigen::SparseMatrix<double> A(1, n_orig);
  for (int j = 0; j < 10; ++j) A.insert(0, j) = lp.A.coeff(0, j);
  A.makeCompressed();
  lp.A = A;
  lp.Aeq.resize(1, n_orig);
  lp.Aeq.insert(0, 0) = -1.0;
  lp.Aeq.insert(0, 10) = 2.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(1, 3.0);
  lp.vars.push_back({VarType::Continuous, 0.0, 10.0});

  // A retention-less pass caches without the instance; the strict-contract
  // caller then upgrades the same entry in place.
  const auto& first = detail::cached_highs_presolve_side_state(
      lp, 10.0, nullptr, /*retain_postsolve=*/false);
  REQUIRE(first.presolve_ok);
  REQUIRE(first.highs_status == "reduced");
  CHECK(first.impl == nullptr);

  const auto& ss = detail::cached_highs_presolve_side_state(
      lp, 10.0, nullptr, /*retain_postsolve=*/true);
  REQUIRE(ss.presolve_ok);
  REQUIRE(ss.highs_status == "reduced");
  REQUIRE(ss.presolved_lp_available);
  REQUIRE(ss.impl != nullptr);
  REQUIRE(ss.impl_mutex != nullptr);
  REQUIRE(ss.original_cols == n_orig);
  const int n_work = static_cast<int>(ss.presolved_lp.vars.size());
  REQUIRE(n_work > 0);
  REQUIRE(n_work < n_orig);

  // Produce a working-space incumbent exactly as the strict tree does: an
  // integer-feasible point of the presolved working model itself. Then the
  // retained instance must publish it as an original-feasible point.
  MIPModel working;
  working.linear_part = ss.presolved_lp;
  for (int j = 0; j < n_work; ++j) {
    const VarType type = working.linear_part.vars[static_cast<std::size_t>(j)].type;
    if (type == VarType::Binary) working.binary_idx.push_back(j);
    if (type == VarType::Integer) working.integer_idx.push_back(j);
  }
  BCOptions working_opt;
  working_opt.strict_highs_mip_contract = false;
  working_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  working_opt.time_limit_sec = 10.0;
  const BCResult working_result = solve_milp_bc(working, working_opt);
  REQUIRE(working_result.stats.success);
  REQUIRE(working_result.x.size() == static_cast<Eigen::Index>(n_work));

  const Eigen::VectorXd recovered =
      highs_side_state_postsolve_primal(ss, working_result.x);
  REQUIRE(recovered.size() == n_orig);
  double knapsack_weight = 0.0;
  for (int j = 0; j < 10; ++j) {
    knapsack_weight += lp.A.coeff(0, j) * recovered[j];
    CHECK(recovered[j] >= -1e-9);
    CHECK(recovered[j] <= 1.0 + 1e-9);
    CHECK(std::abs(recovered[j] - std::round(recovered[j])) <= 1e-6);
  }
  CHECK(knapsack_weight <= 14.0 + 1e-9);
  CHECK(2.0 * recovered[10] - recovered[0] == Approx(3.0).margin(1e-9));
  CHECK(recovered[10] >= -1e-9);
  CHECK(recovered[10] <= 10.0 + 1e-9);

  // A wrong-dimension working point must be rejected, not fabricated.
  const Eigen::VectorXd bad = highs_side_state_postsolve_primal(
      ss, Eigen::VectorXd::Zero(n_work + 7));
  CHECK(bad.size() == 0);
}
