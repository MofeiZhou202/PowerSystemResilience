/// test_milp_solver.cpp
/// Tests for MILP solving via the engine — exercises every registered MILP solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <chrono>
#include <limits>
#include <random>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_restart.hpp"
#include "mipsolvers/engine/detail/bc_status.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"

using namespace mipsolvers::engine;
using Catch::Approx;

TEST_CASE("MILP: incumbent restart controller enforces every trigger gate",
          "[milp][restart][policy]") {
  BCTreeRestartOptions policy;
  policy.enabled = true;
  policy.max_restarts = 2;
  policy.min_nodes_since_restart = 10;
  policy.min_open_nodes = 3;
  policy.min_relative_incumbent_improvement = 0.01;
  policy.min_remaining_time_sec = 0.5;
  detail::IncumbentTreeRestartController controller(policy);

  controller.note_incumbent(100.0, 4, "first");
  CHECK(controller.pending());
  CHECK_FALSE(controller.ready(9, 3, true, 1.0));
  CHECK_FALSE(controller.ready(10, 2, true, 1.0));
  CHECK_FALSE(controller.ready(10, 3, false, 1.0));
  CHECK_FALSE(controller.ready(10, 3, true, 0.49));
  REQUIRE(controller.ready(10, 3, true, 1.0));
  controller.commit(10);
  CHECK(controller.restart_count() == 1);

  controller.note_incumbent(99.5, 12, "too_small");
  CHECK_FALSE(controller.ready(20, 3, true, 1.0));
  controller.note_incumbent(98.0, 14, "cumulative");
  REQUIRE(controller.ready(20, 3, true, 1.0));
  CHECK(controller.pending_source() == "cumulative");
  controller.commit(20);
  CHECK(controller.restart_count() == 2);

  controller.note_incumbent(90.0, 30, "over_limit");
  CHECK_FALSE(controller.ready(40, 3, true, 1.0));
}

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

TEST_CASE("MILP: native B&C rejects a free general integer explicitly",
          "[milp][validation][integrality]") {
  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c = Eigen::VectorXd::Zero(1);
  mip.linear_part.A.resize(0, 1);
  mip.linear_part.b.resize(0);
  mip.linear_part.Aeq.resize(0, 1);
  mip.linear_part.beq.resize(0);
  mip.linear_part.vars.push_back(
      {VarType::Integer, -1e20, 1e20, "free_integer"});
  mip.integer_idx = {0};

  BCOptions opt;
  opt.strict_highs_mip_contract = false;
  const BCResult result = solve_milp_bc(mip, opt);
  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.status ==
        detail::bc_status::kUnsupportedFreeIntegerVariable);
}

TEST_CASE("MILP: direct B&C validates conflicting integrality indices",
          "[milp][validation][integrality]") {
  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c = Eigen::VectorXd::Zero(1);
  mip.linear_part.A.resize(0, 1);
  mip.linear_part.b.resize(0);
  mip.linear_part.Aeq.resize(0, 1);
  mip.linear_part.beq.resize(0);
  mip.linear_part.vars.push_back({VarType::Continuous, 0.0, 1.0});
  mip.integer_idx = {0};
  mip.binary_idx = {0};

  const BCResult result = solve_milp_bc(mip, BCOptions{});
  CHECK_FALSE(result.stats.success);
  CHECK(result.stats.status ==
        detail::bc_status::kConflictingIntegralityDeclaration);
}

TEST_CASE("MILP: quantized hashes never decide exact deduplication",
          "[milp][dedup][collision]") {
  std::vector<detail::BranchDomainLiteral> first{
      {0, 0.5000001, true}, {1, 0.5000001, false}};
  std::vector<detail::BranchDomainLiteral> second{
      {0, 0.5000002, true}, {1, 0.5000002, false}};
  detail::canonicalize_branch_literals(first);
  detail::canonicalize_branch_literals(second);
  REQUIRE(detail::conflict_clause_hash(first) ==
          detail::conflict_clause_hash(second));
  REQUIRE_FALSE(detail::branch_literal_lists_equal(first, second));

  detail::BranchLiteralListSet literal_lists;
  CHECK(literal_lists.insert(first));
  CHECK(literal_lists.insert(second));

  detail::DomainReasonBound first_reason;
  first_reason.bound = {2, 1.0, true};
  first_reason.reason = first;
  detail::DomainReasonBound second_reason = first_reason;
  second_reason.reason = second;
  CHECK_FALSE(detail::domain_reason_bounds_equal(first_reason, second_reason));

  detail::ConflictPool pool;
  CHECK(pool.add(first));
  CHECK(pool.add(second));
  CHECK(pool.size() == 2);

  detail::SharedConflictPool shared_pool;
  CHECK(shared_pool.add(first));
  CHECK(shared_pool.add(second));
  CHECK(shared_pool.size() == 2);

  Eigen::SparseVector<double> row_a(1), row_b(1);
  row_a.insertBack(0) = 1.00001;
  row_b.insertBack(0) = 1.00002;
  REQUIRE(detail::sparse_cut_hash(row_a) == detail::sparse_cut_hash(row_b));
  CHECK_FALSE(detail::sparse_cut_identical(row_a, 1.0, row_b, 1.0));
  detail::SparseCutSet sparse_rows;
  CHECK(sparse_rows.insert(row_a, 1.0));
  CHECK(sparse_rows.insert(row_b, 1.0));
}

TEST_CASE("MILP: conflict propagation wakes indexed clauses only",
          "[milp][conflict][propagation][index]") {
  constexpr int n = 128;
  std::vector<VariableMeta> vars(
      n, VariableMeta{VarType::Binary, 0.0, 1.0});
  detail::ConflictPool pool(/*max_size=*/256, /*max_literals=*/8);

  REQUIRE(pool.add({{0, 1.0, true}, {1, 1.0, true}}));
  REQUIRE(pool.add({{1, 0.0, false}, {2, 1.0, true}}));
  for (int j = 3; j < 120; ++j) {
    REQUIRE(pool.add({{j, 1.0, true}, {127, 1.0, true}}));
  }

  Eigen::VectorXd lb = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd ub = Eigen::VectorXd::Ones(n);
  lb[0] = 1.0;
  int tightened = 0;
  std::vector<BoundChangeInfo> changes;
  REQUIRE(pool.propagate(vars, lb, ub, &tightened, &changes));

  CHECK(ub[1] == Approx(0.0));
  CHECK(ub[2] == Approx(0.0));
  CHECK(tightened == 2);
  CHECK(pool.last_propagation_clauses_scanned() < pool.size() / 4);
}

TEST_CASE("MILP: queued node domains use exact bounds for deduplication",
          "[milp][dedup][domain]") {
  detail::ThreadSafeNodeQueue queue{NodeSelection::BestFirst};
  Eigen::VectorXd root_lb(1), root_ub(1);
  root_lb << 0.0;
  root_ub << 10.0;
  queue.configure_domain_signature({1}, root_lb, root_ub);

  detail::Node first;
  first.lb = Eigen::VectorXd::Constant(1, 1.1);
  first.ub = root_ub;
  first.bound = 0.0;
  first.estimate = 0.0;
  detail::Node second = first;
  second.lb[0] = 1.2;

  queue.push(std::move(first), true);
  queue.push(std::move(second), true);
  CHECK(queue.size() == 2);
}

TEST_CASE("MILP: serial queued domains are sparse at rest and exact on pop",
          "[milp][queue][domain_storage]") {
  detail::NodeQueue queue{NodeSelection::BestFirst};
  Eigen::VectorXd root_lb(4), root_ub(4);
  root_lb << 0.0, -detail::kInf, 0.0, -5.0;
  root_ub << 1.0, detail::kInf, 10.0, 5.0;
  queue.configure_domain_signature({1, 1, 1, 1}, root_lb, root_ub);

  detail::Node node;
  node.lb = root_lb;
  node.ub = root_ub;
  node.lb[1] = -3.0;
  node.ub[2] = 7.0;
  node.bound = 2.0;
  node.estimate = 2.5;
  queue.push_priority(node);

  auto storage = queue.domain_storage_stats();
  CHECK(storage.current_compact_nodes == 1);
  CHECK(storage.current_compact_entries == 2);
  CHECK(storage.dense_bound_values_released == 8);
  CHECK(storage.compaction_failures == 0);

  detail::Node out;
  REQUIRE(queue.pop(out, true));
  CHECK(out.lb.size() == 4);
  CHECK(out.ub.size() == 4);
  CHECK(out.lb[0] == Approx(0.0));
  CHECK(out.ub[0] == Approx(1.0));
  CHECK(out.lb[1] == Approx(-3.0));
  CHECK(std::isinf(out.ub[1]));
  CHECK(out.ub[2] == Approx(7.0));
  CHECK(out.lb[3] == Approx(-5.0));
  CHECK(out.compact_domain.empty());
  storage = queue.domain_storage_stats();
  CHECK(storage.current_compact_nodes == 0);
  CHECK(storage.current_compact_entries == 0);
  CHECK(storage.materializations == 1);
  CHECK(storage.materialization_failures == 0);
}

TEST_CASE("MILP: restart queue transaction discards every index and compact domain",
          "[milp][restart][queue]") {
  detail::NodeQueue queue{NodeSelection::Hybrid};
  Eigen::VectorXd root_lb = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd root_ub = Eigen::VectorXd::Constant(3, 10.0);
  queue.configure_domain_signature({1, 1, 1}, root_lb, root_ub);

  detail::Node first;
  first.lb = root_lb;
  first.ub = root_ub;
  first.lb[0] = 2.0;
  first.bound = 1.0;
  first.estimate = 1.5;
  queue.push_dfs(first);

  detail::Node second = first;
  second.lb = root_lb;
  second.ub = root_ub;
  second.ub[2] = 7.0;
  second.bound = 2.0;
  second.estimate = 2.5;
  queue.push_priority(second);
  REQUIRE(queue.size() == 2);
  REQUIRE(queue.domain_storage_stats().current_compact_nodes == 2);

  CHECK(queue.discard_all() == 2);
  CHECK(queue.empty());
  CHECK(queue.size() == 0);
  CHECK(queue.priority_size() == 0);
  CHECK(queue.dfs_size() == 0);
  CHECK(queue.domain_storage_stats().current_compact_nodes == 0);
  CHECK(queue.domain_storage_stats().current_compact_entries == 0);

  detail::Node restart_root;
  restart_root.lb = root_lb;
  restart_root.ub = root_ub;
  restart_root.bound = 0.5;
  restart_root.estimate = 0.5;
  restart_root.lp_refresh_needed = true;
  queue.push_priority(std::move(restart_root));
  detail::Node out;
  REQUIRE(queue.pop(out, true));
  CHECK(out.lb == root_lb);
  CHECK(out.ub == root_ub);
  CHECK(out.lp_refresh_needed);
}

TEST_CASE("MILP: queued domains rebase after root tightening and recompact",
          "[milp][queue][domain_storage][root_domain]") {
  detail::NodeQueue queue{NodeSelection::DepthFirst};
  Eigen::VectorXd root_lb(2), root_ub(2);
  root_lb << 0.0, 0.0;
  root_ub << 10.0, 10.0;
  queue.configure_domain_signature({1, 1}, root_lb, root_ub);

  detail::Node node;
  node.lb = root_lb;
  node.ub = root_ub;
  node.lb[0] = 2.0;
  node.ub[1] = 8.0;
  node.bound = 0.0;
  queue.push_dfs(std::move(node));

  Eigen::VectorXd tightened_root_lb = root_lb;
  Eigen::VectorXd tightened_root_ub = root_ub;
  tightened_root_lb[0] = 1.0;
  tightened_root_ub[1] = 9.0;
  CHECK(queue.inherit_root_domain(tightened_root_lb, tightened_root_ub) == 0);
  const auto storage = queue.domain_storage_stats();
  CHECK(storage.current_compact_nodes == 1);
  CHECK(storage.current_compact_entries == 2);
  CHECK(storage.compactions >= 2);
  CHECK(storage.materializations >= 1);

  detail::Node out;
  REQUIRE(queue.pop(out, false));
  CHECK(out.lb[0] == Approx(2.0));
  CHECK(out.ub[1] == Approx(8.0));
  CHECK(out.lb[1] == Approx(0.0));
  CHECK(out.ub[0] == Approx(10.0));
}

TEST_CASE("MILP: parallel queued domains use the same sparse storage contract",
          "[milp][parallel][queue][domain_storage]") {
  detail::ThreadSafeNodeQueue queue{NodeSelection::BestFirst};
  Eigen::VectorXd root_lb = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd root_ub = Eigen::VectorXd::Constant(3, 10.0);
  queue.configure_domain_signature({1, 1, 1}, root_lb, root_ub);

  detail::Node node;
  node.lb = root_lb;
  node.ub = root_ub;
  node.lb[2] = 4.0;
  node.bound = 1.0;
  node.estimate = 1.0;
  queue.push(std::move(node), true);
  auto storage = queue.domain_storage_stats();
  CHECK(storage.current_compact_nodes == 1);
  CHECK(storage.current_compact_entries == 1);

  detail::Node out;
  REQUIRE(queue.pop(out, true, std::chrono::milliseconds(1)));
  CHECK(out.lb[2] == Approx(4.0));
  CHECK(out.ub[2] == Approx(10.0));
  CHECK(out.compact_domain.empty());
  storage = queue.domain_storage_stats();
  CHECK(storage.current_compact_nodes == 0);
  CHECK(storage.materializations == 1);
}

TEST_CASE("MILP: zero is a replayable work-stealing seed",
          "[milp][parallel][determinism]") {
  detail::WorkStealingPool first(3, 16, 0);
  detail::WorkStealingPool second(3, 16, 0);
  for (int worker = 1; worker <= 2; ++worker) {
    for (int i = 0; i < 6; ++i) {
      detail::Node a;
      a.bound = 100.0 * worker + i;
      detail::Node b = a;
      REQUIRE(first.get(worker).push(std::move(a)));
      REQUIRE(second.get(worker).push(std::move(b)));
    }
  }

  for (int i = 0; i < 12; ++i) {
    detail::Node a;
    detail::Node b;
    REQUIRE(first.try_steal(0, a));
    REQUIRE(second.try_steal(0, b));
    CHECK(a.bound == b.bound);
  }
}

TEST_CASE("MILP: queued-domain propagation restores sparse-at-rest storage",
          "[milp][queue][domain_storage][propagation]") {
  detail::NodeQueue queue{NodeSelection::DepthFirst};
  Eigen::VectorXd root_lb = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd root_ub = Eigen::VectorXd::Constant(2, 10.0);
  queue.configure_domain_signature({1, 1}, root_lb, root_ub);

  detail::Node node;
  node.lb = root_lb;
  node.ub = root_ub;
  node.lb[0] = 1.0;
  node.bound = 0.0;
  queue.push_dfs(std::move(node));

  std::uint64_t tightened = 0;
  CHECK(queue.propagate_by_domain_object(
            [](const detail::Node&, Eigen::VectorXd& lb,
               Eigen::VectorXd&, int& local_tightened, int& local_pruned) {
              lb[1] = 2.0;
              local_tightened = 1;
              local_pruned = 0;
              return true;
            },
            false, detail::kInf, 1e-9, &tightened) == 0);
  CHECK(tightened == 1);
  auto storage = queue.domain_storage_stats();
  CHECK(storage.materializations == 1);
  CHECK(storage.compactions == 2);
  CHECK(storage.current_compact_nodes == 1);
  CHECK(storage.current_compact_entries == 2);

  CHECK(queue.propagate_by_domain_object(
            [](const detail::Node&, Eigen::VectorXd&, Eigen::VectorXd&,
               int& local_tightened, int& local_pruned) {
              local_tightened = 0;
              local_pruned = 1;
              return false;
            }) == 1);
  storage = queue.domain_storage_stats();
  CHECK(queue.empty());
  CHECK(storage.materializations == 2);
  CHECK(storage.current_compact_nodes == 0);
  CHECK(storage.current_compact_entries == 0);
  CHECK(storage.compaction_failures == 0);
  CHECK(storage.materialization_failures == 0);
}

TEST_CASE("MILP: branch payloads detach only when a child mutates them",
          "[milp][branch][memory][cow]") {
  detail::Node parent;
  parent.lb = Eigen::VectorXd::Zero(1);
  parent.ub = Eigen::VectorXd::Ones(1);
  parent.branch_reasons.push_back({0, 0.0, true});
  detail::PoolCut cut;
  cut.coeff.resize(1);
  cut.coeff.insert(0) = 1.0;
  cut.rhs = 1.0;
  parent.local_cuts.push_back(cut);

  detail::NodePayloadSharingStats sharing;
  detail::Node child = parent.branch_child(&sharing);
  CHECK(sharing.child_creations == 1);
  CHECK(sharing.shared_payload_vectors == 2);
  CHECK(sharing.shared_payload_elements == 2);
  REQUIRE(parent.branch_reasons.shares_storage_with(child.branch_reasons));
  REQUIRE(parent.local_cuts.shares_storage_with(child.local_cuts));
  CHECK(parent.branch_reasons.storage_use_count() == 2);
  CHECK(parent.local_cuts.storage_use_count() == 2);

  child.branch_reasons.push_back({0, 1.0, true});
  detail::PoolCut child_cut = cut;
  child_cut.rhs = 0.5;
  child.local_cuts.push_back(std::move(child_cut));

  CHECK_FALSE(parent.branch_reasons.shares_storage_with(child.branch_reasons));
  CHECK_FALSE(parent.local_cuts.shares_storage_with(child.local_cuts));
  CHECK(parent.branch_reasons.size() == 1);
  CHECK(child.branch_reasons.size() == 2);
  CHECK(parent.local_cuts.size() == 1);
  CHECK(child.local_cuts.size() == 2);
  CHECK(parent.local_cuts[0].rhs == Approx(1.0));
  CHECK(child.local_cuts[1].rhs == Approx(0.5));
}

TEST_CASE("MILP: parallel progress notifications survive a late wait",
          "[milp][parallel][monitor]") {
  detail::ParallelProgressEvent event;
  std::uint64_t observed = event.generation();
  CHECK(observed == 0);

  event.notify();
  CHECK(event.wait_until_change(
      observed, std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(50)));
  CHECK(observed == 1);

  CHECK_FALSE(event.wait_until_change(
      observed, std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(1)));
  CHECK(observed == 1);
}

TEST_CASE("MILP: cut-pool selection ages and purges without stale indices",
          "[milp][cutpool]") {
  detail::CutPool pool(/*max_size=*/4, /*max_age=*/1, /*expected_size=*/1);
  Eigen::SparseVector<double> row(1);
  row.insertBack(0) = 1.0;
  REQUIRE(pool.add(row, 0.0));

  Eigen::VectorXd x(1);
  x << 1.0;
  auto selected = pool.select_violated_and_age(x, 1e-6, 1);
  REQUIRE(selected.size() == 1);
  CHECK(selected.front().rhs == Approx(0.0));
  CHECK(pool.size() == 1);

  x << 0.0;
  CHECK(pool.select_violated_and_age(x, 1e-6, 1).empty());
  CHECK(pool.size() == 1);
  CHECK(pool.select_violated_and_age(x, 1e-6, 1).empty());
  CHECK(pool.size() == 0);
}

TEST_CASE("MILP: shared incumbent publishes a matching solution and objective",
          "[milp][parallel][incumbent]") {
  detail::SharedIncumbent incumbent;
  Eigen::VectorXd first(2), better(2), worse(2);
  first << 1.0, 0.0;
  better << 0.0, 1.0;
  worse << 1.0, 1.0;

  REQUIRE(incumbent.try_update(first, 4.0));
  REQUIRE(incumbent.try_update(better, 3.0));
  CHECK_FALSE(incumbent.try_update(worse, 5.0));

  const auto snapshot = incumbent.snapshot();
  REQUIRE(snapshot.has);
  CHECK(snapshot.obj == Approx(3.0));
  CHECK(incumbent.get_x().isApprox(better));
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
  const double dense_max = detail::compute_node_estimate(
      branchable, x, dense, 1e-6, 10.0,
      NodeEstimateAggregation::Maximum);
  CHECK(dense_max == Approx(detail::compute_node_estimate(
      branchable, x, compact, 1e-6, 10.0,
      NodeEstimateAggregation::Maximum)).margin(1e-12));
  CHECK(dense_max <=
        detail::compute_node_estimate(branchable, x, dense, 1e-6, 10.0));
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

TEST_CASE("MILP: current-node exact evidence overrides historical pseudocosts",
          "[milp][branching][reliability]") {
  Eigen::VectorXd x(2);
  x << 0.5, 0.5;
  std::vector<detail::PseudoCost> pc(2);
  for (int sample = 0; sample < 1000; ++sample) {
    pc[0].add_down(100.0);
    pc[0].add_up(100.0);
  }
  pc[1].add_down(1.0);
  pc[1].add_up(1.0);

  const std::vector<int> candidates{0, 1};
  const std::vector<int> priority;
  CHECK(detail::choose_branch_var_pseudocost(candidates, x, pc) == 0);

  const std::vector<detail::BranchDirectionalObservation> observations{
      {0, false, detail::BranchEvidenceState::ExactOptimal, 0.1},
      {0, true, detail::BranchEvidenceState::ExactOptimal, 0.1},
      {1, false, detail::BranchEvidenceState::ExactOptimal, 2.0},
      {1, true, detail::BranchEvidenceState::ExactOptimal, 2.0},
  };
  CHECK(detail::choose_branch_var_pseudocost_with_overlay(
            candidates, x, pc, priority, observations) == 1);
}

TEST_CASE("MILP: reliability overlay mixes measured and predicted directions",
          "[milp][branching][reliability]") {
  detail::PseudoCost pc;
  pc.add_down(4.0);
  pc.add_up(6.0);
  const detail::BranchDirectionalObservation down{
      0, false, detail::BranchEvidenceState::ExactOptimal, 3.0};
  CHECK(detail::compute_branch_product_score(pc, 0.25, &down, nullptr) ==
        Approx(13.5));
}

TEST_CASE("MILP: child direction scheduling respects primal and proof modes",
          "[milp][branching][direction]") {
  detail::PseudoCost pc;
  pc.add_down(10.0);
  pc.add_up(2.0);
  CHECK(detail::choose_up_branch_first(
      pc, 0.5, nullptr, nullptr, false));

  const detail::BranchDirectionalObservation cheap_down{
      0, false, detail::BranchEvidenceState::ExactOptimal, 0.1};
  CHECK_FALSE(detail::choose_up_branch_first(
      pc, 0.5, &cheap_down, nullptr, false));

  const detail::BranchDirectionalObservation up_cutoff{
      0, true, detail::BranchEvidenceState::ProvenCutoff, 0.0};
  CHECK(detail::choose_up_branch_first(
      pc, 0.5, &cheap_down, &up_cutoff, true));
  CHECK_FALSE(detail::choose_up_branch_first(
      pc, 0.5, &cheap_down, nullptr, true));
  CHECK(detail::choose_up_branch_first(
      pc, 0.5, &cheap_down, nullptr, true, false, 1));
}

TEST_CASE("MILP: estimator calibration separates node and direction errors",
          "[milp][branching][calibration]") {
  const auto calibration = detail::compute_branch_estimator_calibration(
      10.0, 15.0, true, 2.0, true, 5.0, true, 13.0, true, 14.0);
  CHECK(calibration.node_samples == 1);
  CHECK(calibration.node_predicted_lift_sum == Approx(5.0));
  CHECK(calibration.node_realized_lift_sum == Approx(3.0));
  CHECK(calibration.node_abs_error_sum == Approx(2.0));
  CHECK(calibration.node_squared_error_sum == Approx(4.0));
  CHECK(calibration.node_predicted_sq_sum == Approx(25.0));
  CHECK(calibration.node_realized_sq_sum == Approx(9.0));
  CHECK(calibration.node_cross_sum == Approx(15.0));
  CHECK(calibration.directional_samples == 2);
  CHECK(calibration.directional_predicted_gain_sum == Approx(7.0));
  CHECK(calibration.directional_realized_gain_sum == Approx(7.0));
  CHECK(calibration.directional_abs_error_sum == Approx(2.0));
  CHECK(calibration.directional_squared_error_sum == Approx(2.0));
  CHECK(calibration.directional_rank_samples == 1);
  CHECK(calibration.directional_rank_concordant == 1);

  const auto probe_filtered = detail::compute_branch_estimator_calibration(
      10.0, 15.0, false, 2.0, true, 5.0, true, 13.0, true, 14.0);
  CHECK(probe_filtered.node_samples == 1);
  CHECK(probe_filtered.node_realized_lift_sum == Approx(3.0));
  CHECK(probe_filtered.directional_samples == 1);
  CHECK(probe_filtered.directional_rank_samples == 0);
}

TEST_CASE("MILP: reliable unprobed candidate can beat a measured candidate",
          "[milp][branching][reliability]") {
  Eigen::VectorXd x(2);
  x << 0.5, 0.5;
  std::vector<detail::PseudoCost> pc(2);
  pc[0].add_down(1.0);
  pc[0].add_up(1.0);
  pc[1].add_down(4.0);
  pc[1].add_up(4.0);
  const std::vector<detail::BranchDirectionalObservation> observations{
      {0, false, detail::BranchEvidenceState::Measured, 1.0},
      {0, true, detail::BranchEvidenceState::Measured, 1.0},
  };
  CHECK(detail::choose_branch_var_pseudocost_with_overlay(
            {0, 1}, x, pc, {}, observations) == 1);
}

TEST_CASE("MILP: reliability evidence preserves priority and dynamic prior",
          "[milp][branching][reliability]") {
  Eigen::VectorXd x(2);
  x << 0.5, 0.5;
  std::vector<detail::PseudoCost> pc(2);
  const std::vector<int> candidates{0, 1};
  const std::vector<detail::BranchDirectionalObservation> observations{
      {1, false, detail::BranchEvidenceState::ExactOptimal, 100.0},
      {1, true, detail::BranchEvidenceState::ExactOptimal, 100.0},
  };
  CHECK(detail::choose_branch_var_pseudocost_with_overlay(
            candidates, x, pc, {10, 0}, observations) == 0);

  BCBranchingPriorFn prior = [](const BCBranchContext&,
                                std::vector<double>& scores) {
    scores = {-10.0, 10.0};
  };
  CHECK(detail::choose_branch_var_pseudocost_with_overlay(
            candidates, x, pc, {}, {}, prior, {}) == 1);
}

TEST_CASE("MILP: parallel candidate snapshot matches serial branch selection",
          "[milp][branching][reliability][parallel]") {
  Eigen::VectorXd x(3);
  x << 0.5, 0.5, 0.5;
  std::vector<detail::PseudoCost> pc(3);
  pc[0].add_down(8.0);
  pc[0].add_up(8.0);
  pc[1].add_down(100.0);
  pc[1].add_up(100.0);
  pc[2].add_down(1.0);
  pc[2].add_up(1.0);

  const std::vector<int> candidates{0, 1, 2};
  const std::vector<int> priority{5, 0, 5};
  const std::vector<detail::PseudoCost> candidate_pc{pc[0], pc[1], pc[2]};
  const std::vector<int> original_candidates{10, 11, 12};
  BCBranchContext context;
  context.candidates = &original_candidates;
  bool saw_original_context = false;
  BCBranchingPriorFn prior = [&](const BCBranchContext& callback_context,
                                 std::vector<double>& scores) {
    saw_original_context =
        callback_context.candidates == &original_candidates;
    scores = {0.0, 100.0, 10.0};
  };
  const std::vector<detail::BranchDirectionalObservation> observations{
      {0, false, detail::BranchEvidenceState::ExactOptimal, 0.2},
      {0, true, detail::BranchEvidenceState::ExactOptimal, 0.2},
      {2, false, detail::BranchEvidenceState::Measured, 1.0},
  };

  const int serial = detail::choose_branch_var_pseudocost_with_overlay(
      candidates, x, pc, priority, observations, prior, context);
  const int parallel = detail::choose_branch_var_pseudocost_candidates(
      candidates, x, candidate_pc, priority, prior, context, observations);
  CHECK(serial == 2);
  CHECK(parallel == serial);
  CHECK(saw_original_context);

  LPModel unused_lp;
  StandardFormLP unused_sf;
  Eigen::VectorXd unused_lb = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd unused_ub = Eigen::VectorXd::Ones(3);
  SimplexOptions simplex_options;
  int lp_solves = 0;
  CHECK(detail::choose_branch_var_reliability(
            candidates, x, pc, unused_lp, unused_sf, unused_lb, unused_ub,
            nullptr, simplex_options, 0.0, lp_solves, 0, 1, 0, priority,
            prior, context) == serial);
  CHECK(detail::choose_branch_var_reliability(
            candidates, x, pc, unused_lp, unused_sf, unused_lb, unused_ub,
            nullptr, simplex_options, 0.0, lp_solves, 7, 1, 4, priority,
            prior, context) == serial);
  CHECK(lp_solves == 0);
}

TEST_CASE("MILP: probe failures never create pseudocost gain samples",
          "[milp][branching][reliability]") {
  detail::PseudoCost pc;
  detail::update_pseudocost_from_branch_evidence(
      pc, false, detail::BranchEvidenceState::UnknownFailure, 1e6, 0.25);
  CHECK(pc.down_cnt == 0);
  CHECK(pc.down_cutoff_cnt == 0);

  detail::update_pseudocost_from_branch_evidence(
      pc, false, detail::BranchEvidenceState::ProvenCutoff, 1e6, 0.25, true);
  CHECK(pc.down_cnt == 0);
  CHECK(pc.down_cutoff_cnt == 1);
  CHECK(pc.down_conflict_score == Approx(1.0));
}

TEST_CASE("MILP: reliability probes reuse one transactional standard form",
          "[milp][branching][reliability][transaction]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(1);
  lp.A.resize(1, 1);
  lp.A.insert(0, 0) = 1.0;
  lp.A.makeCompressed();
  lp.b = Eigen::VectorXd::Ones(1);
  lp.Aeq.resize(0, 1);
  lp.beq.resize(0);
  lp.vars = {{VarType::Binary, 0.0, 1.0}};

  StandardFormLP sf = build_standard_form_lp(lp);
  ruiz_scale_standard_form(sf);
  const Eigen::VectorXd lb = Eigen::VectorXd::Zero(1);
  const Eigen::VectorXd ub = Eigen::VectorXd::Ones(1);
  const Eigen::VectorXd x = Eigen::VectorXd::Constant(1, 0.5);
  std::vector<detail::PseudoCost> pc(1);
  SimplexOptions simplex_options;
  simplex_options.max_iter = 100;
  int lp_solves = 0;
  detail::StrongBranchProbeStorageStats storage;
  std::vector<detail::BranchDirectionalObservation> observations;

  const int selected = detail::choose_branch_var_reliability(
      {0}, x, pc, lp, sf, lb, ub, nullptr, simplex_options, 0.0,
      lp_solves, 0, 1, 1, {}, {}, {}, &observations, &storage);
  CHECK(selected == 0);
  CHECK(lp_solves == 2);
  CHECK(storage.base_sf_materializations == 1);
  CHECK(storage.bound_transactions == 2);
  CHECK(storage.transaction_rollbacks == 2);
  CHECK(storage.transaction_failures == 0);
  CHECK(storage.transaction_snapshot_values >= 6);
  CHECK(storage.backend_cold_solves == 1);
  CHECK(storage.backend_persistent_resolves == 1);
  CHECK(observations.size() == 2);
}

TEST_CASE("MILP: consumed probe result is not sampled twice",
          "[milp][branching][reliability]") {
  detail::PseudoCost pc;
  detail::update_pseudocost_from_branch_evidence(
      pc, true, detail::BranchEvidenceState::ExactOptimal, 6.0, 0.75);
  REQUIRE(pc.up_cnt == 1);
  CHECK(pc.up_avg() == Approx(8.0));

  detail::PCUpdate consumed{0, true};
  consumed.branch_distance = 0.75;
  consumed.has_gain = true;
  consumed.gain = 6.0;
  consumed.suppress_gain_update = true;
  consumed.has_inference = true;
  consumed.inference_count = 2.0;
  consumed.apply(pc);
  CHECK(pc.up_cnt == 1);
  CHECK(pc.up_inference_cnt == 1);
}

TEST_CASE("MILP: probe reuse requires an exact domain and state signature",
          "[milp][branching][reuse]") {
  Eigen::VectorXd probe_lb(2), probe_ub(2);
  probe_lb << 0.0, -1.0;
  probe_ub << 1.0, 4.0;

  auto classify = [&](const Eigen::VectorXd& child_lb,
                      const Eigen::VectorXd& child_ub,
                      std::uint64_t child_rows,
                      std::uint64_t child_model_epoch,
                      std::uint64_t child_objective_epoch,
                      bool proof_usable,
                      bool warm_usable,
                      bool consumed = false) {
    return detail::classify_branch_probe_reuse(
        true, consumed, 1, true, 1, true, probe_lb, probe_ub, child_lb,
        child_ub, 41, child_rows, 7, child_model_epoch, 11,
        child_objective_epoch, proof_usable, warm_usable);
  };

  CHECK(classify(probe_lb, probe_ub, 41, 7, 11, true, true) ==
        detail::BranchProbeReuseKind::Exact);
  CHECK(classify(probe_lb, probe_ub, 41, 7, 11, false, true) ==
        detail::BranchProbeReuseKind::Warm);

  Eigen::VectorXd tighter_lb = probe_lb;
  tighter_lb[0] = 0.25;
  CHECK(classify(tighter_lb, probe_ub, 41, 7, 11, true, true) ==
        detail::BranchProbeReuseKind::Warm);

  Eigen::VectorXd looser_lb = probe_lb;
  looser_lb[1] = -2.0;
  CHECK(classify(looser_lb, probe_ub, 41, 7, 11, true, true) ==
        detail::BranchProbeReuseKind::None);
  CHECK(classify(probe_lb, probe_ub, 42, 7, 11, true, true) ==
        detail::BranchProbeReuseKind::None);
  CHECK(classify(probe_lb, probe_ub, 41, 8, 11, true, true) ==
        detail::BranchProbeReuseKind::None);
  CHECK(classify(probe_lb, probe_ub, 41, 7, 12, true, true) ==
        detail::BranchProbeReuseKind::None);
  CHECK(classify(probe_lb, probe_ub, 41, 7, 11, true, true, true) ==
        detail::BranchProbeReuseKind::None);
}

TEST_CASE("MILP: local-cut reuse signature includes row order and RHS",
          "[milp][branching][reuse]") {
  detail::PoolCut first;
  first.coeff = Eigen::SparseVector<double>(2);
  first.coeff.insert(0) = 1.0;
  first.coeff.insert(1) = -2.0;
  first.rhs = 3.0;
  first.validity_scope = ValidityScope::LocalNodeCut;

  detail::PoolCut second;
  second.coeff = Eigen::SparseVector<double>(2);
  second.coeff.insert(1) = 4.0;
  second.rhs = 5.0;
  second.validity_scope = ValidityScope::LocalNodeCut;

  const std::vector<detail::PoolCut> ordered{first, second};
  const std::vector<detail::PoolCut> reordered{second, first};
  auto rhs_changed = ordered;
  rhs_changed[1].rhs = 5.5;

  const auto signature = detail::ordered_local_cut_signature(ordered);
  CHECK(signature == detail::ordered_local_cut_signature(ordered));
  CHECK(signature != detail::ordered_local_cut_signature(reordered));
  CHECK(signature != detail::ordered_local_cut_signature(rhs_changed));
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
  INFO("solver=" << res.stats.solver_name << " status=" << res.stats.status);
  REQUIRE(res.stats.success);
  // Solver should still find the global optimum (obj=9)
  CHECK(res.stats.objective == Approx(9.0).margin(1e-4));
  CHECK(res.stats.objective == Approx(mip.linear_part.c.dot(res.x)).margin(1e-8));
}

// ─── StrictHiGHS tests (improvements A, B, D) ─────────────────────────────
//
// These tests exercise the vendored-HiGHS LP kernel path
// (BCOptions::lp_kernel_backend = LpKernelBackend::HiGHS). They verify:
//
//   [D] mip_detect_symmetry=false – still finds the correct optimum.
//   [A+B] Warm-start via dispatch LP + setSolution() – still finds the correct
//         optimum, and nodes_explored with warm start <= nodes_explored cold.

TEST_CASE("MILP LP backend selection is independent of StrictHiGHS policy",
          "[milp][dispatch]") {
  BCOptions opt;
  REQUIRE(opt.lp_kernel_backend == LpKernelBackend::HiGHS);
  REQUIRE_FALSE(opt.strict_highs_mip_contract);
  REQUIRE_FALSE(opt.require_tree_exhaustion_certificate);
  REQUIRE_FALSE(opt.enable_reduced_cost_proof_cut_resolve);

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

TEST_CASE("MILP row propagators preserve small terms across cancellation",
          "[milp][propagation][numerics]") {
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

  auto initial_lb = [] {
    Eigen::VectorXd lb(4);
    lb << 1.0, 1.0, 1.0, 0.0;
    return lb;
  };
  auto initial_ub = [] {
    Eigen::VectorXd ub(4);
    ub << 1.0, 1.0, 1.0, 1.0;
    return ub;
  };

  Eigen::VectorXd lb = initial_lb();
  Eigen::VectorXd ub = initial_ub();
  CHECK(detail::node_bound_propagation(lp, lb, ub, 2) >= 1);
  CHECK(ub[3] == Approx(0.0));

  Eigen::SparseMatrix<double, Eigen::RowMajor> a_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> aeq_row = lp.Aeq;
  lb = initial_lb();
  ub = initial_ub();
  std::vector<BoundChangeInfo> tracked_changes;
  CHECK(detail::node_bound_propagation_tracked(
            lp, a_row, aeq_row, lb, ub, 2, tracked_changes) >= 1);
  CHECK(ub[3] == Approx(0.0));
  CHECK_FALSE(tracked_changes.empty());

  detail::RowPropagationIndex row_index;
  row_index.build(4, a_row, aeq_row);
  lb = initial_lb();
  ub = initial_ub();
  std::vector<BoundChangeInfo> domain_changes;
  const std::vector<detail::BranchDomainLiteral> branch_reasons{
      {0, 1.0, true}};
  int tightened = 0;
  REQUIRE(detail::propagate_node_domain(
      lp, a_row, aeq_row, row_index, lb, ub, 2,
      static_cast<const detail::ConflictPool*>(nullptr), nullptr, nullptr,
      domain_changes, branch_reasons, nullptr, &tightened, nullptr));
  CHECK(tightened >= 1);
  CHECK(ub[3] == Approx(0.0));
}

TEST_CASE("MILP domain propagation materializes reasons only on request",
          "[milp][propagation][reasons][performance]") {
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
  const std::vector<detail::BranchDomainLiteral> branch_reasons{
      {0, 1.0, true}};

  auto run = [&](std::vector<detail::DomainReasonBound>* reasons) {
    Eigen::VectorXd lb(2), ub(2);
    lb << 1.0, 0.0;
    ub << 1.0, 1.0;
    std::vector<BoundChangeInfo> changes;
    int tightened = 0;
    REQUIRE(detail::propagate_node_domain(
        lp, a_row, aeq_row, row_index, lb, ub, 2,
        static_cast<const detail::ConflictPool*>(nullptr), nullptr, nullptr,
        changes, branch_reasons, nullptr, &tightened, reasons));
    CHECK(ub[1] == Approx(0.0));
    CHECK(tightened >= 1);
  };

  detail::reset_propagation_profile();
  run(nullptr);
  CHECK(detail::get_propagation_profile().reason_clauses_materialized == 0);

  detail::reset_propagation_profile();
  std::vector<detail::DomainReasonBound> reasons;
  run(&reasons);
  CHECK_FALSE(reasons.empty());
  CHECK(detail::get_propagation_profile().reason_clauses_materialized > 0);
}

TEST_CASE("MILP row propagation honors a zero round budget",
          "[milp][propagation][round-budget]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(2);
  lp.A.resize(0, 2);
  lp.b.resize(0);
  lp.Aeq.resize(1, 2);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Constant(1, 1.0);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  Eigen::SparseMatrix<double, Eigen::RowMajor> a_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> aeq_row = lp.Aeq;
  detail::RowPropagationIndex row_index;
  row_index.build(2, a_row, aeq_row);
  const std::vector<detail::BranchDomainLiteral> branch_reasons{
      {0, 0.0, false}};

  Eigen::VectorXd lb(2), ub(2);
  lb << 0.0, 0.0;
  ub << 0.0, 1.0;
  std::vector<BoundChangeInfo> changes;
  std::vector<detail::DomainPropagationEvent> events;
  detail::reset_propagation_profile();
  REQUIRE(detail::propagate_node_domain(
      lp, a_row, aeq_row, row_index, lb, ub, 0,
      static_cast<const detail::ConflictPool*>(nullptr), nullptr, nullptr,
      changes, branch_reasons, nullptr, nullptr, nullptr, nullptr, &events));
  CHECK(lb[1] == Approx(0.0));
  CHECK(detail::get_propagation_profile().eq_rows_visited == 0);
  CHECK(events.empty());

  detail::BinaryImplicationGraph implications(2);
  REQUIRE(implications.add_implication(0, false, 1, true, 1.0));
  lb << 0.0, 0.0;
  ub << 0.0, 1.0;
  changes.clear();
  events.clear();
  REQUIRE(detail::propagate_node_domain(
      lp, a_row, aeq_row, row_index, lb, ub, 0,
      static_cast<const detail::ConflictPool*>(nullptr), nullptr,
      &implications, changes, branch_reasons, nullptr, nullptr, nullptr,
      nullptr, &events));
  CHECK(lb[1] == Approx(1.0));
  REQUIRE(events.size() == 1);
  CHECK(events.front().source ==
        detail::DomainPropagationSource::Implication);
}

TEST_CASE("MILP propagation reasons survive into child conflict analysis",
          "[milp][propagation][reasons][regression]") {
  LPModel lp;
  lp.sense = Sense::Minimize;
  lp.c = Eigen::VectorXd::Zero(3);
  lp.A.resize(0, 3);
  lp.b.resize(0);
  lp.Aeq.resize(2, 3);
  lp.Aeq.insert(0, 0) = 1.0;
  lp.Aeq.insert(0, 1) = 1.0;
  lp.Aeq.insert(1, 1) = 1.0;
  lp.Aeq.insert(1, 2) = 1.0;
  lp.Aeq.makeCompressed();
  lp.beq = Eigen::VectorXd::Ones(2);
  lp.vars = {{VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0},
             {VarType::Binary, 0.0, 1.0}};

  Eigen::SparseMatrix<double, Eigen::RowMajor> a_row = lp.A;
  Eigen::SparseMatrix<double, Eigen::RowMajor> aeq_row = lp.Aeq;
  detail::RowPropagationIndex row_index;
  row_index.build(3, a_row, aeq_row);

  Eigen::VectorXd parent_lb(3), parent_ub(3);
  parent_lb << 0.0, 0.0, 0.0;
  parent_ub << 0.0, 1.0, 1.0;
  const std::vector<detail::BranchDomainLiteral> parent_branches{
      {0, 0.0, false}};
  std::vector<BoundChangeInfo> parent_changes;
  std::vector<detail::DomainReasonBound> parent_reasons;
  REQUIRE(detail::propagate_node_domain(
      lp, a_row, aeq_row, row_index, parent_lb, parent_ub, 1,
      static_cast<const detail::ConflictPool*>(nullptr), nullptr, nullptr,
      parent_changes, parent_branches, nullptr, nullptr, &parent_reasons));
  CHECK(parent_lb[1] == Approx(1.0));
  CHECK(parent_lb[2] == Approx(0.0));
  REQUIRE_FALSE(parent_reasons.empty());

  Eigen::VectorXd child_lb = parent_lb;
  Eigen::VectorXd child_ub = parent_ub;
  child_lb[2] = 1.0;
  const std::vector<detail::BranchDomainLiteral> child_branches{
      {0, 0.0, false}, {2, 1.0, true}};
  std::vector<BoundChangeInfo> child_changes{
      {2, 1.0, true, 0.0, 1.0}};
  std::vector<detail::BranchDomainLiteral> learned_conflict;
  std::vector<detail::DomainReasonBound> child_reasons;
  CHECK_FALSE(detail::propagate_node_domain(
      lp, a_row, aeq_row, row_index, child_lb, child_ub, 1,
      static_cast<const detail::ConflictPool*>(nullptr), nullptr, nullptr,
      child_changes, child_branches, &learned_conflict, nullptr,
      &child_reasons, &parent_reasons));

  const auto contains_literal =
      [&](int variable, bool is_lb, double value) {
        return std::any_of(
            learned_conflict.begin(), learned_conflict.end(),
            [&](const detail::BranchDomainLiteral& literal) {
              return literal.var_idx == variable && literal.is_lb == is_lb &&
                     literal.value == Approx(value);
            });
      };
  CHECK(contains_literal(0, false, 0.0));
  CHECK(contains_literal(2, true, 1.0));

  const Eigen::Vector3d feasible_witness(1.0, 0.0, 1.0);
  const bool learned_clause_hits_witness = std::all_of(
      learned_conflict.begin(), learned_conflict.end(),
      [&](const detail::BranchDomainLiteral& literal) {
        return literal.is_lb
            ? feasible_witness[literal.var_idx] >= literal.value - 1e-9
            : feasible_witness[literal.var_idx] <= literal.value + 1e-9;
      });
  CHECK_FALSE(learned_clause_hits_witness);
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

TEST_CASE("StrictHiGHS adapter executes the HiGHS MIP contract",
          "[milp][dispatch][strict_highs]") {
  BCOptions opt;
  const BCOptions strict = make_strict_highs_production_options(opt);
  CHECK(strict.strict_highs_mip_contract);
  CHECK(strict.lp_kernel_backend == LpKernelBackend::HiGHS);
}

TEST_CASE("HiGHS root ownership preserves native tree ownership",
          "[milp][dispatch][highs_root]") {
  BCOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opt.highs_root_native_tree_contract = true;

  const auto state = detail::apply_bc_strict_highs_contract(opt);
  CHECK(state.requested_vendored_highs_lp);
  CHECK(state.strict_highs_lp_contract);
  CHECK(opt.lp_kernel_backend == LpKernelBackend::HiGHS);
  CHECK(opt.auto_highs_root_pipeline);
  CHECK_FALSE(opt.strict_highs_mip_contract);
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

static MIPModel make_complete_graph_vertex_cover_8() {
  constexpr int kVertices = 8;
  constexpr int kEdges = kVertices * (kVertices - 1) / 2;
  MIPModel mip;
  mip.linear_part.sense = Sense::Minimize;
  mip.linear_part.c = Eigen::VectorXd::Ones(kVertices);
  mip.linear_part.A.resize(kEdges, kVertices);
  mip.linear_part.b = Eigen::VectorXd::Constant(kEdges, -1.0);
  int row = 0;
  for (int i = 0; i < kVertices; ++i) {
    for (int j = i + 1; j < kVertices; ++j) {
      mip.linear_part.A.insert(row, i) = -1.0;
      mip.linear_part.A.insert(row, j) = -1.0;
      ++row;
    }
  }
  mip.linear_part.A.makeCompressed();
  mip.linear_part.Aeq.resize(0, kVertices);
  mip.linear_part.beq.resize(0);
  mip.linear_part.vars.assign(
      kVertices, VariableMeta{VarType::Binary, 0.0, 1.0});
  for (int i = 0; i < kVertices; ++i) mip.binary_idx.push_back(i);
  return mip;
}

TEST_CASE("MILP: objective clique events are not counted twice",
          "[milp][objective_propagation][correctness]") {
  constexpr int kVertices = 8;
  const MIPModel mip = make_complete_graph_vertex_cover_8();

  BCOptions options;
  options.lp_kernel_backend = LpKernelBackend::HiGHS;
  options.num_threads = 1;
  options.use_papilo_presolve = false;
  options.root_cut_rounds = 0;
  options.cuts = CutType::None;
  options.use_feasibility_pump = false;
  options.use_progressive_rounding = false;
  options.enable_feasibility_jump = false;
  options.max_dive_lps = 0;
  options.max_probe_vars = 0;
  options.root_split_bound_probing = false;
  options.enable_lns = false;
  options.enable_incumbent_local_branching = false;
  options.enable_root_low_fractionality_rens = false;
  options.incumbent_quality_reject_factor = 0.0;
  options.require_tree_exhaustion_certificate = true;

  const BCResult result = solve_milp_bc(mip, options);
  REQUIRE(result.stats.success);
  CHECK(result.stats.objective == Approx(7.0).margin(1e-7));
  REQUIRE(result.x.size() == kVertices);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(kVertices);
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(kVertices);
  CHECK(detail::satisfies_with_bounds(
      mip.linear_part, result.x, lower, upper, 1e-7));
}

TEST_CASE("MILP: production optimum matches exhaustive binary oracle",
          "[milp][differential][correctness]") {
  constexpr int kVariables = 6;
  constexpr int kRows = 5;
  constexpr int kCases = 128;
  std::mt19937_64 rng(0x8f3d9b71ULL);
  std::uniform_int_distribution<int> bit(0, 1);
  std::uniform_int_distribution<int> coeff(-4, 4);
  std::uniform_int_distribution<int> objective_coeff(-5, 5);
  std::uniform_int_distribution<int> slack(0, 3);

  for (int case_index = 0; case_index < kCases; ++case_index) {
    INFO("differential case=" << case_index);
    Eigen::VectorXd witness(kVariables);
    for (int j = 0; j < kVariables; ++j) witness[j] = bit(rng);

    MIPModel mip;
    LPModel& lp = mip.linear_part;
    lp.sense = case_index % 2 == 0 ? Sense::Minimize : Sense::Maximize;
    lp.c.resize(kVariables);
    bool nonzero_objective = false;
    for (int j = 0; j < kVariables; ++j) {
      const int value = objective_coeff(rng);
      lp.c[j] = value;
      nonzero_objective = nonzero_objective || value != 0;
    }
    if (!nonzero_objective) lp.c[0] = 1.0;

    std::vector<Eigen::Triplet<double>> rows;
    Eigen::MatrixXd dense_rows = Eigen::MatrixXd::Zero(kRows, kVariables);
    lp.row_lhs.resize(kRows);
    lp.b.resize(kRows);
    for (int i = 0; i < kRows; ++i) {
      bool nonzero_row = false;
      for (int j = 0; j < kVariables; ++j) {
        const int value = coeff(rng);
        if (value == 0) continue;
        dense_rows(i, j) = value;
        rows.emplace_back(i, j, value);
        nonzero_row = true;
      }
      if (!nonzero_row) {
        dense_rows(i, i % kVariables) = 1.0;
        rows.emplace_back(i, i % kVariables, 1.0);
      }
      const double activity = dense_rows.row(i).dot(witness);
      if (i % 3 == 0) {
        lp.row_lhs[i] = -std::numeric_limits<double>::infinity();
        lp.b[i] = activity + slack(rng);
      } else if (i % 3 == 1) {
        lp.row_lhs[i] = activity - slack(rng);
        lp.b[i] = std::numeric_limits<double>::infinity();
      } else {
        lp.row_lhs[i] = activity - slack(rng);
        lp.b[i] = activity + slack(rng);
      }
    }
    lp.A.resize(kRows, kVariables);
    lp.A.setFromTriplets(rows.begin(), rows.end());
    lp.A.makeCompressed();

    if (case_index % 3 == 0) {
      std::vector<Eigen::Triplet<double>> equality;
      Eigen::VectorXd dense_equality = Eigen::VectorXd::Zero(kVariables);
      for (int j = 0; j < kVariables; ++j) {
        if (!bit(rng)) continue;
        const int value = coeff(rng);
        if (value == 0) continue;
        dense_equality[j] = value;
        equality.emplace_back(0, j, value);
      }
      if (equality.empty()) {
        dense_equality[case_index % kVariables] = 1.0;
        equality.emplace_back(0, case_index % kVariables, 1.0);
      }
      lp.Aeq.resize(1, kVariables);
      lp.Aeq.setFromTriplets(equality.begin(), equality.end());
      lp.Aeq.makeCompressed();
      lp.beq = Eigen::VectorXd::Constant(1, dense_equality.dot(witness));
    } else {
      lp.Aeq.resize(0, kVariables);
      lp.beq.resize(0);
    }
    lp.vars.assign(
        kVariables, VariableMeta{VarType::Binary, 0.0, 1.0});
    for (int j = 0; j < kVariables; ++j) mip.binary_idx.push_back(j);

    double oracle = lp.sense == Sense::Minimize
                        ? std::numeric_limits<double>::infinity()
                        : -std::numeric_limits<double>::infinity();
    for (int assignment = 0; assignment < (1 << kVariables); ++assignment) {
      Eigen::VectorXd x(kVariables);
      for (int j = 0; j < kVariables; ++j) {
        x[j] = (assignment >> j) & 1;
      }
      bool feasible = true;
      for (int i = 0; i < kRows && feasible; ++i) {
        const double activity = dense_rows.row(i).dot(x);
        feasible = activity >= lp.row_lhs[i] - 1e-9 &&
                   activity <= lp.b[i] + 1e-9;
      }
      if (feasible && lp.Aeq.rows() == 1) {
        feasible = std::abs(lp.Aeq.row(0).dot(x) - lp.beq[0]) <= 1e-9;
      }
      if (!feasible) continue;
      const double value = lp.c.dot(x);
      oracle = lp.sense == Sense::Minimize ? std::min(oracle, value)
                                           : std::max(oracle, value);
    }
    REQUIRE(std::isfinite(oracle));

    BCOptions options;
    options.lp_kernel_backend = LpKernelBackend::HiGHS;
    options.num_threads = 1;
    options.use_papilo_presolve = case_index % 2 == 0;
    options.require_tree_exhaustion_certificate = true;
    options.random_seed = 0;
    INFO("sense=" << (lp.sense == Sense::Minimize ? "min" : "max"));
    INFO("objective=" << lp.c.transpose());
    INFO("witness=" << witness.transpose());
    INFO("rows=\n" << dense_rows);
    INFO("row_lhs=" << lp.row_lhs.transpose());
    INFO("row_rhs=" << lp.b.transpose());
    INFO("equality_rows=" << lp.Aeq.rows());
    if (lp.Aeq.rows() == 1) {
      UNSCOPED_INFO("equality=" << Eigen::MatrixXd(lp.Aeq));
      UNSCOPED_INFO("equality_rhs=" << lp.beq.transpose());
    }
    const BCResult result = solve_milp_bc(mip, options);

    INFO("status=" << result.stats.status << " oracle=" << oracle);
    REQUIRE(result.stats.success);
    CHECK(result.stats.objective == Approx(oracle).margin(1e-7));
    if (case_index == 112) {
      INFO("regression: direct and implied-event objective contributions share "
           "target x3");
      CHECK(oracle == Approx(-10.0).margin(1e-9));
      CHECK(result.stats.objective == Approx(-10.0).margin(1e-7));
    }
    REQUIRE(result.x.size() == kVariables);
    const Eigen::VectorXd lower = Eigen::VectorXd::Zero(kVariables);
    const Eigen::VectorXd upper = Eigen::VectorXd::Ones(kVariables);
    CHECK(detail::satisfies_with_bounds(lp, result.x, lower, upper, 1e-7));
    for (int j = 0; j < kVariables; ++j) {
      CHECK(result.x[j] == Approx(std::round(result.x[j])).margin(1e-7));
    }
  }
}

TEST_CASE("MILP: production tree restart rebuilds the root and preserves the answer",
          "[milp][restart][integration]") {
  constexpr int kVertices = 8;
  const MIPModel mip = make_complete_graph_vertex_cover_8();

  BCOptions baseline;
  baseline.lp_kernel_backend = LpKernelBackend::HiGHS;
  baseline.num_threads = 1;
  baseline.use_papilo_presolve = false;
  baseline.root_cut_rounds = 0;
  baseline.cuts = CutType::None;
  baseline.use_feasibility_pump = false;
  baseline.use_progressive_rounding = false;
  baseline.enable_feasibility_jump = false;
  baseline.max_dive_lps = 0;
  baseline.max_probe_vars = 0;
  baseline.root_split_bound_probing = false;
  baseline.enable_lns = false;
  baseline.enable_incumbent_local_branching = false;
  baseline.enable_root_low_fractionality_rens = false;
  baseline.incumbent_quality_reject_factor = 0.0;
  baseline.require_tree_exhaustion_certificate = true;
  // This test needs an incumbent improvement while the frontier remains open.
  // Objective propagation proves the K8 lower bound at the root, so disable
  // that independent feature to exercise the production restart transaction.
  baseline.enable_objective_cutoff_conflict_cuts = false;
  baseline.enable_objective_cutoff_weighted_event_cuts = false;
  baseline.enable_objective_cutoff_domain_fixing = false;
  baseline.enable_reduced_cost_proof_conflict_minimization = false;
  baseline.enable_nonviolated_cutoff_conflict_covers = false;

  const BCResult without_restart = solve_milp_bc(mip, baseline);
  REQUIRE(without_restart.stats.success);
  CHECK(without_restart.stats.objective == Approx(7.0).margin(1e-7));
  CHECK(without_restart.bc_stats.tree_restarts == 0);

  BCOptions restarted_options = baseline;
  restarted_options.tree_restart.enabled = true;
  restarted_options.tree_restart.max_restarts = 1;
  restarted_options.tree_restart.min_nodes_since_restart = 0;
  restarted_options.tree_restart.min_open_nodes = 1;
  restarted_options.tree_restart.min_relative_incumbent_improvement = 0.0;
  restarted_options.tree_restart.min_remaining_time_sec = 0.0;
  const BCResult restarted = solve_milp_bc(mip, restarted_options);

  REQUIRE(restarted.stats.success);
  CHECK(restarted.stats.objective ==
        Approx(without_restart.stats.objective).margin(1e-7));
  REQUIRE(restarted.x.size() == kVertices);
  Eigen::VectorXd lower = Eigen::VectorXd::Zero(kVertices);
  Eigen::VectorXd upper = Eigen::VectorXd::Ones(kVertices);
  CHECK(detail::satisfies_with_bounds(
      mip.linear_part, restarted.x, lower, upper, 1e-7));
  CHECK(restarted.bc_stats.tree_restarts == 1);
  CHECK(restarted.bc_stats.tree_restart_nodes_discarded > 0);
  CHECK(restarted.bc_stats.tree_restart_root_requeues == 1);
  CHECK(restarted.bc_stats.tree_restart_last_node >= 0);
  CHECK_FALSE(restarted.bc_stats.tree_restart_last_source.empty());
}

TEST_CASE("MILP: production solve publishes queued-domain and COW telemetry",
          "[milp][native][memory][telemetry]") {
  BCOptions opt;
  opt.lp_kernel_backend = LpKernelBackend::ExperimentalNative;
  opt.use_papilo_presolve = false;
  // This fixture measures child-domain copy-on-write telemetry and therefore
  // requires a nontrivial tree. Root separation is covered independently.
  opt.cuts = CutType::None;
  opt.root_cut_rounds = 0;
  opt.use_feasibility_pump = false;
  opt.use_progressive_rounding = false;
  opt.enable_feasibility_jump = false;

  MIPModel mip;
  mip.linear_part.sense = Sense::Maximize;
  mip.linear_part.c.resize(3);
  mip.linear_part.c << 1.0, 1.1, 1.2;
  mip.linear_part.A.resize(3, 3);
  mip.linear_part.A.insert(0, 0) = 1.0;
  mip.linear_part.A.insert(0, 1) = 1.0;
  mip.linear_part.A.insert(1, 1) = 1.0;
  mip.linear_part.A.insert(1, 2) = 1.0;
  mip.linear_part.A.insert(2, 0) = 1.0;
  mip.linear_part.A.insert(2, 2) = 1.0;
  mip.linear_part.A.makeCompressed();
  mip.linear_part.b = Eigen::VectorXd::Ones(3);
  mip.linear_part.Aeq.resize(0, 3);
  mip.linear_part.beq.resize(0);
  mip.linear_part.vars.assign(
      3, VariableMeta{VarType::Binary, 0.0, 1.0});
  mip.binary_idx = {0, 1, 2};

  const auto result = solve_milp_bc(mip, opt);
  REQUIRE(result.stats.success);
  CHECK(result.stats.objective == Approx(1.2).margin(1e-7));
  CHECK(result.bc_stats.node_queue_domain_compactions > 0);
  CHECK(result.bc_stats.node_queue_domain_materializations > 0);
  CHECK(result.bc_stats.node_queue_dense_bound_values_released > 0);
  CHECK(result.bc_stats.node_queue_domain_compactions ==
        result.bc_stats.node_queue_serial_compactions +
            result.bc_stats.node_queue_parallel_compactions +
            result.bc_stats.node_queue_submip_compactions);
  CHECK(result.bc_stats.node_queue_domain_compaction_failures == 0);
  CHECK(result.bc_stats.node_queue_domain_materialization_failures == 0);
  CHECK(result.bc_stats.branch_payload_child_creations > 0);
  CHECK(result.bc_stats.branch_payload_shared_vectors <=
        9 * result.bc_stats.branch_payload_child_creations);
  CHECK(result.bc_stats.branch_domain_dense_copies > 0);
  CHECK(result.bc_stats.branch_domain_dense_values_copied >=
        2 * result.bc_stats.branch_domain_dense_copies);
  CHECK(result.bc_stats.branch_domain_moves > 0);

  BCOptions probe_opt = opt;
  probe_opt.lp_kernel_backend = LpKernelBackend::HiGHS;
  probe_opt.ipm_root_probe = false;
  probe_opt.probe_reliability = 4;
  probe_opt.probe_max_candidates = 3;
  probe_opt.require_tree_exhaustion_certificate = true;
  probe_opt.enable_objective_cutoff_conflict_cuts = false;
  probe_opt.enable_objective_cutoff_weighted_event_cuts = false;
  probe_opt.enable_objective_cutoff_domain_fixing = false;
  probe_opt.enable_reduced_cost_proof_conflict_minimization = false;
  probe_opt.enable_nonviolated_cutoff_conflict_covers = false;
  MIPModel probe_mip;
  probe_mip.linear_part.sense = Sense::Minimize;
  probe_mip.linear_part.c = Eigen::VectorXd::Zero(5);
  probe_mip.linear_part.A.resize(0, 5);
  probe_mip.linear_part.b.resize(0);
  probe_mip.linear_part.Aeq.resize(1, 5);
  for (int col = 0; col < 5; ++col) {
    probe_mip.linear_part.Aeq.insert(0, col) = 1.0;
  }
  probe_mip.linear_part.Aeq.makeCompressed();
  probe_mip.linear_part.beq = Eigen::VectorXd::Constant(1, 2.5);
  probe_mip.linear_part.vars.assign(
      5, VariableMeta{VarType::Binary, 0.0, 1.0});
  probe_mip.binary_idx = {0, 1, 2, 3, 4};
  const auto probe_result = solve_milp_bc(probe_mip, probe_opt);
  CHECK_FALSE(probe_result.stats.success);
  CHECK(probe_result.bc_stats.root_domain_probe_workspace_initializations ==
        1);
  CHECK(probe_result.bc_stats.root_domain_probe_worlds > 1);
  CHECK(probe_result.bc_stats.root_domain_probe_worlds ==
        probe_result.bc_stats.root_domain_probe_rollbacks);
  CHECK(probe_result.bc_stats.root_domain_probe_failures == 0);
  CHECK(probe_result.bc_stats.root_domain_probe_trail_pushes >=
        probe_result.bc_stats.root_domain_probe_worlds);
  CHECK(probe_result.bc_stats.root_domain_probe_changed_columns <=
        probe_result.bc_stats.root_domain_probe_trail_pushes);
  CHECK(probe_result.bc_stats
            .root_domain_probe_implication_export_columns_visited <=
        probe_result.bc_stats.root_domain_probe_changed_columns);
  CHECK(probe_result.bc_stats.root_domain_probe_rows_processed > 0);
  CHECK(probe_result.bc_stats.root_lp_probe_bound_workspace_initializations ==
        1);
  CHECK(probe_result.bc_stats.root_lp_probe_bound_transactions > 1);
  CHECK(probe_result.bc_stats.root_lp_probe_bound_transactions ==
        probe_result.bc_stats.root_lp_probe_transaction_rollbacks);
  CHECK(probe_result.bc_stats.root_lp_probe_transaction_failures == 0);
  CHECK(probe_result.bc_stats.root_lp_probe_transaction_snapshot_values >=
        3 * probe_result.bc_stats.root_lp_probe_bound_transactions);
  CHECK(probe_result.bc_stats.root_lp_probe_backend_cold_solves >= 1);
  CHECK(probe_result.bc_stats.root_lp_probe_backend_persistent_resolves >= 1);
  CHECK(probe_result.bc_stats.root_lp_probe_backend_cold_solves +
            probe_result.bc_stats.root_lp_probe_backend_persistent_resolves ==
        probe_result.bc_stats.root_lp_probe_bound_transactions);
  CHECK(probe_result.bc_stats.strong_probe_bound_transactions > 0);
  CHECK(probe_result.bc_stats.strong_probe_bound_transactions ==
        probe_result.bc_stats.strong_probe_transaction_rollbacks);
  CHECK(probe_result.bc_stats.strong_probe_transaction_failures == 0);
  CHECK(probe_result.bc_stats.strong_probe_transaction_snapshot_values >=
        3 * probe_result.bc_stats.strong_probe_bound_transactions);
  CHECK(probe_result.bc_stats.strong_probe_backend_cold_solves >= 1);
  CHECK(probe_result.bc_stats.strong_probe_backend_persistent_resolves >= 1);
  CHECK(probe_result.bc_stats.strong_probe_backend_cold_solves +
            probe_result.bc_stats.strong_probe_backend_persistent_resolves ==
        probe_result.bc_stats.strong_branch_lp_solves);
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

TEST_CASE("MILP: extended API installs primal hints and computes real features",
          "[milp][api][warmstart][callbacks]") {
  MIPModel mip = make_knapsack_10();
  BCOptions opt;
  const auto cold = solve_milp_bc(mip, opt);
  REQUIRE(cold.stats.success);

  BCWarmStart warm_start;
  BCPrimalHint hint;
  hint.x = cold.x;
  warm_start.primal_hints.push_back(std::move(hint));

  bool tuner_called = false;
  BCInstanceFeatures observed;
  BCCallbacks callbacks;
  callbacks.hyperparam_tuner =
      [&](const BCInstanceFeatures& features, const BCOptions& base,
          const BCStats*) {
        tuner_called = true;
        observed = features;
        return base;
      };

  const auto warm = solve_milp_bc(mip, opt, warm_start, callbacks);
  REQUIRE(warm.stats.success);
  CHECK(warm.stats.objective == Approx(cold.stats.objective).margin(1e-6));
  CHECK(warm.stats.objective ==
        Approx(mip.linear_part.c.dot(warm.x)).margin(1e-8));
  CHECK(tuner_called);
  CHECK(observed.n_vars == 10);
  CHECK(observed.n_rows == 1);
  CHECK(observed.n_nnz == 10);
  CHECK(observed.row_density_median == Approx(10.0));
  CHECK(observed.obj_density > 0.0);
  CHECK(observed.obj_range_ratio > 1.0);
  CHECK(observed.rhs_range_ratio == Approx(1.0));
  CHECK(observed.coeff_range_ratio > 1.0);
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
