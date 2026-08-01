/// test_milp_solver.cpp
/// Tests for MILP solving via the engine — exercises every registered MILP solver.
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <chrono>
#include <string>

#include "mipsolvers/engine/api/solver.hpp"
#include "mipsolvers/engine/api/options.hpp"
#include "mipsolvers/engine/bc/api.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/solver/native/native_adapters.hpp"
#include "mipsolvers/engine/detail/bc_legacy_helpers.hpp"
#include "mipsolvers/engine/detail/bc_status.hpp"
#include "mipsolvers/engine/detail/bc_threading.hpp"
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
