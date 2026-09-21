#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"
#include "mipsolvers/l2o/branching_policy.hpp"
#include "mipsolvers/l2o/feature_schema.hpp"
#include "mipsolvers/l2o/model_fingerprint.hpp"
#include "mipsolvers/l2o/policy.hpp"
#include "mipsolvers/l2o/scuc_warm_start.hpp"
#include "mipsolvers/l2o/solver_config_policy.hpp"
#include "mipsolvers/l2o/trace_event.hpp"
#include "mipsolvers/l2o/trace_writer.hpp"

using namespace mipsolvers;

namespace {

engine::MIPModel small_mip() {
  engine::MIPModel mip;
  mip.linear_part.c.resize(2);
  mip.linear_part.c << 1.0, 2.0;
  mip.linear_part.vars.push_back({engine::VarType::Binary, 0.0, 1.0, "x"});
  mip.linear_part.vars.push_back({engine::VarType::Continuous, 0.0, 10.0, "y"});
  mip.linear_part.A.resize(1, 2);
  mip.linear_part.A.insert(0, 0) = 1.0;
  mip.linear_part.A.insert(0, 1) = 2.0;
  mip.linear_part.A.makeCompressed();
  mip.linear_part.b.resize(1);
  mip.linear_part.b << 4.0;
  mip.linear_part.Aeq.resize(1, 2);
  mip.linear_part.Aeq.insert(0, 1) = 1.0;
  mip.linear_part.Aeq.makeCompressed();
  mip.linear_part.beq.resize(1);
  mip.linear_part.beq << 2.0;
  mip.binary_idx = {0};

  engine::MIPModel::UCGenHint hint;
  hint.ng = 1;
  hint.T = 1;
  hint.ig_start = 0;
  hint.su_start = 1;
  hint.sd_start = 1;
  hint.ig_cols = {0};
  hint.pg_cols = {1};
  hint.certifies_generation_capacity_rows = true;
  mip.uc_hint = hint;
  return mip;
}

engine::MIPModel small_scuc_mip_for_warm_start() {
  engine::MIPModel mip;
  mip.linear_part.c = Eigen::VectorXd::Zero(18);
  mip.linear_part.A.resize(0, 18);
  mip.linear_part.Aeq.resize(0, 18);
  mip.linear_part.vars.resize(18);
  for (auto& var : mip.linear_part.vars) var.type = engine::VarType::Binary;
  mip.initial_solution = Eigen::VectorXd::Constant(18, 0.25);

  engine::MIPModel::UCGenHint hint;
  hint.ng = 2;
  hint.T = 3;
  hint.ig0 = {0, 1};
  hint.ig_cols.resize(6);
  hint.su_cols.resize(6);
  hint.sd_cols.resize(6);
  for (int t = 0; t < hint.T; ++t) {
    for (int g = 0; g < hint.ng; ++g) {
      const size_t pos = static_cast<size_t>(t * hint.ng + g);
      hint.ig_cols[pos] = g * hint.T + t;
      hint.su_cols[pos] = 6 + g * hint.T + t;
      hint.sd_cols[pos] = 12 + g * hint.T + t;
    }
  }
  mip.uc_hint = hint;
  return mip;
}

std::vector<std::string> read_lines(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(stream, line)) lines.push_back(line);
  return lines;
}

}  // namespace

TEST_CASE("L2O feature schema validates names and serializes", "[l2o]") {
  l2o::FeatureSchema schema;
  schema.name = "unit-test-schema";
  schema.features.push_back({"num_variables", l2o::FeatureTarget::Instance, l2o::FeatureDType::Int64,
                             "Number of variables", "count", true, 0});
  schema.features.push_back({"fractionality", l2o::FeatureTarget::Variable, l2o::FeatureDType::Float64,
                             "LP fractionality", "", false, 0.0});

  CHECK(l2o::validate_schema(schema).empty());
  CHECK(l2o::schema_has_feature(schema, l2o::FeatureTarget::Instance, "num_variables"));

  const nlohmann::json encoded = schema;
  const auto decoded = encoded.get<l2o::FeatureSchema>();
  CHECK(decoded.name == schema.name);
  CHECK(decoded.features.size() == 2);
  CHECK(decoded.features[0].dtype == l2o::FeatureDType::Int64);

  schema.features.push_back(schema.features.front());
  CHECK_FALSE(l2o::validate_schema(schema).empty());
}

TEST_CASE("L2O policy metadata serializes safety mode", "[l2o]") {
  l2o::PolicyMetadata metadata;
  metadata.name = "warm-start-commitment";
  metadata.version = "1";
  metadata.mode = l2o::PolicyMode::TraceOnly;
  metadata.max_inference_time_sec = 0.01;

  const nlohmann::json encoded = metadata;
  CHECK(encoded.at("mode") == "trace_only");
  const auto decoded = encoded.get<l2o::PolicyMetadata>();
  CHECK(decoded.mode == l2o::PolicyMode::TraceOnly);
  CHECK(decoded.name == metadata.name);
}

TEST_CASE("L2O trace writer emits JSONL header and events", "[l2o]") {
  const auto path = std::filesystem::temp_directory_path() / "mipsolvers_l2o_trace_test.jsonl";
  std::filesystem::remove(path);

  l2o::TraceWriterOptions options;
  options.run_id = "run-unit";
  options.metadata = nlohmann::json{{"case", "trace-writer"}};

  {
    l2o::TraceWriter writer(path, options);
    writer.write(l2o::TraceEventType::InstanceStart,
                 nlohmann::json{{"num_variables", 2}},
                 "instance-a",
                 0.25);
    CHECK(writer.events_written() == 2);
  }

  const auto lines = read_lines(path);
  REQUIRE(lines.size() == 2);

  const auto header = nlohmann::json::parse(lines[0]).get<l2o::TraceEvent>();
  CHECK(header.type == l2o::TraceEventType::TraceHeader);
  CHECK(header.sequence == 0);
  CHECK(header.run_id == "run-unit");
  CHECK(header.payload.at("trace_schema_version") == "mipsolvers.l2o.trace.v1");

  const auto event = nlohmann::json::parse(lines[1]).get<l2o::TraceEvent>();
  CHECK(event.type == l2o::TraceEventType::InstanceStart);
  CHECK(event.sequence == 1);
  CHECK(event.run_id == "run-unit");
  CHECK(event.instance_id == "instance-a");
  CHECK(event.payload.at("num_variables") == 2);

  std::filesystem::remove(path);
}

TEST_CASE("L2O model fingerprint captures structure and optional numeric values", "[l2o]") {
  auto mip = small_mip();
  const auto structural = l2o::fingerprint_mip(mip);
  const auto structural_again = l2o::fingerprint_mip(mip);

  CHECK(l2o::compatible_fingerprint(structural, structural_again));
  CHECK(structural.num_variables == 2);
  CHECK(structural.num_ineq_rows == 1);
  CHECK(structural.num_eq_rows == 1);
  CHECK(structural.num_binary_vars == 1);
  CHECK(structural.has_uc_hint);
  CHECK(structural.metadata.at("uc_hint").at("ng") == 1);

  auto changed_numeric = mip;
  changed_numeric.linear_part.A.coeffRef(0, 1) = 3.0;
  changed_numeric.linear_part.A.makeCompressed();
  CHECK(l2o::fingerprint_mip(changed_numeric).value == structural.value);

  l2o::FingerprintOptions numeric_options;
  numeric_options.include_matrix_values = true;
  CHECK(l2o::fingerprint_mip(changed_numeric, numeric_options).value !=
        l2o::fingerprint_mip(mip, numeric_options).value);

  const nlohmann::json encoded = structural;
  const auto decoded = encoded.get<l2o::ModelFingerprint>();
  CHECK(decoded.value == structural.value);
}

TEST_CASE("L2O SCUC warm start maps commitment predictions to UC columns", "[l2o]") {
  auto mip = small_scuc_mip_for_warm_start();
  const scuc::Matrix2D commitment{{0.8, 0.7, 0.2}, {0.1, 0.9, 0.9}};

  const auto result = l2o::make_scuc_commitment_warm_start(mip, commitment);
  CHECK(result.report.success);
  CHECK(result.report.used_existing_seed);
  CHECK(result.report.inferred_startup);
  CHECK(result.report.inferred_shutdown);
  CHECK(result.report.num_ig_set == 6);
  CHECK(result.report.num_su_set == 6);
  CHECK(result.report.num_sd_set == 6);
  CHECK(result.report.num_missing_cols == 0);

  CHECK(result.x[0] == 1.0);
  CHECK(result.x[1] == 1.0);
  CHECK(result.x[2] == 0.0);
  CHECK(result.x[3] == 0.0);
  CHECK(result.x[4] == 1.0);
  CHECK(result.x[5] == 1.0);

  CHECK(result.x[6] == 1.0);
  CHECK(result.x[7] == 0.0);
  CHECK(result.x[8] == 0.0);
  CHECK(result.x[9] == 0.0);
  CHECK(result.x[10] == 1.0);
  CHECK(result.x[11] == 0.0);

  CHECK(result.x[12] == 0.0);
  CHECK(result.x[13] == 0.0);
  CHECK(result.x[14] == 1.0);
  CHECK(result.x[15] == 1.0);
  CHECK(result.x[16] == 0.0);
  CHECK(result.x[17] == 0.0);
}

TEST_CASE("L2O SCUC branching policy maps learned scores to UC priorities", "[l2o]") {
  auto mip = small_scuc_mip_for_warm_start();
  const scuc::Matrix2D scores{{0.0, 0.0, 0.0}, {0.0, 0.0, 10.0}};

  l2o::SCUCBranchingPolicyOptions options;
  options.prefer_earlier_periods = false;
  options.clear_existing_priorities = true;

  l2o::SCUCBranchingPriorityReport report;
  const auto priorities = l2o::make_scuc_branching_priorities(
      mip, scores, options, &report);

  REQUIRE(report.success);
  CHECK(report.used_uc_hint);
  CHECK(report.used_learned_scores);
  CHECK(report.installed_static_priorities);
  CHECK(report.num_commitment_priorities == 6);
  CHECK(report.num_transition_priorities == 12);
  CHECK(report.num_missing_cols == 0);
  REQUIRE(priorities.size() == 18);

  const int high_ig_col = 5;
  const int low_ig_col = 0;
  const int high_su_col = 11;
  const int low_su_col = 6;
  const int high_sd_col = 17;
  const int low_sd_col = 12;

  CHECK(priorities[high_ig_col] > priorities[low_ig_col]);
  CHECK(priorities[high_su_col] > priorities[low_su_col]);
  CHECK(priorities[high_sd_col] > priorities[low_sd_col]);
}

TEST_CASE("L2O dynamic branch prior reranks equal-priority candidates", "[l2o]") {
  engine::BCOptions options;
  options.branching = engine::BranchingStrategy::Pseudocost;
  const std::vector<int> candidates{0, 1};
  Eigen::VectorXd x(2);
  x << 0.5, 0.5;
  std::vector<engine::detail::PseudoCost> pseudocosts(2);
  const std::vector<int> priorities{0, 0};

  engine::BCBranchContext context;
  context.depth = 3;
  context.candidates = &candidates;
  context.lp_x = x.data();
  context.lp_x_size = static_cast<std::size_t>(x.size());
  engine::BCBranchingPriorFn prefer_second =
      [](const engine::BCBranchContext&, std::vector<double>& out_scores) {
        out_scores = {0.0, 10.0};
      };

  const int chosen = engine::detail::choose_branch_var(
      options, candidates, x, pseudocosts, priorities, prefer_second, context);
  CHECK(chosen == 1);
}

TEST_CASE("L2O dynamic branch prior uses original-space candidate context", "[l2o]") {
  engine::BCOptions options;
  options.branching = engine::BranchingStrategy::Pseudocost;
  const std::vector<int> reduced_candidates{0, 1};
  const std::vector<int> original_candidates{10, 11};
  Eigen::VectorXd reduced_x(2);
  reduced_x << 0.5, 0.5;
  Eigen::VectorXd original_x = Eigen::VectorXd::Zero(12);
  original_x[10] = 0.5;
  original_x[11] = 0.5;
  std::vector<engine::detail::PseudoCost> pseudocosts(2);
  const std::vector<int> reduced_priorities{0, 0};

  engine::BCBranchContext context;
  context.candidates = &original_candidates;
  context.lp_x = original_x.data();
  context.lp_x_size = static_cast<std::size_t>(original_x.size());
  engine::BCBranchingPriorFn original_space_prior =
      [](const engine::BCBranchContext& ctx, std::vector<double>& out_scores) {
        REQUIRE(ctx.candidates != nullptr);
        REQUIRE(ctx.candidates->size() == 2);
        CHECK((*ctx.candidates)[0] == 10);
        CHECK((*ctx.candidates)[1] == 11);
        out_scores.assign(12, 0.0);
        out_scores[11] = 10.0;
      };

  const int chosen = engine::detail::choose_branch_var(
      options, reduced_candidates, reduced_x, pseudocosts, reduced_priorities,
      original_space_prior, context);
  CHECK(chosen == 1);
}

TEST_CASE("L2O solver configuration policy tunes SCUC-like short-budget runs", "[l2o]") {
  auto mip = small_scuc_mip_for_warm_start();
  mip.binary_idx.resize(18);
  for (int i = 0; i < 18; ++i) mip.binary_idx[static_cast<size_t>(i)] = i;

  engine::BCOptions base;
  base.time_limit_sec = 1.0;
  base.num_threads = 4;
  base.deterministic_parallel = false;
  base.highs_mip_heuristic_effort = 0.05;
  base.highs_max_root_sepa_rounds = 0;

  l2o::SolverConfigPolicyOptions policy;
  policy.scuc_min_binary_vars = 1;
  policy.scuc_min_rows = 0;

  const auto features = l2o::extract_solver_config_features(mip, base);
  const auto decision = l2o::choose_solver_config(features, base, policy);
  CHECK(decision.profile == "scuc_short_budget");
  CHECK(decision.changed_options);
  CHECK(decision.options.num_threads == 1);
  CHECK(decision.options.deterministic_parallel);
  CHECK(decision.options.highs_mip_run_zi_round);
  CHECK(decision.options.highs_mip_run_shifting);
  CHECK(decision.options.highs_mip_heuristic_effort >= 0.30);
  CHECK(decision.options.highs_max_root_sepa_rounds == policy.short_budget_root_sepa_rounds);

  const nlohmann::json encoded = decision;
  CHECK(encoded.at("profile") == "scuc_short_budget");
  CHECK(encoded.at("features").at("has_uc_hint") == true);
}

TEST_CASE("L2O artifact cache validates fingerprint and applies reusable payloads", "[l2o]") {
  auto mip = small_mip();
  engine::BCResult result;
  result.highs_root_cuts = std::make_shared<engine::BCRootCuts>();
  result.highs_root_cuts->start = {0, 2};
  result.highs_root_cuts->index = {0, 1};
  result.highs_root_cuts->value = {1.0, -1.0};
  result.highs_root_cuts->lower = {-1e30};
  result.highs_root_cuts->upper = {0.0};

  result.highs_root_basis = std::make_shared<engine::BCRootBasis>();
  result.highs_root_basis->valid = true;
  result.highs_root_basis->col_status = {0, 1};
  result.highs_root_basis->row_status = {0, 1, 2};

  result.highs_pseudocost_init = std::make_shared<engine::BCPseudocostInit>();
  result.highs_pseudocost_init->n_orig_cols = 2;
  result.highs_pseudocost_init->pseudocostup = {1.0, 2.0};
  result.highs_pseudocost_init->pseudocostdown = {1.5, 2.5};

  l2o::SolverArtifactCache cache;
  cache.store(mip, result);
  CHECK(cache.size() == 1);

  engine::BCOptions options;
  const auto report = cache.apply(mip, options);
  CHECK(report.compatible);
  CHECK(report.root_cuts_used);
  CHECK(report.root_basis_used);
  CHECK(report.pseudocost_used);
  CHECK(options.highs_root_cut_warm_start != nullptr);
  CHECK(options.highs_root_basis_warm_start != nullptr);
  CHECK(options.highs_pseudocost_warm_start != nullptr);

  auto mismatch = mip;
  mismatch.linear_part.c.resize(3);
  engine::BCOptions mismatch_options;
  const auto mismatch_report = cache.apply(mismatch, mismatch_options);
  CHECK_FALSE(mismatch_report.compatible);
  CHECK_FALSE(mismatch_options.highs_root_cut_warm_start);
}

TEST_CASE("L2O solver config callbacks tune and record runs", "[l2o]") {
  l2o::SolverConfigPolicyOptions policy;
  policy.scuc_min_binary_vars = 1;
  policy.scuc_min_rows = 0;
  std::vector<l2o::SolverConfigRunRecord> records;
  auto callbacks = l2o::make_solver_config_callbacks(policy, &records, "unit-policy");

  engine::BCInstanceFeatures features;
  features.n_vars = 10;
  features.n_bin = 6;
  features.n_rows = 20;
  features.extra = {1.0, 2.0, 3.0, 0.0, 0.0, 1.0};

  engine::BCOptions base;
  base.time_limit_sec = 0.5;
  const auto tuned = callbacks.hyperparam_tuner(features, base, nullptr);
  CHECK(tuned.highs_mip_run_zi_round);
  CHECK(tuned.highs_max_root_sepa_rounds == policy.short_budget_root_sepa_rounds);

  engine::BCStats stats;
  stats.status = "unit";
  stats.gap = 0.25;
  stats.nodes_explored = 7;
  callbacks.post_solve(features, tuned, stats);

  REQUIRE(records.size() == 1);
  CHECK(records.front().policy_tag == "unit-policy");
  CHECK(records.front().features.has_uc_hint);
  CHECK(records.front().stats.gap == 0.25);
}