#include "mipsolvers/l2o/solver_config_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace mipsolvers::l2o {
namespace {

int model_num_cols(const engine::MIPModel& mip) {
  return static_cast<int>(mip.linear_part.c.size());
}

int model_num_rows(const engine::MIPModel& mip) {
  return static_cast<int>(mip.linear_part.A.rows() + mip.linear_part.Aeq.rows());
}

int model_num_eq_rows(const engine::MIPModel& mip) {
  return static_cast<int>(mip.linear_part.Aeq.rows());
}

int model_num_nonzeros(const engine::MIPModel& mip) {
  return static_cast<int>(mip.linear_part.A.nonZeros() + mip.linear_part.Aeq.nonZeros());
}

bool has_valid_size(const std::vector<int>& values, int expected) {
  return static_cast<int>(values.size()) == expected;
}

bool has_valid_size(const std::vector<double>& values, int expected) {
  return static_cast<int>(values.size()) == expected;
}

bool validate_root_cuts(const engine::BCRootCuts& cuts,
                        int n_cols,
                        std::string& reason) {
  if (cuts.empty()) {
    reason = "root cut bundle is empty";
    return false;
  }
  const int n_cuts = cuts.numCuts();
  if (static_cast<int>(cuts.start.size()) != n_cuts + 1) {
    reason = "root cut start size does not match cut count";
    return false;
  }
  if (static_cast<int>(cuts.upper.size()) != n_cuts) {
    reason = "root cut upper size does not match cut count";
    return false;
  }
  if (cuts.value.size() != cuts.index.size()) {
    reason = "root cut value/index sizes differ";
    return false;
  }
  if (cuts.start.empty() || cuts.start.front() != 0 ||
      cuts.start.back() != static_cast<int>(cuts.index.size())) {
    reason = "root cut start array is not CSR-compatible";
    return false;
  }
  for (int i = 1; i < static_cast<int>(cuts.start.size()); ++i) {
    if (cuts.start[static_cast<size_t>(i)] < cuts.start[static_cast<size_t>(i - 1)]) {
      reason = "root cut start array is not monotone";
      return false;
    }
  }
  for (int col : cuts.index) {
    if (col < 0 || col >= n_cols) {
      reason = "root cut contains an out-of-range column index";
      return false;
    }
  }
  for (double value : cuts.value) {
    if (!std::isfinite(value)) {
      reason = "root cut contains a non-finite coefficient";
      return false;
    }
  }
  return true;
}

bool validate_root_basis(const engine::BCRootBasis& basis,
                         int n_cols,
                         int expected_rows,
                         std::string& reason) {
  if (basis.empty()) {
    reason = "root basis bundle is empty";
    return false;
  }
  if (!has_valid_size(basis.col_status, n_cols)) {
    reason = "root basis column status count does not match model columns";
    return false;
  }
  if (!has_valid_size(basis.row_status, expected_rows)) {
    reason = "root basis row status count does not match model rows plus root cuts";
    return false;
  }
  return true;
}

bool validate_pseudocost(const engine::BCPseudocostInit& pc,
                         int n_cols,
                         std::string& reason) {
  if (pc.empty()) {
    reason = "pseudocost bundle is empty";
    return false;
  }
  if (pc.n_orig_cols != n_cols) {
    reason = "pseudocost original column count does not match model";
    return false;
  }
  if (!has_valid_size(pc.pseudocostup, n_cols) ||
      !has_valid_size(pc.pseudocostdown, n_cols)) {
    reason = "pseudocost value arrays do not match model columns";
    return false;
  }
  const bool optional_arrays_ok =
      (pc.nsamplesup.empty() || has_valid_size(pc.nsamplesup, n_cols)) &&
      (pc.nsamplesdown.empty() || has_valid_size(pc.nsamplesdown, n_cols)) &&
      (pc.inferencesup.empty() || has_valid_size(pc.inferencesup, n_cols)) &&
      (pc.inferencesdown.empty() || has_valid_size(pc.inferencesdown, n_cols)) &&
      (pc.ninferencesup.empty() || has_valid_size(pc.ninferencesup, n_cols)) &&
      (pc.ninferencesdown.empty() || has_valid_size(pc.ninferencesdown, n_cols)) &&
      (pc.conflictscoreup.empty() || has_valid_size(pc.conflictscoreup, n_cols)) &&
      (pc.conflictscoredown.empty() || has_valid_size(pc.conflictscoredown, n_cols));
  if (!optional_arrays_ok) {
    reason = "pseudocost metadata arrays do not match model columns";
    return false;
  }
  return true;
}

nlohmann::json options_to_json(const engine::BCOptions& options) {
  return nlohmann::json{
      {"max_nodes", options.max_nodes},
      {"time_limit_sec", options.time_limit_sec},
      {"gap_tol", options.gap_tol},
      {"root_cut_rounds", options.root_cut_rounds},
      {"cuts_per_round", options.cuts_per_round},
      {"num_threads", options.num_threads},
      {"deterministic_parallel", options.deterministic_parallel},
      {"accept_verified_warm_start_incumbent", options.accept_verified_warm_start_incumbent},
      {"highs_mip_detect_symmetry", options.highs_mip_detect_symmetry},
      {"highs_mip_heuristic_effort", options.highs_mip_heuristic_effort},
      {"highs_mip_pscost_minreliable", options.highs_mip_pscost_minreliable},
      {"highs_mip_max_stall_nodes", options.highs_mip_max_stall_nodes},
      {"highs_mip_run_zi_round", options.highs_mip_run_zi_round},
      {"highs_mip_run_shifting", options.highs_mip_run_shifting},
      {"highs_force_presolve_on", options.highs_force_presolve_on},
      {"highs_presolve_substitution_maxfillin", options.highs_presolve_substitution_maxfillin},
      {"highs_mip_lp_age_limit", options.highs_mip_lp_age_limit},
      {"highs_mip_lp_solver", options.highs_mip_lp_solver},
      {"highs_mip_root_crossover", options.highs_mip_root_crossover},
      {"highs_max_root_sepa_rounds", options.highs_max_root_sepa_rounds},
      {"has_root_cut_warm_start", static_cast<bool>(options.highs_root_cut_warm_start)},
      {"has_root_basis_warm_start", static_cast<bool>(options.highs_root_basis_warm_start)},
      {"has_pseudocost_warm_start", static_cast<bool>(options.highs_pseudocost_warm_start)}};
}

nlohmann::json stats_to_json(const engine::BCStats& stats) {
  return nlohmann::json{
      {"status", stats.status},
      {"collection_scope", stats.collection_scope},
      {"lp_solve_count_available", stats.lp_solve_count_available},
      {"incumbent_timeline_available", stats.incumbent_timeline_available},
      {"cut_diagnostics_available", stats.cut_diagnostics_available},
      {"nodes_explored", stats.nodes_explored},
      {"lp_solves", stats.lp_solve_count_available
                        ? nlohmann::json(stats.lp_solves) : nlohmann::json(nullptr)},
      {"cuts_added", stats.cut_diagnostics_available
                        ? nlohmann::json(stats.cuts_added) : nlohmann::json(nullptr)},
      {"root_cuts_added", stats.cut_diagnostics_available
                             ? nlohmann::json(stats.root_cuts_added) : nlohmann::json(nullptr)},
      {"best_bound", stats.best_bound},
      {"best_obj", stats.best_obj},
      {"gap", stats.gap},
      {"runtime_sec", stats.runtime_sec},
      {"incumbent_updates", stats.incumbent_timeline_available
                                ? nlohmann::json(stats.incumbent_updates) : nlohmann::json(nullptr)},
      {"parallel_effective_threads", stats.parallel_effective_threads}};
}

bool is_scuc_like(const SolverConfigFeatures& features,
                  const SolverConfigPolicyOptions& policy_options) {
  return features.has_uc_hint ||
         (features.num_binary_vars >= policy_options.scuc_min_binary_vars &&
          features.num_rows >= policy_options.scuc_min_rows);
}

bool has_artifact_warm_start(const SolverConfigFeatures& features) {
  return features.has_root_cut_warm_start ||
         features.has_root_basis_warm_start ||
         features.has_pseudocost_warm_start;
}

void apply_scuc_safe_defaults(engine::BCOptions& options,
                              const SolverConfigPolicyOptions& policy_options) {
  options.accept_verified_warm_start_incumbent = true;
  options.highs_mip_run_zi_round = true;
  options.highs_mip_run_shifting = true;
  options.highs_force_presolve_on = true;
  options.highs_presolve_substitution_maxfillin = std::max(
      options.highs_presolve_substitution_maxfillin,
      policy_options.presolve_substitution_maxfillin);
  options.highs_mip_lp_age_limit = std::max(options.highs_mip_lp_age_limit,
                                            policy_options.mip_lp_age_limit);
  options.highs_mip_detect_symmetry = true;
}

}  // namespace

bool SolverArtifactBundle::empty() const {
  return (!root_cuts || root_cuts->empty()) &&
         (!root_basis || root_basis->empty()) &&
         (!pseudocost_init || pseudocost_init->empty());
}

bool SolverArtifactCache::empty() const { return bundles_.empty(); }

std::size_t SolverArtifactCache::size() const { return bundles_.size(); }

void SolverArtifactCache::clear() { bundles_.clear(); }

void SolverArtifactCache::store(const engine::MIPModel& mip,
                                const engine::BCResult& result,
                                const FingerprintOptions& fingerprint_options) {
  SolverArtifactBundle bundle;
  bundle.fingerprint = fingerprint_mip(mip, fingerprint_options);
  if (result.highs_root_cuts && !result.highs_root_cuts->empty()) {
    bundle.root_cuts = std::make_shared<engine::BCRootCuts>(*result.highs_root_cuts);
  }
  if (result.highs_root_basis && !result.highs_root_basis->empty()) {
    bundle.root_basis = std::make_shared<engine::BCRootBasis>(*result.highs_root_basis);
  }
  if (result.highs_pseudocost_init && !result.highs_pseudocost_init->empty()) {
    bundle.pseudocost_init = std::make_shared<engine::BCPseudocostInit>(*result.highs_pseudocost_init);
  }
  if (bundle.empty()) return;

  for (auto& existing : bundles_) {
    if (compatible_fingerprint(existing.fingerprint, bundle.fingerprint)) {
      existing = std::move(bundle);
      return;
    }
  }
  bundles_.push_back(std::move(bundle));
}

SolverArtifactReuseReport SolverArtifactCache::apply(
    const engine::MIPModel& mip,
    engine::BCOptions& options,
    const FingerprintOptions& fingerprint_options) const {
  SolverArtifactReuseReport report;
  report.attempted = true;
  if (bundles_.empty()) {
    report.message = "artifact cache is empty";
    return report;
  }

  const auto current = fingerprint_mip(mip, fingerprint_options);
  const SolverArtifactBundle* matched = nullptr;
  for (const auto& bundle : bundles_) {
    if (compatible_fingerprint(current, bundle.fingerprint)) {
      matched = &bundle;
      break;
    }
  }
  if (!matched) {
    report.message = "no compatible artifact fingerprint";
    return report;
  }

  report.compatible = true;
  const int n_cols = model_num_cols(mip);
  const int original_rows = model_num_rows(mip);
  std::vector<std::string> rejected;

  if (matched->root_cuts) {
    std::string reason;
    if (validate_root_cuts(*matched->root_cuts, n_cols, reason)) {
      options.highs_root_cut_warm_start = matched->root_cuts;
      report.root_cuts_used = true;
      report.root_cut_count = matched->root_cuts->numCuts();
      report.root_cut_nonzeros = static_cast<int>(matched->root_cuts->index.size());
    } else {
      rejected.push_back(reason);
    }
  }

  if (matched->root_basis) {
    std::string reason;
    const int expected_rows = original_rows + report.root_cut_count;
    if (report.root_cuts_used &&
        validate_root_basis(*matched->root_basis, n_cols, expected_rows, reason)) {
      options.highs_root_basis_warm_start = matched->root_basis;
      report.root_basis_used = true;
      report.root_basis_cols = static_cast<int>(matched->root_basis->col_status.size());
      report.root_basis_rows = static_cast<int>(matched->root_basis->row_status.size());
    } else if (!report.root_cuts_used) {
      rejected.push_back("root basis requires accepted root cuts");
    } else {
      rejected.push_back(reason);
    }
  }

  if (matched->pseudocost_init) {
    std::string reason;
    if (validate_pseudocost(*matched->pseudocost_init, n_cols, reason)) {
      options.highs_pseudocost_warm_start = matched->pseudocost_init;
      report.pseudocost_used = true;
      report.pseudocost_cols = matched->pseudocost_init->n_orig_cols;
    } else {
      rejected.push_back(reason);
    }
  }

  if (report.root_cuts_used || report.root_basis_used || report.pseudocost_used) {
    report.message = "ok";
  } else if (!rejected.empty()) {
    std::ostringstream os;
    for (size_t i = 0; i < rejected.size(); ++i) {
      if (i != 0) os << "; ";
      os << rejected[i];
    }
    report.message = os.str();
  } else {
    report.message = "compatible artifact bundle had no reusable payload";
  }
  return report;
}

nlohmann::json SolverArtifactCache::summary() const {
  nlohmann::json bundles = nlohmann::json::array();
  for (const auto& bundle : bundles_) bundles.push_back(bundle);
  return nlohmann::json{{"size", bundles_.size()}, {"bundles", bundles}};
}

void to_json(nlohmann::json& j, const SolverConfigPolicyOptions& options) {
  j = nlohmann::json{
      {"enable_config_tuning", options.enable_config_tuning},
      {"enable_artifact_reuse", options.enable_artifact_reuse},
      {"deterministic_evaluation", options.deterministic_evaluation},
      {"short_time_limit_sec", options.short_time_limit_sec},
      {"short_budget_heuristic_effort", options.short_budget_heuristic_effort},
      {"short_budget_root_sepa_rounds", options.short_budget_root_sepa_rounds},
      {"artifact_root_sepa_rounds", options.artifact_root_sepa_rounds},
      {"scuc_min_binary_vars", options.scuc_min_binary_vars},
      {"scuc_min_rows", options.scuc_min_rows},
      {"presolve_substitution_maxfillin", options.presolve_substitution_maxfillin},
      {"mip_lp_age_limit", options.mip_lp_age_limit},
      {"short_budget_max_stall_nodes", options.short_budget_max_stall_nodes}};
}

void to_json(nlohmann::json& j, const SolverConfigFeatures& features) {
  j = nlohmann::json{
      {"fingerprint", features.fingerprint},
      {"num_variables", features.num_variables},
      {"num_rows", features.num_rows},
      {"num_eq_rows", features.num_eq_rows},
      {"num_nonzeros", features.num_nonzeros},
      {"num_integer_vars", features.num_integer_vars},
      {"num_binary_vars", features.num_binary_vars},
      {"time_limit_sec", features.time_limit_sec},
      {"has_uc_hint", features.has_uc_hint},
      {"uc_generators", features.uc_generators},
      {"uc_periods", features.uc_periods},
      {"uc_segments", features.uc_segments},
      {"uc_storage_units", features.uc_storage_units},
      {"uc_network_lines", features.uc_network_lines},
      {"has_root_cut_warm_start", features.has_root_cut_warm_start},
      {"has_root_basis_warm_start", features.has_root_basis_warm_start},
      {"has_pseudocost_warm_start", features.has_pseudocost_warm_start},
      {"root_cut_count", features.root_cut_count},
      {"root_cut_nonzeros", features.root_cut_nonzeros},
      {"root_basis_rows", features.root_basis_rows},
      {"root_basis_cols", features.root_basis_cols},
      {"pseudocost_cols", features.pseudocost_cols},
      {"previous_stats_available", features.previous_stats_available},
      {"previous_gap", features.previous_gap},
      {"previous_runtime_sec", features.previous_runtime_sec},
      {"previous_best_obj", features.previous_best_obj},
      {"previous_best_bound", features.previous_best_bound},
      {"previous_nodes_explored", features.previous_nodes_explored},
      {"previous_lp_solves", features.previous_lp_solves}};
}

void to_json(nlohmann::json& j, const SolverConfigDecision& decision) {
  j = nlohmann::json{
      {"profile", decision.profile},
      {"changed_options", decision.changed_options},
      {"features", decision.features},
      {"options", options_to_json(decision.options)}};
}

void to_json(nlohmann::json& j, const SolverArtifactReuseReport& report) {
  j = nlohmann::json{
      {"attempted", report.attempted},
      {"compatible", report.compatible},
      {"root_cuts_used", report.root_cuts_used},
      {"root_basis_used", report.root_basis_used},
      {"pseudocost_used", report.pseudocost_used},
      {"message", report.message},
      {"root_cut_count", report.root_cut_count},
      {"root_cut_nonzeros", report.root_cut_nonzeros},
      {"root_basis_rows", report.root_basis_rows},
      {"root_basis_cols", report.root_basis_cols},
      {"pseudocost_cols", report.pseudocost_cols}};
}

void to_json(nlohmann::json& j, const SolverArtifactBundle& bundle) {
  j = nlohmann::json{
      {"fingerprint", bundle.fingerprint},
      {"empty", bundle.empty()},
      {"has_root_cuts", static_cast<bool>(bundle.root_cuts) && !bundle.root_cuts->empty()},
      {"has_root_basis", static_cast<bool>(bundle.root_basis) && !bundle.root_basis->empty()},
      {"has_pseudocost_init", static_cast<bool>(bundle.pseudocost_init) && !bundle.pseudocost_init->empty()},
      {"root_cut_count", bundle.root_cuts ? bundle.root_cuts->numCuts() : 0},
      {"root_cut_nonzeros", bundle.root_cuts ? static_cast<int>(bundle.root_cuts->index.size()) : 0},
      {"root_basis_rows", bundle.root_basis ? static_cast<int>(bundle.root_basis->row_status.size()) : 0},
      {"root_basis_cols", bundle.root_basis ? static_cast<int>(bundle.root_basis->col_status.size()) : 0},
      {"pseudocost_cols", bundle.pseudocost_init ? bundle.pseudocost_init->n_orig_cols : 0}};
}

void to_json(nlohmann::json& j, const SolverConfigRunRecord& record) {
  j = nlohmann::json{
      {"policy_tag", record.policy_tag},
      {"features", record.features},
      {"options", options_to_json(record.options)},
      {"stats", stats_to_json(record.stats)}};
}

SolverConfigFeatures extract_solver_config_features(
    const engine::MIPModel& mip,
    const engine::BCOptions& options,
    const engine::BCStats* previous_stats,
    const FingerprintOptions& fingerprint_options) {
  SolverConfigFeatures features;
  features.fingerprint = fingerprint_mip(mip, fingerprint_options);
  features.num_variables = model_num_cols(mip);
  features.num_rows = model_num_rows(mip);
  features.num_eq_rows = model_num_eq_rows(mip);
  features.num_nonzeros = model_num_nonzeros(mip);
  features.num_integer_vars = static_cast<int>(mip.integer_idx.size());
  features.num_binary_vars = static_cast<int>(mip.binary_idx.size());
  features.time_limit_sec = options.time_limit_sec;
  features.has_uc_hint = mip.uc_hint.has_value();
  if (mip.uc_hint) {
    const auto& hint = *mip.uc_hint;
    features.uc_generators = hint.ng;
    features.uc_periods = hint.T;
    features.uc_segments = hint.n_segments;
    features.uc_storage_units = hint.n_storage;
    features.uc_network_lines = hint.network_line_count;
  }
  features.has_root_cut_warm_start = options.highs_root_cut_warm_start &&
                                      !options.highs_root_cut_warm_start->empty();
  features.has_root_basis_warm_start = options.highs_root_basis_warm_start &&
                                        !options.highs_root_basis_warm_start->empty();
  features.has_pseudocost_warm_start = options.highs_pseudocost_warm_start &&
                                        !options.highs_pseudocost_warm_start->empty();
  if (options.highs_root_cut_warm_start) {
    features.root_cut_count = options.highs_root_cut_warm_start->numCuts();
    features.root_cut_nonzeros = static_cast<int>(options.highs_root_cut_warm_start->index.size());
  }
  if (options.highs_root_basis_warm_start) {
    features.root_basis_rows = static_cast<int>(options.highs_root_basis_warm_start->row_status.size());
    features.root_basis_cols = static_cast<int>(options.highs_root_basis_warm_start->col_status.size());
  }
  if (options.highs_pseudocost_warm_start) {
    features.pseudocost_cols = options.highs_pseudocost_warm_start->n_orig_cols;
  }
  if (previous_stats) {
    features.previous_stats_available = true;
    features.previous_gap = previous_stats->gap;
    features.previous_runtime_sec = previous_stats->runtime_sec;
    features.previous_best_obj = previous_stats->best_obj;
    features.previous_best_bound = previous_stats->best_bound;
    features.previous_nodes_explored = previous_stats->nodes_explored;
    features.previous_lp_solves = previous_stats->lp_solves;
  }
  return features;
}

SolverConfigDecision choose_solver_config(
    const SolverConfigFeatures& features,
    const engine::BCOptions& base_options,
    const SolverConfigPolicyOptions& policy_options) {
  SolverConfigDecision decision;
  decision.options = base_options;
  decision.features = features;

  if (!policy_options.enable_config_tuning) {
    decision.profile = "disabled";
    return decision;
  }

  const bool scuc_like = is_scuc_like(features, policy_options);
  const bool artifacts = has_artifact_warm_start(features);
  const bool short_budget = features.time_limit_sec > 0.0 &&
                            features.time_limit_sec <= policy_options.short_time_limit_sec;

  if (policy_options.deterministic_evaluation) {
    decision.options.num_threads = 1;
    decision.options.deterministic_parallel = true;
  }

  if (scuc_like) {
    apply_scuc_safe_defaults(decision.options, policy_options);
    decision.profile = short_budget ? "scuc_short_budget" : "scuc_balanced";
    if (short_budget) {
      decision.options.highs_mip_heuristic_effort = std::max(
          decision.options.highs_mip_heuristic_effort,
          policy_options.short_budget_heuristic_effort);
      decision.options.highs_mip_lp_solver = "choose";
      decision.options.highs_strict_auto_ipm_root_for_large_models = false;
      decision.options.highs_mip_max_stall_nodes = std::max(
          decision.options.highs_mip_max_stall_nodes,
          policy_options.short_budget_max_stall_nodes);
      if (policy_options.short_budget_root_sepa_rounds > 0) {
        decision.options.highs_max_root_sepa_rounds = policy_options.short_budget_root_sepa_rounds;
      }
    }
  }

  if (artifacts && policy_options.enable_artifact_reuse) {
    decision.profile = scuc_like ? decision.profile + "+artifact_reuse" : "artifact_reuse";
    if (policy_options.artifact_root_sepa_rounds > 0 && features.has_root_cut_warm_start) {
      decision.options.highs_max_root_sepa_rounds = policy_options.artifact_root_sepa_rounds;
    }
    decision.options.highs_mip_pscost_minreliable = std::min(
        decision.options.highs_mip_pscost_minreliable, 3);
  }

  decision.changed_options = options_to_json(decision.options) != options_to_json(base_options);
  return decision;
}

engine::BCOptions tune_bc_options_from_features(
    const SolverConfigFeatures& features,
    const engine::BCOptions& base_options,
    const SolverConfigPolicyOptions& policy_options) {
  return choose_solver_config(features, base_options, policy_options).options;
}

SolverConfigFeatures solver_config_features_from_bc_instance(
    const engine::BCInstanceFeatures& instance_features,
    const engine::BCOptions& options,
    const engine::BCStats* previous_stats) {
  SolverConfigFeatures features;
  features.num_variables = instance_features.n_vars;
  features.num_rows = instance_features.n_rows;
  features.num_eq_rows = instance_features.n_eq;
  features.num_nonzeros = instance_features.n_nnz;
  features.num_binary_vars = instance_features.n_bin;
  features.num_integer_vars = instance_features.n_int;
  features.time_limit_sec = options.time_limit_sec;
  if (!instance_features.extra.empty()) {
    features.has_uc_hint = instance_features.extra[0] > 0.5;
    if (instance_features.extra.size() > 1) features.uc_generators = static_cast<int>(instance_features.extra[1]);
    if (instance_features.extra.size() > 2) features.uc_periods = static_cast<int>(instance_features.extra[2]);
    if (instance_features.extra.size() > 3) features.uc_segments = static_cast<int>(instance_features.extra[3]);
    if (instance_features.extra.size() > 4) features.uc_storage_units = static_cast<int>(instance_features.extra[4]);
    if (instance_features.extra.size() > 5) features.uc_network_lines = static_cast<int>(instance_features.extra[5]);
  }
  features.has_root_cut_warm_start = options.highs_root_cut_warm_start &&
                                      !options.highs_root_cut_warm_start->empty();
  features.has_root_basis_warm_start = options.highs_root_basis_warm_start &&
                                        !options.highs_root_basis_warm_start->empty();
  features.has_pseudocost_warm_start = options.highs_pseudocost_warm_start &&
                                        !options.highs_pseudocost_warm_start->empty();
  if (options.highs_root_cut_warm_start) {
    features.root_cut_count = options.highs_root_cut_warm_start->numCuts();
    features.root_cut_nonzeros = static_cast<int>(options.highs_root_cut_warm_start->index.size());
  }
  if (options.highs_root_basis_warm_start) {
    features.root_basis_rows = static_cast<int>(options.highs_root_basis_warm_start->row_status.size());
    features.root_basis_cols = static_cast<int>(options.highs_root_basis_warm_start->col_status.size());
  }
  if (options.highs_pseudocost_warm_start) {
    features.pseudocost_cols = options.highs_pseudocost_warm_start->n_orig_cols;
  }
  if (previous_stats) {
    features.previous_stats_available = true;
    features.previous_gap = previous_stats->gap;
    features.previous_runtime_sec = previous_stats->runtime_sec;
    features.previous_best_obj = previous_stats->best_obj;
    features.previous_best_bound = previous_stats->best_bound;
    features.previous_nodes_explored = previous_stats->nodes_explored;
    features.previous_lp_solves = previous_stats->lp_solves;
  }
  return features;
}

engine::BCCallbacks make_solver_config_callbacks(
    const SolverConfigPolicyOptions& policy_options,
    std::vector<SolverConfigRunRecord>* records,
    std::string policy_tag) {
  engine::BCCallbacks callbacks;
  callbacks.policy_tag = policy_tag;
  callbacks.hyperparam_tuner = [policy_options](const engine::BCInstanceFeatures& feat,
                                                const engine::BCOptions& base,
                                                const engine::BCStats* previous_stats) {
    const auto features = solver_config_features_from_bc_instance(feat, base, previous_stats);
    return tune_bc_options_from_features(features, base, policy_options);
  };
  callbacks.post_solve = [policy_options, records, policy_tag = std::move(policy_tag)](
                              const engine::BCInstanceFeatures& feat,
                              const engine::BCOptions& options_used,
                              const engine::BCStats& stats) {
    if (!records) return;
    SolverConfigRunRecord record;
    record.features = solver_config_features_from_bc_instance(feat, options_used, nullptr);
    record.options = options_used;
    record.stats = stats;
    record.policy_tag = policy_tag;
    records->push_back(std::move(record));
    (void)policy_options;
  };
  return callbacks;
}

}  // namespace mipsolvers::l2o
