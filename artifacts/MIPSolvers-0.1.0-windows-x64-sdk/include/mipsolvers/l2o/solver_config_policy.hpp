#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "mipsolvers/engine/bc/ml_hooks.hpp"
#include "mipsolvers/engine/bc/options.hpp"
#include "mipsolvers/engine/bc/stats.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/l2o/model_fingerprint.hpp"

namespace mipsolvers::l2o {

struct SolverConfigPolicyOptions {
  bool enable_config_tuning{true};
  bool enable_artifact_reuse{true};
  bool deterministic_evaluation{true};
  double short_time_limit_sec{5.0};
  double short_budget_heuristic_effort{0.30};
  int short_budget_root_sepa_rounds{12};
  int artifact_root_sepa_rounds{8};
  int scuc_min_binary_vars{500};
  int scuc_min_rows{2000};
  int presolve_substitution_maxfillin{30};
  int mip_lp_age_limit{30};
  int short_budget_max_stall_nodes{2000};
};

struct SolverConfigFeatures {
  ModelFingerprint fingerprint;
  int num_variables{0};
  int num_rows{0};
  int num_eq_rows{0};
  int num_nonzeros{0};
  int num_integer_vars{0};
  int num_binary_vars{0};
  double time_limit_sec{0.0};
  bool has_uc_hint{false};
  int uc_generators{0};
  int uc_periods{0};
  int uc_segments{0};
  int uc_storage_units{0};
  int uc_network_lines{0};
  bool has_root_cut_warm_start{false};
  bool has_root_basis_warm_start{false};
  bool has_pseudocost_warm_start{false};
  int root_cut_count{0};
  int root_cut_nonzeros{0};
  int root_basis_rows{0};
  int root_basis_cols{0};
  int pseudocost_cols{0};
  bool previous_stats_available{false};
  double previous_gap{0.0};
  double previous_runtime_sec{0.0};
  double previous_best_obj{0.0};
  double previous_best_bound{0.0};
  int previous_nodes_explored{0};
  int previous_lp_solves{0};
};

struct SolverConfigDecision {
  engine::BCOptions options;
  SolverConfigFeatures features;
  std::string profile{"generic"};
  bool changed_options{false};
};

struct SolverArtifactReuseReport {
  bool attempted{false};
  bool compatible{false};
  bool root_cuts_used{false};
  bool root_basis_used{false};
  bool pseudocost_used{false};
  std::string message;
  int root_cut_count{0};
  int root_cut_nonzeros{0};
  int root_basis_rows{0};
  int root_basis_cols{0};
  int pseudocost_cols{0};
};

struct SolverArtifactBundle {
  ModelFingerprint fingerprint;
  std::shared_ptr<engine::BCRootCuts> root_cuts;
  std::shared_ptr<engine::BCRootBasis> root_basis;
  std::shared_ptr<engine::BCPseudocostInit> pseudocost_init;

  bool empty() const;
};

class SolverArtifactCache {
 public:
  bool empty() const;
  std::size_t size() const;
  void clear();

  void store(const engine::MIPModel& mip,
             const engine::BCResult& result,
             const FingerprintOptions& fingerprint_options = FingerprintOptions{});

  SolverArtifactReuseReport apply(
      const engine::MIPModel& mip,
      engine::BCOptions& options,
      const FingerprintOptions& fingerprint_options = FingerprintOptions{}) const;

  nlohmann::json summary() const;

 private:
  std::vector<SolverArtifactBundle> bundles_;
};

struct SolverConfigRunRecord {
  SolverConfigFeatures features;
  engine::BCOptions options;
  engine::BCStats stats;
  std::string policy_tag;
};

void to_json(nlohmann::json& j, const SolverConfigPolicyOptions& options);
void to_json(nlohmann::json& j, const SolverConfigFeatures& features);
void to_json(nlohmann::json& j, const SolverConfigDecision& decision);
void to_json(nlohmann::json& j, const SolverArtifactReuseReport& report);
void to_json(nlohmann::json& j, const SolverArtifactBundle& bundle);
void to_json(nlohmann::json& j, const SolverConfigRunRecord& record);

SolverConfigFeatures extract_solver_config_features(
    const engine::MIPModel& mip,
    const engine::BCOptions& options,
    const engine::BCStats* previous_stats = nullptr,
    const FingerprintOptions& fingerprint_options = FingerprintOptions{});

SolverConfigDecision choose_solver_config(
    const SolverConfigFeatures& features,
    const engine::BCOptions& base_options,
    const SolverConfigPolicyOptions& policy_options = SolverConfigPolicyOptions{});

engine::BCOptions tune_bc_options_from_features(
    const SolverConfigFeatures& features,
    const engine::BCOptions& base_options,
    const SolverConfigPolicyOptions& policy_options = SolverConfigPolicyOptions{});

SolverConfigFeatures solver_config_features_from_bc_instance(
    const engine::BCInstanceFeatures& features,
    const engine::BCOptions& options,
    const engine::BCStats* previous_stats = nullptr);

engine::BCCallbacks make_solver_config_callbacks(
    const SolverConfigPolicyOptions& policy_options = SolverConfigPolicyOptions{},
    std::vector<SolverConfigRunRecord>* records = nullptr,
    std::string policy_tag = "l2o-phase4-config");

}  // namespace mipsolvers::l2o