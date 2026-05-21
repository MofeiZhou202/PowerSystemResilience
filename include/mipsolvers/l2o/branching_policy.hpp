#pragma once

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "mipsolvers/engine/bc/ml_hooks.hpp"
#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/scuc/scuc.hpp"

namespace mipsolvers::l2o {

struct SCUCBranchingPolicyOptions {
  bool enable_static_priorities{true};
  bool enable_dynamic_priors{true};
  bool clear_existing_priorities{true};
  bool include_startup_shutdown{true};
  bool include_dispatch_priorities{false};
  bool prefer_earlier_periods{true};
  bool dynamic_scale_by_fractionality{true};
  int commitment_base_priority{3000};
  int transition_base_priority{2000};
  int dispatch_base_priority{500};
  int time_priority_scale{10};
  double learned_priority_scale{1000.0};
  double transition_score_scale{0.50};
  double dispatch_score_scale{0.25};
  double dynamic_prior_weight{0.75};
  double score_epsilon{1e-9};
};

struct SCUCBranchingPriorityReport {
  bool success{false};
  bool used_uc_hint{false};
  bool used_learned_scores{false};
  bool installed_static_priorities{false};
  bool installed_dynamic_prior{false};
  std::string message;
  int ng{0};
  int T{0};
  int num_priority_entries{0};
  int num_commitment_priorities{0};
  int num_transition_priorities{0};
  int num_dispatch_priorities{0};
  int num_missing_cols{0};
  double min_score{0.0};
  double max_score{0.0};
};

void to_json(nlohmann::json& j, const SCUCBranchingPolicyOptions& options);
void to_json(nlohmann::json& j, const SCUCBranchingPriorityReport& report);

std::vector<double> normalize_scuc_score_matrix(
    const engine::MIPModel::UCGenHint& hint,
    const scuc::Matrix2D& generator_time_scores,
    const SCUCBranchingPolicyOptions& options,
    SCUCBranchingPriorityReport* report = nullptr);

SCUCBranchingPriorityReport apply_scuc_branching_priorities(
    engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores = {},
    const SCUCBranchingPolicyOptions& options = SCUCBranchingPolicyOptions{});

std::vector<int> make_scuc_branching_priorities(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores = {},
    const SCUCBranchingPolicyOptions& options = SCUCBranchingPolicyOptions{},
    SCUCBranchingPriorityReport* report = nullptr);

engine::BCCallbacks make_scuc_branching_callbacks(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores = {},
    const SCUCBranchingPolicyOptions& options = SCUCBranchingPolicyOptions{},
    std::string policy_tag = "l2o-phase5-branching");

}  // namespace mipsolvers::l2o