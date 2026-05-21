#pragma once

#include <string>

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"
#include "mipsolvers/scuc/scuc.hpp"

namespace mipsolvers::l2o {

struct SCUCWarmStartOptions {
  double binary_threshold{0.5};
  bool infer_missing_transitions{true};
  bool initialize_from_existing_seed{true};
};

struct SCUCWarmStartReport {
  bool success{false};
  std::string message;
  int ng{0};
  int T{0};
  int num_ig_set{0};
  int num_su_set{0};
  int num_sd_set{0};
  int num_missing_cols{0};
  int num_out_of_range_predictions{0};
  bool used_existing_seed{false};
  bool inferred_startup{false};
  bool inferred_shutdown{false};
};

struct SCUCWarmStartResult {
  Eigen::VectorXd x;
  SCUCWarmStartReport report;
};

SCUCWarmStartResult make_scuc_commitment_warm_start(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& commitment,
    const scuc::Matrix2D& startup = {},
    const scuc::Matrix2D& shutdown = {},
    const SCUCWarmStartOptions& options = {});

void apply_scuc_commitment_warm_start(
    engine::MIPModel& mip,
    const scuc::Matrix2D& commitment,
    const scuc::Matrix2D& startup = {},
    const scuc::Matrix2D& shutdown = {},
    const SCUCWarmStartOptions& options = {});

}  // namespace mipsolvers::l2o