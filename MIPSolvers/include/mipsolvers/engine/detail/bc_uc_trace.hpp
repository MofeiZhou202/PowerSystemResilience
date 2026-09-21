/// @file bc_uc_trace.hpp
/// @brief UC-specific progressive-rounding trace and rounding helpers.

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/problem_types.hpp"

namespace mipsolvers::engine::detail {

struct ProgRoundTraceFocusVar {
  int g{0};
  int t{0};
  int col{-1};
  double root_val{0.0};
  int frac_rank{-1};
  bool available{true};
  bool last_fixed{false};
  double last_value{0.0};
};

std::vector<ProgRoundTraceFocusVar> make_prog_round_focus_ig_trace(
    const std::optional<MIPModel::UCGenHint>& uc_hint,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub);

void annotate_prog_round_focus_frac_ranks(
    std::vector<ProgRoundTraceFocusVar>& focus,
    const std::vector<int>& frac_cols);

void print_prog_round_focus_initial(
    const char* phase,
    const std::vector<ProgRoundTraceFocusVar>& focus);

void log_prog_round_focus_changes(const char* phase,
                                  const std::string& source,
                                  std::vector<ProgRoundTraceFocusVar>& focus,
                                  const Eigen::VectorXd& lb,
                                  const Eigen::VectorXd& ub);

void uc_round_commitment(Eigen::VectorXd& x,
                         const MIPModel::UCGenHint& uc,
                         int ng_stride);

}  // namespace mipsolvers::engine::detail
