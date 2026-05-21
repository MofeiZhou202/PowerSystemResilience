#include "mipsolvers/l2o/branching_policy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace mipsolvers::l2o {
namespace {

int model_num_cols(const engine::MIPModel& mip) {
  return static_cast<int>(mip.linear_part.c.size());
}

int uc_pos(const engine::MIPModel::UCGenHint& hint, int g, int t) {
  return t * hint.ng + g;
}

int column_at(const std::vector<int>& cols,
              const engine::MIPModel::UCGenHint& hint,
              int g,
              int t) {
  if (g < 0 || t < 0 || g >= hint.ng || t >= hint.T) return -1;
  const auto pos = static_cast<std::size_t>(uc_pos(hint, g, t));
  return pos < cols.size() ? cols[pos] : -1;
}

double score_at(const scuc::Matrix2D& scores, int g, int t) {
  if (g < 0 || g >= static_cast<int>(scores.size())) return 0.0;
  const auto& row = scores[static_cast<std::size_t>(g)];
  if (t < 0 || t >= static_cast<int>(row.size())) return 0.0;
  const double value = row[static_cast<std::size_t>(t)];
  return std::isfinite(value) ? value : 0.0;
}

bool has_score_shape(const scuc::Matrix2D& scores,
                     const engine::MIPModel::UCGenHint& hint) {
  if (scores.empty()) return false;
  if (static_cast<int>(scores.size()) != hint.ng) return false;
  for (const auto& row : scores) {
    if (static_cast<int>(row.size()) != hint.T) return false;
  }
  return true;
}

int scaled_score_priority(double normalized_score, double scale) {
  if (!std::isfinite(normalized_score) || !std::isfinite(scale)) return 0;
  return static_cast<int>(std::llround(std::max(0.0, normalized_score) * scale));
}

void set_priority(std::vector<int>& priorities,
                  int col,
                  int value,
                  bool clear_existing,
                  int& missing_cols,
                  int& installed) {
  if (col < 0 || col >= static_cast<int>(priorities.size())) {
    ++missing_cols;
    return;
  }
  if (clear_existing) {
    priorities[static_cast<std::size_t>(col)] = value;
  } else {
    priorities[static_cast<std::size_t>(col)] =
        std::max(priorities[static_cast<std::size_t>(col)], value);
  }
  ++installed;
}

std::vector<double> build_column_dynamic_scores(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores,
    const SCUCBranchingPolicyOptions& options,
    SCUCBranchingPriorityReport* report) {
  std::vector<double> col_scores(static_cast<std::size_t>(model_num_cols(mip)), 0.0);
  if (!mip.uc_hint) return col_scores;
  const auto& hint = *mip.uc_hint;
  SCUCBranchingPriorityReport local_report;
  auto normalized = normalize_scuc_score_matrix(
      hint, generator_time_scores, options, report ? report : &local_report);
  if (static_cast<int>(normalized.size()) != hint.ng * hint.T) return col_scores;

  auto assign_col = [&](int col, double value) {
    if (col >= 0 && col < static_cast<int>(col_scores.size())) {
      col_scores[static_cast<std::size_t>(col)] =
          std::max(col_scores[static_cast<std::size_t>(col)], value);
    }
  };

  for (int t = 0; t < hint.T; ++t) {
    for (int g = 0; g < hint.ng; ++g) {
      const auto pos = static_cast<std::size_t>(uc_pos(hint, g, t));
      const double score = normalized[pos];
      const double commit_score = options.dynamic_prior_weight * score;
      const double transition_score =
          options.dynamic_prior_weight * options.transition_score_scale * score;
      const double dispatch_score =
          options.dynamic_prior_weight * options.dispatch_score_scale * score;
      assign_col(column_at(hint.ig_cols, hint, g, t), commit_score);
      if (options.include_startup_shutdown) {
        assign_col(column_at(hint.su_cols, hint, g, t), transition_score);
        assign_col(column_at(hint.sd_cols, hint, g, t), transition_score);
      }
      if (options.include_dispatch_priorities) {
        assign_col(column_at(hint.pg_cols, hint, g, t), dispatch_score);
      }
    }
  }
  return col_scores;
}

}  // namespace

void to_json(nlohmann::json& j, const SCUCBranchingPolicyOptions& options) {
  j = nlohmann::json{
      {"enable_static_priorities", options.enable_static_priorities},
      {"enable_dynamic_priors", options.enable_dynamic_priors},
      {"clear_existing_priorities", options.clear_existing_priorities},
      {"include_startup_shutdown", options.include_startup_shutdown},
      {"include_dispatch_priorities", options.include_dispatch_priorities},
      {"prefer_earlier_periods", options.prefer_earlier_periods},
      {"dynamic_scale_by_fractionality", options.dynamic_scale_by_fractionality},
      {"commitment_base_priority", options.commitment_base_priority},
      {"transition_base_priority", options.transition_base_priority},
      {"dispatch_base_priority", options.dispatch_base_priority},
      {"time_priority_scale", options.time_priority_scale},
      {"learned_priority_scale", options.learned_priority_scale},
      {"transition_score_scale", options.transition_score_scale},
      {"dispatch_score_scale", options.dispatch_score_scale},
      {"dynamic_prior_weight", options.dynamic_prior_weight},
      {"score_epsilon", options.score_epsilon}};
}

void to_json(nlohmann::json& j, const SCUCBranchingPriorityReport& report) {
  j = nlohmann::json{
      {"success", report.success},
      {"used_uc_hint", report.used_uc_hint},
      {"used_learned_scores", report.used_learned_scores},
      {"installed_static_priorities", report.installed_static_priorities},
      {"installed_dynamic_prior", report.installed_dynamic_prior},
      {"message", report.message},
      {"ng", report.ng},
      {"T", report.T},
      {"num_priority_entries", report.num_priority_entries},
      {"num_commitment_priorities", report.num_commitment_priorities},
      {"num_transition_priorities", report.num_transition_priorities},
      {"num_dispatch_priorities", report.num_dispatch_priorities},
      {"num_missing_cols", report.num_missing_cols},
      {"min_score", report.min_score},
      {"max_score", report.max_score}};
}

std::vector<double> normalize_scuc_score_matrix(
    const engine::MIPModel::UCGenHint& hint,
    const scuc::Matrix2D& generator_time_scores,
    const SCUCBranchingPolicyOptions& options,
    SCUCBranchingPriorityReport* report) {
  const int total = std::max(0, hint.ng * hint.T);
  std::vector<double> normalized(static_cast<std::size_t>(total), 0.0);
  if (report) {
    report->ng = hint.ng;
    report->T = hint.T;
    report->used_uc_hint = true;
  }
  if (total == 0) return normalized;

  const bool use_scores = has_score_shape(generator_time_scores, hint);
  if (report) report->used_learned_scores = use_scores;

  double min_score = std::numeric_limits<double>::infinity();
  double max_score = -std::numeric_limits<double>::infinity();
  for (int t = 0; t < hint.T; ++t) {
    for (int g = 0; g < hint.ng; ++g) {
      const double raw = use_scores ? score_at(generator_time_scores, g, t) : 1.0;
      min_score = std::min(min_score, raw);
      max_score = std::max(max_score, raw);
    }
  }
  if (!std::isfinite(min_score) || !std::isfinite(max_score)) {
    min_score = 0.0;
    max_score = 0.0;
  }

  const double span = max_score - min_score;
  for (int t = 0; t < hint.T; ++t) {
    for (int g = 0; g < hint.ng; ++g) {
      double value = use_scores ? score_at(generator_time_scores, g, t) : 1.0;
      if (span > options.score_epsilon) value = (value - min_score) / span;
      else value = use_scores ? 1.0 : 0.0;
      if (options.prefer_earlier_periods && hint.T > 1) {
        const double time_bonus = static_cast<double>(hint.T - 1 - t) /
                                  static_cast<double>(hint.T - 1);
        value = 0.85 * value + 0.15 * time_bonus;
      }
      normalized[static_cast<std::size_t>(uc_pos(hint, g, t))] =
          std::clamp(value, 0.0, 1.0);
    }
  }
  if (report) {
    report->min_score = min_score;
    report->max_score = max_score;
  }
  return normalized;
}

std::vector<int> make_scuc_branching_priorities(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores,
    const SCUCBranchingPolicyOptions& options,
    SCUCBranchingPriorityReport* report) {
  SCUCBranchingPriorityReport local_report;
  SCUCBranchingPriorityReport& out = report ? *report : local_report;
  out = SCUCBranchingPriorityReport{};

  const int n_cols = model_num_cols(mip);
  std::vector<int> priorities(static_cast<std::size_t>(std::max(0, n_cols)), 0);
  if (!options.clear_existing_priorities &&
      static_cast<int>(mip.branching_priority.size()) == n_cols) {
    priorities = mip.branching_priority;
  }
  if (!options.enable_static_priorities) {
    out.success = true;
    out.message = "static priorities disabled";
    out.num_priority_entries = static_cast<int>(std::count_if(
        priorities.begin(), priorities.end(), [](int value) { return value != 0; }));
    return priorities;
  }
  if (!mip.uc_hint) {
    out.message = "SCUC UC hint is missing";
    return priorities;
  }

  const auto& hint = *mip.uc_hint;
  auto normalized = normalize_scuc_score_matrix(
      hint, generator_time_scores, options, &out);
  if (static_cast<int>(normalized.size()) != hint.ng * hint.T) {
    out.message = "invalid SCUC score matrix";
    return priorities;
  }

  for (int t = 0; t < hint.T; ++t) {
    const int time_bonus = options.prefer_earlier_periods
                               ? options.time_priority_scale * (hint.T - t)
                               : 0;
    for (int g = 0; g < hint.ng; ++g) {
      const double score = normalized[static_cast<std::size_t>(uc_pos(hint, g, t))];
      const int learned = scaled_score_priority(score, options.learned_priority_scale);

      const int commitment_priority =
          options.commitment_base_priority + time_bonus + learned;
      set_priority(priorities, column_at(hint.ig_cols, hint, g, t),
                   commitment_priority, options.clear_existing_priorities,
                   out.num_missing_cols, out.num_commitment_priorities);

      if (options.include_startup_shutdown) {
        const int transition_priority = options.transition_base_priority +
            time_bonus + scaled_score_priority(
                score, options.learned_priority_scale * options.transition_score_scale);
        set_priority(priorities, column_at(hint.su_cols, hint, g, t),
                     transition_priority, options.clear_existing_priorities,
                     out.num_missing_cols, out.num_transition_priorities);
        set_priority(priorities, column_at(hint.sd_cols, hint, g, t),
                     transition_priority, options.clear_existing_priorities,
                     out.num_missing_cols, out.num_transition_priorities);
      }

      if (options.include_dispatch_priorities) {
        const int dispatch_priority = options.dispatch_base_priority +
            time_bonus + scaled_score_priority(
                score, options.learned_priority_scale * options.dispatch_score_scale);
        set_priority(priorities, column_at(hint.pg_cols, hint, g, t),
                     dispatch_priority, options.clear_existing_priorities,
                     out.num_missing_cols, out.num_dispatch_priorities);
      }
    }
  }

  out.success = true;
  out.installed_static_priorities = true;
  out.num_priority_entries = static_cast<int>(std::count_if(
      priorities.begin(), priorities.end(), [](int value) { return value != 0; }));
  out.message = "ok";
  return priorities;
}

SCUCBranchingPriorityReport apply_scuc_branching_priorities(
    engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores,
    const SCUCBranchingPolicyOptions& options) {
  SCUCBranchingPriorityReport report;
  mip.branching_priority = make_scuc_branching_priorities(
      mip, generator_time_scores, options, &report);
  return report;
}

engine::BCCallbacks make_scuc_branching_callbacks(
    const engine::MIPModel& mip,
    const scuc::Matrix2D& generator_time_scores,
    const SCUCBranchingPolicyOptions& options,
    std::string policy_tag) {
  engine::BCCallbacks callbacks;
  callbacks.policy_tag = std::move(policy_tag);
  if (!options.enable_dynamic_priors || !mip.uc_hint) return callbacks;

  SCUCBranchingPriorityReport report;
  auto col_scores = build_column_dynamic_scores(
      mip, generator_time_scores, options, &report);
  const bool any_score = std::any_of(col_scores.begin(), col_scores.end(),
                                     [](double value) { return value > 0.0; });
  if (!any_score) return callbacks;

  callbacks.branching_prior =
      [scores = std::move(col_scores), options](const engine::BCBranchContext& ctx,
                                                std::vector<double>& out_scores) {
        out_scores.clear();
        if (ctx.candidates == nullptr || ctx.candidates->empty()) return;
        out_scores.resize(ctx.candidates->size(), 0.0);
        for (std::size_t k = 0; k < ctx.candidates->size(); ++k) {
          const int col = (*ctx.candidates)[k];
          if (col < 0 || col >= static_cast<int>(scores.size())) continue;
          double value = scores[static_cast<std::size_t>(col)];
          if (options.dynamic_scale_by_fractionality && ctx.lp_x != nullptr &&
              static_cast<std::size_t>(col) < ctx.lp_x_size) {
            const double x = ctx.lp_x[col];
            if (std::isfinite(x)) {
              const double frac = std::abs(x - std::round(x));
              const double centered = std::min(1.0, 2.0 * std::min(frac, 1.0 - frac));
              value *= 0.25 + 0.75 * std::max(0.0, centered);
            }
          }
          out_scores[k] = value;
        }
      };
  return callbacks;
}

}  // namespace mipsolvers::l2o