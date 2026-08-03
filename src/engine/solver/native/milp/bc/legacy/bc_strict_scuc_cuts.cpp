/// @file bc_strict_scuc_cuts.cpp
/// @brief Definitions for SCUC dynamic node-cut separation (strict HiGHS path).

#include "mipsolvers/engine/detail/bc_strict_scuc_cuts.hpp"

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <fmt/format.h>

#include "mipsolvers/engine/detail/bc_env_options.hpp"

namespace mipsolvers::engine::detail {
namespace {

// Mirrors HiGHS' default MIP feasibility tolerance; kept local so this unit is
// independent of the legacy branch_and_cut translation unit.
constexpr double kHighsDefaultMipFeasibilityTolerance = 1e-6;

struct StrictScucDynamicCutCandidate {
  BCDynamicNodeCut cut;
  double score{0.0};
  double violation{0.0};
  double norm{0.0};
  int active_nnz{0};
  int ordinal{0};
};

int strict_scuc_uc_col(const std::vector<int>& cols,
                       const MIPModel::UCGenHint& uc, int g, int t) {
  if (g < 0 || t < 0 || g >= uc.ng || t >= uc.T) return -1;
  const std::size_t pos = static_cast<std::size_t>(t * uc.ng + g);
  return pos < cols.size() ? cols[pos] : -1;
}

int strict_scuc_env_int(const char* name, int fallback, int lo, int hi) {
  const char* raw = bc_env_options().value(name);
  if (raw == nullptr || *raw == '\0') return fallback;
  char* end = nullptr;
  const long parsed = std::strtol(raw, &end, 10);
  if (end == raw) return fallback;
  return std::max(lo, std::min(hi, static_cast<int>(parsed)));
}

double strict_scuc_env_double(const char* name, double fallback,
                              double lo, double hi) {
  const char* raw = bc_env_options().value(name);
  if (raw == nullptr || *raw == '\0') return fallback;
  char* end = nullptr;
  const double parsed = std::strtod(raw, &end);
  if (end == raw || !std::isfinite(parsed)) return fallback;
  return std::max(lo, std::min(hi, parsed));
}

double strict_scuc_original_value(const BCDynamicNodeCutContext& ctx, int col) {
  if (col < 0 || col >= static_cast<int>(ctx.original_col_value.size())) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return ctx.original_col_value[static_cast<std::size_t>(col)];
}

}  // namespace

bool strict_scuc_dynamic_cuts_enabled(const MIPModel::UCGenHint& uc) {
  if (!bc_env_options().scuc_dynamic_cuts_enabled) {
    return false;
  }
  if (bc_env_options().scuc_dynamic_cuts_disabled) {
    return false;
  }
  const std::size_t block = static_cast<std::size_t>(std::max(0, uc.ng) *
                                                    std::max(0, uc.T));
  if (uc.ng <= 0 || uc.T <= 0 || block == 0) return false;
  const bool has_transition_maps = uc.ig_cols.size() >= block &&
                                   uc.su_cols.size() >= block &&
                                   uc.sd_cols.size() >= block;
  return has_transition_maps &&
     (uc.certifies_min_up_down_rows || uc.certifies_ramping_rows ||
      uc.certifies_system_reserve_rows);
}

void append_strict_scuc_dynamic_cuts(const MIPModel::UCGenHint& uc,
                                    const BCOptions& opt,
                                    const BCDynamicNodeCutContext& ctx,
                                    const LPModel* original_lp,
                                    std::vector<BCDynamicNodeCut>& rows) {
  if (!strict_scuc_dynamic_cuts_enabled(uc)) return;
  const bool root_event =
      ctx.depth <= 0 || ctx.event.find("root") != std::string::npos;
  if (root_event) return;
  const int max_depth = opt.max_cut_depth > 0 ? opt.max_cut_depth : 4;
  if (!root_event && ctx.depth > max_depth) return;
  if (ctx.original_col_value.empty()) return;

  const int ng = uc.ng;
  const int T = uc.T;
  const double abs_tol = strict_scuc_env_double(
      "MIPSOLVERS_SCUC_DYNAMIC_MIN_VIOLATION", 1e-6, 0.0, 1e-2);
  const double eff_tol = strict_scuc_env_double(
      "MIPSOLVERS_SCUC_DYNAMIC_MIN_EFFICACY", 1e-8, 0.0, 1e-2);
  const double pair_floor = strict_scuc_env_double(
      "MIPSOLVERS_SCUC_DYNAMIC_PAIR_FLOOR", 1e-5, 0.0, 0.25);
  const int depth_budget = ctx.depth <= 1 ? 4 : (ctx.depth <= 3 ? 6 : 3);
  const int max_return = strict_scuc_env_int(
      "MIPSOLVERS_SCUC_DYNAMIC_MAX_CUTS", depth_budget, 1, 32);
  const int max_per_family = strict_scuc_env_int(
      "MIPSOLVERS_SCUC_DYNAMIC_MAX_PER_FAMILY", 1, 1, 16);
  const double min_score_factor = strict_scuc_env_double(
      "MIPSOLVERS_SCUC_DYNAMIC_MIN_SCORE_FACTOR", 0.20, 0.0, 1.0);
  const double max_parallelism = strict_scuc_env_double(
      "MIPSOLVERS_SCUC_DYNAMIC_MAX_PARALLELISM", 0.10, 0.0, 1.0);
  const bool unit_ramp_cuts = bc_env_options().scuc_dynamic_unit_ramp_cuts;

  std::vector<StrictScucDynamicCutCandidate> candidates;
  candidates.reserve(static_cast<std::size_t>(max_return * 8));

  auto add_upper_cut = [&](std::vector<std::pair<int, double>> terms,
                           double upper, const char* family,
                           const std::string& key, int period,
                           bool integral, bool propagate) {
    if (!std::isfinite(upper) || terms.empty()) return;
    double activity = 0.0;
    double norm_sq = 0.0;
    double active_norm_sq = 0.0;
    int active_nnz = 0;
    for (const auto& [col, value] : terms) {
      if (col < 0 || col >= static_cast<int>(ctx.original_col_value.size()) ||
          !std::isfinite(value)) {
        return;
      }
      const double x = ctx.original_col_value[static_cast<std::size_t>(col)];
      if (!std::isfinite(x)) return;
      activity += value * x;
      norm_sq += value * value;
      if (original_lp != nullptr &&
          col < static_cast<int>(original_lp->vars.size())) {
        const auto& var = original_lp->vars[static_cast<std::size_t>(col)];
        const double lb = var.lb;
        const double ub = var.ub;
        if ((value > 0.0 && x > lb + kHighsDefaultMipFeasibilityTolerance) ||
            (value < 0.0 && x < ub - kHighsDefaultMipFeasibilityTolerance)) {
          active_norm_sq += value * value;
          ++active_nnz;
        }
      }
    }
    const double norm = std::sqrt(std::max(0.0, norm_sq));
    const double violation = activity - upper;
    const double scale =
      std::max({1.0, norm, std::fabs(activity), std::fabs(upper)});
    const double efficacy = violation / std::max(1.0, norm);
    if (violation <= abs_tol * scale || efficacy <= eff_tol) return;
    if (active_nnz == 0 || !(active_norm_sq > 0.0) ||
      !std::isfinite(active_norm_sq)) {
      active_nnz = static_cast<int>(terms.size());
      active_norm_sq = norm_sq;
    }
    if (active_nnz <= 0 || !(active_norm_sq > 0.0)) return;
    const double highs_score = violation /
        (static_cast<double>(active_nnz) * std::sqrt(active_norm_sq));
    if (!std::isfinite(highs_score)) return;

    BCDynamicNodeCut cut;
    cut.indices.reserve(terms.size());
    cut.values.reserve(terms.size());
    for (const auto& [col, value] : terms) {
      cut.indices.push_back(col);
      cut.values.push_back(value);
    }
    cut.upper = upper;
    cut.integral = integral;
    cut.propagate = propagate;
    cut.column_space = BCDynamicNodeCut::ColumnSpace::Original;
    cut.validity_scope = ValidityScope::GlobalCut;
    cut.audit_family = family;
    cut.audit_key = key;
    cut.audit_period = period;
    const int ordinal = static_cast<int>(candidates.size());
    candidates.push_back({std::move(cut), highs_score, violation, norm,
                          active_nnz, ordinal});
  };

  auto add_reserve_cover_cut = [&](const std::vector<double>& caps,
                                   double requirement,
                                   const char* family,
                                   int period) {
    if (!(requirement > 0.0) || caps.size() < static_cast<std::size_t>(ng)) {
      return;
    }
    std::vector<std::pair<int, double>> terms;
    terms.reserve(static_cast<std::size_t>(ng));
    for (int g = 0; g < ng; ++g) {
      const int ig = strict_scuc_uc_col(uc.ig_cols, uc, g, period);
      const double cap = std::max(0.0, caps[static_cast<std::size_t>(g)]);
      if (ig < 0 || !(cap > 0.0)) continue;
      terms.emplace_back(ig, -cap);
    }
    add_upper_cut(std::move(terms), -requirement, family,
                  fmt::format("t{}", period), period, false, true);
  };

  if (uc.certifies_system_reserve_rows &&
      uc.ig_cols.size() >= static_cast<std::size_t>(ng * T)) {
    for (int t = 0; t < T; ++t) {
      if (uc.reserve_requirement.size() >= static_cast<std::size_t>(T) &&
          uc.up_reserve_headroom_cap.size() >= static_cast<std::size_t>(ng)) {
        add_reserve_cover_cut(uc.up_reserve_headroom_cap,
                              uc.reserve_requirement[static_cast<std::size_t>(t)],
                              "SCUC_UP_RESERVE_COVER", t);
      }
      if (uc.spinning_requirement.size() >= static_cast<std::size_t>(T) &&
          uc.spinning_reserve_cap.size() >= static_cast<std::size_t>(ng)) {
        add_reserve_cover_cut(uc.spinning_reserve_cap,
                              uc.spinning_requirement[static_cast<std::size_t>(t)],
                              "SCUC_SPIN_RESERVE_COVER", t);
      }
      if (uc.regulation_up_requirement.size() >= static_cast<std::size_t>(T) &&
          uc.regulation_up_cap.size() >= static_cast<std::size_t>(ng)) {
        add_reserve_cover_cut(uc.regulation_up_cap,
                              uc.regulation_up_requirement[static_cast<std::size_t>(t)],
                              "SCUC_REGUP_RESERVE_COVER", t);
      }
      if (uc.regulation_down_requirement.size() >= static_cast<std::size_t>(T) &&
          uc.regulation_down_cap.size() >= static_cast<std::size_t>(ng)) {
        add_reserve_cover_cut(uc.regulation_down_cap,
                              uc.regulation_down_requirement[static_cast<std::size_t>(t)],
                              "SCUC_REGDOWN_RESERVE_COVER", t);
      }
    }
  }

  if (uc.certifies_ramping_rows &&
      uc.pg_cols.size() >= static_cast<std::size_t>(ng * T) &&
      uc.pmax.size() >= static_cast<std::size_t>(ng) &&
      uc.ramp.size() >= static_cast<std::size_t>(ng)) {
    for (int t = 1; t < T; ++t) {
      std::vector<std::pair<int, double>> up_terms;
      std::vector<std::pair<int, double>> down_terms;
      up_terms.reserve(static_cast<std::size_t>(4 * ng));
      down_terms.reserve(static_cast<std::size_t>(4 * ng));
      for (int g = 0; g < ng; ++g) {
        const int pg_t = strict_scuc_uc_col(uc.pg_cols, uc, g, t);
        const int pg_prev = strict_scuc_uc_col(uc.pg_cols, uc, g, t - 1);
        const int ig_prev = strict_scuc_uc_col(uc.ig_cols, uc, g, t - 1);
        const int ig_t = strict_scuc_uc_col(uc.ig_cols, uc, g, t);
        const int su_t = strict_scuc_uc_col(uc.su_cols, uc, g, t);
        const int sd_t = strict_scuc_uc_col(uc.sd_cols, uc, g, t);
        const double pmax = std::max(0.0, uc.pmax[static_cast<std::size_t>(g)]);
        const double ramp = std::min(pmax, std::max(0.0, uc.ramp[static_cast<std::size_t>(g)]));
        if (pg_t < 0 || pg_prev < 0 || ig_prev < 0 || ig_t < 0 || su_t < 0 || sd_t < 0) {
          continue;
        }
        if (unit_ramp_cuts) {
          add_upper_cut({{pg_t, 1.0}, {pg_prev, -1.0}, {ig_prev, -ramp}, {su_t, -pmax}},
                        0.0, "SCUC_RAMP_UP_PERSPECTIVE",
                        fmt::format("g{}:t{}", g, t), t, false, false);
          add_upper_cut({{pg_prev, 1.0}, {pg_t, -1.0}, {ig_t, -ramp}, {sd_t, -pmax}},
                        0.0, "SCUC_RAMP_DOWN_PERSPECTIVE",
                        fmt::format("g{}:t{}", g, t), t, false, false);
        }
        up_terms.emplace_back(pg_t, 1.0);
        up_terms.emplace_back(pg_prev, -1.0);
        up_terms.emplace_back(ig_prev, -ramp);
        up_terms.emplace_back(su_t, -pmax);
        down_terms.emplace_back(pg_prev, 1.0);
        down_terms.emplace_back(pg_t, -1.0);
        down_terms.emplace_back(ig_t, -ramp);
        down_terms.emplace_back(sd_t, -pmax);
      }
      add_upper_cut(std::move(up_terms), 0.0, "SCUC_SYSTEM_RAMP_UP",
                    fmt::format("t{}", t), t, false, false);
      add_upper_cut(std::move(down_terms), 0.0, "SCUC_SYSTEM_RAMP_DOWN",
                    fmt::format("t{}", t), t, false, false);
    }
  }

  if (!root_event && uc.certifies_min_up_down_rows) {
    for (int g = 0; g < ng; ++g) {
      const int min_up = (g < static_cast<int>(uc.min_up.size()))
          ? std::max(0, uc.min_up[static_cast<std::size_t>(g)])
          : 0;
      const int min_down = (g < static_cast<int>(uc.min_down.size()))
          ? std::max(0, uc.min_down[static_cast<std::size_t>(g)])
          : 0;
      if (min_up > 1) {
        for (int start = 0; start < T; ++start) {
          const int su = strict_scuc_uc_col(uc.su_cols, uc, g, start);
          if (su < 0) continue;
          const double su_val = strict_scuc_original_value(ctx, su);
          if (!(su_val > pair_floor)) continue;
          const int last = std::min(T - 1, start + min_up - 1);
          for (int shut = start + 1; shut <= last; ++shut) {
            const int sd = strict_scuc_uc_col(uc.sd_cols, uc, g, shut);
            if (sd < 0) continue;
            const double sd_val = strict_scuc_original_value(ctx, sd);
            if (!(sd_val > pair_floor) || su_val + sd_val <= 1.0 + pair_floor) continue;
            add_upper_cut({{su, 1.0}, {sd, 1.0}}, 1.0,
                          "SCUC_MINUP_START_SHUT_CONFLICT",
                          fmt::format("g{}:{}-{}", g, start, shut),
                          shut, true, true);
          }
        }
      }
      if (min_down > 1) {
        for (int shut = 0; shut < T; ++shut) {
          const int sd = strict_scuc_uc_col(uc.sd_cols, uc, g, shut);
          if (sd < 0) continue;
          const double sd_val = strict_scuc_original_value(ctx, sd);
          if (!(sd_val > pair_floor)) continue;
          const int last = std::min(T - 1, shut + min_down - 1);
          for (int start = shut + 1; start <= last; ++start) {
            const int su = strict_scuc_uc_col(uc.su_cols, uc, g, start);
            if (su < 0) continue;
            const double su_val = strict_scuc_original_value(ctx, su);
            if (!(su_val > pair_floor) || sd_val + su_val <= 1.0 + pair_floor) continue;
            add_upper_cut({{sd, 1.0}, {su, 1.0}}, 1.0,
                          "SCUC_MINDOWN_SHUT_START_CONFLICT",
                          fmt::format("g{}:{}-{}", g, shut, start),
                          start, true, true);
          }
        }
      }
    }
  }

  if (candidates.empty()) return;
  std::sort(candidates.begin(), candidates.end(),
            [](const StrictScucDynamicCutCandidate& a,
               const StrictScucDynamicCutCandidate& b) {
              if (a.score != b.score) return a.score > b.score;
              if (a.violation != b.violation) return a.violation > b.violation;
              if (a.cut.indices.size() != b.cut.indices.size()) {
                return a.cut.indices.size() < b.cut.indices.size();
              }
              return a.ordinal < b.ordinal;
            });
  const double best_score = candidates.front().score;
  const double min_score = min_score_factor * best_score;
  std::vector<StrictScucDynamicCutCandidate*> selected;
  selected.reserve(static_cast<std::size_t>(max_return));
  std::map<std::string, int> family_count;
  auto row_parallelism = [](const BCDynamicNodeCut& lhs, double lhs_norm,
                            const BCDynamicNodeCut& rhs, double rhs_norm) {
    if (!(lhs_norm > 0.0) || !(rhs_norm > 0.0)) return 0.0;
    double dot = 0.0;
    for (std::size_t i = 0; i < lhs.indices.size(); ++i) {
      const int col = lhs.indices[i];
      for (std::size_t j = 0; j < rhs.indices.size(); ++j) {
        if (rhs.indices[j] == col) dot += lhs.values[i] * rhs.values[j];
      }
    }
    return dot / (lhs_norm * rhs_norm);
  };

  for (StrictScucDynamicCutCandidate& cand : candidates) {
    if (static_cast<int>(selected.size()) >= max_return) break;
    if (cand.score < min_score) break;
    const std::string family = cand.cut.audit_family;
    if (family_count[family] >= max_per_family) continue;
    bool parallel = false;
    for (const StrictScucDynamicCutCandidate* keep : selected) {
      if (row_parallelism(cand.cut, cand.norm, keep->cut, keep->norm) >
          max_parallelism) {
        parallel = true;
        break;
      }
    }
    if (parallel) continue;
    ++family_count[family];
    selected.push_back(&cand);
  }

  if (const char* trace_env = bc_env_options().value("MIPSOLVERS_SCUC_DYNAMIC_SCORE_TRACE")) {
    int trace_limit = 16;
    if (std::string(trace_env) == "all") {
      trace_limit = std::numeric_limits<int>::max();
    } else if (trace_env[0] != '\0') {
      char* end = nullptr;
      const long parsed = std::strtol(trace_env, &end, 10);
      if (end != trace_env) {
        trace_limit = parsed > 0
                          ? static_cast<int>(std::min<long>(parsed, 1000000))
                          : 0;
      }
    }
    for (int ord = 0; ord < static_cast<int>(candidates.size()) && ord < trace_limit;
         ++ord) {
      const auto& cand = candidates[static_cast<std::size_t>(ord)];
      fmt::print(stderr,
                 "[SCUC-DYNCUT-SCORED] ord={} family={} key={} score={:.12g} "
                 "viol={:.12g} active={} nnz={} norm={:.12g}\n",
                 ord, cand.cut.audit_family, cand.cut.audit_key, cand.score,
                 cand.violation, cand.active_nnz,
                 static_cast<int>(cand.cut.indices.size()), cand.norm);
    }
    fmt::print(stderr,
               "[SCUC-DYNCUT-SELECT] selected={} candidates={} bestScore={:.12g} "
               "minScore={:.12g} maxPar={:.3g} maxPerFamily={}\n",
               static_cast<int>(selected.size()), static_cast<int>(candidates.size()),
               best_score, min_score, max_parallelism, max_per_family);
  }

  rows.reserve(rows.size() + selected.size());
  for (StrictScucDynamicCutCandidate* cand : selected) {
    rows.push_back(std::move(cand->cut));
  }
}

}  // namespace mipsolvers::engine::detail

#endif  // MIPSOLVERS_HAVE_HIGHS_LIB
