/// @file bc_uc_trace.cpp
/// @brief UC-specific progressive-rounding trace and rounding helpers.

#include "mipsolvers/engine/detail/bc_uc_trace.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace mipsolvers::engine::detail {

std::vector<ProgRoundTraceFocusVar> make_prog_round_focus_ig_trace(
    const std::optional<MIPModel::UCGenHint>& uc_hint,
    const Eigen::VectorXd& x_relax,
    const Eigen::VectorXd& lb,
    const Eigen::VectorXd& ub) {
  constexpr int kFocusGenerators[] = {53, 45, 13};
  constexpr int kFocusPeriods[] = {0, 1, 2, 3, 4, 23};

  std::vector<ProgRoundTraceFocusVar> out;
  if (!uc_hint.has_value()) return out;

  const auto& uc = *uc_hint;
  const int n = static_cast<int>(x_relax.size());
  const bool use_cols = !uc.ig_cols.empty();
  out.reserve(static_cast<size_t>(sizeof(kFocusGenerators) / sizeof(kFocusGenerators[0]) *
                                  sizeof(kFocusPeriods) / sizeof(kFocusPeriods[0])));

  for (int g : kFocusGenerators) {
    if (g < 0 || g >= uc.ng) continue;
    for (int t : kFocusPeriods) {
      if (t < 0 || t >= uc.T) continue;
      const size_t pos = static_cast<size_t>(t * uc.ng + g);
      int col = -1;
      if (use_cols) {
        if (pos < uc.ig_cols.size()) col = uc.ig_cols[pos];
      } else if (uc.ig_start >= 0) {
        col = uc.ig_start + static_cast<int>(pos);
      }

      ProgRoundTraceFocusVar focus;
      focus.g = g;
      focus.t = t;
      focus.col = col;
      focus.available = (col >= 0 && col < n);
      if (focus.available) {
        focus.root_val = x_relax[col];
        focus.last_fixed = (ub[col] - lb[col] < 1e-9);
        focus.last_value = focus.last_fixed ? lb[col] : x_relax[col];
      }
      out.push_back(focus);
    }
  }
  return out;
}

void annotate_prog_round_focus_frac_ranks(
    std::vector<ProgRoundTraceFocusVar>& focus,
    const std::vector<int>& frac_cols) {
  for (auto& entry : focus) {
    entry.frac_rank = -1;
    if (!entry.available || entry.col < 0) continue;
    for (int k = 0; k < static_cast<int>(frac_cols.size()); ++k) {
      if (frac_cols[static_cast<size_t>(k)] == entry.col) {
        entry.frac_rank = k;
        break;
      }
    }
  }
}

void print_prog_round_focus_initial(const char* phase,
                                    const std::vector<ProgRoundTraceFocusVar>& focus) {
  if (focus.empty()) return;

  fprintf(stderr, "[%s-TRACE] tracking %zu focused IG vars\n",
          phase, focus.size());
  for (const auto& entry : focus) {
    if (!entry.available || entry.col < 0) {
      fprintf(stderr,
              "[%s-TRACE] start IG(g=%d,t=%d) col=-1 status=eliminated-by-presolve\n",
              phase, entry.g, entry.t);
      continue;
    }
    const int frac_rank = (entry.frac_rank >= 0) ? (entry.frac_rank + 1) : -1;
    if (entry.last_fixed) {
      fprintf(stderr,
              "[%s-TRACE] start IG(g=%d,t=%d) col=%d root=%.3f frac_rank=%d status=fixed value=%.0f\n",
              phase, entry.g, entry.t, entry.col, entry.root_val, frac_rank,
              entry.last_value);
    } else {
      fprintf(stderr,
              "[%s-TRACE] start IG(g=%d,t=%d) col=%d root=%.3f frac_rank=%d status=free\n",
              phase, entry.g, entry.t, entry.col, entry.root_val, frac_rank);
    }
  }
}

void log_prog_round_focus_changes(const char* phase,
                                  const std::string& source,
                                  std::vector<ProgRoundTraceFocusVar>& focus,
                                  const Eigen::VectorXd& lb,
                                  const Eigen::VectorXd& ub) {
  if (focus.empty()) return;

  for (auto& entry : focus) {
    if (!entry.available || entry.col < 0) continue;
    const bool fixed = (ub[entry.col] - lb[entry.col] < 1e-9);
    const double value = fixed ? lb[entry.col] : entry.root_val;
    const bool changed = (fixed != entry.last_fixed) ||
                         (fixed && std::abs(value - entry.last_value) > 1e-9);
    if (!changed) continue;

    const int frac_rank = (entry.frac_rank >= 0) ? (entry.frac_rank + 1) : -1;
    if (fixed) {
      fprintf(stderr,
              "[%s-TRACE] %s IG(g=%d,t=%d) col=%d root=%.3f frac_rank=%d fixed=%.0f\n",
              phase, source.c_str(), entry.g, entry.t, entry.col, entry.root_val,
              frac_rank, value);
    } else {
      fprintf(stderr,
              "[%s-TRACE] %s IG(g=%d,t=%d) col=%d root=%.3f frac_rank=%d released\n",
              phase, source.c_str(), entry.g, entry.t, entry.col, entry.root_val,
              frac_rank);
    }

    entry.last_fixed = fixed;
    entry.last_value = value;
  }
}

// ════════════════════════════════════════════════════════════════════════════
// UC-aware commitment rounding
// ════════════════════════════════════════════════════════════════════════════
// When the MILP encodes a unit-commitment problem, simple independent rounding
// of IG/SU/SD leaves coupling constraints violated.  This helper rounds IG for
// each generator as contiguous on/off blocks respecting min-up/min-down, then
// derives SU/SD from transitions.

void uc_round_commitment(Eigen::VectorXd& x,
                         const MIPModel::UCGenHint& uc,
                         int ng_stride) {
  const int ng = uc.ng;
  const int T = uc.T;
  const bool use_cols = !uc.ig_cols.empty();
  auto ig_col = [&](int g, int h) -> int {
    if (use_cols) {
      size_t idx = static_cast<size_t>(h * ng + g);
      return idx < uc.ig_cols.size() ? uc.ig_cols[idx] : -1;
    }
    return uc.ig_start + h * ng_stride + g;
  };
  auto su_col = [&](int g, int h) -> int {
    if (use_cols) {
      size_t idx = static_cast<size_t>(h * ng + g);
      return idx < uc.su_cols.size() ? uc.su_cols[idx] : -1;
    }
    return uc.su_start + h * ng_stride + g;
  };
  auto sd_col = [&](int g, int h) -> int {
    if (use_cols) {
      size_t idx = static_cast<size_t>(h * ng + g);
      return idx < uc.sd_cols.size() ? uc.sd_cols[idx] : -1;
    }
    if (uc.sd_start == uc.su_start) return -1;
    return uc.sd_start + h * ng_stride + g;
  };
  const int n = static_cast<int>(x.size());

  for (int g = 0; g < ng; ++g) {
    const int mu = (g < static_cast<int>(uc.min_up.size())) ? uc.min_up[g] : 1;
    const int md = (g < static_cast<int>(uc.min_down.size())) ? uc.min_down[g] : 1;
    const int ig0 = (g < static_cast<int>(uc.ig0.size())) ? uc.ig0[g] : 0;

    // Phase 1: greedy forward sweep — round IG respecting min-up/min-down.
    std::vector<int> ig(static_cast<size_t>(T));
    int prev = ig0;
    int time_in_state = 100;

    for (int h = 0; h < T; ++h) {
      int ic = ig_col(g, h);
      const double frac = (ic >= 0 && ic < n) ? x[ic] : 0.0;
      int desired = (frac >= 0.5) ? 1 : 0;

      if (prev == 1 && desired == 0 && time_in_state < mu) {
        desired = 1;
      } else if (prev == 0 && desired == 1 && time_in_state < md) {
        desired = 0;
      }

      ig[h] = desired;
      if (desired == prev) {
        ++time_in_state;
      } else {
        time_in_state = 1;
      }
      prev = desired;
    }

    // Phase 2: write back IG and derive SU, SD.
    int prev_ig = ig0;
    for (int h = 0; h < T; ++h) {
      int ic = ig_col(g, h);
      if (ic >= 0 && ic < n) x[ic] = static_cast<double>(ig[h]);
      const int su = (ig[h] == 1 && prev_ig == 0) ? 1 : 0;
      const int sd = (ig[h] == 0 && prev_ig == 1) ? 1 : 0;
      int sc = su_col(g, h);
      if (sc >= 0 && sc < n) x[sc] = static_cast<double>(su);
      int dc = sd_col(g, h);
      if (dc >= 0 && dc < n) x[dc] = static_cast<double>(sd);
      prev_ig = ig[h];
    }
  }
}


}  // namespace mipsolvers::engine::detail
