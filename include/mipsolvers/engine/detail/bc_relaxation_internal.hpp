/// @file bc_relaxation_internal.hpp
/// @brief Shared internal helpers for the split bc_relaxation translation units.
///
/// These were file-local (anonymous namespace) helpers in bc_relaxation.cpp.
/// They are promoted to inline `detail`-namespace functions so the relaxation
/// core and the extracted basis-recovery unit can share one definition.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <Eigen/Core>

#include "mipsolvers/engine/detail/bc_env_options.hpp"
#include "mipsolvers/engine/detail/bc_utils.hpp"

namespace mipsolvers::engine::detail {

inline bool lp_basis_trace_enabled() {
  return bc_env_options().value("MIPSOLVERS_LP_BASIS_TRACE") != nullptr;
}

inline int lp_basis_trace_terms() {
  const char* env = bc_env_options().value("MIPSOLVERS_LP_BASIS_TRACE_TERMS");
  if (env == nullptr) return 8;
  char* end = nullptr;
  long val = std::strtol(env, &end, 10);
  if (end == env) return 8;
  return static_cast<int>(std::clamp<long>(val, 0, 64));
}

struct BasisComposition {
  int orig{0};
  int slack{0};
  int surplus{0};
  int artificial{0};
  int invalid{0};
  int duplicate{0};
  int upper{0};
};

inline const char* basis_col_kind(const StandardFormLP& sf, int col) {
  if (col < 0 || col >= static_cast<int>(sf.A.cols())) return "invalid";
  if (col < sf.n_original) return "orig";
  if (col < sf.n_original + sf.n_slack) return "slack";
  if (col < sf.n_original + sf.n_slack + sf.n_surplus) return "surplus";
  if (col < sf.n_original + sf.n_slack + sf.n_surplus + sf.n_artificial) return "art";
  return "unknown";
}

inline BasisComposition basis_composition(const StandardFormLP& sf,
                                          const std::vector<int>& basis,
                                          const std::vector<char>& at_upper) {
  const int n = static_cast<int>(sf.A.cols());
  BasisComposition comp;
  std::vector<char> seen(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int idx : basis) {
    if (idx < 0 || idx >= n) {
      ++comp.invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(idx)]) ++comp.duplicate;
    seen[static_cast<std::size_t>(idx)] = 1;
    const char* kind = basis_col_kind(sf, idx);
    if (std::strcmp(kind, "orig") == 0) ++comp.orig;
    else if (std::strcmp(kind, "slack") == 0) ++comp.slack;
    else if (std::strcmp(kind, "surplus") == 0) ++comp.surplus;
    else if (std::strcmp(kind, "art") == 0) ++comp.artificial;
  }
  for (int j = 0; j < n && j < static_cast<int>(at_upper.size()); ++j) {
    if (j < static_cast<int>(seen.size()) && seen[static_cast<std::size_t>(j)]) continue;
    if (at_upper[static_cast<std::size_t>(j)]) ++comp.upper;
  }
  return comp;
}

inline void trace_basis_source(const char* stage,
                               const StandardFormLP& sf,
                               const SimplexBasis& basis,
                               const char* detail = "") {
  if (!lp_basis_trace_enabled()) return;
  const auto indices = basis.basis_indices();
  const BasisComposition comp = basis_composition(sf, indices, basis.at_upper);
  fprintf(stderr,
          "[B&C-LPBASIS-SOURCE] stage=%s m=%d n=%d nOrig=%d nSlack=%d "
          "nSurplus=%d nArt=%d basic=orig%d:slack%d:surplus%d:art%d "
          "invalid=%d dup=%d nonbasicUpper=%d %s sample=[",
          stage == nullptr ? "unknown" : stage,
          static_cast<int>(sf.A.rows()), static_cast<int>(sf.A.cols()),
          sf.n_original, sf.n_slack, sf.n_surplus, sf.n_artificial,
          comp.orig, comp.slack, comp.surplus, comp.artificial, comp.invalid,
          comp.duplicate, comp.upper, detail == nullptr ? "" : detail);
  const int max_terms = lp_basis_trace_terms();
  for (int i = 0; i < static_cast<int>(indices.size()) && i < max_terms; ++i) {
    if (i > 0) std::fprintf(stderr, ";");
    const int col = indices[static_cast<std::size_t>(i)];
    std::fprintf(stderr, "%d:%d:%s", i, col, basis_col_kind(sf, col));
  }
  std::fprintf(stderr, "]\n");
}

inline double row_col_coefficient(const StandardFormLP& sf, int row, int col) {
  for (StandardRowMatrix::InnerIterator it(sf.A_row, row); it; ++it) {
    if (it.col() == col) return it.value();
  }
  return 0.0;
}

inline Eigen::VectorXd primal_to_scaled_standard_form(const StandardFormLP& sf,
                                                      const Eigen::VectorXd& x_orig) {
  const int sf_m = static_cast<int>(sf.A.rows());
  const int sf_n = static_cast<int>(sf.A.cols());
  const bool have_col_scale = (sf.col_scale.size() == sf_n);
  Eigen::VectorXd x_sf = Eigen::VectorXd::Zero(sf_n);
  for (int j = 0; j < sf.n_original && j < x_orig.size(); ++j) {
    double x_shifted = x_orig[j] - sf.lb_shift[j];
    x_sf[j] = have_col_scale ? (x_shifted / sf.col_scale[j]) : x_shifted;
  }
  Eigen::VectorXd ax = sf.A * x_sf;
  for (int i = 0; i < sf_m; ++i) {
    int sc = sf.row_to_slack_col[i];
    if (sc >= 0) {
      const double col_scale = have_col_scale ? sf.col_scale[sc] : 1.0;
      const double a_sc = row_col_coefficient(sf, i, sc);
      if (std::abs(a_sc) > 1e-15) {
        x_sf[sc] = std::max(0.0, (sf.b[i] - ax[i]) / a_sc);
      } else {
        x_sf[sc] = have_col_scale
            ? std::max(0.0, sf.b[i] - ax[i]) / col_scale
            : std::max(0.0, sf.b[i] - ax[i]);
      }
    }
    int su = sf.row_to_surplus_col[i];
    if (su >= 0) {
      const double col_scale = have_col_scale ? sf.col_scale[su] : 1.0;
      const double a_su = row_col_coefficient(sf, i, su);
      if (std::abs(a_su) > 1e-15) {
        x_sf[su] = std::max(0.0, (sf.b[i] - ax[i]) / a_su);
      } else {
        x_sf[su] = have_col_scale
            ? std::max(0.0, ax[i] - sf.b[i]) / col_scale
            : std::max(0.0, ax[i] - sf.b[i]);
      }
    }
  }
  return x_sf;
}

CrashBasisRecoveryStats recover_primal_activity_basis_impl(
    const StandardFormLP& sf, const Eigen::VectorXd& x_orig,
    const SolveResult* ipm_res, const SimplexBasis* seed_basis,
    SimplexBasis& out_basis);

SimplexBasis build_primal_crash_basis(const StandardFormLP& sf,
                                      const Eigen::VectorXd& x_orig);

}  // namespace mipsolvers::engine::detail
