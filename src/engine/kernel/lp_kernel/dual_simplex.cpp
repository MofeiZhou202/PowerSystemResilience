// Public LP-kernel integration. The native revised-simplex algorithms live in
// native_dual/; this file owns result translation, explicit HiGHS integration,
// basis diagnostics, and the stable BasisOps forwarding surface.

#include "mipsolvers/engine/kernel/lp_kernel/dual_simplex.hpp"
#include "native_dual_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <fmt/format.h>

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
#include "Highs.h"
#endif

#include "mipsolvers/core/logging.hpp"
#include "mipsolvers/engine/strategy/highs_presolve_side_state.hpp"

namespace mipsolvers::engine {

SparseFactorTelemetry BasisOps::factor_telemetry() const {
  return {};
}


namespace {


constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kHighsDefaultKktTolerance = 1e-7;
constexpr double kHighsDefaultMipTolerance = 1e-6;

bool simplex_wall_time_limit_hit(
    const SimplexOptions& opt,
    const std::chrono::steady_clock::time_point& start) {
  if (!(opt.time_limit_sec > 0.0) || !std::isfinite(opt.time_limit_sec)) {
    return false;
  }
  const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  if (elapsed < opt.time_limit_sec) return false;
  if (opt.time_limit_hit != nullptr) *opt.time_limit_hit = true;
  return true;
}

void mark_simplex_time_limit(SolveStats& stats,
                             const std::chrono::steady_clock::time_point& start) {
  stats.success = false;
  if (!stats.status.starts_with("Time limit")) stats.status = "Time limit";
  stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
}

std::vector<char> basis_basic_mask(int n, const std::vector<int>& basis) {
  std::vector<char> is_basic(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int idx : basis) {
    if (idx >= 0 && idx < n) is_basic[static_cast<std::size_t>(idx)] = 1;
  }
  return is_basic;
}


Eigen::VectorXd extract_solution(const StandardFormLP& sf, const Eigen::VectorXd& x_std) {
  Eigen::VectorXd x = sf.lb_shift;
  if (sf.col_scale.size() > 0) {
    // x_std is in scaled space; unscale: x_orig[j] = col_scale[j] * x_scaled[j]
    for (int j = 0; j < sf.n_original; ++j)
      x[j] += sf.col_scale[j] * x_std[j];
  } else {
    x += x_std.head(sf.n_original);
  }
  return x;
}

Eigen::VectorXd extract_primal_ray(const StandardFormLP& sf,
                                   const Eigen::VectorXd& ray_std) {
  if (ray_std.size() != sf.A.cols()) return {};
  Eigen::VectorXd ray = ray_std.head(sf.n_original);
  if (sf.col_scale.size() == sf.A.cols()) {
    ray.array() *= sf.col_scale.head(sf.n_original).array();
  }
  const double norm = ray.size() > 0 ? ray.lpNorm<Eigen::Infinity>() : 0.0;
  if (norm > 0.0 && std::isfinite(norm)) ray /= norm;
  return ray;
}

bool lp_basis_trace_enabled() {
  const char* env = std::getenv("MIPSOLVERS_LP_BASIS_TRACE");
  return env != nullptr && env[0] != '\0' && std::string(env) != "0";
}

int lp_basis_trace_terms() {
  const char* env = std::getenv("MIPSOLVERS_LP_BASIS_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 24;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 24;
  return static_cast<int>(std::min<long>(value, 200));
}

std::uint64_t lp_basis_hash_combine(std::uint64_t seed,
                                    std::uint64_t value) {
  seed ^= value + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
  return seed;
}

std::uint64_t lp_basis_hash_double(double value) {
  if (std::isinf(value)) return value > 0.0 ? 0x7ff0000000000000ULL
                                            : 0xfff0000000000000ULL;
  if (!std::isfinite(value)) return 0x7ff8000000000000ULL;
  const auto q = static_cast<std::int64_t>(std::llround(value * 1e9));
  return static_cast<std::uint64_t>(q) ^ 0x517cc1b727220a95ULL;
}

double lp_basis_row_scale_or_one(const StandardFormLP& sf, int row) {
  if (row >= 0 && row < sf.row_scale.size()) {
    const double scale = sf.row_scale[row];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

double lp_basis_col_scale_or_one(const StandardFormLP& sf, int col) {
  if (col >= 0 && col < sf.col_scale.size()) {
    const double scale = sf.col_scale[col];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

double lp_basis_original_value(const StandardFormLP& sf,
                               int col,
                               double scaled_value) {
  if (col < 0) return scaled_value;
  const double col_scale = lp_basis_col_scale_or_one(sf, col);
  if (col >= sf.n_original) return col_scale * scaled_value;
  const double shift =
      col < sf.lb_shift.size() ? sf.lb_shift[col] : 0.0;
  return shift + col_scale * scaled_value;
}

double lp_basis_source_row_coeff(const StandardFormLP& sf, int row, int col) {
  if (row < 0 || row >= sf.A_row.rows() || col < 0 ||
      col >= sf.n_original) {
    return 0.0;
  }
  const double scaled = sf.A_row.coeff(row, col);
  if (std::abs(scaled) <= 1e-15) return 0.0;
  return scaled / (lp_basis_row_scale_or_one(sf, row) *
                   lp_basis_col_scale_or_one(sf, col));
}

double lp_basis_source_row_side(const StandardFormLP& sf, int row) {
  if (row < 0 || row >= sf.A_row.rows()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const int sign = row < static_cast<int>(sf.row_sign.size())
                       ? sf.row_sign[static_cast<std::size_t>(row)]
                       : 1;
  if (sf.row_rhs_value.size() == sf.A_row.rows()) {
    return static_cast<double>(sign) * sf.row_rhs_value[row];
  }
  return sf.b[row] / lp_basis_row_scale_or_one(sf, row);
}

bool lp_basis_logical_row_bounds(const StandardFormLP& sf,
                                 int row,
                                 double& lower,
                                 double& upper) {
  const int m = static_cast<int>(sf.A_row.rows());
  const int n_std = static_cast<int>(sf.A.cols());
  if (row < 0 || row >= m || row >= sf.b.size()) return false;

  lower = -std::numeric_limits<double>::infinity();
  upper = std::numeric_limits<double>::infinity();
  const double side = lp_basis_source_row_side(sf, row);
  if (!std::isfinite(side)) return false;

  auto activity_domain = [&]() {
    double min_activity = 0.0;
    double max_activity = 0.0;
    bool min_finite = true;
    bool max_finite = true;
    for (StandardRowMatrix::InnerIterator it(
             sf.A_row, row);
         it; ++it) {
      const int j = static_cast<int>(it.col());
      if (j < 0 || j >= sf.n_original) continue;
      const double a = lp_basis_source_row_coeff(sf, row, j);
      if (std::abs(a) <= 1e-12) continue;
      const double lb =
          j < sf.lb_shift.size() ? sf.lb_shift[j]
                                 : -std::numeric_limits<double>::infinity();
      const double ub =
          j < sf.var_ub.size()
              ? lp_basis_original_value(sf, j, sf.var_ub[j])
              : std::numeric_limits<double>::infinity();
      if (a >= 0.0) {
        if (std::isfinite(lb) && min_finite) min_activity += a * lb;
        else min_finite = false;
        if (std::isfinite(ub) && max_finite) max_activity += a * ub;
        else max_finite = false;
      } else {
        if (std::isfinite(ub) && min_finite) min_activity += a * ub;
        else min_finite = false;
        if (std::isfinite(lb) && max_finite) max_activity += a * lb;
        else max_finite = false;
      }
    }
    return std::pair<double, double>{
        min_finite ? min_activity : -std::numeric_limits<double>::infinity(),
        max_finite ? max_activity : std::numeric_limits<double>::infinity()};
  };

  const int slack_col =
      row < static_cast<int>(sf.row_to_slack_col.size())
          ? sf.row_to_slack_col[static_cast<std::size_t>(row)]
          : -1;
  const int surplus_col =
      row < static_cast<int>(sf.row_to_surplus_col.size())
          ? sf.row_to_surplus_col[static_cast<std::size_t>(row)]
          : -1;

  if (slack_col >= 0) {
    upper = side;
    if (slack_col < n_std && slack_col < sf.var_ub.size() &&
        std::isfinite(sf.var_ub[slack_col])) {
      const double width =
          sf.var_ub[slack_col] * lp_basis_col_scale_or_one(sf, slack_col);
      if (!std::isfinite(width) || width < -1e-9) return false;
      lower = upper - std::max(0.0, width);
    } else {
      lower = activity_domain().first;
    }
    return std::isfinite(upper);
  }

  if (surplus_col >= 0) {
    lower = side;
    upper = activity_domain().second;
    return std::isfinite(lower);
  }

  lower = side;
  upper = side;
  return std::isfinite(lower);
}

struct LpBasisColumnInfo {
  std::vector<int> row_for_col;
  std::vector<unsigned char> kind_for_col;
};

enum : unsigned char {
  kLpBasisColOriginal = 0,
  kLpBasisColSlack = 1,
  kLpBasisColSurplus = 2,
  kLpBasisColArtificial = 3,
  kLpBasisColUnknown = 4,
};

const char* lp_basis_col_kind_name(unsigned char kind) {
  switch (kind) {
    case kLpBasisColOriginal:
      return "orig";
    case kLpBasisColSlack:
      return "slack";
    case kLpBasisColSurplus:
      return "surplus";
    case kLpBasisColArtificial:
      return "art";
    default:
      return "unknown";
  }
}

LpBasisColumnInfo lp_basis_build_column_info(const StandardFormLP& sf) {
  LpBasisColumnInfo info;
  const int n = static_cast<int>(sf.A.cols());
  info.row_for_col.assign(static_cast<std::size_t>(n), -1);
  info.kind_for_col.assign(static_cast<std::size_t>(n), kLpBasisColUnknown);
  for (int j = 0; j < std::min(n, sf.n_original); ++j) {
    info.kind_for_col[static_cast<std::size_t>(j)] = kLpBasisColOriginal;
  }
  const int m = static_cast<int>(sf.A.rows());
  for (int row = 0; row < m; ++row) {
    auto mark = [&](const std::vector<int>& map, unsigned char kind) {
      if (row >= static_cast<int>(map.size())) return;
      const int col = map[static_cast<std::size_t>(row)];
      if (col < 0 || col >= n) return;
      info.row_for_col[static_cast<std::size_t>(col)] = row;
      info.kind_for_col[static_cast<std::size_t>(col)] = kind;
    };
    mark(sf.row_to_slack_col, kLpBasisColSlack);
    mark(sf.row_to_surplus_col, kLpBasisColSurplus);
    mark(sf.row_to_artificial_col, kLpBasisColArtificial);
  }
  return info;
}

std::uint64_t lp_basis_semantic_row_hash(const StandardFormLP& sf, int row) {
  double lower = 0.0;
  double upper = 0.0;
  lp_basis_logical_row_bounds(sf, row, lower, upper);
  std::vector<std::pair<int, double>> terms;
  terms.reserve(static_cast<std::size_t>(sf.A_row.row_end(row) -
                                         sf.A_row.row_start(row)));
  double first = 0.0;
  for (StandardRowMatrix::InnerIterator it(
           sf.A_row, row);
       it; ++it) {
    const int col = static_cast<int>(it.col());
    if (col < 0 || col >= sf.n_original) continue;
    const double val = lp_basis_source_row_coeff(sf, row, col);
    if (std::abs(val) <= 1e-12) continue;
    if (first == 0.0) first = val;
    terms.emplace_back(col, val);
  }
  if (first < 0.0) {
    for (auto& term : terms) term.second = -term.second;
    const double old_lower = lower;
    lower = std::isfinite(upper) ? -upper : -upper;
    upper = std::isfinite(old_lower) ? -old_lower : -old_lower;
  }
  std::uint64_t h = 0x4c50424153495331ULL;
  h = lp_basis_hash_combine(h, lp_basis_hash_double(lower));
  h = lp_basis_hash_combine(h, lp_basis_hash_double(upper));
  h = lp_basis_hash_combine(h, static_cast<std::uint64_t>(terms.size()));
  for (const auto& term : terms) {
    h = lp_basis_hash_combine(h, static_cast<std::uint64_t>(term.first));
    h = lp_basis_hash_combine(h, lp_basis_hash_double(term.second));
  }
  return h;
}


void trace_lp_basis_hint_state(const StandardFormLP& sf,
                               const std::vector<int>& basis,
                               const std::vector<char>& at_upper,
                               const char* stage,
                               int call_id,
                               bool hint_match) {
  if (!lp_basis_trace_enabled()) return;
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  const auto col_info = lp_basis_build_column_info(sf);

  int basic_orig = 0, basic_slack = 0, basic_surplus = 0, basic_art = 0;
  int basic_unknown = 0, invalid = 0, dup_cols = 0;
  std::vector<char> seen(static_cast<std::size_t>(std::max(0, n)), 0);
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) {
      ++invalid;
      continue;
    }
    if (seen[static_cast<std::size_t>(col)]) ++dup_cols;
    seen[static_cast<std::size_t>(col)] = 1;
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal) ++basic_orig;
    else if (kind == kLpBasisColSlack) ++basic_slack;
    else if (kind == kLpBasisColSurplus) ++basic_surplus;
    else if (kind == kLpBasisColArtificial) ++basic_art;
    else ++basic_unknown;
  }

  int nonbasic_lower = 0;
  int nonbasic_upper = 0;
  for (int j = 0; j < n && j < static_cast<int>(seen.size()); ++j) {
    if (seen[static_cast<std::size_t>(j)]) continue;
    if (j < static_cast<int>(at_upper.size()) &&
        at_upper[static_cast<std::size_t>(j)]) {
      ++nonbasic_upper;
    } else {
      ++nonbasic_lower;
    }
  }

  struct DuplicateRows {
    int total{0};
    std::vector<int> rows;
    std::vector<int> basic_rows;
    std::vector<int> basic_cols;
  };
  std::unordered_map<std::uint64_t, DuplicateRows> row_classes;
  row_classes.reserve(static_cast<std::size_t>(m * 2 + 1));
  for (int row = 0; row < m; ++row) {
    auto& bucket = row_classes[lp_basis_semantic_row_hash(sf, row)];
    ++bucket.total;
    if (static_cast<int>(bucket.rows.size()) < lp_basis_trace_terms()) {
      bucket.rows.push_back(row);
    }
  }
  for (int i = 0; i < m && i < static_cast<int>(basis.size()); ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (col < 0 || col >= n) continue;
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    if (kind == kLpBasisColOriginal || kind == kLpBasisColUnknown) continue;
    const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
    if (row < 0 || row >= m) continue;
    auto it = row_classes.find(lp_basis_semantic_row_hash(sf, row));
    if (it == row_classes.end()) continue;
    if (static_cast<int>(it->second.basic_rows.size()) < lp_basis_trace_terms()) {
      it->second.basic_rows.push_back(i);
      it->second.basic_cols.push_back(col);
    }
  }

  int duplicate_classes = 0;
  int duplicate_rows = 0;
  int duplicate_basic_classes = 0;
  int duplicate_basic_rows = 0;
  for (const auto& entry : row_classes) {
    const DuplicateRows& bucket = entry.second;
    if (bucket.total <= 1) continue;
    ++duplicate_classes;
    duplicate_rows += bucket.total;
    if (!bucket.basic_rows.empty()) {
      ++duplicate_basic_classes;
      duplicate_basic_rows += static_cast<int>(bucket.basic_rows.size());
    }
  }

  fmt::memory_buffer sample;
  int emitted = 0;
  const int max_terms = lp_basis_trace_terms();
  for (int i = 0; i < m && i < static_cast<int>(basis.size()) &&
                  emitted < max_terms;
       ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    if (emitted > 0) fmt::format_to(std::back_inserter(sample), ";");
    if (col < 0 || col >= n) {
      fmt::format_to(std::back_inserter(sample), "{}:{}:invalid", i, col);
      ++emitted;
      continue;
    }
    const unsigned char kind = col_info.kind_for_col[static_cast<std::size_t>(col)];
    const int row = col_info.row_for_col[static_cast<std::size_t>(col)];
    std::uint64_t row_hash = 0;
    if (row >= 0 && kind != kLpBasisColOriginal) {
      row_hash = lp_basis_semantic_row_hash(sf, row);
    }
    fmt::format_to(std::back_inserter(sample),
                   "{}:{}:{}:row{}:rh{:016x}",
                   i, col, lp_basis_col_kind_name(kind), row, row_hash);
    ++emitted;
  }

  fmt::print(stderr,
             "[B&C-LPBASIS-HINT] stage={} call={} match={} m={} n={} "
             "nOrig={} nSlack={} nSurplus={} nArt={} "
             "basic=orig{}:slack{}:surplus{}:art{}:unknown{} "
             "invalid={} dupCols={} nonbasic=lower{}:upper{} "
             "dupLogical=classes{}:rows{}:basicClasses{}:basicRows{} "
             "sample=[{}]\n",
             stage == nullptr ? "unknown" : stage, call_id, hint_match ? 1 : 0,
             m, n, sf.n_original, sf.n_slack, sf.n_surplus, sf.n_artificial,
             basic_orig, basic_slack, basic_surplus, basic_art, basic_unknown,
             invalid, dup_cols, nonbasic_lower, nonbasic_upper,
             duplicate_classes, duplicate_rows, duplicate_basic_classes,
             duplicate_basic_rows, fmt::to_string(sample));
}


void populate_dual_certificate(const StandardFormLP& sf,
                               const std::vector<int>& basis,
                               const std::vector<char>& at_upper,
                               const Eigen::MatrixXd& binv,
                               const std::shared_ptr<BasisOps>& sparse_basis,
                               const Eigen::VectorXd& reduced_costs,
                               SolveResult& result) {
  const int m = sf.A.rows();
  const int n = sf.A.cols();
  const int n_orig = sf.n_original;
  if (m <= 0 || static_cast<int>(basis.size()) != m ||
      static_cast<int>(sf.c_max.size()) != n) {
    return;
  }

  Eigen::VectorXd c_b(m);
  for (int i = 0; i < m; ++i) {
    const int col = basis[static_cast<std::size_t>(i)];
    c_b[i] = (col >= 0 && col < n) ? sf.c_max[col] : 0.0;
  }

  Eigen::VectorXd y_scaled;
  if (sparse_basis) {
    y_scaled = sparse_basis_btran(sparse_basis, c_b);
  } else if (binv.rows() == m && binv.cols() == m) {
    y_scaled.noalias() = binv.transpose() * c_b;
  }
  if (y_scaled.size() != m || !y_scaled.allFinite()) return;

  result.constraint_duals.resize(m);
  const bool have_row_scale = sf.row_scale.size() == m;
  for (int i = 0; i < m; ++i) {
    const double row_scale = have_row_scale ? sf.row_scale[i] : 1.0;
    const double row_sign =
        (i < static_cast<int>(sf.row_sign.size()))
            ? static_cast<double>(sf.row_sign[static_cast<std::size_t>(i)])
            : 1.0;
    // Internal simplex solves max c_max^T z in the signed/scaled row space.
    // The public LP convention is effective minimization, matching HiGHS:
    // row upper active => row_dual <= 0, row lower active => row_dual >= 0.
    result.constraint_duals[i] = -row_sign * row_scale * y_scaled[i];
  }

  result.box_dual_lb = Eigen::VectorXd::Zero(n_orig);
  result.box_dual_ub = Eigen::VectorXd::Zero(n_orig);
  const bool have_col_scale = sf.col_scale.size() >= n_orig;
  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int col : basis) {
    if (col >= 0 && col < n) is_basic[static_cast<std::size_t>(col)] = 1;
  }
  const double dual_tol = 1e-10;
  for (int j = 0; j < n_orig; ++j) {
    double col_dual = 0.0;
    if (reduced_costs.size() > j) {
      const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
      if (col_scale > 0.0 && std::isfinite(col_scale)) {
        col_dual = -reduced_costs[j] / col_scale;
      }
    }
    if (std::abs(col_dual) <= dual_tol || is_basic[static_cast<std::size_t>(j)]) {
      continue;
    }
    if (col_dual > 0.0) {
      result.box_dual_lb[j] = col_dual;
    } else {
      result.box_dual_ub[j] = -col_dual;
    }
  }

  (void)at_upper;
}

#ifdef MIPSOLVERS_HAVE_HIGHS_LIB
const char* highs_sf_model_status_label(HighsModelStatus status) {
  switch (status) {
    case HighsModelStatus::kOptimal:
      return "Optimal";
    case HighsModelStatus::kInfeasible:
      return "Infeasible";
    case HighsModelStatus::kUnbounded:
      return "Unbounded";
    case HighsModelStatus::kUnboundedOrInfeasible:
      return "UnboundedOrInfeasible";
    case HighsModelStatus::kObjectiveBound:
      return "Objective cutoff";
    case HighsModelStatus::kIterationLimit:
      return "Iteration limit";
    case HighsModelStatus::kTimeLimit:
      return "Time limit";
    default:
      return "Other";
  }
}

bool native_sf_basis_to_highs(const StandardFormLP& sf,
                              const SimplexBasis* basis_hint,
                              HighsBasis& hbasis) {
  if (basis_hint == nullptr) return false;
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  if (basis_hint->rows != m || basis_hint->cols != n ||
      static_cast<int>(basis_hint->index_count()) != m ||
      static_cast<int>(basis_hint->at_upper.size()) < n) {
    return false;
  }

  hbasis.valid = false;
  hbasis.alien = true;
  hbasis.useful = true;
  hbasis.col_status.assign(static_cast<std::size_t>(n),
                           HighsBasisStatus::kLower);
  hbasis.row_status.assign(static_cast<std::size_t>(m),
                           HighsBasisStatus::kBasic);

  for (int j = 0; j < n; ++j) {
    const bool fixed =
        sf.var_ub.size() > j && std::isfinite(sf.var_ub[j]) &&
        std::abs(sf.var_ub[j]) <= 1e-12;
    if (fixed) {
      hbasis.col_status[static_cast<std::size_t>(j)] =
          HighsBasisStatus::kLower;
    } else if (basis_hint->at_upper[static_cast<std::size_t>(j)] != 0) {
      hbasis.col_status[static_cast<std::size_t>(j)] =
          HighsBasisStatus::kUpper;
    }
  }
  for (int row = 0; row < m; ++row) {
    const int col = basis_hint->basis_indices()[static_cast<std::size_t>(row)];
    if (col < 0 || col >= n) return false;
    hbasis.col_status[static_cast<std::size_t>(col)] =
        HighsBasisStatus::kBasic;
    hbasis.row_status[static_cast<std::size_t>(row)] =
        HighsBasisStatus::kLower;
  }
  return true;
}

int native_sf_col_from_highs_basic(const StandardFormLP& sf,
                                   HighsInt highs_basic) {
  const int n = static_cast<int>(sf.A.cols());
  if (highs_basic >= 0) {
    const int col = static_cast<int>(highs_basic);
    return (col >= 0 && col < n) ? col : -1;
  }

  // HiGHS reports a basic row/logical variable as -(row + 1).  Native's
  // standard-form basis stores the corresponding explicit logical column.
  const int row = static_cast<int>(-highs_basic - 1);
  if (row < 0 || row >= static_cast<int>(sf.A.rows())) return -1;
  auto row_col = [&](const std::vector<int>& map) -> int {
    if (row >= static_cast<int>(map.size())) return -1;
    const int col = map[static_cast<std::size_t>(row)];
    return (col >= 0 && col < n) ? col : -1;
  };
  int col = row_col(sf.row_to_slack_col);
  if (col >= 0) return col;
  col = row_col(sf.row_to_surplus_col);
  if (col >= 0) return col;
  col = row_col(sf.row_to_artificial_col);
  if (col >= 0) return col;
  return -1;
}

bool pass_standard_form_to_highs(Highs& highs, const StandardFormLP& sf) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n) is_artificial[static_cast<std::size_t>(art)] = 1;
  }
  std::vector<double> col_cost(static_cast<std::size_t>(n), 0.0);
  std::vector<double> col_lower(static_cast<std::size_t>(n), 0.0);
  std::vector<double> col_upper(static_cast<std::size_t>(n),
                                std::numeric_limits<double>::infinity());
  for (int j = 0; j < n; ++j) {
    col_cost[static_cast<std::size_t>(j)] = sf.c_max[j];
    if (is_artificial[static_cast<std::size_t>(j)]) {
      // Artificial columns are a native Phase-I device, not true LP
      // variables.  Fix them at zero when delegating the real SF LP to HiGHS.
      col_upper[static_cast<std::size_t>(j)] = 0.0;
    } else if (sf.var_ub.size() > j) {
      col_upper[static_cast<std::size_t>(j)] = sf.var_ub[j];
    }
  }
  std::vector<double> row_lower(static_cast<std::size_t>(m), 0.0);
  std::vector<double> row_upper(static_cast<std::size_t>(m), 0.0);
  for (int i = 0; i < m; ++i) {
    row_lower[static_cast<std::size_t>(i)] = sf.b[i];
    row_upper[static_cast<std::size_t>(i)] = sf.b[i];
  }

  std::vector<HighsInt> start(static_cast<std::size_t>(n + 1), 0);
  std::vector<HighsInt> index;
  std::vector<double> value;
  index.reserve(static_cast<std::size_t>(sf.A.nonZeros()));
  value.reserve(index.capacity());
  for (int j = 0; j < n; ++j) {
    start[static_cast<std::size_t>(j)] =
        static_cast<HighsInt>(index.size());
    for (StandardColumnMatrix::InnerIterator it(sf.A, j); it; ++it) {
      if (it.value() == 0.0) continue;
      index.push_back(static_cast<HighsInt>(it.row()));
      value.push_back(it.value());
    }
  }
  start[static_cast<std::size_t>(n)] =
      static_cast<HighsInt>(index.size());

  const HighsStatus st = highs.passModel(
      static_cast<HighsInt>(n), static_cast<HighsInt>(m),
      static_cast<HighsInt>(index.size()),
      static_cast<HighsInt>(MatrixFormat::kColwise),
      static_cast<HighsInt>(ObjSense::kMaximize), 0.0, col_cost.data(),
      col_lower.data(), col_upper.data(), row_lower.data(), row_upper.data(),
      start.data(), index.data(), value.data(), nullptr);
  return st == HighsStatus::kOk;
}

bool audit_vendored_highs_sf_result(const StandardFormLP& sf,
                                    const SimplexOptions& opt,
                                    const SimplexResult& out,
                                    double& primal_residual,
                                    double& bound_violation,
                                    double& artificial_activity,
                                    double& dual_violation,
                                    double& objective_gap,
                                    double highs_dual_violation) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  primal_residual = std::numeric_limits<double>::infinity();
  bound_violation = std::numeric_limits<double>::infinity();
  artificial_activity = std::numeric_limits<double>::infinity();
  dual_violation = std::numeric_limits<double>::infinity();
  objective_gap = std::numeric_limits<double>::infinity();
  if (out.x_std.size() != n || out.reduced_costs.size() != n ||
      static_cast<int>(out.basis.index_count()) != m ||
      static_cast<int>(out.basis.at_upper.size()) < n) {
    return false;
  }
  if (!out.x_std.allFinite() || !out.reduced_costs.allFinite()) return false;
  if (m > 0) {
    primal_residual = (sf.A * out.x_std - sf.b).cwiseAbs().maxCoeff();
  } else {
    primal_residual = 0.0;
  }
  bound_violation = 0.0;
  for (int j = 0; j < n; ++j) {
    bound_violation = std::max(bound_violation, std::max(0.0, -out.x_std[j]));
    if (sf.var_ub.size() > j && std::isfinite(sf.var_ub[j])) {
      bound_violation =
          std::max(bound_violation, std::max(0.0, out.x_std[j] - sf.var_ub[j]));
    }
  }
  artificial_activity = 0.0;
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n) {
      artificial_activity =
          std::max(artificial_activity, std::abs(out.x_std[art]));
    }
  }
  (void)opt;
  const double primal_tol = kHighsDefaultKktTolerance;
  dual_violation = 0.0;
  const std::vector<char> is_basic =
      basis_basic_mask(n, out.basis.basis_indices());
  for (int j = 0; j < n; ++j) {
    if (is_basic[static_cast<std::size_t>(j)]) continue;
    const bool fixed =
        sf.var_ub.size() > j && std::isfinite(sf.var_ub[j]) &&
        std::abs(sf.var_ub[j]) <= primal_tol;
    if (fixed) continue;
    const bool at_up = out.basis.at_upper[static_cast<std::size_t>(j)] != 0;
    if (at_up) {
      dual_violation = std::max(dual_violation, std::max(0.0, -out.reduced_costs[j]));
    } else {
      dual_violation = std::max(dual_violation, std::max(0.0, out.reduced_costs[j]));
    }
  }
  const double computed_max_obj = sf.c_max.dot(out.x_std);
  objective_gap = std::abs(computed_max_obj - out.max_objective);
  const double dual_tol = kHighsDefaultKktTolerance;
  if (std::isfinite(highs_dual_violation) &&
      highs_dual_violation <= dual_tol) {
    dual_violation = std::min(dual_violation, highs_dual_violation);
  }
  const double obj_tol =
      std::max(1e-6, 1e-8 * std::max(1.0, std::abs(out.max_objective)));
  return primal_residual <= primal_tol && bound_violation <= primal_tol &&
         artificial_activity <= primal_tol && dual_violation <= dual_tol &&
         objective_gap <= obj_tol;
}

int inject_basic_degenerate_col_duals_from_highs(
    const StandardFormLP& sf,
    Highs& highs,
    const HighsSolution& sol,
    const std::vector<int>& basis_indices,
    Eigen::VectorXd& reduced_costs,
    double feastol,
    double epsilon) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  const int n_orig = sf.n_original;
  if (m <= 0 || n <= 0 || n_orig <= 0 ||
      static_cast<int>(basis_indices.size()) != m ||
      reduced_costs.size() != n ||
      static_cast<int>(sol.col_value.size()) < n ||
      static_cast<int>(sol.col_dual.size()) < n) {
    return 0;
  }

  std::vector<int> basic_row_for_col(static_cast<std::size_t>(n), -1);
  for (int row = 0; row < m; ++row) {
    const int col = basis_indices[static_cast<std::size_t>(row)];
    if (col >= 0 && col < n) basic_row_for_col[static_cast<std::size_t>(col)] = row;
  }

  const bool have_col_scale = sf.col_scale.size() == n;
  const bool have_ub = sf.var_ub.size() == n;
  const double dual_accept_tol = std::max(10.0 * feastol, epsilon);
  std::vector<double> row_ap(static_cast<std::size_t>(n), 0.0);
  int injected = 0;

  for (int col = 0; col < n_orig; ++col) {
    if (col >= static_cast<int>(sf.original_types.size())) break;
    const VarType type = sf.original_types[static_cast<std::size_t>(col)];
    if (type != VarType::Integer && type != VarType::Binary) continue;

    const int basis_row = basic_row_for_col[static_cast<std::size_t>(col)];
    if (basis_row < 0) continue;

    const double lb = 0.0;
    const double ub = have_ub ? sf.var_ub[col] : kInf;
    if (std::isfinite(ub) && ub - lb < feastol) continue;

    const double value = sol.col_value[static_cast<std::size_t>(col)];
    double sign = 0.0;
    if (!std::isfinite(ub) || value - lb < ub - value) {
      if (value > lb + feastol) continue;
      sign = 1.0;
    } else {
      if (value < ub - feastol) continue;
      sign = -1.0;
    }

    std::fill(row_ap.begin(), row_ap.end(), 0.0);
    if (highs.getReducedRow(static_cast<HighsInt>(basis_row),
                            row_ap.data()) != HighsStatus::kOk) {
      continue;
    }

    double degenerate_col_dual = kInf;
    for (int j = 0; j < n; ++j) {
      if (j == col) continue;
      const double other_ub = have_ub ? sf.var_ub[j] : kInf;
      if (std::isfinite(other_ub) && other_ub <= feastol) continue;
      const double val = sign * row_ap[static_cast<std::size_t>(j)];
      if (val > epsilon) {
        if (sol.col_value[static_cast<std::size_t>(j)] > feastol) {
          const double ratio =
              -sol.col_dual[static_cast<std::size_t>(j)] / val;
          if (std::isfinite(ratio)) {
            degenerate_col_dual = std::min(degenerate_col_dual, ratio);
          }
        }
      } else if (val < -epsilon) {
        const bool has_room_to_upper =
            !std::isfinite(other_ub) ||
            other_ub - sol.col_value[static_cast<std::size_t>(j)] > feastol;
        if (has_room_to_upper) {
          const double ratio =
              -sol.col_dual[static_cast<std::size_t>(j)] / val;
          if (std::isfinite(ratio)) {
            degenerate_col_dual = std::min(degenerate_col_dual, ratio);
          }
        }
      }
    }

    if (!std::isfinite(degenerate_col_dual) ||
        degenerate_col_dual <= dual_accept_tol) {
      continue;
    }

    const double effective_col_dual = sign * degenerate_col_dual;
    const double col_scale = have_col_scale ? sf.col_scale[col] : 1.0;
    if (!std::isfinite(effective_col_dual) ||
        !std::isfinite(col_scale) || std::abs(col_scale) <= 1e-18) {
      continue;
    }
    reduced_costs[col] = -effective_col_dual * col_scale;
    ++injected;
  }

  return injected;
}

class VendoredHighsBasis : public BasisOps {
 public:
  VendoredHighsBasis(std::shared_ptr<Highs> highs,
                     int rows,
                     int cols,
                     std::uint64_t structure_id)
      : highs_(std::move(highs)),
        m_(rows),
        n_(cols),
        structure_id_(structure_id) {
    if (highs_) capture_snapshot(owner_snapshot_);
  }

  BasisOpsKind kind() const override { return BasisOpsKind::VendoredHighs; }

  Eigen::VectorXd ftran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(m_,
                                       std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  Eigen::VectorXd btran(const Eigen::VectorXd& rhs) const override {
    Eigen::VectorXd out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || rhs.size() != m_) return out;
    std::vector<double> h_rhs(static_cast<std::size_t>(m_), 0.0);
    std::vector<double> h_out(static_cast<std::size_t>(m_), 0.0);
    for (int i = 0; i < m_; ++i) h_rhs[static_cast<std::size_t>(i)] = rhs[i];
    if (highs_->getBasisTransposeSolve(h_rhs.data(), h_out.data()) !=
        HighsStatus::kOk) {
      return Eigen::VectorXd::Constant(m_,
                                       std::numeric_limits<double>::quiet_NaN());
    }
    for (int i = 0; i < m_; ++i) out[i] = h_out[static_cast<std::size_t>(i)];
    return out;
  }

  bool basis_inverse_row(int row, Eigen::VectorXd& out) const override {
    out = Eigen::VectorXd::Zero(m_);
    if (!highs_ || row < 0 || row >= m_) return false;
    std::vector<double> row_vec(static_cast<std::size_t>(m_), 0.0);
    HighsInt row_num_nz = 0;
    std::vector<HighsInt> row_indices(static_cast<std::size_t>(m_), 0);
    const HighsStatus st =
        highs_->getBasisInverseRow(static_cast<HighsInt>(row), row_vec.data(),
                                   &row_num_nz, row_indices.data());
    if (st != HighsStatus::kOk) return false;
    if (row_num_nz < 0 || row_num_nz > m_) return false;
    for (HighsInt k = 0; k < row_num_nz; ++k) {
      const int r = static_cast<int>(row_indices[static_cast<std::size_t>(k)]);
      if (r < 0 || r >= m_) return false;
      out[r] = row_vec[static_cast<std::size_t>(r)];
    }
    return out.allFinite();
  }

  bool tableau_row(int row, Eigen::RowVectorXd& out) const override {
    if (!highs_ || row < 0 || row >= m_ || n_ <= 0) return false;
    std::vector<double> h_row(static_cast<std::size_t>(n_), 0.0);
    if (highs_->getReducedRow(static_cast<HighsInt>(row), h_row.data()) ==
        HighsStatus::kOk) {
      out.resize(n_);
      for (int j = 0; j < n_; ++j) out[j] = h_row[static_cast<std::size_t>(j)];
      return out.allFinite();
    }
    return false;
  }

  SparseFactorTelemetry factor_telemetry() const override {
    SparseFactorTelemetry t;
    t.ft_valid = true;
    return t;
  }

  void rebind_A(const StandardColumnMatrix&) override {}

  bool bound_to_A(const StandardColumnMatrix& A) const override {
    return A.rows() == m_ && A.cols() == n_;
  }

  std::shared_ptr<Highs> highs_handle() const override { return highs_; }

  bool import_optimal_result(const std::shared_ptr<Highs>& highs,
                             const StandardFormLP& sf,
                             const SimplexOptions& opt,
                             bool solved_from_hint,
                             const char* context,
                             SimplexResult& out) {
    if (!highs) return false;
    const int m = static_cast<int>(sf.A.rows());
    const int n = static_cast<int>(sf.A.cols());
    if (m <= 0 || n <= 0) return false;
    const HighsModelStatus model_status = highs->getModelStatus();
    if (model_status != HighsModelStatus::kOptimal) {
      out.result.stats.solver_name = "VendoredHighsLpKernel";
      out.result.stats.success = false;
      out.result.stats.status =
          std::string("HiGHS ") + highs_sf_model_status_label(model_status);
      return false;
    }

    const HighsSolution& sol = highs->getSolution();
    const HighsBasis& basis = highs->getBasis();
    if (!basis.valid || static_cast<int>(basis.col_status.size()) < n ||
        static_cast<int>(basis.row_status.size()) < m ||
        static_cast<int>(sol.col_value.size()) < n) {
      return false;
    }

    out = SimplexResult{};
    out.x_std = Eigen::VectorXd::Zero(n);
    for (int j = 0; j < n; ++j) {
      out.x_std[j] = sol.col_value[static_cast<std::size_t>(j)];
    }
    out.result.x = extract_solution(sf, out.x_std);
    out.x_basic = Eigen::VectorXd::Zero(m);
    out.basis.rows = m;
    out.basis.cols = n;
    out.basis.indices.assign(static_cast<std::size_t>(m), -1);
    out.basis.at_upper.assign(static_cast<std::size_t>(n), 0);
    out.basis.sf_n_slack = sf.n_slack;
    out.basis.sf_n_surplus = sf.n_surplus;
    out.basis.sf_n_artificial = sf.n_artificial;

    for (int j = 0; j < n; ++j) {
      out.basis.at_upper[static_cast<std::size_t>(j)] =
          basis.col_status[static_cast<std::size_t>(j)] ==
                  HighsBasisStatus::kUpper
              ? 1
              : 0;
    }

    std::vector<HighsInt> basic(static_cast<std::size_t>(m), 0);
    if (highs->getBasicVariables(basic.data()) != HighsStatus::kOk) {
      return false;
    }
    std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
    for (int art : sf.row_to_artificial_col) {
      if (art >= 0 && art < n) {
        is_artificial[static_cast<std::size_t>(art)] = 1;
      }
    }
    for (int row = 0; row < m; ++row) {
      const int col = native_sf_col_from_highs_basic(
          sf, basic[static_cast<std::size_t>(row)]);
      if (col < 0 || col >= n) return false;
      if (is_artificial[static_cast<std::size_t>(col)] &&
          std::abs(out.x_std[col]) >
              std::max(1e-7, opt.feasibility_tol * 20.0)) {
        return false;
      }
      out.basis.indices[static_cast<std::size_t>(row)] = col;
      out.x_basic[row] = out.x_std[col];
    }

    out.reduced_costs = Eigen::VectorXd::Zero(n);
    const bool have_col_dual = static_cast<int>(sol.col_dual.size()) >= n;
    if (!have_col_dual) return false;
    const bool have_col_scale = sf.col_scale.size() == n;
    for (int j = 0; j < n; ++j) {
      const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
      out.reduced_costs[j] =
          -sol.col_dual[static_cast<std::size_t>(j)] * col_scale;
    }

    const HighsInfo& info = highs->getInfo();
    out.basis_inverse.resize(0, 0);
    out.max_objective = info.objective_function_value;
    out.result.stats.solver_name = "VendoredHighsLpKernel";
    out.result.stats.iterations =
        static_cast<int>(info.simplex_iteration_count);
    out.result.stats.objective = sf.objective_const - out.max_objective;
    out.result.stats.status = "Optimal";
    out.result.stats.success = true;
    out.result.stats.primal_feas = 0.0;
    out.result.stats.residual_inf = 0.0;
    out.solved_from_hint = solved_from_hint;
    out.dual_reoptimized = solved_from_hint;
    out.exact_optimal = true;
    out.basis.cached_reduced_costs =
        std::make_shared<const Eigen::VectorXd>(out.reduced_costs);
    if (sf.col_scale.size() == n) {
      out.basis.cached_col_scale =
          std::make_shared<const Eigen::VectorXd>(sf.col_scale);
    }
    out.basis.cached_sparse_basis =
        std::make_shared<VendoredHighsBasis>(
            highs, m, n, sf.structure_id);
    out.basis.persist_eta_count = 0;

    double primal_residual = 0.0;
    double bound_violation = 0.0;
    double artificial_activity = 0.0;
    double dual_violation = 0.0;
    double objective_gap = 0.0;
    const double highs_dual_violation =
        info.num_dual_infeasibilities <= 0
            ? 0.0
            : (std::isfinite(info.max_dual_infeasibility)
                   ? info.max_dual_infeasibility
                   : std::numeric_limits<double>::infinity());
    if (!audit_vendored_highs_sf_result(sf, opt, out, primal_residual,
                                        bound_violation, artificial_activity,
                                        dual_violation, objective_gap,
                                        highs_dual_violation)) {
      if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
        fmt::print(stderr,
                   "[SF_RELAX] VendoredHiGHS SF audit rejected: ctx={} "
                   "primal={:.3e} bound={:.3e} art={:.3e} dual={:.3e} "
                   "obj_gap={:.3e} m={} n={} hint={}\n",
                   context != nullptr ? context : "solve", primal_residual,
                   bound_violation, artificial_activity, dual_violation,
                   objective_gap, m, n, solved_from_hint ? 1 : 0);
      }
      return false;
    }
    out.result.stats.primal_feas = std::max(primal_residual, bound_violation);
    out.result.stats.residual_inf = primal_residual;
    out.result.stats.dual_feas = dual_violation;
    populate_dual_certificate(sf, out.basis.indices, out.basis.at_upper,
                              out.basis_inverse, out.basis.cached_sparse_basis,
                              out.reduced_costs, out.result);
    return true;
  }

  bool resolve_same_structure(const StandardFormLP& sf,
                              const SimplexBasis* basis_hint,
                              const SimplexOptions& opt,
                              SimplexResult& out) override {
    if (!highs_ || basis_hint == nullptr || sf.structure_id == 0 ||
        sf.structure_id != structure_id_ ||
        sf.A.rows() != m_ || sf.A.cols() != n_ ||
        !owner_snapshot_.basis.valid) {
      return false;
    }
    HighsBasis requested_basis;
    if (!native_sf_basis_to_highs(sf, basis_hint, requested_basis)) {
      return false;
    }

    const int m = m_;
    const int n = n_;
    std::vector<double> target_col_lower(static_cast<std::size_t>(n), 0.0);
    std::vector<double> target_col_upper(static_cast<std::size_t>(n), kInf);
    std::vector<char> artificial(static_cast<std::size_t>(n), 0);
    for (int col : sf.row_to_artificial_col) {
      if (col >= 0 && col < n) artificial[static_cast<std::size_t>(col)] = 1;
    }
    for (int col = 0; col < n; ++col) {
      if (artificial[static_cast<std::size_t>(col)]) {
        target_col_upper[static_cast<std::size_t>(col)] = 0.0;
      } else if (sf.var_ub.size() > col) {
        target_col_upper[static_cast<std::size_t>(col)] = sf.var_ub[col];
      }
    }
    std::vector<double> target_row(static_cast<std::size_t>(m), 0.0);
    for (int row = 0; row < m; ++row) {
      target_row[static_cast<std::size_t>(row)] = sf.b[row];
    }

    const HighsLp& active_lp = highs_->getLp();
    if (active_lp.num_row_ != m || active_lp.num_col_ != n ||
        active_lp.col_cost_.size() != static_cast<std::size_t>(n)) {
      return false;
    }
    for (int col = 0; col < n; ++col) {
      if (active_lp.col_cost_[static_cast<std::size_t>(col)] != sf.c_max[col]) {
        return false;
      }
    }

    if (!same_basis(highs_->getBasis(), requested_basis) &&
        highs_->setBasis(requested_basis,
                         "MIPSOLVERS persistent node LP basis") ==
            HighsStatus::kError) {
      return false;
    }

    std::vector<HighsInt> changed_cols;
    std::vector<double> changed_col_lower;
    std::vector<double> changed_col_upper;
    changed_cols.reserve(static_cast<std::size_t>(n));
    for (int col = 0; col < n; ++col) {
      if (active_lp.col_lower_[static_cast<std::size_t>(col)] ==
              target_col_lower[static_cast<std::size_t>(col)] &&
          active_lp.col_upper_[static_cast<std::size_t>(col)] ==
              target_col_upper[static_cast<std::size_t>(col)]) {
        continue;
      }
      changed_cols.push_back(col);
      changed_col_lower.push_back(target_col_lower[static_cast<std::size_t>(col)]);
      changed_col_upper.push_back(target_col_upper[static_cast<std::size_t>(col)]);
    }
    std::vector<HighsInt> changed_rows;
    std::vector<double> changed_row_lower;
    std::vector<double> changed_row_upper;
    changed_rows.reserve(static_cast<std::size_t>(m));
    for (int row = 0; row < m; ++row) {
      if (active_lp.row_lower_[static_cast<std::size_t>(row)] ==
              target_row[static_cast<std::size_t>(row)] &&
          active_lp.row_upper_[static_cast<std::size_t>(row)] ==
              target_row[static_cast<std::size_t>(row)]) {
        continue;
      }
      changed_rows.push_back(row);
      changed_row_lower.push_back(target_row[static_cast<std::size_t>(row)]);
      changed_row_upper.push_back(target_row[static_cast<std::size_t>(row)]);
    }

    bool ok = changed_cols.empty() ||
              highs_->changeColsBounds(
                  static_cast<HighsInt>(changed_cols.size()),
                  changed_cols.data(), changed_col_lower.data(),
                  changed_col_upper.data()) != HighsStatus::kError;
    if (ok && !changed_rows.empty()) {
      ok = highs_->changeRowsBounds(
               static_cast<HighsInt>(changed_rows.size()), changed_rows.data(),
               changed_row_lower.data(), changed_row_upper.data()) !=
           HighsStatus::kError;
    }
    if (ok) {
      ok = highs_->setOptionValue("simplex_iteration_limit",
                                  std::max(1, opt.max_iter)) !=
           HighsStatus::kError;
    }
    if (ok && opt.time_limit_sec > 0.0 &&
        std::isfinite(opt.time_limit_sec)) {
      highs_->setOptionValue("time_limit", std::max(0.001, opt.time_limit_sec));
    } else if (ok) {
      highs_->setOptionValue("time_limit", kHighsInf);
    }
    const auto t0 = std::chrono::steady_clock::now();
    if (ok) ok = highs_->run() != HighsStatus::kError;
    const bool run_completed = ok;
    const HighsModelStatus model_status =
        run_completed ? highs_->getModelStatus() : HighsModelStatus::kNotset;
    if (ok && model_status == HighsModelStatus::kOptimal) {
      ok = import_optimal_result(highs_, sf, opt, true,
                                 "persistent_bounds", out);
    } else {
      ok = false;
    }
    if (ok) {
      out.result.stats.runtime_sec = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0).count();
      out.result.stats.solver_name = "VendoredHighsPersistentLpKernel";
      return true;
    }
    if (run_completed) {
      if (model_status == HighsModelStatus::kIterationLimit ||
          model_status == HighsModelStatus::kTimeLimit ||
          model_status == HighsModelStatus::kObjectiveBound) {
        const HighsInfo& info = highs_->getInfo();
        out.result.stats.success = false;
        out.result.stats.solver_name = "VendoredHighsPersistentLpKernel";
        out.result.stats.iterations =
            static_cast<int>(info.simplex_iteration_count);
        out.result.stats.runtime_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        out.result.stats.status =
            std::string("HiGHS ") + highs_sf_model_status_label(model_status);
        (void)restore_snapshot(owner_snapshot_);
        return true;
      }
    }
    (void)restore_snapshot(owner_snapshot_);
    return false;
  }

  bool delete_rows_cols_and_resolve(
      const StandardFormLP& compact_sf,
      const SimplexBasis* compact_basis_hint,
      const std::vector<int>& delete_rows,
      const std::vector<int>& delete_cols,
      const SimplexOptions& opt,
      SimplexResult& out) override {
    if (!highs_ || delete_rows.empty() ||
        compact_basis_hint == nullptr ||
        compact_basis_hint->rows != static_cast<int>(compact_sf.A.rows()) ||
        compact_basis_hint->cols != static_cast<int>(compact_sf.A.cols()) ||
        static_cast<int>(compact_basis_hint->index_count()) !=
            static_cast<int>(compact_sf.A.rows())) {
      return false;
    }
    const int old_m = m_;
    const int old_n = n_;
    auto highs = highs_;
    if (!delete_cols.empty()) {
      std::vector<HighsInt> cols;
      cols.reserve(delete_cols.size());
      for (int col : delete_cols) {
        if (col < 0 || col >= old_n) return false;
        cols.push_back(static_cast<HighsInt>(col));
      }
      if (highs->deleteCols(static_cast<HighsInt>(cols.size()),
                            cols.data()) != HighsStatus::kOk) {
        return false;
      }
    }
    {
      std::vector<HighsInt> rows;
      rows.reserve(delete_rows.size());
      for (int row : delete_rows) {
        if (row < 0 || row >= old_m) return false;
        rows.push_back(static_cast<HighsInt>(row));
      }
      if (highs->deleteRows(static_cast<HighsInt>(rows.size()),
                            rows.data()) != HighsStatus::kOk) {
        return false;
      }
    }
    if (highs->getNumRow() != compact_sf.A.rows() ||
        highs->getNumCol() != compact_sf.A.cols()) {
      return false;
    }
    HighsBasis hbasis;
    if (!native_sf_basis_to_highs(compact_sf, compact_basis_hint, hbasis)) {
      return false;
    }
    if (highs->setBasis(hbasis,
                        "MIPSOLVERS vendored xpool aging removeCuts") !=
        HighsStatus::kOk) {
      return false;
    }
    const HighsStatus run_status = highs->run();
    if (run_status != HighsStatus::kOk ||
        highs->getModelStatus() != HighsModelStatus::kOptimal) {
      out.result.stats.solver_name = "VendoredHighsLpKernel";
      out.result.stats.success = false;
      out.result.stats.status =
          std::string("HiGHS ") +
          highs_sf_model_status_label(highs->getModelStatus());
      return false;
    }
    return import_optimal_result(highs, compact_sf, opt, true,
                                 "delete_rows_cols_and_resolve", out);
  }

 private:
  struct Snapshot {
    std::vector<double> col_lower;
    std::vector<double> col_upper;
    std::vector<double> row_lower;
    std::vector<double> row_upper;
    HighsBasis basis;
  };

  bool capture_snapshot(Snapshot& snapshot) const {
    if (!highs_ || highs_->getNumRow() != m_ || highs_->getNumCol() != n_) {
      return false;
    }
    const HighsLp& lp = highs_->getLp();
    snapshot.col_lower = lp.col_lower_;
    snapshot.col_upper = lp.col_upper_;
    snapshot.row_lower = lp.row_lower_;
    snapshot.row_upper = lp.row_upper_;
    snapshot.basis = highs_->getBasis();
    return snapshot.basis.valid;
  }

  bool restore_snapshot(const Snapshot& snapshot) {
    if (!highs_ || highs_->getNumRow() != m_ || highs_->getNumCol() != n_) {
      return false;
    }
    bool ok = n_ == 0 ||
              highs_->changeColsBounds(0, n_ - 1, snapshot.col_lower.data(),
                                        snapshot.col_upper.data()) !=
                  HighsStatus::kError;
    if (ok && m_ > 0) {
      ok = highs_->changeRowsBounds(0, m_ - 1, snapshot.row_lower.data(),
                                    snapshot.row_upper.data()) !=
           HighsStatus::kError;
    }
    if (ok) {
      ok = highs_->setBasis(snapshot.basis,
                            "MIPSOLVERS persistent LP rollback") !=
           HighsStatus::kError;
    }
    if (ok) highs_->setOptionValue("time_limit", kHighsInf);
    return ok && highs_->run() != HighsStatus::kError;
  }

  static bool same_basis(const HighsBasis& lhs, const HighsBasis& rhs) {
    return lhs.valid && lhs.col_status == rhs.col_status &&
           lhs.row_status == rhs.row_status;
  }

  std::shared_ptr<Highs> highs_;
  int m_{0};
  int n_{0};
  std::uint64_t structure_id_{0};
  Snapshot owner_snapshot_;
};

bool solve_standard_form_with_vendored_highs(const StandardFormLP& sf,
                                             const SimplexOptions& opt,
                                             const SimplexBasis* basis_hint,
                                             SimplexResult& out) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  if (m <= 0 || n <= 0) return false;

  if (opt.allow_persistent_lp_state && basis_hint &&
      basis_hint->cached_sparse_basis &&
      basis_hint->cached_sparse_basis->resolve_same_structure(
          sf, basis_hint, opt, out)) {
    return true;
  }

  const auto t0 = std::chrono::steady_clock::now();
  const bool trace_sf = std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr;
  auto highs = std::make_shared<Highs>();
  highs->setOptionValue("output_flag", false);
  highs->setOptionValue("log_to_console", false);
  highs->setOptionValue("threads", 1);
  highs->setOptionValue("presolve", "off");
  highs->setOptionValue("solver", "simplex");
  highs->setOptionValue("simplex_strategy", 1);
  highs->setOptionValue("simplex_scale_strategy", 2);
  highs->setOptionValue("simplex_initial_condition_check", false);
  highs->setOptionValue("simplex_iteration_limit", std::max(1, opt.max_iter));
  if (opt.time_limit_sec > 0.0 && std::isfinite(opt.time_limit_sec)) {
    highs->setOptionValue("time_limit", std::max(0.001, opt.time_limit_sec));
  }
  highs->setOptionValue("kkt_tolerance", kHighsDefaultKktTolerance);
  highs->setOptionValue("primal_feasibility_tolerance",
                       kHighsDefaultKktTolerance);
  highs->setOptionValue("dual_feasibility_tolerance",
                       kHighsDefaultKktTolerance);
  highs->setOptionValue("mip_feasibility_tolerance",
                       kHighsDefaultMipTolerance);
  auto reject = [&](const char* reason) {
    out.result.stats.solver_name = "VendoredHighsLpKernel";
    out.result.stats.success = false;
    out.result.stats.status = std::string("VendoredHiGHS SF reject: ") +
                              (reason != nullptr ? reason : "unknown");
    out.result.stats.runtime_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    if (trace_sf) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF rejected: reason={} m={} n={} "
                 "hint={} time={:.3f}ms\n",
                 reason != nullptr ? reason : "unknown", m, n,
                 basis_hint ? 1 : 0,
                 out.result.stats.runtime_sec * 1000.0);
    }
    return false;
  };

  if (!pass_standard_form_to_highs(*highs, sf)) {
    return reject("pass_model");
  }
  HighsBasis hbasis;
  if (native_sf_basis_to_highs(sf, basis_hint, hbasis)) {
    (void)highs->setBasis(hbasis, "MIPSOLVERS native SF basis hint");
  }

  const HighsStatus run_status = highs->run();
  const HighsModelStatus model_status = highs->getModelStatus();
  const HighsInfo& info = highs->getInfo();

  out.result.stats.solver_name = "VendoredHighsLpKernel";
  out.result.stats.iterations =
      static_cast<int>(info.simplex_iteration_count);
  out.result.stats.runtime_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
          .count();
  out.result.stats.status =
      std::string("HiGHS ") + highs_sf_model_status_label(model_status);
  if (model_status == HighsModelStatus::kInfeasible) {
    out.result.stats.status = "LP infeasible";
    out.result.stats.success = false;
    out.result.stats.objective = kInf;
    out.result.stats.has_farkas_certificate = false;
    bool has_dual_ray = false;
    std::vector<double> dual_ray(static_cast<std::size_t>(m), 0.0);
    if (highs->getDualRay(has_dual_ray, dual_ray.data()) ==
            HighsStatus::kOk &&
        has_dual_ray) {
      // A Farkas multiplier is sign-indeterminate. Normalize it to the
      // public lower-separation convention:
      //   min_z (A' y)' z - y' b > 0.
      // Artificial columns are fixed to zero in the original Phase-II domain
      // and therefore do not participate in the support interval.
      Eigen::Map<Eigen::VectorXd> ray_map(dual_ray.data(), m);
      Eigen::VectorXd ray_column = sf.A.transpose() * ray_map;
      std::vector<char> artificial(static_cast<std::size_t>(n), 0);
      for (int col : sf.row_to_artificial_col) {
        if (col >= 0 && col < n) artificial[static_cast<std::size_t>(col)] = 1;
      }
      double attainable_lower = 0.0;
      double attainable_upper = 0.0;
      for (int col = 0; col < n; ++col) {
        if (artificial[static_cast<std::size_t>(col)]) continue;
        const double coefficient = ray_column[col];
        const double upper =
            col < sf.var_ub.size() ? sf.var_ub[col] : kInf;
        if (coefficient >= 0.0) {
          if (std::isfinite(upper)) {
            attainable_upper += coefficient * upper;
          } else if (coefficient > 0.0) {
            attainable_upper = kInf;
          }
        } else if (std::isfinite(upper)) {
          attainable_lower += coefficient * upper;
        } else {
          attainable_lower = -kInf;
        }
      }
      const double ray_rhs = ray_map.dot(sf.b);
      const double below = attainable_lower - ray_rhs;
      const double above = ray_rhs - attainable_upper;
      if (above > below) ray_map = -ray_map;

      int eq_count = 0;
      int ineq_count = 0;
      for (int row = 0; row < m; ++row) {
        if (sf.row_to_artificial_col[row] >= 0 &&
            sf.row_to_surplus_col[row] < 0) {
          ++eq_count;
        } else {
          ++ineq_count;
        }
      }
      out.result.stats.farkas_ray =
          Eigen::VectorXd::Zero(std::max(0, ineq_count));
      out.result.stats.farkas_ray_eq =
          Eigen::VectorXd::Zero(std::max(0, eq_count));
      int ineq_pos = 0;
      int eq_pos = 0;
      for (int row = 0; row < m; ++row) {
        const double value =
            (row < static_cast<int>(sf.row_sign.size()) ? sf.row_sign[row]
                                                        : 1.0) *
            dual_ray[static_cast<std::size_t>(row)];
        if (sf.row_to_artificial_col[row] >= 0 &&
            sf.row_to_surplus_col[row] < 0) {
          if (eq_pos < eq_count) out.result.stats.farkas_ray_eq[eq_pos++] = value;
        } else {
          if (ineq_pos < ineq_count) out.result.stats.farkas_ray[ineq_pos++] = value;
        }
      }
      out.result.stats.has_farkas_certificate = true;
    }
    if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF: infeasible m={} n={} "
                 "hint={} ray={} iter={} time={:.3f}ms\n",
                 m, n, basis_hint ? 1 : 0, has_dual_ray ? 1 : 0,
                 out.result.stats.iterations,
                 out.result.stats.runtime_sec * 1000.0);
    }
    return true;
  }
  if (run_status != HighsStatus::kOk ||
      model_status != HighsModelStatus::kOptimal) {
    out.result.stats.status =
        std::string("HiGHS ") + highs_sf_model_status_label(model_status);
    if (trace_sf) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF rejected: reason=status "
                 "run={} model={} iter={} m={} n={} hint={} time={:.3f}ms\n",
                 static_cast<int>(run_status),
                 highs_sf_model_status_label(model_status),
                 out.result.stats.iterations, m, n, basis_hint ? 1 : 0,
                 out.result.stats.runtime_sec * 1000.0);
    }
    return false;
  }

  const HighsSolution& sol = highs->getSolution();
  const HighsBasis& basis = highs->getBasis();
  if (!basis.valid || static_cast<int>(basis.col_status.size()) < n ||
      static_cast<int>(basis.row_status.size()) < m ||
      static_cast<int>(sol.col_value.size()) < n) {
    return reject("invalid_basis_or_solution");
  }

  out.x_std = Eigen::VectorXd::Zero(n);
  for (int j = 0; j < n; ++j) {
    out.x_std[j] = sol.col_value[static_cast<std::size_t>(j)];
  }
  out.result.x = extract_solution(sf, out.x_std);
  out.x_basic = Eigen::VectorXd::Zero(m);
  out.basis.rows = m;
  out.basis.cols = n;
  out.basis.indices.assign(static_cast<std::size_t>(m), -1);
  out.basis.at_upper.assign(static_cast<std::size_t>(n), 0);
  out.basis.sf_n_slack = sf.n_slack;
  out.basis.sf_n_surplus = sf.n_surplus;
  out.basis.sf_n_artificial = sf.n_artificial;

  for (int j = 0; j < n; ++j) {
    out.basis.at_upper[static_cast<std::size_t>(j)] =
        basis.col_status[static_cast<std::size_t>(j)] ==
                HighsBasisStatus::kUpper
            ? 1
            : 0;
  }

  std::vector<HighsInt> basic(static_cast<std::size_t>(m), 0);
  if (highs->getBasicVariables(basic.data()) != HighsStatus::kOk) {
    return reject("basic_variables");
  }
  std::vector<char> is_artificial(static_cast<std::size_t>(n), 0);
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n) is_artificial[static_cast<std::size_t>(art)] = 1;
  }
  for (int row = 0; row < m; ++row) {
    const int col = native_sf_col_from_highs_basic(
        sf, basic[static_cast<std::size_t>(row)]);
    if (col < 0 || col >= n) return reject("basic_index_out_of_range");
    if (is_artificial[static_cast<std::size_t>(col)] &&
        std::abs(out.x_std[col]) > std::max(1e-7, opt.feasibility_tol * 20.0)) {
      return reject("positive_artificial_basic");
    }
    out.basis.indices[static_cast<std::size_t>(row)] = col;
    out.x_basic[row] = out.x_std[col];
  }

  out.reduced_costs = Eigen::VectorXd::Zero(n);
  {
    const bool have_col_dual = static_cast<int>(sol.col_dual.size()) >= n;
    if (have_col_dual) {
      const bool have_col_scale = sf.col_scale.size() == n;
      for (int j = 0; j < n; ++j) {
        const double col_scale = have_col_scale ? sf.col_scale[j] : 1.0;
        // Keep the same sign convention as native simplex:
        // col_dual = -reduced_cost / col_scale  => reduced_cost = -col_dual*col_scale
        out.reduced_costs[j] =
            -sol.col_dual[static_cast<std::size_t>(j)] * col_scale;
      }
    } else {
      Eigen::VectorXd c_b = Eigen::VectorXd::Zero(m);
      for (int row = 0; row < m; ++row) {
        const int col = out.basis.indices[static_cast<std::size_t>(row)];
        c_b[row] = (col >= 0 && col < n) ? sf.c_max[col] : 0.0;
      }
      std::vector<double> rhs(static_cast<std::size_t>(m), 0.0);
      std::vector<double> y(static_cast<std::size_t>(m), 0.0);
      for (int row = 0; row < m; ++row) {
        rhs[static_cast<std::size_t>(row)] = c_b[row];
      }
      if (highs->getBasisTransposeSolve(rhs.data(), y.data()) !=
          HighsStatus::kOk) {
        return reject("basis_transpose_solve");
      }
      Eigen::VectorXd y_scaled = Eigen::VectorXd::Zero(m);
      for (int row = 0; row < m; ++row) {
        y_scaled[row] = y[static_cast<std::size_t>(row)];
      }
      out.reduced_costs =
          sf.c_max - Eigen::VectorXd(sf.A.transpose() * y_scaled);
    }
  }

  out.basis_inverse.resize(0, 0);

  out.max_objective = info.objective_function_value;
  out.result.stats.success = true;
  out.result.stats.objective = sf.objective_const - out.max_objective;
  out.result.stats.status = "Optimal";
  out.result.stats.primal_feas = 0.0;
  out.result.stats.residual_inf = 0.0;
  out.solved_from_hint = basis_hint != nullptr;
  out.dual_reoptimized = basis_hint != nullptr;
  out.exact_optimal = true;
  out.basis.cached_reduced_costs =
      std::make_shared<const Eigen::VectorXd>(out.reduced_costs);
  if (sf.col_scale.size() == n) {
    out.basis.cached_col_scale =
        std::make_shared<const Eigen::VectorXd>(sf.col_scale);
  }
  out.basis.cached_sparse_basis =
      std::make_shared<VendoredHighsBasis>(
          highs, m, n, sf.structure_id);
  out.basis.persist_eta_count = 0;
  const int degenerate_duals = inject_basic_degenerate_col_duals_from_highs(
      sf, *highs, sol, out.basis.basis_indices(), out.reduced_costs,
      kHighsDefaultMipTolerance, kHighsDefaultKktTolerance);
  if (degenerate_duals > 0) {
    out.basis.cached_reduced_costs =
        std::make_shared<const Eigen::VectorXd>(out.reduced_costs);
  }
  if (trace_sf && degenerate_duals > 0) {
    fmt::print(stderr,
               "[SF_RELAX] VendoredHiGHS basic-degenerate duals injected={}\n",
               degenerate_duals);
  }
  double primal_residual = 0.0;
  double bound_violation = 0.0;
  double artificial_activity = 0.0;
  double dual_violation = 0.0;
  double objective_gap = 0.0;
  const double highs_dual_violation =
      info.num_dual_infeasibilities <= 0
          ? 0.0
          : (std::isfinite(info.max_dual_infeasibility)
                 ? info.max_dual_infeasibility
                 : std::numeric_limits<double>::infinity());
  if (!audit_vendored_highs_sf_result(sf, opt, out, primal_residual,
                                      bound_violation, artificial_activity,
                                      dual_violation, objective_gap,
                                      highs_dual_violation)) {
    if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
      fmt::print(stderr,
                 "[SF_RELAX] VendoredHiGHS SF audit rejected: "
                 "primal={:.3e} bound={:.3e} art={:.3e} dual={:.3e} "
                 "obj_gap={:.3e} m={} n={} hint={}\n",
                 primal_residual, bound_violation, artificial_activity,
                 dual_violation, objective_gap, m, n, basis_hint ? 1 : 0);
    }
    return false;
  }
  out.result.stats.primal_feas = std::max(primal_residual, bound_violation);
  out.result.stats.residual_inf = primal_residual;
  out.result.stats.dual_feas = dual_violation;
  populate_dual_certificate(sf, out.basis.indices, out.basis.at_upper,
                            out.basis_inverse, out.basis.cached_sparse_basis,
                            out.reduced_costs, out.result);
  if (static_cast<int>(sol.row_dual.size()) >= m) {
    out.result.constraint_duals.resize(m);
    for (int row = 0; row < m; ++row) {
      const double row_scale =
          sf.row_scale.size() == m ? sf.row_scale[row] : 1.0;
      const double row_sign =
          row < static_cast<int>(sf.row_sign.size())
              ? static_cast<double>(sf.row_sign[static_cast<std::size_t>(row)])
              : 1.0;
      // The delegated standard-form model is solved with the internal
      // maximization objective. Publish the derivative of the original
      // minimization value with respect to the original row RHS: undo the row
      // sign/scale and reverse the internal objective direction. For an active
      // original <= row this is non-positive, matching HiGHS' public LP
      // row-dual convention.
      out.result.constraint_duals[row] =
          -row_sign * row_scale * sol.row_dual[static_cast<std::size_t>(row)];
    }
  }
  if (std::getenv("MIPSOLVERS_HIGHS_LP_KERNEL_TRACE") != nullptr) {
    fmt::print(stderr,
               "[SF_RELAX] VendoredHiGHS SF: success=1 iter={} obj={:.12g} "
               "m={} n={} hint={} time={:.3f}ms\n",
               out.result.stats.iterations, out.result.stats.objective,
               m, n, basis_hint ? 1 : 0,
               out.result.stats.runtime_sec * 1000.0);
  }
  return true;
}
#else
bool solve_standard_form_with_vendored_highs(const StandardFormLP&,
                                             const SimplexOptions&,
                                             const SimplexBasis*,
                                             SimplexResult&) {
  return false;
}
#endif

}  // namespace

void clear_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb) {
  if (sb) sb->clear_etas();
}

void truncate_sparse_basis_etas(const std::shared_ptr<BasisOps>& sb, int target_count) {
  if (!sb) return;
  const int cur = sb->eta_count();
  if (target_count < cur) {
    // Truncate etas back to the target count. This restores the SparseBasis
    // to the state it was in at 'target_count' etas (before sibling mutation).
    sb->truncate_etas_to(target_count);
  }
}

int sparse_basis_eta_count(const std::shared_ptr<BasisOps>& sb) {
  if (!sb) return 0;
  return sb->eta_count();
}

int sparse_basis_generation(const std::shared_ptr<BasisOps>& sb) {
  if (sb) {
    return sb->generation();
  }
  return -1;
}

SparseFactorTelemetry get_sparse_basis_factor_telemetry(
    const std::shared_ptr<BasisOps>& sb) {
  if (!sb) return {};
  return sb->factor_telemetry();
}

void rebind_sparse_basis_matrix(const std::shared_ptr<BasisOps>& sb,
                                const StandardColumnMatrix& A) {
  if (!sb) return;
  sb->rebind_A(A);
}

bool sparse_basis_bound_to_matrix(const std::shared_ptr<BasisOps>& sb,
                                  const StandardColumnMatrix& A) {
  if (!sb) return false;
  return sb->bound_to_A(A);
}

Eigen::VectorXd sparse_basis_btran(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                                   const Eigen::VectorXd& rhs) {
  if (!cached_sparse_basis) {
    return Eigen::VectorXd::Zero(rhs.size());
  }
  return cached_sparse_basis->btran(rhs);
}

bool sparse_basis_inverse_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::VectorXd& out) {
  if (!cached_sparse_basis) return false;
  return cached_sparse_basis->basis_inverse_row(row, out);
}

bool sparse_basis_inverse_row_sparse_entries(
    const std::shared_ptr<BasisOps>& cached_sparse_basis,
    int row,
    std::vector<std::pair<int, double>>& out) {
  if (!cached_sparse_basis) return false;
  return cached_sparse_basis->basis_inverse_row_sparse_entries(row, out);
}

bool sparse_basis_tableau_row(const std::shared_ptr<BasisOps>& cached_sparse_basis,
                              int row,
                              Eigen::RowVectorXd& out) {
  if (!cached_sparse_basis) return false;
  return cached_sparse_basis->tableau_row(row, out);
}

static SimplexResult solve_lp_from_sf_impl(
    const StandardFormLP& sf, const SimplexOptions& input_opt,
    const SimplexBasis* basis_hint);

static SimplexResult solve_lp_with_basis_impl(const LPModel& lp,
                                              const SimplexOptions& input_opt,
                                              const SimplexBasis* basis_hint) {
  SimplexOptions opt = input_opt;
  opt.retain_standard_form = false;
  bool local_time_limit_hit = false;
  if (opt.time_limit_hit == nullptr) opt.time_limit_hit = &local_time_limit_hit;

  StandardFormLP form = build_standard_form_lp(lp);
  ruiz_scale_standard_form(form, 10);

  SimplexResult solved = solve_lp_from_sf_impl(form, opt, basis_hint);
  std::shared_ptr<BasisOps> solved_basis = solved.basis.cached_sparse_basis;
  solved.form = std::move(form);

  // The factor retains its numeric decomposition across the StandardFormLP
  // move; only the matrix object address changes.
  const int m = solved.form.A.rows();
  if (solved.result.stats.success && solved_basis &&
      static_cast<int>(solved.basis.index_count()) == m) {
    solved_basis->rebind_A(solved.form.A);
    solved.basis.cached_sparse_basis = std::move(solved_basis);
    solved.basis.persist_eta_count = 0;
  }
  return solved;
}

/// Original-space feasibility audit for every successful solve. Accepts when
///   max(row violation, bound violation) <= tol * max(1, |b|_inf, |beq|_inf).
bool lp_solution_residual_acceptable(const LPModel& lp,
                                     const Eigen::VectorXd& x, double tol) {
  const int n = x.size();
  if (n != static_cast<int>(lp.vars.size()) || !x.allFinite() ||
      !std::isfinite(tol)) {
    return false;
  }
  if (lp.A.rows() != lp.b.size() ||
      (lp.A.rows() > 0 && lp.A.cols() != n) ||
      (lp.row_lhs.size() != 0 && lp.row_lhs.size() != lp.A.rows()) ||
      lp.Aeq.rows() != lp.beq.size() ||
      (lp.Aeq.rows() > 0 && lp.Aeq.cols() != n)) {
    return false;
  }
  double viol = 0.0;
  double scale = 1.0;
  // Row sides with |side| >= 1e19 are "no bound" sentinels (1e20, possibly
  // enlarged further by adversarial instance scaling).  They must not enter
  // the acceptance scale — otherwise tol * scale turns astronomical and the
  // audit accepts anything (found on stocfor1 at 1e6 diagonal scaling,
  // rel-err 7e17 returned as "Optimal").
  constexpr double kSideSentinel = 1e19;
  if (lp.A.rows() > 0) {
    const Eigen::VectorXd ax = lp.A * x;
    if (!ax.allFinite()) return false;
    for (int i = 0; i < ax.size(); ++i) {
      if (std::isnan(lp.b[i])) return false;
      if (std::abs(lp.b[i]) < kSideSentinel) {
        viol = std::max(viol, ax[i] - lp.b[i]);
        scale = std::max(scale, std::abs(lp.b[i]));
      }
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (std::isnan(lhs)) return false;
      if (std::isfinite(lhs) && std::abs(lhs) < kSideSentinel) {
        viol = std::max(viol, lhs - ax[i]);
        scale = std::max(scale, std::abs(lhs));
      }
    }
  }
  if (lp.Aeq.rows() > 0) {
    const Eigen::VectorXd aeqx = lp.Aeq * x;
    if (!aeqx.allFinite() || !lp.beq.allFinite()) return false;
    for (int i = 0; i < aeqx.size(); ++i) {
      viol = std::max(viol, std::abs(aeqx[i] - lp.beq[i]));
      scale = std::max(scale, std::abs(lp.beq[i]));
    }
  }
  for (int j = 0; j < x.size(); ++j) {
    const auto& v = lp.vars[static_cast<std::size_t>(j)];
    if (std::isnan(v.lb) || std::isnan(v.ub)) return false;
    if (std::isfinite(v.lb)) viol = std::max(viol, v.lb - x[j]);
    if (std::isfinite(v.ub)) viol = std::max(viol, x[j] - v.ub);
  }
  return viol <= tol * scale;
}

// AUDIT-NAV: LP kernel 分派与原空间结果恢复入口；先审核标准型映射，再审核
// HiGHS/Native 后端选择、basis hint 兼容性和最终残差/证书。
SimplexResult solve_lp_with_basis(const LPModel& lp,
                                  const SimplexOptions& input_opt,
                                  const SimplexBasis* basis_hint) {
  if (std::getenv("MIPSOLVERS_PRESOLVE_ANALYZE") != nullptr) {
    const int n = static_cast<int>(lp.vars.size());
    const int mi = static_cast<int>(lp.A.rows());
    const int me = static_cast<int>(lp.Aeq.rows());
    std::vector<int> ineq_nnz(static_cast<std::size_t>(mi), 0);
    std::vector<int> eq_nnz(static_cast<std::size_t>(me), 0);
    std::vector<int> col_nnz(static_cast<std::size_t>(n), 0);
    for (int k = 0; k < lp.A.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, k); it; ++it) {
        if (it.value() != 0.0) {
          ++ineq_nnz[static_cast<std::size_t>(it.row())];
          ++col_nnz[static_cast<std::size_t>(it.col())];
        }
      }
    }
    for (int k = 0; k < lp.Aeq.outerSize(); ++k) {
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.Aeq, k); it; ++it) {
        if (it.value() != 0.0) {
          ++eq_nnz[static_cast<std::size_t>(it.row())];
          ++col_nnz[static_cast<std::size_t>(it.col())];
        }
      }
    }
    int ineq_empty = 0, ineq_single = 0, ineq_free = 0;
    for (int i = 0; i < mi; ++i) {
      if (ineq_nnz[static_cast<std::size_t>(i)] == 0) ++ineq_empty;
      else if (ineq_nnz[static_cast<std::size_t>(i)] == 1) ++ineq_single;
      const bool ub_inf = std::abs(lp.b[i]) >= 1e19;
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      if (ub_inf && !std::isfinite(lhs)) ++ineq_free;
    }
    // Redundant inequality rows: activity range implied by variable bounds is
    // already inside [lhs, b].  min/max activity accumulated per row.
    std::vector<double> min_act(static_cast<std::size_t>(mi), 0.0);
    std::vector<double> max_act(static_cast<std::size_t>(mi), 0.0);
    for (int k = 0; k < lp.A.outerSize(); ++k) {
      const auto& v = lp.vars[static_cast<std::size_t>(k)];
      const double lo = v.lb, hi = v.ub;
      for (Eigen::SparseMatrix<double>::InnerIterator it(lp.A, k); it; ++it) {
        const double a = it.value();
        if (a == 0.0) continue;
        const std::size_t r = static_cast<std::size_t>(it.row());
        min_act[r] += (a > 0.0) ? a * lo : a * hi;
        max_act[r] += (a > 0.0) ? a * hi : a * lo;
      }
    }
    int ineq_redundant = 0;
    for (int i = 0; i < mi; ++i) {
      const double bi = lp.b[i];
      const double lhs = lp_row_lhs_or_neg_inf(lp, i);
      const bool ub_ok = (std::abs(bi) >= 1e19) ||
                         (max_act[static_cast<std::size_t>(i)] <= bi + 1e-9);
      const bool lb_ok = !std::isfinite(lhs) ||
                         (min_act[static_cast<std::size_t>(i)] >= lhs - 1e-9);
      if (ub_ok && lb_ok && ineq_nnz[static_cast<std::size_t>(i)] > 0) {
        ++ineq_redundant;
      }
    }
    int eq_empty = 0, eq_single = 0, eq_double = 0;
    for (int i = 0; i < me; ++i) {
      if (eq_nnz[static_cast<std::size_t>(i)] == 0) ++eq_empty;
      else if (eq_nnz[static_cast<std::size_t>(i)] == 1) ++eq_single;
      else if (eq_nnz[static_cast<std::size_t>(i)] == 2) ++eq_double;
    }
    int col_empty = 0, col_fixed = 0, col_singleton = 0;
    for (int j = 0; j < n; ++j) {
      if (col_nnz[static_cast<std::size_t>(j)] == 0) ++col_empty;
      else if (col_nnz[static_cast<std::size_t>(j)] == 1) ++col_singleton;
      const auto& v = lp.vars[static_cast<std::size_t>(j)];
      if (std::isfinite(v.lb) && std::isfinite(v.ub) &&
          v.ub - v.lb <= 1e-11 * std::max(1.0, std::abs(v.lb))) {
        ++col_fixed;
      }
    }
    std::fprintf(stderr,
                 "PRESOLVE-ANALYZE: n=%d m_ineq=%d m_eq=%d || ineq_empty=%d "
                 "ineq_single=%d ineq_free=%d ineq_redundant=%d || eq_empty=%d "
                 "eq_single=%d eq_double=%d || col_empty=%d col_single=%d "
                 "col_fixed=%d\n",
                 n, mi, me, ineq_empty, ineq_single, ineq_free, ineq_redundant,
                 eq_empty, eq_single, eq_double, col_empty, col_singleton,
                 col_fixed);
  }

  // Adaptive HiGHS presolve (opt-in via SimplexOptions::use_highs_presolve or
  // the MIPSOLVERS_PRESOLVE env var).  Solve the reduced LP with the native
  // kernel, then postsolve the primal to original space.  Only applied on cold
  // solves (a basis hint refers to the original LP, not the reduced one).  Any
  // failure falls through to a direct solve, so a wrong or infeasible answer is
  // never published.
  {
    HighsLpPresolveConfig pcfg;
    pcfg.enabled = input_opt.use_highs_presolve;
    pcfg = highs_lp_presolve_config_from_env(pcfg);
    if (pcfg.enabled && basis_hint == nullptr) {
      HighsLpPresolveResult ps = highs_presolve_lp(lp, pcfg);
      if (ps.infeasible) {
        SimplexResult r;
        r.result.stats.solver_name = "natDualSimplex+HiGHSpresolve";
        r.result.stats.success = false;
        r.result.stats.status = "Infeasible (presolve)";
        return r;
      }
      if (ps.solved_by_presolve || ps.use_reduced) {
        Eigen::VectorXd x_reduced;
        int iters = 0;
        SolveStats reduced_stats;
        bool reduced_ok = true;
        if (ps.use_reduced) {
          // Solve the reduced LP directly via the impl (no re-entrant presolve).
          SimplexResult rr =
              solve_lp_with_basis_impl(ps.reduced, input_opt, nullptr);
          reduced_ok = rr.result.stats.success;
          x_reduced = rr.result.x;
          iters = rr.result.stats.iterations;
          reduced_stats = rr.result.stats;
        }  // else: reduced-to-empty -> postsolve an empty primal.
        if (reduced_ok) {
          Eigen::VectorXd x_orig;
          double obj = 0.0;
          if (highs_presolve_recover_primal(lp, ps, x_reduced,
                                            input_opt.escalation_residual_tol,
                                            x_orig, obj)) {
            SimplexResult r;
            r.result.x = std::move(x_orig);
            r.result.stats = std::move(reduced_stats);
            r.result.stats.solver_name = "natDualSimplex+HiGHSpresolve";
            r.result.stats.success = true;
            r.result.stats.status = "Optimal";
            r.result.stats.objective = obj;
            r.result.stats.iterations = iters;
            return r;
          }
        }
        // Postsolve/audit/reduced-solve failure: fall through to a direct solve.
      }
    }
  }

  SimplexResult res =
      solve_lp_with_basis_impl(lp, input_opt, basis_hint);
  if (res.result.stats.success &&
      (!std::isfinite(res.result.stats.objective) ||
       !lp_solution_residual_acceptable(lp, res.result.x,
                                        input_opt.escalation_residual_tol))) {
    res.result.stats.success = false;
    res.result.stats.status = "Residual audit rejected";
  }
  return res;
}

// Retained as an ABI-compatible no-op for older branch-and-cut integrations.
void dump_solve_lp_counters() {}

static SimplexResult solve_lp_from_sf_impl(
    const StandardFormLP& sf, const SimplexOptions& input_opt,
    const SimplexBasis* basis_hint) {
  SimplexOptions opt = input_opt;
  bool local_time_limit_hit = false;
  if (opt.time_limit_hit == nullptr) opt.time_limit_hit = &local_time_limit_hit;
  SimplexResult out;
  out.result.stats.solver_name = "NativeSimplex";
  const auto solve_wall_t0 = std::chrono::steady_clock::now();
  struct TimeLimitStatusGuard {
    SimplexResult& out;
    const SimplexOptions& opt;
    const std::chrono::steady_clock::time_point& start;
    ~TimeLimitStatusGuard() {
      const bool hit = opt.time_limit_hit != nullptr && *opt.time_limit_hit;
      if (!out.result.stats.success &&
          (hit || simplex_wall_time_limit_hit(opt, start))) {
        mark_simplex_time_limit(out.result.stats, start);
      }
    }
  } time_limit_guard{out, opt, solve_wall_t0};

  const int m = sf.A.rows();
  const int n = sf.A.cols();
  thread_local int sf_call_count = 0;
  ++sf_call_count;
  if (m == 0) {
    out.x_std = Eigen::VectorXd::Zero(n);
    for (int col = 0; col < n; ++col) {
      if (sf.c_max[col] <= opt.optimality_tol) continue;
      if (!std::isfinite(sf.var_ub[col])) {
        Eigen::VectorXd ray_std = Eigen::VectorXd::Zero(n);
        ray_std[col] = 1.0;
        out.result.primal_ray = extract_primal_ray(sf, ray_std);
        out.result.stats.has_unbounded_certificate =
            out.result.primal_ray.size() == sf.n_original &&
            out.result.primal_ray.lpNorm<Eigen::Infinity>() > 0.0;
        out.result.stats.success = false;
        out.result.stats.objective = -kInf;
        out.result.stats.status = "LP unbounded";
        return out;
      }
      out.x_std[col] = sf.var_ub[col];
    }
    out.result.x = extract_solution(sf, out.x_std);
    out.result.stats.success = true;
    out.max_objective = sf.c_max.dot(out.x_std);
    out.result.stats.objective = sf.objective_const - out.max_objective;
    out.result.stats.status = "Optimal";
    out.exact_optimal = true;
    return out;
  }

  if (uses_highs_lp_kernel(opt.lp_kernel_backend)) {
    if (solve_standard_form_with_vendored_highs(sf, opt, basis_hint, out)) {
      return out;
    }
    out.result.stats.success = false;
    if (out.result.stats.status.empty()) {
      out.result.stats.status = "VendoredHiGHS SF rejected";
    }
    return out;
  }

  int actual_iters = 0;
  std::vector<int> basis(static_cast<size_t>(m), -1);
  std::vector<char> at_upper(static_cast<size_t>(n), 0);
  for (int i = 0; i < m; ++i) {
    if (sf.row_to_slack_col[i] >= 0) {
      basis[i] = sf.row_to_slack_col[i];
    } else if (sf.row_to_artificial_col[i] >= 0) {
      basis[i] = sf.row_to_artificial_col[i];
    }
  }

  Eigen::MatrixXd binv;
  Eigen::VectorXd x_b;
  Eigen::VectorXd reduced_costs;
  double obj = 0.0;
  std::shared_ptr<BasisOps> solved_basis_ops;

  // Try to extend basis hint when root cuts added extra rows.
  std::shared_ptr<SimplexBasis> extended_hint;
    if (basis_hint != nullptr && basis_hint->rows < m && basis_hint->rows > 0 &&
      static_cast<int>(basis_hint->index_count()) == basis_hint->rows) {
      const auto& hint_indices = basis_hint->basis_indices();
    // Extend: copy old basis indices, add slack columns for new rows.
    // SF row layout: [ineq | eq]. Adding cut rows (ineq) inserts them between
    // old ineq and old eq, so old eq rows shift down.
    const int old_rows = basis_hint->rows;
    const int old_cols = basis_hint->cols;
    const int n_new_rows = m - old_rows;
    const int n_orig = sf.n_original;

    // Determine old SF column counts using saved values if available.
    const int n_slack_old = (basis_hint->sf_n_slack >= 0)
        ? basis_hint->sf_n_slack
        : old_cols - n_orig - sf.n_surplus - sf.n_artificial;
    const int n_surplus_old = (basis_hint->sf_n_surplus >= 0)
        ? basis_hint->sf_n_surplus : sf.n_surplus;
    const int delta_slack = sf.n_slack - n_slack_old;
    const int delta_surplus = sf.n_surplus - n_surplus_old;

    // Determine the boundary between old ineq and eq rows.
    // old_m_eq = old_rows - old_m_ineq. We know m_eq from the SF:
    // m_eq = number of rows with sense 'E'. Since original eq rows don't change,
    // m_eq_new == m_eq_old. Count eq rows from the new SF.
    int m_eq_new = 0;
    for (int i = 0; i < m; ++i) {
      if (sf.row_to_slack_col[i] < 0 && sf.row_to_surplus_col[i] < 0 &&
          sf.row_to_artificial_col[i] >= 0) {
        ++m_eq_new;
      }
    }
    const int old_m_ineq = old_rows - m_eq_new;

    auto adjust_col = [&](int idx) -> int {
      if (idx < n_orig + n_slack_old) return idx;
      if (idx < n_orig + n_slack_old + n_surplus_old)
        return idx + delta_slack;
      return idx + delta_slack + delta_surplus;
    };

    extended_hint = std::make_shared<SimplexBasis>(*basis_hint);
    // Materialize shared basis storage before resizing.  The appended-row
    // refactor oracle must start from the same extended basis as the
    // parent-factor path; otherwise basis_indices() keeps returning the old
    // shared vector and the "refactor" comparison silently falls back to a
    // different solve path.
    extended_hint->ensure_owned_indices();
    extended_hint->rows = m;
    extended_hint->cols = n;
    extended_hint->indices.resize(static_cast<size_t>(m));
    extended_hint->at_upper.assign(static_cast<size_t>(n), 0);
    if (!basis_hint->at_upper.empty()) {
      for (int j = 0; j < old_cols &&
                      j < static_cast<int>(basis_hint->at_upper.size());
           ++j) {
        const int new_j = adjust_col(j);
        if (new_j >= 0 && new_j < n) {
          extended_hint->at_upper[static_cast<std::size_t>(new_j)] =
              basis_hint->at_upper[static_cast<std::size_t>(j)];
        }
      }
    }

    // Old inequality rows (0..old_m_ineq-1) stay at same positions.
    for (int i = 0; i < old_m_ineq; ++i) {
        extended_hint->indices[static_cast<size_t>(i)] =
          adjust_col(hint_indices[static_cast<size_t>(i)]);
    }
    // New cut rows → positions old_m_ineq..old_m_ineq+n_new_rows-1.
    for (int k = 0; k < n_new_rows; ++k) {
      const int r = old_m_ineq + k;
      const int slack = sf.row_to_slack_col[static_cast<size_t>(r)];
      const int art = sf.row_to_artificial_col[static_cast<size_t>(r)];
      extended_hint->indices[static_cast<size_t>(r)] =
          (slack >= 0) ? slack : (art >= 0) ? art : -1;
    }
    // Old equality rows shift down by n_new_rows.
    for (int i = old_m_ineq; i < old_rows; ++i) {
      const int new_row = i + n_new_rows;
        extended_hint->indices[static_cast<size_t>(new_row)] =
          adjust_col(hint_indices[static_cast<size_t>(i)]);
    }

    // Invalidate cached basis state with the old dimensions.
    extended_hint->cached_reduced_costs.reset();
    extended_hint->cached_sparse_basis.reset();
    basis_hint = extended_hint.get();
  }

  // BASIS PROJECTION: hint was built for a larger LP (e.g., root_lp+cuts),
  // but current SF is for base_lp (no cuts). Project out cut rows.
  // Cut slacks can only appear in their own cut rows, so base rows are clean.
  // This enables warm-start for pump/prog/dive LPs that use base_lp.
  if (!extended_hint && basis_hint != nullptr &&
      basis_hint->rows > m && m > 0 &&
      basis_hint->sf_n_slack >= 0 &&
      static_cast<int>(basis_hint->index_count()) == basis_hint->rows) {
    const int old_rows = basis_hint->rows;
    const auto& hint_indices = basis_hint->basis_indices();
    const int n_orig = sf.n_original;
    const int n_slack_new = sf.n_slack;
    const int n_slack_old = basis_hint->sf_n_slack;
    const int delta_slack = n_slack_old - n_slack_new;  // = k_cuts (positive)
    const int n_surplus_old = (basis_hint->sf_n_surplus >= 0) ? basis_hint->sf_n_surplus : sf.n_surplus;
    const int delta_surplus = n_surplus_old - sf.n_surplus;

    if (delta_slack > 0 && delta_slack == basis_hint->rows - m &&
        delta_surplus == 0) {
      // Count eq rows in current (smaller) SF.
      int m_eq_new = 0;
      for (int i = 0; i < m; ++i) {
        if (sf.row_to_slack_col[i] < 0 && sf.row_to_surplus_col[i] < 0 &&
            sf.row_to_artificial_col[i] >= 0) {
          ++m_eq_new;
        }
      }
      const int m_ineq_new = m - m_eq_new;
      const int m_ineq_old = m_ineq_new + delta_slack;  // includes cut rows

      // Column adjustment: project from larger SF to smaller SF.
      // Cut slack columns [n_orig+n_slack_new, n_orig+n_slack_old) are absent.
      auto project_col = [&](int col) -> int {
        if (col < 0) return col;
        if (col < n_orig + n_slack_new) return col;           // original or base slack
        if (col < n_orig + n_slack_old) return -(col + 10);   // cut slack (unexpected in base row)
        return col - delta_slack;                              // surplus/artificial: shift down
      };

      auto proj_hint = std::make_shared<SimplexBasis>();
      proj_hint->rows = m;
      proj_hint->cols = n;
      proj_hint->indices.resize(static_cast<size_t>(m), -1);
      proj_hint->at_upper.resize(static_cast<size_t>(n), 0);
      // Copy at_upper for original and base-slack columns (same positions).
      if (!basis_hint->at_upper.empty()) {
        const int copy_n = std::min(n_orig + n_slack_new,
                                    std::min(n, static_cast<int>(basis_hint->at_upper.size())));
        for (int j = 0; j < copy_n; ++j)
          proj_hint->at_upper[static_cast<size_t>(j)] = basis_hint->at_upper[static_cast<size_t>(j)];
        // Surplus/art columns: shift from old positions.
        const int surplus_offset_new = n_orig + n_slack_new;
        const int surplus_offset_old = n_orig + n_slack_old;
        const int n_sa = n - surplus_offset_new;
        for (int j = 0; j < n_sa; ++j) {
          const int old_j = surplus_offset_old + j;
          if (old_j < static_cast<int>(basis_hint->at_upper.size()))
            proj_hint->at_upper[static_cast<size_t>(surplus_offset_new + j)] =
                basis_hint->at_upper[static_cast<size_t>(old_j)];
        }
      }

      bool valid = true;
      // Base inequality rows 0..m_ineq_new-1 (same position in both).
      for (int i = 0; i < m_ineq_new && valid; ++i) {
        const int col = project_col(hint_indices[static_cast<size_t>(i)]);
        if (col < 0 || col >= n) {
          if (col < 0) {
            // cut slack in base row — substitute row's own slack/art
            const int s = sf.row_to_slack_col[i];
            const int a = sf.row_to_artificial_col[i];
            proj_hint->indices[static_cast<size_t>(i)] = (s >= 0) ? s : (a >= 0) ? a : -1;
            if (proj_hint->indices[static_cast<size_t>(i)] < 0) valid = false;
          } else { valid = false; }
        } else {
          proj_hint->indices[static_cast<size_t>(i)] = col;
        }
      }
      // Equality rows: in hint at m_ineq_old..m_ineq_old+m_eq_new-1,
      // in base SF at m_ineq_new..m_ineq_new+m_eq_new-1.
      for (int i = 0; i < m_eq_new && valid; ++i) {
        const int hint_row = m_ineq_old + i;
        const int base_row = m_ineq_new + i;
        if (hint_row >= old_rows) { valid = false; break; }
        const int col = project_col(hint_indices[static_cast<size_t>(hint_row)]);
        if (col < 0 || col >= n) {
          if (col < 0) {
            const int s = sf.row_to_slack_col[base_row];
            const int a = sf.row_to_artificial_col[base_row];
            proj_hint->indices[static_cast<size_t>(base_row)] = (s >= 0) ? s : (a >= 0) ? a : -1;
            if (proj_hint->indices[static_cast<size_t>(base_row)] < 0) valid = false;
          } else { valid = false; }
        } else {
          proj_hint->indices[static_cast<size_t>(base_row)] = col;
        }
      }

      if (valid) {
        proj_hint->cached_reduced_costs.reset();
        proj_hint->cached_sparse_basis.reset();
        proj_hint->sf_n_slack = n_slack_new;
        proj_hint->sf_n_surplus = sf.n_surplus;
        proj_hint->sf_n_artificial = sf.n_artificial;
        extended_hint = proj_hint;
        basis_hint = extended_hint.get();
      }
    }
  }

  bool hint_match = false;
  if (basis_hint != nullptr && basis_hint->rows == m && basis_hint->cols == n &&
      static_cast<int>(basis_hint->index_count()) == m) {
    const auto& hint_indices = basis_hint->basis_indices();
    // Validate all basis indices are in [0, n).
    bool indices_valid = true;
    for (int i = 0; i < m; ++i) {
      if (hint_indices[static_cast<size_t>(i)] < 0 ||
          hint_indices[static_cast<size_t>(i)] >= n) {
        indices_valid = false;
        break;
      }
    }
    if (indices_valid) {
      basis = hint_indices;
      if (!basis_hint->at_upper.empty()) at_upper = basis_hint->at_upper;
      hint_match = true;
    }
  }
  trace_lp_basis_hint_state(sf, basis, at_upper,
                            hint_match ? "entry_hint" : "entry_default",
                            sf_call_count, hint_match);

  // The rewritten native kernel owns both cold and warm state machines.
  // Every failure is terminal here: retrying a different factor policy,
  // discarding a warm basis, or entering the legacy driver would hide a
  // violated algebraic invariant.
  const auto native_kernel_start = std::chrono::steady_clock::now();
  native_dual::Result native =
      native_dual::solve(sf, opt, hint_match ? basis_hint : nullptr);
  const double native_kernel_time_sec =
      std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                    native_kernel_start)
          .count();
  actual_iters = native.statistics.iterations;
  out.result.stats.dual_phase_one_iterations =
      native.statistics.dual_phase_one_iterations;
  out.result.stats.dual_phase_two_iterations =
      native.statistics.dual_phase_two_iterations;
  out.result.stats.dse_initialization_solves =
      native.statistics.dse_initialization_solves;
  out.result.stats.certified_dse_btrans =
      native.statistics.certified_dse_btrans;
  out.result.stats.certified_dse_candidates =
      native.statistics.certified_dse_candidates;
  out.result.stats.certified_dse_rejections =
      native.statistics.certified_dse_rejections;
  out.result.stats.dse_initialization_time_sec =
      native.statistics.dse_initialization_time_sec;
  out.result.stats.certified_dse_time_sec =
      native.statistics.certified_dse_time_sec;
  out.result.stats.native_dual_kernel_time_sec = native_kernel_time_sec;
  MIPSOLVERS_LOG_DEBUG(
      "[NATIVE DUAL] status={} message='{}' m={} n={} iterations={} "
      "reinversions={} rank_repairs={} primal_inf={:.3e} dual_inf={:.3e}",
      native_dual::status_name(native.status), native.message, m, n,
      native.statistics.iterations, native.statistics.reinversions,
      native.statistics.rank_repairs,
      native.statistics.max_primal_infeasibility,
      native.statistics.max_dual_infeasibility);

  if (native.status != native_dual::Status::Optimal) {
    out.result.stats.success = false;
    out.result.stats.iterations = actual_iters;
    out.result.stats.objective = sf.objective_const - native.max_objective;
    if (native.dual_bound_certified) {
      // The kernel re-certified the interrupted state against the original
      // cost (dual-feasibility audit), so objective_const - max_objective is a
      // rigorous lower bound on the LP minimum despite the unfinished solve.
      out.result.stats.certified_dual_bound =
          sf.objective_const - native.max_objective;
    }
    if (native.status == native_dual::Status::ObjectiveCutoff) {
      out.result.stats.status = "Objective cutoff";
    } else if (native.status == native_dual::Status::TimeLimit) {
      out.result.stats.status = "Time limit: " + native.message;
    } else if (native.status == native_dual::Status::PrimalInfeasible) {
      out.result.stats.status = "LP infeasible";
      if (native.has_farkas_certificate &&
          native.farkas_multiplier.size() == m &&
          native.farkas_multiplier.allFinite()) {
        int inequality_count = 0;
        int equality_count = 0;
        for (int row = 0; row < m; ++row) {
          const bool equality = sf.row_to_artificial_col[row] >= 0 &&
                                sf.row_to_surplus_col[row] < 0;
          equality ? ++equality_count : ++inequality_count;
        }
        out.result.stats.farkas_ray =
            Eigen::VectorXd::Zero(inequality_count);
        out.result.stats.farkas_ray_eq =
            Eigen::VectorXd::Zero(equality_count);
        int inequality_position = 0;
        int equality_position = 0;
        for (int row = 0; row < m; ++row) {
          const double row_sign =
              row < static_cast<int>(sf.row_sign.size())
                  ? static_cast<double>(sf.row_sign[row])
                  : 1.0;
          const double value = row_sign * native.farkas_multiplier[row];
          const bool equality = sf.row_to_artificial_col[row] >= 0 &&
                                sf.row_to_surplus_col[row] < 0;
          if (equality) {
            out.result.stats.farkas_ray_eq[equality_position++] = value;
          } else {
            out.result.stats.farkas_ray[inequality_position++] = value;
          }
        }
        out.result.stats.has_farkas_certificate = true;
      }
    } else if (native.status == native_dual::Status::Unbounded &&
               native.has_unbounded_certificate &&
               native.primal_ray.size() == n && native.primal_ray.allFinite()) {
      out.result.primal_ray = extract_primal_ray(sf, native.primal_ray);
      out.result.stats.has_unbounded_certificate =
          out.result.primal_ray.size() == sf.n_original &&
          out.result.primal_ray.allFinite() &&
          out.result.primal_ray.lpNorm<Eigen::Infinity>() > 0.0;
      out.result.stats.objective = -kInf;
      out.result.stats.status = "LP unbounded";
    } else {
      out.result.stats.status =
          std::string("Native dual simplex: ") + native.message;
    }
    return out;
  }

  basis = std::move(native.basis);
  at_upper = std::move(native.at_upper);
  x_b = std::move(native.x_basic);
  reduced_costs = std::move(native.reduced_costs);
  obj = native.max_objective;
  solved_basis_ops = std::move(native.basis_ops);

  out.solved_from_hint = hint_match;
  out.dual_reoptimized = hint_match;
  out.exact_optimal = true;
  out.basis.bound_domain_version = SimplexBasis::kBoundDomainVersion;
  out.basis.indices = basis;
  out.basis.rows = m;
  out.basis.cols = n;
  out.basis.at_upper = at_upper;
  out.basis.cached_sparse_basis = solved_basis_ops;
  if (native.edge_weights.size() == static_cast<std::size_t>(m)) {
    out.basis.cached_dse_basis =
        std::make_shared<const std::vector<int>>(basis);
    out.basis.cached_dse_weights = std::make_shared<const std::vector<double>>(
        std::move(native.edge_weights));
  }
  out.basis.sf_n_slack = sf.n_slack;
  out.basis.sf_n_surplus = sf.n_surplus;
  out.basis.sf_n_artificial = sf.n_artificial;
  out.x_basic = x_b;
  out.reduced_costs = reduced_costs;
  out.max_objective = obj;

  out.x_std = Eigen::VectorXd::Zero(n);
  std::vector<char> is_basic(static_cast<std::size_t>(n), 0);
  for (int row = 0; row < m; ++row) {
    const int col = basis[static_cast<std::size_t>(row)];
    is_basic[static_cast<std::size_t>(col)] = 1;
    out.x_std[col] = x_b[row];
  }
  for (int col = 0; col < n; ++col) {
    if (!is_basic[static_cast<std::size_t>(col)] &&
        at_upper[static_cast<std::size_t>(col)]) {
      out.x_std[col] = sf.var_ub[col];
    }
  }
  out.result.x = extract_solution(sf, out.x_std);
  out.result.stats.success = true;
  out.result.stats.iterations = actual_iters;
  out.result.stats.objective = sf.objective_const - obj;
  out.result.stats.status = "Optimal";
  out.result.stats.primal_feas = native.statistics.max_primal_infeasibility;
  out.result.stats.dual_feas = native.statistics.max_dual_infeasibility;
  populate_dual_certificate(sf, basis, at_upper, out.basis_inverse,
                            out.basis.cached_sparse_basis,
                            out.reduced_costs, out.result);
  return out;
}

bool sf_solution_residual_acceptable(const StandardFormLP& sf,
                                     const Eigen::VectorXd& x_std,
                                     double tol) {
  const int m = static_cast<int>(sf.A.rows());
  const int n = static_cast<int>(sf.A.cols());
  if (x_std.size() != n) return false;
  const bool has_row_scale = sf.row_scale.size() == m;
  const bool has_col_scale = sf.col_scale.size() == n;
  double viol = 0.0;
  double scale = 1.0;
  double max_row_viol = 0.0;
  double max_bound_viol = 0.0;
  double max_artificial = 0.0;
  // Scaled system: (D_r A D_c) y = D_r b, so the original-units residual and
  // RHS are the scaled ones divided by the row factor.
  const Eigen::VectorXd r = sf.A * x_std - sf.b;
  for (int i = 0; i < m; ++i) {
    const double rs = has_row_scale ? sf.row_scale[i] : 1.0;
    max_row_viol = std::max(max_row_viol, std::abs(r[i]) / rs);
    viol = std::max(viol, max_row_viol);
    scale = std::max(scale, std::abs(sf.b[i]) / rs);
  }
  // x_orig = D_c y: bound violations convert with the column factor.
  const bool has_ub = sf.var_ub.size() == n;
  for (int j = 0; j < n; ++j) {
    const double cs = has_col_scale ? sf.col_scale[j] : 1.0;
    if (x_std[j] < 0.0)
      max_bound_viol = std::max(max_bound_viol, -x_std[j] * cs);
    if (has_ub && std::isfinite(sf.var_ub[j]) && x_std[j] > sf.var_ub[j]) {
      max_bound_viol =
          std::max(max_bound_viol, (x_std[j] - sf.var_ub[j]) * cs);
    }
    viol = std::max(viol, max_bound_viol);
  }
  for (int art : sf.row_to_artificial_col) {
    if (art >= 0 && art < n)
      max_artificial = std::max(max_artificial, std::abs(x_std[art]));
  }
  const bool acceptable =
      max_artificial <= tol * scale && viol <= tol * scale;
  if (!acceptable && std::getenv("MIPSOLVERS_SF_RESIDUAL_DIAG")) {
    std::fprintf(stderr,
                 "[SF-RESIDUAL] rows=%.6e bounds=%.6e artificial=%.6e "
                 "scale=%.6e tol=%.6e\n",
                 max_row_viol, max_bound_viol, max_artificial, scale, tol);
    for (int j = 0; j < n; ++j) {
      if (!std::isfinite(x_std[j])) {
        std::fprintf(stderr,
                     "[SF-RESIDUAL] nonfinite column=%d value=%g upper=%g\n",
                     j, x_std[j], has_ub ? sf.var_ub[j] : 0.0);
        break;
      }
    }
  }
  return acceptable;
}

SimplexResult solve_lp_from_sf(const StandardFormLP& sf,
                               const SimplexOptions& input_opt,
                               const SimplexBasis* basis_hint) {
  SimplexResult res = solve_lp_from_sf_impl(sf, input_opt, basis_hint);
  if (res.result.stats.success &&
      (!std::isfinite(res.result.stats.objective) ||
       !sf_solution_residual_acceptable(sf, res.x_std,
                                        input_opt.escalation_residual_tol))) {
    res.result.stats.success = false;
    res.result.stats.status = "Residual audit rejected";
  }
  if (input_opt.retain_standard_form) res.form = sf;
  return res;
}

}  // namespace mipsolvers::engine
