/// @file bc_cuts_xtab_a.hpp
/// @brief Transformed-tableau (xtab) cut subsystem, part A (bc_cuts.cpp split).
/// Internal header included only by bc_cuts_transformed.cpp via bc_cuts_xtab_b.hpp.

#pragma once

#include "bc_cuts_common.hpp"

namespace mipsolvers::engine::detail {
namespace {

double fast_floor_xtab(double x) {
  // Guard UB: static_cast<int64_t> is undefined for values outside the int64
  // range (~±9.22e18) and for inf/NaN.  For those rare inputs fall back to the
  // standard library floor which is always well-defined.
  constexpr double kSafeMax = 4503599627370496.0;  // 2^52: every integer here
                                                    // is exactly representable
  if (std::abs(x) >= kSafeMax || !std::isfinite(x)) {
    return std::floor(x);
  }
  const auto ix = static_cast<int64_t>(x);
  return static_cast<double>(ix - (x < static_cast<double>(ix)));
}

double pow2_scale_for_max_abs(double max_abs) {
  if (!(max_abs > 0.0) || !std::isfinite(max_abs)) {
    return 1.0;
  }
  int exp_shift = 0;
  std::frexp(max_abs, &exp_shift);
  exp_shift = std::min(10, -exp_shift);
  return std::ldexp(1.0, exp_shift);
}

double xtab_nearest_integer(double value) {
  return static_cast<double>(HighsIntegers::nearestInteger(value));
}

double xtab_integral_scale(const std::vector<double>& values,
                           double feastol,
                           double epsilon) {
  return HighsIntegers::integralScale(values, feastol, epsilon);
}

double xtab_fractionality_distance(double value) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::abs(value - std::round(value));
}

enum class XTabBoundType : unsigned char {
  SimpleLb = 0,
  SimpleUb = 1,
  VariableLb = 2,
  VariableUb = 3,
};

struct XTabVarBoundExpr {
  bool valid{false};
  int trigger_col{-1};
  double constant{0.0};    // In transformed-space of target column.
  double coef{0.0};        // Coefficient on transformed trigger column.
  double dist{std::numeric_limits<double>::infinity()};
  double scaled_dist{std::numeric_limits<double>::infinity()};
  double min_value{std::numeric_limits<double>::infinity()};
  double max_value{-std::numeric_limits<double>::infinity()};
  double lift{0.0};
  bool integer_target{false};
};

struct XTabSparseVectorSum {
  std::vector<double> values;
  std::vector<int> nonzeros;

  explicit XTabSparseVectorSum(int dim = 0) { reset_dim(dim); }

  void reset_dim(int dim) {
    values.assign(static_cast<std::size_t>(std::max(0, dim)), 0.0);
    nonzeros.clear();
    nonzeros.reserve(static_cast<std::size_t>(std::max(0, dim)));
  }

  bool empty() const { return nonzeros.empty(); }

  void add(int index, double value) {
    if (index < 0 || index >= static_cast<int>(values.size())) return;
    double& current = values[static_cast<std::size_t>(index)];
    if (current != 0.0) {
      current += value;
    } else {
      current = value;
      nonzeros.push_back(index);
    }
    if (current == 0.0) {
      current = std::numeric_limits<double>::min();
    }
  }

  template <typename IsZero>
  void cleanup(IsZero&& is_zero) {
    int num_nz = static_cast<int>(nonzeros.size());
    for (int i = num_nz - 1; i >= 0; --i) {
      const int col = nonzeros[static_cast<std::size_t>(i)];
      const double value = values[static_cast<std::size_t>(col)];
      if (!is_zero(col, value)) continue;
      values[static_cast<std::size_t>(col)] = 0.0;
      --num_nz;
      std::swap(nonzeros[static_cast<std::size_t>(num_nz)],
                nonzeros[static_cast<std::size_t>(i)]);
    }
    nonzeros.resize(static_cast<std::size_t>(num_nz));
  }
};

struct XTabTransformContext {
  std::vector<XTabVarBoundExpr> best_vlb;
  std::vector<XTabVarBoundExpr> best_vub;
  std::uint64_t num_vlb{0};
  std::uint64_t num_vub{0};
  std::uint64_t num_integral_vlb{0};
  std::uint64_t num_integral_vub{0};
};

struct XTabRow {
  std::vector<int> inds;
  std::vector<double> vals;
  std::vector<double> upper;
  std::vector<double> solval;
  std::vector<XTabBoundType> bound_type;
  std::vector<XTabVarBoundExpr> varbound_expr;
  std::vector<unsigned char> complemented;
  std::vector<unsigned char> is_integral;
  double rhs{0.0};
  double initial_scale{1.0};
  bool integral_support{false};
  bool integral_coefficients{false};
};

struct XTabCmirTrace {
  int integer_terms{0};
  int continuous_terms{0};
  int initial_delta_count{0};
  int tested_delta_count{0};
  double continuous_contribution{0.0};
  double continuous_norm_sq{0.0};
  double max_abs_delta{0.0};
  double best_delta{-1.0};
  double best_efficacy{-std::numeric_limits<double>::infinity()};
  double final_f0{std::numeric_limits<double>::quiet_NaN()};
  double final_rhs{std::numeric_limits<double>::quiet_NaN()};
  bool accepted{false};
  bool lifted_accepted{false};
};

struct XTabLiftedCoverState {
  std::vector<int> cover;
  HighsCDouble cover_weight{0.0};
  HighsCDouble lambda{0.0};
};

struct XTabCutgenRandom {
  std::optional<HighsRandom> randgen;

  explicit XTabCutgenRandom(std::optional<std::uint64_t> seed) {
    if (seed.has_value()) {
      randgen.emplace(static_cast<HighsUInt>(*seed));
    }
  }

  int next_tiebreaker(int fallback) {
    if (!randgen.has_value()) return fallback;
    return static_cast<int>(randgen->integer());
  }
};

std::uint64_t xtab_highs_pair_hash(std::uint32_t a,
                                   std::uint32_t b,
                                   int k);

struct XTabSourceContext {
  int n_original{0};
  int n_rows{0};
  int dim{0};
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  Eigen::VectorXd solution;
  std::vector<unsigned char> integral;
  bool valid{false};

  bool is_logical_row_slack(int col) const {
    return col >= n_original && col < dim;
  }

  int row_from_logical_slack(int col) const {
    return col - n_original;
  }
};

double xtab_row_scale_or_one(const StandardFormLP& sf, int row) {
  if (row >= 0 && row < sf.row_scale.size()) {
    const double scale = sf.row_scale[row];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

double xtab_col_scale_or_one(const StandardFormLP& sf, int col) {
  if (col >= 0 && col < sf.col_scale.size()) {
    const double scale = sf.col_scale[col];
    if (std::isfinite(scale) && std::abs(scale) > 1e-12) return scale;
  }
  return 1.0;
}

[[maybe_unused]] double xtab_source_row_coeff(const StandardFormLP& sf, int row, int col) {
  if (row < 0 || row >= sf.A_row.rows() || col < 0 ||
      col >= sf.n_original) {
    return 0.0;
  }
  const double scaled = sf.A_row.coeff(row, col);
  if (std::abs(scaled) <= 1e-15) return 0.0;
  return scaled / (xtab_row_scale_or_one(sf, row) *
                   xtab_col_scale_or_one(sf, col));
}

bool xtab_source_row_has_highs_order(const StandardFormLP& sf, int row) {
  if (row < 0 || row >= static_cast<int>(sf.source_highs_row.size())) {
    return false;
  }
  const int highs_row = sf.source_highs_row[static_cast<std::size_t>(row)];
  if (highs_row < 0 ||
      highs_row + 1 > static_cast<int>(sf.source_row_start.size()) - 1) {
    return false;
  }
  const int start = sf.source_row_start[static_cast<std::size_t>(highs_row)];
  const int end = sf.source_row_start[static_cast<std::size_t>(highs_row + 1)];
  return start >= 0 && end >= start &&
         end <= static_cast<int>(sf.source_row_index.size()) &&
         end <= static_cast<int>(sf.source_row_value.size());
}

template <typename Func>
void xtab_visit_source_row_terms(const StandardFormLP& sf,
                                 int row,
                                 Func&& func) {
  if (xtab_source_row_has_highs_order(sf, row)) {
    const int highs_row = sf.source_highs_row[static_cast<std::size_t>(row)];
    const int start = sf.source_row_start[static_cast<std::size_t>(highs_row)];
    const int end = sf.source_row_start[static_cast<std::size_t>(highs_row + 1)];
    for (int p = start; p < end; ++p) {
      const int col = sf.source_row_index[static_cast<std::size_t>(p)];
      if (col < 0 || col >= sf.n_original) continue;
      const double value = sf.source_row_value[static_cast<std::size_t>(p)];
      if (std::abs(value) <= 1e-15) continue;
      func(col, value);
    }
    return;
  }

  const double row_scale_inv = 1.0 / xtab_row_scale_or_one(sf, row);
  for (StandardRowMatrix::InnerIterator it(sf.A_row,
                                                                      row);
       it; ++it) {
    const int col = static_cast<int>(it.col());
    if (col < 0 || col >= sf.n_original) continue;
    const double scaled = it.value();
    if (std::abs(scaled) <= 1e-15) continue;
    const double value = scaled * row_scale_inv / xtab_col_scale_or_one(sf, col);
    if (std::abs(value) <= 1e-15) continue;
    func(col, value);
  }
}

double xtab_source_row_side(const StandardFormLP& sf, int row) {
  if (row < 0 || row >= sf.A_row.rows()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const int sign = row < static_cast<int>(sf.row_sign.size())
                       ? sf.row_sign[static_cast<std::size_t>(row)]
                       : 1;
  if (sf.row_rhs_value.size() == sf.A_row.rows()) {
    return static_cast<double>(sign) * sf.row_rhs_value[row];
  }
  return sf.b[row] / xtab_row_scale_or_one(sf, row);
}

int xtab_row_trace_limit() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_ROW_TRACE");
  if (env == nullptr) return 0;
  if (env[0] == '\0') return 20;
  if (std::string(env) == "all") return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return 20;
  if (value <= 0) return 0;
  return static_cast<int>(std::min<long>(value, 1000000));
}

int xtab_row_trace_terms() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_ROW_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 12;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 12;
  return static_cast<int>(std::min<long>(value, 200));
}

std::uint64_t xtab_row_trace_id_bound(const char* name,
                                      std::uint64_t default_value) {
  const char* env = bc_env_options().value(name);
  if (env == nullptr || env[0] == '\0') return default_value;
  char* end = nullptr;
  const unsigned long long value = std::strtoull(env, &end, 10);
  if (end == env) return default_value;
  return static_cast<std::uint64_t>(value);
}

int xtab_transform_trace_col() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_TRANSFORM_TRACE_COL");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

int xtab_varbound_trace_col() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_VB_TRACE_COL");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

int xtab_modk_system_trace_limit() {
  const char* env = bc_env_options().value("MIPSOLVERS_XMODK_SYSTEM_TRACE");
  if (env == nullptr || env[0] == '\0') return 0;
  if (std::string(env) == "all") return 1000000000;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return 40;
  return value > 0 ? static_cast<int>(std::min<long>(value, 1000000)) : 0;
}

int xtab_modk_system_trace_terms() {
  const char* env = bc_env_options().value("MIPSOLVERS_XMODK_SYSTEM_TRACE_TERMS");
  if (env == nullptr || env[0] == '\0') return 16;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env || value <= 0) return 16;
  return static_cast<int>(std::min<long>(value, 200));
}

int xtab_modk_transform_trace_row() {
  const char* env = bc_env_options().value("MIPSOLVERS_XMODK_TRANSFORM_ROW");
  if (env == nullptr || env[0] == '\0') return -1;
  char* end = nullptr;
  const long value = std::strtol(env, &end, 10);
  if (end == env) return -1;
  return static_cast<int>(value);
}

bool xtab_row_trace_family_enabled(const char* family) {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_ROW_TRACE_FAMILY");
  if (env == nullptr || env[0] == '\0' || std::string(env) == "all") {
    return true;
  }
  return family != nullptr && std::string(env) == family;
}

bool xtab_row_trace_meta_enabled() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_ROW_TRACE_META");
  if (env == nullptr) return false;
  return env[0] != '\0' && std::string(env) != "0";
}

bool xtab_row_trace_basis_enabled() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_ROW_TRACE_BASIS");
  if (env == nullptr) return false;
  return env[0] != '\0' && std::string(env) != "0";
}

std::uint64_t xtab_highs_pair_hash(std::uint32_t a,
                                   std::uint32_t b,
                                   int k) {
  static constexpr std::uint64_t c[] = {
      std::uint64_t{0xc8497d2a400d9551},
      std::uint64_t{0x80c8963be3e4c2f3},
      std::uint64_t{0x042d8680e260ae5b},
      std::uint64_t{0x8a183895eeac1536},
  };
  const int idx = 2 * k;
  return (static_cast<std::uint64_t>(a) + c[idx]) *
         (static_cast<std::uint64_t>(b) + c[idx + 1]);
}

std::uint64_t xtab_highs_hash_i64(std::int64_t value) {
  const std::uint64_t bits = static_cast<std::uint64_t>(value);
  const auto lo = static_cast<std::uint32_t>(bits & 0xffffffffu);
  const auto hi = static_cast<std::uint32_t>(bits >> 32);
  return xtab_highs_pair_hash(lo, hi, 1) ^
         (xtab_highs_pair_hash(lo, hi, 0) >> 32);
}

std::uint64_t xtab_modk_hash_mix(std::uint64_t h, std::uint64_t v) {
  v ^= v >> 33;
  v *= std::uint64_t{0xff51afd7ed558ccd};
  v ^= v >> 33;
  v *= std::uint64_t{0xc4ceb9fe1a85ec53};
  v ^= v >> 33;
  return h ^ (v + std::uint64_t{0x9e3779b97f4a7c15} + (h << 6) + (h >> 2));
}

std::uint64_t xtab_modk_system_hash(
    const std::vector<std::int64_t>& values,
    const std::vector<int>& indices,
    const std::vector<int>& starts) {
  std::uint64_t h = std::uint64_t{0x48494748534d4f44};
  h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(values.size()));
  h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(indices.size()));
  h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(starts.size()));
  for (int start : starts) {
    h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(start));
  }
  for (int index : indices) {
    h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(index));
  }
  for (std::int64_t value : values) {
    h = xtab_modk_hash_mix(h, static_cast<std::uint64_t>(value));
  }
  return h;
}

bool xtab_claim_row_trace(std::uint64_t& id, const char* family) {
  if (!xtab_row_trace_family_enabled(family)) return false;
  const int limit = xtab_row_trace_limit();
  if (limit == 0) return false;
  static std::atomic<std::uint64_t> next_id{1};
  id = next_id.fetch_add(1, std::memory_order_relaxed);
  const std::uint64_t min_id =
      xtab_row_trace_id_bound("MIPSOLVERS_XTAB_ROW_TRACE_MIN_ID", 1);
  const std::uint64_t max_id = xtab_row_trace_id_bound(
      "MIPSOLVERS_XTAB_ROW_TRACE_MAX_ID",
      std::numeric_limits<std::uint64_t>::max());
  if (id < min_id || id > max_id) return false;
  return limit < 0 || id <= static_cast<std::uint64_t>(limit);
}

const char* xtab_bound_type_name(XTabBoundType type) {
  switch (type) {
    case XTabBoundType::SimpleLb:
      return "simple_lb";
    case XTabBoundType::SimpleUb:
      return "simple_ub";
    case XTabBoundType::VariableLb:
      return "vlb";
    case XTabBoundType::VariableUb:
      return "vub";
  }
  return "unknown";
}

struct XTabSparseStats {
  int nnz{0};
  double l1{0.0};
  double max_abs{0.0};
  std::string sample;
};

XTabSparseStats xtab_sparse_stats(const Eigen::VectorXd& coeff,
                                  const std::vector<int>* active_cols,
                                  int max_terms) {
  XTabSparseStats stats;
  fmt::memory_buffer buffer;
  int emitted = 0;
  auto visit = [&](int col) {
    if (col < 0 || col >= coeff.size()) return;
    const double val = coeff[col];
    if (std::abs(val) <= 1e-12) return;
    ++stats.nnz;
    stats.l1 += std::abs(val);
    stats.max_abs = std::max(stats.max_abs, std::abs(val));
    if (emitted < max_terms) {
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{}:{:.6g}", col, val);
      ++emitted;
    }
  };
  if (active_cols != nullptr) {
    for (int col : *active_cols) visit(col);
  } else {
    for (int col = 0; col < coeff.size(); ++col) visit(col);
  }
  if (stats.nnz > emitted) {
    fmt::format_to(std::back_inserter(buffer), ",...");
  }
  stats.sample = fmt::to_string(buffer);
  return stats;
}

struct XTabRowStats {
  int nnz{0};
  int simple_lb{0};
  int simple_ub{0};
  int variable_lb{0};
  int variable_ub{0};
  int integral{0};
  int continuous{0};
  int logical{0};
  int complemented{0};
  double l1{0.0};
  double max_abs{0.0};
  double activity{0.0};
  double norm{0.0};
  std::string sample;
};

XTabRowStats xtab_row_stats(const XTabRow& row,
                            const XTabSourceContext& source_context,
                            int max_terms) {
  XTabRowStats stats;
  stats.nnz = static_cast<int>(row.inds.size());
  fmt::memory_buffer buffer;
  int emitted = 0;
  double norm_sq = 0.0;
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    const int col = row.inds[pos];
    const double val = row.vals[pos];
    stats.l1 += std::abs(val);
    stats.max_abs = std::max(stats.max_abs, std::abs(val));
    stats.activity += val * row.solval[pos];
    norm_sq += val * val;
    switch (row.bound_type[pos]) {
      case XTabBoundType::SimpleLb:
        ++stats.simple_lb;
        break;
      case XTabBoundType::SimpleUb:
        ++stats.simple_ub;
        break;
      case XTabBoundType::VariableLb:
        ++stats.variable_lb;
        break;
      case XTabBoundType::VariableUb:
        ++stats.variable_ub;
        break;
    }
    if (row.is_integral[pos] != 0) {
      ++stats.integral;
    } else {
      ++stats.continuous;
    }
    if (source_context.is_logical_row_slack(col)) ++stats.logical;
    if (row.complemented[pos] != 0) ++stats.complemented;
    if (emitted < max_terms) {
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{}:{:.17g}:{}:s{:.17g}",
                     col, val, xtab_bound_type_name(row.bound_type[pos]),
                     row.solval[pos]);
      ++emitted;
    }
  }
  if (stats.nnz > emitted) {
    fmt::format_to(std::back_inserter(buffer), ",...");
  }
  stats.norm = std::sqrt(norm_sq);
  stats.sample = fmt::to_string(buffer);
  return stats;
}

void xtab_trace_source_row(std::uint64_t id,
                           const char* family,
                           const char* stage,
                           const Eigen::VectorXd& coeff,
                           double rhs,
                           const std::vector<int>* active_cols) {
  const XTabSparseStats stats =
      xtab_sparse_stats(coeff, active_cols, xtab_row_trace_terms());
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={} rhs={:.17g} nnz={} "
             "l1={:.6g} max={:.6g} baseRowIndsVals=[{}]\n",
             id, family, stage, rhs, stats.nnz, stats.l1, stats.max_abs,
             stats.sample);
}

std::string xtab_source_row_signature(const SimplexResult& simplex,
                                      int row,
                                      int max_terms) {
  fmt::memory_buffer buffer;
  int emitted = 0;
  int nnz = 0;
  xtab_visit_source_row_terms(simplex.form, row, [&](int col, double val) {
    if (std::abs(val) <= 1e-12) return;
    ++nnz;
    if (emitted < max_terms) {
      if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ",");
      fmt::format_to(std::back_inserter(buffer), "{}:{:.12g}", col, val);
      ++emitted;
    }
  });
  if (nnz > emitted) {
    fmt::format_to(std::back_inserter(buffer), ",...");
  }
  return fmt::to_string(buffer);
}

void xtab_trace_source_meta(std::uint64_t id,
                            const char* family,
                            const char* stage,
                            const SimplexResult& simplex,
                            const XTabSourceContext& source_context,
                            const Eigen::VectorXd& coeff,
                            const std::vector<int>* active_cols) {
  if (!xtab_row_trace_meta_enabled() || !source_context.valid) return;
  const int max_terms = xtab_row_trace_terms();
  fmt::memory_buffer buffer;
  int emitted = 0;
  auto visit = [&](int col) {
    if (col < source_context.n_original || col >= source_context.dim ||
        col >= coeff.size()) {
      return;
    }
    const double val = coeff[col];
    if (std::abs(val) <= 1e-12) return;
    const int row = source_context.row_from_logical_slack(col);
    if (row < 0 || row >= source_context.n_rows) return;
    if (emitted > 0) fmt::format_to(std::back_inserter(buffer), ";");
    fmt::format_to(std::back_inserter(buffer),
                   "{}:{:.12g}>row{}:lb{:.12g}:ub{:.12g}:act{:.12g}:sig[{}]",
                   col, val, row, source_context.lower[col],
                   source_context.upper[col], source_context.solution[col],
                   xtab_source_row_signature(simplex, row, max_terms));
    ++emitted;
  };
  if (active_cols != nullptr) {
    for (int col : *active_cols) {
      if (emitted >= max_terms) break;
      visit(col);
    }
  } else {
    for (int col = source_context.n_original;
         col < source_context.dim && emitted < max_terms; ++col) {
      visit(col);
    }
  }
  if (emitted == 0) return;
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={}_meta slacks=[{}]\n",
             id, family, stage, fmt::to_string(buffer));
}

void xtab_trace_row(std::uint64_t id,
                    const char* family,
                    const char* stage,
                    const XTabRow& row,
                    const XTabSourceContext& source_context,
                    bool integers_positive) {
  const XTabRowStats stats =
      xtab_row_stats(row, source_context, xtab_row_trace_terms());
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={} rhs={:.12g} nnz={} "
             "types=slb{}:sub{}:vlb{}:vub{} int={} cont={} log={} "
             "comp={} intPos={} activity={:.17g} viol={:.17g} "
             "norm={:.17g} l1={:.17g} max={:.17g} row=[{}]\n",
             id, family, stage, row.rhs, stats.nnz, stats.simple_lb,
             stats.simple_ub, stats.variable_lb, stats.variable_ub,
             stats.integral, stats.continuous, stats.logical,
             stats.complemented, integers_positive ? 1 : 0, stats.activity,
             stats.activity - row.rhs, stats.norm, stats.l1, stats.max_abs,
             stats.sample);
}

void xtab_trace_original_row(std::uint64_t id,
                             const char* family,
                             const char* stage,
                             const Eigen::VectorXd& cut,
                             double rhs,
                             const Eigen::VectorXd& x) {
  const XTabSparseStats stats =
      xtab_sparse_stats(cut, nullptr, xtab_row_trace_terms());
  const double activity =
      cut.size() == x.size() ? cut.dot(x)
                             : std::numeric_limits<double>::quiet_NaN();
  const double norm = std::max(1e-12, cut.norm());
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage={} rhs={:.12g} nnz={} "
             "activity={:.12g} viol={:.12g} eff={:.6g} l1={:.6g} "
             "max={:.6g} row=[{}]\n",
             id, family, stage, rhs, stats.nnz, activity, activity - rhs,
             (activity - rhs) / norm, stats.l1, stats.max_abs, stats.sample);
}

bool xtab_cmir_delta_trace_enabled() {
  const char* env = bc_env_options().value("MIPSOLVERS_XTAB_CMIR_DELTA_TRACE");
  return env != nullptr && env[0] != '\0' && std::string(env) != "0";
}

void xtab_trace_cmir_delta(std::uint64_t id,
                           const char* family,
                           const char* phase,
                           double delta,
                           double scale,
                           double down_rhs,
                           double f0,
                           double violation,
                           double norm_sq,
                           double efficacy,
                           double best_before) {
  fmt::print(stderr,
             "[B&C-XROW] id={} family={} stage=cmir_delta phase={} "
             "delta={:.17g} scale={:.17g} downRhs={:.17g} f0={:.17g} "
             "viol={:.17g} normSq={:.17g} eff={:.17g} bestBefore={:.17g}\n",
             id, family == nullptr ? "" : family, phase, delta, scale,
             down_rhs, f0, violation, norm_sq, efficacy, best_before);
}

void xtab_trace_fail(std::uint64_t id,
                     const char* family,
                     const char* stage,
                     const char* reason) {
  fmt::print(stderr, "[B&C-XROW] id={} family={} stage={} fail={}\n", id,
             family, stage, reason);
}

enum class XTabRejectReason : unsigned char {
  None = 0,
  Transform,
  Preprocess,
  Cmir,
  EmptyCut,
  Untransform,
  NotViolated,
};

struct XTabSourceDiag {
  std::uint64_t basis_rows{0};
  std::uint64_t integer_basic_original{0};
  std::uint64_t integer_basic_aux{0};
  std::uint64_t fractional_basic_original{0};
  std::uint64_t fractional_basic_aux{0};
  std::uint64_t btran_rows{0};
  std::uint64_t row_ep_rows{0};
  std::uint64_t aggregate_ok{0};
  std::uint64_t transform_fail{0};
  std::uint64_t preprocess_fail{0};
  std::uint64_t cmir_fail{0};
  std::uint64_t empty_fail{0};
  std::uint64_t untransform_fail{0};
  std::uint64_t not_violated{0};
  std::uint64_t gate_calls{0};
  std::uint64_t gate_transform_ok{0};
  std::uint64_t gate_preprocess_ok{0};
  std::uint64_t gate_cmir_ok{0};
  std::uint64_t gate_untransform_ok{0};
  std::uint64_t gate_violation_ok{0};
  std::uint64_t generated{0};
  std::uint64_t filtered_density{0};
  std::uint64_t filtered_efficacy{0};
  std::uint64_t filtered_activity{0};
  std::uint64_t filtered_binary_support{0};
  std::uint64_t filtered_parallel{0};
  std::uint64_t selected{0};
  std::uint64_t vb_substitutions{0};
  std::uint64_t vb_trigger_terms{0};
};

void xtab_record_reject(XTabSourceDiag* diag, XTabRejectReason reason) {
  if (diag == nullptr) return;
  switch (reason) {
    case XTabRejectReason::Transform:
      ++diag->transform_fail;
      break;
    case XTabRejectReason::Preprocess:
      ++diag->preprocess_fail;
      break;
    case XTabRejectReason::Cmir:
      ++diag->cmir_fail;
      break;
    case XTabRejectReason::EmptyCut:
      ++diag->empty_fail;
      break;
    case XTabRejectReason::Untransform:
      ++diag->untransform_fail;
      break;
    case XTabRejectReason::NotViolated:
      ++diag->not_violated;
      break;
    case XTabRejectReason::None:
      break;
  }
}

void maybe_print_xtab_diag(const char* family, const XTabSourceDiag& diag) {
  if (bc_env_options().value("MIPSOLVERS_XTAB_DIAG") == nullptr) return;
  fmt::print(stderr,
             "[B&C-XTAB-DIAG] family={} basis={} intOrig={} intAux={} "
             "fracOrig={} fracAux={} btran={} rowEp={} agg={} "
             "reject=T{} P{} C{} E{} U{} V{} gen={} "
             "gate={}>{}>{}>{}>{}>{} filt=d{} e{} a{} b{} p{} sel={} "
             "vb={}/{}\n",
             family, diag.basis_rows, diag.integer_basic_original,
             diag.integer_basic_aux, diag.fractional_basic_original,
             diag.fractional_basic_aux, diag.btran_rows, diag.row_ep_rows,
             diag.aggregate_ok, diag.transform_fail, diag.preprocess_fail,
             diag.cmir_fail, diag.empty_fail, diag.untransform_fail,
             diag.not_violated, diag.generated, diag.gate_calls,
             diag.gate_transform_ok, diag.gate_preprocess_ok,
             diag.gate_cmir_ok, diag.gate_untransform_ok,
             diag.gate_violation_ok, diag.filtered_density,
             diag.filtered_efficacy, diag.filtered_activity,
             diag.filtered_binary_support, diag.filtered_parallel,
             diag.selected, diag.vb_substitutions, diag.vb_trigger_terms);
}

struct XTabCandidateCut {
  XTabCandidateCut(Eigen::VectorXd dense_coeff,
                   double rhs_value,
                   double score_value,
                   double efficacy_value,
                   double norm_value,
                   int nnz_value,
                   std::uint64_t trace_id = 0)
      : coeff(dense_to_sparse_cut(dense_coeff)),
        rhs(rhs_value),
        score(score_value),
        efficacy(efficacy_value),
        norm(norm_value),
        nnz(nnz_value),
        source_trace_id(trace_id) {}

  Eigen::SparseVector<double> coeff;
  double rhs{0.0};
  double score{0.0};
  double efficacy{0.0};
  double norm{0.0};
  int nnz{0};
  std::uint64_t source_trace_id{0};
};

PoolCut xtab_candidate_to_pool_cut(const XTabCandidateCut& cand) {
  Eigen::SparseVector<double> sparse = cand.coeff;
  const std::size_t hash = sparse_cut_hash(sparse);
  PoolCut cut{std::move(sparse), cand.rhs, 0, cand.efficacy, cand.norm, hash};
  cut.source_trace_id = cand.source_trace_id;
  return cut;
}

void xtab_flip_complementation(XTabRow& row, int k) {
  row.complemented[static_cast<std::size_t>(k)] =
      1 - row.complemented[static_cast<std::size_t>(k)];
  row.solval[static_cast<std::size_t>(k)] =
      row.upper[static_cast<std::size_t>(k)] -
      row.solval[static_cast<std::size_t>(k)];
  row.rhs -= row.upper[static_cast<std::size_t>(k)] *
             row.vals[static_cast<std::size_t>(k)];
  row.vals[static_cast<std::size_t>(k)] =
      -row.vals[static_cast<std::size_t>(k)];
}

void xtab_remove_complementation(XTabRow& row) {
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    if (row.complemented[static_cast<std::size_t>(k)] != 0) {
      xtab_flip_complementation(row, k);
    }
  }
}

void xtab_update_violation_and_norm(const XTabRow& row,
                                    int k,
                                    double coeff,
                                    double& violation,
                                    double& norm_sq,
                                    double feastol) {
  violation += coeff * row.solval[static_cast<std::size_t>(k)];
  if (coeff > 0.0 &&
      row.solval[static_cast<std::size_t>(k)] <= feastol) {
    return;
  }
  if (coeff < 0.0 &&
      row.solval[static_cast<std::size_t>(k)] >=
          row.upper[static_cast<std::size_t>(k)] - feastol) {
    return;
  }
  norm_sq += coeff * coeff;
}

bool xtab_is_integral_value(double value, double feastol) {
  return std::abs(value - std::round(value)) <= feastol;
}

bool xtab_determine_cover(const XTabRow& row,
                          double feastol,
                          XTabCutgenRandom* cutgen_random,
                          int fallback_tiebreaker,
                          XTabLiftedCoverState& state) {
  if (row.rhs <= 10.0 * feastol) return false;

  state.cover.clear();
  state.cover.reserve(row.inds.size());
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    if (row.is_integral[static_cast<std::size_t>(k)] == 0) continue;
    if (row.solval[static_cast<std::size_t>(k)] <= feastol) continue;
    state.cover.push_back(k);
  }

  const int max_cover_size = static_cast<int>(state.cover.size());
  int cover_size = 0;
  state.cover_weight = 0.0;
  const int random_tiebreaker =
      cutgen_random != nullptr
          ? cutgen_random->next_tiebreaker(fallback_tiebreaker)
          : fallback_tiebreaker;

  auto ub = [&](int k) { return row.upper[static_cast<std::size_t>(k)]; };
  auto sol = [&](int k) { return row.solval[static_cast<std::size_t>(k)]; };
  auto val = [&](int k) { return row.vals[static_cast<std::size_t>(k)]; };

  auto mid = std::partition(state.cover.begin(), state.cover.end(), [&](int k) {
    return sol(k) >= ub(k) - feastol;
  });
  cover_size = static_cast<int>(mid - state.cover.begin());
  for (int i = 0; i < cover_size; ++i) {
    const int k = state.cover[static_cast<std::size_t>(i)];
    state.cover_weight += val(k) * ub(k);
  }

  std::sort(mid, state.cover.end(), [&](int a, int b) {
    if (ub(a) < 1.5 && ub(b) > 1.5) return true;
    if (ub(a) > 1.5 && ub(b) < 1.5) return false;

    const double contrib_a = sol(a) * val(a);
    const double contrib_b = sol(b) * val(b);
    if (contrib_a > contrib_b + feastol) return true;
    if (contrib_a < contrib_b - feastol) return false;
    if (std::abs(val(a) - val(b)) <= feastol) {
      return xtab_highs_pair_hash(
                 static_cast<std::uint32_t>(row.inds[static_cast<std::size_t>(a)]),
                 static_cast<std::uint32_t>(random_tiebreaker), 0) >
             xtab_highs_pair_hash(
                 static_cast<std::uint32_t>(row.inds[static_cast<std::size_t>(b)]),
                 static_cast<std::uint32_t>(random_tiebreaker), 0);
    }
    return val(a) > val(b);
  });

  const double min_lambda =
      std::max(10.0 * feastol, feastol * std::abs(row.rhs));
  for (; cover_size != max_cover_size; ++cover_size) {
    const double lambda = double(state.cover_weight - row.rhs);
    if (lambda > min_lambda) break;

    const int k = state.cover[static_cast<std::size_t>(cover_size)];
    state.cover_weight += val(k) * ub(k);
  }
  if (cover_size == 0) return false;

  state.cover_weight.renormalize();
  state.lambda = state.cover_weight - row.rhs;
  if (double(state.lambda) <= min_lambda) return false;

  state.cover.resize(static_cast<std::size_t>(cover_size));
  return true;
}

void xtab_separate_lifted_knapsack_cover(XTabRow& row,
                                         const XTabLiftedCoverState& state,
                                         double feastol,
                                         double epsilon) {
  const int cover_size = static_cast<int>(state.cover.size());
  std::vector<int> cover = state.cover;
  std::vector<double> partial(static_cast<std::size_t>(cover_size), 0.0);
  std::vector<signed char> cover_flag(row.inds.size(), 0);

  std::sort(cover.begin(), cover.end(), [&](int a, int b) {
    return row.vals[static_cast<std::size_t>(a)] >
           row.vals[static_cast<std::size_t>(b)];
  });

  HighsCDouble abar_tmp = row.vals[static_cast<std::size_t>(cover.front())];
  HighsCDouble sigma = state.lambda;
  for (int i = 1; i != cover_size; ++i) {
    const HighsCDouble delta =
        abar_tmp - row.vals[static_cast<std::size_t>(cover[i])];
    const HighsCDouble kdelta = static_cast<double>(i) * delta;
    if (double(kdelta) < double(sigma)) {
      abar_tmp = row.vals[static_cast<std::size_t>(cover[i])];
      sigma -= kdelta;
    } else {
      abar_tmp -= sigma * (1.0 / static_cast<double>(i));
      sigma = 0.0;
      break;
    }
  }

  if (double(sigma) > 0.0) {
    abar_tmp = HighsCDouble(row.rhs) / static_cast<double>(cover_size);
  }
  const double abar = double(abar_tmp);

  HighsCDouble sum = 0.0;
  int cplus_size = 0;
  for (int i = 0; i != cover_size; ++i) {
    const int k = cover[static_cast<std::size_t>(i)];
    sum += std::min(abar, row.vals[static_cast<std::size_t>(k)]);
    partial[static_cast<std::size_t>(i)] = double(sum);

    if (row.vals[static_cast<std::size_t>(k)] > abar + feastol) {
      ++cplus_size;
      cover_flag[static_cast<std::size_t>(k)] = 1;
    } else {
      cover_flag[static_cast<std::size_t>(k)] = -1;
    }
  }

  bool half_integral = false;
  auto g = [&](double z) {
    const double hfrac = z / abar;
    double coef = 0.0;

    int h = static_cast<int>(std::floor(hfrac + 0.5));
    if (h != 0 && std::abs(hfrac - static_cast<double>(h)) *
                          std::max(1.0, abar) <= epsilon &&
        h <= cplus_size - 1) {
      half_integral = true;
      coef = 0.5;
    }

    h = std::max(h - 1, 0);
    for (; h < cover_size; ++h) {
      if (z <= partial[static_cast<std::size_t>(h)] + feastol) break;
    }

    return coef + static_cast<double>(h);
  };

  row.rhs = static_cast<double>(cover_size - 1);
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.vals[pos] == 0.0) continue;
    if (cover_flag[pos] == -1) {
      row.vals[pos] = 1.0;
    } else {
      row.vals[pos] = g(row.vals[pos]);
    }
  }

  if (half_integral) {
    row.rhs *= 2.0;
    for (double& v : row.vals) v *= 2.0;
  }

  row.integral_support = true;
  row.integral_coefficients = true;
}

bool xtab_separate_lifted_mixed_binary_cover(
    XTabRow& row,
    const XTabLiftedCoverState& state,
    double epsilon) {
  row.integral_support = false;
  row.integral_coefficients = false;

  const int cover_size = static_cast<int>(state.cover.size());
  if (cover_size == 0) return false;

  std::vector<int> cover = state.cover;
  std::vector<double> partial(static_cast<std::size_t>(cover_size), 0.0);
  std::vector<unsigned char> cover_flag(row.inds.size(), 0);
  for (int k : cover) cover_flag[static_cast<std::size_t>(k)] = 1;

  std::sort(cover.begin(), cover.end(), [&](int a, int b) {
    return row.vals[static_cast<std::size_t>(a)] >
           row.vals[static_cast<std::size_t>(b)];
  });

  double sum = 0.0;
  int p = cover_size;
  for (int i = 0; i != cover_size; ++i) {
    const int k = cover[static_cast<std::size_t>(i)];
    if (row.vals[static_cast<std::size_t>(k)] - double(state.lambda) <=
      epsilon) {
      p = i;
      break;
    }
    sum += row.vals[static_cast<std::size_t>(k)];
    partial[static_cast<std::size_t>(i)] = sum;
  }
  if (p == 0) return false;

  auto phi = [&](double a) {
    for (int i = 0; i < p; ++i) {
      if (a <= partial[static_cast<std::size_t>(i)] - double(state.lambda)) {
        return double(static_cast<double>(i) * state.lambda);
      }

      if (a <= partial[static_cast<std::size_t>(i)]) {
        return double(static_cast<double>(i + 1) * state.lambda +
                      (HighsCDouble(a) - partial[static_cast<std::size_t>(i)]));
      }
    }

    return double(static_cast<double>(p) * state.lambda +
                  (HighsCDouble(a) - partial[static_cast<std::size_t>(p - 1)]));
  };

  HighsCDouble rhs_acc = -state.lambda;
  row.integral_support = true;
  row.integral_coefficients = false;
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.is_integral[pos] == 0) {
      if (row.vals[pos] < 0.0) {
        row.integral_support = false;
      } else {
        row.vals[pos] = 0.0;
      }
      continue;
    }

    if (cover_flag[pos] != 0) {
      row.vals[pos] = std::min(row.vals[pos], double(state.lambda));
      rhs_acc += row.vals[pos];
    } else {
      row.vals[pos] = phi(row.vals[pos]);
    }
  }

  row.rhs = double(rhs_acc);
  return true;
}

bool xtab_separate_lifted_mixed_integer_cover(
    XTabRow& row,
    const XTabLiftedCoverState& state,
    double feastol,
    double epsilon) {
  (void)epsilon;
  row.integral_support = false;
  row.integral_coefficients = false;

  std::vector<int> cover = state.cover;
  const int cover_size = static_cast<int>(cover.size());
  std::vector<unsigned char> cover_flag(row.inds.size(), 0);
  for (int k : cover) cover_flag[static_cast<std::size_t>(k)] = 1;

  std::sort(cover.begin(), cover.end(), [&](int a, int b) {
    return row.vals[static_cast<std::size_t>(a)] >
           row.vals[static_cast<std::size_t>(b)];
  });

  std::vector<double> a(static_cast<std::size_t>(cover_size), 0.0);
  std::vector<double> u(static_cast<std::size_t>(cover_size + 1), 0.0);
  std::vector<double> m(static_cast<std::size_t>(cover_size + 1), 0.0);

  double usum = 0.0;
  double msum = 0.0;
  for (int c = 0; c != cover_size; ++c) {
    const int k = cover[static_cast<std::size_t>(c)];
    u[static_cast<std::size_t>(c)] = usum;
    m[static_cast<std::size_t>(c)] = msum;
    a[static_cast<std::size_t>(c)] = row.vals[static_cast<std::size_t>(k)];
    const double ub = row.upper[static_cast<std::size_t>(k)];
    usum += ub;
    msum += ub * a[static_cast<std::size_t>(c)];
  }
  u[static_cast<std::size_t>(cover_size)] = usum;
  m[static_cast<std::size_t>(cover_size)] = msum;

  int lpos = -1;
  int best_l_cplus_end = -1;
  double best_l_val = 0.0;
  bool best_l_at_upper = true;

  for (int i = 0; i != cover_size; ++i) {
    const int k = cover[static_cast<std::size_t>(i)];
    const double ub = row.upper[static_cast<std::size_t>(k)];

    const bool at_upper =
        row.solval[static_cast<std::size_t>(k)] >= ub - feastol;
    if (at_upper && !best_l_at_upper) continue;

    const double mju = ub * row.vals[static_cast<std::size_t>(k)];
    const double mu = mju - double(state.lambda);

    if (mu <= 10.0 * feastol) continue;
    if (std::abs(row.vals[static_cast<std::size_t>(k)]) <
        1000.0 * feastol) {
      continue;
    }

    const double mudival = mu / row.vals[static_cast<std::size_t>(k)];
    if (xtab_is_integral_value(mudival, feastol)) continue;
    const double eta = std::ceil(mudival);

    const double ul_minus_eta_plus_one = ub - eta + 1.0;
    const double cplus_threshold =
        ul_minus_eta_plus_one * row.vals[static_cast<std::size_t>(k)];

    const int cplus_end = static_cast<int>(
        std::upper_bound(cover.begin(), cover.end(), cplus_threshold,
                         [&](double threshold, int col) {
                           return threshold >
                                  row.vals[static_cast<std::size_t>(col)];
                         }) -
        cover.begin());

    double mcplus = m[static_cast<std::size_t>(cplus_end)];
    if (i < cplus_end) mcplus -= mju;

    const double jl_val =
        mcplus + eta * row.vals[static_cast<std::size_t>(k)];

    if (jl_val > best_l_val || (!at_upper && best_l_at_upper)) {
      lpos = i;
      best_l_cplus_end = cplus_end;
      best_l_val = jl_val;
      best_l_at_upper = at_upper;
    }
  }

  if (lpos == -1) return false;

  const int l = cover[static_cast<std::size_t>(lpos)];
  const double al = row.vals[static_cast<std::size_t>(l)];
  const double upper_l = row.upper[static_cast<std::size_t>(l)];
  const double mlu = upper_l * al;
  const double mu = mlu - double(state.lambda);

  a.resize(static_cast<std::size_t>(best_l_cplus_end));
  cover.resize(static_cast<std::size_t>(best_l_cplus_end));
  u.resize(static_cast<std::size_t>(best_l_cplus_end + 1));
  m.resize(static_cast<std::size_t>(best_l_cplus_end + 1));

  if (lpos < best_l_cplus_end) {
    a.erase(a.begin() + lpos);
    cover.erase(cover.begin() + lpos);
    u.erase(u.begin() + lpos + 1);
    m.erase(m.begin() + lpos + 1);
    for (int i = lpos + 1; i < best_l_cplus_end; ++i) {
      u[static_cast<std::size_t>(i)] -= upper_l;
      m[static_cast<std::size_t>(i)] -= mlu;
    }
  }

  const int cplus_size = static_cast<int>(a.size());
  const double mudival = mu / al;
  const double eta = std::ceil(mudival);
  double r = mu - std::floor(mudival) * al;
  if (r < 0.0) r = 0.0;

  const double ul_minus_eta_plus_one = upper_l - eta + 1.0;
  const double cplus_threshold = ul_minus_eta_plus_one * al;
  const int64_t kmin = static_cast<int64_t>(std::floor(eta - upper_l - 0.5));

  auto phi_l = [&](double value) {
    int64_t k = std::min(static_cast<int64_t>(value / al), int64_t{-1});

    for (; k >= kmin; --k) {
      if (value >= static_cast<double>(k) * al + r) {
        return value - static_cast<double>(k + 1) * r;
      }

      if (value >= static_cast<double>(k) * al) {
        return static_cast<double>(k) * (al - r);
      }
    }

    return static_cast<double>(kmin) * (al - r);
  };

  const int64_t kmax = static_cast<int64_t>(std::floor(upper_l - eta + 0.5));

  auto gamma_l = [&](double z) {
    for (int i = 0; i < cplus_size; ++i) {
      const int col = cover[static_cast<std::size_t>(i)];
      const int upper_i = static_cast<int>(
          row.upper[static_cast<std::size_t>(col)]);

      for (int h = 0; h <= upper_i; ++h) {
        const double mih = m[static_cast<std::size_t>(i)] +
                           static_cast<double>(h) *
                               a[static_cast<std::size_t>(i)];
        const double uih =
            u[static_cast<std::size_t>(i)] + static_cast<double>(h);
        const double mih_plus_delta_i =
            mih + a[static_cast<std::size_t>(i)] - cplus_threshold;
        if (z <= mih_plus_delta_i) {
          return uih * ul_minus_eta_plus_one * (al - r);
        }

        int64_t k =
            static_cast<int64_t>((z - mih_plus_delta_i) / al) - 1;
        for (; k <= kmax; ++k) {
          if (z <= mih_plus_delta_i + static_cast<double>(k) * al + r) {
            return (uih * ul_minus_eta_plus_one + static_cast<double>(k)) *
                   (al - r);
          }

          if (z <= mih_plus_delta_i + static_cast<double>(k + 1) * al) {
            return uih * ul_minus_eta_plus_one * (al - r) + z - mih -
                   a[static_cast<std::size_t>(i)] + cplus_threshold -
                   static_cast<double>(k + 1) * r;
          }
        }
      }
    }

    int64_t p = static_cast<int64_t>(
                    (z - m[static_cast<std::size_t>(cplus_size)]) / al) -
                1;
    for (;; ++p) {
      if (z <= m[static_cast<std::size_t>(cplus_size)] +
                   static_cast<double>(p) * al + r) {
        return (u[static_cast<std::size_t>(cplus_size)] *
                    ul_minus_eta_plus_one +
                static_cast<double>(p)) *
               (al - r);
      }

      if (z <= m[static_cast<std::size_t>(cplus_size)] +
                   static_cast<double>(p + 1) * al) {
        return u[static_cast<std::size_t>(cplus_size)] *
                   ul_minus_eta_plus_one * (al - r) +
               z - m[static_cast<std::size_t>(cplus_size)] -
               static_cast<double>(p + 1) * r;
      }
    }
  };

  row.rhs = (upper_l - eta) * r - double(state.lambda);
  row.integral_support = true;
  row.integral_coefficients = false;
  for (int k = 0; k < static_cast<int>(row.inds.size()); ++k) {
    const std::size_t pos = static_cast<std::size_t>(k);
    if (row.vals[pos] == 0.0) continue;
    if (row.is_integral[pos] == 0) {
      if (row.vals[pos] < 0.0) {
        row.integral_support = false;
      } else {
        row.vals[pos] = 0.0;
      }
      continue;
    }

    if (cover_flag[pos] != 0) {
      row.vals[pos] = -phi_l(-row.vals[pos]);
      row.rhs += row.vals[pos] * row.upper[pos];
    } else {
      row.vals[pos] = gamma_l(row.vals[pos]);
    }
  }

  return true;
}

void xtab_erase_positions(XTabRow& row, std::vector<unsigned char>& erase) {
  int write = 0;
  const int len = static_cast<int>(row.inds.size());
  for (int read = 0; read < len; ++read) {
    if (erase[static_cast<std::size_t>(read)] != 0) {
      continue;
    }
    if (write != read) {
      row.inds[static_cast<std::size_t>(write)] =
          row.inds[static_cast<std::size_t>(read)];
      row.vals[static_cast<std::size_t>(write)] =
          row.vals[static_cast<std::size_t>(read)];
      row.upper[static_cast<std::size_t>(write)] =
          row.upper[static_cast<std::size_t>(read)];
      row.solval[static_cast<std::size_t>(write)] =
          row.solval[static_cast<std::size_t>(read)];
      row.bound_type[static_cast<std::size_t>(write)] =
          row.bound_type[static_cast<std::size_t>(read)];
      row.varbound_expr[static_cast<std::size_t>(write)] =
          row.varbound_expr[static_cast<std::size_t>(read)];
      row.complemented[static_cast<std::size_t>(write)] =
          row.complemented[static_cast<std::size_t>(read)];
      row.is_integral[static_cast<std::size_t>(write)] =
          row.is_integral[static_cast<std::size_t>(read)];
    }
    ++write;
  }
  row.inds.resize(static_cast<std::size_t>(write));
  row.vals.resize(static_cast<std::size_t>(write));
  row.upper.resize(static_cast<std::size_t>(write));
  row.solval.resize(static_cast<std::size_t>(write));
  row.bound_type.resize(static_cast<std::size_t>(write));
  row.varbound_expr.resize(static_cast<std::size_t>(write));
  row.complemented.resize(static_cast<std::size_t>(write));
  row.is_integral.resize(static_cast<std::size_t>(write));
}

bool xtab_logical_row_bounds(const SimplexResult& simplex,
                             int row,
                             double& lower,
                             double& upper) {
  const StandardFormLP& sf = simplex.form;
  const int m = static_cast<int>(sf.A_row.rows());
  const int n_std = static_cast<int>(sf.A.cols());
  if (row < 0 || row >= m || row >= sf.b.size()) {
    return false;
  }
  lower = -std::numeric_limits<double>::infinity();
  upper = std::numeric_limits<double>::infinity();
  const double side = xtab_source_row_side(sf, row);
  if (!std::isfinite(side)) {
    return false;
  }

  auto compute_activity_domain = [&]() {
    double min_activity = 0.0;
    double max_activity = 0.0;
    bool min_finite = true;
    bool max_finite = true;
    xtab_visit_source_row_terms(sf, row, [&](int j, double a) {
      if (std::abs(a) <= 1e-12) return;
      const double lb =
          j < sf.lb_shift.size() ? sf.lb_shift[j] : -std::numeric_limits<double>::infinity();
      const double ub =
          j < sf.var_ub.size()
              ? original_space_value(simplex, j, sf.var_ub[j])
              : std::numeric_limits<double>::infinity();
      if (a >= 0.0) {
        if (std::isfinite(lb) && min_finite) {
          min_activity += a * lb;
        } else {
          min_finite = false;
        }
        if (std::isfinite(ub) && max_finite) {
          max_activity += a * ub;
        } else {
          max_finite = false;
        }
      } else {
        if (std::isfinite(ub) && min_finite) {
          min_activity += a * ub;
        } else {
          min_finite = false;
        }
        if (std::isfinite(lb) && max_finite) {
          max_activity += a * lb;
        } else {
          max_finite = false;
        }
      }
    });
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
      const double slack_scale = xtab_col_scale_or_one(sf, slack_col);
      const double width = sf.var_ub[slack_col] * slack_scale;
      if (!std::isfinite(width) || width < -1e-9) {
        return false;
      }
      lower = upper - std::max(0.0, width);
    } else {
      lower = compute_activity_domain().first;
    }
    return std::isfinite(upper);
  }

  if (surplus_col >= 0) {
    lower = side;
    upper = compute_activity_domain().second;
    return std::isfinite(lower);
  }

  lower = side;
  upper = side;
  return std::isfinite(lower);
}

bool xtab_logical_row_is_integer_like(
    const SimplexResult& simplex,
    int row,
    const std::vector<char>* implied_integer_cols) {
  const StandardFormLP& sf = simplex.form;
  if (row < 0 || row >= sf.A_row.rows()) {
    return false;
  }
  double lower = 0.0;
  double upper = 0.0;
  if (!xtab_logical_row_bounds(simplex, row, lower, upper)) {
    return false;
  }
  if (std::isfinite(lower) && !is_integral(lower, 1e-8)) {
    return false;
  }
  if (std::isfinite(upper) && !is_integral(upper, 1e-8)) {
    return false;
  }

  bool has_structural_coeff = false;
  bool row_is_integral = true;
  xtab_visit_source_row_terms(sf, row, [&](int j, double unscaled_coeff) {
    if (!transformed_col_is_integer_like(simplex, j, implied_integer_cols)) {
      row_is_integral = false;
      return;
    }
    if (!std::isfinite(unscaled_coeff) || !is_integral(unscaled_coeff, 1e-8)) {
      row_is_integral = false;
      return;
    }
    has_structural_coeff = true;
  });
  return row_is_integral && has_structural_coeff;
}

XTabSourceContext xtab_build_source_context(
    const SimplexResult& simplex,
    const std::vector<char>* implied_integer_cols) {
  XTabSourceContext ctx;
  const StandardFormLP& sf = simplex.form;
  const int n_orig = sf.n_original;
  const int m = static_cast<int>(sf.A_row.rows());
  if (n_orig <= 0 || m <= 0 || simplex.x_std.size() < n_orig ||
      sf.var_ub.size() < n_orig || sf.lb_shift.size() < n_orig ||
      sf.A_row.cols() < n_orig) {
    return ctx;
  }

  ctx.n_original = n_orig;
  ctx.n_rows = m;
  ctx.dim = n_orig + m;
  ctx.lower = Eigen::VectorXd::Constant(
      ctx.dim, -std::numeric_limits<double>::infinity());
  ctx.upper = Eigen::VectorXd::Constant(
      ctx.dim, std::numeric_limits<double>::infinity());
  ctx.solution = Eigen::VectorXd::Zero(ctx.dim);
  ctx.integral.assign(static_cast<std::size_t>(ctx.dim), 0);

  for (int j = 0; j < n_orig; ++j) {
    ctx.lower[j] = sf.lb_shift[j];
    ctx.upper[j] = original_space_value(simplex, j, sf.var_ub[j]);
    ctx.solution[j] = original_space_value(simplex, j, simplex.x_std[j]);
    if (!std::isfinite(ctx.solution[j])) {
      return XTabSourceContext{};
    }
    ctx.integral[static_cast<std::size_t>(j)] =
        transformed_col_is_integer_like(simplex, j, implied_integer_cols) ? 1
                                                                         : 0;
  }

  for (int row = 0; row < m; ++row) {
    const int col = n_orig + row;
    double lower = 0.0;
    double upper = 0.0;
    if (!xtab_logical_row_bounds(simplex, row, lower, upper)) {
      return XTabSourceContext{};
    }
    ctx.lower[col] = lower;
    ctx.upper[col] = upper;
    double activity = 0.0;
    xtab_visit_source_row_terms(sf, row, [&](int j, double value) {
      activity += value * ctx.solution[j];
    });
    if (!std::isfinite(activity)) {
      return XTabSourceContext{};
    }
    ctx.solution[col] = activity;
    ctx.integral[static_cast<std::size_t>(col)] =
        xtab_logical_row_is_integer_like(simplex, row, implied_integer_cols) ? 1
                                                                            : 0;
  }

  ctx.valid = true;
  return ctx;
}

double xtab_unscaled_source_value(const SimplexResult& simplex,
                                  const XTabSourceContext& source_context,
                                  int source_col) {
  (void)simplex;
  if (source_col < 0 || source_col >= source_context.dim) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return source_context.solution[source_col];
}

bool xtab_basic_source_info(const SimplexResult& simplex,
                            const XTabSourceContext& source_context,
                            int basic_col,
                            int& source_col,
                            double& btran_scale) {
  source_col = -1;
  btran_scale = 1.0;
  if (!source_context.valid || basic_col < 0) {
    return false;
  }
  if (basic_col < source_context.n_original) {
    source_col = basic_col;
    btran_scale = xtab_col_scale_or_one(simplex.form, basic_col);
    return true;
  }
  const int row = standard_form_aux_col_row(simplex.form, basic_col);
  if (row < 0 || row >= source_context.n_rows) {
    return false;
  }
  const double aux_coeff = simplex.form.A_row.coeff(row, basic_col);
  if (!std::isfinite(aux_coeff) || std::abs(aux_coeff) <= 1e-12) {
    return false;
  }
  source_col = source_context.n_original + row;
  // Native BTRAN is performed on the scaled standard-form basis.  HiGHS'
  // transformed-row sources are expressed in the unscaled row-activity model
  //     a_r x - row_activity_r = 0.
  // For a native slack/surplus basic column with scaled row coefficient q,
  // the derivative of the logical row activity with respect to the scaled
  // auxiliary variable is -q / row_scale(row).  The per-row multiplier is
  // mapped back to HiGHS row space later by multiplying each BTRAN entry by
  // that row's row_scale.
  btran_scale = -aux_coeff / xtab_row_scale_or_one(simplex.form, row);
  return true;
}

bool xtab_is_binary_trigger_col(const SimplexResult& simplex,
                                int col,
                                const std::vector<char>* implied_integer_cols) {
  if (col < 0 || col >= simplex.form.n_original ||
      col >= static_cast<int>(simplex.form.var_ub.size()) ||
      col >= static_cast<int>(simplex.form.lb_shift.size()) ||
      col >= static_cast<int>(simplex.form.original_types.size())) {
    return false;
  }
  if (!transformed_col_is_integer_like(simplex, col, implied_integer_cols)) {
    return false;
  }
  const VarType t = simplex.form.original_types[static_cast<std::size_t>(col)];
  const bool declared_binary_like =
      t == VarType::Binary || t == VarType::Integer;
  if (!declared_binary_like) {
    return false;
  }
  const double lb = simplex.form.lb_shift[col];
  const double ub_std = simplex.form.var_ub[col];
  if (!std::isfinite(lb) || !std::isfinite(ub_std)) {
    return false;
  }
  const double ub = original_space_value(simplex, col, ub_std);
  return lb >= -1e-9 && ub <= 1.0 + 1e-9;
}

bool xtab_can_use_varbound_target(const SimplexResult& simplex,
                                  int col,
                                  const std::vector<char>* implied_integer_cols,
                                  bool* integer_target_out = nullptr) {
  if (integer_target_out != nullptr) *integer_target_out = false;
  if (col < 0 || col >= simplex.form.n_original ||
      col >= static_cast<int>(simplex.form.var_ub.size()) ||
      col >= static_cast<int>(simplex.form.lb_shift.size())) {
    return false;
  }
  const double ub_std = simplex.form.var_ub[col];
  const double lb = simplex.form.lb_shift[col];
  if (!std::isfinite(lb) || !std::isfinite(ub_std)) {
    return false;
  }
  const double ub = original_space_value(simplex, col, ub_std);
  if (!std::isfinite(ub)) {
    return false;
  }

  const bool integer_like =
      transformed_col_is_integer_like(simplex, col, implied_integer_cols);
  if (!integer_like) {
    return true;
  }

  // Match HighsTransformedLp::transform(): non-binary integral columns may use
  // VUB/VLB rows when both simple bound slacks are positive.  The selected
  // variable-bound distance is checked per candidate below.
  if (col >= simplex.x_std.size()) {
    return false;
  }
  const double value = original_space_value(simplex, col, simplex.x_std[col]);
  if (!std::isfinite(value)) {
    return false;
  }
  const double simple_lb_dist = std::max(0.0, value - lb);
  const double simple_ub_dist = std::max(0.0, ub - value);
  if (ub - lb <= 1.5 + 1e-9 || simple_lb_dist <= 1e-8 ||
      simple_ub_dist <= 1e-8) {
    return false;
  }
  if (integer_target_out != nullptr) *integer_target_out = true;
  return true;
}

void xtab_update_best_varbound(XTabVarBoundExpr& best,
                               const XTabVarBoundExpr& cand,
                               bool is_lower_bound) {
  if (!cand.valid) return;
  if (!best.valid) {
    best = cand;
    return;
  }
  if (cand.scaled_dist < best.scaled_dist - 1e-12) {
    best = cand;
    return;
  }
  if (std::abs(cand.scaled_dist - best.scaled_dist) > 1e-12) {
    return;
  }

  // Match HighsImplications::getBestVub/getBestVlb: after the LP-distance
  // filter, prefer the globally stronger variable bound.  VUBs are stronger
  // when their minimum possible upper value is smaller; VLBs are stronger when
  // their maximum possible lower value is larger.  The lift tie-break is kept
  // only after this HiGHS-conformant ordering is exhausted.
  if (!is_lower_bound &&
      cand.min_value < best.min_value - 1e-12) {
    best = cand;
    return;
  }
  if (is_lower_bound &&
      cand.max_value > best.max_value + 1e-12) {
    best = cand;
    return;
  }
  if (((!is_lower_bound &&
        std::abs(cand.min_value - best.min_value) <= 1e-12) ||
       (is_lower_bound &&
        std::abs(cand.max_value - best.max_value) <= 1e-12)) &&
      cand.lift > best.lift + 1e-12) {
    best = cand;
  }
}

XTabTransformContext xtab_build_transform_context(
    const SimplexResult& simplex,
    const std::vector<char>* implied_integer_cols,
    const BinaryImplicationGraph* implication_graph,
    const VariableBoundTable* variable_bound_table) {
  XTabTransformContext ctx;
  const int n_orig = simplex.form.n_original;
  const bool have_graph =
      implication_graph != nullptr && !implication_graph->empty();
  const bool have_varbounds =
      variable_bound_table != nullptr && !variable_bound_table->empty();
  if (n_orig <= 0 || (!have_graph && !have_varbounds) ||
      simplex.x_std.size() < n_orig ||
      simplex.form.var_ub.size() < static_cast<Eigen::Index>(n_orig) ||
      simplex.form.lb_shift.size() < static_cast<Eigen::Index>(n_orig)) {
    return ctx;
  }
  ctx.best_vlb.assign(static_cast<std::size_t>(n_orig), XTabVarBoundExpr{});
  ctx.best_vub.assign(static_cast<std::size_t>(n_orig), XTabVarBoundExpr{});
  const int trace_vb_col = xtab_varbound_trace_col();

  auto add_varbound_expr = [&](int target,
                               int trigger,
                               double coef_orig,
                               double constant_orig,
                               bool is_lower_bound,
                               const char* source) {
    const bool trace = target == trace_vb_col;
    auto trace_reject = [&](const char* reason) {
      if (!trace) return;
      fmt::print(stderr,
                 "[B&C-XTAB-VB-CAND] target={} trigger={} sense={} "
                 "source={} reject={} coef={:.17g} constant={:.17g}\n",
                 target, trigger, is_lower_bound ? "vlb" : "vub",
                 source != nullptr ? source : "unknown", reason, coef_orig,
                 constant_orig);
    };
    bool integer_target = false;
    if (trigger < 0 || trigger >= n_orig || target < 0 ||
        target >= n_orig || target == trigger ||
        !std::isfinite(coef_orig) || !std::isfinite(constant_orig)) {
      trace_reject("invalid_input");
      return;
    }
    if (!xtab_is_binary_trigger_col(simplex, trigger, implied_integer_cols)) {
      trace_reject("trigger_not_binary");
      return;
    }
    if (!xtab_can_use_varbound_target(simplex, target,
                                      implied_integer_cols, &integer_target)) {
      trace_reject("target_unusable");
      return;
    }
    const double trigger_ub = simplex.form.var_ub[trigger];
    if (!std::isfinite(trigger_ub) || trigger_ub <= 1e-12) {
      trace_reject("trigger_bad_ub");
      return;
    }
    const double trigger_lb = simplex.form.lb_shift[trigger];
    const double trigger_ub_orig =
        original_space_value(simplex, trigger, trigger_ub);
    const double trigger_val =
        std::clamp(original_space_value(simplex, trigger,
                                        simplex.x_std[trigger]),
                   trigger_lb, trigger_ub_orig);
    if (!std::isfinite(trigger_lb) || !std::isfinite(trigger_ub_orig) ||
        !std::isfinite(trigger_val)) {
      trace_reject("trigger_bad_value");
      return;
    }

    const double target_ub_std = simplex.form.var_ub[target];
    const double target_lb = simplex.form.lb_shift[target];
    if (!std::isfinite(target_lb) || !std::isfinite(target_ub_std)) {
      trace_reject("target_bad_bounds");
      return;
    }
    const double target_ub =
        original_space_value(simplex, target, target_ub_std);
    const double target_val =
        original_space_value(simplex, target, simplex.x_std[target]);
    if (!std::isfinite(target_ub) || !std::isfinite(target_val)) {
      trace_reject("target_bad_value");
      return;
    }
    const double target_range = std::max(1.0, target_ub - target_lb);
    if (!std::isfinite(target_range) || target_range <= 0.0) {
      trace_reject("target_bad_range");
      return;
    }

    const double expr_at_lp = constant_orig + coef_orig * trigger_val;
    double dist = 0.0;
    double y_dist = 0.0;
    if (is_lower_bound) {
      dist = std::max(0.0, target_val - expr_at_lp);
      y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_val - trigger_lb
                                       : trigger_ub_orig - trigger_val);
    } else {
      dist = std::max(0.0, expr_at_lp - target_val);
      y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_ub_orig - trigger_val
                                       : trigger_val - trigger_lb);
    }
    y_dist = std::max(1e-9, y_dist);
    const double norm2 = 1.0 + coef_orig * coef_orig;
    if (dist * dist > y_dist * y_dist * norm2 + 1e-12) {
      trace_reject("bad_vbd_distance");
      return;
    }
    if (integer_target && dist > 1e-7) {
      trace_reject("integer_target_distance");
      return;
    }

    XTabVarBoundExpr cand;
    cand.valid = true;
    cand.trigger_col = trigger;
    cand.constant = constant_orig;
    cand.coef = coef_orig;
    cand.dist = dist;
    cand.scaled_dist = dist / target_range;
    cand.min_value =
        constant_orig + std::min(coef_orig * trigger_lb,
                                 coef_orig * trigger_ub_orig);
    cand.max_value =
        constant_orig + std::max(coef_orig * trigger_lb,
                                 coef_orig * trigger_ub_orig);
    cand.lift = std::abs(coef_orig) * (trigger_ub_orig - trigger_lb);
    cand.integer_target = integer_target;
    if (trace) {
      fmt::print(stderr,
                 "[B&C-XTAB-VB-CAND] target={} trigger={} sense={} "
                 "source={} accept coef={:.17g} constant={:.17g} "
                 "dist={:.17g} scaled={:.17g} min={:.17g} max={:.17g} "
                 "lift={:.17g} integerTarget={} targetVal={:.17g} "
                 "triggerVal={:.17g}\n",
                 target, trigger, is_lower_bound ? "vlb" : "vub",
                 source != nullptr ? source : "unknown", coef_orig,
                 constant_orig, cand.dist, cand.scaled_dist, cand.min_value,
                 cand.max_value, cand.lift, cand.integer_target ? 1 : 0,
                 target_val, trigger_val);
    }
    if (is_lower_bound) {
      XTabVarBoundExpr& best = ctx.best_vlb[static_cast<std::size_t>(target)];
      xtab_update_best_varbound(
          best, cand, true);
      if (trace) {
        fmt::print(stderr,
                   "[B&C-XTAB-VB-BEST] target={} sense=vlb trigger={} "
                   "coef={:.17g} constant={:.17g} dist={:.17g} "
                   "scaled={:.17g} min={:.17g} max={:.17g}\n",
                   target, best.trigger_col, best.coef, best.constant,
                   best.dist, best.scaled_dist, best.min_value,
                   best.max_value);
      }
    } else {
      XTabVarBoundExpr& best = ctx.best_vub[static_cast<std::size_t>(target)];
      xtab_update_best_varbound(
          best, cand, false);
      if (trace) {
        fmt::print(stderr,
                   "[B&C-XTAB-VB-BEST] target={} sense=vub trigger={} "
                   "coef={:.17g} constant={:.17g} dist={:.17g} "
                   "scaled={:.17g} min={:.17g} max={:.17g}\n",
                   target, best.trigger_col, best.coef, best.constant,
                   best.dist, best.scaled_dist, best.min_value,
                   best.max_value);
      }
    }
  };

  if (have_varbounds) {
    const int table_cols =
        std::min(n_orig, variable_bound_table->num_vars());
    for (int target = 0; target < table_cols; ++target) {
      for (const auto& entry : variable_bound_table->vlbs(target)) {
        add_varbound_expr(target, entry.trigger_col, entry.bound.coef,
                          entry.bound.constant, true, "table");
      }
      for (const auto& entry : variable_bound_table->vubs(target)) {
        add_varbound_expr(target, entry.trigger_col, entry.bound.coef,
                          entry.bound.constant, false, "table");
      }
    }
  }

  if (!have_graph) {
    for (const auto& vb : ctx.best_vlb) {
      if (vb.valid) {
        ++ctx.num_vlb;
        if (vb.integer_target) ++ctx.num_integral_vlb;
      }
    }
    for (const auto& vb : ctx.best_vub) {
      if (vb.valid) {
        ++ctx.num_vub;
        if (vb.integer_target) ++ctx.num_integral_vub;
      }
    }
    return ctx;
  }

  for (int trigger = 0; trigger < n_orig; ++trigger) {
        if (!xtab_is_binary_trigger_col(simplex, trigger, implied_integer_cols)) {
      continue;
    }
    const double trigger_ub = simplex.form.var_ub[trigger];
    if (!std::isfinite(trigger_ub) || trigger_ub <= 1e-12) {
      continue;
    }
    const double trigger_lb = simplex.form.lb_shift[trigger];
    const double trigger_ub_orig =
        original_space_value(simplex, trigger, trigger_ub);
    const double trigger_val =
        std::clamp(original_space_value(simplex, trigger,
                                        simplex.x_std[trigger]),
                   trigger_lb, trigger_ub_orig);
    if (!std::isfinite(trigger_lb) || !std::isfinite(trigger_ub_orig) ||
        !std::isfinite(trigger_val)) {
      continue;
    }

    for (bool trigger_one : {false, true}) {
      const auto range = implication_graph->implications(trigger, trigger_one);
      for (const auto* p = range.first; p != range.second; ++p) {
        if (p == nullptr || p->var_idx < 0 || p->var_idx >= n_orig ||
            p->var_idx == trigger || !std::isfinite(p->value) ||
            !xtab_can_use_varbound_target(simplex, p->var_idx,
                                          implied_integer_cols)) {
          continue;
        }

        const int target = p->var_idx;
        const double target_lb = simplex.form.lb_shift[target];
        const double target_ub_std = simplex.form.var_ub[target];
        if (!std::isfinite(target_lb) || !std::isfinite(target_ub_std)) {
          continue;
        }
        const double target_ub =
            original_space_value(simplex, target, target_ub_std);
        if (!std::isfinite(target_ub)) {
          continue;
        }
        const double target_range =
            std::max(1.0, target_ub - target_lb);
        const double target_val =
            original_space_value(simplex, target, simplex.x_std[target]);
        if (!std::isfinite(target_range) || target_range <= 0.0 ||
            !std::isfinite(target_val)) {
          continue;
        }

        // x_target_orig >=/<= constant_orig + coef_orig * y_trigger_orig.
        double constant_orig = 0.0;
        double coef_orig = 0.0;
        double improvement = 0.0;
        bool integer_target = false;
        if (!xtab_can_use_varbound_target(simplex, target,
                                          implied_integer_cols,
                                          &integer_target)) {
          continue;
        }

        if (p->is_lb) {
          if (p->value <= target_lb + 1e-9) continue;
          improvement = p->value - target_lb;
          if (trigger_one) {
            constant_orig = target_lb;
            coef_orig = improvement;
          } else {
            constant_orig = p->value;
            coef_orig = -improvement;
          }
        } else {
          if (p->value >= target_ub - 1e-9) continue;
          improvement = target_ub - p->value;
          if (trigger_one) {
            constant_orig = target_ub;
            coef_orig = -improvement;
          } else {
            constant_orig = p->value;
            coef_orig = improvement;
          }
        }
        if (!(improvement > 1e-12)) continue;

        const double expr_at_lp = constant_orig + coef_orig * trigger_val;
        double dist = 0.0;
        double y_dist = 0.0;
        if (p->is_lb) {
          dist = std::max(0.0, target_val - expr_at_lp);
          y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_val - trigger_lb
                                           : trigger_ub_orig - trigger_val);
        } else {
          dist = std::max(0.0, expr_at_lp - target_val);
          y_dist = 1e-9 + (coef_orig > 0.0 ? trigger_ub_orig - trigger_val
                                           : trigger_val - trigger_lb);
        }
        y_dist = std::max(1e-9, y_dist);
        const double norm2 = 1.0 + coef_orig * coef_orig;
        if (dist * dist > y_dist * y_dist * norm2 + 1e-12) {
          continue;
        }
        if (integer_target && dist > 1e-7) {
          continue;
        }

        XTabVarBoundExpr cand;
        cand.valid = true;
        cand.trigger_col = trigger;
        cand.constant = constant_orig;
        cand.coef = coef_orig;
        cand.dist = dist;
        cand.scaled_dist = dist / target_range;
        cand.min_value =
            constant_orig + std::min(coef_orig * trigger_lb,
                                     coef_orig * trigger_ub_orig);
        cand.max_value =
            constant_orig + std::max(coef_orig * trigger_lb,
                                     coef_orig * trigger_ub_orig);
        cand.lift = std::abs(coef_orig) * (trigger_ub_orig - trigger_lb);
        cand.integer_target = integer_target;
        if (p->is_lb) {
          xtab_update_best_varbound(
              ctx.best_vlb[static_cast<std::size_t>(target)], cand, true);
        } else {
          xtab_update_best_varbound(
              ctx.best_vub[static_cast<std::size_t>(target)], cand, false);
        }
        if (target == trace_vb_col) {
          const XTabVarBoundExpr& best =
              p->is_lb ? ctx.best_vlb[static_cast<std::size_t>(target)]
                       : ctx.best_vub[static_cast<std::size_t>(target)];
          fmt::print(stderr,
                     "[B&C-XTAB-VB-BEST] target={} sense={} trigger={} "
                     "source=graph coef={:.17g} constant={:.17g} "
                     "dist={:.17g} scaled={:.17g} min={:.17g} max={:.17g}\n",
                     target, p->is_lb ? "vlb" : "vub", best.trigger_col,
                     best.coef, best.constant, best.dist, best.scaled_dist,
                     best.min_value, best.max_value);
        }
      }
    }
  }
  if (trace_vb_col >= 0 && trace_vb_col < n_orig) {
    const XTabVarBoundExpr& vlb =
        ctx.best_vlb[static_cast<std::size_t>(trace_vb_col)];
    const XTabVarBoundExpr& vub =
        ctx.best_vub[static_cast<std::size_t>(trace_vb_col)];
    fmt::print(stderr,
               "[B&C-XTAB-VB-FINAL] target={} vlbValid={} vlbTrigger={} "
               "vlbCoef={:.17g} vlbConst={:.17g} vlbDist={:.17g} "
               "vubValid={} vubTrigger={} vubCoef={:.17g} "
               "vubConst={:.17g} vubDist={:.17g}\n",
               trace_vb_col, vlb.valid ? 1 : 0, vlb.trigger_col, vlb.coef,
               vlb.constant, vlb.dist, vub.valid ? 1 : 0, vub.trigger_col,
               vub.coef, vub.constant, vub.dist);
  }
  for (const auto& vb : ctx.best_vlb) {
    if (vb.valid) {
      ++ctx.num_vlb;
      if (vb.integer_target) ++ctx.num_integral_vlb;
    }
  }
  for (const auto& vb : ctx.best_vub) {
    if (vb.valid) {
      ++ctx.num_vub;
      if (vb.integer_target) ++ctx.num_integral_vub;
    }
  }
  return ctx;
}


}  // anonymous namespace
}  // namespace mipsolvers::engine::detail
